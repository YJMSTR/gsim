// GSIM_SCHEDULE_SEED2 exact replay: record outcomes at every order-consuming graph
// point (topoSort, every resort, and mergeWhenNodes' whenMap) so replay can force each
// point verbatim instead of re-deriving it with a policy comparator (the first text-format
// seed replay re-derived downstream order from a policy and paid a +32% shape tax).
//
// Binary format (little-endian), file extension convention *.gsimseed2:
//   char  magic[4]   = "GS2\0"
//   u32   version    = 1 (legacy, all existing champion seeds), 2 (codec-framed)
//                     or 3 (codec-framed + per-point canon algorithm tag)
//   u64   input_hash                 (graph::canonInputHash() at topoSort entry)
//   u32   generator_len, bytes       (generator tag)
//   u32   point_count
//   u32   name_count
//   u32   whenmap_group_count
//   -- version 1 ends here; the payload follows stored uncompressed --
//   -- version 2/3 codec header (quick-parse prefix above stays at v1 offsets) --
//   u32   codec                      (0 = none, 1 = zlib)
//   u32   codec_level                (informational: zlib deflate level)
//   u64   raw_payload_bytes          (uncompressed payload size)
//   u32   payload_crc32              (CRC-32 of the uncompressed payload bytes)
//   u64   comp_payload_bytes         (on-disk payload size; must match file tail)
//   [payload = string table + points + whenmap, byte-identical to the v1 body,
//    stored as one codec frame]
//   [string table] name_count x { u32 len, bytes }      (SuperNode keys, see keyOf)
//   [points] point_count x {
//     u32 tag_len, bytes
//     [version 3 only] u8 canon_algo  (1 = serial FNV-1a v1, 2 = segmented v2)
//     u64 canon_hash                 (order-free content hash at this point)
//     u32 node_count
//     u32 ranks[node_count]          (string-table ids, in recorded sortedSuper order)
//   }
//   [whenmap] whenmap_group_count x {
//     u32 cond_key  (string-table id of the mergeCond SuperNode key)
//     u32 source_count
//     u32 sources[source_count]      (string-table ids; index 0 is the recorded target)
//   }
//
// The canon hash is VERIFICATION-ONLY: mtSeed2RecordPoint stores it next to the
// rank permutation, mtSeed2ApplyPoint compares it (Assert abort on mismatch) and
// forces sortedSuper strictly from the recorded ranks - the hash never reaches
// the emitted model. GSIM_SEED2_CANON=v2 (default off) opts new writes into the
// parallel segmented mix (graph::canonMixV2): such seeds are written as version 3
// with a per-point canon_algo tag, replay computes exactly the tagged algorithm
// per point, and untagged v1/v2 seeds keep computing the frozen v1 mix - so all
// existing champion seeds replay byte-identically and mixed v1/v2 point streams
// inside one version-3 file also replay correctly.
//
// v2 payloads are produced and consumed through bounded chunk buffers (Seed2Output /
// Seed2Input): the writer streams its in-memory state straight into z_stream as it
// serializes and the reader inflates on demand as the parsers consume, so neither side
// ever stages a second full-size copy. Compressing the payload changes nothing about
// intern ids, point tags, rank order or whenMap order; replay semantics, canon checks
// and generated models are byte-identical to a v1 write+replay of the same run.
// The reader auto-detects v1/v2 by the version field; v1 champion seeds keep replaying.
//
// A SuperNode key is the ';'-joined member-name list (same construction as
// graph::canonInputHash's keyOf) - member names are unique, so the list keys the node.
//
// S1 scope: write path (passive recording; never changes generation behavior).
// Replay lands in S2 (points) and S3 (whenMap).

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <memory>
#include <string>
#include <unordered_map>
#include <deque>
#include <vector>
#include "common.h"
#include "graph.h"
#include "phaseTimer.h"
#include <zlib.h>

#define SEED2_MAGIC "GS2\0"
#define SEED2_VERSION 1u   // legacy layout: payload stored uncompressed after the counts
#define SEED2_VERSION2 2u  // codec-framed payload (see header comment)
#define SEED2_VERSION3 3u  // codec-framed payload + per-point canon algorithm tag
#define SEED2_CODEC_NONE 0u
#define SEED2_CODEC_ZLIB 1u
#define SEED2_CANON_ALGO_V1 1  // frozen serial FNV-1a (all champion seeds)
#define SEED2_CANON_ALGO_V2 2  // parallel segmented mix (GSIM_SEED2_CANON=v2)

bool mtSeed2WriteActive() { return std::getenv("GSIM_SCHEDULE_SEED2_WRITE") != nullptr; }
bool mtSeed2ReplayActive() { return std::getenv("GSIM_SCHEDULE_SEED2") != nullptr; }
bool mtSeed2CanonVerifyEnabled() {
  static const bool enabled = [](){
    const char* e = std::getenv("GSIM_SEED2_VERIFY_CANON");
    return !(e != nullptr && e[0] == '0');
  }();
  return enabled;
}

// Write-side knobs (read side auto-detects everything from the header):
//   GSIM_SEED2_CODEC = zlib (default) | none | v1 (write the legacy layout, for
//                      byte-for-byte comparison against existing champion seeds)
//   GSIM_SEED2_LEVEL = zlib deflate level 1..9 (default 6)
//   GSIM_SEED2_CANON = v1 (default) | v2. v2 records per-point canon_algo=2 tags
//                      (file version 3) and hashes each point with the parallel
//                      segmented mix; v1 keeps the frozen serial FNV-1a values.
//                      Incompatible with GSIM_SEED2_CODEC=v1 (the legacy layout
//                      cannot carry per-point tags).

// GSIM_SEED2_CANON for the write side: SEED2_CANON_ALGO_V1 (default) or V2.
// Anything but "" / "v1" / "v2" fails fast instead of silently hashing v1.
static int seed2CanonEnvAlgo() {
  const char* c = std::getenv("GSIM_SEED2_CANON");
  if (c == nullptr || c[0] == '\0' || std::strcmp(c, "v1") == 0) return SEED2_CANON_ALGO_V1;
  Assert(std::strcmp(c, "v2") == 0, "seed2: unknown GSIM_SEED2_CANON '%s' (want v1|v2)", c);
  if (std::strcmp(std::getenv("GSIM_SEED2_CODEC") ? std::getenv("GSIM_SEED2_CODEC") : "", "v1") == 0)
    Assert(false, "seed2: GSIM_SEED2_CANON=v2 cannot be combined with GSIM_SEED2_CODEC=v1 "
                  "(the legacy layout has no per-point canon tag; drop the codec override)");
  return SEED2_CANON_ALGO_V2;
}

void mtSeed2AssertCompatible() {
  Assert(!(mtSeed2WriteActive() && mtSeed2ReplayActive()),
         "GSIM_SCHEDULE_SEED2 and GSIM_SCHEDULE_SEED2_WRITE are mutually exclusive");
  Assert(!(mtSeed2WriteActive() && (mtSeedReplayActive() || mtSeedWriteActive())),
         "GSIM_SCHEDULE_SEED2_WRITE and GSIM_SCHEDULE_SEED(_WRITE) are mutually exclusive (use one seed format at a time)");
  Assert(!(mtSeed2ReplayActive() && (mtSeedReplayActive() || mtSeedWriteActive())),
         "GSIM_SCHEDULE_SEED2 and GSIM_SCHEDULE_SEED(_WRITE) are mutually exclusive");
  // Fail fast on GSIM_SEED2_CANON=v2 + GSIM_SEED2_CODEC=v1 (layout cannot carry tags).
  if (mtSeed2WriteActive()) seed2CanonEnvAlgo();
  // GSIM_STABLE_ORDER + SEED2_WRITE is allowed (records a stable-order run).
  // GSIM_STABLE_ORDER + SEED2 replay is pointless (two fixed orders) but harmless;
  // replay wins because it forces verbatim order after each point.
}

// ---------------- writer state ----------------

namespace {

struct Seed2Point {
  std::string tag;
  int canonAlgo = SEED2_CANON_ALGO_V1;  // algorithm that produced canonHash (v3 tag)
  uint64_t canonHash = 0;
  std::vector<uint32_t> nameIds;  // string-table ids in recorded sortedSuper order
};

struct Seed2WhenGroup {
  uint32_t condKey = 0;
  std::vector<uint32_t> sources;  // sources[0] = recorded merge target
};
struct Seed2Writer {
  std::unordered_map<std::string, uint32_t> idByKey;
  std::vector<std::string> keys;
  std::vector<Seed2Point> points;
  std::vector<Seed2WhenGroup> whenGroups;
  uint64_t inputHash = 0;
  bool inputHashSet = false;
  bool finalized = false;

  uint32_t intern(const std::string& key) {
    auto it = idByKey.find(key);
    if (it != idByKey.end()) return it->second;
    uint32_t id = (uint32_t)keys.size();
    idByKey.emplace(key, id);
    keys.push_back(key);
    return id;
  }
};

Seed2Writer& seed2Writer() {
  static Seed2Writer w;
  return w;
}

std::string seed2KeyOf(const SuperNode* super) {
  std::string k;
  for (Node* m : super->member) { k += m->name; k += ';'; }
  return k;
}

// Full-record key (member names + endpoint keys, same construction as canonInputHash).
// Used to disambiguate supers whose name-key collides (e.g. empty-member shells all
// hash to ""): identical full records are interchangeable, distinct ones get distinct keys.
std::string seed2FullKeyOf(const SuperNode* super) {
  auto keyOf = [](const SuperNode* e) {
    std::string k;
    for (Node* m : e->member) { k += m->name; k += ';'; }
    return k;
  };
  auto sortedEnds = [&](const std::set<SuperNode*>& ends) {
    std::vector<std::string> keys;
    for (SuperNode* e : ends) keys.push_back(keyOf(e));
    std::sort(keys.begin(), keys.end());
    std::string joined;
    for (const std::string& k : keys) { joined += k; joined += ','; }
    return joined;
  };
  return keyOf(super) + "|" + sortedEnds(super->prev) + "|" + sortedEnds(super->next) + "|" + sortedEnds(super->depPrev) + "|" + sortedEnds(super->depNext);
}

// Name-key with duplicate disambiguation: cheap name-key for unique supers, full-record
// key for colliding ones. dupCounts must be computed over the same super set (record side
// and replay side compute it independently; canon-verified identical content makes them agree).
static std::string seed2NodeKey(const SuperNode* super, const std::unordered_map<std::string, int>& dupCounts) {
  std::string k = seed2KeyOf(super);
  auto it = dupCounts.find(k);
  if (it != dupCounts.end() && it->second > 1) return "\x01" + seed2FullKeyOf(super);
  return k;
}

static void seed2CountKeys(const std::vector<SuperNode*>& supers, std::unordered_map<std::string, int>& out) {
  for (const SuperNode* super : supers) out[seed2KeyOf(super)]++;
}

static double seed2NowSec() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

// zlib's crc32/crc32 take a uInt (32-bit) length: feed in <=1GiB chunks so arbitrarily
// large rank arrays stay on the defined path.
static void seed2CrcUpdate(uint32_t& crc, const void* p, size_t n) {
  const unsigned char* b = static_cast<const unsigned char*>(p);
  while (n) {
    size_t chunk = n > ((size_t)1 << 30) ? ((size_t)1 << 30) : n;
    crc = (uint32_t)crc32(crc, b, (uInt)chunk);
    b += chunk;
    n -= chunk;
  }
}

// Header helpers (the quick-parse prefix is always stored uncompressed).
void seed2WriteU32(FILE* fp, uint32_t v) { std::fwrite(&v, 4, 1, fp); }
void seed2WriteU64(FILE* fp, uint64_t v) { std::fwrite(&v, 8, 1, fp); }
void seed2WriteStr(FILE* fp, const std::string& s) {
  seed2WriteU32(fp, (uint32_t)s.size());
  if (!s.empty()) Assert(std::fwrite(s.data(), 1, s.size(), fp) == s.size(), "seed2: write failed (disk full?)");
}
static void seed2StoreU64(unsigned char* p, uint64_t v) {
  for (int i = 0; i < 8; i++) p[i] = (unsigned char)(v >> (8 * i));
}
static void seed2StoreU32(unsigned char* p, uint32_t v) {
  for (int i = 0; i < 4; i++) p[i] = (unsigned char)(v >> (8 * i));
}

// Streaming payload sink: routes the exact v1 byte order either straight to the file
// (none / v1 layout) or through a zlib deflate stream, flushing bounded output chunks as
// they fill. The compressed image is never staged whole - peak extra memory is the one
// 256KiB output buffer.
struct Seed2Output {
  static constexpr size_t BUF = 256 * 1024;
  FILE* fp = nullptr;
  const char* path = nullptr;
  uint32_t codec = SEED2_CODEC_NONE;
  z_stream zs = {};
  bool zsActive = false;
  std::unique_ptr<unsigned char[]> outBuf;
  uint64_t rawBytes = 0;   // uncompressed payload bytes fed through write()
  uint64_t compBytes = 0;  // payload bytes flushed to the file
  uint32_t crc = 0;

  void open(FILE* f, const char* p, uint32_t codec_, int level) {
    fp = f;
    path = p;
    codec = codec_;
    crc = (uint32_t)crc32(0L, Z_NULL, 0);
    outBuf.reset(new unsigned char[BUF]);
    if (codec == SEED2_CODEC_ZLIB) {
      Assert(deflateInit(&zs, level) == Z_OK, "seed2: deflateInit failed on %s", path);
      zsActive = true;
    }
  }
  void putOut(size_t n) {
    if (!n) return;
    Assert(std::fwrite(outBuf.get(), 1, n, fp) == n, "seed2: write failed on %s (disk full?)", path);
    compBytes += n;
  }
  void write(const void* p, size_t n) {
    seed2CrcUpdate(crc, p, n);
    rawBytes += n;
    if (codec != SEED2_CODEC_ZLIB) {
      if (n) Assert(std::fwrite(p, 1, n, fp) == n, "seed2: write failed on %s (disk full?)", path);
      compBytes += n;
      return;
    }
    const unsigned char* b = static_cast<const unsigned char*>(p);
    while (n) {
      size_t feed = n > BUF ? BUF : n;
      zs.next_in = const_cast<Bytef*>(b);
      zs.avail_in = (uInt)feed;
      while (zs.avail_in > 0) {
        zs.next_out = outBuf.get();
        zs.avail_out = (uInt)BUF;
        int ret = deflate(&zs, Z_NO_FLUSH);
        Assert(ret == Z_OK, "seed2: deflate failed on %s (%s)", path, zs.msg ? zs.msg : "zlib error");
        putOut(BUF - zs.avail_out);
      }
      b += feed;
      n -= feed;
    }
  }
  void u32(uint32_t v) { write(&v, 4); }
  void u64(uint64_t v) { write(&v, 8); }
  void u8(uint8_t v) { write(&v, 1); }
  void str(const std::string& s) {
    u32((uint32_t)s.size());
    if (!s.empty()) write(s.data(), s.size());
  }
  void finish() {
    if (!zsActive) return;
    int ret = Z_OK;
    do {
      zs.next_out = outBuf.get();
      zs.avail_out = (uInt)BUF;
      ret = deflate(&zs, Z_FINISH);
      Assert(ret == Z_OK || ret == Z_STREAM_END || ret == Z_BUF_ERROR,
             "seed2: deflate finish failed on %s (%s)", path, zs.msg ? zs.msg : "zlib error");
      putOut(BUF - zs.avail_out);
    } while (ret != Z_STREAM_END);
    deflateEnd(&zs);
    zsActive = false;
  }
};

void mtSeed2Finalize() {
  Seed2Writer& w = seed2Writer();
  if (w.finalized || w.points.empty()) return;
  const char* path = std::getenv("GSIM_SCHEDULE_SEED2_WRITE");
  Assert(path != nullptr, "mtSeed2Finalize without GSIM_SCHEDULE_SEED2_WRITE");
  Assert(w.inputHashSet, "seed2: input hash was never set (topoSort record missing)");
  uint32_t codec = SEED2_CODEC_ZLIB;
  bool legacy = false;
  if (const char* c = std::getenv("GSIM_SEED2_CODEC")) {
    if (std::strcmp(c, "zlib") == 0) codec = SEED2_CODEC_ZLIB;
    else if (std::strcmp(c, "none") == 0) codec = SEED2_CODEC_NONE;
    else if (std::strcmp(c, "v1") == 0) { legacy = true; codec = SEED2_CODEC_NONE; }
    else Assert(false, "seed2: unknown GSIM_SEED2_CODEC '%s' (want zlib|none|v1)", c);
  }
  int level = 6;
  if (const char* l = std::getenv("GSIM_SEED2_LEVEL")) {
    level = std::atoi(l);
    Assert(level >= 1 && level <= 9, "seed2: GSIM_SEED2_LEVEL %d out of range 1..9", level);
  }
  const double t0 = seed2NowSec();
  FILE* fp = std::fopen(path, "wb");
  Assert(fp != nullptr, "cannot open seed2 output %s", path);
  Assert(std::fwrite(SEED2_MAGIC, 1, 4, fp) == 4, "seed2: write failed on %s (disk full?)", path);
  // 3 = codec-framed + per-point canon tags (any v2-canon point forces 3).
  bool anyCanonV2 = false;
  for (const Seed2Point& p : w.points) anyCanonV2 = anyCanonV2 || p.canonAlgo == SEED2_CANON_ALGO_V2;
  const uint32_t outVersion = legacy ? SEED2_VERSION : (anyCanonV2 ? SEED2_VERSION3 : SEED2_VERSION2);
  seed2WriteU32(fp, outVersion);
  seed2WriteU64(fp, w.inputHash);
  seed2WriteStr(fp, "wip/dense-b1-lookahead");
  seed2WriteU32(fp, (uint32_t)w.points.size());
  seed2WriteU32(fp, (uint32_t)w.keys.size());
  seed2WriteU32(fp, (uint32_t)w.whenGroups.size());
  long metaOff = -1;
  if (!legacy) {
    seed2WriteU32(fp, codec);
    seed2WriteU32(fp, (uint32_t)level);
    metaOff = std::ftell(fp);
    unsigned char zero[20] = {0};  // raw_payload_bytes, payload_crc32, comp_payload_bytes; patched below
    Assert(std::fwrite(zero, 1, 20, fp) == 20, "seed2: write failed on %s", path);
  }
  Seed2Output out;
  out.open(fp, path, codec, level);
  for (const std::string& k : w.keys) out.str(k);
  for (const Seed2Point& p : w.points) {
    out.str(p.tag);
    if (outVersion == SEED2_VERSION3) out.u8((uint8_t)p.canonAlgo);
    out.u64(p.canonHash);
    out.u32((uint32_t)p.nameIds.size());
    if (!p.nameIds.empty()) out.write(p.nameIds.data(), p.nameIds.size() * 4);
  }
  for (const Seed2WhenGroup& g : w.whenGroups) {
    out.u32(g.condKey);
    out.u32((uint32_t)g.sources.size());
    if (!g.sources.empty()) out.write(g.sources.data(), g.sources.size() * 4);
  }
  out.finish();
  if (!legacy) {
    unsigned char meta[20];
    seed2StoreU64(meta + 0, out.rawBytes);
    seed2StoreU32(meta + 8, out.crc);
    seed2StoreU64(meta + 12, out.compBytes);
    Assert(std::fseek(fp, metaOff, SEEK_SET) == 0, "seed2: cannot rewind header of %s", path);
    Assert(std::fwrite(meta, 1, 20, fp) == 20, "seed2: write failed on %s", path);
  }
  // Verify flush/close BEFORE declaring success: a delayed write error (full disk) must
  // never leave a silently truncated seed behind (observed in the campaign).
  Assert(std::fflush(fp) == 0, "seed2: flush failed on %s (disk full?)", path);
  Assert(std::fclose(fp) == 0, "seed2: close failed on %s (disk full?)", path);
  w.finalized = true;
  size_t bytes = 0;
  for (const std::string& k : w.keys) bytes += 4 + k.size();
  const double sec = seed2NowSec() - t0;
  fprintf(stderr,
          "[schedule-seed2] wrote %s: v=%u codec=%s level=%d canon=%s points=%zu names=%zu whenGroups=%zu "
          "table=%.1fMB payload=%.1fMB->%.1fMB (%.2fx) in %.2fs\n",
          path, outVersion, legacy ? "v1" : (codec == SEED2_CODEC_ZLIB ? "zlib" : "none"), level,
          anyCanonV2 ? "v2" : "v1",
          w.points.size(), w.keys.size(), w.whenGroups.size(), bytes / 1048576.0,
          out.rawBytes / 1048576.0, out.compBytes / 1048576.0,
          out.compBytes ? (double)out.rawBytes / (double)out.compBytes : 0.0, sec);
}

}  // namespace

void mtSeed2SetInputHash(uint64_t h) {
  Seed2Writer& w = seed2Writer();
  Assert(!w.inputHashSet, "seed2: input hash set twice");
  w.inputHash = h;
  w.inputHashSet = true;
}

void mtSeed2RecordPoint(const char* tag, const std::vector<SuperNode*>& sortedSuper, uint64_t canonHash) {
  mtSeed2AssertCompatible();
  Seed2Writer& w = seed2Writer();
  if (w.points.empty()) std::atexit(mtSeed2Finalize);
  Seed2Point p;
  p.tag = tag;
  p.canonAlgo = seed2CanonEnvAlgo();  // tag = the algorithm canonInputHash() used
  p.canonHash = canonHash;
  p.nameIds.reserve(sortedSuper.size());
  std::unordered_map<std::string, int> dupCounts;
  seed2CountKeys(sortedSuper, dupCounts);
  for (SuperNode* super : sortedSuper) p.nameIds.push_back(w.intern(seed2NodeKey(super, dupCounts)));
  w.points.push_back(std::move(p));
  fprintf(stderr, "[schedule-seed2] recorded %-24s nodes=%zu canon=%016zx\n",
          tag, sortedSuper.size(), (size_t)canonHash);
}

// S3: record one mergeWhenNodes group (application order = call order).
// Keys are FULL-record keys: no duplicate-count pass needed, and empty-member/merged
// shells stay distinguishable. Group count is small (thousands), size is irrelevant.
void mtSeed2RecordWhenGroup(const SuperNode* cond, const std::vector<SuperNode*>& sources) {
  mtSeed2AssertCompatible();
  Seed2Writer& w = seed2Writer();
  Assert(!sources.empty(), "seed2: when group with no sources");
  Seed2WhenGroup g;
  g.condKey = w.intern("\x01" + seed2FullKeyOf(cond));
  g.sources.reserve(sources.size());
  for (const SuperNode* s : sources) g.sources.push_back(w.intern("\x01" + seed2FullKeyOf(s)));
  w.whenGroups.push_back(std::move(g));
}

// ---------------- replay side (S2) ----------------

namespace {

struct Seed2Reader {
  bool loaded = false;
  uint64_t inputHash = 0;
  std::vector<std::string> keys;
  std::vector<Seed2Point> points;
  std::vector<Seed2WhenGroup> whenGroups;
  size_t cursor = 0;
};

Seed2Reader& seed2Reader() {
  static Seed2Reader r;
  return r;
}

// Header helpers (the quick-parse prefix is always stored uncompressed).
uint32_t seed2ReadU32(FILE* fp, const char* path) {
  uint32_t v;
  Assert(std::fread(&v, 4, 1, fp) == 1, "seed2: truncated file %s", path);
  return v;
}
uint64_t seed2ReadU64(FILE* fp, const char* path) {
  uint64_t v;
  Assert(std::fread(&v, 8, 1, fp) == 1, "seed2: truncated file %s", path);
  return v;
}
std::string seed2ReadStr(FILE* fp, const char* path, long fileBytes) {
  uint32_t n = seed2ReadU32(fp, path);
  // Bound the allocation by the remaining file size before touching memory: a corrupt
  // length of 0xffffffff must fail here, not in a 4 GiB allocation.
  long remaining = fileBytes >= 0 ? fileBytes - std::ftell(fp) : -1;
  Assert(remaining < 0 || (long)n <= remaining,
         "seed2: string length %u exceeds remaining file size in %s (corrupt seed)", n, path);
  std::string s(n, '\0');
  if (n) Assert(std::fread(s.data(), 1, n, fp) == n, "seed2: truncated file %s", path);
  return s;
}

// Streaming payload source: parses the exact v1 byte order from either the plain file
// (v1 / none) or an on-demand inflate stream. The uncompressed image is never staged
// whole - peak extra memory is two 256KiB chunk buffers. All length bounds use the
// remaining *uncompressed* payload bytes, which is the correct budget for every codec.
struct Seed2Input {
  static constexpr size_t BUF = 256 * 1024;
  FILE* fp = nullptr;
  const char* path = nullptr;
  uint32_t codec = SEED2_CODEC_NONE;
  z_stream zs = {};
  bool zsActive = false;
  bool inflateDone = false;
  std::unique_ptr<unsigned char[]> compBuf;  // file -> inflate input chunks
  std::unique_ptr<unsigned char[]> rawBuf;   // inflate (or fread) -> parser chunks
  size_t rawLen = 0, rawPos = 0;
  uint64_t rawRemaining = 0;   // unconsumed uncompressed payload bytes (buffered + future)
  uint64_t compRemaining = 0;  // unread payload bytes left in the file
  uint32_t crc = 0;
  uint32_t wantCrc = 0;
  bool verify = false;         // v2 only: strict trailing-byte + CRC checks

  void open(FILE* f, const char* p, uint32_t codec_, uint64_t rawPayload, uint64_t compPayload,
            uint32_t crcExpect, bool verify_) {
    fp = f;
    path = p;
    codec = codec_;
    rawRemaining = rawPayload;
    compRemaining = compPayload;
    crc = (uint32_t)crc32(0L, Z_NULL, 0);
    wantCrc = crcExpect;
    verify = verify_;
    compBuf.reset(new unsigned char[BUF]);
    rawBuf.reset(new unsigned char[BUF]);
    if (codec == SEED2_CODEC_ZLIB) {
      Assert(inflateInit(&zs) == Z_OK, "seed2: inflateInit failed on %s", path);
      zsActive = true;
    }
  }
  uint64_t remaining() const { return rawRemaining; }

  // Pull one chunk of uncompressed payload bytes (plain fread or one inflate call).
  void fill() {
    Assert(!inflateDone, "seed2: payload truncated in %s (structures exceed recorded payload)", path);
    if (codec != SEED2_CODEC_ZLIB) {
      Assert(compRemaining > 0, "seed2: payload truncated in %s", path);
      size_t k = compRemaining < BUF ? (size_t)compRemaining : BUF;
      Assert(std::fread(rawBuf.get(), 1, k, fp) == k, "seed2: truncated file %s", path);
      compRemaining -= k;
      rawLen = k;
      rawPos = 0;
      return;
    }
    if (zs.avail_in == 0 && compRemaining > 0) {
      size_t k = compRemaining < BUF ? (size_t)compRemaining : BUF;
      Assert(std::fread(compBuf.get(), 1, k, fp) == k, "seed2: truncated file %s", path);
      compRemaining -= k;
      zs.next_in = compBuf.get();
      zs.avail_in = (uInt)k;
    }
    zs.next_out = rawBuf.get();
    zs.avail_out = (uInt)BUF;
    int ret = inflate(&zs, Z_NO_FLUSH);
    Assert(ret == Z_OK || ret == Z_STREAM_END || ret == Z_BUF_ERROR,
           "seed2: corrupt zlib payload in %s (%s)", path, zs.msg ? zs.msg : "zlib error");
    rawLen = BUF - zs.avail_out;
    rawPos = 0;
    if (ret == Z_STREAM_END) {
      inflateDone = true;
      Assert(compRemaining == 0 && zs.avail_in == 0,
             "seed2: trailing compressed bytes after zlib stream end in %s (corrupt seed)", path);
    } else if (rawLen == 0 && zs.avail_in == 0 && compRemaining == 0) {
      // No progress possible and no input left: the stream cannot deliver the payload it promised.
      Assert(false, "seed2: compressed payload truncated in %s", path);
    }
  }

  void readExact(void* dst, size_t n) {
    unsigned char* d = static_cast<unsigned char*>(dst);
    while (n) {
      if (rawPos == rawLen) fill();
      size_t k = rawLen - rawPos;
      if (k > n) k = n;
      seed2CrcUpdate(crc, rawBuf.get() + rawPos, k);
      std::memcpy(d, rawBuf.get() + rawPos, k);
      rawPos += k;
      d += k;
      n -= k;
      rawRemaining -= k;
    }
  }
  uint32_t u32() {
    uint32_t v;
    readExact(&v, 4);
    return v;
  }
  uint64_t u64() {
    uint64_t v;
    readExact(&v, 8);
    return v;
  }
  std::string str() {
    uint32_t n = u32();
    Assert((uint64_t)n <= remaining(),
           "seed2: string length %u exceeds remaining payload size in %s (corrupt seed)", n, path);
    std::string s(n, '\0');
    if (n) readExact(s.data(), n);
    return s;
  }

  void finishPayload() {
    if (!zsActive) {
      if (verify) {
        Assert(rawRemaining == 0, "seed2: %llu unparsed payload bytes in %s (corrupt seed)",
               (unsigned long long)rawRemaining, path);
        Assert(compRemaining == 0, "seed2: %llu unread payload bytes in %s (corrupt seed)",
               (unsigned long long)compRemaining, path);
        Assert(crc == wantCrc, "seed2: payload checksum mismatch in %s (corrupt seed): have %08x want %08x",
               path, crc, wantCrc);
      }
      return;
    }
    while (!inflateDone) fill();
    Assert(rawRemaining == 0, "seed2: %llu unparsed payload bytes in %s (corrupt seed)",
           (unsigned long long)rawRemaining, path);
    inflateEnd(&zs);
    zsActive = false;
    Assert(crc == wantCrc, "seed2: payload checksum mismatch in %s (corrupt seed): have %08x want %08x",
           path, crc, wantCrc);
  }
};

void mtSeed2Load() {
  Seed2Reader& r = seed2Reader();
  if (r.loaded) return;
  mtSeed2AssertCompatible();
  const char* path = std::getenv("GSIM_SCHEDULE_SEED2");
  Assert(path != nullptr, "mtSeed2Load without GSIM_SCHEDULE_SEED2");
  const double t0 = seed2NowSec();
  FILE* fp = std::fopen(path, "rb");
  Assert(fp != nullptr, "cannot open schedule seed2 %s", path);
  Assert(std::fseek(fp, 0, SEEK_END) == 0, "seed2: cannot seek %s", path);
  const long fileBytes = std::ftell(fp);
  Assert(std::fseek(fp, 0, SEEK_SET) == 0, "seed2: cannot seek %s", path);
  char magic[4];
  Assert(std::fread(magic, 1, 4, fp) == 4 && std::memcmp(magic, SEED2_MAGIC, 4) == 0,
         "seed2: bad magic in %s (not a GS2 file)", path);
  uint32_t version = seed2ReadU32(fp, path);
  Assert(version == SEED2_VERSION || version == SEED2_VERSION2 || version == SEED2_VERSION3,
         "seed2: unsupported version %u in %s", version, path);
  r.inputHash = seed2ReadU64(fp, path);
  std::string generator = seed2ReadStr(fp, path, fileBytes);
  uint32_t pointCount = seed2ReadU32(fp, path);
  uint32_t nameCount = seed2ReadU32(fp, path);
  uint32_t whenGroupCount = seed2ReadU32(fp, path);
  uint32_t codec = SEED2_CODEC_NONE;
  uint32_t level = 0;
  uint64_t rawPayload = 0, compPayload = 0;
  uint32_t wantCrc = 0;
  if (version >= SEED2_VERSION2) {
    codec = seed2ReadU32(fp, path);
    level = seed2ReadU32(fp, path);
    rawPayload = seed2ReadU64(fp, path);
    wantCrc = seed2ReadU32(fp, path);
    compPayload = seed2ReadU64(fp, path);
    Assert(codec == SEED2_CODEC_NONE || codec == SEED2_CODEC_ZLIB,
           "seed2: unsupported codec %u in %s (this build reads none/zlib)", codec, path);
    long payloadOnDisk = fileBytes - std::ftell(fp);
    Assert(payloadOnDisk >= 0 && (uint64_t)payloadOnDisk == compPayload,
           "seed2: payload size mismatch in %s: header says %llu bytes, file has %ld (truncated or corrupt)",
           path, (unsigned long long)compPayload, payloadOnDisk);
  } else {
    compPayload = (uint64_t)(fileBytes - std::ftell(fp));
  }
  // Bound every count before reserving: each entry costs >= 4 payload bytes (a name
  // needs >= 4 for its length; a rank id 4), so counts beyond payload/4 are impossible
  // and indicate a corrupt seed. v1 bounds by the file size (payload == file tail).
  const uint64_t boundBytes = version >= SEED2_VERSION2 ? rawPayload : compPayload;
  const uint64_t maxEntries = boundBytes / 4;
  Assert((uint64_t)nameCount <= maxEntries && (uint64_t)pointCount <= maxEntries &&
         (uint64_t)whenGroupCount <= maxEntries,
         "seed2: implausible counts (points=%u names=%u groups=%u) for %llu-byte payload in %s",
         pointCount, nameCount, whenGroupCount, (unsigned long long)boundBytes, path);
  Seed2Input in;
  in.open(fp, path, codec, version >= SEED2_VERSION2 ? rawPayload : compPayload, compPayload,
          wantCrc, version >= SEED2_VERSION2);
  r.keys.reserve(nameCount);
  for (uint32_t i = 0; i < nameCount; i++) r.keys.push_back(in.str());
  r.points.reserve(pointCount);
  for (uint32_t i = 0; i < pointCount; i++) {
    Seed2Point p;
    p.tag = in.str();
    if (version == SEED2_VERSION3) {
      uint8_t algo;
      in.readExact(&algo, 1);
      Assert(algo == SEED2_CANON_ALGO_V1 || algo == SEED2_CANON_ALGO_V2,
             "seed2: unknown canon algorithm tag %u at point %u in %s (corrupt seed)", algo, i, path);
      p.canonAlgo = algo;
    }
    p.canonHash = in.u64();
    uint32_t n = in.u32();
    Assert((uint64_t)n * 4 <= in.remaining(),
           "seed2: rank count %u exceeds remaining payload size in %s (corrupt seed)", n, path);
    p.nameIds.resize(n);
    if (n) in.readExact(p.nameIds.data(), (size_t)n * 4);
    r.points.push_back(std::move(p));
  }
  // whenMap section (S3): recorded merge groups in application order.
  r.whenGroups.reserve(whenGroupCount);
  for (uint32_t i = 0; i < whenGroupCount; i++) {
    Seed2WhenGroup g;
    g.condKey = in.u32();
    uint32_t n = in.u32();
    Assert((uint64_t)n * 4 <= in.remaining(),
           "seed2: when-group size %u exceeds remaining payload size in %s (corrupt seed)", n, path);
    g.sources.resize(n);
    if (n) in.readExact(g.sources.data(), (size_t)n * 4);
    r.whenGroups.push_back(std::move(g));
  }
  in.finishPayload();
  std::fclose(fp);
  // Validate every table ID before any indexing: an out-of-range id would otherwise be
  // an out-of-bounds vector access (UB) instead of a clean compatibility error.
  const uint32_t nameTotal = (uint32_t)r.keys.size();
  for (const Seed2Point& p : r.points)
    for (uint32_t id : p.nameIds)
      Assert(id < nameTotal, "seed2: rank id %u out of range (%u names) in %s", id, nameTotal, path);
  for (const Seed2WhenGroup& g : r.whenGroups) {
    Assert(g.condKey < nameTotal, "seed2: cond key %u out of range (%u names) in %s", g.condKey, nameTotal, path);
    for (uint32_t id : g.sources)
      Assert(id < nameTotal, "seed2: when-group source id %u out of range (%u names) in %s", id, nameTotal, path);
  }
  r.loaded = true;
  const double sec = seed2NowSec() - t0;
  char payloadInfo[64];
  if (version >= SEED2_VERSION2 && codec == SEED2_CODEC_ZLIB)
    snprintf(payloadInfo, sizeof payloadInfo, "%.1fMB->%.1fMB on disk", rawPayload / 1048576.0,
             compPayload / 1048576.0);
  else
    snprintf(payloadInfo, sizeof payloadInfo, "%.1fMB",
             (version >= SEED2_VERSION2 ? rawPayload : compPayload) / 1048576.0);
  fprintf(stderr,
          "[schedule-seed2] loaded %s: v=%u codec=%s level=%u generator=%s points=%zu names=%zu "
          "whenGroups=%u payload=%s in %.2fs\n",
          path, version,
          version >= SEED2_VERSION2 ? (codec == SEED2_CODEC_ZLIB ? "zlib" : "none") : "legacy", level,
          generator.c_str(), r.points.size(), r.keys.size(), whenGroupCount, payloadInfo, sec);
}


}  // namespace

// ---------------- standalone transcode (format conversion, no generation) ----------------
// GSIM_SEED2_TRANSCODE=in[:out] turns a v1 seed into a v2 codec-framed seed (and can
// re-frame a v2 seed at another codec/level) WITHOUT running any graph pass: it pipes
// the source payload byte-for-byte through Seed2Input -> Seed2Output. The payload is
// never re-serialized from parsed structures, so intern ids, rank order, point tags,
// canon hashes, whenMap order and the input hash are identical by construction - a
// transcoded seed replays exactly like its source. Output codec/level come from
// GSIM_SEED2_CODEC (zlib|none; v1 is meaningless here) / GSIM_SEED2_LEVEL. Exits 0.
int mtSeed2TranscodeMain() {
  const char* spec = std::getenv("GSIM_SEED2_TRANSCODE");
  Assert(spec != nullptr && spec[0] != '\0', "GSIM_SEED2_TRANSCODE must be in[:out]");
  Assert(!mtSeed2WriteActive() && !mtSeed2ReplayActive(),
         "GSIM_SEED2_TRANSCODE is a standalone mode; unset GSIM_SCHEDULE_SEED2(_WRITE)");
  std::string in = spec, out;
  size_t colon = in.find(':');
  if (colon != std::string::npos) {
    out = in.substr(colon + 1);
    in.resize(colon);
  } else {
    out = in + ".v2";
  }
  Assert(!in.empty() && !out.empty(), "GSIM_SEED2_TRANSCODE paths must be non-empty");
  uint32_t codec = SEED2_CODEC_ZLIB;
  if (const char* c = std::getenv("GSIM_SEED2_CODEC")) {
    if (std::strcmp(c, "zlib") == 0) codec = SEED2_CODEC_ZLIB;
    else if (std::strcmp(c, "none") == 0) codec = SEED2_CODEC_NONE;
    else Assert(false, "seed2 transcode: GSIM_SEED2_CODEC '%s' unsupported here (want zlib|none)", c);
  }
  int level = 6;
  if (const char* l = std::getenv("GSIM_SEED2_LEVEL")) {
    level = std::atoi(l);
    Assert(level >= 1 && level <= 9, "seed2: GSIM_SEED2_LEVEL %d out of range 1..9", level);
  }
  const double t0 = seed2NowSec();
  FILE* fin = std::fopen(in.c_str(), "rb");
  Assert(fin != nullptr, "seed2 transcode: cannot open input %s", in.c_str());
  Assert(std::fseek(fin, 0, SEEK_END) == 0, "seed2 transcode: cannot seek %s", in.c_str());
  const long inBytes = std::ftell(fin);
  Assert(std::fseek(fin, 0, SEEK_SET) == 0, "seed2 transcode: cannot seek %s", in.c_str());
  char magic[4];
  Assert(std::fread(magic, 1, 4, fin) == 4 && std::memcmp(magic, SEED2_MAGIC, 4) == 0,
         "seed2: bad magic in %s (not a GS2 file)", in.c_str());
  uint32_t inVersion = seed2ReadU32(fin, in.c_str());
  Assert(inVersion == SEED2_VERSION || inVersion == SEED2_VERSION2 || inVersion == SEED2_VERSION3,
         "seed2 transcode: unsupported version %u in %s", inVersion, in.c_str());
  uint64_t inputHash = seed2ReadU64(fin, in.c_str());
  std::string generator = seed2ReadStr(fin, in.c_str(), inBytes);
  uint32_t pointCount = seed2ReadU32(fin, in.c_str());
  uint32_t nameCount = seed2ReadU32(fin, in.c_str());
  uint32_t whenGroupCount = seed2ReadU32(fin, in.c_str());
  uint32_t inCodec = SEED2_CODEC_NONE;
  uint64_t rawPayload = 0, compPayload = 0;
  uint32_t wantCrc = 0;
  if (inVersion >= SEED2_VERSION2) {
    inCodec = seed2ReadU32(fin, in.c_str());
    seed2ReadU32(fin, in.c_str());  // input level (informational)
    rawPayload = seed2ReadU64(fin, in.c_str());
    wantCrc = seed2ReadU32(fin, in.c_str());
    compPayload = seed2ReadU64(fin, in.c_str());
    Assert(inCodec == SEED2_CODEC_NONE || inCodec == SEED2_CODEC_ZLIB,
           "seed2 transcode: unsupported codec %u in %s", inCodec, in.c_str());
    long tail = inBytes - std::ftell(fin);
    Assert(tail >= 0 && (uint64_t)tail == compPayload,
           "seed2 transcode: payload size mismatch in %s (truncated or corrupt)", in.c_str());
  } else {
    compPayload = (uint64_t)(inBytes - std::ftell(fin));
    rawPayload = compPayload;
  }
  const uint64_t maxEntries = rawPayload / 4;
  Assert((uint64_t)nameCount <= maxEntries && (uint64_t)pointCount <= maxEntries &&
         (uint64_t)whenGroupCount <= maxEntries,
         "seed2 transcode: implausible counts (points=%u names=%u groups=%u) for %llu-byte payload in %s",
         pointCount, nameCount, whenGroupCount, (unsigned long long)rawPayload, in.c_str());
  FILE* fout = std::fopen(out.c_str(), "wb");
  Assert(fout != nullptr, "seed2 transcode: cannot open output %s", out.c_str());
  Assert(std::fwrite(SEED2_MAGIC, 1, 4, fout) == 4, "seed2 transcode: write failed on %s", out.c_str());
  // v3 inputs keep version 3 (the payload carries per-point canon tags byte-for-byte);
  // v1/v2 inputs have no tags, so the output is v2.
  seed2WriteU32(fout, inVersion >= SEED2_VERSION3 ? SEED2_VERSION3 : SEED2_VERSION2);
  seed2WriteU64(fout, inputHash);
  seed2WriteStr(fout, generator);
  seed2WriteU32(fout, pointCount);
  seed2WriteU32(fout, nameCount);
  seed2WriteU32(fout, whenGroupCount);
  seed2WriteU32(fout, codec);
  seed2WriteU32(fout, (uint32_t)level);
  const long metaOff = std::ftell(fout);
  unsigned char zero[20] = {0};  // raw_payload_bytes, payload_crc32, comp_payload_bytes; patched below
  Assert(std::fwrite(zero, 1, 20, fout) == 20, "seed2 transcode: write failed on %s", out.c_str());
  Seed2Input src;
  src.open(fin, in.c_str(), inCodec, rawPayload, compPayload, wantCrc, inVersion >= SEED2_VERSION2);
  Seed2Output dst;
  dst.open(fout, out.c_str(), codec, level);
  std::vector<unsigned char> buf(1 << 20);
  while (src.remaining() > 0) {
    size_t k = src.remaining() < buf.size() ? (size_t)src.remaining() : buf.size();
    src.readExact(buf.data(), k);
    dst.write(buf.data(), k);
  }
  dst.finish();
  src.finishPayload();  // input trailing-byte + CRC verification (v2 sources)
  if (inVersion == SEED2_VERSION)
    Assert(std::fgetc(fin) == EOF, "seed2 transcode: unparsed bytes at end of %s (corrupt seed)", in.c_str());
  Assert(dst.rawBytes == rawPayload,
         "seed2 transcode: copied %llu payload bytes but header promised %llu (internal error)",
         (unsigned long long)dst.rawBytes, (unsigned long long)rawPayload);
  unsigned char meta[20];
  seed2StoreU64(meta + 0, dst.rawBytes);
  seed2StoreU32(meta + 8, dst.crc);
  seed2StoreU64(meta + 12, dst.compBytes);
  Assert(std::fseek(fout, metaOff, SEEK_SET) == 0, "seed2 transcode: cannot rewind header of %s", out.c_str());
  Assert(std::fwrite(meta, 1, 20, fout) == 20, "seed2 transcode: write failed on %s", out.c_str());
  Assert(std::fflush(fout) == 0 && std::fclose(fout) == 0, "seed2 transcode: close failed on %s", out.c_str());
  std::fclose(fin);
  fprintf(stderr,
          "[schedule-seed2] transcoded %s(v=%u) -> %s(v=%u codec=%s level=%d): points=%u names=%u "
          "whenGroups=%u payload=%.1fMB->%.1fMB (%.2fx) in %.2fs\n",
          in.c_str(), inVersion, out.c_str(), inVersion >= SEED2_VERSION3 ? 3u : 2u,
          codec == SEED2_CODEC_ZLIB ? "zlib" : "none", level,
          pointCount, nameCount, whenGroupCount, dst.rawBytes / 1048576.0, dst.compBytes / 1048576.0,
          dst.compBytes ? (double)dst.rawBytes / (double)dst.compBytes : 0.0, seed2NowSec() - t0);
  return 0;
}

void mtSeed2VerifyInputHash(uint64_t computedInputHash) {
  mtSeed2Load();
  Seed2Reader& r = seed2Reader();
  Assert(r.inputHash == computedInputHash,
         "seed2 input mismatch: seed has %016zx, this run computes %016zx (different RTL/front-end state)",
         (size_t)r.inputHash, (size_t)computedInputHash);
}

// Canon algorithm for the point about to be hashed. Write mode: env (uniform per
// run). Replay mode: the NEXT point's recorded tag - points are consumed strictly
// in order, and canonInputHash() runs before mtSeed2ApplyPoint advances the
// cursor, so the peek always names the point being verified. Untagged v1/v2
// seeds and any non-seed2 context are v1, which is why every existing champion
// seed keeps hashing exactly as before. Exhausted replay (cursor at end) has no
// point left to verify; v1 is the harmless default there.
int mtSeed2CanonAlgo() {
  if (mtSeed2WriteActive()) return seed2CanonEnvAlgo();
  if (mtSeed2ReplayActive()) {
    mtSeed2Load();
    Seed2Reader& r = seed2Reader();
    if (r.cursor < r.points.size()) return r.points[r.cursor].canonAlgo;
  }
  return SEED2_CANON_ALGO_V1;
}

// Force sortedSuper to the recorded permutation of the NEXT expected point.
// The caller recomputes order fields (orderAllNodes) after this returns.
void mtSeed2ApplyPoint(const char* tag, std::vector<SuperNode*>& sortedSuper, uint64_t currentCanonHash) {
  mtSeed2Load();
  Seed2Reader& r = seed2Reader();
  Assert(r.cursor < r.points.size(),
         "seed2 replay: run produced point %s but the seed has only %zu points (config mismatch)",
         tag, r.points.size());
  const Seed2Point& p = r.points[r.cursor];
  Assert(p.tag == tag,
         "seed2 replay divergence: run reached point %s but the seed expects %s (pass-flow mismatch)",
         p.tag.c_str(), tag);
  Assert(!mtSeed2CanonVerifyEnabled() || p.canonHash == currentCanonHash,
         "seed2 replay divergence at %s: seed canon %016zx vs run %016zx (an upstream order-consuming point was not pinned)",
         tag, (size_t)p.canonHash, (size_t)currentCanonHash);
  Assert(p.nameIds.size() == sortedSuper.size(),
         "seed2 replay divergence at %s: seed has %zu nodes, run has %zu",
         tag, p.nameIds.size(), sortedSuper.size());
  // Resolve with duplicate disambiguation (same scheme as RecordPoint). Colliding
  // name-keys (e.g. empty-member shells) fall back to full-record keys; supers with
  // identical full records are interchangeable, so queue order within a key is free.
  // Memory: one inline pointer per unique key; only the (tiny) duplicate-key set gets
  // deque storage. A deque per key over 7.2M keys was an OOM-prone allocation storm.
  auto tKeys = phasetimer::now();
  std::unordered_map<std::string, int> dupCounts;
  seed2CountKeys(sortedSuper, dupCounts);
  phasetimer::mark("apply.countKeys", tKeys);
  auto tIdx = phasetimer::now();
  std::unordered_map<std::string, SuperNode*> firstByKey;
  std::unordered_map<std::string, std::deque<SuperNode*>> extraByKey;
  firstByKey.reserve(sortedSuper.size() * 2);
  for (SuperNode* super : sortedSuper) {
    std::string k = seed2NodeKey(super, dupCounts);
    auto [it, inserted] = firstByKey.emplace(std::move(k), super);
    if (!inserted) extraByKey[it->first].push_back(super);
  }
  phasetimer::mark("apply.firstByKey", tIdx);
  auto tForced = phasetimer::now();
  std::vector<SuperNode*> forced;
  forced.reserve(sortedSuper.size());
  for (uint32_t id : p.nameIds) {
    if (id >= r.keys.size()) {
      Assert(false, "seed2 replay: rank id %u out of range (table has %zu names) at %s (corrupt seed?)",
             id, r.keys.size(), tag);
    }
    const std::string& key = r.keys[id];
    auto it = firstByKey.find(key);
    if (it != firstByKey.end()) {
      forced.push_back(it->second);
      firstByKey.erase(it);
      continue;
    }
    auto eit = extraByKey.find(key);
    Assert(eit != extraByKey.end() && !eit->second.empty(),
           "seed2 replay: recorded node key missing at %s (seed/run graph mismatch)", tag);
    forced.push_back(eit->second.front());
    eit->second.pop_front();
  }
  phasetimer::mark("apply.forced", tForced);
  sortedSuper = std::move(forced);
  r.cursor++;
  fprintf(stderr, "[schedule-seed2] applied  %-24s nodes=%zu canon=%016zx (%zu/%zu)\n",
          tag, sortedSuper.size(), (size_t)currentCanonHash, r.cursor, r.points.size());
}

bool mtSeed2ReplayPointPending(const char* tag) {
  if (!mtSeed2ReplayActive()) return false;
  mtSeed2Load();
  Seed2Reader& r = seed2Reader();
  return r.cursor < r.points.size() && r.points[r.cursor].tag == tag;
}

// S3 replay accessors: recorded when groups as key strings, in application order.
std::string mtSeed2KeyOf(const SuperNode* super) { return seed2KeyOf(super); }
std::string mtSeed2FullKeyOf(const SuperNode* super) { return "\x01" + seed2FullKeyOf(super); }

size_t mtSeed2WhenGroupCount() {
  mtSeed2Load();
  return seed2Reader().whenGroups.size();
}

std::string mtSeed2WhenGroupCondKey(size_t i) {
  Seed2Reader& r = seed2Reader();
  Assert(i < r.whenGroups.size(), "seed2: when group index %zu out of range", i);
  return r.keys[r.whenGroups[i].condKey];
}

std::vector<std::string> mtSeed2WhenGroupSourceKeys(size_t i) {
  Seed2Reader& r = seed2Reader();
  Assert(i < r.whenGroups.size(), "seed2: when group index %zu out of range", i);
  std::vector<std::string> out;
  out.reserve(r.whenGroups[i].sources.size());
  for (uint32_t id : r.whenGroups[i].sources) out.push_back(r.keys[id]);
  return out;
}

// ---------------- standalone canon fuzz (GSIM_SEED2_CANON_FUZZ=<iters>) ----------
// Divergence-detection harness on synthetic record streams: v1 and v2 must BOTH
// change whenever the stream changes by one record (byte flip, insert, delete,
// duplicate). Also pins v2 determinism (same stream -> same value, independent
// of thread scheduling) and the exact segment-count construction. Exits 0 on
// PASS, nonzero on the first violation. No generation runs.
int mtSeed2CanonFuzzMain() {
  const char* spec = std::getenv("GSIM_SEED2_CANON_FUZZ");
  long iters = spec ? std::atol(spec) : 100;
  if (iters < 1) iters = 1;
  Assert(!mtSeed2WriteActive() && !mtSeed2ReplayActive(),
         "GSIM_SEED2_CANON_FUZZ is a standalone mode; unset GSIM_SCHEDULE_SEED2(_WRITE)");
  // Deterministic splitmix64 stream: fixed seed, so a failure reproduces exactly.
  uint64_t rngState = 0x9E3779B97F4A7C15ULL;
  auto nextRand = [&]() {
    uint64_t z = (rngState += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  };
  auto makeViews = [&](size_t n, std::vector<std::string>& storage) {
    storage.clear();
    storage.reserve(n);
    for (size_t i = 0; i < n; i++) {
      // Lengths 0..63 hit empty records and segment-boundary straddling alike;
      // biased alphabet keeps accidental coincidences meaningful, not certain.
      size_t len = (size_t)(nextRand() % 64);
      std::string s(len, '\0');
      for (char& c : s) c = (char)('a' + (int)(nextRand() % 8));
      storage.push_back(std::move(s));
    }
  };
  auto toViewList = [](const std::vector<std::string>& storage) {
    std::vector<std::string_view> views;
    views.reserve(storage.size());
    for (const std::string& s : storage) views.emplace_back(s.data(), s.size());
    return views;
  };
  long flips = 0, inserts = 0, deletes = 0, dups = 0, sizes = 0;
  std::vector<std::string> storage;
  for (long it = 0; it < iters; it++) {
    // Sizes sweep the segment-count function: 0, 1, small, k*256-1, k*256, k*256+1.
    static const size_t fixedSizes[] = {0, 1, 2, 255, 256, 257, 511, 512, 65535, 65536, 65537, 1048577};
    size_t n = (it % 12 == 0) ? fixedSizes[(it / 12) % 12]
                              : (size_t)(nextRand() % 900);
    makeViews(n, storage);
    auto views = toViewList(storage);
    uint64_t h1 = graph::canonMixV1(views);
    uint64_t h2 = graph::canonMixV2(views);
    // Determinism: recompute both; v2 spawns threads whose count/interleaving
    // must not leak into the value.
    Assert(graph::canonMixV1(views) == h1, "canon fuzz iter %ld: v1 not deterministic", it);
    Assert(graph::canonMixV2(views) == h2, "canon fuzz iter %ld: v2 not deterministic", it);
    if (n == 0) continue;
    int mode = (int)(nextRand() % 4);
    // streamVisible = the concatenated byte stream v1 hashes actually changed.
    // v1 frames nothing per record, so inserting/deleting an EMPTY record is
    // invisible to it (frozen construction; empty-member shells hash to "").
    // v2 frames every segment with byte+record counts and folds n, so it must
    // catch structural changes even when zero bytes move.
    bool streamVisible = true;
    if (mode == 0) {
      // Flip one byte in one record (falls through to duplicate when the picked
      // record is empty - there is no byte to flip).
      size_t ri = nextRand() % n;
      if (!storage[ri].empty()) {
        size_t bi = nextRand() % storage[ri].size();
        storage[ri][bi] = (char)('a' + ((storage[ri][bi] - 'a' + 1 + (int)(nextRand() % 7)) % 8));
        flips++;
      } else {
        const std::string& rec = storage[nextRand() % n];
        streamVisible = !rec.empty();
        storage.push_back(rec);
        dups++;
      }
    } else if (mode == 1) {
      std::string rec((size_t)(nextRand() % 9), 'x');
      streamVisible = !rec.empty();
      storage.insert(storage.begin() + (long)(nextRand() % n), std::move(rec));
      inserts++;
    } else if (mode == 2 && n > 1) {
      size_t ri = nextRand() % n;
      streamVisible = !storage[ri].empty();
      storage.erase(storage.begin() + (long)ri);
      deletes++;
    } else {
      const std::string& rec = storage[nextRand() % n];
      streamVisible = !rec.empty();
      storage.push_back(rec);
      dups++;
    }
    auto mutViews = toViewList(storage);
    uint64_t m1 = graph::canonMixV1(mutViews);
    uint64_t m2 = graph::canonMixV2(mutViews);
    if (streamVisible) {
      Assert(m1 != h1,
             "canon fuzz iter %ld: v1 MISSED a stream-visible mutation (n=%zu mode=%d h=%016zx)", it, n, mode, (size_t)h1);
    } else {
      Assert(m1 == h1,
             "canon fuzz iter %ld: v1 changed on an empty-record mutation (n=%zu mode=%d)", it, n, mode);
    }
    Assert(m2 != h2,
           "canon fuzz iter %ld: v2 MISSED the mutation (n=%zu mode=%d h=%016zx)", it, n, mode, (size_t)h2);
    sizes += (long)n;
  }
  fprintf(stderr, "[canon-fuzz] PASS: %ld iters (%ld flips, %ld inserts, %ld deletes, %ld dups, "
          "%ld total records): every stream-visible mutation changed v1; EVERY mutation "
          "(including empty-record structural changes v1 cannot see) changed v2\n",
          iters, flips, inserts, deletes, dups, sizes);
  return 0;
}
