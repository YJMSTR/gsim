// GSIM_SCHEDULE_SEED2 exact replay (design: docs/exact-replay-v2-design.md).
// Seed v2 records the OUTCOME of every order-consuming graph point (topoSort and every
// resort, plus mergeWhenNodes' whenMap) so replay can force each point verbatim instead
// of re-deriving it with a policy comparator (v1's +32% shape tax, v436).
//
// Binary format (little-endian), file extension convention *.gsimseed2:
//   char  magic[4]   = "GS2\0"
//   u32   version    = 1
//   u64   input_hash                 (graph::canonInputHash() at topoSort entry)
//   u32   generator_len, bytes       (generator tag)
//   u32   point_count
//   u32   name_count
//   u32   whenmap_group_count
//   [string table] name_count x { u32 len, bytes }      (SuperNode keys, see keyOf)
//   [points] point_count x {
//     u32 tag_len, bytes
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
// A SuperNode key is the ';'-joined member-name list (same construction as
// graph::canonInputHash's keyOf) - member names are unique, so the list keys the node.
//
// S1 scope: write path (passive recording; never changes generation behavior).
// Replay lands in S2 (points) and S3 (whenMap).

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unordered_map>
#include <deque>
#include <vector>
#include "common.h"

#define SEED2_MAGIC "GS2\0"
#define SEED2_VERSION 1u

bool mtSeed2WriteActive() { return std::getenv("GSIM_SCHEDULE_SEED2_WRITE") != nullptr; }
bool mtSeed2ReplayActive() { return std::getenv("GSIM_SCHEDULE_SEED2") != nullptr; }

void mtSeed2AssertCompatible() {
  Assert(!(mtSeed2WriteActive() && mtSeed2ReplayActive()),
         "GSIM_SCHEDULE_SEED2 and GSIM_SCHEDULE_SEED2_WRITE are mutually exclusive");
  Assert(!(mtSeed2WriteActive() && (mtSeedReplayActive() || mtSeedWriteActive())),
         "GSIM_SCHEDULE_SEED2_WRITE and GSIM_SCHEDULE_SEED(_WRITE) are mutually exclusive (record v1 with v1, v2 with v2)");
  Assert(!(mtSeed2ReplayActive() && (mtSeedReplayActive() || mtSeedWriteActive())),
         "GSIM_SCHEDULE_SEED2 and GSIM_SCHEDULE_SEED(_WRITE) are mutually exclusive");
  // GSIM_STABLE_ORDER + SEED2_WRITE is allowed (records a stable-order run).
  // GSIM_STABLE_ORDER + SEED2 replay is pointless (two fixed orders) but harmless;
  // replay wins because it forces verbatim order after each point.
}

// ---------------- writer state ----------------

namespace {

struct Seed2Point {
  std::string tag;
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

void seed2WriteU32(FILE* fp, uint32_t v) { std::fwrite(&v, 4, 1, fp); }
void seed2WriteU64(FILE* fp, uint64_t v) { std::fwrite(&v, 8, 1, fp); }
void seed2WriteStr(FILE* fp, const std::string& s) {
  seed2WriteU32(fp, (uint32_t)s.size());
  if (!s.empty()) std::fwrite(s.data(), 1, s.size(), fp);
}

void mtSeed2Finalize() {
  Seed2Writer& w = seed2Writer();
  if (w.finalized || w.points.empty()) return;
  w.finalized = true;
  const char* path = std::getenv("GSIM_SCHEDULE_SEED2_WRITE");
  Assert(path != nullptr, "mtSeed2Finalize without GSIM_SCHEDULE_SEED2_WRITE");
  Assert(w.inputHashSet, "seed2: input hash was never set (topoSort record missing)");
  FILE* fp = std::fopen(path, "wb");
  Assert(fp != nullptr, "cannot open seed2 output %s", path);
  std::fwrite(SEED2_MAGIC, 1, 4, fp);
  seed2WriteU32(fp, SEED2_VERSION);
  seed2WriteU64(fp, w.inputHash);
  seed2WriteStr(fp, "wip/dense-b1-lookahead");
  seed2WriteU32(fp, (uint32_t)w.points.size());
  seed2WriteU32(fp, (uint32_t)w.keys.size());
  seed2WriteU32(fp, (uint32_t)w.whenGroups.size());
  for (const std::string& k : w.keys) seed2WriteStr(fp, k);
  for (const Seed2Point& p : w.points) {
    seed2WriteStr(fp, p.tag);
    seed2WriteU64(fp, p.canonHash);
    seed2WriteU32(fp, (uint32_t)p.nameIds.size());
    if (!p.nameIds.empty()) std::fwrite(p.nameIds.data(), 4, p.nameIds.size(), fp);
  }
  for (const Seed2WhenGroup& g : w.whenGroups) {
    seed2WriteU32(fp, g.condKey);
    seed2WriteU32(fp, (uint32_t)g.sources.size());
    if (!g.sources.empty()) std::fwrite(g.sources.data(), 4, g.sources.size(), fp);
  }
  std::fclose(fp);
  size_t bytes = 0;
  for (const std::string& k : w.keys) bytes += 4 + k.size();
  fprintf(stderr, "[schedule-seed2] wrote %s: points=%zu names=%zu whenGroups=%zu table=%.1fMB\n",
          path, w.points.size(), w.keys.size(), w.whenGroups.size(), bytes / 1048576.0);
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
std::string seed2ReadStr(FILE* fp, const char* path) {
  uint32_t n = seed2ReadU32(fp, path);
  std::string s(n, '\0');
  if (n) Assert(std::fread(s.data(), 1, n, fp) == n, "seed2: truncated file %s", path);
  return s;
}

void mtSeed2Load() {
  Seed2Reader& r = seed2Reader();
  if (r.loaded) return;
  mtSeed2AssertCompatible();
  const char* path = std::getenv("GSIM_SCHEDULE_SEED2");
  Assert(path != nullptr, "mtSeed2Load without GSIM_SCHEDULE_SEED2");
  FILE* fp = std::fopen(path, "rb");
  Assert(fp != nullptr, "cannot open schedule seed2 %s", path);
  char magic[4];
  Assert(std::fread(magic, 1, 4, fp) == 4 && std::memcmp(magic, SEED2_MAGIC, 4) == 0,
         "seed2: bad magic in %s (not a GS2 file)", path);
  uint32_t version = seed2ReadU32(fp, path);
  Assert(version == SEED2_VERSION, "seed2: unsupported version %u in %s", version, path);
  r.inputHash = seed2ReadU64(fp, path);
  std::string generator = seed2ReadStr(fp, path);
  uint32_t pointCount = seed2ReadU32(fp, path);
  uint32_t nameCount = seed2ReadU32(fp, path);
  uint32_t whenGroupCount = seed2ReadU32(fp, path);
  r.keys.reserve(nameCount);
  for (uint32_t i = 0; i < nameCount; i++) r.keys.push_back(seed2ReadStr(fp, path));
  r.points.reserve(pointCount);
  for (uint32_t i = 0; i < pointCount; i++) {
    Seed2Point p;
    p.tag = seed2ReadStr(fp, path);
    p.canonHash = seed2ReadU64(fp, path);
    uint32_t n = seed2ReadU32(fp, path);
    p.nameIds.resize(n);
    if (n) Assert(std::fread(p.nameIds.data(), 4, n, fp) == n, "seed2: truncated file %s", path);
    r.points.push_back(std::move(p));
  }
  // whenMap section (S3): recorded merge groups in application order.
  r.whenGroups.reserve(whenGroupCount);
  for (uint32_t i = 0; i < whenGroupCount; i++) {
    Seed2WhenGroup g;
    g.condKey = seed2ReadU32(fp, path);
    uint32_t n = seed2ReadU32(fp, path);
    g.sources.resize(n);
    if (n) Assert(std::fread(g.sources.data(), 4, n, fp) == n, "seed2: truncated file %s", path);
    r.whenGroups.push_back(std::move(g));
  }
  std::fclose(fp);
  r.loaded = true;
  fprintf(stderr, "[schedule-seed2] loaded %s: generator=%s points=%zu names=%zu whenGroups=%u\n",
          path, generator.c_str(), r.points.size(), r.keys.size(), whenGroupCount);
}

}  // namespace

void mtSeed2VerifyInputHash(uint64_t computedInputHash) {
  mtSeed2Load();
  Seed2Reader& r = seed2Reader();
  Assert(r.inputHash == computedInputHash,
         "seed2 input mismatch: seed has %016zx, this run computes %016zx (different RTL/front-end state)",
         (size_t)r.inputHash, (size_t)computedInputHash);
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
  Assert(p.canonHash == currentCanonHash,
         "seed2 replay divergence at %s: seed canon %016zx vs run %016zx (an upstream order-consuming point was not pinned)",
         tag, (size_t)p.canonHash, (size_t)currentCanonHash);
  Assert(p.nameIds.size() == sortedSuper.size(),
         "seed2 replay divergence at %s: seed has %zu nodes, run has %zu",
         tag, p.nameIds.size(), sortedSuper.size());
  // Resolve with duplicate disambiguation (same scheme as RecordPoint). Colliding
  // name-keys (e.g. empty-member shells) fall back to full-record keys; supers with
  // identical full records are interchangeable, so queue order within a key is free.
  std::unordered_map<std::string, int> dupCounts;
  seed2CountKeys(sortedSuper, dupCounts);
  std::unordered_map<std::string, std::deque<SuperNode*>> byKey;
  byKey.reserve(sortedSuper.size() * 2);
  for (SuperNode* super : sortedSuper) byKey[seed2NodeKey(super, dupCounts)].push_back(super);
  std::vector<SuperNode*> forced;
  forced.reserve(sortedSuper.size());
  for (uint32_t id : p.nameIds) {
    auto it = byKey.find(r.keys[id]);
    Assert(it != byKey.end() && !it->second.empty(),
           "seed2 replay: recorded node key missing at %s (seed/run graph mismatch)", tag);
    forced.push_back(it->second.front());
    it->second.pop_front();
  }
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
