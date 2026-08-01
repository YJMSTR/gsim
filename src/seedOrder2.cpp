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
  for (SuperNode* super : sortedSuper) p.nameIds.push_back(w.intern(seed2KeyOf(super)));
  w.points.push_back(std::move(p));
  fprintf(stderr, "[schedule-seed2] recorded %-24s nodes=%zu canon=%016zx\n",
          tag, sortedSuper.size(), (size_t)canonHash);
}
