// GSIM_SCHEDULE_SEED fixed-order replay (design: docs/schedule-pinning-design.md).
// Seed file format (plain text, v1):
//   # gsimseed v1
//   input_hash <hex64>
//   node_count <N>
//   generator <string>
//   ---
//   <member-name-per-line, in champion sortedSuper order>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>
#include <climits>
#include "common.h"

static bool mtEnvEnabled(const char* name) {
  const char* env = std::getenv(name);
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

bool mtSeedReplayActive() { return std::getenv("GSIM_SCHEDULE_SEED") != nullptr; }
bool mtSeedWriteActive() { return std::getenv("GSIM_SCHEDULE_SEED_WRITE") != nullptr; }

static std::unordered_map<std::string, long long> seedRankByName;
static std::string seedDeclaredHash;
static bool seedLoaded = false;

void mtSeedAssertCompatible() {
  Assert(!(mtSeedReplayActive() && mtSeedWriteActive()),
         "GSIM_SCHEDULE_SEED and GSIM_SCHEDULE_SEED_WRITE are mutually exclusive");
  Assert(!(mtSeedReplayActive() && mtEnvEnabled("GSIM_STABLE_ORDER")),
         "GSIM_SCHEDULE_SEED and GSIM_STABLE_ORDER are mutually exclusive (two determinism paths)");
}

static void mtSeedLoad() {
  if (seedLoaded) return;
  mtSeedAssertCompatible();
  const char* path = std::getenv("GSIM_SCHEDULE_SEED");
  Assert(path != nullptr, "mtSeedLoad called without GSIM_SCHEDULE_SEED");
  FILE* fp = std::fopen(path, "r");
  Assert(fp != nullptr, "cannot open schedule seed %s", path);
  char line[4096];
  bool headerDone = false;
  long long rank = 0;
  long long declaredCount = -1;
  char declaredHash[128] = {0};
  while (std::fgets(line, sizeof(line), fp)) {
    if (!headerDone) {
      if (std::strncmp(line, "# gsimseed v1", 13) == 0) continue;
      if (std::sscanf(line, "input_hash %127s", declaredHash) == 1) continue;
      if (std::sscanf(line, "node_count %lld", &declaredCount) == 1) continue;
      if (std::strncmp(line, "generator", 9) == 0) continue;
      if (std::strncmp(line, "date", 4) == 0) continue;
      if (std::strncmp(line, "---", 3) == 0) { headerDone = true; continue; }
      Assert(false, "malformed schedule seed header line: %s", line);
    } else {
      size_t len = std::strlen(line);
      while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) line[--len] = '\0';
      if (len == 0) continue;
      seedRankByName.emplace(std::string(line, len), rank++);
    }
  }
  std::fclose(fp);
  Assert(declaredCount < 0 || declaredCount == rank,
         "schedule seed %s declares %lld nodes but lists %lld", path, declaredCount, rank);
  Assert(declaredHash[0] != '\0', "schedule seed %s lacks input_hash", path);
  seedDeclaredHash = declaredHash;
  seedLoaded = true;
  fprintf(stderr, "[schedule-seed] loaded %lld ranks from %s\n", rank, path);
}

void mtSeedVerifyInputHash(uint64_t computedInputHash) {
  if (!mtSeedReplayActive()) return;
  if (!seedLoaded) mtSeedLoad();
  char computed[32];
  std::snprintf(computed, sizeof(computed), "%016llx", (unsigned long long)computedInputHash);
  Assert(std::strcmp(computed, seedDeclaredHash.c_str()) == 0,
         "schedule seed input mismatch: seed hash %s vs current graph hash %s "
         "(input FIR or generator pipeline changed; regenerate a fresh seed with GSIM_SCHEDULE_SEED_WRITE)",
         seedDeclaredHash.c_str(), computed);
}

long long mtSeedRankOf(const SuperNode* super) {
  if (!seedLoaded) mtSeedLoad();
  long long best = LLONG_MAX;
  for (Node* member : super->member) {
    auto it = seedRankByName.find(member->name);
    if (it != seedRankByName.end()) {
      best = std::min(best, it->second);
    }
  }
  return best;
}

void mtSeedWrite(const char* path, const std::vector<SuperNode*>& sortedSuper, uint64_t inputHash, const char* generatorTag) {
  FILE* fp = std::fopen(path, "w");
  Assert(fp != nullptr, "cannot write schedule seed %s", path);
  std::fprintf(fp, "# gsimseed v1\n");
  std::fprintf(fp, "input_hash %016llx\n", (unsigned long long)inputHash);
  std::fprintf(fp, "node_count %zu\n", sortedSuper.size());
  std::fprintf(fp, "generator %s\n", generatorTag);
  std::fprintf(fp, "---\n");
  for (const SuperNode* super : sortedSuper) {
    for (Node* member : super->member) {
      std::fprintf(fp, "%s\n", member->name.c_str());
      break;  // seed-time supers carry one node each; first member is the identity
    }
  }
  std::fclose(fp);
  fprintf(stderr, "[schedule-seed] wrote %zu ranks to %s\n", sortedSuper.size(), path);
}
