#include "common.h"
#include <algorithm>
#include <atomic>
#include "phaseTimer.h"
#include <climits>
#include <cstddef>
#include <map>
#include <cstdio>
#include <queue>
#include <stack>
#include <map>
#include <thread>
#include <tuple>

// #define SUPER_BOUND 35

// GSIM_STABLE_ORDER=1 selects the deterministic frontier (see topoSort).
void graph::resort(const char* seed2Tag) {
  PhaseName resortLabel("resort", seed2Tag);
  PhaseTimer resortTimer(resortLabel.c_str());
  const bool stableOrder = [](){ const char* e = std::getenv("GSIM_STABLE_ORDER"); return e && e[0] && e[0] != '0'; }();
  const bool seedReplay = mtSeedReplayActive();
  const bool seed2Write = mtSeed2WriteActive();
  const bool seed2Replay = mtSeed2ReplayActive();
  auto recordSeed2 = [&]() {
    auto t0 = phasetimer::now();
    if (seed2Write && seed2Tag) mtSeed2RecordPoint(seed2Tag, sortedSuper, canonInputHash());
    if (seed2Replay && seed2Tag) {
      // Canon verification is the replay integrity check; with GSIM_SEED2_VERIFY_CANON=0
      // skip the (serial, ~minutes on v1 seeds) hash entirely - the applied order comes
      // from the pin payload either way.
      mtSeed2ApplyPoint(seed2Tag, sortedSuper, mtSeed2CanonVerifyEnabled() ? canonInputHash() : 0);
      orderAllNodes();
    }
    phasetimer::mark("resort.recordSeed2", t0);
  };
  std::map<SuperNode*, int>times;
  if (seedReplay) {
    std::set<SuperNode*, SeedRankLess> s;
    std::set<SuperNode*> visited;
    std::vector<SuperNode*> prevSuper(sortedSuper);

    size_t prevSize = sortedSuper.size();
    for (SuperNode* node : sortedSuper) {
      if (node->depPrev.size() == 0) s.insert(node);
      times[node] = 0;
    }
    sortedSuper.clear();

    while(!s.empty()) {
      SuperNode* top = *s.begin();
      s.erase(s.begin());
      Assert(visited.find(top) == visited.end(), "superNode %d is already visited\n", top->id);
      visited.insert(top);
      sortedSuper.push_back(top);
      for (SuperNode* next : seedRankOrdered(top->depNext)) {
        times[next] ++;
        if (times[next] == (int)next->depPrev.size()) s.insert(next);
      }
    }
    Assert(sortedSuper.size() == prevSize, "invalid size %ld %ld\n", prevSize, sortedSuper.size());
    orderAllNodes();
    recordSeed2();
    return;
  }
  if (stableOrder) {
    std::set<SuperNode*, SuperNodeStableLess> s;
    std::set<SuperNode*> visited;
    std::vector<SuperNode*> prevSuper(sortedSuper);

    size_t prevSize = sortedSuper.size();
    for (SuperNode* node : sortedSuper) {
      if (node->depPrev.size() == 0) s.insert(node);
      times[node] = 0;
    }
    sortedSuper.clear();

    while(!s.empty()) {
      SuperNode* top = *s.begin();
      s.erase(s.begin());
      Assert(visited.find(top) == visited.end(), "superNode %d is already visited\n", top->id);
      visited.insert(top);
      sortedSuper.push_back(top);
      for (SuperNode* next : stableOrdered(top->depNext)) {
        times[next] ++;
        if (times[next] == (int)next->depPrev.size()) s.insert(next);
      }
    }
    Assert(sortedSuper.size() == prevSize, "invalid size %ld %ld\n", prevSize, sortedSuper.size());
    orderAllNodes();
    recordSeed2();
    return;
  }
  std::stack<SuperNode*> s;
  std::set<SuperNode*> visited;
  std::vector<SuperNode*> prevSuper(sortedSuper);

  size_t prevSize = sortedSuper.size();
  for (SuperNode* node : sortedSuper) {
    if (node->depPrev.size() == 0) s.push(node);
    times[node] = 0;
  }
  sortedSuper.clear();

  while(!s.empty()) {
    SuperNode* top = s.top();
    s.pop();
    Assert(visited.find(top) == visited.end(), "superNode %d is already visited\n", top->id);
    visited.insert(top);
    sortedSuper.push_back(top);
#ifdef ORDERED_TOPO_SORT
    std::vector<SuperNode*> sortedNext;
    sortedNext.insert(sortedNext.end(), top->depNext.begin(), top->depNext.end());
    std::sort(sortedNext.begin(), sortedNext.end(), [](SuperNode* a, SuperNode* b) {return a->id < b->id;});
    for (SuperNode* next : sortedNext) {
#else
    for (SuperNode* next : top->depNext) {
#endif
      times[next] ++;
      if (times[next] == (int)next->depPrev.size()) s.push(next);
    }
  }

  Assert(sortedSuper.size() == prevSize, "invalid size %ld %ld\n", prevSize, sortedSuper.size());
  orderAllNodes();
  recordSeed2();
}

// coarsen phase
// diagnostic: compact canonical graph fingerprint at pass boundaries. Per VALID
// super one record (member names in order + sorted prev/next/depPrev/depNext endpoint
// keys); records sorted in memory and stream-hashed (FNV-1a 64) — no files, no GB dumps.
// GSIM_DEBUG_CANON_HASH=1 enables; GSIM_DEBUG_CANON_STOP_AFTER=<tag> exits after a tag.
void graph::canonDumpTag(const char* tag) {
  if (!std::getenv("GSIM_DEBUG_CANON_HASH")) return;
  auto keyOf = [](SuperNode* e) {
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
  std::vector<std::string> records;
  records.reserve(sortedSuper.size());
  uint64_t orderHash = 1469598103934665603ULL;
  auto mixStr = [&](uint64_t h, const std::string& s) {
    for (unsigned char c : s) { h ^= c; h *= 1099511628211ULL; }
    return h;
  };
  for (SuperNode* super : sortedSuper) {
    if (super->superType != SUPER_VALID) continue;
    std::string rec = keyOf(super) + "|" + sortedEnds(super->prev) + "|" + sortedEnds(super->next) + "|" + sortedEnds(super->depPrev) + "|" + sortedEnds(super->depNext);
    orderHash = mixStr(orderHash, rec);
    records.push_back(std::move(rec));
  }
  std::sort(records.begin(), records.end());
  uint64_t recHash = 1469598103934665603ULL;
  for (const std::string& r : records) recHash = mixStr(recHash, r);
  fprintf(stderr, "[canon] %-24s records=%016zx order=%016zx count=%zu\n", tag, recHash, orderHash, records.size());
  if (const char* dumpDir = std::getenv("GSIM_DEBUG_CANON_DUMP")) {
    std::string p = std::string(dumpDir) + "/canon-" + tag + ".txt";
    FILE* df = std::fopen(p.c_str(), "w");
    if (df) { for (const std::string& r : records) std::fprintf(df, "%s\n", r.c_str()); std::fclose(df); }
  }
  const char* stop = std::getenv("GSIM_DEBUG_CANON_STOP_AFTER");
  if (stop && std::string(stop) == tag) { std::fprintf(stderr, "[canon] stop after %s\n", tag); std::exit(0); }
}

// ---- canonInputHash internals (arena record storage + exact stream cache) ----
namespace {
// Realloc-backed bump arena: unlike std::vector::resize (which value-initializes
// every grown region), appending never writes the destination twice and growth
// preserves content via realloc. Moves steal the buffer, so views stay valid.
struct CanonArena {
  char* buf = nullptr;
  size_t sz = 0;
  size_t cap = 0;
  CanonArena() = default;
  CanonArena(const CanonArena&) = delete;
  CanonArena& operator=(const CanonArena&) = delete;
  CanonArena(CanonArena&& o) noexcept : buf(o.buf), sz(o.sz), cap(o.cap) { o.buf = nullptr; o.sz = o.cap = 0; }
  CanonArena& operator=(CanonArena&& o) noexcept {
    if (this != &o) { std::free(buf); buf = o.buf; sz = o.sz; cap = o.cap; o.buf = nullptr; o.sz = o.cap = 0; }
    return *this;
  }
  ~CanonArena() { std::free(buf); }
  inline void ensure(size_t n) {
    if (sz + n <= cap) return;
    size_t newCap = cap ? cap * 2 : (size_t)1 << 20;
    while (newCap < sz + n) newCap *= 2;
    char* nb = (char*)std::realloc(buf, newCap);
    Assert(nb != nullptr, "canon arena realloc %zu failed", newCap);
    buf = nb;
    cap = newCap;
  }
  // p must not point into this arena (growth may reallocate it).
  inline void append(const char* p, size_t n) {
    ensure(n);
    if (n) std::memcpy(buf + sz, p, n);
    sz += n;
  }
  inline void append1(char c) {
    ensure(1);
    buf[sz ++] = c;
  }
};

struct CanonStreamCache {
  bool valid = false;
  uint64_t hash = 0;
  int algo = 0;                                  // canon algorithm that produced `hash` (1|2)
  std::vector<CanonArena> arenas;                // record byte storage
  std::vector<std::string_view> sorted;          // sorted views into arenas
  void reset() { valid = false; hash = 0; algo = 0; std::vector<CanonArena>().swap(arenas); std::vector<std::string_view>().swap(sorted); }
};
CanonStreamCache& canonStreamCache() { static CanonStreamCache c; return c; }

// Exact lexicographic order over views: memcmp semantics + shorter-prefix rule,
// identical to std::string ordering used by the original implementation.
bool canonViewLess(std::string_view a, std::string_view b) { return a.compare(b) < 0; }
bool canonViewEq(std::string_view a, std::string_view b) {
  return a.size() == b.size() && (a.size() == 0 || std::memcmp(a.data(), b.data(), a.size()) == 0);
}
}  // namespace

// ---- exact canon mixes (verification-only values; never touch model bytes) ----
// Both are pure functions of the sorted record view list.
namespace {
// The v1 basis/prime are frozen: every champion seed's recorded canon values were
// produced by these constants, so they must never change.
constexpr uint64_t CANON_FNV_BASIS = 1469598103934665603ULL;
constexpr uint64_t CANON_FNV_PRIME = 1099511628211ULL;
constexpr char CANON_V2_DOMAIN[] = "GSIM-CANON-V2";  // no NUL: 13 explicit bytes

inline void canonFnvBytes(uint64_t& h, const unsigned char* p, size_t n) {
  for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= CANON_FNV_PRIME; }
}
inline void canonFnvU64(uint64_t& h, uint64_t v) {
  for (int b = 0; b < 8; b++) { h ^= (unsigned char)(v >> (8 * b)); h *= CANON_FNV_PRIME; }
}
}  // namespace

// v1: serial FNV-1a over the whole sorted stream (verbatim legacy mix, ~0.8GB/s).
uint64_t graph::canonMixV1(const std::vector<std::string_view>& views) {
  uint64_t h = CANON_FNV_BASIS;
  for (std::string_view r : views) canonFnvBytes(h, (const unsigned char*)r.data(), r.size());
  return h;
}

// v2: parallel segmented hash (GSIM_SEED2_CANON=v2). Deterministic construction -
// the value depends only on the record bytes, never on the thread count:
//   S      = n == 0 ? 1 : min(4096, ceil(n / 256))   (segment count, from n alone)
//   seg_j  = views[n*j/S .. n*(j+1)/S)
//   h_j    = FNV-1a over LE64(segBytes_j) || LE64(segCount_j) || seg_j bytes
//   final  = FNV-1a over "GSIM-CANON-V2" || LE64(n) || LE64(S) || LE64(h_0) .. LE64(h_{S-1})
// (LE = fixed little-endian byte order; the domain tag keeps v2 values distinct
// from v1 values.) Each segment is hashed independently, so segments spread over
// workers; the combine is a fixed-order serial fold of the h_j sequence.
uint64_t graph::canonMixV2(const std::vector<std::string_view>& views) {
  const size_t n = views.size();
  const size_t S = n == 0 ? 1 : std::min<size_t>(4096, (n + 255) / 256);
  std::vector<uint64_t> seg(S, 0);
  auto hashSegment = [&](size_t j) {
    size_t lo = n * j / S, hi = n * (j + 1) / S;
    uint64_t bytes = 0;
    for (size_t i = lo; i < hi; i++) bytes += views[i].size();
    uint64_t h = CANON_FNV_BASIS;
    canonFnvU64(h, bytes);
    canonFnvU64(h, (uint64_t)(hi - lo));
    for (size_t i = lo; i < hi; i++)
      canonFnvBytes(h, (const unsigned char*)views[i].data(), views[i].size());
    seg[j] = h;
  };
  const int nWorkers = std::max(1, std::min<int>(std::min((int)S, 16),
                                                  (int)std::thread::hardware_concurrency()));
  std::atomic<size_t> next(0);
  auto workerLoop = [&]() {
    size_t j;
    while ((j = next.fetch_add(1)) < S) hashSegment(j);
  };
  std::vector<std::thread> pool;
  for (int w = 1; w < nWorkers; w++) pool.emplace_back(workerLoop);
  workerLoop();  // worker 0 runs inline
  for (std::thread& t : pool) t.join();
  uint64_t h = CANON_FNV_BASIS;
  canonFnvBytes(h, (const unsigned char*)CANON_V2_DOMAIN, sizeof(CANON_V2_DOMAIN) - 1);
  canonFnvU64(h, (uint64_t)n);
  canonFnvU64(h, (uint64_t)S);
  for (uint64_t v : seg) canonFnvU64(h, v);
  return h;
}

uint64_t graph::canonInputHash() {
  PhaseTimer hashTotal("canonInputHash.total");
  // Which exact mix to run: v1 (frozen serial FNV-1a, the only value every
  // existing champion seed ever recorded) or v2 (parallel segmented, opt-in via
  // GSIM_SEED2_CANON=v2 on writes; on replay the next point's recorded tag
  // decides, so v1 seeds keep computing v1 and mixed seeds replay correctly).
  const int canonAlgo = mtSeed2CanonAlgo();
  auto tBuild = phasetimer::now();
  // Record byte image (unchanged from the original construction): member names in
  // order + '|' + sorted prev/next/depPrev/depNext endpoint keys, ',' after each key.
  // Storage moved from std::string (one allocation per record + realloc-copies while
  // growing to the multi-KB records seen at coarsen points; 145GB stream at the
  // out1 pin measured 27s of growth memcpy) to a per-worker bump arena addressed by
  // (offset,len) views. Sorting/mixing consume the identical byte stream, so the
  // hash value is unchanged.
  const int nWorkers = std::min(16, (int)std::thread::hardware_concurrency());
  std::vector<SuperNode*> valid;
  valid.reserve(sortedSuper.size());
  for (SuperNode* super : sortedSuper) if (super->superType == SUPER_VALID) valid.push_back(super);
  size_t total = valid.size();
  std::vector<CanonArena> arenas((size_t)nWorkers);
  std::vector<CanonArena> keyArenas((size_t)nWorkers);
  std::vector<std::vector<std::pair<size_t, size_t>>> recRanges((size_t)nWorkers);  // (offset, len)
  std::vector<std::thread> pool;
  for (int w = 0; w < nWorkers; w ++) {
    size_t lo = total * (size_t)w / (size_t)nWorkers;
    size_t hi = total * (size_t)(w + 1) / (size_t)nWorkers;
    pool.emplace_back([&, w, lo, hi]() {
      CanonArena& arena = arenas[(size_t)w];          // record bytes
      CanonArena& keyArena = keyArenas[(size_t)w];    // endpoint-key bytes only
      std::vector<std::pair<size_t, size_t>>& ranges = recRanges[(size_t)w];
      // Records and endpoint keys live in separate arenas: caching a NEW key appends
      // to keyArena only, so it can never land inside a record range mid-build, and
      // growth of either arena cannot invalidate the other's bytes.
      // Endpoint-key memo in keyArena coordinates: a super's key is embedded by every
      // record that lists it as an end; append the bytes once and reference forever.
      std::unordered_map<SuperNode*, std::pair<size_t, size_t>> keyCache;
      std::vector<std::pair<size_t, size_t>> endKeyRanges;
      auto keyRangeOf = [&](SuperNode* e) {
        auto it = keyCache.find(e);
        if (it == keyCache.end()) {
          size_t beg = keyArena.sz;
          for (Node* m : e->member) { keyArena.append(m->name.data(), m->name.size()); keyArena.append1(';'); }
          it = keyCache.emplace(e, std::make_pair(beg, keyArena.sz - beg)).first;
        }
        return it->second;
      };
      auto appendEnds = [&](const std::set<SuperNode*>& ends) {
        endKeyRanges.clear();
        for (SuperNode* e : ends) endKeyRanges.push_back(keyRangeOf(e));
        const CanonArena& a = keyArena;
        std::sort(endKeyRanges.begin(), endKeyRanges.end(), [&a](const auto& x, const auto& y) {
          size_t n = std::min(x.second, y.second);
          int c = n ? std::memcmp(a.buf + x.first, a.buf + y.first, n) : 0;
          if (c != 0) return c < 0;
          return x.second < y.second;
        });
        for (const auto& k : endKeyRanges) { arena.append(keyArena.buf + k.first, k.second); arena.append1(','); }
      };
      for (size_t i = lo; i < hi; i ++) {
        SuperNode* super = valid[i];
        size_t beg = arena.sz;
        for (Node* m : super->member) { arena.append(m->name.data(), m->name.size()); arena.append1(';'); }
        arena.append1('|');
        appendEnds(super->prev);
        arena.append1('|');
        appendEnds(super->next);
        arena.append1('|');
        appendEnds(super->depPrev);
        arena.append1('|');
        appendEnds(super->depNext);
        ranges.emplace_back(beg, arena.sz - beg);
      }
    });
  }
  for (std::thread& t : pool) t.join();
  // Fresh arenas can invalidate cached views if the cache aliases freed memory;
  // the cache owns its arenas, so this ordering is safe (new arenas are separate).
  std::vector<std::string_view> views;
  views.reserve(total);
  for (int w = 0; w < nWorkers; w ++) {
    const CanonArena& arena = arenas[(size_t)w];
    for (const auto& r : recRanges[(size_t)w]) views.emplace_back(arena.buf + r.first, r.second);
  }
  std::vector<CanonArena>().swap(keyArenas);  // key bytes are dead once records exist
  phasetimer::mark("canonInputHash.build", tBuild);
  auto tSort = phasetimer::now();
  std::sort(views.begin(), views.end(), canonViewLess);
  phasetimer::mark("canonInputHash.sort", tSort);
  // Exact stream cache: if the sorted view list is byte-for-byte equal to the
  // previous computation's AND was hashed with the same algorithm, the value is
  // identical by construction, so the cached hash is returned without re-running
  // the mix. The equality check is a full compare - exactly as strong as rehashing.
  CanonStreamCache& cache = canonStreamCache();
  if (cache.valid && cache.algo == canonAlgo && cache.sorted.size() == views.size()) {
    bool equal = true;
    size_t n = views.size();
    int cmpWorkers = nWorkers;
    std::vector<int> diffs((size_t)cmpWorkers, 0);
    std::vector<std::thread> cmpPool;
    for (int w = 0; w < cmpWorkers; w ++) {
      size_t lo = n * (size_t)w / (size_t)cmpWorkers;
      size_t hi = n * (size_t)(w + 1) / (size_t)cmpWorkers;
      cmpPool.emplace_back([&views, &cache, lo, hi, &diffs, w]() {
        for (size_t i = lo; i < hi; i ++) {
          if (!canonViewEq(views[i], cache.sorted[i])) { diffs[(size_t)w] = 1; return; }
        }
      });
    }
    for (std::thread& t : cmpPool) t.join();
    for (int d : diffs) equal = equal && d == 0;
    if (equal) {
      phasetimer::mark("canonInputHash.cacheHit", tSort);
      uint64_t h = cache.hash;
      auto tDtor = phasetimer::now();
      for (CanonArena& a : arenas) a = CanonArena();
      phasetimer::mark("canonInputHash.dtor", tDtor);
      return h;
    }
  }
  auto tMix = phasetimer::now();
  size_t mixBytes = 0;
  for (std::string_view r : views) mixBytes += r.size();
  uint64_t recHash = canonAlgo == 2 ? canonMixV2(views) : canonMixV1(views);
  phasetimer::mark("canonInputHash.mix", tMix);
  if (phasetimer::enabled()) fprintf(stderr, "[gpart-phase] canonInputHash.bytes = %zu (%.2f GB, %zu records) algo=%d\n", mixBytes, (double)mixBytes / 1e9, views.size(), canonAlgo);
  auto tDtor = phasetimer::now();
  cache.reset();
  cache.valid = true;
  cache.algo = canonAlgo;
  cache.hash = recHash;
  cache.arenas = std::move(arenas);
  cache.sorted = std::move(views);
  phasetimer::mark("canonInputHash.dtor", tDtor);
  return recHash;
}

void graph::canonSeed2Record(const char* tag) { mtSeed2RecordPoint(tag, sortedSuper, canonInputHash()); }
void graph::canonSeed2Apply(const char* tag) { mtSeed2ApplyPoint(tag, sortedSuper, mtSeed2CanonVerifyEnabled() ? canonInputHash() : 0); orderAllNodes(); }

// Pre-topoSort variant over supersrc (sortedSuper is empty before topoSort).
// Same record construction so hashes are comparable with canonInputHash.
uint64_t graph::canonRawHash() {
  auto keyOf = [](SuperNode* e) {
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
  auto mixStr = [](uint64_t h, const std::string& s) {
    for (unsigned char c : s) { h ^= c; h *= 1099511628211ULL; }
    return h;
  };
  std::vector<std::string> records;
  records.reserve(supersrc.size());
  for (SuperNode* super : supersrc) {
    if (super->superType != SUPER_VALID) continue;
    records.push_back(keyOf(super) + "|" + sortedEnds(super->prev) + "|" + sortedEnds(super->next) + "|" + sortedEnds(super->depPrev) + "|" + sortedEnds(super->depNext));
  }
  std::sort(records.begin(), records.end());
  uint64_t recHash = 1469598103934665603ULL;
  for (const std::string& r : records) recHash = mixStr(recHash, r);
  return recHash;
}

void graph::graphCoarsen() {
  canonDumpTag("coarsen.entry");
  {
    PhaseTimer t("coarsen.mergeResetAll");
    mergeResetAll();
  }
  canonDumpTag("coarsen.resetAll");

  {
    PhaseTimer t("coarsen.mergeWhenNodes");
    mergeWhenNodes();
  }
  {
    PhaseTimer t("coarsen.resortWhen");
    resort("coarsen.when.resort");
  }
  canonDumpTag("coarsen.when");

  // pin every order-consuming pass outcome, not just resorts. mergeOut1/In1/
  // Sublings mutate content AND order; recording the post-pass sortedSuper (with canon
  // verification) closes the gap where run-to-run content variance was observed
  // (partition.postCoarsen canon 70725c vs 2237bc across identical write runs).
  const bool seed2Write = mtSeed2WriteActive();
  const bool seed2Replay = mtSeed2ReplayActive();
  auto pinSeed2 = [&](const char* tag) {
    PhaseName pn("pin", tag);
    PhaseTimer t(pn.c_str());
    if (seed2Write) mtSeed2RecordPoint(tag, sortedSuper, canonInputHash());
    if (seed2Replay) { mtSeed2ApplyPoint(tag, sortedSuper, mtSeed2CanonVerifyEnabled() ? canonInputHash() : 0); orderAllNodes(); }
  };

  {
    PhaseTimer t("coarsen.mergeOut1");
    mergeOut1();
  }
  pinSeed2("coarsen.out1");
  canonDumpTag("coarsen.out1");
  {
    PhaseTimer t("coarsen.mergeIn1");
    mergeIn1();
  }
  pinSeed2("coarsen.in1");
  canonDumpTag("coarsen.in1");
  {
    PhaseTimer t("coarsen.mergeSublings");
    mergeSublings();
  }
  pinSeed2("coarsen.sublings");
  canonDumpTag("coarsen.sublings");
}

// initial partition

void graph::graphInitPartition() {
  printf("[graphPartition] Setting the maximum size of a superNode to %d\n", globalConfig.SuperNodeMaxSize);
  /*
  Kernighan’s algorithm: new part start at x
  * cost：C(x) = sum(i<x<=j)(cij)
  * T(x): partial cost 
    - T(1) = 0
    - T(x) = C(x) + T(y)
  * dynamic programming
  */
  std::vector<int> T(sortedSuper.size() + 1, INT_MAX);
  std::vector<int> b(sortedSuper.size() + 1, -1); // backtrace
  // Note: the historical C[]/internal[] precompute (edge-span cost + prefix sum)
  // was dead - the DP below rebuilds its cost term inline (Cij) and nothing read
  // C or internal. Removed after decomposition showed it at 17s/pass.
  auto tDP = phasetimer::now();
  T[0] = 0;
  /* compute T by dynamic programming */
  for (size_t i = 0; i < sortedSuper.size(); i ++) {
    // printf("T[%ld] = %d size %ld\n", i, T[i], sortedSuper[i]->member.size());
    size_t nextBound = i + 1;
    size_t accuCost = sortedSuper[i]->member.size();
    for (; nextBound < sortedSuper.size() && accuCost + sortedSuper[nextBound]->member.size() <= globalConfig.SuperNodeMaxSize; nextBound ++) {
      accuCost += sortedSuper[nextBound]->member.size();
    }
    /* update T[i + 1] to T[nextBound] that jmp at i */
    int Cij = 0;
    for (size_t j = i + 1; j <= nextBound; j ++) {
      Cij += sortedSuper[j - 1]->next.size();
      for (SuperNode* prev : sortedSuper[j - 1]->prev) {
        if (prev->order >= (int)i) Cij --;
      }
      int newT = T[i] + Cij;
      if(T[j] > newT) {
        T[j] = newT;
        b[j] = i;
      }
    }
  }
  phasetimer::mark("initPart.dp", tDP);
  auto tCut = phasetimer::now();
  std::set<int> cut;
  int idx = sortedSuper.size();
  while (b[idx] > 0) {
    cut.insert(b[idx]);
    idx = b[idx];
  }
  cut.insert(sortedSuper.size());
  for (size_t i = 0; i < sortedSuper.size(); i ++) {
    if (sortedSuper[i]->superType != SUPER_VALID) {
      cut.insert(i);
      if (i + 1 < sortedSuper.size()) cut.insert(i + 1);
    }
  }
  // for (int i : cut) printf("cut %d\n", i);
  
  int cutBeg = 0;
  for (int cutEnd : cut) {
    SuperNode* master = sortedSuper[cutBeg];
    for (int i = cutBeg + 1; i < cutEnd; i ++) {
      SuperNode* memberSuper = sortedSuper[i];
      for (Node* node : memberSuper->member) {
        node->super = master;
      }
      master->member.insert(master->member.end(), memberSuper->member.begin(), memberSuper->member.end());
      memberSuper->member.clear();
    }
    cutBeg = cutEnd;
  }
  phasetimer::mark("initPart.cutApply", tCut);
  auto tCleanup = phasetimer::now();
  removeEmptySuper();
  reconnectSuper();
  phasetimer::mark("initPart.cleanup", tCleanup);
}


#define REFINE_TYPE std::tuple<Node*, SuperNode*, int>
#define REFINE_NODE(gain) std::get<0>(gain)
#define REFINE_DST(gain) std::get<1>(gain)
#define REFINE_GAIN(gain) std::get<2>(gain)
struct gainLess { // ascending order
  bool operator()(REFINE_TYPE n1, REFINE_TYPE n2) {
    return REFINE_GAIN(n1) < REFINE_GAIN(n2);
  }
};
static std::priority_queue<REFINE_TYPE, std::vector<REFINE_TYPE>, gainLess> gainQueue;
static std::vector<Node*> allGainNodes;
static std::map<Node*, std::pair<SuperNode*, int>> incomingMap;
static std::map<Node*, std::pair<SuperNode*, int>> outcomingMap;
static std::map<Node*, int> anyExtEdge;

void addGainQueue(Node* node, SuperNode* dst, int gain) {
  gainQueue.push(std::make_tuple(node, dst, gain));
}

void updateExtEdge(Node* node) {
  for (Node* next : node->next) {
    if (node->super != next->super) {
      anyExtEdge[node] = 1;
      break;
    }
  }
}

int nodeGain(Node* node, SuperNode* dst) {
/* number of edge cut */
  Assert(node->super != dst, "invalid gain %s\n", node->name.c_str());
  int gain = 0;
  for (Node* prev : node->prev) {
    if (prev->super == dst) gain ++;
    else if (prev->super == node->super) gain --;
  }
  for (Node* next : node->next) {
    if (next->super == dst) gain ++;
    else if (next->super == node->super) gain --;
  }

/* number of nodes that has extEdge */
  int boundaryNum = 0;
  /* prev may become boundary */
  for (Node* prev : node->prev) {
    if (node->super == prev->super && anyExtEdge[prev] == 0) boundaryNum ++;
  }
  bool nodeIsBoundary = false;
  for (Node* next : node->next) {
    if (node->super != next->super) {
      nodeIsBoundary = true;
    }
  }
  if (nodeIsBoundary) boundaryNum --;
  gain -= boundaryNum * 2;
  return gain;
}

REFINE_TYPE popGainQueue() {
  REFINE_TYPE ret = gainQueue.top();
  gainQueue.pop();
  return ret;
}

bool validGainEmpty() {
  while (!gainQueue.empty()) {
    REFINE_TYPE ret = gainQueue.top();
    if (incomingMap.find(REFINE_NODE(ret)) != incomingMap.end()
        && incomingMap[REFINE_NODE(ret)].first == REFINE_DST(ret)
        && incomingMap[REFINE_NODE(ret)].second == REFINE_GAIN(ret)) {
      break;
    }
    if (outcomingMap.find(REFINE_NODE(ret)) != outcomingMap.end()
        && outcomingMap[REFINE_NODE(ret)].first == REFINE_DST(ret)
        && outcomingMap[REFINE_NODE(ret)].second == REFINE_GAIN(ret)) {
      break;
    }
    popGainQueue();
  }
  return gainQueue.empty();
}

SuperNode* checkIncoming(Node* node) {
  bool isInComing = true;
  Node* maxNode = nullptr;
  int maxOrder = -1;
  for (Node* prev : node->prev) {
    if (prev->super->order == node->super->order) {
      isInComing = false;
      break;
    } else {
      if (prev->super->order > maxOrder) {
        maxOrder = prev->super->order;
        maxNode = prev;
      }
    }
  }
  if (isInComing && maxNode && maxNode->super->superType == SUPER_VALID) return maxNode->super;
  return nullptr;
}

SuperNode* checkOutcoming(Node* node) {
  bool isOutComing = true;
  Node* minNode = nullptr;
  int minOrder = -1;
  for (Node* next : node->next) {
    if (next->super->order == node->super->order) {
      isOutComing = false;
      break;
    } else {
      if (next->super->order < minOrder) {
        minOrder = next->super->order;
        minNode = next;
      }
    }
  }
  if (isOutComing && minNode && minNode->super->superType == SUPER_VALID) return minNode->super;
  return nullptr;
}

void addIncoming(Node* node, SuperNode* incomingNode) {
  int gain = nodeGain(node, incomingNode);
  addGainQueue(node, incomingNode, gain);
  incomingMap[node] = std::make_pair(incomingNode, gain);
}

void addOutcoming(Node* node, SuperNode* outcomingNode) {
  int gain = nodeGain(node, outcomingNode);
  addGainQueue(node, outcomingNode, gain);
  outcomingMap[node] = std::make_pair(outcomingNode, gain);
}

// refine & uncoarsen phase
void graph::graphRefine() {
  /* initial gain for each incoming and outcoming nodes */
  for (SuperNode* super : sortedSuper) {
    for (Node* node : super->member) {
      allGainNodes.push_back(node);
    }
  }
  for (SuperNode* super : sortedSuper) {
    if (super->superType != SUPER_VALID) continue;
    for (Node* node : super->member) {
      /* check incoming */
      SuperNode* incomingNode = checkIncoming(node);
      if (incomingNode) addIncoming(node, incomingNode);
      /* check outcoming */
      SuperNode* outcomingNode = checkOutcoming(node);
      if (outcomingNode) addOutcoming(node, outcomingNode);
    }
  }
  /* move */
  int refineNum = 0;
  while (!validGainEmpty()) {
    refineNum ++;
    Node* node;
    SuperNode* dst;
    int gain;
    std::tie(node, dst, gain) = gainQueue.top();
    popGainQueue();
    if (gain <= 1) break;
    if (dst == node->super) continue;

    for (Node* prev : node->prev) { // previous node can become outcoming node or dst may change
      /* prev is no longer outcoming */
      if (prev->super == dst && outcomingMap.find(prev) != outcomingMap.end()) {
        outcomingMap.erase(prev);
        continue;
      }
      /* prev may become outcoming */
      if (node->super == prev->super) {
        SuperNode* outcoming = checkOutcoming(prev);
        if (outcoming) addOutcoming(prev, outcoming);
        continue;
      }
      if (outcomingMap.find(prev) != outcomingMap.end()) {
        SuperNode* outcoming = checkOutcoming(prev);
        if (outcoming) addOutcoming(prev, outcoming);
      }  
    }
  
    for (Node* next : node->next) {
      /* next is no longer incoming */
      if (next->super == dst && incomingMap.find(next) != incomingMap.end()) {
        incomingMap.erase(next);
        continue;
      }
      /* next may become incoming */
      if (node->super == next->super) {
        SuperNode* incoming = checkIncoming(next);
        if (incoming) addIncoming(next, incoming);
        continue;
      }
      /* update gain & dst */
      if (incomingMap.find(next) != incomingMap.end()) {
        SuperNode* incoming = checkIncoming(next);
        if (incoming) addIncoming(next, incoming);
      }
    }
    /* move node to dst */
    SuperNode* prevSuper = node->super;
    node->super = dst;
    for (Node* next : node->next) {
      if (next->super != dst) {
        anyExtEdge[node] = 1;
      }
    }
    for (Node* prev : node->prev) {
      if (prev->super == prevSuper) anyExtEdge[prev] = 1;
      else if(prev->super == dst) updateExtEdge(prev);
    }
  }
  /* construct super relationship */
  for (SuperNode* super : sortedSuper) super->member.clear();
  for (Node* node : allGainNodes) {
    node->super->member.push_back(node);
  }
  removeEmptySuper();
  reconnectSuper();
  printf("refine %d times\n", refineNum);
}
void graph::graphPartition() {
  PhaseTimer gpartTotal("gpart.total");
  size_t totalSuper = sortedSuper.size();
  size_t phaseSuper = sortedSuper.size();
  {
    PhaseTimer t("gpart.orderAllNodes.entry");
    orderAllNodes();
  }

/* coarsen phase */
  {
    PhaseTimer t("gpart.coarsen");
    graphCoarsen();
  }
  {
    PhaseTimer t("gpart.postCoarsenResort");
    resort("partition.postCoarsen.resort");
  }
  printf("[graphCoarsen] remove %ld superNodes (%ld -> %ld)\n", phaseSuper - sortedSuper.size(), phaseSuper, sortedSuper.size());

/* initial partition */
  phaseSuper = sortedSuper.size();
  {
    PhaseTimer t("gpart.initPartition");
    graphInitPartition();
  }
  {
    PhaseTimer t("pin.partition.init");
    if (mtSeed2WriteActive()) mtSeed2RecordPoint("partition.init", sortedSuper, canonInputHash());
    if (mtSeed2ReplayActive()) { mtSeed2ApplyPoint("partition.init", sortedSuper, mtSeed2CanonVerifyEnabled() ? canonInputHash() : 0); orderAllNodes(); }
  }
  canonDumpTag("partition.init");
  {
    PhaseTimer t("gpart.orderAllNodes.exit");
    orderAllNodes();
  }
  printf("[InitPartition] remove %ld superNodes (%ld -> %ld)\n", phaseSuper - sortedSuper.size(), phaseSuper, sortedSuper.size());
/* refine & uncoarsen phase */
  // graphRefine();
  phaseSuper = sortedSuper.size();
  canonDumpTag("partition.post");
  printf("[graphPartition] remove %ld superNodes (%ld -> %ld)\n", totalSuper - phaseSuper, totalSuper, phaseSuper);
}
