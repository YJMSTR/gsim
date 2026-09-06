/*
  cppEmitter: emit C++ files for simulation
*/

#include "cppEmitterImpl.h"
#ifdef DIFFTEST_PER_SIG
FILE* sigFile = nullptr;
#endif

static bool generatedOutputFilesEqual(const std::string& lhsPath, const std::string& rhsPath) {
  std::ifstream lhs(lhsPath, std::ios::binary);
  std::ifstream rhs(rhsPath, std::ios::binary);
  if (!lhs.good() || !rhs.good()) return false;
  static constexpr size_t kBufferSize = 1 << 16;
  std::vector<char> lhsBuffer(kBufferSize);
  std::vector<char> rhsBuffer(kBufferSize);
  while (lhs.good() || rhs.good()) {
    lhs.read(lhsBuffer.data(), static_cast<std::streamsize>(lhsBuffer.size()));
    rhs.read(rhsBuffer.data(), static_cast<std::streamsize>(rhsBuffer.size()));
    std::streamsize lhsCount = lhs.gcount();
    std::streamsize rhsCount = rhs.gcount();
    if (lhsCount != rhsCount) return false;
    if (lhsCount == 0) break;
    if (!std::equal(lhsBuffer.begin(), lhsBuffer.begin() + lhsCount, rhsBuffer.begin())) return false;
  }
  return true;
}


static void commitStableOutputFile(const std::string& tmpPath, const std::string& finalPath) {
  if (tmpPath.empty()) return;
  if (generatedOutputFilesEqual(tmpPath, finalPath)) {
    // Never embed filesystem side effects in assert(): under -DNDEBUG the call is
    // compiled out and the tmp file would never be removed/installed.
    int rc = std::remove(tmpPath.c_str());
    Assert(rc == 0, "failed to remove identical tmp output %s", tmpPath.c_str());
    return;
  }
  int rc = std::rename(tmpPath.c_str(), finalPath.c_str());
  Assert(rc == 0, "failed to install stable output %s -> %s", tmpPath.c_str(), finalPath.c_str());
}

static std::vector<std::vector<int>> mtStepActiveWordGuards;
static std::vector<char> mtStepActiveWordGuardable;



// Read-only view of super2ResetId with std::map::operator[] value semantics:
// returns the value-initialized pair a first operator[] access would have
// inserted, without mutating the shared map (parallel emission units call this
// concurrently; genResetAll has already populated every live key by then).
static const std::pair<int, int>& super2ResetIdLookup(Node* resetNode) {
  static const std::pair<int, int> kDefault = {0, 0};
  auto iter = super2ResetId.find(resetNode);
  return iter == super2ResetId.end() ? kDefault : iter->second;
}







static void mtDenseAddEdge(MtDenseSchedule& schedule,
                           std::vector<std::set<int>>& succSets,
                           std::vector<std::set<int>>& predSets,
                           std::set<std::tuple<int, int, std::string>>& edgeKinds,
                           int fromCppId, int toCppId, const std::string& kind) {
  if (fromCppId < 0 || toCppId < 0 || fromCppId >= schedule.taskCount || toCppId >= schedule.taskCount) return;
  if (fromCppId == toCppId) return;
  std::tuple<int, int, std::string> key(fromCppId, toCppId, kind);
  if (!edgeKinds.insert(key).second) return;
  MtDenseEdge edge;
  edge.fromCppId = fromCppId;
  edge.toCppId = toCppId;
  edge.kind = kind;
  schedule.edges.push_back(edge);
  if (kind == "dependency") schedule.dependencyEdgeCount ++;
  else if (kind == "active") schedule.activeEdgeCount ++;
  else if (kind == "active_back") schedule.activeEdgeCount ++;
  else if (kind == "need_activate") schedule.needActivateEdgeCount ++;
  if (succSets[(size_t)fromCppId].insert(toCppId).second) {
    predSets[(size_t)toCppId].insert(fromCppId);
  }
}

static void mtDenseAddSuperEdges(MtDenseSchedule& schedule,
                                 std::vector<std::set<int>>& succSets,
                                 std::vector<std::set<int>>& predSets,
                                 std::set<std::tuple<int, int, std::string>>& edgeKinds,
                                 int fromCppId, const std::set<SuperNode*>& supers,
                                 const std::string& kind) {
  for (SuperNode* super : supers) {
    if (super && super->cppId >= 0) {
      mtDenseAddEdge(schedule, succSets, predSets, edgeKinds, fromCppId, super->cppId, kind);
    }
  }
}

// faithful Verilator V3OrderParallel edge contraction on the SCC DAG. The key detail that
// makes contraction tractable (missed by budget-limited BFS that gave 3.2M false cycle
// rejections) is Verilator's CP-bound-pruned, generation-tagged cycle check: a path fromp~>top
// cannot exist if fromp.cpRev < top.cpRev+top.step OR fromp.cpFwd+fromp.step > top.cpFwd, so most
// checks resolve in O(1). Merges the lowest edgeScore (merged local critical path) until liveCount
// <= maxMTasks (=50*threads) or scoreLimit exceeded. worker0-only SCCs never merge with non-worker0.
// Emits merged MTasks in Kahn topo order so ids are topo-monotone.
// Legacy step cost (was a per-call loop; see the trajectory table in
// mtBuildDenseMTasksVerilatorContract for the exact same values, table-driven):
//   uint64_t mtDenseStepCostV(uint64_t c) {
//     if (c <= 1) return c;
//     uint64_t s = 1; while (s < c) s = s + s / 20 + 1; return s;
//   }

static void mtBuildDenseScheduleOrder(const std::vector<MtDenseMTask>& mtasks, int threadCount, std::vector<int>& assign, std::vector<int>& order);
// ---- VCONTRACT_POLICY=auto pass control (file scope; recomputeCP's compaction
// gate and the two-pass driver in mtaskBuild coordinate through these) ----
static int gVcAutoPass = 0;          // 0 = plain pass, 1 = compact pass
static void vcAutoPassReset() { gVcAutoPass = 0; }
static void vcAutoPassAdvance() { gVcAutoPass = 1; }
static void vcAutoPassFinish(bool pickCompact) { gVcAutoPass = pickCompact ? 1 : 0; }
// Scoring assignment: the REAL dependency-aware list scheduler (same function
// the emission path uses). A free-form LPT was tried first and erased exactly
// the term that decides T16 (order/dependency-constrained imbalance: measured
// plain maxW=47234 vs LPT's optimistic 34267) - the model then collapsed to
// "fewer edges always wins" and picked wrong.
static void vcAutoAssign(MtDenseSchedule& sched, int threadCount) {
  std::vector<int> assignTmp, orderTmp;
  mtBuildDenseScheduleOrder(sched.mtasks, threadCount, assignTmp, orderTmp);
  sched.mtaskThreadAssign = assignTmp;
}

static std::vector<MtDenseMTask> mtBuildDenseMTasksVerilatorContract(const MtDenseSchedule& schedule,
                                                                     int threadCount) {
  const int realN = static_cast<int>(schedule.sccs.size());
  std::vector<MtDenseMTask> mtasks;
  if (realN == 0) return mtasks;
  if (threadCount < 1) threadCount = 1;
  const int n = realN;
  const auto denseNode = [](int scc) { return scc; };
  auto sc = [&](int s) -> uint64_t { return (uint64_t)std::max(1, schedule.sccs[(size_t)s].memberNodeCost); };
  const uint64_t totalCost = [&]{ uint64_t c = 0; for (int scc = 0; scc < realN; ++scc) c += sc(scc); return c; }();
  // Legacy stepCost calls mtDenseStepCostV per query (a ~log_{1.05}(cost)-iteration loop with a
  // division); scoring does this ~10M times per T16 contraction. Precompute the fixed trajectory
  // s_{k+1} = s_k + s_k/20 + 1 once (O(log) entries up to totalCost - every queried cost is a
  // group cost or pair sum, both <= totalCost) and answer by binary search. Identical values by
  // construction: the table IS the loop's trajectory, so decisions and output are unchanged.
  std::vector<uint64_t> legacyStepSeq;
  legacyStepSeq.push_back(1);
  while (legacyStepSeq.back() < totalCost) {
    uint64_t s = legacyStepSeq.back();
    legacyStepSeq.push_back(s + s / 20 + 1);
  }
  const auto stepCost = [&legacyStepSeq](uint64_t cost) -> uint64_t {
    if (cost <= 1) return cost;
    return *std::lower_bound(legacyStepSeq.begin(), legacyStepSeq.end(), cost);
  };
  // GSIM_EMIT_PHASE_TIMING=1 sub-phase accounting (pure measurement; zero
  // emitted-byte impact). Leaf timers (recomputeCP / pathExists /
  // refreshBestTwo / rebuildPQ) are inclusive wherever they nest inside other
  // timed regions - reported calls make the overlap interpretable.
  const bool vcPhaseTiming = emitPhaseTimingEnabled();
  EmitPhaseAccum vcAccInit{"Final.vcontract.init"};
  EmitPhaseAccum vcAccLoop{"Final.vcontract.mergeLoop"};
  EmitPhaseAccum vcAccRecompute{"Final.vcontract.recomputeCP"};
  EmitPhaseAccum vcAccPath{"Final.vcontract.pathExists"};
  EmitPhaseAccum vcAccBestTwo{"Final.vcontract.refreshBestTwo"};
  EmitPhaseAccum vcAccRebuildPQ{"Final.vcontract.rebuildPQ"};
  EmitPhaseAccum vcAccMergeApply{"Final.vcontract.mergeApply"};
  EmitPhaseAccum vcAccMaterialize{"Final.vcontract.materialize"};
  const std::chrono::steady_clock::time_point vcInitBegin =
      vcPhaseTiming ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point();
  std::vector<int> uf((size_t)n); for (int i = 0; i < n; i ++) uf[(size_t)i] = i;
  std::function<int(int)> find = [&](int x){ while (uf[(size_t)x]!=x){ uf[(size_t)x]=uf[(size_t)uf[(size_t)x]]; x=uf[(size_t)x]; } return x; };
  std::vector<uint64_t> gcost((size_t)n), gF((size_t)n, 0), gR((size_t)n, 0);
  // Relation sets as SORTED FLAT VECTORS (was std::set<int>): the merge loop, candidate scoring
  // and cycle DFS walk these millions of times; sequential scans replace red-black-tree pointer
  // chases. Sorted order == std::set order, so every traversal visits neighbors in exactly the
  // same sequence as before - identical decisions, byte-identical output.
  std::vector<std::vector<int>> gS((size_t)n), gP((size_t)n);
  const auto relContains = [](const std::vector<int>& v, int x) -> bool {
    return std::binary_search(v.begin(), v.end(), x);
  };
  const auto relInsert = [](std::vector<int>& v, int x) {
    auto it = std::lower_bound(v.begin(), v.end(), x);
    if (it == v.end() || *it != x) v.insert(it, x);
  };
  const auto relErase = [](std::vector<int>& v, int x) {
    auto it = std::lower_bound(v.begin(), v.end(), x);
    if (it != v.end() && *it == x) v.erase(it);
  };
  // Sorted-union of two sorted unique spans via a reusable out-buffer + O(1) swap (the
  // out-buffer's allocation recycles across merges; no per-merge allocation, no
  // in-place-merge overlap hazards). Result equals the former relInsert-per-edge sequence.
  std::vector<int> relUnionOut;
  const auto relUnionInto = [](std::vector<int>& dst, const std::vector<int>& sortedUniqueSrc, std::vector<int>& out) {
    out.clear();
    out.reserve(dst.size() + sortedUniqueSrc.size());
    std::set_union(dst.begin(), dst.end(), sortedUniqueSrc.begin(), sortedUniqueSrc.end(), std::back_inserter(out));
    dst.swap(out);
  };
  std::vector<bool> gw0((size_t)n);
  for (int scc = 0; scc < realN; ++scc) { int node = denseNode(scc); gcost[(size_t)node] = sc(scc); gw0[(size_t)node] = schedule.sccs[(size_t)scc].workerZeroOnly; }
  for (int u = 0; u < realN; ++u) for (int v : schedule.sccs[(size_t)u].succSccs) if (v != u && v >= 0 && v < realN) { int from = denseNode(u), to = denseNode(v); gS[(size_t)from].push_back(to); gP[(size_t)to].push_back(from); }
  for (int i = 0; i < n; i ++) { std::sort(gS[(size_t)i].begin(), gS[(size_t)i].end()); gS[(size_t)i].erase(std::unique(gS[(size_t)i].begin(), gS[(size_t)i].end()), gS[(size_t)i].end()); std::sort(gP[(size_t)i].begin(), gP[(size_t)i].end()); gP[(size_t)i].erase(std::unique(gP[(size_t)i].begin(), gP[(size_t)i].end()), gP[(size_t)i].end()); }
  // Forward (to-end) and reverse (from-start) stepped critical paths over the SCC DAG (topo by id).
  for (int u = n - 1; u >= 0; u --) { uint64_t b = 0; for (int v : gS[(size_t)u]) b = std::max(b, gF[(size_t)v] + stepCost(gcost[(size_t)v])); gF[(size_t)u] = b; }
  for (int u = 0; u < n; u ++) { uint64_t b = 0; for (int p : gP[(size_t)u]) b = std::max(b, gR[(size_t)p] + stepCost(gcost[(size_t)p])); gR[(size_t)u] = b; }
  int maxMTasks = std::max(threadCount, 50 * threadCount);
  { const char* e = std::getenv("GSIM_MT_DENSE_VCONTRACT_MAXMT"); if (e && e[0]) { int v = std::atoi(e); if (v > threadCount) maxMTasks = v; } }
  int capMul = 3;
  { const char* e = std::getenv("GSIM_MT_DENSE_VCONTRACT_CAP"); if (e && e[0]) { int v = std::atoi(e); if (v >= 1) capMul = v; } }
  const uint64_t legacyMTaskCap = std::max<uint64_t>(1, (totalCost / (uint64_t)maxMTasks) * (uint64_t)capMul);
  const uint64_t perMTaskCap = legacyMTaskCap;
  uint64_t scoreLimit = std::numeric_limits<uint64_t>::max();
  bool sibEnabled = true;
  { const char* e = std::getenv("GSIM_MT_DENSE_VCONTRACT_SIBLING"); if (e && e[0] == '0') sibEnabled = false; }
  // use V3's critPathCostWithout edge score in the legacy contraction
  // without coupling to V3's cost domain or its soft stop policy.
  const bool edgeCpWithout = [](){ const char* e = std::getenv("GSIM_MT_DENSE_VCONTRACT_EDGE_CPWO"); return e && e[0] && e[0] != '0'; }();
  // Verilator PropagateCp port: keep fwd/rev critical paths ACCURATE through merges so
  // edgeScore reflects live critical paths (gsim previously froze gF/gR at initial values -> stale
  // edgeScore -> suboptimal merge ORDER vs Verilator's lowest-local-CP order). Rather than
  // Verilator's incremental pairing-heap propagation, this port EXACTLY recomputes gF/gR over the
  // live quotient DAG every K merges (see recomputeCP below) and rebuilds the candidate PQ so pops
  // follow the freshly-recomputed CP order. Between recomputes CP is briefly stale, which only
  // shifts merge ORDER (a heuristic, safe for any CP value) -- contraction correctness never
  // depends on it. Default-off knob. NOTE: no CP-ordering cycle prune is added (a prune on stale CP
  // is unsound), so cycle-safety stays the plain gen-tagged DFS; CP feeds ONLY edgeScore.
  bool propagateCp = edgeCpWithout || [](){ const char* e = std::getenv("GSIM_MT_DENSE_VCONTRACT_PROPCP"); return e && e[0] && e[0] != '0'; }();
  // recomputeEvery K: recompute cost is O(V+E) per call * (merges/K), so K amortizes it against the
  // merge loop. The exact recompute (vs Verilator's incremental heap) sidesteps the union-find
  // quotient hazard where CP both rises (cost growth) and falls (a->b edge internalizes) per merge.
  // Scratch buffers reused across recomputeCP calls (cleared per call): 143 calls each
  // allocating ~n vectors of vectors was most of recomputeCP's wall. clear()/assign() reuse
  // the existing allocations; the traversal and arithmetic below are byte-for-byte the
  // former fresh-vector version, so gF/gR come out identical.
  std::vector<int> cpRoots; cpRoots.reserve((size_t)n);
  std::vector<std::vector<int>> cpSucc((size_t)n), cpPred((size_t)n);
  std::vector<int> cpIndeg((size_t)n, 0);
  std::vector<int> cpSs; cpSs.reserve(1024);
  std::vector<int> cpTopo; cpTopo.reserve((size_t)n);
  std::vector<int> cpQ; cpQ.reserve((size_t)n);
  auto recomputeCP = [&]() {
    EmitPhaseAccumScope vcScope(vcAccRecompute, vcPhaseTiming);
    // Live roots and their find()-normalized, deduped succ/pred sets.
    cpRoots.clear();
    for (int i = 0; i < n; i ++) if (find(i) == i) cpRoots.push_back(i);
    for (int r : cpRoots) cpPred[(size_t)r].clear();
    // Dead-entry compaction (GSIM_MT_DENSE_VCONTRACT_COMPACT=1, default OFF):
    // NOT neutral - absorbed groups' stale entries act as live-edge DUPLICATES after
    // find()-normalization; the candidate-time temporary erase removes one occurrence,
    // so a dead duplicate keeps the direct edge visible to pathExists and the merge is
    // spuriously cycle-rejected (measured 16x: cycRej 3.29M -> 0.20M, mtasks 12275 ->
    // 8253 on the T32 champion graph). Default off preserves the registered champion
    // schedules; opt-in produces coarser schedules for perf experimentation.
    // GSIM_MT_DENSE_VCONTRACT_POLICY=auto: two-pass schedule search. Pass 1 runs the
    // legacy mergeLoop (no compaction), records the resulting schedule's invariants
    // (max per-worker static cost, cross-thread edge count); pass 2 re-runs from the
    // pristine graph WITH compaction and records its invariants. The winner is picked
    // by the calibrated two-term floor model (constants from the machine-measured
    // token latencies 24.5ns same-CCD / 290ns cross-CCD and one work-rate point per
    // tier):
    //   score = maxW * A + crossEdges * B * L(threads),  L = 1 (<=16 workers, 1 CCD)
    //                                                         8 (>16 workers, CCD mix)
    // Essence: compaction trades sync points for balance. It wins when the plain
    // schedule is sync-bound (T32: edges -40%, balance 1.63->1.49) and loses when
    // work/balance-bound (T16: maxW floor dominates; coarser tasks fit 16 bins worse).
    // The thread-count heuristic alone matched all measured points; the score model
    // additionally adapts to RTL/workload shape via the actual invariants.
    static const int vcPolicy = [](){
      const char* e = std::getenv("GSIM_MT_DENSE_VCONTRACT_POLICY");
      if (e == nullptr || e[0] == '\0') return 1;   // default: manual COMPACT knob
      if (std::strncmp(e, "auto", 4) == 0) return 2;
      return 1;
    }();
    static const bool vcCompactManual = [](){
      const char* e = std::getenv("GSIM_MT_DENSE_VCONTRACT_COMPACT");
      return e != nullptr && e[0] != '\0' && e[0] != '0';
    }();
    const bool vcCompact = vcPolicy == 2 ? (gVcAutoPass == 1) : vcCompactManual;
    if (vcCompact) for (int r : cpRoots) {
      auto compact = [&](std::vector<int>& v) {
        size_t w = 0;
        for (size_t i2 = 0; i2 < v.size(); i2 ++) if (find(v[i2]) == v[i2]) v[w ++] = v[i2];
        v.resize(w);
      };
      compact(gS[(size_t)r]);
      compact(gP[(size_t)r]);
    }
    for (int r : cpRoots) {
      cpSs.clear();
      for (int s2 : gS[(size_t)r]) { int rs = find(s2); if (rs != r) cpSs.push_back(rs); }
      std::sort(cpSs.begin(), cpSs.end());
      cpSs.erase(std::unique(cpSs.begin(), cpSs.end()), cpSs.end());
      cpSucc[(size_t)r].assign(cpSs.begin(), cpSs.end());
      for (int rs : cpSucc[(size_t)r]) { cpPred[(size_t)rs].push_back(r); }
    }
    for (int r : cpRoots) cpIndeg[(size_t)r] = (int)cpPred[(size_t)r].size();
    // Kahn topo order of live roots.
    cpTopo.clear();
    cpQ.clear(); for (int r : cpRoots) if (cpIndeg[(size_t)r] == 0) cpQ.push_back(r);
    size_t qh = 0;
    while (qh < cpQ.size()) { int u = cpQ[qh ++]; cpTopo.push_back(u); for (int v : cpSucc[(size_t)u]) if (-- cpIndeg[(size_t)v] == 0) cpQ.push_back(v); }
    // gF (cost-to-sink): reverse topo. gR (cost-from-source): forward topo.
    for (int r : cpRoots) { gF[(size_t)r] = 0; gR[(size_t)r] = 0; }
    for (size_t k = cpTopo.size(); k-- > 0; ) { int u = cpTopo[k]; uint64_t b = 0; for (int v : cpSucc[(size_t)u]) b = std::max(b, gF[(size_t)v] + stepCost(gcost[(size_t)v])); gF[(size_t)u] = b; }
    for (int u : cpTopo) { uint64_t b = 0; for (int p : cpPred[(size_t)u]) b = std::max(b, gR[(size_t)p] + stepCost(gcost[(size_t)p])); gR[(size_t)u] = b; }
  };
  // CP-bound-pruned, generation-tagged DFS: does a path root(frm)~>root(to) exist (excluding the
  // direct frm->to edge, which the caller removes temporarily)?
  std::vector<uint32_t> gen((size_t)n, 0); uint32_t curGen = 0;
  // Sound interval bounds for pruning (four per group): reverse-CP range [gRmin,gRmax] and
  // forward-CP range [gFmin,gFmax] over the group's members. During a path search to `to`, a group
  // x is skipped only if it provably cannot precede ANY member of to (gRmin[x] > gRmax[to] or
  // gFmax[x] < gFmin[to]) -- conservative, so the DFS never wrongly prunes a real path.
  std::vector<uint64_t> gRmin((size_t)n), gRmax((size_t)n), gFmin((size_t)n), gFmax((size_t)n);
  for (int i = 0; i < n; i ++) { gRmin[(size_t)i] = gRmax[(size_t)i] = gR[(size_t)i]; gFmin[(size_t)i] = gFmax[(size_t)i] = gF[(size_t)i]; }
  // Exact-answer memo, valid within one merge generation: merges are the ONLY quotient-graph
  // mutation (recomputeCP touches only CP values, PQ churn only re-orders candidates, and the
  // edge branch's direct-edge erase/restore is per-key transient state), so a cached
  // (srcRoot,dstRoot) -> reachable answer equals a fresh computation until the next merge.
  // Cleared after each applied merge; duplicate PQ entries make repeat queries common.
  std::unordered_map<uint64_t, bool> pathMemo;
  uint64_t pathMemoHits = 0, pathCapHits = 0;
  std::vector<int> mergeFixScratch; mergeFixScratch.reserve(1024); // per-merge union scratch: reused, no per-merge alloc
  std::vector<int> pathSt; pathSt.reserve(4096); // reusable DFS stack: no per-query heap traffic
  // DFS-local, epoch-normalized successor cache: gSN[x] holds x's CURRENT-ROOT successors,
  // rebuilt from the raw gS[x] on the first DFS visit of each merge epoch (pathEpoch bumps on
  // every applied merge; the edge branch's transient erase/restore additionally invalidates
  // the one affected list). The merge protocol itself (relContains adjacency checks, candidate
  // scoring/flow, merge fixups) keeps reading the RAW gS lists, so every protocol decision --
  // and therefore the merge sequence and the emitted model -- is untouched.
  // pathExists answers are identical by construction: resolving gS entries with find() at push
  // time (legacy) walks exactly the same quotient graph as reading the pre-resolved gSN
  // entries; the neighbor ORDER differs, but reachability booleans are order-independent and
  // the 500k-visit cap is unreachable here (visits <= live roots <= n < cap), so no answer can
  // diverge. This removes the per-successor find() and the pop-time find() from the DFS hot
  // loop and dedups the stale entries every merge leaves behind (each merge inserts the fresh
  // root but leaves the absorbed group's old ids in neighboring lists).
  // Differential proof: difftest3 harness (/tmp/pathopt/difftest3.cpp, extends the
  // /tmp/schedopt/difftest2 precedent), 33/33 graph-shape x seed x protocol configs
  // (24 mixed edge+sibling + 9 edge-only, up to 20k nodes / 120k edges / 250k queries),
  // zero answer mismatches, zero state divergence.
  std::vector<std::vector<int>> gSN((size_t)n);
  std::vector<uint64_t> gsnEpoch((size_t)n, 0);
  uint64_t pathEpoch = 1;
  auto gsn = [&](int x) -> const std::vector<int>& {
    if (gsnEpoch[(size_t)x] != pathEpoch) {
      std::vector<int>& L = gSN[(size_t)x];
      L = gS[(size_t)x];
      for (int& e : L) e = find(e);
      std::sort(L.begin(), L.end());
      L.erase(std::unique(L.begin(), L.end()), L.end());
      if (std::binary_search(L.begin(), L.end(), x)) L.erase(std::remove(L.begin(), L.end(), x), L.end());
      gsnEpoch[(size_t)x] = pathEpoch;
    }
    return gSN[(size_t)x];
  };
  std::function<bool(int,int)> pathExists = [&](int frm, int to) -> bool {
    EmitPhaseAccumScope vcScope(vcAccPath, vcPhaseTiming);
    const int src = find(frm), dst = find(to);
    if (src == dst) return true;
    // Exact-answer memo, valid within one merge generation. Merges are the only quotient-graph
    // mutation (the edge branch's erase/restore is per-key transient state, restored before any
    // other query), and every (src,dst) query inside a generation sees the identical graph, so a
    // cached answer equals the freshly computed one. Cleared after each applied merge.
    const uint64_t memoKey = ((uint64_t)(uint32_t)src << 32) | (uint32_t)dst;
    auto memoIt = pathMemo.find(memoKey);
    if (memoIt != pathMemo.end()) { pathMemoHits ++; return memoIt->second; }
    curGen ++;
    pathSt.clear();
    pathSt.push_back(src);
    long long visits = 0; const long long visitCap = 500000; // overflow -> assume reachable (reject merge, safe)
    bool reachable = false;
    while (!pathSt.empty()) {
      const int x = pathSt.back(); pathSt.pop_back(); // stack entries are current roots: no pop-time find
      if (x == to) { reachable = true; break; }
      if (gen[(size_t)x] == curGen) continue;
      gen[(size_t)x] = curGen;
      if (++ visits > visitCap) { reachable = true; ++ pathCapHits; break; } // same conservative fallback as before
      // NOTE: a CP-ordering prune here (skip x if gF[x]+step>gF[to]) is UNSOUND with bounded/
      // stale-LOW propagated CP: a stale-low gF[to] would make the prune over-fire and wrongly
      // reject a real ancestor -> a cycle-creating merge. gF is only a LOWER bound (propagation
      // never overshoots), and comparing two lower bounds cannot soundly prune. So we keep the
      // plain gen-tagged DFS for cycle-safety; propagated CP is used ONLY in edgeScore (merge
      // ORDER, a heuristic that is safe for any CP value). This still captures Verilator's
      // exact-merge-order fidelity win without the unsound prune.
      for (int s2 : gsn(x)) {
        // s2 is a current root. A successor equal to the target proves reachability right now:
        // the legacy DFS proves the same fact when it pops this entry, so the boolean is
        // identical -- this only skips the push/pop round trip (and the subtree exploration
        // that would precede the target's pop).
        if (s2 == to) { reachable = true; break; }
        pathSt.push_back(s2);
      }
      if (reachable) break;
    }
    pathMemo.emplace(memoKey, reachable);
    return reachable;
  };
  uint32_t scoreCacheGeneration = 1;
  std::vector<uint32_t> incomingScoreCacheGeneration((size_t)n, 0), outgoingScoreCacheGeneration((size_t)n, 0);
  std::vector<int> incomingBestRoot((size_t)n, -1), incomingSecondRoot((size_t)n, -1), outgoingBestRoot((size_t)n, -1), outgoingSecondRoot((size_t)n, -1);
  std::vector<uint64_t> incomingBestValue((size_t)n, 0), incomingSecondValue((size_t)n, 0), outgoingBestValue((size_t)n, 0), outgoingSecondValue((size_t)n, 0);
  auto refreshBestTwo = [&](int node, const std::vector<int>& relations, const std::vector<uint64_t>& cp,
                            std::vector<uint32_t>& cacheGeneration, std::vector<int>& bestRoot,
                            std::vector<int>& secondRoot, std::vector<uint64_t>& bestValue,
                            std::vector<uint64_t>& secondValue) {
    if (cacheGeneration[(size_t)node] == scoreCacheGeneration) return;
    EmitPhaseAccumScope vcScope(vcAccBestTwo, vcPhaseTiming);
    cacheGeneration[(size_t)node] = scoreCacheGeneration;
    bestRoot[(size_t)node] = -1; secondRoot[(size_t)node] = -1;
    bestValue[(size_t)node] = 0; secondValue[(size_t)node] = 0;
    for (int relation : relations) {
      int root = find(relation);
      if (root == node || root == bestRoot[(size_t)node] || root == secondRoot[(size_t)node]) continue;
      uint64_t value = cp[(size_t)root] + stepCost(gcost[(size_t)root]);
      if (bestRoot[(size_t)node] < 0 || value > bestValue[(size_t)node] || (value == bestValue[(size_t)node] && root < bestRoot[(size_t)node])) {
        secondRoot[(size_t)node] = bestRoot[(size_t)node]; secondValue[(size_t)node] = bestValue[(size_t)node];
        bestRoot[(size_t)node] = root; bestValue[(size_t)node] = value;
      } else if (secondRoot[(size_t)node] < 0 || value > secondValue[(size_t)node] || (value == secondValue[(size_t)node] && root < secondRoot[(size_t)node])) {
        secondRoot[(size_t)node] = root; secondValue[(size_t)node] = value;
      }
    }
  };
  auto incomingWithout = [&](int node, int excluded) -> uint64_t {
    refreshBestTwo(node, gP[(size_t)node], gR, incomingScoreCacheGeneration, incomingBestRoot, incomingSecondRoot, incomingBestValue, incomingSecondValue);
    return incomingBestRoot[(size_t)node] == excluded ? incomingSecondValue[(size_t)node] : incomingBestValue[(size_t)node];
  };
  auto outgoingWithout = [&](int node, int excluded) -> uint64_t {
    refreshBestTwo(node, gS[(size_t)node], gF, outgoingScoreCacheGeneration, outgoingBestRoot, outgoingSecondRoot, outgoingBestValue, outgoingSecondValue);
    return outgoingBestRoot[(size_t)node] == excluded ? outgoingSecondValue[(size_t)node] : outgoingBestValue[(size_t)node];
  };
  auto siblingScore = [&](int a, int b) -> uint64_t {
    return std::max(gF[(size_t)a], gF[(size_t)b]) + std::max(gR[(size_t)a], gR[(size_t)b])
        + stepCost(gcost[(size_t)a] + gcost[(size_t)b]);
  };
  auto edgeScore = [&](int a, int b) -> uint64_t {
    if (!edgeCpWithout) {
      return std::max(gF[(size_t)a], gF[(size_t)b]) + std::max(gR[(size_t)a], gR[(size_t)b])
          + stepCost(gcost[(size_t)a] + gcost[(size_t)b]);
    }
    const uint64_t mergedFromStart = std::max(gR[(size_t)a], incomingWithout(b, a));
    const uint64_t mergedToEnd = std::max(outgoingWithout(a, b), gF[(size_t)b]);
    return mergedFromStart + mergedToEnd + stepCost(gcost[(size_t)a] + gcost[(size_t)b]);
  };
  auto candidateScore = [&](int a, int b, bool sibling) -> uint64_t {
    uint64_t score = sibling ? siblingScore(a, b) : edgeScore(a, b);
    return score;
  };
  struct Cand { uint64_t score; int a; int b; bool sibling; };
  struct Cmp {
    bool operator()(const Cand& x, const Cand& y) const {
      return x.score > y.score;
    }
  };
  std::priority_queue<Cand, std::vector<Cand>, Cmp> pq;
  auto pushEdges = [&](int r) {
    for (int s2 : gS[(size_t)r]) { int rs = find(s2); if (rs != r && gw0[(size_t)r] == gw0[(size_t)rs] && gcost[(size_t)r] + gcost[(size_t)rs] <= perMTaskCap) pq.push({candidateScore(r, rs, false), r, rs, false}); }
  };
  // The legacy heuristic pairs a node with successors of each predecessor.
  auto pushSiblings = [&](int r) {
    int emitted = 0; const int sibCap = 8;
    for (int p : gP[(size_t)r]) { int rp = find(p);
      for (int s2 : gS[(size_t)rp]) { int rs = find(s2);
        if (rs != r && rs != rp && gw0[(size_t)r] == gw0[(size_t)rs] && gcost[(size_t)r] + gcost[(size_t)rs] <= perMTaskCap) { pq.push({candidateScore(r, rs, true), r, rs, true}); if (++ emitted >= sibCap) return; } } }
  };
  auto rebuildPQ = [&]() {
    EmitPhaseAccumScope vcScope(vcAccRebuildPQ, vcPhaseTiming);
    // Rebuild the candidate heap from scratch over all live roots with LIVE edgeScore. Called after
    // recomputeCP() so every candidate is scored against the freshly-recomputed critical paths
    // (a recompute can LOWER a CP, burying a now-cheaper edge under a stale-high key that on-pop
    // revalidation alone could never surface). O(V+E) per call, amortized by the fixed K=256.
    std::priority_queue<Cand, std::vector<Cand>, Cmp> empty; pq.swap(empty);
    for (int u = 0; u < n; u ++) if (find(u) == u) { pushEdges(u); if (sibEnabled) pushSiblings(u); }
  };
  for (int u = 0; u < n; u ++) if (find(u) == u) { pushEdges(u); if (sibEnabled) pushSiblings(u); }
  if (vcPhaseTiming) vcAccInit.ns += std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - vcInitBegin).count();
  int live = n; uint64_t merges = 0, cycRej = 0, sibMerges = 0, entryExitSkips = 0, scoreLimitEscalations = 0;
  {
  EmitPhaseAccumScope vcLoopScope(vcAccLoop, vcPhaseTiming);
  while (!pq.empty()) {
    if (live <= maxMTasks) break;
    Cand c = pq.top(); pq.pop();
    int a = find(c.a), b = find(c.b);
    if (a == b || gw0[(size_t)a] != gw0[(size_t)b]) continue;
    if (gcost[(size_t)a] + gcost[(size_t)b] > perMTaskCap) continue;
    if (propagateCp) {
      // Lazy stale-key check: gcost grows within a recompute window, so a candidate's stored score
      // may no longer equal the live score.  The periodic full rebuild also repairs score decreases.
      uint64_t liveScore = candidateScore(a, b, c.sibling);
      if (liveScore != c.score) { pq.push({liveScore, a, b, c.sibling}); continue; }
    }
    bool cyc;
    if (c.sibling) {
      // Sibling merge: no direct edge assumed. Cycle-safe iff neither a~>b nor b~>a.
      if (relContains(gS[(size_t)a], b) || relContains(gS[(size_t)b], a)) continue; // became adjacent; let edge path handle
      cyc = pathExists(a, b) || pathExists(b, a);
      if (cyc) { cycRej ++; continue; }
    } else {
      if (!relContains(gS[(size_t)a], b)) continue;
      // Cycle-safe iff no ALTERNATE path a~>b once the direct edge is excluded.
      relErase(gS[(size_t)a], b); relErase(gP[(size_t)b], a); gsnEpoch[(size_t)a] = 0; // cached gSN[a] would still contain the erased edge
      cyc = pathExists(a, b);
      if (cyc) { relInsert(gS[(size_t)a], b); relInsert(gP[(size_t)b], a); gsnEpoch[(size_t)a] = 0; cycRej ++; continue; } // restored list differs from any cached copy too
    }
    // merge b into a
    {
    EmitPhaseAccumScope vcScope(vcAccMergeApply, vcPhaseTiming);
    uf[(size_t)b] = a; gcost[(size_t)a] += gcost[(size_t)b];
    gF[(size_t)a] = std::max(gF[(size_t)a], gF[(size_t)b]); gR[(size_t)a] = std::max(gR[(size_t)a], gR[(size_t)b]);
    gRmin[(size_t)a] = std::min(gRmin[(size_t)a], gRmin[(size_t)b]); gRmax[(size_t)a] = std::max(gRmax[(size_t)a], gRmax[(size_t)b]);
    gFmin[(size_t)a] = std::min(gFmin[(size_t)a], gFmin[(size_t)b]); gFmax[(size_t)a] = std::max(gFmax[(size_t)a], gFmax[(size_t)b]);
    // Batch-union fixup: collect b's live root-neighbor set once (sorted+deduped), dedup-
    // insert the survivor into each surviving neighbor's mirror list, then rebuild the
    // survivor's own two lists with one sorted union each (relUnionInto). Content-identical
    // to the former incremental insert-per-edge sequence: both end at (old set union added
    // roots) as sorted unique vectors, and no reader observes the intermediate states (the
    // only lists read between the writes are b's own two lists plus lists the incremental
    // version had not reached either; find() sees uf[b]=a in both). The O(len) memmove-per-
    // edge of relInsert collapses to one linear pass on the survivor side; perMTaskCap
    // bounds group size, so the unioned spans stay small.
    mergeFixScratch.clear();
    for (int s2 : gS[(size_t)b]) { int rs = find(s2); if (rs != a) mergeFixScratch.push_back(rs); }
    std::sort(mergeFixScratch.begin(), mergeFixScratch.end());
    mergeFixScratch.erase(std::unique(mergeFixScratch.begin(), mergeFixScratch.end()), mergeFixScratch.end());
    for (int rs : mergeFixScratch) relInsert(gP[(size_t)rs], a);
    relUnionInto(gS[(size_t)a], mergeFixScratch, relUnionOut);
    mergeFixScratch.clear();
    for (int p : gP[(size_t)b]) { int rp = find(p); if (rp != a) mergeFixScratch.push_back(rp); }
    std::sort(mergeFixScratch.begin(), mergeFixScratch.end());
    mergeFixScratch.erase(std::unique(mergeFixScratch.begin(), mergeFixScratch.end()), mergeFixScratch.end());
    for (int rp : mergeFixScratch) relInsert(gS[(size_t)rp], a);
    relUnionInto(gP[(size_t)a], mergeFixScratch, relUnionOut);
    relErase(gS[(size_t)a], a); relErase(gP[(size_t)a], a); relErase(gS[(size_t)a], b); relErase(gP[(size_t)a], b);
    live --; merges ++; ++scoreCacheGeneration; if (c.sibling) sibMerges ++;
    pathMemo.clear(); // quotient graph changed: reachability answers are stale
    pathEpoch ++;     // ... and every gSN snapshot is stale
    }
    if (propagateCp && (merges % 256u) == 0) { recomputeCP(); ++scoreCacheGeneration; rebuildPQ(); }
    {
    EmitPhaseAccumScope vcScope(vcAccMergeApply, vcPhaseTiming);
    pushEdges(a); if (sibEnabled) pushSiblings(a);
    for (int p : gP[(size_t)a]) {
      int rp = find(p);
      if (rp != a && gw0[(size_t)rp] == gw0[(size_t)a] && gcost[(size_t)rp] + gcost[(size_t)a] <= perMTaskCap) pq.push({candidateScore(rp, a, false), rp, a, false});
    }
    }
  }
  }
  const std::chrono::steady_clock::time_point vcMaterializeBegin =
      vcPhaseTiming ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point();
  // Materialize groups in Kahn topo order for monotone ids.
  std::map<int,int> rootIdx; for (int scc = 0; scc < realN; ++scc) { int r = find(denseNode(scc)); if (!rootIdx.count(r)) rootIdx[r] = (int)rootIdx.size(); }
  int R = (int)rootIdx.size();
  std::vector<std::set<int>> rS((size_t)R), rP((size_t)R);
  for (int fromScc = 0; fromScc < realN; ++fromScc) { int ru = rootIdx[find(denseNode(fromScc))]; for (int toScc : schedule.sccs[(size_t)fromScc].succSccs) { if (toScc < 0 || toScc >= realN) continue; int rv = rootIdx[find(denseNode(toScc))]; if (rv != ru) { rS[(size_t)ru].insert(rv); rP[(size_t)rv].insert(ru); } } }
  std::vector<int> minScc((size_t)R, INT32_MAX);
  for (int s : schedule.topoSccOrder) { auto it = rootIdx.find(find(denseNode(s))); if (it != rootIdx.end()) minScc[(size_t)it->second] = std::min(minScc[(size_t)it->second], s); }
  std::vector<int> indeg((size_t)R, 0); for (int i = 0; i < R; i ++) indeg[(size_t)i] = (int)rP[(size_t)i].size();
  auto cmp = [&](int a, int b){ if (minScc[(size_t)a] != minScc[(size_t)b]) return minScc[(size_t)a] > minScc[(size_t)b]; return a > b; };
  std::priority_queue<int, std::vector<int>, decltype(cmp)> rq(cmp);
  for (int i = 0; i < R; i ++) if (indeg[(size_t)i] == 0) rq.push(i);
  std::vector<int> rootOrder((size_t)R, -1); int nextId = 0;
  while (!rq.empty()) { int u = rq.top(); rq.pop(); rootOrder[(size_t)u] = nextId ++; for (int v : rS[(size_t)u]) if (-- indeg[(size_t)v] == 0) rq.push(v); }
  Assert(nextId == R, "verilator contraction cyclic MTask graph (%d/%d)", nextId, R);
  mtasks.assign((size_t)R, MtDenseMTask());
  std::vector<int> sccToMTask((size_t)realN, -1);
  for (int s : schedule.topoSccOrder) { auto it = rootIdx.find(find(denseNode(s))); if (it == rootIdx.end()) continue; int mi = rootOrder[(size_t)it->second]; MtDenseMTask& mt = mtasks[(size_t)mi]; mt.sccIds.push_back(s); mt.staticCost += schedule.sccs[(size_t)s].staticCost; mt.schedCost += (int)sc(s); mt.taskCount += (int)schedule.sccs[(size_t)s].cppIds.size(); mt.workerZeroOnly = mt.workerZeroOnly || schedule.sccs[(size_t)s].workerZeroOnly; sccToMTask[(size_t)s] = mi; }
  std::vector<std::set<int>> predSets((size_t)R), succSets((size_t)R);
  for (int fromScc = 0; fromScc < realN; ++fromScc) { int fm = sccToMTask[(size_t)fromScc]; if (fm < 0) continue; for (int toScc : schedule.sccs[(size_t)fromScc].succSccs) { int tm = (toScc >= 0 && toScc < realN) ? sccToMTask[(size_t)toScc] : -1; if (tm < 0 || tm == fm) continue; succSets[(size_t)fm].insert(tm); predSets[(size_t)tm].insert(fm); } }
  for (int mi = 0; mi < R; mi ++) { mtasks[(size_t)mi].predMTasks.assign(predSets[(size_t)mi].begin(), predSets[(size_t)mi].end()); mtasks[(size_t)mi].succMTasks.assign(succSets[(size_t)mi].begin(), succSets[(size_t)mi].end()); }
  fprintf(stderr, "[mt-dense-vcontract] sccs=%d -> mtasks=%d merges=%llu (sibling=%llu) cycRej=%llu entryExitSkips=%llu maxMTasks=%d v3Policy=%d scoreLimit=%llu escalations=%llu pathMemoHits=%llu pathCapHits=%llu\n", realN, R, (unsigned long long)merges, (unsigned long long)sibMerges, (unsigned long long)cycRej, (unsigned long long)entryExitSkips, maxMTasks, 0, (unsigned long long)scoreLimit, (unsigned long long)scoreLimitEscalations, (unsigned long long)pathMemoHits, (unsigned long long)pathCapHits);
  if (vcPhaseTiming) vcAccMaterialize.ns += std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - vcMaterializeBegin).count();
  emitPhaseAccumReport(vcAccInit);
  emitPhaseAccumReport(vcAccLoop);
  emitPhaseAccumReport(vcAccRecompute);
  emitPhaseAccumReport(vcAccPath);
  emitPhaseAccumReport(vcAccBestTwo);
  emitPhaseAccumReport(vcAccRebuildPQ);
  emitPhaseAccumReport(vcAccMergeApply);
  emitPhaseAccumReport(vcAccMaterialize);
  return mtasks;
}

static std::vector<MtDenseMTask> mtBuildDenseMTasks(const MtDenseSchedule& schedule,
                                                    bool preserveWorkerZeroOnlyBoundary) {
  const int nSccs = static_cast<int>(schedule.sccs.size());
  const int maxSccsPerMTask = 30;
  std::vector<MtDenseMTask> mtasks;
  std::vector<int> sccToMTask((size_t)nSccs, -1);
  if (preserveWorkerZeroOnlyBoundary) {
    MtDenseMTask current;
    auto flushCurrent = [&]() {
      if (current.sccIds.empty()) return;
      mtasks.push_back(current);
      current = MtDenseMTask();
    };
    for (int sccId : schedule.topoSccOrder) {
      if (sccId < 0 || sccId >= nSccs) continue;
      const MtDenseScc& scc = schedule.sccs[(size_t)sccId];
      if (!current.sccIds.empty() &&
          (static_cast<int>(current.sccIds.size()) >= maxSccsPerMTask || current.workerZeroOnly != scc.workerZeroOnly)) {
        flushCurrent();
      }
      sccToMTask[(size_t)sccId] = static_cast<int>(mtasks.size());
      current.sccIds.push_back(sccId);
      current.staticCost += scc.staticCost;
      current.taskCount += static_cast<int>(scc.cppIds.size());
      current.workerZeroOnly = current.workerZeroOnly || scc.workerZeroOnly;
    }
    flushCurrent();
  } else {
    for (int begin = 0; begin < static_cast<int>(schedule.topoSccOrder.size()); begin += maxSccsPerMTask) {
      int end = std::min(begin + maxSccsPerMTask, static_cast<int>(schedule.topoSccOrder.size()));
      int mtaskId = static_cast<int>(mtasks.size());
      MtDenseMTask mtask;
      for (int orderIndex = begin; orderIndex < end; orderIndex ++) {
        int sccId = schedule.topoSccOrder[(size_t)orderIndex];
        Assert(sccId >= 0 && sccId < nSccs, "dense schedule topo index out of range");
        sccToMTask[(size_t)sccId] = mtaskId;
        mtask.sccIds.push_back(sccId);
        mtask.staticCost += schedule.sccs[(size_t)sccId].staticCost;
        mtask.taskCount += static_cast<int>(schedule.sccs[(size_t)sccId].cppIds.size());
        mtask.workerZeroOnly = mtask.workerZeroOnly || schedule.sccs[(size_t)sccId].workerZeroOnly;
      }
      mtasks.push_back(mtask);
    }
  }
  std::vector<std::set<int>> predSets(mtasks.size()), succSets(mtasks.size());
  for (int fromScc = 0; fromScc < nSccs; fromScc ++) {
    int fromMTask = sccToMTask[(size_t)fromScc];
    if (fromMTask < 0) continue;
    for (int toScc : schedule.sccs[(size_t)fromScc].succSccs) {
      int toMTask = toScc >= 0 && toScc < nSccs ? sccToMTask[(size_t)toScc] : -1;
      if (toMTask < 0 || toMTask == fromMTask) continue;
      succSets[(size_t)fromMTask].insert(toMTask);
      predSets[(size_t)toMTask].insert(fromMTask);
    }
  }
  for (size_t i = 0; i < mtasks.size(); i ++) {
    mtasks[i].predMTasks.assign(predSets[i].begin(), predSets[i].end());
    mtasks[i].succMTasks.assign(succSets[i].begin(), succSets[i].end());
  }
  return mtasks;
}

static std::vector<std::vector<int>> mtBuildDenseRuntimeSuccs(const std::vector<MtDenseMTask>& mtasks,
                                                             const std::vector<int>& assignment,
                                                             bool xthreadDepsOnly,
                                                             int* sameThreadElidedCount = nullptr) {
  const int nMTasks = static_cast<int>(mtasks.size());
  std::vector<std::vector<int>> runtimeSuccs((size_t)nMTasks);
  if (sameThreadElidedCount != nullptr) *sameThreadElidedCount = 0;
  for (int mtaskId = 0; mtaskId < nMTasks; mtaskId ++) {
    int srcThread = mtaskId < static_cast<int>(assignment.size()) ? assignment[(size_t)mtaskId] : -1;
    for (int succ : mtasks[(size_t)mtaskId].succMTasks) {
      if (succ < 0 || succ >= nMTasks) continue;
      int dstThread = succ < static_cast<int>(assignment.size()) ? assignment[(size_t)succ] : -1;
      if (xthreadDepsOnly && srcThread >= 0 && dstThread >= 0 && srcThread == dstThread) {
        if (sameThreadElidedCount != nullptr) (*sameThreadElidedCount) ++;
        continue;
      }
      runtimeSuccs[(size_t)mtaskId].push_back(succ);
    }
    std::sort(runtimeSuccs[(size_t)mtaskId].begin(), runtimeSuccs[(size_t)mtaskId].end());
    runtimeSuccs[(size_t)mtaskId].erase(std::unique(runtimeSuccs[(size_t)mtaskId].begin(), runtimeSuccs[(size_t)mtaskId].end()), runtimeSuccs[(size_t)mtaskId].end());
  }
  return runtimeSuccs;
}

static int mtDenseRuntimeEdgeCount(const std::vector<std::vector<int>>& runtimeSuccs) {
  int edgeCount = 0;
  for (const std::vector<int>& succs : runtimeSuccs) edgeCount += static_cast<int>(succs.size());
  return edgeCount;
}

static MtDenseOwnerReadyLayout mtBuildDenseOwnerReadyLayout(
    const std::vector<std::vector<int>>& runtimeSuccs,
    const std::vector<int>& assignment,
    int threadCount) {
  const int nMTasks = static_cast<int>(runtimeSuccs.size());
  Assert(threadCount > 0, "dense owner-ready layout requires at least one thread");
  Assert(static_cast<int>(assignment.size()) == nMTasks,
         "dense owner-ready layout assignment size %zu does not match MTask count %d",
         assignment.size(), nMTasks);

  // Fixed workers execute their assigned MTasks in ascending id order. Record that
  // order explicitly so the selected writer is tied to the actual emitted order.
  std::vector<int> ownerOrdinal((size_t)nMTasks, -1);
  for (int owner = 0; owner < threadCount; owner ++) {
    int ordinal = 0;
    for (int mtaskId = 0; mtaskId < nMTasks; mtaskId ++) {
      Assert(assignment[(size_t)mtaskId] >= 0 && assignment[(size_t)mtaskId] < threadCount,
             "dense owner-ready MTask %d has invalid owner %d for %d threads",
             mtaskId, assignment[(size_t)mtaskId], threadCount);
      if (assignment[(size_t)mtaskId] == owner) ownerOrdinal[(size_t)mtaskId] = ordinal ++;
    }
  }
  for (int mtaskId = 0; mtaskId < nMTasks; mtaskId ++) {
    Assert(ownerOrdinal[(size_t)mtaskId] >= 0,
           "dense owner-ready MTask %d is absent from fixed owner order", mtaskId);
  }

  // One token represents all reduced edges from one producer owner to one
  // destination. The last such producer in fixed-owner order publishes it.
  std::map<std::pair<int, int>, std::vector<int>> sourcesByDestinationOwner;
  int edgeCount = 0;
  for (int source = 0; source < nMTasks; source ++) {
    int previousDestination = -1;
    for (int destination : runtimeSuccs[(size_t)source]) {
      Assert(destination >= 0 && destination < nMTasks,
             "dense owner-ready edge %d -> %d has invalid destination", source, destination);
      Assert(destination > source,
             "dense owner-ready edge %d -> %d is not forward in fixed MTask order",
             source, destination);
      Assert(destination > previousDestination,
             "dense owner-ready successors for MTask %d are not unique ascending ids", source);
      previousDestination = destination;
      const int producerOwner = assignment[(size_t)source];
      const int consumerOwner = assignment[(size_t)destination];
      Assert(producerOwner != consumerOwner,
             "dense owner-ready edge %d -> %d is not cross-thread (%d)",
             source, destination, producerOwner);
      sourcesByDestinationOwner[std::make_pair(destination, producerOwner)].push_back(source);
      edgeCount ++;
    }
  }

  struct Group {
    int destination = -1;
    int producerOwner = -1;
    int consumerOwner = -1;
    int lastSource = -1;
    int slot = -1;
    int logicalToken = -1;
    std::vector<int> sources;
  };
  std::vector<Group> groups;
  int groupedEdgeCount = 0;
  for (const auto& entry : sourcesByDestinationOwner) {
    Group group;
    group.destination = entry.first.first;
    group.producerOwner = entry.first.second;
    group.consumerOwner = assignment[(size_t)group.destination];
    group.sources = entry.second;
    Assert(!group.sources.empty(),
           "dense owner-ready group destination %d producer %d is empty",
           group.destination, group.producerOwner);
    int previousOrdinal = -1;
    for (int source : group.sources) {
      Assert(assignment[(size_t)source] == group.producerOwner,
             "dense owner-ready group source %d owner %d does not match producer %d",
             source, assignment[(size_t)source], group.producerOwner);
      Assert(ownerOrdinal[(size_t)source] > previousOrdinal,
             "dense owner-ready group destination %d producer %d is not in fixed-owner order",
             group.destination, group.producerOwner);
      previousOrdinal = ownerOrdinal[(size_t)source];
      groupedEdgeCount ++;
    }
    group.lastSource = group.sources.back();
    Assert(assignment[(size_t)group.lastSource] == group.producerOwner,
           "dense owner-ready last source %d has wrong owner", group.lastSource);
    for (int source : group.sources) {
      Assert(ownerOrdinal[(size_t)source] <= ownerOrdinal[(size_t)group.lastSource],
             "dense owner-ready source %d follows selected last source %d",
             source, group.lastSource);
    }
    group.logicalToken = static_cast<int>(groups.size());
    groups.push_back(group);
  }
  Assert(groupedEdgeCount == edgeCount,
         "dense owner-ready groups cover %d of %d reduced edges", groupedEdgeCount, edgeCount);

  std::map<std::pair<int, int>, std::vector<int>> groupsByOwnerPair;
  for (int groupId = 0; groupId < static_cast<int>(groups.size()); groupId ++) {
    const Group& group = groups[(size_t)groupId];
    groupsByOwnerPair[std::make_pair(group.producerOwner, group.consumerOwner)].push_back(groupId);
  }

  MtDenseOwnerReadyLayout layout;
  layout.edgeCount = edgeCount;
  layout.tokenCount = static_cast<int>(groups.size());
  layout.pairBankCount = static_cast<int>(groupsByOwnerPair.size());
  layout.waitSlotsByMTask.assign((size_t)nMTasks, std::vector<int>());
  layout.storeSlotsByMTask.assign((size_t)nMTasks, std::vector<int>());
  auto alignCacheLine = [](int slot) { return (slot + 63) & ~63; };
  int nextSlot = 0;
  std::vector<std::tuple<int, int, int, int>> bankRanges;
  for (const auto& bank : groupsByOwnerPair) {
    const int producerOwner = bank.first.first;
    const int consumerOwner = bank.first.second;
    Assert(producerOwner != consumerOwner,
           "dense owner-ready pair bank unexpectedly aliases owner %d", producerOwner);
    nextSlot = alignCacheLine(nextSlot);
    const int bankBegin = nextSlot;
    Assert((bankBegin & 63) == 0,
           "dense owner-ready pair bank %d -> %d does not start on 64-byte boundary",
           producerOwner, consumerOwner);
    for (int groupId : bank.second) {
      Group& group = groups[(size_t)groupId];
      Assert(group.slot < 0, "dense owner-ready group %d received duplicate slots", groupId);
      Assert(group.producerOwner == producerOwner && group.consumerOwner == consumerOwner,
             "dense owner-ready group %d escaped pair bank %d -> %d",
             groupId, producerOwner, consumerOwner);
      group.slot = nextSlot ++;
    }
    const int bankEnd = alignCacheLine(nextSlot);
    Assert((bankEnd & 63) == 0 && bankEnd > bankBegin,
           "dense owner-ready pair bank %d -> %d has invalid aligned extent [%d,%d)",
           producerOwner, consumerOwner, bankBegin, bankEnd);
    for (int groupId : bank.second) {
      const Group& group = groups[(size_t)groupId];
      Assert(group.slot >= bankBegin && group.slot + 1 <= bankEnd,
             "dense owner-ready one-byte slot %d escapes pair bank [%d,%d)",
             group.slot, bankBegin, bankEnd);
    }
    bankRanges.emplace_back(producerOwner, consumerOwner, bankBegin, bankEnd);
    nextSlot = bankEnd;
  }
  for (size_t lhs = 0; lhs < bankRanges.size(); lhs ++) {
    int lhsBegin = std::get<2>(bankRanges[lhs]);
    int lhsEnd = std::get<3>(bankRanges[lhs]);
    for (size_t rhs = lhs + 1; rhs < bankRanges.size(); rhs ++) {
      int rhsBegin = std::get<2>(bankRanges[rhs]);
      int rhsEnd = std::get<3>(bankRanges[rhs]);
      Assert(lhsEnd <= rhsBegin || rhsEnd <= lhsBegin,
             "dense owner-ready pair banks overlap: [%d,%d) and [%d,%d)",
             lhsBegin, lhsEnd, rhsBegin, rhsEnd);
    }
  }

  layout.tokenProvenanceByLogicalToken.assign(
      (size_t)layout.tokenCount, MtDenseOwnerReadyTokenProvenance());
  layout.sourceMTasksByLogicalToken.assign((size_t)layout.tokenCount, std::vector<int>());
  layout.logicalTokenByPhysicalSlot.assign((size_t)nextSlot, -1);
  std::set<int> uniqueSlots;
  for (const Group& group : groups) {
    Assert(group.slot >= 0, "dense owner-ready group has no physical slot");
    Assert(uniqueSlots.insert(group.slot).second,
           "dense owner-ready physical slot %d is shared by multiple groups", group.slot);
    Assert(group.logicalToken >= 0 && group.logicalToken < layout.tokenCount,
           "dense owner-ready group has invalid logical token %d", group.logicalToken);
    Assert(layout.logicalTokenByPhysicalSlot[(size_t)group.slot] < 0,
           "dense owner-ready physical slot %d aliases logical tokens %d and %d",
           group.slot, layout.logicalTokenByPhysicalSlot[(size_t)group.slot], group.logicalToken);
    MtDenseOwnerReadyTokenProvenance& provenance =
        layout.tokenProvenanceByLogicalToken[(size_t)group.logicalToken];
    Assert(provenance.readySlot < 0,
           "dense owner-ready logical token %d has duplicate provenance", group.logicalToken);
    provenance.readySlot = group.slot;
    provenance.producerMTask = group.lastSource;
    provenance.producerOwner = group.producerOwner;
    provenance.consumerMTask = group.destination;
    provenance.consumerOwner = group.consumerOwner;
    layout.sourceMTasksByLogicalToken[(size_t)group.logicalToken] = group.sources;
    layout.logicalTokenByPhysicalSlot[(size_t)group.slot] = group.logicalToken;
    layout.waitSlotsByMTask[(size_t)group.destination].push_back(group.slot);
    layout.storeSlotsByMTask[(size_t)group.lastSource].push_back(group.slot);
  }
  for (int logicalToken = 0; logicalToken < layout.tokenCount; logicalToken ++) {
    const MtDenseOwnerReadyTokenProvenance& provenance =
        layout.tokenProvenanceByLogicalToken[(size_t)logicalToken];
    Assert(provenance.readySlot >= 0 && provenance.readySlot < nextSlot,
           "dense owner-ready logical token %d has invalid physical slot %d",
           logicalToken, provenance.readySlot);
    Assert(layout.logicalTokenByPhysicalSlot[(size_t)provenance.readySlot] == logicalToken,
           "dense owner-ready logical token %d does not round-trip through slot %d",
           logicalToken, provenance.readySlot);
    Assert(provenance.producerMTask >= 0 && provenance.producerMTask < nMTasks
           && provenance.consumerMTask >= 0 && provenance.consumerMTask < nMTasks
           && provenance.producerOwner >= 0 && provenance.producerOwner < threadCount
           && provenance.consumerOwner >= 0 && provenance.consumerOwner < threadCount,
           "dense owner-ready logical token %d has incomplete provenance", logicalToken);
    Assert(assignment[(size_t)provenance.producerMTask] == provenance.producerOwner
           && assignment[(size_t)provenance.consumerMTask] == provenance.consumerOwner,
           "dense owner-ready logical token %d owner provenance mismatch", logicalToken);
  }
  Assert(static_cast<int>(uniqueSlots.size()) == layout.tokenCount,
         "dense owner-ready unique slot count %zu does not match token count %d",
         uniqueSlots.size(), layout.tokenCount);
  int waitTokenCount = 0;
  int storeTokenCount = 0;
  for (int mtaskId = 0; mtaskId < nMTasks; mtaskId ++) {
    std::vector<int>& waits = layout.waitSlotsByMTask[(size_t)mtaskId];
    std::vector<int>& stores = layout.storeSlotsByMTask[(size_t)mtaskId];
    std::sort(waits.begin(), waits.end());
    std::sort(stores.begin(), stores.end());
    waitTokenCount += static_cast<int>(waits.size());
    storeTokenCount += static_cast<int>(stores.size());
  }
  Assert(waitTokenCount == layout.tokenCount && storeTokenCount == layout.tokenCount,
         "dense owner-ready token coverage mismatch: waits=%d stores=%d tokens=%d",
         waitTokenCount, storeTokenCount, layout.tokenCount);
  if (nextSlot == 0) nextSlot = 64;
  layout.logicalTokenByPhysicalSlot.resize((size_t)nextSlot, -1);
  Assert((nextSlot & 63) == 0,
         "dense owner-ready physical layout does not end on 64-byte boundary: %d", nextSlot);
  layout.physicalSlotCount = nextSlot;
  return layout;
}

static MtDenseBreakdownWindowWaitLayout mtBuildDenseBreakdownWindowWaitLayout(
    const MtDenseOwnerReadyLayout& readyLayout,
    const std::vector<int>& assignment,
    int threadCount) {
  Assert(threadCount > 0, "dense breakdown window requires at least one wait lane");
  MtDenseBreakdownWindowWaitLayout layout;
  layout.laneOffsets.assign((size_t)threadCount + 1, 0);
  for (int mtaskId = 0; mtaskId < static_cast<int>(assignment.size()); mtaskId ++) {
    const int worker = assignment[(size_t)mtaskId];
    Assert(worker >= 0 && worker < threadCount,
           "dense breakdown window MTask %d has invalid owner %d", mtaskId, worker);
    layout.laneOffsets[(size_t)worker + 1] +=
        static_cast<int>(readyLayout.waitSlotsByMTask[(size_t)mtaskId].size());
  }
  for (int worker = 0; worker < threadCount; worker ++) {
    layout.laneOffsets[(size_t)worker + 1] += layout.laneOffsets[(size_t)worker];
  }
  layout.totalWaitRecords = layout.laneOffsets.back();
  return layout;
}

static MtDenseBreakdownWindowAllOwnerLayout mtBuildDenseBreakdownWindowAllOwnerLayout(
    const std::vector<int>& assignment, int threadCount) {
  Assert(threadCount > 0, "dense breakdown all-owner layout requires workers");
  constexpr int kRecordsPerAlignedOwnerLane = 8;
  MtDenseBreakdownWindowAllOwnerLayout layout;
  layout.laneOffsets.assign((size_t)threadCount + 1, 0);
  layout.recordIndexByMTask.assign(assignment.size(), -1);
  std::vector<int> laneCounts((size_t)threadCount, 0);
  for (int mtaskId = 0; mtaskId < static_cast<int>(assignment.size()); mtaskId ++) {
    const int owner = assignment[(size_t)mtaskId];
    Assert(owner >= 0 && owner < threadCount,
           "dense breakdown all-owner MTask %d has invalid owner %d", mtaskId, owner);
    laneCounts[(size_t)owner] += 1;
  }
  for (int owner = 0; owner < threadCount; owner ++) {
    const int unpaddedEnd = layout.laneOffsets[(size_t)owner] + laneCounts[(size_t)owner];
    layout.laneOffsets[(size_t)owner + 1] =
        ((unpaddedEnd + kRecordsPerAlignedOwnerLane - 1) / kRecordsPerAlignedOwnerLane)
        * kRecordsPerAlignedOwnerLane;
    Assert((layout.laneOffsets[(size_t)owner] % kRecordsPerAlignedOwnerLane) == 0
           && (layout.laneOffsets[(size_t)owner + 1] % kRecordsPerAlignedOwnerLane) == 0,
           "dense breakdown all-owner lane %d is not cache-line aligned", owner);
  }
  std::vector<int> next(layout.laneOffsets.begin(), layout.laneOffsets.end() - 1);
  for (int mtaskId = 0; mtaskId < static_cast<int>(assignment.size()); mtaskId ++) {
    const int owner = assignment[(size_t)mtaskId];
    const int recordIndex = next[(size_t)owner] ++;
    Assert(recordIndex >= layout.laneOffsets[(size_t)owner]
           && recordIndex < layout.laneOffsets[(size_t)owner + 1],
           "dense breakdown all-owner MTask %d escapes owner %d lane", mtaskId, owner);
    layout.recordIndexByMTask[(size_t)mtaskId] = recordIndex;
  }
  for (int owner = 0; owner < threadCount; owner ++) {
    Assert(next[(size_t)owner] == layout.laneOffsets[(size_t)owner] + laneCounts[(size_t)owner],
           "dense breakdown all-owner lane %d fill mismatch", owner);
  }
  layout.recordCount = layout.laneOffsets.back();
  return layout;
}

// Runtime token reduction models each worker's strict chain so a cross-worker
// token is only retained when the chain cannot carry its reachability.  B7 also
// needs an independent reduction of the real MTask DAG, without those synthetic
// chains, for slow-path epoch readiness.
static int mtReduceDenseRuntimeSuccsTransitive(std::vector<std::vector<int>>& runtimeSuccs,
                                               const std::vector<int>& assignment,
                                               bool injectWorkerChains = true) {
  const int nMTasks = static_cast<int>(runtimeSuccs.size());
  if (nMTasks <= 1) return 0;
  std::vector<std::vector<int>> effectiveSuccs = runtimeSuccs;
  int maxWorker = -1;
  for (int worker : assignment) maxWorker = std::max(maxWorker, worker);
  if (injectWorkerChains && maxWorker >= 0) {
    std::vector<int> previousOnWorker((size_t)maxWorker + 1, -1);
    for (int mtaskId = 0; mtaskId < nMTasks; mtaskId ++) {
      int worker = mtaskId < static_cast<int>(assignment.size()) ? assignment[(size_t)mtaskId] : -1;
      if (worker < 0 || worker > maxWorker) continue;
      int previous = previousOnWorker[(size_t)worker];
      if (previous >= 0) effectiveSuccs[(size_t)previous].push_back(mtaskId);
      previousOnWorker[(size_t)worker] = mtaskId;
    }
  }
  for (std::vector<int>& succs : effectiveSuccs) {
    std::sort(succs.begin(), succs.end());
    succs.erase(std::unique(succs.begin(), succs.end()), succs.end());
  }
  for (int from = 0; from < nMTasks; from ++) {
    for (int succ : effectiveSuccs[(size_t)from]) {
      if (succ <= from || succ >= nMTasks) return 0;
    }
  }
  const int wordCount = (nMTasks + 63) / 64;
  std::vector<std::vector<uint64_t>> reachable((size_t)nMTasks, std::vector<uint64_t>((size_t)wordCount, 0));
  auto setReachable = [&](int from, int to) {
    reachable[(size_t)from][(size_t)to >> 6] |= (uint64_t{1} << (to & 63));
  };
  auto isReachable = [&](int from, int to) -> bool {
    return (reachable[(size_t)from][(size_t)to >> 6] & (uint64_t{1} << (to & 63))) != 0;
  };
  for (int from = nMTasks - 1; from >= 0; from --) {
    for (int succ : effectiveSuccs[(size_t)from]) {
      setReachable(from, succ);
      for (int word = 0; word < wordCount; word ++) {
        reachable[(size_t)from][(size_t)word] |= reachable[(size_t)succ][(size_t)word];
      }
    }
  }
  int removed = 0;
  for (int from = 0; from < nMTasks; from ++) {
    std::vector<int> kept;
    kept.reserve(runtimeSuccs[(size_t)from].size());
    for (int succ : runtimeSuccs[(size_t)from]) {
      bool redundant = false;
      for (int alt : effectiveSuccs[(size_t)from]) {
        if (alt == succ) continue;
        if (isReachable(alt, succ)) {
          redundant = true;
          break;
        }
      }
      if (redundant) removed ++;
      else kept.push_back(succ);
    }
    runtimeSuccs[(size_t)from].swap(kept);
  }
  return removed;
}

static std::pair<std::vector<int>, int> mtBuildDensePackThreadsAssignment(const std::vector<MtDenseMTask>& mtasks,
                                                                          int threadCount) {
  if (threadCount < 1) threadCount = 1;
  // prefer the real dense scheduling cost when present; fall back to staticCost.
  auto costOf = [](const MtDenseMTask& m) -> int { return m.schedCost > 0 ? m.schedCost : m.staticCost; };
  std::vector<int> assignment(mtasks.size(), -1);
  std::vector<int> completion(mtasks.size(), 0);
  std::vector<int> busyUntil((size_t)threadCount, 0);
  std::vector<int> remainingPreds(mtasks.size(), 0);
  std::vector<int> priority(mtasks.size(), 0);
  for (size_t i = 0; i < mtasks.size(); i ++) remainingPreds[i] = static_cast<int>(mtasks[i].predMTasks.size());
  for (size_t i = mtasks.size(); i > 0; i --) {
    size_t mtaskId = i - 1;
    int bestSuccPriority = 0;
    for (int succ : mtasks[mtaskId].succMTasks) {
      if (succ >= 0 && succ < static_cast<int>(mtasks.size())) bestSuccPriority = std::max(bestSuccPriority, priority[(size_t)succ]);
    }
    priority[mtaskId] = costOf(mtasks[mtaskId]) + bestSuccPriority;
  }
  std::vector<int> ready;
  for (size_t i = 0; i < mtasks.size(); i ++) {
    if (remainingPreds[i] == 0) ready.push_back(static_cast<int>(i));
  }
  // GSIM_MT_DENSE_PACK_CCD_AFFINITY=<pct> (default 0 = off, byte-identical):
  // extra greedy penalty for producer->consumer mtask pairs the assignment
  // splits across the CCD/L3 boundary. Ground truth (lscpu, this machine):
  // 8 cores per L3 domain; a T16 run on cores 0-15 spans CCD0 (0-7) and
  // CCD1 (8-15); measured token latency 24.5ns same-CCD vs 290-322ns
  // cross-CCD, and the split doubles the state footprint pressure on the
  // 2x32MB L3. Soft term only: busyUntil/balance still dominate ordering.
  int ccdExtra = 0;
  { const char* e = std::getenv("GSIM_MT_DENSE_PACK_CCD_AFFINITY"); if (e && e[0]) { int v = std::atoi(e); if (v >= 0) ccdExtra = v; } }
  const int ccdSize = 8;
  int scheduled = 0;
  while (!ready.empty()) {
    int bestReadyIndex = -1;
    int bestMTask = -1;
    int bestWorker = 0;
    int bestTime = std::numeric_limits<int>::max();
    for (int readyIndex = 0; readyIndex < static_cast<int>(ready.size()); readyIndex ++) {
      int mtaskId = ready[(size_t)readyIndex];
      const MtDenseMTask& mtask = mtasks[(size_t)mtaskId];
      int workerLimit = mtask.workerZeroOnly ? 1 : threadCount;
      for (int worker = 0; worker < workerLimit; worker ++) {
        int timeBegin = busyUntil[(size_t)worker];
        for (int pred : mtask.predMTasks) {
          if (pred < 0 || pred >= static_cast<int>(mtasks.size())) continue;
          int predEnd = completion[(size_t)pred];
          int predWorker = assignment[(size_t)pred];
          if (predWorker >= 0 && predWorker != worker) {
            predEnd += (costOf(mtasks[(size_t)pred]) * 30) / 100;
            if (ccdExtra > 0 && (predWorker / ccdSize) != (worker / ccdSize))
              predEnd += (costOf(mtasks[(size_t)pred]) * ccdExtra) / 100;
          }
          if (predEnd > timeBegin) timeBegin = predEnd;
        }
        if (timeBegin < bestTime ||
            (timeBegin == bestTime && bestMTask >= 0 && priority[(size_t)mtaskId] > priority[(size_t)bestMTask]) ||
            (timeBegin == bestTime && bestMTask >= 0 && priority[(size_t)mtaskId] == priority[(size_t)bestMTask] && mtaskId < bestMTask)) {
          bestTime = timeBegin;
          bestReadyIndex = readyIndex;
          bestMTask = mtaskId;
          bestWorker = worker;
        }
      }
    }
    if (bestMTask < 0) break;
    const MtDenseMTask& mtask = mtasks[(size_t)bestMTask];
    assignment[(size_t)bestMTask] = bestWorker;
    int endTime = bestTime + std::max(1, costOf(mtask));
    completion[(size_t)bestMTask] = endTime;
    busyUntil[(size_t)bestWorker] = endTime;
    ready[(size_t)bestReadyIndex] = ready.back();
    ready.pop_back();
    scheduled ++;
    for (int succ : mtask.succMTasks) {
      if (succ < 0 || succ >= static_cast<int>(mtasks.size())) continue;
      int& deps = remainingPreds[(size_t)succ];
      deps --;
      if (deps == 0) ready.push_back(succ);
    }
  }
  int makespan = 0;
  for (int endTime : completion) makespan = std::max(makespan, endTime);
  if (scheduled != static_cast<int>(mtasks.size())) makespan = -1;
  return std::make_pair(assignment, makespan);
}


// list-scheduler that returns BOTH the worker assignment AND the schedule order (the
// sequence in which MTasks are scheduled). Renumbering MTask ids by this order makes the
// fixed-order runtime (which runs each worker's MTasks in ascending global id) execute them in
// earliest-start schedule order -- Verilator's static per-worker chain behavior -- with no
// runtime change. The order is a valid topological order (only ready MTasks are scheduled), so
// ids stay topo-monotone (succ>from) as the runtime protocol / transitive reduction require.
static void mtBuildDenseScheduleOrder(const std::vector<MtDenseMTask>& mtasks, int threadCount,
                                      std::vector<int>& outAssign, std::vector<int>& outOrder) {
  if (threadCount < 1) threadCount = 1;
  const int n = static_cast<int>(mtasks.size());
  auto costOf = [](const MtDenseMTask& m) -> int { return m.schedCost > 0 ? m.schedCost : m.staticCost; };
  outAssign.assign((size_t)n, -1);
  outOrder.clear(); outOrder.reserve((size_t)n);
  std::vector<long long> completion((size_t)n, 0);
  std::vector<long long> busyUntil((size_t)threadCount, 0);
  std::vector<int> remainingPreds((size_t)n, 0);
  std::vector<long long> priority((size_t)n, 0);
  for (int i = 0; i < n; i ++) remainingPreds[(size_t)i] = static_cast<int>(mtasks[(size_t)i].predMTasks.size());
  for (int i = n - 1; i >= 0; i --) {
    long long best = 0;
    for (int succ : mtasks[(size_t)i].succMTasks) if (succ >= 0 && succ < n) best = std::max(best, priority[(size_t)succ]);
    priority[(size_t)i] = costOf(mtasks[(size_t)i]) + best;
  }
  std::vector<int> ready;
  for (int i = 0; i < n; i ++) if (remainingPreds[(size_t)i] == 0) ready.push_back(i);
  // GSIM_MT_DENSE_PACK_CCD_AFFINITY=<pct> (default 0 = off, byte-identical):
  // extra greedy penalty when a producer->consumer mtask pair lands across a
  // CCD/L3 boundary. Ground truth (lscpu): 8 cores per L3 domain; T16 on cores
  // 0-15 spans CCD0(0-7)/CCD1(8-15); token latency 24.5ns same- vs 290-322ns
  // cross-CCD, and the split doubles 81MB of state across the 2x32MB L3s.
  // This is the SCHED_ORDER path - the assignment the dense recipe actually
  // uses (the PackThreads variant of the term sits in its own builder).
  int ccdExtra = 0;
  { const char* e = std::getenv("GSIM_MT_DENSE_PACK_CCD_AFFINITY"); if (e && e[0]) { int v = std::atoi(e); if (v >= 0) ccdExtra = v; } }
  const int ccdSize = 8;
  while (!ready.empty()) {
    int bestReadyIndex = -1, bestMTask = -1, bestWorker = 0;
    long long bestTime = std::numeric_limits<long long>::max();
    for (int ri = 0; ri < static_cast<int>(ready.size()); ri ++) {
      int mtaskId = ready[(size_t)ri];
      const MtDenseMTask& mtask = mtasks[(size_t)mtaskId];
      // Reserve thread 0 for worker0-only (pinned side-effect) MTasks: non-worker0 MTasks start
      // their thread search at 1 when threadCount>1, so the scheduler does not pile parallel work
      // onto thread 0 and then serialize the pinned load behind it (the 52x imbalance).
      int workerStart = mtask.workerZeroOnly ? 0 : (threadCount > 1 ? 1 : 0);
      int workerLimit = mtask.workerZeroOnly ? 1 : threadCount;
      for (int worker = workerStart; worker < workerLimit; worker ++) {
        long long timeBegin = busyUntil[(size_t)worker];
        for (int pred : mtask.predMTasks) {
          if (pred < 0 || pred >= n) continue;
          long long predEnd = completion[(size_t)pred];
          int predWorker = outAssign[(size_t)pred];
          if (predWorker >= 0 && predWorker != worker) {
            predEnd += (long long)(costOf(mtasks[(size_t)pred])) * 30 / 100;
            if (ccdExtra > 0 && (predWorker / ccdSize) != (worker / ccdSize))
              predEnd += (long long)(costOf(mtasks[(size_t)pred])) * ccdExtra / 100;
          }
          if (predEnd > timeBegin) timeBegin = predEnd;
        }
        if (timeBegin < bestTime ||
            (timeBegin == bestTime && bestMTask >= 0 && priority[(size_t)mtaskId] > priority[(size_t)bestMTask]) ||
            (timeBegin == bestTime && bestMTask >= 0 && priority[(size_t)mtaskId] == priority[(size_t)bestMTask] && mtaskId < bestMTask)) {
          bestTime = timeBegin; bestReadyIndex = ri; bestMTask = mtaskId; bestWorker = worker;
        }
      }
    }
    if (bestMTask < 0) break;
    outAssign[(size_t)bestMTask] = bestWorker;
    completion[(size_t)bestMTask] = bestTime + std::max(1, costOf(mtasks[(size_t)bestMTask]));
    busyUntil[(size_t)bestWorker] = completion[(size_t)bestMTask];
    outOrder.push_back(bestMTask);
    ready[(size_t)bestReadyIndex] = ready.back(); ready.pop_back();
    for (int succ : mtasks[(size_t)bestMTask].succMTasks) {
      if (succ < 0 || succ >= n) continue;
      if (-- remainingPreds[(size_t)succ] == 0) ready.push_back(succ);
    }
  }
  // Any unscheduled (shouldn't happen for a DAG) appended in id order.
  if (static_cast<int>(outOrder.size()) != n) {
    std::vector<char> seen((size_t)n, 0);
    for (int m : outOrder) seen[(size_t)m] = 1;
    for (int i = 0; i < n; i ++) if (!seen[(size_t)i]) { outOrder.push_back(i); if (outAssign[(size_t)i] < 0) outAssign[(size_t)i] = mtasks[(size_t)i].workerZeroOnly ? 0 : 0; }
  }
}

static MtDenseSchedule buildMtDenseSchedule(const std::map<int, MtTaskInfo>& tasks, bool codegenEnabled) {
  MtDenseSchedule schedule;
  schedule.codegenEnabled = codegenEnabled;
  schedule.taskCount = superId;
  schedule.succCppIds.assign((size_t)superId, std::vector<int>());
  schedule.predCppIds.assign((size_t)superId, std::vector<int>());
  std::vector<std::set<int>> succSets((size_t)superId);
  std::vector<std::set<int>> predSets((size_t)superId);
  std::set<std::tuple<int, int, std::string>> edgeKinds;

  {
  // Phase 1: Add dependency edges only.
  EmitPhaseTimer edgesTimer("Final.denseSched.edges");
  for (int cppId = 0; cppId < superId; cppId ++) {
    auto superIter = cppId2Super.find(cppId);
    if (superIter == cppId2Super.end() || !superIter->second) continue;
    SuperNode* super = superIter->second;
    mtDenseAddSuperEdges(schedule, succSets, predSets, edgeKinds, cppId, super->next, "dependency");
    mtDenseAddSuperEdges(schedule, succSets, predSets, edgeKinds, cppId, super->depNext, "dependency");
  }

  // When GSIM_MT_DENSE_FORWARD_ACTIVATION_ONLY=1, compute a dependency-only
  // topological rank and only add activation edges that are forward (rank[from] < rank[to]).
  // Backward activation edges are cross-cycle (next-cycle) activations that create false
  // cycles in the within-cycle SCC graph. Excluding them makes the graph acyclic.
  std::vector<int> depTopoRank;
  bool forwardActivationOnly = mtUseDenseForwardActivationOnly();
  if (forwardActivationOnly) {
    std::vector<int> depInDegree((size_t)superId, 0);
    for (int cppId = 0; cppId < superId; cppId ++) {
      for (int succ : succSets[(size_t)cppId]) {
        depInDegree[(size_t)succ] ++;
      }
    }
    std::vector<int> readyQueue;
    for (int cppId = 0; cppId < superId; cppId ++) {
      if (depInDegree[(size_t)cppId] == 0) readyQueue.push_back(cppId);
    }
    depTopoRank.assign((size_t)superId, 0);
    size_t rankCounter = 0;
    size_t queueHead = 0;
    while (queueHead < readyQueue.size()) {
      int node = readyQueue[queueHead ++];
      depTopoRank[(size_t)node] = static_cast<int>(rankCounter ++);
      for (int succ : succSets[(size_t)node]) {
        if (-- depInDegree[(size_t)succ] == 0) readyQueue.push_back(succ);
      }
    }
    Assert(rankCounter == (size_t)superId, "dependency graph has cycles; cannot compute topo rank for forward-activation filter");
  }

  // Phase 2: Add activation edges (filtered by topo rank if enabled).
  for (int cppId = 0; cppId < superId; cppId ++) {
    auto superIter = cppId2Super.find(cppId);
    if (superIter == cppId2Super.end() || !superIter->second) continue;
    SuperNode* super = superIter->second;
    for (Node* member : super->member) {
      if (!member) continue;
      for (int toCppId : member->nextActiveId) {
        if (forwardActivationOnly && (depTopoRank[(size_t)cppId] >= depTopoRank[(size_t)toCppId])) continue;
        mtDenseAddEdge(schedule, succSets, predSets, edgeKinds, cppId, toCppId, "active");
      }
      for (int toCppId : member->nextNeedActivate) {
        if (forwardActivationOnly && (depTopoRank[(size_t)cppId] >= depTopoRank[(size_t)toCppId])) continue;
        mtDenseAddEdge(schedule, succSets, predSets, edgeKinds, cppId, toCppId, "need_activate");
      }
    }
  }
  }
  {
  EmitPhaseTimer sccTimer("Final.denseSched.scc");

  for (int cppId = 0; cppId < superId; cppId ++) {
    schedule.succCppIds[(size_t)cppId].assign(succSets[(size_t)cppId].begin(), succSets[(size_t)cppId].end());
    schedule.predCppIds[(size_t)cppId].assign(predSets[(size_t)cppId].begin(), predSets[(size_t)cppId].end());
  }
  schedule.edgeCount = static_cast<int>(schedule.edges.size());

  std::vector<char> visited((size_t)superId, 0);
  std::vector<int> order;
  order.reserve((size_t)superId);
  for (int start = 0; start < superId; start ++) {
    if (visited[(size_t)start]) continue;
    std::vector<std::pair<int, size_t>> stack;
    stack.push_back({start, 0});
    visited[(size_t)start] = 1;
    while (!stack.empty()) {
      int node = stack.back().first;
      size_t& nextIndex = stack.back().second;
      const std::vector<int>& succs = schedule.succCppIds[(size_t)node];
      if (nextIndex < succs.size()) {
        int succ = succs[nextIndex ++];
        if (!visited[(size_t)succ]) {
          visited[(size_t)succ] = 1;
          stack.push_back({succ, 0});
        }
      } else {
        order.push_back(node);
        stack.pop_back();
      }
    }
  }

  std::vector<int> sccOf((size_t)superId, -1);
  for (int orderIndex = static_cast<int>(order.size()) - 1; orderIndex >= 0; orderIndex --) {
    int start = order[(size_t)orderIndex];
    if (sccOf[(size_t)start] >= 0) continue;
    int sccId = static_cast<int>(schedule.sccs.size());
    MtDenseScc scc;
    std::vector<int> stack;
    stack.push_back(start);
    sccOf[(size_t)start] = sccId;
    while (!stack.empty()) {
      int node = stack.back();
      stack.pop_back();
      scc.cppIds.push_back(node);
      auto taskIter = tasks.find(node);
      if (taskIter != tasks.end() && hasWorker0OnlyReasonDense(taskIter->second.serialReasons)) {
        scc.workerZeroOnly = true;
        scc.worker0OnlyTaskCount ++;
        schedule.worker0OnlyCppIds.push_back(node);
      }
      if (isAlwaysActive(node)) {
        scc.isAlwaysActive = true;
        scc.alwaysActiveTaskCount ++;
        schedule.alwaysActiveCppIds.push_back(node);
      }
      scc.staticCost += mtTaskEstimatedCost(tasks, node);
      auto superIter = cppId2Super.find(node);
      if (superIter != cppId2Super.end() && superIter->second) {
        scc.memberNodeCost += static_cast<int>(superIter->second->member.size());
      }
      const std::vector<int>& preds = schedule.predCppIds[(size_t)node];
      for (int pred : preds) {
        if (sccOf[(size_t)pred] < 0) {
          sccOf[(size_t)pred] = sccId;
          stack.push_back(pred);
        }
      }
    }
    std::sort(scc.cppIds.begin(), scc.cppIds.end());
    if (static_cast<int>(scc.cppIds.size()) > schedule.maxSccSize) schedule.maxSccSize = static_cast<int>(scc.cppIds.size());
    if (scc.cppIds.size() > 1) schedule.cycleSccCount ++;
    schedule.sccs.push_back(scc);
  }

  std::sort(schedule.worker0OnlyCppIds.begin(), schedule.worker0OnlyCppIds.end());
  schedule.worker0OnlyCppIds.erase(std::unique(schedule.worker0OnlyCppIds.begin(), schedule.worker0OnlyCppIds.end()), schedule.worker0OnlyCppIds.end());
  std::sort(schedule.alwaysActiveCppIds.begin(), schedule.alwaysActiveCppIds.end());
  schedule.alwaysActiveCppIds.erase(std::unique(schedule.alwaysActiveCppIds.begin(), schedule.alwaysActiveCppIds.end()), schedule.alwaysActiveCppIds.end());

  std::vector<std::set<int>> sccSuccSets(schedule.sccs.size());
  std::vector<std::set<int>> sccPredSets(schedule.sccs.size());
  for (int from = 0; from < superId; from ++) {
    int fromScc = sccOf[(size_t)from];
    for (int to : schedule.succCppIds[(size_t)from]) {
      int toScc = sccOf[(size_t)to];
      if (fromScc < 0 || toScc < 0 || fromScc == toScc) continue;
      if (sccSuccSets[(size_t)fromScc].insert(toScc).second) {
        sccPredSets[(size_t)toScc].insert(fromScc);
      }
    }
  }
  for (size_t sccId = 0; sccId < schedule.sccs.size(); sccId ++) {
    schedule.sccs[sccId].succSccs.assign(sccSuccSets[sccId].begin(), sccSuccSets[sccId].end());
    schedule.sccs[sccId].predSccs.assign(sccPredSets[sccId].begin(), sccPredSets[sccId].end());
  }

  for (const MtDenseEdge& edge : schedule.edges) {
    int fromScc = sccOf[(size_t)edge.fromCppId];
    int toScc = sccOf[(size_t)edge.toCppId];
    if (fromScc < 0 || toScc < 0) continue;
    if (fromScc == toScc) {
      MtDenseScc& scc = schedule.sccs[(size_t)fromScc];
      scc.internalEdgeCount ++;
      if (edge.kind == "dependency") scc.internalDependencyEdgeCount ++;
      else if (edge.kind == "active") scc.internalActiveEdgeCount ++;
      else if (edge.kind == "need_activate") scc.internalNeedActivateEdgeCount ++;
    } else {
      schedule.sccs[(size_t)fromScc].outgoingEdgeCount ++;
      schedule.sccs[(size_t)toScc].incomingEdgeCount ++;
    }
  }

  // Coarsen SCC DAG by merging chains (edges A->B where A has 1 succ
  // and B has 1 pred). This reduces layer depth and barrier count without
  // reducing parallelism. Inspired by Verilator's V3OrderParallel edge contraction.
  }
  {
  EmitPhaseTimer coarsenTimer("Final.denseSched.coarsen");
  {
    int n = static_cast<int>(schedule.sccs.size());
    std::vector<int> parent(n);
    for (int i = 0; i < n; i++) parent[i] = i;
    auto find = [&](int x) -> int {
      while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
      return x;
    };
    auto effOutDeg = [&](int root) -> int {
      std::set<int> uniqueSuccs;
      for (int s : schedule.sccs[root].succSccs) { int rs = find(s); if (rs != root) uniqueSuccs.insert(rs); }
      return static_cast<int>(uniqueSuccs.size());
    };
    auto effInDeg = [&](int root) -> int {
      std::set<int> uniquePreds;
      for (int p : schedule.sccs[root].predSccs) { int rp = find(p); if (rp != root) uniquePreds.insert(rp); }
      return static_cast<int>(uniquePreds.size());
    };
    bool changed = true;
    while (changed) {
      changed = false;
      for (int i = 0; i < n; i++) {
        if (find(i) != i) continue;
        if (effOutDeg(i) != 1) continue;
        int succ = -1;
        for (int s : schedule.sccs[i].succSccs) { int rs = find(s); if (rs != i) { succ = rs; break; } }
        if (succ < 0 || find(succ) != succ) continue;
        if (effInDeg(succ) != 1) continue;
        if (schedule.sccs[i].workerZeroOnly != schedule.sccs[succ].workerZeroOnly) continue;
        parent[i] = succ;
        changed = true;
      }
    }
    std::vector<int> remap(n, -1);
    int newCount = 0;
    for (int i = 0; i < n; i++) { if (find(i) == i) remap[i] = newCount++; }
    if (newCount < n) {
      std::vector<MtDenseScc> newSccs(newCount);
      for (int i = 0; i < n; i++) {
        int newId = remap[find(i)];
        MtDenseScc& ns = newSccs[newId];
        ns.cppIds.insert(ns.cppIds.end(), schedule.sccs[i].cppIds.begin(), schedule.sccs[i].cppIds.end());
        ns.staticCost += schedule.sccs[i].staticCost;
        ns.memberNodeCost += schedule.sccs[i].memberNodeCost;
        ns.workerZeroOnly = schedule.sccs[i].workerZeroOnly;
        ns.worker0OnlyTaskCount += schedule.sccs[i].worker0OnlyTaskCount;
        ns.alwaysActiveTaskCount += schedule.sccs[i].alwaysActiveTaskCount;
        ns.isAlwaysActive = ns.isAlwaysActive || schedule.sccs[i].isAlwaysActive;
      }
      for (auto& ns : newSccs) std::sort(ns.cppIds.begin(), ns.cppIds.end());
      std::vector<std::set<int>> newSuccSets(newCount), newPredSets(newCount);
      for (int i = 0; i < n; i++) {
        int fromNew = remap[find(i)];
        for (int succ : schedule.sccs[i].succSccs) {
          int toRoot = find(succ);
          if (toRoot == find(i)) continue;
          int toNew = remap[toRoot];
          newSuccSets[fromNew].insert(toNew);
          newPredSets[toNew].insert(fromNew);
        }
      }
      for (int i = 0; i < newCount; i++) {
        newSccs[i].succSccs.assign(newSuccSets[i].begin(), newSuccSets[i].end());
        newSccs[i].predSccs.assign(newPredSets[i].begin(), newPredSets[i].end());
        newSccs[i].incomingEdgeCount = static_cast<int>(newSccs[i].predSccs.size());
        newSccs[i].outgoingEdgeCount = static_cast<int>(newSccs[i].succSccs.size());
      }
      schedule.sccs = newSccs;
    }
  }
  }

  {
  EmitPhaseTimer layersTimer("Final.denseSched.layers");
  std::vector<int> indegree(schedule.sccs.size(), 0);
  std::set<int> ready;
  for (size_t sccId = 0; sccId < schedule.sccs.size(); sccId ++) {
    indegree[sccId] = static_cast<int>(schedule.sccs[sccId].predSccs.size());
    if (indegree[sccId] == 0) ready.insert(static_cast<int>(sccId));
  }
  while (!ready.empty()) {
    int worker0Scc = -1;
    for (int sccId : ready) {
      if (schedule.sccs[(size_t)sccId].workerZeroOnly) {
        worker0Scc = sccId;
        break;
      }
    }
    MtDenseLayer layer;
    if (worker0Scc >= 0) {
      layer.sccIds.push_back(worker0Scc);
      layer.workerZeroOnly = true;
    } else {
      for (int sccId : ready) layer.sccIds.push_back(sccId);
    }
    std::sort(layer.sccIds.begin(), layer.sccIds.end(), [&](int lhs, int rhs) {
      return schedule.sccs[(size_t)lhs].cppIds.front() < schedule.sccs[(size_t)rhs].cppIds.front();
    });
    for (int sccId : layer.sccIds) {
      ready.erase(sccId);
      schedule.topoSccOrder.push_back(sccId);
      layer.taskCount += static_cast<int>(schedule.sccs[(size_t)sccId].cppIds.size());
      layer.staticCost += schedule.sccs[(size_t)sccId].staticCost;
    }
    for (int sccId : layer.sccIds) {
      for (int succScc : schedule.sccs[(size_t)sccId].succSccs) {
        indegree[(size_t)succScc] --;
        if (indegree[(size_t)succScc] == 0) ready.insert(succScc);
      }
    }
    schedule.layers.push_back(layer);
  }
  // Split wide layers so no single worker function exceeds Clang limits.
  {
    const int MAX_SCCS_PER_WORKER = 256;
    std::vector<MtDenseLayer> splitLayers;
    for (const MtDenseLayer& layer : schedule.layers) {
      if ((int)layer.sccIds.size() <= MAX_SCCS_PER_WORKER) {
        splitLayers.push_back(layer);
      } else {
        for (int i = 0; i < (int)layer.sccIds.size(); i += MAX_SCCS_PER_WORKER) {
          MtDenseLayer sub;
          int end = std::min(i + MAX_SCCS_PER_WORKER, (int)layer.sccIds.size());
          sub.sccIds.assign(layer.sccIds.begin() + i, layer.sccIds.begin() + end);
          sub.workerZeroOnly = layer.workerZeroOnly;
          for (int sccId : sub.sccIds) {
            sub.taskCount += static_cast<int>(schedule.sccs[(size_t)sccId].cppIds.size());
            sub.staticCost += schedule.sccs[(size_t)sccId].staticCost;
          }
          splitLayers.push_back(sub);
        }
      }
    }
    schedule.layers = splitLayers;
  }
  }
  // Dense dependency executor: form MTasks and assign them to worker threads.
  // Default path: fixed 30-SCC topological chunking + round-robin assignment.
  {
  EmitPhaseTimer mtaskBuildTimer("Final.denseSched.mtaskBuild");
  {
    int threadCount = 8;
    const char* threadsEnv = std::getenv("GSIM_THREADS");
    if (threadsEnv != nullptr && threadsEnv[0] != '\0') threadCount = std::atoi(threadsEnv);
    if (threadCount < 1) threadCount = 1;

    if (std::getenv("GSIM_MT_DENSE_VCONTRACT") && std::getenv("GSIM_MT_DENSE_VCONTRACT")[0] == '1') {

      // GSIM_MT_DENSE_VCONTRACT_MAXMT_AUTO=1: in-generation MAXMT search. The
      // contract builder rebuilds its quotient graph from schedule.sccs on every
      // call (same pristine-restart mechanism the two-pass POLICY=auto search
      // relies on), so we can evaluate a ladder of MAXMT candidates on the same
      // constructed graph and pick by the calibrated floor model
      // score = maxW*A + cross*B*L(threads) - U-shaped in MAXMT: coarse schedules
      // lose on balance (maxW), fine schedules lose on sync (crossEdges). The
      // winner is exported via setenv so the single real contraction below (and,
      // under POLICY=auto, both of its passes) runs with it. Default off; when
      // off, byte-identical output is unaffected.
      if (std::getenv("GSIM_MT_DENSE_VCONTRACT_MAXMT_AUTO") &&
          std::getenv("GSIM_MT_DENSE_VCONTRACT_MAXMT_AUTO")[0] == '1') {
        auto probeInvariants = [](const std::vector<MtDenseMTask>& mts,
                                  const std::vector<int>& assign,
                                  std::vector<int>& workerCosts) {
          workerCosts.assign(64, 0);
          long cross = 0;
          for (size_t i = 0; i < mts.size(); i ++) {
            int w = i < assign.size() ? assign[(size_t)i] : 0;
            if (w >= 0 && w < 64) workerCosts[(size_t)w] += mts[i].staticCost;
            for (int succ : mts[i].succMTasks) {
              int ws = succ >= 0 && succ < (int)assign.size() ? assign[(size_t)succ] : -1;
              if (ws >= 0 && ws != w) cross ++;
            }
          }
          return cross;
        };
        // Level-synchronous critical invariant: the dense executor advances
        // level by level, so a worker with no task at a level still waits for
        // the level's straggler. The floor is sum over levels of the max
        // per-worker work in that level - NOT the global per-worker max.
        auto probeLevelSum = [](const std::vector<MtDenseMTask>& mts, const std::vector<int>& assign) {
          const size_t n = mts.size();
          std::vector<int> indeg(n, 0);
          for (size_t i = 0; i < n; i ++) indeg[i] = (int)mts[i].predMTasks.size();
          std::vector<int> q; for (size_t i = 0; i < n; i ++) if (indeg[i] == 0) q.push_back((int)i);
          long levelSum = 0; size_t done = 0;
          std::vector<int> lw(64, 0);
          while (!q.empty()) {
            for (int w = 0; w < 64; w ++) lw[(size_t)w] = 0;
            long mx = 0;
            for (int u : q) {
              int w = u < (int)assign.size() ? assign[(size_t)u] : 0;
              if (w >= 0 && w < 64) { lw[(size_t)w] += mts[(size_t)u].staticCost; if (lw[(size_t)w] > mx) mx = lw[(size_t)w]; }
            }
            levelSum += mx; done += q.size();
            std::vector<int> nq;
            for (int u : q) for (int v : mts[(size_t)u].succMTasks) if (-- indeg[(size_t)v] == 0) nq.push_back(v);
            q.swap(nq);
          }
          return done == n ? levelSum : -1;  // -1: cycle guard (should not happen post-contraction)
        };
        // (the maxW+cross floor score was retired: it misranked both validated RTLs)
        std::vector<int> probeCands;
        { static const int probeMults[] = {25, 50, 75, 100, 150, 200, 300}; for (int m : probeMults) probeCands.push_back(m * threadCount); }
        fprintf(stderr, "[maxmt-auto] searching MAXMT ladder for threads=%d\n", threadCount);
        int bestVal = -1; double bestScore = 1e30; long bestMaxW = 0, bestCross = 0;
        for (int cand : probeCands) {
          char cbuf[32]; snprintf(cbuf, sizeof cbuf, "%d", cand);
          setenv("GSIM_MT_DENSE_VCONTRACT_MAXMT", cbuf, 1);
          MtDenseSchedule probeSched = schedule;   // pristine copy; builder rebuilds from .sccs
          probeSched.mtasks = mtBuildDenseMTasksVerilatorContract(probeSched, threadCount);
          vcAutoAssign(probeSched, threadCount);
          std::vector<int> wc; long cross = probeInvariants(probeSched.mtasks, probeSched.mtaskThreadAssign, wc);
          long maxW = *std::max_element(wc.begin(), wc.end());
          long lvlSum = probeLevelSum(probeSched.mtasks, probeSched.mtaskThreadAssign);
          // Pick rule (validated  on two RTLs): the level-synchronous
          // straggler sum is the primary physical invariant (v86-T16: argmin lvlSum
          // = 1200 = measured optimum, zero-fitting). The old maxW+cross floor
          // misranked both RTLs (picked 2400/1000 where 2000/1200 measured best).
          // lvlSum can overestimate schedules that bounded lookahead rescues, so
          // cross breaks near-ties and the recommended protocol prunes to the
          // top-2 candidates and confirms by a real bench.
          double s = (double)lvlSum;
          fprintf(stderr, "[maxmt-auto]   cand=%d mtasks=%zu maxW=%ld cross=%ld lvlSum=%ld score=%.1f\n",
                  cand, probeSched.mtasks.size(), maxW, cross, lvlSum, s);
          if (s < bestScore || (s == bestScore && cross < bestCross)) { bestScore = s; bestVal = cand; bestMaxW = maxW; bestCross = cross; }
        }
        char wbuf[32]; snprintf(wbuf, sizeof wbuf, "%d", bestVal);
        setenv("GSIM_MT_DENSE_VCONTRACT_MAXMT", wbuf, 1);
        fprintf(stderr, "[maxmt-auto] picked MAXMT=%d (maxW=%ld cross=%ld score=%.1f)\n",
                bestVal, bestMaxW, bestCross, bestScore);
      }
      if (std::getenv("GSIM_MT_DENSE_VCONTRACT_POLICY") &&
          std::strncmp(std::getenv("GSIM_MT_DENSE_VCONTRACT_POLICY"), "auto", 4) == 0) {
        // Two-pass schedule search: plain then compacted, pick by the calibrated
        // two-term floor model. The contract builder rebuilds its quotient graph
        // from schedule.sccs on every call, so the second pass starts pristine.
        auto scheduleInvariants = [](const std::vector<MtDenseMTask>& mts,
                                     const std::vector<int>& assign,
                                     std::vector<int>& workerCosts) {
          workerCosts.assign(64, 0);
          long cross = 0;
          for (size_t i = 0; i < mts.size(); i ++) {
            int w = i < assign.size() ? assign[(size_t)i] : 0;
            if (w >= 0 && w < 64) workerCosts[(size_t)w] += mts[i].staticCost;
            for (int succ : mts[i].succMTasks) {
              int ws = succ >= 0 && succ < (int)assign.size() ? assign[(size_t)succ] : -1;
              if (ws >= 0 && ws != w) cross ++;
            }
          }
          return cross;
        };
        auto scoreOf = [](long maxW, long cross, int threads) {
          const double A = 2.647e-3;              // us per static-cost unit (work rate, machine-fit)
          const double B = 2.198e-5;              // us per cross-thread edge (same-CCD token)
          const double L = threads <= 16 ? 1.0 : 8.0;  // cross-CCD latency weight (290/24.5 ~ 12, fit 8)
          return maxW * A + cross * B * L;
        };
        MtDenseSchedule plainSched = schedule, compactSched = schedule;
        vcAutoPassReset();
        plainSched.mtasks = mtBuildDenseMTasksVerilatorContract(plainSched, threadCount);
        vcAutoAssign(plainSched, threadCount);
        std::vector<int> wcP; long crossP = scheduleInvariants(plainSched.mtasks, plainSched.mtaskThreadAssign, wcP);
        long maxWP = *std::max_element(wcP.begin(), wcP.end());
        vcAutoPassAdvance();
        compactSched.mtasks = mtBuildDenseMTasksVerilatorContract(compactSched, threadCount);
        vcAutoAssign(compactSched, threadCount);
        std::vector<int> wcC; long crossC = scheduleInvariants(compactSched.mtasks, compactSched.mtaskThreadAssign, wcC);
        long maxWC = *std::max_element(wcC.begin(), wcC.end());
        double sP = scoreOf(maxWP, crossP, threadCount);
        double sC = scoreOf(maxWC, crossC, threadCount);
        // Score rule (recalibrated ): the earlier conjunction with a
        // threads>16 prior was calibrated on two cross-graph contaminated T16
        // A/Bs; the CLEAN same-graph A/B (determinism-fixed binary, identical
        // graph verified) FLIPPED the T16 verdict to compact -8.4% (5-pair
        // non-overlapping). The pure score model had picked COMPACT on T16 all
        // along - the model was right, the falsifying data was wrong. The score
        // stands alone; the physical invariants (maxW, crossEdges) carry the
        // input-adaptivity.
        bool pickCompact = sC < sP;
        fprintf(stderr, "[vcontract-policy] auto: plain(maxW=%ld cross=%ld score=%.1fus) vs compact(maxW=%ld cross=%ld score=%.1fus) -> %s\n",
                maxWP, crossP, sP, maxWC, crossC, sC, pickCompact ? "COMPACT" : "PLAIN");
        schedule = pickCompact ? compactSched : plainSched;
        vcAutoPassFinish(pickCompact);
      } else {
        schedule.mtasks = mtBuildDenseMTasksVerilatorContract(schedule, threadCount);
      }
    } else {
      schedule.mtasks = mtBuildDenseMTasks(schedule, mtUseDenseSplitWorker0MTasks());
    }
    // Executes each worker's MTasks in schedule order (Verilator static per-worker chain). Only
    // reorders ids; keeps topo-monotonicity. Also sets the assignment from the scheduler.
    bool schedOrder = false;
    { const char* e = std::getenv("GSIM_MT_DENSE_SCHED_ORDER"); if (e) schedOrder = e[0] && e[0] != '0'; }
    std::vector<int> schedOrderAssign;
    if (schedOrder && static_cast<int>(schedule.mtasks.size()) > 1) {
      std::vector<int> assignTmp, orderTmp;
      mtBuildDenseScheduleOrder(schedule.mtasks, threadCount, assignTmp, orderTmp);
      const int n = static_cast<int>(schedule.mtasks.size());
      if (static_cast<int>(orderTmp.size()) == n) {
        std::vector<int> newId((size_t)n, -1);
        for (int newPos = 0; newPos < n; newPos ++) newId[(size_t)orderTmp[(size_t)newPos]] = newPos;
        std::vector<MtDenseMTask> reordered((size_t)n);
        schedOrderAssign.assign((size_t)n, 0);
        for (int oldId = 0; oldId < n; oldId ++) {
          int ni = newId[(size_t)oldId];
          reordered[(size_t)ni] = schedule.mtasks[(size_t)oldId];
          schedOrderAssign[(size_t)ni] = assignTmp[(size_t)oldId];
        }
        // Remap pred/succ MTask ids to new numbering.
        for (int ni = 0; ni < n; ni ++) {
          for (int& p : reordered[(size_t)ni].predMTasks) if (p >= 0 && p < n) p = newId[(size_t)p];
          for (int& s : reordered[(size_t)ni].succMTasks) if (s >= 0 && s < n) s = newId[(size_t)s];
          std::sort(reordered[(size_t)ni].predMTasks.begin(), reordered[(size_t)ni].predMTasks.end());
          std::sort(reordered[(size_t)ni].succMTasks.begin(), reordered[(size_t)ni].succMTasks.end());
        }
        schedule.mtasks.swap(reordered);
        fprintf(stderr, "[mt-dense-schedorder] renumbered %d MTasks by list-schedule order\n", n);
      }
    }
    int nMTasks = static_cast<int>(schedule.mtasks.size());
    schedule.mtaskThreadAssign.resize(nMTasks);
    if (!schedOrderAssign.empty() && static_cast<int>(schedOrderAssign.size()) == nMTasks) {
      // schedule-order ids -> use the list-scheduler's own (earliest-free)
      // assignment, co-designed with the order.
      for (int i = 0; i < nMTasks; i ++) schedule.mtaskThreadAssign[(size_t)i] = schedule.mtasks[(size_t)i].workerZeroOnly ? 0 : schedOrderAssign[(size_t)i];
    }
    if (!schedOrderAssign.empty() && static_cast<int>(schedOrderAssign.size()) == nMTasks) {
      // schedule-order assignment already installed above; nothing to do.
    } else {
      for (int i = 0; i < nMTasks; i ++) schedule.mtaskThreadAssign[(size_t)i] = schedule.mtasks[(size_t)i].workerZeroOnly ? 0 : (i % threadCount);
    }
    // verify MTask ids are topologically monotone (every edge from<to) after any renumber.
    for (int mi = 0; mi < nMTasks; mi ++) {
      for (int s : schedule.mtasks[(size_t)mi].succMTasks) {
        Assert(s > mi, "dense MTask id order not topo-monotone: edge %d->%d", mi, s);
      }
    }
  }
  }

  if (!codegenEnabled) {
    schedule.valid = false;
    schedule.fallbackReason = "codegen_disabled";
  } else if (schedule.cycleSccCount != 0) {
    schedule.valid = false;
    schedule.fallbackReason = "cycle_scc";
  } else if (schedule.topoSccOrder.size() != schedule.sccs.size()) {
    schedule.valid = false;
    schedule.fallbackReason = "scc_topology_incomplete";
  } else {
    schedule.valid = true;
    schedule.fallbackReason = "none";
  }
  return schedule;
}





void graph::dumpMtScheduleJson() {
  std::string baseName = globalConfig.InputBaseName.empty() ? name : globalConfig.InputBaseName;
  std::string path = globalConfig.OutputDir + "/" + baseName + "_mt_schedule.json";
  FILE* fp = std::fopen(path.c_str(), "w");
  Assert(fp != nullptr, "failed to open mt schedule json %s", path.c_str());
  struct timeval mtReportTimerStart = getTime();
  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapForInvocation();
  std::set<std::string> allStateTargetNames = collectAllMtStateTargetNames(mtTasks);
  MtStateTargetWriterUniverse stateTargetWriterUniverse = collectMtStateTargetWriters(mtTasks);

  fprintf(fp, "{\n");
  fprintf(fp, "  \"format\": \"gsim.mt-schedule.v1\",\n");
  fprintf(fp, "  \"tasks\": [\n");

  for (int cppId = 0; cppId < superId; cppId ++) {
    SuperNode* super = cppId2Super[cppId];
    int activeWord;
    uint64_t activeMask;
    std::tie(activeWord, activeMask) = setIdxMask(cppId);

    MtTaskInfo& mtTask = mtTasks[cppId];
    const MtBoundaryInfo& boundary = mtTask.boundary;
    std::set<int> predCppIds;
    std::set<int> succCppIds;
    std::set<int> activeFanout;

    addCppIdsIfExecutable(predCppIds, super->prev);
    addCppIdsIfExecutable(predCppIds, super->depPrev);
    addCppIdsIfExecutable(succCppIds, super->next);
    addCppIdsIfExecutable(succCppIds, super->depNext);

    for (Node* member : super->member) {
      for (int nextCppId : member->nextNeedActivate) {
        if (nextCppId >= 0) activeFanout.insert(nextCppId);
      }
      if (mtUseActivationEventTraceCodegen()) {
        for (int nextCppId : member->nextActiveId) {
          if (nextCppId >= 0) activeFanout.insert(nextCppId);
        }
      }
    }
    if (mtUseActivationEventTraceCodegen() && super->superType == SUPER_ASYNC_RESET) {
      SuperNode* resetSourceSuper = nullptr;
      for (SuperNode* candidate : allReset) {
        if (candidate->superType == SUPER_ASYNC_RESET && candidate->resetNode == super->resetNode) resetSourceSuper = candidate;
      }
      Assert(resetSourceSuper != nullptr, "missing async reset trace fanout source for cppId %d", cppId);
      for (Node* member : resetSourceSuper->member) {
        Node* source = member->type == NODE_REG_RESET ? member->getResetSrc() : member;
        for (Node* next : source->next) {
          if (next->super->cppId >= 0) activeFanout.insert(next->super->cppId);
        }
      }
    }

    fprintf(fp, "    {\n");
    fprintf(fp, "      \"cpp_id\": %d,\n", cppId);
    fprintf(fp, "      \"scan_index\": %d,\n", cppId);
    fprintf(fp, "      \"super_id\": %d,\n", super->cppId);  // cppId is pinned/content-stable (super->id drifts)
    fprintf(fp, "      \"super_type\": \"%s\",\n", superTypeName(super->superType));
    fprintf(fp, "      \"task_kind\": \"%s\",\n", mtTask.taskKind.c_str());
    fprintf(fp, "      \"serial_reasons\": ");
    dumpJsonStringArray(fp, mtTask.serialReasons);
    fprintf(fp, ",\n");
    fprintf(fp, "      \"active_word\": %d,\n", activeWord);
    fprintf(fp, "      \"active_mask\": \"0x%" PRIx64 "\",\n", activeMask);
    fprintf(fp, "      \"node_kinds\": {");
    bool firstKind = true;
    for (auto iter : boundary.nodeKinds) {
      if (!firstKind) fprintf(fp, ", ");
      firstKind = false;
      fprintf(fp, "\"%s\": %d", iter.first.c_str(), iter.second);
    }
    fprintf(fp, "},\n");

    fprintf(fp, "      \"pred_cpp_ids\": ");
    dumpJsonIntArray(fp, predCppIds);
    fprintf(fp, ",\n");
    fprintf(fp, "      \"succ_cpp_ids\": ");
    dumpJsonIntArray(fp, succCppIds);
    fprintf(fp, ",\n");
    fprintf(fp, "      \"active_fanout\": ");
    dumpJsonIntArray(fp, activeFanout);
    fprintf(fp, ",\n");
    if (mtUseActivationEventTraceCodegen()) {
      fprintf(fp, "      \"always_active\": %s,\n", isAlwaysActive(cppId) ? "true" : "false");
    }

    fprintf(fp, "      \"boundary\": {\n");
    fprintf(fp, "        \"has_state_update\": %s,\n", boundary.hasStateUpdate ? "true" : "false");
    fprintf(fp, "        \"has_memory_write\": %s,\n", boundary.hasMemoryWrite ? "true" : "false");
    fprintf(fp, "        \"has_reset\": %s,\n", boundary.hasReset ? "true" : "false");
    fprintf(fp, "        \"has_external\": %s,\n", boundary.hasExternal ? "true" : "false");
    fprintf(fp, "        \"has_special\": %s,\n", boundary.hasSpecial ? "true" : "false");
    fprintf(fp, "        \"clock_names\": ");
    dumpJsonStringArray(fp, boundary.clockNames);
    fprintf(fp, "\n");
    fprintf(fp, "      },\n");

    std::string rhsTimingClass = mtStateUpdateRhsTimingClass(boundary);
    std::string rhsTimingEvidence = mtStateUpdateRhsTimingEvidence(boundary, rhsTimingClass);
    bool rhsReadsSameCycleTarget = mtStateUpdateHasSameCycleTargetRead(boundary, allStateTargetNames);
    bool hasMemoryOrDynamicArray = mtStateUpdateHasMemoryOrDynamicArray(boundary);
    bool hasExternalOrSpecial = mtStateUpdateHasExternalOrSpecial(boundary);
    bool activationCanUseDelta = mtStateUpdateActivationCanUseDelta(boundary);
    std::vector<std::string> stateUpdateBlockReasons = mtStateUpdateBlockReasons(boundary, super, rhsTimingClass, rhsReadsSameCycleTarget);
    std::string stateUpdateCandidateKind = mtStateUpdateCandidateKind(boundary, stateUpdateBlockReasons);
    MtStateTargetWriterInfo stateTargetWriterInfo = mtStateUpdateWriterInfo(boundary, stateTargetWriterUniverse.targetWriters);
    std::string targetWriterConflictKind = mtStateUpdateTargetWriterConflictKind(
        boundary, stateTargetWriterInfo, stateTargetWriterUniverse.hasIncompleteWriterUniverse);
    std::string targetWriterProof = mtStateUpdateTargetWriterProof(targetWriterConflictKind);
    std::vector<std::string> runtimeBlockReasons = mtStateUpdateRuntimeBlockReasons(
        stateUpdateCandidateKind, targetWriterConflictKind, stateTargetWriterInfo);
    bool runtimeSafeCandidate = stateUpdateCandidateKind == "safe_candidate" && runtimeBlockReasons.empty();
    fprintf(fp, "      \"state_update\": {\n");
    fprintf(fp, "        \"has_state_update\": %s,\n", boundary.hasStateUpdate ? "true" : "false");
    fprintf(fp, "        \"state_target_names\": ");
    dumpJsonStringArray(fp, boundary.stateTargetNames);
    fprintf(fp, ",\n");
    fprintf(fp, "        \"state_target_count\": %zu,\n", boundary.stateTargetNames.size());
    fprintf(fp, "        \"single_target\": %s,\n", boundary.stateTargetNames.size() == 1 ? "true" : "false");
    fprintf(fp, "        \"rhs_timing_class\": \"%s\",\n", rhsTimingClass.c_str());
    fprintf(fp, "        \"rhs_timing_evidence\": \"%s\",\n", rhsTimingEvidence.c_str());
    fprintf(fp, "        \"rhs_reads_state_targets\": ");
    dumpJsonStringArray(fp, boundary.rhsReadStateTargetNames);
    fprintf(fp, ",\n");
    fprintf(fp, "        \"rhs_reads_same_cycle_target\": %s,\n", rhsReadsSameCycleTarget ? "true" : "false");
    fprintf(fp, "        \"has_reset_behavior\": %s,\n", boundary.hasReset ? "true" : "false");
    fprintf(fp, "        \"has_async_reset_behavior\": %s,\n", boundary.hasAsyncReset ? "true" : "false");
    fprintf(fp, "        \"has_memory_or_dynamic_array\": %s,\n", hasMemoryOrDynamicArray ? "true" : "false");
    fprintf(fp, "        \"has_external_or_special\": %s,\n", hasExternalOrSpecial ? "true" : "false");
    fprintf(fp, "        \"activation_fanout_count\": %zu,\n", activeFanout.size());
    fprintf(fp, "        \"activation_can_use_delta\": %s,\n", activationCanUseDelta ? "true" : "false");
    fprintf(fp, "        \"candidate_kind\": \"%s\",\n", stateUpdateCandidateKind.c_str());
    fprintf(fp, "        \"block_reasons\": ");
    dumpJsonStringArray(fp, stateUpdateBlockReasons);
    fprintf(fp, "\n");
    fprintf(fp, "      },\n");

    fprintf(fp, "      \"state_update_group\": {\n");
    fprintf(fp, "        \"local_safe_candidate\": %s,\n", stateUpdateCandidateKind == "safe_candidate" ? "true" : "false");
    fprintf(fp, "        \"runtime_safe_candidate\": %s,\n", runtimeSafeCandidate ? "true" : "false");
    fprintf(fp, "        \"target_writer_count\": %d,\n", stateTargetWriterInfo.writerCount);
    fprintf(fp, "        \"target_multi_target_writer_count\": %d,\n", stateTargetWriterInfo.multiTargetWriterCount);
    fprintf(fp, "        \"target_writer_universe_complete\": %s,\n",
            stateTargetWriterUniverse.hasIncompleteWriterUniverse ? "false" : "true");
    fprintf(fp, "        \"target_writer_cpp_ids\": ");
    dumpJsonIntArray(fp, stateTargetWriterInfo.writerCppIds);
    fprintf(fp, ",\n");
    fprintf(fp, "        \"target_writer_conflict_kind\": \"%s\",\n", targetWriterConflictKind.c_str());
    fprintf(fp, "        \"target_writer_proof\": \"%s\",\n", targetWriterProof.c_str());
    fprintf(fp, "        \"runtime_block_reasons\": ");
    dumpJsonStringArray(fp, runtimeBlockReasons);
    fprintf(fp, "\n");
    fprintf(fp, "      }\n");
    fprintf(fp, "    }%s\n", cppId + 1 == superId ? "" : ",");
  }

  fprintf(fp, "  ]\n");
  fprintf(fp, "}\n");
  fclose(fp);
  printf("[mt-schedule] wrote %d tasks to %s\n", superId, path.c_str());
  logMtReportTimer("schedule", mtReportTimerStart, getTime());
}

void graph::dumpMtDenseScheduleJson() {
  std::string baseName = globalConfig.InputBaseName.empty() ? name : globalConfig.InputBaseName;
  std::string path = globalConfig.OutputDir + "/" + baseName + "_mt_dense_schedule.json";
  FILE* fp = std::fopen(path.c_str(), "w");
  Assert(fp != nullptr, "failed to open mt dense schedule json %s", path.c_str());
  struct timeval mtReportTimerStart = getTime();
  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapForInvocation();
  MtDenseSchedule schedule = buildMtDenseSchedule(mtTasks, mtUseDenseExecutorCodegen());
  if (schedule.codegenEnabled) {
    mtDenseScheduleCache = schedule;
    mtDenseScheduleCacheValid = true;
  }

  int maxLayerWidth = 0;
  int worker0OnlyLayerCount = 0;
  for (const MtDenseLayer& layer : schedule.layers) {
    if (layer.taskCount > maxLayerWidth) maxLayerWidth = layer.taskCount;
    if (layer.workerZeroOnly) worker0OnlyLayerCount ++;
  }

  int denseMTaskCount = static_cast<int>(schedule.mtasks.size());
  int denseMTaskEdgeCount = 0;
  int denseMTaskCrossThreadEdgeCount = 0;
  int denseMTaskSameThreadEdgeCount = 0;
  int denseMTaskMaxPredCount = 0;
  int denseMTaskMaxSuccCount = 0;
  int denseMTaskMaxStaticCost = 0;
  int denseMTaskMaxTaskCount = 0;
  int denseThreadCount = 0;
  for (int worker : schedule.mtaskThreadAssign) denseThreadCount = std::max(denseThreadCount, worker + 1);
  std::vector<int> denseWorkerStaticCosts((size_t)denseThreadCount, 0);
  std::vector<int> denseWorkerTaskCounts((size_t)denseThreadCount, 0);
  std::vector<int> denseWorkerMTaskCounts((size_t)denseThreadCount, 0);
  std::vector<int> denseMTaskOrder((size_t)denseMTaskCount);
  for (int i = 0; i < denseMTaskCount; i ++) {
    denseMTaskOrder[(size_t)i] = i;
    const MtDenseMTask& mtask = schedule.mtasks[(size_t)i];
    int predCount = static_cast<int>(mtask.predMTasks.size());
    int succCount = static_cast<int>(mtask.succMTasks.size());
    denseMTaskMaxPredCount = std::max(denseMTaskMaxPredCount, predCount);
    denseMTaskMaxSuccCount = std::max(denseMTaskMaxSuccCount, succCount);
    denseMTaskMaxStaticCost = std::max(denseMTaskMaxStaticCost, mtask.staticCost);
    denseMTaskMaxTaskCount = std::max(denseMTaskMaxTaskCount, mtask.taskCount);
    denseMTaskEdgeCount += succCount;
    int worker = i < static_cast<int>(schedule.mtaskThreadAssign.size()) ? schedule.mtaskThreadAssign[(size_t)i] : -1;
    if (worker >= 0 && worker < denseThreadCount) {
      denseWorkerStaticCosts[(size_t)worker] += mtask.staticCost;
      denseWorkerTaskCounts[(size_t)worker] += mtask.taskCount;
      denseWorkerMTaskCounts[(size_t)worker] ++;
    }
    for (int succ : mtask.succMTasks) {
      int succWorker = succ >= 0 && succ < static_cast<int>(schedule.mtaskThreadAssign.size()) ? schedule.mtaskThreadAssign[(size_t)succ] : -1;
      if (worker >= 0 && succWorker >= 0 && worker == succWorker) denseMTaskSameThreadEdgeCount ++;
      else denseMTaskCrossThreadEdgeCount ++;
    }
  }
  bool denseXThreadDepsOnly = mtUseDenseXThreadDepsOnly();
  bool denseTransitiveReduceEdges = mtUseDenseTransitiveReduceEdges();
  bool denseStaticEmptyElide = mtUseDenseStaticEmptyElide();
  int denseRuntimeSameThreadEdgeElidedCount = 0;
  std::vector<std::vector<int>> denseRuntimeSuccs = mtBuildDenseRuntimeSuccs(schedule.mtasks, schedule.mtaskThreadAssign, denseXThreadDepsOnly, &denseRuntimeSameThreadEdgeElidedCount);
  int denseRuntimeDependencyEdgeCountBeforeTransitiveReduce = mtDenseRuntimeEdgeCount(denseRuntimeSuccs);
  int denseRuntimeTransitiveEdgeElidedCount = denseTransitiveReduceEdges ? mtReduceDenseRuntimeSuccsTransitive(denseRuntimeSuccs, schedule.mtaskThreadAssign) : 0;
  int denseRuntimeDependencyEdgeCount = mtDenseRuntimeEdgeCount(denseRuntimeSuccs);
  int denseRuntimeZeroDepMTaskCount = 0;
  int denseRuntimeZeroSuccMTaskCount = 0;
  std::vector<uint32_t> denseRuntimeDepCountsForReport((size_t)denseMTaskCount, 0);
  for (int mtaskId = 0; mtaskId < denseMTaskCount; mtaskId ++) {
    if (denseRuntimeSuccs[(size_t)mtaskId].empty()) denseRuntimeZeroSuccMTaskCount ++;
    for (int succ : denseRuntimeSuccs[(size_t)mtaskId]) {
      if (succ >= 0 && succ < denseMTaskCount) denseRuntimeDepCountsForReport[(size_t)succ] ++;
    }
  }
  for (uint32_t depCount : denseRuntimeDepCountsForReport) if (depCount == 0) denseRuntimeZeroDepMTaskCount ++;
  auto denseTopBy = [&](auto metric) {
    std::vector<int> top = denseMTaskOrder;
    std::sort(top.begin(), top.end(), [&](int lhs, int rhs) {
      int lhsMetric = metric(schedule.mtasks[(size_t)lhs]);
      int rhsMetric = metric(schedule.mtasks[(size_t)rhs]);
      if (lhsMetric != rhsMetric) return lhsMetric > rhsMetric;
      return lhs < rhs;
    });
    if (top.size() > 20) top.resize(20);
    return top;
  };
  std::vector<int> denseTopPredMTasks = denseTopBy([](const MtDenseMTask& mtask) { return static_cast<int>(mtask.predMTasks.size()); });
  std::vector<int> denseTopSuccMTasks = denseTopBy([](const MtDenseMTask& mtask) { return static_cast<int>(mtask.succMTasks.size()); });
  std::vector<int> denseTopStaticCostMTasks = denseTopBy([](const MtDenseMTask& mtask) { return mtask.staticCost; });
  struct DenseAssignmentStats {
    std::vector<int> workerStaticCosts;
    std::vector<int> workerTaskCounts;
    std::vector<int> workerMTaskCounts;
    int crossThreadEdgeCount = 0;
    int sameThreadEdgeCount = 0;
    int maxWorkerStaticCost = 0;
    int minWorkerStaticCost = 0;
    int predictedMakespan = 0;
  };
  auto computeDenseAssignmentStats = [&](const std::vector<int>& assignment) {
    DenseAssignmentStats stats;
    stats.workerStaticCosts.assign((size_t)denseThreadCount, 0);
    stats.workerTaskCounts.assign((size_t)denseThreadCount, 0);
    stats.workerMTaskCounts.assign((size_t)denseThreadCount, 0);
    for (int i = 0; i < denseMTaskCount; i ++) {
      int worker = i < static_cast<int>(assignment.size()) ? assignment[(size_t)i] : -1;
      if (worker >= 0 && worker < denseThreadCount) {
        const MtDenseMTask& mtask = schedule.mtasks[(size_t)i];
        stats.workerStaticCosts[(size_t)worker] += mtask.staticCost;
        stats.workerTaskCounts[(size_t)worker] += mtask.taskCount;
        stats.workerMTaskCounts[(size_t)worker] ++;
      }
    }
    for (int i = 0; i < denseMTaskCount; i ++) {
      int worker = i < static_cast<int>(assignment.size()) ? assignment[(size_t)i] : -1;
      for (int succ : schedule.mtasks[(size_t)i].succMTasks) {
        int succWorker = succ >= 0 && succ < static_cast<int>(assignment.size()) ? assignment[(size_t)succ] : -1;
        if (worker >= 0 && succWorker >= 0 && worker == succWorker) stats.sameThreadEdgeCount ++;
        else stats.crossThreadEdgeCount ++;
      }
    }
    if (!stats.workerStaticCosts.empty()) {
      stats.maxWorkerStaticCost = *std::max_element(stats.workerStaticCosts.begin(), stats.workerStaticCosts.end());
      stats.minWorkerStaticCost = *std::min_element(stats.workerStaticCosts.begin(), stats.workerStaticCosts.end());
    }
    return stats;
  };
  auto computeDenseAssignmentMakespan = [&](const std::vector<int>& assignment) {
    std::vector<int> completion((size_t)denseMTaskCount, 0);
    std::vector<int> busyUntil((size_t)denseThreadCount, 0);
    for (int mtaskId : denseMTaskOrder) {
      if (mtaskId < 0 || mtaskId >= denseMTaskCount) continue;
      int worker = assignment[(size_t)mtaskId];
      if (worker < 0 || worker >= denseThreadCount) return -1;
      const MtDenseMTask& mtask = schedule.mtasks[(size_t)mtaskId];
      int timeBegin = busyUntil[(size_t)worker];
      for (int pred : mtask.predMTasks) {
        if (pred < 0 || pred >= denseMTaskCount) continue;
        int predWorker = assignment[(size_t)pred];
        int predEnd = completion[(size_t)pred];
        if (predWorker >= 0 && predWorker != worker) predEnd += (schedule.mtasks[(size_t)pred].staticCost * 30) / 100;
        if (predEnd > timeBegin) timeBegin = predEnd;
      }
      int endTime = timeBegin + std::max(1, mtask.staticCost);
      completion[(size_t)mtaskId] = endTime;
      busyUntil[(size_t)worker] = endTime;
    }
    int makespan = 0;
    for (int endTime : completion) makespan = std::max(makespan, endTime);
    return makespan;
  };
  auto buildDenseLptAssignment = [&]() {
    std::vector<int> assignment((size_t)denseMTaskCount, -1);
    std::vector<int> loads((size_t)denseThreadCount, 0);
    std::vector<int> order = denseMTaskOrder;
    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
      const MtDenseMTask& lm = schedule.mtasks[(size_t)lhs];
      const MtDenseMTask& rm = schedule.mtasks[(size_t)rhs];
      if (lm.workerZeroOnly != rm.workerZeroOnly) return lm.workerZeroOnly > rm.workerZeroOnly;
      if (lm.staticCost != rm.staticCost) return lm.staticCost > rm.staticCost;
      return lhs < rhs;
    });
    for (int mtaskId : order) {
      const MtDenseMTask& mtask = schedule.mtasks[(size_t)mtaskId];
      int bestWorker = 0;
      if (!mtask.workerZeroOnly && denseThreadCount > 1) {
        bestWorker = 0;
        for (int worker = 1; worker < denseThreadCount; worker ++) {
          if (loads[(size_t)worker] < loads[(size_t)bestWorker]) bestWorker = worker;
        }
      }
      assignment[(size_t)mtaskId] = bestWorker;
      if (bestWorker >= 0 && bestWorker < denseThreadCount) loads[(size_t)bestWorker] += mtask.staticCost;
    }
    return assignment;
  };
  auto buildDensePredAffinityAssignment = [&]() {
    std::vector<int> assignment((size_t)denseMTaskCount, -1);
    std::vector<int> loads((size_t)denseThreadCount, 0);
    for (int mtaskId = 0; mtaskId < denseMTaskCount; mtaskId ++) {
      const MtDenseMTask& mtask = schedule.mtasks[(size_t)mtaskId];
      int bestWorker = 0;
      if (!mtask.workerZeroOnly && denseThreadCount > 1) {
        int bestPredAffinity = -1;
        for (int worker = 0; worker < denseThreadCount; worker ++) {
          int predAffinity = 0;
          for (int pred : mtask.predMTasks) {
            if (pred >= 0 && pred < static_cast<int>(assignment.size()) && assignment[(size_t)pred] == worker) predAffinity ++;
          }
          if (predAffinity > bestPredAffinity ||
              (predAffinity == bestPredAffinity && loads[(size_t)worker] < loads[(size_t)bestWorker])) {
            bestPredAffinity = predAffinity;
            bestWorker = worker;
          }
        }
      }
      assignment[(size_t)mtaskId] = bestWorker;
      if (bestWorker >= 0 && bestWorker < denseThreadCount) loads[(size_t)bestWorker] += mtask.staticCost;
    }
    return assignment;
  };
  DenseAssignmentStats denseCurrentAssignmentStats = computeDenseAssignmentStats(schedule.mtaskThreadAssign);
  denseCurrentAssignmentStats.predictedMakespan = computeDenseAssignmentMakespan(schedule.mtaskThreadAssign);
  std::vector<int> denseLptAssignment = buildDenseLptAssignment();
  DenseAssignmentStats denseLptAssignmentStats = computeDenseAssignmentStats(denseLptAssignment);
  denseLptAssignmentStats.predictedMakespan = computeDenseAssignmentMakespan(denseLptAssignment);
  std::vector<int> densePredAffinityAssignment = buildDensePredAffinityAssignment();
  DenseAssignmentStats densePredAffinityAssignmentStats = computeDenseAssignmentStats(densePredAffinityAssignment);
  densePredAffinityAssignmentStats.predictedMakespan = computeDenseAssignmentMakespan(densePredAffinityAssignment);
  auto densePackThreadsProjection = mtBuildDensePackThreadsAssignment(schedule.mtasks, denseThreadCount);
  DenseAssignmentStats densePackThreadsAssignmentStats = computeDenseAssignmentStats(densePackThreadsProjection.first);
  densePackThreadsAssignmentStats.predictedMakespan = densePackThreadsProjection.second;
  struct DenseWorker0MixStats {
    int worker0OnlyMTaskCount = 0;
    int contaminatedMTaskCount = 0;
    int worker0OnlyTaskCount = 0;
    int contaminatedTaskCount = 0;
    int contaminatedStaticCost = 0;
  };
  auto computeDenseWorker0MixStats = [&](const std::vector<MtDenseMTask>& mtasks) {
    DenseWorker0MixStats stats;
    for (const MtDenseMTask& mtask : mtasks) {
      int worker0OnlyTasks = 0;
      for (int sccId : mtask.sccIds) {
        if (sccId < 0 || sccId >= static_cast<int>(schedule.sccs.size())) continue;
        worker0OnlyTasks += schedule.sccs[(size_t)sccId].worker0OnlyTaskCount;
      }
      if (mtask.workerZeroOnly) stats.worker0OnlyMTaskCount ++;
      stats.worker0OnlyTaskCount += worker0OnlyTasks;
      if (worker0OnlyTasks > 0 && worker0OnlyTasks < mtask.taskCount) {
        stats.contaminatedMTaskCount ++;
        stats.contaminatedTaskCount += mtask.taskCount;
        stats.contaminatedStaticCost += mtask.staticCost;
      }
    }
    return stats;
  };
  auto computeProjectedDenseAssignmentStats = [&](const std::vector<MtDenseMTask>& mtasks, const std::vector<int>& assignment) {
    DenseAssignmentStats stats;
    stats.workerStaticCosts.assign((size_t)denseThreadCount, 0);
    stats.workerTaskCounts.assign((size_t)denseThreadCount, 0);
    stats.workerMTaskCounts.assign((size_t)denseThreadCount, 0);
    for (size_t i = 0; i < mtasks.size(); i ++) {
      int worker = i < assignment.size() ? assignment[i] : -1;
      if (worker >= 0 && worker < denseThreadCount) {
        const MtDenseMTask& mtask = mtasks[i];
        stats.workerStaticCosts[(size_t)worker] += mtask.staticCost;
        stats.workerTaskCounts[(size_t)worker] += mtask.taskCount;
        stats.workerMTaskCounts[(size_t)worker] ++;
      }
    }
    for (size_t i = 0; i < mtasks.size(); i ++) {
      int worker = i < assignment.size() ? assignment[i] : -1;
      for (int succ : mtasks[i].succMTasks) {
        int succWorker = succ >= 0 && succ < static_cast<int>(assignment.size()) ? assignment[(size_t)succ] : -1;
        if (worker >= 0 && succWorker >= 0 && worker == succWorker) stats.sameThreadEdgeCount ++;
        else stats.crossThreadEdgeCount ++;
      }
    }
    if (!stats.workerStaticCosts.empty()) {
      stats.maxWorkerStaticCost = *std::max_element(stats.workerStaticCosts.begin(), stats.workerStaticCosts.end());
      stats.minWorkerStaticCost = *std::min_element(stats.workerStaticCosts.begin(), stats.workerStaticCosts.end());
    }
    return stats;
  };
  auto computeProjectedDenseAssignmentMakespan = [&](const std::vector<MtDenseMTask>& mtasks, const std::vector<int>& assignment) {
    std::vector<int> completion(mtasks.size(), 0);
    std::vector<int> busyUntil((size_t)denseThreadCount, 0);
    for (size_t mtaskId = 0; mtaskId < mtasks.size(); mtaskId ++) {
      int worker = mtaskId < assignment.size() ? assignment[mtaskId] : -1;
      if (worker < 0 || worker >= denseThreadCount) return -1;
      const MtDenseMTask& mtask = mtasks[mtaskId];
      int timeBegin = busyUntil[(size_t)worker];
      for (int pred : mtask.predMTasks) {
        if (pred < 0 || pred >= static_cast<int>(mtasks.size())) continue;
        int predWorker = assignment[(size_t)pred];
        int predEnd = completion[(size_t)pred];
        if (predWorker >= 0 && predWorker != worker) predEnd += (mtasks[(size_t)pred].staticCost * 30) / 100;
        if (predEnd > timeBegin) timeBegin = predEnd;
      }
      int endTime = timeBegin + std::max(1, mtask.staticCost);
      completion[mtaskId] = endTime;
      busyUntil[(size_t)worker] = endTime;
    }
    int makespan = 0;
    for (int endTime : completion) makespan = std::max(makespan, endTime);
    return makespan;
  };
  auto buildProjectedDenseCurrentAssignment = [&](const std::vector<MtDenseMTask>& mtasks) {
    std::vector<int> assignment(mtasks.size(), 0);
    for (size_t i = 0; i < mtasks.size(); i ++) {
      assignment[i] = mtasks[i].workerZeroOnly ? 0 : (static_cast<int>(i) % denseThreadCount);
    }
    return assignment;
  };
  DenseWorker0MixStats denseWorker0MixStats = computeDenseWorker0MixStats(schedule.mtasks);
  std::vector<MtDenseMTask> denseWorker0SplitMTasks = mtBuildDenseMTasks(schedule, true);
  int denseWorker0SplitEdgeCount = 0;
  for (const MtDenseMTask& mtask : denseWorker0SplitMTasks) denseWorker0SplitEdgeCount += static_cast<int>(mtask.succMTasks.size());
  DenseWorker0MixStats denseWorker0SplitMixStats = computeDenseWorker0MixStats(denseWorker0SplitMTasks);
  std::vector<int> denseWorker0SplitCurrentAssignment = buildProjectedDenseCurrentAssignment(denseWorker0SplitMTasks);
  DenseAssignmentStats denseWorker0SplitCurrentStats = computeProjectedDenseAssignmentStats(denseWorker0SplitMTasks, denseWorker0SplitCurrentAssignment);
  denseWorker0SplitCurrentStats.predictedMakespan = computeProjectedDenseAssignmentMakespan(denseWorker0SplitMTasks, denseWorker0SplitCurrentAssignment);
  auto denseWorker0SplitPackThreadsProjection = mtBuildDensePackThreadsAssignment(denseWorker0SplitMTasks, denseThreadCount);
  DenseAssignmentStats denseWorker0SplitPackThreadsStats = computeProjectedDenseAssignmentStats(denseWorker0SplitMTasks, denseWorker0SplitPackThreadsProjection.first);
  denseWorker0SplitPackThreadsStats.predictedMakespan = denseWorker0SplitPackThreadsProjection.second;

  struct DenseHybridReasonStat {
    int mtaskCount = 0;
    int taskCount = 0;
    long long staticCost = 0;
    long long schedCost = 0;
  };
  struct DenseHybridMTaskDiag {
    bool gateable = false;
    bool mustAlwaysRun = false;
    bool conservativeIneligible = false;
    std::set<std::string> reasons;
    std::set<std::string> mustAlwaysRunReasons;
    std::set<std::string> unprovenReasons;
    int cppIdCount = 0;
    int worker0OnlyTaskCount = 0;
    int alwaysActiveTaskCount = 0;
    int stateUpdateTaskCount = 0;
    int stateSourceCommitTaskCount = 0;
    int stateNextUpdateTaskCount = 0;
    int stateResetUpdateTaskCount = 0;
    int resetTaskCount = 0;
    int asyncResetTaskCount = 0;
    int activateAllTaskCount = 0;
    int rhsNextStateObjectReadTaskCount = 0;
    int unexpandedRhsDependencyTaskCount = 0;
    int ambiguousStateTargetTaskCount = 0;
    int memoryWriteTaskCount = 0;
    int memoryReadTaskCount = 0;
    int externalTaskCount = 0;
    int specialTaskCount = 0;
    int unknownTaskCount = 0;
    int arrayOrDynamicIndexTaskCount = 0;
    int dependencyEdgeInCount = 0;
    int dependencyEdgeOutCount = 0;
    int activeEdgeInCount = 0;
    int activeEdgeOutCount = 0;
    int needActivateEdgeInCount = 0;
    int needActivateEdgeOutCount = 0;
    int runtimeDepCount = 0;
    int runtimeSuccCount = 0;
  };
  bool denseHybridEligibilityDiag = mtUseDenseHybridEligibilityDiag();
  std::vector<DenseHybridMTaskDiag> denseHybridMTaskDiags((size_t)denseMTaskCount);
  std::map<std::string, DenseHybridReasonStat> denseHybridReasonStats;
  std::map<std::string, DenseHybridReasonStat> denseHybridMustAlwaysRunReasonStats;
  std::map<std::string, DenseHybridReasonStat> denseHybridUnprovenReasonStats;
  DenseHybridReasonStat denseHybridGateableStats;
  DenseHybridReasonStat denseHybridMustAlwaysRunStats;
  DenseHybridReasonStat denseHybridConservativeIneligibleStats;
  int denseHybridMultiReasonMTaskCount = 0;
  int denseHybridGateableRuntimeZeroDepMTaskCount = 0;
  int denseHybridGateableRuntimeZeroSuccMTaskCount = 0;
  int denseHybridGateableRuntimeDependencyEdgeCount = 0;
  int denseHybridGateableRuntimeSuccEdgeCount = 0;
  int denseHybridGateableActiveTouchedMTaskCount = 0;
  int denseHybridGateableNeedActivateTouchedMTaskCount = 0;
  int denseHybridMustAlwaysRunRuntimeDependencyEdgeCount = 0;
  int denseHybridMustAlwaysRunRuntimeSuccEdgeCount = 0;
  int denseHybridConservativeIneligibleRuntimeDependencyEdgeCount = 0;
  int denseHybridConservativeIneligibleRuntimeSuccEdgeCount = 0;
  int denseHybridStaticDependencyInterMTaskEdgeCount = 0;
  int denseHybridStaticActiveInterMTaskEdgeCount = 0;
  int denseHybridStaticNeedActivateInterMTaskEdgeCount = 0;
  std::vector<int> denseHybridTopMustAlwaysRunMTasks;
  std::vector<int> denseHybridTopConservativeIneligibleMTasks;
  std::vector<int> denseHybridTopGateableMTasks;
  std::vector<int> cppIdToDenseMTask((size_t)superId, -1);
  for (int mtaskId = 0; mtaskId < denseMTaskCount; mtaskId ++) {
    const MtDenseMTask& mtask = schedule.mtasks[(size_t)mtaskId];
    for (int sccId : mtask.sccIds) {
      if (sccId < 0 || sccId >= static_cast<int>(schedule.sccs.size())) continue;
      const MtDenseScc& scc = schedule.sccs[(size_t)sccId];
      for (int cppId : scc.cppIds) {
        if (cppId >= 0 && cppId < superId) cppIdToDenseMTask[(size_t)cppId] = mtaskId;
      }
    }
  }
  if (denseHybridEligibilityDiag) {
    auto denseHybridReasonIsMustAlwaysRun = [](const std::string& reason) {
      return reason == "always_active" || reason == "state_update" ||
             reason == "state_source_commit" || reason == "state_next_update" ||
             reason == "state_reset_update" || reason == "reset" ||
             reason == "async_reset" || reason == "memory_write" ||
             reason == "memory_read_unsupported" || reason == "external" ||
             reason == "special" || reason == "super_type_SUPER_EXTMOD";
    };
    auto addDenseHybridReason = [&](DenseHybridMTaskDiag& diag, const std::string& reason) {
      diag.reasons.insert(reason);
      if (denseHybridReasonIsMustAlwaysRun(reason)) diag.mustAlwaysRunReasons.insert(reason);
      else diag.unprovenReasons.insert(reason);
    };
    for (int mtaskId = 0; mtaskId < denseMTaskCount; mtaskId ++) {
      DenseHybridMTaskDiag& diag = denseHybridMTaskDiags[(size_t)mtaskId];
      const MtDenseMTask& mtask = schedule.mtasks[(size_t)mtaskId];
      for (int sccId : mtask.sccIds) {
        if (sccId < 0 || sccId >= static_cast<int>(schedule.sccs.size())) continue;
        const MtDenseScc& scc = schedule.sccs[(size_t)sccId];
        for (int cppId : scc.cppIds) {
          diag.cppIdCount ++;
          auto taskIter = mtTasks.find(cppId);
          if (taskIter == mtTasks.end()) {
            addDenseHybridReason(diag, "missing_task_info");
            continue;
          }
          const MtTaskInfo& task = taskIter->second;
          const MtBoundaryInfo& boundary = task.boundary;
          for (const std::string& reason : task.serialReasons) addDenseHybridReason(diag, reason);
          if (hasWorker0OnlyReason(task.serialReasons)) {
            diag.worker0OnlyTaskCount ++;
            addDenseHybridReason(diag, "worker0_only");
          }
          if (isAlwaysActive(cppId)) {
            diag.alwaysActiveTaskCount ++;
            addDenseHybridReason(diag, "always_active");
          }
          if (boundary.hasStateUpdate) { diag.stateUpdateTaskCount ++; addDenseHybridReason(diag, "state_update"); }
          if (boundary.stateSourceCommitCount > 0) { diag.stateSourceCommitTaskCount ++; addDenseHybridReason(diag, "state_source_commit"); }
          if (boundary.stateNextUpdateCount > 0) { diag.stateNextUpdateTaskCount ++; addDenseHybridReason(diag, "state_next_update"); }
          if (boundary.stateResetUpdateCount > 0) { diag.stateResetUpdateTaskCount ++; addDenseHybridReason(diag, "state_reset_update"); }
          if (boundary.hasReset) { diag.resetTaskCount ++; addDenseHybridReason(diag, "reset"); }
          if (boundary.hasAsyncReset) { diag.asyncResetTaskCount ++; addDenseHybridReason(diag, "async_reset"); }
          if (boundary.hasActivateAllPath) { diag.activateAllTaskCount ++; addDenseHybridReason(diag, "activate_all_path"); }
          if (boundary.hasRhsNextStateObjectRead) { diag.rhsNextStateObjectReadTaskCount ++; addDenseHybridReason(diag, "rhs_next_state_object_read"); }
          if (boundary.hasUnexpandedRhsDependency) { diag.unexpandedRhsDependencyTaskCount ++; addDenseHybridReason(diag, "unexpanded_rhs_dependency"); }
          if (boundary.hasAmbiguousStateTarget) { diag.ambiguousStateTargetTaskCount ++; addDenseHybridReason(diag, "ambiguous_state_target"); }
          if (boundary.hasMemoryWrite) { diag.memoryWriteTaskCount ++; addDenseHybridReason(diag, "memory_write"); }
          if (boundary.hasMemoryRead) { diag.memoryReadTaskCount ++; addDenseHybridReason(diag, "memory_read_unsupported"); }
          if (boundary.hasExternal) { diag.externalTaskCount ++; addDenseHybridReason(diag, "external"); }
          if (boundary.hasSpecial) { diag.specialTaskCount ++; addDenseHybridReason(diag, "special"); }
          if (boundary.hasUnknownNode) { diag.unknownTaskCount ++; addDenseHybridReason(diag, "unknown_node"); }
          if (boundary.hasUnknownOp) { diag.unknownTaskCount ++; addDenseHybridReason(diag, "unknown_op"); }
          if (boundary.hasArrayOrDynamicIndex) { diag.arrayOrDynamicIndexTaskCount ++; addDenseHybridReason(diag, "array_or_dynamic_index"); }
        }
      }
      if (diag.cppIdCount == 0) addDenseHybridReason(diag, "empty_mtask");
    }
    for (const MtDenseEdge& edge : schedule.edges) {
      int fromMTask = edge.fromCppId >= 0 && edge.fromCppId < superId ? cppIdToDenseMTask[(size_t)edge.fromCppId] : -1;
      int toMTask = edge.toCppId >= 0 && edge.toCppId < superId ? cppIdToDenseMTask[(size_t)edge.toCppId] : -1;
      if (fromMTask < 0 || toMTask < 0 || fromMTask == toMTask) continue;
      DenseHybridMTaskDiag& fromDiag = denseHybridMTaskDiags[(size_t)fromMTask];
      DenseHybridMTaskDiag& toDiag = denseHybridMTaskDiags[(size_t)toMTask];
      if (edge.kind == "dependency") {
        fromDiag.dependencyEdgeOutCount ++;
        toDiag.dependencyEdgeInCount ++;
        denseHybridStaticDependencyInterMTaskEdgeCount ++;
      } else if (edge.kind == "active") {
        fromDiag.activeEdgeOutCount ++;
        toDiag.activeEdgeInCount ++;
        denseHybridStaticActiveInterMTaskEdgeCount ++;
      } else if (edge.kind == "need_activate") {
        fromDiag.needActivateEdgeOutCount ++;
        toDiag.needActivateEdgeInCount ++;
        denseHybridStaticNeedActivateInterMTaskEdgeCount ++;
      }
    }
    auto addDenseHybridStat = [](DenseHybridReasonStat& stat, const MtDenseMTask& mtask) {
      stat.mtaskCount ++;
      stat.taskCount += mtask.taskCount;
      stat.staticCost += mtask.staticCost;
      stat.schedCost += mtask.schedCost > 0 ? mtask.schedCost : mtask.staticCost;
    };
    for (int mtaskId = 0; mtaskId < denseMTaskCount; mtaskId ++) {
      DenseHybridMTaskDiag& diag = denseHybridMTaskDiags[(size_t)mtaskId];
      const MtDenseMTask& mtask = schedule.mtasks[(size_t)mtaskId];
      diag.runtimeDepCount = mtaskId < static_cast<int>(denseRuntimeDepCountsForReport.size()) ? static_cast<int>(denseRuntimeDepCountsForReport[(size_t)mtaskId]) : 0;
      diag.runtimeSuccCount = mtaskId < static_cast<int>(denseRuntimeSuccs.size()) ? static_cast<int>(denseRuntimeSuccs[(size_t)mtaskId].size()) : 0;
      diag.mustAlwaysRun = !diag.mustAlwaysRunReasons.empty();
      diag.conservativeIneligible = !diag.mustAlwaysRun && !diag.unprovenReasons.empty();
      diag.gateable = !diag.mustAlwaysRun && !diag.conservativeIneligible;
      if (diag.gateable) {
        addDenseHybridStat(denseHybridGateableStats, mtask);
        if (diag.runtimeDepCount == 0) denseHybridGateableRuntimeZeroDepMTaskCount ++;
        if (diag.runtimeSuccCount == 0) denseHybridGateableRuntimeZeroSuccMTaskCount ++;
        denseHybridGateableRuntimeDependencyEdgeCount += diag.runtimeDepCount;
        denseHybridGateableRuntimeSuccEdgeCount += diag.runtimeSuccCount;
        if (diag.activeEdgeInCount + diag.activeEdgeOutCount > 0) denseHybridGateableActiveTouchedMTaskCount ++;
        if (diag.needActivateEdgeInCount + diag.needActivateEdgeOutCount > 0) denseHybridGateableNeedActivateTouchedMTaskCount ++;
        denseHybridTopGateableMTasks.push_back(mtaskId);
      } else if (diag.mustAlwaysRun) {
        addDenseHybridStat(denseHybridMustAlwaysRunStats, mtask);
        denseHybridMustAlwaysRunRuntimeDependencyEdgeCount += diag.runtimeDepCount;
        denseHybridMustAlwaysRunRuntimeSuccEdgeCount += diag.runtimeSuccCount;
        denseHybridTopMustAlwaysRunMTasks.push_back(mtaskId);
      } else {
        addDenseHybridStat(denseHybridConservativeIneligibleStats, mtask);
        denseHybridConservativeIneligibleRuntimeDependencyEdgeCount += diag.runtimeDepCount;
        denseHybridConservativeIneligibleRuntimeSuccEdgeCount += diag.runtimeSuccCount;
        denseHybridTopConservativeIneligibleMTasks.push_back(mtaskId);
      }
      if (diag.reasons.size() > 1) denseHybridMultiReasonMTaskCount ++;
      for (const std::string& reason : diag.reasons) addDenseHybridStat(denseHybridReasonStats[reason], mtask);
      for (const std::string& reason : diag.mustAlwaysRunReasons) addDenseHybridStat(denseHybridMustAlwaysRunReasonStats[reason], mtask);
      for (const std::string& reason : diag.unprovenReasons) addDenseHybridStat(denseHybridUnprovenReasonStats[reason], mtask);
    }
    auto denseHybridTopCmp = [&](int lhs, int rhs) {
      const MtDenseMTask& lm = schedule.mtasks[(size_t)lhs];
      const MtDenseMTask& rm = schedule.mtasks[(size_t)rhs];
      if (lm.staticCost != rm.staticCost) return lm.staticCost > rm.staticCost;
      if (lm.taskCount != rm.taskCount) return lm.taskCount > rm.taskCount;
      return lhs < rhs;
    };
    std::sort(denseHybridTopGateableMTasks.begin(), denseHybridTopGateableMTasks.end(), denseHybridTopCmp);
    std::sort(denseHybridTopMustAlwaysRunMTasks.begin(), denseHybridTopMustAlwaysRunMTasks.end(), denseHybridTopCmp);
    std::sort(denseHybridTopConservativeIneligibleMTasks.begin(), denseHybridTopConservativeIneligibleMTasks.end(), denseHybridTopCmp);
    if (denseHybridTopGateableMTasks.size() > 20) denseHybridTopGateableMTasks.resize(20);
    if (denseHybridTopMustAlwaysRunMTasks.size() > 20) denseHybridTopMustAlwaysRunMTasks.resize(20);
    if (denseHybridTopConservativeIneligibleMTasks.size() > 20) denseHybridTopConservativeIneligibleMTasks.resize(20);
  }

  fprintf(fp, "{\n");
  fprintf(fp, "  \"format\": \"gsim.mt-dense-schedule.v1\",\n");
  fprintf(fp, "  \"codegen_enabled\": %s,\n", schedule.codegenEnabled ? "true" : "false");
  fprintf(fp, "  \"valid\": %s,\n", schedule.valid ? "true" : "false");
  fprintf(fp, "  \"fallback_reason\": \"%s\",\n", jsonEscape(schedule.fallbackReason).c_str());
  fprintf(fp, "  \"dense_xthread_deps_only_enabled\": %s,\n", denseXThreadDepsOnly ? "true" : "false");
  fprintf(fp, "  \"dense_transitive_reduce_edges_enabled\": %s,\n", denseTransitiveReduceEdges ? "true" : "false");
  fprintf(fp, "  \"dense_split_worker0_mtasks_enabled\": %s,\n", mtUseDenseSplitWorker0MTasks() ? "true" : "false");
  fprintf(fp, "  \"dense_runtime_dependency_edge_count\": %d,\n", denseRuntimeDependencyEdgeCount);
  fprintf(fp, "  \"dense_runtime_dependency_edge_count_before_transitive_reduce\": %d,\n", denseRuntimeDependencyEdgeCountBeforeTransitiveReduce);
  fprintf(fp, "  \"dense_runtime_same_thread_edge_elided_count\": %d,\n", denseRuntimeSameThreadEdgeElidedCount);
  fprintf(fp, "  \"dense_runtime_transitive_edge_elided_count\": %d,\n", denseRuntimeTransitiveEdgeElidedCount);
  fprintf(fp, "  \"dense_static_empty_elide_enabled\": %s,\n", denseStaticEmptyElide ? "true" : "false");
  fprintf(fp, "  \"dense_runtime_zero_dep_mtask_count\": %d,\n", denseRuntimeZeroDepMTaskCount);
  fprintf(fp, "  \"dense_runtime_zero_succ_mtask_count\": %d,\n", denseRuntimeZeroSuccMTaskCount);
  fprintf(fp, "  \"task_count\": %d,\n", schedule.taskCount);
  fprintf(fp, "  \"active_width\": %d,\n", ACTIVE_WIDTH);
  fprintf(fp, "  \"edge_count\": %d,\n", schedule.edgeCount);
  fprintf(fp, "  \"dependency_edge_count\": %d,\n", schedule.dependencyEdgeCount);
  fprintf(fp, "  \"active_edge_count\": %d,\n", schedule.activeEdgeCount);
  fprintf(fp, "  \"need_activate_edge_count\": %d,\n", schedule.needActivateEdgeCount);
  fprintf(fp, "  \"scc_count\": %zu,\n", schedule.sccs.size());
  fprintf(fp, "  \"cycle_scc_count\": %d,\n", schedule.cycleSccCount);
  fprintf(fp, "  \"max_scc_size\": %d,\n", schedule.maxSccSize);
  fprintf(fp, "  \"dense_counter_bytes_u8_t8\": %zu,\n", schedule.sccs.size() * (size_t)8);
  fprintf(fp, "  \"layer_count\": %zu,\n", schedule.layers.size());
  fprintf(fp, "  \"max_layer_width\": %d,\n", maxLayerWidth);
  fprintf(fp, "  \"worker0_only_layer_count\": %d,\n", worker0OnlyLayerCount);
  fprintf(fp, "  \"dense_mtask_count\": %d,\n", denseMTaskCount);
  fprintf(fp, "  \"dense_mtask_edge_count\": %d,\n", denseMTaskEdgeCount);
  fprintf(fp, "  \"dense_mtask_cross_thread_edge_count\": %d,\n", denseMTaskCrossThreadEdgeCount);
  fprintf(fp, "  \"dense_mtask_same_thread_edge_count\": %d,\n", denseMTaskSameThreadEdgeCount);
  fprintf(fp, "  \"dense_mtask_max_pred_count\": %d,\n", denseMTaskMaxPredCount);
  fprintf(fp, "  \"dense_mtask_max_succ_count\": %d,\n", denseMTaskMaxSuccCount);
  fprintf(fp, "  \"dense_mtask_max_static_cost\": %d,\n", denseMTaskMaxStaticCost);
  fprintf(fp, "  \"dense_mtask_max_task_count\": %d,\n", denseMTaskMaxTaskCount);
  fprintf(fp, "  \"dense_worker0_only_mtask_count\": %d,\n", denseWorker0MixStats.worker0OnlyMTaskCount);
  fprintf(fp, "  \"dense_worker0_contaminated_mtask_count\": %d,\n", denseWorker0MixStats.contaminatedMTaskCount);
  fprintf(fp, "  \"dense_worker0_contaminated_task_count\": %d,\n", denseWorker0MixStats.contaminatedTaskCount);
  fprintf(fp, "  \"dense_worker0_contaminated_static_cost\": %d,\n", denseWorker0MixStats.contaminatedStaticCost);
  fprintf(fp, "  \"dense_worker0_split_mtask_count\": %zu,\n", denseWorker0SplitMTasks.size());
  fprintf(fp, "  \"dense_worker0_split_edge_count\": %d,\n", denseWorker0SplitEdgeCount);
  fprintf(fp, "  \"dense_worker0_split_worker0_only_mtask_count\": %d,\n", denseWorker0SplitMixStats.worker0OnlyMTaskCount);
  fprintf(fp, "  \"dense_worker0_split_contaminated_mtask_count\": %d,\n", denseWorker0SplitMixStats.contaminatedMTaskCount);
  fprintf(fp, "  \"dense_worker0_split_contaminated_task_count\": %d,\n", denseWorker0SplitMixStats.contaminatedTaskCount);
  fprintf(fp, "  \"dense_worker0_split_contaminated_static_cost\": %d,\n", denseWorker0SplitMixStats.contaminatedStaticCost);
  fprintf(fp, "  \"dense_worker_static_costs\": ");
  dumpJsonIntArray(fp, denseWorkerStaticCosts);
  fprintf(fp, ",\n");
  fprintf(fp, "  \"dense_worker_task_counts\": ");
  dumpJsonIntArray(fp, denseWorkerTaskCounts);
  fprintf(fp, ",\n");
  fprintf(fp, "  \"dense_worker_mtask_counts\": ");
  dumpJsonIntArray(fp, denseWorkerMTaskCounts);
  fprintf(fp, ",\n");
  auto dumpDenseMTaskSummaryArray = [&](const char* key, const std::vector<int>& indices) {
    fprintf(fp, "  \"%s\": [\n", key);
    for (size_t rank = 0; rank < indices.size(); rank ++) {
      int mtaskId = indices[rank];
      const MtDenseMTask& mtask = schedule.mtasks[(size_t)mtaskId];
      int worker = mtaskId < static_cast<int>(schedule.mtaskThreadAssign.size()) ? schedule.mtaskThreadAssign[(size_t)mtaskId] : -1;
      fprintf(fp, "    {\"rank\": %zu, \"mtask_id\": %d, \"thread\": %d, \"pred_count\": %zu, \"succ_count\": %zu, \"static_cost\": %d, \"task_count\": %d}%s\n",
              rank, mtaskId, worker, mtask.predMTasks.size(), mtask.succMTasks.size(), mtask.staticCost, mtask.taskCount,
              rank + 1 == indices.size() ? "" : ",");
    }
    fprintf(fp, "  ],\n");
  };
  dumpDenseMTaskSummaryArray("dense_top_pred_mtasks", denseTopPredMTasks);
  dumpDenseMTaskSummaryArray("dense_top_succ_mtasks", denseTopSuccMTasks);
  dumpDenseMTaskSummaryArray("dense_top_static_cost_mtasks", denseTopStaticCostMTasks);
  auto dumpDenseAssignmentStats = [&](const char* key, const DenseAssignmentStats& stats) {
    fprintf(fp, "  \"%s\": {\n", key);
    fprintf(fp, "    \"cross_thread_edge_count\": %d,\n", stats.crossThreadEdgeCount);
    fprintf(fp, "    \"same_thread_edge_count\": %d,\n", stats.sameThreadEdgeCount);
    fprintf(fp, "    \"max_worker_static_cost\": %d,\n", stats.maxWorkerStaticCost);
    fprintf(fp, "    \"min_worker_static_cost\": %d,\n", stats.minWorkerStaticCost);
    fprintf(fp, "    \"predicted_makespan\": %d,\n", stats.predictedMakespan);
    fprintf(fp, "    \"worker_static_costs\": ");
    dumpJsonIntArray(fp, stats.workerStaticCosts);
    fprintf(fp, ",\n");
    fprintf(fp, "    \"worker_task_counts\": ");
    dumpJsonIntArray(fp, stats.workerTaskCounts);
    fprintf(fp, ",\n");
    fprintf(fp, "    \"worker_mtask_counts\": ");
    dumpJsonIntArray(fp, stats.workerMTaskCounts);
    fprintf(fp, "\n  },\n");
  };
  dumpDenseAssignmentStats("dense_assignment_current", denseCurrentAssignmentStats);
  dumpDenseAssignmentStats("dense_assignment_lpt", denseLptAssignmentStats);
  dumpDenseAssignmentStats("dense_assignment_pred_affinity", densePredAffinityAssignmentStats);
  dumpDenseAssignmentStats("dense_assignment_packthreads", densePackThreadsAssignmentStats);
  dumpDenseAssignmentStats("dense_worker0_split_assignment_current", denseWorker0SplitCurrentStats);
  dumpDenseAssignmentStats("dense_worker0_split_assignment_packthreads", denseWorker0SplitPackThreadsStats);
  fprintf(fp, "  \"worker0_only_cpp_ids\": ");
  dumpJsonIntArray(fp, schedule.worker0OnlyCppIds);
  fprintf(fp, ",\n");
  fprintf(fp, "  \"always_active_cpp_ids\": ");
  dumpJsonIntArray(fp, schedule.alwaysActiveCppIds);
  fprintf(fp, ",\n");

  auto dumpDenseHybridStatObject = [&](const char* key, const DenseHybridReasonStat& stat, bool trailingComma) {
    fprintf(fp, "    \"%s\": {\"mtask_count\": %d, \"task_count\": %d, \"static_cost\": %lld, \"sched_cost\": %lld}%s\n",
            key, stat.mtaskCount, stat.taskCount, stat.staticCost, stat.schedCost, trailingComma ? "," : "");
  };
  auto dumpDenseHybridReasonHistogram = [&](const char* key, const std::map<std::string, DenseHybridReasonStat>& stats, bool trailingComma) {
    fprintf(fp, "    \"%s\": [\n", key);
    size_t rank = 0;
    for (const auto& item : stats) {
      const DenseHybridReasonStat& stat = item.second;
      fprintf(fp, "      {\"reason\": \"%s\", \"mtask_count\": %d, \"task_count\": %d, \"static_cost\": %lld, \"sched_cost\": %lld}%s\n",
              jsonEscape(item.first).c_str(), stat.mtaskCount, stat.taskCount, stat.staticCost, stat.schedCost,
              ++ rank == stats.size() ? "" : ",");
    }
    fprintf(fp, "    ]%s\n", trailingComma ? "," : "");
  };
  auto dumpDenseHybridMTaskDiagArray = [&](const char* key, const std::vector<int>& indices, bool trailingComma) {
    fprintf(fp, "    \"%s\": [\n", key);
    for (size_t rank = 0; rank < indices.size(); rank ++) {
      int mtaskId = indices[rank];
      const MtDenseMTask& mtask = schedule.mtasks[(size_t)mtaskId];
      const DenseHybridMTaskDiag& diag = denseHybridMTaskDiags[(size_t)mtaskId];
      int worker = mtaskId < static_cast<int>(schedule.mtaskThreadAssign.size()) ? schedule.mtaskThreadAssign[(size_t)mtaskId] : -1;
      fprintf(fp, "      {\"rank\": %zu, \"mtask_id\": %d, \"thread\": %d, \"gateable\": %s, \"must_always_run\": %s, \"conservative_ineligible\": %s, \"reasons\": ",
              rank, mtaskId, worker, diag.gateable ? "true" : "false", diag.mustAlwaysRun ? "true" : "false", diag.conservativeIneligible ? "true" : "false");
      dumpJsonStringArray(fp, diag.reasons);
      fprintf(fp, ", \"must_always_run_reasons\": ");
      dumpJsonStringArray(fp, diag.mustAlwaysRunReasons);
      fprintf(fp, ", \"unproven_reasons\": ");
      dumpJsonStringArray(fp, diag.unprovenReasons);
      fprintf(fp, ", \"pred_count\": %zu, \"succ_count\": %zu, \"runtime_dep_count\": %d, \"runtime_succ_count\": %d, \"task_count\": %d, \"static_cost\": %d, \"sched_cost\": %d, \"worker0_only_task_count\": %d, \"always_active_task_count\": %d, \"state_update_task_count\": %d, \"state_source_commit_task_count\": %d, \"state_next_update_task_count\": %d, \"state_reset_update_task_count\": %d, \"reset_task_count\": %d, \"async_reset_task_count\": %d, \"activate_all_task_count\": %d, \"rhs_next_state_object_read_task_count\": %d, \"unexpanded_rhs_dependency_task_count\": %d, \"ambiguous_state_target_task_count\": %d, \"memory_write_task_count\": %d, \"memory_read_task_count\": %d, \"external_task_count\": %d, \"special_task_count\": %d, \"unknown_task_count\": %d, \"array_or_dynamic_index_task_count\": %d, \"dependency_edge_in_count\": %d, \"dependency_edge_out_count\": %d, \"active_edge_in_count\": %d, \"active_edge_out_count\": %d, \"need_activate_edge_in_count\": %d, \"need_activate_edge_out_count\": %d}%s\n",
              mtask.predMTasks.size(), mtask.succMTasks.size(), diag.runtimeDepCount, diag.runtimeSuccCount,
              mtask.taskCount, mtask.staticCost, mtask.schedCost > 0 ? mtask.schedCost : mtask.staticCost,
              diag.worker0OnlyTaskCount, diag.alwaysActiveTaskCount, diag.stateUpdateTaskCount,
              diag.stateSourceCommitTaskCount, diag.stateNextUpdateTaskCount, diag.stateResetUpdateTaskCount,
              diag.resetTaskCount, diag.asyncResetTaskCount, diag.activateAllTaskCount,
              diag.rhsNextStateObjectReadTaskCount, diag.unexpandedRhsDependencyTaskCount,
              diag.ambiguousStateTargetTaskCount, diag.memoryWriteTaskCount, diag.memoryReadTaskCount,
              diag.externalTaskCount, diag.specialTaskCount, diag.unknownTaskCount, diag.arrayOrDynamicIndexTaskCount,
              diag.dependencyEdgeInCount, diag.dependencyEdgeOutCount, diag.activeEdgeInCount, diag.activeEdgeOutCount,
              diag.needActivateEdgeInCount, diag.needActivateEdgeOutCount,
              rank + 1 == indices.size() ? "" : ",");
    }
    fprintf(fp, "    ]%s\n", trailingComma ? "," : "");
  };
  fprintf(fp, "  \"dense_hybrid_eligibility\": {\n");
  fprintf(fp, "    \"enabled\": %s,\n", denseHybridEligibilityDiag ? "true" : "false");
  fprintf(fp, "    \"basis\": \"static_mtask_membership_upper_bound\",\n");
  fprintf(fp, "    \"per_cycle_skip_rate_source\": \"none_static_only_requires_sparse_dynamic_trace\",\n");
  fprintf(fp, "    \"runtime_dependency_is_eligibility_blocker\": false,\n");
  fprintf(fp, "    \"dynamic_trace_required_for_skip_rate\": true,\n");
  fprintf(fp, "    \"must_always_run_policy\": \"Always-run MTasks include always-active work, worker0-only external calls, memory/state-update/commit work, EXTMOD/DPIC side effects, or reset-sensitive work; see docs/draft.md dense/sparse hybrid workflow notes.\",\n");
  fprintf(fp, "    \"cpp_id_to_dense_mtask\": ");
  dumpJsonIntArray(fp, cppIdToDenseMTask);
  fprintf(fp, ",\n");
  dumpDenseHybridStatObject("gateable_upper_bound", denseHybridGateableStats, true);
  dumpDenseHybridStatObject("must_always_run", denseHybridMustAlwaysRunStats, true);
  dumpDenseHybridStatObject("conservative_ineligible", denseHybridConservativeIneligibleStats, true);
  fprintf(fp, "    \"multi_reason_mtask_count\": %d,\n", denseHybridMultiReasonMTaskCount);
  fprintf(fp, "    \"static_dependency_inter_mtask_edge_count\": %d,\n", denseHybridStaticDependencyInterMTaskEdgeCount);
  fprintf(fp, "    \"static_active_inter_mtask_edge_count\": %d,\n", denseHybridStaticActiveInterMTaskEdgeCount);
  fprintf(fp, "    \"static_need_activate_inter_mtask_edge_count\": %d,\n", denseHybridStaticNeedActivateInterMTaskEdgeCount);
  fprintf(fp, "    \"gateable_runtime_zero_dep_mtask_count\": %d,\n", denseHybridGateableRuntimeZeroDepMTaskCount);
  fprintf(fp, "    \"gateable_runtime_zero_succ_mtask_count\": %d,\n", denseHybridGateableRuntimeZeroSuccMTaskCount);
  fprintf(fp, "    \"gateable_runtime_dependency_edge_count\": %d,\n", denseHybridGateableRuntimeDependencyEdgeCount);
  fprintf(fp, "    \"gateable_runtime_succ_edge_count\": %d,\n", denseHybridGateableRuntimeSuccEdgeCount);
  fprintf(fp, "    \"gateable_active_touched_mtask_count\": %d,\n", denseHybridGateableActiveTouchedMTaskCount);
  fprintf(fp, "    \"gateable_need_activate_touched_mtask_count\": %d,\n", denseHybridGateableNeedActivateTouchedMTaskCount);
  fprintf(fp, "    \"must_always_run_runtime_dependency_edge_count\": %d,\n", denseHybridMustAlwaysRunRuntimeDependencyEdgeCount);
  fprintf(fp, "    \"must_always_run_runtime_succ_edge_count\": %d,\n", denseHybridMustAlwaysRunRuntimeSuccEdgeCount);
  fprintf(fp, "    \"conservative_ineligible_runtime_dependency_edge_count\": %d,\n", denseHybridConservativeIneligibleRuntimeDependencyEdgeCount);
  fprintf(fp, "    \"conservative_ineligible_runtime_succ_edge_count\": %d,\n", denseHybridConservativeIneligibleRuntimeSuccEdgeCount);
  dumpDenseHybridReasonHistogram("reason_histogram", denseHybridReasonStats, true);
  dumpDenseHybridReasonHistogram("must_always_run_reason_histogram", denseHybridMustAlwaysRunReasonStats, true);
  dumpDenseHybridReasonHistogram("unproven_reason_histogram", denseHybridUnprovenReasonStats, true);
  dumpDenseHybridMTaskDiagArray("top_static_cost_gateable_mtasks", denseHybridTopGateableMTasks, true);
  dumpDenseHybridMTaskDiagArray("top_static_cost_must_always_run_mtasks", denseHybridTopMustAlwaysRunMTasks, true);
  dumpDenseHybridMTaskDiagArray("top_static_cost_conservative_ineligible_mtasks", denseHybridTopConservativeIneligibleMTasks, false);
  fprintf(fp, "  },\n");

  fprintf(fp, "  \"tasks\": [\n");
  for (int cppId = 0; cppId < superId; cppId ++) {
    SuperNode* super = cppId2Super[cppId];
    MtTaskInfo& mtTask = mtTasks[cppId];
    int activeWord;
    uint64_t activeMask;
    std::tie(activeWord, activeMask) = setIdxMask(cppId);
    int denseMTaskId = cppId < static_cast<int>(cppIdToDenseMTask.size()) ? cppIdToDenseMTask[(size_t)cppId] : -1;
    fprintf(fp, "    {\"cpp_id\": %d, \"scan_index\": %d, \"super_id\": %d, \"super_type\": \"%s\", ", cppId, cppId, super->cppId, superTypeName(super->superType));  // cppId for super->id
    fprintf(fp, "\"task_kind\": \"%s\", \"dense_mtask_id\": %d, \"serial_reasons\": ", mtTask.taskKind.c_str(), denseMTaskId);
    dumpJsonStringArray(fp, mtTask.serialReasons);
    fprintf(fp, ", \"worker0_only\": %s, \"is_always_active\": %s, ",
            hasWorker0OnlyReasonDense(mtTask.serialReasons) ? "true" : "false",
            isAlwaysActive(cppId) ? "true" : "false");
    fprintf(fp, "\"active_word\": %d, \"active_mask\": \"0x%" PRIx64 "\", ", activeWord, activeMask);
    fprintf(fp, "\"static_cost\": %d, \"member_node_cost\": %zu, ", mtTaskEstimatedCost(mtTasks, cppId), super->member.size());
    fprintf(fp, "\"pred_cpp_ids\": ");
    dumpJsonIntArray(fp, schedule.predCppIds[(size_t)cppId]);
    fprintf(fp, ", \"succ_cpp_ids\": ");
    dumpJsonIntArray(fp, schedule.succCppIds[(size_t)cppId]);
    fprintf(fp, ", \"boundary\": {\"has_state_update\": %s, \"has_memory_write\": %s, \"has_reset\": %s, \"has_external\": %s, \"has_special\": %s}",
            mtTask.boundary.hasStateUpdate ? "true" : "false",
            mtTask.boundary.hasMemoryWrite ? "true" : "false",
            mtTask.boundary.hasReset ? "true" : "false",
            mtTask.boundary.hasExternal ? "true" : "false",
            mtTask.boundary.hasSpecial ? "true" : "false");
    if (mtUseDenseMemberMetadata()) {
      dumpMtDenseMemberMetadataForTask(fp, super);
    }
    fprintf(fp, "}");
    fprintf(fp, "%s\n", cppId + 1 == superId ? "" : ",");
  }
  fprintf(fp, "  ],\n");

  fprintf(fp, "  \"edges\": [\n");
  // edge enumeration follows allocator-sensitive pointer iteration order. Emit in
  // a deterministic canonical order so identical schedules produce identical files.
  std::vector<MtDenseEdge> canonEdges(schedule.edges.begin(), schedule.edges.end());
  std::sort(canonEdges.begin(), canonEdges.end(), [](const MtDenseEdge& a, const MtDenseEdge& b) {
    if (a.fromCppId != b.fromCppId) return a.fromCppId < b.fromCppId;
    if (a.toCppId != b.toCppId) return a.toCppId < b.toCppId;
    return a.kind < b.kind;
  });
  for (size_t i = 0; i < canonEdges.size(); i ++) {
    const MtDenseEdge& edge = canonEdges[i];
    fprintf(fp, "    {\"from_cpp_id\": %d, \"to_cpp_id\": %d, \"kind\": \"%s\"}%s\n",
            edge.fromCppId, edge.toCppId, edge.kind.c_str(), i + 1 == canonEdges.size() ? "" : ",");
  }
  fprintf(fp, "  ],\n");
  dumpMtDenseActivationOrigins(fp);

  fprintf(fp, "  \"sccs\": [\n");
  for (size_t sccId = 0; sccId < schedule.sccs.size(); sccId ++) {
    const MtDenseScc& scc = schedule.sccs[sccId];
    fprintf(fp, "    {\"scc_id\": %zu, \"cpp_ids\": ", sccId);
    dumpJsonIntArray(fp, scc.cppIds);
    fprintf(fp, ", \"task_count\": %zu, \"static_cost\": %d, \"member_node_cost\": %d, \"worker0_only\": %s, \"worker0_only_task_count\": %d, \"is_always_active\": %s, \"always_active_task_count\": %d, \"is_trivial\": %s, \"internal_edge_count\": %d, \"internal_dependency_edge_count\": %d, \"internal_active_edge_count\": %d, \"internal_need_activate_edge_count\": %d, \"incoming_edge_count\": %d, \"outgoing_edge_count\": %d, \"pred_sccs\": ",
            scc.cppIds.size(), scc.staticCost, scc.memberNodeCost,
            scc.workerZeroOnly ? "true" : "false", scc.worker0OnlyTaskCount,
            scc.isAlwaysActive ? "true" : "false", scc.alwaysActiveTaskCount,
            scc.cppIds.size() == 1 ? "true" : "false",
            scc.internalEdgeCount, scc.internalDependencyEdgeCount, scc.internalActiveEdgeCount,
            scc.internalNeedActivateEdgeCount, scc.incomingEdgeCount, scc.outgoingEdgeCount);
    dumpJsonIntArray(fp, scc.predSccs);
    fprintf(fp, ", \"succ_sccs\": ");
    dumpJsonIntArray(fp, scc.succSccs);
    fprintf(fp, "}%s\n", sccId + 1 == schedule.sccs.size() ? "" : ",");
  }
  fprintf(fp, "  ],\n");

  fprintf(fp, "  \"layers\": [\n");
  for (size_t layerId = 0; layerId < schedule.layers.size(); layerId ++) {
    const MtDenseLayer& layer = schedule.layers[layerId];
    fprintf(fp, "    {\"layer_id\": %zu, \"scc_ids\": ", layerId);
    dumpJsonIntArray(fp, layer.sccIds);
    fprintf(fp, ", \"worker0_only\": %s, \"task_count\": %d, \"static_cost\": %d}%s\n",
            layer.workerZeroOnly ? "true" : "false", layer.taskCount, layer.staticCost,
            layerId + 1 == schedule.layers.size() ? "" : ",");
  }
  fprintf(fp, "  ],\n");
  std::vector<int> cycleSccIds;
  for (size_t sccId = 0; sccId < schedule.sccs.size(); sccId ++) {
    if (schedule.sccs[sccId].cppIds.size() > 1) cycleSccIds.push_back(static_cast<int>(sccId));
  }
  std::sort(cycleSccIds.begin(), cycleSccIds.end(), [&](int lhs, int rhs) {
    const MtDenseScc& lhsScc = schedule.sccs[(size_t)lhs];
    const MtDenseScc& rhsScc = schedule.sccs[(size_t)rhs];
    if (lhsScc.cppIds.size() != rhsScc.cppIds.size()) return lhsScc.cppIds.size() > rhsScc.cppIds.size();
    if (lhsScc.internalEdgeCount != rhsScc.internalEdgeCount) return lhsScc.internalEdgeCount > rhsScc.internalEdgeCount;
    return lhs < rhs;
  });

  fprintf(fp, "  \"cycle_scc_top_by_task_count\": [\n");
  size_t topCycleLimit = std::min<size_t>(cycleSccIds.size(), 20);
  for (size_t rank = 0; rank < topCycleLimit; rank ++) {
    int sccId = cycleSccIds[rank];
    const MtDenseScc& scc = schedule.sccs[(size_t)sccId];
    fprintf(fp, "    {\"rank\": %zu, \"scc_id\": %d, \"task_count\": %zu, \"static_cost\": %d, \"member_node_cost\": %d, \"worker0_only_task_count\": %d, \"always_active_task_count\": %d, \"internal_edge_count\": %d, \"internal_dependency_edge_count\": %d, \"internal_active_edge_count\": %d, \"internal_need_activate_edge_count\": %d, \"incoming_edge_count\": %d, \"outgoing_edge_count\": %d}%s\n",
            rank, sccId, scc.cppIds.size(), scc.staticCost, scc.memberNodeCost,
            scc.worker0OnlyTaskCount, scc.alwaysActiveTaskCount, scc.internalEdgeCount,
            scc.internalDependencyEdgeCount, scc.internalActiveEdgeCount,
            scc.internalNeedActivateEdgeCount, scc.incomingEdgeCount, scc.outgoingEdgeCount,
            rank + 1 == topCycleLimit ? "" : ",");
  }
  fprintf(fp, "  ],\n");

  fprintf(fp, "  \"cycle_sccs\": [\n");
  for (size_t rank = 0; rank < cycleSccIds.size(); rank ++) {
    int sccId = cycleSccIds[rank];
    const MtDenseScc& scc = schedule.sccs[(size_t)sccId];
    std::vector<int> cppSample;
    size_t sampleCount = std::min<size_t>(scc.cppIds.size(), 32);
    cppSample.insert(cppSample.end(), scc.cppIds.begin(), scc.cppIds.begin() + sampleCount);
    fprintf(fp, "    {\"scc_id\": %d, \"cpp_id_sample\": ", sccId);
    dumpJsonIntArray(fp, cppSample);
    fprintf(fp, ", \"worker0_only\": %s, \"worker0_only_task_count\": %d, \"always_active_task_count\": %d, \"task_count\": %zu, \"static_cost\": %d, \"member_node_cost\": %d, \"internal_edge_count\": %d, \"internal_dependency_edge_count\": %d, \"internal_active_edge_count\": %d, \"internal_need_activate_edge_count\": %d, \"incoming_edge_count\": %d, \"outgoing_edge_count\": %d}%s\n",
            scc.workerZeroOnly ? "true" : "false", scc.worker0OnlyTaskCount, scc.alwaysActiveTaskCount,
            scc.cppIds.size(), scc.staticCost, scc.memberNodeCost, scc.internalEdgeCount,
            scc.internalDependencyEdgeCount, scc.internalActiveEdgeCount,
            scc.internalNeedActivateEdgeCount, scc.incomingEdgeCount, scc.outgoingEdgeCount,
            rank + 1 == cycleSccIds.size() ? "" : ",");
  }
  fprintf(fp, "  ]\n");
  fprintf(fp, "}\n");
  fclose(fp);
  printf("[mt-dense-schedule] wrote %d tasks, %d edges, %zu sccs, valid=%d fallback=%s to %s\n",
         schedule.taskCount, schedule.edgeCount, schedule.sccs.size(), schedule.valid ? 1 : 0,
         schedule.fallbackReason.c_str(), path.c_str());
  logMtReportTimer("dense-schedule", mtReportTimerStart, getTime());
}

void graph::dumpMtCoarseRegionReport() {
  std::string baseName = globalConfig.InputBaseName.empty() ? name : globalConfig.InputBaseName;
  std::string path = globalConfig.OutputDir + "/" + baseName + "_mt_coarse_regions.json";
  FILE* fp = std::fopen(path.c_str(), "w");
  Assert(fp != nullptr, "failed to open mt coarse-region report %s", path.c_str());
  struct timeval mtReportTimerStart = getTime();
  struct timeval mtCoarsePhaseStart = mtReportTimerStart;
  auto mtCoarseLogPhase = [&](const char* name) {
    struct timeval now = getTime();
    logMtReportTimer(name, mtCoarsePhaseStart, now);
    mtCoarsePhaseStart = now;
  };
  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapForInvocation();
  mtCoarseLogPhase("coarse-region.task-map");
  const char* segmentReportEnv = std::getenv("GSIM_MT_SEGMENT_REPORT");
  bool segmentReportEnabled = segmentReportEnv != nullptr && segmentReportEnv[0] != '\0' && segmentReportEnv[0] != '0';
  MtCoarseRegionPlan coarsePlan = planMtCoarseRegionsForInvocation();
  mtCoarseLogPhase("coarse-region.plan-regions");
  MtPureBatchPlan fallbackPlan = planMtPureBatchesActiveFrequency(mtTasks);
  mtCoarseLogPhase("coarse-region.fallback-plan");

  struct MtSerialSegmentStats {
    int segmentCount = 0;
    int cleanRegionCount = 0;
    int maxSegmentRegions = 0;
    int maxSegmentTasks = 0;
    int maxSegmentStaticCost = 0;
    int activeBoundaryCount = 0;
    int orderingBoundaryCount = 0;
    int gapBoundaryCount = 0;
    int nonCleanBoundaryCount = 0;
    uint64_t totalSegmentRegions = 0;
    uint64_t totalSegmentTasks = 0;
    uint64_t totalSegmentStaticCost = 0;
    uint64_t regionCountHist[6] = {0, 0, 0, 0, 0, 0};
  } segmentStats;
  auto mtRegionCleanSerialFallback = [&](const MtCoarseRegion& region) -> bool {
    for (int rcid = region.beginCppId; rcid < region.endCppId; rcid ++) {
      auto mtIter = mtTasks.find(rcid);
      if (mtIter == mtTasks.end() || hasWorker0OnlyReason(mtIter->second.serialReasons) || !hasOnlyA44DirectFallbackReasons(mtIter->second.serialReasons)) return false;
    }
    return true;
  };
  bool cycleBatchReportEnabled = mtUseCycleBatchReport();
  std::vector<int> cycleBatchCleanRegionIds;
  std::vector<int> cycleBatchCppToCleanRegion(superId, -1);
  std::vector<int> cycleBatchCppToRegion(superId, -1);
  std::vector<int> cycleBatchRegionToClean;
  std::set<std::tuple<int, int, std::string>> cycleBatchEdges;
  std::set<std::tuple<int, int, std::string, std::string>> cycleBatchBarrierEdges;
  std::map<int, std::set<int>> cycleBatchSinkWords;
  if (cycleBatchReportEnabled) {
    cycleBatchRegionToClean.assign(coarsePlan.regions.size(), -1);
    for (size_t regionIndex = 0; regionIndex < coarsePlan.regions.size(); regionIndex ++) {
      const MtCoarseRegion& region = coarsePlan.regions[regionIndex];
      for (int cppId = region.beginCppId; cppId < region.endCppId; cppId ++) {
        if (cppId >= 0 && cppId < superId) cycleBatchCppToRegion[cppId] = static_cast<int>(regionIndex);
      }
    }
    for (size_t regionIndex = 0; regionIndex < coarsePlan.regions.size(); regionIndex ++) {
      const MtCoarseRegion& region = coarsePlan.regions[regionIndex];
      bool clean = region.runtimeEligible && mtRegionCleanSerialFallback(region);
      if (!clean) continue;
      int cleanIndex = static_cast<int>(cycleBatchCleanRegionIds.size());
      cycleBatchCleanRegionIds.push_back(static_cast<int>(regionIndex));
      cycleBatchRegionToClean[regionIndex] = cleanIndex;
      for (int cppId = region.beginCppId; cppId < region.endCppId; cppId ++) {
        if (cppId >= 0 && cppId < superId) cycleBatchCppToCleanRegion[cppId] = cleanIndex;
      }
    }
    auto addCycleBatchEdge = [&](int fromCppId, int toCppId, const char* kind) {
      if (fromCppId < 0 || fromCppId >= superId || toCppId < 0 || toCppId >= superId) return;
      int fromCleanRegion = cycleBatchCppToCleanRegion[fromCppId];
      int toCleanRegion = cycleBatchCppToCleanRegion[toCppId];
      int fromRegion = cycleBatchCppToRegion[fromCppId];
      int toRegion = cycleBatchCppToRegion[toCppId];
      if (fromRegion < 0 || toRegion < 0 || fromRegion == toRegion) return;
      if (fromCleanRegion >= 0 && toCleanRegion >= 0) {
        if (fromCleanRegion != toCleanRegion) cycleBatchEdges.insert(std::make_tuple(fromCleanRegion, toCleanRegion, std::string(kind)));
      } else if (fromCleanRegion >= 0) {
        cycleBatchBarrierEdges.insert(std::make_tuple(fromCleanRegion, toRegion, std::string("clean_to_region"), std::string(kind)));
      } else if (toCleanRegion >= 0) {
        cycleBatchBarrierEdges.insert(std::make_tuple(toCleanRegion, fromRegion, std::string("region_to_clean"), std::string(kind)));
      }
    };
    for (int cppId = 0; cppId < superId; cppId ++) {
      int fromRegion = cycleBatchCppToRegion[cppId];
      if (fromRegion < 0) continue;
      auto superIter = cppId2Super.find(cppId);
      if (superIter == cppId2Super.end() || !superIter->second) continue;
      SuperNode* super = superIter->second;
      for (SuperNode* next : super->next) if (next && next->cppId >= 0) addCycleBatchEdge(cppId, next->cppId, "order");
      for (SuperNode* next : super->depNext) if (next && next->cppId >= 0) addCycleBatchEdge(cppId, next->cppId, "order");
      for (Node* member : super->member) {
        if (!member) continue;
        for (int activeId : member->nextNeedActivate) if (activeId >= 0) addCycleBatchEdge(cppId, activeId, "active");
        for (int activeId : member->nextActiveId) {
          if (activeId < 0) continue;
          bool sameWordForward = (activeId / ACTIVE_WIDTH == cppId / ACTIVE_WIDTH) && activeId > cppId;
          if (!sameWordForward && cycleBatchCppToCleanRegion[cppId] >= 0) cycleBatchSinkWords[cycleBatchCppToCleanRegion[cppId]].insert(activeId / ACTIVE_WIDTH);
        }
      }
    }
  }
  struct CycleBatchTraceSummary {
    bool enabled = false;
    int cycles = 0;
    std::vector<int> phaseCounts;
    std::vector<int> batchRegionCounts;
    std::vector<int> multiRegionPhaseCounts;
    std::vector<int> largeRegionPhaseCounts;
    std::vector<int> largeCostPhaseCounts;
    std::vector<int> largestPhaseCosts;
    std::vector<double> batchableFractions;
  } cycleBatchTraceSummary;
  auto cycleBatchPctInt = [](std::vector<int> values, int pct) -> int {
    if (values.empty()) return 0;
    std::sort(values.begin(), values.end());
    return values[(values.size() - 1) * static_cast<size_t>(pct) / 100];
  };
  auto cycleBatchPctDouble = [](std::vector<double> values, int pct) -> double {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    return values[(values.size() - 1) * static_cast<size_t>(pct) / 100];
  };
  if (cycleBatchReportEnabled) {
    const char* tracePath = std::getenv("GSIM_MT_CYCLE_BATCH_TRACE");
    if (tracePath != nullptr && tracePath[0] != '\0') {
      std::ifstream trace(tracePath);
      if (trace.good()) {
        cycleBatchTraceSummary.enabled = true;
        std::vector<std::set<int>> cleanEdges(cycleBatchCleanRegionIds.size());
        std::vector<std::set<int>> barrierEdges(cycleBatchCleanRegionIds.size());
        for (const auto& edge : cycleBatchEdges) {
          int a = std::get<0>(edge), b = std::get<1>(edge);
          cleanEdges[a].insert(b);
          cleanEdges[b].insert(a);
        }
        for (const auto& edge : cycleBatchBarrierEdges) barrierEdges[std::get<0>(edge)].insert(std::get<1>(edge));
        auto edgeWithPending = [&](int cleanRegion, const std::vector<int>& pending) -> bool {
          for (int p : pending) if (cleanEdges[p].find(cleanRegion) != cleanEdges[p].end()) return true;
          return false;
        };
        auto barrierWithPending = [&](int regionIndex, const std::vector<int>& pending) -> bool {
          for (int p : pending) if (barrierEdges[p].find(regionIndex) != barrierEdges[p].end()) return true;
          return false;
        };
        auto finishBatch = [&](std::vector<std::vector<int>>& batches, std::vector<int>& pending) {
          if (!pending.empty()) { batches.push_back(pending); pending.clear(); }
        };
        std::string line;
        while (std::getline(trace, line)) {
          size_t pos = line.find(" tasks=");
          if (line.find("[mt-dyn-trace]") == std::string::npos || pos == std::string::npos) continue;
          std::set<int> activeClean;
          std::stringstream ss(line.substr(pos + 7));
          std::string item;
          while (std::getline(ss, item, ',')) {
            int cppId = std::atoi(item.c_str());
            if (cppId >= 0 && cppId < superId && cycleBatchCppToCleanRegion[cppId] >= 0) activeClean.insert(cycleBatchCppToCleanRegion[cppId]);
          }
          std::vector<std::vector<int>> batches;
          std::vector<int> pending;
          for (size_t regionIndex = 0; regionIndex < coarsePlan.regions.size(); regionIndex ++) {
            int cleanRegion = cycleBatchRegionToClean[regionIndex];
            bool isActiveClean = cleanRegion >= 0 && activeClean.find(cleanRegion) != activeClean.end();
            if (isActiveClean) {
              if (!pending.empty() && edgeWithPending(cleanRegion, pending)) finishBatch(batches, pending);
              pending.push_back(cleanRegion);
            } else if (!pending.empty()) {
              if ((cleanRegion >= 0 && edgeWithPending(cleanRegion, pending)) || barrierWithPending(static_cast<int>(regionIndex), pending)) finishBatch(batches, pending);
            }
          }
          finishBatch(batches, pending);
          cycleBatchTraceSummary.cycles ++;
          cycleBatchTraceSummary.phaseCounts.push_back(static_cast<int>(batches.size()));
          uint64_t activeCost = 0;
          for (int cleanRegion : activeClean) activeCost += static_cast<uint64_t>(coarsePlan.regions[cycleBatchCleanRegionIds[cleanRegion]].memberNodeCost);
          uint64_t batchableCost = 0;
          int multiRegionPhases = 0;
          int largeRegionPhases = 0;
          int largeCostPhases = 0;
          int largestPhaseCost = 0;
          for (const auto& batch : batches) {
            int batchCost = 0;
            for (int cleanRegion : batch) batchCost += coarsePlan.regions[cycleBatchCleanRegionIds[cleanRegion]].memberNodeCost;
            largestPhaseCost = std::max(largestPhaseCost, batchCost);
            if (batch.size() > 1) multiRegionPhases ++;
            if (batch.size() >= 10) largeRegionPhases ++;
            if (batchCost >= 50000) largeCostPhases ++;
            cycleBatchTraceSummary.batchRegionCounts.push_back(static_cast<int>(batch.size()));
            if (batch.size() > 1) batchableCost += static_cast<uint64_t>(batchCost);
          }
          cycleBatchTraceSummary.multiRegionPhaseCounts.push_back(multiRegionPhases);
          cycleBatchTraceSummary.largeRegionPhaseCounts.push_back(largeRegionPhases);
          cycleBatchTraceSummary.largeCostPhaseCounts.push_back(largeCostPhases);
          cycleBatchTraceSummary.largestPhaseCosts.push_back(largestPhaseCost);
          cycleBatchTraceSummary.batchableFractions.push_back(activeCost == 0 ? 0.0 : static_cast<double>(batchableCost) / static_cast<double>(activeCost));
        }
      }
    }
  }
  auto mtSegmentHasCrossEdge = [&](int segBeginCppId, int segEndCppId, const MtCoarseRegion& region, bool activeOnly) -> bool {
    for (int segCppId = segBeginCppId; segCppId < segEndCppId; segCppId ++) {
      for (int curCppId = region.beginCppId; curCppId < region.endCppId; curCppId ++) {
        if (activeOnly) {
          if (mtTaskHasActiveEdgeTo(segCppId, curCppId) || mtTaskHasActiveEdgeTo(curCppId, segCppId)) return true;
        } else {
          if (mtTaskHasOrderingEdgeTo(segCppId, curCppId) || mtTaskHasOrderingEdgeTo(curCppId, segCppId)) return true;
        }
      }
    }
    return false;
  };
  auto mtRecordSegment = [&](int regions, int tasks, int staticCost) {
    if (regions <= 0) return;
    segmentStats.segmentCount ++;
    segmentStats.totalSegmentRegions += (uint64_t)regions;
    segmentStats.totalSegmentTasks += (uint64_t)tasks;
    segmentStats.totalSegmentStaticCost += (uint64_t)staticCost;
    segmentStats.maxSegmentRegions = std::max(segmentStats.maxSegmentRegions, regions);
    segmentStats.maxSegmentTasks = std::max(segmentStats.maxSegmentTasks, tasks);
    segmentStats.maxSegmentStaticCost = std::max(segmentStats.maxSegmentStaticCost, staticCost);
    int bucket = regions <= 1 ? 0 : (regions == 2 ? 1 : (regions <= 4 ? 2 : (regions <= 8 ? 3 : (regions <= 16 ? 4 : 5))));
    segmentStats.regionCountHist[bucket] ++;
  };
  bool segmentOpen = false;
  int segmentBeginCppId = -1;
  int segmentEndCppId = -1;
  int segmentRegions = 0;
  int segmentTasks = 0;
  int segmentStaticCost = 0;
  if (segmentReportEnabled) {
  for (const MtCoarseRegion& region : coarsePlan.regions) {
    bool clean = region.runtimeEligible && mtRegionCleanSerialFallback(region);
    if (!clean) {
      if (segmentOpen) {
        mtRecordSegment(segmentRegions, segmentTasks, segmentStaticCost);
        segmentOpen = false;
        segmentStats.nonCleanBoundaryCount ++;
      }
      continue;
    }
    segmentStats.cleanRegionCount ++;
    bool startNew = !segmentOpen;
    if (segmentOpen) {
      if (segmentEndCppId != region.beginCppId) {
        segmentStats.gapBoundaryCount ++;
        startNew = true;
      } else if (mtSegmentHasCrossEdge(segmentBeginCppId, segmentEndCppId, region, true)) {
        segmentStats.activeBoundaryCount ++;
        startNew = true;
      } else if (mtSegmentHasCrossEdge(segmentBeginCppId, segmentEndCppId, region, false)) {
        segmentStats.orderingBoundaryCount ++;
        startNew = true;
      }
    }
    if (startNew) {
      if (segmentOpen) mtRecordSegment(segmentRegions, segmentTasks, segmentStaticCost);
      segmentOpen = true;
      segmentBeginCppId = region.beginCppId;
      segmentEndCppId = region.endCppId;
      segmentRegions = 1;
      segmentTasks = region.taskCount;
      segmentStaticCost = region.staticCost;
    } else {
      segmentEndCppId = region.endCppId;
      segmentRegions ++;
      segmentTasks += region.taskCount;
      segmentStaticCost += region.staticCost;
    }
  }
  if (segmentOpen) mtRecordSegment(segmentRegions, segmentTasks, segmentStaticCost);
  }
  int runtimeEligibleCount = 0;
  int maxTaskCount = 0;
  int maxActiveWordSpan = 0;
  int maxParallelWidth = 0;
  std::map<std::string, int> blockerCounts;
  for (const MtCoarseRegion& region : coarsePlan.regions) {
    if (region.runtimeEligible) runtimeEligibleCount ++;
    maxTaskCount = std::max(maxTaskCount, region.taskCount);
    maxActiveWordSpan = std::max(maxActiveWordSpan, region.activeWordSpan);
    maxParallelWidth = std::max(maxParallelWidth, region.estimatedMaxParallelWidth);
    for (const std::string& blocker : region.blockers) blockerCounts[blocker] ++;
  }
  mtCoarseLogPhase("coarse-region.metadata");

  fprintf(fp, "{\n");
  fprintf(fp, "  \"format\": \"gsim.mt-coarse-region-report.v2\",\n");
  fprintf(fp, "  \"mode\": \"%s\",\n", globalConfig.MtBatchFormationMode.c_str());
  fprintf(fp, "  \"coarse_runtime\": \"%s\",\n", globalConfig.MtCoarseRuntimeMode.c_str());
  fprintf(fp, "  \"coarse_profitability\": \"%s\",\n", globalConfig.MtCoarseProfitabilityMode.c_str());
  fprintf(fp, "  \"coarse_worker_policy\": \"%s\",\n", globalConfig.MtCoarseWorkerPolicyMode.c_str());
  fprintf(fp, "  \"task_count\": %d,\n", superId);
  fprintf(fp, "  \"active_width\": %d,\n", ACTIVE_WIDTH);
  fprintf(fp, "  \"same_word_fallback_batch_count\": %zu,\n", fallbackPlan.batches.size());
  fprintf(fp, "  \"candidate_region_count\": %zu,\n", coarsePlan.regions.size());
  fprintf(fp, "  \"runtime_eligible_region_count\": %d,\n", runtimeEligibleCount);
  fprintf(fp, "  \"max_task_count\": %d,\n", maxTaskCount);
  fprintf(fp, "  \"serial_fallback_segment_summary\": {\n");
  fprintf(fp, "    \"enabled\": %s,\n", segmentReportEnabled ? "true" : "false");
  fprintf(fp, "    \"candidate_kind\": \"very_conservative_contiguous_static_bidirectional_boundary_checked\",\n");
  fprintf(fp, "    \"clean_region_count\": %d,\n", segmentStats.cleanRegionCount);
  fprintf(fp, "    \"segment_count\": %d,\n", segmentStats.segmentCount);
  fprintf(fp, "    \"max_segment_regions\": %d,\n", segmentStats.maxSegmentRegions);
  fprintf(fp, "    \"max_segment_tasks\": %d,\n", segmentStats.maxSegmentTasks);
  fprintf(fp, "    \"max_segment_static_cost\": %d,\n", segmentStats.maxSegmentStaticCost);
  fprintf(fp, "    \"total_segment_regions\": %lu,\n", segmentStats.totalSegmentRegions);
  fprintf(fp, "    \"total_segment_tasks\": %lu,\n", segmentStats.totalSegmentTasks);
  fprintf(fp, "    \"total_segment_static_cost\": %lu,\n", segmentStats.totalSegmentStaticCost);
  fprintf(fp, "    \"active_boundary_count\": %d,\n", segmentStats.activeBoundaryCount);
  fprintf(fp, "    \"ordering_boundary_count\": %d,\n", segmentStats.orderingBoundaryCount);
  fprintf(fp, "    \"gap_boundary_count\": %d,\n", segmentStats.gapBoundaryCount);
  fprintf(fp, "    \"non_clean_boundary_count\": %d,\n", segmentStats.nonCleanBoundaryCount);
  fprintf(fp, "    \"region_count_hist\": {\"1\": %lu, \"2\": %lu, \"3_4\": %lu, \"5_8\": %lu, \"9_16\": %lu, \"17_plus\": %lu}\n",
          segmentStats.regionCountHist[0], segmentStats.regionCountHist[1], segmentStats.regionCountHist[2],
          segmentStats.regionCountHist[3], segmentStats.regionCountHist[4], segmentStats.regionCountHist[5]);
  fprintf(fp, "  },\n");
  fprintf(fp, "  \"max_active_word_span\": %d,\n", maxActiveWordSpan);
  fprintf(fp, "  \"max_parallel_width\": %d,\n", maxParallelWidth);
  fprintf(fp, "  \"blocker_counts\": {");
  bool firstBlocker = true;
  for (const auto& blocker : blockerCounts) {
    if (!firstBlocker) fprintf(fp, ", ");
    firstBlocker = false;
    fprintf(fp, "\"%s\": %d", jsonEscape(blocker.first).c_str(), blocker.second);
  }
  fprintf(fp, "},\n");
  if (cycleBatchReportEnabled) {
    fprintf(fp, "  \"cycle_batch_report\": {\n");
    fprintf(fp, "    \"enabled\": true,\n");
    fprintf(fp, "    \"candidate_kind\": \"static_clean_region_graph_report_only\",\n");
    fprintf(fp, "    \"clean_region_count\": %zu,\n", cycleBatchCleanRegionIds.size());
    fprintf(fp, "    \"directed_edge_count\": %zu,\n", cycleBatchEdges.size());
    fprintf(fp, "    \"barrier_edge_count\": %zu,\n", cycleBatchBarrierEdges.size());
    fprintf(fp, "    \"clean_regions\": [\n");
    for (size_t i = 0; i < cycleBatchCleanRegionIds.size(); i ++) {
      int regionIndex = cycleBatchCleanRegionIds[i];
      const MtCoarseRegion& region = coarsePlan.regions[regionIndex];
      fprintf(fp, "      {\"clean_region_index\": %zu, \"region_index\": %d, \"begin_cpp_id\": %d, \"end_cpp_id\": %d, \"task_count\": %d, \"active_word_span\": %d, \"static_cost\": %d, \"member_node_cost\": %d, \"sink_words\": ",
              i, regionIndex, region.beginCppId, region.endCppId, region.taskCount, region.activeWordSpan, region.staticCost, region.memberNodeCost);
      auto sinkIter = cycleBatchSinkWords.find(static_cast<int>(i));
      if (sinkIter == cycleBatchSinkWords.end()) dumpJsonIntArray(fp, std::set<int>());
      else dumpJsonIntArray(fp, sinkIter->second);
      fprintf(fp, "}%s\n", i + 1 == cycleBatchCleanRegionIds.size() ? "" : ",");
    }
    fprintf(fp, "    ],\n");
    fprintf(fp, "    \"directed_edges\": [\n");
    size_t edgeIndex = 0;
    for (const auto& edge : cycleBatchEdges) {
      int fromRegion = std::get<0>(edge);
      int toRegion = std::get<1>(edge);
      const std::string& kind = std::get<2>(edge);
      fprintf(fp, "      {\"from_clean_region\": %d, \"to_clean_region\": %d, \"kind\": \"%s\"}%s\n",
              fromRegion, toRegion, kind.c_str(), edgeIndex + 1 == cycleBatchEdges.size() ? "" : ",");
      edgeIndex ++;
    }
    fprintf(fp, "    ],\n");
    fprintf(fp, "    \"barrier_edges\": [\n");
    size_t barrierEdgeIndex = 0;
    for (const auto& edge : cycleBatchBarrierEdges) {
      int cleanRegion = std::get<0>(edge);
      int otherRegion = std::get<1>(edge);
      const std::string& direction = std::get<2>(edge);
      const std::string& kind = std::get<3>(edge);
      fprintf(fp, "      {\"clean_region\": %d, \"other_region\": %d, \"direction\": \"%s\", \"kind\": \"%s\"}%s\n",
              cleanRegion, otherRegion, direction.c_str(), kind.c_str(), barrierEdgeIndex + 1 == cycleBatchBarrierEdges.size() ? "" : ",");
      barrierEdgeIndex ++;
    }
    fprintf(fp, "    ],\n");
    fprintf(fp, "    \"trace_summary\": {\n");
    fprintf(fp, "      \"enabled\": %s,\n", cycleBatchTraceSummary.enabled ? "true" : "false");
    fprintf(fp, "      \"cycles\": %d,\n", cycleBatchTraceSummary.cycles);
    fprintf(fp, "      \"phase_count_p50\": %d,\n", cycleBatchPctInt(cycleBatchTraceSummary.phaseCounts, 50));
    fprintf(fp, "      \"phase_count_p95\": %d,\n", cycleBatchPctInt(cycleBatchTraceSummary.phaseCounts, 95));
    fprintf(fp, "      \"phase_count_max\": %d,\n", cycleBatchPctInt(cycleBatchTraceSummary.phaseCounts, 100));
    fprintf(fp, "      \"multi_region_phase_count_p50\": %d,\n", cycleBatchPctInt(cycleBatchTraceSummary.multiRegionPhaseCounts, 50));
    fprintf(fp, "      \"multi_region_phase_count_p95\": %d,\n", cycleBatchPctInt(cycleBatchTraceSummary.multiRegionPhaseCounts, 95));
    fprintf(fp, "      \"large_region_phase_count_p50\": %d,\n", cycleBatchPctInt(cycleBatchTraceSummary.largeRegionPhaseCounts, 50));
    fprintf(fp, "      \"large_region_phase_count_p95\": %d,\n", cycleBatchPctInt(cycleBatchTraceSummary.largeRegionPhaseCounts, 95));
    fprintf(fp, "      \"large_cost_phase_count_p50\": %d,\n", cycleBatchPctInt(cycleBatchTraceSummary.largeCostPhaseCounts, 50));
    fprintf(fp, "      \"large_cost_phase_count_p95\": %d,\n", cycleBatchPctInt(cycleBatchTraceSummary.largeCostPhaseCounts, 95));
    fprintf(fp, "      \"largest_phase_cost_p50\": %d,\n", cycleBatchPctInt(cycleBatchTraceSummary.largestPhaseCosts, 50));
    fprintf(fp, "      \"largest_phase_cost_p95\": %d,\n", cycleBatchPctInt(cycleBatchTraceSummary.largestPhaseCosts, 95));
    fprintf(fp, "      \"batch_regions_p50\": %d,\n", cycleBatchPctInt(cycleBatchTraceSummary.batchRegionCounts, 50));
    fprintf(fp, "      \"batch_regions_p95\": %d,\n", cycleBatchPctInt(cycleBatchTraceSummary.batchRegionCounts, 95));
    fprintf(fp, "      \"batch_regions_max\": %d,\n", cycleBatchPctInt(cycleBatchTraceSummary.batchRegionCounts, 100));
    fprintf(fp, "      \"batchable_fraction_p50\": %.6f,\n", cycleBatchPctDouble(cycleBatchTraceSummary.batchableFractions, 50));
    fprintf(fp, "      \"batchable_fraction_p95\": %.6f,\n", cycleBatchPctDouble(cycleBatchTraceSummary.batchableFractions, 95));
    fprintf(fp, "      \"batchable_fraction_max\": %.6f\n", cycleBatchPctDouble(cycleBatchTraceSummary.batchableFractions, 100));
    fprintf(fp, "    }\n");
    fprintf(fp, "  },\n");
  }
  fprintf(fp, "  \"regions\": [\n");
  mtCoarseLogPhase("coarse-region.header-json");
  for (size_t i = 0; i < coarsePlan.regions.size(); i ++) {
    const MtCoarseRegion& region = coarsePlan.regions[i];
    fprintf(fp, "    {\n");
    fprintf(fp, "      \"begin_cpp_id\": %d,\n", region.beginCppId);
    fprintf(fp, "      \"end_cpp_id\": %d,\n", region.endCppId);
    fprintf(fp, "      \"task_count\": %d,\n", region.taskCount);
    fprintf(fp, "      \"active_word_span\": %d,\n", region.activeWordSpan);
    fprintf(fp, "      \"static_cost\": %d,\n", region.staticCost);
    fprintf(fp, "      \"member_node_cost\": %d,\n", region.memberNodeCost);
    fprintf(fp, "      \"expected_active_cost\": %d,\n", region.expectedActiveCost);
    fprintf(fp, "      \"estimated_useful_work\": %d,\n", region.estimatedUsefulWork);
    fprintf(fp, "      \"pure_task_count\": %d,\n", region.pureTaskCount);
    fprintf(fp, "      \"serial_blocker_count\": %d,\n", region.serialBlockerCount);
    fprintf(fp, "      \"dependency_edges_inside\": %d,\n", region.dependencyEdgeCount);
    fprintf(fp, "      \"active_visibility_edges\": %d,\n", region.activeVisibilityEdgeCount);
    fprintf(fp, "      \"same_cycle_activation_hazards\": %d,\n", region.sameCycleActivationHazardCount);
    fprintf(fp, "      \"estimated_layer_count\": %d,\n", region.estimatedLayerCount);
    fprintf(fp, "      \"estimated_max_parallel_width\": %d,\n", region.estimatedMaxParallelWidth);
    fprintf(fp, "      \"mtask_count\": %zu,\n", region.mtasks.size());
    fprintf(fp, "      \"mtask_static_cost_min\": %d,\n", region.mtaskStaticCostMin);
    fprintf(fp, "      \"mtask_static_cost_max\": %d,\n", region.mtaskStaticCostMax);
    fprintf(fp, "      \"mtask_static_cost_total\": %d,\n", region.mtaskStaticCostTotal);
    fprintf(fp, "      \"mtask_member_node_cost_max\": %d,\n", region.mtaskMemberNodeCostMax);
    fprintf(fp, "      \"mtask_member_node_cost_total\": %d,\n", region.mtaskMemberNodeCostTotal);
    fprintf(fp, "      \"estimated_copy_words_at_t4\": %d,\n", 4 * region.activeWordSpan);
    fprintf(fp, "      \"estimated_merge_words_at_t4\": %d,\n", 4 * region.activeWordSpan);
    fprintf(fp, "      \"static_recommended_workers\": {");
    for (int configuredWorkers : {1, 2, 4, 8, 16}) {
      int recommendedWorkers = mtCoarseStaticRecommendedWorkers(region, configuredWorkers);
      bool staticAdmitted = mtCoarseStaticAdmitsRegion(region, recommendedWorkers);
      fprintf(fp, "%s\"t%d\": {\"workers\": %d, \"admitted\": %s}",
              configuredWorkers == 1 ? "" : ", ", configuredWorkers, recommendedWorkers,
              staticAdmitted ? "true" : "false");
    }
    fprintf(fp, "},\n");
    fprintf(fp, "      \"recommended_workers\": {");
    for (int configuredWorkers : {1, 2, 4, 8, 16}) {
      int recommendedWorkers = mtCoarseRecommendedWorkersForPolicy(region, configuredWorkers, globalConfig.MtCoarseWorkerPolicyMode);
      bool admitted = mtCoarseAdmitsRegionForPolicy(region, recommendedWorkers, globalConfig.MtCoarseWorkerPolicyMode);
      fprintf(fp, "%s\"t%d\": {\"workers\": %d, \"admitted\": %s}",
              configuredWorkers == 1 ? "" : ", ", configuredWorkers, recommendedWorkers,
              admitted ? "true" : "false");
    }
    fprintf(fp, "},\n");
    fprintf(fp, "      \"mtask_assignments\": {");
    bool firstAssignment = true;
    for (int configuredWorkers : {1, 2, 4, 8, 16}) {
      MtCoarseMTaskAssignment assignment =
        mtBuildCoarseMTaskAssignment(region, configuredWorkers, globalConfig.MtCoarseWorkerPolicyMode);
      if (!firstAssignment) fprintf(fp, ", ");
      firstAssignment = false;
      fprintf(fp, "\"t%d\": {", configuredWorkers);
      fprintf(fp, "\"requested_workers\": %d, ", assignment.requestedWorkers);
      fprintf(fp, "\"effective_workers\": %d, ", assignment.effectiveWorkers);
      fprintf(fp, "\"admitted\": %s, ", assignment.admitted ? "true" : "false");
      fprintf(fp, "\"contiguous_worst_static_cost\": %d, ", assignment.contiguousWorstStaticCost);
      fprintf(fp, "\"contiguous_best_static_cost\": %d, ", assignment.contiguousBestStaticCost);
      fprintf(fp, "\"contiguous_worst_task_count\": %d, ", assignment.contiguousWorstTaskCount);
      fprintf(fp, "\"balanced_worst_static_cost\": %d, ", assignment.balancedWorstStaticCost);
      fprintf(fp, "\"balanced_best_static_cost\": %d, ", assignment.balancedBestStaticCost);
      fprintf(fp, "\"balanced_worst_task_count\": %d, ", assignment.balancedWorstTaskCount);
      fprintf(fp, "\"worker_static_costs\": ");
      dumpJsonIntArray(fp, assignment.workerStaticCosts);
      fprintf(fp, ", \"worker_task_counts\": ");
      dumpJsonIntArray(fp, assignment.workerTaskCounts);
      fprintf(fp, ", \"worker_mtask_indices\": [");
      for (size_t worker = 0; worker < assignment.workerMTaskIndices.size(); worker ++) {
        if (worker != 0) fprintf(fp, ", ");
        dumpJsonIntArray(fp, assignment.workerMTaskIndices[worker]);
      }
      fprintf(fp, "]}");
    }
    fprintf(fp, "},\n");
    fprintf(fp, "      \"replication_candidate_count\": %d,\n", region.replicationCandidateCount);
    fprintf(fp, "      \"runtime_eligible\": %s,\n", region.runtimeEligible ? "true" : "false");
    fprintf(fp, "      \"blockers\": ");
    dumpJsonStringArray(fp, region.blockers);
    fprintf(fp, ",\n");
    fprintf(fp, "      \"layers\": [\n");
    for (size_t layerIdx = 0; layerIdx < region.layers.size(); layerIdx ++) {
      const MtCoarseLayer& layer = region.layers[layerIdx];
      fprintf(fp, "        {\"index\": %zu, \"task_cpp_ids\": ", layerIdx);
      dumpJsonIntArray(fp, layer.taskCppIds);
      fprintf(fp, "}%s\n", layerIdx + 1 == region.layers.size() ? "" : ",");
    }
    fprintf(fp, "      ],\n");
    fprintf(fp, "      \"mtasks\": [\n");
    for (size_t mtaskIdx = 0; mtaskIdx < region.mtasks.size(); mtaskIdx ++) {
      const MtCoarseMTask& mtask = region.mtasks[mtaskIdx];
      fprintf(fp, "        {\n");
      fprintf(fp, "          \"index\": %zu,\n", mtaskIdx);
      fprintf(fp, "          \"task_count\": %d,\n", mtask.taskCount);
      fprintf(fp, "          \"static_cost\": %d,\n", mtask.staticCost);
      fprintf(fp, "          \"member_node_cost\": %d,\n", mtask.memberNodeCost);
      fprintf(fp, "          \"ordering_edges_inside\": %d,\n", mtask.orderingEdgeCount);
      fprintf(fp, "          \"layer_task_cpp_ids\": [");
      for (size_t layerIdx = 0; layerIdx < mtask.layerTaskCppIds.size(); layerIdx ++) {
        if (layerIdx != 0) fprintf(fp, ", ");
        dumpJsonIntArray(fp, mtask.layerTaskCppIds[layerIdx]);
      }
      fprintf(fp, "]\n");
      fprintf(fp, "        }%s\n", mtaskIdx + 1 == region.mtasks.size() ? "" : ",");
    }
    fprintf(fp, "      ]\n");
    fprintf(fp, "    }%s\n", i + 1 == coarsePlan.regions.size() ? "" : ",");
  }
  mtCoarseLogPhase("coarse-region.regions-json");
  fprintf(fp, "  ]\n");
  fprintf(fp, "}\n");
  fclose(fp);
  mtCoarseLogPhase("coarse-region.close");
  printf("[mt-coarse-region] wrote %zu regions (%d runtime eligible) to %s\n",
         coarsePlan.regions.size(), runtimeEligibleCount, path.c_str());
  logMtReportTimer("coarse-region", mtReportTimerStart, getTime());
}

void graph::dumpMtReadyBatchReport() {
  std::string baseName = globalConfig.InputBaseName.empty() ? name : globalConfig.InputBaseName;
  std::string path = globalConfig.OutputDir + "/" + baseName + "_mt_ready_batch_lanes.json";
  FILE* fp = std::fopen(path.c_str(), "w");
  Assert(fp != nullptr, "failed to open mt ready-batch lane report %s", path.c_str());
  struct timeval mtReportTimerStart = getTime();

  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapForInvocation();
  std::vector<MtStateUpdateTraceInfo> stateUpdateTraceInfo = buildMtStateUpdateTraceInfoForInvocation(mtTasks);
  MtCoarseRegionPlan coarsePlan = planMtCoarseRegionsForInvocation();

  struct ReadyBatchCapStats {
    int cap = 1;
    int staticSerial = 0;
    int staticMakespan = 0;
    int staticDispatches = 0;
    uint64_t traceSerial = 0;
    uint64_t traceMakespan = 0;
    uint64_t traceDispatches = 0;
    int traceCycles = 0;
    std::vector<int> traceMakespans;
    std::vector<int> traceDispatchCounts;
  };
  struct ReadyBatchScheduleResult {
    int serial = 0;
    int makespan = 0;
    int dispatches = 0;
  };
  struct ReadyBatchLocalEvalResult {
    int serial = 0;
    int makespan = 0;
    int dispatches = 0;
    int activeSccs = 0;
    int activeTasks = 0;
    int selectedTasks = 0;
    int gapTasks = 0;
    int serialTasks = 0;
    int serialTaskDispatches = 0;
  };
  struct ReadyBatchLocalEvalStats {
    int workerCap = 4;
    int chunkCap = 4;
    int traceCycles = 0;
    uint64_t traceSerial = 0;
    uint64_t traceMakespan = 0;
    uint64_t traceDispatches = 0;
    uint64_t traceActiveSccs = 0;
    uint64_t traceActiveTasks = 0;
    uint64_t traceSelectedTasks = 0;
    uint64_t traceGapTasks = 0;
    uint64_t traceSerialTasks = 0;
    uint64_t traceSerialTaskDispatches = 0;
    uint64_t traceActiveWordOrVolume = 0;
    std::vector<int> traceMakespans;
    std::vector<int> traceDispatchCounts;
    std::vector<int> traceSerialTaskDispatchCounts;
    std::vector<int> traceActiveWordOrVolumes;
  };
  struct ReadyBatchLaneGraph {
    std::string name;
    std::vector<int> regionIndices;
    bool valid = true;
    std::string invalidReason;
    int taskCount = 0;
    int memberNodeCost = 0;
    int activeWordSpanSum = 0;
    int envelopeBeginCppId = -1;
    int envelopeEndCppId = -1;
    std::vector<std::pair<int, int>> gapIntervals;
    int gapTaskCount = 0;
    int gapPureTaskCount = 0;
    int gapSerialTaskCount = 0;
    int gapStateUpdateTaskCount = 0;
    int gapWorker0OnlyTaskCount = 0;
    int gapOrderLaneToGapEdges = 0;
    int gapOrderGapToLaneEdges = 0;
    int gapActiveLaneToGapEdges = 0;
    int gapActiveGapToLaneEdges = 0;
    std::vector<int> cppIds;
    std::map<int, int> cppToLocal;
    std::vector<std::set<int>> localSucc;
    std::vector<int> sccOfLocal;
    std::vector<int> sccCost;
    std::vector<std::set<int>> sccSucc;
    std::vector<std::set<int>> sccPred;
    std::vector<int> topo;
    int largestScc = 0;
    int sccEdgeCount = 0;
    std::vector<int> sccSerialTaskCount;
    std::vector<ReadyBatchCapStats> caps;
    int envelopeTaskCount = 0;
    int envelopeSelectedTaskCount = 0;
    int envelopeGapTaskCount = 0;
    int envelopePureTaskCount = 0;
    int envelopeSerialTaskCount = 0;
    int envelopeStateUpdateTaskCount = 0;
    int envelopeWorker0OnlyTaskCount = 0;
    std::vector<int> envelopeCppIds;
    std::map<int, int> envelopeCppToLocal;
    std::vector<std::set<int>> envelopeLocalSucc;
    std::vector<int> envelopeSccOfLocal;
    std::vector<int> envelopeSccCost;
    std::vector<std::set<int>> envelopeSccSucc;
    std::vector<std::set<int>> envelopeSccPred;
    std::vector<int> envelopeTopo;
    int envelopeLargestScc = 0;
    int envelopeSccEdgeCount = 0;
    std::vector<int> envelopeSccSerialTaskCount;
    std::vector<int> envelopeSccSelectedTaskCount;
    std::vector<int> envelopeSccGapTaskCount;
    std::vector<std::map<std::string, int>> envelopeSccSerialReasonTaskCount;
    std::map<std::string, int> envelopeSerialReasonTaskCount;
    std::map<std::string, uint64_t> envelopeTraceSerialReasonTaskCount;
    std::map<int, uint64_t> envelopeTraceActiveWordHits;
    int envelopeStateUpdateBlockedTaskCount = 0;
    int envelopeStateUpdateLocalSafeOnlyTaskCount = 0;
    int envelopeStateUpdateRuntimeSafeTaskCount = 0;
    std::map<std::string, int> envelopeStateUpdateTargetWriterConflictTaskCount;
    std::map<std::string, int> envelopeStateUpdateRuntimeBlockReasonTaskCount;
    uint64_t envelopeTraceStateUpdateBlockedHits = 0;
    uint64_t envelopeTraceStateUpdateLocalSafeOnlyHits = 0;
    uint64_t envelopeTraceStateUpdateRuntimeSafeHits = 0;
    std::map<std::string, uint64_t> envelopeTraceStateUpdateTargetWriterConflictHits;
    std::map<std::string, uint64_t> envelopeTraceStateUpdateRuntimeBlockReasonHits;
    ReadyBatchLocalEvalStats envelopeLocalEvalSelectedOnly;
    ReadyBatchLocalEvalStats envelopeLocalEvalSerialWorker0;
    ReadyBatchLocalEvalStats envelopeLocalEval;
    std::vector<ReadyBatchCapStats> envelopeCaps;
  };

  auto readyBatchPctInt = [](std::vector<int> values, int pct) -> int {
    if (values.empty()) return 0;
    std::sort(values.begin(), values.end());
    return values[(values.size() - 1) * static_cast<size_t>(pct) / 100];
  };
  auto readyBatchPrimarySerialReason = [](const MtTaskInfo* task) -> std::string {
    if (task == nullptr || task->serialReasons.empty()) return "unknown";
    return task->serialReasons[0];
  };
  auto dumpReadyBatchStringIntCountArray = [](FILE* out, const std::map<std::string, int>& values) {
    fprintf(out, "[");
    bool first = true;
    for (const auto& kv : values) {
      if (!first) fprintf(out, ", ");
      first = false;
      fprintf(out, "{\"name\": \"%s\", \"count\": %d}", jsonEscape(kv.first).c_str(), kv.second);
    }
    fprintf(out, "]");
  };
  auto dumpReadyBatchStringUint64CountArray = [](FILE* out, const std::map<std::string, uint64_t>& values) {
    fprintf(out, "[");
    bool first = true;
    for (const auto& kv : values) {
      if (!first) fprintf(out, ", ");
      first = false;
      fprintf(out, "{\"name\": \"%s\", \"count\": %lu}", jsonEscape(kv.first).c_str(), kv.second);
    }
    fprintf(out, "]");
  };
  auto readyBatchStateInfoForCpp = [&](int cppId) -> const MtStateUpdateTraceInfo* {
    if (cppId < 0 || cppId >= static_cast<int>(stateUpdateTraceInfo.size())) return nullptr;
    const MtStateUpdateTraceInfo& info = stateUpdateTraceInfo[cppId];
    if (!info.hasStateUpdate) return nullptr;
    return &info;
  };
  auto accumulateReadyBatchStateUpdateStatic = [&](ReadyBatchLaneGraph& lane, int cppId) {
    const MtStateUpdateTraceInfo* info = readyBatchStateInfoForCpp(cppId);
    if (info == nullptr) return;
    if (info->runtimeSafeCandidate) lane.envelopeStateUpdateRuntimeSafeTaskCount ++;
    else if (info->localSafeCandidate) lane.envelopeStateUpdateLocalSafeOnlyTaskCount ++;
    else lane.envelopeStateUpdateBlockedTaskCount ++;
    lane.envelopeStateUpdateTargetWriterConflictTaskCount[info->targetWriterConflictKind.empty() ? "none" : info->targetWriterConflictKind] ++;
    if (info->runtimeBlockReasons.empty()) {
      lane.envelopeStateUpdateRuntimeBlockReasonTaskCount["none"] ++;
    } else {
      for (const std::string& reason : info->runtimeBlockReasons) lane.envelopeStateUpdateRuntimeBlockReasonTaskCount[reason] ++;
    }
  };
  auto accumulateReadyBatchStateUpdateTrace = [&](ReadyBatchLaneGraph& lane, int cppId) {
    const MtStateUpdateTraceInfo* info = readyBatchStateInfoForCpp(cppId);
    if (info == nullptr) return;
    if (info->runtimeSafeCandidate) lane.envelopeTraceStateUpdateRuntimeSafeHits ++;
    else if (info->localSafeCandidate) lane.envelopeTraceStateUpdateLocalSafeOnlyHits ++;
    else lane.envelopeTraceStateUpdateBlockedHits ++;
    lane.envelopeTraceStateUpdateTargetWriterConflictHits[info->targetWriterConflictKind.empty() ? "none" : info->targetWriterConflictKind] ++;
    if (info->runtimeBlockReasons.empty()) {
      lane.envelopeTraceStateUpdateRuntimeBlockReasonHits["none"] ++;
    } else {
      for (const std::string& reason : info->runtimeBlockReasons) lane.envelopeTraceStateUpdateRuntimeBlockReasonHits[reason] ++;
    }
  };

  auto addReadyBatchEdge = [](std::vector<std::set<int>>& succ,
                              const std::map<int, int>& cppToLocal,
                              int fromCppId, int toCppId) {
    auto fromIter = cppToLocal.find(fromCppId);
    if (fromIter == cppToLocal.end()) return;
    auto toIter = cppToLocal.find(toCppId);
    if (toIter == cppToLocal.end()) return;
    if (fromIter->second == toIter->second) return;
    succ[fromIter->second].insert(toIter->second);
  };

  auto computeReadyBatchScc = [&](const std::vector<int>& cppIds,
                                  const std::vector<std::set<int>>& localSucc,
                                  std::vector<int>& sccOfLocal,
                                  std::vector<int>& sccCost,
                                  std::vector<std::set<int>>& sccSucc,
                                  std::vector<std::set<int>>& sccPred,
                                  std::vector<int>& topo,
                                  int& largestScc,
                                  int& sccEdgeCount,
                                  bool& valid,
                                  std::string& invalidReason) {
    int localCount = static_cast<int>(cppIds.size());
    std::vector<int> index(localCount, -1), lowlink(localCount, 0), stack;
    std::vector<char> onStack(localCount, 0);
    int nextIndex = 0;
    std::vector<std::vector<int>> sccs;
    std::function<void(int)> strongConnect = [&](int v) {
      index[v] = lowlink[v] = nextIndex ++;
      stack.push_back(v);
      onStack[v] = 1;
      for (int w : localSucc[v]) {
        if (index[w] < 0) {
          strongConnect(w);
          lowlink[v] = std::min(lowlink[v], lowlink[w]);
        } else if (onStack[w]) {
          lowlink[v] = std::min(lowlink[v], index[w]);
        }
      }
      if (lowlink[v] == index[v]) {
        std::vector<int> component;
        while (!stack.empty()) {
          int w = stack.back();
          stack.pop_back();
          onStack[w] = 0;
          component.push_back(w);
          if (w == v) break;
        }
        sccs.push_back(component);
      }
    };
    for (int local = 0; local < localCount; local ++) if (index[local] < 0) strongConnect(local);

    sccOfLocal.assign(localCount, -1);
    sccCost.assign(sccs.size(), 0);
    largestScc = 0;
    for (size_t scc = 0; scc < sccs.size(); scc ++) {
      largestScc = std::max(largestScc, static_cast<int>(sccs[scc].size()));
      sccCost[scc] = static_cast<int>(sccs[scc].size());
      for (int local : sccs[scc]) sccOfLocal[local] = static_cast<int>(scc);
    }
    sccSucc.assign(sccs.size(), std::set<int>());
    sccPred.assign(sccs.size(), std::set<int>());
    sccEdgeCount = 0;
    for (int local = 0; local < localCount; local ++) {
      int fromScc = sccOfLocal[local];
      for (int toLocal : localSucc[local]) {
        int toScc = sccOfLocal[toLocal];
        if (fromScc == toScc) continue;
        if (sccSucc[fromScc].insert(toScc).second) {
          sccPred[toScc].insert(fromScc);
          sccEdgeCount ++;
        }
      }
    }
    std::vector<int> indegree(sccs.size(), 0);
    std::deque<int> ready;
    topo.clear();
    for (size_t scc = 0; scc < sccs.size(); scc ++) {
      indegree[scc] = static_cast<int>(sccPred[scc].size());
      if (indegree[scc] == 0) ready.push_back(static_cast<int>(scc));
    }
    while (!ready.empty()) {
      int scc = ready.front();
      ready.pop_front();
      topo.push_back(scc);
      for (int succ : sccSucc[scc]) {
        indegree[succ] --;
        if (indegree[succ] == 0) ready.push_back(succ);
      }
    }
    if (topo.size() != sccs.size()) {
      valid = false;
      if (invalidReason.empty()) invalidReason = "condensation_toposort_failed";
    }
  };

  auto buildReadyBatchLane = [&](const std::string& laneName, const std::vector<int>& regionIndices) {
    ReadyBatchLaneGraph lane;
    lane.name = laneName;
    lane.regionIndices = regionIndices;
    for (int regionIndex : regionIndices) {
      if (regionIndex < 0 || regionIndex >= static_cast<int>(coarsePlan.regions.size())) {
        lane.valid = false;
        lane.invalidReason = "region_index_out_of_range";
        return lane;
      }
      const MtCoarseRegion& region = coarsePlan.regions[regionIndex];
      lane.taskCount += region.taskCount;
      lane.memberNodeCost += region.memberNodeCost;
      lane.activeWordSpanSum += region.activeWordSpan;
      for (int cppId = region.beginCppId; cppId < region.endCppId; cppId ++) {
        lane.cppToLocal[cppId] = static_cast<int>(lane.cppIds.size());
        lane.cppIds.push_back(cppId);
      }
    }
    int previousRegionEnd = -1;
    for (int regionIndex : regionIndices) {
      const MtCoarseRegion& region = coarsePlan.regions[regionIndex];
      if (lane.envelopeBeginCppId < 0 || region.beginCppId < lane.envelopeBeginCppId) lane.envelopeBeginCppId = region.beginCppId;
      if (region.endCppId > lane.envelopeEndCppId) lane.envelopeEndCppId = region.endCppId;
      if (previousRegionEnd >= 0 && previousRegionEnd < region.beginCppId) lane.gapIntervals.push_back(std::make_pair(previousRegionEnd, region.beginCppId));
      previousRegionEnd = region.endCppId;
    }
    auto readyBatchIsGapCpp = [&](int cppId) {
      return cppId >= lane.envelopeBeginCppId && cppId < lane.envelopeEndCppId && lane.cppToLocal.find(cppId) == lane.cppToLocal.end();
    };
    for (int cppId = lane.envelopeBeginCppId; cppId < lane.envelopeEndCppId; cppId ++) {
      if (!readyBatchIsGapCpp(cppId)) continue;
      auto taskIter = mtTasks.find(cppId);
      if (taskIter == mtTasks.end()) continue;
      lane.gapTaskCount ++;
      if (taskIter->second.taskKind == "pure_compute") lane.gapPureTaskCount ++;
      else lane.gapSerialTaskCount ++;
      for (const std::string& reason : taskIter->second.serialReasons) {
        if (reason == "state_update") lane.gapStateUpdateTaskCount ++;
      }
      if (hasWorker0OnlyReason(taskIter->second.serialReasons)) lane.gapWorker0OnlyTaskCount ++;
    }
    auto countReadyBatchGapEdges = [&](int fromCppId, SuperNode* super) {
      if (super == nullptr) return;
      bool fromLane = lane.cppToLocal.find(fromCppId) != lane.cppToLocal.end();
      bool fromGap = readyBatchIsGapCpp(fromCppId);
      if (!fromLane && !fromGap) return;
      auto countTarget = [&](int toCppId, bool activeEdge) {
        bool toLane = lane.cppToLocal.find(toCppId) != lane.cppToLocal.end();
        bool toGap = readyBatchIsGapCpp(toCppId);
        if (fromLane && toGap) {
          if (activeEdge) lane.gapActiveLaneToGapEdges ++;
          else lane.gapOrderLaneToGapEdges ++;
        } else if (fromGap && toLane) {
          if (activeEdge) lane.gapActiveGapToLaneEdges ++;
          else lane.gapOrderGapToLaneEdges ++;
        }
      };
      for (SuperNode* next : super->next) if (next && next->cppId >= 0) countTarget(next->cppId, false);
      for (SuperNode* next : super->depNext) if (next && next->cppId >= 0) countTarget(next->cppId, false);
      for (Node* member : super->member) {
        if (!member) continue;
        for (int activeId : member->nextNeedActivate) if (activeId >= 0) countTarget(activeId, true);
      }
    };
    for (int cppId : lane.cppIds) {
      auto superIter = cppId2Super.find(cppId);
      if (superIter != cppId2Super.end()) countReadyBatchGapEdges(cppId, superIter->second);
    }
    for (int cppId = lane.envelopeBeginCppId; cppId < lane.envelopeEndCppId; cppId ++) {
      if (!readyBatchIsGapCpp(cppId)) continue;
      auto superIter = cppId2Super.find(cppId);
      if (superIter != cppId2Super.end()) countReadyBatchGapEdges(cppId, superIter->second);
    }
    lane.localSucc.assign(lane.cppIds.size(), std::set<int>());
    for (int cppId : lane.cppIds) {
      auto superIter = cppId2Super.find(cppId);
      if (superIter == cppId2Super.end() || superIter->second == nullptr) continue;
      SuperNode* super = superIter->second;
      for (SuperNode* next : super->next) if (next && next->cppId >= 0) addReadyBatchEdge(lane.localSucc, lane.cppToLocal, cppId, next->cppId);
      for (SuperNode* next : super->depNext) if (next && next->cppId >= 0) addReadyBatchEdge(lane.localSucc, lane.cppToLocal, cppId, next->cppId);
      for (Node* member : super->member) {
        if (!member) continue;
        for (int activeId : member->nextNeedActivate) if (activeId >= 0) addReadyBatchEdge(lane.localSucc, lane.cppToLocal, cppId, activeId);
      }
    }

    computeReadyBatchScc(lane.cppIds, lane.localSucc, lane.sccOfLocal,
                         lane.sccCost, lane.sccSucc, lane.sccPred, lane.topo,
                         lane.largestScc, lane.sccEdgeCount, lane.valid,
                         lane.invalidReason);
    lane.sccSerialTaskCount.assign(lane.sccCost.size(), 0);
    for (size_t local = 0; local < lane.cppIds.size(); local ++) {
      int scc = lane.sccOfLocal[local];
      if (scc < 0 || scc >= static_cast<int>(lane.sccCost.size())) continue;
      auto taskIter = mtTasks.find(lane.cppIds[local]);
      bool isPure = taskIter != mtTasks.end() && taskIter->second.taskKind == "pure_compute";
      if (!isPure) lane.sccSerialTaskCount[scc] ++;
    }

    for (int cppId = lane.envelopeBeginCppId; cppId < lane.envelopeEndCppId; cppId ++) {
      if (cppId2Super.find(cppId) == cppId2Super.end()) continue;
      lane.envelopeCppToLocal[cppId] = static_cast<int>(lane.envelopeCppIds.size());
      lane.envelopeCppIds.push_back(cppId);
      lane.envelopeTaskCount ++;
      if (lane.cppToLocal.find(cppId) != lane.cppToLocal.end()) lane.envelopeSelectedTaskCount ++;
      else lane.envelopeGapTaskCount ++;
      auto taskIter = mtTasks.find(cppId);
      if (taskIter != mtTasks.end() && taskIter->second.taskKind == "pure_compute") {
        lane.envelopePureTaskCount ++;
      } else {
        lane.envelopeSerialTaskCount ++;
      }
      if (taskIter != mtTasks.end()) {
        for (const std::string& reason : taskIter->second.serialReasons) {
          if (reason == "state_update") lane.envelopeStateUpdateTaskCount ++;
        }
        if (hasWorker0OnlyReason(taskIter->second.serialReasons)) lane.envelopeWorker0OnlyTaskCount ++;
      }
    }
    lane.envelopeLocalSucc.assign(lane.envelopeCppIds.size(), std::set<int>());
    auto addReadyBatchEnvelopeEdge = [&](int fromCppId, int toCppId) {
      auto fromIter = lane.envelopeCppToLocal.find(fromCppId);
      if (fromIter == lane.envelopeCppToLocal.end()) return;
      auto toIter = lane.envelopeCppToLocal.find(toCppId);
      if (toIter == lane.envelopeCppToLocal.end()) return;
      if (fromIter->second == toIter->second) return;
      lane.envelopeLocalSucc[fromIter->second].insert(toIter->second);
    };
    for (int cppId : lane.envelopeCppIds) {
      auto superIter = cppId2Super.find(cppId);
      if (superIter == cppId2Super.end() || superIter->second == nullptr) continue;
      SuperNode* super = superIter->second;
      for (SuperNode* next : super->next) if (next && next->cppId >= 0) addReadyBatchEnvelopeEdge(cppId, next->cppId);
      for (SuperNode* next : super->depNext) if (next && next->cppId >= 0) addReadyBatchEnvelopeEdge(cppId, next->cppId);
      for (Node* member : super->member) {
        if (!member) continue;
        for (int activeId : member->nextNeedActivate) if (activeId >= 0) addReadyBatchEnvelopeEdge(cppId, activeId);
      }
    }
    computeReadyBatchScc(lane.envelopeCppIds, lane.envelopeLocalSucc,
                         lane.envelopeSccOfLocal, lane.envelopeSccCost,
                         lane.envelopeSccSucc, lane.envelopeSccPred,
                         lane.envelopeTopo, lane.envelopeLargestScc,
                         lane.envelopeSccEdgeCount, lane.valid,
                         lane.invalidReason);
    lane.envelopeSccSerialTaskCount.assign(lane.envelopeSccCost.size(), 0);
    lane.envelopeSccSelectedTaskCount.assign(lane.envelopeSccCost.size(), 0);
    lane.envelopeSccGapTaskCount.assign(lane.envelopeSccCost.size(), 0);
    lane.envelopeSccSerialReasonTaskCount.assign(lane.envelopeSccCost.size(), std::map<std::string, int>());
    for (size_t local = 0; local < lane.envelopeCppIds.size(); local ++) {
      int cppId = lane.envelopeCppIds[local];
      int scc = lane.envelopeSccOfLocal[local];
      if (scc < 0 || scc >= static_cast<int>(lane.envelopeSccCost.size())) continue;
      auto taskIter = mtTasks.find(cppId);
      const MtTaskInfo* taskInfo = taskIter == mtTasks.end() ? nullptr : &taskIter->second;
      accumulateReadyBatchStateUpdateStatic(lane, cppId);
      bool isPure = taskInfo != nullptr && taskInfo->taskKind == "pure_compute";
      if (!isPure) {
        lane.envelopeSccSerialTaskCount[scc] ++;
        std::string reason = readyBatchPrimarySerialReason(taskInfo);
        lane.envelopeSccSerialReasonTaskCount[scc][reason] ++;
        lane.envelopeSerialReasonTaskCount[reason] ++;
      }
      if (lane.cppToLocal.find(cppId) != lane.cppToLocal.end()) lane.envelopeSccSelectedTaskCount[scc] ++;
      else lane.envelopeSccGapTaskCount[scc] ++;
    }
    return lane;
  };

  auto scheduleReadyBatchGraphCap = [&](const std::vector<int>& sccCost,
                                        const std::vector<std::set<int>>& sccSucc,
                                        const std::vector<std::set<int>>& sccPred,
                                        const std::vector<int>& topo,
                                        bool valid,
                                        const std::vector<char>& active,
                                        int cap) {
    ReadyBatchScheduleResult result;
    if (!valid || active.empty()) return result;
    int sccCount = static_cast<int>(sccCost.size());
    std::vector<int> criticalPath(sccCount, 0);
    for (auto iter = topo.rbegin(); iter != topo.rend(); ++ iter) {
      int scc = *iter;
      if (!active[scc]) continue;
      int bestSucc = 0;
      for (int succ : sccSucc[scc]) if (active[succ]) bestSucc = std::max(bestSucc, criticalPath[succ]);
      criticalPath[scc] = sccCost[scc] + bestSucc;
    }
    std::vector<int> indegree(sccCount, 0), readyTime(sccCount, 0);
    std::vector<std::tuple<int, int, int>> readyHeap;
    for (int scc = 0; scc < sccCount; scc ++) {
      if (!active[scc]) continue;
      result.serial += sccCost[scc];
      for (int pred : sccPred[scc]) if (active[pred]) indegree[scc] ++;
      if (indegree[scc] == 0) readyHeap.push_back(std::make_tuple(criticalPath[scc], sccCost[scc], scc));
    }
    std::make_heap(readyHeap.begin(), readyHeap.end());
    int workerReady[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    while (!readyHeap.empty()) {
      int worker = 0;
      for (int w = 1; w < 8; w ++) if (workerReady[w] < workerReady[worker]) worker = w;
      std::vector<int> chunk;
      std::vector<std::tuple<int, int, int>> stash;
      int chunkCost = 0;
      int chunkReadyTime = 0;
      while (!readyHeap.empty()) {
        std::pop_heap(readyHeap.begin(), readyHeap.end());
        std::tuple<int, int, int> item = readyHeap.back();
        readyHeap.pop_back();
        int scc = std::get<2>(item);
        int cost = sccCost[scc];
        if (!chunk.empty() && chunkCost + cost > cap) {
          stash.push_back(item);
          break;
        }
        chunk.push_back(scc);
        chunkCost += cost;
        chunkReadyTime = std::max(chunkReadyTime, readyTime[scc]);
        if (chunkCost >= cap) break;
      }
      for (const auto& item : stash) {
        readyHeap.push_back(item);
        std::push_heap(readyHeap.begin(), readyHeap.end());
      }
      if (chunk.empty()) break;
      int start = std::max(workerReady[worker], chunkReadyTime);
      int finish = start + chunkCost;
      workerReady[worker] = finish;
      result.dispatches ++;
      for (int scc : chunk) {
        for (int succ : sccSucc[scc]) {
          if (!active[succ]) continue;
          readyTime[succ] = std::max(readyTime[succ], finish);
          indegree[succ] --;
          if (indegree[succ] == 0) {
            readyHeap.push_back(std::make_tuple(criticalPath[succ], sccCost[succ], succ));
            std::push_heap(readyHeap.begin(), readyHeap.end());
          }
        }
      }
    }
    for (int w = 0; w < 8; w ++) result.makespan = std::max(result.makespan, workerReady[w]);
    return result;
  };
  auto scheduleReadyBatchCap = [&](const ReadyBatchLaneGraph& lane, const std::vector<char>& active, int cap) {
    return scheduleReadyBatchGraphCap(lane.sccCost, lane.sccSucc, lane.sccPred, lane.topo, lane.valid, active, cap);
  };
  auto scheduleReadyBatchEnvelopeCap = [&](const ReadyBatchLaneGraph& lane, const std::vector<char>& active, int cap) {
    return scheduleReadyBatchGraphCap(lane.envelopeSccCost, lane.envelopeSccSucc, lane.envelopeSccPred,
                                      lane.envelopeTopo, lane.valid, active, cap);
  };
  std::vector<int> readyBatchEmptyCounts;
  auto scheduleReadyBatchLocalEvalGraph = [&](bool valid,
                                             const std::vector<int>& sccCost,
                                             const std::vector<std::set<int>>& sccSucc,
                                             const std::vector<std::set<int>>& sccPred,
                                             const std::vector<int>& topo,
                                             const std::vector<int>& sccSerialTaskCount,
                                             const std::vector<int>& sccSelectedTaskCount,
                                             const std::vector<int>& sccGapTaskCount,
                                             const std::vector<char>& active,
                                             int workerCap, int chunkCap, bool serialOnWorker0) {
    ReadyBatchLocalEvalResult result;
    if (!valid || active.empty()) return result;
    int sccCount = static_cast<int>(sccCost.size());
    if (sccCount <= 0) return result;
    int workers = std::max(1, std::min(workerCap, 8));
    int maxChunkCost = std::max(1, chunkCap);
    std::vector<int> criticalPath(sccCount, 0);
    for (auto iter = topo.rbegin(); iter != topo.rend(); ++ iter) {
      int scc = *iter;
      if (scc < 0 || scc >= sccCount || !active[scc]) continue;
      int bestSucc = 0;
      for (int succ : sccSucc[scc]) if (succ >= 0 && succ < sccCount && active[succ]) bestSucc = std::max(bestSucc, criticalPath[succ]);
      criticalPath[scc] = sccCost[scc] + bestSucc;
    }
    std::vector<int> indegree(sccCount, 0), readyTime(sccCount, 0), workerReady(workers, 0);
    std::vector<std::tuple<int, int, int>> readyHeap;
    for (int scc = 0; scc < sccCount; scc ++) {
      if (!active[scc]) continue;
      result.serial += sccCost[scc];
      result.activeSccs ++;
      result.activeTasks += sccCost[scc];
      if (!sccSelectedTaskCount.empty() && scc < static_cast<int>(sccSelectedTaskCount.size())) result.selectedTasks += sccSelectedTaskCount[scc];
      else result.selectedTasks += sccCost[scc];
      if (scc < static_cast<int>(sccGapTaskCount.size())) result.gapTasks += sccGapTaskCount[scc];
      if (scc < static_cast<int>(sccSerialTaskCount.size())) result.serialTasks += sccSerialTaskCount[scc];
      for (int pred : sccPred[scc]) if (pred >= 0 && pred < sccCount && active[pred]) indegree[scc] ++;
      if (indegree[scc] == 0) readyHeap.push_back(std::make_tuple(criticalPath[scc], sccCost[scc], scc));
    }
    std::make_heap(readyHeap.begin(), readyHeap.end());
    while (!readyHeap.empty()) {
      std::vector<int> chunk;
      std::vector<std::tuple<int, int, int>> stash;
      int chunkCost = 0;
      int chunkReadyTime = 0;
      int chunkSerialTasks = 0;
      bool chunkSerial = false;
      while (!readyHeap.empty()) {
        std::pop_heap(readyHeap.begin(), readyHeap.end());
        std::tuple<int, int, int> item = readyHeap.back();
        readyHeap.pop_back();
        int scc = std::get<2>(item);
        int cost = sccCost[scc];
        int itemSerialTasks = scc < static_cast<int>(sccSerialTaskCount.size()) ? sccSerialTaskCount[scc] : 0;
        bool itemSerial = itemSerialTasks > 0;
        if (!chunk.empty()) {
          if (serialOnWorker0 && itemSerial != chunkSerial) {
            stash.push_back(item);
            break;
          }
          if (chunkCost + cost > maxChunkCost) {
            stash.push_back(item);
            break;
          }
        } else {
          chunkSerial = itemSerial;
        }
        chunk.push_back(scc);
        chunkCost += cost;
        chunkReadyTime = std::max(chunkReadyTime, readyTime[scc]);
        chunkSerialTasks += itemSerialTasks;
        if (chunkCost >= maxChunkCost) break;
      }
      for (const auto& item : stash) {
        readyHeap.push_back(item);
        std::push_heap(readyHeap.begin(), readyHeap.end());
      }
      if (chunk.empty()) break;
      int worker = 0;
      if (!(serialOnWorker0 && chunkSerial)) {
        for (int w = 1; w < workers; w ++) if (workerReady[w] < workerReady[worker]) worker = w;
      }
      int start = std::max(workerReady[worker], chunkReadyTime);
      int finish = start + chunkCost;
      workerReady[worker] = finish;
      result.dispatches ++;
      if (chunkSerialTasks > 0) result.serialTaskDispatches ++;
      for (int scc : chunk) {
        for (int succ : sccSucc[scc]) {
          if (succ < 0 || succ >= sccCount || !active[succ]) continue;
          readyTime[succ] = std::max(readyTime[succ], finish);
          indegree[succ] --;
          if (indegree[succ] == 0) {
            readyHeap.push_back(std::make_tuple(criticalPath[succ], sccCost[succ], succ));
            std::push_heap(readyHeap.begin(), readyHeap.end());
          }
        }
      }
    }
    for (int w = 0; w < workers; w ++) result.makespan = std::max(result.makespan, workerReady[w]);
    return result;
  };
  auto scheduleReadyBatchEnvelopeLocalEval = [&](const ReadyBatchLaneGraph& lane, const std::vector<char>& active,
                                                int workerCap, int chunkCap, bool serialOnWorker0) {
    return scheduleReadyBatchLocalEvalGraph(lane.valid, lane.envelopeSccCost, lane.envelopeSccSucc, lane.envelopeSccPred,
                                            lane.envelopeTopo, lane.envelopeSccSerialTaskCount,
                                            lane.envelopeSccSelectedTaskCount, lane.envelopeSccGapTaskCount,
                                            active, workerCap, chunkCap, serialOnWorker0);
  };
  auto scheduleReadyBatchSelectedOnlyLocalEval = [&](const ReadyBatchLaneGraph& lane, const std::vector<char>& active,
                                                    int workerCap, int chunkCap) {
    return scheduleReadyBatchLocalEvalGraph(lane.valid, lane.sccCost, lane.sccSucc, lane.sccPred, lane.topo,
                                            lane.sccSerialTaskCount, readyBatchEmptyCounts, readyBatchEmptyCounts,
                                            active, workerCap, chunkCap, false);
  };
  auto accumulateReadyBatchLocalEvalStats = [&](ReadyBatchLocalEvalStats& evalStats,
                                               const ReadyBatchLocalEvalResult& evalResult,
                                               int activeWordVolume) {
    evalStats.traceCycles ++;
    evalStats.traceSerial += static_cast<uint64_t>(evalResult.serial);
    evalStats.traceMakespan += static_cast<uint64_t>(evalResult.makespan);
    evalStats.traceDispatches += static_cast<uint64_t>(evalResult.dispatches);
    evalStats.traceActiveSccs += static_cast<uint64_t>(evalResult.activeSccs);
    evalStats.traceActiveTasks += static_cast<uint64_t>(evalResult.activeTasks);
    evalStats.traceSelectedTasks += static_cast<uint64_t>(evalResult.selectedTasks);
    evalStats.traceGapTasks += static_cast<uint64_t>(evalResult.gapTasks);
    evalStats.traceSerialTasks += static_cast<uint64_t>(evalResult.serialTasks);
    evalStats.traceSerialTaskDispatches += static_cast<uint64_t>(evalResult.serialTaskDispatches);
    evalStats.traceActiveWordOrVolume += static_cast<uint64_t>(activeWordVolume);
    evalStats.traceMakespans.push_back(evalResult.makespan);
    evalStats.traceDispatchCounts.push_back(evalResult.dispatches);
    evalStats.traceSerialTaskDispatchCounts.push_back(evalResult.serialTaskDispatches);
    evalStats.traceActiveWordOrVolumes.push_back(activeWordVolume);
  };

  std::vector<ReadyBatchLaneGraph> lanes;
  lanes.push_back(buildReadyBatchLane("hot_576_583", std::vector<int>{576, 577, 578, 579, 580, 581, 582, 583}));
  lanes.push_back(buildReadyBatchLane("early_39_44", std::vector<int>{39, 40, 42, 43, 44}));
  lanes.push_back(buildReadyBatchLane("mid_557_family", std::vector<int>{531, 535, 553, 554, 555, 557, 558, 559, 560, 561, 562, 570, 571}));
  lanes.push_back(buildReadyBatchLane("region_39", std::vector<int>{39}));
  lanes.push_back(buildReadyBatchLane("region_40", std::vector<int>{40}));
  lanes.push_back(buildReadyBatchLane("region_42", std::vector<int>{42}));
  lanes.push_back(buildReadyBatchLane("region_44", std::vector<int>{44}));
  lanes.push_back(buildReadyBatchLane("region_576", std::vector<int>{576}));
  lanes.push_back(buildReadyBatchLane("region_577", std::vector<int>{577}));
  lanes.push_back(buildReadyBatchLane("region_582", std::vector<int>{582}));
  lanes.push_back(buildReadyBatchLane("region_583", std::vector<int>{583}));
  lanes.push_back(buildReadyBatchLane("region_557", std::vector<int>{557}));
  for (ReadyBatchLaneGraph& lane : lanes) {
    for (int cap : {1, 2, 4}) {
      ReadyBatchCapStats stats;
      stats.cap = cap;
      if (lane.valid) {
        std::vector<char> active(lane.sccCost.size(), 1);
        ReadyBatchScheduleResult staticResult = scheduleReadyBatchCap(lane, active, cap);
        stats.staticSerial = staticResult.serial;
        stats.staticMakespan = staticResult.makespan;
        stats.staticDispatches = staticResult.dispatches;
      }
      lane.caps.push_back(stats);

      ReadyBatchCapStats envelopeStats;
      envelopeStats.cap = cap;
      if (lane.valid) {
        std::vector<char> active(lane.envelopeSccCost.size(), 1);
        ReadyBatchScheduleResult staticResult = scheduleReadyBatchEnvelopeCap(lane, active, cap);
        envelopeStats.staticSerial = staticResult.serial;
        envelopeStats.staticMakespan = staticResult.makespan;
        envelopeStats.staticDispatches = staticResult.dispatches;
      }
      lane.envelopeCaps.push_back(envelopeStats);
    }
  }

  bool traceEnabled = false;
  bool envelopeLocalEvalDiagnosticsEnabled = mtUseEnvelopeLocalEvalDiagnostics();
  bool envelopeLocalEvalEnabled = envelopeLocalEvalDiagnosticsEnabled;
  int traceCycles = 0;
  const char* tracePath = std::getenv("GSIM_MT_READY_BATCH_TRACE");
  if (tracePath != nullptr && tracePath[0] != '\0') {
    std::ifstream trace(tracePath);
    if (trace.good()) {
      traceEnabled = true;
      std::map<int, std::pair<int, int>> cppToLaneScc;
      std::map<int, std::vector<std::pair<int, int>>> cppToEnvelopeLaneScc;
      for (size_t laneIdx = 0; laneIdx < lanes.size(); laneIdx ++) {
        const ReadyBatchLaneGraph& lane = lanes[laneIdx];
        if (!lane.valid) continue;
        for (size_t local = 0; local < lane.cppIds.size(); local ++) cppToLaneScc[lane.cppIds[local]] = std::make_pair(static_cast<int>(laneIdx), lane.sccOfLocal[local]);
        for (size_t local = 0; local < lane.envelopeCppIds.size(); local ++) {
          cppToEnvelopeLaneScc[lane.envelopeCppIds[local]].push_back(std::make_pair(static_cast<int>(laneIdx), lane.envelopeSccOfLocal[local]));
        }
      }
      std::string line;
      while (std::getline(trace, line)) {
        size_t pos = line.find(" tasks=");
        if (line.find("[mt-dyn-trace]") == std::string::npos || pos == std::string::npos) continue;
        traceCycles ++;
        std::vector<std::vector<char>> active(lanes.size());
        std::vector<std::vector<char>> envelopeActive(lanes.size());
        std::vector<std::set<int>> envelopeActiveWords(lanes.size());
        std::vector<std::set<int>> activeWords(lanes.size());
        for (size_t laneIdx = 0; laneIdx < lanes.size(); laneIdx ++) {
          active[laneIdx].assign(lanes[laneIdx].sccCost.size(), 0);
          envelopeActive[laneIdx].assign(lanes[laneIdx].envelopeSccCost.size(), 0);
        }
        std::stringstream ss(line.substr(pos + 7));
        std::string item;
        while (std::getline(ss, item, ',')) {
          int cppId = std::atoi(item.c_str());
          auto iter = cppToLaneScc.find(cppId);
          if (iter != cppToLaneScc.end()) {
            int laneIdx = iter->second.first;
            int scc = iter->second.second;
            if (laneIdx >= 0 && laneIdx < static_cast<int>(active.size()) && scc >= 0 && scc < static_cast<int>(active[laneIdx].size())) {
              active[laneIdx][scc] = 1;
              activeWords[laneIdx].insert(cppId / ACTIVE_WIDTH);
            }
          }
          auto envelopeIter = cppToEnvelopeLaneScc.find(cppId);
          if (envelopeIter != cppToEnvelopeLaneScc.end()) {
            int activeWord = cppId / ACTIVE_WIDTH;
            for (const auto& laneScc : envelopeIter->second) {
              int laneIdx = laneScc.first;
              int scc = laneScc.second;
              if (laneIdx >= 0 && laneIdx < static_cast<int>(envelopeActive.size()) && scc >= 0 && scc < static_cast<int>(envelopeActive[laneIdx].size())) {
                envelopeActive[laneIdx][scc] = 1;
                envelopeActiveWords[laneIdx].insert(activeWord);
                accumulateReadyBatchStateUpdateTrace(lanes[laneIdx], cppId);
              }
            }
          }
        }
        for (size_t laneIdx = 0; laneIdx < lanes.size(); laneIdx ++) {
          ReadyBatchLaneGraph& lane = lanes[laneIdx];
          if (!lane.valid) continue;
          bool anyActive = false;
          for (char value : active[laneIdx]) if (value) { anyActive = true; break; }
          if (anyActive) {
            for (ReadyBatchCapStats& stats : lane.caps) {
              ReadyBatchScheduleResult result = scheduleReadyBatchCap(lane, active[laneIdx], stats.cap);
              stats.traceCycles ++;
              stats.traceSerial += static_cast<uint64_t>(result.serial);
              stats.traceMakespan += static_cast<uint64_t>(result.makespan);
              stats.traceDispatches += static_cast<uint64_t>(result.dispatches);
              stats.traceMakespans.push_back(result.makespan);
              stats.traceDispatchCounts.push_back(result.dispatches);
            }
          }
          bool anyEnvelopeActive = false;
          for (char value : envelopeActive[laneIdx]) if (value) { anyEnvelopeActive = true; break; }
          if (anyEnvelopeActive) {
            for (ReadyBatchCapStats& stats : lane.envelopeCaps) {
              ReadyBatchScheduleResult result = scheduleReadyBatchEnvelopeCap(lane, envelopeActive[laneIdx], stats.cap);
              stats.traceCycles ++;
              stats.traceSerial += static_cast<uint64_t>(result.serial);
              stats.traceMakespan += static_cast<uint64_t>(result.makespan);
              stats.traceDispatches += static_cast<uint64_t>(result.dispatches);
              stats.traceMakespans.push_back(result.makespan);
              stats.traceDispatchCounts.push_back(result.dispatches);
            }
          }
          if (envelopeLocalEvalEnabled && anyEnvelopeActive) {
            ReadyBatchLocalEvalResult evalResult = scheduleReadyBatchEnvelopeLocalEval(lane, envelopeActive[laneIdx],
                                                                                      lane.envelopeLocalEval.workerCap,
                                                                                      lane.envelopeLocalEval.chunkCap,
                                                                                      false);
            int activeWordVolume = laneIdx < envelopeActiveWords.size() ? static_cast<int>(envelopeActiveWords[laneIdx].size()) : 0;
            accumulateReadyBatchLocalEvalStats(lane.envelopeLocalEval, evalResult, activeWordVolume);
            if (envelopeLocalEvalDiagnosticsEnabled) {
              ReadyBatchLocalEvalResult selectedOnlyResult = scheduleReadyBatchSelectedOnlyLocalEval(lane, active[laneIdx],
                                                                                                    lane.envelopeLocalEvalSelectedOnly.workerCap,
                                                                                                    lane.envelopeLocalEvalSelectedOnly.chunkCap);
              int selectedOnlyActiveWordVolume = laneIdx < activeWords.size() ? static_cast<int>(activeWords[laneIdx].size()) : 0;
              accumulateReadyBatchLocalEvalStats(lane.envelopeLocalEvalSelectedOnly, selectedOnlyResult, selectedOnlyActiveWordVolume);
              ReadyBatchLocalEvalResult serialWorker0Result = scheduleReadyBatchEnvelopeLocalEval(lane, envelopeActive[laneIdx],
                                                                                                 lane.envelopeLocalEvalSerialWorker0.workerCap,
                                                                                                 lane.envelopeLocalEvalSerialWorker0.chunkCap,
                                                                                                 true);
              accumulateReadyBatchLocalEvalStats(lane.envelopeLocalEvalSerialWorker0, serialWorker0Result, activeWordVolume);
              for (int activeWord : envelopeActiveWords[laneIdx]) lane.envelopeTraceActiveWordHits[activeWord] ++;
              for (size_t scc = 0; scc < envelopeActive[laneIdx].size(); scc ++) {
                if (!envelopeActive[laneIdx][scc]) continue;
                if (scc >= lane.envelopeSccSerialReasonTaskCount.size()) continue;
                for (const auto& kv : lane.envelopeSccSerialReasonTaskCount[scc]) {
                  lane.envelopeTraceSerialReasonTaskCount[kv.first] += static_cast<uint64_t>(kv.second);
                }
              }
            }
          }
        }
      }
    }
  }

  fprintf(fp, "{\n");
  fprintf(fp, "  \"format\": \"gsim.mt-ready-batch-lanes.v1\",\n");
  fprintf(fp, "  \"candidate_kind\": \"report_only_lane_scc_ready_batch\",\n");
  fprintf(fp, "  \"task_count\": %d,\n", superId);
  fprintf(fp, "  \"trace\": {\"enabled\": %s, \"path\": ", traceEnabled ? "true" : "false");
  if (tracePath != nullptr && tracePath[0] != '\0') fprintf(fp, "\"%s\"", jsonEscape(tracePath).c_str());
  else fprintf(fp, "null");
  fprintf(fp, ", \"cycles\": %d},\n", traceCycles);
  fprintf(fp, "  \"lanes\": [\n");
  for (size_t laneIdx = 0; laneIdx < lanes.size(); laneIdx ++) {
    const ReadyBatchLaneGraph& lane = lanes[laneIdx];
    fprintf(fp, "    {\n");
    fprintf(fp, "      \"name\": \"%s\",\n", jsonEscape(lane.name).c_str());
    fprintf(fp, "      \"valid\": %s,\n", lane.valid ? "true" : "false");
    if (lane.valid) fprintf(fp, "      \"invalid_reason\": null,\n");
    else fprintf(fp, "      \"invalid_reason\": \"%s\",\n", jsonEscape(lane.invalidReason).c_str());
    fprintf(fp, "      \"region_indices\": ");
    dumpJsonIntArray(fp, lane.regionIndices);
    fprintf(fp, ",\n");
    fprintf(fp, "      \"task_count\": %d,\n", lane.taskCount);
    fprintf(fp, "      \"member_node_cost\": %d,\n", lane.memberNodeCost);
    fprintf(fp, "      \"active_word_span_sum\": %d,\n", lane.activeWordSpanSum);
    fprintf(fp, "      \"envelope_begin_cpp_id\": %d,\n", lane.envelopeBeginCppId);
    fprintf(fp, "      \"envelope_end_cpp_id\": %d,\n", lane.envelopeEndCppId);
    fprintf(fp, "      \"gap_intervals\": [\n");
    for (size_t gapIdx = 0; gapIdx < lane.gapIntervals.size(); gapIdx ++) {
      fprintf(fp, "        {\"begin_cpp_id\": %d, \"end_cpp_id\": %d, \"task_count\": %d}%s\n",
              lane.gapIntervals[gapIdx].first, lane.gapIntervals[gapIdx].second,
              lane.gapIntervals[gapIdx].second - lane.gapIntervals[gapIdx].first,
              gapIdx + 1 == lane.gapIntervals.size() ? "" : ",");
    }
    fprintf(fp, "      ],\n");
    fprintf(fp, "      \"gap_task_count\": %d,\n", lane.gapTaskCount);
    fprintf(fp, "      \"gap_pure_task_count\": %d,\n", lane.gapPureTaskCount);
    fprintf(fp, "      \"gap_serial_task_count\": %d,\n", lane.gapSerialTaskCount);
    fprintf(fp, "      \"gap_state_update_task_count\": %d,\n", lane.gapStateUpdateTaskCount);
    fprintf(fp, "      \"gap_worker0_only_task_count\": %d,\n", lane.gapWorker0OnlyTaskCount);
    fprintf(fp, "      \"gap_order_lane_to_gap_edges\": %d,\n", lane.gapOrderLaneToGapEdges);
    fprintf(fp, "      \"gap_order_gap_to_lane_edges\": %d,\n", lane.gapOrderGapToLaneEdges);
    fprintf(fp, "      \"gap_active_lane_to_gap_edges\": %d,\n", lane.gapActiveLaneToGapEdges);
    fprintf(fp, "      \"gap_active_gap_to_lane_edges\": %d,\n", lane.gapActiveGapToLaneEdges);
    fprintf(fp, "      \"scc_count\": %zu,\n", lane.sccCost.size());
    fprintf(fp, "      \"largest_scc\": %d,\n", lane.largestScc);
    fprintf(fp, "      \"scc_edge_count\": %d,\n", lane.sccEdgeCount);
    fprintf(fp, "      \"dense_counter_bytes_u8_t8\": %zu,\n", lane.sccCost.size() * static_cast<size_t>(8));
    fprintf(fp, "      \"envelope_task_count\": %d,\n", lane.envelopeTaskCount);
    fprintf(fp, "      \"envelope_selected_task_count\": %d,\n", lane.envelopeSelectedTaskCount);
    fprintf(fp, "      \"envelope_gap_task_count\": %d,\n", lane.envelopeGapTaskCount);
    fprintf(fp, "      \"envelope_pure_task_count\": %d,\n", lane.envelopePureTaskCount);
    fprintf(fp, "      \"envelope_serial_task_count\": %d,\n", lane.envelopeSerialTaskCount);
    fprintf(fp, "      \"envelope_state_update_task_count\": %d,\n", lane.envelopeStateUpdateTaskCount);
      fprintf(fp, "      \"state_update_group\": {\n");
      fprintf(fp, "        \"blocked_task_count\": %d,\n", lane.envelopeStateUpdateBlockedTaskCount);
      fprintf(fp, "        \"local_safe_only_task_count\": %d,\n", lane.envelopeStateUpdateLocalSafeOnlyTaskCount);
      fprintf(fp, "        \"runtime_safe_task_count\": %d,\n", lane.envelopeStateUpdateRuntimeSafeTaskCount);
      fprintf(fp, "        \"target_writer_conflict_task_counts\": ");
      dumpReadyBatchStringIntCountArray(fp, lane.envelopeStateUpdateTargetWriterConflictTaskCount);
      fprintf(fp, ",\n");
      fprintf(fp, "        \"runtime_block_reason_task_counts\": ");
      dumpReadyBatchStringIntCountArray(fp, lane.envelopeStateUpdateRuntimeBlockReasonTaskCount);
      fprintf(fp, "\n");
      fprintf(fp, "      },\n");
    fprintf(fp, "      \"envelope_worker0_only_task_count\": %d,\n", lane.envelopeWorker0OnlyTaskCount);
    fprintf(fp, "      \"envelope_scc_count\": %zu,\n", lane.envelopeSccCost.size());
    fprintf(fp, "      \"envelope_largest_scc\": %d,\n", lane.envelopeLargestScc);
    fprintf(fp, "      \"envelope_scc_edge_count\": %d,\n", lane.envelopeSccEdgeCount);
    fprintf(fp, "      \"envelope_dense_counter_bytes_u8_t8\": %zu,\n", lane.envelopeSccCost.size() * static_cast<size_t>(8));
    fprintf(fp, "      \"scc_costs\": ");
    dumpJsonIntArray(fp, lane.sccCost);
    fprintf(fp, ",\n");
    fprintf(fp, "      \"topo_order\": ");
    dumpJsonIntArray(fp, lane.topo);
    fprintf(fp, ",\n");
    fprintf(fp, "      \"scc_edges\": [\n");
    size_t readyBatchEdgeWritten = 0;
    for (size_t fromScc = 0; fromScc < lane.sccSucc.size(); fromScc ++) {
      for (int toScc : lane.sccSucc[fromScc]) {
        fprintf(fp, "        {\"from\": %zu, \"to\": %d}%s\n",
                fromScc, toScc, readyBatchEdgeWritten + 1 == static_cast<size_t>(lane.sccEdgeCount) ? "" : ",");
        readyBatchEdgeWritten ++;
      }
    }
    fprintf(fp, "      ],\n");
    fprintf(fp, "      \"cpp_to_scc\": [\n");
    for (size_t local = 0; local < lane.cppIds.size(); local ++) {
      fprintf(fp, "        {\"cpp_id\": %d, \"scc\": %d}%s\n",
              lane.cppIds[local], lane.sccOfLocal[local], local + 1 == lane.cppIds.size() ? "" : ",");
    }
    fprintf(fp, "      ],\n");
    fprintf(fp, "      \"envelope_scc_costs\": ");
    dumpJsonIntArray(fp, lane.envelopeSccCost);
    fprintf(fp, ",\n");
    fprintf(fp, "      \"envelope_topo_order\": ");
    dumpJsonIntArray(fp, lane.envelopeTopo);
    fprintf(fp, ",\n");
    fprintf(fp, "      \"envelope_scc_edges\": [\n");
    size_t readyBatchEnvelopeEdgeWritten = 0;
    for (size_t fromScc = 0; fromScc < lane.envelopeSccSucc.size(); fromScc ++) {
      for (int toScc : lane.envelopeSccSucc[fromScc]) {
        fprintf(fp, "        {\"from\": %zu, \"to\": %d}%s\n",
                fromScc, toScc, readyBatchEnvelopeEdgeWritten + 1 == static_cast<size_t>(lane.envelopeSccEdgeCount) ? "" : ",");
        readyBatchEnvelopeEdgeWritten ++;
      }
    }
    fprintf(fp, "      ],\n");
    fprintf(fp, "      \"envelope_cpp_to_scc\": [\n");
    for (size_t local = 0; local < lane.envelopeCppIds.size(); local ++) {
      fprintf(fp, "        {\"cpp_id\": %d, \"scc\": %d}%s\n",
              lane.envelopeCppIds[local], lane.envelopeSccOfLocal[local], local + 1 == lane.envelopeCppIds.size() ? "" : ",");
    }
    fprintf(fp, "      ],\n");
    fprintf(fp, "      \"regions\": [\n");
    for (size_t idx = 0; idx < lane.regionIndices.size(); idx ++) {
      int regionIndex = lane.regionIndices[idx];
      if (regionIndex >= 0 && regionIndex < static_cast<int>(coarsePlan.regions.size())) {
        const MtCoarseRegion& region = coarsePlan.regions[regionIndex];
        fprintf(fp, "        {\"region_index\": %d, \"begin_cpp_id\": %d, \"end_cpp_id\": %d, \"task_count\": %d, \"member_node_cost\": %d, \"active_word_span\": %d}%s\n",
                regionIndex, region.beginCppId, region.endCppId, region.taskCount, region.memberNodeCost, region.activeWordSpan,
                idx + 1 == lane.regionIndices.size() ? "" : ",");
      } else {
        fprintf(fp, "        {\"region_index\": %d, \"invalid\": true}%s\n", regionIndex, idx + 1 == lane.regionIndices.size() ? "" : ",");
      }
    }
    fprintf(fp, "      ],\n");
    fprintf(fp, "      \"caps\": [\n");
    for (size_t capIdx = 0; capIdx < lane.caps.size(); capIdx ++) {
      const ReadyBatchCapStats& stats = lane.caps[capIdx];
      fprintf(fp, "        {\n");
      fprintf(fp, "          \"cap\": %d,\n", stats.cap);
      fprintf(fp, "          \"static_serial\": %d,\n", stats.staticSerial);
      fprintf(fp, "          \"static_makespan\": %d,\n", stats.staticMakespan);
      fprintf(fp, "          \"static_dispatches\": %d,\n", stats.staticDispatches);
      double staticSpeedup = stats.staticMakespan == 0 ? 0.0 : static_cast<double>(stats.staticSerial) / static_cast<double>(stats.staticMakespan);
      fprintf(fp, "          \"static_speedup\": %.6f,\n", staticSpeedup);
      double traceSpeedup = stats.traceMakespan == 0 ? 0.0 : static_cast<double>(stats.traceSerial) / static_cast<double>(stats.traceMakespan);
      fprintf(fp, "          \"trace\": {\"cycles\": %d, \"serial_total\": %lu, \"makespan_total\": %lu, \"dispatch_total\": %lu, \"aggregate_speedup\": %.6f, \"makespan_p50\": %d, \"makespan_p95\": %d, \"dispatch_p50\": %d, \"dispatch_p95\": %d}\n",
              stats.traceCycles, stats.traceSerial, stats.traceMakespan, stats.traceDispatches, traceSpeedup,
              readyBatchPctInt(stats.traceMakespans, 50), readyBatchPctInt(stats.traceMakespans, 95),
              readyBatchPctInt(stats.traceDispatchCounts, 50), readyBatchPctInt(stats.traceDispatchCounts, 95));
      fprintf(fp, "        }%s\n", capIdx + 1 == lane.caps.size() ? "" : ",");
    }
    fprintf(fp, "      ],\n");
    fprintf(fp, "      \"envelope_caps\": [\n");
    for (size_t capIdx = 0; capIdx < lane.envelopeCaps.size(); capIdx ++) {
      const ReadyBatchCapStats& stats = lane.envelopeCaps[capIdx];
      fprintf(fp, "        {\n");
      fprintf(fp, "          \"cap\": %d,\n", stats.cap);
      fprintf(fp, "          \"static_serial\": %d,\n", stats.staticSerial);
      fprintf(fp, "          \"static_makespan\": %d,\n", stats.staticMakespan);
      fprintf(fp, "          \"static_dispatches\": %d,\n", stats.staticDispatches);
      double staticSpeedup = stats.staticMakespan == 0 ? 0.0 : static_cast<double>(stats.staticSerial) / static_cast<double>(stats.staticMakespan);
      fprintf(fp, "          \"static_speedup\": %.6f,\n", staticSpeedup);
      double traceSpeedup = stats.traceMakespan == 0 ? 0.0 : static_cast<double>(stats.traceSerial) / static_cast<double>(stats.traceMakespan);
      fprintf(fp, "          \"trace\": {\"cycles\": %d, \"serial_total\": %lu, \"makespan_total\": %lu, \"dispatch_total\": %lu, \"aggregate_speedup\": %.6f, \"makespan_p50\": %d, \"makespan_p95\": %d, \"dispatch_p50\": %d, \"dispatch_p95\": %d}\n",
              stats.traceCycles, stats.traceSerial, stats.traceMakespan, stats.traceDispatches, traceSpeedup,
              readyBatchPctInt(stats.traceMakespans, 50), readyBatchPctInt(stats.traceMakespans, 95),
              readyBatchPctInt(stats.traceDispatchCounts, 50), readyBatchPctInt(stats.traceDispatchCounts, 95));
      fprintf(fp, "        }%s\n", capIdx + 1 == lane.envelopeCaps.size() ? "" : ",");
    }
    fprintf(fp, "      ],\n");
    const ReadyBatchLocalEvalStats& evalStats = lane.envelopeLocalEval;
    double localEvalSpeedup = evalStats.traceMakespan == 0 ? 0.0 : static_cast<double>(evalStats.traceSerial) / static_cast<double>(evalStats.traceMakespan);
    double localEvalSerialFraction = evalStats.traceActiveTasks == 0 ? 0.0 : static_cast<double>(evalStats.traceSerialTasks) / static_cast<double>(evalStats.traceActiveTasks);
    fprintf(fp, "      \"envelope_local_eval\": {\n");
    fprintf(fp, "        \"enabled\": %s,\n", envelopeLocalEvalEnabled ? "true" : "false");
    fprintf(fp, "        \"worker_cap\": %d,\n", evalStats.workerCap);
    fprintf(fp, "        \"chunk_cap\": %d,\n", evalStats.chunkCap);
    fprintf(fp, "        \"trace_cycles\": %d,\n", evalStats.traceCycles);
    fprintf(fp, "        \"trace_serial_total\": %lu,\n", evalStats.traceSerial);
    fprintf(fp, "        \"trace_makespan_total\": %lu,\n", evalStats.traceMakespan);
    fprintf(fp, "        \"trace_dispatch_total\": %lu,\n", evalStats.traceDispatches);
    fprintf(fp, "        \"trace_active_scc_total\": %lu,\n", evalStats.traceActiveSccs);
    fprintf(fp, "        \"trace_active_task_total\": %lu,\n", evalStats.traceActiveTasks);
    fprintf(fp, "        \"trace_selected_task_total\": %lu,\n", evalStats.traceSelectedTasks);
    fprintf(fp, "        \"trace_gap_task_total\": %lu,\n", evalStats.traceGapTasks);
    fprintf(fp, "        \"trace_serial_task_total\": %lu,\n", evalStats.traceSerialTasks);
    fprintf(fp, "        \"trace_serial_task_dispatch_total\": %lu,\n", evalStats.traceSerialTaskDispatches);
    fprintf(fp, "        \"trace_active_word_or_volume_total\": %lu,\n", evalStats.traceActiveWordOrVolume);
    fprintf(fp, "        \"local_eval_cap4_trace_speedup\": %.6f,\n", localEvalSpeedup);
    fprintf(fp, "        \"serial_fraction\": %.6f,\n", localEvalSerialFraction);
    fprintf(fp, "        \"makespan_p50\": %d,\n", readyBatchPctInt(evalStats.traceMakespans, 50));
    fprintf(fp, "        \"makespan_p95\": %d,\n", readyBatchPctInt(evalStats.traceMakespans, 95));
    fprintf(fp, "        \"dispatch_p50\": %d,\n", readyBatchPctInt(evalStats.traceDispatchCounts, 50));
    fprintf(fp, "        \"dispatch_p95\": %d,\n", readyBatchPctInt(evalStats.traceDispatchCounts, 95));
    fprintf(fp, "        \"serial_task_dispatch_p50\": %d,\n", readyBatchPctInt(evalStats.traceSerialTaskDispatchCounts, 50));
    fprintf(fp, "        \"serial_task_dispatch_p95\": %d,\n", readyBatchPctInt(evalStats.traceSerialTaskDispatchCounts, 95));
    fprintf(fp, "        \"active_word_or_volume_p50\": %d,\n", readyBatchPctInt(evalStats.traceActiveWordOrVolumes, 50));
    fprintf(fp, "        \"active_word_or_volume_p95\": %d,\n", readyBatchPctInt(evalStats.traceActiveWordOrVolumes, 95));
    uint64_t staticReasonTotal = 0;
    for (const auto& kv : lane.envelopeSerialReasonTaskCount) staticReasonTotal += static_cast<uint64_t>(kv.second);
    uint64_t traceReasonTotal = 0;
    for (const auto& kv : lane.envelopeTraceSerialReasonTaskCount) traceReasonTotal += kv.second;
    uint64_t activeWordHitTotal = 0;
    std::vector<std::pair<int, uint64_t>> activeWordHits;
    for (const auto& kv : lane.envelopeTraceActiveWordHits) {
      activeWordHitTotal += kv.second;
      activeWordHits.push_back(kv);
    }
    std::sort(activeWordHits.begin(), activeWordHits.end(), [](const std::pair<int, uint64_t>& a, const std::pair<int, uint64_t>& b) {
      if (a.second != b.second) return a.second > b.second;
      return a.first < b.first;
    });
    auto readyBatchTopActiveWordShare = [&](size_t limit) -> double {
      if (activeWordHitTotal == 0) return 0.0;
      uint64_t top = 0;
      size_t n = std::min(limit, activeWordHits.size());
      for (size_t idx = 0; idx < n; idx ++) top += activeWordHits[idx].second;
      return static_cast<double>(top) / static_cast<double>(activeWordHitTotal);
    };
    int activeWordMax = 0;
    for (int value : evalStats.traceActiveWordOrVolumes) activeWordMax = std::max(activeWordMax, value);
    const ReadyBatchLocalEvalStats& selectedOnlyStats = lane.envelopeLocalEvalSelectedOnly;
    const ReadyBatchLocalEvalStats& serialWorker0Stats = lane.envelopeLocalEvalSerialWorker0;
    double selectedOnlySpeedup = selectedOnlyStats.traceMakespan == 0 ? 0.0 : static_cast<double>(selectedOnlyStats.traceSerial) / static_cast<double>(selectedOnlyStats.traceMakespan);
    double selectedOnlySerialFraction = selectedOnlyStats.traceActiveTasks == 0 ? 0.0 : static_cast<double>(selectedOnlyStats.traceSerialTasks) / static_cast<double>(selectedOnlyStats.traceActiveTasks);
    double serialWorker0Speedup = serialWorker0Stats.traceMakespan == 0 ? 0.0 : static_cast<double>(serialWorker0Stats.traceSerial) / static_cast<double>(serialWorker0Stats.traceMakespan);
    double serialWorker0SerialFraction = serialWorker0Stats.traceActiveTasks == 0 ? 0.0 : static_cast<double>(serialWorker0Stats.traceSerialTasks) / static_cast<double>(serialWorker0Stats.traceActiveTasks);
    fprintf(fp, "        \"diagnostics\": {\n");
    fprintf(fp, "          \"enabled\": %s,\n", envelopeLocalEvalDiagnosticsEnabled ? "true" : "false");
    fprintf(fp, "          \"serial_reason_task_total\": %lu,\n", staticReasonTotal);
    fprintf(fp, "          \"trace_serial_reason_task_total\": %lu,\n", traceReasonTotal);
    fprintf(fp, "          \"serial_reason_task_counts\": [\n");
    {
      size_t reasonIdx = 0;
      for (const auto& kv : lane.envelopeSerialReasonTaskCount) {
        fprintf(fp, "            {\"reason\": \"%s\", \"count\": %d}%s\n",
                jsonEscape(kv.first).c_str(), kv.second, reasonIdx + 1 == lane.envelopeSerialReasonTaskCount.size() ? "" : ",");
        reasonIdx ++;
      }
    }
    fprintf(fp, "          ],\n");
    fprintf(fp, "          \"trace_serial_reason_task_counts\": [\n");
    {
      size_t reasonIdx = 0;
      for (const auto& kv : lane.envelopeTraceSerialReasonTaskCount) {
        fprintf(fp, "            {\"reason\": \"%s\", \"count\": %lu}%s\n",
                jsonEscape(kv.first).c_str(), kv.second, reasonIdx + 1 == lane.envelopeTraceSerialReasonTaskCount.size() ? "" : ",");
        reasonIdx ++;
      }
    }
    fprintf(fp, "          ],\n");
    fprintf(fp, "          \"trace_state_update_group_hits\": {\n");
    fprintf(fp, "            \"blocked\": %lu,\n", lane.envelopeTraceStateUpdateBlockedHits);
    fprintf(fp, "            \"local_safe_only\": %lu,\n", lane.envelopeTraceStateUpdateLocalSafeOnlyHits);
    fprintf(fp, "            \"runtime_safe\": %lu,\n", lane.envelopeTraceStateUpdateRuntimeSafeHits);
    fprintf(fp, "            \"target_writer_conflict_hit_counts\": ");
    dumpReadyBatchStringUint64CountArray(fp, lane.envelopeTraceStateUpdateTargetWriterConflictHits);
    fprintf(fp, ",\n");
    fprintf(fp, "            \"runtime_block_reason_hit_counts\": ");
    dumpReadyBatchStringUint64CountArray(fp, lane.envelopeTraceStateUpdateRuntimeBlockReasonHits);
    fprintf(fp, "\n");
    fprintf(fp, "          },\n");
    fprintf(fp, "          \"active_word_trace_hit_total\": %lu,\n", activeWordHitTotal);
    fprintf(fp, "          \"active_word_or_volume_p90\": %d,\n", readyBatchPctInt(evalStats.traceActiveWordOrVolumes, 90));
    fprintf(fp, "          \"active_word_or_volume_p99\": %d,\n", readyBatchPctInt(evalStats.traceActiveWordOrVolumes, 99));
    fprintf(fp, "          \"active_word_or_volume_max\": %d,\n", activeWordMax);
    fprintf(fp, "          \"active_word_top1_share\": %.6f,\n", readyBatchTopActiveWordShare(1));
    fprintf(fp, "          \"active_word_top5_share\": %.6f,\n", readyBatchTopActiveWordShare(5));
    fprintf(fp, "          \"active_word_top10_share\": %.6f,\n", readyBatchTopActiveWordShare(10));
    fprintf(fp, "          \"active_word_top\": [\n");
    {
      size_t limit = std::min(static_cast<size_t>(10), activeWordHits.size());
      for (size_t idx = 0; idx < limit; idx ++) {
        double share = activeWordHitTotal == 0 ? 0.0 : static_cast<double>(activeWordHits[idx].second) / static_cast<double>(activeWordHitTotal);
        fprintf(fp, "            {\"active_word\": %d, \"hits\": %lu, \"share\": %.6f}%s\n",
                activeWordHits[idx].first, activeWordHits[idx].second, share, idx + 1 == limit ? "" : ",");
      }
    }
    fprintf(fp, "          ],\n");
    fprintf(fp, "          \"selected_only\": {\"trace_cycles\": %d, \"trace_serial_total\": %lu, \"trace_makespan_total\": %lu, \"trace_dispatch_total\": %lu, \"local_eval_cap4_trace_speedup\": %.6f, \"serial_fraction\": %.6f, \"dispatch_p95\": %d, \"serial_task_dispatch_p95\": %d, \"trace_active_task_total\": %lu, \"trace_selected_task_total\": %lu, \"trace_gap_task_total\": %lu},\n",
            selectedOnlyStats.traceCycles, selectedOnlyStats.traceSerial, selectedOnlyStats.traceMakespan,
            selectedOnlyStats.traceDispatches, selectedOnlySpeedup, selectedOnlySerialFraction,
            readyBatchPctInt(selectedOnlyStats.traceDispatchCounts, 95),
            readyBatchPctInt(selectedOnlyStats.traceSerialTaskDispatchCounts, 95),
            selectedOnlyStats.traceActiveTasks, selectedOnlyStats.traceSelectedTasks, selectedOnlyStats.traceGapTasks);
    fprintf(fp, "          \"serial_on_worker0\": {\"trace_cycles\": %d, \"trace_serial_total\": %lu, \"trace_makespan_total\": %lu, \"trace_dispatch_total\": %lu, \"local_eval_cap4_trace_speedup\": %.6f, \"serial_fraction\": %.6f, \"dispatch_p95\": %d, \"serial_task_dispatch_p95\": %d, \"trace_active_task_total\": %lu, \"trace_selected_task_total\": %lu, \"trace_gap_task_total\": %lu}\n",
            serialWorker0Stats.traceCycles, serialWorker0Stats.traceSerial, serialWorker0Stats.traceMakespan,
            serialWorker0Stats.traceDispatches, serialWorker0Speedup, serialWorker0SerialFraction,
            readyBatchPctInt(serialWorker0Stats.traceDispatchCounts, 95),
            readyBatchPctInt(serialWorker0Stats.traceSerialTaskDispatchCounts, 95),
            serialWorker0Stats.traceActiveTasks, serialWorker0Stats.traceSelectedTasks, serialWorker0Stats.traceGapTasks);
    fprintf(fp, "        }\n");
    fprintf(fp, "      }\n");
    fprintf(fp, "    }%s\n", laneIdx + 1 == lanes.size() ? "" : ",");
  }
  fprintf(fp, "  ]\n");
  fprintf(fp, "}\n");
  fclose(fp);
  printf("[mt-ready-batch] wrote %zu lanes to %s\n", lanes.size(), path.c_str());
  logMtReportTimer("ready-batch", mtReportTimerStart, getTime());
}



FILE* graph::genHeaderStart() {
  headerFilePath = globalConfig.OutputDir + "/" + name + ".h";
  headerTmpFilePath = globalConfig.MtStableOutput ? headerFilePath + ".tmp" : "";
  const std::string openPath = globalConfig.MtStableOutput ? headerTmpFilePath : headerFilePath;
  FILE* header = std::fopen(openPath.c_str(), "w");
  assert(header != NULL);
  setvbuf(header, NULL, _IOFBF, 4 * 1024 * 1024);
  fprintf(header, "#ifndef %s_H\n#define %s_H\n", name.c_str(), name.c_str());
  fprintf(header, "#ifndef _GNU_SOURCE\n#define _GNU_SOURCE\n#endif\n");
  fprintf(header, "#ifdef __linux\n");
  includeLib(header, "pthread.h", true);
  includeLib(header, "sched.h", true);
  fprintf(header, "#endif\n");
  /* include all libs */
  includeLib(header, "iostream", true);
  includeLib(header, "vector", true);
  includeLib(header, "assert.h", true);
  includeLib(header, "stdlib.h", true);
  includeLib(header, "cstdio", true);
  includeLib(header, "cstdint", true);
  includeLib(header, "ctime", true);
  includeLib(header, "iomanip", true);
  includeLib(header, "cstring", true);
  includeLib(header, "map", true);
  includeLib(header, "cstdarg", true);
  includeLib(header, "thread", true);
  includeLib(header, "mutex", true);
  includeLib(header, "condition_variable", true);
  includeLib(header, "chrono", true);
  includeLib(header, "cstdlib", true);
  includeLib(header, "algorithm", true);
  includeLib(header, "atomic", true);
  if (mtUseActivationEventTraceCodegen()) includeLib(header, "cstddef", true);
  newLine(header);

  fprintf(header, "\n// User configuration\n");
  fprintf(header, "//#define ENABLE_LOG\n");
  fprintf(header, "//#define RANDOMIZE_INIT\n");

  fprintf(header, "\n#define gAssert(cond, ...) do {"
                     "if (!(cond)) {"
                       "fprintf(stderr, \"\\33[1;31m\");"
                       "fprintf(stderr, __VA_ARGS__);"
                       "fprintf(stderr, \"\\33[0m\\n\");"
                       "assert(cond);"
                     "}"
                   "} while (0)\n");
  fprintf(header, "#define gdiv(a, b) ((b) == 0 ? 0 : (a) / (b))\n");

  fprintf(header, "#ifndef __BITINT_MAXWIDTH__\n");
  fprintf(header, "#error  BITINT support is required\n");
  fprintf(header, "#endif\n\n");

  /* There is some bugs with _BitInt in clang 18 */
  fprintf(header, "#ifdef __clang__\n");
  fprintf(header, "#if __clang_major__ < 19\n");
  fprintf(header, "#error  Please compile with clang 19 or above\n");
  fprintf(header, "#endif\n");
  fprintf(header, "#endif // __clang__ \n\n");

  fprintf(header, "#define likely(x) __builtin_expect(!!(x), 1)\n");
  fprintf(header, "#define unlikely(x) __builtin_expect(!!(x), 0)\n");
  fprintf(header, "void gprintf(const char *fmt, ...);\n\n");

  for (int num = 2; num <= maxConcatNum; num ++) {
    std::string param;
    for (int i = num; i > 0; i --) param += format(i == num ? "_%d" : ", _%d", i);
    std::string value;
    std::string type = widthUType(num * 64);
    for (int i = num; i > 1; i --) {
      value += format(i == num ? "((%s)_%d << %d) " : "| ((%s)_%d << %d)", type.c_str(), i, (i-1) * 64);
    }
    value += format("| ((%s)_1)", type.c_str());
    fprintf(header, "#define UINT_CONCAT%d(%s) (%s)\n", num, param.c_str(), value.c_str());
  }
  for (std::string str : extDecl) fprintf(header, "%s\n", str.c_str());
  newLine(header);
  return header;
}

void graph::genInterfaceInput(Node* input) {
  /* set by string */
  emitFuncDecl(0, "void S%s::set_%s(%s val) {\n", name.c_str(), input->name.c_str(), widthUType(input->width).c_str());
  emitBodyLock(1, "if (%s != val) { \n", input->name.c_str());
  emitBodyLock(2, "%s = val;\n", input->name.c_str());
  /* update nodes in the same superNode */
  std::set<int> allNext;
  for (Node* next : input->next) {
    if (next->super->cppId >= 0) allNext.insert(next->super->cppId);
  }
  std::map<uint64_t, ActiveType> bitMapInfo;
  activeSet2bitMap(allNext, bitMapInfo, -1);
  for (auto iter : bitMapInfo) {
    emitBodyLock(2, "%s // %s\n", updateActiveStr(iter.first, ACTIVE_MASK(iter.second)).c_str(), ACTIVE_COMMENT(iter.second).c_str());
    if (mtUseActivationEventTraceCodegen()) {
      emitBodyLock(2, "recordMtActivationEvent(-1, (uint32_t)%lu, (uint64_t)0x%lx, MT_ACTIVATION_EVENT_CONDITIONAL);\n", iter.first, ACTIVE_MASK(iter.second));
    }
  }
  emitBodyLock(1, "}\n");
  emitBodyLock(0, "}\n");
}

void graph::genInterfaceOutput(Node* output) {
  emitFuncDecl(0, "%s S%s::get_%s() {\n"
               "  return %s;\n"
               "}\n",
               widthUType(output->width).c_str(), name.c_str(),
               output->name.c_str(), output->status == CONSTANT_NODE ? output->computeInfo->valStr.c_str() : output->name.c_str());
}

static void emitActiveBufferDef(FILE* header, int activeWords) {
  int packedActiveWords = 64 / ACTIVE_WIDTH;
  fprintf(header,
          "struct ActiveBuffer {\n"
          "  uint%d_t words[%d];\n"
          "  int touchedWords[%d];\n"
          "  int touchedCount;\n"
          "  bool allActive;\n"
          "  ActiveBuffer() : touchedCount(0), allActive(false) {\n"
          "    memset(words, 0, sizeof(words));\n"
          "  }\n"
          "  void clear() {\n"
          "    if (allActive) {\n"
          "      memset(words, 0, sizeof(words));\n"
          "      allActive = false;\n"
          "      touchedCount = 0;\n"
          "      return;\n"
          "    }\n"
          "    for (int touchedIdx = 0; touchedIdx < touchedCount; touchedIdx ++) words[touchedWords[touchedIdx]] = 0;\n"
          "    touchedCount = 0;\n"
          "  }\n"
          "  void orWord(int idx, uint64_t mask) {\n"
          "    // mask packs consecutive active words in little-endian ACTIVE_WIDTH chunks.\n"
          "    for (int i = 0; i < %d && idx + i < %d; i ++) {\n"
          "      uint%d_t value = (uint%d_t)(mask >> (i * %d));\n"
          "      if (value == 0) continue;\n"
          "      int wordIdx = idx + i;\n"
          "      if (!allActive && words[wordIdx] == 0) touchedWords[touchedCount ++] = wordIdx;\n"
          "      words[wordIdx] |= value;\n"
          "    }\n"
          "  }\n"
          "  void activateAll() {\n"
          "    memset(words, 0xff, sizeof(words));\n"
          "    touchedCount = 0;\n"
          "    allActive = true;\n"
          "  }\n"
          "  void mergeFrom(uint%d_t *activeFlags) const {\n"
          "    if (allActive) {\n"
          "      for (int i = 0; i < %d; i ++) activeFlags[i] |= words[i];\n"
          "      return;\n"
          "    }\n"
          "    for (int touchedIdx = 0; touchedIdx < touchedCount; touchedIdx ++) {\n"
          "      int wordIdx = touchedWords[touchedIdx];\n"
          "      activeFlags[wordIdx] |= words[wordIdx];\n"
          "    }\n"
          "  }\n"
          "};\n\n",
          ACTIVE_WIDTH, activeWords, activeWords, packedActiveWords, activeWords, ACTIVE_WIDTH, ACTIVE_WIDTH, ACTIVE_WIDTH, ACTIVE_WIDTH, activeWords);
}

static void emitActivationDeltaDef(FILE* header, int activeWords) {
  int packedActiveWords = 64 / ACTIVE_WIDTH;
  fprintf(header,
          "struct ActivationDeltaEntry {\n"
          "  int idx;\n"
          "  uint64_t mask;\n"
          "};\n"
          "struct alignas(64) ActivationDelta {\n"
          "  std::vector<ActivationDeltaEntry> entries;\n"
          "  bool allActive;\n"
          "  ActivationDelta() : allActive(false) {}\n"
          "  void clear() {\n"
          "    entries.clear();\n"
          "    allActive = false;\n"
          "  }\n"
          "  void orWord(int idx, uint64_t mask) {\n"
          "    // mask packs consecutive active words in little-endian ACTIVE_WIDTH chunks.\n"
          "    for (int i = 0; i < %d && idx + i < %d; i ++) {\n"
          "      uint%d_t value = (uint%d_t)(mask >> (i * %d));\n"
          "      if (value == 0) continue;\n"
          "      entries.push_back({idx + i, value});\n"
          "    }\n"
          "  }\n"
          "  void activateAll() {\n"
          "    allActive = true;\n"
          "  }\n"
          "  void mergeInto(uint%d_t *activeFlags) const {\n"
          "    if (allActive) {\n"
          "      for (int i = 0; i < %d; i ++) activeFlags[i] = (uint%d_t)-1;\n"
          "      return;\n"
          "    }\n"
          "    for (const ActivationDeltaEntry &entry : entries) {\n"
          "      activeFlags[entry.idx] |= (uint%d_t)entry.mask;\n"
          "    }\n"
          "  }\n"
          "};\n\n",
          packedActiveWords, activeWords, ACTIVE_WIDTH, ACTIVE_WIDTH, ACTIVE_WIDTH,
          ACTIVE_WIDTH, activeWords, ACTIVE_WIDTH, ACTIVE_WIDTH);
}

static void emitActivationEventTraceDef(FILE* header) {
  fprintf(header,
          "enum MtActivationEventKind : uint8_t {\n"
          "  MT_ACTIVATION_EVENT_CONDITIONAL = 1,\n"
          "  MT_ACTIVATION_EVENT_UNCONDITIONAL = 2,\n"
          "  MT_ACTIVATION_EVENT_ACTIVATE_ALL = 3,\n"
          "  MT_ACTIVATION_EVENT_FRONTIER = 4,\n"
          "  MT_ACTIVATION_EVENT_CYCLE_END = 5\n"
          "};\n"
          "struct MtActivationEventTraceHeader {\n"
          "  uint8_t magic[8];\n"
          "  uint16_t version;\n"
          "  uint16_t headerSize;\n"
          "  uint16_t activeWidth;\n"
          "  uint16_t reserved0;\n"
          "  uint32_t taskCount;\n"
          "  uint32_t recordSize;\n"
          "  uint64_t traceStart;\n"
          "  uint64_t traceCount;\n"
          "  uint64_t reserved1;\n"
          "};\n"
          "struct MtActivationEventTraceRecord {\n"
          "  uint64_t cycle;\n"
          "  int32_t sourceCppId;\n"
          "  uint32_t activeWordBase;\n"
          "  uint64_t mask;\n"
          "  MtActivationEventKind kind;\n"
          "  uint8_t reserved[7];\n"
          "};\n"
          "static_assert(sizeof(MtActivationEventTraceHeader) == 48, \"activation-event trace header size\");\n"
          "static_assert(offsetof(MtActivationEventTraceHeader, version) == 8, \"activation-event trace header layout\");\n"
          "static_assert(offsetof(MtActivationEventTraceHeader, traceStart) == 24, \"activation-event trace header layout\");\n"
          "static_assert(offsetof(MtActivationEventTraceHeader, traceCount) == 32, \"activation-event trace header layout\");\n"
          "static_assert(offsetof(MtActivationEventTraceHeader, reserved1) == 40, \"activation-event trace header layout\");\n"
          "static_assert(sizeof(MtActivationEventTraceRecord) == 32, \"activation-event trace record size\");\n"
          "static_assert(offsetof(MtActivationEventTraceRecord, sourceCppId) == 8, \"activation-event trace record layout\");\n"
          "static_assert(offsetof(MtActivationEventTraceRecord, mask) == 16, \"activation-event trace record layout\");\n"
          "static_assert(offsetof(MtActivationEventTraceRecord, kind) == 24, \"activation-event trace record layout\");\n\n");
}

#if defined(DIFFTEST_PER_SIG) && defined(GSIM_DIFF)
void graph::genDiffSig(FILE* fp, Node* node) {
  std::set<std::string> allNames;
  std::string diffNodeName = node->name;
  std::string originName = node->name;
  if (node->type == NODE_MEMORY){

  } else if (node->isArray()) {
    int num = node->arrayEntryNum();
    std::vector<std::string> suffix(num);
    int pairNum = 1;
    for (size_t i = 0; i < node->dimension.size(); i ++) {
      int suffixIdx = 0;
      for (int l = 0; l < pairNum; l ++) {
        for (int j = 0; j < node->dimension[i]; j ++) {
          int suffixNum = num / node->dimension[i];
          for (int k = 0; k < suffixNum; k ++) {
            suffix[suffixIdx] += "[" + std::to_string(j) + "]";
            suffixIdx ++;
          }
        }
      }
      num = num / node->dimension[i];
      pairNum *= node->dimension[i];
    }
    for (size_t i = 0; i < suffix.size(); i ++) {
      allNames.insert(diffNodeName + suffix[i]);
    }
  } else {
    allNames.insert(diffNodeName);
  }
  for (auto iter : allNames)
    fprintf(sigFile, "%d %d %s %s\n", node->sign, node->width, iter.c_str(), iter.c_str());
}
#endif

#if defined(DIFFTEST_PER_SIG) && defined(VERILATOR_DIFF)
void graph::genDiffSig(FILE* fp, Node* node) {
  std::string verilatorName = name + "__DOT__" + node->name;
  size_t pos;
  while ((pos = verilatorName.find("$$")) != std::string::npos) {
    verilatorName.replace(pos, 2, "_");
  }
  while ((pos = verilatorName.find("$")) != std::string::npos) {
    verilatorName.replace(pos, 1, "__DOT__");
  }
  std::map<std::string, std::string> allNames;
  std::string diffNodeName = node->name;
  std::string originName = node->name;
  if (node->type == NODE_MEMORY){

  } else if (node->isArray()) {
    int num = node->arrayEntryNum();
    std::vector<std::string> suffix(num);
    std::vector<std::string> verilatorSuffix(num);
    int pairNum = 1;
    for (size_t i = 0; i < node->dimension.size(); i ++) {
      int suffixIdx = 0;
      for (int l = 0; l < pairNum; l ++) {
        for (int j = 0; j < node->dimension[i]; j ++) {
          int suffixNum = num / node->dimension[i];
          for (int k = 0; k < suffixNum; k ++) {
            verilatorSuffix[suffixIdx] += "_" + std::to_string(j);
            suffix[suffixIdx] += "[" + std::to_string(j) + "]";
            suffixIdx ++;
          }
        }
      }
      num = num / node->dimension[i];
      pairNum *= node->dimension[i];
    }
    for (size_t i = 0; i < suffix.size(); i ++) {
      if (!nameExist(originName + verilatorSuffix[i])) {
        allNames[diffNodeName + suffix[i]] = verilatorName + verilatorSuffix[i];
      }
    }
  } else {
    allNames[diffNodeName] = verilatorName;
  }
  for (auto iter : allNames)
    fprintf(sigFile, "%d %d %s %s\n", node->sign, node->width, iter.first.c_str(), iter.second.c_str());
}
#endif

void graph::genNodeDef(FILE* fp, Node* node) {
  if (node->type == NODE_SPECIAL || node->type == NODE_REG_RESET || (node->status != VALID_NODE)) return;
  if (node->type == NODE_REG_DST && !node->regSplit) return;
  if (node->type == NODE_WRITER) return;
  if (node->isLocal()) return;
#if defined(GSIM_DIFF) || defined(VERILATOR_DIFF)
  genDiffSig(fp, node);
#endif
  if (definedNode.find(node) != definedNode.end()) return;
  definedNode.insert(node);
  fprintf(fp, "%s %s", widthUType(node->width).c_str(), node->name.c_str());
  if (node->type == NODE_MEMORY) fprintf(fp, "[%d]", upperPower2(node->depth));
  for (int dim : node->dimension) fprintf(fp, "[%d]", upperPower2(dim));
  if (const std::string* orig = mtShortNameOrigOf(node)) {
    fprintf(fp, "; // width = %d, lineno = %d, orig=%s\n", node->width, node->lineno, orig->c_str());
  } else {
    fprintf(fp, "; // width = %d, lineno = %d\n", node->width, node->lineno);
  }
  int w = node->width;
  bool needInitMask = (node->type != NODE_MEMORY && node->type != NODE_WRITER) &&
    (((w < 64) && (w != 8 && w != 16 && w != 32 && w != 64)) || ((w > 64) && (w % 32 != 0)));
  if (needInitMask) {
    if (node->dimension.empty()) {
      emitBodyLock(1, "%s &= %s;\n", node->name.c_str(), bitMask(w).c_str());
    } else {
      int indent = 1;
      int dims = node->dimension.size();
      for (int i = 0; i < dims; i ++) {
        emitBodyLock(indent ++, "for (int i%d = 0; i%d < %d; i%d ++) {\n", i, i, node->dimension[i], i);
      }
      emitBodyLock(indent, "%s", node->name.c_str());
      for (int i = 0; i < dims; i ++) { emitBodyLock(0, "[i%d]", i); }
      emitBodyLock(0, "&= %s;\n", bitMask(w).c_str());
      for (int i = 0; i < dims; i ++) { emitBodyLock(-- indent, "}\n"); }
    }
  }

  /* save reset registers */
  if (node->isReset() && node->type == NODE_REG_SRC) {
    Assert(!node->isArray() && node->width <= BASIC_WIDTH, "%s is treated as reset (isArray: %d width: %d)", node->name.c_str(), node->isArray(), node->width);
    fprintf(fp, "%s %s;\n", widthUType(node->width).c_str(), RESET_NAME(node).c_str());
    if (needInitMask) {
      emitBodyLock(1, "%s = %s & %s;\n", RESET_NAME(node).c_str(), RESET_NAME(node).c_str(), bitMask(w).c_str());
    }
  }
}

void graph::activateNext(Node* node, std::set<int>& nextNodeId, std::string oldName, bool inStep, std::string flagName,
                         std::string activeBufferName, int indent, bool emitActivation) {
  std::string nodeName = node->name;
  if (!emitActivation) {
    if (inStep) {
      if (node->isReset() && node->type == NODE_REG_SRC) emitBodyLock(indent, "%s = %s;\n", RESET_NAME(node).c_str(), newName(node).c_str());
      emitBodyLock(indent, "%s = %s;\n", node->name.c_str(), newName(node).c_str());
    }
    return;
  }
  auto condName = std::string("cond_") + nodeName;
  bool opt{false};

  std::map<uint64_t, ActiveType> bitMapInfo;
  ActiveType curMask;
  if (node->isAsyncReset()) {
    emitBodyLock(indent ++, "if (%s || (%s != %s)) {\n", oldName.c_str(), nodeName.c_str(), oldName.c_str());
  } else {
    curMask = activeSet2bitMap(nextNodeId, bitMapInfo, node->super->cppId);
    opt = ((ACTIVE_MASK(curMask) != 0) + bitMapInfo.size()) <= 3;
    if (opt) {
      if (node->width == 1) emitBodyLock(indent, "bool %s = %s ^ %s;\n", condName.c_str(), nodeName.c_str(), oldName.c_str());
      else emitBodyLock(indent, "bool %s = %s != %s;\n", condName.c_str(), nodeName.c_str(), oldName.c_str());
    }
    else {
      emitBodyLock(indent ++, "if (%s != %s) {\n", nodeName.c_str(), oldName.c_str());
    }
  }
  if (inStep) {
    if (node->isReset() && node->type == NODE_REG_SRC) emitBodyLock(indent, "%s = %s;\n", RESET_NAME(node).c_str(), newName(node).c_str());
    emitBodyLock(indent, "%s = %s;\n", node->name.c_str(), newName(node).c_str());
  }
  if (node->isAsyncReset()) {
    Assert(!opt, "invalid opt");
    if (activeBufferName.empty()) {
      if (mtUseActivationEventTraceCodegen() && !mtActivationEventTraceSuppressed) emitBodyLock(indent, "activateAll(%d);\n", mtActivationEventTraceSourceCppId);
      else emitBodyLock(indent, "activateAll();\n");
    } else {
      emitBodyLock(indent, "%s.activateAll();\n", activeBufferName.c_str());
      if (mtUseActivationEventTraceCodegen() && !mtActivationEventTraceSuppressed) {
        emitBodyLock(indent, "recordMtActivationEvent(%d, 0, UINT64_MAX, MT_ACTIVATION_EVENT_ACTIVATE_ALL);\n", mtActivationEventTraceSourceCppId);
      }
    }
    emitBodyLock(indent, "%s = -1;\n", flagName.c_str());
  } else {
    if (ACTIVE_MASK(curMask) != 0) {
      if (opt) emitBodyLock(indent, "%s |= -(uint%d_t)%s & 0x%lx; // %s\n", flagName.c_str(), ACTIVE_WIDTH, condName.c_str(), ACTIVE_MASK(curMask), ACTIVE_COMMENT(curMask).c_str());
      else emitBodyLock(indent, "%s |= 0x%lx; // %s\n", flagName.c_str(), ACTIVE_MASK(curMask), ACTIVE_COMMENT(curMask).c_str());
      if (mtUseActivationEventTraceCodegen() && !mtActivationEventTraceSuppressed) {
        if (opt) {
          emitBodyLock(indent, "recordMtActivationEvent(%d, (uint32_t)%d, (-(uint64_t)%s & (uint64_t)0x%lx), MT_ACTIVATION_EVENT_CONDITIONAL);\n",
                       mtActivationEventTraceSourceCppId, node->super->cppId / ACTIVE_WIDTH, condName.c_str(), ACTIVE_MASK(curMask));
        } else {
          emitBodyLock(indent, "recordMtActivationEvent(%d, (uint32_t)%d, (uint64_t)0x%lx, MT_ACTIVATION_EVENT_CONDITIONAL);\n",
                       mtActivationEventTraceSourceCppId, node->super->cppId / ACTIVE_WIDTH, ACTIVE_MASK(curMask));
        }
      }
    }
    for (auto iter : bitMapInfo) {
      auto str = opt ? updateActiveStr(iter.first, ACTIVE_MASK(iter.second), condName, ACTIVE_UNIQUE(iter.second), activeBufferName)
                     : updateActiveStr(iter.first, ACTIVE_MASK(iter.second), activeBufferName);
      emitBodyLock(indent, "%s // %s\n", str.c_str(), ACTIVE_COMMENT(iter.second).c_str());
      if (mtUseActivationEventTraceCodegen() && !mtActivationEventTraceSuppressed) {
        if (!opt) {
          emitBodyLock(indent, "recordMtActivationEvent(%d, (uint32_t)%lu, (uint64_t)0x%lx, MT_ACTIVATION_EVENT_CONDITIONAL);\n",
                       mtActivationEventTraceSourceCppId, iter.first, ACTIVE_MASK(iter.second));
        } else if (ACTIVE_UNIQUE(iter.second) >= 0) {
          emitBodyLock(indent, "recordMtActivationEvent(%d, (uint32_t)%lu, ((uint64_t)%s << %d), MT_ACTIVATION_EVENT_CONDITIONAL);\n",
                       mtActivationEventTraceSourceCppId, iter.first, condName.c_str(), ACTIVE_UNIQUE(iter.second));
        } else {
          emitBodyLock(indent, "recordMtActivationEvent(%d, (uint32_t)%lu, (-(uint64_t)%s & (uint64_t)0x%lx), MT_ACTIVATION_EVENT_CONDITIONAL);\n",
                       mtActivationEventTraceSourceCppId, iter.first, condName.c_str(), ACTIVE_MASK(iter.second));
        }
      }
    }
  #ifdef PERF
    #if ENABLE_ACTIVATOR
    for (int id : nextNodeId) {
      emitBodyLock(indent, "if (activator[%d].find(%d) == activator[%d].end()) activator[%d][%d] = 0;\nactivator[%d][%d] ++;\n",
                  id, node->super->cppId, id, id, node->super->cppId, id, node->super->cppId);
    }
    #endif
    if (inStep && node->type != NODE_EXT_OUT) emitBodyLock(indent, "isActivateValid = true;\n");
  #endif
  }
  if (!opt) emitBodyLock(-- indent, "}\n");
}
void graph::activateUncondNext(Node* node, std::set<int>& activateId, bool inStep, std::string flagName,
                               std::string activeBufferName, int indent, bool emitActivation) {
  if (!emitActivation) return;
  std::map<uint64_t, ActiveType> bitMapInfo;
  auto curMask = activeSet2bitMap(activateId, bitMapInfo, node->super->cppId);
  if (ACTIVE_MASK(curMask) != 0) {
    emitBodyLock(indent, "%s |= 0x%lx; // %s\n", flagName.c_str(), ACTIVE_MASK(curMask), ACTIVE_COMMENT(curMask).c_str());
    if (mtUseActivationEventTraceCodegen() && !mtActivationEventTraceSuppressed) {
      emitBodyLock(indent, "recordMtActivationEvent(%d, (uint32_t)%d, (uint64_t)0x%lx, MT_ACTIVATION_EVENT_UNCONDITIONAL);\n",
                   mtActivationEventTraceSourceCppId, node->super->cppId / ACTIVE_WIDTH, ACTIVE_MASK(curMask));
    }
  }
  for (auto iter : bitMapInfo) {
    emitBodyLock(indent, "%s // %s\n", updateActiveStr(iter.first, ACTIVE_MASK(iter.second), activeBufferName).c_str(), ACTIVE_COMMENT(iter.second).c_str());
    if (mtUseActivationEventTraceCodegen() && !mtActivationEventTraceSuppressed) {
      emitBodyLock(indent, "recordMtActivationEvent(%d, (uint32_t)%lu, (uint64_t)0x%lx, MT_ACTIVATION_EVENT_UNCONDITIONAL);\n",
                   mtActivationEventTraceSourceCppId, iter.first, ACTIVE_MASK(iter.second));
    }
  }
#ifdef PERF
  #if ENABLE_ACTIVATOR
  for (int id : activateId) {
    emitBodyLock(indent, "if (activator[%d].find(%d) == activator[%d].end()) activator[%d][%d] = 0;\n activator[%d][%d] ++;\n",
                id, node->super->cppId, id, id, node->super->cppId, id, node->super->cppId);
  }
  #endif
  if (inStep) emitBodyLock(indent, "isActivateValid = true;\n");
#endif
}
int graph::genNodeStepStart(SuperNode* node, uint64_t mask, int idx, std::string flagName, int indent, bool skipAdmissionGuard) {
  nodeNum ++;
  if (!skipAdmissionGuard && !isAlwaysActive(node->cppId)) {
    emitBodyLock(indent ++, "if(unlikely(%s & 0x%lx)) { // id=%d\n", flagName.c_str(), mask, idx);
  }
  int id;
  uint64_t newMask;
  std::tie(id, newMask) = clearIdxMask(node->cppId);
#ifdef PERF
  emitBodyLock(indent, "activeTimes[%d] ++;\n", node->cppId);
  if (node->superType != SUPER_EXTMOD) {
    emitBodyLock(indent, "bool isActivateValid = false;\n");
  }
#endif
  return indent;
}

void graph::nodeDisplay(Node* member, int indent) {
#define emit_display(varname, width, indent) \
  do { \
    int n = ROUNDUP(width, 64) / 64; \
    std::string s = "printf(\"%%lx"; \
    for (int i = n - 2; i >= 0; i --) { \
      s += "|%%lx"; \
    } \
    s += "\", "; \
    for (n --; n > 0; n --) { \
      s += format("(uint64_t)(%s >> %d)", varname, n * 64); \
      s += ", "; \
    } \
    s += format("(uint64_t)%s",varname);\
    s += ");"; \
    emitBodyLock(indent, s.c_str()); \
  } while (0)

  if (member->status != VALID_NODE) return;
  if (member->type == NODE_WRITER) return;
  emitBodyLock(indent, "printf(\"%%ld %d %s: \", cycles);\n", member->super->cppId, member->name.c_str());
  if (member->dimension.size() != 0) {
    std::string idxStr;
    for (size_t i = 0; i < member->dimension.size(); i ++) {
      emitBodyLock(indent ++, "for(int i%ld = 0; i%ld < %d; i%ld ++) {\n", i, i, member->dimension[i], i);
      idxStr += "[i" + std::to_string(i) + "]";
    }
    std::string nameIdx = member->name + idxStr;
    emit_display(nameIdx.c_str(), member->width, indent);
    emitBodyLock(indent, "printf(\" \");\n");
    for (size_t i = 0; i < member->dimension.size(); i ++) {
      emitBodyLock(-- indent, "}\n");
    }
  } else {
    if (member->anyNextActive() || member->type != NODE_SPECIAL) {
      emit_display(member->name.c_str(), member->width, indent);
    }
  }
  emitBodyLock(indent, "printf(\"\\n\");\n");
}

int graph::genNodeStepEnd(SuperNode* node, int indent, bool skipAdmissionGuard) {
#ifdef PERF
  if (node->superType != SUPER_EXTMOD) {
    emitBodyLock(indent, "validActive[%d] += isActivateValid;\n", node->cppId);
  }
#endif

  if(!skipAdmissionGuard && !isAlwaysActive(node->cppId)) {
    emitBodyLock(-- indent, "}\n");
  }
  return indent;
}

bool Node::isLocal() { // TODO: isArray is OK
  return status == VALID_NODE && type == NODE_OTHERS && !anyNextActive() && !isArray() && !isReset();
}


int graph::translateInst(InstInfo inst, int indent, std::string flagName, std::string activeBufferName, bool emitActivation) {
  switch (inst.infoType) {
    case SUPER_INFO_IF:
      emitBodyLock(indent ++, "%s\n", inst.inst.c_str());
      break;
    case SUPER_INFO_ELSE:
      emitBodyLock(indent - 1,  "%s\n", inst.inst.c_str());
      break;
    case SUPER_INFO_DEDENT:
      emitBodyLock(--indent, "%s\n", inst.inst.c_str());
      break;
    case SUPER_INFO_STR:
      emitBodyLock(indent, "%s\n", inst.inst.c_str());
      break;
    case SUPER_INFO_ASSIGN_BEG:
      if (inst.node->isLocal() || inst.node->isArray() || inst.node->type == NODE_WRITER) break;
      // Report-only histogram (GSIM_MT_DENSE_OLDVALUE_HISTOGRAM=1): does this snapshot's
      // change-detection feed any activation consumer? nextActiveId empty = candidate for
      // per-node dead-code elimination (the audit's largest bookkeeping class).
      if (mtOldValueHistogramEnabled()) {
        if (inst.node->nextActiveId.empty()) mtOldSnapNoConsumers.fetch_add(1, std::memory_order_relaxed);
        else mtOldSnapWithConsumers.fetch_add(1, std::memory_order_relaxed);
      }
      emitBodyLock(indent, "%s %s = %s;\n", widthUType(inst.node->width).c_str(), oldName(inst.node).c_str(), inst.node->name.c_str());
      break;
    case SUPER_INFO_ASSIGN_END:
      if (inst.node->isLocal() || !inst.node->needActivate()) break;
      if (inst.node->isArray() || inst.node->type == NODE_WRITER) activateUncondNext(inst.node, inst.node->nextActiveId, false, flagName, activeBufferName, indent, emitActivation);
      else activateNext(inst.node, inst.node->nextActiveId, oldName(inst.node), false, flagName, activeBufferName, indent, emitActivation);
      break;
    default:
      break;
  }
  return indent;
}

void graph::genSuperEval(SuperNode* super, std::string flagName, std::string activeBufferName, int indent, bool emitActivation) { // current indent = 2
  int savedTraceSourceCppId = mtActivationEventTraceSourceCppId;
  if (emitActivation && !mtActivationEventTraceSuppressed) mtActivationEventTraceSourceCppId = super->cppId;
  if (super->superType == SUPER_EXTMOD) { // TODO: normalize
    auto emitExtAsyncReset = [&](Node* extOut) {
      if (!extOut->isAsyncReset()) return;
      auto resetId = super2ResetId.find(extOut);
      Assert(resetId != super2ResetId.end() && resetId->second.second >= 0, "missing async reset id for %s", extOut->name.c_str());
      emitBodyLock(indent, "subReset%d();\n", resetId->second.second);
    };
    for (size_t i = 1; i < super->member.size(); i ++) {
      emitExtAsyncReset(super->member[i]);
    }
    /* save old EXT_OUT*/
    for (size_t i = 1; i < super->member.size(); i ++) {
      if (!super->member[i]->needActivate()) continue;
      Node* extOut = super->member[i];
      emitBodyLock(indent, "%s %s = %s;\n", widthUType(extOut->width).c_str(), oldName(extOut).c_str(), extOut->name.c_str());
    }
    for (InstInfo inst : super->insts) {
      indent = translateInst(inst, indent, flagName, activeBufferName, emitActivation);
    }
    for (size_t i = 1; i < super->member.size(); i ++) {
      emitExtAsyncReset(super->member[i]);
    }
    for (size_t i = 1; i < super->member.size(); i ++) {
      if (!super->member[i]->needActivate()) continue;
      if (super->member[i]->isArray()) activateUncondNext(super->member[i], super->member[i]->nextActiveId, false, flagName, activeBufferName, indent, emitActivation);
      else activateNext(super->member[i], super->member[i]->nextActiveId, oldName(super->member[i]), false, flagName, activeBufferName, indent, emitActivation);
    }
  } else {
    if (super->superType == SUPER_ASYNC_RESET) {
      int resetId = super2ResetIdLookup(super->resetNode).second;
      if (!emitActivation && activeBufferName.empty()) {
        int denseResetId = -1;
        auto denseResetIt = super2DenseResetId.find(super->resetNode);
        if (denseResetIt != super2DenseResetId.end()) denseResetId = denseResetIt->second.second;
        Assert(denseResetId >= 0, "missing dense async reset id for %s", super->resetNode->name.c_str());
        emitBodyLock(indent, "subResetDense%d();\n", denseResetId);
      } else if (activeBufferName.empty()) {
        if (mtUseActivationEventTraceCodegen()) emitBodyLock(indent, "subReset%d(%d);\n", resetId, mtActivationEventTraceSourceCppId);
        else emitBodyLock(indent, "subReset%d();\n", resetId);
      } else {
        if (mtUseActivationEventTraceCodegen()) emitBodyLock(indent, "subReset%d(%s, %d);\n", resetId, activeBufferName.c_str(), mtActivationEventTraceSourceCppId);
        else emitBodyLock(indent, "subReset%d(%s);\n", resetId, activeBufferName.c_str());
      }
    }
    /* local nodes definition */
    for (Node* n : super->member) {
      if (n->isLocal()) {
        emitBodyLock(indent, "%s %s;\n", widthUType(n->width).c_str(), n->name.c_str());
      }
    }
    if (mtUseDenseElideObservability() && mtDenseObservabilitySpansBalanced(super->insts)) {
      const std::set<Node*>& droppable = mtDenseObservabilityDroppableSet();
      if (!droppable.empty()) {
        std::vector<Node*> frameStack;
        for (InstInfo inst : super->insts) {
          if (inst.infoType == SUPER_INFO_ASSIGN_BEG) {
            frameStack.push_back(inst.node);
            if (droppable.find(inst.node) == droppable.end()) indent = translateInst(inst, indent, flagName, activeBufferName, emitActivation);
            continue;
          }
          if (inst.infoType == SUPER_INFO_ASSIGN_END) {
            Node* endNode = inst.node;
            if (!frameStack.empty()) frameStack.pop_back();
            if (droppable.find(endNode) == droppable.end()) indent = translateInst(inst, indent, flagName, activeBufferName, emitActivation);
            continue;
          }
          if (!frameStack.empty() && droppable.find(frameStack.back()) != droppable.end()) continue;
          indent = translateInst(inst, indent, flagName, activeBufferName, emitActivation);
        }
      } else {
        for (InstInfo inst : super->insts) {
          indent = translateInst(inst, indent, flagName, activeBufferName, emitActivation);
        }
      }
    } else {
      for (InstInfo inst : super->insts) {
        indent = translateInst(inst, indent, flagName, activeBufferName, emitActivation);
      }
    }
    if (super->superType == SUPER_ASYNC_RESET) {
      int resetId = super2ResetIdLookup(super->resetNode).second;
      if (!emitActivation && activeBufferName.empty()) {
        int denseResetId = -1;
        auto denseResetIt = super2DenseResetId.find(super->resetNode);
        if (denseResetIt != super2DenseResetId.end()) denseResetId = denseResetIt->second.second;
        Assert(denseResetId >= 0, "missing dense async reset id for %s", super->resetNode->name.c_str());
        emitBodyLock(indent, "subResetDense%d();\n", denseResetId);
      } else if (activeBufferName.empty()) {
        if (mtUseActivationEventTraceCodegen()) emitBodyLock(indent, "subReset%d(%d);\n", resetId, mtActivationEventTraceSourceCppId);
        else emitBodyLock(indent, "subReset%d();\n", resetId);
      } else {
        if (mtUseActivationEventTraceCodegen()) emitBodyLock(indent, "subReset%d(%s, %d);\n", resetId, activeBufferName.c_str(), mtActivationEventTraceSourceCppId);
        else emitBodyLock(indent, "subReset%d(%s);\n", resetId, activeBufferName.c_str());
      }
    }
    emitBodyLock(indent, "#ifdef ENABLE_LOG\n");
    emitBodyLock(indent ++, "if (cycles >= LOG_START && cycles <= LOG_END) {\n");
    for (Node* n : super->member) nodeDisplay(n, indent);
    emitBodyLock(-- indent, "}\n");
    emitBodyLock(indent, "#endif\n");
  }
  mtActivationEventTraceSourceCppId = savedTraceSourceCppId;
}


int graph::genActivate(const std::string& subStepSuffix) {
    emitFuncDecl(0, "void S%s::subStep0%s() {\n", name.c_str(), subStepSuffix.c_str());
    int indent = 1;
    int nextSubStepIdx = 1;
    std::string nextFuncDef = format("void S%s::subStep%d%s()", name.c_str(), nextSubStepIdx, subStepSuffix.c_str());
    bool prevActiveWhole = false;
    for (int idx = 0; idx < superId; idx ++) {
      int id;
      uint64_t mask;
      std::tie(id, mask) = setIdxMask(idx);
      int offset = idx % ACTIVE_WIDTH;
      if (offset == 0) {
        if (prevActiveWhole) {
          emitBodyLock(--indent, "}\n");
        }
        prevActiveWhole = true;
        for (int j = 0; j < ACTIVE_WIDTH && idx + j < superId; j ++) {
          if (isAlwaysActive(idx + j)) prevActiveWhole = false;
        }
        if (prevActiveWhole) {
          bool newFile = __emitSrc(indent ++, true, false, nextFuncDef.c_str(), "if(unlikely(activeFlags[%d] != 0)) {\n", id);
          if (newFile) {
            nextFuncDef = format("void S%s::subStep%d%s()", name.c_str(), ++ nextSubStepIdx, subStepSuffix.c_str());
          }
          emitBodyLock(indent, "uint%d_t oldFlag = activeFlags[%d];\n", ACTIVE_WIDTH, id);
          emitBodyLock(indent, "activeFlags[%d] = 0;\n", id);
        }
      }
      SuperNode* super = cppId2Super[idx];
      std::string flagName = prevActiveWhole ? "oldFlag" : format("activeFlags[%d]", id);
      indent = genNodeStepStart(super, mask, idx, flagName, indent, false);
      static bool wallfracAudit = (std::getenv("GSIM_WALLFRAC_AUDIT") && std::string(std::getenv("GSIM_WALLFRAC_AUDIT")) == "1");
      bool wfCommit = (super->superType == SUPER_UPDATE_REG);
      for (Node* wfm : super->member) { if (nodeHasStateUpdate(wfm)) { wfCommit = true; break; } }
      if (wallfracAudit) {
        emitBodyLock(indent, "uint32_t __wf_a0_%d,__wf_d0_%d; __asm__ __volatile__(\"rdtsc\":\"=a\"(__wf_a0_%d),\"=d\"(__wf_d0_%d)); uint64_t __wf_t0_%d=((uint64_t)__wf_d0_%d<<32)|__wf_a0_%d;\n", idx, idx, idx, idx, idx, idx, idx);
      }
      genSuperEval(super, flagName, "", indent, true);
      if (wallfracAudit) {
        emitBodyLock(indent, "uint32_t __wf_a1_%d,__wf_d1_%d; __asm__ __volatile__(\"rdtsc\":\"=a\"(__wf_a1_%d),\"=d\"(__wf_d1_%d)); uint64_t __wf_t1_%d=((uint64_t)__wf_d1_%d<<32)|__wf_a1_%d;\n", idx, idx, idx, idx, idx, idx, idx);
        emitBodyLock(indent, "%s += __wf_t1_%d-__wf_t0_%d; %s ++;\n", wfCommit?"wallfracCommitCycles":"wallfracCombCycles", idx, idx, wfCommit?"wallfracCommitBrackets":"wallfracCombBrackets");
      }
      indent = genNodeStepEnd(super, indent, false);
    }
    emitBodyLock(--indent, "}\n");
    if (prevActiveWhole) emitBodyLock(--indent, "}\n");

    return nextSubStepIdx - 1; // return the maxinum subStepIdx currently used
}

void graph::genMtTaskHelper(SuperNode* super, bool buffered, const std::string& activeSinkType) {
  int savedTraceSourceCppId = mtActivationEventTraceSourceCppId;
  mtActivationEventTraceSourceCppId = super->cppId;
  if (buffered) {
    emitFuncDecl(0, "void S%s::mtTask%d(uint%d_t &flag, %s &nextActive) {\n", name.c_str(), super->cppId, ACTIVE_WIDTH, activeSinkType.c_str());
    genSuperEval(super, "flag", "nextActive", 1, true);
  } else {
    emitFuncDecl(0, "void S%s::mtTask%d(uint%d_t &flag) {\n", name.c_str(), super->cppId, ACTIVE_WIDTH);
    genSuperEval(super, "flag", "", 1, true);
  }
  emitBodyLock(0, "}\n");
  mtActivationEventTraceSourceCppId = savedTraceSourceCppId;
}

void graph::genMtTaskRunner(const MtPureBatchPlan& batchPlan) {
  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapForInvocation();
  const bool denseBreakdownWindowCodegen = mtUseDenseBreakdownProfileCodegen() && mtUseDenseBreakdownWindowCodegen();
  int shardCount = mtPureBatchShardCount();
  bool useCoarse = globalConfig.MtBatchFormationMode == "coarse";
  // Dense-only: pure-batch shard switch tables and mtRunPureBatchWorkerRange
  // are sparse-dispatch text (they call the dropped buffered mtTaskN helpers);
  // the worker pool core below stays because the dense executor posts jobKind
  // 6/7 jobs to it.
  const bool denseOnlyCodegen = mtUseDenseOnlyCodegen();
  [[maybe_unused]] auto emitPureTaskSwitchCases = [&](int shardBegin, int shardEnd, bool workerMode) {
    for (int cppId = shardBegin; cppId < shardEnd; cppId ++) {
      if (mtTasks[cppId].taskKind != "pure_compute") continue;
      emitBodyLock(4, "case %d:\n", cppId);
      if (workerMode) {
        emitBodyLock(5, "if (mtWorkerFlags[worker] & 0x%lx) {\n", (uint64_t)1 << (cppId % ACTIVE_WIDTH));
        emitBodyLock(6, "if (mtProfileEnabled) {\n");
        emitBodyLock(7, "mtTask%d(mtWorkerFlags[worker], mtWorkerDeltas[worker]);\n", cppId);
        emitBodyLock(7, "mtProfileLocalTaskIds[worker].push_back(%d);\n", cppId);
        emitBodyLock(7, "mtProfileLocalWorkerTaskCount[worker] ++;\n");
        emitBodyLock(6, "} else {\n");
        emitBodyLock(7, "mtTask%d(mtWorkerFlags[worker], mtWorkerDeltas[worker]);\n", cppId);
        emitBodyLock(6, "}\n");
        emitBodyLock(5, "}\n");
      } else {
        emitBodyLock(5, "if (activeWord & 0x%lx) {\n", (uint64_t)1 << (cppId % ACTIVE_WIDTH));
        emitBodyLock(6, "if (mtProfileEnabled) {\n");
        emitBodyLock(7, "std::chrono::steady_clock::time_point mtProfileTaskBegin = std::chrono::steady_clock::now();\n");
        emitBodyLock(7, "mtTask%d(activeWord);\n", cppId);
        emitBodyLock(7, "recordMtProfileTask(%d, true, std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileTaskBegin).count());\n", cppId);
        emitBodyLock(7, "mtProfileWorkerTaskCount[0] ++;\n");
        emitBodyLock(6, "} else {\n");
        emitBodyLock(7, "mtTask%d(activeWord);\n", cppId);
        emitBodyLock(6, "}\n");
        emitBodyLock(5, "}\n");
      }
      emitBodyLock(5, "break;\n");
    }
  };
  if (!denseOnlyCodegen) {
  for (int shard = 0; shard < shardCount; shard ++) {
    int shardBegin = shard * MT_PURE_BATCH_SHARD_SIZE;
    int shardEnd = std::min(superId, shardBegin + MT_PURE_BATCH_SHARD_SIZE);
    emitFuncDecl(0, "void S%s::mtRunPureBatchDirectShard%d(int chunkBegin, int chunkEnd, uint%d_t &activeWord) {\n",
                 name.c_str(), shard, ACTIVE_WIDTH);
    emitBodyLock(1, "if (chunkEnd <= %d || chunkBegin >= %d) return;\n", shardBegin, shardEnd);
    emitBodyLock(1, "int localBegin = std::max(chunkBegin, %d);\n", shardBegin);
    emitBodyLock(1, "int localEnd = std::min(chunkEnd, %d);\n", shardEnd);
    emitBodyLock(1, "for (int cppId = localBegin; cppId < localEnd; cppId ++) {\n");
    emitBodyLock(2, "switch (cppId) {\n");
    emitPureTaskSwitchCases(shardBegin, shardEnd, false);
    emitBodyLock(3, "default:\n");
    emitBodyLock(4, "break;\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(0, "}\n");

    emitFuncDecl(0, "void S%s::mtRunPureBatchWorkerShard%d(int worker, int chunkBegin, int chunkEnd, std::vector<std::vector<int>> &mtProfileLocalTaskIds, std::vector<uint64_t> &mtProfileLocalWorkerTaskCount) {\n",
                 name.c_str(), shard);
    emitBodyLock(1, "if (chunkEnd <= %d || chunkBegin >= %d) return;\n", shardBegin, shardEnd);
    emitBodyLock(1, "int localBegin = std::max(chunkBegin, %d);\n", shardBegin);
    emitBodyLock(1, "int localEnd = std::min(chunkEnd, %d);\n", shardEnd);
    emitBodyLock(1, "for (int cppId = localBegin; cppId < localEnd; cppId ++) {\n");
    emitBodyLock(2, "switch (cppId) {\n");
    emitPureTaskSwitchCases(shardBegin, shardEnd, true);
    emitBodyLock(3, "default:\n");
    emitBodyLock(4, "break;\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(0, "}\n");
  }
  emitFuncDecl(0, "void S%s::mtRunPureBatchWorkerRange(int worker, int chunkBegin, int chunkEnd) {\n", name.c_str());
  emitBodyLock(1, "if (chunkEnd <= chunkBegin) return;\n");
  emitBodyLock(1, "int firstShard = chunkBegin / %d;\n", MT_PURE_BATCH_SHARD_SIZE);
  emitBodyLock(1, "int lastShard = (chunkEnd - 1) / %d;\n", MT_PURE_BATCH_SHARD_SIZE);
  emitBodyLock(1, "for (int shard = firstShard; shard <= lastShard; shard ++) {\n");
  emitBodyLock(2, "switch (shard) {\n");
  for (int shard = 0; shard < shardCount; shard ++) {
    emitBodyLock(3, "case %d:\n", shard);
    emitBodyLock(4, "mtRunPureBatchWorkerShard%d(worker, chunkBegin, chunkEnd, mtProfileLocalTaskIds, mtProfileLocalWorkerTaskCount);\n", shard);
    emitBodyLock(4, "break;\n");
  }
  emitBodyLock(3, "default:\n");
  emitBodyLock(4, "break;\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(0, "}\n");
  }

  emitFuncDecl(0, "void S%s::mtWorkerPoolPause() {\n", name.c_str());
  emitBodyLock(1, "#if defined(__x86_64__) || defined(__i386__)\n");
  emitBodyLock(1, "__asm__ __volatile__(\"pause\" ::: \"memory\");\n");
  emitBodyLock(1, "#else\n");
  emitBodyLock(1, "std::this_thread::yield();\n");
  emitBodyLock(1, "#endif\n");
  emitBodyLock(0, "}\n");

  // Worker-pool job payload is published by mtWorkerPoolPost()'s release
  // increment of mtWorkerPoolGeneration and consumed after workers acquire the
  // new generation. Every spawned background worker acknowledges each generation,
  // including inactive high IDs, before the main thread reuses payload fields.
  emitFuncDecl(0, "void S%s::mtWorkerPoolPost() {\n", name.c_str());
  if (mtUseWorkerPoolFlagJoinCodegen()) {
    emitBodyLock(1, "#if !defined(GSIM_MT_WORKER_POOL_FLAG_JOIN_COMPILE) || !GSIM_MT_WORKER_POOL_FLAG_JOIN_COMPILE\n");
    emitBodyLock(1, "mtWorkerPoolDoneCount.store(0, std::memory_order_relaxed);\n");
    emitBodyLock(1, "#endif\n");
  } else {
    emitBodyLock(1, "mtWorkerPoolDoneCount.store(0, std::memory_order_relaxed);\n");
  }
  if (denseBreakdownWindowCodegen) {
    emitBodyLock(1, "if (unlikely(mtDenseBreakdownWindowEnabled && mtWorkerPoolJobKind == 7 && cycles >= mtDenseBreakdownWindowStart && cycles - mtDenseBreakdownWindowStart < mtDenseBreakdownWindowCycles)) {\n");
    emitBodyLock(2, "if (unlikely(mtDenseBreakdownWindowRecordedCycles >= mtDenseBreakdownWindowCycles || mtDenseBreakdownWindowRecordedCycles >= kDenseBreakdownWindowMaxCycles)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window cycle record overflow\\n\"); abort(); }\n");
    emitBodyLock(2, "mtDenseBreakdownWindowCurrentSlot = (int)mtDenseBreakdownWindowRecordedCycles;\n");
    emitBodyLock(2, "mtDenseBreakdownWindowCycleNumbers[mtDenseBreakdownWindowCurrentSlot] = cycles;\n");
    emitBodyLock(2, "mtDenseBreakdownWindowEpoch = std::chrono::steady_clock::now();\n");
    emitBodyLock(2, "if (unlikely(mtDenseBreakdownWindowCausalChainMode)) { for (int token = 0; token < kDenseBreakdownWindowCausalTokenCount; token ++) { mtDenseBreakdownWindowReadyTokens[mtDenseBreakdownWindowCurrentSlot][token].releaseBeforeOffsetNs = UINT64_MAX; mtDenseBreakdownWindowReadyTokens[mtDenseBreakdownWindowCurrentSlot][token].releaseAfterOffsetNs = UINT64_MAX; } }\n");
    emitBodyLock(2, "mtDenseBreakdownWindowRecordedCycles += 1;\n");
    emitBodyLock(1, "} else {\n");
    emitBodyLock(2, "mtDenseBreakdownWindowCurrentSlot = -1;\n");
    emitBodyLock(1, "}\n");
  }
  emitBodyLock(1, "mtWorkerPoolGeneration.fetch_add(1, std::memory_order_release);\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::mtWorkerPoolWaitForDone(int expectedDoneCount) {\n", name.c_str());
  emitBodyLock(1, "expectedDoneCount = mtWorkerPoolThreadCount;\n");
  if (mtUseWorkerPoolFlagJoinCodegen()) {
    emitBodyLock(1, "#if defined(GSIM_MT_WORKER_POOL_FLAG_JOIN_COMPILE) && GSIM_MT_WORKER_POOL_FLAG_JOIN_COMPILE\n");
    emitBodyLock(1, "const uint8_t expectedParity = static_cast<uint8_t>(mtWorkerPoolGeneration.load(std::memory_order_acquire) & 1);\n");
    emitBodyLock(1, "while (true) {\n");
    emitBodyLock(2, "bool allDone = true;\n");
    emitBodyLock(2, "for (int worker = 0; worker < expectedDoneCount; worker ++) {\n");
    emitBodyLock(3, "if (mtWorkerPoolDoneFlags[(size_t)worker].parity.load(std::memory_order_acquire) != expectedParity) { allDone = false; break; }\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(2, "if (allDone) break;\n");
    emitBodyLock(2, "mtWorkerPoolPause();\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(1, "#else\n");
  }
  emitBodyLock(1, "while (mtWorkerPoolDoneCount.load(std::memory_order_acquire) < expectedDoneCount) {\n");
  emitBodyLock(2, "mtWorkerPoolPause();\n");
  emitBodyLock(1, "}\n");
  if (mtUseWorkerPoolFlagJoinCodegen()) emitBodyLock(1, "#endif\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::mtWorkerPoolLoop(int worker) {\n", name.c_str());
  // Workers publish readiness before startMtWorkerPool returns, so the first
  // post cannot race a late-start worker that has not captured the baseline generation.
  emitBodyLock(1, "uint64_t seenGeneration = mtWorkerPoolGeneration.load(std::memory_order_acquire);\n");
  emitBodyLock(1, "mtWorkerPoolReadyCount.fetch_add(1, std::memory_order_release);\n");
  emitBodyLock(1, "while (true) {\n");
  emitBodyLock(2, "uint64_t generation = seenGeneration;\n");
  if (mtDenseDutyCodegen()) emitBodyLock(2, "std::chrono::steady_clock::time_point mtDutySpinBegin; if (mtDutyEnabled) mtDutySpinBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(2, "while (true) {\n");
  emitBodyLock(3, "if (mtWorkerPoolStop.load(std::memory_order_acquire)) return;\n");
  emitBodyLock(3, "generation = mtWorkerPoolGeneration.load(std::memory_order_acquire);\n");
  emitBodyLock(3, "if (generation != seenGeneration) break;\n");
  emitBodyLock(3, "mtWorkerPoolPause();\n");
  emitBodyLock(2, "}\n");
  if (mtDenseDutyCodegen()) emitBodyLock(2, "if (mtDutyEnabled && worker >= 1 && worker <= kDenseDutyLaneMax) mtDutyLanes[worker].spinNs += (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDutySpinBegin).count();\n");
  emitBodyLock(2, "seenGeneration = generation;\n");
  emitBodyLock(2, "if (mtWorkerPoolStop.load(std::memory_order_acquire)) return;\n");
  emitBodyLock(2, "const int workerCount = mtWorkerPoolCurrentWorkerCount;\n");
  emitBodyLock(2, "if (worker >= workerCount) {\n");
  if (mtUseWorkerPoolFlagJoinCodegen()) {
    emitBodyLock(3, "#if defined(GSIM_MT_WORKER_POOL_FLAG_JOIN_COMPILE) && GSIM_MT_WORKER_POOL_FLAG_JOIN_COMPILE\n");
    emitBodyLock(3, "mtWorkerPoolDoneFlags[(size_t)(worker - 1)].parity.store(static_cast<uint8_t>(seenGeneration & 1), std::memory_order_release);\n");
    emitBodyLock(3, "#else\n");
    emitBodyLock(3, "mtWorkerPoolDoneCount.fetch_add(1, std::memory_order_release);\n");
    emitBodyLock(3, "#endif\n");
  } else {
    emitBodyLock(3, "mtWorkerPoolDoneCount.fetch_add(1, std::memory_order_release);\n");
  }
  emitBodyLock(3, "continue;\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "const int chunkBegin = mtWorkerPoolChunks[(size_t)worker].begin;\n");
  emitBodyLock(2, "const int chunkEnd = mtWorkerPoolChunks[(size_t)worker].end;\n");
  emitBodyLock(2, "const int jobKind = mtWorkerPoolJobKind;\n");
  if (denseOnlyCodegen) {
    // Dense-only: the pool only ever serves dense jobs (kind 6 dense layer,
    // kind 7 dense thread worker). The pure-batch (0) and coarse (1/2/3/5)
    // sparse job kinds have no emitted handlers.
    emitBodyLock(2, "if (jobKind == 6) {\n");
    emitBodyLock(3, "(this->*mtWorkerPoolDenseLayerFn)(worker, chunkBegin, chunkEnd);\n");
    emitBodyLock(2, "} else if (jobKind == 7) {\n");
    emitBodyLock(3, "stepDenseThreadWorker(worker);\n");
    emitBodyLock(2, "}\n");
  } else if (useCoarse) {
    emitBodyLock(2, "const int coarseRegionIndex = mtWorkerPoolCoarseRegionIndex;\n");
    emitBodyLock(2, "const int coarseLayerIndex = mtWorkerPoolCoarseLayerIndex;\n");
    emitBodyLock(2, "if (jobKind == 1) {\n");
    emitBodyLock(3, "mtRunCoarseLayerWorkerRange(worker, coarseRegionIndex, coarseLayerIndex, chunkBegin, chunkEnd);\n");
    emitBodyLock(2, "} else if (jobKind == 2) {\n");
    if (globalConfig.MtCoarseWorkerPolicyMode == "profitable") {
      emitBodyLock(3, "mtRunCoarseMTaskWorkerList(worker, coarseRegionIndex, mtWorkerPoolMTaskAssignments[(size_t)worker].data(), (int)mtWorkerPoolMTaskAssignments[(size_t)worker].size());\n");
    } else {
      emitBodyLock(3, "mtRunCoarseMTaskWorkerRange(worker, coarseRegionIndex, chunkBegin, chunkEnd);\n");
    }
    emitBodyLock(2, "} else if (jobKind == 3) {\n");
    // pool worker dispatched into the codegen-time
    // flat-array path. Region/wc/begin/span carried on dedicated fields
    // so we don't overload chunk[].begin/.end semantics.
    emitBodyLock(3, "mtRunCoarseRegionStaticDispatch(coarseRegionIndex, mtWorkerPoolCoarseStaticRoundedWC, worker, mtWorkerPoolCoarseStaticBeginActiveWord, mtWorkerPoolCoarseStaticActiveWordSpan);\n");
    emitBodyLock(2, "} else if (jobKind == 4) {\n");
    emitBodyLock(3, "/* A35-P empty-barrier microbench: worker performs no work */\n");
    emitBodyLock(2, "} else if (jobKind == 5) {\n");
    emitBodyLock(3, "mtRunCoarseMTaskDynamic(coarseRegionIndex, worker);\n");
    emitBodyLock(2, "} else if (jobKind == 6) {\n");
    emitBodyLock(3, "(this->*mtWorkerPoolDenseLayerFn)(worker, chunkBegin, chunkEnd);\n");
    emitBodyLock(2, "} else if (jobKind == 7) {\n");
    emitBodyLock(3, "stepDenseThreadWorker(worker);\n");
    emitBodyLock(2, "} else {\n");
    emitBodyLock(3, "mtRunPureBatchWorkerRange(worker, chunkBegin, chunkEnd);\n");
    emitBodyLock(2, "}\n");
  } else {
    emitBodyLock(2, "if (jobKind == 6) {\n");
    emitBodyLock(3, "(this->*mtWorkerPoolDenseLayerFn)(worker, chunkBegin, chunkEnd);\n");
    emitBodyLock(2, "} else if (jobKind == 7) {\n");
    emitBodyLock(3, "stepDenseThreadWorker(worker);\n");
    emitBodyLock(2, "} else {\n");
    emitBodyLock(3, "mtRunPureBatchWorkerRange(worker, chunkBegin, chunkEnd);\n");
    emitBodyLock(2, "}\n");
  }
  if (mtUseWorkerPoolFlagJoinCodegen()) {
    emitBodyLock(2, "#if defined(GSIM_MT_WORKER_POOL_FLAG_JOIN_COMPILE) && GSIM_MT_WORKER_POOL_FLAG_JOIN_COMPILE\n");
    emitBodyLock(2, "mtWorkerPoolDoneFlags[(size_t)(worker - 1)].parity.store(static_cast<uint8_t>(seenGeneration & 1), std::memory_order_release);\n");
    emitBodyLock(2, "#else\n");
    emitBodyLock(2, "mtWorkerPoolDoneCount.fetch_add(1, std::memory_order_release);\n");
    emitBodyLock(2, "#endif\n");
  } else {
    emitBodyLock(2, "mtWorkerPoolDoneCount.fetch_add(1, std::memory_order_release);\n");
  }
  emitBodyLock(1, "}\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::startMtWorkerPool() {\n", name.c_str());
  emitBodyLock(1, "if (!mtWorkerPoolEnabled || mtConfiguredWorkerCount <= 1 || !mtWorkerPoolThreads.empty()) return;\n");
  emitBodyLock(1, "// Logical worker 0 is the main thread; spawn N-1 background workers (logical IDs 1..N-1).\n");
  emitBodyLock(1, "mtWorkerPoolThreadCount = mtConfiguredWorkerCount - 1;\n");
  emitBodyLock(1, "mtWorkerPoolReadyCount.store(0, std::memory_order_relaxed);\n");
  emitBodyLock(1, "mtWorkerPoolChunks.assign((size_t)mtConfiguredWorkerCount, MtWorkerPoolChunk{0, 0});\n");
  if (mtUseWorkerPoolFlagJoinCodegen()) {
    emitBodyLock(1, "#if defined(GSIM_MT_WORKER_POOL_FLAG_JOIN_COMPILE) && GSIM_MT_WORKER_POOL_FLAG_JOIN_COMPILE\n");
    emitBodyLock(1, "delete[] mtWorkerPoolDoneFlags;\n");
    emitBodyLock(1, "mtWorkerPoolDoneFlags = new MtWorkerPoolDoneFlag[(size_t)mtWorkerPoolThreadCount];\n");
    emitBodyLock(1, "for (int worker = 0; worker < mtWorkerPoolThreadCount; worker ++) mtWorkerPoolDoneFlags[(size_t)worker].parity.store(0, std::memory_order_relaxed);\n");
    emitBodyLock(1, "#endif\n");
  }
  emitBodyLock(1, "mtWorkerDeltas.resize((size_t)mtConfiguredWorkerCount);\n");
  emitBodyLock(1, "mtWorkerFlags.resize((size_t)mtConfiguredWorkerCount);\n");
  if (useCoarse) {
    emitBodyLock(1, "mtWorkerCoarseFlags.resize((size_t)mtConfiguredWorkerCount);\n");
    if (globalConfig.MtCoarseWorkerPolicyMode == "profitable") {
      emitBodyLock(1, "mtWorkerPoolMTaskAssignments.resize((size_t)mtConfiguredWorkerCount);\n");
    }
    // per-region atomic state for antichain runtime is initialized
    // in initMtProfile() so it is available even when the worker pool is disabled
    // or only one thread is used.
    emitBodyLock(1, "mtWorkerPoolCoarseActiveWords = nullptr;\n");
  }
  emitBodyLock(1, "mtWorkerPoolThreads.reserve((size_t)mtWorkerPoolThreadCount);\n");
  emitBodyLock(1, "const char *mtCpuAffinityEnv = getenv(\"GSIM_MT_CPU_AFFINITY\");\n");
  emitBodyLock(1, "int mtCpuAffinityBase = -1;\n");
  emitBodyLock(1, "if (mtCpuAffinityEnv != nullptr && mtCpuAffinityEnv[0] != '\\0') {\n");
  emitBodyLock(2, "if (mtCpuAffinityEnv[0] == 'a' || mtCpuAffinityEnv[0] == 'A') mtCpuAffinityBase = 0;\n");
  emitBodyLock(2, "else mtCpuAffinityBase = atoi(mtCpuAffinityEnv);\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "std::vector<int> mtAllowedCpus;\n");
  emitBodyLock(1, "#ifdef __linux\n");
  emitBodyLock(1, "if (mtCpuAffinityBase >= 0) {\n");
  emitBodyLock(2, "cpu_set_t mtAllowedSet;\n");
  emitBodyLock(2, "CPU_ZERO(&mtAllowedSet);\n");
  emitBodyLock(2, "if (sched_getaffinity(0, sizeof(mtAllowedSet), &mtAllowedSet) == 0) {\n");
  emitBodyLock(3, "for (int mtCpu = 0; mtCpu < CPU_SETSIZE; mtCpu ++) {\n");
  emitBodyLock(4, "if (CPU_ISSET(mtCpu, &mtAllowedSet)) mtAllowedCpus.push_back(mtCpu);\n");
  emitBodyLock(3, "}\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "#endif\n");
  // Explicit owner->CPU map (default-off codegen + runtime env): a comma-separated
  // CPU id per logical worker. Unset emits no map text at all; any parse,
  // validation, or affinity failure aborts; there is no silent fallback.
  if (mtUseOwnerCpuMapCodegen()) {
  emitBodyLock(1, "std::vector<int> mtOwnerCpuMap;\n");
  emitBodyLock(1, "#ifdef __linux\n");
  emitBodyLock(1, "const char *mtOwnerCpuMapEnv = getenv(\"GSIM_MT_OWNER_CPU_MAP\");\n");
  emitBodyLock(1, "if (mtOwnerCpuMapEnv != nullptr && mtOwnerCpuMapEnv[0] != '\\0') {\n");
  emitBodyLock(2, "const char *mtMapPtr = mtOwnerCpuMapEnv;\n");
  emitBodyLock(2, "while (*mtMapPtr != '\\0') {\n");
  emitBodyLock(3, "char *mtMapEnd = nullptr;\n");
  emitBodyLock(3, "long mtMapCpu = strtol(mtMapPtr, &mtMapEnd, 10);\n");
  emitBodyLock(3, "if (mtMapEnd == mtMapPtr || mtMapCpu < 0 || mtMapCpu >= CPU_SETSIZE) { fprintf(stderr, \"[mt-owner-cpu-map] invalid cpu id in GSIM_MT_OWNER_CPU_MAP\\n\"); abort(); }\n");
  emitBodyLock(3, "mtOwnerCpuMap.push_back((int)mtMapCpu);\n");
  emitBodyLock(3, "mtMapPtr = (*mtMapEnd == ',') ? mtMapEnd + 1 : mtMapEnd;\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "if ((int)mtOwnerCpuMap.size() != mtConfiguredWorkerCount) { fprintf(stderr, \"[mt-owner-cpu-map] got %%d cpus, need %%d workers\\n\", (int)mtOwnerCpuMap.size(), mtConfiguredWorkerCount); abort(); }\n");
  emitBodyLock(2, "for (size_t mi = 0; mi < mtOwnerCpuMap.size(); mi ++) for (size_t mj = mi + 1; mj < mtOwnerCpuMap.size(); mj ++) {\n");
  emitBodyLock(3, "if (mtOwnerCpuMap[mi] == mtOwnerCpuMap[mj]) { fprintf(stderr, \"[mt-owner-cpu-map] duplicate cpu %%d\\n\", mtOwnerCpuMap[mi]); abort(); }\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "if (!mtAllowedCpus.empty()) {\n");
  emitBodyLock(3, "for (int mtMapCpu : mtOwnerCpuMap) {\n");
  emitBodyLock(4, "bool mtMapAllowed = false; for (int mtAc : mtAllowedCpus) if (mtAc == mtMapCpu) mtMapAllowed = true;\n");
  emitBodyLock(4, "if (!mtMapAllowed) { fprintf(stderr, \"[mt-owner-cpu-map] cpu %%d outside allowed affinity set\\n\", mtMapCpu); abort(); }\n");
  emitBodyLock(3, "}\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "cpu_set_t mtOwnerMainSet;\n");
  emitBodyLock(2, "CPU_ZERO(&mtOwnerMainSet);\n");
  emitBodyLock(2, "CPU_SET(mtOwnerCpuMap[0], &mtOwnerMainSet);\n");
  emitBodyLock(2, "if (sched_setaffinity(0, sizeof(mtOwnerMainSet), &mtOwnerMainSet) != 0) { fprintf(stderr, \"[mt-owner-cpu-map] pin main worker 0 to cpu %%d failed\\n\", mtOwnerCpuMap[0]); abort(); }\n");
  emitBodyLock(2, "fprintf(stderr, \"[mt-owner-cpu-map] applied explicit map (%%d workers, main -> cpu %%d)\\n\", mtConfiguredWorkerCount, mtOwnerCpuMap[0]);\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "#endif\n");
  }
  emitBodyLock(1, "for (int worker = 1; worker < mtConfiguredWorkerCount; worker ++) {\n");
  emitBodyLock(2, "mtWorkerPoolThreads.emplace_back([this, worker]() { mtWorkerPoolLoop(worker); });\n");
  emitBodyLock(2, "std::thread& t = mtWorkerPoolThreads.back();\n");
  emitBodyLock(2, "#ifdef __linux\n");
  if (mtUseOwnerCpuMapCodegen()) {
    emitBodyLock(2, "if (!mtOwnerCpuMap.empty() || !mtAllowedCpus.empty()) {\n");
    emitBodyLock(3, "int mtCpu = !mtOwnerCpuMap.empty() ? mtOwnerCpuMap[(size_t)worker] : mtAllowedCpus[(mtCpuAffinityBase + worker - 1) % (int)mtAllowedCpus.size()];\n");
    emitBodyLock(3, "cpu_set_t mtCpuset;\n");
    emitBodyLock(3, "CPU_ZERO(&mtCpuset);\n");
    emitBodyLock(3, "CPU_SET(mtCpu, &mtCpuset);\n");
    emitBodyLock(3, "if (pthread_setaffinity_np(t.native_handle(), sizeof(mtCpuset), &mtCpuset) != 0 && !mtOwnerCpuMap.empty()) { fprintf(stderr, \"[mt-owner-cpu-map] pin worker %%d to cpu %%d failed\\n\", worker, mtCpu); abort(); }\n");
    emitBodyLock(2, "}\n");
  } else {
    emitBodyLock(2, "if (!mtAllowedCpus.empty()) {\n");
    emitBodyLock(3, "int mtCpu = mtAllowedCpus[(mtCpuAffinityBase + worker - 1) % (int)mtAllowedCpus.size()];\n");
    emitBodyLock(3, "cpu_set_t mtCpuset;\n");
    emitBodyLock(3, "CPU_ZERO(&mtCpuset);\n");
    emitBodyLock(3, "CPU_SET(mtCpu, &mtCpuset);\n");
    emitBodyLock(3, "pthread_setaffinity_np(t.native_handle(), sizeof(mtCpuset), &mtCpuset);\n");
    emitBodyLock(2, "}\n");
  }
  emitBodyLock(2, "#endif\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "while (mtWorkerPoolReadyCount.load(std::memory_order_acquire) < mtWorkerPoolThreadCount) mtWorkerPoolPause();\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::stopMtWorkerPool() {\n", name.c_str());
  emitBodyLock(1, "if (mtWorkerPoolThreads.empty()) return;\n");
  emitBodyLock(1, "mtWorkerPoolStop.store(true, std::memory_order_release);\n");
  emitBodyLock(1, "mtWorkerPoolGeneration.fetch_add(1, std::memory_order_release);\n");
  emitBodyLock(1, "for (std::thread &worker : mtWorkerPoolThreads) {\n");
  emitBodyLock(2, "if (worker.joinable()) worker.join();\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "mtWorkerPoolThreads.clear();\n");
  emitBodyLock(0, "}\n");

  // Dense-only: mtRunPureBatch is the sparse pure-batch dispatcher; its only
  // callers are the plain subStepN() scan, which is not emitted.
  if (!denseOnlyCodegen) {
  emitFuncDecl(0, "void S%s::mtRunPureBatch(int beginCppId, int endCppId, uint%d_t &activeWord) {\n", name.c_str(), ACTIVE_WIDTH);
  emitBodyLock(1, "int taskCount = endCppId - beginCppId;\n");
  emitBodyLock(1, "if (taskCount <= 0) return;\n");
  emitBodyLock(1, "std::chrono::steady_clock::time_point mtProfileBatchBegin;\n");
  emitBodyLock(1, "if (mtProfileEnabled) mtProfileBatchBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(1, "int workerCount = mtConfiguredWorkerCount;\n");
  emitBodyLock(1, "bool mtSkippedBelowMinBatch = false;\n");
  emitBodyLock(1, "if (taskCount < mtMinBatchTasks) {\n");
  emitBodyLock(2, "if (mtProfileEnabled) mtProfileRejectBelowMinBatch ++;\n");
  emitBodyLock(2, "mtSkippedBelowMinBatch = true;\n");
  emitBodyLock(2, "workerCount = 1;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "if (workerCount > taskCount) workerCount = taskCount;\n");
  emitBodyLock(1, "if (workerCount < 2) workerCount = 1;\n");
  emitBodyLock(1, "if (mtProfileEnabled) {\n");
  emitBodyLock(2, "int batchSizeBucket = taskCount <= 1 ? 0 : (taskCount == 2 ? 1 : (taskCount <= 4 ? 2 : (taskCount <= 8 ? 3 : (taskCount <= 15 ? 4 : 5))));\n");
  emitBodyLock(2, "mtProfileBatchSizeHist[batchSizeBucket] ++;\n");
  emitBodyLock(2, "mtProfilePureBatchCount ++;\n");
  emitBodyLock(1, "}\n");
  if (!batchPlan.batches.empty()) {
    emitBodyLock(1, "if (mtProfileEnabled) {\n");
    emitBodyLock(2, "switch (beginCppId) {\n");
    for (auto batch : batchPlan.batches) {
      int memberNodeCount = 0;
      int sameActiveWordForwardEdges = 0;
      int crossBatchActivationFanout = 0;
      for (int cppId = batch.first; cppId < batch.second; cppId ++) {
        SuperNode* super = cppId2Super[cppId];
        memberNodeCount += (int)super->member.size();
        for (Node* member : super->member) {
          for (int activeId : member->nextActiveId) {
            if (activeId >= batch.first && activeId < batch.second && activeId > cppId &&
                activeId / ACTIVE_WIDTH == cppId / ACTIVE_WIDTH) sameActiveWordForwardEdges ++;
            if (activeId >= 0 && (activeId < batch.first || activeId >= batch.second)) crossBatchActivationFanout ++;
          }
        }
      }
      emitBodyLock(3, "case %d:\n", batch.first);
      emitBodyLock(4, "mtProfileBatchMemberNodeCount += %d;\n", memberNodeCount);
      emitBodyLock(4, "mtProfileSameActiveWordForwardEdges += %d;\n", sameActiveWordForwardEdges);
      emitBodyLock(4, "mtProfileCrossBatchActivationFanout += %d;\n", crossBatchActivationFanout);
      emitBodyLock(4, "break;\n");
    }
    emitBodyLock(3, "default:\n");
    emitBodyLock(4, "break;\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(1, "}\n");
  }
  emitBodyLock(1, "if (mtProfileEnabled && mtProfileWorkerTaskCount.size() < (size_t)workerCount) mtProfileWorkerTaskCount.resize((size_t)workerCount, 0);\n");
  emitBodyLock(1, "if (workerCount == 1) {\n");
  emitBodyLock(2, "if (mtProfileEnabled) {\n");
  emitBodyLock(3, "mtProfileSkippedFakeParallelBatchCount ++;\n");
  emitBodyLock(3, "if (mtProfileEffectiveWorkerCountHist.size() <= 1) mtProfileEffectiveWorkerCountHist.resize(2, 0);\n");
  emitBodyLock(3, "mtProfileEffectiveWorkerCountHist[1] ++;\n");
  emitBodyLock(3, "if (!mtSkippedBelowMinBatch) mtProfileRejectConfiguredSingleWorker ++;\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "int firstShard = beginCppId / %d;\n", MT_PURE_BATCH_SHARD_SIZE);
  emitBodyLock(2, "int lastShard = (endCppId - 1) / %d;\n", MT_PURE_BATCH_SHARD_SIZE);
  emitBodyLock(2, "for (int shard = firstShard; shard <= lastShard; shard ++) {\n");
  emitBodyLock(3, "switch (shard) {\n");
  for (int shard = 0; shard < shardCount; shard ++) {
    emitBodyLock(4, "case %d:\n", shard);
    emitBodyLock(5, "mtRunPureBatchDirectShard%d(beginCppId, endCppId, activeWord);\n", shard);
    emitBodyLock(5, "break;\n");
  }
  emitBodyLock(4, "default:\n");
  emitBodyLock(5, "break;\n");
  emitBodyLock(3, "}\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "if (mtProfileEnabled) mtProfileBatchWallNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileBatchBegin).count();\n");
  emitBodyLock(2, "return;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "if (mtProfileEnabled) {\n");
  emitBodyLock(2, "mtProfileTrueParallelBatchCount ++;\n");
  emitBodyLock(2, "if (workerCount > mtProfileMaxWorkerCount) mtProfileMaxWorkerCount = workerCount;\n");
  emitBodyLock(2, "if (mtProfileEffectiveWorkerCountHist.size() <= (size_t)workerCount) mtProfileEffectiveWorkerCountHist.resize((size_t)workerCount + 1, 0);\n");
  emitBodyLock(2, "mtProfileEffectiveWorkerCountHist[(size_t)workerCount] ++;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "if (mtProfileEnabled) {\n");
  emitBodyLock(2, "mtProfileLocalWorkerTaskCount.assign((size_t)workerCount, 0);\n");
  emitBodyLock(2, "if (mtProfileLocalTaskIds.size() < (size_t)workerCount) mtProfileLocalTaskIds.resize((size_t)workerCount);\n");
  emitBodyLock(2, "for (int worker = 0; worker < workerCount; worker ++) mtProfileLocalTaskIds[worker].clear();\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "if (mtWorkerDeltas.size() < (size_t)workerCount) mtWorkerDeltas.resize((size_t)workerCount);\n");
  emitBodyLock(1, "for (int worker = 0; worker < workerCount; worker ++) mtWorkerDeltas[worker].clear();\n");
  emitBodyLock(1, "if (mtWorkerFlags.size() < (size_t)workerCount) mtWorkerFlags.resize((size_t)workerCount);\n");
  emitBodyLock(1, "for (int worker = 0; worker < workerCount; worker ++) mtWorkerFlags[worker] = activeWord;\n");
  emitBodyLock(1, "if (workerCount == 1) {\n");
  emitBodyLock(2, "mtRunPureBatchWorkerRange(0, beginCppId, endCppId);\n");
  emitBodyLock(1, "} else if (mtWorkerPoolEnabled && mtWorkerPoolThreadCount + 1 >= workerCount) {\n");
  emitBodyLock(2, "mtWorkerPoolJobKind = 0;\n");
  emitBodyLock(2, "mtWorkerPoolCurrentWorkerCount = workerCount;\n");
  emitBodyLock(2, "for (int worker = 0; worker < workerCount; worker ++) {\n");
  emitBodyLock(3, "mtWorkerPoolChunks[(size_t)worker].begin = beginCppId + (taskCount * worker) / workerCount;\n");
  emitBodyLock(3, "mtWorkerPoolChunks[(size_t)worker].end = beginCppId + (taskCount * (worker + 1)) / workerCount;\n");
  emitBodyLock(2, "}\n");
  if (useCoarse) {
    emitBodyLock(2, "std::chrono::steady_clock::time_point mtPhaseBodyBegin;\n");
    emitBodyLock(2, "if (mtProfileEnabled) mtPhaseBodyBegin = std::chrono::steady_clock::now();\n");
  }
  emitBodyLock(2, "mtWorkerPoolPost();\n");
  emitBodyLock(2, "mtRunPureBatchWorkerRange(0, mtWorkerPoolChunks[0].begin, mtWorkerPoolChunks[0].end);\n");
  if (useCoarse) {
    emitBodyLock(2, "std::chrono::steady_clock::time_point mtPhaseWaitBegin;\n");
    emitBodyLock(2, "if (mtProfileEnabled) {\n");
    emitBodyLock(3, "mtPhaseWaitBegin = std::chrono::steady_clock::now();\n");
    emitBodyLock(3, "mtProfileCoarseBodyNs += std::chrono::duration_cast<std::chrono::nanoseconds>(mtPhaseWaitBegin - mtPhaseBodyBegin).count();\n");
    emitBodyLock(2, "}\n");
  }
  emitBodyLock(2, "mtWorkerPoolWaitForDone(workerCount - 1);\n");
  if (useCoarse) {
    emitBodyLock(2, "if (mtProfileEnabled) mtProfileCoarseWaitNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtPhaseWaitBegin).count();\n");
  }
  emitBodyLock(1, "} else {\n");
  emitBodyLock(2, "std::vector<std::thread> workers;\n");
  emitBodyLock(2, "workers.reserve(workerCount);\n");
  emitBodyLock(2, "for (int worker = 0; worker < workerCount; worker ++) {\n");
  emitBodyLock(3, "int chunkBegin = beginCppId + (taskCount * worker) / workerCount;\n");
  emitBodyLock(3, "int chunkEnd = beginCppId + (taskCount * (worker + 1)) / workerCount;\n");
  emitBodyLock(3, "workers.emplace_back([&, worker, chunkBegin, chunkEnd]() { mtRunPureBatchWorkerRange(worker, chunkBegin, chunkEnd); });\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "for (std::thread &worker : workers) worker.join();\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "std::chrono::steady_clock::time_point mtProfileMergeBegin;\n");
  emitBodyLock(1, "if (mtProfileEnabled) mtProfileMergeBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(1, "for (int worker = 0; worker < workerCount; worker ++) activeWord |= mtWorkerFlags[worker];\n");
  emitBodyLock(1, "for (int worker = 0; worker < workerCount; worker ++) mtWorkerDeltas[worker].mergeInto(activeFlags);\n");
  emitBodyLock(1, "if (mtProfileEnabled) {\n");
  emitBodyLock(2, "for (int worker = 0; worker < workerCount; worker ++) {\n");
  emitBodyLock(3, "mtProfileActivationDeltaEntries += mtWorkerDeltas[worker].entries.size();\n");
  emitBodyLock(3, "if (mtWorkerDeltas[worker].entries.size() > mtProfileActivationDeltaMaxEntriesPerWorker) mtProfileActivationDeltaMaxEntriesPerWorker = mtWorkerDeltas[worker].entries.size();\n");
  emitBodyLock(3, "if (mtWorkerDeltas[worker].allActive) mtProfileActivationDeltaActivateAllCount ++;\n");
  emitBodyLock(3, "mtProfileWorkerTaskCount[(size_t)worker] += mtProfileLocalWorkerTaskCount[worker];\n");
  emitBodyLock(3, "mtProfilePureTasks += mtProfileLocalWorkerTaskCount[worker];\n");
  emitBodyLock(3, "bool mtTraceCycleActive = mtProfileDynamicTraceFile != nullptr && cycles >= mtProfileDynamicTraceCycleStart && cycles < mtProfileDynamicTraceCycleLimit;\n");
  emitBodyLock(3, "for (int cppId : mtProfileLocalTaskIds[worker]) { if (cppId >= 0 && cppId < %d) { mtProfileTaskExecCount[cppId] ++; if (mtTraceCycleActive) mtProfileDynamicTraceTaskIds.push_back(cppId); } }\n", superId);
  emitBodyLock(2, "}\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "if (mtProfileEnabled) mtProfileMergeWallNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileMergeBegin).count();\n");
  emitBodyLock(1, "if (mtProfileEnabled) mtProfileBatchWallNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileBatchBegin).count();\n");
  emitBodyLock(1, "if (mtProfileEnabled) mtProfileTrueParallelWallNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileBatchBegin).count();\n");
  emitBodyLock(0, "}\n");
  }
}

void graph::genMtCoarseRegionRunner(const MtCoarseRegionPlan& coarsePlan) {
  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapForInvocation();
  // shared emitter for `switch (mtaskIndex) { case M: <body>; break; ... }` body.
  // Used by both mtRunCoarseMTaskWorkerList and mtRunCoarseMTaskWorkerRange so the
  // per-mtask semantics stay in sync. Outer caller emits indent N for `switch (mtaskIndex)`.
  auto emitMtaskInnerSwitch = [&](const MtCoarseRegion& region, int outerIndent) {
    emitBodyLock(outerIndent, "switch (mtaskIndex) {\n");
    for (size_t mtaskIdx = 0; mtaskIdx < region.mtasks.size(); mtaskIdx ++) {
      const MtCoarseMTask& mtask = region.mtasks[mtaskIdx];
      emitBodyLock(outerIndent + 1, "case %zu:\n", mtaskIdx);
      for (size_t layerIdx = 0; layerIdx < mtask.layerTaskCppIds.size(); layerIdx ++) {
        const std::vector<int>& taskCppIds = mtask.layerTaskCppIds[layerIdx];
        if (taskCppIds.empty()) continue;
        emitBodyLock(outerIndent + 2, "{\n");
        for (int cppId : taskCppIds) {
          int wordOffset = cppId / ACTIVE_WIDTH - region.beginActiveWord;
          uint64_t mask = (uint64_t)1 << (cppId % ACTIVE_WIDTH);
          emitBodyLock(outerIndent + 3, "if (mtWorkerCoarseFlags[worker][%d] & 0x%lx) {\n", wordOffset, mask);
          emitBodyLock(outerIndent + 4, "mtTask%d(mtWorkerCoarseFlags[worker][%d], mtWorkerDeltas[worker]);\n", cppId, wordOffset);
          emitBodyLock(outerIndent + 4, "if (mtProfileEnabled) {\n");
          emitBodyLock(outerIndent + 5, "mtProfileLocalTaskIds[worker].push_back(%d);\n", cppId);
          emitBodyLock(outerIndent + 5, "mtProfileLocalWorkerTaskCount[worker] ++;\n");
          emitBodyLock(outerIndent + 4, "}\n");
          emitBodyLock(outerIndent + 3, "}\n");
        }
        emitBodyLock(outerIndent + 3, "mtMergeLocalCoarseDelta(worker, %d, %d);\n", region.beginActiveWord, region.activeWordSpan);
        emitBodyLock(outerIndent + 2, "}\n");
      }
      emitBodyLock(outerIndent + 2, "break;\n");
    }
    emitBodyLock(outerIndent + 1, "default:\n");
    emitBodyLock(outerIndent + 2, "break;\n");
    emitBodyLock(outerIndent, "}\n");
  };

  emitFuncDecl(0, "void S%s::mtRunCoarseLayerWorkerRange(int worker, int regionIndex, int layerIndex, int chunkBegin, int chunkEnd) {\n", name.c_str());
  emitBodyLock(1, "if (chunkEnd <= chunkBegin) return;\n");
  emitBodyLock(1, "if (mtCoarseSkeletalMode) return;\n");
  emitBodyLock(1, "switch (regionIndex) {\n");
  int regionIndex = 0;
  for (const MtCoarseRegion& region : coarsePlan.regions) {
    if (!region.runtimeEligible) continue;
    emitBodyLock(2, "case %d:\n", regionIndex);
    emitBodyLock(3, "switch (layerIndex) {\n");
    for (size_t layerIdx = 0; layerIdx < region.layers.size(); layerIdx ++) {
      const MtCoarseLayer& layer = region.layers[layerIdx];
      emitBodyLock(4, "case %zu:\n", layerIdx);
      emitBodyLock(5, "for (int localIndex = chunkBegin; localIndex < chunkEnd; localIndex ++) {\n");
      emitBodyLock(6, "switch (localIndex) {\n");
      for (size_t localIndex = 0; localIndex < layer.taskCppIds.size(); localIndex ++) {
        int cppId = layer.taskCppIds[localIndex];
        int wordOffset = cppId / ACTIVE_WIDTH - region.beginActiveWord;
        uint64_t mask = (uint64_t)1 << (cppId % ACTIVE_WIDTH);
        emitBodyLock(7, "case %zu:\n", localIndex);
        emitBodyLock(8, "if (mtWorkerCoarseFlags[worker][%d] & 0x%lx) {\n", wordOffset, mask);
        emitBodyLock(9, "mtTask%d(mtWorkerCoarseFlags[worker][%d], mtWorkerDeltas[worker]);\n", cppId, wordOffset);
        emitBodyLock(9, "if (mtProfileEnabled) {\n");
        emitBodyLock(10, "mtProfileLocalTaskIds[worker].push_back(%d);\n", cppId);
        emitBodyLock(10, "mtProfileLocalWorkerTaskCount[worker] ++;\n");
        emitBodyLock(9, "}\n");
        emitBodyLock(8, "}\n");
        emitBodyLock(8, "break;\n");
      }
      emitBodyLock(7, "default:\n");
      emitBodyLock(8, "break;\n");
      emitBodyLock(6, "}\n");
      emitBodyLock(5, "}\n");
      emitBodyLock(5, "break;\n");
    }
    emitBodyLock(4, "default:\n");
    emitBodyLock(5, "break;\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "break;\n");
    regionIndex ++;
  }
  emitBodyLock(2, "default:\n");
  emitBodyLock(3, "break;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::mtMergeLocalCoarseDelta(int worker, int regionBeginActiveWord, int regionActiveWordSpan) {\n", name.c_str());
  emitBodyLock(1, "if (mtWorkerDeltas[worker].allActive) {\n");
  emitBodyLock(2, "for (int word = 0; word < regionActiveWordSpan; word ++) mtWorkerCoarseFlags[worker][word] = (uint%d_t)-1;\n", ACTIVE_WIDTH);
  emitBodyLock(2, "return;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "size_t entryCountBeforeMerge = mtWorkerDeltas[worker].entries.size();\n");
  emitBodyLock(1, "size_t localEntryCount = 0;\n");
  emitBodyLock(1, "size_t writeIndex = 0;\n");
  emitBodyLock(1, "for (size_t readIndex = 0; readIndex < mtWorkerDeltas[worker].entries.size(); readIndex ++) {\n");
  emitBodyLock(2, "const ActivationDeltaEntry &entry = mtWorkerDeltas[worker].entries[readIndex];\n");
  emitBodyLock(2, "int localWord = entry.idx - regionBeginActiveWord;\n");
  emitBodyLock(2, "if (localWord >= 0 && localWord < regionActiveWordSpan) {\n");
  emitBodyLock(3, "mtWorkerCoarseFlags[worker][localWord] |= (uint%d_t)entry.mask;\n", ACTIVE_WIDTH);
  emitBodyLock(3, "localEntryCount ++;\n");
  emitBodyLock(2, "} else {\n");
  emitBodyLock(3, "mtWorkerDeltas[worker].entries[writeIndex ++] = entry;\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "mtWorkerDeltas[worker].entries.resize(writeIndex);\n");
  emitBodyLock(1, "if (mtProfileEnabled && localEntryCount > 0) {\n");
  emitBodyLock(2, "mtProfileLocalActivationDeltaEntries[worker] += localEntryCount;\n");
  emitBodyLock(2, "if (entryCountBeforeMerge > mtProfileLocalActivationDeltaMaxEntries[worker]) mtProfileLocalActivationDeltaMaxEntries[worker] = entryCountBeforeMerge;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::mtRunCoarseMTaskWorkerList(int worker, int regionIndex, const int *mtaskIndices, int mtaskCount) {\n", name.c_str());
  emitBodyLock(1, "if (mtaskCount <= 0) return;\n");
  emitBodyLock(1, "if (mtCoarseSkeletalMode) return;\n");
  emitBodyLock(1, "switch (regionIndex) {\n");
  regionIndex = 0;
  for (const MtCoarseRegion& region : coarsePlan.regions) {
    if (!region.runtimeEligible) continue;
    emitBodyLock(2, "case %d:\n", regionIndex);
    emitBodyLock(3, "for (int assignedIndex = 0; assignedIndex < mtaskCount; assignedIndex ++) {\n");
    emitBodyLock(4, "int mtaskIndex = mtaskIndices[assignedIndex];\n");
    emitMtaskInnerSwitch(region, 4);
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "break;\n");
    regionIndex ++;
  }
  emitBodyLock(2, "default:\n");
  emitBodyLock(3, "break;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::mtRunCoarseMTaskWorkerRange(int worker, int regionIndex, int mtaskBegin, int mtaskEnd) {\n", name.c_str());
  emitBodyLock(1, "if (mtaskEnd <= mtaskBegin) return;\n");
  emitBodyLock(1, "if (mtCoarseSkeletalMode) return;\n");
  emitBodyLock(1, "switch (regionIndex) {\n");
  regionIndex = 0;
  for (const MtCoarseRegion& region : coarsePlan.regions) {
    if (!region.runtimeEligible) continue;
    emitBodyLock(2, "case %d:\n", regionIndex);
    emitBodyLock(3, "for (int mtaskIndex = mtaskBegin; mtaskIndex < mtaskEnd; mtaskIndex ++) {\n");
    emitMtaskInnerSwitch(region, 4);
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "break;\n");
    regionIndex ++;
  }
  emitBodyLock(2, "default:\n");
  emitBodyLock(3, "break;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(0, "}\n");
  // mutex-protected ready queue for antichain scheduler.
  // Push is called by any worker after a predecessor completes; pop prefers
  // worker0-only tasks on logical worker 0, then parallel tasks.
  emitFuncDecl(0, "void S%s::mtCoarseReadyQueuePush(int regionIndex, int mtaskIndex, bool worker0Only) {\n", name.c_str());
  emitBodyLock(1, "std::lock_guard<std::mutex> lock(mtCoarseReadyQueueMutex);\n");
  emitBodyLock(1, "if (worker0Only) mtCoarseReadyQueueWorker0[regionIndex].push_back(mtaskIndex);\n");
  emitBodyLock(1, "else mtCoarseReadyQueueParallel[regionIndex].push_back(mtaskIndex);\n");
  emitBodyLock(0, "}\n");
  emitFuncDecl(0, "int S%s::mtCoarseReadyQueuePop(int regionIndex, int worker) {\n", name.c_str());
  emitBodyLock(1, "std::lock_guard<std::mutex> lock(mtCoarseReadyQueueMutex);\n");
  emitBodyLock(1, "if (worker == 0) {\n");
  emitBodyLock(2, "auto &q0 = mtCoarseReadyQueueWorker0[regionIndex];\n");
  emitBodyLock(2, "if (!q0.empty()) { int m = q0.back(); q0.pop_back(); return m; }\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "auto &q = mtCoarseReadyQueueParallel[regionIndex];\n");
  emitBodyLock(1, "if (!q.empty()) { int m = q.back(); q.pop_back(); return m; }\n");
  emitBodyLock(1, "return -1;\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::mtRunCoarseMTaskDynamic(int regionIndex, int worker) {\n", name.c_str());
  emitBodyLock(1, "if (mtCoarseSkeletalMode) return;\n");
  emitBodyLock(1, "switch (regionIndex) {\n");
  emitBodyLock(2, "default:\n");
  emitBodyLock(3, "break;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "int S%s::mtCountActiveCoarseMTasks(int regionIndex, uint%d_t *coarseActiveWords, int *activeStaticCost) {\n", name.c_str(), ACTIVE_WIDTH);
  emitBodyLock(1, "int activeCount = 0;\n");
  emitBodyLock(1, "if (activeStaticCost != nullptr) *activeStaticCost = 0;\n");
  emitBodyLock(1, "switch (regionIndex) {\n");
  regionIndex = 0;
  for (const MtCoarseRegion& region : coarsePlan.regions) {
    if (!region.runtimeEligible) continue;
    emitBodyLock(2, "case %d:\n", regionIndex);
    for (size_t mtaskIdx = 0; mtaskIdx < region.mtasks.size(); mtaskIdx ++) {
      const MtCoarseMTask& mtask = region.mtasks[mtaskIdx];
      emitBodyLock(3, "{\n");
      emitBodyLock(4, "bool active = false;\n");
      for (const std::vector<int>& taskCppIds : mtask.layerTaskCppIds) {
        for (int cppId : taskCppIds) {
          int wordOffset = cppId / ACTIVE_WIDTH - region.beginActiveWord;
          uint64_t mask = (uint64_t)1 << (cppId % ACTIVE_WIDTH);
          emitBodyLock(4, "active = active || ((coarseActiveWords[%d] & 0x%lx) != 0);\n", wordOffset, mask);
        }
      }
      emitBodyLock(4, "if (active) {\n");
      emitBodyLock(5, "activeCount ++;\n");
      emitBodyLock(5, "if (activeStaticCost != nullptr) *activeStaticCost += %d;\n", mtask.staticCost);
      emitBodyLock(4, "}\n");
      emitBodyLock(3, "}\n");
    }
    emitBodyLock(3, "break;\n");
    regionIndex ++;
  }
  emitBodyLock(2, "default:\n");
  emitBodyLock(3, "break;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "return activeCount;\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::mtBuildCoarseMTaskWorkerAssignment(int regionIndex, int workerCount, std::vector<std::vector<int>> &assignments, std::vector<uint64_t> &workerStaticCosts, std::vector<uint64_t> &workerTaskCounts) {\n", name.c_str());
  emitBodyLock(1, "if (workerCount < 1) workerCount = 1;\n");
  emitBodyLock(1, "assignments.assign((size_t)workerCount, std::vector<int>());\n");
  emitBodyLock(1, "workerStaticCosts.assign((size_t)workerCount, 0);\n");
  emitBodyLock(1, "workerTaskCounts.assign((size_t)workerCount, 0);\n");
  emitBodyLock(1, "switch (regionIndex) {\n");
  regionIndex = 0;
  for (const MtCoarseRegion& region : coarsePlan.regions) {
    if (!region.runtimeEligible) continue;
    MtCoarseMTaskAssignment assignment =
      mtBuildCoarseMTaskAssignment(region, std::max(1, static_cast<int>(region.mtasks.size())), "profitable");
    std::vector<int> order;
    for (const std::vector<int>& indices : assignment.workerMTaskIndices) {
      order.insert(order.end(), indices.begin(), indices.end());
    }
    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
      if (region.mtasks[lhs].staticCost != region.mtasks[rhs].staticCost) {
        return region.mtasks[lhs].staticCost > region.mtasks[rhs].staticCost;
      }
      if (region.mtasks[lhs].taskCount != region.mtasks[rhs].taskCount) {
        return region.mtasks[lhs].taskCount > region.mtasks[rhs].taskCount;
      }
      return lhs < rhs;
    });
    emitBodyLock(2, "case %d:\n", regionIndex);
    emitBodyLock(3, "{\n");
    std::vector<int> orderedStaticCosts;
    std::vector<int> orderedTaskCounts;
    for (int mtaskIndex : order) {
      orderedStaticCosts.push_back(region.mtasks[mtaskIndex].staticCost);
      orderedTaskCounts.push_back(region.mtasks[mtaskIndex].taskCount);
    }
    emitBodyLock(4, "static const int mtaskOrder[] = {%s};\n", mtJoinIntList(order).c_str());
    emitBodyLock(4, "static const int mtaskStaticCosts[] = {%s};\n", mtJoinIntList(orderedStaticCosts).c_str());
    emitBodyLock(4, "static const int mtaskTaskCounts[] = {%s};\n", mtJoinIntList(orderedTaskCounts).c_str());
    emitBodyLock(4, "const int mtaskOrderCount = %zu;\n", order.size());
    emitBodyLock(4, "for (int orderIndex = 0; orderIndex < mtaskOrderCount; orderIndex ++) {\n");
    emitBodyLock(5, "int bestWorker = 0;\n");
    emitBodyLock(5, "for (int worker = 1; worker < workerCount; worker ++) {\n");
    emitBodyLock(6, "if (workerStaticCosts[(size_t)worker] < workerStaticCosts[(size_t)bestWorker] ||\n");
    emitBodyLock(6, "    (workerStaticCosts[(size_t)worker] == workerStaticCosts[(size_t)bestWorker] && workerTaskCounts[(size_t)worker] < workerTaskCounts[(size_t)bestWorker])) bestWorker = worker;\n");
    emitBodyLock(5, "}\n");
    emitBodyLock(5, "assignments[(size_t)bestWorker].push_back(mtaskOrder[orderIndex]);\n");
    emitBodyLock(5, "workerStaticCosts[(size_t)bestWorker] += (uint64_t)mtaskStaticCosts[orderIndex];\n");
    emitBodyLock(5, "workerTaskCounts[(size_t)bestWorker] += (uint64_t)mtaskTaskCounts[orderIndex];\n");
    emitBodyLock(4, "}\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "break;\n");
    regionIndex ++;
  }
  emitBodyLock(2, "default:\n");
  emitBodyLock(3, "break;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(0, "}\n");

  // codegen-time LPT + flat per-cppId arrays.
  // For each runtime-eligible region and each rounded worker count
  // wc in {1, 2, 4, 8}, we pre-compute an LPT-balanced mtask -> worker
  // assignment and emit per-(region, wc, worker) flat arrays of
  // SCoarseTaskRef. Each ref encodes (wordOffset, mask, fn) plus a
  // mergeAfter flag that triggers mtMergeLocalCoarseDelta at layer
  // boundaries. The inner switch (regionIndex, mtaskIndex) and the
  // runtime LPT loop are both replaced by a flat loop with one indirect
  // call per active task.
  emitFuncDecl(0, "void S%s::mtRunCoarseStaticRefList(int regionIndex, int roundedWC, int worker, int regionBeginActiveWord, int regionActiveWordSpan, const SCoarseTaskRef *refs, int refCount) {\n", name.c_str());
  emitBodyLock(1, "for (int i = 0; i < refCount; i ++) {\n");
  emitBodyLock(2, "const SCoarseTaskRef &r = refs[i];\n");
  emitBodyLock(2, "if (mtWorkerCoarseFlags[worker][r.wordOffset] & r.mask) {\n");
  emitBodyLock(3, "(this->*r.fn)(mtWorkerCoarseFlags[worker][r.wordOffset], mtWorkerDeltas[worker]);\n");
  emitBodyLock(3, "if (mtProfileEnabled) {\n");
  emitBodyLock(4, "mtProfileLocalTaskIds[worker].push_back(r.cppId);\n");
  emitBodyLock(4, "mtProfileLocalWorkerTaskCount[worker] ++;\n");
  emitBodyLock(3, "}\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "if (r.mergeAfter) {\n");
  emitBodyLock(3, "mtMergeLocalCoarseDelta(worker, regionBeginActiveWord, regionActiveWordSpan);\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(0, "}\n");

  // LPT (longest-processing-time) for a given rounded worker count.
  // Same comparator as the runtime mtBuildCoarseMTaskWorkerAssignment so
  // the static plan matches the runtime one at the same wc.
  auto dstaticLptForWc = [&](const MtCoarseRegion& region, int wc) -> std::vector<std::vector<int>> {
    std::vector<std::vector<int>> result(wc);
    if (region.mtasks.empty() || wc <= 0) return result;
    std::vector<int> order(region.mtasks.size());
    for (size_t i = 0; i < order.size(); i ++) order[i] = (int)i;
    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
      if (region.mtasks[lhs].staticCost != region.mtasks[rhs].staticCost)
        return region.mtasks[lhs].staticCost > region.mtasks[rhs].staticCost;
      if (region.mtasks[lhs].taskCount != region.mtasks[rhs].taskCount)
        return region.mtasks[lhs].taskCount > region.mtasks[rhs].taskCount;
      return lhs < rhs;
    });
    std::vector<int> staticCosts(wc, 0);
    std::vector<int> taskCounts(wc, 0);
    for (int mi : order) {
      int best = 0;
      for (int w = 1; w < wc; w ++) {
        if (staticCosts[w] < staticCosts[best] ||
            (staticCosts[w] == staticCosts[best] && taskCounts[w] < taskCounts[best])) best = w;
      }
      result[best].push_back(mi);
      staticCosts[best] += region.mtasks[mi].staticCost;
      taskCounts[best] += region.mtasks[mi].taskCount;
    }
    for (auto& list : result) std::sort(list.begin(), list.end());
    return result;
  };

  // Per-region static dispatchers. One function per runtime-eligible
  // region. Inside, switch on roundedWC, then on worker. Each leaf case
  // owns a static const SCoarseTaskRef[] for that (region, wc, worker).
  static const int kCoarseStaticWcChoices[] = {1, 2, 4, 8};
  regionIndex = 0;
  for (const MtCoarseRegion& region : coarsePlan.regions) {
    if (!region.runtimeEligible) continue;
    emitFuncDecl(0, "void S%s::mtRunCoarseRegionStaticR%d(int roundedWC, int worker, int regionBeginActiveWord, int regionActiveWordSpan) {\n", name.c_str(), regionIndex);
    emitBodyLock(1, "if (mtCoarseSkeletalMode) return;\n");
    emitBodyLock(1, "switch (roundedWC) {\n");
    for (int wc : kCoarseStaticWcChoices) {
      std::vector<std::vector<int>> assign = dstaticLptForWc(region, wc);
      emitBodyLock(2, "case %d:\n", wc);
      emitBodyLock(3, "switch (worker) {\n");
      for (int w = 0; w < wc; w ++) {
        const std::vector<int>& mtIndices = assign[w];
        // Mtask-major flatten: walk this worker's mtasks in scan order
        // (mtaskIdx ascending), and within each mtask walk its internal
        // sub-layers in order, marking the last entry of each (mtask,
        // sub-layer) with mergeAfter=1. This matches the old
        // emitMtaskInnerSwitch cadence: `for layer in mtask: ...; merge;`
        // intra-mtask sub-layer activations stay visible to the next
        // sub-layer's gate check on this worker. (mtIndices is already
        // sorted ascending by dstaticLptForWc.)
        struct DEntry { int cppId; int wordOffset; uint64_t mask; bool mergeAfter; };
        std::vector<DEntry> entries;
        for (int mi : mtIndices) {
          const MtCoarseMTask& mtask = region.mtasks[mi];
          for (size_t L = 0; L < mtask.layerTaskCppIds.size(); L ++) {
            const std::vector<int>& taskCppIds = mtask.layerTaskCppIds[L];
            if (taskCppIds.empty()) continue;
            size_t blockStart = entries.size();
            for (int cppId : taskCppIds) {
              DEntry e;
              e.cppId = cppId;
              e.wordOffset = cppId / ACTIVE_WIDTH - region.beginActiveWord;
              e.mask = (uint64_t)1 << (cppId % ACTIVE_WIDTH);
              e.mergeAfter = false;
              entries.push_back(e);
            }
            if (entries.size() > blockStart) entries.back().mergeAfter = true;
          }
        }
        emitBodyLock(4, "case %d: {\n", w);
        if (entries.empty()) {
          emitBodyLock(5, "(void)worker;\n");
          emitBodyLock(5, "(void)regionBeginActiveWord;\n");
          emitBodyLock(5, "(void)regionActiveWordSpan;\n");
          emitBodyLock(5, "break;\n");
          emitBodyLock(4, "}\n");
          continue;
        }
        emitBodyLock(5, "static const SCoarseTaskRef refs[] = {\n");
        for (const DEntry& e : entries) {
          emitBodyLock(6, "{%d, %d, %d, 0, 0x%lxULL, &S%s::mtTask%d},\n",
                       e.cppId, e.wordOffset, e.mergeAfter ? 1 : 0,
                       (unsigned long)e.mask,
                       name.c_str(),
                       e.cppId);
        }
        emitBodyLock(5, "};\n");
        emitBodyLock(5, "mtRunCoarseStaticRefList(%d, %d, worker, regionBeginActiveWord, regionActiveWordSpan, refs, (int)(sizeof(refs) / sizeof(refs[0])));\n", regionIndex, wc);
        emitBodyLock(5, "break;\n");
        emitBodyLock(4, "}\n");
      }
      emitBodyLock(4, "default:\n");
      emitBodyLock(5, "break;\n");
      emitBodyLock(3, "}\n");
      emitBodyLock(3, "break;\n");
    }
    emitBodyLock(2, "default:\n");
    emitBodyLock(3, "break;\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(0, "}\n");
    regionIndex ++;
  }

  // Top-level dispatcher: thin shim from regionIndex to per-region helper.
  emitFuncDecl(0, "void S%s::mtRunCoarseRegionStaticDispatch(int regionIndex, int roundedWC, int worker, int regionBeginActiveWord, int regionActiveWordSpan) {\n", name.c_str());
  emitBodyLock(1, "switch (regionIndex) {\n");
  regionIndex = 0;
  for (const MtCoarseRegion& region : coarsePlan.regions) {
    if (!region.runtimeEligible) continue;
    (void)region;
    emitBodyLock(2, "case %d:\n", regionIndex);
    emitBodyLock(3, "mtRunCoarseRegionStaticR%d(roundedWC, worker, regionBeginActiveWord, regionActiveWordSpan);\n", regionIndex);
    emitBodyLock(3, "break;\n");
    regionIndex ++;
  }
  emitBodyLock(2, "default:\n");
  emitBodyLock(3, "break;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(0, "}\n");


  // A104 R1-S1: lift the per-cycle per-region constant switch (9 ints * 660
  // regions = ~6000 lines of compare-and-load in a hot path) to function-local
  // static const arrays indexed by runtime-eligible regionIndex.
  // Pure refactor: behavior identical to the previous switch; bit-exact.
  // Function-local statics keep the arrays in the same TU as the function
  // (emitFuncDecl can roll a new .cpp file; file-scope arrays would leak).
  int a104EligibleCount = 0;
  for (const MtCoarseRegion& region : coarsePlan.regions) {
    if (region.runtimeEligible) a104EligibleCount ++;
  }


  emitFuncDecl(0, "void S%s::mtRunCoarseRegion(int regionIndex, uint%d_t *coarseActiveWords) {\n", name.c_str(), ACTIVE_WIDTH);
  // Emit the static const arrays as function-local statics. C++ guarantees
  // single-time initialization with no per-call cost in the hot path.
  emitBodyLock(1, "static const int kCoarseRegionTaskCount[%d] = {", a104EligibleCount);
  {
    bool first = true;
    for (const MtCoarseRegion& region : coarsePlan.regions) {
      if (!region.runtimeEligible) continue;
      emitBodyLock(0, "%s%d", first ? "" : ", ", region.taskCount);
      first = false;
    }
  }
  emitBodyLock(0, "};\n");
  emitBodyLock(1, "static const int kCoarseRegionBeginActiveWord[%d] = {", a104EligibleCount);
  {
    bool first = true;
    for (const MtCoarseRegion& region : coarsePlan.regions) {
      if (!region.runtimeEligible) continue;
      emitBodyLock(0, "%s%d", first ? "" : ", ", region.beginActiveWord);
      first = false;
    }
  }
  emitBodyLock(0, "};\n");
  emitBodyLock(1, "static const int kCoarseRegionActiveWordSpan[%d] = {", a104EligibleCount);
  {
    bool first = true;
    for (const MtCoarseRegion& region : coarsePlan.regions) {
      if (!region.runtimeEligible) continue;
      emitBodyLock(0, "%s%d", first ? "" : ", ", region.activeWordSpan);
      first = false;
    }
  }
  emitBodyLock(0, "};\n");
  emitBodyLock(1, "static const int kCoarseRegionLayerCount[%d] = {", a104EligibleCount);
  {
    bool first = true;
    for (const MtCoarseRegion& region : coarsePlan.regions) {
      if (!region.runtimeEligible) continue;
      emitBodyLock(0, "%s%d", first ? "" : ", ", region.estimatedLayerCount);
      first = false;
    }
  }
  emitBodyLock(0, "};\n");
  emitBodyLock(1, "static const int kCoarseRegionMemberNodeCount[%d] = {", a104EligibleCount);
  {
    bool first = true;
    for (const MtCoarseRegion& region : coarsePlan.regions) {
      if (!region.runtimeEligible) continue;
      emitBodyLock(0, "%s%d", first ? "" : ", ", region.memberNodeCost);
      first = false;
    }
  }
  emitBodyLock(0, "};\n");
  emitBodyLock(1, "static const int kCoarseRegionStaticCost[%d] = {", a104EligibleCount);
  {
    bool first = true;
    for (const MtCoarseRegion& region : coarsePlan.regions) {
      if (!region.runtimeEligible) continue;
      emitBodyLock(0, "%s%d", first ? "" : ", ", region.staticCost);
      first = false;
    }
  }
  emitBodyLock(0, "};\n");
  emitBodyLock(1, "static const int kCoarseRegionUsefulWork[%d] = {", a104EligibleCount);
  {
    bool first = true;
    for (const MtCoarseRegion& region : coarsePlan.regions) {
      if (!region.runtimeEligible) continue;
      emitBodyLock(0, "%s%d", first ? "" : ", ", region.estimatedUsefulWork);
      first = false;
    }
  }
  emitBodyLock(0, "};\n");
  emitBodyLock(1, "static const int kCoarseRegionMaxParallelWidth[%d] = {", a104EligibleCount);
  {
    bool first = true;
    for (const MtCoarseRegion& region : coarsePlan.regions) {
      if (!region.runtimeEligible) continue;
      emitBodyLock(0, "%s%d", first ? "" : ", ", region.estimatedMaxParallelWidth);
      first = false;
    }
  }
  emitBodyLock(0, "};\n");
  emitBodyLock(1, "static const int kCoarseRegionMTaskCount[%d] = {", a104EligibleCount);
  {
    bool first = true;
    for (const MtCoarseRegion& region : coarsePlan.regions) {
      if (!region.runtimeEligible) continue;
      emitBodyLock(0, "%s%zu", first ? "" : ", ", region.mtasks.size());
      first = false;
    }
  }
  emitBodyLock(0, "};\n");
  emitBodyLock(1, "static const int kCoarseRegionBeginCppId[%d] = {", a104EligibleCount);
  {
    bool first = true;
    for (const MtCoarseRegion& region : coarsePlan.regions) {
      if (!region.runtimeEligible) continue;
      emitBodyLock(0, "%s%d", first ? "" : ", ", region.beginCppId);
      first = false;
    }
  }
  emitBodyLock(0, "};\n");
  emitBodyLock(1, "static const int kCoarseRegionEndCppId[%d] = {", a104EligibleCount);
  {
    bool first = true;
    for (const MtCoarseRegion& region : coarsePlan.regions) {
      if (!region.runtimeEligible) continue;
      emitBodyLock(0, "%s%d", first ? "" : ", ", region.endCppId);
      first = false;
    }
  }
  emitBodyLock(0, "};\n");
  // antichain runtime constant arrays (inert scaffold kept for emission
  // byte-identity: the scheduler can no longer be selected, so every region
  // emits the disabled value).
  {
    std::vector<int> useAntichainRuntimeValues(a104EligibleCount, 0);
    std::vector<int> antichainMTaskCountValues(a104EligibleCount, 0);
    std::vector<int> antichainUpstreamOffsets(a104EligibleCount + 1, 0);
    std::vector<int> antichainUpstreamValues;
    std::vector<int> antichainWorker0OnlyValues;
    emitBodyLock(1, "static const bool kCoarseRegionUseAntichainRuntime[%d] = {%s};\n", a104EligibleCount, mtJoinIntList(useAntichainRuntimeValues).c_str());
    emitBodyLock(1, "static const int kCoarseRegionAntichainMTaskCount[%d] = {%s};\n", a104EligibleCount, mtJoinIntList(antichainMTaskCountValues).c_str());
    emitBodyLock(1, "static const int kCoarseRegionAntichainUpstreamOffset[%d] = {%s};\n", a104EligibleCount + 1, mtJoinIntList(antichainUpstreamOffsets).c_str());
    emitBodyLock(1, "static const int kCoarseRegionAntichainUpstreamValues[%zu] = {%s};\n", antichainUpstreamValues.size(), mtJoinIntList(antichainUpstreamValues).c_str());
    emitBodyLock(1, "static const bool kCoarseRegionAntichainWorker0Only[%zu] = {%s};\n", antichainWorker0OnlyValues.size(), mtJoinIntList(antichainWorker0OnlyValues).c_str());
  }
  emitBodyLock(1, "std::chrono::steady_clock::time_point mtProfileBatchBegin;\n");
  emitBodyLock(1, "if (mtProfileEnabled) mtProfileBatchBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(1, "if ((unsigned)regionIndex >= %du) return;\n", a104EligibleCount);
  emitBodyLock(1, "const int regionTaskCount = kCoarseRegionTaskCount[regionIndex];\n");
  emitBodyLock(1, "const int regionBeginActiveWord = kCoarseRegionBeginActiveWord[regionIndex];\n");
  emitBodyLock(1, "const int regionActiveWordSpan = kCoarseRegionActiveWordSpan[regionIndex];\n");
  emitBodyLock(1, "const int regionLayerCount = kCoarseRegionLayerCount[regionIndex];\n");
  emitBodyLock(1, "const int regionMemberNodeCount = kCoarseRegionMemberNodeCount[regionIndex];\n");
  emitBodyLock(1, "const int regionStaticCost = kCoarseRegionStaticCost[regionIndex];\n");
  emitBodyLock(1, "const int regionUsefulWork = kCoarseRegionUsefulWork[regionIndex];\n");
  emitBodyLock(1, "const int regionMaxParallelWidth = kCoarseRegionMaxParallelWidth[regionIndex];\n");
  emitBodyLock(1, "const int regionMTaskCount = kCoarseRegionMTaskCount[regionIndex];\n");
  emitBodyLock(1, "const int regionBeginCppId = kCoarseRegionBeginCppId[regionIndex];\n");
  emitBodyLock(1, "const int regionEndCppId = kCoarseRegionEndCppId[regionIndex];\n");
  emitBodyLock(1, "const bool mtVerilatorDualPathSelected = mtUseVerilatorDualPath && ((mtVerilatorDualPathRegionIndex >= 0 && regionIndex == mtVerilatorDualPathRegionIndex) || (regionBeginCppId == mtVerilatorDualPathBeginCppId && regionEndCppId == mtVerilatorDualPathEndCppId));\n");
  emitBodyLock(1, "if (mtProfileEnabled && mtVerilatorDualPathSelected) mtProfileVerilatorDualPathDispatches ++;\n");
  emitBodyLock(1, "int activeMTaskCount = 0;\n");
  emitBodyLock(1, "int activeMTaskStaticCost = 0;\n");
  emitBodyLock(1, "if (mtProfileEnabled) {\n");
  emitBodyLock(2, "mtProfileCoarseRegionInvocations ++;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "int workerCount = mtConfiguredWorkerCount;\n");
  emitBodyLock(1, "if (workerCount > regionTaskCount) workerCount = regionTaskCount;\n");
  emitBodyLock(1, "if (workerCount < 2) workerCount = 1;\n");
  if (globalConfig.MtCoarseProfitabilityMode == "static" &&
      globalConfig.MtCoarseWorkerPolicyMode != "profitable") {
    emitBodyLock(1, "if (workerCount > 1) {\n");
    emitBodyLock(2, "if (regionMaxParallelWidth > 0 && workerCount > regionMaxParallelWidth) workerCount = regionMaxParallelWidth;\n");
    emitBodyLock(2, "int workerCapByActiveWords = regionActiveWordSpan <= 0 ? workerCount : regionMemberNodeCount / (regionActiveWordSpan * 8);\n");
    emitBodyLock(2, "int workerCapByMemberCost = regionMemberNodeCount / 8;\n");
    emitBodyLock(2, "int workerCapByStaticCost = regionStaticCost / 4;\n");
    emitBodyLock(2, "int profitabilityWorkerCap = workerCount;\n");
    emitBodyLock(2, "if (workerCapByActiveWords > 0 && profitabilityWorkerCap > workerCapByActiveWords) profitabilityWorkerCap = workerCapByActiveWords;\n");
    emitBodyLock(2, "if (workerCapByMemberCost > 0 && profitabilityWorkerCap > workerCapByMemberCost) profitabilityWorkerCap = workerCapByMemberCost;\n");
    emitBodyLock(2, "if (workerCapByStaticCost > 0 && profitabilityWorkerCap > workerCapByStaticCost) profitabilityWorkerCap = workerCapByStaticCost;\n");
    emitBodyLock(2, "if (profitabilityWorkerCap < 1) profitabilityWorkerCap = 1;\n");
    emitBodyLock(2, "workerCount = profitabilityWorkerCap;\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(1, "if (workerCount > 1 && regionMemberNodeCount < workerCount * 8) workerCount = 1;\n");
    emitBodyLock(1, "if (workerCount > 1 && regionStaticCost < workerCount * 4) workerCount = 1;\n");
    emitBodyLock(1, "if (workerCount > 1 && regionActiveWordSpan > 0 && regionMemberNodeCount < regionActiveWordSpan * workerCount * 8) workerCount = 1;\n");
  }
  if (globalConfig.MtCoarseProfitabilityMode == "static") {
    emitBodyLock(1, "if (mtCoarseUseMTaskRuntime || mtVerilatorDualPathSelected) {\n");
    if (globalConfig.MtCoarseWorkerPolicyMode == "profitable") {
      emitBodyLock(2, "activeMTaskCount = mtCountActiveCoarseMTasks(regionIndex, coarseActiveWords, &activeMTaskStaticCost);\n");
      emitBodyLock(2, "if (regionMaxParallelWidth > 0 && workerCount > regionMaxParallelWidth) workerCount = regionMaxParallelWidth;\n");
      emitBodyLock(2, "if (workerCount > activeMTaskCount) workerCount = activeMTaskCount;\n");
      emitBodyLock(2, "if (workerCount > 1) {\n");
      emitBodyLock(3, "int activeUsefulCost = activeMTaskStaticCost > 0 ? activeMTaskStaticCost : regionUsefulWork;\n");
      emitBodyLock(3, "while (workerCount > 1) {\n");
      emitBodyLock(4, "int copyMergeWords = regionActiveWordSpan * workerCount * 2;\n");
      emitBodyLock(4, "// mirror the stricter codegen-time admission gate.\n");
      emitBodyLock(4, "if (regionMTaskCount >= 8 && regionMaxParallelWidth >= workerCount &&\n");
      emitBodyLock(4, "    activeUsefulCost >= 256 && activeUsefulCost / workerCount >= 64 &&\n");
      emitBodyLock(4, "    activeUsefulCost >= copyMergeWords * 16) break;\n");
      emitBodyLock(4, "workerCount --;\n");
      emitBodyLock(3, "}\n");
      emitBodyLock(2, "}\n");
    } else {
      emitBodyLock(2, "if (mtProfileEnabled) activeMTaskCount = mtCountActiveCoarseMTasks(regionIndex, coarseActiveWords, &activeMTaskStaticCost);\n");
    }
    emitBodyLock(2, "if (workerCount > regionMTaskCount) workerCount = regionMTaskCount;\n");
    emitBodyLock(2, "if (workerCount < 1) workerCount = 1;\n");
    emitBodyLock(1, "}\n");
  }
  // runtime profitability gate. Pop-count actual active bits in this
  // region for this cycle. If below an explicit GSIM_MT_COARSE_MIN_ACTIVE_BITS
  // threshold (default 0 disables the gate), force workerCount=1 so the layer
  // loop runs inline and avoids per-region/per-layer worker-pool overhead.
  emitBodyLock(1, "int coarseRuntimeActiveBits = 0;\n");
  emitBodyLock(1, "for (int w = 0; w < regionActiveWordSpan; w ++) {\n");
  emitBodyLock(2, "coarseRuntimeActiveBits += __builtin_popcount((unsigned int)coarseActiveWords[w]);\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "if (mtCoarseMinActiveBits > 0 && coarseRuntimeActiveBits < mtCoarseMinActiveBits) workerCount = 1;\n");
  emitBodyLock(1, "if (mtVerilatorDualPathSelected && workerCount < 2 && mtConfiguredWorkerCount >= 2 && regionMTaskCount >= 2) workerCount = 2;\n");
  emitBodyLock(1, "if (mtProfileEnabled) {\n");
  emitBodyLock(2, "int batchSizeBucket = regionTaskCount <= 1 ? 0 : (regionTaskCount == 2 ? 1 : (regionTaskCount <= 4 ? 2 : (regionTaskCount <= 8 ? 3 : (regionTaskCount <= 15 ? 4 : 5))));\n");
  emitBodyLock(2, "mtProfileBatchSizeHist[batchSizeBucket] ++;\n");
  emitBodyLock(2, "mtProfilePureBatchCount ++;\n");
  emitBodyLock(2, "mtProfileBatchMemberNodeCount += regionMemberNodeCount;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "if (mtProfileEnabled && mtProfileWorkerTaskCount.size() < (size_t)workerCount) mtProfileWorkerTaskCount.resize((size_t)workerCount, 0);\n");
  emitBodyLock(1, "if (mtProfileEnabled) {\n");
  emitBodyLock(2, "if (mtProfileCoarseSelectedWorkerCountHist.size() <= (size_t)workerCount) mtProfileCoarseSelectedWorkerCountHist.resize((size_t)workerCount + 1, 0);\n");
  emitBodyLock(2, "mtProfileCoarseSelectedWorkerCountHist[(size_t)workerCount] ++;\n");
  emitBodyLock(2, "mtProfileCoarseEstimatedUsefulWork += regionUsefulWork;\n");
  emitBodyLock(2, "mtProfileCoarseEstimatedOverheadWords += (uint64_t)workerCount * (uint64_t)regionActiveWordSpan * 2;\n");
  emitBodyLock(2, "mtProfileCoarseActiveMTaskCount += (uint64_t)activeMTaskCount;\n");
  emitBodyLock(2, "mtProfileCoarseActiveMTaskStaticCost += (uint64_t)activeMTaskStaticCost;\n");
  emitBodyLock(2, "if (workerCount > 1) mtProfileCoarseAcceptedRegions ++;\n");
  emitBodyLock(2, "else {\n");
  emitBodyLock(3, "mtProfileCoarseRejectedRegions ++;\n");
  emitBodyLock(3, "mtProfileCoarseEstimatedRejectedUsefulWork += regionUsefulWork;\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "if (workerCount == 1) {\n");
  emitBodyLock(2, "if (mtProfileEnabled) {\n");
  emitBodyLock(3, "mtProfileSkippedFakeParallelBatchCount ++;\n");
  emitBodyLock(3, "if (mtProfileEffectiveWorkerCountHist.size() <= 1) mtProfileEffectiveWorkerCountHist.resize(2, 0);\n");
  emitBodyLock(3, "mtProfileEffectiveWorkerCountHist[1] ++;\n");
  emitBodyLock(3, "mtProfileRejectConfiguredSingleWorker ++;\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(1, "} else if (mtProfileEnabled) {\n");
  emitBodyLock(2, "mtProfileTrueParallelBatchCount ++;\n");
  emitBodyLock(2, "if (workerCount > mtProfileMaxWorkerCount) mtProfileMaxWorkerCount = workerCount;\n");
  emitBodyLock(2, "if (mtProfileEffectiveWorkerCountHist.size() <= (size_t)workerCount) mtProfileEffectiveWorkerCountHist.resize((size_t)workerCount + 1, 0);\n");
  emitBodyLock(2, "mtProfileEffectiveWorkerCountHist[(size_t)workerCount] ++;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "if (mtWorkerDeltas.size() < (size_t)workerCount) mtWorkerDeltas.resize((size_t)workerCount);\n");
  emitBodyLock(1, "if (mtWorkerCoarseFlags.size() < (size_t)workerCount) mtWorkerCoarseFlags.resize((size_t)workerCount);\n");
  emitBodyLock(1, "if (mtProfileEnabled) {\n");
  emitBodyLock(2, "mtProfileLocalWorkerTaskCount.assign((size_t)workerCount, 0);\n");
  emitBodyLock(2, "if (mtProfileLocalTaskIds.size() < (size_t)workerCount) mtProfileLocalTaskIds.resize((size_t)workerCount);\n");
  emitBodyLock(2, "for (int worker = 0; worker < workerCount; worker ++) mtProfileLocalTaskIds[worker].clear();\n");
  emitBodyLock(2, "mtProfileLocalActivationDeltaEntries.assign((size_t)workerCount, 0);\n");
  emitBodyLock(2, "mtProfileLocalActivationDeltaMaxEntries.assign((size_t)workerCount, 0);\n");
  emitBodyLock(1, "}\n");
  // atomic-counter antichain runtime. Single-threaded init,
  // then workers scan/CAS ready mtasks and hand off via shared region flags.
  emitBodyLock(1, "if (mtCoarseUseAntichainRuntime && !mtVerilatorDualPathSelected && kCoarseRegionUseAntichainRuntime[regionIndex]) {\n");
  emitBodyLock(2, "mtWorkerPoolCoarseActiveWords = coarseActiveWords;\n");
  emitBodyLock(2, "int antichainMTaskCount = kCoarseRegionAntichainMTaskCount[regionIndex];\n");
  emitBodyLock(2, "if (antichainMTaskCount <= 0) return;\n");
  emitBodyLock(2, "int antichainWorkerCount = workerCount;\n");
  emitBodyLock(2, "if (antichainWorkerCount > antichainMTaskCount) antichainWorkerCount = antichainMTaskCount;\n");
  emitBodyLock(2, "if (antichainWorkerCount < 1) antichainWorkerCount = 1;\n");
  emitBodyLock(2, "if (mtWorkerDeltas.size() < (size_t)antichainWorkerCount) mtWorkerDeltas.resize((size_t)antichainWorkerCount);\n");
  emitBodyLock(2, "if (mtWorkerCoarseFlags.size() < (size_t)antichainWorkerCount) mtWorkerCoarseFlags.resize((size_t)antichainWorkerCount);\n");
  emitBodyLock(2, "for (int worker = 0; worker < antichainWorkerCount; worker ++) {\n");
  emitBodyLock(3, "mtWorkerDeltas[worker].clear();\n");
  emitBodyLock(3, "mtWorkerCoarseFlags[worker].assign(coarseActiveWords, coarseActiveWords + regionActiveWordSpan);\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "if (mtProfileEnabled) {\n");
  emitBodyLock(3, "mtProfileLocalWorkerTaskCount.assign((size_t)antichainWorkerCount, 0);\n");
  emitBodyLock(3, "if (mtProfileLocalTaskIds.size() < (size_t)antichainWorkerCount) mtProfileLocalTaskIds.resize((size_t)antichainWorkerCount);\n");
  emitBodyLock(3, "for (int worker = 0; worker < antichainWorkerCount; worker ++) mtProfileLocalTaskIds[worker].clear();\n");
  emitBodyLock(3, "mtProfileLocalActivationDeltaEntries.assign((size_t)antichainWorkerCount, 0);\n");
  emitBodyLock(3, "mtProfileLocalActivationDeltaMaxEntries.assign((size_t)antichainWorkerCount, 0);\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "// Lazy-allocate per-region atomic state (first invocation only).\n");
  emitBodyLock(2, "if (mtCoarseMTaskClaimGen[regionIndex] == nullptr) {\n");
  emitBodyLock(3, "mtCoarseMTaskClaimGen[regionIndex] = new std::atomic<uint64_t>[antichainMTaskCount];\n");
  emitBodyLock(3, "mtCoarseMTaskUpstream[regionIndex] = new std::atomic<int>[antichainMTaskCount];\n");
  emitBodyLock(3, "mtCoarseRegionSharedFlags[regionIndex] = new std::atomic<uint%d_t>[regionActiveWordSpan]();\n", ACTIVE_WIDTH);
  emitBodyLock(3, "mtCoarseRegionCycle[regionIndex] = new std::atomic<uint64_t>[1];\n");
  emitBodyLock(3, "mtCoarseMTaskCount[regionIndex] = antichainMTaskCount;\n");
  emitBodyLock(3, "mtCoarseRegionCycle[regionIndex][0].store(0, std::memory_order_relaxed);\n");
  emitBodyLock(3, "// First invocation is odd (cycle=1): upstream must count up to depCount.\n");
  emitBodyLock(3, "for (int m = 0; m < antichainMTaskCount; m ++) {\n");
  emitBodyLock(4, "mtCoarseMTaskClaimGen[regionIndex][m].store(0, std::memory_order_relaxed);\n");
  emitBodyLock(4, "mtCoarseMTaskUpstream[regionIndex][m].store(0, std::memory_order_relaxed);\n");
  emitBodyLock(4, "}\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "// Stamp a new cycle; even cycles decrement upstream to 0, odd cycles increment to depCount.\n");
  emitBodyLock(2, "uint64_t cycle = ++mtCoarseRegionCycle[regionIndex][0];\n");
  emitBodyLock(2, "bool evenCycle = (cycle % 2 == 0);\n");
  emitBodyLock(2, "(void)evenCycle;\n");
  emitBodyLock(2, "for (int w = 0; w < regionActiveWordSpan; w ++) {\n");
  emitBodyLock(3, "mtCoarseRegionSharedFlags[regionIndex][w].store(0, std::memory_order_relaxed);\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "mtCoarseMTaskRemaining.store(antichainMTaskCount, std::memory_order_relaxed);\n");
    // seed the antichain ready queue with source mtasks (zero upstream deps).
    emitBodyLock(2, "if (mtCoarseUseAntichainQueue) {\n");
    emitBodyLock(3, "{\n");
    emitBodyLock(4, "std::lock_guard<std::mutex> lock(mtCoarseReadyQueueMutex);\n");
    emitBodyLock(4, "mtCoarseReadyQueueParallel[regionIndex].clear();\n");
    emitBodyLock(4, "mtCoarseReadyQueueWorker0[regionIndex].clear();\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "mtCoarseMTaskInFlight.store(0, std::memory_order_relaxed);\n");
    emitBodyLock(3, "int antichainUpstreamOffset = kCoarseRegionAntichainUpstreamOffset[regionIndex];\n");
    emitBodyLock(3, "for (int m = 0; m < antichainMTaskCount; m ++) {\n");
    emitBodyLock(4, "if (kCoarseRegionAntichainUpstreamValues[antichainUpstreamOffset + m] == 0) {\n");
    emitBodyLock(5, "mtCoarseReadyQueuePush(regionIndex, m, kCoarseRegionAntichainWorker0Only[antichainUpstreamOffset + m]);\n");
    emitBodyLock(4, "}\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(2, "}\n");
  emitBodyLock(2, "if (mtProfileEnabled) {\n");
  emitBodyLock(3, "mtProfileCoarseMTaskDispatches += antichainMTaskCount;\n");
  emitBodyLock(3, "mtProfileCoarseAntichainDispatches ++;\n");
  emitBodyLock(3, "mtProfileCoarseWorkerJobs += antichainWorkerCount;\n");
  emitBodyLock(3, "mtProfileCoarseFlagWordCopies += (uint64_t)antichainWorkerCount * (uint64_t)regionActiveWordSpan;\n");
  emitBodyLock(3, "mtProfileCoarseEstimatedBarrierCount += 1;\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "if (antichainWorkerCount == 1) {\n");
  emitBodyLock(3, "mtRunCoarseMTaskDynamic(regionIndex, 0);\n");
  emitBodyLock(2, "} else if (mtWorkerPoolEnabled && mtWorkerPoolThreadCount + 1 >= antichainWorkerCount) {\n");
  emitBodyLock(3, "mtWorkerPoolJobKind = 5;\n");
  emitBodyLock(3, "mtWorkerPoolCoarseRegionIndex = regionIndex;\n");
  emitBodyLock(3, "mtWorkerPoolCoarseLayerIndex = -1;\n");
  emitBodyLock(3, "mtWorkerPoolCurrentWorkerCount = antichainWorkerCount;\n");
  emitBodyLock(3, "mtWorkerPoolPost();\n");
  emitBodyLock(3, "mtRunCoarseMTaskDynamic(regionIndex, 0);\n");
  emitBodyLock(3, "mtWorkerPoolWaitForDone(antichainWorkerCount - 1);\n");
  emitBodyLock(2, "} else {\n");
  emitBodyLock(3, "std::vector<std::thread> workers;\n");
  emitBodyLock(3, "workers.reserve((size_t)antichainWorkerCount - 1);\n");
  emitBodyLock(3, "for (int worker = 1; worker < antichainWorkerCount; worker ++) {\n");
  emitBodyLock(4, "workers.emplace_back([this, worker, regionIndex]() { mtRunCoarseMTaskDynamic(regionIndex, worker); });\n");
  emitBodyLock(3, "}\n");
  emitBodyLock(3, "mtRunCoarseMTaskDynamic(regionIndex, 0);\n");
  emitBodyLock(3, "for (std::thread &worker : workers) worker.join();\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "// Merge residual deltas into coarseActiveWords/activeFlags and shared flags into coarseActiveWords.\n");
  emitBodyLock(2, "for (int worker = 0; worker < antichainWorkerCount; worker ++) {\n");
  emitBodyLock(3, "for (const ActivationDeltaEntry &entry : mtWorkerDeltas[worker].entries) {\n");
  emitBodyLock(4, "int localWord = entry.idx - regionBeginActiveWord;\n");
  emitBodyLock(4, "if (localWord >= 0 && localWord < regionActiveWordSpan) coarseActiveWords[localWord] |= (uint%d_t)entry.mask;\n", ACTIVE_WIDTH);
  emitBodyLock(4, "else activeFlags[entry.idx] |= (uint%d_t)entry.mask;\n", ACTIVE_WIDTH);
  emitBodyLock(3, "}\n");
  emitBodyLock(3, "if (mtWorkerDeltas[worker].allActive) {\n");
  emitBodyLock(4, "for (int word = 0; word < regionActiveWordSpan; word ++) coarseActiveWords[word] = (uint%d_t)-1;\n", ACTIVE_WIDTH);
  emitBodyLock(4, "for (int word = 0; word < %d; word ++) activeFlags[word] = (uint%d_t)-1;\n", activeFlagNum, ACTIVE_WIDTH);
  emitBodyLock(3, "}\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "for (int word = 0; word < regionActiveWordSpan; word ++) {\n");
  emitBodyLock(3, "coarseActiveWords[word] |= mtCoarseRegionSharedFlags[regionIndex][word].load(std::memory_order_relaxed);\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "if (mtProfileEnabled) {\n");
  emitBodyLock(3, "mtProfileCoarseMergeWordScans += (uint64_t)antichainWorkerCount * (uint64_t)regionActiveWordSpan;\n");
  emitBodyLock(3, "for (int worker = 0; worker < antichainWorkerCount; worker ++) {\n");
  emitBodyLock(4, "mtProfileCoarseActivationDeltaEntries += mtProfileLocalActivationDeltaEntries[worker];\n");
  emitBodyLock(4, "mtProfileActivationDeltaEntries += mtProfileLocalActivationDeltaEntries[worker];\n");
  emitBodyLock(4, "mtProfileCoarseActivationDeltaEntries += mtWorkerDeltas[worker].entries.size();\n");
  emitBodyLock(4, "mtProfileActivationDeltaEntries += mtWorkerDeltas[worker].entries.size();\n");
  emitBodyLock(4, "if (mtProfileLocalActivationDeltaMaxEntries[worker] > mtProfileActivationDeltaMaxEntriesPerWorker) mtProfileActivationDeltaMaxEntriesPerWorker = mtProfileLocalActivationDeltaMaxEntries[worker];\n");
  emitBodyLock(4, "if (mtWorkerDeltas[worker].entries.size() > mtProfileActivationDeltaMaxEntriesPerWorker) mtProfileActivationDeltaMaxEntriesPerWorker = mtWorkerDeltas[worker].entries.size();\n");
  emitBodyLock(4, "if (mtWorkerDeltas[worker].allActive) mtProfileActivationDeltaActivateAllCount ++;\n");
  emitBodyLock(4, "mtProfileWorkerTaskCount[(size_t)worker] += mtProfileLocalWorkerTaskCount[worker];\n");
  emitBodyLock(4, "mtProfilePureTasks += mtProfileLocalWorkerTaskCount[worker];\n");
  emitBodyLock(4, "bool mtTraceCycleActive = mtProfileDynamicTraceFile != nullptr && cycles >= mtProfileDynamicTraceCycleStart && cycles < mtProfileDynamicTraceCycleLimit;\n");
  emitBodyLock(4, "for (int cppId : mtProfileLocalTaskIds[worker]) { if (cppId >= 0 && cppId < %d) { mtProfileTaskExecCount[cppId] ++; if (mtTraceCycleActive) mtProfileDynamicTraceTaskIds.push_back(cppId); } }\n", superId);
  emitBodyLock(3, "}\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "if (mtProfileEnabled) mtProfileBatchWallNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileBatchBegin).count();\n");
  emitBodyLock(2, "if (mtProfileEnabled && antichainWorkerCount > 1) mtProfileTrueParallelWallNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileBatchBegin).count();\n");
  emitBodyLock(1, "return;\n");
  emitBodyLock(1, "}\n");
  // mtask runtime is a runtime branch (env GSIM_MT_COARSE_RUNTIME=mtask|layered);
  // emit the mtask block unconditionally and gate execution by mtCoarseUseMTaskRuntime.
  emitBodyLock(1, "if (mtCoarseUseMTaskRuntime || mtVerilatorDualPathSelected) {\n");
  emitBodyLock(1, "if (regionMTaskCount <= 0) return;\n");
  emitBodyLock(1, "int mtaskWorkerCount = workerCount;\n");
  emitBodyLock(1, "if (mtaskWorkerCount > regionMTaskCount) mtaskWorkerCount = regionMTaskCount;\n");
  emitBodyLock(1, "if (mtaskWorkerCount < 1) mtaskWorkerCount = 1;\n");
  if (globalConfig.MtCoarseWorkerPolicyMode == "profitable") {
    emitBodyLock(1, "std::vector<std::vector<int>> mtaskWorkerAssignments;\n");
    emitBodyLock(1, "std::vector<uint64_t> mtaskWorkerStaticCosts;\n");
    emitBodyLock(1, "std::vector<uint64_t> mtaskWorkerTaskCounts;\n");
    emitBodyLock(1, "mtBuildCoarseMTaskWorkerAssignment(regionIndex, mtaskWorkerCount, mtaskWorkerAssignments, mtaskWorkerStaticCosts, mtaskWorkerTaskCounts);\n");
    emitBodyLock(1, "uint64_t balancedWorstStaticCost = 0;\n");
    emitBodyLock(1, "uint64_t balancedBestStaticCost = UINT64_MAX;\n");
    emitBodyLock(1, "uint64_t balancedAssignedStaticCost = 0;\n");
    emitBodyLock(1, "for (uint64_t cost : mtaskWorkerStaticCosts) {\n");
    emitBodyLock(2, "if (cost > balancedWorstStaticCost) balancedWorstStaticCost = cost;\n");
    emitBodyLock(2, "if (cost < balancedBestStaticCost) balancedBestStaticCost = cost;\n");
    emitBodyLock(2, "balancedAssignedStaticCost += cost;\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(1, "if (balancedBestStaticCost == UINT64_MAX) balancedBestStaticCost = 0;\n");
    emitBodyLock(1, "uint64_t contiguousWorstStaticCost = 0;\n");
    emitBodyLock(1, "for (int worker = 0; worker < mtaskWorkerCount; worker ++) {\n");
    emitBodyLock(2, "int begin = (regionMTaskCount * worker) / mtaskWorkerCount;\n");
    emitBodyLock(2, "int end = (regionMTaskCount * (worker + 1)) / mtaskWorkerCount;\n");
    emitBodyLock(2, "uint64_t cost = 0;\n");
    emitBodyLock(2, "switch (regionIndex) {\n");
    regionIndex = 0;
    for (const MtCoarseRegion& region : coarsePlan.regions) {
      if (!region.runtimeEligible) continue;
      emitBodyLock(3, "case %d:\n", regionIndex);
      emitBodyLock(4, "{\n");
      std::vector<int> costs;
      for (const MtCoarseMTask& mtask : region.mtasks) costs.push_back(mtask.staticCost);
      emitBodyLock(5, "static const int mtaskStaticCosts[] = {%s};\n", mtJoinIntList(costs).c_str());
      emitBodyLock(5, "for (int mtaskIndex = begin; mtaskIndex < end; mtaskIndex ++) cost += (uint64_t)mtaskStaticCosts[mtaskIndex];\n");
      emitBodyLock(4, "}\n");
      emitBodyLock(4, "break;\n");
      regionIndex ++;
    }
    emitBodyLock(3, "default:\n");
    emitBodyLock(4, "break;\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(2, "if (cost > contiguousWorstStaticCost) contiguousWorstStaticCost = cost;\n");
    emitBodyLock(1, "}\n");
  }
  // D-static uses precomputed worker plans for powers of two only. Round before
  // per-worker clear/copy/profile so accepted regions do not pay for workers
  // that the D-static executor will not launch.
  emitBodyLock(1, "if (mtCoarseUseDStatic && !mtVerilatorDualPathSelected) {\n");
  emitBodyLock(2, "if (mtaskWorkerCount >= 8) mtaskWorkerCount = 8;\n");
  emitBodyLock(2, "else if (mtaskWorkerCount >= 4) mtaskWorkerCount = 4;\n");
  emitBodyLock(2, "else if (mtaskWorkerCount >= 2) mtaskWorkerCount = 2;\n");
  emitBodyLock(2, "else mtaskWorkerCount = 1;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "for (int worker = 0; worker < mtaskWorkerCount; worker ++) {\n");
  emitBodyLock(2, "mtWorkerDeltas[worker].clear();\n");
  emitBodyLock(2, "mtWorkerCoarseFlags[worker].assign(coarseActiveWords, coarseActiveWords + regionActiveWordSpan);\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "if (mtProfileEnabled) {\n");
  emitBodyLock(2, "mtProfileCoarseMTaskDispatches += regionMTaskCount;\n");
  emitBodyLock(2, "mtProfileCoarseWorkerJobs += mtaskWorkerCount;\n");
  emitBodyLock(2, "mtProfileCoarseFlagWordCopies += (uint64_t)mtaskWorkerCount * (uint64_t)regionActiveWordSpan;\n");
  emitBodyLock(2, "mtProfileCoarseEstimatedBarrierCount += 1;\n");
  if (globalConfig.MtCoarseWorkerPolicyMode == "profitable") {
    emitBodyLock(2, "mtProfileCoarseAssignedStaticCost += balancedAssignedStaticCost;\n");
    emitBodyLock(2, "mtProfileCoarseWorstWorkerStaticCost += balancedWorstStaticCost;\n");
    emitBodyLock(2, "mtProfileCoarseBestWorkerStaticCost += balancedBestStaticCost;\n");
    emitBodyLock(2, "mtProfileCoarseContiguousWorstStaticCost += contiguousWorstStaticCost;\n");
    emitBodyLock(2, "mtProfileCoarseBalancedWorstStaticCost += balancedWorstStaticCost;\n");
  }
  emitBodyLock(1, "}\n");
  // when mtCoarseUseDStatic is set, replace the
  // double-switch (regionIndex, mtaskIndex) dispatch with the codegen-time
  // LPT + flat-array path. mtaskWorkerCount was rounded above to match the
  // available precomputed plans.
  emitBodyLock(1, "if (mtCoarseUseDStatic && !mtVerilatorDualPathSelected) {\n");
  emitBodyLock(2, "const int dstaticRoundedWC = mtaskWorkerCount;\n");
  emitBodyLock(2, "if (dstaticRoundedWC == 1) {\n");
  emitBodyLock(3, "mtRunCoarseRegionStaticDispatch(regionIndex, dstaticRoundedWC, 0, regionBeginActiveWord, regionActiveWordSpan);\n");
  emitBodyLock(2, "} else if (mtWorkerPoolEnabled && mtWorkerPoolThreadCount + 1 >= dstaticRoundedWC) {\n");
  emitBodyLock(3, "mtWorkerPoolJobKind = 3;\n");
  emitBodyLock(3, "mtWorkerPoolCoarseRegionIndex = regionIndex;\n");
  emitBodyLock(3, "mtWorkerPoolCoarseLayerIndex = -1;\n");
  emitBodyLock(3, "mtWorkerPoolCoarseStaticRoundedWC = dstaticRoundedWC;\n");
  emitBodyLock(3, "mtWorkerPoolCoarseStaticBeginActiveWord = regionBeginActiveWord;\n");
  emitBodyLock(3, "mtWorkerPoolCoarseStaticActiveWordSpan = regionActiveWordSpan;\n");
  emitBodyLock(3, "mtWorkerPoolCurrentWorkerCount = dstaticRoundedWC;\n");
  emitBodyLock(3, "std::chrono::steady_clock::time_point mtPhaseBodyBegin;\n");
  emitBodyLock(3, "if (mtProfileEnabled) mtPhaseBodyBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(3, "mtWorkerPoolPost();\n");
  emitBodyLock(3, "mtRunCoarseRegionStaticDispatch(regionIndex, dstaticRoundedWC, 0, regionBeginActiveWord, regionActiveWordSpan);\n");
  emitBodyLock(3, "std::chrono::steady_clock::time_point mtPhaseWaitBegin;\n");
  emitBodyLock(3, "if (mtProfileEnabled) {\n");
  emitBodyLock(4, "mtPhaseWaitBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(4, "mtProfileCoarseBodyNs += std::chrono::duration_cast<std::chrono::nanoseconds>(mtPhaseWaitBegin - mtPhaseBodyBegin).count();\n");
  emitBodyLock(3, "}\n");
  emitBodyLock(3, "mtWorkerPoolWaitForDone(dstaticRoundedWC - 1);\n");
  emitBodyLock(3, "if (mtProfileEnabled) mtProfileCoarseWaitNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtPhaseWaitBegin).count();\n");
  emitBodyLock(2, "} else {\n");
  emitBodyLock(3, "std::vector<std::thread> workers;\n");
  emitBodyLock(3, "workers.reserve(dstaticRoundedWC);\n");
  emitBodyLock(3, "for (int worker = 0; worker < dstaticRoundedWC; worker ++) {\n");
  emitBodyLock(4, "workers.emplace_back([&, worker]() { mtRunCoarseRegionStaticDispatch(regionIndex, dstaticRoundedWC, worker, regionBeginActiveWord, regionActiveWordSpan); });\n");
  emitBodyLock(3, "}\n");
  emitBodyLock(3, "for (std::thread &worker : workers) worker.join();\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(1, "} else {\n");
  emitBodyLock(1, "if (mtaskWorkerCount == 1) {\n");
  if (globalConfig.MtCoarseWorkerPolicyMode == "profitable") {
    emitBodyLock(2, "mtRunCoarseMTaskWorkerList(0, regionIndex, mtaskWorkerAssignments[0].data(), (int)mtaskWorkerAssignments[0].size());\n");
  } else {
    emitBodyLock(2, "mtRunCoarseMTaskWorkerRange(0, regionIndex, 0, regionMTaskCount);\n");
  }
  emitBodyLock(1, "} else if (mtWorkerPoolEnabled && mtWorkerPoolThreadCount + 1 >= mtaskWorkerCount) {\n");
  emitBodyLock(2, "mtWorkerPoolJobKind = 2;\n");
  emitBodyLock(2, "if (mtProfileEnabled && mtVerilatorDualPathSelected) mtProfileVerilatorDualPathWorkerPoolDispatches ++;\n");
  emitBodyLock(2, "mtWorkerPoolCoarseRegionIndex = regionIndex;\n");
  emitBodyLock(2, "mtWorkerPoolCoarseLayerIndex = -1;\n");
  emitBodyLock(2, "mtWorkerPoolCurrentWorkerCount = mtaskWorkerCount;\n");
  emitBodyLock(2, "for (int worker = 0; worker < mtaskWorkerCount; worker ++) {\n");
  if (globalConfig.MtCoarseWorkerPolicyMode == "profitable") {
    emitBodyLock(3, "mtWorkerPoolMTaskAssignments[(size_t)worker] = mtaskWorkerAssignments[(size_t)worker];\n");
    emitBodyLock(3, "mtWorkerPoolChunks[(size_t)worker].begin = 0;\n");
    emitBodyLock(3, "mtWorkerPoolChunks[(size_t)worker].end = (int)mtWorkerPoolMTaskAssignments[(size_t)worker].size();\n");
  } else {
    emitBodyLock(3, "mtWorkerPoolChunks[(size_t)worker].begin = (regionMTaskCount * worker) / mtaskWorkerCount;\n");
    emitBodyLock(3, "mtWorkerPoolChunks[(size_t)worker].end = (regionMTaskCount * (worker + 1)) / mtaskWorkerCount;\n");
  }
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "std::chrono::steady_clock::time_point mtPhaseBodyBegin;\n");
  emitBodyLock(2, "if (mtProfileEnabled) mtPhaseBodyBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(2, "mtWorkerPoolPost();\n");
  if (globalConfig.MtCoarseWorkerPolicyMode == "profitable") {
    emitBodyLock(2, "mtRunCoarseMTaskWorkerList(0, regionIndex, mtaskWorkerAssignments[0].data(), (int)mtaskWorkerAssignments[0].size());\n");
  } else {
    emitBodyLock(2, "mtRunCoarseMTaskWorkerRange(0, regionIndex, mtWorkerPoolChunks[0].begin, mtWorkerPoolChunks[0].end);\n");
  }
  emitBodyLock(2, "std::chrono::steady_clock::time_point mtPhaseWaitBegin;\n");
  emitBodyLock(2, "if (mtProfileEnabled) {\n");
  emitBodyLock(3, "mtPhaseWaitBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(3, "mtProfileCoarseBodyNs += std::chrono::duration_cast<std::chrono::nanoseconds>(mtPhaseWaitBegin - mtPhaseBodyBegin).count();\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "mtWorkerPoolWaitForDone(mtaskWorkerCount - 1);\n");
  emitBodyLock(2, "if (mtProfileEnabled) mtProfileCoarseWaitNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtPhaseWaitBegin).count();\n");
  emitBodyLock(1, "} else {\n");
  emitBodyLock(2, "std::vector<std::thread> workers;\n");
  emitBodyLock(2, "workers.reserve(mtaskWorkerCount);\n");
  emitBodyLock(2, "for (int worker = 0; worker < mtaskWorkerCount; worker ++) {\n");
  if (globalConfig.MtCoarseWorkerPolicyMode == "profitable") {
    emitBodyLock(3, "workers.emplace_back([&, worker]() { mtRunCoarseMTaskWorkerList(worker, regionIndex, mtaskWorkerAssignments[(size_t)worker].data(), (int)mtaskWorkerAssignments[(size_t)worker].size()); });\n");
  } else {
    emitBodyLock(3, "int mtaskBegin = (regionMTaskCount * worker) / mtaskWorkerCount;\n");
    emitBodyLock(3, "int mtaskEnd = (regionMTaskCount * (worker + 1)) / mtaskWorkerCount;\n");
    emitBodyLock(3, "workers.emplace_back([&, worker, mtaskBegin, mtaskEnd]() { mtRunCoarseMTaskWorkerRange(worker, regionIndex, mtaskBegin, mtaskEnd); });\n");
  }
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "for (std::thread &worker : workers) worker.join();\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "}\n");  // close `if (mtCoarseUseDStatic) ... else { ... }`
  emitBodyLock(1, "std::chrono::steady_clock::time_point mtProfileMergeBegin;\n");
  emitBodyLock(1, "if (mtProfileEnabled) mtProfileMergeBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(1, "for (int worker = 0; worker < mtaskWorkerCount; worker ++) {\n");
  emitBodyLock(2, "for (const ActivationDeltaEntry &entry : mtWorkerDeltas[worker].entries) {\n");
  emitBodyLock(3, "int localWord = entry.idx - regionBeginActiveWord;\n");
  emitBodyLock(3, "if (localWord >= 0 && localWord < regionActiveWordSpan) mtWorkerCoarseFlags[worker][localWord] |= (uint%d_t)entry.mask;\n", ACTIVE_WIDTH);
  emitBodyLock(3, "else activeFlags[entry.idx] |= (uint%d_t)entry.mask;\n", ACTIVE_WIDTH);
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "if (mtWorkerDeltas[worker].allActive) {\n");
  emitBodyLock(3, "for (int word = 0; word < regionActiveWordSpan; word ++) mtWorkerCoarseFlags[worker][word] = (uint%d_t)-1;\n", ACTIVE_WIDTH);
  emitBodyLock(3, "for (int word = 0; word < %d; word ++) activeFlags[word] = (uint%d_t)-1;\n", activeFlagNum, ACTIVE_WIDTH);
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "for (int word = 0; word < regionActiveWordSpan; word ++) coarseActiveWords[word] |= mtWorkerCoarseFlags[worker][word];\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "if (mtProfileEnabled) {\n");
  emitBodyLock(2, "mtProfileCoarseMergeWordScans += (uint64_t)mtaskWorkerCount * (uint64_t)regionActiveWordSpan;\n");
  emitBodyLock(2, "for (int worker = 0; worker < mtaskWorkerCount; worker ++) {\n");
  emitBodyLock(3, "mtProfileCoarseActivationDeltaEntries += mtProfileLocalActivationDeltaEntries[worker];\n");
  emitBodyLock(3, "mtProfileActivationDeltaEntries += mtProfileLocalActivationDeltaEntries[worker];\n");
  emitBodyLock(3, "mtProfileCoarseActivationDeltaEntries += mtWorkerDeltas[worker].entries.size();\n");
  emitBodyLock(3, "mtProfileActivationDeltaEntries += mtWorkerDeltas[worker].entries.size();\n");
  emitBodyLock(3, "if (mtProfileLocalActivationDeltaMaxEntries[worker] > mtProfileActivationDeltaMaxEntriesPerWorker) mtProfileActivationDeltaMaxEntriesPerWorker = mtProfileLocalActivationDeltaMaxEntries[worker];\n");
  emitBodyLock(3, "if (mtWorkerDeltas[worker].entries.size() > mtProfileActivationDeltaMaxEntriesPerWorker) mtProfileActivationDeltaMaxEntriesPerWorker = mtWorkerDeltas[worker].entries.size();\n");
  emitBodyLock(3, "if (mtWorkerDeltas[worker].allActive) mtProfileActivationDeltaActivateAllCount ++;\n");
  emitBodyLock(3, "mtProfileWorkerTaskCount[(size_t)worker] += mtProfileLocalWorkerTaskCount[worker];\n");
  emitBodyLock(3, "mtProfilePureTasks += mtProfileLocalWorkerTaskCount[worker];\n");
  emitBodyLock(3, "bool mtTraceCycleActive = mtProfileDynamicTraceFile != nullptr && cycles >= mtProfileDynamicTraceCycleStart && cycles < mtProfileDynamicTraceCycleLimit;\n");
  emitBodyLock(3, "for (int cppId : mtProfileLocalTaskIds[worker]) { if (cppId >= 0 && cppId < %d) { mtProfileTaskExecCount[cppId] ++; if (mtTraceCycleActive) mtProfileDynamicTraceTaskIds.push_back(cppId); } }\n", superId);
  emitBodyLock(2, "}\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "if (mtProfileEnabled) mtProfileMergeWallNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileMergeBegin).count();\n");
  emitBodyLock(1, "if (mtProfileEnabled) mtProfileBatchWallNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileBatchBegin).count();\n");
  if (globalConfig.MtCoarseProfitabilityMode == "static") {
    emitBodyLock(1, "if (mtProfileEnabled && mtaskWorkerCount > 1) mtProfileTrueParallelWallNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileBatchBegin).count();\n");
  } else {
    emitBodyLock(1, "if (mtProfileEnabled && workerCount > 1) mtProfileTrueParallelWallNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileBatchBegin).count();\n");
  }
  emitBodyLock(1, "return;\n");
  emitBodyLock(1, "}\n");  // close `if (mtCoarseUseMTaskRuntime)`
  emitBodyLock(1, "for (int layer = 0; layer < regionLayerCount; layer ++) {\n");
  emitBodyLock(2, "for (int worker = 0; worker < workerCount; worker ++) {\n");
  emitBodyLock(3, "mtWorkerDeltas[worker].clear();\n");
  emitBodyLock(3, "mtWorkerCoarseFlags[worker].assign(coarseActiveWords, coarseActiveWords + regionActiveWordSpan);\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "int layerTaskCount = 0;\n");
  emitBodyLock(2, "switch (regionIndex) {\n");
  regionIndex = 0;
  for (const MtCoarseRegion& region : coarsePlan.regions) {
    if (!region.runtimeEligible) continue;
    emitBodyLock(3, "case %d:\n", regionIndex);
    emitBodyLock(4, "switch (layer) {\n");
    for (size_t layerIdx = 0; layerIdx < region.layers.size(); layerIdx ++) {
      emitBodyLock(5, "case %zu: layerTaskCount = %zu; break;\n", layerIdx, region.layers[layerIdx].taskCppIds.size());
    }
    emitBodyLock(5, "default: layerTaskCount = 0; break;\n");
    emitBodyLock(4, "}\n");
    emitBodyLock(4, "break;\n");
    regionIndex ++;
  }
  emitBodyLock(3, "default:\n");
  emitBodyLock(4, "layerTaskCount = 0;\n");
  emitBodyLock(4, "break;\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "if (layerTaskCount <= 0) continue;\n");
  emitBodyLock(2, "int layerWorkerCount = workerCount;\n");
  emitBodyLock(2, "if (layerWorkerCount > layerTaskCount) layerWorkerCount = layerTaskCount;\n");
  emitBodyLock(2, "if (layerWorkerCount < 1) layerWorkerCount = 1;\n");
  emitBodyLock(2, "if (mtProfileEnabled) {\n");
  emitBodyLock(3, "mtProfileCoarseLayerDispatches ++;\n");
  emitBodyLock(3, "mtProfileCoarseWorkerJobs += layerWorkerCount;\n");
  emitBodyLock(3, "mtProfileCoarseFlagWordCopies += (uint64_t)workerCount * (uint64_t)regionActiveWordSpan;\n");
  emitBodyLock(3, "mtProfileCoarseEstimatedBarrierCount ++;\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "if (layerWorkerCount == 1) {\n");
  emitBodyLock(3, "mtRunCoarseLayerWorkerRange(0, regionIndex, layer, 0, layerTaskCount);\n");
  emitBodyLock(2, "} else if (mtWorkerPoolEnabled && mtWorkerPoolThreadCount + 1 >= layerWorkerCount) {\n");
  emitBodyLock(3, "mtWorkerPoolJobKind = 1;\n");
  emitBodyLock(3, "mtWorkerPoolCoarseRegionIndex = regionIndex;\n");
  emitBodyLock(3, "mtWorkerPoolCoarseLayerIndex = layer;\n");
  emitBodyLock(3, "mtWorkerPoolCurrentWorkerCount = layerWorkerCount;\n");
  emitBodyLock(3, "for (int worker = 0; worker < layerWorkerCount; worker ++) {\n");
  emitBodyLock(4, "mtWorkerPoolChunks[(size_t)worker].begin = (layerTaskCount * worker) / layerWorkerCount;\n");
  emitBodyLock(4, "mtWorkerPoolChunks[(size_t)worker].end = (layerTaskCount * (worker + 1)) / layerWorkerCount;\n");
  emitBodyLock(3, "}\n");
  emitBodyLock(3, "std::chrono::steady_clock::time_point mtPhaseBodyBegin;\n");
  emitBodyLock(3, "if (mtProfileEnabled) mtPhaseBodyBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(3, "mtWorkerPoolPost();\n");
  emitBodyLock(3, "mtRunCoarseLayerWorkerRange(0, regionIndex, layer, mtWorkerPoolChunks[0].begin, mtWorkerPoolChunks[0].end);\n");
  emitBodyLock(3, "std::chrono::steady_clock::time_point mtPhaseWaitBegin;\n");
  emitBodyLock(3, "if (mtProfileEnabled) {\n");
  emitBodyLock(4, "mtPhaseWaitBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(4, "mtProfileCoarseBodyNs += std::chrono::duration_cast<std::chrono::nanoseconds>(mtPhaseWaitBegin - mtPhaseBodyBegin).count();\n");
  emitBodyLock(3, "}\n");
  emitBodyLock(3, "mtWorkerPoolWaitForDone(layerWorkerCount - 1);\n");
  emitBodyLock(3, "if (mtProfileEnabled) mtProfileCoarseWaitNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtPhaseWaitBegin).count();\n");
  emitBodyLock(2, "} else {\n");
  emitBodyLock(3, "std::vector<std::thread> workers;\n");
  emitBodyLock(3, "workers.reserve(layerWorkerCount);\n");
  emitBodyLock(3, "for (int worker = 0; worker < layerWorkerCount; worker ++) {\n");
  emitBodyLock(4, "int chunkBegin = (layerTaskCount * worker) / layerWorkerCount;\n");
  emitBodyLock(4, "int chunkEnd = (layerTaskCount * (worker + 1)) / layerWorkerCount;\n");
  emitBodyLock(4, "workers.emplace_back([&, worker, chunkBegin, chunkEnd]() { mtRunCoarseLayerWorkerRange(worker, regionIndex, layer, chunkBegin, chunkEnd); });\n");
  emitBodyLock(3, "}\n");
  emitBodyLock(3, "for (std::thread &worker : workers) worker.join();\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "std::chrono::steady_clock::time_point mtProfileMergeBegin;\n");
  emitBodyLock(2, "if (mtProfileEnabled) mtProfileMergeBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(2, "for (int worker = 0; worker < layerWorkerCount; worker ++) {\n");
  emitBodyLock(3, "for (int word = 0; word < regionActiveWordSpan; word ++) coarseActiveWords[word] |= mtWorkerCoarseFlags[worker][word];\n");
  emitBodyLock(3, "for (const ActivationDeltaEntry &entry : mtWorkerDeltas[worker].entries) {\n");
  emitBodyLock(4, "int localWord = entry.idx - regionBeginActiveWord;\n");
  emitBodyLock(4, "if (localWord >= 0 && localWord < regionActiveWordSpan) coarseActiveWords[localWord] |= (uint%d_t)entry.mask;\n", ACTIVE_WIDTH);
  emitBodyLock(4, "else activeFlags[entry.idx] |= (uint%d_t)entry.mask;\n", ACTIVE_WIDTH);
  emitBodyLock(3, "}\n");
  emitBodyLock(3, "if (mtWorkerDeltas[worker].allActive) {\n");
  emitBodyLock(4, "for (int word = 0; word < regionActiveWordSpan; word ++) coarseActiveWords[word] = (uint%d_t)-1;\n", ACTIVE_WIDTH);
  emitBodyLock(4, "for (int word = 0; word < %d; word ++) activeFlags[word] = (uint%d_t)-1;\n", activeFlagNum, ACTIVE_WIDTH);
  emitBodyLock(3, "}\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "if (mtProfileEnabled) {\n");
  emitBodyLock(3, "mtProfileCoarseMergeWordScans += (uint64_t)layerWorkerCount * (uint64_t)regionActiveWordSpan;\n");
  emitBodyLock(3, "for (int worker = 0; worker < layerWorkerCount; worker ++) {\n");
  emitBodyLock(4, "mtProfileCoarseActivationDeltaEntries += mtWorkerDeltas[worker].entries.size();\n");
  emitBodyLock(4, "mtProfileActivationDeltaEntries += mtWorkerDeltas[worker].entries.size();\n");
  emitBodyLock(4, "if (mtWorkerDeltas[worker].entries.size() > mtProfileActivationDeltaMaxEntriesPerWorker) mtProfileActivationDeltaMaxEntriesPerWorker = mtWorkerDeltas[worker].entries.size();\n");
  emitBodyLock(4, "if (mtWorkerDeltas[worker].allActive) mtProfileActivationDeltaActivateAllCount ++;\n");
  emitBodyLock(4, "mtProfileWorkerTaskCount[(size_t)worker] += mtProfileLocalWorkerTaskCount[worker];\n");
  emitBodyLock(4, "mtProfilePureTasks += mtProfileLocalWorkerTaskCount[worker];\n");
  emitBodyLock(4, "bool mtTraceCycleActive = mtProfileDynamicTraceFile != nullptr && cycles >= mtProfileDynamicTraceCycleStart && cycles < mtProfileDynamicTraceCycleLimit;\n");
  emitBodyLock(4, "for (int cppId : mtProfileLocalTaskIds[worker]) { if (cppId >= 0 && cppId < %d) { mtProfileTaskExecCount[cppId] ++; if (mtTraceCycleActive) mtProfileDynamicTraceTaskIds.push_back(cppId); } }\n", superId);
  emitBodyLock(3, "}\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "if (mtProfileEnabled) mtProfileMergeWallNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileMergeBegin).count();\n");
  emitBodyLock(2, "if (mtProfileEnabled) {\n");
  emitBodyLock(3, "for (int worker = 0; worker < workerCount; worker ++) {\n");
  emitBodyLock(4, "mtProfileLocalWorkerTaskCount[worker] = 0;\n");
  emitBodyLock(4, "mtProfileLocalTaskIds[worker].clear();\n");
  emitBodyLock(3, "}\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "if (mtProfileEnabled) mtProfileBatchWallNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileBatchBegin).count();\n");
  emitBodyLock(1, "if (mtProfileEnabled && workerCount > 1) mtProfileTrueParallelWallNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileBatchBegin).count();\n");
  emitBodyLock(0, "}\n");
}

int graph::genActivateSeqHelpers(bool buffered) {
    std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapForInvocation();
    for (int idx = 0; idx < superId; idx ++) {
      genMtTaskHelper(cppId2Super[idx], buffered, "ActiveBuffer");
    }

    emitFuncDecl(0, "void S%s::subStep0() {\n", name.c_str());
    int indent = 1;
    int nextSubStepIdx = 1;
    std::string nextFuncDef = format("void S%s::subStep%d()", name.c_str(), nextSubStepIdx);
    bool prevActiveWhole = false;
    for (int idx = 0; idx < superId; idx ++) {
      int id;
      uint64_t mask;
      std::tie(id, mask) = setIdxMask(idx);
      int offset = idx % ACTIVE_WIDTH;
      if (offset == 0) {
        if (prevActiveWhole) {
          emitBodyLock(--indent, "}\n");
        }
        prevActiveWhole = true;
        for (int j = 0; j < ACTIVE_WIDTH && idx + j < superId; j ++) {
          if (isAlwaysActive(idx + j)) prevActiveWhole = false;
        }
        if (prevActiveWhole) {
          bool newFile = __emitSrc(indent ++, true, false, nextFuncDef.c_str(), "if(unlikely(activeFlags[%d] != 0)) {\n", id);
          if (newFile) {
            nextFuncDef = format("void S%s::subStep%d()", name.c_str(), ++ nextSubStepIdx);
          }
          emitBodyLock(indent, "uint%d_t oldFlag = activeFlags[%d];\n", ACTIVE_WIDTH, id);
          emitBodyLock(indent, "activeFlags[%d] = 0;\n", id);
          emitBodyLock(indent, "if (mtProfileEnabled) mtProfileActiveWordCount ++;\n");
        } else if (buffered) {
          emitBodyLock(indent, "uint%d_t activeWord%d = activeFlags[%d];\n", ACTIVE_WIDTH, id, id);
        }
      }
      SuperNode* super = cppId2Super[idx];
      std::string flagName = prevActiveWhole ? "oldFlag" : (buffered ? format("activeWord%d", id) : format("activeFlags[%d]", id));
      indent = genNodeStepStart(super, mask, idx, flagName, indent, false);
      emitBodyLock(indent ++, "{\n");
      if (buffered) {
        emitBodyLock(indent, "ActiveBuffer mtBuffer;\n");
        emitBodyLock(indent, "mtBuffer.clear();\n");
        emitBodyLock(indent, "std::chrono::steady_clock::time_point mtProfileTaskBegin;\n");
        emitBodyLock(indent, "if (mtProfileEnabled) mtProfileTaskBegin = std::chrono::steady_clock::now();\n");
        emitBodyLock(indent, "mtTask%d(%s, mtBuffer);\n", idx, flagName.c_str());
        emitBodyLock(indent, "recordMtProfileTask(%d, %s, mtProfileEnabled ? std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileTaskBegin).count() : 0);\n",
                     idx, mtTasks[idx].taskKind == "pure_compute" ? "true" : "false");
        emitBodyLock(indent, "mtBuffer.mergeFrom(activeFlags);\n");
      } else {
        emitBodyLock(indent, "std::chrono::steady_clock::time_point mtProfileTaskBegin;\n");
        emitBodyLock(indent, "if (mtProfileEnabled) mtProfileTaskBegin = std::chrono::steady_clock::now();\n");
        emitBodyLock(indent, "mtTask%d(%s);\n", idx, flagName.c_str());
        emitBodyLock(indent, "recordMtProfileTask(%d, %s, mtProfileEnabled ? std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileTaskBegin).count() : 0);\n",
                     idx, mtTasks[idx].taskKind == "pure_compute" ? "true" : "false");
      }
      emitBodyLock(-- indent, "}\n");
      indent = genNodeStepEnd(super, indent, false);
    }
    emitBodyLock(--indent, "}\n");
    if (prevActiveWhole) emitBodyLock(--indent, "}\n");

    return nextSubStepIdx - 1;
}

int graph::genActivateMtHelpers(int serialFastSubStepMax, const std::string& serialFastSuffix) {
    std::map<int, MtTaskInfo> mtTasks;
    MtPureBatchPlan batchPlan;
    MtCoarseRegionPlan coarsePlan;
    { EmitPhaseTimer prologueTimer("Final.mtHelpers.prologue");
      mtTasks = buildMtTaskInfoMapForInvocation();
      { EmitPhaseTimer planTimer("Final.mtHelpers.prologuePlan"); batchPlan = planMtPureBatches(mtTasks); }
      { EmitPhaseTimer planTimer("Final.mtHelpers.prologueCoarse"); coarsePlan = planMtCoarseRegionsForInvocation(); }
    }
    std::map<int, int> batchEndByStart;
    for (auto batch : batchPlan.batches) {
      batchEndByStart[batch.first] = batch.second;
    }
    std::map<int, int> coarseRegionIndexByStart;
    std::map<int, MtCoarseRegion> coarseRegionByStart;
    if (globalConfig.MtBatchFormationMode == "coarse") {
      int regionIndex = 0;
      for (const MtCoarseRegion& region : coarsePlan.regions) {
        if (!region.runtimeEligible) continue;
        coarseRegionIndexByStart[region.beginCppId] = regionIndex;
        coarseRegionByStart[region.beginCppId] = region;
        regionIndex ++;
      }
    }
    {
      EmitPhaseTimer helperTimer("Final.mtTaskHelpers");
      // Per-super mtTaskN definitions are independent emission units: each
      // renders one self-contained function reading only frozen state
      // (cppId2Super, super2ResetId) plus the per-thread emission context.
      // Render in parallel, assemble in cppId order - byte-identical to
      // sequential emission.
      const bool denseOnlyCodegen = mtUseDenseOnlyCodegen();
      emitUnitsParallel((size_t)superId, [this, denseOnlyCodegen](size_t unit) {
        int idx = (int)unit;
        // Dense-only: the buffered mtTaskN(flag, ActivationDelta&) variant is
        // referenced only by the sparse coarse/pure-batch dispatch, which is
        // not emitted. Keep the unbuffered mtTaskN(flag) bodies.
        if (!denseOnlyCodegen) genMtTaskHelper(cppId2Super[idx], true, "ActivationDelta");
        genMtTaskHelper(cppId2Super[idx], false, "ActivationDelta");
      });
    }
    { EmitPhaseTimer runnersTimer("Final.mtTaskRunners"); genMtTaskRunner(batchPlan); }
    if (globalConfig.MtBatchFormationMode == "coarse" && !mtUseDenseOnlyCodegen()) {
      EmitPhaseTimer coarseRunnerTimer("Final.mtCoarseRegionRunner");
      genMtCoarseRegionRunner(coarsePlan);
    }
    if (mtUseDenseOnlyCodegen()) {
      // Dense-only model: the plain serial subStepN() scan is sparse-dispatch
      // fallback text (coarse-region dispatch, pure-batch calls, scalar task
      // dispatch). step() aborts instead of falling through to it, so no
      // subStepN() bodies or declarations are emitted at all.
      return -1;
    }

    EmitPhaseTimer subStepTimer("Final.subSteps");
    emitFuncDecl(0, "void S%s::subStep0() {\n", name.c_str());
    int indent = 1;
    (void)serialFastSubStepMax;
    (void)serialFastSuffix;
    bool directInlineFallback = mtUseDirectInlineFallback();
    bool directInlineSerialFallback = mtUseDirectInlineSerialFallback();
    bool directInlineWorker0Fallback = mtUseDirectInlineWorker0Fallback();
    bool profileOffDirectSerial = mtUseProfileOffDirectSerialFallback();
    bool profileOffActiveWordCount = mtUseProfileOffActiveWordCount();
    bool inlineSmallPureBatches = mtUseInlineSmallPureBatches();
    bool inlineSmallPureBatchBodies = mtUseInlineSmallPureBatchBodies();
    bool inlineSmallPureBatchMaskGuard = mtUseInlineSmallPureBatchMaskGuard();
    bool splitMixedStepGuards = mtUseSplitMixedStepGuards();
    int nextSubStepIdx = 1;
    mtStepActiveWordGuards.clear();
    mtStepActiveWordGuardable.clear();
    int currentSubStepIdx = 0;
    auto ensureMtSubStepGuard = [&](int subStepIdx) {
      if ((int)mtStepActiveWordGuards.size() <= subStepIdx) {
        mtStepActiveWordGuards.resize((size_t)subStepIdx + 1);
        mtStepActiveWordGuardable.resize((size_t)subStepIdx + 1, 1);
      }
    };
    auto recordMtSubStepGuardWord = [&](int subStepIdx, int activeWord) {
      ensureMtSubStepGuard(subStepIdx);
      std::vector<int>& guards = mtStepActiveWordGuards[(size_t)subStepIdx];
      if (std::find(guards.begin(), guards.end(), activeWord) == guards.end()) guards.push_back(activeWord);
    };
    auto markMtSubStepUnguarded = [&](int subStepIdx) {
      ensureMtSubStepGuard(subStepIdx);
      mtStepActiveWordGuardable[(size_t)subStepIdx] = 0;
    };
    auto currentMtSubStepHasGuardedPrefix = [&]() {
      return currentSubStepIdx < (int)mtStepActiveWordGuards.size() &&
             currentSubStepIdx < (int)mtStepActiveWordGuardable.size() &&
             mtStepActiveWordGuardable[(size_t)currentSubStepIdx] &&
             !mtStepActiveWordGuards[(size_t)currentSubStepIdx].empty();
    };
    ensureMtSubStepGuard(currentSubStepIdx);
    std::string nextFuncDef = format("void S%s::subStep%d()", name.c_str(), nextSubStepIdx);
    bool prevActiveWhole = false;
    for (int idx = 0; idx < superId; idx ++) {
      int id;
      uint64_t mask;
      std::tie(id, mask) = setIdxMask(idx);
      int offset = idx % ACTIVE_WIDTH;
      auto coarseIter = coarseRegionIndexByStart.find(idx);
      if (coarseIter != coarseRegionIndexByStart.end()) {
        const MtCoarseRegion& region = coarseRegionByStart[idx];
        if (prevActiveWhole) emitBodyLock(--indent, "}\n");
        prevActiveWhole = false;
        emitBodyLock(indent, "uint%d_t mtCoarseWords%d[%d];\n", ACTIVE_WIDTH, idx, region.activeWordSpan);
        std::string coarseGuard;
        for (int word = 0; word < region.activeWordSpan; word ++) {
          int activeWord = region.beginActiveWord + word;
          emitBodyLock(indent, "mtCoarseWords%d[%d] = activeFlags[%d];\n", idx, word, activeWord);
          emitBodyLock(indent, "activeFlags[%d] = 0;\n", activeWord);
          if (!coarseGuard.empty()) coarseGuard += " | ";
          coarseGuard += format("mtCoarseWords%d[%d]", idx, word);
        }
        for (int word = 0; word < region.activeWordSpan; word ++) recordMtSubStepGuardWord(currentSubStepIdx, region.beginActiveWord + word);
        emitBodyLock(indent ++, "if(unlikely((%s) != 0)) {\n", coarseGuard.c_str());
        if (!profileOffDirectSerial) {
          emitBodyLock(indent, "if (mtProfileEnabled) {\n");
          for (int word = 0; word < region.activeWordSpan; word ++) {
            emitBodyLock(indent + 1, "if (mtCoarseWords%d[%d] != 0) mtProfileActiveWordCount ++;\n", idx, word);
          }
          emitBodyLock(indent, "}\n");
        }
        // A43/A44: default-off/env-on direct serial fallback before mtRunCoarseRegion.
        // Clean regions only: original mtTaskN(flag) bodies and only pure_compute plus
        // the explicit A44 safe-serial allowlist; unbuffered mtTaskN/genSuperEval keeps
        // same-word activation in the local flag and cross-word activation in activeFlags.
        // This is the ST-parity floor path: it bypasses worker flag copies, post/wait,
        // SCoarseTaskRef member-pointer dispatch, mergeAfter, and final merge.
        bool regionHasNonPure = false;
        for (int rcid = region.beginCppId; rcid < region.endCppId; rcid ++) {
          auto mtIter = mtTasks.find(rcid);
          if (mtIter == mtTasks.end() || hasWorker0OnlyReason(mtIter->second.serialReasons) || !hasOnlyA44DirectFallbackReasons(mtIter->second.serialReasons)) regionHasNonPure = true;
        }
        bool regionCleanSerialFallback = !regionHasNonPure;
        auto emitCoarseInlineWord = [&](int word, int wordIndent) {
          int activeWord = region.beginActiveWord + word;
          emitBodyLock(wordIndent, "uint%d_t coarseInlineFlag%d_%d = mtCoarseWords%d[%d] | activeFlags[%d];\n",
                       ACTIVE_WIDTH, idx, word, idx, word, activeWord);
          auto emitCoarseInlineTask = [&](int cppId, int taskIndent) {
            if (profileOffDirectSerial) {
              if (directInlineFallback) {
                genSuperEval(cppId2Super[cppId], format("coarseInlineFlag%d_%d", idx, word), "", taskIndent, true);
              } else {
                emitBodyLock(taskIndent, "mtTask%d(coarseInlineFlag%d_%d);\n", cppId, idx, word);
              }
              if (mtUseActivationEventTraceCodegen()) emitBodyLock(taskIndent, "recordMtProfileDynamicTraceTask(%d);\n", cppId);
            } else {
              emitBodyLock(taskIndent ++, "if (mtProfileEnabled) {\n");
              emitBodyLock(taskIndent, "std::chrono::steady_clock::time_point mtProfileTaskBegin = std::chrono::steady_clock::now();\n");
              emitBodyLock(taskIndent, "mtTask%d(coarseInlineFlag%d_%d);\n", cppId, idx, word);
              emitBodyLock(taskIndent, "recordMtProfileTask(%d, %s, std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileTaskBegin).count());\n", cppId, mtTasks[cppId].taskKind == "pure_compute" ? "true" : "false");
              emitBodyLock(--taskIndent, "} else {\n");
              if (directInlineFallback) {
                genSuperEval(cppId2Super[cppId], format("coarseInlineFlag%d_%d", idx, word), "", taskIndent, true);
              } else {
                emitBodyLock(taskIndent, "mtTask%d(coarseInlineFlag%d_%d);\n", cppId, idx, word);
              }
              emitBodyLock(--taskIndent, "}\n");
            }
          };
          emitBodyLock(wordIndent, "activeFlags[%d] = 0;\n", activeWord);
          emitBodyLock(wordIndent ++, "if (coarseInlineFlag%d_%d) {\n", idx, word);
          auto emitFixedBitScan = [&](int scanIndent) {
            for (int bit = 0; bit < ACTIVE_WIDTH; bit ++) {
              int cppId = activeWord * ACTIVE_WIDTH + bit;
              if (cppId >= region.endCppId) break;
              if (cppId < region.beginCppId) continue;
              emitBodyLock(scanIndent ++, "if (coarseInlineFlag%d_%d & 0x%lx) {\n", idx, word, (uint64_t)1 << bit);
              emitCoarseInlineTask(cppId, scanIndent);
              emitBodyLock(--scanIndent, "}\n");
            }
          };
          emitFixedBitScan(wordIndent);
          emitBodyLock(--wordIndent, "}\n");
        };
        auto emitCoarseInlineSavedProfile = [&](int profileIndent) {
          if (profileOffDirectSerial) return;
          emitBodyLock(profileIndent, "if (mtProfileEnabled) {\n");
          emitBodyLock(profileIndent + 1, "int coarseInlineSavedWorkers%d = mtConfiguredWorkerCount;\n", idx);
          emitBodyLock(profileIndent + 1, "if (coarseInlineSavedWorkers%d > %d) coarseInlineSavedWorkers%d = %d;\n", idx, region.taskCount, idx, region.taskCount);
          emitBodyLock(profileIndent + 1, "if (coarseInlineSavedWorkers%d < 1) coarseInlineSavedWorkers%d = 1;\n", idx, idx);
          emitBodyLock(profileIndent + 1, "mtProfileCoarseSerialFallbackTaken ++;\n");
          emitBodyLock(profileIndent + 1, "mtProfileCoarseSerialFallbackActiveBits += (uint64_t)coarseInlineActiveBits%d;\n", idx);
          emitBodyLock(profileIndent + 1, "mtProfileCoarseSerialFallbackSavedWorkerJobs += (uint64_t)coarseInlineSavedWorkers%d;\n", idx);
          emitBodyLock(profileIndent + 1, "mtProfileCoarseSerialFallbackSavedFlagWordCopies += (uint64_t)coarseInlineSavedWorkers%d * (uint64_t)%d;\n", idx, region.activeWordSpan);
          emitBodyLock(profileIndent + 1, "mtProfileCoarseSerialFallbackSavedMergeWordScans += (uint64_t)coarseInlineSavedWorkers%d * (uint64_t)%d;\n", idx, region.activeWordSpan);
          emitBodyLock(profileIndent + 1, "mtProfileCoarseSerialFallbackSavedBarriers ++;\n");
          emitBodyLock(profileIndent, "}\n");
        };
        auto emitCoarseDispatchAndMerge = [&](int bodyIndent) {
          emitBodyLock(bodyIndent, "mtRunCoarseRegion(%d, mtCoarseWords%d);\n", coarseIter->second, idx);
          for (int word = 0; word < region.activeWordSpan; word ++) {
            int activeWord = region.beginActiveWord + word;
            emitBodyLock(bodyIndent, "activeFlags[%d] |= mtCoarseWords%d[%d];\n", activeWord, idx, word);
          }
        };
        const bool emitVerilatorDualPathCallerBypass = region.beginCppId == 4488 && region.endCppId == 5080;
        if (regionCleanSerialFallback) {
          if (emitVerilatorDualPathCallerBypass) {
            emitBodyLock(indent, "const bool mtVerilatorDualPathCallerSelected%d = mtUseVerilatorDualPath && ((mtVerilatorDualPathRegionIndex >= 0 && mtVerilatorDualPathRegionIndex == %d) || (mtVerilatorDualPathRegionIndex < 0 && mtVerilatorDualPathBeginCppId == %d && mtVerilatorDualPathEndCppId == %d));\n", idx, coarseIter->second, region.beginCppId, region.endCppId);
            emitBodyLock(indent ++, "if (unlikely(mtVerilatorDualPathCallerSelected%d)) {\n", idx);
            emitCoarseDispatchAndMerge(indent);
            emitBodyLock(--indent, "} else {\n");
            indent ++;
          }
          emitBodyLock(indent ++, "if (mtCoarseInlineThreshold > 0) {\n");
          emitBodyLock(indent, "int coarseInlineActiveBits%d = 0;\n", idx);
          if (mtUseStaticCoarseInlineBound() && profileOffDirectSerial) {
            int coarseInlineStaticMaxBits = region.activeWordSpan * ACTIVE_WIDTH;
            emitBodyLock(indent ++, "if (unlikely(mtCoarseInlineThreshold < %d)) {\n", coarseInlineStaticMaxBits);
            for (int word = 0; word < region.activeWordSpan; word ++) {
              emitBodyLock(indent, "coarseInlineActiveBits%d += __builtin_popcountll((unsigned long long)mtCoarseWords%d[%d]);\n", idx, idx, word);
            }
            emitBodyLock(--indent, "} else {\n");
            emitBodyLock(indent, "coarseInlineActiveBits%d = %d;\n", idx, coarseInlineStaticMaxBits);
            emitBodyLock(--indent, "}\n");
          } else {
            for (int word = 0; word < region.activeWordSpan; word ++) {
              emitBodyLock(indent, "coarseInlineActiveBits%d += __builtin_popcountll((unsigned long long)mtCoarseWords%d[%d]);\n", idx, idx, word);
            }
          }
          if (!profileOffDirectSerial) emitBodyLock(indent, "if (mtProfileEnabled) mtProfileCoarseSerialFallbackEligible ++;\n");
          emitBodyLock(indent ++, "if (coarseInlineActiveBits%d <= mtCoarseInlineThreshold) {\n", idx);
          emitCoarseInlineSavedProfile(indent);
          for (int word = 0; word < region.activeWordSpan; word ++) emitCoarseInlineWord(word, indent);
          emitBodyLock(--indent, "} else {\n");
          indent ++;
          emitCoarseDispatchAndMerge(indent);
          emitBodyLock(--indent, "}\n");
          emitBodyLock(--indent, "} else {\n");
          indent ++;
          emitCoarseDispatchAndMerge(indent);
          emitBodyLock(--indent, "}\n");
          if (emitVerilatorDualPathCallerBypass) emitBodyLock(--indent, "}\n");
        } else {
          if (regionHasNonPure) emitBodyLock(indent, "if (mtProfileEnabled && mtCoarseInlineThreshold > 0) mtProfileCoarseSerialFallbackNonPureExcluded ++;\n");
          emitBodyLock(indent, "mtRunCoarseRegion(%d, mtCoarseWords%d);\n", coarseIter->second, idx);
          for (int word = 0; word < region.activeWordSpan; word ++) {
            int activeWord = region.beginActiveWord + word;
            emitBodyLock(indent, "activeFlags[%d] |= mtCoarseWords%d[%d];\n", activeWord, idx, word);
          }
        }
        emitBodyLock(--indent, "}\n");
        idx = region.endCppId - 1;
        continue;
      }
      if (offset == 0) {
        if (prevActiveWhole) {
          emitBodyLock(--indent, "}\n");
        }
        prevActiveWhole = true;
        for (int j = 0; j < ACTIVE_WIDTH && idx + j < superId; j ++) {
          if (isAlwaysActive(idx + j)) prevActiveWhole = false;
        }
        if (prevActiveWhole) {
          bool newFile = __emitSrc(indent ++, true, false, nextFuncDef.c_str(), "if(unlikely(activeFlags[%d] != 0)) {\n", id);
          if (newFile) {
            currentSubStepIdx = nextSubStepIdx;
            ensureMtSubStepGuard(currentSubStepIdx);
            nextFuncDef = format("void S%s::subStep%d()", name.c_str(), ++ nextSubStepIdx);
          }
          recordMtSubStepGuardWord(currentSubStepIdx, id);
          emitBodyLock(indent, "uint%d_t oldFlag = activeFlags[%d];\n", ACTIVE_WIDTH, id);
          emitBodyLock(indent, "activeFlags[%d] = 0;\n", id);
          if (!profileOffActiveWordCount) emitBodyLock(indent, "if (mtProfileEnabled) mtProfileActiveWordCount ++;\n");
        } else {
          if (splitMixedStepGuards && currentMtSubStepHasGuardedPrefix()) {
            emitBodyLock(0, "}\n");
            emitFuncDecl(0, "%s {\n", nextFuncDef.c_str());
            currentSubStepIdx = nextSubStepIdx;
            ensureMtSubStepGuard(currentSubStepIdx);
            nextFuncDef = format("void S%s::subStep%d()", name.c_str(), ++ nextSubStepIdx);
          }
          emitBodyLock(indent, "uint%d_t activeWord%d = activeFlags[%d];\n", ACTIVE_WIDTH, id, id);
          markMtSubStepUnguarded(currentSubStepIdx);
        }
      }

      auto batchIter = batchEndByStart.find(idx);
      if (prevActiveWhole && batchIter != batchEndByStart.end()) {
        int batchEnd = batchIter->second;
        int batchLen = batchEnd - idx;
        if (batchLen > 1) {
          if (inlineSmallPureBatches && batchLen < 16) {
            emitBodyLock(indent ++, "if (likely(!mtProfileEnabled && mtMinBatchTasks > %d)) {\n", batchLen);
            uint64_t batchActiveMask = 0;
            for (int batchCppId = idx; batchCppId < batchEnd; batchCppId ++) {
              batchActiveMask |= (uint64_t)1 << (batchCppId % ACTIVE_WIDTH);
            }
            if (inlineSmallPureBatchMaskGuard) emitBodyLock(indent ++, "if (unlikely(oldFlag & 0x%lx)) {\n", batchActiveMask);
            for (int batchCppId = idx; batchCppId < batchEnd; batchCppId ++) {
              uint64_t batchMask = (uint64_t)1 << (batchCppId % ACTIVE_WIDTH);
              emitBodyLock(indent ++, "if (unlikely(oldFlag & 0x%lx)) {\n", batchMask);
              if (inlineSmallPureBatchBodies) {
                genSuperEval(cppId2Super[batchCppId], "oldFlag", "", indent, true);
              } else {
                emitBodyLock(indent, "mtTask%d(oldFlag);\n", batchCppId);
              }
              emitBodyLock(--indent, "}\n");
            }
            if (inlineSmallPureBatchMaskGuard) emitBodyLock(--indent, "}\n");
            emitBodyLock(--indent, "} else {\n");
            emitBodyLock(indent, "mtRunPureBatch(%d, %d, oldFlag);\n", idx, batchEnd);
            emitBodyLock(--indent, "}\n");
          } else {
            emitBodyLock(indent, "mtRunPureBatch(%d, %d, oldFlag);\n", idx, batchEnd);
          }
          idx = batchEnd - 1;
          continue;
        }
      }

      SuperNode* super = cppId2Super[idx];
      std::string flagName = prevActiveWhole ? "oldFlag" : format("activeWord%d", id);
      auto directInlineSerialIter = mtTasks.find(idx);
      bool directInlineSerialTask = directInlineSerialFallback && mtIsLevelDispatchMode() &&
                                    directInlineSerialIter != mtTasks.end() &&
                                    !hasWorker0OnlyReason(directInlineSerialIter->second.serialReasons) &&
                                    hasOnlyA44DirectFallbackReasons(directInlineSerialIter->second.serialReasons);
      bool directInlineWorker0Task = directInlineWorker0Fallback && mtIsLevelDispatchMode() &&
                                    directInlineSerialIter != mtTasks.end() &&
                                    !directInlineSerialTask &&
                                    hasOnlyA73Worker0SafeReasons(directInlineSerialIter->second.serialReasons);
      indent = genNodeStepStart(super, mask, idx, flagName, indent, false);
      if (profileOffDirectSerial) {
        // A77 D1-NARROW: emit only the lean profile-off body, matching the
        // SerialFast shape (genActivate). Drops both `if (mtProfileEnabled)`
        // wrappers and the outer scope; profile counters/timers are not emitted.
        if (directInlineSerialTask || directInlineWorker0Task) {
          genSuperEval(super, flagName, "", indent, true);
        } else {
          emitBodyLock(indent, "mtTask%d(%s);\n", idx, flagName.c_str());
        }
        if (mtUseActivationEventTraceCodegen()) emitBodyLock(indent, "recordMtProfileDynamicTraceTask(%d);\n", idx);
      } else {
      emitBodyLock(indent ++, "{\n");
      emitBodyLock(indent, "if (mtProfileEnabled) {\n");
      emitBodyLock(indent + 1, "if (mtProfileEffectiveWorkerCountHist.size() <= 1) mtProfileEffectiveWorkerCountHist.resize(2, 0);\n");
      emitBodyLock(indent + 1, "mtProfileEffectiveWorkerCountHist[1] ++;\n");
      if (!prevActiveWhole) {
        emitBodyLock(indent + 1, "mtProfileRejectNotActiveWhole ++;\n");
      } else if (isAlwaysActive(idx)) {
        emitBodyLock(indent + 1, "mtProfileRejectAlwaysActiveTask ++;\n");
      } else if (mtTasks[idx].taskKind != "pure_compute") {
        // under mt-level-dispatch, distinguish worker0-only (forced
        // serial by side-effect) from safe-serial that fell out of any region.
        if (mtIsLevelDispatchMode() && hasWorker0OnlyReason(mtTasks[idx].serialReasons)) {
          emitBodyLock(indent + 1, "mtProfileWorker0OnlyDispatched ++;\n");
        } else if (mtIsLevelDispatchMode()) {
          emitBodyLock(indent + 1, "mtProfileSafeSerialDispatched ++;\n");
        }
        emitBodyLock(indent + 1, "mtProfileRejectSerialTask ++;\n");
      } else if (mtTaskHasSameActiveWordHazard(mtTasks, idx)) {
        emitBodyLock(indent + 1, "mtProfileRejectSameActiveWordHazard ++;\n");
      } else {
        emitBodyLock(indent + 1, "mtProfileRejectBelowMinBatch ++;\n");
      }
      emitBodyLock(indent, "}\n");
      emitBodyLock(indent, "if (mtProfileEnabled) {\n");
      emitBodyLock(indent + 1, "std::chrono::steady_clock::time_point mtProfileTaskBegin = std::chrono::steady_clock::now();\n");
      emitBodyLock(indent + 1, "mtTask%d(%s);\n", idx, flagName.c_str());
      emitBodyLock(indent + 1, "recordMtProfileTask(%d, %s, std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileTaskBegin).count());\n",
                   idx, mtTasks[idx].taskKind == "pure_compute" ? "true" : "false");
      emitBodyLock(indent, "} else {\n");
      if (directInlineSerialTask || directInlineWorker0Task) {
        genSuperEval(super, flagName, "", indent + 1, true);
      } else {
        emitBodyLock(indent + 1, "mtTask%d(%s);\n", idx, flagName.c_str());
      }
      emitBodyLock(indent, "}\n");
      emitBodyLock(-- indent, "}\n");
      }
      indent = genNodeStepEnd(super, indent, false);
    }
    emitBodyLock(--indent, "}\n");
    if (prevActiveWhole) emitBodyLock(--indent, "}\n");

    return nextSubStepIdx - 1;
}

// GSIM_EMIT_RESET_CHUNK=<n>: subResetN bodies carry millions of scalar
// assignments in ONE function; the C++ frontend is superlinear on that shape
// (measured 1366s for one 5MB body vs 2.8s stubbed). Splitting the body into
// chain-called chunks of <n> statements (split only at top-level depth, so the
// IF/ELSE nesting is never cut) restores linear frontend cost. Default off.
static long mtResetChunkSize() {
  const char* e = std::getenv("GSIM_EMIT_RESET_CHUNK");
  // Default 4096: chunking is semantics-preserving (chain-called helpers under
  // identical re-opened guards) and removes a ~500x clang frontend blowup on
  // giant reset bodies. Set =0 to emit the legacy monolithic bodies.
  if (e == nullptr || e[0] == '\0') return 4096;
  long v = std::atol(e);
  if (v == 0) return 0;
  return v >= 256 ? v : 4096;
}
static std::vector<std::string>& mtResetChunkDecls() {
  static std::vector<std::string> decls;
  return decls;
}
void graph::genResetDef(SuperNode* super, bool isUIntReset, bool buffered, int resetId, int indent, const std::string& nameSuffix, bool emitActivation) {
  std::string activeSinkType = (globalConfig.MtHelperMode == "mt" ||
                                globalConfig.MtHelperMode == "mt-level-dispatch")
                                 ? "ActivationDelta" : "ActiveBuffer";
  std::string resetFuncName = format("subReset%s%d", nameSuffix.c_str(), resetId);
  bool traceSourceParam = mtUseActivationEventTraceCodegen() && emitActivation;
  if (buffered) {
    if (traceSourceParam) emitBodyLock(indent ++, "void S%s::%s(%s &nextActive, int32_t traceSourceCppId){ // %s reset\n", name.c_str(), resetFuncName.c_str(), activeSinkType.c_str(), isUIntReset ? "uint" : "async");
    else emitBodyLock(indent ++, "void S%s::%s(%s &nextActive){ // %s reset\n", name.c_str(), resetFuncName.c_str(), activeSinkType.c_str(), isUIntReset ? "uint" : "async");
  } else {
    if (traceSourceParam) emitBodyLock(indent ++, "void S%s::%s(int32_t traceSourceCppId){ // %s reset\n", name.c_str(), resetFuncName.c_str(), isUIntReset ? "uint" : "async");
    else emitBodyLock(indent ++, "void S%s::%s(){ // %s reset\n", name.c_str(), resetFuncName.c_str(), isUIntReset ? "uint" : "async");
  }
  std::string resetName = super->resetNode->type == NODE_REG_SRC ? RESET_NAME(super->resetNode).c_str() : super->resetNode->name.c_str();
  if (!emitActivation) {
    emitBodyLock(indent ++, "if(unlikely(%s)) {\n", resetName.c_str());
  }
  if (emitActivation) {
    emitBodyLock(indent ++, "if(unlikely(%s)) {\n", resetName.c_str());
    std::set<int> allNext;
    for (size_t i = 0; i < super->member.size(); i ++) {
      Node* node = super->member[i];
      if (node->type == NODE_REG_RESET) node = node->getResetSrc();
      for (Node* next : node->next) {
        if (next->super->cppId >= 0) allNext.insert(next->super->cppId);
      }
    }

    if (allNext.size() > 100) {
      if (buffered) {
        emitBodyLock(indent, "nextActive.activateAll();\n");
        if (mtUseActivationEventTraceCodegen()) {
          emitBodyLock(indent, "recordMtActivationEvent(traceSourceCppId, 0, UINT64_MAX, MT_ACTIVATION_EVENT_ACTIVATE_ALL);\n");
        }
      } else if (mtUseActivationEventTraceCodegen()) {
        emitBodyLock(indent, "activateAll(traceSourceCppId);\n");
      } else {
        emitBodyLock(indent, "activateAll();\n");
      }
    }
    else {
      std::map<uint64_t, ActiveType> bitMapInfo;
      activeSet2bitMap(allNext, bitMapInfo, -1);
      for (auto iter : bitMapInfo) {
        emitBodyLock(indent, "%s // %s\n", updateActiveStr(iter.first, ACTIVE_MASK(iter.second), buffered ? "nextActive" : "").c_str(), ACTIVE_COMMENT(iter.second).c_str());
        if (mtUseActivationEventTraceCodegen()) {
          emitBodyLock(indent, "recordMtActivationEvent(traceSourceCppId, (uint32_t)%lu, (uint64_t)0x%lx, MT_ACTIVATION_EVENT_CONDITIONAL);\n",
                       iter.first, ACTIVE_MASK(iter.second));
        }
      }
    }
    emitBodyLock(-- indent, "}\n");
  }
  // Chunked emission (GSIM_EMIT_RESET_CHUNK=<n>): the reset body's statement
  // stream is split into chain-called member helpers at top-level depth (the
  // IF/ELSE nesting is never cut). The parent keeps the reset guard and
  // activation logic; each chunk ends with a call to the next, so a chunk
  // executes under exactly the same conditions as the unsplit body. The C++
  // frontend is superlinear on million-statement functions (measured 1366s
  // for one 5MB subReset body, 2.8s stubbed); chunking restores linear cost.
  const long chunkSize = mtResetChunkSize();
  const char* chunkCallArgs = traceSourceParam ? "(nextActive, traceSourceCppId)" : (buffered ? "(nextActive)" : "()");
  const std::string chunkParamList = traceSourceParam
    ? (std::string("(") + (buffered ? "ActivationDelta &nextActive, " : "") + "int32_t traceSourceCppId)")
    : (buffered ? "(ActivationDelta &nextActive)" : "()");
  // The statement stream typically nests entirely inside `if (reset) { ... }`
  // (depth 1 throughout), so a depth-0-only split would never fire. Instead we
  // track the open IF stack; on a split we close the open braces, chain into
  // the next chunk function, and re-open the identical IF conditions inside
  // it. Reset conditions are pure member loads, so re-evaluating them is
  // side-effect-free and yields the same guard context for every statement.
  std::vector<std::string> openIfs;
  long emittedInChunk = 0;
  int chunkIdx = 0;
  for (InstInfo inst : super->insts) {
    if (chunkSize > 0 && emittedInChunk >= chunkSize &&
        inst.infoType != SUPER_INFO_IF && inst.infoType != SUPER_INFO_ELSE &&
        inst.infoType != SUPER_INFO_DEDENT) {
      // close the open if-chain, chain into the next chunk, reopen it there
      for (size_t d = 0; d < openIfs.size(); d ++) emitBodyLock(-- indent, "}\n");
      emitBodyLock(indent, "%s_c%d%s;\n", resetFuncName.c_str(), chunkIdx + 1, chunkCallArgs);
      if (chunkIdx == 0) {
        if (!emitActivation) emitBodyLock(-- indent, "}\n");  // reset guard
        emitBodyLock(-- indent, "}\n");                        // parent function
      } else {
        emitBodyLock(indent, "}\n");
      }
      chunkIdx ++;
      emitFuncDecl(indent, "void S%s::%s_c%d%s {\n", name.c_str(), resetFuncName.c_str(), chunkIdx, chunkParamList.c_str());
      mtResetChunkDecls().push_back(format("  void %s_c%d%s;\n", resetFuncName.c_str(), chunkIdx, chunkParamList.c_str()));
      for (const std::string& ifInst : openIfs) emitBodyLock(indent ++, "%s\n", ifInst.c_str());
      emittedInChunk = 0;
    }
    switch (inst.infoType) {
      case SUPER_INFO_IF:
        emitBodyLock(indent ++, "%s\n", inst.inst.c_str());
        openIfs.push_back(inst.inst);
        emittedInChunk ++;
        break;
      case SUPER_INFO_DEDENT:
        emitBodyLock(--indent, "%s\n", inst.inst.c_str());
        if (!openIfs.empty()) openIfs.pop_back();
        break;
      case SUPER_INFO_ELSE:
      case SUPER_INFO_STR:
        emitBodyLock(indent, "%s\n", inst.inst.c_str());
        emittedInChunk ++;
        break;
      default:
        break;
    }
  }
  if (chunkIdx > 0) {
    for (size_t d = 0; d < openIfs.size(); d ++) emitBodyLock(-- indent, "}\n");
    emitBodyLock(indent, "}\n");  // close the final chunk
  } else {
    if (!emitActivation) emitBodyLock(-- indent, "}\n");
    emitBodyLock(-- indent, "}\n");
  }
}

void graph::genResetActivation(SuperNode* super, bool isUIntReset, int indent, int resetId) {
  if (mtUseActivationEventTraceCodegen()) emitBodyLock(indent, "subReset%d(-1);\n", resetId);
  else emitBodyLock(indent, "subReset%d();\n", resetId);
}

void graph::genResetActivationDense(SuperNode* super, bool isUIntReset, int indent, int resetId) {
  (void)super;
  (void)isUIntReset;
  emitBodyLock(indent, "subResetDense%d();\n", resetId);
}

void graph::genResetAll() {
  std::vector<SuperNode*> resetSuper;
  for (SuperNode* super : allReset) {
    if (super->resetNode->status == CONSTANT_NODE) {
      Assert(mpz_sgn(super->resetNode->computeInfo->consVal) == 0, "reset %s is always true", super->resetNode->name.c_str());
      continue;
    }
    super2ResetId.emplace(super->resetNode, std::make_pair(-1, -1));
    int resetId = resetFuncNum ++;
    bool isUIntReset = super->superType == SUPER_UINT_RESET;
    if (isUIntReset) super2ResetId[super->resetNode].first = resetId;
    else super2ResetId[super->resetNode].second = resetId;
    genResetDef(super, isUIntReset, false, resetId, 0);
    if (globalConfig.MtHelperMode == "buffered-seq" ||
        globalConfig.MtHelperMode == "mt" ||
        globalConfig.MtHelperMode == "mt-level-dispatch") {
      genResetDef(super, isUIntReset, true, resetId, 0);
    }
    resetSuper.push_back(super);
  }

  emitFuncDecl(0, "void S%s::resetAll(){\n", name.c_str());
  for (size_t i = 0; i < resetSuper.size(); i ++) {
    if (resetSuper[i]->superType == SUPER_ASYNC_RESET) continue;
    genResetActivation(resetSuper[i], true, 1, i);
  }
  emitBodyLock(0, "}\n");
}

void graph::genResetAllDense() {
  std::vector<std::tuple<SuperNode*, bool, int>> resetSuper;
  super2DenseResetId.clear();
  int denseResetFuncNum = 0;
  for (SuperNode* super : allReset) {
    if (super->resetNode->status == CONSTANT_NODE) continue;
    bool isUIntReset = super->superType == SUPER_UINT_RESET;
    int resetId = denseResetFuncNum ++;
    if (super2DenseResetId.find(super->resetNode) == super2DenseResetId.end()) {
      super2DenseResetId[super->resetNode] = std::make_pair(-1, -1);
    }
    if (isUIntReset) super2DenseResetId[super->resetNode].first = resetId;
    else super2DenseResetId[super->resetNode].second = resetId;
    genResetDef(super, isUIntReset, false, resetId, 0, "Dense", false);
    resetSuper.push_back(std::make_tuple(super, isUIntReset, resetId));
  }

  emitFuncDecl(0, "void S%s::resetAllDense(){\n", name.c_str());
  emitBodyLock(1, "memset(activeFlags, 0, sizeof(activeFlags));\n");
  for (const auto& entry : resetSuper) {
    SuperNode* super;
    bool isUIntReset;
    int resetId;
    std::tie(super, isUIntReset, resetId) = entry;
    if (super->superType == SUPER_ASYNC_RESET) continue;
    genResetActivationDense(super, isUIntReset, 1, resetId);
  }
  emitBodyLock(0, "}\n");
}

void graph::genDenseExecutor(const MtDenseSchedule& denseSchedule, FILE* header) {
  Assert(denseSchedule.valid, "cannot emit dense executor for invalid schedule: %s", denseSchedule.fallbackReason.c_str());
  Assert(!denseSchedule.mtasks.empty(), "dense executor requires MTask partitioning");
  bool savedActivationEventTraceSuppressed = mtActivationEventTraceSuppressed;
  mtActivationEventTraceSuppressed = true;
  int nMTasks = static_cast<int>(denseSchedule.mtasks.size());
  int threadCount = 8;
  const char* threadsEnv = std::getenv("GSIM_THREADS");
  if (threadsEnv != nullptr && threadsEnv[0] != '\0') threadCount = std::atoi(threadsEnv);
  if (threadCount < 1) threadCount = 1;
  bool xthreadDepsOnly = mtUseDenseXThreadDepsOnly();
  bool transitiveReduceEdges = mtUseDenseTransitiveReduceEdges();
  bool staticEmptyElide = mtUseDenseStaticEmptyElide();
  bool ownerBankCountersDiag = mtUseDenseOwnerBankCountersDiag();
  bool ownerReadyFlags = mtUseDenseOwnerReadyFlags();
  bool denseBreakdownProfileCodegen = mtUseDenseBreakdownProfileCodegen();
  bool denseBreakdownWindowCodegen = denseBreakdownProfileCodegen && mtUseDenseBreakdownWindowCodegen();
  int denseBreakdownWindowWorker0MTaskCount = 0;
  if (denseBreakdownWindowCodegen) {
    for (int mtaskId = 0; mtaskId < nMTasks; mtaskId ++) {
      if (denseSchedule.mtaskThreadAssign[(size_t)mtaskId] == 0) denseBreakdownWindowWorker0MTaskCount ++;
    }
    Assert(denseBreakdownWindowWorker0MTaskCount <= 2267,
           "GSIM_MT_DENSE_BREAKDOWN window supports at most 2267 worker0 MTasks (got %d)",
           denseBreakdownWindowWorker0MTaskCount);
    Assert(threadCount >= 2,
           "GSIM_MT_DENSE_BREAKDOWN window requires at least 2 workers (got %d)", threadCount);
  }
  if (denseBreakdownProfileCodegen) {
    Assert(ownerReadyFlags,
           "GSIM_MT_DENSE_BREAKDOWN_PROFILE requires GSIM_MT_DENSE_OWNER_READY_FLAGS=1");
    Assert(threadCount <= 16,
           "GSIM_MT_DENSE_BREAKDOWN_PROFILE supports at most 16 workers (got %d)", threadCount);
  }

  if (ownerReadyFlags) {
    Assert(xthreadDepsOnly,
           "GSIM_MT_DENSE_OWNER_READY_FLAGS requires GSIM_MT_DENSE_XTHREAD_DEPS_ONLY=1");
    Assert(transitiveReduceEdges,
           "GSIM_MT_DENSE_OWNER_READY_FLAGS experiment requires GSIM_MT_DENSE_TRANSITIVE_REDUCE_EDGES=1");
  }
  const std::chrono::steady_clock::time_point densePrologueBegin = std::chrono::steady_clock::now();
  std::vector<std::vector<int>> denseRuntimeSuccs;
  int transitiveElidedEdges = 0;
  MtDenseOwnerReadyLayout ownerReadyLayout;
  {
    EmitPhaseTimer proSuccTimer("Final.densePrologue.runtimeSuccs");
    denseRuntimeSuccs = mtBuildDenseRuntimeSuccs(
        denseSchedule.mtasks, denseSchedule.mtaskThreadAssign, xthreadDepsOnly);
    transitiveElidedEdges = transitiveReduceEdges
        ? mtReduceDenseRuntimeSuccsTransitive(denseRuntimeSuccs, denseSchedule.mtaskThreadAssign) : 0;
    if (ownerReadyFlags) {
      ownerReadyLayout = mtBuildDenseOwnerReadyLayout(
          denseRuntimeSuccs, denseSchedule.mtaskThreadAssign, threadCount);
    }
  }
  const int denseLookaheadWindow = mtDenseLookaheadWindow();
  const bool denseLookahead = denseLookaheadWindow > 0;
  const bool denseDuty = mtDenseDutyCodegen();
  Assert(!denseLookahead || ownerReadyFlags,
         "GSIM_MT_DENSE_LOOKAHEAD requires GSIM_MT_DENSE_OWNER_READY_FLAGS=1");
  Assert(!denseDuty || ownerReadyFlags,
         "GSIM_MT_DENSE_DUTY requires GSIM_MT_DENSE_OWNER_READY_FLAGS=1");
  Assert(!denseLookahead || (!denseBreakdownProfileCodegen && !denseBreakdownWindowCodegen),
         "GSIM_MT_DENSE_LOOKAHEAD is incompatible with dense breakdown codegen");
  struct MtActivityCommitField { std::string name; uint64_t mask = 0; bool conservative = false; int shadowSlot = -1; int fanoutBegin = 0; int fanoutEnd = 0; };
  struct MtActivityInputField { std::string name; uint64_t mask = 0; bool conservative = false; int shadowSlot = -1; int fanoutBegin = 0; int fanoutEnd = 0; };
  std::vector<MtActivityInputField> activityInputFields;
  std::vector<std::vector<MtActivityCommitField>> activityCommitFields;
  std::vector<std::vector<int>> activitySuccFlags;
  std::vector<std::vector<int>> activitySuccLite;
  std::vector<std::vector<int>> activityNextEdges;
  std::vector<int> activityFanoutList;
  std::vector<char> activityIsCommitMTask;
  std::vector<char> activityAlwaysActive;
  std::vector<char> activityResetHandler;
  std::vector<int> activityRegionOf;
  std::vector<std::string> activityRegionNames;
  int activityShadowCount = 0;
  int activitySuccTotal = 0;
  int activityNextTotal = 0;
  int activityConservativeFields = 0;
  int activityScalarFields = 0;
  int activityAlwaysActiveCount = 0;
  // The PEG dump (GSIM_MT_DENSE_PEG_DUMP) needs the field read/write sets
  // computed in this block. The activity-only outputs remain unused locals.
  const bool pegDump = std::getenv("GSIM_MT_DENSE_PEG_DUMP") != nullptr;
  if (pegDump) {
    const int nM = nMTasks;
    activityAlwaysActive.assign((size_t)nM, 0);
    activityResetHandler.assign((size_t)nM, 0);
    // Region-level activity (r): group MTasks into subsystem regions; a region runs fully
    // when any member is activated. Fine-grained gating breaks intra-subsystem queue invariants
    // (yank/missQueue/ldu asserts from one-cycle-late comb signals); region granularity preserves
    // them because all producers+consumers in the region evaluate together. Region assignment by
    // top module path of member nodes.
    activityRegionOf.assign((size_t)nM, 0);
    std::map<std::string, int> regionIds;
    auto regionOfSuper = [&](SuperNode* super) -> std::string {
      for (Node* member : super->member) {
        const std::string& nm = member->name;
        if (nm.rfind("cpu__DOT__l_soc__DOT__core_with_l2__DOT__core__DOT__frontend", 0) == 0) return "frontend";
        if (nm.rfind("cpu__DOT__l_soc__DOT__core_with_l2__DOT__core__DOT__memBlock", 0) == 0) return "memblock";
        if (nm.rfind("cpu__DOT__l_soc__DOT__core_with_l2__DOT__l2top", 0) == 0) return "l2";
        if (nm.rfind("cpu__DOT__l_soc__DOT__chi_openllc", 0) == 0) return "linkmon";
        if (nm.rfind("cpu__DOT__l_simMMIO", 0) == 0) return "mmio";
      }
      return "other";
    };
    std::vector<std::set<std::string>> readFields((size_t)nM);
    std::vector<std::set<std::string>> dstReadFields((size_t)nM);
    std::vector<std::set<std::string>> commitFields((size_t)nM);
    std::vector<std::set<std::string>> nextWriteFields((size_t)nM);
    std::map<std::string, Node*> commitTargetNode;
    for (int m = 0; m < nM; m++) {
      const MtDenseMTask& mt = denseSchedule.mtasks[(size_t)m];
      for (int sccId : mt.sccIds) {
        if (sccId < 0 || sccId >= static_cast<int>(denseSchedule.sccs.size())) continue;
        const MtDenseScc& scc = denseSchedule.sccs[(size_t)sccId];
        for (int cppId : scc.cppIds) {
          auto superIter = cppId2Super.find(cppId);
          if (superIter == cppId2Super.end() || !superIter->second) continue;
          SuperNode* super = superIter->second;
          int dummyCost = 0;
          MtBoundaryInfo b = collectMtBoundaryInfo(super, dummyCost);
          if ((b.hasStateUpdate && b.hasAmbiguousStateTarget) || b.hasActivateAllPath) activityAlwaysActive[(size_t)m] = 1;
          if (super->superType == SUPER_ASYNC_RESET || b.hasAsyncReset) activityResetHandler[(size_t)m] = 1;
          if (activityRegionOf[(size_t)m] == 0) {
            std::string rn = regionOfSuper(super);
            auto rid = regionIds.find(rn);
            if (rid == regionIds.end()) rid = regionIds.emplace(rn, (int)regionIds.size()).first;
            activityRegionOf[(size_t)m] = rid->second + 1;
          }
          for (const std::string& r : b.rhsReadStateTargetNames) readFields[(size_t)m].insert(r);
          for (Node* member : super->member) {
            for (ExpTree* tree : member->assignTree) {
              MtActivityReads tr;
              mtActivityCollectFromTree(tree->getRoot(), tr);
              readFields[(size_t)m].insert(tr.src.begin(), tr.src.end());
              dstReadFields[(size_t)m].insert(tr.dst.begin(), tr.dst.end());
            }
            if (member->type == NODE_WRITER || member->type == NODE_READWRITER) {
              if (member->parent) commitFields[(size_t)m].insert(member->parent->name);
            }
          }
          if (b.stateSourceCommitCount > 0 || b.stateResetUpdateCount > 0) {
            for (const std::string& f : b.stateTargetNames) commitFields[(size_t)m].insert(f);
            for (Node* member : super->member) {
              if (!nodeHasStateUpdate(member)) continue;
              std::string tn;
              if (!stateTargetNameForNode(member, tn)) continue;
              Node* target = member->type == NODE_REG_SRC ? member
                           : (member->type == NODE_REG_DST ? member->getSrc() : member->getResetSrc());
              if (target) commitTargetNode[tn] = target;
            }
          }
          if (b.stateNextUpdateCount > 0) {
            for (const std::string& f : b.stateTargetNames) nextWriteFields[(size_t)m].insert(f);
          }
        }
      }
    }
    std::vector<char> activityElided((size_t)nM, 0);
    for (int m = 0; m < nM; m++) if (denseSchedule.mtaskThreadAssign[(size_t)m] < 0) activityElided[(size_t)m] = 1;
    // runtime successors redirected through elided MTasks (they never run, so their successors
    // would never receive a flag from them)
    activitySuccFlags.assign((size_t)nM, {});
    for (int m = 0; m < nM; m++) {
      if (activityElided[(size_t)m]) continue;
      std::vector<int> stack;
      std::vector<char> seen((size_t)nM, 0);
      for (int s : denseSchedule.mtasks[(size_t)m].succMTasks) if (s >= 0 && s < nM) stack.push_back(s);
      while (!stack.empty()) {
        int s = stack.back(); stack.pop_back();
        if (seen[(size_t)s]) continue; seen[(size_t)s] = 1;
        if (activityElided[(size_t)s]) {
          for (int s2 : denseSchedule.mtasks[(size_t)s].succMTasks) if (s2 >= 0 && s2 < nM && !seen[(size_t)s2]) stack.push_back(s2);
        } else {
          activitySuccFlags[(size_t)m].push_back(s);
        }
      }
      activitySuccTotal += (int)activitySuccFlags[(size_t)m].size();
    }
    // next-state edges: nextWriter[X] -> nextReader[X]. The dense DAG excludes $NEXT reads
    // (next-state objects are cross-cycle buffers: the producer at cycle N is read by the
    // commit at cycle N+1), so no DAG edge exists and the succ-flag propagation would never
    // reach commit MTasks (activity5: OH commit MTask 1018 has 0 DAG preds and starved).
    std::map<std::string, std::vector<int>> nextWriters;
    std::map<std::string, std::vector<int>> nextReaders;
    for (int m = 0; m < nM; m++) {
      if (activityElided[(size_t)m]) continue;
      for (const std::string& f : nextWriteFields[(size_t)m]) nextWriters[f].push_back(m);
      for (const std::string& f : dstReadFields[(size_t)m]) nextReaders[f].push_back(m);
      for (const std::string& f : commitFields[(size_t)m]) nextReaders[f].push_back(m);
    }
    // GSIM_MT_DENSE_PEG_DUMP=<prefix>: dump the precedence event graph for recurrence
    // (max-cycle-ratio) analysis: within-cycle dependency edges (dist=0), cross-cycle
    // register-feedback edges writer->reader (dist=1), mcost per MTask. Format matches
    // the mcr tooling (int32 u,v,dist triples; double costs).
    if (const char* pegPath = std::getenv("GSIM_MT_DENSE_PEG_DUMP")) {
      std::string edgePath = std::string(pegPath) + "-edges.bin";
      std::string costPath = std::string(pegPath) + "-cost.bin";
      FILE* pe = std::fopen(edgePath.c_str(), "wb");
      FILE* pc = std::fopen(costPath.c_str(), "wb");
      Assert(pe && pc, "cannot open PEG dump %s", pegPath);
      int64_t depEdges = 0, wrapEdges = 0;
      for (int m = 0; m < nM; m++) {
        if (activityElided[(size_t)m]) continue;
        for (int succ : denseSchedule.mtasks[(size_t)m].succMTasks) {
          if (succ < 0 || succ >= nM || activityElided[(size_t)succ]) continue;
          int32_t t[3] = {m, succ, 0};
          std::fwrite(t, 4, 3, pe);
          depEdges ++;
        }
      }
      for (const auto& kv : nextWriters) {
        auto it = nextReaders.find(kv.first);
        if (it == nextReaders.end()) continue;
        for (int w : kv.second) {
          for (int r : it->second) {
            if (r == w) continue;
            int32_t t[3] = {w, r, 1};
            std::fwrite(t, 4, 3, pe);
            wrapEdges ++;
          }
        }
      }
      for (int m = 0; m < nM; m++) {
        const MtDenseMTask& mt = denseSchedule.mtasks[(size_t)m];
        double c = activityElided[(size_t)m] ? 0.0 : (double)(mt.schedCost > 0 ? mt.schedCost : mt.staticCost);
        std::fwrite(&c, 8, 1, pc);
      }
      std::fclose(pe);
      std::fclose(pc);
      fprintf(stderr, "[mt-dense-peg] dumped %d nodes dep=%lld wrap=%lld to %s-{edges,cost}.bin\n",
              nM, (long long)depEdges, (long long)wrapEdges, pegPath);
    }
    // Report-only per-MTask provenance for offline witness attribution: the
    // dist=1 PEG edge set is exactly nextWriteFields[u] x (dstReadFields |
    // commitFields)[v] per shared field, so exporting these sets per MTask
    // lets the mcr5 witness be joined back to fields and module paths without
    // another generation. Default-off: only under GSIM_MT_DENSE_PEG_DUMP.
    if (const char* provPegPath = std::getenv("GSIM_MT_DENSE_PEG_DUMP")) {
      std::string provPath = std::string(provPegPath) + "-provenance.json";
      FILE* pj = std::fopen(provPath.c_str(), "w");
      Assert(pj != nullptr, "cannot open PEG provenance dump %s", provPath.c_str());
      std::vector<std::string> regionNameById(regionIds.size() + 1, "other");
      for (const auto& rn : regionIds) regionNameById[(size_t)rn.second + 1] = rn.first;
      auto dumpProvStrSet = [&](FILE* out, const std::set<std::string>& s) {
        fprintf(out, "[");
        bool first = true;
        for (const std::string& x : s) {
          fprintf(out, "%s\"%s\"", first ? "" : ",", jsonEscape(x).c_str());
          first = false;
        }
        fprintf(out, "]");
      };
      auto dumpProvIntVec = [&](FILE* out, const std::vector<int>& v) {
        fprintf(out, "[");
        bool first = true;
        for (int x : v) { fprintf(out, "%s%d", first ? "" : ",", x); first = false; }
        fprintf(out, "]");
      };
      const size_t kMaxNodeNames = 256;
      fprintf(pj, "{\n\"format\": \"gsim.mt-dense-peg-provenance.v1\",\n\"mtask_count\": %d,\n\"mtasks\": [\n", nM);
      for (int m = 0; m < nM; m++) {
        const MtDenseMTask& mt = denseSchedule.mtasks[(size_t)m];
        int worker = m < static_cast<int>(denseSchedule.mtaskThreadAssign.size())
                       ? denseSchedule.mtaskThreadAssign[(size_t)m] : -1;
        std::vector<int> sccIdsOut;
        std::set<int> cppIdSet;
        std::set<std::string> nodeNames;
        bool namesTruncated = false;
        std::set<std::string> modulePaths;
        for (int sccId : mt.sccIds) {
          if (sccId < 0 || sccId >= static_cast<int>(denseSchedule.sccs.size())) continue;
          sccIdsOut.push_back(sccId);
          for (int cppId : denseSchedule.sccs[(size_t)sccId].cppIds) {
            cppIdSet.insert(cppId);
            auto superIter = cppId2Super.find(cppId);
            if (superIter == cppId2Super.end() || !superIter->second) continue;
            for (Node* member : superIter->second->member) {
              const std::string& nm = member->name;
              if (nm.empty()) continue;
              if (nodeNames.size() < kMaxNodeNames) nodeNames.insert(nm);
              else namesTruncated = true;
              size_t cut = 0;
              int segs = 0;
              while (segs < 7) {
                size_t pos = nm.find("__DOT__", cut);
                if (pos == std::string::npos) { cut = nm.size(); break; }
                cut = pos + 7; segs++;
              }
              modulePaths.insert(nm.substr(0, cut < nm.size() ? cut : nm.size()));
            }
          }
        }
        std::vector<int> cppIdsOut(cppIdSet.begin(), cppIdSet.end());
        fprintf(pj, "  {\"id\": %d, \"worker\": %d, \"sched_cost\": %d, \"static_cost\": %d, \"task_count\": %d,\n",
                m, worker, mt.schedCost, mt.staticCost, mt.taskCount);
        fprintf(pj, "   \"region\": \"%s\", \"always_active\": %s, \"reset_handler\": %s,\n",
                jsonEscape(regionNameById[(size_t)activityRegionOf[(size_t)m]]).c_str(),
                activityAlwaysActive[(size_t)m] ? "true" : "false",
                activityResetHandler[(size_t)m] ? "true" : "false");
        fprintf(pj, "   \"scc_ids\": ");
        dumpProvIntVec(pj, sccIdsOut);
        fprintf(pj, ",\n   \"cpp_ids\": ");
        dumpProvIntVec(pj, cppIdsOut);
        fprintf(pj, ",\n   \"node_names_truncated\": %s,\n   \"node_names\": ", namesTruncated ? "true" : "false");
        dumpProvStrSet(pj, nodeNames);
        fprintf(pj, ",\n   \"module_paths\": ");
        dumpProvStrSet(pj, modulePaths);
        fprintf(pj, ",\n   \"read_fields\": ");
        dumpProvStrSet(pj, readFields[(size_t)m]);
        fprintf(pj, ",\n   \"dst_read_fields\": ");
        dumpProvStrSet(pj, dstReadFields[(size_t)m]);
        fprintf(pj, ",\n   \"commit_fields\": ");
        dumpProvStrSet(pj, commitFields[(size_t)m]);
        fprintf(pj, ",\n   \"next_write_fields\": ");
        dumpProvStrSet(pj, nextWriteFields[(size_t)m]);
        fprintf(pj, "}%s\n", m + 1 == nM ? "" : ",");
      }
      fprintf(pj, "]}\n");
      std::fclose(pj);
      fprintf(stderr, "[mt-dense-peg] wrote per-MTask provenance to %s\n", provPath.c_str());
    }
    activityNextEdges.assign((size_t)nM, {});
    for (int m = 0; m < nM; m++) {
      if (activityElided[(size_t)m]) continue;
      if (nextWriteFields[(size_t)m].empty()) continue;
      std::set<int> targets;
      for (const std::string& f : nextWriteFields[(size_t)m]) {
        auto it = nextReaders.find(f);
        if (it == nextReaders.end()) continue;
        for (int r : it->second) if (r != m) targets.insert(r);
      }
      activityNextEdges[(size_t)m].assign(targets.begin(), targets.end());
      activityNextTotal += (int)activityNextEdges[(size_t)m].size();
    }
    std::map<std::string, std::vector<int>> fieldReaders;
    for (int m = 0; m < nM; m++) {
      if (activityElided[(size_t)m]) continue;
      for (const std::string& r : readFields[(size_t)m]) fieldReaders[r].push_back(m);
    }
    // harness-driven input watcher: io_* input fields change exogenously (no commit fires), so
    // their readers must be activated by a shadow-compare pass in stepDense before each cycle.
    activityInputFields.clear();
    {
      std::set<std::string> inputNames;
      for (int m = 0; m < nM; m++) {
        const MtDenseMTask& mt = denseSchedule.mtasks[(size_t)m];
        for (int sccId : mt.sccIds) {
          if (sccId < 0 || sccId >= static_cast<int>(denseSchedule.sccs.size())) continue;
          for (int cppId : denseSchedule.sccs[(size_t)sccId].cppIds) {
            auto superIter = cppId2Super.find(cppId);
            if (superIter == cppId2Super.end() || !superIter->second) continue;
            for (Node* member : superIter->second->member) {
              if (member->type == NODE_INP && !member->name.empty()) inputNames.insert(member->name);
            }
          }
        }
      }
      for (const std::string& f : inputNames) {
        MtActivityInputField inf; inf.name = f;
        Node* node = nullptr;
        for (int m = 0; m < nM && node == nullptr; m++) {
          const MtDenseMTask& mt = denseSchedule.mtasks[(size_t)m];
          for (int sccId : mt.sccIds) {
            if (sccId < 0 || sccId >= static_cast<int>(denseSchedule.sccs.size())) continue;
            for (int cppId : denseSchedule.sccs[(size_t)sccId].cppIds) {
              auto superIter = cppId2Super.find(cppId);
              if (superIter == cppId2Super.end() || !superIter->second) continue;
              for (Node* member : superIter->second->member) if (member->type == NODE_INP && member->name == f) { node = member; break; }
              if (node) break;
            }
            if (node) break;
          }
        }
        bool scalar = node != nullptr && !node->isArray() && node->width <= 64;
        if (scalar) {
          inf.mask = node->width >= 64 ? ~0ULL : ((1ULL << node->width) - 1);
          inf.shadowSlot = activityShadowCount++;
        } else {
          inf.conservative = true;
        }
        inf.fanoutBegin = (int)activityFanoutList.size();
        auto readerIter = fieldReaders.find(f);
        if (readerIter != fieldReaders.end()) for (int r : readerIter->second) activityFanoutList.push_back(r);
        inf.fanoutEnd = (int)activityFanoutList.size();
        activityInputFields.push_back(inf);
      }
    }
    activityCommitFields.assign((size_t)nM, {});
    for (int m = 0; m < nM; m++) {
      if (activityElided[(size_t)m]) continue;
      for (const std::string& f : commitFields[(size_t)m]) {
        MtActivityCommitField cf; cf.name = f;
        Node* target = nullptr;
        auto nodeIter = commitTargetNode.find(f);
        if (nodeIter != commitTargetNode.end()) target = nodeIter->second;
        bool scalar = target != nullptr && target->type != NODE_MEMORY && !target->isArray() && target->width <= 64;
        if (scalar) {
          cf.mask = target->width >= 64 ? ~0ULL : ((1ULL << target->width) - 1);
          cf.shadowSlot = activityShadowCount++;
          activityScalarFields++;
        } else {
          cf.conservative = true;
          activityConservativeFields++;
        }
        cf.fanoutBegin = (int)activityFanoutList.size();
        auto readerIter = fieldReaders.find(f);
        if (readerIter != fieldReaders.end()) for (int r : readerIter->second) activityFanoutList.push_back(r);
        cf.fanoutEnd = (int)activityFanoutList.size();
        activityCommitFields[(size_t)m].push_back(cf);
      }
    }
    // succLite for commit MTasks: successors NOT covered by the commit fanout (i.e. consumers of
    // non-state wire outputs). Commit MTasks fire only compare+fanout (tight) plus succLite;
    // without this split the conservative succ store keeps the whole DAG active forever (no decay).
    activitySuccLite.assign((size_t)nM, {});
    for (int m = 0; m < nM; m++) {
      if (activityElided[(size_t)m] || activityCommitFields[(size_t)m].empty()) continue;
      std::set<int> covered;
      for (const MtActivityCommitField& cf : activityCommitFields[(size_t)m])
        for (int j = cf.fanoutBegin; j < cf.fanoutEnd; j++) covered.insert(activityFanoutList[(size_t)j]);
      for (int s : activitySuccFlags[(size_t)m])
        if (!covered.count(s)) activitySuccLite[(size_t)m].push_back(s);
    }
    int activityCommitMTasks = 0;
    activityIsCommitMTask.assign((size_t)nMTasks, 0);
    for (int m = 0; m < nMTasks; m++) {
      activityIsCommitMTask[(size_t)m] = !activityCommitFields[(size_t)m].empty() ? 1 : 0;
      activityCommitMTasks += activityIsCommitMTask[(size_t)m];
      activityAlwaysActiveCount += activityAlwaysActive[(size_t)m] ? 1 : 0;
    }
    int activityResetHandlerCount = 0;
    for (int m = 0; m < nMTasks; m++) activityResetHandlerCount += activityResetHandler[(size_t)m] ? 1 : 0;
    activityRegionNames.assign(regionIds.size() + 1, "other");
    for (const auto& kv : regionIds) activityRegionNames[(size_t)kv.second + 1] = kv.first;
    // The CHI link layer does cycle-precise credit accounting that is hostile to any staleness;
    // keep it always-active (small share of the model). Same for l2/mmio: their queue invariants
    // span the link<->DUT boundary and fail under any gating (yank/missQueue/L-credit asserts).
    // memblock (LSU/dcache/missQueue) is the same class of cycle-precise queue accounting.
    // frontend/other stay fine-grained sparse.
    for (int m = 0; m < nMTasks; m++) {
      const std::string& rn = activityRegionNames[(size_t)activityRegionOf[(size_t)m]];
      if (rn == "linkmon" || rn == "l2" || rn == "mmio" || rn == "memblock") activityAlwaysActive[(size_t)m] = 1;
    }
    std::map<std::string, int> regionMTaskCounts;
    for (int m = 0; m < nMTasks; m++) regionMTaskCounts[activityRegionNames[(size_t)activityRegionOf[(size_t)m]]]++;
    fprintf(stderr, "[mt-dense-activity] mtasks=%d commit_mtasks=%d always_active=%d reset_handlers=%d input_fields=%zu scalar_fields=%d conservative_fields=%d fanout=%zu succ_flags=%d next_edges=%d shadow=%d regions=",
            nMTasks, activityCommitMTasks, activityAlwaysActiveCount, activityResetHandlerCount, activityInputFields.size(),
            activityScalarFields, activityConservativeFields, activityFanoutList.size(), activitySuccTotal, activityNextTotal, activityShadowCount);
    for (const auto& kv : regionMTaskCounts) fprintf(stderr, "%s:%d ", kv.first.c_str(), kv.second);
    fprintf(stderr, "\n");
  }
  std::vector<int> denseDispatchWorkerCounts;
  std::vector<uint32_t> denseDispatchWaitBegin, denseDispatchWaitEnd;
  std::vector<uint32_t> denseDispatchStoreBegin, denseDispatchStoreEnd;
  uint32_t denseDispatchWaitTotal = 0;
  const std::chrono::steady_clock::time_point denseProFieldSetsBegin = std::chrono::steady_clock::now();
  if (emitPhaseTimingEnabled()) {
    fprintf(stderr, "[emit-phase] Final.densePrologue.fieldSets = %ld ms\n",
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - denseProFieldSetsBegin).count());
  }
  MtDenseBreakdownWindowWaitLayout denseBreakdownWindowWaitLayout;
  if (denseBreakdownWindowCodegen) {
    denseBreakdownWindowWaitLayout = mtBuildDenseBreakdownWindowWaitLayout(
        ownerReadyLayout, denseSchedule.mtaskThreadAssign, threadCount);
    Assert(denseBreakdownWindowWaitLayout.totalWaitRecords <= 65536,
           "GSIM_MT_DENSE_BREAKDOWN window supports at most 65536 ready waits per cycle (got %d)",
           denseBreakdownWindowWaitLayout.totalWaitRecords);
  }
  MtDenseBreakdownWindowAllOwnerLayout denseBreakdownWindowAllOwnerLayout;
  if (denseBreakdownWindowCodegen) {
    denseBreakdownWindowAllOwnerLayout = mtBuildDenseBreakdownWindowAllOwnerLayout(
        denseSchedule.mtaskThreadAssign, threadCount);
    Assert(denseBreakdownWindowAllOwnerLayout.recordCount >= nMTasks
           && denseBreakdownWindowAllOwnerLayout.recordCount <= nMTasks + 7 * threadCount,
           "dense breakdown all-owner physical record count %d is invalid for %d MTasks",
           denseBreakdownWindowAllOwnerLayout.recordCount, nMTasks);
  }
  const std::chrono::steady_clock::time_point denseProDepsBegin = std::chrono::steady_clock::now();
  std::vector<uint32_t> denseRuntimeDepCounts((size_t)nMTasks, 0);
  int totalSuccs = 0;
  for (int mtaskId = 0; mtaskId < nMTasks; mtaskId++) {
    for (int succ : denseRuntimeSuccs[(size_t)mtaskId]) {
      if (succ < 0 || succ >= nMTasks) continue;
      denseRuntimeDepCounts[(size_t)succ] ++;
      totalSuccs ++;
    }
  }
  if (totalSuccs == 0) totalSuccs = 1;
  if (emitPhaseTimingEnabled()) {
    fprintf(stderr, "[emit-phase] Final.densePrologue.depCounts = %ld ms\n",
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - denseProDepsBegin).count());
  }
  std::vector<uint32_t> denseLookaheadLocalBegin, denseLookaheadLocalEnd,
      denseLookaheadLocalPrereqs;
  std::vector<uint8_t> denseLookaheadLocalPrereqKinds;
  std::vector<std::vector<int>> denseLookaheadDagOnlySuccs;
  int denseLookaheadDagOnlyKeptEdges = 0;
  int denseLookaheadCrossEdgeCount = 0;
  int denseLookaheadSameWorkerPredCount = 0;
  int denseLookaheadPublisherConstrainedEntryCount = 0;
  int denseLookaheadPublisherSiblingPositionCount = 0;
  const std::chrono::steady_clock::time_point denseProLookaheadBegin = std::chrono::steady_clock::now();
  if (denseLookahead) {
    // The B1 token layout reduces the complete MTask DAG without the synthetic
    // fixed-worker chains used by the strict Step-A token layout.
    denseLookaheadDagOnlySuccs = mtBuildDenseRuntimeSuccs(
        denseSchedule.mtasks, denseSchedule.mtaskThreadAssign, false);
    mtReduceDenseRuntimeSuccsTransitive(
        denseLookaheadDagOnlySuccs, denseSchedule.mtaskThreadAssign, false);
    denseLookaheadDagOnlyKeptEdges = mtDenseRuntimeEdgeCount(denseLookaheadDagOnlySuccs);
    std::vector<std::vector<int>> denseLookaheadCrossWorkerSuccs((size_t)nMTasks);
    for (int pred = 0; pred < nMTasks; ++pred) {
      for (int succ : denseLookaheadDagOnlySuccs[(size_t)pred]) {
        if (denseSchedule.mtaskThreadAssign[(size_t)pred] != denseSchedule.mtaskThreadAssign[(size_t)succ])
          denseLookaheadCrossWorkerSuccs[(size_t)pred].push_back(succ);
      }
    }
    ownerReadyLayout = mtBuildDenseOwnerReadyLayout(
        denseLookaheadCrossWorkerSuccs, denseSchedule.mtaskThreadAssign, threadCount);

    std::vector<int> positionByMTask((size_t)nMTasks, -1);
    for (int worker = 0; worker < threadCount; ++worker) {
      int position = 0;
      for (int mtaskId = 0; mtaskId < nMTasks; ++mtaskId) {
        if (denseSchedule.mtaskThreadAssign[(size_t)mtaskId] == worker)
          positionByMTask[(size_t)mtaskId] = position++;
      }
    }
    std::vector<std::set<uint32_t>> sameWorkerPredsByMTask((size_t)nMTasks);
    std::vector<std::set<uint32_t>> publisherSiblingsByMTask((size_t)nMTasks);
    std::vector<char> publisherConstrained((size_t)nMTasks, 0);
    for (int pred = 0; pred < nMTasks; ++pred) {
      for (int succ : denseLookaheadDagOnlySuccs[(size_t)pred]) {
        Assert(succ > pred && succ < nMTasks,
               "lookahead DAG-only reduction produced invalid edge %d -> %d", pred, succ);
        const int predOwner = denseSchedule.mtaskThreadAssign[(size_t)pred];
        const int succOwner = denseSchedule.mtaskThreadAssign[(size_t)succ];
        if (predOwner == succOwner) {
          Assert(positionByMTask[(size_t)pred] >= 0
                     && positionByMTask[(size_t)pred] < positionByMTask[(size_t)succ],
                 "lookahead same-worker predecessor %d is not before successor %d", pred, succ);
          sameWorkerPredsByMTask[(size_t)succ].insert(
              static_cast<uint32_t>(positionByMTask[(size_t)pred]));
        } else {
          ++denseLookaheadCrossEdgeCount;
        }
      }
    }
    for (int token = 0; token < ownerReadyLayout.tokenCount; ++token) {
      const MtDenseOwnerReadyTokenProvenance& provenance =
          ownerReadyLayout.tokenProvenanceByLogicalToken[(size_t)token];
      const std::vector<int>& sources =
          ownerReadyLayout.sourceMTasksByLogicalToken[(size_t)token];
      Assert(!sources.empty() && sources.back() == provenance.producerMTask,
             "lookahead token %d has invalid publisher membership", token);
      if (sources.size() > 1) publisherConstrained[(size_t)provenance.producerMTask] = 1;
      for (int source : sources) {
        if (source == provenance.producerMTask) continue;
        Assert(denseSchedule.mtaskThreadAssign[(size_t)source] == provenance.producerOwner
                   && positionByMTask[(size_t)source] < positionByMTask[(size_t)provenance.producerMTask],
               "lookahead publisher %d does not follow sibling %d", provenance.producerMTask, source);
        publisherSiblingsByMTask[(size_t)provenance.producerMTask].insert(
            static_cast<uint32_t>(positionByMTask[(size_t)source]));
      }
    }
    denseLookaheadLocalBegin.assign((size_t)nMTasks, 0);
    denseLookaheadLocalEnd.assign((size_t)nMTasks, 0);
    for (int mtaskId = 0; mtaskId < nMTasks; ++mtaskId) {
      const std::set<uint32_t>& direct = sameWorkerPredsByMTask[(size_t)mtaskId];
      const std::set<uint32_t>& siblings = publisherSiblingsByMTask[(size_t)mtaskId];
      std::set<uint32_t> local = direct;
      local.insert(siblings.begin(), siblings.end());
      denseLookaheadLocalBegin[(size_t)mtaskId] =
          static_cast<uint32_t>(denseLookaheadLocalPrereqs.size());
      for (uint32_t position : local) {
        Assert(position < static_cast<uint32_t>(positionByMTask[(size_t)mtaskId]),
               "lookahead local prerequisite position %u is not before MTask %d", position, mtaskId);
        denseLookaheadLocalPrereqs.push_back(position);
        denseLookaheadLocalPrereqKinds.push_back(
            static_cast<uint8_t>((direct.count(position) ? 1u : 0u)
                                 | (siblings.count(position) ? 2u : 0u)));
      }
      denseLookaheadLocalEnd[(size_t)mtaskId] =
          static_cast<uint32_t>(denseLookaheadLocalPrereqs.size());
      denseLookaheadSameWorkerPredCount += static_cast<int>(direct.size());
      denseLookaheadPublisherSiblingPositionCount += static_cast<int>(siblings.size());
      denseLookaheadPublisherConstrainedEntryCount += publisherConstrained[(size_t)mtaskId] ? 1 : 0;
    }
    denseDispatchWorkerCounts.assign((size_t)threadCount, 0);
    denseDispatchWaitBegin.assign((size_t)nMTasks, 0);
    denseDispatchWaitEnd.assign((size_t)nMTasks, 0);
    denseDispatchStoreBegin.assign((size_t)nMTasks, 0);
    denseDispatchStoreEnd.assign((size_t)nMTasks, 0);
    uint32_t storeOff = 0;
    for (int mtaskId = 0; mtaskId < nMTasks; ++mtaskId) {
      denseDispatchStoreBegin[(size_t)mtaskId] = storeOff;
      storeOff += static_cast<uint32_t>(ownerReadyLayout.storeSlotsByMTask[(size_t)mtaskId].size());
      denseDispatchStoreEnd[(size_t)mtaskId] = storeOff;
    }
    denseDispatchWaitTotal = 0;
    for (int worker = 0; worker < threadCount; ++worker) {
      for (int mtaskId = 0; mtaskId < nMTasks; ++mtaskId) {
        if (denseSchedule.mtaskThreadAssign[(size_t)mtaskId] != worker) continue;
        ++denseDispatchWorkerCounts[(size_t)worker];
        denseDispatchWaitBegin[(size_t)mtaskId] = denseDispatchWaitTotal;
        denseDispatchWaitTotal += static_cast<uint32_t>(ownerReadyLayout.waitSlotsByMTask[(size_t)mtaskId].size());
        denseDispatchWaitEnd[(size_t)mtaskId] = denseDispatchWaitTotal;
      }
    }
  }
  if (emitPhaseTimingEnabled()) {
    fprintf(stderr, "[emit-phase] Final.densePrologue.lookahead = %ld ms\n",
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - denseProLookaheadBegin).count());
  }
  if (ownerReadyFlags) {
    fprintf(stderr,
            "[mt-dense-owner-ready] mtasks=%d edges=%d tokens=%d slots=%d padding=%d banks=%d threads=%d\n",
            nMTasks, ownerReadyLayout.edgeCount, ownerReadyLayout.tokenCount,
            ownerReadyLayout.physicalSlotCount,
            ownerReadyLayout.physicalSlotCount - ownerReadyLayout.tokenCount,
            ownerReadyLayout.pairBankCount, threadCount);
  }
  if (denseLookahead) {
    fprintf(stderr,
            "[mt-dense-lookahead] window=%d dag_only_kept_edges=%d cross_edges=%d same_worker_preds=%d groups=%d wait_entries=%d store_entries=%d publisher_constrained_entries=%d sibling_positions=%d\n",
            denseLookaheadWindow, denseLookaheadDagOnlyKeptEdges, denseLookaheadCrossEdgeCount,
            denseLookaheadSameWorkerPredCount, ownerReadyLayout.tokenCount,
            ownerReadyLayout.tokenCount, ownerReadyLayout.tokenCount,
            denseLookaheadPublisherConstrainedEntryCount,
            denseLookaheadPublisherSiblingPositionCount);
  }

  auto emitDenseDepCounts = [&]() {
    fprintf(header, "static constexpr uint32_t kDenseMTaskDepCount[%d] = {", nMTasks);
    for (int i = 0; i < nMTasks; i++) { if (i > 0) fprintf(header, ","); fprintf(header, "%u", denseRuntimeDepCounts[(size_t)i]); }
    fprintf(header, "};\n");
  };
  auto emitDenseOwnerBankDiagnostics = [&]() {
    if (!ownerBankCountersDiag) return;
    fprintf(header, "static constexpr int kDenseMTaskPhysicalSlotCount = %d;\n", nMTasks);
    fprintf(header, "static constexpr int kDenseMTaskVertexSlot[%d] = {", nMTasks);
    for (int i = 0; i < nMTasks; i ++) { if (i > 0) fprintf(header, ","); fprintf(header, "%d", i); }
    fprintf(header, "};\n");
    fprintf(header, "static constexpr int kDenseMTaskVertexOwner[%d] = {", nMTasks);
    for (int i = 0; i < nMTasks; i ++) { if (i > 0) fprintf(header, ","); fprintf(header, "%d", denseSchedule.mtaskThreadAssign[(size_t)i]); }
    fprintf(header, "};\n");
  };
  auto emitDenseSuccOffsets = [&]() {
    fprintf(header, "static constexpr int kDenseMTaskSuccOffsets[%d] = {", nMTasks + 1);
    int off = 0;
    for (int i = 0; i < nMTasks; i++) {
      if (i > 0) fprintf(header, ",");
      fprintf(header, "%d", off);
      off += static_cast<int>(denseRuntimeSuccs[(size_t)i].size());
    }
    fprintf(header, ",%d};\n", off);
  };
  auto emitDenseSuccList = [&]() {
    fprintf(header, "static constexpr int kDenseMTaskSuccList[%d] = {", totalSuccs);
    bool firstSucc = true;
    for (const std::vector<int>& succs : denseRuntimeSuccs) {
      for (int succ : succs) {
        if (!firstSucc) fprintf(header, ",");
        fprintf(header, "%d", succ);
        firstSucc = false;
      }
    }
    if (firstSucc) fprintf(header, "0");
    fprintf(header, "};\n");
  };

  const std::chrono::steady_clock::time_point denseProHeaderTablesBegin = std::chrono::steady_clock::now();
  fprintf(header, "static constexpr bool kDenseXThreadDepsOnly = %s;\n", xthreadDepsOnly ? "true" : "false");
  fprintf(header, "static constexpr bool kDenseTransitiveReduceEdges = %s;\n", transitiveReduceEdges ? "true" : "false");
  fprintf(header, "static constexpr int kDenseTransitiveElidedEdgeCount = %d;\n", transitiveElidedEdges);
  if (!ownerReadyFlags) {
    emitDenseDepCounts();
    emitDenseOwnerBankDiagnostics();
    emitDenseSuccOffsets();
    emitDenseSuccList();
    fprintf(header, "struct MtDenseMTaskVertex { std::atomic<uint32_t> depsDone{0}; };\n");
    fprintf(header, "MtDenseMTaskVertex mtDenseMTaskVertices[%d];\n", nMTasks);
  } else {
    if (denseBreakdownProfileCodegen) {
      fprintf(header, "#if !defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) || !GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
      fprintf(header, "#error \"GSIM_MT_DENSE_BREAKDOWN_PROFILE requires GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\"\n");
      fprintf(header, "#endif\n");
    }

    fprintf(header, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    fprintf(header, "static constexpr int kDenseOwnerReadyTokenCount = %d;\n", ownerReadyLayout.tokenCount);
    fprintf(header, "static constexpr int kDenseOwnerReadyWorkerCount = %d;\n", threadCount);
    fprintf(header, "static constexpr int kDenseOwnerReadyPhysicalSlotCount = %d;\n", ownerReadyLayout.physicalSlotCount);
    fprintf(header, "static constexpr int kDenseOwnerReadyStoreOffsets[%d] = {", nMTasks + 1);
    {
      int off = 0;
      for (int mtaskId = 0; mtaskId < nMTasks; mtaskId ++) {
        if (mtaskId > 0) fprintf(header, ",");
        fprintf(header, "%d", off);
        off += static_cast<int>(ownerReadyLayout.storeSlotsByMTask[(size_t)mtaskId].size());
      }
      fprintf(header, ",%d};\n", off);
      Assert(off == ownerReadyLayout.tokenCount,
             "dense owner-ready signal table has %d entries for %d tokens",
             off, ownerReadyLayout.tokenCount);
    }
    fprintf(header, "static constexpr int kDenseOwnerReadyStoreList[%d] = {",
            std::max(1, ownerReadyLayout.tokenCount));
    {
      bool firstSlot = true;
      for (const std::vector<int>& slots : ownerReadyLayout.storeSlotsByMTask) {
        for (int slot : slots) {
          if (!firstSlot) fprintf(header, ",");
          fprintf(header, "%d", slot);
          firstSlot = false;
        }
      }
      if (firstSlot) fprintf(header, "0");
      fprintf(header, "};\n");
    }
    fprintf(header, "struct MtDenseOwnerReadyToken { std::atomic<uint8_t> ready{0}; };\n");
    if (denseDuty) {
      fprintf(header, "struct alignas(64) MtDenseDutyLane { uint64_t spinNs = 0, spanNs = 0, tailNs = 0, blockNs = 0, resetNs = 0, joinNs = 0, stepWallNs = 0, count = 0; };\n");
      fprintf(header, "MtDenseDutyLane mtDutyLanes[%d];\n", threadCount + 1);
      fprintf(header, "static constexpr int kDenseDutyLaneMax = %d;\n", threadCount);
      fprintf(header, "bool mtDutyEnabled = false;\n");
      // RAII span recorder: survives the early returns inside the worker switch cases.
      fprintf(header, "struct MtDenseDutyGuard { uint64_t* acc; std::chrono::steady_clock::time_point t0; MtDenseDutyGuard(uint64_t* a) : acc(a), t0(std::chrono::steady_clock::now()) {} ~MtDenseDutyGuard() { if (acc) *acc += (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count(); } };\n");
    }
    fprintf(header, "static_assert(sizeof(std::atomic<uint8_t>) == 1, \"owner-ready atomic must occupy one byte\");\n");
    fprintf(header, "static_assert(std::atomic<uint8_t>::is_always_lock_free, \"owner-ready atomic must be lock-free\");\n");
    fprintf(header, "static_assert(sizeof(MtDenseOwnerReadyToken) == 1, \"owner-ready slots must have one-byte extent\");\n");
    fprintf(header, "static_assert((kDenseOwnerReadyPhysicalSlotCount %% 64) == 0, \"owner-ready storage must end on a 64-byte boundary\");\n");
    fprintf(header, "alignas(64) MtDenseOwnerReadyToken mtDenseOwnerReadyTokens[%d];\n",
            ownerReadyLayout.physicalSlotCount);
    fprintf(header, "bool mtDenseOwnerReadyTokensPrimed = false;\n");
    if (denseLookahead) {
      fprintf(header, "static constexpr int kDenseOwnerReadyWaitList[%d] = {",
              std::max(1, (int)denseDispatchWaitTotal));
      bool firstSlot = true;
      // GSIM_EMIT_SORTED_WAITS=1 (default off): sort each mtask's wait-slot
      // list by token index. Conjunction is commutative so readiness semantics
      // are unchanged; grouping same-line tokens (64/line) converts scattered
      // acquire loads into sequential same-line accesses that stay L1-resident.
      // Attacks the 136G scan/dispatch pool (attribution-revision-analytical).
      bool sortWaits = false;
      { const char* e = std::getenv("GSIM_EMIT_SORTED_WAITS"); sortWaits = e && e[0] && e[0] != '0'; }
      for (int t = 0; t < threadCount; t++) {
        for (int m = 0; m < nMTasks; m++) {
          if (denseSchedule.mtaskThreadAssign[(size_t)m] != t) continue;
          std::vector<int> slots = ownerReadyLayout.waitSlotsByMTask[(size_t)m];
          if (sortWaits && slots.size() > 1) std::sort(slots.begin(), slots.end());
          for (int slot : slots) {
            if (!firstSlot) fprintf(header, ",");
            fprintf(header, "%d", slot);
            firstSlot = false;
          }
        }
      }
      if (firstSlot) fprintf(header, "0");
      fprintf(header, "};\n");
        int denseLookaheadDoneWordCount = 1;
        for (int count : denseDispatchWorkerCounts)
          denseLookaheadDoneWordCount = std::max(denseLookaheadDoneWordCount, (count + 63) / 64);
        fprintf(header, "static constexpr uint32_t kDenseLookaheadWindow = %du;\n", denseLookaheadWindow);
        fprintf(header, "static constexpr uint32_t kDenseLookaheadDoneWordCount = %du;\n", denseLookaheadDoneWordCount);
        fprintf(header, "static constexpr uint32_t kDenseLookaheadLocalPrereqs[%zu] = {", std::max<size_t>(1, denseLookaheadLocalPrereqs.size()));
        for (size_t i = 0; i < denseLookaheadLocalPrereqs.size(); ++i) fprintf(header, "%s%u", i ? "," : "", denseLookaheadLocalPrereqs[i]);
        if (denseLookaheadLocalPrereqs.empty()) fprintf(header, "0");
        fprintf(header, "};\n");
        fprintf(header, "static constexpr uint8_t kDenseLookaheadLocalPrereqKinds[%zu] = {", std::max<size_t>(1, denseLookaheadLocalPrereqKinds.size()));
        for (size_t i = 0; i < denseLookaheadLocalPrereqKinds.size(); ++i) fprintf(header, "%s%u", i ? "," : "", static_cast<unsigned>(denseLookaheadLocalPrereqKinds[i]));
        if (denseLookaheadLocalPrereqKinds.empty()) fprintf(header, "0");
        fprintf(header, "};\n");
        fprintf(header, "static constexpr uint32_t kDenseLookaheadGroupDestination[%d] = {", std::max(1, ownerReadyLayout.tokenCount));
        for (int token = 0; token < ownerReadyLayout.tokenCount; ++token) fprintf(header, "%s%u", token ? "," : "", static_cast<unsigned>(ownerReadyLayout.tokenProvenanceByLogicalToken[(size_t)token].consumerMTask));
        if (ownerReadyLayout.tokenCount == 0) fprintf(header, "0");
        fprintf(header, "};\n");
        fprintf(header, "static constexpr uint32_t kDenseLookaheadGroupProducerOwner[%d] = {", std::max(1, ownerReadyLayout.tokenCount));
        for (int token = 0; token < ownerReadyLayout.tokenCount; ++token) fprintf(header, "%s%u", token ? "," : "", static_cast<unsigned>(ownerReadyLayout.tokenProvenanceByLogicalToken[(size_t)token].producerOwner));
        if (ownerReadyLayout.tokenCount == 0) fprintf(header, "0");
        fprintf(header, "};\n");
        fprintf(header, "static constexpr uint32_t kDenseLookaheadGroupPublisher[%d] = {", std::max(1, ownerReadyLayout.tokenCount));
        for (int token = 0; token < ownerReadyLayout.tokenCount; ++token) fprintf(header, "%s%u", token ? "," : "", static_cast<unsigned>(ownerReadyLayout.tokenProvenanceByLogicalToken[(size_t)token].producerMTask));
        if (ownerReadyLayout.tokenCount == 0) fprintf(header, "0");
        fprintf(header, "};\n");
        fprintf(header, "static constexpr uint32_t kDenseLookaheadGroupSourceOffsets[%d] = {", ownerReadyLayout.tokenCount + 1);
        uint32_t sourceOffset = 0;
        for (int token = 0; token < ownerReadyLayout.tokenCount; ++token) {
          fprintf(header, "%s%u", token ? "," : "", sourceOffset);
          sourceOffset += static_cast<uint32_t>(ownerReadyLayout.sourceMTasksByLogicalToken[(size_t)token].size());
        }
        fprintf(header, "%s%u};\n", ownerReadyLayout.tokenCount ? "," : "", sourceOffset);
        fprintf(header, "static constexpr uint32_t kDenseLookaheadGroupSources[%u] = {", std::max(1u, sourceOffset));
        bool firstSource = true;
        for (const std::vector<int>& sources : ownerReadyLayout.sourceMTasksByLogicalToken) {
          for (int source : sources) { fprintf(header, "%s%u", firstSource ? "" : ",", static_cast<unsigned>(source)); firstSource = false; }
        }
        if (firstSource) fprintf(header, "0");
        fprintf(header, "};\n");
        fprintf(header, "struct MtDenseDispatchEntry { void (S%s::*fn)(); uint32_t waitBegin; uint32_t waitEnd; uint32_t storeBegin; uint32_t storeEnd; uint32_t localBegin; uint32_t localEnd; };\n",
                name.c_str());
        if (denseDuty) {
          fprintf(header, "void stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtDutyLane);\n");
        } else {
          fprintf(header, "void stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target);\n");
        }
      for (int t = 0; t < threadCount; t++) {
        fprintf(header, "static const MtDenseDispatchEntry kDenseDispatchTableW%d[%d];\n",
                t, std::max(1, denseDispatchWorkerCounts[(size_t)t]));
      }
    }
    fprintf(header, "#else\n");
    emitDenseDepCounts();
    emitDenseOwnerBankDiagnostics();
    emitDenseSuccOffsets();
    emitDenseSuccList();
    fprintf(header, "struct MtDenseMTaskVertex { std::atomic<uint32_t> depsDone{0}; };\n");
    fprintf(header, "MtDenseMTaskVertex mtDenseMTaskVertices[%d];\n", nMTasks);
    fprintf(header, "#endif\n");
  }
  fprintf(header, "void stepDenseThreadWorker(int threadId);\n");
  for (int i = 0; i < nMTasks; i++) fprintf(header, "void stepDenseMTask%d();\n", i);
  bool workerMajorText = mtUseDenseWorkerMajorText();
  std::vector<int> denseMTaskEmissionOrder;
  denseMTaskEmissionOrder.reserve((size_t)nMTasks);
  if (workerMajorText) {
    for (int owner = 0; owner < threadCount; owner++) {
      for (int mtaskId = 0; mtaskId < nMTasks; mtaskId++) {
        if (denseSchedule.mtaskThreadAssign[(size_t)mtaskId] == owner)
          denseMTaskEmissionOrder.push_back(mtaskId);
      }
    }
    Assert((int)denseMTaskEmissionOrder.size() == nMTasks,
           "worker-major text emission did not cover all dense MTasks");
    int originalRuns = nMTasks > 0 ? 1 : 0;
    for (int mtaskId = 1; mtaskId < nMTasks; mtaskId++) {
      if (denseSchedule.mtaskThreadAssign[(size_t)mtaskId] !=
          denseSchedule.mtaskThreadAssign[(size_t)mtaskId - 1]) originalRuns++;
    }
    int emittedRuns = nMTasks > 0 ? 1 : 0;
    for (int i = 1; i < nMTasks; i++) {
      if (denseSchedule.mtaskThreadAssign[(size_t)denseMTaskEmissionOrder[(size_t)i]] !=
          denseSchedule.mtaskThreadAssign[(size_t)denseMTaskEmissionOrder[(size_t)i - 1]]) emittedRuns++;
    }
    fprintf(stderr,
            "[mt-dense-worker-major-text] mtasks=%d original_owner_runs=%d emitted_owner_runs=%d threads=%d\n",
            nMTasks, originalRuns, emittedRuns, threadCount);
  } else {
    for (int mtaskId = 0; mtaskId < nMTasks; mtaskId++)
      denseMTaskEmissionOrder.push_back(mtaskId);
  }
  if (emitPhaseTimingEnabled()) {
    fprintf(stderr, "[emit-phase] Final.densePrologue.headerTables = %ld ms\n",
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - denseProHeaderTablesBegin).count());
  }
  if (emitPhaseTimingEnabled()) {
    long prologueMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - densePrologueBegin).count();
    fprintf(stderr, "[emit-phase] Final.densePrologue.total = %ld ms\n", prologueMs);
  }
  {
  EmitPhaseTimer denseBodyTimer("Final.denseMTaskBodies");
  // Each stepDenseMTaskN body is an independent emission unit (reads frozen
  // schedule + graph; per-super emission context flags are thread-local and
  // saved/restored around genSuperEval exactly as in the sequential loop).
  // Unit u renders denseMTaskEmissionOrder[u]; assembly replays buffers in
  // emission order, so output is byte-identical.
  emitUnitsParallel(denseMTaskEmissionOrder.size(), [this, &denseSchedule, &denseMTaskEmissionOrder](size_t unit) {
    int mtaskId = denseMTaskEmissionOrder[unit];

    const MtDenseMTask& mtask = denseSchedule.mtasks[mtaskId];
    emitFuncDecl(0, "void S%s::stepDenseMTask%d() {\n", name.c_str(), mtaskId);
    // Sparse-in-dense: emit members ASCENDING by cppId (proven valid topo order of intra-MTask
    // forward activation edges: 0 backward in cppId order). Gather then sort.
    std::vector<int> memberCppIds;
    for (int sccId : mtask.sccIds) {
      Assert(sccId >= 0 && sccId < static_cast<int>(denseSchedule.sccs.size()), "dense schedule scc index out of range");
      const MtDenseScc& scc = denseSchedule.sccs[(size_t)sccId];
      for (int cppId : scc.cppIds) memberCppIds.push_back(cppId);
    }
    for (int cppId : memberCppIds) {
      auto superIter = cppId2Super.find(cppId);
      if (superIter == cppId2Super.end() || !superIter->second) continue;
      SuperNode* super = superIter->second;
      emitBodyLock(1, "{\n");
      emitBodyLock(2, "std::chrono::steady_clock::time_point mtProfileDenseTaskBegin;\n");
      emitBodyLock(2, "if (unlikely(mtProfileEnabled)) mtProfileDenseTaskBegin = std::chrono::steady_clock::now();\n");
      genSuperEval(super, "activeFlags[0]", "", 2, false);
      emitBodyLock(2, "if (unlikely(mtProfileEnabled)) recordMtProfileTask(%d, true, std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileDenseTaskBegin).count());\n", cppId);
      emitBodyLock(1, "}\n");
    }
    emitBodyLock(0, "}\n");
  });
  }

  auto emitFixedDenseThreadWorker = [&](const char* funcName) {
    emitFuncDecl(0, "void S%s::%s(int threadId) {\n", name.c_str(), funcName);
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(1, "const int mtDenseBreakdownWindowSlot = mtDenseBreakdownWindowCurrentSlot;\n");
      emitBodyLock(1, "const bool mtDenseBreakdownWindow = mtDenseBreakdownWindowSlot >= 0;\n");
      emitBodyLock(1, "MtDenseBreakdownWindowWorker *mtDenseBreakdownWindowWorker = nullptr;\n");
      emitBodyLock(1, "int mtDenseBreakdownWindowWorker0MTaskNext = 0;\n");
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownWindow)) {\n");
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownWindowSlot >= kDenseBreakdownWindowMaxCycles || threadId < 0 || threadId >= kDenseBreakdownWindowThreadCount)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] invalid window worker lane\\n\"); abort(); }\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker = &mtDenseBreakdownWindowWorkers[mtDenseBreakdownWindowSlot][threadId];\n");
      emitBodyLock(2, "const uint64_t mtDenseBreakdownWindowStartOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDenseBreakdownWindowEpoch).count();\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker->startOffsetNs = mtDenseBreakdownWindowStartOffsetNs;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker->finishOffsetNs = 0;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker->blockedWaitNs = 0;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker->bodyNs = 0;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker->controlNs = 0;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWaitCounts[mtDenseBreakdownWindowSlot][threadId] = 0;\n");
      emitBodyLock(1, "}\n");
    }
    if (denseBreakdownProfileCodegen) {
      if (denseBreakdownWindowCodegen) {
        emitBodyLock(1, "const bool mtDenseBreakdownProfile = mtDenseBreakdownProfileEnabled && mtDenseBreakdownWindow;\n");
      } else {
        emitBodyLock(1, "const bool mtDenseBreakdownProfile = mtDenseBreakdownProfileEnabled;\n");
      }
      emitBodyLock(1, "std::chrono::steady_clock::time_point mtDenseBreakdownDispatchBegin;\n");
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownProfile)) mtDenseBreakdownDispatchBegin = std::chrono::steady_clock::now();\n");
    }

    if (denseDuty) emitBodyLock(1, "MtDenseDutyGuard mtDutyGuard(mtDutyEnabled && threadId >= 0 && threadId <= %d ? &mtDutyLanes[threadId].spanNs : nullptr);\n", threadCount);
    emitBodyLock(1, "bool evenCycle = (cycles & 1) == 0;\n");
    emitBodyLock(1, "switch (threadId) {\n");
    for (int t = 0; t < threadCount; t++) {
      emitBodyLock(2, "case %d: {\n", t);
      if (denseLookahead) {
        emitBodyLock(3, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
          emitBodyLock(3, "{ const uint8_t target = evenCycle ? uint8_t{1} : uint8_t{0};\n");
          int tablePosition = 0;
          for (int mtaskId = 0; mtaskId < nMTasks; ++mtaskId) {
            if (denseSchedule.mtaskThreadAssign[(size_t)mtaskId] != t) continue;
            emitBodyLock(4, "{ bool mtDenseInlineReady = true;\n");
            for (int slot : ownerReadyLayout.waitSlotsByMTask[(size_t)mtaskId]) {
              emitBodyLock(5, "mtDenseInlineReady &= (mtDenseOwnerReadyTokens[%d].ready.load(std::memory_order_acquire) == target);\n", slot);
            }
            if (denseDuty) {
              emitBodyLock(5, "if (!mtDenseInlineReady) { stepDenseLookaheadTail(kDenseDispatchTableW%d, kDenseDispatchTableW%d + %d, %uu, target, %du); return; }\n",
                           t, t, denseDispatchWorkerCounts[(size_t)t], static_cast<unsigned>(tablePosition), t);
            } else {
              emitBodyLock(5, "if (!mtDenseInlineReady) { stepDenseLookaheadTail(kDenseDispatchTableW%d, kDenseDispatchTableW%d + %d, %uu, target); return; }\n",
                           t, t, denseDispatchWorkerCounts[(size_t)t], static_cast<unsigned>(tablePosition));
            }
            emitBodyLock(5, "stepDenseMTask%d();\n", mtaskId);
            for (int slot : ownerReadyLayout.storeSlotsByMTask[(size_t)mtaskId]) {
              emitBodyLock(5, "mtDenseOwnerReadyTokens[%d].ready.store(target, std::memory_order_release);\n", slot);
            }
            emitBodyLock(4, "}\n");
            ++tablePosition;
          }
          emitBodyLock(3, "}\n");
        emitBodyLock(3, "#else\n");
      }
      for (int mtaskId = 0; mtaskId < nMTasks; mtaskId++) {
        if (denseSchedule.mtaskThreadAssign[mtaskId] != t) continue;
        const bool skipDenseWait = staticEmptyElide && denseRuntimeDepCounts[(size_t)mtaskId] == 0;
        if (!ownerReadyFlags) {
          if (!skipDenseWait) {
            // Fixed dependency-counter fallback keeps the historical 256-spin yield budget.
            emitBodyLock(3, "{ const uint32_t target = evenCycle ? kDenseMTaskDepCount[%d] : 0u;\n", mtaskId);
            emitBodyLock(3, "  unsigned ct = 0;\n");
            emitBodyLock(3, "  while (mtDenseMTaskVertices[%d].depsDone.load(std::memory_order_acquire) != target) {\n", mtaskId);
            emitBodyLock(4, "mtWorkerPoolPause(); if (++ct > 256) { ct = 0; std::this_thread::yield(); } }\n");
            emitBodyLock(3, "}\n");
          }
        } else {
          emitBodyLock(3, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
          for (int slot : ownerReadyLayout.waitSlotsByMTask[(size_t)mtaskId]) {
            if (denseBreakdownProfileCodegen) {
              emitBodyLock(3, "{ const uint8_t target = evenCycle ? uint8_t{1} : uint8_t{0};\n");
              if (denseBreakdownWindowCodegen) {
                emitBodyLock(3, "  if (unlikely(mtDenseBreakdownProfile && !(mtDenseBreakdownWindow && mtDenseBreakdownWindowFinishOnlyMode))) {\n");
              } else {
                emitBodyLock(3, "  if (unlikely(mtDenseBreakdownProfile)) {\n");
              }
              emitBodyLock(4, "bool mtDenseBreakdownBlocked = false;\n");
              emitBodyLock(4, "std::chrono::steady_clock::time_point mtDenseBreakdownBlockedBegin;\n");
              emitBodyLock(4, "unsigned ct = 0;\n");
              emitBodyLock(4, "while (mtDenseOwnerReadyTokens[%d].ready.load(std::memory_order_acquire) != target) {\n", slot);
              emitBodyLock(5, "if (!mtDenseBreakdownBlocked) { mtDenseBreakdownBlocked = true; mtDenseBreakdownBlockedBegin = std::chrono::steady_clock::now(); }\n");
              emitBodyLock(5, "mtWorkerPoolPause(); if (++ct > 256) { ct = 0; std::this_thread::yield(); }\n");
              emitBodyLock(4, "}\n");
              emitBodyLock(4, "if (mtDenseBreakdownBlocked) {\n");
              emitBodyLock(5, "MtDenseBreakdownWorker &mtDenseBreakdownWorker = mtDenseBreakdownWorkers[threadId];\n");
              if (denseBreakdownWindowCodegen) {
                emitBodyLock(5, "const std::chrono::steady_clock::time_point mtDenseBreakdownBlockedEnd = std::chrono::steady_clock::now();\n");
                emitBodyLock(5, "uint64_t mtDenseBreakdownBlockedNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownBlockedEnd - mtDenseBreakdownBlockedBegin).count();\n");
              } else {
                emitBodyLock(5, "uint64_t mtDenseBreakdownBlockedNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDenseBreakdownBlockedBegin).count();\n");
              }
              emitBodyLock(5, "if (unlikely(UINT64_MAX - mtDenseBreakdownWorker.blockedWaitNs < mtDenseBreakdownBlockedNs || mtDenseBreakdownWorker.blockedWaitCount == UINT64_MAX)) { fprintf(stderr, \"[mt-dense-breakdown] blocked-wait counter overflow\\n\"); abort(); }\n");
              emitBodyLock(5, "mtDenseBreakdownWorker.blockedWaitNs += mtDenseBreakdownBlockedNs;\n");
              emitBodyLock(5, "mtDenseBreakdownWorker.blockedWaitCount += 1;\n");
              if (denseBreakdownWindowCodegen) {
                emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindow)) { if (unlikely(UINT64_MAX - mtDenseBreakdownWindowWorker->blockedWaitNs < mtDenseBreakdownBlockedNs)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window blocked-wait overflow\\n\"); abort(); } mtDenseBreakdownWindowWorker->blockedWaitNs += mtDenseBreakdownBlockedNs; }\n");
              }
              if (denseBreakdownWindowCodegen) {
                emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindow)) { uint32_t &mtDenseBreakdownWindowWaitCount = mtDenseBreakdownWindowWaitCounts[mtDenseBreakdownWindowSlot][threadId]; const uint32_t mtDenseBreakdownWindowWaitCapacity = (uint32_t)(kDenseBreakdownWindowWaitLaneOffsets[threadId + 1] - kDenseBreakdownWindowWaitLaneOffsets[threadId]); if (unlikely(mtDenseBreakdownWindowWaitCount >= mtDenseBreakdownWindowWaitCapacity)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window ready-wait record overflow\\n\"); abort(); } MtDenseBreakdownWindowWait &mtDenseBreakdownWindowWait = mtDenseBreakdownWindowWaits[mtDenseBreakdownWindowSlot][kDenseBreakdownWindowWaitLaneOffsets[threadId] + mtDenseBreakdownWindowWaitCount ++]; mtDenseBreakdownWindowWait.cycleSlot = (uint16_t)mtDenseBreakdownWindowSlot; mtDenseBreakdownWindowWait.threadId = (uint16_t)threadId; mtDenseBreakdownWindowWait.consumerMtaskId = %d; mtDenseBreakdownWindowWait.readySlot = %d; mtDenseBreakdownWindowWait.blockedNs = mtDenseBreakdownBlockedNs; }\n", mtaskId, slot);
                emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindow)) { uint64_t mtDenseBreakdownWindowWaitEndOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownBlockedEnd - mtDenseBreakdownWindowEpoch).count(); if (unlikely(mtDenseBreakdownWindowWaitEndOffsetNs < mtDenseBreakdownBlockedNs)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window ready-wait timeline underflow\\n\"); abort(); } const uint32_t mtDenseBreakdownWindowWaitIndex = mtDenseBreakdownWindowWaitCounts[mtDenseBreakdownWindowSlot][threadId] - 1; mtDenseBreakdownWindowWaits[mtDenseBreakdownWindowSlot][kDenseBreakdownWindowWaitLaneOffsets[threadId] + mtDenseBreakdownWindowWaitIndex].endOffsetNs = mtDenseBreakdownWindowWaitEndOffsetNs; }\n");
              }
              if (denseBreakdownWindowCodegen) {
                emitBodyLock(4, "} else if (unlikely(mtDenseBreakdownWindow && mtDenseBreakdownWindowCausalChainMode)) {\n");
                emitBodyLock(5, "uint32_t &mtDenseBreakdownWindowWaitCount = mtDenseBreakdownWindowWaitCounts[mtDenseBreakdownWindowSlot][threadId];\n");
                emitBodyLock(5, "const uint32_t mtDenseBreakdownWindowWaitCapacity = (uint32_t)(kDenseBreakdownWindowWaitLaneOffsets[threadId + 1] - kDenseBreakdownWindowWaitLaneOffsets[threadId]);\n");
                emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowWaitCount >= mtDenseBreakdownWindowWaitCapacity)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window causal wait record overflow\\n\"); abort(); }\n");
                emitBodyLock(5, "const std::chrono::steady_clock::time_point mtDenseBreakdownWindowWaitEnd = std::chrono::steady_clock::now();\n");
                emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowWaitEnd < mtDenseBreakdownWindowEpoch)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window causal wait clock regression\\n\"); abort(); }\n");
                emitBodyLock(5, "MtDenseBreakdownWindowWait &mtDenseBreakdownWindowWait = mtDenseBreakdownWindowWaits[mtDenseBreakdownWindowSlot][kDenseBreakdownWindowWaitLaneOffsets[threadId] + mtDenseBreakdownWindowWaitCount ++];\n");
                emitBodyLock(5, "mtDenseBreakdownWindowWait.cycleSlot = (uint16_t)mtDenseBreakdownWindowSlot; mtDenseBreakdownWindowWait.threadId = (uint16_t)threadId; mtDenseBreakdownWindowWait.consumerMtaskId = %d; mtDenseBreakdownWindowWait.readySlot = %d; mtDenseBreakdownWindowWait.blockedNs = 0; mtDenseBreakdownWindowWait.endOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowWaitEnd - mtDenseBreakdownWindowEpoch).count();\n", mtaskId, slot);
                emitBodyLock(4, "}\n");
              } else {
                emitBodyLock(4, "}\n");
              }
              emitBodyLock(3, "  } else {\n");
              emitBodyLock(4, "unsigned ct = 0;\n");
              emitBodyLock(4, "while (mtDenseOwnerReadyTokens[%d].ready.load(std::memory_order_acquire) != target) {\n", slot);
              emitBodyLock(5, "mtWorkerPoolPause(); if (++ct > 256) { ct = 0; std::this_thread::yield(); }\n");
              emitBodyLock(4, "}\n");
              emitBodyLock(3, "  }\n");
              emitBodyLock(3, "}\n");
            } else {
              emitBodyLock(3, "{ const uint8_t target = evenCycle ? uint8_t{1} : uint8_t{0};\n");
              emitBodyLock(3, "  unsigned ct = 0;\n");
              emitBodyLock(3, "  while (mtDenseOwnerReadyTokens[%d].ready.load(std::memory_order_acquire) != target) {\n", slot);
              emitBodyLock(4, "mtWorkerPoolPause(); if (++ct > 256) { ct = 0; std::this_thread::yield(); } }\n");
              emitBodyLock(3, "}\n");
            }

          }
          emitBodyLock(3, "#else\n");
          if (!skipDenseWait) {
            emitBodyLock(3, "{ const uint32_t target = evenCycle ? kDenseMTaskDepCount[%d] : 0u;\n", mtaskId);
            emitBodyLock(3, "  unsigned ct = 0;\n");
            emitBodyLock(3, "  while (mtDenseMTaskVertices[%d].depsDone.load(std::memory_order_acquire) != target) {\n", mtaskId);
            emitBodyLock(4, "mtWorkerPoolPause(); if (++ct > 256) { ct = 0; std::this_thread::yield(); } }\n");
            emitBodyLock(3, "}\n");
          }
          emitBodyLock(3, "#endif\n");
        }
        if (denseBreakdownWindowCodegen) {
          emitBodyLock(3, "if (unlikely(mtDenseBreakdownWindow && (mtDenseBreakdownWindowAllOwnerBodyMode || mtDenseBreakdownWindowCausalChainMode))) {\n");
          emitBodyLock(4, "MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowAllOwnerMTask = mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowSlot][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[%d]];\n", mtaskId);
          emitBodyLock(4, "mtDenseBreakdownWindowAllOwnerMTask.mtaskId = %d;\n", mtaskId);
          emitBodyLock(4, "mtDenseBreakdownWindowAllOwnerMTask.ownerThreadId = (uint16_t)threadId;\n");
          emitBodyLock(4, "mtDenseBreakdownWindowAllOwnerMTask.readyTokenStoreCount = %d;\n", (int)ownerReadyLayout.storeSlotsByMTask[(size_t)mtaskId].size());
          emitBodyLock(4, "mtDenseBreakdownWindowAllOwnerMTask.releaseEndOffsetNs = UINT64_MAX;\n");
          emitBodyLock(4, "std::chrono::steady_clock::time_point mtDenseBreakdownWindowBodyBegin = std::chrono::steady_clock::now();\n");
          emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowCausalChainMode && mtDenseBreakdownWindowBodyBegin < mtDenseBreakdownWindowEpoch)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal body start clock regression\\n\"); abort(); }\n");
          emitBodyLock(4, "mtDenseBreakdownWindowAllOwnerMTask.bodyStartOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowBodyBegin - mtDenseBreakdownWindowEpoch).count();\n");
          emitBodyLock(4, "stepDenseMTask%d();\n", mtaskId);
          emitBodyLock(4, "std::chrono::steady_clock::time_point mtDenseBreakdownWindowBodyEnd = std::chrono::steady_clock::now();\n");
          emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowCausalChainMode && mtDenseBreakdownWindowBodyEnd < mtDenseBreakdownWindowEpoch)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal body end clock regression\\n\"); abort(); }\n");
          emitBodyLock(4, "mtDenseBreakdownWindowAllOwnerMTask.bodyEndOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowBodyEnd - mtDenseBreakdownWindowEpoch).count();\n");
          emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowAllOwnerMTask.bodyEndOffsetNs < mtDenseBreakdownWindowAllOwnerMTask.bodyStartOffsetNs)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] all-owner body timeline underflow\\n\"); abort(); }\n");
          emitBodyLock(4, "uint64_t mtDenseBreakdownWindowBodyNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowBodyEnd - mtDenseBreakdownWindowBodyBegin).count();\n");
          emitBodyLock(4, "if (unlikely(UINT64_MAX - mtDenseBreakdownWindowWorker->bodyNs < mtDenseBreakdownWindowBodyNs)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window all-owner body counter overflow\\n\"); abort(); }\n");
          emitBodyLock(4, "mtDenseBreakdownWindowAllOwnerMTask.bodyNs = mtDenseBreakdownWindowBodyNs;\n");
          emitBodyLock(4, "mtDenseBreakdownWindowWorker->bodyNs += mtDenseBreakdownWindowBodyNs;\n");
          if (t == 0) {
            emitBodyLock(3, "} else if (unlikely(mtDenseBreakdownWindow && mtDenseBreakdownWindowWorker0BodyMode)) {\n");
            emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowWorker0MTaskNext >= kDenseBreakdownWindowWorker0MTaskCount)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window worker0 MTask record overflow\\n\"); abort(); }\n");
            emitBodyLock(4, "MtDenseBreakdownWindowMTask &mtDenseBreakdownWindowMTask = mtDenseBreakdownWindowWorker0MTasks[mtDenseBreakdownWindowSlot][mtDenseBreakdownWindowWorker0MTaskNext ++];\n");
            emitBodyLock(4, "mtDenseBreakdownWindowMTask.mtaskId = %d;\n", mtaskId);
            emitBodyLock(4, "std::chrono::steady_clock::time_point mtDenseBreakdownWindowBodyBegin = std::chrono::steady_clock::now();\n");
            emitBodyLock(4, "stepDenseMTask%d();\n", mtaskId);
            emitBodyLock(4, "uint64_t mtDenseBreakdownWindowBodyNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDenseBreakdownWindowBodyBegin).count();\n");
            emitBodyLock(4, "if (unlikely(UINT64_MAX - mtDenseBreakdownWindowWorker->bodyNs < mtDenseBreakdownWindowBodyNs)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window body counter overflow\\n\"); abort(); }\n");
            emitBodyLock(4, "mtDenseBreakdownWindowMTask.bodyNs = mtDenseBreakdownWindowBodyNs;\n");
            emitBodyLock(4, "mtDenseBreakdownWindowWorker->bodyNs += mtDenseBreakdownWindowBodyNs;\n");
          }
          emitBodyLock(3, "} else {\n");
          emitBodyLock(4, "stepDenseMTask%d();\n", mtaskId);
          emitBodyLock(3, "}\n");
        } else {
          emitBodyLock(3, "stepDenseMTask%d();\n", mtaskId);
        }
        const bool skipDenseSignal = staticEmptyElide && denseRuntimeSuccs[(size_t)mtaskId].empty();
        if (!ownerReadyFlags) {
          if (!skipDenseSignal) {
            // Verilator signalUpstreamDone: fetch_add (even) or fetch_sub (odd).
            emitBodyLock(3, "if (evenCycle) {\n");
            emitBodyLock(4, "for (int j = kDenseMTaskSuccOffsets[%d]; j < kDenseMTaskSuccOffsets[%d]; j++)\n", mtaskId, mtaskId + 1);
            emitBodyLock(5, "mtDenseMTaskVertices[kDenseMTaskSuccList[j]].depsDone.fetch_add(1, std::memory_order_release);\n");
            emitBodyLock(3, "} else {\n");
            emitBodyLock(4, "for (int j = kDenseMTaskSuccOffsets[%d]; j < kDenseMTaskSuccOffsets[%d]; j++)\n", mtaskId, mtaskId + 1);
            emitBodyLock(5, "mtDenseMTaskVertices[kDenseMTaskSuccList[j]].depsDone.fetch_sub(1, std::memory_order_release);\n");
            emitBodyLock(3, "}\n");
          }
        } else {
          emitBodyLock(3, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
          if (!ownerReadyLayout.storeSlotsByMTask[(size_t)mtaskId].empty()) {
            emitBodyLock(3, "if (evenCycle) {\n");
            emitBodyLock(4, "for (int j = kDenseOwnerReadyStoreOffsets[%d]; j < kDenseOwnerReadyStoreOffsets[%d]; j++) {\n", mtaskId, mtaskId + 1);
            if (denseBreakdownProfileCodegen) {
              emitBodyLock(5, "const int mtDenseBreakdownWindowReadySlot = kDenseOwnerReadyStoreList[j];\n");
              emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindow && mtDenseBreakdownWindowCausalChainMode)) { const int mtDenseBreakdownWindowLogicalToken = kDenseBreakdownWindowCausalLogicalTokenByReadySlot[mtDenseBreakdownWindowReadySlot]; if (unlikely(mtDenseBreakdownWindowLogicalToken < 0 || mtDenseBreakdownWindowLogicalToken >= kDenseBreakdownWindowCausalTokenCount || kDenseBreakdownWindowCausalTokenReadySlot[mtDenseBreakdownWindowLogicalToken] != mtDenseBreakdownWindowReadySlot || kDenseBreakdownWindowCausalTokenProducerMTask[mtDenseBreakdownWindowLogicalToken] != %d || kDenseBreakdownWindowCausalTokenProducerOwner[mtDenseBreakdownWindowLogicalToken] != threadId)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token provenance mismatch\\n\"); abort(); } MtDenseBreakdownWindowReadyToken &mtDenseBreakdownWindowTokenRelease = mtDenseBreakdownWindowReadyTokens[mtDenseBreakdownWindowSlot][mtDenseBreakdownWindowLogicalToken]; const std::chrono::steady_clock::time_point mtDenseBreakdownWindowReleaseBefore = std::chrono::steady_clock::now(); if (unlikely(mtDenseBreakdownWindowReleaseBefore < mtDenseBreakdownWindowEpoch || mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs != UINT64_MAX)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token release-before invalid\\n\"); abort(); } mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowReleaseBefore - mtDenseBreakdownWindowEpoch).count(); mtDenseOwnerReadyTokens[mtDenseBreakdownWindowReadySlot].ready.store(uint8_t{1}, std::memory_order_release); const std::chrono::steady_clock::time_point mtDenseBreakdownWindowReleaseAfter = std::chrono::steady_clock::now(); if (unlikely(mtDenseBreakdownWindowReleaseAfter < mtDenseBreakdownWindowReleaseBefore)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token release-after clock regression\\n\"); abort(); } mtDenseBreakdownWindowTokenRelease.releaseAfterOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowReleaseAfter - mtDenseBreakdownWindowEpoch).count(); } else { mtDenseOwnerReadyTokens[mtDenseBreakdownWindowReadySlot].ready.store(uint8_t{1}, std::memory_order_release); }\n", mtaskId);
            } else {
              emitBodyLock(5, "mtDenseOwnerReadyTokens[kDenseOwnerReadyStoreList[j]].ready.store(uint8_t{1}, std::memory_order_release);\n");
            }
            emitBodyLock(4, "}\n");
            emitBodyLock(3, "} else {\n");
            emitBodyLock(4, "for (int j = kDenseOwnerReadyStoreOffsets[%d]; j < kDenseOwnerReadyStoreOffsets[%d]; j++) {\n", mtaskId, mtaskId + 1);
            if (denseBreakdownProfileCodegen) {
              emitBodyLock(5, "const int mtDenseBreakdownWindowReadySlot = kDenseOwnerReadyStoreList[j];\n");
              emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindow && mtDenseBreakdownWindowCausalChainMode)) { const int mtDenseBreakdownWindowLogicalToken = kDenseBreakdownWindowCausalLogicalTokenByReadySlot[mtDenseBreakdownWindowReadySlot]; if (unlikely(mtDenseBreakdownWindowLogicalToken < 0 || mtDenseBreakdownWindowLogicalToken >= kDenseBreakdownWindowCausalTokenCount || kDenseBreakdownWindowCausalTokenReadySlot[mtDenseBreakdownWindowLogicalToken] != mtDenseBreakdownWindowReadySlot || kDenseBreakdownWindowCausalTokenProducerMTask[mtDenseBreakdownWindowLogicalToken] != %d || kDenseBreakdownWindowCausalTokenProducerOwner[mtDenseBreakdownWindowLogicalToken] != threadId)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token provenance mismatch\\n\"); abort(); } MtDenseBreakdownWindowReadyToken &mtDenseBreakdownWindowTokenRelease = mtDenseBreakdownWindowReadyTokens[mtDenseBreakdownWindowSlot][mtDenseBreakdownWindowLogicalToken]; const std::chrono::steady_clock::time_point mtDenseBreakdownWindowReleaseBefore = std::chrono::steady_clock::now(); if (unlikely(mtDenseBreakdownWindowReleaseBefore < mtDenseBreakdownWindowEpoch || mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs != UINT64_MAX)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token release-before invalid\\n\"); abort(); } mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowReleaseBefore - mtDenseBreakdownWindowEpoch).count(); mtDenseOwnerReadyTokens[mtDenseBreakdownWindowReadySlot].ready.store(uint8_t{0}, std::memory_order_release); const std::chrono::steady_clock::time_point mtDenseBreakdownWindowReleaseAfter = std::chrono::steady_clock::now(); if (unlikely(mtDenseBreakdownWindowReleaseAfter < mtDenseBreakdownWindowReleaseBefore)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token release-after clock regression\\n\"); abort(); } mtDenseBreakdownWindowTokenRelease.releaseAfterOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowReleaseAfter - mtDenseBreakdownWindowEpoch).count(); } else { mtDenseOwnerReadyTokens[mtDenseBreakdownWindowReadySlot].ready.store(uint8_t{0}, std::memory_order_release); }\n", mtaskId);
            } else {
              emitBodyLock(5, "mtDenseOwnerReadyTokens[kDenseOwnerReadyStoreList[j]].ready.store(uint8_t{0}, std::memory_order_release);\n");
            }
            emitBodyLock(4, "}\n");
            emitBodyLock(3, "}\n");
          }
          emitBodyLock(3, "#else\n");
          if (!skipDenseSignal) {
            emitBodyLock(3, "if (evenCycle) {\n");
            emitBodyLock(4, "for (int j = kDenseMTaskSuccOffsets[%d]; j < kDenseMTaskSuccOffsets[%d]; j++)\n", mtaskId, mtaskId + 1);
            emitBodyLock(5, "mtDenseMTaskVertices[kDenseMTaskSuccList[j]].depsDone.fetch_add(1, std::memory_order_release);\n");
            emitBodyLock(3, "} else {\n");
            emitBodyLock(4, "for (int j = kDenseMTaskSuccOffsets[%d]; j < kDenseMTaskSuccOffsets[%d]; j++)\n", mtaskId, mtaskId + 1);
            emitBodyLock(5, "mtDenseMTaskVertices[kDenseMTaskSuccList[j]].depsDone.fetch_sub(1, std::memory_order_release);\n");
            emitBodyLock(3, "}\n");
          }
          emitBodyLock(3, "#endif\n");
        }
        if (denseBreakdownWindowCodegen) {
          emitBodyLock(3, "if (unlikely(mtDenseBreakdownWindow && (mtDenseBreakdownWindowAllOwnerBodyMode || mtDenseBreakdownWindowCausalChainMode))) {\n");
          emitBodyLock(4, "const std::chrono::steady_clock::time_point mtDenseBreakdownWindowReleaseEnd = std::chrono::steady_clock::now();\n");
          emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowCausalChainMode && mtDenseBreakdownWindowReleaseEnd < mtDenseBreakdownWindowEpoch)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal MTask release clock regression\\n\"); abort(); }\n");
          emitBodyLock(4, "mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowSlot][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[%d]].releaseEndOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowReleaseEnd - mtDenseBreakdownWindowEpoch).count();\n", mtaskId);
          emitBodyLock(3, "}\n");
        }
      }
      if (denseLookahead) {
        emitBodyLock(3, "#endif\n");
      }
      emitBodyLock(3, "break;\n");
      emitBodyLock(2, "}\n");
    }
    emitBodyLock(2, "default: break;\n");
    emitBodyLock(1, "}\n");
    if (denseBreakdownProfileCodegen) {
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownProfile)) {\n");
      emitBodyLock(2, "MtDenseBreakdownWorker &mtDenseBreakdownWorker = mtDenseBreakdownWorkers[threadId];\n");
      emitBodyLock(2, "uint64_t mtDenseBreakdownDispatchNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDenseBreakdownDispatchBegin).count();\n");
      emitBodyLock(2, "if (unlikely(UINT64_MAX - mtDenseBreakdownWorker.dispatchSpanNs < mtDenseBreakdownDispatchNs)) { fprintf(stderr, \"[mt-dense-breakdown] dispatch counter overflow\\n\"); abort(); }\n");
      emitBodyLock(2, "mtDenseBreakdownWorker.dispatchSpanNs += mtDenseBreakdownDispatchNs;\n");
      emitBodyLock(1, "}\n");
    }
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownWindow)) {\n");
      emitBodyLock(2, "uint64_t mtDenseBreakdownWindowFinishOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDenseBreakdownWindowEpoch).count();\n");
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownWindowFinishOffsetNs < mtDenseBreakdownWindowWorker->startOffsetNs)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window negative dispatch span\\n\"); abort(); }\n");
      emitBodyLock(2, "uint64_t mtDenseBreakdownWindowDispatchNs = mtDenseBreakdownWindowFinishOffsetNs - mtDenseBreakdownWindowWorker->startOffsetNs;\n");
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownWindowWorker->blockedWaitNs > mtDenseBreakdownWindowDispatchNs || mtDenseBreakdownWindowWorker->bodyNs > mtDenseBreakdownWindowDispatchNs - mtDenseBreakdownWindowWorker->blockedWaitNs)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window attribution exceeds dispatch span\\n\"); abort(); }\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker->finishOffsetNs = mtDenseBreakdownWindowFinishOffsetNs;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker->controlNs = mtDenseBreakdownWindowDispatchNs - mtDenseBreakdownWindowWorker->blockedWaitNs - mtDenseBreakdownWindowWorker->bodyNs;\n");
      emitBodyLock(2, "if (threadId == 0) { if (mtDenseBreakdownWindowWorker0BodyMode) { if (unlikely(mtDenseBreakdownWindowWorker0MTaskNext != kDenseBreakdownWindowWorker0MTaskCount)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window worker0 MTask record count mismatch\\n\"); abort(); } } mtDenseBreakdownWindowWorker0MTaskCounts[mtDenseBreakdownWindowSlot] = (uint32_t)mtDenseBreakdownWindowWorker0MTaskNext; }\n");
      emitBodyLock(1, "}\n");
    }

    emitBodyLock(0, "}\n");
  };
  if (denseLookahead) {
    // Tail-scan instrumentation (E3): zero-cost unless the stats compile macro is set.
    // Counters live in the HEADER as inline variables: the tail function and the
    // dumpMtProfile print land in different SimTop*.cpp shards, so a cpp-local
    // definition would be an undefined reference at link time.
    // emit the #if guard atomically with the function start. A file-split
    // boundary between them orphaned the guard (SimTop1209 ended with #if,
    // SimTop1210 began with the body -> #endif without #if at MAXMT=800).
    if (denseDuty) {
      emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtDutyLane) {\n", name.c_str());
    } else {
      emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target) {\n", name.c_str());
    }
    if (denseDuty) emitBodyLock(1, "MtDenseDutyGuard mtDutyTailGuard(mtDutyEnabled ? &mtDutyLanes[mtDutyLane].tailNs : nullptr);\n");
    emitBodyLock(1, "const uint32_t mtDenseDispatchCount = static_cast<uint32_t>(mtDenseDispatchEnd - mtDenseDispatchBegin);\n");
    emitBodyLock(1, "uint64_t mtDenseDoneBits[kDenseLookaheadDoneWordCount] = {};\n");
    emitBodyLock(1, "bool anyOutOfOrder = false;\n");
    emitBodyLock(1, "uint32_t head = startHead;\n");
    emitBodyLock(1, "while (head < mtDenseDispatchCount) {\n");
    emitBodyLock(2, "const MtDenseDispatchEntry* mtDenseDispatchEntry = mtDenseDispatchBegin + head;\n");
    emitBodyLock(2, "bool mtDenseEntryReady = true;\n");
    emitBodyLock(2, "for (uint32_t mtDenseDispatchWait = mtDenseDispatchEntry->waitBegin; mtDenseDispatchWait < mtDenseDispatchEntry->waitEnd; ++mtDenseDispatchWait) {\n");
    emitBodyLock(3, "mtDenseEntryReady &= (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) == target);\n");
    emitBodyLock(2, "}\n");
    // Tail-scan instrumentation (E3): zero-cost unless the stats compile macro is set.
    emitBodyLock(2, "#if defined(GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE) && GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE\n");
    emitBodyLock(2, "if (!mtDenseEntryReady) mtDenseLookaheadTailCalls.fetch_add(1, std::memory_order_relaxed);\n");
    emitBodyLock(2, "#endif\n");
    emitBodyLock(2, "if (mtDenseEntryReady) {\n");
    emitBodyLock(3, "(this->*mtDenseDispatchEntry->fn)();\n");
    emitBodyLock(3, "for (uint32_t mtDenseDispatchStore = mtDenseDispatchEntry->storeBegin; mtDenseDispatchStore < mtDenseDispatchEntry->storeEnd; ++mtDenseDispatchStore) {\n");
    emitBodyLock(4, "mtDenseOwnerReadyTokens[kDenseOwnerReadyStoreList[mtDenseDispatchStore]].ready.store(target, std::memory_order_release);\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "++head;\n");
    emitBodyLock(3, "while (head < mtDenseDispatchCount && anyOutOfOrder && (mtDenseDoneBits[head >> 6] & (uint64_t{1} << (head & 63))) != 0) {\n");
    emitBodyLock(4, "mtDenseDoneBits[head >> 6] &= ~(uint64_t{1} << (head & 63));\n");
    emitBodyLock(4, "++head;\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "continue;\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(2, "bool progressed = false;\n");
    emitBodyLock(2, "uint32_t mtDenseScanEnd = head + 1u + kDenseLookaheadWindow;\n");
    emitBodyLock(2, "if (mtDenseScanEnd > mtDenseDispatchCount) mtDenseScanEnd = mtDenseDispatchCount;\n");
    emitBodyLock(2, "for (uint32_t j = head + 1u; j < mtDenseScanEnd; ++j) {\n");
    emitBodyLock(3, "#if defined(GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE) && GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE\n");
    emitBodyLock(3, "mtDenseLookaheadScanned.fetch_add(1, std::memory_order_relaxed);\n");
    emitBodyLock(3, "#endif\n");
    emitBodyLock(3, "if (anyOutOfOrder && (mtDenseDoneBits[j >> 6] & (uint64_t{1} << (j & 63))) != 0) continue;\n");
    emitBodyLock(3, "const MtDenseDispatchEntry* mtDenseCandidate = mtDenseDispatchBegin + j;\n");
    emitBodyLock(3, "bool mtDenseCandidateReady = true;\n");
    emitBodyLock(3, "for (uint32_t mtDenseLocalPrereq = mtDenseCandidate->localBegin; mtDenseLocalPrereq < mtDenseCandidate->localEnd; ++mtDenseLocalPrereq) {\n");
    emitBodyLock(4, "const uint32_t mtDensePredPosition = kDenseLookaheadLocalPrereqs[mtDenseLocalPrereq];\n");
    emitBodyLock(4, "if (!(mtDensePredPosition < head || (anyOutOfOrder && (mtDenseDoneBits[mtDensePredPosition >> 6] & (uint64_t{1} << (mtDensePredPosition & 63))) != 0))) { mtDenseCandidateReady = false; break; }\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "if (!mtDenseCandidateReady) continue;\n");
    emitBodyLock(3, "for (uint32_t mtDenseDispatchWait = mtDenseCandidate->waitBegin; mtDenseDispatchWait < mtDenseCandidate->waitEnd; ++mtDenseDispatchWait) {\n");
    emitBodyLock(4, "if (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) != target) { mtDenseCandidateReady = false; break; }\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "if (!mtDenseCandidateReady) continue;\n");
    emitBodyLock(3, "#if defined(GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE) && GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE\n");
    emitBodyLock(3, "mtDenseLookaheadFound.fetch_add(1, std::memory_order_relaxed);\n");
    emitBodyLock(3, "#endif\n");
    emitBodyLock(3, "(this->*mtDenseCandidate->fn)();\n");
    emitBodyLock(3, "for (uint32_t mtDenseDispatchStore = mtDenseCandidate->storeBegin; mtDenseDispatchStore < mtDenseCandidate->storeEnd; ++mtDenseDispatchStore) {\n");
    emitBodyLock(4, "mtDenseOwnerReadyTokens[kDenseOwnerReadyStoreList[mtDenseDispatchStore]].ready.store(target, std::memory_order_release);\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "mtDenseDoneBits[j >> 6] |= uint64_t{1} << (j & 63);\n");
    emitBodyLock(3, "anyOutOfOrder = true;\n");
    emitBodyLock(3, "progressed = true;\n");
    emitBodyLock(3, "break;\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(2, "#if defined(GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE) && GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE\n");
    emitBodyLock(2, "if (!progressed) mtDenseLookaheadFullMiss.fetch_add(1, std::memory_order_relaxed);\n");
    emitBodyLock(2, "#endif\n");
    emitBodyLock(2, "if (progressed) continue;\n");
    if (denseDuty) emitBodyLock(2, "std::chrono::steady_clock::time_point mtDutyBlockBegin; if (mtDutyEnabled) mtDutyBlockBegin = std::chrono::steady_clock::now();\n");
    emitBodyLock(2, "for (uint32_t mtDenseDispatchWait = mtDenseDispatchEntry->waitBegin; mtDenseDispatchWait < mtDenseDispatchEntry->waitEnd; ++mtDenseDispatchWait) {\n");
    emitBodyLock(3, "unsigned ct = 0;\n");
    emitBodyLock(3, "while (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) != target) {\n");
    emitBodyLock(4, "mtWorkerPoolPause(); if (++ct > 256) { ct = 0; std::this_thread::yield(); } }\n");
    emitBodyLock(2, "}\n");
    if (denseDuty) emitBodyLock(2, "if (mtDutyEnabled) mtDutyLanes[mtDutyLane].blockNs += (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDutyBlockBegin).count();\n");
    emitBodyLock(2, "(this->*mtDenseDispatchEntry->fn)();\n");
    emitBodyLock(2, "for (uint32_t mtDenseDispatchStore = mtDenseDispatchEntry->storeBegin; mtDenseDispatchStore < mtDenseDispatchEntry->storeEnd; ++mtDenseDispatchStore) {\n");
    emitBodyLock(3, "mtDenseOwnerReadyTokens[kDenseOwnerReadyStoreList[mtDenseDispatchStore]].ready.store(target, std::memory_order_release);\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(2, "++head;\n");
    emitBodyLock(2, "while (head < mtDenseDispatchCount && anyOutOfOrder && (mtDenseDoneBits[head >> 6] & (uint64_t{1} << (head & 63))) != 0) {\n");
    emitBodyLock(3, "mtDenseDoneBits[head >> 6] &= ~(uint64_t{1} << (head & 63));\n");
    emitBodyLock(3, "++head;\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(0, "}\n");
    emitBodyLock(0, "#endif\n");
  }
  if (denseLookahead) {
    // The entry type, table declarations and the lookahead tail are all gated on the
    // owner-ready compile macro; the definitions must match or a macro-less compile
    // sees definitions of undeclared members (build failure).
    emitBodyLock(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    for (int t = 0; t < threadCount; t++) {
      int cnt = denseDispatchWorkerCounts[(size_t)t];
      emitBodyLock(0, "const S%s::MtDenseDispatchEntry S%s::kDenseDispatchTableW%d[%d] = {\n",
                   name.c_str(), name.c_str(), t, std::max(1, cnt));
      if (cnt == 0) {
        emitBodyLock(0, "{nullptr, 0u, 0u, 0u, 0u, 0u, 0u}\n");
      } else {
        for (int m = 0; m < nMTasks; m++) {
          if (denseSchedule.mtaskThreadAssign[(size_t)m] != t) continue;
          emitBodyLock(0, "{&S%s::stepDenseMTask%d, %uu, %uu, %uu, %uu, %uu, %uu},\n",
                       name.c_str(), m,
                       denseDispatchWaitBegin[(size_t)m], denseDispatchWaitEnd[(size_t)m],
                       denseDispatchStoreBegin[(size_t)m], denseDispatchStoreEnd[(size_t)m],
                       denseLookaheadLocalBegin[(size_t)m], denseLookaheadLocalEnd[(size_t)m]);
        }
      }
      emitBodyLock(0, "};\n");
    }
    emitBodyLock(0, "#endif\n");
  }
  emitFixedDenseThreadWorker("stepDenseThreadWorker");
  emitFuncDecl(0, "void S%s::stepDense() {\n", name.c_str());
  if (denseDuty) emitBodyLock(1, "MtDenseDutyGuard mtDutyStepGuard(mtDutyEnabled ? &mtDutyLanes[%d].stepWallNs : nullptr);\n", threadCount);
  if (denseBreakdownWindowCodegen) {
    emitBodyLock(1, "const bool mtDenseBreakdownWindowInCycle = mtDenseBreakdownWindowEnabled && cycles >= mtDenseBreakdownWindowStart && cycles - mtDenseBreakdownWindowStart < mtDenseBreakdownWindowCycles;\n");
    emitBodyLock(1, "const bool mtDenseBreakdownProfileInCycle = mtDenseBreakdownProfileEnabled && mtDenseBreakdownWindowInCycle;\n");
  }
  if (denseBreakdownProfileCodegen) {
    emitBodyLock(1, "std::chrono::steady_clock::time_point mtDenseBreakdownStepBegin;\n");
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownProfileInCycle)) mtDenseBreakdownStepBegin = std::chrono::steady_clock::now();\n");
    } else {
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownProfileEnabled)) mtDenseBreakdownStepBegin = std::chrono::steady_clock::now();\n");
    }
  }

  emitBodyLock(1, "std::chrono::steady_clock::time_point mtProfileStepBegin;\n");
  emitBodyLock(1, "if (unlikely(mtProfileEnabled)) mtProfileStepBegin = std::chrono::steady_clock::now();\n");
  if (denseDuty) emitBodyLock(1, "std::chrono::steady_clock::time_point mtDutyResetBegin; if (mtDutyEnabled) mtDutyResetBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(1, "resetAllDense();\n");
  if (denseDuty) emitBodyLock(1, "if (mtDutyEnabled) mtDutyLanes[%d].resetNs += (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDutyResetBegin).count();\n", threadCount);
  for (SuperNode* super : sortedSuper) {
    for (Node* member : super->member) {
      if (member->isReset() && member->type == NODE_REG_SRC) {
        emitBodyLock(1, "%s = %s;\n", RESET_NAME(member).c_str(), member->name.c_str());
      }
    }
  }
  // No counter reset needed — Verilator even/odd alternation handles it
  if (ownerReadyFlags) {
    emitBodyLock(1, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    emitBodyLock(1, "if (mtConfiguredWorkerCount == kDenseOwnerReadyWorkerCount && mtConfiguredWorkerCount > 1 && mtWorkerPoolEnabled && mtWorkerPoolThreadCount + 1 >= mtConfiguredWorkerCount) {\n");
    emitBodyLock(1, "#else\n");
    emitBodyLock(1, "if (mtConfiguredWorkerCount > 1 && mtWorkerPoolEnabled && mtWorkerPoolThreadCount + 1 >= mtConfiguredWorkerCount) {\n");
    emitBodyLock(1, "#endif\n");
  } else {
    // Fixed-owner counter path: ownership was baked in at generation time, so a
    // runtime worker-count mismatch would silently omit whole worker lanes. Require
    // the generated count; otherwise the caller falls through to the serial path.
    emitBodyLock(1, "if (mtConfiguredWorkerCount == %d && mtWorkerPoolEnabled && mtWorkerPoolThreadCount + 1 >= mtConfiguredWorkerCount) {\n", threadCount);
  }
  emitBodyLock(2, "mtWorkerPoolJobKind = 7;\n");
  emitBodyLock(2, "mtWorkerPoolCurrentWorkerCount = mtConfiguredWorkerCount;\n");
  if (ownerReadyFlags) {
    emitBodyLock(2, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    emitBodyLock(2, "if (!mtDenseOwnerReadyTokensPrimed) {\n");
    emitBodyLock(3, "const uint8_t mtDenseOwnerReadyRephaseValue = (cycles & 1) == 0 ? uint8_t{0} : uint8_t{1};\n");
    emitBodyLock(3, "for (int i = 0; i < kDenseOwnerReadyTokenCount; i++)\n");
    emitBodyLock(4, "mtDenseOwnerReadyTokens[kDenseOwnerReadyStoreList[i]].ready.store(mtDenseOwnerReadyRephaseValue, std::memory_order_relaxed);\n");
    emitBodyLock(3, "mtDenseOwnerReadyTokensPrimed = true;\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(2, "#endif\n");
  }
  if (denseBreakdownProfileCodegen) {
    emitBodyLock(2, "std::chrono::steady_clock::time_point mtDenseBreakdownPoolIdleBegin;\n");
    emitBodyLock(2, "std::chrono::steady_clock::time_point mtDenseBreakdownPoolDoneBegin;\n");
  }

  emitBodyLock(2, "mtWorkerPoolPost();\n");
  if (denseBreakdownProfileCodegen) {
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownProfileInCycle)) mtDenseBreakdownPoolIdleBegin = std::chrono::steady_clock::now();\n");
    } else {
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownProfileEnabled)) mtDenseBreakdownPoolIdleBegin = std::chrono::steady_clock::now();\n");
    }
  }
  emitBodyLock(2, "stepDenseThreadWorker(0);\n");
  if (denseBreakdownProfileCodegen) {
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownProfileInCycle)) {\n");
    } else {
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownProfileEnabled)) {\n");
    }
    emitBodyLock(3, "mtDenseBreakdownPoolDoneBegin = std::chrono::steady_clock::now();\n");
    emitBodyLock(3, "uint64_t mtDenseBreakdownPoolIdleNsThisStep = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownPoolDoneBegin - mtDenseBreakdownPoolIdleBegin).count();\n");
    emitBodyLock(3, "if (unlikely(UINT64_MAX - mtDenseBreakdownPoolIdleNs < mtDenseBreakdownPoolIdleNsThisStep)) { fprintf(stderr, \"[mt-dense-breakdown] pool-idle counter overflow\\n\"); abort(); }\n");
    emitBodyLock(3, "mtDenseBreakdownPoolIdleNs += mtDenseBreakdownPoolIdleNsThisStep;\n");
    emitBodyLock(2, "}\n");
  }

  if (denseDuty) emitBodyLock(2, "std::chrono::steady_clock::time_point mtDutyJoinBegin; if (mtDutyEnabled) mtDutyJoinBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(2, "mtWorkerPoolWaitForDone(mtConfiguredWorkerCount - 1);\n");
  if (denseDuty) emitBodyLock(2, "if (mtDutyEnabled) mtDutyLanes[%d].joinNs += (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDutyJoinBegin).count();\n", threadCount);
  if (denseBreakdownWindowCodegen) {
    emitBodyLock(2, "if (unlikely(mtDenseBreakdownProfileInCycle && mtDenseBreakdownWindowCausalChainMode)) {\n");
    emitBodyLock(3, "const int mtDenseBreakdownWindowCausalSlot = mtDenseBreakdownWindowCurrentSlot;\n");
    emitBodyLock(3, "if (unlikely(mtDenseBreakdownWindowCausalSlot < 0 || mtDenseBreakdownWindowCausalSlot >= kDenseBreakdownWindowMaxCycles)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal invalid post-join slot\\n\"); abort(); }\n");
    emitBodyLock(3, "MtDenseBreakdownWindowCausalSummary &mtDenseBreakdownWindowCausalSummary = mtDenseBreakdownWindowCausalSummaries[mtDenseBreakdownWindowCausalSlot];\n");
    emitBodyLock(3, "mtDenseBreakdownWindowCausalSummary.causalBoundNs = 0; mtDenseBreakdownWindowCausalSummary.maxLagNs = 0; mtDenseBreakdownWindowCausalSummary.timestampBoundUncertaintyNs = 0; mtDenseBreakdownWindowCausalSummary.latestOwnerFinishNs = 0; mtDenseBreakdownWindowCausalSummary.makespanNs = 0; mtDenseBreakdownWindowCausalSummary.chainBodyNs = 0; mtDenseBreakdownWindowCausalSummary.chainReleaseNs = 0; mtDenseBreakdownWindowCausalSummary.chainGapNs = 0; mtDenseBreakdownWindowCausalSummary.sameOwnerPredecessorCount = 0; mtDenseBreakdownWindowCausalSummary.remoteTokenPredecessorCount = 0; mtDenseBreakdownWindowCausalSummary.waitObservationCount = 0; mtDenseBreakdownWindowCausalSummary.chainNodeCount = 0; mtDenseBreakdownWindowCausalSummary.chainEdgeCount = 0; mtDenseBreakdownWindowCausalSummary.latestOwner = -1; mtDenseBreakdownWindowCausalSummary.complete = false; mtDenseBreakdownWindowCausalSummary.incompleteMapping = false; mtDenseBreakdownWindowCausalSummary.clockRegression = false; mtDenseBreakdownWindowCausalSummary.overflow = false;\n");
    emitBodyLock(3, "for (int mtaskId = 0; mtaskId < kDenseBreakdownWindowAllOwnerMTaskCount; mtaskId ++) { mtDenseBreakdownWindowCausalRemotePredecessorEnds[mtaskId] = UINT64_MAX; mtDenseBreakdownWindowCausalRemotePredecessorTokens[mtaskId] = -1; mtDenseBreakdownWindowCausalSelectedPredecessors[mtaskId] = -1; mtDenseBreakdownWindowCausalSelectedPredecessorEnds[mtaskId] = 0; }\n");
    emitBodyLock(3, "for (int token = 0; token < kDenseBreakdownWindowCausalTokenCount; token ++) {\n");
    emitBodyLock(4, "const int mtDenseBreakdownWindowReadySlot = kDenseBreakdownWindowCausalTokenReadySlot[token]; const int mtDenseBreakdownWindowProducerMTask = kDenseBreakdownWindowCausalTokenProducerMTask[token]; const int mtDenseBreakdownWindowConsumerMTask = kDenseBreakdownWindowCausalTokenConsumerMTask[token]; const int mtDenseBreakdownWindowProducerOwner = kDenseBreakdownWindowCausalTokenProducerOwner[token]; const int mtDenseBreakdownWindowConsumerOwner = kDenseBreakdownWindowCausalTokenConsumerOwner[token];\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowReadySlot < 0 || mtDenseBreakdownWindowReadySlot >= kDenseOwnerReadyPhysicalSlotCount || kDenseBreakdownWindowCausalLogicalTokenByReadySlot[mtDenseBreakdownWindowReadySlot] != token || mtDenseBreakdownWindowProducerMTask < 0 || mtDenseBreakdownWindowProducerMTask >= kDenseBreakdownWindowAllOwnerMTaskCount || mtDenseBreakdownWindowConsumerMTask < 0 || mtDenseBreakdownWindowConsumerMTask >= kDenseBreakdownWindowAllOwnerMTaskCount || mtDenseBreakdownWindowProducerOwner < 0 || mtDenseBreakdownWindowProducerOwner >= mtConfiguredWorkerCount || mtDenseBreakdownWindowConsumerOwner < 0 || mtDenseBreakdownWindowConsumerOwner >= mtConfiguredWorkerCount || mtDenseBreakdownWindowProducerOwner == mtDenseBreakdownWindowConsumerOwner)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal static provenance incomplete\\n\"); abort(); }\n");
    emitBodyLock(4, "const MtDenseBreakdownWindowReadyToken &mtDenseBreakdownWindowTokenRelease = mtDenseBreakdownWindowReadyTokens[mtDenseBreakdownWindowCausalSlot][token];\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs == UINT64_MAX || mtDenseBreakdownWindowTokenRelease.releaseAfterOffsetNs == UINT64_MAX)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token release missing\\n\"); abort(); }\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowTokenRelease.releaseAfterOffsetNs < mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs)) { mtDenseBreakdownWindowCausalSummary.clockRegression = true; mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token release clock regression\\n\"); abort(); }\n");
    emitBodyLock(4, "const uint64_t mtDenseBreakdownWindowTokenUncertainty = mtDenseBreakdownWindowTokenRelease.releaseAfterOffsetNs - mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs; if (mtDenseBreakdownWindowTokenUncertainty > mtDenseBreakdownWindowCausalSummary.timestampBoundUncertaintyNs) mtDenseBreakdownWindowCausalSummary.timestampBoundUncertaintyNs = mtDenseBreakdownWindowTokenUncertainty;\n");
    emitBodyLock(4, "MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowProducer = mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowCausalSlot][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[mtDenseBreakdownWindowProducerMTask]];\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowProducer.mtaskId != (uint32_t)mtDenseBreakdownWindowProducerMTask || mtDenseBreakdownWindowProducer.ownerThreadId != (uint16_t)mtDenseBreakdownWindowProducerOwner || mtDenseBreakdownWindowProducer.bodyEndOffsetNs < mtDenseBreakdownWindowProducer.bodyStartOffsetNs || mtDenseBreakdownWindowProducer.releaseEndOffsetNs == UINT64_MAX || mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs < mtDenseBreakdownWindowProducer.bodyEndOffsetNs || mtDenseBreakdownWindowTokenRelease.releaseAfterOffsetNs > mtDenseBreakdownWindowProducer.releaseEndOffsetNs)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal producer record mismatch\\n\"); abort(); }\n");
    emitBodyLock(4, "uint64_t &mtDenseBreakdownWindowRemoteEnd = mtDenseBreakdownWindowCausalRemotePredecessorEnds[mtDenseBreakdownWindowConsumerMTask]; int &mtDenseBreakdownWindowRemoteToken = mtDenseBreakdownWindowCausalRemotePredecessorTokens[mtDenseBreakdownWindowConsumerMTask];\n");
    emitBodyLock(4, "if (mtDenseBreakdownWindowRemoteEnd == UINT64_MAX || mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs > mtDenseBreakdownWindowRemoteEnd || (mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs == mtDenseBreakdownWindowRemoteEnd && (mtDenseBreakdownWindowRemoteToken < 0 || token < mtDenseBreakdownWindowRemoteToken))) { mtDenseBreakdownWindowRemoteEnd = mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs; mtDenseBreakdownWindowRemoteToken = token; }\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowCausalSummary.remoteTokenPredecessorCount == UINT32_MAX)) { mtDenseBreakdownWindowCausalSummary.overflow = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal remote predecessor count overflow\\n\"); abort(); } mtDenseBreakdownWindowCausalSummary.remoteTokenPredecessorCount ++;\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "for (int worker = 0; worker < mtConfiguredWorkerCount; worker ++) {\n");
    emitBodyLock(4, "const uint32_t mtDenseBreakdownWindowWaitCount = mtDenseBreakdownWindowWaitCounts[mtDenseBreakdownWindowCausalSlot][worker];\n");
    emitBodyLock(4, "const uint32_t mtDenseBreakdownWindowWaitCapacity = (uint32_t)(kDenseBreakdownWindowWaitLaneOffsets[worker + 1] - kDenseBreakdownWindowWaitLaneOffsets[worker]);\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowWaitCount != mtDenseBreakdownWindowWaitCapacity)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal wait observation count mismatch\\n\"); abort(); }\n");
    emitBodyLock(4, "for (uint32_t w = 0; w < mtDenseBreakdownWindowWaitCount; w ++) {\n");
    emitBodyLock(5, "const MtDenseBreakdownWindowWait &mtDenseBreakdownWindowWait = mtDenseBreakdownWindowWaits[mtDenseBreakdownWindowCausalSlot][kDenseBreakdownWindowWaitLaneOffsets[worker] + w];\n");
    emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowWait.cycleSlot != (uint16_t)mtDenseBreakdownWindowCausalSlot || mtDenseBreakdownWindowWait.threadId != (uint16_t)worker || mtDenseBreakdownWindowWait.readySlot >= (uint32_t)kDenseOwnerReadyPhysicalSlotCount)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal wait record malformed\\n\"); abort(); }\n");
    emitBodyLock(5, "const int mtDenseBreakdownWindowLogicalToken = kDenseBreakdownWindowCausalLogicalTokenByReadySlot[mtDenseBreakdownWindowWait.readySlot];\n");
    emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowLogicalToken < 0 || mtDenseBreakdownWindowLogicalToken >= kDenseBreakdownWindowCausalTokenCount || kDenseBreakdownWindowCausalTokenConsumerMTask[mtDenseBreakdownWindowLogicalToken] != (int)mtDenseBreakdownWindowWait.consumerMtaskId || kDenseBreakdownWindowCausalTokenConsumerOwner[mtDenseBreakdownWindowLogicalToken] != worker)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal wait provenance mismatch\\n\"); abort(); }\n");
    emitBodyLock(5, "const MtDenseBreakdownWindowReadyToken &mtDenseBreakdownWindowTokenRelease = mtDenseBreakdownWindowReadyTokens[mtDenseBreakdownWindowCausalSlot][mtDenseBreakdownWindowLogicalToken];\n");
    emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowWait.endOffsetNs < mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs || mtDenseBreakdownWindowCausalSummary.waitObservationCount == UINT32_MAX)) { mtDenseBreakdownWindowCausalSummary.clockRegression = true; mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal wait timestamp regression\\n\"); abort(); }\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalSummary.waitObservationCount ++;\n");
    emitBodyLock(4, "}\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "int mtDenseBreakdownWindowLastMTaskByOwner[kDenseBreakdownWindowThreadCount];\n");
    emitBodyLock(3, "for (int worker = 0; worker < kDenseBreakdownWindowThreadCount; worker ++) mtDenseBreakdownWindowLastMTaskByOwner[worker] = -1;\n");
    emitBodyLock(3, "for (int mtaskId = 0; mtaskId < kDenseBreakdownWindowAllOwnerMTaskCount; mtaskId ++) {\n");
    emitBodyLock(4, "const MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowMTask = mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowCausalSlot][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[mtaskId]];\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowMTask.mtaskId != (uint32_t)mtaskId || mtDenseBreakdownWindowMTask.ownerThreadId >= (uint16_t)mtConfiguredWorkerCount || mtDenseBreakdownWindowMTask.bodyEndOffsetNs < mtDenseBreakdownWindowMTask.bodyStartOffsetNs || mtDenseBreakdownWindowMTask.releaseEndOffsetNs == UINT64_MAX || mtDenseBreakdownWindowMTask.releaseEndOffsetNs < mtDenseBreakdownWindowMTask.bodyEndOffsetNs)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal MTask record incomplete\\n\"); abort(); }\n");
    emitBodyLock(4, "const int mtDenseBreakdownWindowOwner = (int)mtDenseBreakdownWindowMTask.ownerThreadId; int mtDenseBreakdownWindowSelectedPredecessor = -1; uint64_t mtDenseBreakdownWindowSelectedEnd = 0;\n");
    emitBodyLock(4, "const int mtDenseBreakdownWindowPreviousMTask = mtDenseBreakdownWindowLastMTaskByOwner[mtDenseBreakdownWindowOwner];\n");
    emitBodyLock(4, "if (mtDenseBreakdownWindowPreviousMTask >= 0) { const MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowPrevious = mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowCausalSlot][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[mtDenseBreakdownWindowPreviousMTask]]; if (unlikely(mtDenseBreakdownWindowPrevious.releaseEndOffsetNs == UINT64_MAX || mtDenseBreakdownWindowPrevious.ownerThreadId != (uint16_t)mtDenseBreakdownWindowOwner || mtDenseBreakdownWindowCausalSummary.sameOwnerPredecessorCount == UINT32_MAX)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal same-owner predecessor missing\\n\"); abort(); } mtDenseBreakdownWindowSelectedPredecessor = mtDenseBreakdownWindowPreviousMTask; mtDenseBreakdownWindowSelectedEnd = mtDenseBreakdownWindowPrevious.releaseEndOffsetNs; mtDenseBreakdownWindowCausalSummary.sameOwnerPredecessorCount ++; }\n");
    emitBodyLock(4, "const int mtDenseBreakdownWindowRemoteToken = mtDenseBreakdownWindowCausalRemotePredecessorTokens[mtaskId];\n");
    emitBodyLock(4, "if (mtDenseBreakdownWindowRemoteToken >= 0) { const int mtDenseBreakdownWindowRemoteProducer = kDenseBreakdownWindowCausalTokenProducerMTask[mtDenseBreakdownWindowRemoteToken]; const uint64_t mtDenseBreakdownWindowRemoteEnd = mtDenseBreakdownWindowCausalRemotePredecessorEnds[mtaskId]; if (unlikely(mtDenseBreakdownWindowRemoteEnd == UINT64_MAX || mtDenseBreakdownWindowRemoteProducer < 0 || mtDenseBreakdownWindowRemoteProducer >= mtaskId)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal remote predecessor invalid\\n\"); abort(); } if (mtDenseBreakdownWindowSelectedPredecessor < 0 || mtDenseBreakdownWindowRemoteEnd > mtDenseBreakdownWindowSelectedEnd || (mtDenseBreakdownWindowRemoteEnd == mtDenseBreakdownWindowSelectedEnd && mtDenseBreakdownWindowRemoteProducer < mtDenseBreakdownWindowSelectedPredecessor)) { mtDenseBreakdownWindowSelectedPredecessor = mtDenseBreakdownWindowRemoteProducer; mtDenseBreakdownWindowSelectedEnd = mtDenseBreakdownWindowRemoteEnd; } }\n");
    emitBodyLock(4, "if (mtDenseBreakdownWindowSelectedPredecessor >= 0) { if (unlikely(mtDenseBreakdownWindowMTask.bodyStartOffsetNs < mtDenseBreakdownWindowSelectedEnd)) { mtDenseBreakdownWindowCausalSummary.clockRegression = true; mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal predecessor timestamp regression\\n\"); abort(); } const uint64_t mtDenseBreakdownWindowLag = mtDenseBreakdownWindowMTask.bodyStartOffsetNs - mtDenseBreakdownWindowSelectedEnd; if (mtDenseBreakdownWindowLag > mtDenseBreakdownWindowCausalSummary.maxLagNs) mtDenseBreakdownWindowCausalSummary.maxLagNs = mtDenseBreakdownWindowLag; if (mtDenseBreakdownWindowSelectedEnd > mtDenseBreakdownWindowCausalSummary.causalBoundNs) mtDenseBreakdownWindowCausalSummary.causalBoundNs = mtDenseBreakdownWindowSelectedEnd; mtDenseBreakdownWindowCausalSelectedPredecessors[mtaskId] = mtDenseBreakdownWindowSelectedPredecessor; mtDenseBreakdownWindowCausalSelectedPredecessorEnds[mtaskId] = mtDenseBreakdownWindowSelectedEnd; }\n");
    emitBodyLock(4, "mtDenseBreakdownWindowLastMTaskByOwner[mtDenseBreakdownWindowOwner] = mtaskId;\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "int mtDenseBreakdownWindowLatestOwner = -1; uint64_t mtDenseBreakdownWindowLatestFinish = 0;\n");
    emitBodyLock(3, "for (int worker = 0; worker < mtConfiguredWorkerCount; worker ++) { if (mtDenseBreakdownWindowLastMTaskByOwner[worker] < 0) continue; const MtDenseBreakdownWindowWorker &mtDenseBreakdownWindowWorker = mtDenseBreakdownWindowWorkers[mtDenseBreakdownWindowCausalSlot][worker]; if (unlikely(mtDenseBreakdownWindowWorker.finishOffsetNs < mtDenseBreakdownWindowWorker.startOffsetNs)) { mtDenseBreakdownWindowCausalSummary.clockRegression = true; mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal worker finish regression\\n\"); abort(); } if (mtDenseBreakdownWindowLatestOwner < 0 || mtDenseBreakdownWindowWorker.finishOffsetNs > mtDenseBreakdownWindowLatestFinish || (mtDenseBreakdownWindowWorker.finishOffsetNs == mtDenseBreakdownWindowLatestFinish && worker < mtDenseBreakdownWindowLatestOwner)) { mtDenseBreakdownWindowLatestOwner = worker; mtDenseBreakdownWindowLatestFinish = mtDenseBreakdownWindowWorker.finishOffsetNs; } }\n");
    emitBodyLock(3, "if (unlikely(mtDenseBreakdownWindowLatestOwner < 0)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal latest owner missing\\n\"); abort(); }\n");
    emitBodyLock(3, "mtDenseBreakdownWindowCausalSummary.latestOwner = mtDenseBreakdownWindowLatestOwner; mtDenseBreakdownWindowCausalSummary.latestOwnerFinishNs = mtDenseBreakdownWindowLatestFinish; mtDenseBreakdownWindowCausalSummary.makespanNs = mtDenseBreakdownWindowLatestFinish;\n");
    emitBodyLock(3, "const uint32_t mtDenseBreakdownWindowVisitMark = (uint32_t)mtDenseBreakdownWindowCausalSlot + 1; int mtDenseBreakdownWindowChainMTask = mtDenseBreakdownWindowLastMTaskByOwner[mtDenseBreakdownWindowLatestOwner]; uint64_t mtDenseBreakdownWindowChainCausalReleaseEnd = UINT64_MAX;\n");
    emitBodyLock(3, "while (mtDenseBreakdownWindowChainMTask >= 0) { if (unlikely(mtDenseBreakdownWindowChainMTask >= kDenseBreakdownWindowAllOwnerMTaskCount || mtDenseBreakdownWindowCausalSummary.chainNodeCount >= (uint32_t)kDenseBreakdownWindowAllOwnerMTaskCount || mtDenseBreakdownWindowCausalVisitMarks[mtDenseBreakdownWindowChainMTask] == mtDenseBreakdownWindowVisitMark)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal backtrack cycle or bound failure\\n\"); abort(); } mtDenseBreakdownWindowCausalVisitMarks[mtDenseBreakdownWindowChainMTask] = mtDenseBreakdownWindowVisitMark; const MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowChainNode = mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowCausalSlot][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[mtDenseBreakdownWindowChainMTask]]; const uint64_t mtDenseBreakdownWindowChainReleaseEnd = mtDenseBreakdownWindowChainCausalReleaseEnd == UINT64_MAX ? mtDenseBreakdownWindowChainNode.releaseEndOffsetNs : mtDenseBreakdownWindowChainCausalReleaseEnd; if (unlikely(mtDenseBreakdownWindowChainReleaseEnd < mtDenseBreakdownWindowChainNode.bodyEndOffsetNs)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal chain release boundary invalid\\n\"); abort(); } const uint64_t mtDenseBreakdownWindowChainBodyNs = mtDenseBreakdownWindowChainNode.bodyEndOffsetNs - mtDenseBreakdownWindowChainNode.bodyStartOffsetNs; const uint64_t mtDenseBreakdownWindowReleaseNs = mtDenseBreakdownWindowChainReleaseEnd - mtDenseBreakdownWindowChainNode.bodyEndOffsetNs; if (unlikely(UINT64_MAX - mtDenseBreakdownWindowCausalSummary.chainBodyNs < mtDenseBreakdownWindowChainBodyNs || UINT64_MAX - mtDenseBreakdownWindowCausalSummary.chainReleaseNs < mtDenseBreakdownWindowReleaseNs)) { mtDenseBreakdownWindowCausalSummary.overflow = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal chain sum overflow\\n\"); abort(); } mtDenseBreakdownWindowCausalSummary.chainBodyNs += mtDenseBreakdownWindowChainBodyNs; mtDenseBreakdownWindowCausalSummary.chainReleaseNs += mtDenseBreakdownWindowReleaseNs; mtDenseBreakdownWindowCausalSummary.chainNodeCount ++; const int mtDenseBreakdownWindowChainPredecessor = mtDenseBreakdownWindowCausalSelectedPredecessors[mtDenseBreakdownWindowChainMTask]; if (mtDenseBreakdownWindowChainPredecessor < 0) break; const uint64_t mtDenseBreakdownWindowChainPredecessorEnd = mtDenseBreakdownWindowCausalSelectedPredecessorEnds[mtDenseBreakdownWindowChainMTask]; if (unlikely(mtDenseBreakdownWindowChainNode.bodyStartOffsetNs < mtDenseBreakdownWindowChainPredecessorEnd || mtDenseBreakdownWindowCausalSummary.chainEdgeCount == UINT32_MAX || UINT64_MAX - mtDenseBreakdownWindowCausalSummary.chainGapNs < mtDenseBreakdownWindowChainNode.bodyStartOffsetNs - mtDenseBreakdownWindowChainPredecessorEnd)) { mtDenseBreakdownWindowCausalSummary.clockRegression = true; mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal chain gap invalid\\n\"); abort(); } mtDenseBreakdownWindowCausalSummary.chainGapNs += mtDenseBreakdownWindowChainNode.bodyStartOffsetNs - mtDenseBreakdownWindowChainPredecessorEnd; mtDenseBreakdownWindowCausalSummary.chainEdgeCount ++; mtDenseBreakdownWindowChainCausalReleaseEnd = mtDenseBreakdownWindowChainPredecessorEnd; mtDenseBreakdownWindowChainMTask = mtDenseBreakdownWindowChainPredecessor; }\n");
    emitBodyLock(3, "if (mtDenseBreakdownWindowCausalHotspotsMode) {\n");
    emitBodyLock(4, "MtDenseBreakdownWindowCausalHotspotTotals mtDenseBreakdownWindowCausalHotspotCycleTotals = {};\n");
    emitBodyLock(4, "auto mtDenseBreakdownWindowCausalHotspotsAdd = [&](uint64_t &target, uint64_t value) { if (unlikely(UINT64_MAX - target < value)) { mtDenseBreakdownWindowCausalSummary.overflow = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots aggregate overflow\\n\"); abort(); } target += value; };\n");
    emitBodyLock(4, "int mtDenseBreakdownWindowCausalHotspotMTaskId = mtDenseBreakdownWindowLastMTaskByOwner[mtDenseBreakdownWindowLatestOwner];\n");
    emitBodyLock(4, "uint64_t mtDenseBreakdownWindowCausalHotspotReleaseEnd = UINT64_MAX;\n");
    emitBodyLock(4, "uint32_t mtDenseBreakdownWindowCausalHotspotNodes = 0;\n");
    emitBodyLock(4, "while (mtDenseBreakdownWindowCausalHotspotMTaskId >= 0) {\n");
    emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowCausalHotspotMTaskId >= kDenseBreakdownWindowAllOwnerMTaskCount || mtDenseBreakdownWindowCausalHotspotNodes >= (uint32_t)kDenseBreakdownWindowAllOwnerMTaskCount)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots logical MTask mapping invalid\\n\"); abort(); }\n");
    emitBodyLock(5, "const MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowCausalHotspotNode = mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowCausalSlot][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[mtDenseBreakdownWindowCausalHotspotMTaskId]];\n");
    emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowCausalHotspotNode.mtaskId != (uint32_t)mtDenseBreakdownWindowCausalHotspotMTaskId || mtDenseBreakdownWindowCausalHotspotNode.bodyEndOffsetNs < mtDenseBreakdownWindowCausalHotspotNode.bodyStartOffsetNs || mtDenseBreakdownWindowCausalHotspotNode.releaseEndOffsetNs == UINT64_MAX)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots MTask record invalid\\n\"); abort(); }\n");
    emitBodyLock(5, "const uint64_t mtDenseBreakdownWindowCausalHotspotNodeReleaseEnd = mtDenseBreakdownWindowCausalHotspotReleaseEnd == UINT64_MAX ? mtDenseBreakdownWindowCausalHotspotNode.releaseEndOffsetNs : mtDenseBreakdownWindowCausalHotspotReleaseEnd;\n");
    emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowCausalHotspotNodeReleaseEnd < mtDenseBreakdownWindowCausalHotspotNode.bodyEndOffsetNs)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots release boundary invalid\\n\"); abort(); }\n");
    emitBodyLock(5, "const uint64_t mtDenseBreakdownWindowCausalHotspotBodyNs = mtDenseBreakdownWindowCausalHotspotNode.bodyEndOffsetNs - mtDenseBreakdownWindowCausalHotspotNode.bodyStartOffsetNs;\n");
    emitBodyLock(5, "const uint64_t mtDenseBreakdownWindowCausalHotspotReleaseNs = mtDenseBreakdownWindowCausalHotspotNodeReleaseEnd - mtDenseBreakdownWindowCausalHotspotNode.bodyEndOffsetNs;\n");
    emitBodyLock(5, "MtDenseBreakdownWindowCausalHotspotMTask &mtDenseBreakdownWindowCausalHotspotMTask = mtDenseBreakdownWindowCausalHotspotMTasks[mtDenseBreakdownWindowCausalHotspotMTaskId];\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotMTask.count, 1);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotMTask.bodyNs, mtDenseBreakdownWindowCausalHotspotBodyNs);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotMTask.releaseNs, mtDenseBreakdownWindowCausalHotspotReleaseNs);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotTotals.count, 1);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotTotals.bodyNs, mtDenseBreakdownWindowCausalHotspotBodyNs);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotTotals.releaseNs, mtDenseBreakdownWindowCausalHotspotReleaseNs);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotCycleTotals.count, 1);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotCycleTotals.bodyNs, mtDenseBreakdownWindowCausalHotspotBodyNs);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotCycleTotals.releaseNs, mtDenseBreakdownWindowCausalHotspotReleaseNs);\n");
    emitBodyLock(5, "if (mtDenseBreakdownWindowCausalHotspotNodes == 0) { mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotMTask.latestTailCount, 1); mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotTotals.latestTailCount, 1); mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotCycleTotals.latestTailCount, 1); }\n");
    emitBodyLock(5, "const int mtDenseBreakdownWindowCausalHotspotPredecessor = mtDenseBreakdownWindowCausalSelectedPredecessors[mtDenseBreakdownWindowCausalHotspotMTaskId];\n");
    emitBodyLock(5, "if (mtDenseBreakdownWindowCausalHotspotPredecessor < 0) break;\n");
    emitBodyLock(5, "const uint64_t mtDenseBreakdownWindowCausalHotspotPredecessorEnd = mtDenseBreakdownWindowCausalSelectedPredecessorEnds[mtDenseBreakdownWindowCausalHotspotMTaskId];\n");
    emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowCausalHotspotPredecessor >= mtDenseBreakdownWindowCausalHotspotMTaskId || mtDenseBreakdownWindowCausalHotspotNode.bodyStartOffsetNs < mtDenseBreakdownWindowCausalHotspotPredecessorEnd)) { mtDenseBreakdownWindowCausalSummary.clockRegression = true; mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots predecessor mapping invalid\\n\"); abort(); }\n");
    emitBodyLock(5, "const uint64_t mtDenseBreakdownWindowCausalHotspotGapNs = mtDenseBreakdownWindowCausalHotspotNode.bodyStartOffsetNs - mtDenseBreakdownWindowCausalHotspotPredecessorEnd;\n");
    emitBodyLock(5, "const int mtDenseBreakdownWindowCausalHotspotRemoteToken = mtDenseBreakdownWindowCausalRemotePredecessorTokens[mtDenseBreakdownWindowCausalHotspotMTaskId];\n");
    emitBodyLock(5, "bool mtDenseBreakdownWindowCausalHotspotSelectedRemoteToken = false;\n");
    emitBodyLock(5, "if (mtDenseBreakdownWindowCausalHotspotRemoteToken >= 0) { if (unlikely(mtDenseBreakdownWindowCausalHotspotRemoteToken >= kDenseBreakdownWindowCausalTokenCount)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots remote token index invalid\\n\"); abort(); } mtDenseBreakdownWindowCausalHotspotSelectedRemoteToken = kDenseBreakdownWindowCausalTokenProducerMTask[mtDenseBreakdownWindowCausalHotspotRemoteToken] == mtDenseBreakdownWindowCausalHotspotPredecessor && mtDenseBreakdownWindowCausalRemotePredecessorEnds[mtDenseBreakdownWindowCausalHotspotMTaskId] == mtDenseBreakdownWindowCausalHotspotPredecessorEnd; }\n");
    emitBodyLock(5, "if (mtDenseBreakdownWindowCausalHotspotSelectedRemoteToken && kDenseBreakdownWindowCausalSameOwnerProducerMTask[mtDenseBreakdownWindowCausalHotspotMTaskId] == mtDenseBreakdownWindowCausalHotspotPredecessor) mtDenseBreakdownWindowCausalHotspotSelectedRemoteToken = false;\n");
    emitBodyLock(5, "if (mtDenseBreakdownWindowCausalHotspotSelectedRemoteToken) {\n");
    emitBodyLock(6, "const int mtDenseBreakdownWindowCausalHotspotReadySlot = kDenseBreakdownWindowCausalTokenReadySlot[mtDenseBreakdownWindowCausalHotspotRemoteToken];\n");
    emitBodyLock(6, "if (unlikely(mtDenseBreakdownWindowCausalHotspotReadySlot < 0 || mtDenseBreakdownWindowCausalHotspotReadySlot >= kDenseOwnerReadyPhysicalSlotCount || kDenseBreakdownWindowCausalLogicalTokenByReadySlot[mtDenseBreakdownWindowCausalHotspotReadySlot] != mtDenseBreakdownWindowCausalHotspotRemoteToken || kDenseBreakdownWindowCausalTokenConsumerMTask[mtDenseBreakdownWindowCausalHotspotRemoteToken] != mtDenseBreakdownWindowCausalHotspotMTaskId)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots remote token provenance invalid\\n\"); abort(); }\n");
    emitBodyLock(6, "if (unlikely(mtDenseBreakdownWindowReadyTokens[mtDenseBreakdownWindowCausalSlot][mtDenseBreakdownWindowCausalHotspotRemoteToken].releaseBeforeOffsetNs != mtDenseBreakdownWindowCausalHotspotPredecessorEnd)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots remote release-before endpoint mismatch\\n\"); abort(); }\n");
    emitBodyLock(6, "MtDenseBreakdownWindowCausalHotspotEdge &mtDenseBreakdownWindowCausalHotspotEdge = mtDenseBreakdownWindowCausalHotspotRemoteTokenEdges[mtDenseBreakdownWindowCausalHotspotRemoteToken];\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotMTask.remoteTokenPredCount, 1);\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotTotals.remoteTokenPredCount, 1);\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotCycleTotals.remoteTokenPredCount, 1);\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotEdge.count, 1);\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotEdge.gapNs, mtDenseBreakdownWindowCausalHotspotGapNs);\n");
    emitBodyLock(5, "} else {\n");
    emitBodyLock(6, "const int mtDenseBreakdownWindowCausalHotspotSameOwnerProducer = kDenseBreakdownWindowCausalSameOwnerProducerMTask[mtDenseBreakdownWindowCausalHotspotMTaskId];\n");
    emitBodyLock(6, "const MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowCausalHotspotProducer = mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowCausalSlot][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[mtDenseBreakdownWindowCausalHotspotPredecessor]];\n");
    emitBodyLock(6, "if (unlikely(mtDenseBreakdownWindowCausalHotspotSameOwnerProducer != mtDenseBreakdownWindowCausalHotspotPredecessor || mtDenseBreakdownWindowCausalHotspotProducer.mtaskId != (uint32_t)mtDenseBreakdownWindowCausalHotspotPredecessor || mtDenseBreakdownWindowCausalHotspotProducer.ownerThreadId != mtDenseBreakdownWindowCausalHotspotNode.ownerThreadId)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots same-owner mapping invalid\\n\"); abort(); }\n");
    emitBodyLock(6, "MtDenseBreakdownWindowCausalHotspotEdge &mtDenseBreakdownWindowCausalHotspotEdge = mtDenseBreakdownWindowCausalHotspotSameOwnerEdges[mtDenseBreakdownWindowCausalHotspotMTaskId];\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotMTask.sameOwnerPredCount, 1);\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotTotals.sameOwnerPredCount, 1);\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotCycleTotals.sameOwnerPredCount, 1);\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotEdge.count, 1);\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotEdge.gapNs, mtDenseBreakdownWindowCausalHotspotGapNs);\n");
    emitBodyLock(5, "}\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotMTask.gapNs, mtDenseBreakdownWindowCausalHotspotGapNs);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotTotals.gapNs, mtDenseBreakdownWindowCausalHotspotGapNs);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotTotals.edgeCount, 1);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotCycleTotals.gapNs, mtDenseBreakdownWindowCausalHotspotGapNs);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotCycleTotals.edgeCount, 1);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotReleaseEnd = mtDenseBreakdownWindowCausalHotspotPredecessorEnd;\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotMTaskId = mtDenseBreakdownWindowCausalHotspotPredecessor;\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotNodes ++;\n");
    emitBodyLock(4, "}\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowCausalHotspotCycleTotals.count != mtDenseBreakdownWindowCausalSummary.chainNodeCount || mtDenseBreakdownWindowCausalHotspotCycleTotals.bodyNs != mtDenseBreakdownWindowCausalSummary.chainBodyNs || mtDenseBreakdownWindowCausalHotspotCycleTotals.releaseNs != mtDenseBreakdownWindowCausalSummary.chainReleaseNs || mtDenseBreakdownWindowCausalHotspotCycleTotals.gapNs != mtDenseBreakdownWindowCausalSummary.chainGapNs || mtDenseBreakdownWindowCausalHotspotCycleTotals.edgeCount != mtDenseBreakdownWindowCausalSummary.chainEdgeCount || mtDenseBreakdownWindowCausalHotspotCycleTotals.sameOwnerPredCount > mtDenseBreakdownWindowCausalHotspotCycleTotals.edgeCount || mtDenseBreakdownWindowCausalHotspotCycleTotals.remoteTokenPredCount > mtDenseBreakdownWindowCausalHotspotCycleTotals.edgeCount - mtDenseBreakdownWindowCausalHotspotCycleTotals.sameOwnerPredCount || mtDenseBreakdownWindowCausalHotspotCycleTotals.latestTailCount != 1)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots cycle reconciliation failed\\n\"); abort(); }\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowCausalHotspotCycleTotals.sameOwnerPredCount > mtDenseBreakdownWindowCausalHotspotCycleTotals.edgeCount || mtDenseBreakdownWindowCausalHotspotCycleTotals.remoteTokenPredCount != mtDenseBreakdownWindowCausalHotspotCycleTotals.edgeCount - mtDenseBreakdownWindowCausalHotspotCycleTotals.sameOwnerPredCount)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots selected edge-kind reconciliation failed\\n\"); abort(); }\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowCausalHotspotCycleTotals.sameOwnerPredCount > UINT32_MAX || mtDenseBreakdownWindowCausalHotspotCycleTotals.remoteTokenPredCount > UINT32_MAX)) { mtDenseBreakdownWindowCausalSummary.overflow = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots selected predecessor count overflow\\n\"); abort(); }\n");
    emitBodyLock(4, "mtDenseBreakdownWindowCausalSummary.sameOwnerPredecessorCount = (uint32_t)mtDenseBreakdownWindowCausalHotspotCycleTotals.sameOwnerPredCount; mtDenseBreakdownWindowCausalSummary.remoteTokenPredecessorCount = (uint32_t)mtDenseBreakdownWindowCausalHotspotCycleTotals.remoteTokenPredCount;\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "if (mtDenseBreakdownWindowCausalSummary.timestampBoundUncertaintyNs > mtDenseBreakdownWindowCausalTimestampBoundUncertaintyNs) mtDenseBreakdownWindowCausalTimestampBoundUncertaintyNs = mtDenseBreakdownWindowCausalSummary.timestampBoundUncertaintyNs; mtDenseBreakdownWindowCausalSummary.complete = true;\n");
    emitBodyLock(2, "}\n");
  }
  if (denseBreakdownProfileCodegen) {
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownProfileInCycle)) {\n");
    } else {
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownProfileEnabled)) {\n");
    }
    emitBodyLock(3, "uint64_t mtDenseBreakdownPoolDoneNsThisStep = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDenseBreakdownPoolDoneBegin).count();\n");
    emitBodyLock(3, "if (unlikely(UINT64_MAX - mtDenseBreakdownPoolDoneNs < mtDenseBreakdownPoolDoneNsThisStep)) { fprintf(stderr, \"[mt-dense-breakdown] pool-done counter overflow\\n\"); abort(); }\n");
    emitBodyLock(3, "mtDenseBreakdownPoolDoneNs += mtDenseBreakdownPoolDoneNsThisStep;\n");
    emitBodyLock(2, "}\n");
  }

  emitBodyLock(1, "} else {\n");
  if (ownerReadyFlags) {
    emitBodyLock(2, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    emitBodyLock(2, "mtDenseOwnerReadyTokensPrimed = false;\n");
    emitBodyLock(2, "#endif\n");
  }
  for (int mtaskId = 0; mtaskId < nMTasks; mtaskId++) {
    emitBodyLock(2, "stepDenseMTask%d();\n", mtaskId);
  }
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "if (mtProfileDynamicTraceFile != nullptr) dumpMtProfileDynamicTraceCycle();\n");
  emitBodyLock(1, "cycles ++;\n");
  emitBodyLock(1, "if (unlikely(mtProfileEnabled)) mtProfileTotalStepNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileStepBegin).count();\n");
  if (denseBreakdownProfileCodegen) {
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownProfileInCycle)) {\n");
    } else {
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownProfileEnabled)) {\n");
    }
    emitBodyLock(2, "uint64_t mtDenseBreakdownStepNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDenseBreakdownStepBegin).count();\n");
    emitBodyLock(2, "if (unlikely(UINT64_MAX - mtDenseBreakdownTotalStepNs < mtDenseBreakdownStepNs)) { fprintf(stderr, \"[mt-dense-breakdown] total-step counter overflow\\n\"); abort(); }\n");
    emitBodyLock(2, "mtDenseBreakdownTotalStepNs += mtDenseBreakdownStepNs;\n");
    emitBodyLock(1, "}\n");
  }
  emitBodyLock(0, "}\n");
  mtActivationEventTraceSuppressed = savedActivationEventTraceSuppressed;
}

void graph::genStep(int subStepIdxMax, int serialFastSubStepMax, const std::string& serialFastSuffix, bool denseExecutorValid) {
  emitFuncDecl(0, "void S%s::step() {\n", name.c_str());
  emitBodyLock(1, "std::chrono::steady_clock::time_point mtProfileStepBegin;\n");
  if (denseExecutorValid) emitBodyLock(1, "if (unlikely(mtUseDenseExecutor)) { stepDense(); return; }\n");
  if (mtUseDenseOnlyCodegenLevel2()) {
    // Level 2: the SerialFast subSteps and the sparse serial scan are not
    // compiled in, so the dense executor early-return above is the only live
    // path; everything after it would be dead text before an abort.
    emitBodyLock(1, "fprintf(stderr, \"[gsim] dense-only level-2 model: step() requires the dense executor (run with GSIM_MT_EXECUTOR=dense, or rebuild without GSIM_MT_DENSE_ONLY_CODEGEN=2)\\n\");\n");
    emitBodyLock(1, "abort();\n");
    emitBodyLock(0, "}\n");
    return;
  }
  if (mtUseActivationEventTraceCodegen()) emitBodyLock(1, "beginMtActivationEventTraceCycle();\n");
  emitBodyLock(1, "if (unlikely(mtProfileEnabled)) mtProfileStepBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(1, "resetAll();\n");
  for (SuperNode* super : sortedSuper) {
    for (Node* member : super->member) {
      if (member->isReset() && member->type == NODE_REG_SRC) {
        emitBodyLock(1, "%s = %s;\n", RESET_NAME(member).c_str(), member->name.c_str());
      }
    }
  }
  if (serialFastSubStepMax >= 0) {
    emitBodyLock(1, "if (mtConfiguredWorkerCount <= mtSparseSerialFastMaxWorkers && !mtProfileEnabled) {\n");
    for (int i = 0; i <= serialFastSubStepMax; i ++) {
      emitBodyLock(2, "subStep%d%s();\n", i, serialFastSuffix.c_str());
    }
    if (mtUseActivationEventTraceCodegen()) emitBodyLock(2, "flushMtActivationEventTraceCycle();\n");
    emitBodyLock(2, "cycles ++;\n");
    emitBodyLock(2, "return;\n");
    emitBodyLock(1, "}\n");
  }
  if (mtUseDenseOnlyCodegen()) {
    // Dense-only model: the sparse serial scan is not compiled in. The only
    // live paths are the dense executor (returned earlier) and the serial-fast
    // block above; anything reaching here has no runtime to run on.
    emitBodyLock(1, "fprintf(stderr, \"[gsim] dense-only model: this worker configuration requires the sparse runtime (rebuild without GSIM_MT_DENSE_ONLY_CODEGEN)\\n\");\n");
    emitBodyLock(1, "abort();\n");
  } else {
    bool stepActiveWordGuard = mtUseStepActiveWordGuard();
    for (int i = 0; i <= subStepIdxMax; i ++) {
      bool guardedSubStep = stepActiveWordGuard && i < (int)mtStepActiveWordGuards.size() &&
                            i < (int)mtStepActiveWordGuardable.size() &&
                            mtStepActiveWordGuardable[(size_t)i] &&
                            !mtStepActiveWordGuards[(size_t)i].empty();
      if (guardedSubStep) {
        const std::vector<int>& guards = mtStepActiveWordGuards[(size_t)i];
        std::string guardExpr;
        for (int activeWord : guards) {
          if (!guardExpr.empty()) guardExpr += " | ";
          guardExpr += format("activeFlags[%d]", activeWord);
        }
        emitBodyLock(1, "if (unlikely((%s) != 0)) subStep%d();\n", guardExpr.c_str(), i);
      } else {
        emitBodyLock(1, "subStep%d();\n", i);
      }
    }

    // Dump before cycles++ so the trace line names the cycle whose substeps just ran.
    emitBodyLock(1, "if (mtProfileDynamicTraceFile != nullptr) dumpMtProfileDynamicTraceCycle();\n");
    if (mtUseActivationEventTraceCodegen()) emitBodyLock(1, "flushMtActivationEventTraceCycle();\n");
  }
  emitBodyLock(1, "cycles ++;\n");
  emitBodyLock(1, "if (unlikely(mtProfileEnabled)) mtProfileTotalStepNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileStepBegin).count();\n");
  emitBodyLock(0, "}\n");
}

bool SuperNode::instsEmpty() {
  return insts.size() == 0;
}

// ---- Parallel Final emission: capture/replay machinery ----
// Emission units (per-super mtTask helpers, per-MTask stepDenseMTask bodies)
// render concurrently with __emitSrc in capture mode: every call appends one
// graph::EmitChunk to the unit's graph::EmitBuf instead of writing srcFp.
// After the join, flushEmitBufs() replays all chunks in unit order on the
// main thread and performs the same byte-count-based file rotation __emitSrc
// would have, so the assembled files are byte-identical to a sequential run.
//
// countedBytes mirrors the original accounting quirk: indent spaces are
// written to the file but do NOT count toward srcFileBytes/rotation, because
// the original __emitSrc adds only the vfprintf return value.
static thread_local graph::EmitBuf* emitCaptureTarget = nullptr;

// vsnprintf into a std::string without truncation. `args` is never consumed;
// each vsnprintf call runs on its own va_copy.
static void appendVFormat(std::string& out, const char* fmt, va_list args) {
  char local[1024];
  va_list probe;
  va_copy(probe, args);
  int needed = std::vsnprintf(local, sizeof(local), fmt, probe);
  va_end(probe);
  Assert(needed >= 0, "vsnprintf encoding error");
  if ((size_t)needed < sizeof(local)) {
    out.append(local, (size_t)needed);
    return;
  }
  size_t base = out.size();
  out.resize(base + (size_t)needed + 1);
  va_list work;
  va_copy(work, args);
  std::vsnprintf(&out[base], (size_t)needed + 1, fmt, work);
  va_end(work);
  out.resize(base + (size_t)needed);
}

void graph::rotateSrcFile(bool alreadyEndFunc, const char *nextFuncDef) {
  if (srcFp != NULL) {
    if (!alreadyEndFunc) fprintf(srcFp, "}"); // the end of the current function
    fclose(srcFp);
    commitStableOutputFile(srcTmpFilePath, srcFilePath);
  }
  srcFilePath = format("%s%d.cpp", (globalConfig.OutputDir + "/" + name).c_str(), srcFileIdx);
  srcTmpFilePath = globalConfig.MtStableOutput ? srcFilePath + ".tmp" : "";
  const std::string openPath = globalConfig.MtStableOutput ? srcTmpFilePath : srcFilePath;
  srcFp = std::fopen(openPath.c_str(), "w");
  srcFileIdx ++;
  assert(srcFp != NULL);
  // 4 MiB stdio buffer: generated text is ~12 GB streamed line-by-line;
  // the default 4-8 KB buffer turns into millions of write syscalls.
  setvbuf(srcFp, NULL, _IOFBF, 4 * 1024 * 1024);
  srcFileBytes = fprintf(srcFp, "#include \"%s.h\"\n", name.c_str());
  if (nextFuncDef != NULL) {
    srcFileBytes += fprintf(srcFp, "%s {\n", nextFuncDef);
  }
}

bool graph::__emitSrc(int indent, bool canNewFile, bool alreadyEndFunc, const char *nextFuncDef, const char *fmt, ...) {
  if (emitCaptureTarget != nullptr) {
    EmitChunk chunk;
    chunk.canNewFile = canNewFile;
    chunk.alreadyEndFunc = alreadyEndFunc;
    if (nextFuncDef != nullptr) {
      chunk.hasNextFuncDef = true;
      chunk.nextFuncDef = nextFuncDef;
    }
    if (indent > 0) chunk.text.append((size_t)indent * 2, ' ');
    va_list args;
    va_start(args, fmt);
    appendVFormat(chunk.text, fmt, args);
    va_end(args);
    chunk.countedBytes = chunk.text.size() - (size_t)indent * 2;
    assert(chunk.countedBytes > 0);
    emitCaptureTarget->chunks.push_back(std::move(chunk));
    return false;
  }
  bool newFile = false;
  if (srcFp == NULL || (srcFileBytes > (globalConfig.cppMaxSizeKB * 1024) && canNewFile)) {
    rotateSrcFile(alreadyEndFunc, nextFuncDef);
    newFile = true;
  }
  for (int i = 0; i < indent; i ++) fprintf(srcFp, "  ");
  va_list args;
  va_start(args, fmt);
  int bytes = vfprintf(srcFp, fmt, args);
  assert(bytes > 0);
  va_end(args);
  srcFileBytes += bytes;
  return newFile;
}

// Replays captured units in order; rotation decisions depend only on the
// cumulative byte count, which equals the sequential stream's at every chunk
// boundary, so files and rotation points come out identical.
void graph::flushEmitBufs(std::vector<EmitBuf>& bufs) {
  for (EmitBuf& buf : bufs) {
    for (EmitChunk& chunk : buf.chunks) {
      if (srcFp == NULL || (srcFileBytes > (globalConfig.cppMaxSizeKB * 1024) && chunk.canNewFile)) {
        rotateSrcFile(chunk.alreadyEndFunc, chunk.hasNextFuncDef ? chunk.nextFuncDef.c_str() : nullptr);
      }
      fwrite(chunk.text.data(), 1, chunk.text.size(), srcFp);
      srcFileBytes += (int)chunk.countedBytes;
    }
    std::vector<EmitChunk>().swap(buf.chunks);
  }
}

struct EmitCtxSnapshot {
  bool traceSuppressed;
  int traceSourceCppId;
};


// Renders `unitCount` independent emission units on a worker pool, then
// assembles them sequentially in unit order. Determinism does not depend on
// scheduling: buffers are indexed by unit, never by worker.
void graph::emitUnitsParallel(size_t unitCount, const std::function<void(size_t)>& renderUnit) {
  if (unitCount == 0) return;
  std::vector<EmitBuf> bufs(unitCount);
  const size_t nWorkers = std::min((size_t)emitParallelThreadCount(), unitCount);
  if (nWorkers <= 1) {
    for (size_t u = 0; u < unitCount; u ++) {
      emitCaptureTarget = &bufs[u];
      renderUnit(u);
      emitCaptureTarget = nullptr;
    }
    flushEmitBufs(bufs);
    return;
  }
  EmitCtxSnapshot ctx;
  ctx.traceSuppressed = mtActivationEventTraceSuppressed;
  ctx.traceSourceCppId = mtActivationEventTraceSourceCppId;
  std::atomic<size_t> nextUnit(0);
  std::vector<std::thread> pool;
  pool.reserve(nWorkers);
  for (size_t w = 0; w < nWorkers; w ++) {
    pool.emplace_back([&, ctx]() {
      mtActivationEventTraceSuppressed = ctx.traceSuppressed;
      mtActivationEventTraceSourceCppId = ctx.traceSourceCppId;
      size_t u;
      while ((u = nextUnit.fetch_add(1, std::memory_order_relaxed)) < unitCount) {
        emitCaptureTarget = &bufs[u];
        renderUnit(u);
        emitCaptureTarget = nullptr;
      }
    });
  }
  for (std::thread& t : pool) t.join();
  flushEmitBufs(bufs);
}

void graph::emitPrintf() {
  emitFuncDecl(0, "void gprintf(const char *fmt, ...) {\n");
  if (mtUseDenseUnpinSpecial()) {
    emitBodyLock(0,
    "  static std::mutex gGprintfMutex;\n"
    "  std::lock_guard<std::mutex> gprintfLock(gGprintfMutex);\n");
  }
  emitBodyLock(0,
  "  FILE *fp = stderr;\n"
  "  va_list args;\n"
  "  va_start(args, fmt);\n"
  "  int fmt_idx = 0;\n"
  "  while (true) {\n"
  "    char c = fmt[fmt_idx ++];\n"
  "    switch (c) {\n"
  "      case '%%': break;\n"
  "      case 0: return;\n"
  "      default: fputc(c, fp); continue;\n"
  "    }\n"
  "\n"
  "    uint64_t lval = 0;\n"
  "    int bits = va_arg(args, uint32_t);\n"
  "    if      (bits <= 32) { lval = va_arg(args, uint32_t); }\n"
  "    else if (bits <= 64) { lval = va_arg(args, uint64_t); }\n"
  "    else                 { assert(0); }\n"
  "\n"
  "    c = fmt[fmt_idx ++];\n"
  "    switch (c) {\n"
  "      case 'd': fprintf(fp, \"%%ld\", lval); break;\n"
  "      case 'c': fputc(lval & 0xff, fp); break;\n"
  "      case 'x': fprintf(fp, \"%%lx\", lval); break;\n"
  "      default: assert(0);\n"
  "    }\n"
  "  }\n"
  "}\n"
  );
}

void graph::cppEmitter() {
  for (SuperNode* super : sortedSuper) {
    if (!super->instsEmpty() || super->superType == SUPER_EXTMOD || super->superType == SUPER_ASYNC_RESET) {
      super->cppId = superId ++;
      cppId2Super[super->cppId] = super;
      if (super->superType == SUPER_EXTMOD) {
        alwaysActive.insert(super->cppId);
      }
    }
  }
  activeFlagNum = (superId + ACTIVE_WIDTH - 1) / ACTIVE_WIDTH;
  // avoid buffer overflow when accessing the last elements as uint64_t
  activeFlagNum = ROUNDUP(activeFlagNum, 8);

  for (SuperNode* super : sortedSuper) {
    for (Node* member : super->member) {
      if (member->status == VALID_NODE) {
        member->updateActivate();
        member->updateNeedActivate(alwaysActive);
      }
    }
  }
  resetMtContextCache();
  { EmitPhaseTimer t("Final.dumpMtScheduleJson");
    if (globalConfig.DumpMtScheduleJson) dumpMtScheduleJson(); }
  { EmitPhaseTimer t("Final.dumpMtCoarseRegionReport");
    if (globalConfig.DumpMtCoarseRegionReport || globalConfig.MtBatchFormationMode == "coarse") dumpMtCoarseRegionReport(); }
  { EmitPhaseTimer t("Final.dumpMtReadyBatchReport");
    if (mtUseReadyBatchReport() || mtUseEnvelopeLocalEvalDiagnostics()) dumpMtReadyBatchReport(); }
  { EmitPhaseTimer t("Final.dumpMtDenseScheduleJson");
    if (mtUseDenseExecutorCodegen()) dumpMtDenseScheduleJson(); }
  // Intern node names only after every report/dump above (mt schedule JSON,
  // coarse/ready-batch reports, dense schedule JSON): they name nodes
  // by their original full names, and mtDenseObservabilityDroppableSet's
  // classifier (still on call) must keep seeing the original hierarchy.
  if (mtUseShortNames()) { EmitPhaseTimer t("Final.shortNames"); mtInternNodeNames(); }
  if (globalConfig.MtReportOnly) {
    printf("[cppEmitter] mt-report-only: skipped generated C++ emission after reports\n");
    return;
  }

  if (!globalConfig.MtStableOutput) {
    // remove stale SimTop*.cpp files from previous runs so the
    // linker never sees a cppEmitter file the current run did not regenerate.
    for (int staleIdx = 0; ; staleIdx ++) {
      std::string stalePath = format("%s%d.cpp", (globalConfig.OutputDir + "/" + name).c_str(), staleIdx);
      if (std::remove(stalePath.c_str()) != 0) break;
    }
  }

  srcFp = NULL;
  srcFileIdx = 0;

  FILE* header = genHeaderStart();
#ifdef DIFFTEST_PER_SIG
  sigFile = fopen((globalConfig.OutputDir + "/" + name + "_sigs.txt").c_str(), "w");
#endif

  /* class start*/
  bool useMtHelpers = globalConfig.MtHelperMode == "mt" ||
                      globalConfig.MtHelperMode == "mt-level-dispatch";
  bool useSeqHelpers = globalConfig.MtHelperMode == "seq" ||
                       globalConfig.MtHelperMode == "buffered-seq";
  bool useBufferedHelpers = globalConfig.MtHelperMode == "buffered-seq" || useMtHelpers;
  bool useHelperTasks = useSeqHelpers || useMtHelpers;
  bool useCoarseMt = useMtHelpers && globalConfig.MtBatchFormationMode == "coarse";
  std::map<int, MtTaskInfo> mtHeaderTasks;
  MtCoarseProfileFacts mtCoarseProfileFacts;
  bool useDenseExecutorCodegen = mtUseDenseExecutorCodegen();
  bool activationEventTraceCodegen = mtUseActivationEventTraceCodegen();
  MtDenseSchedule mtDenseSchedule;
  EmitPhaseTimer scheduleTimer("Final.scheduleBuild");
  if (useMtHelpers) {
    { EmitPhaseTimer infoMapTimer("Final.schedBuild.infoMap");
      mtHeaderTasks = buildMtTaskInfoMapForInvocation();
    }
    if (useCoarseMt) {
      MtCoarseRegionPlan coarsePlan;
      { EmitPhaseTimer coarsePlanTimer("Final.schedBuild.coarsePlan"); coarsePlan = planMtCoarseRegionsForInvocation(); }
      { EmitPhaseTimer coarseFactsTimer("Final.schedBuild.coarseFacts"); mtCoarseProfileFacts = mtComputeCoarseProfileFacts(coarsePlan); }
    }
  }
  if (useDenseExecutorCodegen) {
    Assert(useCoarseMt, "GSIM_MT_DENSE_EXECUTOR_CODEGEN requires --mt-helper-mode=mt-level-dispatch with coarse batch formation");
    if (mtHeaderTasks.empty()) {
      { EmitPhaseTimer infoMapTimer("Final.schedBuild.infoMap");
        mtHeaderTasks = buildMtTaskInfoMapForInvocation();
      }
    }
    { EmitPhaseTimer denseSchedTimer("Final.schedBuild.denseSched");
      if (mtDenseScheduleCacheValid && mtDenseScheduleCache.codegenEnabled) {
        mtDenseSchedule = std::move(mtDenseScheduleCache);
        mtDenseScheduleCacheValid = false;
      } else {
        mtDenseSchedule = buildMtDenseSchedule(mtHeaderTasks, true);
      }
    }
  }
  bool denseExecutorValid = useDenseExecutorCodegen && mtDenseSchedule.valid;
  bool denseBreakdownProfileCodegen = mtUseDenseBreakdownProfileCodegen();
  bool denseBreakdownWindowCodegen = denseBreakdownProfileCodegen && mtUseDenseBreakdownWindowCodegen();
  int denseBreakdownWindowWorker0MTaskCount = 0;
  int denseBreakdownWindowAllOwnerMTaskCount = 0;
  int denseBreakdownWindowThreadCount = 8;
  if (denseBreakdownWindowCodegen) {
    const char* denseBreakdownWindowThreadsEnv = std::getenv("GSIM_THREADS");
    if (denseBreakdownWindowThreadsEnv != nullptr && denseBreakdownWindowThreadsEnv[0] != '\0') denseBreakdownWindowThreadCount = std::atoi(denseBreakdownWindowThreadsEnv);
    if (denseBreakdownWindowThreadCount < 1) denseBreakdownWindowThreadCount = 1;
    Assert(denseBreakdownWindowThreadCount >= 2 && denseBreakdownWindowThreadCount <= 16,
           "GSIM_MT_DENSE_BREAKDOWN window requires 2..16 workers (got %d)",
           denseBreakdownWindowThreadCount);
  }
  if (denseBreakdownProfileCodegen) {
    Assert(denseExecutorValid,
           "GSIM_MT_DENSE_BREAKDOWN_PROFILE requires a valid dense executor");
  }
  if (denseBreakdownWindowCodegen) {
    for (int mtaskId = 0; mtaskId < static_cast<int>(mtDenseSchedule.mtaskThreadAssign.size()); mtaskId ++) {
      if (mtDenseSchedule.mtaskThreadAssign[(size_t)mtaskId] == 0) denseBreakdownWindowWorker0MTaskCount ++;
    }
    Assert(denseBreakdownWindowWorker0MTaskCount <= 2267,
           "GSIM_MT_DENSE_BREAKDOWN window supports at most 2267 worker0 MTasks (got %d)",
           denseBreakdownWindowWorker0MTaskCount);
  }
  if (denseBreakdownWindowCodegen) {
    denseBreakdownWindowAllOwnerMTaskCount = static_cast<int>(mtDenseSchedule.mtaskThreadAssign.size());
    Assert(denseBreakdownWindowAllOwnerMTaskCount <= 32768,
           "GSIM_MT_DENSE_BREAKDOWN allownerbody supports at most 32768 MTasks (got %d)",
           denseBreakdownWindowAllOwnerMTaskCount);
  }
  MtDenseBreakdownWindowAllOwnerLayout denseBreakdownWindowAllOwnerLayout;
  if (denseBreakdownWindowCodegen) {
    denseBreakdownWindowAllOwnerLayout = mtBuildDenseBreakdownWindowAllOwnerLayout(
        mtDenseSchedule.mtaskThreadAssign, denseBreakdownWindowThreadCount);
    Assert(denseBreakdownWindowAllOwnerLayout.recordCount >= denseBreakdownWindowAllOwnerMTaskCount
           && denseBreakdownWindowAllOwnerLayout.recordCount
                  <= denseBreakdownWindowAllOwnerMTaskCount + 7 * denseBreakdownWindowThreadCount,
           "dense breakdown header all-owner physical layout count mismatch");
  }
  std::vector<int> denseBreakdownWindowCausalSameOwnerProducerByMTask;
  if (denseBreakdownWindowCodegen) {
    denseBreakdownWindowCausalSameOwnerProducerByMTask.assign(
        (size_t)denseBreakdownWindowAllOwnerMTaskCount, -1);
    std::vector<int> denseBreakdownWindowLastMTaskByOwner(
        (size_t)denseBreakdownWindowThreadCount, -1);
    for (int mtaskId = 0; mtaskId < denseBreakdownWindowAllOwnerMTaskCount; mtaskId ++) {
      const int owner = mtDenseSchedule.mtaskThreadAssign[(size_t)mtaskId];
      Assert(owner >= 0 && owner < denseBreakdownWindowThreadCount,
             "dense breakdown causal same-owner MTask %d has invalid owner %d", mtaskId, owner);
      denseBreakdownWindowCausalSameOwnerProducerByMTask[(size_t)mtaskId] =
          denseBreakdownWindowLastMTaskByOwner[(size_t)owner];
      denseBreakdownWindowLastMTaskByOwner[(size_t)owner] = mtaskId;
    }
  }
  MtDenseOwnerReadyLayout denseBreakdownWindowReadyLayout;
  MtDenseBreakdownWindowWaitLayout denseBreakdownWindowWaitLayout;
  if (denseBreakdownWindowCodegen) {
    const bool denseBreakdownWindowXThreadDepsOnly = mtUseDenseXThreadDepsOnly();
    const bool denseBreakdownWindowTransitiveReduce = mtUseDenseTransitiveReduceEdges();
    std::vector<std::vector<int>> denseBreakdownWindowRuntimeSuccs = mtBuildDenseRuntimeSuccs(
        mtDenseSchedule.mtasks, mtDenseSchedule.mtaskThreadAssign, denseBreakdownWindowXThreadDepsOnly);
    if (denseBreakdownWindowTransitiveReduce) {
      mtReduceDenseRuntimeSuccsTransitive(denseBreakdownWindowRuntimeSuccs, mtDenseSchedule.mtaskThreadAssign);
    }
    denseBreakdownWindowReadyLayout = mtBuildDenseOwnerReadyLayout(
        denseBreakdownWindowRuntimeSuccs, mtDenseSchedule.mtaskThreadAssign, denseBreakdownWindowThreadCount);
    denseBreakdownWindowWaitLayout = mtBuildDenseBreakdownWindowWaitLayout(
        denseBreakdownWindowReadyLayout, mtDenseSchedule.mtaskThreadAssign, denseBreakdownWindowThreadCount);
    Assert(denseBreakdownWindowWaitLayout.totalWaitRecords <= 65536,
           "GSIM_MT_DENSE_BREAKDOWN window supports at most 65536 ready waits per cycle (got %d)",
           denseBreakdownWindowWaitLayout.totalWaitRecords);
    Assert(denseBreakdownWindowWaitLayout.totalWaitRecords == denseBreakdownWindowReadyLayout.tokenCount,
           "dense breakdown window wait/token count mismatch: waits=%d tokens=%d",
           denseBreakdownWindowWaitLayout.totalWaitRecords, denseBreakdownWindowReadyLayout.tokenCount);
    Assert(static_cast<int>(denseBreakdownWindowReadyLayout.tokenProvenanceByLogicalToken.size())
               == denseBreakdownWindowReadyLayout.tokenCount
           && static_cast<int>(denseBreakdownWindowReadyLayout.logicalTokenByPhysicalSlot.size())
                  == denseBreakdownWindowReadyLayout.physicalSlotCount,
           "dense breakdown window causal token provenance is incomplete");
  }
  std::vector<int> denseBreakdownWindowCausalHotspotRemoteTokenOutputOrder;
  std::vector<int> denseBreakdownWindowCausalHotspotSameOwnerConsumerOutputOrder;
  if (denseBreakdownWindowCodegen) {
    for (int token = 0; token < denseBreakdownWindowReadyLayout.tokenCount; token ++) {
      denseBreakdownWindowCausalHotspotRemoteTokenOutputOrder.push_back(token);
    }
    std::sort(denseBreakdownWindowCausalHotspotRemoteTokenOutputOrder.begin(),
              denseBreakdownWindowCausalHotspotRemoteTokenOutputOrder.end(),
              [&](int left, int right) {
                const MtDenseOwnerReadyTokenProvenance& leftToken =
                    denseBreakdownWindowReadyLayout.tokenProvenanceByLogicalToken[(size_t)left];
                const MtDenseOwnerReadyTokenProvenance& rightToken =
                    denseBreakdownWindowReadyLayout.tokenProvenanceByLogicalToken[(size_t)right];
                if (leftToken.producerMTask != rightToken.producerMTask)
                  return leftToken.producerMTask < rightToken.producerMTask;
                if (leftToken.consumerMTask != rightToken.consumerMTask)
                  return leftToken.consumerMTask < rightToken.consumerMTask;
                return left < right;
              });
    for (int mtaskId = 0; mtaskId < denseBreakdownWindowAllOwnerMTaskCount; mtaskId ++) {
      denseBreakdownWindowCausalHotspotSameOwnerConsumerOutputOrder.push_back(mtaskId);
    }
    std::sort(denseBreakdownWindowCausalHotspotSameOwnerConsumerOutputOrder.begin(),
              denseBreakdownWindowCausalHotspotSameOwnerConsumerOutputOrder.end(),
              [&](int left, int right) {
                const int leftProducer =
                    denseBreakdownWindowCausalSameOwnerProducerByMTask[(size_t)left];
                const int rightProducer =
                    denseBreakdownWindowCausalSameOwnerProducerByMTask[(size_t)right];
                if (leftProducer != rightProducer) return leftProducer < rightProducer;
                return left < right;
              });
  }
  if (useDenseExecutorCodegen && !mtDenseSchedule.valid) {
    printf("[mt-dense-schedule] dense executor codegen disabled: fallback=%s\n", mtDenseSchedule.fallbackReason.c_str());
  }
  std::vector<unsigned char> mtProfileStateUpdateTraceKindByCppIdCodegen;
  if (mtUseDynamicStateTraceCodegen()) {
    mtProfileStateUpdateTraceKindByCppIdCodegen.assign(superId, 0);
    std::map<int, MtTaskInfo> mtStateTraceTasks = mtHeaderTasks;
    if (mtStateTraceTasks.empty()) mtStateTraceTasks = buildMtTaskInfoMapForInvocation();
    std::vector<MtStateUpdateTraceInfo> mtStateTraceInfos = buildMtStateUpdateTraceInfoForInvocation(mtStateTraceTasks);
    for (int cppId = 0; cppId < static_cast<int>(mtStateTraceInfos.size()); cppId ++) {
      const MtStateUpdateTraceInfo& info = mtStateTraceInfos[cppId];
      if (!info.hasStateUpdate) continue;
      mtProfileStateUpdateTraceKindByCppIdCodegen[cppId] = info.runtimeSafeCandidate ? 3 : (info.localSafeCandidate ? 2 : 1);
    }
  }
  if (globalConfig.MtHelperMode == "buffered-seq") emitActiveBufferDef(header, activeFlagNum);
  if (useMtHelpers) emitActivationDeltaDef(header, activeFlagNum);
  if (activationEventTraceCodegen) emitActivationEventTraceDef(header);

  fprintf(header, "class S%s {\npublic:\n", name.c_str());
  fprintf(header, "uint64_t cycles;\n");
  fprintf(header, "uint64_t LOG_START, LOG_END;\n");
  fprintf(header, "uint%d_t activeFlags[%d];\n", ACTIVE_WIDTH, activeFlagNum); // or super.size() if id == idx
  if (denseExecutorValid) fprintf(header, "bool mtUseDenseExecutor;\n");
  fprintf(header, "bool mtProfileEnabled;\n");
  if (denseBreakdownProfileCodegen) {
    fprintf(header, "static constexpr int kDenseBreakdownProfileWorkerCount = 16;\n");
    fprintf(header, "struct alignas(64) MtDenseBreakdownWorker {\n");
    fprintf(header, "  uint64_t dispatchSpanNs;\n");
    fprintf(header, "  uint64_t blockedWaitNs;\n");
    fprintf(header, "  uint64_t blockedWaitCount;\n");
    fprintf(header, "  uint8_t padding[64 - 3 * sizeof(uint64_t)];\n");
    fprintf(header, "};\n");
    fprintf(header, "static_assert(sizeof(MtDenseBreakdownWorker) == 64, \"dense breakdown worker lanes must be cache-line sized\");\n");
    fprintf(header, "bool mtDenseBreakdownProfileEnabled;\n");
    fprintf(header, "char mtDenseBreakdownProfileOutPath[4096];\n");
    fprintf(header, "MtDenseBreakdownWorker mtDenseBreakdownWorkers[kDenseBreakdownProfileWorkerCount];\n");
    fprintf(header, "uint64_t mtDenseBreakdownPoolIdleNs;\n");
    fprintf(header, "uint64_t mtDenseBreakdownPoolDoneNs;\n");
    fprintf(header, "uint64_t mtDenseBreakdownTotalStepNs;\n");
    if (denseBreakdownWindowCodegen) {
      const int denseBreakdownWindowWorker0MTaskStorageCount = std::max(1, denseBreakdownWindowWorker0MTaskCount);
      const int denseBreakdownWindowAllOwnerMTaskStorageCount = std::max(8, denseBreakdownWindowAllOwnerLayout.recordCount);
      const int denseBreakdownWindowWaitRecordStorageCount = std::max(1, denseBreakdownWindowWaitLayout.totalWaitRecords);
      const int denseBreakdownWindowCausalTokenStorageCount =
          std::max(1, denseBreakdownWindowReadyLayout.tokenCount);
      const int denseBreakdownWindowCausalReadySlotStorageCount =
          std::max(1, denseBreakdownWindowReadyLayout.physicalSlotCount);
      fprintf(header, "static constexpr int kDenseBreakdownWindowMaxCycles = 256;\n");
      fprintf(header, "static constexpr int kDenseBreakdownWindowMaxWorker0MTasks = 2267;\n");
      fprintf(header, "static constexpr int kDenseBreakdownWindowThreadCount = %d;\n", denseBreakdownWindowThreadCount);
      fprintf(header, "static constexpr int kDenseBreakdownWindowWorker0MTaskCount = %d;\n", denseBreakdownWindowWorker0MTaskCount);
      fprintf(header, "static constexpr int kDenseBreakdownWindowWorker0MTaskStorageCount = %d;\n", denseBreakdownWindowWorker0MTaskStorageCount);
      fprintf(header, "static constexpr int kDenseBreakdownWindowCausalTokenCount = %d;\n", denseBreakdownWindowReadyLayout.tokenCount);
      fprintf(header, "static constexpr int kDenseBreakdownWindowCausalTokenStorageCount = %d;\n", denseBreakdownWindowCausalTokenStorageCount);
      fprintf(header, "static constexpr int kDenseBreakdownWindowCausalTokenReadySlot[%d] = {", denseBreakdownWindowCausalTokenStorageCount);
      for (int token = 0; token < denseBreakdownWindowReadyLayout.tokenCount; token ++) {
        if (token != 0) fprintf(header, ",");
        fprintf(header, "%d", denseBreakdownWindowReadyLayout.tokenProvenanceByLogicalToken[(size_t)token].readySlot);
      }
      if (denseBreakdownWindowReadyLayout.tokenCount == 0) fprintf(header, "0");
      fprintf(header, "};\n");
      fprintf(header, "static constexpr int kDenseBreakdownWindowCausalTokenProducerMTask[%d] = {", denseBreakdownWindowCausalTokenStorageCount);
      for (int token = 0; token < denseBreakdownWindowReadyLayout.tokenCount; token ++) { if (token != 0) fprintf(header, ","); fprintf(header, "%d", denseBreakdownWindowReadyLayout.tokenProvenanceByLogicalToken[(size_t)token].producerMTask); }
      if (denseBreakdownWindowReadyLayout.tokenCount == 0) fprintf(header, "0");
      fprintf(header, "};\n");
      fprintf(header, "static constexpr int kDenseBreakdownWindowCausalTokenProducerOwner[%d] = {", denseBreakdownWindowCausalTokenStorageCount);
      for (int token = 0; token < denseBreakdownWindowReadyLayout.tokenCount; token ++) { if (token != 0) fprintf(header, ","); fprintf(header, "%d", denseBreakdownWindowReadyLayout.tokenProvenanceByLogicalToken[(size_t)token].producerOwner); }
      if (denseBreakdownWindowReadyLayout.tokenCount == 0) fprintf(header, "0");
      fprintf(header, "};\n");
      fprintf(header, "static constexpr int kDenseBreakdownWindowCausalTokenConsumerMTask[%d] = {", denseBreakdownWindowCausalTokenStorageCount);
      for (int token = 0; token < denseBreakdownWindowReadyLayout.tokenCount; token ++) { if (token != 0) fprintf(header, ","); fprintf(header, "%d", denseBreakdownWindowReadyLayout.tokenProvenanceByLogicalToken[(size_t)token].consumerMTask); }
      if (denseBreakdownWindowReadyLayout.tokenCount == 0) fprintf(header, "0");
      fprintf(header, "};\n");
      fprintf(header, "static constexpr int kDenseBreakdownWindowCausalTokenConsumerOwner[%d] = {", denseBreakdownWindowCausalTokenStorageCount);
      for (int token = 0; token < denseBreakdownWindowReadyLayout.tokenCount; token ++) { if (token != 0) fprintf(header, ","); fprintf(header, "%d", denseBreakdownWindowReadyLayout.tokenProvenanceByLogicalToken[(size_t)token].consumerOwner); }
      if (denseBreakdownWindowReadyLayout.tokenCount == 0) fprintf(header, "0");
      fprintf(header, "};\n");
      fprintf(header, "static constexpr int kDenseBreakdownWindowCausalLogicalTokenByReadySlot[%d] = {", denseBreakdownWindowCausalReadySlotStorageCount);
      for (int readySlot = 0; readySlot < denseBreakdownWindowReadyLayout.physicalSlotCount; readySlot ++) { if (readySlot != 0) fprintf(header, ","); fprintf(header, "%d", denseBreakdownWindowReadyLayout.logicalTokenByPhysicalSlot[(size_t)readySlot]); }
      fprintf(header, "};\n");
      fprintf(header, "static constexpr int kDenseBreakdownWindowMaxAllOwnerMTasks = 32768;\n");
      fprintf(header, "static constexpr int kDenseBreakdownWindowMaxAllOwnerMTaskStorageCount = 32880;\n");
      fprintf(header, "static constexpr int kDenseBreakdownWindowAllOwnerMTaskCount = %d;\n", denseBreakdownWindowAllOwnerMTaskCount);
      fprintf(header, "static constexpr int kDenseBreakdownWindowAllOwnerMTaskStorageCount = %d;\n", denseBreakdownWindowAllOwnerMTaskStorageCount);
      fprintf(header, "static constexpr int kDenseBreakdownWindowAllOwnerLaneOffsets[kDenseBreakdownWindowThreadCount + 1] = {");
      for (int worker = 0; worker <= denseBreakdownWindowThreadCount; worker ++) {
        if (worker != 0) fprintf(header, ",");
        fprintf(header, "%d", denseBreakdownWindowAllOwnerLayout.laneOffsets[(size_t)worker]);
      }
      fprintf(header, "};\n");
      fprintf(header, "static constexpr int kDenseBreakdownWindowAllOwnerMTaskRecordIndex[kDenseBreakdownWindowAllOwnerMTaskStorageCount] = {");
      for (int mtaskId = 0; mtaskId < denseBreakdownWindowAllOwnerMTaskCount; mtaskId ++) {
        if (mtaskId != 0) fprintf(header, ",");
        fprintf(header, "%d", denseBreakdownWindowAllOwnerLayout.recordIndexByMTask[(size_t)mtaskId]);
      }
      fprintf(header, "};\n");
      // Fixed workers traverse their assigned logical MTask ids in ascending order after
      // SCHED_ORDER has renumbered the schedule. This map is deliberately logical-sized:
      // all-owner storage padding is not a causal same-owner vertex.
      fprintf(header, "static constexpr int kDenseBreakdownWindowCausalSameOwnerProducerMTask[kDenseBreakdownWindowAllOwnerMTaskCount] = {");
      for (int mtaskId = 0; mtaskId < denseBreakdownWindowAllOwnerMTaskCount; mtaskId ++) {
        if (mtaskId != 0) fprintf(header, ",");
        fprintf(header, "%d", denseBreakdownWindowCausalSameOwnerProducerByMTask[(size_t)mtaskId]);
      }
      fprintf(header, "};\n");
      // These permutations contain only logical ids. Do not use storage capacities here:
      // those include sentinel padding that is not a causal edge or MTask.
      fprintf(header, "static constexpr int kDenseBreakdownWindowCausalHotspotRemoteTokenOutputOrder[kDenseBreakdownWindowCausalTokenCount] = {");
      for (int index = 0; index < denseBreakdownWindowReadyLayout.tokenCount; index ++) {
        if (index != 0) fprintf(header, ",");
        fprintf(header, "%d", denseBreakdownWindowCausalHotspotRemoteTokenOutputOrder[(size_t)index]);
      }
      fprintf(header, "};\n");
      fprintf(header, "static constexpr int kDenseBreakdownWindowCausalHotspotSameOwnerConsumerOutputOrder[kDenseBreakdownWindowAllOwnerMTaskCount] = {");
      for (int index = 0; index < denseBreakdownWindowAllOwnerMTaskCount; index ++) {
        if (index != 0) fprintf(header, ",");
        fprintf(header, "%d", denseBreakdownWindowCausalHotspotSameOwnerConsumerOutputOrder[(size_t)index]);
      }
      fprintf(header, "};\n");
      fprintf(header, "static_assert(kDenseBreakdownWindowAllOwnerMTaskCount <= kDenseBreakdownWindowMaxAllOwnerMTasks, \"dense breakdown all-owner MTask cap\");\n");
      fprintf(header, "static_assert(kDenseBreakdownWindowAllOwnerMTaskStorageCount <= kDenseBreakdownWindowMaxAllOwnerMTaskStorageCount, \"dense breakdown all-owner physical storage cap\");\n");
      fprintf(header, "static_assert((kDenseBreakdownWindowAllOwnerMTaskStorageCount %% 8) == 0, \"dense breakdown all-owner row alignment\");\n");
      fprintf(header, "static constexpr int kDenseBreakdownWindowMaxWaitRecordsPerCycle = 65536;\n");
      fprintf(header, "static constexpr int kDenseBreakdownWindowWaitRecordCount = %d;\n", denseBreakdownWindowWaitLayout.totalWaitRecords);
      fprintf(header, "static constexpr int kDenseBreakdownWindowWaitRecordStorageCount = %d;\n", denseBreakdownWindowWaitRecordStorageCount);
      fprintf(header, "static constexpr int kDenseBreakdownWindowWaitLaneOffsets[kDenseBreakdownWindowThreadCount + 1] = {");
      for (int worker = 0; worker <= denseBreakdownWindowThreadCount; worker ++) {
        if (worker != 0) fprintf(header, ",");
        fprintf(header, "%d", denseBreakdownWindowWaitLayout.laneOffsets[(size_t)worker]);
      }
      fprintf(header, "};\n");
      fprintf(header, "static_assert(kDenseBreakdownWindowWaitRecordCount <= kDenseBreakdownWindowMaxWaitRecordsPerCycle, \"dense breakdown window wait cap\");\n");
      fprintf(header, "static_assert(kDenseBreakdownWindowWorker0MTaskCount <= kDenseBreakdownWindowMaxWorker0MTasks, \"dense breakdown window worker0 MTask cap\");\n");
      fprintf(header, "struct alignas(64) MtDenseBreakdownWindowWorker {\n");
      fprintf(header, "  uint64_t startOffsetNs;\n");
      fprintf(header, "  uint64_t finishOffsetNs;\n");
      fprintf(header, "  uint64_t blockedWaitNs;\n");
      fprintf(header, "  uint64_t bodyNs;\n");
      fprintf(header, "  uint64_t controlNs;\n");
      fprintf(header, "  uint8_t padding[64 - 5 * sizeof(uint64_t)];\n");
      fprintf(header, "};\n");
      fprintf(header, "static_assert(sizeof(MtDenseBreakdownWindowWorker) == 64, \"dense breakdown window worker lanes must be cache-line sized\");\n");
      fprintf(header, "struct MtDenseBreakdownWindowMTask { uint32_t mtaskId; uint32_t padding; uint64_t bodyNs; };\n");
      fprintf(header, "struct MtDenseBreakdownWindowWait { uint16_t cycleSlot; uint16_t threadId; uint32_t consumerMtaskId; uint32_t readySlot; uint32_t padding; uint64_t blockedNs; uint64_t endOffsetNs; };\n");
      fprintf(header, "struct MtDenseBreakdownWindowAllOwnerMTask { uint32_t mtaskId; uint16_t ownerThreadId; uint16_t readyTokenStoreCount; uint64_t bodyStartOffsetNs; uint64_t bodyEndOffsetNs; uint64_t bodyNs; uint64_t releaseEndOffsetNs; };\n");
      fprintf(header, "static_assert(sizeof(MtDenseBreakdownWindowAllOwnerMTask) == 40, \"dense breakdown all-owner record size\");\n");
      fprintf(header, "struct MtDenseBreakdownWindowReadyToken { uint64_t releaseBeforeOffsetNs; uint64_t releaseAfterOffsetNs; };\n");
      fprintf(header, "struct MtDenseBreakdownWindowCausalSummary { uint64_t causalBoundNs; uint64_t maxLagNs; uint64_t timestampBoundUncertaintyNs; uint64_t latestOwnerFinishNs; uint64_t makespanNs; uint64_t chainBodyNs; uint64_t chainReleaseNs; uint64_t chainGapNs; uint32_t sameOwnerPredecessorCount; uint32_t remoteTokenPredecessorCount; uint32_t waitObservationCount; uint32_t chainNodeCount; uint32_t chainEdgeCount; int32_t latestOwner; bool complete; bool incompleteMapping; bool clockRegression; bool overflow; };\n");
      fprintf(header, "struct MtDenseBreakdownWindowCausalHotspotMTask { uint64_t count; uint64_t bodyNs; uint64_t releaseNs; uint64_t gapNs; uint64_t sameOwnerPredCount; uint64_t remoteTokenPredCount; uint64_t latestTailCount; };\n");
      fprintf(header, "struct MtDenseBreakdownWindowCausalHotspotEdge { uint64_t count; uint64_t gapNs; };\n");
      fprintf(header, "struct MtDenseBreakdownWindowCausalHotspotTotals { uint64_t count; uint64_t bodyNs; uint64_t releaseNs; uint64_t gapNs; uint64_t sameOwnerPredCount; uint64_t remoteTokenPredCount; uint64_t latestTailCount; uint64_t edgeCount; };\n");
      fprintf(header, "bool mtDenseBreakdownWindowEnabled;\n");
      fprintf(header, "bool mtDenseBreakdownWindowWorker0BodyMode;\n");
      fprintf(header, "uint64_t mtDenseBreakdownWindowCausalRemotePredecessorEnds[kDenseBreakdownWindowMaxAllOwnerMTasks];\n");
      fprintf(header, "int mtDenseBreakdownWindowCausalRemotePredecessorTokens[kDenseBreakdownWindowMaxAllOwnerMTasks];\n");
      fprintf(header, "int mtDenseBreakdownWindowCausalSelectedPredecessors[kDenseBreakdownWindowMaxAllOwnerMTasks];\n");
      fprintf(header, "uint64_t mtDenseBreakdownWindowCausalSelectedPredecessorEnds[kDenseBreakdownWindowMaxAllOwnerMTasks];\n");
      fprintf(header, "uint32_t mtDenseBreakdownWindowCausalVisitMarks[kDenseBreakdownWindowMaxAllOwnerMTasks];\n");
      fprintf(header, "MtDenseBreakdownWindowCausalHotspotMTask mtDenseBreakdownWindowCausalHotspotMTasks[kDenseBreakdownWindowMaxAllOwnerMTasks];\n");
      fprintf(header, "MtDenseBreakdownWindowCausalHotspotEdge mtDenseBreakdownWindowCausalHotspotRemoteTokenEdges[%d];\n", denseBreakdownWindowCausalTokenStorageCount);
      fprintf(header, "MtDenseBreakdownWindowCausalHotspotEdge mtDenseBreakdownWindowCausalHotspotSameOwnerEdges[kDenseBreakdownWindowMaxAllOwnerMTasks];\n");
      fprintf(header, "MtDenseBreakdownWindowCausalHotspotTotals mtDenseBreakdownWindowCausalHotspotTotals;\n");
      fprintf(header, "bool mtDenseBreakdownWindowAllOwnerBodyMode;\n");
      fprintf(header, "bool mtDenseBreakdownWindowFinishOnlyMode;\n");
      fprintf(header, "bool mtDenseBreakdownWindowCausalChainMode;\n");
      fprintf(header, "bool mtDenseBreakdownWindowCausalHotspotsMode;\n");
      fprintf(header, "bool mtDenseBreakdownWindowCausalClockRegression;\n");
      fprintf(header, "uint64_t mtDenseBreakdownWindowCausalTimestampBoundUncertaintyNs;\n");
      fprintf(header, "std::atomic<bool> mtDenseBreakdownWindowOverflow;\n");
      fprintf(header, "uint64_t mtDenseBreakdownWindowStart;\n");
      fprintf(header, "uint64_t mtDenseBreakdownWindowCycles;\n");
      fprintf(header, "uint64_t mtDenseBreakdownWindowRecordedCycles;\n");
      fprintf(header, "int mtDenseBreakdownWindowCurrentSlot;\n");
      fprintf(header, "std::chrono::steady_clock::time_point mtDenseBreakdownWindowEpoch;\n");
      fprintf(header, "char mtDenseBreakdownWindowOutPath[4096];\n");
      fprintf(header, "uint64_t mtDenseBreakdownWindowCycleNumbers[kDenseBreakdownWindowMaxCycles];\n");
      fprintf(header, "uint32_t mtDenseBreakdownWindowWorker0MTaskCounts[kDenseBreakdownWindowMaxCycles];\n");
      fprintf(header, "MtDenseBreakdownWindowWorker mtDenseBreakdownWindowWorkers[kDenseBreakdownWindowMaxCycles][kDenseBreakdownWindowThreadCount];\n");
      fprintf(header, "MtDenseBreakdownWindowMTask mtDenseBreakdownWindowWorker0MTasks[kDenseBreakdownWindowMaxCycles][kDenseBreakdownWindowWorker0MTaskStorageCount];\n");
      fprintf(header, "uint32_t mtDenseBreakdownWindowWaitCounts[kDenseBreakdownWindowMaxCycles][kDenseBreakdownWindowThreadCount];\n");
      fprintf(header, "MtDenseBreakdownWindowWait mtDenseBreakdownWindowWaits[kDenseBreakdownWindowMaxCycles][kDenseBreakdownWindowWaitRecordStorageCount];\n");
      fprintf(header, "alignas(64) MtDenseBreakdownWindowAllOwnerMTask mtDenseBreakdownWindowAllOwnerMTasks[kDenseBreakdownWindowMaxCycles][kDenseBreakdownWindowAllOwnerMTaskStorageCount];\n");
      fprintf(header, "MtDenseBreakdownWindowReadyToken mtDenseBreakdownWindowReadyTokens[kDenseBreakdownWindowMaxCycles][%d];\n", denseBreakdownWindowCausalTokenStorageCount);
      fprintf(header, "MtDenseBreakdownWindowCausalSummary mtDenseBreakdownWindowCausalSummaries[kDenseBreakdownWindowMaxCycles];\n");
      fprintf(header, "static_assert(kDenseBreakdownWindowCausalTokenCount <= kDenseBreakdownWindowCausalTokenStorageCount, \"dense breakdown causal token storage cap\");\n");
    }
  }
  fprintf(header, "FILE *mtProfileDynamicTraceFile;\n");
  fprintf(header, "uint64_t mtProfileDynamicTraceCycleStart;\n");
  fprintf(header, "uint64_t mtProfileDynamicTraceCycleLimit;\n");
  fprintf(header, "std::vector<int> mtProfileDynamicTraceTaskIds;\n");
  if (activationEventTraceCodegen) {
    fprintf(header, "FILE *mtActivationEventTraceFile;\n");
    fprintf(header, "uint64_t mtActivationEventTraceCycleStart;\n");
    fprintf(header, "uint64_t mtActivationEventTraceCycleLimit;\n");
    fprintf(header, "bool mtActivationEventTraceCycleOpen;\n");
    fprintf(header, "std::vector<MtActivationEventTraceRecord> mtActivationEventTraceRecords;\n");
    fprintf(header, "std::vector<MtActivationEventTraceRecord> mtActivationEventTracePendingRecords;\n");
  }
  if (mtUseDynamicStateTraceCodegen()) {
    fprintf(header, "bool mtProfileDynamicStateTraceEnabled;\n");
    fprintf(header, "std::vector<uint8_t> mtProfileStateUpdateTraceKindByCppId;\n");
  }
  fprintf(header, "const char *mtProfileHelperMode;\n");
  fprintf(header, "int mtConfiguredWorkerCount;\n");
  fprintf(header, "int mtMinBatchTasks;\n");
  fprintf(header, "int mtSparseSerialFastMaxWorkers;\n");
  fprintf(header, "int mtCoarseMinActiveBits;\n");
  fprintf(header, "int mtCoarseInlineThreshold;\n");
  fprintf(header, "bool mtCoarseSkeletalMode;\n");
  fprintf(header, "int mtProfileConfiguredWorkerCount;\n");
  fprintf(header, "int mtProfileMaxWorkerCount;\n");
  fprintf(header, "uint64_t mtProfileActiveWordCount;\n");
  fprintf(header, "uint64_t mtProfileSerialTasks;\n");
  fprintf(header, "uint64_t mtProfilePureTasks;\n");
  fprintf(header, "uint64_t mtProfilePureBatchCount;\n");
  fprintf(header, "uint64_t mtProfileTrueParallelBatchCount;\n");
  fprintf(header, "uint64_t mtProfileSkippedFakeParallelBatchCount;\n");
  fprintf(header, "uint64_t mtProfileSerialFastTaskCount;\n");
  fprintf(header, "uint64_t mtProfileActivationDeltaEntries;\n");
  fprintf(header, "uint64_t mtProfileActivationDeltaMaxEntriesPerWorker;\n");
  fprintf(header, "uint64_t mtProfileActivationDeltaActivateAllCount;\n");
  fprintf(header, "uint64_t wallfracCommitCycles;\n");
  fprintf(header, "uint64_t wallfracCombCycles;\n");
  fprintf(header, "uint64_t wallfracCommitBrackets;\n");
  fprintf(header, "uint64_t wallfracCombBrackets;\n");
  fprintf(header, "uint64_t mtProfileRejectNotActiveWhole;\n");
  fprintf(header, "uint64_t mtProfileRejectAlwaysActiveTask;\n");
  fprintf(header, "uint64_t mtProfileRejectSerialTask;\n");
  fprintf(header, "uint64_t mtProfileSafeSerialDispatched;\n");      // 28c Phase 1A
  fprintf(header, "uint64_t mtProfileWorker0OnlyDispatched;\n");
  fprintf(header, "uint64_t mtProfileRejectDependencyEdge;\n");
  fprintf(header, "uint64_t mtProfileRejectSameActiveWordHazard;\n");
  fprintf(header, "uint64_t mtProfileRejectBelowMinBatch;\n");
  fprintf(header, "uint64_t mtProfileRejectConfiguredSingleWorker;\n");
  fprintf(header, "uint64_t mtProfileBatchMemberNodeCount;\n");
  fprintf(header, "uint64_t mtProfileSameActiveWordForwardEdges;\n");
  fprintf(header, "uint64_t mtProfileCrossBatchActivationFanout;\n");
  fprintf(header, "uint64_t mtProfileBatchWallNs;\n");
  fprintf(header, "uint64_t mtProfileTrueParallelWallNs;\n");
  fprintf(header, "uint64_t mtProfileSerialWallNs;\n");
  fprintf(header, "uint64_t mtProfileMergeWallNs;\n");
  fprintf(header, "uint64_t mtProfileTotalStepNs;\n");
  if (useCoarseMt) {
    fprintf(header, "uint64_t mtProfileCoarseStaticRuntimeEligibleRegions;\n");
    fprintf(header, "uint64_t mtProfileCoarseStaticLayerCount;\n");
    fprintf(header, "uint64_t mtProfileCoarseStaticMaxRegionLayerCount;\n");
    fprintf(header, "uint64_t mtProfileCoarseStaticMTaskCount;\n");
    fprintf(header, "uint64_t mtProfileCoarseRegionInvocations;\n");
    fprintf(header, "uint64_t mtProfileCoarseAcceptedRegions;\n");
    fprintf(header, "uint64_t mtProfileCoarseRejectedRegions;\n");
    fprintf(header, "uint64_t mtProfileCoarseLayerDispatches;\n");
    fprintf(header, "uint64_t mtProfileCoarseMTaskDispatches;\n");
    fprintf(header, "uint64_t mtProfileCoarseAntichainDispatches;\n");
    fprintf(header, "uint64_t mtProfileCoarseWorkerJobs;\n");
    fprintf(header, "uint64_t mtProfileCoarseFlagWordCopies;\n");
    fprintf(header, "uint64_t mtProfileCoarseMergeWordScans;\n");
    fprintf(header, "uint64_t mtProfileCoarseActivationDeltaEntries;\n");
    fprintf(header, "uint64_t mtProfileCoarseEstimatedBarrierCount;\n");
    fprintf(header, "uint64_t mtProfileCoarseBodyNs;\n");
    fprintf(header, "uint64_t mtProfileCoarseWaitNs;\n");
    fprintf(header, "uint64_t mtProfileCoarseEstimatedUsefulWork;\n");
    fprintf(header, "uint64_t mtProfileCoarseEstimatedRejectedUsefulWork;\n");
    fprintf(header, "uint64_t mtProfileCoarseEstimatedOverheadWords;\n");
    fprintf(header, "uint64_t mtProfileCoarseActiveMTaskCount;\n");
    fprintf(header, "uint64_t mtProfileCoarseActiveMTaskStaticCost;\n");
    fprintf(header, "uint64_t mtProfileCoarseAssignedStaticCost;\n");
    fprintf(header, "uint64_t mtProfileCoarseWorstWorkerStaticCost;\n");
    fprintf(header, "uint64_t mtProfileCoarseBestWorkerStaticCost;\n");
    fprintf(header, "uint64_t mtProfileCoarseContiguousWorstStaticCost;\n");
    fprintf(header, "uint64_t mtProfileCoarseBalancedWorstStaticCost;\n");
    fprintf(header, "uint64_t mtProfileCoarseSerialFallbackEligible;\n");
    fprintf(header, "uint64_t mtProfileCoarseSerialFallbackTaken;\n");
    fprintf(header, "uint64_t mtProfileCoarseSerialFallbackActiveBits;\n");
    fprintf(header, "uint64_t mtProfileCoarseSerialFallbackNonPureExcluded;\n");
    fprintf(header, "uint64_t mtProfileCoarseSerialFallbackSavedWorkerJobs;\n");
    fprintf(header, "uint64_t mtProfileCoarseSerialFallbackSavedFlagWordCopies;\n");
    fprintf(header, "uint64_t mtProfileCoarseSerialFallbackSavedMergeWordScans;\n");
    fprintf(header, "uint64_t mtProfileCoarseSerialFallbackSavedBarriers;\n");
    fprintf(header, "uint64_t mtProfileCoarseLayerSizeHist[6];\n");
    fprintf(header, "uint64_t mtProfileCoarseRegionLayerCountHist[6];\n");
    fprintf(header, "std::vector<uint64_t> mtProfileCoarseSelectedWorkerCountHist;\n");
  }
  fprintf(header, "uint64_t mtProfileTaskExecCount[%d];\n", superId);
  fprintf(header, "uint64_t mtProfileTaskWallNs[%d];\n", superId);
    fprintf(header, "std::vector<uint64_t> mtProfileWorkerTaskCount;\n");
    fprintf(header, "uint64_t mtProfileBatchSizeHist[6];\n");
    fprintf(header, "std::vector<uint64_t> mtProfileEffectiveWorkerCountHist;\n");
    if (useCoarseMt) {
      fprintf(header, "std::vector<uint64_t> mtProfileLocalActivationDeltaEntries;\n");
      fprintf(header, "std::vector<uint64_t> mtProfileLocalActivationDeltaMaxEntries;\n");
    }
  if (useMtHelpers) {
    fprintf(header, "struct alignas(64) MtWorkerPoolChunk {\n");
    fprintf(header, "  int begin;\n");
    fprintf(header, "  int end;\n");
    fprintf(header, "};\n");
    // ActivationDelta is alignas(64); each per-worker slot lives on its own cache line.
    fprintf(header, "std::vector<ActivationDelta> mtWorkerDeltas;\n");
    fprintf(header, "std::vector<uint%d_t> mtWorkerFlags;\n", ACTIVE_WIDTH);
    if (useCoarseMt) {
      fprintf(header, "std::vector<std::vector<uint%d_t>> mtWorkerCoarseFlags;\n", ACTIVE_WIDTH);
      if (globalConfig.MtCoarseWorkerPolicyMode == "profitable") {
        fprintf(header, "std::vector<std::vector<int>> mtWorkerPoolMTaskAssignments;\n");
      }
      fprintf(header, "int mtWorkerPoolCoarseRegionIndex;\n");
      fprintf(header, "int mtWorkerPoolCoarseLayerIndex;\n");
      fprintf(header, "bool mtCoarseUseMTaskRuntime;\n");
      // codegen-time LPT + flat per-cppId arrays.
      // SCoarseTaskFn points to the 2-arg mtTaskN overload (uint%d_t&,
      // ActivationDelta&). Keep mask before the member-function pointer;
      // cppId is uint32_t because XiangShan has >65535 emitted tasks.
      fprintf(header, "typedef void (S%s::*SCoarseTaskFn)(uint%d_t&, ActivationDelta&);\n", name.c_str(), ACTIVE_WIDTH);
      fprintf(header, "struct SCoarseTaskRef {\n");
      fprintf(header, "  uint32_t cppId;\n");
      fprintf(header, "  uint16_t wordOffset;\n");
      fprintf(header, "  uint8_t mergeAfter;\n");
      fprintf(header, "  uint8_t reserved;\n");
      fprintf(header, "  uint64_t mask;\n");
      fprintf(header, "  SCoarseTaskFn fn;\n");
      fprintf(header, "};\n");
      fprintf(header, "bool mtCoarseUseDStatic;\n");
      fprintf(header, "bool mtCoarseUseAntichainRuntime;\n");
      fprintf(header, "bool mtUseVerilatorDualPath;\n");
      fprintf(header, "int mtVerilatorDualPathRegionIndex;\n");
      fprintf(header, "int mtVerilatorDualPathBeginCppId;\n");
      fprintf(header, "int mtVerilatorDualPathEndCppId;\n");
      fprintf(header, "uint64_t mtProfileVerilatorDualPathDispatches;\n");
      fprintf(header, "uint64_t mtProfileVerilatorDualPathWorkerPoolDispatches;\n");
      fprintf(header, "int mtWorkerPoolCoarseStaticRoundedWC;\n");
      fprintf(header, "int mtWorkerPoolCoarseStaticBeginActiveWord;\n");
      fprintf(header, "int mtWorkerPoolCoarseStaticActiveWordSpan;\n");
      // per-mtask atomic counters and shared region flags for antichain runtime.
      // use a stamped claim-generation counter and even-cycle upstream target
      // so that neither state[] nor upstream[] need a per-invocation reset loop.
      fprintf(header, "std::vector<std::atomic<uint64_t>*> mtCoarseMTaskClaimGen;\n");
      fprintf(header, "std::vector<std::atomic<int>*> mtCoarseMTaskUpstream;\n");
      fprintf(header, "std::vector<int> mtCoarseMTaskCount;\n");
      fprintf(header, "std::vector<std::atomic<uint%d_t>*> mtCoarseRegionSharedFlags;\n", ACTIVE_WIDTH);
      fprintf(header, "alignas(64) std::atomic<int> mtCoarseMTaskRemaining;\n");
      fprintf(header, "std::vector<std::atomic<uint64_t>*> mtCoarseRegionCycle;\n");
      fprintf(header, "uint%d_t* mtWorkerPoolCoarseActiveWords;\n", ACTIVE_WIDTH);
      // mutex-protected ready queues for antichain scheduler.
      // Avoids O(M^2) scan/CAS by pushing ready mtasks once and popping once.
      // Kept behind GSIM_MT_ANTICHAIN_QUEUE env knob; old scan path still available.
      fprintf(header, "std::mutex mtCoarseReadyQueueMutex;\n");
      fprintf(header, "std::vector<std::vector<int>> mtCoarseReadyQueueParallel;\n");
      fprintf(header, "std::vector<std::vector<int>> mtCoarseReadyQueueWorker0;\n");
      fprintf(header, "alignas(64) std::atomic<int> mtCoarseMTaskInFlight;\n");
      fprintf(header, "bool mtCoarseUseAntichainQueue;\n");
    }
    fprintf(header, "int mtWorkerPoolJobKind;\n");
    fprintf(header, "void (S%s::*mtWorkerPoolDenseLayerFn)(int, int, int);\n", name.c_str());
    fprintf(header, "bool mtWorkerPoolEnabled;\n");
    fprintf(header, "bool mtWorkerPoolLazyStart;\n");
    fprintf(header, "int mtWorkerPoolThreadCount;\n");
    fprintf(header, "std::vector<std::thread> mtWorkerPoolThreads;\n");
    // hot atomics on independent cache lines.
    fprintf(header, "alignas(64) std::atomic<uint64_t> mtWorkerPoolGeneration;\n");
  if (mtUseWorkerPoolFlagJoinCodegen()) {
    fprintf(header, "#if defined(GSIM_MT_WORKER_POOL_FLAG_JOIN_COMPILE) && GSIM_MT_WORKER_POOL_FLAG_JOIN_COMPILE\n");
    fprintf(header, "struct alignas(64) MtWorkerPoolDoneFlag { std::atomic<uint8_t> parity{0}; };\n");
    fprintf(header, "static_assert(alignof(MtWorkerPoolDoneFlag) == 64 && sizeof(MtWorkerPoolDoneFlag) == 64, \"worker-pool done flags require one cache line per worker\");\n");
    fprintf(header, "static_assert(std::atomic<uint8_t>::is_always_lock_free, \"worker-pool done flag must be lock-free\");\n");
    fprintf(header, "MtWorkerPoolDoneFlag* mtWorkerPoolDoneFlags;\n");
    fprintf(header, "#else\n");
    fprintf(header, "alignas(64) std::atomic<int> mtWorkerPoolDoneCount;\n");
    fprintf(header, "#endif\n");
  } else {
    fprintf(header, "alignas(64) std::atomic<int> mtWorkerPoolDoneCount;\n");
  }
    fprintf(header, "alignas(64) std::atomic<int> mtWorkerPoolReadyCount;\n");
    fprintf(header, "alignas(64) std::atomic<bool> mtWorkerPoolStop;\n");
    fprintf(header, "alignas(64) int mtWorkerPoolCurrentWorkerCount;\n");
    fprintf(header, "std::vector<MtWorkerPoolChunk> mtWorkerPoolChunks;\n");
    fprintf(header, "std::vector<std::vector<int>> mtProfileLocalTaskIds;\n");
    fprintf(header, "std::vector<uint64_t> mtProfileLocalWorkerTaskCount;\n");
  }
#ifdef PERF
  fprintf(header, "size_t activeTimes[%d];\n", superId);
#if ENABLE_ACTIVATOR
  fprintf(header, "std::map<int, int>activator[%d];\n", superId);
#endif
  fprintf(header, "size_t validActive[%d];\n", superId);
  fprintf(header, "size_t nodeNum[%d];\n", superId);
#endif
  emitPrintf();
  /* constrcutor */
  emitFuncDecl(0, "S%s::S%s() {\n", name.c_str(), name.c_str());
  emitBodyLock(1, "cycles = 0;\n");
  emitBodyLock(1, "LOG_START = 1;\n");
  emitBodyLock(1, "LOG_END = 0;\n");
  emitBodyLock(1, "initMtProfile();\n");
  if (denseBreakdownProfileCodegen) emitBodyLock(1, "initMtDenseBreakdownProfile();\n");

  if (activationEventTraceCodegen) emitBodyLock(1, "initMtActivationEventTrace();\n");
  if (useMtHelpers) {
    if (denseExecutorValid) emitBodyLock(1, "if (!mtWorkerPoolLazyStart && !(!mtProfileEnabled && !mtUseDenseExecutor && mtConfiguredWorkerCount <= mtSparseSerialFastMaxWorkers)) startMtWorkerPool();\n");
    else emitBodyLock(1, "if (!mtWorkerPoolLazyStart && !(!mtProfileEnabled && mtConfiguredWorkerCount <= mtSparseSerialFastMaxWorkers)) startMtWorkerPool();\n");
  }
  emitBodyLock(1, "init();\n");
  emitBodyLock(0, "}\n");

  /* initialization */
  emitFuncDecl(0, "void S%s::init() {\n", name.c_str());
  emitBodyLock(1, "activateAll();\n");
#ifdef PERF
  emitBodyLock(1, "for (int i = 0; i < %d; i ++) activeTimes[i] = 0;\n", superId);
  #if ENABLE_ACTIVATOR
  emitBodyLock(1, "for (int i = 0; i < %d; i ++) activator[i] = std::map<int, int>();\n", superId);
  #endif
  for (SuperNode* super : sortedSuper) {
    if (super->cppId >= 0) {
      size_t num = 0;
      for (Node* member : super->member) {
        if (member->anyNextActive()) num ++;
      }
      emitBodyLock(1, "nodeNum[%d] = %ld; // memberNum=%ld\n", super->cppId, num, super->member.size());
    }
  }
  emitBodyLock(1, "for (int i = 0; i < %d; i ++) validActive[i] = 0;\n", superId);
#endif
  emitBodyLock(0, "#ifdef RANDOMIZE_INIT\n"
               "  srand((unsigned int)time(NULL));\n"
               "  for (uint32_t *p = &_var_start; p != &_var_end; p ++) {\n"
               "    *p = rand();\n"
               "  }\n"
               "// mask out the bits out of the width range\n");

  // header: node definition; src: node evaluation
  fprintf(header, "uint32_t _var_start;\n");
  // GSIM_EMIT_STATE_BY_OWNER=1 (default off, no effect when unset): reorder the
  // STATE-member declaration order so members whose owning mtask is assigned to
  // the same CCD are declared adjacently. C++ lays members out in declaration
  // order, so this groups each 8-core L3 domain's written state into contiguous
  // arenas, cutting cross-CCD line ownership transfer on written lines. Pure
  // layout: evaluation order and semantics are unchanged (the emission loop
  // below is the ONLY consumer of this order for declarations; evaluation code
  // references nodes by name).
  bool stateByOwner = false;
  { const char* e = std::getenv("GSIM_EMIT_STATE_BY_OWNER"); stateByOwner = e && e[0] && e[0] != '0'; }
  std::vector<SuperNode*> emitOrderSuper(sortedSuper.begin(), sortedSuper.end());
  if (stateByOwner && mtDenseSchedule.mtaskThreadAssign.size() == mtDenseSchedule.mtasks.size()) {
    // node -> ccd of the worker of the mtask containing it (first scc hit wins;
    // mtasks reference sccs, sccs reference nodes via the schedule)
    std::map<Node*, int> nodeCcd;
    const int ccdSize = 8;
    for (size_t mi = 0; mi < mtDenseSchedule.mtasks.size(); mi ++) {
      int w = mtDenseSchedule.mtaskThreadAssign[mi];
      if (w < 0) continue;
      int ccd = w / ccdSize;
      for (int scc : mtDenseSchedule.mtasks[mi].sccIds) {
        if (scc < 0 || scc >= (int)mtDenseSchedule.sccs.size()) continue;
        for (int cpp : mtDenseSchedule.sccs[(size_t)scc].cppIds) {
          if (cpp < 0 || cpp >= (int)supersrc.size()) continue;
          for (Node* n : supersrc[(size_t)cpp]->member) {
            if (n && nodeCcd.find(n) == nodeCcd.end()) nodeCcd[n] = ccd;
          }
        }
      }
    }
    std::vector<int> superCcd(emitOrderSuper.size(), 1 << 30);
    for (size_t i = 0; i < emitOrderSuper.size(); i ++) {
      for (Node* n : emitOrderSuper[i]->member) { auto it = nodeCcd.find(n); if (it != nodeCcd.end()) { superCcd[i] = it->second; break; } }
    }
    std::vector<size_t> orderIdx(emitOrderSuper.size());
    for (size_t i = 0; i < orderIdx.size(); i ++) orderIdx[i] = i;
    std::sort(orderIdx.begin(), orderIdx.end(), [&](size_t a, size_t b) {
      if (superCcd[a] != superCcd[b]) return superCcd[a] < superCcd[b];
      return a < b; // original-order tiebreak == stability
    });
    std::vector<SuperNode*> reordered(emitOrderSuper.size());
    for (size_t i = 0; i < orderIdx.size(); i ++) reordered[i] = emitOrderSuper[orderIdx[i]];
    emitOrderSuper.swap(reordered);
    // (dead lambda removed; index sort above is the stable reorder)
    fprintf(stderr, "[state-by-owner] member decl reordered by owner CCD (%zu supers, %zu attributed nodes)\n",
            emitOrderSuper.size(), nodeCcd.size());
  }
  for (SuperNode* super : emitOrderSuper) {
    // std::string insts;
    if (super->superType == SUPER_VALID || super->superType == SUPER_ASYNC_RESET) {
      for (Node* n : super->member) genNodeDef(header, n);
    }
    if (super->superType == SUPER_EXTMOD) {
      for (size_t i = 1; i < super->member.size(); i ++) genNodeDef(header, super->member[i]);
    }
  }
  /* memory definition */
  for (Node* mem : memory) genNodeDef(header, mem);
  fprintf(header, "uint32_t _var_end;\n");

  emitBodyLock(0, "// initialize registers with reset value 0 to overwrite the rand() results\n" );
  emitBodyLock(1, "memset(&_var_start, 0, &_var_end - &_var_start);\n");

  emitBodyLock(0, "#else\n" // RANDOMIZE_INIT
               "  memset(&_var_start, 0, &_var_end - &_var_start);\n"
               "#endif\n");

  fprintf(header, "S%s();\n", name.c_str());
  fprintf(header, "~S%s();\n", name.c_str());
  fprintf(header, "void init();\n");
  fprintf(header, "void initMtProfile();\n");
  if (denseBreakdownProfileCodegen) fprintf(header, "void initMtDenseBreakdownProfile();\n");
  if (denseBreakdownProfileCodegen) fprintf(header, "void dumpMtDenseBreakdownProfile();\n");

  fprintf(header, "void dumpMtProfile();\n");
  fprintf(header, "void recordMtProfileTask(int cppId, bool pureTask, uint64_t elapsedNs);\n");
  fprintf(header, "void dumpMtProfileDynamicTraceCycle();\n");
  fprintf(header, "void recordMtProfileDynamicTraceTask(int cppId);\n");
  fprintf(header, "void recordMtProfileWorkerTask(int worker);\n");
  if (activationEventTraceCodegen) {
    fprintf(header, "void initMtActivationEventTrace();\n");
    fprintf(header, "void recordMtActivationEvent(int32_t sourceCppId, uint32_t activeWordBase, uint64_t mask, MtActivationEventKind kind);\n");
    fprintf(header, "void beginMtActivationEventTraceCycle();\n");
    fprintf(header, "void flushMtActivationEventTraceCycle();\n");
    fprintf(header, "void closeMtActivationEventTrace();\n");
  }

  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::initMtProfile() {\n", name.c_str());
  emitBodyLock(1, "const char *profileEnv = getenv(\"GSIM_MT_PROFILE\");\n");
  emitBodyLock(1, "const char *fireProfileEnv = getenv(\"GSIM_MT_FIRE_PROFILE\");\n");
  emitBodyLock(1, "const char *dynamicTraceEnv = getenv(\"GSIM_MT_DYNAMIC_TRACE\");\n");
  if (mtUseDynamicStateTraceCodegen()) emitBodyLock(1, "const char *dynamicStateTraceEnv = getenv(\"GSIM_MT_DYNAMIC_STATE_TRACE\");\n");
  emitBodyLock(1, "bool dynamicTraceEnabled = dynamicTraceEnv != nullptr && dynamicTraceEnv[0] != '\\0';\n");
  emitBodyLock(1, "mtProfileEnabled = (profileEnv != nullptr && profileEnv[0] != '\\0' && profileEnv[0] != '0') || (fireProfileEnv != nullptr && fireProfileEnv[0] != '\\0' && fireProfileEnv[0] != '0') || dynamicTraceEnabled;\n");
  emitBodyLock(1, "mtProfileHelperMode = \"%s\";\n", globalConfig.MtHelperMode.c_str());
  if (denseExecutorValid) {
    emitBodyLock(1, "const char *mtExecutorEnv = getenv(\"GSIM_MT_EXECUTOR\");\n");
    emitBodyLock(1, "mtUseDenseExecutor = mtExecutorEnv != nullptr && mtExecutorEnv[0] == 'd' && mtExecutorEnv[1] == 'e' && mtExecutorEnv[2] == 'n' && mtExecutorEnv[3] == 's' && mtExecutorEnv[4] == 'e' && mtExecutorEnv[5] == '\\0';\n");
  }
  emitBodyLock(1, "const char *threadsEnv = getenv(\"GSIM_THREADS\");\n");
  emitBodyLock(1, "mtConfiguredWorkerCount = threadsEnv == nullptr ? 1 : atoi(threadsEnv);\n");
  emitBodyLock(1, "if (mtConfiguredWorkerCount < 1) mtConfiguredWorkerCount = 1;\n");
  emitBodyLock(1, "int mtMinBatchTasks = %d;\n", globalConfig.MtBatchFormationMode == "active-frequency" ? 2 : 16);
  emitBodyLock(1, "const char *minBatchEnv = getenv(\"GSIM_MT_MIN_BATCH_TASKS\");\n");
  emitBodyLock(1, "if (minBatchEnv != nullptr) mtMinBatchTasks = atoi(minBatchEnv);\n");
  emitBodyLock(1, "if (mtMinBatchTasks < 1) mtMinBatchTasks = 1;\n");
  emitBodyLock(1, "this->mtMinBatchTasks = mtMinBatchTasks;\n");
  emitBodyLock(1, "int mtSparseSerialFastMaxWorkers = 1;\n");
  emitBodyLock(1, "const char *sparseSerialFastEnv = getenv(\"GSIM_MT_SPARSE_SERIAL_FAST_MAX_WORKERS\");\n");
  emitBodyLock(1, "if (sparseSerialFastEnv != nullptr) mtSparseSerialFastMaxWorkers = atoi(sparseSerialFastEnv);\n");
  emitBodyLock(1, "if (mtSparseSerialFastMaxWorkers < 1) mtSparseSerialFastMaxWorkers = 1;\n");
  emitBodyLock(1, "this->mtSparseSerialFastMaxWorkers = mtSparseSerialFastMaxWorkers;\n");
  if (mtDenseDutyCodegen()) {
    emitBodyLock(1, "const char *dutyEnv = getenv(\"GSIM_MT_DENSE_DUTY\");\n");
    emitBodyLock(1, "mtDutyEnabled = dutyEnv != nullptr && dutyEnv[0] != '\\0' && dutyEnv[0] != '0';\n");
  }
  emitBodyLock(1, "int mtCoarseMinActiveBits = 0;\n");
  emitBodyLock(1, "const char *coarseMinActiveBitsEnv = getenv(\"GSIM_MT_COARSE_MIN_ACTIVE_BITS\");\n");
  emitBodyLock(1, "if (coarseMinActiveBitsEnv != nullptr) mtCoarseMinActiveBits = atoi(coarseMinActiveBitsEnv);\n");
  emitBodyLock(1, "if (mtCoarseMinActiveBits < 0) mtCoarseMinActiveBits = 0;\n");
  emitBodyLock(1, "this->mtCoarseMinActiveBits = mtCoarseMinActiveBits;\n");
  emitBodyLock(1, "int mtCoarseInlineThreshold = 1240;\n");
  emitBodyLock(1, "const char *coarseInlineThresholdEnv = getenv(\"GSIM_MT_COARSE_INLINE_THRESHOLD\");\n");
  emitBodyLock(1, "if (coarseInlineThresholdEnv != nullptr) mtCoarseInlineThreshold = atoi(coarseInlineThresholdEnv);\n");
  emitBodyLock(1, "if (mtCoarseInlineThreshold < 0) mtCoarseInlineThreshold = 0;\n");
  emitBodyLock(1, "this->mtCoarseInlineThreshold = mtCoarseInlineThreshold;\n");
  emitBodyLock(1, "bool mtCoarseSkeletalMode = false;\n");
  emitBodyLock(1, "const char *coarseSkeletalEnv = getenv(\"GSIM_MT_COARSE_SKELETAL\");\n");
  emitBodyLock(1, "if (coarseSkeletalEnv != nullptr && coarseSkeletalEnv[0] != '\\0' && coarseSkeletalEnv[0] != '0') mtCoarseSkeletalMode = true;\n");
  emitBodyLock(1, "this->mtCoarseSkeletalMode = mtCoarseSkeletalMode;\n");
  emitBodyLock(1, "mtProfileConfiguredWorkerCount = mtConfiguredWorkerCount;\n");
  emitBodyLock(1, "mtProfileMaxWorkerCount = 1;\n");
  emitBodyLock(1, "mtProfileActiveWordCount = 0;\n");
  emitBodyLock(1, "mtProfileSerialTasks = 0;\n");
  emitBodyLock(1, "mtProfilePureTasks = 0;\n");
  emitBodyLock(1, "mtProfilePureBatchCount = 0;\n");
  emitBodyLock(1, "mtProfileTrueParallelBatchCount = 0;\n");
  emitBodyLock(1, "mtProfileSkippedFakeParallelBatchCount = 0;\n");
  emitBodyLock(1, "mtProfileSerialFastTaskCount = 0;\n");
  emitBodyLock(1, "mtProfileActivationDeltaEntries = 0;\n");
  emitBodyLock(1, "mtProfileActivationDeltaMaxEntriesPerWorker = 0;\n");
  emitBodyLock(1, "mtProfileActivationDeltaActivateAllCount = 0;\n");
  emitBodyLock(1, "wallfracCommitCycles = 0; wallfracCombCycles = 0; wallfracCommitBrackets = 0; wallfracCombBrackets = 0;\n");
  emitBodyLock(1, "mtProfileRejectNotActiveWhole = 0;\n");
  emitBodyLock(1, "mtProfileRejectAlwaysActiveTask = 0;\n");
  emitBodyLock(1, "mtProfileRejectSerialTask = 0;\n");
  emitBodyLock(1, "mtProfileSafeSerialDispatched = 0;\n");      // 28c Phase 1A
  emitBodyLock(1, "mtProfileWorker0OnlyDispatched = 0;\n");
  emitBodyLock(1, "mtProfileRejectDependencyEdge = 0;\n");
  emitBodyLock(1, "mtProfileRejectSameActiveWordHazard = 0;\n");
  emitBodyLock(1, "mtProfileRejectBelowMinBatch = 0;\n");
  emitBodyLock(1, "mtProfileRejectConfiguredSingleWorker = 0;\n");
  emitBodyLock(1, "mtProfileBatchMemberNodeCount = 0;\n");
  emitBodyLock(1, "mtProfileSameActiveWordForwardEdges = 0;\n");
  emitBodyLock(1, "mtProfileCrossBatchActivationFanout = 0;\n");
  emitBodyLock(1, "mtProfileBatchWallNs = 0;\n");
  emitBodyLock(1, "mtProfileTrueParallelWallNs = 0;\n");
  emitBodyLock(1, "mtProfileSerialWallNs = 0;\n");
  emitBodyLock(1, "mtProfileMergeWallNs = 0;\n");
  emitBodyLock(1, "mtProfileTotalStepNs = 0;\n");
  emitBodyLock(1, "mtProfileDynamicTraceFile = nullptr;\n");
  emitBodyLock(1, "mtProfileDynamicTraceCycleStart = 0;\n");
  emitBodyLock(1, "mtProfileDynamicTraceCycleLimit = 0;\n");
  emitBodyLock(1, "mtProfileDynamicTraceTaskIds.clear();\n");
  emitBodyLock(1, "if (dynamicTraceEnabled) {\n");
  emitBodyLock(2, "const char *startEnv = getenv(\"GSIM_MT_DYNAMIC_TRACE_START\");\n");
  emitBodyLock(2, "const char *cyclesEnv = getenv(\"GSIM_MT_DYNAMIC_TRACE_CYCLES\");\n");
  emitBodyLock(2, "mtProfileDynamicTraceCycleStart = (startEnv != nullptr && startEnv[0] != '\\0') ? strtoull(startEnv, nullptr, 10) : 0;\n");
  emitBodyLock(2, "uint64_t traceCycleCount = (cyclesEnv != nullptr && cyclesEnv[0] != '\\0') ? strtoull(cyclesEnv, nullptr, 10) : 0;\n");
  emitBodyLock(2, "mtProfileDynamicTraceCycleLimit = mtProfileDynamicTraceCycleStart + traceCycleCount;\n");
  emitBodyLock(2, "if (traceCycleCount > 0 && mtProfileDynamicTraceCycleLimit > mtProfileDynamicTraceCycleStart) {\n");
  emitBodyLock(3, "mtProfileDynamicTraceFile = fopen(dynamicTraceEnv, \"w\");\n");
  emitBodyLock(3, "gAssert(mtProfileDynamicTraceFile != nullptr, \"failed to open GSIM_MT_DYNAMIC_TRACE=%%s\", dynamicTraceEnv);\n");
  emitBodyLock(3, "fprintf(mtProfileDynamicTraceFile, \"# gsim mt dynamic trace v1 start=%%lu cycles=%%lu\\n\", mtProfileDynamicTraceCycleStart, traceCycleCount);\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(1, "}\n");
  if (useCoarseMt) {
    // ready-queue state for antichain scheduler.
    emitBodyLock(1, "mtCoarseMTaskInFlight.store(0, std::memory_order_relaxed);\n");
    emitBodyLock(1, "const char *antichainQueueEnv = getenv(\"GSIM_MT_ANTICHAIN_QUEUE\");\n");
    emitBodyLock(1, "mtCoarseUseAntichainQueue = (antichainQueueEnv == nullptr) || (antichainQueueEnv[0] != '0');\n");
    emitBodyLock(1, "mtProfileCoarseStaticRuntimeEligibleRegions = %d;\n", mtCoarseProfileFacts.runtimeEligibleRegionCount);
    emitBodyLock(1, "mtCoarseReadyQueueParallel.assign((size_t)mtProfileCoarseStaticRuntimeEligibleRegions, std::vector<int>());\n");
    emitBodyLock(1, "mtCoarseReadyQueueWorker0.assign((size_t)mtProfileCoarseStaticRuntimeEligibleRegions, std::vector<int>());\n");
    // claim-generation + even-cycle arrays; cycle counters allocated per-region.
    emitBodyLock(1, "mtCoarseMTaskClaimGen.assign((size_t)mtProfileCoarseStaticRuntimeEligibleRegions, nullptr);\n");
    emitBodyLock(1, "mtCoarseMTaskUpstream.assign((size_t)mtProfileCoarseStaticRuntimeEligibleRegions, nullptr);\n");
    emitBodyLock(1, "mtCoarseRegionCycle.assign((size_t)mtProfileCoarseStaticRuntimeEligibleRegions, nullptr);\n");
    emitBodyLock(1, "mtCoarseMTaskCount.assign((size_t)mtProfileCoarseStaticRuntimeEligibleRegions, 0);\n");
    emitBodyLock(1, "mtCoarseRegionSharedFlags.assign((size_t)mtProfileCoarseStaticRuntimeEligibleRegions, nullptr);\n");
    emitBodyLock(1, "mtCoarseMTaskRemaining.store(0, std::memory_order_relaxed);\n");
    emitBodyLock(1, "mtWorkerPoolCoarseActiveWords = nullptr;\n");
    emitBodyLock(1, "mtProfileCoarseStaticLayerCount = %d;\n", mtCoarseProfileFacts.runtimeLayerCount);
    emitBodyLock(1, "mtProfileCoarseStaticMaxRegionLayerCount = %d;\n", mtCoarseProfileFacts.maxRegionLayerCount);
    emitBodyLock(1, "mtProfileCoarseStaticMTaskCount = %d;\n", mtCoarseProfileFacts.runtimeMTaskCount);
    emitBodyLock(1, "mtProfileCoarseRegionInvocations = 0;\n");
    emitBodyLock(1, "mtProfileCoarseAcceptedRegions = 0;\n");
    emitBodyLock(1, "mtProfileCoarseRejectedRegions = 0;\n");
    emitBodyLock(1, "mtProfileCoarseLayerDispatches = 0;\n");
    emitBodyLock(1, "mtProfileCoarseMTaskDispatches = 0;\n");
    emitBodyLock(1, "mtProfileCoarseAntichainDispatches = 0;\n");
    emitBodyLock(1, "mtProfileCoarseWorkerJobs = 0;\n");
    emitBodyLock(1, "mtProfileCoarseFlagWordCopies = 0;\n");
    emitBodyLock(1, "mtProfileCoarseMergeWordScans = 0;\n");
    emitBodyLock(1, "mtProfileCoarseActivationDeltaEntries = 0;\n");
    emitBodyLock(1, "mtProfileCoarseEstimatedBarrierCount = 0;\n");
    emitBodyLock(1, "mtProfileCoarseBodyNs = 0;\n");
    emitBodyLock(1, "mtProfileCoarseWaitNs = 0;\n");
    emitBodyLock(1, "mtProfileCoarseEstimatedUsefulWork = 0;\n");
    emitBodyLock(1, "mtProfileCoarseEstimatedRejectedUsefulWork = 0;\n");
    emitBodyLock(1, "mtProfileCoarseEstimatedOverheadWords = 0;\n");
    emitBodyLock(1, "mtProfileCoarseActiveMTaskCount = 0;\n");
    emitBodyLock(1, "mtProfileCoarseActiveMTaskStaticCost = 0;\n");
    emitBodyLock(1, "mtProfileCoarseAssignedStaticCost = 0;\n");
    emitBodyLock(1, "mtProfileCoarseWorstWorkerStaticCost = 0;\n");
    emitBodyLock(1, "mtProfileCoarseBestWorkerStaticCost = 0;\n");
    emitBodyLock(1, "mtProfileCoarseContiguousWorstStaticCost = 0;\n");
    emitBodyLock(1, "mtProfileCoarseBalancedWorstStaticCost = 0;\n");
    emitBodyLock(1, "mtProfileCoarseSerialFallbackEligible = 0;\n");
    emitBodyLock(1, "mtProfileCoarseSerialFallbackTaken = 0;\n");
    emitBodyLock(1, "mtProfileCoarseSerialFallbackActiveBits = 0;\n");
    emitBodyLock(1, "mtProfileCoarseSerialFallbackNonPureExcluded = 0;\n");
    emitBodyLock(1, "mtProfileCoarseSerialFallbackSavedWorkerJobs = 0;\n");
    emitBodyLock(1, "mtProfileCoarseSerialFallbackSavedFlagWordCopies = 0;\n");
    emitBodyLock(1, "mtProfileCoarseSerialFallbackSavedMergeWordScans = 0;\n");
    emitBodyLock(1, "mtProfileCoarseSerialFallbackSavedBarriers = 0;\n");
    emitBodyLock(1, "mtProfileCoarseSelectedWorkerCountHist.assign((size_t)mtProfileConfiguredWorkerCount + 1, 0);\n");
    emitBodyLock(1, "for (int i = 0; i < 6; i ++) mtProfileCoarseLayerSizeHist[i] = 0;\n");
    for (int i = 0; i < 6; i ++) {
      emitBodyLock(1, "mtProfileCoarseRegionLayerCountHist[%d] = %d;\n", i, mtCoarseProfileFacts.regionLayerCountHist[i]);
    }
  }
  emitBodyLock(1, "for (int i = 0; i < %d; i ++) { mtProfileTaskExecCount[i] = 0; mtProfileTaskWallNs[i] = 0; }\n", superId);
  emitBodyLock(1, "for (int i = 0; i < 6; i ++) mtProfileBatchSizeHist[i] = 0;\n");
  emitBodyLock(1, "mtProfileWorkerTaskCount.assign((size_t)mtProfileConfiguredWorkerCount, 0);\n");
  emitBodyLock(1, "mtProfileEffectiveWorkerCountHist.assign((size_t)mtProfileConfiguredWorkerCount + 1, 0);\n");
  if (useMtHelpers) {
    emitBodyLock(1, "const char *workerPoolEnv = getenv(\"GSIM_MT_WORKER_POOL\");\n");
    emitBodyLock(1, "mtWorkerPoolEnabled = workerPoolEnv == nullptr || workerPoolEnv[0] == '\\0' || workerPoolEnv[0] != '0';\n");
    emitBodyLock(1, "const char *workerPoolLazyEnv = getenv(\"GSIM_MT_LAZY_WORKER_POOL\");\n");
    emitBodyLock(1, "mtWorkerPoolLazyStart = workerPoolLazyEnv != nullptr && workerPoolLazyEnv[0] != '\\0' && workerPoolLazyEnv[0] != '0';\n");
    emitBodyLock(1, "mtWorkerPoolThreadCount = 0;\n");
    emitBodyLock(1, "mtWorkerPoolGeneration.store(0, std::memory_order_relaxed);\n");
    emitBodyLock(1, "mtWorkerPoolStop.store(false, std::memory_order_relaxed);\n");
  if (mtUseWorkerPoolFlagJoinCodegen()) {
    emitBodyLock(1, "#if defined(GSIM_MT_WORKER_POOL_FLAG_JOIN_COMPILE) && GSIM_MT_WORKER_POOL_FLAG_JOIN_COMPILE\n");
    emitBodyLock(1, "mtWorkerPoolDoneFlags = nullptr;\n");
    emitBodyLock(1, "#else\n");
    emitBodyLock(1, "mtWorkerPoolDoneCount.store(0, std::memory_order_relaxed);\n");
    emitBodyLock(1, "#endif\n");
  } else {
    emitBodyLock(1, "mtWorkerPoolDoneCount.store(0, std::memory_order_relaxed);\n");
  }
    emitBodyLock(1, "mtWorkerPoolReadyCount.store(0, std::memory_order_relaxed);\n");
    emitBodyLock(1, "mtWorkerPoolCurrentWorkerCount = 0;\n");
    emitBodyLock(1, "mtWorkerPoolJobKind = 0;\n");
    emitBodyLock(1, "mtWorkerPoolDenseLayerFn = nullptr;\n");
    if (useCoarseMt) {
      emitBodyLock(1, "mtWorkerPoolCoarseRegionIndex = -1;\n");
      emitBodyLock(1, "mtWorkerPoolCoarseLayerIndex = -1;\n");
      // mtask runtime for mt-level-dispatch; env GSIM_MT_COARSE_RUNTIME=layered overrides.
      if (globalConfig.MtHelperMode == "mt-level-dispatch") {
        emitBodyLock(1, "mtCoarseUseMTaskRuntime = true;\n");
      } else {
        emitBodyLock(1, "mtCoarseUseMTaskRuntime = %s;\n",
                     globalConfig.MtCoarseRuntimeMode == "mtask" ? "true" : "false");
      }
      emitBodyLock(1, "const char *coarseRuntimeEnv = getenv(\"GSIM_MT_COARSE_RUNTIME\");\n");
      emitBodyLock(1, "if (coarseRuntimeEnv != nullptr && coarseRuntimeEnv[0] != '\\0') {\n");
      emitBodyLock(2, "if (coarseRuntimeEnv[0] == 'l' || coarseRuntimeEnv[0] == 'L') mtCoarseUseMTaskRuntime = false;\n");
      emitBodyLock(2, "else if (coarseRuntimeEnv[0] == 'm' || coarseRuntimeEnv[0] == 'M') mtCoarseUseMTaskRuntime = true;\n");
      emitBodyLock(1, "}\n");
      // env GSIM_MT_COARSE_DSTATIC=0 disables the
      // codegen-time LPT + flat-array path so we can A/B without regenerating.
      emitBodyLock(1, "mtCoarseUseDStatic = true;\n");
      emitBodyLock(1, "const char *coarseDStaticEnv = getenv(\"GSIM_MT_COARSE_DSTATIC\");\n");
      emitBodyLock(1, "if (coarseDStaticEnv != nullptr && coarseDStaticEnv[0] != '\\0' && coarseDStaticEnv[0] == '0') mtCoarseUseDStatic = false;\n");
      emitBodyLock(1, "mtUseVerilatorDualPath = false;\n");
      emitBodyLock(1, "const char *verilatorDualPathEnv = getenv(\"GSIM_MT_VERILATOR_DUAL_PATH\");\n");
      emitBodyLock(1, "if (verilatorDualPathEnv != nullptr && verilatorDualPathEnv[0] != '\\0' && verilatorDualPathEnv[0] != '0') mtUseVerilatorDualPath = true;\n");
      emitBodyLock(1, "mtVerilatorDualPathRegionIndex = -1;\n");
      emitBodyLock(1, "mtVerilatorDualPathBeginCppId = 4488;\n");
      emitBodyLock(1, "mtVerilatorDualPathEndCppId = 5080;\n");
      emitBodyLock(1, "const char *verilatorDualRegionEnv = getenv(\"GSIM_MT_VERILATOR_DUAL_REGION\");\n");
      emitBodyLock(1, "if (verilatorDualRegionEnv != nullptr && verilatorDualRegionEnv[0] != '\\0') {\n");
      emitBodyLock(2, "const char *sep = strchr(verilatorDualRegionEnv, ':');\n");
      emitBodyLock(2, "if (sep == nullptr) sep = strchr(verilatorDualRegionEnv, '-');\n");
      emitBodyLock(2, "if (sep != nullptr) {\n");
      emitBodyLock(3, "mtVerilatorDualPathBeginCppId = atoi(verilatorDualRegionEnv);\n");
      emitBodyLock(3, "mtVerilatorDualPathEndCppId = atoi(sep + 1);\n");
      emitBodyLock(3, "mtVerilatorDualPathRegionIndex = -1;\n");
      emitBodyLock(2, "} else {\n");
      emitBodyLock(3, "mtVerilatorDualPathRegionIndex = atoi(verilatorDualRegionEnv);\n");
      emitBodyLock(3, "mtVerilatorDualPathBeginCppId = -1;\n");
      emitBodyLock(3, "mtVerilatorDualPathEndCppId = -1;\n");
      emitBodyLock(2, "}\n");
      emitBodyLock(1, "}\n");
      emitBodyLock(1, "mtProfileVerilatorDualPathDispatches = 0;\n");
      emitBodyLock(1, "mtProfileVerilatorDualPathWorkerPoolDispatches = 0;\n");
      // env GSIM_MT_ANTICHAIN_RUNTIME=1 enables per-mtask
      // atomic-counter scheduler for antichain-enabled coarse regions.
      emitBodyLock(1, "mtCoarseUseAntichainRuntime = false;\n");
      emitBodyLock(1, "const char *antichainRuntimeEnv = getenv(\"GSIM_MT_ANTICHAIN_RUNTIME\");\n");
      emitBodyLock(1, "if (antichainRuntimeEnv != nullptr && antichainRuntimeEnv[0] == '1') mtCoarseUseAntichainRuntime = true;\n");
      emitBodyLock(1, "mtWorkerPoolCoarseStaticRoundedWC = 0;\n");
      emitBodyLock(1, "mtWorkerPoolCoarseStaticBeginActiveWord = 0;\n");
      emitBodyLock(1, "mtWorkerPoolCoarseStaticActiveWordSpan = 0;\n");
    }
  }
  emitBodyLock(0, "}\n");
  if (denseBreakdownProfileCodegen) {
    emitFuncDecl(0, "void S%s::initMtDenseBreakdownProfile() {\n", name.c_str());
    emitBodyLock(1, "mtDenseBreakdownProfileEnabled = false;\n");
    emitBodyLock(1, "mtDenseBreakdownProfileOutPath[0] = '\\0';\n");
    emitBodyLock(1, "mtDenseBreakdownPoolIdleNs = 0;\n");
    emitBodyLock(1, "mtDenseBreakdownPoolDoneNs = 0;\n");
    emitBodyLock(1, "mtDenseBreakdownTotalStepNs = 0;\n");
    emitBodyLock(1, "for (int i = 0; i < kDenseBreakdownProfileWorkerCount; i++) { mtDenseBreakdownWorkers[i].dispatchSpanNs = 0; mtDenseBreakdownWorkers[i].blockedWaitNs = 0; mtDenseBreakdownWorkers[i].blockedWaitCount = 0; }\n");
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(1, "mtDenseBreakdownWindowEnabled = false;\n");
      emitBodyLock(1, "mtDenseBreakdownWindowWorker0BodyMode = false;\n");
      emitBodyLock(1, "mtDenseBreakdownWindowAllOwnerBodyMode = false;\n");
      emitBodyLock(1, "mtDenseBreakdownWindowFinishOnlyMode = false;\n");
      emitBodyLock(1, "mtDenseBreakdownWindowCausalChainMode = false;\n");
      emitBodyLock(1, "mtDenseBreakdownWindowCausalHotspotsMode = false;\n");
      emitBodyLock(1, "mtDenseBreakdownWindowCausalClockRegression = false;\n");
      emitBodyLock(1, "mtDenseBreakdownWindowCausalTimestampBoundUncertaintyNs = 0;\n");
      emitBodyLock(1, "for (int mtaskId = 0; mtaskId < kDenseBreakdownWindowMaxAllOwnerMTasks; mtaskId ++) mtDenseBreakdownWindowCausalVisitMarks[mtaskId] = 0;\n");
      emitBodyLock(1, "mtDenseBreakdownWindowOverflow.store(false, std::memory_order_relaxed);\n");
      emitBodyLock(1, "mtDenseBreakdownWindowStart = 0;\n");
      emitBodyLock(1, "mtDenseBreakdownWindowCycles = 0;\n");
      emitBodyLock(1, "mtDenseBreakdownWindowRecordedCycles = 0;\n");
      emitBodyLock(1, "mtDenseBreakdownWindowCurrentSlot = -1;\n");
      emitBodyLock(1, "mtDenseBreakdownWindowOutPath[0] = '\\0';\n");
    }
    emitBodyLock(1, "const char *mtDenseBreakdownProfileEnv = getenv(\"GSIM_MT_DENSE_BREAKDOWN_PROFILE\");\n");
    emitBodyLock(1, "if (mtDenseBreakdownProfileEnv == nullptr || mtDenseBreakdownProfileEnv[0] == '\\0' || mtDenseBreakdownProfileEnv[0] == '0') return;\n");
    emitBodyLock(1, "const char *mtDenseBreakdownProfileOutEnv = getenv(\"GSIM_MT_DENSE_BREAKDOWN_OUT\");\n");
    emitBodyLock(1, "gAssert(mtDenseBreakdownProfileOutEnv != nullptr && mtDenseBreakdownProfileOutEnv[0] != '\\0', \"GSIM_MT_DENSE_BREAKDOWN_OUT is required when GSIM_MT_DENSE_BREAKDOWN_PROFILE is enabled\");\n");
    emitBodyLock(1, "if (mtDenseBreakdownProfileOutEnv == nullptr || mtDenseBreakdownProfileOutEnv[0] == '\\0') abort();\n");
    emitBodyLock(1, "gAssert(mtConfiguredWorkerCount <= kDenseBreakdownProfileWorkerCount, \"GSIM_MT_DENSE_BREAKDOWN_PROFILE supports at most %%d workers (got %%d)\", kDenseBreakdownProfileWorkerCount, mtConfiguredWorkerCount);\n");
    emitBodyLock(1, "if (mtConfiguredWorkerCount > kDenseBreakdownProfileWorkerCount) abort();\n");
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(1, "gAssert(mtConfiguredWorkerCount == kDenseBreakdownWindowThreadCount, \"GSIM_MT_DENSE_BREAKDOWN window requires the codegen worker count %%d (got %%d)\", kDenseBreakdownWindowThreadCount, mtConfiguredWorkerCount);\n");
      emitBodyLock(1, "if (mtConfiguredWorkerCount != kDenseBreakdownWindowThreadCount) abort();\n");
    }
    emitBodyLock(1, "size_t mtDenseBreakdownProfileOutPathLen = strlen(mtDenseBreakdownProfileOutEnv);\n");
    emitBodyLock(1, "gAssert(mtDenseBreakdownProfileOutPathLen < sizeof(mtDenseBreakdownProfileOutPath), \"GSIM_MT_DENSE_BREAKDOWN_OUT path is too long\");\n");
    emitBodyLock(1, "if (mtDenseBreakdownProfileOutPathLen >= sizeof(mtDenseBreakdownProfileOutPath)) abort();\n");
    emitBodyLock(1, "FILE *mtDenseBreakdownProfileProbe = fopen(mtDenseBreakdownProfileOutEnv, \"wb\");\n");
    emitBodyLock(1, "gAssert(mtDenseBreakdownProfileProbe != nullptr, \"failed to open GSIM_MT_DENSE_BREAKDOWN_OUT=%%s\", mtDenseBreakdownProfileOutEnv);\n");
    emitBodyLock(1, "if (mtDenseBreakdownProfileProbe == nullptr) abort();\n");
    emitBodyLock(1, "if (fclose(mtDenseBreakdownProfileProbe) != 0) { fprintf(stderr, \"[mt-dense-breakdown] failed to close output probe path=%%s\\n\", mtDenseBreakdownProfileOutEnv); abort(); }\n");
    emitBodyLock(1, "memcpy(mtDenseBreakdownProfileOutPath, mtDenseBreakdownProfileOutEnv, mtDenseBreakdownProfileOutPathLen + 1);\n");
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(1, "const char *mtDenseBreakdownWindowStartEnv = getenv(\"GSIM_MT_DENSE_BREAKDOWN_WINDOW_START\");\n");
      emitBodyLock(1, "const char *mtDenseBreakdownWindowCyclesEnv = getenv(\"GSIM_MT_DENSE_BREAKDOWN_WINDOW_CYCLES\");\n");
      emitBodyLock(1, "const bool mtDenseBreakdownWindowHasStart = mtDenseBreakdownWindowStartEnv != nullptr && mtDenseBreakdownWindowStartEnv[0] != '\\0';\n");
      emitBodyLock(1, "const bool mtDenseBreakdownWindowHasCycles = mtDenseBreakdownWindowCyclesEnv != nullptr && mtDenseBreakdownWindowCyclesEnv[0] != '\\0';\n");
      emitBodyLock(1, "gAssert(mtDenseBreakdownWindowHasStart && mtDenseBreakdownWindowHasCycles, \"GSIM_MT_DENSE_BREAKDOWN_WINDOW_START and GSIM_MT_DENSE_BREAKDOWN_WINDOW_CYCLES are required together\");\n");
      emitBodyLock(1, "if (!mtDenseBreakdownWindowHasStart || !mtDenseBreakdownWindowHasCycles) abort();\n");
      emitBodyLock(1, "auto mtDenseBreakdownParseWindowUint = [](const char *text, uint64_t *value) -> bool {\n");
      emitBodyLock(2, "if (text == nullptr || text[0] == '\\0') return false;\n");
      emitBodyLock(2, "uint64_t parsed = 0;\n");
      emitBodyLock(2, "for (const unsigned char *p = (const unsigned char *)text; *p != 0; ++p) {\n");
      emitBodyLock(3, "if (*p < '0' || *p > '9') return false;\n");
      emitBodyLock(3, "uint64_t digit = (uint64_t)(*p - '0');\n");
      emitBodyLock(3, "if (parsed > (UINT64_MAX - digit) / 10) return false;\n");
      emitBodyLock(3, "parsed = parsed * 10 + digit;\n");
      emitBodyLock(2, "}\n");
      emitBodyLock(2, "*value = parsed;\n");
      emitBodyLock(2, "return true;\n");
      emitBodyLock(1, "};\n");
      emitBodyLock(1, "const bool mtDenseBreakdownWindowStartValid = mtDenseBreakdownParseWindowUint(mtDenseBreakdownWindowStartEnv, &mtDenseBreakdownWindowStart);\n");
      emitBodyLock(1, "const bool mtDenseBreakdownWindowCyclesValid = mtDenseBreakdownParseWindowUint(mtDenseBreakdownWindowCyclesEnv, &mtDenseBreakdownWindowCycles);\n");
      emitBodyLock(1, "gAssert(mtDenseBreakdownWindowStartValid && mtDenseBreakdownWindowCyclesValid && mtDenseBreakdownWindowCycles >= 1 && mtDenseBreakdownWindowCycles <= kDenseBreakdownWindowMaxCycles, \"GSIM_MT_DENSE_BREAKDOWN window requires an unsigned start and 1..%%d cycles\", kDenseBreakdownWindowMaxCycles);\n");
      emitBodyLock(1, "if (!mtDenseBreakdownWindowStartValid || !mtDenseBreakdownWindowCyclesValid || mtDenseBreakdownWindowCycles < 1 || mtDenseBreakdownWindowCycles > kDenseBreakdownWindowMaxCycles) abort();\n");
      emitBodyLock(1, "const char *mtDenseBreakdownWindowModeEnv = getenv(\"GSIM_MT_DENSE_BREAKDOWN_WINDOW_MODE\");\n");
      emitBodyLock(1, "if (mtDenseBreakdownWindowModeEnv == nullptr || mtDenseBreakdownWindowModeEnv[0] == '\\0' || strcmp(mtDenseBreakdownWindowModeEnv, \"criticality\") == 0) {\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker0BodyMode = false;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowAllOwnerBodyMode = false;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowFinishOnlyMode = false;\n");
      emitBodyLock(1, "} else if (strcmp(mtDenseBreakdownWindowModeEnv, \"worker0body\") == 0) {\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker0BodyMode = true;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowAllOwnerBodyMode = false;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowFinishOnlyMode = false;\n");
      emitBodyLock(1, "} else if (strcmp(mtDenseBreakdownWindowModeEnv, \"allownerbody\") == 0) {\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker0BodyMode = false;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowAllOwnerBodyMode = true;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowFinishOnlyMode = false;\n");
      emitBodyLock(1, "} else if (strcmp(mtDenseBreakdownWindowModeEnv, \"finishonly\") == 0) {\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker0BodyMode = false;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowAllOwnerBodyMode = false;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowFinishOnlyMode = true;\n");
      emitBodyLock(1, "} else if (strcmp(mtDenseBreakdownWindowModeEnv, \"causalhotspots\") == 0) {\n");
      emitBodyLock(2, "#if !defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) || !GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
      emitBodyLock(2, "gAssert(false, \"causalhotspots requires GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\");\n");
      emitBodyLock(2, "abort();\n");
      emitBodyLock(2, "#endif\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker0BodyMode = false;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowAllOwnerBodyMode = false;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowFinishOnlyMode = false;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowCausalChainMode = true;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowCausalHotspotsMode = true;\n");
      emitBodyLock(2, "for (int mtaskId = 0; mtaskId < kDenseBreakdownWindowAllOwnerMTaskCount; mtaskId ++) { mtDenseBreakdownWindowCausalHotspotMTasks[mtaskId] = {}; mtDenseBreakdownWindowCausalHotspotSameOwnerEdges[mtaskId] = {}; }\n");
      emitBodyLock(2, "for (int token = 0; token < kDenseBreakdownWindowCausalTokenCount; token ++) mtDenseBreakdownWindowCausalHotspotRemoteTokenEdges[token] = {};\n");
      emitBodyLock(2, "mtDenseBreakdownWindowCausalHotspotTotals = {};\n");
      emitBodyLock(1, "} else if (strcmp(mtDenseBreakdownWindowModeEnv, \"causalchain\") == 0) {\n");
      emitBodyLock(2, "#if !defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) || !GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
      emitBodyLock(2, "gAssert(false, \"causalchain requires GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\");\n");
      emitBodyLock(2, "abort();\n");
      emitBodyLock(2, "#endif\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker0BodyMode = false;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowAllOwnerBodyMode = false;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowFinishOnlyMode = false;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowCausalChainMode = true;\n");
      emitBodyLock(1, "} else {\n");
      emitBodyLock(2, "gAssert(false, \"GSIM_MT_DENSE_BREAKDOWN_WINDOW_MODE must be finishonly, criticality, worker0body, allownerbody, causalchain, or causalhotspots\");\n");
      emitBodyLock(2, "abort();\n");
      emitBodyLock(1, "}\n");
      emitBodyLock(1, "static const char mtDenseBreakdownWindowSuffix[] = \".window.json\";\n");
      emitBodyLock(1, "gAssert(mtDenseBreakdownProfileOutPathLen + sizeof(mtDenseBreakdownWindowSuffix) <= sizeof(mtDenseBreakdownWindowOutPath), \"GSIM_MT_DENSE_BREAKDOWN_OUT path is too long for window sibling\");\n");
      emitBodyLock(1, "if (mtDenseBreakdownProfileOutPathLen + sizeof(mtDenseBreakdownWindowSuffix) > sizeof(mtDenseBreakdownWindowOutPath)) abort();\n");
      emitBodyLock(1, "memcpy(mtDenseBreakdownWindowOutPath, mtDenseBreakdownProfileOutPath, mtDenseBreakdownProfileOutPathLen);\n");
      emitBodyLock(1, "memcpy(mtDenseBreakdownWindowOutPath + mtDenseBreakdownProfileOutPathLen, mtDenseBreakdownWindowSuffix, sizeof(mtDenseBreakdownWindowSuffix));\n");
      emitBodyLock(1, "mtDenseBreakdownWindowEnabled = true;\n");
    }
    emitBodyLock(1, "mtDenseBreakdownProfileEnabled = true;\n");
    emitBodyLock(0, "}\n");

    emitFuncDecl(0, "void S%s::dumpMtDenseBreakdownProfile() {\n", name.c_str());
    emitBodyLock(1, "if (!mtDenseBreakdownProfileEnabled) return;\n");
    emitBodyLock(1, "FILE *mtDenseBreakdownProfileFile = fopen(mtDenseBreakdownProfileOutPath, \"wb\");\n");
    emitBodyLock(1, "gAssert(mtDenseBreakdownProfileFile != nullptr, \"failed to open GSIM_MT_DENSE_BREAKDOWN_OUT=%%s\", mtDenseBreakdownProfileOutPath);\n");
    emitBodyLock(1, "if (mtDenseBreakdownProfileFile == nullptr) abort();\n");
    emitBodyLock(1, "int mtDenseBreakdownProfileWriteError = 0;\n");
    emitBodyLock(1, "int mtDenseBreakdownProfileWorkerCount = mtConfiguredWorkerCount;\n");
    emitBodyLock(1, "if (mtDenseBreakdownProfileWorkerCount < 1) mtDenseBreakdownProfileWorkerCount = 1;\n");
    emitBodyLock(1, "if (mtDenseBreakdownProfileWorkerCount > kDenseBreakdownProfileWorkerCount) { fclose(mtDenseBreakdownProfileFile); abort(); }\n");
    emitBodyLock(1, "if (fprintf(mtDenseBreakdownProfileFile, \"{\\\"magic\\\":\\\"GSIM_MT_DENSE_BREAKDOWN\\\",\\\"version\\\":1,\\\"threadCount\\\":%%d,\\\"cycles\\\":%%llu,\\\"totalStepNs\\\":%%llu,\\\"poolIdleNs\\\":%%llu,\\\"poolDoneNs\\\":%%llu,\\\"workers\\\":[\", mtDenseBreakdownProfileWorkerCount, (unsigned long long)cycles, (unsigned long long)mtDenseBreakdownTotalStepNs, (unsigned long long)mtDenseBreakdownPoolIdleNs, (unsigned long long)mtDenseBreakdownPoolDoneNs) < 0) mtDenseBreakdownProfileWriteError = 1;\n");
    emitBodyLock(1, "for (int i = 0; i < mtDenseBreakdownProfileWorkerCount; i++) {\n");
    emitBodyLock(2, "if (i != 0 && fputc(',', mtDenseBreakdownProfileFile) == EOF) mtDenseBreakdownProfileWriteError = 1;\n");
    emitBodyLock(2, "MtDenseBreakdownWorker &mtDenseBreakdownWorker = mtDenseBreakdownWorkers[i];\n");
    emitBodyLock(2, "if (fprintf(mtDenseBreakdownProfileFile, \"{\\\"dispatchSpanNs\\\":%%llu,\\\"blockedWaitNs\\\":%%llu,\\\"blockedWaitCount\\\":%%llu}\", (unsigned long long)mtDenseBreakdownWorker.dispatchSpanNs, (unsigned long long)mtDenseBreakdownWorker.blockedWaitNs, (unsigned long long)mtDenseBreakdownWorker.blockedWaitCount) < 0) mtDenseBreakdownProfileWriteError = 1;\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(1, "if (fprintf(mtDenseBreakdownProfileFile, \"]}\\n\") < 0) mtDenseBreakdownProfileWriteError = 1;\n");
    emitBodyLock(1, "if (fflush(mtDenseBreakdownProfileFile) != 0) mtDenseBreakdownProfileWriteError = 1;\n");
    emitBodyLock(1, "if (fclose(mtDenseBreakdownProfileFile) != 0) mtDenseBreakdownProfileWriteError = 1;\n");
    emitBodyLock(1, "if (mtDenseBreakdownProfileWriteError) { fprintf(stderr, \"[mt-dense-breakdown] failed to write output path=%%s\\n\", mtDenseBreakdownProfileOutPath); abort(); }\n");
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(1, "if (mtDenseBreakdownWindowEnabled) {\n");
      emitBodyLock(2, "FILE *mtDenseBreakdownWindowFile = fopen(mtDenseBreakdownWindowOutPath, \"wb\");\n");
      emitBodyLock(2, "gAssert(mtDenseBreakdownWindowFile != nullptr, \"failed to open dense breakdown window output=%%s\", mtDenseBreakdownWindowOutPath);\n");
      emitBodyLock(2, "if (mtDenseBreakdownWindowFile == nullptr) abort();\n");
      emitBodyLock(2, "int mtDenseBreakdownWindowWriteError = 0;\n");
      emitBodyLock(2, "const bool mtDenseBreakdownWindowOverflowed = mtDenseBreakdownWindowOverflow.load(std::memory_order_relaxed);\n");
      emitBodyLock(2, "const bool mtDenseBreakdownWindowComplete = !mtDenseBreakdownWindowOverflowed && mtDenseBreakdownWindowRecordedCycles == mtDenseBreakdownWindowCycles;\n");
      emitBodyLock(2, "const char *mtDenseBreakdownWindowMode = mtDenseBreakdownWindowCausalHotspotsMode ? \"causalhotspots\" : (mtDenseBreakdownWindowCausalChainMode ? \"causalchain\" : (mtDenseBreakdownWindowFinishOnlyMode ? \"finishonly\" : (mtDenseBreakdownWindowAllOwnerBodyMode ? \"allownerbody\" : (mtDenseBreakdownWindowWorker0BodyMode ? \"worker0body\" : \"criticality\"))));\n");
      emitBodyLock(2, "if (mtDenseBreakdownWindowCausalHotspotsMode) {\n");
      emitBodyLock(3, "bool mtDenseBreakdownWindowCausalMappingComplete = !mtDenseBreakdownWindowOverflowed && !mtDenseBreakdownWindowCausalClockRegression && mtDenseBreakdownWindowRecordedCycles == mtDenseBreakdownWindowCycles;\n");
      emitBodyLock(3, "for (uint64_t c = 0; c < mtDenseBreakdownWindowRecordedCycles; c ++) { const MtDenseBreakdownWindowCausalSummary &mtDenseBreakdownWindowCausalSummary = mtDenseBreakdownWindowCausalSummaries[c]; if (!mtDenseBreakdownWindowCausalSummary.complete || mtDenseBreakdownWindowCausalSummary.incompleteMapping || mtDenseBreakdownWindowCausalSummary.clockRegression || mtDenseBreakdownWindowCausalSummary.overflow) mtDenseBreakdownWindowCausalMappingComplete = false; }\n");
      emitBodyLock(3, "const bool mtDenseBreakdownWindowCausalHotspotComplete = mtDenseBreakdownWindowCausalMappingComplete;\n");
      emitBodyLock(3, "MtDenseBreakdownWindowCausalHotspotTotals mtDenseBreakdownWindowCausalHotspotMTaskTotals = {};\n");
      emitBodyLock(3, "MtDenseBreakdownWindowCausalHotspotTotals mtDenseBreakdownWindowCausalHotspotEdgeTotals = {};\n");
      emitBodyLock(3, "MtDenseBreakdownWindowCausalHotspotTotals mtDenseBreakdownWindowCausalHotspotSummaryTotals = {};\n");
      emitBodyLock(3, "if (unlikely(mtDenseBreakdownWindowCausalHotspotTotals.sameOwnerPredCount > mtDenseBreakdownWindowCausalHotspotTotals.edgeCount || mtDenseBreakdownWindowCausalHotspotTotals.remoteTokenPredCount != mtDenseBreakdownWindowCausalHotspotTotals.edgeCount - mtDenseBreakdownWindowCausalHotspotTotals.sameOwnerPredCount)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots dump edge-kind reconciliation failed\\n\"); abort(); }\n");
      emitBodyLock(3, "auto mtDenseBreakdownWindowCausalHotspotsDumpAdd = [&](uint64_t &target, uint64_t value) { if (unlikely(UINT64_MAX - target < value)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots dump aggregate overflow\\n\"); abort(); } target += value; };\n");
      emitBodyLock(3, "for (uint64_t c = 0; c < mtDenseBreakdownWindowRecordedCycles; c ++) { const MtDenseBreakdownWindowCausalSummary &mtDenseBreakdownWindowCausalSummary = mtDenseBreakdownWindowCausalSummaries[c]; mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotSummaryTotals.sameOwnerPredCount, mtDenseBreakdownWindowCausalSummary.sameOwnerPredecessorCount); mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotSummaryTotals.remoteTokenPredCount, mtDenseBreakdownWindowCausalSummary.remoteTokenPredecessorCount); if (unlikely((uint64_t)mtDenseBreakdownWindowCausalSummary.sameOwnerPredecessorCount > mtDenseBreakdownWindowCausalSummary.chainEdgeCount || (uint64_t)mtDenseBreakdownWindowCausalSummary.remoteTokenPredecessorCount != (uint64_t)mtDenseBreakdownWindowCausalSummary.chainEdgeCount - mtDenseBreakdownWindowCausalSummary.sameOwnerPredecessorCount || mtDenseBreakdownWindowCausalSummary.remoteTokenPredecessorCount > mtDenseBreakdownWindowCausalSummary.waitObservationCount)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots cycle edge-kind counters corrupt\\n\"); abort(); } }\n");
      emitBodyLock(3, "if (unlikely(mtDenseBreakdownWindowCausalHotspotSummaryTotals.sameOwnerPredCount != mtDenseBreakdownWindowCausalHotspotTotals.sameOwnerPredCount || mtDenseBreakdownWindowCausalHotspotSummaryTotals.remoteTokenPredCount != mtDenseBreakdownWindowCausalHotspotTotals.remoteTokenPredCount)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots summary edge-kind reconciliation failed\\n\"); abort(); }\n");
      emitBodyLock(3, "for (int mtaskId = 0; mtaskId < kDenseBreakdownWindowAllOwnerMTaskCount; mtaskId ++) { const MtDenseBreakdownWindowCausalHotspotMTask &mtDenseBreakdownWindowCausalHotspotMTask = mtDenseBreakdownWindowCausalHotspotMTasks[mtaskId]; if (unlikely(mtDenseBreakdownWindowCausalHotspotMTask.sameOwnerPredCount > mtDenseBreakdownWindowCausalHotspotMTask.count || mtDenseBreakdownWindowCausalHotspotMTask.remoteTokenPredCount > mtDenseBreakdownWindowCausalHotspotMTask.count - mtDenseBreakdownWindowCausalHotspotMTask.sameOwnerPredCount || (mtDenseBreakdownWindowCausalHotspotMTask.count == 0 && (mtDenseBreakdownWindowCausalHotspotMTask.bodyNs != 0 || mtDenseBreakdownWindowCausalHotspotMTask.releaseNs != 0 || mtDenseBreakdownWindowCausalHotspotMTask.gapNs != 0 || mtDenseBreakdownWindowCausalHotspotMTask.latestTailCount != 0)))) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots MTask aggregate corrupt\\n\"); abort(); } mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotMTaskTotals.count, mtDenseBreakdownWindowCausalHotspotMTask.count); mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotMTaskTotals.bodyNs, mtDenseBreakdownWindowCausalHotspotMTask.bodyNs); mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotMTaskTotals.releaseNs, mtDenseBreakdownWindowCausalHotspotMTask.releaseNs); mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotMTaskTotals.gapNs, mtDenseBreakdownWindowCausalHotspotMTask.gapNs); mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotMTaskTotals.sameOwnerPredCount, mtDenseBreakdownWindowCausalHotspotMTask.sameOwnerPredCount); mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotMTaskTotals.remoteTokenPredCount, mtDenseBreakdownWindowCausalHotspotMTask.remoteTokenPredCount); mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotMTaskTotals.latestTailCount, mtDenseBreakdownWindowCausalHotspotMTask.latestTailCount); }\n");
      emitBodyLock(3, "for (int index = 0; index < kDenseBreakdownWindowCausalTokenCount; index ++) { const int token = kDenseBreakdownWindowCausalHotspotRemoteTokenOutputOrder[index]; if (unlikely(token < 0 || token >= kDenseBreakdownWindowCausalTokenCount)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots remote output map invalid\\n\"); abort(); } const MtDenseBreakdownWindowCausalHotspotEdge &mtDenseBreakdownWindowCausalHotspotEdge = mtDenseBreakdownWindowCausalHotspotRemoteTokenEdges[token]; if (unlikely(mtDenseBreakdownWindowCausalHotspotEdge.count == 0 && mtDenseBreakdownWindowCausalHotspotEdge.gapNs != 0)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots remote aggregate corrupt\\n\"); abort(); } mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotEdgeTotals.edgeCount, mtDenseBreakdownWindowCausalHotspotEdge.count); mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotEdgeTotals.gapNs, mtDenseBreakdownWindowCausalHotspotEdge.gapNs); mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotEdgeTotals.remoteTokenPredCount, mtDenseBreakdownWindowCausalHotspotEdge.count); }\n");
      emitBodyLock(3, "for (int index = 0; index < kDenseBreakdownWindowAllOwnerMTaskCount; index ++) { const int consumerMTask = kDenseBreakdownWindowCausalHotspotSameOwnerConsumerOutputOrder[index]; if (unlikely(consumerMTask < 0 || consumerMTask >= kDenseBreakdownWindowAllOwnerMTaskCount)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots same-owner output map invalid\\n\"); abort(); } const int producerMTask = kDenseBreakdownWindowCausalSameOwnerProducerMTask[consumerMTask]; const MtDenseBreakdownWindowCausalHotspotEdge &mtDenseBreakdownWindowCausalHotspotEdge = mtDenseBreakdownWindowCausalHotspotSameOwnerEdges[consumerMTask]; if (unlikely((mtDenseBreakdownWindowCausalHotspotEdge.count == 0 && mtDenseBreakdownWindowCausalHotspotEdge.gapNs != 0) || (mtDenseBreakdownWindowCausalHotspotEdge.count != 0 && (producerMTask < 0 || producerMTask >= consumerMTask)))) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots same-owner aggregate corrupt\\n\"); abort(); } mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotEdgeTotals.edgeCount, mtDenseBreakdownWindowCausalHotspotEdge.count); mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotEdgeTotals.gapNs, mtDenseBreakdownWindowCausalHotspotEdge.gapNs); mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotEdgeTotals.sameOwnerPredCount, mtDenseBreakdownWindowCausalHotspotEdge.count); }\n");
      emitBodyLock(3, "for (uint64_t c = 0; c < mtDenseBreakdownWindowRecordedCycles; c ++) { const MtDenseBreakdownWindowCausalSummary &mtDenseBreakdownWindowCausalSummary = mtDenseBreakdownWindowCausalSummaries[c]; mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotSummaryTotals.count, mtDenseBreakdownWindowCausalSummary.chainNodeCount); mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotSummaryTotals.bodyNs, mtDenseBreakdownWindowCausalSummary.chainBodyNs); mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotSummaryTotals.releaseNs, mtDenseBreakdownWindowCausalSummary.chainReleaseNs); mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotSummaryTotals.gapNs, mtDenseBreakdownWindowCausalSummary.chainGapNs); mtDenseBreakdownWindowCausalHotspotsDumpAdd(mtDenseBreakdownWindowCausalHotspotSummaryTotals.edgeCount, mtDenseBreakdownWindowCausalSummary.chainEdgeCount); }\n");
      emitBodyLock(3, "if (unlikely(mtDenseBreakdownWindowCausalHotspotMTaskTotals.count != mtDenseBreakdownWindowCausalHotspotTotals.count || mtDenseBreakdownWindowCausalHotspotMTaskTotals.bodyNs != mtDenseBreakdownWindowCausalHotspotTotals.bodyNs || mtDenseBreakdownWindowCausalHotspotMTaskTotals.releaseNs != mtDenseBreakdownWindowCausalHotspotTotals.releaseNs || mtDenseBreakdownWindowCausalHotspotMTaskTotals.gapNs != mtDenseBreakdownWindowCausalHotspotTotals.gapNs || mtDenseBreakdownWindowCausalHotspotMTaskTotals.sameOwnerPredCount != mtDenseBreakdownWindowCausalHotspotTotals.sameOwnerPredCount || mtDenseBreakdownWindowCausalHotspotMTaskTotals.remoteTokenPredCount != mtDenseBreakdownWindowCausalHotspotTotals.remoteTokenPredCount || mtDenseBreakdownWindowCausalHotspotMTaskTotals.latestTailCount != mtDenseBreakdownWindowCausalHotspotTotals.latestTailCount || mtDenseBreakdownWindowCausalHotspotEdgeTotals.edgeCount != mtDenseBreakdownWindowCausalHotspotTotals.edgeCount || mtDenseBreakdownWindowCausalHotspotEdgeTotals.gapNs != mtDenseBreakdownWindowCausalHotspotTotals.gapNs || mtDenseBreakdownWindowCausalHotspotEdgeTotals.sameOwnerPredCount != mtDenseBreakdownWindowCausalHotspotTotals.sameOwnerPredCount || mtDenseBreakdownWindowCausalHotspotEdgeTotals.remoteTokenPredCount != mtDenseBreakdownWindowCausalHotspotTotals.remoteTokenPredCount || mtDenseBreakdownWindowCausalHotspotSummaryTotals.count != mtDenseBreakdownWindowCausalHotspotTotals.count || mtDenseBreakdownWindowCausalHotspotSummaryTotals.bodyNs != mtDenseBreakdownWindowCausalHotspotTotals.bodyNs || mtDenseBreakdownWindowCausalHotspotSummaryTotals.releaseNs != mtDenseBreakdownWindowCausalHotspotTotals.releaseNs || mtDenseBreakdownWindowCausalHotspotSummaryTotals.gapNs != mtDenseBreakdownWindowCausalHotspotTotals.gapNs || mtDenseBreakdownWindowCausalHotspotSummaryTotals.edgeCount != mtDenseBreakdownWindowCausalHotspotTotals.edgeCount || mtDenseBreakdownWindowCausalHotspotTotals.sameOwnerPredCount > mtDenseBreakdownWindowCausalHotspotTotals.edgeCount || mtDenseBreakdownWindowCausalHotspotTotals.remoteTokenPredCount > mtDenseBreakdownWindowCausalHotspotTotals.edgeCount - mtDenseBreakdownWindowCausalHotspotTotals.sameOwnerPredCount || mtDenseBreakdownWindowCausalHotspotTotals.latestTailCount != mtDenseBreakdownWindowRecordedCycles)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots aggregate reconciliation failed\\n\"); abort(); }\n");
      emitBodyLock(3, "if (fprintf(mtDenseBreakdownWindowFile, \"{\\\"magic\\\":\\\"GSIM_MT_DENSE_BREAKDOWN_WINDOW\\\",\\\"version\\\":2,\\\"window_start\\\":%%llu,\\\"window_cycles\\\":%%llu,\\\"accepted_cycles\\\":%%llu,\\\"threadCount\\\":%%d,\\\"mode\\\":\\\"causalhotspots\\\",\\\"criticality_offsets_valid\\\":false,\\\"worker0_mtask_count\\\":0,\\\"allowner_mtask_count\\\":%%d,\\\"overflow\\\":%%s,\\\"complete\\\":%%s,\\\"causalchain_mapping_complete\\\":%%s,\\\"causalchain_timestamp_bound_uncertainty\\\":true,\\\"causalchain_timestamp_bound_uncertainty_ns\\\":%%llu,\\\"clock_regression\\\":%%s,\\\"cycles\\\":[\", (unsigned long long)mtDenseBreakdownWindowStart, (unsigned long long)mtDenseBreakdownWindowCycles, (unsigned long long)mtDenseBreakdownWindowRecordedCycles, mtDenseBreakdownProfileWorkerCount, kDenseBreakdownWindowAllOwnerMTaskCount, mtDenseBreakdownWindowOverflowed ? \"true\" : \"false\", mtDenseBreakdownWindowCausalHotspotComplete ? \"true\" : \"false\", mtDenseBreakdownWindowCausalMappingComplete ? \"true\" : \"false\", (unsigned long long)mtDenseBreakdownWindowCausalTimestampBoundUncertaintyNs, mtDenseBreakdownWindowCausalClockRegression ? \"true\" : \"false\") < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(3, "for (uint64_t c = 0; c < mtDenseBreakdownWindowRecordedCycles; c ++) { if (c != 0 && fputc(',', mtDenseBreakdownWindowFile) == EOF) mtDenseBreakdownWindowWriteError = 1; const MtDenseBreakdownWindowCausalSummary &mtDenseBreakdownWindowCausalSummary = mtDenseBreakdownWindowCausalSummaries[c]; if (fprintf(mtDenseBreakdownWindowFile, \"{\\\"cycle\\\":%%llu,\\\"causalSummary\\\":{\\\"complete\\\":%%s,\\\"causal_edges\\\":%%u,\\\"max_predecessor_end_ns\\\":%%llu,\\\"max_lag_ns\\\":%%llu,\\\"sameOwnerPredecessorCount\\\":%%u,\\\"remoteTokenPredecessorCount\\\":%%u,\\\"waitObservationCount\\\":%%u,\\\"causalBoundNs\\\":%%llu,\\\"chain_node_count\\\":%%u,\\\"chain_edge_count\\\":%%u,\\\"latest_owner\\\":%%d,\\\"latest_owner_finish_ns\\\":%%llu,\\\"makespan_ns\\\":%%llu,\\\"chain_body_ns\\\":%%llu,\\\"chain_release_ns\\\":%%llu,\\\"chain_gap_ns\\\":%%llu,\\\"timestampBoundUncertainty\\\":true,\\\"timestampBoundUncertaintyNs\\\":%%llu,\\\"incompleteMapping\\\":%%s,\\\"clockRegression\\\":%%s,\\\"overflow\\\":%%s}}\", (unsigned long long)mtDenseBreakdownWindowCycleNumbers[c], mtDenseBreakdownWindowCausalSummary.complete ? \"true\" : \"false\", mtDenseBreakdownWindowCausalSummary.chainEdgeCount, (unsigned long long)mtDenseBreakdownWindowCausalSummary.causalBoundNs, (unsigned long long)mtDenseBreakdownWindowCausalSummary.maxLagNs, mtDenseBreakdownWindowCausalSummary.sameOwnerPredecessorCount, mtDenseBreakdownWindowCausalSummary.remoteTokenPredecessorCount, mtDenseBreakdownWindowCausalSummary.waitObservationCount, (unsigned long long)mtDenseBreakdownWindowCausalSummary.causalBoundNs, mtDenseBreakdownWindowCausalSummary.chainNodeCount, mtDenseBreakdownWindowCausalSummary.chainEdgeCount, (int)mtDenseBreakdownWindowCausalSummary.latestOwner, (unsigned long long)mtDenseBreakdownWindowCausalSummary.latestOwnerFinishNs, (unsigned long long)mtDenseBreakdownWindowCausalSummary.makespanNs, (unsigned long long)mtDenseBreakdownWindowCausalSummary.chainBodyNs, (unsigned long long)mtDenseBreakdownWindowCausalSummary.chainReleaseNs, (unsigned long long)mtDenseBreakdownWindowCausalSummary.chainGapNs, (unsigned long long)mtDenseBreakdownWindowCausalSummary.timestampBoundUncertaintyNs, mtDenseBreakdownWindowCausalSummary.incompleteMapping ? \"true\" : \"false\", mtDenseBreakdownWindowCausalSummary.clockRegression ? \"true\" : \"false\", mtDenseBreakdownWindowCausalSummary.overflow ? \"true\" : \"false\") < 0) mtDenseBreakdownWindowWriteError = 1; }\n");
      emitBodyLock(3, "if (fprintf(mtDenseBreakdownWindowFile, \"],\\\"causalHotspots\\\":{\\\"version\\\":1,\\\"complete\\\":%%s,\\\"mapping_complete\\\":%%s,\\\"overflow\\\":%%s,\\\"clock_regression\\\":%%s,\\\"logical_mtask_count\\\":%%d,\\\"logical_token_count\\\":%%d,\\\"mtasks\\\":[\", mtDenseBreakdownWindowCausalHotspotComplete ? \"true\" : \"false\", mtDenseBreakdownWindowCausalMappingComplete ? \"true\" : \"false\", mtDenseBreakdownWindowOverflowed ? \"true\" : \"false\", mtDenseBreakdownWindowCausalClockRegression ? \"true\" : \"false\", kDenseBreakdownWindowAllOwnerMTaskCount, kDenseBreakdownWindowCausalTokenCount) < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(3, "bool mtDenseBreakdownWindowCausalHotspotFirstMTask = true; for (int mtaskId = 0; mtaskId < kDenseBreakdownWindowAllOwnerMTaskCount; mtaskId ++) { const MtDenseBreakdownWindowCausalHotspotMTask &mtDenseBreakdownWindowCausalHotspotMTask = mtDenseBreakdownWindowCausalHotspotMTasks[mtaskId]; if (mtDenseBreakdownWindowCausalHotspotMTask.count == 0) continue; if (!mtDenseBreakdownWindowCausalHotspotFirstMTask && fputc(',', mtDenseBreakdownWindowFile) == EOF) mtDenseBreakdownWindowWriteError = 1; mtDenseBreakdownWindowCausalHotspotFirstMTask = false; if (fprintf(mtDenseBreakdownWindowFile, \"{\\\"mtask_id\\\":%%d,\\\"count\\\":%%llu,\\\"body_ns\\\":%%llu,\\\"release_ns\\\":%%llu,\\\"gap_ns\\\":%%llu,\\\"same_owner_pred_count\\\":%%llu,\\\"remote_token_pred_count\\\":%%llu,\\\"latest_tail_count\\\":%%llu}\", mtaskId, (unsigned long long)mtDenseBreakdownWindowCausalHotspotMTask.count, (unsigned long long)mtDenseBreakdownWindowCausalHotspotMTask.bodyNs, (unsigned long long)mtDenseBreakdownWindowCausalHotspotMTask.releaseNs, (unsigned long long)mtDenseBreakdownWindowCausalHotspotMTask.gapNs, (unsigned long long)mtDenseBreakdownWindowCausalHotspotMTask.sameOwnerPredCount, (unsigned long long)mtDenseBreakdownWindowCausalHotspotMTask.remoteTokenPredCount, (unsigned long long)mtDenseBreakdownWindowCausalHotspotMTask.latestTailCount) < 0) mtDenseBreakdownWindowWriteError = 1; }\n");
      emitBodyLock(3, "if (fprintf(mtDenseBreakdownWindowFile, \"],\\\"edges\\\":[\") < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(3, "bool mtDenseBreakdownWindowCausalHotspotFirstEdge = true; for (int index = 0; index < kDenseBreakdownWindowCausalTokenCount; index ++) { const int token = kDenseBreakdownWindowCausalHotspotRemoteTokenOutputOrder[index]; const MtDenseBreakdownWindowCausalHotspotEdge &mtDenseBreakdownWindowCausalHotspotEdge = mtDenseBreakdownWindowCausalHotspotRemoteTokenEdges[token]; if (mtDenseBreakdownWindowCausalHotspotEdge.count == 0) continue; if (!mtDenseBreakdownWindowCausalHotspotFirstEdge && fputc(',', mtDenseBreakdownWindowFile) == EOF) mtDenseBreakdownWindowWriteError = 1; mtDenseBreakdownWindowCausalHotspotFirstEdge = false; if (fprintf(mtDenseBreakdownWindowFile, \"{\\\"kind\\\":\\\"remote_token\\\",\\\"producer_mtask_id\\\":%%d,\\\"consumer_mtask_id\\\":%%d,\\\"logical_token_id\\\":%%d,\\\"count\\\":%%llu,\\\"gap_ns\\\":%%llu}\", kDenseBreakdownWindowCausalTokenProducerMTask[token], kDenseBreakdownWindowCausalTokenConsumerMTask[token], token, (unsigned long long)mtDenseBreakdownWindowCausalHotspotEdge.count, (unsigned long long)mtDenseBreakdownWindowCausalHotspotEdge.gapNs) < 0) mtDenseBreakdownWindowWriteError = 1; }\n");
      emitBodyLock(3, "for (int index = 0; index < kDenseBreakdownWindowAllOwnerMTaskCount; index ++) { const int consumerMTask = kDenseBreakdownWindowCausalHotspotSameOwnerConsumerOutputOrder[index]; const int producerMTask = kDenseBreakdownWindowCausalSameOwnerProducerMTask[consumerMTask]; const MtDenseBreakdownWindowCausalHotspotEdge &mtDenseBreakdownWindowCausalHotspotEdge = mtDenseBreakdownWindowCausalHotspotSameOwnerEdges[consumerMTask]; if (mtDenseBreakdownWindowCausalHotspotEdge.count == 0) continue; if (!mtDenseBreakdownWindowCausalHotspotFirstEdge && fputc(',', mtDenseBreakdownWindowFile) == EOF) mtDenseBreakdownWindowWriteError = 1; mtDenseBreakdownWindowCausalHotspotFirstEdge = false; if (fprintf(mtDenseBreakdownWindowFile, \"{\\\"kind\\\":\\\"same_owner\\\",\\\"producer_mtask_id\\\":%%d,\\\"consumer_mtask_id\\\":%%d,\\\"logical_token_id\\\":-1,\\\"count\\\":%%llu,\\\"gap_ns\\\":%%llu}\", producerMTask, consumerMTask, (unsigned long long)mtDenseBreakdownWindowCausalHotspotEdge.count, (unsigned long long)mtDenseBreakdownWindowCausalHotspotEdge.gapNs) < 0) mtDenseBreakdownWindowWriteError = 1; }\n");
      emitBodyLock(3, "if (fprintf(mtDenseBreakdownWindowFile, \"],\\\"totals\\\":{\\\"count\\\":%%llu,\\\"body_ns\\\":%%llu,\\\"release_ns\\\":%%llu,\\\"gap_ns\\\":%%llu,\\\"same_owner_pred_count\\\":%%llu,\\\"remote_token_pred_count\\\":%%llu,\\\"latest_tail_count\\\":%%llu,\\\"edge_count\\\":%%llu}}}\\n\", (unsigned long long)mtDenseBreakdownWindowCausalHotspotTotals.count, (unsigned long long)mtDenseBreakdownWindowCausalHotspotTotals.bodyNs, (unsigned long long)mtDenseBreakdownWindowCausalHotspotTotals.releaseNs, (unsigned long long)mtDenseBreakdownWindowCausalHotspotTotals.gapNs, (unsigned long long)mtDenseBreakdownWindowCausalHotspotTotals.sameOwnerPredCount, (unsigned long long)mtDenseBreakdownWindowCausalHotspotTotals.remoteTokenPredCount, (unsigned long long)mtDenseBreakdownWindowCausalHotspotTotals.latestTailCount, (unsigned long long)mtDenseBreakdownWindowCausalHotspotTotals.edgeCount) < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(2, "} else if (mtDenseBreakdownWindowCausalChainMode) {\n");
      emitBodyLock(3, "bool mtDenseBreakdownWindowCausalMappingComplete = !mtDenseBreakdownWindowOverflowed && !mtDenseBreakdownWindowCausalClockRegression && mtDenseBreakdownWindowRecordedCycles == mtDenseBreakdownWindowCycles;\n");
      emitBodyLock(3, "for (uint64_t c = 0; c < mtDenseBreakdownWindowRecordedCycles; c ++) { const MtDenseBreakdownWindowCausalSummary &mtDenseBreakdownWindowCausalSummary = mtDenseBreakdownWindowCausalSummaries[c]; if (!mtDenseBreakdownWindowCausalSummary.complete || mtDenseBreakdownWindowCausalSummary.incompleteMapping || mtDenseBreakdownWindowCausalSummary.clockRegression || mtDenseBreakdownWindowCausalSummary.overflow) mtDenseBreakdownWindowCausalMappingComplete = false; }\n");
      emitBodyLock(3, "const bool mtDenseBreakdownWindowCausalComplete = mtDenseBreakdownWindowCausalMappingComplete;\n");
      emitBodyLock(3, "if (fprintf(mtDenseBreakdownWindowFile, \"{\\\"magic\\\":\\\"GSIM_MT_DENSE_BREAKDOWN_WINDOW\\\",\\\"version\\\":1,\\\"window_start\\\":%%llu,\\\"window_cycles\\\":%%llu,\\\"accepted_cycles\\\":%%llu,\\\"threadCount\\\":%%d,\\\"mode\\\":\\\"causalchain\\\",\\\"criticality_offsets_valid\\\":false,\\\"worker0_mtask_count\\\":0,\\\"allowner_mtask_count\\\":%%d,\\\"overflow\\\":%%s,\\\"complete\\\":%%s,\\\"causalchain_mapping_complete\\\":%%s,\\\"causalchain_timestamp_bound_uncertainty\\\":true,\\\"causalchain_timestamp_bound_uncertainty_ns\\\":%%llu,\\\"clock_regression\\\":%%s,\\\"cycles\\\":[\", (unsigned long long)mtDenseBreakdownWindowStart, (unsigned long long)mtDenseBreakdownWindowCycles, (unsigned long long)mtDenseBreakdownWindowRecordedCycles, mtDenseBreakdownProfileWorkerCount, kDenseBreakdownWindowAllOwnerMTaskCount, mtDenseBreakdownWindowOverflowed ? \"true\" : \"false\", mtDenseBreakdownWindowCausalComplete ? \"true\" : \"false\", mtDenseBreakdownWindowCausalMappingComplete ? \"true\" : \"false\", (unsigned long long)mtDenseBreakdownWindowCausalTimestampBoundUncertaintyNs, mtDenseBreakdownWindowCausalClockRegression ? \"true\" : \"false\") < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(3, "for (uint64_t c = 0; c < mtDenseBreakdownWindowRecordedCycles; c ++) { if (c != 0 && fputc(',', mtDenseBreakdownWindowFile) == EOF) mtDenseBreakdownWindowWriteError = 1; const MtDenseBreakdownWindowCausalSummary &mtDenseBreakdownWindowCausalSummary = mtDenseBreakdownWindowCausalSummaries[c]; if (fprintf(mtDenseBreakdownWindowFile, \"{\\\"cycle\\\":%%llu,\\\"causalSummary\\\":{\\\"complete\\\":%%s,\\\"causal_edges\\\":%%u,\\\"max_predecessor_end_ns\\\":%%llu,\\\"max_lag_ns\\\":%%llu,\\\"sameOwnerPredecessorCount\\\":%%u,\\\"remoteTokenPredecessorCount\\\":%%u,\\\"waitObservationCount\\\":%%u,\\\"causalBoundNs\\\":%%llu,\\\"chain_node_count\\\":%%u,\\\"chain_edge_count\\\":%%u,\\\"latest_owner\\\":%%d,\\\"latest_owner_finish_ns\\\":%%llu,\\\"makespan_ns\\\":%%llu,\\\"chain_body_ns\\\":%%llu,\\\"chain_release_ns\\\":%%llu,\\\"chain_gap_ns\\\":%%llu,\\\"timestampBoundUncertainty\\\":true,\\\"timestampBoundUncertaintyNs\\\":%%llu,\\\"incompleteMapping\\\":%%s,\\\"clockRegression\\\":%%s,\\\"overflow\\\":%%s}}\", (unsigned long long)mtDenseBreakdownWindowCycleNumbers[c], mtDenseBreakdownWindowCausalSummary.complete ? \"true\" : \"false\", mtDenseBreakdownWindowCausalSummary.chainEdgeCount, (unsigned long long)mtDenseBreakdownWindowCausalSummary.causalBoundNs, (unsigned long long)mtDenseBreakdownWindowCausalSummary.maxLagNs, mtDenseBreakdownWindowCausalSummary.sameOwnerPredecessorCount, mtDenseBreakdownWindowCausalSummary.remoteTokenPredecessorCount, mtDenseBreakdownWindowCausalSummary.waitObservationCount, (unsigned long long)mtDenseBreakdownWindowCausalSummary.causalBoundNs, mtDenseBreakdownWindowCausalSummary.chainNodeCount, mtDenseBreakdownWindowCausalSummary.chainEdgeCount, (int)mtDenseBreakdownWindowCausalSummary.latestOwner, (unsigned long long)mtDenseBreakdownWindowCausalSummary.latestOwnerFinishNs, (unsigned long long)mtDenseBreakdownWindowCausalSummary.makespanNs, (unsigned long long)mtDenseBreakdownWindowCausalSummary.chainBodyNs, (unsigned long long)mtDenseBreakdownWindowCausalSummary.chainReleaseNs, (unsigned long long)mtDenseBreakdownWindowCausalSummary.chainGapNs, (unsigned long long)mtDenseBreakdownWindowCausalSummary.timestampBoundUncertaintyNs, mtDenseBreakdownWindowCausalSummary.incompleteMapping ? \"true\" : \"false\", mtDenseBreakdownWindowCausalSummary.clockRegression ? \"true\" : \"false\", mtDenseBreakdownWindowCausalSummary.overflow ? \"true\" : \"false\") < 0) mtDenseBreakdownWindowWriteError = 1; }\n");
      emitBodyLock(3, "if (fprintf(mtDenseBreakdownWindowFile, \"]}\\n\") < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(2, "} else {\n");
      emitBodyLock(2, "if (fprintf(mtDenseBreakdownWindowFile, \"{\\\"magic\\\":\\\"GSIM_MT_DENSE_BREAKDOWN_WINDOW\\\",\\\"version\\\":1,\\\"window_start\\\":%%llu,\\\"window_cycles\\\":%%llu,\\\"accepted_cycles\\\":%%llu,\\\"threadCount\\\":%%d,\\\"mode\\\":\\\"%%s\\\",\\\"criticality_offsets_valid\\\":%%s,\\\"worker0_mtask_count\\\":%%d,\\\"allowner_mtask_count\\\":%%d,\\\"overflow\\\":%%s,\\\"complete\\\":%%s,\\\"cycles\\\":[\", (unsigned long long)mtDenseBreakdownWindowStart, (unsigned long long)mtDenseBreakdownWindowCycles, (unsigned long long)mtDenseBreakdownWindowRecordedCycles, mtDenseBreakdownProfileWorkerCount, mtDenseBreakdownWindowMode, mtDenseBreakdownWindowFinishOnlyMode ? \"true\" : \"false\", mtDenseBreakdownWindowWorker0BodyMode ? kDenseBreakdownWindowWorker0MTaskCount : 0, mtDenseBreakdownWindowAllOwnerBodyMode ? kDenseBreakdownWindowAllOwnerMTaskCount : 0, mtDenseBreakdownWindowOverflowed ? \"true\" : \"false\", mtDenseBreakdownWindowComplete ? \"true\" : \"false\") < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(2, "for (uint64_t c = 0; c < mtDenseBreakdownWindowRecordedCycles; c ++) {\n");
      emitBodyLock(3, "if (c != 0 && fputc(',', mtDenseBreakdownWindowFile) == EOF) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(3, "if (fprintf(mtDenseBreakdownWindowFile, \"{\\\"cycle\\\":%%llu,\\\"workers\\\":[\", (unsigned long long)mtDenseBreakdownWindowCycleNumbers[c]) < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(3, "for (int worker = 0; worker < mtDenseBreakdownProfileWorkerCount; worker ++) {\n");
      emitBodyLock(4, "if (worker != 0 && fputc(',', mtDenseBreakdownWindowFile) == EOF) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(4, "const MtDenseBreakdownWindowWorker &mtDenseBreakdownWindowWorker = mtDenseBreakdownWindowWorkers[c][worker];\n");
      emitBodyLock(4, "if (fprintf(mtDenseBreakdownWindowFile, \"{\\\"startOffsetNs\\\":%%llu,\\\"finishOffsetNs\\\":%%llu,\\\"blockedWaitNs\\\":%%llu,\\\"bodyNs\\\":%%llu,\\\"controlNs\\\":%%llu}\", (unsigned long long)mtDenseBreakdownWindowWorker.startOffsetNs, (unsigned long long)mtDenseBreakdownWindowWorker.finishOffsetNs, (unsigned long long)mtDenseBreakdownWindowWorker.blockedWaitNs, (unsigned long long)mtDenseBreakdownWindowWorker.bodyNs, (unsigned long long)mtDenseBreakdownWindowWorker.controlNs) < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(3, "}\n");
      emitBodyLock(3, "uint32_t mtDenseBreakdownWindowMTaskCount = mtDenseBreakdownWindowWorker0MTaskCounts[c];\n");
      emitBodyLock(3, "if (unlikely(mtDenseBreakdownWindowMTaskCount > kDenseBreakdownWindowWorker0MTaskCount)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] corrupt window worker0 MTask count\\n\"); abort(); }\n");
      emitBodyLock(3, "if (fprintf(mtDenseBreakdownWindowFile, \"],\\\"worker0MTasks\\\":[\") < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(3, "for (uint32_t m = 0; m < mtDenseBreakdownWindowMTaskCount; m ++) {\n");
      emitBodyLock(4, "if (m != 0 && fputc(',', mtDenseBreakdownWindowFile) == EOF) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(4, "const MtDenseBreakdownWindowMTask &mtDenseBreakdownWindowMTask = mtDenseBreakdownWindowWorker0MTasks[c][m];\n");
      emitBodyLock(4, "if (fprintf(mtDenseBreakdownWindowFile, \"{\\\"mtaskId\\\":%%u,\\\"bodyNs\\\":%%llu}\", mtDenseBreakdownWindowMTask.mtaskId, (unsigned long long)mtDenseBreakdownWindowMTask.bodyNs) < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(3, "}\n");
      emitBodyLock(3, "if (fprintf(mtDenseBreakdownWindowFile, \"],\\\"allOwnerMTasks\\\":[\") < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(3, "if (mtDenseBreakdownWindowAllOwnerBodyMode) {\n");
      emitBodyLock(4, "for (int mtaskId = 0; mtaskId < kDenseBreakdownWindowAllOwnerMTaskCount; mtaskId ++) {\n");
      emitBodyLock(5, "const MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowAllOwnerMTask = mtDenseBreakdownWindowAllOwnerMTasks[c][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[mtaskId]];\n");
      emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowAllOwnerMTask.mtaskId != (uint32_t)mtaskId || mtDenseBreakdownWindowAllOwnerMTask.ownerThreadId >= (uint16_t)mtDenseBreakdownProfileWorkerCount || mtDenseBreakdownWindowAllOwnerMTask.bodyEndOffsetNs < mtDenseBreakdownWindowAllOwnerMTask.bodyStartOffsetNs || mtDenseBreakdownWindowAllOwnerMTask.releaseEndOffsetNs < mtDenseBreakdownWindowAllOwnerMTask.bodyEndOffsetNs || mtDenseBreakdownWindowAllOwnerMTask.releaseEndOffsetNs == UINT64_MAX)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] corrupt all-owner MTask record\\n\"); abort(); }\n");
      emitBodyLock(5, "if (mtaskId != 0 && fputc(',', mtDenseBreakdownWindowFile) == EOF) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(5, "if (fprintf(mtDenseBreakdownWindowFile, \"{\\\"mtaskId\\\":%%u,\\\"ownerThreadId\\\":%%u,\\\"readyTokenStoreCount\\\":%%u,\\\"bodyStartOffsetNs\\\":%%llu,\\\"bodyEndOffsetNs\\\":%%llu,\\\"bodyNs\\\":%%llu,\\\"releaseEndOffsetNs\\\":%%llu}\", mtDenseBreakdownWindowAllOwnerMTask.mtaskId, (unsigned)mtDenseBreakdownWindowAllOwnerMTask.ownerThreadId, (unsigned)mtDenseBreakdownWindowAllOwnerMTask.readyTokenStoreCount, (unsigned long long)mtDenseBreakdownWindowAllOwnerMTask.bodyStartOffsetNs, (unsigned long long)mtDenseBreakdownWindowAllOwnerMTask.bodyEndOffsetNs, (unsigned long long)mtDenseBreakdownWindowAllOwnerMTask.bodyNs, (unsigned long long)mtDenseBreakdownWindowAllOwnerMTask.releaseEndOffsetNs) < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(5, "}\n");
      emitBodyLock(3, "}\n");
      emitBodyLock(3, "if (fprintf(mtDenseBreakdownWindowFile, \"],\\\"waits\\\":[\") < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(3, "bool mtDenseBreakdownWindowFirstWait = true;\n");
      emitBodyLock(3, "for (int worker = 0; worker < mtDenseBreakdownProfileWorkerCount; worker ++) {\n");
      emitBodyLock(4, "const uint32_t mtDenseBreakdownWindowWaitCount = mtDenseBreakdownWindowWaitCounts[c][worker];\n");
      emitBodyLock(4, "const uint32_t mtDenseBreakdownWindowWaitCapacity = (uint32_t)(kDenseBreakdownWindowWaitLaneOffsets[worker + 1] - kDenseBreakdownWindowWaitLaneOffsets[worker]);\n");
      emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowWaitCount > mtDenseBreakdownWindowWaitCapacity)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] corrupt window ready-wait count\\n\"); abort(); }\n");
      emitBodyLock(4, "for (uint32_t w = 0; w < mtDenseBreakdownWindowWaitCount; w ++) {\n");
      emitBodyLock(5, "const MtDenseBreakdownWindowWait &mtDenseBreakdownWindowWait = mtDenseBreakdownWindowWaits[c][kDenseBreakdownWindowWaitLaneOffsets[worker] + w];\n");
      emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowWait.cycleSlot != (uint16_t)c || mtDenseBreakdownWindowWait.threadId != (uint16_t)worker)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] corrupt window ready-wait lane\\n\"); abort(); }\n");
      emitBodyLock(5, "if (!mtDenseBreakdownWindowFirstWait && fputc(',', mtDenseBreakdownWindowFile) == EOF) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(5, "mtDenseBreakdownWindowFirstWait = false;\n");
      emitBodyLock(5, "if (fprintf(mtDenseBreakdownWindowFile, \"{\\\"cycleSlot\\\":%%u,\\\"threadId\\\":%%u,\\\"consumerMtaskId\\\":%%u,\\\"slot\\\":%%u,\\\"blockedNs\\\":%%llu,\\\"endOffsetNs\\\":%%llu}\", (unsigned)mtDenseBreakdownWindowWait.cycleSlot, (unsigned)mtDenseBreakdownWindowWait.threadId, mtDenseBreakdownWindowWait.consumerMtaskId, mtDenseBreakdownWindowWait.readySlot, (unsigned long long)mtDenseBreakdownWindowWait.blockedNs, (unsigned long long)mtDenseBreakdownWindowWait.endOffsetNs) < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(4, "}\n");
      emitBodyLock(3, "}\n");
      emitBodyLock(3, "if (fprintf(mtDenseBreakdownWindowFile, \"]}\") < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(2, "}\n");
      emitBodyLock(2, "if (fprintf(mtDenseBreakdownWindowFile, \"]}\\n\") < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(2, "}\n");
      emitBodyLock(2, "if (fflush(mtDenseBreakdownWindowFile) != 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(2, "if (fclose(mtDenseBreakdownWindowFile) != 0) mtDenseBreakdownWindowWriteError = 1;\n");
      emitBodyLock(2, "if (mtDenseBreakdownWindowWriteError) { fprintf(stderr, \"[mt-dense-breakdown] failed to write window output path=%%s\\n\", mtDenseBreakdownWindowOutPath); abort(); }\n");
      emitBodyLock(1, "}\n");
    }
    emitBodyLock(0, "}\n");
  }

  if (activationEventTraceCodegen) {
    emitFuncDecl(0, "void S%s::initMtActivationEventTrace() {\n", name.c_str());
    emitBodyLock(1, "mtActivationEventTraceFile = nullptr;\n");
    emitBodyLock(1, "mtActivationEventTraceCycleStart = 0;\n");
    emitBodyLock(1, "mtActivationEventTraceCycleLimit = 0;\n");
    emitBodyLock(1, "mtActivationEventTraceCycleOpen = false;\n");
    emitBodyLock(1, "mtActivationEventTraceRecords.clear();\n");
    emitBodyLock(1, "mtActivationEventTracePendingRecords.clear();\n");
    emitBodyLock(1, "const char *tracePath = getenv(\"GSIM_MT_ACTIVATION_EVENT_TRACE\");\n");
    emitBodyLock(1, "if (tracePath == nullptr || tracePath[0] == '\\0') return;\n");
    emitBodyLock(1, "const char *startEnv = getenv(\"GSIM_MT_DYNAMIC_TRACE_START\");\n");
    emitBodyLock(1, "const char *cyclesEnv = getenv(\"GSIM_MT_DYNAMIC_TRACE_CYCLES\");\n");
    emitBodyLock(1, "uint64_t traceStart = (startEnv != nullptr && startEnv[0] != '\\0') ? strtoull(startEnv, nullptr, 10) : 0;\n");
    emitBodyLock(1, "uint64_t traceCount = (cyclesEnv != nullptr && cyclesEnv[0] != '\\0') ? strtoull(cyclesEnv, nullptr, 10) : 0;\n");
    emitBodyLock(1, "if (traceCount == 0 || traceCount > UINT64_MAX - traceStart) return;\n");
    emitBodyLock(1, "gAssert(mtConfiguredWorkerCount == 1, \"GSIM_MT_ACTIVATION_EVENT_TRACE requires GSIM_THREADS=1 (got %%d)\", mtConfiguredWorkerCount);\n");
    emitBodyLock(1, "if (mtConfiguredWorkerCount != 1) abort();\n");
    if (denseExecutorValid) {
      emitBodyLock(1, "gAssert(!mtUseDenseExecutor, \"GSIM_MT_ACTIVATION_EVENT_TRACE is sparse-only; GSIM_MT_EXECUTOR=dense is unsupported\");\n");
      emitBodyLock(1, "if (mtUseDenseExecutor) abort();\n");
    }
    emitBodyLock(1, "const uint16_t endianProbe = 1;\n");
    emitBodyLock(1, "gAssert(*(const uint8_t *)&endianProbe == 1, \"GSIM_MT_ACTIVATION_EVENT_TRACE requires little-endian byte order\");\n");
    emitBodyLock(1, "if (*(const uint8_t *)&endianProbe != 1) abort();\n");
    emitBodyLock(1, "mtActivationEventTraceCycleStart = traceStart;\n");
    emitBodyLock(1, "mtActivationEventTraceCycleLimit = traceStart + traceCount;\n");
    emitBodyLock(1, "mtActivationEventTraceFile = fopen(tracePath, \"wb\");\n");
    emitBodyLock(1, "gAssert(mtActivationEventTraceFile != nullptr, \"failed to open GSIM_MT_ACTIVATION_EVENT_TRACE=%%s\", tracePath);\n");
    emitBodyLock(1, "if (mtActivationEventTraceFile == nullptr) abort();\n");
    emitBodyLock(1, "MtActivationEventTraceHeader header = {};\n");
    emitBodyLock(1, "const uint8_t magic[8] = {'G', 'S', 'I', 'M', 'A', 'E', 'V', 'T'};\n");
    emitBodyLock(1, "memcpy(header.magic, magic, sizeof(magic));\n");
    emitBodyLock(1, "header.version = 2;\n");
    emitBodyLock(1, "header.headerSize = (uint16_t)sizeof(MtActivationEventTraceHeader);\n");
    emitBodyLock(1, "header.activeWidth = (uint16_t)%d;\n", ACTIVE_WIDTH);
    emitBodyLock(1, "header.taskCount = (uint32_t)%d;\n", superId);
    emitBodyLock(1, "header.recordSize = (uint32_t)sizeof(MtActivationEventTraceRecord);\n");
    emitBodyLock(1, "header.traceStart = traceStart;\n");
    emitBodyLock(1, "header.traceCount = traceCount;\n");
    emitBodyLock(1, "if (fwrite(&header, sizeof(header), 1, mtActivationEventTraceFile) != 1) {\n");
    emitBodyLock(2, "fprintf(stderr, \"[mt-activation-event-trace] failed to write header path=%%s\\n\", tracePath);\n");
    emitBodyLock(2, "fclose(mtActivationEventTraceFile);\n");
    emitBodyLock(2, "mtActivationEventTraceFile = nullptr;\n");
    emitBodyLock(2, "abort();\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(1, "fprintf(stderr, \"[mt-activation-event-trace] path=%%s start=%%lu cycles=%%lu\\n\", tracePath, traceStart, traceCount);\n");
    emitBodyLock(0, "}\n");

    emitFuncDecl(0, "void S%s::recordMtActivationEvent(int32_t sourceCppId, uint32_t activeWordBase, uint64_t mask, MtActivationEventKind kind) {\n", name.c_str());
    emitBodyLock(1, "if (mtActivationEventTraceFile == nullptr || mask == 0) return;\n");
    emitBodyLock(1, "if (cycles < mtActivationEventTraceCycleStart || cycles >= mtActivationEventTraceCycleLimit) return;\n");
    emitBodyLock(1, "MtActivationEventTraceRecord record = {};\n");
    emitBodyLock(1, "record.cycle = cycles;\n");
    emitBodyLock(1, "record.sourceCppId = sourceCppId;\n");
    emitBodyLock(1, "record.activeWordBase = activeWordBase;\n");
    emitBodyLock(1, "record.mask = mask;\n");
    emitBodyLock(1, "record.kind = kind;\n");
    emitBodyLock(1, "if (mtActivationEventTraceCycleOpen) mtActivationEventTraceRecords.push_back(record);\n");
    emitBodyLock(1, "else mtActivationEventTracePendingRecords.push_back(record);\n");
    emitBodyLock(0, "}\n");

    emitFuncDecl(0, "void S%s::beginMtActivationEventTraceCycle() {\n", name.c_str());
    emitBodyLock(1, "if (mtActivationEventTraceFile == nullptr) return;\n");
    emitBodyLock(1, "gAssert(!mtActivationEventTraceCycleOpen, \"activation-event trace cycle already open at cycle %%lu\", cycles);\n");
    emitBodyLock(1, "if (mtActivationEventTraceCycleOpen) abort();\n");
    emitBodyLock(1, "if (cycles >= mtActivationEventTraceCycleStart && cycles < mtActivationEventTraceCycleLimit) {\n");
    emitBodyLock(2, "for (uint32_t base = 0; base < (uint32_t)%d; base += (uint32_t)(64 / %d)) {\n", activeFlagNum, ACTIVE_WIDTH);
    emitBodyLock(3, "uint64_t mask = 0;\n");
    emitBodyLock(3, "for (uint32_t i = 0; i < (uint32_t)(64 / %d) && base + i < (uint32_t)%d; i ++) mask |= (uint64_t)activeFlags[base + i] << (i * %d);\n", ACTIVE_WIDTH, activeFlagNum, ACTIVE_WIDTH);
    emitBodyLock(3, "if (mask != 0) {\n");
    emitBodyLock(4, "MtActivationEventTraceRecord record = {};\n");
    emitBodyLock(4, "record.cycle = cycles;\n");
    emitBodyLock(4, "record.sourceCppId = -1;\n");
    emitBodyLock(4, "record.activeWordBase = base;\n");
    emitBodyLock(4, "record.mask = mask;\n");
    emitBodyLock(4, "record.kind = MT_ACTIVATION_EVENT_FRONTIER;\n");
    emitBodyLock(4, "mtActivationEventTraceRecords.push_back(record);\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(2, "mtActivationEventTraceRecords.insert(mtActivationEventTraceRecords.end(), mtActivationEventTracePendingRecords.begin(), mtActivationEventTracePendingRecords.end());\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(1, "mtActivationEventTracePendingRecords.clear();\n");
    emitBodyLock(1, "mtActivationEventTraceCycleOpen = true;\n");
    emitBodyLock(0, "}\n");

    emitFuncDecl(0, "void S%s::flushMtActivationEventTraceCycle() {\n", name.c_str());
    emitBodyLock(1, "if (mtActivationEventTraceFile == nullptr) return;\n");
    emitBodyLock(1, "if (cycles >= mtActivationEventTraceCycleStart && cycles < mtActivationEventTraceCycleLimit) {\n");
    emitBodyLock(2, "MtActivationEventTraceRecord cycleEnd = {};\n");
    emitBodyLock(2, "cycleEnd.cycle = cycles;\n");
    emitBodyLock(2, "cycleEnd.sourceCppId = -1;\n");
    emitBodyLock(2, "cycleEnd.kind = MT_ACTIVATION_EVENT_CYCLE_END;\n");
    emitBodyLock(2, "mtActivationEventTraceRecords.push_back(cycleEnd);\n");
    emitBodyLock(2, "size_t recordCount = mtActivationEventTraceRecords.size();\n");
    emitBodyLock(2, "size_t written = fwrite(mtActivationEventTraceRecords.data(), sizeof(MtActivationEventTraceRecord), recordCount, mtActivationEventTraceFile);\n");
    emitBodyLock(2, "if (written != recordCount) { fprintf(stderr, \"[mt-activation-event-trace] short write cycle=%%lu expected=%%zu written=%%zu\\n\", cycles, recordCount, written); abort(); }\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(1, "mtActivationEventTraceRecords.clear();\n");
    emitBodyLock(1, "mtActivationEventTraceCycleOpen = false;\n");
    emitBodyLock(1, "if (cycles >= mtActivationEventTraceCycleLimit - 1) closeMtActivationEventTrace();\n");
    emitBodyLock(0, "}\n");

    emitFuncDecl(0, "void S%s::closeMtActivationEventTrace() {\n", name.c_str());
    emitBodyLock(1, "if (mtActivationEventTraceFile != nullptr) {\n");
    emitBodyLock(2, "int mtActivationEventTraceFlushStatus = fflush(mtActivationEventTraceFile);\n");
    emitBodyLock(2, "int mtActivationEventTraceCloseStatus = fclose(mtActivationEventTraceFile);\n");
    emitBodyLock(2, "mtActivationEventTraceFile = nullptr;\n");
    emitBodyLock(2, "if (mtActivationEventTraceFlushStatus != 0 || mtActivationEventTraceCloseStatus != 0) { fprintf(stderr, \"[mt-activation-event-trace] close failed fflush=%%d fclose=%%d\\n\", mtActivationEventTraceFlushStatus, mtActivationEventTraceCloseStatus); abort(); }\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(1, "mtActivationEventTraceRecords.clear();\n");
    emitBodyLock(1, "mtActivationEventTracePendingRecords.clear();\n");
    emitBodyLock(1, "mtActivationEventTraceCycleOpen = false;\n");
    emitBodyLock(0, "}\n");
  }

  emitFuncDecl(0, "S%s::~S%s() {\n", name.c_str(), name.c_str());
  if (useMtHelpers) emitBodyLock(1, "stopMtWorkerPool();\n");
  if (useMtHelpers && mtUseWorkerPoolFlagJoinCodegen()) {
    emitBodyLock(1, "#if defined(GSIM_MT_WORKER_POOL_FLAG_JOIN_COMPILE) && GSIM_MT_WORKER_POOL_FLAG_JOIN_COMPILE\n");
    emitBodyLock(1, "delete[] mtWorkerPoolDoneFlags; mtWorkerPoolDoneFlags = nullptr;\n");
    emitBodyLock(1, "#endif\n");
  }
  if (denseBreakdownProfileCodegen) emitBodyLock(1, "dumpMtDenseBreakdownProfile();\n");
  emitBodyLock(1, "if (wallfracCommitBrackets + wallfracCombBrackets > 0) {\n");
  emitBodyLock(2, "uint64_t __wf_tot = wallfracCommitCycles + wallfracCombCycles;\n");
  emitBodyLock(2, "fprintf(stderr, \"[wallfrac] commit_cycles=%%lu comb_cycles=%%lu commit_brackets=%%lu comb_brackets=%%lu commit_frac=%%.4f comb_frac=%%.4f\\n\", wallfracCommitCycles, wallfracCombCycles, wallfracCommitBrackets, wallfracCombBrackets, __wf_tot? (double)wallfracCommitCycles/__wf_tot : 0.0, __wf_tot? (double)wallfracCombCycles/__wf_tot : 0.0);\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "dumpMtProfile();\n");
  emitBodyLock(1, "if (mtProfileDynamicTraceFile != nullptr) { fclose(mtProfileDynamicTraceFile); mtProfileDynamicTraceFile = nullptr; }\n");
  if (activationEventTraceCodegen) emitBodyLock(1, "closeMtActivationEventTrace();\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::recordMtProfileTask(int cppId, bool pureTask, uint64_t elapsedNs) {\n", name.c_str());
  emitBodyLock(1, "if (!mtProfileEnabled) return;\n");
  emitBodyLock(1, "if (cppId >= 0 && cppId < %d) { mtProfileTaskExecCount[cppId] ++; mtProfileTaskWallNs[cppId] += elapsedNs; }\n", superId);
  emitBodyLock(1, "recordMtProfileDynamicTraceTask(cppId);\n");
  emitBodyLock(1, "if (pureTask) mtProfilePureTasks ++;\n");
  emitBodyLock(1, "else mtProfileSerialTasks ++;\n");
  emitBodyLock(1, "mtProfileSerialFastTaskCount ++;\n");
  emitBodyLock(1, "mtProfileSerialWallNs += elapsedNs;\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::recordMtProfileDynamicTraceTask(int cppId) {\n", name.c_str());
  emitBodyLock(1, "if (mtProfileDynamicTraceFile == nullptr) return;\n");
  emitBodyLock(1, "if (cycles < mtProfileDynamicTraceCycleStart || cycles >= mtProfileDynamicTraceCycleLimit) return;\n");
  emitBodyLock(1, "mtProfileDynamicTraceTaskIds.push_back(cppId);\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::dumpMtProfileDynamicTraceCycle() {\n", name.c_str());
  emitBodyLock(1, "if (mtProfileDynamicTraceFile == nullptr) return;\n");
  emitBodyLock(1, "if (cycles < mtProfileDynamicTraceCycleStart) return;\n");
  emitBodyLock(1, "if (cycles >= mtProfileDynamicTraceCycleLimit) return;\n");
  emitBodyLock(1, "fprintf(mtProfileDynamicTraceFile, \"[mt-dyn-trace] cycle=%%lu task_count=%%zu tasks=\", cycles, mtProfileDynamicTraceTaskIds.size());\n");
  emitBodyLock(1, "for (size_t i = 0; i < mtProfileDynamicTraceTaskIds.size(); i ++) fprintf(mtProfileDynamicTraceFile, \"%%s%%d\", i == 0 ? \"\" : \",\", mtProfileDynamicTraceTaskIds[i]);\n");
  emitBodyLock(1, "fprintf(mtProfileDynamicTraceFile, \"\\n\");\n");
  if (mtUseDynamicStateTraceCodegen()) {
    emitBodyLock(1, "if (mtProfileDynamicStateTraceEnabled) {\n");
    emitBodyLock(2, "size_t mtStateTraceCount = 0, mtStateTraceBlockedCount = 0, mtStateTraceLocalSafeOnlyCount = 0, mtStateTraceRuntimeSafeCount = 0;\n");
    emitBodyLock(2, "for (int cppId : mtProfileDynamicTraceTaskIds) {\n");
    emitBodyLock(3, "if (cppId < 0 || cppId >= (int)mtProfileStateUpdateTraceKindByCppId.size()) continue;\n");
    emitBodyLock(3, "uint8_t kind = mtProfileStateUpdateTraceKindByCppId[(size_t)cppId];\n");
    emitBodyLock(3, "if (kind == 0) continue;\n");
    emitBodyLock(3, "mtStateTraceCount ++;\n");
    emitBodyLock(3, "if (kind == 3) mtStateTraceRuntimeSafeCount ++; else if (kind == 2) mtStateTraceLocalSafeOnlyCount ++; else mtStateTraceBlockedCount ++;\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(2, "if (mtStateTraceCount != 0) {\n");
    emitBodyLock(3, "fprintf(mtProfileDynamicTraceFile, \"[mt-dyn-state-trace] cycle=%%lu state_task_count=%%zu blocked_count=%%zu local_safe_only_count=%%zu runtime_safe_count=%%zu blocked=\", cycles, mtStateTraceCount, mtStateTraceBlockedCount, mtStateTraceLocalSafeOnlyCount, mtStateTraceRuntimeSafeCount);\n");
    emitBodyLock(3, "bool mtStateTraceFirst = true;\n");
    emitBodyLock(3, "for (int cppId : mtProfileDynamicTraceTaskIds) { if (cppId >= 0 && cppId < (int)mtProfileStateUpdateTraceKindByCppId.size() && mtProfileStateUpdateTraceKindByCppId[(size_t)cppId] == 1) { fprintf(mtProfileDynamicTraceFile, \"%%s%%d\", mtStateTraceFirst ? \"\" : \",\", cppId); mtStateTraceFirst = false; } }\n");
    emitBodyLock(3, "fprintf(mtProfileDynamicTraceFile, \" local_safe_only=\");\n");
    emitBodyLock(3, "mtStateTraceFirst = true;\n");
    emitBodyLock(3, "for (int cppId : mtProfileDynamicTraceTaskIds) { if (cppId >= 0 && cppId < (int)mtProfileStateUpdateTraceKindByCppId.size() && mtProfileStateUpdateTraceKindByCppId[(size_t)cppId] == 2) { fprintf(mtProfileDynamicTraceFile, \"%%s%%d\", mtStateTraceFirst ? \"\" : \",\", cppId); mtStateTraceFirst = false; } }\n");
    emitBodyLock(3, "fprintf(mtProfileDynamicTraceFile, \" runtime_safe=\");\n");
    emitBodyLock(3, "mtStateTraceFirst = true;\n");
    emitBodyLock(3, "for (int cppId : mtProfileDynamicTraceTaskIds) { if (cppId >= 0 && cppId < (int)mtProfileStateUpdateTraceKindByCppId.size() && mtProfileStateUpdateTraceKindByCppId[(size_t)cppId] == 3) { fprintf(mtProfileDynamicTraceFile, \"%%s%%d\", mtStateTraceFirst ? \"\" : \",\", cppId); mtStateTraceFirst = false; } }\n");
    emitBodyLock(3, "fprintf(mtProfileDynamicTraceFile, \" state_tasks=\");\n");
    emitBodyLock(3, "mtStateTraceFirst = true;\n");
    emitBodyLock(3, "for (int cppId : mtProfileDynamicTraceTaskIds) { if (cppId >= 0 && cppId < (int)mtProfileStateUpdateTraceKindByCppId.size() && mtProfileStateUpdateTraceKindByCppId[(size_t)cppId] != 0) { fprintf(mtProfileDynamicTraceFile, \"%%s%%d\", mtStateTraceFirst ? \"\" : \",\", cppId); mtStateTraceFirst = false; } }\n");
    emitBodyLock(3, "fprintf(mtProfileDynamicTraceFile, \"\\n\");\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(1, "}\n");
  }
  emitBodyLock(1, "mtProfileDynamicTraceTaskIds.clear();\n");
  emitBodyLock(1, "if (cycles + 1 >= mtProfileDynamicTraceCycleLimit) { fclose(mtProfileDynamicTraceFile); mtProfileDynamicTraceFile = nullptr; }\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::recordMtProfileWorkerTask(int worker) {\n", name.c_str());
  emitBodyLock(1, "if (!mtProfileEnabled) return;\n");
  emitBodyLock(1, "if (worker >= 0 && (size_t)worker < mtProfileWorkerTaskCount.size()) mtProfileWorkerTaskCount[(size_t)worker] ++;\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::dumpMtProfile() {\n", name.c_str());
  if (mtDenseDutyCodegen()) {
    // Per-lane duty-cycle report: coordinator lane = threadCount (reset/join/stepWall),
    // worker lanes 0..threadCount-1 (span=chain busy incl. tail, tail=lookahead scan+OOO work,
    // block=in-chain token spin, spin=pool between-cycle wait).  %%.3f => ms.
    emitBodyLock(1, "if (mtDutyEnabled) {\n");
    emitBodyLock(2, "fprintf(stderr, \"[mt-duty] lane spinMs spanMs tailMs blockMs resetMs joinMs stepWallMs\\n\");\n");
    emitBodyLock(2, "for (int i = 0; i <= kDenseDutyLaneMax; i++) {\n");
    emitBodyLock(3, "MtDenseDutyLane &L = mtDutyLanes[i];\n");
    emitBodyLock(3, "fprintf(stderr, \"[mt-duty] %%d %%.3f %%.3f %%.3f %%.3f %%.3f %%.3f %%.3f\\n\", i, L.spinNs/1e6, L.spanNs/1e6, L.tailNs/1e6, L.blockNs/1e6, L.resetNs/1e6, L.joinNs/1e6, L.stepWallNs/1e6);\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(1, "}\n");
  }
  emitBodyLock(1, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE && defined(GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE) && GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE\n");
  emitBodyLock(1, "fprintf(stderr, \"[mt-lookahead-tail] calls=%%llu scanned=%%llu found=%%llu fullmiss=%%llu\\n\", (unsigned long long)mtDenseLookaheadTailCalls.load(std::memory_order_relaxed), (unsigned long long)mtDenseLookaheadScanned.load(std::memory_order_relaxed), (unsigned long long)mtDenseLookaheadFound.load(std::memory_order_relaxed), (unsigned long long)mtDenseLookaheadFullMiss.load(std::memory_order_relaxed));\n");
  emitBodyLock(1, "#endif\n");
  emitBodyLock(1, "if (!mtProfileEnabled) return;\n");
  if (useMtHelpers) {
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] helper_mode=%%s worker_count=%%d worker_pool=%%d lazy_worker_pool=%%d worker_pool_threads=%%d min_batch_tasks=%%d max_worker_count=%%d cycles=%%lu active_word_count=%%lu serial_tasks=%%lu pure_tasks=%%lu pure_batch_count=%%lu true_parallel_batch_count=%%lu skipped_fake_parallel_batch_count=%%lu serial_fast_task_count=%%lu batch_wall_ns=%%lu true_parallel_wall_ns=%%lu serial_wall_ns=%%lu merge_wall_ns=%%lu total_step_ns=%%lu\\n\", mtProfileHelperMode, mtProfileConfiguredWorkerCount, mtWorkerPoolEnabled ? 1 : 0, mtWorkerPoolLazyStart ? 1 : 0, mtWorkerPoolThreadCount, mtMinBatchTasks, mtProfileMaxWorkerCount, cycles, mtProfileActiveWordCount, mtProfileSerialTasks, mtProfilePureTasks, mtProfilePureBatchCount, mtProfileTrueParallelBatchCount, mtProfileSkippedFakeParallelBatchCount, mtProfileSerialFastTaskCount, mtProfileBatchWallNs, mtProfileTrueParallelWallNs, mtProfileSerialWallNs, mtProfileMergeWallNs, mtProfileTotalStepNs);\n");
  } else {
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] helper_mode=%%s worker_count=%%d min_batch_tasks=%%d max_worker_count=%%d cycles=%%lu active_word_count=%%lu serial_tasks=%%lu pure_tasks=%%lu pure_batch_count=%%lu true_parallel_batch_count=%%lu skipped_fake_parallel_batch_count=%%lu serial_fast_task_count=%%lu batch_wall_ns=%%lu true_parallel_wall_ns=%%lu serial_wall_ns=%%lu merge_wall_ns=%%lu total_step_ns=%%lu\\n\", mtProfileHelperMode, mtProfileConfiguredWorkerCount, mtMinBatchTasks, mtProfileMaxWorkerCount, cycles, mtProfileActiveWordCount, mtProfileSerialTasks, mtProfilePureTasks, mtProfilePureBatchCount, mtProfileTrueParallelBatchCount, mtProfileSkippedFakeParallelBatchCount, mtProfileSerialFastTaskCount, mtProfileBatchWallNs, mtProfileTrueParallelWallNs, mtProfileSerialWallNs, mtProfileMergeWallNs, mtProfileTotalStepNs);\n");
  }
  if (useCoarseMt) {
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_dispatch coarse_runtime=%%s coarse_profitability=%%s coarse_worker_policy=%%s static_runtime_eligible_regions=%%lu static_layer_count=%%lu max_region_layer_count=%%lu static_mtask_count=%%lu region_invocations=%%lu accepted_regions=%%lu rejected_regions=%%lu layer_dispatches=%%lu mtask_dispatches=%%lu worker_jobs=%%lu flag_word_copies=%%lu merge_word_scans=%%lu activation_delta_entries=%%lu estimated_barriers=%%lu estimated_useful_work=%%lu estimated_rejected_useful_work=%%lu estimated_overhead_words=%%lu active_mtasks=%%lu active_mtask_static_cost=%%lu assigned_static_cost=%%lu\\n\", (mtCoarseUseMTaskRuntime ? \"mtask\" : \"layered\"), \"%s\", \"%s\", mtProfileCoarseStaticRuntimeEligibleRegions, mtProfileCoarseStaticLayerCount, mtProfileCoarseStaticMaxRegionLayerCount, mtProfileCoarseStaticMTaskCount, mtProfileCoarseRegionInvocations, mtProfileCoarseAcceptedRegions, mtProfileCoarseRejectedRegions, mtProfileCoarseLayerDispatches, mtProfileCoarseMTaskDispatches, mtProfileCoarseWorkerJobs, mtProfileCoarseFlagWordCopies, mtProfileCoarseMergeWordScans, mtProfileCoarseActivationDeltaEntries, mtProfileCoarseEstimatedBarrierCount, mtProfileCoarseEstimatedUsefulWork, mtProfileCoarseEstimatedRejectedUsefulWork, mtProfileCoarseEstimatedOverheadWords, mtProfileCoarseActiveMTaskCount, mtProfileCoarseActiveMTaskStaticCost, mtProfileCoarseAssignedStaticCost);\n", globalConfig.MtCoarseProfitabilityMode.c_str(), globalConfig.MtCoarseWorkerPolicyMode.c_str());
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_antichain_dispatches=%%lu\\n\", mtProfileCoarseAntichainDispatches);\n");
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] verilator_dual_path enabled=%%d selected_dispatches=%%lu pool_dispatches=%%lu region_index=%%d region_cpp=%%d:%%d\\n\", mtUseVerilatorDualPath ? 1 : 0, mtProfileVerilatorDualPathDispatches, mtProfileVerilatorDualPathWorkerPoolDispatches, mtVerilatorDualPathRegionIndex, mtVerilatorDualPathBeginCppId, mtVerilatorDualPathEndCppId);\n");
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_phase body_ns=%%lu wait_ns=%%lu\\n\", mtProfileCoarseBodyNs, mtProfileCoarseWaitNs);\n");
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_assignment worst_worker_static_cost=%%lu best_worker_static_cost=%%lu contiguous_worst_static_cost=%%lu balanced_worst_static_cost=%%lu\\n\", mtProfileCoarseWorstWorkerStaticCost, mtProfileCoarseBestWorkerStaticCost, mtProfileCoarseContiguousWorstStaticCost, mtProfileCoarseBalancedWorstStaticCost);\n");
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_serial_fallback eligible=%%lu taken=%%lu active_bits=%%lu nonpure_excluded=%%lu saved_worker_jobs=%%lu saved_flag_word_copies=%%lu saved_merge_word_scans=%%lu saved_barriers=%%lu\\n\", mtProfileCoarseSerialFallbackEligible, mtProfileCoarseSerialFallbackTaken, mtProfileCoarseSerialFallbackActiveBits, mtProfileCoarseSerialFallbackNonPureExcluded, mtProfileCoarseSerialFallbackSavedWorkerJobs, mtProfileCoarseSerialFallbackSavedFlagWordCopies, mtProfileCoarseSerialFallbackSavedMergeWordScans, mtProfileCoarseSerialFallbackSavedBarriers);\n");
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_layer_size_hist=%%lu,%%lu,%%lu,%%lu,%%lu,%%lu static=%%d,%%d,%%d,%%d,%%d,%%d labels=1,2,3-4,5-8,9-15,16+\\n\", mtProfileCoarseLayerSizeHist[0], mtProfileCoarseLayerSizeHist[1], mtProfileCoarseLayerSizeHist[2], mtProfileCoarseLayerSizeHist[3], mtProfileCoarseLayerSizeHist[4], mtProfileCoarseLayerSizeHist[5], %d, %d, %d, %d, %d, %d);\n",
                 mtCoarseProfileFacts.layerSizeHist[0], mtCoarseProfileFacts.layerSizeHist[1], mtCoarseProfileFacts.layerSizeHist[2],
                 mtCoarseProfileFacts.layerSizeHist[3], mtCoarseProfileFacts.layerSizeHist[4], mtCoarseProfileFacts.layerSizeHist[5]);
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_region_layer_count_hist=%%lu,%%lu,%%lu,%%lu,%%lu,%%lu labels=1,2,3-4,5-8,9-15,16+\\n\", mtProfileCoarseRegionLayerCountHist[0], mtProfileCoarseRegionLayerCountHist[1], mtProfileCoarseRegionLayerCountHist[2], mtProfileCoarseRegionLayerCountHist[3], mtProfileCoarseRegionLayerCountHist[4], mtProfileCoarseRegionLayerCountHist[5]);\n");
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_selected_worker_count_hist=\");\n");
    emitBodyLock(1, "for (size_t i = 0; i < mtProfileCoarseSelectedWorkerCountHist.size(); i ++) fprintf(stderr, \"%%s%%zu:%%lu\", i == 0 ? \"\" : \",\", i, mtProfileCoarseSelectedWorkerCountHist[i]);\n");
    emitBodyLock(1, "fprintf(stderr, \"\\n\");\n");
  }
  emitBodyLock(1, "fprintf(stderr, \"[mt-profile] activation_delta entries=%%lu max_entries_per_worker=%%lu activate_all_count=%%lu\\n\", mtProfileActivationDeltaEntries, mtProfileActivationDeltaMaxEntriesPerWorker, mtProfileActivationDeltaActivateAllCount);\n");
  emitBodyLock(1, "fprintf(stderr, \"[mt-profile] rejection_reasons not_active_whole=%%lu always_active_task=%%lu serial_task=%%lu dependency_edge=%%lu same_active_word_hazard=%%lu below_min_batch=%%lu configured_single_worker=%%lu\\n\", mtProfileRejectNotActiveWhole, mtProfileRejectAlwaysActiveTask, mtProfileRejectSerialTask, mtProfileRejectDependencyEdge, mtProfileRejectSameActiveWordHazard, mtProfileRejectBelowMinBatch, mtProfileRejectConfiguredSingleWorker);\n");
  emitBodyLock(1, "fprintf(stderr, \"[mt-profile] level_dispatch safe_serial_dispatched=%%lu worker0_only_dispatched=%%lu region_span_cap=%d\\n\", mtProfileSafeSerialDispatched, mtProfileWorker0OnlyDispatched);\n", MT_LEVEL_DISPATCH_REGION_SPAN_CAP);
  emitBodyLock(1, "fprintf(stderr, \"[mt-profile] batch_size_hist=%%lu,%%lu,%%lu,%%lu,%%lu,%%lu labels=1,2,3-4,5-8,9-15,16+\\n\", mtProfileBatchSizeHist[0], mtProfileBatchSizeHist[1], mtProfileBatchSizeHist[2], mtProfileBatchSizeHist[3], mtProfileBatchSizeHist[4], mtProfileBatchSizeHist[5]);\n");
  emitBodyLock(1, "fprintf(stderr, \"[mt-profile] effective_worker_count_hist=\");\n");
  emitBodyLock(1, "for (size_t i = 0; i < mtProfileEffectiveWorkerCountHist.size(); i ++) fprintf(stderr, \"%%s%%zu:%%lu\", i == 0 ? \"\" : \",\", i, mtProfileEffectiveWorkerCountHist[i]);\n");
  emitBodyLock(1, "fprintf(stderr, \"\\n\");\n");
  emitBodyLock(1, "fprintf(stderr, \"[mt-profile] partition_facts batch_member_node_count=%%lu same_active_word_forward_edges=%%lu cross_batch_activation_fanout=%%lu\\n\", mtProfileBatchMemberNodeCount, mtProfileSameActiveWordForwardEdges, mtProfileCrossBatchActivationFanout);\n");
  emitBodyLock(1, "fprintf(stderr, \"[mt-profile] worker_task_count=\");\n");
  emitBodyLock(1, "for (size_t i = 0; i < mtProfileWorkerTaskCount.size(); i ++) fprintf(stderr, \"%%s%%lu\", i == 0 ? \"\" : \",\", mtProfileWorkerTaskCount[i]);\n");
  emitBodyLock(1, "fprintf(stderr, \"\\n\");\n");
  emitBodyLock(1, "const char *fireProfileEnv = getenv(\"GSIM_MT_FIRE_PROFILE\");\n");
  emitBodyLock(1, "const char *taskProfileEnv = getenv(\"GSIM_MT_PROFILE_TASKS\");\n");
  emitBodyLock(1, "if ((taskProfileEnv != nullptr && taskProfileEnv[0] != '\\0' && taskProfileEnv[0] != '0') || (fireProfileEnv != nullptr && fireProfileEnv[0] != '\\0' && fireProfileEnv[0] != '0')) {\n");
  emitBodyLock(2, "fprintf(stderr, \"[mt-profile] task_cpp_ids=count:wall_ns \");\n");
  emitBodyLock(2, "bool firstTask = true;\n");
  emitBodyLock(2, "for (int i = 0; i < %d; i ++) {\n", superId);
  emitBodyLock(3, "if (mtProfileTaskExecCount[i] == 0) continue;\n");
  emitBodyLock(3, "fprintf(stderr, \"%%s%%d:%%lu:%%lu\", firstTask ? \"\" : \",\", i, mtProfileTaskExecCount[i], mtProfileTaskWallNs[i]);\n");
  emitBodyLock(3, "firstTask = false;\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "fprintf(stderr, \"\\n\");\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(0, "}\n");

  /* activation all nodes for reset */
  if (activationEventTraceCodegen) {
    fprintf(header, "void activateAll(int32_t sourceCppId = -1);\n");
    emitFuncDecl(0, "void S%s::activateAll(int32_t sourceCppId) {\n"
                 "  memset(activeFlags, 0xff, sizeof(activeFlags));\n"
                 "  recordMtActivationEvent(sourceCppId, 0, UINT64_MAX, MT_ACTIVATION_EVENT_ACTIVATE_ALL);\n"
                 "}\n", name.c_str());
  } else {
    fprintf(header, "void activateAll();\n");
    emitFuncDecl(0, "void S%s::activateAll() {\n"
                 "  memset(activeFlags, 0xff, sizeof(activeFlags));\n"
                 "}\n", name.c_str());
  }
   /* input/output interface */
  for (Node* node : input) {
    fprintf(header, "void set_%s(%s val);\n", node->name.c_str(), widthUType(node->width).c_str());
    genInterfaceInput(node);
  }
  for (Node* node : output) {
    fprintf(header, "%s get_%s();\n", widthUType(node->width).c_str(), node->name.c_str());
    genInterfaceOutput(node);
  }

  /* reset functions */
  fprintf(header, "void resetAll();\n");
  genResetAll();
  if (denseExecutorValid) {
    fprintf(header, "void resetAllDense();\n");
    genResetAllDense();
  }
  for (int i = 0; i < resetFuncNum; i ++) {
    if (mtUseActivationEventTraceCodegen()) fprintf(header, "void subReset%d(int32_t traceSourceCppId);\n", i);
    else fprintf(header, "void subReset%d();\n", i);
    if (denseExecutorValid) fprintf(header, "void subResetDense%d();\n", i);
    if (globalConfig.MtHelperMode == "buffered-seq") {
      if (mtUseActivationEventTraceCodegen()) fprintf(header, "void subReset%d(ActiveBuffer &nextActive, int32_t traceSourceCppId);\n", i);
      else fprintf(header, "void subReset%d(ActiveBuffer &nextActive);\n", i);
    }
    if (useMtHelpers) {
      if (mtUseActivationEventTraceCodegen()) fprintf(header, "void subReset%d(ActivationDelta &nextActive, int32_t traceSourceCppId);\n", i);
      else fprintf(header, "void subReset%d(ActivationDelta &nextActive);\n", i);
    }
  }
  for (const std::string& decl : mtResetChunkDecls()) fprintf(header, "%s", decl.c_str());

  /* main evaluation loop (step) */
  int subStepIdxMax = 0;
  int serialFastSubStepMax = -1;
  std::string serialFastSuffix;
  Assert(!mtUseDenseOnlyCodegen() || (useMtHelpers && denseExecutorValid),
         "GSIM_MT_DENSE_ONLY_CODEGEN requires --mt-helper-mode=mt-level-dispatch and the dense executor (GSIM_MT_DENSE_EXECUTOR_CODEGEN)");
  const bool denseOnlyCodegen = mtUseDenseOnlyCodegen();
  if (useMtHelpers) {
    serialFastSuffix = "SerialFast";
    if (!mtUseDenseOnlyCodegenLevel2()) {
      { EmitPhaseTimer genActivateTimer("Final.genActivate"); serialFastSubStepMax = genActivate(serialFastSuffix); }
    }
    subStepIdxMax = genActivateMtHelpers(serialFastSubStepMax, serialFastSuffix);
  } else if (useSeqHelpers) {
    subStepIdxMax = genActivateSeqHelpers(useBufferedHelpers);
  } else {
    subStepIdxMax = genActivate();
  }
  for (int i = 0; i <= subStepIdxMax; i ++) {
    fprintf(header, "void subStep%d();\n", i);
  }
  if (serialFastSubStepMax >= 0) {
    for (int i = 0; i <= serialFastSubStepMax; i ++) {
      fprintf(header, "void subStep%d%s();\n", i, serialFastSuffix.c_str());
    }
  }
  if (useHelperTasks) {
    for (int i = 0; i < superId; i ++) {
      if (globalConfig.MtHelperMode == "buffered-seq") fprintf(header, "void mtTask%d(uint%d_t &flag, ActiveBuffer &nextActive);\n", i, ACTIVE_WIDTH);
      if (useMtHelpers && !denseOnlyCodegen) fprintf(header, "void mtTask%d(uint%d_t &flag, ActivationDelta &nextActive);\n", i, ACTIVE_WIDTH);
      if (!useBufferedHelpers || useMtHelpers) fprintf(header, "void mtTask%d(uint%d_t &flag);\n", i, ACTIVE_WIDTH);
    }
    if (useMtHelpers) {
      // Dense-only: shard/pure-batch dispatchers are not emitted; the worker
      // pool core stays (dense executor jobKind 6/7 posts).
      if (!denseOnlyCodegen) {
        int shardCount = mtPureBatchShardCount();
        for (int shard = 0; shard < shardCount; shard ++) {
          fprintf(header, "void mtRunPureBatchDirectShard%d(int chunkBegin, int chunkEnd, uint%d_t &activeWord);\n", shard, ACTIVE_WIDTH);
          fprintf(header, "void mtRunPureBatchWorkerShard%d(int worker, int chunkBegin, int chunkEnd, std::vector<std::vector<int>> &mtProfileLocalTaskIds, std::vector<uint64_t> &mtProfileLocalWorkerTaskCount);\n", shard);
        }
        fprintf(header, "void mtRunPureBatchWorkerRange(int worker, int chunkBegin, int chunkEnd);\n");
      }
      fprintf(header, "void mtWorkerPoolPause();\n");
      fprintf(header, "void mtWorkerPoolPost();\n");
      fprintf(header, "void mtWorkerPoolWaitForDone(int expectedDoneCount);\n");
      fprintf(header, "void mtWorkerPoolLoop(int worker);\n");
      fprintf(header, "void startMtWorkerPool();\n");
      fprintf(header, "void stopMtWorkerPool();\n");
      if (!denseOnlyCodegen) fprintf(header, "void mtRunPureBatch(int beginCppId, int endCppId, uint%d_t &activeWord);\n", ACTIVE_WIDTH);
      if (useCoarseMt && !denseOnlyCodegen) {
        fprintf(header, "void mtRunCoarseLayerWorkerRange(int worker, int regionIndex, int layerIndex, int chunkBegin, int chunkEnd);\n");
        fprintf(header, "void mtMergeLocalCoarseDelta(int worker, int regionBeginActiveWord, int regionActiveWordSpan);\n");
        fprintf(header, "void mtRunCoarseMTaskWorkerList(int worker, int regionIndex, const int *mtaskIndices, int mtaskCount);\n");
        fprintf(header, "void mtRunCoarseMTaskWorkerRange(int worker, int regionIndex, int mtaskBegin, int mtaskEnd);\n");
        fprintf(header, "void mtRunCoarseMTaskDynamic(int regionIndex, int worker);\n");
        // antichain ready-queue helpers.
        fprintf(header, "void mtCoarseReadyQueuePush(int regionIndex, int mtaskIndex, bool worker0Only);\n");
        fprintf(header, "int mtCoarseReadyQueuePop(int regionIndex, int worker);\n");
        fprintf(header, "int mtCountActiveCoarseMTasks(int regionIndex, uint%d_t *coarseActiveWords, int *activeStaticCost);\n", ACTIVE_WIDTH);
        fprintf(header, "void mtBuildCoarseMTaskWorkerAssignment(int regionIndex, int workerCount, std::vector<std::vector<int>> &assignments, std::vector<uint64_t> &workerStaticCosts, std::vector<uint64_t> &workerTaskCounts);\n");
        fprintf(header, "void mtRunCoarseRegion(int regionIndex, uint%d_t *coarseActiveWords);\n", ACTIVE_WIDTH);
        // codegen-time LPT + flat per-cppId arrays.
        fprintf(header, "void mtRunCoarseStaticRefList(int regionIndex, int roundedWC, int worker, int regionBeginActiveWord, int regionActiveWordSpan, const SCoarseTaskRef *refs, int refCount);\n");
        fprintf(header, "void mtRunCoarseRegionStaticDispatch(int regionIndex, int roundedWC, int worker, int regionBeginActiveWord, int regionActiveWordSpan);\n");
        {
          MtCoarseRegionPlan dstaticPlan;
          { EmitPhaseTimer dstaticPlanTimer("Final.header.dstaticPlan"); dstaticPlan = planMtCoarseRegionsForInvocation(); }
          int regionIndex = 0;
          for (const MtCoarseRegion& region : dstaticPlan.regions) {
            if (!region.runtimeEligible) continue;
            fprintf(header, "void mtRunCoarseRegionStaticR%d(int roundedWC, int worker, int regionBeginActiveWord, int regionActiveWordSpan);\n", regionIndex);
            regionIndex ++;
          }
        }
      }
    }
  }

  if (denseExecutorValid) {
    fprintf(header, "void stepDense();\n");
    genDenseExecutor(mtDenseSchedule, header);
  }
  else if (useMtHelpers) {
    fprintf(header, "void stepDenseThreadWorker(int threadId);\n");
    emitFuncDecl(0, "void S%s::stepDenseThreadWorker(int threadId) {\n", name.c_str());
    emitBodyLock(1, "(void)threadId;\n");
    emitBodyLock(0, "}\n");
  }

  /* step wrapper */
  fprintf(header, "void step();\n");
  genStep(subStepIdxMax, serialFastSubStepMax, serialFastSuffix, denseExecutorValid);

  /* end of file */
  // Tail-scan instrumentation counters live OUTSIDE the class as inline namespace
  // variables (the tail function and the dumpMtProfile print land in different
  // SimTop*.cpp shards; class members would need out-of-line definitions).
  fprintf(header, "};\n"
                  "#if defined(GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE) && GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE\n"
                  "inline std::atomic<uint64_t> mtDenseLookaheadTailCalls{0}, mtDenseLookaheadScanned{0}, mtDenseLookaheadFound{0}, mtDenseLookaheadFullMiss{0};\n"
                  "#endif\n"
                  "#endif\n");
  fclose(header);
  commitStableOutputFile(headerTmpFilePath, headerFilePath);
  fclose(srcFp);
  commitStableOutputFile(srcTmpFilePath, srcFilePath);
  if (globalConfig.MtStableOutput) {
    for (int staleIdx = srcFileIdx; ; staleIdx ++) {
      std::string stalePath = format("%s%d.cpp", (globalConfig.OutputDir + "/" + name).c_str(), staleIdx);
      if (std::remove(stalePath.c_str()) != 0) break;
    }
  }
#ifdef DIFFTEST_PER_SIG
  fclose(sigFile);
#endif

  printf("[cppEmitter] define %ld nodes %d superNodes\n", definedNode.size(), superId);
  if (mtOldValueHistogramEnabled()) {
    uint64_t histTotal = mtOldSnapWithConsumers.load(std::memory_order_relaxed) + mtOldSnapNoConsumers.load(std::memory_order_relaxed);
    fprintf(stderr, "[mt-oldvalue-histogram] with_activation_consumers=%llu no_consumers=%llu (no-consumer share %.2f%%)\n",
            (unsigned long long)mtOldSnapWithConsumers.load(std::memory_order_relaxed),
            (unsigned long long)mtOldSnapNoConsumers.load(std::memory_order_relaxed),
            histTotal ? 100.0 * (double)mtOldSnapNoConsumers.load(std::memory_order_relaxed) / (double)histTotal : 0.0);
  }
  std::cout << "[cppEmitter] finish writing " << srcFileIdx << " cpp files to " + globalConfig.OutputDir + "/" << std::endl;
}
