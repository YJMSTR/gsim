// cppEmitterDenseSchedule.cpp - split from cppEmitter.cpp (pure code motion):
// dense edges -> Kosaraju SCC -> chain coarsening -> verilator contract (vcontract)
// -> mtBuildDenseScheduleOrder -> owner-ready/breakdown layouts -> buildMtDenseSchedule.
// gVcAutoPass two-pass control travels WITH vcontract as TU-local state.
#include "cppEmitterImpl.h"
#include <cstring>

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

std::vector<MtDenseMTask> mtBuildDenseMTasks(const MtDenseSchedule& schedule,
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

std::vector<std::vector<int>> mtBuildDenseRuntimeSuccs(const std::vector<MtDenseMTask>& mtasks,
                                                             const std::vector<int>& assignment,
                                                             bool xthreadDepsOnly,
                                                             int* sameThreadElidedCount) {
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

int mtDenseRuntimeEdgeCount(const std::vector<std::vector<int>>& runtimeSuccs) {
  int edgeCount = 0;
  for (const std::vector<int>& succs : runtimeSuccs) edgeCount += static_cast<int>(succs.size());
  return edgeCount;
}


MtDenseOwnerReadyLayout mtBuildDenseOwnerReadyLayout(
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




MtDenseBreakdownWindowWaitLayout mtBuildDenseBreakdownWindowWaitLayout(
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



MtDenseBreakdownWindowAllOwnerLayout mtBuildDenseBreakdownWindowAllOwnerLayout(
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
int mtReduceDenseRuntimeSuccsTransitive(std::vector<std::vector<int>>& runtimeSuccs,
                                               const std::vector<int>& assignment,
                                               bool injectWorkerChains) {
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

std::pair<std::vector<int>, int> mtBuildDensePackThreadsAssignment(const std::vector<MtDenseMTask>& mtasks,
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

MtDenseSchedule buildMtDenseSchedule(const std::map<int, MtTaskInfo>& tasks, bool codegenEnabled) {
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
    // GSIM_MT_DENSE_SCHED_COSTFILE (default off): import measured per-MTask
    // body costs keyed by the immutable member key (mtDenseMTaskMemberKey),
    // applied AFTER contraction/membership is frozen and BEFORE the
    // SCHED_ORDER list scheduler + renumber. Keyed by membership, never by
    // emitted id — the rejected 706ba67 importer's pre/post-renumber
    // miskeying is the failure this avoids (measured-cost-identity-correction).
    // File format: first line "# firSha256 <hex> mtasks <n>", then
    // "0x<memberKeyHex> <costNs>" per MTask. Full coverage and positive costs
    // required; any mismatch aborts generation.
    {
      const char* costFileEnv = std::getenv("GSIM_MT_DENSE_SCHED_COSTFILE");
      if (costFileEnv && costFileEnv[0] && costFileEnv[0] != '0') {
        const char* policyEnv = std::getenv("GSIM_MT_DENSE_VCONTRACT_POLICY");
        const char* autoEnv = std::getenv("GSIM_MT_DENSE_VCONTRACT_MAXMT_AUTO");
        Assert(!(policyEnv && std::strncmp(policyEnv, "auto", 4) == 0)
                   && !(autoEnv && autoEnv[0] && autoEnv[0] != '0'),
               "GSIM_MT_DENSE_SCHED_COSTFILE is incompatible with vcontract auto probes (probe/final cost domains would diverge)");
        FILE* fp = fopen(costFileEnv, "r");
        Assert(fp != nullptr, "cannot open GSIM_MT_DENSE_SCHED_COSTFILE %s", costFileEnv);
        if (fp == nullptr) abort();
        char hashTag[64]; char countTag[64]; char firSha[128]; int fileMTasks = -1;
        Assert(fscanf(fp, "%*63s %63s %127s %63s %d", hashTag, firSha, countTag, &fileMTasks) == 4
                   && std::strcmp(countTag, "mtasks") == 0,
               "dense sched cost file %s missing lineage header", costFileEnv);
        Assert(fileMTasks == static_cast<int>(schedule.mtasks.size()),
               "dense sched cost file %s covers %d MTasks but schedule has %zu",
               costFileEnv, fileMTasks, schedule.mtasks.size());
        std::map<uint64_t, long long> costByKey;
        unsigned long long key = 0; long long cost = 0;
        while (fscanf(fp, " 0x%llx %lld", &key, &cost) == 2) {
          Assert(costByKey.insert({key, cost}).second,
                 "dense sched cost file %s has duplicate memberKey 0x%llx", costFileEnv, key);
        }
        fclose(fp);
        Assert(static_cast<int>(costByKey.size()) == fileMTasks,
               "dense sched cost file %s has %zu entries, header claims %d",
               costFileEnv, costByKey.size(), fileMTasks);
        int matched = 0;
        for (int mtaskId = 0; mtaskId < static_cast<int>(schedule.mtasks.size()); ++mtaskId) {
          const uint64_t mkey = mtDenseMTaskMemberKey(schedule, mtaskId);
          auto it = costByKey.find(mkey);
          Assert(it != costByKey.end(),
                 "dense sched cost file %s lacks memberKey 0x%llx for MTask %d",
                 costFileEnv, (unsigned long long)mkey, mtaskId);
          if (it == costByKey.end()) abort();
          // costOf falls back to staticCost on schedCost<=0; never import a zero.
          schedule.mtasks[(size_t)mtaskId].schedCost =
              static_cast<int>(std::max<long long>(1, std::min<long long>(it->second, INT32_MAX)));
          ++matched;
        }
        Assert(matched == fileMTasks,
               "dense sched cost coverage mismatch: matched %d of %d", matched, fileMTasks);
        fprintf(stderr, "[mt-dense-sched-cost] imported %d measured MTask costs (memberKey-keyed) from %s\n",
                matched, costFileEnv);
      }
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

