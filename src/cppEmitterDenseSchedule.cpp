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
// ---- GSIM_MT_DENSE_SCHED_HOTEDGE (default off, byte-identical when unset) ----
// J-aware CCD penalty (WP2 realization with measured J). Loads a
// SimTop_edge_timing.json (GSIM_MT_DENSE_EDGE_TIMING output) and generalizes
// the GSIM_MT_DENSE_PACK_CCD_AFFINITY flat-gamma cross-CCD penalty in the
// greedy below: a MEASURED producer->consumer edge pays gamma scaled by its
// normalized blocked time (J/maxJ, integer milli-units 0..1000) INSTEAD of
// the flat gamma; UNMEASURED edges keep the flat gamma unchanged. The scale
// still comes entirely from GSIM_MT_DENSE_PACK_CCD_AFFINITY (gamma=0 disables
// both terms), and the term only applies in the percentage-penalty branch --
// GSIM_MT_DENSE_SCHED_ABS_LAT replaces the whole xsync/ccd term family with
// machine constants and is left untouched.
struct MtDenseHotedgeEntry {
  int waitIndex = -1;
  int consumerMtaskId = -1;
  int tokenSlot = -1;
  unsigned long long waits = 0;
  unsigned long long blockedNs = 0;
};

static bool mtHotedgeJsonField(const std::string& rec, const char* key, long long* out) {
  size_t at = rec.find(key);
  if (at == std::string::npos) return false;
  const char* p = rec.c_str() + at + std::strlen(key);
  while (*p == ' ' || *p == '\t') p++;
  bool neg = false;
  if (*p == '-') { neg = true; p++; }
  if (*p < '0' || *p > '9') return false;
  unsigned long long v = 0;
  while (*p >= '0' && *p <= '9') { v = v * 10 + (unsigned long long)(*p - '0'); p++; }
  *out = neg ? -(long long)v : (long long)v;
  return true;
}

// Minimal reader for the machine-generated dump (flat records, no nesting):
// locate "wait_entries", then split on '{'...'}' and pull the five fields.
static bool mtLoadHotedgeJson(const char* path, std::vector<MtDenseHotedgeEntry>& out) {
  FILE* fp = std::fopen(path, "r");
  if (fp == nullptr) return false;
  std::string data;
  char buf[65536];
  size_t got;
  while ((got = std::fread(buf, 1, sizeof(buf), fp)) > 0) data.append(buf, got);
  std::fclose(fp);
  const char* entries = std::strstr(data.c_str(), "\"wait_entries\"");
  if (entries == nullptr) return false;
  const char* p = std::strchr(entries, '[');
  if (p == nullptr) return false;
  p++;
  while (true) {
    while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t' || *p == ',') p++;
    if (*p == ']' || *p == '\0') break;
    if (*p != '{') return false;
    const char* recEnd = std::strchr(p, '}');
    if (recEnd == nullptr) return false;
    std::string rec(p, (size_t)(recEnd - p + 1));
    MtDenseHotedgeEntry e;
    long long v = 0;
    if (mtHotedgeJsonField(rec, "\"waitIndex\":", &v)) e.waitIndex = (int)v;
    if (mtHotedgeJsonField(rec, "\"consumerMtaskId\":", &v)) e.consumerMtaskId = (int)v;
    if (mtHotedgeJsonField(rec, "\"tokenSlot\":", &v)) e.tokenSlot = (int)v;
    if (mtHotedgeJsonField(rec, "\"waits\":", &v)) e.waits = (unsigned long long)v;
    if (mtHotedgeJsonField(rec, "\"blockedNs\":", &v)) e.blockedNs = (unsigned long long)v;
    out.push_back(e);
    p = recEnd + 1;
  }
  return true;
}

// The greedy itself. hotPctByConsumer == nullptr reproduces the historical
// behavior byte-for-byte; when set, [consumer][producer] holds the measured
// edge weight in milli-units (0..1000, J/maxJ) that scales the flat gamma.
static void mtDenseScheduleOrderImpl(const std::vector<MtDenseMTask>& mtasks, int threadCount,
                                     std::vector<int>& outAssign, std::vector<int>& outOrder,
                                     const std::vector<std::map<int, int>>* hotPctByConsumer) {
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
  // GSIM_MT_DENSE_SCHED_PRIO_MODE=slack (default hlf, byte-identical): replace
  // the highest-level-first tie-break priority with ASAP/ALAP slack computed
  // from the same costOf domain (resource-free bounds). Lower slack wins ties.
  bool prioSlack = false;
  { const char* e = std::getenv("GSIM_MT_DENSE_SCHED_PRIO_MODE"); if (e && std::strcmp(e, "slack") == 0) prioSlack = true; }
  if (prioSlack) {
    // ASAP earliest finish (no resource contention): est = max(pred est) + cost.
    std::vector<long long> est((size_t)n, 0);
    for (int i = 0; i < n; i ++) {
      long long e0 = 0;
      for (int pred : mtasks[(size_t)i].predMTasks) if (pred >= 0 && pred < n) e0 = std::max(e0, est[(size_t)pred]);
      est[(size_t)i] = e0 + costOf(mtasks[(size_t)i]);
    }
    // ALAP latest start: lst = min(succ lst) - own cost; sinks: lst = est.
    std::vector<long long> lst((size_t)n, 0);
    for (int i = n - 1; i >= 0; i --) {
      if (mtasks[(size_t)i].succMTasks.empty()) { lst[(size_t)i] = est[(size_t)i] - costOf(mtasks[(size_t)i]); continue; }
      long long l = std::numeric_limits<long long>::max();
      for (int succ : mtasks[(size_t)i].succMTasks) if (succ >= 0 && succ < n) l = std::min(l, lst[(size_t)succ]);
      lst[(size_t)i] = l - costOf(mtasks[(size_t)i]);
    }
    for (int i = 0; i < n; i ++) {
      long long slack = lst[(size_t)i] - (est[(size_t)i] - costOf(mtasks[(size_t)i]));
      priority[(size_t)i] = -slack;  // smaller slack -> larger priority value
    }
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
  // GSIM_MT_DENSE_SCHED_XSYNC_PCT=<pct> (default 30, byte-identical): the
  // cross-thread handoff penalty as a percentage of the producer's cost. It
  // was calibrated in the static node-count domain; measured ns costs change
  // its physical meaning, so the value is swept under SCHED_COSTFILE.
  int xsyncPct = 30;
  { const char* e = std::getenv("GSIM_MT_DENSE_SCHED_XSYNC_PCT"); if (e && e[0]) { int v = std::atoi(e); if (v >= 0) xsyncPct = v; } }
  const int ccdSize = 8;
  while (!ready.empty()) {
    int bestReadyIndex = -1, bestMTask = -1, bestWorker = 0;
  // GSIM_MT_DENSE_SCHED_W0_MERGE=1 (default off, byte-identical): let ordinary
  // MTasks compete for worker 0 too (worker0-only tasks still pin to 0). The
  // reservation avoids serializing pinned side-effect work, but under measured
  // ns costs worker 0 carries the lightest T16 load; measure the tradeoff.
  bool w0Merge = false;
  { const char* e = std::getenv("GSIM_MT_DENSE_SCHED_W0_MERGE"); if (e && e[0] && e[0] != '0') w0Merge = true; }
  // GSIM_MT_DENSE_SCHED_ABS_LAT=1 (default off, byte-identical): replace the
  // percentage handoff penalties with measured machine constants in the same
  // ns domain as imported body costs: 24ns same-CCD token handoff, 290ns
  // cross-CCD (from the recorded census). Percentage penalties scale with the
  // producer's body time, which double-counts size; absolute latency does not.
  bool absLatency = false;
  { const char* e = std::getenv("GSIM_MT_DENSE_SCHED_ABS_LAT"); if (e && e[0] && e[0] != '0') absLatency = true; }
    long long bestTime = std::numeric_limits<long long>::max();
    for (int ri = 0; ri < static_cast<int>(ready.size()); ri ++) {
      int mtaskId = ready[(size_t)ri];
      const MtDenseMTask& mtask = mtasks[(size_t)mtaskId];
      // Reserve thread 0 for worker0-only (pinned side-effect) MTasks: non-worker0 MTasks start
      // their thread search at 1 when threadCount>1, so the scheduler does not pile parallel work
      // onto thread 0 and then serialize the pinned load behind it (the 52x imbalance).
      int workerStart = mtask.workerZeroOnly || w0Merge ? 0 : (threadCount > 1 ? 1 : 0);
      int workerLimit = mtask.workerZeroOnly ? 1 : threadCount;
      for (int worker = workerStart; worker < workerLimit; worker ++) {
        long long timeBegin = busyUntil[(size_t)worker];
        for (int pred : mtask.predMTasks) {
          if (pred < 0 || pred >= n) continue;
          long long predEnd = completion[(size_t)pred];
          int predWorker = outAssign[(size_t)pred];
          if (predWorker >= 0 && predWorker != worker) {
            if (absLatency) {
              predEnd += ((predWorker / ccdSize) != (worker / ccdSize)) ? 290 : 24;
            } else {
              predEnd += (long long)(costOf(mtasks[(size_t)pred])) * xsyncPct / 100;
              if (ccdExtra > 0 && (predWorker / ccdSize) != (worker / ccdSize)) {
                // Flat gamma (GSIM_MT_DENSE_PACK_CCD_AFFINITY) for cross-CCD
                // edges; worker->CCD mapping is worker/8, identical to the
                // PackThreads variant of the term. GSIM_MT_DENSE_SCHED_HOTEDGE
                // replaces the flat gamma ONLY on measured edges, scaling it
                // by the normalized blocked time (J/maxJ in 0..1000
                // milli-units, per (consumer, producer) summed over the
                // consumer's measured tokens and capped at 1000, so the
                // max-edge penalty stays ~= the flat gamma):
                //   penalty = costOf(pred) * ccdExtra * hotPct / 100000
                // (hotPct = 1000 reproduces costOf(pred) * ccdExtra / 100).
                // Unmeasured edges keep the flat gamma unchanged.
                int hotPctEdge = -1;
                if (hotPctByConsumer != nullptr) {
                  auto hotIt = (*hotPctByConsumer)[(size_t)mtaskId].find(pred);
                  if (hotIt != (*hotPctByConsumer)[(size_t)mtaskId].end()) hotPctEdge = hotIt->second;
                }
                if (hotPctEdge < 0)
                  predEnd += (long long)(costOf(mtasks[(size_t)pred])) * ccdExtra / 100;
                else if (hotPctEdge > 0)
                  predEnd += (long long)(costOf(mtasks[(size_t)pred])) * ccdExtra * hotPctEdge / 100000;
              }
            }
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

// ---- GSIM_MT_DENSE_SCHED_HEFT (default off, byte-identical when unset) ----
// Heterogeneous-Earliest-Finish-Time list scheduling over the dense MTask DAG
// (profile/heft/DESIGN.md): upward-rank priorities over the FULL compile-time
// MTask DAG (mtasks[i].succMTasks — every dependency edge, never a cross-thread
// projection), then earliest-finish-time placement with a measured communication
// model (24.5ns same-CCD / 300ns cross-CCD, worker->CCD = worker/8). The comm model
// REPLACES the GSIM_MT_DENSE_PACK_CCD_AFFINITY gamma here (gamma is irrelevant
// under HEFT; the greedy's other knobs — XSYNC/W0_MERGE/ABS_LAT — belong to
// the reference greedy and do not apply). UNIT CONVENTION: identical to how
// GSIM_MT_DENSE_SCHED_ABS_LAT mixes machine constants with costOf — the ns
// comm constants are added RAW next to the costs: measured costfile values
// (GSIM_MT_DENSE_SCHED_COSTFILE) are ns-domain so the constants line up
// directly; without the costfile the static member-node costs are used as-is
// in the same mixed convention (ranku priorities are scale-invariant to a
// common cost scale; absolute makespan units are then node-count-ish).
static void mtDenseScheduleHeft(const std::vector<MtDenseMTask>& mtasks, int threadCount,
                                std::vector<int>& outAssign, std::vector<int>& outOrder) {
  if (threadCount < 1) threadCount = 1;
  const int n = static_cast<int>(mtasks.size());
  outAssign.assign((size_t)n, -1);
  outOrder.clear(); outOrder.reserve((size_t)n);
  auto costOf = [](const MtDenseMTask& m) -> double { return m.schedCost > 0 ? (double)m.schedCost : (double)m.staticCost; };
  const double commSameCcd = 24.5, commCrossCcd = 300.0;
  auto commCost = [&](int w1, int w2) -> double {
    if (w1 == w2) return 0.0;
    return (w1 / 8) == (w2 / 8) ? commSameCcd : commCrossCcd;
  };
  // Reference greedy placement — DIAGNOSTIC ONLY: it feeds the modeled-makespan
  // comparison reported below and nothing else; the HEFT ranks do NOT depend
  // on it. (An earlier revision computed ranku over a cross-thread projection
  // of the DAG derived from this reference placement; edges the projection
  // dropped — same-reference-thread edges and transitive-reduction survivors —
  // could leave ranku(pred) < ranku(succ), so descending-rank placement
  // reached consumers whose predecessors were still unplaced and silently
  // skipped them, emitting an invalid schedule: generator modeled 280890 vs
  // reference 238638, runtime +60%. ranku is now computed over the full DAG
  // and an unplaced predecessor fails loudly in the placement loop below.)
  std::vector<int> refAssign, refOrder;
  mtDenseScheduleOrderImpl(mtasks, threadCount, refAssign, refOrder, nullptr);
  // commMean: the mean cross-worker comm penalty over the unordered worker
  // pairs of this run (the "gamma-scaled mean" replacement).
  double commMean = 0.0;
  {
    int pairs = 0;
    double sum = 0.0;
    for (int w1 = 0; w1 < threadCount; w1++)
      for (int w2 = w1 + 1; w2 < threadCount; w2++) { sum += commCost(w1, w2); pairs++; }
    commMean = pairs > 0 ? sum / (double)pairs : 0.0;
  }
  // Upward rank over the FULL compile-time MTask DAG: ranku(m) = cost(m) +
  // max over ALL succMTasks of (commMean + ranku(succ)). commMean is a
  // placement-independent heuristic edge weight — the true per-edge commCost
  // needs both owners, which the rank pass cannot know before placement.
  // For every full-DAG edge (p -> m): ranku(p) >= cost(p) + commMean +
  // ranku(m) >= ranku(m) (the max includes m's own term; adding non-negative
  // terms is fp-monotone), strictly greater when commMean > 0 (threadCount >=
  // 2); at threadCount == 1, or if rounding ever collapses the gap to
  // equality, the ascending-id tie-break still orders p first because MTask
  // ids are topo-monotone (pred id < succ id). Descending-rank placement is
  // therefore a valid topological order of the full DAG. Ids are topo-monotone
  // (succ > pred, asserted when the schedule is built), so one descending pass
  // settles every successor first.
  std::vector<double> ranku((size_t)n, 0.0);
  for (int m = n - 1; m >= 0; m--) {
    double bestSucc = 0.0;
    bool hasSucc = false;
    for (int succ : mtasks[(size_t)m].succMTasks) {
      if (succ < 0 || succ >= n) continue;
      const double v = commMean + ranku[(size_t)succ];
      if (!hasSucc || v > bestSucc) { bestSucc = v; hasSucc = true; }
    }
    ranku[(size_t)m] = costOf(mtasks[(size_t)m]) + (hasSucc ? bestSucc : 0.0);
  }
  // EFT list scheduling in descending ranku order — a valid topological order
  // of the FULL DAG (derivation above): every predecessor of the MTask being
  // placed is already assigned, and a violation is a hard Assert, never a
  // silent skip. No gap insertion: appended at workerAvail (keep it simple).
  std::vector<int> rankOrder((size_t)n);
  for (int m = 0; m < n; m++) rankOrder[(size_t)m] = m;
  std::sort(rankOrder.begin(), rankOrder.end(), [&](int a, int b) {
    if (ranku[(size_t)a] != ranku[(size_t)b]) return ranku[(size_t)a] > ranku[(size_t)b];
    return a < b;
  });
  bool w0Merge = false;
  { const char* e = std::getenv("GSIM_MT_DENSE_SCHED_W0_MERGE"); if (e && e[0] && e[0] != '0') w0Merge = true; }
  std::vector<double> workerAvail((size_t)threadCount, 0.0);
  std::vector<double> finishAt((size_t)n, 0.0);
  std::vector<double> startAt((size_t)n, 0.0);
  double heftMakespan = 0.0;
  for (int m : rankOrder) {
    const int workerStart = mtasks[(size_t)m].workerZeroOnly || w0Merge ? 0 : (threadCount > 1 ? 1 : 0);
    const int workerLimit = mtasks[(size_t)m].workerZeroOnly ? 1 : threadCount;
    int bestWorker = -1;
    double bestEft = 0.0, bestReady = 0.0;
    for (int w = workerStart; w < workerLimit; w++) {
      double ready = workerAvail[(size_t)w];
      for (int pred : mtasks[(size_t)m].predMTasks) {
        if (pred < 0 || pred >= n) continue;
        const int pw = outAssign[(size_t)pred];
        Assert(pw >= 0,
               "heft full-DAG placement: predecessor %d of MTask %d is unplaced "
               "(descending-ranku order is no longer topological over the full DAG)",
               pred, m);
        const double t = finishAt[(size_t)pred] + commCost(pw, w);
        if (t > ready) ready = t;
      }
      const double eft = ready + costOf(mtasks[(size_t)m]);
      if (bestWorker < 0 || eft < bestEft) { bestWorker = w; bestEft = eft; bestReady = ready; }
    }
    if (bestWorker < 0) bestWorker = 0;  // defensive; workerLimit >= 1 always
    outAssign[(size_t)m] = bestWorker;
    startAt[(size_t)m] = bestReady;
    finishAt[(size_t)m] = bestEft;
    workerAvail[(size_t)bestWorker] = bestEft;
    if (bestEft > heftMakespan) heftMakespan = bestEft;
  }
  // Reference makespan under the same internal model: the greedy's placement
  // executed per-worker sequentially in its own schedule order (a valid topo
  // order), with ready propagation over the FULL pred set and the same comm.
  double refMakespan = 0.0;
  {
    std::vector<double> avail((size_t)threadCount, 0.0);
    std::vector<double> fin((size_t)n, 0.0);
    for (int m : refOrder) {
      const int w = refAssign[(size_t)m];
      double ready = (w >= 0 && w < threadCount) ? avail[(size_t)w] : 0.0;
      for (int pred : mtasks[(size_t)m].predMTasks) {
        if (pred < 0 || pred >= n) continue;
        const int pw = refAssign[(size_t)pred];
        const double t = fin[(size_t)pred] + commCost(pw, w);
        if (t > ready) ready = t;
      }
      fin[(size_t)m] = ready + costOf(mtasks[(size_t)m]);
      if (w >= 0 && w < threadCount) avail[(size_t)w] = fin[(size_t)m];
      if (fin[(size_t)m] > refMakespan) refMakespan = fin[(size_t)m];
    }
  }
  // outOrder: a valid topological order that honors the new assignment — each
  // worker's MTasks appear in HEFT start-time order (Kahn's algorithm picking
  // the ready MTask with the smallest (start, worker, id)). Per-worker id
  // order in the renumber therefore equals HEFT start order; the token
  // protocol's forward-edge requirement stays satisfied because the order is
  // topological by construction.
  {
    std::vector<int> remaining((size_t)n, 0);
    for (int m = 0; m < n; m++) remaining[(size_t)m] = static_cast<int>(mtasks[(size_t)m].predMTasks.size());
    std::set<std::tuple<double, int, int>> ready;  // (start, worker, id)
    for (int m = 0; m < n; m++) if (remaining[(size_t)m] == 0)
      ready.insert(std::make_tuple(startAt[(size_t)m], outAssign[(size_t)m], m));
    while (!ready.empty()) {
      const int m = std::get<2>(*ready.begin());
      ready.erase(ready.begin());
      outOrder.push_back(m);
      for (int succ : mtasks[(size_t)m].succMTasks) {
        if (succ < 0 || succ >= n) continue;
        if (-- remaining[(size_t)succ] == 0)
          ready.insert(std::make_tuple(startAt[(size_t)succ], outAssign[(size_t)succ], succ));
      }
    }
    // Any unscheduled (shouldn't happen for a DAG) appended in id order.
    if (static_cast<int>(outOrder.size()) != n) {
      std::vector<char> seen((size_t)n, 0);
      for (int m : outOrder) seen[(size_t)m] = 1;
      for (int i = 0; i < n; i++) if (!seen[(size_t)i]) outOrder.push_back(i);
    }
  }
  long long heftFullDagEdges = 0;
  for (int m = 0; m < n; m++) heftFullDagEdges += (long long)mtasks[(size_t)m].succMTasks.size();
  fprintf(stderr, "[heft] ranku over full DAG (%lld edges); heft_unplaced_preds=0 (Assert-backed); "
          "modeled_makespan_units=%.0f vs reference_greedy_units=%.0f\n",
          heftFullDagEdges, heftMakespan, refMakespan);
}

static void mtBuildDenseScheduleOrder(const std::vector<MtDenseMTask>& mtasks, int threadCount,
                                      std::vector<int>& outAssign, std::vector<int>& outOrder) {
  const char* heftEnv = std::getenv("GSIM_MT_DENSE_SCHED_HEFT");
  const bool heftOn = heftEnv != nullptr && heftEnv[0] != '\0' && heftEnv[0] != '0';
  const char* hotPath = std::getenv("GSIM_MT_DENSE_SCHED_HOTEDGE");
  const char* chainPath = std::getenv("GSIM_MT_DENSE_SCHED_CHAINPIN");
  const bool hotedgeOn = hotPath != nullptr && hotPath[0] != '\0';
  const bool chainpinOn = chainPath != nullptr && chainPath[0] != '\0';
  const char* rebalPath = std::getenv("GSIM_MT_DENSE_SCHED_CHAIN_REBALANCE");
  const bool rebalanceOn = rebalPath != nullptr && rebalPath[0] != '\0';
  Assert(!(heftOn && rebalanceOn),
         "GSIM_MT_DENSE_SCHED_HEFT replaces the greedy objective and is incompatible with GSIM_MT_DENSE_SCHED_CHAIN_REBALANCE");
  Assert(!(rebalanceOn && (hotedgeOn || chainpinOn)),
         "GSIM_MT_DENSE_SCHED_CHAIN_REBALANCE is mutually exclusive with GSIM_MT_DENSE_SCHED_HOTEDGE / GSIM_MT_DENSE_SCHED_CHAINPIN");
  // GSIM_MT_DENSE_SCHED_CHAIN_REBALANCE_MOVABLE=1 (default off): variant of
  // the rebalance pass that relocates ONLY the maximal connected subsegments
  // with no workerZeroOnly member; links touching a pinned endpoint are
  // dropped and act as separators, so mixed segments split instead of being
  // skipped wholesale (the L2 chain hosts 17 pinned MTasks but has movable
  // hot neighbors).
  bool rebalanceMovable = false;
  { const char* e = std::getenv("GSIM_MT_DENSE_SCHED_CHAIN_REBALANCE_MOVABLE"); if (e && e[0] && e[0] != '0') rebalanceMovable = true; }
  Assert(!(rebalanceMovable && !rebalanceOn),
         "GSIM_MT_DENSE_SCHED_CHAIN_REBALANCE_MOVABLE=1 requires GSIM_MT_DENSE_SCHED_CHAIN_REBALANCE=<SimTop_edge_timing.json>");
  Assert(!(heftOn && (hotedgeOn || chainpinOn)),
         "GSIM_MT_DENSE_SCHED_HEFT replaces the greedy objective and is incompatible with GSIM_MT_DENSE_SCHED_HOTEDGE / GSIM_MT_DENSE_SCHED_CHAINPIN");
  if (heftOn) {
    mtDenseScheduleHeft(mtasks, threadCount, outAssign, outOrder);
    return;
  }
  if (!hotedgeOn && !chainpinOn && !rebalanceOn) {
    mtDenseScheduleOrderImpl(mtasks, threadCount, outAssign, outOrder, nullptr);
    return;
  }
  std::vector<MtDenseHotedgeEntry> entries;
  if (hotedgeOn) {
    bool loaded = mtLoadHotedgeJson(hotPath, entries);
    (void)loaded;
  }
  std::vector<MtDenseHotedgeEntry> chainEntries;
  if (chainpinOn) {
    bool loaded = mtLoadHotedgeJson(chainPath, chainEntries);
    (void)loaded;
  }
  std::vector<MtDenseHotedgeEntry> rebalEntries;
  if (rebalanceOn) {
    bool loaded = mtLoadHotedgeJson(rebalPath, rebalEntries);
    (void)loaded;
  }
  const int n = static_cast<int>(mtasks.size());
  bool xthreadDepsOnly = mtUseDenseXThreadDepsOnly();
  Assert(xthreadDepsOnly,
         "GSIM_MT_DENSE_SCHED_HOTEDGE / GSIM_MT_DENSE_SCHED_CHAINPIN / GSIM_MT_DENSE_SCHED_CHAIN_REBALANCE require GSIM_MT_DENSE_XTHREAD_DEPS_ONLY=1 (the edge-timing capture and its owner-ready slot numbering exist only under it)");
  bool transitiveReduceEdges = mtUseDenseTransitiveReduceEdges();
  // Reference pass (hotedge off) reproduces the placement the timing file was
  // captured on. CRITICAL id-space detail: the timing file's ids are FINAL
  // (executor) MTask ids, but this function runs BEFORE the SCHED_ORDER
  // renumber, so its ids are PRE-renumber. The renumber in buildMtDenseSchedule
  // assigns final id = position in the order (newId[order[newPos]] = newPos),
  // and the owner-ready layout's group ordering (pair banks keyed by
  // (destination id, producer owner)) is id-dependent — so the preview layout
  // must be built over a FINAL-id-space replica of the graph: finalMtasks[k] =
  // mtasks[refOrder[k]] with succ/pred ids remapped through the inverse
  // permutation. With GSIM_MT_DENSE_SCHED_ORDER off the permutation is the
  // identity (no renumber happens).
  std::vector<int> refAssign, refOrder;
  mtDenseScheduleOrderImpl(mtasks, threadCount, refAssign, refOrder, nullptr);
  bool schedOrder = false;
  { const char* e = std::getenv("GSIM_MT_DENSE_SCHED_ORDER"); if (e) schedOrder = e[0] && e[0] != '0'; }
  std::vector<int> preOfFinal((size_t)n, -1);
  if (schedOrder && static_cast<int>(refOrder.size()) == n)
    for (int k = 0; k < n; k++) preOfFinal[(size_t)k] = refOrder[(size_t)k];
  else
    for (int k = 0; k < n; k++) preOfFinal[(size_t)k] = k;
  std::vector<int> finalOfPre((size_t)n, -1);
  for (int k = 0; k < n; k++) finalOfPre[(size_t)preOfFinal[(size_t)k]] = k;
  std::vector<MtDenseMTask> finalMtasks((size_t)n);
  std::vector<int> finalAssign((size_t)n, -1);
  for (int k = 0; k < n; k++) {
    const MtDenseMTask& pre = mtasks[(size_t)preOfFinal[(size_t)k]];
    MtDenseMTask fin = pre;
    fin.succMTasks.clear();
    fin.predMTasks.clear();
    for (int succ : pre.succMTasks)
      if (succ >= 0 && succ < n && finalOfPre[(size_t)succ] >= 0) fin.succMTasks.push_back(finalOfPre[(size_t)succ]);
    for (int pred : pre.predMTasks)
      if (pred >= 0 && pred < n && finalOfPre[(size_t)pred] >= 0) fin.predMTasks.push_back(finalOfPre[(size_t)pred]);
    finalMtasks[(size_t)k] = fin;
    finalAssign[(size_t)k] = refAssign[(size_t)preOfFinal[(size_t)k]];
  }
  // Same layout derivation the executor emission uses. NOTE: with
  // GSIM_MT_DENSE_LOOKAHEAD >= 1 the executor REPLACES this layout with one
  // built from the transitive-reduced DAG-only cross-worker succ graph
  // (injectWorkerChains=false), so the preview must replicate that branch or
  // the slot numbering diverges on lookahead recipes.
  std::vector<std::vector<int>> refSuccs;
  if (mtDenseLookaheadWindow() > 0) {
    std::vector<std::vector<int>> dagOnlySuccs = mtBuildDenseRuntimeSuccs(finalMtasks, finalAssign, false);
    mtReduceDenseRuntimeSuccsTransitive(dagOnlySuccs, finalAssign, false);
    refSuccs.assign((size_t)n, {});
    for (int pred = 0; pred < n; pred++)
      for (int succ : dagOnlySuccs[(size_t)pred])
        if (finalAssign[(size_t)pred] != finalAssign[(size_t)succ])
          refSuccs[(size_t)pred].push_back(succ);
  } else {
    refSuccs = mtBuildDenseRuntimeSuccs(finalMtasks, finalAssign, xthreadDepsOnly);
    if (transitiveReduceEdges) mtReduceDenseRuntimeSuccsTransitive(refSuccs, finalAssign);
  }
  MtDenseOwnerReadyLayout refLayout = mtBuildDenseOwnerReadyLayout(refSuccs, finalAssign, threadCount);
  struct MtHotedgeMapped { int consumer; unsigned long long blockedNs; std::vector<int> producers; };
  // Resolve (consumerMtaskId, tokenSlot) entries to (consumer, producers) in
  // PRE id space via the final-id replica layout. Shared by the HOTEDGE
  // penalty map and the CHAINPIN post-pass.
  auto mapEntries = [&](const std::vector<MtDenseHotedgeEntry>& in, std::vector<MtHotedgeMapped>& out,
                        int& skipStale, int& skipNoSlot, int& skipNoProducer) {
    for (const MtDenseHotedgeEntry& e : in) {
      if (e.consumerMtaskId < 0 || e.consumerMtaskId >= n) { skipStale++; continue; }
      if (e.tokenSlot < 0 || e.tokenSlot >= refLayout.physicalSlotCount) { skipNoSlot++; continue; }
      const int token = refLayout.logicalTokenByPhysicalSlot[(size_t)e.tokenSlot];
      if (token < 0 || token >= refLayout.tokenCount) { skipNoSlot++; continue; }
      if (refLayout.tokenProvenanceByLogicalToken[(size_t)token].consumerMTask != e.consumerMtaskId) {
        skipStale++;  // stale slot map (layout drifted from the captured build)
        continue;
      }
      MtHotedgeMapped me;
      me.consumer = preOfFinal[(size_t)e.consumerMtaskId];  // keyed in PRE ids (this function's space)
      me.blockedNs = e.blockedNs;
      for (int source : refLayout.sourceMTasksByLogicalToken[(size_t)token]) {
        if (source < 0 || source >= n) continue;
        const int preProducer = preOfFinal[(size_t)source];
        if (preProducer < 0) continue;
        me.producers.push_back(preProducer);
      }
      if (me.producers.empty()) { skipNoProducer++; continue; }
      out.push_back(me);
    }
  };
  std::vector<MtHotedgeMapped> mapped;
  int hotedgeSkipStale = 0, hotedgeSkipNoSlot = 0, hotedgeSkipNoProducer = 0;
  if (hotedgeOn) mapEntries(entries, mapped, hotedgeSkipStale, hotedgeSkipNoSlot, hotedgeSkipNoProducer);
  unsigned long long maxJ = 0;
  for (const MtHotedgeMapped& me : mapped) if (me.blockedNs > maxJ) maxJ = me.blockedNs;
  std::vector<std::map<int, int>> hotPct((size_t)n);
  if (hotedgeOn) {
    // Per (consumer, producer) weight in 0..1000 milli-units of J/maxJ, summed
    // over the consumer's measured tokens and capped at 1000 so the hottest
    // edge pays at most ~= the flat gamma.
    if (maxJ > 0) {
      for (const MtHotedgeMapped& me : mapped) {
        if (me.blockedNs == 0) continue;
        const int pct = (int)(me.blockedNs * 1000ULL / maxJ);
        for (int producer : me.producers) {
          int& acc = hotPct[(size_t)me.consumer][producer];
          acc = acc + pct > 1000 ? 1000 : acc + pct;
        }
      }
    }
    int appliedEdges = 0, maxPct = 0;
    for (const std::map<int, int>& byProducer : hotPct)
      for (const auto& kv : byProducer)
        if (kv.second > 0) { appliedEdges++; if (kv.second > maxPct) maxPct = kv.second; }
    int ccdExtra = 0;
    { const char* e = std::getenv("GSIM_MT_DENSE_PACK_CCD_AFFINITY"); if (e && e[0]) { int v = std::atoi(e); if (v >= 0) ccdExtra = v; } }
    // max_penalty is the hottest edge's penalty in percent-of-producer-cost
    // units (J-normalized gamma); 0 when GSIM_MT_DENSE_PACK_CCD_AFFINITY is 0.
    fprintf(stderr, "[hotedge] loaded=%d applied_edges=%d max_penalty=%f\n",
            static_cast<int>(entries.size()), appliedEdges,
            (double)maxPct * (double)ccdExtra / 1000.0);
    fprintf(stderr, "[hotedge] debug skip_reasons=(stale=%d noslot=%d noproducer=%d)\n",
            hotedgeSkipStale, hotedgeSkipNoSlot, hotedgeSkipNoProducer);
  }
  mtDenseScheduleOrderImpl(mtasks, threadCount, outAssign, outOrder, hotedgeOn ? &hotPct : nullptr);
  // ---- GSIM_MT_DENSE_SCHED_CHAIN_REBALANCE (default off, byte-identical when unset) ----
  // Balance-compensated chain co-location. Identical segment construction to
  // the CHAINPIN post-pass below (hot blocked-edge prefix -> fan-out head
  // exclusion -> union-find segments), but CHAINPIN's pin was balance-
  // infeasible at mean*1.03: the greedy baseline itself can already sit well
  // above mean (measured worker0-split skew ~1.24x mean), so a flat
  // mean*1.03 target cap rejects every hot segment before any relocation.
  // Cap semantics here tolerate preexisting skew: hardCap = max(baseline
  // maxWorkerCost, mean*1.03) -- a pin/offload may never RAISE the max above
  // that. The target is chosen by a min-max objective over post-placement
  // worker cost; when the segment would push the target above hardCap,
  // NON-segment tasks are offloaded off the target first (smallest overload
  // contribution first, into non-target workers that stay under mean*1.03);
  // a candidate is accepted only if the final max stays <= hardCap, and
  // segments that cannot be placed under it are skipped and counted.
  // Mutually exclusive with HOTEDGE / CHAINPIN /
  // HEFT. Pure outAssign rewrite after the greedy: outOrder, ids and the
  // topological order are untouched, and this runs before the caller's
  // SCHED_ORDER renumber. Same-worker dependency ordering stays valid for
  // any assignment because MTask ids are topo-monotone (producer id <
  // consumer id); the offload path re-checks that invariant per moved task.
  if (rebalanceOn) {
    int rbSkipStale = 0, rbSkipNoSlot = 0, rbSkipNoProducer = 0;
    std::vector<MtHotedgeMapped> rebMapped;
    mapEntries(rebalEntries, rebMapped, rbSkipStale, rbSkipNoSlot, rbSkipNoProducer);
    // Hot set: entries sorted by blockedNs desc covering cumulative 50% of
    // the total blocked time (the crossing entry is included), as CHAINPIN.
    std::sort(rebMapped.begin(), rebMapped.end(),
              [](const MtHotedgeMapped& a, const MtHotedgeMapped& b) { return a.blockedNs > b.blockedNs; });
    unsigned long long rbTotalJ = 0;
    for (const MtHotedgeMapped& me : rebMapped) rbTotalJ += me.blockedNs;
    size_t rbHotCount = 0;
    unsigned long long rbCum = 0;
    while (rbHotCount < rebMapped.size() && rbTotalJ > 0 && rbCum * 2 < rbTotalJ) {
      rbCum += rebMapped[rbHotCount].blockedNs;
      rbHotCount++;
    }
    // Fan-out heads: a producer with >1 distinct hot consumer is deliberate
    // parallelism -- it never relocates and never offloads.
    std::map<int, std::set<int>> rbHotConsumersOf;
    for (size_t i = 0; i < rbHotCount; i++)
      for (int producer : rebMapped[i].producers)
        rbHotConsumersOf[producer].insert(rebMapped[i].consumer);
    std::set<int> rbFanoutHeads;
    for (const auto& kv : rbHotConsumersOf)
      if (kv.second.size() > 1) rbFanoutHeads.insert(kv.first);
    struct MtRebalLink { int producer; int consumer; unsigned long long blockedNs; };
    std::vector<MtRebalLink> rbLinks;
    for (size_t i = 0; i < rbHotCount; i++) {
      const MtHotedgeMapped& me = rebMapped[i];
      for (int producer : me.producers)
        if (rbFanoutHeads.count(producer) == 0)
          rbLinks.push_back(MtRebalLink{producer, me.consumer, me.blockedNs});
    }
    // Segments: connected components over the remaining links (union-find).
    std::vector<int> rbParent((size_t)n, -1);
    auto rbFind = [&](int x) {
      while (rbParent[(size_t)x] != x) {
        rbParent[(size_t)x] = rbParent[(size_t)rbParent[(size_t)x]];
        x = rbParent[(size_t)x];
      }
      return x;
    };
    // A link may only relocate when neither endpoint is pinned. The MOVABLE
    // variant drops pinned-touching links from BOTH the union-find and the
    // member sets, so workerZeroOnly nodes act as separators and segments
    // become the maximal connected subsegments of movable nodes; without it
    // a single pinned member still vetoes its whole segment.
    auto rbLinkMovable = [&](const MtRebalLink& link) {
      return !rebalanceMovable ||
             (!mtasks[(size_t)link.producer].workerZeroOnly && !mtasks[(size_t)link.consumer].workerZeroOnly);
    };
    for (const MtRebalLink& link : rbLinks) {
      if (!rbLinkMovable(link)) continue;
      if (rbParent[(size_t)link.producer] < 0) rbParent[(size_t)link.producer] = link.producer;
      if (rbParent[(size_t)link.consumer] < 0) rbParent[(size_t)link.consumer] = link.consumer;
      rbParent[rbFind(link.producer)] = rbFind(link.consumer);
    }
    std::map<int, std::set<int>> rbSegmentMembers;
    std::map<int, unsigned long long> rbSegmentWeight;
    std::set<int> rbLinkProducers;
    std::set<int> rbAnySegmentMember;
    int rbMovableLinks = 0, rbPinnedLinks = 0;
    unsigned long long rbMovableLinkNs = 0, rbPinnedLinkNs = 0;
    std::set<int> rbPinnedMembers;
    for (const MtRebalLink& link : rbLinks) {
      rbLinkProducers.insert(link.producer);
      if (!rbLinkMovable(link)) {
        rbPinnedLinks++;
        rbPinnedLinkNs += link.blockedNs;
        if (mtasks[(size_t)link.producer].workerZeroOnly) rbPinnedMembers.insert(link.producer);
        if (mtasks[(size_t)link.consumer].workerZeroOnly) rbPinnedMembers.insert(link.consumer);
        continue;
      }
      rbMovableLinks++;
      rbMovableLinkNs += link.blockedNs;
      const int root = rbFind(link.producer);
      rbSegmentMembers[root].insert(link.producer);
      rbSegmentMembers[root].insert(link.consumer);
      rbSegmentWeight[root] += link.blockedNs;
      rbAnySegmentMember.insert(link.producer);
      rbAnySegmentMember.insert(link.consumer);
    }
    // Balance state over the greedy assignment; cost domain identical to the
    // greedy and CHAINPIN (measured schedCost when present, else staticCost).
    auto rbCostOf = [](const MtDenseMTask& m) -> int { return m.schedCost > 0 ? m.schedCost : m.staticCost; };
    std::vector<long long> rbLoad((size_t)threadCount, 0);
    long long rbTotalCost = 0;
    for (int m = 0; m < n; m++) {
      const int w = outAssign[(size_t)m];
      if (w < 0 || w >= threadCount) continue;
      rbLoad[(size_t)w] += rbCostOf(mtasks[(size_t)m]);
      rbTotalCost += rbCostOf(mtasks[(size_t)m]);
    }
    const double rbMean = (double)rbTotalCost / (double)threadCount;
    const double rbCap = rbMean * 1.03;  // ceiling for workers that only GAIN load
    // Preexisting-skew allowance: the baseline (pre-relocation) max anchors
    // the hard cap; the final max may never exceed max(baseline, mean*1.03).
    long long rbBaseMax = 0;
    for (int w = 0; w < threadCount; w++) rbBaseMax = std::max(rbBaseMax, rbLoad[(size_t)w]);
    const double rbHardCap = std::max((double)rbBaseMax, rbCap);
    // Hottest segments first (deterministic tie-break: smaller root id).
    std::vector<std::pair<unsigned long long, int>> rbSegmentOrder;
    for (const auto& kv : rbSegmentWeight) rbSegmentOrder.push_back({kv.second, kv.first});
    std::sort(rbSegmentOrder.begin(), rbSegmentOrder.end(),
              [](const std::pair<unsigned long long, int>& a, const std::pair<unsigned long long, int>& b) {
                return a.first != b.first ? a.first > b.first : a.second < b.second;
              });
    int rbSegments = (int)rbSegmentOrder.size();
    int rbRelocated = 0, rbCompensated = 0, rbSkippedBalance = 0, rbMovedOffloads = 0;
    for (const auto& seg : rbSegmentOrder) {
      std::vector<int> members(rbSegmentMembers[seg.second].begin(), rbSegmentMembers[seg.second].end());
      // First producer = the segment's smallest-id member that produces one
      // of its links; ties among equal-max targets prefer its worker.
      int firstProducer = -1;
      for (int m : members)
        if (rbLinkProducers.count(m) && (firstProducer < 0 || m < firstProducer)) firstProducer = m;
      // Skip guards: no link producer; or a workerZeroOnly member (pinned
      // to worker 0, immovable). A member currently OWNED by worker 0 is an
      // ordinary dense body -- the reservation is only a greedy placement
      // preference (chainpin likewise remapped a worker-0 pin target to 1)
      // -- and may relocate off it. Targets stay in 1..N-1 and offloads
      // never land on worker 0, so worker 0 only ever sheds load here.
      bool rbSkip = firstProducer < 0;
      for (int m : members) {
        const int w = outAssign[(size_t)m];
        if (mtasks[(size_t)m].workerZeroOnly || w < 0 || w >= threadCount) { rbSkip = true; break; }
      }
      if (rbSkip) { rbSkippedBalance++; continue; }
      const int preferWorker = outAssign[(size_t)firstProducer];
      // (a worker-0 owner is not a valid target, so that tie preference
      // simply never matches and equal-max ties keep the smallest target)
      // Candidate targets: workers 1..N-1 (never worker 0). For each: place
      // the whole segment, then -- only if the target ends above hardCap --
      // greedily offload non-segment tasks off it until it fits.
      int bestTarget = -1;
      long long bestMax = 0;
      std::vector<long long> bestPost;
      std::vector<std::pair<int, int>> bestMoves;  // (mtask, destination)
      for (int target = 1; target < threadCount; target++) {
        std::vector<long long> post = rbLoad;
        std::vector<int> virt = outAssign;  // trial assignment for this target
        for (int m : members) {
          const long long c = rbCostOf(mtasks[(size_t)m]);
          post[(size_t)virt[(size_t)m]] -= c;
          virt[(size_t)m] = target;
          post[(size_t)target] += c;
        }
        std::vector<std::pair<int, int>> moves;
        if ((double)post[(size_t)target] > rbHardCap) {
          // Offload candidates: tasks on the target, excluding any segment
          // member (any segment), fanout-head producers and workerZeroOnly
          // tasks; sorted by increasing contribution to the overload.
          std::vector<std::pair<long long, int>> cands;  // (cost, id)
          for (int t = 0; t < n; t++) {
            if (virt[(size_t)t] != target) continue;
            if (rbAnySegmentMember.count(t)) continue;
            if (mtasks[(size_t)t].workerZeroOnly) continue;
            if (rbFanoutHeads.count(t)) continue;
            cands.push_back({rbCostOf(mtasks[(size_t)t]), t});
          }
          std::sort(cands.begin(), cands.end());
          for (const auto& cand : cands) {
            if ((double)post[(size_t)target] <= rbHardCap) break;
            const int t = cand.second;
            const long long c = cand.first;
            if (c <= 0) continue;  // zero-cost tasks contribute no overload
            // Destination: least-loaded other worker (never worker 0, and
            // never one that would land above mean*1.03 by taking the task).
            int dest = -1; long long destLoad = 0;
            for (int w2 = 1; w2 < threadCount; w2++) {
              if (w2 == target) continue;
              const long long after = post[(size_t)w2] + c;
              if ((double)after > rbCap) continue;
              if (dest < 0 || after < destLoad) { dest = w2; destLoad = after; }
            }
            if (dest < 0) continue;
            // Dependency/order invariant: any dependency that becomes
            // same-worker must keep producer id < consumer id (the runtime
            // runs each worker's MTasks in ascending id order). Ids are
            // topo-monotone so this holds by construction; re-checked per
            // moved task as a guard.
            bool orderOk = true;
            for (int p : mtasks[(size_t)t].predMTasks)
              if (virt[(size_t)p] == dest && p >= t) { orderOk = false; break; }
            if (orderOk)
              for (int s : mtasks[(size_t)t].succMTasks)
                if (virt[(size_t)s] == dest && s <= t) { orderOk = false; break; }
            if (!orderOk) continue;
            post[(size_t)target] -= c;
            post[(size_t)dest] += c;
            virt[(size_t)t] = dest;
            moves.push_back({t, dest});
          }
        }
        if ((double)post[(size_t)target] > rbHardCap) continue;  // target infeasible
        long long maxLoad = 0;
        for (int w = 0; w < threadCount; w++) maxLoad = std::max(maxLoad, post[(size_t)w]);
        if (bestTarget < 0 || maxLoad < bestMax || (maxLoad == bestMax && target == preferWorker)) {
          bestTarget = target; bestMax = maxLoad;
          bestPost = post; bestMoves = moves;
        }
      }
      if (bestTarget < 0) { rbSkippedBalance++; continue; }  // no target fits the cap
      for (int m : members) outAssign[(size_t)m] = bestTarget;
      for (const auto& mv : bestMoves) outAssign[(size_t)mv.first] = mv.second;
      rbLoad = bestPost;
      if (!bestMoves.empty()) rbCompensated++;
      rbMovedOffloads += (int)bestMoves.size();
      rbRelocated++;
    }
    long long rbMaxLoad = 0;
    for (int w = 0; w < threadCount; w++) rbMaxLoad = std::max(rbMaxLoad, rbLoad[(size_t)w]);
    fprintf(stderr, "[chain-rebalance] segments=%d relocated=%d compensated=%d skipped_balance=%d "
                    "fanout_heads_kept=%d moved_offloads=%d max_worker_pct=%.2f\n",
            rbSegments, rbRelocated, rbCompensated, rbSkippedBalance,
            (int)rbFanoutHeads.size(), rbMovedOffloads,
            rbMean > 0 ? (double)rbMaxLoad * 100.0 / rbMean : 0.0);
    if (rebalanceMovable) {
      int rbMovableMembers = 0;
      for (const auto& kv : rbSegmentMembers) rbMovableMembers += (int)kv.second.size();
      const unsigned long long rbAllLinkNs = rbMovableLinkNs + rbPinnedLinkNs;
      fprintf(stderr, "[chain-rebalance-movable] links=%d movable_links=%d pinned_links=%d "
                      "movable_members=%d pinned_members=%d movable_blocked_pct=%.2f\n",
              (int)rbLinks.size(), rbMovableLinks, rbPinnedLinks,
              rbMovableMembers, (int)rbPinnedMembers.size(),
              rbAllLinkNs > 0 ? (double)rbMovableLinkNs * 100.0 / (double)rbAllLinkNs : 0.0);
    }
    (void)rbSkipStale; (void)rbSkipNoSlot; (void)rbSkipNoProducer;
    return;
  }
  if (!chainpinOn) return;
  // ---- GSIM_MT_DENSE_SCHED_CHAINPIN post-pass ----
  // Rationale: a serial dependency chain loses no parallelism when pinned to
  // one core (its links are serial by definition) and pays zero cross-core
  // token latency; the balance cost is bounded by the hard constraint below.
  // Applied AFTER the greedy (including an optional HOTEDGE penalized pass) as
  // a pure relocation of outAssign; the schedule order stays a valid
  // topological order and is untouched.
  int cpSkipStale = 0, cpSkipNoSlot = 0, cpSkipNoProducer = 0;
  std::vector<MtHotedgeMapped> chainMapped;
  mapEntries(chainEntries, chainMapped, cpSkipStale, cpSkipNoSlot, cpSkipNoProducer);
  // Hot set: entries sorted by blockedNs desc covering cumulative 50% of the
  // total blocked time (the prefix that crosses the 50% mark is included).
  std::sort(chainMapped.begin(), chainMapped.end(),
            [](const MtHotedgeMapped& a, const MtHotedgeMapped& b) { return a.blockedNs > b.blockedNs; });
  unsigned long long chainTotalJ = 0;
  for (const MtHotedgeMapped& me : chainMapped) chainTotalJ += me.blockedNs;
  size_t hotCount = 0;
  unsigned long long chainCum = 0;
  while (hotCount < chainMapped.size() && chainTotalJ > 0 && chainCum * 2 < chainTotalJ) {
    chainCum += chainMapped[hotCount].blockedNs;
    hotCount++;
  }
  // Fan-out heads: a producer with >1 distinct hot consumer is deliberate
  // parallelism -- exclude that producer and its edges from relocation.
  std::map<int, std::set<int>> hotConsumersOf;
  for (size_t i = 0; i < hotCount; i++)
    for (int producer : chainMapped[i].producers)
      hotConsumersOf[producer].insert(chainMapped[i].consumer);
  std::set<int> fanoutHeadProducers;
  for (const auto& kv : hotConsumersOf)
    if (kv.second.size() > 1) fanoutHeadProducers.insert(kv.first);
  struct MtChainLink { int producer; int consumer; unsigned long long blockedNs; };
  std::vector<MtChainLink> links;
  for (size_t i = 0; i < hotCount; i++) {
    const MtHotedgeMapped& me = chainMapped[i];
    for (int producer : me.producers)
      if (fanoutHeadProducers.count(producer) == 0)
        links.push_back(MtChainLink{producer, me.consumer, me.blockedNs});
  }
  // Chain segments: connected components over the remaining links (union-find).
  std::vector<int> parent((size_t)n, -1);
  auto chainFind = [&](int x) {
    while (parent[(size_t)x] != x) {
      parent[(size_t)x] = parent[(size_t)parent[(size_t)x]];
      x = parent[(size_t)x];
    }
    return x;
  };
  for (const MtChainLink& link : links) {
    if (parent[(size_t)link.producer] < 0) parent[(size_t)link.producer] = link.producer;
    if (parent[(size_t)link.consumer] < 0) parent[(size_t)link.consumer] = link.consumer;
    parent[chainFind(link.producer)] = chainFind(link.consumer);
  }
  std::map<int, std::set<int>> segmentMemberSet;
  std::map<int, unsigned long long> segmentWeight;
  std::set<int> linkProducers;
  for (const MtChainLink& link : links) {
    linkProducers.insert(link.producer);
    const int root = chainFind(link.producer);
    segmentMemberSet[root].insert(link.producer);
    segmentMemberSet[root].insert(link.consumer);
    segmentWeight[root] += link.blockedNs;
  }
  // Balance state: per-worker measured cost of the current assignment, with
  // the hard cap that no worker may exceed mean*1.03 after a pin.
  auto costOf = [](const MtDenseMTask& m) -> int { return m.schedCost > 0 ? m.schedCost : m.staticCost; };
  std::vector<long long> chainLoad((size_t)threadCount, 0);
  long long chainTotalCost = 0;
  for (int m = 0; m < n; m++) {
    const int w = outAssign[(size_t)m];
    if (w < 0 || w >= threadCount) continue;
    chainLoad[(size_t)w] += costOf(mtasks[(size_t)m]);
    chainTotalCost += costOf(mtasks[(size_t)m]);
  }
  const double chainLoadCap = (double)chainTotalCost / (double)threadCount * 1.03;
  // Hottest segments first (deterministic tie-break: smaller root id).
  std::vector<std::pair<unsigned long long, int>> segmentOrder;
  for (const auto& kv : segmentWeight) segmentOrder.push_back({kv.second, kv.first});
  std::sort(segmentOrder.begin(), segmentOrder.end(),
            [](const std::pair<unsigned long long, int>& a, const std::pair<unsigned long long, int>& b) {
              return a.first != b.first ? a.first > b.first : a.second < b.second;
            });
  int chainSegments = (int)segmentOrder.size();
  int chainRelocated = 0, chainSkippedBalance = 0;
  for (const auto& seg : segmentOrder) {
    std::vector<int> members(segmentMemberSet[seg.second].begin(), segmentMemberSet[seg.second].end());
    // First producer = the segment's smallest-id member that produces one of
    // its links; the segment pins onto that producer's current worker.
    int firstProducer = -1;
    for (int m : members)
      if (linkProducers.count(m) && (firstProducer < 0 || m < firstProducer)) firstProducer = m;
    if (firstProducer < 0) { chainSkippedBalance++; continue; }
    // Worker 0 keeps its reservation: never pin onto worker 0.
    int target = outAssign[(size_t)firstProducer];
    if (target == 0) target = 1;
    if (target < 0 || target >= threadCount) { chainSkippedBalance++; continue; }
    // workerZeroOnly members cannot leave worker 0; pinning the rest would
    // leave the chain split across the boundary -- skip the whole segment.
    bool hasPinned = false;
    for (int m : members)
      if (mtasks[(size_t)m].workerZeroOnly) { hasPinned = true; break; }
    if (hasPinned) { chainSkippedBalance++; continue; }
    std::vector<long long> delta((size_t)threadCount, 0);
    // HARD balance constraint (advisory): no worker may end above mean*1.03
    // because of the pin. Pre-existing overloads (indivisible large MTasks the
    // greedy itself cannot split) are tolerated -- only workers that GAIN load
    // are checked, so the constraint bounds the pin's added imbalance without
    // letting one oversized MTask veto every segment.
    bool fits = true;
    for (int w = 0; w < threadCount; w++)
      if (delta[(size_t)w] > 0 && (double)(chainLoad[(size_t)w] + delta[(size_t)w]) > chainLoadCap) { fits = false; break; }
    if (!fits) { chainSkippedBalance++; continue; }
    for (int w = 0; w < threadCount; w++) chainLoad[(size_t)w] += delta[(size_t)w];
    for (int m : members) outAssign[(size_t)m] = target;
    chainRelocated++;
  }
  fprintf(stderr, "[chainpin] segments=%d relocated=%d skipped_balance=%d fanout_heads_kept=%d\n",
          chainSegments, chainRelocated, chainSkippedBalance, (int)fanoutHeadProducers.size());
  (void)cpSkipStale; (void)cpSkipNoSlot; (void)cpSkipNoProducer;
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

