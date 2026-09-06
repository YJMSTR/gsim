// cppEmitterMtPlan.cpp - split from cppEmitter.cpp (pure code motion):
// coarse-region planning (layers/mtasks/profitability/admission/assignment),
// pure-batch planning, and the context cache group: mtContextCache,
// mtDenseScheduleCache(+Valid) and the dependency/active edge caches (inline in
// the impl header) with resetMtContextCache() and the *ForInvocation memos as the
// ONE cohesive write side. mtDenseScheduleCache's write side hands the schedule to
// the driver via move from dumpMtDenseScheduleJson (unchanged).
#include "cppEmitterImpl.h"

static void mtAddCoarseLayers(MtCoarseRegion& region) {
  int n = region.endCppId - region.beginCppId;
  if (n <= 0) return;
  std::vector<int> indegree(n, 0);
  std::vector<std::vector<int>> edges(n);
  for (int from = region.beginCppId; from < region.endCppId; from ++) {
    for (int to = region.beginCppId; to < region.endCppId; to ++) {
      if (from == to) continue;
      if (!mtTaskHasOrderingEdgeTo(from, to)) continue;
      int fromIndex = from - region.beginCppId;
      int toIndex = to - region.beginCppId;
      edges[fromIndex].push_back(toIndex);
      indegree[toIndex] ++;
    }
  }

  std::vector<bool> emitted(n, false);
  int emittedCount = 0;
  while (emittedCount < n) {
    MtCoarseLayer layer;
    for (int i = 0; i < n; i ++) {
      if (!emitted[i] && indegree[i] == 0) layer.taskCppIds.push_back(region.beginCppId + i);
    }
    if (layer.taskCppIds.empty()) {
      mtAddCoarseBlocker(region, "data_dependency");
      mtAddCoarseBlocker(region, "codegen_runtime_limit");
      region.layers.clear();
      region.runtimeEligible = false;
      region.estimatedLayerCount = 0;
      region.estimatedMaxParallelWidth = 0;
      return;
    }
    for (int cppId : layer.taskCppIds) {
      int index = cppId - region.beginCppId;
      if (emitted[index]) continue;
      emitted[index] = true;
      emittedCount ++;
      for (int toIndex : edges[index]) indegree[toIndex] --;
    }
    region.estimatedMaxParallelWidth = std::max(region.estimatedMaxParallelWidth, static_cast<int>(layer.taskCppIds.size()));
    region.layers.push_back(layer);
  }
  region.estimatedLayerCount = static_cast<int>(region.layers.size());
}

static void mtAddCoarseMTasks(MtCoarseRegion& region, const std::map<int, MtTaskInfo>& tasks) {
  region.mtasks.clear();
  int n = region.endCppId - region.beginCppId;
  if (n <= 0 || region.layers.empty()) return;

  std::vector<int> parent(n);
  for (int i = 0; i < n; i ++) parent[i] = i;
  auto findRoot = [&](int value) {
    int root = value;
    while (parent[root] != root) root = parent[root];
    while (parent[value] != value) {
      int next = parent[value];
      parent[value] = root;
      value = next;
    }
    return root;
  };
  auto unite = [&](int a, int b) {
    int rootA = findRoot(a);
    int rootB = findRoot(b);
    if (rootA != rootB) parent[rootB] = rootA;
  };

  for (int from = region.beginCppId; from < region.endCppId; from ++) {
    for (int to = region.beginCppId; to < region.endCppId; to ++) {
      if (from == to) continue;
      if (!mtTaskHasOrderingEdgeTo(from, to)) continue;
      unite(from - region.beginCppId, to - region.beginCppId);
    }
  }

  std::map<int, int> groupIndexByRoot;
  std::map<int, int> layerIndexByCppId;
  for (size_t layerIdx = 0; layerIdx < region.layers.size(); layerIdx ++) {
    for (int cppId : region.layers[layerIdx].taskCppIds) {
      int local = cppId - region.beginCppId;
      int root = findRoot(local);
      if (groupIndexByRoot.find(root) == groupIndexByRoot.end()) {
        int groupIndex = static_cast<int>(region.mtasks.size());
        groupIndexByRoot[root] = groupIndex;
        region.mtasks.push_back(MtCoarseMTask());
      }
      int groupIndex = groupIndexByRoot[root];
      while (region.mtasks[groupIndex].layerTaskCppIds.size() <= layerIdx) {
        region.mtasks[groupIndex].layerTaskCppIds.push_back(std::vector<int>());
      }
      region.mtasks[groupIndex].layerTaskCppIds[layerIdx].push_back(cppId);
      region.mtasks[groupIndex].taskCount ++;
      region.mtasks[groupIndex].staticCost += mtTaskEstimatedCost(tasks, cppId);
      auto superIter = cppId2Super.find(cppId);
      if (superIter != cppId2Super.end() && superIter->second) {
        region.mtasks[groupIndex].memberNodeCost += static_cast<int>(superIter->second->member.size());
      }
      layerIndexByCppId[cppId] = static_cast<int>(layerIdx);
    }
  }

  for (int from = region.beginCppId; from < region.endCppId; from ++) {
    for (int to = region.beginCppId; to < region.endCppId; to ++) {
      if (from == to) continue;
      if (!mtTaskHasOrderingEdgeTo(from, to)) continue;
      int fromRoot = findRoot(from - region.beginCppId);
      int toRoot = findRoot(to - region.beginCppId);
      if (fromRoot != toRoot) continue;
      int groupIndex = groupIndexByRoot[fromRoot];
      region.mtasks[groupIndex].orderingEdgeCount ++;
    }
  }
}

static void mtFinalizeCoarseProfitability(MtCoarseRegion& region);

struct MtCoarsePlannerTimerStats {
  uint64_t regionBuildUs = 0;
  uint64_t scanBeginEnterUs = 0;
  uint64_t scanBeginSameWordUs = 0;
  uint64_t scanEndEnterUs = 0;
  uint64_t scanEndSameWordUs = 0;
  uint64_t scanReverseIntoRegionUs = 0;
  uint64_t regionCostUs = 0;
  uint64_t taskScanUs = 0;
  uint64_t edgeScanUs = 0;
  uint64_t blockerFinalizeUs = 0;
  uint64_t layerUs = 0;
  uint64_t mtaskUs = 0;
  uint64_t profitabilityUs = 0;
};

static MtCoarsePlannerTimerStats mtCoarsePlannerTimerStats;

static struct timeval mtCoarsePlannerTimerStart() {
  if (!globalConfig.MtReportTimers) {
    struct timeval empty = {0, 0};
    return empty;
  }
  return getTime();
}

static uint64_t mtCoarsePlannerElapsedUs(struct timeval start, struct timeval end) {
  int64_t elapsedUs = (static_cast<int64_t>(end.tv_sec - start.tv_sec) * 1000000ll) +
                      static_cast<int64_t>(end.tv_usec - start.tv_usec);
  if (elapsedUs < 0) elapsedUs = 0;
  return static_cast<uint64_t>(elapsedUs);
}

static void mtCoarsePlannerTimerAdd(uint64_t& totalUs, struct timeval start) {
  if (!globalConfig.MtReportTimers) return;
  totalUs += mtCoarsePlannerElapsedUs(start, getTime());
}

static void logMtReportTimerUs(const char* name, uint64_t elapsedUs) {
  if (!globalConfig.MtReportTimers) return;
  printf("[mt-report-timer] %s = %llu ms\n", name, static_cast<unsigned long long>(elapsedUs / 1000ull));
}


static MtCoarseRegion mtBuildCoarseRegion(const std::map<int, MtTaskInfo>& tasks, int beginCppId, int endCppId) {
  MtCoarseRegion region;
  region.beginCppId = beginCppId;
  region.endCppId = endCppId;
  region.beginActiveWord = beginCppId / ACTIVE_WIDTH;
  region.endActiveWord = (endCppId - 1) / ACTIVE_WIDTH + 1;
  region.taskCount = endCppId - beginCppId;
  region.activeWordSpan = region.endActiveWord - region.beginActiveWord;
  struct timeval mtCoarsePlannerPhaseStart = mtCoarsePlannerTimerStart();
  region.staticCost = mtBatchEstimatedCost(tasks, beginCppId, endCppId);
  region.memberNodeCost = mtBatchMemberNodeCost(beginCppId, endCppId);
  mtCoarsePlannerTimerAdd(mtCoarsePlannerTimerStats.regionCostUs, mtCoarsePlannerPhaseStart);
  mtCoarsePlannerPhaseStart = mtCoarsePlannerTimerStart();
  region.expectedActiveCost = region.staticCost;
  region.estimatedUsefulWork = std::max(region.staticCost, region.memberNodeCost);
  region.pureTaskCount = region.taskCount;
  for (int cppId = beginCppId; cppId < endCppId; cppId ++) {
    auto iter = tasks.find(cppId);
    bool isPure = iter != tasks.end() && iter->second.taskKind == "pure_compute";
    bool isSafeSerial = mtIsLevelDispatchMode() && !isPure &&
                        iter != tasks.end() &&
                        !isAlwaysActive(cppId) &&
                        !hasWorker0OnlyReason(iter->second.serialReasons);
    if (!isPure) {
      region.pureTaskCount --;
      region.serialBlockerCount ++;
      if (!isSafeSerial) {
        // Either legacy mt mode (any non-pure rejects) OR worker0-only / always-active
        // under mt-level-dispatch -> region rejected via serial_task blocker.
        mtAddCoarseBlocker(region, "serial_task");
        continue;
      }
      // mt-level-dispatch + safe-serial: admit into the coarse region path; cppId still
      // runs in-worker emit-order (worker per-buffer + post-layer merge).
      region.safeSerialTaskCount ++;
    }
    if (isAlwaysActive(cppId)) mtAddCoarseBlocker(region, "codegen_runtime_limit");
  }
  mtCoarsePlannerTimerAdd(mtCoarsePlannerTimerStats.taskScanUs, mtCoarsePlannerPhaseStart);
  mtCoarsePlannerPhaseStart = mtCoarsePlannerTimerStart();

  for (int from = beginCppId; from < endCppId; from ++) {
    for (int to = beginCppId; to < endCppId; to ++) {
      if (from == to) continue;
      if (mtTaskHasDependencyEdgeTo(from, to)) {
        region.dependencyEdgeCount ++;
        if (from > to) mtAddCoarseBlocker(region, "data_dependency");
      }
      if (mtTaskHasActiveEdgeTo(from, to)) {
        region.activeVisibilityEdgeCount ++;
        if (to > from) region.sameCycleActivationHazardCount ++;
        if (from > to) mtAddCoarseBlocker(region, "active_visibility_edge");
      }
    }
  }
  mtCoarsePlannerTimerAdd(mtCoarsePlannerTimerStats.edgeScanUs, mtCoarsePlannerPhaseStart);
  mtCoarsePlannerPhaseStart = mtCoarsePlannerTimerStart();

  if (beginCppId % ACTIVE_WIDTH != 0 || endCppId % ACTIVE_WIDTH != 0) {
    mtAddCoarseBlocker(region, "codegen_runtime_limit");
  }
  if (region.taskCount < globalConfig.MtActiveFrequencyCostThreshold) {
    mtAddCoarseBlocker(region, "codegen_runtime_limit");
  }
  if (region.activeWordSpan <= 1) {
    mtAddCoarseBlocker(region, "codegen_runtime_limit");
  }
  if (region.pureTaskCount + region.safeSerialTaskCount != region.taskCount) {
    mtAddCoarseBlocker(region, "serial_task");
  }

  region.runtimeEligible = region.blockers.empty();
  mtCoarsePlannerTimerAdd(mtCoarsePlannerTimerStats.blockerFinalizeUs, mtCoarsePlannerPhaseStart);
  mtCoarsePlannerPhaseStart = mtCoarsePlannerTimerStart();
  mtAddCoarseLayers(region);
  if (region.layers.empty()) {
    region.runtimeEligible = false;
  }
  if (region.estimatedMaxParallelWidth < 2) {
    mtAddCoarseBlocker(region, "codegen_runtime_limit");
    region.runtimeEligible = false;
  }
  mtCoarsePlannerTimerAdd(mtCoarsePlannerTimerStats.layerUs, mtCoarsePlannerPhaseStart);
  mtCoarsePlannerPhaseStart = mtCoarsePlannerTimerStart();
  if (region.runtimeEligible) mtAddCoarseMTasks(region, tasks);
  mtCoarsePlannerTimerAdd(mtCoarsePlannerTimerStats.mtaskUs, mtCoarsePlannerPhaseStart);
  mtCoarsePlannerPhaseStart = mtCoarsePlannerTimerStart();
  mtFinalizeCoarseProfitability(region);
  mtCoarsePlannerTimerAdd(mtCoarsePlannerTimerStats.profitabilityUs, mtCoarsePlannerPhaseStart);
  return region;
}

static bool mtCoarseWordCanEnterRegion(const std::map<int, MtTaskInfo>& tasks, int wordBegin) {
  if (!mtActiveWordIsWhole(wordBegin)) return false;
  int wordEnd = std::min(superId, wordBegin + ACTIVE_WIDTH);
  if (wordEnd - wordBegin != ACTIVE_WIDTH) return false;
  for (int cppId = wordBegin; cppId < wordEnd; cppId ++) {
    bool ok = mtIsLevelDispatchMode()
                ? mtTaskCanEnterCoarseDispatch(tasks, cppId)
                : mtTaskCanEnterPureBatch(tasks, cppId);
    if (!ok) return false;
  }
  return true;
}

static bool mtCoarseWordHasSameWordReverseOrderingEdge(int wordBegin) {
  int wordEnd = std::min(superId, wordBegin + ACTIVE_WIDTH);
  for (int from = wordBegin; from < wordEnd; from ++) {
    for (int to = wordBegin; to < from; to ++) {
      if (mtTaskHasOrderingEdgeTo(from, to)) return true;
    }
  }
  return false;
}

static bool mtCoarseWordHasReverseOrderingEdgeIntoRegion(int beginCppId, int wordBegin) {
  int wordEnd = std::min(superId, wordBegin + ACTIVE_WIDTH);
  for (int from = wordBegin; from < wordEnd; from ++) {
    for (int to = beginCppId; to < wordBegin; to ++) {
      if (mtTaskHasOrderingEdgeTo(from, to)) return true;
    }
  }
  return false;
}


static MtCoarseRegionPlan planMtCoarseRegions(const std::map<int, MtTaskInfo>& tasks) {
  MtCoarseRegionPlan plan;
  struct timeval mtCoarsePlannerStart = mtCoarsePlannerTimerStart();
  if (globalConfig.MtReportTimers) mtCoarsePlannerTimerStats = MtCoarsePlannerTimerStats();
  bool levelDispatch = mtIsLevelDispatchMode();
  for (int beginWord = 0; beginWord * ACTIVE_WIDTH < superId; beginWord ++) {
    int beginCppId = beginWord * ACTIVE_WIDTH;
    struct timeval mtCoarsePlannerScanStart = mtCoarsePlannerTimerStart();
    bool beginCanEnter = mtCoarseWordCanEnterRegion(tasks, beginCppId);
    mtCoarsePlannerTimerAdd(mtCoarsePlannerTimerStats.scanBeginEnterUs, mtCoarsePlannerScanStart);
    if (!beginCanEnter) continue;
    if (levelDispatch) {
      mtCoarsePlannerScanStart = mtCoarsePlannerTimerStart();
      bool beginHasSameWordReverseOrderingEdge = mtCoarseWordHasSameWordReverseOrderingEdge(beginCppId);
      mtCoarsePlannerTimerAdd(mtCoarsePlannerTimerStats.scanBeginSameWordUs, mtCoarsePlannerScanStart);
      if (beginHasSameWordReverseOrderingEdge) continue;
    }
    int endWord = beginWord;
    while (endWord * ACTIVE_WIDTH < superId) {
      int wordBegin = endWord * ACTIVE_WIDTH;
      mtCoarsePlannerScanStart = mtCoarsePlannerTimerStart();
      bool endCanEnter = mtCoarseWordCanEnterRegion(tasks, wordBegin);
      mtCoarsePlannerTimerAdd(mtCoarsePlannerTimerStats.scanEndEnterUs, mtCoarsePlannerScanStart);
      if (!endCanEnter) break;
      if (levelDispatch) {
        mtCoarsePlannerScanStart = mtCoarsePlannerTimerStart();
        bool endHasSameWordReverseOrderingEdge = mtCoarseWordHasSameWordReverseOrderingEdge(wordBegin);
        mtCoarsePlannerTimerAdd(mtCoarsePlannerTimerStats.scanEndSameWordUs, mtCoarsePlannerScanStart);
        if (endHasSameWordReverseOrderingEdge) break;
        mtCoarsePlannerScanStart = mtCoarsePlannerTimerStart();
        bool hasReverseOrderingEdgeIntoRegion = mtCoarseWordHasReverseOrderingEdgeIntoRegion(beginCppId, wordBegin);
        mtCoarsePlannerTimerAdd(mtCoarsePlannerTimerStats.scanReverseIntoRegionUs, mtCoarsePlannerScanStart);
        if (hasReverseOrderingEdgeIntoRegion) break;
        if ((endWord + 1) - beginWord > MT_LEVEL_DISPATCH_REGION_SPAN_CAP) break;
      }
      endWord ++;
    }
    int endCppId = endWord * ACTIVE_WIDTH;
    if (endCppId - beginCppId >= ACTIVE_WIDTH) {
      struct timeval mtCoarsePlannerBuildStart = mtCoarsePlannerTimerStart();
      MtCoarseRegion region = mtBuildCoarseRegion(tasks, beginCppId, endCppId);
      mtCoarsePlannerTimerAdd(mtCoarsePlannerTimerStats.regionBuildUs, mtCoarsePlannerBuildStart);
      plan.regions.push_back(region);
      beginWord = std::max(beginWord, endWord - 1);
    }
  }
  if (globalConfig.MtReportTimers) {
    uint64_t totalUs = mtCoarsePlannerElapsedUs(mtCoarsePlannerStart, getTime());
    uint64_t scanOtherUs = totalUs > mtCoarsePlannerTimerStats.regionBuildUs ?
                           totalUs - mtCoarsePlannerTimerStats.regionBuildUs : 0;
    uint64_t scanAttributedUs = mtCoarsePlannerTimerStats.scanBeginEnterUs +
                                mtCoarsePlannerTimerStats.scanBeginSameWordUs +
                                mtCoarsePlannerTimerStats.scanEndEnterUs +
                                mtCoarsePlannerTimerStats.scanEndSameWordUs +
                                mtCoarsePlannerTimerStats.scanReverseIntoRegionUs;
    uint64_t scanUnattributedUs = scanOtherUs > scanAttributedUs ? scanOtherUs - scanAttributedUs : 0;
    logMtReportTimerUs("coarse-planner.total", totalUs);
    logMtReportTimerUs("coarse-planner.scan-other", scanOtherUs);
    logMtReportTimerUs("coarse-planner.scan-begin-enter", mtCoarsePlannerTimerStats.scanBeginEnterUs);
    logMtReportTimerUs("coarse-planner.scan-begin-same-word", mtCoarsePlannerTimerStats.scanBeginSameWordUs);
    logMtReportTimerUs("coarse-planner.scan-end-enter", mtCoarsePlannerTimerStats.scanEndEnterUs);
    logMtReportTimerUs("coarse-planner.scan-end-same-word", mtCoarsePlannerTimerStats.scanEndSameWordUs);
    logMtReportTimerUs("coarse-planner.scan-reverse-into-region", mtCoarsePlannerTimerStats.scanReverseIntoRegionUs);
    logMtReportTimerUs("coarse-planner.scan-unattributed", scanUnattributedUs);
    logMtReportTimerUs("coarse-planner.region-build", mtCoarsePlannerTimerStats.regionBuildUs);
    logMtReportTimerUs("coarse-planner.region-cost", mtCoarsePlannerTimerStats.regionCostUs);
    logMtReportTimerUs("coarse-planner.task-scan", mtCoarsePlannerTimerStats.taskScanUs);
    logMtReportTimerUs("coarse-planner.edge-scan", mtCoarsePlannerTimerStats.edgeScanUs);
    logMtReportTimerUs("coarse-planner.blocker-finalize", mtCoarsePlannerTimerStats.blockerFinalizeUs);
    logMtReportTimerUs("coarse-planner.layers", mtCoarsePlannerTimerStats.layerUs);
    logMtReportTimerUs("coarse-planner.mtasks", mtCoarsePlannerTimerStats.mtaskUs);
    logMtReportTimerUs("coarse-planner.profitability", mtCoarsePlannerTimerStats.profitabilityUs);
  }
  return plan;
}

static int mtHistBucket(int count) {
  if (count <= 1) return 0;
  if (count == 2) return 1;
  if (count <= 4) return 2;
  if (count <= 8) return 3;
  if (count <= 15) return 4;
  return 5;
}

static void mtFinalizeCoarseProfitability(MtCoarseRegion& region) {
  region.estimatedUsefulWork = std::max(region.staticCost, region.memberNodeCost);
  region.mtaskStaticCostMin = 0;
  region.mtaskStaticCostMax = 0;
  region.mtaskStaticCostTotal = 0;
  region.mtaskMemberNodeCostMin = 0;
  region.mtaskMemberNodeCostMax = 0;
  region.mtaskMemberNodeCostTotal = 0;
  for (const MtCoarseMTask& mtask : region.mtasks) {
    if (region.mtaskStaticCostMin == 0 || mtask.staticCost < region.mtaskStaticCostMin) {
      region.mtaskStaticCostMin = mtask.staticCost;
    }
    region.mtaskStaticCostMax = std::max(region.mtaskStaticCostMax, mtask.staticCost);
    region.mtaskStaticCostTotal += mtask.staticCost;
    if (region.mtaskMemberNodeCostMin == 0 || mtask.memberNodeCost < region.mtaskMemberNodeCostMin) {
      region.mtaskMemberNodeCostMin = mtask.memberNodeCost;
    }
    region.mtaskMemberNodeCostMax = std::max(region.mtaskMemberNodeCostMax, mtask.memberNodeCost);
    region.mtaskMemberNodeCostTotal += mtask.memberNodeCost;
  }
}

int mtCoarseStaticRecommendedWorkers(const MtCoarseRegion& region, int configuredWorkers) {
  if (configuredWorkers < 1) return 1;
  if (!region.runtimeEligible) return 1;
  int mtaskCount = static_cast<int>(region.mtasks.size());
  if (mtaskCount <= 1) return 1;

  int workerCap = std::min(configuredWorkers, mtaskCount);
  workerCap = std::min(workerCap, region.estimatedMaxParallelWidth);
  if (region.activeWordSpan > 0) {
    workerCap = std::min(workerCap, std::max(1, region.memberNodeCost / (region.activeWordSpan * 8)));
  }
  if (region.memberNodeCost > 0) {
    workerCap = std::min(workerCap, std::max(1, region.memberNodeCost / 8));
  }
  if (region.staticCost > 0) {
    workerCap = std::min(workerCap, std::max(1, region.staticCost / 4));
  }
  if (workerCap < 1) workerCap = 1;
  if (workerCap > configuredWorkers) workerCap = configuredWorkers;
  return workerCap;
}

bool mtCoarseStaticAdmitsRegion(const MtCoarseRegion& region, int workerCount) {
  if (workerCount <= 1) return false;
  if (region.mtasks.size() <= 1) return false;
  if (region.memberNodeCost < workerCount * 8) return false;
  if (region.staticCost < workerCount * 4) return false;
  if (region.activeWordSpan > 0 && region.memberNodeCost < region.activeWordSpan * workerCount * 8) return false;
  return true;
}

static int mtCoarseProfitableRecommendedWorkers(const MtCoarseRegion& region, int configuredWorkers) {
  if (configuredWorkers < 1) return 1;
  if (!region.runtimeEligible) return 1;
  int mtaskCount = static_cast<int>(region.mtasks.size());
  if (mtaskCount <= 1) return 1;
  int workerCap = std::min(configuredWorkers, mtaskCount);
  workerCap = std::min(workerCap, region.estimatedMaxParallelWidth);
  workerCap = std::min(workerCap, mtaskCount);
  if (workerCap <= 1) return 1;

  int usefulWork = std::max(region.mtaskStaticCostTotal, region.mtaskMemberNodeCostTotal);
  if (usefulWork <= 0) usefulWork = region.estimatedUsefulWork;
  if (usefulWork <= 0) return 1;

  while (workerCap > 1) {
    int copyMergeWords = std::max(0, region.activeWordSpan) * workerCap * 2;
    int usefulPerWorker = usefulWork / workerCap;
    // be much more conservative about which regions are worth
    // parallelizing. Require meaningful per-worker work and a large ratio of
    // useful work to synchronization overhead before suggesting workers.
    if (mtaskCount >= 8 && region.estimatedMaxParallelWidth >= workerCap &&
        usefulWork >= 256 && usefulPerWorker >= 64 && usefulWork >= copyMergeWords * 16)
      break;
    workerCap --;
  }
  return std::max(1, workerCap);
}

int mtCoarseRecommendedWorkersForPolicy(const MtCoarseRegion& region,
                                               int configuredWorkers,
                                               const std::string& workerPolicy) {
  if (workerPolicy == "profitable") return mtCoarseProfitableRecommendedWorkers(region, configuredWorkers);
  return mtCoarseStaticRecommendedWorkers(region, configuredWorkers);
}

bool mtCoarseAdmitsRegionForPolicy(const MtCoarseRegion& region,
                                          int workerCount,
                                          const std::string& workerPolicy) {
  if (workerPolicy != "profitable") return mtCoarseStaticAdmitsRegion(region, workerCount);
  if (workerCount <= 1) return false;
  if (!region.runtimeEligible) return false;
  if (region.mtasks.size() <= 1) return false;
  if (static_cast<int>(region.mtasks.size()) < workerCount) return false;
  int usefulWork = std::max(region.mtaskStaticCostTotal, region.mtaskMemberNodeCostTotal);
  if (usefulWork <= 0) usefulWork = region.estimatedUsefulWork;
  int copyMergeWords = std::max(0, region.activeWordSpan) * workerCount * 2;
  // be much more conservative about which regions are worth
  // parallelizing. The per-region barrier/copy/merge cost dominates for small
  // or low-width regions, so require meaningful per-worker work and a large
  // ratio of useful work to synchronization overhead.
  if (static_cast<int>(region.mtasks.size()) < 8) return false;
  if (region.estimatedMaxParallelWidth < workerCount) return false;
  if (usefulWork < 256) return false;
  if (usefulWork / workerCount < 64) return false;
  return usefulWork >= copyMergeWords * 16;
}

std::string mtJoinIntList(const std::vector<int>& values) {
  std::string result;
  for (size_t i = 0; i < values.size(); i ++) {
    if (i != 0) result += ", ";
    result += std::to_string(values[i]);
  }
  return result;
}

MtCoarseMTaskAssignment mtBuildCoarseMTaskAssignment(const MtCoarseRegion& region,
                                                            int configuredWorkers,
                                                            const std::string& workerPolicy) {
  MtCoarseMTaskAssignment assignment;
  assignment.requestedWorkers = configuredWorkers;
  assignment.effectiveWorkers = mtCoarseRecommendedWorkersForPolicy(region, configuredWorkers, workerPolicy);
  assignment.admitted = mtCoarseAdmitsRegionForPolicy(region, assignment.effectiveWorkers, workerPolicy);
  if (!assignment.admitted) assignment.effectiveWorkers = 1;
  if (assignment.effectiveWorkers < 1) assignment.effectiveWorkers = 1;

  int mtaskCount = static_cast<int>(region.mtasks.size());
  if (mtaskCount <= 0) {
    assignment.effectiveWorkers = 1;
    assignment.admitted = false;
    return assignment;
  }
  if (assignment.effectiveWorkers > mtaskCount) assignment.effectiveWorkers = mtaskCount;

  std::vector<int> contiguousStaticCosts(assignment.effectiveWorkers, 0);
  std::vector<int> contiguousTaskCounts(assignment.effectiveWorkers, 0);
  for (int worker = 0; worker < assignment.effectiveWorkers; worker ++) {
    int begin = (mtaskCount * worker) / assignment.effectiveWorkers;
    int end = (mtaskCount * (worker + 1)) / assignment.effectiveWorkers;
    for (int mtaskIndex = begin; mtaskIndex < end; mtaskIndex ++) {
      contiguousStaticCosts[worker] += region.mtasks[mtaskIndex].staticCost;
      contiguousTaskCounts[worker] += region.mtasks[mtaskIndex].taskCount;
    }
  }

  assignment.workerMTaskIndices.assign(assignment.effectiveWorkers, std::vector<int>());
  assignment.workerStaticCosts.assign(assignment.effectiveWorkers, 0);
  assignment.workerTaskCounts.assign(assignment.effectiveWorkers, 0);

  std::vector<int> order(mtaskCount);
  for (int i = 0; i < mtaskCount; i ++) order[i] = i;
  std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
    if (region.mtasks[lhs].staticCost != region.mtasks[rhs].staticCost) {
      return region.mtasks[lhs].staticCost > region.mtasks[rhs].staticCost;
    }
    if (region.mtasks[lhs].taskCount != region.mtasks[rhs].taskCount) {
      return region.mtasks[lhs].taskCount > region.mtasks[rhs].taskCount;
    }
    return lhs < rhs;
  });

  for (int mtaskIndex : order) {
    int bestWorker = 0;
    for (int worker = 1; worker < assignment.effectiveWorkers; worker ++) {
      if (assignment.workerStaticCosts[worker] < assignment.workerStaticCosts[bestWorker]) {
        bestWorker = worker;
      } else if (assignment.workerStaticCosts[worker] == assignment.workerStaticCosts[bestWorker] &&
                 assignment.workerTaskCounts[worker] < assignment.workerTaskCounts[bestWorker]) {
        bestWorker = worker;
      }
    }
    assignment.workerMTaskIndices[bestWorker].push_back(mtaskIndex);
    assignment.workerStaticCosts[bestWorker] += region.mtasks[mtaskIndex].staticCost;
    assignment.workerTaskCounts[bestWorker] += region.mtasks[mtaskIndex].taskCount;
  }

  for (std::vector<int>& indices : assignment.workerMTaskIndices) std::sort(indices.begin(), indices.end());

  if (!contiguousStaticCosts.empty()) {
    assignment.contiguousBestStaticCost = *std::min_element(contiguousStaticCosts.begin(), contiguousStaticCosts.end());
    assignment.contiguousWorstStaticCost = *std::max_element(contiguousStaticCosts.begin(), contiguousStaticCosts.end());
    assignment.contiguousWorstTaskCount = *std::max_element(contiguousTaskCounts.begin(), contiguousTaskCounts.end());
  }
  if (!assignment.workerStaticCosts.empty()) {
    assignment.balancedBestStaticCost = *std::min_element(assignment.workerStaticCosts.begin(), assignment.workerStaticCosts.end());
    assignment.balancedWorstStaticCost = *std::max_element(assignment.workerStaticCosts.begin(), assignment.workerStaticCosts.end());
    assignment.balancedWorstTaskCount = *std::max_element(assignment.workerTaskCounts.begin(), assignment.workerTaskCounts.end());
  }
  return assignment;
}

MtCoarseProfileFacts mtComputeCoarseProfileFacts(const MtCoarseRegionPlan& coarsePlan) {
  MtCoarseProfileFacts facts;
  for (const MtCoarseRegion& region : coarsePlan.regions) {
    if (!region.runtimeEligible) continue;
    facts.runtimeEligibleRegionCount ++;
    facts.runtimeLayerCount += static_cast<int>(region.layers.size());
    facts.runtimeMTaskCount += static_cast<int>(region.mtasks.size());
    facts.maxRegionLayerCount = std::max(facts.maxRegionLayerCount, static_cast<int>(region.layers.size()));
    facts.regionLayerCountHist[mtHistBucket(static_cast<int>(region.layers.size()))] ++;
    for (const MtCoarseLayer& layer : region.layers) {
      facts.layerSizeHist[mtHistBucket(static_cast<int>(layer.taskCppIds.size()))] ++;
    }
  }
  return facts;
}

static MtPureBatchPlan planMtPureBatchesLegacy(const std::map<int, MtTaskInfo>& tasks) {
  MtPureBatchPlan plan;
  for (int idx = 0; idx < superId; idx ++) {
    int id;
    uint64_t mask;
    std::tie(id, mask) = setIdxMask(idx);
    bool activeWhole = mtActiveWordIsWhole(idx);
    if (!activeWhole || !mtTaskCanEnterPureBatch(tasks, idx)) continue;
    plan.segmentCount ++;

    std::vector<int> batch;
    batch.push_back(idx);
    int batchEnd = idx + 1;
    while (batchEnd < superId && batchEnd / ACTIVE_WIDTH == id &&
           mtTaskCanJoinPureBatch(tasks, batch, batchEnd)) {
      batch.push_back(batchEnd);
      batchEnd ++;
    }
    if (batchEnd - idx > 1) {
      plan.batches.push_back(std::make_pair(idx, batchEnd));
      idx = batchEnd - 1;
    }
  }
  return plan;
}

MtPureBatchPlan planMtPureBatchesActiveFrequency(const std::map<int, MtTaskInfo>& tasks) {
  MtPureBatchPlan plan;
  int threshold = globalConfig.MtActiveFrequencyCostThreshold;
  for (int wordBegin = 0; wordBegin < superId; wordBegin += ACTIVE_WIDTH) {
    if (!mtActiveWordIsWhole(wordBegin)) continue;
    int wordEnd = std::min(superId, wordBegin + ACTIVE_WIDTH);
    int idx = wordBegin;
    while (idx < wordEnd) {
      while (idx < wordEnd && !mtTaskCanEnterPureBatch(tasks, idx)) idx ++;
      if (idx >= wordEnd) break;
      plan.segmentCount ++;

      std::vector<int> batch;
      int batchBegin = idx;
      int batchEnd = idx;
      while (batchEnd < wordEnd && mtTaskCanJoinPureBatch(tasks, batch, batchEnd)) {
        batch.push_back(batchEnd);
        batchEnd ++;
      }
      if (batchEnd - batchBegin > 1 &&
          mtBatchEstimatedCost(tasks, batchBegin, batchEnd) >= threshold) {
        plan.batches.push_back(std::make_pair(batchBegin, batchEnd));
      }
      idx = std::max(batchEnd, batchBegin + 1);
    }
  }
  return plan;
}

MtPureBatchPlan planMtPureBatches(const std::map<int, MtTaskInfo>& tasks) {
  if (globalConfig.MtBatchFormationMode == "active-frequency" ||
      globalConfig.MtBatchFormationMode == "coarse") {
    return planMtPureBatchesActiveFrequency(tasks);
  }
  return planMtPureBatchesLegacy(tasks);
}


void resetMtContextCache() {
  mtContextCache = MtContextCacheState();
  mtDependencyEdgeCache.clear();
  mtActiveEdgeCache.clear();
  mtDenseScheduleCache = MtDenseSchedule();
  mtDenseScheduleCacheValid = false;
}

// The task-info map is a pure function of the frozen graph + config within one
// generation, but the Final phases rebuild it from scratch at every call site
// (7 full rebuilds per T16 champion generation, ~48 s). Memoize unconditionally -
// same provenance as the mtDenseScheduleCache hand-off in d69757c - and hand
// every caller its own copy (return by value), so post-return mutations never
// touch the cached map. resetMtContextCache() invalidates it per generation.
// The plan/trace caches below stay gated on --mt-context-cache because they are
// keyed by their caller's task map, not just global state.
std::map<int, MtTaskInfo> buildMtTaskInfoMapForInvocation() {
  if (!mtContextCache.hasTasks) {
    mtContextCache.tasks = buildMtTaskInfoMap();
    mtContextCache.hasTasks = true;
  }
  return mtContextCache.tasks;
}

// The coarse-region plan is a pure function of the task map within one generation,
// so one plan serves all invocation sites. Memoize unconditionally - same
// provenance as the task-map cache above - and hand every caller its own copy.
// resetMtContextCache() invalidates per generation.
MtCoarseRegionPlan planMtCoarseRegionsForInvocation() {
  if (!mtContextCache.hasCoarseRegionPlan) {
    mtContextCache.coarseRegionPlan = planMtCoarseRegions(buildMtTaskInfoMapForInvocation());
    mtContextCache.hasCoarseRegionPlan = true;
  }
  return mtContextCache.coarseRegionPlan;
}

std::vector<MtStateUpdateTraceInfo> buildMtStateUpdateTraceInfoForInvocation(const std::map<int, MtTaskInfo>& tasks) {
  if (!globalConfig.MtContextCache) return buildMtStateUpdateTraceInfo(tasks);
  if (!mtContextCache.hasStateUpdateTraceInfo) {
    mtContextCache.stateUpdateTraceInfo = buildMtStateUpdateTraceInfo(tasks);
    mtContextCache.hasStateUpdateTraceInfo = true;
  }
  return mtContextCache.stateUpdateTraceInfo;
}

void logMtReportTimer(const char* name, struct timeval start, struct timeval end) {
  if (!globalConfig.MtReportTimers) return;
  int64_t elapsedUs = (static_cast<int64_t>(end.tv_sec - start.tv_sec) * 1000000ll) + static_cast<int64_t>(end.tv_usec - start.tv_usec);
  if (elapsedUs < 0) elapsedUs = 0;
  printf("[mt-report-timer] %s = %llu ms\n", name, static_cast<unsigned long long>(elapsedUs / 1000ll));
}
