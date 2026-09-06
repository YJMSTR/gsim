// cppEmitterMtReport.cpp - split from cppEmitter.cpp (pure code motion):
// the dump reports: dumpMtScheduleJson, dumpMtDenseScheduleJson (also the WRITE
// side of mtDenseScheduleCache: the built schedule is handed to the driver via
// move, unchanged), dumpMtCoarseRegionReport, dumpMtReadyBatchReport.
#include "cppEmitterImpl.h"

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

