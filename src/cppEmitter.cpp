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
