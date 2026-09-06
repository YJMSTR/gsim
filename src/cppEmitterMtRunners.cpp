// cppEmitterMtRunners.cpp - split from cppEmitter.cpp (pure code motion):
// genMtTaskRunner (incl. the shared worker-pool runtime text: the pool serves
// sparse/coarse/dense dispatch alike) and genMtCoarseRegionRunner.
#include "cppEmitterImpl.h"

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
