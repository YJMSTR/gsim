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
  bool denseBreakdownWindowLaBodyCodegen =
      denseBreakdownWindowCodegen && mtUseDenseBreakdownWindowLaBodyCodegen();
  int denseBreakdownWindowWorker0MTaskCount = 0;
  int denseBreakdownWindowAllOwnerMTaskCount = 0;
  int denseBreakdownWindowThreadCount = 8;
  if (denseBreakdownWindowCodegen) {
    const char* denseBreakdownWindowThreadsEnv = std::getenv("GSIM_THREADS");
    if (denseBreakdownWindowThreadsEnv != nullptr && denseBreakdownWindowThreadsEnv[0] != '\0') denseBreakdownWindowThreadCount = std::atoi(denseBreakdownWindowThreadsEnv);
    if (denseBreakdownWindowThreadCount < 1) denseBreakdownWindowThreadCount = 1;
    // Body-only lookahead timing never records worker lanes; allow up to 32
    // there so T32 labels are capturable. Other window modes keep 2..16.
    const int denseBreakdownWindowMaxWorkers =
        mtUseDenseBreakdownWindowLaBodyCodegen() ? 32 : 16;
    Assert(denseBreakdownWindowThreadCount >= 2
               && denseBreakdownWindowThreadCount <= denseBreakdownWindowMaxWorkers,
           "GSIM_MT_DENSE_BREAKDOWN window requires 2..%d workers (got %d)",
           denseBreakdownWindowMaxWorkers, denseBreakdownWindowThreadCount);
  }
  if (denseBreakdownProfileCodegen) {
    Assert(denseExecutorValid,
           "GSIM_MT_DENSE_BREAKDOWN_PROFILE requires a valid dense executor");
  }
  if (mtUseDenseBreakdownWindowLaBodyCodegen()) {
    Assert(denseBreakdownWindowCodegen,
           "GSIM_MT_DENSE_BREAKDOWN_WINDOW_LA_BODY requires GSIM_MT_DENSE_BREAKDOWN_PROFILE=1 with GSIM_MT_DENSE_BREAKDOWN_WINDOW_START and GSIM_MT_DENSE_BREAKDOWN_WINDOW_CYCLES");
    Assert(mtDenseLookaheadWindow() > 0,
           "GSIM_MT_DENSE_BREAKDOWN_WINDOW_LA_BODY requires GSIM_MT_DENSE_LOOKAHEAD >= 1");
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
    fprintf(header, "static constexpr int kDenseBreakdownProfileWorkerCount = %d;\n",
            mtUseDenseBreakdownWindowLaBodyCodegen() ? 32 : 16);
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
      if (denseBreakdownWindowLaBodyCodegen) {
        std::vector<uint8_t> denseBreakdownWindowLaBodyRecordIsLogical(
            (size_t)denseBreakdownWindowAllOwnerMTaskStorageCount, 0);
        for (int mtaskId = 0; mtaskId < denseBreakdownWindowAllOwnerMTaskCount; mtaskId ++) {
          const int denseBreakdownWindowLaBodyRecord =
              denseBreakdownWindowAllOwnerLayout.recordIndexByMTask[(size_t)mtaskId];
          Assert(denseBreakdownWindowLaBodyRecord >= 0
                     && denseBreakdownWindowLaBodyRecord < denseBreakdownWindowAllOwnerMTaskStorageCount,
                 "dense breakdown LA-body record index out of range for MTask %d", mtaskId);
          denseBreakdownWindowLaBodyRecordIsLogical[(size_t)denseBreakdownWindowLaBodyRecord] = 1;
        }
        fprintf(header, "static constexpr uint8_t kDenseBreakdownWindowLaBodyRecordIsLogical[kDenseBreakdownWindowAllOwnerMTaskStorageCount] = {");
        for (int i = 0; i < denseBreakdownWindowAllOwnerMTaskStorageCount; i ++) {
          if (i != 0) fprintf(header, ",");
          fprintf(header, "%u", (unsigned)denseBreakdownWindowLaBodyRecordIsLogical[(size_t)i]);
        }
        fprintf(header, "};\n");
        fprintf(header, "static constexpr int kDenseBreakdownWindowLaBodyMTaskOwner[kDenseBreakdownWindowAllOwnerMTaskCount] = {");
        for (int mtaskId = 0; mtaskId < denseBreakdownWindowAllOwnerMTaskCount; mtaskId ++) {
          if (mtaskId != 0) fprintf(header, ",");
          fprintf(header, "%d", mtDenseSchedule.mtaskThreadAssign[(size_t)mtaskId]);
        }
        fprintf(header, "};\n");
        std::vector<uint64_t> denseBreakdownWindowLaBodyMTaskKeys;
        std::set<uint64_t> denseBreakdownWindowLaBodyUniqueKeys;
        for (int mtaskId = 0; mtaskId < denseBreakdownWindowAllOwnerMTaskCount; ++mtaskId) {
          const uint64_t key = mtDenseMTaskMemberKey(mtDenseSchedule, mtaskId);
          Assert(denseBreakdownWindowLaBodyUniqueKeys.insert(key).second,
                 "dense breakdown LA-body MTask member key collision at MTask %d", mtaskId);
          denseBreakdownWindowLaBodyMTaskKeys.push_back(key);
        }
        fprintf(header, "static constexpr uint64_t kDenseBreakdownWindowLaBodyMTaskMemberKey[kDenseBreakdownWindowAllOwnerMTaskCount] = {");
        for (int mtaskId = 0; mtaskId < denseBreakdownWindowAllOwnerMTaskCount; ++mtaskId) {
          if (mtaskId != 0) fprintf(header, ",");
          fprintf(header, "UINT64_C(%llu)", (unsigned long long)denseBreakdownWindowLaBodyMTaskKeys[(size_t)mtaskId]);
        }
        fprintf(header, "};\n");
      }
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
      if (denseBreakdownWindowLaBodyCodegen) {
        // Exactly-once latch for lookahead body-only samples: 0 untouched,
        // 1 recorded exactly once this window slot, anything else is a
        // duplicate dispatch and aborts. Zeroed in initMtDenseBreakdownProfile
        // before the window is enabled.
        fprintf(header, "uint8_t mtDenseBreakdownWindowLaBodySeen[kDenseBreakdownWindowMaxCycles][kDenseBreakdownWindowAllOwnerMTaskStorageCount];\n");
      }
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
      if (denseBreakdownWindowLaBodyCodegen) {
        emitBodyLock(1, "gAssert(mtDenseBreakdownWindowModeEnv != nullptr && strcmp(mtDenseBreakdownWindowModeEnv, \"allownerbody\") == 0, \"GSIM_MT_DENSE_BREAKDOWN_WINDOW_LA_BODY requires GSIM_MT_DENSE_BREAKDOWN_WINDOW_MODE=allownerbody; other window modes are incompatible with lookahead\");\n");
        emitBodyLock(1, "if (mtDenseBreakdownWindowModeEnv == nullptr || strcmp(mtDenseBreakdownWindowModeEnv, \"allownerbody\") != 0) abort();\n");
        emitBodyLock(1, "mtDenseBreakdownWindowWorker0BodyMode = false;\n");
        emitBodyLock(1, "mtDenseBreakdownWindowAllOwnerBodyMode = true;\n");
        emitBodyLock(1, "mtDenseBreakdownWindowFinishOnlyMode = false;\n");
      } else {
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
      }
      emitBodyLock(1, "static const char mtDenseBreakdownWindowSuffix[] = \".window.json\";\n");
      emitBodyLock(1, "gAssert(mtDenseBreakdownProfileOutPathLen + sizeof(mtDenseBreakdownWindowSuffix) <= sizeof(mtDenseBreakdownWindowOutPath), \"GSIM_MT_DENSE_BREAKDOWN_OUT path is too long for window sibling\");\n");
      emitBodyLock(1, "if (mtDenseBreakdownProfileOutPathLen + sizeof(mtDenseBreakdownWindowSuffix) > sizeof(mtDenseBreakdownWindowOutPath)) abort();\n");
      emitBodyLock(1, "memcpy(mtDenseBreakdownWindowOutPath, mtDenseBreakdownProfileOutPath, mtDenseBreakdownProfileOutPathLen);\n");
      emitBodyLock(1, "memcpy(mtDenseBreakdownWindowOutPath + mtDenseBreakdownProfileOutPathLen, mtDenseBreakdownWindowSuffix, sizeof(mtDenseBreakdownWindowSuffix));\n");
      if (denseBreakdownWindowLaBodyCodegen) {
        emitBodyLock(1, "for (int c = 0; c < kDenseBreakdownWindowMaxCycles; c ++) { for (int i = 0; i < kDenseBreakdownWindowAllOwnerMTaskStorageCount; i ++) mtDenseBreakdownWindowLaBodySeen[c][i] = 0; }\n");
      }
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
      if (denseBreakdownWindowLaBodyCodegen) {
        // Lookahead body-only dump. Reads ONLY the body records written inside
        // stepDenseMTaskN(): no worker spans, no waits, no release endpoints
        // (the LA tail's early return skips the worker epilogue, and the
        // release-end recorder is dead text under lookahead codegen, so those
        // cycle recorded and every task sampled exactly once per cycle.
        emitBodyLock(2, "for (int c = 0; c < kDenseBreakdownWindowMaxCycles; c ++) {\n");
        emitBodyLock(3, "for (int i = 0; i < kDenseBreakdownWindowAllOwnerMTaskStorageCount; i ++) {\n");
        emitBodyLock(4, "const uint8_t mtDenseBreakdownWindowLaBodySeenCount = mtDenseBreakdownWindowLaBodySeen[c][i];\n");
        emitBodyLock(4, "if (c < (int)mtDenseBreakdownWindowRecordedCycles) {\n");
        emitBodyLock(5, "if (kDenseBreakdownWindowLaBodyRecordIsLogical[i] == 0) {\n");
        emitBodyLock(6, "if (unlikely(mtDenseBreakdownWindowLaBodySeenCount != 0)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] lookahead body sample in padding record\\n\"); abort(); }\n");
        emitBodyLock(5, "} else if (unlikely(mtDenseBreakdownWindowLaBodySeenCount != 1)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] lookahead body sample count is not exactly one\\n\"); abort(); }\n");
        emitBodyLock(4, "} else if (unlikely(mtDenseBreakdownWindowLaBodySeenCount != 0)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] lookahead body sample outside the recorded window\\n\"); abort(); }\n");
        emitBodyLock(3, "}\n");
        emitBodyLock(2, "}\n");
        emitBodyLock(2, "for (uint64_t c = 0; c < mtDenseBreakdownWindowRecordedCycles; c ++) {\n");
        emitBodyLock(3, "for (int mtaskId = 0; mtaskId < kDenseBreakdownWindowAllOwnerMTaskCount; mtaskId ++) {\n");
        emitBodyLock(4, "const MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowLaBodyRecord = mtDenseBreakdownWindowAllOwnerMTasks[c][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[mtaskId]];\n");
        emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowLaBodyRecord.mtaskId != (uint32_t)mtaskId || mtDenseBreakdownWindowLaBodyRecord.ownerThreadId != (uint16_t)kDenseBreakdownWindowLaBodyMTaskOwner[mtaskId] || mtDenseBreakdownWindowLaBodyRecord.ownerThreadId >= (uint16_t)mtDenseBreakdownProfileWorkerCount || mtDenseBreakdownWindowLaBodyRecord.bodyEndOffsetNs < mtDenseBreakdownWindowLaBodyRecord.bodyStartOffsetNs)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fclose(mtDenseBreakdownWindowFile); fprintf(stderr, \"[mt-dense-breakdown] corrupt lookahead body MTask record\\n\"); abort(); }\n");
        emitBodyLock(3, "}\n");
        emitBodyLock(2, "}\n");
        emitBodyLock(2, "if (fprintf(mtDenseBreakdownWindowFile, \"{\\\"magic\\\":\\\"GSIM_MT_DENSE_LOOKAHEAD_BODY_WINDOW\\\",\\\"version\\\":1,\\\"window_start\\\":%%llu,\\\"window_cycles\\\":%%llu,\\\"accepted_cycles\\\":%%llu,\\\"threadCount\\\":%%d,\\\"mode\\\":\\\"%%s\\\",\\\"allowner_mtask_count\\\":%%d,\\\"overflow\\\":%%s,\\\"complete\\\":%%s,\\\"cycles\\\":[\", (unsigned long long)mtDenseBreakdownWindowStart, (unsigned long long)mtDenseBreakdownWindowCycles, (unsigned long long)mtDenseBreakdownWindowRecordedCycles, mtDenseBreakdownProfileWorkerCount, mtDenseBreakdownWindowMode, kDenseBreakdownWindowAllOwnerMTaskCount, mtDenseBreakdownWindowOverflowed ? \"true\" : \"false\", mtDenseBreakdownWindowComplete ? \"true\" : \"false\") < 0) mtDenseBreakdownWindowWriteError = 1;\n");
        emitBodyLock(2, "for (uint64_t c = 0; c < mtDenseBreakdownWindowRecordedCycles; c ++) {\n");
        emitBodyLock(3, "if (c != 0 && fputc(',', mtDenseBreakdownWindowFile) == EOF) mtDenseBreakdownWindowWriteError = 1;\n");
        emitBodyLock(3, "if (fprintf(mtDenseBreakdownWindowFile, \"{\\\"cycle\\\":%%llu,\\\"mtasks\\\":[\", (unsigned long long)mtDenseBreakdownWindowCycleNumbers[c]) < 0) mtDenseBreakdownWindowWriteError = 1;\n");
        emitBodyLock(3, "for (int mtaskId = 0; mtaskId < kDenseBreakdownWindowAllOwnerMTaskCount; mtaskId ++) {\n");
        emitBodyLock(4, "if (mtaskId != 0 && fputc(',', mtDenseBreakdownWindowFile) == EOF) mtDenseBreakdownWindowWriteError = 1;\n");
        emitBodyLock(4, "const MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowLaBodyRecord = mtDenseBreakdownWindowAllOwnerMTasks[c][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[mtaskId]];\n");
        emitBodyLock(4, "if (fprintf(mtDenseBreakdownWindowFile, \"{\\\"mtaskId\\\":%%u,\\\"memberKey\\\":\\\"0x%%016llx\\\",\\\"ownerThreadId\\\":%%u,\\\"bodyStartOffsetNs\\\":%%llu,\\\"bodyEndOffsetNs\\\":%%llu,\\\"bodyNs\\\":%%llu}\", mtDenseBreakdownWindowLaBodyRecord.mtaskId, (unsigned long long)kDenseBreakdownWindowLaBodyMTaskMemberKey[mtaskId], (unsigned)mtDenseBreakdownWindowLaBodyRecord.ownerThreadId, (unsigned long long)mtDenseBreakdownWindowLaBodyRecord.bodyStartOffsetNs, (unsigned long long)mtDenseBreakdownWindowLaBodyRecord.bodyEndOffsetNs, (unsigned long long)mtDenseBreakdownWindowLaBodyRecord.bodyNs) < 0) mtDenseBreakdownWindowWriteError = 1;\n");
        emitBodyLock(3, "}\n");
        emitBodyLock(3, "if (fprintf(mtDenseBreakdownWindowFile, \"]}\") < 0) mtDenseBreakdownWindowWriteError = 1;\n");
        emitBodyLock(2, "}\n");
        emitBodyLock(2, "if (fprintf(mtDenseBreakdownWindowFile, \"]}\\n\") < 0) mtDenseBreakdownWindowWriteError = 1;\n");
      } else {
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
      }
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
