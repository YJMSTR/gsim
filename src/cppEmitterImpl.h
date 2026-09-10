/*
  cppEmitterImpl.h - internal shared header for the cppEmitter TU family.
  Created by the cppEmitter.cpp split (pure code motion); every src/cppEmitter*.cpp
  TU includes this. NOT part of the public graph API - do not include elsewhere.
*/
#ifndef CPPEMITTER_IMPL_H
#define CPPEMITTER_IMPL_H

#include "common.h"
#include "util.h"

#include <atomic>
#include <thread>
#include <cstddef>
#include <cstdio>
#include <cinttypes>
#include <cctype>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <deque>
#include <functional>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <queue>
#include <stack>
#include <string>
#include <tuple>
#include <sstream>
#include <utility>
#include <unordered_map>
#include <vector>

#define ACTIVE_WIDTH 8
#define RESET_PER_FUNC 400
#define MT_PURE_BATCH_SHARD_SIZE 256
// hard cap on coarse region active-word span under mt-level-dispatch.
// Pre-implementation G-1A2.0 simulation showed raw max span 649; cap=192 keeps the
// dense merge upper bound bounded while only splitting 6 oversized regions.
#define MT_LEVEL_DISPATCH_REGION_SPAN_CAP 192

#define ENABLE_ACTIVATOR false

#ifdef DIFFTEST_PER_SIG
extern FILE* sigFile;  // defined in cppEmitter.cpp
#endif

extern int maxConcatNum;
bool nameExist(std::string str);
std::pair<int, uint64_t> setIdxMask(int cppId);

#define RESET_NAME(node) (node->name + "$RESET")
#define emitFuncDecl(indent, ...) __emitSrc(indent, true, true, NULL, __VA_ARGS__)
#define emitBodyLock(indent, ...) __emitSrc(indent, false, false, NULL, __VA_ARGS__)

#define ActiveType std::tuple<uint64_t, std::string, int>
#define ACTIVE_MASK(active) std::get<0>(active)
#define ACTIVE_COMMENT(active) std::get<1>(active)
#define ACTIVE_UNIQUE(active) std::get<2>(active)

bool emitPhaseTimingEnabled();  // def: cppEmitterUtil.cpp (used by EmitPhaseTimer dtor below)

// Final-phase breakdown instrumentation (GSIM_EMIT_PHASE_TIMING=1): wall time
// per emission region, printed to stderr when the region scope closes. Pure
// measurement - never changes emitted bytes.
struct EmitPhaseTimer {
  const char* phaseName;
  std::chrono::steady_clock::time_point begin;
  explicit EmitPhaseTimer(const char* n) : phaseName(n), begin(std::chrono::steady_clock::now()) {}
  ~EmitPhaseTimer() {
    if (emitPhaseTimingEnabled()) {
      long ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();
      fprintf(stderr, "[emit-phase] %s = %ld ms\n", phaseName, ms);
    }
  }
};

// Accumulating variant for sub-phases invoked many times inside one region
// (e.g. per-pass work inside a merge loop). Aggregates wall ns across calls;
// the owning region prints totals via emitPhaseAccumReport(). Like
// EmitPhaseTimer this is pure measurement - it never changes emitted bytes,
// and costs two predictable branches when GSIM_EMIT_PHASE_TIMING is unset.
struct EmitPhaseAccum {
  const char* phaseName = "";
  uint64_t ns = 0;
  uint64_t calls = 0;
};
class EmitPhaseAccumScope {
 public:
  EmitPhaseAccumScope(EmitPhaseAccum& accum, bool enabled)
      : accum_(enabled ? &accum : nullptr),
        begin_(enabled ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point()) {}
  ~EmitPhaseAccumScope() {
    if (accum_) {
      accum_->ns += std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - begin_).count();
      accum_->calls ++;
    }
  }
 private:
  EmitPhaseAccum* accum_;
  std::chrono::steady_clock::time_point begin_;
};

// ---- shared emitter struct definitions (moved verbatim from cppEmitter.cpp; 
// single global type identity across all emitter TUs) ----
struct MtBoundaryInfo {
  std::map<std::string, int> nodeKinds;
  std::set<std::string> clockNames;
  std::set<std::string> stateTargetNames;
  std::set<std::string> rhsReadStateTargetNames;
  bool hasStateUpdate = false;
  bool hasAmbiguousStateTarget = false;
  bool hasRhsNextStateObjectRead = false;
  bool hasUnexpandedRhsDependency = false;
  bool hasMemoryWrite = false;
  bool hasMemoryRead = false;
  bool hasReset = false;
  bool hasAsyncReset = false;
  bool hasActivateAllPath = false;
  bool hasExternal = false;
  bool hasSpecial = false;
  bool hasUnknownNode = false;
  bool hasUnknownOp = false;
  bool hasArrayOrDynamicIndex = false;
  int stateSourceCommitCount = 0;
  int stateNextUpdateCount = 0;
  int stateResetUpdateCount = 0;
};

struct MtTaskInfo {
  MtBoundaryInfo boundary;
  std::string taskKind;
  std::vector<std::string> serialReasons;
  bool isSource = false;
  bool isSink = false;
  int candidateCost = 0;
  bool hasCandidateCost = false;
};

struct MtStateUpdateTraceInfo {
  bool hasStateUpdate = false;
  bool localSafeCandidate = false;
  bool runtimeSafeCandidate = false;
  std::string targetWriterConflictKind = "none";
  std::vector<std::string> runtimeBlockReasons;
};

struct MtPureBatchPlan {
  std::vector<std::pair<int, int>> batches;
  int segmentCount = 0;
};

struct MtCoarseLayer {
  std::vector<int> taskCppIds;
};

struct MtCoarseMTask {
  std::vector<std::vector<int>> layerTaskCppIds;
  int taskCount = 0;
  int staticCost = 0;
  int memberNodeCost = 0;
  int orderingEdgeCount = 0;
};

struct MtCoarseRegion {
  int beginCppId = -1;
  int endCppId = -1;
  int beginActiveWord = -1;
  int endActiveWord = -1;
  int taskCount = 0;
  int activeWordSpan = 0;
  int staticCost = 0;
  int memberNodeCost = 0;
  int expectedActiveCost = 0;
  int estimatedUsefulWork = 0;
  int pureTaskCount = 0;
  int safeSerialTaskCount = 0;   // serial cppIds admitted under mt-level-dispatch
  int serialBlockerCount = 0;
  int dependencyEdgeCount = 0;
  int activeVisibilityEdgeCount = 0;
  int sameCycleActivationHazardCount = 0;
  int replicationCandidateCount = 0;
  int estimatedLayerCount = 0;
  int estimatedMaxParallelWidth = 0;
  int mtaskStaticCostMin = 0;
  int mtaskStaticCostMax = 0;
  int mtaskStaticCostTotal = 0;
  int mtaskMemberNodeCostMin = 0;
  int mtaskMemberNodeCostMax = 0;
  int mtaskMemberNodeCostTotal = 0;
  bool runtimeEligible = false;
  std::vector<std::string> blockers;
  std::vector<MtCoarseLayer> layers;
  std::vector<MtCoarseMTask> mtasks;
};

struct MtCoarseRegionPlan {
  std::vector<MtCoarseRegion> regions;
};

struct MtCoarseProfileFacts {
  int runtimeEligibleRegionCount = 0;
  int runtimeLayerCount = 0;
  int maxRegionLayerCount = 0;
  int runtimeMTaskCount = 0;
  int layerSizeHist[6] = {0, 0, 0, 0, 0, 0};
  int regionLayerCountHist[6] = {0, 0, 0, 0, 0, 0};
};

struct MtDenseEdge {
  int fromCppId = -1;
  int toCppId = -1;
  std::string kind;
};

struct MtDenseScc {
  std::vector<int> cppIds;
  std::vector<int> predSccs;
  std::vector<int> succSccs;
  int staticCost = 0;
  int memberNodeCost = 0;
  int worker0OnlyTaskCount = 0;
  int alwaysActiveTaskCount = 0;
  int internalEdgeCount = 0;
  int internalDependencyEdgeCount = 0;
  int internalActiveEdgeCount = 0;
  int internalNeedActivateEdgeCount = 0;
  int incomingEdgeCount = 0;
  int outgoingEdgeCount = 0;
  bool workerZeroOnly = false;
  bool isAlwaysActive = false;
};
struct MtDenseMTask {
  std::vector<int> sccIds;
  std::vector<int> predMTasks;
  std::vector<int> succMTasks;
  int staticCost = 0;
  int taskCount = 0;
  bool workerZeroOnly = false;
  // real dense scheduling cost (sum of member-node work of contained SCCs).
  // staticCost is ~1 for serial tasks, a weak PackThreads signal; schedCost drives
  // the CP-contraction path's priority/end-time when > 0, else falls back to staticCost.
  int schedCost = 0;
};

struct MtDenseLayer {
  std::vector<int> sccIds;
  bool workerZeroOnly = false;
  int taskCount = 0;
  int staticCost = 0;
};

struct MtDenseSchedule {
  bool codegenEnabled = false;
  bool valid = false;
  std::string fallbackReason = "not_built";
  int taskCount = 0;
  int edgeCount = 0;
  int dependencyEdgeCount = 0;
  int activeEdgeCount = 0;
  int needActivateEdgeCount = 0;
  int cycleSccCount = 0;
  int maxSccSize = 0;
  std::vector<std::vector<int>> succCppIds;
  std::vector<std::vector<int>> predCppIds;
  std::vector<MtDenseEdge> edges;
  std::vector<MtDenseScc> sccs;
  std::vector<MtDenseLayer> layers;
  std::vector<MtDenseMTask> mtasks;
  std::vector<int> mtaskThreadAssign;
  std::vector<int> topoSccOrder;
  std::vector<int> worker0OnlyCppIds;
  std::vector<int> alwaysActiveCppIds;
};
// Immutable, renumber-invariant identity of an MTask: order-independent 64-bit
// hash over its sorted member cppIds. Shared by the lookahead body-timing key
// table (cppEmitter.cpp) and the measured-cost import (cppEmitterDenseSchedule.cpp).
inline uint64_t mtDenseMTaskMemberKey(const MtDenseSchedule& schedule, int mtaskId) {
  Assert(mtaskId >= 0 && mtaskId < static_cast<int>(schedule.mtasks.size()),
         "dense MTask member key id %d out of range", mtaskId);
  std::vector<int> memberCppIds;
  for (int sccId : schedule.mtasks[(size_t)mtaskId].sccIds) {
    Assert(sccId >= 0 && sccId < static_cast<int>(schedule.sccs.size()),
           "dense MTask member key has invalid SCC %d", sccId);
    const std::vector<int>& cppIds = schedule.sccs[(size_t)sccId].cppIds;
    memberCppIds.insert(memberCppIds.end(), cppIds.begin(), cppIds.end());
  }
  std::sort(memberCppIds.begin(), memberCppIds.end());
  uint64_t key = UINT64_C(1469598103934665603);
  for (int cppId : memberCppIds) {
    key ^= static_cast<uint64_t>(static_cast<uint32_t>(cppId)) + UINT64_C(0x9e3779b97f4a7c15);
    key *= UINT64_C(1099511628211);
  }
  return key;
}

struct MtCoarseMTaskAssignment {
  int requestedWorkers = 1;
  int effectiveWorkers = 1;
  bool admitted = false;
  int contiguousWorstStaticCost = 0;
  int contiguousBestStaticCost = 0;
  int contiguousWorstTaskCount = 0;
  int balancedWorstStaticCost = 0;
  int balancedBestStaticCost = 0;
  int balancedWorstTaskCount = 0;
  std::vector<std::vector<int>> workerMTaskIndices;
  std::vector<int> workerStaticCosts;
  std::vector<int> workerTaskCounts;
};

// (the activity4 rab one-hot divergence: deqPtrOH stuck at 0). No truncation budget.
// Speed (activity5 generation took 52min vs 27min baseline): memoize per-Node read sets —
// shared subexpression nodes are expanded once globally, cppIds merge cached sets.
struct MtActivityReads { std::set<std::string> src; std::set<std::string> dst; };

struct MtDenseOwnerReadyTokenProvenance {
  int readySlot = -1;
  int producerMTask = -1;
  int producerOwner = -1;
  int consumerMTask = -1;
  int consumerOwner = -1;
};

struct MtDenseOwnerReadyLayout {
  int edgeCount = 0;
  int tokenCount = 0;
  int physicalSlotCount = 0;
  int pairBankCount = 0;
  std::vector<MtDenseOwnerReadyTokenProvenance> tokenProvenanceByLogicalToken;
  std::vector<std::vector<int>> sourceMTasksByLogicalToken;
  std::vector<int> logicalTokenByPhysicalSlot;
  std::vector<std::vector<int>> waitSlotsByMTask;
  std::vector<std::vector<int>> storeSlotsByMTask;
};

struct MtDenseBreakdownWindowWaitLayout {
  int totalWaitRecords = 0;
  std::vector<int> laneOffsets;
};

// The all-owner record is 40 bytes.  Round each owner lane to eight records
// (320 bytes), preserving a 64-byte-aligned base and lane boundary without
// spending an entire cache line on every record.
struct MtDenseBreakdownWindowAllOwnerLayout {
  int recordCount = 0;
  std::vector<int> laneOffsets;
  std::vector<int> recordIndexByMTask;
};

struct MtStateTargetWriterInfo {
  int writerCount = 0;
  std::set<int> writerCppIds;
  int multiTargetWriterCount = 0;
};

struct MtStateTargetWriterUniverse {
  std::map<std::string, MtStateTargetWriterInfo> targetWriters;
  bool hasIncompleteWriterUniverse = false;
  std::set<int> incompleteWriterCppIds;
};

struct MtContextCacheState {
  bool hasTasks = false;
  std::map<int, MtTaskInfo> tasks;
  bool hasCoarseRegionPlan = false;
  bool hasStateUpdateTraceInfo = false;
  MtCoarseRegionPlan coarseRegionPlan;
  std::vector<MtStateUpdateTraceInfo> stateUpdateTraceInfo;
};

// ---- cross-TU emitter state (C++17 inline: exactly one instance across the
// emitter TU family; sole writer + write phase annotated). NEVER re-declare any of
// these as file-static in a TU - a static copy silently forks the state. ----
// Report-only: classify emitted old-value snapshots by whether their change detection
// feeds any activation consumer (nextActiveId). Answers the per-node DCE question for
// the largest bookkeeping class without touching emitted semantics.
// Atomic so parallel emission units can bump them without a lock; the report
// only prints the sum, which is order-independent.
inline std::atomic<uint64_t> mtOldSnapWithConsumers{0}, mtOldSnapNoConsumers{0};
inline int superId = 0;
inline int activeFlagNum = 0;
inline std::set<Node*> definedNode;
inline std::map<int, SuperNode*> cppId2Super;
inline std::set<int> alwaysActive;
// Parallel Final emission: the two context flags below are per-thread. Worker
// threads that render independent emission units each get their own copy;
// emitUnitsParallel() snapshots the main-thread values into every worker.
// Suppress sparse activation-event calls while emitting the optional dense executor body.
inline thread_local bool mtActivationEventTraceSuppressed = false;
// Codegen-time semantic source: admitted sparse mtTask cppId, or -1 for reset/external paths.
inline thread_local int mtActivationEventTraceSourceCppId = -1;
inline std::map<Node*, std::pair<int, int>> super2ResetId;  // uint & async reset
inline std::map<Node*, std::pair<int, int>> super2DenseResetId;  // dense uint & async reset
inline int resetFuncNum = 0;
inline std::unordered_map<uint64_t, bool> mtDependencyEdgeCache;
inline std::unordered_map<uint64_t, bool> mtActiveEdgeCache;
inline MtContextCacheState mtContextCache;
// One-shot dense schedule hand-off: dumpMtDenseScheduleJson builds the schedule before Final
// emission starts; with dense executor codegen Final.scheduleBuild would rebuild the exact same
// schedule (same task map, same global graph state, nothing mutates in between), costing a
// second full vcontract pass. resetMtContextCache() invalidates it per generation.
inline MtDenseSchedule mtDenseScheduleCache;
inline bool mtDenseScheduleCacheValid = false;

// ---- function declarations (added one module per commit; defs live in the
// corresponding cppEmitter<Module>.cpp TU; default arguments live HERE only) ----

// cppEmitterUtil.cpp - knob readers, shared helpers, name interning, bitmask helpers
bool mtOldValueHistogramEnabled();
bool emitPhaseTimingEnabled();
void emitPhaseAccumReport(const EmitPhaseAccum& accum);
bool isAlwaysActive(int cppId);
bool hasCppId(const std::set<SuperNode*>& supers, int cppId);
const char* nodeTypeName(NodeType type);
bool isKnownNodeType(NodeType type);
const char* superTypeName(SuperType type);
std::string jsonEscape(const std::string& str);
void dumpJsonIntArray(FILE* fp, const std::set<int>& values);
void dumpJsonIntArray(FILE* fp, const std::vector<int>& values);
void dumpJsonStringArray(FILE* fp, const std::set<std::string>& values);
void dumpJsonStringArray(FILE* fp, const std::vector<std::string>& values);
bool mtUseDenseMemberMetadata();
bool mtUseDenseUnpinSpecial();
bool mtUseDenseElideObservability();
const std::set<Node*>& mtDenseObservabilityDroppableSet();
bool mtDenseObservabilitySpansBalanced(const std::vector<InstInfo>& insts);
bool mtUseDenseForwardActivationOnly();
void dumpMtDenseMemberMetadataForTask(FILE* fp, SuperNode* super);
void dumpMtDenseActivationOrigins(FILE* fp);
bool mtIsLevelDispatchMode();
bool mtUseDirectInlineFallback();
bool mtUseDirectInlineSerialFallback();
bool mtUseDirectInlineWorker0Fallback();
bool mtUseProfileOffDirectSerialFallback();
bool mtUseProfileOffActiveWordCount();
bool mtUseInlineSmallPureBatches();
bool mtUseInlineSmallPureBatchBodies();
bool mtUseInlineSmallPureBatchMaskGuard();
bool mtUseStepActiveWordGuard();
bool mtUseSplitMixedStepGuards();
bool mtUseCycleBatchReport();
bool mtUseReadyBatchReport();
bool mtUseEnvelopeLocalEvalDiagnostics();
bool mtUseDynamicStateTraceCodegen();
bool mtUseActivationEventTraceCodegen();
bool mtUseDenseExecutorCodegen();
bool mtUseDenseOnlyCodegen();
bool mtUseDenseOnlyCodegenLevel2();
bool mtUseDenseXThreadDepsOnly();
bool mtUseDenseTransitiveReduceEdges();
bool mtUseDenseStaticEmptyElide();
bool mtUseDenseOwnerBankCountersDiag();
bool mtUseDenseOwnerReadyFlags();
bool mtUseWorkerPoolFlagJoinCodegen();
bool mtUseOwnerCpuMapCodegen();
bool mtUseDenseBreakdownProfileCodegen();
bool mtUseDenseBreakdownWindowCodegen();
bool mtUseDenseBreakdownWindowLaBodyCodegen();
bool mtUseDenseWorkerMajorText();
bool mtUseEmitTaskLocals();
bool mtUseEmitPscdBits();
bool mtUsePscdVerify();
bool mtUseDenseEdgeTiming();
bool mtUseDensePushReadyShadow();
bool mtUseDensePushReady();
bool mtUseDensePushReadyDebug();
bool mtUseDensePushReadyDirectTable();
int mtDenseSpinYieldEvery();
int mtDenseLookaheadWindow();
bool mtDenseDutyCodegen();
void mtActivityCollectFromTree(ENode* root, MtActivityReads& out);
bool mtUseDenseHybridEligibilityDiag();
bool mtUseDenseSplitWorker0MTasks();
bool mtUseStaticCoarseInlineBound();
bool mtUseShortNames();
const std::string* mtShortNameOrigOf(Node* node);
std::pair<int, int> cppId2flagIdx(int cppId);
std::pair<int, uint64_t>clearIdxMask(int cppId);
ActiveType activeSet2bitMap(std::set<int>& activeId, std::map<uint64_t, ActiveType>& bitMapInfo, int curId);
std::string updateActiveStr(int idx, uint64_t mask, const std::string& activeBufferName = "");
std::string updateActiveStr(int idx, uint64_t mask, std::string& cond, int uniqueId, const std::string& activeBufferName = "");
void includeLib(FILE* fp, std::string lib, bool isStd);
void newLine(FILE* fp);
int emitParallelThreadCount();


// cppEmitterMtAnalysis.cpp - task classification, boundary collection,
// state-update classification, cross-task edge memos
void addCppIdsIfExecutable(std::set<int>& ids, const std::set<SuperNode*>& supers);
bool nodeHasStateUpdate(Node* node);
bool stateTargetNameForNode(Node* node, std::string& targetName);
MtBoundaryInfo collectMtBoundaryInfo(SuperNode* super, int& candidateCost);
std::map<int, MtTaskInfo> buildMtTaskInfoMap();
bool hasWorker0OnlyReason(const std::vector<std::string>& reasons);
bool hasWorker0OnlyReasonDense(const std::vector<std::string>& reasons);
bool hasOnlyA44DirectFallbackReasons(const std::vector<std::string>& reasons);
bool hasOnlyA73Worker0SafeReasons(const std::vector<std::string>& reasons);
bool mtTaskCanEnterCoarseDispatch(const std::map<int, MtTaskInfo>& tasks, int cppId);
bool mtTaskCanEnterPureBatch(const std::map<int, MtTaskInfo>& tasks, int cppId);
bool mtTaskHasSameActiveWordHazard(const std::map<int, MtTaskInfo>& tasks, int cppId);
bool mtTaskCanJoinPureBatch(const std::map<int, MtTaskInfo>& tasks,
                                   const std::vector<int>& batch,
                                   int cppId);
int mtTaskEstimatedCost(const std::map<int, MtTaskInfo>& tasks, int cppId);
int mtBatchEstimatedCost(const std::map<int, MtTaskInfo>& tasks, int beginCppId, int endCppId);
int mtBatchMemberNodeCost(int beginCppId, int endCppId);
bool mtActiveWordIsWhole(int beginCppId);
int mtPureBatchShardCount();
void mtAddCoarseBlocker(MtCoarseRegion& region, const std::string& blocker);
bool mtTaskHasActiveEdgeTo(int fromCppId, int toCppId);
bool mtTaskHasDependencyEdgeTo(int fromCppId, int toCppId);
bool mtTaskHasOrderingEdgeTo(int fromCppId, int toCppId);
bool mtStateUpdateHasMemoryOrDynamicArray(const MtBoundaryInfo& boundary);
bool mtStateUpdateHasExternalOrSpecial(const MtBoundaryInfo& boundary);
bool mtStateUpdateHasSameCycleTargetRead(const MtBoundaryInfo& boundary,
                                                const std::set<std::string>& allStateTargetNames);
std::string mtStateUpdateRhsTimingClass(const MtBoundaryInfo& boundary);
std::string mtStateUpdateRhsTimingEvidence(const MtBoundaryInfo& boundary,
                                                  const std::string& rhsTimingClass);
bool mtStateUpdateActivationCanUseDelta(const MtBoundaryInfo& boundary);
std::vector<std::string> mtStateUpdateBlockReasons(const MtBoundaryInfo& boundary,
                                                          SuperNode* super,
                                                          const std::string& rhsTimingClass,
                                                          bool rhsReadsSameCycleTarget);
std::string mtStateUpdateCandidateKind(const MtBoundaryInfo& boundary,
                                              const std::vector<std::string>& blockReasons);
MtStateTargetWriterUniverse
collectMtStateTargetWriters(const std::map<int, MtTaskInfo>& mtTasks);
MtStateTargetWriterInfo mtStateUpdateWriterInfo(
    const MtBoundaryInfo& boundary,
    const std::map<std::string, MtStateTargetWriterInfo>& targetWriters);
std::string mtStateUpdateTargetWriterConflictKind(
    const MtBoundaryInfo& boundary,
    const MtStateTargetWriterInfo& writerInfo,
    bool hasIncompleteWriterUniverse);
std::string mtStateUpdateTargetWriterProof(const std::string& conflictKind);
std::vector<std::string> mtStateUpdateRuntimeBlockReasons(
    const std::string& stateUpdateCandidateKind,
    const std::string& targetWriterConflictKind,
    const MtStateTargetWriterInfo& writerInfo);
std::set<std::string> collectAllMtStateTargetNames(const std::map<int, MtTaskInfo>& mtTasks);
std::vector<MtStateUpdateTraceInfo> buildMtStateUpdateTraceInfo(const std::map<int, MtTaskInfo>& mtTasks);

// cppEmitterMtPlan.cpp - coarse/pure-batch planning, profitability, assignment,
// context cache group (mtContextCache/mtDenseScheduleCache/edge caches + *ForInvocation memos)
int mtCoarseStaticRecommendedWorkers(const MtCoarseRegion& region, int configuredWorkers);
bool mtCoarseStaticAdmitsRegion(const MtCoarseRegion& region, int workerCount);
int mtCoarseRecommendedWorkersForPolicy(const MtCoarseRegion& region,
                                               int configuredWorkers,
                                               const std::string& workerPolicy);
bool mtCoarseAdmitsRegionForPolicy(const MtCoarseRegion& region,
                                          int workerCount,
                                          const std::string& workerPolicy);
std::string mtJoinIntList(const std::vector<int>& values);
MtCoarseMTaskAssignment mtBuildCoarseMTaskAssignment(const MtCoarseRegion& region,
                                                            int configuredWorkers,
                                                            const std::string& workerPolicy);
MtCoarseProfileFacts mtComputeCoarseProfileFacts(const MtCoarseRegionPlan& coarsePlan);
MtPureBatchPlan planMtPureBatchesActiveFrequency(const std::map<int, MtTaskInfo>& tasks);
MtPureBatchPlan planMtPureBatches(const std::map<int, MtTaskInfo>& tasks);
void resetMtContextCache();
std::map<int, MtTaskInfo> buildMtTaskInfoMapForInvocation();
MtCoarseRegionPlan planMtCoarseRegionsForInvocation();
std::vector<MtStateUpdateTraceInfo> buildMtStateUpdateTraceInfoForInvocation(const std::map<int, MtTaskInfo>& tasks);
void logMtReportTimer(const char* name, struct timeval start, struct timeval end);

// cppEmitterDenseSchedule.cpp - dense DAG build: edges, Kosaraju SCC, chain coarsening,
// vcontract (gVcAutoPass two-pass state is TU-local), mtask partition, owner-ready layouts
std::vector<MtDenseMTask> mtBuildDenseMTasks(const MtDenseSchedule& schedule,
                                                    bool preserveWorkerZeroOnlyBoundary);
std::vector<std::vector<int>> mtBuildDenseRuntimeSuccs(const std::vector<MtDenseMTask>& mtasks,
                                                             const std::vector<int>& assignment,
                                                             bool xthreadDepsOnly,
                                                             int* sameThreadElidedCount = nullptr);
int mtDenseRuntimeEdgeCount(const std::vector<std::vector<int>>& runtimeSuccs);
MtDenseOwnerReadyLayout mtBuildDenseOwnerReadyLayout(
    const std::vector<std::vector<int>>& runtimeSuccs,
    const std::vector<int>& assignment,
    int threadCount);
MtDenseBreakdownWindowWaitLayout mtBuildDenseBreakdownWindowWaitLayout(
    const MtDenseOwnerReadyLayout& readyLayout,
    const std::vector<int>& assignment,
    int threadCount);
MtDenseBreakdownWindowAllOwnerLayout mtBuildDenseBreakdownWindowAllOwnerLayout(
    const std::vector<int>& assignment, int threadCount);
int mtReduceDenseRuntimeSuccsTransitive(std::vector<std::vector<int>>& runtimeSuccs,
                                               const std::vector<int>& assignment,
                                               bool injectWorkerChains = true);
std::pair<std::vector<int>, int> mtBuildDensePackThreadsAssignment(const std::vector<MtDenseMTask>& mtasks,
                                                                          int threadCount);
MtDenseSchedule buildMtDenseSchedule(const std::map<int, MtTaskInfo>& tasks, bool codegenEnabled);

// cppEmitterReset.cpp - reset chunking + reset def/activation/all; owns the
// super2ResetId/super2DenseResetId/resetFuncNum write side (header-inline state)
const std::pair<int, int>& super2ResetIdLookup(Node* resetNode);
std::vector<std::string>& mtResetChunkDecls();

// cppEmitterEmitCore.cpp - header scaffolding + per-node/per-super emission core
void emitActiveBufferDef(FILE* header, int activeWords);
void emitActivationDeltaDef(FILE* header, int activeWords);
void emitActivationEventTraceDef(FILE* header);
#endif  // CPPEMITTER_IMPL_H
