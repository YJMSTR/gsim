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

#endif  // CPPEMITTER_IMPL_H
