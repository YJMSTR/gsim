/*
  cppEmitter: emit C++ files for simulation
*/

#include "common.h"
#include "util.h"

#include <cstddef>
#include <cstdio>
#include <cinttypes>
#include <cctype>
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
// 28c Phase 1A: hard cap on coarse region active-word span under mt-level-dispatch.
// Pre-implementation G-1A2.0 simulation showed raw max span 649; cap=192 keeps the
// dense merge upper bound bounded while only splitting 6 oversized regions.
#define MT_LEVEL_DISPATCH_REGION_SPAN_CAP 192

#define ENABLE_ACTIVATOR false

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
    assert(std::remove(tmpPath.c_str()) == 0);
    return;
  }
  assert(std::rename(tmpPath.c_str(), finalPath.c_str()) == 0);
}

#define RESET_NAME(node) (node->name + "$RESET")
#define emitFuncDecl(indent, ...) __emitSrc(indent, true, true, NULL, __VA_ARGS__)
#define emitBodyLock(indent, ...) __emitSrc(indent, false, false, NULL, __VA_ARGS__)

#define ActiveType std::tuple<uint64_t, std::string, int>
#define ACTIVE_MASK(active) std::get<0>(active)
#define ACTIVE_COMMENT(active) std::get<1>(active)
#define ACTIVE_UNIQUE(active) std::get<2>(active)

static int superId = 0;
static int activeFlagNum = 0;
static std::set<Node*> definedNode;
static std::map<int, SuperNode*> cppId2Super;
static std::vector<int> mtProfileRepCutBatchBeginCppIds;
static std::vector<int> mtProfileRepCutRuntimeCppIds;
static std::set<int> alwaysActive;
static std::vector<std::vector<int>> mtStepActiveWordGuards;
static std::vector<char> mtStepActiveWordGuardable;

static std::map<Node*, std::pair<int, int>> super2ResetId;  // uint & async reset
static std::map<Node*, std::pair<int, int>> super2DenseResetId;  // dense uint & async reset

extern int maxConcatNum;
bool nameExist(std::string str);
static int resetFuncNum = 0;
std::pair<int, uint64_t> setIdxMask(int cppId);

static bool isAlwaysActive(int cppId) {
  return alwaysActive.find(cppId) != alwaysActive.end();
}

static bool hasCppId(const std::set<SuperNode*>& supers, int cppId) {
  for (SuperNode* super : supers) {
    if (super && super->cppId == cppId) return true;
  }
  return false;
}

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
  std::string repcutRole = "none";
  int repcutSourceCount = 0;
  int repcutSinkCount = 0;
  int repcutFanout = 0;
  int repcutCopyCost = 0;
  std::string repcutBlockReason;
  bool repcutSelected = false;
  bool repcutRuntimeApplied = false;
  int repcutCutInEdges = 0;
  int repcutCutOutEdges = 0;
};

struct MtStateUpdateTraceInfo {
  bool hasStateUpdate = false;
  bool localSafeCandidate = false;
  bool runtimeSafeCandidate = false;
  std::string targetWriterConflictKind = "none";
  std::vector<std::string> runtimeBlockReasons;
};


struct MtRepCutEdge {
  int fromCppId = -1;
  int toCppId = -1;
  std::string reason;
};

struct MtPureBatchPlan {
  std::vector<std::pair<int, int>> batches;
  std::vector<MtRepCutEdge> cutEdges;
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
  // Track 2 Week 3: per-mtask dependency graph for atomic-counter scheduling.
  std::vector<int> predMTaskIndices;
  std::vector<int> succMTaskIndices;
  int upstreamDepCount = 0;
  // Track 2 Week 4: serial/hazard singletons must stay on worker 0 for ordering correctness.
  bool workerZeroOnly = false;
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
  int safeSerialTaskCount = 0;   // 28c Phase 1A: serial cppIds admitted under mt-level-dispatch
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
  bool repcutLiteCouldHelp = false;
  std::vector<std::string> blockers;
  std::vector<MtCoarseLayer> layers;
  std::vector<MtCoarseMTask> mtasks;
  std::vector<MtCoarseMTask> antichainProbeGroups;   // Track 2 Week 2: report-only inside-component antichain grouping
  int antichainProbeMaxBlockWidth = 0;                  // max chain-cover width across non-serial blocks
  int antichainProbeTotalGroups = 0;                    // total groups incl. serial singletons (may be large)
  bool antichainProbeDagAcyclic = false;                // Track 2 Week 3: quotient DAG on antichainProbeGroups acyclic
  bool useAntichainRuntime = false;                     // Track 2 Week 4: route this region through atomic-counter scheduler
  std::string antichainSelectionReason;
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
  // v236: real dense scheduling cost (sum of member-node work of contained SCCs).
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

struct MtRepCutLocalDecl {
  Node* node = nullptr;
  std::string cloneName;
  std::string expr;
  int exprCost = 0;
};

struct MtRepCutClone {
  int sourceCppId = -1;
  int sinkCppId = -1;
  Node* sourceNode = nullptr;
  std::string cloneName;
  std::string expr;
  std::string fallbackReason;
  int sourceExprCost = 0;
  int localExprCost = 0;
  int plannedCloneCost = 0;
  std::vector<MtRepCutLocalDecl> localDecls;
};

struct MtRepCutBatch {
  int beginCppId = -1;
  int endCppId = -1;
  int cutEdgeCount = 0;
  int cloneCount = 0;
  uint64_t forcedSinkMask = 0;
  std::set<int> forcedSinkCppIds;
  bool forcedSerial = true;
  bool parallelSafe = false;
  bool forcedSinkActivation = false;
  std::string parallelSafeReason;
  std::string fallbackReason;
  std::map<std::string, int> cloneFallbackReasons;
  int plannedLocalDeclCount = 0;
  int plannedExprCost = 0;
  int plannedCloneCost = 0;
  int emittedCloneCount = 0;
  int emittedLocalDeclCount = 0;
  int emittedExprCost = 0;
  int emittedCloneCost = 0;
};

struct MtRepCutSemanticPlan {
  MtPureBatchPlan batchPlan;
  std::vector<MtRepCutClone> clones;
  std::vector<MtRepCutBatch> cutBatches;
};

static const char* nodeTypeName(NodeType type) {
  switch (type) {
    case NODE_INVALID: return "NODE_INVALID";
    case NODE_REG_SRC: return "NODE_REG_SRC";
    case NODE_REG_DST: return "NODE_REG_DST";
    case NODE_SPECIAL: return "NODE_SPECIAL";
    case NODE_INP: return "NODE_INP";
    case NODE_OUT: return "NODE_OUT";
    case NODE_MEMORY: return "NODE_MEMORY";
    case NODE_READER: return "NODE_READER";
    case NODE_WRITER: return "NODE_WRITER";
    case NODE_READWRITER: return "NODE_READWRITER";
    case NODE_INFER: return "NODE_INFER";
    case NODE_OTHERS: return "NODE_OTHERS";
    case NODE_REG_RESET: return "NODE_REG_RESET";
    case NODE_EXT_IN: return "NODE_EXT_IN";
    case NODE_EXT_OUT: return "NODE_EXT_OUT";
    case NODE_EXT: return "NODE_EXT";
  }
  return "NODE_UNKNOWN";
}

static bool isKnownNodeType(NodeType type) {
  switch (type) {
    case NODE_INVALID:
    case NODE_REG_SRC:
    case NODE_REG_DST:
    case NODE_SPECIAL:
    case NODE_INP:
    case NODE_OUT:
    case NODE_MEMORY:
    case NODE_READER:
    case NODE_WRITER:
    case NODE_READWRITER:
    case NODE_INFER:
    case NODE_OTHERS:
    case NODE_REG_RESET:
    case NODE_EXT_IN:
    case NODE_EXT_OUT:
    case NODE_EXT:
      return true;
  }
  return false;
}

static const char* superTypeName(SuperType type) {
  switch (type) {
    case SUPER_VALID: return "SUPER_VALID";
    case SUPER_EXTMOD: return "SUPER_EXTMOD";
    case SUPER_ASYNC_RESET: return "SUPER_ASYNC_RESET";
    case SUPER_UINT_RESET: return "SUPER_UINT_RESET";
    case SUPER_UPDATE_REG: return "SUPER_UPDATE_REG";
  }
  return "SUPER_UNKNOWN";
}
static const char* superInfoName(SuperInfo info) {
  switch (info) {
    case SUPER_INFO_IF: return "if";
    case SUPER_INFO_ELSE: return "else";
    case SUPER_INFO_DEDENT: return "dedent";
    case SUPER_INFO_STR: return "str";
    case SUPER_INFO_ASSIGN_BEG: return "assign_beg";
    case SUPER_INFO_ASSIGN_END: return "assign_end";
  }
  return "unknown";
}

static std::string jsonEscape(const std::string& str) {
  std::string ret;
  for (char ch : str) {
    switch (ch) {
      case '\\': ret += "\\\\"; break;
      case '"': ret += "\\\""; break;
      case '\b': ret += "\\b"; break;
      case '\f': ret += "\\f"; break;
      case '\n': ret += "\\n"; break;
      case '\r': ret += "\\r"; break;
      case '\t': ret += "\\t"; break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20) ret += format("\\u%04x", ch);
        else ret += ch;
        break;
    }
  }
  return ret;
}


static void dumpJsonIntArray(FILE* fp, const std::set<int>& values) {
  fprintf(fp, "[");
  bool first = true;
  for (int value : values) {
    if (!first) fprintf(fp, ", ");
    first = false;
    fprintf(fp, "%d", value);
  }
  fprintf(fp, "]");
}

static void dumpJsonIntArray(FILE* fp, const std::vector<int>& values) {
  fprintf(fp, "[");
  bool first = true;
  for (int value : values) {
    if (!first) fprintf(fp, ", ");
    first = false;
    fprintf(fp, "%d", value);
  }
  fprintf(fp, "]");
}

static void dumpJsonStringArray(FILE* fp, const std::set<std::string>& values) {
  fprintf(fp, "[");
  bool first = true;
  for (const std::string& value : values) {
    if (!first) fprintf(fp, ", ");
    first = false;
    fprintf(fp, "\"%s\"", jsonEscape(value).c_str());
  }
  fprintf(fp, "]");
}

static void dumpJsonStringArray(FILE* fp, const std::vector<std::string>& values) {
  fprintf(fp, "[");
  bool first = true;
  for (const std::string& value : values) {
    if (!first) fprintf(fp, ", ");
    first = false;
    fprintf(fp, "\"%s\"", jsonEscape(value).c_str());
  }
  fprintf(fp, "]");
}

static bool mtUseDenseActivationOrigins() {
  const char* env = std::getenv("GSIM_MT_DENSE_ACTIVATION_ORIGINS");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

static bool mtUseDenseMemberMetadata() {
  const char* env = std::getenv("GSIM_MT_DENSE_MEMBER_METADATA");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// v198: When enabled, exclude backward (cross-cycle) activation edges from the dense SCC graph.
// Backward activation edges are next-cycle activations that create false within-cycle cycles.
static bool mtUseDenseForwardActivationOnly() {
  const char* env = std::getenv("GSIM_MT_DENSE_FORWARD_ACTIVATION_ONLY");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// v194: dump member-node and instruction-ownership metadata for a single dense task.
// Reads already-computed super->insts (populated by instsGenerator before cppEmitter);
// never calls StmtTree::compute and never mutates super->insts.
static void dumpMtDenseMemberMetadataForTask(FILE* fp, SuperNode* super) {
  fprintf(fp, ", \"member_nodes\": [");
  bool first = true;
  for (Node* node : super->member) {
    if (!node) continue;
    if (!first) fprintf(fp, ", ");
    first = false;
    fprintf(fp, "{\"node_id\": %d, \"node_name\": \"%s\", \"node_type\": \"%s\"}",
            node->id, jsonEscape(node->name).c_str(), nodeTypeName(node->type));
  }
  fprintf(fp, "], \"instruction_ownership\": [");
  first = true;
  for (const InstInfo& inst : super->insts) {
    if (!first) fprintf(fp, ", ");
    first = false;
    fprintf(fp, "{\"kind\": \"%s\", \"inst\": \"%s\"", superInfoName(inst.infoType), jsonEscape(inst.inst).c_str());
    // Only ASSIGN_BEG/ASSIGN_END constructors initialize InstInfo::node;
    // the string constructor leaves it uninitialized, so check infoType first.
    if ((inst.infoType == SUPER_INFO_ASSIGN_BEG || inst.infoType == SUPER_INFO_ASSIGN_END) && inst.node) {
      fprintf(fp, ", \"owner_node_id\": %d, \"owner_node_name\": \"%s\", \"owner_node_type\": \"%s\"",
              inst.node->id, jsonEscape(inst.node->name).c_str(), nodeTypeName(inst.node->type));
    }
    fprintf(fp, "}");
  }
  fprintf(fp, "]");
}

static bool mtDenseActivationOriginEdgeEligible(int fromCppId, int toCppId) {
  return fromCppId >= 0 && toCppId >= 0 && fromCppId < superId && toCppId < superId && fromCppId != toCppId;
}

static void dumpMtDenseActivationOriginRecord(FILE* fp,
                                             const Node::ActivationOriginRecord& origin,
                                             const char* kind,
                                             bool& first) {
  if (!mtDenseActivationOriginEdgeEligible(origin.fromCppId, origin.toCppId)) return;
  if (!first) fprintf(fp, ",\n");
  first = false;
  fprintf(fp, "    {\"from_cpp_id\": %d, \"to_cpp_id\": %d, \"kind\": \"%s\", ", origin.fromCppId, origin.toCppId, kind);
  fprintf(fp, "\"reason\": \"%s\", ", jsonEscape(origin.reason).c_str());
  fprintf(fp, "\"source_node_id\": %d, \"source_node_name\": \"%s\", \"source_node_type\": \"%s\", ",
          origin.sourceNodeId, jsonEscape(origin.sourceNodeName).c_str(), nodeTypeName(origin.sourceNodeType));
  fprintf(fp, "\"target_node_id\": %d, \"target_node_name\": \"%s\", \"target_node_type\": \"%s\"}",
          origin.targetNodeId, jsonEscape(origin.targetNodeName).c_str(), nodeTypeName(origin.targetNodeType));
}

static void dumpMtDenseActivationOrigins(FILE* fp) {
  bool enabled = mtUseDenseActivationOrigins();
  fprintf(fp, "  \"activation_origin_capture_enabled\": %s,\n", enabled ? "true" : "false");
  fprintf(fp, "  \"activation_origins\": [\n");
  bool first = true;
  if (enabled) {
    for (int cppId = 0; cppId < superId; cppId ++) {
      auto superIter = cppId2Super.find(cppId);
      if (superIter == cppId2Super.end() || !superIter->second) continue;
      for (Node* member : superIter->second->member) {
        if (!member) continue;
        for (const Node::ActivationOriginRecord& origin : member->activationOrigins()) {
          dumpMtDenseActivationOriginRecord(fp, origin, "active", first);
          if (!isAlwaysActive(origin.toCppId)) {
            dumpMtDenseActivationOriginRecord(fp, origin, "need_activate", first);
          }
        }
      }
    }
  }
  fprintf(fp, "\n  ],\n");
}

static void addCppIdIfExecutable(std::set<int>& ids, SuperNode* super) {
  if (super && super->cppId >= 0) ids.insert(super->cppId);
}

static void addCppIdsIfExecutable(std::set<int>& ids, const std::set<SuperNode*>& supers) {
  for (SuperNode* super : supers) addCppIdIfExecutable(ids, super);
}

static bool nodeHasStateUpdate(Node* node) {
  return node->type == NODE_REG_DST || node->type == NODE_REG_RESET ||
         (node->type == NODE_REG_SRC && node->regNext && node->regNext->status == VALID_NODE);
}

static bool stateTargetNameForNode(Node* node, std::string& targetName) {
  if (!node) return false;
  Node* target = nullptr;
  if (node->type == NODE_REG_SRC) {
    if (!node->regNext || node->regNext->status != VALID_NODE) return false;
    target = node;
  } else if (node->type == NODE_REG_DST) {
    if (!node->regNext) return false;
    target = node->getSrc();
  } else if (node->type == NODE_REG_RESET) {
    if (!node->regNext) return false;
    target = node->getResetSrc();
  } else {
    return false;
  }
  if (!target || target->name.empty()) return false;
  targetName = target->name;
  return true;
}

static void collectStateTargetName(Node* node, MtBoundaryInfo& info) {
  if (!nodeHasStateUpdate(node)) return;
  if (node->type == NODE_REG_SRC) info.stateSourceCommitCount ++;
  else if (node->type == NODE_REG_DST) info.stateNextUpdateCount ++;
  else if (node->type == NODE_REG_RESET) info.stateResetUpdateCount ++;
  std::string targetName;
  if (stateTargetNameForNode(node, targetName)) {
    info.stateTargetNames.insert(targetName);
  } else {
    info.hasAmbiguousStateTarget = true;
  }
}

static const size_t MT_RHS_STATE_READ_ENODE_LIMIT = 128;
static const size_t MT_RHS_STATE_READ_EXPAND_NODE_LIMIT = 32;

static void collectMtRhsStateReads(ENode* root,
                                   MtBoundaryInfo& info,
                                   Node* owner,
                                   std::set<ENode*>& visitedENodes,
                                   std::set<Node*>& expandedNodes) {
  if (!root) return;
  std::stack<ENode*> stack;
  stack.push(root);
  while (!stack.empty()) {
    ENode* top = stack.top();
    stack.pop();
    if (!top) continue;
    if (visitedENodes.find(top) != visitedENodes.end()) continue;
    if (visitedENodes.size() >= MT_RHS_STATE_READ_ENODE_LIMIT) {
      info.hasUnexpandedRhsDependency = true;
      return;
    }
    visitedENodes.insert(top);
    if (top->nodePtr) {
      if (top->nodePtr->type == NODE_REG_SRC) {
        info.rhsReadStateTargetNames.insert(top->nodePtr->name);
      } else if (top->nodePtr->type == NODE_REG_DST || top->nodePtr->type == NODE_REG_RESET) {
        bool expectedCommitRead = owner && owner->type == NODE_REG_SRC &&
                                  top->nodePtr->type == NODE_REG_DST &&
                                  owner->getDst() == top->nodePtr;
        if (!expectedCommitRead) info.hasRhsNextStateObjectRead = true;
      } else if (!top->nodePtr->assignTree.empty() &&
                 expandedNodes.find(top->nodePtr) == expandedNodes.end()) {
        if (expandedNodes.size() >= MT_RHS_STATE_READ_EXPAND_NODE_LIMIT) {
          info.hasUnexpandedRhsDependency = true;
          continue;
        }
        expandedNodes.insert(top->nodePtr);
        // Bounded expansion preserves small local proofs without recursively
        // exploding through large XiangShan combinational cones.
        for (ExpTree* tree : top->nodePtr->assignTree) {
          collectMtRhsStateReads(tree->getRoot(), info, owner, visitedENodes, expandedNodes);
        }
      }
    }
    for (ENode* child : top->child) {
      if (child) stack.push(child);
    }
  }
}

static bool nodeHasMemoryWrite(Node* node) {
  return node->type == NODE_WRITER || node->type == NODE_READWRITER;
}

static bool nodeHasMemoryRead(Node* node) {
  return node->type == NODE_READER || node->type == NODE_READWRITER;
}

static void addSerialReason(std::vector<std::string>& reasons, const std::string& reason) {
  if (std::find(reasons.begin(), reasons.end(), reason) == reasons.end()) {
    reasons.push_back(reason);
  }
}

static void visitMtENode(ENode* root, MtBoundaryInfo& info, int& cost) {
  if (!root) return;
  std::stack<ENode*> stack;
  stack.push(root);
  while (!stack.empty()) {
    ENode* top = stack.top();
    stack.pop();
    if (!top) continue;

    if (top->nodePtr) {
      if (top->nodePtr->isArray()) info.hasArrayOrDynamicIndex = true;
      if (!isKnownNodeType(top->nodePtr->type) || top->nodePtr->type == NODE_INVALID || top->nodePtr->type == NODE_INFER) {
        info.hasUnknownNode = true;
      }
    } else {
      switch (top->opType) {
        case OP_EMPTY:
        case OP_INT:
        case OP_INDEX_INT:
          break;
        case OP_ADD:
        case OP_SUB:
        case OP_MUL:
        case OP_DIV:
        case OP_REM:
        case OP_LT:
        case OP_LEQ:
        case OP_GT:
        case OP_GEQ:
        case OP_EQ:
        case OP_NEQ:
        case OP_DSHL:
        case OP_DSHR:
        case OP_AND:
        case OP_OR:
        case OP_XOR:
        case OP_CAT:
        case OP_ASUINT:
        case OP_ASSINT:
        case OP_ASCLOCK:
        case OP_ASASYNCRESET:
        case OP_CVT:
        case OP_NEG:
        case OP_NOT:
        case OP_ANDR:
        case OP_ORR:
        case OP_XORR:
        case OP_PAD:
        case OP_SHL:
        case OP_SHR:
        case OP_HEAD:
        case OP_TAIL:
        case OP_BITS:
        case OP_BITS_NOSHIFT:
        case OP_MUX:
        case OP_WHEN:
        case OP_GROUP:
        case OP_SEXT:
        case OP_STMT_SEQ:
        case OP_STMT_WHEN:
        case OP_STMT_NODE:
          cost ++;
          break;
        case OP_INDEX:
          info.hasArrayOrDynamicIndex = true;
          cost ++;
          break;
        case OP_READ_MEM:
          info.hasMemoryRead = true;
          cost ++;
          break;
        case OP_WRITE_MEM:
        case OP_INFER_MEM:
          info.hasMemoryWrite = true;
          cost ++;
          break;
        case OP_RESET:
          info.hasReset = true;
          cost ++;
          break;
        case OP_PRINTF:
        case OP_ASSERT:
        case OP_EXIT:
          info.hasSpecial = true;
          cost ++;
          break;
        case OP_EXT_FUNC:
          info.hasExternal = true;
          cost ++;
          break;
        case OP_INVALID:
        default:
          info.hasUnknownOp = true;
          break;
      }
    }

    for (ENode* child : top->child) {
      if (child) stack.push(child);
    }
  }
}

static int mtENodeStaticCost(ENode* root) {
  MtBoundaryInfo info;
  int cost = 0;
  visitMtENode(root, info, cost);
  return cost;
}

static MtBoundaryInfo collectMtBoundaryInfo(SuperNode* super, int& candidateCost) {
  MtBoundaryInfo info;
  candidateCost = 0;
  info.hasStateUpdate = super->superType == SUPER_UPDATE_REG;
  info.hasReset = super->superType == SUPER_ASYNC_RESET || super->superType == SUPER_UINT_RESET;
  info.hasAsyncReset = super->superType == SUPER_ASYNC_RESET;
  info.hasExternal = super->superType == SUPER_EXTMOD;

  for (Node* member : super->member) {
    info.nodeKinds[nodeTypeName(member->type)] ++;
    info.hasStateUpdate = info.hasStateUpdate || nodeHasStateUpdate(member);
    collectStateTargetName(member, info);
    info.hasMemoryWrite = info.hasMemoryWrite || nodeHasMemoryWrite(member);
    info.hasMemoryRead = info.hasMemoryRead || nodeHasMemoryRead(member);
    info.hasReset = info.hasReset || member->isReset() || member->type == NODE_REG_RESET;
    info.hasAsyncReset = info.hasAsyncReset || member->isAsyncReset() || member->reset == ASYRESET;
    info.hasExternal = info.hasExternal || member->isExt();
    info.hasSpecial = info.hasSpecial || member->type == NODE_SPECIAL;
    info.hasArrayOrDynamicIndex = info.hasArrayOrDynamicIndex || member->isArray();
    if (!isKnownNodeType(member->type) || member->type == NODE_INVALID || member->type == NODE_INFER || member->type == NODE_MEMORY) {
      info.hasUnknownNode = true;
    }
    if (member->clock) info.clockNames.insert(member->clock->name);
    for (ExpTree* tree : member->assignTree) {
      visitMtENode(tree->getRoot(), info, candidateCost);
      visitMtENode(tree->getlval(), info, candidateCost);
      if (nodeHasStateUpdate(member)) {
        std::set<ENode*> visitedENodes;
        std::set<Node*> expandedNodes;
        collectMtRhsStateReads(tree->getRoot(), info, member, visitedENodes, expandedNodes);
      }
    }
    if (member->resetTree) {
      visitMtENode(member->resetTree->getRoot(), info, candidateCost);
      visitMtENode(member->resetTree->getlval(), info, candidateCost);
    }
  }

  if (super->resetNode) {
    info.hasReset = true;
    info.hasAsyncReset = info.hasAsyncReset || super->resetNode->isAsyncReset() || super->resetNode->reset == ASYRESET;
    if (super->resetNode->clock) info.clockNames.insert(super->resetNode->clock->name);
  }

  info.hasActivateAllPath = info.hasAsyncReset;
  return info;
}

static MtTaskInfo classifyMtTask(SuperNode* super) {
  MtTaskInfo task;
  int candidateCost = 0;
  task.boundary = collectMtBoundaryInfo(super, candidateCost);

  if (super->superType != SUPER_VALID) {
    addSerialReason(task.serialReasons, "super_type_" + std::string(superTypeName(super->superType)));
  }
  if (task.boundary.hasStateUpdate) addSerialReason(task.serialReasons, "state_update");
  if (task.boundary.hasMemoryWrite) addSerialReason(task.serialReasons, "memory_write");
  if (task.boundary.hasMemoryRead) addSerialReason(task.serialReasons, "memory_read_unsupported");
  if (task.boundary.hasReset) addSerialReason(task.serialReasons, "reset");
  if (task.boundary.hasAsyncReset) addSerialReason(task.serialReasons, "async_reset");
  if (task.boundary.hasActivateAllPath) addSerialReason(task.serialReasons, "activate_all_path");
  if (task.boundary.hasExternal) addSerialReason(task.serialReasons, "external");
  if (task.boundary.hasSpecial) addSerialReason(task.serialReasons, "special");
  if (task.boundary.hasUnknownNode) addSerialReason(task.serialReasons, "unknown_node");
  if (task.boundary.hasUnknownOp) addSerialReason(task.serialReasons, "unknown_op");
  if (task.boundary.hasArrayOrDynamicIndex) addSerialReason(task.serialReasons, "array_or_dynamic_index");

  if (task.serialReasons.empty()) {
    task.taskKind = "pure_compute";
    task.hasCandidateCost = true;
    task.candidateCost = std::max(1, candidateCost);
  } else {
    task.taskKind = "serial";
  }
  return task;
}

static std::map<int, MtTaskInfo> buildMtTaskInfoMap() {
  std::map<int, MtTaskInfo> tasks;
  for (int cppId = 0; cppId < superId; cppId ++) {
    tasks[cppId] = classifyMtTask(cppId2Super[cppId]);
  }

  for (auto& iter : tasks) {
    int cppId = iter.first;
    MtTaskInfo& task = iter.second;
    if (task.taskKind != "pure_compute") continue;

    SuperNode* super = cppId2Super[cppId];
    bool hasPred = false;
    bool hasSucc = false;
    auto checkPred = [&](SuperNode* pred) {
      if (!pred || pred->cppId < 0) return;
      hasPred = true;
      if (tasks[pred->cppId].taskKind == "serial") task.isSource = true;
    };
    auto checkSucc = [&](SuperNode* succ) {
      if (!succ || succ->cppId < 0) return;
      hasSucc = true;
      if (tasks[succ->cppId].taskKind == "serial") task.isSink = true;
    };
    for (SuperNode* pred : super->prev) checkPred(pred);
    for (SuperNode* pred : super->depPrev) checkPred(pred);
    for (SuperNode* succ : super->next) checkSucc(succ);
    for (SuperNode* succ : super->depNext) checkSucc(succ);
    for (Node* member : super->member) {
      for (int activeId : member->nextNeedActivate) {
        if (activeId < 0) continue;
        hasSucc = true;
        if (tasks[activeId].taskKind == "serial") task.isSink = true;
      }
    }
    if (!hasPred) task.isSource = true;
    if (!hasSucc) task.isSink = true;
  }
  return tasks;
}

static bool mtTasksHaveEdge(SuperNode* lhs, SuperNode* rhs) {
  int lhsId = lhs->cppId;
  int rhsId = rhs->cppId;
  if (hasCppId(lhs->prev, rhsId) || hasCppId(lhs->next, rhsId) ||
      hasCppId(lhs->depPrev, rhsId) || hasCppId(lhs->depNext, rhsId) ||
      hasCppId(rhs->prev, lhsId) || hasCppId(rhs->next, lhsId) ||
      hasCppId(rhs->depPrev, lhsId) || hasCppId(rhs->depNext, lhsId)) {
    return true;
  }
  for (Node* member : lhs->member) {
    if (member->nextActiveId.find(rhsId) != member->nextActiveId.end()) return true;
  }
  for (Node* member : rhs->member) {
    if (member->nextActiveId.find(lhsId) != member->nextActiveId.end()) return true;
  }
  return false;
}

static bool mtTasksHaveDirectedEdge(SuperNode* from, SuperNode* to) {
  int toId = to->cppId;
  int fromId = from->cppId;
  if (hasCppId(from->next, toId) || hasCppId(from->depNext, toId) ||
      hasCppId(to->prev, fromId) || hasCppId(to->depPrev, fromId)) {
    return true;
  }
  for (Node* member : from->member) {
    if (member->nextActiveId.find(toId) != member->nextActiveId.end()) return true;
  }
  return false;
}

// 28c Phase 1A: any of these reasons forces the cppId to run on worker 0 (single-threaded
// fall-through). external/memory_write/memory_read_unsupported/special are kimi-2.7-code
// review I5 conservative. super_type_SUPER_EXTMOD covers the alwaysActive set.
static bool hasWorker0OnlyReason(const std::vector<std::string>& reasons) {
  for (const std::string& r : reasons) {
    if (r == "external" || r == "memory_write" ||
        r == "memory_read_unsupported" || r == "special" ||
        r == "super_type_SUPER_EXTMOD") {
      return true;
    }
  }
  return false;
}

// A44: explicit allowlist for direct serial fallback beyond pure_compute.
// These classes are already admitted by mt-level-dispatch and are safe only
// when executed by the original one-arg mtTaskN(flag) in ST scan order. Keep
// worker0-only side effects, unknown_node/unknown_op, uint reset, and future reasons out by default.
static bool hasOnlyA44DirectFallbackReasons(const std::vector<std::string>& reasons) {
  if (reasons.empty()) return true;
  for (const std::string& r : reasons) {
    if (r == "state_update" || r == "reset" || r == "async_reset" ||
        r == "activate_all_path" || r == "array_or_dynamic_index" ||
        r == "super_type_SUPER_ASYNC_RESET") {
      continue;
    }
    return false;
  }
  return true;
}

static bool hasOnlyA73Worker0SafeReasons(const std::vector<std::string>& reasons) {
  if (reasons.empty()) return false;
  bool hasWorker0Safe = false;
  for (const std::string& r : reasons) {
    if (r == "memory_write" || r == "memory_read_unsupported" || r == "special") {
      hasWorker0Safe = true;
      continue;
    }
    if (r == "state_update" || r == "reset" || r == "async_reset" ||
        r == "activate_all_path" || r == "array_or_dynamic_index" ||
        r == "super_type_SUPER_ASYNC_RESET") {
      continue;
    }
    return false;
  }
  return hasWorker0Safe;
}

static bool mtIsLevelDispatchMode() {
  return globalConfig.MtHelperMode == "mt-level-dispatch";
}

static bool mtCodegenEnvEnabledByDefault(const char* name) {
  const char* env = std::getenv(name);
  return env == nullptr || env[0] == '\0' || env[0] != '0';
}


static bool mtUseDirectInlineFallback() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_DIRECT_INLINE_FALLBACK");
}

static bool mtUseDirectInlineSerialFallback() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_DIRECT_INLINE_SERIAL_FALLBACK");
}

static bool mtUseDirectInlineWorker0Fallback() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_DIRECT_INLINE_WORKER0_FALLBACK");
}

// A77 D1-NARROW / A110 Probe 5: codegen-time, default-on profile-off fast path.
// The retained diagnostic/profile-capable emission is still available by setting
// GSIM_MT_PROFILE_OFF_DIRECT_SERIAL_FALLBACK=0 during gsim-gen-cpp. Default-on
// drops serial-fallback profile wrappers in the generated profile-off hot path.
static bool mtUseProfileOffDirectSerialFallback() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_PROFILE_OFF_DIRECT_SERIAL_FALLBACK");
}

// Default-on profile-off fast path for non-coarse active-word accounting in
// generated MT substeps. Set GSIM_MT_PROFILE_OFF_ACTIVE_WORD_COUNT=0 during
// gsim-gen-cpp to retain the diagnostic counter branch.
static bool mtUseProfileOffActiveWordCount() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_PROFILE_OFF_ACTIVE_WORD_COUNT");
}

// Default-on fast path for pure batches that runtime will force to workerCount=1
// under the default min-batch threshold. The generated branch preserves runtime
// lowering of GSIM_MT_MIN_BATCH_TASKS by falling back to mtRunPureBatch().
static bool mtUseInlineSmallPureBatches() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_INLINE_SMALL_PURE_BATCHES");
}

// Probe: inside the small-batch inline path, optionally inline each pure task
// body instead of calling mtTaskN(oldFlag). RepCut helpers still use their
// dedicated helper because they need cloned-value delta plumbing.
static bool mtUseInlineSmallPureBatchBodies() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_INLINE_SMALL_PURE_BATCH_BODIES");
}

// Default-on guard around inlined small-batch task bodies. When no task bit in
// the static batch mask is active, skip the generated per-bit branch chain.
static bool mtUseInlineSmallPureBatchMaskGuard() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_INLINE_SMALL_PURE_BATCH_MASK_GUARD");
}

// Default-on algorithmic fast path: skip subStep function calls whose generated
// body only consumes currently-zero active-flag words. This suppresses call
// overhead above the existing in-function sparse guards. Set
// GSIM_MT_STEP_ACTIVE_WORD_GUARD=0 during codegen to retain the older shape.
static bool mtUseStepActiveWordGuard() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_STEP_ACTIVE_WORD_GUARD");
}

static bool mtUseSplitMixedStepGuards() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_SPLIT_MIXED_STEP_GUARDS");
}


// Default-off diagnostic codegen: wait-probe instrumentation is useful for
// scheduler experiments but should not perturb normal generated models.
static bool mtUseWaitProbeCodegen() {
  const char* env = std::getenv("GSIM_MT_WAIT_PROBE_CODEGEN");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Probe-only: emit static graph data for runtime/cycle clean-region batching
// validation. Default-off and report-only; normal generated execution is unchanged.
static bool mtUseCycleBatchReport() {
  const char* env = std::getenv("GSIM_MT_CYCLE_BATCH_REPORT");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Probe-only: emit lane-local SCC ready-batch scheduler metadata. Default-off
// and report-only; normal generated execution is unchanged.
static bool mtUseReadyBatchReport() {
  const char* env = std::getenv("GSIM_MT_READY_BATCH_REPORT");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Probe-only: evaluate a gap-inclusive envelope SCC schedule over an existing
// dynamic trace. Default-off and report-only; it does not emit runtime code.
static bool mtUseEnvelopeLocalEval() {
  const char* env = std::getenv("GSIM_MT_ENVELOPE_LOCAL_EVAL");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Probe-only: decompose local envelope-eval pressure into serial reasons,
// active-word concentration, and counterfactual schedules. Default-off and
// report-only; normal generated execution is unchanged.
static bool mtUseEnvelopeLocalEvalDiagnostics() {
  const char* env = std::getenv("GSIM_MT_ENVELOPE_LOCAL_EVAL_DIAGNOSTICS");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Probe-only: include generated runtime support for state-update dynamic hit
// trace lines. Default-off at codegen; emitted models still require
// GSIM_MT_DYNAMIC_STATE_TRACE=1 alongside GSIM_MT_DYNAMIC_TRACE at runtime.
static bool mtUseDynamicStateTraceCodegen() {
  const char* env = std::getenv("GSIM_MT_DYNAMIC_STATE_TRACE_CODEGEN");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Default-off v181 codegen: emit a whole-design dense schedule report and,
// when the graph is acyclic, dense runtime wrappers selected by GSIM_MT_EXECUTOR=dense.
static bool mtUseDenseExecutorCodegen() {
  const char* env = std::getenv("GSIM_MT_DENSE_EXECUTOR_CODEGEN");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

static bool mtUseDenseXThreadDepsOnly() {
  const char* env = std::getenv("GSIM_MT_DENSE_XTHREAD_DEPS_ONLY");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

static bool mtUseDenseTransitiveReduceEdges() {
  const char* env = std::getenv("GSIM_MT_DENSE_TRANSITIVE_REDUCE_EDGES");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

static bool mtUseDenseSplitWorker0MTasks() {
  const char* env = std::getenv("GSIM_MT_DENSE_SPLIT_WORKER0_MTASKS");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}
// v236: Verilator-style critical-path MTask contraction. Default-off. Replaces the
// fixed 30-SCC topological chunking in mtBuildDenseMTasks with edge/sibling-score
// contraction bounded by cpLimit and maxMTasks (see docs/verilator-partition-spec.md).
static bool mtUseDenseCpContraction() {
  const char* env = std::getenv("GSIM_MT_DENSE_CP_CONTRACTION");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// v236: wire the Verilator-like PackThreads DAG-aware assignment (already used only
// for the report) into the codegen mtaskThreadAssign, replacing i % threadCount.
static bool mtUseDensePackThreadsAssignment() {
  const char* env = std::getenv("GSIM_MT_DENSE_PACKTHREADS_ASSIGNMENT");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// v239: dense runtime work-stealing. Default-off. Replaces the fixed ascending-id per-thread
// MTask execution (which spin-stalls on cross-thread deps) with owner-affine ready deques +
// steal-from-tail: a worker runs any READY assigned MTask, steals when idle. Lifts scaling
// past the ~3x cap of the fixed-order executor. See docs/codex-dense-direction.md.
static bool mtUseDenseWorkSteal() {
  const char* env = std::getenv("GSIM_MT_DENSE_WORKSTEAL");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}


// Probe-only: emit extra counters for dynamic work inside the clean coarse
// serial-inline fallback. Codegen-gated so normal generated models keep the
// old hot path; runtime profiling still controls whether counters are updated.
static bool mtUseSubchunkProbe() {
  const char* env = std::getenv("GSIM_MT_SUBCHUNK_PROBE");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Probe-only: iterate active bits in clean coarse serial-inline words using
// ascending ctz instead of emitting one fixed branch per bit. The generated loop
// re-reads the flag after each task but masks off already-scanned lower bits, so
// forward same-word activations are preserved while backward activations keep the
// original fixed-order semantics.
static bool mtUseCtzCoarseInlineWord() {
  const char* env = std::getenv("GSIM_MT_CTZ_COARSE_INLINE_WORD");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Probe-only: optimize the common sparse case where a clean coarse serial-inline
// word starts with exactly one active bit. Execute that bit directly, then scan
// only higher bits so forward same-word activations remain visible.
static bool mtUseSingleBitCoarseInlineWord() {
  const char* env = std::getenv("GSIM_MT_SINGLEBIT_COARSE_INLINE_WORD");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Default-on active-path optimization: when a clean coarse region's static
// maximum active bits cannot exceed the runtime inline threshold, generated code
// skips the per-region popcount scan and goes straight to serial-inline fallback.
// Lower runtime thresholds still fall back to the existing dynamic popcount gate.
// Set GSIM_MT_STATIC_INLINE_BOUND=0 during codegen to emit the old always-popcount gate.
static bool mtUseStaticCoarseInlineBound() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_STATIC_INLINE_BOUND");
}

// Runtime partial-subchunk dispatch is now generated only when explicitly
// requested. After static-bound serial-inline removed the popcount threshold
// scan from the default hot path, the smaller no-subchunk model is consistently
// slightly faster on XiangShan/CoreMark. Set GSIM_MT_SUBCHUNK_RUNTIME=1 during
// gsim-gen-cpp to emit the diagnostic/runtime subchunk fields, branch,
// counters, and printf paths.
static bool mtUseSubchunkRuntime() {
  const char* env = std::getenv("GSIM_MT_SUBCHUNK_RUNTIME");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Default-off XiangShan diagnostic: allow RepCut-lite cloned sink helpers to be
// used under mt-level-dispatch. Keep off by default so promoted direct-inline
// serial/coarse fallbacks are not disabled by selected-but-cold RepCut tasks.
static bool mtUseLevelDispatchRepCutRuntime() {
  const char* env = std::getenv("GSIM_MT_REPCUT_LEVEL_RUNTIME");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Experimental RepCut runtime probe. Codegen-gated and default-off: when set,
// parallel-safe RepCut batches may bypass the global min-batch single-worker
// clamp so the cloned-value path can be measured on real designs.
static bool mtForceParallelRepCutBatches() {
  const char* env = std::getenv("GSIM_MT_REPCUT_FORCE_PARALLEL");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// 28c Phase 1A: admission gate for the coarse region under mt-level-dispatch.
// pure_compute matches mtTaskCanEnterPureBatch; safe-serial cppIds whose only
// serial_reasons are state_update/reset/async_reset/activate_all_path/
// array_or_dynamic_index/super_type_SUPER_ASYNC_RESET are also admitted.
// Worker0-only side effects (external/memory_write/memory_read_unsupported/special)
// and future/unknown serial reasons are rejected: they fall through to the
// main-thread serial path.
static bool mtTaskCanEnterCoarseDispatch(const std::map<int, MtTaskInfo>& tasks, int cppId) {
  auto iter = tasks.find(cppId);
  if (iter == tasks.end()) return false;
  if (isAlwaysActive(cppId)) return false;
  if (iter->second.taskKind == "pure_compute") return true;
  return hasOnlyA44DirectFallbackReasons(iter->second.serialReasons);
}

static bool mtTaskCanEnterPureBatch(const std::map<int, MtTaskInfo>& tasks, int cppId) {
  auto iter = tasks.find(cppId);
  if (iter == tasks.end()) return false;
  if (iter->second.taskKind != "pure_compute") return false;
  if (isAlwaysActive(cppId)) return false;
  return true;
}

static bool mtCanCutEdge(const std::map<int, MtTaskInfo>& tasks, int fromCppId, int toCppId) {
  if (fromCppId == toCppId) return false;
  auto from = tasks.find(fromCppId);
  auto to = tasks.find(toCppId);
  if (from == tasks.end() || to == tasks.end()) return false;
  if (from->second.taskKind != "pure_compute" || to->second.taskKind != "pure_compute") return false;
  if (!from->second.serialReasons.empty() || !to->second.serialReasons.empty()) return false;
  if (!to->second.repcutSelected) return false;
  if (isAlwaysActive(fromCppId) || isAlwaysActive(toCppId)) return false;
  if (fromCppId / ACTIVE_WIDTH != toCppId / ACTIVE_WIDTH) return false;
  return mtTasksHaveDirectedEdge(cppId2Super[fromCppId], cppId2Super[toCppId]);
}

static bool mtTaskHasSameActiveWordHazard(const std::map<int, MtTaskInfo>& tasks, int cppId, bool allowCuts) {
  if (!mtTaskCanEnterPureBatch(tasks, cppId)) return false;
  int wordBegin = (cppId / ACTIVE_WIDTH) * ACTIVE_WIDTH;
  int wordEnd = std::min(superId, wordBegin + ACTIVE_WIDTH);
  for (int otherCppId = wordBegin; otherCppId < wordEnd; otherCppId ++) {
    if (otherCppId == cppId) continue;
    if (!mtTaskCanEnterPureBatch(tasks, otherCppId)) continue;
    if (!mtTasksHaveEdge(cppId2Super[cppId], cppId2Super[otherCppId])) continue;
    bool cutForward = allowCuts && mtCanCutEdge(tasks, cppId, otherCppId);
    bool cutBackward = allowCuts && mtCanCutEdge(tasks, otherCppId, cppId);
    if (!cutForward && !cutBackward) return true;
  }
  return false;
}

static bool mtTaskCanJoinPureBatchWithCuts(const std::map<int, MtTaskInfo>& tasks,
                                           const std::vector<int>& batch,
                                           int cppId,
                                           bool allowCuts,
                                           std::vector<MtRepCutEdge>* cutEdges) {
  if (!mtTaskCanEnterPureBatch(tasks, cppId)) return false;
  for (int existingCppId : batch) {
    if (!mtTasksHaveEdge(cppId2Super[existingCppId], cppId2Super[cppId])) continue;
    bool cutForward = allowCuts && mtCanCutEdge(tasks, existingCppId, cppId);
    bool cutBackward = allowCuts && mtCanCutEdge(tasks, cppId, existingCppId);
    if (!cutForward && !cutBackward) return false;
    if (cutEdges) {
      if (cutForward) cutEdges->push_back({existingCppId, cppId, "pure_successor_selected"});
      if (cutBackward) cutEdges->push_back({cppId, existingCppId, "pure_successor_selected"});
    }
  }
  return true;
}

static int mtTaskEstimatedCost(const std::map<int, MtTaskInfo>& tasks, int cppId) {
  auto iter = tasks.find(cppId);
  if (iter == tasks.end()) return 1;
  if (iter->second.hasCandidateCost && iter->second.candidateCost > 0) return iter->second.candidateCost;
  return 1;
}

static int mtBatchEstimatedCost(const std::map<int, MtTaskInfo>& tasks, int beginCppId, int endCppId) {
  int cost = 0;
  for (int cppId = beginCppId; cppId < endCppId; cppId ++) {
    cost += mtTaskEstimatedCost(tasks, cppId);
  }
  return cost;
}

static int mtBatchMemberNodeCost(int beginCppId, int endCppId) {
  int cost = 0;
  for (int cppId = beginCppId; cppId < endCppId; cppId ++) {
    auto iter = cppId2Super.find(cppId);
    if (iter != cppId2Super.end() && iter->second) cost += static_cast<int>(iter->second->member.size());
  }
  return cost;
}

static bool mtActiveWordIsWhole(int beginCppId) {
  for (int j = 0; j < ACTIVE_WIDTH && beginCppId + j < superId; j ++) {
    if (isAlwaysActive(beginCppId + j)) return false;
  }
  return true;
}

static int mtPureBatchShardCount() {
  return (superId + MT_PURE_BATCH_SHARD_SIZE - 1) / MT_PURE_BATCH_SHARD_SIZE;
}

static void mtAddCoarseBlocker(MtCoarseRegion& region, const std::string& blocker) {
  if (std::find(region.blockers.begin(), region.blockers.end(), blocker) == region.blockers.end()) {
    region.blockers.push_back(blocker);
  }
}

static std::unordered_map<uint64_t, bool> mtDependencyEdgeCache;
static std::unordered_map<uint64_t, bool> mtActiveEdgeCache;

static uint64_t mtEdgeCacheKey(int fromCppId, int toCppId) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(fromCppId)) << 32) |
         static_cast<uint32_t>(toCppId);
}

static bool mtTaskHasActiveEdgeToUncached(int fromCppId, int toCppId) {
  auto iter = cppId2Super.find(fromCppId);
  if (iter == cppId2Super.end() || !iter->second) return false;
  for (Node* member : iter->second->member) {
    if (member && member->nextActiveId.find(toCppId) != member->nextActiveId.end()) return true;
  }
  return false;
}

static bool mtTaskHasDependencyEdgeToUncached(int fromCppId, int toCppId) {
  auto from = cppId2Super.find(fromCppId);
  auto to = cppId2Super.find(toCppId);
  if (from == cppId2Super.end() || to == cppId2Super.end() || !from->second || !to->second) return false;
  if (hasCppId(from->second->next, toCppId) || hasCppId(from->second->depNext, toCppId) ||
      hasCppId(to->second->prev, fromCppId) || hasCppId(to->second->depPrev, fromCppId)) {
    return true;
  }
  return false;
}


static bool mtTaskHasActiveEdgeTo(int fromCppId, int toCppId) {
  if (!globalConfig.MtContextCache) return mtTaskHasActiveEdgeToUncached(fromCppId, toCppId);
  uint64_t key = mtEdgeCacheKey(fromCppId, toCppId);
  auto iter = mtActiveEdgeCache.find(key);
  if (iter != mtActiveEdgeCache.end()) return iter->second;
  bool value = mtTaskHasActiveEdgeToUncached(fromCppId, toCppId);
  mtActiveEdgeCache.emplace(key, value);
  return value;
}
// Track 2 Week 7: check whether fromCppId's SuperNode has a needActivate edge to toCppId.
// Unlike nextActiveId (which includes always-active), nextNeedActivate tracks the
// conditional activation edges gated by output change. Used by the Sarkar probe.
static bool mtTaskHasNeedActivateEdgeTo(int fromCppId, int toCppId) {
  auto iter = cppId2Super.find(fromCppId);
  if (iter == cppId2Super.end() || !iter->second) return false;
  for (Node* member : iter->second->member) {
    if (member && member->nextNeedActivate.find(toCppId) != member->nextNeedActivate.end()) return true;
  }
  return false;
}

static bool mtTaskHasDependencyEdgeTo(int fromCppId, int toCppId) {
  if (!globalConfig.MtContextCache) return mtTaskHasDependencyEdgeToUncached(fromCppId, toCppId);
  uint64_t key = mtEdgeCacheKey(fromCppId, toCppId);
  auto iter = mtDependencyEdgeCache.find(key);
  if (iter != mtDependencyEdgeCache.end()) return iter->second;
  bool value = mtTaskHasDependencyEdgeToUncached(fromCppId, toCppId);
  mtDependencyEdgeCache.emplace(key, value);
  return value;
}

static bool mtTaskHasOrderingEdgeTo(int fromCppId, int toCppId) {
  return mtTaskHasDependencyEdgeTo(fromCppId, toCppId) || mtTaskHasActiveEdgeTo(fromCppId, toCppId);
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

static std::vector<MtDenseMTask> mtBuildDenseMTasksCpContraction(const MtDenseSchedule& schedule,
                                                                 int threadCount) {
  // Level-bucketed sibling contraction (Verilator-inspired, adapted for gsim's diamond-mesh
  // dependency graph where pure edge contraction stalls: v237 edge-only contraction hit
  // 3.23M cycle rejections vs 27.8k merges). Each SCC gets a critical-path LEVEL = longest
  // dependency depth from a source. SCCs at the same level are mutually independent (no edge
  // between them), so grouping same-level SCCs is ALWAYS cycle-free (sibling contraction).
  // Within a level, non-worker0 SCCs are bucketed across threads balancing member-node cost
  // (LPT); worker0-only SCCs form their own separate MTask (never mixed with parallel SCCs,
  // preserving the v235 worker0 boundary). One MTask per (level, bucket). MTask ids follow
  // (level, bucket) order, and every SCC edge goes to a strictly greater level, so ids are
  // topologically monotone (succ>from) as the runtime protocol / transitive reduction require.
  const int nSccs = static_cast<int>(schedule.sccs.size());
  std::vector<MtDenseMTask> mtasks;
  if (nSccs == 0) return mtasks;
  if (threadCount < 1) threadCount = 1;
  auto sccCost = [&](int s) -> int { return std::max(1, schedule.sccs[(size_t)s].memberNodeCost); };

  // 1. Critical-path level per SCC. schedule.topoSccOrder is a valid topological order, so a
  //    single forward relaxation computes the longest-path depth.
  std::vector<int> level((size_t)nSccs, 0);
  int maxLevel = 0;
  for (int s : schedule.topoSccOrder) {
    int lv = level[(size_t)s];
    if (lv > maxLevel) maxLevel = lv;
    for (int t : schedule.sccs[(size_t)s].succSccs) {
      if (t < 0 || t >= nSccs) continue;
      if (level[(size_t)t] < lv + 1) { level[(size_t)t] = lv + 1; if (lv + 1 > maxLevel) maxLevel = lv + 1; }
    }
  }
  const int numLevels = maxLevel + 1;

  // 2. Group SCC ids by level, preserving topo order within a level.
  std::vector<std::vector<int>> levelSccs((size_t)numLevels);
  for (int s : schedule.topoSccOrder) levelSccs[(size_t)level[(size_t)s]].push_back(s);

  // 3. Per level: emit worker0-only SCCs as one dedicated MTask, and bucket the parallel
  //    SCCs into <=threadCount MTasks balancing member cost. Ids assigned in emission order.
  std::vector<int> sccToMTask((size_t)nSccs, -1);
  auto emitBucket = [&](const std::vector<int>& members, bool isW0) {
    if (members.empty()) return;
    int mi = static_cast<int>(mtasks.size());
    mtasks.emplace_back();
    MtDenseMTask& mt = mtasks.back();
    mt.workerZeroOnly = isW0;
    std::vector<int> sorted = members;
    std::sort(sorted.begin(), sorted.end()); // topo (ascending scc id) for deterministic emission
    for (int s : sorted) {
      mt.sccIds.push_back(s);
      mt.staticCost += schedule.sccs[(size_t)s].staticCost;
      mt.schedCost += sccCost(s);
      mt.taskCount += static_cast<int>(schedule.sccs[(size_t)s].cppIds.size());
      sccToMTask[(size_t)s] = mi;
    }
  };
  for (int lv = 0; lv < numLevels; lv ++) {
    std::vector<int> normal, w0;
    for (int s : levelSccs[(size_t)lv]) {
      if (schedule.sccs[(size_t)s].workerZeroOnly) w0.push_back(s); else normal.push_back(s);
    }
    if (!w0.empty()) emitBucket(w0, true); // worker0-only: its own MTask (no contamination)
    // LPT bucketing of parallel SCCs by descending cost.
    std::sort(normal.begin(), normal.end(), [&](int a, int b) { return sccCost(a) != sccCost(b) ? sccCost(a) > sccCost(b) : a < b; });
    std::vector<std::vector<int>> buckets((size_t)threadCount);
    std::vector<long long> bucketLoad((size_t)threadCount, 0);
    for (int s : normal) {
      int best = 0;
      for (int t = 1; t < threadCount; t ++) if (bucketLoad[(size_t)t] < bucketLoad[(size_t)best]) best = t;
      buckets[(size_t)best].push_back(s); bucketLoad[(size_t)best] += sccCost(s);
    }
    for (int t = 0; t < threadCount; t ++) emitBucket(buckets[(size_t)t], false);
  }

  // 3b. v240 phase-2: band/edge contraction on the level-bucket MTask graph. Level-bucketing
  //     (phase 1) gives depth==numLevels (~333) with many cross-level cross-thread deps. To reach
  //     Verilator's depth ~15 shape, merge each MTask with a successor MTask when they are within
  //     the same worker AND merging creates no cycle, using union-find + bounded reachability on the
  //     (small, ~few-thousand) MTask graph. Gated by GSIM_MT_DENSE_BAND_CONTRACT (levels per band);
  //     0/unset disables (pure level-bucket). Cycle checks are cheap here vs v237's 45k-node graph.
  int bandContract = 0;
  { const char* env = std::getenv("GSIM_MT_DENSE_BAND_CONTRACT"); if (env && env[0]) bandContract = std::atoi(env); }
  if (bandContract > 0 && static_cast<int>(mtasks.size()) > threadCount) {
    const int nm = static_cast<int>(mtasks.size());
    // Build the level-bucket MTask DAG (adjacency) for contraction.
    std::vector<int> mtLevel((size_t)nm, 0);
    for (int mi = 0; mi < nm; mi ++) {
      int lv = 0; for (int s : mtasks[(size_t)mi].sccIds) lv = std::max(lv, level[(size_t)s]); mtLevel[(size_t)mi] = lv;
    }
    std::vector<int> sccToMT((size_t)nSccs, -1);
    for (int mi = 0; mi < nm; mi ++) for (int s : mtasks[(size_t)mi].sccIds) sccToMT[(size_t)s] = mi;
    std::vector<std::set<int>> bSucc((size_t)nm), bPred((size_t)nm);
    for (int fromScc = 0; fromScc < nSccs; fromScc ++) {
      int fm = sccToMT[(size_t)fromScc]; if (fm < 0) continue;
      for (int toScc : schedule.sccs[(size_t)fromScc].succSccs) {
        int tm = (toScc >= 0 && toScc < nSccs) ? sccToMT[(size_t)toScc] : -1;
        if (tm < 0 || tm == fm) continue;
        bSucc[(size_t)fm].insert(tm); bPred[(size_t)tm].insert(fm);
      }
    }
    std::vector<int> uf((size_t)nm); for (int i = 0; i < nm; i ++) uf[(size_t)i] = i;
    std::function<int(int)> find = [&](int x){ while (uf[(size_t)x]!=x){ uf[(size_t)x]=uf[(size_t)uf[(size_t)x]]; x=uf[(size_t)x]; } return x; };
    std::vector<long long> gcost((size_t)nm, 0); std::vector<bool> gw0((size_t)nm, false); std::vector<int> gbase((size_t)nm, 0);
    for (int mi = 0; mi < nm; mi ++) { gcost[(size_t)mi] = mtasks[(size_t)mi].schedCost; gw0[(size_t)mi] = mtasks[(size_t)mi].workerZeroOnly; gbase[(size_t)mi] = mtLevel[(size_t)mi]; }
    long long total = 0; for (int mi = 0; mi < nm; mi ++) total += gcost[(size_t)mi];
    const long long capCost = std::max<long long>(1, (total / std::max(1, 50 * threadCount)) * 3);
    auto reachB = [&](int from, int to, int budget) -> bool {
      if (from == to) return true;
      std::vector<int> st; st.push_back(from); std::set<int> seen; seen.insert(from); int steps = 0;
      while (!st.empty()) { int g = st.back(); st.pop_back(); if (++steps > budget) return true;
        for (int s : bSucc[(size_t)g]) { int rs = find(s); if (rs == to) return true; if (seen.insert(rs).second) st.push_back(rs); } }
      return false;
    };
    auto mergeB = [&](int a, int b){ a=find(a); b=find(b); if(a==b) return; uf[(size_t)b]=a; gcost[(size_t)a]+=gcost[(size_t)b]; gw0[(size_t)a]=gw0[(size_t)a]||gw0[(size_t)b];
      for (int s : bSucc[(size_t)b]){ int rs=find(s); if(rs!=a){ bSucc[(size_t)a].insert(rs); bPred[(size_t)rs].insert(a);} }
      for (int p : bPred[(size_t)b]){ int rp=find(p); if(rp!=a){ bPred[(size_t)a].insert(rp); bSucc[(size_t)rp].insert(a);} }
      bSucc[(size_t)a].erase(a); bPred[(size_t)a].erase(a); bSucc[(size_t)a].erase(b); bPred[(size_t)a].erase(b); };
    // Contract a->b (b a successor of a) when within the band window, same w0, cost ok, cycle-safe.
    bool changed = true; int mergeCount = 0; const int cycBudget = 4000;
    while (changed) { changed = false;
      for (int mi = 0; mi < nm; mi ++) { int a = find(mi); if (a != mi) continue;
        int chosen = -1;
        for (int s : bSucc[(size_t)a]) { int rs = find(s); if (rs == a) continue;
          if (gw0[(size_t)a] != gw0[(size_t)rs]) continue;
          if (gbase[(size_t)rs] - gbase[(size_t)a] > bandContract) continue; // band window
          if (gcost[(size_t)a] + gcost[(size_t)rs] > capCost) continue;
          bool cyc = false; for (int s2 : bSucc[(size_t)a]) { int rs2 = find(s2); if (rs2 == rs || rs2 == a) continue; if (reachB(rs2, rs, cycBudget)) { cyc = true; break; } }
          if (cyc) continue; chosen = rs; break; }
        if (chosen >= 0) { mergeB(a, chosen); mergeCount ++; changed = true; }
      }
    }
    // Rebuild mtasks from union-find groups, in topological (Kahn) order for monotone ids.
    std::map<int,int> rootTmp; for (int mi = 0; mi < nm; mi ++) { int r = find(mi); if (!rootTmp.count(r)) rootTmp[r] = static_cast<int>(rootTmp.size()); }
    int R = static_cast<int>(rootTmp.size());
    std::vector<std::set<int>> rSucc((size_t)R), rPred((size_t)R);
    for (int mi = 0; mi < nm; mi ++) { int ra = rootTmp[find(mi)]; for (int s : bSucc[(size_t)find(mi)]) { int rs = rootTmp[find(s)]; if (rs != ra) { rSucc[(size_t)ra].insert(rs); rPred[(size_t)rs].insert(ra); } } }
    std::vector<int> rIndeg((size_t)R, 0); for (int i = 0; i < R; i ++) rIndeg[(size_t)i] = static_cast<int>(rPred[(size_t)i].size());
    std::vector<int> minBase((size_t)R, INT32_MAX); for (int mi = 0; mi < nm; mi ++) { int ri = rootTmp[find(mi)]; minBase[(size_t)ri] = std::min(minBase[(size_t)ri], gbase[(size_t)mi]); }
    auto cmpB = [&](int a, int b){ if (minBase[(size_t)a] != minBase[(size_t)b]) return minBase[(size_t)a] > minBase[(size_t)b]; return a > b; };
    std::priority_queue<int, std::vector<int>, decltype(cmpB)> rq(cmpB);
    for (int i = 0; i < R; i ++) if (rIndeg[(size_t)i] == 0) rq.push(i);
    std::vector<int> rootOrder((size_t)R, -1); int nextR = 0;
    while (!rq.empty()) { int u = rq.top(); rq.pop(); rootOrder[(size_t)u] = nextR ++; for (int v : rSucc[(size_t)u]) if (-- rIndeg[(size_t)v] == 0) rq.push(v); }
    Assert(nextR == R, "dense band contraction produced a cyclic MTask graph (%d of %d)", nextR, R);
    std::vector<MtDenseMTask> newMtasks((size_t)R);
    for (int mi = 0; mi < nm; mi ++) {
      int ri = rootOrder[(size_t)rootTmp[find(mi)]]; MtDenseMTask& nt = newMtasks[(size_t)ri]; const MtDenseMTask& ot = mtasks[(size_t)mi];
      for (int s : ot.sccIds) nt.sccIds.push_back(s);
      nt.staticCost += ot.staticCost; nt.schedCost += ot.schedCost; nt.taskCount += ot.taskCount; nt.workerZeroOnly = nt.workerZeroOnly || ot.workerZeroOnly;
    }
    for (int ri = 0; ri < R; ri ++) std::sort(newMtasks[(size_t)ri].sccIds.begin(), newMtasks[(size_t)ri].sccIds.end());
    fprintf(stderr, "[mt-dense-band] band=%d level-bucket-mtasks=%d -> contracted=%d merges=%d\n", bandContract, nm, R, mergeCount);
    mtasks.swap(newMtasks);
    // Rebuild sccToMTask for the new mtasks (used by the DAG build below).
    std::fill(sccToMTask.begin(), sccToMTask.end(), -1);
    for (int mi = 0; mi < static_cast<int>(mtasks.size()); mi ++) for (int s : mtasks[(size_t)mi].sccIds) sccToMTask[(size_t)s] = mi;
  }

  // 4. Build the MTask dependency DAG from SCC edges (ids are topologically monotone).
  const int nMTasks = static_cast<int>(mtasks.size());
  std::vector<std::set<int>> predSets((size_t)nMTasks), succSets((size_t)nMTasks);
  for (int fromScc = 0; fromScc < nSccs; fromScc ++) {
    int fromMTask = sccToMTask[(size_t)fromScc];
    if (fromMTask < 0) continue;
    for (int toScc : schedule.sccs[(size_t)fromScc].succSccs) {
      int toMTask = (toScc >= 0 && toScc < nSccs) ? sccToMTask[(size_t)toScc] : -1;
      if (toMTask < 0 || toMTask == fromMTask) continue;
      Assert(toMTask > fromMTask, "dense level-bucket contraction backward MTask edge %d->%d (lv %d->%d)",
             fromMTask, toMTask, level[(size_t)fromScc], level[(size_t)toScc]);
      succSets[(size_t)fromMTask].insert(toMTask);
      predSets[(size_t)toMTask].insert(fromMTask);
    }
  }
  for (int mi = 0; mi < nMTasks; mi ++) {
    mtasks[(size_t)mi].predMTasks.assign(predSets[(size_t)mi].begin(), predSets[(size_t)mi].end());
    mtasks[(size_t)mi].succMTasks.assign(succSets[(size_t)mi].begin(), succSets[(size_t)mi].end());
  }
  fprintf(stderr, "[mt-dense-levelbucket] levels=%d mtasks=%d threads=%d\n", numLevels, nMTasks, threadCount);
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

static int mtReduceDenseRuntimeSuccsTransitive(std::vector<std::vector<int>>& runtimeSuccs,
                                               const std::vector<int>& assignment) {
  const int nMTasks = static_cast<int>(runtimeSuccs.size());
  if (nMTasks <= 1) return 0;
  std::vector<std::vector<int>> effectiveSuccs = runtimeSuccs;
  int maxWorker = -1;
  for (int worker : assignment) maxWorker = std::max(maxWorker, worker);
  if (maxWorker >= 0) {
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
  // v236: prefer the real dense scheduling cost when present; fall back to staticCost.
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
          if (predWorker >= 0 && predWorker != worker) predEnd += (costOf(mtasks[(size_t)pred]) * 30) / 100;
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

// v243: list-scheduler that returns BOTH the worker assignment AND the schedule order (the
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
  while (!ready.empty()) {
    int bestReadyIndex = -1, bestMTask = -1, bestWorker = 0;
    long long bestTime = std::numeric_limits<long long>::max();
    for (int ri = 0; ri < static_cast<int>(ready.size()); ri ++) {
      int mtaskId = ready[(size_t)ri];
      const MtDenseMTask& mtask = mtasks[(size_t)mtaskId];
      int workerLimit = mtask.workerZeroOnly ? 1 : threadCount;
      for (int worker = 0; worker < workerLimit; worker ++) {
        long long timeBegin = busyUntil[(size_t)worker];
        for (int pred : mtask.predMTasks) {
          if (pred < 0 || pred >= n) continue;
          long long predEnd = completion[(size_t)pred];
          int predWorker = outAssign[(size_t)pred];
          if (predWorker >= 0 && predWorker != worker) predEnd += (long long)(costOf(mtasks[(size_t)pred])) * 30 / 100;
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

  // Phase 1: Add dependency edges only.
  for (int cppId = 0; cppId < superId; cppId ++) {
    auto superIter = cppId2Super.find(cppId);
    if (superIter == cppId2Super.end() || !superIter->second) continue;
    SuperNode* super = superIter->second;
    mtDenseAddSuperEdges(schedule, succSets, predSets, edgeKinds, cppId, super->next, "dependency");
    mtDenseAddSuperEdges(schedule, succSets, predSets, edgeKinds, cppId, super->depNext, "dependency");
  }

  // v198: When GSIM_MT_DENSE_FORWARD_ACTIVATION_ONLY=1, compute a dependency-only
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
      if (taskIter != tasks.end() && hasWorker0OnlyReason(taskIter->second.serialReasons)) {
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

  // v201b: Coarsen SCC DAG by merging chains (edges A->B where A has 1 succ
  // and B has 1 pred). This reduces layer depth and barrier count without
  // reducing parallelism. Inspired by Verilator's V3OrderParallel edge contraction.
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
  // Dense dependency executor: form MTasks and assign them to worker threads.
  // Default path: fixed 30-SCC topological chunking + round-robin assignment.
  // v236 (GSIM_MT_DENSE_CP_CONTRACTION): Verilator-style critical-path edge contraction.
  // v236 (GSIM_MT_DENSE_PACKTHREADS_ASSIGNMENT): DAG-aware list-scheduling assignment.
  {
    int threadCount = 8;
    const char* threadsEnv = std::getenv("GSIM_THREADS");
    if (threadsEnv != nullptr && threadsEnv[0] != '\0') threadCount = std::atoi(threadsEnv);
    if (threadCount < 1) threadCount = 1;

    if (mtUseDenseCpContraction()) {
      schedule.mtasks = mtBuildDenseMTasksCpContraction(schedule, threadCount);
    } else {
      schedule.mtasks = mtBuildDenseMTasks(schedule, mtUseDenseSplitWorker0MTasks());
    }
    // v243: renumber MTasks by list-schedule (earliest-start) order so the fixed-order runtime
    // executes each worker's MTasks in schedule order (Verilator static per-worker chain). Only
    // reorders ids; keeps topo-monotonicity. Also sets the assignment from the scheduler.
    bool schedOrder = false;
    { const char* e = std::getenv("GSIM_MT_DENSE_SCHED_ORDER"); schedOrder = e && e[0] && e[0] != '0'; }
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
    bool lptAssign = false;
    { const char* e = std::getenv("GSIM_MT_DENSE_LPT_ASSIGN"); lptAssign = e && e[0] && e[0] != '0'; }
    if (!lptAssign && !schedOrderAssign.empty() && static_cast<int>(schedOrderAssign.size()) == nMTasks) {
      // v243: schedule-order ids WITHOUT LPT -> use the list-scheduler's own (earliest-free)
      // assignment, co-designed with the order. With LPT on, we instead keep schedule-order ids
      // (reduced stalls) but override with cost-balanced LPT assignment below.
      for (int i = 0; i < nMTasks; i ++) schedule.mtaskThreadAssign[(size_t)i] = schedule.mtasks[(size_t)i].workerZeroOnly ? 0 : schedOrderAssign[(size_t)i];
    }
    if (lptAssign && nMTasks > 0) {
      // LPT (longest-processing-time) balancing by real dense work (schedCost=member cost, else
      // staticCost). Sorts non-worker0 MTasks by descending cost, greedily assigns each to the
      // least-loaded worker. worker0-only MTasks pinned to thread 0. Fixes the severe imbalance
      // seen with round-robin/PackThreads on coarse graphs where cost is concentrated.
      auto mtCost = [&](int i) { return schedule.mtasks[(size_t)i].schedCost > 0 ? schedule.mtasks[(size_t)i].schedCost : schedule.mtasks[(size_t)i].staticCost; };
      std::vector<int> order; order.reserve(nMTasks);
      for (int i = 0; i < nMTasks; i ++) if (!schedule.mtasks[(size_t)i].workerZeroOnly) order.push_back(i);
      std::sort(order.begin(), order.end(), [&](int a, int b){ int ca=mtCost(a), cb=mtCost(b); return ca != cb ? ca > cb : a < b; });
      std::vector<long long> loads((size_t)threadCount, 0);
      for (int i = 0; i < nMTasks; i ++) if (schedule.mtasks[(size_t)i].workerZeroOnly) { schedule.mtaskThreadAssign[(size_t)i] = 0; loads[0] += mtCost(i); }
      for (int i : order) {
        int best = 0; for (int t = 1; t < threadCount; t ++) if (loads[(size_t)t] < loads[(size_t)best]) best = t;
        schedule.mtaskThreadAssign[(size_t)i] = best; loads[(size_t)best] += mtCost(i);
      }
    } else if (!schedOrderAssign.empty() && static_cast<int>(schedOrderAssign.size()) == nMTasks) {
      // schedule-order assignment already installed above; nothing to do.
    } else if (mtUseDensePackThreadsAssignment() && nMTasks > 0) {
      std::vector<int> packAssign = mtBuildDensePackThreadsAssignment(schedule.mtasks, threadCount).first;
      bool ok = true;
      for (int i = 0; i < nMTasks; i ++) { if (packAssign[(size_t)i] < 0) { ok = false; break; } }
      if (ok) {
        for (int i = 0; i < nMTasks; i ++) schedule.mtaskThreadAssign[(size_t)i] = schedule.mtasks[(size_t)i].workerZeroOnly ? 0 : packAssign[(size_t)i];
      } else {
        for (int i = 0; i < nMTasks; i ++) schedule.mtaskThreadAssign[(size_t)i] = schedule.mtasks[(size_t)i].workerZeroOnly ? 0 : (i % threadCount);
      }
    } else {
      for (int i = 0; i < nMTasks; i ++) schedule.mtaskThreadAssign[(size_t)i] = schedule.mtasks[(size_t)i].workerZeroOnly ? 0 : (i % threadCount);
    }
    // v243: verify MTask ids are topologically monotone (every edge from<to) after any renumber.
    for (int mi = 0; mi < nMTasks; mi ++) {
      for (int s : schedule.mtasks[(size_t)mi].succMTasks) {
        Assert(s > mi, "dense MTask id order not topo-monotone: edge %d->%d", mi, s);
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

// Track 2 Week 2: report-only inside-component antichain grouping.
// Computes dependency-connected components, splits serial/hazard nodes into
// singleton groups, and covers each contiguous non-serial block with a minimum
// chain decomposition (Dilworth).  The resulting groups are stored only for
// the coarse-region report; region.mtasks is left unchanged so the runtime
// executor still sees the original ordering-edge components.
// Forward declaration for Week 3 DAG builder.
static bool mtBuildAntichainMTaskDAG(MtCoarseRegion& region);

static void mtComputeAntichainGroups(MtCoarseRegion& region, const std::map<int, MtTaskInfo>& tasks) {
  region.antichainProbeGroups.clear();
  region.antichainProbeMaxBlockWidth = 0;
  region.antichainProbeTotalGroups = 0;

  int n = region.endCppId - region.beginCppId;
  if (n <= 0) return;

  // 1. Union-find on dependency edges only.
  std::vector<int> parent(n);
  for (int i = 0; i < n; i++) parent[i] = i;
  std::function<int(int)> findRoot = [&](int value) -> int {
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

  for (int from = region.beginCppId; from < region.endCppId; from++) {
    for (int to = region.beginCppId; to < region.endCppId; to++) {
      if (from == to) continue;
      if (!mtTaskHasDependencyEdgeTo(from, to)) continue;
      unite(from - region.beginCppId, to - region.beginCppId);
    }
  }

  std::map<int, std::vector<int>> depComponentByRoot;
  for (int cppId = region.beginCppId; cppId < region.endCppId; cppId++) {
    int root = findRoot(cppId - region.beginCppId);
    depComponentByRoot[root].push_back(cppId);
  }

  // Build local layer index map.
  std::map<int, int> layerIndexByCppId;
  for (size_t layerIdx = 0; layerIdx < region.layers.size(); layerIdx++) {
    for (int cppId : region.layers[layerIdx].taskCppIds) {
      layerIndexByCppId[cppId] = static_cast<int>(layerIdx);
    }
  }

  auto addSingletonGroup = [&](int cppId, bool workerZeroOnly) {
    MtCoarseMTask group;
    auto layerIter = layerIndexByCppId.find(cppId);
    if (layerIter != layerIndexByCppId.end()) {
      while ((int)group.layerTaskCppIds.size() <= layerIter->second) {
        group.layerTaskCppIds.push_back(std::vector<int>());
      }
      group.layerTaskCppIds[layerIter->second].push_back(cppId);
    }
    group.taskCount = 1;
    group.staticCost = mtTaskEstimatedCost(tasks, cppId);
    group.workerZeroOnly = workerZeroOnly;
    auto superIter = cppId2Super.find(cppId);
    if (superIter != cppId2Super.end() && superIter->second) {
      group.memberNodeCost = static_cast<int>(superIter->second->member.size());
    }
    region.antichainProbeGroups.push_back(group);
  };

  // Minimum chain cover via Hopcroft-Karp on the transitive closure of a DAG.
  auto chainCover = [&](const std::vector<int>& block) {
    if (block.empty()) return;
    if (block.size() == 1) {
      addSingletonGroup(block[0], false);
      return;
    }
    std::vector<int> verts = block;
    std::sort(verts.begin(), verts.end());
    std::map<int, int> idx;
    for (size_t i = 0; i < verts.size(); i++) idx[verts[i]] = static_cast<int>(i);
    int m = static_cast<int>(verts.size());

    // Reachability bitset (forward edges only, by cppId order).
    std::vector<uint64_t> reach(m * ((m + 63) / 64), 0);
    int words = (m + 63) / 64;
    auto setBit = [&](int r, int c) {
      if (r == c) return;
      if (idx[verts[r]] < idx[verts[c]]) {
        reach[r * words + (c >> 6)] |= (1ULL << (c & 63));
      }
    };
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < m; j++) {
        if (i == j) continue;
        if (mtTaskHasDependencyEdgeTo(verts[i], verts[j]) ||
            mtTaskHasActiveEdgeTo(verts[i], verts[j])) {
          setBit(i, j);
        }
      }
    }

    // Transitive closure (Warshall via bitsets).
    for (int k = 0; k < m; k++) {
      for (int i = 0; i < m; i++) {
        if (reach[i * words + (k >> 6)] & (1ULL << (k & 63))) {
          for (int w = 0; w < words; w++) {
            reach[i * words + w] |= reach[k * words + w];
          }
        }
      }
    }

    // Build adjacency list from left U to right V.
    std::vector<std::vector<int>> adj(m);
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < m; j++) {
        if (i == j) continue;
        if (reach[i * words + (j >> 6)] & (1ULL << (j & 63))) {
          adj[i].push_back(j);
        }
      }
    }

    // Hopcroft-Karp.
    std::vector<int> pairU(m, -1), pairV(m, -1), dist(m);
    std::function<bool()> bfs = [&]() -> bool {
      std::deque<int> q;
      for (int u = 0; u < m; u++) {
        if (pairU[u] == -1) {
          dist[u] = 0;
          q.push_back(u);
        } else {
          dist[u] = -1;
        }
      }
      bool found = false;
      while (!q.empty()) {
        int u = q.front(); q.pop_front();
        for (int v : adj[u]) {
          int pu = pairV[v];
          if (pu != -1 && dist[pu] == -1) {
            dist[pu] = dist[u] + 1;
            q.push_back(pu);
          } else if (pu == -1) {
            found = true;
          }
        }
      }
      return found;
    };
    std::function<bool(int)> dfs = [&](int u) -> bool {
      for (int v : adj[u]) {
        int pu = pairV[v];
        if (pu == -1 || (dist[pu] == dist[u] + 1 && dfs(pu))) {
          pairU[u] = v;
          pairV[v] = u;
          return true;
        }
      }
      dist[u] = -1;
      return false;
    };

    int matching = 0;
    while (bfs()) {
      for (int u = 0; u < m; u++) {
        if (pairU[u] == -1 && dfs(u)) matching++;
      }
    }
    int width = m - matching;
    if (width > region.antichainProbeMaxBlockWidth) region.antichainProbeMaxBlockWidth = width;

    // Extract chains from matching: pairU[u] = v means u precedes v in a chain.
    std::map<int, int> nxt;
    std::set<int> hasPred;
    for (int u = 0; u < m; u++) {
      if (pairU[u] != -1) {
        nxt[u] = pairU[u];
        hasPred.insert(pairU[u]);
      }
    }
    std::vector<std::vector<int>> chains;
    for (int u = 0; u < m; u++) {
      if (hasPred.find(u) == hasPred.end()) {
        std::vector<int> chain;
        int cur = u;
        while (true) {
          chain.push_back(cur);
          auto it = nxt.find(cur);
          if (it == nxt.end()) break;
          cur = it->second;
        }
        chains.push_back(chain);
      }
    }
    // Safety: any unmatched node not in a chain becomes its own chain.
    std::set<int> covered;
    for (auto& chain : chains) {
      for (int x : chain) covered.insert(x);
    }
    for (int i = 0; i < m; i++) {
      if (covered.find(i) == covered.end()) {
        chains.push_back({i});
      }
    }

    for (auto& chain : chains) {
      MtCoarseMTask group;
      for (int localIdx : chain) {
        int cppId = verts[localIdx];
        auto layerIter = layerIndexByCppId.find(cppId);
        if (layerIter != layerIndexByCppId.end()) {
          while ((int)group.layerTaskCppIds.size() <= layerIter->second) {
            group.layerTaskCppIds.push_back(std::vector<int>());
          }
          group.layerTaskCppIds[layerIter->second].push_back(cppId);
        }
        group.taskCount++;
        group.staticCost += mtTaskEstimatedCost(tasks, cppId);
        auto superIter = cppId2Super.find(cppId);
        if (superIter != cppId2Super.end() && superIter->second) {
          group.memberNodeCost += static_cast<int>(superIter->second->member.size());
        }
      }
      region.antichainProbeGroups.push_back(group);
    }
  };

  for (auto& kv : depComponentByRoot) {
    std::vector<int>& comp = kv.second;
    std::sort(comp.begin(), comp.end());

    std::vector<std::vector<int>> blocks;
    std::vector<int> curBlock;
    for (int cppId : comp) {
      auto iter = tasks.find(cppId);
      bool isSerial = iter == tasks.end() || iter->second.taskKind != "pure_compute" || !iter->second.serialReasons.empty();
      if (isSerial) {
        if (!curBlock.empty()) {
          blocks.push_back(curBlock);
          curBlock.clear();
        }
        blocks.push_back({cppId});
      } else {
        curBlock.push_back(cppId);
      }
    }
    if (!curBlock.empty()) blocks.push_back(curBlock);

    for (auto& block : blocks) {
      if (block.size() == 1) {
        auto iter = tasks.find(block[0]);
        bool isSerial = iter == tasks.end() || iter->second.taskKind != "pure_compute" || !iter->second.serialReasons.empty();
        addSingletonGroup(block[0], isSerial);
      } else {
        chainCover(block);
      }
    }
  }
  bool dagAcyclic = mtBuildAntichainMTaskDAG(region);
  (void)dagAcyclic;  // Reported via JSON in Week 3; runtime not yet enabled.

  region.antichainProbeTotalGroups = static_cast<int>(region.antichainProbeGroups.size());

  // Week 3 gate: quotient DAG must be acyclic for antichain groups to be valid mtasks.
  // If cyclic, report but do not enable runtime.
  region.antichainProbeDagAcyclic = dagAcyclic;
}



// Track 2 Week 3: build the cross-mtask dependency DAG for antichainProbeGroups.
// Uses all ordering edges (dependency + active) between different groups.
// Returns true iff the quotient DAG is acyclic (required for these groups to be
// schedulable as atomic mtasks).
static bool mtBuildAntichainMTaskDAG(MtCoarseRegion& region) {
  std::map<int, int> cppIdToGroup;
  for (size_t g = 0; g < region.antichainProbeGroups.size(); g++) {
    for (const auto& layer : region.antichainProbeGroups[g].layerTaskCppIds) {
      for (int cppId : layer) cppIdToGroup[cppId] = static_cast<int>(g);
    }
  }

  int groupCount = static_cast<int>(region.antichainProbeGroups.size());
  std::set<std::pair<int, int>> seenEdges;
  for (int from = region.beginCppId; from < region.endCppId; from++) {
    for (int to = region.beginCppId; to < region.endCppId; to++) {
      if (from == to) continue;
      if (!mtTaskHasOrderingEdgeTo(from, to)) continue;
      auto fromIter = cppIdToGroup.find(from);
      auto toIter = cppIdToGroup.find(to);
      if (fromIter == cppIdToGroup.end() || toIter == cppIdToGroup.end()) continue;
      if (fromIter->second == toIter->second) continue;
      int fromGroup = fromIter->second;
      int toGroup = toIter->second;
      if (!seenEdges.insert({fromGroup, toGroup}).second) continue;
      region.antichainProbeGroups[fromGroup].succMTaskIndices.push_back(toGroup);
      region.antichainProbeGroups[toGroup].predMTaskIndices.push_back(fromGroup);
      region.antichainProbeGroups[toGroup].upstreamDepCount++;
    }
  }

  // Topological sort / cycle detection on the quotient DAG.
  std::vector<int> indegree(groupCount, 0);
  for (int g = 0; g < groupCount; g++) {
    for (int succ : region.antichainProbeGroups[g].succMTaskIndices) {
      indegree[succ]++;
    }
  }
  std::deque<int> q;
  for (int g = 0; g < groupCount; g++) {
    if (indegree[g] == 0) q.push_back(g);
  }
  int visited = 0;
  while (!q.empty()) {
    int u = q.front(); q.pop_front();
    visited++;
    for (int v : region.antichainProbeGroups[u].succMTaskIndices) {
      if (--indegree[v] == 0) q.push_back(v);
    }
  }
  return visited == groupCount;
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
  // Track 2 Week 2: report-only antichain probe.
  // Track 2 Week 4: when GSIM_MT_ANTICHAIN_RUNTIME=1, compute antichain groups
  // for selected runtime-eligible regions and use them as the real mtask set.
  static bool antichainRuntimeEnabled = []() {
    const char* env = std::getenv("GSIM_MT_ANTICHAIN_RUNTIME");
    return env && env[0] == '1';
  }();
  static bool probeEnabled = []() {
    const char* env = std::getenv("GSIM_MT_ANTICHAIN_PROBE");
    return env && env[0] == '1';
  }();
  static bool sarkarProbe = []() {
    const char* env = std::getenv("GSIM_MT_SARKAR_PROBE");
    return env && env[0] == '1';
  }();
  static bool sarkarContract = []() {
    const char* env = std::getenv("GSIM_MT_SARKAR_CONTRACT");
    return env && env[0] == '1';
  }();
  static bool antichainAllRegions = []() {
    const char* env = std::getenv("GSIM_MT_ANTICHAIN_ALL");
    return env && env[0] == '1';
  }();
  static int antichainMinUseful = []() {
    const char* env = std::getenv("GSIM_MT_ANTICHAIN_MIN_USEFUL");
    return env && env[0] != '\0' ? std::atoi(env) : 0;
  }();
  static bool antichainRegionEnvSpecified = []() {
    const char* env = std::getenv("GSIM_MT_ANTICHAIN_REGION");
    return env && env[0] != '\0';
  }();
  static std::pair<int, int> antichainSelectedRange = []() {
    const char* env = std::getenv("GSIM_MT_ANTICHAIN_REGION");
    if (env == nullptr || env[0] == '\0') return std::make_pair(-1, -1);
    char* end = nullptr;
    long begin = std::strtol(env, &end, 10);
    if (end == env || (*end != ':' && *end != '-')) return std::make_pair(-1, -1);
    char* end2 = nullptr;
    long finish = std::strtol(end + 1, &end2, 10);
    if (end2 == end + 1 || *end2 != '\0') return std::make_pair(-1, -1);
    return std::make_pair(static_cast<int>(begin), static_cast<int>(finish));
  }();
  static bool antichainLegacyOverride = []() {
    const char* env = std::getenv("GSIM_MT_ANTICHAIN_LEGACY");
    return env && env[0] == '1';
  }();
  static bool antichainLegacyDisabled = []() {
    const char* env = std::getenv("GSIM_MT_ANTICHAIN_LEGACY");
    return env && env[0] == '0';
  }();
  bool explicitRangeRegion = (antichainSelectedRange.first == region.beginCppId &&
                              antichainSelectedRange.second == region.endCppId);
  bool usefulRegion = antichainMinUseful > 0 && region.estimatedUsefulWork >= antichainMinUseful;
  bool selectorSpecified = antichainRegionEnvSpecified || antichainMinUseful > 0 || antichainAllRegions;
  bool legacyHotRegion = (region.beginCppId == 38872 && region.endCppId == 39056) &&
                         !antichainLegacyDisabled &&
                         (antichainLegacyOverride || !selectorSpecified);
  bool antichainSelectedRegion = legacyHotRegion || explicitRangeRegion || usefulRegion || antichainAllRegions;
  if (legacyHotRegion) region.antichainSelectionReason = "legacy_hot_region";
  else if (explicitRangeRegion) region.antichainSelectionReason = "explicit_region";
  else if (usefulRegion) region.antichainSelectionReason = "min_useful";
  else if (antichainAllRegions) region.antichainSelectionReason = "all";
  if (antichainSelectedRegion && (antichainRuntimeEnabled || probeEnabled || sarkarProbe || sarkarContract)) {
    mtComputeAntichainGroups(region, tasks);
  }
  // Track 2 Week 7: cost-based Sarkar edge-contraction probe (report-only).
  // Phase 1 (fix-hazards): contract need-only edges (no structural dep).
  // Phase 2 (cost-based): contract any edge where min(staticCost_u, staticCost_v) < sync_cost.
  // sync_cost from GSIM_MT_SARKAR_COST env (default 100).
  // USAGE: GSIM_MT_SARKAR_COST=N GSIM_MT_SARKAR_PROBE=1 ./build/gsim/gsim ...
  if (sarkarProbe && region.antichainProbeGroups.size() >= 2) {
      int groupCount = static_cast<int>(region.antichainProbeGroups.size());
      int syncCost = 100; { const char* s = getenv("GSIM_MT_SARKAR_COST"); if (s && s[0]) syncCost = atoi(s); }
      // Group cppIds and costs.
      std::map<int, int> cppIdToGroup;
      std::vector<int> groupCosts(groupCount, 0);
      for (int g = 0; g < groupCount; g++) {
        groupCosts[g] = region.antichainProbeGroups[g].staticCost;
        for (const auto& layer : region.antichainProbeGroups[g].layerTaskCppIds) {
          for (int cppId : layer) cppIdToGroup[cppId] = g;
        }
      }
      std::vector<std::vector<int>> groupCppIds(groupCount);
      for (auto& kv : cppIdToGroup) groupCppIds[kv.second].push_back(kv.first);
      // Precompute cross-group edge types and static costs.
      // For each ordered pair (gi,gj): hasEdge, hasDep, hasNeed, minCost.
      std::vector<std::vector<int>> hasEdge(groupCount, std::vector<int>(groupCount, 0));
      std::vector<std::vector<int>> edgeClass(groupCount, std::vector<int>(groupCount, 0)); // 0=need,1=other,2=dep
      for (int gi = 0; gi < groupCount; gi++) {
        for (int gj = 0; gj < groupCount; gj++) {
          if (gi == gj) continue;
          bool anyDep = false, anyNeed = false, anyOther = false;
          for (int from : groupCppIds[gi]) {
            if (anyDep && anyNeed && anyOther) break;
            for (int to : groupCppIds[gj]) {
              if (!anyDep && mtTaskHasDependencyEdgeTo(from, to)) { anyDep = true; break; }
            }
          }
          if (!anyDep) {
            for (int from : groupCppIds[gi]) {
              if (anyNeed && anyOther) break;
              for (int to : groupCppIds[gj]) {
                if (!anyNeed && mtTaskHasNeedActivateEdgeTo(from, to)) anyNeed = true;
                if (!anyOther && mtTaskHasActiveEdgeTo(from, to) && !mtTaskHasNeedActivateEdgeTo(from, to)) anyOther = true;
              }
            }
          }
          if (anyNeed || anyOther || anyDep) {
            hasEdge[gi][gj] = 1;
            edgeClass[gi][gj] = anyDep ? 2 : (anyNeed ? 0 : 1);
          }
        }
      }
      // ---------- Phase 1: fix-hazards (need-only) ----------
      std::vector<int> p1Parent(groupCount);
      for (int g = 0; g < groupCount; g++) p1Parent[g] = g;
      auto find = [&](auto& p, int x) -> int {
        int r = x;
        while (p[r] != r) r = p[r];
        while (p[x] != x) { int n = p[x]; p[x] = r; x = n; }
        return r;
      };
      auto unite = [&](auto& p, int a, int b) { p[find(p, b)] = find(p, a); };
      int directedEdges = 0, depEdges = 0, needEdges = 0, otherEdges = 0;
      for (int gi = 0; gi < groupCount; gi++) {
        for (int gj = 0; gj < groupCount; gj++) {
          if (!hasEdge[gi][gj]) continue;
          directedEdges++;
          if (edgeClass[gi][gj] == 2) { depEdges++; continue; }
          if (edgeClass[gi][gj] == 0) { needEdges++; unite(p1Parent, gi, gj); continue; }
          if (edgeClass[gi][gj] == 1) { otherEdges++; continue; }
        }
      }
      std::set<int> p1Remaining;
      for (int g = 0; g < groupCount; g++) p1Remaining.insert(find(p1Parent, g));
      int p1Contracted = groupCount - static_cast<int>(p1Remaining.size());
      // ---------- Phase 2: cost-based contraction ----------
      int c2Edges = 0, c2BelowAll = 0;
      // Pre-pass: count edges below threshold over p1 unions.
      for (int gi = 0; gi < groupCount; gi++) {
        for (int gj = 0; gj < groupCount; gj++) {
          if (!hasEdge[gi][gj]) continue;
          if (find(p1Parent, gi) == find(p1Parent, gj)) continue;
          c2Edges++;
          if (std::min(groupCosts[gi], groupCosts[gj]) < syncCost) c2BelowAll++;
        }
      }
      int c2Contracted = 0;
      std::vector<int> p2Parent = p1Parent;
      for (int gi = 0; gi < groupCount; gi++) {
        for (int gj = 0; gj < groupCount; gj++) {
          if (!hasEdge[gi][gj]) continue;
          if (find(p2Parent, gi) == find(p2Parent, gj)) continue;
          if (std::min(groupCosts[gi], groupCosts[gj]) >= syncCost) continue;
          c2Contracted++;
          unite(p2Parent, gi, gj);
        }
      }
      std::set<int> p2Remaining;
      for (int g = 0; g < groupCount; g++) p2Remaining.insert(find(p2Parent, g));
      int p2Contracted = groupCount - static_cast<int>(p2Remaining.size()) - p1Contracted;
      // Report.
      fprintf(stderr, "[sarkar-probe] region [%d,%d) groups=%d directed=%d dep=%d need=%d other=%d "
              "fix-hazards_contracted=%d p1_remaining=%d "
              "cost=%d above=%d below=%d union=%d total=%d final=%d",
              region.beginCppId, region.endCppId,
              groupCount, directedEdges, depEdges, needEdges, otherEdges,
              p1Contracted, groupCount - p1Contracted,
              syncCost, c2Edges - c2BelowAll, c2BelowAll, c2Contracted,
              p1Contracted + p2Contracted, groupCount - p1Contracted - p2Contracted);
      // Critical path estimate: longest chain in the surviving quotient DAG.
      if (groupCount - p1Contracted - p2Contracted > 1) {
        int c = 0;
        for (int gi = 0; gi < groupCount; gi++) {
          for (int gj = 0; gj < groupCount; gj++) {
            if (!hasEdge[gi][gj]) continue;
            if (find(p2Parent, gi) != find(p2Parent, gj)) c++;
          }
        }
        fprintf(stderr, " remaining_edges=%d", c);
      }
      fprintf(stderr, "\n");
    }
  // Track 2 Week 7: actual Sarkar contraction (apply, not probe).
  // Gated by GSIM_MT_SARKAR_CONTRACT=1; uses GSIM_MT_SARKAR_COST for threshold.
  {
    const char* contractEnv = std::getenv("GSIM_MT_SARKAR_CONTRACT");
    bool doContract = contractEnv && contractEnv[0] == '1';
    if (doContract && region.antichainProbeGroups.size() >= 2) {
      int syncCost = 100;
      { const char* e = std::getenv("GSIM_MT_SARKAR_COST"); if (e && e[0]) syncCost = atoi(e); }
      if (syncCost <= 0) syncCost = 100;
      int G = static_cast<int>(region.antichainProbeGroups.size());
      // Build group cost vector.
      std::vector<int> groupCosts(G, 0);
      for (int g = 0; g < G; g++) groupCosts[g] = region.antichainProbeGroups[g].staticCost;
      // Build cross-group edge matrix (same as probe Phase 2 pre-pass).
      std::map<int, int> cppIdToGroup;
      for (int g = 0; g < G; g++) {
        for (const auto& layer : region.antichainProbeGroups[g].layerTaskCppIds) {
          for (int cppId : layer) cppIdToGroup[cppId] = g;
        }
      }
      std::vector<std::vector<int>> groupCppIds(G);
      for (auto& kv : cppIdToGroup) groupCppIds[kv.second].push_back(kv.first);
      // Union-find over cost-below-threshold edges.
      std::vector<int> parent(G);
      for (int g = 0; g < G; g++) parent[g] = g;
      auto find = [&](auto& p, int x) -> int {
        int r = x; while (p[r] != r) r = p[r];
        while (p[x] != x) { int n = p[x]; p[x] = r; x = n; }
        return r;
      };
      auto unite = [&](auto& p, int a, int b) { p[find(p, b)] = find(p, a); };
      for (int gi = 0; gi < G; gi++) {
        for (int gj = 0; gj < G; gj++) {
          if (gi == gj) continue;
          if (find(parent, gi) == find(parent, gj)) continue;
          // Check if there's any ordering edge between the two groups.
          bool hasEdge = false;
          for (int from : groupCppIds[gi]) {
            if (hasEdge) break;
            for (int to : groupCppIds[gj]) {
              if (mtTaskHasOrderingEdgeTo(from, to)) { hasEdge = true; break; }
            }
          }
          if (!hasEdge) continue;
          int benefit = std::min(groupCosts[gi], groupCosts[gj]);
          if (benefit < syncCost) unite(parent, gi, gj);
        }
      }
      // Build merged groups.
      std::map<int, std::vector<int>> rootToGroups;
      for (int g = 0; g < G; g++) rootToGroups[find(parent, g)].push_back(g);
      if (static_cast<int>(rootToGroups.size()) == G) {
        fprintf(stderr, "[sarkar-contract] region [%d,%d) no contraction candidates\n", region.beginCppId, region.endCppId);
      } else {
        std::vector<MtCoarseMTask> newGroups;
        for (auto& kv : rootToGroups) {
          MtCoarseMTask merged;
          for (int g : kv.second) {
            const auto& src = region.antichainProbeGroups[g];
            merged.taskCount += src.taskCount;
            merged.staticCost += src.staticCost;
            merged.memberNodeCost += src.memberNodeCost;
            merged.workerZeroOnly = merged.workerZeroOnly || src.workerZeroOnly;
            for (const auto& layer : src.layerTaskCppIds) {
              for (int cppId : layer) {
                int layerIdx = -1;
                for (size_t li = 0; li < region.layers.size(); li++) {
                  for (int tc : region.layers[li].taskCppIds) {
                    if (tc == cppId) { layerIdx = static_cast<int>(li); break; }
                  }
                  if (layerIdx >= 0) break;
                }
                if (layerIdx < 0) continue;
                while (static_cast<int>(merged.layerTaskCppIds.size()) <= layerIdx) {
                  merged.layerTaskCppIds.push_back(std::vector<int>());
                }
                merged.layerTaskCppIds[layerIdx].push_back(cppId);
              }
            }
          }
          newGroups.push_back(merged);
        }
        int oldCount = G;
        region.antichainProbeGroups = std::move(newGroups);
        int newCount = static_cast<int>(region.antichainProbeGroups.size());
        region.antichainProbeTotalGroups = newCount;
        // Rebuild quotient DAG for the merged groups.
        region.antichainProbeDagAcyclic = false;
        std::map<int, int> newCppIdToGroup;
        for (int g = 0; g < newCount; g++) {
          for (const auto& layer : region.antichainProbeGroups[g].layerTaskCppIds) {
            for (int cppId : layer) newCppIdToGroup[cppId] = g;
          }
        }
        std::vector<std::vector<int>> newGroupCppIds(newCount);
        for (auto& kv : newCppIdToGroup) newGroupCppIds[kv.second].push_back(kv.first);
        for (int g = 0; g < newCount; g++) {
          region.antichainProbeGroups[g].succMTaskIndices.clear();
          region.antichainProbeGroups[g].predMTaskIndices.clear();
          region.antichainProbeGroups[g].upstreamDepCount = 0;
        }
        for (int gi = 0; gi < newCount; gi++) {
          for (int gj = 0; gj < newCount; gj++) {
            if (gi == gj) continue;
            bool edge = false;
            for (int from : newGroupCppIds[gi]) {
              if (edge) break;
              for (int to : newGroupCppIds[gj]) {
                if (mtTaskHasOrderingEdgeTo(from, to)) { edge = true; break; }
              }
            }
            if (edge) {
              region.antichainProbeGroups[gi].succMTaskIndices.push_back(gj);
              region.antichainProbeGroups[gj].predMTaskIndices.push_back(gi);
              region.antichainProbeGroups[gj].upstreamDepCount++;
            }
          }
        }
        // Verify acyclicity via topological sort.
        std::vector<int> indegree(newCount, 0);
        for (int g = 0; g < newCount; g++) {
          for (int s : region.antichainProbeGroups[g].succMTaskIndices) indegree[s]++;
        }
        std::deque<int> q;
        for (int g = 0; g < newCount; g++) if (indegree[g] == 0) q.push_back(g);
        int visited = 0;
        while (!q.empty()) {
          int u = q.front(); q.pop_front();
          visited++;
          for (int v : region.antichainProbeGroups[u].succMTaskIndices) {
            if (--indegree[v] == 0) q.push_back(v);
          }
        }
        region.antichainProbeDagAcyclic = (visited == newCount);
        fprintf(stderr, "[sarkar-contract] region [%d,%d) groups %d->%d acyclic=%d\n",
                region.beginCppId, region.endCppId, oldCount, newCount,
                region.antichainProbeDagAcyclic ? 1 : 0);
      }
    }
  }
  if (antichainRuntimeEnabled && region.antichainProbeDagAcyclic && antichainSelectedRegion) {
    region.useAntichainRuntime = true;
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
      if (from / ACTIVE_WIDTH == to / ACTIVE_WIDTH && mtCanCutEdge(tasks, from, to)) {
        region.replicationCandidateCount ++;
        region.repcutLiteCouldHelp = true;
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

static int mtCoarseStaticRecommendedWorkers(const MtCoarseRegion& region, int configuredWorkers) {
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

static bool mtCoarseStaticAdmitsRegion(const MtCoarseRegion& region, int workerCount) {
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
    // Track 2 Week 5: be much more conservative about which regions are worth
    // parallelizing. Require meaningful per-worker work and a large ratio of
    // useful work to synchronization overhead before suggesting workers.
    if (mtaskCount >= 8 && region.estimatedMaxParallelWidth >= workerCap &&
        usefulWork >= 256 && usefulPerWorker >= 64 && usefulWork >= copyMergeWords * 16)
      break;
    workerCap --;
  }
  return std::max(1, workerCap);
}

static int mtCoarseRecommendedWorkersForPolicy(const MtCoarseRegion& region,
                                               int configuredWorkers,
                                               const std::string& workerPolicy) {
  if (workerPolicy == "profitable") return mtCoarseProfitableRecommendedWorkers(region, configuredWorkers);
  return mtCoarseStaticRecommendedWorkers(region, configuredWorkers);
}

static bool mtCoarseAdmitsRegionForPolicy(const MtCoarseRegion& region,
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
  // Track 2 Week 5: be much more conservative about which regions are worth
  // parallelizing. The per-region barrier/copy/merge cost dominates for small
  // or low-width regions, so require meaningful per-worker work and a large
  // ratio of useful work to synchronization overhead.
  if (static_cast<int>(region.mtasks.size()) < 8) return false;
  if (region.estimatedMaxParallelWidth < workerCount) return false;
  if (usefulWork < 256) return false;
  if (usefulWork / workerCount < 64) return false;
  return usefulWork >= copyMergeWords * 16;
}

static std::string mtJoinIntList(const std::vector<int>& values) {
  std::string result;
  for (size_t i = 0; i < values.size(); i ++) {
    if (i != 0) result += ", ";
    result += std::to_string(values[i]);
  }
  return result;
}

static MtCoarseMTaskAssignment mtBuildCoarseMTaskAssignment(const MtCoarseRegion& region,
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

static MtCoarseProfileFacts mtComputeCoarseProfileFacts(const MtCoarseRegionPlan& coarsePlan) {
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

static MtPureBatchPlan planMtPureBatchesLegacy(const std::map<int, MtTaskInfo>& tasks, bool allowCuts) {
  MtPureBatchPlan plan;
  for (int idx = 0; idx < superId; idx ++) {
    int id;
    uint64_t mask;
    std::tie(id, mask) = setIdxMask(idx);
    bool activeWhole = mtActiveWordIsWhole(idx);
    if (!activeWhole || !mtTaskCanEnterPureBatch(tasks, idx)) continue;
    plan.segmentCount ++;

    std::vector<int> batch;
    std::vector<MtRepCutEdge> batchCutEdges;
    batch.push_back(idx);
    int batchEnd = idx + 1;
    while (batchEnd < superId && batchEnd / ACTIVE_WIDTH == id &&
           mtTaskCanJoinPureBatchWithCuts(tasks, batch, batchEnd, allowCuts, &batchCutEdges)) {
      batch.push_back(batchEnd);
      batchEnd ++;
    }
    if (batchEnd - idx > 1) {
      plan.batches.push_back(std::make_pair(idx, batchEnd));
      plan.cutEdges.insert(plan.cutEdges.end(), batchCutEdges.begin(), batchCutEdges.end());
      idx = batchEnd - 1;
    }
  }
  return plan;
}

static MtPureBatchPlan planMtPureBatchesActiveFrequency(const std::map<int, MtTaskInfo>& tasks, bool allowCuts) {
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
      std::vector<MtRepCutEdge> batchCutEdges;
      int batchBegin = idx;
      int batchEnd = idx;
      while (batchEnd < wordEnd &&
             mtTaskCanJoinPureBatchWithCuts(tasks, batch, batchEnd, allowCuts, &batchCutEdges)) {
        batch.push_back(batchEnd);
        batchEnd ++;
      }
      if (batchEnd - batchBegin > 1 &&
          mtBatchEstimatedCost(tasks, batchBegin, batchEnd) >= threshold) {
        plan.batches.push_back(std::make_pair(batchBegin, batchEnd));
        plan.cutEdges.insert(plan.cutEdges.end(), batchCutEdges.begin(), batchCutEdges.end());
      }
      idx = std::max(batchEnd, batchBegin + 1);
    }
  }
  return plan;
}

static MtPureBatchPlan planMtPureBatches(const std::map<int, MtTaskInfo>& tasks, bool allowCuts) {
  if (globalConfig.MtBatchFormationMode == "active-frequency" ||
      globalConfig.MtBatchFormationMode == "coarse") {
    return planMtPureBatchesActiveFrequency(tasks, allowCuts);
  }
  return planMtPureBatchesLegacy(tasks, allowCuts);
}

static std::string mtRepCutIntLiteral(ENode* enode) {
  if (!enode) return "";
  if (!enode->strVal.empty()) {
    if (enode->width > 64) return "";
    int base = 10;
    std::string digits;
    std::tie(base, digits) = firStrBase(enode->strVal);
    bool negative = !digits.empty() && digits[0] == '-';
    if (digits.empty()) digits = "0";
    if (negative) digits = digits.substr(1);
    std::string value;
    if (base == 16) value = "0x" + digits;
    else if (base == 2) value = "0b" + digits;
    else if (base == 8) value = "0" + digits;
    else {
      size_t firstNonZero = digits.find_first_not_of('0');
      value = firstNonZero == std::string::npos ? "0" : digits.substr(firstNonZero);
    }
    if (negative) return "(-" + value + ")";
    return value;
  }
  if (!enode->values.empty()) return std::to_string(enode->values[0]);
  return "";
}

static bool mtRepCutExprString(ENode* enode,
                               std::string& expr,
                               std::string& reason,
                               const std::map<Node*, std::string>& replacements = {},
                               int dependencySourceCppId = -1,
                               int dependencyBatchBegin = -1,
                               int dependencyBatchEnd = -1,
                               std::vector<MtRepCutLocalDecl>* localDecls = nullptr,
                               std::map<Node*, std::string>* localReplacements = nullptr,
                               std::set<Node*>* localVisitStack = nullptr,
                               const std::string& localNamePrefix = "") {
  std::vector<MtRepCutLocalDecl> localDeclStorage;
  std::map<Node*, std::string> localReplacementStorage;
  std::set<Node*> localVisitStorage;
  if (localDecls == nullptr) localDecls = &localDeclStorage;
  if (localReplacements == nullptr) localReplacements = &localReplacementStorage;
  if (localVisitStack == nullptr) localVisitStack = &localVisitStorage;
  if (!enode) {
    reason = "unsupported_expr";
    return false;
  }
  if (enode->nodePtr) {
    auto repl = replacements.find(enode->nodePtr);
    if (repl != replacements.end()) {
      expr = repl->second;
      return true;
    }
    auto localRepl = localReplacements->find(enode->nodePtr);
    if (localRepl != localReplacements->end()) {
      expr = localRepl->second;
      return true;
    }
    if (enode->nodePtr->isLocal()) {
      if (!enode->nodePtr->super || enode->nodePtr->super->cppId != dependencySourceCppId ||
          enode->nodePtr->assignTree.size() != 1 ||
          localVisitStack->find(enode->nodePtr) != localVisitStack->end()) {
        reason = "local_expr_dependency";
        return false;
      }
      ExpTree* localTree = enode->nodePtr->assignTree[0];
      ENode* localLval = localTree ? localTree->getlval() : nullptr;
      ENode* localRoot = localTree ? localTree->getRoot() : nullptr;
      if (!localLval || localLval->getChildNum() != 0 || localLval->getNode() != enode->nodePtr || !localRoot ||
          localRoot->opType == OP_WHEN || localRoot->opType == OP_RESET ||
          localRoot->opType == OP_STMT_WHEN || localRoot->opType == OP_STMT_SEQ) {
        reason = "local_expr_dependency";
        return false;
      }
      localVisitStack->insert(enode->nodePtr);
      std::string localExpr;
      bool ok = mtRepCutExprString(localRoot, localExpr, reason, replacements,
                                   dependencySourceCppId, dependencyBatchBegin, dependencyBatchEnd,
                                   localDecls, localReplacements, localVisitStack,
                                   localNamePrefix);
      localVisitStack->erase(enode->nodePtr);
      if (!ok) return false;
      MtRepCutLocalDecl decl;
      decl.node = enode->nodePtr;
      decl.cloneName = format("%s_local_%s", localNamePrefix.c_str(), enode->nodePtr->name.c_str());
      decl.expr = localExpr;
      decl.exprCost = mtENodeStaticCost(localRoot);
      localDecls->push_back(decl);
      (*localReplacements)[enode->nodePtr] = decl.cloneName;
      expr = decl.cloneName;
      return true;
    }
    if (enode->nodePtr->type == NODE_OTHERS && enode->nodePtr->super &&
        enode->nodePtr->super->cppId == dependencySourceCppId) {
      reason = "same_source_dependency_without_clone";
      return false;
    }
    if (dependencyBatchBegin >= 0 && enode->nodePtr->type == NODE_OTHERS &&
        enode->nodePtr->super && enode->nodePtr->super->cppId != dependencySourceCppId &&
        enode->nodePtr->super->cppId >= dependencyBatchBegin &&
        enode->nodePtr->super->cppId < dependencyBatchEnd) {
      reason = "same_batch_dependency_without_clone";
      return false;
    }
    expr = enode->nodePtr->name;
    return true;
  }
  if (enode->opType == OP_INT) {
    expr = mtRepCutIntLiteral(enode);
    if (expr.empty()) reason = "unsupported_expr";
    return !expr.empty();
  }

  auto childExpr = [&](size_t idx, std::string& out) {
    if (idx >= enode->getChildNum()) {
      reason = "unsupported_expr";
      return false;
    }
    return mtRepCutExprString(enode->getChild(idx), out, reason, replacements,
                              dependencySourceCppId, dependencyBatchBegin, dependencyBatchEnd,
                              localDecls, localReplacements, localVisitStack,
                              localNamePrefix);
  };

  std::string lhs;
  std::string rhs;
  switch (enode->opType) {
    case OP_ADD:
      if (!childExpr(0, lhs) || !childExpr(1, rhs)) return false;
      expr = "(" + lhs + " + " + rhs + ")";
      if (enode->width > 0) expr = "(" + expr + " & " + bitMask(enode->width) + ")";
      return true;
    case OP_AND:
      if (!childExpr(0, lhs) || !childExpr(1, rhs)) return false;
      expr = "(" + lhs + " & " + rhs + ")";
      return true;
    case OP_OR:
      if (!childExpr(0, lhs) || !childExpr(1, rhs)) return false;
      expr = "(" + lhs + " | " + rhs + ")";
      return true;
    case OP_XOR:
      if (!childExpr(0, lhs) || !childExpr(1, rhs)) return false;
      expr = "(" + lhs + " ^ " + rhs + ")";
      return true;
    case OP_EQ:
    case OP_NEQ: {
      if (enode->getChildNum() < 2) {
        reason = "unsupported_expr";
        return false;
      }
      ENode* lhsNode = enode->getChild(0);
      ENode* rhsNode = enode->getChild(1);
      if (!lhsNode || !rhsNode) {
        reason = "unsupported_expr";
        return false;
      }
      if (lhsNode->sign || rhsNode->sign ||
          lhsNode->opType == OP_SEXT || rhsNode->opType == OP_SEXT ||
          (lhsNode->opType == OP_INT && !lhsNode->strVal.empty() && lhsNode->strVal[0] == '-') ||
          (rhsNode->opType == OP_INT && !rhsNode->strVal.empty() && rhsNode->strVal[0] == '-')) {
        reason = "signed_compare_unsupported";
        return false;
      }
      if (lhsNode->width != rhsNode->width ||
          lhsNode->opType == OP_PAD || rhsNode->opType == OP_PAD) {
        reason = "compare_width_unsupported";
        return false;
      }
      if (!childExpr(0, lhs) || !childExpr(1, rhs)) return false;
      expr = "(" + lhs + (enode->opType == OP_EQ ? " == " : " != ") + rhs + ")";
      return true;
    }
    case OP_TAIL:
      if (!childExpr(0, lhs)) return false;
      if (enode->values.empty()) {
        reason = "unsupported_expr";
        return false;
      }
      expr = "(" + lhs + " & " + bitMask(MIN(enode->width, enode->values[0])) + ")";
      return true;
    case OP_BITS: {
      if (enode->getChildNum() < 1 || enode->values.size() < 2) {
        reason = "unsupported_expr";
        return false;
      }
      ENode* childNode = enode->getChild(0);
      if (!childNode || childNode->width <= 0 || enode->values[0] < enode->values[1]) {
        reason = "unsupported_expr";
        return false;
      }
      if (enode->values[1] >= childNode->width) {
        expr = "0";
        return true;
      }
      if (!childExpr(0, lhs)) return false;
      expr = "((" + lhs + " >> " + std::to_string(enode->values[1]) + ") & " + bitMask(enode->width) + ")";
      return true;
    }
    case OP_BITS_NOSHIFT: {
      if (enode->getChildNum() < 1 || enode->values.size() < 2) {
        reason = "unsupported_expr";
        return false;
      }
      ENode* childNode = enode->getChild(0);
      if (!childNode || childNode->width <= 0 || enode->values[0] < enode->values[1]) {
        reason = "unsupported_expr";
        return false;
      }
      if (enode->values[1] >= childNode->width) {
        expr = "0";
        return true;
      }
      if (!childExpr(0, lhs)) return false;
      expr = "(" + lhs + " & (" + bitMask(enode->values[0] + 1) + shiftBits(enode->values[1], ShiftDir::Right) + shiftBits(enode->values[1], ShiftDir::Left) + "))";
      return true;
    }
    case OP_PAD:
    case OP_ASUINT:
      if (!childExpr(0, lhs)) return false;
      expr = lhs;
      return true;
    case OP_MUX: {
      std::string cond;
      std::string tval;
      std::string fval;
      if (!childExpr(0, cond) || !childExpr(1, tval) || !childExpr(2, fval)) return false;
      if (enode->width == 1) expr = "((" + cond + " & " + tval + ") | ((!" + cond + ") & " + fval + "))";
      else expr = format("((-(%s)%s & %s) | ((-(%s)!%s) & %s))",
                         widthUType(enode->width).c_str(), cond.c_str(), tval.c_str(),
                         widthUType(enode->width).c_str(), cond.c_str(), fval.c_str());
      return true;
    }
    default:
      reason = "unsupported_expr";
      return false;
  }
}

static bool mtRepCutNodeFeedsSink(Node* node, int sinkCppId) {
  if (!node) return false;
  for (Node* next : node->next) {
    if (next && next->super && next->super->cppId == sinkCppId) return true;
  }
  for (Node* next : node->depNext) {
    if (next && next->super && next->super->cppId == sinkCppId) return true;
  }
  return false;
}

static Node* mtRepCutUniqueProducedNodeForSink(int cppId, int sinkCppId, std::string& reason) {
  auto superIter = cppId2Super.find(cppId);
  if (superIter == cppId2Super.end()) {
    reason = "missing_clone";
    return nullptr;
  }
  std::vector<Node*> candidates;
  for (Node* member : superIter->second->member) {
    if (!member || member->isLocal() || member->isArray()) continue;
    if (member->type != NODE_OTHERS) continue;
    if (member->assignTree.size() != 1) continue;
    if (!mtRepCutNodeFeedsSink(member, sinkCppId)) continue;
    candidates.push_back(member);
  }
  if (candidates.size() != 1) {
    reason = candidates.empty() ? "missing_clone" : "multi_output_sink_candidate";
    return nullptr;
  }
  return candidates[0];
}

static bool mtRepCutNodeOnlyFeedsAllowedSinks(Node* node,
                                             int sourceCppId,
                                             int batchBeginCppId,
                                             int batchEndCppId,
                                             const std::set<int>& allowedSinkCppIds) {
  if (!node) return false;
  bool hasAllowedSink = false;
  auto checkConsumer = [&](Node* next) {
    if (!next || !next->super || next->super->cppId < 0) return true;
    int nextCppId = next->super->cppId;
    if (nextCppId == sourceCppId) return true;
    if (allowedSinkCppIds.find(nextCppId) != allowedSinkCppIds.end()) {
      hasAllowedSink = true;
      return true;
    }
    if (batchBeginCppId <= nextCppId && nextCppId < batchEndCppId) return false;
    return true;
  };
  for (Node* next : node->next) {
    if (!checkConsumer(next)) return false;
  }
  for (Node* next : node->depNext) {
    if (!checkConsumer(next)) return false;
  }
  return hasAllowedSink;
}

static std::map<Node*, std::string> mtRepCutReplacementMap(const std::vector<MtRepCutClone>& clones) {
  std::map<Node*, std::string> replacements;
  for (const MtRepCutClone& clone : clones) {
    if (clone.sourceNode && clone.fallbackReason.empty()) replacements[clone.sourceNode] = clone.cloneName;
  }
  return replacements;
}

static MtRepCutClone mtPlanRepCutCloneForEdge(const MtRepCutEdge& edge,
                                              const std::map<Node*, std::string>& replacements,
                                              int batchBeginCppId,
                                              int batchEndCppId,
                                              const std::set<int>& allowedSinkCppIds) {
  MtRepCutClone clone;
  clone.sourceCppId = edge.fromCppId;
  clone.sinkCppId = edge.toCppId;
  std::string reason;
  Node* sourceNode = mtRepCutUniqueProducedNodeForSink(edge.fromCppId, edge.toCppId, reason);
  if (!sourceNode) {
    clone.fallbackReason = reason.empty() ? "missing_clone" : reason;
    return clone;
  }
  clone.sourceNode = sourceNode;
  clone.cloneName = format("repcut_%d_%d_%s", edge.fromCppId, edge.toCppId, sourceNode->name.c_str());
  if (!mtRepCutNodeOnlyFeedsAllowedSinks(sourceNode, edge.fromCppId, batchBeginCppId, batchEndCppId, allowedSinkCppIds)) {
    clone.fallbackReason = "multi_consumer_not_supported";
    return clone;
  }
  if (sourceNode->assignTree.size() != 1) {
    clone.fallbackReason = "missing_clone";
    return clone;
  }
  ExpTree* sourceTree = sourceNode->assignTree[0];
  ENode* sourceLval = sourceTree ? sourceTree->getlval() : nullptr;
  ENode* sourceRoot = sourceTree ? sourceTree->getRoot() : nullptr;
  if (!sourceLval || sourceLval->getChildNum() != 0 || sourceLval->getNode() != sourceNode || !sourceRoot ||
      sourceRoot->opType == OP_WHEN || sourceRoot->opType == OP_RESET ||
      sourceRoot->opType == OP_STMT_WHEN || sourceRoot->opType == OP_STMT_SEQ) {
    clone.fallbackReason = "unsupported_expr";
    return clone;
  }
  std::string expr;
  std::map<Node*, std::string> localReplacements;
  if (!mtRepCutExprString(sourceRoot, expr, reason, replacements,
                          edge.fromCppId, batchBeginCppId, batchEndCppId,
                          &clone.localDecls, &localReplacements, nullptr, clone.cloneName)) {
    clone.localDecls.clear();
    clone.fallbackReason = reason.empty() ? "unsupported_expr" : reason;
    return clone;
  }
  clone.expr = expr;
  clone.sourceExprCost = mtENodeStaticCost(sourceRoot);
  for (const MtRepCutLocalDecl& localDecl : clone.localDecls) clone.localExprCost += localDecl.exprCost;
  clone.plannedCloneCost = clone.sourceExprCost + clone.localExprCost + 1 + static_cast<int>(clone.localDecls.size());
  return clone;
}

static bool mtRepCutEdgeInBatch(const MtRepCutEdge& edge, const std::pair<int, int>& batch) {
  return batch.first <= edge.fromCppId && edge.fromCppId < batch.second &&
         batch.first <= edge.toCppId && edge.toCppId < batch.second;
}

static bool mtRepCutHasClonedEdge(const std::set<std::pair<int, int>>& clonedEdges,
                                  int fromCppId,
                                  int toCppId) {
  return clonedEdges.find(std::make_pair(fromCppId, toCppId)) != clonedEdges.end();
}

static MtRepCutSemanticPlan planMtRepCutSemantics(const std::map<int, MtTaskInfo>& tasks) {
  MtRepCutSemanticPlan semanticPlan;
  semanticPlan.batchPlan = planMtPureBatches(tasks, globalConfig.MtRepCutLiteMode == "on");

  for (auto batch : semanticPlan.batchPlan.batches) {
    MtRepCutBatch cutBatch;
    cutBatch.beginCppId = batch.first;
    cutBatch.endCppId = batch.second;
    std::vector<MtRepCutEdge> batchEdges;
    for (const MtRepCutEdge& edge : semanticPlan.batchPlan.cutEdges) {
      if (!mtRepCutEdgeInBatch(edge, batch)) continue;
      cutBatch.cutEdgeCount ++;
      cutBatch.forcedSinkCppIds.insert(edge.toCppId);
      batchEdges.push_back(edge);
    }
    if (cutBatch.cutEdgeCount == 0) continue;
    std::sort(batchEdges.begin(), batchEdges.end(), [](const MtRepCutEdge& lhs, const MtRepCutEdge& rhs) {
      if (lhs.toCppId != rhs.toCppId) return lhs.toCppId < rhs.toCppId;
      return lhs.fromCppId < rhs.fromCppId;
    });

    std::map<int, std::set<int>> allowedSinksBySource;
    for (const MtRepCutEdge& edge : batchEdges) allowedSinksBySource[edge.fromCppId].insert(edge.toCppId);

    std::vector<MtRepCutClone> batchClones;
    std::set<std::pair<int, int>> clonedEdges;
    size_t edgeIndex = 0;
    while (edgeIndex < batchEdges.size()) {
      int sinkCppId = batchEdges[edgeIndex].toCppId;
      std::map<Node*, std::string> sinkReplacements;
      size_t sinkEnd = edgeIndex;
      while (sinkEnd < batchEdges.size() && batchEdges[sinkEnd].toCppId == sinkCppId) sinkEnd ++;
      for (size_t idx = edgeIndex; idx < sinkEnd; idx ++) {
        const MtRepCutEdge& edge = batchEdges[idx];
        auto allowedIter = allowedSinksBySource.find(edge.fromCppId);
        static const std::set<int> emptyAllowedSinks;
        const std::set<int>& allowedSinkCppIds = allowedIter == allowedSinksBySource.end() ? emptyAllowedSinks : allowedIter->second;
        MtRepCutClone clone = mtPlanRepCutCloneForEdge(edge, sinkReplacements, batch.first, batch.second, allowedSinkCppIds);
        if (!clone.fallbackReason.empty()) {
          cutBatch.cloneFallbackReasons[clone.fallbackReason] ++;
          if (cutBatch.fallbackReason.empty()) cutBatch.fallbackReason = clone.fallbackReason;
        } else {
          cutBatch.cloneCount ++;
          cutBatch.plannedLocalDeclCount += static_cast<int>(clone.localDecls.size());
          cutBatch.plannedExprCost += clone.sourceExprCost + clone.localExprCost;
          cutBatch.plannedCloneCost += clone.plannedCloneCost;
          clonedEdges.insert(std::make_pair(clone.sourceCppId, clone.sinkCppId));
          if (clone.sourceNode) sinkReplacements[clone.sourceNode] = clone.cloneName;
        }
        batchClones.push_back(clone);
      }
      edgeIndex = sinkEnd;
    }
    bool forcedSinkInputsCloned = cutBatch.fallbackReason.empty() &&
                                  cutBatch.cloneCount == cutBatch.cutEdgeCount &&
                                  !cutBatch.forcedSinkCppIds.empty();
    for (int sinkCppId : cutBatch.forcedSinkCppIds) {
      auto sink = tasks.find(sinkCppId);
      if (sink == tasks.end() || sink->second.taskKind != "pure_compute" ||
          !sink->second.repcutSelected || !sink->second.serialReasons.empty()) {
        forcedSinkInputsCloned = false;
        if (cutBatch.fallbackReason.empty()) cutBatch.fallbackReason = "unsafe_forced_sink";
        break;
      }
      for (int sourceCppId = batch.first; sourceCppId < batch.second; sourceCppId ++) {
        if (sourceCppId == sinkCppId) continue;
        if (!mtTasksHaveDirectedEdge(cppId2Super[sourceCppId], cppId2Super[sinkCppId])) continue;
        if (!mtRepCutHasClonedEdge(clonedEdges, sourceCppId, sinkCppId)) {
          forcedSinkInputsCloned = false;
          if (cutBatch.fallbackReason.empty()) cutBatch.fallbackReason = "forced_sink_input_without_clone";
          break;
        }
      }
      if (!forcedSinkInputsCloned) break;
      cutBatch.forcedSinkMask |= (uint64_t)1 << (sinkCppId % ACTIVE_WIDTH);
    }
    cutBatch.parallelSafe = forcedSinkInputsCloned && cutBatch.forcedSinkMask != 0;
    cutBatch.forcedSerial = !cutBatch.parallelSafe;
    if (cutBatch.forcedSerial && cutBatch.fallbackReason.empty()) cutBatch.fallbackReason = "missing_clone";
    if (cutBatch.parallelSafe) {
      cutBatch.forcedSinkActivation = true;
      cutBatch.parallelSafeReason = "all_forced_sink_cut_inputs_cloned";
    }
    if (cutBatch.parallelSafe) {
      for (const MtRepCutClone& clone : batchClones) {
        if (!clone.fallbackReason.empty()) continue;
        auto sink = tasks.find(clone.sinkCppId);
        if (sink == tasks.end() || !sink->second.repcutRuntimeApplied) continue;
        cutBatch.emittedCloneCount ++;
        cutBatch.emittedLocalDeclCount += static_cast<int>(clone.localDecls.size());
        cutBatch.emittedExprCost += clone.sourceExprCost + clone.localExprCost;
        cutBatch.emittedCloneCost += clone.plannedCloneCost;
      }
    }
    semanticPlan.cutBatches.push_back(cutBatch);
    semanticPlan.clones.insert(semanticPlan.clones.end(), batchClones.begin(), batchClones.end());
  }


  return semanticPlan;
}
static void mtSetProfileRepCutBatchBeginCppIds(const MtRepCutSemanticPlan& semanticPlan) {
  mtProfileRepCutBatchBeginCppIds.clear();
  for (const MtRepCutBatch& batch : semanticPlan.cutBatches) mtProfileRepCutBatchBeginCppIds.push_back(batch.beginCppId);
}

static void mtSetProfileRepCutRuntimeCppIds(const std::map<int, MtTaskInfo>& tasks) {
  mtProfileRepCutRuntimeCppIds.clear();
  for (const auto& iter : tasks) {
    if (iter.second.repcutRuntimeApplied) mtProfileRepCutRuntimeCppIds.push_back(iter.first);
  }
}


static std::vector<MtRepCutClone> mtRepCutClonesForSink(const MtRepCutSemanticPlan& semanticPlan, int sinkCppId) {
  std::vector<MtRepCutClone> clones;
  for (const MtRepCutClone& clone : semanticPlan.clones) {
    if (clone.sinkCppId == sinkCppId && clone.fallbackReason.empty()) clones.push_back(clone);
  }
  return clones;
}

static bool mtRepCutNameChar(char ch) {
  return std::isalnum(static_cast<unsigned char>(ch)) || ch == '_' || ch == '$';
}

static std::string mtRepCutReplaceNodeNames(const std::string& text, const std::map<Node*, std::string>& replacements) {
  std::string result = text;
  for (const auto& repl : replacements) {
    const std::string& from = repl.first->name;
    const std::string& to = repl.second;
    std::string replaced;
    size_t pos = 0;
    while (pos < result.size()) {
      size_t hit = result.find(from, pos);
      if (hit == std::string::npos) {
        replaced.append(result.substr(pos));
        break;
      }
      bool leftOk = hit == 0 || !mtRepCutNameChar(result[hit - 1]);
      bool rightOk = hit + from.size() >= result.size() || !mtRepCutNameChar(result[hit + from.size()]);
      replaced.append(result.substr(pos, hit - pos));
      if (leftOk && rightOk) {
        replaced.append(to);
      } else {
        replaced.append(from);
      }
      pos = hit + from.size();
    }
    result.swap(replaced);
  }
  return result;
}

static uint64_t mtRepCutForcedSinkMaskForBatch(const MtRepCutSemanticPlan& semanticPlan, int beginCppId) {
  for (const MtRepCutBatch& batch : semanticPlan.cutBatches) {
    if (batch.beginCppId == beginCppId && batch.parallelSafe) return batch.forcedSinkMask;
  }
  return 0;
}

static void collectMtTaskRepCutCounts(std::map<int, MtTaskInfo>& tasks) {
  for (auto& iter : tasks) {
    MtTaskInfo& task = iter.second;
    task.repcutRole = "none";
    task.repcutSourceCount = 0;
    task.repcutSinkCount = 0;
    task.repcutFanout = 0;
    task.repcutCopyCost = task.hasCandidateCost ? task.candidateCost : 0;
    task.repcutBlockReason.clear();
    task.repcutSelected = false;
  }

  for (auto& iter : tasks) {
    int cppId = iter.first;
    MtTaskInfo& task = iter.second;
    SuperNode* super = cppId2Super[cppId];
    std::set<int> predCppIds;
    std::set<int> succCppIds;
    std::set<int> activeFanout;

    addCppIdsIfExecutable(predCppIds, super->prev);
    addCppIdsIfExecutable(predCppIds, super->depPrev);
    addCppIdsIfExecutable(succCppIds, super->next);
    addCppIdsIfExecutable(succCppIds, super->depNext);
    for (Node* member : super->member) {
      for (int activeId : member->nextNeedActivate) {
        if (activeId >= 0) activeFanout.insert(activeId);
      }
    }

    int sourceCount = 0;
    for (int predId : predCppIds) {
      auto pred = tasks.find(predId);
      if (pred != tasks.end() && pred->second.taskKind == "serial") sourceCount ++;
    }
    int sinkCount = 0;
    std::set<int> sinks = succCppIds;
    sinks.insert(activeFanout.begin(), activeFanout.end());
    for (int succId : sinks) {
      auto succ = tasks.find(succId);
      if (succ != tasks.end() && succ->second.taskKind == "serial") sinkCount ++;
    }

    if (task.taskKind == "pure_compute") {
      if (sourceCount == 0 && task.isSource) sourceCount = 1;
      if (sinkCount == 0 && task.isSink) sinkCount = 1;
      task.repcutSourceCount = sourceCount;
      task.repcutSinkCount = sinkCount;
      task.repcutFanout = static_cast<int>(sinks.size());
      if (sourceCount > 0 && sinkCount > 0) task.repcutRole = "candidate";
      else if (sourceCount > 0) task.repcutRole = "source";
      else if (sinkCount > 0) task.repcutRole = "sink";
    } else {
      task.repcutFanout = static_cast<int>(sinks.size());
      if (!task.serialReasons.empty()) task.repcutBlockReason = task.serialReasons.front();
      else task.repcutBlockReason = "serial_without_reason";
    }
  }
}

static std::map<int, MtTaskInfo> buildMtTaskInfoMapWithRepCut() {
  std::map<int, MtTaskInfo> tasks = buildMtTaskInfoMap();
  collectMtTaskRepCutCounts(tasks);
  return tasks;
}

static std::string repcutBlockReasonForTask(const MtTaskInfo& task) {
  if (task.taskKind != "pure_compute") {
    if (!task.serialReasons.empty()) return task.serialReasons.front();
    return task.repcutBlockReason.empty() ? "serial_without_reason" : task.repcutBlockReason;
  }
  if (!task.hasCandidateCost) return "missing_candidate_cost";
  if (task.repcutRole != "candidate") return "not_boundary_candidate";
  if (task.repcutSourceCount <= 0) return "missing_source_evidence";
  if (task.repcutSinkCount <= 0) return "missing_sink_evidence";
  if (task.repcutCopyCost <= 0) return "missing_copy_cost";
  return "";
}

static bool mtStateUpdateHasMemoryOrDynamicArray(const MtBoundaryInfo& boundary) {
  return boundary.hasMemoryWrite || boundary.hasMemoryRead || boundary.hasArrayOrDynamicIndex;
}

static bool mtStateUpdateHasExternalOrSpecial(const MtBoundaryInfo& boundary) {
  return boundary.hasExternal || boundary.hasSpecial || boundary.hasUnknownNode || boundary.hasUnknownOp;
}

static bool mtStateUpdateHasSameCycleTargetRead(const MtBoundaryInfo& boundary,
                                                const std::set<std::string>& allStateTargetNames) {
  if (boundary.hasRhsNextStateObjectRead) return true;
  for (const std::string& name : boundary.rhsReadStateTargetNames) {
    if (boundary.stateTargetNames.find(name) != boundary.stateTargetNames.end()) return true;
    if (allStateTargetNames.find(name) != allStateTargetNames.end()) return true;
  }
  return false;
}

static bool mtStateUpdateCanClassifyRhsTiming(const MtBoundaryInfo& boundary) {
  if (!boundary.hasStateUpdate) return false;
  if (boundary.hasAmbiguousStateTarget || boundary.stateTargetNames.size() != 1) return false;
  if (boundary.hasReset || boundary.hasAsyncReset || boundary.hasActivateAllPath) return false;
  if (mtStateUpdateHasMemoryOrDynamicArray(boundary)) return false;
  if (mtStateUpdateHasExternalOrSpecial(boundary)) return false;
  if (boundary.hasRhsNextStateObjectRead) return false;
  if (boundary.hasUnexpandedRhsDependency) return false;
  return true;
}

static std::string mtStateUpdateRhsTimingClass(const MtBoundaryInfo& boundary) {
  if (!mtStateUpdateCanClassifyRhsTiming(boundary)) return "unknown";
  if (boundary.stateSourceCommitCount == 1 && boundary.stateNextUpdateCount == 0 && boundary.stateResetUpdateCount == 0) {
    return "precomputed";
  }
  if (boundary.stateSourceCommitCount == 0 && boundary.stateNextUpdateCount == 1 && boundary.stateResetUpdateCount == 0) {
    return "old_state_only";
  }
  return "unknown";
}

static std::string mtStateUpdateRhsTimingEvidence(const MtBoundaryInfo& boundary,
                                                  const std::string& rhsTimingClass) {
  if (rhsTimingClass == "precomputed") return "reg_src_commit_reads_next_state_object";
  if (rhsTimingClass == "old_state_only") return "reg_dst_assign_tree_old_state_only";
  if (!boundary.hasStateUpdate) return "no_state_update";
  if (boundary.hasAmbiguousStateTarget || boundary.stateTargetNames.empty()) return "target_identity_ambiguous";
  if (boundary.stateTargetNames.size() > 1) return "multiple_state_targets";
  if (boundary.hasReset || boundary.hasAsyncReset || boundary.hasActivateAllPath) return "reset_or_activate_all";
  if (mtStateUpdateHasMemoryOrDynamicArray(boundary)) return "memory_or_dynamic_array";
  if (mtStateUpdateHasExternalOrSpecial(boundary)) return "external_or_special_or_unknown";
  if (boundary.hasRhsNextStateObjectRead) return "rhs_reads_next_state_object";
  if (boundary.hasUnexpandedRhsDependency) return "rhs_dependency_unexpanded";
  return "mixed_or_unproven_state_update";
}

static bool mtStateUpdateActivationCanUseDelta(const MtBoundaryInfo& boundary) {
  if (!boundary.hasStateUpdate) return false;
  if (boundary.hasReset || boundary.hasAsyncReset || boundary.hasActivateAllPath) return false;
  if (mtStateUpdateHasMemoryOrDynamicArray(boundary)) return false;
  if (mtStateUpdateHasExternalOrSpecial(boundary)) return false;
  return true;
}

static std::vector<std::string> mtStateUpdateBlockReasons(const MtBoundaryInfo& boundary,
                                                          SuperNode* super,
                                                          const std::string& rhsTimingClass,
                                                          bool rhsReadsSameCycleTarget) {
  std::vector<std::string> reasons;
  if (!boundary.hasStateUpdate) {
    addSerialReason(reasons, "no_state_update");
  } else {
    if (boundary.stateTargetNames.empty() || boundary.hasAmbiguousStateTarget) {
      addSerialReason(reasons, "target_identity_ambiguous");
    }
    if (boundary.stateTargetNames.size() > 1) addSerialReason(reasons, "multiple_state_targets");
    if (rhsTimingClass == "unknown") addSerialReason(reasons, "rhs_timing_unknown");
    if (rhsReadsSameCycleTarget) addSerialReason(reasons, "rhs_reads_same_cycle_target");
  }
  if (boundary.hasReset) addSerialReason(reasons, "reset_behavior");
  if (boundary.hasAsyncReset) addSerialReason(reasons, "async_reset_behavior");
  if (mtStateUpdateHasMemoryOrDynamicArray(boundary)) addSerialReason(reasons, "memory_or_dynamic_array");
  if (mtStateUpdateHasExternalOrSpecial(boundary)) addSerialReason(reasons, "external_or_special_or_unknown");
  if (boundary.hasStateUpdate && !mtStateUpdateActivationCanUseDelta(boundary)) addSerialReason(reasons, "activation_delta_unproven");
  if (super->superType != SUPER_VALID) addSerialReason(reasons, "non_valid_super_type");
  return reasons;
}

static std::string mtStateUpdateCandidateKind(const MtBoundaryInfo& boundary,
                                              const std::vector<std::string>& blockReasons) {
  if (!boundary.hasStateUpdate) return "blocked";
  if (blockReasons.empty() && boundary.stateTargetNames.size() == 1) return "safe_candidate";
  if (blockReasons.size() == 1 && blockReasons[0] == "multiple_state_targets") return "needs_split";
  return "blocked";
}

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

static const size_t MT_STATE_TARGET_WRITER_ID_LIMIT = 16;

static MtStateTargetWriterUniverse
collectMtStateTargetWriters(const std::map<int, MtTaskInfo>& mtTasks) {
  MtStateTargetWriterUniverse universe;
  for (const auto& iter : mtTasks) {
    int cppId = iter.first;
    const MtBoundaryInfo& boundary = iter.second.boundary;
    if (!boundary.hasStateUpdate) continue;
    if (boundary.stateTargetNames.empty() || boundary.hasAmbiguousStateTarget) {
      universe.hasIncompleteWriterUniverse = true;
      universe.incompleteWriterCppIds.insert(cppId);
      continue;
    }
    bool multiTargetWriter = boundary.stateTargetNames.size() > 1;
    for (const std::string& target : boundary.stateTargetNames) {
      universe.targetWriters[target].writerCount ++;
      universe.targetWriters[target].writerCppIds.insert(cppId);
      if (multiTargetWriter) universe.targetWriters[target].multiTargetWriterCount ++;
    }
  }
  return universe;
}

static MtStateTargetWriterInfo mtStateUpdateWriterInfo(
    const MtBoundaryInfo& boundary,
    const std::map<std::string, MtStateTargetWriterInfo>& targetWriters) {
  MtStateTargetWriterInfo info;
  if (boundary.stateTargetNames.size() != 1) return info;
  const std::string& target = *boundary.stateTargetNames.begin();
  auto iter = targetWriters.find(target);
  if (iter == targetWriters.end()) return info;
  info.writerCount = iter->second.writerCount;
  info.multiTargetWriterCount = iter->second.multiTargetWriterCount;
  for (int cppId : iter->second.writerCppIds) {
    if (info.writerCppIds.size() >= MT_STATE_TARGET_WRITER_ID_LIMIT) break;
    info.writerCppIds.insert(cppId);
  }
  return info;
}

static std::string mtStateUpdateTargetWriterConflictKind(
    const MtBoundaryInfo& boundary,
    const MtStateTargetWriterInfo& writerInfo,
    bool hasIncompleteWriterUniverse) {
  if (!boundary.hasStateUpdate || boundary.stateTargetNames.empty()) return "none";
  if (boundary.stateTargetNames.size() != 1) return "multi_target_unproven";
  if (writerInfo.writerCount <= 1 && hasIncompleteWriterUniverse) return "writer_universe_incomplete";
  if (writerInfo.writerCount <= 1) return "unique_writer";
  return "multi_writer_unproven";
}

static std::string mtStateUpdateTargetWriterProof(const std::string& conflictKind) {
  if (conflictKind == "unique_writer") return "target_unique_writer";
  if (conflictKind == "multi_writer_unproven") return "none";
  return "none";
}

static std::vector<std::string> mtStateUpdateRuntimeBlockReasons(
    const std::string& stateUpdateCandidateKind,
    const std::string& targetWriterConflictKind,
    const MtStateTargetWriterInfo& writerInfo) {
  std::vector<std::string> reasons;
  if (stateUpdateCandidateKind != "safe_candidate") addSerialReason(reasons, "not_local_safe_candidate");
  if (targetWriterConflictKind == "multi_writer_unproven") {
    addSerialReason(reasons, "target_multi_writer_unproven");
    if (writerInfo.multiTargetWriterCount > 0) addSerialReason(reasons, "target_multi_target_writer_unproven");
  } else if (targetWriterConflictKind == "multi_target_unproven") {
    addSerialReason(reasons, "target_multi_target_unproven");
  } else if (targetWriterConflictKind == "writer_universe_incomplete") {
    addSerialReason(reasons, "target_writer_universe_incomplete");
  } else if (targetWriterConflictKind != "unique_writer") {
    addSerialReason(reasons, "target_writer_proof_missing");
  }
  return reasons;
}

static std::set<std::string> collectAllMtStateTargetNames(const std::map<int, MtTaskInfo>& mtTasks) {
  std::set<std::string> names;
  for (const auto& iter : mtTasks) {
    const MtBoundaryInfo& boundary = iter.second.boundary;
    names.insert(boundary.stateTargetNames.begin(), boundary.stateTargetNames.end());
  }
  return names;
}

static std::vector<MtStateUpdateTraceInfo> buildMtStateUpdateTraceInfo(const std::map<int, MtTaskInfo>& mtTasks) {
  std::vector<MtStateUpdateTraceInfo> infos(superId);
  std::set<std::string> allStateTargetNames = collectAllMtStateTargetNames(mtTasks);
  MtStateTargetWriterUniverse stateTargetWriterUniverse = collectMtStateTargetWriters(mtTasks);
  for (const auto& iter : mtTasks) {
    int cppId = iter.first;
    if (cppId < 0 || cppId >= superId) continue;
    auto superIter = cppId2Super.find(cppId);
    if (superIter == cppId2Super.end() || superIter->second == nullptr) continue;
    const MtBoundaryInfo& boundary = iter.second.boundary;
    MtStateUpdateTraceInfo& info = infos[cppId];
    info.hasStateUpdate = boundary.hasStateUpdate;
    if (!boundary.hasStateUpdate) continue;
    std::string rhsTimingClass = mtStateUpdateRhsTimingClass(boundary);
    bool rhsReadsSameCycleTarget = mtStateUpdateHasSameCycleTargetRead(boundary, allStateTargetNames);
    std::vector<std::string> blockReasons = mtStateUpdateBlockReasons(boundary, superIter->second, rhsTimingClass, rhsReadsSameCycleTarget);
    std::string candidateKind = mtStateUpdateCandidateKind(boundary, blockReasons);
    MtStateTargetWriterInfo stateTargetWriterInfo = mtStateUpdateWriterInfo(boundary, stateTargetWriterUniverse.targetWriters);
    info.targetWriterConflictKind = mtStateUpdateTargetWriterConflictKind(
        boundary, stateTargetWriterInfo, stateTargetWriterUniverse.hasIncompleteWriterUniverse);
    info.runtimeBlockReasons = mtStateUpdateRuntimeBlockReasons(candidateKind, info.targetWriterConflictKind, stateTargetWriterInfo);
    info.localSafeCandidate = candidateKind == "safe_candidate";
    info.runtimeSafeCandidate = info.localSafeCandidate && info.runtimeBlockReasons.empty();
  }
  return infos;
}


static void applyRepCutLiteSelection(std::map<int, MtTaskInfo>& tasks) {
  int remainingBudget = globalConfig.MtRepCutCopyBudget;
  bool enabled = globalConfig.MtRepCutLiteMode == "on";
  for (auto& iter : tasks) {
    MtTaskInfo& task = iter.second;
    task.repcutSelected = false;
    task.repcutRuntimeApplied = false;
    task.repcutCutInEdges = 0;
    task.repcutCutOutEdges = 0;
    task.repcutBlockReason = repcutBlockReasonForTask(task);

    if (!enabled) {
      if (task.repcutBlockReason.empty()) task.repcutBlockReason = "disabled";
      continue;
    }
    if (globalConfig.MtRepCutCopyBudget <= 0) {
      if (task.repcutBlockReason.empty()) task.repcutBlockReason = "copy_budget_zero";
      continue;
    }
    if (globalConfig.MtRepCutFanoutBudget <= 0) {
      if (task.repcutBlockReason.empty()) task.repcutBlockReason = "fanout_budget_zero";
      continue;
    }
    if (!task.repcutBlockReason.empty()) continue;
    if (task.repcutFanout > globalConfig.MtRepCutFanoutBudget) {
      task.repcutBlockReason = "fanout_budget_exceeded";
      continue;
    }
    if (task.repcutCopyCost > remainingBudget) {
      task.repcutBlockReason = "copy_budget_exceeded";
      continue;
    }
    task.repcutSelected = true;
    remainingBudget -= task.repcutCopyCost;
  }

  for (auto& iter : tasks) {
    int cppId = iter.first;
    MtTaskInfo& task = iter.second;
    if (!enabled || task.repcutSelected || task.taskKind != "pure_compute") continue;
    if (task.hasCandidateCost && task.repcutCopyCost > 0) {
      std::set<int> predCppIds;
      addCppIdsIfExecutable(predCppIds, cppId2Super[cppId]->prev);
      addCppIdsIfExecutable(predCppIds, cppId2Super[cppId]->depPrev);
      bool hasPurePred = false;
      for (int predId : predCppIds) {
        auto pred = tasks.find(predId);
        if (pred != tasks.end() && pred->second.taskKind == "pure_compute") hasPurePred = true;
      }
      if (!hasPurePred) continue;
      if (globalConfig.MtRepCutCopyBudget <= 0) {
        task.repcutBlockReason = "copy_budget_zero";
        continue;
      }
      if (globalConfig.MtRepCutFanoutBudget <= 0) {
        task.repcutBlockReason = "fanout_budget_zero";
        continue;
      }
      if (task.repcutFanout > globalConfig.MtRepCutFanoutBudget) {
        task.repcutBlockReason = "fanout_budget_exceeded";
        continue;
      }
      if (task.repcutCopyCost > remainingBudget) {
        task.repcutBlockReason = "copy_budget_exceeded";
        continue;
      }
      task.repcutSelected = true;
      task.repcutBlockReason.clear();
      remainingBudget -= task.repcutCopyCost;
    }
  }
}

static std::map<int, MtTaskInfo> buildMtTaskInfoMapWithRepCutSelection() {
  std::map<int, MtTaskInfo> tasks = buildMtTaskInfoMapWithRepCut();
  applyRepCutLiteSelection(tasks);
  return tasks;
}

struct MtContextCacheState {
  bool hasRepCutTasks = false;
  bool hasRepCutSelectedTasks = false;
  std::map<int, MtTaskInfo> repCutTasks;
  bool hasCoarseRegionPlan = false;
  bool hasStateUpdateTraceInfo = false;
  MtCoarseRegionPlan coarseRegionPlan;
  std::vector<MtStateUpdateTraceInfo> stateUpdateTraceInfo;
  std::map<int, MtTaskInfo> repCutSelectedTasks;
};

static MtContextCacheState mtContextCache;

static void resetMtContextCache() {
  mtContextCache = MtContextCacheState();
  mtDependencyEdgeCache.clear();
  mtActiveEdgeCache.clear();
}

static std::map<int, MtTaskInfo> buildMtTaskInfoMapWithRepCutForInvocation() {
  if (!globalConfig.MtContextCache) return buildMtTaskInfoMapWithRepCut();
  if (!mtContextCache.hasRepCutTasks) {
    mtContextCache.repCutTasks = buildMtTaskInfoMapWithRepCut();
    mtContextCache.hasRepCutTasks = true;
  }
  return mtContextCache.repCutTasks;
}

static std::map<int, MtTaskInfo> buildMtTaskInfoMapWithRepCutSelectionForInvocation() {
  if (!globalConfig.MtContextCache) return buildMtTaskInfoMapWithRepCutSelection();
  if (!mtContextCache.hasRepCutSelectedTasks) {
    std::map<int, MtTaskInfo> tasks = buildMtTaskInfoMapWithRepCutForInvocation();
    applyRepCutLiteSelection(tasks);
    mtContextCache.repCutSelectedTasks = std::move(tasks);
    mtContextCache.hasRepCutSelectedTasks = true;
  }
  return mtContextCache.repCutSelectedTasks;
}

static MtCoarseRegionPlan planMtCoarseRegionsForInvocation(const std::map<int, MtTaskInfo>& tasks) {
  if (!globalConfig.MtContextCache) return planMtCoarseRegions(tasks);
  if (!mtContextCache.hasCoarseRegionPlan) {
    mtContextCache.coarseRegionPlan = planMtCoarseRegions(tasks);
    mtContextCache.hasCoarseRegionPlan = true;
  }
  return mtContextCache.coarseRegionPlan;
}

static std::vector<MtStateUpdateTraceInfo> buildMtStateUpdateTraceInfoForInvocation(const std::map<int, MtTaskInfo>& tasks) {
  if (!globalConfig.MtContextCache) return buildMtStateUpdateTraceInfo(tasks);
  if (!mtContextCache.hasStateUpdateTraceInfo) {
    mtContextCache.stateUpdateTraceInfo = buildMtStateUpdateTraceInfo(tasks);
    mtContextCache.hasStateUpdateTraceInfo = true;
  }
  return mtContextCache.stateUpdateTraceInfo;
}

static void logMtReportTimer(const char* name, struct timeval start, struct timeval end) {
  if (!globalConfig.MtReportTimers) return;
  int64_t elapsedUs = (static_cast<int64_t>(end.tv_sec - start.tv_sec) * 1000000ll) + static_cast<int64_t>(end.tv_usec - start.tv_usec);
  if (elapsedUs < 0) elapsedUs = 0;
  printf("[mt-report-timer] %s = %llu ms\n", name, static_cast<unsigned long long>(elapsedUs / 1000ll));
}


static bool mtRepCutLiteRuntimeHelperModeEnabled() {
  if (globalConfig.MtHelperMode == "mt") return true;
  return globalConfig.MtHelperMode == "mt-level-dispatch" && mtUseLevelDispatchRepCutRuntime();
}

static bool mtTaskUsesRepCutLiteRuntime(const std::map<int, MtTaskInfo>& tasks, int cppId) {
  auto iter = tasks.find(cppId);
  if (iter == tasks.end()) return false;
  return mtRepCutLiteRuntimeHelperModeEnabled() && iter->second.repcutSelected &&
         iter->second.taskKind == "pure_compute";
}

static bool markMtRepCutLiteRuntimeApplied(std::map<int, MtTaskInfo>& tasks) {
  for (auto& iter : tasks) {
    iter.second.repcutRuntimeApplied = false;
    iter.second.repcutCutInEdges = 0;
    iter.second.repcutCutOutEdges = 0;
  }

  MtRepCutSemanticPlan semanticPlan = planMtRepCutSemantics(tasks);
  for (const MtRepCutEdge& edge : semanticPlan.batchPlan.cutEdges) {
    tasks[edge.fromCppId].repcutCutOutEdges ++;
    tasks[edge.toCppId].repcutCutInEdges ++;
  }

  if (!mtRepCutLiteRuntimeHelperModeEnabled()) return false;

  std::set<int> runtimeSinks;
  for (const MtRepCutBatch& batch : semanticPlan.cutBatches) {
    if (!batch.parallelSafe) continue;
    for (const MtRepCutClone& clone : semanticPlan.clones) {
      if (!clone.fallbackReason.empty()) continue;
      if (clone.sourceCppId < batch.beginCppId || clone.sourceCppId >= batch.endCppId) continue;
      if (clone.sinkCppId < batch.beginCppId || clone.sinkCppId >= batch.endCppId) continue;
      if (batch.forcedSinkCppIds.find(clone.sinkCppId) == batch.forcedSinkCppIds.end()) continue;
      runtimeSinks.insert(clone.sinkCppId);
    }
  }

  bool anyApplied = false;
  for (int cppId : runtimeSinks) {
    auto task = tasks.find(cppId);
    if (task != tasks.end() && mtTaskUsesRepCutLiteRuntime(tasks, cppId)) {
      task->second.repcutRuntimeApplied = true;
      anyApplied = true;
    }
  }

  return anyApplied;
}

void graph::dumpMtScheduleJson() {
  std::string baseName = globalConfig.InputBaseName.empty() ? name : globalConfig.InputBaseName;
  std::string path = globalConfig.OutputDir + "/" + baseName + "_mt_schedule.json";
  FILE* fp = std::fopen(path.c_str(), "w");
  Assert(fp != nullptr, "failed to open mt schedule json %s", path.c_str());
  struct timeval mtReportTimerStart = getTime();
  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapWithRepCutForInvocation();
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
    }

    fprintf(fp, "    {\n");
    fprintf(fp, "      \"cpp_id\": %d,\n", cppId);
    fprintf(fp, "      \"scan_index\": %d,\n", cppId);
    fprintf(fp, "      \"super_id\": %d,\n", super->id);
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
    fprintf(fp, "      },\n");

    fprintf(fp, "      \"repcut\": {\n");
    fprintf(fp, "        \"is_source\": %s,\n", mtTask.isSource ? "true" : "false");
    fprintf(fp, "        \"is_sink\": %s,\n", mtTask.isSink ? "true" : "false");
    if (mtTask.hasCandidateCost) {
      fprintf(fp, "        \"candidate_cost\": %d,\n", mtTask.candidateCost);
    } else {
      fprintf(fp, "        \"candidate_cost\": null,\n");
    }
    fprintf(fp, "        \"repcut_role\": \"%s\",\n", mtTask.repcutRole.c_str());
    fprintf(fp, "        \"repcut_source_count\": %d,\n", mtTask.repcutSourceCount);
    fprintf(fp, "        \"repcut_sink_count\": %d,\n", mtTask.repcutSinkCount);
    fprintf(fp, "        \"repcut_copy_cost\": %d,\n", mtTask.repcutCopyCost);
    fprintf(fp, "        \"repcut_fanout\": %d,\n", mtTask.repcutFanout);
    std::string blockReason = repcutBlockReasonForTask(mtTask);
    if (blockReason.empty()) {
      fprintf(fp, "        \"repcut_block_reason\": null\n");
    } else {
      fprintf(fp, "        \"repcut_block_reason\": \"%s\"\n", jsonEscape(blockReason).c_str());
    }
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
  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapWithRepCutSelectionForInvocation();
  markMtRepCutLiteRuntimeApplied(mtTasks);
  MtDenseSchedule schedule = buildMtDenseSchedule(mtTasks, mtUseDenseExecutorCodegen());

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
  int denseRuntimeSameThreadEdgeElidedCount = 0;
  std::vector<std::vector<int>> denseRuntimeSuccs = mtBuildDenseRuntimeSuccs(schedule.mtasks, schedule.mtaskThreadAssign, denseXThreadDepsOnly, &denseRuntimeSameThreadEdgeElidedCount);
  int denseRuntimeDependencyEdgeCountBeforeTransitiveReduce = mtDenseRuntimeEdgeCount(denseRuntimeSuccs);
  int denseRuntimeTransitiveEdgeElidedCount = denseTransitiveReduceEdges ? mtReduceDenseRuntimeSuccsTransitive(denseRuntimeSuccs, schedule.mtaskThreadAssign) : 0;
  int denseRuntimeDependencyEdgeCount = mtDenseRuntimeEdgeCount(denseRuntimeSuccs);
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

  fprintf(fp, "  \"tasks\": [\n");
  for (int cppId = 0; cppId < superId; cppId ++) {
    SuperNode* super = cppId2Super[cppId];
    MtTaskInfo& mtTask = mtTasks[cppId];
    int activeWord;
    uint64_t activeMask;
    std::tie(activeWord, activeMask) = setIdxMask(cppId);
    fprintf(fp, "    {\"cpp_id\": %d, \"scan_index\": %d, \"super_id\": %d, \"super_type\": \"%s\", ", cppId, cppId, super->id, superTypeName(super->superType));
    fprintf(fp, "\"task_kind\": \"%s\", \"serial_reasons\": ", mtTask.taskKind.c_str());
    dumpJsonStringArray(fp, mtTask.serialReasons);
    fprintf(fp, ", \"worker0_only\": %s, \"is_always_active\": %s, ",
            hasWorker0OnlyReason(mtTask.serialReasons) ? "true" : "false",
            isAlwaysActive(cppId) ? "true" : "false");
    fprintf(fp, "\"active_word\": %d, \"active_mask\": \"0x%" PRIx64 "\", ", activeWord, activeMask);
    fprintf(fp, "\"static_cost\": %d, \"member_node_cost\": %zu, ", mtTaskEstimatedCost(mtTasks, cppId), super->member.size());
    fprintf(fp, "\"pred_cpp_ids\": ");
    dumpJsonIntArray(fp, schedule.predCppIds[(size_t)cppId]);
    fprintf(fp, ", \"succ_cpp_ids\": ");
    dumpJsonIntArray(fp, schedule.succCppIds[(size_t)cppId]);
    fprintf(fp, ", \"boundary\": {\"has_state_update\": %s, \"has_memory_write\": %s, \"has_reset\": %s, \"has_external\": %s, \"has_special\": %s}, ",
            mtTask.boundary.hasStateUpdate ? "true" : "false",
            mtTask.boundary.hasMemoryWrite ? "true" : "false",
            mtTask.boundary.hasReset ? "true" : "false",
            mtTask.boundary.hasExternal ? "true" : "false",
            mtTask.boundary.hasSpecial ? "true" : "false");
    fprintf(fp, "\"repcut_runtime_applied\": %s", mtTask.repcutRuntimeApplied ? "true" : "false");
    if (mtUseDenseMemberMetadata()) {
      dumpMtDenseMemberMetadataForTask(fp, super);
    }
    fprintf(fp, "}");
    fprintf(fp, "%s\n", cppId + 1 == superId ? "" : ",");
  }
  fprintf(fp, "  ],\n");

  fprintf(fp, "  \"edges\": [\n");
  for (size_t i = 0; i < schedule.edges.size(); i ++) {
    const MtDenseEdge& edge = schedule.edges[i];
    fprintf(fp, "    {\"from_cpp_id\": %d, \"to_cpp_id\": %d, \"kind\": \"%s\"}%s\n",
            edge.fromCppId, edge.toCppId, edge.kind.c_str(), i + 1 == schedule.edges.size() ? "" : ",");
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

void graph::dumpMtRepCutLiteReport() {
  std::string baseName = globalConfig.InputBaseName.empty() ? name : globalConfig.InputBaseName;
  std::string path = globalConfig.OutputDir + "/" + baseName + "_mt_repcut_lite.json";
  FILE* fp = std::fopen(path.c_str(), "w");
  Assert(fp != nullptr, "failed to open mt repcut-lite report %s", path.c_str());
  struct timeval mtReportTimerStart = getTime();
  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapWithRepCutSelectionForInvocation();
  bool appliedToRuntime = markMtRepCutLiteRuntimeApplied(mtTasks);
  MtPureBatchPlan uncutPlan = planMtPureBatches(mtTasks, false);
  MtPureBatchPlan cutPlan = planMtPureBatches(mtTasks, globalConfig.MtRepCutLiteMode == "on");
  MtRepCutSemanticPlan semanticPlan = planMtRepCutSemantics(mtTasks);

  int selectedCount = 0;
  int selectedBoundaryCandidateCount = 0;
  int selectedSinkCount = 0;
  int selectedCost = 0;
  int boundaryCandidateCount = 0;
  int pureCount = 0;
  for (auto& iter : mtTasks) {
    const MtTaskInfo& task = iter.second;
    if (task.taskKind == "pure_compute") pureCount ++;
    if (task.repcutRole == "candidate") boundaryCandidateCount ++;
    if (task.repcutSelected) {
      selectedCount ++;
      selectedCost += task.repcutCopyCost;
      if (task.repcutRole == "candidate") selectedBoundaryCandidateCount ++;
      if (task.repcutRole == "sink") selectedSinkCount ++;
    }
  }

  int plannedCloneCount = 0;
  int plannedLocalDeclCount = 0;
  int plannedExprCost = 0;
  int plannedCloneCost = 0;
  int emittedCloneCount = 0;
  int emittedLocalDeclCount = 0;
  int emittedExprCost = 0;
  int emittedCloneCost = 0;
  for (const MtRepCutBatch& batch : semanticPlan.cutBatches) {
    plannedCloneCount += batch.cloneCount;
    plannedLocalDeclCount += batch.plannedLocalDeclCount;
    plannedExprCost += batch.plannedExprCost;
    plannedCloneCost += batch.plannedCloneCost;
    emittedCloneCount += batch.emittedCloneCount;
    emittedLocalDeclCount += batch.emittedLocalDeclCount;
    emittedExprCost += batch.emittedExprCost;
    emittedCloneCost += batch.emittedCloneCost;
  }

  fprintf(fp, "{\n");
  fprintf(fp, "  \"format\": \"gsim.mt-repcut-lite.v1\",\n");
  fprintf(fp, "  \"mode\": \"%s\",\n", globalConfig.MtRepCutLiteMode.c_str());
  fprintf(fp, "  \"copy_budget\": %d,\n", globalConfig.MtRepCutCopyBudget);
  fprintf(fp, "  \"fanout_budget\": %d,\n", globalConfig.MtRepCutFanoutBudget);
  fprintf(fp, "  \"task_count\": %d,\n", superId);
  fprintf(fp, "  \"pure_task_count\": %d,\n", pureCount);
  fprintf(fp, "  \"boundary_candidate_count\": %d,\n", boundaryCandidateCount);
  fprintf(fp, "  \"candidate_count\": %d,\n", boundaryCandidateCount);
  fprintf(fp, "  \"selected_count\": %d,\n", selectedCount);
  fprintf(fp, "  \"selected_sink_count\": %d,\n", selectedSinkCount);
  fprintf(fp, "  \"selected_boundary_candidate_count\": %d,\n", selectedBoundaryCandidateCount);
  fprintf(fp, "  \"selected_copy_cost\": %d,\n", selectedCost);
  fprintf(fp, "  \"planned_clone_count\": %d,\n", plannedCloneCount);
  fprintf(fp, "  \"planned_local_decl_count\": %d,\n", plannedLocalDeclCount);
  fprintf(fp, "  \"planned_expr_cost\": %d,\n", plannedExprCost);
  fprintf(fp, "  \"planned_clone_cost\": %d,\n", plannedCloneCost);
  fprintf(fp, "  \"emitted_clone_count\": %d,\n", emittedCloneCount);
  fprintf(fp, "  \"emitted_local_decl_count\": %d,\n", emittedLocalDeclCount);
  fprintf(fp, "  \"emitted_expr_cost\": %d,\n", emittedExprCost);
  fprintf(fp, "  \"emitted_clone_cost\": %d,\n", emittedCloneCost);
  fprintf(fp, "  \"ordering\": \"cpp_id\",\n");
  fprintf(fp, "  \"applied_to_runtime\": %s,\n", appliedToRuntime ? "true" : "false");
  fprintf(fp, "  \"uncut_batch_count\": %d,\n", uncutPlan.segmentCount);
  fprintf(fp, "  \"cut_batch_count\": %d,\n", cutPlan.segmentCount);
  fprintf(fp, "  \"cut_edge_count\": %zu,\n", cutPlan.cutEdges.size());
  fprintf(fp, "  \"cut_edges\": [\n");
  for (size_t i = 0; i < cutPlan.cutEdges.size(); i ++) {
    const MtRepCutEdge& edge = cutPlan.cutEdges[i];
    fprintf(fp, "    {\n");
    fprintf(fp, "      \"from_cpp_id\": %d,\n", edge.fromCppId);
    fprintf(fp, "      \"to_cpp_id\": %d,\n", edge.toCppId);
    fprintf(fp, "      \"reason\": \"%s\"\n", jsonEscape(edge.reason).c_str());
    fprintf(fp, "    }%s\n", i + 1 == cutPlan.cutEdges.size() ? "" : ",");
  }
  fprintf(fp, "  ],\n");
  fprintf(fp, "  \"cut_batches\": [\n");
  for (size_t i = 0; i < semanticPlan.cutBatches.size(); i ++) {
    const MtRepCutBatch& batch = semanticPlan.cutBatches[i];
    fprintf(fp, "    {\n");
    fprintf(fp, "      \"begin_cpp_id\": %d,\n", batch.beginCppId);
    fprintf(fp, "      \"end_cpp_id\": %d,\n", batch.endCppId);
    fprintf(fp, "      \"cut_edge_count\": %d,\n", batch.cutEdgeCount);
    fprintf(fp, "      \"clone_count\": %d,\n", batch.cloneCount);
    fprintf(fp, "      \"planned_local_decl_count\": %d,\n", batch.plannedLocalDeclCount);
    fprintf(fp, "      \"planned_expr_cost\": %d,\n", batch.plannedExprCost);
    fprintf(fp, "      \"planned_clone_cost\": %d,\n", batch.plannedCloneCost);
    fprintf(fp, "      \"emitted_clone_count\": %d,\n", batch.emittedCloneCount);
    fprintf(fp, "      \"emitted_local_decl_count\": %d,\n", batch.emittedLocalDeclCount);
    fprintf(fp, "      \"emitted_expr_cost\": %d,\n", batch.emittedExprCost);
    fprintf(fp, "      \"emitted_clone_cost\": %d,\n", batch.emittedCloneCost);
    fprintf(fp, "      \"clone_fallback_reasons\": {");
    bool firstCloneFallbackReason = true;
    for (const auto& reason : batch.cloneFallbackReasons) {
      if (!firstCloneFallbackReason) fprintf(fp, ", ");
      firstCloneFallbackReason = false;
      fprintf(fp, "\"%s\": %d", jsonEscape(reason.first).c_str(), reason.second);
    }
    fprintf(fp, "},\n");
    fprintf(fp, "      \"forced_sink_cpp_ids\": ");
    dumpJsonIntArray(fp, batch.forcedSinkCppIds);
    fprintf(fp, ",\n");
    fprintf(fp, "      \"forced_sink_mask\": \"0x%lx\",\n", batch.forcedSinkMask);
    fprintf(fp, "      \"forced_sink_activation\": %s,\n", batch.forcedSinkActivation ? "true" : "false");
    fprintf(fp, "      \"forced_serial\": %s,\n", batch.forcedSerial ? "true" : "false");
    fprintf(fp, "      \"parallel_safe\": %s,\n", batch.parallelSafe ? "true" : "false");
    if (!batch.parallelSafeReason.empty()) {
      fprintf(fp, "      \"parallel_safe_reason\": \"%s\"", jsonEscape(batch.parallelSafeReason).c_str());
    } else {
      fprintf(fp, "      \"parallel_safe_reason\": null");
    }
    if (!batch.fallbackReason.empty()) {
      fprintf(fp, ",\n      \"fallback_reason\": \"%s\"\n", jsonEscape(batch.fallbackReason).c_str());
    } else {
      fprintf(fp, ",\n      \"fallback_reason\": null\n");
    }
    fprintf(fp, "    }%s\n", i + 1 == semanticPlan.cutBatches.size() ? "" : ",");
  }
  fprintf(fp, "  ],\n");
  fprintf(fp, "  \"duplicated_nodes\": [\n");
  bool firstClone = true;
  for (const MtRepCutClone& clone : semanticPlan.clones) {
    if (!clone.fallbackReason.empty()) continue;
    auto sinkTask = mtTasks.find(clone.sinkCppId);
    bool cloneEmitted = sinkTask != mtTasks.end() && sinkTask->second.repcutRuntimeApplied;
    if (!firstClone) fprintf(fp, ",\n");
    firstClone = false;
    fprintf(fp, "    {\n");
    fprintf(fp, "      \"source_cpp_id\": %d,\n", clone.sourceCppId);
    fprintf(fp, "      \"sink_cpp_id\": %d,\n", clone.sinkCppId);
    fprintf(fp, "      \"source_node\": \"%s\",\n", jsonEscape(clone.sourceNode ? clone.sourceNode->name : "").c_str());
    fprintf(fp, "      \"clone_name\": \"%s\",\n", jsonEscape(clone.cloneName).c_str());
    fprintf(fp, "      \"emitted\": %s,\n", cloneEmitted ? "true" : "false");
    fprintf(fp, "      \"local_decl_count\": %zu,\n", clone.localDecls.size());
    fprintf(fp, "      \"source_expr_cost\": %d,\n", clone.sourceExprCost);
    fprintf(fp, "      \"local_expr_cost\": %d,\n", clone.localExprCost);
    fprintf(fp, "      \"emitted_clone_cost\": %d,\n", cloneEmitted ? clone.plannedCloneCost : 0);
    fprintf(fp, "      \"planned_local_decl_count\": %zu,\n", clone.localDecls.size());
    fprintf(fp, "      \"planned_source_expr_cost\": %d,\n", clone.sourceExprCost);
    fprintf(fp, "      \"planned_local_expr_cost\": %d,\n", clone.localExprCost);
    fprintf(fp, "      \"planned_clone_cost\": %d\n", clone.plannedCloneCost);
    fprintf(fp, "    }");
  }
  if (!firstClone) fprintf(fp, "\n");
  fprintf(fp, "  ],\n");
  fprintf(fp, "  \"tasks\": [\n");
  for (int cppId = 0; cppId < superId; cppId ++) {
    const MtTaskInfo& task = mtTasks[cppId];
    fprintf(fp, "    {\n");
    fprintf(fp, "      \"cpp_id\": %d,\n", cppId);
    fprintf(fp, "      \"task_kind\": \"%s\",\n", task.taskKind.c_str());
    fprintf(fp, "      \"repcut_role\": \"%s\",\n", task.repcutRole.c_str());
    fprintf(fp, "      \"repcut_source_count\": %d,\n", task.repcutSourceCount);
    fprintf(fp, "      \"repcut_sink_count\": %d,\n", task.repcutSinkCount);
    fprintf(fp, "      \"repcut_copy_cost\": %d,\n", task.repcutCopyCost);
    fprintf(fp, "      \"repcut_fanout\": %d,\n", task.repcutFanout);
    fprintf(fp, "      \"selected\": %s,\n", task.repcutSelected ? "true" : "false");
    fprintf(fp, "      \"runtime_applied\": %s,\n", task.repcutRuntimeApplied ? "true" : "false");
    fprintf(fp, "      \"cut_in_edges\": %d,\n", task.repcutCutInEdges);
    fprintf(fp, "      \"cut_out_edges\": %d,\n", task.repcutCutOutEdges);
    if (task.repcutBlockReason.empty()) {
      fprintf(fp, "      \"block_reason\": null,\n");
    } else {
      fprintf(fp, "      \"block_reason\": \"%s\",\n", jsonEscape(task.repcutBlockReason).c_str());
    }
    fprintf(fp, "      \"serial_reasons\": ");
    dumpJsonStringArray(fp, task.serialReasons);
    fprintf(fp, "\n");
    fprintf(fp, "    }%s\n", cppId + 1 == superId ? "" : ",");
  }
  fprintf(fp, "  ]\n");
  fprintf(fp, "}\n");
  fclose(fp);
  printf("[mt-repcut-lite] wrote %d tasks (%d selected, cost %d) to %s\n",
         superId, selectedCount, selectedCost, path.c_str());
  logMtReportTimer("repcut-lite", mtReportTimerStart, getTime());
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
  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapWithRepCutSelectionForInvocation();
  mtCoarseLogPhase("coarse-region.task-map");
  const char* segmentReportEnv = std::getenv("GSIM_MT_SEGMENT_REPORT");
  bool segmentReportEnabled = segmentReportEnv != nullptr && segmentReportEnv[0] != '\0' && segmentReportEnv[0] != '0';
  if (segmentReportEnabled) markMtRepCutLiteRuntimeApplied(mtTasks);
  MtCoarseRegionPlan coarsePlan = planMtCoarseRegionsForInvocation(mtTasks);
  mtCoarseLogPhase("coarse-region.plan-regions");
  MtPureBatchPlan fallbackPlan = planMtPureBatchesActiveFrequency(mtTasks, globalConfig.MtRepCutLiteMode == "on");
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
      if (mtIter->second.repcutRuntimeApplied) return false;
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
    fprintf(fp, "      \"antichain_probe_max_block_width\": %d,\n", region.antichainProbeMaxBlockWidth);
    fprintf(fp, "      \"antichain_probe_total_groups\": %d,\n", region.antichainProbeTotalGroups);
    fprintf(fp, "      \"mtask_static_cost_min\": %d,\n", region.mtaskStaticCostMin);
    fprintf(fp, "      \"mtask_static_cost_max\": %d,\n", region.mtaskStaticCostMax);
    fprintf(fp, "      \"mtask_static_cost_total\": %d,\n", region.mtaskStaticCostTotal);
    if (region.antichainProbeTotalGroups > 0) {
      fprintf(fp, "      \"antichain_probe_dag_is_acyclic\": %s,\n", region.antichainProbeDagAcyclic ? "true" : "false");
      fprintf(fp, "      \"use_antichain_runtime\": %s,\n", region.useAntichainRuntime ? "true" : "false");
      fprintf(fp, "      \"antichain_selection_reason\": \"%s\",\n", jsonEscape(region.antichainSelectionReason).c_str());
    }
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
    fprintf(fp, "      \"bounded_repcut_lite_could_remove_blocking_dependency\": %s,\n", region.repcutLiteCouldHelp ? "true" : "false");
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
    fprintf(fp, "      ],\n");
    fprintf(fp, "      \"antichain_probe_groups\": [\n");
    for (size_t mtaskIdx = 0; mtaskIdx < region.antichainProbeGroups.size(); mtaskIdx++) {
      const MtCoarseMTask& mtask = region.antichainProbeGroups[mtaskIdx];
      fprintf(fp, "        {\n");
      fprintf(fp, "          \"index\": %zu,\n", mtaskIdx);
      fprintf(fp, "          \"task_count\": %d,\n", mtask.taskCount);
      fprintf(fp, "          \"static_cost\": %d,\n", mtask.staticCost);
      fprintf(fp, "          \"member_node_cost\": %d,\n", mtask.memberNodeCost);
      fprintf(fp, "          \"upstream_dep_count\": %d,\n", mtask.upstreamDepCount);
      fprintf(fp, "          \"pred_mtask_indices\": ");
      dumpJsonIntArray(fp, mtask.predMTaskIndices);
      fprintf(fp, ",\n");
      fprintf(fp, "          \"succ_mtask_indices\": ");
      dumpJsonIntArray(fp, mtask.succMTaskIndices);
      fprintf(fp, ",\n");
      fprintf(fp, "          \"layer_task_cpp_ids\": [");
      for (size_t layerIdx = 0; layerIdx < mtask.layerTaskCppIds.size(); layerIdx++) {
        if (layerIdx != 0) fprintf(fp, ", ");
        dumpJsonIntArray(fp, mtask.layerTaskCppIds[layerIdx]);
      }
      fprintf(fp, "]\n");
      fprintf(fp, "        }%s\n", mtaskIdx + 1 == region.antichainProbeGroups.size() ? "" : ",");
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

  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapWithRepCutSelectionForInvocation();
  std::vector<MtStateUpdateTraceInfo> stateUpdateTraceInfo = buildMtStateUpdateTraceInfoForInvocation(mtTasks);
  MtCoarseRegionPlan coarsePlan = planMtCoarseRegionsForInvocation(mtTasks);

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
  bool envelopeLocalEvalEnabled = mtUseEnvelopeLocalEval() || envelopeLocalEvalDiagnosticsEnabled;
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

std::pair<int, int> cppId2flagIdx(int cppId) {
  int id = cppId / ACTIVE_WIDTH;
  int bit = cppId % ACTIVE_WIDTH;
  return std::make_pair(id, bit);
}

std::pair<int, uint64_t>setIdxMask(int cppId) {
  int id, bit;
  std::tie(id, bit) = cppId2flagIdx(cppId);
  uint64_t mask = (uint64_t)1 << bit;
  return std::make_pair(id, mask);
}

std::pair<int, uint64_t>clearIdxMask(int cppId) {
  int id, bit;
  std::tie(id, bit) = cppId2flagIdx(cppId);
  uint64_t mask = (uint64_t)1 << bit;
  if (ACTIVE_WIDTH == 64) mask = ~mask;
  else mask = (~mask) & (((uint64_t)1 << ACTIVE_WIDTH) - 1);
  return std::make_pair(id, mask);
}

ActiveType activeSet2bitMap(std::set<int>& activeId, std::map<uint64_t, ActiveType>& bitMapInfo, int curId) {
  uint64_t ret = 0;
  std::string comment = "";
  int uniqueIdx = 0;
  for (int id : activeId) {
    if (isAlwaysActive(id)) continue;
    int bitMapId;
    uint64_t bitMapMask;
    std::tie(bitMapId, bitMapMask) = setIdxMask(id);
    int num = 64 / ACTIVE_WIDTH;
    if (curId >= 0 && id > curId && bitMapId == curId / ACTIVE_WIDTH) {
      if (ret == 0) uniqueIdx = id % ACTIVE_WIDTH;
      else uniqueIdx = -1;
      ret |= bitMapMask;
      comment += std::to_string(id) + " ";
    } else {
      int beg = bitMapId - bitMapId % num;
      int end = beg + num;
      int findType = 0;
      uint64_t newMask = bitMapMask << ((bitMapId - beg) * ACTIVE_WIDTH);
      std::string newComment = std::to_string(id);
      if (bitMapInfo.find(bitMapId) != bitMapInfo.end()) {
        ACTIVE_MASK(bitMapInfo[bitMapId]) |= bitMapMask;
        ACTIVE_COMMENT(bitMapInfo[bitMapId]) += " " + std::to_string(id);
        ACTIVE_UNIQUE(bitMapInfo[bitMapId]) = -1;
        findType = 1; // no nothing
      } else {
        for (int newId = beg; newId < end; newId ++) {
          if (bitMapInfo.find(newId) != bitMapInfo.end()) {
            newMask |= ACTIVE_MASK(bitMapInfo[newId]) << ((newId - beg) * ACTIVE_WIDTH);
            newComment += " " + ACTIVE_COMMENT(bitMapInfo[newId]);
            findType = 2;  // find to merge
            bitMapInfo.erase(newId);
          }
        }
      }
      if (findType == 0) bitMapInfo[bitMapId] = std::make_tuple(bitMapMask, std::to_string(id), id % ACTIVE_WIDTH);
      else if (findType == 2) bitMapInfo[beg] = std::make_tuple(newMask, newComment, -1);
    }
  }
  return std::make_tuple(ret, comment, uniqueIdx);
}

std::string updateActiveStr(int idx, uint64_t mask, const std::string& activeBufferName = "") {
  if (!activeBufferName.empty()) return format("%s.orWord(%d, 0x%lx);", activeBufferName.c_str(), idx, mask);
  if (mask <= MAX_U8) return format("activeFlags[%d] |= 0x%lx;", idx, mask);
  if (mask <= MAX_U16) return format("*(uint16_t*)&activeFlags[%d] |= 0x%lx;", idx, mask);
  if (mask <= MAX_U32) return format("*(uint32_t*)&activeFlags[%d] |= 0x%lx;", idx, mask);
  return format("*(uint64_t*)&activeFlags[%d] |= 0x%lx;", idx, mask);
}

std::string updateActiveStr(int idx, uint64_t mask, std::string& cond, int uniqueId, const std::string& activeBufferName = "") {
  if (!activeBufferName.empty()) {
    if (uniqueId >= 0) {
      return format("%s.orWord(%d, %s%s);", activeBufferName.c_str(), idx, cond.c_str(), shiftBits(uniqueId, ShiftDir::Left).c_str());
    }
    int castWidth = 64;
    if (mask <= MAX_U8) castWidth = 8;
    else if (mask <= MAX_U16) castWidth = 16;
    else if (mask <= MAX_U32) castWidth = 32;
    return format("%s.orWord(%d, -(uint%d_t)%s & 0x%lx);", activeBufferName.c_str(), idx, castWidth, cond.c_str(), mask);
  }
  auto activeFlags = std::string("activeFlags[") + std::to_string(idx) + std::string("]");

  if (mask <= MAX_U8) {
    if (uniqueId >= 0) return format("%s |= %s%s;", activeFlags.c_str(), cond.c_str(), shiftBits(uniqueId, ShiftDir::Left).c_str());
    else return format("%s |= -(uint8_t)%s & 0x%lx;", activeFlags.c_str(), cond.c_str(), mask, activeFlags.c_str());
  }
  if (mask <= MAX_U16)
    return format("*(uint16_t*)&%s |= -(uint16_t)%s & 0x%lx;", activeFlags.c_str(), cond.c_str(), mask, activeFlags.c_str());
  if (mask <= MAX_U32)
    return format("*(uint32_t*)&%s |= -(uint32_t)%s & 0x%lx;", activeFlags.c_str(), cond.c_str(), mask, activeFlags.c_str());
  return format("*(uint64_t*)&%s |= -(uint64_t)%s & 0x%lx;", activeFlags.c_str(), cond.c_str(), mask, activeFlags.c_str());
}

static void inline includeLib(FILE* fp, std::string lib, bool isStd) {
  std::string format = isStd ? "#include <%s>\n" : "#include \"%s\"\n";
  fprintf(fp, format.c_str(), lib.c_str());
}

static void inline newLine(FILE* fp) {
  fprintf(fp, "\n");
}

std::string strReplace(std::string s, std::string oldStr, std::string newStr) {
  size_t pos;
  while ((pos = s.find(oldStr)) != std::string::npos) {
    s.replace(pos, oldStr.length(), newStr);
  }
  return s;
}

FILE* graph::genHeaderStart() {
  headerFilePath = globalConfig.OutputDir + "/" + name + ".h";
  headerTmpFilePath = globalConfig.MtStableOutput ? headerFilePath + ".tmp" : "";
  const std::string openPath = globalConfig.MtStableOutput ? headerTmpFilePath : headerFilePath;
  FILE* header = std::fopen(openPath.c_str(), "w");
  assert(header != NULL);
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

void graph::genHeaderEnd(FILE* fp) {
  fprintf(fp, "};\n");
  fprintf(fp, "#endif\n");
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
  fprintf(fp, "; // width = %d, lineno = %d\n", node->width, node->lineno);
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
                         std::string activeBufferName, int indent,
                         const std::string& accumFlagName, bool emitActivation) {
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
    if (activeBufferName.empty()) emitBodyLock(indent, "activateAll();\n");
    else emitBodyLock(indent, "%s.activateAll();\n", activeBufferName.c_str());
    emitBodyLock(indent, "%s = -1;\n", flagName.c_str());
  } else {
    if (ACTIVE_MASK(curMask) != 0) {
      std::string flagForOr = (!accumFlagName.empty() && activeBufferName.empty()) ? accumFlagName : flagName;
      if (opt) emitBodyLock(indent, "%s |= -(uint%d_t)%s & 0x%lx; // %s\n", flagForOr.c_str(), ACTIVE_WIDTH, condName.c_str(), ACTIVE_MASK(curMask), ACTIVE_COMMENT(curMask).c_str());
      else emitBodyLock(indent, "%s |= 0x%lx; // %s\n", flagForOr.c_str(), ACTIVE_MASK(curMask), ACTIVE_COMMENT(curMask).c_str());
    }
    for (auto iter : bitMapInfo) {
      auto str = opt ? updateActiveStr(iter.first, ACTIVE_MASK(iter.second), condName, ACTIVE_UNIQUE(iter.second), activeBufferName)
                     : updateActiveStr(iter.first, ACTIVE_MASK(iter.second), activeBufferName);
      emitBodyLock(indent, "%s // %s\n", str.c_str(), ACTIVE_COMMENT(iter.second).c_str());
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
                               std::string activeBufferName, int indent,
                               const std::string& accumFlagName, bool emitActivation) {
  if (!emitActivation) return;
  std::map<uint64_t, ActiveType> bitMapInfo;
  auto curMask = activeSet2bitMap(activateId, bitMapInfo, node->super->cppId);
  if (ACTIVE_MASK(curMask) != 0) {
    std::string orFlag = (!accumFlagName.empty() && activeBufferName.empty()) ? accumFlagName : flagName;
    emitBodyLock(indent, "%s |= 0x%lx; // %s\n", orFlag.c_str(), ACTIVE_MASK(curMask), ACTIVE_COMMENT(curMask).c_str());
  }
  for (auto iter : bitMapInfo) {
    emitBodyLock(indent, "%s // %s\n", updateActiveStr(iter.first, ACTIVE_MASK(iter.second), activeBufferName).c_str(), ACTIVE_COMMENT(iter.second).c_str());
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

static std::map<Node*, std::string> mtRepCutActiveReplacements;

int graph::translateInst(InstInfo inst, int indent, std::string flagName, std::string activeBufferName, const std::string& accumFlagName, bool emitActivation) {
  switch (inst.infoType) {
    case SUPER_INFO_IF:
      emitBodyLock(indent ++, "%s\n", mtRepCutReplaceNodeNames(inst.inst, mtRepCutActiveReplacements).c_str());
      break;
    case SUPER_INFO_ELSE:
      emitBodyLock(indent - 1,  "%s\n", mtRepCutReplaceNodeNames(inst.inst, mtRepCutActiveReplacements).c_str());
      break;
    case SUPER_INFO_DEDENT:
      emitBodyLock(--indent, "%s\n", mtRepCutReplaceNodeNames(inst.inst, mtRepCutActiveReplacements).c_str());
      break;
    case SUPER_INFO_STR:
      emitBodyLock(indent, "%s\n", mtRepCutReplaceNodeNames(inst.inst, mtRepCutActiveReplacements).c_str());
      break;
    case SUPER_INFO_ASSIGN_BEG:
      if (inst.node->isLocal() || inst.node->isArray() || inst.node->type == NODE_WRITER) break;
      emitBodyLock(indent, "%s %s = %s;\n", widthUType(inst.node->width).c_str(), oldName(inst.node).c_str(), inst.node->name.c_str());
      break;
    case SUPER_INFO_ASSIGN_END:
      if (inst.node->isLocal() || !inst.node->needActivate()) break;
      if (inst.node->isArray() || inst.node->type == NODE_WRITER) activateUncondNext(inst.node, inst.node->nextActiveId, false, flagName, activeBufferName, indent, accumFlagName, emitActivation);
      else activateNext(inst.node, inst.node->nextActiveId, oldName(inst.node), false, flagName, activeBufferName, indent, accumFlagName, emitActivation);
      break;
    default:
      break;
  }
  return indent;
}

static bool mtActAccEnabled() {
  const char* e = std::getenv("GSIM_MT_ACTACC");
  return e && e[0] == '1';
}

void graph::genSuperEval(SuperNode* super, std::string flagName, std::string activeBufferName, int indent, bool emitActivation) { // current indent = 2
  bool useAccum = emitActivation && mtActAccEnabled() && activeBufferName.empty() && super->superType != SUPER_EXTMOD && super->superType != SUPER_ASYNC_RESET;
  std::string accumVar;
  if (useAccum) {
    accumVar = format("__actac_%d", super->cppId);
    emitBodyLock(indent, "uint%d_t %s = 0;\n", ACTIVE_WIDTH, accumVar.c_str());
  }
  if (super->superType == SUPER_EXTMOD) { // TODO: normalize
    /* save old EXT_OUT*/
    for (size_t i = 1; i < super->member.size(); i ++) {
      if (!super->member[i]->needActivate()) continue;
      Node* extOut = super->member[i];
      emitBodyLock(indent, "%s %s = %s;\n", widthUType(extOut->width).c_str(), oldName(extOut).c_str(), extOut->name.c_str());
    }
    for (InstInfo inst : super->insts) {
      indent = translateInst(inst, indent, flagName, activeBufferName, accumVar, emitActivation);
    }
    for (size_t i = 1; i < super->member.size(); i ++) {
      if (!super->member[i]->needActivate()) continue;
      if (super->member[i]->isArray()) activateUncondNext(super->member[i], super->member[i]->nextActiveId, false, flagName, activeBufferName, indent, accumVar, emitActivation);
      else activateNext(super->member[i], super->member[i]->nextActiveId, oldName(super->member[i]), false, flagName, activeBufferName, indent, accumVar, emitActivation);
    }
  } else {
    if (super->superType == SUPER_ASYNC_RESET) {
      int resetId = super2ResetId[super->resetNode].second;
      if (!emitActivation && activeBufferName.empty()) {
        int denseResetId = -1;
        auto denseResetIt = super2DenseResetId.find(super->resetNode);
        if (denseResetIt != super2DenseResetId.end()) denseResetId = denseResetIt->second.second;
        Assert(denseResetId >= 0, "missing dense async reset id for %s", super->resetNode->name.c_str());
        emitBodyLock(indent, "subResetDense%d();\n", denseResetId);
      } else if (activeBufferName.empty()) emitBodyLock(indent, "subReset%d();\n", resetId);
      else emitBodyLock(indent, "subReset%d(%s);\n", resetId, activeBufferName.c_str());
    }
    /* local nodes definition */
    for (Node* n : super->member) {
      if (n->isLocal()) {
        emitBodyLock(indent, "%s %s;\n", widthUType(n->width).c_str(), n->name.c_str());
      }
    }
    for (InstInfo inst : super->insts) {
      indent = translateInst(inst, indent, flagName, activeBufferName, accumVar, emitActivation);
    }
    if (super->superType == SUPER_ASYNC_RESET) {
      int resetId = super2ResetId[super->resetNode].second;
      if (!emitActivation && activeBufferName.empty()) {
        int denseResetId = -1;
        auto denseResetIt = super2DenseResetId.find(super->resetNode);
        if (denseResetIt != super2DenseResetId.end()) denseResetId = denseResetIt->second.second;
        Assert(denseResetId >= 0, "missing dense async reset id for %s", super->resetNode->name.c_str());
        emitBodyLock(indent, "subResetDense%d();\n", denseResetId);
      } else if (activeBufferName.empty()) emitBodyLock(indent, "subReset%d();\n", resetId);
      else emitBodyLock(indent, "subReset%d(%s);\n", resetId, activeBufferName.c_str());
    }
    emitBodyLock(indent, "#ifdef ENABLE_LOG\n");
    emitBodyLock(indent ++, "if (cycles >= LOG_START && cycles <= LOG_END) {\n");
    for (Node* n : super->member) nodeDisplay(n, indent);
    emitBodyLock(-- indent, "}\n");
    emitBodyLock(indent, "#endif\n");
  }
  if (useAccum) {
    emitBodyLock(indent, "%s |= %s;\n", flagName.c_str(), accumVar.c_str());
  }
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
  if (buffered) {
    emitFuncDecl(0, "void S%s::mtTask%d(uint%d_t &flag, %s &nextActive) {\n", name.c_str(), super->cppId, ACTIVE_WIDTH, activeSinkType.c_str());
    genSuperEval(super, "flag", "nextActive", 1, true);
  } else {
    emitFuncDecl(0, "void S%s::mtTask%d(uint%d_t &flag) {\n", name.c_str(), super->cppId, ACTIVE_WIDTH);
    genSuperEval(super, "flag", "", 1, true);
  }
  emitBodyLock(0, "}\n");
}

void graph::genMtRepCutLiteTaskHelper(SuperNode* super, const std::vector<MtRepCutClone>& clones, const std::string& activeSinkType) {
  emitFuncDecl(0, "void S%s::mtRepCutLiteTask%d(uint%d_t &flag, %s &nextActive) {\n", name.c_str(), super->cppId, ACTIVE_WIDTH, activeSinkType.c_str());
  emitBodyLock(1, "if (mtProfileEnabled) mtProfileRepCutLiteTaskCallsByCppId[%d].fetch_add(1, std::memory_order_relaxed);\n", super->cppId);
  std::map<Node*, std::string> replacements = mtRepCutReplacementMap(clones);
  for (const MtRepCutClone& clone : clones) {
    for (const MtRepCutLocalDecl& localDecl : clone.localDecls) {
      emitBodyLock(1, "%s %s = %s;\n", widthUType(localDecl.node->width).c_str(), localDecl.cloneName.c_str(), localDecl.expr.c_str());
    }
    emitBodyLock(1, "%s %s = %s;\n", widthUType(clone.sourceNode->width).c_str(), clone.cloneName.c_str(), clone.expr.c_str());
  }
  mtRepCutActiveReplacements = replacements;
  genSuperEval(super, "flag", "nextActive", 1, true);
  mtRepCutActiveReplacements.clear();
  emitBodyLock(0, "}\n");
}

void graph::genMtTaskRunner(const MtRepCutSemanticPlan& semanticPlan) {
  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapWithRepCutSelectionForInvocation();
  markMtRepCutLiteRuntimeApplied(mtTasks);
  int shardCount = mtPureBatchShardCount();
  bool useCoarse = globalConfig.MtBatchFormationMode == "coarse";
  bool waitProbeCodegen = mtUseWaitProbeCodegen();
  auto emitPureTaskSwitchCases = [&](int shardBegin, int shardEnd, bool workerMode) {
    for (int cppId = shardBegin; cppId < shardEnd; cppId ++) {
      if (mtTasks[cppId].taskKind != "pure_compute") continue;
      emitBodyLock(4, "case %d:\n", cppId);
      if (workerMode) {
        emitBodyLock(5, "if (mtWorkerFlags[worker] & 0x%lx) {\n", (uint64_t)1 << (cppId % ACTIVE_WIDTH));
        emitBodyLock(6, "if (mtProfileEnabled) {\n");
        if (mtTasks[cppId].repcutRuntimeApplied) {
          emitBodyLock(7, "mtRepCutLiteTask%d(mtWorkerFlags[worker], mtWorkerDeltas[worker]);\n", cppId);
        } else {
          emitBodyLock(7, "mtTask%d(mtWorkerFlags[worker], mtWorkerDeltas[worker]);\n", cppId);
        }
        emitBodyLock(7, "mtProfileLocalTaskIds[worker].push_back(%d);\n", cppId);
        emitBodyLock(7, "mtProfileLocalWorkerTaskCount[worker] ++;\n");
        emitBodyLock(6, "} else {\n");
        if (mtTasks[cppId].repcutRuntimeApplied) {
          emitBodyLock(7, "mtRepCutLiteTask%d(mtWorkerFlags[worker], mtWorkerDeltas[worker]);\n", cppId);
        } else {
          emitBodyLock(7, "mtTask%d(mtWorkerFlags[worker], mtWorkerDeltas[worker]);\n", cppId);
        }
        emitBodyLock(6, "}\n");
        emitBodyLock(5, "}\n");
      } else {
        emitBodyLock(5, "if (activeWord & 0x%lx) {\n", (uint64_t)1 << (cppId % ACTIVE_WIDTH));
        emitBodyLock(6, "if (mtProfileEnabled) {\n");
        emitBodyLock(7, "std::chrono::steady_clock::time_point mtProfileTaskBegin = std::chrono::steady_clock::now();\n");
        if (mtTasks[cppId].repcutRuntimeApplied) {
          emitBodyLock(7, "ActivationDelta mtDirectDelta;\n");
          emitBodyLock(7, "mtRepCutLiteTask%d(activeWord, mtDirectDelta);\n", cppId);
          emitBodyLock(7, "mtDirectDelta.mergeInto(activeFlags);\n");
          emitBodyLock(7, "mtProfileActivationDeltaEntries += mtDirectDelta.entries.size();\n");
          emitBodyLock(7, "if (mtDirectDelta.entries.size() > mtProfileActivationDeltaMaxEntriesPerWorker) mtProfileActivationDeltaMaxEntriesPerWorker = mtDirectDelta.entries.size();\n");
          emitBodyLock(7, "if (mtDirectDelta.allActive) mtProfileActivationDeltaActivateAllCount ++;\n");
        } else {
          emitBodyLock(7, "mtTask%d(activeWord);\n", cppId);
        }
        emitBodyLock(7, "recordMtProfileTask(%d, true, std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileTaskBegin).count());\n", cppId);
        emitBodyLock(7, "mtProfileWorkerTaskCount[0] ++;\n");
        emitBodyLock(6, "} else {\n");
        if (mtTasks[cppId].repcutRuntimeApplied) {
          emitBodyLock(7, "ActivationDelta mtDirectDelta;\n");
          emitBodyLock(7, "mtRepCutLiteTask%d(activeWord, mtDirectDelta);\n", cppId);
          emitBodyLock(7, "mtDirectDelta.mergeInto(activeFlags);\n");
        } else {
          emitBodyLock(7, "mtTask%d(activeWord);\n", cppId);
        }
        emitBodyLock(6, "}\n");
        emitBodyLock(5, "}\n");
      }
      emitBodyLock(5, "break;\n");
    }
  };
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
  emitBodyLock(1, "mtWorkerPoolDoneCount.store(0, std::memory_order_relaxed);\n");
  emitBodyLock(1, "mtWorkerPoolGeneration.fetch_add(1, std::memory_order_release);\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::mtWorkerPoolWaitForDone(int expectedDoneCount) {\n", name.c_str());
  emitBodyLock(1, "expectedDoneCount = mtWorkerPoolThreadCount;\n");
  emitBodyLock(1, "while (mtWorkerPoolDoneCount.load(std::memory_order_acquire) < expectedDoneCount) {\n");
  emitBodyLock(2, "mtWorkerPoolPause();\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::mtWorkerPoolLoop(int worker) {\n", name.c_str());
  // Workers publish readiness before startMtWorkerPool returns, so the first
  // post cannot race a late-start worker that has not captured the baseline generation.
  emitBodyLock(1, "uint64_t seenGeneration = mtWorkerPoolGeneration.load(std::memory_order_acquire);\n");
  emitBodyLock(1, "mtWorkerPoolReadyCount.fetch_add(1, std::memory_order_release);\n");
  emitBodyLock(1, "while (true) {\n");
  emitBodyLock(2, "uint64_t generation = seenGeneration;\n");
  emitBodyLock(2, "while (true) {\n");
  emitBodyLock(3, "if (mtWorkerPoolStop.load(std::memory_order_acquire)) return;\n");
  emitBodyLock(3, "generation = mtWorkerPoolGeneration.load(std::memory_order_acquire);\n");
  emitBodyLock(3, "if (generation != seenGeneration) break;\n");
  emitBodyLock(3, "mtWorkerPoolPause();\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "seenGeneration = generation;\n");
  emitBodyLock(2, "if (mtWorkerPoolStop.load(std::memory_order_acquire)) return;\n");
  emitBodyLock(2, "const int workerCount = mtWorkerPoolCurrentWorkerCount;\n");
  emitBodyLock(2, "if (worker >= workerCount) {\n");
  emitBodyLock(3, "mtWorkerPoolDoneCount.fetch_add(1, std::memory_order_release);\n");
  emitBodyLock(3, "continue;\n");
  emitBodyLock(2, "}\n");
  emitBodyLock(2, "const int chunkBegin = mtWorkerPoolChunks[(size_t)worker].begin;\n");
  emitBodyLock(2, "const int chunkEnd = mtWorkerPoolChunks[(size_t)worker].end;\n");
  emitBodyLock(2, "const int jobKind = mtWorkerPoolJobKind;\n");
  if (useCoarse) {
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
    // 28c D-static Step 1: pool worker dispatched into the codegen-time
    // flat-array path. Region/wc/begin/span carried on dedicated fields
    // so we don't overload chunk[].begin/.end semantics.
    emitBodyLock(3, "mtRunCoarseRegionStaticDispatch(coarseRegionIndex, mtWorkerPoolCoarseStaticRoundedWC, worker, mtWorkerPoolCoarseStaticBeginActiveWord, mtWorkerPoolCoarseStaticActiveWordSpan);\n");
    if (waitProbeCodegen) emitBodyLock(3, "if (mtWaitProbeEnabled && (size_t)worker < mtWaitProbeWorkerFinishNs.size()) mtWaitProbeWorkerFinishNs[(size_t)worker] = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtWaitProbePostTp).count();\n");
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
  emitBodyLock(2, "mtWorkerPoolDoneCount.fetch_add(1, std::memory_order_release);\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::startMtWorkerPool() {\n", name.c_str());
  emitBodyLock(1, "if (!mtWorkerPoolEnabled || mtConfiguredWorkerCount <= 1 || !mtWorkerPoolThreads.empty()) return;\n");
  emitBodyLock(1, "// Logical worker 0 is the main thread; spawn N-1 background workers (logical IDs 1..N-1).\n");
  emitBodyLock(1, "mtWorkerPoolThreadCount = mtConfiguredWorkerCount - 1;\n");
  emitBodyLock(1, "mtWorkerPoolReadyCount.store(0, std::memory_order_relaxed);\n");
  emitBodyLock(1, "mtWorkerPoolChunks.assign((size_t)mtConfiguredWorkerCount, MtWorkerPoolChunk{0, 0});\n");
  emitBodyLock(1, "mtWorkerDeltas.resize((size_t)mtConfiguredWorkerCount);\n");
  emitBodyLock(1, "mtWorkerFlags.resize((size_t)mtConfiguredWorkerCount);\n");
  if (useCoarse) {
    emitBodyLock(1, "mtWorkerCoarseFlags.resize((size_t)mtConfiguredWorkerCount);\n");
    if (globalConfig.MtCoarseWorkerPolicyMode == "profitable") {
      emitBodyLock(1, "mtWorkerPoolMTaskAssignments.resize((size_t)mtConfiguredWorkerCount);\n");
    }
    // Track 2 Week 4: per-region atomic state for antichain runtime is initialized
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
  emitBodyLock(1, "for (int worker = 1; worker < mtConfiguredWorkerCount; worker ++) {\n");
  emitBodyLock(2, "mtWorkerPoolThreads.emplace_back([this, worker]() { mtWorkerPoolLoop(worker); });\n");
  emitBodyLock(2, "std::thread& t = mtWorkerPoolThreads.back();\n");
  emitBodyLock(2, "#ifdef __linux\n");
  emitBodyLock(2, "if (!mtAllowedCpus.empty()) {\n");
  emitBodyLock(3, "int mtCpuIdx = (mtCpuAffinityBase + worker - 1) % (int)mtAllowedCpus.size();\n");
  emitBodyLock(3, "cpu_set_t mtCpuset;\n");
  emitBodyLock(3, "CPU_ZERO(&mtCpuset);\n");
  emitBodyLock(3, "CPU_SET(mtAllowedCpus[mtCpuIdx], &mtCpuset);\n");
  emitBodyLock(3, "pthread_setaffinity_np(t.native_handle(), sizeof(mtCpuset), &mtCpuset);\n");
  emitBodyLock(2, "}\n");
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

  emitFuncDecl(0, "void S%s::mtRunPureBatch(int beginCppId, int endCppId, uint%d_t &activeWord) {\n", name.c_str(), ACTIVE_WIDTH);
  emitBodyLock(1, "int taskCount = endCppId - beginCppId;\n");
  emitBodyLock(1, "if (taskCount <= 0) return;\n");
  emitBodyLock(1, "std::chrono::steady_clock::time_point mtProfileBatchBegin;\n");
  emitBodyLock(1, "if (mtProfileEnabled) mtProfileBatchBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(1, "int workerCount = mtConfiguredWorkerCount;\n");
  emitBodyLock(1, "bool mtSkippedBelowMinBatch = false;\n");
  emitBodyLock(1, "bool mtSkippedForcedSerialBatch = false;\n");
  emitBodyLock(1, "if (taskCount < mtMinBatchTasks) {\n");
  emitBodyLock(2, "if (mtProfileEnabled) mtProfileRejectBelowMinBatch ++;\n");
  emitBodyLock(2, "mtSkippedBelowMinBatch = true;\n");
  emitBodyLock(2, "workerCount = 1;\n");
  emitBodyLock(1, "}\n");
  bool hasForcedSerialBatch = false;
  for (const MtRepCutBatch& batch : semanticPlan.cutBatches) {
    if (batch.forcedSerial) hasForcedSerialBatch = true;
  }
  if (hasForcedSerialBatch) {
    emitBodyLock(1, "switch (beginCppId) {\n");
    for (const MtRepCutBatch& batch : semanticPlan.cutBatches) {
      if (batch.forcedSerial) emitBodyLock(2, "case %d:\n", batch.beginCppId);
    }
    emitBodyLock(3, "mtSkippedForcedSerialBatch = true;\n");
    emitBodyLock(3, "if (mtProfileEnabled) mtProfileRejectDependencyEdge ++;\n");
    emitBodyLock(3, "workerCount = 1;\n");
    emitBodyLock(3, "break;\n");
    emitBodyLock(2, "default:\n");
    emitBodyLock(3, "break;\n");
    emitBodyLock(1, "}\n");
  }
  if (!semanticPlan.cutBatches.empty()) {
    emitBodyLock(1, "if (mtProfileEnabled) {\n");
    emitBodyLock(2, "switch (beginCppId) {\n");
    for (size_t batchIndex = 0; batchIndex < semanticPlan.cutBatches.size(); batchIndex ++) {
      const MtRepCutBatch& batch = semanticPlan.cutBatches[batchIndex];
      emitBodyLock(3, "case %d:\n", batch.beginCppId);
      emitBodyLock(4, "if (mtProfileRepCutBatchHits.size() > %zu) mtProfileRepCutBatchHits[%zu] ++;\n", batchIndex, batchIndex);
      emitBodyLock(4, "break;\n");
    }
    emitBodyLock(3, "default:\n");
    emitBodyLock(4, "break;\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(1, "}\n");
  }
  if (mtForceParallelRepCutBatches()) {
    bool hasParallelSafeBatch = false;
    for (const MtRepCutBatch& batch : semanticPlan.cutBatches) {
      if (batch.parallelSafe) hasParallelSafeBatch = true;
    }
    if (hasParallelSafeBatch) {
      emitBodyLock(1, "switch (beginCppId) {\n");
      for (const MtRepCutBatch& batch : semanticPlan.cutBatches) {
        if (batch.parallelSafe) emitBodyLock(2, "case %d:\n", batch.beginCppId);
      }
      emitBodyLock(3, "workerCount = mtConfiguredWorkerCount;\n");
      emitBodyLock(3, "mtSkippedBelowMinBatch = false;\n");
      emitBodyLock(3, "break;\n");
      emitBodyLock(2, "default:\n");
      emitBodyLock(3, "break;\n");
      emitBodyLock(1, "}\n");
    }
  }
  emitBodyLock(1, "if (workerCount > taskCount) workerCount = taskCount;\n");
  emitBodyLock(1, "if (workerCount < 2) workerCount = 1;\n");
  emitBodyLock(1, "if (mtProfileEnabled) {\n");
  emitBodyLock(2, "int batchSizeBucket = taskCount <= 1 ? 0 : (taskCount == 2 ? 1 : (taskCount <= 4 ? 2 : (taskCount <= 8 ? 3 : (taskCount <= 15 ? 4 : 5))));\n");
  emitBodyLock(2, "mtProfileBatchSizeHist[batchSizeBucket] ++;\n");
  emitBodyLock(2, "mtProfilePureBatchCount ++;\n");
  emitBodyLock(1, "}\n");
  if (!semanticPlan.batchPlan.batches.empty()) {
    emitBodyLock(1, "if (mtProfileEnabled) {\n");
    emitBodyLock(2, "switch (beginCppId) {\n");
    for (auto batch : semanticPlan.batchPlan.batches) {
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
  emitBodyLock(3, "if (!mtSkippedBelowMinBatch && !mtSkippedForcedSerialBatch) mtProfileRejectConfiguredSingleWorker ++;\n");
  emitBodyLock(2, "}\n");
  if (!semanticPlan.cutBatches.empty()) {
    emitBodyLock(2, "switch (beginCppId) {\n");
    for (const MtRepCutBatch& batch : semanticPlan.cutBatches) {
      uint64_t forcedSinkMask = mtRepCutForcedSinkMaskForBatch(semanticPlan, batch.beginCppId);
      if (forcedSinkMask != 0) {
        emitBodyLock(3, "case %d:\n", batch.beginCppId);
        emitBodyLock(4, "activeWord |= 0x%lx;\n", forcedSinkMask);
        emitBodyLock(4, "break;\n");
      }
    }
    emitBodyLock(3, "default:\n");
    emitBodyLock(4, "break;\n");
    emitBodyLock(2, "}\n");
  }
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
  if (!semanticPlan.cutBatches.empty()) {
    emitBodyLock(1, "switch (beginCppId) {\n");
    for (const MtRepCutBatch& batch : semanticPlan.cutBatches) {
      uint64_t forcedSinkMask = mtRepCutForcedSinkMaskForBatch(semanticPlan, batch.beginCppId);
      if (forcedSinkMask != 0) {
        emitBodyLock(2, "case %d:\n", batch.beginCppId);
        emitBodyLock(3, "for (int worker = 0; worker < workerCount; worker ++) mtWorkerFlags[worker] |= 0x%lx;\n", forcedSinkMask);
        emitBodyLock(3, "break;\n");
      }
    }
    emitBodyLock(2, "default:\n");
    emitBodyLock(3, "break;\n");
    emitBodyLock(1, "}\n");
  }
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

void graph::genMtCoarseRegionRunner(const MtRepCutSemanticPlan& semanticPlan, const MtCoarseRegionPlan& coarsePlan) {
  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapWithRepCutSelectionForInvocation();
  markMtRepCutLiteRuntimeApplied(mtTasks);
  bool waitProbeCodegen = mtUseWaitProbeCodegen();
  // 28c-2: shared emitter for `switch (mtaskIndex) { case M: <body>; break; ... }` body.
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
          if (mtTasks[cppId].repcutRuntimeApplied) {
            emitBodyLock(outerIndent + 4, "mtRepCutLiteTask%d(mtWorkerCoarseFlags[worker][%d], mtWorkerDeltas[worker]);\n", cppId, wordOffset);
          } else {
            emitBodyLock(outerIndent + 4, "mtTask%d(mtWorkerCoarseFlags[worker][%d], mtWorkerDeltas[worker]);\n", cppId, wordOffset);
          }
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
        if (mtTasks[cppId].repcutRuntimeApplied) {
          emitBodyLock(9, "mtRepCutLiteTask%d(mtWorkerCoarseFlags[worker][%d], mtWorkerDeltas[worker]);\n", cppId, wordOffset);
        } else {
          emitBodyLock(9, "mtTask%d(mtWorkerCoarseFlags[worker][%d], mtWorkerDeltas[worker]);\n", cppId, wordOffset);
        }
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
  // Track 2 Week 4: shared emitter for antichain mtask body switch.
  // Used by mtRunCoarseMTaskDynamic; walks region.antichainProbeGroups
  // in the same topo/layer order as the old mtask switch.
  auto emitAntichainMtaskInnerSwitch = [&](const MtCoarseRegion& region, int outerIndent) {
    emitBodyLock(outerIndent, "switch (mtaskIndex) {\n");
    for (size_t mtaskIdx = 0; mtaskIdx < region.antichainProbeGroups.size(); mtaskIdx ++) {
      const MtCoarseMTask& mtask = region.antichainProbeGroups[mtaskIdx];
      emitBodyLock(outerIndent + 1, "case %zu:\n", mtaskIdx);
      for (size_t layerIdx = 0; layerIdx < mtask.layerTaskCppIds.size(); layerIdx ++) {
        const std::vector<int>& taskCppIds = mtask.layerTaskCppIds[layerIdx];
        if (taskCppIds.empty()) continue;
        emitBodyLock(outerIndent + 2, "{\n");
        for (int cppId : taskCppIds) {
          int wordOffset = cppId / ACTIVE_WIDTH - region.beginActiveWord;
          uint64_t mask = (uint64_t)1 << (cppId % ACTIVE_WIDTH);
          emitBodyLock(outerIndent + 3, "if (mtWorkerCoarseFlags[worker][%d] & 0x%lx) {\n", wordOffset, mask);
          if (mtTasks[cppId].repcutRuntimeApplied) {
            emitBodyLock(outerIndent + 4, "mtRepCutLiteTask%d(mtWorkerCoarseFlags[worker][%d], mtWorkerDeltas[worker]);\n", cppId, wordOffset);
          } else {
            emitBodyLock(outerIndent + 4, "mtTask%d(mtWorkerCoarseFlags[worker][%d], mtWorkerDeltas[worker]);\n", cppId, wordOffset);
          }
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
  // Track 2 Week 7: mutex-protected ready queue for antichain scheduler.
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
  {
    int regionIndex = 0;
    for (const MtCoarseRegion& region : coarsePlan.regions) {
      if (!region.runtimeEligible) continue;
      if (!region.useAntichainRuntime) {
        regionIndex ++;
        continue;
      }
      emitBodyLock(2, "case %d:\n", regionIndex);
      int antichainMTaskCount = static_cast<int>(region.antichainProbeGroups.size());
      emitBodyLock(3, "{\n");
      emitBodyLock(3, "static const int kMTaskCount = %d;\n", antichainMTaskCount);
      emitBodyLock(3, "static const int kBeginActiveWord = %d;\n", region.beginActiveWord);
      emitBodyLock(3, "static const int kActiveWordSpan = %d;\n", region.activeWordSpan);
      std::vector<int> upstreamCounts;
      std::vector<int> succOffsets;
      std::vector<int> succIndices;
      std::vector<int> workerZeroOnlyFlags;
      upstreamCounts.reserve(antichainMTaskCount);
      workerZeroOnlyFlags.reserve(antichainMTaskCount);
      succOffsets.push_back(0);
      for (const MtCoarseMTask& mtask : region.antichainProbeGroups) {
        upstreamCounts.push_back(mtask.upstreamDepCount);
        workerZeroOnlyFlags.push_back(mtask.workerZeroOnly ? 1 : 0);
        for (int succ : mtask.succMTaskIndices) succIndices.push_back(succ);
        succOffsets.push_back(static_cast<int>(succIndices.size()));
      }
      emitBodyLock(3, "static const int kUpstream[%d] = {%s};\n", antichainMTaskCount, mtJoinIntList(upstreamCounts).c_str());
      emitBodyLock(3, "static const int kSuccOffset[%d] = {%s};\n", antichainMTaskCount + 1, mtJoinIntList(succOffsets).c_str());
      emitBodyLock(3, "static const int kSuccIndices[%zu] = {%s};\n", succIndices.size(), mtJoinIntList(succIndices).c_str());
      emitBodyLock(3, "static const bool kWorkerZeroOnly[%d] = {%s};\n", antichainMTaskCount, mtJoinIntList(workerZeroOnlyFlags).c_str());
      emitBodyLock(3, "uint64_t cycle = mtCoarseRegionCycle[regionIndex][0].load(std::memory_order_acquire);\n");
      emitBodyLock(3, "bool evenCycle = (cycle % 2 == 0);\n");
      emitBodyLock(3, "if (!mtCoarseUseAntichainQueue) {\n");
      // Legacy scan-based dispatch: keep for bisection / NEMU mismatch debugging.
      emitBodyLock(4, "while (mtCoarseMTaskRemaining.load(std::memory_order_acquire) > 0) {\n");
      emitBodyLock(5, "int found = -1;\n");
      emitBodyLock(5, "for (int m = 0; m < kMTaskCount; m ++) {\n");
      emitBodyLock(6, "if (kWorkerZeroOnly[m] && worker != 0) continue;\n");
      emitBodyLock(6, "uint64_t claimed = mtCoarseMTaskClaimGen[regionIndex][m].load(std::memory_order_relaxed);\n");
      emitBodyLock(6, "if (claimed == cycle) continue;\n");
      emitBodyLock(6, "int target = evenCycle ? 0 : kUpstream[m];\n");
      emitBodyLock(6, "if (mtCoarseMTaskUpstream[regionIndex][m].load(std::memory_order_acquire) != target) continue;\n");
      emitBodyLock(6, "if (mtCoarseMTaskClaimGen[regionIndex][m].compare_exchange_strong(claimed, cycle, std::memory_order_acquire)) { found = m; break; }\n");
      emitBodyLock(5, "}\n");
      emitBodyLock(5, "if (found < 0) {\n");
      emitBodyLock(6, "if (mtCoarseMTaskRemaining.load(std::memory_order_acquire) == 0) break;\n");
      emitBodyLock(6, "mtWorkerPoolPause();\n");
      emitBodyLock(6, "continue;\n");
      emitBodyLock(5, "}\n");
      emitBodyLock(5, "for (int w = 0; w < kActiveWordSpan; w ++) {\n");
      emitBodyLock(6, "mtWorkerCoarseFlags[worker][w] = mtWorkerPoolCoarseActiveWords[w] | mtCoarseRegionSharedFlags[regionIndex][w].load(std::memory_order_acquire);\n");
      emitBodyLock(5, "}\n");
      emitBodyLock(5, "int mtaskIndex = found;\n");
      emitAntichainMtaskInnerSwitch(region, 5);
      emitBodyLock(5, "for (int w = 0; w < kActiveWordSpan; w ++) {\n");
      emitBodyLock(6, "mtCoarseRegionSharedFlags[regionIndex][w].fetch_or(mtWorkerCoarseFlags[worker][w], std::memory_order_release);\n");
      emitBodyLock(5, "}\n");
      emitBodyLock(5, "mtCoarseMTaskRemaining.fetch_sub(1, std::memory_order_relaxed);\n");
      emitBodyLock(5, "for (int s = kSuccOffset[found]; s < kSuccOffset[found + 1]; s ++) {\n");
      emitBodyLock(6, "int succ = kSuccIndices[s];\n");
      emitBodyLock(6, "if (evenCycle) {\n");
      emitBodyLock(7, "mtCoarseMTaskUpstream[regionIndex][succ].fetch_sub(1, std::memory_order_acq_rel);\n");
      emitBodyLock(6, "} else {\n");
      emitBodyLock(7, "mtCoarseMTaskUpstream[regionIndex][succ].fetch_add(1, std::memory_order_acq_rel);\n");
      emitBodyLock(6, "}\n");
      emitBodyLock(5, "}\n");
      emitBodyLock(4, "}\n");
      emitBodyLock(3, "} else {\n");
      // Ready-queue dispatch: each mtask is pushed once (when ready) and popped once.
      emitBodyLock(4, "while (true) {\n");
      emitBodyLock(5, "int found = mtCoarseReadyQueuePop(regionIndex, worker);\n");
      emitBodyLock(5, "if (found < 0) {\n");
      emitBodyLock(6, "if (mtCoarseMTaskRemaining.load(std::memory_order_acquire) == 0) {\n");
      emitBodyLock(7, "std::lock_guard<std::mutex> lock(mtCoarseReadyQueueMutex);\n");
      emitBodyLock(7, "if (mtCoarseReadyQueueParallel[regionIndex].empty() && mtCoarseReadyQueueWorker0[regionIndex].empty()) break;\n");
      emitBodyLock(6, "}\n");
      emitBodyLock(6, "mtWorkerPoolPause();\n");
      emitBodyLock(6, "continue;\n");
      emitBodyLock(5, "}\n");
      emitBodyLock(5, "mtCoarseMTaskInFlight.fetch_add(1, std::memory_order_relaxed);\n");
      emitBodyLock(5, "for (int w = 0; w < kActiveWordSpan; w ++) {\n");
      emitBodyLock(6, "mtWorkerCoarseFlags[worker][w] = mtWorkerPoolCoarseActiveWords[w] | mtCoarseRegionSharedFlags[regionIndex][w].load(std::memory_order_acquire);\n");
      emitBodyLock(5, "}\n");
      emitBodyLock(5, "int mtaskIndex = found;\n");
      emitAntichainMtaskInnerSwitch(region, 5);
      emitBodyLock(5, "for (int w = 0; w < kActiveWordSpan; w ++) {\n");
      emitBodyLock(6, "mtCoarseRegionSharedFlags[regionIndex][w].fetch_or(mtWorkerCoarseFlags[worker][w], std::memory_order_release);\n");
      emitBodyLock(5, "}\n");
      emitBodyLock(5, "for (int s = kSuccOffset[found]; s < kSuccOffset[found + 1]; s ++) {\n");
      emitBodyLock(6, "int succ = kSuccIndices[s];\n");
      emitBodyLock(6, "bool ready = false;\n");
      emitBodyLock(6, "if (evenCycle) {\n");
      emitBodyLock(7, "int old = mtCoarseMTaskUpstream[regionIndex][succ].fetch_sub(1, std::memory_order_acq_rel);\n");
      emitBodyLock(7, "ready = (old == 1);\n");
      emitBodyLock(6, "} else {\n");
      emitBodyLock(7, "int old = mtCoarseMTaskUpstream[regionIndex][succ].fetch_add(1, std::memory_order_acq_rel);\n");
      emitBodyLock(7, "ready = (old == kUpstream[succ] - 1);\n");
      emitBodyLock(6, "}\n");
      emitBodyLock(6, "if (ready) mtCoarseReadyQueuePush(regionIndex, succ, kWorkerZeroOnly[succ]);\n");
      emitBodyLock(5, "}\n");
      emitBodyLock(5, "mtCoarseMTaskRemaining.fetch_sub(1, std::memory_order_relaxed);\n");
      emitBodyLock(5, "mtCoarseMTaskInFlight.fetch_sub(1, std::memory_order_relaxed);\n");
      emitBodyLock(4, "}\n");
      emitBodyLock(3, "}\n");
      emitBodyLock(3, "}\n");
      emitBodyLock(3, "break;\n");
      regionIndex ++;
    }
  }
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

  // 28c D-static Step 1: codegen-time LPT + flat per-cppId arrays.
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
    static int w0Penalty = []() {
      const char* env = std::getenv("GSIM_MT_W0_PENALTY");
      if (env == nullptr || env[0] == '\0') return 0;
      int value = std::atoi(env);
      return value > 0 ? value : 0;
    }();
    std::vector<int> staticCosts(wc, 0);
    if (wc > 1 && w0Penalty > 0) staticCosts[0] = w0Penalty;
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
        struct DEntry { int cppId; int wordOffset; uint64_t mask; bool isRepCut; bool mergeAfter; };
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
              e.isRepCut = mtTasks[cppId].repcutRuntimeApplied;
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
          emitBodyLock(6, "{%d, %d, %d, 0, 0x%lxULL, &S%s::%s%d},\n",
                       e.cppId, e.wordOffset, e.mergeAfter ? 1 : 0,
                       (unsigned long)e.mask,
                       name.c_str(),
                       e.isRepCut ? "mtRepCutLiteTask" : "mtTask",
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
  // Track 2 Week 4: antichain runtime constant arrays.
  {
    std::vector<int> useAntichainRuntimeValues;
    std::vector<int> antichainMTaskCountValues;
    std::vector<int> antichainUpstreamOffsets;
    std::vector<int> antichainUpstreamValues;
    std::vector<int> antichainWorker0OnlyValues;
    antichainUpstreamOffsets.push_back(0);
    for (const MtCoarseRegion& region : coarsePlan.regions) {
      if (!region.runtimeEligible) continue;
      bool useAntichain = region.useAntichainRuntime;
      useAntichainRuntimeValues.push_back(useAntichain ? 1 : 0);
      int antichainCount = useAntichain ? static_cast<int>(region.antichainProbeGroups.size()) : 0;
      antichainMTaskCountValues.push_back(antichainCount);
      if (useAntichain) {
        for (const MtCoarseMTask& mtask : region.antichainProbeGroups) {
          antichainUpstreamValues.push_back(mtask.upstreamDepCount);
          antichainWorker0OnlyValues.push_back(mtask.workerZeroOnly ? 1 : 0);
        }
      }
      antichainUpstreamOffsets.push_back(static_cast<int>(antichainUpstreamValues.size()));
    }
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
      emitBodyLock(4, "// Track 2 Week 5: mirror the stricter codegen-time admission gate.\n");
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
  // 28c-2: runtime profitability gate. Pop-count actual active bits in this
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
  // Track 2 Week 4: atomic-counter antichain runtime. Single-threaded init,
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
    // Track 2 Week 7: seed the antichain ready queue with source mtasks (zero upstream deps).
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
  // 28c-2: mtask runtime is a runtime branch (env GSIM_MT_COARSE_RUNTIME=mtask|layered);
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
  // 28c D-static Step 1: when mtCoarseUseDStatic is set, replace the
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
  if (waitProbeCodegen) emitBodyLock(3, "if (mtWaitProbeEnabled) mtWaitProbePostTp = std::chrono::steady_clock::now();\n");
  emitBodyLock(3, "mtWorkerPoolPost();\n");
  emitBodyLock(3, "mtRunCoarseRegionStaticDispatch(regionIndex, dstaticRoundedWC, 0, regionBeginActiveWord, regionActiveWordSpan);\n");
  if (waitProbeCodegen) emitBodyLock(3, "if (mtWaitProbeEnabled && !mtWaitProbeWorkerFinishNs.empty()) mtWaitProbeWorkerFinishNs[0] = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtWaitProbePostTp).count();\n");
  emitBodyLock(3, "std::chrono::steady_clock::time_point mtPhaseWaitBegin;\n");
  emitBodyLock(3, "if (mtProfileEnabled) {\n");
  emitBodyLock(4, "mtPhaseWaitBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(4, "mtProfileCoarseBodyNs += std::chrono::duration_cast<std::chrono::nanoseconds>(mtPhaseWaitBegin - mtPhaseBodyBegin).count();\n");
  emitBodyLock(3, "}\n");
  emitBodyLock(3, "mtWorkerPoolWaitForDone(dstaticRoundedWC - 1);\n");
  emitBodyLock(3, "if (mtProfileEnabled) mtProfileCoarseWaitNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtPhaseWaitBegin).count();\n");
  if (waitProbeCodegen) {
    emitBodyLock(3, "if (mtWaitProbeEnabled) {\n");
    emitBodyLock(4, "uint64_t mtwpWaitDone = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtWaitProbePostTp).count();\n");
    emitBodyLock(4, "uint64_t mtwpW0 = mtWaitProbeWorkerFinishNs[0];\n");
    emitBodyLock(4, "uint64_t mtwpMaxFinish = 0, mtwpMaxBg = 0, mtwpMinBg = (uint64_t)-1; int mtwpLast = 0;\n");
    emitBodyLock(4, "for (int w = 0; w < dstaticRoundedWC; w ++) {\n");
    emitBodyLock(5, "uint64_t f = mtWaitProbeWorkerFinishNs[(size_t)w];\n");
    emitBodyLock(5, "mtWaitProbeWorkerFinishSumNs[(size_t)w] += f;\n");
    emitBodyLock(5, "if (f > mtwpMaxFinish) { mtwpMaxFinish = f; mtwpLast = w; }\n");
    emitBodyLock(5, "if (w >= 1) { if (f > mtwpMaxBg) mtwpMaxBg = f; if (f < mtwpMinBg) mtwpMinBg = f; }\n");
    emitBodyLock(4, "}\n");
    emitBodyLock(4, "if (dstaticRoundedWC <= 1) { mtwpMaxBg = 0; mtwpMinBg = 0; }\n");
    emitBodyLock(4, "mtWaitProbeDispatchCount ++;\n");
    emitBodyLock(4, "mtWaitProbeW0BodySumNs += mtwpW0;\n");
    emitBodyLock(4, "mtWaitProbeWaitSumNs += (mtwpWaitDone >= mtwpW0 ? mtwpWaitDone - mtwpW0 : 0);\n");
    emitBodyLock(4, "mtWaitProbeMaxFinishSumNs += mtwpMaxFinish;\n");
    emitBodyLock(4, "mtWaitProbeMinBgFinishSumNs += mtwpMinBg;\n");
    emitBodyLock(4, "mtWaitProbeTailBeyondW0SumNs += (mtwpMaxBg > mtwpW0 ? mtwpMaxBg - mtwpW0 : 0);\n");
    emitBodyLock(4, "if (mtwpW0 >= mtwpMaxBg) mtWaitProbeWorker0LastCount ++;\n");
    emitBodyLock(4, "if ((size_t)mtwpLast < mtWaitProbeWorkerLastHist.size()) mtWaitProbeWorkerLastHist[(size_t)mtwpLast] ++;\n");
    emitBodyLock(3, "}\n");
  }
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
    std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapWithRepCutForInvocation();
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
    std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapWithRepCutSelectionForInvocation();
    markMtRepCutLiteRuntimeApplied(mtTasks);
    MtRepCutSemanticPlan semanticPlan = planMtRepCutSemantics(mtTasks);
    MtCoarseRegionPlan coarsePlan = planMtCoarseRegionsForInvocation(mtTasks);
    MtPureBatchPlan batchPlan = semanticPlan.batchPlan;
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
    for (int idx = 0; idx < superId; idx ++) {
      genMtTaskHelper(cppId2Super[idx], true, "ActivationDelta");
      genMtTaskHelper(cppId2Super[idx], false, "ActivationDelta");
    }
    for (int idx = 0; idx < superId; idx ++) {
      if (mtTasks[idx].repcutRuntimeApplied) genMtRepCutLiteTaskHelper(cppId2Super[idx], mtRepCutClonesForSink(semanticPlan, idx), "ActivationDelta");
    }
    genMtTaskRunner(semanticPlan);
    if (globalConfig.MtBatchFormationMode == "coarse") genMtCoarseRegionRunner(semanticPlan, coarsePlan);

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
        if (!profileOffDirectSerial || mtUseSubchunkProbe()) {
          emitBodyLock(indent, "if (mtProfileEnabled) {\n");
          for (int word = 0; word < region.activeWordSpan; word ++) {
            emitBodyLock(indent + 1, "if (mtCoarseWords%d[%d] != 0) mtProfileActiveWordCount ++;\n", idx, word);
          }
          emitBodyLock(indent, "}\n");
        }
        // A43/A44: default-off/env-on direct serial fallback before mtRunCoarseRegion.
        // Clean regions only: original mtTaskN(flag) bodies and only pure_compute plus
        // the explicit A44 safe-serial allowlist. RepCut cloned helpers are not needed
        // on this single-threaded fallback path; unbuffered mtTaskN/genSuperEval keeps
        // same-word activation in the local flag and cross-word activation in activeFlags.
        // This is the ST-parity floor path: it bypasses worker flag copies, post/wait,
        // SCoarseTaskRef member-pointer dispatch, mergeAfter, and final merge.
        bool regionHasRepcut = false;
        bool regionHasNonPure = false;
        for (int rcid = region.beginCppId; rcid < region.endCppId; rcid ++) {
          auto mtIter = mtTasks.find(rcid);
          if (mtIter == mtTasks.end() || hasWorker0OnlyReason(mtIter->second.serialReasons) || !hasOnlyA44DirectFallbackReasons(mtIter->second.serialReasons)) regionHasNonPure = true;
          if (mtIter != mtTasks.end() && mtIter->second.repcutRuntimeApplied) regionHasRepcut = true;
        }
        bool regionCleanSerialFallback = !regionHasRepcut && !regionHasNonPure;
        auto emitCoarseInlineWord = [&](int word, int wordIndent, bool declareFlag = true) {
          int activeWord = region.beginActiveWord + word;
          if (declareFlag) {
            emitBodyLock(wordIndent, "uint%d_t coarseInlineFlag%d_%d = mtCoarseWords%d[%d] | activeFlags[%d];\n",
                         ACTIVE_WIDTH, idx, word, idx, word, activeWord);
          }
          if (mtUseSubchunkProbe()) {
            emitBodyLock(wordIndent ++, "if (mtProfileEnabled && coarseInlineFlag%d_%d != 0) {\n", idx, word);
            emitBodyLock(wordIndent, "mtProfileCoarseSerialFallbackDynamicWords ++;\n");
            emitBodyLock(--wordIndent, "}\n");
          }
          if (mtUseSubchunkProbe()) {
            emitBodyLock(wordIndent, "int subchunkProbeWordTasks%d_%d = 0;\n", idx, word);
            emitBodyLock(wordIndent, "int subchunkProbeWordStaticCost%d_%d = 0;\n", idx, word);
          }
          auto emitCoarseInlineTask = [&](int cppId, int taskIndent) {
            if (mtUseSubchunkProbe()) {
              emitBodyLock(taskIndent ++, "if (mtProfileEnabled) {\n");
              emitBodyLock(taskIndent, "mtProfileCoarseSerialFallbackDynamicTasks ++;\n");
              emitBodyLock(taskIndent, "mtProfileCoarseSerialFallbackDynamicTaskStaticCost += %d;\n", mtTaskEstimatedCost(mtTasks, cppId));
              emitBodyLock(taskIndent, "subchunkProbeWordTasks%d_%d ++;\n", idx, word);
              emitBodyLock(taskIndent, "subchunkProbeWordStaticCost%d_%d += %d;\n", idx, word, mtTaskEstimatedCost(mtTasks, cppId));
              emitBodyLock(--taskIndent, "}\n");
            }
            if (profileOffDirectSerial && !mtUseSubchunkProbe()) {
              if (directInlineFallback) {
                genSuperEval(cppId2Super[cppId], format("coarseInlineFlag%d_%d", idx, word), "", taskIndent, true);
              } else {
                emitBodyLock(taskIndent, "mtTask%d(coarseInlineFlag%d_%d);\n", cppId, idx, word);
              }
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
          if (mtUseCtzCoarseInlineWord()) {
            emitBodyLock(wordIndent, "uint32_t coarseInlineTodo%d_%d = (uint32_t)coarseInlineFlag%d_%d;\n", idx, word, idx, word);
            emitBodyLock(wordIndent ++, "while (coarseInlineTodo%d_%d) {\n", idx, word);
            emitBodyLock(wordIndent, "int coarseInlineBit%d_%d = __builtin_ctz(coarseInlineTodo%d_%d);\n", idx, word, idx, word);
            emitBodyLock(wordIndent, "coarseInlineTodo%d_%d &= coarseInlineTodo%d_%d - 1u;\n", idx, word, idx, word);
            emitBodyLock(wordIndent ++, "switch (coarseInlineBit%d_%d) {\n", idx, word);
            for (int bit = 0; bit < ACTIVE_WIDTH; bit ++) {
              int cppId = activeWord * ACTIVE_WIDTH + bit;
              if (cppId >= region.endCppId) break;
              if (cppId < region.beginCppId) continue;
              emitBodyLock(wordIndent ++, "case %d: {\n", bit);
              emitCoarseInlineTask(cppId, wordIndent);
              emitBodyLock(wordIndent, "break;\n");
              emitBodyLock(--wordIndent, "}\n");
            }
            emitBodyLock(--wordIndent, "}\n");
            emitBodyLock(wordIndent, "coarseInlineTodo%d_%d = (coarseInlineTodo%d_%d | (uint32_t)coarseInlineFlag%d_%d) & (~((1u << (coarseInlineBit%d_%d + 1)) - 1u) & 0x%xu);\n", idx, word, idx, word, idx, word, idx, word, (1u << ACTIVE_WIDTH) - 1u);
            emitBodyLock(--wordIndent, "}\n");
          } else {
            auto emitFixedBitScan = [&](int scanIndent, int minBit) {
              for (int bit = 0; bit < ACTIVE_WIDTH; bit ++) {
                int cppId = activeWord * ACTIVE_WIDTH + bit;
                if (cppId >= region.endCppId) break;
                if (cppId < region.beginCppId) continue;
                if (minBit >= 0) {
                  emitBodyLock(scanIndent ++, "if (%d > coarseInlineFirstBit%d_%d && (coarseInlineFlag%d_%d & 0x%lx)) {\n", bit, idx, word, idx, word, (uint64_t)1 << bit);
                } else {
                  emitBodyLock(scanIndent ++, "if (coarseInlineFlag%d_%d & 0x%lx) {\n", idx, word, (uint64_t)1 << bit);
                }
                emitCoarseInlineTask(cppId, scanIndent);
                emitBodyLock(--scanIndent, "}\n");
              }
            };
            if (mtUseSingleBitCoarseInlineWord()) {
              emitBodyLock(wordIndent ++, "if ((coarseInlineFlag%d_%d & (coarseInlineFlag%d_%d - 1)) == 0) {\n", idx, word, idx, word);
              emitBodyLock(wordIndent, "int coarseInlineFirstBit%d_%d = __builtin_ctz((uint32_t)coarseInlineFlag%d_%d);\n", idx, word, idx, word);
              emitBodyLock(wordIndent ++, "switch (coarseInlineFirstBit%d_%d) {\n", idx, word);
              for (int bit = 0; bit < ACTIVE_WIDTH; bit ++) {
                int cppId = activeWord * ACTIVE_WIDTH + bit;
                if (cppId >= region.endCppId) break;
                if (cppId < region.beginCppId) continue;
                emitBodyLock(wordIndent ++, "case %d: {\n", bit);
                emitCoarseInlineTask(cppId, wordIndent);
                emitBodyLock(wordIndent, "break;\n");
                emitBodyLock(--wordIndent, "}\n");
              }
              emitBodyLock(--wordIndent, "}\n");
              emitFixedBitScan(wordIndent, 0);
              emitBodyLock(--wordIndent, "} else {\n");
              emitFixedBitScan(wordIndent, -1);
              emitBodyLock(--wordIndent, "}\n");
            } else {
              emitFixedBitScan(wordIndent, -1);
            }
          }
          emitBodyLock(--wordIndent, "}\n");
          if (mtUseSubchunkProbe()) {
            emitBodyLock(wordIndent ++, "if (mtProfileEnabled) {\n");
            emitBodyLock(wordIndent, "int subchunkProbeCostBucket%d_%d = subchunkProbeWordStaticCost%d_%d < 64 ? 0 : (subchunkProbeWordStaticCost%d_%d < 128 ? 1 : (subchunkProbeWordStaticCost%d_%d < 256 ? 2 : (subchunkProbeWordStaticCost%d_%d < 512 ? 3 : (subchunkProbeWordStaticCost%d_%d < 1024 ? 4 : 5))));\n", idx, word, idx, word, idx, word, idx, word, idx, word, idx, word);
            emitBodyLock(wordIndent, "int subchunkProbeTaskBucket%d_%d = subchunkProbeWordTasks%d_%d <= 1 ? 0 : (subchunkProbeWordTasks%d_%d == 2 ? 1 : (subchunkProbeWordTasks%d_%d <= 4 ? 2 : (subchunkProbeWordTasks%d_%d <= 8 ? 3 : (subchunkProbeWordTasks%d_%d <= 15 ? 4 : 5))));\n", idx, word, idx, word, idx, word, idx, word, idx, word, idx, word);
            emitBodyLock(wordIndent, "mtProfileCoarseSerialFallbackDynamicWordCostHist[subchunkProbeCostBucket%d_%d] ++;\n", idx, word);
            emitBodyLock(wordIndent, "mtProfileCoarseSerialFallbackDynamicWordTaskHist[subchunkProbeTaskBucket%d_%d] ++;\n", idx, word);
            emitBodyLock(wordIndent, "if (subchunkProbeWordStaticCost%d_%d >= 64) mtProfileCoarseSerialFallbackDynamicWordCostGe64 ++;\n", idx, word);
            emitBodyLock(wordIndent, "if (subchunkProbeWordStaticCost%d_%d >= 128) mtProfileCoarseSerialFallbackDynamicWordCostGe128 ++;\n", idx, word);
            emitBodyLock(wordIndent, "if (subchunkProbeWordStaticCost%d_%d >= 256) mtProfileCoarseSerialFallbackDynamicWordCostGe256 ++;\n", idx, word);
            emitBodyLock(wordIndent, "if (subchunkProbeWordTasks%d_%d >= 2) mtProfileCoarseSerialFallbackDynamicWordTasksGe2 ++;\n", idx, word);
            emitBodyLock(--wordIndent, "}\n");
          }
        };
        auto emitCoarseInlineSavedProfile = [&](int profileIndent) {
          if (profileOffDirectSerial && !mtUseSubchunkProbe()) return;
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
          if (mtUseStaticCoarseInlineBound() && profileOffDirectSerial && !mtUseSubchunkProbe()) {
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
          if (!profileOffDirectSerial || mtUseSubchunkProbe()) emitBodyLock(indent, "if (mtProfileEnabled) mtProfileCoarseSerialFallbackEligible ++;\n");
          emitBodyLock(indent ++, "if (coarseInlineActiveBits%d <= mtCoarseInlineThreshold) {\n", idx);
          if (!mtUseSubchunkRuntime()) {
            emitCoarseInlineSavedProfile(indent);
            for (int word = 0; word < region.activeWordSpan; word ++) emitCoarseInlineWord(word, indent);
          } else {
          // Default-off subchunk dispatch must not sit before the hot clean
          // serial-inline fallback in generated code: the cold block is large
          // enough to perturb the default model's code layout even when the
          // runtime cost knob is zero.
          emitBodyLock(indent ++, "if (likely(mtSubchunkDispatchCost <= 0)) {\n");
          emitCoarseInlineSavedProfile(indent);
          for (int word = 0; word < region.activeWordSpan; word ++) emitCoarseInlineWord(word, indent);
          emitBodyLock(--indent, "} else {\n");
          if (regionHasRepcut) {
            indent ++;
            emitCoarseDispatchAndMerge(indent);
            emitBodyLock(--indent, "}\n");
          } else {
          indent ++;
          // Prefix-hybrid fallback: decide the cut dynamically in word order.
          // Each candidate word observes activations produced by earlier inlined
          // prefix words via activeFlags. The cost gate intentionally uses the
          // pre-task word flag; same-word activations produced while executing
          // the word stay in the normal by-reference serial semantics.
          emitBodyLock(indent, "bool coarseSubchunkDispatch%d = false;\n", idx);
          emitBodyLock(indent, "int coarseSubchunkCut%d = %d;\n", idx, region.activeWordSpan);
          emitBodyLock(indent, "uint64_t coarseSubchunkInlineActiveBits%d = 0;\n", idx);
          for (int word = 0; word < region.activeWordSpan; word ++) {
            int activeWord = region.beginActiveWord + word;
            emitBodyLock(indent ++, "if (!coarseSubchunkDispatch%d) {\n", idx);
            emitBodyLock(indent, "uint%d_t coarseInlineFlag%d_%d = mtCoarseWords%d[%d] | activeFlags[%d];\n", ACTIVE_WIDTH, idx, word, idx, word, activeWord);
            emitBodyLock(indent, "int coarseSubchunkWordCost%d_%d = 0;\n", idx, word);
            for (int bit = 0; bit < ACTIVE_WIDTH; bit ++) {
              int cppId = activeWord * ACTIVE_WIDTH + bit;
              if (cppId >= region.endCppId) break;
              if (cppId < region.beginCppId) continue;
              emitBodyLock(indent, "if (coarseInlineFlag%d_%d & 0x%lx) coarseSubchunkWordCost%d_%d += %d;\n", idx, word, (uint64_t)1 << bit, idx, word, mtTaskEstimatedCost(mtTasks, cppId));
            }
            emitBodyLock(indent ++, "if (coarseInlineFlag%d_%d != 0 && coarseSubchunkWordCost%d_%d >= mtSubchunkDispatchCost) {\n", idx, word, idx, word);
            emitBodyLock(indent, "coarseSubchunkDispatch%d = true;\n", idx);
            emitBodyLock(indent, "coarseSubchunkCut%d = %d;\n", idx, word);
            emitBodyLock(--indent, "} else {\n");
            emitBodyLock(indent, "if (mtProfileEnabled) coarseSubchunkInlineActiveBits%d += (uint64_t)__builtin_popcountll((unsigned long long)coarseInlineFlag%d_%d);\n", idx, idx, word);
            emitCoarseInlineWord(word, indent, false);
            emitBodyLock(--indent, "}\n");
            emitBodyLock(--indent, "}\n");
          }
          emitBodyLock(indent, "if (mtProfileEnabled) mtProfileCoarseSubchunkDispatchEligible ++;\n");
          emitBodyLock(indent ++, "if (coarseSubchunkDispatch%d) {\n", idx);
          emitBodyLock(indent, "int coarseSubchunkResidualActiveBits%d = 0;\n", idx);
          for (int word = 0; word < region.activeWordSpan; word ++) {
            int activeWord = region.beginActiveWord + word;
            emitBodyLock(indent ++, "if (coarseSubchunkCut%d > %d) {\n", idx, word);
            emitBodyLock(indent, "mtCoarseWords%d[%d] = 0;\n", idx, word);
            emitBodyLock(--indent, "} else {\n");
            emitBodyLock(indent, "mtCoarseWords%d[%d] |= activeFlags[%d];\n", idx, word, activeWord);
            emitBodyLock(indent, "coarseSubchunkResidualActiveBits%d += __builtin_popcountll((unsigned long long)mtCoarseWords%d[%d]);\n", idx, idx, word);
            emitBodyLock(indent, "if (mtProfileEnabled) mtProfileCoarseSubchunkDispatchResidualInitialActiveBits += (uint64_t)__builtin_popcountll((unsigned long long)mtCoarseWords%d[%d]);\n", idx, word);
            emitBodyLock(indent, "activeFlags[%d] = 0;\n", activeWord);
            emitBodyLock(--indent, "}\n");
          }
          emitBodyLock(indent ++, "if (mtSubchunkMinActiveBits > 0 && coarseSubchunkResidualActiveBits%d < mtSubchunkMinActiveBits) {\n", idx);
          emitBodyLock(indent ++, "if (mtProfileEnabled) {\n");
          emitBodyLock(indent, "mtProfileCoarseSubchunkDispatchSkippedBelowMinActive ++;\n");
          emitBodyLock(indent, "mtProfileCoarseSubchunkDispatchInlineWords += (uint64_t)%d;\n", region.activeWordSpan);
          emitBodyLock(indent, "mtProfileCoarseSubchunkDispatchInlineActiveBits += coarseSubchunkInlineActiveBits%d;\n", idx);
          emitBodyLock(--indent, "}\n");
          for (int word = 0; word < region.activeWordSpan; word ++) {
            int activeWord = region.beginActiveWord + word;
            emitBodyLock(indent ++, "if (coarseSubchunkCut%d <= %d) {\n", idx, word);
            emitBodyLock(indent, "uint%d_t coarseInlineFlag%d_%d = mtCoarseWords%d[%d] | activeFlags[%d];\n", ACTIVE_WIDTH, idx, word, idx, word, activeWord);
            emitBodyLock(indent, "if (mtProfileEnabled) mtProfileCoarseSubchunkDispatchInlineActiveBits += (uint64_t)__builtin_popcountll((unsigned long long)coarseInlineFlag%d_%d);\n", idx, word);
            emitCoarseInlineWord(word, indent, false);
            emitBodyLock(--indent, "}\n");
          }
          emitBodyLock(--indent, "} else {\n");
          emitBodyLock(indent ++, "if (mtProfileEnabled) {\n");
          emitBodyLock(indent, "mtProfileCoarseSubchunkDispatchTaken ++;\n");
          emitBodyLock(indent, "mtProfileCoarseSubchunkDispatchResidualDispatches ++;\n");
          emitBodyLock(indent, "mtProfileCoarseSubchunkDispatchInlineWords += (uint64_t)coarseSubchunkCut%d;\n", idx);
          emitBodyLock(indent, "mtProfileCoarseSubchunkDispatchInlineActiveBits += coarseSubchunkInlineActiveBits%d;\n", idx);
          emitBodyLock(--indent, "}\n");
          emitCoarseDispatchAndMerge(indent);
          emitBodyLock(--indent, "}\n");
          emitBodyLock(--indent, "} else {\n");
          indent ++;
          emitBodyLock(indent ++, "if (mtProfileEnabled) {\n");
          emitBodyLock(indent, "mtProfileCoarseSubchunkDispatchFullyInlined ++;\n");
          emitBodyLock(indent, "mtProfileCoarseSubchunkDispatchInlineWords += (uint64_t)%d;\n", region.activeWordSpan);
          emitBodyLock(indent, "mtProfileCoarseSubchunkDispatchInlineActiveBits += coarseSubchunkInlineActiveBits%d;\n", idx);
          emitBodyLock(--indent, "}\n");
          emitBodyLock(--indent, "}\n");
          emitBodyLock(--indent, "}\n");
          }
          }
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
          if (regionHasRepcut) emitBodyLock(indent, "if (mtProfileEnabled && mtCoarseInlineThreshold > 0) mtProfileCoarseSerialFallbackRepcutExcluded ++;\n");
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
            uint64_t forcedSinkMask = mtRepCutForcedSinkMaskForBatch(semanticPlan, idx);
            if (forcedSinkMask != 0) emitBodyLock(indent, "oldFlag |= 0x%lx;\n", forcedSinkMask);
            uint64_t batchActiveMask = forcedSinkMask;
            for (int batchCppId = idx; batchCppId < batchEnd; batchCppId ++) {
              batchActiveMask |= (uint64_t)1 << (batchCppId % ACTIVE_WIDTH);
            }
            if (inlineSmallPureBatchMaskGuard) emitBodyLock(indent ++, "if (unlikely(oldFlag & 0x%lx)) {\n", batchActiveMask);
            for (int batchCppId = idx; batchCppId < batchEnd; batchCppId ++) {
              uint64_t batchMask = (uint64_t)1 << (batchCppId % ACTIVE_WIDTH);
              emitBodyLock(indent ++, "if (unlikely(oldFlag & 0x%lx)) {\n", batchMask);
              if (mtTasks[batchCppId].repcutRuntimeApplied) {
                emitBodyLock(indent ++, "{\n");
                emitBodyLock(indent, "ActivationDelta mtInlineBatchDelta%d;\n", batchCppId);
                emitBodyLock(indent, "mtRepCutLiteTask%d(oldFlag, mtInlineBatchDelta%d);\n", batchCppId, batchCppId);
                emitBodyLock(indent, "mtInlineBatchDelta%d.mergeInto(activeFlags);\n", batchCppId);
                emitBodyLock(--indent, "}\n");
              } else if (inlineSmallPureBatchBodies) {
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
                                    !directInlineSerialIter->second.repcutRuntimeApplied &&
                                    !hasWorker0OnlyReason(directInlineSerialIter->second.serialReasons) &&
                                    hasOnlyA44DirectFallbackReasons(directInlineSerialIter->second.serialReasons);
      bool directInlineWorker0Task = directInlineWorker0Fallback && mtIsLevelDispatchMode() &&
                                    directInlineSerialIter != mtTasks.end() &&
                                    !directInlineSerialIter->second.repcutRuntimeApplied &&
                                    !directInlineSerialTask &&
                                    hasOnlyA73Worker0SafeReasons(directInlineSerialIter->second.serialReasons);
      indent = genNodeStepStart(super, mask, idx, flagName, indent, false);
      if (profileOffDirectSerial) {
        // A77 D1-NARROW: emit only the lean profile-off body, matching the
        // SerialFast shape (genActivate). Drops both `if (mtProfileEnabled)`
        // wrappers and the outer scope; profile counters/timers are not emitted.
        if (directInlineSerialTask || directInlineWorker0Task) {
          genSuperEval(super, flagName, "", indent, true);
        } else if (mtTasks[idx].repcutRuntimeApplied) {
          emitBodyLock(indent ++, "{\n");
          emitBodyLock(indent, "ActivationDelta mtScalarDelta;\n");
          emitBodyLock(indent, "mtRepCutLiteTask%d(%s, mtScalarDelta);\n", idx, flagName.c_str());
          emitBodyLock(indent, "mtScalarDelta.mergeInto(activeFlags);\n");
          emitBodyLock(-- indent, "}\n");
        } else {
          emitBodyLock(indent, "mtTask%d(%s);\n", idx, flagName.c_str());
        }
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
        // 28c Phase 1A: under mt-level-dispatch, distinguish worker0-only (forced
        // serial by side-effect) from safe-serial that fell out of any region.
        if (mtIsLevelDispatchMode() && hasWorker0OnlyReason(mtTasks[idx].serialReasons)) {
          emitBodyLock(indent + 1, "mtProfileWorker0OnlyDispatched ++;\n");
        } else if (mtIsLevelDispatchMode()) {
          emitBodyLock(indent + 1, "mtProfileSafeSerialDispatched ++;\n");
        }
        emitBodyLock(indent + 1, "mtProfileRejectSerialTask ++;\n");
      } else if (mtTaskHasSameActiveWordHazard(mtTasks, idx, globalConfig.MtRepCutLiteMode == "on")) {
        emitBodyLock(indent + 1, "mtProfileRejectSameActiveWordHazard ++;\n");
      } else {
        emitBodyLock(indent + 1, "mtProfileRejectBelowMinBatch ++;\n");
      }
      emitBodyLock(indent, "}\n");
      emitBodyLock(indent, "if (mtProfileEnabled) {\n");
      emitBodyLock(indent + 1, "std::chrono::steady_clock::time_point mtProfileTaskBegin = std::chrono::steady_clock::now();\n");
      if (mtTasks[idx].repcutRuntimeApplied) {
        emitBodyLock(indent + 1, "ActivationDelta mtScalarDelta;\n");
        emitBodyLock(indent + 1, "mtRepCutLiteTask%d(%s, mtScalarDelta);\n", idx, flagName.c_str());
        emitBodyLock(indent + 1, "mtScalarDelta.mergeInto(activeFlags);\n");
        emitBodyLock(indent + 1, "mtProfileActivationDeltaEntries += mtScalarDelta.entries.size();\n");
        emitBodyLock(indent + 1, "if (mtScalarDelta.entries.size() > mtProfileActivationDeltaMaxEntriesPerWorker) mtProfileActivationDeltaMaxEntriesPerWorker = mtScalarDelta.entries.size();\n");
        emitBodyLock(indent + 1, "if (mtScalarDelta.allActive) mtProfileActivationDeltaActivateAllCount ++;\n");
      } else {
        emitBodyLock(indent + 1, "mtTask%d(%s);\n", idx, flagName.c_str());
      }
      emitBodyLock(indent + 1, "recordMtProfileTask(%d, %s, std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileTaskBegin).count());\n",
                   idx, mtTasks[idx].taskKind == "pure_compute" ? "true" : "false");
      emitBodyLock(indent, "} else {\n");
      if (directInlineSerialTask || directInlineWorker0Task) {
        genSuperEval(super, flagName, "", indent + 1, true);
      } else if (mtTasks[idx].repcutRuntimeApplied) {
        emitBodyLock(indent + 1, "ActivationDelta mtScalarDelta;\n");
        emitBodyLock(indent + 1, "mtRepCutLiteTask%d(%s, mtScalarDelta);\n", idx, flagName.c_str());
        emitBodyLock(indent + 1, "mtScalarDelta.mergeInto(activeFlags);\n");
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

void graph::genResetDef(SuperNode* super, bool isUIntReset, bool buffered, int resetId, int indent, const std::string& nameSuffix, bool emitActivation) {
  std::string activeSinkType = (globalConfig.MtHelperMode == "mt" ||
                                globalConfig.MtHelperMode == "mt-level-dispatch")
                                 ? "ActivationDelta" : "ActiveBuffer";
  std::string resetFuncName = format("subReset%s%d", nameSuffix.c_str(), resetId);
  if (buffered) emitBodyLock(indent ++, "void S%s::%s(%s &nextActive){ // %s reset\n", name.c_str(), resetFuncName.c_str(), activeSinkType.c_str(), isUIntReset ? "uint" : "async");
  else emitBodyLock(indent ++, "void S%s::%s(){ // %s reset\n", name.c_str(), resetFuncName.c_str(), isUIntReset ? "uint" : "async");
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
      if (buffered) emitBodyLock(indent, "nextActive.activateAll();\n");
      else emitBodyLock(indent, "activateAll();\n");
    }
    else {
      std::map<uint64_t, ActiveType> bitMapInfo;
      activeSet2bitMap(allNext, bitMapInfo, -1);
      for (auto iter : bitMapInfo) {
        emitBodyLock(indent, "%s // %s\n", updateActiveStr(iter.first, ACTIVE_MASK(iter.second), buffered ? "nextActive" : "").c_str(), ACTIVE_COMMENT(iter.second).c_str());
      }
    }
    emitBodyLock(-- indent, "}\n");
  }
  for (InstInfo inst : super->insts) {
    switch (inst.infoType) {
      case SUPER_INFO_IF:
        emitBodyLock(indent ++, "%s\n", inst.inst.c_str());
        break;
      case SUPER_INFO_DEDENT:
        emitBodyLock(--indent, "%s\n", inst.inst.c_str());
        break;
      case SUPER_INFO_ELSE:
      case SUPER_INFO_STR:
        emitBodyLock(indent, "%s\n", inst.inst.c_str());
        break;
      default:
        break;
    }
  }
  if (!emitActivation) {
    emitBodyLock(-- indent, "}\n");
  }
  emitBodyLock(-- indent, "}\n");
}

void graph::genResetActivation(SuperNode* super, bool isUIntReset, int indent, int resetId) {
  emitBodyLock(indent, "subReset%d();\n", resetId);
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
  Assert(!denseSchedule.mtasks.empty(), "v202 requires MTask partitioning");
  int nMTasks = static_cast<int>(denseSchedule.mtasks.size());
  int threadCount = 8;
  const char* threadsEnv = std::getenv("GSIM_THREADS");
  if (threadsEnv != nullptr && threadsEnv[0] != '\0') threadCount = std::atoi(threadsEnv);
  if (threadCount < 1) threadCount = 1;
  bool workSteal = mtUseDenseWorkSteal();
  // Work-stealing runs MTasks in DYNAMIC order, so the static in-order-per-owner guarantee that
  // made same-thread-edge elision and per-worker-sequence transitive reduction safe no longer
  // holds. Under work-stealing we MUST keep ALL dependency edges (same-thread included) and skip
  // the sequence-based transitive reduction, or a successor could run before a same-owner pred.
  bool xthreadDepsOnly = workSteal ? false : mtUseDenseXThreadDepsOnly();
  bool transitiveReduceEdges = workSteal ? false : mtUseDenseTransitiveReduceEdges();
  std::vector<std::vector<int>> denseRuntimeSuccs = mtBuildDenseRuntimeSuccs(denseSchedule.mtasks, denseSchedule.mtaskThreadAssign, xthreadDepsOnly);
  int transitiveElidedEdges = transitiveReduceEdges ? mtReduceDenseRuntimeSuccsTransitive(denseRuntimeSuccs, denseSchedule.mtaskThreadAssign) : 0;
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
  fprintf(header, "static constexpr bool kDenseXThreadDepsOnly = %s;\n", xthreadDepsOnly ? "true" : "false");
  fprintf(header, "static constexpr bool kDenseTransitiveReduceEdges = %s;\n", transitiveReduceEdges ? "true" : "false");
  fprintf(header, "static constexpr int kDenseTransitiveElidedEdgeCount = %d;\n", transitiveElidedEdges);
  fprintf(header, "static constexpr uint32_t kDenseMTaskDepCount[%d] = {", nMTasks);
  for (int i = 0; i < nMTasks; i++) { if (i > 0) fprintf(header, ","); fprintf(header, "%u", denseRuntimeDepCounts[(size_t)i]); }
  fprintf(header, "};\n");
  fprintf(header, "static constexpr int kDenseMTaskSuccOffsets[%d] = {", nMTasks + 1);
  { int off = 0; for (int i = 0; i < nMTasks; i++) { if (i > 0) fprintf(header, ","); fprintf(header, "%d", off); off += static_cast<int>(denseRuntimeSuccs[(size_t)i].size()); } fprintf(header, ",%d};\n", off); }
  fprintf(header, "static constexpr int kDenseMTaskSuccList[%d] = {", totalSuccs);
  { bool firstS = true; for (const auto& succs : denseRuntimeSuccs) for (int s : succs) { if (!firstS) fprintf(header, ","); fprintf(header, "%d", s); firstS = false; } if (firstS) fprintf(header, "0"); fprintf(header, "};\n"); }
  fprintf(header, "struct MtDenseMTaskVertex { std::atomic<uint32_t> depsDone{0}; };\n");
  fprintf(header, "MtDenseMTaskVertex mtDenseMTaskVertices[%d];\n", nMTasks);
  // workSteal already computed above.
  if (workSteal) {
    // Preferred owner thread per MTask (from assignment) and worker0-only pin flag.
    fprintf(header, "static constexpr int kDenseMTaskOwner[%d] = {", nMTasks);
    for (int i = 0; i < nMTasks; i++) { if (i > 0) fprintf(header, ","); int o = denseSchedule.mtaskThreadAssign[(size_t)i]; if (o < 0 || o >= threadCount) o = 0; fprintf(header, "%d", o); }
    fprintf(header, "};\n");
    fprintf(header, "static constexpr bool kDenseMTaskW0[%d] = {", nMTasks);
    for (int i = 0; i < nMTasks; i++) { if (i > 0) fprintf(header, ","); fprintf(header, "%s", denseSchedule.mtasks[(size_t)i].workerZeroOnly ? "true" : "false"); }
    fprintf(header, "};\n");
    // Per-thread ready deque (bounded to nMTasks) + spinlock; global remaining counter.
    fprintf(header, "static constexpr int kDenseWorkStealThreads = %d;\n", threadCount);
    fprintf(header, "int mtDenseDeque[%d][%d];\n", threadCount, nMTasks);
    fprintf(header, "std::atomic<int> mtDenseDequeHead[%d];\n", threadCount); // pop point (LIFO top)
    fprintf(header, "std::atomic<int> mtDenseDequeTail[%d];\n", threadCount); // steal point (bottom)
    fprintf(header, "std::atomic_flag mtDenseDequeLock[%d];\n", threadCount);
    fprintf(header, "std::atomic<int> mtDenseRemaining;\n");
    fprintf(header, "void stepDenseMTaskById(int mtaskId);\n");
  }
  fprintf(header, "void stepDenseThreadWorker(int threadId);\n");
  for (int i = 0; i < nMTasks; i++) fprintf(header, "void stepDenseMTask%d();\n", i);
  for (int mtaskId = 0; mtaskId < nMTasks; mtaskId++) {
    const MtDenseMTask& mtask = denseSchedule.mtasks[mtaskId];
    emitFuncDecl(0, "void S%s::stepDenseMTask%d() {\n", name.c_str(), mtaskId);
    for (int sccId : mtask.sccIds) {
      Assert(sccId >= 0 && sccId < static_cast<int>(denseSchedule.sccs.size()), "dense schedule scc index out of range");
      const MtDenseScc& scc = denseSchedule.sccs[(size_t)sccId];
      for (int cppId : scc.cppIds) {
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
    }
    emitBodyLock(0, "}\n");
  }
  if (workSteal) {
    // Dispatch an MTask body by id (work-stealing runs MTasks in dynamic order).
    emitFuncDecl(0, "void S%s::stepDenseMTaskById(int mtaskId) {\n", name.c_str());
    emitBodyLock(1, "switch (mtaskId) {\n");
    for (int mtaskId = 0; mtaskId < nMTasks; mtaskId++) {
      emitBodyLock(2, "case %d: stepDenseMTask%d(); break;\n", mtaskId, mtaskId);
    }
    emitBodyLock(2, "default: break;\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(0, "}\n");
  }
  emitFuncDecl(0, "void S%s::stepDenseThreadWorker(int threadId) {\n", name.c_str());
  emitBodyLock(1, "bool evenCycle = (cycles & 1) == 0;\n");
  if (workSteal) {
    // Owner-affine ready-deque work-stealing. A worker pops ready MTasks from its own deque
    // (LIFO), steals from other deques' tails when idle, runs the body, and on each successor's
    // LAST dependency (enqueue-on-last-dep, avoids double-run races) pushes it to its owner's
    // deque. worker0-only MTasks are never stolen by nonzero workers. Exits when no MTask
    // remains globally. Seeding of dep-0 MTasks is done in stepDense before workers start.
    emitBodyLock(1, "const bool evenParity = evenCycle;\n");
    emitBodyLock(1, "auto dqPush = [&](int owner, int mt) {\n");
    emitBodyLock(2, "while (mtDenseDequeLock[owner].test_and_set(std::memory_order_acquire)) mtWorkerPoolPause();\n");
    emitBodyLock(2, "int t = mtDenseDequeTail[owner].load(std::memory_order_relaxed);\n");
    emitBodyLock(2, "mtDenseDeque[owner][t] = mt; mtDenseDequeTail[owner].store(t + 1, std::memory_order_release);\n");
    emitBodyLock(2, "mtDenseDequeLock[owner].clear(std::memory_order_release);\n");
    emitBodyLock(1, "};\n");
    emitBodyLock(1, "auto dqPopLocal = [&](int owner) -> int {\n");
    emitBodyLock(2, "int mt = -1;\n");
    emitBodyLock(2, "while (mtDenseDequeLock[owner].test_and_set(std::memory_order_acquire)) mtWorkerPoolPause();\n");
    emitBodyLock(2, "int h = mtDenseDequeHead[owner].load(std::memory_order_relaxed);\n");
    emitBodyLock(2, "int t = mtDenseDequeTail[owner].load(std::memory_order_relaxed);\n");
    emitBodyLock(2, "if (t > h) { t--; mt = mtDenseDeque[owner][t]; mtDenseDequeTail[owner].store(t, std::memory_order_relaxed); }\n");
    emitBodyLock(2, "mtDenseDequeLock[owner].clear(std::memory_order_release);\n");
    emitBodyLock(2, "return mt;\n");
    emitBodyLock(1, "};\n");
    emitBodyLock(1, "auto dqSteal = [&](int victim, int thief) -> int {\n");
    emitBodyLock(2, "int mt = -1;\n");
    emitBodyLock(2, "while (mtDenseDequeLock[victim].test_and_set(std::memory_order_acquire)) mtWorkerPoolPause();\n");
    emitBodyLock(2, "int h = mtDenseDequeHead[victim].load(std::memory_order_relaxed);\n");
    emitBodyLock(2, "int t = mtDenseDequeTail[victim].load(std::memory_order_relaxed);\n");
    emitBodyLock(2, "if (t > h) { int cand = mtDenseDeque[victim][h]; if (!(kDenseMTaskW0[cand] && thief != 0)) { mtDenseDequeHead[victim].store(h + 1, std::memory_order_relaxed); mt = cand; } }\n");
    emitBodyLock(2, "mtDenseDequeLock[victim].clear(std::memory_order_release);\n");
    emitBodyLock(2, "return mt;\n");
    emitBodyLock(1, "};\n");
    emitBodyLock(1, "auto signalSuccs = [&](int mt) {\n");
    emitBodyLock(2, "for (int j = kDenseMTaskSuccOffsets[mt]; j < kDenseMTaskSuccOffsets[mt + 1]; j++) {\n");
    emitBodyLock(3, "int s = kDenseMTaskSuccList[j];\n");
    emitBodyLock(3, "bool ready;\n");
    emitBodyLock(3, "if (evenParity) { uint32_t old = mtDenseMTaskVertices[s].depsDone.fetch_add(1, std::memory_order_acq_rel); ready = (old + 1 == kDenseMTaskDepCount[s]); }\n");
    emitBodyLock(3, "else { uint32_t old = mtDenseMTaskVertices[s].depsDone.fetch_sub(1, std::memory_order_acq_rel); ready = (old == 1); }\n");
    emitBodyLock(3, "if (ready) { int owner = kDenseMTaskOwner[s]; dqPush(owner, s); }\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(1, "};\n");
    emitBodyLock(1, "uint64_t idleSpins = 0;\n");
    emitBodyLock(1, "for (;;) {\n");
    emitBodyLock(2, "int mt = dqPopLocal(threadId);\n");
    emitBodyLock(2, "if (mt < 0) {\n");
    emitBodyLock(3, "for (int v = 0; v < kDenseWorkStealThreads && mt < 0; v++) { if (v != threadId) mt = dqSteal(v, threadId); }\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(2, "if (mt < 0) {\n");
    emitBodyLock(3, "if (mtDenseRemaining.load(std::memory_order_acquire) <= 0) break;\n");
    emitBodyLock(3, "if (++idleSpins > 200000000u) { fprintf(stderr, \"[mt-dense-worksteal] DEADLOCK thread %%d remaining=%%d\\n\", threadId, mtDenseRemaining.load(std::memory_order_acquire)); abort(); }\n");
    emitBodyLock(3, "mtWorkerPoolPause(); continue;\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(2, "idleSpins = 0;\n");
    emitBodyLock(2, "stepDenseMTaskById(mt);\n");
    emitBodyLock(2, "signalSuccs(mt);\n");
    emitBodyLock(2, "mtDenseRemaining.fetch_sub(1, std::memory_order_acq_rel);\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(1, "return;\n");
  }
  emitBodyLock(1, "switch (threadId) {\n");
  for (int t = 0; t < threadCount; t++) {
    emitBodyLock(2, "case %d: {\n", t);
    for (int mtaskId = 0; mtaskId < nMTasks; mtaskId++) {
      if (denseSchedule.mtaskThreadAssign[mtaskId] != t) continue;
      // Verilator VlMTaskVertex::waitUntilUpstreamDone pattern: spin 64, then yield
      emitBodyLock(3, "{ const uint32_t target = evenCycle ? kDenseMTaskDepCount[%d] : 0u;\n", mtaskId);
      emitBodyLock(3, "  unsigned ct = 0;\n");
      emitBodyLock(3, "  while (mtDenseMTaskVertices[%d].depsDone.load(std::memory_order_acquire) != target) {\n", mtaskId);
      emitBodyLock(4, "mtWorkerPoolPause(); if (++ct > 64) { ct = 0; std::this_thread::yield(); } }\n");
      emitBodyLock(3, "}\n");
      emitBodyLock(3, "stepDenseMTask%d();\n", mtaskId);
      // Verilator signalUpstreamDone: fetch_add (even) or fetch_sub (odd)
      emitBodyLock(3, "if (evenCycle) {\n");
      emitBodyLock(4, "for (int j = kDenseMTaskSuccOffsets[%d]; j < kDenseMTaskSuccOffsets[%d]; j++)\n", mtaskId, mtaskId + 1);
      emitBodyLock(5, "mtDenseMTaskVertices[kDenseMTaskSuccList[j]].depsDone.fetch_add(1, std::memory_order_release);\n");
      emitBodyLock(3, "} else {\n");
      emitBodyLock(4, "for (int j = kDenseMTaskSuccOffsets[%d]; j < kDenseMTaskSuccOffsets[%d]; j++)\n", mtaskId, mtaskId + 1);
      emitBodyLock(5, "mtDenseMTaskVertices[kDenseMTaskSuccList[j]].depsDone.fetch_sub(1, std::memory_order_release);\n");
      emitBodyLock(3, "}\n");
    }
    emitBodyLock(3, "break;\n");
    emitBodyLock(2, "}\n");
  }
  emitBodyLock(2, "default: break;\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(0, "}\n");
  emitFuncDecl(0, "void S%s::stepDense() {\n", name.c_str());
  emitBodyLock(1, "std::chrono::steady_clock::time_point mtProfileStepBegin;\n");
  emitBodyLock(1, "if (unlikely(mtProfileEnabled)) mtProfileStepBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(1, "resetAllDense();\n");
  for (SuperNode* super : sortedSuper) {
    for (Node* member : super->member) {
      if (member->isReset() && member->type == NODE_REG_SRC) {
        emitBodyLock(1, "%s = %s;\n", RESET_NAME(member).c_str(), member->name.c_str());
      }
    }
  }
  // No counter reset needed — Verilator even/odd alternation handles it
  if (workSteal) {
    // Seed the work-stealing deques: reset head/tail, set remaining, and push every
    // dependency-free MTask (depCount==0) to its owner's deque. depsDone counters keep the
    // Verilator even/odd parity across cycles, so a dep-0 MTask is ready every cycle.
    emitBodyLock(1, "if (mtConfiguredWorkerCount > 1 && mtWorkerPoolEnabled && mtWorkerPoolThreadCount + 1 >= mtConfiguredWorkerCount) {\n");
    emitBodyLock(2, "for (int t = 0; t < kDenseWorkStealThreads; t++) { mtDenseDequeHead[t].store(0, std::memory_order_relaxed); mtDenseDequeTail[t].store(0, std::memory_order_relaxed); mtDenseDequeLock[t].clear(std::memory_order_relaxed); }\n");
    emitBodyLock(2, "mtDenseRemaining.store(%d, std::memory_order_relaxed);\n", nMTasks);
    emitBodyLock(2, "for (int mt = 0; mt < %d; mt++) { if (kDenseMTaskDepCount[mt] == 0) { int o = kDenseMTaskOwner[mt]; int tl = mtDenseDequeTail[o].load(std::memory_order_relaxed); mtDenseDeque[o][tl] = mt; mtDenseDequeTail[o].store(tl + 1, std::memory_order_relaxed); } }\n", nMTasks);
    emitBodyLock(2, "std::atomic_thread_fence(std::memory_order_release);\n");
    emitBodyLock(1, "}\n");
  }
  emitBodyLock(1, "if (mtConfiguredWorkerCount > 1 && mtWorkerPoolEnabled && mtWorkerPoolThreadCount + 1 >= mtConfiguredWorkerCount) {\n");
  emitBodyLock(2, "mtWorkerPoolJobKind = 7;\n");
  emitBodyLock(2, "mtWorkerPoolCurrentWorkerCount = mtConfiguredWorkerCount;\n");
  emitBodyLock(2, "mtWorkerPoolPost();\n");
  emitBodyLock(2, "stepDenseThreadWorker(0);\n");
  emitBodyLock(2, "mtWorkerPoolWaitForDone(mtConfiguredWorkerCount - 1);\n");
  emitBodyLock(1, "} else {\n");
  for (int mtaskId = 0; mtaskId < nMTasks; mtaskId++) {
    emitBodyLock(2, "stepDenseMTask%d();\n", mtaskId);
  }
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "if (mtProfileDynamicTraceFile != nullptr) dumpMtProfileDynamicTraceCycle();\n");
  emitBodyLock(1, "cycles ++;\n");
  emitBodyLock(1, "if (unlikely(mtProfileEnabled)) mtProfileTotalStepNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileStepBegin).count();\n");
  emitBodyLock(0, "}\n");
}

void graph::genStep(int subStepIdxMax, int serialFastSubStepMax, const std::string& serialFastSuffix, bool denseExecutorValid) {
  emitFuncDecl(0, "void S%s::step() {\n", name.c_str());
  emitBodyLock(1, "std::chrono::steady_clock::time_point mtProfileStepBegin;\n");
  if (denseExecutorValid) emitBodyLock(1, "if (unlikely(mtUseDenseExecutor)) { stepDense(); return; }\n");
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
    emitBodyLock(1, "if (mtConfiguredWorkerCount <= 1 && !mtProfileEnabled) {\n");
    for (int i = 0; i <= serialFastSubStepMax; i ++) {
      emitBodyLock(2, "subStep%d%s();\n", i, serialFastSuffix.c_str());
    }
    emitBodyLock(2, "cycles ++;\n");
    emitBodyLock(2, "return;\n");
    emitBodyLock(1, "}\n");
  }
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
  emitBodyLock(1, "cycles ++;\n");
  emitBodyLock(1, "if (unlikely(mtProfileEnabled)) mtProfileTotalStepNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileStepBegin).count();\n");
  emitBodyLock(0, "}\n");
}

bool SuperNode::instsEmpty() {
  return insts.size() == 0;
}

bool graph::__emitSrc(int indent, bool canNewFile, bool alreadyEndFunc, const char *nextFuncDef, const char *fmt, ...) {
  bool newFile = false;
  if (srcFp == NULL || (srcFileBytes > (globalConfig.cppMaxSizeKB * 1024) && canNewFile)) {
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
    srcFileBytes = fprintf(srcFp, "#include \"%s.h\"\n", name.c_str());
    if (nextFuncDef != NULL) {
      srcFileBytes += fprintf(srcFp, "%s {\n", nextFuncDef);
    }
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

void graph::emitPrintf() {
  emitFuncDecl(0, "void gprintf(const char *fmt, ...) {\n");
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
#if 0
      if (super->member.size() == 1) {
        alwaysActive.insert(super->cppId);
        printf("alwaysActive %d\n", super->cppId);
      }
#endif
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
  if (globalConfig.DumpMtScheduleJson) dumpMtScheduleJson();
  if (globalConfig.DumpMtRepCutLiteReport || globalConfig.MtRepCutLiteMode == "on") dumpMtRepCutLiteReport();
  if (globalConfig.DumpMtCoarseRegionReport || globalConfig.MtBatchFormationMode == "coarse") dumpMtCoarseRegionReport();
  if (mtUseReadyBatchReport() || mtUseEnvelopeLocalEval() || mtUseEnvelopeLocalEvalDiagnostics()) dumpMtReadyBatchReport();
  if (mtUseDenseExecutorCodegen()) dumpMtDenseScheduleJson();
  if (globalConfig.MtReportOnly) {
    printf("[cppEmitter] mt-report-only: skipped generated C++ emission after reports\n");
    return;
  }

  if (!globalConfig.MtStableOutput) {
    // 28c Phase 1A: remove stale SimTop*.cpp files from previous runs so the
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
  mtProfileRepCutBatchBeginCppIds.clear();
  mtProfileRepCutRuntimeCppIds.clear();
  std::map<int, MtTaskInfo> mtRepCutHeaderTasks;
  MtCoarseProfileFacts mtCoarseProfileFacts;
  bool useDenseExecutorCodegen = mtUseDenseExecutorCodegen();
  MtDenseSchedule mtDenseSchedule;
  if (useMtHelpers) {
    mtRepCutHeaderTasks = buildMtTaskInfoMapWithRepCutSelectionForInvocation();
    markMtRepCutLiteRuntimeApplied(mtRepCutHeaderTasks);
    MtRepCutSemanticPlan mtRepCutHeaderSemanticPlan = planMtRepCutSemantics(mtRepCutHeaderTasks);
    mtSetProfileRepCutBatchBeginCppIds(mtRepCutHeaderSemanticPlan);
    mtSetProfileRepCutRuntimeCppIds(mtRepCutHeaderTasks);
    if (useCoarseMt) {
      mtCoarseProfileFacts = mtComputeCoarseProfileFacts(planMtCoarseRegionsForInvocation(mtRepCutHeaderTasks));
    }
  }
  if (useDenseExecutorCodegen) {
    Assert(useCoarseMt, "GSIM_MT_DENSE_EXECUTOR_CODEGEN requires --mt-helper-mode=mt-level-dispatch with coarse batch formation in v181");
    if (mtRepCutHeaderTasks.empty()) {
      mtRepCutHeaderTasks = buildMtTaskInfoMapWithRepCutSelectionForInvocation();
      markMtRepCutLiteRuntimeApplied(mtRepCutHeaderTasks);
    }
    mtDenseSchedule = buildMtDenseSchedule(mtRepCutHeaderTasks, true);
  }
  bool denseExecutorValid = useDenseExecutorCodegen && mtDenseSchedule.valid;
  if (useDenseExecutorCodegen && !mtDenseSchedule.valid) {
    printf("[mt-dense-schedule] dense executor codegen disabled: fallback=%s\n", mtDenseSchedule.fallbackReason.c_str());
  }
  std::vector<unsigned char> mtProfileStateUpdateTraceKindByCppIdCodegen;
  if (mtUseDynamicStateTraceCodegen()) {
    mtProfileStateUpdateTraceKindByCppIdCodegen.assign(superId, 0);
    std::map<int, MtTaskInfo> mtStateTraceTasks = mtRepCutHeaderTasks;
    if (mtStateTraceTasks.empty()) mtStateTraceTasks = buildMtTaskInfoMapWithRepCutSelectionForInvocation();
    std::vector<MtStateUpdateTraceInfo> mtStateTraceInfos = buildMtStateUpdateTraceInfoForInvocation(mtStateTraceTasks);
    for (int cppId = 0; cppId < static_cast<int>(mtStateTraceInfos.size()); cppId ++) {
      const MtStateUpdateTraceInfo& info = mtStateTraceInfos[cppId];
      if (!info.hasStateUpdate) continue;
      mtProfileStateUpdateTraceKindByCppIdCodegen[cppId] = info.runtimeSafeCandidate ? 3 : (info.localSafeCandidate ? 2 : 1);
    }
  }
  if (globalConfig.MtHelperMode == "buffered-seq") emitActiveBufferDef(header, activeFlagNum);
  if (useMtHelpers) emitActivationDeltaDef(header, activeFlagNum);

  fprintf(header, "class S%s {\npublic:\n", name.c_str());
  fprintf(header, "uint64_t cycles;\n");
  fprintf(header, "uint64_t LOG_START, LOG_END;\n");
  fprintf(header, "uint%d_t activeFlags[%d];\n", ACTIVE_WIDTH, activeFlagNum); // or super.size() if id == idx
  if (denseExecutorValid) fprintf(header, "bool mtUseDenseExecutor;\n");
  fprintf(header, "bool mtProfileEnabled;\n");
  fprintf(header, "FILE *mtProfileDynamicTraceFile;\n");
  fprintf(header, "uint64_t mtProfileDynamicTraceCycleStart;\n");
  fprintf(header, "uint64_t mtProfileDynamicTraceCycleLimit;\n");
  fprintf(header, "std::vector<int> mtProfileDynamicTraceTaskIds;\n");
  if (mtUseDynamicStateTraceCodegen()) {
    fprintf(header, "bool mtProfileDynamicStateTraceEnabled;\n");
    fprintf(header, "std::vector<uint8_t> mtProfileStateUpdateTraceKindByCppId;\n");
  }
  fprintf(header, "const char *mtProfileHelperMode;\n");
  fprintf(header, "int mtConfiguredWorkerCount;\n");
  fprintf(header, "int mtMinBatchTasks;\n");
  fprintf(header, "int mtCoarseMinActiveBits;\n");
  fprintf(header, "int mtCoarseInlineThreshold;\n");
  if (mtUseSubchunkRuntime()) {
    fprintf(header, "int mtSubchunkDispatchCost;\n");
    fprintf(header, "int mtSubchunkMinActiveBits;\n");
  }
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
  fprintf(header, "std::atomic<uint64_t> mtProfileRepCutLiteTaskCallsByCppId[%d];\n", superId);
  fprintf(header, "uint64_t wallfracCommitCycles;\n");
  fprintf(header, "uint64_t wallfracCombCycles;\n");
  fprintf(header, "uint64_t wallfracCommitBrackets;\n");
  fprintf(header, "uint64_t wallfracCombBrackets;\n");
  fprintf(header, "uint64_t mtProfileRejectNotActiveWhole;\n");
  fprintf(header, "uint64_t mtProfileRejectAlwaysActiveTask;\n");
  fprintf(header, "uint64_t mtProfileRejectSerialTask;\n");
  fprintf(header, "uint64_t mtProfileSafeSerialDispatched;\n");      // 28c Phase 1A
  fprintf(header, "uint64_t mtProfileWorker0OnlyDispatched;\n");     // 28c Phase 1A
  fprintf(header, "uint64_t mtProfileRejectDependencyEdge;\n");
  fprintf(header, "uint64_t mtProfileRejectSameActiveWordHazard;\n");
  fprintf(header, "uint64_t mtProfileRejectBelowMinBatch;\n");
  fprintf(header, "uint64_t mtProfileRejectConfiguredSingleWorker;\n");
  fprintf(header, "uint64_t mtProfileBatchMemberNodeCount;\n");
  fprintf(header, "uint64_t mtProfileSameActiveWordForwardEdges;\n");
  fprintf(header, "uint64_t mtProfileCrossBatchActivationFanout;\n");
  fprintf(header, "uint64_t mtProfileBatchWallNs;\n");
  fprintf(header, "uint64_t mtProfileTrueParallelWallNs;\n");
  fprintf(header, "std::vector<uint64_t> mtProfileRepCutBatchHits;\n");
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
    fprintf(header, "uint64_t mtProfileCoarseSerialFallbackRepcutExcluded;\n");
    fprintf(header, "uint64_t mtProfileCoarseSerialFallbackNonPureExcluded;\n");
    fprintf(header, "uint64_t mtProfileCoarseSerialFallbackSavedWorkerJobs;\n");
    fprintf(header, "uint64_t mtProfileCoarseSerialFallbackSavedFlagWordCopies;\n");
    fprintf(header, "uint64_t mtProfileCoarseSerialFallbackSavedMergeWordScans;\n");
    fprintf(header, "uint64_t mtProfileCoarseSerialFallbackSavedBarriers;\n");
    if (mtUseSubchunkRuntime()) {
      fprintf(header, "uint64_t mtProfileCoarseSubchunkDispatchEligible;\n");
      fprintf(header, "uint64_t mtProfileCoarseSubchunkDispatchTaken;\n");
      fprintf(header, "uint64_t mtProfileCoarseSubchunkDispatchInlineWords;\n");
      fprintf(header, "uint64_t mtProfileCoarseSubchunkDispatchSkippedBelowMinActive;\n");
      fprintf(header, "uint64_t mtProfileCoarseSubchunkDispatchResidualDispatches;\n");
      fprintf(header, "uint64_t mtProfileCoarseSubchunkDispatchFullyInlined;\n");
      fprintf(header, "uint64_t mtProfileCoarseSubchunkDispatchInlineActiveBits;\n");
      fprintf(header, "uint64_t mtProfileCoarseSubchunkDispatchResidualInitialActiveBits;\n");
    }
    if (mtUseSubchunkProbe()) {
      fprintf(header, "uint64_t mtProfileCoarseSerialFallbackDynamicWords;\n");
      fprintf(header, "uint64_t mtProfileCoarseSerialFallbackDynamicTasks;\n");
      fprintf(header, "uint64_t mtProfileCoarseSerialFallbackDynamicTaskStaticCost;\n");
      fprintf(header, "uint64_t mtProfileCoarseSerialFallbackDynamicWordCostHist[6];\n");
      fprintf(header, "uint64_t mtProfileCoarseSerialFallbackDynamicWordTaskHist[6];\n");
      fprintf(header, "uint64_t mtProfileCoarseSerialFallbackDynamicWordCostGe64;\n");
      fprintf(header, "uint64_t mtProfileCoarseSerialFallbackDynamicWordCostGe128;\n");
      fprintf(header, "uint64_t mtProfileCoarseSerialFallbackDynamicWordCostGe256;\n");
      fprintf(header, "uint64_t mtProfileCoarseSerialFallbackDynamicWordTasksGe2;\n");
    }
    fprintf(header, "uint64_t mtProfileCoarseLayerSizeHist[6];\n");
    fprintf(header, "uint64_t mtProfileCoarseRegionLayerCountHist[6];\n");
    fprintf(header, "std::vector<uint64_t> mtProfileCoarseSelectedWorkerCountHist;\n");
    if (mtUseWaitProbeCodegen()) {
      fprintf(header, "bool mtWaitProbeEnabled;\n");
      fprintf(header, "std::chrono::steady_clock::time_point mtWaitProbePostTp;\n");
      fprintf(header, "std::vector<uint64_t> mtWaitProbeWorkerFinishNs;\n");
      fprintf(header, "std::vector<uint64_t> mtWaitProbeWorkerFinishSumNs;\n");
      fprintf(header, "std::vector<uint64_t> mtWaitProbeWorkerLastHist;\n");
      fprintf(header, "uint64_t mtWaitProbeDispatchCount;\n");
      fprintf(header, "uint64_t mtWaitProbeWaitSumNs;\n");
      fprintf(header, "uint64_t mtWaitProbeW0BodySumNs;\n");
      fprintf(header, "uint64_t mtWaitProbeTailBeyondW0SumNs;\n");
      fprintf(header, "uint64_t mtWaitProbeMaxFinishSumNs;\n");
      fprintf(header, "uint64_t mtWaitProbeMinBgFinishSumNs;\n");
      fprintf(header, "uint64_t mtWaitProbeWorker0LastCount;\n");
      fprintf(header, "uint64_t mtWaitProbeEmptyBarrierIters;\n");
      fprintf(header, "uint64_t mtWaitProbeEmptyBarrierTotalNs;\n");
    }
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
      // 28c D-static Step 1: codegen-time LPT + flat per-cppId arrays.
      // SCoarseTaskFn points to the 2-arg mtTaskN/mtRepCutLiteTaskN overload
      // (uint%d_t&, ActivationDelta&). Keep mask before the member-function
      // pointer; cppId is uint32_t because XiangShan has >65535 emitted tasks.
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
      // Track 2 Week 4: per-mtask atomic counters and shared region flags for antichain runtime.
      // Track 2 Week 6: use a stamped claim-generation counter and even-cycle upstream target
      // so that neither state[] nor upstream[] need a per-invocation reset loop.
      fprintf(header, "std::vector<std::atomic<uint64_t>*> mtCoarseMTaskClaimGen;\n");
      fprintf(header, "std::vector<std::atomic<int>*> mtCoarseMTaskUpstream;\n");
      fprintf(header, "std::vector<int> mtCoarseMTaskCount;\n");
      fprintf(header, "std::vector<std::atomic<uint%d_t>*> mtCoarseRegionSharedFlags;\n", ACTIVE_WIDTH);
      fprintf(header, "alignas(64) std::atomic<int> mtCoarseMTaskRemaining;\n");
      fprintf(header, "std::vector<std::atomic<uint64_t>*> mtCoarseRegionCycle;\n");
      fprintf(header, "uint%d_t* mtWorkerPoolCoarseActiveWords;\n", ACTIVE_WIDTH);
      // Track 2 Week 7: mutex-protected ready queues for antichain scheduler.
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
    // 28c-2 atomic-spin worker pool: hot atomics on independent cache lines.
    fprintf(header, "alignas(64) std::atomic<uint64_t> mtWorkerPoolGeneration;\n");
    fprintf(header, "alignas(64) std::atomic<int> mtWorkerPoolDoneCount;\n");
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
  if (useMtHelpers) emitBodyLock(1, "if (!mtWorkerPoolLazyStart) startMtWorkerPool();\n");
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
  for (SuperNode* super : sortedSuper) {
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
  fprintf(header, "void dumpMtProfile();\n");
  fprintf(header, "void recordMtProfileTask(int cppId, bool pureTask, uint64_t elapsedNs);\n");
  fprintf(header, "void dumpMtProfileDynamicTraceCycle();\n");
  fprintf(header, "void recordMtProfileDynamicTraceTask(int cppId);\n");
  fprintf(header, "void recordMtProfileWorkerTask(int worker);\n");
  if (useCoarseMt && mtUseWaitProbeCodegen()) {
    fprintf(header, "void runMtWaitProbeEmptyBarrier();\n");
    fprintf(header, "void dumpMtWaitProbe();\n");
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
  if (mtUseSubchunkRuntime()) {
    emitBodyLock(1, "int mtSubchunkDispatchCost = 0;\n");
    emitBodyLock(1, "const char *subchunkDispatchCostEnv = getenv(\"GSIM_MT_SUBCHUNK_DISPATCH_COST\");\n");
    emitBodyLock(1, "if (subchunkDispatchCostEnv != nullptr) mtSubchunkDispatchCost = atoi(subchunkDispatchCostEnv);\n");
    emitBodyLock(1, "if (mtSubchunkDispatchCost < 0) mtSubchunkDispatchCost = 0;\n");
    emitBodyLock(1, "this->mtSubchunkDispatchCost = mtSubchunkDispatchCost;\n");
    emitBodyLock(1, "int mtSubchunkMinActiveBits = 0;\n");
    emitBodyLock(1, "const char *subchunkMinActiveBitsEnv = getenv(\"GSIM_MT_SUBCHUNK_MIN_ACTIVE_BITS\");\n");
    emitBodyLock(1, "if (subchunkMinActiveBitsEnv != nullptr) mtSubchunkMinActiveBits = atoi(subchunkMinActiveBitsEnv);\n");
    emitBodyLock(1, "if (mtSubchunkMinActiveBits < 0) mtSubchunkMinActiveBits = 0;\n");
    emitBodyLock(1, "this->mtSubchunkMinActiveBits = mtSubchunkMinActiveBits;\n");
  }
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
  emitBodyLock(1, "for (int i = 0; i < %d; i ++) mtProfileRepCutLiteTaskCallsByCppId[i].store(0, std::memory_order_relaxed);\n", superId);
  emitBodyLock(1, "wallfracCommitCycles = 0; wallfracCombCycles = 0; wallfracCommitBrackets = 0; wallfracCombBrackets = 0;\n");
  emitBodyLock(1, "mtProfileRejectNotActiveWhole = 0;\n");
  emitBodyLock(1, "mtProfileRejectAlwaysActiveTask = 0;\n");
  emitBodyLock(1, "mtProfileRejectSerialTask = 0;\n");
  emitBodyLock(1, "mtProfileSafeSerialDispatched = 0;\n");      // 28c Phase 1A
  emitBodyLock(1, "mtProfileWorker0OnlyDispatched = 0;\n");     // 28c Phase 1A
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
  emitBodyLock(1, "mtProfileRepCutBatchHits.assign((size_t)%zu, 0);\n", mtProfileRepCutBatchBeginCppIds.size());
  emitBodyLock(1, "mtProfileDynamicTraceFile = nullptr;\n");
  emitBodyLock(1, "mtProfileDynamicTraceCycleStart = 0;\n");
  emitBodyLock(1, "mtProfileDynamicTraceCycleLimit = 0;\n");
  emitBodyLock(1, "mtProfileDynamicTraceTaskIds.clear();\n");
  if (mtUseDynamicStateTraceCodegen()) {
    emitBodyLock(1, "mtProfileDynamicStateTraceEnabled = dynamicTraceEnabled && dynamicStateTraceEnv != nullptr && dynamicStateTraceEnv[0] != '\\0' && dynamicStateTraceEnv[0] != '0';\n");
    emitBodyLock(1, "static const uint8_t mtProfileStateUpdateTraceKindInit[%d] = {", superId);
    for (int cppId = 0; cppId < static_cast<int>(mtProfileStateUpdateTraceKindByCppIdCodegen.size()); cppId ++) {
      if (cppId != 0) emitBodyLock(0, ",");
      if (cppId % 64 == 0) emitBodyLock(0, "\n    ");
      emitBodyLock(0, "%u", static_cast<unsigned>(mtProfileStateUpdateTraceKindByCppIdCodegen[cppId]));
    }
    emitBodyLock(0, "\n");
    emitBodyLock(1, "};\n");
    emitBodyLock(1, "mtProfileStateUpdateTraceKindByCppId.assign(mtProfileStateUpdateTraceKindInit, mtProfileStateUpdateTraceKindInit + %d);\n", superId);
  }
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
    // Track 2 Week 7: ready-queue state for antichain scheduler.
    emitBodyLock(1, "mtCoarseMTaskInFlight.store(0, std::memory_order_relaxed);\n");
    emitBodyLock(1, "const char *antichainQueueEnv = getenv(\"GSIM_MT_ANTICHAIN_QUEUE\");\n");
    emitBodyLock(1, "mtCoarseUseAntichainQueue = (antichainQueueEnv == nullptr) || (antichainQueueEnv[0] != '0');\n");
    emitBodyLock(1, "mtProfileCoarseStaticRuntimeEligibleRegions = %d;\n", mtCoarseProfileFacts.runtimeEligibleRegionCount);
    emitBodyLock(1, "mtCoarseReadyQueueParallel.assign((size_t)mtProfileCoarseStaticRuntimeEligibleRegions, std::vector<int>());\n");
    emitBodyLock(1, "mtCoarseReadyQueueWorker0.assign((size_t)mtProfileCoarseStaticRuntimeEligibleRegions, std::vector<int>());\n");
    // Track 2 Week 6: claim-generation + even-cycle arrays; cycle counters allocated per-region.
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
    emitBodyLock(1, "mtProfileCoarseSerialFallbackRepcutExcluded = 0;\n");
    emitBodyLock(1, "mtProfileCoarseSerialFallbackNonPureExcluded = 0;\n");
    emitBodyLock(1, "mtProfileCoarseSerialFallbackSavedWorkerJobs = 0;\n");
    emitBodyLock(1, "mtProfileCoarseSerialFallbackSavedFlagWordCopies = 0;\n");
    emitBodyLock(1, "mtProfileCoarseSerialFallbackSavedMergeWordScans = 0;\n");
    emitBodyLock(1, "mtProfileCoarseSerialFallbackSavedBarriers = 0;\n");
    if (mtUseSubchunkRuntime()) {
      emitBodyLock(1, "mtProfileCoarseSubchunkDispatchEligible = 0;\n");
      emitBodyLock(1, "mtProfileCoarseSubchunkDispatchTaken = 0;\n");
      emitBodyLock(1, "mtProfileCoarseSubchunkDispatchInlineWords = 0;\n");
      emitBodyLock(1, "mtProfileCoarseSubchunkDispatchSkippedBelowMinActive = 0;\n");
      emitBodyLock(1, "mtProfileCoarseSubchunkDispatchResidualDispatches = 0;\n");
      emitBodyLock(1, "mtProfileCoarseSubchunkDispatchFullyInlined = 0;\n");
      emitBodyLock(1, "mtProfileCoarseSubchunkDispatchInlineActiveBits = 0;\n");
      emitBodyLock(1, "mtProfileCoarseSubchunkDispatchResidualInitialActiveBits = 0;\n");
    }
    if (mtUseSubchunkProbe()) {
      emitBodyLock(1, "mtProfileCoarseSerialFallbackDynamicWords = 0;\n");
      emitBodyLock(1, "mtProfileCoarseSerialFallbackDynamicTasks = 0;\n");
      emitBodyLock(1, "mtProfileCoarseSerialFallbackDynamicTaskStaticCost = 0;\n");
      emitBodyLock(1, "for (int i = 0; i < 6; i ++) { mtProfileCoarseSerialFallbackDynamicWordCostHist[i] = 0; mtProfileCoarseSerialFallbackDynamicWordTaskHist[i] = 0; }\n");
      emitBodyLock(1, "mtProfileCoarseSerialFallbackDynamicWordCostGe64 = 0;\n");
      emitBodyLock(1, "mtProfileCoarseSerialFallbackDynamicWordCostGe128 = 0;\n");
      emitBodyLock(1, "mtProfileCoarseSerialFallbackDynamicWordCostGe256 = 0;\n");
      emitBodyLock(1, "mtProfileCoarseSerialFallbackDynamicWordTasksGe2 = 0;\n");
    }
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
    emitBodyLock(1, "mtWorkerPoolDoneCount.store(0, std::memory_order_relaxed);\n");
    emitBodyLock(1, "mtWorkerPoolReadyCount.store(0, std::memory_order_relaxed);\n");
    emitBodyLock(1, "mtWorkerPoolCurrentWorkerCount = 0;\n");
    emitBodyLock(1, "mtWorkerPoolJobKind = 0;\n");
    emitBodyLock(1, "mtWorkerPoolDenseLayerFn = nullptr;\n");
    if (useCoarseMt) {
      emitBodyLock(1, "mtWorkerPoolCoarseRegionIndex = -1;\n");
      emitBodyLock(1, "mtWorkerPoolCoarseLayerIndex = -1;\n");
      // 28c-2 default: mtask runtime for mt-level-dispatch; env GSIM_MT_COARSE_RUNTIME=layered overrides.
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
      // 28c D-static Step 1: env GSIM_MT_COARSE_DSTATIC=0 disables the
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
      // Track 2 Week 4: env GSIM_MT_ANTICHAIN_RUNTIME=1 enables per-mtask
      // atomic-counter scheduler for antichain-enabled coarse regions.
      emitBodyLock(1, "mtCoarseUseAntichainRuntime = false;\n");
      emitBodyLock(1, "const char *antichainRuntimeEnv = getenv(\"GSIM_MT_ANTICHAIN_RUNTIME\");\n");
      emitBodyLock(1, "if (antichainRuntimeEnv != nullptr && antichainRuntimeEnv[0] == '1') mtCoarseUseAntichainRuntime = true;\n");
      emitBodyLock(1, "mtWorkerPoolCoarseStaticRoundedWC = 0;\n");
      emitBodyLock(1, "mtWorkerPoolCoarseStaticBeginActiveWord = 0;\n");
      emitBodyLock(1, "mtWorkerPoolCoarseStaticActiveWordSpan = 0;\n");
      if (mtUseWaitProbeCodegen()) {
        emitBodyLock(1, "const char *waitProbeEnv = getenv(\"GSIM_MT_WAIT_PROBE\");\n");
        emitBodyLock(1, "mtWaitProbeEnabled = waitProbeEnv != nullptr && waitProbeEnv[0] != '\\0' && waitProbeEnv[0] != '0';\n");
        emitBodyLock(1, "mtWaitProbeWorkerFinishNs.assign((size_t)mtConfiguredWorkerCount, 0);\n");
        emitBodyLock(1, "mtWaitProbeWorkerFinishSumNs.assign((size_t)mtConfiguredWorkerCount, 0);\n");
        emitBodyLock(1, "mtWaitProbeWorkerLastHist.assign((size_t)mtConfiguredWorkerCount, 0);\n");
        emitBodyLock(1, "mtWaitProbeDispatchCount = 0;\n");
        emitBodyLock(1, "mtWaitProbeWaitSumNs = 0;\n");
        emitBodyLock(1, "mtWaitProbeW0BodySumNs = 0;\n");
        emitBodyLock(1, "mtWaitProbeTailBeyondW0SumNs = 0;\n");
        emitBodyLock(1, "mtWaitProbeMaxFinishSumNs = 0;\n");
        emitBodyLock(1, "mtWaitProbeMinBgFinishSumNs = 0;\n");
        emitBodyLock(1, "mtWaitProbeWorker0LastCount = 0;\n");
        emitBodyLock(1, "mtWaitProbeEmptyBarrierIters = 0;\n");
        emitBodyLock(1, "mtWaitProbeEmptyBarrierTotalNs = 0;\n");
      }
    }
  }
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "S%s::~S%s() {\n", name.c_str(), name.c_str());
  if (useMtHelpers && useCoarseMt && mtUseWaitProbeCodegen()) emitBodyLock(1, "runMtWaitProbeEmptyBarrier();\n");
  if (useMtHelpers) emitBodyLock(1, "stopMtWorkerPool();\n");
  emitBodyLock(1, "if (wallfracCommitBrackets + wallfracCombBrackets > 0) {\n");
  emitBodyLock(2, "uint64_t __wf_tot = wallfracCommitCycles + wallfracCombCycles;\n");
  emitBodyLock(2, "fprintf(stderr, \"[wallfrac] commit_cycles=%%lu comb_cycles=%%lu commit_brackets=%%lu comb_brackets=%%lu commit_frac=%%.4f comb_frac=%%.4f\\n\", wallfracCommitCycles, wallfracCombCycles, wallfracCommitBrackets, wallfracCombBrackets, __wf_tot? (double)wallfracCommitCycles/__wf_tot : 0.0, __wf_tot? (double)wallfracCombCycles/__wf_tot : 0.0);\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "dumpMtProfile();\n");
  if (useCoarseMt && mtUseWaitProbeCodegen()) emitBodyLock(1, "dumpMtWaitProbe();\n");
  emitBodyLock(1, "if (mtProfileDynamicTraceFile != nullptr) { fclose(mtProfileDynamicTraceFile); mtProfileDynamicTraceFile = nullptr; }\n");
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
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_serial_fallback eligible=%%lu taken=%%lu active_bits=%%lu repcut_excluded=%%lu nonpure_excluded=%%lu saved_worker_jobs=%%lu saved_flag_word_copies=%%lu saved_merge_word_scans=%%lu saved_barriers=%%lu\\n\", mtProfileCoarseSerialFallbackEligible, mtProfileCoarseSerialFallbackTaken, mtProfileCoarseSerialFallbackActiveBits, mtProfileCoarseSerialFallbackRepcutExcluded, mtProfileCoarseSerialFallbackNonPureExcluded, mtProfileCoarseSerialFallbackSavedWorkerJobs, mtProfileCoarseSerialFallbackSavedFlagWordCopies, mtProfileCoarseSerialFallbackSavedMergeWordScans, mtProfileCoarseSerialFallbackSavedBarriers);\n");
    if (mtUseSubchunkRuntime()) {
      emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_subchunk_dispatch cost_threshold=%%d min_active_bits=%%d eligible=%%lu taken=%%lu fully_inlined=%%lu skipped_below_min_active=%%lu inline_words=%%lu residual_dispatches=%%lu inline_active_bits=%%lu residual_initial_active_bits=%%lu\\n\", mtSubchunkDispatchCost, mtSubchunkMinActiveBits, mtProfileCoarseSubchunkDispatchEligible, mtProfileCoarseSubchunkDispatchTaken, mtProfileCoarseSubchunkDispatchFullyInlined, mtProfileCoarseSubchunkDispatchSkippedBelowMinActive, mtProfileCoarseSubchunkDispatchInlineWords, mtProfileCoarseSubchunkDispatchResidualDispatches, mtProfileCoarseSubchunkDispatchInlineActiveBits, mtProfileCoarseSubchunkDispatchResidualInitialActiveBits);\n");
    }
    if (mtUseSubchunkProbe()) {
      emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_subchunk_fallback codegen_enabled=1 dynamic_words=%%lu dynamic_tasks=%%lu dynamic_task_static_cost=%%lu\\n\", mtProfileCoarseSerialFallbackDynamicWords, mtProfileCoarseSerialFallbackDynamicTasks, mtProfileCoarseSerialFallbackDynamicTaskStaticCost);\n");
      emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_subchunk_word_cost_hist=%%lu,%%lu,%%lu,%%lu,%%lu,%%lu labels=0-63,64-127,128-255,256-511,512-1023,1024+ ge64=%%lu ge128=%%lu ge256=%%lu\\n\", mtProfileCoarseSerialFallbackDynamicWordCostHist[0], mtProfileCoarseSerialFallbackDynamicWordCostHist[1], mtProfileCoarseSerialFallbackDynamicWordCostHist[2], mtProfileCoarseSerialFallbackDynamicWordCostHist[3], mtProfileCoarseSerialFallbackDynamicWordCostHist[4], mtProfileCoarseSerialFallbackDynamicWordCostHist[5], mtProfileCoarseSerialFallbackDynamicWordCostGe64, mtProfileCoarseSerialFallbackDynamicWordCostGe128, mtProfileCoarseSerialFallbackDynamicWordCostGe256);\n");
      emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_subchunk_word_task_hist=%%lu,%%lu,%%lu,%%lu,%%lu,%%lu labels=1,2,3-4,5-8,9-15,16+ ge2=%%lu\\n\", mtProfileCoarseSerialFallbackDynamicWordTaskHist[0], mtProfileCoarseSerialFallbackDynamicWordTaskHist[1], mtProfileCoarseSerialFallbackDynamicWordTaskHist[2], mtProfileCoarseSerialFallbackDynamicWordTaskHist[3], mtProfileCoarseSerialFallbackDynamicWordTaskHist[4], mtProfileCoarseSerialFallbackDynamicWordTaskHist[5], mtProfileCoarseSerialFallbackDynamicWordTasksGe2);\n");
    }
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_layer_size_hist=%%lu,%%lu,%%lu,%%lu,%%lu,%%lu static=%%d,%%d,%%d,%%d,%%d,%%d labels=1,2,3-4,5-8,9-15,16+\\n\", mtProfileCoarseLayerSizeHist[0], mtProfileCoarseLayerSizeHist[1], mtProfileCoarseLayerSizeHist[2], mtProfileCoarseLayerSizeHist[3], mtProfileCoarseLayerSizeHist[4], mtProfileCoarseLayerSizeHist[5], %d, %d, %d, %d, %d, %d);\n",
                 mtCoarseProfileFacts.layerSizeHist[0], mtCoarseProfileFacts.layerSizeHist[1], mtCoarseProfileFacts.layerSizeHist[2],
                 mtCoarseProfileFacts.layerSizeHist[3], mtCoarseProfileFacts.layerSizeHist[4], mtCoarseProfileFacts.layerSizeHist[5]);
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_region_layer_count_hist=%%lu,%%lu,%%lu,%%lu,%%lu,%%lu labels=1,2,3-4,5-8,9-15,16+\\n\", mtProfileCoarseRegionLayerCountHist[0], mtProfileCoarseRegionLayerCountHist[1], mtProfileCoarseRegionLayerCountHist[2], mtProfileCoarseRegionLayerCountHist[3], mtProfileCoarseRegionLayerCountHist[4], mtProfileCoarseRegionLayerCountHist[5]);\n");
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_selected_worker_count_hist=\");\n");
    emitBodyLock(1, "for (size_t i = 0; i < mtProfileCoarseSelectedWorkerCountHist.size(); i ++) fprintf(stderr, \"%%s%%zu:%%lu\", i == 0 ? \"\" : \",\", i, mtProfileCoarseSelectedWorkerCountHist[i]);\n");
    emitBodyLock(1, "fprintf(stderr, \"\\n\");\n");
  }
  if (!mtProfileRepCutBatchBeginCppIds.empty()) {
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] repcut_batch_hits\");\n");
    for (size_t batchIndex = 0; batchIndex < mtProfileRepCutBatchBeginCppIds.size(); batchIndex ++) {
      emitBodyLock(1, "fprintf(stderr, \" %d:%%lu\", mtProfileRepCutBatchHits.size() > %zu ? mtProfileRepCutBatchHits[%zu] : 0);\n", mtProfileRepCutBatchBeginCppIds[batchIndex], batchIndex, batchIndex);
    }
    emitBodyLock(1, "fprintf(stderr, \"\\n\");\n");
  }
  emitBodyLock(1, "uint64_t mtProfileRepCutLiteTaskCallsTotal = 0;\n");
  for (int cppId : mtProfileRepCutRuntimeCppIds) {
    emitBodyLock(1, "mtProfileRepCutLiteTaskCallsTotal += mtProfileRepCutLiteTaskCallsByCppId[%d].load(std::memory_order_relaxed);\n", cppId);
  }
  emitBodyLock(1, "fprintf(stderr, \"[mt-profile] repcut_runtime cloned_task_calls=%%lu\\n\", mtProfileRepCutLiteTaskCallsTotal);\n");
  if (!mtProfileRepCutRuntimeCppIds.empty()) {
    emitBodyLock(1, "bool mtProfileRepcutFirstHelperCount = true;\n");
    for (int cppId : mtProfileRepCutRuntimeCppIds) {
      emitBodyLock(1, "{ uint64_t mtProfileRepcutHelperCount = mtProfileRepCutLiteTaskCallsByCppId[%d].load(std::memory_order_relaxed); if (mtProfileRepcutHelperCount != 0) { if (mtProfileRepcutFirstHelperCount) { fprintf(stderr, \"[mt-profile] repcut_runtime_by_cppid\"); mtProfileRepcutFirstHelperCount = false; } fprintf(stderr, \" %d:%%lu\", mtProfileRepcutHelperCount); } }\n", cppId, cppId);
    }
    emitBodyLock(1, "if (!mtProfileRepcutFirstHelperCount) fprintf(stderr, \"\\n\");\n");
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
  if (useCoarseMt && mtUseWaitProbeCodegen()) {
    emitFuncDecl(0, "void S%s::runMtWaitProbeEmptyBarrier() {\n", name.c_str());
    emitBodyLock(1, "if (!mtWaitProbeEnabled || !mtWorkerPoolEnabled || mtConfiguredWorkerCount <= 1) return;\n");
    emitBodyLock(1, "int wc = mtConfiguredWorkerCount;\n");
    emitBodyLock(1, "if (mtWorkerPoolThreadCount + 1 < wc) return;\n");
    emitBodyLock(1, "const char *itersEnv = getenv(\"GSIM_MT_WAIT_PROBE_ITERS\");\n");
    emitBodyLock(1, "uint64_t iters = (itersEnv != nullptr && itersEnv[0] != '\\0') ? strtoull(itersEnv, nullptr, 10) : 200000;\n");
    emitBodyLock(1, "if (iters == 0) return;\n");
    emitBodyLock(1, "mtWorkerPoolJobKind = 4;\n");
    emitBodyLock(1, "mtWorkerPoolCurrentWorkerCount = wc;\n");
    emitBodyLock(1, "for (uint64_t i = 0; i < 2000; i ++) { mtWorkerPoolPost(); mtWorkerPoolWaitForDone(wc - 1); }\n");
    emitBodyLock(1, "std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();\n");
    emitBodyLock(1, "for (uint64_t i = 0; i < iters; i ++) { mtWorkerPoolPost(); mtWorkerPoolWaitForDone(wc - 1); }\n");
    emitBodyLock(1, "mtWaitProbeEmptyBarrierTotalNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count();\n");
    emitBodyLock(1, "mtWaitProbeEmptyBarrierIters = iters;\n");
    emitBodyLock(0, "}\n");

    emitFuncDecl(0, "void S%s::dumpMtWaitProbe() {\n", name.c_str());
    emitBodyLock(1, "if (!mtWaitProbeEnabled) return;\n");
    emitBodyLock(1, "double empAvgNs = mtWaitProbeEmptyBarrierIters ? (double)mtWaitProbeEmptyBarrierTotalNs / (double)mtWaitProbeEmptyBarrierIters : 0.0;\n");
    emitBodyLock(1, "uint64_t dc = mtWaitProbeDispatchCount;\n");
    emitBodyLock(1, "fprintf(stderr, \"[mt-wait-probe] empty_barrier_iters=%%lu empty_barrier_total_ns=%%lu empty_barrier_ns_per_iter=%%.2f worker_count=%%d\\n\", mtWaitProbeEmptyBarrierIters, mtWaitProbeEmptyBarrierTotalNs, empAvgNs, mtConfiguredWorkerCount);\n");
    emitBodyLock(1, "fprintf(stderr, \"[mt-wait-probe] dispatch_count=%%lu wait_sum_ns=%%lu w0_body_sum_ns=%%lu tail_beyond_w0_sum_ns=%%lu max_finish_sum_ns=%%lu min_bg_finish_sum_ns=%%lu worker0_last_count=%%lu\\n\", dc, mtWaitProbeWaitSumNs, mtWaitProbeW0BodySumNs, mtWaitProbeTailBeyondW0SumNs, mtWaitProbeMaxFinishSumNs, mtWaitProbeMinBgFinishSumNs, mtWaitProbeWorker0LastCount);\n");
    emitBodyLock(1, "if (dc) fprintf(stderr, \"[mt-wait-probe] avg_wait_ns=%%.2f avg_w0_body_ns=%%.2f avg_tail_beyond_w0_ns=%%.2f avg_max_finish_ns=%%.2f tail_share=%%.4f\\n\", (double)mtWaitProbeWaitSumNs/dc, (double)mtWaitProbeW0BodySumNs/dc, (double)mtWaitProbeTailBeyondW0SumNs/dc, (double)mtWaitProbeMaxFinishSumNs/dc, mtWaitProbeWaitSumNs ? (double)mtWaitProbeTailBeyondW0SumNs/(double)mtWaitProbeWaitSumNs : 0.0);\n");
    emitBodyLock(1, "fprintf(stderr, \"[mt-wait-probe] worker_last_hist=\");\n");
    emitBodyLock(1, "for (size_t i = 0; i < mtWaitProbeWorkerLastHist.size(); i ++) fprintf(stderr, \"%%s%%zu:%%lu\", i == 0 ? \"\" : \",\", i, mtWaitProbeWorkerLastHist[i]);\n");
    emitBodyLock(1, "fprintf(stderr, \"\\n\");\n");
    emitBodyLock(1, "fprintf(stderr, \"[mt-wait-probe] worker_finish_avg_ns=\");\n");
    emitBodyLock(1, "for (size_t i = 0; i < mtWaitProbeWorkerFinishSumNs.size(); i ++) fprintf(stderr, \"%%s%%zu:%%.2f\", i == 0 ? \"\" : \",\", i, dc ? (double)mtWaitProbeWorkerFinishSumNs[i]/dc : 0.0);\n");
    emitBodyLock(1, "fprintf(stderr, \"\\n\");\n");
    emitBodyLock(0, "}\n");
  }

  /* activation all nodes for reset */
  fprintf(header, "void activateAll();\n");
  emitFuncDecl(0, "void S%s::activateAll() {\n"
               "  memset(activeFlags, 0xff, sizeof(activeFlags));\n"
               "}\n", name.c_str());

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
    fprintf(header, "void subReset%d();\n", i);
    if (denseExecutorValid) fprintf(header, "void subResetDense%d();\n", i);
    if (globalConfig.MtHelperMode == "buffered-seq") fprintf(header, "void subReset%d(ActiveBuffer &nextActive);\n", i);
    if (useMtHelpers) fprintf(header, "void subReset%d(ActivationDelta &nextActive);\n", i);
  }

  /* main evaluation loop (step) */
  int subStepIdxMax = 0;
  int serialFastSubStepMax = -1;
  std::string serialFastSuffix;
  if (useMtHelpers) {
    serialFastSuffix = "SerialFast";
    serialFastSubStepMax = genActivate(serialFastSuffix);
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
      if (useMtHelpers) fprintf(header, "void mtTask%d(uint%d_t &flag, ActivationDelta &nextActive);\n", i, ACTIVE_WIDTH);
      if (!useBufferedHelpers || useMtHelpers) fprintf(header, "void mtTask%d(uint%d_t &flag);\n", i, ACTIVE_WIDTH);
    }
    if (useMtHelpers) {
      for (int i = 0; i < superId; i ++) {
        if (mtRepCutHeaderTasks[i].repcutRuntimeApplied) {
          fprintf(header, "void mtRepCutLiteTask%d(uint%d_t &flag, ActivationDelta &nextActive);\n", i, ACTIVE_WIDTH);
        }
      }
    }
    if (useMtHelpers) {
      int shardCount = mtPureBatchShardCount();
      for (int shard = 0; shard < shardCount; shard ++) {
        fprintf(header, "void mtRunPureBatchDirectShard%d(int chunkBegin, int chunkEnd, uint%d_t &activeWord);\n", shard, ACTIVE_WIDTH);
        fprintf(header, "void mtRunPureBatchWorkerShard%d(int worker, int chunkBegin, int chunkEnd, std::vector<std::vector<int>> &mtProfileLocalTaskIds, std::vector<uint64_t> &mtProfileLocalWorkerTaskCount);\n", shard);
      }
      fprintf(header, "void mtRunPureBatchWorkerRange(int worker, int chunkBegin, int chunkEnd);\n");
      fprintf(header, "void mtWorkerPoolPause();\n");
      fprintf(header, "void mtWorkerPoolPost();\n");
      fprintf(header, "void mtWorkerPoolWaitForDone(int expectedDoneCount);\n");
      fprintf(header, "void mtWorkerPoolLoop(int worker);\n");
      fprintf(header, "void startMtWorkerPool();\n");
      fprintf(header, "void stopMtWorkerPool();\n");
      fprintf(header, "void mtRunPureBatch(int beginCppId, int endCppId, uint%d_t &activeWord);\n", ACTIVE_WIDTH);
      if (useCoarseMt) {
        fprintf(header, "void mtRunCoarseLayerWorkerRange(int worker, int regionIndex, int layerIndex, int chunkBegin, int chunkEnd);\n");
        fprintf(header, "void mtMergeLocalCoarseDelta(int worker, int regionBeginActiveWord, int regionActiveWordSpan);\n");
        fprintf(header, "void mtRunCoarseMTaskWorkerList(int worker, int regionIndex, const int *mtaskIndices, int mtaskCount);\n");
        fprintf(header, "void mtRunCoarseMTaskWorkerRange(int worker, int regionIndex, int mtaskBegin, int mtaskEnd);\n");
        fprintf(header, "void mtRunCoarseMTaskDynamic(int regionIndex, int worker);\n");
        // Track 2 Week 7: antichain ready-queue helpers.
        fprintf(header, "void mtCoarseReadyQueuePush(int regionIndex, int mtaskIndex, bool worker0Only);\n");
        fprintf(header, "int mtCoarseReadyQueuePop(int regionIndex, int worker);\n");
        fprintf(header, "int mtCountActiveCoarseMTasks(int regionIndex, uint%d_t *coarseActiveWords, int *activeStaticCost);\n", ACTIVE_WIDTH);
        fprintf(header, "void mtBuildCoarseMTaskWorkerAssignment(int regionIndex, int workerCount, std::vector<std::vector<int>> &assignments, std::vector<uint64_t> &workerStaticCosts, std::vector<uint64_t> &workerTaskCounts);\n");
        fprintf(header, "void mtRunCoarseRegion(int regionIndex, uint%d_t *coarseActiveWords);\n", ACTIVE_WIDTH);
        // 28c D-static Step 1: codegen-time LPT + flat per-cppId arrays.
        fprintf(header, "void mtRunCoarseStaticRefList(int regionIndex, int roundedWC, int worker, int regionBeginActiveWord, int regionActiveWordSpan, const SCoarseTaskRef *refs, int refCount);\n");
        fprintf(header, "void mtRunCoarseRegionStaticDispatch(int regionIndex, int roundedWC, int worker, int regionBeginActiveWord, int regionActiveWordSpan);\n");
        {
          MtCoarseRegionPlan dstaticPlan = planMtCoarseRegionsForInvocation(mtRepCutHeaderTasks);
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
  fprintf(header, "};\n"
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
  std::cout << "[cppEmitter] finish writing " << srcFileIdx << " cpp files to " + globalConfig.OutputDir + "/" << std::endl;
}
