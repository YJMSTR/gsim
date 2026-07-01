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
#include <map>
#include <set>
#include <stack>
#include <string>
#include <tuple>
#include <utility>
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

static std::map<Node*, std::pair<int, int>> super2ResetId;  // uint & async reset

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

static bool mtTaskHasActiveEdgeTo(int fromCppId, int toCppId) {
  auto iter = cppId2Super.find(fromCppId);
  if (iter == cppId2Super.end() || !iter->second) return false;
  for (Node* member : iter->second->member) {
    if (member && member->nextActiveId.find(toCppId) != member->nextActiveId.end()) return true;
  }
  return false;
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
  auto from = cppId2Super.find(fromCppId);
  auto to = cppId2Super.find(toCppId);
  if (from == cppId2Super.end() || to == cppId2Super.end() || !from->second || !to->second) return false;
  if (hasCppId(from->second->next, toCppId) || hasCppId(from->second->depNext, toCppId) ||
      hasCppId(to->second->prev, fromCppId) || hasCppId(to->second->depPrev, fromCppId)) {
    return true;
  }
  return false;
}

static bool mtTaskHasOrderingEdgeTo(int fromCppId, int toCppId) {
  return mtTaskHasDependencyEdgeTo(fromCppId, toCppId) || mtTaskHasActiveEdgeTo(fromCppId, toCppId);
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

static MtCoarseRegion mtBuildCoarseRegion(const std::map<int, MtTaskInfo>& tasks, int beginCppId, int endCppId) {
  MtCoarseRegion region;
  region.beginCppId = beginCppId;
  region.endCppId = endCppId;
  region.beginActiveWord = beginCppId / ACTIVE_WIDTH;
  region.endActiveWord = (endCppId - 1) / ACTIVE_WIDTH + 1;
  region.taskCount = endCppId - beginCppId;
  region.activeWordSpan = region.endActiveWord - region.beginActiveWord;
  region.staticCost = mtBatchEstimatedCost(tasks, beginCppId, endCppId);
  region.memberNodeCost = mtBatchMemberNodeCost(beginCppId, endCppId);
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
      if (mtCanCutEdge(tasks, from, to)) {
        region.replicationCandidateCount ++;
        region.repcutLiteCouldHelp = true;
      }
    }
  }

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
  mtAddCoarseLayers(region);
  if (region.layers.empty()) {
    region.runtimeEligible = false;
  }
  if (region.estimatedMaxParallelWidth < 2) {
    mtAddCoarseBlocker(region, "codegen_runtime_limit");
    region.runtimeEligible = false;
  }
  if (region.runtimeEligible) mtAddCoarseMTasks(region, tasks);
  mtFinalizeCoarseProfitability(region);
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
  bool levelDispatch = mtIsLevelDispatchMode();
  for (int beginWord = 0; beginWord * ACTIVE_WIDTH < superId; beginWord ++) {
    int beginCppId = beginWord * ACTIVE_WIDTH;
    if (!mtCoarseWordCanEnterRegion(tasks, beginCppId)) continue;
    if (levelDispatch && mtCoarseWordHasSameWordReverseOrderingEdge(beginCppId)) continue;
    int endWord = beginWord;
    while (endWord * ACTIVE_WIDTH < superId) {
      int wordBegin = endWord * ACTIVE_WIDTH;
      if (!mtCoarseWordCanEnterRegion(tasks, wordBegin)) break;
      if (levelDispatch) {
        if (mtCoarseWordHasSameWordReverseOrderingEdge(wordBegin)) break;
        if (mtCoarseWordHasReverseOrderingEdgeIntoRegion(beginCppId, wordBegin)) break;
        if ((endWord + 1) - beginWord > MT_LEVEL_DISPATCH_REGION_SPAN_CAP) break;
      }
      endWord ++;
    }
    int endCppId = endWord * ACTIVE_WIDTH;
    if (endCppId - beginCppId >= ACTIVE_WIDTH) {
      MtCoarseRegion region = mtBuildCoarseRegion(tasks, beginCppId, endCppId);
      plan.regions.push_back(region);
      beginWord = std::max(beginWord, endWord - 1);
    }
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
  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapWithRepCut();
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
}

void graph::dumpMtRepCutLiteReport() {
  std::string baseName = globalConfig.InputBaseName.empty() ? name : globalConfig.InputBaseName;
  std::string path = globalConfig.OutputDir + "/" + baseName + "_mt_repcut_lite.json";
  FILE* fp = std::fopen(path.c_str(), "w");
  Assert(fp != nullptr, "failed to open mt repcut-lite report %s", path.c_str());
  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapWithRepCutSelection();
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
}

void graph::dumpMtCoarseRegionReport() {
  std::string baseName = globalConfig.InputBaseName.empty() ? name : globalConfig.InputBaseName;
  std::string path = globalConfig.OutputDir + "/" + baseName + "_mt_coarse_regions.json";
  FILE* fp = std::fopen(path.c_str(), "w");
  Assert(fp != nullptr, "failed to open mt coarse-region report %s", path.c_str());
  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapWithRepCutSelection();
  const char* segmentReportEnv = std::getenv("GSIM_MT_SEGMENT_REPORT");
  bool segmentReportEnabled = segmentReportEnv != nullptr && segmentReportEnv[0] != '\0' && segmentReportEnv[0] != '0';
  if (segmentReportEnabled) markMtRepCutLiteRuntimeApplied(mtTasks);
  MtCoarseRegionPlan coarsePlan = planMtCoarseRegions(mtTasks);
  MtPureBatchPlan fallbackPlan = planMtPureBatchesActiveFrequency(mtTasks, globalConfig.MtRepCutLiteMode == "on");

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
  fprintf(fp, "  \"regions\": [\n");
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
  fprintf(fp, "  ]\n");
  fprintf(fp, "}\n");
  fclose(fp);
  printf("[mt-coarse-region] wrote %zu regions (%d runtime eligible) to %s\n",
         coarsePlan.regions.size(), runtimeEligibleCount, path.c_str());
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
  FILE* header = std::fopen((globalConfig.OutputDir + "/" + name + ".h").c_str(), "w");

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
                         const std::string& accumFlagName) {
  std::string nodeName = node->name;
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
                               const std::string& accumFlagName) {
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
int graph::genNodeStepStart(SuperNode* node, uint64_t mask, int idx, std::string flagName, int indent) {
  nodeNum ++;
  if (!isAlwaysActive(node->cppId)) {
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

int graph::genNodeStepEnd(SuperNode* node, int indent) {
#ifdef PERF
  if (node->superType != SUPER_EXTMOD) {
    emitBodyLock(indent, "validActive[%d] += isActivateValid;\n", node->cppId);
  }
#endif

  if(!isAlwaysActive(node->cppId)) {
    emitBodyLock(-- indent, "}\n");
  }
  return indent;
}

bool Node::isLocal() { // TODO: isArray is OK
  return status == VALID_NODE && type == NODE_OTHERS && !anyNextActive() && !isArray() && !isReset();
}

static std::map<Node*, std::string> mtRepCutActiveReplacements;

int graph::translateInst(InstInfo inst, int indent, std::string flagName, std::string activeBufferName, const std::string& accumFlagName) {
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
      if (inst.node->isArray() || inst.node->type == NODE_WRITER) activateUncondNext(inst.node, inst.node->nextActiveId, false, flagName, activeBufferName, indent, accumFlagName);
      else activateNext(inst.node, inst.node->nextActiveId, oldName(inst.node), false, flagName, activeBufferName, indent, accumFlagName);
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

void graph::genSuperEval(SuperNode* super, std::string flagName, std::string activeBufferName, int indent) { // current indent = 2
  bool useAccum = mtActAccEnabled() && activeBufferName.empty() && super->superType != SUPER_EXTMOD && super->superType != SUPER_ASYNC_RESET;
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
      indent = translateInst(inst, indent, flagName, activeBufferName, accumVar);
    }
    for (size_t i = 1; i < super->member.size(); i ++) {
      if (!super->member[i]->needActivate()) continue;
      if (super->member[i]->isArray()) activateUncondNext(super->member[i], super->member[i]->nextActiveId, false, flagName, activeBufferName, indent, accumVar);
      else activateNext(super->member[i], super->member[i]->nextActiveId, oldName(super->member[i]), false, flagName, activeBufferName, indent, accumVar);
    }
  } else {
    if (super->superType == SUPER_ASYNC_RESET) {
      if (activeBufferName.empty()) emitBodyLock(indent, "subReset%d();\n", super2ResetId[super->resetNode].second);
      else emitBodyLock(indent, "subReset%d(%s);\n", super2ResetId[super->resetNode].second, activeBufferName.c_str());
    }
    /* local nodes definition */
    for (Node* n : super->member) {
      if (n->isLocal()) {
        emitBodyLock(indent, "%s %s;\n", widthUType(n->width).c_str(), n->name.c_str());
      }
    }
    for (InstInfo inst : super->insts) {
      indent = translateInst(inst, indent, flagName, activeBufferName, accumVar);
    }
    if (super->superType == SUPER_ASYNC_RESET) {
      if (activeBufferName.empty()) emitBodyLock(indent, "subReset%d();\n", super2ResetId[super->resetNode].second);
      else emitBodyLock(indent, "subReset%d(%s);\n", super2ResetId[super->resetNode].second, activeBufferName.c_str());
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
      indent = genNodeStepStart(super, mask, idx, flagName, indent);
      static bool wallfracAudit = (std::getenv("GSIM_WALLFRAC_AUDIT") && std::string(std::getenv("GSIM_WALLFRAC_AUDIT")) == "1");
      bool wfCommit = (super->superType == SUPER_UPDATE_REG);
      for (Node* wfm : super->member) { if (nodeHasStateUpdate(wfm)) { wfCommit = true; break; } }
      if (wallfracAudit) {
        emitBodyLock(indent, "uint32_t __wf_a0_%d,__wf_d0_%d; __asm__ __volatile__(\"rdtsc\":\"=a\"(__wf_a0_%d),\"=d\"(__wf_d0_%d)); uint64_t __wf_t0_%d=((uint64_t)__wf_d0_%d<<32)|__wf_a0_%d;\n", idx, idx, idx, idx, idx, idx, idx);
      }
      genSuperEval(super, flagName, "", indent);
      if (wallfracAudit) {
        emitBodyLock(indent, "uint32_t __wf_a1_%d,__wf_d1_%d; __asm__ __volatile__(\"rdtsc\":\"=a\"(__wf_a1_%d),\"=d\"(__wf_d1_%d)); uint64_t __wf_t1_%d=((uint64_t)__wf_d1_%d<<32)|__wf_a1_%d;\n", idx, idx, idx, idx, idx, idx, idx);
        emitBodyLock(indent, "%s += __wf_t1_%d-__wf_t0_%d; %s ++;\n", wfCommit?"wallfracCommitCycles":"wallfracCombCycles", idx, idx, wfCommit?"wallfracCommitBrackets":"wallfracCombBrackets");
      }
      indent = genNodeStepEnd(super, indent);
    }
    emitBodyLock(--indent, "}\n");
    if (prevActiveWhole) emitBodyLock(--indent, "}\n");

    return nextSubStepIdx - 1; // return the maxinum subStepIdx currently used
}

void graph::genMtTaskHelper(SuperNode* super, bool buffered, const std::string& activeSinkType) {
  if (buffered) {
    emitFuncDecl(0, "void S%s::mtTask%d(uint%d_t &flag, %s &nextActive) {\n", name.c_str(), super->cppId, ACTIVE_WIDTH, activeSinkType.c_str());
    genSuperEval(super, "flag", "nextActive", 1);
  } else {
    emitFuncDecl(0, "void S%s::mtTask%d(uint%d_t &flag) {\n", name.c_str(), super->cppId, ACTIVE_WIDTH);
    genSuperEval(super, "flag", "", 1);
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
  genSuperEval(super, "flag", "nextActive", 1);
  mtRepCutActiveReplacements.clear();
  emitBodyLock(0, "}\n");
}

void graph::genMtTaskRunner(const MtRepCutSemanticPlan& semanticPlan) {
  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapWithRepCutSelection();
  markMtRepCutLiteRuntimeApplied(mtTasks);
  int shardCount = mtPureBatchShardCount();
  bool useCoarse = globalConfig.MtBatchFormationMode == "coarse";
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
  if (useCoarse) {
    emitBodyLock(2, "const int jobKind = mtWorkerPoolJobKind;\n");
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
    emitBodyLock(3, "if (mtWaitProbeEnabled && (size_t)worker < mtWaitProbeWorkerFinishNs.size()) mtWaitProbeWorkerFinishNs[(size_t)worker] = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtWaitProbePostTp).count();\n");
    emitBodyLock(2, "} else if (jobKind == 4) {\n");
    emitBodyLock(3, "/* A35-P empty-barrier microbench: worker performs no work */\n");
    emitBodyLock(2, "} else if (jobKind == 5) {\n");
    emitBodyLock(3, "mtRunCoarseMTaskDynamic(coarseRegionIndex, worker);\n");
    emitBodyLock(2, "} else {\n");
    emitBodyLock(3, "mtRunPureBatchWorkerRange(worker, chunkBegin, chunkEnd);\n");
    emitBodyLock(2, "}\n");
  } else {
    emitBodyLock(2, "mtRunPureBatchWorkerRange(worker, chunkBegin, chunkEnd);\n");
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
  if (useCoarse) {
    emitBodyLock(2, "mtWorkerPoolJobKind = 0;\n");
  }
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
  std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapWithRepCutSelection();
  markMtRepCutLiteRuntimeApplied(mtTasks);
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
    emitBodyLock(1, "if (mtCoarseUseMTaskRuntime) {\n");
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
  emitBodyLock(1, "if (mtCoarseUseAntichainRuntime && kCoarseRegionUseAntichainRuntime[regionIndex]) {\n");
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
  emitBodyLock(1, "if (mtCoarseUseMTaskRuntime) {\n");
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
  emitBodyLock(1, "if (mtCoarseUseDStatic) {\n");
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
  emitBodyLock(1, "if (mtCoarseUseDStatic) {\n");
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
  emitBodyLock(3, "if (mtWaitProbeEnabled) mtWaitProbePostTp = std::chrono::steady_clock::now();\n");
  emitBodyLock(3, "mtWorkerPoolPost();\n");
  emitBodyLock(3, "mtRunCoarseRegionStaticDispatch(regionIndex, dstaticRoundedWC, 0, regionBeginActiveWord, regionActiveWordSpan);\n");
  emitBodyLock(3, "if (mtWaitProbeEnabled && !mtWaitProbeWorkerFinishNs.empty()) mtWaitProbeWorkerFinishNs[0] = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtWaitProbePostTp).count();\n");
  emitBodyLock(3, "std::chrono::steady_clock::time_point mtPhaseWaitBegin;\n");
  emitBodyLock(3, "if (mtProfileEnabled) {\n");
  emitBodyLock(4, "mtPhaseWaitBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(4, "mtProfileCoarseBodyNs += std::chrono::duration_cast<std::chrono::nanoseconds>(mtPhaseWaitBegin - mtPhaseBodyBegin).count();\n");
  emitBodyLock(3, "}\n");
  emitBodyLock(3, "mtWorkerPoolWaitForDone(dstaticRoundedWC - 1);\n");
  emitBodyLock(3, "if (mtProfileEnabled) mtProfileCoarseWaitNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtPhaseWaitBegin).count();\n");
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
    std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapWithRepCut();
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
      indent = genNodeStepStart(super, mask, idx, flagName, indent);
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
      indent = genNodeStepEnd(super, indent);
    }
    emitBodyLock(--indent, "}\n");
    if (prevActiveWhole) emitBodyLock(--indent, "}\n");

    return nextSubStepIdx - 1;
}

int graph::genActivateMtHelpers(int serialFastSubStepMax, const std::string& serialFastSuffix) {
    std::map<int, MtTaskInfo> mtTasks = buildMtTaskInfoMapWithRepCutSelection();
    markMtRepCutLiteRuntimeApplied(mtTasks);
    MtRepCutSemanticPlan semanticPlan = planMtRepCutSemantics(mtTasks);
    MtCoarseRegionPlan coarsePlan = planMtCoarseRegions(mtTasks);
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
    int nextSubStepIdx = 1;
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
                genSuperEval(cppId2Super[cppId], format("coarseInlineFlag%d_%d", idx, word), "", taskIndent);
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
                genSuperEval(cppId2Super[cppId], format("coarseInlineFlag%d_%d", idx, word), "", taskIndent);
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
        if (regionCleanSerialFallback) {
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
            nextFuncDef = format("void S%s::subStep%d()", name.c_str(), ++ nextSubStepIdx);
          }
          emitBodyLock(indent, "uint%d_t oldFlag = activeFlags[%d];\n", ACTIVE_WIDTH, id);
          emitBodyLock(indent, "activeFlags[%d] = 0;\n", id);
          emitBodyLock(indent, "if (mtProfileEnabled) mtProfileActiveWordCount ++;\n");
        } else {
          emitBodyLock(indent, "uint%d_t activeWord%d = activeFlags[%d];\n", ACTIVE_WIDTH, id, id);
        }
      }

      auto batchIter = batchEndByStart.find(idx);
      if (prevActiveWhole && batchIter != batchEndByStart.end()) {
        int batchEnd = batchIter->second;
        if (batchEnd - idx > 1) {
          emitBodyLock(indent, "mtRunPureBatch(%d, %d, oldFlag);\n", idx, batchEnd);
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
      indent = genNodeStepStart(super, mask, idx, flagName, indent);
      if (profileOffDirectSerial) {
        // A77 D1-NARROW: emit only the lean profile-off body, matching the
        // SerialFast shape (genActivate). Drops both `if (mtProfileEnabled)`
        // wrappers and the outer scope; profile counters/timers are not emitted.
        if (directInlineSerialTask || directInlineWorker0Task) {
          genSuperEval(super, flagName, "", indent);
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
        genSuperEval(super, flagName, "", indent + 1);
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
      indent = genNodeStepEnd(super, indent);
    }
    emitBodyLock(--indent, "}\n");
    if (prevActiveWhole) emitBodyLock(--indent, "}\n");

    return nextSubStepIdx - 1;
}

void graph::genResetDef(SuperNode* super, bool isUIntReset, bool buffered, int resetId, int indent) {
  std::string activeSinkType = (globalConfig.MtHelperMode == "mt" ||
                                globalConfig.MtHelperMode == "mt-level-dispatch")
                                 ? "ActivationDelta" : "ActiveBuffer";
  if (buffered) emitBodyLock(indent ++, "void S%s::subReset%d(%s &nextActive){ // %s reset\n", name.c_str(), resetId, activeSinkType.c_str(), isUIntReset ? "uint" : "async");
  else emitBodyLock(indent ++, "void S%s::subReset%d(){ // %s reset\n", name.c_str(), resetId, isUIntReset ? "uint" : "async");
  std::string resetName = super->resetNode->type == NODE_REG_SRC ? RESET_NAME(super->resetNode).c_str() : super->resetNode->name.c_str();
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
  emitBodyLock(-- indent, "}\n");
}

void graph::genResetActivation(SuperNode* super, bool isUIntReset, int indent, int resetId) {
  emitBodyLock(indent, "subReset%d();\n", resetId);
}

void graph::genResetAll() {
  std::vector<SuperNode*> resetSuper;
  for (SuperNode* super : allReset) {
    if (super->resetNode->status == CONSTANT_NODE) {
      Assert(mpz_sgn(super->resetNode->computeInfo->consVal) == 0, "reset %s is always true", super->resetNode->name.c_str());
      continue;
    }
    if (super2ResetId.find(super->resetNode) != super2ResetId.end()) {
      super2ResetId[super->resetNode] = std::make_pair(-1, -1);
    }
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

void graph::genStep(int subStepIdxMax, int serialFastSubStepMax, const std::string& serialFastSuffix) {
  emitFuncDecl(0, "void S%s::step() {\n", name.c_str());
  emitBodyLock(1, "std::chrono::steady_clock::time_point mtProfileStepBegin;\n");
  emitBodyLock(1, "if (mtProfileEnabled) mtProfileStepBegin = std::chrono::steady_clock::now();\n");
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
  for (int i = 0; i <= subStepIdxMax; i ++) {
    emitBodyLock(1, "subStep%d();\n", i);
  }

  // Dump before cycles++ so the trace line names the cycle whose substeps just ran.
  emitBodyLock(1, "dumpMtProfileDynamicTraceCycle();\n");
  emitBodyLock(1, "cycles ++;\n");
  emitBodyLock(1, "if (mtProfileEnabled) mtProfileTotalStepNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileStepBegin).count();\n");
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
    }
    srcFp = std::fopen(format("%s%d.cpp", (globalConfig.OutputDir + "/" + name).c_str(), srcFileIdx).c_str(), "w");
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
  if (globalConfig.DumpMtScheduleJson) dumpMtScheduleJson();
  if (globalConfig.DumpMtRepCutLiteReport || globalConfig.MtRepCutLiteMode == "on") dumpMtRepCutLiteReport();
  if (globalConfig.DumpMtCoarseRegionReport || globalConfig.MtBatchFormationMode == "coarse") dumpMtCoarseRegionReport();

  // 28c Phase 1A: remove stale SimTop*.cpp files from previous runs so the
  // linker never sees a cppEmitter file the current run did not regenerate.
  for (int staleIdx = 0; ; staleIdx ++) {
    std::string stalePath = format("%s%d.cpp", (globalConfig.OutputDir + "/" + name).c_str(), staleIdx);
    if (std::remove(stalePath.c_str()) != 0) break;
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
  if (useMtHelpers) {
    mtRepCutHeaderTasks = buildMtTaskInfoMapWithRepCutSelection();
    markMtRepCutLiteRuntimeApplied(mtRepCutHeaderTasks);
    MtRepCutSemanticPlan mtRepCutHeaderSemanticPlan = planMtRepCutSemantics(mtRepCutHeaderTasks);
    mtSetProfileRepCutBatchBeginCppIds(mtRepCutHeaderSemanticPlan);
    mtSetProfileRepCutRuntimeCppIds(mtRepCutHeaderTasks);
    if (useCoarseMt) {
      mtCoarseProfileFacts = mtComputeCoarseProfileFacts(planMtCoarseRegions(mtRepCutHeaderTasks));
    }
  }
  if (globalConfig.MtHelperMode == "buffered-seq") emitActiveBufferDef(header, activeFlagNum);
  if (useMtHelpers) emitActivationDeltaDef(header, activeFlagNum);

  fprintf(header, "class S%s {\npublic:\n", name.c_str());
  fprintf(header, "uint64_t cycles;\n");
  fprintf(header, "uint64_t LOG_START, LOG_END;\n");
  fprintf(header, "uint%d_t activeFlags[%d];\n", ACTIVE_WIDTH, activeFlagNum); // or super.size() if id == idx
  fprintf(header, "bool mtProfileEnabled;\n");
  fprintf(header, "FILE *mtProfileDynamicTraceFile;\n");
  fprintf(header, "uint64_t mtProfileDynamicTraceCycleStart;\n");
  fprintf(header, "uint64_t mtProfileDynamicTraceCycleLimit;\n");
  fprintf(header, "std::vector<int> mtProfileDynamicTraceTaskIds;\n");
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
      fprintf(header, "int mtWorkerPoolJobKind;\n");
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
    fprintf(header, "bool mtWorkerPoolEnabled;\n");
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
  if (useMtHelpers) emitBodyLock(1, "startMtWorkerPool();\n");
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
  if (useCoarseMt) {
    fprintf(header, "void runMtWaitProbeEmptyBarrier();\n");
    fprintf(header, "void dumpMtWaitProbe();\n");
  }

  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "void S%s::initMtProfile() {\n", name.c_str());
  emitBodyLock(1, "const char *profileEnv = getenv(\"GSIM_MT_PROFILE\");\n");
  emitBodyLock(1, "const char *fireProfileEnv = getenv(\"GSIM_MT_FIRE_PROFILE\");\n");
  emitBodyLock(1, "const char *dynamicTraceEnv = getenv(\"GSIM_MT_DYNAMIC_TRACE\");\n");
  emitBodyLock(1, "bool dynamicTraceEnabled = dynamicTraceEnv != nullptr && dynamicTraceEnv[0] != '\\0';\n");
  emitBodyLock(1, "mtProfileEnabled = (profileEnv != nullptr && profileEnv[0] != '\\0' && profileEnv[0] != '0') || (fireProfileEnv != nullptr && fireProfileEnv[0] != '\\0' && fireProfileEnv[0] != '0') || dynamicTraceEnabled;\n");
  emitBodyLock(1, "mtProfileHelperMode = \"%s\";\n", globalConfig.MtHelperMode.c_str());
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
    emitBodyLock(1, "mtWorkerPoolThreadCount = 0;\n");
    emitBodyLock(1, "mtWorkerPoolGeneration.store(0, std::memory_order_relaxed);\n");
    emitBodyLock(1, "mtWorkerPoolStop.store(false, std::memory_order_relaxed);\n");
    emitBodyLock(1, "mtWorkerPoolDoneCount.store(0, std::memory_order_relaxed);\n");
    emitBodyLock(1, "mtWorkerPoolReadyCount.store(0, std::memory_order_relaxed);\n");
    emitBodyLock(1, "mtWorkerPoolCurrentWorkerCount = 0;\n");
    if (useCoarseMt) {
      emitBodyLock(1, "mtWorkerPoolJobKind = 0;\n");
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
      // Track 2 Week 4: env GSIM_MT_ANTICHAIN_RUNTIME=1 enables per-mtask
      // atomic-counter scheduler for antichain-enabled coarse regions.
      emitBodyLock(1, "mtCoarseUseAntichainRuntime = false;\n");
      emitBodyLock(1, "const char *antichainRuntimeEnv = getenv(\"GSIM_MT_ANTICHAIN_RUNTIME\");\n");
      emitBodyLock(1, "if (antichainRuntimeEnv != nullptr && antichainRuntimeEnv[0] == '1') mtCoarseUseAntichainRuntime = true;\n");
      emitBodyLock(1, "mtWorkerPoolCoarseStaticRoundedWC = 0;\n");
      emitBodyLock(1, "mtWorkerPoolCoarseStaticBeginActiveWord = 0;\n");
      emitBodyLock(1, "mtWorkerPoolCoarseStaticActiveWordSpan = 0;\n");
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
  emitBodyLock(0, "}\n");

  emitFuncDecl(0, "S%s::~S%s() {\n", name.c_str(), name.c_str());
  if (useMtHelpers && useCoarseMt) emitBodyLock(1, "runMtWaitProbeEmptyBarrier();\n");
  if (useMtHelpers) emitBodyLock(1, "stopMtWorkerPool();\n");
  emitBodyLock(1, "if (wallfracCommitBrackets + wallfracCombBrackets > 0) {\n");
  emitBodyLock(2, "uint64_t __wf_tot = wallfracCommitCycles + wallfracCombCycles;\n");
  emitBodyLock(2, "fprintf(stderr, \"[wallfrac] commit_cycles=%%lu comb_cycles=%%lu commit_brackets=%%lu comb_brackets=%%lu commit_frac=%%.4f comb_frac=%%.4f\\n\", wallfracCommitCycles, wallfracCombCycles, wallfracCommitBrackets, wallfracCombBrackets, __wf_tot? (double)wallfracCommitCycles/__wf_tot : 0.0, __wf_tot? (double)wallfracCombCycles/__wf_tot : 0.0);\n");
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "dumpMtProfile();\n");
  if (useCoarseMt) emitBodyLock(1, "dumpMtWaitProbe();\n");
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
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] helper_mode=%%s worker_count=%%d worker_pool=%%d min_batch_tasks=%%d max_worker_count=%%d cycles=%%lu active_word_count=%%lu serial_tasks=%%lu pure_tasks=%%lu pure_batch_count=%%lu true_parallel_batch_count=%%lu skipped_fake_parallel_batch_count=%%lu serial_fast_task_count=%%lu batch_wall_ns=%%lu true_parallel_wall_ns=%%lu serial_wall_ns=%%lu merge_wall_ns=%%lu total_step_ns=%%lu\\n\", mtProfileHelperMode, mtProfileConfiguredWorkerCount, mtWorkerPoolEnabled ? 1 : 0, mtMinBatchTasks, mtProfileMaxWorkerCount, cycles, mtProfileActiveWordCount, mtProfileSerialTasks, mtProfilePureTasks, mtProfilePureBatchCount, mtProfileTrueParallelBatchCount, mtProfileSkippedFakeParallelBatchCount, mtProfileSerialFastTaskCount, mtProfileBatchWallNs, mtProfileTrueParallelWallNs, mtProfileSerialWallNs, mtProfileMergeWallNs, mtProfileTotalStepNs);\n");
  } else {
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] helper_mode=%%s worker_count=%%d min_batch_tasks=%%d max_worker_count=%%d cycles=%%lu active_word_count=%%lu serial_tasks=%%lu pure_tasks=%%lu pure_batch_count=%%lu true_parallel_batch_count=%%lu skipped_fake_parallel_batch_count=%%lu serial_fast_task_count=%%lu batch_wall_ns=%%lu true_parallel_wall_ns=%%lu serial_wall_ns=%%lu merge_wall_ns=%%lu total_step_ns=%%lu\\n\", mtProfileHelperMode, mtProfileConfiguredWorkerCount, mtMinBatchTasks, mtProfileMaxWorkerCount, cycles, mtProfileActiveWordCount, mtProfileSerialTasks, mtProfilePureTasks, mtProfilePureBatchCount, mtProfileTrueParallelBatchCount, mtProfileSkippedFakeParallelBatchCount, mtProfileSerialFastTaskCount, mtProfileBatchWallNs, mtProfileTrueParallelWallNs, mtProfileSerialWallNs, mtProfileMergeWallNs, mtProfileTotalStepNs);\n");
  }
  if (useCoarseMt) {
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_dispatch coarse_runtime=%%s coarse_profitability=%%s coarse_worker_policy=%%s static_runtime_eligible_regions=%%lu static_layer_count=%%lu max_region_layer_count=%%lu static_mtask_count=%%lu region_invocations=%%lu accepted_regions=%%lu rejected_regions=%%lu layer_dispatches=%%lu mtask_dispatches=%%lu worker_jobs=%%lu flag_word_copies=%%lu merge_word_scans=%%lu activation_delta_entries=%%lu estimated_barriers=%%lu estimated_useful_work=%%lu estimated_rejected_useful_work=%%lu estimated_overhead_words=%%lu active_mtasks=%%lu active_mtask_static_cost=%%lu assigned_static_cost=%%lu\\n\", (mtCoarseUseMTaskRuntime ? \"mtask\" : \"layered\"), \"%s\", \"%s\", mtProfileCoarseStaticRuntimeEligibleRegions, mtProfileCoarseStaticLayerCount, mtProfileCoarseStaticMaxRegionLayerCount, mtProfileCoarseStaticMTaskCount, mtProfileCoarseRegionInvocations, mtProfileCoarseAcceptedRegions, mtProfileCoarseRejectedRegions, mtProfileCoarseLayerDispatches, mtProfileCoarseMTaskDispatches, mtProfileCoarseWorkerJobs, mtProfileCoarseFlagWordCopies, mtProfileCoarseMergeWordScans, mtProfileCoarseActivationDeltaEntries, mtProfileCoarseEstimatedBarrierCount, mtProfileCoarseEstimatedUsefulWork, mtProfileCoarseEstimatedRejectedUsefulWork, mtProfileCoarseEstimatedOverheadWords, mtProfileCoarseActiveMTaskCount, mtProfileCoarseActiveMTaskStaticCost, mtProfileCoarseAssignedStaticCost);\n", globalConfig.MtCoarseProfitabilityMode.c_str(), globalConfig.MtCoarseWorkerPolicyMode.c_str());
    emitBodyLock(1, "fprintf(stderr, \"[mt-profile] coarse_antichain_dispatches=%%lu\\n\", mtProfileCoarseAntichainDispatches);\n");
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
  if (useCoarseMt) {
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
  for (int i = 0; i < resetFuncNum; i ++) {
    fprintf(header, "void subReset%d();\n", i);
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
          MtCoarseRegionPlan dstaticPlan = planMtCoarseRegions(mtRepCutHeaderTasks);
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

  /* step wrapper */
  fprintf(header, "void step();\n");
  genStep(subStepIdxMax, serialFastSubStepMax, serialFastSuffix);

  /* end of file */
  fprintf(header, "};\n"
                  "#endif\n");
  fclose(header);
  fclose(srcFp);
#ifdef DIFFTEST_PER_SIG
  fclose(sigFile);
#endif

  printf("[cppEmitter] define %ld nodes %d superNodes\n", definedNode.size(), superId);
  std::cout << "[cppEmitter] finish writing " << srcFileIdx << " cpp files to " + globalConfig.OutputDir + "/" << std::endl;
}
