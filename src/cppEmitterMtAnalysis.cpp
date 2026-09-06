// cppEmitterMtAnalysis.cpp - split from cppEmitter.cpp (pure code motion):
// MtBoundaryInfo collection (visitMtENode), MtTaskInfo classification + serial reasons,
// worker0-only/fallback reason predicates, pure-batch/coarse admission helpers,
// state-update trace classification, cross-task edge memos (dependency/active caches).
#include "cppEmitterImpl.h"

static void addCppIdIfExecutable(std::set<int>& ids, SuperNode* super) {
  if (super && super->cppId >= 0) ids.insert(super->cppId);
}

void addCppIdsIfExecutable(std::set<int>& ids, const std::set<SuperNode*>& supers) {
  for (SuperNode* super : supers) addCppIdIfExecutable(ids, super);
}

bool nodeHasStateUpdate(Node* node) {
  return node->type == NODE_REG_DST || node->type == NODE_REG_RESET ||
         (node->type == NODE_REG_SRC && node->regNext && node->regNext->status == VALID_NODE);
}

bool stateTargetNameForNode(Node* node, std::string& targetName) {
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

MtBoundaryInfo collectMtBoundaryInfo(SuperNode* super, int& candidateCost) {
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

std::map<int, MtTaskInfo> buildMtTaskInfoMap() {
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

// any of these reasons forces the cppId to run on worker 0 (single-threaded
// fall-through). external/memory_write/memory_read_unsupported/special are kimi-2.7-code
// review I5 conservative. super_type_SUPER_EXTMOD covers the alwaysActive set.
bool hasWorker0OnlyReason(const std::vector<std::string>& reasons) {
  for (const std::string& r : reasons) {
    if (r == "external" || r == "memory_write" ||
        r == "memory_read_unsupported" || r == "special" ||
        r == "super_type_SUPER_EXTMOD") {
      return true;
    }
  }
  return false;
}

// dense worker0 pinning predicate; identical to hasWorker0OnlyReason
// except "special" unpinned when GSIM_MT_DENSE_UNPIN_SPECIAL=1.
bool hasWorker0OnlyReasonDense(const std::vector<std::string>& reasons) {
  const bool unpinSpecial = mtUseDenseUnpinSpecial();
  for (const std::string& r : reasons) {
    if (unpinSpecial && r == "special") continue;
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
bool hasOnlyA44DirectFallbackReasons(const std::vector<std::string>& reasons) {
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

bool hasOnlyA73Worker0SafeReasons(const std::vector<std::string>& reasons) {
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


bool mtTaskCanEnterCoarseDispatch(const std::map<int, MtTaskInfo>& tasks, int cppId) {
  auto iter = tasks.find(cppId);
  if (iter == tasks.end()) return false;
  if (isAlwaysActive(cppId)) return false;
  if (iter->second.taskKind == "pure_compute") return true;
  return hasOnlyA44DirectFallbackReasons(iter->second.serialReasons);
}

bool mtTaskCanEnterPureBatch(const std::map<int, MtTaskInfo>& tasks, int cppId) {
  auto iter = tasks.find(cppId);
  if (iter == tasks.end()) return false;
  if (iter->second.taskKind != "pure_compute") return false;
  if (isAlwaysActive(cppId)) return false;
  return true;
}


bool mtTaskHasSameActiveWordHazard(const std::map<int, MtTaskInfo>& tasks, int cppId) {
  if (!mtTaskCanEnterPureBatch(tasks, cppId)) return false;
  int wordBegin = (cppId / ACTIVE_WIDTH) * ACTIVE_WIDTH;
  int wordEnd = std::min(superId, wordBegin + ACTIVE_WIDTH);
  for (int otherCppId = wordBegin; otherCppId < wordEnd; otherCppId ++) {
    if (otherCppId == cppId) continue;
    if (!mtTaskCanEnterPureBatch(tasks, otherCppId)) continue;
    if (mtTasksHaveEdge(cppId2Super[cppId], cppId2Super[otherCppId])) return true;
  }
  return false;
}

bool mtTaskCanJoinPureBatch(const std::map<int, MtTaskInfo>& tasks,
                                   const std::vector<int>& batch,
                                   int cppId) {
  if (!mtTaskCanEnterPureBatch(tasks, cppId)) return false;
  for (int existingCppId : batch) {
    if (mtTasksHaveEdge(cppId2Super[existingCppId], cppId2Super[cppId])) return false;
  }
  return true;
}


int mtTaskEstimatedCost(const std::map<int, MtTaskInfo>& tasks, int cppId) {
  auto iter = tasks.find(cppId);
  if (iter == tasks.end()) return 1;
  if (iter->second.hasCandidateCost && iter->second.candidateCost > 0) return iter->second.candidateCost;
  return 1;
}

int mtBatchEstimatedCost(const std::map<int, MtTaskInfo>& tasks, int beginCppId, int endCppId) {
  int cost = 0;
  for (int cppId = beginCppId; cppId < endCppId; cppId ++) {
    cost += mtTaskEstimatedCost(tasks, cppId);
  }
  return cost;
}

int mtBatchMemberNodeCost(int beginCppId, int endCppId) {
  int cost = 0;
  for (int cppId = beginCppId; cppId < endCppId; cppId ++) {
    auto iter = cppId2Super.find(cppId);
    if (iter != cppId2Super.end() && iter->second) cost += static_cast<int>(iter->second->member.size());
  }
  return cost;
}

bool mtActiveWordIsWhole(int beginCppId) {
  for (int j = 0; j < ACTIVE_WIDTH && beginCppId + j < superId; j ++) {
    if (isAlwaysActive(beginCppId + j)) return false;
  }
  return true;
}

int mtPureBatchShardCount() {
  return (superId + MT_PURE_BATCH_SHARD_SIZE - 1) / MT_PURE_BATCH_SHARD_SIZE;
}

void mtAddCoarseBlocker(MtCoarseRegion& region, const std::string& blocker) {
  if (std::find(region.blockers.begin(), region.blockers.end(), blocker) == region.blockers.end()) {
    region.blockers.push_back(blocker);
  }
}


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


bool mtTaskHasActiveEdgeTo(int fromCppId, int toCppId) {
  if (!globalConfig.MtContextCache) return mtTaskHasActiveEdgeToUncached(fromCppId, toCppId);
  uint64_t key = mtEdgeCacheKey(fromCppId, toCppId);
  auto iter = mtActiveEdgeCache.find(key);
  if (iter != mtActiveEdgeCache.end()) return iter->second;
  bool value = mtTaskHasActiveEdgeToUncached(fromCppId, toCppId);
  mtActiveEdgeCache.emplace(key, value);
  return value;
}

bool mtTaskHasDependencyEdgeTo(int fromCppId, int toCppId) {
  if (!globalConfig.MtContextCache) return mtTaskHasDependencyEdgeToUncached(fromCppId, toCppId);
  uint64_t key = mtEdgeCacheKey(fromCppId, toCppId);
  auto iter = mtDependencyEdgeCache.find(key);
  if (iter != mtDependencyEdgeCache.end()) return iter->second;
  bool value = mtTaskHasDependencyEdgeToUncached(fromCppId, toCppId);
  mtDependencyEdgeCache.emplace(key, value);
  return value;
}

bool mtTaskHasOrderingEdgeTo(int fromCppId, int toCppId) {
  return mtTaskHasDependencyEdgeTo(fromCppId, toCppId) || mtTaskHasActiveEdgeTo(fromCppId, toCppId);
}

bool mtStateUpdateHasMemoryOrDynamicArray(const MtBoundaryInfo& boundary) {
  return boundary.hasMemoryWrite || boundary.hasMemoryRead || boundary.hasArrayOrDynamicIndex;
}

bool mtStateUpdateHasExternalOrSpecial(const MtBoundaryInfo& boundary) {
  return boundary.hasExternal || boundary.hasSpecial || boundary.hasUnknownNode || boundary.hasUnknownOp;
}

bool mtStateUpdateHasSameCycleTargetRead(const MtBoundaryInfo& boundary,
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

std::string mtStateUpdateRhsTimingClass(const MtBoundaryInfo& boundary) {
  if (!mtStateUpdateCanClassifyRhsTiming(boundary)) return "unknown";
  if (boundary.stateSourceCommitCount == 1 && boundary.stateNextUpdateCount == 0 && boundary.stateResetUpdateCount == 0) {
    return "precomputed";
  }
  if (boundary.stateSourceCommitCount == 0 && boundary.stateNextUpdateCount == 1 && boundary.stateResetUpdateCount == 0) {
    return "old_state_only";
  }
  return "unknown";
}

std::string mtStateUpdateRhsTimingEvidence(const MtBoundaryInfo& boundary,
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

bool mtStateUpdateActivationCanUseDelta(const MtBoundaryInfo& boundary) {
  if (!boundary.hasStateUpdate) return false;
  if (boundary.hasReset || boundary.hasAsyncReset || boundary.hasActivateAllPath) return false;
  if (mtStateUpdateHasMemoryOrDynamicArray(boundary)) return false;
  if (mtStateUpdateHasExternalOrSpecial(boundary)) return false;
  return true;
}

std::vector<std::string> mtStateUpdateBlockReasons(const MtBoundaryInfo& boundary,
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

std::string mtStateUpdateCandidateKind(const MtBoundaryInfo& boundary,
                                              const std::vector<std::string>& blockReasons) {
  if (!boundary.hasStateUpdate) return "blocked";
  if (blockReasons.empty() && boundary.stateTargetNames.size() == 1) return "safe_candidate";
  if (blockReasons.size() == 1 && blockReasons[0] == "multiple_state_targets") return "needs_split";
  return "blocked";
}


static const size_t MT_STATE_TARGET_WRITER_ID_LIMIT = 16;

MtStateTargetWriterUniverse
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

MtStateTargetWriterInfo mtStateUpdateWriterInfo(
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

std::string mtStateUpdateTargetWriterConflictKind(
    const MtBoundaryInfo& boundary,
    const MtStateTargetWriterInfo& writerInfo,
    bool hasIncompleteWriterUniverse) {
  if (!boundary.hasStateUpdate || boundary.stateTargetNames.empty()) return "none";
  if (boundary.stateTargetNames.size() != 1) return "multi_target_unproven";
  if (writerInfo.writerCount <= 1 && hasIncompleteWriterUniverse) return "writer_universe_incomplete";
  if (writerInfo.writerCount <= 1) return "unique_writer";
  return "multi_writer_unproven";
}

std::string mtStateUpdateTargetWriterProof(const std::string& conflictKind) {
  if (conflictKind == "unique_writer") return "target_unique_writer";
  if (conflictKind == "multi_writer_unproven") return "none";
  return "none";
}

std::vector<std::string> mtStateUpdateRuntimeBlockReasons(
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

std::set<std::string> collectAllMtStateTargetNames(const std::map<int, MtTaskInfo>& mtTasks) {
  std::set<std::string> names;
  for (const auto& iter : mtTasks) {
    const MtBoundaryInfo& boundary = iter.second.boundary;
    names.insert(boundary.stateTargetNames.begin(), boundary.stateTargetNames.end());
  }
  return names;
}

std::vector<MtStateUpdateTraceInfo> buildMtStateUpdateTraceInfo(const std::map<int, MtTaskInfo>& mtTasks) {
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
