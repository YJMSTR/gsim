#include "mtNodeSink.h"

#include "common.h"
#include "mtCostModel.h"
#include "mtTaskSchedule.h"

#include <algorithm>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <queue>
#include <map>
#include <numeric>
#include <set>
#include <unordered_map>
#include <unistd.h>
#include <utility>

// Node-level sink planner. Runs after the shared MT partition/lowering inside
// MtWorkerBuilder::build when GSIM_MT_NODE_SINK=1: it discards the MTask task
// formation and rebuilds the compute phase as one microtask per graph node.
// Side-effect nodes (printf/assert, memory writers/readwriters, register
// resets, ext modules and every node that writes state or pending storage)
// are unique microtasks; plain NODE_OTHERS/NODE_INFER nodes without state
// writes and with at most 32 array elements are pure and may be replicated.
// The shared state-update, ready-token and local-wait machinery downstream of
// the planner is reused unchanged.
//
// After placement and replication, each worker's chain is linearly coarsened
// (p2sim style): contiguous pure-computation intervals are merged by an
// interval DP that minimizes merged lowered cost plus cross-block activation
// edge cost, with at most nodeSinkBlockMaxInsts() statements per block.
// Side-effect/ext/memory/reset microtasks stay isolated singletons so reset,
// pending-write and side-effect ordering is preserved.
//
// GSIM_MT_NODE_SINK_BLOCK_V2=1 (default off) widens the DP's mergeable set
// with unique plain-assignment microtasks: REG_DST next-value writes and
// pinned pure-combinational nodes. Merged blocks stay single-instance (never
// cloned), so the unique state-commit owner, memory pending order, reset and
// printf/assert ordering are untouched; only the per-microtask token/task
// boundary between same-worker neighbours disappears.

namespace {

bool nodeSinkEnabled() {
  const char* value = std::getenv("GSIM_MT_NODE_SINK");
  return value != nullptr && value[0] == '1';
}

bool nodeSinkRebalanceEnabled() {
  const char* value = std::getenv("GSIM_MT_NODE_SINK_REBALANCE");
  return value != nullptr && value[0] == '1';
}

// Coarsening is on by default; GSIM_MT_NODE_SINK_COARSEN=0 disables it (used
// to verify the un-coarsened clone mechanics in isolation).
bool nodeSinkBlockRebalanceEnabled() {
  const char* value = std::getenv("GSIM_MT_NODE_SINK_BLOCK_REBALANCE");
  return nodeSinkRebalanceEnabled() && value != nullptr && value[0] == '1';
}

bool coarsenEnabled() {
  const char* value = std::getenv("GSIM_MT_NODE_SINK_COARSEN");
  return value == nullptr || value[0] != '0';
}

// Block formation v2, default off. Extends the coarsening DP to unique
// plain-assignment microtasks (see the file header). The default path is
// byte-identical to v1.
bool blockV2Enabled() {
  const char* value = std::getenv("GSIM_MT_NODE_SINK_BLOCK_V2");
  return value != nullptr && value[0] == '1';
}
double cloneBudgetCap() {
  const char* value = std::getenv("GSIM_MT_NODE_SINK_CLONE_CAP");
  if (value == nullptr || value[0] == '\0') return 1.25;
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  return end != value && *end == '\0' && parsed >= 1.0 && parsed <= 16.0
             ? parsed : 1.25;
}

// Pure-computation block statement cap. Default 140; overridden for
// generate-only sweeps with GSIM_MT_NODE_SINK_BLOCK_INSTS.
int nodeSinkBlockMaxInsts() {
  static const int cached = [] {
    const char* value = std::getenv("GSIM_MT_NODE_SINK_BLOCK_INSTS");
    if (value == nullptr || value[0] == '\0') return 140;
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    return end != value && *end == '\0' && parsed >= 1 && parsed <= 1000000
               ? static_cast<int>(parsed)
               : 140;
  }();
  return cached;
}
constexpr long long kNodeSinkEdgeCost =
    mtcost::tokenLatency + mtcost::remotePublish + mtcost::remoteCheck;

size_t nodeArrayElements(const Node* node) {
  size_t elements = 1;
  for (int dimension : node->dimension) {
    elements *= static_cast<size_t>(upperPower2(dimension));
  }
  return elements;
}
bool nodeCloneEnabled() {
  const char* value = std::getenv("GSIM_MT_NODE_SINK_CLONE");
  return value != nullptr && value[0] != '\0' && std::strtol(value, nullptr, 10) != 0;
}

bool nodePureGroupingEnabled() {
  const char* value = std::getenv("GSIM_MT_NODE_SINK_GROUP_PURE");
  return value != nullptr && value[0] != '\0' && std::strtol(value, nullptr, 10) != 0;
}

size_t nodeCloneArrayCap() {
  const char* value = std::getenv("GSIM_MT_NODE_SINK_ARRAY_CAP");
  if (value == nullptr || value[0] == '\0') return 32;
  char* end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  return end != value && *end == '\0' && parsed >= 1 && parsed <= 1000000
             ? static_cast<size_t>(parsed) : 32;
}

// Replace full C++ identifiers, including '$', without touching strings or
// comments. Same rules as the HEFT replication redirect in
// mtReplicationSchedule.cpp; duplicated here because that helper is file
// local and this translation unit must stay independent of it.
std::string redirectText(const std::string& text,
                         const std::map<std::string, std::string>& names) {
  std::string result;
  result.reserve(text.size());
  for (size_t i = 0; i < text.size();) {
    const size_t begin = i;
    if (text[i] == '"' || text[i] == '\'') {
      const char quote = text[i++];
      while (i < text.size()) {
        if (text[i] == '\\') {
          i = std::min(i + 2, text.size());
          continue;
        }
        if (text[i++] == quote) break;
      }
    } else if (text.compare(i, 2, "//") == 0) {
      i = text.find('\n', i);
      if (i == std::string::npos) i = text.size();
    } else if (text.compare(i, 2, "/*") == 0) {
      i = text.find("*/", i + 2);
      i = i == std::string::npos ? text.size() : i + 2;
    } else if (mtcost::identifier(text[i])) {
      while (i < text.size() && mtcost::identifier(text[i])) ++i;
      auto found = names.find(text.substr(begin, i - begin));
      if (found != names.end()) {
        result += found->second;
        continue;
      }
    } else {
      ++i;
    }
    result.append(text, begin, i - begin);
  }
  return result;
}

void redirectInstructions(std::vector<InstInfo>& insts,
                          const std::map<std::string, std::string>& names) {
  for (InstInfo& inst : insts) {
    if (inst.infoType == SUPER_INFO_ASSIGN_BEG ||
        inst.infoType == SUPER_INFO_ASSIGN_END) {
      continue;
    }
    inst.inst = redirectText(inst.inst, names);
  }
}

void replaceReferences(ExpTree* tree,
                       const std::unordered_map<Node*, Node*>& replacements) {
  std::vector<ENode*> pending{tree->getRoot(), tree->getlval()};
  while (!pending.empty()) {
    ENode* expression = pending.back();
    pending.pop_back();
    if (expression == nullptr) continue;
    auto replacement = replacements.find(expression->nodePtr);
    if (replacement != replacements.end()) expression->nodePtr = replacement->second;
    expression->computeInfo = nullptr;
    for (ENode* child : expression->child) pending.push_back(child);
  }
}

// Per-node lowering. Mirrors mergeNodeAssignments/buildTaskTree from
// mtTaskLowering.cpp for a single-node scope: sparse register pending writes,
// plain assignment trees and the async-reset destination copy. Paths are empty
// because a one-node task has no in-task statement dependencies; every
// cross-node value is a class member published before the consumer runs.
void lowerNodeTask(MtTaskPlan& plan, Node* node, MtTask& task) {
  Assert(!mtIsCycleStartRegisterUpdate(node),
         "cycle-start register %s leaked into node-sink compute lowering",
         node->name.c_str());
  task.stmtTree = new StmtTree();
  task.stmtTree->root = new StmtNode(OP_STMT_SEQ);
  std::vector<int> prevPath;
  std::vector<int> nodePath;
  auto sparse = plan.sparseRegisterByDestination_.find(node);
  if (sparse != plan.sparseRegisterByDestination_.end()) {
    for (ExpTree* write : plan.sparseRegisters_[sparse->second].writeTrees) {
      task.stmtTree->mergeExpTree(write, prevPath, nodePath, node);
    }
  } else {
    for (ExpTree* assignment : node->assignTree) {
      task.stmtTree->mergeExpTree(assignment, prevPath, nodePath, node);
      if (node->type != NODE_REG_SRC || node->reset != ASYRESET ||
          !node->regSplit || node->getDst()->status != VALID_NODE) {
        continue;
      }
      ENode* root = assignment->getRoot();
      if (root->nodePtr == node->getDst() && root->getChildNum() == 0) continue;
      ENode* lvalue = assignment->getlval()->dup();
      lvalue->nodePtr = node->getDst();
      task.stmtTree->mergeExpTree(new ExpTree(root, lvalue), prevPath, nodePath,
                                  nullptr);
    }
  }
  task.insts = new std::vector<InstInfo>();
  task.stmtTree->compute(*task.insts);
}

int statementCount(const MtTask& task) {
  int statements = 0;
  for (const InstInfo& inst : *task.insts) {
    if (inst.infoType == SUPER_INFO_IF || inst.infoType == SUPER_INFO_ELSE ||
        inst.infoType == SUPER_INFO_STR) {
      ++statements;
    }
  }
  return statements;
}

struct PlanMetrics {
  size_t tasks = 0;
  size_t crossEdges = 0;
  size_t tokenGroups = 0;
  long long makespan = 0;
};

class NodeSinkPlanner {
 public:
  NodeSinkPlanner(graph& g, MtTaskPlan& plan, int workers)
      : graph_(g), plan_(plan), workers_(workers) {}

  void run(MtNodeSinkSchedule& schedule) {
    collectResetDependencies();
    formNodeTasks();
    buildDependencies();
    applyActivityProfile();
    if (workers_ > 1) {
      if (!assignBySinkHypergraphPartition()) {
        assignSinkAwareLpt();
        if (nodeCloneEnabled()) replicatePureNodes();
      }
      finalizeOrder();
    } else {
      singleWorkerPlan();
    }
    reportUnique_ = 0;
    reportPure_ = 0;
    for (size_t id = 0; id < isClone_.size(); ++id) {
      if (isClone_[id] != 0) continue;
      if (pure_[id] != 0) ++reportPure_;
      else ++reportUnique_;
    }
    const PlanMetrics before = measure();
    reportV2_ = blockV2Enabled();
    if (coarsenEnabled() && !tasks_.empty()) {
      coarsenChains();
      finalizeOrder();
      if (nodeSinkBlockRebalanceEnabled()) rebalanceBlockClosures();
    }
    before_ = before;
    rebuildPlanMetadata();
    remapWorkersForTopology();
    computeSchedule(schedule);
    writeReport();
  }

 private:
  graph& graph_;
  MtTaskPlan& plan_;
  int workers_;

  std::vector<MtTask> tasks_;
  std::vector<Node*> taskNode_;  // null for multi-member microtasks/blocks
  std::vector<uint8_t> isClone_;
  std::vector<uint8_t> pure_;
  std::vector<std::vector<int>> preds_;
  std::vector<std::vector<int>> succs_;
  std::vector<int> owner_;
  std::vector<int> cost_;
  std::vector<long long> load_;
  std::unordered_map<Node*, int> taskOf_;
  std::set<Node*> resetDependencies_;
  std::set<Node*> keepDespiteEmpty_;  // state destinations the plan must own
  std::vector<std::unordered_map<Node*, Node*>> cloneOn_;  // worker -> src -> copy
  struct Clone {
    Node* original = nullptr;
    Node* copy = nullptr;
    int worker = -1;
    int redirected = 0;
  };
  std::vector<Clone> clones_;
  int cloneCounter_ = 0;

  // Report numbers.
  PlanMetrics before_;
  PlanMetrics after_;
  size_t reportRemovedEdges_ = 0;
  size_t reportUnique_ = 0;
  size_t reportPure_ = 0;
  long long reportFairShare_ = 0;
  long long reportBudget_ = 0;
    bool reportRebalance_ = false;
    int reportRebalanceMoves_ = 0;
    double reportRebalancePreImbalance_ = 0;
    double reportRebalancePostImbalance_ = 0;
    size_t reportRebalancePreTokens_ = 0;
    size_t reportRebalancePostTokens_ = 0;
    long long reportRebalancePreMakespan_ = 0;
    bool reportBlockRebalance_ = false;
    int reportBlockRebalanceMoves_ = 0;
    double reportBlockRebalancePreImbalance_ = 0;
    double reportBlockRebalancePostImbalance_ = 0;
    size_t reportBlockRebalancePreTokens_ = 0;
    size_t reportBlockRebalancePostTokens_ = 0;
    long long reportBlockRebalancePreMakespan_ = 0;
    long long reportBlockRebalancePostMakespan_ = 0;
    long long reportRebalancePostMakespan_ = 0;
  bool reportV2_ = false;
  size_t reportV2MergedUnique_ = 0;
  std::map<std::string, size_t> reportV2Pinned_;
  bool coarsened_ = false;
  std::string reportPartitioner_ = "lpt";

  void collectResetDependencies() {
    for (SuperNode* reset : graph_.allReset) {
      for (Node* member : reset->member) {
        for (ExpTree* assignment : member->assignTree) {
          assignment->getRelyNodes(resetDependencies_);
        }
      }
    }
  }

  bool isPureCloneable(Node* node) const {
    if (node->status != VALID_NODE) return false;
    if (node->type != NODE_OTHERS && node->type != NODE_INFER) return false;
    if (node->isReset() || node->isClock) return false;
    if (resetDependencies_.count(node) != 0) return false;
    // Sparse register destinations lower to pending-write publications.
    if (plan_.sparseRegisterByDestination_.count(node) != 0) return false;
    if (node->assignTree.empty()) return false;
    if (!node->dimension.empty() && nodeArrayElements(node) > nodeCloneArrayCap()) return false;
    // Only nodes whose assignments write exactly their own storage are pure.
    for (ExpTree* tree : node->assignTree) {
      ENode* lval = tree->getlval();
      if (lval == nullptr || lval->nodePtr != node) return false;
    }
    return true;
  }

  // Block-v2 admission for a unique microtask's members: plain storage
  // writes only. NODE_REG_DST carries the register next-value assignment
  // (the state task commits it later, so the write itself is a plain
  // persistent-storage store); NODE_OTHERS/NODE_INFER covers pure
  // combinational nodes that are pinned only by the clone rules (foreign
  // lval, large array, reset-cone input). Every side-effect class stays a
  // singleton: memory ports, register resets, printf/assert, ext nodes,
  // sparse pending publications and REG_SRC storage.
  bool v2MergeableNode(Node* node) const {
    if (node->status != VALID_NODE) return false;
    if (node->isReset() || node->isClock) return false;
    // Sparse register destinations lower to pending-write publications.
    if (plan_.sparseRegisterByDestination_.count(node) != 0) return false;
    if (node->type == NODE_REG_DST) return true;
    return node->type == NODE_OTHERS || node->type == NODE_INFER;
  }

  // A unique (non-pure, non-clone) microtask may join a v2 block when every
  // member passes v2MergeableNode.
  bool v2UniqueMergeable(int taskId) const {
    for (Node* member : tasks_[taskId].members) {
      if (!v2MergeableNode(member)) return false;
    }
    return true;
  }

  // Why a unique microtask stayed a singleton under v2 (report only).
  const char* v2PinClass(const MtTask& task) const {
    if (task.kind == MtTaskKind::ExtModule) return "ext";
    for (Node* member : task.members) {
      if (member->isExt()) return "ext";
      if (member->type == NODE_SPECIAL) return "special";
      if (member->type == NODE_READER || member->type == NODE_WRITER ||
          member->type == NODE_READWRITER) {
        return "memory";
      }
      if (member->type == NODE_REG_RESET || member->isReset()) return "reset";
      if (plan_.sparseRegisterByDestination_.count(member) != 0) return "sparse";
      if (member->type == NODE_REG_SRC) return "reg_src";
    }
    return "other";
  }

  void formNodeTasks() {
    tasks_.clear();
    taskNode_.clear();
    isClone_.clear();
    pure_.clear();
    cost_.clear();
    taskOf_.clear();
    keepDespiteEmpty_.clear();
    for (Node* reg : plan_.stateRegisters_) {
      Node* dst = reg->getDst();
      if (dst != nullptr && dst->status == VALID_NODE) keepDespiteEmpty_.insert(dst);
    }
    std::unordered_map<Node*, const MtTask*> extTaskByMember;
    for (const MtTask& old : plan_.tasks_) {
      if (old.kind != MtTaskKind::ExtModule) continue;
      for (Node* node : old.members) extTaskByMember[node] = &old;
    }

    auto registerTask = [&](MtTask task, std::vector<Node*> members, bool pure) {
      if (members.empty()) return;
      const int id = static_cast<int>(tasks_.size());
      for (Node* node : members) {
        Assert(taskOf_.count(node) == 0, "node %s assigned twice", node->name.c_str());
        taskOf_[node] = id;
      }
      taskNode_.push_back(members.size() == 1 ? members.front() : nullptr);
      isClone_.push_back(0);
      pure_.push_back(pure ? 1 : 0);
      tasks_.push_back(std::move(task));
    };

    // Super-order formation: consecutive pure combinational nodes share one
    // task (natural SuperNode blocks); every side-effect/state/memory/reset/
    // ext node and every pinned (large array, foreign lval, sparse) node
    // flushes the group and becomes its own unique microtask.
    std::vector<Node*> group;
    auto flushGroup = [&]() {
      if (group.empty()) return;
      MtTask task;
      task.kind = MtTaskKind::Normal;
      task.insts = new std::vector<InstInfo>();
      std::vector<Node*> members;
      bool pure = true;
      for (Node* node : group) {
        MtTask lowered;
        lowerNodeTask(plan_, node, lowered);
        if (lowered.insts->empty()) continue;  // dead statement, keep graph node out
        task.insts->insert(task.insts->end(), lowered.insts->begin(),
                           lowered.insts->end());
        task.members.push_back(node);
        members.push_back(node);
        pure = pure && isPureCloneable(node);
      }
      group.clear();
      if (members.empty()) return;
      registerTask(std::move(task), members, pure);
    };
    auto isolate = [&](Node* node) {
      flushGroup();
      MtTask task;
      task.kind = MtTaskKind::Normal;
      task.members.push_back(node);
      lowerNodeTask(plan_, node, task);
      if (task.insts->empty() && keepDespiteEmpty_.count(node) == 0) return;
      std::vector<Node*> members{node};
      registerTask(std::move(task), members, isPureCloneable(node));
    };

    for (SuperNode* super : graph_.sortedSuper) {
      if (super->member.empty()) continue;
      if (super->superType == SUPER_EXTMOD) {
        // Ext module lowering (extDecl side effects) already ran in the
        // shared lowering; keep the microtask and its stream atomic.
        const MtTask* old = nullptr;
        for (Node* node : super->member) {
          auto found = extTaskByMember.find(node);
          if (found != extTaskByMember.end()) {
            old = found->second;
            break;
          }
        }
        if (old == nullptr) continue;
        bool already = false;
        for (Node* node : old->members) already |= taskOf_.count(node) != 0;
        if (already) continue;
        flushGroup();
        MtTask task = *old;
        std::vector<Node*> members = task.members;
        registerTask(std::move(task), members, false);
        continue;
      }
      for (Node* node : super->member) {
        if (mtIsCycleStartRegisterUpdate(node)) continue;
        if (taskOf_.count(node) != 0) continue;
        if (nodePureGroupingEnabled() && isPureCloneable(node)) {
          group.push_back(node);
          continue;
        }
        if (node->assignTree.empty() &&
            plan_.sparseRegisterByDestination_.count(node) == 0 &&
            keepDespiteEmpty_.count(node) == 0) {
          continue;  // no code: pass-through storage handled via edge expansion
        }
        isolate(node);
      }
    }
    flushGroup();

    // At this granularity every value with a live consumer outside its task
    // crosses a task boundary, so every emitted node stays a persistent root.
    for (Node* node : plan_.emissionNodes_) node->nodeIsRoot = true;

    for (MtTask& task : tasks_) {
      task.globalNodeCount = nodeGlobalWrites(task);
      task.cost = mtLoweredTaskCost(task);
      cost_.push_back(task.cost);
      if (task.members.size() > 1) {
        std::set<Node*> memberSet(task.members.begin(), task.members.end());
        collectBlockLocalNodes(task, memberSet);
      }
    }
    Assert(cost_.size() == tasks_.size(), "task cost bookkeeping mismatch");
  }

  int nodeGlobalWrites(const MtTask& task) const {
    int writes = 0;
    for (Node* node : task.members) {
      if (node->status != VALID_NODE || node->type == NODE_SPECIAL ||
          node->type == NODE_REG_RESET || node->type == NODE_WRITER) {
        continue;
      }
      if (node->type == NODE_REG_DST && !node->regSplit) continue;
      bool crosses = resetDependencies_.count(node) != 0;
      if (!crosses) {
        for (Node* next : node->next) {
          if (taskOf_.count(next) != 0) {
            crosses = true;
            break;
          }
        }
      }
      if (crosses && !mtIsCycleStartRegisterUpdate(node)) ++writes;
    }
    return writes;
  }

  // Producer tasks of `node`, expanding pass-through nodes (no code, no task)
  // transitively so dropped storage cannot hide edges.
  void collectProducerTasks(Node* node, int self, std::set<int>& result) const {
    std::vector<Node*> pending(node->depPrev.begin(), node->depPrev.end());
    std::set<Node*> visited;
    while (!pending.empty()) {
      Node* producer = pending.back();
      pending.pop_back();
      if (!visited.insert(producer).second) continue;
      auto found = taskOf_.find(producer);
      if (found != taskOf_.end()) {
        if (found->second != self) result.insert(found->second);
        continue;
      }
      pending.insert(pending.end(), producer->depPrev.begin(), producer->depPrev.end());
    }
  }

  // Same-port memory ordering edges the MTask partitioner installs
  // (addActivationEdges): writer/readwriter before the other read ports of
  // the same memory, forward in the global topological order only.
  void collectMemoryOrderTasks(Node* node, int self, std::set<int>& result) const {
    if (node->type != NODE_WRITER && node->type != NODE_READWRITER) return;
    if (node->parent == nullptr || node->parent->type != NODE_MEMORY) return;
    for (Node* port : node->parent->member) {
      if (port->status != VALID_NODE || port == node) continue;
      if (port->type != NODE_READER && port->type != NODE_READWRITER) continue;
      auto found = taskOf_.find(port);
      if (found == taskOf_.end() || found->second == self) continue;
      if (found->second > self) result.insert(found->second);  // forward only
    }
  }

  void buildDependencies() {
    const int count = static_cast<int>(tasks_.size());
    preds_.assign(count, {});
    succs_.assign(count, {});
    for (int id = 0; id < count; ++id) {
      std::set<int> producers;
      if (taskNode_[id] != nullptr) {
        collectProducerTasks(taskNode_[id], id, producers);
        collectMemoryOrderTasks(taskNode_[id], id, producers);
      } else {
        for (Node* member : tasks_[id].members) {
          collectProducerTasks(member, id, producers);
        }
      }
      preds_[id].assign(producers.begin(), producers.end());
      for (int producer : preds_[id]) {
        Assert(producer < id, "non-forward node-sink edge %d -> %d", producer, id);
        succs_[producer].push_back(id);
      }
    }
    for (std::vector<int>& next : succs_) {
      std::sort(next.begin(), next.end());
      next.erase(std::unique(next.begin(), next.end()), next.end());
    }
  }

  // --- p2sim-style repcut: sink hypergraph min-cut via mt-kahypar ----------
  // Vertices are sink microtasks; every task's cost is amortized over its
  // reachable sinks ((cost+|bis|)/|bis| per sink, pass.rs). Each task adds a
  // hyperedge of weight insts/5+1 over its reachable sinks; always (ext)
  // tasks add a 10000 edge; memories add co-location edges for their ports
  // (2*elements when > 32, 10000 when multiple writer ports).

  static std::string helperPath() {
    if (const char* env = std::getenv("GSIM_MT_NODE_SINK_KAHYPAR")) {
      return env;
    }
    const char* candidates[] = {
        "tools/node-sink-partition/target/release/node-sink-partition",
        "build/tools/node-sink-partition/node-sink-partition",
    };
    for (const char* candidate : candidates) {
      if (std::fopen(candidate, "r") != nullptr) return candidate;
    }
    char executable[4096];
    const ssize_t length =
        readlink("/proc/self/exe", executable, sizeof(executable) - 1);
    if (length > 0) {
      executable[length] = '\0';
      std::string directory(executable);
      const size_t slash = directory.rfind('/');
      if (slash != std::string::npos) {
        std::string beside =
            directory.substr(0, slash + 1) + "node-sink-partition";
        if (std::fopen(beside.c_str(), "r") != nullptr) return beside;
      }
    }
    return std::string();
  }

  static std::vector<int> mergeIdSets(const std::vector<std::vector<int>>& sets,
                                      const std::vector<int>& extra = {}) {
    std::vector<int> merged = extra;
    for (const std::vector<int>& set : sets) {
      merged.insert(merged.end(), set.begin(), set.end());
    }
    std::sort(merged.begin(), merged.end());
    merged.erase(std::unique(merged.begin(), merged.end()), merged.end());
    return merged;
  }

  int sinkStateWrites(int taskId) const {
    Node* node = taskNode_[taskId];
    if (node == nullptr) {
      int writes = 0;
      for (Node* member : tasks_[taskId].members) {
        if ((member->type == NODE_WRITER || member->type == NODE_READWRITER ||
             member->type == NODE_REG_DST) &&
            member->parent == nullptr) {
          ++writes;
        }
      }
      return writes;
    }
    if (node->type == NODE_REG_DST || node->type == NODE_WRITER ||
        node->type == NODE_READWRITER) {
      return 1;
    }
    return 0;
  }

  // Returns false when the partition helper is unavailable (LPT fallback).
  bool assignBySinkHypergraphPartition() {
    const int count = static_cast<int>(tasks_.size());
    if (count == 0) return true;
    const std::string helper = helperPath();
    if (helper.empty()) {
      fprintf(stderr, "[cppEmitter-mt] node-sink partitioner=lpt "
                      "(mt-kahypar helper not found)\n");
      return false;
    }

    // Reachable sink sets, reverse topological accumulation.
    std::vector<std::vector<int>> bis(count);
    for (int id = count - 1; id >= 0; --id) {
      if (succs_[id].empty()) {
        bis[id].push_back(id);
        continue;
      }
      std::vector<std::vector<int>> childSets;
      childSets.reserve(succs_[id].size());
      for (int successor : succs_[id]) childSets.push_back(bis[successor]);
      bis[id] = mergeIdSets(childSets);
      Assert(!bis[id].empty(), "task %d reaches no sink", id);
    }
    std::vector<int> sinkOfVertex;  // vertex -> sink task id (ascending task)
    std::vector<int> vertexOfSink(count, -1);
    for (int id = 0; id < count; ++id) {
      if (bis[id].size() == 1 && bis[id].front() == id) {
        vertexOfSink[id] = static_cast<int>(sinkOfVertex.size());
        sinkOfVertex.push_back(id);
      }
    }
    const int vertices = static_cast<int>(sinkOfVertex.size());

    // Vertex weights: per-sink state count plus amortized cone cost.
    std::vector<long long> sinkWeight(vertices, 0);
    for (int vertex = 0; vertex < vertices; ++vertex) {
      sinkWeight[vertex] = 1 + sinkStateWrites(sinkOfVertex[vertex]);
    }
    for (int id = count - 1; id >= 0; --id) {
      const long long shares = static_cast<long long>(bis[id].size());
      const long long amortized =
          (static_cast<long long>(cost_[id]) + shares) / shares;
      for (int sink : bis[id]) {
        sinkWeight[vertexOfSink[sink]] += amortized;
      }
    }

    // Hyperedges: one per task over its reachable sinks, plus always and
    // memory co-location edges.
    struct Edge {
      long long weight = 0;
      std::vector<int> pins;
    };
    std::vector<Edge> edges;
    edges.reserve(count + graph_.memory.size() * 2);
    auto addEdge = [&](long long weight, std::vector<int> pins) {
      if (pins.size() < 2 || weight <= 0) return;
      edges.push_back({weight, std::move(pins)});
    };
    for (int id = 0; id < count; ++id) {
      std::vector<int> pins;
      pins.reserve(bis[id].size());
      for (int sink : bis[id]) pins.push_back(vertexOfSink[sink]);
      const long long weight =
          static_cast<long long>(tasks_[id].insts->size()) / 5 + 1;
      addEdge(weight, pins);
      if (tasks_[id].kind == MtTaskKind::ExtModule) {
        addEdge(10000, pins);
      }
    }
    for (Node* memory : graph_.memory) {
      if (memory->status != VALID_NODE) continue;
      std::vector<std::vector<int>> portSets;
      int writers = 0;
      for (Node* port : memory->member) {
        if (port->status != VALID_NODE) continue;
        if (port->type != NODE_READER && port->type != NODE_WRITER &&
            port->type != NODE_READWRITER) {
          continue;
        }
        if (port->type != NODE_READER) ++writers;
        auto found = taskOf_.find(port);
        if (found == taskOf_.end()) continue;
        portSets.push_back(bis[found->second]);
      }
      if (portSets.empty()) continue;
      std::vector<int> pins = mergeIdSets(portSets);
      std::vector<int> vertexPins;
      vertexPins.reserve(pins.size());
      for (int sink : pins) vertexPins.push_back(vertexOfSink[sink]);
      const long long elements =
          static_cast<long long>(upperPower2(std::max(1, memory->depth)));
      if (elements > 32) addEdge(2 * elements, vertexPins);
      if (writers > 1) addEdge(10000, vertexPins);
    }

    // Serialize the hypergraph and ask the helper for a partition. Edges stay
    // fixed across rounds; only vertex weights change.
    std::string inputPath = globalConfig.OutputDir + "/mt-node-sink.hgr";
    if (globalConfig.OutputDir.empty()) inputPath = "/tmp/mt-node-sink.hgr";
    const std::string epsilon = [] {
      const char* value = std::getenv("GSIM_MT_NODE_SINK_EPS");
      return value != nullptr && value[0] != '\0' ? std::string(value)
                                                  : std::string("0.1");
    }();
    auto runKahypar = [&](const std::vector<long long>& weights) {
      std::vector<int> parts;
      FILE* input = std::fopen(inputPath.c_str(), "w");
      if (input == nullptr) return parts;
      fprintf(input, "%d %zu\n", vertices, edges.size());
      for (int vertex = 0; vertex < vertices; ++vertex) {
        fprintf(input, "%lld%c", weights[vertex],
                vertex + 1 == vertices ? '\n' : ' ');
      }
      for (const Edge& edge : edges) {
        fprintf(input, "%lld %zu", edge.weight, edge.pins.size());
        for (int pin : edge.pins) fprintf(input, " %d", pin);
        fprintf(input, "\n");
      }
      std::fclose(input);
      const std::string command =
          helper + " " + inputPath + " " + std::to_string(workers_) + " " +
          epsilon + " 2>&1";
      FILE* stream = popen(command.c_str(), "r");
      if (stream == nullptr) return parts;
      char line[512];
      while (std::fgets(line, sizeof(line), stream) != nullptr) {
        char* end = nullptr;
        const long value = std::strtol(line, &end, 10);
        if (end == line ||
            (end != nullptr && *end != '\n' && *end != '\0' && *end != '\r')) {
          fprintf(stderr, "[cppEmitter-mt] %s", line);  // helper diagnostics
          continue;
        }
        parts.push_back(static_cast<int>(value));
      }
      const int status = pclose(stream);
      if (status != 0 || parts.size() != static_cast<size_t>(vertices)) {
        fprintf(stderr,
                "[cppEmitter-mt] node-sink helper failed: status=%d "
                "parts=%zu want=%d\n",
                status, parts.size(), vertices);
        parts.clear();
      }
      return parts;
    };

    // Parts per task: union of the parts of its reachable sinks.
    auto computeTaskParts = [&](const std::vector<int>& parts) {
      std::vector<std::vector<int>> result(count);
      for (int id = count - 1; id >= 0; --id) {
        std::vector<std::vector<int>> childSets;
        for (int successor : succs_[id]) childSets.push_back(result[successor]);
        result[id] = mergeIdSets(childSets);
        if (bis[id].size() == 1 && bis[id].front() == id) {
          result[id].push_back(parts[vertexOfSink[id]]);
        }
        std::sort(result[id].begin(), result[id].end());
        result[id].erase(
            std::unique(result[id].begin(), result[id].end()), result[id].end());
        Assert(!result[id].empty(), "task %d has no part", id);
        for (int part : result[id]) {
          Assert(part >= 0 && part < workers_, "helper returned part %d", part);
        }
      }
      return result;
    };

    // Direct-load-aware model: actual hosted loads. Primaries land on their
    // first part; pure tasks replicate into extra parts under the same 1.25x
    // budget the clone loop uses, so the model matches the final placement
    // (equal to the full closure whenever the budget does not bind).
    auto hostedLoads = [&](const std::vector<std::vector<int>>& taskParts) {
      long long total = 0;
      for (int cost : cost_) total += cost;
      const long long fair = (total + workers_ - 1) / workers_;
      const long long budget = fair + fair / 4;
      std::vector<long long> load(workers_, 0);
      for (int id = 0; id < count; ++id) {
        load[taskParts[id].front()] += cost_[id];
      }
      for (int id = 0; id < count; ++id) {
        if (pure_[id] == 0 || taskParts[id].size() < 2) continue;
        for (size_t index = 1; index < taskParts[id].size(); ++index) {
          const int part = taskParts[id][index];
          if (load[part] + cost_[id] <= budget) load[part] += cost_[id];
        }
      }
      return load;
    };

    struct Candidate {
      std::vector<int> sinkPart;
      std::vector<std::vector<int>> taskParts;
      double imbalance = 0;
      size_t tokenGroups = 0;
      long long makespan = 0;
    };
    auto evaluate = [&](const std::vector<int>& parts,
                        const std::vector<std::vector<int>>& taskParts) {
      std::vector<long long> load = hostedLoads(taskParts);
      long long maxLoad = 0;
      long long sumLoad = 0;
      for (long long part : load) {
        maxLoad = std::max(maxLoad, part);
        sumLoad += part;
      }
      const double mean =
          static_cast<double>(sumLoad) / static_cast<double>(workers_);
      const std::vector<int> savedOwner = owner_;
      owner_.assign(count, 0);
      for (int id = 0; id < count; ++id) owner_[id] = taskParts[id].front();
      const PlanMetrics metrics = measure();
      owner_ = savedOwner;
      return Candidate{parts, taskParts, maxLoad / std::max(1.0, mean),
                       metrics.tokenGroups, metrics.makespan};
    };

    // Round 0 cuts with the amortized weights; each later round re-weights
    // every sink vertex with its part's full closure load (spread over the
    // part's vertices so block weights equal hosted loads) and re-cuts.
    constexpr int kRebalanceRounds = 3;
    std::vector<long long> weights = sinkWeight;
    Candidate best;
    bool haveBest = false;
    for (int round = 0; round < kRebalanceRounds; ++round) {
      std::vector<int> parts = runKahypar(weights);
      if (parts.empty()) break;
      std::vector<std::vector<int>> taskParts = computeTaskParts(parts);
      Candidate candidate = evaluate(parts, taskParts);
      fprintf(stderr,
              "[cppEmitter-mt] node-sink round=%d max/mean=%.3f tokens=%zu "
              "makespan=%lld\n",
              round, candidate.imbalance, candidate.tokenGroups,
              candidate.makespan);
      if (!haveBest || candidate.imbalance < best.imbalance - 1e-9 ||
          (candidate.imbalance < best.imbalance + 1e-9 &&
           candidate.tokenGroups < best.tokenGroups)) {
        best = std::move(candidate);
        haveBest = true;
      }
      std::vector<long long> load = hostedLoads(taskParts);
      std::vector<long long> verticesPerPart(workers_, 0);
      for (int vertex = 0; vertex < vertices; ++vertex) {
        ++verticesPerPart[parts[vertex]];
      }
      weights.assign(vertices, 0);
      for (int vertex = 0; vertex < vertices; ++vertex) {
        const int part = parts[vertex];
        weights[vertex] = 1 + sinkStateWrites(sinkOfVertex[vertex]) +
            load[part] / std::max<long long>(1, verticesPerPart[part]);
      }
    }
    if (!haveBest) {
      fprintf(stderr,
              "[cppEmitter-mt] node-sink partitioner=lpt (no kahypar round "
              "succeeded)\n");
      return false;
    }
    reportPartitioner_ = "mt-kahypar";

    // --- deterministic post-cut component migration -------------------------
    // The rounds above balance amortized sink weights, not the hosted loads,
    // so the winning cut can still leave one worker far above fair share.
    // Repair it with whole-component moves: re-homing sink component s from
    // the current max-load part H to the min-load part L swaps H for L (or
    // adds L when sibling sinks keep H) in the closure of every task that
    // reaches s, so taskParts stays exactly the union of reachable sink parts
    // and the unique sink migrates together with its pure closure. Every
    // applied move must strictly lower the exact hosted max/mean and keep the
    // target part within the 1.25x pre-replication budget; stop at 1.2 or at
    // the first move that does not improve. Token growth is accepted and only
    // reported.
    {
      long long totalCost = 0;
      for (int cost : cost_) totalCost += cost;
      const long long fair = (totalCost + workers_ - 1) / workers_;
      const long long budget = fair + fair / 4;
      auto hostedImbalance = [&](const std::vector<long long>& load) {
        long long maxLoad = 0;
        long long sumLoad = 0;
        for (long long part : load) {
          maxLoad = std::max(maxLoad, part);
          sumLoad += part;
        }
        return maxLoad / std::max(1.0, static_cast<double>(sumLoad) / workers_);
      };

      // partCount[id][p]: how many of task id's reachable sinks are homed on
      // p; taskParts[id] is exactly the set of parts with a positive count.
      // componentCost[v]: hosted cost of the whole component of sink vertex v
      // (the unique sink plus every task whose only reachable sink is v's).
      std::vector<int> partCount(static_cast<size_t>(count) * workers_, 0);
      std::vector<std::vector<int>> tasksBySink(count);
      std::vector<long long> componentCost(vertices, 0);
      for (int id = 0; id < count; ++id) {
        if (bis[id].size() == 1) {
          componentCost[vertexOfSink[bis[id].front()]] += cost_[id];
        }
        for (int sink : bis[id]) {
          tasksBySink[sink].push_back(id);
          ++partCount[static_cast<size_t>(id) * workers_ +
                      best.sinkPart[vertexOfSink[sink]]];
        }
      }

      // Re-home one sink component between parts. Re-applying with the parts
      // swapped restores the previous state exactly, so a rejected candidate
      // costs one extra pass and no snapshots.
      auto rehome = [&](int vertex, int from, int to) {
        best.sinkPart[vertex] = to;
        for (int id : tasksBySink[sinkOfVertex[vertex]]) {
          int* counts = &partCount[static_cast<size_t>(id) * workers_];
          --counts[from];
          ++counts[to];
          std::vector<int> parts;
          for (int w = 0; w < workers_; ++w) {
            if (counts[w] > 0) parts.push_back(w);
          }
          best.taskParts[id] = std::move(parts);
        }
      };

      constexpr int kMaxMoves = 1024;
      constexpr int kMaxAttempts = 8;
      std::vector<long long> load = hostedLoads(best.taskParts);
      double imbalance = hostedImbalance(load);
      const double preImbalance = imbalance;
      const size_t preTokens = best.tokenGroups;
      const long long preMakespan = best.makespan;
      int moves = 0;
      struct Move {
        int vertex;
        long long targetLoad;
        long long estimate;
      };
      std::vector<Move> ranked;
      while (imbalance > 1.2 && moves < kMaxMoves) {
        int heavy = 0;
        int light = 0;
        for (int w = 1; w < workers_; ++w) {
          if (load[w] > load[heavy]) heavy = w;
          if (load[w] < load[light]) light = w;
        }
        if (heavy == light) break;
        long long otherMax = 0;
        for (int w = 0; w < workers_; ++w) {
          if (w != heavy && w != light) otherMax = std::max(otherMax, load[w]);
        }
        ranked.clear();
        for (int vertex = 0; vertex < vertices; ++vertex) {
          if (best.sinkPart[vertex] != heavy) continue;
          const long long targetLoad = load[light] + componentCost[vertex];
          if (targetLoad > budget) continue;
          const long long estimate = std::max(
              otherMax,
              std::max(load[heavy] - componentCost[vertex], targetLoad));
          if (estimate >= load[heavy]) continue;  // cannot lower the max
          ranked.push_back({vertex, targetLoad, estimate});
        }
        if (ranked.empty()) break;
        std::sort(ranked.begin(), ranked.end(),
                  [](const Move& lhs, const Move& rhs) {
                    if (lhs.estimate != rhs.estimate) {
                      return lhs.estimate < rhs.estimate;
                    }
                    if (lhs.targetLoad != rhs.targetLoad) {
                      return lhs.targetLoad < rhs.targetLoad;
                    }
                    return lhs.vertex < rhs.vertex;
                  });
        bool improved = false;
        for (size_t attempt = 0; attempt < ranked.size() && attempt < kMaxAttempts;
             ++attempt) {
          const Move& move = ranked[attempt];
          rehome(move.vertex, heavy, light);
          std::vector<long long> nextLoad = hostedLoads(best.taskParts);
          const double nextImbalance = hostedImbalance(nextLoad);
          if (nextImbalance < imbalance - 1e-9 && nextLoad[light] <= budget) {
            load = std::move(nextLoad);
            imbalance = nextImbalance;
            ++moves;
            improved = true;
            break;
          }
          rehome(move.vertex, light, heavy);
        }
        if (!improved) break;
      }
      if (imbalance > 1.2) {
        // Stalled above target: report why. A component bigger than the
        // remaining headroom (budget - min load) can never migrate, so one
        // oversized cone pins the max no matter how many moves are tried.
        int heavy = 0;
        int light = 0;
        for (int w = 1; w < workers_; ++w) {
          if (load[w] > load[heavy]) heavy = w;
          if (load[w] < load[light]) light = w;
        }
        long long biggest = 0;
        long long movable = 0;
        int components = 0;
        for (int vertex = 0; vertex < vertices; ++vertex) {
          if (best.sinkPart[vertex] != heavy) continue;
          ++components;
          biggest = std::max(biggest, componentCost[vertex]);
          if (load[light] + componentCost[vertex] <= budget) {
            movable += componentCost[vertex];
          }
        }
        fprintf(stderr,
                "[cppEmitter-mt] node-sink rebalance-stall max-part=%d "
                "load=%lld min-part=%d load=%lld budget=%lld headroom=%lld "
                "components=%d biggest=%lld movable=%lld\n",
                heavy, load[heavy], light, load[light], budget,
                budget - load[light], components, biggest, movable);
      }
      best = evaluate(best.sinkPart, best.taskParts);
      fprintf(stderr,
              "[cppEmitter-mt] node-sink rebalance moves=%d max/mean=%.3f->%.3f "
              "tokens=%zu->%zu makespan=%lld->%lld budget=%lld\n",
              moves, preImbalance, best.imbalance, preTokens,
              best.tokenGroups, preMakespan, best.makespan, budget);
      reportRebalance_ = true;
      reportRebalanceMoves_ = moves;
      reportRebalancePreImbalance_ = preImbalance;
      reportRebalancePostImbalance_ = best.imbalance;
      reportRebalancePreTokens_ = preTokens;
      reportRebalancePostTokens_ = best.tokenGroups;
      reportRebalancePreMakespan_ = preMakespan;
      reportRebalancePostMakespan_ = best.makespan;
    }
    std::vector<std::vector<int>>& taskParts = best.taskParts;

    owner_.assign(count, -1);
    load_.assign(workers_, 0);
    cloneOn_.assign(workers_, {});
    long long total = 0;
    for (int cost : cost_) total += cost;
    reportFairShare_ = (total + workers_ - 1) / workers_;
    reportBudget_ = static_cast<long long>(reportFairShare_ * cloneBudgetCap());
    for (int id = 0; id < count; ++id) {
      owner_[id] = taskParts[id].front();
      load_[owner_[id]] += cost_[id];
    }

    // Closure replication: copy pure tasks into every extra part that needs
    // them, under the 1.25x budget. Unique microtasks (side effect, state
    // commit, memory, reset, ext) stay single-instance by construction.
    if (nodeCloneEnabled()) for (int id = 0; id < count; ++id) {
      if (pure_[id] == 0 || taskParts[id].size() < 2) continue;
      for (size_t index = 1; index < taskParts[id].size(); ++index) {
        const int worker = taskParts[id][index];
        if (load_[worker] + cost_[id] > reportBudget_) continue;
        std::vector<int> consumers;
        for (int successor : succs_[id]) {
          if (owner_[successor] == worker) consumers.push_back(successor);
        }
        if (consumers.empty()) continue;
        commitClone(id, worker, consumers);
      }
    }
    for (const Clone& clone : clones_) reportRemovedEdges_ += clone.redirected;
    fprintf(stderr,
            "[cppEmitter-mt] node-sink partitioner=mt-kahypar rounds=%d "
            "best-max/mean=%.3f vertices=%d edges=%zu total-cost=%lld k=%d "
            "eps=%s objective=cut seed=42\n",
            kRebalanceRounds, best.imbalance, vertices, edges.size(), total,
            workers_, epsilon.c_str());
    return true;
  }
  void assignSinkAwareLpt() {
    reportPartitioner_ = "lpt";
    const int count = static_cast<int>(tasks_.size());
    owner_.assign(count, -1);
    load_.assign(workers_, 0);

    // reach[id][w]: weight of already-placed work reachable from id on w.
    std::vector<std::vector<long long>> reach(count,
                                              std::vector<long long>(workers_, 0));
    auto place = [&](int id, int worker) {
      owner_[id] = worker;
      load_[worker] += cost_[id];
      auto& own = reach[id];
      for (int w = 0; w < workers_; ++w) {
        long long weight = 0;
        for (int successor : succs_[id]) weight += reach[successor][w];
        own[w] = weight;
      }
      own[worker] += cost_[id];
    };

    // Sinks first (side-effect microtasks and nodes without successors),
    // heaviest by lowered cost, tie: earlier node.
    std::vector<int> sinkOrder;
    for (int id = 0; id < count; ++id) {
      if (pure_[id] == 0 || succs_[id].empty()) sinkOrder.push_back(id);
    }
    std::sort(sinkOrder.begin(), sinkOrder.end(), [&](int lhs, int rhs) {
      if (cost_[lhs] != cost_[rhs]) return cost_[lhs] > cost_[rhs];
      return lhs < rhs;
    });
    for (int id : sinkOrder) {
      int best = 0;
      for (int w = 1; w < workers_; ++w) {
        if (load_[w] < load_[best]) best = w;
      }
      place(id, best);
    }

    // Remaining nodes in reverse topological order: successors are placed, so
    // each node is pulled toward the worker holding its reachable sink weight.
    std::vector<long long> pull(workers_, 0);
    for (int id = count - 1; id >= 0; --id) {
      if (owner_[id] >= 0) continue;
      long long total = 0;
      std::fill(pull.begin(), pull.end(), 0);
      for (int successor : succs_[id]) {
        for (int w = 0; w < workers_; ++w) pull[w] += reach[successor][w];
      }
      for (int w = 0; w < workers_; ++w) total += pull[w];
      int best = 0;
      for (int w = 1; w < workers_; ++w) {
        const long long bestScore = load_[best] + (total - pull[best]);
        const long long score = load_[w] + (total - pull[w]);
        if (score < bestScore || (score == bestScore && load_[w] < load_[best])) {
          best = w;
        }
      }
      place(id, best);
    }
    for (int id = 0; id < count; ++id) {
      Assert(owner_[id] >= 0, "node-sink LPT left task %d unplaced", id);
    }
  }

  void remapWorkersForTopology() {
    const int count = static_cast<int>(tasks_.size());
    if (count == 0 || workers_ != 32) return;
    std::vector<std::vector<long long>> affinity(32, std::vector<long long>(32, 0));
    for (int id = 0; id < count; ++id) {
      for (int successor : tasks_[id].successors) {
        const int lhs = owner_[id];
        const int rhs = owner_[successor];
        if (lhs == rhs) continue;
        const long long weight = 1 + tasks_[successor].cost / 16;
        affinity[lhs][rhs] += weight;
        affinity[rhs][lhs] += weight;
      }
    }
    auto crossCcd = [&](const std::vector<int>& mapping) {
      long long result = 0;
      for (int id = 0; id < count; ++id) {
        for (int successor : tasks_[id].successors) {
          const int lhs = mapping[owner_[id]];
          const int rhs = mapping[owner_[successor]];
          if (lhs / 8 != rhs / 8) result += 1 + tasks_[successor].cost / 16;
        }
      }
      return result;
    };
    std::vector<int> identity(32);
    std::iota(identity.begin(), identity.end(), 0);
    const long long before = crossCcd(identity);

    std::vector<int> order(32);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
      const long long lhsWeight = std::accumulate(affinity[lhs].begin(), affinity[lhs].end(), 0LL);
      const long long rhsWeight = std::accumulate(affinity[rhs].begin(), affinity[rhs].end(), 0LL);
      if (lhsWeight != rhsWeight) return lhsWeight > rhsWeight;
      return lhs < rhs;
    });
    std::vector<std::vector<int>> groups(4);
    for (int worker : order) {
      int bestGroup = -1;
      long long bestScore = LLONG_MIN;
      for (int group = 0; group < 4; ++group) {
        if (groups[group].size() >= 8) continue;
        long long score = 0;
        for (int member : groups[group]) score += affinity[worker][member];
        if (score > bestScore || (score == bestScore && group < bestGroup) ||
            (bestGroup < 0 && score == 0)) {
          bestGroup = group;
          bestScore = score;
        }
      }
      groups[bestGroup].push_back(worker);
    }
    std::vector<int> mapping(32);
    for (int group = 0; group < 4; ++group) {
      for (size_t index = 0; index < groups[group].size(); ++index) {
        mapping[groups[group][index]] = group * 8 + static_cast<int>(index);
      }
    }
    const long long after = crossCcd(mapping);
    if (after >= before) {
      fprintf(stderr, "[cppEmitter-mt] node-sink ccd-map rejected %lld->%lld\n",
              before, after);
      return;
    }
    for (int id = 0; id < count; ++id) owner_[id] = mapping[owner_[id]];
    fprintf(stderr, "[cppEmitter-mt] node-sink ccd-map accepted %lld->%lld\n",
            before, after);
  }

  void applyActivityProfile() {
    const char* statsPath = std::getenv("GSIM_MT_NODE_SINK_ACT_CSV");
    const char* mapPath = std::getenv("GSIM_MT_NODE_SINK_MAP_CSV");
    if (statsPath == nullptr || mapPath == nullptr) return;
    std::unordered_map<int, long long> activity;
    if (FILE* map = std::fopen(mapPath, "r")) {
      char line[512];
      while (std::fgets(line, sizeof(line), map)) {
        if (std::strncmp(line, "task_map,", 9) != 0) continue;
        int task = -1, node = -1;
        if (std::sscanf(line, "task_map,%d,%d", &task, &node) == 2) {
          activity[node] = 0;
        }
      }
      std::fclose(map);
    }
    if (activity.empty()) return;
    if (FILE* stats = std::fopen(statsPath, "r")) {
      char line[512];
      while (std::fgets(line, sizeof(line), stats)) {
        int task = -1;
        long long count = 0;
        if (std::sscanf(line, "%d,%lld", &task, &count) == 2) {
          auto found = activity.find(task);
          if (found != activity.end()) found->second = count;
        }
      }
      std::fclose(stats);
    }
    long long structuralTotal = 0, activityTotal = 0;
    for (long long cost : cost_) structuralTotal += cost;
    for (const auto& entry : activity) activityTotal += entry.second;
    if (structuralTotal == 0 || activityTotal == 0) return;
    for (size_t id = 0; id < cost_.size(); ++id) {
      Node* node = taskNode_[id];
      auto found = activity.find(node == nullptr ? -1 : node->id);
      const long long count = found == activity.end() ? 0 : found->second;
      const long long scaled = static_cast<long long>(
          std::llround(static_cast<long double>(count) * structuralTotal / activityTotal));
      cost_[id] = std::max<long long>(1, scaled);
    }
    fprintf(stderr, "[cppEmitter-mt] node-sink activity-profile tasks=%zu total=%lld\n",
            activity.size(), activityTotal);
  }

  // Post-coarsening whole-closure migration in the real block-cost space.
  // The pre-clone rebalance repairs the hosted microtask model, but the
  // binding constraint for the final plan is each worker's block-chain cost
  // after the coarsening DP. Move a sink block's exclusive closure (the sink
  // plus every block whose only reachable sink block is it, restricted to
  // blocks already on the sink's worker) from the max-load worker to the
  // min-load worker while the exact block-cost max/mean strictly improves
  // and the target stays within the 1.25x block fair share. Blocks are never
  // replicated, so the load delta is exact and the move needs no re-scoring.
  // Only owners change: worker chains remain subsequences of the global
  // topological order, same-worker dependencies stay forward, and every
  // local wait / ready token slot is re-derived downstream from the final
  // owners. Cross-worker edges created at a closure's inputs keep the
  // standard ready-token protocol; their cost is tracked via measure().
  void rebalanceBlockClosures() {
    const int count = static_cast<int>(tasks_.size());
    if (count == 0 || workers_ <= 1) return;

    // Reachable sink blocks per block, reverse topological accumulation.
    std::vector<std::vector<int>> reach(count);
    for (int id = count - 1; id >= 0; --id) {
      if (succs_[id].empty()) {
        reach[id].push_back(id);
        continue;
      }
      std::vector<std::vector<int>> childSets;
      childSets.reserve(succs_[id].size());
      for (int successor : succs_[id]) childSets.push_back(reach[successor]);
      reach[id] = mergeIdSets(childSets);
    }

    // Exclusive closures. A block with exactly one reachable sink only ever
    // migrates with that sink, so the co-owner membership is stable across
    // moves and the closure lists can be built once.
    std::vector<std::vector<int>> closure(count);
    std::vector<long long> closureCost(count, 0);
    std::vector<long long> load(workers_, 0);
    long long total = 0;
    for (int id = 0; id < count; ++id) {
      load[owner_[id]] += cost_[id];
      total += cost_[id];
    }
    for (int id = 0; id < count; ++id) {
      if (reach[id].size() != 1) continue;
      const int sink = reach[id].front();
      if (owner_[id] != owner_[sink]) continue;
      closure[sink].push_back(id);
      closureCost[sink] += cost_[id];
    }

    const long long fair = (total + workers_ - 1) / workers_;
    const long long budget = fair + fair / 4;
    auto imbalanceOf = [&](const std::vector<long long>& loads) {
      long long maxLoad = 0;
      for (long long part : loads) maxLoad = std::max(maxLoad, part);
      return maxLoad / std::max(1.0, static_cast<double>(total) / workers_);
    };

    const PlanMetrics pre = measure();
    double imbalance = imbalanceOf(load);
    const double preImbalance = imbalance;
    int moves = 0;
    constexpr int kMaxMoves = 1024;
    while (imbalance > 1.2 && moves < kMaxMoves) {
      int heavy = 0;
      for (int w = 1; w < workers_; ++w) {
        if (load[w] > load[heavy]) heavy = w;
      }
      // Three largest loads outside `heavy` so the post-move maximum of any
      // (closure, target) pair is available in O(1).
      int top1 = -1, top2 = -1, top3 = -1;
      for (int w = 0; w < workers_; ++w) {
        if (w == heavy) continue;
        if (top1 < 0 || load[w] >= load[top1]) {
          top3 = top2; top2 = top1; top1 = w;
        } else if (top2 < 0 || load[w] >= load[top2]) {
          top3 = top2; top2 = w;
        } else if (top3 < 0 || load[w] >= load[top3]) {
          top3 = w;
        }
      }
      auto maxOutside = [&](int target) {
        long long result = 0;
        for (int idx : {top1, top2, top3}) {
          if (idx < 0 || idx == target) continue;
          result = std::max(result, load[idx]);
        }
        return result;
      };
      // Blocks are single-instance: moving a closure shifts its cost exactly,
      // so the post-move maximum is known without re-scoring. Any part with
      // budget headroom is a valid target; the lightest one usually wins.
      int bestSink = -1;
      int bestTarget = -1;
      long long bestMax = load[heavy];
      long long bestTargetLoad = 0;
      for (int sink = 0; sink < count; ++sink) {
        if (closure[sink].empty() || owner_[sink] != heavy) continue;
        const long long cost = closureCost[sink];
        const long long fromHeavy = load[heavy] - cost;
        for (int target = 0; target < workers_; ++target) {
          if (target == heavy) continue;
          const long long targetLoad = load[target] + cost;
          if (targetLoad > budget) continue;
          const long long nextMax =
              std::max(maxOutside(target), std::max(fromHeavy, targetLoad));
          if (nextMax >= load[heavy]) continue;  // must lower the max
          if (bestSink < 0 || nextMax < bestMax ||
              (nextMax == bestMax &&
               (targetLoad < bestTargetLoad ||
                (targetLoad == bestTargetLoad &&
                 (target < bestTarget ||
                  (target == bestTarget && sink < bestSink)))))) {
            bestSink = sink;
            bestTarget = target;
            bestMax = nextMax;
            bestTargetLoad = targetLoad;
          }
        }
      }
      if (bestSink < 0) break;
      for (int id : closure[bestSink]) owner_[id] = bestTarget;
      load[heavy] -= closureCost[bestSink];
      load[bestTarget] += closureCost[bestSink];
      imbalance = imbalanceOf(load);
      ++moves;
    }
    if (imbalance > 1.2) {
      int heavy = 0;
      for (int w = 1; w < workers_; ++w) {
        if (load[w] > load[heavy]) heavy = w;
      }
      long long headroom = 0;
      for (int w = 0; w < workers_; ++w) {
        if (w != heavy) headroom = std::max(headroom, budget - load[w]);
      }
      long long biggest = 0;
      int closures = 0;
      for (int sink = 0; sink < count; ++sink) {
        if (closure[sink].empty() || owner_[sink] != heavy) continue;
        ++closures;
        biggest = std::max(biggest, closureCost[sink]);
      }
      fprintf(stderr,
              "[cppEmitter-mt] node-sink block-rebalance-stall max-part=%d "
              "load=%lld budget=%lld best-headroom=%lld closures=%d "
              "biggest=%lld shared-residue=%lld\n",
              heavy, load[heavy], budget, headroom, closures, biggest,
              load[heavy] - biggest);
    }
    if (moves == 0) {
      reportBlockRebalance_ = true;
      reportBlockRebalanceMoves_ = 0;
      reportBlockRebalancePreImbalance_ = preImbalance;
      reportBlockRebalancePostImbalance_ = preImbalance;
      reportBlockRebalancePreTokens_ = pre.tokenGroups;
      reportBlockRebalancePostTokens_ = pre.tokenGroups;
      reportBlockRebalancePreMakespan_ = pre.makespan;
      reportBlockRebalancePostMakespan_ = pre.makespan;
      return;
    }
    const PlanMetrics post = measure();
    fprintf(stderr,
            "[cppEmitter-mt] node-sink block-rebalance moves=%d max/mean=%.3f->%.3f "
            "tokens=%zu->%zu makespan=%lld->%lld budget=%lld\n",
            moves, preImbalance, imbalance, pre.tokenGroups,
            post.tokenGroups, pre.makespan, post.makespan, budget);
    reportBlockRebalance_ = true;
    reportBlockRebalanceMoves_ = moves;
    reportBlockRebalancePreImbalance_ = preImbalance;
    reportBlockRebalancePostImbalance_ = imbalance;
    reportBlockRebalancePreTokens_ = pre.tokenGroups;
    reportBlockRebalancePostTokens_ = post.tokenGroups;
    reportBlockRebalancePreMakespan_ = pre.makespan;
    reportBlockRebalancePostMakespan_ = post.makespan;
  }

  void replicatePureNodes() {
    cloneOn_.assign(workers_, {});
    long long total = 0;
    for (int cost : cost_) total += cost;
    reportFairShare_ = (total + workers_ - 1) / workers_;
    reportBudget_ = static_cast<long long>(reportFairShare_ * cloneBudgetCap());

    const int originalCount = static_cast<int>(tasks_.size());
    for (int id = 0; id < originalCount; ++id) {
      if (pure_[id] == 0) continue;
      std::map<int, std::vector<int>> consumersByWorker;
      for (int consumer : succs_[id]) {
        if (owner_[consumer] != owner_[id]) {
          consumersByWorker[owner_[consumer]].push_back(consumer);
        }
      }
      for (const auto& entry : consumersByWorker) {
        const int worker = entry.first;
        const std::vector<int>& consumers = entry.second;
        const long long saved = static_cast<long long>(consumers.size()) *
            (mtcost::tokenLatency + mtcost::remoteCheck + mtcost::remotePublish);
        if (saved <= cost_[id]) continue;
        if (load_[worker] + cost_[id] > reportBudget_) continue;
        commitClone(id, worker, consumers);
      }
    }
    for (const Clone& clone : clones_) reportRemovedEdges_ += clone.redirected;
  }

  void commitClone(int originalTask, int worker, const std::vector<int>& consumers) {
    // Whole-task cloning: every member gets a copy, the instruction stream is
    // redirected wholesale and consumers are rewired per member. Works for
    // single-node microtasks and pure SuperNode groups alike.
    MtTask& source = tasks_[originalTask];
    Assert(!source.members.empty(), "cannot clone an empty task");
    Node* const sourceFront = source.members.front();
    std::vector<Node*> copies;
    std::unordered_map<Node*, Node*> replacements;
        for (Node* original : source.members) {
      Node* copy = original->dup(NODE_OTHERS,
                                 original->name + "$MT_NS$" +
                                     std::to_string(cloneCounter_++));
      copy->nodeIsRoot = true;
      replacements[original] = copy;
            copies.push_back(copy);
    }
    std::unordered_map<Node*, Node*> producerMap = cloneOn_[worker];
    producerMap.insert(replacements.begin(), replacements.end());
    std::map<std::string, std::string> instructionNames;
    for (const auto& entry : producerMap) {
      instructionNames.emplace(entry.first->name, entry.second->name);
    }
    for (size_t index = 0; index < source.members.size(); ++index) {
      Node* original = source.members[index];
      Node* copy = copies[index];
      for (ExpTree* assignment : original->assignTree) {
        ExpTree* duplicated = assignment->dup();
        replaceReferences(duplicated, producerMap);
        copy->assignTree.push_back(duplicated);
      }
      SuperNode* super = new SuperNode();
      super->superType = SUPER_VALID;
      super->add_member(copy);
      graph_.sortedSuper.push_back(super);
      graph_.allNodes.push_back(copy);
      plan_.emissionNodes_.push_back(copy);
      plan_.partitionGroupByNode_[copy] = plan_.partitionGroupCount_++;
      // Expression-level producers become graph neighbours; addPrev/addNext
      // maintain prev/next and depPrev/depNext on both sides.
      copy->updateConnect();
    }

    const int cloneTaskId = static_cast<int>(tasks_.size());
    MtTask cloneTask;
    cloneTask.members = copies;
    cloneTask.insts = new std::vector<InstInfo>(*source.insts);
    redirectInstructions(*cloneTask.insts, instructionNames);
    cloneTask.owner = worker;
    cloneTask.kind = MtTaskKind::Normal;
    cloneTask.globalNodeCount = source.globalNodeCount;
    cloneTask.cost = mtLoweredTaskCost(cloneTask);
    tasks_.push_back(std::move(cloneTask));
    taskNode_.push_back(copies.size() == 1 ? copies.front() : nullptr);
    isClone_.push_back(1);
    pure_.push_back(0);
    cost_.push_back(tasks_[cloneTaskId].cost);
    preds_.push_back({});
    succs_.push_back({});
    owner_.push_back(worker);
    load_[worker] += tasks_[cloneTaskId].cost;
    for (Node* copy : copies) taskOf_[copy] = cloneTaskId;

    std::set<int> producerTasks;
    for (Node* copy : copies) collectProducerTasks(copy, cloneTaskId, producerTasks);
    preds_[cloneTaskId].assign(producerTasks.begin(), producerTasks.end());
    for (int producer : preds_[cloneTaskId]) succs_[producer].push_back(cloneTaskId);

    for (int consumer : consumers) {
      // The consumer now reads the local copies: same worker, so the edge
      // keeps the chain order (local wait) instead of a ready token.
      redirectInstructions(*tasks_[consumer].insts, instructionNames);
      Node* consumerNode = taskNode_[consumer];
      std::vector<Node*> consumerNodes;
      if (consumerNode != nullptr) {
        consumerNodes.push_back(consumerNode);
      } else {
        consumerNodes = tasks_[consumer].members;
      }
      for (Node* node : consumerNodes) {
        for (const auto& entry : replacements) {
          Node* original = entry.first;
          Node* copy = entry.second;
          const bool data = node->prev.erase(original) != 0;
          const bool dep = node->depPrev.erase(original) != 0;
          original->next.erase(node);
          original->depNext.erase(node);
          if (data) {
            node->prev.insert(copy);
            copy->next.insert(node);
          }
          if (data || dep) {
            node->depPrev.insert(copy);
            copy->depNext.insert(node);
          }
        }
        for (ExpTree* tree : node->assignTree) replaceReferences(tree, replacements);
      }
      std::vector<int>& consumerPreds = preds_[consumer];
      consumerPreds.erase(std::remove(consumerPreds.begin(), consumerPreds.end(),
                                      originalTask),
                          consumerPreds.end());
      consumerPreds.push_back(cloneTaskId);
      std::sort(consumerPreds.begin(), consumerPreds.end());
      succs_[cloneTaskId].push_back(consumer);
      std::vector<int>& originalSuccs = succs_[originalTask];
      originalSuccs.erase(std::remove(originalSuccs.begin(), originalSuccs.end(),
                                      consumer),
                          originalSuccs.end());
    }
    std::sort(succs_[cloneTaskId].begin(), succs_[cloneTaskId].end());
    succs_[cloneTaskId].erase(std::unique(succs_[cloneTaskId].begin(),
                                          succs_[cloneTaskId].end()),
                              succs_[cloneTaskId].end());

    for (const auto& entry : replacements) cloneOn_[worker][entry.first] = entry.second;
    clones_.push_back({sourceFront, copies.front(), worker,
                       static_cast<int>(consumers.size())});
  }

  // p2sim-style linear coarsening of each worker chain: contiguous intervals
  // of pure computation merged by DP, minimizing merged lowered cost plus
  // cross-block activation edge cost. Side-effect microtasks (and ext tasks)
  // never merge, so reset/pending/side-effect order is untouched.
  void coarsenChains() {
    const int count = static_cast<int>(tasks_.size());
    std::vector<std::vector<int>> chains(workers_);
    for (int id = 0; id < count; ++id) chains[owner_[id]].push_back(id);
    std::vector<int> position(count, -1);  // index inside the owner's chain
    for (int w = 0; w < workers_; ++w) {
      for (size_t index = 0; index < chains[w].size(); ++index) {
        position[chains[w][index]] = static_cast<int>(index);
      }
    }

    std::vector<int> statements(count, 0);
    for (int id = 0; id < count; ++id) statements[id] = statementCount(tasks_[id]);
    const bool v2 = blockV2Enabled();
    auto mergeable = [&](int id) {
      // Pure SuperNode groups (taskNode_ == nullptr) merge like any other
      // pure computation; ext microtasks carry kind ExtModule and never do.
      if (tasks_[id].kind != MtTaskKind::Normal) return false;
      if (pure_[id] != 0 || isClone_[id] != 0) return true;
      // Block v2 (default off): unique plain-assignment microtasks join the
      // intervals too. They keep pure_ == 0, so the clone machinery never
      // touches the merged block and the state-commit owner stays unique.
      return v2 && v2UniqueMergeable(id);
    };

    std::vector<MtTask> blocks;
    std::vector<int> blockOwner;
    std::vector<int> oldToBlock(count, -1);
    std::vector<int> multiplicity(count, 0);

    for (int w = 0; w < workers_; ++w) {
      const std::vector<int>& chain = chains[w];
      const int n = static_cast<int>(chain.size());
      if (n == 0) continue;
      std::vector<long long> best(n + 1, 0);
      std::vector<int> blockStart(n + 1, -1);
      for (int i = 0; i < n; ++i) {
        const int task = chain[i];
        if (!mergeable(task)) {
          best[i + 1] = best[i] + cost_[task];
          blockStart[i + 1] = i;
          continue;
        }
        long long sumCost = 0;
        int sumStatements = 0;
        int crossBlocks = 0;  // distinct cross-worker successor tasks
        bool hasCrossOut = false;
        bool hasCrossIn = false;
        std::vector<int> touched;
        long long bestTotal = LLONG_MAX;
        int bestStart = -1;
        int j = i;
        while (j >= 0 && mergeable(chain[j])) {
          const int member = chain[j];
          sumCost += cost_[member];
          sumStatements += statements[member];
          for (int successor : succs_[member]) {
            // Successors are forward, so same-owner successors at or before i
            // are internalized by the interval [j..i].
            if (owner_[successor] == w && position[successor] <= i) continue;
            if (multiplicity[successor]++ == 0) {
              touched.push_back(successor);
              if (owner_[successor] != w) {
                ++crossBlocks;
                hasCrossOut = true;
              }
            }
          }
          for (int predecessor : preds_[member]) {
            if (owner_[predecessor] != w) hasCrossIn = true;
          }
          // A block cycle needs a cross-worker edge both into and out of the
          // merged interval (all edges are forward in the global order, so
          // same-worker cut edges alone can never return). A multi-task
          // interval is therefore only admissible when its cross-worker cut
          // edges point one way. Singletons never merge anything and stay
          // admissible; the statement cap cannot split one microtask either.
          const bool singleton = j == i;
          if (singleton || !hasCrossOut || !hasCrossIn) {
            if (sumStatements <= nodeSinkBlockMaxInsts() || singleton) {
              const long long estimate =
                  sumCost - static_cast<long long>(i - j) * mtcost::taskOverhead +
                  static_cast<long long>(crossBlocks) * kNodeSinkEdgeCost;
              const long long total = best[j] + estimate;
              if (total <= bestTotal) {  // ties: prefer the larger interval
                bestTotal = total;
                bestStart = j;
              }
            }
          }
          if (sumStatements > nodeSinkBlockMaxInsts()) break;
          --j;
        }
        Assert(bestStart >= 0, "coarsening DP failed at chain position %d", i);
        best[i + 1] = bestTotal;
        blockStart[i + 1] = bestStart;
        for (int successor : touched) multiplicity[successor] = 0;
      }

      // Materialize the chosen intervals as blocks (worker-major ids).
      int i = n - 1;
      std::vector<std::pair<int, int>> intervals;  // [start, end] chain indices
      while (i >= 0) {
        const int start = blockStart[i + 1];
        intervals.push_back({start, i});
        i = start - 1;
      }
      std::reverse(intervals.begin(), intervals.end());
      for (const auto& interval : intervals) {
        const int blockId = static_cast<int>(blocks.size());
        if (interval.first == interval.second) {
          const int task = chain[interval.first];
          if (v2 && pure_[task] == 0 && isClone_[task] == 0 &&
              !mergeable(task)) {
            ++reportV2Pinned_[v2PinClass(tasks_[task])];
          }
          blocks.push_back(std::move(tasks_[task]));
          blockOwner.push_back(w);
          oldToBlock[task] = blockId;
          continue;
        }
        MtTask block;
        block.owner = w;
        block.kind = MtTaskKind::Normal;
        std::set<Node*> memberNodes;
        for (int index = interval.first; index <= interval.second; ++index) {
          const int task = chain[index];
          if (v2 && pure_[task] == 0 && isClone_[task] == 0) {
            ++reportV2MergedUnique_;
          }
          MtTask& member = tasks_[task];
          block.members.insert(block.members.end(), member.members.begin(),
                               member.members.end());
          memberNodes.insert(member.members.begin(), member.members.end());
          block.globalNodeCount += member.globalNodeCount;
          oldToBlock[task] = blockId;
        }
        // Chain order is the global topological order restricted to w, so the
        // concatenated instruction streams stay dependency-forward.
        block.insts = new std::vector<InstInfo>();
        for (int index = interval.first; index <= interval.second; ++index) {
          const std::vector<InstInfo>& insts = *tasks_[chain[index]].insts;
          block.insts->insert(block.insts->end(), insts.begin(), insts.end());
        }
        collectBlockLocalNodes(block, memberNodes);
        block.cost = mtLoweredTaskCost(block);
        blocks.push_back(std::move(block));
        blockOwner.push_back(w);
      }
    }

    // Rebuild the task DAG at block granularity: an edge survives when it
    // crosses a block boundary.
    const int blockCount = static_cast<int>(blocks.size());
    std::vector<std::vector<int>> blockPreds(blockCount);
    for (int id = 0; id < count; ++id) {
      const int home = oldToBlock[id];
      Assert(home >= 0, "task %d was not assigned to a coarsened block", id);
      std::set<int> producers;
      for (int producer : preds_[id]) {
        const int producerBlock = oldToBlock[producer];
        if (producerBlock != home) producers.insert(producerBlock);
      }
      auto& destination = blockPreds[home];
      destination.insert(destination.end(), producers.begin(), producers.end());
    }
    for (auto& producers : blockPreds) {
      std::sort(producers.begin(), producers.end());
      producers.erase(std::unique(producers.begin(), producers.end()),
                      producers.end());
    }
    std::vector<std::vector<int>> blockSuccs(blockCount);
    for (int id = 0; id < blockCount; ++id) {
      for (int producer : blockPreds[id]) blockSuccs[producer].push_back(id);
    }

    std::vector<Node*> blockTaskNode(blockCount, nullptr);
    std::vector<uint8_t> blockIsClone(blockCount, 0);
    std::vector<uint8_t> blockPure(blockCount, 0);
    std::vector<int> blockCost(blockCount, 0);
    for (int id = 0; id < blockCount; ++id) blockCost[id] = blocks[id].cost;
    // Map singleton blocks back to their node for scope-local bookkeeping.
    std::unordered_map<Node*, int> nodeBlock;
    for (int id = 0; id < count; ++id) {
      if (taskNode_[id] != nullptr && oldToBlock[id] >= 0 &&
          blocks[oldToBlock[id]].members.size() == 1) {
        nodeBlock[taskNode_[id]] = oldToBlock[id];
      }
    }
    for (int id = 0; id < blockCount; ++id) {
      if (blocks[id].members.size() == 1) {
        auto found = nodeBlock.find(blocks[id].members.front());
        if (found != nodeBlock.end()) blockTaskNode[id] = found->first;
      }
    }

    tasks_.swap(blocks);
    taskNode_.swap(blockTaskNode);
    isClone_.swap(blockIsClone);
    pure_.swap(blockPure);
    preds_.swap(blockPreds);
    succs_.swap(blockSuccs);
    owner_.swap(blockOwner);
    cost_.swap(blockCost);
    for (auto& entry : taskOf_) {
      entry.second = oldToBlock[entry.second];
    }
    coarsened_ = true;
  }

  // Same rule as collectTaskLocalNodes, with the block member set as scope: a
  // NODE_OTHERS whose every live consumer is inside the block (or outside the
  // compute phase) can live in the block's stack frame.
  void collectBlockLocalNodes(MtTask& block, const std::set<Node*>& memberNodes) {
    if (blockV2Enabled()) return;
    for (Node* node : memberNodes) {
      if (node->status != VALID_NODE || node->type != NODE_OTHERS ||
          node->isReset() || resetDependencies_.count(node) != 0) {
        continue;
      }
      // V2 blocks can contain pinned arrays larger than the clone cap; those
      // stay class storage instead of a per-cycle stack frame. Pure members
      // are already capped at 32 elements by isPureCloneable, so this is a
      // no-op for the default path.
      if (nodeArrayElements(node) > 32) continue;
      bool local = true;
      for (Node* next : node->next) {
        const auto consumer = taskOf_.find(next);
        if (consumer != taskOf_.end() &&
            memberNodes.count(next) == 0) {
          local = false;
          break;
        }
      }
      if (local && node->isArray()) {
        for (Node* next : node->depNext) {
          const auto consumer = taskOf_.find(next);
          if (consumer != taskOf_.end() && memberNodes.count(next) == 0) {
            local = false;
            break;
          }
        }
      }
      if (local) block.localNodes.insert(node);
    }
  }

  // Restore one forward topological task order (stable by creation id) for
  // token reduction and local waits; worker chains are re-derived from it.
  void finalizeOrder() {
    const int count = static_cast<int>(tasks_.size());
    if (count == 0) return;
    std::vector<int> indegree(count, 0);
    for (int id = 0; id < count; ++id) {
      std::sort(preds_[id].begin(), preds_[id].end());
      preds_[id].erase(std::unique(preds_[id].begin(), preds_[id].end()),
                       preds_[id].end());
      indegree[id] = static_cast<int>(preds_[id].size());
    }
    std::priority_queue<int, std::vector<int>, std::greater<int>> ready;
    for (int id = 0; id < count; ++id) {
      if (indegree[id] == 0) ready.push(id);
    }
    std::vector<int> order;
    order.reserve(count);
    std::vector<int> newId(count, -1);
    while (!ready.empty()) {
      const int id = ready.top();
      ready.pop();
      newId[id] = static_cast<int>(order.size());
      order.push_back(id);
      for (int successor : succs_[id]) {
        if (--indegree[successor] == 0) ready.push(successor);
      }
    }
    Assert(order.size() == static_cast<size_t>(count),
           "cycle in node-sink plan after coarsening");

    std::vector<MtTask> reordered;
    std::vector<Node*> reorderedNode;
    std::vector<uint8_t> reorderedClone;
    std::vector<uint8_t> reorderedPure;
    std::vector<std::vector<int>> reorderedPreds;
    std::vector<int> reorderedOwner;
    std::vector<int> reorderedCost;
    reordered.reserve(order.size());
    for (int id : order) {
      reordered.push_back(std::move(tasks_[id]));
      reorderedNode.push_back(taskNode_[id]);
      reorderedClone.push_back(isClone_[id]);
      reorderedPure.push_back(pure_[id]);
      reorderedPreds.push_back(preds_[id]);
      reorderedOwner.push_back(owner_[id]);
      reorderedCost.push_back(cost_[id]);
    }
    tasks_.swap(reordered);
    taskNode_.swap(reorderedNode);
    isClone_.swap(reorderedClone);
    pure_.swap(reorderedPure);
    preds_.swap(reorderedPreds);
    owner_.swap(reorderedOwner);
    cost_.swap(reorderedCost);
    for (std::vector<int>& producers : preds_) {
      for (int& producer : producers) producer = newId[producer];
    }
    succs_.assign(tasks_.size(), {});
    for (size_t id = 0; id < tasks_.size(); ++id) {
      for (int producer : preds_[id]) succs_[producer].push_back(static_cast<int>(id));
    }
    for (auto& entry : taskOf_) entry.second = newId[entry.second];
  }

  void singleWorkerPlan() {
    owner_.assign(tasks_.size(), 0);
    load_.assign(1, 0);
    for (int cost : cost_) load_[0] += cost;
    reportFairShare_ = load_[0];
    reportBudget_ = load_[0];
  }

  PlanMetrics measure() const {
    PlanMetrics result;
    const int count = static_cast<int>(tasks_.size());
    result.tasks = tasks_.size();
    std::vector<long long> finish(count, 0);
    std::vector<long long> workerFree(workers_, 0);
    for (int id = 0; id < count; ++id) {
      long long start = workerFree[owner_[id]];
      std::set<int> remoteOwners;
      for (int producer : preds_[id]) {
        if (owner_[producer] == owner_[id]) {
          start = std::max(start, finish[producer]);
        } else {
          start = std::max(start, finish[producer] + mtcost::tokenLatency);
          remoteOwners.insert(owner_[producer]);
          ++result.crossEdges;
        }
      }
      result.tokenGroups += remoteOwners.size();
      finish[id] = start + std::max(1, cost_[id]);
      workerFree[owner_[id]] = finish[id];
      result.makespan = std::max(result.makespan, finish[id]);
    }
    return result;
  }

  // Same rule as collectTaskLocalNodes for a single-node scope: a NODE_OTHERS
  // whose every live consumer is outside the compute phase (state machinery)
  // may live in the task's stack frame.
  bool isScopeLocal(Node* node) const {
    if (node->status != VALID_NODE || node->type != NODE_OTHERS || node->isReset() ||
        resetDependencies_.count(node) != 0) {
      return false;
    }
    for (Node* next : node->next) {
      if (taskOf_.count(next) != 0) return false;
    }
    if (node->isArray()) {
      for (Node* next : node->depNext) {
        if (taskOf_.count(next) != 0) return false;
      }
    }
    return true;
  }

  void rebuildPlanMetadata() {
    plan_.localNodes_.clear();
    for (size_t id = 0; id < tasks_.size(); ++id) {
      MtTask& task = tasks_[id];
      // Multi-member blocks keep the localNodes chosen during coarsening.
      if (task.members.size() == 1) {
        task.localNodes.clear();
        if (taskNode_[id] != nullptr && isScopeLocal(task.members.front())) {
          task.localNodes.insert(task.members.front());
        }
      }
      task.predecessors = preds_[id];
      task.successors = succs_[id];
      for (Node* node : task.localNodes) plan_.localNodes_.insert(node);
    }
    plan_.tasks_ = tasks_;
    plan_.taskByNode_ = taskOf_;
  }

  void computeSchedule(MtNodeSinkSchedule& schedule) {
    const int count = static_cast<int>(tasks_.size());
    schedule.owner = owner_;
    schedule.chains.assign(workers_, {});
    // The per-worker chains are the global topological order filtered by
    // owner: every same-worker dependency is forward inside the chain.
    for (int id = 0; id < count; ++id) {
      schedule.chains[owner_[id]].push_back(id);
    }
    after_ = measure();
    schedule.makespan = after_.makespan;
  }

  void writeReport() const {
    const size_t unique = reportUnique_;
    const size_t pure = reportPure_;
    const PlanMetrics& view = after_;
    fprintf(stderr,
            "[cppEmitter-mt] node-sink tasks=%zu unique=%zu pure=%zu replicas=%zu "
            "cloned-nodes=%zu removed-edges=%zu cross-edges=%zu tokens=%zu "
            "fair-share=%lld budget=%lld makespan=%lld coarsen=%s "
            "coarsen-tasks=%zu->%zu coarsen-tokens=%zu->%zu "
            "coarsen-makespan=%lld->%lld block-insts<=%d\n",
            view.tasks, unique, pure, clones_.size(), clones_.size(),
            reportRemovedEdges_, view.crossEdges, view.tokenGroups,
            reportFairShare_, reportBudget_, view.makespan,
            coarsened_ ? "on" : "off", before_.tasks, view.tasks,
            before_.tokenGroups, view.tokenGroups, before_.makespan,
            view.makespan, nodeSinkBlockMaxInsts());
    if (reportV2_) {
      size_t pinnedTotal = 0;
      for (const auto& entry : reportV2Pinned_) pinnedTotal += entry.second;
      std::string pinned;
      for (const auto& entry : reportV2Pinned_) {
        pinned += " " + entry.first + "=" + std::to_string(entry.second);
      }
      fprintf(stderr,
              "[cppEmitter-mt] node-sink block-v2 merged-unique=%zu "
              "pinned-singletons=%zu%s\n",
              reportV2MergedUnique_, pinnedTotal, pinned.c_str());
    }
    if (globalConfig.OutputDir.empty()) return;
    FILE* out = std::fopen((globalConfig.OutputDir + "/mt-node-sink.csv").c_str(), "w");
    Assert(out != nullptr, "cannot write node-sink report");
    fprintf(out, "metric,value\n");
    fprintf(out, "workers,%d\n", workers_);
    fprintf(out, "tasks,%zu\n", view.tasks);
    fprintf(out, "unique_tasks,%zu\n", unique);
    fprintf(out, "pure_tasks,%zu\n", pure);
    fprintf(out, "replica_tasks,%zu\n", clones_.size());
    fprintf(out, "cloned_nodes,%zu\n", clones_.size());
    fprintf(out, "removed_cross_edges,%zu\n", reportRemovedEdges_);
    fprintf(out, "residual_cross_edges,%zu\n", view.crossEdges);
    fprintf(out, "residual_token_groups,%zu\n", view.tokenGroups);
    fprintf(out, "fair_share,%lld\n", reportFairShare_);
    fprintf(out, "clone_budget,%lld\n", reportBudget_);
    fprintf(out, "estimated_makespan,%lld\n", view.makespan);
    fprintf(out, "coarsen,%d\n", coarsened_ ? 1 : 0);
    fprintf(out, "partitioner,%s\n", reportPartitioner_.c_str());
    if (reportRebalance_) {
      fprintf(out, "rebalance_moves,%d\n", reportRebalanceMoves_);
      fprintf(out, "rebalance_pre_imbalance,%.3f\n", reportRebalancePreImbalance_);
      fprintf(out, "rebalance_post_imbalance,%.3f\n",
              reportRebalancePostImbalance_);
      fprintf(out, "rebalance_pre_tokens,%zu\n", reportRebalancePreTokens_);
      fprintf(out, "rebalance_post_tokens,%zu\n", reportRebalancePostTokens_);
      fprintf(out, "rebalance_pre_makespan,%lld\n", reportRebalancePreMakespan_);
      fprintf(out, "rebalance_post_makespan,%lld\n",
              reportRebalancePostMakespan_);
    }
    if (reportBlockRebalance_) {
      fprintf(out, "block_rebalance_moves,%d\n", reportBlockRebalanceMoves_);
      fprintf(out, "block_rebalance_pre_imbalance,%.3f\n",
              reportBlockRebalancePreImbalance_);
      fprintf(out, "block_rebalance_post_imbalance,%.3f\n",
              reportBlockRebalancePostImbalance_);
      fprintf(out, "block_rebalance_pre_tokens,%zu\n",
              reportBlockRebalancePreTokens_);
      fprintf(out, "block_rebalance_post_tokens,%zu\n",
              reportBlockRebalancePostTokens_);
      fprintf(out, "block_rebalance_pre_makespan,%lld\n",
              reportBlockRebalancePreMakespan_);
      fprintf(out, "block_rebalance_post_makespan,%lld\n",
              reportBlockRebalancePostMakespan_);
    }
    fprintf(out, "block_max_statements,%d\n", nodeSinkBlockMaxInsts());
    fprintf(out, "coarsen_pre_tasks,%zu\n", before_.tasks);
    fprintf(out, "coarsen_post_tasks,%zu\n", view.tasks);
    fprintf(out, "coarsen_pre_tokens,%zu\n", before_.tokenGroups);
    fprintf(out, "coarsen_post_tokens,%zu\n", view.tokenGroups);
    fprintf(out, "coarsen_pre_makespan,%lld\n", before_.makespan);
    fprintf(out, "coarsen_post_makespan,%lld\n", view.makespan);
    if (reportV2_) {
      size_t pinnedTotal = 0;
      for (const auto& entry : reportV2Pinned_) pinnedTotal += entry.second;
      fprintf(out, "block_v2,1\n");
      fprintf(out, "v2_merged_unique_tasks,%zu\n", reportV2MergedUnique_);
      fprintf(out, "v2_pinned_singletons,%zu\n", pinnedTotal);
      for (const auto& entry : reportV2Pinned_) {
        fprintf(out, "v2_pinned_%s,%zu\n", entry.first.c_str(), entry.second);
      }
    }
    for (int worker = 0; worker < workers_; ++worker) {
      long long cost = 0;
      size_t count = 0;
      for (size_t id = 0; id < tasks_.size(); ++id) {
        if (owner_[id] == worker) {
          cost += cost_[id];
          ++count;
        }
      }
      fprintf(out, "worker_cost_%d,%lld\n", worker, cost);
      fprintf(out, "worker_tasks_%d,%zu\n", worker, count);
    }
    std::unordered_map<Node*, Node*> originalOf;
    for (const Clone& clone : clones_) originalOf[clone.copy] = clone.original;
    for (size_t id = 0; id < tasks_.size(); ++id) {
      Node* node = taskNode_[id];
      if (node == nullptr) continue;
      auto found = originalOf.find(node);
      const int originalId = found == originalOf.end() ? node->id : found->second->id;
      fprintf(out, "task_map,%zu,%d\n", id, originalId);
    }
    for (size_t index = 0; index < clones_.size(); ++index) {
      const Clone& clone = clones_[index];
      const std::string prefix = "clone_" + std::to_string(index);
      fprintf(out, "%s_original,%s\n", prefix.c_str(),
              clone.original->name.c_str());
      fprintf(out, "%s_replica,%s\n", prefix.c_str(), clone.copy->name.c_str());
      fprintf(out, "%s_worker,%d\n", prefix.c_str(), clone.worker);
      fprintf(out, "%s_redirected_consumers,%d\n", prefix.c_str(),
              clone.redirected);
    }
    Assert(std::fclose(out) == 0, "cannot finish node-sink report");
  }
};

}  // namespace

bool MtNodeSinkPlanner::enabled() { return nodeSinkEnabled(); }

void MtNodeSinkPlanner::plan(graph& graph, MtTaskPlan& plan, int workerCount,
                             MtNodeSinkSchedule& schedule) {
  NodeSinkPlanner(graph, plan, std::max(1, workerCount)).run(schedule);
}
