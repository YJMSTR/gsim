#include "mtRepcutTop.h"

#include "common.h"
#include "mtCostModel.h"
#include "mtTaskPartition.h"
#include "mtTaskSchedule.h"
#include "mtReplicationSchedule.h"
#include "Node.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace {

// RepCut prior: duplicated-compute share ceiling from the paper's measured
// 3.81%. Without the profile's own calibration the first-shot cap uses one
// third of it (the cost cap is one third of the expected spin savings).
constexpr double kRepcutPriorPct = 3.81;

struct RepcutConfig {
  int topN = 0;
  int depth = 0;
  int maxCostPctMilli = 3810;  // 3.81% RepCut prior, in thousandths
  int coneMode = 0;            // 1 = sub-task pure-cone granularity
  int maxOps = 0;              // per-cone pure-op budget (0 = unlimited)
  std::string csvPath;
};

// Percent of total static task cost that replication may add, measured by
// the RepCut paper at 3.81%. This ceiling binds even when the profiled
// spin-savings budget is larger (D12: replication-rate budget cap).
int envPercent(const char* name, int fallbackMilli) {
  const char* value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') return fallbackMilli;
  char* end = nullptr;
  errno = 0;
  const double parsed = std::strtod(value, &end);
  if (errno != 0 || end == value || *end != '\0' || parsed < 0.0 || parsed > 1000.0) {
    return fallbackMilli;
  }
  return static_cast<int>(parsed * 1000.0);
}

int envInt(const char* name, int fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') return fallback;
  char* end = nullptr;
  errno = 0;
  const long parsed = std::strtol(value, &end, 10);
  if (errno != 0 || end == value || *end != '\0' || parsed < 0 ||
      parsed > 1000000) {
    return fallback;
  }
  return static_cast<int>(parsed);
}

RepcutConfig parseConfig() {
  RepcutConfig config;
  config.topN = envInt("GSIM_MT_REPCUT_TOP", 0);
  config.depth = envInt("GSIM_MT_REPCUT_DEPTH", 0);
  config.maxCostPctMilli = envPercent("GSIM_MT_REPCUT_MAX_COST_PCT", 3810);
  config.coneMode = envInt("GSIM_MT_REPCUT_MODE", 0);
  config.maxOps = envInt("GSIM_MT_REPCUT_MAX_OPS", 0);
  const char* path = std::getenv("GSIM_MT_REPCUT_CSV");
  config.csvPath = path != nullptr && path[0] != '\0'
                       ? std::string(path)
                       : "profile/state-graph-hybrid/analysis/bubble-top-wait-slots.csv";
  return config;
}

std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  std::stringstream stream(line);
  while (std::getline(stream, field, ',')) fields.push_back(field);
  return fields;
}

// spinner_workers can hold a single worker id or a joined mask summary; the
// first integer is the representative spinning consumer worker.
int firstInteger(const std::string& text) {
  size_t index = 0;
  while (index < text.size() &&
         !std::isdigit(static_cast<unsigned char>(text[index]))) {
    ++index;
  }
  if (index >= text.size()) return -1;
  long value = 0;
  while (index < text.size() &&
         std::isdigit(static_cast<unsigned char>(text[index]))) {
    value = value * 10 + (text[index++] - '0');
    if (value > 1000000) return -1;
  }
  return static_cast<int>(value);
}

// Nanosecond counters legitimately reach tens of billions; no id cap here.
long long integerField(const std::string& text) {
  size_t index = 0;
  while (index < text.size() &&
         !std::isdigit(static_cast<unsigned char>(text[index]))) {
    ++index;
  }
  if (index >= text.size()) return -1;
  long long value = 0;
  while (index < text.size() &&
         std::isdigit(static_cast<unsigned char>(text[index]))) {
    value = value * 10 + (text[index++] - '0');
    if (value > 1000000000000000000LL) return -1;
  }
  return value;
}

bool loadSlotRows(const std::string& path, std::vector<mtrepcut::SlotRow>* rows) {
  FILE* file = std::fopen(path.c_str(), "r");
  if (file == nullptr) return false;
  char buffer[8192];
  bool header = true;
  std::unordered_map<std::string, size_t> column;
  while (std::fgets(buffer, sizeof(buffer), file) != nullptr) {
    std::string line(buffer);
    if (!line.empty() && line.back() == '\n') line.pop_back();
    if (line.empty()) continue;
    const std::vector<std::string> fields = splitCsvLine(line);
    if (header) {
      header = false;
      for (size_t index = 0; index < fields.size(); ++index) {
        column[fields[index]] = index;
      }
      continue;
    }
    auto field = [&](const char* name) -> const std::string& {
      static const std::string empty;
      auto found = column.find(name);
      return found == column.end() || found->second >= fields.size() ? empty
                                                                     : fields[found->second];
    };
    mtrepcut::SlotRow row;
    row.slotId = firstInteger(field("slot_id"));
    row.producerTask = firstInteger(field("producer_task"));
    row.producerWorker = firstInteger(field("producer_worker"));
    row.spinnerWorker = firstInteger(field("spinner_workers"));
    row.totalNs = integerField(field("total_ns"));
    row.crossCcd = firstInteger(field("cross_ccd"));
    rows->push_back(row);
  }
  std::fclose(file);
  return true;
}

// Profile calibration: the summed task-compute core-ns of the profiled run.
// The per-cycle cost-unit conversion cancels the cycle count, so only the
// total compute nanoseconds are needed (budget = totalCost * savingNs /
// (3 * computeNs)).
long long loadProfileComputeNs(const std::string& csvPath) {
  const size_t slash = csvPath.find_last_of('/');
  const std::string directory =
      slash == std::string::npos ? std::string(".") : csvPath.substr(0, slash);
  FILE* file =
      std::fopen((directory + "/bubble-worker-summary.csv").c_str(), "r");
  if (file == nullptr) return -1;
  char buffer[8192];
  bool header = true;
  size_t computeColumn = std::numeric_limits<size_t>::max();
  long long total = 0;
  bool any = false;
  while (std::fgets(buffer, sizeof(buffer), file) != nullptr) {
    std::string line(buffer);
    if (!line.empty() && line.back() == '\n') line.pop_back();
    if (line.empty()) continue;
    const std::vector<std::string> fields = splitCsvLine(line);
    if (header) {
      header = false;
      for (size_t index = 0; index < fields.size(); ++index) {
        if (fields[index] == "task_compute_ns") computeColumn = index;
      }
      continue;
    }
    if (computeColumn >= fields.size()) break;
    const long long value = integerField(fields[computeColumn]);
    if (value >= 0) {
      total += value;
      any = true;
    }
  }
  std::fclose(file);
  return any ? total : -1;
}

// Replace full C++ identifiers (including '$') without touching strings or
// comments — the same tokenization the scheduling-side redirector uses.
std::string redirectText(
    const std::string& text,
    const std::unordered_map<std::string, std::string>& names) {
  if (names.empty()) return text;
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


// Swap node pointers inside an expression tree (root and lvalue) to the
// replacement copies and drop stale compute caches. Mirrors the
// scheduling-side cone redirector so worker-local alias discovery in
// emitTask sees the copy references.
void replaceTreeReferences(ExpTree* tree,
                           const std::unordered_map<Node*, Node*>& replacements) {
  if (tree == nullptr || replacements.empty()) return;
  std::vector<ENode*> pending{tree->getRoot(), tree->getlval()};
  while (!pending.empty()) {
    ENode* expression = pending.back();
    pending.pop_back();
    if (expression == nullptr) continue;
    auto found = replacements.find(expression->nodePtr);
    if (found != replacements.end()) expression->nodePtr = found->second;
    expression->computeInfo = nullptr;
    for (ENode* child : expression->child) {
      if (child != nullptr) pending.push_back(child);
    }
  }
}

// Statement-level observation calls. The producer still executes them once;
// a replica that repeated them would emit every printf/assert twice.
bool isObservationStatement(const std::string& text) {
  size_t index = 0;
  while (index < text.size() && std::isspace(static_cast<unsigned char>(text[index]))) {
    ++index;
  }
  return text.compare(index, 8, "gprintf(") == 0 ||
         text.compare(index, 8, "gAssert(") == 0;
}

// Ports (writer/readwriter) publish into shared per-port pending slots that
// the state phase commits exactly once; register src/dst names are mapped to
// packed state storage at emission. Replicas copy plain value nodes only and
// keep writing the shared pending slots and packed fields with identical
// values (benign same-value stores, committed after the worker join).
bool copyableMember(const Node* node) {
  switch (node->type) {
    case NODE_WRITER:
    case NODE_READWRITER:
    case NODE_EXT:
    case NODE_SPECIAL:
    case NODE_REG_RESET:
    case NODE_REG_SRC:
    case NODE_REG_DST:
    case NODE_MEMORY:
      return false;
    default:
      return true;
  }
}

}  // namespace

namespace mtrepcut {

bool repcutTopEnabled() { return parseConfig().topN > 0; }

Report applyRepcutTop(graph& g, MtTaskPlan& plan, std::vector<int>& owners,
                      std::vector<std::vector<int>>& chains, int workerCount) {
  Report report;
  const RepcutConfig config = parseConfig();
  if (config.topN <= 0 || workerCount <= 1) return report;
  report.active = true;
  report.csvPath = config.csvPath;
  report.topN = config.topN;
  std::vector<SlotRow> rows;
  if (!loadSlotRows(config.csvPath, &rows) || rows.empty()) {
    report.rejected[rows.empty() ? "csv-empty" : "csv-unreadable"] = 1;
    return report;
  }
  report.csvRows = static_cast<int>(rows.size());
  for (size_t index = 0; index < rows.size(); ++index) {
    rows[index].rank = static_cast<int>(index) + 1;
  }

  std::vector<MtTask>& tasks = plan.tasks_;
  report.totalCost = 0;
  for (const MtTask& task : tasks) report.totalCost += task.cost;
  const long long computeNs = loadProfileComputeNs(config.csvPath);
  report.priorFallbackCap = computeNs <= 0;

  std::vector<int> workerPos(tasks.size(), -1);
  auto rebuildPositions = [&]() {
    std::fill(workerPos.begin(), workerPos.end(), -1);
    for (int worker = 0; worker < workerCount; ++worker) {
      const std::vector<int>& chain = chains[static_cast<size_t>(worker)];
      for (size_t position = 0; position < chain.size(); ++position) {
        workerPos[static_cast<size_t>(chain[position])] = static_cast<int>(position);
      }
    }
  };
  rebuildPositions();
  if (std::getenv("GSIM_MT_REPCUT_DEBUG") != nullptr) {
    for (size_t taskId = 0; taskId < tasks.size(); ++taskId) {
      fprintf(stderr, "[repcut-dbg] entry task %zu(w%d) preds=[", taskId,
              owners[taskId]);
      for (int predecessor : tasks[taskId].predecessors) {
        fprintf(stderr, "%d ", predecessor);
      }
      fprintf(stderr, "]\n");
    }
  }

  // Tasks whose lowered text was rewritten to read replica copies. A replica
  // of such a task would inherit references that may live in another
  // worker's private state, so they are excluded from further replication.
  std::set<int> redirectedTasks;
  std::set<std::pair<int, int>> targets;  // (producer, consumer worker)
  std::set<int> replicatedProducers;
  int sequence = 0;

  // True when the task owns a writer port of a memory that has more than
  // one writer/readwriter port across different tasks.
  auto producerWritesSharedMemory = [&](const MtTask& producer) {
    for (Node* member : producer.members) {
      if (member->type != NODE_WRITER && member->type != NODE_READWRITER) continue;
      Node* memory = member->parent;
      if (memory == nullptr || memory->type != NODE_MEMORY) continue;
      std::set<int> writerTasks;
      for (Node* port : memory->member) {
        if (port->status != VALID_NODE) continue;
        if (port->type != NODE_WRITER && port->type != NODE_READWRITER) continue;
        auto owner = plan.taskByNode_.find(port);
        if (owner != plan.taskByNode_.end()) writerTasks.insert(owner->second);
      }
      if (writerTasks.size() > 1) return true;
    }
    return false;
  };

  // Node-level data sources of a producer: every cross-task value its
  // members read. Transitive reduction may drop task edges that node reads
  // still imply (their ordering survives only through same-worker chain
  // sequencing, which a replica on another worker cannot inherit), so
  // replicas must be ordered by the true read set, not the reduced one.
  auto truePredecessors = [&](int producerId) {
    std::vector<int> sources;
    const MtTask& producer = tasks[static_cast<size_t>(producerId)];
    for (Node* member : producer.members) {
      for (Node* input : member->prev) {
        if (producer.localNodes.count(input) != 0) continue;
        auto found = plan.taskByNode_.find(input);
        if (found == plan.taskByNode_.end()) continue;  // state or interface
        if (found->second == producerId) continue;      // intra-task value
        sources.push_back(found->second);
      }
    }
    std::sort(sources.begin(), sources.end());
    sources.erase(std::unique(sources.begin(), sources.end()), sources.end());
    return sources;
  };

  auto reject = [&report](const SlotRow& row, const char* reason) {
    ++report.rejected[reason];
    report.decisions.push_back(std::to_string(row.rank) + "," +
                               std::to_string(row.slotId) + "," +
                               std::to_string(row.producerTask) + ",reject," + reason);
  };

  // Creates the replica task of `producerId` on `consumerWorker`, inserts it
  // into the chain at `insertPosition` and registers its duplicated nodes.
  // Returns the new task id, or -1 with `*reason` set (nothing mutated).
  auto replicate = [&](int producerId, int consumerWorker, int insertPosition,
                       const std::unordered_map<Node*, Node*>& extraRedirects,
                       const std::vector<std::pair<int, int>>& replacePreds,
                       std::string* reason) -> int {
    const MtTask& producer = tasks[static_cast<size_t>(producerId)];
    if (producer.kind != MtTaskKind::Normal) {
      *reason = "task-kind";
      return -1;
    }
    if (producer.insts == nullptr || producer.insts->empty()) {
      *reason = "empty-body";
      return -1;
    }
    if (redirectedTasks.count(producerId) != 0) {
      *reason = "reads-replica-copies";
      return -1;
    }
    std::vector<Node*> copyable;
    for (Node* member : producer.members) {
      if (copyableMember(member)) copyable.push_back(member);
    }
    if (copyable.empty()) {
      *reason = "no-copyable-members";
      return -1;
    }
    // D3 hard constraint (multi-write-port memory): a memory whose write
    // ports span several tasks commits per-port pending slots; replicating
    // one writer re-executes its slots with identical values, but the
    // ordering web around a shared symbol is exactly where a silent
    // schedule bug costs a whole NEMU run, so such producers stay
    // unreplicated in the first shot.
    if (producerWritesSharedMemory(producer)) {
      *reason = "multi-writer-memory";
      return -1;
    }
    // Upstream dev-mt replication invariant: only pure data cones may be
    // replicated. Register-destination members compile to packed-state
    // side-effect writes (mtRegisterDstW*), and OTHER tasks can read that
    // next-value node mid-cycle. A replica never re-writes the packed field
    // (it is excluded from copying), so a consumer rewired to the replica
    // could read the field before the original producer's write lands.
    // Producers whose side-effect members have cross-task readers stay
    // unreplicated.
    for (Node* member : producer.members) {
      if (member->type != NODE_REG_DST) continue;
      *reason = "side-effect-write-member";
      return -1;
    }
    bool hasValueStatement = false;
    for (const InstInfo& instruction : *producer.insts) {
      if (instruction.infoType != SUPER_INFO_STR ||
          !isObservationStatement(instruction.inst)) {
        hasValueStatement = true;
        break;
      }
    }
    if (!hasValueStatement) {
      *reason = "observation-only-body";
      return -1;
    }

    const int replicaId = static_cast<int>(tasks.size());
    if (report.firstReplicaId < 0) report.firstReplicaId = replicaId;
    MtTask replica;
    replica.kind = MtTaskKind::Normal;
    replica.owner = consumerWorker;
    auto* super = new SuperNode();
    g.sortedSuper.push_back(super);
    const int group = plan.partitionGroupCount_++;
    std::unordered_map<Node*, Node*> replacements;
    for (Node* member : copyable) {
      Node* copy = member->dup(member->type,
                               member->name + "$MT_RC$" + std::to_string(sequence++));
      copy->nodeIsRoot = true;
      replacements[member] = copy;
      replica.members.push_back(copy);
      if (producer.localNodes.count(member) != 0) {
        replica.localNodes.insert(copy);
        plan.localNodes_.insert(copy);
      }
      super->add_member(copy);
      g.allNodes.push_back(copy);
      plan.emissionNodes_.push_back(copy);
      plan.taskByNode_[copy] = replicaId;
      plan.partitionGroupByNode_[copy] = group;
    }
    // Every copy carries the producer's assignment trees with member
    // references redirected to the copies. Emission discovers worker-local
    // aliases through these trees; without them a worker-local copy would be
    // referenced unqualified.
    std::unordered_map<Node*, Node*> treeRedirects = replacements;
    for (const auto& entry : extraRedirects) {
      treeRedirects.emplace(entry.first, entry.second);
    }
    for (Node* member : copyable) {
      Node* copy = replacements[member];
      for (ExpTree* assignment : member->assignTree) {
        ExpTree* duplicate = assignment->dup();
        replaceTreeReferences(duplicate, treeRedirects);
        copy->assignTree.push_back(duplicate);
      }
    }

    // Every cross-task input the replica reads must stay globally visible:
    // registering the copy as a use prevents worker-local storage of a value
    // that would otherwise be private to the producer's worker.
    for (Node* member : producer.members) {
      Node* copy = replacements.count(member) != 0 ? replacements[member] : nullptr;
      if (copy == nullptr) continue;
      for (Node* input : member->prev) {
        if (producer.localNodes.count(input) != 0) continue;
        if (plan.taskByNode_.count(input) == 0) continue;  // state or interface
        if (replacements.count(input) != 0) continue;      // producer-local member
        input->next.insert(copy);
        copy->prev.insert(input);
      }
    }

    std::unordered_map<std::string, std::string> names;
    for (const auto& entry : replacements) {
      names.emplace(entry.first->name, entry.second->name);
    }
    for (const auto& entry : extraRedirects) {
      names.emplace(entry.first->name, entry.second->name);
    }
    auto* instructions = new std::vector<InstInfo>();
    instructions->reserve(producer.insts->size());
    for (const InstInfo& instruction : *producer.insts) {
      if (instruction.infoType == SUPER_INFO_STR &&
          isObservationStatement(instruction.inst)) {
        continue;  // printf/assert already fire once in the producer
      }
      instructions->emplace_back(instruction.infoType, instruction.node,
                                 redirectText(instruction.inst, names));
    }
    replica.insts = instructions;
    replica.predecessors = producer.predecessors;
    for (int source : truePredecessors(producerId)) {
      replica.predecessors.push_back(source);
    }
    std::sort(replica.predecessors.begin(), replica.predecessors.end());
    for (const auto& replacement : replacePreds) {
      auto& preds = replica.predecessors;
      preds.erase(std::remove(preds.begin(), preds.end(), replacement.first), preds.end());
      preds.push_back(replacement.second);
      auto& oldSuccessors = tasks[static_cast<size_t>(replacement.first)].successors;
      oldSuccessors.erase(std::remove(oldSuccessors.begin(), oldSuccessors.end(), replicaId),
                          oldSuccessors.end());
    }
    std::sort(replica.predecessors.begin(), replica.predecessors.end());
    replica.predecessors.erase(
        std::unique(replica.predecessors.begin(), replica.predecessors.end()),
        replica.predecessors.end());
    int globalCount = 0;
    for (Node* member : replica.members) {
      if (replica.localNodes.count(member) == 0) ++globalCount;
    }
    replica.globalNodeCount = globalCount;
    replica.cost = mtLoweredTaskCost(replica);

    for (int predecessor : replica.predecessors) {
      tasks[static_cast<size_t>(predecessor)].successors.push_back(replicaId);
    }
    tasks.push_back(std::move(replica));
    owners.push_back(consumerWorker);
    workerPos.push_back(-1);
    std::vector<int>& chain = chains[static_cast<size_t>(consumerWorker)];
    if (std::getenv("GSIM_MT_REPCUT_DEBUG") != nullptr) {
      fprintf(stderr, "[repcut-dbg] replica %d of task %d onto w%d at pos %d preds=[",
              replicaId, producerId, consumerWorker, insertPosition);
      for (int predecessor : replica.predecessors) {
        fprintf(stderr, "%d(w%d) ", predecessor, owners[static_cast<size_t>(predecessor)]);
      }
      fprintf(stderr, "] chain=[");
      for (int taskId : chain) fprintf(stderr, "%d ", taskId);
      fprintf(stderr, "]\n");
    }
    chain.insert(chain.begin() + insertPosition, replicaId);
    rebuildPositions();
    return replicaId;
  };

  // Sub-task granularity: replicate only the PURE combinational share of
  // the producer that the consumer worker actually reads. Roots are the
  // producer's plain value members with readers on the consumer worker;
  // the cone walk stops at state inputs and at values already produced on
  // the consumer worker (chain-ordered locals) or elsewhere (token
  // boundaries). Register sinks and memory writes are structurally outside
  // the cone — the sink task keeps them.
  auto replicateCone = [&](int producerId, int consumerWorker, int insertPosition,
                           std::string* reason) -> int {
    const MtTask& producer = tasks[static_cast<size_t>(producerId)];
    if (producer.insts == nullptr || producer.insts->empty()) {
      *reason = "empty-body";
      return -1;
    }
    if (redirectedTasks.count(producerId) != 0) {
      *reason = "reads-replica-copies";
      return -1;
    }
    std::vector<Node*> roots;
    for (Node* member : producer.members) {
      if (member->type != NODE_OTHERS) continue;
      if (producer.localNodes.count(member) != 0) continue;
      for (Node* reader : member->next) {
        auto owner = plan.taskByNode_.find(reader);
        if (owner != plan.taskByNode_.end() &&
            owners[static_cast<size_t>(owner->second)] == consumerWorker) {
          roots.push_back(member);
          break;
        }
      }
    }
    if (roots.empty()) {
      *reason = "no-pure-share-read";
      return -1;
    }
    std::sort(roots.begin(), roots.end(), [](Node* a, Node* b) { return a->id < b->id; });

    const int replicaId = static_cast<int>(tasks.size());
    // Structural cone walk over the producer's own pure members: any input
    // that is another pure member of the same producer is absorbed; every
    // other input (state, interface, other tasks' values, this producer's
    // local temporaries) is a boundary. Copies lower through the same
    // per-node machinery as the original, so no operator whitelist applies.
    const int nodeBudget = config.maxOps > 0 ? config.maxOps : 1000000000;
    std::vector<Node*> coneNodes;      // dependency-first (post-order)
    std::set<Node*> coneSet(roots.begin(), roots.end());
    std::vector<Node*> stack(roots.begin(), roots.end());
    std::set<Node*> visited;
    while (!stack.empty()) {
      Node* node = stack.back();
      stack.pop_back();
      if (!visited.insert(node).second) continue;
      bool pureProducerMember = node->type == NODE_OTHERS &&
                                 plan.taskByNode_.count(node) != 0 &&
                                 plan.taskByNode_[node] == producerId &&
                                 producer.localNodes.count(node) == 0 &&
                                 node->assignTree.size() == 1;
      if (pureProducerMember) {
        for (Node* input : node->prev) {
          if (coneSet.count(input) != 0) continue;
          coneSet.insert(input);
          stack.push_back(input);
        }
      }
    }
    for (Node* node : coneSet) {
      if (node->type != NODE_OTHERS || plan.taskByNode_.count(node) == 0 ||
          plan.taskByNode_[node] != producerId ||
          producer.localNodes.count(node) != 0 || node->assignTree.size() != 1) {
        continue;
      }
      coneNodes.push_back(node);
    }
    if (coneNodes.empty()) {
      *reason = "empty-cone";
      return -1;
    }
    if (static_cast<int>(coneNodes.size()) > nodeBudget) {
      *reason = "cone-node-budget";
      return -1;
    }
    std::sort(coneNodes.begin(), coneNodes.end(),
              [](Node* a, Node* b) { return a->order < b->order; });
    struct MtReplicationConeShim {};
    (void)sizeof(MtReplicationConeShim);

    MtTask replica;
    replica.kind = MtTaskKind::Normal;
    replica.owner = consumerWorker;
    auto* super = new SuperNode();
    g.sortedSuper.push_back(super);
    const int group = plan.partitionGroupCount_++;
    std::unordered_map<Node*, Node*> replacements;
    std::unordered_map<Node*, Node*> boundaryCopies;
    std::vector<Node*>& ordered = coneNodes;  // already deps-first by node order
    for (Node* member : ordered) {
      Node* copy = member->dup(member->type,
                               member->name + "$MT_RC$" + std::to_string(sequence++));
      copy->nodeIsRoot = true;
      replacements[member] = copy;
      replica.members.push_back(copy);
      super->add_member(copy);
      g.allNodes.push_back(copy);
      plan.emissionNodes_.push_back(copy);
      plan.taskByNode_[copy] = replicaId;
      plan.partitionGroupByNode_[copy] = group;
    }
    for (Node* member : ordered) {
      Node* copy = replacements[member];
      for (ExpTree* assignment : member->assignTree) {
        ExpTree* duplicate = assignment->dup();
        replaceTreeReferences(duplicate, replacements);
        copy->assignTree.push_back(duplicate);
      }
      for (Node* input : member->prev) {
        if (replacements.count(input) != 0) continue;
        if (producer.localNodes.count(input) != 0) continue;
        if (plan.taskByNode_.count(input) == 0) continue;
        input->next.insert(copy);
        copy->prev.insert(input);
      }
    }
    // Instruction stream: each cone node's assignment, deps-first, with
    // member references redirected to the copies (printf/assert dropped as
    // in the task-body path).
    std::unordered_map<std::string, std::string> names;
    for (const auto& entry : replacements) {
      names.emplace(entry.first->name, entry.second->name);
    }
    auto* instructions = new std::vector<InstInfo>();
    for (Node* member : ordered) {
      StmtTree tree;
      tree.root = new StmtNode(OP_STMT_SEQ);
      for (ExpTree* assignment : member->assignTree) {
        std::vector<int> emptyPath;
        tree.mergeExpTree(assignment, emptyPath, emptyPath, member);
      }
      std::vector<InstInfo> nodeInsts;
      tree.compute(nodeInsts);
      for (const InstInfo& instruction : nodeInsts) {
        if (instruction.infoType == SUPER_INFO_STR &&
            isObservationStatement(instruction.inst)) {
          continue;
        }
        instructions->emplace_back(instruction.infoType, instruction.node,
                                   redirectText(instruction.inst, names));
      }
    }
    if (instructions->empty()) {
      *reason = "observation-only-cone";
      delete instructions;
      return -1;
    }
    replica.insts = instructions;
    // Boundary inputs define the replica's predecessors: every boundary
    // node's owning task (deduplicated, sorted).
    std::set<int> sources;
    for (Node* member : ordered) {
      for (Node* boundary : member->prev) {
      if (producer.localNodes.count(boundary) != 0) continue;
      auto found = plan.taskByNode_.find(boundary);
      if (found == plan.taskByNode_.end()) continue;
      if (found->second == producerId) continue;
      sources.insert(found->second);
      }
    }
    for (Node* member : ordered) {
      for (Node* input : member->prev) {
        if (replacements.count(input) != 0) continue;
        if (producer.localNodes.count(input) != 0) continue;
        auto found = plan.taskByNode_.find(input);
        if (found == plan.taskByNode_.end() || found->second == producerId) continue;
        sources.insert(found->second);
      }
    }
    replica.predecessors.assign(sources.begin(), sources.end());
    int globalCount = 0;
    for (Node* copy : replica.members) { (void)copy; ++globalCount; }
    replica.globalNodeCount = globalCount;
    replica.cost = mtLoweredTaskCost(replica);
    for (int predecessor : replica.predecessors) {
      tasks[static_cast<size_t>(predecessor)].successors.push_back(replicaId);
    }
    tasks.push_back(std::move(replica));
    owners.push_back(consumerWorker);
    workerPos.push_back(-1);
    std::vector<int>& chain = chains[static_cast<size_t>(consumerWorker)];
    chain.insert(chain.begin() + insertPosition, replicaId);
    rebuildPositions();
    return replicaId;
  };

  // Points every consumer of `producerId` on `consumerWorker` at the local
  // replica: the ready-token edge P->C becomes an in-chain rep->C edge and
  // the consumer's lowered text reads the replica's copies. Returns the
  // number of rewired consumers.
  auto rewireConsumers = [&](int producerId, int replicaId, int producerWorker,
                             int consumerWorker) -> size_t {
    const MtTask& producer = tasks[static_cast<size_t>(producerId)];
    MtTask& replica = tasks[static_cast<size_t>(replicaId)];
    std::unordered_map<std::string, std::string> names;
    std::unordered_map<Node*, Node*> nodeRedirects;
    size_t copyIndex = 0;
    for (Node* original : producer.members) {
      if (!copyableMember(original)) continue;
      Node* copy = replica.members[copyIndex++];
      names.emplace(original->name, copy->name);
      nodeRedirects.emplace(original, copy);
    }
    std::vector<int> consumers;
    for (int successor : producer.successors) {
      if (owners[static_cast<size_t>(successor)] == consumerWorker &&
          successor != producerId && successor != replicaId) {
        consumers.push_back(successor);
      }
    }
    for (int consumer : consumers) {
      MtTask& consumerTask = tasks[static_cast<size_t>(consumer)];
      if (std::getenv("GSIM_MT_REPCUT_DEBUG") != nullptr) {
        fprintf(stderr, "[repcut-dbg] rewire consumer %d of task %d (rep %d): text-has-copy=%d\n",
                consumer, producerId, replicaId,
                consumerTask.insts != nullptr &&
                    consumerTask.insts->front().inst.find("$MT_RC$") != std::string::npos);
      }
      auto& preds = consumerTask.predecessors;
      preds.erase(std::remove(preds.begin(), preds.end(), producerId), preds.end());
      if (std::find(preds.begin(), preds.end(), replicaId) == preds.end()) {
        preds.push_back(replicaId);
      }
      std::sort(preds.begin(), preds.end());
      preds.erase(std::unique(preds.begin(), preds.end()), preds.end());
      auto& producerSuccessors = tasks[static_cast<size_t>(producerId)].successors;
      producerSuccessors.erase(std::remove(producerSuccessors.begin(),
                                           producerSuccessors.end(), consumer),
                               producerSuccessors.end());
      if (std::find(replica.successors.begin(), replica.successors.end(), consumer) ==
          replica.successors.end()) {
        replica.successors.push_back(consumer);
      }
      for (InstInfo& instruction : *consumerTask.insts) {
        instruction.inst = redirectText(instruction.inst, names);
      }
      for (Node* member : consumerTask.members) {
        for (ExpTree* assignment : member->assignTree) {
          replaceTreeReferences(assignment, nodeRedirects);
        }
      }
      redirectedTasks.insert(consumer);
      ++report.rewiredConsumers;
      bool residual = false;
      for (int source : consumerTask.predecessors) {
        if (owners[static_cast<size_t>(source)] == producerWorker) {
          residual = true;
          break;
        }
      }
      if (residual) {
        ++report.residualGroups;
      } else {
        ++report.killedGroups;
      }
    }

    // Implicit readers: transitive reduction removes task edges that node
    // reads still imply; those readers stayed correct only through the
    // same-worker sequencing of an explicit consumer's token wait. Removing
    // that wait (the rewire above) evaporates their cover, so every task on
    // this worker that still reads the producer's values — discovered via
    // node->next, the exact data edges — must also be pointed at the local
    // replica. The replica sits before the earliest explicit consumer, hence
    // before every covered reader.
    std::set<int> handled(consumers.begin(), consumers.end());
    for (Node* original : producer.members) {
      if (!copyableMember(original)) continue;
      for (Node* readerNode : original->next) {
        auto found = plan.taskByNode_.find(readerNode);
        if (found == plan.taskByNode_.end()) continue;
        const int readerTask = found->second;
        if (readerTask == producerId || readerTask == replicaId) continue;
        if (owners[static_cast<size_t>(readerTask)] != consumerWorker) continue;
        if (!handled.insert(readerTask).second) continue;
        MtTask& task = tasks[static_cast<size_t>(readerTask)];
        if (std::find(task.predecessors.begin(), task.predecessors.end(),
                      replicaId) == task.predecessors.end()) {
          task.predecessors.push_back(replicaId);
        }
        std::sort(task.predecessors.begin(), task.predecessors.end());
        task.predecessors.erase(
            std::unique(task.predecessors.begin(), task.predecessors.end()),
            task.predecessors.end());
        if (std::find(replica.successors.begin(), replica.successors.end(),
                      readerTask) == replica.successors.end()) {
          replica.successors.push_back(readerTask);
        }
        for (InstInfo& instruction : *task.insts) {
          instruction.inst = redirectText(instruction.inst, names);
        }
        for (Node* member : task.members) {
          for (ExpTree* assignment : member->assignTree) {
            replaceTreeReferences(assignment, nodeRedirects);
          }
        }
        redirectedTasks.insert(readerTask);
        ++report.rewiredConsumers;
        ++report.residualGroups;  // conservative bookkeeping for this class
      }
    }
    return consumers.size();
  };

  const int limit = std::min<int>(config.topN, static_cast<int>(rows.size()));
  for (int index = 0; index < limit; ++index) {
    const SlotRow& row = rows[static_cast<size_t>(index)];
    if (row.producerTask < 0 || row.producerTask >= static_cast<int>(tasks.size())) {
      reject(row, "task-out-of-range");
      continue;
    }
    ++report.considered;
    if (row.spinnerWorker < 0 || row.spinnerWorker >= workerCount) {
      reject(row, "no-spinner");
      continue;
    }
    const int producerId = row.producerTask;
    const int consumerWorker = row.spinnerWorker;
    if (targets.count({producerId, consumerWorker}) != 0) {
      // A hotter row already replicated this producer for this worker; that
      // replica covers every consumer, so this slot's token is gone too.
      ++report.replicatedRows;
      report.decisions.push_back(std::to_string(row.rank) + "," +
                                 std::to_string(row.slotId) + "," +
                                 std::to_string(producerId) + ",covered,duplicate-target");
      continue;
    }
    const int producerWorker = owners[static_cast<size_t>(producerId)];
    if (producerWorker == consumerWorker) {
      reject(row, "not-cross-worker");
      continue;
    }
    if (row.producerWorker >= 0 && row.producerWorker != producerWorker) {
      // The attribution run used a different schedule; proceed keyed on the
      // structural producer task and record the drift.
      ++report.rejected["producer-worker-drift"];
    }
    if (redirectedTasks.count(producerId) != 0) {
      reject(row, "reads-replica-copies");
      continue;
    }

    auto consumersOn = [&](int taskId) {
      std::vector<int> consumers;
      for (int successor : tasks[static_cast<size_t>(taskId)].successors) {
        if (owners[static_cast<size_t>(successor)] == consumerWorker &&
            successor != taskId) {
          consumers.push_back(successor);
        }
      }
      return consumers;
    };
    std::vector<int> consumers = consumersOn(producerId);
    if (consumers.empty()) {
      reject(row, "no-consumer-on-spinner");
      continue;
    }
    int insertBefore = std::numeric_limits<int>::max();
    for (int consumer : consumers) {
      insertBefore = std::min(insertBefore, workerPos[static_cast<size_t>(consumer)]);
    }

    // Safe-insertion rule (composed closure, exact for the runtime
    // dependency graph). The runtime orders a task after (a) every earlier
    // position in its own worker's chain and (b) the whole chain prefix of
    // every token publisher it waits on — including tasks it has no DAG edge
    // to. A replica therefore deadlocks iff its runtime-causal closure
    // contains a consumer-worker task positioned after the insertion point.
    // The closure composes DAG predecessors with chain prefixes to a fixed
    // point; neither relation alone is sufficient.
    std::vector<char> closureScratch;
    auto closureSafeAfter = [&](const std::vector<int>& predecessors,
                                int consumerWorker, int after) {
      closureScratch.assign(tasks.size(), 0);
      std::vector<int> pending(predecessors);
      for (int start : predecessors) {
        if (!closureScratch[static_cast<size_t>(start)]) {
          closureScratch[static_cast<size_t>(start)] = 1;
        }
      }
      while (!pending.empty()) {
        const int current = pending.back();
        pending.pop_back();
        for (int input : tasks[static_cast<size_t>(current)].predecessors) {
          if (closureScratch[static_cast<size_t>(input)]) continue;
          closureScratch[static_cast<size_t>(input)] = 1;
          pending.push_back(input);
        }
        const int owner = owners[static_cast<size_t>(current)];
        const int position = workerPos[static_cast<size_t>(current)];
        if (owner < 0 || position < 0) continue;
        const std::vector<int>& chain = chains[static_cast<size_t>(owner)];
        for (int earlier = 0; earlier < position; ++earlier) {
          const int prefixTask = chain[static_cast<size_t>(earlier)];
          if (closureScratch[static_cast<size_t>(prefixTask)]) continue;
          closureScratch[static_cast<size_t>(prefixTask)] = 1;
          pending.push_back(prefixTask);
        }
      }
      for (size_t taskId = 0; taskId < tasks.size(); ++taskId) {
        if (!closureScratch[taskId]) continue;
        if (owners[taskId] != consumerWorker) continue;
        const int position = workerPos[taskId];
        if (position > after) after = position;
      }
      return after;
    };

    // Depth-1 tier (escalation, first shot keeps it off): replicate the
    // producer's own-worker input producers first so the main replica stops
    // waiting on the producer worker's chain entirely. Feasibility is
    // validated with conservative positions before anything is mutated.
    std::vector<std::pair<int, int>> depthCandidates;  // (pred task, planned pos)
    const std::vector<int> producerSources = [&]() {
      std::vector<int> sources = tasks[static_cast<size_t>(producerId)].predecessors;
      for (int source : truePredecessors(producerId)) sources.push_back(source);
      std::sort(sources.begin(), sources.end());
      sources.erase(std::unique(sources.begin(), sources.end()), sources.end());
      return sources;
    }();
    int plannedInsertAfter = -1;
    for (int predecessor : producerSources) {
      if (owners[static_cast<size_t>(predecessor)] != consumerWorker) continue;
      plannedInsertAfter =
          std::max(plannedInsertAfter, workerPos[static_cast<size_t>(predecessor)]);
    }
    if (config.depth >= 1) {
      std::vector<std::pair<int, int>> candidates;
      for (int predecessor : tasks[static_cast<size_t>(producerId)].predecessors) {
        if (owners[static_cast<size_t>(predecessor)] != producerWorker) continue;
        if (targets.count({predecessor, consumerWorker}) != 0) continue;
        if (redirectedTasks.count(predecessor) != 0) continue;
        const MtTask& candidate = tasks[static_cast<size_t>(predecessor)];
        if (candidate.kind != MtTaskKind::Normal || candidate.insts == nullptr ||
            candidate.insts->empty()) {
          continue;
        }
        int after = -1;
        std::vector<int> candidateSources = candidate.predecessors;
        for (int source : truePredecessors(predecessor)) {
          candidateSources.push_back(source);
        }
        std::sort(candidateSources.begin(), candidateSources.end());
        candidateSources.erase(std::unique(candidateSources.begin(), candidateSources.end()),
                               candidateSources.end());
        for (int input : candidateSources) {
          if (owners[static_cast<size_t>(input)] != consumerWorker) continue;
          after = std::max(after, workerPos[static_cast<size_t>(input)]);
        }
        after = closureSafeAfter(candidateSources, consumerWorker, after);
        candidates.emplace_back(predecessor, after + 1);
      }
      std::sort(candidates.begin(), candidates.end(),
                [](const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) {
                  return lhs.second != rhs.second ? lhs.second < rhs.second
                                                  : lhs.first < rhs.first;
                });
      int shift = 0;
      for (const auto& candidate : candidates) {
        const int position = candidate.second + shift;
        if (position >= insertBefore) break;
        depthCandidates.emplace_back(candidate.first, position);
        ++shift;
        plannedInsertAfter = std::max(plannedInsertAfter, position);
      }
    }
    plannedInsertAfter =
        closureSafeAfter(producerSources, consumerWorker, plannedInsertAfter);
    if (plannedInsertAfter + 1 > insertBefore) {
      reject(row, "ordering-conflict");
      continue;
    }

    // Cost cap: admitted replicated cost (including the depth tier) must
    // stay within one third of the expected spin savings, converted into
    // static cost units via the profile's compute/cost calibration. Without
    // calibration, one third of the RepCut prior share applies.
    long long plannedCost = tasks[static_cast<size_t>(producerId)].cost;
    for (const auto& candidate : depthCandidates) {
      plannedCost += tasks[static_cast<size_t>(candidate.first)].cost;
    }
    const long long savingsNs = report.expectedSavingNs + std::max(0LL, row.totalNs);
    const long long savingsBudget =
        computeNs > 0
            ? report.totalCost * savingsNs / (3 * computeNs)
            : static_cast<long long>(static_cast<double>(report.totalCost) *
                                     kRepcutPriorPct / 100.0 / 3.0);
    // D12 replication-rate ceiling: the RepCut prior share of the total
    // static cost binds first; the escalation tier may raise it via
    // GSIM_MT_REPCUT_MAX_COST_PCT.
    const long long priorCeiling = static_cast<long long>(
        static_cast<double>(report.totalCost) * config.maxCostPctMilli / 1000.0 / 100.0);
    const long long budget = std::min(savingsBudget, priorCeiling);
    report.budgetCost = budget;
    if (report.replicaCost + plannedCost > budget) {
      reject(row, "cost-cap");
      continue;
    }

    // Mutate: depth replicas first (their consumers on this worker also stop
    // waiting), then the main replica, then the consumer rewiring.
    std::unordered_map<Node*, Node*> depthRedirects;
    std::vector<std::pair<int, int>> replacePreds;
    for (const auto& candidate : depthCandidates) {
      std::string inner;
      const int depthReplica =
          replicate(candidate.first, consumerWorker, candidate.second, {}, {}, &inner);
      if (depthReplica < 0) {
        continue;  // optional helper; the main replica falls back to the token
      }
      targets.insert({candidate.first, consumerWorker});
      replicatedProducers.insert(candidate.first);
      ++report.depthReplicas;
      ++report.replicas;
      const MtTask& depthTask = tasks[static_cast<size_t>(depthReplica)];
      report.replicaCost += depthTask.cost;
      replacePreds.emplace_back(candidate.first, depthReplica);
      const MtTask& depthProducer = tasks[static_cast<size_t>(candidate.first)];
      size_t copyIndex = 0;
      for (Node* original : depthProducer.members) {
        if (!copyableMember(original)) continue;
        Node* depthCopy = depthTask.members[copyIndex++];
        // Producer-local members become function locals inside the depth
        // replica; another task can never reference them, so the main
        // replica keeps reading (and waiting on) their originals.
        if (depthProducer.localNodes.count(original) != 0) continue;
        depthRedirects[original] = depthCopy;
      }
      rewireConsumers(candidate.first, depthReplica, producerWorker, consumerWorker);
      ++report.replicatedRows;
      report.decisions.push_back(std::to_string(row.rank) + "," +
                                 std::to_string(row.slotId) + "," +
                                 std::to_string(candidate.first) + ",depth-replica," +
                                 std::to_string(depthReplica));
    }
    // Recompute the insertion point: depth replicas shifted the chain and
    // are predecessors of the main replica through replacePreds.
    int finalInsertAfter = -1;
    for (int predecessor : producerSources) {
      if (owners[static_cast<size_t>(predecessor)] != consumerWorker) continue;
      finalInsertAfter =
          std::max(finalInsertAfter, workerPos[static_cast<size_t>(predecessor)]);
    }
    for (const auto& replacement : replacePreds) {
      finalInsertAfter =
          std::max(finalInsertAfter, workerPos[static_cast<size_t>(replacement.second)]);
    }
    finalInsertAfter = closureSafeAfter(producerSources, consumerWorker, finalInsertAfter);
    consumers = consumersOn(producerId);
    int finalInsertBefore = std::numeric_limits<int>::max();
    for (int consumer : consumers) {
      finalInsertBefore =
          std::min(finalInsertBefore, workerPos[static_cast<size_t>(consumer)]);
    }
    if (finalInsertAfter >= finalInsertBefore) {
      // The main replica no longer fits (depth insertions pushed it past a
      // consumer). The depth replicas are independently valid, so keep them
      // and drop only the main target.
      reject(row, "ordering-conflict-after-depth");
      continue;
    }
    if (config.coneMode >= 1) {
      std::string coneReason;
      const int coneReplica =
          replicateCone(producerId, consumerWorker, finalInsertAfter + 1, &coneReason);
      if (coneReplica < 0) {
        reject(row, coneReason.c_str());
        continue;
      }
      MtTask& coneRep = tasks[static_cast<size_t>(coneReplica)];
      std::unordered_map<std::string, std::string> coneNames;
      for (Node* member : tasks[static_cast<size_t>(producerId)].members) {
        if (member->type != NODE_OTHERS) continue;
        for (Node* copy : coneRep.members) {
          if (copy->name.compare(0, member->name.size() + 8,
                                 member->name + "$MT_RC$") == 0) {
            coneNames.emplace(member->name, copy->name);
            break;
          }
        }
      }
      for (int consumer : consumers) {
        MtTask& consumerTask = tasks[static_cast<size_t>(consumer)];
        if (std::find(consumerTask.predecessors.begin(), consumerTask.predecessors.end(),
                      coneReplica) == consumerTask.predecessors.end()) {
          consumerTask.predecessors.push_back(coneReplica);
        }
        std::sort(consumerTask.predecessors.begin(), consumerTask.predecessors.end());
        consumerTask.predecessors.erase(
            std::unique(consumerTask.predecessors.begin(), consumerTask.predecessors.end()),
            consumerTask.predecessors.end());
        if (std::find(coneRep.successors.begin(), coneRep.successors.end(), consumer) ==
            coneRep.successors.end()) {
          coneRep.successors.push_back(consumer);
        }
        for (InstInfo& instruction : *consumerTask.insts) {
          instruction.inst = redirectText(instruction.inst, coneNames);
        }
        redirectedTasks.insert(consumer);
        ++report.rewiredConsumers;
        ++report.residualGroups;
      }
      targets.insert({producerId, consumerWorker});
      replicatedProducers.insert(producerId);
      ++report.replicas;
      report.replicaCost += coneRep.cost;
      report.expectedSavingNs += std::max(0LL, row.totalNs);
      ++report.replicatedRows;
      report.decisions.push_back(
          std::to_string(row.rank) + "," + std::to_string(row.slotId) + "," +
          std::to_string(producerId) + ",cone-ok," + std::to_string(coneReplica) +
          ",cost" + std::to_string(coneRep.cost));
      continue;
    }
    std::string reason;
    const int replicaId = replicate(producerId, consumerWorker, finalInsertAfter + 1,
                                    depthRedirects, replacePreds, &reason);
    if (replicaId < 0) {
      reject(row, reason.c_str());
      continue;
    }
    targets.insert({producerId, consumerWorker});
    replicatedProducers.insert(producerId);
    ++report.replicas;
    report.replicaCost += tasks[static_cast<size_t>(replicaId)].cost;
    report.expectedSavingNs += std::max(0LL, row.totalNs);
    rewireConsumers(producerId, replicaId, producerWorker, consumerWorker);
    ++report.replicatedRows;
    report.decisions.push_back(
        std::to_string(row.rank) + "," + std::to_string(row.slotId) + "," +
        std::to_string(producerId) + ",ok," + std::to_string(replicaId) + ",w" +
        std::to_string(consumerWorker) + ",cost" +
        std::to_string(tasks[static_cast<size_t>(replicaId)].cost));
  }

  // D6 note: gsim's dense memory model is read-old with per-port pending
  // slots committed by the state phase, so plain readers never require
  // writer ordering and write-forwarding ("new" memories) reads only a
  // port's own pending slots inside its own task. The multi-writer-memory
  // eligibility cap above is the conservative guard this architecture
  // needs; the wolvrix cross-partition-value whitelist has no equivalent
  // violation surface here.

  // Liveness gate (gold standard): event-driven progress simulation of the
  // transformed schedule. Every worker advances its chain cursor; a task at
  // the cursor may run when all its token publishers (last-position source
  // per producer worker) have already run. If no cursor can advance while
  // tasks remain, the schedule deadlocks at cycle 0 — abort with the
  // offending worker/position instead of shipping a hanging model.
  if (report.replicas > 0) {
    std::vector<int> chainCursor(static_cast<size_t>(workerCount), 0);
    std::vector<char> hasRun(tasks.size(), 0);
    size_t completed = 0;
    bool progressed = true;
    while (completed < tasks.size() && progressed) {
      progressed = false;
      for (int worker = 0; worker < workerCount; ++worker) {
        std::vector<int>& chain = chains[static_cast<size_t>(worker)];
        int& cursor = chainCursor[static_cast<size_t>(worker)];
        while (cursor < static_cast<int>(chain.size())) {
          const int taskId = chain[static_cast<size_t>(cursor)];
          bool ready = true;
          for (int predecessor : tasks[static_cast<size_t>(taskId)].predecessors) {
            if (!hasRun[static_cast<size_t>(predecessor)]) {
              ready = false;
              break;
            }
          }
          if (!ready) break;
          hasRun[static_cast<size_t>(taskId)] = 1;
          ++completed;
          ++cursor;
          progressed = true;
        }
      }
    }
    if (completed < tasks.size()) {
      for (int worker = 0; worker < workerCount; ++worker) {
        const std::vector<int>& chain = chains[static_cast<size_t>(worker)];
        const int cursor = chainCursor[static_cast<size_t>(worker)];
        if (cursor >= static_cast<int>(chain.size())) continue;
        const int stuck = chain[static_cast<size_t>(cursor)];
        fprintf(stderr, "[cppEmitter-mt] repcut-top FATAL: worker %d stuck at "
                        "position %d task %d waiting [",
                worker, cursor, stuck);
        for (int predecessor : tasks[static_cast<size_t>(stuck)].predecessors) {
          if (!hasRun[static_cast<size_t>(predecessor)]) {
            fprintf(stderr, "%d(w%d) ", predecessor,
                    owners[static_cast<size_t>(predecessor)]);
          }
        }
        fprintf(stderr, "]\n");
      }
      Assert(false, "repcut-top produced a deadlocking schedule");
    }
  }

  report.replicatedProducers = static_cast<int>(replicatedProducers.size());
  report.costPct = report.totalCost > 0
                       ? 100.0 * static_cast<double>(report.replicaCost) /
                             static_cast<double>(report.totalCost)
                       : 0.0;

  std::string rejections;
  for (const auto& entry : report.rejected) {
    if (!rejections.empty()) rejections += ' ';
    rejections += entry.first + "=" + std::to_string(entry.second);
  }
  fprintf(stderr,
          "[cppEmitter-mt] repcut-top csv=%s rows=%d top=%d considered=%d "
          "replicated-rows=%d replicas=%d depth-replicas=%d producers=%d "
          "consumers-rewired=%d groups-killed=%d groups-residual=%d "
          "cost=+%lldu(%.3f%%/%lldu) cap=%lldu(%s) saving-ns=%lld%s%s\n",
          report.csvPath.c_str(), report.csvRows, report.topN, report.considered,
          report.replicatedRows, report.replicas, report.depthReplicas,
          report.replicatedProducers, report.rewiredConsumers, report.killedGroups,
          report.residualGroups, report.replicaCost, report.costPct, report.totalCost,
          report.budgetCost,
          report.priorFallbackCap ? "prior-fallback" : "profile-calibrated",
          report.expectedSavingNs, rejections.empty() ? "" : " rejected ",
          rejections.c_str());

  if (!globalConfig.OutputDir.empty()) {
    FILE* file =
        std::fopen((globalConfig.OutputDir + "/mt-repcut-top.csv").c_str(), "w");
    if (file != nullptr) {
      fprintf(file, "rank,slot,producer,decision,replica,detail\n");
      for (const std::string& line : report.decisions) fprintf(file, "%s\n", line.c_str());
      std::fclose(file);
    }
  }
  return report;
}

}  // namespace mtrepcut
