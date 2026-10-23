#ifndef MT_TASK_SCHEDULE_H
#define MT_TASK_SCHEDULE_H

#include "mtTaskPartition.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

class graph;
class Node;
class SuperNode;

// One cost definition for lowered originals and lowered replication blocks.
int mtLoweredTaskCost(const MtTask& task);

struct MtReset {
  struct Instruction {
    uint8_t type = 0;
    std::string text;
  };

  SuperNode* super = nullptr;
  int id = -1;
  int chunkCount = 0;
  std::vector<Instruction> body;
};

// One pinned MTask per worker, including workers with an empty state body.
// Members own current-state storage / reset operations. The writer lists
// reference compute-phase producers of prior-cycle pending writes; those
// producers remain compute tasks and are NOT state-task members.
struct MtStateTask : MtTask {
  MtStateTask() { kind = MtTaskKind::StateUpdate; owner = -1; }
  int chunkCount = 0;
  size_t registerStorageBytes = 0;
  size_t memoryWriteBytes = 0;
  size_t sparseRegisterWriteBytes = 0;
  std::vector<Node*> registers;
  std::vector<Node*> memoryWriters;
  std::vector<Node*> sparseRegisterWriters;
  std::vector<MtReset::Instruction> body;
};

struct MtWorkerPlan {
  int readySlotCount_ = 1;
  // GSIM_MT_READY_WORDS=1 packs 64 ready slots per uint64_t word. Slot runs
  // within a bank are reordered so each publish task owns whole words only;
  // publishing becomes one release store per word and waiting becomes one
  // acquire load plus a precomputed mask-parity compare per word.
  bool readyWords_ = false;
  int readyWordCount_ = 1;
  std::vector<std::vector<int>> workerTasks_;
  std::vector<int> waitSlots_;
  std::vector<int> storeSlots_;
  std::vector<int> localWaitSlots_;
  // Word-mode replacements for waitSlots_/storeSlots_. waitWords_[i] pairs
  // with waitMasks_[i]; task waitBegin/waitEnd and storeBegin/storeEnd index
  // these tables instead of the per-slot tables.
  std::vector<int> waitWords_;
  std::vector<uint64_t> waitMasks_;
  std::vector<int> storeWords_;
  // stateTasks_[w] -> all-worker state barrier -> workerTasks_[w].
  // State and compute task IDs have separate namespaces; no state task is
  // included in the token DAG, lookahead, or asynchronous-reset replay.
  std::vector<MtStateTask> stateTasks_;
  std::unordered_map<Node*, int> stateTaskByNode_;
  std::vector<MtReset> resets_;
};

class MtWorkerBuilder {
 public:
  static void build(graph& graph, MtTaskPlan& tasks, MtWorkerPlan& workers,
                    int workerCount, int resetChunk);
};

#endif
