#ifndef MT_TASK_SCHEDULE_H
#define MT_TASK_SCHEDULE_H

#include "mtTaskPartition.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
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

// Cone-activity probe (GSIM_MT_CONE_ACTIVITY): generation-time layout of the
// monotone dirty-bit map produced at the state barrier (on-change for packed
// register 64B chunks, on-write for memory/sparse pending commits, on-change
// for top-level inputs) plus the per-task check plans derived from the
// combined cone closure. The probe only counts would-skip; it never alters
// task execution.
struct MtConeCheckWord {
  uint32_t word = 0;
  uint64_t mask = 0;
};

struct MtConeTaskPlan {
  uint16_t sourceBits = 0;
  uint16_t coneWords = 0;
  uint32_t checkBegin = 0;
  uint32_t checkEnd = 0;
  bool always = false;  // cone contains an always-active source or side effects
  bool overCap = false; // cone spans more check words than the cap
  bool eventFree = false;
};

struct MtConeRegisterLayout {
  Node* node = nullptr;
  size_t offset = 0;
  size_t bytes = 0;
};

struct MtConePlan {
  bool enabled = false;
  uint32_t bitCount = 0;
  uint32_t dirtyWords = 0;
  uint32_t inputRegionWords = 0;
  std::vector<Node*> inputs;              // sorted by node id
  std::vector<uint32_t> inputBits;        // parallel to inputs
  struct WorkerRegion {
    uint32_t wordBegin = 0;
    uint32_t wordEnd = 0;   // exclusive, padded to whole cache lines
    uint32_t bitBegin = 0;
    uint32_t bitCount = 0;
    size_t regBlockBytes = 0;
    uint32_t regChunkCount = 0;
    std::vector<MtConeRegisterLayout> registers;
  };
  std::vector<WorkerRegion> regions;
  std::unordered_map<Node*, uint32_t> bitByWriter;   // memory + sparse writers
  std::unordered_map<Node*, uint32_t> bitByInput;
  // Packed register -> contiguous chunk-bit range [bitBegin, bitBegin+bits).
  std::unordered_map<Node*, std::pair<uint32_t, uint32_t>> regChunkBits;
  std::vector<MtConeCheckWord> checks;               // flat, indexed by plans
  std::vector<MtConeTaskPlan> tasks;                 // parallel to compute tasks
};

struct MtWorkerPlan {
  int readySlotCount_ = 1;
  std::vector<std::vector<int>> workerTasks_;
  std::vector<int> waitSlots_;
  std::vector<int> storeSlots_;
  std::vector<int> localWaitSlots_;
  // stateTasks_[w] -> all-worker state barrier -> workerTasks_[w].
  // State and compute task IDs have separate namespaces; no state task is
  // included in the token DAG, lookahead, or asynchronous-reset replay.
  std::vector<MtStateTask> stateTasks_;
  std::unordered_map<Node*, int> stateTaskByNode_;
  std::vector<MtReset> resets_;
  MtConePlan cone_;
};

class MtWorkerBuilder {
 public:
  static void build(graph& graph, MtTaskPlan& tasks, MtWorkerPlan& workers,
                    int workerCount, int resetChunk);
};

#endif
