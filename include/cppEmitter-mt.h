#ifndef CPP_EMITTER_MT_H
#define CPP_EMITTER_MT_H

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include "mtTaskPartition.h"
#include "mtTaskSchedule.h"

class graph;
class Node;
class SuperNode;

// Adds a dense, fixed-owner executor to the C++ emitted by the original
// single-threaded backend. Planning data stays in this object and never leaks
// into Node or SuperNode.
class CppEmitterMt : private MtTaskPlan, private MtWorkerPlan {
 public:
  explicit CppEmitterMt(graph& graph);
  CppEmitterMt(const CppEmitterMt&) = delete;
  CppEmitterMt& operator=(const CppEmitterMt&) = delete;

  bool enabled() const;
  void prepare();
  void emitHeaderPreamble(FILE* header) const;
  void emitClassMembers(FILE* header) const;
  void emitConstructorInit();
  void emitConstructorStart();
  void emitDefinitions();
  void emitStep();
  bool isTaskLocal(Node* node) const;
  bool isWorkerLocal(Node* node) const;
  int workerLocalOwner(Node* node) const;

 private:
  using Task = MtTask;
  using Reset = MtReset;
  using ResetJoin = MtResetJoin;

  // Worker-local lookahead bookkeeping for one dispatch-table position.
  // kind: 0 = strict in-order entry, 1 = may run while the chain head is
  // blocked, 2 = barrier that no later entry may overtake.
  enum : uint8_t {
    kLookaheadStrict = 0,
    kLookaheadEligible = 1,
    kLookaheadBarrier = 2,
  };

  struct LookaheadInfo {
    uint8_t kind = 0;
    uint32_t predBegin = 0;
    uint32_t predEnd = 0;
  };

  void emitText(int indent, bool canStartFile, const std::string& text);
  void emitResetBodyFunction(const std::string& name, const std::string& condition,
                             const std::vector<Reset::Instruction>& body, int chunkCount);
  void emitResetFunction(const Reset& reset);
  void emitAsyncResetWorkerFunction(const Reset& reset, const Reset::Worker& worker);
  void emitInstructions(const std::vector<InstInfo>& instructions, int indent);
  void emitTask(const Task& task, int indent);
  void buildLookaheadPlan();
  bool taskLookaheadEligible(const Task& task) const;

  graph& graph_;
  bool enabled_ = false;
  int workerCount_ = 1;
  int targetTasks_ = 1600;
  int lookahead_ = 0;
  int resetChunk_ = 4096;
  std::vector<std::vector<LookaheadInfo>> lookaheadPlan_;
  std::vector<uint32_t> lookaheadPreds_;
};

#endif
