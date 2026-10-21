#ifndef CPP_EMITTER_MT_H
#define CPP_EMITTER_MT_H

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <unordered_map>
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
  void emitStateInitialization();
  void emitInitPartDeclarations(FILE* header) const;
  void emitDefinitions();
  void emitStep();
  bool isTaskLocal(Node* node) const;
  bool isWorkerLocal(Node* node) const;
  int workerLocalOwner(Node* node) const;
  bool isPackedRegister(Node* node) const;
  bool isSparseRegisterDestination(Node* node) const;
  std::string packedRegisterName(Node* node) const;
  const std::vector<Node*>& emissionNodes() const;
  static constexpr size_t kInitNodesPerPart = 256;
  size_t initPartCount() const;

 private:
  using Task = MtTask;
  using StateUpdate = MtStateTask;
  using Reset = MtReset;

  void emitText(int indent, bool canStartFile, const std::string& text);
  void emitResetBodyFunction(const std::string& name, const std::string& condition,
                             const std::vector<Reset::Instruction>& body, int chunkCount);
  void emitAsyncResetFunction(const Reset& reset);
  void emitStateUpdateFunction(const StateUpdate& update);
  void emitInstructions(const std::vector<InstInfo>& instructions, int indent);
  void emitTask(const Task& task, int indent);
  void buildRegisterStorageNames();
  void buildStateReadyMasks();
  bool stateReadyActive() const { return stateReady_; }
  void decideDynamicQuery();
  void writeTaskPhaseReport() const;
  std::string mapRegisterNames(const std::string& text) const;

  graph& graph_;
  bool enabled_ = false;
  int workerCount_ = 1;
  int targetTasks_ = 2400;
  int resetChunk_ = 4096;
  int lookaheadWindow_ = 128;
  bool lookaheadStats_ = false;
  // GSIM_MT_DYNAMIC_QUERY=1: generation-time query-vs-scan decision per
  // worker (analysis/query-vs-scan-threshold.json). Bit w of queryWorkerMask_
  // selects the guarded ready-query hint layer for worker w's lookahead tail;
  // with the knob unset (or scan selected everywhere) emission is unchanged.
  bool dynamicQuery_ = false;
  uint64_t queryWorkerMask_ = 0;
  // GSIM_MT_STATE_READY=1: replace the all-worker cycle-start state barrier
  // with per-worker publication plus per-consumer state-owner masks. Bit w of
  // stateReadyMasks_[t] marks the state owner whose mtUpdateStateW<w> must
  // complete before compute task t runs; task t's own owner bit is omitted
  // (program order inside mtRunWorker). Unmapped stateful references,
  // ExtModule tasks, rheft replica tasks and OP_EXT_FUNC expressions fail
  // closed to the all-owners mask. With the knob unset every dependent
  // emission site is skipped and generation output is byte-identical.
  bool stateReady_ = false;
  std::vector<uint32_t> stateReadyMasks_;
  std::map<Node*, std::string> packedRegisterNames_;
  std::unordered_map<std::string, std::string> packedRegisterNamesByText_;
};

#endif
