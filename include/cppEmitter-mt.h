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
  void emitInstructions(const std::vector<InstInfo>& instructions, int indent,
                        int specialTaskId = -1);
  void emitTask(const Task& task, int taskId, int indent);
  void emitWorkerChainFunction(const std::string& className, int worker);
  void buildRegisterStorageNames();
  void buildActivationPlan();
  void buildMirrorPlan();
  void writeTaskPhaseReport() const;
  std::string mapRegisterNames(const std::string& text) const;
  void emitTaskActivation(const std::vector<int>& consumers, int sourceWorker, int indent);

  graph& graph_;
  bool enabled_ = false;
  bool eventGeneration_ = false;
  int workerCount_ = 1;
  int targetTasks_ = 2400;
  int resetChunk_ = 4096;
  int lookaheadWindow_ = 128;
  bool lookaheadStats_ = false;
  bool nodeSinkEmission_ = false;
  bool joinStateEnabled_ = false;
  bool bitmapDispatchEnabled_ = true;
  // GSIM_MT_NODE_SINK_MIRROR=1: for cross-worker token groups whose sources
  // all sit in earlier capped phases than the consumer, the phase barrier
  // already orders the shared-storage publish/pull; drop the ready token.
  bool mirrorEnabled_ = false;
  std::map<Node*, std::string> packedRegisterNames_;
  std::unordered_map<std::string, std::string> packedRegisterNamesByText_;
  std::vector<uint8_t> mirrorDroppedSlot_;
  size_t mirrorDroppedTokens_ = 0;
  size_t mirrorKeptTokens_ = 0;
  struct ActivationWatch {
    Node* node = nullptr;
    std::vector<int> consumers;
  };
  std::vector<std::vector<ActivationWatch>> activationWatches_;
  std::unordered_map<Node*, std::vector<int>> stateNodeActivationTasks_;
  std::unordered_map<Node*, std::vector<int>> inputActivationTasks_;
  std::vector<uint8_t> taskAlwaysActive_;
  std::vector<int> eventTaskPhase_;
  std::vector<int> eventTopoOrder_;
  int eventPhaseCount_ = 0;
  int eventPhaseCap_ = 0;
};

#endif
