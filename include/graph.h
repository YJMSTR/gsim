/*
  graph class which describe the whole design graph
*/

#ifndef GRAPH_H
#define GRAPH_H

#include <functional>
#include <vector>
#include <string_view>

struct MtRepCutClone;
struct MtRepCutSemanticPlan;
struct MtCoarseRegionPlan;
struct MtDenseSchedule;

class graph {
public:
  // diagnostic: content-canonical pass-boundary hashes (GSIM_DEBUG_CANON_DIR).
  void canonDumpTag(const char* tag);
  // Canonical order-free content hash of the current graph (seed input identity).
  uint64_t canonInputHash();
  // Exact FNV-1a mixes over the sorted record stream (verification-only values;
  // see seedOrder2.cpp). V1 = frozen serial whole-stream mix (champion seeds).
  // V2 = parallel segmented construction (GSIM_SEED2_CANON=v2); pure function of
  // the view list, deterministic across thread counts and machines, so the fuzz
  // harness can exercise it on synthetic streams.
  static uint64_t canonMixV1(const std::vector<std::string_view>& views);
  static uint64_t canonMixV2(const std::vector<std::string_view>& views);
  uint64_t canonRawHash();
    // seed2 pin-point wrappers (record/apply + canon verify at a pass boundary).
  void canonSeed2Record(const char* tag);
  void canonSeed2Apply(const char* tag);
private:
  FILE *srcFp;
  int srcFileIdx;
  int srcFileBytes;
  std::string srcFilePath;
  std::string srcTmpFilePath;
  std::string headerFilePath;
  std::string headerTmpFilePath;

public:
  // EmitChunk = one captured __emitSrc call (text + the byte counts/rotation
  // flags the sequential path would have used); EmitBuf = a unit's chunks.
  struct EmitChunk {
    std::string text;
    size_t countedBytes = 0;
    bool canNewFile = false;
    bool alreadyEndFunc = false;
    bool hasNextFuncDef = false;
    std::string nextFuncDef;
  };
  struct EmitBuf {
    std::vector<EmitChunk> chunks;
  };
  bool __emitSrc(int indent, bool canNewFile, bool alreadyEndFunc, const char *nextFuncDef, const char *fmt, ...);
  void rotateSrcFile(bool alreadyEndFunc, const char* nextFuncDef);
  void flushEmitBufs(std::vector<EmitBuf>& bufs);
  void emitUnitsParallel(size_t unitCount, const std::function<void(size_t)>& renderUnit);
  void emitPrintf();
  void activateNext(Node* node, std::set<int>& nextNodeId, std::string oldName, bool inStep, std::string flagName,
                    std::string activeBufferName, int indent,
                    const std::string& accumFlagName = "", bool emitActivation = true);
private:
  void activateUncondNext(Node* node, std::set<int>& activateId, bool inStep, std::string flagName,
                          std::string activeBufferName, int indent,
                          const std::string& accumFlagName = "", bool emitActivation = true);

  FILE* genHeaderStart();
  void genNodeDef(FILE* fp, Node* node);
  void genInterfaceInput(Node* input);
  void genInterfaceOutput(Node* output);
  void genStep(int subStepIdxMax, int serialFastSubStepMax = -1, const std::string& serialFastSuffix = "", bool denseExecutorValid = false);
  int genNodeStepStart(SuperNode* node, uint64_t mask, int idx, std::string flagName, int indent, bool skipAdmissionGuard = false);
  int genNodeStepEnd(SuperNode* node, int indent, bool skipAdmissionGuard = false);
  void genMemInit(Node* node);
  void nodeDisplay(Node* member, int indent);
  void genMemRead(FILE* fp);
  int genActivate(const std::string& subStepSuffix = "");
  void genUpdateRegister(FILE* fp);
  void genMemWrite(FILE* fp);
  void saveDiffRegs();
  void genResetAll();
  void genResetAllDense();
  void genResetDef(SuperNode* super, bool isUIntReset, bool buffered, int resetId, int indent, const std::string& nameSuffix = "", bool emitActivation = true);
  void genResetActivation(SuperNode* super, bool isUIntReset, int indent, int resetId);
  void genResetActivationDense(SuperNode* super, bool isUIntReset, int indent, int resetId);
  void genResetDecl(FILE* fp);
  int translateInst(InstInfo inst, int indent, std::string flagName, std::string activeBufferName, const std::string& accumFlagName = "", bool emitActivation = true);
  void genSuperEval(SuperNode* super, std::string flagName, std::string activeBufferName, int indent, bool emitActivation = true);
  void genMtTaskHelper(SuperNode* super, bool buffered, const std::string& activeSinkType);
  void genMtRepCutLiteTaskHelper(SuperNode* super, const std::vector<MtRepCutClone>& clones, const std::string& activeSinkType);
  void genMtTaskRunner(const MtRepCutSemanticPlan& semanticPlan);
  void genMtCoarseRegionRunner(const MtRepCutSemanticPlan& semanticPlan, const MtCoarseRegionPlan& coarsePlan);
  int genActivateSeqHelpers(bool buffered);
  int genActivateMtHelpers(int serialFastSubStepMax = -1, const std::string& serialFastSuffix = "");
  void genDenseExecutor(const MtDenseSchedule& denseSchedule, FILE* header);
  void removeNodesNoConnect(NodeStatus status);
  void reconnectSuper();
  void reconnectAll();
  void resetAnalysis();
  /* graphPartition */
  void mergeWhenNodes();
  void mergeResetAll();
  void mergeOut1();
  void mergeIn1();
  void mergeSublings();
  void splitArrayNode(Node* node);
  void checkNodeSplit(Node* node);
  void splitOptionalArray();
  void constantMemory();
  void orderAllNodes();
  void genDiffSig(FILE* fp, Node* node);
  void graphCoarsen();
  void graphInitPartition();
  void graphRefine();
  void resort(const char* seed2Tag = nullptr);
  void detectSortedSuperLoop();
  void when2mux();
 public:
  std::vector<Node*> allNodes;
  std::vector<Node*> input;
  std::vector<Node*> output;
  std::vector<Node*> regsrc;
  std::vector<Node*> sorted;
  std::vector<Node*> memory;
  std::vector<Node*> external;
  std::set<Node*> halfConstantArray;
  std::vector<Node*> specialNodes;
  /* used before toposort */
  std::vector<SuperNode*> supersrc;
  /* used after toposort */
  std::vector<SuperNode*> sortedSuper;
  std::vector<SuperNode*> allReset;
  std::vector<std::string> extDecl;
  std::string name;
  int nodeNum = 0;
  void addReg(Node* reg) {
    regsrc.push_back(reg);
  }
  void detectLoop();
  void topoSort();
  void instsGenerator();
  void mtInternNodeNames();
  void cppEmitter();
  void dumpMtScheduleJson();
  void dumpMtRepCutLiteReport();
  void dumpMtCoarseRegionReport();
  void dumpMtReadyBatchReport();
  void dumpMtDenseScheduleJson();
  void usedBits();
  void traversal();
    void splitArray();
  void removeDeadNodes();
  void aliasAnalysis();
  size_t countNodes();
  void removeEmptySuper();
  void removeNodes(NodeStatus status);
  void mergeRegister();
  void clockOptimize(std::map<std::string, Node*>& allSignals);
  void constantAnalysis();
  void constructRegs();
  void commonExpr();
  void splitNodes();
  void replicationOpt();
    void exprOpt();
  void patternDetect();
  void graphPartition();
    void inferAllWidth();
  void dump(std::string FileName);
    void generateStmtTree();
  void connectDep();
};

#endif
