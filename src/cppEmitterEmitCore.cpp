// cppEmitterEmitCore.cpp - split from cppEmitter.cpp (pure code motion):
// genHeaderStart, genInterfaceInput/Output, genDiffSig (per-mode), active-buffer/
// activation-delta/event-trace header defs, genNodeDef, activateNext,
// genNodeStepStart/End, translateInst, genSuperEval, genMtTaskHelper.
#include "cppEmitterImpl.h"

FILE* graph::genHeaderStart() {
  headerFilePath = globalConfig.OutputDir + "/" + name + ".h";
  headerTmpFilePath = globalConfig.MtStableOutput ? headerFilePath + ".tmp" : "";
  const std::string openPath = globalConfig.MtStableOutput ? headerTmpFilePath : headerFilePath;
  FILE* header = std::fopen(openPath.c_str(), "w");
  assert(header != NULL);
  setvbuf(header, NULL, _IOFBF, 4 * 1024 * 1024);
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
  if (mtUseActivationEventTraceCodegen()) includeLib(header, "cstddef", true);
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
    if (mtUseActivationEventTraceCodegen()) {
      emitBodyLock(2, "recordMtActivationEvent(-1, (uint32_t)%lu, (uint64_t)0x%lx, MT_ACTIVATION_EVENT_CONDITIONAL);\n", iter.first, ACTIVE_MASK(iter.second));
    }
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

void emitActiveBufferDef(FILE* header, int activeWords) {
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

void emitActivationDeltaDef(FILE* header, int activeWords) {
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

void emitActivationEventTraceDef(FILE* header) {
  fprintf(header,
          "enum MtActivationEventKind : uint8_t {\n"
          "  MT_ACTIVATION_EVENT_CONDITIONAL = 1,\n"
          "  MT_ACTIVATION_EVENT_UNCONDITIONAL = 2,\n"
          "  MT_ACTIVATION_EVENT_ACTIVATE_ALL = 3,\n"
          "  MT_ACTIVATION_EVENT_FRONTIER = 4,\n"
          "  MT_ACTIVATION_EVENT_CYCLE_END = 5\n"
          "};\n"
          "struct MtActivationEventTraceHeader {\n"
          "  uint8_t magic[8];\n"
          "  uint16_t version;\n"
          "  uint16_t headerSize;\n"
          "  uint16_t activeWidth;\n"
          "  uint16_t reserved0;\n"
          "  uint32_t taskCount;\n"
          "  uint32_t recordSize;\n"
          "  uint64_t traceStart;\n"
          "  uint64_t traceCount;\n"
          "  uint64_t reserved1;\n"
          "};\n"
          "struct MtActivationEventTraceRecord {\n"
          "  uint64_t cycle;\n"
          "  int32_t sourceCppId;\n"
          "  uint32_t activeWordBase;\n"
          "  uint64_t mask;\n"
          "  MtActivationEventKind kind;\n"
          "  uint8_t reserved[7];\n"
          "};\n"
          "static_assert(sizeof(MtActivationEventTraceHeader) == 48, \"activation-event trace header size\");\n"
          "static_assert(offsetof(MtActivationEventTraceHeader, version) == 8, \"activation-event trace header layout\");\n"
          "static_assert(offsetof(MtActivationEventTraceHeader, traceStart) == 24, \"activation-event trace header layout\");\n"
          "static_assert(offsetof(MtActivationEventTraceHeader, traceCount) == 32, \"activation-event trace header layout\");\n"
          "static_assert(offsetof(MtActivationEventTraceHeader, reserved1) == 40, \"activation-event trace header layout\");\n"
          "static_assert(sizeof(MtActivationEventTraceRecord) == 32, \"activation-event trace record size\");\n"
          "static_assert(offsetof(MtActivationEventTraceRecord, sourceCppId) == 8, \"activation-event trace record layout\");\n"
          "static_assert(offsetof(MtActivationEventTraceRecord, mask) == 16, \"activation-event trace record layout\");\n"
          "static_assert(offsetof(MtActivationEventTraceRecord, kind) == 24, \"activation-event trace record layout\");\n\n");
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
  if (const std::string* orig = mtShortNameOrigOf(node)) {
    fprintf(fp, "; // width = %d, lineno = %d, orig=%s\n", node->width, node->lineno, orig->c_str());
  } else {
    fprintf(fp, "; // width = %d, lineno = %d\n", node->width, node->lineno);
  }
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
                         std::string activeBufferName, int indent, bool emitActivation) {
  std::string nodeName = node->name;
  if (!emitActivation) {
    if (inStep) {
      if (node->isReset() && node->type == NODE_REG_SRC) emitBodyLock(indent, "%s = %s;\n", RESET_NAME(node).c_str(), newName(node).c_str());
      emitBodyLock(indent, "%s = %s;\n", node->name.c_str(), newName(node).c_str());
    }
    return;
  }
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
    if (activeBufferName.empty()) {
      if (mtUseActivationEventTraceCodegen() && !mtActivationEventTraceSuppressed) emitBodyLock(indent, "activateAll(%d);\n", mtActivationEventTraceSourceCppId);
      else emitBodyLock(indent, "activateAll();\n");
    } else {
      emitBodyLock(indent, "%s.activateAll();\n", activeBufferName.c_str());
      if (mtUseActivationEventTraceCodegen() && !mtActivationEventTraceSuppressed) {
        emitBodyLock(indent, "recordMtActivationEvent(%d, 0, UINT64_MAX, MT_ACTIVATION_EVENT_ACTIVATE_ALL);\n", mtActivationEventTraceSourceCppId);
      }
    }
    emitBodyLock(indent, "%s = -1;\n", flagName.c_str());
  } else {
    if (ACTIVE_MASK(curMask) != 0) {
      if (opt) emitBodyLock(indent, "%s |= -(uint%d_t)%s & 0x%lx; // %s\n", flagName.c_str(), ACTIVE_WIDTH, condName.c_str(), ACTIVE_MASK(curMask), ACTIVE_COMMENT(curMask).c_str());
      else emitBodyLock(indent, "%s |= 0x%lx; // %s\n", flagName.c_str(), ACTIVE_MASK(curMask), ACTIVE_COMMENT(curMask).c_str());
      if (mtUseActivationEventTraceCodegen() && !mtActivationEventTraceSuppressed) {
        if (opt) {
          emitBodyLock(indent, "recordMtActivationEvent(%d, (uint32_t)%d, (-(uint64_t)%s & (uint64_t)0x%lx), MT_ACTIVATION_EVENT_CONDITIONAL);\n",
                       mtActivationEventTraceSourceCppId, node->super->cppId / ACTIVE_WIDTH, condName.c_str(), ACTIVE_MASK(curMask));
        } else {
          emitBodyLock(indent, "recordMtActivationEvent(%d, (uint32_t)%d, (uint64_t)0x%lx, MT_ACTIVATION_EVENT_CONDITIONAL);\n",
                       mtActivationEventTraceSourceCppId, node->super->cppId / ACTIVE_WIDTH, ACTIVE_MASK(curMask));
        }
      }
    }
    for (auto iter : bitMapInfo) {
      auto str = opt ? updateActiveStr(iter.first, ACTIVE_MASK(iter.second), condName, ACTIVE_UNIQUE(iter.second), activeBufferName)
                     : updateActiveStr(iter.first, ACTIVE_MASK(iter.second), activeBufferName);
      emitBodyLock(indent, "%s // %s\n", str.c_str(), ACTIVE_COMMENT(iter.second).c_str());
      if (mtUseActivationEventTraceCodegen() && !mtActivationEventTraceSuppressed) {
        if (!opt) {
          emitBodyLock(indent, "recordMtActivationEvent(%d, (uint32_t)%lu, (uint64_t)0x%lx, MT_ACTIVATION_EVENT_CONDITIONAL);\n",
                       mtActivationEventTraceSourceCppId, iter.first, ACTIVE_MASK(iter.second));
        } else if (ACTIVE_UNIQUE(iter.second) >= 0) {
          emitBodyLock(indent, "recordMtActivationEvent(%d, (uint32_t)%lu, ((uint64_t)%s << %d), MT_ACTIVATION_EVENT_CONDITIONAL);\n",
                       mtActivationEventTraceSourceCppId, iter.first, condName.c_str(), ACTIVE_UNIQUE(iter.second));
        } else {
          emitBodyLock(indent, "recordMtActivationEvent(%d, (uint32_t)%lu, (-(uint64_t)%s & (uint64_t)0x%lx), MT_ACTIVATION_EVENT_CONDITIONAL);\n",
                       mtActivationEventTraceSourceCppId, iter.first, condName.c_str(), ACTIVE_MASK(iter.second));
        }
      }
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
                               std::string activeBufferName, int indent, bool emitActivation) {
  if (!emitActivation) return;
  std::map<uint64_t, ActiveType> bitMapInfo;
  auto curMask = activeSet2bitMap(activateId, bitMapInfo, node->super->cppId);
  if (ACTIVE_MASK(curMask) != 0) {
    emitBodyLock(indent, "%s |= 0x%lx; // %s\n", flagName.c_str(), ACTIVE_MASK(curMask), ACTIVE_COMMENT(curMask).c_str());
    if (mtUseActivationEventTraceCodegen() && !mtActivationEventTraceSuppressed) {
      emitBodyLock(indent, "recordMtActivationEvent(%d, (uint32_t)%d, (uint64_t)0x%lx, MT_ACTIVATION_EVENT_UNCONDITIONAL);\n",
                   mtActivationEventTraceSourceCppId, node->super->cppId / ACTIVE_WIDTH, ACTIVE_MASK(curMask));
    }
  }
  for (auto iter : bitMapInfo) {
    emitBodyLock(indent, "%s // %s\n", updateActiveStr(iter.first, ACTIVE_MASK(iter.second), activeBufferName).c_str(), ACTIVE_COMMENT(iter.second).c_str());
    if (mtUseActivationEventTraceCodegen() && !mtActivationEventTraceSuppressed) {
      emitBodyLock(indent, "recordMtActivationEvent(%d, (uint32_t)%lu, (uint64_t)0x%lx, MT_ACTIVATION_EVENT_UNCONDITIONAL);\n",
                   mtActivationEventTraceSourceCppId, iter.first, ACTIVE_MASK(iter.second));
    }
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
int graph::genNodeStepStart(SuperNode* node, uint64_t mask, int idx, std::string flagName, int indent, bool skipAdmissionGuard) {
  nodeNum ++;
  if (!skipAdmissionGuard && !isAlwaysActive(node->cppId)) {
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

int graph::genNodeStepEnd(SuperNode* node, int indent, bool skipAdmissionGuard) {
#ifdef PERF
  if (node->superType != SUPER_EXTMOD) {
    emitBodyLock(indent, "validActive[%d] += isActivateValid;\n", node->cppId);
  }
#endif

  if(!skipAdmissionGuard && !isAlwaysActive(node->cppId)) {
    emitBodyLock(-- indent, "}\n");
  }
  return indent;
}

bool Node::isLocal() { // TODO: isArray is OK
  return status == VALID_NODE && type == NODE_OTHERS && !anyNextActive() && !isArray() && !isReset();
}


int graph::translateInst(InstInfo inst, int indent, std::string flagName, std::string activeBufferName, bool emitActivation) {
  switch (inst.infoType) {
    case SUPER_INFO_IF:
      emitBodyLock(indent ++, "%s\n", inst.inst.c_str());
      break;
    case SUPER_INFO_ELSE:
      emitBodyLock(indent - 1,  "%s\n", inst.inst.c_str());
      break;
    case SUPER_INFO_DEDENT:
      emitBodyLock(--indent, "%s\n", inst.inst.c_str());
      break;
    case SUPER_INFO_STR:
      emitBodyLock(indent, "%s\n", inst.inst.c_str());
      break;
    case SUPER_INFO_ASSIGN_BEG:
      if (inst.node->isLocal() || inst.node->isArray() || inst.node->type == NODE_WRITER) break;
      // Report-only histogram (GSIM_MT_DENSE_OLDVALUE_HISTOGRAM=1): does this snapshot's
      // change-detection feed any activation consumer? nextActiveId empty = candidate for
      // per-node dead-code elimination (the audit's largest bookkeeping class).
      if (mtOldValueHistogramEnabled()) {
        if (inst.node->nextActiveId.empty()) mtOldSnapNoConsumers.fetch_add(1, std::memory_order_relaxed);
        else mtOldSnapWithConsumers.fetch_add(1, std::memory_order_relaxed);
      }
      emitBodyLock(indent, "%s %s = %s;\n", widthUType(inst.node->width).c_str(), oldName(inst.node).c_str(), inst.node->name.c_str());
      break;
    case SUPER_INFO_ASSIGN_END:
      if (inst.node->isLocal() || !inst.node->needActivate()) break;
      if (inst.node->isArray() || inst.node->type == NODE_WRITER) activateUncondNext(inst.node, inst.node->nextActiveId, false, flagName, activeBufferName, indent, emitActivation);
      else activateNext(inst.node, inst.node->nextActiveId, oldName(inst.node), false, flagName, activeBufferName, indent, emitActivation);
      break;
    default:
      break;
  }
  return indent;
}

void graph::genSuperEval(SuperNode* super, std::string flagName, std::string activeBufferName, int indent, bool emitActivation) { // current indent = 2
  int savedTraceSourceCppId = mtActivationEventTraceSourceCppId;
  if (emitActivation && !mtActivationEventTraceSuppressed) mtActivationEventTraceSourceCppId = super->cppId;
  if (super->superType == SUPER_EXTMOD) { // TODO: normalize
    auto emitExtAsyncReset = [&](Node* extOut) {
      if (!extOut->isAsyncReset()) return;
      auto resetId = super2ResetId.find(extOut);
      Assert(resetId != super2ResetId.end() && resetId->second.second >= 0, "missing async reset id for %s", extOut->name.c_str());
      emitBodyLock(indent, "subReset%d();\n", resetId->second.second);
    };
    for (size_t i = 1; i < super->member.size(); i ++) {
      emitExtAsyncReset(super->member[i]);
    }
    /* save old EXT_OUT*/
    for (size_t i = 1; i < super->member.size(); i ++) {
      if (!super->member[i]->needActivate()) continue;
      Node* extOut = super->member[i];
      emitBodyLock(indent, "%s %s = %s;\n", widthUType(extOut->width).c_str(), oldName(extOut).c_str(), extOut->name.c_str());
    }
    for (InstInfo inst : super->insts) {
      indent = translateInst(inst, indent, flagName, activeBufferName, emitActivation);
    }
    for (size_t i = 1; i < super->member.size(); i ++) {
      emitExtAsyncReset(super->member[i]);
    }
    for (size_t i = 1; i < super->member.size(); i ++) {
      if (!super->member[i]->needActivate()) continue;
      if (super->member[i]->isArray()) activateUncondNext(super->member[i], super->member[i]->nextActiveId, false, flagName, activeBufferName, indent, emitActivation);
      else activateNext(super->member[i], super->member[i]->nextActiveId, oldName(super->member[i]), false, flagName, activeBufferName, indent, emitActivation);
    }
  } else {
    if (super->superType == SUPER_ASYNC_RESET) {
      int resetId = super2ResetIdLookup(super->resetNode).second;
      if (!emitActivation && activeBufferName.empty()) {
        int denseResetId = -1;
        auto denseResetIt = super2DenseResetId.find(super->resetNode);
        if (denseResetIt != super2DenseResetId.end()) denseResetId = denseResetIt->second.second;
        Assert(denseResetId >= 0, "missing dense async reset id for %s", super->resetNode->name.c_str());
        emitBodyLock(indent, "subResetDense%d();\n", denseResetId);
      } else if (activeBufferName.empty()) {
        if (mtUseActivationEventTraceCodegen()) emitBodyLock(indent, "subReset%d(%d);\n", resetId, mtActivationEventTraceSourceCppId);
        else emitBodyLock(indent, "subReset%d();\n", resetId);
      } else {
        if (mtUseActivationEventTraceCodegen()) emitBodyLock(indent, "subReset%d(%s, %d);\n", resetId, activeBufferName.c_str(), mtActivationEventTraceSourceCppId);
        else emitBodyLock(indent, "subReset%d(%s);\n", resetId, activeBufferName.c_str());
      }
    }
    /* local nodes definition */
    for (Node* n : super->member) {
      if (n->isLocal()) {
        emitBodyLock(indent, "%s %s;\n", widthUType(n->width).c_str(), n->name.c_str());
      }
    }
    if (mtUseDenseElideObservability() && mtDenseObservabilitySpansBalanced(super->insts)) {
      const std::set<Node*>& droppable = mtDenseObservabilityDroppableSet();
      if (!droppable.empty()) {
        std::vector<Node*> frameStack;
        for (InstInfo inst : super->insts) {
          if (inst.infoType == SUPER_INFO_ASSIGN_BEG) {
            frameStack.push_back(inst.node);
            if (droppable.find(inst.node) == droppable.end()) indent = translateInst(inst, indent, flagName, activeBufferName, emitActivation);
            continue;
          }
          if (inst.infoType == SUPER_INFO_ASSIGN_END) {
            Node* endNode = inst.node;
            if (!frameStack.empty()) frameStack.pop_back();
            if (droppable.find(endNode) == droppable.end()) indent = translateInst(inst, indent, flagName, activeBufferName, emitActivation);
            continue;
          }
          if (!frameStack.empty() && droppable.find(frameStack.back()) != droppable.end()) continue;
          indent = translateInst(inst, indent, flagName, activeBufferName, emitActivation);
        }
      } else {
        for (InstInfo inst : super->insts) {
          indent = translateInst(inst, indent, flagName, activeBufferName, emitActivation);
        }
      }
    } else {
      for (InstInfo inst : super->insts) {
        indent = translateInst(inst, indent, flagName, activeBufferName, emitActivation);
      }
    }
    if (super->superType == SUPER_ASYNC_RESET) {
      int resetId = super2ResetIdLookup(super->resetNode).second;
      if (!emitActivation && activeBufferName.empty()) {
        int denseResetId = -1;
        auto denseResetIt = super2DenseResetId.find(super->resetNode);
        if (denseResetIt != super2DenseResetId.end()) denseResetId = denseResetIt->second.second;
        Assert(denseResetId >= 0, "missing dense async reset id for %s", super->resetNode->name.c_str());
        emitBodyLock(indent, "subResetDense%d();\n", denseResetId);
      } else if (activeBufferName.empty()) {
        if (mtUseActivationEventTraceCodegen()) emitBodyLock(indent, "subReset%d(%d);\n", resetId, mtActivationEventTraceSourceCppId);
        else emitBodyLock(indent, "subReset%d();\n", resetId);
      } else {
        if (mtUseActivationEventTraceCodegen()) emitBodyLock(indent, "subReset%d(%s, %d);\n", resetId, activeBufferName.c_str(), mtActivationEventTraceSourceCppId);
        else emitBodyLock(indent, "subReset%d(%s);\n", resetId, activeBufferName.c_str());
      }
    }
    emitBodyLock(indent, "#ifdef ENABLE_LOG\n");
    emitBodyLock(indent ++, "if (cycles >= LOG_START && cycles <= LOG_END) {\n");
    for (Node* n : super->member) nodeDisplay(n, indent);
    emitBodyLock(-- indent, "}\n");
    emitBodyLock(indent, "#endif\n");
  }
  mtActivationEventTraceSourceCppId = savedTraceSourceCppId;
}



void graph::genMtTaskHelper(SuperNode* super, bool buffered, const std::string& activeSinkType) {
  int savedTraceSourceCppId = mtActivationEventTraceSourceCppId;
  mtActivationEventTraceSourceCppId = super->cppId;
  if (buffered) {
    emitFuncDecl(0, "void S%s::mtTask%d(uint%d_t &flag, %s &nextActive) {\n", name.c_str(), super->cppId, ACTIVE_WIDTH, activeSinkType.c_str());
    genSuperEval(super, "flag", "nextActive", 1, true);
  } else {
    emitFuncDecl(0, "void S%s::mtTask%d(uint%d_t &flag) {\n", name.c_str(), super->cppId, ACTIVE_WIDTH);
    genSuperEval(super, "flag", "", 1, true);
  }
  emitBodyLock(0, "}\n");
  mtActivationEventTraceSourceCppId = savedTraceSourceCppId;
}
