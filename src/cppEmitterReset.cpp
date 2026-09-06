// cppEmitterReset.cpp - split from cppEmitter.cpp (pure code motion):
// GSIM_EMIT_RESET_CHUNK chunking (mtResetChunkSize + mtResetChunkDecls: the decl
// list is a function-local static that travels WITH this accessor), super2ResetIdLookup,
// genResetDef, genResetActivation(Dense), genResetAll(Dense). This TU owns the write
// side of super2ResetId/super2DenseResetId/resetFuncNum (header-inline state).
#include "cppEmitterImpl.h"

// Read-only view of super2ResetId with std::map::operator[] value semantics:
// returns the value-initialized pair a first operator[] access would have
// inserted, without mutating the shared map (parallel emission units call this
// concurrently; genResetAll has already populated every live key by then).
const std::pair<int, int>& super2ResetIdLookup(Node* resetNode) {
  static const std::pair<int, int> kDefault = {0, 0};
  auto iter = super2ResetId.find(resetNode);
  return iter == super2ResetId.end() ? kDefault : iter->second;
}


// GSIM_EMIT_RESET_CHUNK=<n>: subResetN bodies carry millions of scalar
// assignments in ONE function; the C++ frontend is superlinear on that shape
// (measured 1366s for one 5MB body vs 2.8s stubbed). Splitting the body into
// chain-called chunks of <n> statements (split only at top-level depth, so the
// IF/ELSE nesting is never cut) restores linear frontend cost. Default off.
static long mtResetChunkSize() {
  const char* e = std::getenv("GSIM_EMIT_RESET_CHUNK");
  // Default 4096: chunking is semantics-preserving (chain-called helpers under
  // identical re-opened guards) and removes a ~500x clang frontend blowup on
  // giant reset bodies. Set =0 to emit the legacy monolithic bodies.
  if (e == nullptr || e[0] == '\0') return 4096;
  long v = std::atol(e);
  if (v == 0) return 0;
  return v >= 256 ? v : 4096;
}
std::vector<std::string>& mtResetChunkDecls() {
  static std::vector<std::string> decls;
  return decls;
}
void graph::genResetDef(SuperNode* super, bool isUIntReset, bool buffered, int resetId, int indent, const std::string& nameSuffix, bool emitActivation) {
  std::string activeSinkType = (globalConfig.MtHelperMode == "mt" ||
                                globalConfig.MtHelperMode == "mt-level-dispatch")
                                 ? "ActivationDelta" : "ActiveBuffer";
  std::string resetFuncName = format("subReset%s%d", nameSuffix.c_str(), resetId);
  bool traceSourceParam = mtUseActivationEventTraceCodegen() && emitActivation;
  if (buffered) {
    if (traceSourceParam) emitBodyLock(indent ++, "void S%s::%s(%s &nextActive, int32_t traceSourceCppId){ // %s reset\n", name.c_str(), resetFuncName.c_str(), activeSinkType.c_str(), isUIntReset ? "uint" : "async");
    else emitBodyLock(indent ++, "void S%s::%s(%s &nextActive){ // %s reset\n", name.c_str(), resetFuncName.c_str(), activeSinkType.c_str(), isUIntReset ? "uint" : "async");
  } else {
    if (traceSourceParam) emitBodyLock(indent ++, "void S%s::%s(int32_t traceSourceCppId){ // %s reset\n", name.c_str(), resetFuncName.c_str(), isUIntReset ? "uint" : "async");
    else emitBodyLock(indent ++, "void S%s::%s(){ // %s reset\n", name.c_str(), resetFuncName.c_str(), isUIntReset ? "uint" : "async");
  }
  std::string resetName = super->resetNode->type == NODE_REG_SRC ? RESET_NAME(super->resetNode).c_str() : super->resetNode->name.c_str();
  if (!emitActivation) {
    emitBodyLock(indent ++, "if(unlikely(%s)) {\n", resetName.c_str());
  }
  if (emitActivation) {
    emitBodyLock(indent ++, "if(unlikely(%s)) {\n", resetName.c_str());
    std::set<int> allNext;
    for (size_t i = 0; i < super->member.size(); i ++) {
      Node* node = super->member[i];
      if (node->type == NODE_REG_RESET) node = node->getResetSrc();
      for (Node* next : node->next) {
        if (next->super->cppId >= 0) allNext.insert(next->super->cppId);
      }
    }

    if (allNext.size() > 100) {
      if (buffered) {
        emitBodyLock(indent, "nextActive.activateAll();\n");
        if (mtUseActivationEventTraceCodegen()) {
          emitBodyLock(indent, "recordMtActivationEvent(traceSourceCppId, 0, UINT64_MAX, MT_ACTIVATION_EVENT_ACTIVATE_ALL);\n");
        }
      } else if (mtUseActivationEventTraceCodegen()) {
        emitBodyLock(indent, "activateAll(traceSourceCppId);\n");
      } else {
        emitBodyLock(indent, "activateAll();\n");
      }
    }
    else {
      std::map<uint64_t, ActiveType> bitMapInfo;
      activeSet2bitMap(allNext, bitMapInfo, -1);
      for (auto iter : bitMapInfo) {
        emitBodyLock(indent, "%s // %s\n", updateActiveStr(iter.first, ACTIVE_MASK(iter.second), buffered ? "nextActive" : "").c_str(), ACTIVE_COMMENT(iter.second).c_str());
        if (mtUseActivationEventTraceCodegen()) {
          emitBodyLock(indent, "recordMtActivationEvent(traceSourceCppId, (uint32_t)%lu, (uint64_t)0x%lx, MT_ACTIVATION_EVENT_CONDITIONAL);\n",
                       iter.first, ACTIVE_MASK(iter.second));
        }
      }
    }
    emitBodyLock(-- indent, "}\n");
  }
  // Chunked emission (GSIM_EMIT_RESET_CHUNK=<n>): the reset body's statement
  // stream is split into chain-called member helpers at top-level depth (the
  // IF/ELSE nesting is never cut). The parent keeps the reset guard and
  // activation logic; each chunk ends with a call to the next, so a chunk
  // executes under exactly the same conditions as the unsplit body. The C++
  // frontend is superlinear on million-statement functions (measured 1366s
  // for one 5MB subReset body, 2.8s stubbed); chunking restores linear cost.
  const long chunkSize = mtResetChunkSize();
  // Call/parameter matrix must match the parent signature exactly
  // (cppEmitterReset.cpp genResetDef): buffered parents have nextActive of
  // activeSinkType (ActivationDelta or ActiveBuffer by helper mode); trace
  // parents add traceSourceCppId. The old code hardcoded nextActive under
  // trace, generating an undeclared-identifier call in unbuffered+trace.
  const char* chunkCallArgs = traceSourceParam
    ? (buffered ? "(nextActive, traceSourceCppId)" : "(traceSourceCppId)")
    : (buffered ? "(nextActive)" : "()");
  const std::string chunkParamList = traceSourceParam
    ? ("(" + (buffered ? activeSinkType + " &nextActive, " : "") + "int32_t traceSourceCppId)")
    : (buffered ? "(" + activeSinkType + " &nextActive)" : "()");
  // The statement stream typically nests entirely inside `if (reset) { ... }`
  // (depth 1 throughout), so a depth-0-only split would never fire. Instead we
  // track the open IF stack; on a split we close the open braces, chain into
  // the next chunk function, and re-open the identical IF conditions inside
  // it. Reset conditions are pure member loads, so re-evaluating them is
  // side-effect-free and yields the same guard context for every statement.
  std::vector<std::string> openIfs;
  long emittedInChunk = 0;
  int chunkIdx = 0;
  for (InstInfo inst : super->insts) {
    if (chunkSize > 0 && emittedInChunk >= chunkSize &&
        inst.infoType != SUPER_INFO_IF && inst.infoType != SUPER_INFO_ELSE &&
        inst.infoType != SUPER_INFO_DEDENT) {
      // close the open if-chain, chain into the next chunk, reopen it there
      for (size_t d = 0; d < openIfs.size(); d ++) emitBodyLock(-- indent, "}\n");
      emitBodyLock(indent, "%s_c%d%s;\n", resetFuncName.c_str(), chunkIdx + 1, chunkCallArgs);
      if (chunkIdx == 0) {
        if (!emitActivation) emitBodyLock(-- indent, "}\n");  // reset guard
        emitBodyLock(-- indent, "}\n");                        // parent function
      } else {
        emitBodyLock(indent, "}\n");
      }
      chunkIdx ++;
      emitFuncDecl(indent, "void S%s::%s_c%d%s {\n", name.c_str(), resetFuncName.c_str(), chunkIdx, chunkParamList.c_str());
      mtResetChunkDecls().push_back(format("  void %s_c%d%s;\n", resetFuncName.c_str(), chunkIdx, chunkParamList.c_str()));
      for (const std::string& ifInst : openIfs) emitBodyLock(indent ++, "%s\n", ifInst.c_str());
      emittedInChunk = 0;
    }
    switch (inst.infoType) {
      case SUPER_INFO_IF:
        emitBodyLock(indent ++, "%s\n", inst.inst.c_str());
        openIfs.push_back(inst.inst);
        emittedInChunk ++;
        break;
      case SUPER_INFO_DEDENT:
        emitBodyLock(--indent, "%s\n", inst.inst.c_str());
        if (!openIfs.empty()) openIfs.pop_back();
        break;
      case SUPER_INFO_ELSE:
      case SUPER_INFO_STR:
        emitBodyLock(indent, "%s\n", inst.inst.c_str());
        emittedInChunk ++;
        break;
      default:
        break;
    }
  }
  if (chunkIdx > 0) {
    for (size_t d = 0; d < openIfs.size(); d ++) emitBodyLock(-- indent, "}\n");
    emitBodyLock(indent, "}\n");  // close the final chunk
  } else {
    if (!emitActivation) emitBodyLock(-- indent, "}\n");
    emitBodyLock(-- indent, "}\n");
  }
}

void graph::genResetActivation(SuperNode* super, bool isUIntReset, int indent, int resetId) {
  if (mtUseActivationEventTraceCodegen()) emitBodyLock(indent, "subReset%d(-1);\n", resetId);
  else emitBodyLock(indent, "subReset%d();\n", resetId);
}

void graph::genResetActivationDense(SuperNode* super, bool isUIntReset, int indent, int resetId) {
  (void)super;
  (void)isUIntReset;
  emitBodyLock(indent, "subResetDense%d();\n", resetId);
}

void graph::genResetAll() {
  std::vector<SuperNode*> resetSuper;
  for (SuperNode* super : allReset) {
    if (super->resetNode->status == CONSTANT_NODE) {
      Assert(mpz_sgn(super->resetNode->computeInfo->consVal) == 0, "reset %s is always true", super->resetNode->name.c_str());
      continue;
    }
    super2ResetId.emplace(super->resetNode, std::make_pair(-1, -1));
    int resetId = resetFuncNum ++;
    bool isUIntReset = super->superType == SUPER_UINT_RESET;
    if (isUIntReset) super2ResetId[super->resetNode].first = resetId;
    else super2ResetId[super->resetNode].second = resetId;
    genResetDef(super, isUIntReset, false, resetId, 0);
    if (globalConfig.MtHelperMode == "buffered-seq" ||
        globalConfig.MtHelperMode == "mt" ||
        globalConfig.MtHelperMode == "mt-level-dispatch") {
      genResetDef(super, isUIntReset, true, resetId, 0);
    }
    resetSuper.push_back(super);
  }

  emitFuncDecl(0, "void S%s::resetAll(){\n", name.c_str());
  for (size_t i = 0; i < resetSuper.size(); i ++) {
    if (resetSuper[i]->superType == SUPER_ASYNC_RESET) continue;
    genResetActivation(resetSuper[i], true, 1, i);
  }
  emitBodyLock(0, "}\n");
}

void graph::genResetAllDense() {
  std::vector<std::tuple<SuperNode*, bool, int>> resetSuper;
  super2DenseResetId.clear();
  int denseResetFuncNum = 0;
  for (SuperNode* super : allReset) {
    if (super->resetNode->status == CONSTANT_NODE) continue;
    bool isUIntReset = super->superType == SUPER_UINT_RESET;
    int resetId = denseResetFuncNum ++;
    if (super2DenseResetId.find(super->resetNode) == super2DenseResetId.end()) {
      super2DenseResetId[super->resetNode] = std::make_pair(-1, -1);
    }
    if (isUIntReset) super2DenseResetId[super->resetNode].first = resetId;
    else super2DenseResetId[super->resetNode].second = resetId;
    genResetDef(super, isUIntReset, false, resetId, 0, "Dense", false);
    resetSuper.push_back(std::make_tuple(super, isUIntReset, resetId));
  }

  emitFuncDecl(0, "void S%s::resetAllDense(){\n", name.c_str());
  emitBodyLock(1, "memset(activeFlags, 0, sizeof(activeFlags));\n");
  for (const auto& entry : resetSuper) {
    SuperNode* super;
    bool isUIntReset;
    int resetId;
    std::tie(super, isUIntReset, resetId) = entry;
    if (super->superType == SUPER_ASYNC_RESET) continue;
    genResetActivationDense(super, isUIntReset, 1, resetId);
  }
  emitBodyLock(0, "}\n");
}

