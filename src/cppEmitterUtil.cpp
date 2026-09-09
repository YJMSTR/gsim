// cppEmitterUtil.cpp - split from cppEmitter.cpp (pure code motion):
// knob readers (mtUse*/mtDense*), json/node-type helpers, observability classification,
// name interning (mtInternNodeNames), bitmask/active-word helpers, phase-timer helpers.
#include "cppEmitterImpl.h"

bool mtOldValueHistogramEnabled() {
  const char* env = std::getenv("GSIM_MT_DENSE_OLDVALUE_HISTOGRAM");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

bool emitPhaseTimingEnabled() {
  const char* env = std::getenv("GSIM_EMIT_PHASE_TIMING");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

void emitPhaseAccumReport(const EmitPhaseAccum& accum) {
  if (!emitPhaseTimingEnabled()) return;
  fprintf(stderr, "[emit-phase] %s = %.1f ms (%llu calls)\n", accum.phaseName,
          (double)accum.ns / 1e6, (unsigned long long)accum.calls);
}

bool isAlwaysActive(int cppId) {
  return alwaysActive.find(cppId) != alwaysActive.end();
}

bool hasCppId(const std::set<SuperNode*>& supers, int cppId) {
  for (SuperNode* super : supers) {
    if (super && super->cppId == cppId) return true;
  }
  return false;
}

const char* nodeTypeName(NodeType type) {
  switch (type) {
    case NODE_INVALID: return "NODE_INVALID";
    case NODE_REG_SRC: return "NODE_REG_SRC";
    case NODE_REG_DST: return "NODE_REG_DST";
    case NODE_SPECIAL: return "NODE_SPECIAL";
    case NODE_INP: return "NODE_INP";
    case NODE_OUT: return "NODE_OUT";
    case NODE_MEMORY: return "NODE_MEMORY";
    case NODE_READER: return "NODE_READER";
    case NODE_WRITER: return "NODE_WRITER";
    case NODE_READWRITER: return "NODE_READWRITER";
    case NODE_INFER: return "NODE_INFER";
    case NODE_OTHERS: return "NODE_OTHERS";
    case NODE_REG_RESET: return "NODE_REG_RESET";
    case NODE_EXT_IN: return "NODE_EXT_IN";
    case NODE_EXT_OUT: return "NODE_EXT_OUT";
    case NODE_EXT: return "NODE_EXT";
  }
  return "NODE_UNKNOWN";
}

bool isKnownNodeType(NodeType type) {
  switch (type) {
    case NODE_INVALID:
    case NODE_REG_SRC:
    case NODE_REG_DST:
    case NODE_SPECIAL:
    case NODE_INP:
    case NODE_OUT:
    case NODE_MEMORY:
    case NODE_READER:
    case NODE_WRITER:
    case NODE_READWRITER:
    case NODE_INFER:
    case NODE_OTHERS:
    case NODE_REG_RESET:
    case NODE_EXT_IN:
    case NODE_EXT_OUT:
    case NODE_EXT:
      return true;
  }
  return false;
}

const char* superTypeName(SuperType type) {
  switch (type) {
    case SUPER_VALID: return "SUPER_VALID";
    case SUPER_EXTMOD: return "SUPER_EXTMOD";
    case SUPER_ASYNC_RESET: return "SUPER_ASYNC_RESET";
    case SUPER_UINT_RESET: return "SUPER_UINT_RESET";
    case SUPER_UPDATE_REG: return "SUPER_UPDATE_REG";
  }
  return "SUPER_UNKNOWN";
}
static const char* superInfoName(SuperInfo info) {
  switch (info) {
    case SUPER_INFO_IF: return "if";
    case SUPER_INFO_ELSE: return "else";
    case SUPER_INFO_DEDENT: return "dedent";
    case SUPER_INFO_STR: return "str";
    case SUPER_INFO_ASSIGN_BEG: return "assign_beg";
    case SUPER_INFO_ASSIGN_END: return "assign_end";
  }
  return "unknown";
}

std::string jsonEscape(const std::string& str) {
  std::string ret;
  for (char ch : str) {
    switch (ch) {
      case '\\': ret += "\\\\"; break;
      case '"': ret += "\\\""; break;
      case '\b': ret += "\\b"; break;
      case '\f': ret += "\\f"; break;
      case '\n': ret += "\\n"; break;
      case '\r': ret += "\\r"; break;
      case '\t': ret += "\\t"; break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20) ret += format("\\u%04x", ch);
        else ret += ch;
        break;
    }
  }
  return ret;
}


void dumpJsonIntArray(FILE* fp, const std::set<int>& values) {
  fprintf(fp, "[");
  bool first = true;
  for (int value : values) {
    if (!first) fprintf(fp, ", ");
    first = false;
    fprintf(fp, "%d", value);
  }
  fprintf(fp, "]");
}

void dumpJsonIntArray(FILE* fp, const std::vector<int>& values) {
  fprintf(fp, "[");
  bool first = true;
  for (int value : values) {
    if (!first) fprintf(fp, ", ");
    first = false;
    fprintf(fp, "%d", value);
  }
  fprintf(fp, "]");
}

void dumpJsonStringArray(FILE* fp, const std::set<std::string>& values) {
  fprintf(fp, "[");
  bool first = true;
  for (const std::string& value : values) {
    if (!first) fprintf(fp, ", ");
    first = false;
    fprintf(fp, "\"%s\"", jsonEscape(value).c_str());
  }
  fprintf(fp, "]");
}

void dumpJsonStringArray(FILE* fp, const std::vector<std::string>& values) {
  fprintf(fp, "[");
  bool first = true;
  for (const std::string& value : values) {
    if (!first) fprintf(fp, ", ");
    first = false;
    fprintf(fp, "\"%s\"", jsonEscape(value).c_str());
  }
  fprintf(fp, "]");
}

static bool mtUseDenseActivationOrigins() {
  const char* env = std::getenv("GSIM_MT_DENSE_ACTIVATION_ORIGINS");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

bool mtUseDenseMemberMetadata() {
  const char* env = std::getenv("GSIM_MT_DENSE_MEMBER_METADATA");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// default-off. Verilator runs display/print MTasks distributed across
// workers (VL_PRINTF_MT posts to a message queue); gsim instead pins every
// task containing printf/assert/exit ops on worker0, which serializes ~70% of
// worker0's pinned work (52% of the T16 wall at C50000). This knob excludes
// "special" from the DENSE worker0-only classification only; sparse/coarse
// fallback predicates keep the conservative reason set. Print side effects are
// serialized by a gprintf mutex; log printf is compiled out in benchmark
// builds and assert/exit races are benign (first abort wins).
bool mtUseDenseUnpinSpecial() {
  const char* env = std::getenv("GSIM_MT_DENSE_UNPIN_SPECIAL");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// default-off observability-cone elimination (user-approved scope
// ). Perf-counter/debug-observability nodes whose ENTIRE data and
// activation fanout is itself observability or print-only are droppable: their
// assignment spans are not emitted. [PERF] printout values become stale/zero
// (accepted); NEMU architectural endpoints must stay exact, so classification
// is fail-closed: any architectural consumer (data or activation) keeps the
// node alive, and supers with unbalanced assignment-marker spans emit everything.
bool mtUseDenseElideObservability() {
  const char* env = std::getenv("GSIM_MT_DENSE_ELIDE_OBSERVABILITY");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

static bool mtDenseObservabilityNameClassified(const std::string& name) {
  if (name.rfind("logEndpoint__DOT__", 0) == 0) return true;
  if (name.find("perfCtrl") != std::string::npos || name.find("perfEvents") != std::string::npos ||
      name.find("PerfCtf") != std::string::npos || name.find("perf_") != std::string::npos) return true;
  if (name.find("debugModule") != std::string::npos || name.find("dtm") != std::string::npos ||
      name.find("DTM") != std::string::npos || name.find("jtag") != std::string::npos ||
      name.find("JTAG") != std::string::npos) return false;
  if (name.find("debug_") != std::string::npos || name.find("Debug") != std::string::npos) return true;
  return false;
}

const std::set<Node*>& mtDenseObservabilityDroppableSet() {
  // C++11 magic-static: the initializer runs exactly once under the compiler's
  // guard, fixing the hand-rolled computed-flag race (first call may happen
  // inside an emitUnitsParallel worker - the old pattern could publish a
  // half-built set). The set is built on first call and read-only after.
  static const std::set<Node*> droppable = [] {
  std::set<Node*> built;
  if (!mtUseDenseElideObservability()) return built;
  std::vector<Node*> candidates;
  for (int cppId = 0; cppId < superId; cppId ++) {
    auto superIter = cppId2Super.find(cppId);
    if (superIter == cppId2Super.end() || !superIter->second) continue;
    for (Node* node : superIter->second->member) {
      if (!node || node->status != VALID_NODE || node->type != NODE_OTHERS) continue;
      if (node->isArray() || node->parent || node->inAggr || node->isClock || node->isReset() || node->isExt()) continue;
      if (!node->member.empty()) continue;
      if (!mtDenseObservabilityNameClassified(node->name)) continue;
      candidates.push_back(node);
    }
  }
  auto fullyDroppableSuper = [&](SuperNode* super) {
    if (!super) return false;
    for (Node* member : super->member) {
      if (!member) continue;
      if (member->type == NODE_SPECIAL) continue;
      if (built.find(member) == built.end()) return false;
    }
    return true;
  };
  bool changed = true;
  while (changed) {
    changed = false;
    for (Node* node : candidates) {
      if (built.find(node) != built.end()) continue;
      bool alive = false;
      auto consumerAlive = [&](Node* consumer) {
        return consumer && built.find(consumer) == built.end();
      };
      for (Node* consumer : node->next) if (consumerAlive(consumer)) { alive = true; break; }
      if (!alive) for (Node* consumer : node->depNext) if (consumerAlive(consumer)) { alive = true; break; }
      if (!alive) {
        for (int target : node->nextActiveId) {
          auto targetIter = cppId2Super.find(target);
          if (targetIter == cppId2Super.end() || !fullyDroppableSuper(targetIter->second)) { alive = true; break; }
        }
      }
      if (!alive) {
        for (int target : node->nextNeedActivate) {
          auto targetIter = cppId2Super.find(target);
          if (targetIter == cppId2Super.end() || !fullyDroppableSuper(targetIter->second)) { alive = true; break; }
        }
      }
      if (!alive) {
        built.insert(node);
        changed = true;
      }
    }
  }
  fprintf(stderr, "[mt-dense-elide-observability] candidates=%zu droppable=%zu\n",
          candidates.size(), built.size());
  return built;
  }();
  return droppable;
}

bool mtDenseObservabilitySpansBalanced(const std::vector<InstInfo>& insts) {
  std::vector<Node*> stack;
  for (const InstInfo& inst : insts) {
    if (inst.infoType == SUPER_INFO_ASSIGN_BEG) {
      stack.push_back(inst.node);
    } else if (inst.infoType == SUPER_INFO_ASSIGN_END) {
      if (stack.empty() || stack.back() != inst.node) return false;
      stack.pop_back();
    }
  }
  return stack.empty();
}

// When enabled, exclude backward (cross-cycle) activation edges from the dense SCC graph.
// Backward activation edges are next-cycle activations that create false within-cycle cycles.
bool mtUseDenseForwardActivationOnly() {
  const char* env = std::getenv("GSIM_MT_DENSE_FORWARD_ACTIVATION_ONLY");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// dump member-node and instruction-ownership metadata for a single dense task.
// Reads already-computed super->insts (populated by instsGenerator before cppEmitter);
// never calls StmtTree::compute and never mutates super->insts.
void dumpMtDenseMemberMetadataForTask(FILE* fp, SuperNode* super) {
  fprintf(fp, ", \"member_nodes\": [");
  bool first = true;
  int memberIdx = 0;  // content-stable local index (node->id drifts, see oldName)
  for (Node* node : super->member) {
    if (!node) continue;
    if (!first) fprintf(fp, ", ");
    first = false;
    fprintf(fp, "{\"node_id\": %d, \"node_name\": \"%s\", \"node_type\": \"%s\"}",
            memberIdx ++, jsonEscape(node->name).c_str(), nodeTypeName(node->type));
  }
  fprintf(fp, "], \"instruction_ownership\": [");
  first = true;
  // owner ids as content-stable member indices (node->id drifts, see oldName).
  std::map<Node*, int> memberIndex;
  for (size_t mi = 0; mi < super->member.size(); mi ++) memberIndex[super->member[mi]] = (int)mi;
  for (const InstInfo& inst : super->insts) {
    if (!first) fprintf(fp, ", ");
    first = false;
    fprintf(fp, "{\"kind\": \"%s\", \"inst\": \"%s\"", superInfoName(inst.infoType), jsonEscape(inst.inst).c_str());
    // Only ASSIGN_BEG/ASSIGN_END constructors initialize InstInfo::node;
    // the string constructor leaves it uninitialized, so check infoType first.
    if ((inst.infoType == SUPER_INFO_ASSIGN_BEG || inst.infoType == SUPER_INFO_ASSIGN_END) && inst.node) {
      auto miIt = memberIndex.find(inst.node);
      int ownerId = miIt == memberIndex.end() ? -1 : miIt->second;
      fprintf(fp, ", \"owner_node_id\": %d, \"owner_node_name\": \"%s\", \"owner_node_type\": \"%s\"",
              ownerId, jsonEscape(inst.node->name).c_str(), nodeTypeName(inst.node->type));
    }
    fprintf(fp, "}");
  }
  fprintf(fp, "]");
}

static bool mtDenseActivationOriginEdgeEligible(int fromCppId, int toCppId) {
  return fromCppId >= 0 && toCppId >= 0 && fromCppId < superId && toCppId < superId && fromCppId != toCppId;
}

static void dumpMtDenseActivationOriginRecord(FILE* fp,
                                             const Node::ActivationOriginRecord& origin,
                                             const char* kind,
                                             bool& first) {
  if (!mtDenseActivationOriginEdgeEligible(origin.fromCppId, origin.toCppId)) return;
  if (!first) fprintf(fp, ",\n");
  first = false;
  fprintf(fp, "    {\"from_cpp_id\": %d, \"to_cpp_id\": %d, \"kind\": \"%s\", ", origin.fromCppId, origin.toCppId, kind);
  fprintf(fp, "\"reason\": \"%s\", ", jsonEscape(origin.reason).c_str());
  fprintf(fp, "\"source_node_id\": %d, \"source_node_name\": \"%s\", \"source_node_type\": \"%s\", ",
          origin.sourceNodeId, jsonEscape(origin.sourceNodeName).c_str(), nodeTypeName(origin.sourceNodeType));
  fprintf(fp, "\"target_node_id\": %d, \"target_node_name\": \"%s\", \"target_node_type\": \"%s\"}",
          origin.targetNodeId, jsonEscape(origin.targetNodeName).c_str(), nodeTypeName(origin.targetNodeType));
}

void dumpMtDenseActivationOrigins(FILE* fp) {
  bool enabled = mtUseDenseActivationOrigins();
  fprintf(fp, "  \"activation_origin_capture_enabled\": %s,\n", enabled ? "true" : "false");
  fprintf(fp, "  \"activation_origins\": [\n");
  bool first = true;
  if (enabled) {
    for (int cppId = 0; cppId < superId; cppId ++) {
      auto superIter = cppId2Super.find(cppId);
      if (superIter == cppId2Super.end() || !superIter->second) continue;
      for (Node* member : superIter->second->member) {
        if (!member) continue;
        for (const Node::ActivationOriginRecord& origin : member->activationOrigins()) {
          dumpMtDenseActivationOriginRecord(fp, origin, "active", first);
          if (!isAlwaysActive(origin.toCppId)) {
            dumpMtDenseActivationOriginRecord(fp, origin, "need_activate", first);
          }
        }
      }
    }
  }
  fprintf(fp, "\n  ],\n");
}

bool mtIsLevelDispatchMode() {
  return globalConfig.MtHelperMode == "mt-level-dispatch";
}

static bool mtCodegenEnvEnabledByDefault(const char* name) {
  const char* env = std::getenv(name);
  return env == nullptr || env[0] == '\0' || env[0] != '0';
}


bool mtUseDirectInlineFallback() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_DIRECT_INLINE_FALLBACK");
}

bool mtUseDirectInlineSerialFallback() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_DIRECT_INLINE_SERIAL_FALLBACK");
}

bool mtUseDirectInlineWorker0Fallback() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_DIRECT_INLINE_WORKER0_FALLBACK");
}

// A77 D1-NARROW / A110 Probe 5: codegen-time, default-on profile-off fast path.
// The retained diagnostic/profile-capable emission is still available by setting
// GSIM_MT_PROFILE_OFF_DIRECT_SERIAL_FALLBACK=0 during gsim-gen-cpp. Default-on
// drops serial-fallback profile wrappers in the generated profile-off hot path.
bool mtUseProfileOffDirectSerialFallback() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_PROFILE_OFF_DIRECT_SERIAL_FALLBACK");
}

// Default-on profile-off fast path for non-coarse active-word accounting in
// generated MT substeps. Set GSIM_MT_PROFILE_OFF_ACTIVE_WORD_COUNT=0 during
// gsim-gen-cpp to retain the diagnostic counter branch.
bool mtUseProfileOffActiveWordCount() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_PROFILE_OFF_ACTIVE_WORD_COUNT");
}

// Default-on fast path for pure batches that runtime will force to workerCount=1
// under the default min-batch threshold. The generated branch preserves runtime
// lowering of GSIM_MT_MIN_BATCH_TASKS by falling back to mtRunPureBatch().
bool mtUseInlineSmallPureBatches() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_INLINE_SMALL_PURE_BATCHES");
}

// Probe: inside the small-batch inline path, optionally inline each pure task
// body instead of calling mtTaskN(oldFlag).
bool mtUseInlineSmallPureBatchBodies() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_INLINE_SMALL_PURE_BATCH_BODIES");
}

// Default-on guard around inlined small-batch task bodies. When no task bit in
// the static batch mask is active, skip the generated per-bit branch chain.
bool mtUseInlineSmallPureBatchMaskGuard() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_INLINE_SMALL_PURE_BATCH_MASK_GUARD");
}

// Default-on algorithmic fast path: skip subStep function calls whose generated
// body only consumes currently-zero active-flag words. This suppresses call
// overhead above the existing in-function sparse guards. Set
// GSIM_MT_STEP_ACTIVE_WORD_GUARD=0 during codegen to retain the older shape.
bool mtUseStepActiveWordGuard() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_STEP_ACTIVE_WORD_GUARD");
}

bool mtUseSplitMixedStepGuards() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_SPLIT_MIXED_STEP_GUARDS");
}


// Probe-only: emit static graph data for runtime/cycle clean-region batching
// validation. Default-off and report-only; normal generated execution is unchanged.
bool mtUseCycleBatchReport() {
  const char* env = std::getenv("GSIM_MT_CYCLE_BATCH_REPORT");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Probe-only: emit lane-local SCC ready-batch scheduler metadata. Default-off
// and report-only; normal generated execution is unchanged.
bool mtUseReadyBatchReport() {
  const char* env = std::getenv("GSIM_MT_READY_BATCH_REPORT");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Probe-only: decompose local envelope-eval pressure into serial reasons,
// active-word concentration, and counterfactual schedules. Default-off and
// report-only; normal generated execution is unchanged.
bool mtUseEnvelopeLocalEvalDiagnostics() {
  const char* env = std::getenv("GSIM_MT_ENVELOPE_LOCAL_EVAL_DIAGNOSTICS");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Probe-only: include generated runtime support for state-update dynamic hit
// trace lines. Default-off at codegen; emitted models still require
// GSIM_MT_DYNAMIC_STATE_TRACE=1 alongside GSIM_MT_DYNAMIC_TRACE at runtime.
bool mtUseDynamicStateTraceCodegen() {
  const char* env = std::getenv("GSIM_MT_DYNAMIC_STATE_TRACE_CODEGEN");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Default-off exact sparse activation-event trace support. The generated model
// still requires GSIM_MT_ACTIVATION_EVENT_TRACE=/path and a non-empty dynamic
// trace cycle range at runtime.
bool mtUseActivationEventTraceCodegen() {
  const char* env = std::getenv("GSIM_MT_ACTIVATION_EVENT_TRACE_CODEGEN");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}


// Default-off dense-executor codegen: emit a whole-design dense schedule report and,
// when the graph is acyclic, dense runtime wrappers selected by GSIM_MT_EXECUTOR=dense.
bool mtUseDenseExecutorCodegen() {
  const char* env = std::getenv("GSIM_MT_DENSE_EXECUTOR_CODEGEN");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// GSIM_MT_DENSE_ONLY_CODEGEN (default off): emit only the dense executor and
// the lean serial-fast fallback. The sparse-dispatch runtime text is dropped:
// the buffered mtTaskN(flag, ActivationDelta&) helpers, the plain serial
// subStepN() scan (with its coarse-region dispatch and pure-batch calls), the
// pure-batch shard switch tables, and the whole coarse runner. The worker
// pool itself stays (the dense executor posts jobKind 6/7 jobs to it); its
// sparse job kinds are not emitted. step() aborts with a clear message when
// the configured runtime would need the sparse path. Default off keeps the
// generated model byte-identical. See mtDenseOnlyCodegenLevel() for =2.
static int mtDenseOnlyCodegenLevel() {
  const char* env = std::getenv("GSIM_MT_DENSE_ONLY_CODEGEN");
  // Default ON (level 2) under the dense recipe (--mt-helper-mode=mt-level-dispatch
  // plus GSIM_MT_DENSE_EXECUTOR_CODEGEN=1): the dense executor is the shipped
  // runtime and the dropped text (sparse-dispatch runtime, SerialFast) is dead
  // weight for it. Outside the dense recipe the default stays OFF so upstream-
  // style generation keeps its legacy output. Set =0 (or none) to opt back into
  // the legacy full emission under the dense recipe too.
  const bool denseRecipe = globalConfig.MtHelperMode == "mt-level-dispatch" && mtUseDenseExecutorCodegen();
  if (env == nullptr || env[0] == '\0') return denseRecipe ? 2 : 0;
  if (env[0] == '0' || strncmp(env, "none", 4) == 0) return 0;
  int level = std::atoi(env);
  if (level < 1) level = 1;   // any truthy non-numeric value behaves as level 1
  if (level > 2) level = 2;
  return level;
}

bool mtUseDenseOnlyCodegen() {
  return mtDenseOnlyCodegenLevel() >= 1;
}

// GSIM_MT_DENSE_ONLY_CODEGEN=2 ("level 2") additionally drops the SerialFast
// subSteps (subStepNSerialFast) and step()'s serial-fast dispatch branch: the
// T<=mtSparseSerialFastMaxWorkers fallback is a different deployment target
// than a dense-only model. step() keeps only the dense executor path and an
// abort for everything else. Level 1 behavior is unchanged; default off keeps
// the generated model byte-identical.
bool mtUseDenseOnlyCodegenLevel2() {
  return mtDenseOnlyCodegenLevel() >= 2;
}

bool mtUseDenseXThreadDepsOnly() {
  const char* env = std::getenv("GSIM_MT_DENSE_XTHREAD_DEPS_ONLY");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

bool mtUseDenseTransitiveReduceEdges() {
  const char* env = std::getenv("GSIM_MT_DENSE_TRANSITIVE_REDUCE_EDGES");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

bool mtUseDenseStaticEmptyElide() {
  const char* env = std::getenv("GSIM_MT_DENSE_STATIC_EMPTY_ELIDE");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Diagnostics-only layout tables for the fixed-order dependency-counter executor
// (identity vertex slots + per-mtask owner). Generation is default-off.
bool mtUseDenseOwnerBankCountersDiag() {
  const char* env = std::getenv("GSIM_MT_DENSE_OWNER_BANK_COUNTERS_DIAG");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// optionally emit compile-exclusive producer-owner ready flags beside the
// fixed-order identity dependency-counter layouts.
bool mtUseDenseOwnerReadyFlags() {
  const char* env = std::getenv("GSIM_MT_DENSE_OWNER_READY_FLAGS");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// default-off compile-exclusive per-worker completion join.
bool mtUseWorkerPoolFlagJoinCodegen() {
  const char* env = std::getenv("GSIM_MT_WORKER_POOL_FLAG_JOIN");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}





// Default-off emission of the GSIM_MT_OWNER_CPU_MAP runtime machinery.
// Unset emits no map text and the original upstream spawn-pinning block.
bool mtUseOwnerCpuMapCodegen() {
  const char* env = std::getenv("GSIM_MT_OWNER_CPU_MAP_CODEGEN");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Default-off dense ready-token scheduler breakdown profiler. Generation is
// deliberately opt-in so its unset path emits no additional model text.
bool mtUseDenseBreakdownProfileCodegen() {
  const char* env = std::getenv("GSIM_MT_DENSE_BREAKDOWN_PROFILE");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Bounded per-cycle dense-breakdown window trace. This is a codegen sub-gate:
// absent window knobs leave Slice A's emitted model text byte-identical.
bool mtUseDenseBreakdownWindowCodegen() {
  const char* start = std::getenv("GSIM_MT_DENSE_BREAKDOWN_WINDOW_START");
  const char* cycles = std::getenv("GSIM_MT_DENSE_BREAKDOWN_WINDOW_CYCLES");
  const bool hasStart = start != nullptr && start[0] != '\0';
  const bool hasCycles = cycles != nullptr && cycles[0] != '\0';
  if (!hasStart && !hasCycles) return false;
  Assert(hasStart && hasCycles,
         "GSIM_MT_DENSE_BREAKDOWN_WINDOW_START and GSIM_MT_DENSE_BREAKDOWN_WINDOW_CYCLES must be set together");
  return true;
}

// Default-off lookahead-compatible body-only window sub-gate. Requires the
// breakdown window knobs (GSIM_MT_DENSE_BREAKDOWN_PROFILE=1 plus
// GSIM_MT_DENSE_BREAKDOWN_WINDOW_START/CYCLES) and GSIM_MT_DENSE_LOOKAHEAD>0.
// When set, per-MTask body spans are recorded inside each stepDenseMTaskN()
// body so all four lookahead dispatch sites (inline fast path plus the three
// tail paths) are covered without touching dispatch tables, and allownerbody
// becomes the only accepted runtime window mode. Unset keeps breakdown+
// lookahead rejected and the emitted model byte-identical.
bool mtUseDenseBreakdownWindowLaBodyCodegen() {
  const char* env = std::getenv("GSIM_MT_DENSE_BREAKDOWN_WINDOW_LA_BODY");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}


// reorder only dense MTask function emission by fixed owner so each worker's hot text is
// contiguous. Logical IDs, bodies, owner call order, dependency protocol, and schedule are unchanged.
bool mtUseDenseWorkerMajorText() {
  const char* env = std::getenv("GSIM_MT_DENSE_WORKER_MAJOR_TEXT");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}


// Default-off bounded lookahead.  A positive value is the candidate window;
// unset, empty, and zero retain the strict-order dispatch path byte-for-byte.
int mtDenseLookaheadWindow() {
  const char* env = std::getenv("GSIM_MT_DENSE_LOOKAHEAD");
  if (env == nullptr || env[0] == '\0') return 0;
  char* end = nullptr;
  const long window = std::strtol(env, &end, 10);
  Assert(end != env && end != nullptr && *end == '\0' && window >= 0
             && window <= std::numeric_limits<int>::max(),
         "GSIM_MT_DENSE_LOOKAHEAD must be 0 or an integer window N >= 1 (got %s)", env);
  return static_cast<int>(window);
}

// Default-off perturbation-free duty-cycle instrumentation.  Per-cycle, per-thread
// chrono into 64B-padded per-lane counters (no shared cache line, ~8 clock reads
// per worker per cycle, ~0.2% of a 106us cycle) — replaces the per-task chrono
// whose single shared counter line distorted profiled runs ~19x.
bool mtDenseDutyCodegen() {
  const char* env = std::getenv("GSIM_MT_DENSE_DUTY");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}





// activity local collector: register reads (REG_SRC) plus MEMORY/array reads. The shared
// MtBoundaryInfo read set only records NODE_REG_SRC names; memory readers (NODE_READER /
// READWRITER / WRITER / MEMORY) were missing from the activation fanout, leaving a wavefront
// closure hole (L2/array-driven one-hot assertions fired on the first build of this collector). The collector
// must be COMPLETE: any missed read is a closure hole that lets a body compute from stale inputs

static std::map<Node*, MtActivityReads> mtActivityReadCache;
static const MtActivityReads& mtActivityReadsForNode(Node* n);
void mtActivityCollectFromTree(ENode* root, MtActivityReads& out) {
  if (!root) return;
  std::stack<ENode*> st;
  st.push(root);
  std::set<ENode*> seen;
  while (!st.empty()) {
    ENode* t = st.top(); st.pop();
    if (!t || seen.count(t)) continue;
    seen.insert(t);
    for (ENode* c : t->child) st.push(c);
    Node* n = t->nodePtr;
    if (!n) continue;
    if (n->type == NODE_REG_SRC) { out.src.insert(n->name); continue; }
    if (n->type == NODE_REG_DST) { Node* s = n->getSrc(); if (s) out.dst.insert(s->name); continue; }
    if (n->type == NODE_REG_RESET) { Node* s = n->getResetSrc(); if (s) out.dst.insert(s->name); continue; }
    if (n->type == NODE_MEMORY || n->type == NODE_READER || n->type == NODE_READWRITER || n->type == NODE_WRITER) {
      if (n->parent) out.src.insert(n->parent->name);
      out.src.insert(n->name);
      continue;
    }
    if (!n->assignTree.empty()) {
      const MtActivityReads& sub = mtActivityReadsForNode(n);
      out.src.insert(sub.src.begin(), sub.src.end());
      out.dst.insert(sub.dst.begin(), sub.dst.end());
      continue;
    }
    if (n->type == NODE_INP) { out.src.insert(n->name); continue; }
  }
}
static const MtActivityReads& mtActivityReadsForNode(Node* n) {
  auto it = mtActivityReadCache.find(n);
  if (it != mtActivityReadCache.end()) return it->second;
  // insert placeholder first to terminate on any unexpected self-reference cycle
  auto res = mtActivityReadCache.emplace(n, MtActivityReads());
  for (ExpTree* t : n->assignTree) mtActivityCollectFromTree(t->getRoot(), res.first->second);
  return res.first->second;
}




bool mtUseDenseHybridEligibilityDiag() {
  const char* env = std::getenv("GSIM_MT_DENSE_HYBRID_ELIGIBILITY_DIAG");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}


bool mtUseDenseSplitWorker0MTasks() {
  const char* env = std::getenv("GSIM_MT_DENSE_SPLIT_WORKER0_MTASKS");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

// Default-on active-path optimization: when a clean coarse region's static
// maximum active bits cannot exceed the runtime inline threshold, generated code
// skips the per-region popcount scan and goes straight to serial-inline fallback.
// Lower runtime thresholds still fall back to the existing dynamic popcount gate.
// Set GSIM_MT_STATIC_INLINE_BOUND=0 during codegen to emit the old always-popcount gate.
bool mtUseStaticCoarseInlineBound() {
  return mtCodegenEnvEnabledByDefault("GSIM_MT_STATIC_INLINE_BOUND");
}

// admission gate for the coarse region under mt-level-dispatch.
// pure_compute matches mtTaskCanEnterPureBatch; safe-serial cppIds whose only
// serial_reasons are state_update/reset/async_reset/activate_all_path/
// array_or_dynamic_index/super_type_SUPER_ASYNC_RESET are also admitted.
// Worker0-only side effects (external/memory_write/memory_read_unsupported/special)
// and future/unknown serial reasons are rejected: they fall through to the
// main-thread serial path.

static bool mtRepCutNameChar(char ch) {
  return std::isalnum(static_cast<unsigned char>(ch)) || ch == '_' || ch == '$';
}


// ---- GSIM_SHORT_NAMES: emission-time node-name interning (default off) ----
// Long node names (>= 8 bytes) are renamed to _v<idx> right before body
// emission. Full names stay for input/output nodes (the set_/get_ accessor
// spellings are an external contract) and for names that are already short.
// The mapping is kept in mtShortNameOrig and re-emitted as `// orig=<name>`
// comments at the genNodeDef declaration site so models stay debuggable.
static std::map<Node*, std::string> mtShortNameOrig;
static std::unordered_map<std::string, std::string> mtShortNameLookup;
static size_t mtShortNameMinFromLen = 0;

bool mtUseShortNames() {
  const char* env = std::getenv("GSIM_SHORT_NAMES");
  // Default OFF: interning is NEMU-exact but NOT perf-neutral - a T32 C50000
  // 5-pair measured +4.5% (champ 5.338s vs interned 5.578s, non-overlapping;
  // T16 showed the same direction at +1.4%). Shorter identifiers shift znver4
  // codegen alignment in the hot bodies. Opt in only for size/disk-constrained
  // builds (4.8GB -> 1.1GB model).
  if (env == nullptr || env[0] == '\0') return false;
  return env[0] != '0';
}

const std::string* mtShortNameOrigOf(Node* node) {
  if (mtShortNameOrig.empty()) return nullptr;
  auto iter = mtShortNameOrig.find(node);
  return iter == mtShortNameOrig.end() ? nullptr : &iter->second;
}

// String-keyed replacement-map rewriter for the InstInfo::inst snippets baked
// by instsGenerator. Token-boundary semantics (mtRepCutNameChar), implemented as one
// left-to-right pass: names are unique whole tokens and replacement outputs
// (_v<idx>, shorter than every from-name) are never themselves in the from-set,
// so per-token lookup is equivalent to the reference's sequential
// longest-replacement-first scan while staying linear in the text size.
// Tokens that start with a digit are skipped: names never do, and this keeps
// numeric literals (incl. long hex constants) out of the hash path.
static void mtRewriteInstNodeNames(std::string& text) {
  if (mtShortNameLookup.empty()) return;
  std::string out;
  out.reserve(text.size());
  size_t i = 0;
  const size_t size = text.size();
  while (i < size) {
    char ch = text[i];
    if (mtRepCutNameChar(ch) && !(ch >= '0' && ch <= '9')) {
      size_t j = i + 1;
      while (j < size && mtRepCutNameChar(text[j])) j ++;
      if (j - i >= mtShortNameMinFromLen) {
        auto iter = mtShortNameLookup.find(text.substr(i, j - i));
        if (iter != mtShortNameLookup.end()) {
          out.append(iter->second);
          i = j;
          continue;
        }
      }
      out.append(text, i, j - i);
      i = j;
    } else {
      out.push_back(ch);
      i ++;
    }
  }
  text.swap(out);
}

void graph::mtInternNodeNames() {
  // Collect every node the emitter can reach. graph::allNodes is never
  // populated; the live universe is the super containers' members plus the
  // standalone node vectors from include/graph.h.
  std::set<Node*> nodes;
  auto collectSuper = [&](SuperNode* super) {
    if (!super) return;
    for (Node* member : super->member) {
      if (member) nodes.insert(member);
    }
  };
  for (SuperNode* super : sortedSuper) collectSuper(super);
  for (SuperNode* super : supersrc) collectSuper(super);
  for (SuperNode* super : allReset) collectSuper(super);
  for (Node* node : memory) if (node) nodes.insert(node);
  for (Node* node : specialNodes) if (node) nodes.insert(node);
  for (Node* node : external) if (node) nodes.insert(node);
  for (Node* node : regsrc) if (node) nodes.insert(node);

  // Interning is NAME-keyed, not node-keyed: REG_DST/REG_RESET twins share
  // their REG_SRC's name, so two Node objects can carry one spelling. Every
  // node holding a name must flip together with the InstInfo token rewrite,
  // or bodies reference an undeclared member.
  std::set<Node*> keepFull;
  for (Node* node : input) if (node) keepFull.insert(node);
  for (Node* node : output) if (node) keepFull.insert(node);
  std::set<std::string> keptNames;
  std::set<std::string> internableNames;
  for (Node* node : nodes) {
    if (!node || node->name.empty()) continue;
    // Extmodule blackbox calls declare and call free functions named after
    // the EXT node (instsGenerator's computeExtMod); those spellings are baked
    // outside the InstInfo token rewrite, so their names must stay full.
    if (node->type == NODE_EXT) { keptNames.insert(node->name); continue; }
    if (keepFull.count(node) || node->name.size() < 8) {
      keptNames.insert(node->name);
    } else {
      internableNames.insert(node->name);
    }
  }
  // Deterministic assignment: names in sorted order; a generated _v<idx> that
  // would repeat a kept name is skipped so the generated pool stays disjoint.
  std::map<std::string, std::string> nameMap;
  size_t nextIdx = 0;
  for (const std::string& name : internableNames) {
    std::string shortName;
    do {
      shortName = format("_v%zu", nextIdx ++);
    } while (keptNames.count(shortName) != 0);
    if (shortName.size() >= name.size()) continue;  // never lengthen
    nameMap[name] = shortName;
  }
  if (nameMap.empty()) return;

  mtShortNameOrig.clear();
  mtShortNameLookup.clear();
  mtShortNameMinFromLen = SIZE_MAX;
  for (const auto& mapping : nameMap) {
    mtShortNameLookup[mapping.first] = mapping.second;
    mtShortNameMinFromLen = std::min(mtShortNameMinFromLen, mapping.first.size());
  }
  for (Node* node : nodes) {
    if (!node) continue;
    if (nameMap.count(node->name) != 0) mtShortNameOrig[node] = node->name;
  }

  // Rewrite the baked InstInfo::inst snippets before mutating Node::name:
  // the from-keys are the original names. Supers can appear in more than one
  // container (sortedSuper vs supersrc); dedupe by pointer so each inst string
  // is rewritten exactly once. Independent per-super work, no shared state
  // besides the read-only lookup - render on a worker pool.
  std::vector<SuperNode*> supers;
  {
    std::set<SuperNode*> seen;
    auto pushSuper = [&](SuperNode* super) {
      if (super && seen.insert(super).second) supers.push_back(super);
    };
    for (SuperNode* super : sortedSuper) pushSuper(super);
    for (SuperNode* super : supersrc) pushSuper(super);
    for (SuperNode* super : allReset) pushSuper(super);
  }
  {
    std::atomic<size_t> next(0);
    const size_t nWorkers = std::min((size_t)emitParallelThreadCount(), supers.size());
    std::vector<std::thread> pool;
    pool.reserve(nWorkers);
    for (size_t w = 0; w < nWorkers; w ++) {
      pool.emplace_back([&]() {
        size_t u;
        while ((u = next.fetch_add(1, std::memory_order_relaxed)) < supers.size()) {
          SuperNode* super = supers[u];
          for (InstInfo& inst : super->insts) {
            if (!inst.inst.empty()) mtRewriteInstNodeNames(inst.inst);
          }
        }
      });
    }
    for (std::thread& thread : pool) thread.join();
  }
  // Mutate Node::name last: from here on every emission site (member
  // declarations, $old$/$RESET/cond_ temporaries, activateNext commits,
  // accessor member reads) derives from the short name. All twins of a
  // shared name flip together.
  size_t internedNodeCount = 0;
  for (Node* node : nodes) {
    auto mapping = nameMap.find(node->name);
    if (mapping == nameMap.end()) continue;
    node->name = mapping->second;
    internedNodeCount ++;
  }
  uint64_t instBytesRewritten = 0;
  for (SuperNode* super : supers) {
    for (const InstInfo& inst : super->insts) instBytesRewritten += inst.inst.size();
  }
  fprintf(stderr, "[gsim-short-names] interned %zu names across %zu nodes (%.1f MB inst text rewritten)\n",
          nameMap.size(), internedNodeCount, (double)instBytesRewritten / (1024.0 * 1024.0));
}


std::pair<int, int> cppId2flagIdx(int cppId) {
  int id = cppId / ACTIVE_WIDTH;
  int bit = cppId % ACTIVE_WIDTH;
  return std::make_pair(id, bit);
}

std::pair<int, uint64_t>setIdxMask(int cppId) {
  int id, bit;
  std::tie(id, bit) = cppId2flagIdx(cppId);
  uint64_t mask = (uint64_t)1 << bit;
  return std::make_pair(id, mask);
}

std::pair<int, uint64_t>clearIdxMask(int cppId) {
  int id, bit;
  std::tie(id, bit) = cppId2flagIdx(cppId);
  uint64_t mask = (uint64_t)1 << bit;
  if (ACTIVE_WIDTH == 64) mask = ~mask;
  else mask = (~mask) & (((uint64_t)1 << ACTIVE_WIDTH) - 1);
  return std::make_pair(id, mask);
}

ActiveType activeSet2bitMap(std::set<int>& activeId, std::map<uint64_t, ActiveType>& bitMapInfo, int curId) {
  uint64_t ret = 0;
  std::string comment = "";
  int uniqueIdx = 0;
  for (int id : activeId) {
    if (isAlwaysActive(id)) continue;
    int bitMapId;
    uint64_t bitMapMask;
    std::tie(bitMapId, bitMapMask) = setIdxMask(id);
    int num = 64 / ACTIVE_WIDTH;
    if (curId >= 0 && id > curId && bitMapId == curId / ACTIVE_WIDTH) {
      if (ret == 0) uniqueIdx = id % ACTIVE_WIDTH;
      else uniqueIdx = -1;
      ret |= bitMapMask;
      comment += std::to_string(id) + " ";
    } else {
      int beg = bitMapId - bitMapId % num;
      int end = beg + num;
      int findType = 0;
      uint64_t newMask = bitMapMask << ((bitMapId - beg) * ACTIVE_WIDTH);
      std::string newComment = std::to_string(id);
      if (bitMapInfo.find(bitMapId) != bitMapInfo.end()) {
        ACTIVE_MASK(bitMapInfo[bitMapId]) |= bitMapMask;
        ACTIVE_COMMENT(bitMapInfo[bitMapId]) += " " + std::to_string(id);
        ACTIVE_UNIQUE(bitMapInfo[bitMapId]) = -1;
        findType = 1; // no nothing
      } else {
        for (int newId = beg; newId < end; newId ++) {
          if (bitMapInfo.find(newId) != bitMapInfo.end()) {
            newMask |= ACTIVE_MASK(bitMapInfo[newId]) << ((newId - beg) * ACTIVE_WIDTH);
            newComment += " " + ACTIVE_COMMENT(bitMapInfo[newId]);
            findType = 2;  // find to merge
            bitMapInfo.erase(newId);
          }
        }
      }
      if (findType == 0) bitMapInfo[bitMapId] = std::make_tuple(bitMapMask, std::to_string(id), id % ACTIVE_WIDTH);
      else if (findType == 2) bitMapInfo[beg] = std::make_tuple(newMask, newComment, -1);
    }
  }
  return std::make_tuple(ret, comment, uniqueIdx);
}

std::string updateActiveStr(int idx, uint64_t mask, const std::string& activeBufferName) {
  if (!activeBufferName.empty()) return format("%s.orWord(%d, 0x%lx);", activeBufferName.c_str(), idx, mask);
  if (mask <= MAX_U8) return format("activeFlags[%d] |= 0x%lx;", idx, mask);
  if (mask <= MAX_U16) return format("*(uint16_t*)&activeFlags[%d] |= 0x%lx;", idx, mask);
  if (mask <= MAX_U32) return format("*(uint32_t*)&activeFlags[%d] |= 0x%lx;", idx, mask);
  return format("*(uint64_t*)&activeFlags[%d] |= 0x%lx;", idx, mask);
}

std::string updateActiveStr(int idx, uint64_t mask, std::string& cond, int uniqueId, const std::string& activeBufferName) {
  if (!activeBufferName.empty()) {
    if (uniqueId >= 0) {
      return format("%s.orWord(%d, %s%s);", activeBufferName.c_str(), idx, cond.c_str(), shiftBits(uniqueId, ShiftDir::Left).c_str());
    }
    int castWidth = 64;
    if (mask <= MAX_U8) castWidth = 8;
    else if (mask <= MAX_U16) castWidth = 16;
    else if (mask <= MAX_U32) castWidth = 32;
    return format("%s.orWord(%d, -(uint%d_t)%s & 0x%lx);", activeBufferName.c_str(), idx, castWidth, cond.c_str(), mask);
  }
  auto activeFlags = std::string("activeFlags[") + std::to_string(idx) + std::string("]");

  if (mask <= MAX_U8) {
    if (uniqueId >= 0) return format("%s |= %s%s;", activeFlags.c_str(), cond.c_str(), shiftBits(uniqueId, ShiftDir::Left).c_str());
    else return format("%s |= -(uint8_t)%s & 0x%lx;", activeFlags.c_str(), cond.c_str(), mask, activeFlags.c_str());
  }
  if (mask <= MAX_U16)
    return format("*(uint16_t*)&%s |= -(uint16_t)%s & 0x%lx;", activeFlags.c_str(), cond.c_str(), mask, activeFlags.c_str());
  if (mask <= MAX_U32)
    return format("*(uint32_t*)&%s |= -(uint32_t)%s & 0x%lx;", activeFlags.c_str(), cond.c_str(), mask, activeFlags.c_str());
  return format("*(uint64_t*)&%s |= -(uint64_t)%s & 0x%lx;", activeFlags.c_str(), cond.c_str(), mask, activeFlags.c_str());
}

void includeLib(FILE* fp, std::string lib, bool isStd) {
  std::string format = isStd ? "#include <%s>\n" : "#include \"%s\"\n";
  fprintf(fp, format.c_str(), lib.c_str());
}

void newLine(FILE* fp) {
  fprintf(fp, "\n");
}

int emitParallelThreadCount() {
  return std::max(1, std::min(16, (int)std::thread::hardware_concurrency()));
}
