// cppEmitterDenseExec.cpp - split from cppEmitter.cpp (pure code motion):
// genDenseExecutor - moved WHOLE (no lambda extraction): activity provenance,
// owner-ready/lookahead tables, per-MTask dispatch bodies, the dense worker pool
// entry points and stepDense().
#include "cppEmitterImpl.h"

// ---- GSIM_EMIT_TASK_LOCALS (WP1): per-task register-state value localization ----
// The dense MTask bodies evaluate statements that repeatedly load/store class
// members (register current values `foo` and next-state buffers `foo$NEXT`);
// that intra-body memory-mediated state flow is the measured 2.85x instruction
// tax vs the classic serial emitter. When the knob is on, each stepDenseMTaskN
// shadows the eligible scalar register-state members it touches with same-name
// value locals loaded once at body entry and writes the written ones back
// before returning. Token release stores live in the dispatcher after the
// return, so cross-worker visibility is unchanged; same-worker chains see the
// predecessor's write-backs by program order.
//
// The collector derives the read/write member sets from the same structure the
// statement generator consumes (StmtNode::compute): every member reference in
// the emitted text comes from an ENode nodePtr (allocNodeInfo emits node->name),
// assignment leaves carry the write target as the lvalue root, WHEN conditions
// and RHS subtrees are reads. Node classification follows the activity
// collector (mtActivityCollectFromTree): REG_SRC/REG_DST references are
// register state, REG_RESET references and the subResetDense helpers invoked
// around SUPER_ASYNC_RESET supers must stay member-resident, arrays/memories
// (the dynamic-index boundary) keep direct member access. Unlike the activity
// collector it does NOT recurse into shared wires' assignTrees: the emitted
// body text names the wire itself, and its computation is emitted in its own
// super, so recursion would only overapproximate this body's reads.
namespace {
class MtTaskLocalCollector {
 public:
  std::set<std::string> reads;            // scalar register-state names read
  std::set<std::string> writes;           // scalar register-state names written
  std::map<std::string, Node*> typeNode;  // name -> representative node (width/type)
  std::set<std::string> resetTouched;     // names referenced through NODE_REG_RESET nodes
  std::set<std::string> nonRegNames;      // array/memory-class or undeclared names

  void walkSuper(SuperNode* super) {
    if (super == nullptr || super->stmtTree == nullptr || super->stmtTree->root == nullptr) return;
    walkStmt(super->stmtTree->root);
  }

 private:
  std::set<ENode*> seen;

  void classify(Node* n, bool isWrite) {
    if (n == nullptr || n->name.empty()) return;
    if (n->type == NODE_REG_SRC || n->type == NODE_REG_DST) {
      const bool declared = n->status == VALID_NODE && (n->type == NODE_REG_SRC || n->regSplit);
      if (!declared || n->isArray() || n->width <= 0) {
        nonRegNames.insert(n->name);
        return;
      }
      typeNode.emplace(n->name, n);
      if (isWrite) writes.insert(n->name);
      else reads.insert(n->name);
      return;
    }
    if (n->type == NODE_REG_RESET) {
      resetTouched.insert(n->name);
      return;
    }
    if (n->type == NODE_MEMORY || n->type == NODE_READER || n->type == NODE_READWRITER || n->type == NODE_WRITER) {
      if (n->parent != nullptr) nonRegNames.insert(n->parent->name);
      nonRegNames.insert(n->name);
      return;
    }
    // NODE_INP / NODE_OUT / NODE_OTHERS / NODE_SPECIAL: wires, inputs and
    // observability text - not register state, no localization decision.
  }

  void walkExp(ENode* e) {
    if (e == nullptr || seen.count(e)) return;
    seen.insert(e);
    for (ENode* c : e->child) walkExp(c);
    if (e->memoryNode != nullptr) classify(e->memoryNode, false);
    classify(e->nodePtr, false);
  }

  void walkStmt(StmtNode* s) {
    if (s == nullptr) return;
    if (s->type == OP_STMT_SEQ) {
      for (StmtNode* c : s->child) walkStmt(c);
      return;
    }
    if (s->type == OP_STMT_WHEN) {
      if (!s->child.empty() && s->child[0]->isENode) walkExp(s->child[0]->enode);
      if (s->child.size() > 1) walkStmt(s->child[1]);
      if (s->child.size() > 2) walkStmt(s->child[2]);
      return;
    }
    if (s->isENode) {
      walkExp(s->enode);
      return;
    }
    if (s->tree == nullptr) return;
    ENode* lval = s->tree->getlval();
    if (lval != nullptr) {
      classify(lval->getNode(), true);  // assignment target, classified outside the seen-dedup
      for (ENode* c : lval->child) walkExp(c);
      seen.insert(lval);                // root already handled; skip re-classifying it as a read
    }
    walkExp(s->tree->getRoot());
  }
};
}  // namespace

void graph::genDenseExecutor(const MtDenseSchedule& denseSchedule, FILE* header) {
  Assert(denseSchedule.valid, "cannot emit dense executor for invalid schedule: %s", denseSchedule.fallbackReason.c_str());
  Assert(!denseSchedule.mtasks.empty(), "dense executor requires MTask partitioning");
  bool savedActivationEventTraceSuppressed = mtActivationEventTraceSuppressed;
  mtActivationEventTraceSuppressed = true;
  int nMTasks = static_cast<int>(denseSchedule.mtasks.size());
  int threadCount = 8;
  const char* threadsEnv = std::getenv("GSIM_THREADS");
  if (threadsEnv != nullptr && threadsEnv[0] != '\0') threadCount = std::atoi(threadsEnv);
  if (threadCount < 1) threadCount = 1;
  bool xthreadDepsOnly = mtUseDenseXThreadDepsOnly();
  bool transitiveReduceEdges = mtUseDenseTransitiveReduceEdges();
  bool staticEmptyElide = mtUseDenseStaticEmptyElide();
  bool ownerBankCountersDiag = mtUseDenseOwnerBankCountersDiag();
  bool ownerReadyFlags = mtUseDenseOwnerReadyFlags();
  // PSCD slice 1 (default off): producer change-bits + optional consumer
  // shadow cross-check. All PSCD emission is generation-gated on pscdBits and
  // compile-gated on GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE, so both knobs
  // unset keep every emitted byte identical.
  bool pscdBits = mtUseEmitPscdBits();
  bool pscdVerify = mtUsePscdVerify();
  // Per-edge wait-latency instrumentation (default off), same gating
  // discipline as PSCD: generation-gated here, compile-gated on the
  // owner-ready macro, byte-identical when unset.
  bool edgeTiming = mtUseDenseEdgeTiming();
  // Push-ready shadow (default off): full-model fan-in oracle census for the
  // proposed push protocol. Same gating discipline as PSCD/edge-timing:
  // generation-gated here, compile-gated on the owner-ready macro, so both
  // knobs unset keep every emitted byte identical.
  bool pushShadow = mtUseDensePushReadyShadow();
  // Push-ready (default off): the hybrid push protocol itself, consuming the
  // exact fan-in/notify census the shadow knob validated. Worker0 keeps the
  // full pull lane; pool workers 1..N-1 become queue-driven. Requires
  // owner-ready flags + lookahead and is mutually exclusive with the
  // shadow/PSCD/edge-timing/breakdown diagnostic knobs (the shadow proved the
  // mapping; running it alongside would double-count every edge).
  bool pushReady = mtUseDensePushReady();
  bool pushReadyDebug = pushReady && mtUseDensePushReadyDebug();
  // GSIM_MT_DENSE_PUSH_READY_DIRECT_TABLE (default off): refinement of the
  // push protocol's dispatch only - replaces stepDensePushReadyRun's
  // generated all-MTask switch with a static member-function pointer table
  // (same bodies, same id mapping, same invalid-id abort). Requires
  // GSIM_MT_DENSE_PUSH_READY=1; unset keeps the emitted switch identical.
  const bool pushReadyDirectTableKnob = mtUseDensePushReadyDirectTable();
  bool pushReadyDirectTable = pushReady && pushReadyDirectTableKnob;
  // GSIM_MT_DENSE_PUSH_READY_BITMAP (default off): queue-free refinement of
  // the push protocol's arrival mechanics, consuming the exact same fan-in
  // CSR, pending counters, worker0 hybrid pull lane and owner-ready token
  // stores as v1. Each push worker's MPMC sequence ring is replaced by one
  // lock-free ready byte per MTask: notify release-stores the byte on the
  // final fan-in decrement and the owner worker acquire-scans its own
  // kDenseDispatchTableW slice until its exact assigned count has fired.
  // Requires GSIM_MT_DENSE_PUSH_READY=1 and is mutually exclusive with the
  // direct-table refinement so the comparison stays isolated. Unset keeps
  // the emitted v1 runtime byte-identical.
  const bool pushReadyBitmapKnob = mtUseDensePushReadyBitmap();
  bool pushReadyBitmap = pushReady && pushReadyBitmapKnob;
  // GSIM_MT_DENSE_PUSH_READY_HINT (default off): queue-free pull-hint variant
  // of the push protocol. Reuses the exact 13,378-edge fan-in CSR the shadow
  // knob validated, but keeps the existing pull/lookahead worker control flow
  // and direct `entry->fn` body calls: producers push one per-task ready byte
  // (atomic uint8, release-stored by the final fan-in decrement) and workers
  // 1..N-1 replace only their readiness decisions (inline head test, tail
  // head test, candidate admission, tail fallback spin) with one acquire load
  // of that byte instead of loading each remote token wait list. Worker0
  // stays fully original pull for its pinned side effects. No queues, no
  // enqueue/dequeue, no dynamic worker assignment, no repeated full-table
  // scan. Mutually exclusive with the actual push protocol (v1/BITMAP/
  // DIRECT_TABLE) and every diagnostic knob to isolate the probe. Unset
  // keeps every emitted byte identical.
  const bool pushReadyHint = mtUseDensePushReadyHint();
  // GSIM_MT_DENSE_TOKEN_MASK_HINT (default off): counter-free readiness
  // hint that reuses only the owner-ready token layout. No fan-in pending
  // counters (the exact ready-hint's 13,378 fetch_sub RMWs per cycle cost
  // +44.23%), no ready bytes, no queues: each owner-ready release fetch_ors
  // one bit of the consumer's per-cycle uint16 token mask, and pool workers
  // 1..N-1 replace their remote-token wait conjunction with a single
  // acquire load comparing the mask against the generated expected value.
  // Worker0 stays fully original pull; body calls and token release stores
  // are unchanged. Requires owner-ready flags + lookahead, mutually
  // exclusive with every push/bitmap/direct-table/hint knob and every
  // diagnostic knob (shadow/PSCD/edge-timing/breakdown) so the probe stays
  // isolated. Unset keeps every emitted byte identical.
  const bool tokenMaskHint = mtUseDenseTokenMaskHint();
  bool denseBreakdownProfileCodegen = mtUseDenseBreakdownProfileCodegen();
  bool denseBreakdownWindowCodegen = denseBreakdownProfileCodegen && mtUseDenseBreakdownWindowCodegen();
  int denseBreakdownWindowWorker0MTaskCount = 0;
  if (denseBreakdownWindowCodegen) {
    for (int mtaskId = 0; mtaskId < nMTasks; mtaskId ++) {
      if (denseSchedule.mtaskThreadAssign[(size_t)mtaskId] == 0) denseBreakdownWindowWorker0MTaskCount ++;
    }
    Assert(denseBreakdownWindowWorker0MTaskCount <= 2267,
           "GSIM_MT_DENSE_BREAKDOWN window supports at most 2267 worker0 MTasks (got %d)",
           denseBreakdownWindowWorker0MTaskCount);
    Assert(threadCount >= 2,
           "GSIM_MT_DENSE_BREAKDOWN window requires at least 2 workers (got %d)", threadCount);
  }
  if (denseBreakdownProfileCodegen) {
    Assert(ownerReadyFlags,
           "GSIM_MT_DENSE_BREAKDOWN_PROFILE requires GSIM_MT_DENSE_OWNER_READY_FLAGS=1");
    const int denseBreakdownMaxWorkers =
        mtUseDenseBreakdownWindowLaBodyCodegen() ? 32 : 16;
    Assert(threadCount <= denseBreakdownMaxWorkers,
           "GSIM_MT_DENSE_BREAKDOWN_PROFILE supports at most %d workers (got %d)",
           denseBreakdownMaxWorkers, threadCount);
  }

  if (ownerReadyFlags) {
    Assert(xthreadDepsOnly,
           "GSIM_MT_DENSE_OWNER_READY_FLAGS requires GSIM_MT_DENSE_XTHREAD_DEPS_ONLY=1");
    Assert(transitiveReduceEdges,
           "GSIM_MT_DENSE_OWNER_READY_FLAGS experiment requires GSIM_MT_DENSE_TRANSITIVE_REDUCE_EDGES=1");
  }
  Assert(!pushShadow || ownerReadyFlags,
         "GSIM_MT_DENSE_PUSH_READY_SHADOW requires GSIM_MT_DENSE_OWNER_READY_FLAGS=1");
  Assert(!edgeTiming || ownerReadyFlags,
         "GSIM_MT_DENSE_EDGE_TIMING requires GSIM_MT_DENSE_OWNER_READY_FLAGS=1");
  Assert(!pscdBits || ownerReadyFlags,
         "GSIM_EMIT_PSCD_BITS requires GSIM_MT_DENSE_OWNER_READY_FLAGS=1");
  Assert(!pscdVerify || pscdBits,
         "GSIM_PSCD_VERIFY requires GSIM_EMIT_PSCD_BITS=1");
  Assert(!pushReady || ownerReadyFlags,
         "GSIM_MT_DENSE_PUSH_READY requires GSIM_MT_DENSE_OWNER_READY_FLAGS=1");
  Assert(!pushReadyDirectTableKnob || pushReady,
         "GSIM_MT_DENSE_PUSH_READY_DIRECT_TABLE requires GSIM_MT_DENSE_PUSH_READY=1");
  Assert(!(pushReady && pushShadow),
         "GSIM_MT_DENSE_PUSH_READY is mutually exclusive with GSIM_MT_DENSE_PUSH_READY_SHADOW");
  Assert(!(pushReady && pscdBits),
         "GSIM_MT_DENSE_PUSH_READY is mutually exclusive with GSIM_EMIT_PSCD_BITS");
  Assert(!pushReadyBitmapKnob || pushReady,
         "GSIM_MT_DENSE_PUSH_READY_BITMAP requires GSIM_MT_DENSE_PUSH_READY=1");
  Assert(!(pushReadyBitmap && pushReadyDirectTable),
         "GSIM_MT_DENSE_PUSH_READY_BITMAP is mutually exclusive with GSIM_MT_DENSE_PUSH_READY_DIRECT_TABLE");
  Assert(!(pushReady && edgeTiming),
         "GSIM_MT_DENSE_PUSH_READY is mutually exclusive with GSIM_MT_DENSE_EDGE_TIMING");
  Assert(!(pushReady && denseBreakdownProfileCodegen),
         "GSIM_MT_DENSE_PUSH_READY is mutually exclusive with GSIM_MT_DENSE_BREAKDOWN_PROFILE");
  Assert(!pushReadyHint || ownerReadyFlags,
         "GSIM_MT_DENSE_PUSH_READY_HINT requires GSIM_MT_DENSE_OWNER_READY_FLAGS=1");
  Assert(!(pushReadyHint && pushReady),
         "GSIM_MT_DENSE_PUSH_READY_HINT is mutually exclusive with GSIM_MT_DENSE_PUSH_READY");
  Assert(!(pushReadyHint && (pushReadyDirectTableKnob || pushReadyBitmapKnob)),
         "GSIM_MT_DENSE_PUSH_READY_HINT is mutually exclusive with GSIM_MT_DENSE_PUSH_READY_DIRECT_TABLE and GSIM_MT_DENSE_PUSH_READY_BITMAP");
  Assert(!(pushReadyHint && pushShadow),
         "GSIM_MT_DENSE_PUSH_READY_HINT is mutually exclusive with GSIM_MT_DENSE_PUSH_READY_SHADOW");
  Assert(!(pushReadyHint && pscdBits),
         "GSIM_MT_DENSE_PUSH_READY_HINT is mutually exclusive with GSIM_EMIT_PSCD_BITS");
  Assert(!(pushReadyHint && edgeTiming),
         "GSIM_MT_DENSE_PUSH_READY_HINT is mutually exclusive with GSIM_MT_DENSE_EDGE_TIMING");
  Assert(!tokenMaskHint || ownerReadyFlags,
         "GSIM_MT_DENSE_TOKEN_MASK_HINT requires GSIM_MT_DENSE_OWNER_READY_FLAGS=1");
  Assert(!(tokenMaskHint && pushReady),
         "GSIM_MT_DENSE_TOKEN_MASK_HINT is mutually exclusive with GSIM_MT_DENSE_PUSH_READY");
  Assert(!(tokenMaskHint && (pushReadyDirectTableKnob || pushReadyBitmapKnob)),
         "GSIM_MT_DENSE_TOKEN_MASK_HINT is mutually exclusive with GSIM_MT_DENSE_PUSH_READY_DIRECT_TABLE and GSIM_MT_DENSE_PUSH_READY_BITMAP");
  Assert(!(tokenMaskHint && pushReadyHint),
         "GSIM_MT_DENSE_TOKEN_MASK_HINT is mutually exclusive with GSIM_MT_DENSE_PUSH_READY_HINT");
  Assert(!(tokenMaskHint && pushShadow),
         "GSIM_MT_DENSE_TOKEN_MASK_HINT is mutually exclusive with GSIM_MT_DENSE_PUSH_READY_SHADOW");
  Assert(!(tokenMaskHint && pscdBits),
         "GSIM_MT_DENSE_TOKEN_MASK_HINT is mutually exclusive with GSIM_EMIT_PSCD_BITS");
  Assert(!(tokenMaskHint && edgeTiming),
         "GSIM_MT_DENSE_TOKEN_MASK_HINT is mutually exclusive with GSIM_MT_DENSE_EDGE_TIMING");
  Assert(!(tokenMaskHint && denseBreakdownProfileCodegen),
         "GSIM_MT_DENSE_TOKEN_MASK_HINT is mutually exclusive with GSIM_MT_DENSE_BREAKDOWN_PROFILE");
  Assert(!(pushReadyHint && denseBreakdownProfileCodegen),
         "GSIM_MT_DENSE_PUSH_READY_HINT is mutually exclusive with GSIM_MT_DENSE_BREAKDOWN_PROFILE");
  const std::chrono::steady_clock::time_point densePrologueBegin = std::chrono::steady_clock::now();
  std::vector<std::vector<int>> denseRuntimeSuccs;
  int transitiveElidedEdges = 0;
  MtDenseOwnerReadyLayout ownerReadyLayout;
  {
    EmitPhaseTimer proSuccTimer("Final.densePrologue.runtimeSuccs");
    denseRuntimeSuccs = mtBuildDenseRuntimeSuccs(
        denseSchedule.mtasks, denseSchedule.mtaskThreadAssign, xthreadDepsOnly);
    transitiveElidedEdges = transitiveReduceEdges
        ? mtReduceDenseRuntimeSuccsTransitive(denseRuntimeSuccs, denseSchedule.mtaskThreadAssign) : 0;
    if (ownerReadyFlags) {
      ownerReadyLayout = mtBuildDenseOwnerReadyLayout(
          denseRuntimeSuccs, denseSchedule.mtaskThreadAssign, threadCount);
    }
  }
  const int denseLookaheadWindow = mtDenseLookaheadWindow();
  const bool denseLookahead = denseLookaheadWindow > 0;
  const bool denseDuty = mtDenseDutyCodegen();
  const char* adaptiveScanEnv = std::getenv("GSIM_MT_DENSE_ADAPTIVE_SCAN");
  const bool adaptiveScan = adaptiveScanEnv && adaptiveScanEnv[0] && adaptiveScanEnv[0] != '0';
  Assert(!adaptiveScan || denseLookahead,
         "GSIM_MT_DENSE_ADAPTIVE_SCAN requires GSIM_MT_DENSE_LOOKAHEAD >= 1");
  Assert(!denseLookahead || ownerReadyFlags,
         "GSIM_MT_DENSE_LOOKAHEAD requires GSIM_MT_DENSE_OWNER_READY_FLAGS=1");
  Assert(!denseDuty || ownerReadyFlags,
         "GSIM_MT_DENSE_DUTY requires GSIM_MT_DENSE_OWNER_READY_FLAGS=1");
  Assert(!pushReady || denseLookahead,
         "GSIM_MT_DENSE_PUSH_READY requires GSIM_MT_DENSE_LOOKAHEAD >= 1");
  Assert(!pushReadyHint || denseLookahead,
         "GSIM_MT_DENSE_PUSH_READY_HINT requires GSIM_MT_DENSE_LOOKAHEAD >= 1");
  Assert(!tokenMaskHint || denseLookahead,
         "GSIM_MT_DENSE_TOKEN_MASK_HINT requires GSIM_MT_DENSE_LOOKAHEAD >= 1");
  const bool denseBreakdownWindowLaBodyCodegen =
      denseBreakdownWindowCodegen && mtUseDenseBreakdownWindowLaBodyCodegen();
  if (mtUseDenseBreakdownWindowLaBodyCodegen()) {
    Assert(denseBreakdownWindowCodegen,
           "GSIM_MT_DENSE_BREAKDOWN_WINDOW_LA_BODY requires GSIM_MT_DENSE_BREAKDOWN_PROFILE=1 with GSIM_MT_DENSE_BREAKDOWN_WINDOW_START and GSIM_MT_DENSE_BREAKDOWN_WINDOW_CYCLES");
    Assert(denseLookahead,
           "GSIM_MT_DENSE_BREAKDOWN_WINDOW_LA_BODY requires GSIM_MT_DENSE_LOOKAHEAD >= 1");
  }
  Assert(!denseLookahead || denseBreakdownWindowLaBodyCodegen
             || (!denseBreakdownProfileCodegen && !denseBreakdownWindowCodegen),
         "GSIM_MT_DENSE_LOOKAHEAD is incompatible with dense breakdown codegen");
  struct MtActivityCommitField { std::string name; uint64_t mask = 0; bool conservative = false; int shadowSlot = -1; int fanoutBegin = 0; int fanoutEnd = 0; };
  struct MtActivityInputField { std::string name; uint64_t mask = 0; bool conservative = false; int shadowSlot = -1; int fanoutBegin = 0; int fanoutEnd = 0; };
  std::vector<MtActivityInputField> activityInputFields;
  std::vector<std::vector<MtActivityCommitField>> activityCommitFields;
  std::vector<std::vector<int>> activitySuccFlags;
  std::vector<std::vector<int>> activitySuccLite;
  std::vector<std::vector<int>> activityNextEdges;
  std::vector<int> activityFanoutList;
  std::vector<char> activityIsCommitMTask;
  std::vector<char> activityAlwaysActive;
  std::vector<char> activityResetHandler;
  std::vector<int> activityRegionOf;
  std::vector<std::string> activityRegionNames;
  int activityShadowCount = 0;
  int activitySuccTotal = 0;
  int activityNextTotal = 0;
  int activityConservativeFields = 0;
  int activityScalarFields = 0;
  int activityAlwaysActiveCount = 0;
  // The PEG dump (GSIM_MT_DENSE_PEG_DUMP) needs the field read/write sets
  // computed in this block. The activity-only outputs remain unused locals.
  const bool pegDump = std::getenv("GSIM_MT_DENSE_PEG_DUMP") != nullptr;
  if (pegDump) {
    const int nM = nMTasks;
    activityAlwaysActive.assign((size_t)nM, 0);
    activityResetHandler.assign((size_t)nM, 0);
    // Region-level activity (r): group MTasks into subsystem regions; a region runs fully
    // when any member is activated. Fine-grained gating breaks intra-subsystem queue invariants
    // (yank/missQueue/ldu asserts from one-cycle-late comb signals); region granularity preserves
    // them because all producers+consumers in the region evaluate together. Region assignment by
    // top module path of member nodes.
    activityRegionOf.assign((size_t)nM, 0);
    std::map<std::string, int> regionIds;
    auto regionOfSuper = [&](SuperNode* super) -> std::string {
      for (Node* member : super->member) {
        const std::string& nm = member->name;
        if (nm.rfind("cpu__DOT__l_soc__DOT__core_with_l2__DOT__core__DOT__frontend", 0) == 0) return "frontend";
        if (nm.rfind("cpu__DOT__l_soc__DOT__core_with_l2__DOT__core__DOT__memBlock", 0) == 0) return "memblock";
        if (nm.rfind("cpu__DOT__l_soc__DOT__core_with_l2__DOT__l2top", 0) == 0) return "l2";
        if (nm.rfind("cpu__DOT__l_soc__DOT__chi_openllc", 0) == 0) return "linkmon";
        if (nm.rfind("cpu__DOT__l_simMMIO", 0) == 0) return "mmio";
      }
      return "other";
    };
    std::vector<std::set<std::string>> readFields((size_t)nM);
    std::vector<std::set<std::string>> dstReadFields((size_t)nM);
    std::vector<std::set<std::string>> commitFields((size_t)nM);
    std::vector<std::set<std::string>> nextWriteFields((size_t)nM);
    std::map<std::string, Node*> commitTargetNode;
    for (int m = 0; m < nM; m++) {
      const MtDenseMTask& mt = denseSchedule.mtasks[(size_t)m];
      for (int sccId : mt.sccIds) {
        if (sccId < 0 || sccId >= static_cast<int>(denseSchedule.sccs.size())) continue;
        const MtDenseScc& scc = denseSchedule.sccs[(size_t)sccId];
        for (int cppId : scc.cppIds) {
          auto superIter = cppId2Super.find(cppId);
          if (superIter == cppId2Super.end() || !superIter->second) continue;
          SuperNode* super = superIter->second;
          int dummyCost = 0;
          MtBoundaryInfo b = collectMtBoundaryInfo(super, dummyCost);
          if ((b.hasStateUpdate && b.hasAmbiguousStateTarget) || b.hasActivateAllPath) activityAlwaysActive[(size_t)m] = 1;
          if (super->superType == SUPER_ASYNC_RESET || b.hasAsyncReset) activityResetHandler[(size_t)m] = 1;
          if (activityRegionOf[(size_t)m] == 0) {
            std::string rn = regionOfSuper(super);
            auto rid = regionIds.find(rn);
            if (rid == regionIds.end()) rid = regionIds.emplace(rn, (int)regionIds.size()).first;
            activityRegionOf[(size_t)m] = rid->second + 1;
          }
          for (const std::string& r : b.rhsReadStateTargetNames) readFields[(size_t)m].insert(r);
          for (Node* member : super->member) {
            for (ExpTree* tree : member->assignTree) {
              MtActivityReads tr;
              mtActivityCollectFromTree(tree->getRoot(), tr);
              readFields[(size_t)m].insert(tr.src.begin(), tr.src.end());
              dstReadFields[(size_t)m].insert(tr.dst.begin(), tr.dst.end());
            }
            if (member->type == NODE_WRITER || member->type == NODE_READWRITER) {
              if (member->parent) commitFields[(size_t)m].insert(member->parent->name);
            }
          }
          if (b.stateSourceCommitCount > 0 || b.stateResetUpdateCount > 0) {
            for (const std::string& f : b.stateTargetNames) commitFields[(size_t)m].insert(f);
            for (Node* member : super->member) {
              if (!nodeHasStateUpdate(member)) continue;
              std::string tn;
              if (!stateTargetNameForNode(member, tn)) continue;
              Node* target = member->type == NODE_REG_SRC ? member
                           : (member->type == NODE_REG_DST ? member->getSrc() : member->getResetSrc());
              if (target) commitTargetNode[tn] = target;
            }
          }
          if (b.stateNextUpdateCount > 0) {
            for (const std::string& f : b.stateTargetNames) nextWriteFields[(size_t)m].insert(f);
          }
        }
      }
    }
    std::vector<char> activityElided((size_t)nM, 0);
    for (int m = 0; m < nM; m++) if (denseSchedule.mtaskThreadAssign[(size_t)m] < 0) activityElided[(size_t)m] = 1;
    // runtime successors redirected through elided MTasks (they never run, so their successors
    // would never receive a flag from them)
    activitySuccFlags.assign((size_t)nM, {});
    for (int m = 0; m < nM; m++) {
      if (activityElided[(size_t)m]) continue;
      std::vector<int> stack;
      std::vector<char> seen((size_t)nM, 0);
      for (int s : denseSchedule.mtasks[(size_t)m].succMTasks) if (s >= 0 && s < nM) stack.push_back(s);
      while (!stack.empty()) {
        int s = stack.back(); stack.pop_back();
        if (seen[(size_t)s]) continue; seen[(size_t)s] = 1;
        if (activityElided[(size_t)s]) {
          for (int s2 : denseSchedule.mtasks[(size_t)s].succMTasks) if (s2 >= 0 && s2 < nM && !seen[(size_t)s2]) stack.push_back(s2);
        } else {
          activitySuccFlags[(size_t)m].push_back(s);
        }
      }
      activitySuccTotal += (int)activitySuccFlags[(size_t)m].size();
    }
    // next-state edges: nextWriter[X] -> nextReader[X]. The dense DAG excludes $NEXT reads
    // (next-state objects are cross-cycle buffers: the producer at cycle N is read by the
    // commit at cycle N+1), so no DAG edge exists and the succ-flag propagation would never
    // reach commit MTasks (activity5: OH commit MTask 1018 has 0 DAG preds and starved).
    std::map<std::string, std::vector<int>> nextWriters;
    std::map<std::string, std::vector<int>> nextReaders;
    for (int m = 0; m < nM; m++) {
      if (activityElided[(size_t)m]) continue;
      for (const std::string& f : nextWriteFields[(size_t)m]) nextWriters[f].push_back(m);
      for (const std::string& f : dstReadFields[(size_t)m]) nextReaders[f].push_back(m);
      for (const std::string& f : commitFields[(size_t)m]) nextReaders[f].push_back(m);
    }
    // GSIM_MT_DENSE_PEG_DUMP=<prefix>: dump the precedence event graph for recurrence
    // (max-cycle-ratio) analysis: within-cycle dependency edges (dist=0), cross-cycle
    // register-feedback edges writer->reader (dist=1), mcost per MTask. Format matches
    // the mcr tooling (int32 u,v,dist triples; double costs).
    if (const char* pegPath = std::getenv("GSIM_MT_DENSE_PEG_DUMP")) {
      std::string edgePath = std::string(pegPath) + "-edges.bin";
      std::string costPath = std::string(pegPath) + "-cost.bin";
      FILE* pe = std::fopen(edgePath.c_str(), "wb");
      FILE* pc = std::fopen(costPath.c_str(), "wb");
      Assert(pe && pc, "cannot open PEG dump %s", pegPath);
      int64_t depEdges = 0, wrapEdges = 0;
      for (int m = 0; m < nM; m++) {
        if (activityElided[(size_t)m]) continue;
        for (int succ : denseSchedule.mtasks[(size_t)m].succMTasks) {
          if (succ < 0 || succ >= nM || activityElided[(size_t)succ]) continue;
          int32_t t[3] = {m, succ, 0};
          std::fwrite(t, 4, 3, pe);
          depEdges ++;
        }
      }
      for (const auto& kv : nextWriters) {
        auto it = nextReaders.find(kv.first);
        if (it == nextReaders.end()) continue;
        for (int w : kv.second) {
          for (int r : it->second) {
            if (r == w) continue;
            int32_t t[3] = {w, r, 1};
            std::fwrite(t, 4, 3, pe);
            wrapEdges ++;
          }
        }
      }
      for (int m = 0; m < nM; m++) {
        const MtDenseMTask& mt = denseSchedule.mtasks[(size_t)m];
        double c = activityElided[(size_t)m] ? 0.0 : (double)(mt.schedCost > 0 ? mt.schedCost : mt.staticCost);
        std::fwrite(&c, 8, 1, pc);
      }
      std::fclose(pe);
      std::fclose(pc);
      fprintf(stderr, "[mt-dense-peg] dumped %d nodes dep=%lld wrap=%lld to %s-{edges,cost}.bin\n",
              nM, (long long)depEdges, (long long)wrapEdges, pegPath);
    }
    // Report-only per-MTask provenance for offline witness attribution: the
    // dist=1 PEG edge set is exactly nextWriteFields[u] x (dstReadFields |
    // commitFields)[v] per shared field, so exporting these sets per MTask
    // lets the mcr5 witness be joined back to fields and module paths without
    // another generation. Default-off: only under GSIM_MT_DENSE_PEG_DUMP.
    if (const char* provPegPath = std::getenv("GSIM_MT_DENSE_PEG_DUMP")) {
      std::string provPath = std::string(provPegPath) + "-provenance.json";
      FILE* pj = std::fopen(provPath.c_str(), "w");
      Assert(pj != nullptr, "cannot open PEG provenance dump %s", provPath.c_str());
      std::vector<std::string> regionNameById(regionIds.size() + 1, "other");
      for (const auto& rn : regionIds) regionNameById[(size_t)rn.second + 1] = rn.first;
      auto dumpProvStrSet = [&](FILE* out, const std::set<std::string>& s) {
        fprintf(out, "[");
        bool first = true;
        for (const std::string& x : s) {
          fprintf(out, "%s\"%s\"", first ? "" : ",", jsonEscape(x).c_str());
          first = false;
        }
        fprintf(out, "]");
      };
      auto dumpProvIntVec = [&](FILE* out, const std::vector<int>& v) {
        fprintf(out, "[");
        bool first = true;
        for (int x : v) { fprintf(out, "%s%d", first ? "" : ",", x); first = false; }
        fprintf(out, "]");
      };
      const size_t kMaxNodeNames = 256;
      fprintf(pj, "{\n\"format\": \"gsim.mt-dense-peg-provenance.v1\",\n\"mtask_count\": %d,\n\"mtasks\": [\n", nM);
      for (int m = 0; m < nM; m++) {
        const MtDenseMTask& mt = denseSchedule.mtasks[(size_t)m];
        int worker = m < static_cast<int>(denseSchedule.mtaskThreadAssign.size())
                       ? denseSchedule.mtaskThreadAssign[(size_t)m] : -1;
        std::vector<int> sccIdsOut;
        std::set<int> cppIdSet;
        std::set<std::string> nodeNames;
        bool namesTruncated = false;
        std::set<std::string> modulePaths;
        for (int sccId : mt.sccIds) {
          if (sccId < 0 || sccId >= static_cast<int>(denseSchedule.sccs.size())) continue;
          sccIdsOut.push_back(sccId);
          for (int cppId : denseSchedule.sccs[(size_t)sccId].cppIds) {
            cppIdSet.insert(cppId);
            auto superIter = cppId2Super.find(cppId);
            if (superIter == cppId2Super.end() || !superIter->second) continue;
            for (Node* member : superIter->second->member) {
              const std::string& nm = member->name;
              if (nm.empty()) continue;
              if (nodeNames.size() < kMaxNodeNames) nodeNames.insert(nm);
              else namesTruncated = true;
              size_t cut = 0;
              int segs = 0;
              while (segs < 7) {
                size_t pos = nm.find("__DOT__", cut);
                if (pos == std::string::npos) { cut = nm.size(); break; }
                cut = pos + 7; segs++;
              }
              modulePaths.insert(nm.substr(0, cut < nm.size() ? cut : nm.size()));
            }
          }
        }
        std::vector<int> cppIdsOut(cppIdSet.begin(), cppIdSet.end());
        fprintf(pj, "  {\"id\": %d, \"worker\": %d, \"sched_cost\": %d, \"static_cost\": %d, \"task_count\": %d,\n",
                m, worker, mt.schedCost, mt.staticCost, mt.taskCount);
        fprintf(pj, "   \"region\": \"%s\", \"always_active\": %s, \"reset_handler\": %s,\n",
                jsonEscape(regionNameById[(size_t)activityRegionOf[(size_t)m]]).c_str(),
                activityAlwaysActive[(size_t)m] ? "true" : "false",
                activityResetHandler[(size_t)m] ? "true" : "false");
        fprintf(pj, "   \"scc_ids\": ");
        dumpProvIntVec(pj, sccIdsOut);
        fprintf(pj, ",\n   \"cpp_ids\": ");
        dumpProvIntVec(pj, cppIdsOut);
        fprintf(pj, ",\n   \"node_names_truncated\": %s,\n   \"node_names\": ", namesTruncated ? "true" : "false");
        dumpProvStrSet(pj, nodeNames);
        fprintf(pj, ",\n   \"module_paths\": ");
        dumpProvStrSet(pj, modulePaths);
        fprintf(pj, ",\n   \"read_fields\": ");
        dumpProvStrSet(pj, readFields[(size_t)m]);
        fprintf(pj, ",\n   \"dst_read_fields\": ");
        dumpProvStrSet(pj, dstReadFields[(size_t)m]);
        fprintf(pj, ",\n   \"commit_fields\": ");
        dumpProvStrSet(pj, commitFields[(size_t)m]);
        fprintf(pj, ",\n   \"next_write_fields\": ");
        dumpProvStrSet(pj, nextWriteFields[(size_t)m]);
        fprintf(pj, "}%s\n", m + 1 == nM ? "" : ",");
      }
      fprintf(pj, "]}\n");
      std::fclose(pj);
      fprintf(stderr, "[mt-dense-peg] wrote per-MTask provenance to %s\n", provPath.c_str());
    }
    activityNextEdges.assign((size_t)nM, {});
    for (int m = 0; m < nM; m++) {
      if (activityElided[(size_t)m]) continue;
      if (nextWriteFields[(size_t)m].empty()) continue;
      std::set<int> targets;
      for (const std::string& f : nextWriteFields[(size_t)m]) {
        auto it = nextReaders.find(f);
        if (it == nextReaders.end()) continue;
        for (int r : it->second) if (r != m) targets.insert(r);
      }
      activityNextEdges[(size_t)m].assign(targets.begin(), targets.end());
      activityNextTotal += (int)activityNextEdges[(size_t)m].size();
    }
    std::map<std::string, std::vector<int>> fieldReaders;
    for (int m = 0; m < nM; m++) {
      if (activityElided[(size_t)m]) continue;
      for (const std::string& r : readFields[(size_t)m]) fieldReaders[r].push_back(m);
    }
    // harness-driven input watcher: io_* input fields change exogenously (no commit fires), so
    // their readers must be activated by a shadow-compare pass in stepDense before each cycle.
    activityInputFields.clear();
    {
      std::set<std::string> inputNames;
      for (int m = 0; m < nM; m++) {
        const MtDenseMTask& mt = denseSchedule.mtasks[(size_t)m];
        for (int sccId : mt.sccIds) {
          if (sccId < 0 || sccId >= static_cast<int>(denseSchedule.sccs.size())) continue;
          for (int cppId : denseSchedule.sccs[(size_t)sccId].cppIds) {
            auto superIter = cppId2Super.find(cppId);
            if (superIter == cppId2Super.end() || !superIter->second) continue;
            for (Node* member : superIter->second->member) {
              if (member->type == NODE_INP && !member->name.empty()) inputNames.insert(member->name);
            }
          }
        }
      }
      for (const std::string& f : inputNames) {
        MtActivityInputField inf; inf.name = f;
        Node* node = nullptr;
        for (int m = 0; m < nM && node == nullptr; m++) {
          const MtDenseMTask& mt = denseSchedule.mtasks[(size_t)m];
          for (int sccId : mt.sccIds) {
            if (sccId < 0 || sccId >= static_cast<int>(denseSchedule.sccs.size())) continue;
            for (int cppId : denseSchedule.sccs[(size_t)sccId].cppIds) {
              auto superIter = cppId2Super.find(cppId);
              if (superIter == cppId2Super.end() || !superIter->second) continue;
              for (Node* member : superIter->second->member) if (member->type == NODE_INP && member->name == f) { node = member; break; }
              if (node) break;
            }
            if (node) break;
          }
        }
        bool scalar = node != nullptr && !node->isArray() && node->width <= 64;
        if (scalar) {
          inf.mask = node->width >= 64 ? ~0ULL : ((1ULL << node->width) - 1);
          inf.shadowSlot = activityShadowCount++;
        } else {
          inf.conservative = true;
        }
        inf.fanoutBegin = (int)activityFanoutList.size();
        auto readerIter = fieldReaders.find(f);
        if (readerIter != fieldReaders.end()) for (int r : readerIter->second) activityFanoutList.push_back(r);
        inf.fanoutEnd = (int)activityFanoutList.size();
        activityInputFields.push_back(inf);
      }
    }
    activityCommitFields.assign((size_t)nM, {});
    for (int m = 0; m < nM; m++) {
      if (activityElided[(size_t)m]) continue;
      for (const std::string& f : commitFields[(size_t)m]) {
        MtActivityCommitField cf; cf.name = f;
        Node* target = nullptr;
        auto nodeIter = commitTargetNode.find(f);
        if (nodeIter != commitTargetNode.end()) target = nodeIter->second;
        bool scalar = target != nullptr && target->type != NODE_MEMORY && !target->isArray() && target->width <= 64;
        if (scalar) {
          cf.mask = target->width >= 64 ? ~0ULL : ((1ULL << target->width) - 1);
          cf.shadowSlot = activityShadowCount++;
          activityScalarFields++;
        } else {
          cf.conservative = true;
          activityConservativeFields++;
        }
        cf.fanoutBegin = (int)activityFanoutList.size();
        auto readerIter = fieldReaders.find(f);
        if (readerIter != fieldReaders.end()) for (int r : readerIter->second) activityFanoutList.push_back(r);
        cf.fanoutEnd = (int)activityFanoutList.size();
        activityCommitFields[(size_t)m].push_back(cf);
      }
    }
    // succLite for commit MTasks: successors NOT covered by the commit fanout (i.e. consumers of
    // non-state wire outputs). Commit MTasks fire only compare+fanout (tight) plus succLite;
    // without this split the conservative succ store keeps the whole DAG active forever (no decay).
    activitySuccLite.assign((size_t)nM, {});
    for (int m = 0; m < nM; m++) {
      if (activityElided[(size_t)m] || activityCommitFields[(size_t)m].empty()) continue;
      std::set<int> covered;
      for (const MtActivityCommitField& cf : activityCommitFields[(size_t)m])
        for (int j = cf.fanoutBegin; j < cf.fanoutEnd; j++) covered.insert(activityFanoutList[(size_t)j]);
      for (int s : activitySuccFlags[(size_t)m])
        if (!covered.count(s)) activitySuccLite[(size_t)m].push_back(s);
    }
    int activityCommitMTasks = 0;
    activityIsCommitMTask.assign((size_t)nMTasks, 0);
    for (int m = 0; m < nMTasks; m++) {
      activityIsCommitMTask[(size_t)m] = !activityCommitFields[(size_t)m].empty() ? 1 : 0;
      activityCommitMTasks += activityIsCommitMTask[(size_t)m];
      activityAlwaysActiveCount += activityAlwaysActive[(size_t)m] ? 1 : 0;
    }
    int activityResetHandlerCount = 0;
    for (int m = 0; m < nMTasks; m++) activityResetHandlerCount += activityResetHandler[(size_t)m] ? 1 : 0;
    activityRegionNames.assign(regionIds.size() + 1, "other");
    for (const auto& kv : regionIds) activityRegionNames[(size_t)kv.second + 1] = kv.first;
    // The CHI link layer does cycle-precise credit accounting that is hostile to any staleness;
    // keep it always-active (small share of the model). Same for l2/mmio: their queue invariants
    // span the link<->DUT boundary and fail under any gating (yank/missQueue/L-credit asserts).
    // memblock (LSU/dcache/missQueue) is the same class of cycle-precise queue accounting.
    // frontend/other stay fine-grained sparse.
    for (int m = 0; m < nMTasks; m++) {
      const std::string& rn = activityRegionNames[(size_t)activityRegionOf[(size_t)m]];
      if (rn == "linkmon" || rn == "l2" || rn == "mmio" || rn == "memblock") activityAlwaysActive[(size_t)m] = 1;
    }
    std::map<std::string, int> regionMTaskCounts;
    for (int m = 0; m < nMTasks; m++) regionMTaskCounts[activityRegionNames[(size_t)activityRegionOf[(size_t)m]]]++;
    fprintf(stderr, "[mt-dense-activity] mtasks=%d commit_mtasks=%d always_active=%d reset_handlers=%d input_fields=%zu scalar_fields=%d conservative_fields=%d fanout=%zu succ_flags=%d next_edges=%d shadow=%d regions=",
            nMTasks, activityCommitMTasks, activityAlwaysActiveCount, activityResetHandlerCount, activityInputFields.size(),
            activityScalarFields, activityConservativeFields, activityFanoutList.size(), activitySuccTotal, activityNextTotal, activityShadowCount);
    for (const auto& kv : regionMTaskCounts) fprintf(stderr, "%s:%d ", kv.first.c_str(), kv.second);
    fprintf(stderr, "\n");
  }
  std::vector<int> denseDispatchWorkerCounts;
  std::vector<uint32_t> denseDispatchWaitBegin, denseDispatchWaitEnd;
  std::vector<uint32_t> denseDispatchStoreBegin, denseDispatchStoreEnd;
  uint32_t denseDispatchWaitTotal = 0;
  const std::chrono::steady_clock::time_point denseProFieldSetsBegin = std::chrono::steady_clock::now();
  if (emitPhaseTimingEnabled()) {
    fprintf(stderr, "[emit-phase] Final.densePrologue.fieldSets = %ld ms\n",
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - denseProFieldSetsBegin).count());
  }
  MtDenseBreakdownWindowWaitLayout denseBreakdownWindowWaitLayout;
  if (denseBreakdownWindowCodegen) {
    denseBreakdownWindowWaitLayout = mtBuildDenseBreakdownWindowWaitLayout(
        ownerReadyLayout, denseSchedule.mtaskThreadAssign, threadCount);
    Assert(denseBreakdownWindowWaitLayout.totalWaitRecords <= 65536,
           "GSIM_MT_DENSE_BREAKDOWN window supports at most 65536 ready waits per cycle (got %d)",
           denseBreakdownWindowWaitLayout.totalWaitRecords);
  }
  MtDenseBreakdownWindowAllOwnerLayout denseBreakdownWindowAllOwnerLayout;
  if (denseBreakdownWindowCodegen) {
    denseBreakdownWindowAllOwnerLayout = mtBuildDenseBreakdownWindowAllOwnerLayout(
        denseSchedule.mtaskThreadAssign, threadCount);
    Assert(denseBreakdownWindowAllOwnerLayout.recordCount >= nMTasks
           && denseBreakdownWindowAllOwnerLayout.recordCount <= nMTasks + 7 * threadCount,
           "dense breakdown all-owner physical record count %d is invalid for %d MTasks",
           denseBreakdownWindowAllOwnerLayout.recordCount, nMTasks);
  }
  const std::chrono::steady_clock::time_point denseProDepsBegin = std::chrono::steady_clock::now();
  std::vector<uint32_t> denseRuntimeDepCounts((size_t)nMTasks, 0);
  int totalSuccs = 0;
  for (int mtaskId = 0; mtaskId < nMTasks; mtaskId++) {
    for (int succ : denseRuntimeSuccs[(size_t)mtaskId]) {
      if (succ < 0 || succ >= nMTasks) continue;
      denseRuntimeDepCounts[(size_t)succ] ++;
      totalSuccs ++;
    }
  }
  if (totalSuccs == 0) totalSuccs = 1;
  if (emitPhaseTimingEnabled()) {
    fprintf(stderr, "[emit-phase] Final.densePrologue.depCounts = %ld ms\n",
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - denseProDepsBegin).count());
  }
  std::vector<uint32_t> denseLookaheadLocalBegin, denseLookaheadLocalEnd,
      denseLookaheadLocalPrereqs;
  std::vector<uint8_t> denseLookaheadLocalPrereqKinds;
  std::vector<std::vector<int>> denseLookaheadDagOnlySuccs;
  int denseLookaheadDagOnlyKeptEdges = 0;
  int denseLookaheadCrossEdgeCount = 0;
  int denseLookaheadSameWorkerPredCount = 0;
  int denseLookaheadPublisherConstrainedEntryCount = 0;
  int denseLookaheadPublisherSiblingPositionCount = 0;
  const std::chrono::steady_clock::time_point denseProLookaheadBegin = std::chrono::steady_clock::now();
  if (denseLookahead) {
    // The B1 token layout reduces the complete MTask DAG without the synthetic
    // fixed-worker chains used by the strict Step-A token layout.
    denseLookaheadDagOnlySuccs = mtBuildDenseRuntimeSuccs(
        denseSchedule.mtasks, denseSchedule.mtaskThreadAssign, false);
    mtReduceDenseRuntimeSuccsTransitive(
        denseLookaheadDagOnlySuccs, denseSchedule.mtaskThreadAssign, false);
    denseLookaheadDagOnlyKeptEdges = mtDenseRuntimeEdgeCount(denseLookaheadDagOnlySuccs);
    std::vector<std::vector<int>> denseLookaheadCrossWorkerSuccs((size_t)nMTasks);
    for (int pred = 0; pred < nMTasks; ++pred) {
      for (int succ : denseLookaheadDagOnlySuccs[(size_t)pred]) {
        if (denseSchedule.mtaskThreadAssign[(size_t)pred] != denseSchedule.mtaskThreadAssign[(size_t)succ])
          denseLookaheadCrossWorkerSuccs[(size_t)pred].push_back(succ);
      }
    }
    ownerReadyLayout = mtBuildDenseOwnerReadyLayout(
        denseLookaheadCrossWorkerSuccs, denseSchedule.mtaskThreadAssign, threadCount);

    std::vector<int> positionByMTask((size_t)nMTasks, -1);
    for (int worker = 0; worker < threadCount; ++worker) {
      int position = 0;
      for (int mtaskId = 0; mtaskId < nMTasks; ++mtaskId) {
        if (denseSchedule.mtaskThreadAssign[(size_t)mtaskId] == worker)
          positionByMTask[(size_t)mtaskId] = position++;
      }
    }
    std::vector<std::set<uint32_t>> sameWorkerPredsByMTask((size_t)nMTasks);
    std::vector<std::set<uint32_t>> publisherSiblingsByMTask((size_t)nMTasks);
    std::vector<char> publisherConstrained((size_t)nMTasks, 0);
    for (int pred = 0; pred < nMTasks; ++pred) {
      for (int succ : denseLookaheadDagOnlySuccs[(size_t)pred]) {
        Assert(succ > pred && succ < nMTasks,
               "lookahead DAG-only reduction produced invalid edge %d -> %d", pred, succ);
        const int predOwner = denseSchedule.mtaskThreadAssign[(size_t)pred];
        const int succOwner = denseSchedule.mtaskThreadAssign[(size_t)succ];
        if (predOwner == succOwner) {
          Assert(positionByMTask[(size_t)pred] >= 0
                     && positionByMTask[(size_t)pred] < positionByMTask[(size_t)succ],
                 "lookahead same-worker predecessor %d is not before successor %d", pred, succ);
          sameWorkerPredsByMTask[(size_t)succ].insert(
              static_cast<uint32_t>(positionByMTask[(size_t)pred]));
        } else {
          ++denseLookaheadCrossEdgeCount;
        }
      }
    }
    for (int token = 0; token < ownerReadyLayout.tokenCount; ++token) {
      const MtDenseOwnerReadyTokenProvenance& provenance =
          ownerReadyLayout.tokenProvenanceByLogicalToken[(size_t)token];
      const std::vector<int>& sources =
          ownerReadyLayout.sourceMTasksByLogicalToken[(size_t)token];
      Assert(!sources.empty() && sources.back() == provenance.producerMTask,
             "lookahead token %d has invalid publisher membership", token);
      if (sources.size() > 1) publisherConstrained[(size_t)provenance.producerMTask] = 1;
      for (int source : sources) {
        if (source == provenance.producerMTask) continue;
        Assert(denseSchedule.mtaskThreadAssign[(size_t)source] == provenance.producerOwner
                   && positionByMTask[(size_t)source] < positionByMTask[(size_t)provenance.producerMTask],
               "lookahead publisher %d does not follow sibling %d", provenance.producerMTask, source);
        publisherSiblingsByMTask[(size_t)provenance.producerMTask].insert(
            static_cast<uint32_t>(positionByMTask[(size_t)source]));
      }
    }
    denseLookaheadLocalBegin.assign((size_t)nMTasks, 0);
    denseLookaheadLocalEnd.assign((size_t)nMTasks, 0);
    for (int mtaskId = 0; mtaskId < nMTasks; ++mtaskId) {
      const std::set<uint32_t>& direct = sameWorkerPredsByMTask[(size_t)mtaskId];
      const std::set<uint32_t>& siblings = publisherSiblingsByMTask[(size_t)mtaskId];
      std::set<uint32_t> local = direct;
      local.insert(siblings.begin(), siblings.end());
      denseLookaheadLocalBegin[(size_t)mtaskId] =
          static_cast<uint32_t>(denseLookaheadLocalPrereqs.size());
      for (uint32_t position : local) {
        Assert(position < static_cast<uint32_t>(positionByMTask[(size_t)mtaskId]),
               "lookahead local prerequisite position %u is not before MTask %d", position, mtaskId);
        denseLookaheadLocalPrereqs.push_back(position);
        denseLookaheadLocalPrereqKinds.push_back(
            static_cast<uint8_t>((direct.count(position) ? 1u : 0u)
                                 | (siblings.count(position) ? 2u : 0u)));
      }
      denseLookaheadLocalEnd[(size_t)mtaskId] =
          static_cast<uint32_t>(denseLookaheadLocalPrereqs.size());
      denseLookaheadSameWorkerPredCount += static_cast<int>(direct.size());
      denseLookaheadPublisherSiblingPositionCount += static_cast<int>(siblings.size());
      denseLookaheadPublisherConstrainedEntryCount += publisherConstrained[(size_t)mtaskId] ? 1 : 0;
    }
    denseDispatchWorkerCounts.assign((size_t)threadCount, 0);
    denseDispatchWaitBegin.assign((size_t)nMTasks, 0);
    denseDispatchWaitEnd.assign((size_t)nMTasks, 0);
    denseDispatchStoreBegin.assign((size_t)nMTasks, 0);
    denseDispatchStoreEnd.assign((size_t)nMTasks, 0);
    uint32_t storeOff = 0;
    for (int mtaskId = 0; mtaskId < nMTasks; ++mtaskId) {
      denseDispatchStoreBegin[(size_t)mtaskId] = storeOff;
      storeOff += static_cast<uint32_t>(ownerReadyLayout.storeSlotsByMTask[(size_t)mtaskId].size());
      denseDispatchStoreEnd[(size_t)mtaskId] = storeOff;
    }
    denseDispatchWaitTotal = 0;
    for (int worker = 0; worker < threadCount; ++worker) {
      for (int mtaskId = 0; mtaskId < nMTasks; ++mtaskId) {
        if (denseSchedule.mtaskThreadAssign[(size_t)mtaskId] != worker) continue;
        ++denseDispatchWorkerCounts[(size_t)worker];
        denseDispatchWaitBegin[(size_t)mtaskId] = denseDispatchWaitTotal;
        denseDispatchWaitTotal += static_cast<uint32_t>(ownerReadyLayout.waitSlotsByMTask[(size_t)mtaskId].size());
        denseDispatchWaitEnd[(size_t)mtaskId] = denseDispatchWaitTotal;
      }
    }
  }
  if (emitPhaseTimingEnabled()) {
    fprintf(stderr, "[emit-phase] Final.densePrologue.lookahead = %ld ms\n",
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - denseProLookaheadBegin).count());
  }
  if (ownerReadyFlags) {
    fprintf(stderr,
            "[mt-dense-owner-ready] mtasks=%d edges=%d tokens=%d slots=%d padding=%d banks=%d threads=%d\n",
            nMTasks, ownerReadyLayout.edgeCount, ownerReadyLayout.tokenCount,
            ownerReadyLayout.physicalSlotCount,
            ownerReadyLayout.physicalSlotCount - ownerReadyLayout.tokenCount,
            ownerReadyLayout.pairBankCount, threadCount);
  }
  if (denseLookahead) {
    fprintf(stderr,
            "[mt-dense-lookahead] window=%d dag_only_kept_edges=%d cross_edges=%d same_worker_preds=%d groups=%d wait_entries=%d store_entries=%d publisher_constrained_entries=%d sibling_positions=%d\n",
            denseLookaheadWindow, denseLookaheadDagOnlyKeptEdges, denseLookaheadCrossEdgeCount,
            denseLookaheadSameWorkerPredCount, ownerReadyLayout.tokenCount,
            ownerReadyLayout.tokenCount, ownerReadyLayout.tokenCount,
            denseLookaheadPublisherConstrainedEntryCount,
            denseLookaheadPublisherSiblingPositionCount);
  }
  // ---- Push edge census (GSIM_MT_DENSE_PUSH_READY_SHADOW | GSIM_MT_DENSE_PUSH_READY, default off) ----
  // Derives the exact producer->dependent edge set a push protocol would have
  // to transport, from the same three dependency classes the pull executor
  // consumes: (1) owner-ready tokens - one edge from each logical token's
  // publisher to its consumer (the store the consumer's readiness actually
  // waits for); (2) direct same-worker predecessors - same-owner edges of the
  // dag-only transitive-reduced graph the fixed dispatch order and the
  // lookahead local-prereq check enforce; (3) publisher-sibling constraints -
  // every non-final token source gates its token's publisher (the sibling half
  // of the lookahead local prereqs). Edges are deduplicated per
  // (producer, dependent) pair: the direct and sibling classes overlap on real
  // DAG edges, exactly as the lookahead localPrereq set-union dedups them.
  // The runtime shadow re-arms pending[dependent] to this fan-in every cycle
  // (plain relaxed store before dispatch; each cycle is globally barriered by
  // the worker-pool join, so no zero-epoch parity is needed) and decrements
  // once per edge at producer completion, BEFORE the token release stores.
  // GSIM_MT_DENSE_PUSH_READY consumes this same census as its real protocol
  // (pending counters + owner-queue enqueue), so the two knobs share it.
  std::vector<std::vector<int>> pushShadowDagOnlySuccs;
  std::vector<uint32_t> pushShadowFanin;
  std::vector<uint32_t> pushShadowNotifyOffsets;
  std::vector<uint32_t> pushShadowNotifyList;
  int pushShadowTokenEdges = 0, pushShadowDirectEdges = 0, pushShadowSiblingEdges = 0;
  int pushShadowTokenEdgesKept = 0, pushShadowDirectEdgesKept = 0, pushShadowSiblingEdgesKept = 0;
  int pushShadowMaxFanin = 0, pushShadowZeroFaninMTasks = 0;
  if (pushShadow || pushReady || pushReadyHint) {
    if (denseLookahead) {
      pushShadowDagOnlySuccs = denseLookaheadDagOnlySuccs;
    } else {
      // Strict owner-ready mode: the runtime layout was built from the
      // xthread-only reduced succs, but the full-model census still needs the
      // same-worker direct edges, so rebuild the dag-only reduction locally
      // (generation-time analysis only; nothing outside the census reads it).
      pushShadowDagOnlySuccs = mtBuildDenseRuntimeSuccs(
          denseSchedule.mtasks, denseSchedule.mtaskThreadAssign, false);
      mtReduceDenseRuntimeSuccsTransitive(
          pushShadowDagOnlySuccs, denseSchedule.mtaskThreadAssign, false);
    }
    std::vector<std::set<int>> pushShadowProducersByDependent((size_t)nMTasks);
    auto pushShadowAddEdge = [&](int producer, int dependent, int& keptCounter) {
      Assert(producer >= 0 && producer < nMTasks && dependent >= 0 && dependent < nMTasks,
             "push-shadow edge %d -> %d is out of range for %d MTasks",
             producer, dependent, nMTasks);
      Assert(producer < dependent,
             "push-shadow edge %d -> %d is not forward in fixed MTask order",
             producer, dependent);
      if (pushShadowProducersByDependent[(size_t)dependent].insert(producer).second)
        keptCounter++;
    };
    for (int token = 0; token < ownerReadyLayout.tokenCount; ++token) {
      const MtDenseOwnerReadyTokenProvenance& pushShadowProvenance =
          ownerReadyLayout.tokenProvenanceByLogicalToken[(size_t)token];
      pushShadowAddEdge(pushShadowProvenance.producerMTask, pushShadowProvenance.consumerMTask,
                        pushShadowTokenEdgesKept);
      pushShadowTokenEdges++;
      for (int source : ownerReadyLayout.sourceMTasksByLogicalToken[(size_t)token]) {
        if (source == pushShadowProvenance.producerMTask) continue;
        const int siblingKeptBefore = pushShadowSiblingEdgesKept;
        pushShadowAddEdge(source, pushShadowProvenance.producerMTask,
                          pushShadowSiblingEdgesKept);
        if (pushShadowSiblingEdgesKept > siblingKeptBefore) pushShadowSiblingEdges++;
      }
    }
    for (int pred = 0; pred < nMTasks; ++pred) {
      for (int succ : pushShadowDagOnlySuccs[(size_t)pred]) {
        Assert(succ > pred && succ < nMTasks,
               "push-shadow dag-only edge %d -> %d is invalid", pred, succ);
        if (denseSchedule.mtaskThreadAssign[(size_t)pred]
            != denseSchedule.mtaskThreadAssign[(size_t)succ]) continue;
        pushShadowAddEdge(pred, succ, pushShadowDirectEdgesKept);
        pushShadowDirectEdges++;
      }
    }
    std::vector<std::vector<int>> pushShadowDependentsByProducer((size_t)nMTasks);
    pushShadowFanin.assign((size_t)nMTasks, 0);
    size_t pushShadowEdgeTotal = 0;
    for (int dependent = 0; dependent < nMTasks; ++dependent) {
      const size_t pushShadowPredCount = pushShadowProducersByDependent[(size_t)dependent].size();
      pushShadowFanin[(size_t)dependent] = static_cast<uint32_t>(pushShadowPredCount);
      if (pushShadowPredCount == 0) pushShadowZeroFaninMTasks++;
      if (static_cast<int>(pushShadowPredCount) > pushShadowMaxFanin)
        pushShadowMaxFanin = static_cast<int>(pushShadowPredCount);
      for (int producer : pushShadowProducersByDependent[(size_t)dependent])
        pushShadowDependentsByProducer[(size_t)producer].push_back(dependent);
      pushShadowEdgeTotal += pushShadowPredCount;
    }
    pushShadowNotifyOffsets.assign((size_t)nMTasks + 1, 0);
    uint32_t pushShadowNotifyCursor = 0;
    for (int producer = 0; producer < nMTasks; ++producer) {
      pushShadowNotifyOffsets[(size_t)producer] = pushShadowNotifyCursor;
      pushShadowNotifyCursor += static_cast<uint32_t>(pushShadowDependentsByProducer[(size_t)producer].size());
      for (int dependent : pushShadowDependentsByProducer[(size_t)producer])
        pushShadowNotifyList.push_back(static_cast<uint32_t>(dependent));
    }
    pushShadowNotifyOffsets[(size_t)nMTasks] = pushShadowNotifyCursor;
    Assert(pushShadowNotifyCursor == pushShadowEdgeTotal
               && pushShadowNotifyList.size() == pushShadowEdgeTotal,
           "push-shadow CSR notify table covers %zu of %zu fan-in edges",
           pushShadowNotifyList.size(), pushShadowEdgeTotal);
    Assert(pushShadowTokenEdgesKept + pushShadowDirectEdgesKept + pushShadowSiblingEdgesKept
               == static_cast<int>(pushShadowEdgeTotal),
           "push-shadow kept-edge class counts (%d + %d + %d) disagree with deduplicated edge total %zu",
           pushShadowTokenEdgesKept, pushShadowDirectEdgesKept, pushShadowSiblingEdgesKept,
           pushShadowEdgeTotal);
    if (denseLookahead) {
      Assert(pushShadowDirectEdges == denseLookaheadSameWorkerPredCount,
             "push-shadow direct-local census %d disagrees with lookahead same-worker census %d",
             pushShadowDirectEdges, denseLookaheadSameWorkerPredCount);
      Assert(pushShadowSiblingEdges == denseLookaheadPublisherSiblingPositionCount,
             "push-shadow sibling census %d disagrees with lookahead sibling census %d",
             pushShadowSiblingEdges, denseLookaheadPublisherSiblingPositionCount);
    }
    if (pushShadow)
      fprintf(stderr,
              "[push-shadow] mtasks=%d fanin_edges=%zu notify_edges=%zu mismatches=0(runtime_counted) token_edges=%d direct_edges=%d sibling_edges=%d token_edges_kept=%d direct_edges_kept=%d sibling_edges_kept=%d tokens=%d max_fanin=%d zero_fanin=%d threads=%d lookahead=%d\n",
              nMTasks, pushShadowEdgeTotal, pushShadowNotifyList.size(),
              pushShadowTokenEdges, pushShadowDirectEdges, pushShadowSiblingEdges,
              pushShadowTokenEdgesKept, pushShadowDirectEdgesKept, pushShadowSiblingEdgesKept,
              ownerReadyLayout.tokenCount, pushShadowMaxFanin, pushShadowZeroFaninMTasks,
              threadCount, denseLookahead ? 1 : 0);
  }
  // ---- Push-ready queue layout (GSIM_MT_DENSE_PUSH_READY, default off) ----
  // One bounded sequence ring per worker (queue 0 unused: worker0 stays on
  // the pull lane). Capacity is the per-worker assigned MTask count rounded
  // up to a power of two - every task enters its owner queue at most once per
  // cycle (the pending counter enqueues exactly once when it reaches zero and
  // is re-armed before each dispatch), so `assigned` is a hard upper bound on
  // ring occupancy and a full ring aborts loudly instead of overwriting.
  std::vector<uint32_t> pushReadyAssigned;
  std::vector<uint32_t> pushReadyQueueCapacity;
  std::vector<uint32_t> pushReadyQueueOffset;
  int pushReadySlotTotal = 0;
  int pushReadyRootCount = 0;
  if (pushReady) {
    pushReadyAssigned.assign((size_t)threadCount, 0u);
    for (int m = 0; m < nMTasks; ++m) {
      const int pushReadyOwner = denseSchedule.mtaskThreadAssign[(size_t)m];
      Assert(pushReadyOwner >= 0 && pushReadyOwner < threadCount,
             "GSIM_MT_DENSE_PUSH_READY requires a fixed owner for every dense MTask (MTask %d has owner %d)",
             m, pushReadyOwner);
      pushReadyAssigned[(size_t)pushReadyOwner]++;
    }
    auto pushReadyPow2Ceil = [](uint32_t value) -> uint32_t {
      if (value == 0u) return 0u;
      uint32_t power = 1u;
      while (power < value) power <<= 1u;
      return power;
    };
    pushReadyQueueCapacity.assign((size_t)threadCount, 0u);
    for (int w = 1; w < threadCount; ++w)
      pushReadyQueueCapacity[(size_t)w] = pushReadyPow2Ceil(pushReadyAssigned[(size_t)w]);
    pushReadyQueueOffset.assign((size_t)threadCount + 1, 0u);
    for (int w = 0; w < threadCount; ++w) {
      if (w >= 1)
        Assert(pushReadyQueueCapacity[(size_t)w] >= pushReadyAssigned[(size_t)w],
               "push-ready queue capacity %u below assigned count %u for worker %d",
               pushReadyQueueCapacity[(size_t)w], pushReadyAssigned[(size_t)w], w);
      pushReadyQueueOffset[(size_t)w + 1] =
          pushReadyQueueOffset[(size_t)w] + pushReadyQueueCapacity[(size_t)w];
    }
    pushReadySlotTotal = (int)pushReadyQueueOffset[(size_t)threadCount];
    for (int m = 0; m < nMTasks; ++m) {
      if (pushShadowFanin[(size_t)m] != 0u) continue;
      if (denseSchedule.mtaskThreadAssign[(size_t)m] == 0) continue;
      pushReadyRootCount++;
    }
    int pushReadyMaxCapacity = 0;
    for (int w = 1; w < threadCount; ++w)
      pushReadyMaxCapacity = std::max(pushReadyMaxCapacity, (int)pushReadyQueueCapacity[(size_t)w]);
    fprintf(stderr,
            "[push-ready] mtasks=%d fanin_edges=%zu queues=%d capacity=%d worker0_hybrid=1 roots=%d slots=%d threads=%d\n",
            nMTasks, pushShadowNotifyList.size(), std::max(0, threadCount - 1),
            pushReadyMaxCapacity, pushReadyRootCount, pushReadySlotTotal, threadCount);
    fprintf(stderr, "[push-ready] dispatch=%s\n",
            pushReadyBitmap ? "ready-byte"
                            : (pushReadyDirectTable ? "direct-table" : "switch"));
  }
  // ---- Push-ready hint layout (GSIM_MT_DENSE_PUSH_READY_HINT, default off) ----
  // No arrival structures at all: one ready byte per MTask is the whole
  // layout, consumed in place by the existing pull/lookahead control flow of
  // workers 1..N-1 (worker0 keeps the original token wait lists). The fan-in
  // CSR above (pending counters re-armed per cycle, one decrement per
  // deduplicated producer edge, final decrement release-stores the byte) is
  // identical to the one the shadow knob validated.
  if (pushReadyHint) {
    int pushReadyHintRootCount = 0;
    for (int m = 0; m < nMTasks; ++m) {
      if (pushShadowFanin[(size_t)m] != 0u) continue;
      if (denseSchedule.mtaskThreadAssign[(size_t)m] == 0) continue;
      pushReadyHintRootCount++;
    }
    fprintf(stderr,
            "[push-ready] mtasks=%d fanin_edges=%zu token_edges=%d direct_edges=%d sibling_edges=%d hints=%d roots=%d worker0_pull=1 threads=%d lookahead=%d\n",
            nMTasks, pushShadowNotifyList.size(),
            pushShadowTokenEdges, pushShadowDirectEdges, pushShadowSiblingEdges,
            nMTasks, pushReadyHintRootCount, threadCount, denseLookahead ? 1 : 0);
    fprintf(stderr, "[push-ready] dispatch=hint\n");
  }

  // ---- Token-mask hint layout (GSIM_MT_DENSE_TOKEN_MASK_HINT, default off) ----
  // No arrival structures at all: one atomic uint16 mask per consumer is the
  // whole state, summarizing only the owner-ready cross-token waits (the
  // wait-slot lists of the FINAL layout, i.e. after the lookahead rebuild).
  // Each consumer's wait slots get consecutive bit positions 0..n-1; each
  // physical store slot maps to (consumer, bit value), so a token's single
  // publisher fetch_ors exactly its own bit. Consumers with zero cross-token
  // waits keep expected==0 (their mask word is never written; the worker-side
  // comparison is trivially true and their local ordering stays the existing
  // same-worker control flow).
  std::vector<uint16_t> tokenMaskExpected;
  std::vector<uint16_t> tokenMaskConsumerBySlot;
  std::vector<uint16_t> tokenMaskBitBySlot;
  int tokenMaskMaxBits = 0;
  int tokenMaskConsumerCount = 0;
  if (tokenMaskHint) {
    Assert(nMTasks <= 65535,
           "GSIM_MT_DENSE_TOKEN_MASK_HINT supports at most 65535 MTasks (got %d)",
           nMTasks);
    tokenMaskExpected.assign((size_t)nMTasks, uint16_t{0});
    tokenMaskConsumerBySlot.assign((size_t)ownerReadyLayout.physicalSlotCount, uint16_t{0});
    tokenMaskBitBySlot.assign((size_t)ownerReadyLayout.physicalSlotCount, uint16_t{0});
    std::vector<char> tokenMaskSlotAssigned((size_t)ownerReadyLayout.physicalSlotCount, 0);
    int tokenMaskAssignedSlots = 0;
    for (int consumer = 0; consumer < nMTasks; ++consumer) {
      const std::vector<int>& waits = ownerReadyLayout.waitSlotsByMTask[(size_t)consumer];
      if (waits.empty()) continue;
      Assert(waits.size() <= 16,
             "GSIM_MT_DENSE_TOKEN_MASK_HINT consumer %d has %zu cross-token waits; a uint16 mask supports at most 16",
             consumer, waits.size());
      tokenMaskConsumerCount++;
      if (static_cast<int>(waits.size()) > tokenMaskMaxBits)
        tokenMaskMaxBits = static_cast<int>(waits.size());
      for (size_t bit = 0; bit < waits.size(); ++bit) {
        const int slot = waits[bit];
        Assert(slot >= 0 && slot < ownerReadyLayout.physicalSlotCount,
               "token-mask wait slot %d outside dense owner-ready slot range", slot);
        Assert(!tokenMaskSlotAssigned[(size_t)slot],
               "token-mask wait slot %d assigned to two consumers", slot);
        tokenMaskSlotAssigned[(size_t)slot] = 1;
        tokenMaskAssignedSlots++;
        tokenMaskExpected[(size_t)consumer] = static_cast<uint16_t>(
            tokenMaskExpected[(size_t)consumer] | (uint16_t{1} << bit));
        tokenMaskConsumerBySlot[(size_t)slot] = static_cast<uint16_t>(consumer);
        tokenMaskBitBySlot[(size_t)slot] = static_cast<uint16_t>(uint16_t{1} << bit);
      }
    }
    Assert(tokenMaskAssignedSlots == ownerReadyLayout.tokenCount,
           "token-mask assigned %d wait slots for %d dense owner-ready tokens",
           tokenMaskAssignedSlots, ownerReadyLayout.tokenCount);
    fprintf(stderr,
            "[token-mask] tokens=%d consumers=%d max_bits=%d width=16 mtasks=%d threads=%d lookahead=%d\n",
            ownerReadyLayout.tokenCount, tokenMaskConsumerCount, tokenMaskMaxBits,
            nMTasks, threadCount, denseLookahead ? 1 : 0);
    fprintf(stderr, "[token-mask] dispatch=token-mask\n");
  }
  // ---- PSCD slice 1 analysis (GSIM_EMIT_PSCD_BITS, default off) ----
  // Derives, per owner-ready token, the scalar register-state fields crossing
  // that edge: single-writer committed fields of the token sources intersected
  // with the consumer's read set. Producers snapshot those fields at body
  // entry and compute per-field change verdicts at body exit; each store phase
  // folds the verdicts into one epoch-tagged change byte per token slot,
  // stored before the ready-token release store. The optional verify shadow
  // (GSIM_PSCD_VERIFY) re-derives the consumer OR-compare (inputs vs previous
  // entry snapshots) and cross-checks it against the OR of input change bits.
  struct MtPscdFieldRef { int slot = -1; std::string name; };
  std::vector<std::vector<MtPscdFieldRef>> pscdProducerFields((size_t)nMTasks);
  std::vector<std::vector<MtPscdFieldRef>> pscdConsumerShadow((size_t)nMTasks);
  std::vector<std::vector<int>> pscdConsumerTokenSlots((size_t)nMTasks);
  int pscdFieldsDroppedReset = 0;
  int pscdWaitSlotsWithoutStoreEntry = 0;
  std::vector<std::vector<int>> pscdStoreFieldSlots;
  std::vector<char> pscdStoreConservative;
  int pscdFieldSlotCount = 0;
  int pscdShadowSlotCount = 0;
  int pscdProducersInstrumented = 0;
  int pscdConsumersShadowed = 0;
  int pscdExcludedTotal = 0, pscdExcludedWorker0 = 0, pscdExcludedElided = 0,
      pscdExcludedExt = 0, pscdExcludedSpecial = 0, pscdExcludedReset = 0,
      pscdExcludedArray = 0, pscdExcludedAmbiguous = 0;
  int pscdTokensChecked = 0, pscdTokensConservative = 0, pscdTokensZeroField = 0;
  int pscdFieldsDroppedMultiWriter = 0, pscdFieldsDroppedWide = 0, pscdFieldsDroppedOther = 0;
  if (pscdBits) {
    std::vector<std::set<std::string>> pscdCommitFields((size_t)nMTasks);
    std::vector<std::set<std::string>> pscdReadFields((size_t)nMTasks);
    std::vector<char> pscdTaskExcluded((size_t)nMTasks, 0);
    std::vector<char> pscdTaskArrayish((size_t)nMTasks, 0);
    std::vector<char> pscdTaskReset((size_t)nMTasks, 0);
    std::map<std::string, Node*> pscdFieldNode;
    std::map<std::string, std::set<int>> pscdFieldWriters;
    // Task classification. Hard exclusions (never eligible as producer or
    // consumer; their tokens are conservatively marked): worker0-only tasks,
    // elided tasks, SUPER_EXTMOD/DPI, printf/assert/exit, reset machinery,
    // ambiguous-state producers. Reset machinery = membership in a reset
    // super (SUPER_ASYNC_RESET / SUPER_UINT_RESET, the subReset/subResetDense
    // paths) or writing through the reset port (a NODE_REG_RESET member);
    // NOT a hazard for output change detection (reset-window writes are
    // neutralized at runtime by mtDensePscdResetHit marking every change
    // byte changed plus the consumer verify skip). Array /
    // dynamic-index producers are NOT excluded: they keep eligibility as
    // consumers, and every token they source gets conservative whole-region
    // change marking instead of a summarized field list.
    for (int m = 0; m < nMTasks; m++) {
      const int owner = m < static_cast<int>(denseSchedule.mtaskThreadAssign.size())
                            ? denseSchedule.mtaskThreadAssign[(size_t)m] : -1;
      int excludeCode = -1;
      bool arrayishTask = false;
      bool resetMachineryTask = false;
      if (owner < 0) excludeCode = 1;
      else if (owner == 0) excludeCode = 0;
      for (int sccId : denseSchedule.mtasks[(size_t)m].sccIds) {
        if (sccId < 0 || sccId >= static_cast<int>(denseSchedule.sccs.size())) continue;
        for (int cppId : denseSchedule.sccs[(size_t)sccId].cppIds) {
          auto superIter = cppId2Super.find(cppId);
          if (superIter == cppId2Super.end() || !superIter->second) continue;
          SuperNode* super = superIter->second;
          int dummyCost = 0;
          MtBoundaryInfo b = collectMtBoundaryInfo(super, dummyCost);
          bool superIsResetMachinery = super->superType == SUPER_ASYNC_RESET
                                     || super->superType == SUPER_UINT_RESET;
          for (Node* member : super->member)
            if (member->type == NODE_REG_RESET) superIsResetMachinery = true;
          if (excludeCode < 0) {
            if (b.hasExternal) excludeCode = 2;
            else if (b.hasSpecial) excludeCode = 3;
            else if (superIsResetMachinery) excludeCode = 4;
            else if (b.hasStateUpdate && b.hasAmbiguousStateTarget) excludeCode = 6;
          }
          if (superIsResetMachinery) resetMachineryTask = true;
          if (b.hasMemoryWrite || b.hasArrayOrDynamicIndex) arrayishTask = true;
          if (b.stateSourceCommitCount > 0 || b.stateResetUpdateCount > 0)
            pscdCommitFields[(size_t)m].insert(b.stateTargetNames.begin(), b.stateTargetNames.end());
          for (Node* member : super->member) {
            if (nodeHasStateUpdate(member)) {
              std::string tn;
              if (stateTargetNameForNode(member, tn)) {
                Node* target = member->type == NODE_REG_SRC ? member
                             : (member->type == NODE_REG_DST ? member->getSrc() : member->getResetSrc());
                if (target != nullptr) pscdFieldNode[tn] = target;
              }
            }
            for (ExpTree* tree : member->assignTree) {
              MtActivityReads tr;
              mtActivityCollectFromTree(tree->getRoot(), tr);
              pscdReadFields[(size_t)m].insert(tr.src.begin(), tr.src.end());
            }
          }
        }
      }
      pscdTaskArrayish[(size_t)m] = arrayishTask ? 1 : 0;
      pscdTaskReset[(size_t)m] = resetMachineryTask ? 1 : 0;
      if (excludeCode >= 0) {
        pscdTaskExcluded[(size_t)m] = 1;
        pscdExcludedTotal++;
        switch (excludeCode) {
          case 0: pscdExcludedWorker0++; break;
          case 1: pscdExcludedElided++; break;
          case 2: pscdExcludedExt++; break;
          case 3: pscdExcludedSpecial++; break;
          case 4: pscdExcludedReset++; break;
          default: pscdExcludedAmbiguous++; break;
        }
      } else if (arrayishTask) {
        // Never summarized as a producer (conservative whole-region marking),
        // still counted in the never-summarized total, still an eligible
        // consumer for the shadow check.
        pscdExcludedArray++;
        pscdExcludedTotal++;
      }
    }
    // Writer universe over tasks that actually run (elided tasks never
    // execute) and are NOT reset machinery. Reset-supers fire only inside
    // reset windows; their writes are neutralized at runtime by the reset-hit
    // mark-all plus the verify skip (mtDensePscdResetHit), so a field with
    // exactly one non-reset writer plus any number of reset-supers stays
    // ELIGIBLE. Excluded-but-running tasks (worker0/ext/...) still count:
    // their writes are real steady-state writes, so a field they co-write is
    // multi-writer and drops out.
    for (int m = 0; m < nMTasks; m++) {
      if (denseSchedule.mtaskThreadAssign[(size_t)m] < 0) continue;
      if (pscdTaskReset[(size_t)m]) continue;
      for (const std::string& f : pscdCommitFields[(size_t)m]) pscdFieldWriters[f].insert(m);
    }
    auto pscdWriterOf = [&](const std::string& f) -> int {
      auto w = pscdFieldWriters.find(f);
      if (w == pscdFieldWriters.end() || w->second.size() != 1) return -1;
      return *w->second.begin();
    };
    // Per store entry (same j numbering as kDenseOwnerReadyStoreOffsets /
    // kDenseOwnerReadyStoreList): the entry's compare-field slots and a
    // conservative flag. Conservative entries — any excluded source
    // (worker0/elided/ext/special/reset/ambiguous) or any array /
    // dynamic-index source — mark their whole token region changed every
    // cycle and are skipped by the verify shadow.
    pscdStoreFieldSlots.assign((size_t)std::max(1, ownerReadyLayout.tokenCount), {});
    pscdStoreConservative.assign((size_t)std::max(1, ownerReadyLayout.tokenCount), 1);
    std::map<std::string, int> pscdFieldSlotOf;
    std::vector<std::string> pscdFieldSlotName;
    std::vector<int> pscdStoreEntryOfSlot((size_t)std::max(1, ownerReadyLayout.physicalSlotCount), -1);
    {
      int j = 0;
      for (int m = 0; m < nMTasks; m++) {
        for (int slot : ownerReadyLayout.storeSlotsByMTask[(size_t)m]) {
          Assert(j < ownerReadyLayout.tokenCount,
                 "PSCD store list overruns dense owner-ready token count");
          Assert(slot >= 0 && slot < ownerReadyLayout.physicalSlotCount,
                 "PSCD store slot %d outside dense owner-ready slot range", slot);
          pscdStoreEntryOfSlot[(size_t)slot] = j;
          const int token = ownerReadyLayout.logicalTokenByPhysicalSlot[(size_t)slot];
          Assert(token >= 0 && token < ownerReadyLayout.tokenCount,
                 "PSCD store slot %d has no logical token", slot);
          const MtDenseOwnerReadyTokenProvenance& prov =
              ownerReadyLayout.tokenProvenanceByLogicalToken[(size_t)token];
          const std::vector<int>& sources = ownerReadyLayout.sourceMTasksByLogicalToken[(size_t)token];
          const bool consumerValid = prov.consumerMTask >= 0 && prov.consumerMTask < nMTasks;
          const std::set<std::string>& consumerReads =
              consumerValid ? pscdReadFields[(size_t)prov.consumerMTask] : pscdReadFields[0];
          // An out-of-range consumer is unreachable for layouts the builder
          // produces; stay conservative rather than indexing blindly.
          bool tokenConservative = !consumerValid;
          std::set<std::string> crossing;
          for (int s : sources) {
            if (s < 0 || s >= nMTasks || pscdTaskExcluded[(size_t)s] || pscdTaskArrayish[(size_t)s]) { tokenConservative = true; continue; }
            for (const std::string& f : pscdCommitFields[(size_t)s])
              if (consumerReads.count(f)) crossing.insert(f);
          }
          std::vector<int> eligibleSlots;
          if (!tokenConservative) {
            for (const std::string& f : crossing) {
              const int writer = pscdWriterOf(f);
              if (writer < 0 || pscdTaskExcluded[(size_t)writer]) {
                if (writer < 0) pscdFieldsDroppedMultiWriter++;
                else pscdFieldsDroppedOther++;
                continue;
              }
              Node* node = nullptr;
              auto nodeIter = pscdFieldNode.find(f);
              if (nodeIter != pscdFieldNode.end()) node = nodeIter->second;
              if (node == nullptr || node->isArray() || node->type == NODE_MEMORY ||
                  node->width <= 0 || node->width > 64) {
                pscdFieldsDroppedWide++;
                continue;
              }
              auto slotIter = pscdFieldSlotOf.find(f);
              int fieldSlot = -1;
              if (slotIter != pscdFieldSlotOf.end()) {
                fieldSlot = slotIter->second;
              } else {
                fieldSlot = pscdFieldSlotCount++;
                pscdFieldSlotOf.emplace(f, fieldSlot);
                pscdFieldSlotName.push_back(f);
              }
              eligibleSlots.push_back(fieldSlot);
            }
          }
          pscdStoreFieldSlots[(size_t)j] = eligibleSlots;
          pscdStoreConservative[(size_t)j] = tokenConservative ? 1 : 0;
          if (tokenConservative) pscdTokensConservative++;
          else if (eligibleSlots.empty()) pscdTokensZeroField++;
          j++;
        }
      }
      Assert(j == ownerReadyLayout.tokenCount,
             "PSCD store list has %d entries for %d dense owner-ready tokens",
             j, ownerReadyLayout.tokenCount);
    }
    // Producer snapshot/verdict lists: each compare field is owned by its
    // single writer, whose body snapshots it at entry and computes the
    // post-body change verdict at exit.
    for (int j = 0; j < ownerReadyLayout.tokenCount; j++) {
      for (int fieldSlot : pscdStoreFieldSlots[(size_t)j]) {
        const int writer = pscdWriterOf(pscdFieldSlotName[(size_t)fieldSlot]);
        Assert(writer >= 0 && !pscdTaskExcluded[(size_t)writer],
               "PSCD field slot %d has no eligible writer", fieldSlot);
        auto& list = pscdProducerFields[(size_t)writer];
        bool present = false;
        for (const MtPscdFieldRef& fr : list)
          if (fr.slot == fieldSlot) { present = true; break; }
        if (!present) list.push_back(MtPscdFieldRef{fieldSlot, pscdFieldSlotName[(size_t)fieldSlot]});
      }
    }
    for (int m = 0; m < nMTasks; m++)
      if (!pscdProducerFields[(size_t)m].empty()) pscdProducersInstrumented++;
    // Consumer verify lists: shadow fields and checked token slots per
    // eligible consumer. Tokens with conservative or zero-field entries are
    // skipped on both sides of the equality, keeping it self-consistent.
    if (pscdVerify) {
      int shadowBase = 0;
      for (int c = 0; c < nMTasks; c++) {
        if (pscdTaskExcluded[(size_t)c]) continue;
        std::set<int> shadowFieldSlots;
        std::vector<int> tokenSlots;
        for (int slot : ownerReadyLayout.waitSlotsByMTask[(size_t)c]) {
          Assert(slot >= 0 && slot < ownerReadyLayout.physicalSlotCount,
                 "PSCD wait slot %d outside dense owner-ready slot range", slot);
          // Defensive: a wait slot with no store entry is treated as
          // NOT-SUMMARIZED — it contributes to neither side of the shadow
          // equality (no summary read, no shadow field), so the consumer's
          // activation for that input group keeps its original ready-token
          // path untouched. The layout builder guarantees wait and store
          // slot sets are identical, so this path is unreachable; it exists
          // so a future layout change degrades coverage instead of aborting
          // generation. Counted and reported on a separate stderr line when
          // nonzero (the [pscd] format line stays stable).
          const int j = pscdStoreEntryOfSlot[(size_t)slot];
          if (j < 0) {
            pscdWaitSlotsWithoutStoreEntry++;
            continue;
          }
          if (pscdStoreConservative[(size_t)j]) continue;
          if (pscdStoreFieldSlots[(size_t)j].empty()) continue;
          tokenSlots.push_back(slot);
          for (int fieldSlot : pscdStoreFieldSlots[(size_t)j]) shadowFieldSlots.insert(fieldSlot);
        }
        if (tokenSlots.empty() || shadowFieldSlots.empty()) continue;
        for (int fieldSlot : shadowFieldSlots)
          pscdConsumerShadow[(size_t)c].push_back(
              MtPscdFieldRef{shadowBase++, pscdFieldSlotName[(size_t)fieldSlot]});
        pscdShadowSlotCount = shadowBase;
        pscdConsumerTokenSlots[(size_t)c] = tokenSlots;
        pscdTokensChecked += (int)tokenSlots.size();
        pscdConsumersShadowed++;
      }
    }
    fprintf(stderr,
            "[pscd] producers_instrumented=%d consumers_shadowed=%d excluded=%d "
            "(worker0=%d elided=%d ext=%d special=%d reset=%d array=%d ambiguous=%d) "
            "tokens=%d checked=%d conservative=%d zero_field=%d fields=%d "
            "dropped=(multi_writer=%d reset=%d wide_or_array=%d excluded_writer=%d) shadow_slots=%d verify=%d\n",
            pscdProducersInstrumented, pscdConsumersShadowed, pscdExcludedTotal,
            pscdExcludedWorker0, pscdExcludedElided, pscdExcludedExt, pscdExcludedSpecial,
            pscdExcludedReset, pscdExcludedArray, pscdExcludedAmbiguous,
            ownerReadyLayout.tokenCount, pscdTokensChecked, pscdTokensConservative,
            pscdTokensZeroField, pscdFieldSlotCount, pscdFieldsDroppedMultiWriter,
            pscdFieldsDroppedReset, pscdFieldsDroppedWide, pscdFieldsDroppedOther,
            pscdShadowSlotCount, pscdVerify ? 1 : 0);
    if (pscdWaitSlotsWithoutStoreEntry > 0)
      fprintf(stderr, "[pscd] wait_slots_without_store_entry=%d (treated as not-summarized; excluded from both sides of the shadow equality)\n",
              pscdWaitSlotsWithoutStoreEntry);
    // Per-consumer precision dump: a consumer is "all-precise" iff every wait
    // slot resolves to a store entry that is not conservative. Used offline to
    // join against per-mtask profile samples (slice-2 benefit analysis).
    {
      std::string dumpPath = globalConfig.OutputDir + "/" + name + "_pscd_consumers.json";
      FILE* dump = fopen(dumpPath.c_str(), "w");
      if (dump != nullptr) {
        fprintf(dump, "{\"all_precise\":[");
        bool first = true;
        int allPreciseCount = 0;
        for (int c = 0; c < nMTasks; c++) {
          const auto& waits = ownerReadyLayout.waitSlotsByMTask[(size_t)c];
          if (waits.empty()) continue;
          bool allPrecise = true;
          for (int slot : waits) {
            int j = (slot >= 0 && slot < (int)pscdStoreEntryOfSlot.size())
                        ? pscdStoreEntryOfSlot[(size_t)slot] : -1;
            if (j < 0 || pscdStoreConservative[(size_t)j]) { allPrecise = false; break; }
          }
          if (allPrecise) {
            fprintf(dump, "%s%d", first ? "" : ",", c);
            first = false;
            allPreciseCount++;
          }
        }
        fprintf(dump, "],\"all_precise_count\":%d,\"consumers_with_waits\":%d}\n",
                allPreciseCount, nMTasks);
        fclose(dump);
        fprintf(stderr, "[pscd] all_precise_consumers=%d wrote %s\n", allPreciseCount, dumpPath.c_str());
      }
    }
  }
  // ---- GSIM_MT_DENSE_EDGE_TIMING (default off): per-edge wait latency ----
  // Canonical wait-entry numbering: identical to kDenseOwnerReadyWaitList's
  // construction (worker ascending, MTask ascending within worker, per-task
  // slot order with GSIM_EMIT_SORTED_WAITS honored), so the lookahead tail's
  // dispatch-wait loop variable IS this index, and the strict dispatch's
  // literal-slot wait loops look their indices up from this map. Each entry
  // has exactly one consumer worker, so the accumulators are sole-writer
  // plain uint64s — no atomics in the hot path.
  std::map<int, int> edgeTimingWaitIndexOfSlot;
  std::vector<int> edgeTimingWaitSlotList;
  std::vector<int> edgeTimingWaitConsumerList;
  int edgeTimingWaitTotal = 0;
  if (edgeTiming) {
    bool edgeSortWaits = false;
    { const char* e = std::getenv("GSIM_EMIT_SORTED_WAITS"); edgeSortWaits = e && e[0] && e[0] != '0'; }
    for (int t = 0; t < threadCount; t++) {
      for (int m = 0; m < nMTasks; m++) {
        if (denseSchedule.mtaskThreadAssign[(size_t)m] != t) continue;
        std::vector<int> slots = ownerReadyLayout.waitSlotsByMTask[(size_t)m];
        if (edgeSortWaits && slots.size() > 1) std::sort(slots.begin(), slots.end());
        for (int slot : slots) {
          Assert(slot >= 0 && slot < ownerReadyLayout.physicalSlotCount,
                 "edge-timing wait slot %d outside dense owner-ready slot range", slot);
          edgeTimingWaitIndexOfSlot[slot] = edgeTimingWaitTotal++;
          edgeTimingWaitSlotList.push_back(slot);
          edgeTimingWaitConsumerList.push_back(m);
        }
      }
    }
    Assert(!denseLookahead || edgeTimingWaitTotal == (int)denseDispatchWaitTotal,
           "edge-timing wait numbering (%d) diverged from kDenseOwnerReadyWaitList (%u)",
           edgeTimingWaitTotal, denseDispatchWaitTotal);
    fprintf(stderr, "[edge-timing] slots=%d waits=%d\n",
            ownerReadyLayout.physicalSlotCount, edgeTimingWaitTotal);
  }

  auto emitDenseDepCounts = [&]() {
    fprintf(header, "static constexpr uint32_t kDenseMTaskDepCount[%d] = {", nMTasks);
    for (int i = 0; i < nMTasks; i++) { if (i > 0) fprintf(header, ","); fprintf(header, "%u", denseRuntimeDepCounts[(size_t)i]); }
    fprintf(header, "};\n");
  };
  auto emitDenseOwnerBankDiagnostics = [&]() {
    if (!ownerBankCountersDiag) return;
    fprintf(header, "static constexpr int kDenseMTaskPhysicalSlotCount = %d;\n", nMTasks);
    fprintf(header, "static constexpr int kDenseMTaskVertexSlot[%d] = {", nMTasks);
    for (int i = 0; i < nMTasks; i ++) { if (i > 0) fprintf(header, ","); fprintf(header, "%d", i); }
    fprintf(header, "};\n");
    fprintf(header, "static constexpr int kDenseMTaskVertexOwner[%d] = {", nMTasks);
    for (int i = 0; i < nMTasks; i ++) { if (i > 0) fprintf(header, ","); fprintf(header, "%d", denseSchedule.mtaskThreadAssign[(size_t)i]); }
    fprintf(header, "};\n");
  };
  auto emitDenseSuccOffsets = [&]() {
    fprintf(header, "static constexpr int kDenseMTaskSuccOffsets[%d] = {", nMTasks + 1);
    int off = 0;
    for (int i = 0; i < nMTasks; i++) {
      if (i > 0) fprintf(header, ",");
      fprintf(header, "%d", off);
      off += static_cast<int>(denseRuntimeSuccs[(size_t)i].size());
    }
    fprintf(header, ",%d};\n", off);
  };
  auto emitDenseSuccList = [&]() {
    fprintf(header, "static constexpr int kDenseMTaskSuccList[%d] = {", totalSuccs);
    bool firstSucc = true;
    for (const std::vector<int>& succs : denseRuntimeSuccs) {
      for (int succ : succs) {
        if (!firstSucc) fprintf(header, ",");
        fprintf(header, "%d", succ);
        firstSucc = false;
      }
    }
    if (firstSucc) fprintf(header, "0");
    fprintf(header, "};\n");
  };

  const std::chrono::steady_clock::time_point denseProHeaderTablesBegin = std::chrono::steady_clock::now();
  fprintf(header, "static constexpr bool kDenseXThreadDepsOnly = %s;\n", xthreadDepsOnly ? "true" : "false");
  fprintf(header, "static constexpr bool kDenseTransitiveReduceEdges = %s;\n", transitiveReduceEdges ? "true" : "false");
  fprintf(header, "static constexpr int kDenseTransitiveElidedEdgeCount = %d;\n", transitiveElidedEdges);
  if (!ownerReadyFlags) {
    emitDenseDepCounts();
    emitDenseOwnerBankDiagnostics();
    emitDenseSuccOffsets();
    emitDenseSuccList();
    fprintf(header, "struct MtDenseMTaskVertex { std::atomic<uint32_t> depsDone{0}; };\n");
    fprintf(header, "MtDenseMTaskVertex mtDenseMTaskVertices[%d];\n", nMTasks);
  } else {
    if (denseBreakdownProfileCodegen) {
      fprintf(header, "#if !defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) || !GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
      fprintf(header, "#error \"GSIM_MT_DENSE_BREAKDOWN_PROFILE requires GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\"\n");
      fprintf(header, "#endif\n");
    }

    fprintf(header, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    fprintf(header, "static constexpr int kDenseOwnerReadyTokenCount = %d;\n", ownerReadyLayout.tokenCount);
    fprintf(header, "static constexpr int kDenseOwnerReadyWorkerCount = %d;\n", threadCount);
    fprintf(header, "static constexpr int kDenseOwnerReadyPhysicalSlotCount = %d;\n", ownerReadyLayout.physicalSlotCount);
    fprintf(header, "static constexpr int kDenseOwnerReadyStoreOffsets[%d] = {", nMTasks + 1);
    {
      int off = 0;
      for (int mtaskId = 0; mtaskId < nMTasks; mtaskId ++) {
        if (mtaskId > 0) fprintf(header, ",");
        fprintf(header, "%d", off);
        off += static_cast<int>(ownerReadyLayout.storeSlotsByMTask[(size_t)mtaskId].size());
      }
      fprintf(header, ",%d};\n", off);
      Assert(off == ownerReadyLayout.tokenCount,
             "dense owner-ready signal table has %d entries for %d tokens",
             off, ownerReadyLayout.tokenCount);
    }
    fprintf(header, "static constexpr int kDenseOwnerReadyStoreList[%d] = {",
            std::max(1, ownerReadyLayout.tokenCount));
    {
      bool firstSlot = true;
      for (const std::vector<int>& slots : ownerReadyLayout.storeSlotsByMTask) {
        for (int slot : slots) {
          if (!firstSlot) fprintf(header, ",");
          fprintf(header, "%d", slot);
          firstSlot = false;
        }
      }
      if (firstSlot) fprintf(header, "0");
      fprintf(header, "};\n");
    }
    fprintf(header, "struct MtDenseOwnerReadyToken { std::atomic<uint8_t> ready{0}; };\n");
    if (denseDuty) {
      fprintf(header, "struct alignas(64) MtDenseDutyLane { uint64_t spinNs = 0, spanNs = 0, tailNs = 0, blockNs = 0, resetNs = 0, joinNs = 0, stepWallNs = 0, count = 0; };\n");
      fprintf(header, "MtDenseDutyLane mtDutyLanes[%d];\n", threadCount + 1);
      fprintf(header, "static constexpr int kDenseDutyLaneMax = %d;\n", threadCount);
      fprintf(header, "bool mtDutyEnabled = false;\n");
      // RAII span recorder: survives the early returns inside the worker switch cases.
      fprintf(header, "struct MtDenseDutyGuard { uint64_t* acc; std::chrono::steady_clock::time_point t0; MtDenseDutyGuard(uint64_t* a) : acc(a), t0(std::chrono::steady_clock::now()) {} ~MtDenseDutyGuard() { if (acc) *acc += (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count(); } };\n");
    }
    fprintf(header, "static_assert(sizeof(std::atomic<uint8_t>) == 1, \"owner-ready atomic must occupy one byte\");\n");
    fprintf(header, "static_assert(std::atomic<uint8_t>::is_always_lock_free, \"owner-ready atomic must be lock-free\");\n");
    fprintf(header, "static_assert(sizeof(MtDenseOwnerReadyToken) == 1, \"owner-ready slots must have one-byte extent\");\n");
    fprintf(header, "static_assert((kDenseOwnerReadyPhysicalSlotCount %% 64) == 0, \"owner-ready storage must end on a 64-byte boundary\");\n");
    fprintf(header, "alignas(64) MtDenseOwnerReadyToken mtDenseOwnerReadyTokens[%d];\n",
            ownerReadyLayout.physicalSlotCount);
    fprintf(header, "bool mtDenseOwnerReadyTokensPrimed = false;\n");
    if (pscdBits) {
      // PSCD slice 1 runtime state. mtDenseChangeEpoch is slot-indexed and
      // parallel to mtDenseOwnerReadyTokens; mtDenseCycleEpoch flips 1/2 with
      // cycle parity, so a byte equals mtDenseCycleEpoch iff its producer
      // changed the token's fields this cycle (unchanged entries store the
      // opposite phase, 3 - epoch, avoiding any per-cycle clear pass).
      fprintf(header, "// GSIM_EMIT_PSCD_BITS: producer change detection (slice 1)\n");
      fprintf(header, "static constexpr int kDensePscdFieldSlotCount = %d;\n", pscdFieldSlotCount);
      fprintf(header, "uint64_t mtDensePscdOldVal[%d];\n", std::max(1, pscdFieldSlotCount));
      fprintf(header, "uint8_t mtDensePscdFieldChanged[%d];\n", std::max(1, pscdFieldSlotCount));
      fprintf(header, "uint8_t mtDenseChangeEpoch[%d];\n", ownerReadyLayout.physicalSlotCount);
      fprintf(header, "uint8_t mtDenseCycleEpoch = 0;\n");
      fprintf(header, "static constexpr uint8_t kDensePscdStoreConservative[%d] = {",
              std::max(1, ownerReadyLayout.tokenCount));
      for (int j = 0; j < ownerReadyLayout.tokenCount; j++)
        fprintf(header, "%s%u", j ? "," : "", (unsigned)pscdStoreConservative[(size_t)j]);
      if (ownerReadyLayout.tokenCount == 0) fprintf(header, "0");
      fprintf(header, "};\n");
      {
        int fieldListTotal = 0;
        for (const std::vector<int>& slots : pscdStoreFieldSlots) fieldListTotal += (int)slots.size();
        fprintf(header, "static constexpr int kDensePscdStoreFieldOffsets[%d] = {",
                ownerReadyLayout.tokenCount + 1);
        int off = 0;
        for (int j = 0; j < ownerReadyLayout.tokenCount; j++) {
          fprintf(header, "%s%d", j ? "," : "", off);
          off += (int)pscdStoreFieldSlots[(size_t)j].size();
        }
        fprintf(header, ",%d};\n", off);
        fprintf(header, "static constexpr int kDensePscdStoreFieldList[%d] = {", std::max(1, fieldListTotal));
        bool firstSlot = true;
        for (const std::vector<int>& slots : pscdStoreFieldSlots)
          for (int fieldSlot : slots) {
            fprintf(header, "%s%d", firstSlot ? "" : ",", fieldSlot);
            firstSlot = false;
          }
        if (firstSlot) fprintf(header, "0");
        fprintf(header, "};\n");
      }
      fprintf(header, "void mtDensePscdRelease(uint32_t pscdStoreBegin, uint32_t pscdStoreEnd);\n");
      // Reset-window handling: subResetDenseN calls mtDensePscdResetHit()
      // under their reset guard, which marks every change byte changed for
      // the cycle; the consumer verify skips cross-checks in any cycle whose
      // window may contain a reset application (this or previous cycle).
      fprintf(header, "uint8_t mtDensePscdResetActive = 0;\n");
      fprintf(header, "uint8_t mtDensePscdResetActivePrev = 0;\n");
      fprintf(header, "void mtDensePscdResetHit();\n");
      if (pscdVerify) {
        fprintf(header, "// GSIM_PSCD_VERIFY: consumer shadow cross-check state\n");
        fprintf(header, "uint64_t mtDensePscdShadowPrev[%d];\n", std::max(1, pscdShadowSlotCount));
        fprintf(header, "uint8_t mtDensePscdConsumerPrimed[%d] = {0};\n", nMTasks);
        fprintf(header, "uint8_t mtPscdMismatchReported[%d] = {0};\n", nMTasks);
        fprintf(header, "std::atomic<uint64_t> mtPscdMismatchCount{};\n");
      }
    }
    if (edgeTiming) {
      // GSIM_MT_DENSE_EDGE_TIMING: per-edge wait latency state. The fire
      // timestamp array is slot-indexed (sole writer = the releasing owner
      // worker); the blocked accumulators are wait-entry indexed (sole writer
      // = the consuming worker). Plain uint64s — no atomics in hot paths.
      // rdtsc idiom reused from the wallfrac audit emitter.
      fprintf(header, "// GSIM_MT_DENSE_EDGE_TIMING: per-edge wait latency (rdtsc ticks)\n");
      fprintf(header, "static uint64_t mtEdgeTimingRdtsc() { uint32_t __edgeLo, __edgeHi; __asm__ __volatile__(\"rdtsc\":\"=a\"(__edgeLo),\"=d\"(__edgeHi)); return ((uint64_t)__edgeHi << 32) | __edgeLo; }\n");
      fprintf(header, "uint64_t mtDenseEdgeFireRdtsc[%d];\n", std::max(1, ownerReadyLayout.physicalSlotCount));
      fprintf(header, "static constexpr int kDenseEdgeTimingWaitEntryCount = %d;\n", edgeTimingWaitTotal);
      fprintf(header, "uint64_t mtDenseEdgeBlockedNs[%d];\n", std::max(1, edgeTimingWaitTotal));
      fprintf(header, "uint64_t mtDenseEdgeWaits[%d];\n", std::max(1, edgeTimingWaitTotal));
      fprintf(header, "static constexpr int kDenseEdgeTimingWaitSlot[%d] = {", std::max(1, edgeTimingWaitTotal));
      for (size_t w = 0; w < edgeTimingWaitSlotList.size(); w++)
        fprintf(header, "%s%d", w ? "," : "", edgeTimingWaitSlotList[w]);
      if (edgeTimingWaitSlotList.empty()) fprintf(header, "0");
      fprintf(header, "};\n");
      fprintf(header, "static constexpr int kDenseEdgeTimingWaitConsumer[%d] = {", std::max(1, edgeTimingWaitTotal));
      for (size_t w = 0; w < edgeTimingWaitConsumerList.size(); w++)
        fprintf(header, "%s%d", w ? "," : "", edgeTimingWaitConsumerList[w]);
      if (edgeTimingWaitConsumerList.empty()) fprintf(header, "0");
      fprintf(header, "};\n");
      fprintf(header, "void dumpDenseEdgeTiming();\n");
    }
    if (pushShadow) {
      // GSIM_MT_DENSE_PUSH_READY_SHADOW: full-model fan-in oracle (default
      // off). pending[d] counts this cycle's outstanding producers of
      // dependent d under the exact push mapping (owner-ready tokens + direct
      // local preds + publisher siblings, deduplicated); it is re-armed to the
      // generated fan-in before every stepDense dispatch and decremented once
      // per edge at producer completion. Notification/decrement accounting is
      // per-worker lane (sole writer, plain loads/stores); only the pending
      // RMWs and the mismatch-only diagnostic counter are atomic.
      fprintf(header, "// GSIM_MT_DENSE_PUSH_READY_SHADOW: push fan-in oracle (default off)\n");
      fprintf(header, "static constexpr int kDensePushShadowMTaskCount = %d;\n", nMTasks);
      fprintf(header, "static constexpr uint32_t kDensePushShadowFanin[%d] = {", nMTasks);
      for (int m = 0; m < nMTasks; m++)
        fprintf(header, "%s%u", m ? "," : "", pushShadowFanin[(size_t)m]);
      fprintf(header, "};\n");
      fprintf(header, "static constexpr uint32_t kDensePushShadowNotifyOffsets[%d] = {", nMTasks + 1);
      for (int p = 0; p <= nMTasks; p++)
        fprintf(header, "%s%u", p ? "," : "", pushShadowNotifyOffsets[(size_t)p]);
      fprintf(header, "};\n");
      fprintf(header, "static constexpr uint32_t kDensePushShadowNotifyList[%zu] = {",
              std::max<size_t>(1, pushShadowNotifyList.size()));
      for (size_t i = 0; i < pushShadowNotifyList.size(); ++i)
        fprintf(header, "%s%u", i ? "," : "", pushShadowNotifyList[i]);
      if (pushShadowNotifyList.empty()) fprintf(header, "0");
      fprintf(header, "};\n");
      fprintf(header, "static_assert(std::atomic<uint32_t>::is_always_lock_free, \"push-shadow pending must be lock-free\");\n");
      fprintf(header, "std::atomic<uint32_t> mtDensePushShadowPending[%d];\n", nMTasks);
      fprintf(header, "uint64_t mtDensePushShadowNotifyByLane[%d] = {};\n", threadCount + 1);
      fprintf(header, "uint64_t mtDensePushShadowDecrementByLane[%d] = {};\n", threadCount + 1);
      fprintf(header, "std::atomic<uint64_t> mtDensePushShadowMismatchCount{};\n");
      fprintf(header, "uint8_t mtDensePushShadowMismatchReported[%d] = {0};\n", nMTasks);
      fprintf(header, "void mtDensePushShadowNotify(uint32_t mtaskId, uint32_t lane);\n");
    }
    if (pushReady) {
      // GSIM_MT_DENSE_PUSH_READY runtime state (default off, inside the
      // owner-ready compile guard). pending[d] is re-armed to the deduplicated
      // fan-in census before every dispatch and decremented once per producer
      // edge at completion. Arrival state depends on the refinement knob: v1
      // emits Vyukov sequence rings (per-slot sequence numbers - not the tail
      // index - gate consumer visibility, so a consumer that acquires a slot
      // always sees the producer's data plus every write its release-store
      // ordered before it); the bitmap refinement replaces them with one
      // lock-free ready byte per MTask and drops the queue state entirely so
      // the generated comparison stays isolated.
      fprintf(header, "// GSIM_MT_DENSE_PUSH_READY: hybrid push executor state (default off)\n");
      fprintf(header, "static constexpr int kDensePushReadyMTaskCount = %d;\n", nMTasks);
      fprintf(header, "static constexpr int kDensePushReadyWorkerCount = %d;\n", threadCount);
      fprintf(header, "static constexpr uint32_t kDensePushReadyFanin[%d] = {", nMTasks);
      for (int m = 0; m < nMTasks; m++)
        fprintf(header, "%s%u", m ? "," : "", pushShadowFanin[(size_t)m]);
      fprintf(header, "};\n");
      fprintf(header, "static constexpr uint32_t kDensePushReadyNotifyOffsets[%d] = {", nMTasks + 1);
      for (int p = 0; p <= nMTasks; p++)
        fprintf(header, "%s%u", p ? "," : "", pushShadowNotifyOffsets[(size_t)p]);
      fprintf(header, "};\n");
      fprintf(header, "static constexpr uint32_t kDensePushReadyNotifyList[%zu] = {",
              std::max<size_t>(1, pushShadowNotifyList.size()));
      for (size_t i = 0; i < pushShadowNotifyList.size(); ++i)
        fprintf(header, "%s%u", i ? "," : "", pushShadowNotifyList[i]);
      if (pushShadowNotifyList.empty()) fprintf(header, "0");
      fprintf(header, "};\n");
      fprintf(header, "static constexpr uint32_t kDensePushReadyOwner[%d] = {", nMTasks);
      for (int m = 0; m < nMTasks; m++)
        fprintf(header, "%s%u", m ? "," : "",
                (unsigned)denseSchedule.mtaskThreadAssign[(size_t)m]);
      fprintf(header, "};\n");
      fprintf(header, "static constexpr uint32_t kDensePushReadyAssigned[%d] = {", threadCount);
      for (int w = 0; w < threadCount; w++)
        fprintf(header, "%s%u", w ? "," : "", pushReadyAssigned[(size_t)w]);
      fprintf(header, "};\n");
      if (!pushReadyBitmap) {
        fprintf(header, "static constexpr uint32_t kDensePushReadyQueueCapacity[%d] = {", threadCount);
        for (int w = 0; w < threadCount; w++)
          fprintf(header, "%s%u", w ? "," : "", pushReadyQueueCapacity[(size_t)w]);
        fprintf(header, "};\n");
        fprintf(header, "static constexpr uint32_t kDensePushReadyQueueOffset[%d] = {", threadCount + 1);
        for (int w = 0; w <= threadCount; w++)
          fprintf(header, "%s%u", w ? "," : "", pushReadyQueueOffset[(size_t)w]);
        fprintf(header, "};\n");
        fprintf(header, "static constexpr int kDensePushReadySlotTotal = %d;\n", pushReadySlotTotal);
      }
      fprintf(header, "static_assert(std::atomic<uint32_t>::is_always_lock_free, \"push-ready pending must be lock-free\");\n");
      fprintf(header, "std::atomic<uint32_t> mtDensePushReadyPending[%d];\n", nMTasks);
      if (!pushReadyBitmap) {
        fprintf(header, "std::atomic<uint64_t> mtDensePushReadyHead[%d];\n", threadCount);
        fprintf(header, "std::atomic<uint64_t> mtDensePushReadyTail[%d];\n", threadCount);
        fprintf(header, "struct MtDensePushReadySlot { std::atomic<uint32_t> seq; uint32_t mtaskId; };\n");
        fprintf(header, "MtDensePushReadySlot mtDensePushReadySlots[%d];\n", std::max(1, pushReadySlotTotal));
      } else {
        // GSIM_MT_DENSE_PUSH_READY_BITMAP arrival state: one lock-free byte
        // per MTask. Set (release) by the final fan-in decrement in notify or
        // by prepare for zero-fan-in roots; cleared by the owner worker, the
        // byte's single consumer, before it runs the body. No queue state.
        fprintf(header, "// GSIM_MT_DENSE_PUSH_READY_BITMAP: ready-byte arrival state (default off)\n");
        fprintf(header, "static_assert(std::atomic<uint8_t>::is_always_lock_free, \"push-ready ready byte must be lock-free\");\n");
        fprintf(header, "std::atomic<uint8_t> mtDensePushReadyReady[%d];\n", nMTasks);
      }
      if (pushReadyDebug)
        fprintf(header, "uint8_t mtDensePushReadyExecCount[%d] = {0};\n", nMTasks);
      fprintf(header, "void mtDensePushReadyPrepare();\n");
      fprintf(header, "void mtDensePushReadyNotify(uint32_t mtaskId);\n");
      if (!pushReadyBitmap) {
        fprintf(header, "void mtDensePushReadyEnqueue(uint32_t worker, uint32_t mtaskId);\n");
        fprintf(header, "bool mtDensePushReadyDequeue(uint32_t worker, uint32_t &mtaskId);\n");
        fprintf(header, "void stepDensePushReadyRun(uint32_t mtaskId);\n");
        fprintf(header, "void stepDensePushThreadWorker(int threadId);\n");
        if (pushReadyDirectTable) {
          // GSIM_MT_DENSE_PUSH_READY_DIRECT_TABLE: flat dispatch table indexed
          // by mtask ID replacing the generated switch. Same gating discipline
          // as kDenseDispatchTableW: declaration here (inside the owner-ready
          // compile guard), definition next to stepDensePushReadyRun.
          fprintf(header, "// GSIM_MT_DENSE_PUSH_READY_DIRECT_TABLE: member-fn dispatch table (default off)\n");
          fprintf(header, "typedef void (S%s::*MtDensePushReadyDispatchFn)();\n", name.c_str());
          fprintf(header, "static const MtDensePushReadyDispatchFn kDensePushReadyDispatchTable[kDensePushReadyMTaskCount];\n");
        }
      } else {
        // GSIM_MT_DENSE_PUSH_READY_BITMAP: the queue-free push worker. Same
        // gating discipline as the v1 queue worker it replaces.
        fprintf(header, "void stepDensePushReadyBitmapWorker(int threadId);\n");
      }
    }
    if (pushReadyHint) {
      // GSIM_MT_DENSE_PUSH_READY_HINT runtime state (default off, inside the
      // owner-ready compile guard). Same deduplicated fan-in CSR the shadow
      // validated: pending[d] is re-armed to fan-in before every dispatch and
      // decremented once per producer edge at completion; the decrementer that
      // observes old==1 release-stores the dependent's ready byte. The byte is
      // the entire arrival state - no queues, no dispatch switch; workers
      // 1..N-1 acquire-load it in place of their remote token wait lists, and
      // it is never cleared mid-cycle (each assigned task executes exactly
      // once, so a set byte cannot authorize a second execution).
      fprintf(header, "// GSIM_MT_DENSE_PUSH_READY_HINT: queue-free pull-hint state (default off)\n");
      fprintf(header, "static constexpr int kDensePushReadyHintMTaskCount = %d;\n", nMTasks);
      fprintf(header, "static constexpr uint32_t kDensePushReadyHintFanin[%d] = {", nMTasks);
      for (int m = 0; m < nMTasks; m++)
        fprintf(header, "%s%u", m ? "," : "", pushShadowFanin[(size_t)m]);
      fprintf(header, "};\n");
      fprintf(header, "static constexpr uint32_t kDensePushReadyHintNotifyOffsets[%d] = {", nMTasks + 1);
      for (int p = 0; p <= nMTasks; p++)
        fprintf(header, "%s%u", p ? "," : "", pushShadowNotifyOffsets[(size_t)p]);
      fprintf(header, "};\n");
      fprintf(header, "static constexpr uint32_t kDensePushReadyHintNotifyList[%zu] = {",
              std::max<size_t>(1, pushShadowNotifyList.size()));
      for (size_t i = 0; i < pushShadowNotifyList.size(); ++i)
        fprintf(header, "%s%u", i ? "," : "", pushShadowNotifyList[i]);
      if (pushShadowNotifyList.empty()) fprintf(header, "0");
      fprintf(header, "};\n");
      fprintf(header, "static_assert(std::atomic<uint32_t>::is_always_lock_free, \"push-ready hint pending must be lock-free\");\n");
      fprintf(header, "static_assert(std::atomic<uint8_t>::is_always_lock_free, \"push-ready hint byte must be lock-free\");\n");
      fprintf(header, "std::atomic<uint32_t> mtDensePushReadyHintPending[%d];\n", nMTasks);
      fprintf(header, "std::atomic<uint8_t> mtDensePushReadyHint[%d];\n", nMTasks);
      fprintf(header, "void mtDensePushReadyHintPrepare();\n");
      fprintf(header, "void mtDensePushReadyHintNotify(uint32_t mtaskId);\n");
    }
    if (tokenMaskHint) {
      // GSIM_MT_DENSE_TOKEN_MASK_HINT runtime state (default off, inside the
      // owner-ready compile guard). One atomic uint16 per consumer summarizing
      // only its cross-token waits: each owner-ready release fetch_ors the
      // consumer's bit for that token (release RMW before the unchanged token
      // store), and pool workers 1..N-1 acquire the word and compare against
      // the expected mask instead of loading every remote token slot. A
      // token's single publisher writes exactly its own bit, so concurrent
      // producers only ever touch distinct bits of the same word. The mask is
      // never cleared mid-cycle (each task executes once); it is reset to
      // zero once per cycle at the global pool barrier.
      fprintf(header, "// GSIM_MT_DENSE_TOKEN_MASK_HINT: cross-token readiness mask state (default off)\n");
      fprintf(header, "static constexpr int kDenseTokenMaskMTaskCount = %d;\n", nMTasks);
      fprintf(header, "static constexpr uint16_t kDenseTokenMaskExpected[%d] = {", nMTasks);
      for (int m = 0; m < nMTasks; m++)
        fprintf(header, "%s%u", m ? "," : "", (unsigned)tokenMaskExpected[(size_t)m]);
      fprintf(header, "};\n");
      fprintf(header, "static constexpr uint16_t kDenseTokenMaskConsumerBySlot[%d] = {",
              std::max(1, ownerReadyLayout.physicalSlotCount));
      for (int slot = 0; slot < ownerReadyLayout.physicalSlotCount; slot++)
        fprintf(header, "%s%u", slot ? "," : "", (unsigned)tokenMaskConsumerBySlot[(size_t)slot]);
      if (ownerReadyLayout.physicalSlotCount == 0) fprintf(header, "0");
      fprintf(header, "};\n");
      fprintf(header, "static constexpr uint16_t kDenseTokenMaskBitBySlot[%d] = {",
              std::max(1, ownerReadyLayout.physicalSlotCount));
      for (int slot = 0; slot < ownerReadyLayout.physicalSlotCount; slot++)
        fprintf(header, "%s%u", slot ? "," : "", (unsigned)tokenMaskBitBySlot[(size_t)slot]);
      if (ownerReadyLayout.physicalSlotCount == 0) fprintf(header, "0");
      fprintf(header, "};\n");
      fprintf(header, "static_assert(std::atomic<uint16_t>::is_always_lock_free, \"token mask must be lock-free\");\n");
      fprintf(header, "std::atomic<uint16_t> mtDenseTokenMask[%d];\n", nMTasks);
    }
    if (denseLookahead) {
      fprintf(header, "static constexpr int kDenseOwnerReadyWaitList[%d] = {",
              std::max(1, (int)denseDispatchWaitTotal));
      bool firstSlot = true;
      // GSIM_EMIT_SORTED_WAITS=1 (default off): sort each mtask's wait-slot
      // list by token index. Conjunction is commutative so readiness semantics
      // are unchanged; grouping same-line tokens (64/line) converts scattered
      // acquire loads into sequential same-line accesses that stay L1-resident.
      // Attacks the 136G scan/dispatch pool (attribution-revision-analytical).
      bool sortWaits = false;
      { const char* e = std::getenv("GSIM_EMIT_SORTED_WAITS"); sortWaits = e && e[0] && e[0] != '0'; }
      for (int t = 0; t < threadCount; t++) {
        for (int m = 0; m < nMTasks; m++) {
          if (denseSchedule.mtaskThreadAssign[(size_t)m] != t) continue;
          std::vector<int> slots = ownerReadyLayout.waitSlotsByMTask[(size_t)m];
          if (sortWaits && slots.size() > 1) std::sort(slots.begin(), slots.end());
          for (int slot : slots) {
            if (!firstSlot) fprintf(header, ",");
            fprintf(header, "%d", slot);
            firstSlot = false;
          }
        }
      }
      if (firstSlot) fprintf(header, "0");
      fprintf(header, "};\n");
        int denseLookaheadDoneWordCount = 1;
        for (int count : denseDispatchWorkerCounts)
          denseLookaheadDoneWordCount = std::max(denseLookaheadDoneWordCount, (count + 63) / 64);
        fprintf(header, "static constexpr uint32_t kDenseLookaheadWindow = %du;\n", denseLookaheadWindow);
        fprintf(header, "static constexpr uint32_t kDenseLookaheadDoneWordCount = %du;\n", denseLookaheadDoneWordCount);
        fprintf(header, "static constexpr uint32_t kDenseLookaheadLocalPrereqs[%zu] = {", std::max<size_t>(1, denseLookaheadLocalPrereqs.size()));
        for (size_t i = 0; i < denseLookaheadLocalPrereqs.size(); ++i) fprintf(header, "%s%u", i ? "," : "", denseLookaheadLocalPrereqs[i]);
        if (denseLookaheadLocalPrereqs.empty()) fprintf(header, "0");
        fprintf(header, "};\n");
        fprintf(header, "static constexpr uint8_t kDenseLookaheadLocalPrereqKinds[%zu] = {", std::max<size_t>(1, denseLookaheadLocalPrereqKinds.size()));
        for (size_t i = 0; i < denseLookaheadLocalPrereqKinds.size(); ++i) fprintf(header, "%s%u", i ? "," : "", static_cast<unsigned>(denseLookaheadLocalPrereqKinds[i]));
        if (denseLookaheadLocalPrereqKinds.empty()) fprintf(header, "0");
        fprintf(header, "};\n");
        fprintf(header, "static constexpr uint32_t kDenseLookaheadGroupDestination[%d] = {", std::max(1, ownerReadyLayout.tokenCount));
        for (int token = 0; token < ownerReadyLayout.tokenCount; ++token) fprintf(header, "%s%u", token ? "," : "", static_cast<unsigned>(ownerReadyLayout.tokenProvenanceByLogicalToken[(size_t)token].consumerMTask));
        if (ownerReadyLayout.tokenCount == 0) fprintf(header, "0");
        fprintf(header, "};\n");
        fprintf(header, "static constexpr uint32_t kDenseLookaheadGroupProducerOwner[%d] = {", std::max(1, ownerReadyLayout.tokenCount));
        for (int token = 0; token < ownerReadyLayout.tokenCount; ++token) fprintf(header, "%s%u", token ? "," : "", static_cast<unsigned>(ownerReadyLayout.tokenProvenanceByLogicalToken[(size_t)token].producerOwner));
        if (ownerReadyLayout.tokenCount == 0) fprintf(header, "0");
        fprintf(header, "};\n");
        fprintf(header, "static constexpr uint32_t kDenseLookaheadGroupPublisher[%d] = {", std::max(1, ownerReadyLayout.tokenCount));
        for (int token = 0; token < ownerReadyLayout.tokenCount; ++token) fprintf(header, "%s%u", token ? "," : "", static_cast<unsigned>(ownerReadyLayout.tokenProvenanceByLogicalToken[(size_t)token].producerMTask));
        if (ownerReadyLayout.tokenCount == 0) fprintf(header, "0");
        fprintf(header, "};\n");
        fprintf(header, "static constexpr uint32_t kDenseLookaheadGroupSourceOffsets[%d] = {", ownerReadyLayout.tokenCount + 1);
        uint32_t sourceOffset = 0;
        for (int token = 0; token < ownerReadyLayout.tokenCount; ++token) {
          fprintf(header, "%s%u", token ? "," : "", sourceOffset);
          sourceOffset += static_cast<uint32_t>(ownerReadyLayout.sourceMTasksByLogicalToken[(size_t)token].size());
        }
        fprintf(header, "%s%u};\n", ownerReadyLayout.tokenCount ? "," : "", sourceOffset);
        fprintf(header, "static constexpr uint32_t kDenseLookaheadGroupSources[%u] = {", std::max(1u, sourceOffset));
        bool firstSource = true;
        for (const std::vector<int>& sources : ownerReadyLayout.sourceMTasksByLogicalToken) {
          for (int source : sources) { fprintf(header, "%s%u", firstSource ? "" : ",", static_cast<unsigned>(source)); firstSource = false; }
        }
        if (firstSource) fprintf(header, "0");
        fprintf(header, "};\n");
        fprintf(header, "struct MtDenseDispatchEntry { void (S%s::*fn)(); uint32_t waitBegin; uint32_t waitEnd; uint32_t storeBegin; uint32_t storeEnd; uint32_t localBegin; uint32_t localEnd;%s };\n",
                name.c_str(), (pushShadow || pushReady || pushReadyHint || tokenMaskHint) ? " uint32_t mtaskId;" : "");
        // GSIM_MT_DENSE_PUSH_READY_SHADOW threads the notifying worker's lane
        // through the tail so its sole-writer accounting stays atomic-free.
        if (denseDuty && pushShadow) {
          fprintf(header, "void stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtDutyLane, uint32_t mtPushShadowLane);\n");
        } else if (denseDuty && pushReadyHint) {
          fprintf(header, "void stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtDutyLane, uint32_t mtPushReadyHintLane);\n");
        } else if (denseDuty && tokenMaskHint) {
          fprintf(header, "void stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtDutyLane, uint32_t mtDenseTokenMaskLane);\n");
        } else if (denseDuty) {
          fprintf(header, "void stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtDutyLane);\n");
        } else if (pushShadow) {
          fprintf(header, "void stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtPushShadowLane);\n");
        } else if (pushReadyHint) {
          fprintf(header, "void stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtPushReadyHintLane);\n");
        } else if (tokenMaskHint) {
          fprintf(header, "void stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtDenseTokenMaskLane);\n");
        } else {
          fprintf(header, "void stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target);\n");
        }
      for (int t = 0; t < threadCount; t++) {
        fprintf(header, "static const MtDenseDispatchEntry kDenseDispatchTableW%d[%d];\n",
                t, std::max(1, denseDispatchWorkerCounts[(size_t)t]));
      }
    }
    fprintf(header, "#else\n");
    emitDenseDepCounts();
    emitDenseOwnerBankDiagnostics();
    emitDenseSuccOffsets();
    emitDenseSuccList();
    fprintf(header, "struct MtDenseMTaskVertex { std::atomic<uint32_t> depsDone{0}; };\n");
    fprintf(header, "MtDenseMTaskVertex mtDenseMTaskVertices[%d];\n", nMTasks);
    fprintf(header, "#endif\n");
  }
  fprintf(header, "void stepDenseThreadWorker(int threadId);\n");
  for (int i = 0; i < nMTasks; i++) fprintf(header, "void stepDenseMTask%d();\n", i);
  bool workerMajorText = mtUseDenseWorkerMajorText();
  std::vector<int> denseMTaskEmissionOrder;
  denseMTaskEmissionOrder.reserve((size_t)nMTasks);
  if (workerMajorText) {
    for (int owner = 0; owner < threadCount; owner++) {
      for (int mtaskId = 0; mtaskId < nMTasks; mtaskId++) {
        if (denseSchedule.mtaskThreadAssign[(size_t)mtaskId] == owner)
          denseMTaskEmissionOrder.push_back(mtaskId);
      }
    }
    Assert((int)denseMTaskEmissionOrder.size() == nMTasks,
           "worker-major text emission did not cover all dense MTasks");
    int originalRuns = nMTasks > 0 ? 1 : 0;
    for (int mtaskId = 1; mtaskId < nMTasks; mtaskId++) {
      if (denseSchedule.mtaskThreadAssign[(size_t)mtaskId] !=
          denseSchedule.mtaskThreadAssign[(size_t)mtaskId - 1]) originalRuns++;
    }
    int emittedRuns = nMTasks > 0 ? 1 : 0;
    for (int i = 1; i < nMTasks; i++) {
      if (denseSchedule.mtaskThreadAssign[(size_t)denseMTaskEmissionOrder[(size_t)i]] !=
          denseSchedule.mtaskThreadAssign[(size_t)denseMTaskEmissionOrder[(size_t)i - 1]]) emittedRuns++;
    }
    fprintf(stderr,
            "[mt-dense-worker-major-text] mtasks=%d original_owner_runs=%d emitted_owner_runs=%d threads=%d\n",
            nMTasks, originalRuns, emittedRuns, threadCount);
  } else {
    for (int mtaskId = 0; mtaskId < nMTasks; mtaskId++)
      denseMTaskEmissionOrder.push_back(mtaskId);
  }
  if (emitPhaseTimingEnabled()) {
    fprintf(stderr, "[emit-phase] Final.densePrologue.headerTables = %ld ms\n",
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - denseProHeaderTablesBegin).count());
  }
  if (emitPhaseTimingEnabled()) {
    long prologueMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - densePrologueBegin).count();
    fprintf(stderr, "[emit-phase] Final.densePrologue.total = %ld ms\n", prologueMs);
  }
  // GSIM_EMIT_TASK_LOCALS=1 (default off): freeze the reset machinery's member
  // footprint before the parallel body emission. SUPER_ASYNC_RESET supers
  // evaluate with subResetDense<id>() calls at their start AND end; those
  // helpers read/write class members directly (invisible to same-name body
  // locals), so every name they reference is excluded from localization in the
  // calling task.
  const bool emitTaskLocals = mtUseEmitTaskLocals();
  const int spinYieldEvery = mtDenseSpinYieldEvery();
  std::map<Node*, std::set<std::string>> denseAsyncResetReferenced;
  if (emitTaskLocals) {
    for (SuperNode* resetSuper : allReset) {
      if (resetSuper->superType != SUPER_ASYNC_RESET) continue;
      MtTaskLocalCollector collect;
      collect.walkSuper(resetSuper);
      std::set<std::string>& names = denseAsyncResetReferenced[resetSuper->resetNode];
      for (const std::string& name : collect.reads) names.insert(name);
      for (const std::string& name : collect.writes) names.insert(name);
      for (const std::string& name : collect.resetTouched) names.insert(name);
      for (const std::string& name : collect.nonRegNames) names.insert(name);
      for (const auto& kv : collect.typeNode) names.insert(kv.first);
    }
  }
  // Report counters only; order-independent sums, so relaxed atomics suffice
  // for the parallel emission units below.
  std::atomic<uint64_t> taskLocalsLocalizedTasks{0}, taskLocalsWorker0Excluded{0},
      taskLocalsExtExcluded{0}, taskLocalsAsyncHelperTasks{0}, taskLocalsLoads{0},
      taskLocalsStores{0}, taskLocalsCandidates{0}, taskLocalsResetExcluded{0},
      taskLocalsArrayClassNames{0};
  {
  EmitPhaseTimer denseBodyTimer("Final.denseMTaskBodies");
  // Each stepDenseMTaskN body is an independent emission unit (reads frozen
  // schedule + graph; per-super emission context flags are thread-local and
  // saved/restored around genSuperEval exactly as in the sequential loop).
  // Unit u renders denseMTaskEmissionOrder[u]; assembly replays buffers in
  // emission order, so output is byte-identical.
  emitUnitsParallel(denseMTaskEmissionOrder.size(), [this, &denseSchedule, &denseMTaskEmissionOrder, &ownerReadyLayout, denseBreakdownWindowLaBodyCodegen, emitTaskLocals, &denseAsyncResetReferenced, &taskLocalsLocalizedTasks, &taskLocalsWorker0Excluded, &taskLocalsExtExcluded, &taskLocalsAsyncHelperTasks, &taskLocalsLoads, &taskLocalsStores, &taskLocalsCandidates, &taskLocalsResetExcluded, &taskLocalsArrayClassNames, pscdBits, pscdVerify, pushShadow, &pscdProducerFields, &pscdConsumerShadow, &pscdConsumerTokenSlots](size_t unit) {
    int mtaskId = denseMTaskEmissionOrder[unit];

    const MtDenseMTask& mtask = denseSchedule.mtasks[mtaskId];
    emitFuncDecl(0, "void S%s::stepDenseMTask%d() {\n", name.c_str(), mtaskId);
    if (pushShadow) {
      // GSIM_MT_DENSE_PUSH_READY_SHADOW: push skip-soundness census at body
      // entry. Every dispatch path (lookahead inline, all three tail paths,
      // strict wait chain, serial fallback) calls this function only AFTER its
      // pull readiness checks, so a nonzero pending counter here means the
      // push mapping would have skipped a task pull readiness authorized -
      // a missing edge in the generated census. The original pull readiness
      // remains authoritative; the shadow only counts and reports.
      emitBodyLock(1, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
      emitBodyLock(1, "{ // GSIM_MT_DENSE_PUSH_READY_SHADOW: pending must be zero at pull-authorized entry\n");
      emitBodyLock(2, "const uint32_t mtPushShadowPending = mtDensePushShadowPending[%d].load(std::memory_order_acquire);\n", mtaskId);
      emitBodyLock(2, "if (mtPushShadowPending != 0u) {\n");
      emitBodyLock(3, "mtDensePushShadowMismatchCount.fetch_add(1, std::memory_order_relaxed);\n");
      emitBodyLock(3, "if (!mtDensePushShadowMismatchReported[%d]) {\n", mtaskId);
      emitBodyLock(4, "mtDensePushShadowMismatchReported[%d] = 1;\n", mtaskId);
      emitBodyLock(4, "fprintf(stderr, \"[push-shadow] skip-soundness mismatch mtask=%%d cycle=%%lu pending=%%u fanin=%%u mismatches=%%lu\\n\", %d, (unsigned long) cycles, mtPushShadowPending, kDensePushShadowFanin[%d], (unsigned long) mtDensePushShadowMismatchCount.load(std::memory_order_relaxed));\n", mtaskId, mtaskId);
      emitBodyLock(3, "}\n");
      emitBodyLock(2, "}\n");
      emitBodyLock(1, "}\n");
      emitBodyLock(1, "#endif\n");
    }
    // GSIM_EMIT_TASK_LOCALS=1 (default off): value-typed locals shadowing the
    // eligible scalar register-state members this body reads or writes. The
    // loads happen once at body entry; every in-body reference (including the
    // dead $old$ snapshots the dense path still computes) resolves to the
    // local; written members are stored back before the body returns, ahead of
    // the dispatcher's token release stores. Exclusions: worker0-owned tasks,
    // external/DPI tasks, members referenced by the subResetDense helpers
    // called around SUPER_ASYNC_RESET supers, REG_RESET-referenced names, and
    // arrays/memories (which keep direct member access).
    std::vector<std::pair<std::string, Node*>> taskLocalLoads;
    std::vector<std::string> taskLocalStores;
    if (emitTaskLocals) {
      bool taskExcluded = false;
      bool asyncHelperTask = false;
      MtTaskLocalCollector collect;
      std::set<std::string> excluded;
      const int taskLocalOwner = mtaskId < static_cast<int>(denseSchedule.mtaskThreadAssign.size())
                                   ? denseSchedule.mtaskThreadAssign[(size_t)mtaskId] : -1;
      if (taskLocalOwner == 0) taskExcluded = true;  // worker0-only tasks stay direct-member
      for (int sccId : mtask.sccIds) {
        if (sccId < 0 || sccId >= static_cast<int>(denseSchedule.sccs.size())) continue;
        for (int cppId : denseSchedule.sccs[(size_t)sccId].cppIds) {
          auto superIter = cppId2Super.find(cppId);
          if (superIter == cppId2Super.end() || !superIter->second) continue;
          SuperNode* super = superIter->second;
          if (super->superType == SUPER_EXTMOD) taskExcluded = true;  // external/DPI side effects
          if (super->superType == SUPER_ASYNC_RESET) {
            asyncHelperTask = true;
            auto helperNames = denseAsyncResetReferenced.find(super->resetNode);
            if (helperNames != denseAsyncResetReferenced.end())
              excluded.insert(helperNames->second.begin(), helperNames->second.end());
          }
          for (Node* member : super->member) {
            if (member->isExt()) taskExcluded = true;  // DPI boundary: members passed by reference
          }
          collect.walkSuper(super);
        }
      }
      taskLocalsArrayClassNames.fetch_add(collect.nonRegNames.size(), std::memory_order_relaxed);
      taskLocalsCandidates.fetch_add(collect.typeNode.size(), std::memory_order_relaxed);
      if (asyncHelperTask) taskLocalsAsyncHelperTasks.fetch_add(1, std::memory_order_relaxed);
      if (taskExcluded) {
        if (taskLocalOwner == 0) taskLocalsWorker0Excluded.fetch_add(1, std::memory_order_relaxed);
        else taskLocalsExtExcluded.fetch_add(1, std::memory_order_relaxed);
      } else {
        excluded.insert(collect.resetTouched.begin(), collect.resetTouched.end());
        excluded.insert(collect.nonRegNames.begin(), collect.nonRegNames.end());
        for (const auto& kv : collect.typeNode) {
          if (excluded.count(kv.first)) {
            taskLocalsResetExcluded.fetch_add(1, std::memory_order_relaxed);
            continue;
          }
          taskLocalLoads.push_back(kv);
          if (collect.writes.count(kv.first)) taskLocalStores.push_back(kv.first);
        }
      }
      taskLocalsLocalizedTasks.fetch_add(taskLocalLoads.empty() ? 0 : 1, std::memory_order_relaxed);
      taskLocalsLoads.fetch_add(taskLocalLoads.size(), std::memory_order_relaxed);
      taskLocalsStores.fetch_add(taskLocalStores.size(), std::memory_order_relaxed);
      if (!taskLocalLoads.empty()) {
        emitBodyLock(1, "// task-locals: %zu loaded, %zu written back (GSIM_EMIT_TASK_LOCALS)\n",
                     taskLocalLoads.size(), taskLocalStores.size());
        for (const auto& kv : taskLocalLoads) {
          emitBodyLock(1, "%s %s = this->%s;\n", widthUType(kv.second->width).c_str(),
                       kv.first.c_str(), kv.first.c_str());
        }
      }
    }
    if (pscdBits && !pscdProducerFields[(size_t)mtaskId].empty()) {
      // GSIM_EMIT_PSCD_BITS: pre-body snapshots of the committed outputs this
      // producer owns (before any body statement writes them).
      emitBodyLock(1, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
      emitBodyLock(1, "{ // GSIM_EMIT_PSCD_BITS: pre-body output snapshots\n");
      for (const MtPscdFieldRef& fr : pscdProducerFields[(size_t)mtaskId]) {
        emitBodyLock(2, "mtDensePscdOldVal[%d] = (uint64_t)(%s);\n", fr.slot, fr.name.c_str());
      }
      emitBodyLock(1, "}\n");
      emitBodyLock(1, "#endif\n");
    }
    if (pscdVerify && !pscdConsumerShadow[(size_t)mtaskId].empty()) {
      // GSIM_PSCD_VERIFY: shadow cross-check at body entry. The original
      // consumer OR-compare is re-derived from previous-entry snapshots of the
      // inputs crossing this task's tokens and compared against the OR of the
      // producers' change bytes. Read-only shadow — the body itself is
      // unchanged. The first entry only primes the snapshots.
      emitBodyLock(1, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
      emitBodyLock(1, "{ // GSIM_PSCD_VERIFY: shadow cross-check (change-bits == consumer OR-compare)\n");
      emitBodyLock(2, "if (!mtDensePscdConsumerPrimed[%d]) {\n", mtaskId);
      for (const MtPscdFieldRef& fr : pscdConsumerShadow[(size_t)mtaskId]) {
        emitBodyLock(3, "mtDensePscdShadowPrev[%d] = (uint64_t)(%s);\n", fr.slot, fr.name.c_str());
      }
      emitBodyLock(3, "mtDensePscdConsumerPrimed[%d] = 1;\n", mtaskId);
      emitBodyLock(2, "} else {\n");
      emitBodyLock(3, "bool pscdOrig = false;\n");
      for (const MtPscdFieldRef& fr : pscdConsumerShadow[(size_t)mtaskId]) {
        emitBodyLock(3, "{ const uint64_t pscdNow = (uint64_t)(%s); if (pscdNow != mtDensePscdShadowPrev[%d]) pscdOrig = true; mtDensePscdShadowPrev[%d] = pscdNow; }\n",
                     fr.name.c_str(), fr.slot, fr.slot);
      }
      emitBodyLock(3, "// Reset-window cycles (a reset application fired this or the previous\n");
      emitBodyLock(3, "// cycle): reset-supers may have written inputs outside any producer\n");
      emitBodyLock(3, "// compare, so the cross-check is vacuously skipped. Snapshots are\n");
      emitBodyLock(3, "// still updated, keeping the next window clean.\n");
      emitBodyLock(3, "if (!(mtDensePscdResetActive || mtDensePscdResetActivePrev)) {\n");
      emitBodyLock(4, "bool pscdSummary = false;\n");
      for (int slot : pscdConsumerTokenSlots[(size_t)mtaskId]) {
        emitBodyLock(4, "if (mtDenseChangeEpoch[%d] == mtDenseCycleEpoch) pscdSummary = true;\n", slot);
      }
      emitBodyLock(4, "if (pscdOrig != pscdSummary) {\n");
      emitBodyLock(5, "mtPscdMismatchCount.fetch_add(1, std::memory_order_relaxed);\n");
      emitBodyLock(5, "if (!mtPscdMismatchReported[%d]) {\n", mtaskId);
      emitBodyLock(6, "mtPscdMismatchReported[%d] = 1;\n", mtaskId);
      emitBodyLock(6, "fprintf(stderr, \"[pscd-verify] mismatch mtask=%%d cycle=%%lu orig=%%d summary=%%d\\n\", %d, (unsigned long) cycles, (int) pscdOrig, (int) pscdSummary);\n", mtaskId);
      emitBodyLock(5, "}\n");
      emitBodyLock(4, "}\n");
      emitBodyLock(3, "}\n");
      emitBodyLock(2, "}\n");
      emitBodyLock(1, "}\n");
      emitBodyLock(1, "#endif\n");
    }
    if (denseBreakdownWindowLaBodyCodegen) {
      // Body-only lookahead timing: one conditional steady_clock wrap inside the
      // MTask body function covers every dispatch site (inline fast path plus all
      // three tail paths) without duplicating the body. Ready waits, lookahead
      // scan, and token stores stay outside because they live in the dispatch
      // layer, not the body. mtaskId and owner are baked generation constants.
      const int laBodyOwner = denseSchedule.mtaskThreadAssign[(size_t)mtaskId];
      emitBodyLock(1, "const int mtDenseBreakdownWindowLaBodySlot = mtDenseBreakdownWindowAllOwnerBodyMode ? mtDenseBreakdownWindowCurrentSlot : -1;\n");
      emitBodyLock(1, "const int mtDenseBreakdownWindowLaBodyRecord = kDenseBreakdownWindowAllOwnerMTaskRecordIndex[%d];\n", mtaskId);
      emitBodyLock(1, "std::chrono::steady_clock::time_point mtDenseBreakdownWindowLaBodyBegin;\n");
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownWindowLaBodySlot >= 0)) {\n");
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownWindowLaBodyRecord < 0 || mtDenseBreakdownWindowLaBodyRecord >= kDenseBreakdownWindowAllOwnerMTaskStorageCount || mtDenseBreakdownWindowLaBodySlot >= kDenseBreakdownWindowMaxCycles || cycles < mtDenseBreakdownWindowStart || cycles - mtDenseBreakdownWindowStart >= mtDenseBreakdownWindowCycles || cycles != mtDenseBreakdownWindowCycleNumbers[mtDenseBreakdownWindowLaBodySlot])) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] lookahead body window slot mismatch\\n\"); abort(); }\n");
      emitBodyLock(2, "mtDenseBreakdownWindowLaBodyBegin = std::chrono::steady_clock::now();\n");
      emitBodyLock(1, "}\n");
      emitBodyLock(1, "const int mtDenseBreakdownWindowLaBodyOwner = %d;\n", laBodyOwner);
      emitBodyLock(1, "const int mtDenseBreakdownWindowLaBodyStoreCount = %d;\n", (int)ownerReadyLayout.storeSlotsByMTask[(size_t)mtaskId].size());
    }
    // Sparse-in-dense: emit members ASCENDING by cppId (proven valid topo order of intra-MTask
    // forward activation edges: 0 backward in cppId order). Gather then sort.
    std::vector<int> memberCppIds;
    for (int sccId : mtask.sccIds) {
      Assert(sccId >= 0 && sccId < static_cast<int>(denseSchedule.sccs.size()), "dense schedule scc index out of range");
      const MtDenseScc& scc = denseSchedule.sccs[(size_t)sccId];
      for (int cppId : scc.cppIds) memberCppIds.push_back(cppId);
    }
    for (int cppId : memberCppIds) {
      auto superIter = cppId2Super.find(cppId);
      if (superIter == cppId2Super.end() || !superIter->second) continue;
      SuperNode* super = superIter->second;
      emitBodyLock(1, "{\n");
      emitBodyLock(2, "std::chrono::steady_clock::time_point mtProfileDenseTaskBegin;\n");
      emitBodyLock(2, "if (unlikely(mtProfileEnabled)) mtProfileDenseTaskBegin = std::chrono::steady_clock::now();\n");
      genSuperEval(super, "activeFlags[0]", "", 2, false);
      emitBodyLock(2, "if (unlikely(mtProfileEnabled)) recordMtProfileTask(%d, true, std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileDenseTaskBegin).count());\n", cppId);
      emitBodyLock(1, "}\n");
    }
    if (denseBreakdownWindowLaBodyCodegen) {
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownWindowLaBodySlot >= 0)) {\n");
      emitBodyLock(2, "const std::chrono::steady_clock::time_point mtDenseBreakdownWindowLaBodyEnd = std::chrono::steady_clock::now();\n");
      emitBodyLock(2, "MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowLaBodyEntry = mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowLaBodySlot][mtDenseBreakdownWindowLaBodyRecord];\n");
      emitBodyLock(2, "mtDenseBreakdownWindowLaBodyEntry.mtaskId = %d;\n", mtaskId);
      emitBodyLock(2, "mtDenseBreakdownWindowLaBodyEntry.ownerThreadId = (uint16_t)mtDenseBreakdownWindowLaBodyOwner;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowLaBodyEntry.readyTokenStoreCount = (uint16_t)mtDenseBreakdownWindowLaBodyStoreCount;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowLaBodyEntry.releaseEndOffsetNs = UINT64_MAX;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowLaBodyEntry.bodyStartOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowLaBodyBegin - mtDenseBreakdownWindowEpoch).count();\n");
      emitBodyLock(2, "mtDenseBreakdownWindowLaBodyEntry.bodyEndOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowLaBodyEnd - mtDenseBreakdownWindowEpoch).count();\n");
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownWindowLaBodyEntry.bodyEndOffsetNs < mtDenseBreakdownWindowLaBodyEntry.bodyStartOffsetNs)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] lookahead body timeline underflow\\n\"); abort(); }\n");
      emitBodyLock(2, "mtDenseBreakdownWindowLaBodyEntry.bodyNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowLaBodyEnd - mtDenseBreakdownWindowLaBodyBegin).count();\n");
      emitBodyLock(2, "const uint8_t mtDenseBreakdownWindowLaBodySeenCount = ++ mtDenseBreakdownWindowLaBodySeen[mtDenseBreakdownWindowLaBodySlot][mtDenseBreakdownWindowLaBodyRecord];\n");
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownWindowLaBodySeenCount != 1)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] lookahead body duplicate sample\\n\"); abort(); }\n");
      emitBodyLock(1, "}\n");
    }
    if (!taskLocalStores.empty()) {
      emitBodyLock(1, "// task-locals write-back before return (GSIM_EMIT_TASK_LOCALS)\n");
      for (const std::string& taskLocalName : taskLocalStores) {
        emitBodyLock(1, "this->%s = %s;\n", taskLocalName.c_str(), taskLocalName.c_str());
      }
    }
    if (pscdBits && !pscdProducerFields[(size_t)mtaskId].empty()) {
      // GSIM_EMIT_PSCD_BITS: post-body change verdicts per committed output
      // (after task-locals write-backs, before the dispatcher's store phase
      // folds them into token change bytes).
      emitBodyLock(1, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
      emitBodyLock(1, "{ // GSIM_EMIT_PSCD_BITS: change verdicts (post-body vs pre-body)\n");
      for (const MtPscdFieldRef& fr : pscdProducerFields[(size_t)mtaskId]) {
        emitBodyLock(2, "mtDensePscdFieldChanged[%d] = (uint8_t)((uint64_t)(%s) != mtDensePscdOldVal[%d]);\n",
                     fr.slot, fr.name.c_str(), fr.slot);
      }
      emitBodyLock(1, "}\n");
      emitBodyLock(1, "#endif\n");
    }
    emitBodyLock(0, "}\n");
  });
  }
  if (emitTaskLocals) {
    fprintf(stderr,
            "[mt-dense-task-locals] mtasks=%d localized_tasks=%llu worker0_excluded=%llu ext_excluded=%llu async_helper_tasks=%llu loads=%llu stores=%llu candidates=%llu reset_excluded=%llu array_class_names=%llu\n",
            nMTasks,
            (unsigned long long)taskLocalsLocalizedTasks.load(),
            (unsigned long long)taskLocalsWorker0Excluded.load(),
            (unsigned long long)taskLocalsExtExcluded.load(),
            (unsigned long long)taskLocalsAsyncHelperTasks.load(),
            (unsigned long long)taskLocalsLoads.load(),
            (unsigned long long)taskLocalsStores.load(),
            (unsigned long long)taskLocalsCandidates.load(),
            (unsigned long long)taskLocalsResetExcluded.load(),
            (unsigned long long)taskLocalsArrayClassNames.load());
  }

  auto emitFixedDenseThreadWorker = [&](const char* funcName) {
    emitFuncDecl(0, "void S%s::%s(int threadId) {\n", name.c_str(), funcName);
    if (pushReady) {
      // GSIM_MT_DENSE_PUSH_READY hybrid dispatch: pool workers 1..N-1 leave
      // the pull lane immediately; worker0 (the main thread) keeps the whole
      // original pull path below, token waits included. Compile-gated so a
      // macro-off build falls back to pure pull for every worker.
      emitBodyLock(1, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
      emitBodyLock(1, "if (threadId != 0) { %s(threadId); return; }\n",
                   pushReadyBitmap ? "stepDensePushReadyBitmapWorker" : "stepDensePushThreadWorker");
      emitBodyLock(1, "#endif\n");
    }
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(1, "const int mtDenseBreakdownWindowSlot = mtDenseBreakdownWindowCurrentSlot;\n");
      emitBodyLock(1, "const bool mtDenseBreakdownWindow = mtDenseBreakdownWindowSlot >= 0;\n");
      emitBodyLock(1, "MtDenseBreakdownWindowWorker *mtDenseBreakdownWindowWorker = nullptr;\n");
      emitBodyLock(1, "int mtDenseBreakdownWindowWorker0MTaskNext = 0;\n");
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownWindow)) {\n");
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownWindowSlot >= kDenseBreakdownWindowMaxCycles || threadId < 0 || threadId >= kDenseBreakdownWindowThreadCount)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] invalid window worker lane\\n\"); abort(); }\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker = &mtDenseBreakdownWindowWorkers[mtDenseBreakdownWindowSlot][threadId];\n");
      emitBodyLock(2, "const uint64_t mtDenseBreakdownWindowStartOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDenseBreakdownWindowEpoch).count();\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker->startOffsetNs = mtDenseBreakdownWindowStartOffsetNs;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker->finishOffsetNs = 0;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker->blockedWaitNs = 0;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker->bodyNs = 0;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker->controlNs = 0;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWaitCounts[mtDenseBreakdownWindowSlot][threadId] = 0;\n");
      emitBodyLock(1, "}\n");
    }
    if (denseBreakdownProfileCodegen) {
      if (denseBreakdownWindowCodegen) {
        emitBodyLock(1, "const bool mtDenseBreakdownProfile = mtDenseBreakdownProfileEnabled && mtDenseBreakdownWindow;\n");
      } else {
        emitBodyLock(1, "const bool mtDenseBreakdownProfile = mtDenseBreakdownProfileEnabled;\n");
      }
      emitBodyLock(1, "std::chrono::steady_clock::time_point mtDenseBreakdownDispatchBegin;\n");
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownProfile)) mtDenseBreakdownDispatchBegin = std::chrono::steady_clock::now();\n");
    }

    if (denseDuty) emitBodyLock(1, "MtDenseDutyGuard mtDutyGuard(mtDutyEnabled && threadId >= 0 && threadId <= %d ? &mtDutyLanes[threadId].spanNs : nullptr);\n", threadCount);
    emitBodyLock(1, "bool evenCycle = (cycles & 1) == 0;\n");
    emitBodyLock(1, "switch (threadId) {\n");
    for (int t = 0; t < threadCount; t++) {
      emitBodyLock(2, "case %d: {\n", t);
      if (denseLookahead) {
        emitBodyLock(3, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
          emitBodyLock(3, "{ const uint8_t target = evenCycle ? uint8_t{1} : uint8_t{0};\n");
          int tablePosition = 0;
          for (int mtaskId = 0; mtaskId < nMTasks; ++mtaskId) {
            if (denseSchedule.mtaskThreadAssign[(size_t)mtaskId] != t) continue;
            emitBodyLock(4, "{ bool mtDenseInlineReady = true;\n");
            if (pushReadyHint && t != 0) {
              // GSIM_MT_DENSE_PUSH_READY_HINT: workers 1..N-1 replace the
              // whole wait-list conjunction with one acquire of the per-task
              // ready byte (armed only after every mapped producer edge has
              // been paid this cycle). Worker0's case keeps the original
              // token checks below - its lane stays fully original pull.
              emitBodyLock(5, "mtDenseInlineReady = (mtDensePushReadyHint[%d].load(std::memory_order_acquire) != 0u);\n", mtaskId);
            } else if (tokenMaskHint && t != 0) {
              // GSIM_MT_DENSE_TOKEN_MASK_HINT: workers 1..N-1 collapse the
              // whole cross-token wait conjunction into one acquire of the
              // per-consumer mask word - mask == expected means every remote
              // publisher has fetch_or-ed its bit this cycle. Worker0's case
              // keeps the original token checks below - its lane stays fully
              // original pull.
              emitBodyLock(5, "mtDenseInlineReady = (mtDenseTokenMask[%d].load(std::memory_order_acquire) == kDenseTokenMaskExpected[%d]);\n", mtaskId, mtaskId);
            } else {
              for (int slot : ownerReadyLayout.waitSlotsByMTask[(size_t)mtaskId]) {
                emitBodyLock(5, "mtDenseInlineReady &= (mtDenseOwnerReadyTokens[%d].ready.load(std::memory_order_acquire) == target);\n", slot);
              }
            }
            if (denseDuty && pushShadow) {
              emitBodyLock(5, "if (!mtDenseInlineReady) { stepDenseLookaheadTail(kDenseDispatchTableW%d, kDenseDispatchTableW%d + %d, %uu, target, %du, %du); return; }\n",
                           t, t, denseDispatchWorkerCounts[(size_t)t], static_cast<unsigned>(tablePosition), t, t);
            } else if (denseDuty && pushReadyHint) {
              emitBodyLock(5, "if (!mtDenseInlineReady) { stepDenseLookaheadTail(kDenseDispatchTableW%d, kDenseDispatchTableW%d + %d, %uu, target, %du, %du); return; }\n",
                           t, t, denseDispatchWorkerCounts[(size_t)t], static_cast<unsigned>(tablePosition), t, t);
            } else if (denseDuty && tokenMaskHint) {
              emitBodyLock(5, "if (!mtDenseInlineReady) { stepDenseLookaheadTail(kDenseDispatchTableW%d, kDenseDispatchTableW%d + %d, %uu, target, %du, %du); return; }\n",
                           t, t, denseDispatchWorkerCounts[(size_t)t], static_cast<unsigned>(tablePosition), t, t);
            } else if (denseDuty) {
              emitBodyLock(5, "if (!mtDenseInlineReady) { stepDenseLookaheadTail(kDenseDispatchTableW%d, kDenseDispatchTableW%d + %d, %uu, target, %du); return; }\n",
                           t, t, denseDispatchWorkerCounts[(size_t)t], static_cast<unsigned>(tablePosition), t);
            } else if (pushShadow) {
              emitBodyLock(5, "if (!mtDenseInlineReady) { stepDenseLookaheadTail(kDenseDispatchTableW%d, kDenseDispatchTableW%d + %d, %uu, target, %du); return; }\n",
                           t, t, denseDispatchWorkerCounts[(size_t)t], static_cast<unsigned>(tablePosition), t);
            } else if (pushReadyHint) {
              emitBodyLock(5, "if (!mtDenseInlineReady) { stepDenseLookaheadTail(kDenseDispatchTableW%d, kDenseDispatchTableW%d + %d, %uu, target, %du); return; }\n",
                           t, t, denseDispatchWorkerCounts[(size_t)t], static_cast<unsigned>(tablePosition), t);
            } else if (tokenMaskHint) {
              emitBodyLock(5, "if (!mtDenseInlineReady) { stepDenseLookaheadTail(kDenseDispatchTableW%d, kDenseDispatchTableW%d + %d, %uu, target, %du); return; }\n",
                           t, t, denseDispatchWorkerCounts[(size_t)t], static_cast<unsigned>(tablePosition), t);
            } else {
              emitBodyLock(5, "if (!mtDenseInlineReady) { stepDenseLookaheadTail(kDenseDispatchTableW%d, kDenseDispatchTableW%d + %d, %uu, target); return; }\n",
                           t, t, denseDispatchWorkerCounts[(size_t)t], static_cast<unsigned>(tablePosition));
            }
            emitBodyLock(5, "stepDenseMTask%d();\n", mtaskId);
            if (pushShadow)
              emitBodyLock(5, "mtDensePushShadowNotify(%d, (uint32_t)threadId);\n", mtaskId);
            if (pushReady)
              emitBodyLock(5, "mtDensePushReadyNotify(%d);\n", mtaskId);
            if (pushReadyHint)
              emitBodyLock(5, "mtDensePushReadyHintNotify(%d);\n", mtaskId);
            if (pscdBits)
              emitBodyLock(5, "mtDensePscdRelease(kDenseOwnerReadyStoreOffsets[%d], kDenseOwnerReadyStoreOffsets[%d]);\n", mtaskId, mtaskId + 1);
            for (int slot : ownerReadyLayout.storeSlotsByMTask[(size_t)mtaskId]) {
              if (edgeTiming)
                emitBodyLock(5, "mtDenseEdgeFireRdtsc[%d] = mtEdgeTimingRdtsc();\n", slot);
              if (tokenMaskHint)
                emitBodyLock(5, "mtDenseTokenMask[kDenseTokenMaskConsumerBySlot[%d]].fetch_or(kDenseTokenMaskBitBySlot[%d], std::memory_order_release);\n", slot, slot);
              emitBodyLock(5, "mtDenseOwnerReadyTokens[%d].ready.store(target, std::memory_order_release);\n", slot);
            }
            emitBodyLock(4, "}\n");
            ++tablePosition;
          }
          emitBodyLock(3, "}\n");
        emitBodyLock(3, "#else\n");
      }
      for (int mtaskId = 0; mtaskId < nMTasks; mtaskId++) {
        if (denseSchedule.mtaskThreadAssign[mtaskId] != t) continue;
        const bool skipDenseWait = staticEmptyElide && denseRuntimeDepCounts[(size_t)mtaskId] == 0;
        if (!ownerReadyFlags) {
          if (!skipDenseWait) {
            // Fixed dependency-counter fallback keeps the historical 256-spin yield budget.
            emitBodyLock(3, "{ const uint32_t target = evenCycle ? kDenseMTaskDepCount[%d] : 0u;\n", mtaskId);
            emitBodyLock(3, "  unsigned ct = 0;\n");
            emitBodyLock(3, "  while (mtDenseMTaskVertices[%d].depsDone.load(std::memory_order_acquire) != target) {\n", mtaskId);
            if (spinYieldEvery == 0) emitBodyLock(4, "mtWorkerPoolPause(); }\n");
            else emitBodyLock(4, "mtWorkerPoolPause(); if (++ct > %d) { ct = 0; std::this_thread::yield(); } }\n", spinYieldEvery);
            emitBodyLock(3, "}\n");
          }
        } else {
          emitBodyLock(3, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
          for (int slot : ownerReadyLayout.waitSlotsByMTask[(size_t)mtaskId]) {
            int edgeWidx = -1;
            if (edgeTiming) {
              auto edgeWidxIt = edgeTimingWaitIndexOfSlot.find(slot);
              Assert(edgeWidxIt != edgeTimingWaitIndexOfSlot.end(),
                     "edge-timing wait slot %d missing from canonical numbering", slot);
              edgeWidx = edgeWidxIt->second;
            }
            if (denseBreakdownProfileCodegen) {
              emitBodyLock(3, "{ const uint8_t target = evenCycle ? uint8_t{1} : uint8_t{0};\n");
              if (denseBreakdownWindowCodegen) {
                emitBodyLock(3, "  if (unlikely(mtDenseBreakdownProfile && !(mtDenseBreakdownWindow && mtDenseBreakdownWindowFinishOnlyMode))) {\n");
              } else {
                emitBodyLock(3, "  if (unlikely(mtDenseBreakdownProfile)) {\n");
              }
              emitBodyLock(4, "bool mtDenseBreakdownBlocked = false;\n");
              emitBodyLock(4, "std::chrono::steady_clock::time_point mtDenseBreakdownBlockedBegin;\n");
              if (edgeTiming) emitBodyLock(4, "uint64_t mtEdgeBlockedT0 = 0;\n");
              emitBodyLock(4, "unsigned ct = 0;\n");
              emitBodyLock(4, "while (mtDenseOwnerReadyTokens[%d].ready.load(std::memory_order_acquire) != target) {\n", slot);
              emitBodyLock(5, "if (!mtDenseBreakdownBlocked) { mtDenseBreakdownBlocked = true; mtDenseBreakdownBlockedBegin = std::chrono::steady_clock::now(); }\n");
              if (edgeTiming) emitBodyLock(5, "if (mtEdgeBlockedT0 == 0) mtEdgeBlockedT0 = mtEdgeTimingRdtsc();\n");
              if (spinYieldEvery == 0) emitBodyLock(5, "mtWorkerPoolPause();\n");
              else emitBodyLock(5, "mtWorkerPoolPause(); if (++ct > %d) { ct = 0; std::this_thread::yield(); }\n", spinYieldEvery);
              emitBodyLock(4, "}\n");
              emitBodyLock(4, "if (mtDenseBreakdownBlocked) {\n");
              if (edgeTiming)
                emitBodyLock(5, "mtDenseEdgeBlockedNs[%d] += mtEdgeTimingRdtsc() - mtEdgeBlockedT0; mtDenseEdgeWaits[%d] ++;\n", edgeWidx, edgeWidx);
              emitBodyLock(5, "MtDenseBreakdownWorker &mtDenseBreakdownWorker = mtDenseBreakdownWorkers[threadId];\n");
              if (denseBreakdownWindowCodegen) {
                emitBodyLock(5, "const std::chrono::steady_clock::time_point mtDenseBreakdownBlockedEnd = std::chrono::steady_clock::now();\n");
                emitBodyLock(5, "uint64_t mtDenseBreakdownBlockedNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownBlockedEnd - mtDenseBreakdownBlockedBegin).count();\n");
              } else {
                emitBodyLock(5, "uint64_t mtDenseBreakdownBlockedNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDenseBreakdownBlockedBegin).count();\n");
              }
              emitBodyLock(5, "if (unlikely(UINT64_MAX - mtDenseBreakdownWorker.blockedWaitNs < mtDenseBreakdownBlockedNs || mtDenseBreakdownWorker.blockedWaitCount == UINT64_MAX)) { fprintf(stderr, \"[mt-dense-breakdown] blocked-wait counter overflow\\n\"); abort(); }\n");
              emitBodyLock(5, "mtDenseBreakdownWorker.blockedWaitNs += mtDenseBreakdownBlockedNs;\n");
              emitBodyLock(5, "mtDenseBreakdownWorker.blockedWaitCount += 1;\n");
              if (denseBreakdownWindowCodegen) {
                emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindow)) { if (unlikely(UINT64_MAX - mtDenseBreakdownWindowWorker->blockedWaitNs < mtDenseBreakdownBlockedNs)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window blocked-wait overflow\\n\"); abort(); } mtDenseBreakdownWindowWorker->blockedWaitNs += mtDenseBreakdownBlockedNs; }\n");
              }
              if (denseBreakdownWindowCodegen) {
                emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindow)) { uint32_t &mtDenseBreakdownWindowWaitCount = mtDenseBreakdownWindowWaitCounts[mtDenseBreakdownWindowSlot][threadId]; const uint32_t mtDenseBreakdownWindowWaitCapacity = (uint32_t)(kDenseBreakdownWindowWaitLaneOffsets[threadId + 1] - kDenseBreakdownWindowWaitLaneOffsets[threadId]); if (unlikely(mtDenseBreakdownWindowWaitCount >= mtDenseBreakdownWindowWaitCapacity)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window ready-wait record overflow\\n\"); abort(); } MtDenseBreakdownWindowWait &mtDenseBreakdownWindowWait = mtDenseBreakdownWindowWaits[mtDenseBreakdownWindowSlot][kDenseBreakdownWindowWaitLaneOffsets[threadId] + mtDenseBreakdownWindowWaitCount ++]; mtDenseBreakdownWindowWait.cycleSlot = (uint16_t)mtDenseBreakdownWindowSlot; mtDenseBreakdownWindowWait.threadId = (uint16_t)threadId; mtDenseBreakdownWindowWait.consumerMtaskId = %d; mtDenseBreakdownWindowWait.readySlot = %d; mtDenseBreakdownWindowWait.blockedNs = mtDenseBreakdownBlockedNs; }\n", mtaskId, slot);
                emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindow)) { uint64_t mtDenseBreakdownWindowWaitEndOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownBlockedEnd - mtDenseBreakdownWindowEpoch).count(); if (unlikely(mtDenseBreakdownWindowWaitEndOffsetNs < mtDenseBreakdownBlockedNs)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window ready-wait timeline underflow\\n\"); abort(); } const uint32_t mtDenseBreakdownWindowWaitIndex = mtDenseBreakdownWindowWaitCounts[mtDenseBreakdownWindowSlot][threadId] - 1; mtDenseBreakdownWindowWaits[mtDenseBreakdownWindowSlot][kDenseBreakdownWindowWaitLaneOffsets[threadId] + mtDenseBreakdownWindowWaitIndex].endOffsetNs = mtDenseBreakdownWindowWaitEndOffsetNs; }\n");
              }
              if (denseBreakdownWindowCodegen) {
                emitBodyLock(4, "} else if (unlikely(mtDenseBreakdownWindow && mtDenseBreakdownWindowCausalChainMode)) {\n");
                emitBodyLock(5, "uint32_t &mtDenseBreakdownWindowWaitCount = mtDenseBreakdownWindowWaitCounts[mtDenseBreakdownWindowSlot][threadId];\n");
                emitBodyLock(5, "const uint32_t mtDenseBreakdownWindowWaitCapacity = (uint32_t)(kDenseBreakdownWindowWaitLaneOffsets[threadId + 1] - kDenseBreakdownWindowWaitLaneOffsets[threadId]);\n");
                emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowWaitCount >= mtDenseBreakdownWindowWaitCapacity)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window causal wait record overflow\\n\"); abort(); }\n");
                emitBodyLock(5, "const std::chrono::steady_clock::time_point mtDenseBreakdownWindowWaitEnd = std::chrono::steady_clock::now();\n");
                emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowWaitEnd < mtDenseBreakdownWindowEpoch)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window causal wait clock regression\\n\"); abort(); }\n");
                emitBodyLock(5, "MtDenseBreakdownWindowWait &mtDenseBreakdownWindowWait = mtDenseBreakdownWindowWaits[mtDenseBreakdownWindowSlot][kDenseBreakdownWindowWaitLaneOffsets[threadId] + mtDenseBreakdownWindowWaitCount ++];\n");
                emitBodyLock(5, "mtDenseBreakdownWindowWait.cycleSlot = (uint16_t)mtDenseBreakdownWindowSlot; mtDenseBreakdownWindowWait.threadId = (uint16_t)threadId; mtDenseBreakdownWindowWait.consumerMtaskId = %d; mtDenseBreakdownWindowWait.readySlot = %d; mtDenseBreakdownWindowWait.blockedNs = 0; mtDenseBreakdownWindowWait.endOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowWaitEnd - mtDenseBreakdownWindowEpoch).count();\n", mtaskId, slot);
                emitBodyLock(4, "}\n");
              } else {
                emitBodyLock(4, "}\n");
              }
              emitBodyLock(3, "  } else {\n");
              if (edgeTiming) {
                emitBodyLock(4, "if (mtDenseOwnerReadyTokens[%d].ready.load(std::memory_order_acquire) != target) {\n", slot);
                emitBodyLock(5, "const uint64_t mtEdgeT0 = mtEdgeTimingRdtsc(); unsigned ct = 0;\n");
                emitBodyLock(5, "do {\n");
                if (spinYieldEvery == 0) emitBodyLock(6, "mtWorkerPoolPause();\n");
                else emitBodyLock(6, "mtWorkerPoolPause(); if (++ct > %d) { ct = 0; std::this_thread::yield(); }\n", spinYieldEvery);
                emitBodyLock(5, "} while (mtDenseOwnerReadyTokens[%d].ready.load(std::memory_order_acquire) != target);\n", slot);
                emitBodyLock(5, "mtDenseEdgeBlockedNs[%d] += mtEdgeTimingRdtsc() - mtEdgeT0; mtDenseEdgeWaits[%d] ++;\n", edgeWidx, edgeWidx);
                emitBodyLock(4, "}\n");
              } else {
                emitBodyLock(4, "unsigned ct = 0;\n");
                emitBodyLock(4, "while (mtDenseOwnerReadyTokens[%d].ready.load(std::memory_order_acquire) != target) {\n", slot);
                if (spinYieldEvery == 0) emitBodyLock(5, "mtWorkerPoolPause();\n");
                else emitBodyLock(5, "mtWorkerPoolPause(); if (++ct > %d) { ct = 0; std::this_thread::yield(); }\n", spinYieldEvery);
                emitBodyLock(4, "}\n");
              }
              emitBodyLock(3, "  }\n");
              emitBodyLock(3, "}\n");
            } else {
              emitBodyLock(3, "{ const uint8_t target = evenCycle ? uint8_t{1} : uint8_t{0};\n");
              if (edgeTiming) {
                emitBodyLock(3, "  if (mtDenseOwnerReadyTokens[%d].ready.load(std::memory_order_acquire) != target) {\n", slot);
                emitBodyLock(4, "const uint64_t mtEdgeT0 = mtEdgeTimingRdtsc(); unsigned ct = 0;\n");
                emitBodyLock(4, "do {\n");
                if (spinYieldEvery == 0) emitBodyLock(5, "mtWorkerPoolPause();\n");
                else emitBodyLock(5, "mtWorkerPoolPause(); if (++ct > %d) { ct = 0; std::this_thread::yield(); }\n", spinYieldEvery);
                emitBodyLock(4, "} while (mtDenseOwnerReadyTokens[%d].ready.load(std::memory_order_acquire) != target);\n", slot);
                emitBodyLock(4, "mtDenseEdgeBlockedNs[%d] += mtEdgeTimingRdtsc() - mtEdgeT0; mtDenseEdgeWaits[%d] ++;\n", edgeWidx, edgeWidx);
                emitBodyLock(3, "  }\n");
              } else {
                emitBodyLock(3, "  unsigned ct = 0;\n");
                emitBodyLock(3, "  while (mtDenseOwnerReadyTokens[%d].ready.load(std::memory_order_acquire) != target) {\n", slot);
                if (spinYieldEvery == 0) emitBodyLock(4, "mtWorkerPoolPause(); }\n");
                else emitBodyLock(4, "mtWorkerPoolPause(); if (++ct > %d) { ct = 0; std::this_thread::yield(); } }\n", spinYieldEvery);
              }
              emitBodyLock(3, "}\n");
            }

          }
          emitBodyLock(3, "#else\n");
          if (!skipDenseWait) {
            emitBodyLock(3, "{ const uint32_t target = evenCycle ? kDenseMTaskDepCount[%d] : 0u;\n", mtaskId);
            emitBodyLock(3, "  unsigned ct = 0;\n");
            emitBodyLock(3, "  while (mtDenseMTaskVertices[%d].depsDone.load(std::memory_order_acquire) != target) {\n", mtaskId);
            if (spinYieldEvery == 0) emitBodyLock(4, "mtWorkerPoolPause(); }\n");
            else emitBodyLock(4, "mtWorkerPoolPause(); if (++ct > %d) { ct = 0; std::this_thread::yield(); } }\n", spinYieldEvery);
            emitBodyLock(3, "}\n");
          }
          emitBodyLock(3, "#endif\n");
        }
        if (denseBreakdownWindowCodegen) {
          emitBodyLock(3, "if (unlikely(mtDenseBreakdownWindow && (mtDenseBreakdownWindowAllOwnerBodyMode || mtDenseBreakdownWindowCausalChainMode))) {\n");
          emitBodyLock(4, "MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowAllOwnerMTask = mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowSlot][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[%d]];\n", mtaskId);
          emitBodyLock(4, "mtDenseBreakdownWindowAllOwnerMTask.mtaskId = %d;\n", mtaskId);
          emitBodyLock(4, "mtDenseBreakdownWindowAllOwnerMTask.ownerThreadId = (uint16_t)threadId;\n");
          emitBodyLock(4, "mtDenseBreakdownWindowAllOwnerMTask.readyTokenStoreCount = %d;\n", (int)ownerReadyLayout.storeSlotsByMTask[(size_t)mtaskId].size());
          emitBodyLock(4, "mtDenseBreakdownWindowAllOwnerMTask.releaseEndOffsetNs = UINT64_MAX;\n");
          emitBodyLock(4, "std::chrono::steady_clock::time_point mtDenseBreakdownWindowBodyBegin = std::chrono::steady_clock::now();\n");
          emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowCausalChainMode && mtDenseBreakdownWindowBodyBegin < mtDenseBreakdownWindowEpoch)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal body start clock regression\\n\"); abort(); }\n");
          emitBodyLock(4, "mtDenseBreakdownWindowAllOwnerMTask.bodyStartOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowBodyBegin - mtDenseBreakdownWindowEpoch).count();\n");
          emitBodyLock(4, "stepDenseMTask%d();\n", mtaskId);
          emitBodyLock(4, "std::chrono::steady_clock::time_point mtDenseBreakdownWindowBodyEnd = std::chrono::steady_clock::now();\n");
          emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowCausalChainMode && mtDenseBreakdownWindowBodyEnd < mtDenseBreakdownWindowEpoch)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal body end clock regression\\n\"); abort(); }\n");
          emitBodyLock(4, "mtDenseBreakdownWindowAllOwnerMTask.bodyEndOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowBodyEnd - mtDenseBreakdownWindowEpoch).count();\n");
          emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowAllOwnerMTask.bodyEndOffsetNs < mtDenseBreakdownWindowAllOwnerMTask.bodyStartOffsetNs)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] all-owner body timeline underflow\\n\"); abort(); }\n");
          emitBodyLock(4, "uint64_t mtDenseBreakdownWindowBodyNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowBodyEnd - mtDenseBreakdownWindowBodyBegin).count();\n");
          emitBodyLock(4, "if (unlikely(UINT64_MAX - mtDenseBreakdownWindowWorker->bodyNs < mtDenseBreakdownWindowBodyNs)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window all-owner body counter overflow\\n\"); abort(); }\n");
          emitBodyLock(4, "mtDenseBreakdownWindowAllOwnerMTask.bodyNs = mtDenseBreakdownWindowBodyNs;\n");
          emitBodyLock(4, "mtDenseBreakdownWindowWorker->bodyNs += mtDenseBreakdownWindowBodyNs;\n");
          if (t == 0) {
            emitBodyLock(3, "} else if (unlikely(mtDenseBreakdownWindow && mtDenseBreakdownWindowWorker0BodyMode)) {\n");
            emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowWorker0MTaskNext >= kDenseBreakdownWindowWorker0MTaskCount)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window worker0 MTask record overflow\\n\"); abort(); }\n");
            emitBodyLock(4, "MtDenseBreakdownWindowMTask &mtDenseBreakdownWindowMTask = mtDenseBreakdownWindowWorker0MTasks[mtDenseBreakdownWindowSlot][mtDenseBreakdownWindowWorker0MTaskNext ++];\n");
            emitBodyLock(4, "mtDenseBreakdownWindowMTask.mtaskId = %d;\n", mtaskId);
            emitBodyLock(4, "std::chrono::steady_clock::time_point mtDenseBreakdownWindowBodyBegin = std::chrono::steady_clock::now();\n");
            emitBodyLock(4, "stepDenseMTask%d();\n", mtaskId);
            emitBodyLock(4, "uint64_t mtDenseBreakdownWindowBodyNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDenseBreakdownWindowBodyBegin).count();\n");
            emitBodyLock(4, "if (unlikely(UINT64_MAX - mtDenseBreakdownWindowWorker->bodyNs < mtDenseBreakdownWindowBodyNs)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window body counter overflow\\n\"); abort(); }\n");
            emitBodyLock(4, "mtDenseBreakdownWindowMTask.bodyNs = mtDenseBreakdownWindowBodyNs;\n");
            emitBodyLock(4, "mtDenseBreakdownWindowWorker->bodyNs += mtDenseBreakdownWindowBodyNs;\n");
          }
          emitBodyLock(3, "} else {\n");
          emitBodyLock(4, "stepDenseMTask%d();\n", mtaskId);
          emitBodyLock(3, "}\n");
        } else {
          emitBodyLock(3, "stepDenseMTask%d();\n", mtaskId);
        }
        if (pushShadow) {
          // Push-shadow notify: one completion edge flush per body execution,
          // emitted after every body-call variant of the strict dispatch step
          // and BEFORE the owner-ready token release stores below.
          emitBodyLock(3, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
          emitBodyLock(3, "mtDensePushShadowNotify(%d, (uint32_t)threadId);\n", mtaskId);
          emitBodyLock(3, "#endif\n");
        }
        if (pushReady) {
          // Push-ready notify on the strict dispatch path (macro-off builds
          // compile it out together with all push state): same position as
          // the shadow - after the body, before the token release stores.
          emitBodyLock(3, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
          emitBodyLock(3, "mtDensePushReadyNotify(%d);\n", mtaskId);
          emitBodyLock(3, "#endif\n");
        }
        if (pushReadyHint) {
          // Push-ready hint notify on the strict dispatch path (macro-off
          // builds compile it out together with all hint state): same
          // position - after the body, before the token release stores.
          emitBodyLock(3, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
          emitBodyLock(3, "mtDensePushReadyHintNotify(%d);\n", mtaskId);
          emitBodyLock(3, "#endif\n");
        }
        const bool skipDenseSignal = staticEmptyElide && denseRuntimeSuccs[(size_t)mtaskId].empty();
        if (!ownerReadyFlags) {
          if (!skipDenseSignal) {
            // Verilator signalUpstreamDone: fetch_add (even) or fetch_sub (odd).
            emitBodyLock(3, "if (evenCycle) {\n");
            emitBodyLock(4, "for (int j = kDenseMTaskSuccOffsets[%d]; j < kDenseMTaskSuccOffsets[%d]; j++)\n", mtaskId, mtaskId + 1);
            emitBodyLock(5, "mtDenseMTaskVertices[kDenseMTaskSuccList[j]].depsDone.fetch_add(1, std::memory_order_release);\n");
            emitBodyLock(3, "} else {\n");
            emitBodyLock(4, "for (int j = kDenseMTaskSuccOffsets[%d]; j < kDenseMTaskSuccOffsets[%d]; j++)\n", mtaskId, mtaskId + 1);
            emitBodyLock(5, "mtDenseMTaskVertices[kDenseMTaskSuccList[j]].depsDone.fetch_sub(1, std::memory_order_release);\n");
            emitBodyLock(3, "}\n");
          }
        } else {
          emitBodyLock(3, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
          if (!ownerReadyLayout.storeSlotsByMTask[(size_t)mtaskId].empty()) {
            if (pscdBits)
              emitBodyLock(4, "mtDensePscdRelease(kDenseOwnerReadyStoreOffsets[%d], kDenseOwnerReadyStoreOffsets[%d]);\n", mtaskId, mtaskId + 1);
            emitBodyLock(3, "if (evenCycle) {\n");
            emitBodyLock(4, "for (int j = kDenseOwnerReadyStoreOffsets[%d]; j < kDenseOwnerReadyStoreOffsets[%d]; j++) {\n", mtaskId, mtaskId + 1);
            if (edgeTiming)
              emitBodyLock(5, "mtDenseEdgeFireRdtsc[kDenseOwnerReadyStoreList[j]] = mtEdgeTimingRdtsc();\n");
            if (tokenMaskHint)
              emitBodyLock(5, "mtDenseTokenMask[kDenseTokenMaskConsumerBySlot[kDenseOwnerReadyStoreList[j]]].fetch_or(kDenseTokenMaskBitBySlot[kDenseOwnerReadyStoreList[j]], std::memory_order_release);\n");
            if (denseBreakdownProfileCodegen) {
              emitBodyLock(5, "const int mtDenseBreakdownWindowReadySlot = kDenseOwnerReadyStoreList[j];\n");
              emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindow && mtDenseBreakdownWindowCausalChainMode)) { const int mtDenseBreakdownWindowLogicalToken = kDenseBreakdownWindowCausalLogicalTokenByReadySlot[mtDenseBreakdownWindowReadySlot]; if (unlikely(mtDenseBreakdownWindowLogicalToken < 0 || mtDenseBreakdownWindowLogicalToken >= kDenseBreakdownWindowCausalTokenCount || kDenseBreakdownWindowCausalTokenReadySlot[mtDenseBreakdownWindowLogicalToken] != mtDenseBreakdownWindowReadySlot || kDenseBreakdownWindowCausalTokenProducerMTask[mtDenseBreakdownWindowLogicalToken] != %d || kDenseBreakdownWindowCausalTokenProducerOwner[mtDenseBreakdownWindowLogicalToken] != threadId)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token provenance mismatch\\n\"); abort(); } MtDenseBreakdownWindowReadyToken &mtDenseBreakdownWindowTokenRelease = mtDenseBreakdownWindowReadyTokens[mtDenseBreakdownWindowSlot][mtDenseBreakdownWindowLogicalToken]; const std::chrono::steady_clock::time_point mtDenseBreakdownWindowReleaseBefore = std::chrono::steady_clock::now(); if (unlikely(mtDenseBreakdownWindowReleaseBefore < mtDenseBreakdownWindowEpoch || mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs != UINT64_MAX)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token release-before invalid\\n\"); abort(); } mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowReleaseBefore - mtDenseBreakdownWindowEpoch).count(); mtDenseOwnerReadyTokens[mtDenseBreakdownWindowReadySlot].ready.store(uint8_t{1}, std::memory_order_release); const std::chrono::steady_clock::time_point mtDenseBreakdownWindowReleaseAfter = std::chrono::steady_clock::now(); if (unlikely(mtDenseBreakdownWindowReleaseAfter < mtDenseBreakdownWindowReleaseBefore)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token release-after clock regression\\n\"); abort(); } mtDenseBreakdownWindowTokenRelease.releaseAfterOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowReleaseAfter - mtDenseBreakdownWindowEpoch).count(); } else { mtDenseOwnerReadyTokens[mtDenseBreakdownWindowReadySlot].ready.store(uint8_t{1}, std::memory_order_release); }\n", mtaskId);
            } else {
              emitBodyLock(5, "mtDenseOwnerReadyTokens[kDenseOwnerReadyStoreList[j]].ready.store(uint8_t{1}, std::memory_order_release);\n");
            }
            emitBodyLock(4, "}\n");
            emitBodyLock(3, "} else {\n");
            emitBodyLock(4, "for (int j = kDenseOwnerReadyStoreOffsets[%d]; j < kDenseOwnerReadyStoreOffsets[%d]; j++) {\n", mtaskId, mtaskId + 1);
            if (edgeTiming)
              emitBodyLock(5, "mtDenseEdgeFireRdtsc[kDenseOwnerReadyStoreList[j]] = mtEdgeTimingRdtsc();\n");
            if (tokenMaskHint)
              emitBodyLock(5, "mtDenseTokenMask[kDenseTokenMaskConsumerBySlot[kDenseOwnerReadyStoreList[j]]].fetch_or(kDenseTokenMaskBitBySlot[kDenseOwnerReadyStoreList[j]], std::memory_order_release);\n");
            if (denseBreakdownProfileCodegen) {
              emitBodyLock(5, "const int mtDenseBreakdownWindowReadySlot = kDenseOwnerReadyStoreList[j];\n");
              emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindow && mtDenseBreakdownWindowCausalChainMode)) { const int mtDenseBreakdownWindowLogicalToken = kDenseBreakdownWindowCausalLogicalTokenByReadySlot[mtDenseBreakdownWindowReadySlot]; if (unlikely(mtDenseBreakdownWindowLogicalToken < 0 || mtDenseBreakdownWindowLogicalToken >= kDenseBreakdownWindowCausalTokenCount || kDenseBreakdownWindowCausalTokenReadySlot[mtDenseBreakdownWindowLogicalToken] != mtDenseBreakdownWindowReadySlot || kDenseBreakdownWindowCausalTokenProducerMTask[mtDenseBreakdownWindowLogicalToken] != %d || kDenseBreakdownWindowCausalTokenProducerOwner[mtDenseBreakdownWindowLogicalToken] != threadId)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token provenance mismatch\\n\"); abort(); } MtDenseBreakdownWindowReadyToken &mtDenseBreakdownWindowTokenRelease = mtDenseBreakdownWindowReadyTokens[mtDenseBreakdownWindowSlot][mtDenseBreakdownWindowLogicalToken]; const std::chrono::steady_clock::time_point mtDenseBreakdownWindowReleaseBefore = std::chrono::steady_clock::now(); if (unlikely(mtDenseBreakdownWindowReleaseBefore < mtDenseBreakdownWindowEpoch || mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs != UINT64_MAX)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token release-before invalid\\n\"); abort(); } mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowReleaseBefore - mtDenseBreakdownWindowEpoch).count(); mtDenseOwnerReadyTokens[mtDenseBreakdownWindowReadySlot].ready.store(uint8_t{0}, std::memory_order_release); const std::chrono::steady_clock::time_point mtDenseBreakdownWindowReleaseAfter = std::chrono::steady_clock::now(); if (unlikely(mtDenseBreakdownWindowReleaseAfter < mtDenseBreakdownWindowReleaseBefore)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token release-after clock regression\\n\"); abort(); } mtDenseBreakdownWindowTokenRelease.releaseAfterOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowReleaseAfter - mtDenseBreakdownWindowEpoch).count(); } else { mtDenseOwnerReadyTokens[mtDenseBreakdownWindowReadySlot].ready.store(uint8_t{0}, std::memory_order_release); }\n", mtaskId);
            } else {
              emitBodyLock(5, "mtDenseOwnerReadyTokens[kDenseOwnerReadyStoreList[j]].ready.store(uint8_t{0}, std::memory_order_release);\n");
            }
            emitBodyLock(4, "}\n");
            emitBodyLock(3, "}\n");
          }
          emitBodyLock(3, "#else\n");
          if (!skipDenseSignal) {
            emitBodyLock(3, "if (evenCycle) {\n");
            emitBodyLock(4, "for (int j = kDenseMTaskSuccOffsets[%d]; j < kDenseMTaskSuccOffsets[%d]; j++)\n", mtaskId, mtaskId + 1);
            emitBodyLock(5, "mtDenseMTaskVertices[kDenseMTaskSuccList[j]].depsDone.fetch_add(1, std::memory_order_release);\n");
            emitBodyLock(3, "} else {\n");
            emitBodyLock(4, "for (int j = kDenseMTaskSuccOffsets[%d]; j < kDenseMTaskSuccOffsets[%d]; j++)\n", mtaskId, mtaskId + 1);
            emitBodyLock(5, "mtDenseMTaskVertices[kDenseMTaskSuccList[j]].depsDone.fetch_sub(1, std::memory_order_release);\n");
            emitBodyLock(3, "}\n");
          }
          emitBodyLock(3, "#endif\n");
        }
        if (denseBreakdownWindowCodegen) {
          emitBodyLock(3, "if (unlikely(mtDenseBreakdownWindow && (mtDenseBreakdownWindowAllOwnerBodyMode || mtDenseBreakdownWindowCausalChainMode))) {\n");
          emitBodyLock(4, "const std::chrono::steady_clock::time_point mtDenseBreakdownWindowReleaseEnd = std::chrono::steady_clock::now();\n");
          emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowCausalChainMode && mtDenseBreakdownWindowReleaseEnd < mtDenseBreakdownWindowEpoch)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal MTask release clock regression\\n\"); abort(); }\n");
          emitBodyLock(4, "mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowSlot][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[%d]].releaseEndOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowReleaseEnd - mtDenseBreakdownWindowEpoch).count();\n", mtaskId);
          emitBodyLock(3, "}\n");
        }
      }
      if (denseLookahead) {
        emitBodyLock(3, "#endif\n");
      }
      emitBodyLock(3, "break;\n");
      emitBodyLock(2, "}\n");
    }
    emitBodyLock(2, "default: break;\n");
    emitBodyLock(1, "}\n");
    if (denseBreakdownProfileCodegen) {
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownProfile)) {\n");
      emitBodyLock(2, "MtDenseBreakdownWorker &mtDenseBreakdownWorker = mtDenseBreakdownWorkers[threadId];\n");
      emitBodyLock(2, "uint64_t mtDenseBreakdownDispatchNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDenseBreakdownDispatchBegin).count();\n");
      emitBodyLock(2, "if (unlikely(UINT64_MAX - mtDenseBreakdownWorker.dispatchSpanNs < mtDenseBreakdownDispatchNs)) { fprintf(stderr, \"[mt-dense-breakdown] dispatch counter overflow\\n\"); abort(); }\n");
      emitBodyLock(2, "mtDenseBreakdownWorker.dispatchSpanNs += mtDenseBreakdownDispatchNs;\n");
      emitBodyLock(1, "}\n");
    }
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownWindow)) {\n");
      emitBodyLock(2, "uint64_t mtDenseBreakdownWindowFinishOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDenseBreakdownWindowEpoch).count();\n");
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownWindowFinishOffsetNs < mtDenseBreakdownWindowWorker->startOffsetNs)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window negative dispatch span\\n\"); abort(); }\n");
      emitBodyLock(2, "uint64_t mtDenseBreakdownWindowDispatchNs = mtDenseBreakdownWindowFinishOffsetNs - mtDenseBreakdownWindowWorker->startOffsetNs;\n");
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownWindowWorker->blockedWaitNs > mtDenseBreakdownWindowDispatchNs || mtDenseBreakdownWindowWorker->bodyNs > mtDenseBreakdownWindowDispatchNs - mtDenseBreakdownWindowWorker->blockedWaitNs)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window attribution exceeds dispatch span\\n\"); abort(); }\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker->finishOffsetNs = mtDenseBreakdownWindowFinishOffsetNs;\n");
      emitBodyLock(2, "mtDenseBreakdownWindowWorker->controlNs = mtDenseBreakdownWindowDispatchNs - mtDenseBreakdownWindowWorker->blockedWaitNs - mtDenseBreakdownWindowWorker->bodyNs;\n");
      emitBodyLock(2, "if (threadId == 0) { if (mtDenseBreakdownWindowWorker0BodyMode) { if (unlikely(mtDenseBreakdownWindowWorker0MTaskNext != kDenseBreakdownWindowWorker0MTaskCount)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] window worker0 MTask record count mismatch\\n\"); abort(); } } mtDenseBreakdownWindowWorker0MTaskCounts[mtDenseBreakdownWindowSlot] = (uint32_t)mtDenseBreakdownWindowWorker0MTaskNext; }\n");
      emitBodyLock(1, "}\n");
    }

    emitBodyLock(0, "}\n");
  };
  if (denseLookahead) {
    // Tail-scan instrumentation (E3): zero-cost unless the stats compile macro is set.
    // Counters live in the HEADER as inline variables: the tail function and the
    // dumpMtProfile print land in different SimTop*.cpp shards, so a cpp-local
    // definition would be an undefined reference at link time.
    // emit the #if guard atomically with the function start. A file-split
    // boundary between them orphaned the guard (SimTop1209 ended with #if,
    // SimTop1210 began with the body -> #endif without #if at MAXMT=800).
    if (denseDuty && pushShadow) {
      emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtDutyLane, uint32_t mtPushShadowLane) {\n", name.c_str());
    } else if (denseDuty && pushReadyHint) {
      emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtDutyLane, uint32_t mtPushReadyHintLane) {\n", name.c_str());
    } else if (denseDuty && tokenMaskHint) {
      emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtDutyLane, uint32_t mtDenseTokenMaskLane) {\n", name.c_str());
    } else if (denseDuty) {
      emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtDutyLane) {\n", name.c_str());
    } else if (pushShadow) {
      emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtPushShadowLane) {\n", name.c_str());
    } else if (pushReadyHint) {
      emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtPushReadyHintLane) {\n", name.c_str());
    } else if (tokenMaskHint) {
      emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtDenseTokenMaskLane) {\n", name.c_str());
    } else {
      emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target) {\n", name.c_str());
    }
    if (denseDuty) emitBodyLock(1, "MtDenseDutyGuard mtDutyTailGuard(mtDutyEnabled ? &mtDutyLanes[mtDutyLane].tailNs : nullptr);\n");
    emitBodyLock(1, "const uint32_t mtDenseDispatchCount = static_cast<uint32_t>(mtDenseDispatchEnd - mtDenseDispatchBegin);\n");
    emitBodyLock(1, "uint64_t mtDenseDoneBits[kDenseLookaheadDoneWordCount] = {};\n");
    emitBodyLock(1, "bool anyOutOfOrder = false;\n");
    emitBodyLock(1, "uint32_t head = startHead;\n");
    emitBodyLock(1, "while (head < mtDenseDispatchCount) {\n");
    emitBodyLock(2, "const MtDenseDispatchEntry* mtDenseDispatchEntry = mtDenseDispatchBegin + head;\n");
    emitBodyLock(2, "bool mtDenseEntryReady = true;\n");
    if (pushReadyHint) {
      // GSIM_MT_DENSE_PUSH_READY_HINT: lanes >0 gate the head entry on the
      // single per-task ready byte instead of the token wait list; lane 0
      // (worker0) keeps the original pull conjunction unchanged.
      emitBodyLock(2, "if (mtPushReadyHintLane != 0u) {\n");
      emitBodyLock(3, "mtDenseEntryReady = (mtDensePushReadyHint[mtDenseDispatchEntry->mtaskId].load(std::memory_order_acquire) != 0u);\n");
      emitBodyLock(2, "} else {\n");
      emitBodyLock(3, "for (uint32_t mtDenseDispatchWait = mtDenseDispatchEntry->waitBegin; mtDenseDispatchWait < mtDenseDispatchEntry->waitEnd; ++mtDenseDispatchWait) {\n");
      emitBodyLock(4, "mtDenseEntryReady &= (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) == target);\n");
      emitBodyLock(3, "}\n");
      emitBodyLock(2, "}\n");
    } else if (tokenMaskHint) {
      // GSIM_MT_DENSE_TOKEN_MASK_HINT: lanes >0 gate the head entry on the
      // per-consumer mask word reaching its expected value instead of the
      // token wait list; lane 0 (worker0) keeps the original pull
      // conjunction unchanged.
      emitBodyLock(2, "if (mtDenseTokenMaskLane != 0u) {\n");
      emitBodyLock(3, "mtDenseEntryReady = (mtDenseTokenMask[mtDenseDispatchEntry->mtaskId].load(std::memory_order_acquire) == kDenseTokenMaskExpected[mtDenseDispatchEntry->mtaskId]);\n");
      emitBodyLock(2, "} else {\n");
      emitBodyLock(3, "for (uint32_t mtDenseDispatchWait = mtDenseDispatchEntry->waitBegin; mtDenseDispatchWait < mtDenseDispatchEntry->waitEnd; ++mtDenseDispatchWait) {\n");
      emitBodyLock(4, "mtDenseEntryReady &= (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) == target);\n");
      emitBodyLock(3, "}\n");
      emitBodyLock(2, "}\n");
    } else {
      emitBodyLock(2, "for (uint32_t mtDenseDispatchWait = mtDenseDispatchEntry->waitBegin; mtDenseDispatchWait < mtDenseDispatchEntry->waitEnd; ++mtDenseDispatchWait) {\n");
      emitBodyLock(3, "mtDenseEntryReady &= (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) == target);\n");
      emitBodyLock(2, "}\n");
    }
    // Tail-scan instrumentation (E3): zero-cost unless the stats compile macro is set.
    emitBodyLock(2, "#if defined(GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE) && GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE\n");
    emitBodyLock(2, "if (!mtDenseEntryReady) mtDenseLookaheadTailCalls.fetch_add(1, std::memory_order_relaxed);\n");
    emitBodyLock(2, "#endif\n");
    emitBodyLock(2, "if (mtDenseEntryReady) {\n");
    emitBodyLock(3, "(this->*mtDenseDispatchEntry->fn)();\n");
    if (pushShadow)
      emitBodyLock(3, "mtDensePushShadowNotify(mtDenseDispatchEntry->mtaskId, mtPushShadowLane);\n");
    if (pushReady)
      emitBodyLock(3, "mtDensePushReadyNotify(mtDenseDispatchEntry->mtaskId);\n");
    if (pushReadyHint)
      emitBodyLock(3, "mtDensePushReadyHintNotify(mtDenseDispatchEntry->mtaskId);\n");
    if (pscdBits)
      emitBodyLock(3, "mtDensePscdRelease(mtDenseDispatchEntry->storeBegin, mtDenseDispatchEntry->storeEnd);\n");
    emitBodyLock(3, "for (uint32_t mtDenseDispatchStore = mtDenseDispatchEntry->storeBegin; mtDenseDispatchStore < mtDenseDispatchEntry->storeEnd; ++mtDenseDispatchStore) {\n");
    if (edgeTiming)
      emitBodyLock(4, "mtDenseEdgeFireRdtsc[kDenseOwnerReadyStoreList[mtDenseDispatchStore]] = mtEdgeTimingRdtsc();\n");
    if (tokenMaskHint)
      emitBodyLock(4, "mtDenseTokenMask[kDenseTokenMaskConsumerBySlot[kDenseOwnerReadyStoreList[mtDenseDispatchStore]]].fetch_or(kDenseTokenMaskBitBySlot[kDenseOwnerReadyStoreList[mtDenseDispatchStore]], std::memory_order_release);\n");
    emitBodyLock(4, "mtDenseOwnerReadyTokens[kDenseOwnerReadyStoreList[mtDenseDispatchStore]].ready.store(target, std::memory_order_release);\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "++head;\n");
    emitBodyLock(3, "while (head < mtDenseDispatchCount && anyOutOfOrder && (mtDenseDoneBits[head >> 6] & (uint64_t{1} << (head & 63))) != 0) {\n");
    emitBodyLock(4, "mtDenseDoneBits[head >> 6] &= ~(uint64_t{1} << (head & 63));\n");
    emitBodyLock(4, "++head;\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "continue;\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(2, "bool progressed = false;\n");
    if (adaptiveScan) {
      // Adaptive scan depth: start at a small window, double on full miss,
      // never re-scan a range (ascending order within each range preserved).
      // Legal-order relaxation: a candidate that becomes ready between the
      // narrow scan and the widened continuation may dispatch one outer
      // iteration later than the full-window baseline; both orders are legal
      // under the same dependency/token protocol.
      emitBodyLock(2, "uint32_t mtDenseScanFrom = head + 1u;\n");
      emitBodyLock(2, "uint32_t mtDenseScanWindow = 64u;\n");
      emitBodyLock(2, "while (true) {\n");
      emitBodyLock(3, "uint32_t mtDenseScanEnd = mtDenseScanFrom + mtDenseScanWindow;\n");
      emitBodyLock(3, "if (mtDenseScanEnd > mtDenseDispatchCount) mtDenseScanEnd = mtDenseDispatchCount;\n");
      emitBodyLock(3, "if (mtDenseScanEnd > head + 1u + kDenseLookaheadWindow) mtDenseScanEnd = head + 1u + kDenseLookaheadWindow;\n");
      emitBodyLock(3, "if (mtDenseScanEnd > mtDenseDispatchCount) mtDenseScanEnd = mtDenseDispatchCount;\n");
    } else {
      emitBodyLock(2, "uint32_t mtDenseScanEnd = head + 1u + kDenseLookaheadWindow;\n");
      emitBodyLock(2, "if (mtDenseScanEnd > mtDenseDispatchCount) mtDenseScanEnd = mtDenseDispatchCount;\n");
    }
    emitBodyLock(2, "for (uint32_t j = %s; j < mtDenseScanEnd; ++j) {\n", adaptiveScan ? "mtDenseScanFrom" : "head + 1u");
    emitBodyLock(3, "#if defined(GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE) && GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE\n");
    emitBodyLock(3, "mtDenseLookaheadScanned.fetch_add(1, std::memory_order_relaxed);\n");
    emitBodyLock(3, "#endif\n");
    emitBodyLock(3, "if (anyOutOfOrder && (mtDenseDoneBits[j >> 6] & (uint64_t{1} << (j & 63))) != 0) continue;\n");
    emitBodyLock(3, "const MtDenseDispatchEntry* mtDenseCandidate = mtDenseDispatchBegin + j;\n");
    emitBodyLock(3, "bool mtDenseCandidateReady = true;\n");
    emitBodyLock(3, "for (uint32_t mtDenseLocalPrereq = mtDenseCandidate->localBegin; mtDenseLocalPrereq < mtDenseCandidate->localEnd; ++mtDenseLocalPrereq) {\n");
    emitBodyLock(4, "const uint32_t mtDensePredPosition = kDenseLookaheadLocalPrereqs[mtDenseLocalPrereq];\n");
    emitBodyLock(4, "if (!(mtDensePredPosition < head || (anyOutOfOrder && (mtDenseDoneBits[mtDensePredPosition >> 6] & (uint64_t{1} << (mtDensePredPosition & 63))) != 0))) { mtDenseCandidateReady = false; break; }\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "if (!mtDenseCandidateReady) continue;\n");
    if (pushReadyHint) {
      // GSIM_MT_DENSE_PUSH_READY_HINT: candidate admission for lanes >0 is
      // the same single acquire (local-prereq positions above are unchanged
      // same-worker control flow); lane 0 keeps the original token loop.
      emitBodyLock(3, "if (mtPushReadyHintLane != 0u) {\n");
      emitBodyLock(4, "if (mtDensePushReadyHint[mtDenseCandidate->mtaskId].load(std::memory_order_acquire) == 0u) { mtDenseCandidateReady = false; }\n");
      emitBodyLock(3, "} else {\n");
      emitBodyLock(4, "for (uint32_t mtDenseDispatchWait = mtDenseCandidate->waitBegin; mtDenseDispatchWait < mtDenseCandidate->waitEnd; ++mtDenseDispatchWait) {\n");
      emitBodyLock(5, "if (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) != target) { mtDenseCandidateReady = false; break; }\n");
      emitBodyLock(4, "}\n");
      emitBodyLock(3, "}\n");
    } else if (tokenMaskHint) {
      // GSIM_MT_DENSE_TOKEN_MASK_HINT: candidate admission for lanes >0 is
      // the same single mask comparison (local-prereq positions above are
      // unchanged same-worker control flow); lane 0 keeps the original
      // token loop.
      emitBodyLock(3, "if (mtDenseTokenMaskLane != 0u) {\n");
      emitBodyLock(4, "if (mtDenseTokenMask[mtDenseCandidate->mtaskId].load(std::memory_order_acquire) != kDenseTokenMaskExpected[mtDenseCandidate->mtaskId]) { mtDenseCandidateReady = false; }\n");
      emitBodyLock(3, "} else {\n");
      emitBodyLock(4, "for (uint32_t mtDenseDispatchWait = mtDenseCandidate->waitBegin; mtDenseDispatchWait < mtDenseCandidate->waitEnd; ++mtDenseDispatchWait) {\n");
      emitBodyLock(5, "if (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) != target) { mtDenseCandidateReady = false; break; }\n");
      emitBodyLock(4, "}\n");
      emitBodyLock(3, "}\n");
    } else {
      emitBodyLock(3, "for (uint32_t mtDenseDispatchWait = mtDenseCandidate->waitBegin; mtDenseDispatchWait < mtDenseCandidate->waitEnd; ++mtDenseDispatchWait) {\n");
      emitBodyLock(4, "if (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) != target) { mtDenseCandidateReady = false; break; }\n");
      emitBodyLock(3, "}\n");
    }
    emitBodyLock(3, "if (!mtDenseCandidateReady) continue;\n");
    emitBodyLock(3, "#if defined(GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE) && GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE\n");
    emitBodyLock(3, "mtDenseLookaheadFound.fetch_add(1, std::memory_order_relaxed);\n");
    emitBodyLock(3, "#endif\n");
    emitBodyLock(3, "(this->*mtDenseCandidate->fn)();\n");
    if (pushShadow)
      emitBodyLock(3, "mtDensePushShadowNotify(mtDenseCandidate->mtaskId, mtPushShadowLane);\n");
    if (pushReady)
      emitBodyLock(3, "mtDensePushReadyNotify(mtDenseCandidate->mtaskId);\n");
    if (pushReadyHint)
      emitBodyLock(3, "mtDensePushReadyHintNotify(mtDenseCandidate->mtaskId);\n");
    if (pscdBits)
      emitBodyLock(3, "mtDensePscdRelease(mtDenseCandidate->storeBegin, mtDenseCandidate->storeEnd);\n");
    emitBodyLock(3, "for (uint32_t mtDenseDispatchStore = mtDenseCandidate->storeBegin; mtDenseDispatchStore < mtDenseCandidate->storeEnd; ++mtDenseDispatchStore) {\n");
    if (edgeTiming)
      emitBodyLock(4, "mtDenseEdgeFireRdtsc[kDenseOwnerReadyStoreList[mtDenseDispatchStore]] = mtEdgeTimingRdtsc();\n");
    if (tokenMaskHint)
      emitBodyLock(4, "mtDenseTokenMask[kDenseTokenMaskConsumerBySlot[kDenseOwnerReadyStoreList[mtDenseDispatchStore]]].fetch_or(kDenseTokenMaskBitBySlot[kDenseOwnerReadyStoreList[mtDenseDispatchStore]], std::memory_order_release);\n");
    emitBodyLock(4, "mtDenseOwnerReadyTokens[kDenseOwnerReadyStoreList[mtDenseDispatchStore]].ready.store(target, std::memory_order_release);\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "mtDenseDoneBits[j >> 6] |= uint64_t{1} << (j & 63);\n");
    emitBodyLock(3, "anyOutOfOrder = true;\n");
    emitBodyLock(3, "progressed = true;\n");
    emitBodyLock(3, "break;\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(2, "#if defined(GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE) && GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE\n");
    emitBodyLock(2, "if (!progressed) mtDenseLookaheadFullMiss.fetch_add(1, std::memory_order_relaxed);\n");
    emitBodyLock(2, "#endif\n");
    if (adaptiveScan) {
      emitBodyLock(3, "if (progressed) break;\n");
      emitBodyLock(3, "if (mtDenseScanEnd >= mtDenseDispatchCount || mtDenseScanWindow >= kDenseLookaheadWindow) break;\n");
      emitBodyLock(3, "mtDenseScanFrom = mtDenseScanEnd;\n");
      emitBodyLock(3, "mtDenseScanWindow = mtDenseScanWindow >= kDenseLookaheadWindow / 2u ? kDenseLookaheadWindow : mtDenseScanWindow * 2u;\n");
      emitBodyLock(2, "}\n");
      emitBodyLock(2, "if (progressed) continue;\n");
    } else {
      emitBodyLock(2, "if (progressed) continue;\n");
    }
    if (denseDuty) emitBodyLock(2, "std::chrono::steady_clock::time_point mtDutyBlockBegin; if (mtDutyEnabled) mtDutyBlockBegin = std::chrono::steady_clock::now();\n");
    if (!pushReadyHint && !tokenMaskHint)
      emitBodyLock(2, "for (uint32_t mtDenseDispatchWait = mtDenseDispatchEntry->waitBegin; mtDenseDispatchWait < mtDenseDispatchEntry->waitEnd; ++mtDenseDispatchWait) {\n");
    if (edgeTiming) {
      emitBodyLock(3, "if (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) != target) {\n");
      emitBodyLock(4, "const uint64_t mtEdgeT0 = mtEdgeTimingRdtsc(); unsigned ct = 0;\n");
      emitBodyLock(4, "do {\n");
      if (spinYieldEvery == 0) emitBodyLock(5, "mtWorkerPoolPause();\n");
      else emitBodyLock(5, "mtWorkerPoolPause(); if (++ct > %d) { ct = 0; std::this_thread::yield(); }\n", spinYieldEvery);
      emitBodyLock(4, "} while (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) != target);\n");
      emitBodyLock(4, "mtDenseEdgeBlockedNs[mtDenseDispatchWait] += mtEdgeTimingRdtsc() - mtEdgeT0; mtDenseEdgeWaits[mtDenseDispatchWait] ++;\n");
      emitBodyLock(3, "}\n");
    } else if (pushReadyHint) {
      // GSIM_MT_DENSE_PUSH_READY_HINT: the tail fallback spin for lanes >0
      // waits on the single ready byte (armed exactly once per cycle by the
      // final mapped producer edge); lane 0 keeps the original per-slot token
      // spin. edgeTiming is asserted off with the hint knob.
      emitBodyLock(2, "if (mtPushReadyHintLane != 0u) {\n");
      emitBodyLock(3, "unsigned ct = 0;\n");
      emitBodyLock(3, "while (mtDensePushReadyHint[mtDenseDispatchEntry->mtaskId].load(std::memory_order_acquire) == 0u) {\n");
      if (spinYieldEvery == 0) emitBodyLock(4, "mtWorkerPoolPause(); }\n");
      else emitBodyLock(4, "mtWorkerPoolPause(); if (++ct > %d) { ct = 0; std::this_thread::yield(); } }\n", spinYieldEvery);
      emitBodyLock(2, "} else {\n");
      emitBodyLock(3, "for (uint32_t mtDenseDispatchWait = mtDenseDispatchEntry->waitBegin; mtDenseDispatchWait < mtDenseDispatchEntry->waitEnd; ++mtDenseDispatchWait) {\n");
      emitBodyLock(4, "unsigned ct = 0;\n");
      emitBodyLock(4, "while (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) != target) {\n");
      if (spinYieldEvery == 0) emitBodyLock(5, "mtWorkerPoolPause(); }\n");
      else emitBodyLock(5, "mtWorkerPoolPause(); if (++ct > %d) { ct = 0; std::this_thread::yield(); } }\n", spinYieldEvery);
      emitBodyLock(3, "}\n");
      emitBodyLock(2, "}\n");
    } else if (tokenMaskHint) {
      // GSIM_MT_DENSE_TOKEN_MASK_HINT: the tail fallback spin for lanes >0
      // waits on the consumer mask reaching its expected value (each remote
      // publisher fetch_or-s its own bit at release); lane 0 keeps the
      // original per-slot token spin. edgeTiming/breakdown are asserted off
      // with this knob.
      emitBodyLock(2, "if (mtDenseTokenMaskLane != 0u) {\n");
      emitBodyLock(3, "unsigned ct = 0;\n");
      emitBodyLock(3, "while (mtDenseTokenMask[mtDenseDispatchEntry->mtaskId].load(std::memory_order_acquire) != kDenseTokenMaskExpected[mtDenseDispatchEntry->mtaskId]) {\n");
      if (spinYieldEvery == 0) emitBodyLock(4, "mtWorkerPoolPause(); }\n");
      else emitBodyLock(4, "mtWorkerPoolPause(); if (++ct > %d) { ct = 0; std::this_thread::yield(); } }\n", spinYieldEvery);
      emitBodyLock(2, "} else {\n");
      emitBodyLock(3, "for (uint32_t mtDenseDispatchWait = mtDenseDispatchEntry->waitBegin; mtDenseDispatchWait < mtDenseDispatchEntry->waitEnd; ++mtDenseDispatchWait) {\n");
      emitBodyLock(4, "unsigned ct = 0;\n");
      emitBodyLock(4, "while (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) != target) {\n");
      if (spinYieldEvery == 0) emitBodyLock(5, "mtWorkerPoolPause(); }\n");
      else emitBodyLock(5, "mtWorkerPoolPause(); if (++ct > %d) { ct = 0; std::this_thread::yield(); } }\n", spinYieldEvery);
      emitBodyLock(3, "}\n");
      emitBodyLock(2, "}\n");
    } else {
      emitBodyLock(3, "unsigned ct = 0;\n");
      emitBodyLock(3, "while (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) != target) {\n");
      if (spinYieldEvery == 0) emitBodyLock(4, "mtWorkerPoolPause(); }\n");
      else emitBodyLock(4, "mtWorkerPoolPause(); if (++ct > %d) { ct = 0; std::this_thread::yield(); } }\n", spinYieldEvery);
    }
    if (!pushReadyHint && !tokenMaskHint)
      emitBodyLock(2, "}\n");
    if (denseDuty) emitBodyLock(2, "if (mtDutyEnabled) mtDutyLanes[mtDutyLane].blockNs += (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDutyBlockBegin).count();\n");
    emitBodyLock(2, "(this->*mtDenseDispatchEntry->fn)();\n");
    if (pushShadow)
      emitBodyLock(2, "mtDensePushShadowNotify(mtDenseDispatchEntry->mtaskId, mtPushShadowLane);\n");
    if (pushReady)
      emitBodyLock(2, "mtDensePushReadyNotify(mtDenseDispatchEntry->mtaskId);\n");
    if (pushReadyHint)
      emitBodyLock(2, "mtDensePushReadyHintNotify(mtDenseDispatchEntry->mtaskId);\n");
    if (pscdBits)
      emitBodyLock(2, "mtDensePscdRelease(mtDenseDispatchEntry->storeBegin, mtDenseDispatchEntry->storeEnd);\n");
    emitBodyLock(2, "for (uint32_t mtDenseDispatchStore = mtDenseDispatchEntry->storeBegin; mtDenseDispatchStore < mtDenseDispatchEntry->storeEnd; ++mtDenseDispatchStore) {\n");
    if (edgeTiming)
      emitBodyLock(3, "mtDenseEdgeFireRdtsc[kDenseOwnerReadyStoreList[mtDenseDispatchStore]] = mtEdgeTimingRdtsc();\n");
    if (tokenMaskHint)
      emitBodyLock(3, "mtDenseTokenMask[kDenseTokenMaskConsumerBySlot[kDenseOwnerReadyStoreList[mtDenseDispatchStore]]].fetch_or(kDenseTokenMaskBitBySlot[kDenseOwnerReadyStoreList[mtDenseDispatchStore]], std::memory_order_release);\n");
    emitBodyLock(3, "mtDenseOwnerReadyTokens[kDenseOwnerReadyStoreList[mtDenseDispatchStore]].ready.store(target, std::memory_order_release);\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(2, "++head;\n");
    emitBodyLock(2, "while (head < mtDenseDispatchCount && anyOutOfOrder && (mtDenseDoneBits[head >> 6] & (uint64_t{1} << (head & 63))) != 0) {\n");
    emitBodyLock(3, "mtDenseDoneBits[head >> 6] &= ~(uint64_t{1} << (head & 63));\n");
    emitBodyLock(3, "++head;\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(0, "}\n");
    emitBodyLock(0, "#endif\n");
  }
  if (denseLookahead) {
    // The entry type, table declarations and the lookahead tail are all gated on the
    // owner-ready compile macro; the definitions must match or a macro-less compile
    // sees definitions of undeclared members (build failure).
    emitBodyLock(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    for (int t = 0; t < threadCount; t++) {
      int cnt = denseDispatchWorkerCounts[(size_t)t];
      emitBodyLock(0, "const S%s::MtDenseDispatchEntry S%s::kDenseDispatchTableW%d[%d] = {\n",
                   name.c_str(), name.c_str(), t, std::max(1, cnt));
      if (cnt == 0) {
        emitBodyLock(0, (pushShadow || pushReady || pushReadyHint || tokenMaskHint) ? "{nullptr, 0u, 0u, 0u, 0u, 0u, 0u, 0u}\n"
                                                  : "{nullptr, 0u, 0u, 0u, 0u, 0u, 0u}\n");
      } else {
        for (int m = 0; m < nMTasks; m++) {
          if (denseSchedule.mtaskThreadAssign[(size_t)m] != t) continue;
          if (pushShadow || pushReady || pushReadyHint || tokenMaskHint) {
            emitBodyLock(0, "{&S%s::stepDenseMTask%d, %uu, %uu, %uu, %uu, %uu, %uu, %uu},\n",
                         name.c_str(), m,
                         denseDispatchWaitBegin[(size_t)m], denseDispatchWaitEnd[(size_t)m],
                         denseDispatchStoreBegin[(size_t)m], denseDispatchStoreEnd[(size_t)m],
                         denseLookaheadLocalBegin[(size_t)m], denseLookaheadLocalEnd[(size_t)m],
                         static_cast<unsigned>(m));
          } else {
            emitBodyLock(0, "{&S%s::stepDenseMTask%d, %uu, %uu, %uu, %uu, %uu, %uu},\n",
                         name.c_str(), m,
                         denseDispatchWaitBegin[(size_t)m], denseDispatchWaitEnd[(size_t)m],
                         denseDispatchStoreBegin[(size_t)m], denseDispatchStoreEnd[(size_t)m],
                         denseLookaheadLocalBegin[(size_t)m], denseLookaheadLocalEnd[(size_t)m]);
          }
        }
      }
      emitBodyLock(0, "};\n");
    }
    emitBodyLock(0, "#endif\n");
  }
  if (pscdBits) {
    // PSCD store-phase helper: folds per-field change verdicts into one
    // epoch-tagged change byte per token slot. Called at every owner-ready
    // release site (inline fast path, strict dispatch store loops, all three
    // lookahead tail paths, and the serial fallback) BEFORE the ready-token
    // release stores, so the release/acquire token pairing publishes the
    // change bytes to consumers. Conservative entries (worker0/ext/reset/
    // array/ambiguous producers) mark their whole token region changed.
    emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::mtDensePscdRelease(uint32_t pscdStoreBegin, uint32_t pscdStoreEnd) {\n", name.c_str());
    emitBodyLock(1, "for (uint32_t j = pscdStoreBegin; j < pscdStoreEnd; ++j) {\n");
    emitBodyLock(2, "const int pscdSlot = kDenseOwnerReadyStoreList[j];\n");
    emitBodyLock(2, "if (kDensePscdStoreConservative[j]) {\n");
    emitBodyLock(3, "mtDenseChangeEpoch[pscdSlot] = mtDenseCycleEpoch;\n");
    emitBodyLock(2, "} else {\n");
    emitBodyLock(3, "uint8_t pscdVerdict = 0;\n");
    emitBodyLock(3, "for (int k = kDensePscdStoreFieldOffsets[j]; k < kDensePscdStoreFieldOffsets[j + 1]; ++k)\n");
    emitBodyLock(4, "if (mtDensePscdFieldChanged[kDensePscdStoreFieldList[k]]) { pscdVerdict = 1; break; }\n");
    emitBodyLock(3, "mtDenseChangeEpoch[pscdSlot] = pscdVerdict ? mtDenseCycleEpoch : (uint8_t)(3u - mtDenseCycleEpoch);\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(0, "}\n");
    emitBodyLock(0, "#endif\n");
    // Reset-window helper: called by subResetDenseN bodies under their reset
    // guard (generation-gated in genResetDef) whenever a reset application
    // actually fires — cycle-start uint resets from resetAllDense() and
    // mid-cycle async resets from SUPER_ASYNC_RESET MTask bodies. Marks the
    // cycle's flag and every change byte changed (conservative), and the
    // consumer verify skips cross-checks in any cycle whose window may span
    // a reset application. Reset windows are rare, so the O(slots) pass is
    // negligible.
    emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::mtDensePscdResetHit() {\n", name.c_str());
    emitBodyLock(1, "mtDensePscdResetActive = 1;\n");
    emitBodyLock(1, "const uint8_t pscdEpoch = mtDenseCycleEpoch;\n");
    emitBodyLock(1, "for (int s = 0; s < kDenseOwnerReadyPhysicalSlotCount; ++s)\n");
    emitBodyLock(2, "mtDenseChangeEpoch[s] = pscdEpoch;\n");
    emitBodyLock(0, "}\n");
    emitBodyLock(0, "#endif\n");
  }
  if (pushShadow) {
    // Push-ready shadow notification helper: called at every task completion
    // BEFORE the owner-ready token release stores (inline fast path, all three
    // lookahead tail paths, strict dispatch step, and the serial fallback all
    // route through this), decrementing each dependent's pending counter
    // exactly once per deduplicated producer edge. Ordering before the token
    // stores is what makes the entry check sound: a consumer authorized by a
    // token acquire load also observes the producer's earlier decrement.
    // Sole-writer lane counters keep the operation accounting free of
    // contended atomics; the pending RMWs are the shadowed protocol itself.
    // An underflow (prev == 0 or prev > fanin) means the generated census
    // decrements a dependent more times per cycle than its fan-in - loud
    // abort, matching the harness-oracle mutation contract.
    emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::mtDensePushShadowNotify(uint32_t mtaskId, uint32_t lane) {\n", name.c_str());
    emitBodyLock(1, "mtDensePushShadowNotifyByLane[lane] += 1;\n");
    emitBodyLock(1, "for (uint32_t j = kDensePushShadowNotifyOffsets[mtaskId]; j < kDensePushShadowNotifyOffsets[mtaskId + 1]; ++j) {\n");
    emitBodyLock(2, "const uint32_t mtPushShadowDependent = kDensePushShadowNotifyList[j];\n");
    emitBodyLock(2, "const uint32_t mtPushShadowPrev = mtDensePushShadowPending[mtPushShadowDependent].fetch_sub(1u, std::memory_order_acq_rel);\n");
    emitBodyLock(2, "mtDensePushShadowDecrementByLane[lane] += 1;\n");
    emitBodyLock(2, "if (unlikely(mtPushShadowPrev == 0u || mtPushShadowPrev > kDensePushShadowFanin[mtPushShadowDependent])) {\n");
    emitBodyLock(3, "fprintf(stderr, \"[push-shadow] fan-in underflow dependent=%%u producer=%%u cycle=%%lu prev=%%u fanin=%%u\\n\", mtPushShadowDependent, mtaskId, (unsigned long) cycles, mtPushShadowPrev, kDensePushShadowFanin[mtPushShadowDependent]);\n");
    emitBodyLock(3, "abort();\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(0, "}\n");
    emitBodyLock(0, "#endif\n");
  }
  if (edgeTiming) {
    // Teardown dump for GSIM_MT_DENSE_EDGE_TIMING: one JSON record per wait
    // entry (sole-writer accumulators, safe to read after the worker pool
    // has joined). blockedNs values are rdtsc ticks (see mtEdgeTimingRdtsc).
    emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::dumpDenseEdgeTiming() {\n", name.c_str());
    emitBodyLock(1, "FILE *mtEdgeTimingFile = fopen(\"SimTop_edge_timing.json\", \"w\");\n");
    emitBodyLock(1, "if (mtEdgeTimingFile == nullptr) { fprintf(stderr, \"[edge-timing] failed to open SimTop_edge_timing.json\\n\"); return; }\n");
    emitBodyLock(1, "fprintf(mtEdgeTimingFile, \"{\\n\\\"format\\\": \\\"gsim.dense-edge-timing.v1\\\",\\n\\\"wait_entries\\\": [\\n\");\n");
    emitBodyLock(1, "for (int w = 0; w < kDenseEdgeTimingWaitEntryCount; ++w) {\n");
    emitBodyLock(2, "if (w > 0) fprintf(mtEdgeTimingFile, \",\\n\");\n");
    emitBodyLock(2, "fprintf(mtEdgeTimingFile, \"  {\\\"waitIndex\\\": %%d, \\\"consumerMtaskId\\\": %%d, \\\"tokenSlot\\\": %%d, \\\"waits\\\": %%lu, \\\"blockedNs\\\": %%lu}\", w, kDenseEdgeTimingWaitConsumer[w], kDenseEdgeTimingWaitSlot[w], (unsigned long) mtDenseEdgeWaits[w], (unsigned long) mtDenseEdgeBlockedNs[w]);\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(1, "fprintf(mtEdgeTimingFile, \"\\n]}\\n\");\n");
    emitBodyLock(1, "fclose(mtEdgeTimingFile);\n");
    emitBodyLock(0, "}\n");
    emitBodyLock(0, "#endif\n");
  }
  emitFixedDenseThreadWorker("stepDenseThreadWorker");
  if (pushReady) {
    // ---- GSIM_MT_DENSE_PUSH_READY runtime (default off; whole block inside
    // the owner-ready compile guard so a macro-off build keeps pure pull for
    // every worker and no push state is referenced) ----
    // Hybrid protocol: worker0 keeps the pull lane; pool workers 1..N-1
    // execute exactly their assigned MTask count. Notification is the
    // deduplicated producer->dependent CSR: each producer completion
    // decrements every dependent's per-cycle pending counter once and the
    // decrementer that observes old==1 hands the dependent to its owner's
    // arrival mechanism - never worker0, whose roots and dependents stay
    // pull-discovered. v1 arrival is the per-owner bounded sequence queues
    // (enqueue/dequeue below, consumed by stepDensePushThreadWorker);
    // GSIM_MT_DENSE_PUSH_READY_BITMAP replaces them with one ready byte per
    // MTask - notify arms the byte directly and the bitmap worker at the end
    // of this block acquire-scans its own dispatch-table slice - so the
    // queue helpers are not emitted at all in bitmap mode.
    if (!pushReadyBitmap) {
      emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::mtDensePushReadyEnqueue(uint32_t mtPushReadyWorker, uint32_t mtaskId) {\n", name.c_str());
      emitBodyLock(1, "const uint32_t mtPushReadyCapacity = kDensePushReadyQueueCapacity[mtPushReadyWorker];\n");
      emitBodyLock(1, "if (unlikely(mtPushReadyCapacity == 0u)) {\n");
      emitBodyLock(2, "fprintf(stderr, \"[push-ready] enqueue on capacity-zero queue worker=%%u mtask=%%u cycle=%%lu\\n\", mtPushReadyWorker, mtaskId, (unsigned long) cycles);\n");
      emitBodyLock(2, "abort();\n");
      emitBodyLock(1, "}\n");
      emitBodyLock(1, "// Vyukov sequence ring: the slot's sequence number - not the tail\n");
      emitBodyLock(1, "// index - gates consumer visibility, so a consumer that acquires a\n");
      emitBodyLock(1, "// slot's data always sees the producer's release-store and everything\n");
      emitBodyLock(1, "// it ordered before it. Producers race only the tail claim (CAS); the\n");
      emitBodyLock(1, "// loser reloads the tail and retries at the fresh position.\n");
      emitBodyLock(1, "MtDensePushReadySlot *mtPushReadySlots = mtDensePushReadySlots + kDensePushReadyQueueOffset[mtPushReadyWorker];\n");
      emitBodyLock(1, "const uint64_t mtPushReadyMask = (uint64_t)mtPushReadyCapacity - 1u;\n");
      emitBodyLock(1, "uint64_t mtPushReadyPosition = mtDensePushReadyTail[mtPushReadyWorker].load(std::memory_order_relaxed);\n");
      emitBodyLock(1, "while (true) {\n");
      emitBodyLock(2, "MtDensePushReadySlot &mtPushReadySlot = mtPushReadySlots[mtPushReadyPosition & mtPushReadyMask];\n");
      emitBodyLock(2, "const uint64_t mtPushReadySequence = mtPushReadySlot.seq.load(std::memory_order_acquire);\n");
      emitBodyLock(2, "const int64_t mtPushReadyDifference = (int64_t)mtPushReadySequence - (int64_t)mtPushReadyPosition;\n");
      emitBodyLock(2, "if (mtPushReadyDifference == 0) {\n");
      emitBodyLock(3, "if (mtDensePushReadyTail[mtPushReadyWorker].compare_exchange_weak(mtPushReadyPosition, mtPushReadyPosition + 1u, std::memory_order_relaxed, std::memory_order_relaxed)) {\n");
      emitBodyLock(4, "mtPushReadySlot.mtaskId = mtaskId;\n");
      emitBodyLock(4, "mtPushReadySlot.seq.store((uint32_t)(mtPushReadyPosition + 1u), std::memory_order_release);\n");
      emitBodyLock(4, "return;\n");
      emitBodyLock(3, "}\n");
      emitBodyLock(2, "} else if (mtPushReadyDifference < 0) {\n");
      emitBodyLock(3, "// Ring full: impossible while capacity >= the owner's assigned count\n");
      emitBodyLock(3, "// (each task is enqueued at most once per cycle and every queue is\n");
      emitBodyLock(3, "// reset before dispatch), so this is a loud capacity failure.\n");
      emitBodyLock(3, "fprintf(stderr, \"[push-ready] queue overflow worker=%%u mtask=%%u cycle=%%lu capacity=%%u\\n\", mtPushReadyWorker, mtaskId, (unsigned long) cycles, mtPushReadyCapacity);\n");
      emitBodyLock(3, "abort();\n");
      emitBodyLock(2, "} else {\n");
      emitBodyLock(3, "mtPushReadyPosition = mtDensePushReadyTail[mtPushReadyWorker].load(std::memory_order_relaxed);\n");
      emitBodyLock(2, "}\n");
      emitBodyLock(1, "}\n");
      emitBodyLock(0, "}\n");

      emitFuncDecl(0, "bool S%s::mtDensePushReadyDequeue(uint32_t mtPushReadyWorker, uint32_t &mtaskId) {\n", name.c_str());
      emitBodyLock(1, "const uint32_t mtPushReadyCapacity = kDensePushReadyQueueCapacity[mtPushReadyWorker];\n");
      emitBodyLock(1, "if (unlikely(mtPushReadyCapacity == 0u)) return false;\n");
      emitBodyLock(1, "MtDensePushReadySlot *mtPushReadySlots = mtDensePushReadySlots + kDensePushReadyQueueOffset[mtPushReadyWorker];\n");
      emitBodyLock(1, "const uint64_t mtPushReadyMask = (uint64_t)mtPushReadyCapacity - 1u;\n");
      emitBodyLock(1, "// Sole consumer: head is private to this worker, so a relaxed\n");
      emitBodyLock(1, "// load/store pair is sufficient; the slot sequence carries ordering.\n");
      emitBodyLock(1, "const uint64_t mtPushReadyPosition = mtDensePushReadyHead[mtPushReadyWorker].load(std::memory_order_relaxed);\n");
      emitBodyLock(1, "MtDensePushReadySlot &mtPushReadySlot = mtPushReadySlots[mtPushReadyPosition & mtPushReadyMask];\n");
      emitBodyLock(1, "const uint64_t mtPushReadySequence = mtPushReadySlot.seq.load(std::memory_order_acquire);\n");
      emitBodyLock(1, "const int64_t mtPushReadyDifference = (int64_t)mtPushReadySequence - (int64_t)(mtPushReadyPosition + 1u);\n");
      emitBodyLock(1, "if (mtPushReadyDifference == 0) {\n");
      emitBodyLock(2, "mtDensePushReadyHead[mtPushReadyWorker].store(mtPushReadyPosition + 1u, std::memory_order_relaxed);\n");
      emitBodyLock(2, "mtaskId = mtPushReadySlot.mtaskId;\n");
      emitBodyLock(2, "mtPushReadySlot.seq.store((uint32_t)(mtPushReadyPosition + mtPushReadyCapacity), std::memory_order_release);\n");
      emitBodyLock(2, "return true;\n");
      emitBodyLock(1, "}\n");
      emitBodyLock(1, "if (unlikely(mtPushReadyDifference > 0)) {\n");
      emitBodyLock(2, "// Unreachable with a sole consumer (head never advances past a\n");
      emitBodyLock(2, "// slot's published sequence): abort loudly, never report empty.\n");
      emitBodyLock(2, "fprintf(stderr, \"[push-ready] dequeue sequence corruption worker=%%u cycle=%%lu\\n\", mtPushReadyWorker, (unsigned long) cycles);\n");
      emitBodyLock(2, "abort();\n");
      emitBodyLock(1, "}\n");
      emitBodyLock(1, "return false;\n");
      emitBodyLock(0, "}\n");
    }
    emitFuncDecl(0, pushReadyBitmap
                     ? "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::mtDensePushReadyNotify(uint32_t mtaskId) {\n"
                     : "void S%s::mtDensePushReadyNotify(uint32_t mtaskId) {\n",
                 name.c_str());
    emitBodyLock(1, "// Exactly one decrement per deduplicated CSR edge. fetch_sub(acq_rel)\n");
    emitBodyLock(1, "// makes the decrementer that observes old==1 synchronize with every\n");
    emitBodyLock(1, "// earlier producer on the same counter, so its release-ordered %s\n",
                 pushReadyBitmap ? "arrival store" : "enqueue");
    emitBodyLock(1, "// publishes all producers' body writes to the consumer's acquire.\n");
    emitBodyLock(1, "for (uint32_t j = kDensePushReadyNotifyOffsets[mtaskId]; j < kDensePushReadyNotifyOffsets[mtaskId + 1u]; ++j) {\n");
    emitBodyLock(2, "const uint32_t mtPushReadyDependent = kDensePushReadyNotifyList[j];\n");
    emitBodyLock(2, "const uint32_t mtPushReadyPrevious = mtDensePushReadyPending[mtPushReadyDependent].fetch_sub(1u, std::memory_order_acq_rel);\n");
    emitBodyLock(2, "if (unlikely(mtPushReadyPrevious == 0u || mtPushReadyPrevious > kDensePushReadyFanin[mtPushReadyDependent])) {\n");
    emitBodyLock(3, "fprintf(stderr, \"[push-ready] fan-in underflow dependent=%%u producer=%%u cycle=%%lu previous=%%u fanin=%%u\\n\", mtPushReadyDependent, mtaskId, (unsigned long) cycles, mtPushReadyPrevious, kDensePushReadyFanin[mtPushReadyDependent]);\n");
    emitBodyLock(3, "abort();\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(2, "if (mtPushReadyPrevious == 1u) {\n");
    emitBodyLock(3, "const uint32_t mtPushReadyOwner = kDensePushReadyOwner[mtPushReadyDependent];\n");
    emitBodyLock(3, "// Worker0-owned dependents stay pull-discovered: decrement only.\n");
    if (pushReadyBitmap) {
      emitBodyLock(3, "// GSIM_MT_DENSE_PUSH_READY_BITMAP: queue-free arrival - the release\n");
      emitBodyLock(3, "// store of the owner's ready byte is the whole handoff; the owner\n");
      emitBodyLock(3, "// worker's acquire scan consumes it. No queue operation.\n");
      emitBodyLock(3, "if (mtPushReadyOwner != 0u) mtDensePushReadyReady[mtPushReadyDependent].store(1u, std::memory_order_release);\n");
    } else {
      emitBodyLock(3, "if (mtPushReadyOwner != 0u) mtDensePushReadyEnqueue(mtPushReadyOwner, mtPushReadyDependent);\n");
    }
    emitBodyLock(2, "}\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(0, "}\n");

    emitFuncDecl(0, "void S%s::mtDensePushReadyPrepare() {\n", name.c_str());
    emitBodyLock(1, "// Coordinator-only, between the previous cycle's worker-pool join and\n");
    emitBodyLock(1, "// mtWorkerPoolPost()'s generation release: the relaxed stores below\n");
    emitBodyLock(1, "// are published to every worker by the generation acquire, and the join\n");
    emitBodyLock(1, "// barrier guarantees no producer or consumer still runs from the\n");
    emitBodyLock(1, "// previous cycle (parity/epoch reset happens here, every cycle).\n");
    emitBodyLock(1, "for (int m = 0; m < kDensePushReadyMTaskCount; ++m)\n");
    emitBodyLock(2, "mtDensePushReadyPending[m].store(kDensePushReadyFanin[m], std::memory_order_relaxed);\n");
    if (pushReadyBitmap) {
      emitBodyLock(1, "// Fresh ready bytes: cleared globally here, then armed for the\n");
      emitBodyLock(1, "// zero-fan-in roots below; producers arm the rest through notify.\n");
      emitBodyLock(1, "for (int m = 0; m < kDensePushReadyMTaskCount; ++m)\n");
      emitBodyLock(2, "mtDensePushReadyReady[m].store(0u, std::memory_order_relaxed);\n");
    } else {
      emitBodyLock(1, "for (int mtPushReadyWorker = 0; mtPushReadyWorker < kDensePushReadyWorkerCount; ++mtPushReadyWorker) {\n");
      emitBodyLock(2, "mtDensePushReadyHead[mtPushReadyWorker].store(0u, std::memory_order_relaxed);\n");
      emitBodyLock(2, "mtDensePushReadyTail[mtPushReadyWorker].store(0u, std::memory_order_relaxed);\n");
      emitBodyLock(2, "const uint32_t mtPushReadySlotBegin = kDensePushReadyQueueOffset[mtPushReadyWorker];\n");
      emitBodyLock(2, "const uint32_t mtPushReadySlotEnd = kDensePushReadyQueueOffset[mtPushReadyWorker + 1];\n");
      emitBodyLock(2, "// Fresh Vyukov invariant: slot i's sequence equals i, positions 0.\n");
      emitBodyLock(2, "for (uint32_t i = 0; i < mtPushReadySlotEnd - mtPushReadySlotBegin; ++i)\n");
      emitBodyLock(3, "mtDensePushReadySlots[mtPushReadySlotBegin + i].seq.store(i, std::memory_order_relaxed);\n");
      emitBodyLock(1, "}\n");
    }
    if (pushReadyDebug) {
      emitBodyLock(1, "for (int m = 0; m < kDensePushReadyMTaskCount; ++m)\n");
      emitBodyLock(2, "mtDensePushReadyExecCount[m] = 0;\n");
    }
    emitBodyLock(1, "// Zero-fan-in roots of the push lanes; worker0's roots are discovered\n");
    emitBodyLock(1, "// by its unchanged pull path.\n");
    emitBodyLock(1, "for (int m = 0; m < kDensePushReadyMTaskCount; ++m) {\n");
    emitBodyLock(2, "if (kDensePushReadyFanin[m] != 0u) continue;\n");
    emitBodyLock(2, "const uint32_t mtPushReadyOwner = kDensePushReadyOwner[m];\n");
    emitBodyLock(2, "if (mtPushReadyOwner == 0u) continue;\n");
    if (pushReadyBitmap)
      emitBodyLock(2, "mtDensePushReadyReady[m].store(1u, std::memory_order_relaxed);\n");
    else
      emitBodyLock(2, "mtDensePushReadyEnqueue(mtPushReadyOwner, (uint32_t)m);\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(0, "}\n");

    if (!pushReadyBitmap) {
      if (pushReadyDirectTable) {
        // GSIM_MT_DENSE_PUSH_READY_DIRECT_TABLE: the generated switch over
        // every MTask id compiles to an N-case jump table re-entered on every
        // dispatch (65.86% of worker time in the v1 perf report); a flat const
        // member-pointer table keeps the exact same id -> body mapping with one
        // bounds check and one indirect member call. Definition follows the
        // kDenseDispatchTableW discipline: qualified nested typedef, static
        // const array, inside the owner-ready compile guard so a macro-off
        // build never sees an undefined declaration.
        emitBodyLock(0, "const S%s::MtDensePushReadyDispatchFn S%s::kDensePushReadyDispatchTable[kDensePushReadyMTaskCount] = {\n",
                     name.c_str(), name.c_str());
        for (int m = 0; m < nMTasks; m++)
          emitBodyLock(0, "&S%s::stepDenseMTask%d,\n", name.c_str(), m);
        emitBodyLock(0, "};\n");
        emitFuncDecl(0, "void S%s::stepDensePushReadyRun(uint32_t mtaskId) {\n", name.c_str());
        emitBodyLock(1, "if (unlikely(mtaskId >= (uint32_t)kDensePushReadyMTaskCount)) {\n");
        emitBodyLock(2, "fprintf(stderr, \"[push-ready] invalid mtask id %%u cycle=%%lu\\n\", mtaskId, (unsigned long) cycles);\n");
        emitBodyLock(2, "abort();\n");
        emitBodyLock(1, "}\n");
        emitBodyLock(1, "(this->*kDensePushReadyDispatchTable[mtaskId])();\n");
        emitBodyLock(0, "}\n");
      } else {
        emitFuncDecl(0, "void S%s::stepDensePushReadyRun(uint32_t mtaskId) {\n", name.c_str());
        emitBodyLock(1, "switch (mtaskId) {\n");
        for (int m = 0; m < nMTasks; m++) {
          emitBodyLock(2, "case %d: stepDenseMTask%d(); break;\n", m, m);
        }
        emitBodyLock(2, "default:\n");
        emitBodyLock(3, "fprintf(stderr, \"[push-ready] invalid mtask id %%u cycle=%%lu\\n\", mtaskId, (unsigned long) cycles);\n");
        emitBodyLock(3, "abort();\n");
        emitBodyLock(1, "}\n");
        emitBodyLock(0, "}\n");
      }

      emitFuncDecl(0, "void S%s::stepDensePushThreadWorker(int threadId) {\n", name.c_str());
      emitBodyLock(1, "if (unlikely(threadId <= 0 || threadId >= kDensePushReadyWorkerCount)) {\n");
      emitBodyLock(2, "fprintf(stderr, \"[push-ready] push worker invoked with invalid id %%d\\n\", threadId);\n");
      emitBodyLock(2, "abort();\n");
      emitBodyLock(1, "}\n");
      emitBodyLock(1, "const uint8_t mtPushReadyTarget = (cycles & 1) == 0 ? uint8_t{1} : uint8_t{0};\n");
      emitBodyLock(1, "const uint32_t mtPushReadyAssigned = kDensePushReadyAssigned[threadId];\n");
      emitBodyLock(1, "uint32_t mtPushReadyExecuted = 0;\n");
      emitBodyLock(1, "// Termination is the exact assigned count - never queue emptiness:\n");
      emitBodyLock(1, "// an empty queue only means producers elsewhere have not fired yet.\n");
      emitBodyLock(1, "unsigned mtPushReadySpin = 0;\n");
      emitBodyLock(1, "while (mtPushReadyExecuted < mtPushReadyAssigned) {\n");
      emitBodyLock(2, "uint32_t mtPushReadyMTask = 0;\n");
      emitBodyLock(2, "if (!mtDensePushReadyDequeue((uint32_t)threadId, mtPushReadyMTask)) {\n");
      if (spinYieldEvery == 0) {
        emitBodyLock(3, "mtWorkerPoolPause();\n");
      } else {
        emitBodyLock(3, "mtWorkerPoolPause(); if (++mtPushReadySpin > %d) { mtPushReadySpin = 0; std::this_thread::yield(); }\n", spinYieldEvery);
      }
      emitBodyLock(3, "continue;\n");
      emitBodyLock(2, "}\n");
      if (pushReadyDebug) {
        emitBodyLock(2, "// GSIM_MT_DENSE_PUSH_READY_DEBUG: owner + exactly-once validation.\n");
        emitBodyLock(2, "if (unlikely(kDensePushReadyOwner[mtPushReadyMTask] != (uint32_t)threadId)) {\n");
        emitBodyLock(3, "fprintf(stderr, \"[push-ready] owner mismatch mtask=%%u worker=%%d cycle=%%lu\\n\", mtPushReadyMTask, threadId, (unsigned long) cycles);\n");
        emitBodyLock(3, "abort();\n");
        emitBodyLock(2, "}\n");
        emitBodyLock(2, "if (unlikely(++mtDensePushReadyExecCount[mtPushReadyMTask] != 1u)) {\n");
        emitBodyLock(3, "fprintf(stderr, \"[push-ready] duplicate execution mtask=%%u worker=%%d cycle=%%lu\\n\", mtPushReadyMTask, threadId, (unsigned long) cycles);\n");
        emitBodyLock(3, "abort();\n");
        emitBodyLock(2, "}\n");
      }
      emitBodyLock(2, "stepDensePushReadyRun(mtPushReadyMTask);\n");
      emitBodyLock(2, "// Notify after all body state writes and before the token release\n");
      emitBodyLock(2, "// stores below: the pending-decrement chain (release) plus the queue\n");
      emitBodyLock(2, "// slot acquire carries the body writes to the dependent's owner.\n");
      emitBodyLock(2, "mtDensePushReadyNotify(mtPushReadyMTask);\n");
      emitBodyLock(2, "// Owner-ready release stores remain: worker0's pull lane still\n");
      emitBodyLock(2, "// consumes tokens published by push-lane producers.\n");
      emitBodyLock(2, "for (int j = kDenseOwnerReadyStoreOffsets[mtPushReadyMTask]; j < kDenseOwnerReadyStoreOffsets[mtPushReadyMTask + 1u]; ++j)\n");
      emitBodyLock(3, "mtDenseOwnerReadyTokens[kDenseOwnerReadyStoreList[j]].ready.store(mtPushReadyTarget, std::memory_order_release);\n");
      emitBodyLock(2, "++mtPushReadyExecuted;\n");
      emitBodyLock(1, "}\n");
      emitBodyLock(1, "// Queue drain before done acknowledgment: executed == assigned and\n");
      emitBodyLock(1, "// exactly one enqueue per task per cycle imply head == tail here.\n");
      emitBodyLock(1, "if (unlikely(mtDensePushReadyHead[threadId].load(std::memory_order_relaxed) != mtDensePushReadyTail[threadId].load(std::memory_order_relaxed))) {\n");
      emitBodyLock(2, "fprintf(stderr, \"[push-ready] worker %%d exited with a non-drained queue cycle=%%lu\\n\", threadId, (unsigned long) cycles);\n");
      emitBodyLock(2, "abort();\n");
      emitBodyLock(1, "}\n");
      emitBodyLock(0, "}\n");
    } else {
      // ---- GSIM_MT_DENSE_PUSH_READY_BITMAP worker (default off) ----
      // Queue-free push lane: each pool worker 1..N-1 selects its own slice
      // of the existing per-worker dispatch table (same entries the pull
      // lane's lookahead tail consumes) and scans only those entries; one
      // fires when its ready byte is set. The acquire load pairs with the
      // notify's release store, and the byte is cleared by its single
      // consumer before the body runs, so one scan visit executes a task at
      // most once per cycle and a set byte can never be consumed twice. No
      // MPMC sequence/tail CAS and no all-MTask switch: dispatch is the
      // entry's direct member pointer. Termination stays the exact assigned
      // count; an empty scan only pauses, it never terminates.
      emitFuncDecl(0, "void S%s::stepDensePushReadyBitmapWorker(int threadId) {\n", name.c_str());
      emitBodyLock(1, "if (unlikely(threadId <= 0 || threadId >= kDensePushReadyWorkerCount)) {\n");
      emitBodyLock(2, "fprintf(stderr, \"[push-ready] bitmap worker invoked with invalid id %%d\\n\", threadId);\n");
      emitBodyLock(2, "abort();\n");
      emitBodyLock(1, "}\n");
      emitBodyLock(1, "const uint8_t mtPushReadyTarget = (cycles & 1) == 0 ? uint8_t{1} : uint8_t{0};\n");
      emitBodyLock(1, "const uint32_t mtPushReadyAssigned = kDensePushReadyAssigned[threadId];\n");
      emitBodyLock(1, "const MtDenseDispatchEntry *mtPushReadyTableBegin = nullptr;\n");
      emitBodyLock(1, "const MtDenseDispatchEntry *mtPushReadyTableEnd = nullptr;\n");
      emitBodyLock(1, "switch (threadId) {\n");
      for (int t = 1; t < threadCount; t++) {
        emitBodyLock(2, "case %d:\n", t);
        emitBodyLock(3, "mtPushReadyTableBegin = kDenseDispatchTableW%d;\n", t);
        emitBodyLock(3, "mtPushReadyTableEnd = kDenseDispatchTableW%d + %d;\n", t, denseDispatchWorkerCounts[(size_t)t]);
        emitBodyLock(3, "break;\n");
      }
      emitBodyLock(1, "}\n");
      emitBodyLock(1, "uint32_t mtPushReadyExecuted = 0;\n");
      emitBodyLock(1, "// Termination is the exact assigned count - never scan emptiness:\n");
      emitBodyLock(1, "// an empty scan only means producers elsewhere have not fired yet.\n");
      emitBodyLock(1, "unsigned mtPushReadySpin = 0;\n");
      emitBodyLock(1, "while (mtPushReadyExecuted < mtPushReadyAssigned) {\n");
      emitBodyLock(2, "bool mtPushReadyFired = false;\n");
      emitBodyLock(2, "for (const MtDenseDispatchEntry *mtPushReadyEntry = mtPushReadyTableBegin; mtPushReadyEntry != mtPushReadyTableEnd; ++mtPushReadyEntry) {\n");
      emitBodyLock(3, "const uint32_t mtPushReadyMTask = mtPushReadyEntry->mtaskId;\n");
      emitBodyLock(3, "if (mtDensePushReadyReady[mtPushReadyMTask].load(std::memory_order_acquire) == 0u) continue;\n");
      emitBodyLock(3, "// Sole consumer of this byte: clear before the body so one scan\n");
      emitBodyLock(3, "// visit executes the task at most once per cycle.\n");
      emitBodyLock(3, "mtDensePushReadyReady[mtPushReadyMTask].store(0u, std::memory_order_relaxed);\n");
      if (pushReadyDebug) {
        emitBodyLock(3, "// GSIM_MT_DENSE_PUSH_READY_DEBUG: owner + exactly-once validation.\n");
        emitBodyLock(3, "if (unlikely(kDensePushReadyOwner[mtPushReadyMTask] != (uint32_t)threadId)) {\n");
        emitBodyLock(4, "fprintf(stderr, \"[push-ready] owner mismatch mtask=%%u worker=%%d cycle=%%lu\\n\", mtPushReadyMTask, threadId, (unsigned long) cycles);\n");
        emitBodyLock(4, "abort();\n");
        emitBodyLock(3, "}\n");
        emitBodyLock(3, "if (unlikely(++mtDensePushReadyExecCount[mtPushReadyMTask] != 1u)) {\n");
        emitBodyLock(4, "fprintf(stderr, \"[push-ready] duplicate execution mtask=%%u worker=%%d cycle=%%lu\\n\", mtPushReadyMTask, threadId, (unsigned long) cycles);\n");
        emitBodyLock(4, "abort();\n");
        emitBodyLock(3, "}\n");
      }
      emitBodyLock(3, "(this->*mtPushReadyEntry->fn)();\n");
      emitBodyLock(3, "// Notify after all body state writes and before the token release\n");
      emitBodyLock(3, "// stores below: the pending-decrement chain (release) plus the ready\n");
      emitBodyLock(3, "// byte acquire carries the body writes to the dependent's owner.\n");
      emitBodyLock(3, "mtDensePushReadyNotify(mtPushReadyMTask);\n");
      emitBodyLock(3, "// Owner-ready release stores remain: worker0's pull lane still\n");
      emitBodyLock(3, "// consumes tokens published by push-lane producers.\n");
      emitBodyLock(3, "for (uint32_t j = mtPushReadyEntry->storeBegin; j < mtPushReadyEntry->storeEnd; ++j)\n");
      emitBodyLock(4, "mtDenseOwnerReadyTokens[kDenseOwnerReadyStoreList[j]].ready.store(mtPushReadyTarget, std::memory_order_release);\n");
      emitBodyLock(3, "++mtPushReadyExecuted;\n");
      emitBodyLock(3, "mtPushReadyFired = true;\n");
      emitBodyLock(2, "}\n");
      emitBodyLock(2, "if (mtPushReadyFired) continue;\n");
      if (spinYieldEvery == 0) {
        emitBodyLock(2, "mtWorkerPoolPause();\n");
      } else {
        emitBodyLock(2, "mtWorkerPoolPause(); if (++mtPushReadySpin > %d) { mtPushReadySpin = 0; std::this_thread::yield(); }\n", spinYieldEvery);
      }
      emitBodyLock(1, "}\n");
      emitBodyLock(0, "}\n");
    }
    emitBodyLock(0, "#endif\n");
  }
  if (pushReadyHint) {
    // ---- GSIM_MT_DENSE_PUSH_READY_HINT runtime (default off; whole block
    // inside the owner-ready compile guard so a macro-off build keeps pure
    // pull for every worker and no hint state is referenced) ----
    // Queue-free pull-hint protocol: every producer completion decrements
    // each dependent's per-cycle pending counter once (the deduplicated CSR
    // the shadow knob validated) and the decrementer that observes old==1
    // release-stores the dependent's ready byte - the whole arrival
    // mechanism. Workers 1..N-1 acquire-load that byte in place of their
    // remote token wait lists inside the unchanged pull/lookahead control
    // flow; worker0 never reads it and stays fully original pull. The byte
    // is never cleared mid-cycle: each assigned task executes exactly once,
    // so a set byte cannot authorize a second execution.
    emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::mtDensePushReadyHintNotify(uint32_t mtaskId) {\n", name.c_str());
    emitBodyLock(1, "// Exactly one decrement per deduplicated CSR edge. fetch_sub(acq_rel)\n");
    emitBodyLock(1, "// makes the decrementer that observes old==1 synchronize with every\n");
    emitBodyLock(1, "// earlier producer on the same counter, so its release store of the\n");
    emitBodyLock(1, "// ready byte publishes all producers' body writes to the consumer's\n");
    emitBodyLock(1, "// acquire. Worker0-owned dependents are decremented too (their pending\n");
    emitBodyLock(1, "// accounting must stay exact) but their byte is never read: worker0\n");
    emitBodyLock(1, "// keeps pull discovery.\n");
    emitBodyLock(1, "for (uint32_t j = kDensePushReadyHintNotifyOffsets[mtaskId]; j < kDensePushReadyHintNotifyOffsets[mtaskId + 1u]; ++j) {\n");
    emitBodyLock(2, "const uint32_t mtPushReadyHintDependent = kDensePushReadyHintNotifyList[j];\n");
    emitBodyLock(2, "const uint32_t mtPushReadyHintPrevious = mtDensePushReadyHintPending[mtPushReadyHintDependent].fetch_sub(1u, std::memory_order_acq_rel);\n");
    emitBodyLock(2, "if (unlikely(mtPushReadyHintPrevious == 0u || mtPushReadyHintPrevious > kDensePushReadyHintFanin[mtPushReadyHintDependent])) {\n");
    emitBodyLock(3, "fprintf(stderr, \"[push-ready] hint fan-in underflow dependent=%%u producer=%%u cycle=%%lu previous=%%u fanin=%%u\\n\", mtPushReadyHintDependent, mtaskId, (unsigned long) cycles, mtPushReadyHintPrevious, kDensePushReadyHintFanin[mtPushReadyHintDependent]);\n");
    emitBodyLock(3, "abort();\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(2, "if (mtPushReadyHintPrevious == 1u) mtDensePushReadyHint[mtPushReadyHintDependent].store(1u, std::memory_order_release);\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(0, "}\n");

    emitFuncDecl(0, "void S%s::mtDensePushReadyHintPrepare() {\n", name.c_str());
    emitBodyLock(1, "// Coordinator-only, between the previous cycle's worker-pool join and\n");
    emitBodyLock(1, "// mtWorkerPoolPost()'s generation release: the relaxed stores below\n");
    emitBodyLock(1, "// are published to every worker by the generation acquire, and the join\n");
    emitBodyLock(1, "// barrier guarantees no producer or consumer still runs from the\n");
    emitBodyLock(1, "// previous cycle (parity/epoch reset happens here, every cycle).\n");
    emitBodyLock(1, "for (int m = 0; m < kDensePushReadyHintMTaskCount; ++m)\n");
    emitBodyLock(2, "mtDensePushReadyHintPending[m].store(kDensePushReadyHintFanin[m], std::memory_order_relaxed);\n");
    emitBodyLock(1, "// Fresh ready bytes: cleared globally here, armed for the zero-fan-in\n");
    emitBodyLock(1, "// roots below; producers arm the rest through notify. The byte stays\n");
    emitBodyLock(1, "// set for the whole cycle after arming - no consumer-side clear.\n");
    emitBodyLock(1, "for (int m = 0; m < kDensePushReadyHintMTaskCount; ++m)\n");
    emitBodyLock(2, "mtDensePushReadyHint[m].store(0u, std::memory_order_relaxed);\n");
    emitBodyLock(1, "// Zero-fan-in roots arm immediately; worker0's roots are never read\n");
    emitBodyLock(1, "// (its pull path authorizes them itself) but arming keeps the rule\n");
    emitBodyLock(1, "// uniform: ready==1 iff every mapped producer edge has been paid.\n");
    emitBodyLock(1, "for (int m = 0; m < kDensePushReadyHintMTaskCount; ++m) {\n");
    emitBodyLock(2, "if (kDensePushReadyHintFanin[m] == 0u) mtDensePushReadyHint[m].store(1u, std::memory_order_relaxed);\n");
    emitBodyLock(1, "}\n");
    emitBodyLock(0, "}\n");
    emitBodyLock(0, "#endif\n");
  }
  emitFuncDecl(0, "void S%s::stepDense() {\n", name.c_str());
  if (denseDuty) emitBodyLock(1, "MtDenseDutyGuard mtDutyStepGuard(mtDutyEnabled ? &mtDutyLanes[%d].stepWallNs : nullptr);\n", threadCount);
  if (denseBreakdownWindowCodegen) {
    emitBodyLock(1, "const bool mtDenseBreakdownWindowInCycle = mtDenseBreakdownWindowEnabled && cycles >= mtDenseBreakdownWindowStart && cycles - mtDenseBreakdownWindowStart < mtDenseBreakdownWindowCycles;\n");
    emitBodyLock(1, "const bool mtDenseBreakdownProfileInCycle = mtDenseBreakdownProfileEnabled && mtDenseBreakdownWindowInCycle;\n");
  }
  if (denseBreakdownProfileCodegen) {
    emitBodyLock(1, "std::chrono::steady_clock::time_point mtDenseBreakdownStepBegin;\n");
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownProfileInCycle)) mtDenseBreakdownStepBegin = std::chrono::steady_clock::now();\n");
    } else {
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownProfileEnabled)) mtDenseBreakdownStepBegin = std::chrono::steady_clock::now();\n");
    }
  }

  emitBodyLock(1, "std::chrono::steady_clock::time_point mtProfileStepBegin;\n");
  emitBodyLock(1, "if (unlikely(mtProfileEnabled)) mtProfileStepBegin = std::chrono::steady_clock::now();\n");
  if (denseDuty) emitBodyLock(1, "std::chrono::steady_clock::time_point mtDutyResetBegin; if (mtDutyEnabled) mtDutyResetBegin = std::chrono::steady_clock::now();\n");
  if (pscdBits) {
    // PSCD per-cycle epoch (flips 1/2 with cycle parity) and reset-window
    // flags, both emitted BEFORE resetAllDense(): subResetDenseN may fire
    // under its reset guard there (and mid-cycle from async reset supers),
    // calling mtDensePscdResetHit(), which needs the fresh epoch and the
    // cleared per-cycle flag. ResetActivePrev keeps the previous cycle's
    // verdict so consumer windows spanning a reset application are skipped.
    emitBodyLock(1, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    emitBodyLock(1, "mtDenseCycleEpoch = (cycles & 1) == 0 ? uint8_t{1} : uint8_t{2};\n");
    emitBodyLock(1, "mtDensePscdResetActivePrev = mtDensePscdResetActive;\n");
    emitBodyLock(1, "mtDensePscdResetActive = 0;\n");
    emitBodyLock(1, "#endif\n");
  }
  emitBodyLock(1, "resetAllDense();\n");
  if (denseDuty) emitBodyLock(1, "if (mtDutyEnabled) mtDutyLanes[%d].resetNs += (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDutyResetBegin).count();\n", threadCount);
  for (SuperNode* super : sortedSuper) {
    for (Node* member : super->member) {
      if (member->isReset() && member->type == NODE_REG_SRC) {
        emitBodyLock(1, "%s = %s;\n", RESET_NAME(member).c_str(), member->name.c_str());
      }
    }
  }
  // No counter reset needed — Verilator even/odd alternation handles it
  if (ownerReadyFlags) {
    emitBodyLock(1, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    emitBodyLock(1, "if (mtConfiguredWorkerCount == kDenseOwnerReadyWorkerCount && mtConfiguredWorkerCount > 1 && mtWorkerPoolEnabled && mtWorkerPoolThreadCount + 1 >= mtConfiguredWorkerCount) {\n");
    emitBodyLock(1, "#else\n");
    emitBodyLock(1, "if (mtConfiguredWorkerCount > 1 && mtWorkerPoolEnabled && mtWorkerPoolThreadCount + 1 >= mtConfiguredWorkerCount) {\n");
    emitBodyLock(1, "#endif\n");
  } else {
    // Fixed-owner counter path: ownership was baked in at generation time, so a
    // runtime worker-count mismatch would silently omit whole worker lanes. Require
    // the generated count; otherwise the caller falls through to the serial path.
    emitBodyLock(1, "if (mtConfiguredWorkerCount == %d && mtWorkerPoolEnabled && mtWorkerPoolThreadCount + 1 >= mtConfiguredWorkerCount) {\n", threadCount);
  }
  emitBodyLock(2, "mtWorkerPoolJobKind = 7;\n");
  emitBodyLock(2, "mtWorkerPoolCurrentWorkerCount = mtConfiguredWorkerCount;\n");
  if (ownerReadyFlags) {
    emitBodyLock(2, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    emitBodyLock(2, "if (!mtDenseOwnerReadyTokensPrimed) {\n");
    emitBodyLock(3, "const uint8_t mtDenseOwnerReadyRephaseValue = (cycles & 1) == 0 ? uint8_t{0} : uint8_t{1};\n");
    emitBodyLock(3, "for (int i = 0; i < kDenseOwnerReadyTokenCount; i++)\n");
    emitBodyLock(4, "mtDenseOwnerReadyTokens[kDenseOwnerReadyStoreList[i]].ready.store(mtDenseOwnerReadyRephaseValue, std::memory_order_relaxed);\n");
    emitBodyLock(3, "mtDenseOwnerReadyTokensPrimed = true;\n");
    emitBodyLock(2, "}\n");
    emitBodyLock(2, "#endif\n");
  }
  if (pushShadow) {
    // Push-shadow: re-arm every fan-in counter before dispatch. The previous
    // cycle's workers all joined (mtWorkerPoolWaitForDone) before stepDense
    // returned, so each cycle is globally barriered and a plain relaxed store
    // of the fan-in count is sufficient - no zero-epoch or parity trick.
    emitBodyLock(2, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    emitBodyLock(2, "for (int m = 0; m < kDensePushShadowMTaskCount; ++m)\n");
    emitBodyLock(3, "mtDensePushShadowPending[m].store(kDensePushShadowFanin[m], std::memory_order_relaxed);\n");
    emitBodyLock(2, "#endif\n");
  }
  if (denseBreakdownProfileCodegen) {
    emitBodyLock(2, "std::chrono::steady_clock::time_point mtDenseBreakdownPoolIdleBegin;\n");
    emitBodyLock(2, "std::chrono::steady_clock::time_point mtDenseBreakdownPoolDoneBegin;\n");
  }

  if (pushReady) {
    // Push-ready per-cycle setup: re-arm every pending counter to its fan-in,
    // reset every queue (head/tail/slot sequences) and enqueue the zero-fan-in
    // roots owned by push workers. Runs after the previous cycle's
    // mtWorkerPoolWaitForDone join and before mtWorkerPoolPost()'s generation
    // release, so the whole reset is publish-before-dispatch.
    emitBodyLock(2, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    emitBodyLock(2, "mtDensePushReadyPrepare();\n");
    emitBodyLock(2, "#endif\n");
  }
  if (pushReadyHint) {
    // Push-ready hint per-cycle setup: re-arm every pending counter to its
    // fan-in, clear every ready byte and arm the zero-fan-in roots. Runs
    // after the previous cycle's mtWorkerPoolWaitForDone join and before
    // mtWorkerPoolPost()'s generation release, so the whole reset is
    // publish-before-dispatch.
    emitBodyLock(2, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    emitBodyLock(2, "mtDensePushReadyHintPrepare();\n");
    emitBodyLock(2, "#endif\n");
  }
  if (tokenMaskHint) {
    // GSIM_MT_DENSE_TOKEN_MASK_HINT per-cycle reset: clear every consumer
    // mask word. Runs after the previous cycle's mtWorkerPoolWaitForDone
    // join (no producer or consumer still runs) and before
    // mtWorkerPoolPost()'s generation release, so the relaxed clears are
    // publish-before-dispatch. Masks are never cleared mid-cycle: each task
    // executes once per cycle, so accumulated bits stay meaningful until
    // this global barrier resets them.
    emitBodyLock(2, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    emitBodyLock(2, "for (int m = 0; m < kDenseTokenMaskMTaskCount; ++m)\n");
    emitBodyLock(3, "mtDenseTokenMask[m].store(uint16_t{0}, std::memory_order_relaxed);\n");
    emitBodyLock(2, "#endif\n");
  }
  emitBodyLock(2, "mtWorkerPoolPost();\n");
  if (denseBreakdownProfileCodegen) {
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownProfileInCycle)) mtDenseBreakdownPoolIdleBegin = std::chrono::steady_clock::now();\n");
    } else {
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownProfileEnabled)) mtDenseBreakdownPoolIdleBegin = std::chrono::steady_clock::now();\n");
    }
  }
  emitBodyLock(2, "stepDenseThreadWorker(0);\n");
  if (denseBreakdownProfileCodegen) {
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownProfileInCycle)) {\n");
    } else {
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownProfileEnabled)) {\n");
    }
    emitBodyLock(3, "mtDenseBreakdownPoolDoneBegin = std::chrono::steady_clock::now();\n");
    emitBodyLock(3, "uint64_t mtDenseBreakdownPoolIdleNsThisStep = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownPoolDoneBegin - mtDenseBreakdownPoolIdleBegin).count();\n");
    emitBodyLock(3, "if (unlikely(UINT64_MAX - mtDenseBreakdownPoolIdleNs < mtDenseBreakdownPoolIdleNsThisStep)) { fprintf(stderr, \"[mt-dense-breakdown] pool-idle counter overflow\\n\"); abort(); }\n");
    emitBodyLock(3, "mtDenseBreakdownPoolIdleNs += mtDenseBreakdownPoolIdleNsThisStep;\n");
    emitBodyLock(2, "}\n");
  }

  if (denseDuty) emitBodyLock(2, "std::chrono::steady_clock::time_point mtDutyJoinBegin; if (mtDutyEnabled) mtDutyJoinBegin = std::chrono::steady_clock::now();\n");
  emitBodyLock(2, "mtWorkerPoolWaitForDone(mtConfiguredWorkerCount - 1);\n");
  if (denseDuty) emitBodyLock(2, "if (mtDutyEnabled) mtDutyLanes[%d].joinNs += (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDutyJoinBegin).count();\n", threadCount);
  if (denseBreakdownWindowCodegen) {
    emitBodyLock(2, "if (unlikely(mtDenseBreakdownProfileInCycle && mtDenseBreakdownWindowCausalChainMode)) {\n");
    emitBodyLock(3, "const int mtDenseBreakdownWindowCausalSlot = mtDenseBreakdownWindowCurrentSlot;\n");
    emitBodyLock(3, "if (unlikely(mtDenseBreakdownWindowCausalSlot < 0 || mtDenseBreakdownWindowCausalSlot >= kDenseBreakdownWindowMaxCycles)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal invalid post-join slot\\n\"); abort(); }\n");
    emitBodyLock(3, "MtDenseBreakdownWindowCausalSummary &mtDenseBreakdownWindowCausalSummary = mtDenseBreakdownWindowCausalSummaries[mtDenseBreakdownWindowCausalSlot];\n");
    emitBodyLock(3, "mtDenseBreakdownWindowCausalSummary.causalBoundNs = 0; mtDenseBreakdownWindowCausalSummary.maxLagNs = 0; mtDenseBreakdownWindowCausalSummary.timestampBoundUncertaintyNs = 0; mtDenseBreakdownWindowCausalSummary.latestOwnerFinishNs = 0; mtDenseBreakdownWindowCausalSummary.makespanNs = 0; mtDenseBreakdownWindowCausalSummary.chainBodyNs = 0; mtDenseBreakdownWindowCausalSummary.chainReleaseNs = 0; mtDenseBreakdownWindowCausalSummary.chainGapNs = 0; mtDenseBreakdownWindowCausalSummary.sameOwnerPredecessorCount = 0; mtDenseBreakdownWindowCausalSummary.remoteTokenPredecessorCount = 0; mtDenseBreakdownWindowCausalSummary.waitObservationCount = 0; mtDenseBreakdownWindowCausalSummary.chainNodeCount = 0; mtDenseBreakdownWindowCausalSummary.chainEdgeCount = 0; mtDenseBreakdownWindowCausalSummary.latestOwner = -1; mtDenseBreakdownWindowCausalSummary.complete = false; mtDenseBreakdownWindowCausalSummary.incompleteMapping = false; mtDenseBreakdownWindowCausalSummary.clockRegression = false; mtDenseBreakdownWindowCausalSummary.overflow = false;\n");
    emitBodyLock(3, "for (int mtaskId = 0; mtaskId < kDenseBreakdownWindowAllOwnerMTaskCount; mtaskId ++) { mtDenseBreakdownWindowCausalRemotePredecessorEnds[mtaskId] = UINT64_MAX; mtDenseBreakdownWindowCausalRemotePredecessorTokens[mtaskId] = -1; mtDenseBreakdownWindowCausalSelectedPredecessors[mtaskId] = -1; mtDenseBreakdownWindowCausalSelectedPredecessorEnds[mtaskId] = 0; }\n");
    emitBodyLock(3, "for (int token = 0; token < kDenseBreakdownWindowCausalTokenCount; token ++) {\n");
    emitBodyLock(4, "const int mtDenseBreakdownWindowReadySlot = kDenseBreakdownWindowCausalTokenReadySlot[token]; const int mtDenseBreakdownWindowProducerMTask = kDenseBreakdownWindowCausalTokenProducerMTask[token]; const int mtDenseBreakdownWindowConsumerMTask = kDenseBreakdownWindowCausalTokenConsumerMTask[token]; const int mtDenseBreakdownWindowProducerOwner = kDenseBreakdownWindowCausalTokenProducerOwner[token]; const int mtDenseBreakdownWindowConsumerOwner = kDenseBreakdownWindowCausalTokenConsumerOwner[token];\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowReadySlot < 0 || mtDenseBreakdownWindowReadySlot >= kDenseOwnerReadyPhysicalSlotCount || kDenseBreakdownWindowCausalLogicalTokenByReadySlot[mtDenseBreakdownWindowReadySlot] != token || mtDenseBreakdownWindowProducerMTask < 0 || mtDenseBreakdownWindowProducerMTask >= kDenseBreakdownWindowAllOwnerMTaskCount || mtDenseBreakdownWindowConsumerMTask < 0 || mtDenseBreakdownWindowConsumerMTask >= kDenseBreakdownWindowAllOwnerMTaskCount || mtDenseBreakdownWindowProducerOwner < 0 || mtDenseBreakdownWindowProducerOwner >= mtConfiguredWorkerCount || mtDenseBreakdownWindowConsumerOwner < 0 || mtDenseBreakdownWindowConsumerOwner >= mtConfiguredWorkerCount || mtDenseBreakdownWindowProducerOwner == mtDenseBreakdownWindowConsumerOwner)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal static provenance incomplete\\n\"); abort(); }\n");
    emitBodyLock(4, "const MtDenseBreakdownWindowReadyToken &mtDenseBreakdownWindowTokenRelease = mtDenseBreakdownWindowReadyTokens[mtDenseBreakdownWindowCausalSlot][token];\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs == UINT64_MAX || mtDenseBreakdownWindowTokenRelease.releaseAfterOffsetNs == UINT64_MAX)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token release missing\\n\"); abort(); }\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowTokenRelease.releaseAfterOffsetNs < mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs)) { mtDenseBreakdownWindowCausalSummary.clockRegression = true; mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token release clock regression\\n\"); abort(); }\n");
    emitBodyLock(4, "const uint64_t mtDenseBreakdownWindowTokenUncertainty = mtDenseBreakdownWindowTokenRelease.releaseAfterOffsetNs - mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs; if (mtDenseBreakdownWindowTokenUncertainty > mtDenseBreakdownWindowCausalSummary.timestampBoundUncertaintyNs) mtDenseBreakdownWindowCausalSummary.timestampBoundUncertaintyNs = mtDenseBreakdownWindowTokenUncertainty;\n");
    emitBodyLock(4, "MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowProducer = mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowCausalSlot][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[mtDenseBreakdownWindowProducerMTask]];\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowProducer.mtaskId != (uint32_t)mtDenseBreakdownWindowProducerMTask || mtDenseBreakdownWindowProducer.ownerThreadId != (uint16_t)mtDenseBreakdownWindowProducerOwner || mtDenseBreakdownWindowProducer.bodyEndOffsetNs < mtDenseBreakdownWindowProducer.bodyStartOffsetNs || mtDenseBreakdownWindowProducer.releaseEndOffsetNs == UINT64_MAX || mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs < mtDenseBreakdownWindowProducer.bodyEndOffsetNs || mtDenseBreakdownWindowTokenRelease.releaseAfterOffsetNs > mtDenseBreakdownWindowProducer.releaseEndOffsetNs)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal producer record mismatch\\n\"); abort(); }\n");
    emitBodyLock(4, "uint64_t &mtDenseBreakdownWindowRemoteEnd = mtDenseBreakdownWindowCausalRemotePredecessorEnds[mtDenseBreakdownWindowConsumerMTask]; int &mtDenseBreakdownWindowRemoteToken = mtDenseBreakdownWindowCausalRemotePredecessorTokens[mtDenseBreakdownWindowConsumerMTask];\n");
    emitBodyLock(4, "if (mtDenseBreakdownWindowRemoteEnd == UINT64_MAX || mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs > mtDenseBreakdownWindowRemoteEnd || (mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs == mtDenseBreakdownWindowRemoteEnd && (mtDenseBreakdownWindowRemoteToken < 0 || token < mtDenseBreakdownWindowRemoteToken))) { mtDenseBreakdownWindowRemoteEnd = mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs; mtDenseBreakdownWindowRemoteToken = token; }\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowCausalSummary.remoteTokenPredecessorCount == UINT32_MAX)) { mtDenseBreakdownWindowCausalSummary.overflow = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal remote predecessor count overflow\\n\"); abort(); } mtDenseBreakdownWindowCausalSummary.remoteTokenPredecessorCount ++;\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "for (int worker = 0; worker < mtConfiguredWorkerCount; worker ++) {\n");
    emitBodyLock(4, "const uint32_t mtDenseBreakdownWindowWaitCount = mtDenseBreakdownWindowWaitCounts[mtDenseBreakdownWindowCausalSlot][worker];\n");
    emitBodyLock(4, "const uint32_t mtDenseBreakdownWindowWaitCapacity = (uint32_t)(kDenseBreakdownWindowWaitLaneOffsets[worker + 1] - kDenseBreakdownWindowWaitLaneOffsets[worker]);\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowWaitCount != mtDenseBreakdownWindowWaitCapacity)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal wait observation count mismatch\\n\"); abort(); }\n");
    emitBodyLock(4, "for (uint32_t w = 0; w < mtDenseBreakdownWindowWaitCount; w ++) {\n");
    emitBodyLock(5, "const MtDenseBreakdownWindowWait &mtDenseBreakdownWindowWait = mtDenseBreakdownWindowWaits[mtDenseBreakdownWindowCausalSlot][kDenseBreakdownWindowWaitLaneOffsets[worker] + w];\n");
    emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowWait.cycleSlot != (uint16_t)mtDenseBreakdownWindowCausalSlot || mtDenseBreakdownWindowWait.threadId != (uint16_t)worker || mtDenseBreakdownWindowWait.readySlot >= (uint32_t)kDenseOwnerReadyPhysicalSlotCount)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal wait record malformed\\n\"); abort(); }\n");
    emitBodyLock(5, "const int mtDenseBreakdownWindowLogicalToken = kDenseBreakdownWindowCausalLogicalTokenByReadySlot[mtDenseBreakdownWindowWait.readySlot];\n");
    emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowLogicalToken < 0 || mtDenseBreakdownWindowLogicalToken >= kDenseBreakdownWindowCausalTokenCount || kDenseBreakdownWindowCausalTokenConsumerMTask[mtDenseBreakdownWindowLogicalToken] != (int)mtDenseBreakdownWindowWait.consumerMtaskId || kDenseBreakdownWindowCausalTokenConsumerOwner[mtDenseBreakdownWindowLogicalToken] != worker)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal wait provenance mismatch\\n\"); abort(); }\n");
    emitBodyLock(5, "const MtDenseBreakdownWindowReadyToken &mtDenseBreakdownWindowTokenRelease = mtDenseBreakdownWindowReadyTokens[mtDenseBreakdownWindowCausalSlot][mtDenseBreakdownWindowLogicalToken];\n");
    emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowWait.endOffsetNs < mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs || mtDenseBreakdownWindowCausalSummary.waitObservationCount == UINT32_MAX)) { mtDenseBreakdownWindowCausalSummary.clockRegression = true; mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal wait timestamp regression\\n\"); abort(); }\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalSummary.waitObservationCount ++;\n");
    emitBodyLock(4, "}\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "int mtDenseBreakdownWindowLastMTaskByOwner[kDenseBreakdownWindowThreadCount];\n");
    emitBodyLock(3, "for (int worker = 0; worker < kDenseBreakdownWindowThreadCount; worker ++) mtDenseBreakdownWindowLastMTaskByOwner[worker] = -1;\n");
    emitBodyLock(3, "for (int mtaskId = 0; mtaskId < kDenseBreakdownWindowAllOwnerMTaskCount; mtaskId ++) {\n");
    emitBodyLock(4, "const MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowMTask = mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowCausalSlot][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[mtaskId]];\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowMTask.mtaskId != (uint32_t)mtaskId || mtDenseBreakdownWindowMTask.ownerThreadId >= (uint16_t)mtConfiguredWorkerCount || mtDenseBreakdownWindowMTask.bodyEndOffsetNs < mtDenseBreakdownWindowMTask.bodyStartOffsetNs || mtDenseBreakdownWindowMTask.releaseEndOffsetNs == UINT64_MAX || mtDenseBreakdownWindowMTask.releaseEndOffsetNs < mtDenseBreakdownWindowMTask.bodyEndOffsetNs)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal MTask record incomplete\\n\"); abort(); }\n");
    emitBodyLock(4, "const int mtDenseBreakdownWindowOwner = (int)mtDenseBreakdownWindowMTask.ownerThreadId; int mtDenseBreakdownWindowSelectedPredecessor = -1; uint64_t mtDenseBreakdownWindowSelectedEnd = 0;\n");
    emitBodyLock(4, "const int mtDenseBreakdownWindowPreviousMTask = mtDenseBreakdownWindowLastMTaskByOwner[mtDenseBreakdownWindowOwner];\n");
    emitBodyLock(4, "if (mtDenseBreakdownWindowPreviousMTask >= 0) { const MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowPrevious = mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowCausalSlot][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[mtDenseBreakdownWindowPreviousMTask]]; if (unlikely(mtDenseBreakdownWindowPrevious.releaseEndOffsetNs == UINT64_MAX || mtDenseBreakdownWindowPrevious.ownerThreadId != (uint16_t)mtDenseBreakdownWindowOwner || mtDenseBreakdownWindowCausalSummary.sameOwnerPredecessorCount == UINT32_MAX)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal same-owner predecessor missing\\n\"); abort(); } mtDenseBreakdownWindowSelectedPredecessor = mtDenseBreakdownWindowPreviousMTask; mtDenseBreakdownWindowSelectedEnd = mtDenseBreakdownWindowPrevious.releaseEndOffsetNs; mtDenseBreakdownWindowCausalSummary.sameOwnerPredecessorCount ++; }\n");
    emitBodyLock(4, "const int mtDenseBreakdownWindowRemoteToken = mtDenseBreakdownWindowCausalRemotePredecessorTokens[mtaskId];\n");
    emitBodyLock(4, "if (mtDenseBreakdownWindowRemoteToken >= 0) { const int mtDenseBreakdownWindowRemoteProducer = kDenseBreakdownWindowCausalTokenProducerMTask[mtDenseBreakdownWindowRemoteToken]; const uint64_t mtDenseBreakdownWindowRemoteEnd = mtDenseBreakdownWindowCausalRemotePredecessorEnds[mtaskId]; if (unlikely(mtDenseBreakdownWindowRemoteEnd == UINT64_MAX || mtDenseBreakdownWindowRemoteProducer < 0 || mtDenseBreakdownWindowRemoteProducer >= mtaskId)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal remote predecessor invalid\\n\"); abort(); } if (mtDenseBreakdownWindowSelectedPredecessor < 0 || mtDenseBreakdownWindowRemoteEnd > mtDenseBreakdownWindowSelectedEnd || (mtDenseBreakdownWindowRemoteEnd == mtDenseBreakdownWindowSelectedEnd && mtDenseBreakdownWindowRemoteProducer < mtDenseBreakdownWindowSelectedPredecessor)) { mtDenseBreakdownWindowSelectedPredecessor = mtDenseBreakdownWindowRemoteProducer; mtDenseBreakdownWindowSelectedEnd = mtDenseBreakdownWindowRemoteEnd; } }\n");
    emitBodyLock(4, "if (mtDenseBreakdownWindowSelectedPredecessor >= 0) { if (unlikely(mtDenseBreakdownWindowMTask.bodyStartOffsetNs < mtDenseBreakdownWindowSelectedEnd)) { mtDenseBreakdownWindowCausalSummary.clockRegression = true; mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal predecessor timestamp regression\\n\"); abort(); } const uint64_t mtDenseBreakdownWindowLag = mtDenseBreakdownWindowMTask.bodyStartOffsetNs - mtDenseBreakdownWindowSelectedEnd; if (mtDenseBreakdownWindowLag > mtDenseBreakdownWindowCausalSummary.maxLagNs) mtDenseBreakdownWindowCausalSummary.maxLagNs = mtDenseBreakdownWindowLag; if (mtDenseBreakdownWindowSelectedEnd > mtDenseBreakdownWindowCausalSummary.causalBoundNs) mtDenseBreakdownWindowCausalSummary.causalBoundNs = mtDenseBreakdownWindowSelectedEnd; mtDenseBreakdownWindowCausalSelectedPredecessors[mtaskId] = mtDenseBreakdownWindowSelectedPredecessor; mtDenseBreakdownWindowCausalSelectedPredecessorEnds[mtaskId] = mtDenseBreakdownWindowSelectedEnd; }\n");
    emitBodyLock(4, "mtDenseBreakdownWindowLastMTaskByOwner[mtDenseBreakdownWindowOwner] = mtaskId;\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "int mtDenseBreakdownWindowLatestOwner = -1; uint64_t mtDenseBreakdownWindowLatestFinish = 0;\n");
    emitBodyLock(3, "for (int worker = 0; worker < mtConfiguredWorkerCount; worker ++) { if (mtDenseBreakdownWindowLastMTaskByOwner[worker] < 0) continue; const MtDenseBreakdownWindowWorker &mtDenseBreakdownWindowWorker = mtDenseBreakdownWindowWorkers[mtDenseBreakdownWindowCausalSlot][worker]; if (unlikely(mtDenseBreakdownWindowWorker.finishOffsetNs < mtDenseBreakdownWindowWorker.startOffsetNs)) { mtDenseBreakdownWindowCausalSummary.clockRegression = true; mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal worker finish regression\\n\"); abort(); } if (mtDenseBreakdownWindowLatestOwner < 0 || mtDenseBreakdownWindowWorker.finishOffsetNs > mtDenseBreakdownWindowLatestFinish || (mtDenseBreakdownWindowWorker.finishOffsetNs == mtDenseBreakdownWindowLatestFinish && worker < mtDenseBreakdownWindowLatestOwner)) { mtDenseBreakdownWindowLatestOwner = worker; mtDenseBreakdownWindowLatestFinish = mtDenseBreakdownWindowWorker.finishOffsetNs; } }\n");
    emitBodyLock(3, "if (unlikely(mtDenseBreakdownWindowLatestOwner < 0)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal latest owner missing\\n\"); abort(); }\n");
    emitBodyLock(3, "mtDenseBreakdownWindowCausalSummary.latestOwner = mtDenseBreakdownWindowLatestOwner; mtDenseBreakdownWindowCausalSummary.latestOwnerFinishNs = mtDenseBreakdownWindowLatestFinish; mtDenseBreakdownWindowCausalSummary.makespanNs = mtDenseBreakdownWindowLatestFinish;\n");
    emitBodyLock(3, "const uint32_t mtDenseBreakdownWindowVisitMark = (uint32_t)mtDenseBreakdownWindowCausalSlot + 1; int mtDenseBreakdownWindowChainMTask = mtDenseBreakdownWindowLastMTaskByOwner[mtDenseBreakdownWindowLatestOwner]; uint64_t mtDenseBreakdownWindowChainCausalReleaseEnd = UINT64_MAX;\n");
    emitBodyLock(3, "while (mtDenseBreakdownWindowChainMTask >= 0) { if (unlikely(mtDenseBreakdownWindowChainMTask >= kDenseBreakdownWindowAllOwnerMTaskCount || mtDenseBreakdownWindowCausalSummary.chainNodeCount >= (uint32_t)kDenseBreakdownWindowAllOwnerMTaskCount || mtDenseBreakdownWindowCausalVisitMarks[mtDenseBreakdownWindowChainMTask] == mtDenseBreakdownWindowVisitMark)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal backtrack cycle or bound failure\\n\"); abort(); } mtDenseBreakdownWindowCausalVisitMarks[mtDenseBreakdownWindowChainMTask] = mtDenseBreakdownWindowVisitMark; const MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowChainNode = mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowCausalSlot][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[mtDenseBreakdownWindowChainMTask]]; const uint64_t mtDenseBreakdownWindowChainReleaseEnd = mtDenseBreakdownWindowChainCausalReleaseEnd == UINT64_MAX ? mtDenseBreakdownWindowChainNode.releaseEndOffsetNs : mtDenseBreakdownWindowChainCausalReleaseEnd; if (unlikely(mtDenseBreakdownWindowChainReleaseEnd < mtDenseBreakdownWindowChainNode.bodyEndOffsetNs)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal chain release boundary invalid\\n\"); abort(); } const uint64_t mtDenseBreakdownWindowChainBodyNs = mtDenseBreakdownWindowChainNode.bodyEndOffsetNs - mtDenseBreakdownWindowChainNode.bodyStartOffsetNs; const uint64_t mtDenseBreakdownWindowReleaseNs = mtDenseBreakdownWindowChainReleaseEnd - mtDenseBreakdownWindowChainNode.bodyEndOffsetNs; if (unlikely(UINT64_MAX - mtDenseBreakdownWindowCausalSummary.chainBodyNs < mtDenseBreakdownWindowChainBodyNs || UINT64_MAX - mtDenseBreakdownWindowCausalSummary.chainReleaseNs < mtDenseBreakdownWindowReleaseNs)) { mtDenseBreakdownWindowCausalSummary.overflow = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal chain sum overflow\\n\"); abort(); } mtDenseBreakdownWindowCausalSummary.chainBodyNs += mtDenseBreakdownWindowChainBodyNs; mtDenseBreakdownWindowCausalSummary.chainReleaseNs += mtDenseBreakdownWindowReleaseNs; mtDenseBreakdownWindowCausalSummary.chainNodeCount ++; const int mtDenseBreakdownWindowChainPredecessor = mtDenseBreakdownWindowCausalSelectedPredecessors[mtDenseBreakdownWindowChainMTask]; if (mtDenseBreakdownWindowChainPredecessor < 0) break; const uint64_t mtDenseBreakdownWindowChainPredecessorEnd = mtDenseBreakdownWindowCausalSelectedPredecessorEnds[mtDenseBreakdownWindowChainMTask]; if (unlikely(mtDenseBreakdownWindowChainNode.bodyStartOffsetNs < mtDenseBreakdownWindowChainPredecessorEnd || mtDenseBreakdownWindowCausalSummary.chainEdgeCount == UINT32_MAX || UINT64_MAX - mtDenseBreakdownWindowCausalSummary.chainGapNs < mtDenseBreakdownWindowChainNode.bodyStartOffsetNs - mtDenseBreakdownWindowChainPredecessorEnd)) { mtDenseBreakdownWindowCausalSummary.clockRegression = true; mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal chain gap invalid\\n\"); abort(); } mtDenseBreakdownWindowCausalSummary.chainGapNs += mtDenseBreakdownWindowChainNode.bodyStartOffsetNs - mtDenseBreakdownWindowChainPredecessorEnd; mtDenseBreakdownWindowCausalSummary.chainEdgeCount ++; mtDenseBreakdownWindowChainCausalReleaseEnd = mtDenseBreakdownWindowChainPredecessorEnd; mtDenseBreakdownWindowChainMTask = mtDenseBreakdownWindowChainPredecessor; }\n");
    emitBodyLock(3, "if (mtDenseBreakdownWindowCausalHotspotsMode) {\n");
    emitBodyLock(4, "MtDenseBreakdownWindowCausalHotspotTotals mtDenseBreakdownWindowCausalHotspotCycleTotals = {};\n");
    emitBodyLock(4, "auto mtDenseBreakdownWindowCausalHotspotsAdd = [&](uint64_t &target, uint64_t value) { if (unlikely(UINT64_MAX - target < value)) { mtDenseBreakdownWindowCausalSummary.overflow = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots aggregate overflow\\n\"); abort(); } target += value; };\n");
    emitBodyLock(4, "int mtDenseBreakdownWindowCausalHotspotMTaskId = mtDenseBreakdownWindowLastMTaskByOwner[mtDenseBreakdownWindowLatestOwner];\n");
    emitBodyLock(4, "uint64_t mtDenseBreakdownWindowCausalHotspotReleaseEnd = UINT64_MAX;\n");
    emitBodyLock(4, "uint32_t mtDenseBreakdownWindowCausalHotspotNodes = 0;\n");
    emitBodyLock(4, "while (mtDenseBreakdownWindowCausalHotspotMTaskId >= 0) {\n");
    emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowCausalHotspotMTaskId >= kDenseBreakdownWindowAllOwnerMTaskCount || mtDenseBreakdownWindowCausalHotspotNodes >= (uint32_t)kDenseBreakdownWindowAllOwnerMTaskCount)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots logical MTask mapping invalid\\n\"); abort(); }\n");
    emitBodyLock(5, "const MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowCausalHotspotNode = mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowCausalSlot][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[mtDenseBreakdownWindowCausalHotspotMTaskId]];\n");
    emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowCausalHotspotNode.mtaskId != (uint32_t)mtDenseBreakdownWindowCausalHotspotMTaskId || mtDenseBreakdownWindowCausalHotspotNode.bodyEndOffsetNs < mtDenseBreakdownWindowCausalHotspotNode.bodyStartOffsetNs || mtDenseBreakdownWindowCausalHotspotNode.releaseEndOffsetNs == UINT64_MAX)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots MTask record invalid\\n\"); abort(); }\n");
    emitBodyLock(5, "const uint64_t mtDenseBreakdownWindowCausalHotspotNodeReleaseEnd = mtDenseBreakdownWindowCausalHotspotReleaseEnd == UINT64_MAX ? mtDenseBreakdownWindowCausalHotspotNode.releaseEndOffsetNs : mtDenseBreakdownWindowCausalHotspotReleaseEnd;\n");
    emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowCausalHotspotNodeReleaseEnd < mtDenseBreakdownWindowCausalHotspotNode.bodyEndOffsetNs)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots release boundary invalid\\n\"); abort(); }\n");
    emitBodyLock(5, "const uint64_t mtDenseBreakdownWindowCausalHotspotBodyNs = mtDenseBreakdownWindowCausalHotspotNode.bodyEndOffsetNs - mtDenseBreakdownWindowCausalHotspotNode.bodyStartOffsetNs;\n");
    emitBodyLock(5, "const uint64_t mtDenseBreakdownWindowCausalHotspotReleaseNs = mtDenseBreakdownWindowCausalHotspotNodeReleaseEnd - mtDenseBreakdownWindowCausalHotspotNode.bodyEndOffsetNs;\n");
    emitBodyLock(5, "MtDenseBreakdownWindowCausalHotspotMTask &mtDenseBreakdownWindowCausalHotspotMTask = mtDenseBreakdownWindowCausalHotspotMTasks[mtDenseBreakdownWindowCausalHotspotMTaskId];\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotMTask.count, 1);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotMTask.bodyNs, mtDenseBreakdownWindowCausalHotspotBodyNs);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotMTask.releaseNs, mtDenseBreakdownWindowCausalHotspotReleaseNs);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotTotals.count, 1);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotTotals.bodyNs, mtDenseBreakdownWindowCausalHotspotBodyNs);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotTotals.releaseNs, mtDenseBreakdownWindowCausalHotspotReleaseNs);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotCycleTotals.count, 1);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotCycleTotals.bodyNs, mtDenseBreakdownWindowCausalHotspotBodyNs);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotCycleTotals.releaseNs, mtDenseBreakdownWindowCausalHotspotReleaseNs);\n");
    emitBodyLock(5, "if (mtDenseBreakdownWindowCausalHotspotNodes == 0) { mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotMTask.latestTailCount, 1); mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotTotals.latestTailCount, 1); mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotCycleTotals.latestTailCount, 1); }\n");
    emitBodyLock(5, "const int mtDenseBreakdownWindowCausalHotspotPredecessor = mtDenseBreakdownWindowCausalSelectedPredecessors[mtDenseBreakdownWindowCausalHotspotMTaskId];\n");
    emitBodyLock(5, "if (mtDenseBreakdownWindowCausalHotspotPredecessor < 0) break;\n");
    emitBodyLock(5, "const uint64_t mtDenseBreakdownWindowCausalHotspotPredecessorEnd = mtDenseBreakdownWindowCausalSelectedPredecessorEnds[mtDenseBreakdownWindowCausalHotspotMTaskId];\n");
    emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindowCausalHotspotPredecessor >= mtDenseBreakdownWindowCausalHotspotMTaskId || mtDenseBreakdownWindowCausalHotspotNode.bodyStartOffsetNs < mtDenseBreakdownWindowCausalHotspotPredecessorEnd)) { mtDenseBreakdownWindowCausalSummary.clockRegression = true; mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots predecessor mapping invalid\\n\"); abort(); }\n");
    emitBodyLock(5, "const uint64_t mtDenseBreakdownWindowCausalHotspotGapNs = mtDenseBreakdownWindowCausalHotspotNode.bodyStartOffsetNs - mtDenseBreakdownWindowCausalHotspotPredecessorEnd;\n");
    emitBodyLock(5, "const int mtDenseBreakdownWindowCausalHotspotRemoteToken = mtDenseBreakdownWindowCausalRemotePredecessorTokens[mtDenseBreakdownWindowCausalHotspotMTaskId];\n");
    emitBodyLock(5, "bool mtDenseBreakdownWindowCausalHotspotSelectedRemoteToken = false;\n");
    emitBodyLock(5, "if (mtDenseBreakdownWindowCausalHotspotRemoteToken >= 0) { if (unlikely(mtDenseBreakdownWindowCausalHotspotRemoteToken >= kDenseBreakdownWindowCausalTokenCount)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots remote token index invalid\\n\"); abort(); } mtDenseBreakdownWindowCausalHotspotSelectedRemoteToken = kDenseBreakdownWindowCausalTokenProducerMTask[mtDenseBreakdownWindowCausalHotspotRemoteToken] == mtDenseBreakdownWindowCausalHotspotPredecessor && mtDenseBreakdownWindowCausalRemotePredecessorEnds[mtDenseBreakdownWindowCausalHotspotMTaskId] == mtDenseBreakdownWindowCausalHotspotPredecessorEnd; }\n");
    emitBodyLock(5, "if (mtDenseBreakdownWindowCausalHotspotSelectedRemoteToken && kDenseBreakdownWindowCausalSameOwnerProducerMTask[mtDenseBreakdownWindowCausalHotspotMTaskId] == mtDenseBreakdownWindowCausalHotspotPredecessor) mtDenseBreakdownWindowCausalHotspotSelectedRemoteToken = false;\n");
    emitBodyLock(5, "if (mtDenseBreakdownWindowCausalHotspotSelectedRemoteToken) {\n");
    emitBodyLock(6, "const int mtDenseBreakdownWindowCausalHotspotReadySlot = kDenseBreakdownWindowCausalTokenReadySlot[mtDenseBreakdownWindowCausalHotspotRemoteToken];\n");
    emitBodyLock(6, "if (unlikely(mtDenseBreakdownWindowCausalHotspotReadySlot < 0 || mtDenseBreakdownWindowCausalHotspotReadySlot >= kDenseOwnerReadyPhysicalSlotCount || kDenseBreakdownWindowCausalLogicalTokenByReadySlot[mtDenseBreakdownWindowCausalHotspotReadySlot] != mtDenseBreakdownWindowCausalHotspotRemoteToken || kDenseBreakdownWindowCausalTokenConsumerMTask[mtDenseBreakdownWindowCausalHotspotRemoteToken] != mtDenseBreakdownWindowCausalHotspotMTaskId)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots remote token provenance invalid\\n\"); abort(); }\n");
    emitBodyLock(6, "if (unlikely(mtDenseBreakdownWindowReadyTokens[mtDenseBreakdownWindowCausalSlot][mtDenseBreakdownWindowCausalHotspotRemoteToken].releaseBeforeOffsetNs != mtDenseBreakdownWindowCausalHotspotPredecessorEnd)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots remote release-before endpoint mismatch\\n\"); abort(); }\n");
    emitBodyLock(6, "MtDenseBreakdownWindowCausalHotspotEdge &mtDenseBreakdownWindowCausalHotspotEdge = mtDenseBreakdownWindowCausalHotspotRemoteTokenEdges[mtDenseBreakdownWindowCausalHotspotRemoteToken];\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotMTask.remoteTokenPredCount, 1);\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotTotals.remoteTokenPredCount, 1);\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotCycleTotals.remoteTokenPredCount, 1);\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotEdge.count, 1);\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotEdge.gapNs, mtDenseBreakdownWindowCausalHotspotGapNs);\n");
    emitBodyLock(5, "} else {\n");
    emitBodyLock(6, "const int mtDenseBreakdownWindowCausalHotspotSameOwnerProducer = kDenseBreakdownWindowCausalSameOwnerProducerMTask[mtDenseBreakdownWindowCausalHotspotMTaskId];\n");
    emitBodyLock(6, "const MtDenseBreakdownWindowAllOwnerMTask &mtDenseBreakdownWindowCausalHotspotProducer = mtDenseBreakdownWindowAllOwnerMTasks[mtDenseBreakdownWindowCausalSlot][kDenseBreakdownWindowAllOwnerMTaskRecordIndex[mtDenseBreakdownWindowCausalHotspotPredecessor]];\n");
    emitBodyLock(6, "if (unlikely(mtDenseBreakdownWindowCausalHotspotSameOwnerProducer != mtDenseBreakdownWindowCausalHotspotPredecessor || mtDenseBreakdownWindowCausalHotspotProducer.mtaskId != (uint32_t)mtDenseBreakdownWindowCausalHotspotPredecessor || mtDenseBreakdownWindowCausalHotspotProducer.ownerThreadId != mtDenseBreakdownWindowCausalHotspotNode.ownerThreadId)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots same-owner mapping invalid\\n\"); abort(); }\n");
    emitBodyLock(6, "MtDenseBreakdownWindowCausalHotspotEdge &mtDenseBreakdownWindowCausalHotspotEdge = mtDenseBreakdownWindowCausalHotspotSameOwnerEdges[mtDenseBreakdownWindowCausalHotspotMTaskId];\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotMTask.sameOwnerPredCount, 1);\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotTotals.sameOwnerPredCount, 1);\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotCycleTotals.sameOwnerPredCount, 1);\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotEdge.count, 1);\n");
    emitBodyLock(6, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotEdge.gapNs, mtDenseBreakdownWindowCausalHotspotGapNs);\n");
    emitBodyLock(5, "}\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotMTask.gapNs, mtDenseBreakdownWindowCausalHotspotGapNs);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotTotals.gapNs, mtDenseBreakdownWindowCausalHotspotGapNs);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotTotals.edgeCount, 1);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotCycleTotals.gapNs, mtDenseBreakdownWindowCausalHotspotGapNs);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotsAdd(mtDenseBreakdownWindowCausalHotspotCycleTotals.edgeCount, 1);\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotReleaseEnd = mtDenseBreakdownWindowCausalHotspotPredecessorEnd;\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotMTaskId = mtDenseBreakdownWindowCausalHotspotPredecessor;\n");
    emitBodyLock(5, "mtDenseBreakdownWindowCausalHotspotNodes ++;\n");
    emitBodyLock(4, "}\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowCausalHotspotCycleTotals.count != mtDenseBreakdownWindowCausalSummary.chainNodeCount || mtDenseBreakdownWindowCausalHotspotCycleTotals.bodyNs != mtDenseBreakdownWindowCausalSummary.chainBodyNs || mtDenseBreakdownWindowCausalHotspotCycleTotals.releaseNs != mtDenseBreakdownWindowCausalSummary.chainReleaseNs || mtDenseBreakdownWindowCausalHotspotCycleTotals.gapNs != mtDenseBreakdownWindowCausalSummary.chainGapNs || mtDenseBreakdownWindowCausalHotspotCycleTotals.edgeCount != mtDenseBreakdownWindowCausalSummary.chainEdgeCount || mtDenseBreakdownWindowCausalHotspotCycleTotals.sameOwnerPredCount > mtDenseBreakdownWindowCausalHotspotCycleTotals.edgeCount || mtDenseBreakdownWindowCausalHotspotCycleTotals.remoteTokenPredCount > mtDenseBreakdownWindowCausalHotspotCycleTotals.edgeCount - mtDenseBreakdownWindowCausalHotspotCycleTotals.sameOwnerPredCount || mtDenseBreakdownWindowCausalHotspotCycleTotals.latestTailCount != 1)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots cycle reconciliation failed\\n\"); abort(); }\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowCausalHotspotCycleTotals.sameOwnerPredCount > mtDenseBreakdownWindowCausalHotspotCycleTotals.edgeCount || mtDenseBreakdownWindowCausalHotspotCycleTotals.remoteTokenPredCount != mtDenseBreakdownWindowCausalHotspotCycleTotals.edgeCount - mtDenseBreakdownWindowCausalHotspotCycleTotals.sameOwnerPredCount)) { mtDenseBreakdownWindowCausalSummary.incompleteMapping = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots selected edge-kind reconciliation failed\\n\"); abort(); }\n");
    emitBodyLock(4, "if (unlikely(mtDenseBreakdownWindowCausalHotspotCycleTotals.sameOwnerPredCount > UINT32_MAX || mtDenseBreakdownWindowCausalHotspotCycleTotals.remoteTokenPredCount > UINT32_MAX)) { mtDenseBreakdownWindowCausalSummary.overflow = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causalhotspots selected predecessor count overflow\\n\"); abort(); }\n");
    emitBodyLock(4, "mtDenseBreakdownWindowCausalSummary.sameOwnerPredecessorCount = (uint32_t)mtDenseBreakdownWindowCausalHotspotCycleTotals.sameOwnerPredCount; mtDenseBreakdownWindowCausalSummary.remoteTokenPredecessorCount = (uint32_t)mtDenseBreakdownWindowCausalHotspotCycleTotals.remoteTokenPredCount;\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "if (mtDenseBreakdownWindowCausalSummary.timestampBoundUncertaintyNs > mtDenseBreakdownWindowCausalTimestampBoundUncertaintyNs) mtDenseBreakdownWindowCausalTimestampBoundUncertaintyNs = mtDenseBreakdownWindowCausalSummary.timestampBoundUncertaintyNs; mtDenseBreakdownWindowCausalSummary.complete = true;\n");
    emitBodyLock(2, "}\n");
  }
  if (denseBreakdownProfileCodegen) {
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownProfileInCycle)) {\n");
    } else {
      emitBodyLock(2, "if (unlikely(mtDenseBreakdownProfileEnabled)) {\n");
    }
    emitBodyLock(3, "uint64_t mtDenseBreakdownPoolDoneNsThisStep = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDenseBreakdownPoolDoneBegin).count();\n");
    emitBodyLock(3, "if (unlikely(UINT64_MAX - mtDenseBreakdownPoolDoneNs < mtDenseBreakdownPoolDoneNsThisStep)) { fprintf(stderr, \"[mt-dense-breakdown] pool-done counter overflow\\n\"); abort(); }\n");
    emitBodyLock(3, "mtDenseBreakdownPoolDoneNs += mtDenseBreakdownPoolDoneNsThisStep;\n");
    emitBodyLock(2, "}\n");
  }

  emitBodyLock(1, "} else {\n");
  if (ownerReadyFlags) {
    emitBodyLock(2, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    emitBodyLock(2, "mtDenseOwnerReadyTokensPrimed = false;\n");
    emitBodyLock(2, "#endif\n");
  }
  if (pushShadow) {
    // Push-shadow serial fallback: same per-cycle re-arm (single thread, no
    // barrier needed), then a notify after each body. Fixed ascending order
    // is a valid topological order of the census (every edge is forward in
    // MTask id), so the entry checks must all observe zero pending here.
    emitBodyLock(2, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    emitBodyLock(2, "for (int m = 0; m < kDensePushShadowMTaskCount; ++m)\n");
    emitBodyLock(3, "mtDensePushShadowPending[m].store(kDensePushShadowFanin[m], std::memory_order_relaxed);\n");
    emitBodyLock(2, "#endif\n");
  }
  if (pushReady) {
    // Push-ready serial fallback setup: same re-arm/reset/root-enqueue as the
    // pool path (single thread, program order publishes everything).
    emitBodyLock(2, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    emitBodyLock(2, "mtDensePushReadyPrepare();\n");
    emitBodyLock(2, "#endif\n");
  }
  if (pushReadyHint) {
    // Push-ready hint serial fallback setup: same re-arm/clear/root-arm as
    // the pool path (single thread, program order publishes everything).
    emitBodyLock(2, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
    emitBodyLock(2, "mtDensePushReadyHintPrepare();\n");
    emitBodyLock(2, "#endif\n");
  }
  for (int mtaskId = 0; mtaskId < nMTasks; mtaskId++) {
    emitBodyLock(2, "stepDenseMTask%d();\n", mtaskId);
    if (pushShadow) {
      // Serial fallback notify: lane threadCount is the coordinator lane
      // (sole writer; workers use lanes 0..threadCount-1 when the pool runs).
      emitBodyLock(2, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
      emitBodyLock(2, "mtDensePushShadowNotify(%d, %du);\n", mtaskId, threadCount);
      emitBodyLock(2, "#endif\n");
    }
    if (pushReady) {
      // Push-ready serial notify: keeps pending counters and the notify CSR
      // exercised exactly once per body on the fallback lane, mirroring the
      // parallel path. Queue writes are harmless: prepare resets before the
      // next dispatch.
      emitBodyLock(2, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
      emitBodyLock(2, "mtDensePushReadyNotify(%d);\n", mtaskId);
      emitBodyLock(2, "#endif\n");
    }
    if (pscdBits) {
      // Serial fallback: no ready tokens exist on this path, but the change
      // bytes are still released after each producer body (sequential order
      // preserves producer-before-consumer visibility for the verify shadow).
      emitBodyLock(2, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
      emitBodyLock(2, "mtDensePscdRelease(kDenseOwnerReadyStoreOffsets[%d], kDenseOwnerReadyStoreOffsets[%d]);\n", mtaskId, mtaskId + 1);
      emitBodyLock(2, "#endif\n");
    }
    if (pushReadyHint) {
      // Push-ready hint serial notify: keeps pending counters and the notify
      // CSR exercised exactly once per body on the fallback lane (fixed
      // ascending order is a valid topological order of the census), so the
      // accounting stays identical to the parallel path.
      emitBodyLock(2, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
      emitBodyLock(2, "mtDensePushReadyHintNotify(%d);\n", mtaskId);
      emitBodyLock(2, "#endif\n");
    }
  }
  emitBodyLock(1, "}\n");
  emitBodyLock(1, "if (mtProfileDynamicTraceFile != nullptr) dumpMtProfileDynamicTraceCycle();\n");
  emitBodyLock(1, "cycles ++;\n");
  emitBodyLock(1, "if (unlikely(mtProfileEnabled)) mtProfileTotalStepNs += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtProfileStepBegin).count();\n");
  if (denseBreakdownProfileCodegen) {
    if (denseBreakdownWindowCodegen) {
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownProfileInCycle)) {\n");
    } else {
      emitBodyLock(1, "if (unlikely(mtDenseBreakdownProfileEnabled)) {\n");
    }
    emitBodyLock(2, "uint64_t mtDenseBreakdownStepNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDenseBreakdownStepBegin).count();\n");
    emitBodyLock(2, "if (unlikely(UINT64_MAX - mtDenseBreakdownTotalStepNs < mtDenseBreakdownStepNs)) { fprintf(stderr, \"[mt-dense-breakdown] total-step counter overflow\\n\"); abort(); }\n");
    emitBodyLock(2, "mtDenseBreakdownTotalStepNs += mtDenseBreakdownStepNs;\n");
    emitBodyLock(1, "}\n");
  }
  emitBodyLock(0, "}\n");
  mtActivationEventTraceSuppressed = savedActivationEventTraceSuppressed;
}
