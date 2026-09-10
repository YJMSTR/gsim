// cppEmitterDenseExec.cpp - split from cppEmitter.cpp (pure code motion):
// genDenseExecutor - moved WHOLE (no lambda extraction): activity provenance,
// owner-ready/lookahead tables, per-MTask dispatch bodies, the dense worker pool
// entry points and stepDense().
#include "cppEmitterImpl.h"

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
        fprintf(header, "struct MtDenseDispatchEntry { void (S%s::*fn)(); uint32_t waitBegin; uint32_t waitEnd; uint32_t storeBegin; uint32_t storeEnd; uint32_t localBegin; uint32_t localEnd; };\n",
                name.c_str());
        if (denseDuty) {
          fprintf(header, "void stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtDutyLane);\n");
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
  {
  EmitPhaseTimer denseBodyTimer("Final.denseMTaskBodies");
  // Each stepDenseMTaskN body is an independent emission unit (reads frozen
  // schedule + graph; per-super emission context flags are thread-local and
  // saved/restored around genSuperEval exactly as in the sequential loop).
  // Unit u renders denseMTaskEmissionOrder[u]; assembly replays buffers in
  // emission order, so output is byte-identical.
  emitUnitsParallel(denseMTaskEmissionOrder.size(), [this, &denseSchedule, &denseMTaskEmissionOrder, &ownerReadyLayout, denseBreakdownWindowLaBodyCodegen](size_t unit) {
    int mtaskId = denseMTaskEmissionOrder[unit];

    const MtDenseMTask& mtask = denseSchedule.mtasks[mtaskId];
    emitFuncDecl(0, "void S%s::stepDenseMTask%d() {\n", name.c_str(), mtaskId);
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
    emitBodyLock(0, "}\n");
  });
  }

  auto emitFixedDenseThreadWorker = [&](const char* funcName) {
    emitFuncDecl(0, "void S%s::%s(int threadId) {\n", name.c_str(), funcName);
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
            for (int slot : ownerReadyLayout.waitSlotsByMTask[(size_t)mtaskId]) {
              emitBodyLock(5, "mtDenseInlineReady &= (mtDenseOwnerReadyTokens[%d].ready.load(std::memory_order_acquire) == target);\n", slot);
            }
            if (denseDuty) {
              emitBodyLock(5, "if (!mtDenseInlineReady) { stepDenseLookaheadTail(kDenseDispatchTableW%d, kDenseDispatchTableW%d + %d, %uu, target, %du); return; }\n",
                           t, t, denseDispatchWorkerCounts[(size_t)t], static_cast<unsigned>(tablePosition), t);
            } else {
              emitBodyLock(5, "if (!mtDenseInlineReady) { stepDenseLookaheadTail(kDenseDispatchTableW%d, kDenseDispatchTableW%d + %d, %uu, target); return; }\n",
                           t, t, denseDispatchWorkerCounts[(size_t)t], static_cast<unsigned>(tablePosition));
            }
            emitBodyLock(5, "stepDenseMTask%d();\n", mtaskId);
            for (int slot : ownerReadyLayout.storeSlotsByMTask[(size_t)mtaskId]) {
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
            emitBodyLock(4, "mtWorkerPoolPause(); if (++ct > 256) { ct = 0; std::this_thread::yield(); } }\n");
            emitBodyLock(3, "}\n");
          }
        } else {
          emitBodyLock(3, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\n");
          for (int slot : ownerReadyLayout.waitSlotsByMTask[(size_t)mtaskId]) {
            if (denseBreakdownProfileCodegen) {
              emitBodyLock(3, "{ const uint8_t target = evenCycle ? uint8_t{1} : uint8_t{0};\n");
              if (denseBreakdownWindowCodegen) {
                emitBodyLock(3, "  if (unlikely(mtDenseBreakdownProfile && !(mtDenseBreakdownWindow && mtDenseBreakdownWindowFinishOnlyMode))) {\n");
              } else {
                emitBodyLock(3, "  if (unlikely(mtDenseBreakdownProfile)) {\n");
              }
              emitBodyLock(4, "bool mtDenseBreakdownBlocked = false;\n");
              emitBodyLock(4, "std::chrono::steady_clock::time_point mtDenseBreakdownBlockedBegin;\n");
              emitBodyLock(4, "unsigned ct = 0;\n");
              emitBodyLock(4, "while (mtDenseOwnerReadyTokens[%d].ready.load(std::memory_order_acquire) != target) {\n", slot);
              emitBodyLock(5, "if (!mtDenseBreakdownBlocked) { mtDenseBreakdownBlocked = true; mtDenseBreakdownBlockedBegin = std::chrono::steady_clock::now(); }\n");
              emitBodyLock(5, "mtWorkerPoolPause(); if (++ct > 256) { ct = 0; std::this_thread::yield(); }\n");
              emitBodyLock(4, "}\n");
              emitBodyLock(4, "if (mtDenseBreakdownBlocked) {\n");
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
              emitBodyLock(4, "unsigned ct = 0;\n");
              emitBodyLock(4, "while (mtDenseOwnerReadyTokens[%d].ready.load(std::memory_order_acquire) != target) {\n", slot);
              emitBodyLock(5, "mtWorkerPoolPause(); if (++ct > 256) { ct = 0; std::this_thread::yield(); }\n");
              emitBodyLock(4, "}\n");
              emitBodyLock(3, "  }\n");
              emitBodyLock(3, "}\n");
            } else {
              emitBodyLock(3, "{ const uint8_t target = evenCycle ? uint8_t{1} : uint8_t{0};\n");
              emitBodyLock(3, "  unsigned ct = 0;\n");
              emitBodyLock(3, "  while (mtDenseOwnerReadyTokens[%d].ready.load(std::memory_order_acquire) != target) {\n", slot);
              emitBodyLock(4, "mtWorkerPoolPause(); if (++ct > 256) { ct = 0; std::this_thread::yield(); } }\n");
              emitBodyLock(3, "}\n");
            }

          }
          emitBodyLock(3, "#else\n");
          if (!skipDenseWait) {
            emitBodyLock(3, "{ const uint32_t target = evenCycle ? kDenseMTaskDepCount[%d] : 0u;\n", mtaskId);
            emitBodyLock(3, "  unsigned ct = 0;\n");
            emitBodyLock(3, "  while (mtDenseMTaskVertices[%d].depsDone.load(std::memory_order_acquire) != target) {\n", mtaskId);
            emitBodyLock(4, "mtWorkerPoolPause(); if (++ct > 256) { ct = 0; std::this_thread::yield(); } }\n");
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
            emitBodyLock(3, "if (evenCycle) {\n");
            emitBodyLock(4, "for (int j = kDenseOwnerReadyStoreOffsets[%d]; j < kDenseOwnerReadyStoreOffsets[%d]; j++) {\n", mtaskId, mtaskId + 1);
            if (denseBreakdownProfileCodegen) {
              emitBodyLock(5, "const int mtDenseBreakdownWindowReadySlot = kDenseOwnerReadyStoreList[j];\n");
              emitBodyLock(5, "if (unlikely(mtDenseBreakdownWindow && mtDenseBreakdownWindowCausalChainMode)) { const int mtDenseBreakdownWindowLogicalToken = kDenseBreakdownWindowCausalLogicalTokenByReadySlot[mtDenseBreakdownWindowReadySlot]; if (unlikely(mtDenseBreakdownWindowLogicalToken < 0 || mtDenseBreakdownWindowLogicalToken >= kDenseBreakdownWindowCausalTokenCount || kDenseBreakdownWindowCausalTokenReadySlot[mtDenseBreakdownWindowLogicalToken] != mtDenseBreakdownWindowReadySlot || kDenseBreakdownWindowCausalTokenProducerMTask[mtDenseBreakdownWindowLogicalToken] != %d || kDenseBreakdownWindowCausalTokenProducerOwner[mtDenseBreakdownWindowLogicalToken] != threadId)) { mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token provenance mismatch\\n\"); abort(); } MtDenseBreakdownWindowReadyToken &mtDenseBreakdownWindowTokenRelease = mtDenseBreakdownWindowReadyTokens[mtDenseBreakdownWindowSlot][mtDenseBreakdownWindowLogicalToken]; const std::chrono::steady_clock::time_point mtDenseBreakdownWindowReleaseBefore = std::chrono::steady_clock::now(); if (unlikely(mtDenseBreakdownWindowReleaseBefore < mtDenseBreakdownWindowEpoch || mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs != UINT64_MAX)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token release-before invalid\\n\"); abort(); } mtDenseBreakdownWindowTokenRelease.releaseBeforeOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowReleaseBefore - mtDenseBreakdownWindowEpoch).count(); mtDenseOwnerReadyTokens[mtDenseBreakdownWindowReadySlot].ready.store(uint8_t{1}, std::memory_order_release); const std::chrono::steady_clock::time_point mtDenseBreakdownWindowReleaseAfter = std::chrono::steady_clock::now(); if (unlikely(mtDenseBreakdownWindowReleaseAfter < mtDenseBreakdownWindowReleaseBefore)) { mtDenseBreakdownWindowCausalClockRegression = true; mtDenseBreakdownWindowOverflow.store(true, std::memory_order_relaxed); fprintf(stderr, \"[mt-dense-breakdown] causal token release-after clock regression\\n\"); abort(); } mtDenseBreakdownWindowTokenRelease.releaseAfterOffsetNs = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(mtDenseBreakdownWindowReleaseAfter - mtDenseBreakdownWindowEpoch).count(); } else { mtDenseOwnerReadyTokens[mtDenseBreakdownWindowReadySlot].ready.store(uint8_t{1}, std::memory_order_release); }\n", mtaskId);
            } else {
              emitBodyLock(5, "mtDenseOwnerReadyTokens[kDenseOwnerReadyStoreList[j]].ready.store(uint8_t{1}, std::memory_order_release);\n");
            }
            emitBodyLock(4, "}\n");
            emitBodyLock(3, "} else {\n");
            emitBodyLock(4, "for (int j = kDenseOwnerReadyStoreOffsets[%d]; j < kDenseOwnerReadyStoreOffsets[%d]; j++) {\n", mtaskId, mtaskId + 1);
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
    if (denseDuty) {
      emitFuncDecl(0, "#if defined(GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE) && GSIM_MT_DENSE_OWNER_READY_FLAGS_COMPILE\nvoid S%s::stepDenseLookaheadTail(const MtDenseDispatchEntry* mtDenseDispatchBegin, const MtDenseDispatchEntry* mtDenseDispatchEnd, uint32_t startHead, uint8_t target, uint32_t mtDutyLane) {\n", name.c_str());
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
    emitBodyLock(2, "for (uint32_t mtDenseDispatchWait = mtDenseDispatchEntry->waitBegin; mtDenseDispatchWait < mtDenseDispatchEntry->waitEnd; ++mtDenseDispatchWait) {\n");
    emitBodyLock(3, "mtDenseEntryReady &= (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) == target);\n");
    emitBodyLock(2, "}\n");
    // Tail-scan instrumentation (E3): zero-cost unless the stats compile macro is set.
    emitBodyLock(2, "#if defined(GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE) && GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE\n");
    emitBodyLock(2, "if (!mtDenseEntryReady) mtDenseLookaheadTailCalls.fetch_add(1, std::memory_order_relaxed);\n");
    emitBodyLock(2, "#endif\n");
    emitBodyLock(2, "if (mtDenseEntryReady) {\n");
    emitBodyLock(3, "(this->*mtDenseDispatchEntry->fn)();\n");
    emitBodyLock(3, "for (uint32_t mtDenseDispatchStore = mtDenseDispatchEntry->storeBegin; mtDenseDispatchStore < mtDenseDispatchEntry->storeEnd; ++mtDenseDispatchStore) {\n");
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
    emitBodyLock(3, "for (uint32_t mtDenseDispatchWait = mtDenseCandidate->waitBegin; mtDenseDispatchWait < mtDenseCandidate->waitEnd; ++mtDenseDispatchWait) {\n");
    emitBodyLock(4, "if (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) != target) { mtDenseCandidateReady = false; break; }\n");
    emitBodyLock(3, "}\n");
    emitBodyLock(3, "if (!mtDenseCandidateReady) continue;\n");
    emitBodyLock(3, "#if defined(GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE) && GSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE\n");
    emitBodyLock(3, "mtDenseLookaheadFound.fetch_add(1, std::memory_order_relaxed);\n");
    emitBodyLock(3, "#endif\n");
    emitBodyLock(3, "(this->*mtDenseCandidate->fn)();\n");
    emitBodyLock(3, "for (uint32_t mtDenseDispatchStore = mtDenseCandidate->storeBegin; mtDenseDispatchStore < mtDenseCandidate->storeEnd; ++mtDenseDispatchStore) {\n");
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
    emitBodyLock(2, "for (uint32_t mtDenseDispatchWait = mtDenseDispatchEntry->waitBegin; mtDenseDispatchWait < mtDenseDispatchEntry->waitEnd; ++mtDenseDispatchWait) {\n");
    emitBodyLock(3, "unsigned ct = 0;\n");
    emitBodyLock(3, "while (mtDenseOwnerReadyTokens[kDenseOwnerReadyWaitList[mtDenseDispatchWait]].ready.load(std::memory_order_acquire) != target) {\n");
    emitBodyLock(4, "mtWorkerPoolPause(); if (++ct > 256) { ct = 0; std::this_thread::yield(); } }\n");
    emitBodyLock(2, "}\n");
    if (denseDuty) emitBodyLock(2, "if (mtDutyEnabled) mtDutyLanes[mtDutyLane].blockNs += (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - mtDutyBlockBegin).count();\n");
    emitBodyLock(2, "(this->*mtDenseDispatchEntry->fn)();\n");
    emitBodyLock(2, "for (uint32_t mtDenseDispatchStore = mtDenseDispatchEntry->storeBegin; mtDenseDispatchStore < mtDenseDispatchEntry->storeEnd; ++mtDenseDispatchStore) {\n");
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
        emitBodyLock(0, "{nullptr, 0u, 0u, 0u, 0u, 0u, 0u}\n");
      } else {
        for (int m = 0; m < nMTasks; m++) {
          if (denseSchedule.mtaskThreadAssign[(size_t)m] != t) continue;
          emitBodyLock(0, "{&S%s::stepDenseMTask%d, %uu, %uu, %uu, %uu, %uu, %uu},\n",
                       name.c_str(), m,
                       denseDispatchWaitBegin[(size_t)m], denseDispatchWaitEnd[(size_t)m],
                       denseDispatchStoreBegin[(size_t)m], denseDispatchStoreEnd[(size_t)m],
                       denseLookaheadLocalBegin[(size_t)m], denseLookaheadLocalEnd[(size_t)m]);
        }
      }
      emitBodyLock(0, "};\n");
    }
    emitBodyLock(0, "#endif\n");
  }
  emitFixedDenseThreadWorker("stepDenseThreadWorker");
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
  if (denseBreakdownProfileCodegen) {
    emitBodyLock(2, "std::chrono::steady_clock::time_point mtDenseBreakdownPoolIdleBegin;\n");
    emitBodyLock(2, "std::chrono::steady_clock::time_point mtDenseBreakdownPoolDoneBegin;\n");
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
  for (int mtaskId = 0; mtaskId < nMTasks; mtaskId++) {
    emitBodyLock(2, "stepDenseMTask%d();\n", mtaskId);
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
