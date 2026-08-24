/*
  sort superNodes in topological order
*/

#include "common.h"
#include "phaseTimer.h"
#include <stack>
#include <map>

void graph::topoSort() {
  // GSIM_STABLE_ORDER=1 selects the deterministic stable-order frontier
  // (reproducible generation); default keeps the original pointer-order traversal
  // (current schedule-quality baseline) while the stable path is validated further.
  const bool stableOrder = [](){ const char* e = std::getenv("GSIM_STABLE_ORDER"); return e && e[0] && e[0] != '0'; }();
  const bool seedReplay = mtSeedReplayActive();
  const bool seedWrite = mtSeedWriteActive();
  const bool seed2Write = mtSeed2WriteActive();
  const bool seed2Replay = mtSeed2ReplayActive();
  uint64_t seedInputHash = 0;
  {
    PhaseTimer tSeedPrep("TopoSort.seedPrep");
    if (seedReplay || seedWrite || seed2Write || seed2Replay) seedInputHash = canonInputHash();
    if (seedReplay) mtSeedVerifyInputHash(seedInputHash);
    if (seedWrite) mtSeedAssertCompatible();
    if (seed2Write) { mtSeed2AssertCompatible(); mtSeed2SetInputHash(seedInputHash); }
    if (seed2Replay) mtSeed2VerifyInputHash(seedInputHash);
  }
  std::map<SuperNode*, int>times;
  if (seedReplay) {
    std::set<SuperNode*, SeedRankLess> s;
    for (SuperNode* node : supersrc) {
      if (node->depPrev.size() == 0) s.insert(node);
    }
    std::vector<SuperNode*> potentialRegs;
    std::set<SuperNode*> visited;
    while(!s.empty()) {
      SuperNode* top = *s.begin();
      s.erase(s.begin());
      Assert(visited.find(top) == visited.end(), "superNode %d is already visited\n", top->id);
      visited.insert(top);
      sortedSuper.push_back(top);
      for (SuperNode* next : seedRankOrdered(top->depNext)) {
        if (times.find(next) == times.end()) times[next] = 0;
        times[next] ++;
        if (times[next] == (int)next->depPrev.size()) {
          s.insert(next);
        }
      }
    }
    sortedSuper.insert(sortedSuper.end(), potentialRegs.begin(), potentialRegs.end());
    orderAllNodes();
    return;
  }
  if (stableOrder) {
    std::set<SuperNode*, SuperNodeStableLess> s;
    for (SuperNode* node : supersrc) {
      if (node->depPrev.size() == 0) s.insert(node);
    }
    std::vector<SuperNode*> potentialRegs;
    std::set<SuperNode*> visited;
    while(!s.empty()) {
      SuperNode* top = *s.begin();
      s.erase(s.begin());
      Assert(visited.find(top) == visited.end(), "superNode %d is already visited\n", top->id);
      visited.insert(top);
      sortedSuper.push_back(top);
      for (SuperNode* next : stableOrdered(top->depNext)) {
        if (times.find(next) == times.end()) times[next] = 0;
        times[next] ++;
        if (times[next] == (int)next->depPrev.size()) {
          s.insert(next);
        }
      }
    }
    sortedSuper.insert(sortedSuper.end(), potentialRegs.begin(), potentialRegs.end());
    orderAllNodes();
    if (seedWrite) mtSeedWrite(std::getenv("GSIM_SCHEDULE_SEED_WRITE"), sortedSuper, seedInputHash, "wip/dense-b1-lookahead");
    if (seed2Write) mtSeed2RecordPoint("pass.topoSort", sortedSuper, canonInputHash());
    if (seed2Replay) { mtSeed2ApplyPoint("pass.topoSort", sortedSuper, canonInputHash()); orderAllNodes(); }
    return;
  }
  PhaseTimer tDfs("TopoSort.dfs");
  std::stack<SuperNode*> s;
  for (SuperNode* node : supersrc) {
    if (node->depPrev.size() == 0) s.push(node);
  }
  /* next.size() == 0, place the registers at the end to benefit mergeRegisters */
  std::vector<SuperNode*> potentialRegs;
  /* times/visited moved into SuperNode scratch (topoMark = epoch<<32|count,
 * count 0 = visited): the counter semantics (1 on first touch, ==
 * depPrev.size() pushes once) and the push order are unchanged, so sortedSuper
 * is the same sequence as the old std::map/std::set implementation. */
  static uint32_t dfsEpochCounter = 0;
  const uint64_t dfsEpoch = uint64_t(++ dfsEpochCounter) << 32;
  while(!s.empty()) {
    SuperNode* top = s.top();
    s.pop();
    Assert(top->topoMark != dfsEpoch, "superNode %d is already visited\n", top->id);
    top->topoMark = dfsEpoch;
    sortedSuper.push_back(top);
#ifdef ORDERED_TOPO_SORT
    std::vector<SuperNode*> sortedNext;
    sortedNext.insert(sortedNext.end(), top->depNext.begin(), top->depNext.end());
    std::sort(sortedNext.begin(), sortedNext.end(), [](SuperNode* a, SuperNode* b) {return a->id < b->id;});
    for (SuperNode* next : sortedNext) {
#else
    for (SuperNode* next : top->depNext) {
#endif
      uint64_t mark = next->topoMark;
      uint32_t count = (mark >> 32) == (dfsEpoch >> 32) ? uint32_t(mark) : 0;
      count ++;
      next->topoMark = dfsEpoch | count;
      if (count == next->depPrev.size()) {
        s.push(next);
      }
    }
  }
  /* insert registers */
  sortedSuper.insert(sortedSuper.end(), potentialRegs.begin(), potentialRegs.end());
  /* order sortedSuper */
  PhaseTimer tOrder("TopoSort.orderAllNodes");
  orderAllNodes();
  if (seedWrite) mtSeedWrite(std::getenv("GSIM_SCHEDULE_SEED_WRITE"), sortedSuper, seedInputHash, "wip/dense-b1-lookahead");
  if (seed2Write) mtSeed2RecordPoint("pass.topoSort", sortedSuper, canonInputHash());
  if (seed2Replay) { mtSeed2ApplyPoint("pass.topoSort", sortedSuper, canonInputHash()); orderAllNodes(); }
}

