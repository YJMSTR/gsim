/*
  sort superNodes in topological order
*/

#include "common.h"
#include <stack>
#include <map>

void graph::topoSort() {
  // v432: GSIM_STABLE_ORDER=1 selects the deterministic stable-order frontier
  // (reproducible generation); default keeps the original pointer-order traversal
  // (current schedule-quality baseline) while the stable path is validated further.
  const bool stableOrder = [](){ const char* e = std::getenv("GSIM_STABLE_ORDER"); return e && e[0] && e[0] != '0'; }();
  const bool seedReplay = mtSeedReplayActive();
  const bool seedWrite = mtSeedWriteActive();
  const bool seed2Write = mtSeed2WriteActive();
  uint64_t seedInputHash = 0;
  if (seedReplay || seedWrite || seed2Write) seedInputHash = canonInputHash();
  if (seedReplay) mtSeedVerifyInputHash(seedInputHash);
  if (seedWrite) mtSeedAssertCompatible();
  if (seed2Write) { mtSeed2AssertCompatible(); mtSeed2SetInputHash(seedInputHash); }
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
    return;
  }
  std::stack<SuperNode*> s;
  for (SuperNode* node : supersrc) {
    if (node->depPrev.size() == 0) s.push(node);
  }
  /* next.size() == 0, place the registers at the end to benefit mergeRegisters */
  std::vector<SuperNode*> potentialRegs;
  std::set<SuperNode*> visited;
  while(!s.empty()) {
    SuperNode* top = s.top();
    s.pop();
    Assert(visited.find(top) == visited.end(), "superNode %d is already visited\n", top->id);
    visited.insert(top);
    sortedSuper.push_back(top);
#ifdef ORDERED_TOPO_SORT
    std::vector<SuperNode*> sortedNext;
    sortedNext.insert(sortedNext.end(), top->depNext.begin(), top->depNext.end());
    std::sort(sortedNext.begin(), sortedNext.end(), [](SuperNode* a, SuperNode* b) {return a->id < b->id;});
    for (SuperNode* next : sortedNext) {
#else
    for (SuperNode* next : top->depNext) {
#endif
      if (times.find(next) == times.end()) times[next] = 0;
      times[next] ++;
      if (times[next] == (int)next->depPrev.size()) {
        s.push(next);
      }
    }
  }
  /* insert registers */
  sortedSuper.insert(sortedSuper.end(), potentialRegs.begin(), potentialRegs.end());
  /* order sortedSuper */
  orderAllNodes();
  if (seedWrite) mtSeedWrite(std::getenv("GSIM_SCHEDULE_SEED_WRITE"), sortedSuper, seedInputHash, "wip/dense-b1-lookahead");
  if (seed2Write) mtSeed2RecordPoint("pass.topoSort", sortedSuper, canonInputHash());
}

