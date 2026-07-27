/*
  sort superNodes in topological order
*/

#include "common.h"
#include <stack>
#include <map>

void graph::topoSort() {
  std::map<SuperNode*, int>times;
  std::set<SuperNode*, SuperNodeStableLess> s;
  for (SuperNode* node : supersrc) {
    if (node->depPrev.size() == 0) s.insert(node);
  }
  /* next.size() == 0, place the registers at the end to benefit mergeRegisters */
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
  /* insert registers */
  sortedSuper.insert(sortedSuper.end(), potentialRegs.begin(), potentialRegs.end());
  /* order sortedSuper */
  orderAllNodes();
}

