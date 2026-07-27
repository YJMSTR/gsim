
#include <cstdio>
#include <queue>
#include <vector>
#include <stack>
#include <set>
#include "common.h"
#define MAX_NODES_PER_SUPER 7000
#define MAX_SUBLINGS 30

void getENodeRelyNodes(ENode* enode, std::set<Node*>& allNodes);

bool anyPath(SuperNode* src, SuperNode* dst) {
  std::stack<SuperNode*> s;
  std::set<SuperNode*> visited;
  visited.insert(src);
  for (SuperNode* prev : src->depPrev) {
    s.push(prev);
    visited.insert(prev);
  }
  while (!s.empty()) {
    SuperNode* top = s.top();
    s.pop();
    if (top == dst) return true;
    for (SuperNode* prev : top->depPrev) {
      if (visited.find(prev) == visited.end()) {
        visited.insert(prev);
        s.push(prev);
      }
    }
  }
  return false;
}
#if 0
void graph::mergeWhenNodes() {
/* each superNode contain one node */
  std::map<Node*, SuperNode*> whenMap;
  for (SuperNode* super : sortedSuper) {
    if (super->superType != SUPER_VALID) continue;
    Assert(super->member.size() <= 1, "invalid super size %ld", super->member.size());
    for (Node* member : super->member) {
      if (member->isArray() || member->assignTree.size() != 1) continue;
      if (member->assignTree[0]->getRoot()->opType != OP_WHEN) continue;
      Node* whenNode = member->assignTree[0]->getRoot()->getChild(0)->getNode();
      if (!whenNode) continue; // TODO: expr can also be optimized
      /* exists and no loop */
      if (whenMap.find(whenNode) == whenMap.end()) {
        whenMap[whenNode] = super;
        continue;
      }
      SuperNode* whenSuper = whenMap[whenNode];
      if (anyPath(super, whenSuper)) continue;
      /* merge member to whenSuper */
      whenSuper->member.push_back(member);
      member->super = whenSuper;
      super->member.clear();
      /* update super connection */
      whenSuper->addNext(super->next);
      for (SuperNode* next : super->next) {
        next->erasePrev(super);
        next->addPrev(whenSuper);
      }
      whenSuper->addPrev(super->prev);
      for (SuperNode* prev : super->prev) {
        prev->eraseNext(super);
        prev->addNext(whenSuper);
      }
    }
  }

  size_t prevSuper = sortedSuper.size();
  removeEmptySuper();
  reconnectSuper();
  detectSortedSuperLoop();
  printf("[mergeNodes-when] remove %ld superNodes (%ld -> %ld)\n", prevSuper - sortedSuper.size(), prevSuper, sortedSuper.size());
}
#else
void graph::mergeWhenNodes() {
  // v430 diagnostic (env value = dump filepath): canonical pre-merge graph record per
  // super: id, member names, sorted depPrev/depNext endpoint ids and names. Lets two
  // runs verify both id stability and whole-graph equality at this point.
  if (const char* dumpPath = std::getenv("GSIM_DEBUG_DUMP_SUPER_IDS")) {
    FILE* dumpFp = std::fopen(dumpPath, "w");
    if (dumpFp) {
      std::vector<SuperNode*> sortedById;
      for (SuperNode* super : sortedSuper) if (super->superType == SUPER_VALID) sortedById.push_back(super);
      std::sort(sortedById.begin(), sortedById.end(), [](const SuperNode* a, const SuperNode* b) { return a->id < b->id; });
      auto dumpEnds = [dumpFp](const std::set<SuperNode*>& ends) {
        std::vector<std::pair<int, std::string>> keyed;
        for (SuperNode* e : ends) keyed.push_back({e->id, e->member.empty() ? std::string("-") : e->member[0]->name});
        std::sort(keyed.begin(), keyed.end());
        for (const auto& k : keyed) std::fprintf(dumpFp, "%d:%s,", k.first, k.second.c_str());
      };
      for (SuperNode* super : sortedById) {
        std::fprintf(dumpFp, "%d|%zu|", super->id, super->member.size());
        for (Node* member : super->member) std::fprintf(dumpFp, "%s,", member->name.c_str());
        std::fprintf(dumpFp, "|");
        dumpEnds(super->depPrev);
        std::fprintf(dumpFp, "|");
        dumpEnds(super->depNext);
        std::fprintf(dumpFp, "\n");
      }
      std::fclose(dumpFp);
    }
  }
  // v432: GSIM_STABLE_ORDER=1 selects the deterministic stable-order path (reproducible
  // generation); default keeps the original pointer-order behavior.
  const bool stableOrder = [](){ const char* e = std::getenv("GSIM_STABLE_ORDER"); return e && e[0] && e[0] != '0'; }();
  std::queue<SuperNode*> s;
  std::queue<SuperNode*> cond;
  std::map<SuperNode*, std::set<SuperNode*>> allCond;
  std::map<SuperNode*, SuperNode*> node2Cond;
  std::map<SuperNode*, int>times;
  std::set<SuperNode*, SuperNodeStableLess> condWaitStable;
  std::map<SuperNode*, std::vector<SuperNode*>, SuperNodeStableLess> whenMapStable;
  std::set<SuperNode*> condWaitOrig;
  std::map<SuperNode*, std::vector<SuperNode*>> whenMapOrig;
  /* generator all cond nodes */
  for (SuperNode* super : sortedSuper) {
    times[super] = 0;
    if (super->superType != SUPER_VALID) continue;
    Assert(super->member.size() <= 1, "invalid super size %ld", super->member.size());
    for (Node* member : super->member) {
      if (member->isArray() || member->assignTree.size() != 1) continue;
      if (member->assignTree[0]->getRoot()->opType != OP_WHEN) continue;
      Node* whenNode = member->assignTree[0]->getRoot()->getChild(0)->getNode();
      if (!whenNode) continue; // TODO: expr can also be optimized
      if (allCond.find(whenNode->super) == allCond.end()) {
        allCond[whenNode->super] = std::set<SuperNode*>();
      }
      allCond[whenNode->super].insert(super);
      node2Cond[super] = whenNode->super;
    }
  }

  auto addCond = [&](SuperNode* super) {
    int num = 0;
    for (SuperNode* s2 : allCond[super]) {
      if (times[s2] + 1 == (int)s2->depPrev.size()) num ++;
    }
    if (num >= 2) cond.push(super);
    else if (stableOrder) condWaitStable.insert(super);
    else condWaitOrig.insert(super);
  };

  auto cond2Queue = [&](SuperNode* super) {
    if (stableOrder) {
      if (condWaitStable.find(super) != condWaitStable.end()) {
        condWaitStable.erase(super);
        cond.push(super);
      }
    } else {
      if (condWaitOrig.find(super) != condWaitOrig.end()) {
        condWaitOrig.erase(super);
        cond.push(super);
      }
    }
  };
  auto condWaitEmpty = [&]() { return stableOrder ? condWaitStable.empty() : condWaitOrig.empty(); };
  auto condWaitTop = [&]() { return stableOrder ? *condWaitStable.begin() : *condWaitOrig.begin(); };

  std::vector<SuperNode*> initialRoots;
  for (SuperNode* super : sortedSuper) {
    if (super->depPrev.size() == 0) {
      if (allCond.find(super) != allCond.end()) addCond(super);
      else initialRoots.push_back(super);
    }
  }
  if (stableOrder) std::sort(initialRoots.begin(), initialRoots.end(), SuperNodeStableLess());
  for (SuperNode* root : initialRoots) s.push(root);
  while (!s.empty()) {
    SuperNode* top = s.front();
    s.pop();
    for (SuperNode* next : (stableOrder ? stableOrdered(top->depNext) : std::vector<SuperNode*>(top->depNext.begin(), top->depNext.end()))) {
      times[next] ++;
      if (times[next] + 1 == (int)next->depPrev.size()) {
        if (node2Cond.find(next) != node2Cond.end()) {
          cond2Queue(node2Cond[next]);
        }
      }
      if (times[next] == (int)next->depPrev.size()) {
        if (allCond.find(next) != allCond.end()) addCond(next);
        else s.push(next);
      }
    }
    while (s.empty() && (!cond.empty() || !condWaitEmpty())) {
      if (cond.empty()) {
        cond2Queue(condWaitTop());
      }
      SuperNode* mergeCond = cond.front();
      cond.pop();
      std::vector<SuperNode*> mergeSuper;
      for (SuperNode* next : (stableOrder ? stableOrdered(mergeCond->depNext) : std::vector<SuperNode*>(mergeCond->depNext.begin(), mergeCond->depNext.end()))) {
        times[next] ++;
        if (times[next] == (int)next->depPrev.size()) {
          if (allCond[mergeCond].find(next) != allCond[mergeCond].end()) mergeSuper.push_back(next);
          s.push(next);
        }
      }
      if (mergeSuper.size() > globalConfig.MergeWhenSize) {
        if (stableOrder) whenMapStable[mergeCond] = mergeSuper;
        else whenMapOrig[mergeCond] = mergeSuper;
      }
    }
  }
  auto applyWhenMap = [](auto& whenMap) {
    for (auto iter : whenMap) {
      std::vector<SuperNode*> allSuper = iter.second;
      SuperNode* mergeSuper = allSuper[0];
      for (size_t i = 1; i < allSuper.size(); i ++) {
        for (Node* member : allSuper[i]->member) {
          mergeSuper->add_member(member);
          member->super = mergeSuper;
        }
        allSuper[i]->member.clear();
      }
    }
  };
  if (stableOrder) applyWhenMap(whenMapStable);
  else applyWhenMap(whenMapOrig);
  size_t prevSuper = sortedSuper.size();
  removeEmptySuper();
  reconnectSuper();
  detectSortedSuperLoop();
  printf("[mergeNodes-when] remove %ld superNodes (%ld -> %ld)\n", prevSuper - sortedSuper.size(), prevSuper, sortedSuper.size());
  when2mux();
}
#endif


void graph::when2mux() {
  for (SuperNode* super : sortedSuper) {
    if (super->superType != SUPER_VALID) continue;
    std::map<Node*, int> whenTimes;
    std::stack<ENode*>s;
    std::stack<ENode*> allENodes; // parents lie beneath
    for (Node* member : super->member) {
      for (ExpTree* tree : member->assignTree) {
        if (tree->getRoot()->opType != OP_WHEN) continue;
        s.push(tree->getRoot());
      }
    }
    // push all when nodes to allENodes
    while (!s.empty()) {
      ENode* top = s.top();
      s.pop();
      if (top->opType != OP_WHEN) continue;
      if (whenTimes.find(top->getChild(0)->getNode()) == whenTimes.end()) whenTimes[top->getChild(0)->getNode()] = 0;
      whenTimes[top->getChild(0)->getNode()]++;
      allENodes.push(top);
      for (ENode* child : top->child) {
        if (child) s.push(child);
      }
    }
    while (!allENodes.empty()) {
      ENode* top = allENodes.top();
      if (whenTimes[top->getChild(0)->getNode()] <= globalConfig.When2muxBound) {
        if (top->getChild(1) && (top->getChild(1)->opType != OP_WHEN && top->getChild(1)->opType != OP_INVALID)
          && top->getChild(2) && (top->getChild(2)->opType != OP_WHEN && top->getChild(2)->opType != OP_INVALID)) 
        top->opType = OP_MUX;
      }
      allENodes.pop();
    }
  }
}

void graph::mergeResetAll() {
  std::map<Node*, std::pair<SuperNode*, SuperNode*>> resetSuper; // node as uint / async reset
  for (Node* reg : regsrc) {
    if (reg->reset != UINTRESET && reg->reset != ASYRESET) continue;
    std::set<Node*> prev;
    if (reg->resetTree->getRoot()->opType == OP_RESET) {
      getENodeRelyNodes(reg->resetTree->getRoot()->getChild(0), prev);
    } else {
      Panic();
      reg->resetTree->getRelyNodes(prev);
    }
    if (prev.size() != 1) reg->display();
    Assert(prev.size() == 1, "multiple prevReset %s", reg->name.c_str());
    Node* prevNode = *prev.begin();
    SuperNode* uintSuper, *asyncSuper;
    if (resetSuper.find(prevNode) == resetSuper.end()) {
      uintSuper = new SuperNode();
      uintSuper->superType = SUPER_UINT_RESET;
      asyncSuper = new SuperNode();
      asyncSuper->superType = SUPER_ASYNC_RESET;
      uintSuper->resetNode = asyncSuper->resetNode = prevNode;
      resetSuper[prevNode] = std::make_pair(uintSuper, asyncSuper);
    } else {
      std::tie(uintSuper, asyncSuper) = resetSuper[prevNode];
    }
    Node* resetReg = reg->dup(NODE_REG_RESET, reg->name);
    resetReg->regNext = reg;
    resetReg->assignTree.push_back(new ExpTree(reg->resetTree->getRoot(), resetReg));
    if (reg->reset == ASYRESET) asyncSuper->add_member(resetReg);
    else uintSuper->add_member(resetReg);

    if (reg->getDst()->status == VALID_NODE) {
      Node* resetRegDst = reg->dup(NODE_REG_RESET, reg->getDst()->name);
      resetRegDst->regNext = reg;
      resetRegDst->assignTree.push_back(new ExpTree(reg->resetTree->getRoot(), resetRegDst));
      if (reg->reset == ASYRESET) asyncSuper->add_member(resetRegDst);
      else uintSuper->add_member(resetRegDst);
    }
  }
  for (auto iter : resetSuper) {
    SuperNode* uintSuper = iter.second.first;
    SuperNode* asyncSuper = iter.second.second;
    if (uintSuper->member.size() != 0) {
      allReset.push_back(uintSuper);
      iter.first->setUIntReset();
    }
    if (asyncSuper->member.size() != 0) {
      allReset.push_back(asyncSuper);
      iter.first->setAsyncReset();
      iter.first->super->superType = SUPER_ASYNC_RESET;
      iter.first->super->resetNode = asyncSuper->resetNode;
    }
  }
}
/*
  merge nodes with out-degree=1 to their successors
*/
void graph::mergeOut1() {
  for (int i = sortedSuper.size() - 1; i >= 0; i --) {
    SuperNode* super = sortedSuper[i];
    /* do not merge superNodes that contains reg src */
    if (super->superType != SUPER_VALID) continue;
    if (super->next.size() == 1) {
      SuperNode* nextSuper = *(super->next.begin());
      if (nextSuper->superType != SUPER_VALID) continue;
      if (nextSuper->member.size() > MAX_NODES_PER_SUPER) continue;
      bool canMerge = true;
      for (SuperNode* depNext : super->depNext) {
        if (depNext->order < nextSuper->order) canMerge = false;
      }
      if (!canMerge) continue;
      for (Node* member : super->member) member->super = nextSuper;
      /* move members in super to next super*/
      for (Node* member : super->member) {
        member->super = nextSuper;
      }
      /* update member */
      nextSuper->member.insert(nextSuper->member.begin(), super->member.begin(), super->member.end());
      /* update connection */
      nextSuper->erasePrev(super);
      nextSuper->addPrev(super->prev);
      for (SuperNode* prev : super->depPrev) {
        if (super->prev.find(prev) != super->prev.end()) {
          prev->eraseNext(super);
          prev->addNext(nextSuper);
          nextSuper->addPrev(prev);
        } else { // only in depPrev
          prev->eraseDepNext(super);
          prev->addDepNext(nextSuper);
          nextSuper->addDepPrev(prev);
        }
      }
      for (SuperNode* next : super->depNext) {
        if (super->next.find(next) == super->next.end()) { // only in depNext
          next->eraseDepPrev(super);
          next->addDepPrev(nextSuper);
          nextSuper->addDepNext(next);
        }
      }
      super->member.clear();
      super->clear_relation();
    }
  }
  removeEmptySuper();
}

/*
  merge nodes with in-degree=1 to their preceding nodes
*/
void graph::mergeIn1() {
  for (size_t i = 0; i < sortedSuper.size(); i ++) {
    SuperNode* super = sortedSuper[i];
    if (super->superType != SUPER_VALID) continue;
    if (super->prev.size() == 1) {
      SuperNode* prevSuper = *(super->prev.begin());
      if (prevSuper->superType != SUPER_VALID) continue;
      if (prevSuper->member.size() > MAX_NODES_PER_SUPER) continue;
      bool canMerge = true;
      for (SuperNode* depPrev : super->depPrev) {
        if (depPrev->order > prevSuper->order) canMerge = false;
      }
      if (!canMerge) continue;
      /* move members in super to prev super */
      for (Node* member : super->member) member->super = prevSuper;
      Assert(prevSuper->member.size() != 0, "empty prevSuper %d", prevSuper->id);
      prevSuper->member.insert(prevSuper->member.end(), super->member.begin(), super->member.end());
      /* update connection */
      prevSuper->eraseNext(super);
      prevSuper->addNext(super->next);
      for (SuperNode* next : super->depNext) {
        if (super->next.find(next) != super->next.end()) {
          next->erasePrev(super);
          next->addPrev(prevSuper);
          prevSuper->addNext(next);
        } else { // only in depNext
          next->eraseDepPrev(super);
          next->addDepPrev(prevSuper);
          prevSuper->addDepNext(next);
        }
      }
      for (SuperNode* prev : super->depPrev) {
        if (super->prev.find(prev) == super->prev.end()) { // only in depPrev
          prev->eraseDepNext(super);
          prev->addDepNext(prevSuper);
          prevSuper->addDepPrev(prev);
        }
      }
      super->member.clear();
    }
  }

  removeEmptySuper();
  // reconnectSuper();
}

uint64_t prevHash(SuperNode* super) {
  uint64_t ret = super->prev.size()*7;
  for (SuperNode* prev : super->prev) {
    ret += (uint64_t)prev;
  }
  return ret;
}

bool prevEq(SuperNode* super1, SuperNode* super2) {
  return super1->prev == super2->prev && super1->depPrev == super2->depPrev;
}

void graph::mergeSublings() {
  std::map<uint64_t, std::vector<SuperNode*>> prevSuper;
  for (SuperNode* super : sortedSuper) {
    if (super->prev.size() == 0) continue;
    if (super->superType != SUPER_VALID) continue;
    uint64_t id = prevHash(super);
    if (prevSuper.find(id) == prevSuper.end()) prevSuper[id] = std::vector<SuperNode*>();
    prevSuper[id].push_back(super);
  }

  for (auto iter : prevSuper) {
    std::set<SuperNode*> uniquePrev;
    for (SuperNode* super : iter.second) {
      bool find = false;
      for (SuperNode* checkSuper : uniquePrev) {
        if (prevEq(checkSuper, super)) { // merge or replace
          find = true;
          if (checkSuper->member.size() < MAX_SUBLINGS) {
            for (Node* member : super->member) member->super = checkSuper;
            checkSuper->member.insert(checkSuper->member.end(), super->member.begin(), super->member.end());
            super->member.clear();
          } else {
            uniquePrev.erase(checkSuper);
            uniquePrev.insert(super);
          }
          break;
        }
      }
      if (!find) uniquePrev.insert(super);
    }
  }
  removeEmptySuper();
  reconnectSuper();
}
