/*
  Remove deadNodes. A node is a deadNode if it has no successor or all successors are also deadNodes.
  must be called after topo-sort
*/

#include "common.h"
#include "phaseTimer.h"
#include <stack>

static std::set<Node*> nodesInUpdateTree;

void getENodeRelyNodes(ENode* enode, std::set<Node*>& allNodes) {
  std::stack<ENode*> s;
  s.push(enode);
  while (!s.empty()) {
    ENode* top = s.top();
    s.pop();
    Node* prevNode = top->getNode();
    if (prevNode) allNodes.insert(prevNode);
    for (size_t i = 0; i < top->getChildNum(); i ++) {
      if (top->getChild(i)) s.push(top->getChild(i));
    }
  }
}

void ExpTree::getRelyNodes(std::set<Node*>& allNodes) {
  getENodeRelyNodes(getRoot(), allNodes);
  for (ENode* child : getlval()->child) {
    getENodeRelyNodes(child, allNodes);
  }
}


void graph::removeDeadNodes() {
  static int passId = 0;
  const int curPass = passId ++;
  if (globalConfig.LogLevel > 1) {
    fprintf(stderr, "[RemoveDeadNodes] pass %d start\n", curPass);
  }
  /* Reachability marking uses a per-call epoch stamped into Node::reachEpoch
   * instead of a std::set<Node*>: same claim-once semantics, O(1) cache-friendly
   * membership. The visited *set* is the only output of the traversal (the mark
   * loop below reads membership, never order), so the traversal order change
   * from stack->vector is unobservable. */
  static uint32_t reachEpochCounter = 0;
  const uint32_t epoch = ++ reachEpochCounter;
  size_t totalNodes = 0, totalSuper = 0;
  {
    PhaseTimer tReach("RemoveDeadNodes.reach");
  std::vector<Node*> s;
  auto add = [&s, epoch](Node* node) {
    if (node->reachEpoch != epoch) {
      s.push_back(node);
      node->reachEpoch = epoch;
    }
  };
  for (Node* outNode : output) add(outNode);
  for (Node* special : specialNodes) add(special);
  for (SuperNode* super : sortedSuper) {
    for (Node* member : super->member) {
      if (member->type == NODE_EXT || member->type == NODE_EXT_IN || member->type == NODE_EXT_OUT) add(member);
    }
  }
  for (size_t si = 0; si < s.size(); si ++) {
    Node* top = s[si];
    for (Node* prev : top->prev) {
      add(prev);
    }
    if (top->type == NODE_REG_SRC) {
      add(top->getDst());
      std::set<Node*> resetNodes;
      if (top->resetTree) top->resetTree->getRelyNodes(resetNodes);
      for (Node* node : resetNodes) add(node);
    } else if (top->type == NODE_READER) {
      Node* memory = top->parent;
      for (Node* port : memory->member) {
        if (port->type == NODE_WRITER) add(port);
        if (port->type == NODE_READWRITER) add(port);
      }
    } else if (top->type == NODE_EXT_OUT) {
      Node* ext = top->parent;
      for (Node* member : ext->member) add(member);
    }
  }
  }
  {
    PhaseTimer tMark("RemoveDeadNodes.markDead");
    for (SuperNode* super : sortedSuper) {
    for (Node* node : super->member) {
      if (node->type == NODE_INP || node->type == NODE_OUT) continue;
      if (node->reachEpoch != epoch) {
        if (globalConfig.LogLevel > 1) {
          fprintf(stderr, "[RemoveDeadNodes] pass %d mark dead: %s type=%d super=%d line=%d\n",
                  curPass, node->name.c_str(), node->type, super->id, __LINE__);
        }
        node->status = DEAD_NODE;
      }
    }
  }
  }
  {
    PhaseTimer tCleanup("RemoveDeadNodes.cleanup");
  /* counters */
  totalNodes = countNodes();
  totalSuper = sortedSuper.size();

  removeNodes(DEAD_NODE);
  regsrc.erase(
    std::remove_if(regsrc.begin(), regsrc.end(), [](const Node* n){ return n->status == DEAD_NODE; }),
        regsrc.end()
  );

  for (size_t i = 0; i < memory.size(); i ++) {
    Node* mem = memory[i];
    mem->member.erase(
      std::remove_if(mem->member.begin(), mem->member.end(), [](const Node* n) {return n->status == DEAD_NODE; }),
      mem->member.end()
    );
    if (mem->member.size() == 0) {
      mem->status = DEAD_NODE;
      memory.erase(memory.begin() + i);
      i --;
    }
  }
  }
  {
    PhaseTimer tReconnect("RemoveDeadNodes.reconnectAll");
    reconnectAll();
  }
  size_t optimizedNodes = countNodes();
  size_t optimizedSuper = sortedSuper.size();

  printf("[removeDeadNodes] remove %ld deadNodes (%ld -> %ld)\n", totalNodes - optimizedNodes, totalNodes, optimizedNodes);
  printf("[removeDeadNodes] remove %ld superNodes (%ld -> %ld)\n", totalSuper - optimizedSuper, totalSuper, optimizedSuper);
  if (globalConfig.LogLevel > 1) {
    fprintf(stderr, "[RemoveDeadNodes] pass %d done (removed %ld nodes)\n", curPass, totalNodes - optimizedNodes);
  }

}
