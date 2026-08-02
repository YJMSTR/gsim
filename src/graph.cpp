#include "common.h"

// v441: node->id is assigned from a global allocation-sequence counter, which drifts
// with pointer-order-driven creation counts (e.g. temporary dup()s) and leaks into
// emitted names (oldName = name + "$old$" + id) and JSON metadata. Renumber by name
// once the graph is final: deterministic ids, byte-identical emission.
void graph::renumberNodesContentStable() {
  std::vector<Node*> all;
  std::set<Node*> seen;
  auto addNodes = [&](const std::vector<Node*>& members) {
    for (Node* m : members) if (seen.insert(m).second) all.push_back(m);
  };
  for (SuperNode* super : sortedSuper) addNodes(super->member);
  for (SuperNode* super : allReset) addNodes(super->member);
  addNodes(specialNodes);
  std::sort(all.begin(), all.end(), [](const Node* a, const Node* b) { return a->name < b->name; });
  int next = 1;
  for (Node* n : all) n->id = next ++;
  Node::setCounter(next);
}

void graph::reconnectAll() {
  for (SuperNode* super : sortedSuper) {
    for (Node* member : super->member) {
      member->clear_relation();
    }
  }

  for (SuperNode* super : sortedSuper) {
    for (Node* member : super->member) {
      member->updateConnect();
    }
  }
  connectDep();
  reconnectSuper();
}

void graph::connectDep() {
  for (SuperNode* super : sortedSuper) {
    for (Node* member : super->member) {
      if (member->type == NODE_REG_SRC) member->updateDep();
    }
  }
}

void graph::reconnectSuper() {
  for (SuperNode* super : sortedSuper) {
    super->clear_relation();
  }
  for (SuperNode* super : sortedSuper) {
    for (Node* member : super->member) member->constructSuperConnect();
  }
}

void graph::removeNodesNoConnect(NodeStatus status) {
  if (globalConfig.LogLevel > 1) {
    for (SuperNode* super : sortedSuper) {
      for (Node* n : super->member) {
        if (n->status == status) {
          fprintf(stderr, "[RemoveNodes] remove status=%d name=%s type=%d super=%d\n",
                  status, n->name.c_str(), n->type, super->id);
        }
      }
    }
  }
  for (SuperNode* super : sortedSuper) {
    super->member.erase(
      std::remove_if(super->member.begin(), super->member.end(), [status](const Node* n){ return n->status == status; }),
      super->member.end()
    );
  }
  removeEmptySuper();
}

size_t graph::countNodes() {
  size_t ret = 0;
  for (SuperNode* super : sortedSuper) ret += super->member.size();
  return ret;
}

void graph::removeNodes(NodeStatus status) {
  removeNodesNoConnect(status);
  removeEmptySuper();
  reconnectSuper();
}

void graph::removeEmptySuper() {
  sortedSuper.erase(
    std::remove_if(sortedSuper.begin(), sortedSuper.end(), [](const SuperNode* super) {return (super->superType == SUPER_VALID || super->superType == SUPER_UPDATE_REG) && super->member.size() == 0; }),
    sortedSuper.end()
  );
}

void graph::orderAllNodes() {
  int order = 1;
  for (size_t i = 0; i < sortedSuper.size(); i ++) {
    sortedSuper[i]->order = i;
    for (size_t j = 0; j < sortedSuper[i]->member.size(); j ++) {
      sortedSuper[i]->member[j]->orderInSuper = j;
      sortedSuper[i]->member[j]->order = order ++;
    }
  }
}
