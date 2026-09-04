#include "common.h"
#include <map>
/* before splitted array  */

void fillEmptyWhen(ExpTree* newTree, ENode* oldNode);


ResetType Node::inferReset() {
  if (reset != UNCERTAIN) return reset;
  Assert(assignTree.size() > 0, "empty assignTree");
  for (ExpTree* tree : assignTree) {
    ResetType newReset = tree->getRoot()->inferReset();
    if (reset == UNCERTAIN) reset = newReset;
    else if (reset != tree->getRoot()->reset) {
      printf("reset %d %d\n", reset, tree->getRoot()->reset);
      Panic();
    }
  }
  return reset;
}

ResetType ENode::inferReset() {
  if (reset != UNCERTAIN) return reset;
  if (nodePtr) {
    reset = nodePtr->inferReset(); 
    return reset;
  }
  int base;
  std::string str;
  switch (opType) {
    case OP_ASUINT:
    case OP_ASSINT:
      reset = UINTRESET;
      break;
    case OP_INT:
      std::tie(base, str) = firStrBase(strVal);
      if (str == "h0" || str == "0")
        reset = ZERO_RESET;
      else {
        std::cout << "Unknown :" << str << std::endl;
        TODO();
      }
      break;
    case OP_ASASYNCRESET:
      reset = ASYRESET;
      break;
    case OP_BITS:
    case OP_OR:
      reset = UINTRESET;
      break;
    case OP_ASCLOCK:
    default:
      printf("opType %d\n", opType);
      Panic();
  }
  return reset;
}

void Node::addReset() { // remove
  Assert(type == NODE_REG_SRC, "%s(%d) is not regsrc", name.c_str(), type);

  ResetType resetType = resetCond->getRoot()->inferReset();
  reset = resetType;
  Assert(resetType != UNCERTAIN, "reset %s is uncertain", name.c_str());
}