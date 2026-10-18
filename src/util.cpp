/**
 * @file util.cpp
 * @brief utils
 */

#include <string>
#include <map>
#include <iostream>
#include "common.h"
#include "Node.h"
#include <execinfo.h>
#include "util.h"
/* convert firrtl constant to C++ constant 
   In the new FIRRTL spec, there are 'int' and 'rint'.*/
std::pair<int, std::string> firStrBase(std::string s) {
  if (s.length() <= 1) { return std::make_pair(10, s); }
  std::string ret;

  int idx  = 0;
  int base = -1;

  // Check if the constant is negative.
  if (s[0] == '-') {
    ret += '-';
    idx = 1;
  }
  if (s[idx] != '0') return std::make_pair(10, s);

  idx ++;

  // If there is no "0b", "0o", "0d", or "0h" prefix, treat it as a base-10 integer.
  if ((s[idx] != 'b') && (s[idx] != 'o') && (s[idx] != 'd') && (s[idx] != 'h')) {
    ret += s.substr(idx - 1);
    return std::make_pair(10, ret);
  }

  // Determine the base from the prefix.
  switch (s[idx]) {
    case 'b': base = 2; break;
    case 'o': base = 8; break;
    case 'h': base = 16; break;
    default: base = 10; break;
  }

  idx ++;
  ret += s.substr(idx);

  return std::make_pair(base, ret);
}

std::string to_hex_string(BASIC_TYPE x) {
  if (x == 0) { return "0"; }

  std::string ret;

  while (x != 0) {
    int rem = x % 16;
    ret     = (char)(rem < 10 ? (rem + '0') : (rem - 10 + 'a')) + ret;
    x /= 16;
  }

  return ret;
}

static std::map<std::string, OPType> expr2Map = {
  {"add", OP_ADD},  {"sub", OP_SUB},  {"mul", OP_MUL},  {"div", OP_DIV},
  {"rem", OP_REM},  {"lt", OP_LT},  {"leq", OP_LEQ},  {"gt", OP_GT},
  {"geq", OP_GEQ},  {"eq", OP_EQ},  {"neq", OP_NEQ},  {"dshl", OP_DSHL},
  {"dshr", OP_DSHR},  {"and", OP_AND},  {"or", OP_OR},  {"xor", OP_XOR},
  {"cat", OP_CAT},
};

OPType str2op_expr2(std::string name) {
  Assert(expr2Map.find(name) != expr2Map.end(), "invalid 2expr op %s\n", name.c_str());
  return expr2Map[name];
}

static std::map<std::string, OPType> expr1Map = {
  {"asUInt", OP_ASUINT}, {"asSInt", OP_ASSINT}, {"asClock", OP_ASCLOCK}, {"asAsyncReset", OP_ASASYNCRESET},
  {"cvt", OP_CVT}, {"neg", OP_NEG}, {"not", OP_NOT}, {"andr", OP_ANDR},
  {"orr", OP_ORR}, {"xorr", OP_XORR},
};

OPType str2op_expr1(std::string name) {
  Assert(expr1Map.find(name) != expr1Map.end(), "invalid 1expr op %s\n", name.c_str());
  return expr1Map[name];
}

static std::map<std::string, OPType> expr1int1Map = {
  {"pad", OP_PAD}, {"shl", OP_SHL}, {"shr", OP_SHR}, {"head", OP_HEAD}, {"tail", OP_TAIL},
};

OPType str2op_expr1int1(std::string name) {
  Assert(expr1int1Map.find(name) != expr1int1Map.end(), "invalid 1expr op %s\n", name.c_str());
  return expr1int1Map[name];
}

int upperPower2(int x) {
  return x <= 1 ? x : (1 << (32 - __builtin_clz(x - 1)));
}

int upperLog2(int x) {
  if (x <= 1) return x;
  return (32 - __builtin_clz(x - 1));
}

// Thread-safe: the old shared 64 MiB static buffer raced once cppEmitter
// started rendering emission units on worker threads. Fast path formats on
// the stack; oversized payloads (huge expression text) fall back to a
// thread-local growable buffer. Output is byte-identical to the old version.
std::string format(const char *fmt, ...) {
  char local[2048];
  va_list args;
  va_start(args, fmt);
  int needed = std::vsnprintf(local, sizeof(local), fmt, args);
  va_end(args);
  Assert(needed >= 0, "vsnprintf encoding error");
  if ((size_t)needed < sizeof(local)) return std::string(local, (size_t)needed);
  static thread_local std::string big;
  big.resize((size_t)needed + 1);
  va_start(args, fmt);
  std::vsnprintf(&big[0], big.size(), fmt, args);
  va_end(args);
  return std::string(big.data(), (size_t)needed);
}

std::string bitMask(int width) {
  Assert(width > 0, "invalid width %d", width);
  if (width <= 64) {
    std::string ret = std::string(width/4, 'f');
    const char* headTable[] = {"", "1", "3", "7"};
    ret = headTable[width % 4] + ret;
    return "0x" + ret;
  } else {
    std::string type = widthUType(width);
    if (width % 64 == 0) { // in such case, (type)1 << width is undefined
      return format("((%s)0 - 1)", type.c_str());
    } else {
      return format("(((%s)1 << %d) - 1)", type.c_str(), width);
    }
  }
}

std::string shiftBits(unsigned int bits, ShiftDir dir){
  if(bits == 0)
    return "";
  return (dir == ShiftDir::Left? " << " : " >> ") + std::to_string(bits);
}
std::string shiftBits(std:: string bits, ShiftDir dir){
  if(bits == std::to_string(0) || bits == "0x0")
    return "";
  return (dir == ShiftDir::Left? " << " : " >> ") + bits;
}

bool mtDensePendingMemEnabled() {
  const char* value = std::getenv("GSIM_MT_DENSE_PENDING_MEM");
  return value != nullptr && value[0] != '\0' && value[0] != '0';
}

bool mtDensePendingMemEdgeOnly() {
  const char* value = std::getenv("GSIM_MT_DENSE_PENDING_MEM_EDGE_ONLY");
  return value != nullptr && value[0] != '\0' && value[0] != '0';
}
bool mtDenseStatePhaseEnabled() {
  const char* value = std::getenv("GSIM_MT_DENSE_STATE_PHASE");
  return value != nullptr && value[0] != '\0' && value[0] != '0';
}

// A pure register-commit super: every valid member is a NON-async-reset
// NODE_REG_SRC whose assignTree carries the x = x$NEXT update copy. With
// GSIM_MT_DENSE_STATE_PHASE=1 such supers leave the compute DAG; the copies
// run in parallel cycle-start state slices behind an all-worker barrier
// (upstream dev-mt's state-task architecture ported onto the dense executor).
// Async-reset registers keep the in-DAG commit: async supers still write x
// mid-cycle, so a cycle-start copy could clobber them.
static bool mtDensePureCommitSuper(const SuperNode* super) {
  if (super == nullptr || super->superType != SUPER_VALID) return false;
  bool any = false;
  for (const Node* member : super->member) {
    if (member == nullptr || member->status != VALID_NODE) continue;
    if (member->type != NODE_REG_SRC) return false;
    if (member->assignTree.empty()) return false;  // no update copy here
    if (member->reset == ASYRESET) return false;   // async path writes x mid-cycle
    any = true;
  }
  return any;
}

bool mtDenseStatePhaseSuperExcluded(const SuperNode* super) {
  return mtDenseStatePhaseEnabled() && mtDensePureCommitSuper(super);
}
bool mtDensePendingMemPost() {
  const char* value = std::getenv("GSIM_MT_DENSE_PENDING_MEM_POST");
  return value != nullptr && value[0] != '\0' && value[0] != '0';
}

static std::set<int> g_mtDensePendingStagedWriterIds;

bool mtDensePendingMemWriterStaged(const Node* port) {
  return port != nullptr &&
         g_mtDensePendingStagedWriterIds.find(port->id) != g_mtDensePendingStagedWriterIds.end();
}

void mtDensePendingMemSetStagedWriters(const std::set<int>& nodeIds) {
  g_mtDensePendingStagedWriterIds = nodeIds;
}

bool mtDensePendingMemEligible(const Node* port) {
  if (port == nullptr || port->status != VALID_NODE || port->parent == nullptr) return false;
  if (port->type != NODE_WRITER && port->type != NODE_READWRITER) return false;
  const Node* memory = port->parent;
  return memory->type == NODE_MEMORY && port->dimension.empty() &&
         memory->dimension.empty() && port->width > 0 && port->width <= 64 &&
         memory->width > 0 && memory->width <= 64;
}

bool mtDensePendingMemSingleWriter(const Node* memory, const Node* port) {
  if (memory == nullptr || memory->type != NODE_MEMORY || port == nullptr ||
      port->parent != memory || !mtDensePendingMemEligible(port)) return false;
  const Node* onlyWriter = nullptr;
  for (const Node* member : memory->member) {
    if (member == nullptr || member->status != VALID_NODE) continue;
    if (member->type != NODE_WRITER && member->type != NODE_READWRITER) continue;
    if (!mtDensePendingMemEligible(member)) return false;
    if (onlyWriter != nullptr) return false;
    onlyWriter = member;
  }
  return onlyWriter == port;
}
static bool mtDensePendingMemMarkedEdge(const Node* from, const Node* to) {
  return from != nullptr && to != nullptr && from->type == NODE_READER &&
         to->type == NODE_WRITER && from->parent != nullptr &&
         from->parent == to->parent &&
         from->memOrderNext.find(const_cast<Node*>(to)) != from->memOrderNext.end() &&
         to->parent->extraInfo != "new" &&
         mtDensePendingMemSingleWriter(to->parent, to);
}

static bool mtDensePendingMemPairOnly(const SuperNode* from, const SuperNode* to) {
  if (from == nullptr || to == nullptr) return false;
  bool saw = false;
  for (Node* source : from->member) {
    if (source == nullptr) continue;
    for (Node* successor : source->next) {
      if (successor == nullptr || successor->super != to) continue;
      saw = true;
      if (!mtDensePendingMemMarkedEdge(source, successor)) return false;
    }
    for (Node* successor : source->depNext) {
      if (successor == nullptr || successor->super != to) continue;
      saw = true;
      if (!mtDensePendingMemMarkedEdge(source, successor)) return false;
    }
  }
  return saw;
}

bool mtDensePendingMemWriterCandidate(const Node* port) {
  if (!mtDensePendingMemSingleWriter(port ? port->parent : nullptr, port) ||
      port->parent->extraInfo == "new" || port->super == nullptr) return false;
  for (Node* reader : port->parent->member) {
    if (reader == nullptr || reader->type != NODE_READER || reader->super == nullptr ||
        reader->super == port->super) continue;
    if (reader->memOrderNext.find(const_cast<Node*>(port)) == reader->memOrderNext.end()) continue;
    if (mtDensePendingMemPairOnly(reader->super, port->super)) return true;
  }
  return false;
}


std::string mtDensePendingMemAddressName(const Node* port) {
  return "__gsim_dense_pm_" + std::to_string(port->id) + "_addr";
}

std::string mtDensePendingMemDataName(const Node* port) {
  return "__gsim_dense_pm_" + std::to_string(port->id) + "_data";
}

std::string mtDensePendingMemValidName(const Node* port) {
  return "__gsim_dense_pm_" + std::to_string(port->id) + "_valid";
}


void print_stacktrace() {
  int size = 16;
  void * array[16];
  int stack_num = backtrace(array, size);
  char ** stacktrace = backtrace_symbols(array, stack_num);
  for (int i = 0; i < stack_num; ++i) {
    fprintf(stderr, "%s\n", stacktrace[i]);
  }
  free(stacktrace);
}
