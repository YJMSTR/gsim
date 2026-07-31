/*
  nodes in circuit graph
*/

#ifndef NODE_H
#define NODE_H

#include "debug.h"
std::string format(const char *fmt, ...);

class NodeComponent;
class StmtTree;

enum NodeType{
  NODE_INVALID,
  NODE_REG_SRC,
  NODE_REG_DST,
  NODE_SPECIAL, // printf & assert
  NODE_INP,
  NODE_OUT,
  NODE_MEMORY,
  NODE_READER,
  NODE_WRITER,
  NODE_READWRITER,
  NODE_INFER,
  NODE_OTHERS,
  NODE_REG_RESET,
  NODE_EXT_IN,
  NODE_EXT_OUT,
  NODE_EXT,
};

enum NodeStatus{
  VALID_NODE,
  DEAD_NODE,
  CONSTANT_NODE,
  REPLICATION_NODE,
  SPLITTED_NODE,
  EMPTY_REG  /* reg_src or reg_dst that are empty, which means that regNext represets the whole register */
};
enum IndexType{ INDEX_INT, INDEX_NODE };
enum AsReset { EMPTY, NODE_ASYNC_RESET, NODE_UINT_RESET, NODE_ALL_RESET};

enum ReaderMember { READER_ADDR = 0, READER_EN, READER_CLK, READER_DATA, READER_MEMBER_NUM};
enum WriterMember { WRITER_ADDR = 0, WRITER_EN, WRITER_CLK, WRITER_DATA, WRITER_MASK, WRITER_MEMBER_NUM};
enum ReadWriterMember { READWRITER_ADDR = 0, READWRITER_EN, READWRITER_CLK,
                      READWRITER_RDATA, READWRITER_WDATA, READWRITER_WMASK,
                      READWRITER_WMODE, READWRITER_MEMBER_NUM};

class TypeInfo {
public:
  int width;
  bool sign;
  bool clock = false;
  ResetType reset = UNCERTAIN;
  std::vector<int> dimension;
  std::vector<std::pair<Node*, bool>> aggrMember; // all members, if not empty, all other infos above are invalid
  std::vector<AggrParentNode*> aggrParent; // the later the outer

  void add(Node* node, bool isFlip) { aggrMember.push_back(std::make_pair(node, isFlip)); }
  void set_width(int _width) { width = _width; }
  void set_sign(bool _sign) { sign = _sign; }
  void set_clock(bool is_clock) { clock = is_clock; }
  bool isClock() { return clock; }
  void set_reset(ResetType type) { reset = type; }
  ResetType getReset() { return reset; }
  void mergeInto(TypeInfo* info);
  bool isAggr() { return aggrParent.size() != 0; }
  void addDim(int num);
  void newParent(std::string name);
  void flip();
};

/*
e.g. {{a, b}x, {c}y }z;
  z.parent = {z_x, z_y} z.member = {z_x_a, z_x_b, z_y_c};
  x.parent = {}, x.member = {z_x_a, z_x_b}
*/
class AggrParentNode {  // virtual type_aggregate node, used for aggregate connect
  public:
  std::string name;
  std::vector<std::pair<Node*, bool>> member;  // leaf member
  std::vector<AggrParentNode*> parent; // non-leaf member (aggregate type), increasing partial order
  AggrParentNode(std::string _name, TypeInfo* info = nullptr);
  int size() {
    return member.size();
  }
  void addMember(Node* _member, bool isFlip) {
    member.push_back(std::make_pair(_member, isFlip));
  }
  void addParent(AggrParentNode* _parent) {
    parent.push_back(_parent);
  }
};

class Node {
  static int counter;
 public:

  Node(NodeType _type = NODE_OTHERS) {
    type    = _type;
    id = counter ++;
  }

  std::string name;  // concat the module name in order (member in structure / temp variable)
  std::string extraInfo; // ruw for memory
  int id = -1;
  NodeType type;
  int width = -1;
  bool sign = false;
  int usedBit = -1;
  NodeStatus status = VALID_NODE;
  std::vector<int> dimension;
  int order = -1;
  int orderInSuper = -1;
  int lineno = -1;
  /* adjacent */
  std::set<Node*> next;
  std::set<Node*> prev;
  /* dependent but not adjacent
   * e.g. reg_src -> node1; node2->reg_dst; then:
   * node1 is depPrev of reg_dst, as activeFlags of node1 must first be cleared before reg_dst is activated
  */
  std::set<Node*> depPrev;
  std::set<Node*> depNext;
  std::vector <ExpTree*> assignTree;
  SuperNode* super = nullptr;
  std::vector<Node*> member;
  /* for extmodule */
  std::vector<std::pair<bool, std::string>> params; // <isInt, value>
  /* only used in AST2Graph */
  ExpTree* valTree = nullptr;
  /* for registers in AST2Graph*/
  ExpTree* resetCond = nullptr;  // valid in reg_src
  ExpTree* resetVal = nullptr;   // valid in reg_src, used in AST2Graph
  /* used for memory in AST2Graph*/
  int rlatency;
  int wlatency;
  int depth;
  ExpTree* memTree = nullptr;
  Node* parent = nullptr;
/* used for registers */
  Node* regNext = nullptr;
  ExpTree* resetTree = nullptr;
  bool regSplit = true;
/* used for instGerator */
  valInfo* computeInfo = nullptr;
/* used for reg & memory */
  Node* clock = nullptr;
  bool isClock = false;
  ResetType reset = UNCERTAIN;
  AsReset asReset = EMPTY;
  bool inAggr = false;
/* used for visitWhen in AST2Graph */
  size_t whenDepth = 0;

/* used in instsGenerator */
  bool nodeIsRoot = false;

/* used in cppEmitter */
  struct ActivationOriginRecord {
    int fromCppId = -1;
    int toCppId = -1;
    int sourceNodeId = -1;
    int targetNodeId = -1;
    NodeType sourceNodeType = NODE_INVALID;
    NodeType targetNodeType = NODE_INVALID;
    std::string sourceNodeName;
    std::string targetNodeName;
    std::string reason;
  };
  std::set<int> nextActiveId;
  std::set<int> nextNeedActivate;
  void updateInfo(TypeInfo* info);
  void setType(int _width, bool _sign) {
    width = _width;
    sign = _sign;
  }
  Node* dup(NodeType _type = NODE_INVALID, std::string _name = "");
  /* bind two the splitted registers */
  void bindReg(Node* reg) {
    regNext = reg;
    reg->regNext = this;
  }

  Node* getDst () {
    Assert(type == NODE_REG_SRC || type == NODE_REG_DST, "The node %s is not register", name.c_str());
    if (type == NODE_REG_SRC) return this->regNext;
    return this;
  }
  Node* getSrc () {
    Assert(type == NODE_REG_SRC || type == NODE_REG_DST, "The node %s is not register", name.c_str());
    if (type == NODE_REG_DST) return this->regNext;
    return this;
  }
  Node* getResetSrc () {
    Assert(type == NODE_REG_RESET, "The node %s is not register", name.c_str());
    return regNext;
  }
  Node* getBindReg() {
    Assert(type == NODE_REG_SRC || type == NODE_REG_DST, "The node %s is not register", name.c_str());
    return this->regNext;
  }
  void set_memory(int _depth, int r, int w) {
    depth = _depth;
    rlatency = r;
    wlatency = w;
  }
  void add_member(Node* _member) {
    member.push_back(_member);
    if(_member) _member->set_parent(this);
  }
  void set_member(int idx, Node* node) {
    Assert(idx < (int)member.size(), "idx %d is out of bound [0, %ld)", idx, member.size());
    member[idx] = node;
    node->set_parent(this);
  }
  Node* get_member(int idx) {
    Assert(idx < (int)member.size(), "idx %d is out of bound [0, %ld)", idx, member.size());
    return member[idx];
  }
  void set_writer() {
    if (type == NODE_READWRITER) return;
    Assert(type == NODE_WRITER || type == NODE_INFER, "invalid type %d\n", type);
    type = NODE_WRITER;
  }
  void set_reader() {
    Assert(type == NODE_READER || type == NODE_INFER, "invalid type %d\n", type);
    type = NODE_READER;
  }
  Node* get_port_clock() {
    Assert(type == NODE_READER || type == NODE_WRITER, "invalid type %d in node %s", type, name.c_str());
    if (type == NODE_READER) return get_member(READER_CLK);
    else if (type == NODE_WRITER) return get_member(WRITER_CLK);
    Panic();
  }
  void set_parent(Node* _parent) {
    Assert(!parent, "parent in %s is already set", name.c_str());
    parent = _parent;
  }
  bool isArray() {
    return dimension.size() != 0;
  }
  void addArrayVal(ExpTree* val);
  void set_super(SuperNode* _super) {
    super = _super;
  }
  void update_usedBit(int bits) {
    usedBit = MIN(width, MAX(bits, usedBit));
  }
  void setAsyncReset() {
    if (asReset == NODE_UINT_RESET || asReset == NODE_ALL_RESET) asReset = NODE_ALL_RESET;
    else asReset = NODE_ASYNC_RESET;
  }
  void setUIntReset() {
    if (asReset == NODE_ASYNC_RESET || asReset == NODE_ALL_RESET) asReset = NODE_ALL_RESET;
    else asReset = NODE_UINT_RESET;
  }
  bool isAsyncReset() {
    return asReset == NODE_ASYNC_RESET || asReset == NODE_ALL_RESET;
  }
  bool isUIntReset() {
    return asReset == NODE_UINT_RESET || asReset == NODE_ALL_RESET;
  }
  bool isReset() {
    return asReset == NODE_UINT_RESET || asReset == NODE_ASYNC_RESET || asReset == NODE_ALL_RESET;
  }
  bool isExt() {
    return type == NODE_EXT || type == NODE_EXT_IN || type == NODE_EXT_OUT;
  }
  void clear_relation();
  void addPrev(Node* node);
  void addPrev(std::set<Node*>& super);
  void addPrev(std::vector<Node*>& super);
  void erasePrev(Node* node);
  void addDepPrev(Node* node);
  void eraseDepPrev(Node* node);
  void addNext(Node* node);
  void addNext(std::set<Node*>& super);
  void addNext(std::vector<Node*>& super);
  void eraseNext(Node* node);
  void addDepNext(Node* node);
  void eraseDepNext(Node* node);
  void clearPrev();
  void updateDep();
  void updateConnect();
  void inferWidth();
  void clearWidth();
  void addReset();
  void addUpdateTree();
  void constructSuperNode(); // alloc superNode for every node
  void constructSuperConnect(); // connect superNode
  valInfo* computeConstantArray();
  void recomputeConstant();
  void passWidthToPrev();
  void splitArray();
  Node* arrayMemberNode(int idx);
  ENode* isAlias();
  bool anyExtEdge();
  bool needActivate();
  bool anyNextActive();
  const std::vector<ActivationOriginRecord>& activationOrigins() const;
  void recordNextActiveOrigin(int toCppId, Node* targetNode, const char* reason);
  void updateActivate();
  void updateNeedActivate(std::set<int>& alwaysActive);
  void removeConnection();
  void allocArrayVal();
  Node* clockAlias();
  clockVal* clockCompute();
  ResetType inferReset();
  void setConstantZero(int w = -1);
  void setConstantInfoZero(int w = -1);
  bool isFakeArray() { return dimension.size() == 1 && dimension[0] == 1; }
  void display();
  size_t arrayEntryNum() { size_t num = 1; for (size_t idx : dimension) num *= idx; return num; }
  valInfo* computeConstant();
  valInfo* computeRegConstant();
  void invalidArrayOptimize();
  uint64_t keyHash();
  NodeComponent* inferComponent();
  NodeComponent* reInferComponent();
  void updateTreeWithNewWIdth();
  int repOpCount();
  void updateIsRoot();
  void updateHeadTail();
  bool isLocal();
  void setConstant(valInfo* info);
};

enum SuperType {
  SUPER_VALID,
  SUPER_EXTMOD,
  SUPER_ASYNC_RESET,
  SUPER_UINT_RESET,
  SUPER_UPDATE_REG,
};

enum SuperInfo {
  SUPER_INFO_IF,     // start indent
  SUPER_INFO_ELSE,   // dedent and then indent
  SUPER_INFO_DEDENT,
  SUPER_INFO_STR,
  SUPER_INFO_ASSIGN_BEG, // backup the value
  SUPER_INFO_ASSIGN_END  // compare the backuped value
};

class InstInfo{
public:
  SuperInfo infoType = SUPER_INFO_STR;
  std::string inst;
  Node* node;
  InstInfo(SuperInfo _type, Node* _node) {
    infoType = _type;
    node = _node;
  }
  InstInfo(std::string _inst, SuperInfo _type = SUPER_INFO_STR) {
    infoType = _type;
    inst = _inst;
  }
  InstInfo(SuperInfo _infoType, Node* _node, std::string _inst) {
    node = _node;
    inst = _inst;
    infoType = _infoType;
  }
  bool operator<(const InstInfo& other) const {
    if (infoType != other.infoType) return infoType < other.infoType;
    if (inst != other.inst) return inst < other.inst;
    return node < other.node;
  }
};
class SuperNode {
private:
  static int counter;  // initialize to 1
public:
  /* adjacent superNodes */
  std::set<SuperNode*> prev;
  std::set<SuperNode*> next;
  /* dependent but not adjacent */
  std::set<SuperNode*> depPrev;
  std::set<SuperNode*> depNext;
  std::vector<Node*> member; // The order of member is neccessary
  std::vector<InstInfo> insts;
  StmtTree* stmtTree = nullptr;
  int id;
  int order;
  int cppId = -1;
  SuperType superType = SUPER_VALID;
  Node* resetNode = nullptr;
  SuperNode() {
    id = counter ++;
  }
  SuperNode(Node* _member) {
    member.push_back(_member);
    id = counter ++;
  }
  void add_member(Node* _member) {
    Assert(std::find(member.begin(), member.end(), _member) == member.end(), "member %s is already in superNode %d", _member->name.c_str(), id);
    member.push_back(_member);
    _member->set_super(this);
  }
  void connectPrev(SuperNode* _prev) {
    addPrev(_prev);
    _prev->addNext(this);
  }
  void connectNext(SuperNode* _next) {
    addNext(_next);
    _next->addPrev(this);
  }
  bool instsEmpty();
  void display();
  int findIndex(Node* node) {
    for (size_t ret = 0; ret < member.size(); ret ++) {
      if (member[ret] == node) return ret;
    }
    node->super->display();
    node->display();
    Panic();
  }
  void clear_relation();
  void addPrev(SuperNode* super);
  void addPrev(std::set<SuperNode*>& super);
  void erasePrev(SuperNode* super);
  void addDepPrev(SuperNode* super);
  void eraseDepPrev(SuperNode* super);
  void addNext(SuperNode* super);
  void addNext(std::set<SuperNode*>& super);
  void eraseNext(SuperNode* super);
  void eraseDepNext(SuperNode* super);
  void addDepNext(SuperNode* super);
  void reorderMember();
};

// v430: stable semantic order for graph passes. Pointer-ordered containers made pass
// outcomes depend on allocator/address layout (v430 nondeterminism). Key chain:
// first member name, member count, full member-name list, dep sizes; id only as a last
// resort and emits a stderr warning (allocator-unstable, investigate if ever hit).
struct SuperNodeStableLess {
  bool operator()(const SuperNode* a, const SuperNode* b) const {
    // v432: deterministic key approximating parse/creation order (pointer order quality
    // without allocator dependence). lineno is content-stable; -1 for generated nodes.
    const long long la = a->member.empty() ? LLONG_MAX : (long long)a->member[0]->lineno;
    const long long lb = b->member.empty() ? LLONG_MAX : (long long)b->member[0]->lineno;
    if (la != lb) return la < lb;
    const std::string& na = a->member.empty() ? std::string() : a->member[0]->name;
    const std::string& nb = b->member.empty() ? std::string() : b->member[0]->name;
    if (na != nb) return na < nb;
    if (a->member.size() != b->member.size()) return a->member.size() < b->member.size();
    for (size_t i = 0; i < a->member.size() && i < b->member.size(); i ++) {
      if (a->member[i]->name != b->member[i]->name) return a->member[i]->name < b->member[i]->name;
    }
    if (a->depPrev.size() != b->depPrev.size()) return a->depPrev.size() < b->depPrev.size();
    if (a->depNext.size() != b->depNext.size()) return a->depNext.size() < b->depNext.size();
    static bool warned = false;
    if (!warned) { warned = true; fprintf(stderr, "[stable-order] id fallback hit: %d vs %d (names %s | %s)\n", a->id, b->id, na.c_str(), nb.c_str()); }
    return a->id < b->id;
  }
};
static inline std::vector<SuperNode*> stableOrdered(const std::set<SuperNode*>& ends) {
  std::vector<SuperNode*> ordered(ends.begin(), ends.end());
  std::sort(ordered.begin(), ordered.end(), SuperNodeStableLess());
  return ordered;
}

// GSIM_SCHEDULE_SEED fixed-order replay (src/seedOrder.cpp; design docs/schedule-pinning-design.md).
bool mtSeedReplayActive();
bool mtSeedWriteActive();
void mtSeedAssertCompatible();
long long mtSeedRankOf(const SuperNode* super);
void mtSeedVerifyInputHash(uint64_t computedInputHash);
void mtSeedWrite(const char* path, const std::vector<SuperNode*>& sortedSuper, uint64_t inputHash, const char* generatorTag);
struct SeedRankLess {
  bool operator()(const SuperNode* a, const SuperNode* b) const {
    const long long ra = mtSeedRankOf(a), rb = mtSeedRankOf(b);
    if (ra != rb) return ra < rb;
    if (ra == LLONG_MAX) {
      // Seed-missing nodes (created after the seed point, e.g. replication): fall back to
      // the content-name order so the comparator stays deterministic across allocators.
      return SuperNodeStableLess()(a, b);
    }
    return a < b;  // pointer only as a last-resort tie-break (never hit on a matching seed)
  }
};
static inline std::vector<SuperNode*> seedRankOrdered(const std::set<SuperNode*>& ends) {
  std::vector<SuperNode*> ordered(ends.begin(), ends.end());
  std::sort(ordered.begin(), ordered.end(), SeedRankLess());
  return ordered;
}

#endif
