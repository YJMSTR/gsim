/*
  genCheckpoint.cpp: pre-Final global-graph checkpoint. See genCheckpoint.h.

  ============================ KEY COMPLETENESS ============================
  The checkpoint restores the graph produced by the pre-Final pipeline
  (parseFIR .. last removeDeadNodes). A stale hit is impossible by
  construction iff every input that can change one bit of that graph is in
  the key. Those inputs are exactly:

    1. complete FIR file text   - the only semantic design input; hashed
                                  before parseFIR() splits the buffer.
    2. pre-Final source closure - every source file whose code executes
                                  before the boundary (see kPreFinalSrc and
                                  the classification below), plus compiler
                                  identity (first line of clang++ --version)
                                  and allocator identity (sha256 of the
                                  sorted address-stripped ldd dependency
                                  list; the Makefile MALLOC probe links
                                  -ljemalloc / -lmimalloc / -ltcmalloc_minimal
                                  and pointer-order-sensitive pre-Final
                                  trajectories can follow the allocator).
                                  Version macros (git describe baked into
                                  every TU by the Makefile) are NOT in the
                                  closure: source bytes are hashed, so a
                                  rebuild that touches no pre-Final source
                                  keeps hitting (the S1 scenario).
    3. Config fields read       - grepping globalConfig across every
       before the boundary        pre-Final TU yields exactly two:
                                  sep_module/sep_aggr (AST2Graph.cpp:13-14
                                  SEP_MODULE/SEP_AGGR name separators).
                                  Every other Config field is read only by
                                  Final code (see the exclusion table in
                                  the closure comment below), so knob
                                  sweeps over Final flags (the S2 scenario)
                                  keep hitting.
    4. environment read         - none. std::getenv appears pre-Final
       before the boundary         nowhere: the only generation-time env
                                  reads (GSIM_THREADS,
                                  GSIM_EMIT_RESET_CHUNK, GSIM_MT_DYNAMIC_QUERY)
                                  live in cppEmitter-mt.cpp (Final).
                                  NumThreads/GSIM_THREADS shape only the
                                  parse worker count; the parse result is
                                  identical for any thread count (fixed
                                  MODULES_PER_TASK split boundaries,
                                  sequential merge; same argument as
                                  genCache.cpp's NumThreads exclusion).

  Classification (verified by grepping callers, not by file name):
    pre-Final TUs: everything main() calls between parseFIR and the last
      removeDeadNodes plus every TU they call into. StmtTree.cpp and
      mergeNodes.cpp are FINAL-only: SuperNode::reorderMember and
      graph::generateStmtTree run in generateStmtTree (serial Final) or
      mtTaskLowering (mt Final); graph::mergeWhenNodes/when2mux/mergeOut1/
      mergeIn1/mergeSublings/mergeResetAll run in graphCoarsen (serial
      Final), MFFCPartition and MtTaskPartitioner::build (mt Final).
      instsGenerator.cpp IS pre-Final-relevant: constantNode.cpp (pre-Final
      constant analysis) calls allOnes/legalCppCons defined there, and
      valInfo::setConsStr (valInfo.h) calls legalCppCons.
    Final-only TUs/headers are ENUMERATED but NOT hashed: that is the whole
      point of the checkpoint (emitter/scheduler edits and mt knobs change
      only how Final consumes the graph). Any src/ or include/ entry that is
      in neither list disables the feature for the run (fail-closed: a new
      TU must not silently escape the closure).

  ======================= SERIALIZATION FIELD COVERAGE =====================
  Every field of every serialized class is written; the only skipped fields
  (each with grep-verified dead-at-boundary justification):
    Node::valTree/resetCond/resetVal/memTree - written and read only inside
      AST2Graph.cpp (consumed while building assignTree/resetTree); no
      reader exists after AST2Graph returns.
    Node::rlatency/wlatency/depth for non-NODE_MEMORY nodes - the three
      fields have no default initializer (indeterminate for non-memory
      nodes) and are only ever read through a NODE_MEMORY node
      (AST2Graph.cpp:749/1571, cppEmitter*.cpp genNodeDef under a
      NODE_MEMORY guard); stored normalized to 0, never read for
      non-memory nodes.
    valInfo::mask - private, recomputed from width by updateConsVal(),
      never read anywhere else.
    graph::srcFp/srcFileIdx/srcFileBytes - private emitter stream state;
      the restore allocates the graph with the same `new graph()` value
      initialization AST2Graph uses (AST2Graph.cpp:1554), so the state is
      identical by construction.
  Original Node/ENode/SuperNode ids are preserved (they appear in emitted
  text: oldName(node) = name + "$old$" + id, and in the mt CSV reports),
  and the three static id counters are stored and restored exactly so that
  objects Final allocates keep receiving the ids a no-checkpoint run would
  have assigned (byte-identity gate).

  ============================== FAILURE MODES =============================
  Every anomaly resolves to a full-pipeline run, never to a stale restore:
  FIR/pre-Final-source/flag change -> key differs -> different entry -> miss.
  Entry corrupt/truncated/foreign (bad magic, wrong key line, unparseable
  manifest, body sha mismatch, trailing bytes after the graph section,
  out-of-range reference, duplicate id) -> miss. Closure ambiguity (unknown
  TU/header, missing enumerated file, source newer than the running binary,
  clang++/ldd probe failure, source tree not found) -> feature disabled for
  the run (no store, no restore). Dump flags -> feature disabled (a restore
  skips the passes that would produce dump artifacts). A crashed store
  leaves only an unpublished .tmp.* dir (GC'd after 72h). Publishing races
  between two processes are resolved by atomic rename: the loser of an
  ENOTEMPTY rename keeps the existing valid entry.
*/

#include "genCheckpoint.h"

#include "common.h"
#include "genCacheInternal.h"

#include <algorithm>
#include <functional>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace genCheckpoint {

namespace {

using namespace genCacheUtil;

/* ------------------------------------------------------------------ key */

const char kKeyMagic[] = "GSIM_GCKPT_KEY_V1";
const char kEntryMagic[] = "GSIM_GCKPT_ENTRY_V1";
const char kBodyMagic[] = "GSIM_GCKPT_V1";  // 13 chars + NUL
const uint32_t kBodyFormat = 1;

/* src translation units whose code runs before the removeDeadNodes ->
   beginEmission boundary (hashed). genCache.cpp is an observer of the run,
   never a graph mutator; it is hashed anyway (fail-closed conservatism). */
const char* const kPreFinalSrc[] = {
    "AST2Graph.cpp",               /* Init: FIR AST -> global graph */
    "ENode.cpp",                   /* expression-node support used everywhere */
    "ExpTree.cpp",                 /* expression-tree support used everywhere */
    "Node.cpp",                    /* node support used everywhere */
    "aliasAnalysis.cpp",           /* pre-Final pass */
    "clockOptimize.cpp",           /* called from AST2Graph in both modes */
    "commonExpr.cpp",              /* pre-Final pass */
    "constantNode.cpp",            /* pre-Final pass */
    "deadNodes.cpp",               /* pre-Final pass */
    "exprOpt.cpp",                 /* pre-Final pass */
    "firParser.cpp",               /* parseFIR driver */
    "genCache.cpp",                /* run observer; hashed conservatively */
    "genCheckpoint.cpp",           /* this checkpoint (store/restore semantics) */
    "graph.cpp",                   /* graph container support (orderAllNodes ...) */
    "inferWidth.cpp",              /* pre-Final pass */
    "instsGenerator.cpp",          /* allOnes/legalCppCons used by constantNode */
    "loopDetector.cpp",            /* pre-Final pass */
    "main.cpp",                    /* pipeline driver: stage order fixes semantics */
    "opFuncs.cpp",                 /* constant-folding ops used by passes */
    "patternDetect.cpp",           /* pre-Final pass */
    "resetAnalysis.cpp",           /* Node::addReset called from AST2Graph */
    "splitArray.cpp",              /* pre-Final pass */
    "splitNodes.cpp",              /* pre-Final pass */
    "superNode.cpp",               /* SuperNode support used everywhere */
    "topoSort.cpp",                /* pre-Final pass */
    "traversal.cpp",               /* graph traversal support */
    "usedBits.cpp",                /* pre-Final pass */
    "util.cpp",                    /* shared helpers (format, str2op, ...) */
};

/* src translation units provably unreachable before the boundary. NOT
   hashed (Final-only edits must keep hitting), but ENUMERATED: an unknown
   src entry disables the checkpoint instead of silently escaping.
     StmtTree.cpp / mergeNodes.cpp / MFFCPartition.cpp : Final-only, see the
       header comment for the call-graph evidence.
     cppEmitter.cpp / cppEmitter-mt.cpp / graphPartition.cpp /
       mtTaskLowering.cpp / mtTaskPartition.cpp / mtTaskSchedule.cpp /
       mtReplicationSchedule.cpp / replication.cpp : the emission stage and
       its helpers, main()'s post-boundary calls only.
     mergeRegister.cpp : no caller anywhere in the tree (kept enumerated for
       the same reason as in the genCache closure).
     GraphDumper.cpp : runs only under dump flags, which disable this
       feature outright.
     perfAnalysis.cpp : no caller anywhere in the tree. */
const char* const kFinalOnlySrc[] = {
    "GraphDumper.cpp",
    "MFFCPartition.cpp",
    "StmtTree.cpp",
    "cppEmitter-mt.cpp",
    "cppEmitter.cpp",
    "graphPartition.cpp",
    "mergeNodes.cpp",
    "mergeRegister.cpp",
    "mtReplicationSchedule.cpp",
    "mtTaskLowering.cpp",
    "mtTaskPartition.cpp",
    "mtTaskSchedule.cpp",
    "perfAnalysis.cpp",
    "replication.cpp",
};

/* include/ headers reachable from the pre-Final TUs (common.h's transitive
   include set plus splitNode.h used by splitNodes.cpp). Hashed. */
const char* const kPreFinalIncH[] = {
    "ExpTree.h",
    "Node.h",
    "StmtTree.h",
    "common.h",
    "config.h",
    "debug.h",
    "graph.h",
    "opFuncs.h",
    "perf.h",
    "splitNode.h",
    "util.h",
    "valInfo.h",
};

/* include/ headers only the Final TUs include (emitter/scheduler surface).
   Enumerated, not hashed: the S1 scenario. */
const char* const kFinalOnlyIncH[] = {
    "cppEmitter-mt.h",
    "mtCostModel.h",
    "mtReplicationSchedule.h",
    "mtTaskLowering.h",
    "mtTaskPartition.h",
    "mtTaskSchedule.h",
};

/* src headers included by same-directory TUs. */
const char* const kClosureSrcH[] = {
    "genCache.h",
    "genCacheInternal.h",
    "genCheckpoint.h",
};

/* Collects the pre-Final closure and computes its fingerprint. On failure
   returns false with *reason set (caller disables the checkpoint for the
   run: unlike genCache there is no binary-sha fallback key, because the
   whole point is to survive Final-only rebuilds). */
bool collectPreFinalClosure(std::string* closureSha, size_t* fileCount,
                            std::string* reason) {
  std::string compilerLine;
  if (!resolveCompilerIdentity(&compilerLine, reason)) return false;
  std::string exePath;
  if (!resolveSelfExe(&exePath, reason)) return false;
  /* Stale-binary window guard: a hashed source newer than the running
     binary means the binary was not built from the sources being hashed. */
  struct stat exeSb;
  if (stat(exePath.c_str(), &exeSb) != 0) {
    *reason = std::string("cannot stat ") + exePath;
    return false;
  }
  std::string depsSha;
  if (!resolveAllocatorIdentity(exePath, &depsSha, reason)) return false;
  std::string root;
  if (!findSourceRoot(exePath, &root, reason)) return false;

  std::vector<std::string> files;
  std::vector<std::string> entries;
  if (!listDirEntries(root + "/src", &entries)) {
    *reason = "cannot list " + root + "/src";
    return false;
  }
  std::vector<std::string> srcSeen;
  for (const std::string& e : entries) {
    if (e.back() == '/') {
      *reason = "unexpected subdirectory in src/: " + e;
      return false;
    }
    if (endsWith(e, ".cpp")) {
      srcSeen.push_back(e);
      if (nameInList(kFinalOnlySrc, sizeof kFinalOnlySrc / sizeof kFinalOnlySrc[0], e)) {
        continue;
      }
      if (!nameInList(kPreFinalSrc, sizeof kPreFinalSrc / sizeof kPreFinalSrc[0], e)) {
        *reason = "unknown src translation unit: src/" + e;
        return false;
      }
      files.push_back("src/" + e);
    } else if (endsWith(e, ".h")) {
      if (!nameInList(kClosureSrcH, sizeof kClosureSrcH / sizeof kClosureSrcH[0], e)) {
        *reason = "unknown src header: src/" + e;
        return false;
      }
      files.push_back("src/" + e);
    } else {
      *reason = "unrecognized file in src/: " + e;
      return false;
    }
  }
  /* Every enumerated pre-Final TU must be present and hashed. */
  const size_t nPre = sizeof kPreFinalSrc / sizeof kPreFinalSrc[0];
  for (size_t i = 0; i < nPre; i++) {
    if (std::find(srcSeen.begin(), srcSeen.end(), std::string(kPreFinalSrc[i])) ==
        srcSeen.end()) {
      *reason = std::string("enumerated source missing: src/") + kPreFinalSrc[i];
      return false;
    }
  }
  entries.clear();
  if (!listDirEntries(root + "/include", &entries)) {
    *reason = "cannot list " + root + "/include";
    return false;
  }
  for (const std::string& e : entries) {
    if (e.back() == '/') {
      *reason = "unexpected subdirectory in include/: " + e;
      return false;
    }
    if (!endsWith(e, ".h")) {
      *reason = "unrecognized file in include/: " + e;
      return false;
    }
    if (nameInList(kFinalOnlyIncH, sizeof kFinalOnlyIncH / sizeof kFinalOnlyIncH[0], e)) {
      continue;
    }
    if (!nameInList(kPreFinalIncH, sizeof kPreFinalIncH / sizeof kPreFinalIncH[0], e)) {
      *reason = "unknown include header: include/" + e;
      return false;
    }
    files.push_back("include/" + e);
  }
  /* Every non-hidden file under parser/ recursively (the parse runs before
     the boundary, so all of it is pre-Final). */
  {
    /* Every non-hidden file under parser/ recursively (flex/bison inputs,
       the parser support TUs, and parser include headers), depth-capped
       against cycles; nothing under parser/ is ever excluded. */
    std::vector<std::pair<std::string, int>> pending;
    pending.push_back(std::make_pair(std::string("parser"), 0));
    while (!pending.empty()) {
      std::string rel = pending.back().first;
      int depth = pending.back().second;
      pending.pop_back();
      if (depth > 8) {
        *reason = "parser/ nesting too deep at " + rel;
        return false;
      }
      std::vector<std::string> es;
      if (!listDirEntries(root + "/" + rel, &es)) {
        *reason = "cannot list " + rel;
        return false;
      }
      for (const std::string& e : es) {
        if (e.back() == '?') {
          *reason = "unrecognized entry in parser/: " + e;
          return false;
        }
        if (e.back() == '/') {
          pending.push_back(std::make_pair(rel + "/" + e.substr(0, e.size() - 1), depth + 1));
        } else {
          files.push_back(rel + "/" + e);
        }
      }
    }
  }
  std::sort(files.begin(), files.end());
  /* Fingerprint blob: format tag, mode, compiler, allocator deps, then one
     "file <path> <sha256>" line per sorted closure member. */
  std::string blob;
  blob += "closure-format=1\n";
  blob += "mode=pre-final\n";
  blob += "compiler=" + escapeKeyToken(compilerLine) + "\n";
  blob += "deps-sha256=" + depsSha + "\n";
  for (const std::string& f : files) {
    struct stat sb;
    if (stat((root + "/" + f).c_str(), &sb) != 0) {
      *reason = "cannot read " + f;
      return false;
    }
    if (sb.st_mtim.tv_sec > exeSb.st_mtim.tv_sec ||
        (sb.st_mtim.tv_sec == exeSb.st_mtim.tv_sec &&
         sb.st_mtim.tv_nsec > exeSb.st_mtim.tv_nsec)) {
      *reason = "source newer than binary: " + f;
      return false;
    }
    std::string bytes;
    if (!readFileBytes(root + "/" + f, &bytes)) {
      *reason = "cannot read " + f;
      return false;
    }
    blob += "file " + f + " " + sha256Hex(bytes.data(), bytes.size()) + "\n";
  }
  *closureSha = sha256Hex(blob.data(), blob.size());
  *fileCount = files.size();
  return true;
}

/* The only Config fields any pre-Final TU reads are sep_module/sep_aggr
   (AST2Graph.cpp:13-14); every other field is Final-only or non-graph
   (OutputDir/LogLevel/dump flags; dump flags disable the feature). */
std::string buildKeyMaterial(const std::string& firSha, uint64_t firLen,
                             const std::string& closureSha, size_t closureFiles) {
  std::string s;
  auto addLine = [&s](const std::string& line) { s += line; s += '\n'; };
  addLine(kKeyMagic);
  addLine("ckpt-format=" + std::to_string(kBodyFormat));
  addLine("fir-sha256=" + firSha + " fir-bytes=" + std::to_string(firLen));
  addLine("pre-final-closure-sha256=" + closureSha + " files=" +
          std::to_string(closureFiles) + " mode=pre-final");
  addLine(std::string("opt:sep-mod=") + escapeKeyToken(globalConfig.sep_module));
  addLine(std::string("opt:sep-aggr=") + escapeKeyToken(globalConfig.sep_aggr));
  return s;
}

/* --------------------------------------------------------- byte writer */

struct Writer {
  std::string buf;
  void u8(uint8_t v) { buf.push_back((char)v); }
  void u32(uint32_t v) {
    for (int i = 0; i < 4; i++) buf.push_back((char)((v >> (8 * i)) & 0xff));
  }
  void u64(uint64_t v) {
    for (int i = 0; i < 8; i++) buf.push_back((char)((v >> (8 * i)) & 0xff));
  }
  void i32(int32_t v) { u32((uint32_t)v); }
  void str(const std::string& s) {
    u32((uint32_t)s.size());
    buf.append(s);
  }
};

/* --------------------------------------------------------- byte reader */

struct Reader {
  const char* p;
  const char* end;
  bool ok = true;
  Reader(const char* data, size_t n) : p(data), end(data + n) {}
  bool fail() {
    ok = false;
    return false;
  }
  bool need(size_t n) { return ok && (size_t)(end - p) >= n ? true : fail(); }
  uint8_t u8() {
    if (!need(1)) return 0;
    return (uint8_t)*p++;
  }
  uint32_t u32() {
    if (!need(4)) return 0;
    uint32_t v = 0;
    for (int i = 0; i < 4; i++) v |= (uint32_t)(uint8_t)*p++ << (8 * i);
    return v;
  }
  uint64_t u64() {
    if (!need(8)) return 0;
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v |= (uint64_t)(uint8_t)*p++ << (8 * i);
    return v;
  }
  int32_t i32() { return (int32_t)u32(); }
  bool str(std::string* out) {
    if (!need(4)) return false;
    const uint32_t n = u32();
    if (!need(n)) return false;
    out->assign(p, n);
    p += n;
    return true;
  }
};

const uint32_t kNullRef = 0xffffffffu;

std::string mpzToHex(const mpz_t v) {
  char* s = mpz_get_str(nullptr, 16, v);
  std::string out(s);
  void (*freeFn)(void*, size_t);
  mp_get_memory_functions(nullptr, nullptr, &freeFn);
  freeFn(s, strlen(s) + 1);
  return out;
}

bool mpzFromHex(mpz_t v, const std::string& hex) {
  if (hex.empty()) return true;  // stays at the mpz_init value 0
  return mpz_set_str(v, hex.c_str(), 16) == 0;
}

/* ============================== STORE ================================= */

struct StoreCtx {
  graph* g = nullptr;
  std::string why;

  std::unordered_set<Node*> nodes;
  std::unordered_set<SuperNode*> supers;
  /* Entities covered by graph::orderAllNodes (sortedSuper [+ allReset]):
     only those have defined order / orderInSuper; the fields have no default
     initializer, so everything else is normalized to 0 for determinism. */
  std::unordered_set<SuperNode*> orderedSupers;
  std::unordered_set<Node*> orderedNodes;
  std::vector<Node*> nodeOrder;      /* sorted by id */
  std::vector<SuperNode*> superOrder;/* sorted by id */
  std::unordered_map<ENode*, uint32_t> enodeIdx;
  std::unordered_map<ExpTree*, uint32_t> treeIdx;
  std::unordered_map<valInfo*, uint32_t> valIdx;
  std::unordered_map<StmtTree*, uint32_t> stmtIdx;
  std::vector<ENode*> enodeOrder;
  std::vector<ExpTree*> treeOrder;
  std::vector<valInfo*> valOrder;
  std::vector<StmtTree*> stmtOrder;

  bool fail(const std::string& reason) {
    if (why.empty()) why = reason;
    return false;
  }

  void addNode(Node* n, std::vector<Node*>& work) {
    if (n && nodes.insert(n).second) work.push_back(n);
  }
  void addSuper(SuperNode* s, std::vector<SuperNode*>& work) {
    if (s && supers.insert(s).second) work.push_back(s);
  }

  /* Node-pointer closure of an ENode DAG (membership only; index-free). */
  void discoverEnodeNodes(ENode* root, std::vector<Node*>& work,
                          std::unordered_set<ENode*>& visited) {
    std::vector<ENode*> stack;
    if (root) stack.push_back(root);
    while (!stack.empty()) {
      ENode* e = stack.back();
      stack.pop_back();
      if (!visited.insert(e).second) continue;
      if (e->nodePtr) addNode(e->nodePtr, work);
      if (e->memoryNode) addNode(e->memoryNode, work);
      for (ENode* c : e->child) {
        if (c) stack.push_back(c);
      }
    }
  }
  void discoverTreeNodes(ExpTree* t, std::vector<Node*>& work,
                         std::unordered_set<ENode*>& visited) {
    if (!t) return;
    discoverEnodeNodes(t->getRoot(), work, visited);
    discoverEnodeNodes(t->getlval(), work, visited);
  }
  void discoverStmtNodes(StmtTree* st, std::vector<Node*>& work,
                         std::unordered_set<ENode*>& visited) {
    if (!st || !st->root) return;
    std::vector<StmtNode*> stack;
    stack.push_back(st->root);
    while (!stack.empty()) {
      StmtNode* sn = stack.back();
      stack.pop_back();
      if (sn->belong) addNode(sn->belong, work);
      if (sn->isENode) {
        if (sn->enode) discoverEnodeNodes(sn->enode, work, visited);
      } else if (sn->tree) {
        discoverTreeNodes(sn->tree, work, visited);
      }
      for (StmtNode* c : sn->child) stack.push_back(c);
    }
  }

  /* Reachability fixpoint over every Node and SuperNode pointer the Final
     stage or the serializer itself can follow. */
  bool discoverUniverse() {
    std::vector<Node*> nodeWork;
    std::vector<SuperNode*> superWork;
    auto seedNode = [&](Node* n) { addNode(n, nodeWork); };
    for (Node* n : g->allNodes) seedNode(n);
    for (Node* n : g->input) seedNode(n);
    for (Node* n : g->output) seedNode(n);
    for (Node* n : g->regsrc) seedNode(n);
    for (Node* n : g->sorted) seedNode(n);
    for (Node* n : g->memory) seedNode(n);
    for (Node* n : g->external) seedNode(n);
    for (Node* n : g->specialNodes) seedNode(n);
    for (Node* n : g->halfConstantArray) seedNode(n);
    auto seedSuper = [&](SuperNode* s) {
      if (s) {
        addSuper(s, superWork);
        for (Node* m : s->member) addNode(m, nodeWork);
        if (s->resetNode) addNode(s->resetNode, nodeWork);
      }
    };
    for (SuperNode* s : g->supersrc) seedSuper(s);
    for (SuperNode* s : g->sortedSuper) seedSuper(s);
    for (SuperNode* s : g->allReset) seedSuper(s);

    std::unordered_set<ENode*> visited;
    size_t ni = 0, si = 0;
    while (ni < nodeWork.size() || si < superWork.size()) {
      while (ni < nodeWork.size()) {
        Node* n = nodeWork[ni++];
        if (n->super) addSuper(n->super, superWork);
        for (Node* x : n->next) addNode(x, nodeWork);
        for (Node* x : n->prev) addNode(x, nodeWork);
        for (Node* x : n->depPrev) addNode(x, nodeWork);
        for (Node* x : n->depNext) addNode(x, nodeWork);
        for (Node* x : n->member) addNode(x, nodeWork);
        if (n->parent) addNode(n->parent, nodeWork);
        if (n->regNext) addNode(n->regNext, nodeWork);
        if (n->clock) addNode(n->clock, nodeWork);
        for (ExpTree* t : n->assignTree) discoverTreeNodes(t, nodeWork, visited);
        discoverTreeNodes(n->resetTree, nodeWork, visited);
      }
      while (si < superWork.size()) {
        SuperNode* s = superWork[si++];
        for (SuperNode* x : s->prev) addSuper(x, superWork);
        for (SuperNode* x : s->next) addSuper(x, superWork);
        for (SuperNode* x : s->depPrev) addSuper(x, superWork);
        for (SuperNode* x : s->depNext) addSuper(x, superWork);
        for (Node* m : s->member) addNode(m, nodeWork);
        if (s->resetNode) addNode(s->resetNode, nodeWork);
        discoverStmtNodes(s->stmtTree, nodeWork, visited);
        /* s->insts: populated only by Final passes (empty at the boundary);
           node pointers there are NOT dereferenced here (the string-only
           InstInfo ctor leaves them uninitialized); membership is enforced
           at emission time instead. */
      }
    }
    nodeOrder.assign(nodes.begin(), nodes.end());
    std::sort(nodeOrder.begin(), nodeOrder.end(), [](Node* a, Node* b) { return a->id < b->id; });
    superOrder.assign(supers.begin(), supers.end());
    std::sort(superOrder.begin(), superOrder.end(),
              [](SuperNode* a, SuperNode* b) { return a->id < b->id; });
    for (size_t i = 1; i < nodeOrder.size(); i++) {
      if (nodeOrder[i]->id == nodeOrder[i - 1]->id) {
        return fail("duplicate node id " + std::to_string(nodeOrder[i]->id));
      }
    }
    for (size_t i = 1; i < superOrder.size(); i++) {
      if (superOrder[i]->id == superOrder[i - 1]->id) {
        return fail("duplicate superNode id " + std::to_string(superOrder[i]->id));
      }
    }
    return true;
  }

  /* ---- canonical interning (no pointer-ordered iteration anywhere) ---- */
  void internVal(valInfo* v) {
    if (!v) return;
    auto it = valIdx.find(v);
    if (it != valIdx.end()) return;
    valIdx[v] = (uint32_t)valOrder.size();
    valOrder.push_back(v);
    for (valInfo* m : v->memberInfo) internVal(m);
  }
  void internEnode(ENode* e) {
    if (!e) return;
    auto it = enodeIdx.find(e);
    if (it != enodeIdx.end()) return;
    enodeIdx[e] = (uint32_t)enodeOrder.size();
    enodeOrder.push_back(e);
    for (ENode* c : e->child) internEnode(c);
  }
  void internTree(ExpTree* t) {
    if (!t) return;
    auto it = treeIdx.find(t);
    if (it != treeIdx.end()) return;
    treeIdx[t] = (uint32_t)treeOrder.size();
    treeOrder.push_back(t);
    internEnode(t->getRoot());
    internEnode(t->getlval());
  }
  bool internStmtNode(StmtNode* sn) {
    if (!sn) return true;
    if (sn->isENode) {
      internEnode(sn->enode);
    } else if (sn->tree) {
      if (treeIdx.find(sn->tree) == treeIdx.end()) {
        return fail("StmtNode references an ExpTree not reachable from any node");
      }
    }
    for (StmtNode* c : sn->child) {
      if (!internStmtNode(c)) return false;
    }
    return true;
  }
  void internStmtTree(StmtTree* st) {
    if (!st) return;
    auto it = stmtIdx.find(st);
    if (it != stmtIdx.end()) return;
    stmtIdx[st] = (uint32_t)stmtOrder.size();
    stmtOrder.push_back(st);
    internStmtNode(st->root);
  }
  bool internCanonical() {
    for (Node* n : nodeOrder) {
      for (ExpTree* t : n->assignTree) internTree(t);
      internTree(n->resetTree);
    }
    for (SuperNode* s : superOrder) internStmtTree(s->stmtTree);
    /* valInfo sharing (constantNode's consMap hands the same valInfo to a
       node and to ENodes of its trees) is preserved by interning. */
    for (Node* n : nodeOrder) internVal(n->computeInfo);
    for (ENode* e : enodeOrder) internVal(e->computeInfo);
    for (StmtTree* st : stmtOrder) {
      std::vector<StmtNode*> stack;
      if (st->root) stack.push_back(st->root);
      while (!stack.empty()) {
        StmtNode* sn = stack.back();
        stack.pop_back();
        internVal(sn->computeInfo);
        for (StmtNode* c : sn->child) stack.push_back(c);
      }
    }
    return true;
  }

  /* ---------------------------- references ---------------------------- */
  int32_t nid(Node* n) {
    if (!n) return -1;
    if (nodes.find(n) == nodes.end()) {
      fail("node " + n->name + " is outside the serializable universe");
      return -1;
    }
    return n->id;
  }
  int32_t sid(SuperNode* s) {
    if (!s) return -1;
    if (supers.find(s) == supers.end()) {
      fail("superNode outside the serializable universe");
      return -1;
    }
    return s->id;
  }
  uint32_t enodeRef(ENode* e) {
    if (!e) return kNullRef;
    auto it = enodeIdx.find(e);
    if (it == enodeIdx.end()) {
      fail("ENode outside the interned table");
      return kNullRef;
    }
    return it->second;
  }
  uint32_t treeRef(ExpTree* t) {
    if (!t) return kNullRef;
    auto it = treeIdx.find(t);
    if (it == treeIdx.end()) {
      fail("ExpTree outside the interned table");
      return kNullRef;
    }
    return it->second;
  }
  uint32_t valRef(valInfo* v) {
    if (!v) return kNullRef;
    auto it = valIdx.find(v);
    if (it == valIdx.end()) {
      fail("valInfo outside the interned table");
      return kNullRef;
    }
    return it->second;
  }

  template <class Container, class Id>
  void writeIdList(Writer& w, const Container& items, Id idOf) {
    std::vector<int32_t> ids;
    ids.reserve(items.size());
    for (auto* t : items) ids.push_back(idOf(t));
    std::sort(ids.begin(), ids.end());
    w.u32((uint32_t)ids.size());
    for (int32_t id : ids) w.i32(id);
  }
  void writeValInfo(Writer& w, valInfo* v) {
    w.u8((uint8_t)v->status);
    w.u8((uint8_t)v->type);
    w.u8(v->sign ? 1 : 0);
    w.u8(v->sameConstant ? 1 : 0);
    w.u8(v->directUpdate ? 1 : 0);
    w.i32(v->opNum);
    w.i32(v->width);
    w.i32(v->typeWidth);
    w.i32(v->consLength);
    w.i32(v->beg);
    w.i32(v->end);
    w.u32(v->arrayRefLen);
    w.str(v->valStr);
    w.str(mpzToHex(v->consVal));
    w.str(mpzToHex(v->assignmentCons));
    w.u32((uint32_t)v->memberInfo.size());
    for (valInfo* m : v->memberInfo) w.u32(valRef(m));
  }

  void writeEnode(Writer& w, ENode* e) {
    w.u8((uint8_t)e->opType);  /* OPType has < 256 values by construction */
    w.i32(e->width);
    w.i32(e->usedBit);
    w.i32(e->id);
    w.u8(e->sign ? 1 : 0);
    w.u8(e->isClock ? 1 : 0);
    w.u8((uint8_t)e->reset);
    w.i32(nid(e->nodePtr));
    w.i32(nid(e->memoryNode));
    w.u32(valRef(e->computeInfo));
    w.u32((uint32_t)e->child.size());
    for (ENode* c : e->child) w.u32(enodeRef(c));
    w.u32((uint32_t)e->values.size());
    for (int x : e->values) w.i32(x);
    w.str(e->strVal);
  }

  void writeTree(Writer& w, ExpTree* t) {
    if (!t->getRoot()) {
      fail("ExpTree with null root");
      return;
    }
    w.u32(enodeRef(t->getRoot()));
    w.u32(enodeRef(t->getlval()));
  }

  void writeStmtNode(Writer& w, StmtNode* sn) {
    w.u8(sn->isENode ? 1 : 0);
    w.u8((uint8_t)sn->type);
    w.u32(enodeRef(sn->enode));
    w.u32(treeRef(sn->tree));
    w.i32(nid(sn->belong));
    w.u32(valRef(sn->computeInfo));
    w.u32((uint32_t)sn->child.size());
    for (StmtNode* c : sn->child) writeStmtNode(w, c);
  }

  void writeSuper(Writer& w, SuperNode* s) {
    w.i32(s->id);
    w.i32(orderedSupers.count(s) ? s->order : 0);
    w.i32(s->cppId);
    w.i32(s->superType);
    w.i32(nid(s->resetNode));
    auto sit = stmtIdx.find(s->stmtTree);
    w.u32(s->stmtTree && sit != stmtIdx.end() ? sit->second : kNullRef);
    auto sidOf = [this](SuperNode* x) { return sid(x); };
    writeIdList(w, s->prev, sidOf);
    writeIdList(w, s->next, sidOf);
    writeIdList(w, s->depPrev, sidOf);
    writeIdList(w, s->depNext, sidOf);
    w.u32((uint32_t)s->member.size());
    for (Node* m : s->member) w.i32(nid(m));
    w.u32((uint32_t)s->insts.size());
    for (const InstInfo& inst : s->insts) {
      w.i32(inst.infoType);
      w.str(inst.inst);
      /* Fail-closed: the string-only InstInfo ctor leaves node unset;
         anything not provably a live node aborts the store. insts is
         populated only by Final passes, so this is empty at the boundary. */
      if (inst.node == nullptr || nodes.find(inst.node) == nodes.end()) {
        fail("InstInfo with a node outside the serializable universe");
        return;
      }
      w.i32(inst.node->id);
    }
  }

  void writeNode(Writer& w, Node* n) {
    w.str(n->name);
    w.str(n->extraInfo);
    w.i32(n->id);
    w.i32(n->type);
    w.i32(n->width);
    w.i32(n->usedBit);
    w.i32(n->status);
    w.i32(orderedNodes.count(n) ? n->order : 0);
    w.i32(orderedNodes.count(n) ? n->orderInSuper : 0);
    w.i32(n->lineno);
    w.u8(n->sign ? 1 : 0);
    w.u32((uint32_t)n->dimension.size());
    for (int d : n->dimension) w.i32(d);
    auto nidOf = [this](Node* x) { return nid(x); };
    writeIdList(w, n->next, nidOf);
    writeIdList(w, n->prev, nidOf);
    writeIdList(w, n->depPrev, nidOf);
    writeIdList(w, n->depNext, nidOf);
    w.u32((uint32_t)n->assignTree.size());
    for (ExpTree* t : n->assignTree) w.u32(treeRef(t));
    w.u32(treeRef(n->resetTree));
    w.i32(sid(n->super));
    w.u32((uint32_t)n->member.size());
    for (Node* m : n->member) w.i32(nid(m));
    w.i32(nid(n->parent));
    w.i32(nid(n->regNext));
    w.i32(nid(n->clock));
    w.u32((uint32_t)n->params.size());
    for (const std::pair<bool, std::string>& p : n->params) {
      w.u8(p.first ? 1 : 0);
      w.str(p.second);
    }
    /* rlatency/wlatency/depth have no default initializer: only NODE_MEMORY
       values are ever read; store 0 for everything else (deterministic). */
    w.i32(n->type == NODE_MEMORY ? n->rlatency : 0);
    w.i32(n->type == NODE_MEMORY ? n->wlatency : 0);
    w.i32(n->type == NODE_MEMORY ? n->depth : 0);
    w.u8(n->regSplit ? 1 : 0);
    w.u8(n->isClock ? 1 : 0);
    w.u8(n->inAggr ? 1 : 0);
    w.u8(n->nodeIsRoot ? 1 : 0);
    w.i32(n->reset);
    w.i32(n->asReset);
    w.u64(n->whenDepth);
    w.u32(valRef(n->computeInfo));
    w.u32((uint32_t)n->nextActiveId.size());
    for (int id : n->nextActiveId) w.i32(id);
    w.u32((uint32_t)n->nextNeedActivate.size());
    for (int id : n->nextNeedActivate) w.i32(id);
  }

  void writeNodeList(Writer& w, const std::vector<Node*>& list) {
    w.u32((uint32_t)list.size());
    for (Node* n : list) w.i32(nid(n));
  }
  void writeSuperList(Writer& w, const std::vector<SuperNode*>& list) {
    w.u32((uint32_t)list.size());
    for (SuperNode* s : list) w.i32(sid(s));
  }

  bool emit(std::string* body) {
    if (!discoverUniverse() || !internCanonical()) return false;
    auto markOrdered = [&](SuperNode* s) {
      if (!s) return;
      orderedSupers.insert(s);
      for (Node* m : s->member) orderedNodes.insert(m);
    };
    for (SuperNode* s : g->sortedSuper) markOrdered(s);
    for (SuperNode* s : g->allReset) markOrdered(s);
    Writer w;
    w.buf.reserve(1u << 26);
    char magic[14];
    memset(magic, 0, sizeof magic);
    memcpy(magic, kBodyMagic, strlen(kBodyMagic));
    w.buf.append(magic, sizeof magic);
    w.u32(kBodyFormat);
    w.u32((uint32_t)nodeOrder.size());
    w.u32((uint32_t)superOrder.size());
    w.u32((uint32_t)treeOrder.size());
    w.u32((uint32_t)enodeOrder.size());
    w.u32((uint32_t)valOrder.size());
    w.u32((uint32_t)stmtOrder.size());
    w.i32(Node::counterValue());
    w.i32(SuperNode::counterValue());
    w.i32(ENode::counterValue());
    w.u64(0);  /* reserved */
    if (!why.empty()) return false;
    for (valInfo* v : valOrder) writeValInfo(w, v);
    for (ENode* e : enodeOrder) writeEnode(w, e);
    for (ExpTree* t : treeOrder) writeTree(w, t);
    for (StmtTree* st : stmtOrder) {
      if (!st->root) {
        fail("StmtTree with null root");
        return false;
      }
      writeStmtNode(w, st->root);
    }
    for (SuperNode* s : superOrder) writeSuper(w, s);
    for (Node* n : nodeOrder) writeNode(w, n);
    w.str(g->name);
    w.i32(g->nodeNum);
    w.u32((uint32_t)g->extDecl.size());
    for (const std::string& d : g->extDecl) w.str(d);
    writeNodeList(w, g->allNodes);
    writeNodeList(w, g->input);
    writeNodeList(w, g->output);
    writeNodeList(w, g->regsrc);
    writeNodeList(w, g->sorted);
    writeNodeList(w, g->memory);
    writeNodeList(w, g->external);
    writeNodeList(w, g->specialNodes);
    /* set<Node*> iterates in pointer order: sort by id for determinism. */
    {
      std::vector<Node*> half(g->halfConstantArray.begin(), g->halfConstantArray.end());
      std::sort(half.begin(), half.end(), [](Node* a, Node* b) { return a->id < b->id; });
      writeNodeList(w, half);
    }
    writeSuperList(w, g->supersrc);
    writeSuperList(w, g->sortedSuper);
    writeSuperList(w, g->allReset);
    if (!why.empty()) return false;
    *body = std::move(w.buf);
    return true;
  }
};

bool serializeGraph(graph* g, std::string* body, std::string* why) {
  StoreCtx ctx;
  ctx.g = g;
  if (!ctx.emit(body)) {
    *why = ctx.why.empty() ? "serialization failed" : ctx.why;
    return false;
  }
  return true;
}

/* ============================== RESTORE =============================== */

struct ParsedVal {
  uint8_t status = 0, type = 0, sign = 0, sameConstant = 0, directUpdate = 0;
  int32_t opNum = 0, width = 0, typeWidth = 0, consLength = 0, beg = 0, end = 0;
  uint32_t arrayRefLen = 0;
  std::string valStr, consHex, assignHex;
  std::vector<uint32_t> member;
};
struct ParsedEnode {
  uint8_t opType = 0, sign = 0, isClock = 0, reset = 0;
  int32_t width = 0, usedBit = 0, id = 0, nodePtr = -1, memoryNode = -1;
  uint32_t computeInfo = kNullRef;
  std::vector<uint32_t> child;
  std::vector<int32_t> values;
  std::string strVal;
};
struct ParsedTree {
  uint32_t root = kNullRef, lvalue = kNullRef;
};
struct ParsedStmt {
  uint8_t isENode = 0, type = 0;
  uint32_t enode = kNullRef, tree = kNullRef, computeInfo = kNullRef;
  int32_t belong = -1;
  std::vector<ParsedStmt> child;
};
struct ParsedInst {
  int32_t infoType = 0, node = -1;
  std::string inst;
};
struct ParsedSuper {
  int32_t id = 0, order = 0, cppId = 0, superType = 0, resetNode = -1;
  uint32_t stmtTree = kNullRef;
  std::vector<int32_t> prev, next, depPrev, depNext, member;
  std::vector<ParsedInst> insts;
};
struct ParsedNode {
  std::string name, extraInfo;
  int32_t id = -1, type = 0, width = 0, usedBit = 0, status = 0, order = 0;
  int32_t orderInSuper = 0, lineno = 0, super = -1, parent = -1, regNext = -1, clock = -1;
  uint8_t sign = 0, regSplit = 0, isClock = 0, inAggr = 0, nodeIsRoot = 0;
  int32_t reset = 0, asReset = 0, rlatency = 0, wlatency = 0, depth = 0;
  uint64_t whenDepth = 0;
  uint32_t resetTree = kNullRef, computeInfo = kNullRef;
  std::vector<int32_t> dimension, next, prev, depPrev, depNext, member;
  std::vector<int32_t> nextActiveId, nextNeedActivate;
  std::vector<uint32_t> assignTree;
  std::vector<std::pair<uint8_t, std::string>> params;
};
struct ParsedGraph {
  std::string name;
  int32_t nodeNum = 0;
  std::vector<std::string> extDecl;
  std::vector<int32_t> allNodes, input, output, regsrc, sorted, memory, external;
  std::vector<int32_t> specialNodes, halfConstantArray;
  std::vector<int32_t> supersrc, sortedSuper, allReset;
};

struct BuildCtx {
  std::unordered_map<int32_t, Node*> byNid;
  std::unordered_map<int32_t, SuperNode*> bySid;
  std::vector<valInfo*> vals;
  std::vector<ENode*> enodes;
  std::vector<ExpTree*> trees;
  std::vector<StmtTree*> stmts;
  std::string why;

  bool fail(const std::string& reason) {
    if (why.empty()) why = reason;
    return false;
  }
  Node* node(int32_t id) {
    if (id < 0) return nullptr;
    auto it = byNid.find(id);
    if (it == byNid.end()) {
      fail("reference to unknown node id " + std::to_string(id));
      return nullptr;
    }
    return it->second;
  }
  SuperNode* super(int32_t id) {
    if (id < 0) return nullptr;
    auto it = bySid.find(id);
    if (it == bySid.end()) {
      fail("reference to unknown superNode id " + std::to_string(id));
      return nullptr;
    }
    return it->second;
  }
  ENode* enode(uint32_t ref) {
    if (ref == kNullRef) return nullptr;
    if (ref >= enodes.size()) {
      fail("ENode reference out of range");
      return nullptr;
    }
    return enodes[ref];
  }
  ExpTree* tree(uint32_t ref) {
    if (ref == kNullRef) return nullptr;
    if (ref >= trees.size()) {
      fail("ExpTree reference out of range");
      return nullptr;
    }
    return trees[ref];
  }
  valInfo* val(uint32_t ref) {
    if (ref == kNullRef) return nullptr;
    if (ref >= vals.size()) {
      fail("valInfo reference out of range");
      return nullptr;
    }
    return vals[ref];
  }
};

bool readIdList(Reader& r, int32_t maxId, std::vector<int32_t>* out, std::string* why) {
  const uint32_t n = r.u32();
  if (!r.ok) return false;
  out->clear();
  out->reserve(n);
  for (uint32_t i = 0; i < n; i++) {
    const int32_t id = r.i32();
    if (!r.ok) return false;
    if (id < 0 || (maxId >= 0 && id > maxId)) {
      *why = "node id reference out of range";
      return false;
    }
    out->push_back(id);
  }
  return true;
}

bool readSidList(Reader& r, int32_t maxId, std::vector<int32_t>* out, std::string* why) {
  const uint32_t n = r.u32();
  if (!r.ok) return false;
  out->clear();
  out->reserve(n);
  for (uint32_t i = 0; i < n; i++) {
    const int32_t id = r.i32();
    if (!r.ok) return false;
    if (id < 0 || (maxId >= 0 && id > maxId)) {
      *why = "superNode id reference out of range";
      return false;
    }
    out->push_back(id);
  }
  return true;
}

bool readRefList(Reader& r, size_t max, std::vector<uint32_t>* out) {
  const uint32_t n = r.u32();
  if (!r.ok) return false;
  if (n > max) return false;
  out->clear();
  out->reserve(n);
  for (uint32_t i = 0; i < n; i++) {
    const uint32_t ref = r.u32();
    if (!r.ok) return false;
    out->push_back(ref);
  }
  return true;
}

bool parseVal(Reader& r, size_t valCount, ParsedVal* v, std::string* why) {
  v->status = r.u8();
  v->type = r.u8();
  v->sign = r.u8();
  v->sameConstant = r.u8();
  v->directUpdate = r.u8();
  v->opNum = r.i32();
  v->width = r.i32();
  v->typeWidth = r.i32();
  v->consLength = r.i32();
  v->beg = r.i32();
  v->end = r.i32();
  v->arrayRefLen = r.u32();
  if (!r.ok || !r.str(&v->valStr) || !r.str(&v->consHex) || !r.str(&v->assignHex)) {
    *why = "truncated valInfo record";
    return false;
  }
  if (v->status > VAL_EXT || v->type > TYPE_STMT) {
    *why = "valInfo enum out of range";
    return false;
  }
  if (!readRefList(r, valCount, &v->member)) {
    *why = "valInfo member reference out of range";
    return false;
  }
  return true;
}

bool parseEnode(Reader& r, size_t maxRef, ParsedEnode* e, std::string* why) {
  e->opType = r.u8();
  e->width = r.i32();
  e->usedBit = r.i32();
  e->id = r.i32();
  e->sign = r.u8();
  e->isClock = r.u8();
  e->reset = r.u8();
  e->nodePtr = r.i32();
  e->memoryNode = r.i32();
  e->computeInfo = r.u32();
  if (!r.ok) {
    *why = "truncated ENode record";
    return false;
  }
  if (e->reset > ZERO_RESET) {
    *why = "ENode reset enum out of range";
    return false;
  }
  if (!readRefList(r, maxRef, &e->child)) {
    *why = "ENode child reference out of range";
    return false;
  }
  const uint32_t vn = r.u32();
  if (!r.ok || vn > (uint32_t)(r.end - r.p) / 4) {
    *why = "ENode values out of range";
    return false;
  }
  e->values.reserve(vn);
  for (uint32_t i = 0; i < vn; i++) e->values.push_back(r.i32());
  if (!r.ok || !r.str(&e->strVal)) {
    *why = "truncated ENode record";
    return false;
  }
  return true;
}

bool parseStmt(Reader& r, int depth, size_t maxEnode, size_t maxTree, ParsedStmt* st,
               std::string* why) {
  if (depth > 10000) {
    *why = "StmtTree nesting too deep";
    return false;
  }
  st->isENode = r.u8();
  st->type = r.u8();
  st->enode = r.u32();
  st->tree = r.u32();
  st->belong = r.i32();
  st->computeInfo = r.u32();
  if (!r.ok) {
    *why = "truncated StmtNode record";
    return false;
  }
  if (st->enode != kNullRef && st->enode >= maxEnode) {
    *why = "StmtNode ENode reference out of range";
    return false;
  }
  if (st->tree != kNullRef && st->tree >= maxTree) {
    *why = "StmtNode tree reference out of range";
    return false;
  }
  const uint32_t n = r.u32();
  if (!r.ok || n > (uint32_t)(r.end - r.p)) {
    *why = "StmtNode child count out of range";
    return false;
  }
  st->child.resize(n);
  for (uint32_t i = 0; i < n; i++) {
    if (!parseStmt(r, depth + 1, maxEnode, maxTree, &st->child[i], why)) return false;
  }
  return true;
}

bool parseSuper(Reader& r, ParsedSuper* s, std::string* why) {
  s->id = r.i32();
  s->order = r.i32();
  s->cppId = r.i32();
  s->superType = r.i32();
  s->resetNode = r.i32();
  s->stmtTree = r.u32();
  if (!r.ok) {
    *why = "truncated SuperNode record";
    return false;
  }
  if (s->superType < SUPER_VALID || s->superType > SUPER_UPDATE_REG) {
    *why = "SuperType out of range";
    return false;
  }
  if (!readSidList(r, -1, &s->prev, why) || !readSidList(r, -1, &s->next, why) ||
      !readSidList(r, -1, &s->depPrev, why) || !readSidList(r, -1, &s->depNext, why)) {
    return false;
  }
  const uint32_t mn = r.u32();
  if (!r.ok || mn > (uint32_t)(r.end - r.p) / 4) {
    *why = "SuperNode member count out of range";
    return false;
  }
  s->member.reserve(mn);
  for (uint32_t i = 0; i < mn; i++) s->member.push_back(r.i32());
  const uint32_t in = r.u32();
  if (!r.ok || in > (uint32_t)(r.end - r.p) / 6) {
    *why = "SuperNode inst count out of range";
    return false;
  }
  s->insts.resize(in);
  for (uint32_t i = 0; i < in; i++) {
    ParsedInst& inst = s->insts[i];
    inst.infoType = r.i32();
    if (!r.ok || !r.str(&inst.inst)) {
      *why = "truncated InstInfo record";
      return false;
    }
    inst.node = r.i32();
    if (!r.ok) {
      *why = "truncated InstInfo record";
      return false;
    }
  }
  return r.ok;
}

bool parseNode(Reader& r, ParsedNode* n, std::string* why) {
  if (!r.str(&n->name) || !r.str(&n->extraInfo)) {
    *why = "truncated Node record";
    return false;
  }
  n->id = r.i32();
  n->type = r.i32();
  n->width = r.i32();
  n->usedBit = r.i32();
  n->status = r.i32();
  n->order = r.i32();
  n->orderInSuper = r.i32();
  n->lineno = r.i32();
  n->sign = r.u8();
  const uint32_t dn = r.u32();
  if (!r.ok || dn > (uint32_t)(r.end - r.p) / 4) {
    *why = "Node dimension count out of range";
    return false;
  }
  n->dimension.reserve(dn);
  for (uint32_t i = 0; i < dn; i++) n->dimension.push_back(r.i32());
  if (!readIdList(r, -1, &n->next, why) || !readIdList(r, -1, &n->prev, why) ||
      !readIdList(r, -1, &n->depPrev, why) || !readIdList(r, -1, &n->depNext, why)) {
    return false;
  }
  const uint32_t an = r.u32();
  if (!r.ok || an > (uint32_t)(r.end - r.p) / 4) {
    *why = "Node assignTree count out of range";
    return false;
  }
  n->assignTree.reserve(an);
  for (uint32_t i = 0; i < an; i++) n->assignTree.push_back(r.u32());
  n->resetTree = r.u32();
  n->super = r.i32();
  const uint32_t mn = r.u32();
  if (!r.ok || mn > (uint32_t)(r.end - r.p) / 4) {
    *why = "Node member count out of range";
    return false;
  }
  n->member.reserve(mn);
  for (uint32_t i = 0; i < mn; i++) n->member.push_back(r.i32());
  n->parent = r.i32();
  n->regNext = r.i32();
  n->clock = r.i32();
  const uint32_t pn = r.u32();
  if (!r.ok || pn > (uint32_t)(r.end - r.p)) {
    *why = "Node params count out of range";
    return false;
  }
  n->params.resize(pn);
  for (uint32_t i = 0; i < pn; i++) {
    n->params[i].first = r.u8();
    if (!r.ok || !r.str(&n->params[i].second)) {
      *why = "truncated Node param record";
      return false;
    }
  }
  n->rlatency = r.i32();
  n->wlatency = r.i32();
  n->depth = r.i32();
  n->regSplit = r.u8();
  n->isClock = r.u8();
  n->inAggr = r.u8();
  n->nodeIsRoot = r.u8();
  n->reset = r.i32();
  n->asReset = r.i32();
  n->whenDepth = r.u64();
  n->computeInfo = r.u32();
  if (!r.ok) {
    *why = "truncated Node record";
    return false;
  }
  if (n->type <= NODE_INVALID || n->type > NODE_EXT ||
      n->status < VALID_NODE || n->status > EMPTY_REG ||
      n->reset < UNCERTAIN || n->reset > ZERO_RESET ||
      n->asReset < EMPTY || n->asReset > NODE_ALL_RESET) {
    *why = "Node enum out of range";
    return false;
  }
  if (!readIdList(r, -1, &n->nextActiveId, why)) return false;
  return readIdList(r, -1, &n->nextNeedActivate, why);
}

bool readNodeList(Reader& r, std::vector<int32_t>* out, std::string* why) {
  return readIdList(r, -1, out, why);
}

/* Parses the verified body and rebuilds the graph. Every anomaly returns
   false with *why set; the caller falls back to the full pipeline. */
bool deserializeGraph(const std::string& body, graph** out, std::string* why) {
  Reader r(body.data(), body.size());
  char magic[14];
  if (!r.need(sizeof magic)) {
    *why = "body too short";
    return false;
  }
  memcpy(magic, r.p, sizeof magic);
  r.p += sizeof magic;
  if (memcmp(magic, kBodyMagic, strlen(kBodyMagic)) != 0) {
    *why = "bad body magic";
    return false;
  }
  const uint32_t format = r.u32();
  const uint32_t nodeCount = r.u32();
  const uint32_t superCount = r.u32();
  const uint32_t treeCount = r.u32();
  const uint32_t enodeCount = r.u32();
  const uint32_t valCount = r.u32();
  const uint32_t stmtCount = r.u32();
  const int32_t nodeCounter = r.i32();
  const int32_t superCounter = r.i32();
  const int32_t enodeCounter = r.i32();
  const uint64_t reserved = r.u64();
  if (!r.ok) {
    *why = "truncated header";
    return false;
  }
  if (format != kBodyFormat || reserved != 0) {
    *why = "unsupported body format";
    return false;
  }
  if ((uint64_t)nodeCount + superCount + treeCount + enodeCount + valCount + stmtCount >
      (uint64_t)(r.end - r.p)) {
    *why = "header counts exceed body size";
    return false;
  }
  if (nodeCounter < 0 || superCounter < 0 || enodeCounter < 0) {
    *why = "negative id counter";
    return false;
  }

  std::vector<ParsedVal> vals(valCount);
  for (uint32_t i = 0; i < valCount; i++) {
    if (!parseVal(r, valCount, &vals[i], why)) return false;
  }
  std::vector<ParsedEnode> enodes(enodeCount);
  for (uint32_t i = 0; i < enodeCount; i++) {
    if (!parseEnode(r, enodeCount, &enodes[i], why)) return false;
  }
  std::vector<ParsedTree> trees(treeCount);
  for (uint32_t i = 0; i < treeCount; i++) {
    trees[i].root = r.u32();
    trees[i].lvalue = r.u32();
    if (!r.ok) {
      *why = "truncated ExpTree record";
      return false;
    }
  }
  std::vector<ParsedStmt> stmts(stmtCount);
  for (uint32_t i = 0; i < stmtCount; i++) {
    if (!parseStmt(r, 0, enodeCount, treeCount, &stmts[i], why)) return false;
  }
  std::vector<ParsedSuper> supers(superCount);
  for (uint32_t i = 0; i < superCount; i++) {
    if (!parseSuper(r, &supers[i], why)) return false;
  }
  std::vector<ParsedNode> nodes(nodeCount);
  for (uint32_t i = 0; i < nodeCount; i++) {
    if (!parseNode(r, &nodes[i], why)) return false;
  }
  ParsedGraph pg;
  if (!r.str(&pg.name)) {
    *why = "truncated graph record";
    return false;
  }
  pg.nodeNum = r.i32();
  const uint32_t dn = r.u32();
  if (!r.ok || dn > (uint32_t)(r.end - r.p)) {
    *why = "extDecl count out of range";
    return false;
  }
  pg.extDecl.resize(dn);
  for (uint32_t i = 0; i < dn; i++) {
    if (!r.str(&pg.extDecl[i])) {
      *why = "truncated extDecl entry";
      return false;
    }
  }
  if (!readNodeList(r, &pg.allNodes, why) || !readNodeList(r, &pg.input, why) ||
      !readNodeList(r, &pg.output, why) || !readNodeList(r, &pg.regsrc, why) ||
      !readNodeList(r, &pg.sorted, why) || !readNodeList(r, &pg.memory, why) ||
      !readNodeList(r, &pg.external, why) || !readNodeList(r, &pg.specialNodes, why) ||
      !readNodeList(r, &pg.halfConstantArray, why) ||
      !readSidList(r, -1, &pg.supersrc, why) || !readSidList(r, -1, &pg.sortedSuper, why) ||
      !readSidList(r, -1, &pg.allReset, why)) {
    return false;
  }
  if (r.p != r.end) {
    *why = "trailing bytes after graph record";
    return false;
  }

  /* ------------------------------ build ------------------------------ */
  BuildCtx b;
  b.vals.reserve(valCount);
  for (uint32_t i = 0; i < valCount; i++) {
    const ParsedVal& v = vals[i];
    valInfo* obj = new valInfo();
    obj->status = (valStatus)v.status;
    obj->type = (valType)v.type;
    obj->sign = v.sign != 0;
    obj->sameConstant = v.sameConstant != 0;
    obj->directUpdate = v.directUpdate != 0;
    obj->opNum = v.opNum;
    obj->width = v.width;
    obj->typeWidth = v.typeWidth;
    obj->consLength = v.consLength;
    obj->beg = v.beg;
    obj->end = v.end;
    obj->arrayRefLen = v.arrayRefLen;
    obj->valStr = v.valStr;
    if (!mpzFromHex(obj->consVal, v.consHex) ||
        !mpzFromHex(obj->assignmentCons, v.assignHex)) {
      *why = "unparseable mpz constant";
      return false;
    }
    b.vals.push_back(obj);
  }
  for (uint32_t i = 0; i < valCount; i++) {
    valInfo* obj = b.vals[i];
    for (uint32_t ref : vals[i].member) {
      valInfo* m = b.val(ref);
      if (b.why.empty() == false) {
        *why = b.why;
        return false;
      }
      obj->memberInfo.push_back(m);
    }
  }
  b.enodes.reserve(enodeCount);
  for (uint32_t i = 0; i < enodeCount; i++) {
    const ParsedEnode& e = enodes[i];
    ENode* obj = new ENode();
    obj->opType = (OPType)e.opType;
    obj->width = e.width;
    obj->usedBit = e.usedBit;
    obj->id = e.id;
    obj->sign = e.sign != 0;
    obj->isClock = e.isClock != 0;
    obj->reset = (ResetType)e.reset;
    obj->strVal = e.strVal;
    obj->values = e.values;
    b.enodes.push_back(obj);
  }
  /* second pass: ENode pointers become available only after allocation */
  for (uint32_t i = 0; i < enodeCount; i++) {
    const ParsedEnode& e = enodes[i];
    ENode* obj = b.enodes[i];
    for (uint32_t ref : e.child) {
      ENode* c = b.enode(ref);
      if (!b.why.empty()) {
        *why = b.why;
        return false;
      }
      obj->child.push_back(c);
    }
  }
  b.trees.reserve(treeCount);
  for (uint32_t i = 0; i < treeCount; i++) {
    ENode* root = b.enode(trees[i].root);
    if (root == nullptr || !b.why.empty()) {
      *why = b.why.empty() ? "ExpTree with null root" : b.why;
      return false;
    }
    ExpTree* obj = new ExpTree(root);
    obj->setlval(b.enode(trees[i].lvalue));
    if (!b.why.empty()) {
      *why = b.why;
      return false;
    }
    b.trees.push_back(obj);
  }
  /* ENode node/memory/computeInfo refs need the node table: allocate the
     graph, nodes and supers first, then patch everything pointer-shaped. */
  graph* g = new graph();
  for (uint32_t i = 0; i < nodeCount; i++) {
    Node* obj = new Node();
    const ParsedNode& n = nodes[i];
    if (n.id < 0 || !b.byNid.emplace(n.id, obj).second) {
      *why = "duplicate or negative node id " + std::to_string(n.id);
      return false;
    }
  }
  for (uint32_t i = 0; i < superCount; i++) {
    SuperNode* obj = new SuperNode();
    const ParsedSuper& s = supers[i];
    if (s.id < 0 || !b.bySid.emplace(s.id, obj).second) {
      *why = "duplicate or negative superNode id " + std::to_string(s.id);
      return false;
    }
  }
  for (uint32_t i = 0; i < enodeCount; i++) {
    const ParsedEnode& e = enodes[i];
    ENode* obj = b.enodes[i];
    obj->nodePtr = b.node(e.nodePtr);
    obj->memoryNode = b.node(e.memoryNode);
    obj->computeInfo = b.val(e.computeInfo);
    if (!b.why.empty()) {
      *why = b.why;
      return false;
    }
  }
  for (uint32_t i = 0; i < stmtCount; i++) {
    std::function<StmtNode*(const ParsedStmt&)> build = [&](const ParsedStmt& ps) -> StmtNode* {
      StmtNode* sn = new StmtNode((OPType)ps.type);
      sn->isENode = ps.isENode != 0;
      sn->enode = b.enode(ps.enode);
      sn->tree = b.tree(ps.tree);
      sn->belong = b.node(ps.belong);
      sn->computeInfo = b.val(ps.computeInfo);
      for (const ParsedStmt& pc : ps.child) sn->child.push_back(build(pc));
      return sn;
    };
    StmtTree* st = new StmtTree();
    st->root = build(stmts[i]);
    if (!b.why.empty()) {
      *why = b.why;
      return false;
    }
    b.stmts.push_back(st);
  }
  for (uint32_t i = 0; i < superCount; i++) {
    const ParsedSuper& s = supers[i];
    SuperNode* obj = b.bySid[s.id];
    obj->id = s.id;
    obj->order = s.order;
    obj->cppId = s.cppId;
    obj->superType = (SuperType)s.superType;
    obj->resetNode = b.node(s.resetNode);
    if (s.stmtTree != kNullRef && s.stmtTree >= b.stmts.size()) {
      *why = "SuperNode stmtTree reference out of range";
      return false;
    }
    obj->stmtTree = s.stmtTree == kNullRef ? nullptr : b.stmts[s.stmtTree];
    auto fillSupers = [&](const std::vector<int32_t>& ids, std::set<SuperNode*>* dst) {
      for (int32_t id : ids) {
        SuperNode* x = b.super(id);
        if (x) dst->insert(x);
      }
    };
    fillSupers(s.prev, &obj->prev);
    fillSupers(s.next, &obj->next);
    fillSupers(s.depPrev, &obj->depPrev);
    fillSupers(s.depNext, &obj->depNext);
    for (int32_t id : s.member) {
      Node* m = b.node(id);
      if (!b.why.empty()) {
        *why = b.why;
        return false;
      }
      if (m) obj->member.push_back(m);
    }
    for (const ParsedInst& inst : s.insts) {
      Node* n = b.node(inst.node);
      if (!b.why.empty()) {
        *why = b.why;
        return false;
      }
      if (n == nullptr) {
        *why = "InstInfo without a node";
        return false;
      }
      obj->insts.push_back(InstInfo((SuperInfo)inst.infoType, n, inst.inst));
    }
  }
  for (uint32_t i = 0; i < nodeCount; i++) {
    const ParsedNode& n = nodes[i];
    Node* obj = b.byNid[n.id];
    obj->name = n.name;
    obj->extraInfo = n.extraInfo;
    obj->id = n.id;
    obj->type = (NodeType)n.type;
    obj->width = n.width;
    obj->usedBit = n.usedBit;
    obj->status = (NodeStatus)n.status;
    obj->order = n.order;
    obj->orderInSuper = n.orderInSuper;
    obj->lineno = n.lineno;
    obj->sign = n.sign != 0;
    obj->dimension = n.dimension;
    obj->regSplit = n.regSplit != 0;
    obj->isClock = n.isClock != 0;
    obj->inAggr = n.inAggr != 0;
    obj->nodeIsRoot = n.nodeIsRoot != 0;
    obj->reset = (ResetType)n.reset;
    obj->asReset = (AsReset)n.asReset;
    obj->whenDepth = n.whenDepth;
    obj->rlatency = n.rlatency;
    obj->wlatency = n.wlatency;
    obj->depth = n.depth;
    obj->super = b.super(n.super);
    obj->parent = b.node(n.parent);
    obj->regNext = b.node(n.regNext);
    obj->clock = b.node(n.clock);
    obj->computeInfo = b.val(n.computeInfo);
    obj->resetTree = b.tree(n.resetTree);
    for (uint32_t ref : n.assignTree) {
      ExpTree* t = b.tree(ref);
      if (!b.why.empty()) {
        *why = b.why;
        return false;
      }
      obj->assignTree.push_back(t);
    }
    auto fillNodes = [&](const std::vector<int32_t>& ids, std::set<Node*>* dst) {
      for (int32_t id : ids) {
        Node* x = b.node(id);
        if (x) dst->insert(x);
      }
    };
    fillNodes(n.next, &obj->next);
    fillNodes(n.prev, &obj->prev);
    fillNodes(n.depPrev, &obj->depPrev);
    fillNodes(n.depNext, &obj->depNext);
    for (int32_t id : n.member) {
      Node* m = b.node(id);
      if (!b.why.empty()) {
        *why = b.why;
        return false;
      }
      if (m) obj->member.push_back(m);
    }
    for (const std::pair<uint8_t, std::string>& p : n.params) {
      obj->params.push_back(std::make_pair(p.first != 0, p.second));
    }
    for (int32_t id : n.nextActiveId) obj->nextActiveId.insert(id);
    for (int32_t id : n.nextNeedActivate) obj->nextNeedActivate.insert(id);
    if (!b.why.empty()) {
      *why = b.why;
      return false;
    }
  }
  auto fillNodeVec = [&](const std::vector<int32_t>& ids, std::vector<Node*>* dst) {
    dst->reserve(ids.size());
    for (int32_t id : ids) {
      Node* x = b.node(id);
      if (!b.why.empty()) return false;
      dst->push_back(x);
    }
    return true;
  };
  auto fillSuperVec = [&](const std::vector<int32_t>& ids, std::vector<SuperNode*>* dst) {
    dst->reserve(ids.size());
    for (int32_t id : ids) {
      SuperNode* x = b.super(id);
      if (!b.why.empty()) return false;
      dst->push_back(x);
    }
    return true;
  };
  g->name = pg.name;
  g->nodeNum = pg.nodeNum;
  g->extDecl = pg.extDecl;
  if (!fillNodeVec(pg.allNodes, &g->allNodes) || !fillNodeVec(pg.input, &g->input) ||
      !fillNodeVec(pg.output, &g->output) || !fillNodeVec(pg.regsrc, &g->regsrc) ||
      !fillNodeVec(pg.sorted, &g->sorted) || !fillNodeVec(pg.memory, &g->memory) ||
      !fillNodeVec(pg.external, &g->external) ||
      !fillNodeVec(pg.specialNodes, &g->specialNodes) ||
      !fillSuperVec(pg.supersrc, &g->supersrc) ||
      !fillSuperVec(pg.sortedSuper, &g->sortedSuper) ||
      !fillSuperVec(pg.allReset, &g->allReset)) {
    *why = b.why.empty() ? "graph vector reference error" : b.why;
    return false;
  }
  for (int32_t id : pg.halfConstantArray) {
    Node* x = b.node(id);
    if (!b.why.empty()) {
      *why = b.why;
      return false;
    }
    if (x) g->halfConstantArray.insert(x);
  }
  /* Restore the id counters exactly: Final-allocated objects must receive
     the ids a no-checkpoint run would have assigned. */
  Node::setCounterValue(nodeCounter);
  SuperNode::setCounterValue(superCounter);
  ENode::setCounterValue(enodeCounter);
  *out = g;
  return true;
}

/* ============================= ENTRY I/O ============================== */

struct EntryMeta {
  std::string keyHex;
  std::string bodySha;
  uint64_t bodyBytes = 0;
};

/* Parses and strictly validates a ckpt entry KEY file. */
bool parseEntryKey(const std::string& key, const std::string& expectKey, EntryMeta* meta,
                   std::string* reason) {
  const size_t magicLen = strlen(kEntryMagic);
  if (key.compare(0, magicLen, kEntryMagic) != 0 || key.size() <= magicLen ||
      key[magicLen] != '\n') {
    *reason = "bad entry magic";
    return false;
  }
  const size_t line2 = magicLen + 1;
  const size_t line2End = key.find('\n', line2);
  if (line2End == std::string::npos || line2End - line2 != 4 + 64 ||
      key.compare(line2, 4, "key=") != 0 || !isHexString(key.substr(line2 + 4, 64), 64)) {
    *reason = "bad key line";
    return false;
  }
  meta->keyHex = key.substr(line2 + 4, 64);
  if (meta->keyHex != expectKey) {
    *reason = "key mismatch";
    return false;
  }
  const size_t marker = key.find("\n--body--\n", line2End);
  if (marker == std::string::npos) {
    *reason = "no body marker";
    return false;
  }
  const size_t pos = marker + strlen("\n--body--\n");
  const size_t end = key.find('\n', pos);
  const std::string line =
      key.substr(pos, (end == std::string::npos ? key.size() : end) - pos);
  const size_t sp = line.find(' ');
  if (sp == std::string::npos || !isHexString(line.substr(0, sp), 64)) {
    *reason = "bad body line";
    return false;
  }
  char* endp = nullptr;
  errno = 0;
  const unsigned long long bytes = strtoull(line.c_str() + sp + 1, &endp, 10);
  if (errno != 0 || endp == line.c_str() + sp + 1 || *endp != '\0') {
    *reason = "bad body size";
    return false;
  }
  meta->bodySha = line.substr(0, sp);
  meta->bodyBytes = bytes;
  return true;
}

}  // namespace

/* =============================== IMPL ================================= */

struct Checkpointer::Impl {
  std::string ckptDir;
  bool verify = false;
  bool firHashed = false;
  std::string firSha;
  uint64_t firLen = 0;
  std::string closureSha;
  size_t closureFiles = 0;
  std::string keyHex;
  std::string entryDir;

  bool buildKey() {
    const std::string material = buildKeyMaterial(firSha, firLen, closureSha, closureFiles);
    keyHex = sha256Hex(material.data(), material.size());
    entryDir = ckptDir + "/" + keyHex;
    return true;
  }

  /* Loads and sha-verifies the body of the entry for the current key. */
  bool loadEntryBody(std::string* body, std::string* reason) const {
    std::string key;
    if (!readFileBytes(entryDir + "/KEY", &key)) {
      *reason = "no KEY file";
      return false;
    }
    EntryMeta meta;
    if (!parseEntryKey(key, keyHex, &meta, reason)) return false;
    body->clear();
    if (!readFileBytes(entryDir + "/graph.ckpt", body)) {
      *reason = "graph body integrity failure";
      return false;
    }
    /* Multi-volume entries (RLIMIT_FSIZE guard): concatenate graph.ckpt.part2..N
       in order until the expected total size is reached; a missing or extra
       part fails integrity below via the size/sha check. */
    for (size_t part = 2; body->size() < meta.bodyBytes; part ++) {
      std::string piece;
      if (!readFileBytes(entryDir + "/graph.ckpt.part" + std::to_string(part), &piece)) break;
      body->append(piece);
    }
    if (body->size() != meta.bodyBytes ||
        sha256Hex(body->data(), body->size()) != meta.bodySha) {
      *reason = "graph body integrity failure";
      return false;
    }
    return true;
  }

  /* Reads only the manifest of the current entry (drift comparison). */
  bool peekEntryMeta(EntryMeta* meta, std::string* reason) const {
    std::string key;
    if (!readFileBytes(entryDir + "/KEY", &key)) {
      *reason = "no KEY file";
      return false;
    }
    return parseEntryKey(key, keyHex, meta, reason);
  }

  void publish(const std::string& body) {
    const std::string bodySha = sha256Hex(body.data(), body.size());
    if (verify) {
      EntryMeta existing;
      std::string reason;
      if (peekEntryMeta(&existing, &reason)) {
        if (existing.bodySha == bodySha) {
          fprintf(stderr,
                  "[genCheckpoint] verify: fresh fingerprint identical to cached entry "
                  "(no pre-Final drift)\n");
        } else {
          fprintf(stderr,
                  "[genCheckpoint] VERIFY DRIFT: fresh pre-Final fingerprint %s differs "
                  "from cached entry %s (pointer-order-sensitive trajectory or an input "
                  "missing from the key; replacing the entry)\n",
                  bodySha.c_str(), existing.bodySha.c_str());
        }
      }
    }
    std::string tmpPattern = ckptDir + "/.tmp.XXXXXX";
    std::vector<char> tmpl(tmpPattern.begin(), tmpPattern.end());
    tmpl.push_back('\0');
    if (mkdtemp(tmpl.data()) == nullptr) {
      fprintf(stderr, "[genCheckpoint] store aborted: cannot create temp dir in %s (%s)\n",
              ckptDir.c_str(), strerror(errno));
      return;
    }
    const std::string tmp = tmpl.data();
    /* RLIMIT_FSAFE guard: the whole-graph body can exceed a 2 GB RLIMIT_FSIZE
       (write() fails mid-file). Store as N volumes of <= 1.5 GB named
       graph.ckpt (single-volume case keeps the original name) and
       graph.ckpt.part2..N; loadEntryBody concatenates in order. */
    {
      const size_t kVolume = (size_t)1536 << 20;
      size_t volumes = (body.size() + kVolume - 1) / kVolume;
      if (volumes == 0) volumes = 1;
      bool ok = true;
      for (size_t v = 0; v < volumes && ok; v ++) {
        const size_t beg = v * kVolume;
        const size_t end = std::min(body.size(), beg + kVolume);
        std::string name = v == 0 ? "graph.ckpt" : "graph.ckpt.part" + std::to_string(v + 1);
        ok = writePrivateFile(tmp + "/" + name, body.substr(beg, end - beg));
      }
      if (!ok) {
        fprintf(stderr, "[genCheckpoint] store aborted: cannot write graph.ckpt volumes\n");
        removeTreeBestEffort(tmp);
        return;
      }
    }
    std::string keyFile;
    keyFile += std::string(kEntryMagic) + "\n";
    keyFile += "key=" + keyHex + "\n";
    keyFile += "--key-material--\n";
    keyFile += buildKeyMaterial(firSha, firLen, closureSha, closureFiles);
    keyFile += "--body--\n";
    keyFile += bodySha + " " + std::to_string(body.size()) + "\n";
    if (!writePrivateFile(tmp + "/KEY", keyFile)) {
      fprintf(stderr, "[genCheckpoint] store aborted: cannot write KEY\n");
      removeTreeBestEffort(tmp);
      return;
    }
    fsyncFileOrDir(tmp, true);
    if (rename(tmp.c_str(), entryDir.c_str()) != 0) {
      if (errno == ENOTEMPTY || errno == EEXIST) {
        /* Keep a still-valid entry; replace a corrupt or drifted one. */
        EntryMeta existing;
        std::string reason;
        const bool valid = peekEntryMeta(&existing, &reason);
        const bool identical = valid && existing.bodySha == bodySha;
        if (identical) {
          removeTreeBestEffort(tmp);
          fprintf(stderr, "[genCheckpoint] store skipped: identical entry already present\n");
          return;
        }
        const std::string trash =
            ckptDir + "/.trash." + std::to_string((long)getpid()) + "." +
            std::to_string((unsigned long)time(nullptr));
        if (rename(entryDir.c_str(), trash.c_str()) != 0) {
          fprintf(stderr,
                  "[genCheckpoint] store aborted: existing entry cannot be replaced (%s)\n",
                  strerror(errno));
          removeTreeBestEffort(tmp);
          return;
        }
        removeTreeBestEffort(trash);
        if (rename(tmp.c_str(), entryDir.c_str()) != 0) {
          fprintf(stderr, "[genCheckpoint] store aborted: rename failed (%s)\n",
                  strerror(errno));
          removeTreeBestEffort(tmp);
          return;
        }
      } else {
        fprintf(stderr, "[genCheckpoint] store aborted: rename failed (%s)\n", strerror(errno));
        removeTreeBestEffort(tmp);
        return;
      }
    }
    fsyncFileOrDir(ckptDir, true);
    fprintf(stderr, "[genCheckpoint] stored key=%s body-bytes=%zu\n",
            keyHex.c_str(), body.size());
  }
};

Checkpointer* Checkpointer::create() {
  const char* dir = std::getenv("GSIM_GEN_CHECKPOINT");
  const char* verify = std::getenv("GSIM_GEN_CHECKPOINT_VERIFY");
  if (dir == nullptr || dir[0] == '\0') {
    if (verify != nullptr && verify[0] != '\0') {
      fprintf(stderr,
              "[genCheckpoint] GSIM_GEN_CHECKPOINT_VERIFY ignored: GSIM_GEN_CHECKPOINT "
              "is not set\n");
    }
    return nullptr;
  }
  /* A restore skips the passes that produce debug artifacts, so any dump
     request disables the checkpoint entirely (fail-closed). */
  if (globalConfig.EnableDumpGraph || globalConfig.DumpAssignTree ||
      globalConfig.DumpConstStatus) {
    fprintf(stderr,
            "[genCheckpoint] disabled: debug dump flags requested, regenerating fully\n");
    return nullptr;
  }
  Impl* impl = new Impl();
  impl->ckptDir = std::string(dir) + "/ckpt";
  if (verify != nullptr && strcmp(verify, "1") == 0) impl->verify = true;
  if (!mkdirRecursive(impl->ckptDir)) {
    fprintf(stderr, "[genCheckpoint] disabled: cannot use checkpoint dir %s (%s)\n",
            impl->ckptDir.c_str(), strerror(errno));
    delete impl;
    return nullptr;
  }
  pruneStaleScratchDirs(impl->ckptDir);
  std::string reason;
  if (!collectPreFinalClosure(&impl->closureSha, &impl->closureFiles, &reason)) {
    fprintf(stderr,
            "[genCheckpoint] disabled: pre-Final source closure unavailable (%s); "
            "full pipeline, no store, no restore\n",
            reason.c_str());
    delete impl;
    return nullptr;
  }
  Checkpointer* ckpt = new Checkpointer();
  ckpt->impl_ = impl;
  fprintf(stderr,
          "[genCheckpoint] enabled dir=%s verify=%d closure-files=%zu "
          "closure-sha256=%s\n",
          impl->ckptDir.c_str(), impl->verify ? 1 : 0, impl->closureFiles,
          impl->closureSha.c_str());
  return ckpt;
}

void Checkpointer::hashFirInput(const char* bytes, size_t len) {
  impl_->firSha = sha256Hex(bytes, len);
  impl_->firLen = len;
  impl_->firHashed = true;
}

bool Checkpointer::tryRestore(graph** out) {
  Impl& impl = *impl_;
  if (!impl.firHashed) {
    fprintf(stderr, "[genCheckpoint] miss: FIR input was not hashed\n");
    return false;
  }
  impl.buildKey();
  std::string body;
  std::string reason;
  if (!impl.loadEntryBody(&body, &reason)) {
    fprintf(stderr, "[genCheckpoint] miss: key=%s (%s)\n", impl.keyHex.c_str(),
            reason.c_str());
    return false;
  }
  graph* g = nullptr;
  if (!deserializeGraph(body, &g, &reason)) {
    fprintf(stderr, "[genCheckpoint] miss: key=%s body parse failed (%s)\n",
            impl.keyHex.c_str(), reason.c_str());
    return false;
  }
  fprintf(stderr,
          "[genCheckpoint] hit: key=%s body-bytes=%zu; skipping the pre-Final "
          "pipeline (parseFIR ran only for the genCache module-name lookup)\n",
          impl.keyHex.c_str(), body.size());
  *out = g;
  return true;
}

void Checkpointer::store(graph* g) {
  Impl& impl = *impl_;
  if (!impl.firHashed) {
    fprintf(stderr, "[genCheckpoint] store skipped: FIR input was not hashed\n");
    return;
  }
  impl.buildKey();
  std::string body;
  std::string why;
  if (!serializeGraph(g, &body, &why)) {
    fprintf(stderr, "[genCheckpoint] store aborted: %s\n", why.c_str());
    return;
  }
  impl.publish(body);
}

Checkpointer::~Checkpointer() { delete impl_; }

}  // namespace genCheckpoint
