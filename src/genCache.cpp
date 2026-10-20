/*
  genCache.cpp: fail-closed module-level generation cache. See genCache.h.

  One gsim invocation compiles exactly one module: the FIR circuit name
  (root->name, == graph::name) with every module definition in the input
  file flattened into it (AST2Graph builds moduleMap from ALL root children
  and inlines instances, so any module text in the file can change emitted
  bytes). The cache therefore keys on, and only on:

    1. module name            - fixes every emitted symbol and file name
                                ("per module, not per file": the input path
                                is deliberately NOT part of the key)
    2. complete FIR file text - the only semantic design input; hashed
                                before parseFIR(), which splits the buffer
    3. running gsim binary    - sha256(/proc/self/exe); covers GSIM_VERSION,
                                build date, compiler, and every compile-time
                                switch that shapes output (PERF,
                                DIFFTEST_PER_SIG, GSIM_DIFF, VERILATOR_DIFF,
                                ENABLE_ACTIVATOR, RW_COND_READ, ...) and any
                                generator code change
    4. generation options     - every Config field that can reach emitted
                                bytes (see buildKeyMaterial())
    5. generation env vars    - GSIM_THREADS, GSIM_EMIT_RESET_CHUNK and
                                GSIM_MT_DYNAMIC_QUERY: the complete set of
                                environment the generator itself reads
                                (see buildKeyMaterial())

  Completeness of (4): config.h holds exactly these fields. Excluded fields
  and why the exclusion cannot change one emitted byte:
    - OutputDir  : location only; never appears in emitted text
    - NumThreads : parser worker count only. Task decomposition (fixed
                   MODULES_PER_TASK boundaries at module markers) and merge
                   order are sequential, so the parse result is identical
                   for any thread count. The parse also still runs on hits.
    - LogLevel   : stderr/stdout verbosity only.
    - EnableDumpGraph / DumpGraphDot / DumpGraphJson / DumpStages /
      DumpAssignTree / DumpConstStatus : debug-artifact switches; if any is
      set the cache is disabled outright (a hit skips the stages that would
      produce those artifacts), so they cannot affect a cached run.

  Failure modes (every one resolves to full regeneration, never a stale or
  partial restore):
    FIR text changed / any flag or env changed / binary rebuilt ... key
      differs -> different entry dir -> miss.
    Entry artifact corrupted, truncated, extra, or foreign (tampered
      manifest, wrong key line, unparseable KEY) -> manifest sha256 or
      strict parse check fails -> miss.
    Partial entry from a crashed store -> store publishes by atomic
      directory rename only; an unpublished tmp dir is never looked up.
    Cache dir missing/unwritable, /proc/self/exe unreadable, dump flags
      set -> cache disabled (or store skipped) -> full generation.
    Stale or foreign same-namespace files in OutputDir (including
      name-ambiguous ones: <Foo>10.cpp is also module Foo1's part 0)
      -> moved to a private staging dir under an exclusive per-OutputDir
      flock before this run writes anything; what is stored or verified
      afterwards was provably produced by this invocation.
    OutputDir lock held by another generation -> abort: GSIM_GEN_CACHE
      enforces a single writer per OutputDir, because a racing writer would
      poison capture. Concurrent gsim runs sharing an OutputDir WITHOUT
      the cache remain unsupported pre-existing behavior.
    Staging impossible, OutputDir unlistable, or any OutputDir mutation
      fails -> cache goes miss-through (no restore, no store, no verify):
      full regeneration, output identical to a cache-less run.
*/

#include "genCache.h"

#include "common.h"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace genCache {

namespace {

/* ---------------------------------------------------------------- SHA-256 */
class Sha256 {
 public:
  Sha256() { reset(); }
  void reset() {
    static const uint32_t kInit[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372,
                                      0xa54ff53a, 0x510e527f, 0x9b05688c,
                                      0x1f83d9ab, 0x5be0cd19};
    memcpy(h_, kInit, sizeof h_);
    len_ = 0;
    bufLen_ = 0;
  }
  void update(const void* data, size_t n) {
    const unsigned char* p = static_cast<const unsigned char*>(data);
    len_ += n;
    if (bufLen_ != 0) {
      size_t take = std::min(n, sizeof buf_ - bufLen_);
      memcpy(buf_ + bufLen_, p, take);
      bufLen_ += (unsigned char)take;
      p += take;
      n -= take;
      if (bufLen_ == sizeof buf_) {
        block(buf_);
        bufLen_ = 0;
      }
    }
    while (n >= 64) {
      block(p);
      p += 64;
      n -= 64;
    }
    if (n != 0) {
      memcpy(buf_, p, n);
      bufLen_ = (unsigned char)n;
    }
  }
  void final(unsigned char out[32]) {
    const uint64_t bitLen = len_ * 8;
    const unsigned char one = 0x80, zero = 0;
    update(&one, 1);
    while (bufLen_ != 56) update(&zero, 1);
    unsigned char lenBuf[8];
    for (int i = 0; i < 8; i++) lenBuf[i] = (unsigned char)(bitLen >> (56 - 8 * i));
    update(lenBuf, 8);
    for (int i = 0; i < 8; i++) {
      out[4 * i + 0] = (unsigned char)(h_[i] >> 24);
      out[4 * i + 1] = (unsigned char)(h_[i] >> 16);
      out[4 * i + 2] = (unsigned char)(h_[i] >> 8);
      out[4 * i + 3] = (unsigned char)(h_[i]);
    }
  }

 private:
  static uint32_t rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }
  void block(const unsigned char* p) {
    static const uint32_t K[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1,
        0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
        0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
        0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
        0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
        0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
        0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
        0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    uint32_t w[64];
    for (int i = 0; i < 16; i++) {
      w[i] = ((uint32_t)p[4 * i] << 24) | ((uint32_t)p[4 * i + 1] << 16) |
             ((uint32_t)p[4 * i + 2] << 8) | (uint32_t)p[4 * i + 3];
    }
    for (int i = 16; i < 64; i++) {
      const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h_[0], b = h_[1], c = h_[2], d = h_[3];
    uint32_t e = h_[4], f = h_[5], g = h_[6], h = h_[7];
    for (int i = 0; i < 64; i++) {
      const uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const uint32_t ch = (e & f) ^ (~e & g);
      const uint32_t t1 = h + S1 + ch + K[i] + w[i];
      const uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t t2 = S0 + maj;
      h = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
    }
    h_[0] += a; h_[1] += b; h_[2] += c; h_[3] += d;
    h_[4] += e; h_[5] += f; h_[6] += g; h_[7] += h;
  }

  uint32_t h_[8];
  uint64_t len_;
  unsigned char buf_[64];
  unsigned char bufLen_;
};

std::string toHex(const unsigned char* bytes, size_t n) {
  static const char kDigits[] = "0123456789abcdef";
  std::string out(n * 2, '0');
  for (size_t i = 0; i < n; i++) {
    out[2 * i] = kDigits[bytes[i] >> 4];
    out[2 * i + 1] = kDigits[bytes[i] & 0xf];
  }
  return out;
}

std::string sha256Hex(const void* data, size_t n) {
  Sha256 sha;
  sha.update(data, n);
  unsigned char digest[32];
  sha.final(digest);
  return toHex(digest, 32);
}

/* --------------------------------------------------------------- fs utils */
bool readFileBytes(const std::string& path, std::string* out) {
  int fd = open(path.c_str(), O_RDONLY);
  if (fd < 0) return false;
  out->clear();
  char buf[65536];
  ssize_t n;
  while ((n = read(fd, buf, sizeof buf)) > 0) out->append(buf, (size_t)n);
  const bool ok = n == 0;
  close(fd);
  return ok;
}

bool listDirFiles(const std::string& dir, std::vector<std::string>* out) {
  DIR* d = opendir(dir.c_str());
  if (d == nullptr) return false;
  out->clear();
  while (struct dirent* e = readdir(d)) {
    if (strcmp(e->d_name, ".") == 0 || strcmp(e->d_name, "..") == 0) continue;
    struct stat sb;
    if (fstatat(dirfd(d), e->d_name, &sb, 0) != 0 || !S_ISREG(sb.st_mode)) continue;
    out->push_back(e->d_name);
  }
  closedir(d);
  std::sort(out->begin(), out->end());
  return true;
}

bool isDirectory(const std::string& path) {
  struct stat sb;
  return stat(path.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode);
}

bool mkdirOne(const std::string& path) {
  if (mkdir(path.c_str(), 0777) == 0) return true;
  return errno == EEXIST && isDirectory(path);
}

bool mkdirRecursive(const std::string& path) {
  if (path.empty()) return false;
  if (isDirectory(path)) return true;
  std::string prefix;
  size_t i = 0;
  if (path[0] == '/') prefix = "/";
  while (i < path.size()) {
    size_t next = path.find('/', i);
    if (next == std::string::npos) next = path.size();
    if (next > i) {
      prefix += path.substr(i, next - i);
      if (!mkdirOne(prefix)) return false;
    }
    if (next < path.size()) prefix += "/";
    i = next + 1;
  }
  return isDirectory(path);
}

void fsyncFileOrDir(const std::string& path, bool directory) {
  int fd = open(path.c_str(), directory ? (O_RDONLY | O_DIRECTORY) : O_RDONLY);
  if (fd < 0) return;
  fsync(fd);
  close(fd);
}

bool writePrivateFile(const std::string& path, const std::string& data) {
  int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0666);
  if (fd < 0) return false;
  size_t off = 0;
  while (off < data.size()) {
    const ssize_t n = write(fd, data.data() + off, data.size() - off);
    if (n <= 0) {
      close(fd);
      return false;
    }
    off += (size_t)n;
  }
  fsync(fd);
  close(fd);
  return true;
}

void removeTreeBestEffort(const std::string& dir) {
  std::vector<std::string> files;
  if (!listDirFiles(dir, &files)) {
    rmdir(dir.c_str());
    return;
  }
  for (const std::string& f : files) unlink((dir + "/" + f).c_str());
  rmdir(dir.c_str());
}

/* Best-effort GC of cache-private scratch dirs (.staging.*, .tmp.*, .trash.*)
 * left behind by crashed or killed runs. Older than 72h means abandoned;
 * every error is ignored. */
void pruneStaleScratchDirs(const std::string& cacheDir) {
  static const char* kPrefixes[] = {".staging.", ".tmp.", ".trash."};
  const long long kMaxAgeSeconds = 72LL * 3600;
  DIR* d = opendir(cacheDir.c_str());
  if (d == nullptr) return;
  const long long now = (long long)time(nullptr);
  while (struct dirent* e = readdir(d)) {
    const std::string name = e->d_name;
    if (name == "." || name == "..") continue;
    bool matched = false;
    for (const char* p : kPrefixes) {
      if (name.compare(0, strlen(p), p) == 0) {
        matched = true;
        break;
      }
    }
    if (!matched) continue;
    struct stat sb;
    if (fstatat(dirfd(d), name.c_str(), &sb, 0) != 0 || !S_ISDIR(sb.st_mode)) continue;
    if ((long long)sb.st_mtime > now - kMaxAgeSeconds) continue;  // too recent
    removeTreeBestEffort(cacheDir + "/" + name);
  }
  closedir(d);
}

/* Escape any byte outside printable-non-space ASCII so key material lines
   stay one-per-field and the serialized blob is injective. */
std::string escapeKeyToken(const std::string& s) {
  static const char* kHex = "0123456789abcdef";
  std::string out;
  for (unsigned char c : s) {
    if (c >= 0x21 && c <= 0x7e) {
      out += (char)c;
    } else {
      out += "\\x";
      out += kHex[c >> 4];
      out += kHex[c & 0xf];
    }
  }
  return out;
}

/* Path-safe module name for the entry directory. The key suffix keeps entry
   directories unique even if two distinct module names sanitize alike. */
std::string sanitizePathComponent(const std::string& s) {
  std::string out;
  for (char c : s) {
    out += (c == '_' || (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
            (c >= 'A' && c <= 'Z')) ? c : '_';
  }
  return out;
}

bool isHexString(const std::string& s, size_t len) {
  if (s.size() != len) return false;
  for (char c : s) {
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  }
  return true;
}

/* Files the two emitters can write into OutputDir for module `name`, plus
   the fixed-name MT schedule reports. Derived from every fopen/ofstream on
   OutputDir in the tree: <name>.h, <name><N>.cpp, <name>_sigs.txt
   (cppEmitter / cppEmitter-mt), mt-task-phases.csv,
   mt-state-task-members.csv, mt-replication-decisions.csv,
   mt-replication-final.csv. Graph dumps and <name>_ConstStatus.json only
   appear under dump flags, which disable the cache. */
bool isNamespaceFile(const std::string& module, const std::string& name) {
  static const char* kFixed[] = {
      "mt-task-phases.csv", "mt-state-task-members.csv",
      "mt-replication-decisions.csv", "mt-replication-final.csv"};
  for (const char* f : kFixed) {
    if (name == f) return true;
  }
  if (name.size() <= module.size() + 1) return false;
  if (name.compare(0, module.size(), module) != 0) return false;
  const std::string suffix = name.substr(module.size());
  if (suffix == ".h" || suffix == "_sigs.txt" || suffix == "_ConstStatus.json") return true;
  size_t i = 0;
  while (i < suffix.size() && suffix[i] >= '0' && suffix[i] <= '9') i++;
  return i > 0 && i + 4 == suffix.size() && suffix.compare(i, 4, ".cpp") == 0;
}

bool isSafeArtifactName(const std::string& name) {
  if (name.empty() || name[0] == '.') return false;
  for (char c : name) {
    if (c == '/' || c == '\\' || c == '\n' || c == '\r' || c == '\t' || c == ' ') return false;
  }
  return true;
}

struct EntryArtifact {
  std::string name;
  std::string sha256;  // hex
  uint64_t size = 0;
};

const char kEntryMagic[] = "GSIM_GEN_CACHE_ENTRY_V1";
const char kKeyMagic[] = "GSIM_GEN_CACHE_KEY_V1";

/* Every option below can change emitted bytes; see the header comment for
   why the omitted Config fields cannot. */
std::string buildKeyMaterial(const std::string& module, const std::string& firSha,
                             uint64_t firLen, const std::string& binSha, uint64_t binLen) {
  std::string s;
  auto addLine = [&s](const std::string& line) { s += line; s += '\n'; };
  addLine(kKeyMagic);
  addLine("module=" + escapeKeyToken(module));
  addLine("fir-sha256=" + firSha + " fir-bytes=" + std::to_string(firLen));
  addLine("emitter-sha256=" + binSha + " emitter-bytes=" + std::to_string(binLen));
  auto addFlag = [&](const char* k, const std::string& v) { addLine(std::string("opt:") + k + "=" + v); };
  auto addFlagInt = [&](const char* k, long long v) { addFlag(k, std::to_string(v)); };
  auto addFlagBool = [&](const char* k, bool v) { addFlagInt(k, v ? 1 : 0); };
  auto addFlagStr = [&](const char* k, const std::string& v) { addFlag(k, escapeKeyToken(v)); };
  addFlagBool("mt-mode", globalConfig.MtMode);
  addFlagInt("supernode-max-size", globalConfig.SuperNodeMaxSize);
  addFlagInt("cpp-max-size-KB", (long long)globalConfig.cppMaxSizeKB);
  addFlagStr("sep-mod", globalConfig.sep_module);
  addFlagStr("sep-aggr", globalConfig.sep_aggr);
  addFlagInt("when-size", globalConfig.MergeWhenSize);
  addFlagInt("when2mux-bound", globalConfig.When2muxBound);
  addFlagInt("mt-target-tasks", globalConfig.MtTargetTasks);
  addFlagInt("mt-partition-node-weight", globalConfig.MtPartitionNodeWeight);
  addFlagInt("mt-schedule-global-weight", globalConfig.MtScheduleGlobalWeight);
  addFlagInt("mt-schedule-comm-node-weight", globalConfig.MtScheduleCommNodeWeight);
  addFlagStr("mt-scheduler", globalConfig.MtScheduler);
  addFlagInt("mt-replication-max-ops", globalConfig.MtReplicationMaxOps);
  addFlagInt("mt-replication-min-fanout", globalConfig.MtReplicationMinFanout);
  addFlagInt("mt-lookahead-window", globalConfig.MtLookaheadWindow);
  addFlagBool("mt-lookahead-stats", globalConfig.MtLookaheadStats);
  addFlagInt("mt-sparse-register-min-bytes", globalConfig.MtSparseRegisterMinBytes);
  addFlagInt("mt-sparse-register-max-writes", globalConfig.MtSparseRegisterMaxWrites);
  /* Every environment variable the generator itself reads: enumerated by
     grepping std::getenv across src/ and keeping generation-time reads.
     GSIM_MT_EXECUTOR and GSIM_MT_CPU_AFFINITY appear only inside string
     literals baked into the emitted model (runtime knobs of the generated
     code) and so cannot change generated bytes; GSIM_GEN_CACHE /
     GSIM_GEN_CACHE_VERIFY control this cache, not the emitter. */
  static const char* const kGenEnvVars[] = {
      "GSIM_THREADS",           /* MT worker count baked into the model */
      "GSIM_EMIT_RESET_CHUNK",  /* MT reset-emission chunking */
      "GSIM_MT_DYNAMIC_QUERY",  /* MT ready-query codegen knob */
  };
  for (const char* name : kGenEnvVars) {
    const char* value = std::getenv(name);
    addLine(std::string("env:") + name + "=" + escapeKeyToken(value ? value : "(unset)"));
  }
  return s;
}

}  // namespace

struct Cache::Impl {
  std::string cacheDir;
  bool verify = false;
  bool firHashed = false;
  std::string firSha;
  uint64_t firLen = 0;
  std::string binSha;
  uint64_t binLen = 0;
  std::string module;
  std::string keyHex;
  std::string entryDir;
  bool verifyPending = false;  // verified hit awaiting fresh emission to compare
  bool cacheActive = true;     // false -> miss-through: no OutputDir mutation, no store
  bool staged = false;         // pre-existing namespace files moved away before our writes
  int lockFd = -1;             // exclusive OutputDir lock, held for the process lifetime
  std::string stagingDir;      // where pre-existing namespace files were moved

  /* Returns false when the directory cannot be listed. A failed listing is
     NOT an empty namespace (unknown files may exist), so every caller must
     treat false as fail-closed. */
  bool listNamespaceFiles(const std::string& dir, std::vector<std::string>* out) const {
    std::vector<std::string> all;
    if (!listDirFiles(dir, &all)) return false;
    out->clear();
    for (const std::string& f : all) {
      if (isNamespaceFile(module, f)) out->push_back(f);
    }
    return true;
  }

  /* Fail-closed OutputDir ownership. gsim file names cannot prove which
     module produced a file: <Foo>10.cpp is both Foo's part 10 and module
     Foo1's part 0, so a leftover namespace file may belong to a different
     module that once shared this OutputDir. Pre-existing namespace files
     are therefore MOVED into a private staging dir (recoverable, logged)
     instead of deleted, and nothing is ever captured by name pattern
     alone: after staging, every namespace file present was produced by
     this invocation under the OutputDir lock. Any failure here disables
     all cache activity for the run (miss-through), because an unmovable
     leftover could otherwise be captured into the new key. */
  bool stageNamespaceOut() {
    if (staged) return true;
    if (!cacheActive) return false;
    std::vector<std::string> files;
    if (!listNamespaceFiles(globalConfig.OutputDir, &files)) {
      fprintf(stderr, "[genCache] miss-through: cannot list %s (%s)\n",
              globalConfig.OutputDir.c_str(), strerror(errno));
      cacheActive = false;
      return false;
    }
    if (files.empty()) {
      staged = true;  // provably nothing to move; no staging dir needed
      return true;
    }
    const std::string pattern = cacheDir + "/.staging.XXXXXX";
    std::vector<char> tmpl(pattern.begin(), pattern.end());
    tmpl.push_back('\0');
    if (mkdtemp(tmpl.data()) == nullptr) {
      fprintf(stderr, "[genCache] miss-through: cannot create staging dir (%s)\n",
              strerror(errno));
      cacheActive = false;
      return false;
    }
    stagingDir = tmpl.data();
    for (const std::string& f : files) {
      if (rename((globalConfig.OutputDir + "/" + f).c_str(),
                 (stagingDir + "/" + f).c_str()) != 0) {
        fprintf(stderr, "[genCache] miss-through: cannot move %s out of %s (%s); "
                "moved files remain recoverable in %s\n",
                f.c_str(), globalConfig.OutputDir.c_str(), strerror(errno),
                stagingDir.c_str());
        cacheActive = false;
        return false;
      }
    }
    staged = true;
    if (!files.empty()) {
      fprintf(stderr, "[genCache] moved %zu pre-existing namespace file(s) from %s to %s\n",
              files.size(), globalConfig.OutputDir.c_str(), stagingDir.c_str());
    }
    return true;
  }

  /* Exclusive advisory lock serializing cache activity per OutputDir. The
     emitter writes plain files with no interprocess guard, so two gsim
     generations sharing --dir would corrupt each other's capture; refuse
     that outright (miss-through) instead of racing. Held until the Cache
     is destroyed / process exit. */
  bool acquireOutputLock() {
    if (lockFd >= 0) return true;
    if (!cacheActive) return false;
    if (!mkdirRecursive(globalConfig.OutputDir)) {
      fprintf(stderr, "[genCache] miss-through: cannot access %s\n",
              globalConfig.OutputDir.c_str());
      cacheActive = false;
      return false;
    }
    const std::string lockPath = globalConfig.OutputDir + "/.gencache.lock";
    lockFd = open(lockPath.c_str(), O_CREAT | O_RDWR, 0666);
    if (lockFd < 0) {
      fprintf(stderr, "[genCache] miss-through: cannot create %s (%s)\n",
              lockPath.c_str(), strerror(errno));
      cacheActive = false;
      return false;
    }
    if (flock(lockFd, LOCK_EX | LOCK_NB) != 0) {
      const int busyErrno = errno;
      close(lockFd);
      lockFd = -1;
      fprintf(stderr,
              "[genCache] ABORT: %s is locked by another gsim generation (flock: %s). "
              "GSIM_GEN_CACHE enforces a single writer per OutputDir: a concurrent "
              "generation would poison cache capture. Concurrent gsim runs sharing an "
              "OutputDir WITHOUT the cache are (as ever) unsupported.\n",
              globalConfig.OutputDir.c_str(), strerror(busyErrno));
      std::abort();
    }
    return true;
  }

  bool loadEntry(const std::string& dir, const std::string& expectKey,
                 std::vector<EntryArtifact>* artifacts, std::string* reason) const {
    std::string key;
    if (!readFileBytes(dir + "/KEY", &key)) {
      *reason = "no KEY file";
      return false;
    }
    if (key.compare(0, strlen(kEntryMagic), kEntryMagic) != 0 ||
        key.size() <= strlen(kEntryMagic) || key[strlen(kEntryMagic)] != '\n') {
      *reason = "bad entry magic";
      return false;
    }
    /* Line 2: key=<64hex>. */
    const size_t line2 = strlen(kEntryMagic) + 1;
    const size_t line2End = key.find('\n', line2);
    if (line2End == std::string::npos || line2End - line2 != 4 + 64 ||
        key.compare(line2, 4, "key=") != 0 ||
        !isHexString(key.substr(line2 + 4, 64), 64)) {
      *reason = "bad key line";
      return false;
    }
    const std::string storedKey = key.substr(line2 + 4, 64);
    if (storedKey != expectKey) {
      *reason = "key mismatch";
      return false;
    }
    /* Artifact lines after the --artifacts-- marker. */
    const size_t marker = key.find("\n--artifacts--\n", line2End);
    if (marker == std::string::npos) {
      *reason = "no artifacts marker";
      return false;
    }
    size_t pos = marker + strlen("\n--artifacts--\n");
    std::vector<EntryArtifact> parsed;
    while (pos < key.size()) {
      const size_t end = key.find('\n', pos);
      const std::string line =
          key.substr(pos, (end == std::string::npos ? key.size() : end) - pos);
      pos = (end == std::string::npos) ? key.size() : end + 1;
      if (line.empty()) continue;
      const size_t sp1 = line.find(' ');
      const size_t sp2 = line.find(' ', sp1 == std::string::npos ? 0 : sp1 + 1);
      if (sp1 == std::string::npos || sp2 == std::string::npos) {
        *reason = "bad artifact line";
        return false;
      }
      EntryArtifact a;
      a.sha256 = line.substr(0, sp1);
      const std::string sizeStr = line.substr(sp1 + 1, sp2 - sp1 - 1);
      a.name = line.substr(sp2 + 1);
      if (!isHexString(a.sha256, 64) || a.name.empty() || !isSafeArtifactName(a.name)) {
        *reason = "bad artifact fields";
        return false;
      }
      char* endp = nullptr;
      errno = 0;
      const unsigned long long sz = strtoull(sizeStr.c_str(), &endp, 10);
      if (errno != 0 || endp == sizeStr.c_str() || *endp != '\0') {
        *reason = "bad artifact size";
        return false;
      }
      a.size = sz;
      parsed.push_back(a);
    }
    if (parsed.empty()) {
      *reason = "empty artifact list";
      return false;
    }
    for (size_t i = 0; i + 1 < parsed.size(); i++) {
      for (size_t j = i + 1; j < parsed.size(); j++) {
        if (parsed[i].name == parsed[j].name) {
          *reason = "duplicate artifact name";
          return false;
        }
      }
    }
    /* Entry dir must contain KEY plus exactly the listed artifacts. */
    std::vector<std::string> onDisk;
    if (!listDirFiles(dir, &onDisk) || onDisk.size() != parsed.size() + 1) {
      *reason = "entry file set does not match manifest";
      return false;
    }
    bool keySeen = false;
    size_t matched = 0;
    for (const std::string& f : onDisk) {
      if (f == "KEY") {
        keySeen = true;
        continue;
      }
      for (const EntryArtifact& a : parsed) {
        if (a.name == f) {
          matched++;
          break;
        }
      }
    }
    if (!keySeen || matched != parsed.size()) {
      *reason = "manifest/file-set mismatch";
      return false;
    }
    for (const EntryArtifact& a : parsed) {
      std::string bytes;
      if (!readFileBytes(dir + "/" + a.name, &bytes) || bytes.size() != a.size ||
          sha256Hex(bytes.data(), bytes.size()) != a.sha256) {
        *reason = "artifact integrity failure: " + a.name;
        return false;
      }
    }
    *artifacts = parsed;
    return true;
  }

  bool restore(const std::vector<EntryArtifact>& artifacts) {
    if (!mkdirRecursive(globalConfig.OutputDir)) return false;
    /* Namespace was already emptied into staging under the OutputDir lock. */
    for (const EntryArtifact& a : artifacts) {
      std::string bytes;
      if (!readFileBytes(entryDir + "/" + a.name, &bytes) ||
          bytes.size() != a.size || sha256Hex(bytes.data(), bytes.size()) != a.sha256) {
        return false;
      }
      const std::string dst = globalConfig.OutputDir + "/" + a.name;
      const std::string tmp =
          dst + ".gencache-tmp." + std::to_string((long)getpid());
      unlink(tmp.c_str());
      if (!writePrivateFile(tmp, bytes)) {
        unlink(tmp.c_str());
        return false;
      }
      if (rename(tmp.c_str(), dst.c_str()) != 0) {
        unlink(tmp.c_str());
        return false;
      }
    }
    fsyncFileOrDir(globalConfig.OutputDir, true);
    return true;
  }

  void storeNow() {
    if (!cacheActive || !staged) return;  // capture is sound only post-staging under the lock
    std::vector<std::string> files;
    if (!listNamespaceFiles(globalConfig.OutputDir, &files)) {
      fprintf(stderr, "[genCache] store aborted: cannot list %s; staging state unverified\n",
              globalConfig.OutputDir.c_str());
      return;
    }
    if (files.empty()) {
      fprintf(stderr, "[genCache] store skipped: no namespace files found in %s\n",
              globalConfig.OutputDir.c_str());
      return;
    }
    std::string tmpPattern = cacheDir + "/.tmp.XXXXXX";
    std::vector<char> tmpl(tmpPattern.begin(), tmpPattern.end());
    tmpl.push_back('\0');
    if (mkdtemp(tmpl.data()) == nullptr) {
      fprintf(stderr, "[genCache] store skipped: cannot create temp dir in %s (%s)\n",
              cacheDir.c_str(), strerror(errno));
      return;
    }
    const std::string tmp = tmpl.data();
    std::vector<EntryArtifact> artifacts;
    for (const std::string& f : files) {
      if (!isSafeArtifactName(f)) continue;
      std::string bytes;
      if (!readFileBytes(globalConfig.OutputDir + "/" + f, &bytes)) {
        fprintf(stderr, "[genCache] store aborted: cannot read %s\n", f.c_str());
        removeTreeBestEffort(tmp);
        return;
      }
      if (!writePrivateFile(tmp + "/" + f, bytes)) {
        fprintf(stderr, "[genCache] store aborted: cannot copy %s\n", f.c_str());
        removeTreeBestEffort(tmp);
        return;
      }
      EntryArtifact a;
      a.name = f;
      a.size = bytes.size();
      a.sha256 = sha256Hex(bytes.data(), bytes.size());
      artifacts.push_back(a);
    }
    if (artifacts.empty()) {
      fprintf(stderr, "[genCache] store aborted: no usable artifacts\n");
      removeTreeBestEffort(tmp);
      return;
    }
    std::string keyFile;
    keyFile += std::string(kEntryMagic) + "\n";
    keyFile += "key=" + keyHex + "\n";
    keyFile += "--key-material--\n";
    keyFile += buildKeyMaterial(module, firSha, firLen, binSha, binLen);
    keyFile += "--artifacts--\n";
    for (const EntryArtifact& a : artifacts) {
      keyFile += a.sha256 + " " + std::to_string(a.size) + " " + a.name + "\n";
    }
    if (!writePrivateFile(tmp + "/KEY", keyFile)) {
      fprintf(stderr, "[genCache] store aborted: cannot write KEY\n");
      removeTreeBestEffort(tmp);
      return;
    }
    fsyncFileOrDir(tmp, true);
    /* rename() onto an existing non-empty directory fails; an entry with
       this exact key that is still valid is kept as-is. */
    if (rename(tmp.c_str(), entryDir.c_str()) != 0) {
      if (errno == ENOTEMPTY || errno == EEXIST) {
        std::vector<EntryArtifact> existing;
        std::string reason;
        if (loadEntry(entryDir, keyHex, &existing, &reason)) {
          removeTreeBestEffort(tmp);
          fprintf(stderr, "[genCache] store skipped: valid entry already present\n");
          return;
        }
        const std::string trash = cacheDir + "/.trash." + std::to_string((long)getpid()) + "." +
                                  std::to_string((unsigned long)time(nullptr));
        if (rename(entryDir.c_str(), trash.c_str()) != 0) {
          fprintf(stderr, "[genCache] store aborted: corrupt entry cannot be replaced (%s)\n",
                  strerror(errno));
          removeTreeBestEffort(tmp);
          return;
        }
        removeTreeBestEffort(trash);
        if (rename(tmp.c_str(), entryDir.c_str()) != 0) {
          fprintf(stderr, "[genCache] store aborted: rename failed (%s)\n", strerror(errno));
          removeTreeBestEffort(tmp);
          return;
        }
      } else {
        fprintf(stderr, "[genCache] store aborted: rename failed (%s)\n", strerror(errno));
        removeTreeBestEffort(tmp);
        return;
      }
    }
    fsyncFileOrDir(cacheDir, true);
    fprintf(stderr, "[genCache] stored module=%s key=%s artifacts=%zu\n",
            module.c_str(), keyHex.c_str(), artifacts.size());
  }

  void verifyAgainstFresh() {
    if (!cacheActive || !staged) {
      fprintf(stderr, "[genCache] verify skipped: namespace was not staged under the lock\n");
      return;
    }
    /* Re-validate the entry now (not the lookup-time result) so the check
       covers the whole window up to the comparison. */
    std::vector<EntryArtifact> cached;
    std::string reason;
    if (!loadEntry(entryDir, keyHex, &cached, &reason)) {
      fprintf(stderr, "[genCache] VERIFY FAILED: entry no longer valid (%s)\n", reason.c_str());
      std::abort();
    }
    std::vector<std::string> fresh;
    if (!listNamespaceFiles(globalConfig.OutputDir, &fresh)) {
      fprintf(stderr, "[genCache] verify skipped: cannot list %s\n",
              globalConfig.OutputDir.c_str());
      return;
    }
    std::map<std::string, std::string> freshSha;
    for (const std::string& f : fresh) {
      std::string bytes;
      if (!readFileBytes(globalConfig.OutputDir + "/" + f, &bytes)) {
        fprintf(stderr, "[genCache] VERIFY FAILED: cannot read fresh artifact %s\n", f.c_str());
        std::abort();
      }
      freshSha[f] = sha256Hex(bytes.data(), bytes.size());
    }
    bool ok = fresh.size() == cached.size();
    if (ok) {
      for (const EntryArtifact& a : cached) {
        std::map<std::string, std::string>::const_iterator it = freshSha.find(a.name);
        if (it == freshSha.end() || it->second != a.sha256) {
          fprintf(stderr, "[genCache] VERIFY FAILED: artifact %s differs from cache\n",
                  a.name.c_str());
          ok = false;
          break;
        }
      }
    } else {
      fprintf(stderr, "[genCache] VERIFY FAILED: artifact count differs (fresh=%zu cached=%zu)\n",
              fresh.size(), cached.size());
      for (const std::string& f : fresh) {
        bool known = false;
        for (const EntryArtifact& a : cached) {
          if (a.name == f) {
            known = true;
            break;
          }
        }
        if (!known) fprintf(stderr, "[genCache]   fresh-only: %s\n", f.c_str());
      }
      for (const EntryArtifact& a : cached) {
        if (freshSha.count(a.name) == 0) {
          fprintf(stderr, "[genCache]   cached-only: %s\n", a.name.c_str());
        }
      }
    }
    if (!ok) std::abort();
    fprintf(stderr, "[genCache] verify ok: %zu artifacts byte-identical to cache\n",
            cached.size());
  }
};

Cache* Cache::create() {
  const char* dir = std::getenv("GSIM_GEN_CACHE");
  const char* verify = std::getenv("GSIM_GEN_CACHE_VERIFY");
  if (dir == nullptr || dir[0] == '\0') {
    if (verify != nullptr && verify[0] != '\0') {
      fprintf(stderr, "[genCache] GSIM_GEN_CACHE_VERIFY ignored: GSIM_GEN_CACHE is not set\n");
    }
    return nullptr;
  }
  /* A cache hit skips the stages that produce debug artifacts, so any dump
     request disables the cache entirely (fail-closed). */
  if (globalConfig.EnableDumpGraph || globalConfig.DumpAssignTree || globalConfig.DumpConstStatus) {
    fprintf(stderr, "[genCache] disabled: debug dump flags requested, regenerating fully\n");
    return nullptr;
  }
  Impl* impl = new Impl();
  impl->cacheDir = dir;
  if (verify != nullptr && strcmp(verify, "1") == 0) impl->verify = true;
  /* Emitter fingerprint: the exact running binary. */
  {
    Sha256 sha;
    int fd = open("/proc/self/exe", O_RDONLY);
    if (fd < 0) {
      fprintf(stderr, "[genCache] disabled: cannot open /proc/self/exe (%s)\n", strerror(errno));
      delete impl;
      return nullptr;
    }
    char buf[65536];
    ssize_t n;
    while ((n = read(fd, buf, sizeof buf)) > 0) {
      sha.update(buf, (size_t)n);
      impl->binLen += (uint64_t)n;
    }
    const int savedErrno = errno;
    close(fd);
    if (n < 0) {
      fprintf(stderr, "[genCache] disabled: cannot read /proc/self/exe (%s)\n", strerror(savedErrno));
      delete impl;
      return nullptr;
    }
    unsigned char digest[32];
    sha.final(digest);
    impl->binSha = toHex(digest, 32);
  }
  if (!mkdirRecursive(impl->cacheDir)) {
    fprintf(stderr, "[genCache] disabled: cannot use cache dir %s (%s)\n", dir, strerror(errno));
    delete impl;
    return nullptr;
  }
  pruneStaleScratchDirs(impl->cacheDir);
  Cache* cache = new Cache();
  cache->impl_ = impl;
  fprintf(stderr, "[genCache] enabled dir=%s verify=%d emitter-sha256=%s\n",
          impl->cacheDir.c_str(), impl->verify ? 1 : 0, impl->binSha.c_str());
  return cache;
}

void Cache::hashFirInput(const char* bytes, size_t len) {
  impl_->firSha = sha256Hex(bytes, len);
  impl_->firLen = len;
  impl_->firHashed = true;
}

bool Cache::lookup(const std::string& moduleName) {
  Impl& impl = *impl_;
  if (!impl.firHashed) {
    fprintf(stderr, "[genCache] miss: FIR input was not hashed\n");
    return false;
  }
  impl.module = moduleName;
  const std::string material =
      buildKeyMaterial(moduleName, impl.firSha, impl.firLen, impl.binSha, impl.binLen);
  impl.keyHex = sha256Hex(material.data(), material.size());
  impl.entryDir =
      impl.cacheDir + "/" + sanitizePathComponent(moduleName) + "." + impl.keyHex.substr(0, 32);
  /* Everything below mutates or observes OutputDir; serialize first. */
  if (!impl.acquireOutputLock()) return false;
  std::vector<EntryArtifact> artifacts;
  std::string reason;
  if (!impl.loadEntry(impl.entryDir, impl.keyHex, &artifacts, &reason)) {
    fprintf(stderr, "[genCache] miss: module=%s key=%s (%s)\n", moduleName.c_str(),
            impl.keyHex.c_str(), reason.c_str());
    return false;
  }
  if (impl.verify) {
    /* Validation mode: regenerate fully and byte-compare against the entry. */
    impl.verifyPending = true;
    fprintf(stderr, "[genCache] hit (verify mode: regenerating to compare) module=%s key=%s\n",
            moduleName.c_str(), impl.keyHex.c_str());
    return false;
  }
  if (!impl.stageNamespaceOut() || !impl.restore(artifacts)) {
    fprintf(stderr, "[genCache] miss: restore failed for module=%s, regenerating\n",
            moduleName.c_str());
    return false;
  }
  /* Hit path returns to main immediately, so reclaim staging here: the old
     files were replaced by the verified restore above. */
  removeTreeBestEffort(impl.stagingDir);
  impl.stagingDir.clear();
  fprintf(stderr, "[genCache] hit: module=%s key=%s artifacts=%zu restored to %s\n",
          moduleName.c_str(), impl.keyHex.c_str(), artifacts.size(),
          globalConfig.OutputDir.c_str());
  return true;
}

void Cache::beginEmission() {
  if (impl_->module.empty()) return;  // lookup() never ran; nothing is captured later
  impl_->stageNamespaceOut();
}

void Cache::finishEmission() {
  if (impl_->module.empty() || !impl_->cacheActive) return;
  if (impl_->verifyPending) {
    impl_->verifyAgainstFresh();
  } else {
    impl_->storeNow();
  }
  /* Staging held the previous contents only so this run could replace them
   * cleanly; drop it now. On a miss-through (cacheActive false) staging is
   * kept for manual recovery and logged above. */
  if (!impl_->stagingDir.empty()) removeTreeBestEffort(impl_->stagingDir);
}

Cache::~Cache() {
  if (impl_ != nullptr && impl_->lockFd >= 0) close(impl_->lockFd);
  delete impl_;
}

}  // namespace genCache
