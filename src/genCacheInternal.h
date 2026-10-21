/*
  genCacheInternal.h: filesystem / hashing / identity primitives shared by
  genCache.cpp (module-level artifact cache) and genCheckpoint.cpp (pre-Final
  graph checkpoint, GSIM_GEN_CHECKPOINT). Internal to src/; not installed.

  Everything here is fail-closed utility code: callers must treat a false
  return (with *reason set where provided) as "cannot prove identity" and
  degrade to their documented fallback, never to a guess.
*/

#ifndef GSIM_GEN_CACHE_INTERNAL_H
#define GSIM_GEN_CACHE_INTERNAL_H

#include <cstddef>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

namespace genCacheUtil {

/* ------------------------------------------------------------- hashing */
std::string toHex(const unsigned char* bytes, size_t n);
std::string sha256Hex(const void* data, size_t n);

/* ------------------------------------------------------------- fs utils */
bool readFileBytes(const std::string& path, std::string* out);
bool listDirFiles(const std::string& dir, std::vector<std::string>* out);
/* All non-hidden dirents, sorted; directories carry a trailing '/', other
   non-regular entries a trailing '?'. A failed listing is NOT an empty
   directory: treat false as fail-closed. */
bool listDirEntries(const std::string& dir, std::vector<std::string>* out);
bool isDirectory(const std::string& path);
bool isRegularFile(const std::string& path);
bool mkdirOne(const std::string& path);
bool mkdirRecursive(const std::string& path);
void fsyncFileOrDir(const std::string& path, bool directory);
bool writePrivateFile(const std::string& path, const std::string& data);
void removeTreeBestEffort(const std::string& dir);
/* Best-effort GC of scratch dirs (.staging.* / .tmp.* / .trash.*) older
   than 72h; every error is ignored. */
void pruneStaleScratchDirs(const std::string& cacheDir);

/* ------------------------------------------------------------ key text */
/* Escape any byte outside printable-non-space ASCII so key material lines
   stay one-per-field and the serialized blob is injective. */
std::string escapeKeyToken(const std::string& s);
/* Path-safe component for entry directories. */
std::string sanitizePathComponent(const std::string& s);
bool isHexString(const std::string& s, size_t len);

/* -------------------------------------------------- probes and identity */
bool nameInList(const char* const* list, size_t count, const std::string& name);
bool endsWith(const std::string& s, const char* suffix);
std::string shellQuote(const std::string& s);
/* Runs cmd through /bin/sh and captures stdout; false on start failure or
   nonzero exit. */
bool captureCommandOutput(const std::string& cmd, std::string* out);
std::string parentDirOf(const std::string& path);
/* Source tree root: must contain src/, include/, parser/ and the
   src/genCache.cpp sentinel. */
bool isSourceRoot(const std::string& dir);

/* /proc/self/exe path. */
bool resolveSelfExe(std::string* exePath, std::string* reason);
/* First line of `clang++ --version` (compiler identity). */
bool resolveCompilerIdentity(std::string* compilerLine, std::string* reason);
/* sha256 over the sorted, address-stripped ldd(1) dependency list of exePath
   (allocator identity: the Makefile MALLOC probe links -ljemalloc /
   -lmimalloc / -ltcmalloc_minimal, which changes that list). */
bool resolveAllocatorIdentity(const std::string& exePath, std::string* depsSha,
                              std::string* reason);
/* Source tree root near the binary or cwd (three dirname levels up from the
   binary dir, then shallower, then cwd-based fallbacks). */
bool findSourceRoot(const std::string& exePath, std::string* root, std::string* reason);

}  // namespace genCacheUtil

#endif  // GSIM_GEN_CACHE_INTERNAL_H
