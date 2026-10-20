/*
  genCache: module-level generation cache (default OFF).

  Enabled by GSIM_GEN_CACHE=<dir>. A gsim invocation compiles exactly one
  module (the FIR circuit, flattened across every module definition in the
  input file) into one emitted artifact set. The cache stores that artifact
  set keyed by every input that can change one emitted byte:

    key = sha256( cache format version
                | module (circuit) name
                | sha256 of the complete FIR input file bytes
                | sha256 of the running gsim binary (/proc/self/exe)
                | canonical serialization of all generation-relevant options
                | generation-relevant environment (GSIM_THREADS,
                  GSIM_EMIT_RESET_CHUNK, GSIM_MT_DYNAMIC_QUERY) )

  Fail-closed: any ambiguity (missing key component, unreadable/corrupt/
  partial/foreign cache entry) falls back to full regeneration. A stale hit
  is impossible by construction because every output-affecting input is in
  the key and every stored artifact is integrity-checked against the
  manifest on every hit. Before this process writes into OutputDir it takes
  an exclusive flock on <OutputDir>/.gencache.lock and moves pre-existing
  same-namespace files (which cannot be attributed by name: Foo10.cpp is
  both Foo's part 10 and module Foo1's part 0) into a private staging dir,
  so stored or verified artifacts were provably produced by this
  invocation. A busy lock aborts (single-writer-per-OutputDir contract;
  concurrent runs sharing an OutputDir without the cache are unsupported
  pre-existing behavior); any staging or listing failure is a miss-through
  (full regeneration, no store, no delete).

  GSIM_GEN_CACHE_VERIFY=1 (validation mode, requires GSIM_GEN_CACHE): cache
  hits still run the full generation, and the freshly emitted artifacts are
  byte-compared against the cached entry; any mismatch aborts. See
  genCache.cpp for the key-completeness argument and the failure-mode table.
*/

#ifndef GSIM_GEN_CACHE_H
#define GSIM_GEN_CACHE_H

#include <cstddef>
#include <string>

namespace genCache {

class Cache {
 public:
  /* Returns nullptr unless GSIM_GEN_CACHE names a usable cache directory and
     no debug-dump flag is active (dump runs always regenerate fully). */
  static Cache* create();

  /* Must be called before parseFIR(): the parser splits the input buffer by
     writing NUL bytes, so the raw file bytes must be hashed first. */
  void hashFirInput(const char* bytes, size_t len);

  /* Called after parseFIR() with the circuit (top module) name.
     Returns true only when a fully verified entry was restored into
     OutputDir and the whole graph pipeline may be skipped. Returns false on
     any miss/ambiguity, or in verify mode (which must regenerate). */
  bool lookup(const std::string& moduleName);

  /* Call immediately before the final emitter stage: under the exclusive
     OutputDir lock, moves pre-existing namespace files aside so what is
     captured afterwards is exactly what this emission produces. */
  void beginEmission();

  /* Call immediately after the final emitter stage: stores the freshly
     emitted artifacts (atomic rename), or — in verify mode after a verified
     hit — byte-compares them against the cached entry and aborts on any
     mismatch. */
  void finishEmission();

 private:
  Cache() = default;
  ~Cache();
  struct Impl;
  Impl* impl_ = nullptr;
};

}  // namespace genCache

#endif  // GSIM_GEN_CACHE_H
