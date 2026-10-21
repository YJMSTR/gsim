/*
  genCheckpoint: pre-Final global-graph checkpoint (default OFF).

  Enabled by GSIM_GEN_CHECKPOINT=<dir>. gsim spends ~87% of a cold run in
  the pre-emission pipeline (parseFIR .. last removeDeadNodes, main.cpp)
  building one global graph that the Final emitter stage then consumes.
  Emitter/scheduler source edits and mt-knob sweeps change ONLY how Final
  consumes that graph, so the fully built graph can be checkpointed at the
  removeDeadNodes -> beginEmission boundary and restored to skip the whole
  pre-Final pipeline (Stage-2 / GCKPT-1 of the gencache incremental plan).

  Entries live under <dir>/ckpt/<key64hex>/ (KEY manifest + graph.ckpt
  binary body), sharing the genCache directory conventions: staging-free
  atomic publish via rename, scratch-dir GC, sha-verified manifests.

  Key (see genCheckpoint.cpp for the completeness argument):

    key = sha256( ckpt key format version
                | sha256 of the complete FIR input file bytes
                | sha256 of the PRE-FINAL source closure (every parser/
                  input, every pre-Final src TU, every shared header; NOT
                  the Final-only emitter/scheduler TUs and headers) +
                  compiler identity + allocator identity
                | the only Config fields any pre-Final pass reads
                  (sep_module, sep_aggr) )

  Fail-closed everywhere: dump flags disable the checkpoint outright (a
  restore would skip the stages that produce those artifacts); any closure
  ambiguity (unknown TU/header, missing enumerated file, unreadable source,
  source newer than the running binary, failed clang++/ldd probe) disables
  the feature for the run (full pipeline, no store); any entry or body
  anomaly is a miss (full pipeline). Restore trusts the key and never
  re-runs the pipeline to double-check.

  GSIM_GEN_CHECKPOINT_VERIFY=1 (validation mode, requires
  GSIM_GEN_CHECKPOINT): fresh runs additionally store a canonical body
  fingerprint and compare it against an existing same-key entry, reporting
  drift statistics (the pre-Final pipeline is known to have
  pointer-order-sensitive trajectories upstream; identical fingerprints
  across fresh runs mean the stored trajectory is the current one).
*/

#ifndef GSIM_GEN_CHECKPOINT_H
#define GSIM_GEN_CHECKPOINT_H

#include <cstddef>
#include <string>

class graph;

namespace genCheckpoint {

class Checkpointer {
 public:
  /* Returns nullptr unless GSIM_GEN_CHECKPOINT names a usable cache
     directory, no debug-dump flag is active, and the pre-Final source
     closure could be resolved. */
  static Checkpointer* create();

  /* Must be called before parseFIR(): the parser splits the input buffer
     by writing NUL bytes, so the raw file bytes must be hashed first. */
  void hashFirInput(const char* bytes, size_t len);

  /* Called after the genCache lookup point. On true, *out holds the fully
     restored graph (ids, pointers, trees and graph vectors rebuilt) and the
     caller skips AST2Graph .. removeDeadNodes entirely, proceeding straight
     to the emission stage. On any miss/ambiguity returns false and the
     caller runs the full pipeline. */
  bool tryRestore(graph** out);

  /* Called after the last removeDeadNodes(), before beginEmission():
     serializes the graph and publishes it under <dir>/ckpt/. */
  void store(graph* g);

 private:
  Checkpointer() = default;
  ~Checkpointer();
  struct Impl;
  Impl* impl_ = nullptr;
};

}  // namespace genCheckpoint

#endif  // GSIM_GEN_CHECKPOINT_H
