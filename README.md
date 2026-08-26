# GSIM: A Fast RTL Simulator for Large-Scale Designs

GSIM accepts chirrtl, and compiles it to C++

## Prerequisites

+ Install [GMP](https://gmplib.org/), [clang 19(+)](https://clang.llvm.org/).

## Quike Start

+ GSIM provides 4 RISC-V cores ready for simulation: [ysyx3](https://ysyx.oscc.cc/), [Rocket](https://github.com/chipsalliance/rocket-chip), [BOOM](https://github.com/riscv-boom/riscv-boom), [XiangShan](https://github.com/OpenXiangShan/XiangShan).

+ To try GSIM, using
    ```
    $ make init
    $ make run dutName=core
    ```
+ Set core to `ysyx3`, `rocket`, `small-boom`, `large-boom`, `minimal-xiangshan` or `default-xiangshan`

## Usage

+ Run `make build-gsim` to build GSIM
+ Build a static binary locally with `make STATIC=1 build-gsim` (CI artifacts are built statically).
+ Run `build/gsim/gsim $(chirrtl-file)` to compile chirrtl to C++
+ Refer to `build/gsim/gsim --help` for more information
+ See [C++ harness example](https://github.com/jaypiper/simulator/blob/master/emu/emu.cpp) to know how it interacts with the emitted C++ code.

## Multithreaded dense executor (gsim-mt)

This branch adds a multithreaded dense execution engine: the design is condensed into SCCs, contracted into MTasks, statically assigned to worker threads, and emitted as a fixed-order executor synchronized by owner-ready tokens, with an optional bounded-lookahead fill for stalled workers. On the XiangShan RISC-V SoC design (the SimTop netlist) it reaches **5.47s for 50K CoreMark cycles at 32 threads (2.04x Verilator T16)** and 6.43s at 16 threads, bit-exact against NEMU (C5000 528/4941/0x800027ba, C50000 46540/50000).

### Benchmarks (XiangShan SimTop, CoreMark C50000, NEMU bit-exact)

**RTL under test: XiangShan Default config (Kunminghu)** — `CONFIG=DefaultConfig` (XiangShan Makefile default), full out-of-order core with vector extension (the FIR contains the VPU); NOT the Minimal config. All numbers on this page are for the Default-config `SimTop` netlist (886 MB FIR, 45,163 SCCs condensed to 8,436–12,275 MTasks depending on thread count).

| Threads | gsim-mt (best config) | Verilator (same machine/RTL) | gsim-mt speedup |
|---|---:|---:|---:|
| 1 | 18.7 s (sparse serial) | 71.95 s | 3.85× |
| 2 | 17.7 s (serial-fast fallback¹) | — | — |
| 4 | 17.8 s (serial-fast fallback¹) | — | — |
| 6 | 12.5 s (dense) | — | — |
| 8 | 8.9–9.2 s (dense, pinned²) | 14.18 s | ~1.6× |
| 16 | 6.43 s (dense, registered; 6.08–6.26 s pinned today) | 11.15 s | 1.73× |
| 32 | **5.47 s** (dense, registered; 5.33–5.37 s pinned today) | 9.73–9.84 s | 1.78–1.80× |
| 48 | 5.75 s (dense, pinned) | — | — |
| 64 | 5.89 s (dense, pinned) | — | — |

Scaling peaks at 32 threads on this design+machine: beyond it, cross-CCD/socket token latency (290–322ns vs 24.5ns same-CCD) and per-MTask protocol cost outgrow the shorter chains (T48 with doubled MTask granularity is 6.60s — worse, ruling out mis-tuning). 
Reference: clean single-thread gsim 18.96 s; sparse-vs-dense crossover is at ~5 threads (sparse wins ≤T4, dense wins ≥T6). All gsim-mt numbers are same-workload NEMU-exact C50000 runs on an AMD EPYC 9654.

¹ `GSIM_MT_SPARSE_SERIAL_FAST_MAX_WORKERS=4`: below ~6 workers the sparse runtime's batch bookkeeping costs more than the parallelism it finds, so the model should run the lean serial-fast path (beats even plain single-thread).
² `GSIM_MT_CPU_AFFINITY=auto`: unpinned dense runs at ≥8 workers show a migration-churn bistability (random 2–3× slow modes); pinning is required for these numbers.

Build flags: plain `-O3 -march=znver4`. PGO was measured and is **not** used — a clang `-fprofile-generate/-fprofile-use` pipeline on this model runs ~7.5% slower (5.76s vs 5.35s, 5-pair pinned, NEMU-exact); profile-guided layout degrades the 12K-hot-body instruction-cache behavior vs the default layout. LTO is not used either.

All `GSIM_MT_DENSE_*` knobs are default-off; with none of them set, generation output is byte-identical to upstream.

### Toolchain pipeline cost (generation + build)

Same machine (EPYC 9654, `-j48` builds), same input (XiangShan Default-config `SimTop`, 886 MB FIR / 2,089 Verilog files), clang 23 for the gsim model builds. Note the outputs differ: Verilator emits a 16-thread model, upstream gsim a serial model, gsim-mt the 16-thread dense model — this table compares pipeline cost, not simulation speed (that is the table above).

| Toolchain | Model generation | Model build (-O3 -march=znver4) | Total | Model size |
|---|---:|---:|---:|---:|
| Verilator 5.034 | 29m40s (verilate, single-threaded) | 3m22s (864 TUs; emu 95 MB) | **33m02s** | 2.0 GB |
| upstream gsim (serial) | 10m15s | 4m00s (257 TUs) | **14m14s** | 2.5 GB |
| gsim-mt, fresh generation | 7m38s (jemalloc-linked generator; 10m39s with the system allocator) | ~24m53s (fresh model is same shape, 1,214 TUs) | **~32m31s** | 12 GB |
| gsim-mt, champion seed replay | 19m30s (default, canon-verified; 8m39s with `GSIM_SEED2_VERIFY_CANON=0` — byte-identical output) | 24m53s (champion model) | **44m23s** | 12 GB |
| gsim-mt, `GSIM_MT_DENSE_ONLY_CODEGEN=1` | (same generation cost) | **23m08s** (751 TUs) | **~30m46s** fresh / **~31m47s** replay-verify-off | **7.1 GB** |
| gsim-mt, `GSIM_MT_DENSE_ONLY_CODEGEN=2` | (same) | 23m51s (502 TUs) | **~31m29s** fresh | **4.8 GB** |
| gsim-mt, level 2 + `GSIM_SHORT_NAMES=1` | (same) | 23m27s (92 TUs) | **~31m05s** fresh | **1.1 GB** |
| gsim-mt, all three knobs (T16) | (same) | **1m42s** (89 TUs; emu 73 MB) | **~9m20s** fresh | **1.1 GB** |
| gsim-mt, all three knobs (T32) | 8m36s (champion seed replay) | **1m46s** (89 TUs; emu 74 MB) | **~10m22s** replay | **1.1 GB** |

Notes:

- gsim-mt fresh generation costs about the same as upstream's serial generation (7m38s vs 10m15s with jemalloc; 10m39s vs 10m15s with the system allocator): the vcontract MTask-contraction search (mergeLoop 32s) fits inside the savings from the parallel/canon emission work. The generator links a thread-caching allocator when available (`MALLOC=auto`, upstream #116) — measured -28.8% on fresh generation; seed replay is unaffected (19m30s) because its cost is the deterministic schedule search and canon verification, not allocation; `GSIM_SEED2_VERIFY_CANON=0` skips the v1 serial canon mixes (the integrity check, not an input to the applied order) and replays in 8m39s with byte-identical output. Seed replay reproduces the champion schedule exactly (9/9 canon checks, facts verbatim) and its output is byte-identical across allocators; only *fresh* (seedless) search tie-breaks are allocator-sensitive, landing on a different but self-consistent schedule (8,577 vs 8,478 MTasks). Note also that replay pins the *schedule*, not the text: the emitted model is byte-identical only against a model generated by the same generator revision — this tip's output differs from the older v457-registered artifact in 5 of 1,213 files (generator evolution added default-on fields), same schedule throughout. One more determinism boundary, root-caused 2026-08-25: with a thread-caching allocator linked (the `MALLOC=auto` default), *fresh* generation text is not guaranteed run-to-run identical — the array-split pass selects candidates by iterating pointer-ordered sets (`std::set<Node*>` in `splitArray.cpp`), so declaration/assignment order can permute (schedule facts and simulation behavior are unaffected, and NEMU-exactness is unaffected; seed replay is byte-exact run-to-run under either allocator, verified). For byte-reproducible fresh output build the generator with `MALLOC=system`.
- The gsim-mt model build is the bottleneck: its dense executor emits per-worker major-text bodies plus the owner-ready synchronization machinery (12 GB of C++ vs upstream's 2.5 GB), and the build is -O3-optimizer-bound on large translation units (header parsing is ~1% — precompiled headers were measured and rejected; a duplicate-body census found 0.0% redundant text, so no dedup lever either). Peak generation memory: 81 GB (fresh) / 297 GB (seed replay — the 2M-line applied-seed validation structures stay resident) / 104 GB (Verilator verilate) / 76 GB (upstream).
- Level 2 (`=2`) additionally drops the SerialFast subSteps (the T≤4 fallback): a level-2 model runs dense only, or aborts. `GSIM_SHORT_NAMES=1` interns the 2.03M internal node names to `v<idx>` at emission time (I/O accessor members and blackbox function names keep their full spellings; `// orig=` comments keep the mapping debuggable). Combined: **12.4 GB → 1.1 GB (-91%), 92 TUs**, NEMU bit-exact, perf ~neutral (+1.4% mean with one outlier pair — identifier shortening shifts znver4 codegen alignment slightly). The build wall barely moves (24m53s → 23m27s across the whole ladder) because it is bound by -O3 optimization+link of the hot dense bodies, which no text-level reduction removes; splitting TUs finer (2048 KB → 2468 TUs, 27m35s) or coarser (32768 KB → 206 TUs) both measure worse than the 8192 KB default.
- **These three knobs are now the DEFAULT under the dense recipe** (`--mt-helper-mode=mt-level-dispatch` + `GSIM_MT_DENSE_EXECUTOR_CODEGEN=1`); export them as `=0` to get the legacy full emission byte-identically. Outside the dense recipe all three default off.
- `GSIM_EMIT_RESET_CHUNK=<n>` (legacy-default off) attacks the last build-wall component: the emitted `subResetN` bodies carry the whole reset statement stream (≈176K assignments for one function) inside a single giant function, and the clang frontend is superlinear on that shape — one such TU measured 1366s vs 2.8s with the body stubbed (a 494× effect independent of -O level). The knob splits the stream into chain-called helpers `subResetN_cK` of ≤n statements, re-opening the identical `if(reset)` conditions inside each chunk (pure member loads; side-effect-free). Combined pipeline: **build 23m27s → 1m42s (-93%)**, faster than both Verilator and upstream gsim, with NEMU bit-exact output and neutral C50000 perf.
- The runtime payoff for this pipeline cost is in the benchmark table above (6.43s vs Verilator 11.15s at 16 threads).
- `GSIM_MT_DENSE_ONLY_CODEGEN=1` (generation-time, default-off, byte-identical when off) drops the sparse-dispatch runtime's text from the model: the buffered `mtTaskN(flag, ActivationDelta&)` helpers, the plain serial `subStepN()` scan, and the coarse-region/pure-batch runners — three of the four evaluation-body copies the default emission ships, which serve runtimes the dense executor never executes. The dense-only model keeps the dense path and the serial-fast fallback (workers ≤ `GSIM_MT_SPARSE_SERIAL_FAST_MAX_WORKERS`, no profile) bit-exact; any other configuration aborts with a clear message. Measured: model 12.4→7.1 GB (-42%), emu 215→110 MB, build 24m53s→23m08s, C50000 perf neutral (-1.19%, 5-pair), NEMU bit-exact (C5000 528/4941/0x800027ba, C50000 46540/50000). The build wall shrinks less than the text because it is -O3-optimizer-bound on the hot TUs, which are unchanged; the dropped cold code was cheap to compile.

### Full chain from XiangShan source (end-to-end, T32, measured 2026-08-26)

The quick start below assumes `SimTop.fir` already exists. The complete chain from XiangShan source, with per-stage walls on the reference machine (EPYC 9654, `-j48`):

| Stage | Command (essence) | Wall | Output |
|---|---|---:|---|
| 1. XiangShan elaboration | `make sim-verilog CONFIG=DefaultConfig` (mill; needs `NOOP_HOME` set) | ~15–20 min | 2,089 `.sv` / `SimTop.fir` (886 MB) |
| 2. gsim generation | champion recipe below (T32: `MAXMT=2400`), `GSIM_SCHEDULE_SEED2=<seed>` + `GSIM_SEED2_VERIFY_CANON=0` | 8m36s (champion seed replay) / 7m38s fresh | model (89 TUs, 1.1 GB; knobs default-on) + schedule JSON |
| 3. model build | difftest `gsim-build-emu` (`EMU_THREADS=32`, `-O3 -march=znver4 -j48`) | **1m46s** (23m46s with all three knobs disabled) | `emu` 74 MB |
| 4. simulation | `GSIM_THREADS=32 GSIM_MT_EXECUTOR=dense GSIM_MT_CPU_AFFINITY=auto ./emu -i <workload> [-C N]` | Linux `linux.bin` 100K cycles ≈ **10.9 s** (1M ≈ 105 s); CoreMark C50000 ≈ 5.5 s | cycle/instruction counters |

Reproducibility: with the champion seed, stage 2 replays the registered schedule exactly (facts line verbatim, 9/9 canon pins) and stages 3–4 land on the registered performance (Linux 100K 10.6–11.3 s vs registered mean 10.8 s; 1M 105.3 s vs 107.4 s). Note `emu` requires `NEMU_HOME` pointing at a tree containing `build/riscv64-nemu-interpreter-so` even for non-difftest runs (the difftest library is always linked).

### Quick start (XiangShan SimTop netlist, 32 threads)


```bash
export GSIM_THREADS=32
export GSIM_MT_DENSE_EXECUTOR_CODEGEN=1
export GSIM_MT_DENSE_FORWARD_ACTIVATION_ONLY=1
export GSIM_MT_DENSE_XTHREAD_DEPS_ONLY=1
export GSIM_MT_DENSE_TRANSITIVE_REDUCE_EDGES=1
export GSIM_MT_DENSE_SPLIT_WORKER0_MTASKS=1
export GSIM_MT_DENSE_STATIC_EMPTY_ELIDE=1
export GSIM_MT_DENSE_OWNER_BANK_COUNTERS=1
export GSIM_MT_DENSE_OWNER_READY_FLAGS=1
export GSIM_MT_DENSE_WORKER_MAJOR_TEXT=1
export GSIM_MT_DENSE_VCONTRACT=1
export GSIM_MT_DENSE_VCONTRACT_SIBLING=0
export GSIM_MT_DENSE_VCONTRACT_CAP=1
export GSIM_MT_DENSE_VCONTRACT_MAXMT=2400   # use 800 for 16 threads
export GSIM_MT_DENSE_VCONTRACT_PROPCP=1
export GSIM_MT_DENSE_SCHED_ORDER=1
export GSIM_MT_DENSE_VCONTRACT_EDGE_CPWO=1
export GSIM_MT_WORKER_POOL_FLAG_JOIN=1
export GSIM_MT_DENSE_UNPIN_SPECIAL=1
export GSIM_MT_DENSE_LOOKAHEAD=128          # bounded lookahead window (0/unset = off)

build/gsim/gsim --supernode-max-size=30 --cpp-max-size-KB=8192 \
  --sep-mod=__DOT__ --sep-aggr=__DOT__ --mt-helper-mode=mt-level-dispatch \
  --dir <OUT_DIR> SimTop.fir                 # <OUT_DIR> must exist; ~8-9 min for XiangShan (see pipeline table)
```

Then build and run the emitted model with your MT harness (a difftest-based flow is one option), with `GSIM_THREADS=32 GSIM_MT_EXECUTOR=dense GSIM_MT_CPU_AFFINITY=auto` at runtime.

### Knob overview

| Knob | Default | Effect |
|---|---|---|
| `GSIM_MT_DENSE_EXECUTOR_CODEGEN` | off | Emit the dense executor + schedule JSON report |
| `GSIM_MT_DENSE_VCONTRACT` | off | Verilator-style SCC contraction into MTasks |
| `GSIM_MT_DENSE_VCONTRACT_MAXMT` | 1600 | MTask count cap (granularity; T32 optimum 2400, T16 optimum 800 on XiangShan — the landscape is non-smooth, sweep per design) |
| `GSIM_MT_DENSE_VCONTRACT_EDGE_CPWO` | off | Critical-path-weighted edge scoring (load-bearing) |
| `GSIM_MT_DENSE_VCONTRACT_PROPCP` | off | Periodic exact critical-path recompute during contraction |
| `GSIM_MT_DENSE_OWNER_READY_FLAGS` | off | Owner-ready token protocol (single-writer, parity tags) |
| `GSIM_MT_DENSE_TRANSITIVE_REDUCE_EDGES` | off | Drop transitively redundant dependency edges |
| `GSIM_MT_DENSE_LOOKAHEAD` | 0 (off) | Bounded out-of-order fill window per worker chain |
| `GSIM_MT_DENSE_UNPIN_SPECIAL` | off | Distribute printf-style special tasks off worker 0 |
| `GSIM_MT_WORKER_POOL_FLAG_JOIN` | off | Per-worker completion-flag cycle join |

### Runtime knobs

| Knob | Default | Effect |
|---|---|---|
| `GSIM_THREADS` | 1 | Worker count for the emitted model |
| `GSIM_MT_EXECUTOR=dense` | unset (sparse/layered path) | Select the dense executor when the model was generated with it |
| `GSIM_MT_SPARSE_SERIAL_FAST_MAX_WORKERS` | 1 | At worker counts ≤ N the non-dense step takes the lean serial-fast path and the worker pool is not started (unless the dense executor or profiling is selected). On XiangShan the sparse runtime loses to single-thread serial below 6 workers — every MTask batch falls below the minimum batch size, so MT bookkeeping is pure overhead with zero parallel gain. Set to 4 for 2–4 worker deployments: measured 17.7s vs 19.4s at 2 workers for 50K CoreMark cycles (vs 18.7s single-thread) |
| `GSIM_MT_CPU_AFFINITY` | unset (workers unpinned) | Pin worker w to CPU (base+w−1): `auto` (base 0) or a CPU list. **Always set for dense runs at ≥8 workers**: unpinned spinning workers migrate across CPUs, and a lagging/migrated worker makes the others' lookahead windows churn — a positive-feedback bistability with 2–3× slow runs. Pinning removes it and speeds the fast mode too (T8: 13s→9s; T32: erratic 8.6–18.2s → stable 5.33–5.37s, NEMU-exact) |
| `GSIM_MT_DENSE_DUTY` | off (gen-time and runtime) | Perturbation-free duty-cycle instrumentation: per-thread, per-cycle chrono into 64B-padded per-lane counters (chain span / lookahead tail / in-chain token block / pool spin / coordinator reset+join+step wall), printed as `[mt-duty]` lines at exit. ~8 clock reads per worker per cycle; measured distortion +0.63%. Unlike `GSIM_MT_PROFILE` (per-task chrono on one shared cache line, ~19× observer effect), this is safe for overhead accounting |
| `GSIM_MT_OWNER_CPU_MAP` (runtime, requires codegen `GSIM_MT_OWNER_CPU_MAP_CODEGEN=1`) | unset (sequential affinity) | Explicit worker→CPU map: a comma-separated CPU id per logical worker (count must match `GSIM_THREADS`, no duplicates, all inside the allowed set; any parse/validation/affinity failure aborts). Worker 0 is pinned too. On the registered T32 XiangShan champion (EPYC 9654, 4 CCDs × 8 cores, `taskset -c 0-31`), the annealed map below was measured at **−3.9%** (5/5 pairs) and **−1.6%** (4/5) on an idle machine — direction confirmed, magnitude load-dependent. The map is hardware- and schedule-specific; regenerate it from the token traffic matrix before reusing on another machine or recipe: `0,1,24,2,8,3,25,9,10,16,26,17,11,27,4,12,28,18,13,19,14,20,5,29,15,6,30,21,22,23,31,7` |

### Exact replay (deterministic regeneration)

```bash
GSIM_SCHEDULE_SEED2_WRITE=champion.gsimseed2 build/gsim/gsim ...   # record a generation
GSIM_SCHEDULE_SEED2=champion.gsimseed2       build/gsim/gsim ...   # replay it byte-identically
```

The seed records nine pin points (topoSort, all coarsen merges, partition, replication, pre-emit) plus the when-merge table; replay forces each recorded outcome and verifies a content hash at every point, so the same recipe on any allocator reproduces the same model byte-for-byte. Seeds are tied to the generator build — re-record after any content-affecting change.

`GSIM_SEED2_VERIFY_CANON=0` skips the per-point content-hash verification (the integrity check, not an input to the applied order): T16 champion replay 19m30s → 8m39s with byte-identical output. The input-hash gate, pass-flow tags, and pin-count checks still run; a wrong-seed mistake still surfaces in the schedule-facts line. Verification stays the default.

### Diagnostics

+ `GSIM_DEBUG_CANON_HASH=1` prints per-pass content hashes; `GSIM_DEBUG_CANON_DUMP=<dir>` dumps canonical records for diffing.
+ `GSIM_MT_DENSE_PEG_DUMP=<prefix>` dumps the precedence event graph (dependency edges dist=0, register wrap edges dist=1, per-MTask cost) for recurrence (max-cycle-ratio) analysis.
+ `GSIM_MT_PROFILE=1 GSIM_MT_PROFILE_TASKS=1` at runtime prints per-task wall-time accounting.
+ Generator-side instrumentation knobs (all default-off, all report-only — they never change emitted logic): `GSIM_EMIT_PHASE_TIMING=1` prints per-phase generation walls; `GSIM_MT_DENSE_OLDVALUE_HISTOGRAM=1` reports how many `$old$` snapshots have downstream consumers; `GSIM_MT_ASSERTS=0` (at generation time) omits the runtime assertions from the emitted model. Model-build macro: compiling the model with `-DGSIM_MT_DENSE_LOOKAHEAD_TAIL_STATS_COMPILE=1` adds lookahead tail-scan counters (calls/scanned/found/fullmiss) to the model's profile dump.


## Debug logs & dumps

+ By default `gsim` runs quietly (`LogLevel=0`, dump disabled). Enable lightweight stage logs with `--log-level=1` (prints pass begin/end). Use `--log-level=2` for verbose constant-analysis traces; expect a lot more stderr.
+ Graph dumps: `--dump` turns on both DOT and JSON dumps for every stage; `--dump-json` / `--dump-dot` turn on a single format. Combine with `--dump-stages=a,b,c` to limit which stages emit (e.g., `AfterSplitNodes,ConstantAnalysis`). Set `--dir=tmp-out/gsim-dumps` to choose the output directory.
+ Extra debugging artifacts: `--dump-assign-tree` includes assignTree structure in JSON dumps; `--dump-const-status` writes `<name>_<stage>_ConstStatus.json` with per-node constant-analysis status.
+ Example: `build/gsim/gsim --dir tmp-out/gsim-dumps --dump --dump-stages=AfterSplitNodes,ConstantAnalysis --dump-assign-tree --log-level=1 ready-to-run/TestHarness-rocket.fir`


## Papers and Presentations
### GSIM: Accelerating RTL Simulation for Large-Scale Design
Lu Chen, Dingyi Zhao, Zihao Yu, Ninghui Sun, Yungang Bao

Design Automation Conference (DAC), 2025

[Paper PDF](https://github.com/jaypiper/simulator/blob/master/docs/dac-gsim.pdf) | [Slides](https://github.com/jaypiper/simulator/blob/master/docs/2025-6-26-dac.pdf) | [Chinese Slides](https://github.com/jaypiper/simulator/blob/master/docs/2025-4-9.pdf)
