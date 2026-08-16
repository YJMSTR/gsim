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

### Quick start (XiangShan SimTop netlist, 32 threads)

Generate the model with the champion recipe:

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
  --dir <OUT_DIR> SimTop.fir                 # <OUT_DIR> must exist; ~33 min for XiangShan
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

### Exact replay (deterministic regeneration)

```bash
GSIM_SCHEDULE_SEED2_WRITE=champion.gsimseed2 build/gsim/gsim ...   # record a generation
GSIM_SCHEDULE_SEED2=champion.gsimseed2       build/gsim/gsim ...   # replay it byte-identically
```

The seed records nine pin points (topoSort, all coarsen merges, partition, replication, pre-emit) plus the when-merge table; replay forces each recorded outcome and verifies a content hash at every point, so the same recipe on any allocator reproduces the same model byte-for-byte. Seeds are tied to the generator build — re-record after any content-affecting change.

### Diagnostics

+ `GSIM_DEBUG_CANON_HASH=1` prints per-pass content hashes; `GSIM_DEBUG_CANON_DUMP=<dir>` dumps canonical records for diffing.
+ `GSIM_MT_DENSE_PEG_DUMP=<prefix>` dumps the precedence event graph (dependency edges dist=0, register wrap edges dist=1, per-MTask cost) for recurrence (max-cycle-ratio) analysis.
+ `GSIM_MT_PROFILE=1 GSIM_MT_PROFILE_TASKS=1` at runtime prints per-task wall-time accounting.


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
