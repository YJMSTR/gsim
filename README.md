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

Then build and run the emitted model with your MT harness (a difftest-based flow is one option), with `GSIM_THREADS=32 GSIM_MT_EXECUTOR=dense` at runtime.

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
