#!/usr/bin/env bash
# PMU collection for the db0b1ba champion (R7, per Main directive).
# Usage: pmu_collect.sh {stat-coremark|stat-fulltrap|record} [--run]
#   --run  actually execute (default: print planned commands only)
set -euo pipefail
VC=/home/zhangyangjie/test/gsim-task-upstream-vcontract
EMU=$VC/profile/state-graph-hybrid/cells/db0b1ba-clean/gsim-compile/emu
READY=/home/zhangyangjie/test/XiangShan/ready-to-run
OUT=$VC/profile/state-graph-hybrid/campaigns/pmu-db0b1ba
MODE=${1:-}; DO_RUN=${2:-}

EVENTS="instructions,branches,branch-misses,L1-icache-load-misses,L1-dcache-load-misses,dTLB-load-misses,iTLB-load-misses"
# fallback set if the I-cache event is unavailable on this AMD (znver4 uses
# different PMU names); probe first.
probe() {
  perf stat -e "$EVENTS" -- true > /dev/null 2>&1 && echo "$EVENTS" || \
    echo "instructions,branches,branch-misses,dcache-misses,icache-misses,dTLB-load-misses,iTLB-load-misses"
}
EV=$(probe)

run_stat() {  # workload image cycles tag
  local IMG=$1 CYC=$2 TAG=$3
  mkdir -p "$OUT"
  (cd "$READY" && perf stat -x, -e "$EV" \
      env GSIM_MT_EXECUTOR=dense GSIM_THREADS=32 GSIM_MT_CPU_AFFINITY=auto \
      taskset -c 48-79 "$EMU" -i "$IMG" \
      --diff riscv64-nemu-interpreter-so -C "$CYC" \
      > /dev/null 2> "$OUT/stat-$TAG.txt")
  echo "== $TAG =="; cat "$OUT/stat-$TAG.txt"
}

run_record() {
  mkdir -p "$OUT"
  (cd "$READY" && perf record -g --call-graph dwarf,16384 -F 199 -C 48-79 -- \
      env GSIM_MT_EXECUTOR=dense GSIM_THREADS=32 GSIM_MT_CPU_AFFINITY=auto \
      taskset -c 48-79 "$EMU" -i coremark-2-iteration.bin \
      --diff riscv64-nemu-interpreter-so -C 50000 \
      > /dev/null 2> "$OUT/record.err")
  perf report -i perf.data --stdio --no-children \
      --percent-limit 0.5 > "$OUT/report-top.txt" 2>/dev/null || true
  cp perf.data "$OUT/perf-coremark.data" 2>/dev/null || true
  head -60 "$OUT/report-top.txt"
}

case "$MODE" in
  probe) echo "events: $(probe)" ;;
  stat-coremark)
    CMD="run_stat coremark-2-iteration.bin 50000 coremark50k"
    [ "$DO_RUN" = --run ] && $CMD || echo "PLAN: $CMD (events: $EV)" ;;
  stat-fulltrap)
    CMD="run_stat coremark-2-iteration.bin 700000 fulltrap"
    [ "$DO_RUN" = --run ] && $CMD || echo "PLAN: $CMD (events: $EV)" ;;
  record)
    [ "$DO_RUN" = --run ] && run_record || echo "PLAN: perf record -g dwarf -F199 -C48-79 coremark 50k -> $OUT" ;;
  *) echo "usage: $0 {probe|stat-coremark|stat-fulltrap|record} [--run]"; exit 1 ;;
esac
