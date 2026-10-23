#!/usr/bin/env bash
# rush-protocol variant matrix for sub-task cone replication
set -euo pipefail
VC=/home/zhangyangjie/test/gsim-task-upstream-vcontract
RTL=/home/zhangyangjie/test/rtl-kunminghu-v3-frozen/SimTop.fir
CSV=$VC/profile/state-graph-hybrid/analysis/bubble-top-wait-slots-post138.csv
WT=/home/zhangyangjie/test/wt-repcut7

gen_variant() {  # name topn extra_env...
  local NAME=$1 TOPN=$2; shift 2
  local D=$VC/profile/state-graph-hybrid/cells/$NAME
  rm -rf "$D" && mkdir -p "$D/gsim-compile/model"
  (cd "$WT" && env "$@" GSIM_MT_REPCUT_TOP=$TOPN GSIM_MT_REPCUT_CSV=$CSV \
    GSIM_THREADS=32 build/gsim/gsim --mt-mode=on --mt-target-tasks=2400 \
    --mt-scheduler=heft --mt-lookahead-window=64 --cpp-max-size-KB=4096 \
    --sep-mod=__DOT__ --sep-aggr=__DOT__ --dir="$D/gsim-compile/model" \
    "$RTL" > "$D/gsim-compile/generate.log" 2>&1)
  python3 -c "import json; print(json.dumps({'returncode': 0}, indent=2))" > "$D/generate.json"
  echo "== $NAME (TOP=$TOPN $*) =="
  grep -E "repcut-top csv|workers=32 groups" "$D/gsim-compile/generate.log"
}

build_variant() {
  local NAME=$1
  (cd "$VC" && python3 scripts/state_graph_build.py build "$NAME" --jobs 14 > /dev/null 2>&1)
  ls "$VC/profile/state-graph-hybrid/cells/$NAME/gsim-compile/emu" > /dev/null 2>&1 \
    && echo "BUILD $NAME OK" || echo "BUILD $NAME FAIL"
}

preview() {  # name rounds — coremark-only quick screen
  local NAME=$1 ROUNDS=${2:-2}
  local E=$VC/profile/state-graph-hybrid/cells/$NAME/gsim-compile/emu
  cd /home/zhangyangjie/test/XiangShan/ready-to-run
  for r in $(seq 1 "$ROUNDS"); do
    /usr/bin/time -f "%e" -o "/tmp/preview-$NAME-r$r.time" \
      env GSIM_MT_EXECUTOR=dense GSIM_THREADS=32 GSIM_MT_CPU_AFFINITY=auto \
      taskset -c 48-79 "$E" -i coremark-2-iteration.bin \
      --diff riscv64-nemu-interpreter-so -b 0 -e 0 -C 50000 -I 67944 \
      > /dev/null 2> "/tmp/preview-$NAME-r$r.log"
    grep -q "instrCnt = 67,944" "/tmp/preview-$NAME-r$r.log" \
      && echo "$NAME r$r: $(cat /tmp/preview-$NAME-r$r.time)s CORRECT" \
      || echo "$NAME r$r: $(cat /tmp/preview-$NAME-r$r.time)s DIVERGED"
  done
}

case "${1:-}" in
  gen)    shift; gen_variant "$@" ;;
  build)  build_variant "$2" ;;
  preview) shift; preview "$@" ;;
  *) echo "usage: $0 {gen <name> <topn> <ENV=V...>|build <name>|preview <name> [rounds]}"; exit 1 ;;
esac
