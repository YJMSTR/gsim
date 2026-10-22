#!/usr/bin/env bash
set -euo pipefail
VC=/home/zhangyangjie/test/gsim-task-upstream-vcontract
CELL=$VC/profile/state-graph-hybrid/cells/repcut7-prof
RTL=/home/zhangyangjie/test/rtl-kunminghu-v3-frozen/SimTop.fir
mkdir -p "$CELL/gsim-compile/model"
cd /tmp/repcut7-prof
GSIM_THREADS=32 build/gsim/gsim --mt-mode=on --mt-target-tasks=2400 --mt-scheduler=heft \
  --mt-lookahead-window=64 --cpp-max-size-KB=4096 --sep-mod=__DOT__ --sep-aggr=__DOT__ \
  --dir="$CELL/gsim-compile/model" "$RTL" > "$CELL/generate.log" 2>&1
grep -E "workers=32 groups" "$CELL/generate.log"
cd "$VC" && python3 scripts/state_graph_build.py build repcut7-prof --jobs 14
