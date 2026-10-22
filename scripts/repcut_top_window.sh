#!/usr/bin/env bash
# R7 window driver: repcut-top knob-on/off kunminghu generations + gates.
# Usage: repcut_top_window.sh {identity|generate|build|gates|campaign} ...
set -euo pipefail

WT=/home/zhangyangjie/test/wt-repcut7
VC=/home/zhangyangjie/test/gsim-task-upstream-vcontract
CELLS=$VC/profile/state-graph-hybrid/cells
CHAMP=$CELLS/split-4096-kb/gsim-compile/model
RTL=/home/zhangyangjie/test/rtl-kunminghu-v3-frozen/SimTop.fir
CSV=$VC/profile/state-graph-hybrid/analysis/bubble-top-wait-slots.csv
READY=/home/zhangyangjie/test/XiangShan/ready-to-run

gen() {  # name outdir topn
  local name=$1 out=$2 topn=$3
  mkdir -p "$out"
  local envcmd="GSIM_THREADS=32"
  if [ "$topn" != "0" ]; then
    envcmd="$envcmd GSIM_MT_REPCUT_TOP=$topn GSIM_MT_REPCUT_CSV=$CSV"
  fi
  local manifest="$out/../generate.json"
  local start=$(date +%s)
  (cd "$WT" && eval "$envcmd" build/gsim/gsim \
      --mt-mode=on --mt-target-tasks=2400 --mt-scheduler=heft \
      --mt-lookahead-window=64 --cpp-max-size-KB=4096 \
      --sep-mod=__DOT__ --sep-aggr=__DOT__ --dir="$out" "$RTL" \
      > "$out/../generate.log" 2>&1)
  local rc=$?
  python3 - "$manifest" "$WT" "$name" "$topn" "$start" "$out" <<'PYEOF'
import json, subprocess, sys, time
manifest, wt, name, topn, start, out = sys.argv[1:7]
print(json.dumps({
  "cell": name, "source": wt,
  "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=wt, text=True).strip(),
  "source_branch": subprocess.check_output(["git", "branch", "--show-current"], cwd=wt, text=True).strip(),
  "repcut_top": int(topn), "repcut_csv": "profile/state-graph-hybrid/analysis/bubble-top-wait-slots.csv",
  "start_unix": int(start), "end_unix": int(time.time()),
}, indent=2), file=open(manifest, "w"))
PYEOF
  echo "generated $name rc=$rc in $(( $(date +%s) - start ))s"
}

case "${1:-}" in
  identity)  # knob-off full generation vs champion bytes
    rm -rf /tmp/repcut7-identity && mkdir -p /tmp/repcut7-identity/model
    gen repcut7-identity /tmp/repcut7-identity/model 0
    n_new=$(ls /tmp/repcut7-identity/model | wc -l)
    n_old=$(ls "$CHAMP" | wc -l)
    echo "files: new=$n_new champ=$n_old"
    if diff -r -q "$CHAMP" /tmp/repcut7-identity/model > /tmp/repcut7-identity/diff.txt 2>&1; then
      echo "IDENTITY: PASS"
    else
      echo "IDENTITY: FAIL"; head -5 /tmp/repcut7-identity/diff.txt
    fi ;;
  generate)  # generate <cellname> <topn>
    rm -rf "$CELLS/$2" && mkdir -p "$CELLS/$2/gsim-compile/model"
    gen "$2" "$CELLS/$2/gsim-compile/model" "${3:-16}" ;;
  build)     # build <cellname>
    cd "$VC" && python3 scripts/state_graph_build.py build "$2" --jobs 14 ;;
  gates)     # NEMU gates on <cellname> emu
    EMU=$CELLS/$2/gsim-compile/emu
    for spec in "coremark 50000 67944" "linux 30000 86469"; do
      set -- $spec
      (cd "$READY" && GSIM_MT_EXECUTOR=dense GSIM_THREADS=32 GSIM_MT_CPU_AFFINITY=auto \
        taskset -c 48-79 "$EMU" -i $1.bin --diff riscv64-nemu-interpreter-so \
        -b 0 -e 0 -C $2 -I $3 > /tmp/repcut7-$1-gate.log 2>&1) || true
      if grep -q "HIT GOOD TRAP" /tmp/repcut7-$1-gate.log; then echo "$1: GOOD"; else echo "$1: FAIL"; tail -3 /tmp/repcut7-$1-gate.log; fi
    done
    (cd "$READY" && GSIM_MT_EXECUTOR=dense GSIM_THREADS=32 GSIM_MT_CPU_AFFINITY=auto \
      taskset -c 48-79 "$EMU" -i coremark-2-iteration.bin --diff riscv64-nemu-interpreter-so \
      -C 700000 > /tmp/repcut7-fulltrap-gate.log 2>&1) || true
    grep -E "HIT GOOD TRAP" /tmp/repcut7-fulltrap-gate.log | head -2 ;;
  campaign)  # campaign <name> <label>:<emu> ...
    shift
    cd "$VC" && python3 scripts/state_graph_campaign.py --campaign "$@" \
      --side "s4096=$CELLS/split-4096-kb/gsim-compile/emu" --preset orig \
      --workloads coremark,linux,fulltrap --rounds 5 --wait-lock --mask 48-79 --guard-cpu-scope ;;
  *) echo "usage: $0 {identity|generate <cell> <topn>|build <cell>|gates <cell>|campaign <name> <label>:<emu>}"; exit 1 ;;
esac
