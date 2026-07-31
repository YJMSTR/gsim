#!/bin/bash
# gsim perf gate: same-session C50000 A/B, candidate emu vs champion emu.
# Usage: gsim-perf-gate.sh <candidate_emu> [champion_emu] [pairs]
#   champion_emu default: champions/xiangshan-t32-v371-lookahead/emu (workspace-relative)
#   pairs default: 5 (Q5-A protocol)
# Exit 0 = PASS (candidate mean <= champion mean * 1.03, all runs NEMU-exact); 1 = FAIL; 2 = usage error.
set -u
CAND="${1:?candidate emu path required}"
HERE="$(cd "$(dirname "$0")" && pwd)"
CHAMP="${2:-$HERE/../champions/xiangshan-t32-v371-lookahead/emu}"
PAIRS="${3:-5}"
TOL_PCT=3.0
RTR=/home/zhangyangjie/test/XiangShan/ready-to-run
[ -x "$CAND" ] || { echo "FAIL: candidate emu not executable: $CAND"; exit 2; }
[ -x "$CHAMP" ] || { echo "FAIL: champion emu not executable: $CHAMP"; exit 2; }

cand_times=(); champ_times=()
for r in $(seq 1 "$PAIRS"); do
  (cd "$RTR" && taskset -c 0-31 env GSIM_THREADS=32 GSIM_MT_EXECUTOR=dense \
     "$CHAMP" -i coremark-2-iteration.bin --diff riscv64-nemu-interpreter-so -C 50000 -I 46540 \
     > "/tmp/gsim-perfgate-champ-$r.log" 2>&1) || { echo "FAIL: champion run $r rc=$?"; exit 1; }
  (cd "$RTR" && taskset -c 0-31 env GSIM_THREADS=32 GSIM_MT_EXECUTOR=dense \
     "$CAND" -i coremark-2-iteration.bin --diff riscv64-nemu-interpreter-so -C 50000 -I 46540 \
     > "/tmp/gsim-perfgate-cand-$r.log" 2>&1) || { echo "FAIL: candidate run $r rc=$?"; exit 1; }
  ct=$(grep -h 'Host time spent' "/tmp/gsim-perfgate-champ-$r.log" | awk '{print $4}' | tr -d ms)
  nt=$(grep -h 'Host time spent' "/tmp/gsim-perfgate-cand-$r.log" | awk '{print $4}' | tr -d ms)
  champ_times+=("$ct"); cand_times+=("$nt")
done
# NEMU endpoint verification
for f in /tmp/gsim-perfgate-champ-*.log /tmp/gsim-perfgate-cand-*.log; do
  grep -q 'instrCnt = 46540, cycleCnt = 50000' "$f" || { echo "FAIL: NEMU endpoint mismatch in $f"; exit 1; }
done
python3 - "$TOL_PCT" "${champ_times[@]}" -- "${cand_times[@]}" <<'EOF'
import sys
tol = float(sys.argv[1])
args = sys.argv[2:]
sep = args.index('--')
champ = [float(x) for x in args[:sep]]
cand = [float(x) for x in args[sep+1:]]
cm = sum(champ) / len(champ)
nm = sum(cand) / len(cand)
delta = (nm - cm) / cm * 100.0
print(f'champion mean {cm:.1f}ms {champ}')
print(f'candidate mean {nm:.1f}ms {cand}')
print(f'delta {delta:+.2f}% (tolerance +{tol}%)')
if nm <= cm * (1 + tol / 100.0):
    print('PASS')
    sys.exit(0)
print('FAIL: candidate exceeds perf tolerance; regenerate (new epoch) or fall back to champion artifact')
sys.exit(1)
EOF
