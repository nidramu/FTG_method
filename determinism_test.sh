#!/bin/bash
# Determinism diagnosis: same worlds, 5 repeats each, measure variance
set -e
cd /media/mery/fd293421-6374-44a7-b166-acc94be5a5571/FTG_method

# 5 stratified worlds: mix of easy/mid/hard
WORLDS=(3 50 103 155 205)
RUNS=5
TS=$(date +%Y%m%d_%H%M%S)
RAW_OUT="determinism_raw_${TS}.txt"
SUMMARY="determinism_summary_${TS}.txt"
rm -f "$RAW_OUT" "$SUMMARY"

cleanup() {
  pkill -9 gzserver gzclient gazebo 2>/dev/null || true
  pkill -9 roscore rosmaster rosout 2>/dev/null || true
  pkill -f 'python3 run.py' 2>/dev/null || true
  pkill -f 'docker_run.sh' 2>/dev/null || true
  ids=$(docker ps --filter ancestor=barn-ftg:latest -q)
  [ -n "$ids" ] && docker rm -f $ids >/dev/null 2>&1 || true
  rm -rf /tmp/.gazebo /tmp/gazebo-* 2>/dev/null || true
  sleep 2
}

echo "=========================================="
echo "DETERMINISM DIAGNOSIS  ($TS)"
echo "=========================================="
echo "Worlds: ${WORLDS[*]}"
echo "Runs per world: $RUNS"
echo "Total runs: $((${#WORLDS[@]} * RUNS))"
echo "Estimated time: ~15-20 minutes"
echo ""

# Header
printf "%-6s %-4s %-3s %-3s %-3s %-8s %-8s\n" "world" "run" "S" "C" "T" "time" "metric" | tee -a "$RAW_OUT"
printf "%-6s %-4s %-3s %-3s %-3s %-8s %-8s\n" "-----" "---" "-" "-" "-" "----" "------" | tee -a "$RAW_OUT"

for w in "${WORLDS[@]}"; do
  for r in $(seq 1 $RUNS); do
    cleanup
    tmp_out="det_tmp_w${w}_r${r}.txt"
    rm -f "$tmp_out"
    docker run --rm \
        --network host \
        -e ROS_MASTER_URI="http://127.0.0.1:11311" \
        -e ROS_IP="127.0.0.1" \
        -e ROS_HOSTNAME="127.0.0.1" \
        -e GAZEBO_RANDOM_SEED=42 \
        -v "$(pwd):/jackal_ws/src/the-barn-challenge" \
        barn-ftg:latest \
        python3 run.py --world_idx "$w" --out "$tmp_out" > /dev/null 2>&1 || true
    if [ -s "$tmp_out" ]; then
      # Line format: world_idx succ coll time_out duration metric
      line=$(tail -n 1 "$tmp_out")
      S=$(echo "$line" | awk '{print $2}')
      C=$(echo "$line" | awk '{print $3}')
      T=$(echo "$line" | awk '{print $4}')
      DUR=$(echo "$line" | awk '{printf "%.2f", $5}')
      MET=$(echo "$line" | awk '{printf "%.4f", $6}')
      printf "%-6d %-4d %-3s %-3s %-3s %-8s %-8s\n" "$w" "$r" "$S" "$C" "$T" "$DUR" "$MET" | tee -a "$RAW_OUT"
    else
      printf "%-6d %-4d %-3s %-3s %-3s %-8s %-8s\n" "$w" "$r" "?" "?" "?" "???" "????" | tee -a "$RAW_OUT"
    fi
    rm -f "$tmp_out"
  done
done

echo "" | tee -a "$RAW_OUT"
echo "=== ANALYSIS ===" | tee -a "$SUMMARY"

python3 << 'PYEOF' | tee -a "$SUMMARY"
import re, statistics, os, glob

# Find the raw output file
raw_files = sorted(glob.glob("determinism_raw_*.txt"))
if not raw_files:
    print("ERROR: No raw data file found")
    exit(1)
raw_file = raw_files[-1]

with open(raw_file) as f:
    lines = f.readlines()

by_world = {}
for line in lines:
    parts = line.split()
    if len(parts) < 7: continue
    try:
        w = int(parts[0])
        s = int(parts[2])
        c = int(parts[3])
        t = int(parts[4])
        dur = float(parts[5])
        met = float(parts[6])
    except (ValueError, IndexError):
        continue
    by_world.setdefault(w, []).append((s, c, t, dur, met))

print()
print(f"{'World':<6} {'Outcomes':<20} {'Time (mean+-std)':<22} {'Metric (mean+-std)':<22} {'Verdict'}")
print('-' * 95)

overall_deterministic = True
for w, runs in sorted(by_world.items()):
    if len(runs) < 2:
        print(f"{w:<6} INSUFFICIENT DATA")
        continue

    successes = sum(r[0] for r in runs)
    collisions = sum(r[1] for r in runs)
    timeouts = sum(r[2] for r in runs)
    durs = [r[3] for r in runs]
    mets = [r[4] for r in runs]

    d_mean = statistics.mean(durs)
    d_std  = statistics.stdev(durs) if len(durs) > 1 else 0
    m_mean = statistics.mean(mets)
    m_std  = statistics.stdev(mets) if len(mets) > 1 else 0

    # Verdict rules
    total_outcomes = successes + collisions + timeouts
    outcomes_consistent = (successes == len(runs)) or (collisions == len(runs)) or (timeouts == len(runs))
    time_cv = d_std / d_mean if d_mean > 0.1 else float('inf')

    if total_outcomes == 0 or d_mean < 0.01:
        verdict = "NO DATA"
        overall_deterministic = False
    elif outcomes_consistent and d_std < 0.3 and m_std < 0.02:
        verdict = "DETERMINISTIC"
    elif outcomes_consistent and d_std < 1.0:
        verdict = "MOSTLY DET."
    elif outcomes_consistent:
        verdict = "TIMING VAR."
        overall_deterministic = False
    else:
        verdict = "NON-DET."
        overall_deterministic = False

    outcome_str = f"S={successes} C={collisions} T={timeouts}"
    print(f"{w:<6} {outcome_str:<20} {d_mean:5.2f} +- {d_std:5.2f} s     {m_mean:.4f} +- {m_std:.4f}   {verdict}")

print()
print('=' * 95)
if overall_deterministic:
    print("OVERALL VERDICT: DETERMINISTIC -- Safe to proceed with Faz 1")
    print("  Action: Use 1 repeat per world in Faz 1 tests; variance is minimal.")
else:
    print("OVERALL VERDICT: NON-DETERMINISTIC -- Gazebo varies run-to-run")
    print("  Action before Faz 1:")
    print("  1. Check GAZEBO_RANDOM_SEED env var (is it set? same each run?)")
    print("  2. Check world .world files for <max_step_size>")
    print("  3. Consider increasing repeats to 5 in Faz 1 test protocol")
print('=' * 95)
PYEOF

echo ""
echo "Files:"
echo "  Raw data: $RAW_OUT"
echo "  Summary:  $SUMMARY"
