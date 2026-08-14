#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
rm -rf results
mkdir -p results

SCENARIOS=(hover circle figure8 aggressive)
DURATION=25
for scenario in "${SCENARIOS[@]}"; do
  for mode in rate torque; do
    echo "===== ${scenario} / ${mode} ====="
    set +e
    bash ./scripts/run_case_native.sh "$mode" "$scenario" "$DURATION"
    CASE_STATUS=$?
    set -e

    if [[ $CASE_STATUS -ne 0 ]]; then
      RUN_DIR="results/${scenario}_${mode}"
      # The controller currently issues its disarm at flight_t > duration and,
      # on the following timer tick, its generic 18 s armed/Offboard watchdog
      # can observe the intentional disarm first and exit 30 before reaching
      # the normal "Experiment complete" shutdown branch.  Do not hide a real
      # startup/offboard failure: accept status 30 only when the real SITL CSV
      # proves this case actually reached the requested experiment duration.
      if [[ $CASE_STATUS -eq 30 ]] && python3 - "$RUN_DIR/controller.csv" "$DURATION" <<'PY'
import csv
import math
import sys

path = sys.argv[1]
required = float(sys.argv[2])
try:
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    flight_t = [float(r["flight_t"]) for r in rows if r.get("flight_t") not in (None, "")]
except Exception as exc:
    print(f"could not validate completed SITL CSV: {exc}", file=sys.stderr)
    raise SystemExit(1)

if not flight_t:
    print("no flight_t samples in controller.csv", file=sys.stderr)
    raise SystemExit(1)
max_flight_t = max(x for x in flight_t if math.isfinite(x))
print(f"validated real SITL samples through flight_t={max_flight_t:.3f}s")
raise SystemExit(0 if max_flight_t >= required else 1)
PY
      then
        if ! grep -q 'PX4 confirmed ARMED + OFFBOARD; starting experiment clock' "$RUN_DIR/controller.log"; then
          echo "status 30 occurred without confirmed armed Offboard experiment start" >&2
          exit "$CASE_STATUS"
        fi
        if ! find "$RUN_DIR" -maxdepth 1 -type f -name '*.ulg' -size +0c | grep -q .; then
          echo "status 30 occurred after duration but no non-empty PX4 ULog was captured" >&2
          exit "$CASE_STATUS"
        fi
        echo "Case reached full real-SITL duration and produced ULog; treating post-disarm watchdog exit 30 as completed."
      else
        echo "Case ${scenario}/${mode} failed with status=$CASE_STATUS before a validated full SITL experiment completed" >&2
        exit "$CASE_STATUS"
      fi
    fi
  done
done
python3 scripts/analyze_results.py --root results
