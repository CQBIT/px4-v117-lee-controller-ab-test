#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
rm -rf results
mkdir -p results

SCENARIOS=(hover circle figure8 aggressive)
for scenario in "${SCENARIOS[@]}"; do
  for mode in rate torque; do
    echo "===== ${scenario} / ${mode} ====="
    ./scripts/run_case_native.sh "$mode" "$scenario" 25
  done
done
python3 scripts/analyze_results.py --root results
