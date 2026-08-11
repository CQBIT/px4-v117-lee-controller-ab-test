#!/usr/bin/env bash
set -euo pipefail
MODE=${1:?rate|torque}
SCENARIO=${2:?hover|circle|figure8|aggressive}
DURATION=${3:-25}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
source "$ROOT/codex/runtime_env.sh"
source_ros
source_lee_ws
RUN_DIR="$ROOT/results/${SCENARIO}_${MODE}"
rm -rf "$RUN_DIR"
mkdir -p "$RUN_DIR"

cleanup() {
  set +e
  [[ -n "${CTRL_PID:-}" ]] && kill "$CTRL_PID" 2>/dev/null || true
  [[ -n "${PX4_PID:-}" ]] && kill "$PX4_PID" 2>/dev/null || true
  [[ -n "${XRCE_PID:-}" ]] && kill "$XRCE_PID" 2>/dev/null || true
  pkill -f "MicroXRCEAgent udp4 -p 8888" >/dev/null 2>&1 || true
  pkill -f "build/px4_sitl_default/bin/px4" >/dev/null 2>&1 || true
  pkill -f "gz sim" >/dev/null 2>&1 || true
  pkill -f "ruby.*gz" >/dev/null 2>&1 || true
}
trap cleanup EXIT
cleanup
sleep 1

"${XRCE_BIN}" udp4 -p 8888 > "$RUN_DIR/xrce.log" 2>&1 &
XRCE_PID=$!

rm -rf "$PX4_DIR/build/px4_sitl_default/rootfs/log" || true

(
  cd "$PX4_DIR"
  export HEADLESS=1
  export PX4_GZ_WORLD=default
  make px4_sitl gz_x500
) > "$RUN_DIR/px4_gz.log" 2>&1 &
PX4_PID=$!

READY=0
for i in $(seq 1 120); do
  # Check if the topic exists
  if timeout 2s ros2 topic list 2>/dev/null | grep -q '/fmu/out/vehicle_odometry'; then
    # Topic exists, try to receive one message with timeout
    if timeout 3s ros2 topic echo /fmu/out/vehicle_odometry --once 2>/dev/null | grep -q 'frame_id'; then
      echo "Received vehicle_odometry data, PX4 ready"
      READY=1
      break
    fi
  fi
  sleep 1
done
if [[ $READY -ne 1 ]]; then
  echo "PX4 did not produce vehicle_odometry data within 120 seconds" >&2
  tail -n 200 "$RUN_DIR/px4_gz.log" >&2 || true
  echo "---" >&2
  timeout 2s ros2 topic list 2>/dev/null | grep -i odometry || echo "No odometry topics found" >&2
  exit 20
fi

set +e
timeout "$((DURATION+35))"s ros2 run lee_ab_controller lee_ab_controller --ros-args \
  -p mode:="$MODE" \
  -p scenario:="$SCENARIO" \
  -p duration:="$DURATION" \
  -p output_csv:="$RUN_DIR/controller.csv" \
  > "$RUN_DIR/controller.log" 2>&1
STATUS=$?
set -e
sleep 2

find "$PX4_DIR/build/px4_sitl_default/rootfs/log" -type f -name '*.ulg' -print -exec cp -f {} "$RUN_DIR/" \; 2>/dev/null || true

if [[ ! -s "$RUN_DIR/controller.csv" ]]; then
  echo "controller.csv missing/empty" >&2
  tail -n 160 "$RUN_DIR/controller.log" >&2 || true
  tail -n 160 "$RUN_DIR/px4_gz.log" >&2 || true
  exit 21
fi
if [[ $STATUS -ne 0 && $STATUS -ne 124 ]]; then
  echo "controller exited status=$STATUS" >&2
  exit "$STATUS"
fi
