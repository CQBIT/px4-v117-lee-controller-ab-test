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

TOPIC_SEEN=0
for i in $(seq 1 120); do
  if timeout 2s ros2 topic list 2>/dev/null | grep -q '^/fmu/out/vehicle_odometry$'; then
    TOPIC_SEEN=1
    break
  fi
  sleep 1
done

timeout 5s ros2 topic info /fmu/out/vehicle_odometry -v \
  > "$RUN_DIR/vehicle_odometry_topic_info.txt" 2>&1 || true

if [[ $TOPIC_SEEN -ne 1 ]]; then
  echo "PX4 did not advertise /fmu/out/vehicle_odometry within 120 seconds" >&2
  tail -n 200 "$RUN_DIR/px4_gz.log" >&2 || true
  cat "$RUN_DIR/vehicle_odometry_topic_info.txt" >&2 || true
  exit 20
fi

# Use the same message type and SensorDataQoS as the actual controller rather
# than relying on ros2 topic echo CLI QoS/argument behavior. Keep stderr as an
# artifact so a transport or type-support failure is directly diagnosable.
set +e
timeout 20s python3 - <<'PY' \
  > "$RUN_DIR/vehicle_odometry_probe.txt" \
  2> "$RUN_DIR/vehicle_odometry_probe.err"
import sys
import time
import rclpy
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = rclpy.create_node("lee_ab_vehicle_odometry_probe")
received = []

def on_odom(msg):
    received.append(msg)

sub = node.create_subscription(
    VehicleOdometry,
    "/fmu/out/vehicle_odometry",
    on_odom,
    qos_profile_sensor_data,
)

deadline = time.monotonic() + 12.0
while rclpy.ok() and not received and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.25)

if not received:
    print("No VehicleOdometry sample received with SensorDataQoS", file=sys.stderr)
    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(2)

msg = received[0]
print(f"timestamp={msg.timestamp}")
print(f"timestamp_sample={msg.timestamp_sample}")
print(f"pose_frame={msg.pose_frame}")
node.destroy_subscription(sub)
node.destroy_node()
rclpy.shutdown()
PY
PROBE_STATUS=$?
set -e

if [[ $PROBE_STATUS -ne 0 ]] || ! grep -q '^timestamp=' "$RUN_DIR/vehicle_odometry_probe.txt"; then
  echo "PX4 advertised vehicle_odometry but a SensorDataQoS subscriber received no sample (status=$PROBE_STATUS)" >&2
  echo "--- vehicle_odometry topic info ---" >&2
  cat "$RUN_DIR/vehicle_odometry_topic_info.txt" >&2 || true
  echo "--- vehicle_odometry probe stdout ---" >&2
  cat "$RUN_DIR/vehicle_odometry_probe.txt" >&2 || true
  echo "--- vehicle_odometry probe stderr ---" >&2
  cat "$RUN_DIR/vehicle_odometry_probe.err" >&2 || true
  tail -n 200 "$RUN_DIR/px4_gz.log" >&2 || true
  exit 20
fi
echo "Received vehicle_odometry data with SensorDataQoS, PX4 ready"

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

CSV_LINES=$(wc -l < "$RUN_DIR/controller.csv")
if [[ $CSV_LINES -lt 50 ]]; then
  echo "controller.csv has only $CSV_LINES lines; real SITL controller data is incomplete" >&2
  tail -n 160 "$RUN_DIR/controller.log" >&2 || true
  exit 22
fi

if [[ $STATUS -ne 0 ]]; then
  echo "controller exited or timed out with status=$STATUS" >&2
  tail -n 160 "$RUN_DIR/controller.log" >&2 || true
  exit "$STATUS"
fi
