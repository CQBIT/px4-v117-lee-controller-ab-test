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

# Do not use `ros2 topic list` as the primary readiness condition between
# sequential cases. The ROS 2 discovery graph can briefly retain endpoints
# from the previous Micro XRCE-DDS Agent, which made a stale VehicleOdometry
# endpoint look ready while the newly-started PX4 client was still performing
# XRCE time synchronization/entity creation. Instead wait for this PX4 process
# itself to report creation of its VehicleOdometry DDS writer.
DDS_WRITER_READY=0
for i in $(seq 1 180); do
  if grep -q 'successfully created rt/fmu/out/vehicle_odometry data writer' "$RUN_DIR/px4_gz.log" 2>/dev/null; then
    DDS_WRITER_READY=1
    break
  fi
  if ! kill -0 "$PX4_PID" 2>/dev/null; then
    echo "PX4/Gazebo process exited before creating VehicleOdometry DDS writer" >&2
    break
  fi
  sleep 1
done

timeout 5s ros2 topic info /fmu/out/vehicle_odometry -v \
  > "$RUN_DIR/vehicle_odometry_topic_info.txt" 2>&1 || true

if [[ $DDS_WRITER_READY -ne 1 ]]; then
  echo "Current PX4 instance did not create the VehicleOdometry DDS writer within 180 seconds" >&2
  echo "--- vehicle_odometry topic info (may include stale discovery data) ---" >&2
  cat "$RUN_DIR/vehicle_odometry_topic_info.txt" >&2 || true
  echo "--- XRCE agent tail ---" >&2
  tail -n 200 "$RUN_DIR/xrce.log" >&2 || true
  echo "--- PX4/Gazebo tail ---" >&2
  tail -n 240 "$RUN_DIR/px4_gz.log" >&2 || true
  exit 20
fi

# Confirm end-to-end DDS delivery with the same message type and SensorDataQoS
# used by the controller. This verifies that the current writer is not merely
# registered but actually delivering real PX4 SITL odometry samples.
set +e
timeout 35s python3 - <<'PY' \
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

deadline = time.monotonic() + 30.0
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
  echo "Current PX4 VehicleOdometry writer exists but SensorDataQoS subscriber received no sample (status=$PROBE_STATUS)" >&2
  echo "--- vehicle_odometry topic info ---" >&2
  cat "$RUN_DIR/vehicle_odometry_topic_info.txt" >&2 || true
  echo "--- vehicle_odometry probe stdout ---" >&2
  cat "$RUN_DIR/vehicle_odometry_probe.txt" >&2 || true
  echo "--- vehicle_odometry probe stderr ---" >&2
  cat "$RUN_DIR/vehicle_odometry_probe.err" >&2 || true
  echo "--- XRCE agent tail ---" >&2
  tail -n 200 "$RUN_DIR/xrce.log" >&2 || true
  echo "--- PX4/Gazebo tail ---" >&2
  tail -n 240 "$RUN_DIR/px4_gz.log" >&2 || true
  exit 20
fi
echo "Received VehicleOdometry from current PX4 instance with SensorDataQoS; PX4 ready"

# rclcpp declares `duration` as a double. A bare YAML scalar such as `25` is
# parsed by the ROS 2 CLI as an integer and is rejected for a statically typed
# double parameter. Normalize integer durations to an explicit floating-point
# scalar while retaining the integer value for GNU timeout arithmetic.
if [[ "$DURATION" =~ ^[0-9]+$ ]]; then
  DURATION_PARAM="${DURATION}.0"
else
  DURATION_PARAM="$DURATION"
fi
TIMEOUT_SECONDS=$(python3 - "$DURATION" <<'PY'
import math
import sys
print(max(1, math.ceil(float(sys.argv[1]) + 35.0)))
PY
)

set +e
timeout "${TIMEOUT_SECONDS}s" ros2 run lee_ab_controller lee_ab_controller --ros-args \
  -p mode:="$MODE" \
  -p scenario:="$SCENARIO" \
  -p duration:="$DURATION_PARAM" \
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
