#!/usr/bin/env bash
set -euo pipefail

# Native cloud setup for PX4 v1.17 + Gazebo x500 + ROS 2.
# Supports Ubuntu 22.04 (ROS 2 Humble) and Ubuntu 24.04 (ROS 2 Jazzy).

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
. /etc/os-release
case "${VERSION_ID}" in
  22.04) ROS_DISTRO=humble ;;
  24.04) ROS_DISTRO=jazzy ;;
  *) echo "Unsupported Ubuntu ${VERSION_ID}; expected 22.04 or 24.04" >&2; exit 2 ;;
esac

echo "[codex-setup] Ubuntu=${VERSION_ID} ROS_DISTRO=${ROS_DISTRO}"

sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  ca-certificates curl gnupg lsb-release software-properties-common \
  git git-lfs build-essential cmake ninja-build pkg-config ccache \
  python3 python3-pip python3-venv python3-dev \
  libeigen3-dev libyaml-cpp-dev libasio-dev libtinyxml2-dev \
  libssl-dev libxml2-dev unzip zip rsync wget

sudo add-apt-repository -y universe
sudo mkdir -p /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null
ARCH="$(dpkg --print-architecture)"
CODENAME="${UBUNTU_CODENAME}"
echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  "ros-${ROS_DISTRO}-ros-base" \
  "ros-${ROS_DISTRO}-rclcpp" \
  "ros-${ROS_DISTRO}-rmw-fastrtps-cpp" \
  python3-colcon-common-extensions python3-rosdep python3-vcstool

PX4_DIR="${HOME}/PX4-Autopilot"
if [[ ! -d "${PX4_DIR}/.git" ]]; then
  git clone --recursive --branch v1.17.0 --depth 1 --shallow-submodules \
    https://github.com/PX4/PX4-Autopilot.git "${PX4_DIR}"
else
  git -C "${PX4_DIR}" fetch --tags --depth 1 origin v1.17.0
  git -C "${PX4_DIR}" checkout -f v1.17.0
  git -C "${PX4_DIR}" submodule update --init --recursive --depth 1
fi

bash "${PX4_DIR}/Tools/setup/ubuntu.sh" --no-nuttx

PX4_MSGS_DIR="${ROOT}/ros2_ws/src/px4_msgs"
if [[ ! -d "${PX4_MSGS_DIR}/.git" ]]; then
  rm -rf "${PX4_MSGS_DIR}"
  git clone --depth 1 --branch release/1.17 https://github.com/PX4/px4_msgs.git "${PX4_MSGS_DIR}"
fi

XRCE_DIR="${HOME}/Micro-XRCE-DDS-Agent"
if [[ ! -d "${XRCE_DIR}/.git" ]]; then
  git clone --depth 1 --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "${XRCE_DIR}"
fi
cmake -S "${XRCE_DIR}" -B "${XRCE_DIR}/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "${XRCE_DIR}/build" -j"$(nproc)"
sudo cmake --install "${XRCE_DIR}/build"
sudo ldconfig

cd "${PX4_DIR}"
make -j"$(nproc)" px4_sitl_default

# ROS setup scripts are not guaranteed to be nounset-safe. Temporarily disable
# `set -u` while sourcing them, then restore strict mode for our own script.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
cd "${ROOT}/ros2_ws"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

python3 -m pip install --user --break-system-packages numpy pandas matplotlib pyulog || \
python3 -m pip install --user numpy pandas matplotlib pyulog

# Generate a reusable environment helper. Setup-time paths/distros are expanded
# here; function-local variables are escaped so they are evaluated when sourced.
cat > "${ROOT}/codex/runtime_env.sh" <<ENVEOF
export ROS_DISTRO=${ROS_DISTRO}
export PX4_DIR=${PX4_DIR}
export LEE_AB_ROOT=${ROOT}
export XRCE_BIN=/usr/local/bin/MicroXRCEAgent
export HEADLESS=1

source_no_unset() {
  local setup_file="\$1"
  local restore_u=0
  case "\$-" in
    *u*) restore_u=1; set +u ;;
  esac
  # shellcheck disable=SC1090
  source "\$setup_file"
  if [[ "\$restore_u" -eq 1 ]]; then
    set -u
  fi
}

source_ros() {
  source_no_unset "/opt/ros/\${ROS_DISTRO}/setup.bash"
}

source_lee_ws() {
  source_no_unset "\${LEE_AB_ROOT}/ros2_ws/install/setup.bash"
}
ENVEOF

echo "[codex-setup] complete"
echo "[codex-setup] source ${ROOT}/codex/runtime_env.sh"
