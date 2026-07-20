#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/setup/s2_bundle_env.sh" "${S2_NETWORK_INTERFACE:-}"

if [[ "$(. /etc/os-release && echo "${VERSION_ID}")" != "22.04" ]]; then
  echo "[s2-install] Ubuntu 22.04 is required for this ROS 2 Humble bundle" >&2
  exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[s2-install] ROS 2 Humble is not installed under /opt/ros/humble" >&2
  echo "Install ROS 2 Humble Desktop from the official ROS apt repository, then rerun this script." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git \
  python3-colcon-common-extensions python3-numpy python3-opencv python3-rosdep python3-yaml \
  ros-humble-rmw-cyclonedds-cpp ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-sensor-msgs-py

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

mkdir -p "${S2_NAV2_WS}/src"
if [[ ! -d "${S2_NAV2_WS}/src/navigation2" ]]; then
  tar -xzf "${ROOT}/third_party/navigation2_humble_3c3db59.tar.gz" \
    -C "${S2_NAV2_WS}/src"
fi

rosdep install \
  --from-paths "${S2_NAV2_WS}/src/navigation2" \
  --ignore-src --rosdistro humble -r -y

mkdir -p "${S2_NAV2_WS}/local_deps"
"${ROOT}/tools/build_s2_nav2_source.sh"

echo "[s2-install] build complete: ${S2_NAV2_WS}/install/setup.bash"
"${ROOT}/bin/s2_doctor" || true

