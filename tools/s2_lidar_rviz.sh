#!/usr/bin/env bash
set -euo pipefail

ROOT="${S2_DREAM_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
CONFIG="${S2_RVIZ_CONFIG:-${ROOT}/configs/rviz/s2_lidar_mapping.rviz}"
CYCLONEDDS_CONFIG="${S2_CYCLONEDDS_CONFIG:-${ROOT}/configs/robots/s2_cyclonedds.xml}"
BRIDGE_LOG="${S2_RVIZ_BRIDGE_LOG:-/tmp/s2_lidar_rviz_bridge.log}"
MODEL_LOG="${S2_RVIZ_MODEL_LOG:-/tmp/s2_robot_model.log}"
URDF_FILE="${S2_URDF_FILE:-${ROOT}/robot_description/S2_3DURDF_gazebo_clean.urdf}"
NAV2_WS="${S2_NAV2_WS:-${ROOT}/nav2_ws}"

set +u
if [[ -f "${NAV2_WS}/install/setup.bash" ]]; then
  source "${ROOT}/tools/s2_nav2_source_env.sh"
else
  source /opt/ros/humble/setup.bash
fi
set -u

export ROS_DOMAIN_ID="${S2_ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DISABLE_DAEMON=1
export CYCLONEDDS_URI="file://${CYCLONEDDS_CONFIG}"

/usr/bin/python3 "${ROOT}/tools/s2_lidar_rviz_bridge.py" \
  --odom-topic "${S2_ODOM_TOPIC:-/controller/odom}" \
  --odom-yaw-mode "${S2_ODOM_YAW_MODE:-orientation_w}" \
  --parent-frame "${S2_MAP_FRAME:-odom}" \
  --child-frame "${S2_BASE_FRAME:-base_link}" \
  >"${BRIDGE_LOG}" 2>&1 &
BRIDGE_PID=$!

ros2 launch "${ROOT}/tools/s2_robot_model.launch.py" \
  urdf_file:="${URDF_FILE}" \
  >"${MODEL_LOG}" 2>&1 &
MODEL_PID=$!

cleanup() {
  kill "${BRIDGE_PID}" >/dev/null 2>&1 || true
  kill "${MODEL_PID}" >/dev/null 2>&1 || true
  wait "${BRIDGE_PID}" >/dev/null 2>&1 || true
  wait "${MODEL_PID}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

sleep 1
if ! kill -0 "${BRIDGE_PID}" >/dev/null 2>&1; then
  echo "[s2-rviz] bridge failed to start:" >&2
  cat "${BRIDGE_LOG}" >&2 || true
  exit 1
fi
if ! kill -0 "${MODEL_PID}" >/dev/null 2>&1; then
  echo "[s2-rviz] robot model publisher failed to start:" >&2
  cat "${MODEL_LOG}" >&2 || true
  exit 1
fi

echo "[s2-rviz] bridge_pid=${BRIDGE_PID} model_pid=${MODEL_PID}"
echo "[s2-rviz] urdf=${URDF_FILE} config=${CONFIG}"
rviz2 -d "${CONFIG}"
