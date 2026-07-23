#!/usr/bin/env bash
set -euo pipefail

ROOT="${S2_DREAM_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
MOTION="${S2_NAV2_ENABLE_MOTION:-false}"
SHARED="${S2_NAV2_ALLOW_SHARED_OUTPUT:-false}"
WS="${S2_NAV2_WS:-${ROOT}/nav2_ws}"

if [[ ! -f "${WS}/install/setup.bash" ]]; then
  echo "[s2-nav2] Nav2 is not built: ${WS}/install/setup.bash" >&2
  echo "[s2-nav2] run ${ROOT}/tools/build_s2_nav2_source.sh first" >&2
  exit 1
fi

set +u
source "${ROOT}/tools/s2_nav2_source_env.sh"
set -u

LAUNCH_ARGS=(
  "use_rviz:=${S2_NAV2_USE_RVIZ:-true}"
  "enable_motion:=${MOTION}"
  "allow_shared_output:=${SHARED}"
  "controller_host:=${S2_CONTROLLER_IP:-192.168.127.10}"
  "robot_id:=${S2_ROBOT_ID:-1}"
  "max_linear_x:=${S2_NAV2_MAX_LINEAR_X:-0.12}"
  "max_linear_y:=${S2_NAV2_MAX_LINEAR_Y:-0.10}"
  "max_angular_z:=${S2_NAV2_MAX_ANGULAR_Z:-0.30}"
  "command_transport:=${S2_NAV2_COMMAND_TRANSPORT:-ros_topic}"
  "command_output_topic:=${S2_NAV2_COMMAND_OUTPUT_TOPIC:-/move/ManualMoveCmd}"
  "urdf_file:=${S2_URDF_FILE:-${ROOT}/robot_description/S2_3DURDF_gazebo_clean.urdf}"
  "params_file:=${S2_NAV2_PARAMS:-${ROOT}/configs/navigation/s2_nav2_params.yaml}"
)
if [[ -n "${S2_MAP_YAML:-}" ]]; then
  LAUNCH_ARGS+=("map_yaml:=${S2_MAP_YAML}")
fi

exec ros2 launch "${ROOT}/tools/s2_nav2_navigation.launch.py" "${LAUNCH_ARGS[@]}"
