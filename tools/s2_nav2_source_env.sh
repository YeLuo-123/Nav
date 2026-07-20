#!/usr/bin/env bash
# Source this file before launching the locally compiled Nav2 workspace.

S2_ENV_ROOT="${S2_DREAM_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
S2_NAV2_WS="${S2_NAV2_WS:-${S2_ENV_ROOT}/nav2_ws}"
S2_NAV2_DEPS="${S2_NAV2_DEPS:-${S2_NAV2_WS}/local_deps}"
S2_NAV2_ROS_LOCAL="${S2_NAV2_DEPS}/opt/ros/humble"

set +u
source /opt/ros/humble/setup.bash
set -u

remove_path_prefix() {
  local value="${1:-}"
  local prefix="${2:-}"
  local result=""
  local entry
  local old_ifs="${IFS}"
  IFS=:
  for entry in ${value}; do
    [[ -z "${entry}" || "${entry}" == "${prefix}"* ]] && continue
    result="${result:+${result}:}${entry}"
  done
  IFS="${old_ifs}"
  printf '%s' "${result}"
}

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  export LD_LIBRARY_PATH="$(remove_path_prefix "${LD_LIBRARY_PATH:-}" "${CONDA_PREFIX}")"
  export LIBRARY_PATH="$(remove_path_prefix "${LIBRARY_PATH:-}" "${CONDA_PREFIX}")"
  export CPATH="$(remove_path_prefix "${CPATH:-}" "${CONDA_PREFIX}")"
  export PKG_CONFIG_PATH="$(remove_path_prefix "${PKG_CONFIG_PATH:-}" "${CONDA_PREFIX}")"
fi
unset -f remove_path_prefix

export AMENT_PREFIX_PATH="${S2_NAV2_ROS_LOCAL}:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="${S2_NAV2_ROS_LOCAL}:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${S2_NAV2_ROS_LOCAL}/lib:${S2_NAV2_DEPS}/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export LIBRARY_PATH="${S2_NAV2_ROS_LOCAL}/lib:${S2_NAV2_DEPS}/usr/lib/x86_64-linux-gnu:${LIBRARY_PATH:-}"
export CPATH="${S2_NAV2_ROS_LOCAL}/include:${S2_NAV2_DEPS}/usr/include:${CPATH:-}"
export PKG_CONFIG_PATH="${S2_NAV2_DEPS}/usr/lib/x86_64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH:-}"

set +u
if [[ -f "${S2_NAV2_WS}/install/setup.bash" ]]; then
  source "${S2_NAV2_WS}/install/setup.bash"
else
  echo "[s2-nav2] install space not found; build ${S2_NAV2_WS} first" >&2
fi
set -u

export ROS_DOMAIN_ID="${S2_ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DISABLE_DAEMON=1
export CYCLONEDDS_URI="file://${S2_CYCLONEDDS_CONFIG:-${S2_ENV_ROOT}/configs/robots/s2_cyclonedds.xml}"

echo "[s2-nav2] workspace=${S2_NAV2_WS} domain=${ROS_DOMAIN_ID}"
unset S2_ENV_ROOT
