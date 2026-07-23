#!/usr/bin/env bash
# Source this file, or use the scripts under bin/ which source it automatically.

S2_BUNDLE_ROOT="${S2_BUNDLE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
export S2_BUNDLE_ROOT
export S2_DREAM_ROOT="${S2_DREAM_ROOT:-${S2_BUNDLE_ROOT}}"
export S2_NAV2_WS="${S2_NAV2_WS:-${S2_BUNDLE_ROOT}/nav2_ws}"
export S2_NAV2_DEPS="${S2_NAV2_DEPS:-${S2_NAV2_WS}/local_deps}"
export S2_URDF_FILE="${S2_URDF_FILE:-${S2_BUNDLE_ROOT}/robot_description/S2_3DURDF_gazebo_clean.urdf}"

export S2_CONTROLLER_IP="${S2_CONTROLLER_IP:-192.168.127.10}"
export S2_HOST_CIDR="${S2_HOST_CIDR:-192.168.127.100/24}"
export S2_ROS_DOMAIN_ID="${S2_ROS_DOMAIN_ID:-0}"
export S2_NETWORK_INTERFACE="${1:-${S2_NETWORK_INTERFACE:-}}"

if [[ -z "${S2_NETWORK_INTERFACE}" ]]; then
  S2_NETWORK_INTERFACE="$(
    ip -o -4 address show 2>/dev/null \
      | awk '$4 ~ /^192\.168\.127\./ {print $2; exit}'
  )"
fi
export S2_NETWORK_INTERFACE="${S2_NETWORK_INTERFACE:-eno1}"

# Controller HTTP/WebSocket traffic must never pass through a desktop proxy.
# Some curl/urllib versions do not match CIDR entries in no_proxy reliably, so
# include the exact robot address in both conventional variable spellings.
export no_proxy="${S2_CONTROLLER_IP},${no_proxy:-}"
export NO_PROXY="${S2_CONTROLLER_IP},${NO_PROXY:-${no_proxy}}"

dds_safe_interface="${S2_NETWORK_INTERFACE//[^A-Za-z0-9_.-]/_}"
dds_runtime="${TMPDIR:-/tmp}/s2_cyclonedds_${USER:-user}_${dds_safe_interface}.xml"
cat >"${dds_runtime}" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="${S2_NETWORK_INTERFACE}"/>
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer Address="${S2_CONTROLLER_IP}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
unset dds_safe_interface
export S2_CYCLONEDDS_CONFIG="${S2_CYCLONEDDS_CONFIG:-${dds_runtime}}"
unset dds_runtime

export ROS_DOMAIN_ID="${S2_ROS_DOMAIN_ID}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DISABLE_DAEMON=1
export CYCLONEDDS_URI="file://${S2_CYCLONEDDS_CONFIG}"

echo "[s2-bundle] root=${S2_BUNDLE_ROOT}"
echo "[s2-bundle] interface=${S2_NETWORK_INTERFACE} host=${S2_HOST_CIDR} controller=${S2_CONTROLLER_IP}"
