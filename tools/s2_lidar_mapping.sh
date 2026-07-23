#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-status}"
ROOT="${S2_DREAM_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
INTERFACE="${S2_NETWORK_INTERFACE:-eno1}"
HOST_CIDR="${S2_HOST_CIDR:-192.168.127.100/24}"
CONTROLLER_IP="${S2_CONTROLLER_IP:-192.168.127.10}"
DOMAIN_ID="${S2_ROS_DOMAIN_ID:-0}"
CYCLONEDDS_CONFIG="${S2_CYCLONEDDS_CONFIG:-${ROOT}/configs/robots/s2_cyclonedds.xml}"
OUTPUT_DIR="${S2_LIDAR_OUTPUT_DIR:-${ROOT}/debug/s2_lidar_mapping/latest}"
HTTP_PORT="${S2_LIDAR_HTTP_PORT:-8776}"
MAP_TOPIC="${S2_MAP_TOPIC:-/map}"
ODOM_TOPIC="${S2_ODOM_TOPIC:-/controller/odom}"
CLOUD_TOPIC="${S2_CLOUD_TOPIC:-/driver/lidar/point_cloud/Data}"
CLOUD_FRAME_MODE="${S2_CLOUD_FRAME_MODE:-base}"

PID_DIR="${OUTPUT_DIR}/pids"
MAPPER_PID="${PID_DIR}/mapper.pid"
EXPORTER_PID="${PID_DIR}/exporter.pid"
HTTP_PID="${PID_DIR}/http.pid"

setup_ros() {
  set +u
  source /opt/ros/humble/setup.bash
  [[ -f "${S2_LIVOX_WS:-${ROOT}/livox_ws}/install/setup.bash" ]] && source "${S2_LIVOX_WS:-${ROOT}/livox_ws}/install/setup.bash"
  set -u
  export ROS_DOMAIN_ID="$DOMAIN_ID"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DISABLE_DAEMON=1
  export CYCLONEDDS_URI="file://${CYCLONEDDS_CONFIG}"
}

preflight_network() {
  if [[ ! -e "/sys/class/net/${INTERFACE}" ]]; then
    echo "[s2-lidar] network interface does not exist: ${INTERFACE}" >&2
    return 1
  fi
  if [[ "$(cat "/sys/class/net/${INTERFACE}/carrier" 2>/dev/null || echo 0)" != "1" ]]; then
    echo "[s2-lidar] ${INTERFACE} has no Ethernet carrier; check the cable and S2 power" >&2
    return 1
  fi
  if ! ip -4 -o address show dev "$INTERFACE" | awk '{print $4}' | grep -Fxq "$HOST_CIDR"; then
    echo "[s2-lidar] ${INTERFACE} is missing ${HOST_CIDR}; run: $0 network" >&2
    return 1
  fi
  if ! ping -c 1 -W 1 "$CONTROLLER_IP" >/dev/null 2>&1; then
    echo "[s2-lidar] S2 controller is unreachable: ${CONTROLLER_IP}" >&2
    return 1
  fi
}

stop_pid() {
  local path="$1"
  if [[ -f "$path" ]]; then
    local pid
    pid="$(cat "$path")"
    kill "$pid" >/dev/null 2>&1 || true
    rm -f "$path"
  fi
}

network() {
  nmcli connection delete DREAM-S2-test >/dev/null 2>&1 || true
  nmcli connection add type ethernet ifname "$INTERFACE" con-name DREAM-S2-test \
    ipv4.method manual ipv4.addresses "$HOST_CIDR" ipv4.never-default yes \
    ipv6.method disabled >/dev/null
  nmcli connection up DREAM-S2-test >/dev/null
  ip -br address show "$INTERFACE"
  ip neigh flush dev "$INTERFACE" >/dev/null 2>&1 || true
  ping -c 1 -W 1 "$CONTROLLER_IP" >/dev/null
  ip neigh show "$CONTROLLER_IP" dev "$INTERFACE"
}

diagnose() {
  preflight_network
  setup_ros
  mkdir -p "$OUTPUT_DIR"
  /usr/bin/python3 "$ROOT/tools/s2_lidar_topic_diagnostics.py" \
    --duration-sec "${S2_DIAGNOSTIC_DURATION_SEC:-12}" \
    --output "$OUTPUT_DIR/s2_lidar_topic_diagnostics_latest.json"
}

start_http() {
  mkdir -p "$PID_DIR"
  nohup python3 -m http.server "$HTTP_PORT" --bind 0.0.0.0 --directory "$OUTPUT_DIR" \
    >"$OUTPUT_DIR/http.log" 2>&1 &
  echo $! >"$HTTP_PID"
}

start_native_map() {
  stop_all
  preflight_network
  setup_ros
  mkdir -p "$PID_DIR"
  nohup /usr/bin/python3 "$ROOT/tools/g1_occupancy_grid_exporter.py" \
    --map-topic "$MAP_TOPIC" \
    --odom-topic "$ODOM_TOPIC" \
    --odom-yaw-mode "${S2_ODOM_YAW_MODE:-orientation_w}" \
    --map-frame "${S2_MAP_FRAME:-odom}" \
    --base-frame "${S2_BASE_FRAME:-base_link}" \
    --output-dir "$OUTPUT_DIR" \
    --output-prefix lidar_nav_map_latest \
    >"$OUTPUT_DIR/exporter.log" 2>&1 &
  echo $! >"$EXPORTER_PID"
  sleep 1
  if ! kill -0 "$(cat "$EXPORTER_PID")" >/dev/null 2>&1; then
    echo "[s2-lidar] native map exporter failed to start:" >&2
    tail -40 "$OUTPUT_DIR/exporter.log" >&2 || true
    rm -f "$EXPORTER_PID"
    return 1
  fi
  start_http
  status_all
}

start_registered_cloud() {
  stop_all
  preflight_network
  setup_ros
  mkdir -p "$PID_DIR"
  nohup /usr/bin/python3 "$ROOT/tools/s2_registered_cloud_mapper.py" \
    --cloud-topic "$CLOUD_TOPIC" \
    --output-cloud-topic "${S2_OUTPUT_CLOUD_TOPIC:-/s2_lidar_slam/point_cloud}" \
    --cloud-publish-hz "${S2_OUTPUT_CLOUD_HZ:-3.0}" \
    --export-hz "${S2_MAP_EXPORT_HZ:-0.5}" \
    --odom-topic "$ODOM_TOPIC" \
    --odom-yaw-mode "${S2_ODOM_YAW_MODE:-orientation_w}" \
    --output-dir "$OUTPUT_DIR" \
    --cloud-frame-mode "$CLOUD_FRAME_MODE" \
    --map-frame "${S2_MAP_FRAME:-odom}" \
    --min-z "${S2_LIDAR_MIN_Z:-0.25}" \
    --max-z "${S2_LIDAR_MAX_Z:-1.20}" \
    --overhang-min-z "${S2_OVERHANG_MIN_Z:-0.35}" \
    --overhang-max-z "${S2_OVERHANG_MAX_Z:-1.60}" \
    --overhang-inflation-m "${S2_OVERHANG_INFLATION_M:-0.10}" \
    --overhang-miss-log-odds "${S2_OVERHANG_MISS_LOG_ODDS:-0.20}" \
    --max-range "${S2_LIDAR_MAX_RANGE_M:-8.0}" \
    --max-cloud-odom-delta-sec "${S2_MAX_CLOUD_ODOM_DELTA_SEC:-0.05}" \
    >"$OUTPUT_DIR/mapper.log" 2>&1 &
  echo $! >"$MAPPER_PID"
  sleep 1
  if ! kill -0 "$(cat "$MAPPER_PID")" >/dev/null 2>&1; then
    echo "[s2-lidar] registered-cloud mapper failed to start:" >&2
    tail -40 "$OUTPUT_DIR/mapper.log" >&2 || true
    rm -f "$MAPPER_PID"
    return 1
  fi
  start_http
  status_all
}

stop_all() {
  stop_pid "$MAPPER_PID"
  stop_pid "$EXPORTER_PID"
  stop_pid "$HTTP_PID"
}

status_all() {
  local mapper_running=false
  local exporter_running=false
  local http_running=false
  [[ -f "$MAPPER_PID" ]] && kill -0 "$(cat "$MAPPER_PID")" >/dev/null 2>&1 && mapper_running=true
  [[ -f "$EXPORTER_PID" ]] && kill -0 "$(cat "$EXPORTER_PID")" >/dev/null 2>&1 && exporter_running=true
  [[ -f "$HTTP_PID" ]] && kill -0 "$(cat "$HTTP_PID")" >/dev/null 2>&1 && http_running=true
  echo "interface=$INTERFACE host=$HOST_CIDR controller=$CONTROLLER_IP domain=$DOMAIN_ID"
  echo "mapper_running=$mapper_running exporter_running=$exporter_running http_running=$http_running"
  ip -br address show "$INTERFACE" || true
  ip neigh show "$CONTROLLER_IP" dev "$INTERFACE" || true
  for path in "$MAPPER_PID" "$EXPORTER_PID" "$HTTP_PID"; do
    if [[ -f "$path" ]]; then
      pid="$(cat "$path")"
      if ! ps -p "$pid" -o pid=,stat=,etime=,cmd=; then
        echo "stale_pid_file=$path pid=$pid"
      fi
    fi
  done
  if [[ -f "$OUTPUT_DIR/lidar_nav_map_latest.json" ]]; then
    python3 -c 'import json,sys,time,os; p=sys.argv[1]; d=json.load(open(p)); print({k:d.get(k) for k in ("source","width","height","odom_yaw_mode","robot_xyt","pose_valid","cloud_count","integrated_scan_count","sync_skip_count","cloud_odom_sync_delta_sec","map_updated_at")}); print(f"map_file_age_sec={max(0.0, time.time()-os.path.getmtime(p)):.1f}")' "$OUTPUT_DIR/lidar_nav_map_latest.json"
  fi
}

case "$ACTION" in
  network) network ;;
  diagnose) diagnose ;;
  start-native-map) start_native_map ;;
  start-registered-cloud) start_registered_cloud ;;
  stop) stop_all ;;
  status) status_all ;;
  *)
    echo "Usage: $0 {network|diagnose|start-native-map|start-registered-cloud|stop|status}" >&2
    exit 2
    ;;
esac
