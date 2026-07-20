#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

/usr/bin/python3 "${ROOT}/tools/s2_nav2_cmd_vel_bridge.py" "$@"
status=$?
if [[ ${status} -eq 0 ]]; then
  exit 0
fi

echo "[s2-keyboard-map] 控制器状态接口暂时不可用，尝试复用当前手动控制状态" >&2
controller="${S2_CONTROLLER_IP:-192.168.127.10}"
robot_id="${S2_ROBOT_ID:-1}"
for attempt in 1 2 3 4 5; do
  manual="$(
    curl -sS --max-time 3 \
      -H "Content-Type: application/json" \
      -d "{\"id\":${robot_id}}" \
      "http://${controller}:8080/api/AMR/SetManualMode" 2>/dev/null || true
  )"
  unpark="$(
    curl -sS --max-time 3 \
      -H "Content-Type: application/json" \
      -d '{}' \
      "http://${controller}:8080/api/AMR/StopParking" 2>/dev/null || true
  )"
  # StopParking returns ErrorCode 5/Conflict when parking is already released,
  # which is also the desired state for keyboard control.
  if [[ "${manual}" == *'"ErrorCode":0'* ]] && {
    [[ "${unpark}" == *'"ErrorCode":0'* ]] ||
    [[ "${unpark}" == *'"ErrorCode":5'* && "${unpark}" == *'Conflict'* ]]
  }; then
    echo "[s2-keyboard-map] 底盘已切换为手动模式并退出驻车"
    break
  fi
  echo "[s2-keyboard-map] 状态切换重试 ${attempt}/5" >&2
  sleep 1
done
exec /usr/bin/python3 "${ROOT}/tools/s2_nav2_cmd_vel_bridge.py" \
  "$@" --skip-set-manual-mode --skip-exit-parking
