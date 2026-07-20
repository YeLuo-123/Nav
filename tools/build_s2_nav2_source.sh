#!/usr/bin/env bash
set -euo pipefail

ROOT="${S2_DREAM_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
WS="${S2_NAV2_WS:-${ROOT}/nav2_ws}"

set +u
source "${ROOT}/tools/s2_nav2_source_env.sh"
set -u

cd "${WS}"
/usr/bin/colcon build \
  --base-paths src \
  --symlink-install \
  --executor parallel \
  --parallel-workers "${S2_NAV2_BUILD_WORKERS:-4}" \
  --packages-up-to \
    nav2_planner \
    nav2_controller \
    nav2_bt_navigator \
    nav2_behaviors \
    nav2_lifecycle_manager \
    nav2_rviz_plugins \
    nav2_velocity_smoother \
  --packages-skip nav2_map_server \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
  --event-handlers console_cohesion+
