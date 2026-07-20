#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="${S2_RVIZ_LOG:-/tmp/s2_keyboard_mapping_rviz.log}"

# RViz/OGRE on this Intel/Mesa combination emits a known GLSL sampler warning
# while its Map display continues to work. Keep the terminal focused on
# actionable keyboard/controller errors; the full RViz log remains available.
exec rviz2 -d "${ROOT}/configs/rviz/s2_lidar_mapping.rviz" >"${LOG}" 2>&1
