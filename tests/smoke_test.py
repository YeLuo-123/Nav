#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(os.environ.get("S2_BUNDLE_ROOT", Path(__file__).resolve().parents[1]))


def load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


required = [
    ROOT / "bin/s2_keyboard",
    ROOT / "tools/s2_lidar_mapping.sh",
    ROOT / "tools/s2_registered_cloud_mapper.py",
    ROOT / "tools/s2_lidar_rviz_bridge.py",
    ROOT / "tools/s2_nav2_navigation.launch.py",
    ROOT / "configs/navigation/s2_nav2_params.yaml",
    ROOT / "configs/rviz/s2_lidar_mapping.rviz",
    ROOT / "robot_description/S2_3DURDF_gazebo_clean.urdf",
    ROOT / "robot_description/meshes/YD_link.STL",
]
missing = [str(path) for path in required if not path.is_file()]
if missing:
    raise SystemExit("missing bundle files:\n" + "\n".join(missing))

model = load("s2_robot_model", ROOT / "tools/s2_robot_model.py")
description = model.load_rviz_robot_description(
    ROOT / "robot_description/S2_3DURDF_gazebo_clean.urdf"
)
if "package://fqplanner_gazebo_nav" in description:
    raise SystemExit("URDF still contains unresolved package URIs")
ET.fromstring(description)

params = yaml.safe_load(
    (ROOT / "configs/navigation/s2_nav2_params.yaml").read_text(encoding="utf-8")
)
footprint = params["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
if "0.46" not in footprint or "0.36" not in footprint:
    raise SystemExit(f"unexpected Nav2 footprint: {footprint}")

for script in list((ROOT / "bin").iterdir()) + list((ROOT / "setup").glob("*.sh")):
    subprocess.run(["bash", "-n", str(script)], check=True)

print("S2 bundle smoke test: PASS")
