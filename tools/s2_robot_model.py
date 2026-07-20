#!/usr/bin/env python3
"""Prepare the supplied S2 URDF for local RViz visualization."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = ROOT / "robot_description/S2_3DURDF_gazebo_clean.urdf"


def infer_mesh_root(urdf_path: Path) -> Path:
    candidates = (
        urdf_path.parent / "meshes",
        urdf_path.parent.parent / "meshes",
    )
    for candidate in candidates:
        if candidate.is_dir():
            return candidate.resolve()
    raise FileNotFoundError(f"cannot find meshes directory beside {urdf_path}")


def load_rviz_robot_description(
    urdf_path: str | Path = DEFAULT_URDF,
    mesh_root: str | Path | None = None,
) -> str:
    source = Path(urdf_path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(source)

    resolved_mesh_root = (
        Path(mesh_root).expanduser().resolve()
        if mesh_root is not None
        else infer_mesh_root(source)
    )
    root = ET.parse(source).getroot()

    # Gazebo plugins and ros2_control are irrelevant to robot_state_publisher.
    for child in list(root):
        if child.tag in {"gazebo", "ros2_control"}:
            root.remove(child)

    child_links = {
        child.attrib["link"]
        for child in root.findall("./joint/child")
        if "link" in child.attrib
    }
    root_link = next(
        (
            link
            for link in root.findall("link")
            if link.attrib.get("name") not in child_links
        ),
        None,
    )
    if root_link is not None:
        inertial = root_link.find("inertial")
        if inertial is not None:
            root_link.remove(inertial)

    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        mesh_path = resolved_mesh_root / Path(filename).name
        if not mesh_path.is_file():
            raise FileNotFoundError(
                f"URDF mesh {filename!r} was not found under {resolved_mesh_root}"
            )
        mesh.set("filename", mesh_path.as_uri())

    return ET.tostring(root, encoding="unicode")
