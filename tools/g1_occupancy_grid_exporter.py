#!/usr/bin/env python3
"""Export a ROS OccupancyGrid (typically LiDAR SLAM) as atomic map files."""

from __future__ import annotations

import argparse
import json
import math
import os
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-topic", default="/g1_lidar_slam/map")
    parser.add_argument("--odom-topic", default="/fast_lio2/Odometry")
    parser.add_argument(
        "--odom-yaw-mode",
        choices=("auto", "quaternion", "orientation_w", "orientation_z"),
        default="quaternion",
    )
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="body")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--output-prefix", default="lidar_nav_map_latest")
    parser.add_argument("--occupied-threshold", type=int, default=65)
    parser.add_argument("--free-threshold", type=int, default=25)
    parser.add_argument("--filter-radius-cells", type=int, default=2)
    parser.add_argument("--filter-min-support-cells", type=int, default=3)
    parser.add_argument("--filter-max-component-cells", type=int, default=2)
    parser.add_argument("--save-raw-map", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--pose-hz", type=float, default=2.0)
    parser.add_argument("--pose-stale-sec", type=float, default=2.0)
    parser.add_argument("--max-abs-position-m", type=float, default=100.0)
    parser.add_argument("--max-pose-speed-mps", type=float, default=5.0)
    return parser.parse_args()


def atomic_bytes(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{threading.get_ident()}.tmp")
    with temporary.open("wb") as handle:
        handle.write(content)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def atomic_text(path: Path, content: str) -> None:
    atomic_bytes(path, content.encode("utf-8"))


def quaternion_yaw(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def odometry_yaw(q: Any, mode: str) -> float:
    if mode == "auto":
        values = np.asarray([q.x, q.y, q.z, q.w], dtype=np.float64)
        if abs(float(np.linalg.norm(values)) - 1.0) <= 0.15:
            return quaternion_yaw(q)
        if abs(float(q.x)) < 1e-6 and abs(float(q.y)) < 1e-6:
            if abs(float(q.z)) < 1e-6 and abs(float(q.w)) <= 2.0 * math.pi:
                return float(q.w)
            if abs(float(q.w)) < 1e-6 and abs(float(q.z)) <= 2.0 * math.pi:
                return float(q.z)
        return quaternion_yaw(q)
    if mode == "orientation_w":
        return float(q.w)
    if mode == "orientation_z":
        return float(q.z)
    return quaternion_yaw(q)


def pgm_bytes(image: np.ndarray) -> bytes:
    height, width = image.shape
    return f"P5\n{width} {height}\n255\n".encode("ascii") + image.astype(np.uint8).tobytes()


def occupancy_grid_to_image(
    values: Any,
    width: int,
    height: int,
    *,
    occupied_threshold: int = 65,
    free_threshold: int = 25,
) -> np.ndarray:
    """Convert ROS bottom-left occupancy rows to a top-left PGM image."""

    raw = np.asarray(values, dtype=np.int16).reshape(int(height), int(width))
    image_bottom_up = np.full((int(height), int(width)), 205, dtype=np.uint8)
    image_bottom_up[(raw >= 0) & (raw <= int(free_threshold))] = 254
    image_bottom_up[raw >= int(occupied_threshold)] = 0
    return np.flipud(image_bottom_up)


def neighborhood_counts(mask: np.ndarray, radius: int) -> np.ndarray:
    """Count true cells in a square neighborhood without external image libraries."""

    radius = max(0, int(radius))
    binary = np.asarray(mask, dtype=np.uint8)
    if radius == 0:
        return binary.astype(np.int32)
    padded = np.pad(binary, radius, mode="constant")
    integral = np.pad(padded, ((1, 0), (1, 0)), mode="constant").cumsum(0).cumsum(1)
    kernel = radius * 2 + 1
    return (
        integral[kernel:, kernel:]
        - integral[:-kernel, kernel:]
        - integral[kernel:, :-kernel]
        + integral[:-kernel, :-kernel]
    ).astype(np.int32)


def small_component_mask(mask: np.ndarray, max_cells: int, protected: np.ndarray) -> np.ndarray:
    """Return tiny 8-connected components that do not touch protected cells."""

    max_cells = max(0, int(max_cells))
    result = np.zeros_like(mask, dtype=bool)
    if max_cells == 0:
        return result
    occupied = np.asarray(mask, dtype=bool)
    protected = np.asarray(protected, dtype=bool)
    visited = np.zeros_like(occupied, dtype=bool)
    height, width = occupied.shape
    for row, col in zip(*np.nonzero(occupied & ~visited)):
        if visited[row, col]:
            continue
        stack = [(int(row), int(col))]
        visited[row, col] = True
        component: list[tuple[int, int]] = []
        touches_protected = False
        while stack:
            current_row, current_col = stack.pop()
            component.append((current_row, current_col))
            touches_protected = touches_protected or bool(protected[current_row, current_col])
            for row_offset in (-1, 0, 1):
                for col_offset in (-1, 0, 1):
                    if row_offset == 0 and col_offset == 0:
                        continue
                    next_row = current_row + row_offset
                    next_col = current_col + col_offset
                    if not (0 <= next_row < height and 0 <= next_col < width):
                        continue
                    if occupied[next_row, next_col] and not visited[next_row, next_col]:
                        visited[next_row, next_col] = True
                        stack.append((next_row, next_col))
        if len(component) <= max_cells and not touches_protected:
            for component_row, component_col in component:
                result[component_row, component_col] = True
    return result


def filter_occupied_outliers(
    image: np.ndarray,
    *,
    radius_cells: int = 2,
    min_support_cells: int = 3,
    max_component_cells: int = 2,
) -> tuple[np.ndarray, dict[str, int]]:
    """Remove sparse occupied evidence only inside fully observed free space."""

    filtered = np.asarray(image, dtype=np.uint8).copy()
    occupied = filtered == 0
    unknown = filtered == 205
    near_unknown = neighborhood_counts(unknown, 1) > 0
    support = neighborhood_counts(occupied, radius_cells)
    sparse = occupied & (support < max(1, int(min_support_cells))) & ~near_unknown
    kept = occupied & ~sparse
    tiny = small_component_mask(kept, max_component_cells, near_unknown)
    removed = sparse | tiny
    filtered[removed] = 254
    return filtered, {
        "occupied_cells_raw": int(np.count_nonzero(occupied)),
        "occupied_cells_filtered": int(np.count_nonzero(filtered == 0)),
        "removed_sparse_cells": int(np.count_nonzero(sparse)),
        "removed_tiny_component_cells": int(np.count_nonzero(tiny)),
    }


def pose_plausibility_reason(
    xyt: list[float] | None,
    source_stamp: float | None,
    previous: tuple[float, list[float]] | None,
    *,
    max_abs_position_m: float,
    max_pose_speed_mps: float,
) -> str:
    if xyt is None or source_stamp is None or len(xyt) < 3:
        return "pose_unavailable"
    if not all(math.isfinite(float(value)) for value in xyt[:3]):
        return "pose_nonfinite"
    if math.hypot(float(xyt[0]), float(xyt[1])) > float(max_abs_position_m):
        return "pose_outside_absolute_limit"
    if previous is not None:
        previous_stamp, previous_xyt = previous
        delta_sec = float(source_stamp) - float(previous_stamp)
        if delta_sec > 1e-3:
            distance = math.hypot(float(xyt[0]) - previous_xyt[0], float(xyt[1]) - previous_xyt[1])
            if distance / delta_sec > float(max_pose_speed_mps):
                return "pose_jump_too_fast"
    return "ok"


def main() -> None:
    args = parse_args()
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    prefix = str(args.output_prefix)
    paths = {
        "yaml": output_dir / f"{prefix}.yaml",
        "pgm": output_dir / f"{prefix}.pgm",
        "raw_pgm": output_dir / f"{prefix}_raw.pgm",
        "json": output_dir / f"{prefix}.json",
    }

    class Exporter(Node):
        def __init__(self) -> None:
            super().__init__("g1_occupancy_grid_exporter")
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.lock = threading.Lock()
            self.map_metadata: dict[str, Any] = {}
            self.latest_odom: tuple[float, list[float], str] | None = None
            self.last_valid_pose: tuple[float, list[float]] | None = None
            map_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(OccupancyGrid, args.map_topic, self.on_map, map_qos)
            self.create_subscription(Odometry, args.odom_topic, self.on_odom, 20)
            self.create_timer(1.0 / max(0.1, float(args.pose_hz)), self.publish_pose)

        def on_odom(self, msg: Any) -> None:
            pose = msg.pose.pose
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            self.latest_odom = (
                time.time(),
                [
                    float(pose.position.x),
                    float(pose.position.y),
                    odometry_yaw(pose.orientation, args.odom_yaw_mode),
                ],
                str(msg.header.frame_id),
            )

        def on_map(self, msg: Any) -> None:
            width = int(msg.info.width)
            height = int(msg.info.height)
            raw_image = occupancy_grid_to_image(
                msg.data,
                width,
                height,
                occupied_threshold=int(args.occupied_threshold),
                free_threshold=int(args.free_threshold),
            )
            image, filter_stats = filter_occupied_outliers(
                raw_image,
                radius_cells=int(args.filter_radius_cells),
                min_support_cells=int(args.filter_min_support_cells),
                max_component_cells=int(args.filter_max_component_cells),
            )
            origin = msg.info.origin
            origin_yaw = quaternion_yaw(origin.orientation)
            map_yaml = {
                "image": paths["pgm"].name,
                "mode": "trinary",
                "resolution": float(msg.info.resolution),
                "origin": [float(origin.position.x), float(origin.position.y), origin_yaw],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }
            if bool(args.save_raw_map):
                atomic_bytes(paths["raw_pgm"], pgm_bytes(raw_image))
            atomic_bytes(paths["pgm"], pgm_bytes(image))
            atomic_text(paths["yaml"], yaml.safe_dump(map_yaml, sort_keys=False, allow_unicode=True))
            with self.lock:
                self.map_metadata = {
                    "ok": True,
                    "source": "ros_occupancy_grid",
                    "map_topic": args.map_topic,
                    "frame_id": str(msg.header.frame_id or args.map_frame),
                    "width": width,
                    "height": height,
                    "resolution_m": float(msg.info.resolution),
                    "origin": map_yaml["origin"],
                    "paths": {key: str(path) for key, path in paths.items()},
                    "occupancy_filter": {
                        "enabled": True,
                        "radius_cells": int(args.filter_radius_cells),
                        "min_support_cells": int(args.filter_min_support_cells),
                        "max_component_cells": int(args.filter_max_component_cells),
                        "unknown_boundary_protected": True,
                        **filter_stats,
                    },
                    "map_stamp": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
                    "map_updated_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                }
            self.publish_pose()
            self.get_logger().info(f"exported {width}x{height} map at {msg.info.resolution:.3f} m/cell")

        def map_pose(self) -> tuple[list[float] | None, str, float | None]:
            try:
                transform = self.tf_buffer.lookup_transform(
                    args.map_frame,
                    args.base_frame,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
                value = transform.transform
                stamp = float(transform.header.stamp.sec) + float(transform.header.stamp.nanosec) * 1e-9
                return [float(value.translation.x), float(value.translation.y), quaternion_yaw(value.rotation)], "tf", stamp
            except TransformException:
                pass
            if self.latest_odom is not None:
                received_at, xyt, frame_id = self.latest_odom
                if frame_id == args.map_frame:
                    return list(xyt), "odom_same_map_frame", received_at
            return None, "unavailable", None

        def publish_pose(self) -> None:
            with self.lock:
                if not self.map_metadata:
                    return
                payload = dict(self.map_metadata)
            xyt, source, source_stamp = self.map_pose()
            age = float("inf") if source_stamp is None else max(0.0, time.time() - source_stamp)
            reason = pose_plausibility_reason(
                xyt,
                source_stamp,
                self.last_valid_pose,
                max_abs_position_m=float(args.max_abs_position_m),
                max_pose_speed_mps=float(args.max_pose_speed_mps),
            )
            valid = reason == "ok" and age <= float(args.pose_stale_sec)
            if reason == "ok" and age > float(args.pose_stale_sec):
                reason = "pose_stale"
            if valid and source_stamp is not None and xyt is not None:
                self.last_valid_pose = (float(source_stamp), list(xyt))
            payload.update(
                {
                    "robot_xyt": xyt if valid else None,
                    "pose_valid": valid,
                    "pose_validation_reason": reason,
                    "pose_source": source,
                    "pose_source_stamp": source_stamp,
                    "pose_age_sec": age if math.isfinite(age) else None,
                    "updated_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                    "coordinate_contract": (
                        "Scene-graph coordinates must be transformed into this same map frame before combined FQPlanner publication."
                    ),
                }
            )
            atomic_text(paths["json"], json.dumps(payload, indent=2, ensure_ascii=False) + "\n")

    rclpy.init()
    node = Exporter()
    print(f"[lidar-map] waiting for {args.map_topic}; output prefix={output_dir / prefix}", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
