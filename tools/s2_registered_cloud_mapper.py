#!/usr/bin/env python3
"""Build a filtered 2D occupancy map from a registered S2 PointCloud2 stream."""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml
import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent))

from g1_occupancy_grid_exporter import (
    atomic_bytes,
    atomic_text,
    filter_occupied_outliers,
    pgm_bytes,
    quaternion_yaw,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cloud-topic", required=True)
    parser.add_argument("--odom-topic", required=True)
    parser.add_argument(
        "--odom-yaw-mode",
        choices=("auto", "quaternion", "orientation_w", "orientation_z"),
        default="orientation_w",
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--output-prefix", default="lidar_nav_map_latest")
    parser.add_argument("--map-topic", default="/s2_lidar_slam/map")
    parser.add_argument("--output-cloud-topic", default="/s2_lidar_slam/point_cloud")
    parser.add_argument("--cloud-publish-hz", type=float, default=3.0)
    parser.add_argument("--output-cloud-max-points", type=int, default=12000)
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--cloud-frame-mode", choices=("map", "base"), default="map")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--map-size-m", type=float, default=60.0)
    parser.add_argument("--min-z", type=float, default=0.10)
    parser.add_argument("--max-z", type=float, default=0.80)
    parser.add_argument("--min-range", type=float, default=0.25)
    parser.add_argument("--max-range", type=float, default=8.0)
    parser.add_argument("--self-filter-half-x", type=float, default=0.48)
    parser.add_argument("--self-filter-half-y", type=float, default=0.38)
    parser.add_argument("--integration-hz", type=float, default=2.0)
    parser.add_argument("--export-hz", type=float, default=1.0)
    parser.add_argument(
        "--manual-save-only",
        action="store_true",
        help="Publish the live map, but only write map files when the save service is called.",
    )
    parser.add_argument(
        "--save-service",
        default="/s2_lidar_slam/save_map",
        help="std_srvs/Trigger service used to save the current map.",
    )
    parser.add_argument("--max-rays-per-scan", type=int, default=3000)
    parser.add_argument("--hit-log-odds", type=float, default=0.85)
    parser.add_argument("--miss-log-odds", type=float, default=0.40)
    parser.add_argument("--occupied-log-odds", type=float, default=1.1)
    parser.add_argument("--free-log-odds", type=float, default=-0.7)
    parser.add_argument("--odom-stale-sec", type=float, default=2.0)
    parser.add_argument("--max-cloud-odom-delta-sec", type=float, default=0.05)
    parser.add_argument("--odom-history-sec", type=float, default=5.0)
    return parser.parse_args()


def bresenham(row0: int, col0: int, row1: int, col1: int) -> list[tuple[int, int]]:
    points: list[tuple[int, int]] = []
    delta_col = abs(col1 - col0)
    delta_row = -abs(row1 - row0)
    step_col = 1 if col0 < col1 else -1
    step_row = 1 if row0 < row1 else -1
    error = delta_col + delta_row
    while True:
        points.append((row0, col0))
        if row0 == row1 and col0 == col1:
            return points
        doubled = 2 * error
        if doubled >= delta_row:
            error += delta_row
            col0 += step_col
        if doubled <= delta_col:
            error += delta_col
            row0 += step_row


class OccupancyAccumulator:
    def __init__(
        self,
        *,
        resolution: float,
        map_size_m: float,
        hit_log_odds: float,
        miss_log_odds: float,
    ) -> None:
        self.resolution = float(resolution)
        self.size = int(math.ceil(float(map_size_m) / self.resolution))
        self.hit_log_odds = float(hit_log_odds)
        self.miss_log_odds = float(miss_log_odds)
        self.origin_x: float | None = None
        self.origin_y: float | None = None
        self.log_odds = np.zeros((self.size, self.size), dtype=np.float32)
        self.observed = np.zeros((self.size, self.size), dtype=bool)

    def initialize(self, center_x: float, center_y: float) -> None:
        if self.origin_x is None:
            half = self.size * self.resolution * 0.5
            self.origin_x = float(center_x) - half
            self.origin_y = float(center_y) - half

    def cell(self, x: float, y: float) -> tuple[int, int] | None:
        if self.origin_x is None or self.origin_y is None:
            return None
        col = int(math.floor((float(x) - self.origin_x) / self.resolution))
        row = int(math.floor((float(y) - self.origin_y) / self.resolution))
        if 0 <= row < self.size and 0 <= col < self.size:
            return row, col
        return None

    def integrate(self, sensor_xy: tuple[float, float], endpoints_xy: np.ndarray, max_rays: int) -> int:
        self.initialize(*sensor_xy)
        sensor_cell = self.cell(*sensor_xy)
        if sensor_cell is None or endpoints_xy.size == 0:
            return 0
        endpoint_cells = {
            cell for x, y in endpoints_xy for cell in [self.cell(float(x), float(y))] if cell is not None
        }
        ordered = sorted(endpoint_cells)
        if len(ordered) > int(max_rays):
            indices = np.linspace(0, len(ordered) - 1, int(max_rays), dtype=np.int64)
            ordered = [ordered[int(index)] for index in indices]
        for endpoint in ordered:
            ray = bresenham(sensor_cell[0], sensor_cell[1], endpoint[0], endpoint[1])
            for row, col in ray[:-1]:
                self.observed[row, col] = True
                self.log_odds[row, col] -= self.miss_log_odds
            row, col = endpoint
            self.observed[row, col] = True
            self.log_odds[row, col] += self.hit_log_odds
        np.clip(self.log_odds, -3.5, 3.5, out=self.log_odds)
        return len(ordered)

    def occupancy_values(self, occupied_threshold: float, free_threshold: float) -> np.ndarray:
        values = np.full((self.size, self.size), -1, dtype=np.int8)
        values[self.observed & (self.log_odds <= float(free_threshold))] = 0
        values[self.observed & (self.log_odds >= float(occupied_threshold))] = 100
        uncertain = self.observed & (values < 0)
        values[uncertain] = 50
        return values


def pointcloud_xyz(msg: Any) -> np.ndarray:
    from sensor_msgs_py import point_cloud2

    values = np.asarray(point_cloud2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True))
    if values.dtype.names:
        return np.column_stack([values[name] for name in ("x", "y", "z")]).astype(np.float32)
    return values.reshape(-1, 3).astype(np.float32, copy=False)


def odometry_yaw(msg: Any, mode: str) -> float:
    orientation = msg.pose.pose.orientation
    if mode == "auto":
        if (
            abs(float(orientation.x)) < 1e-6
            and abs(float(orientation.y)) < 1e-6
            and abs(float(orientation.z)) < 1e-6
            and abs(float(orientation.w)) <= 2.0 * math.pi
        ):
            return float(orientation.w)
        values = np.asarray(
            [orientation.x, orientation.y, orientation.z, orientation.w], dtype=np.float64
        )
        norm = float(np.linalg.norm(values))
        if abs(norm - 1.0) <= 0.15:
            return quaternion_yaw(orientation)
        if abs(float(orientation.x)) < 1e-6 and abs(float(orientation.y)) < 1e-6:
            if abs(float(orientation.z)) < 1e-6 and abs(float(orientation.w)) <= 2.0 * math.pi:
                return float(orientation.w)
            if abs(float(orientation.w)) < 1e-6 and abs(float(orientation.z)) <= 2.0 * math.pi:
                return float(orientation.z)
        return quaternion_yaw(orientation)
    if mode == "orientation_w":
        return float(orientation.w)
    if mode == "orientation_z":
        return float(orientation.z)
    return quaternion_yaw(orientation)


def stamp_seconds(header: Any) -> float:
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1.0e-9


def transform_base_points(points: np.ndarray, pose: list[float]) -> np.ndarray:
    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
    transformed = np.empty_like(points, dtype=np.float32)
    transformed[:, 0] = pose[0] + cosine * points[:, 0] - sine * points[:, 1]
    transformed[:, 1] = pose[1] + sine * points[:, 0] + cosine * points[:, 1]
    transformed[:, 2] = points[:, 2]
    return transformed


def main() -> None:
    args = parse_args()
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header, UInt64
    from std_srvs.srv import Trigger

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    prefix = str(args.output_prefix)
    paths = {
        "pgm": output_dir / f"{prefix}.pgm",
        "png": output_dir / f"{prefix}.png",
        "raw_pgm": output_dir / f"{prefix}_raw.pgm",
        "yaml": output_dir / f"{prefix}.yaml",
        "json": output_dir / f"{prefix}.json",
    }

    class Mapper(Node):
        def __init__(self) -> None:
            super().__init__("s2_registered_cloud_mapper")
            self.accumulator = OccupancyAccumulator(
                resolution=float(args.resolution),
                map_size_m=float(args.map_size_m),
                hit_log_odds=float(args.hit_log_odds),
                miss_log_odds=float(args.miss_log_odds),
            )
            self.latest_pose: tuple[float, list[float]] | None = None
            self.odom_history: deque[tuple[float, float, list[float]]] = deque()
            self.last_cloud_at = 0.0
            self.last_cloud_publish_at = 0.0
            self.cloud_count = 0
            self.integrated_scan_count = 0
            self.integrated_ray_count = 0
            self.sync_skip_count = 0
            self.self_filtered_point_count = 0
            self.latest_cloud_odom_delta_sec: float | None = None
            self.last_cloud_frame = ""
            sensor_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            map_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.map_publisher = self.create_publisher(OccupancyGrid, args.map_topic, map_qos)
            self.cloud_publisher = self.create_publisher(
                PointCloud2, args.output_cloud_topic, map_qos
            )
            self.cloud_count_publisher = self.create_publisher(
                UInt64, "/s2_lidar_slam/cloud_count", sensor_qos
            )
            self.create_subscription(Odometry, args.odom_topic, self.on_odom, sensor_qos)
            self.create_subscription(PointCloud2, args.cloud_topic, self.on_cloud, sensor_qos)
            self.create_timer(1.0 / max(0.1, float(args.export_hz)), self.export)
            self.create_service(Trigger, args.save_service, self.on_save_map)

        def on_save_map(
            self, _request: Trigger.Request, response: Trigger.Response
        ) -> Trigger.Response:
            if self.accumulator.origin_x is None:
                response.success = False
                response.message = "地图尚无有效数据，请先移动机器人采集环境"
                return response
            self.export(save_to_disk=True)
            response.success = True
            response.message = str(paths["yaml"])
            self.get_logger().info(f"Map saved manually: {paths['yaml']}")
            return response

        def on_odom(self, msg: Odometry) -> None:
            pose = msg.pose.pose
            arrival_time = time.time()
            parsed_pose = [
                float(pose.position.x),
                float(pose.position.y),
                odometry_yaw(msg, args.odom_yaw_mode),
            ]
            self.latest_pose = (arrival_time, parsed_pose)
            source_stamp = stamp_seconds(msg.header)
            self.odom_history.append((source_stamp, arrival_time, parsed_pose))
            oldest_stamp = source_stamp - max(1.0, float(args.odom_history_sec))
            while self.odom_history and self.odom_history[0][0] < oldest_stamp:
                self.odom_history.popleft()

        def pose_for_cloud(self, msg: PointCloud2) -> tuple[list[float], float] | None:
            if not self.odom_history:
                return None
            cloud_stamp = stamp_seconds(msg.header)
            matched = min(
                self.odom_history, key=lambda item: abs(item[0] - cloud_stamp)
            )
            delta = abs(matched[0] - cloud_stamp)
            self.latest_cloud_odom_delta_sec = delta
            if delta > float(args.max_cloud_odom_delta_sec):
                return None
            return list(matched[2]), delta

        def on_cloud(self, msg: PointCloud2) -> None:
            self.cloud_count += 1
            count_message = UInt64()
            count_message.data = self.cloud_count
            self.cloud_count_publisher.publish(count_message)
            now = time.monotonic()
            should_publish = float(args.cloud_publish_hz) > 0.0 and (
                now - self.last_cloud_publish_at
                >= 1.0 / float(args.cloud_publish_hz)
            )
            should_integrate = now - self.last_cloud_at >= 1.0 / max(
                0.1, float(args.integration_hz)
            )
            if not should_publish and not should_integrate:
                return
            matched = self.pose_for_cloud(msg)
            if matched is None:
                self.sync_skip_count += 1
                return
            pose, _ = matched
            self.last_cloud_frame = str(msg.header.frame_id)
            points = pointcloud_xyz(msg)
            if points.size == 0:
                return
            finite = np.isfinite(points).all(axis=1)
            nonzero = np.linalg.norm(points, axis=1) > 1.0e-4
            valid_points = points[finite & nonzero]
            if valid_points.size == 0:
                return
            if args.cloud_frame_mode == "base":
                # The fused S2 cloud contains dense returns from the chassis
                # itself. Remove points inside the padded physical footprint
                # before publishing to costmaps/collision monitoring or
                # integrating the occupancy map.
                self_points = (
                    (np.abs(valid_points[:, 0]) <= float(args.self_filter_half_x))
                    & (
                        np.abs(valid_points[:, 1])
                        <= float(args.self_filter_half_y)
                    )
                )
                self.self_filtered_point_count += int(self_points.sum())
                valid_points = valid_points[~self_points]
                if valid_points.size == 0:
                    return
                map_points = transform_base_points(valid_points, pose)
            else:
                map_points = valid_points.astype(np.float32, copy=False)
            if should_publish:
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = str(args.map_frame)
                publish_points = map_points
                max_output_points = max(1, int(args.output_cloud_max_points))
                if len(publish_points) > max_output_points:
                    indices = np.linspace(
                        0,
                        len(publish_points) - 1,
                        max_output_points,
                        dtype=np.int64,
                    )
                    publish_points = publish_points[indices]
                cloud = point_cloud2.create_cloud_xyz32(header, publish_points)
                self.cloud_publisher.publish(cloud)
                self.last_cloud_publish_at = now
            if not should_integrate:
                return
            self.last_cloud_at = now
            distance = np.linalg.norm(valid_points[:, :2], axis=1)
            keep = (
                (valid_points[:, 2] >= float(args.min_z))
                & (valid_points[:, 2] <= float(args.max_z))
                & (distance >= float(args.min_range))
                & (distance <= float(args.max_range))
            )
            endpoints = map_points[keep, :2]
            if endpoints.size == 0:
                return
            rays = self.accumulator.integrate(
                (pose[0], pose[1]), endpoints, int(args.max_rays_per_scan)
            )
            self.integrated_scan_count += 1
            self.integrated_ray_count += rays

        def export(self, save_to_disk: bool = False) -> None:
            if self.accumulator.origin_x is None:
                return
            values = self.accumulator.occupancy_values(
                float(args.occupied_log_odds), float(args.free_log_odds)
            )
            raw_image = np.full(values.shape, 205, dtype=np.uint8)
            raw_image[values == 0] = 254
            raw_image[values == 100] = 0
            raw_image = np.flipud(raw_image)
            image, filter_stats = filter_occupied_outliers(raw_image)
            map_yaml = {
                "image": paths["pgm"].name,
                "mode": "trinary",
                "resolution": float(args.resolution),
                "origin": [
                    float(self.accumulator.origin_x),
                    float(self.accumulator.origin_y),
                    0.0,
                ],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }
            should_save = save_to_disk or not bool(args.manual_save_only)
            if should_save:
                atomic_bytes(paths["raw_pgm"], pgm_bytes(raw_image))
                atomic_bytes(paths["pgm"], pgm_bytes(image))
                png_ok, png_data = cv2.imencode(".png", image)
                if png_ok:
                    atomic_bytes(paths["png"], png_data.tobytes())
                atomic_text(paths["yaml"], yaml.safe_dump(map_yaml, sort_keys=False))
            pose_age = None
            robot_xyt = None
            if self.latest_pose is not None:
                pose_age = max(0.0, time.time() - self.latest_pose[0])
                if pose_age <= float(args.odom_stale_sec):
                    robot_xyt = list(self.latest_pose[1])
            payload = {
                "ok": True,
                "source": "s2_registered_pointcloud",
                "cloud_topic": args.cloud_topic,
                "output_cloud_topic": args.output_cloud_topic,
                "odom_topic": args.odom_topic,
                "odom_yaw_mode": args.odom_yaw_mode,
                "frame_id": args.map_frame,
                "cloud_frame_id": self.last_cloud_frame,
                "cloud_frame_mode": args.cloud_frame_mode,
                "width": int(values.shape[1]),
                "height": int(values.shape[0]),
                "resolution_m": float(args.resolution),
                "origin": map_yaml["origin"],
                "height_slice_m": [float(args.min_z), float(args.max_z)],
                "range_m": [float(args.min_range), float(args.max_range)],
                "robot_xyt": robot_xyt,
                "pose_valid": robot_xyt is not None,
                "pose_age_sec": pose_age,
                "cloud_count": self.cloud_count,
                "integrated_scan_count": self.integrated_scan_count,
                "integrated_ray_count": self.integrated_ray_count,
                "self_filtered_point_count": self.self_filtered_point_count,
                "sync_skip_count": self.sync_skip_count,
                "cloud_odom_sync_delta_sec": self.latest_cloud_odom_delta_sec,
                "max_cloud_odom_delta_sec": float(args.max_cloud_odom_delta_sec),
                "occupancy_filter": filter_stats,
                "map_updated_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            }
            if should_save:
                atomic_text(paths["json"], json.dumps(payload, indent=2) + "\n")

            message = OccupancyGrid()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = str(args.map_frame)
            message.info.resolution = float(args.resolution)
            message.info.width = int(values.shape[1])
            message.info.height = int(values.shape[0])
            message.info.origin.position.x = float(self.accumulator.origin_x)
            message.info.origin.position.y = float(self.accumulator.origin_y)
            message.info.origin.orientation.w = 1.0
            message.data = values.reshape(-1).astype(np.int8).tolist()
            self.map_publisher.publish(message)

    rclpy.init()
    node = Mapper()
    print(
        f"[s2-map] cloud={args.cloud_topic} odom={args.odom_topic} output={output_dir}",
        flush=True,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
