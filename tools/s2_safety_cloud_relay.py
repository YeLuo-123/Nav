#!/usr/bin/env python3
"""Publish a lightweight, self-filtered S2 safety cloud and lidar heartbeat."""

from __future__ import annotations

import argparse

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", default="/driver/lidar/point_cloud/Data")
    parser.add_argument("--output-topic", default="/s2_lidar_slam/point_cloud")
    parser.add_argument("--heartbeat-topic", default="/s2_lidar_slam/cloud_count")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--publish-hz", type=float, default=3.0)
    parser.add_argument("--max-points", type=int, default=3000)
    parser.add_argument("--self-filter-half-x", type=float, default=0.48)
    parser.add_argument("--self-filter-half-y", type=float, default=0.38)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header, UInt64

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    class SafetyCloudRelay(Node):
        def __init__(self) -> None:
            super().__init__("s2_safety_cloud_relay")
            self.count = 0
            self.last_publish_ns = 0
            self.cloud_publisher = self.create_publisher(
                PointCloud2, args.output_topic, qos
            )
            self.heartbeat_publisher = self.create_publisher(
                UInt64, args.heartbeat_topic, qos
            )
            self.create_subscription(PointCloud2, args.input_topic, self.on_cloud, qos)

        def on_cloud(self, message: PointCloud2) -> None:
            self.count += 1
            heartbeat = UInt64()
            heartbeat.data = self.count
            self.heartbeat_publisher.publish(heartbeat)

            now_ns = self.get_clock().now().nanoseconds
            interval_ns = int(1.0e9 / max(0.1, float(args.publish_hz)))
            if now_ns - self.last_publish_ns < interval_ns:
                return

            values = np.asarray(
                point_cloud2.read_points(
                    message, field_names=["x", "y", "z"], skip_nans=True
                )
            )
            if values.dtype.names:
                points = np.column_stack(
                    [values[name] for name in ("x", "y", "z")]
                ).astype(np.float32)
            else:
                points = values.reshape(-1, 3).astype(np.float32, copy=False)
            if points.size == 0:
                return

            finite = np.isfinite(points).all(axis=1)
            nonzero = np.linalg.norm(points, axis=1) > 1.0e-4
            self_points = (
                (np.abs(points[:, 0]) <= float(args.self_filter_half_x))
                & (np.abs(points[:, 1]) <= float(args.self_filter_half_y))
            )
            points = points[finite & nonzero & ~self_points]
            if points.size == 0:
                return

            maximum = max(1, int(args.max_points))
            if len(points) > maximum:
                indices = np.linspace(0, len(points) - 1, maximum, dtype=np.int64)
                points = points[indices]

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = str(args.frame_id)
            self.cloud_publisher.publish(point_cloud2.create_cloud_xyz32(header, points))
            self.last_publish_ns = now_ns

    rclpy.init()
    node = SafetyCloudRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
