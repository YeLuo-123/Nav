#!/usr/bin/env python3
"""Publish host-timestamped S2 odometry, TF, joints, and RViz helpers."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from g1_occupancy_grid_exporter import odometry_yaw


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--odom-topic", default="/controller/odom")
    parser.add_argument("--output-odom-topic", default="/s2_lidar_slam/odom")
    parser.add_argument("--marker-topic", default="/s2_lidar_slam/robot_markers")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument(
        "--footprint-topic", default="/s2_lidar_slam/configured_footprint"
    )
    parser.add_argument("--parent-frame", default="odom")
    parser.add_argument("--child-frame", default="base_link")
    parser.add_argument(
        "--odom-yaw-mode",
        choices=("auto", "quaternion", "orientation_w", "orientation_z"),
        default="auto",
    )
    parser.add_argument("--body-length-m", type=float, default=0.67)
    parser.add_argument("--body-width-m", type=float, default=0.55)
    parser.add_argument("--body-height-m", type=float, default=0.39)
    parser.add_argument("--footprint-length-m", type=float, default=0.86)
    parser.add_argument("--footprint-width-m", type=float, default=0.66)
    parser.add_argument("--footprint-padding-m", type=float, default=0.03)
    parser.add_argument(
        "--output-hz",
        type=float,
        default=20.0,
        help="Maximum corrected odometry/TF publication rate.",
    )
    return parser.parse_args()


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    return math.sin(float(yaw) * 0.5), math.cos(float(yaw) * 0.5)


def footprint_vertices(
    length: float, width: float, padding: float
) -> list[tuple[float, float]]:
    half_length = max(0.0, float(length) * 0.5 + float(padding))
    half_width = max(0.0, float(width) * 0.5 + float(padding))
    return [
        (half_length, half_width),
        (half_length, -half_width),
        (-half_length, -half_width),
        (-half_length, half_width),
    ]


def main() -> None:
    args = parse_args()

    import rclpy
    from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import JointState
    from tf2_ros import TransformBroadcaster
    from visualization_msgs.msg import Marker, MarkerArray

    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    output_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    class Bridge(Node):
        def __init__(self) -> None:
            super().__init__("s2_lidar_rviz_bridge")
            self.tf_broadcaster = TransformBroadcaster(self)
            self.odom_publisher = self.create_publisher(
                Odometry, args.output_odom_topic, output_qos
            )
            self.marker_publisher = self.create_publisher(
                MarkerArray, args.marker_topic, output_qos
            )
            self.joint_state_publisher = self.create_publisher(
                JointState, args.joint_state_topic, output_qos
            )
            self.footprint_publisher = self.create_publisher(
                PolygonStamped, args.footprint_topic, output_qos
            )
            self.create_subscription(Odometry, args.odom_topic, self.on_odom, sensor_qos)
            self.create_timer(0.1, self.publish_robot_state)
            self.last_odom_publish = 0.0

        @staticmethod
        def copy_covariance(source: Any) -> list[float]:
            return [float(value) for value in source]

        def publish_robot_state(self) -> None:
            stamp = self.get_clock().now().to_msg()

            joints = JointState()
            joints.header.stamp = stamp
            joints.name = ["ZQL", "YQL", "ZHL", "YHL"]
            joints.position = [0.0, 0.0, 0.0, 0.0]
            self.joint_state_publisher.publish(joints)

            footprint = PolygonStamped()
            footprint.header.stamp = stamp
            footprint.header.frame_id = args.child_frame
            footprint.polygon.points = [
                Point32(x=float(x), y=float(y), z=0.02)
                for x, y in footprint_vertices(
                    args.footprint_length_m,
                    args.footprint_width_m,
                    args.footprint_padding_m,
                )
            ]
            self.footprint_publisher.publish(footprint)

        def on_odom(self, source: Odometry) -> None:
            now = time.monotonic()
            if now - self.last_odom_publish < 1.0 / max(1.0, args.output_hz):
                return
            self.last_odom_publish = now
            yaw = odometry_yaw(source.pose.pose.orientation, args.odom_yaw_mode)
            quaternion_z, quaternion_w = quaternion_from_yaw(yaw)
            stamp = self.get_clock().now().to_msg()
            position = source.pose.pose.position

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = args.parent_frame
            odom.child_frame_id = args.child_frame
            odom.pose.pose.position.x = float(position.x)
            odom.pose.pose.position.y = float(position.y)
            odom.pose.pose.position.z = float(position.z)
            odom.pose.pose.orientation.z = quaternion_z
            odom.pose.pose.orientation.w = quaternion_w
            odom.pose.covariance = self.copy_covariance(source.pose.covariance)
            odom.twist.twist = source.twist.twist
            odom.twist.covariance = self.copy_covariance(source.twist.covariance)
            self.odom_publisher.publish(odom)

            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = args.child_frame
            transform.transform.translation.x = float(position.x)
            transform.transform.translation.y = float(position.y)
            transform.transform.translation.z = float(position.z)
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

            body = Marker()
            body.header = odom.header
            body.ns = "s2_body"
            body.id = 0
            body.type = Marker.CUBE
            body.action = Marker.ADD
            body.pose = odom.pose.pose
            body.pose.position.z += float(args.body_height_m) * 0.5
            body.scale.x = float(args.body_length_m)
            body.scale.y = float(args.body_width_m)
            body.scale.z = float(args.body_height_m)
            body.color.r = 0.10
            body.color.g = 0.75
            body.color.b = 0.95
            body.color.a = 0.65

            heading = Marker()
            heading.header = odom.header
            heading.ns = "s2_heading"
            heading.id = 1
            heading.type = Marker.ARROW
            heading.action = Marker.ADD
            heading.pose = odom.pose.pose
            heading.pose.position.z += float(args.body_height_m) + 0.06
            heading.scale.x = 0.85
            heading.scale.y = 0.12
            heading.scale.z = 0.12
            heading.color.r = 1.0
            heading.color.g = 0.25
            heading.color.b = 0.15
            heading.color.a = 0.95
            self.marker_publisher.publish(MarkerArray(markers=[body, heading]))

    rclpy.init()
    node = Bridge()
    print(
        f"[s2-rviz] odom={args.odom_topic} corrected={args.output_odom_topic} "
        f"tf={args.parent_frame}->{args.child_frame}",
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
