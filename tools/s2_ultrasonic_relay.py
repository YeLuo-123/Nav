#!/usr/bin/env python3
"""Convert the S2 vendor ultrasonic endpoints into a Nav2 PointCloud2 source."""

import math

import rclpy
from driver.msg import RadarPoints
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class UltrasonicRelay(Node):
    def __init__(self) -> None:
        super().__init__("s2_ultrasonic_relay")
        self.declare_parameter("input_topic", "/driver/radar/Points")
        self.declare_parameter("output_topic", "/s2_ultrasonic/points")
        self.declare_parameter("min_range", 0.03)
        self.declare_parameter("max_range", 3.5)
        self.declare_parameter("obstacle_radius", 0.12)
        self.declare_parameter("obstacle_height", 0.20)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.radius = float(self.get_parameter("obstacle_radius").value)
        self.height = float(self.get_parameter("obstacle_height").value)
        output = str(self.get_parameter("output_topic").value)
        input_topic = str(self.get_parameter("input_topic").value)
        self.publisher = self.create_publisher(PointCloud2, output, 10)
        self.create_subscription(RadarPoints, input_topic, self.on_points, 10)
        self.get_logger().info(f"Ultrasonic relay: {input_topic} -> {output}")

    def on_points(self, msg: RadarPoints) -> None:
        obstacles = []
        for point in msg.points:
            distance = math.hypot(point.x, point.y)
            if not (math.isfinite(point.x) and math.isfinite(point.y)):
                continue
            if distance < self.min_range or distance > self.max_range:
                continue
            # Inflate sparse sonar endpoints slightly. Nav2 adds its normal
            # footprint inflation on top of this conservative glass target.
            obstacles.append((point.x, point.y, self.height))
            for index in range(8):
                angle = index * math.pi / 4.0
                obstacles.append(
                    (
                        point.x + self.radius * math.cos(angle),
                        point.y + self.radius * math.sin(angle),
                        self.height,
                    )
                )

        header = msg.header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        self.publisher.publish(point_cloud2.create_cloud(header, fields, obstacles))


def main() -> None:
    rclpy.init()
    node = UltrasonicRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
