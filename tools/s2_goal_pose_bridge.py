#!/usr/bin/env python3
"""Forward RViz /goal_pose messages to the Nav2 NavigateToPose action."""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__("s2_goal_pose_bridge")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.get_logger().info(
            "Ready: RViz 2D Goal Pose (/goal_pose) -> NavigateToPose"
        )

    def on_goal(self, pose: PoseStamped) -> None:
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("NavigateToPose action server is unavailable")
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.on_response)

    def on_response(self, future) -> None:
        handle = future.result()
        if handle.accepted:
            self.get_logger().info("RViz navigation goal accepted")
        else:
            self.get_logger().error("RViz navigation goal rejected")


def main() -> None:
    rclpy.init()
    node = GoalPoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
