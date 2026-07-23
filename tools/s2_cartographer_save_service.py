#!/usr/bin/env python3
"""Expose Cartographer state saving through the keyboard mapper Trigger service."""

import argparse
from pathlib import Path

import rclpy
from cartographer_ros_msgs.srv import WriteState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--service", default="/s2_lidar_slam/save_map")
    args = parser.parse_args()
    output = str(Path(args.output).expanduser().resolve())
    rclpy.init()

    class SaveService(Node):
        def __init__(self) -> None:
            super().__init__("s2_cartographer_save_service")
            group = ReentrantCallbackGroup()
            self.client = self.create_client(
                WriteState, "/write_state", callback_group=group
            )
            self.create_service(
                Trigger, args.service, self.save, callback_group=group
            )

        def save(self, _request, response):
            if not self.client.wait_for_service(timeout_sec=2.0):
                response.success = False
                response.message = "Cartographer write_state 服务不可用"
                return response
            request = WriteState.Request()
            request.filename = output
            request.include_unfinished_submaps = True
            result = self.client.call(request)
            response.success = result is not None and result.status.code == 0
            response.message = output if response.success else str(result.status.message if result else "保存超时")
            return response

    node = SaveService()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
