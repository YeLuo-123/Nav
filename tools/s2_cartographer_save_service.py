#!/usr/bin/env python3
"""Expose Cartographer state saving through the keyboard mapper Trigger service."""

import argparse
from pathlib import Path
import threading

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
    parser.add_argument("--autosave-sec", type=float, default=0.0)
    args = parser.parse_args()
    output = str(Path(args.output).expanduser().resolve())
    rclpy.init()

    class SaveService(Node):
        def __init__(self) -> None:
            super().__init__("s2_cartographer_save_service")
            group = ReentrantCallbackGroup()
            self.save_lock = threading.Lock()
            self.client = self.create_client(
                WriteState, "/write_state", callback_group=group
            )
            self.create_service(
                Trigger, args.service, self.save, callback_group=group
            )
            if float(args.autosave_sec) > 0.0:
                self.create_timer(
                    float(args.autosave_sec), self.autosave, callback_group=group
                )

        def write_state(self) -> tuple[bool, str]:
            if not self.save_lock.acquire(blocking=False):
                return False, "另一次保存正在进行"
            try:
                if not self.client.wait_for_service(timeout_sec=2.0):
                    return False, "Cartographer write_state 服务不可用"
                request = WriteState.Request()
                request.filename = output
                request.include_unfinished_submaps = True
                result = self.client.call(request)
                success = result is not None and result.status.code == 0
                message = output if success else str(
                    result.status.message if result else "保存超时"
                )
                return success, message
            finally:
                self.save_lock.release()

        def autosave(self) -> None:
            success, message = self.write_state()
            if success:
                self.get_logger().info(f"Cartographer checkpoint saved: {message}")
            elif message != "另一次保存正在进行":
                self.get_logger().warning(f"Checkpoint failed: {message}")

        def save(self, _request, response):
            response.success, response.message = self.write_state()
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
