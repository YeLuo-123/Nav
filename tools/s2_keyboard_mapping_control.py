#!/usr/bin/env python3
"""Interactive S2 teleoperation with one-key manual map saving."""

from __future__ import annotations

import argparse
import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Trigger


HELP = """
S2 键盘建图
  w / s       前进 / 后退
  a / d       左转 / 右转
  q / e       左移 / 右移
  空格或 x    立即停止
  + / =       提高移动和旋转速度
  -           降低移动和旋转速度
  p           手动保存当前地图
  h           再次显示帮助
  Ctrl+C      停止并退出

按住移动键可持续运动；松开后安全看门狗会自动停车。
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cmd-topic", default="/s2_nav2/cmd_vel")
    parser.add_argument("--save-service", default="/s2_lidar_slam/save_map")
    parser.add_argument("--linear", type=float, default=0.06)
    parser.add_argument("--strafe", type=float, default=0.06)
    parser.add_argument("--angular", type=float, default=0.20)
    parser.add_argument("--speed-scale-step", type=float, default=0.25)
    parser.add_argument("--min-speed-scale", type=float, default=0.50)
    parser.add_argument("--max-speed-scale", type=float, default=2.00)
    return parser.parse_args()


class KeyboardMapping(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("s2_keyboard_mapping_control")
        self.args = args
        self.publisher = self.create_publisher(Twist, args.cmd_topic, 10)
        self.save_client = self.create_client(Trigger, args.save_service)
        self.speed_scale = 1.0

    def stop(self) -> None:
        self.publisher.publish(Twist())

    def command(self, key: str) -> None:
        msg = Twist()
        if key == "w":
            msg.linear.x = self.args.linear * self.speed_scale
        elif key == "s":
            msg.linear.x = -self.args.linear * self.speed_scale
        elif key == "a":
            msg.angular.z = self.args.angular * self.speed_scale
        elif key == "d":
            msg.angular.z = -self.args.angular * self.speed_scale
        elif key == "q":
            msg.linear.y = self.args.strafe * self.speed_scale
        elif key == "e":
            msg.linear.y = -self.args.strafe * self.speed_scale
        self.publisher.publish(msg)

    def adjust_speed(self, increase: bool) -> None:
        delta = self.args.speed_scale_step * (1.0 if increase else -1.0)
        self.speed_scale = max(
            self.args.min_speed_scale,
            min(self.args.max_speed_scale, self.speed_scale + delta),
        )
        self.stop()
        print(
            "\n[速度] "
            f"倍率={self.speed_scale:.2f} "
            f"前后={self.args.linear * self.speed_scale:.3f} m/s "
            f"横移={self.args.strafe * self.speed_scale:.3f} m/s "
            f"旋转={self.args.angular * self.speed_scale:.3f} rad/s",
            flush=True,
        )

    def save(self) -> None:
        self.stop()
        if not self.save_client.wait_for_service(timeout_sec=2.0):
            print("\n[保存失败] 建图保存服务不可用", flush=True)
            return
        future = self.save_client.call_async(Trigger.Request())
        future.add_done_callback(self.on_saved)

    @staticmethod
    def on_saved(future) -> None:
        try:
            result = future.result()
            label = "保存成功" if result.success else "保存失败"
            print(f"\n[{label}] {result.message}", flush=True)
        except Exception as exc:
            print(f"\n[保存失败] {exc}", flush=True)


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = KeyboardMapping(args)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP, flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                continue
            key = sys.stdin.read(1).lower()
            if key == "\x03":
                break
            if key in "wasdqe":
                node.command(key)
            elif key in (" ", "x"):
                node.stop()
            elif key in ("+", "="):
                node.adjust_speed(increase=True)
            elif key == "-":
                node.adjust_speed(increase=False)
            elif key == "p":
                node.save()
            elif key == "h":
                print(HELP, flush=True)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        print("\n[s2-keyboard-map] 已停车并退出", flush=True)


if __name__ == "__main__":
    main()
