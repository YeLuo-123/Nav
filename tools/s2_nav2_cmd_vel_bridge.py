#!/usr/bin/env python3
"""Bridge Nav2 Twist commands to the S2 controller safely."""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import socket
import struct
import time
import urllib.request
import urllib.error
from urllib.parse import urlparse


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", default="/s2_nav2/cmd_vel")
    parser.add_argument("--preview-topic", default="/s2_nav2/cmd_vel_preview")
    parser.add_argument(
        "--transport",
        choices=("vendor_websocket", "ros_topic"),
        default="vendor_websocket",
    )
    parser.add_argument("--output-topic", default="/move/AutoMoveCmd")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--controller-host", default="192.168.127.10")
    parser.add_argument("--controller-http-port", type=int, default=8080)
    parser.add_argument("--controller-ws-port", type=int, default=6001)
    parser.add_argument("--robot-id", type=int, default=1)
    parser.add_argument("--controller-timeout-sec", type=float, default=2.0)
    parser.add_argument("--skip-set-manual-mode", action="store_true")
    parser.add_argument("--skip-exit-parking", action="store_true")
    parser.add_argument("--skip-motion-burst-mode-refresh", action="store_true")
    parser.add_argument("--leave-unparked-on-exit", action="store_true")
    parser.add_argument("--restore-auto-mode-on-exit", action="store_true")
    parser.add_argument("--max-linear-x", type=float, default=0.25)
    parser.add_argument("--max-linear-y", type=float, default=0.20)
    parser.add_argument("--max-angular-z", type=float, default=0.50)
    parser.add_argument("--watchdog-sec", type=float, default=0.35)
    parser.add_argument(
        "--safety-cloud-topic",
        default="/driver/lidar/point_cloud/Data",
    )
    parser.add_argument("--safety-heartbeat-topic", default="")
    # The S2 fused-cloud stream is bursty in real hardware and has measured
    # gaps up to about 2.0 s.  Keep a bounded margin above that; longer loss
    # still forces zero velocity.
    parser.add_argument("--safety-cloud-timeout-sec", type=float, default=2.5)
    parser.add_argument(
        "--disable-safety-cloud-check",
        action="store_true",
        help="Do not gate motion on the duplicate lidar heartbeat subscription.",
    )
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--allow-shared-output", action="store_true")
    return parser.parse_args()


def clamp(value: float, limit: float) -> float:
    if not math.isfinite(value):
        return 0.0
    return max(-abs(limit), min(abs(limit), float(value)))


def post_json(url: str, payload: dict, timeout_sec: float) -> dict:
    request = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=timeout_sec) as response:
        body = response.read().decode("utf-8")
    result = json.loads(body)
    if int(result.get("ErrorCode", -1)) != 0:
        raise RuntimeError(
            f"{result.get('Interface', url)} failed: "
            f"{result.get('ErrorMessage', result)}"
        )
    return result


class VendorManualWebSocket:
    """Minimal RFC 6455 client for the S2 vendor manual-control endpoint."""

    def __init__(self, url: str, timeout_sec: float) -> None:
        parsed = urlparse(url)
        if parsed.scheme != "ws" or not parsed.hostname:
            raise ValueError(f"unsupported WebSocket URL: {url}")
        self.url = url
        self.host = parsed.hostname
        self.port = parsed.port or 80
        self.path = parsed.path or "/"
        if parsed.query:
            self.path += f"?{parsed.query}"
        self.timeout_sec = max(0.2, float(timeout_sec))
        self.socket: socket.socket | None = None

    def connect(self) -> None:
        self.close()
        sock = socket.create_connection(
            (self.host, self.port), timeout=self.timeout_sec
        )
        sock.settimeout(self.timeout_sec)
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        request = (
            f"GET {self.path} HTTP/1.1\r\n"
            f"Host: {self.host}:{self.port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "\r\n"
        )
        sock.sendall(request.encode("ascii"))
        response = bytearray()
        while b"\r\n\r\n" not in response:
            chunk = sock.recv(4096)
            if not chunk:
                raise ConnectionError("WebSocket handshake closed by controller")
            response.extend(chunk)
            if len(response) > 65536:
                raise ConnectionError("WebSocket handshake response is too large")
        header = bytes(response).split(b"\r\n\r\n", 1)[0].decode(
            "iso-8859-1"
        )
        status = header.splitlines()[0] if header else ""
        if " 101 " not in status:
            sock.close()
            raise ConnectionError(f"WebSocket handshake failed: {status}")
        expected = base64.b64encode(
            hashlib.sha1(
                (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")
            ).digest()
        ).decode("ascii")
        headers = {}
        for line in header.splitlines()[1:]:
            if ":" in line:
                name, value = line.split(":", 1)
                headers[name.strip().lower()] = value.strip()
        if headers.get("sec-websocket-accept") != expected:
            sock.close()
            raise ConnectionError("invalid Sec-WebSocket-Accept response")
        self.socket = sock

    def send_json(self, payload: dict) -> None:
        if self.socket is None:
            raise ConnectionError("WebSocket is not connected")
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        first = bytearray([0x81])
        length = len(data)
        if length < 126:
            first.append(0x80 | length)
        elif length <= 0xFFFF:
            first.append(0x80 | 126)
            first.extend(struct.pack("!H", length))
        else:
            first.append(0x80 | 127)
            first.extend(struct.pack("!Q", length))
        mask = os.urandom(4)
        masked = bytes(value ^ mask[index % 4] for index, value in enumerate(data))
        self.socket.sendall(bytes(first) + mask + masked)

    def close(self) -> None:
        if self.socket is None:
            return
        try:
            self.socket.close()
        finally:
            self.socket = None


def main() -> None:
    args = parse_args()

    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import UInt64

    class Bridge(Node):
        def __init__(self) -> None:
            super().__init__("s2_nav2_cmd_vel_bridge")
            self.last_command_monotonic = time.monotonic()
            self.last_output_command = None
            self.sent_nonzero = False
            self.motion_active = False
            self.fatal_error = False
            self.parking_released = False
            self.vendor_socket = None
            self.last_safety_cloud_monotonic: float | None = None
            self.last_sensor_warning_monotonic = 0.0
            self.last_vendor_keepalive_monotonic = 0.0
            self.preview_publisher = self.create_publisher(
                TwistStamped, args.preview_topic, 10
            )
            self.output_publisher = None
            if args.enable_motion:
                if args.transport == "vendor_websocket":
                    self.enable_vendor_websocket()
                else:
                    self.enable_ros_topic()
                    self.output_publisher = self.create_publisher(
                        TwistStamped, args.output_topic, 10
                    )
                    self.motion_active = True
                    self.create_timer(1.0, self.check_output_ownership)
            self.create_subscription(Twist, args.input_topic, self.on_command, 10)
            sensor_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            if args.safety_heartbeat_topic:
                self.create_subscription(
                    UInt64,
                    args.safety_heartbeat_topic,
                    self.on_safety_cloud,
                    sensor_qos,
                )
            else:
                self.create_subscription(
                    PointCloud2,
                    args.safety_cloud_topic,
                    self.on_safety_cloud,
                    sensor_qos,
                )
            # The vendor `server` node publishes zero manual commands at about
            # 20 Hz.  Refresh the latest validated Nav2 command at 100 Hz so it
            # is not continuously overwritten, while the watchdog below still
            # forces zero if Nav2 or lidar updates stop.
            self.create_timer(0.01, self.on_watchdog)

        def on_safety_cloud(self, _msg: PointCloud2 | UInt64) -> None:
            self.last_safety_cloud_monotonic = time.monotonic()

        def safety_cloud_fresh(self) -> bool:
            if args.disable_safety_cloud_check:
                return True
            if self.last_safety_cloud_monotonic is None:
                return False
            return (
                time.monotonic() - self.last_safety_cloud_monotonic
                <= max(0.05, float(args.safety_cloud_timeout_sec))
            )

        def warn_sensor_stale(self) -> None:
            now = time.monotonic()
            if now - self.last_sensor_warning_monotonic < 1.0:
                return
            self.last_sensor_warning_monotonic = now
            self.get_logger().error(
                "Motion blocked: live safety lidar heartbeat is missing or stale "
                f"({args.safety_heartbeat_topic or args.safety_cloud_topic}, timeout "
                f"{args.safety_cloud_timeout_sec:.2f}s)"
            )

        @property
        def controller_base_url(self) -> str:
            return (
                f"http://{args.controller_host}:{args.controller_http_port}"
            )

        def set_controller_mode(self, manual: bool) -> None:
            endpoint = "SetManualMode" if manual else "SetAutoMode"
            post_json(
                f"{self.controller_base_url}/api/AMR/{endpoint}",
                {"id": args.robot_id},
                args.controller_timeout_sec,
            )

        def set_parking(self, enabled: bool) -> None:
            endpoint = "StartParking" if enabled else "StopParking"
            try:
                post_json(
                    f"{self.controller_base_url}/api/AMR/{endpoint}",
                    {},
                    args.controller_timeout_sec,
                )
            except urllib.error.HTTPError as exc:
                if not enabled and exc.code == 409:
                    return
                raise
            except RuntimeError as exc:
                if not enabled and "conflict" in str(exc).lower():
                    return
                raise

        def refresh_controller_mode(self) -> None:
            if not self.motion_active:
                return
            try:
                self.set_controller_mode(manual=True)
                self.set_parking(enabled=False)
                self.parking_released = True
            except Exception as exc:
                self.get_logger().warning(
                    f"Controller manual/unpark refresh failed: {exc}"
                )

        def enable_vendor_websocket(self) -> None:
            try:
                if not args.skip_set_manual_mode:
                    self.set_controller_mode(manual=True)
                if not args.skip_exit_parking:
                    self.set_parking(enabled=False)
                    self.parking_released = True
                url = (
                    f"ws://{args.controller_host}:{args.controller_ws_port}"
                    "/api/AMR/ManualMove"
                )
                self.vendor_socket = VendorManualWebSocket(
                    url, args.controller_timeout_sec
                )
                self.vendor_socket.connect()
            except Exception:
                if self.parking_released:
                    try:
                        self.set_parking(enabled=True)
                    finally:
                        self.parking_released = False
                raise
            self.motion_active = True
            self.get_logger().warn(
                "REAL MOTION ENABLED through the vendor soft-manual WebSocket. "
                "Keep the emergency stop within reach."
            )

        def enable_ros_topic(self) -> None:
            try:
                if not args.skip_set_manual_mode:
                    self.set_controller_mode(manual=True)
                if not args.skip_exit_parking:
                    self.set_parking(enabled=False)
                    self.parking_released = True
            except Exception:
                if self.parking_released:
                    try:
                        self.set_parking(enabled=True)
                    finally:
                        self.parking_released = False
                raise
            self.get_logger().warn(
                "REAL MOTION ENABLED through the vendor ManualMoveCmd ROS topic. "
                "Keep the emergency stop within reach."
            )

        def check_output_ownership(self) -> None:
            if not self.motion_active or args.allow_shared_output:
                return
            others = {
                (info.node_namespace, info.node_name)
                for info in self.get_publishers_info_by_topic(args.output_topic)
                if info.node_name != self.get_name()
            }
            if others:
                self.motion_active = False
                names = ", ".join(f"{ns}/{name}" for ns, name in sorted(others))
                self.get_logger().error(
                    "Motion output disabled because another publisher already owns "
                    f"{args.output_topic}: {names}. Stop that publisher or explicitly "
                    "use --allow-shared-output after confirming command arbitration."
                )

        def make_stamped(self, source: Twist | None = None) -> TwistStamped:
            result = TwistStamped()
            result.header.stamp = self.get_clock().now().to_msg()
            result.header.frame_id = args.frame_id
            if source is not None:
                result.twist.linear.x = clamp(source.linear.x, args.max_linear_x)
                result.twist.linear.y = clamp(source.linear.y, args.max_linear_y)
                result.twist.angular.z = clamp(source.angular.z, args.max_angular_z)
            return result

        @staticmethod
        def is_nonzero(command: TwistStamped) -> bool:
            return any(
                abs(value) > 1.0e-5
                for value in (
                    command.twist.linear.x,
                    command.twist.linear.y,
                    command.twist.angular.z,
                )
            )

        def publish_output(self, command: TwistStamped) -> None:
            if not self.motion_active:
                return
            if self.vendor_socket is not None:
                self.vendor_socket.send_json(
                    {
                        "id": args.robot_id,
                        "x": command.twist.linear.x,
                        "y": command.twist.linear.y,
                        "w": command.twist.angular.z,
                    }
                )
                self.last_vendor_keepalive_monotonic = time.monotonic()
            elif self.output_publisher is not None:
                self.output_publisher.publish(command)

        def fail_motion(self, exc: Exception) -> None:
            if self.fatal_error:
                return
            self.fatal_error = True
            self.motion_active = False
            if self.vendor_socket is not None:
                self.vendor_socket.close()
            self.get_logger().fatal(
                f"Motion transport failed and has been disabled: {exc}"
            )
            if rclpy.ok():
                rclpy.shutdown()

        def on_command(self, source: Twist) -> None:
            command = self.make_stamped(source)
            requested_nonzero = self.is_nonzero(command)
            # The S2 may pause its LiDAR stream while parked.  A first key
            # press must therefore wake manual mode before the fresh-cloud
            # interlock can pass.  Keep this wake-up press at zero velocity;
            # the operator presses the direction again after scans resume.
            if (
                requested_nonzero
                and not self.safety_cloud_fresh()
                and args.enable_motion
                and args.transport == "ros_topic"
                and not args.skip_motion_burst_mode_refresh
            ):
                self.refresh_controller_mode()
            if self.is_nonzero(command) and not self.safety_cloud_fresh():
                command = self.make_stamped()
                self.warn_sensor_stale()
            self.last_command_monotonic = time.monotonic()
            was_nonzero = self.sent_nonzero
            self.sent_nonzero = self.is_nonzero(command)
            if (
                self.sent_nonzero
                and not was_nonzero
                and args.enable_motion
                and args.transport == "ros_topic"
                and not args.skip_motion_burst_mode_refresh
            ):
                # SetManualMode resets the controller's current velocity.  Do
                # this once at the start of a motion burst, then publish the
                # command immediately; repeating it periodically prevents the
                # chassis from ever beginning to move.
                self.refresh_controller_mode()
            self.last_output_command = command
            self.preview_publisher.publish(command)
            try:
                self.publish_output(command)
            except (ConnectionError, OSError) as exc:
                self.fail_motion(exc)

        def on_watchdog(self) -> None:
            if self.sent_nonzero and not self.safety_cloud_fresh():
                command = self.make_stamped()
                self.preview_publisher.publish(command)
                try:
                    self.publish_output(command)
                except (ConnectionError, OSError) as exc:
                    self.fail_motion(exc)
                self.sent_nonzero = False
                self.warn_sensor_stale()
                return
            if not self.sent_nonzero:
                # The controller closes an idle manual WebSocket. A periodic
                # zero command is both a safe keepalive and ensures the next
                # keyboard command is not lost to a stale connection.
                if (
                    self.vendor_socket is not None
                    and time.monotonic() - self.last_vendor_keepalive_monotonic
                    >= 0.20
                ):
                    try:
                        self.publish_output(self.make_stamped())
                    except (ConnectionError, OSError) as exc:
                        self.fail_motion(exc)
                return
            if time.monotonic() - self.last_command_monotonic <= args.watchdog_sec:
                if self.last_output_command is not None:
                    try:
                        # The S2 controller rejects repeated TwistStamped
                        # samples carrying an unchanged timestamp.
                        self.last_output_command.header.stamp = (
                            self.get_clock().now().to_msg()
                        )
                        self.publish_output(self.last_output_command)
                    except (ConnectionError, OSError) as exc:
                        self.fail_motion(exc)
                return
            command = self.make_stamped()
            self.preview_publisher.publish(command)
            try:
                self.publish_output(command)
            except (ConnectionError, OSError) as exc:
                self.fail_motion(exc)
            self.sent_nonzero = False

        def stop_robot(self) -> None:
            if self.motion_active:
                command = self.make_stamped()
                for _ in range(3):
                    try:
                        self.publish_output(command)
                    except (ConnectionError, OSError):
                        break
                    time.sleep(0.05)
            if self.vendor_socket is not None:
                self.vendor_socket.close()
            if self.parking_released and not args.leave_unparked_on_exit:
                try:
                    self.set_parking(enabled=True)
                    self.parking_released = False
                except Exception as exc:
                    self.get_logger().error(f"Failed to enter parking mode: {exc}")
            if args.restore_auto_mode_on_exit:
                try:
                    self.set_controller_mode(manual=False)
                except Exception as exc:
                    self.get_logger().error(
                        f"Failed to restore soft-auto mode: {exc}"
                    )

    rclpy.init()
    try:
        node = Bridge()
    except Exception as exc:
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(f"[s2-nav2-cmd] failed to enable controller: {exc}")
    mode = "ENABLED" if args.enable_motion else "PREVIEW ONLY"
    print(
        f"[s2-nav2-cmd] mode={mode} transport={args.transport} "
        f"input={args.input_topic} "
        f"preview={args.preview_topic} output={args.output_topic}",
        flush=True,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_robot()
        fatal_error = node.fatal_error
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if fatal_error:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
