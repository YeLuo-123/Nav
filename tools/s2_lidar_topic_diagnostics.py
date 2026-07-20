#!/usr/bin/env python3
"""Discover and measure TX-S2 LiDAR, IMU, odometry, and map ROS2 topics."""

from __future__ import annotations

import argparse
import json
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any


TYPE_CATEGORY = {
    "sensor_msgs/msg/PointCloud2": "pointcloud",
    "livox_ros_driver2/msg/CustomMsg": "pointcloud",
    "sensor_msgs/msg/Imu": "imu",
    "nav_msgs/msg/Odometry": "odom",
    "nav_msgs/msg/OccupancyGrid": "map",
    "sensor_msgs/msg/LaserScan": "scan",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-sec", type=float, default=12.0)
    parser.add_argument("--discovery-sec", type=float, default=3.0)
    parser.add_argument("--output", default="")
    parser.add_argument("--min-pointcloud-hz", type=float, default=5.0)
    parser.add_argument("--min-imu-hz", type=float, default=50.0)
    parser.add_argument("--min-odom-hz", type=float, default=5.0)
    parser.add_argument("--min-map-hz", type=float, default=0.1)
    return parser.parse_args()


def classify_topic(type_names: list[str]) -> str | None:
    for type_name in type_names:
        if type_name in TYPE_CATEGORY:
            return TYPE_CATEGORY[type_name]
    return None


def message_stamp(msg: Any) -> float | None:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return value if value > 0 else None


def message_summary(msg: Any, type_name: str) -> dict[str, Any]:
    header = getattr(msg, "header", None)
    summary: dict[str, Any] = {
        "frame_id": str(getattr(header, "frame_id", "")),
        "source_stamp": message_stamp(msg),
    }
    if type_name == "sensor_msgs/msg/PointCloud2":
        summary.update(
            {
                "width": int(msg.width),
                "height": int(msg.height),
                "point_count": int(msg.width) * int(msg.height),
                "fields": [str(field.name) for field in msg.fields],
            }
        )
    elif type_name == "livox_ros_driver2/msg/CustomMsg":
        summary["point_count"] = int(msg.point_num)
    elif type_name == "nav_msgs/msg/OccupancyGrid":
        summary.update(
            {
                "width": int(msg.info.width),
                "height": int(msg.info.height),
                "resolution_m": float(msg.info.resolution),
            }
        )
    elif type_name == "nav_msgs/msg/Odometry":
        summary["child_frame_id"] = str(msg.child_frame_id)
    return summary


def main() -> None:
    args = parse_args()
    import rclpy
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = rclpy.create_node("s2_lidar_topic_diagnostics")
    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    map_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    discovery_deadline = time.monotonic() + max(0.1, float(args.discovery_sec))
    while time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    discovered = sorted(node.get_topic_names_and_types(), key=lambda item: item[0])
    candidates: list[tuple[str, str, str]] = []
    for topic_name, type_names in discovered:
        category = classify_topic(type_names)
        if category is None:
            continue
        type_name = next(name for name in type_names if name in TYPE_CATEGORY)
        candidates.append((topic_name, type_name, category))

    receive_times: dict[str, list[float]] = defaultdict(list)
    summaries: dict[str, dict[str, Any]] = {}
    subscriptions = []

    for topic_name, type_name, category in candidates:
        try:
            message_type = get_message(type_name)
        except (AttributeError, ModuleNotFoundError, RuntimeError, ValueError) as exc:
            summaries[topic_name] = {"subscription_error": str(exc)}
            continue

        def callback(msg: Any, *, name: str = topic_name, ros_type: str = type_name) -> None:
            receive_times[name].append(time.monotonic())
            summaries[name] = message_summary(msg, ros_type)

        subscriptions.append(
            node.create_subscription(
                message_type,
                topic_name,
                callback,
                map_qos if category == "map" else sensor_qos,
            )
        )

    started = time.monotonic()
    deadline = started + max(0.1, float(args.duration_sec))
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    elapsed = max(1e-6, time.monotonic() - started)

    minimum_rates = {
        "pointcloud": float(args.min_pointcloud_hz),
        "imu": float(args.min_imu_hz),
        "odom": float(args.min_odom_hz),
        "map": float(args.min_map_hz),
        "scan": float(args.min_pointcloud_hz),
    }
    topic_reports = []
    category_ok: dict[str, bool] = defaultdict(bool)
    for topic_name, type_name, category in candidates:
        times = receive_times.get(topic_name, [])
        if len(times) >= 2 and times[-1] > times[0]:
            rate = float(len(times) - 1) / float(times[-1] - times[0])
        else:
            rate = float(len(times)) / elapsed
        minimum = minimum_rates[category]
        ok = len(times) > 0 and (category == "map" or rate >= minimum)
        category_ok[category] = category_ok[category] or ok
        topic_reports.append(
            {
                "topic": topic_name,
                "type": type_name,
                "category": category,
                "message_count": len(times),
                "measured_hz": rate,
                "minimum_hz": minimum,
                "ok": ok,
                **summaries.get(topic_name, {}),
            }
        )

    report = {
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "duration_sec": elapsed,
        "discovered_topic_count": len(discovered),
        "candidate_topic_count": len(candidates),
        "category_ok": dict(category_ok),
        "mapping_inputs": {
            "native_map_ready": bool(category_ok.get("map") and category_ok.get("odom")),
            "registered_cloud_ready": bool(category_ok.get("pointcloud") and category_ok.get("odom")),
            "raw_lio_ready": bool(category_ok.get("pointcloud") and category_ok.get("imu")),
        },
        "topics": topic_reports,
        "all_discovered_topics": [
            {"topic": name, "types": types} for name, types in discovered
        ],
    }
    text = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    print(text, end="")
    if args.output:
        output = Path(args.output).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text, encoding="utf-8")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
