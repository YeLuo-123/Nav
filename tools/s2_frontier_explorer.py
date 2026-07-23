#!/usr/bin/env python3
"""Autonomous frontier exploration for the S2 live occupancy mapper."""

from __future__ import annotations

import argparse
from collections import deque
import math
import time

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-topic", default="/s2_lidar_slam/map")
    parser.add_argument("--odom-topic", default="/s2_lidar_slam/odom")
    parser.add_argument("--action-name", default="/navigate_to_pose")
    parser.add_argument("--min-frontier-cells", type=int, default=10)
    parser.add_argument("--goal-clearance-m", type=float, default=0.20)
    parser.add_argument("--min-goal-distance-m", type=float, default=0.75)
    parser.add_argument("--max-goal-distance-m", type=float, default=7.0)
    parser.add_argument("--goal-timeout-sec", type=float, default=90.0)
    parser.add_argument("--inspection-pause-sec", type=float, default=2.0)
    parser.add_argument("--retry-radius-m", type=float, default=0.35)
    parser.add_argument("--blacklist-timeout-sec", type=float, default=45.0)
    parser.add_argument("--frontier-standoff-m", type=float, default=0.50)
    parser.add_argument("--startup-delay-sec", type=float, default=15.0)
    parser.add_argument("--no-frontier-cycles", type=int, default=30)
    return parser.parse_args()


def frontier_mask(grid: np.ndarray) -> np.ndarray:
    """Return free cells with at least one 4-connected unknown neighbor."""
    free = grid == 0
    unknown = grid < 0
    adjacent_unknown = np.zeros_like(free)
    adjacent_unknown[1:, :] |= unknown[:-1, :]
    adjacent_unknown[:-1, :] |= unknown[1:, :]
    adjacent_unknown[:, 1:] |= unknown[:, :-1]
    adjacent_unknown[:, :-1] |= unknown[:, 1:]
    return free & adjacent_unknown


def connected_components(mask: np.ndarray) -> list[np.ndarray]:
    seen = np.zeros_like(mask, dtype=bool)
    components: list[np.ndarray] = []
    height, width = mask.shape
    for start_row, start_col in np.argwhere(mask):
        if seen[start_row, start_col]:
            continue
        queue = deque([(int(start_row), int(start_col))])
        seen[start_row, start_col] = True
        cells: list[tuple[int, int]] = []
        while queue:
            row, col = queue.popleft()
            cells.append((row, col))
            for next_row, next_col in (
                (row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)
            ):
                if (
                    0 <= next_row < height
                    and 0 <= next_col < width
                    and mask[next_row, next_col]
                    and not seen[next_row, next_col]
                ):
                    seen[next_row, next_col] = True
                    queue.append((next_row, next_col))
        components.append(np.asarray(cells, dtype=np.int32))
    return components


def cells_to_world(cells: np.ndarray, origin_x: float, origin_y: float, resolution: float) -> np.ndarray:
    result = np.empty((len(cells), 2), dtype=np.float64)
    result[:, 0] = origin_x + (cells[:, 1] + 0.5) * resolution
    result[:, 1] = origin_y + (cells[:, 0] + 0.5) * resolution
    return result


def approach_cell(
    grid: np.ndarray,
    frontier_cell: np.ndarray,
    robot_cell: tuple[float, float],
    standoff_cells: float,
    clearance_cells: int,
) -> tuple[int, int] | None:
    """Choose a safe known-free cell behind a frontier, toward the robot."""
    frontier = np.asarray(frontier_cell, dtype=np.float64)
    robot = np.asarray(robot_cell, dtype=np.float64)
    direction = robot - frontier
    norm = float(np.linalg.norm(direction))
    if norm < 1e-6:
        return None
    target = frontier + direction / norm * standoff_cells
    search_radius = max(2, int(math.ceil(standoff_cells * 0.75)))
    best: tuple[float, int, int] | None = None
    height, width = grid.shape
    for row in range(max(0, int(target[0]) - search_radius), min(height, int(target[0]) + search_radius + 1)):
        for col in range(max(0, int(target[1]) - search_radius), min(width, int(target[1]) + search_radius + 1)):
            if grid[row, col] != 0:
                continue
            row0, row1 = max(0, row - clearance_cells), min(height, row + clearance_cells + 1)
            col0, col1 = max(0, col - clearance_cells), min(width, col + clearance_cells + 1)
            neighborhood = grid[row0:row1, col0:col1]
            # A navigation goal must sit wholly in observed free space. This
            # avoids Nav2 rejecting poses placed directly on the unknown edge.
            if np.any(neighborhood != 0):
                continue
            score = (row - target[0]) ** 2 + (col - target[1]) ** 2
            if best is None or score < best[0]:
                best = (float(score), row, col)
    return None if best is None else (best[1], best[2])


def main() -> None:
    args = parse_args()
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    class Explorer(Node):
        def __init__(self) -> None:
            super().__init__("s2_frontier_explorer")
            self.grid: OccupancyGrid | None = None
            self.robot_xy: tuple[float, float] | None = None
            self.goal_handle = None
            self.goal_pending = False
            self.goal_started_at = 0.0
            self.next_goal_after = 0.0
            self.no_frontier_count = 0
            self.blacklist: list[tuple[float, float, float]] = []
            self.current_goal: tuple[float, float] | None = None
            self.started_at = time.monotonic()
            map_qos = QoSProfile(depth=1)
            map_qos.reliability = ReliabilityPolicy.RELIABLE
            map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(OccupancyGrid, args.map_topic, self.on_map, map_qos)
            self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
            self.client = ActionClient(self, NavigateToPose, args.action_name)
            self.create_timer(1.0, self.tick)

        def on_map(self, message: OccupancyGrid) -> None:
            self.grid = message

        def on_odom(self, message: Odometry) -> None:
            position = message.pose.pose.position
            self.robot_xy = (float(position.x), float(position.y))

        def choose_goal(self) -> tuple[float, float] | None:
            if self.grid is None or self.robot_xy is None:
                return None
            info = self.grid.info
            values = np.asarray(self.grid.data, dtype=np.int16).reshape(
                int(info.height), int(info.width)
            )
            mask = frontier_mask(values)
            now = time.monotonic()
            self.blacklist = [entry for entry in self.blacklist if entry[2] > now]
            candidates: list[tuple[float, float, float]] = []
            for component in connected_components(mask):
                if len(component) < int(args.min_frontier_cells):
                    continue
                world = cells_to_world(
                    component,
                    float(info.origin.position.x),
                    float(info.origin.position.y),
                    float(info.resolution),
                )
                center = world.mean(axis=0)
                representative_index = int(np.argmin(np.linalg.norm(world - center, axis=1)))
                representative_cell = component[representative_index]
                clearance_cells = int(
                    math.ceil(float(args.goal_clearance_m) / float(info.resolution))
                )
                robot_cell = (
                    (self.robot_xy[1] - float(info.origin.position.y)) / float(info.resolution) - 0.5,
                    (self.robot_xy[0] - float(info.origin.position.x)) / float(info.resolution) - 0.5,
                )
                selected_cell = approach_cell(
                    values, representative_cell, robot_cell,
                    float(args.frontier_standoff_m) / float(info.resolution), clearance_cells,
                )
                if selected_cell is None:
                    continue
                selected_world = cells_to_world(
                    np.asarray([selected_cell]), float(info.origin.position.x),
                    float(info.origin.position.y), float(info.resolution),
                )[0]
                x, y = float(selected_world[0]), float(selected_world[1])
                distance = math.hypot(x - self.robot_xy[0], y - self.robot_xy[1])
                if not (float(args.min_goal_distance_m) <= distance <= float(args.max_goal_distance_m)):
                    continue
                if any(
                    math.hypot(x - old_x, y - old_y) < float(args.retry_radius_m)
                    for old_x, old_y, _ in self.blacklist
                ):
                    continue
                # Prefer large information boundaries without repeatedly making
                # long traversals across the known map.
                score = float(len(component)) * float(info.resolution) - 0.35 * distance
                candidates.append((score, x, y))
            if not candidates:
                return None
            _, x, y = max(candidates)
            return x, y

        def tick(self) -> None:
            now = time.monotonic()
            if now - self.started_at < float(args.startup_delay_sec):
                return
            if self.goal_handle is not None:
                if now - self.goal_started_at > float(args.goal_timeout_sec):
                    self.get_logger().warning("Frontier goal timed out; canceling it")
                    self.goal_handle.cancel_goal_async()
                    if self.current_goal is not None:
                        self.blacklist.append((*self.current_goal, now + float(args.blacklist_timeout_sec)))
                    self.goal_handle = None
                    self.next_goal_after = now + float(args.inspection_pause_sec)
                return
            if self.goal_pending:
                return
            if now < self.next_goal_after:
                return
            if not self.client.wait_for_server(timeout_sec=0.1):
                self.get_logger().info("Waiting for Nav2 navigate_to_pose action")
                return
            selected = self.choose_goal()
            if selected is None:
                self.no_frontier_count += 1
                if self.no_frontier_count >= int(args.no_frontier_cycles):
                    self.get_logger().info(
                        "No reachable frontier currently; waiting for map update and retry cooldown"
                    )
                    self.no_frontier_count = 0
                return
            self.no_frontier_count = 0
            x, y = selected
            yaw = math.atan2(y - self.robot_xy[1], x - self.robot_xy[0])
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.header.frame_id = "map"
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
            goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
            self.current_goal = selected
            self.get_logger().info(f"Exploring frontier x={x:.2f}, y={y:.2f}")
            self.goal_pending = True
            future = self.client.send_goal_async(goal)
            future.add_done_callback(self.on_goal_response)

        def on_goal_response(self, future) -> None:
            self.goal_pending = False
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warning("Frontier goal rejected")
                if self.current_goal is not None:
                    self.blacklist.append((*self.current_goal, time.monotonic() + float(args.blacklist_timeout_sec)))
                self.current_goal = None
                self.next_goal_after = time.monotonic() + 1.0
                return
            self.goal_handle = handle
            self.goal_started_at = time.monotonic()
            handle.get_result_async().add_done_callback(self.on_result)

        def on_result(self, future) -> None:
            status = future.result().status
            if status != GoalStatus.STATUS_SUCCEEDED and self.current_goal is not None:
                self.blacklist.append(
                    (*self.current_goal, time.monotonic() + float(args.blacklist_timeout_sec))
                )
                self.get_logger().warning(f"Frontier navigation failed, status={status}")
            else:
                self.get_logger().info("Frontier reached; observing surroundings")
            self.goal_handle = None
            self.current_goal = None
            self.next_goal_after = time.monotonic() + float(args.inspection_pause_sec)

    rclpy.init()
    node = Explorer()
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
