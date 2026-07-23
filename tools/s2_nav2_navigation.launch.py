#!/usr/bin/env python3
"""Launch the S2 Nav2 stack against DREAM's live lidar occupancy map."""

from __future__ import annotations

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from s2_robot_model import DEFAULT_URDF, load_rviz_robot_description


def _robot_state_publisher(context):
    urdf_file = LaunchConfiguration("urdf_file").perform(context)
    description = load_rviz_robot_description(urdf_file)
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="s2_robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": description,
                    "publish_frequency": 20.0,
                    "ignore_timestamp": True,
                }
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map_yaml")
    use_rviz = LaunchConfiguration("use_rviz")
    enable_motion = LaunchConfiguration("enable_motion")
    allow_shared_output = LaunchConfiguration("allow_shared_output")
    controller_host = LaunchConfiguration("controller_host")
    robot_id = LaunchConfiguration("robot_id")
    max_linear_x = LaunchConfiguration("max_linear_x")
    max_linear_y = LaunchConfiguration("max_linear_y")
    max_angular_z = LaunchConfiguration("max_angular_z")
    command_transport = LaunchConfiguration("command_transport")
    command_output_topic = LaunchConfiguration("command_output_topic")
    log_level = LaunchConfiguration("log_level")

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother",
        "collision_monitor",
    ]
    common = {
        "output": "screen",
        "parameters": [params_file],
        "arguments": ["--ros-args", "--log-level", log_level],
    }
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]

    bridge_command = [
        "/usr/bin/python3",
        str(ROOT / "tools/s2_lidar_rviz_bridge.py"),
        "--odom-topic",
        "/controller/odom",
        "--output-odom-topic",
        "/s2_lidar_slam/odom",
        "--parent-frame",
        "odom",
        "--child-frame",
        "base_link",
        "--odom-yaw-mode",
        "orientation_w",
    ]
    safety_cloud_command = [
        "/usr/bin/python3",
        str(ROOT / "tools/s2_safety_cloud_relay.py"),
        "--input-topic",
        "/driver/lidar/point_cloud/Data",
        "--output-topic",
        "/s2_lidar_slam/point_cloud",
        "--heartbeat-topic",
        "/s2_lidar_slam/cloud_count",
        "--restamp-now",
        "--resubscribe-sec",
        "1.5",
    ]
    goal_pose_bridge_command = [
        "/usr/bin/python3",
        str(ROOT / "tools/s2_goal_pose_bridge.py"),
    ]
    ultrasonic_relay_command = [
        "/usr/bin/python3",
        str(ROOT / "tools/s2_ultrasonic_relay.py"),
    ]
    command_bridge = [
        str(ROOT / "tools/s2_keyboard_mapping_bridge.sh"),
        "--input-topic",
        # CollisionMonitor on the S2/Humble combination can enter an active
        # state while publishing no messages at all. DWB's live voxel and
        # ultrasonic costmaps remain the motion safety authority; bridge the
        # validated smoothed controller output so navigation cannot deadlock.
        "/s2_nav2/cmd_vel_smoothed",
        "--preview-topic",
        "/s2_nav2/cmd_vel_preview",
        "--output-topic",
        command_output_topic,
        "--transport",
        command_transport,
        "--controller-host",
        controller_host,
        "--robot-id",
        robot_id,
        "--max-linear-x",
        max_linear_x,
        "--max-linear-y",
        max_linear_y,
        "--max-angular-z",
        max_angular_z,
        "--safety-cloud-topic",
        "/s2_lidar_slam/point_cloud",
        "--safety-heartbeat-topic",
        "/s2_lidar_slam/cloud_count",
        "--safety-cloud-timeout-sec",
        "3.0",
        "--skip-motion-burst-mode-refresh",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(ROOT / "configs/navigation/s2_nav2_params.yaml"),
            ),
            DeclareLaunchArgument("map_yaml", default_value=""),
            DeclareLaunchArgument("urdf_file", default_value=str(DEFAULT_URDF)),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("enable_motion", default_value="false"),
            DeclareLaunchArgument("allow_shared_output", default_value="false"),
            DeclareLaunchArgument("controller_host", default_value="192.168.127.10"),
            DeclareLaunchArgument("robot_id", default_value="1"),
            DeclareLaunchArgument("max_linear_x", default_value="0.12"),
            DeclareLaunchArgument("max_linear_y", default_value="0.10"),
            DeclareLaunchArgument("max_angular_z", default_value="0.30"),
            DeclareLaunchArgument("command_transport", default_value="ros_topic"),
            DeclareLaunchArgument(
                "command_output_topic", default_value="/move/ManualMoveCmd"
            ),
            DeclareLaunchArgument("log_level", default_value="info"),
            ExecuteProcess(cmd=bridge_command, output="screen"),
            ExecuteProcess(cmd=safety_cloud_command, output="screen"),
            ExecuteProcess(cmd=goal_pose_bridge_command, output="screen"),
            ExecuteProcess(cmd=ultrasonic_relay_command, output="screen"),
            OpaqueFunction(function=_robot_state_publisher),
            # With no saved map/AMCL, the live mapper builds its map directly
            # in odometry coordinates.  Publish the identity transform so the
            # Nav2 global stack can consistently use its configured map frame.
            Node(
                condition=UnlessCondition(
                    PythonExpression(["'", map_yaml, "' != ''"])
                ),
                package="tf2_ros",
                executable="static_transform_publisher",
                name="s2_live_map_to_odom",
                output="screen",
                arguments=[
                    "0", "0", "0", "0", "0", "0", "map", "odom"
                ],
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="s2_pointcloud_to_laserscan",
                output="screen",
                remappings=[
                    ("cloud_in", "/s2_lidar_slam/point_cloud"),
                    ("scan", "/s2_lidar_slam/scan"),
                ],
                parameters=[
                    {
                        "target_frame": "base_link",
                        "transform_tolerance": 0.10,
                        "min_height": 0.25,
                        "max_height": 1.20,
                        "angle_min": -3.14159,
                        "angle_max": 3.14159,
                        "angle_increment": 0.00873,
                        "scan_time": 0.10,
                        "range_min": 0.25,
                        "range_max": 8.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
            ),
            Node(
                condition=IfCondition(
                    PythonExpression(["'", map_yaml, "' != ''"])
                ),
                package="nav2_map_server",
                executable="map_server",
                name="s2_saved_map_server",
                output="screen",
                parameters=[{"yaml_filename": map_yaml, "frame_id": "map"}],
                remappings=[("map", "/s2_lidar_slam/map")],
            ),
            Node(
                condition=IfCondition(
                    PythonExpression(["'", map_yaml, "' != ''"])
                ),
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[params_file],
                remappings=[
                    ("map", "/s2_lidar_slam/map"),
                    ("scan", "/s2_lidar_slam/scan"),
                ],
            ),
            Node(
                condition=IfCondition(
                    PythonExpression(["'", map_yaml, "' != ''"])
                ),
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_saved_map",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "autostart": True,
                        "node_names": ["s2_saved_map_server", "amcl"],
                        "bond_timeout": 4.0,
                    }
                ],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                remappings=tf_remaps
                + [("cmd_vel", "/s2_nav2/cmd_vel_raw")],
                **common,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                remappings=tf_remaps,
                **common,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                remappings=tf_remaps
                + [("cmd_vel", "/s2_nav2/cmd_vel_raw")],
                **common,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                remappings=tf_remaps,
                **common,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                remappings=tf_remaps
                + [
                    ("cmd_vel", "/s2_nav2/cmd_vel_raw"),
                    ("cmd_vel_smoothed", "/s2_nav2/cmd_vel_smoothed"),
                ],
                **common,
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                remappings=tf_remaps,
                **common,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "autostart": True,
                        "node_names": lifecycle_nodes,
                        "bond_timeout": 4.0,
                    }
                ],
            ),
            ExecuteProcess(
                condition=UnlessCondition(enable_motion),
                cmd=command_bridge,
                output="screen",
            ),
            ExecuteProcess(
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            enable_motion,
                            "' == 'true' and '",
                            allow_shared_output,
                            "' != 'true'",
                        ]
                    )
                ),
                cmd=command_bridge + ["--enable-motion"],
                output="screen",
            ),
            ExecuteProcess(
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            enable_motion,
                            "' == 'true' and '",
                            allow_shared_output,
                            "' == 'true'",
                        ]
                    )
                ),
                cmd=command_bridge
                + ["--enable-motion", "--allow-shared-output"],
                output="screen",
            ),
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                name="s2_nav2_rviz",
                output="screen",
                arguments=[
                    "-d",
                    str(ROOT / "configs/rviz/s2_lidar_mapping.rviz"),
                ],
            ),
        ]
    )
