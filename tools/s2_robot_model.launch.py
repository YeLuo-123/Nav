#!/usr/bin/env python3
"""Publish the S2 URDF and STL model for RViz."""

from __future__ import annotations

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


sys.path.insert(0, str(Path(__file__).resolve().parent))

from s2_robot_model import DEFAULT_URDF, load_rviz_robot_description


def _robot_state_publisher(context):
    urdf_file = LaunchConfiguration("urdf_file").perform(context)
    publish_frequency = float(
        LaunchConfiguration("publish_frequency").perform(context)
    )
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
                    "publish_frequency": publish_frequency,
                    "ignore_timestamp": True,
                }
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("urdf_file", default_value=str(DEFAULT_URDF)),
            DeclareLaunchArgument("publish_frequency", default_value="20.0"),
            OpaqueFunction(function=_robot_state_publisher),
        ]
    )
