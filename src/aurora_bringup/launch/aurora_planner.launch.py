#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    package_share = Path(get_package_share_directory("aurora_bringup"))
    parameter_file = package_share / "config" / "aurora_planner.yaml"
    return LaunchDescription(
        [
            Node(
                package="aurora_ros",
                executable="aurora_planner_node",
                name="aurora_planner_node",
                output="screen",
                parameters=[str(parameter_file)],
            )
        ]
    )
