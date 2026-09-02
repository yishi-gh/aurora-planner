#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("aurora_sim"))
    parameter_file = package_share / "config" / "aurora_sim.yaml"
    return LaunchDescription(
        [
            Node(
                package="aurora_sim",
                executable="aurora_sim_node",
                name="aurora_sim_node",
                output="screen",
                parameters=[str(parameter_file)],
            )
        ]
    )
