#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("aurora_bringup"))
    sim_share = Path(get_package_share_directory("aurora_sim"))
    return LaunchDescription(
        [
            Node(
                package="aurora_ros",
                executable="aurora_planner_node",
                name="aurora_planner_node",
                output="screen",
                parameters=[
                    str(bringup_share / "config" / "aurora_planner.yaml")
                ],
            ),
            Node(
                package="aurora_sim",
                executable="aurora_sim_node",
                name="aurora_sim_node",
                output="screen",
                parameters=[str(sim_share / "config" / "aurora_sim.yaml")],
            ),
        ]
    )
