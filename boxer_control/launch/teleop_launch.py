#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    config_teleop_joy = LaunchConfiguration(
        "config_teleop_joy",
        default=os.path.join(
            get_package_share_directory("boxer_control"), "config", "teleop.yaml"
        ),
    )

    return LaunchDescription(
        [
            Node(
                package="joy_linux",
                executable="joy_linux_node",
                name="joy_node",
                output="screen",
                parameters=[config_teleop_joy, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("joy", "joy_teleop/joy"),
                    ("joy/set_feedback", "joy_teleop/joy/set_feedback"),
                ],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                output="screen",
                parameters=[config_teleop_joy, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("joy", "joy_teleop/joy"),
                    ("cmd_vel", "joy_teleop/cmd_vel"),
                ],
            ),
        ]
    )
