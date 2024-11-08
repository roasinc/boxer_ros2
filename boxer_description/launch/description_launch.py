#!/usr/bin/env python3

import os
from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    urdf_file_path = os.path.join(
        get_package_share_directory("boxer_description"), "urdf", "boxer.urdf.xacro"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                name="urdf_file_path",
                default_value=urdf_file_path,
                description="Absolute path to robot urdf file",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro ",
                                LaunchConfiguration("urdf_file_path"),
                            ]
                        ),
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
