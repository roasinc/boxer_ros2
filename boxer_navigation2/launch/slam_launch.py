#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    namespace = LaunchConfiguration("namespace", default_value="")
    config_slam = LaunchConfiguration(
        "config_slam",
        default=os.path.join(
            get_package_share_directory("boxer_navigation2"), "config", "slam.yaml"
        ),
    )

    return LaunchDescription(
        [
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    ParameterFile(
                        RewrittenYaml(
                            source_file=config_slam,
                            param_rewrites={},
                            root_key=namespace,
                        ),
                        allow_substs=True,
                    ),
                    {"use_sim_time": use_sim_time},
                ],
            )
        ]
    )
