#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    api_version = "v1_3"
    serial_no = "A31_002724092"

    arg_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    arg_namespace = DeclareLaunchArgument("namespace", default_value="")

    config_control = LaunchConfiguration(
        "config_control",
        default=os.path.join(
            get_package_share_directory("boxer_control"), "config", "control.yaml"
        ),
    )

    return LaunchDescription(
        [
            arg_use_sim_time,
            arg_namespace,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("boxer_description"),
                        "/launch/description_launch.py",
                    ]
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
            # For simulation
            # Node(
            #     package="controller_manager",
            #     executable="ros2_control_node",
            #     output="screen",
            #     namespace=LaunchConfiguration("namespace"),
            #     parameters=[
            #         ParameterFile(
            #             RewrittenYaml(
            #                 source_file=config_control,
            #                 param_rewrites={},
            #                 root_key=LaunchConfiguration("namespace"),
            #             ),
            #             allow_substs=True,
            #         ),
            #     ],
            #     respawn=False,
            #     remappings=[("~/robot_description", "robot_description")],
            # ),
            # Node(
            #     package="controller_manager",
            #     executable="spawner",
            #     namespace=LaunchConfiguration("namespace"),
            #     arguments=[
            #         "joint_state_broadcaster",
            #         "--controller-manager",
            #         "controller_manager",
            #     ],
            # ),
            # Node(
            #     package="controller_manager",
            #     executable="spawner",
            #     namespace=LaunchConfiguration("namespace"),
            #     arguments=[
            #         "boxer_velocity_controller",
            #         "--controller-manager",
            #         "controller_manager",
            #     ],
            # ),
            Node(
                package="twist_mux",
                executable="twist_mux",
                output="screen",
                parameters=[config_control],
                remappings={
                    (
                        "/cmd_vel_out",
                        f"/cpr_platform_api/{api_version}/{serial_no}/platform_control/cmd_vel",
                    )
                },
            ),
        ]
    )
