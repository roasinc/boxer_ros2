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

    control_launch_dir = os.path.join(
        get_package_share_directory("boxer_control"), "launch"
    )

    return LaunchDescription(
        [
            Node(
                package="topic_tools",
                executable="relay",
                name="imu_module0_data_raw_relay",
                output="screen",
                arguments=[
                    f"/cpr_platform_api/{api_version}/{serial_no}/imu/module0/data_raw",
                    "/imu/module0/data_raw",
                ],
            ),
            Node(
                package="topic_tools",
                executable="relay",
                name="imu_module0_magnetic_field_relay",
                output="screen",
                arguments=[
                    f"/cpr_platform_api/{api_version}/{serial_no}/imu/module0/magnetic_field",
                    "/imu/module0/magnetic_field",
                ],
            ),
            Node(
                package="topic_tools",
                executable="relay",
                name="imu_module1_data_raw_relay",
                output="screen",
                arguments=[
                    f"/cpr_platform_api/{api_version}/{serial_no}/imu/module1/data_raw",
                    "/imu/module1/data_raw",
                ],
            ),
            Node(
                package="topic_tools",
                executable="relay",
                name="realsense_module0_depth_camera_info_relay",
                output="screen",
                arguments=[
                    f"/cpr_platform_api/{api_version}/{serial_no}/realsense/module0/depth/camera_info",
                    "/realsense/depth/camera_info",
                ],
            ),
            Node(
                package="topic_tools",
                executable="relay",
                name="realsense_module0_depth_image_rect_raw_relay",
                output="screen",
                arguments=[
                    f"/cpr_platform_api/{api_version}/{serial_no}/realsense/module0/depth/image_rect_raw",
                    "/realsense/depth/image_rect_raw",
                ],
            ),
            Node(
                package="topic_tools",
                executable="relay",
                name="platform_joint_states_relay",
                output="screen",
                arguments=[
                    f"/cpr_platform_api/{api_version}/{serial_no}/platform/joint_states",
                    "/joint_states",
                ],
            ),
            Node(
                package="topic_tools",
                executable="relay",
                name="platform_emergency_stop_relay",
                output="screen",
                arguments=[
                    f"/cpr_platform_api/{api_version}/{serial_no}/platform/emergency_stop",
                    "/platform/emergency_stop",
                ],
            ),
            Node(
                package="topic_tools",
                executable="relay",
                name="platform_safety_stop_relay",
                output="screen",
                arguments=[
                    f"/cpr_platform_api/{api_version}/{serial_no}/platform/safety_stop",
                    "/platform/safety_stop",
                ],
            ),
            Node(
                package="boxer_bringup",
                executable="boxer_bridge",
                output="screen",
                parameters=[
                    {
                        "api_version": api_version,
                        "serial_no": serial_no,
                        "relay_odom": False,
                        "relay_scan": False,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [control_launch_dir, "/control_launch.py"]
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([control_launch_dir, "/teleop_launch.py"])
            ),
        ]
    )
