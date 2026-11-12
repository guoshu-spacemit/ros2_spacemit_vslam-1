"""Launch the stereo camera publisher with package-local calibration and capture options."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("camera_io"),
        "config",
        "usb_stereo_4000x1200.json",
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Camera bridge JSON configuration",
        ),
        DeclareLaunchArgument(
            "input_file",
            default_value="",
            description="Optional MP4 override; selects input.type=file",
        ),
        DeclareLaunchArgument(
            "device",
            default_value="",
            description="Optional UVC device override",
        ),
        DeclareLaunchArgument("width", default_value="0"),
        DeclareLaunchArgument("height", default_value="0"),
        DeclareLaunchArgument("fps", default_value="0"),
        DeclareLaunchArgument("uvc_device_id", default_value="0"),
        DeclareLaunchArgument("uvc_channel_id", default_value="0"),
        DeclareLaunchArgument("vdec_channel_id", default_value="0"),
        Node(
            package="camera_io",
            executable="camera_mpp_publisher",
            name="camera_io",
            output="screen",
            parameters=[{
                "config_file": LaunchConfiguration("config_file"),
                "input_file": LaunchConfiguration("input_file"),
                "device": LaunchConfiguration("device"),
                "width": ParameterValue(
                    LaunchConfiguration("width"), value_type=int
                ),
                "height": ParameterValue(
                    LaunchConfiguration("height"), value_type=int
                ),
                "fps": ParameterValue(
                    LaunchConfiguration("fps"), value_type=int
                ),
                "uvc_device_id": ParameterValue(
                    LaunchConfiguration("uvc_device_id"), value_type=int
                ),
                "uvc_channel_id": ParameterValue(
                    LaunchConfiguration("uvc_channel_id"), value_type=int
                ),
                "vdec_channel_id": ParameterValue(
                    LaunchConfiguration("vdec_channel_id"), value_type=int
                ),
            }],
        )
    ])
