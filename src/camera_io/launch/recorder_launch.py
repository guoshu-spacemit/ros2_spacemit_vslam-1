"""Launch camera recording with package-local capture and recorder configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("camera_io"),
        "config",
        "mjpeg_recorder.json",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            Node(
                package="camera_io",
                executable="recorder_node",
                name="recorder",
                output="screen",
                parameters=[{"config_file": LaunchConfiguration("config_file")}],
            ),
        ]
    )
