"""Send the complete GLES reconstruction composite through the FPV SDK."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sender_share = FindPackageShare("video_transmission")
    display_width = LaunchConfiguration("display_width")
    display_height = LaunchConfiguration("display_height")
    video_fps = LaunchConfiguration("video_fps")
    render_output_topic = LaunchConfiguration("render_output_topic")

    arguments = [
        DeclareLaunchArgument("display_width", default_value="1920"),
        DeclareLaunchArgument("display_height", default_value="1080"),
        DeclareLaunchArgument("video_fps", default_value="30"),
        DeclareLaunchArgument(
            "headless_render", default_value="true",
            choices=["true", "false"],
            description="Use offscreen GLES; false also displays through DRM/DP"),
        DeclareLaunchArgument(
            "render_output_topic", default_value="/esdf/render/frame_desc"),
        DeclareLaunchArgument(
            "video_config_file",
            default_value=PathJoinSubstitution(
                [sender_share, "config", "sky.yaml"]),
            description="FPV SDK transmitter configuration, including link endpoints"),
        DeclareLaunchArgument(
            "video_cpu_affinity", default_value="0-7",
            description="Transmitter cores; empty leaves placement to the scheduler"),
        DeclareLaunchArgument("video_max_input_age_ms", default_value="100"),
    ]

    reconstruction = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("bringup"), "launch", "vio_las2_esdf_launch.py",
        ])),
        launch_arguments={
            "display_width": display_width,
            "display_height": display_height,
            # The renderer declares a floating-point rate; the encoder uses int.
            "render_fps": PythonExpression(["float(", video_fps, ")"]),
            "headless_render": LaunchConfiguration("headless_render"),
            "publish_render_output": "true",
            "render_output_topic": render_output_topic,
            "render_output_format": "bgra",
            "enable_rtsp": "false",
            "enable_esdf_rtsp": "false",
        }.items(),
    )
    sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            sender_share, "launch", "sender.launch.py",
        ])),
        launch_arguments={
            "config_file": LaunchConfiguration("video_config_file"),
            "input_topic": render_output_topic,
            "width": display_width,
            "height": display_height,
            "fps": video_fps,
            "sender_cpu_affinity": LaunchConfiguration("video_cpu_affinity"),
            "max_input_age_ms": LaunchConfiguration("video_max_input_age_ms"),
        }.items(),
    )
    return LaunchDescription(arguments + [reconstruction, sender])
