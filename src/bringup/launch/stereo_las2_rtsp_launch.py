"""Launch stereo capture and LAS2 depth with optional RTSP presentation."""

from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    runtime = Path(get_package_prefix("las2")) / "lib" / "libstereo_matching_runtime.so"
    default_las2_preload = (
        str(runtime) if runtime.is_file() else "libonnxruntime.so.1:libspacemit_ep.so.2"
    )
    camera_share = Path(get_package_share_directory("camera_io"))

    arguments = [
        DeclareLaunchArgument(
            "camera_config_file",
            default_value=str(camera_share / "config" / "usb_stereo_4000x1200.json"),
        ),
        DeclareLaunchArgument("input_file", default_value=""),
        DeclareLaunchArgument("camera_ld_preload", default_value="libmpp.so"),
        DeclareLaunchArgument(
            "las2_ld_preload",
            default_value=default_las2_preload,
        ),
        DeclareLaunchArgument(
            "calibration_file",
            default_value=str(
                camera_share
                / "config"
                / "0.yaml"
            ),
        ),
        DeclareLaunchArgument("model_path"),
        DeclareLaunchArgument(
            "camera_full_topic", default_value="/camera/full/frame_desc"
        ),
        DeclareLaunchArgument("depth_topic", default_value="/las2/depth_desc"),
        DeclareLaunchArgument(
            "preview_topic", default_value="/las2/preview/frame_desc"
        ),
        DeclareLaunchArgument("las2_instance_count", default_value="2"),
        DeclareLaunchArgument("las2_threads_per_instance", default_value="4"),
        DeclareLaunchArgument(
            "las2_core_affinity", default_value="8;9;10;11/12;13;14;15"
        ),
        DeclareLaunchArgument("las2_queue_depth", default_value="4"),
        DeclareLaunchArgument("las2_output_pool_depth", default_value="8"),
        DeclareLaunchArgument(
            "preview_visualization", default_value="disparity"
        ),
        DeclareLaunchArgument("preview_disparity_min", default_value="0.0"),
        DeclareLaunchArgument("preview_disparity_max", default_value="128.0"),
        DeclareLaunchArgument("preview_depth_min_m", default_value="0.0"),
        DeclareLaunchArgument("preview_depth_max_m", default_value="3.0"),
        DeclareLaunchArgument(
            "rtsp_url", default_value="rtsp://0.0.0.0:8554/stereo"
        ),
        DeclareLaunchArgument("rtsp_fps", default_value="30"),
        DeclareLaunchArgument("rtsp_bitrate", default_value="4000000"),
        DeclareLaunchArgument("rtsp_gop", default_value="30"),
        DeclareLaunchArgument("rtsp_venc_channel", default_value="0"),
        DeclareLaunchArgument("rtsp_mux_channel", default_value="0"),
    ]

    camera = Node(
        package="camera_io",
        executable="camera_mpp_publisher",
        name="camera_io",
        output="screen",
        additional_env={"LD_PRELOAD": LaunchConfiguration("camera_ld_preload")},
        parameters=[
            {
                "config_file": LaunchConfiguration("camera_config_file"),
                "input_file": LaunchConfiguration("input_file"),
                "autostart": True,
            }
        ],
    )

    las2 = Node(
        package="las2",
        executable="las2_node",
        name="las2",
        output="screen",
        additional_env={"LD_PRELOAD": LaunchConfiguration("las2_ld_preload")},
        parameters=[
            {
                "input_topic": LaunchConfiguration("camera_full_topic"),
                "output_topic": LaunchConfiguration("depth_topic"),
                "preview_topic": LaunchConfiguration("preview_topic"),
                "publish_preview": True,
                "preview_visualization": LaunchConfiguration(
                    "preview_visualization"
                ),
                "preview_disparity_min": LaunchConfiguration(
                    "preview_disparity_min"
                ),
                "preview_disparity_max": LaunchConfiguration(
                    "preview_disparity_max"
                ),
                "preview_depth_min_m": LaunchConfiguration("preview_depth_min_m"),
                "preview_depth_max_m": LaunchConfiguration("preview_depth_max_m"),
                "calibration_file": LaunchConfiguration("calibration_file"),
                "model_path": LaunchConfiguration("model_path"),
                "core_count": LaunchConfiguration("las2_instance_count"),
                "threads_per_instance": LaunchConfiguration(
                    "las2_threads_per_instance"
                ),
                "core_affinity": LaunchConfiguration("las2_core_affinity"),
                "queue_depth": LaunchConfiguration("las2_queue_depth"),
                "output_pool_depth": LaunchConfiguration(
                    "las2_output_pool_depth"
                ),
            }
        ],
    )

    rtsp = Node(
        package="mpp_rtsp",
        executable="mpp_rtsp_node",
        name="stereo_rtsp",
        output="screen",
        additional_env={"LD_PRELOAD": LaunchConfiguration("camera_ld_preload")},
        parameters=[
            {
                "input_topic": LaunchConfiguration("preview_topic"),
                "width": 0,
                "height": 0,
                "fps": LaunchConfiguration("rtsp_fps"),
                "codec": "h264",
                "bitrate": LaunchConfiguration("rtsp_bitrate"),
                "gop": LaunchConfiguration("rtsp_gop"),
                "rtsp_url": LaunchConfiguration("rtsp_url"),
                "rtp_timestamp_source": "arrival",
                "venc_channel": LaunchConfiguration("rtsp_venc_channel"),
                "mux_channel": LaunchConfiguration("rtsp_mux_channel"),
                "overlay_enabled": False,
                "feature_overlay_enabled": False,
            }
        ],
    )

    return LaunchDescription(arguments + [camera, las2, rtsp])
