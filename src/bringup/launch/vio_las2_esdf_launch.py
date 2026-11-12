"""Launch independently scheduled stereo VIO, LAS2 depth, mapping, and presentation."""

from pathlib import Path
import re

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def configure_execution(context):
    """Validate file input and retain configurable application CPU placement."""
    file_packet_reader = LaunchConfiguration("file_packet_reader").perform(context)
    if file_packet_reader not in ("mpp", "libavformat"):
        raise ValueError("file_packet_reader must be mpp or libavformat")
    cpu_set = LaunchConfiguration("application_cpu_set").perform(context).strip()
    if cpu_set and not re.fullmatch(r"\d+(?:-\d+)?(?:,\d+(?:-\d+)?)*", cpu_set):
        raise ValueError("application_cpu_set must be a taskset CPU list or empty")
    inherited_prefix = LaunchConfiguration("launch-prefix", default="").perform(context)
    prefix_filter = LaunchConfiguration("launch-prefix-filter", default="").perform(context)
    actions = []
    for executable in ("vio_node", "las2_node", "esdf_drm_node"):
        prefix = inherited_prefix if re.match(prefix_filter, executable) else ""
        if cpu_set:
            prefix = f"taskset -c {cpu_set}"
        actions.append(SetLaunchConfiguration(f"_{executable}_prefix", prefix))
    native_map_io = IfCondition(LaunchConfiguration("slam_enabled")).evaluate(context) and any(
        LaunchConfiguration(name).perform(context).strip()
        for name in ("slam_map_save_directory", "slam_map_load_directory")
    )
    actions.extend([
        SetLaunchConfiguration(
            "_localization_sigterm_timeout",
            LaunchConfiguration("localization_shutdown_timeout_sec")
            if native_map_io else LaunchConfiguration("sigterm_timeout", default="5"),
        ),
        SetLaunchConfiguration(
            "_localization_sigkill_timeout",
            "10.0" if native_map_io else LaunchConfiguration("sigkill_timeout", default="5"),
        ),
    ])
    return actions


def reconstruction_arguments():
    """Declare launch arguments for both startup and session preflight."""
    runtime = Path(get_package_prefix("las2")) / "lib" / "libstereo_matching_runtime.so"
    default_las2_preload = (
        str(runtime) if runtime.is_file() else "libonnxruntime.so.1:libspacemit_ep.so.2"
    )
    camera_config_dir = Path(get_package_share_directory("camera_io")) / "config"
    camera_config = str(camera_config_dir / "usb_stereo_4000x1200.json")
    calibration = str(camera_config_dir / "0.yaml")
    calibration_file = LaunchConfiguration("calibration_file")

    return [
        DeclareLaunchArgument(
            "application_cpu_set",
            default_value="",
            description="Optional application CPU list; empty inherits placement",
        ),
        DeclareLaunchArgument("enable_las2", default_value="true"),
        DeclareLaunchArgument("mapping_enabled", default_value="true"),
        DeclareLaunchArgument("camera_config_file", default_value=camera_config),
        DeclareLaunchArgument(
            "camera_fps",
            default_value="0",
            description="Capture-mode FPS override; 0 uses camera JSON",
        ),
        DeclareLaunchArgument("input_file", default_value=""),
        DeclareLaunchArgument(
            "file_packet_reader",
            default_value="mpp",
            description="File packet reader: mpp (default) or libavformat for MJPEG MP4",
        ),
        DeclareLaunchArgument("las2_ready_topic", default_value="/las2/ready"),
        DeclareLaunchArgument(
            "camera_start_ready_topic",
            default_value=LaunchConfiguration("las2_ready_topic"),
            condition=IfCondition(LaunchConfiguration("enable_las2")),
        ),
        DeclareLaunchArgument(
            "camera_start_ready_topic",
            default_value="",
            condition=UnlessCondition(LaunchConfiguration("enable_las2")),
        ),
        DeclareLaunchArgument("camera_start_ready_timeout_sec", default_value="120.0"),
        DeclareLaunchArgument(
            "file_start_ready_topic",
            default_value=LaunchConfiguration("las2_ready_topic"),
            condition=IfCondition(LaunchConfiguration("enable_las2")),
        ),
        DeclareLaunchArgument(
            "file_start_ready_topic",
            default_value="",
            condition=UnlessCondition(LaunchConfiguration("enable_las2")),
        ),
        DeclareLaunchArgument("camera_ld_preload", default_value="libmpp.so"),
        DeclareLaunchArgument(
            "las2_ld_preload",
            default_value=default_las2_preload,
        ),
        DeclareLaunchArgument("calibration_file", default_value=calibration),
        DeclareLaunchArgument("imu_calibration_file", default_value=calibration_file),
        DeclareLaunchArgument(
            "model_path",
            condition=IfCondition(LaunchConfiguration("enable_las2")),
        ),
        DeclareLaunchArgument("las2_instance_count", default_value="2"),
        DeclareLaunchArgument("las2_threads_per_instance", default_value="4"),
        DeclareLaunchArgument("las2_warmup_iterations", default_value="2"),
        DeclareLaunchArgument("las2_core_affinity", default_value="8;9;10;11/12;13;14;15"),
        DeclareLaunchArgument("camera_full_topic", default_value="/camera/full/frame_desc"),
        DeclareLaunchArgument("left_topic", default_value="/camera/left/frame_desc"),
        DeclareLaunchArgument("right_topic", default_value="/camera/right/frame_desc"),
        DeclareLaunchArgument("imu_topic", default_value="/camera/imu/data_raw"),
        DeclareLaunchArgument("image_width", default_value="640"),
        DeclareLaunchArgument("image_height", default_value="400"),
        DeclareLaunchArgument("use_imu", default_value="true"),
        DeclareLaunchArgument(
            "spacemit_vslam_verbosity",
            default_value="0",
            description="Native diagnostic trace level; 3 includes map handoff and loop edges",
        ),
        DeclareLaunchArgument(
            "minimum_observation_depth_m",
            default_value="0.0",
            description="Minimum observation depth; 0 keeps library defaults",
        ),
        DeclareLaunchArgument(
            "imu_frequency_hz",
            default_value="0.0",
            description="0 resolves the camera profile; positive overrides it",
        ),
        DeclareLaunchArgument("enable_external_propagation", default_value="false"),
        DeclareLaunchArgument("image_queue_depth", default_value="1"),
        DeclareLaunchArgument("sync_queue_depth", default_value="2"),
        DeclareLaunchArgument("asynchronous_processing", default_value="true"),
        DeclareLaunchArgument(
            "enable_stationary_initialization",
            default_value="false",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument("slam_enabled", default_value="false"),
        DeclareLaunchArgument("slam_map_save_directory", default_value=""),
        DeclareLaunchArgument("slam_map_load_directory", default_value=""),
        DeclareLaunchArgument("slam_save_timeout_sec", default_value="60.0"),
        DeclareLaunchArgument(
            "localization_shutdown_timeout_sec",
            default_value="120.0",
            description="Graceful shutdown budget for native map I/O",
        ),
        DeclareLaunchArgument("slam_frame_record_file", default_value=""),
        DeclareLaunchArgument("slam_localization_audit_file", default_value=""),
        DeclareLaunchArgument("slam_local_frame", default_value="slam_world"),
        DeclareLaunchArgument("slam_map_frame", default_value="map"),
        DeclareLaunchArgument("publish_map_to_odom_tf", default_value="false"),
        DeclareLaunchArgument("global_hint_topic", default_value="/localization/global_hint"),
        DeclareLaunchArgument("slam_status_topic", default_value="/localization/slam_status"),
        DeclareLaunchArgument("slam_ready_topic", default_value="/localization/slam_ready"),
        DeclareLaunchArgument("localization_history_capacity", default_value="256"),
        DeclareLaunchArgument("localization_history_max_bytes", default_value="268435456"),
        DeclareLaunchArgument("slam_retention_time_ms", default_value="10000"),
        DeclareLaunchArgument("slam_sync_mode", default_value="false"),
        DeclareLaunchArgument("slam_maximum_pending_keyframes", default_value="8"),
        DeclareLaunchArgument("slam_maximum_tracked_poses", default_value="16384"),
        DeclareLaunchArgument("slam_localization_horizontal_radius", default_value="0.5"),
        DeclareLaunchArgument("slam_localization_vertical_radius", default_value="0.5"),
        DeclareLaunchArgument("vio_frame_record_file", default_value=""),
        DeclareLaunchArgument("vio_dma_audit_file", default_value=""),
        DeclareLaunchArgument("vio_frame_record_queue_depth", default_value="4096"),
        DeclareLaunchArgument("image_pose_topic", default_value="/vio/pose/image"),
        DeclareLaunchArgument("tracking_mode_topic", default_value="/vio/tracking_mode"),
        DeclareLaunchArgument(
            "las2_pose_gate_topic",
            default_value=LaunchConfiguration("image_pose_topic"),
        ),
        DeclareLaunchArgument("las2_pose_gate_image_depth", default_value="8"),
        DeclareLaunchArgument("imu_pose_topic", default_value="/vio/pose/imu"),
        DeclareLaunchArgument("gravity_topic", default_value="/vio/gravity"),
        DeclareLaunchArgument("depth_topic", default_value="/las2/depth_desc"),
        DeclareLaunchArgument("pair_buffer_depth", default_value="64"),
        DeclareLaunchArgument("las2_preview_topic", default_value="/las2/preview/frame_desc"),
        DeclareLaunchArgument(
            "preview_overlay_topic",
            default_value=LaunchConfiguration("las2_preview_topic"),
            condition=IfCondition(LaunchConfiguration("enable_las2")),
        ),
        DeclareLaunchArgument(
            "preview_overlay_topic",
            default_value=LaunchConfiguration("left_topic"),
            condition=UnlessCondition(LaunchConfiguration("enable_las2")),
        ),
        DeclareLaunchArgument(
            "feature_panel_enabled",
            default_value="false",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "publish_pose_inputs",
            default_value="false",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "publish_frame_diagnostics",
            default_value=LaunchConfiguration("feature_panel_enabled"),
            choices=["true", "false"],
        ),
        DeclareLaunchArgument("feature_topic", default_value="/vio/features"),
        DeclareLaunchArgument("feature_panel_age_max_frames", default_value="14"),
        DeclareLaunchArgument(
            "feature_panel_image_topic",
            default_value=LaunchConfiguration("left_topic"),
        ),
        DeclareLaunchArgument("pose_input_topic", default_value="/vio/pose_inputs"),
        DeclareLaunchArgument(
            "feature_panel_topic",
            default_value=LaunchConfiguration("feature_topic"),
        ),
        DeclareLaunchArgument("las2_preview_visualization", default_value="depth"),
        DeclareLaunchArgument(
            "las2_preview_center_depth_overlay",
            default_value="true",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument("las2_preview_depth_roi_size", default_value="9"),
        DeclareLaunchArgument("las2_preview_depth_roi_min_valid_fraction", default_value="0.5"),
        DeclareLaunchArgument("las2_preview_disparity_min", default_value="0.0"),
        DeclareLaunchArgument("las2_preview_disparity_max", default_value="128.0"),
        DeclareLaunchArgument("las2_preview_depth_min_m", default_value="0.0"),
        DeclareLaunchArgument("las2_preview_depth_max_m", default_value="5.0"),
        DeclareLaunchArgument("enable_camera", default_value="true"),
        DeclareLaunchArgument("enable_vio", default_value="true"),
        DeclareLaunchArgument("autostart_camera", default_value="true"),
        DeclareLaunchArgument("enable_mapping", default_value="true"),
        DeclareLaunchArgument("enable_gles_drm", default_value="true"),
        DeclareLaunchArgument(
            "desktop_viewer_enabled",
            default_value="false",
            description="Display the headless composite in an X11/Xwayland window",
        ),
        DeclareLaunchArgument(
            "headless_render",
            default_value=LaunchConfiguration("desktop_viewer_enabled"),
        ),
        DeclareLaunchArgument(
            "publish_render_output",
            default_value=LaunchConfiguration("desktop_viewer_enabled"),
        ),
        DeclareLaunchArgument("render_output_topic", default_value="/esdf/render/frame_desc"),
        DeclareLaunchArgument(
            "render_output_format",
            default_value="nv12",
            choices=["nv12", "bgra"],
        ),
        DeclareLaunchArgument("desktop_window_title", default_value="ROBOT reconstruction"),
        DeclareLaunchArgument("desktop_fullscreen", default_value="false"),
        DeclareLaunchArgument(
            "desktop_allow_software_rendering",
            default_value="false",
            description="Allow CPU window rendering only for explicit diagnostics",
        ),
        DeclareLaunchArgument("record_enabled", default_value="false"),
        DeclareLaunchArgument("record_use_source_timestamps", default_value="false"),
        DeclareLaunchArgument("record_timestamp_topic", default_value="/camera/full/frame_desc"),
        DeclareLaunchArgument("record_timestamp_use_frame_pts", default_value="true"),
        DeclareLaunchArgument("record_timestamp_origin_zero", default_value="false"),
        DeclareLaunchArgument(
            "record_output_pattern",
            default_value="recordings/esdf_%Y%m%d_%H%M%S.mp4",
        ),
        DeclareLaunchArgument("record_fps", default_value="60"),
        DeclareLaunchArgument("record_bitrate_bps", default_value="8000000"),
        DeclareLaunchArgument("record_gop", default_value="60"),
        DeclareLaunchArgument("record_pool_depth", default_value="8"),
        DeclareLaunchArgument("record_venc_channel", default_value="2"),
        DeclareLaunchArgument("record_mux_channel", default_value="2"),
        DeclareLaunchArgument("record_wait_for_map", default_value="false"),
        DeclareLaunchArgument("render_fps", default_value="60.0"),
        DeclareLaunchArgument("display_width", default_value="1920"),
        DeclareLaunchArgument("display_height", default_value="1080"),
        DeclareLaunchArgument("render_max_blocks", default_value="8192"),
        DeclareLaunchArgument("render_max_block_updates_per_frame", default_value="0"),
        DeclareLaunchArgument("render_max_points_per_block", default_value="0"),
        DeclareLaunchArgument("render_point_size", default_value="8.0"),
        DeclareLaunchArgument("render_slice_view_half_height_m", default_value="3.0"),
        DeclareLaunchArgument("mouse_navigation_enabled", default_value="true"),
        DeclareLaunchArgument("mouse_input_device", default_value=""),
        DeclareLaunchArgument("render_tsdf_interpolated_points", default_value="false"),
        DeclareLaunchArgument("render_measure_gpu_time", default_value="false"),
        DeclareLaunchArgument("render_async_page_flip", default_value="true"),
        DeclareLaunchArgument("render_frustum_culling", default_value="true"),
        DeclareLaunchArgument("gravity_align_map", default_value="true"),
        DeclareLaunchArgument("display_gravity_imu_topic", default_value=""),
        DeclareLaunchArgument(
            "display_gravity_imu_rotation",
            default_value="[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]",
        ),
        DeclareLaunchArgument("display_gravity_imu_time_shift_s", default_value="0.0"),
        DeclareLaunchArgument("pointcloud_min_weight", default_value="1.0"),
        DeclareLaunchArgument("pointcloud_surface_band_m", default_value="-1.0"),
        DeclareLaunchArgument("esdf_world_frame", default_value="vio_world"),
        DeclareLaunchArgument("map_clearing_radius_m", default_value="0.0"),
        DeclareLaunchArgument("clear_map_outside_radius_rate_hz", default_value="1.0"),
        DeclareLaunchArgument("render_trajectory_duration_s", default_value="0.0"),
        DeclareLaunchArgument(
            "initial_gravity_camera",
            default_value="[0.0, 0.0, 0.0]",
            description=(
                "Optional display gravity in the first camera frame; zero waits for VIO gravity"
            ),
        ),
        DeclareLaunchArgument("follow_slice_height", default_value="true"),
        DeclareLaunchArgument("slice_thickness_m", default_value="0.05"),
        DeclareLaunchArgument("slice_height_offset_m", default_value="0.0"),
        DeclareLaunchArgument("slice_min_z_m", default_value="-0.25"),
        DeclareLaunchArgument("slice_max_z_m", default_value="0.25"),
        DeclareLaunchArgument("slice_output_z_m", default_value="0.0"),
        DeclareLaunchArgument("voxel_size_m", default_value="0.05"),
        DeclareLaunchArgument("truncation_distance_m", default_value="0.15"),
        DeclareLaunchArgument("max_integration_distance_m", default_value="3.0"),
        DeclareLaunchArgument(
            "projective_tsdf_integrator_invalid_depth_decay_factor",
            default_value="0.8",
        ),
        DeclareLaunchArgument("max_esdf_distance_m", default_value="2.0"),
        DeclareLaunchArgument("depth_subsampling_factor", default_value="1"),
        DeclareLaunchArgument("esdf_update_hz", default_value="5.0"),
        DeclareLaunchArgument("esdf_max_pending_blocks", default_value="16384"),
        DeclareLaunchArgument("publish_debug_pointcloud", default_value="false"),
        DeclareLaunchArgument("pointcloud_max_points", default_value="30000"),
        DeclareLaunchArgument("pointcloud_publish_period_ms", default_value="500"),
        DeclareLaunchArgument("publish_las2_preview", default_value="true"),
        DeclareLaunchArgument("enable_rtsp", default_value="false"),
        DeclareLaunchArgument("rtsp_input_topic", default_value="/camera/left/frame_desc"),
        DeclareLaunchArgument("rtsp_url", default_value="rtsp://0.0.0.0:8554/left"),
        DeclareLaunchArgument("enable_esdf_rtsp", default_value="false"),
        DeclareLaunchArgument("esdf_rtsp_url", default_value="rtsp://0.0.0.0:8555/esdf"),
        DeclareLaunchArgument("rtsp_fps", default_value="30"),
    ]


def reconstruction_node_options():
    """Return node options shared by launch and session parameter validation."""
    camera_config_file = LaunchConfiguration("camera_config_file")
    calibration_file = LaunchConfiguration("calibration_file")
    model_path = LaunchConfiguration("model_path")
    input_file = LaunchConfiguration("input_file")
    camera_ld_preload = LaunchConfiguration("camera_ld_preload")
    las2_ld_preload = LaunchConfiguration("las2_ld_preload")
    las2_instance_count = LaunchConfiguration("las2_instance_count")
    las2_threads_per_instance = LaunchConfiguration("las2_threads_per_instance")
    las2_warmup_iterations = LaunchConfiguration("las2_warmup_iterations")
    las2_core_affinity = LaunchConfiguration("las2_core_affinity")

    return [
        dict(
            condition=IfCondition(LaunchConfiguration("enable_camera")),
            package="camera_io",
            executable="camera_mpp_publisher",
            name="camera_io",
            output="screen",
            additional_env={"LD_PRELOAD": camera_ld_preload},
            parameters=[
                {
                    "config_file": camera_config_file,
                    "fps": LaunchConfiguration("camera_fps"),
                    "input_file": input_file,
                    "file_packet_reader": ParameterValue(
                        LaunchConfiguration("file_packet_reader"),
                        value_type=str,
                    ),
                    "file_start_ready_topic": LaunchConfiguration("file_start_ready_topic"),
                    "autostart": LaunchConfiguration("autostart_camera"),
                    "autostart_ready_topic": ParameterValue(
                        LaunchConfiguration("camera_start_ready_topic"), value_type=str
                    ),
                    "autostart_ready_timeout_sec": ParameterValue(
                        LaunchConfiguration("camera_start_ready_timeout_sec"), value_type=float
                    ),
                    "autostart_shutdown_on_failure": True,
                }
            ],
        ),
        dict(
            condition=IfCondition(LaunchConfiguration("enable_vio")),
            package="vio",
            executable="vio_node",
            name="vio",
            output="screen",
            sigterm_timeout=LaunchConfiguration("_localization_sigterm_timeout"),
            sigkill_timeout=LaunchConfiguration("_localization_sigkill_timeout"),
            prefix=LaunchConfiguration("_vio_node_prefix"),
            additional_env={"LD_PRELOAD": ""},
            parameters=[
                {
                    "spacemit_vslam_verbosity": ParameterValue(
                        LaunchConfiguration("spacemit_vslam_verbosity"), value_type=int
                    ),
                    "frame_record_file": LaunchConfiguration("vio_frame_record_file"),
                    "dma_audit_file": ParameterValue(
                        LaunchConfiguration("vio_dma_audit_file"),
                        value_type=str,
                    ),
                    "frame_record_queue_depth": LaunchConfiguration("vio_frame_record_queue_depth"),
                    "left_topic": LaunchConfiguration("left_topic"),
                    "right_topic": LaunchConfiguration("right_topic"),
                    "imu_topic": LaunchConfiguration("imu_topic"),
                    "camera_config_file": camera_config_file,
                    "camera_fps": LaunchConfiguration("camera_fps"),
                    "calibration_file": calibration_file,
                    "imu_calibration_file": LaunchConfiguration("imu_calibration_file"),
                    "image_width": LaunchConfiguration("image_width"),
                    "image_height": LaunchConfiguration("image_height"),
                    "image_pose_topic": LaunchConfiguration("image_pose_topic"),
                    "tracking_mode_topic": LaunchConfiguration("tracking_mode_topic"),
                    "imu_pose_topic": LaunchConfiguration("imu_pose_topic"),
                    "gravity_topic": LaunchConfiguration("gravity_topic"),
                    "use_imu": LaunchConfiguration("use_imu"),
                    "minimum_observation_depth_m": ParameterValue(
                        LaunchConfiguration("minimum_observation_depth_m"), value_type=float
                    ),
                    "imu_frequency_hz": LaunchConfiguration("imu_frequency_hz"),
                    "enable_external_propagation": LaunchConfiguration(
                        "enable_external_propagation",
                    ),
                    "image_queue_depth": LaunchConfiguration("image_queue_depth"),
                    "sync_queue_depth": LaunchConfiguration("sync_queue_depth"),
                    "asynchronous_processing": LaunchConfiguration("asynchronous_processing"),
                    "enable_stationary_initialization": ParameterValue(
                        LaunchConfiguration("enable_stationary_initialization"), value_type=bool
                    ),
                    "slam_enabled": LaunchConfiguration("slam_enabled"),
                    **{
                        name: ParameterValue(LaunchConfiguration(name), value_type=str)
                        for name in (
                            "slam_map_save_directory",
                            "slam_map_load_directory",
                            "slam_frame_record_file",
                            "slam_localization_audit_file",
                            "slam_local_frame",
                            "slam_map_frame",
                            "global_hint_topic",
                            "slam_status_topic",
                            "slam_ready_topic",
                        )
                    },
                    **{
                        name: ParameterValue(LaunchConfiguration(name), value_type=int)
                        for name in (
                            "localization_history_capacity",
                            "localization_history_max_bytes",
                            "slam_retention_time_ms",
                            "slam_maximum_pending_keyframes",
                            "slam_maximum_tracked_poses",
                        )
                    },
                    **{
                        name: ParameterValue(LaunchConfiguration(name), value_type=float)
                        for name in (
                            "slam_save_timeout_sec",
                            "slam_localization_horizontal_radius",
                            "slam_localization_vertical_radius",
                        )
                    },
                    "publish_map_to_odom_tf": ParameterValue(
                        LaunchConfiguration("publish_map_to_odom_tf"), value_type=bool
                    ),
                    "slam_sync_mode": ParameterValue(
                        LaunchConfiguration("slam_sync_mode"),
                        value_type=bool,
                    ),
                    "publish_pose_inputs": ParameterValue(
                        LaunchConfiguration("publish_pose_inputs"), value_type=bool
                    ),
                    "publish_frame_diagnostics": ParameterValue(
                        LaunchConfiguration("publish_frame_diagnostics"), value_type=bool
                    ),
                    "feature_topic": LaunchConfiguration("feature_topic"),
                    "pose_input_topic": LaunchConfiguration("pose_input_topic"),
                }
            ],
        ),
        dict(
            condition=IfCondition(LaunchConfiguration("enable_las2")),
            package="las2",
            executable="las2_node",
            prefix=LaunchConfiguration("_las2_node_prefix"),
            name="las2",
            output="screen",
            sigterm_timeout=LaunchConfiguration("_localization_sigterm_timeout"),
            sigkill_timeout=LaunchConfiguration("_localization_sigkill_timeout"),
            additional_env={"LD_PRELOAD": las2_ld_preload},
            parameters=[
                {
                    "input_topic": LaunchConfiguration("camera_full_topic"),
                    "pose_gate_topic": ParameterValue(
                        LaunchConfiguration("las2_pose_gate_topic"),
                        value_type=str,
                    ),
                    "pose_gate_image_depth": LaunchConfiguration("las2_pose_gate_image_depth"),
                    "output_topic": LaunchConfiguration("depth_topic"),
                    "ready_topic": LaunchConfiguration("las2_ready_topic"),
                    "preview_topic": LaunchConfiguration("las2_preview_topic"),
                    "publish_preview": LaunchConfiguration("publish_las2_preview"),
                    "preview_center_depth_overlay": ParameterValue(
                        LaunchConfiguration("las2_preview_center_depth_overlay"), value_type=bool
                    ),
                    "preview_depth_roi_size": ParameterValue(
                        LaunchConfiguration("las2_preview_depth_roi_size"), value_type=int
                    ),
                    "preview_depth_roi_min_valid_fraction": ParameterValue(
                        LaunchConfiguration("las2_preview_depth_roi_min_valid_fraction"),
                        value_type=float,
                    ),
                    "preview_visualization": LaunchConfiguration("las2_preview_visualization"),
                    "preview_disparity_min": LaunchConfiguration("las2_preview_disparity_min"),
                    "preview_disparity_max": LaunchConfiguration("las2_preview_disparity_max"),
                    "preview_depth_min_m": LaunchConfiguration("las2_preview_depth_min_m"),
                    "preview_depth_max_m": LaunchConfiguration("las2_preview_depth_max_m"),
                    "calibration_file": calibration_file,
                    "model_path": model_path,
                    "core_count": las2_instance_count,
                    "threads_per_instance": las2_threads_per_instance,
                    "warmup_iterations": las2_warmup_iterations,
                    "core_affinity": las2_core_affinity,
                    "queue_depth": 4,
                }
            ],
        ),
        dict(
            condition=IfCondition(LaunchConfiguration("enable_mapping")),
            package="esdf_drm",
            executable="esdf_drm_node",
            prefix=LaunchConfiguration("_esdf_drm_node_prefix"),
            name="esdf_drm",
            output="screen",
            sigterm_timeout=LaunchConfiguration("_localization_sigterm_timeout"),
            sigkill_timeout=LaunchConfiguration("_localization_sigkill_timeout"),
            additional_env={"LD_PRELOAD": ""},
            parameters=[
                {
                    "pose_topic": LaunchConfiguration("image_pose_topic"),
                    "world_frame": LaunchConfiguration("esdf_world_frame"),
                    "tracking_mode_topic": LaunchConfiguration("tracking_mode_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "pair_buffer_depth": LaunchConfiguration("pair_buffer_depth"),
                    "gravity_topic": LaunchConfiguration("gravity_topic"),
                    "preview_overlay_topic": LaunchConfiguration("preview_overlay_topic"),
                    "feature_panel_enabled": ParameterValue(
                        LaunchConfiguration("feature_panel_enabled"), value_type=bool
                    ),
                    "feature_panel_age_max_frames": ParameterValue(
                        LaunchConfiguration("feature_panel_age_max_frames"), value_type=int
                    ),
                    "feature_panel_image_topic": LaunchConfiguration("feature_panel_image_topic"),
                    "feature_panel_topic": LaunchConfiguration("feature_panel_topic"),
                    "mapping_enabled": LaunchConfiguration("mapping_enabled"),
                    "enable_gles_drm": LaunchConfiguration("enable_gles_drm"),
                    "headless_render": LaunchConfiguration("headless_render"),
                    "publish_render_output": LaunchConfiguration("publish_render_output"),
                    "render_output_topic": LaunchConfiguration("render_output_topic"),
                    "render_output_format": LaunchConfiguration("render_output_format"),
                    "record_enabled": LaunchConfiguration("record_enabled"),
                    "record_use_source_timestamps": LaunchConfiguration(
                        "record_use_source_timestamps",
                    ),
                    "record_timestamp_topic": LaunchConfiguration("record_timestamp_topic"),
                    "record_timestamp_use_frame_pts": LaunchConfiguration(
                        "record_timestamp_use_frame_pts",
                    ),
                    "record_timestamp_origin_zero": LaunchConfiguration(
                        "record_timestamp_origin_zero",
                    ),
                    "record_output_pattern": LaunchConfiguration("record_output_pattern"),
                    "record_fps": LaunchConfiguration("record_fps"),
                    "record_bitrate_bps": LaunchConfiguration("record_bitrate_bps"),
                    "record_gop": LaunchConfiguration("record_gop"),
                    "record_pool_depth": LaunchConfiguration("record_pool_depth"),
                    "record_venc_channel": LaunchConfiguration("record_venc_channel"),
                    "record_mux_channel": LaunchConfiguration("record_mux_channel"),
                    "record_wait_for_map": LaunchConfiguration("record_wait_for_map"),
                    "render_fps": LaunchConfiguration("render_fps"),
                    "display_width": LaunchConfiguration("display_width"),
                    "display_height": LaunchConfiguration("display_height"),
                    "render_max_blocks": LaunchConfiguration("render_max_blocks"),
                    "render_max_block_updates_per_frame": LaunchConfiguration(
                        "render_max_block_updates_per_frame",
                    ),
                    "render_max_points_per_block": LaunchConfiguration(
                        "render_max_points_per_block",
                    ),
                    "render_point_size": LaunchConfiguration("render_point_size"),
                    "render_slice_view_half_height_m": LaunchConfiguration(
                        "render_slice_view_half_height_m",
                    ),
                    "mouse_navigation_enabled": LaunchConfiguration("mouse_navigation_enabled"),
                    "mouse_input_device": LaunchConfiguration("mouse_input_device"),
                    "render_tsdf_interpolated_points": LaunchConfiguration(
                        "render_tsdf_interpolated_points",
                    ),
                    "render_measure_gpu_time": LaunchConfiguration("render_measure_gpu_time"),
                    "render_async_page_flip": LaunchConfiguration("render_async_page_flip"),
                    "render_frustum_culling": LaunchConfiguration("render_frustum_culling"),
                    "gravity_align_map": LaunchConfiguration("gravity_align_map"),
                    "display_gravity_imu_topic": LaunchConfiguration("display_gravity_imu_topic"),
                    "display_gravity_imu_rotation": LaunchConfiguration(
                        "display_gravity_imu_rotation",
                    ),
                    "display_gravity_imu_time_shift_s": ParameterValue(
                        LaunchConfiguration("display_gravity_imu_time_shift_s"), value_type=float
                    ),
                    "map_clearing_radius_m": LaunchConfiguration("map_clearing_radius_m"),
                    "clear_map_outside_radius_rate_hz": LaunchConfiguration(
                        "clear_map_outside_radius_rate_hz",
                    ),
                    "render_trajectory_duration_s": LaunchConfiguration(
                        "render_trajectory_duration_s",
                    ),
                    "initial_gravity_camera": LaunchConfiguration("initial_gravity_camera"),
                    "follow_slice_height": LaunchConfiguration("follow_slice_height"),
                    "slice_thickness_m": LaunchConfiguration("slice_thickness_m"),
                    "slice_height_offset_m": LaunchConfiguration("slice_height_offset_m"),
                    "slice_min_z_m": LaunchConfiguration("slice_min_z_m"),
                    "slice_max_z_m": LaunchConfiguration("slice_max_z_m"),
                    "slice_output_z_m": LaunchConfiguration("slice_output_z_m"),
                    "voxel_size_m": LaunchConfiguration("voxel_size_m"),
                    "pointcloud_min_weight": LaunchConfiguration("pointcloud_min_weight"),
                    "pointcloud_surface_band_m": LaunchConfiguration("pointcloud_surface_band_m"),
                    "truncation_distance_m": LaunchConfiguration("truncation_distance_m"),
                    "max_integration_distance_m": LaunchConfiguration("max_integration_distance_m"),
                    "projective_tsdf_integrator_invalid_depth_decay_factor": LaunchConfiguration(
                        "projective_tsdf_integrator_invalid_depth_decay_factor"
                    ),
                    "max_esdf_distance_m": LaunchConfiguration("max_esdf_distance_m"),
                    "depth_subsampling_factor": LaunchConfiguration("depth_subsampling_factor"),
                    "esdf_update_hz": LaunchConfiguration("esdf_update_hz"),
                    "esdf_max_pending_blocks": LaunchConfiguration("esdf_max_pending_blocks"),
                    "publish_debug_pointcloud": LaunchConfiguration("publish_debug_pointcloud"),
                    "pointcloud_max_points": ParameterValue(
                        LaunchConfiguration("pointcloud_max_points"), value_type=int
                    ),
                    "pointcloud_publish_period_ms": ParameterValue(
                        LaunchConfiguration("pointcloud_publish_period_ms"), value_type=int
                    ),
                }
            ],
        ),
        dict(
            condition=IfCondition(LaunchConfiguration("desktop_viewer_enabled")),
            package="esdf_drm",
            executable="esdf_desktop_viewer",
            name="esdf_desktop_viewer",
            output="screen",
            additional_env={"LD_PRELOAD": ""},
            parameters=[
                {
                    "input_topic": LaunchConfiguration("render_output_topic"),
                    "window_width": ParameterValue(
                        LaunchConfiguration("display_width"),
                        value_type=int,
                    ),
                    "window_height": ParameterValue(
                        LaunchConfiguration("display_height"),
                        value_type=int,
                    ),
                    "fullscreen": ParameterValue(
                        LaunchConfiguration("desktop_fullscreen"),
                        value_type=bool,
                    ),
                    "allow_software_rendering": ParameterValue(
                        LaunchConfiguration("desktop_allow_software_rendering"), value_type=bool
                    ),
                    "title": LaunchConfiguration("desktop_window_title"),
                }
            ],
        ),
        dict(
            condition=IfCondition(LaunchConfiguration("enable_rtsp")),
            package="mpp_rtsp",
            executable="mpp_rtsp_node",
            name="primary_rtsp",
            output="screen",
            additional_env={"LD_PRELOAD": camera_ld_preload},
            parameters=[
                {
                    "input_topic": LaunchConfiguration("rtsp_input_topic"),
                    "width": 0,
                    "height": 0,
                    "fps": LaunchConfiguration("rtsp_fps"),
                    "codec": "h264",
                    "bitrate": 4000000,
                    "gop": 30,
                    "rtsp_url": LaunchConfiguration("rtsp_url"),
                    "rtp_timestamp_source": "arrival",
                    "venc_channel": 0,
                    "mux_channel": 0,
                    "overlay_enabled": False,
                    "feature_overlay_enabled": False,
                }
            ],
        ),
        dict(
            condition=IfCondition(LaunchConfiguration("enable_esdf_rtsp")),
            package="mpp_rtsp",
            executable="mpp_rtsp_node",
            name="esdf_rtsp",
            output="screen",
            additional_env={"LD_PRELOAD": camera_ld_preload},
            parameters=[
                {
                    "input_topic": LaunchConfiguration("render_output_topic"),
                    "width": 0,
                    "height": 0,
                    "fps": LaunchConfiguration("rtsp_fps"),
                    "codec": "h264",
                    "bitrate": 5000000,
                    "gop": 30,
                    "rtsp_url": LaunchConfiguration("esdf_rtsp_url"),
                    "rtp_timestamp_source": "arrival",
                    "venc_channel": 1,
                    "mux_channel": 1,
                    "overlay_enabled": False,
                    "feature_overlay_enabled": False,
                }
            ],
        ),
    ]


def generate_launch_description():
    """Declare the pipeline and launch its enabled nodes in graph order."""
    return LaunchDescription([
        *reconstruction_arguments(),
        OpaqueFunction(function=configure_execution),
        *(Node(**options) for options in reconstruction_node_options()),
    ])
