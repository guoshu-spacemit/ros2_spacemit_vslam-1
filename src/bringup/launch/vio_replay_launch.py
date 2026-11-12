"""Replay a recorded MKV session through the shared VIO algorithm."""

import csv
import json
import math
from pathlib import Path
import runpy
import shutil
import struct

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _has_usable_imu_calibration(snapshot):
    """Check the scalar imu0 fields consumed by VIO in a combined snapshot."""
    try:
        calibration = yaml.safe_load(snapshot.read_text())
    except (UnicodeError, yaml.YAMLError):
        return False
    imu = calibration.get("imu0") if isinstance(calibration, dict) else None
    if not isinstance(imu, dict):
        return False
    fields = ("gyroscope_noise_density", "gyroscope_random_walk",
              "accelerometer_noise_density", "accelerometer_random_walk")
    if "update_rate" in imu:
        fields += ("update_rate",)
    for field in fields:
        raw = imu.get(field)
        if type(raw) not in (int, float, str):
            return False
        try:
            # VIO loads these YAML values as 32-bit floats.
            value, = struct.unpack("f", struct.pack("f", float(raw)))
        except (ValueError, OverflowError, struct.error):
            return False
        if not math.isfinite(value) or value <= 0:
            return False
    return True


def _load_replay_camera_config(session, output, mapping):
    """Validate recorded camera inputs and prepare the replay topics and streams."""
    required_inputs = [
        "camera0.mkv",
        "camera0_timestamps.txt",
        "vio_frames.csv",
        "vio_frames.csv.summary.json",
        "camera.json",
        "calibration.yaml",
        "manifest.json",
    ]
    for name in required_inputs:
        if not (session / name).is_file():
            raise RuntimeError(f"Missing recording input: {session / name}")
    camera = json.loads((session / "camera.json").read_text())
    if camera.get("timestamp", {}).get("source") not in ("embedded_exposure_start", "embedded_exposure_midpoint"):
        raise RuntimeError("Replay requires embedded exposure timestamps")
    if camera["camera"]["input_format"] != "mjpeg":
        raise RuntimeError("Session replay requires original MJPEG packets")
    with (session / "camera0_timestamps.txt").open() as stream:
        timestamps = csv.DictReader(
            (line for line in stream if not line.startswith("#")), delimiter=" "
        )
        stamps = [int(row["timestamp_ns"]) for row in timestamps]
    if len(stamps) > 1:
        intervals = [b - a for a, b in zip(stamps, stamps[1:])]
        if min(intervals) <= 0:
            raise RuntimeError("Camera index timestamps must strictly increase")
        # Preserve the recorded capture mode. Missing video packets or slower
        # playback must not change an embedded IMU's samples-per-frame rate.
    camera["calibration"]["file"] = str(output / "calibration.yaml")
    # The dedicated executable owns MKV input. Never change the MP4 launch.
    camera["input"] = {"type": "uvc"}
    for eye in ("left", "right"):
        camera["streams"][eye]["topic"] = f"/replay/camera/{eye}/frame_desc"
        camera["streams"][eye]["camera_info_topic"] = f"/replay/camera/{eye}/camera_info"
    for stream in ("full", "compressed"):
        camera["streams"].setdefault(stream, {})["enabled"] = False
    camera["streams"]["full"]["enabled"] = mapping
    camera["streams"]["full"]["topic"] = "/replay/camera/full/frame_desc"
    camera["imu"]["raw"]["dds_topic"] = "rt/replay/camera/imu/data_raw"
    camera["imu"]["raw"]["ros_topic"] = "/replay/camera/imu/data_raw"
    camera["imu"]["ahrs"]["enabled"] = False
    return camera


def _load_replay_parameters(context, session, camera):
    """Apply compatible recorded VIO parameters and the requested odometry mode."""
    eye = camera["streams"]["left"]
    shape = eye.get("output", eye["crop"])
    # Match the existing recording launch defaults; an explicit flat parameter
    # YAML can reproduce a recording made with different algorithm settings.
    parameters = {
        "image_width": shape["width"],
        "image_height": shape["height"],
        "use_imu": True,
        "imu_frequency_hz": 0.0,
        "minimum_observation_depth_m": 0.0,
        "enable_external_propagation": False,
        "image_queue_depth": 1,
        "sync_queue_depth": 2,
        "asynchronous_processing": True,
        "slam_enabled": False,
        "enable_stationary_initialization": False,
    }
    override = LaunchConfiguration("vio_parameters_file").perform(context)
    if not override and (session / "vio_parameters.yaml").is_file():
        override = str(session / "vio_parameters.yaml")
    if override:
        values = yaml.safe_load(Path(override).read_text())
        if not isinstance(values, dict):
            raise RuntimeError("vio_parameters_file must contain a flat parameter mapping")
        # Only the functional application's declared settings may be replayed.
        # An explicit replacement YAML can describe old recordings without editing them.
        contract = Path(get_package_share_directory("bringup")) / "launch/vio_las2_esdf_launch.py"
        node_options = runpy.run_path(str(contract))["reconstruction_node_options"]()
        vio_options = next(
            options for options in node_options if options["executable"] == "vio_node"
        )
        supported = set(parameters) | vio_options["parameters"][0].keys()
        unknown = set(values) - supported
        if unknown:
            raise RuntimeError(f"Unsupported recorded settings: {sorted(unknown)}; "
                               "provide a functional vio_parameters_file")
        parameters.update(values)
    mode = LaunchConfiguration("odometry_mode").perform(context)
    if mode != "recorded":
        parameters["use_imu"] = mode == "vio"
    if type(parameters.get("use_imu")) is not bool:
        raise RuntimeError("use_imu must be boolean")
    return parameters


def setup(context):
    """Validate recording inputs, write replay snapshots and return launch actions.

    The launch context supplies the input and output directories and replay
    options. Invalid or incompatible recording inputs raise an error before
    any replay nodes are started.
    """
    screen = LaunchConfiguration("enable_screen").perform(context).lower() == "true"
    mapping = screen and LaunchConfiguration("mapping_enabled").perform(context) == "true"
    realtime = LaunchConfiguration("replay_realtime").perform(context).lower() == "true"
    panel = LaunchConfiguration("feature_panel_enabled").perform(context).lower() == "true"
    record_screen = LaunchConfiguration("record_screen").perform(context).lower() == "true"
    if record_screen and not screen:
        raise RuntimeError("record_screen requires enable_screen:=true")
    fast_forward_sec = float(LaunchConfiguration("replay_fast_forward_sec").perform(context))
    if not math.isfinite(fast_forward_sec) or not 1e-9 <= fast_forward_sec <= 86400:
        raise RuntimeError("replay_fast_forward_sec must be positive and finite, in [1e-9, 86400] seconds")
    start_paused = LaunchConfiguration("replay_start_paused").perform(context).lower() == "true"
    session = Path(LaunchConfiguration("session_directory").perform(context)).resolve()
    output = Path(LaunchConfiguration("output_directory").perform(context)).resolve()
    camera = _load_replay_camera_config(session, output, mapping)
    config_path = output / "camera.json"
    parameters = _load_replay_parameters(context, session, camera)
    output.mkdir(parents=True, exist_ok=False)
    calibration_snapshot = output / "calibration.yaml"
    shutil.copyfile(session / "calibration.yaml", calibration_snapshot)
    config_path.write_text(json.dumps(camera, indent=2) + "\n")
    (output / "replay_contract.json").write_text(json.dumps({
        "version": 2, "replay_mode": "images_and_imu",
    }, indent=2) + "\n")
    parameters.update(
        {
            "left_topic": "/replay/camera/left/frame_desc",
            "right_topic": "/replay/camera/right/frame_desc",
            "imu_topic": "/replay/camera/imu/data_raw",
            "camera_config_file": str(config_path),
            "camera_fps": 0,
            "calibration_file": str(calibration_snapshot),
            "replay_manifest": str(session / "vio_frames.csv"),
            "replay_ack_topic": "/replay/vio/ack",
            "replay_ready_topic": "/replay/vio/ready",
            "replay_imu_ack_topic": "/replay/vio/imu_ack",
            "replay_pause_topic": "/replay/paused",
            "frame_record_file": str(output / "vio_frames.csv"),
            "image_pose_topic": "/replay/vio/pose/image",
            "imu_pose_topic": "/replay/vio/pose/imu",
            "gravity_topic": "/replay/vio/gravity",
            "tracking_mode_topic": "/replay/vio/tracking_mode",
            "trajectory_topic": "/replay/vio/trajectory",
            "feature_topic": "/replay/vio/features",
            "pose_input_topic": "/replay/vio/pose_inputs",
            "publish_frame_diagnostics": (
                screen and panel or LaunchConfiguration("publish_frame_diagnostics").perform(context).lower() == "true"
            ),
            "slam_pose_topic": "/replay/slam/pose/image",
            "slam_trajectory_topic": "/replay/slam/trajectory",
            "replay_timeout_sec": float(LaunchConfiguration("replay_timeout_sec").perform(context)),
        }
    )
    # A copied recording must use its own IMU snapshot, never an absolute path
    # retained from the machine on which it was captured.
    if (session / "imu.yaml").is_file():
        shutil.copyfile(session / "imu.yaml", output / "imu.yaml")
        parameters["imu_calibration_file"] = str(output / "imu.yaml")
    elif parameters["use_imu"] and _has_usable_imu_calibration(calibration_snapshot):
        parameters["imu_calibration_file"] = str(calibration_snapshot)
    else:
        parameters["imu_calibration_file"] = ""
    if not parameters["use_imu"]:
        # VO returns before acknowledging IMU. Do not let that unused
        # subscription satisfy the producer's receipt-peer discovery gate.
        # Only the receipt-only peer subscribes to the actual raw IMU stream.
        parameters["imu_topic"] = "/replay/vo/unused_imu"
    (output / "vio_parameters.yaml").write_text(yaml.safe_dump(parameters))
    (output / "source_session.txt").write_text(str(session) + "\n")
    actions = [
        LogInfo(msg="Replaying recorded images/IMU through the functional localization interface"),
        RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason="Replay pipeline child exited"))])),
        Node(
            package="vio",
            executable="vio_node",
            name="replay_vio",
            output="screen",
            additional_env={"LD_PRELOAD": ""},
            parameters=[parameters],
        ),
        Node(
            package="camera_io",
            executable="session_replay_publisher",
            name="session_replay",
            output="screen",
            additional_env={"LD_PRELOAD": LaunchConfiguration("camera_ld_preload")},
            parameters=[
                {
                    "config_file": str(config_path),
                    "session_directory": str(session),
                    "output_directory": str(output),
                    "autostart": True,
                    "replay_publish_full": mapping,
                    "replay_realtime": realtime,
                    "replay_display_ready_topic": "/replay/las2/ready" if mapping else "",
                    "replay_pause_topic": "/replay/paused",
                    "replay_start_paused": start_paused,
                    "replay_fast_forward_sec": fast_forward_sec,
                    "replay_timeout_sec": float(LaunchConfiguration("replay_timeout_sec").perform(context)),
                }
            ],
        ),
    ]
    if not parameters["use_imu"]:
        actions.append(
            Node(package="bringup", executable="replay_imu_receipt.py", name="replay_imu_receipt_only", output="screen")
        )

    display_gravity = "[0.0, 0.0, 0.0]"
    if mapping and not parameters["use_imu"]:
        from importlib.util import module_from_spec, spec_from_file_location

        spec = spec_from_file_location("replay_display_gravity", Path(__file__).with_name("replay_display_gravity.py"))
        module = module_from_spec(spec)
        spec.loader.exec_module(module)
        try:
            gravity = module.initial_camera_gravity(session)
        except (KeyError, ValueError, OSError, StopIteration) as exc:
            raise RuntimeError(
                "VO mapping needs initial display gravity: "
                + str(exc)
                + ". Use mapping_enabled:=false to inspect VO without a map."
            ) from exc
        (output / "display_gravity.json").write_text(json.dumps(gravity, indent=2) + "\n")
        display_gravity = json.dumps(gravity["gravity_initial_camera"])
        actions.append(
            LogInfo(msg="VO map orientation from initial quiet IMU only; odometry remains pure VO: " + display_gravity)
        )

    if screen:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(Path(get_package_share_directory("bringup")) / "launch/vio_las2_esdf_launch.py")
                ),
                launch_arguments={
                    "enable_camera": "false",
                    "enable_vio": "false",
                    "enable_las2": "true" if mapping else "false",
                    "mapping_enabled": "true" if mapping else "false",
                    "left_topic": "/replay/camera/left/frame_desc",
                    "preview_overlay_topic": "/replay/las2/preview/frame_desc"
                    if mapping
                    else "/replay/camera/left/frame_desc",
                    "tracking_mode_topic": "/replay/vio/tracking_mode",
                    "model_path": LaunchConfiguration("model_path"),
                    "calibration_file": str(output / "calibration.yaml"),
                    "camera_full_topic": "/replay/camera/full/frame_desc",
                    "depth_topic": "/replay/las2/depth_desc",
                    "las2_ready_topic": "/replay/las2/ready",
                    "las2_preview_topic": "/replay/las2/preview/frame_desc",
                    "feature_panel_enabled": LaunchConfiguration("feature_panel_enabled"),
                    "feature_panel_image_topic": "/replay/camera/left/frame_desc",
                    "feature_panel_topic": "/replay/vio/features",
                    "feature_panel_age_max_frames": LaunchConfiguration("feature_panel_age_max_frames"),
                    "record_enabled": "true" if record_screen else "false",
                    "record_output_pattern": str(output / "features_display_%Y%m%d_%H%M%S.mp4"),
                    "image_pose_topic": "/replay/vio/pose/image",
                    "gravity_topic": "/replay/vio/gravity",
                    "initial_gravity_camera": display_gravity,
                    "enable_mapping": "true",
                    "enable_gles_drm": "true",
                    "publish_las2_preview": "true",
                    "enable_rtsp": "false",
                    "enable_esdf_rtsp": "false",
                }.items(),
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_screen", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("mapping_enabled", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("odometry_mode", default_value="recorded", choices=["recorded", "vo", "vio"]),
            DeclareLaunchArgument(
                "replay_realtime",
                default_value=LaunchConfiguration("enable_screen"),
                choices=["true", "false"],
                description="Pace source timestamps; defaults to enable_screen",
            ),
            DeclareLaunchArgument("feature_panel_enabled", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument(
                "publish_frame_diagnostics",
                default_value="false",
                choices=["true", "false"],
                description="Collect diagnostics even without the screen",
            ),
            DeclareLaunchArgument("feature_panel_age_max_frames", default_value="14"),
            DeclareLaunchArgument("record_screen", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument("session_directory"),
            DeclareLaunchArgument("output_directory", description="New directory for replay results"),
            DeclareLaunchArgument("camera_ld_preload", default_value="libmpp.so"),
            DeclareLaunchArgument("vio_parameters_file", default_value=""),
            DeclareLaunchArgument("replay_timeout_sec", default_value="30.0"),
            DeclareLaunchArgument(
                "replay_start_paused",
                default_value="false",
                choices=["true", "false"],
                description="Wait for keyboard resume or step before the first target",
            ),
            DeclareLaunchArgument(
                "replay_fast_forward_sec",
                default_value="5.0",
                description="Source-time interval for each forward command; [1e-9, 86400] seconds",
            ),
            OpaqueFunction(function=setup),
        ]
    )
