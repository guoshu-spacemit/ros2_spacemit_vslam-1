#!/usr/bin/env python3
"""Snapshot a portable live VIO profile without opening any hardware."""

import argparse
import ast
import json
import math
from pathlib import Path
import sys

from calibration_bundle import (MANIFEST, load_json, mapping as require_mapping,
                                validate, write_manifest)


PROFILE_KEYS = {
    "camera_config_file", "calibration_file", "imu_calibration_file",
    "model_path", "launch_arguments", "recorder_parameters", "agc_parameters",
}
# These belong to the controller, not to an independently editable launch preset.
OWNED_ARGUMENTS = {
    "camera_config_file", "calibration_file", "imu_calibration_file", "model_path",
    "camera_fps", "input_file", "enable_camera", "autostart_camera", "enable_vio",
    "enable_las2", "mapping_enabled", "enable_mapping", "enable_gles_drm",
    "headless_render", "slam_enabled", "record_enabled", "publish_render_output",
    "enable_rtsp", "enable_esdf_rtsp", "vio_frame_record_file",
    "left_topic", "right_topic", "camera_full_topic",
    "imu_topic", "use_imu",
}


def validate_launch_arguments(options, *, extra_launch_files=()):
    """Reject unsupported profile settings before creating a session."""
    script = Path(__file__).resolve()
    candidates = (script.parent.parent / "launch/vio_las2_esdf_launch.py",
                  script.parents[2] / "share/bringup/launch/vio_las2_esdf_launch.py")
    path = next((item for item in candidates if item.is_file()), None)
    if path is None:
        raise ValueError("The installed reconstruction launch contract is missing")
    declarations = set()
    for contract in (path, *(path.with_name(name) for name in extra_launch_files)):
        if not contract.is_file():
            raise ValueError(f"The installed launch contract is missing: {contract.name}")
        for node in ast.walk(ast.parse(contract.read_text())):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                    and node.func.id == "DeclareLaunchArgument" and node.args
                    and isinstance(node.args[0], ast.Constant)):
                declarations.add(node.args[0].value)
    unknown = set(options) - declarations
    if unknown:
        raise ValueError(f"Unsupported launch settings: {sorted(unknown)}")


def launch_text(value):
    if isinstance(value, bool):
        return str(value).lower()
    if not isinstance(value, (str, int, float)):
        raise ValueError("launch_arguments values must be strings, numbers or booleans")
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("Non-finite launch argument")
    return str(value)


def resource(profile_dir, value, name, optional=False):
    if optional and not value:
        return None
    if not isinstance(value, str) or not value:
        raise ValueError(f"Missing {name}")
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = profile_dir / path
    path = path.resolve(strict=True)
    if not path.is_file():
        raise ValueError(f"Not a file: {path}")
    return path


def agc_options(profile, camera):
    """Validate the opt-in packed-stereo controller before any hardware opens."""
    supplied = profile.get("agc_parameters", {})
    defaults = {"exposure_raw": -1, "target_y": 80.0,
                "metadata_width": 0, "embedded_metadata": False}
    if not isinstance(supplied, dict) or set(supplied) - set(defaults):
        raise ValueError("Unknown agc_parameters or invalid object")
    values = {**defaults, **supplied}
    raw, target = values["exposure_raw"], values["target_y"]
    width, embedded = values["metadata_width"], values["embedded_metadata"]
    if type(raw) is not int or not 0 < raw <= 2147483647:
        raise ValueError("AGC requires explicit positive exposure_raw (device raw units)")
    if type(target) not in (float, int) or not math.isfinite(target) or not 1 <= target <= 254:
        raise ValueError("AGC target_y must be finite and in [1,254]")
    values["target_y"] = float(target)
    if type(width) is not int or width < 0 or type(embedded) is not bool:
        raise ValueError("AGC metadata_width/embedded_metadata are invalid")
    source = camera["camera"]
    w, h = source.get("width", 0), source.get("height", 0)
    if (type(w) is not int or type(h) is not int or w - width < 20
            or (w - width) % 2 or h < 10):
        raise ValueError("AGC requires two equal side-by-side eyes after metadata_width")
    if embedded and (width != 160 or h < 768):
        raise ValueError("AGC embedded parser requires native 160px metadata and height >= 768")
    full = camera["streams"].get("full", {})
    if not full.get("topic") or full.get("enabled") is False or source.get("decode") is False:
        raise ValueError("AGC requires an enabled decoded full-image stream")
    if camera.get("timestamp", {}).get("gpio", {}).get("startup_alignment", {}).get("enabled"):
        raise ValueError("AGC cannot share exposure ownership with GPIO startup alignment")
    if not isinstance(source.get("device"), str) or not source["device"]:
        raise ValueError("AGC requires camera.device")
    return {**values, "device": source["device"], "topic": full["topic"]}


def prepare(profile_path, directory, *, mapping, slam, record, fps, domain,
            agc=False,
            features=None, mode="auto"):
    """Create a new session; preserve the original camera clock/rate settings."""
    if fps not in (30, 60) or not 0 <= domain <= 232:
        raise ValueError("Expected fps 30/60 and ROS_DOMAIN_ID 0..232")
    if mode not in ("auto", "vo", "vio"):
        raise ValueError("mode must be auto, vo or vio")
    profile_path = Path(profile_path).resolve(strict=True)
    profile = load_json(profile_path.read_bytes())
    if not isinstance(profile, dict) or set(profile) - PROFILE_KEYS:
        raise ValueError("Unknown profile keys or invalid profile object")
    overrides = profile.get("launch_arguments", {})
    if not isinstance(overrides, dict):
        raise ValueError("launch_arguments must be an object")
    conflicts = OWNED_ARGUMENTS.intersection(overrides)
    if conflicts:
        raise ValueError(f"Controller-owned launch arguments: {sorted(conflicts)}")
    validate_launch_arguments(overrides)
    options = {key: launch_text(value) for key, value in overrides.items()}
    if features is not None:
        if type(features) is not bool:
            raise ValueError("features must be boolean or unspecified")
        options["feature_panel_enabled"] = launch_text(features)
        options["publish_frame_diagnostics"] = launch_text(features)
    recorder_parameters = {
        "camera_write_queue_depth": 128,
        "camera_write_buffer_bytes": 2097152,
    }
    supplied = profile.get("recorder_parameters", {})
    if not isinstance(supplied, dict) or set(supplied) - set(recorder_parameters):
        raise ValueError("Unknown recorder_parameters or invalid object")
    recorder_parameters.update(supplied)
    for key, low, high in (("camera_write_queue_depth", 0, 128),
                           ("camera_write_buffer_bytes", 4, 67108864)):
        value = recorder_parameters[key]
        if type(value) is not int or not low <= value <= high:
            raise ValueError(f"{key} must be an integer in [{low}, {high}]")
    camera_path = resource(profile_path.parent, profile.get("camera_config_file"),
                           "camera_config_file")
    calibration = resource(profile_path.parent, profile.get("calibration_file"),
                           "calibration_file")
    imu_calibration = resource(profile_path.parent, profile.get("imu_calibration_file"),
                               "imu_calibration_file", optional=True)
    model = (resource(profile_path.parent, profile.get("model_path"), "model_path")
             if mapping else None)
    camera_source = camera_path.read_bytes()
    calibration_data = calibration.read_bytes()
    imu_data = imu_calibration.read_bytes() if imu_calibration else None
    camera = load_json(camera_source)
    if require_mapping(camera.get("input", {}), "input").get("type", "uvc") != "uvc":
        raise ValueError("Live UVC input required; use the replay launch for files")
    if require_mapping(camera.get("camera"), "camera").get("input_format") != "mjpeg":
        raise ValueError("This live recorder profile requires original MJPEG input")
    streams = require_mapping(camera.get("streams"), "streams")
    for eye in ("left", "right"):
        stream = require_mapping(streams.get(eye), f"streams.{eye}")
        if not stream.get("topic") or stream.get("enabled") is False:
            raise ValueError(f"Missing enabled {eye} stream")
    if mapping and (not streams.get("full", {}).get("topic") or
                    streams["full"].get("enabled") is False):
        raise ValueError("Mapping requires an enabled packed full-image stream")
    compressed = streams.setdefault("compressed", {})
    if record and not compressed.get("topic"):
        raise ValueError("Recording requires streams.compressed.topic")
    imu = require_mapping(camera.get("imu", {}), "imu")
    sensor_imu = imu.get("enabled", False)
    if type(sensor_imu) is not bool:
        raise ValueError("imu.enabled must be boolean")
    if mode == "vio" and not sensor_imu:
        raise ValueError("VIO requires an enabled camera IMU")
    use_imu = sensor_imu and mode != "vo"
    imu_topic = imu.get("raw", {}).get("ros_topic", "")
    if sensor_imu and not imu_topic:
        raise ValueError("Embedded IMU requires imu.raw.ros_topic")
    agc_parameters = agc_options(profile, camera) if agc else {}
    validate(camera, calibration_data, imu_data, use_imu=use_imu)
    # Geometry overrides must agree before creating any output directory.
    options.setdefault("imu_frequency_hz", "0.0")
    # Display cadence is independent of the camera input rate.
    options.setdefault("render_fps", "60.0")
    for axis in ("width", "height"):
        output_size = streams["left"].get("output", streams["left"]["crop"])[axis]
        key = f"image_{axis}"
        if key in options and options[key] != str(output_size):
            raise ValueError(f"{key} must match the camera stream output: {output_size}")
        options[key] = str(output_size)
    directory = Path(directory).resolve()
    # No overwrite, including a previously failed prepare attempt.
    directory.mkdir(parents=True, exist_ok=False)
    (directory / "calibration.yaml").write_bytes(calibration_data)
    if imu_data is not None:
        (directory / "imu.yaml").write_bytes(imu_data)
    (directory / "camera.source.json").write_bytes(camera_source)
    camera["camera"]["fps"] = fps
    camera.setdefault("calibration", {})["file"] = "calibration.yaml"
    if sensor_imu:
        imu["domain_id"] = domain
    compressed["enabled"] = record
    # Do not invent samples_per_frame or a nominal frequency: the input profile
    # owns that sensor-specific relationship, including fixed-rate IMUs.
    options.update({
        "camera_config_file": str(directory / "camera.json"),
        "calibration_file": str(directory / "calibration.yaml"),
        "imu_calibration_file": str(directory / "imu.yaml") if imu_calibration else "",
        "model_path": str(model) if model else "", "input_file": "",
        "camera_fps": str(fps), "slam_enabled": launch_text(slam),
        "enable_las2": launch_text(mapping), "mapping_enabled": launch_text(mapping),
        "enable_camera": "true", "autostart_camera": "true", "enable_vio": "true",
        "enable_mapping": "true", "enable_gles_drm": "true", "headless_render": "false",
        "record_enabled": "false", "publish_render_output": "false",
        "enable_rtsp": "false", "enable_esdf_rtsp": "false",
        "vio_frame_record_file": str(directory / "vio_frames.csv"),
        "use_imu": launch_text(use_imu),
        "left_topic": streams["left"]["topic"], "right_topic": streams["right"]["topic"],
        "camera_full_topic": streams.get("full", {}).get("topic", ""),
        "imu_topic": imu_topic,
    })
    session = {"source": "live", "mode": "vio" if use_imu else "vo",
               "mapping": mapping, "slam": slam, "record": record,
               "agc": agc, "agc_parameters": agc_parameters,
               "fps": fps, "domain": domain, "profile": str(profile_path),
               "calibration_manifest": MANIFEST,
               "recorder_parameters": recorder_parameters}
    recorder = {
        "output_dir": str(directory.parent),
        "cameras": [{"name": "camera0", "topic": compressed.get("topic", ""),
                     "codec": "mjpeg"}],
        "imus": [{"name": "camera0", "topic": imu_topic, "enabled": True,
                  "queue_depth": 2048}] if sensor_imu else [],
    }
    for name, value in (("session.json", session), ("camera.json", camera),
                        ("profile.json", profile), ("launch-options.json", options),
                        ("recorder.json", recorder)):
        (directory / name).write_text(json.dumps(value, indent=2) + "\n")
    write_manifest(directory, use_imu=use_imu, calibration_source=calibration)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", type=Path)
    parser.add_argument("directory", type=Path)
    for switch in ("mapping", "slam", "record", "agc"):
        parser.add_argument(f"--{switch}", choices=("on", "off"), default="off")
    parser.add_argument("--features", choices=("on", "off"))
    parser.add_argument("--mode", choices=("auto", "vo", "vio"), default="auto")
    parser.add_argument("--fps", type=int, choices=(30, 60), default=30)
    parser.add_argument("--domain", type=int, default=0)
    args = parser.parse_args()
    try:
        prepare(args.profile, args.directory, mapping=args.mapping == "on",
                slam=args.slam == "on", record=args.record == "on",
                fps=args.fps, domain=args.domain,
                agc=args.agc == "on",
                features=None if args.features is None else args.features == "on",
                mode=args.mode)
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(1, f"Session configuration failed: {error}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
