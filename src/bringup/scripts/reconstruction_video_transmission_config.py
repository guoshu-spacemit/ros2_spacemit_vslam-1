#!/usr/bin/env python3
"""Resolve and snapshot one live reconstruction/FPV deployment profile."""

import argparse
import ast
import ipaddress
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys

import yaml

from vio_session_config import validate_launch_arguments


PROFILE_KEYS = {
    "camera_config_file", "calibration_file", "imu_calibration_file", "model_path",
    "video_config_file", "camera_device", "camera_fps", "timestamp_source",
    "ros_domain_id", "cpu_affinity", "headless_render", "launch_arguments",
    "runtime_environment", "network", "private_mpp_ipc", "odometry_mode",
}
OWNED_ARGUMENTS = {
    "camera_config_file", "calibration_file", "imu_calibration_file", "model_path",
    "video_config_file", "camera_fps", "input_file", "enable_camera", "enable_vio",
    "enable_las2", "enable_mapping", "mapping_enabled", "enable_gles_drm",
    "autostart_camera", "headless_render", "publish_render_output", "enable_rtsp",
    "enable_esdf_rtsp", "record_enabled", "display_width", "display_height",
    "render_fps", "video_fps", "left_topic", "right_topic", "camera_full_topic",
    "imu_topic", "use_imu", "display_gravity_imu_topic",
    "display_gravity_imu_rotation", "display_gravity_imu_time_shift_s",
}
PATH_ENVIRONMENT = {
    "LD_LIBRARY_PATH", "MPP_PLUGIN_DIR", "__EGL_VENDOR_LIBRARY_FILENAMES",
    "GBM_BACKENDS_PATH", "LIBGL_DRIVERS_PATH",
}


def resource(directory, value, name, optional=False):
    if optional and not value:
        return None
    if not isinstance(value, str) or not value or "\n" in value or "\0" in value:
        raise ValueError(f"Missing {name}")
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = directory / path
    return path.resolve(strict=True)


def launch_text(value):
    if isinstance(value, bool):
        return str(value).lower()
    if not isinstance(value, (str, int, float)):
        raise ValueError("Launch values must be strings, numbers or booleans")
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("Non-finite launch argument")
    if "\n" in str(value) or "\0" in str(value):
        raise ValueError("Newlines/NUL are not allowed in launch arguments")
    return str(value)


def display_imu_alignment(calibration_path):
    """Read measured IMU-to-camera rotation and Kalibr time shift, not a pose prior."""
    document = yaml.safe_load(calibration_path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not isinstance(document.get("cam0"), dict):
        raise ValueError("VO display gravity requires Kalibr cam0")
    calibration = document["cam0"]
    transform = calibration.get("T_cam_imu")
    if (not isinstance(transform, list) or len(transform) != 4
            or any(not isinstance(row, list) or len(row) != 4 for row in transform)):
        raise ValueError("VO display gravity requires 4x4 cam0.T_cam_imu")
    try:
        transform = [[float(value) for value in row] for row in transform]
        shift = float(calibration.get("timeshift_cam_imu", 0.0))
    except (ValueError, TypeError) as error:
        raise ValueError("Invalid Kalibr display gravity alignment") from error
    if (not all(math.isfinite(value) for row in transform for value in row)
            or any(abs(value - expected) > 1e-6
                   for value, expected in zip(transform[3], [0, 0, 0, 1]))):
        raise ValueError("Invalid homogeneous cam0.T_cam_imu")
    rotation = [row[:3] for row in transform[:3]]
    determinant = sum(
        rotation[0][i] * (rotation[1][(i + 1) % 3] * rotation[2][(i + 2) % 3]
                          - rotation[1][(i + 2) % 3] * rotation[2][(i + 1) % 3])
        for i in range(3))
    if abs(determinant - 1.0) > 1e-3:
        raise ValueError("cam0.T_cam_imu rotation is not proper")
    for row in range(3):
        for col in range(3):
            product = sum(rotation[row][i] * rotation[col][i] for i in range(3))
            if abs(product - float(row == col)) > 1e-3:
                raise ValueError("cam0.T_cam_imu rotation is not orthonormal")
    if not math.isfinite(shift) or abs(shift) > 1.0:
        raise ValueError("Display gravity requires finite timeshift_cam_imu in [-1,1] seconds")
    return [value for row in rotation for value in row], shift


def needs_display_imu(odometry_mode, options):
    if odometry_mode != "vo":
        return False
    align = options.get("gravity_align_map", "true").lower()
    if align not in ("true", "false", "1", "0"):
        raise ValueError("gravity_align_map must be boolean")
    seed = yaml.safe_load(options.get("initial_gravity_camera", "[0.0, 0.0, 0.0]"))
    if (not isinstance(seed, list) or len(seed) != 3
            or any(type(value) not in (int, float) or not math.isfinite(value)
                   for value in seed)):
        raise ValueError("initial_gravity_camera must have three finite numbers")
    norm_squared = sum(value * value for value in seed)
    if not (norm_squared == 0.0 or 1.0 <= norm_squared <= 400.0):
        raise ValueError("initial_gravity_camera must be zero or have norm 1..20")
    return align in ("true", "1") and norm_squared == 0.0


def find_camera():
    """Prefer a stable by-id path for the known packed-stereo UVC interface."""
    for node in sorted(Path("/sys/class/video4linux").glob("video*")):
        try:
            name = (node / "name").read_text(encoding="utf-8")
            index = (node / "index").read_text(encoding="utf-8").strip()
        except OSError:
            continue
        if "DECXIN" in name and index == "0":
            device = Path("/dev") / node.name
            for link in sorted(Path("/dev/v4l/by-id").glob("*")):
                if link.resolve() == device:
                    return str(link)
            return str(device)
    return None


def select_usb_interface(selector, sysfs_root=Path("/sys")):
    """Resolve an explicit USB identity, independent of a MAC-based ifname."""
    required = {"driver", "usb_vendor_id", "usb_product_id"}
    if not isinstance(selector, dict) or set(selector) != required:
        raise ValueError("interface_selector requires driver, usb_vendor_id and usb_product_id")
    driver = selector["driver"]
    vendor = selector["usb_vendor_id"]
    product = selector["usb_product_id"]
    if not isinstance(driver, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", driver):
        raise ValueError("Invalid interface_selector.driver")
    if any(not isinstance(value, str) or not re.fullmatch(r"[0-9a-fA-F]{4}", value)
           for value in (vendor, product)):
        raise ValueError("USB vendor/product IDs must be four hexadecimal digits")
    sysfs_root = Path(sysfs_root).resolve(strict=True)
    matches = []
    for interface in sorted((sysfs_root / "class/net").glob("*")):
        # Even an erroneous selector must never select a wireless interface.
        if (interface / "wireless").exists() or (interface / "phy80211").exists():
            continue
        try:
            device = (interface / "device").resolve(strict=True)
            actual_driver = (device / "driver").resolve(strict=True).name
        except OSError:
            continue
        if actual_driver != driver:
            continue
        for ancestor in (device, *device.parents):
            if not ancestor.is_relative_to(sysfs_root):
                break
            try:
                actual_vendor = (ancestor / "idVendor").read_text().strip().lower()
                actual_product = (ancestor / "idProduct").read_text().strip().lower()
            except OSError:
                continue
            if actual_vendor == vendor.lower() and actual_product == product.lower():
                matches.append(interface.name)
            # The nearest USB ancestor identifies this network function.
            break
    if len(matches) != 1:
        raise ValueError(f"USB interface_selector matched {len(matches)} interfaces: {matches}; "
                         "connect exactly one matching FPV device")
    return matches[0]


def network_plan(profile, video_config, *, snapshot_dir=None, sysfs_root=Path("/sys")):
    """Resolve the session's FPV network settings from the deployment files.

    Args:
        profile: Session profile containing optional network overrides.
        video_config: Path to the FPV transmitter's YAML configuration.
        snapshot_dir: New directory for rewritten YAML when selecting a USB
            interface. The source YAML files remain read-only.
        sysfs_root: Sysfs root used for USB interface discovery.

    Returns:
        A dictionary with the interface, address, prefix length, namespace and
        permission to configure the link. USB selection also records the path
        to the snapshot's transmitter configuration.

    Raises:
        ValueError: If network overrides or interface selection are invalid.
        OSError: If configuration files cannot be read or written.
    """

    settings = profile.get("network", {})
    if not isinstance(settings, dict) or set(settings) - {
            "configure", "prefix_length", "interface_selector"}:
        raise ValueError("Unknown network settings")
    configure = settings.get("configure", False)
    prefix = settings.get("prefix_length", 24)
    if type(configure) is not bool or type(prefix) is not int or not 0 <= prefix <= 32:
        raise ValueError("network requires boolean configure and prefix_length in [0,32]")
    sky = yaml.safe_load(video_config.read_text(encoding="utf-8"))
    profile_path = resource(video_config.parent, sky["deployment_profile"], "deployment_profile")
    connections_path = resource(video_config.parent, sky["connections_file"], "connections_file")
    deployment = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    connections = yaml.safe_load(connections_path.read_text(encoding="utf-8"))
    connection = connections["connections"]["sky"]
    binding = deployment.get("bindings", {}).get(connection["connection_id"], {})
    address = str(ipaddress.IPv4Address(connection["device_ip"]))
    interface = binding.get("interface", "")
    netns = binding.get("netns", "")
    if "interface_selector" in settings:
        if netns:
            raise ValueError("USB interface_selector requires a host interface without netns")
        if connection["connection_id"] not in deployment.get("bindings", {}):
            raise ValueError("USB interface_selector requires an existing sky SDK binding")
        interface = select_usb_interface(settings["interface_selector"], sysfs_root)
        if snapshot_dir is None:
            raise ValueError("USB interface_selector requires a session FPV snapshot directory")
        snapshot_dir = Path(snapshot_dir)
        snapshot_dir.mkdir(parents=True, exist_ok=False)
        deployment["bindings"][connection["connection_id"]]["interface"] = interface
        sky["deployment_profile"] = "profile.yaml"
        sky["connections_file"] = "connections.yaml"
        for name, contents in (("sky.yaml", sky), ("connections.yaml", connections),
                               ("profile.yaml", deployment)):
            (snapshot_dir / name).write_text(
                yaml.safe_dump(contents, sort_keys=False), encoding="utf-8")
    if configure and (not interface or netns):
        raise ValueError("Network configure requires explicit FPV interface and no netns")
    result = {"configure": configure, "address": address, "prefix_length": prefix,
              "interface": interface, "netns": netns}
    if "interface_selector" in settings:
        result["interface_selector"] = settings["interface_selector"]
        result["video_config_file"] = str(snapshot_dir / "sky.yaml")
    return result


def check_network(directory, configure=False):
    """Check the recorded FPV plan and optionally prepare the interface.

    Args:
        directory: Session directory containing the prepared network plan.
        configure: Allow configuration when the plan also enables it.
            Configuration saves the interface state to network-before.json,
            raises the link if needed and adds the planned address if missing.

    Raises:
        ValueError: If the link or address does not satisfy the plan.
        subprocess.CalledProcessError: If querying or configuring ip fails.
        OSError: If the session files or ip command are unavailable.
    """
    plan = json.loads((directory / "session.json").read_text())["network"]
    if plan["netns"]:
        print(f"FPV namespace {plan['netns']}: prepare it externally")
        return
    result = subprocess.run(["ip", "-j", "address", "show"], check=True,
                            capture_output=True, text=True)
    devices = json.loads(result.stdout)
    interface = plan["interface"]
    if not interface:
        present = any(info.get("local") == plan["address"] for device in devices
                      for info in device["addr_info"])
        print(f"FPV local address {plan['address']} assigned={present}; "
              "interface left to SDK routing")
        if configure and not present:
            raise ValueError("Prepare the FPV local address or set an explicit SDK interface")
        return
    selected = next((device for device in devices if device["ifname"] == interface), None)
    if selected is None:
        raise ValueError(f"FPV interface not present: {interface}")
    owners = [device["ifname"] for device in devices
              if any(info.get("local") == plan["address"] for info in device["addr_info"])]
    if any(owner != interface for owner in owners):
        raise ValueError(f"FPV local address already belongs to another interface: {owners}")
    present = interface in owners
    up = "UP" in selected.get("flags", [])
    print(f"FPV interface={interface} IP={plan['address']}/{plan['prefix_length']} "
          f"assigned={present} link_up={up} configure={plan['configure']}")
    if not configure:
        return
    if (not present or not up) and not plan["configure"]:
        raise ValueError("Prepare the FPV local address/link, or explicitly enable "
                         "network.configure")
    (directory / "network-before.json").write_text(
        json.dumps(selected, indent=2) + "\n", encoding="utf-8")
    if not up:
        subprocess.run(["ip", "link", "set", "dev", interface, "up"], check=True)
    if not present:
        subprocess.run(["ip", "address", "add",
                        f"{plan['address']}/{plan['prefix_length']}", "dev", interface], check=True)
    print("FPV link prepared; existing addresses and default route retained")


def prepare(profile_path, directory, *, sysfs_root=Path("/sys")):
    """Resolve the deployment profile and write the session's configuration files."""
    profile_path = Path(profile_path).resolve(strict=True)
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    if not isinstance(profile, dict) or set(profile) - PROFILE_KEYS:
        raise ValueError("Unknown profile keys or invalid profile object")
    resources = {}
    for name in ("camera_config_file", "calibration_file", "imu_calibration_file",
                 "model_path", "video_config_file"):
        resources[name] = resource(profile_path.parent, profile.get(name), name,
                                   optional=name == "imu_calibration_file")
        if resources[name] and not resources[name].is_file():
            raise ValueError(f"Not a file: {resources[name]}")
    fps = profile.get("camera_fps", 30)
    odometry_mode = profile.get("odometry_mode", "vio")
    if odometry_mode not in ("vo", "vio"):
        raise ValueError("odometry_mode must be vo or vio")
    domain = profile.get("ros_domain_id", 0)
    if type(fps) is not int or fps not in (30, 60):
        raise ValueError("camera_fps must be 30 or 60")
    if type(domain) is not int or not 0 <= domain <= 232:
        raise ValueError("ros_domain_id must be in [0,232]")
    headless = profile.get("headless_render", True)
    if type(headless) is not bool:
        raise ValueError("headless_render must be boolean")
    private_mpp_ipc = profile.get("private_mpp_ipc", True)
    if type(private_mpp_ipc) is not bool:
        raise ValueError("private_mpp_ipc must be boolean")
    affinity = profile.get("cpu_affinity", "0-7")
    if not isinstance(affinity, str) or not affinity:
        raise ValueError("cpu_affinity must be a nonempty CPU list")
    overrides = profile.get("launch_arguments", {})
    if not isinstance(overrides, dict) or OWNED_ARGUMENTS.intersection(overrides):
        raise ValueError("Invalid or controller-owned launch_arguments")
    validate_launch_arguments(overrides, extra_launch_files=(
        "reconstruction_video_transmission_launch.py",))
    options = {key: launch_text(value) for key, value in overrides.items()}
    display_gravity_from_imu = needs_display_imu(odometry_mode, options)
    if display_gravity_from_imu:
        rotation, shift = display_imu_alignment(resources["calibration_file"])
        options["display_gravity_imu_rotation"] = json.dumps(rotation)
        options["display_gravity_imu_time_shift_s"] = str(shift)
    environment = profile.get("runtime_environment", {})
    if not isinstance(environment, dict) or set(environment) - PATH_ENVIRONMENT - {"GBM_BACKEND"}:
        raise ValueError("Unknown runtime_environment variable")
    resolved_environment = {}
    for key, value in environment.items():
        if not isinstance(value, str) or not value or "\n" in value or "\0" in value:
            raise ValueError(f"Invalid runtime_environment.{key}")
        if key in PATH_ENVIRONMENT:
            value = ":".join(str(resource(profile_path.parent, entry, key))
                             for entry in value.split(":"))
        resolved_environment[key] = value

    camera = json.loads(resources["camera_config_file"].read_text(encoding="utf-8"))
    if camera.get("input", {}).get("type", "uvc") != "uvc":
        raise ValueError("Live UVC input required")
    timestamp = profile.get("timestamp_source", camera.get("timestamp", {}).get("source"))
    if timestamp not in ("embedded_exposure_start", "embedded_exposure_midpoint"):
        raise ValueError("Select the calibration's embedded exposure timestamp convention")
    if not camera.get("metadata", {}).get("enabled", False):
        raise ValueError("Embedded exposure timestamps require metadata")
    streams = camera["streams"]
    for name in ("left", "right", "full"):
        if not streams[name].get("topic") or streams[name].get("enabled") is False:
            raise ValueError(f"Missing enabled {name} descriptor stream")
    imu = camera.get("imu", {})
    imu_topic = imu.get("raw", {}).get("ros_topic", "")
    if odometry_mode == "vio" or display_gravity_from_imu:
        if (imu.get("enabled") is not True or not isinstance(imu_topic, str)
                or not imu_topic.strip()):
            raise ValueError("An enabled embedded IMU raw topic is required for "
                             "VIO or measured display gravity")
    options["display_gravity_imu_topic"] = imu_topic if display_gravity_from_imu else ""
    if "camera_device" in profile:
        camera["camera"]["device"] = (profile["camera_device"] or find_camera()
                                      or camera["camera"].get("device", ""))
    if not isinstance(camera["camera"].get("device"), str):
        raise ValueError("camera.device must be a path")

    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    network = network_plan(profile, resources["video_config_file"],
                           snapshot_dir=directory / "fpv", sysfs_root=sysfs_root)
    shutil.copyfile(resources["calibration_file"], directory / "calibration.yaml")
    if resources["imu_calibration_file"]:
        shutil.copyfile(resources["imu_calibration_file"], directory / "imu.yaml")
    camera["camera"]["fps"] = fps
    camera["timestamp"] = {**camera.get("timestamp", {}), "source": timestamp}
    camera.setdefault("calibration", {})["file"] = str(directory / "calibration.yaml")
    imu["domain_id"] = domain
    # Preserve physical eye mapping, IMU sample-rate model, and all calibration
    # transforms. Do not inject a camera/IMU time offset or inferred rate.
    for axis in ("width", "height"):
        size = streams["left"].get("output", {}).get(axis)
        if size is not None:
            options.setdefault(f"image_{axis}", str(size))
    options.update({
        "camera_config_file": str(directory / "camera.json"),
        "calibration_file": str(directory / "calibration.yaml"),
        "imu_calibration_file": (str(directory / "imu.yaml")
                                 if resources["imu_calibration_file"] else ""),
        "model_path": str(resources["model_path"]),
        "video_config_file": network.get("video_config_file", str(resources["video_config_file"])),
        "camera_fps": str(fps), "input_file": "", "use_imu": launch_text(odometry_mode == "vio"),
        "enable_camera": "true", "autostart_camera": "true", "enable_vio": "true",
        "enable_las2": "true", "enable_mapping": "true", "mapping_enabled": "true",
        "enable_gles_drm": "true", "headless_render": launch_text(headless),
        "publish_render_output": "true", "record_enabled": "false",
        "enable_rtsp": "false", "enable_esdf_rtsp": "false",
        "display_width": "1920", "display_height": "1080", "video_fps": "30",
        "left_topic": streams["left"]["topic"], "right_topic": streams["right"]["topic"],
        "camera_full_topic": streams["full"]["topic"], "imu_topic": imu_topic,
    })
    session = {"profile": str(profile_path), "camera_fps": fps, "ros_domain_id": domain,
               "odometry_mode": odometry_mode,
               "cpu_affinity": affinity, "headless_render": headless,
               "private_mpp_ipc": private_mpp_ipc,
               "network": network,
               "video": {"width": 1920, "height": 1080, "fps": 30},
               "resources": {key: str(value) if value else "" for key, value in resources.items()}}
    for name, contents in (("camera.json", camera), ("profile.json", profile),
                           ("session.json", session), ("launch-options.json", options),
                           ("runtime-environment.json", resolved_environment)):
        (directory / name).write_text(json.dumps(contents, indent=2) + "\n", encoding="utf-8")


def validate_arguments(directory, launch_files):
    names = set()
    for path in launch_files:
        tree = ast.parse(Path(path).read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                    and node.func.id == "DeclareLaunchArgument" and node.args
                    and isinstance(node.args[0], ast.Constant)):
                names.add(node.args[0].value)
    unknown = set(json.loads((directory / "launch-options.json").read_text())) - names
    if unknown:
        raise ValueError(f"Unknown launch arguments: {sorted(unknown)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", type=Path)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--validate-arguments", nargs="+", type=Path)
    parser.add_argument("--network-check", action="store_true")
    parser.add_argument("--network-configure", action="store_true")
    args = parser.parse_args()
    try:
        if args.validate_arguments:
            validate_arguments(args.directory, args.validate_arguments)
        elif args.network_check or args.network_configure:
            check_network(args.directory, configure=args.network_configure)
        else:
            prepare(args.profile, args.directory)
    except (ValueError, KeyError, TypeError, OSError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Reconstruction/FPV configuration failed: {error}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
