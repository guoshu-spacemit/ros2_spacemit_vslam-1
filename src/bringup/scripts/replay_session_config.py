#!/usr/bin/env python3
"""Prepare a detached replay session without opening camera/DRM devices."""

import argparse
import json
from pathlib import Path
import subprocess
import signal

import yaml

from vio_session_config import resource
from calibration_bundle import verify_snapshot


def recorded_mode(source, mode="auto"):
    """Resolve the sensor mode without modifying the recorded inputs."""
    if mode not in ("auto", "vo", "vio"):
        raise ValueError("Invalid replay sensor mode")
    source = Path(source)
    try:
        parameters = yaml.safe_load((source / "vio_parameters.yaml").read_text())
    except yaml.YAMLError as error:
        raise ValueError(f"Invalid vio_parameters.yaml: {error}") from error
    if not isinstance(parameters, dict) or type(parameters.get("use_imu", True)) is not bool:
        raise ValueError("vio_parameters.yaml must contain a boolean use_imu")
    return parameters.get("use_imu", True) if mode == "auto" else mode == "vio"


def prepare(source, directory, profile, *, mode="auto", mapping=False,
            paused=False, features=True, domain=0):
    source = Path(source).resolve(strict=True)
    directory = Path(directory).resolve()
    if not 0 <= domain <= 232 or mode not in ("auto", "vo", "vio"):
        raise ValueError("Invalid domain or odometry mode")
    imu = recorded_mode(source, mode)
    calibration_verified = verify_snapshot(source)
    if not calibration_verified:
        print("WARNING: legacy recording has no calibration-manifest.json; "
              "using its recorded snapshots without integrity assurance.")
    model = ""
    if mapping:
        profile = Path(profile).resolve(strict=True)
        config = json.loads(profile.read_text())
        model = str(resource(profile.parent, config.get("model_path"), "model_path"))
    options = {
        "enable_screen": "true", "mapping_enabled": str(mapping).lower(),
        "model_path": model, "record_screen": "false",
        "replay_realtime": "true", "replay_start_paused": str(paused).lower(),
        "feature_panel_enabled": str(features).lower(),
        "publish_frame_diagnostics": "true", "replay_timeout_sec": "120.0",
        "odometry_mode": "recorded" if mode == "auto" else mode,
    }
    directory.mkdir(parents=True, exist_ok=False)
    session = {"source": "step" if paused else "replay", "input": str(source),
               "mode": "vio" if imu else "vo", "domain": domain,
               "mapping": mapping, "record": False,
               "calibration_verified": calibration_verified,
               "legacy_video_summary": not (source / "camera0_writer_summary.json").is_file(),
               "launch_arguments": options}
    (directory / "session.json").write_text(json.dumps(session, indent=2) + "\n")
    print(f"Replay input: {source}; mode={session['mode']}")
    print("Recorded images/IMU are replayed; pose differences are reported separately.")


def run(directory):
    directory = Path(directory)
    session = json.loads((directory / "session.json").read_text())
    # Recheck immediately before launch; source files may change after prepare.
    verified = verify_snapshot(session["input"])
    if session.get("calibration_verified") and not verified:
        raise ValueError("Calibration manifest disappeared after replay preparation")
    command = ["bash", str(Path(__file__).with_name("run_vio_replay.sh")),
               session["input"], str(directory / "result")]
    command += [f"{k}:={v}" for k, v in session["launch_arguments"].items() if v != ""]
    status_path = directory / "pipeline-status.json"
    status_path.write_text(json.dumps({"state": "running", "readiness": "not_observed"}) + "\n")
    process = subprocess.Popen(command)
    interrupted = False

    def stop(*_):
        nonlocal interrupted
        # systemd/terminal sends SIGINT to the whole process group. The child
        # launch/recorder drains itself; keep the supervisor alive to reap it.
        interrupted = True

    previous = signal.signal(signal.SIGINT, stop)
    try:
        returncode = process.wait()
    finally:
        signal.signal(signal.SIGINT, previous)
    status_path.write_text(json.dumps({
        "state": "interrupted" if interrupted else ("completed" if returncode == 0 else "failed"),
        "returncode": returncode,
        "result_directory": str(directory / "result"),
    }, indent=2) + "\n")
    return 0 if interrupted else returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, nargs="?")
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--source", type=Path)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--mode", choices=("auto", "vo", "vio"), default="auto")
    parser.add_argument("--mapping", choices=("on", "off"), default="off")
    parser.add_argument("--paused", action="store_true")
    parser.add_argument("--features", choices=("on", "off"), default="on")
    parser.add_argument("--domain", type=int, default=0)
    args = parser.parse_args()
    if args.directory is None:
        parser.error("directory is required")
    if args.run:
        return run(args.directory)
    if args.source is None:
        parser.error("--source is required")
    prepare(args.source, args.directory, args.profile, mode=args.mode,
            mapping=args.mapping == "on", paused=args.paused,
            features=args.features == "on", domain=args.domain)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
