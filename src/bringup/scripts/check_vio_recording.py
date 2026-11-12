#!/usr/bin/env python3
"""Validate a stopped stereo recording using original frame IDs and stamps."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess


def check(session, allow_legacy_video_summary=False):
    with (session / "camera0_timestamps.txt").open() as stream:
        camera = list(csv.DictReader(
            (line for line in stream if not line.startswith("#")), delimiter=" "))
    with (session / "vio_frames.csv").open() as stream:
        vio = list(csv.DictReader(line for line in stream if not line.startswith("#")))
    summary = json.loads((session / "vio_frames.csv.summary.json").read_text())
    video_summary_path = session / "camera0_writer_summary.json"
    video_summary = json.loads(video_summary_path.read_text()) if video_summary_path.exists() else None
    video_writer_ok = video_summary is not None and (
        video_summary["written"] == len(camera)
        and all(video_summary[key] == 0 for key in ("queue_dropped", "import_errors", "write_errors")))
    keys = [(int(row["frame_index"]), int(row["timestamp_ns"])) for row in camera]
    key_set = set(keys)
    vio_keys = [(int(row["source_frame_index"]), int(row["image_timestamp_ns"])) for row in vio]
    missing = [key for key in vio_keys if key not in key_set]
    bad_poses = []
    for row in vio:
        if row["status"] == "ok":
            values = [float(row[field]) for field in ("tx", "ty", "tz", "qx", "qy", "qz", "qw")]
            if (not all(math.isfinite(value) for value in values)
                    or abs(sum(value * value for value in values[3:]) - 1) > 1e-3
                    or row["estimate_timestamp_ns"] != row["image_timestamp_ns"]):
                bad_poses.append(row["source_frame_index"])
    packets = subprocess.check_output([
        "ffprobe", "-v", "error", "-select_streams", "v:0", "-count_packets",
        "-show_entries", "stream=nb_read_packets", "-of", "csv=p=0",
        str(session / "camera0.mkv")], text=True)
    packet_count = int(packets.strip())
    monotonic = all(a[1] < b[1] for a, b in zip(keys, keys[1:]))
    vio_monotonic = all(a[1] < b[1] for a, b in zip(vio_keys, vio_keys[1:]))
    legacy = video_summary is None and allow_legacy_video_summary
    checks = {
        "No camera frames": bool(camera),
        "No valid VIO attempts": any(row["status"] == "ok" for row in vio),
        ("Missing camera0_writer_summary.json; ordinary replay can explicitly allow legacy recordings"
         if video_summary is None else "Video writer reports loss or errors"):
            video_writer_ok or legacy,
        "Video packet count differs from timestamp rows": packet_count == len(camera),
        "Duplicate camera frame keys": len(key_set) == len(keys),
        "Duplicate VIO frame keys": len(set(vio_keys)) == len(vio_keys),
        "Non-monotonic camera timestamps": monotonic,
        "Non-monotonic VIO timestamps": vio_monotonic,
        "VIO inputs missing from video": not missing,
        "Invalid recorded poses": not bad_poses,
        "VIO writer summary count mismatch": summary["written"] == len(vio),
        "VIO writer reports dropped rows": summary["dropped"] == 0,
        "VIO writer reports write failure": not summary["write_failed"],
        "Unknown VIO status": all(row["status"] in ("ok", "invalid_pose", "track_error")
                                  for row in vio),
    }
    errors = [reason for reason, passed in checks.items() if not passed]
    valid = not errors
    result = {"pass": valid, "camera_frames": len(camera), "video_packets": packet_count,
              "vio_attempts": len(vio), "vio_ok": sum(row["status"] == "ok" for row in vio),
              "missing_video_frames": missing[:20], "missing_video_count": len(missing),
              "bad_poses": bad_poses[:20], "camera_time_monotonic": monotonic,
              "vio_time_monotonic": vio_monotonic, "summary": summary,
              "video_summary": video_summary, "errors": errors,
              "warnings": (["Legacy video writer statistics are unavailable; packet/frame "
                            "association checked, historical write errors cannot be certified."]
                           if legacy else []),
              "video_writer_verified": bool(video_writer_ok)}
    print(json.dumps(result, indent=2))
    return valid


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--allow-legacy-video-summary", action="store_true",
                        help="Ordinary replay only: permit absent video statistics, not failed statistics")
    args = parser.parse_args()
    try:
        passed = check(args.session, args.allow_legacy_video_summary)
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
        print(json.dumps({"pass": False, "errors": [str(error)]}, indent=2))
        passed = False
    raise SystemExit(0 if passed else 1)
