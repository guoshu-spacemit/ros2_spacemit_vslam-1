"""Initial display-only down direction for pure VO session replay.

This never feeds an IMU measurement or constraint into the VO estimator.
Only an initially quiet IMU interval overlapping the first Track target is used.
"""

import csv
import math
from pathlib import Path

import yaml


NANOSECONDS_PER_SECOND = 1_000_000_000
ROTATION_MATRIX_TOLERANCE = 1e-3
STANDARD_GRAVITY_M_S2 = 9.80665

# Bound initialization to a short window beginning at the first Track target.
# Require samples near the start and across most of the window before using
# the estimate to orient the display.
INITIAL_IMU_WINDOW_NS = 250_000_000
MIN_INITIAL_IMU_SAMPLES = 20
MAX_INITIAL_IMU_DELAY_NS = 20_000_000
MIN_INITIAL_IMU_SPAN_NS = 200_000_000

# Reject movement and acceleration variation before averaging specific force.
MAX_ACCELERATION_MAGNITUDE_ERROR_M_S2 = 0.75
MAX_ANGULAR_SPEED_RAD_S = 0.08
MAX_ACCELERATION_RMS_M_S2 = 0.2


def initial_camera_gravity(session):
    """Estimate display gravity in the initial camera frame from a recording.

    Args:
        session: Directory containing Kalibr calibration, VIO frame results and
            recorded IMU samples.

    Returns:
        A dictionary with the camera-frame gravity vector, source and sample
        statistics for replay display.

    Raises:
        ValueError: If calibration is invalid or the initial IMU interval is
            unsuitable.
        OSError: If a required recording file cannot be read.
    """
    session = Path(session)
    document = yaml.safe_load((session / "calibration.yaml").read_text())
    if not isinstance(document, dict) or not isinstance(document.get("cam0"), dict):
        raise ValueError("calibration has no cam0 mapping")
    calibration = document["cam0"]
    transform = calibration["T_cam_imu"]
    if (len(transform) != 4 or any(len(row) != 4 for row in transform)
            or not all(math.isfinite(float(v)) for row in transform for v in row)):
        raise ValueError("invalid cam0.T_cam_imu for display gravity")
    rotation = [[float(v) for v in row[:3]] for row in transform[:3]]
    determinant = sum(rotation[0][i] * (rotation[1][(i+1) % 3] * rotation[2][(i+2) % 3]
                                      - rotation[1][(i+2) % 3] * rotation[2][(i+1) % 3])
                      for i in range(3))
    if abs(determinant - 1.0) > ROTATION_MATRIX_TOLERANCE:
        raise ValueError("cam0.T_cam_imu rotation is not proper")
    for i in range(3):
        for j in range(3):
            value = sum(rotation[i][k] * rotation[j][k] for k in range(3))
            if abs(value - float(i == j)) > ROTATION_MATRIX_TOLERANCE:
                raise ValueError("cam0.T_cam_imu rotation is not orthonormal")
    with (session / "vio_frames.csv").open() as stream:
        first = next(csv.DictReader(line for line in stream if not line.startswith("#")), None)
    if first is None:
        raise ValueError("VIO frame results contain no Track target")
    if first["status"] != "ok":
        raise ValueError("first Track target has no pose; cannot anchor initial display gravity")
    shift = float(calibration.get("timeshift_cam_imu", 0.0))
    if not math.isfinite(shift):
        raise ValueError("non-finite Kalibr timeshift_cam_imu")
    start = int(first["image_timestamp_ns"]) + round(shift * NANOSECONDS_PER_SECOND)
    samples = []
    last_stamp = None
    with (session / "camera0_imu.txt").open() as stream:
        reader = csv.DictReader((line for line in stream if not line.startswith("#")),
                                delimiter=" ", skipinitialspace=True)
        for row in reader:
            stamp = int(row["timestamp_ns"])
            if last_stamp is not None and stamp <= last_stamp:
                raise ValueError("non-monotonic IMU timestamps")
            last_stamp = stamp
            if stamp < start:
                continue
            if stamp > start + INITIAL_IMU_WINDOW_NS:
                break
            accel = [float(row[key]) for key in ("ax", "ay", "az")]
            gyro = [float(row[key]) for key in ("gx", "gy", "gz")]
            if not all(math.isfinite(v) for v in accel + gyro):
                raise ValueError("non-finite initial IMU sample")
            acceleration_m_s2 = math.sqrt(sum(value * value for value in accel))
            angular_speed_rad_s = math.sqrt(sum(value * value for value in gyro))
            acceleration_error_m_s2 = abs(acceleration_m_s2 - STANDARD_GRAVITY_M_S2)
            if (
                acceleration_error_m_s2 > MAX_ACCELERATION_MAGNITUDE_ERROR_M_S2
                or angular_speed_rad_s > MAX_ANGULAR_SPEED_RAD_S
            ):
                raise ValueError("initial IMU interval is not quiet; no reliable display gravity")
            samples.append((stamp, accel))
    if (
        len(samples) < MIN_INITIAL_IMU_SAMPLES
        or samples[0][0] - start > MAX_INITIAL_IMU_DELAY_NS
        or samples[-1][0] - samples[0][0] < MIN_INITIAL_IMU_SPAN_NS
    ):
        raise ValueError("insufficient initial IMU coverage for display gravity")
    mean = [sum(a[k] for _, a in samples) / len(samples) for k in range(3)]
    rms = math.sqrt(sum(sum((a[k] - mean[k])**2 for k in range(3))
                        for _, a in samples) / len(samples))
    if rms > MAX_ACCELERATION_RMS_M_S2:
        raise ValueError("initial acceleration varies too much for display gravity")
    # At rest accelerometers measure specific force (up), not gravity (down).
    gravity = [-sum(rotation[i][k] * mean[k] for k in range(3)) for i in range(3)]
    return {
        "source": "initial_quiet_imu_display_only",
        "gravity_initial_camera": gravity,
        "samples": len(samples), "accel_rms_m_s2": rms,
        "first_timestamp_ns": samples[0][0], "last_timestamp_ns": samples[-1][0],
        "use_imu_in_odometry": False,
    }
