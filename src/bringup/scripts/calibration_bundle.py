"""Offline validation of the stereo configuration snapshots owned by run_vio_session.sh.

Hashes identify files, not a physical USB camera or calibration accuracy.
This module opens no devices and never changes calibration values.
"""

import hashlib
import json
import math
from pathlib import Path

import yaml


MANIFEST = "calibration-manifest.json"
REQUIRED_FILES = {"camera.json", "camera.source.json", "profile.json",
                  "launch-options.json", "calibration.yaml"}


def unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if not isinstance(key, str) or key in result:
            raise ValueError(f"Configuration contains a duplicate or non-string key: {key!r}")
        result[key] = value
    return result


def load_json(data):
    def invalid_constant(value):
        raise ValueError(f"Non-finite JSON constant: {value}")

    value = json.loads(data, object_pairs_hook=unique_pairs,
                       parse_constant=invalid_constant)
    return mapping(value, "JSON root")


class UniqueYamlLoader(yaml.SafeLoader):
    pass


def yaml_mapping(loader, node):
    # Reject ambiguous duplicate keys rather than silently keeping the last one.
    return unique_pairs((loader.construct_object(key, deep=True),
                         loader.construct_object(value, deep=True))
                        for key, value in node.value)


UniqueYamlLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
                                 yaml_mapping)


def load_yaml(data):
    try:
        return mapping(yaml.load(data, Loader=UniqueYamlLoader), "Kalibr YAML root")
    except yaml.YAMLError as error:
        raise ValueError(f"Invalid Kalibr YAML: {error}") from error


def mapping(value, field):
    if not isinstance(value, dict):
        raise ValueError(f"{field} must be an object/mapping")
    return value


def number(value, field, *, positive=False):
    if (type(value) not in (int, float) or not math.isfinite(value)
            or (positive and value <= 0)):
        raise ValueError(f"{field} must be finite" + (" and positive" if positive else ""))
    return value


def dimensions(value, field):
    if (not isinstance(value, (list, tuple)) or len(value) != 2
            or any(type(item) is not int or item <= 0 for item in value)):
        raise ValueError(f"{field} must contain two positive integer dimensions")
    return tuple(value)


def transform(value, field):
    if (not isinstance(value, list) or len(value) != 4
            or any(not isinstance(row, list) or len(row) != 4 for row in value)):
        raise ValueError(f"{field} must be a 4x4 rigid transform")
    for row in value:
        for item in row:
            number(item, field)
    if any(abs(value[3][i] - expected) > 1e-6
           for i, expected in enumerate((0, 0, 0, 1))):
        raise ValueError(f"{field} has an invalid homogeneous last row")
    r = value
    determinant = (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
                   - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
                   + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))
    orthogonal = all(abs(sum(r[k][i] * r[k][j] for k in range(3)) - (i == j)) < 1e-4
                     for i in range(3) for j in range(3))
    if not orthogonal or abs(determinant - 1) > 1e-4:
        raise ValueError(f"{field} rotation must be orthonormal with determinant +1")


def validate(camera, calibration_data, imu_data, *, use_imu):
    """Validate the supported packed-stereo rig, not its physical accuracy."""
    rig = load_yaml(calibration_data)
    if {key for key in rig if key.startswith("cam") and key[3:].isdigit()} != {"cam0", "cam1"}:
        raise ValueError("This stereo entry requires exactly Kalibr cam0 and cam1")
    capture = mapping(camera.get("camera"), "camera")
    packed = dimensions((capture.get("width"), capture.get("height")), "camera size")
    number(capture.get("fps"), "camera.fps", positive=True)
    timestamp = mapping(camera.get("timestamp"), "timestamp").get("source")
    if timestamp not in ("embedded_exposure_start", "embedded_exposure_midpoint"):
        raise ValueError("Stereo session requires an explicit embedded exposure timestamp source")
    config = mapping(camera.get("calibration"), "calibration")
    if config.get("format", "kalibr") != "kalibr":
        raise ValueError("Only Kalibr calibration is supported")
    streams = mapping(camera.get("streams"), "streams")
    outputs = []
    for index, eye in enumerate(("left", "right")):
        name = f"cam{index}"
        node = mapping(rig.get(name), name)
        if (node.get("camera_model") != "pinhole"
                or node.get("distortion_model") not in ("radtan", "equidistant")):
            raise ValueError(f"{name} requires pinhole + radtan/equidistant")
        for field in ("intrinsics", "distortion_coeffs"):
            values = node.get(field)
            if not isinstance(values, list) or len(values) != 4:
                raise ValueError(f"{name}.{field} must contain four values")
            for i, value in enumerate(values):
                number(value, f"{name}.{field}", positive=field == "intrinsics" and i < 2)
        resolution = dimensions(node.get("resolution"), f"{name}.resolution")
        stream = mapping(streams.get(eye), f"streams.{eye}")
        crop = mapping(stream.get("crop"), f"streams.{eye}.crop")
        size = dimensions((crop.get("width"), crop.get("height")), f"{eye} crop size")
        if size != resolution:
            raise ValueError(f"{eye} crop size {size} differs from {name}.resolution {resolution}")
        for axis, limit, extent in zip(("x", "y"), packed, size):
            origin = crop.get(axis, 0)
            if type(origin) is not int or origin < 0 or origin + extent > limit:
                raise ValueError(f"{eye} crop is outside the packed camera image")
        output = mapping(stream.get("output", crop), f"{eye} output")
        outputs.append(dimensions((output.get("width"), output.get("height")), f"{eye} output"))
        for field in ("T_cam_imu", "T_cn_cnm1"):
            if field in node:
                transform(node[field], f"{name}.{field}")
        if "timeshift_cam_imu" in node:
            number(node["timeshift_cam_imu"], f"{name}.timeshift_cam_imu")
    if outputs[0] != outputs[1]:
        raise ValueError("Left/right VIO output dimensions must match")
    stereo = rig["cam1"].get("T_cn_cnm1")
    transform(stereo, "cam1.T_cn_cnm1")
    if not any(stereo[i][3] != 0 for i in range(3)):
        raise ValueError("Stereo baseline must be nonzero")
    if use_imu:
        if "T_cam_imu" not in rig["cam0"]:
            raise ValueError("VIO requires cam0.T_cam_imu")
        if imu_data is None:
            raise ValueError("VIO requires explicit imu_calibration_file; "
                             "no built-in noise fallback")
    if imu_data is not None:
        imu = mapping(load_yaml(imu_data).get("imu0"), "imu0")
        for field in ("gyroscope_noise_density", "gyroscope_random_walk",
                      "accelerometer_noise_density", "accelerometer_random_walk"):
            number(imu.get(field), f"imu0.{field}", positive=True)
        if "update_rate" in imu:
            number(imu["update_rate"], "imu0.update_rate", positive=True)
    # A missing Kalibr shift retains the existing zero-shift behavior, explicitly
    # recorded here; do not invent a correction or equate noise rate to capture rate.
    return {"timestamp_source": timestamp,
            "cam0_timeshift_cam_imu_s": rig["cam0"].get("timeshift_cam_imu", 0.0)}


def write_manifest(directory, *, use_imu, calibration_source):
    directory = Path(directory)
    files = REQUIRED_FILES | ({"imu.yaml"} if (directory / "imu.yaml").is_file() else set())
    metadata = validate(load_json((directory / "camera.json").read_bytes()),
                        (directory / "calibration.yaml").read_bytes(),
                        (directory / "imu.yaml").read_bytes() if "imu.yaml" in files else None,
                        use_imu=use_imu)
    manifest = {"schema_version": 1, "mode": "vio" if use_imu else "vo",
                "calibration_source_name": Path(calibration_source).name,
                **metadata,
                "files": {name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
                          for name in sorted(files)}}
    # Written last; a partial session must never appear to be a complete bundle.
    with (directory / MANIFEST).open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(manifest, indent=2, allow_nan=False) + "\n")


def verify_snapshot(directory):
    """Return False for a legacy recording; never fall back on a broken manifest."""
    directory = Path(directory)
    path = directory / MANIFEST
    if not path.exists() and not path.is_symlink():
        session_path = directory / "session.json"
        if session_path.is_file() and load_json(session_path.read_bytes()).get("calibration_manifest"):
            raise ValueError(f"Required {MANIFEST} disappeared from the session")
        return False
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{MANIFEST} must be a regular file")
    manifest = load_json(path.read_bytes())
    if type(manifest.get("schema_version")) is not int or manifest["schema_version"] != 1:
        raise ValueError("Unsupported calibration manifest schema_version")
    if manifest.get("mode") not in ("vo", "vio"):
        raise ValueError("Invalid calibration manifest mode")
    files = mapping(manifest.get("files"), "calibration manifest files")
    if not REQUIRED_FILES <= files.keys() or set(files) - REQUIRED_FILES - {"imu.yaml"}:
        raise ValueError("Unexpected or missing files in calibration manifest")
    if (directory / "imu.yaml").exists() != ("imu.yaml" in files):
        raise ValueError("IMU snapshot presence differs from calibration manifest")
    for name, digest in files.items():
        member = directory / name
        if member.is_symlink() or not member.is_file():
            raise ValueError(f"Calibration snapshot must be a regular file: {name}")
        if hashlib.sha256(member.read_bytes()).hexdigest() != digest:
            raise ValueError(f"Calibration snapshot SHA256 mismatch: {name}")
    camera = load_json((directory / "camera.json").read_bytes())
    if camera.get("calibration", {}).get("file") != "calibration.yaml":
        raise ValueError("Snapshot camera.json must refer to local calibration.yaml")
    metadata = validate(camera, (directory / "calibration.yaml").read_bytes(),
                        (directory / "imu.yaml").read_bytes() if "imu.yaml" in files else None,
                        use_imu=manifest["mode"] == "vio")
    if any(manifest.get(key) != value for key, value in metadata.items()):
        raise ValueError("Calibration manifest timing differs from the snapshot")
    return True


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    try:
        if not verify_snapshot(args.directory):
            raise ValueError(f"Missing {MANIFEST}")
    except (ValueError, OSError) as error:
        parser.exit(1, f"Calibration snapshot check failed: {error}\n")
    print("Calibration snapshot SHA256/structure: PASS (no hardware opened)")
