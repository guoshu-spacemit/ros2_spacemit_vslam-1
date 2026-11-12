#!/usr/bin/env python3
"""Build spacemit-vslam native modules and ROS packages through the Robot SDK.

The build helper requires Python 3.6 or later. Set SROBOTIS_PYTHON to select
the SDK entry's Python executable.
"""

import argparse
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PACKAGES = ("common", "camera_io", "vio", "las2", "esdf_drm", "bringup")
MODULES = ("camera", "vslam", "mapping")
DEFAULT_ROS_WORKER_LIMIT = 2
ROS_COMPILE_JOBS_PER_PACKAGE = 1


def find_sdk(source):
    """Return the enclosing SDK root, or None when the source is outside an SDK."""
    for candidate in (source, *source.parents):
        if (candidate / "build/envsetup.sh").is_file():
            return candidate
    return None


def inspect_layout(source, sdk):
    """Print source/dependency checks and return whether every required path exists."""
    manifest = ET.parse(source / "package.xml").getroot()
    checks = {}
    dependencies = (
        manifest.findall("depend")
        + manifest.findall("build_depend")
        + manifest.findall("exec_depend")
    )
    for dependency in dependencies:
        name = dependency.text.strip()
        path = sdk / name
        checks[name] = {"path": str(path), "present": path.is_dir()}
    missing_packages = [
        name for name in PACKAGES
        if not (source / "src" / name / "package.xml").is_file()
    ]
    missing_modules = [
        name for name in MODULES
        if not (source / "modules" / name / "package.xml").is_file()
    ]
    result = {
        "sdk": str(sdk),
        "ros_packages": list(PACKAGES),
        "native_modules": list(MODULES),
        "dependencies": checks,
        "missing_packages": missing_packages,
        "missing_modules": missing_modules,
        "scope": "Source layout only; no compile, ABI, model or hardware claim.",
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return (
        not missing_packages
        and not missing_modules
        and all(item["present"] for item in checks.values())
    )


def run_command(command, sdk, env):
    """Run one checked command in the SDK root using the supplied environment."""
    print("[spacemit-vslam] " + " ".join(map(str, command)), flush=True)
    subprocess.run(list(map(str, command)), cwd=sdk, env=env, check=True)


def build_project(source, sdk, output, prefix, jobs, ros_setup, cmake_args, env, *, ros_workers):
    """Run inside the SDK's existing custom-build install lock.

    Do not invoke build/build.sh here: its nested install lock would deadlock.
    Explicit source paths keep this module independent of SDK global discovery.
    """
    native = [
        sdk / "components/multimedia/mpp",
        source / "modules/camera",
        source / "modules/mapping",
        sdk / "components/perception/stereo_matching",
        source / "modules/vslam",
        sdk / "components/peripherals/imu",
    ]
    defaults = [
        "-DCMAKE_BUILD_TYPE=Release",
        "-DBUILD_SHARED_LIBS=ON",
        "-DBUILD_TESTING=OFF",
        "-DBUILD_TESTS=OFF",
        "-DSTEREO_MATCHING_BUILD_EXTERNAL_INPUT=ON",
    ]
    sdk_prefix_args = [
        f"-DMPP_BUILD_DIR={prefix}",
        f"-DSPACEMIT_SDK_PREFIX={prefix}",
    ]
    prefix_path = str(prefix)
    if env.get("CMAKE_PREFIX_PATH"):
        prefix_path += ";" + env["CMAKE_PREFIX_PATH"].replace(":", ";")
    for package in native:
        key = str(package.relative_to(sdk)).replace("/", "_")
        build = output / "build/cmake/pkgs" / key
        run_command(
            [
                "cmake", "-S", package, "-B", build,
                f"-DCMAKE_INSTALL_PREFIX={prefix}",
                f"-DCMAKE_PREFIX_PATH={prefix_path}",
                *defaults, *sdk_prefix_args, *cmake_args,
            ],
            sdk,
            env,
        )
        run_command(["cmake", "--build", build, "--parallel", jobs], sdk, env)
        run_command(["cmake", "--install", build], sdk, env)

    env = dict(env)
    # Colcon schedules packages in parallel. Give each package one compile job
    # so its generator cannot start another pool sized to the whole board.
    makeflags = f" {env.get('MAKEFLAGS', '')} "
    make_options, separator, make_overrides = makeflags.partition(" -- ")
    env["MAKEFLAGS"] = (
        f"{make_options.rstrip()} -j{ROS_COMPILE_JOBS_PER_PACKAGE}{separator}{make_overrides}"
    ).strip()
    env["CMAKE_BUILD_PARALLEL_LEVEL"] = str(ROS_COMPILE_JOBS_PER_PACKAGE)
    for name in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"):
        env[name] = str(prefix) + (":" + env[name] if env.get(name) else "")
    ros_command = [
        "colcon", "--log-base", output / "log/ros2/spacemit-vslam", "build",
        "--base-paths", source / "src",
        sdk / "middleware/ros2/peripherals/imu",
        sdk / "components/multimedia/mpp/test/ros2_mpp_msgs",
        "--packages-select", "peripherals_imu_node", "mpp_msgs", *PACKAGES,
        "--merge-install", "--install-base", prefix,
        "--build-base", output / "build/ros2/spacemit-vslam",
        "--parallel-workers", ros_workers, "--cmake-args",
        f"-DCMAKE_PREFIX_PATH={prefix_path}", "-DCMAKE_BUILD_TYPE=Release",
        "-DBUILD_TESTING=OFF", f"-DSROBOTIS_ROOT={sdk}", *sdk_prefix_args, *cmake_args,
    ]
    # Use the requested system ROS environment, not a pre-existing product overlay.
    run_command(
        [
            "bash", "-c", 'set -e; source "$1"; shift; exec "$@"',
            "spacemit-vslam-ros", ros_setup, *ros_command,
        ],
        sdk,
        env,
    )


def main():
    """Validate CLI options and dispatch through the original SDK build entry."""
    source = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", type=Path, default=find_sdk(source))
    parser.add_argument("--target", help="Optional existing native SDK product target")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--jobs", type=int)
    parser.add_argument("--cross", action="store_true")
    parser.add_argument(
        "--cmake-arg", action="append", default=[], metavar="ARG",
        help="Extra CMake argument; use --cmake-arg=-DNAME=VALUE",
    )
    parser.add_argument(
        "--ros-setup", type=Path,
        default=Path(os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash")),
    )
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--sdk-entry", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.sdk is None:
        parser.error("--sdk must identify the enclosing Robot SDK")
    sdk = args.sdk.resolve()
    jobs = args.jobs if args.jobs is not None else int(os.environ.get("PARALLEL_JOBS", "4"))
    if jobs < 1:
        parser.error("--jobs must be positive")
    if not (sdk / "build/build.sh").is_file():
        parser.error("--sdk must identify the enclosing Robot SDK")
    try:
        package_path = str(source.relative_to(sdk))
    except ValueError:
        parser.error("Source bundle must be inside --sdk")
    if args.check:
        raise SystemExit(0 if inspect_layout(source, sdk) else 1)
    if args.cross or os.environ.get("SROBOTIS_CROSS_BUILD") == "1":
        parser.error("Project-local cross compilation is not validated; use a native K3 build")
    target = args.target or os.environ.get("BUILD_TARGET")
    target_args = []
    if target:
        target_file = sdk / "target" / f"{target}.json"
        if not target_file.is_file():
            parser.error("--target must name an existing SDK product JSON")
        target_args = json.loads(target_file.read_text()).get("options", {}).get("cmake_args", [])
        if not isinstance(target_args, list) or not all(isinstance(x, str) for x in target_args):
            parser.error("target options.cmake_args must be a string array")
    output = (args.output or Path(os.environ.get("OUTPUT_ROOT", sdk / "output"))).resolve()
    if output == source or output in source.parents:
        parser.error("--output must be a separate build directory")
    prefix = Path(os.environ.get("PREFIX", output / "staging")).resolve()
    if args.output:
        prefix = output / "staging"
    cmake_args = [
        *os.environ.get("SROBOTIS_CMAKE_EXTRA_ARGS", "").splitlines(),
        *target_args, *args.cmake_arg,
    ]
    env = dict(os.environ)
    env.update(
        SROBOTIS_ROOT=str(sdk),
        ROS_SETUP=str(args.ros_setup),
        OUTPUT_ROOT=str(output),
        PREFIX=str(prefix),
        STAGING_PREFIX=str(prefix),
        PARALLEL_JOBS=str(jobs),
        SROBOTIS_CMAKE_EXTRA_ARGS="\n".join(cmake_args),
    )
    if target:
        env["BUILD_TARGET"] = target
    else:
        for name in ("BUILD_TARGET", "BUILD_TARGET_FILE", "BUILD_CONFIG_FILE"):
            env.pop(name, None)
    try:
        if args.sdk_entry:
            if not args.ros_setup.is_file():
                parser.error("--ros-setup must identify an installed ROS setup.bash")
            ros_workers = jobs
            if args.jobs is None and env.get("SROBOTIS_PARALLEL_JOBS_EXPLICIT") != "1":
                ros_workers = min(jobs, DEFAULT_ROS_WORKER_LIMIT)
            build_project(
                source, sdk, output, prefix, jobs, args.ros_setup, cmake_args, env,
                ros_workers=ros_workers,
            )
        else:
            # Original SDK dispatches a project's executable build.sh before
            # classifying its directory. No SDK changes or --with-deps extensions.
            env["SROBOTIS_PYTHON"] = env.get("SROBOTIS_PYTHON") or sys.executable
            command = [sdk / "build/build.sh"]
            if args.jobs is not None:
                command.append(f"-j{jobs}")
            run_command([*command, "package", package_path], sdk, env)
    except subprocess.CalledProcessError as error:
        raise SystemExit(error.returncode) from error


if __name__ == "__main__":
    main()
