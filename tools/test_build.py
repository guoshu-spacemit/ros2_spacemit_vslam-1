#!/usr/bin/env python3
"""Exercise the unmodified SDK custom entry with recording CMake/colcon tools."""

import json
import os
import shutil
import subprocess
import sys
import tempfile
import textwrap
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[1]
SDK = SOURCE.parents[3]


class OriginalSdkBuildTest(unittest.TestCase):
    """Check dispatch order, SDK lock ownership, and error propagation."""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="spacemit-original-sdk-")
        self.addCleanup(self.temp.cleanup)
        self.sdk = Path(self.temp.name)
        shutil.copytree(
            SDK / "build", self.sdk / "build",
            ignore=shutil.ignore_patterns(".git", "__pycache__", ".pytest_cache"),
        )
        self.project = self.sdk / "middleware/ros2/slam/spacemit-vslam"
        (self.project / "tools").mkdir(parents=True)
        for name in ("build.py",):
            shutil.copy2(SOURCE / "tools" / name, self.project / "tools" / name)
        shutil.copy2(SOURCE / "build.sh", self.project / "build.sh")
        shutil.copy2(SOURCE / "package.xml", self.project / "package.xml")
        manifest = ET.parse(SOURCE / "package.xml").getroot()
        for tag in ("depend", "build_depend", "exec_depend"):
            for dependency in manifest.findall(tag):
                (self.sdk / dependency.text.strip()).mkdir(parents=True, exist_ok=True)
        for key in (
            "components/multimedia/mpp",
            "components/perception/stereo_matching",
            "components/peripherals/imu",
            "middleware/ros2/peripherals/imu",
            "components/multimedia/mpp/test/ros2_mpp_msgs",
        ):
            path = self.sdk / key
            name = path.name
            build_type = "ament_cmake" if key in (
                "middleware/ros2/peripherals/imu",
                "components/multimedia/mpp/test/ros2_mpp_msgs",
            ) else "cmake"
            (path / "CMakeLists.txt").write_text(
                f"cmake_minimum_required(VERSION 3.22)\nproject({name})\n"
            )
            (path / "package.xml").write_text(
                f"<package><name>{name}</name>"
                f"<export><build_type>{build_type}</build_type></export></package>\n"
            )
        package_groups = (
            ("modules", ("camera", "mapping", "vslam")),
            ("src", ("common", "camera_io", "vio", "las2", "esdf_drm", "bringup")),
        )
        for directory, packages in package_groups:
            for name in packages:
                path = self.project / directory / name
                path.mkdir(parents=True, exist_ok=True)
                (path / "package.xml").write_text(f"<package><name>{name}</name></package>\n")
        self.ros = self.sdk / "ros-setup.bash"
        self.ros.write_text(":\n")
        self.record = self.sdk / "commands.jsonl"
        self.output = self.sdk / "isolated-output"
        tools = self.sdk / "fake-tools"
        tools.mkdir()
        self.path_python = tools / "python3"
        recorder = f"#!{sys.executable}\n" + textwrap.dedent(r'''
            import fcntl
            import json
            import os
            import sys
            from pathlib import Path

            locked = False
            with open(os.environ["TEST_LOCK"], "a") as handle:
                try:
                    fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    locked = True
            record = {
                "tool": Path(sys.argv[0]).name,
                "args": sys.argv[1:],
                "sdk_lock_held": locked,
                "makeflags": os.environ.get("MAKEFLAGS"),
                "cmake_parallel_level": os.environ.get("CMAKE_BUILD_PARALLEL_LEVEL"),
            }
            with open(os.environ["TEST_RECORD"], "a") as handle:
                handle.write(json.dumps(record) + "\n")
            if (
                "-S" in sys.argv
                and os.environ.get("TEST_FAIL_SOURCE")
                and os.environ["TEST_FAIL_SOURCE"] in sys.argv[sys.argv.index("-S") + 1]
            ):
                sys.exit(17)
        ''')
        for name in ("cmake", "colcon"):
            tool = tools / name
            tool.write_text(recorder)
            tool.chmod(0o755)
        query = tools / "dpkg-query"
        query.write_text(f"#!{sys.executable}\n" + textwrap.dedent(r'''
            import os
            import sys

            package = sys.argv[-1]
            with open(os.environ["TEST_SYSTEM_DEP_RECORD"], "a") as handle:
                handle.write(package + "\n")
            if package == os.environ.get("TEST_MISSING_SYSTEM_DEP"):
                sys.exit(1)
            print("install ok installed")
        '''))
        query.chmod(0o755)
        apt = tools / "apt-get"
        apt.write_text(f"#!{sys.executable}\n" + textwrap.dedent(r'''
            import os
            import sys
            from pathlib import Path

            Path(os.environ["TEST_APT_RECORD"]).write_text("unexpected apt invocation\n")
            sys.exit(97)
        '''))
        apt.chmod(0o755)
        self.system_record = self.sdk / "system-dependencies.txt"
        self.apt_record = self.sdk / "apt-invoked.txt"
        self.env = dict(
            os.environ,
            PATH=str(tools) + os.pathsep + os.environ["PATH"],
            SROBOTIS_SKIP_DEPS_CHECK="1",
            SROBOTIS_USE_DOCKER_BUILD="0",
            TEST_LOCK=str(self.output / ".install.lock"),
            TEST_RECORD=str(self.record),
            TEST_SYSTEM_DEP_RECORD=str(self.system_record),
            TEST_APT_RECORD=str(self.apt_record),
            APT_GET_BIN=str(apt),
        )
        for name in (
            "BUILD_TARGET", "BUILD_TARGET_FILE", "BUILD_CONFIG_FILE", "PREFIX",
            "SROBOTIS_CROSS_BUILD", "SROBOTIS_CMAKE_EXTRA_ARGS", "TEST_MISSING_SYSTEM_DEP",
            "MAKEFLAGS", "CMAKE_BUILD_PARALLEL_LEVEL", "SROBOTIS_PARALLEL_JOBS_EXPLICIT",
            "SROBOTIS_PYTHON",
        ):
            self.env.pop(name, None)

    def invoke(self, *extra, jobs=2):
        """Invoke the project CLI against the isolated SDK fixture."""
        job_args = [] if jobs is None else ["--jobs", str(jobs)]
        return subprocess.run(
            [
                sys.executable, str(self.project / "tools/build.py"),
                "--output", str(self.output), *job_args,
                "--ros-setup", str(self.ros), *extra,
            ],
            env=self.env, capture_output=True, text=True, timeout=30,
        )

    def commands(self):
        """Return the fixture's ordered CMake and colcon invocation records."""
        return [json.loads(line) for line in self.record.read_text().splitlines()]

    def dependency_closure(self):
        """Use the SDK's real non-ROS resolver without bypassing its checks."""
        return subprocess.run(
            [
                "bash", "-c",
                'set -euo pipefail; source "$1/build/common.sh"; '
                'source "$1/build/nonros2.sh"; '
                'resolve_nonros2_dependency_closure "$2" 0',
                "sdk-dependency-check", str(self.sdk),
                "middleware/ros2/slam/spacemit-vslam",
            ],
            env=dict(self.env, REPO_ROOT=str(self.sdk)),
            capture_output=True, text=True, timeout=30,
        )

    def invoke_with_deps(self):
        """Run the real SDK dependency preflight and custom build dispatcher."""
        # Keep SDK scripts and project requirements unchanged. The controlled
        # package database covers project dependencies; omit unrelated base tools.
        metadata = self.sdk / "build/package.xml"
        tree = ET.parse(metadata)
        for parent in tree.iter():
            for child in list(parent):
                if child.tag == "system_depend":
                    parent.remove(child)
        tree.write(metadata)
        env = dict(
            self.env, SROBOTIS_SKIP_DEPS_CHECK="0", AUTO_INSTALL_DEPS="no",
            OUTPUT_ROOT=str(self.output), ROS_SETUP=str(self.ros), PARALLEL_JOBS="8",
        )
        return subprocess.run(
            [
                str(self.sdk / "build/build.sh"), "package",
                "middleware/ros2/slam/spacemit-vslam", "--with-deps",
            ],
            cwd=self.sdk, env=env, input="n\n", capture_output=True, text=True, timeout=30,
        )

    def test_project_cli_uses_default_ros_limit_without_explicit_jobs(self):
        # The SDK must reenter using the Python that invoked tools/build.py.
        self.path_python.symlink_to("/bin/false")
        self.env["PARALLEL_JOBS"] = "8"
        result = self.invoke(jobs=None)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        commands = self.commands()
        colcon_args = next(item["args"] for item in commands if item["tool"] == "colcon")
        self.assertEqual(colcon_args[colcon_args.index("--parallel-workers") + 1], "2")
        native_args = next(item["args"] for item in commands if item["args"][0] == "--build")
        self.assertEqual(native_args[native_args.index("--parallel") + 1], "8")

    def test_original_sdk_dispatch_orders_native_then_exact_ros_sources(self):
        self.env["MAKEFLAGS"] = "-s -j8 -- CCACHE_DIR=/fixture/cache"
        result = self.invoke(
            "--cmake-arg=-DONNXRUNTIME_ROOT=/fixture/runtime",
            "--cmake-arg=-DSTEREO_MATCHING_EXTERNAL_INPUT_MODEL=/fixture/model.onnx",
            jobs=4,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        commands = self.commands()
        self.assertEqual(len(commands), 19)
        self.assertTrue(all(item["sdk_lock_held"] for item in commands))
        configured = [
            item["args"][1] for item in commands
            if item["tool"] == "cmake" and item["args"][0] == "-S"
        ]
        self.assertEqual(
            configured,
            [str(path) for path in (
                self.sdk / "components/multimedia/mpp", self.project / "modules/camera",
                self.project / "modules/mapping",
                self.sdk / "components/perception/stereo_matching",
                self.project / "modules/vslam", self.sdk / "components/peripherals/imu",
            )],
        )
        stereo_matching_config = next(
            item["args"] for item in commands
            if item["tool"] == "cmake" and item["args"][:2] == [
                "-S", str(self.sdk / "components/perception/stereo_matching"),
            ]
        )
        self.assertIn("-DSTEREO_MATCHING_BUILD_EXTERNAL_INPUT=ON", stereo_matching_config)
        self.assertIn(
            "-DSTEREO_MATCHING_EXTERNAL_INPUT_MODEL=/fixture/model.onnx",
            stereo_matching_config,
        )
        for item in commands:
            if item["tool"] == "cmake" and item["args"][0] == "-S":
                self.assertIn("-DONNXRUNTIME_ROOT=/fixture/runtime", item["args"])
                self.assertIn(f"-DCMAKE_INSTALL_PREFIX={self.output}/staging", item["args"])
                self.assertIn(f"-DMPP_BUILD_DIR={self.output}/staging", item["args"])
                self.assertIn(f"-DSPACEMIT_SDK_PREFIX={self.output}/staging", item["args"])
        ros = commands[-1]
        self.assertEqual(ros["tool"], "colcon")
        self.assertEqual(ros["makeflags"], "-s -j8 -j1 -- CCACHE_DIR=/fixture/cache")
        self.assertEqual(ros["cmake_parallel_level"], "1")
        args = ros["args"]
        self.assertEqual(args[args.index("--parallel-workers") + 1], "4")
        start, end = args.index("--base-paths") + 1, args.index("--packages-select")
        self.assertEqual(
            args[start:end],
            [
                str(self.project / "src"),
                str(self.sdk / "middleware/ros2/peripherals/imu"),
                str(self.sdk / "components/multimedia/mpp/test/ros2_mpp_msgs"),
            ],
        )
        self.assertNotIn(str(self.project / "modules"), args)
        self.assertIn(f"-DMPP_BUILD_DIR={self.output}/staging", args)
        self.assertIn(f"-DSPACEMIT_SDK_PREFIX={self.output}/staging", args)

    def test_explicit_dependency_prefixes_override_output_defaults(self):
        result = self.invoke(
            "--cmake-arg=-DMPP_BUILD_DIR=/selected/mpp",
            "--cmake-arg=-DSPACEMIT_SDK_PREFIX=/selected/sdk",
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        for item in self.commands():
            if item["tool"] == "cmake" and item["args"][0] != "-S":
                continue
            for name, selected in (("MPP_BUILD_DIR", "/selected/mpp"),
                                   ("SPACEMIT_SDK_PREFIX", "/selected/sdk")):
                values = [arg for arg in item["args"] if arg.startswith(f"-D{name}=")]
                self.assertEqual(values, [
                    f"-D{name}={self.output}/staging", f"-D{name}={selected}",
                ])

    def test_native_failure_stops_before_ros(self):
        self.env["TEST_FAIL_SOURCE"] = "modules/camera"
        result = self.invoke()
        self.assertEqual(result.returncode, 17, result.stdout + result.stderr)
        self.assertEqual(len(self.commands()), 4)
        self.assertTrue(all(item["tool"] == "cmake" for item in self.commands()))

    def test_layout_check_reads_sdk_dependencies_from_package_xml(self):
        result = self.invoke("--check")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        checks = json.loads(result.stdout)["dependencies"]
        for dependency in (
            "components/perception/stereo_matching",
            "middleware/ros2/peripherals/imu",
            "components/multimedia/mpp/test/ros2_mpp_msgs",
        ):
            with self.subTest(dependency=dependency):
                self.assertTrue(checks[dependency]["present"])
                shutil.rmtree(self.sdk / dependency)
                result = self.invoke("--check")
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assertFalse(json.loads(result.stdout)["dependencies"][dependency]["present"])
                (self.sdk / dependency).mkdir(parents=True)
        self.assertFalse(self.record.exists())

    def test_sdk_nonros_closure_excludes_ros_build_and_runtime_dependencies(self):
        result = self.dependency_closure()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(result.stdout.splitlines(), [
            "components/multimedia/mpp",
            "components/perception/stereo_matching",
            "components/peripherals/imu",
        ])
        manifest = self.project / "package.xml"
        tree = ET.parse(manifest)
        ET.SubElement(tree.getroot(), "depend").text = "middleware/ros2/peripherals/imu"
        tree.write(manifest)
        rejected = self.dependency_closure()
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("only supports non-ROS2 SDK dependencies", rejected.stderr)
        self.assertIn("middleware/ros2/peripherals/imu", rejected.stderr)
        self.assertFalse(self.record.exists())

    def test_sdk_with_deps_prebuilds_native_then_dispatches_project_ros(self):
        self.path_python.symlink_to("/bin/false")
        selected_python = self.sdk / "selected interpreter"
        selected_python.symlink_to(sys.executable)
        self.env["SROBOTIS_PYTHON"] = str(selected_python)
        result = self.invoke_with_deps()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        commands = self.commands()
        self.assertEqual(len(commands), 28)
        self.assertEqual(
            [item["args"][1] for item in commands[:9] if item["args"][0] == "-S"],
            [str(self.sdk / key) for key in (
                "components/multimedia/mpp",
                "components/perception/stereo_matching",
                "components/peripherals/imu",
            )],
        )
        self.assertEqual(
            [item["sdk_lock_held"] for item in commands[:9]],
            [False, False, True] * 3,
        )
        self.assertTrue(all(item["sdk_lock_held"] for item in commands[9:]))
        self.assertEqual([item["tool"] for item in commands].count("colcon"), 1)
        native_builds = [item for item in commands[9:-1] if item["args"][0] == "--build"]
        self.assertEqual(len(native_builds), 6)
        for item in native_builds:
            self.assertEqual(item["args"][-2:], ["--parallel", "8"])
            self.assertIsNone(item["makeflags"])
            self.assertIsNone(item["cmake_parallel_level"])
        ros = commands[-1]
        self.assertEqual(ros["tool"], "colcon")
        self.assertEqual(ros["makeflags"], "-j1")
        self.assertEqual(ros["cmake_parallel_level"], "1")
        args = ros["args"]
        self.assertEqual(args[args.index("--parallel-workers") + 1], "2")
        start, end = args.index("--base-paths") + 1, args.index("--packages-select")
        self.assertEqual(args[start:end], [
            str(self.project / "src"),
            str(self.sdk / "middleware/ros2/peripherals/imu"),
            str(self.sdk / "components/multimedia/mpp/test/ros2_mpp_msgs"),
        ])

    def test_sdk_with_deps_project_failure_stops_before_ros(self):
        self.env["TEST_FAIL_SOURCE"] = "modules/camera"
        result = self.invoke_with_deps()
        self.assertEqual(result.returncode, 17, result.stdout + result.stderr)
        self.assertEqual(len(self.commands()), 13)
        self.assertTrue(all(item["tool"] == "cmake" for item in self.commands()))

    def test_missing_project_system_dependency_stops_before_build_or_install(self):
        self.env["TEST_MISSING_SYSTEM_DEP"] = "libavformat-dev"
        result = self.invoke_with_deps()
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Missing required dependencies: libavformat-dev", result.stdout)
        self.assertIn("System dependency check/installation failed", result.stderr)
        checked = self.system_record.read_text().splitlines()
        required = [
            dependency.text.strip()
            for dependency in ET.parse(self.project / "package.xml").getroot().findall("system_depend")
        ]
        self.assertIn("libavformat-dev", required)
        self.assertEqual(set(checked), set(required))
        self.assertFalse(self.record.exists())
        self.assertFalse(self.apt_record.exists())

    def test_unvalidated_cross_build_is_explicitly_rejected(self):
        result = self.invoke("--cross")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("not validated", result.stderr)
        self.assertFalse(self.record.exists())

    def test_sdk_entry_reports_unusable_interpreter(self):
        result = subprocess.run(
            [str(self.project / "build.sh"), "--help"],
            env=dict(self.env, SROBOTIS_PYTHON="/bin/false"),
            capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        self.assertIn("Python 3.6 or later", result.stderr)
        self.assertIn("SROBOTIS_PYTHON", result.stderr)
        self.assertFalse(self.record.exists())


if __name__ == "__main__":
    unittest.main()
