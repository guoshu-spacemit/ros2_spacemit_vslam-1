#!/usr/bin/env python3
"""Verify binary SDK packaging and CMake exports without loading a target library.

Small ELF header fixtures exercise packaging validation only. These tests do
not validate RISC-V machine instructions, ABI linking or hardware behavior.
"""

import hashlib
import json
from pathlib import Path
import platform
import subprocess
import tempfile
import unittest


HELPER = Path(__file__).resolve().with_name("binary_sdk.cmake")


class BinarySdkMetadataTest(unittest.TestCase):
    """Exercise the actual installer against malformed release artifacts."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="spacemit-binary-sdk-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / "source"
        self.source.mkdir()
        self.licenses = self.source / "licenses"
        files = {
            "include/spacemit_fixture.h": b"struct Fixture { int value; };\n",
            "lib/cmake/spacemit_fixture/spacemit_fixtureConfig.cmake":
                b"set(spacemit_fixture_FOUND TRUE)\n",
            "lib/cmake/spacemit_fixture/spacemit_fixtureTargets.cmake":
                b"# Packaging fixture; no compiled consumer claim.\n",
            "licenses/LICENSE": b"Fixture license\n",
            "licenses/NOTICE": b"Fixture notices\n",
        }
        header = bytearray(64)
        header[0:7] = b"\x7fELF\x02\x01\x01"
        header[16:18] = (3).to_bytes(2, "little")
        header[18:20] = (243).to_bytes(2, "little")
        header[20:24] = (1).to_bytes(4, "little")
        header[48:52] = (5).to_bytes(4, "little")
        files["lib/libspacemit_fixture.so.1"] = header
        for name, content in files.items():
            file = self.source / name
            file.parent.mkdir(parents=True, exist_ok=True)
            file.write_bytes(content)
        (self.source / "lib/libspacemit_fixture.so").symlink_to(
            "libspacemit_fixture.so.1")
        self.manifest = {
            "schema_version": 1,
            "package": "spacemit_fixture",
            "target": {
                "system": "Linux", "processor": "riscv64", "elf_class": 64,
                "endianness": "little", "isa": "rv64gcv", "abi": "lp64d",
            },
            "libraries": ["lib/libspacemit_fixture.so", "lib/libspacemit_fixture.so.1"],
        }
        self.refresh_manifest()
        (self.source / "CMakeLists.txt").write_text(
            "cmake_minimum_required(VERSION 3.19)\n"
            "project(spacemit_fixture LANGUAGES NONE)\n"
            f'include("{HELPER}")\n'
            'spacemit_install_binary_sdk(spacemit_fixture "licenses")\n')
        self.toolchain = self.root / "riscv64.cmake"
        self.toolchain.write_text(
            "set(CMAKE_SYSTEM_NAME Linux)\n"
            "set(CMAKE_SYSTEM_PROCESSOR riscv64)\n")

    def refresh_manifest(self):
        """Update hashes after an intentional artifact mutation."""
        self.manifest["files"] = {
            file.relative_to(self.source).as_posix(): hashlib.sha256(file.read_bytes()).hexdigest()
            for directory in ("include", "lib")
            for file in (self.source / directory).rglob("*")
            if file.is_file()
        }
        (self.source / "manifest.json").write_text(
            json.dumps(self.manifest, indent=2) + "\n")

    def configure(self, toolchain=True):
        """Configure the real installer with the selected target metadata."""
        command = ["cmake", "-S", str(self.source), "-B", str(self.root / "build"),
                   f"-DCMAKE_INSTALL_PREFIX={self.root}/prefix"]
        if toolchain:
            command.append(f"-DCMAKE_TOOLCHAIN_FILE={self.toolchain}")
        return subprocess.run(command, text=True, capture_output=True, timeout=30)

    def reject(self, diagnostic):
        result = self.configure()
        output = result.stdout + result.stderr
        self.assertNotEqual(result.returncode, 0, output)
        self.assertIn(diagnostic, " ".join(output.split()))

    def test_target_riscv64_can_install_relocatable_payload(self):
        result = self.configure()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        subprocess.run(["cmake", "--install", str(self.root / "build")],
                       text=True, capture_output=True, check=True, timeout=30)
        prefix = self.root / "prefix"
        self.assertTrue((prefix / "lib/libspacemit_fixture.so").is_symlink())
        self.assertEqual((prefix / "lib/libspacemit_fixture.so").readlink(),
                         Path("libspacemit_fixture.so.1"))
        self.assertEqual((prefix / "include/spacemit_fixture.h").read_bytes(),
                         (self.source / "include/spacemit_fixture.h").read_bytes())
        self.assertEqual((prefix / "share/licenses/spacemit-vslam/NOTICE").read_bytes(),
                         (self.licenses / "NOTICE").read_bytes())
        self.assertTrue((prefix / "share/spacemit_fixture/binary_sdk/manifest.json").is_file())
        self.assertFalse((prefix / "manifest.json").exists())

    @unittest.skipIf(platform.machine().lower().startswith("riscv64"),
                     "native host is already a supported target")
    def test_native_non_riscv_target_is_rejected(self):
        result = self.configure(toolchain=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("contains Linux/riscv64 prebuilt libraries", result.stderr)

    def test_changed_public_header_is_rejected(self):
        (self.source / "include/spacemit_fixture.h").write_text("changed\n")
        self.reject("SHA256 mismatch")

    def test_missing_central_notice_is_rejected(self):
        (self.licenses / "NOTICE").unlink()
        self.reject("NOTICE")

    def test_unlisted_file_is_rejected(self):
        (self.source / "include/unlisted.cpp").write_text("unexpected implementation\n")
        self.reject("payload and manifest file inventory differ")

    def test_wrong_machine_is_rejected_even_with_valid_checksum(self):
        file = self.source / "lib/libspacemit_fixture.so.1"
        header = bytearray(file.read_bytes())
        header[18:20] = (62).to_bytes(2, "little")
        file.write_bytes(header)
        self.refresh_manifest()
        self.reject("not a Linux RISC-V ELF64 little-endian shared object")

    def test_wrong_float_abi_is_rejected_even_with_valid_checksum(self):
        file = self.source / "lib/libspacemit_fixture.so.1"
        header = bytearray(file.read_bytes())
        header[48:52] = (1).to_bytes(4, "little")
        file.write_bytes(header)
        self.refresh_manifest()
        self.reject("does not use the declared LP64D floating-point ABI")

    def test_declared_architecture_mismatch_is_rejected(self):
        self.manifest["target"]["processor"] = "x86_64"
        self.refresh_manifest()
        self.reject("expected a declared Linux/riscv64")

    def test_absolute_symlink_is_rejected(self):
        link = self.source / "lib/libspacemit_fixture.so"
        link.unlink()
        link.symlink_to(self.source / "lib/libspacemit_fixture.so.1")
        self.refresh_manifest()
        self.reject("binary SDK symlinks must be relative")

    def test_symlink_escaping_payload_is_rejected(self):
        outside = self.root / "outside.txt"
        outside.write_text("external artifact\n")
        (self.source / "include/outside.txt").symlink_to("../../outside.txt")
        self.refresh_manifest()
        self.reject("binary SDK symlink escapes its payload")

    def test_directory_symlink_is_rejected(self):
        outside = self.root / "outside"
        outside.mkdir()
        (self.source / "include/linked-directory").symlink_to(outside)
        self.reject("directory symlinks are not supported")


class BinarySdkConsumerTest(unittest.TestCase):
    """Install and relocate both SDK packages without loading their target libraries."""

    def _run_cmake(self, command):
        result = subprocess.run(command, text=True, capture_output=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_installed_packages_use_flat_headers_and_relocate(self):
        """Resolve both CMake targets after moving their shared install prefix."""
        repo = HELPER.parent.parent
        packages = {
            "vslam": "libspacemit_vslam.so",
            "mapping": "libspacemit_mapping.so",
        }
        features = json.loads((repo / "modules/vslam/manifest.json").read_text())["features"]
        map_io = "ON" if features["map_persistence"] and features["relocalization"] else "OFF"
        rvv = "ON" if features["rvv_implementation"] else "OFF"
        with tempfile.TemporaryDirectory(prefix="spacemit-sdk-consumer-") as temporary:
            root = Path(temporary)
            toolchain = root / "riscv64.cmake"
            toolchain.write_text(
                "set(CMAKE_SYSTEM_NAME Linux)\nset(CMAKE_SYSTEM_PROCESSOR riscv64)\n")
            prefix = root / "prefix"
            configure = ["cmake", f"-DCMAKE_TOOLCHAIN_FILE={toolchain}"]
            for name, library_name in packages.items():
                package = f"spacemit_{name}"
                module = repo / "modules" / name
                self.assertFalse((module / "sdk").exists())
                self.assertFalse((module / "share").exists())
                build = root / name
                self._run_cmake([
                    *configure, "-S", module, "-B", build, f"-DCMAKE_INSTALL_PREFIX={prefix}",
                ])
                self._run_cmake(["cmake", "--install", build])

                manifest_path = module / "manifest.json"
                manifest = json.loads(manifest_path.read_text())
                header = f"include/{package}.h"
                self.assertIn(header, manifest["files"])
                self.assertEqual((prefix / header).read_bytes(), (module / header).read_bytes())
                self.assertFalse((prefix / "include" / package).exists())
                installed_manifest = prefix / "share" / package / "binary_sdk/manifest.json"
                self.assertEqual(installed_manifest.read_bytes(), manifest_path.read_bytes())
                library = f"lib/{library_name}"
                self.assertEqual(manifest["libraries"], [library])
                source = module / library
                installed = prefix / library
                self.assertFalse(source.is_symlink())
                self.assertFalse(installed.is_symlink())
                self.assertEqual(installed.read_bytes(), source.read_bytes())
                config = prefix / "lib/cmake" / package
                self.assertFalse((config / f"{package}ConfigVersion.cmake").exists())

            for filename in ("LICENSE", "NOTICE"):
                installed_license = prefix / "share/licenses/spacemit-vslam" / filename
                self.assertEqual(installed_license.read_bytes(), (repo / filename).read_bytes())
            for name in packages:
                self.assertFalse((prefix / "share/licenses" / f"spacemit_{name}").exists())

            relocated_prefix = root / "relocated"
            prefix.rename(relocated_prefix)

            consumer = root / "consumer"
            consumer.mkdir()
            # The package metadata can be checked without the target OpenCV libraries.
            opencv = root / "opencv"
            opencv.mkdir()
            (opencv / "OpenCVConfigVersion.cmake").write_text(
                "set(PACKAGE_VERSION 4.10.0)\nset(PACKAGE_VERSION_COMPATIBLE TRUE)\n")
            (opencv / "OpenCVConfig.cmake").write_text(
                "set(OpenCV_VERSION 4.10.0)\n"
                "foreach(component IN LISTS OpenCV_FIND_COMPONENTS)\n"
                "  add_library(opencv_${component} INTERFACE IMPORTED)\n"
                "endforeach()\n")
            consumer_cmake = [
                "cmake_minimum_required(VERSION 3.19)\n"
                "project(sdk_consumer LANGUAGES NONE)\n",
            ]
            for name, library in packages.items():
                package = f"spacemit_{name}"
                target = f"{package}::{package}"
                optional = " OPTIONAL_COMPONENTS map_io" if name == "vslam" else ""
                consumer_cmake.append(
                    f"find_package({package} CONFIG REQUIRED COMPONENTS functional{optional})\n"
                    f"get_target_property(sdk_library {target} IMPORTED_LOCATION_RELEASE)\n"
                    f"get_target_property(sdk_soname {target} IMPORTED_SONAME_RELEASE)\n"
                    f"get_target_property(sdk_includes {target} INTERFACE_INCLUDE_DIRECTORIES)\n"
                    f'if(NOT sdk_library STREQUAL "{relocated_prefix}/lib/{library}" OR\n'
                    f'   NOT sdk_soname STREQUAL "{library}" OR\n'
                    f'   NOT sdk_includes STREQUAL "{relocated_prefix}/include" OR\n'
                    f'   NOT {package}_PUBLIC_HEADER STREQUAL '
                    f'"{relocated_prefix}/include/{package}.h")\n'
                    f'  message(FATAL_ERROR "{package} did not resolve its relocated SDK")\n'
                    "endif()\n")
            consumer_cmake.append(
                f'if(NOT spacemit_vslam_map_io_FOUND STREQUAL "{map_io}" OR '
                f'NOT spacemit_vslam_WITH_RVV STREQUAL "{rvv}")\n'
                '  message(FATAL_ERROR "VSLAM package capabilities differ from manifest")\n'
                "endif()\n")
            (consumer / "CMakeLists.txt").write_text("".join(consumer_cmake))
            self._run_cmake([
                *configure, "-S", consumer, "-B", root / "consumer-build",
                f"-DCMAKE_PREFIX_PATH={relocated_prefix}", f"-DOpenCV_DIR={opencv}",
            ])


if __name__ == "__main__":
    unittest.main()
