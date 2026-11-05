"""Regression tests for QA discovery and build inventory."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
RUNNER_PATH = ROOT / "scripts" / "qa" / "run.py"
SPEC = importlib.util.spec_from_file_location("firmware_qa_runner", RUNNER_PATH)
assert SPEC and SPEC.loader
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


class DiscoveryTests(unittest.TestCase):
    def test_application_control_retry_only_matches_winerror_4551(self) -> None:
        blocked = OSError("blocked")
        blocked.winerror = 4551
        other = OSError("other")
        other.winerror = 5
        self.assertTrue(RUNNER.application_control_blocked(blocked))
        self.assertFalse(RUNNER.application_control_blocked(other))

    def test_arm_bare_metal_target_is_not_native(self) -> None:
        self.assertFalse(RUNNER.is_native_target("arm-none-eabi"))
        self.assertTrue(RUNNER.is_native_target("x86_64-w64-mingw32"))
        self.assertTrue(RUNNER.is_native_target("x86_64-linux-gnu"))

    def test_explicit_and_environment_compilers_precede_path_defaults(self) -> None:
        candidates = RUNNER.host_compiler_candidates(
            "C:/explicit/gcc.exe",
            {"QA_HOST_CC": "C:/environment/gcc.exe", "CC": "arm-none-eabi-gcc"},
            "Windows",
        )
        self.assertEqual("C:/explicit/gcc.exe", candidates[0])
        self.assertEqual("C:/environment/gcc.exe", candidates[1])
        self.assertIn(r"C:\msys64\ucrt64\bin\gcc.exe", candidates)

    def test_sdk_version_is_read_from_sdk_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "pico_sdk_version.cmake").write_text(
                "set(PICO_SDK_VERSION_MAJOR 2)\n"
                "set(PICO_SDK_VERSION_MINOR 3)\n"
                "set(PICO_SDK_VERSION_REVISION 1)\n",
                encoding="utf-8",
            )
            self.assertEqual("2.3.1", RUNNER.sdk_version(path))

    def test_windows_host_binaries_default_outside_the_workspace(self) -> None:
        output = RUNNER.default_host_build_dir("Windows", Path("C:/Temp"))
        self.assertEqual(Path("C:/Temp/inf2004-firmware-qa-host"), output)


class InventoryTests(unittest.TestCase):
    def test_inventory_covers_every_supported_firmware_target_once(self) -> None:
        config = json.loads((ROOT / "scripts" / "qa" / "targets.json").read_text(encoding="utf-8"))
        actual = [target["name"]
                  for group in config["build_groups"]
                  for target in group["targets"]]
        expected = {
            "pico_rssi_tracker",
            "picow_access_point",
            "picow_access_point2",
            "picow_access_point3",
            "getting_there",
            "rssi_env_test",
            "wheel_rssi_test",
            "wheel_turn_check",
            "tcs34725_diag",
        }
        self.assertEqual(expected, set(actual))
        self.assertEqual(len(actual), len(set(actual)), "duplicate firmware target in QA inventory")

    def test_pico_sdk_baseline_matches_project_configuration(self) -> None:
        config = json.loads((ROOT / "scripts" / "qa" / "targets.json").read_text(encoding="utf-8"))
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertEqual("2.3.1", config["pico_sdk"])
        self.assertIn("set(sdkVersion 2.3.1)", cmake)

    def test_runner_uses_fresh_cmake_configuration_for_sdk_migrations(self) -> None:
        runner = RUNNER_PATH.read_text(encoding="utf-8")
        self.assertIn('[cmake, "--fresh", "-S"', runner)


if __name__ == "__main__":
    unittest.main()
