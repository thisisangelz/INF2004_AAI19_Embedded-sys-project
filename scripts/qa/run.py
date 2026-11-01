#!/usr/bin/env python3
"""Shared local, agent, and CI entry point for firmware software QA."""

from __future__ import annotations

import argparse
import datetime as dt
import html
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
ARTIFACTS = ROOT / "artifacts" / "qa"
CONFIG = json.loads((Path(__file__).with_name("targets.json")).read_text(encoding="utf-8"))


class Result:
    def __init__(self, name: str, status: str, mandatory: bool, evidence: str,
                 duration: float, command: str = "", output: str = "") -> None:
        self.name = name
        self.status = status
        self.mandatory = mandatory
        self.evidence = evidence
        self.duration = round(duration, 3)
        self.command = command
        self.output = output[-30000:]

    def as_dict(self) -> dict[str, object]:
        return vars(self)


RESULTS: list[Result] = []


def executable(*names: str) -> str | None:
    for name in names:
        found = shutil.which(name)
        if found:
            return found
    return None


def run_step(name: str, command: list[str], *, mandatory: bool,
             evidence: str, cwd: Path = ROOT, env: dict[str, str] | None = None) -> bool:
    started = time.monotonic()
    rendered = subprocess.list2cmdline(command)
    try:
        completed = subprocess.run(
            command, cwd=cwd, env=env, text=True, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, errors="replace", check=False
        )
        passed = completed.returncode == 0
        output = completed.stdout
    except OSError as exc:
        passed = False
        output = str(exc)
    RESULTS.append(Result(name, "PASS" if passed else "FAIL", mandatory, evidence,
                          time.monotonic() - started, rendered, output))
    print(f"[{'PASS' if passed else 'FAIL'}] {name}")
    if not passed:
        print(output[-4000:])
    return passed


def skipped(name: str, reason: str, *, mandatory: bool, evidence: str) -> None:
    RESULTS.append(Result(name, "FAIL" if mandatory else "SKIP", mandatory,
                          evidence, 0.0, output=reason))
    print(f"[{'FAIL' if mandatory else 'SKIP'}] {name}: {reason}")


def find_sdk(explicit: str | None) -> Path | None:
    candidates = [explicit, os.environ.get("PICO_SDK_PATH"),
                  r"C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\pico-sdk"]
    for value in candidates:
        if value and (Path(value) / "pico_sdk_init.cmake").is_file():
            return Path(value).resolve()
    return None


def find_ninja() -> str | None:
    bundled = Path(r"C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\ninja\ninja.exe")
    return executable("ninja") or (str(bundled) if bundled.is_file() else None)


def find_pico_toolchain() -> Path | None:
    candidates = [os.environ.get("PICO_TOOLCHAIN_PATH"),
                  r"C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\gcc-arm-none-eabi"]
    for value in candidates:
        if value and Path(value).is_dir():
            return Path(value).resolve()
    return None


def compiler_version(command: str | None) -> str:
    if not command:
        return "not found"
    try:
        result = subprocess.run([command, "--version"], text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                errors="replace", check=False)
        return result.stdout.splitlines()[0] if result.stdout else command
    except OSError:
        return command


def compile_qa_tests(cc: str, coverage: bool) -> None:
    output = ARTIFACTS / "host"
    output.mkdir(parents=True, exist_ok=True)
    common = [cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O0", "-g",
              "-ffunction-sections", "-fdata-sections", "-I", str(ROOT / "qa" / "stubs"),
              "-I", str(ROOT), "-I", str(ROOT / "getting_there"),
              "-I", str(ROOT / "pico_rssi_tracker")]
    if coverage:
        common.append("--coverage")
    for test in ("packed_bits", "transfer_protocol", "shared_state", "imu_processing",
                 "motion_safety"):
        suffix = ".exe" if os.name == "nt" else ""
        binary = output / f"test_{test}{suffix}"
        command = common + [str(ROOT / "qa" / "tests" / f"test_{test}.c"),
                            "-Wl,--gc-sections", "-lm", "-o", str(binary)]
        if run_step(f"host test build: {test}", command, mandatory=True,
                    evidence="HOST_VERIFIED"):
            run_step(f"host test run: {test}", [str(binary)], mandatory=True,
                     evidence="HOST_VERIFIED")


def existing_host_tests(cc: str, sdk: Path, coverage: bool) -> None:
    command = [sys.executable, str(ROOT / "getting_there" / "tests" / "run_host_checks.py"),
               "--cc", cc, "--sdk", str(sdk), "--output-dir", str(ARTIFACTS / "host")]
    if coverage:
        command.append("--coverage")
    run_step("existing navigation and BLE host tests", command, mandatory=True,
             evidence="HOST_VERIFIED")


def cmake_builds(sdk: Path, toolchain: Path | None) -> list[Path]:
    cmake = executable("cmake")
    ninja = find_ninja()
    if not cmake:
        skipped("Pico firmware builds", "cmake not found", mandatory=True,
                evidence="FIRMWARE_COMPILE_VERIFIED")
        return []
    if not ninja:
        skipped("Pico firmware builds", "ninja not found", mandatory=True,
                evidence="FIRMWARE_COMPILE_VERIFIED")
        return []
    databases: list[Path] = []
    for target in CONFIG["targets"]:
        build = ARTIFACTS / "build" / target["name"]
        source = ROOT / target["source"]
        configure = [cmake, "-S", str(source), "-B", str(build), "-G", "Ninja",
                     f"-DCMAKE_MAKE_PROGRAM={ninja}", f"-DPICO_SDK_PATH={sdk}",
                     "-DPICO_BOARD=pico_w", "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
                     "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"]
        if toolchain:
            configure.append(f"-DPICO_TOOLCHAIN_PATH={toolchain}")
        ok = run_step(f"configure: {target['name']}", configure, mandatory=True,
                      evidence="FIRMWARE_COMPILE_VERIFIED")
        database = build / "compile_commands.json"
        if database.is_file():
            databases.append(database)
        if ok:
            run_step(f"firmware build: {target['name']}",
                     [cmake, "--build", str(build), "--parallel", "2"], mandatory=True,
                     evidence="FIRMWARE_COMPILE_VERIFIED")
    return databases


def filtered_database(databases: list[Path]) -> Path | None:
    entries: list[dict[str, object]] = []
    root = ROOT.resolve()
    for database in databases:
        try:
            values = json.loads(database.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        for item in values:
            source = Path(str(item.get("file", ""))).resolve()
            try:
                relative = source.relative_to(root)
            except ValueError:
                continue
            text = relative.as_posix()
            if not text.startswith(("artifacts/", "build/")) and "/build/" not in text:
                entries.append(item)
    if not entries:
        return None
    destination = ARTIFACTS / "compile_commands.project.json"
    destination.write_text(json.dumps(entries, indent=2), encoding="utf-8")
    return destination


def static_analysis(databases: list[Path]) -> None:
    database = filtered_database(databases)
    if not database:
        skipped("static analysis", "no project compilation database", mandatory=False,
                evidence="SOURCE_CONFIRMED")
        return
    cppcheck = executable("cppcheck")
    if cppcheck:
        run_step("Cppcheck advisory", [cppcheck, f"--project={database}",
                 "--enable=warning,style,performance,portability", "--inline-suppr",
                 f"--suppressions-list={ROOT / 'scripts' / 'qa' / 'cppcheck-suppressions.txt'}",
                 "--error-exitcode=0"], mandatory=False, evidence="SOURCE_CONFIRMED")
    else:
        skipped("Cppcheck advisory", "cppcheck not installed", mandatory=False,
                evidence="SOURCE_CONFIRMED")

    tidy = executable("clang-tidy")
    if not tidy:
        skipped("Clang-Tidy advisory", "clang-tidy not installed", mandatory=False,
                evidence="SOURCE_CONFIRMED")
        return
    entries = json.loads(database.read_text(encoding="utf-8"))
    files = sorted({str(Path(str(entry["file"])).resolve()) for entry in entries})
    if files:
        # clang-tidy expects the conventional filename inside its -p directory.
        tidy_dir = ARTIFACTS / "tidy"
        tidy_dir.mkdir(parents=True, exist_ok=True)
        (tidy_dir / "compile_commands.json").write_text(
            database.read_text(encoding="utf-8"), encoding="utf-8")
        run_step("Clang-Tidy advisory", [tidy, "-p", str(tidy_dir), *files],
                 mandatory=False, evidence="SOURCE_CONFIRMED")


def coverage_report() -> None:
    gcovr = executable("gcovr")
    if not gcovr:
        skipped("Gcovr advisory", "gcovr not installed", mandatory=False,
                evidence="HOST_VERIFIED")
        return
    run_step("Gcovr advisory", [gcovr, "--config", str(ROOT / "gcovr.cfg"),
             "--xml", str(ARTIFACTS / "coverage.xml"), "--html-details",
             str(ARTIFACTS / "coverage.html"), "--txt",
             str(ARTIFACTS / "coverage.txt")], mandatory=False, evidence="HOST_VERIFIED")


def write_reports(mode: str, tools: dict[str, str]) -> None:
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": 1,
        "generated_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "mode": mode,
        "repository": str(ROOT),
        "tools": tools,
        "hardware": {
            "motor_pwm_off_20ms": "HARDWARE_UNVERIFIED",
            "note": "Host and software checks cannot close the physical timing requirement."
        },
        "results": [result.as_dict() for result in RESULTS]
    }
    (ARTIFACTS / "report.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")

    lines = ["# Firmware QA report", "", f"Mode: `{mode}`", "",
             "Physical 20 ms PWM-off requirement: **HARDWARE_UNVERIFIED**", "",
             "| Check | Class | Evidence | Status | Duration |",
             "|---|---|---|---:|---:|"]
    for result in RESULTS:
        lines.append(f"| {result.name} | {'mandatory' if result.mandatory else 'advisory'} | "
                     f"{result.evidence} | {result.status} | {result.duration:.3f}s |")
    lines += ["", "## Tool versions", ""]
    lines.extend(f"- `{key}`: {value}" for key, value in tools.items())
    failures = [item for item in RESULTS if item.status == "FAIL"]
    if failures:
        lines += ["", "## Failures", ""]
        for item in failures:
            lines += [f"### {item.name}", "", f"Command: `{item.command}`", "", "```text",
                      item.output[-6000:], "```", ""]
    (ARTIFACTS / "report.md").write_text("\n".join(lines), encoding="utf-8")

    cases = []
    for result in RESULTS:
        failure = ""
        if result.status == "FAIL":
            failure = f'<failure message="check failed">{html.escape(result.output)}</failure>'
        elif result.status == "SKIP":
            failure = f'<skipped message="{html.escape(result.output)}" />'
        cases.append(f'<testcase name="{html.escape(result.name)}" time="{result.duration}">{failure}</testcase>')
    failed = sum(item.status == "FAIL" for item in RESULTS)
    junit = (f'<?xml version="1.0" encoding="UTF-8"?>\n<testsuite name="firmware-qa" '
             f'tests="{len(RESULTS)}" failures="{failed}">\n' + "\n".join(cases) +
             "\n</testsuite>\n")
    (ARTIFACTS / "junit.xml").write_text(junit, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("quick", "full", "hardware-report"), nargs="?",
                        default="quick")
    parser.add_argument("--cc", help="native C compiler for host tests")
    parser.add_argument("--sdk", help="Pico SDK path")
    args = parser.parse_args()
    ARTIFACTS.mkdir(parents=True, exist_ok=True)

    cc = args.cc or executable("gcc", "clang", "cc")
    sdk = find_sdk(args.sdk)
    toolchain = find_pico_toolchain()
    tools = {
        "python": sys.version.splitlines()[0],
        "platform": platform.platform(),
        "host_cc": compiler_version(cc),
        "cmake": compiler_version(executable("cmake")),
        "cppcheck": compiler_version(executable("cppcheck")),
        "clang-tidy": compiler_version(executable("clang-tidy")),
        "gcovr": compiler_version(executable("gcovr")),
        "pico_sdk": str(sdk) if sdk else "not found",
        "pico_toolchain": str(toolchain) if toolchain else "PATH/default"
    }

    if args.mode == "hardware-report":
        skipped("20 ms motor PWM-off measurement", "physical evidence not recorded",
                mandatory=False, evidence="HARDWARE_UNVERIFIED")
    else:
        if not cc:
            skipped("host tests", "native gcc/clang/cc not found", mandatory=True,
                    evidence="HOST_VERIFIED")
        else:
            compile_qa_tests(cc, args.mode == "full")
            if sdk:
                existing_host_tests(cc, sdk, args.mode == "full")
            else:
                skipped("existing navigation and BLE host tests", "Pico SDK not found",
                        mandatory=True, evidence="HOST_VERIFIED")
        if args.mode == "full":
            if sdk:
                databases = cmake_builds(sdk, toolchain)
                static_analysis(databases)
            else:
                skipped("Pico firmware builds", "Pico SDK not found", mandatory=True,
                        evidence="FIRMWARE_COMPILE_VERIFIED")
                skipped("static analysis", "Pico SDK/compilation database unavailable",
                        mandatory=False, evidence="SOURCE_CONFIRMED")
            coverage_report()

    write_reports(args.mode, tools)
    mandatory_failures = [result for result in RESULTS if result.mandatory and result.status == "FAIL"]
    print(f"Reports: {ARTIFACTS / 'report.md'}")
    return 1 if mandatory_failures else 0


if __name__ == "__main__":
    raise SystemExit(main())

