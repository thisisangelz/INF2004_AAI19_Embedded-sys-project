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
import re
import shutil
import subprocess
import sys
import tempfile
import time


ROOT = Path(__file__).resolve().parents[2]
ARTIFACTS = ROOT / "artifacts" / "qa"
CONFIG_PATH = Path(__file__).with_name("targets.json")
CONFIG = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
RESULTS: list[Result] = []
TOOL_VERSIONS: dict[str, str] = {}


class Result:
    def __init__(self, name: str, status: str, mandatory: bool, evidence: str,
                 duration: float, *, tool: str = "", command: str = "",
                 output: str = "") -> None:
        self.name = name
        self.status = status
        self.mandatory = mandatory
        self.evidence = evidence
        self.duration = round(duration, 3)
        self.tool = tool
        self.tool_version = TOOL_VERSIONS.get(tool, "not recorded") if tool else "n/a"
        self.command = command
        self.output = output[-30000:]

    def as_dict(self) -> dict[str, object]:
        return vars(self)


def record(name: str, status: str, *, mandatory: bool, evidence: str,
           tool: str = "", duration: float = 0.0, command: str = "",
           output: str = "") -> bool:
    RESULTS.append(Result(name, status, mandatory, evidence, duration, tool=tool,
                          command=command, output=output))
    suffix = f": {output.splitlines()[0]}" if output and status in {"FAIL", "SKIP", "NOT_INSTALLED"} else ""
    print(f"[{status}] {name}{suffix}")
    return status != "FAIL"


def application_control_blocked(exc: OSError) -> bool:
    return getattr(exc, "winerror", None) == 4551


def run_step(name: str, command: list[str], *, mandatory: bool,
             evidence: str, tool: str, cwd: Path = ROOT,
             env: dict[str, str] | None = None,
             warning_markers: tuple[str, ...] = (),
             application_control_retries: int = 0) -> bool:
    started = time.monotonic()
    rendered = subprocess.list2cmdline(command)
    for attempt in range(application_control_retries + 1):
        try:
            completed = subprocess.run(
                command, cwd=cwd, env=env, text=True, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, errors="replace", check=False
            )
            output = completed.stdout
            if completed.returncode != 0:
                status = "FAIL"
            elif warning_markers and any(marker in output for marker in warning_markers):
                status = "WARN"
            else:
                status = "PASS"
            break
        except OSError as exc:
            if application_control_blocked(exc) and attempt < application_control_retries:
                delay = 2 * (attempt + 1)
                print(f"[RETRY] {name}: Application Control inspection; waiting {delay}s")
                time.sleep(delay)
                continue
            status = "FAIL"
            output = str(exc)
            break
    ok = record(name, status, mandatory=mandatory, evidence=evidence, tool=tool,
                duration=time.monotonic() - started, command=rendered, output=output)
    if status == "FAIL":
        print(output[-4000:])
    return ok


def unavailable(name: str, reason: str, *, mandatory: bool,
                evidence: str, tool: str) -> None:
    record(name, "FAIL" if mandatory else "NOT_INSTALLED", mandatory=mandatory,
           evidence=evidence, tool=tool, output=reason)


def skipped(name: str, reason: str, *, mandatory: bool,
            evidence: str, tool: str = "") -> None:
    record(name, "FAIL" if mandatory else "SKIP", mandatory=mandatory,
           evidence=evidence, tool=tool, output=reason)


def resolve_executable(value: str | None) -> str | None:
    if not value:
        return None
    expanded = os.path.expandvars(os.path.expanduser(value.strip().strip('"')))
    path = Path(expanded)
    if path.is_file():
        return str(path.resolve())
    return shutil.which(expanded)


def discover_executable(explicit: str | None, env_names: tuple[str, ...],
                        command_names: tuple[str, ...],
                        known_paths: tuple[str, ...] = ()) -> str | None:
    values = [explicit]
    values.extend(os.environ.get(name) for name in env_names)
    values.extend(command_names)
    values.extend(known_paths)
    seen: set[str] = set()
    for value in values:
        if not value or value in seen:
            continue
        seen.add(value)
        found = resolve_executable(value)
        if found:
            return found
    return None


def host_compiler_candidates(explicit: str | None,
                             environ: dict[str, str] | None = None,
                             system_name: str | None = None) -> list[str]:
    env = os.environ if environ is None else environ
    system = platform.system() if system_name is None else system_name
    values = [explicit, env.get("QA_HOST_CC"), env.get("HOST_CC"), env.get("CC"),
              "gcc", "clang", "cc"]
    if system == "Windows":
        values.extend([
            r"C:\msys64\ucrt64\bin\gcc.exe",
            r"C:\msys64\mingw64\bin\gcc.exe",
            r"C:\Program Files\LLVM\bin\clang.exe",
        ])
    return [value for value in values if value]


def compiler_target(command: str) -> str:
    try:
        result = subprocess.run([command, "-dumpmachine"], text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                errors="replace", check=False)
        return result.stdout.strip().splitlines()[0] if result.stdout.strip() else ""
    except OSError:
        return ""


def is_native_target(target: str) -> bool:
    value = target.lower()
    cross_markers = ("none-eabi", "arm-eabi", "riscv", "avr", "xtensa")
    return bool(value) and not any(marker in value for marker in cross_markers)


def find_host_compiler(explicit: str | None) -> tuple[str | None, str]:
    rejected: list[str] = []
    seen: set[str] = set()
    for candidate in host_compiler_candidates(explicit):
        found = resolve_executable(candidate)
        if not found or found in seen:
            continue
        seen.add(found)
        target = compiler_target(found)
        if is_native_target(target):
            return found, target
        rejected.append(f"{found} ({target or 'unknown target'})")
    detail = "No native gcc/clang/cc found. Install MSYS2 UCRT64 GCC or set --cc/QA_HOST_CC."
    if rejected:
        detail += " Rejected non-native/unsupported candidates: " + ", ".join(rejected)
    return None, detail


def sdk_version(sdk: Path | None) -> str | None:
    if not sdk:
        return None
    version_file = sdk / "pico_sdk_version.cmake"
    try:
        text = version_file.read_text(encoding="utf-8")
    except OSError:
        return None
    values: dict[str, str] = {}
    for key in ("MAJOR", "MINOR", "REVISION"):
        match = re.search(rf"set\(PICO_SDK_VERSION_{key}\s+([0-9]+)\)", text)
        if match:
            values[key] = match.group(1)
    if len(values) != 3:
        return None
    return f"{values['MAJOR']}.{values['MINOR']}.{values['REVISION']}"


def find_sdk(explicit: str | None) -> Path | None:
    baseline = CONFIG["pico_sdk"]
    candidates = [
        explicit,
        os.environ.get("PICO_SDK_PATH"),
        str(Path.home() / ".pico-sdk" / "sdk" / baseline),
        rf"C:\Program Files\Raspberry Pi\Pico SDK v{baseline}\pico-sdk",
        r"C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\pico-sdk",
    ]
    for value in candidates:
        if value:
            path = Path(os.path.expandvars(os.path.expanduser(value)))
            if (path / "pico_sdk_init.cmake").is_file():
                return path.resolve()
    return None


def find_pico_toolchain(explicit: str | None) -> tuple[Path | None, str | None]:
    baseline_dir = CONFIG.get("pico_toolchain_dir", "")
    path_compiler = resolve_executable("arm-none-eabi-gcc")
    candidates = [
        explicit,
        os.environ.get("PICO_TOOLCHAIN_PATH"),
        str(Path.home() / ".pico-sdk" / "toolchain" / baseline_dir),
        r"C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\gcc-arm-none-eabi",
        path_compiler,
    ]
    executable_name = "arm-none-eabi-gcc.exe" if os.name == "nt" else "arm-none-eabi-gcc"
    for value in candidates:
        if not value:
            continue
        path = Path(os.path.expandvars(os.path.expanduser(value)))
        if path.is_file() and path.name.lower().startswith("arm-none-eabi-gcc"):
            return path.parent.parent.resolve(), str(path.resolve())
        compiler = path / "bin" / executable_name
        if compiler.is_file():
            return path.resolve(), str(compiler.resolve())
    return None, None


def find_picotool_dir(explicit: str | None) -> tuple[Path | None, str | None]:
    version = CONFIG.get("picotool", CONFIG["pico_sdk"])
    candidates = [
        explicit,
        os.environ.get("QA_PICOTOOL_DIR"),
        os.environ.get("picotool_DIR"),
        str(Path.home() / ".pico-sdk" / "picotool" / version / "picotool"),
    ]
    executable_name = "picotool.exe" if os.name == "nt" else "picotool"
    for value in candidates:
        if not value:
            continue
        path = Path(os.path.expandvars(os.path.expanduser(value)))
        if (path / "picotoolConfig.cmake").is_file():
            executable = path / executable_name
            return path.resolve(), str(executable.resolve()) if executable.is_file() else None
    return None, None


def find_pioasm_dir(explicit: str | None) -> tuple[Path | None, str | None]:
    version = CONFIG["pico_sdk"]
    candidates = [
        explicit,
        os.environ.get("QA_PIOASM_DIR"),
        os.environ.get("pioasm_DIR"),
        str(Path.home() / ".pico-sdk" / "tools" / version / "pioasm"),
    ]
    executable_name = "pioasm.exe" if os.name == "nt" else "pioasm"
    for value in candidates:
        if not value:
            continue
        path = Path(os.path.expandvars(os.path.expanduser(value)))
        if (path / "pioasmConfig.cmake").is_file():
            executable = path / executable_name
            return path.resolve(), str(executable.resolve()) if executable.is_file() else None
    return None, None


def compiler_version(command: str | None,
                     version_args: tuple[str, ...] = ("--version",)) -> str:
    if not command:
        return "not found"
    try:
        result = subprocess.run([command, *version_args], text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                errors="replace", check=False)
        first = result.stdout.splitlines()[0] if result.stdout else "version unavailable"
        return f"{first} [{command}]"
    except OSError:
        return f"unavailable [{command}]"


def host_environment(cc: str) -> dict[str, str]:
    env = os.environ.copy()
    env["PATH"] = str(Path(cc).parent) + os.pathsep + env.get("PATH", "")
    return env


def supports_gcov_coverage(cc: str | None) -> bool:
    if not cc:
        return False
    name = Path(cc).name.lower()
    return "gcc" in name and is_native_target(compiler_target(cc))


def default_host_build_dir(system_name: str | None = None,
                           temp_root: Path | None = None) -> Path:
    system = platform.system() if system_name is None else system_name
    if system == "Windows":
        base = Path(tempfile.gettempdir()) if temp_root is None else temp_root
        return base / "inf2004-firmware-qa-host"
    return ARTIFACTS / "host"


def tooling_regression_tests() -> None:
    run_step("QA tooling regression tests",
             [sys.executable, "-m", "unittest", "discover", "-s",
              str(ROOT / "scripts" / "qa" / "tests"), "-p", "test_*.py"],
             mandatory=True, evidence="HOST_VERIFIED", tool="python")


def compile_qa_tests(cc: str, coverage: bool, output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    common = [cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O0", "-g",
              "-ffunction-sections", "-fdata-sections", "-I", str(ROOT / "qa" / "stubs"),
              "-I", str(ROOT), "-I", str(ROOT / "getting_there"),
              "-I", str(ROOT / "pico_rssi_tracker")]
    if coverage:
        common.append("--coverage")
    tests = [
        ("packed-bit fields", "packed_bits"),
        ("transfer protocol", "transfer_protocol"),
        ("shared state", "shared_state"),
        ("IMU processing", "imu_processing"),
        ("motor safety", "motion_safety"),
    ]
    env = host_environment(cc)
    for label, source_name in tests:
        suffix = ".exe" if os.name == "nt" else ""
        binary = output / f"test_{source_name}{suffix}"
        command = common + [str(ROOT / "qa" / "tests" / f"test_{source_name}.c"),
                            "-Wl,--gc-sections", "-lm", "-o", str(binary)]
        if run_step(f"host test build: {label}", command, mandatory=True,
                    evidence="HOST_VERIFIED", tool="host_cc", env=env):
            run_step(f"host test run: {label}", [str(binary)], mandatory=True,
                     evidence="HOST_VERIFIED", tool="host_cc", env=env,
                     application_control_retries=3)


def existing_host_tests(cc: str, sdk: Path, coverage: bool, output: Path) -> None:
    labels = {
        "navigation": "navigation",
        "payload_client": "BLE payload protocol",
    }
    env = host_environment(cc)
    for test_name, label in labels.items():
        command = [
            sys.executable,
            str(ROOT / "getting_there" / "tests" / "run_host_checks.py"),
            "--cc", cc,
            "--sdk", str(sdk),
            "--output-dir", str(output),
            "--test", test_name,
        ]
        if coverage:
            command.append("--coverage")
        run_step(f"existing host test: {label}", command, mandatory=True,
                 evidence="HOST_VERIFIED", tool="host_cc", env=env)


def check_artifacts(build: Path, target: dict[str, object]) -> None:
    expected = [build / str(relative) for relative in target.get("artifacts", [])]
    missing = [path for path in expected if not path.is_file()]
    lines = [f"{path}: {'present' if path.is_file() else 'MISSING'}" for path in expected]
    record(f"firmware artifacts: {target['name']}", "FAIL" if missing else "PASS",
           mandatory=True, evidence="FIRMWARE_COMPILE_VERIFIED", tool="cmake",
           output="\n".join(lines))


def cmake_builds(sdk: Path, toolchain: Path, cmake: str,
                 ninja: str, picotool_dir: Path | None,
                 pioasm_dir: Path | None) -> list[Path]:
    databases: list[Path] = []
    for group in CONFIG["build_groups"]:
        build = ARTIFACTS / "build" / group["name"]
        source = ROOT / group["source"]
        configure = [cmake, "--fresh", "-S", str(source), "-B", str(build), "-G", "Ninja",
                     f"-DCMAKE_MAKE_PROGRAM={ninja}", f"-DPICO_SDK_PATH={sdk}",
                     "-DPICO_BOARD=pico_w", "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
                     "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
                     f"-DPICO_TOOLCHAIN_PATH={toolchain}"]
        if picotool_dir:
            configure.append(f"-Dpicotool_DIR={picotool_dir}")
        if pioasm_dir:
            configure.append(f"-Dpioasm_DIR={pioasm_dir}")
        configured = run_step(f"configure build group: {group['name']}", configure,
                              mandatory=True, evidence="FIRMWARE_COMPILE_VERIFIED",
                              tool="cmake")
        if not configured:
            continue
        database = build / "compile_commands.json"
        if database.is_file():
            databases.append(database)
        for target in group["targets"]:
            built = run_step(f"firmware build: {target['name']}",
                             [cmake, "--build", str(build), "--target",
                              str(target["name"]), "--parallel", "2"],
                             mandatory=True, evidence="FIRMWARE_COMPILE_VERIFIED",
                             tool="cmake")
            if built:
                check_artifacts(build, target)
    return databases


def filtered_database(databases: list[Path]) -> Path | None:
    entries: list[dict[str, object]] = []
    seen_files: set[str] = set()
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
            parts = {part.lower() for part in relative.parts}
            if parts.intersection({"artifacts", "build", "stubs"}):
                continue
            key = str(source)
            if key not in seen_files:
                entries.append(item)
                seen_files.add(key)
    if not entries:
        return None
    destination = ARTIFACTS / "compile_commands.project.json"
    destination.write_text(json.dumps(entries, indent=2), encoding="utf-8")
    return destination


def static_analysis(databases: list[Path], cppcheck: str | None,
                    tidy: str | None) -> None:
    database = filtered_database(databases)
    if not database:
        skipped("static analysis", "no project compilation database",
                mandatory=False, evidence="SOURCE_CONFIRMED")
        return
    if cppcheck:
        run_step(
            "Cppcheck advisory",
            [cppcheck, f"--project={database}",
             "--enable=warning,style,performance,portability", "--inline-suppr",
             "--quiet", "--template={file}:{line}:{column}: [{severity}] {message} [{id}]",
             f"--suppressions-list={ROOT / 'scripts' / 'qa' / 'cppcheck-suppressions.txt'}"],
            mandatory=False, evidence="SOURCE_CONFIRMED", tool="cppcheck",
            warning_markers=("[warning]", "[style]", "[performance]", "[portability]", "[error]"),
        )
    else:
        unavailable("Cppcheck advisory", "cppcheck not installed; set --cppcheck or QA_CPPCHECK",
                    mandatory=False, evidence="SOURCE_CONFIRMED", tool="cppcheck")

    if not tidy:
        unavailable("Clang-Tidy advisory", "clang-tidy not installed; set --clang-tidy or QA_CLANG_TIDY",
                    mandatory=False, evidence="SOURCE_CONFIRMED", tool="clang_tidy")
        return
    entries = json.loads(database.read_text(encoding="utf-8"))
    files = sorted({str(Path(str(entry["file"])).resolve()) for entry in entries})
    if files:
        tidy_dir = ARTIFACTS / "tidy"
        tidy_dir.mkdir(parents=True, exist_ok=True)
        (tidy_dir / "compile_commands.json").write_text(
            database.read_text(encoding="utf-8"), encoding="utf-8")
        run_step("Clang-Tidy advisory", [tidy, "-p", str(tidy_dir), *files],
                 mandatory=False, evidence="SOURCE_CONFIRMED", tool="clang_tidy",
                 warning_markers=(" warning:", "warning:"))


def coverage_report(gcovr: str | None, cc: str | None, host_output: Path) -> None:
    if not gcovr:
        unavailable("Gcovr advisory", "gcovr not installed; set --gcovr or QA_GCOVR",
                    mandatory=False, evidence="HOST_VERIFIED", tool="gcovr")
        return
    if not supports_gcov_coverage(cc):
        skipped("Gcovr advisory", "selected host compiler does not provide GCC gcov coverage",
                mandatory=False, evidence="HOST_VERIFIED", tool="gcovr")
        return
    run_step("Gcovr advisory", [gcovr, "--config", str(ROOT / "gcovr.cfg"),
             "--xml", str(ARTIFACTS / "coverage.xml"), "--html-details",
             str(ARTIFACTS / "coverage.html"), "--txt",
             str(ARTIFACTS / "coverage.txt"), str(host_output)], mandatory=False,
             evidence="HOST_VERIFIED", tool="gcovr", env=host_environment(cc))


def git_metadata() -> dict[str, str | bool]:
    def git(*args: str) -> str:
        try:
            result = subprocess.run(["git", *args], cwd=ROOT, text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                    check=False)
            return result.stdout.strip()
        except OSError:
            return ""
    return {
        "path": str(ROOT),
        "branch": git("branch", "--show-current") or "unknown",
        "revision": git("rev-parse", "HEAD") or "unknown",
        "dirty": bool(git("status", "--porcelain")),
    }


def markdown_cell(value: str) -> str:
    return value.replace("|", "\\|").replace("\r", " ").replace("\n", " ")


def write_reports(mode: str) -> None:
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    repository = git_metadata()
    payload = {
        "schema_version": 2,
        "generated_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "mode": mode,
        "repository": repository,
        "tools": TOOL_VERSIONS,
        "hardware": {
            "motor_pwm_off_20ms": "HARDWARE_UNVERIFIED",
            "note": "Host and software checks cannot close the physical timing requirement.",
        },
        "results": [result.as_dict() for result in RESULTS],
    }
    (ARTIFACTS / "report.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")

    lines = [
        "# Firmware QA report", "", f"Mode: `{mode}`", "",
        f"Repository: `{repository['branch']}` at `{repository['revision']}` "
        f"({'dirty' if repository['dirty'] else 'clean'} worktree)", "",
        "Physical 20 ms PWM-off requirement: **HARDWARE_UNVERIFIED**", "",
        "| Check | Class | Evidence | Status | Tool | Duration |",
        "|---|---|---|---:|---|---:|",
    ]
    for result in RESULTS:
        lines.append(
            f"| {markdown_cell(result.name)} | {'mandatory' if result.mandatory else 'advisory'} | "
            f"{result.evidence} | {result.status} | {markdown_cell(result.tool_version)} | "
            f"{result.duration:.3f}s |"
        )
    lines += ["", "## Tool versions", ""]
    lines.extend(f"- `{key}`: {value}" for key, value in TOOL_VERSIONS.items())
    notable = [item for item in RESULTS if item.status != "PASS"]
    if notable:
        lines += ["", "## Non-passing checks", ""]
        for item in notable:
            lines += [f"### {item.name} ({item.status})", "",
                      f"Command: `{item.command}`" if item.command else "Command: not run", "",
                      "```text", item.output[-6000:], "```", ""]
    (ARTIFACTS / "report.md").write_text("\n".join(lines), encoding="utf-8")

    cases: list[str] = []
    for result in RESULTS:
        detail = ""
        if result.status == "FAIL" and result.mandatory:
            detail = f'<failure message="mandatory check failed">{html.escape(result.output)}</failure>'
        elif result.status in {"SKIP", "NOT_INSTALLED"}:
            detail = f'<skipped message="{html.escape(result.output)}" />'
        elif result.status in {"WARN", "FAIL"}:
            detail = f"<system-out>{html.escape(result.output)}</system-out>"
        cases.append(f'<testcase name="{html.escape(result.name)}" time="{result.duration}">{detail}</testcase>')
    failed = sum(item.status == "FAIL" and item.mandatory for item in RESULTS)
    skipped_count = sum(item.status in {"SKIP", "NOT_INSTALLED"} for item in RESULTS)
    junit = (f'<?xml version="1.0" encoding="UTF-8"?>\n<testsuite name="firmware-qa" '
             f'tests="{len(RESULTS)}" failures="{failed}" skipped="{skipped_count}">\n' +
             "\n".join(cases) + "\n</testsuite>\n")
    (ARTIFACTS / "junit.xml").write_text(junit, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("quick", "full", "hardware-report"), nargs="?",
                        default="quick")
    parser.add_argument("--cc", help="native C compiler for host tests (env: QA_HOST_CC)")
    parser.add_argument("--sdk", help="Pico SDK path (env: PICO_SDK_PATH)")
    parser.add_argument("--toolchain", help="ARM GNU toolchain root (env: PICO_TOOLCHAIN_PATH)")
    parser.add_argument("--cmake", help="CMake executable (env: QA_CMAKE)")
    parser.add_argument("--ninja", help="Ninja executable (env: QA_NINJA or CMAKE_MAKE_PROGRAM)")
    parser.add_argument("--picotool-dir", help="installed picotool CMake package (env: QA_PICOTOOL_DIR)")
    parser.add_argument("--pioasm-dir", help="installed pioasm CMake package (env: QA_PIOASM_DIR)")
    parser.add_argument("--cppcheck", help="Cppcheck executable (env: QA_CPPCHECK)")
    parser.add_argument("--clang-tidy", dest="clang_tidy",
                        help="Clang-Tidy executable (env: QA_CLANG_TIDY)")
    parser.add_argument("--gcovr", help="Gcovr executable (env: QA_GCOVR)")
    parser.add_argument("--host-build-dir",
                        help="host executable/coverage directory (env: QA_HOST_BUILD_DIR)")
    args = parser.parse_args()
    ARTIFACTS.mkdir(parents=True, exist_ok=True)

    cc, cc_detail = find_host_compiler(args.cc)
    sdk = find_sdk(args.sdk)
    toolchain, arm_gcc = find_pico_toolchain(args.toolchain)
    picotool_dir, picotool = find_picotool_dir(args.picotool_dir)
    pioasm_dir, pioasm = find_pioasm_dir(args.pioasm_dir)
    cmake = discover_executable(
        args.cmake, ("QA_CMAKE",), ("cmake",),
        (str(Path.home() / ".pico-sdk" / "cmake" / "v4.3.4" / "bin" / "cmake.exe"),
         r"C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\cmake\bin\cmake.exe"),
    )
    ninja = discover_executable(
        args.ninja, ("QA_NINJA", "CMAKE_MAKE_PROGRAM"), ("ninja",),
        (str(Path.home() / ".pico-sdk" / "ninja" / "v1.13.2" / "ninja.exe"),
         r"C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\ninja\ninja.exe"),
    )
    cppcheck = discover_executable(args.cppcheck, ("QA_CPPCHECK",), ("cppcheck",))
    tidy = discover_executable(args.clang_tidy, ("QA_CLANG_TIDY",), ("clang-tidy",))
    gcovr = discover_executable(args.gcovr, ("QA_GCOVR",), ("gcovr",))
    actual_sdk_version = sdk_version(sdk)
    host_output = Path(args.host_build_dir or os.environ.get("QA_HOST_BUILD_DIR")
                       or default_host_build_dir()).resolve()
    host_output.mkdir(parents=True, exist_ok=True)

    global TOOL_VERSIONS
    TOOL_VERSIONS = {
        "python": f"{sys.version.splitlines()[0]} [{sys.executable}]",
        "platform": platform.platform(),
        "host_cc": compiler_version(cc),
        "host_target": compiler_target(cc) if cc else "not found",
        "cmake": compiler_version(cmake),
        "ninja": compiler_version(ninja),
        "arm_gcc": compiler_version(arm_gcc),
        "picotool": compiler_version(picotool, ("version",)),
        "pioasm": compiler_version(pioasm),
        "pico_sdk": f"{actual_sdk_version or 'unknown version'} [{sdk}]" if sdk else "not found",
        "cppcheck": compiler_version(cppcheck),
        "clang_tidy": compiler_version(tidy),
        "gcovr": compiler_version(gcovr),
        "host_build_dir": str(host_output),
    }

    if args.mode == "hardware-report":
        skipped("20 ms motor PWM-off measurement", "physical evidence not recorded",
                mandatory=False, evidence="HARDWARE_UNVERIFIED")
    else:
        tooling_regression_tests()
        expected_sdk = CONFIG["pico_sdk"]
        if not sdk:
            unavailable("Pico SDK baseline", "Pico SDK not found; set --sdk or PICO_SDK_PATH",
                        mandatory=True, evidence="SOURCE_CONFIRMED", tool="pico_sdk")
        else:
            status = "PASS" if actual_sdk_version == expected_sdk else "FAIL"
            record("Pico SDK baseline", status, mandatory=True, evidence="SOURCE_CONFIRMED",
                   tool="pico_sdk",
                   output=f"expected {expected_sdk}; discovered {actual_sdk_version or 'unknown'} at {sdk}")

        if not cc:
            unavailable("host tests", cc_detail, mandatory=True,
                        evidence="HOST_VERIFIED", tool="host_cc")
        else:
            coverage_enabled = args.mode == "full" and bool(gcovr) and supports_gcov_coverage(cc)
            compile_qa_tests(cc, coverage_enabled, host_output)
            if sdk:
                existing_host_tests(cc, sdk, coverage_enabled, host_output)
            else:
                skipped("existing navigation and BLE host tests", "Pico SDK not found",
                        mandatory=True, evidence="HOST_VERIFIED", tool="host_cc")

        if args.mode == "full":
            missing_build_tools: list[str] = []
            if not cmake:
                missing_build_tools.append("cmake")
            if not ninja:
                missing_build_tools.append("ninja")
            if not toolchain:
                missing_build_tools.append("ARM GNU toolchain")
            if not sdk:
                missing_build_tools.append("Pico SDK")
            if missing_build_tools:
                unavailable("Pico firmware builds",
                            "missing mandatory build dependencies: " + ", ".join(missing_build_tools),
                            mandatory=True, evidence="FIRMWARE_COMPILE_VERIFIED", tool="cmake")
                skipped("static analysis", "firmware compilation database unavailable",
                        mandatory=False, evidence="SOURCE_CONFIRMED")
            else:
                databases = cmake_builds(sdk, toolchain, cmake, ninja,
                                         picotool_dir, pioasm_dir)
                static_analysis(databases, cppcheck, tidy)
            coverage_report(gcovr, cc, host_output)

    write_reports(args.mode)
    mandatory_failures = [result for result in RESULTS
                          if result.mandatory and result.status == "FAIL"]
    print(f"Reports: {ARTIFACTS / 'report.md'}")
    return 1 if mandatory_failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
