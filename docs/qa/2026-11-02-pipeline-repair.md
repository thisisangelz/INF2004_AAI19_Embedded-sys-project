# Firmware QA pipeline repair report — 2026-11-02

Branch: `dev/tooling-setup`

Revision tested: `f09e31f7560010c4002a6fd17b64c0552480bc46` plus the
uncommitted QA changes listed below. No production firmware was changed. No
hardware was flashed or operated.

## Readiness result

- Quick QA: **PASS** — all mandatory host tests compile and execute.
- Full QA: **PASS** — 37 checks passed, zero mandatory failures, and three
  advisory tools were explicitly reported `NOT_INSTALLED`.
- Firmware: **FIRMWARE_COMPILE_VERIFIED** for all nine supported targets, with
  ELF and UF2 artifacts checked for each target.
- Hardware: **HARDWARE_UNVERIFIED**. In particular, no measurement closes the
  GP21 20 ms PWM-off requirement.
- GitHub Actions: workflow repaired but **CI_UNVERIFIED** because repository
  policy forbids committing or pushing these changes without separate approval.

Evidence: `artifacts/qa/report.md`, `report.json`, and `junit.xml`. The final
JUnit suite contains 40 cases, zero failures, and three advisory skips.

## Root causes and closure

### High — mandatory native host compiler unavailable

- Location: `scripts/qa/run.py:170`, `scripts/dev/env.ps1:37` after repair.
- Reproduction: `powershell -NoProfile -ExecutionPolicy Bypass -File
  scripts/qa/run.ps1 quick` originally returned `native gcc/clang/cc not found`.
- Observed evidence: no native GCC, Clang, or MSVC executable existed; only
  `arm-none-eabi-gcc` was installed.
- Consequence: no host test could compile or execute, so quick and full QA both
  had a mandatory failure.
- Correction: installed MSYS2 plus UCRT64 GCC 16.2.0; added deterministic
  `--cc`/`QA_HOST_CC` discovery and rejection of bare-metal cross-compilers.
- Closure test: quick and full QA compile and run packed-bit, transfer-protocol,
  shared-state, IMU, motor-safety, navigation, and BLE payload tests.

### High — SDK migration reused incompatible CMake caches

- Location: `scripts/qa/run.py:392` after repair.
- Reproduction: configure SDK 1.5.1 build directories with `-DPICO_SDK_PATH`
  pointing to SDK 2.3.1.
- Observed evidence: CMake mixed `C:/Program Files/...SDK v1.5.1/...` modules
  with `C:/Users/alvin/.pico-sdk/sdk/2.3.1`, then failed to find
  `rp2_common.cmake` and Pico commands.
- Consequence: four standalone firmware groups failed configuration.
- Correction: every QA configure uses CMake `--fresh`; compilation databases
  from failed configurations are excluded.
- Closure test: all five build groups configure from fresh state under SDK 2.3.1.

### High — BLE host test encoded the SDK 1.5.1 BTstack event layout

- Location: `getting_there/tests/test_payload_client.c:106` and `:116`.
- Reproduction: run the BLE payload host test using the BTstack headers shipped
  with Pico SDK 2.3.1.
- Observed evidence: the mock placed GATT ATT status at byte 4 and notification
  payload at byte 8; BTstack 1.8.2 reads them at byte 8 and byte 12 after adding
  service and connection IDs. The test reported `GATT operation rejected`.
- Consequence: a valid production state machine was reported as a mandatory
  protocol failure.
- Correction: updated only the mocked event builders to the SDK 2.3.1 layouts;
  split the final compound assertion for more precise diagnostics.
- Closure test: both normal and early-notification exchanges pass, and the full
  BLE payload test completes successfully.

### Medium — standalone projects did not discover installed Pico host tools

- Location: `scripts/qa/run.py:247`, `:266`, and `:398` after repair.
- Reproduction: freshly configure a standalone target under SDK 2.3.1 without
  the Pico VS Code extension's CMake prelude.
- Observed evidence: CMake attempted a network clone of `picotool`; after that
  was fixed, Wi-Fi targets attempted to build `pioasm` and failed without a
  native C++ compiler.
- Consequence: offline local builds were non-deterministic and four supported
  firmware targets failed.
- Correction: discover and pass installed `picotool_DIR` and `pioasm_DIR`, with
  CLI/environment overrides. CI uses a shared pinned picotool fetch directory.
- Closure test: all standalone targets build offline and emit ELF/UF2 files.

### Medium — Windows Application Control inspection race

- Location: `scripts/qa/run.py:85`, `:311`, and
  `getting_there/tests/run_host_checks.py:37` after repair.
- Reproduction: compile and immediately execute unsigned MinGW host binaries in
  the OneDrive workspace.
- Observed evidence: process creation intermittently failed with WinError 4551;
  the identical binary passed after a short delay and from `%TEMP%`.
- Consequence: false mandatory host-test failures despite successful compilation.
- Correction: Windows host binaries default to a temporary build directory and
  process creation retries only WinError 4551 with bounded backoff. Nonzero test
  exits and all other start failures still fail immediately.
- Closure test: the shared quick gate passed; one retry was observed and the
  executable then completed its assertions successfully.

### Low — advisory analyzers are absent locally

- Location: `scripts/qa/run.py:449` and `:486` after repair.
- Reproduction: run full QA without Cppcheck, Clang-Tidy, or Gcovr installed.
- Observed evidence: all three tools are recorded as `NOT_INSTALLED` with their
  resolved version `not found`.
- Consequence: no local static-analysis or coverage baseline was produced.
- Correction: discovery, explicit overrides, compilation-database filtering,
  warning classification, and coverage reporting are implemented. CI installs
  Cppcheck, Clang-Tidy, and pinned Gcovr 8.3.
- Closure test: local report/JUnit accurately show three advisory skips while
  returning success because all mandatory checks pass. CI execution remains to
  be verified after approved commit/push.

## Selected toolchain baseline

- Pico SDK 2.3.1.
- Arm GNU Toolchain 15.2.Rel1 for RP2040 firmware.
- MSYS2 UCRT64 GCC 16.2.0 for Windows host tests.
- CMake 4.3.4, Ninja 1.13.2, picotool 2.3.1, and pioasm 2.3.1 were used locally.

SDK 2.3.1 is an official Pico SDK release and matches the root Pico VS Code
configuration. Raspberry Pi's Pico VS Code toolchain catalog provides Arm GNU
15.2.Rel1 packages for Windows and Linux. CI downloads the pinned Linux archive
and checks the vendor-published SHA-256 value before extraction.

## Dependencies installed or remaining

Installed on Windows during this repair:

- MSYS2 installer 20260927 via `winget`.
- `mingw-w64-ucrt-x86_64-gcc` 16.2.0-4 and its required runtime/binutils
  packages via Pacman.

Not installed locally; advisory and accurately reported:

- Cppcheck
- Clang-Tidy
- Gcovr

## Files changed

- QA runner and inventory: `scripts/qa/run.py`, `run.ps1`, `targets.json`, and
  `scripts/qa/tests/test_run.py`.
- Host test harness: `getting_there/tests/run_host_checks.py` and
  `test_payload_client.c`.
- Windows development discovery: `scripts/dev/env.ps1` and `host_tests.ps1`.
- CI and presets: `.github/workflows/firmware-qa.yml` and `CMakePresets.json`.
- Documentation/ignore rules: `.gitignore`, `README.md`,
  `docs/dev/build-and-debug.md`, and `docs/qa/baseline.md`.

## Significant commands executed

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/qa/run.ps1 quick
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/qa/run.ps1 full
python -m unittest discover -s scripts/qa/tests -p 'test_*.py'
winget install --id MSYS2.MSYS2 --exact --accept-package-agreements --accept-source-agreements --silent
C:\msys64\usr\bin\bash.exe -lc "pacman -S --needed --noconfirm mingw-w64-ucrt-x86_64-gcc"
```

## Before/after summary

| Gate | Before | After |
|---|---|---|
| Quick | Mandatory failure; no native compiler | PASS; all host-test areas execute |
| SDK baseline | Mixed 1.5.1, 2.3.0, and 2.3.1 references | SDK 2.3.1 / Arm GNU 15.2.Rel1 |
| Firmware inventory | Five build groups, TCS artifact not reported by target | Nine targets, target-specific ELF/UF2 evidence |
| Standalone fresh build | Cache mixing, picotool fetch, pioasm build failures | All groups configure and build offline locally |
| Static/coverage | Generic skips | Explicit `NOT_INSTALLED` with tool versions and overrides |
| Reports | Schema 1, aggregated build evidence | Schema 2, repository state, tool per result, accurate JUnit |
| CI | SDK 2.3.0 and distribution ARM package | SDK 2.3.1, pinned ARM 15.2.Rel1, shared full entry point |

## Final assessment

The local Windows software baseline is **QA_READY_WITH_ADVISORIES**. Mandatory
host and cross-compile checks pass. Static-analysis and coverage execution remain
open locally, and GitHub Actions remains unverified until an approved commit and
push triggers the repaired workflow. No software result provides physical
hardware evidence.
