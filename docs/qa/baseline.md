# Independent firmware QA baseline

Baseline branch: `dev/tooling-setup`. Reports record the exact Git revision and
whether the worktree was dirty when each gate ran.

## Supported build inventory

| Build group | Firmware | Role |
|---|---|---|
| Root | `pico_rssi_tracker`, `picow_access_point`, `picow_access_point2`, `picow_access_point3`, `tcs34725_diag` | Tracker, three beacon prototypes, and standalone colour-sensor diagnostic |
| Standalone | `getting_there` | AP1 navigation and payload firmware |
| Standalone | `rssi_env_test` | RSSI environment diagnostic |
| Standalone | `wheel_rssi_test` | Motor, gyro, and RSSI diagnostic |
| Standalone | `wheel_turn_check` | Physical turn diagnostic |

Full QA configures the root project once, builds each root target separately,
then configures each standalone project once. This prevents duplicate builds
while recording target-specific ELF and UF2 locations. The inventory does not
declare that every diagnostic target is production firmware.

## Reproducible toolchain baseline

- Pico SDK `2.3.1` is the selected SDK for Windows and CI. It matches the root
  Pico VS Code project configuration and supersedes the former `1.5.1`/`2.3.0`
  split. Delete or use a new CMake build directory when migrating.
- Arm GNU Toolchain `15.2.Rel1` is the selected RP2040 cross-compiler. The
  Windows Pico extension installation already provides it; CI downloads the
  same vendor archive and verifies the vendor-published SHA-256 value.
- Native host tests require native GCC or Clang. `arm-none-eabi-gcc` is rejected
  because host test executables must run on the build machine.
- CMake, Ninja, Python, Cppcheck, Clang-Tidy and Gcovr versions and resolved
  paths are retained in every report. CLI arguments and documented environment
  variables can override discovery without adding machine-specific paths.

## Confirmed baseline

- Pico SDK CMake is the target build system. Historical documentation used SDK
  1.5.1 and 2.3.0; the QA baseline is now SDK 2.3.1.
- Two existing host executables exercise navigation and the BLE payload client using source inclusion and manual hardware/BTstack stubs.
- `getting_there` runs a foreground navigation loop while CYW43 and BTstack deliver asynchronous callbacks.
- The packed BLE status word and fault mask use interrupt-masked read-modify-write operations. The stored report uses a Pico critical section.
- GP21 STOP is polled. Motion loops normally poll every 5 ms, while several foreground waits poll every 10 ms.
- MPU9250 access uses blocking I2C calls without explicit timeouts. Therefore the source cannot guarantee a 20 ms STOP response during a stalled transaction.
- A standalone TCS34725 diagnostic is present and compile-verified only. No QA
  result may imply physical sensor behavior was validated. No TCRT5000
  implementation is present.

## Evidence boundaries

Software QA uses these evidence classes:

- `SOURCE_CONFIRMED`: established by direct inspection or static analysis.
- `HOST_VERIFIED`: executed natively, possibly with mocks.
- `FIRMWARE_COMPILE_VERIFIED`: cross-compiled for Pico W, not run on target.
- `SIMULATOR_VERIFIED`: executed in an identified simulator.
- `HARDWARE_UNVERIFIED`: requires physical evidence that has not been recorded.
- `HARDWARE_MEASURED`: supported by a reproducible procedure and retained measurements.

Mocked sensor values, a successful cross-build, or host timing cannot be promoted to hardware evidence.

## Tooling policy

Compiler/build/test failures are mandatory. Clang-Tidy, Cppcheck, and coverage begin as advisory until the team accepts a warning and coverage baseline. Third-party Pico SDK and BTstack findings are excluded from project-owned reports unless they expose an integration problem.

