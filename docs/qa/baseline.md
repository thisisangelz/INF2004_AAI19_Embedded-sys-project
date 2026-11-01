# Independent firmware QA baseline

Baseline branch: `Alvin` at `e84d0f9` when this document was created.

## Supported build inventory

| Build group | Firmware | Role |
|---|---|---|
| Root | `pico_rssi_tracker`, `picow_access_point`, `picow_access_point2`, `picow_access_point3` | Four-board tracker and beacon prototype |
| Standalone | `getting_there` | AP1 navigation and payload firmware |
| Standalone | `rssi_env_test` | RSSI environment diagnostic |
| Standalone | `wheel_rssi_test` | Motor, gyro, and RSSI diagnostic |
| Standalone | `wheel_turn_check` | Physical turn diagnostic |

All checked-in build targets are compiled by full QA to prevent diagnostic firmware from silently rotting. This inventory does not declare that every diagnostic target is production firmware.

## Confirmed baseline

- Pico SDK CMake is the target build system. The repository documents SDK 1.5.1 and 2.3.0 use.
- Two existing host executables exercise navigation and the BLE payload client using source inclusion and manual hardware/BTstack stubs.
- `getting_there` runs a foreground navigation loop while CYW43 and BTstack deliver asynchronous callbacks.
- The packed BLE status word and fault mask use interrupt-masked read-modify-write operations. The stored report uses a Pico critical section.
- GP21 STOP is polled. Motion loops normally poll every 5 ms, while several foreground waits poll every 10 ms.
- MPU9250 access uses blocking I2C calls without explicit timeouts. Therefore the source cannot guarantee a 20 ms STOP response during a stalled transaction.
- No TCS34725 or TCRT5000 implementation is present. No QA result may imply those sensors are supported or validated.

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

