# Handoff: TCS34725 hardware diagnostic target

- **Base commit:** `b006447`   **Head commit:** the commit that adds this file, on `dev/tooling-setup`
- **Author:** Claude (developer)   **Reviewer:** Codex

## Purpose and expected behavior
Standalone bring-up target for the Adafruit TCS34725 on I2C0 (SDA GP0, SCL GP1, addr 0x29, 100 kHz). Stage 1 probes 0x29 and reads ID (0x12, accepts 0x44/0x4D). Stage 2 runs only after stage 1 passes: ATIME 0xD5, gain 4x, PON then AEN, then reads clear/R/G/B every 500 ms. All I2C transfers use `*_blocking_until` with a 5 ms deadline. Results are in the volatile global `tcs_diag` for GDB/OpenOCD. USB serial output is best effort (3 s wait, then runs regardless). Not used by navigation firmware; no motors, no radio.

## Files changed
- New `tcs34725_diag/` (`tcs34725_diag.c`, `CMakeLists.txt`, `README.md`).
- `CMakeLists.txt` (root): one line, `add_subdirectory(tcs34725_diag)`.
- New `.vscode/launch.json`: two Cortex-Debug configs (root build ELF, scripts-build ELF).
- Not committed: Pico VS Code extension edits to `.vscode/*`, `pico_sdk_import.cmake`, and the extension block at the top of the root `CMakeLists.txt`.

## Interface changes (headers, pins, constants, ownership)
Claims GP0/GP1 as I2C0 in this target only. No existing pin assignment changed. No headers. Global `tcs_diag` (not `static`, by design, for the debugger).

## Tests actually run (command -> result, verbatim counts)
| Command | Result |
|---|---|
| `.\scripts\dev\build.ps1 tcs34725_diag` (SDK 1.5.1) | Built, no warnings with -Wall -Wextra |
| Root configure + `ninja tcs34725_diag` (SDK 1.5.1, Debug) | Built `tcs34725_diag.elf` |
| Pico VS Code extension compile (SDK 2.3.1) | `build/tcs34725_diag/tcs34725_diag.elf` produced |
| `scripts/dev/host_tests.ps1` | Navigation and BLE host tests: passed (2 lines printed) |
| `python scripts/qa/run.py quick` | `[FAIL] host tests: native gcc/clang/cc not found` (environment, not caused by this change; not investigated) |

## Evidence class
Build, plus **hardware (user-run, Pico W on ROBO-PICO, Debug Probe, OpenOCD via Cortex-Debug)**:
- Probe/SWD connection to both RP2040 cores: confirmed from Debug Console log.
- Reported by the user from the debugger: `stage`=4, `probe_ack`=1, `id_value`=68 (0x44), no I2C errors.
- Reported by the user: R, G, B each highest for a matching coloured object. No numbers recorded.
- Covered `clear` was non-zero. Not resolved; likely light leak or the board LED, unconfirmed.

## Untested hardware assumptions
- Nothing else on the ROBO-PICO uses GP0/GP1 (worked in the test; board not inspected).
- Gain 4x / ~103 ms integration are untuned; saturation and dark offset not measured.
- Behavior while the robot moves, and with other firmware on the same bus.
- Dark `clear` baseline with a truly opaque cover.

## Risks
- Two SDK versions in use (scripts 1.5.1, extension 2.3.1); only the extension build was debugged on hardware.
- Pull-ups enabled on GP0/GP1 in addition to the breakout's.
- `tcs_diag` writes are unsynchronized, single-core, read only by the debugger.

## Known limitations
No calibration or thresholds, no faults.c integration, no freshness struct (diagnostic only). Not for use as the driver for navigation.

## QA findings response (fill after review)
| Finding | Action: fixed (commit/test) / rejected (reason) |
|---|---|
