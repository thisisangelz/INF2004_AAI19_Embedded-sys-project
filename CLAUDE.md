# Beacon Finder rover: developer (Claude) instructions

Claude writes functional firmware. Codex independently reviews (see `AGENTS.md`, `.agents/`, `scripts/qa/`, `qa/`, `docs/qa/`). Never edit those, and never overwrite Codex findings.

## Target
Pico W (RP2040), Cytron Robo Pico, MPU-9250/6500 IMU. Primary executable: `getting_there/` (standalone CMake project; bare-metal, blocking loops, no RTOS). Other targets: `pico_rssi_tracker`, `picow_access_point{,2,3}` (beacons), `rssi_env_test`, `wheel_rssi_test`, `wheel_turn_check`.
Pins in code are truth (IMU I2C GP6/GP7, motors GP8-GP11, start GP20, stop GP21, buzzer GP22). TCS34725/TCRT5000 pins from the brief are proposals only. **Never change pins without checking the board and getting approval.**

## Commands (details: `docs/dev/build-and-debug.md`)
```powershell
. .\scripts\dev\env.ps1
.\scripts\dev\build.ps1 getting_there
.\scripts\dev\host_tests.ps1
python scripts/qa/run.py quick
```

## Rules
- Only the motor module (`motion.c`) writes motor PWM. Sensor modules never drive motors.
- STOP/FAULT latches are never silently cleared (`faults.c`).
- Sensor readings carry validity + freshness (`{value, valid, timestamp_ms}`).
- The CYW43 radio is shared: Wi-Fi scan and BLE need explicit acquire/release.
- Keep the existing bit-packing style (`packed_bits.h`); follow surrounding C style. Minimal diffs, no repo-wide rewrites, no new test framework (Ceedling) pending Codex's assessment.
- Pass `-Wall -Wextra`; no new warnings in project code.

## Workflow per feature
Inspect code -> state interfaces/risks -> minimal change -> host test (mock hardware) -> build affected target -> run existing tests + QA quick -> handoff in `docs/handoffs/` from `TEMPLATE.md`.
Skills: `.claude/skills/` (pico-driver-development, sensor-calibration, firmware-feature-development).

## Hard limits
Work on a feature branch, never `main`. No commit, push, PR, or merge without user approval. **Never flash the Pico or run motors.** Compiling/host tests are not hardware evidence; label evidence class and list untested hardware assumptions.
