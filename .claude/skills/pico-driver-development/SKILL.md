---
name: pico-driver-development
description: Use when implementing or debugging a Pico W peripheral driver (GPIO, I2C, ADC, PWM) for the rover, e.g. TCS34725 on I2C, TCRT5000 on ADC, IMU, motor PWM, buzzer.
---
# Pico driver development

Read `CLAUDE.md` and `docs/dev/build-and-debug.md` first. Reference drivers: `pico_rssi_tracker/imu_mpu9250.c` (I2C), `getting_there/motion.c` (PWM).

## Workflow
1. Confirm pins/bus in existing code and wiring docs (`MPU9250_CONNECTION.md`); if absent or conflicting, stop and ask. Check no other module owns the pin/PWM slice.
2. Define a small header: init, read/write, returns a status code; no blocking forever (timeouts on every I2C/ADC call).
3. Output readings as `{value, valid, timestamp_ms}`; never drive motors from a driver.
4. Host test with stubs in `getting_there/tests/stubs/` (add to the stub, do not touch `qa/stubs`): success, bus error/timeout, boundary values.
5. Build the target with `scripts/dev/build.ps1`, run host tests and `python scripts/qa/run.py quick`.

## Done when
Driver builds warning-free, host tests cover normal/boundary/failure, pin ownership is documented, hardware checks not yet done are listed in the handoff.
