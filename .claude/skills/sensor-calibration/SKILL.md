---
name: sensor-calibration
description: Use when adding sensor sampling, calibration, thresholds or fault handling for the TCS34725 colour sensor, TCRT5000 IR sensors or the MPU IMU gyro.
---
# Sensor calibration

Reference: gyro calibration and turn checks in `getting_there/motion.c`; BLE RSSI filtering rules in `README.md` (trim lowest/highest quarter, average).

## Workflow
1. Separate acquisition (raw sample + timestamp) from classification (threshold/decision) so classifiers are pure functions testable on the host.
2. Sampling: fixed count, discard outliers, reject out-of-range (saturated ADC, TCS34725 clear=0 or saturated, gyro out of +/-1000 dps).
3. Thresholds: derive from recorded data (surface/line/colour samples, with hysteresis), store as named constants with a comment on how they were measured. Never invent thresholds; if no data exists, add a logging mode and list the calibration as a hardware task for the user.
4. Faults: stale or invalid reading -> `valid=false`, report through `faults.c`; never auto-clear a latched fault.
5. Host tests with synthetic data: nominal, at-threshold, noise, saturation, stuck sensor, stale timestamp.

## Done when
Classifier tests pass including boundaries and failures, constants are documented with their measurement source (or marked "uncalibrated"), and the handoff lists required physical calibration runs.
