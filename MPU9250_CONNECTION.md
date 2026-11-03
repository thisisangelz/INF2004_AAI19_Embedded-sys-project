# MPU-9250/MPU-6500 connection and checks

The current robot programme uses the gyro for turns and heading correction.
Its driver is in [`getting_there/imu_mpu9250.c`](getting_there/imu_mpu9250.c).
See the [robot instructions](getting_there/README.md) for the complete startup,
calibration and movement procedure. Earlier standalone diagnostics remain
available through [Git history](docs/history.md).

## Wiring

Use the I2C pins assigned in the robot firmware:

| IMU pin | Pico W |
|---|---|
| VCC/VIN | 3V3 (check the breakout-board rating first) |
| GND | GND |
| SDA | GP6 (I2C1 SDA) |
| SCL | GP7 (I2C1 SCL) |
| AD0/ADO | GND, giving address `0x68` |
| INT | Leave disconnected initially |

Use only 3.3 V logic. Do not connect a 5 V pull-up to SDA or SCL. If the board has no pull-up resistors, add 2.2 kΩ to 4.7 kΩ from SDA to 3V3 and from SCL to 3V3. Keep the wires short and connect the IMU ground to the Pico ground.

The name on the breakout board is not sufficient to identify the sensor. The driver accepts common WHO_AM_I values for MPU-6500 (`0x70`), MPU-9250 (`0x71`) and MPU-9255 (`0x73`).

## Firmware test

Build and flash `getting_there`, then open the USB serial output. The motion
code reports the detected `IMU WHO_AM_I` value and gyro range. If it reports
`FAILED: IMU initialisation or gyro range configuration`, check power, ground,
SDA/SCL order, address selection and pull-ups before changing software.

The driver initially selects +/-250 degrees per second; `motion.c` then selects
and verifies +/-1000 degrees per second for navigation. The robot calibrates
gyro bias for two seconds after GP20. Keep it still during calibration. It
integrates gyro readings during turns and forward or reverse movement.

## Bench test before mounting

1. Keep the robot still and confirm the sensor is detected after every reset.
2. Follow the startup procedure in the robot instructions and keep the robot
   still while it reports `Keep robot still: calibrating gyro for 2 seconds`.
3. Check the reported learnt gyro direction and measured turn angle during
   the controlled movement test described in those instructions.
4. Repeat with the motors powered but not running, then during the controlled
   movement test. Investigate resets or large noise before proceeding.

Motor power must come from the motor supply through a driver such as TB6612FNG or DRV8833. Never drive motors from Pico GPIO pins. Keep motor wiring away from the IMU and add supply decoupling near the motor driver.
