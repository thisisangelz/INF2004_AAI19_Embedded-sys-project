# MPU-9250/MPU-6500 connection and first test

The first navigation milestone is to read the gyro and use it to measure controlled turns. RSSI navigation should be added only after this wiring test is repeatable.

## Wiring

Use the I2C pins assigned in the tracker firmware:

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

The tracker now includes `imu_mpu9250.c` and probes the sensor during startup. Build and flash the tracker, then open the USB serial output. A successful probe reports the I2C pins and a detected WHO_AM_I value. `NOT FOUND` means to check power, ground, SDA/SCL order, address selection and pull-ups before changing software.

The current driver configures the gyro to ±250 degrees/second and exposes `mpu9250_read_gyro_dps()`. It deliberately does not yet estimate heading or control motors.

## Bench test before mounting

1. Keep the robot still and confirm the sensor is detected after every reset.
2. Read gyro Z while still. Record the bias; it should remain near zero and stable.
3. Rotate the robot by hand through 90 degrees and confirm the sign and magnitude change.
4. Repeat with the motors powered but not running, then with the motors running. Investigate resets or large noise before proceeding.
5. Add bias calibration and a timed turn controller only after these checks pass.

Motor power must come from the motor supply through a driver such as TB6612FNG or DRV8833. Never drive motors from Pico GPIO pins. Keep motor wiring away from the IMU and add supply decoupling near the motor driver.

## Next implementation steps

1. Add a startup gyro-bias routine while the robot is stationary.
2. Add motor driver code with acceleration limiting, stop timeout and bumper input.
3. Turn 45 degrees using integrated gyro Z and measure the turn error.
4. Add stationary Wi-Fi RSSI samples at eight headings.
5. Move 20–30 cm towards the strongest filtered heading, then repeat the sweep.
6. Stop Wi-Fi scans, confirm the target with BLE RSSI and perform the existing transfer.
