# Single-turn check

This separate Pico W programme checks one physical turn without Wi-Fi scans or
forward travel. It uses the same M1/M2 polarity, 35% turn duty and MPU Z-axis
integration as `wheel_rssi_test`. It does not change the main tracker.

1. Put tape on the floor to mark the robot's starting direction and a second
   line at a measured 90-degree angle. Allow clear space around the robot.
2. Flash `build/wheel_turn_check.uf2`. Disconnect USB and power from battery.
3. Place the robot on the start mark, with its IMU firmly fixed to the chassis.
   Press GP20. Keep it still for the two-second gyro calibration. It then turns
   once and stops. GP21 stops the motor at any point.
4. Note the robot's *physical* final angle against the floor marks. Do not
   adjust the turn target based only on the gyro's printed value.
5. Keep battery power on, connect Pico USB, open its COM port and read the
   stored report. If necessary, press GP20 again to print it. A reset erases it.

Tell us both the physical angle and the reported integrated gyro angle. If
they disagree, check that the IMU is fixed to the chassis and inspect the
reported X/Y/Z rates. An IMU Z axis tilted away from vertical normally makes
the physical turn *larger* than the reported Z angle. A physically smaller turn
with a reported 90 degrees needs a different explanation, such as IMU motion
relative to the chassis, a mistaken floor reference or sensor scale error.

Build from the repository root in PowerShell:

```powershell
$env:PICO_SDK_PATH = 'C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\pico-sdk'
$env:PICO_TOOLCHAIN_PATH = 'C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\gcc-arm-none-eabi'
cmake -S wheel_turn_check -B wheel_turn_check\build -G Ninja -DCMAKE_MAKE_PROGRAM='C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\ninja\ninja.exe' -DPICO_BOARD=pico_w
cmake --build wheel_turn_check\build --parallel 4
```
