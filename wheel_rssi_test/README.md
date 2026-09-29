# Wheel and Wi-Fi RSSI movement test

This is a separate Pico W programme. It does not modify or run the BLE transfer
in `pico_rssi_tracker`.

## Connections

| Component | Robo Pico connection |
|---|---|
| Left geared motor, two wires | M1 black screw terminal |
| Right geared motor, two wires | M2 black screw terminal |
| MPU-9250/6500 VCC, GND, SDA, SCL, AD0 | 3V3, GND, GP6, GP7, GND |
| Battery | VIN 3.6–6 V or the single-cell LiPo socket |

The Robo Pico motor driver already uses GP8/GP9 for M1 and GP10/GP11 for M2.
The servo is not used. GP20 starts the test; GP21 stops it and requires a reset
before another run. No connections to the proposed GP2/GP3 eCompass are needed.

## Untethered floor test and stored report

The programme keeps up to 8191 bytes of serial messages in RAM. No Debug
Probe or computer cable is needed during movement. UART output is disabled.

1. Flash the new `build/wheel_rssi_test.uf2` with the Pico W USB cable.
2. Disconnect that cable and power the Robo Pico from its battery. Place the
   robot on a clear floor, with AP1 switched on.
3. Press GP20 once. The robot performs its test and stops. GP21 stops it at
   any point. Do not turn off the battery or press RESET after the test.
4. Once the robot has stopped, plug the Pico W USB cable into the computer
   while the battery remains on. Open the Pico's USB COM port at 115200 baud.
5. The stored report prints when the serial monitor connects. If the monitor
   opened too late, press GP20 to print the same report again. Pressing GP20
   at this stage does not repeat the movement.

The report includes AP1 scan values and BSSID, the filtered RSSI at each
position, every probe heading, gyro turn results, the selected direction or
the reason the test stopped. The report
exists only in RAM. A reset or loss of robot power erases it. Check that
reconnecting USB does not reset your particular battery and carrier-board
setup before relying on this during a floor test.

Before the floor test, put the chassis on a stand with both wheels in the air.
Use the Robo Pico's M1A/M1B/M2A/M2B buttons to check each motor. Both wheels
must propel the chassis forwards when the firmware commands positive speed.
If a wheel runs in the opposite direction, turn off power and swap its two
wires, or change the matching `M1_FORWARD_USES_A` or `M2_FORWARD_USES_A`
constant in `wheel_rssi_test.c` and rebuild. Do not change both without testing.
The initial firmware drove both A outputs for forwards. The reported right
spin followed by a left spin indicates opposite physical wheel directions if
M1 is the left wheel and M2 is the right wheel. This build reverses M2's
software polarity. Check that assumption with the chassis lifted. A straight
pulse now stops if the gyro measures more than 25 degrees of turning.

## What happens after GP20

1. Keep the robot still for two seconds while the gyro Z bias is measured.
2. Scan for AP1 (`I AM PICO W`) five times. AP1 must be seen in at least
   three scans before the motors can move. AP2 and AP3 are ignored.
3. At headings 0, 90, 180 and 270 degrees, drive forwards for 1200 ms, stop,
   scan five times, then drive backwards for the same duration. Turn 90
   degrees using the gyro before the next heading.
4. Compare the four RSSI values. A direction must beat the starting value by
   at least 4 dB and the runner-up by at least 3 dB. Otherwise the robot stops without the final
   movement.
5. Turn towards the chosen heading and drive forwards for 250 ms. Stop and
   print `TEST COMPLETE`.

Each Wi-Fi scan keeps the strongest valid AP1 report, as in the main tracker.
At the starting position and each probe position, the test takes five fresh
scans. It sorts the valid readings, drops the lowest and highest quarter, then
averages the rest. At least three of the five scans must detect AP1. Readings
from one position are never reused for another position. The report prints
every raw scan and the resulting filtered value. This programme does not scan
BLE advertisements, so the tracker's BLE filter does not apply here.

The 1200 ms probe is twice the previous 600 ms probe. Test it in a clear area
and stop with GP21 if the travel is too long. The timed reverse movement only approximates the starting position. Motor
speed, wheel slip and battery voltage change the actual distance. The gyro
measures turning, not travel distance. The motor pulses are intentionally short,
but their distance in centimetres is unknown until measured on the assembled
robot. Wi-Fi RSSI can also favour a reflection rather than the direct route.
The gyro turn calculation uses net signed rotation, so a wobble or brief turn
back does not count as progress towards 90 degrees. The gyro still depends on
its Z axis being aligned with the robot's vertical turning axis.
If the printed gyro angle disagrees with the physical turn, run the separate
`wheel_turn_check` firmware before trusting a heading choice. Do not increase
the turn target to compensate until the cause of that disagreement is known.
The reported `-127` value in the earlier three-beacon build meant "not seen";
it was not a signal measurement. This AP1-only build prints `not seen` instead.
An SSID match alone does not prove beacon identity if another access point
uses the same name. Each scan prints the observed BSSID so you can compare it
with AP1's Wi-Fi hardware address.

Run the first floor test in a clear area, away from table edges. Keep a hand
near the Robo Pico power switch. Press GP21 to stop; power off if necessary.
The firmware stops on an IMU read error, scan timeout, turn timeout or missing
beacon. The motors do not start automatically at boot.

## Build with Ninja on Windows

Run these commands in PowerShell from the repository root. The SDK and tools
paths match the Pico SDK v1.5.1 installation used for this build.

```powershell
$env:PICO_SDK_PATH = 'C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\pico-sdk'
$env:PICO_TOOLCHAIN_PATH = 'C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\gcc-arm-none-eabi'
cmake -S wheel_rssi_test -B wheel_rssi_test\build -G Ninja -DCMAKE_MAKE_PROGRAM='C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\ninja\ninja.exe' -DPICO_BOARD=pico_w
cmake --build wheel_rssi_test\build --parallel 4
```

Flash `build/wheel_rssi_test.uf2` from this folder. The `.uf2` was built with
Ninja for Pico W using the same SDK installation, but motor direction and
travel still need a test on the actual robot.
