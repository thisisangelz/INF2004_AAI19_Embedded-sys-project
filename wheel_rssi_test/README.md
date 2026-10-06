# Wheel and Wi-Fi RSSI movement test

On `first-beacon`, this is a one-beacon movement proof of concept. It compares
AP1 RSSI after longer probes in four directions, then drives towards the
strongest valid result without requiring an RSSI gain or a difference between
directions.

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
the reason the test stopped. Each straight movement reports the commanded
forwards or backwards direction, duration, base motor duty and target heading.
Its completion line reports net gyro rotation including the 300 ms after the
motors stop, final heading error and the largest duty correction. Each turn
reports its final measured error, elapsed time, peak gyro rate and correction
count. These readings help compare drift during the forwards probes and
backwards returns. The raw yaw sign follows the IMU's Z axis; it does not
establish which physical direction is left or right. The report
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
pulse stops if its measured heading differs from the target by more than
25 degrees.

## What happens after GP20

1. Keep the robot still for two seconds while the gyro Z bias is measured.
2. Scan for AP1 (`I AM PICO W`) five times. AP1 must be seen in at least
   three scans before the motors can move. AP2 and AP3 are ignored.
3. Command a 15-degree turn to learn the gyro sign for the motor turn
   direction, then turn back to the original 0-degree heading. This makes the
   heading correction account for the IMU's mounting direction.
4. At headings 0, 90, 180 and 270 degrees, drive forwards for 2400 ms, stop,
   scan five times, then drive backwards for the same duration. Correct the
   heading during both movements. Turn to the next target heading, relative
   to the original direction, after each return.
5. Choose the strongest valid RSSI among the four headings. It can be weaker
   than the starting value, and one valid heading is enough. If headings tie,
   choose the first one in the order 0, 90, 180, 270 degrees. If every heading
   lacks a valid score, stop without selecting a direction or moving again.
6. Turn towards the chosen heading and drive forwards for 2400 ms. Stop and
   print `TEST COMPLETE`.

For the supplied test scores of -70, -70, missing and -69 dBm, this rule chooses
270 degrees at -69 dBm, even though the starting value was -68 dBm.

Each Wi-Fi scan keeps the strongest valid AP1 report, as in the main tracker.
At the starting position and each probe position, the test takes five fresh
scans. It sorts the valid readings, drops the lowest and highest quarter, then
averages the rest. At least three of the five scans must detect AP1. Readings
from one position are never reused for another position. The report prints
every raw scan and the resulting filtered value. This programme does not scan
BLE advertisements, so the tracker's BLE filter does not apply here.

The 2400 ms probe is twice the previous 1200 ms probe. The final movement also
lasts 2400 ms, compared with the previous 250 ms. Test it in a clear area and
stop with GP21 if the travel is too long. The backwards movement after each
probe is intentional: it tries to return to the starting position. Equal
forwards and backwards durations only approximate that return. Motor speed,
wheel slip and battery voltage change the actual distance. The gyro measures
turning, not travel distance. Distance in centimetres is unknown until measured
on the assembled robot. Longer movements can make wheel-speed differences and
return-position errors more apparent. Both wheels still receive the same 35%
base duty. The firmware uses gyro heading error and turn rate to adjust the
signed wheel commands by up to eight duty points in opposite directions. This
correction operates during both forwards probes and backwards returns. A
heading error greater than 25 degrees stops the movement. The final heading
is also checked after the 300 ms settling period.
Wi-Fi RSSI can also favour a reflection rather than the direct route.

The previous turn report repeatedly reached a peak of 250 degrees per second.
That suggests the previous gyro range of +/-250 degrees per second could have
clipped faster rotation. It does not prove clipping or establish the robot's
physical angle. This movement firmware now selects +/-1000 degrees per second
and checks the range bits by reading the configuration register back. The
shared driver uses the matching conversion factor of 32.8 raw counts per
degree per second, rather than the previous fixed 131 factor. These settings
follow the [TDK MPU-6500 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf)
and [gyro specifications](https://invensense.tdk.com/wp-content/uploads/2020/06/PS-MPU-6500A-01-v1.3.pdf).
Other programmes retain their default +/-250 range unless they request a
different range. A motion reading at or beyond +/-980 degrees per second stops
this test because the integrated angle may no longer be reliable.

Turns use 25% duty, reduced to 20% within 25 degrees of the target. The motion
loop waits 5 ms between readings. It cuts motor power early using the current
turn rate and a 30 ms estimate of continued rotation, then measures another
300 ms with the motors off. If the measured error is still greater than two
degrees, it permits at most two corrective turns. Those corrective turns use
the measured tolerance rather than the predicted coast angle, so they can make
progress even when the robot barely coasts. Each corrective turn has at most
200 ms of powered movement followed by measured settling. A final error
outside that tolerance stops the test. The integrated heading is retained across straight
movements, turns and their settling periods. Each new target uses that heading
relative to the original direction, so the next turn accounts for measured
error from earlier movements.

These motion settings still need a test on the assembled robot. The 20% duty
may not start a turn on every floor; if the gyro detects no sustained turning
within one second, the test stops and reports the duty. The gyro depends on its
Z axis being aligned with the robot's vertical turning axis. Mark the initial
direction and the 90-degree directions on the floor before testing. If the
printed angle disagrees with the physical turn, check the IMU mounting and
motor directions before trusting the selected heading. The separate
`wheel_turn_check` programme still uses the earlier +/-250 gyro range and 35%
turn duty, so it does not exercise this motion controller.
The logged gyro angles and motor commands do not prove the robot's physical
direction. Check its movement against floor marks. The report alone cannot
identify the cause of a left bias or establish that a motor polarity change is
needed.
The `-127` value means that AP1 was missing or there were too few valid scans;
it is not a signal measurement and cannot win the heading selection. The
individual scans also print `not seen` when AP1 is absent.
An SSID match alone does not prove beacon identity if another access point
uses the same name. Each scan prints the observed BSSID so you can compare it
with AP1's Wi-Fi hardware address.

Run the first floor test in a clear area, away from table edges. Keep a hand
near the Robo Pico power switch. Press GP21 to stop; power off if necessary.
The firmware stops on an IMU read error, failed gyro range configuration,
gyro rate near the range limit, scan timeout, turn timeout, failure to start
turning, excessive straight-movement heading error, a turn outside its final
tolerance, missing starting AP1 or no valid probe scores. An invalid score at
one heading does not prevent testing the other headings. The motors do not
start automatically at boot.

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
