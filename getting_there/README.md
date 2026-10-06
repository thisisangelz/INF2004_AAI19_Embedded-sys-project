# AP1 approach and payload transfer

The `getting-there` branch runs this separate Pico W programme. Flash
`build/getting_there.uf2` onto the robot. It approaches AP1, receives and
verifies AP1's reply payload over BLE, then plays a one-second completion tone.

## Hardware and starting a run

| Function | Robo Pico connection |
|---|---|
| Left motor | M1, GP8/GP9 |
| Right motor | M2, GP10/GP11 |
| MPU-9250/6500 | 3V3, GND, SDA GP6, SCL GP7, AD0 GND |
| Start button | GP20 |
| Stop button | GP21 |
| Onboard piezo buzzer | GP22 |

GP22 is the Robo Pico's onboard buzzer connection, as shown in
[Cytron's robot example](https://my.cytron.io/tutorial/build-an-rc-robot-car-using-pi-pico-w-and-robo-pico).
Set its mute switch so the buzzer can sound. The motor wiring and software
polarity are the same as the `first-beacon` movement firmware. Check that both
wheels drive forwards with positive commands before a floor test.

Power the robot from its battery and switch on AP1. AP1 must run the existing
Wi-Fi and BLE beacon firmware from `picow_access_point`, with Wi-Fi SSID
`I AM PICO W`, BLE service `0xFF20`, protocol version 1 and beacon ID 1.
AP2 and AP3 are ignored. Press and release GP20, then keep the robot still
for the two-second gyro calibration. GP21 stops and latches until reset.

## Approach sequence

1. With the motors stopped, check AP1 BLE proximity. If AP1 is already close
   enough, perform the transfer without moving.
2. Otherwise, require a valid Wi-Fi starting sample. Take five fresh scans,
   require AP1 in at least three, discard the lowest and highest quarter of
   the valid readings and average the rest. Retry a missing starting sample
   three times while stopped, checking BLE between retries.
3. Learn the gyro's turn sign using a 15-degree turn and return. Probe four
   directions from the current position: drive forwards for 2400 ms, stop,
   check BLE, sample Wi-Fi, then reverse for 2400 ms. If a probe reaches
   transfer range, fetch the payload there and finish without reversing.
4. Choose the strongest valid probe RSSI. A gain over the starting value or
   a margin over another direction is not required. A single valid heading
   is enough; ties choose the first heading in the pass. All missing scores
   stop the run.
5. Turn towards the chosen heading and advance in 750 ms steps. Stop and
   check BLE before and after each step. Take a fresh Wi-Fi sample after
   an unsuccessful BLE check. Repeat the direction pass after four steps.
   If Wi-Fi disappears, remain stopped and retry detection before moving.
6. After the BLE close-range check passes, keep the motors stopped while
   transferring. A transfer failure allows three stationary attempts. Once
   proximity has passed, failed retries do not cause another movement step.
7. Verify the beacon payload's length and CRC-32, send `COMPLETE`, and require
   AP1's `COMPLETE_ACK`. Disconnect, sound a 2000 Hz tone on GP22 for 1000 ms,
   then remain stopped and quiet. GP21 also interrupts the completion tone.

The BLE close-range check retains the requested threshold of **-50 dBm**.
It uses 12 fresh target advertisements, trims the lowest and highest quarter,
and requires three consecutive qualifying averages. Samples are cleared for
each check, so readings from different positions are not mixed. Wi-Fi and
BLE scanning are performed separately, with the motors stopped while scanning
or transferring. The BLE scan window is five seconds.

The payload is the existing beacon reply, normally `Reply file from beacon AP1`.
It is received into a RAM buffer of up to 256 bytes. The robot first sends the
existing small test file because that is the beacon's transfer protocol. No
beacon firmware change is needed. Receiving an advertisement, connecting or
passing the RSSI check alone cannot trigger the completion sound. Protocol
errors, a bad sequence, a bad CRC or a missing final acknowledgement prevent
completion. Notifications received before their write acknowledgement are
held until that write finishes.

## Motion settings and stop conditions

The motion code is kept in this folder and uses the corrected `first-beacon`
controller: a checked +/-1000 degrees per second gyro range, 35% base drive
duty with heading correction, 25% turn duty reduced to 20% near the target,
and measurement during the 300 ms motor-off settling period. Target headings
are retained across movement. A turn must finish within two measured degrees
of its target after at most two corrective attempts. IMU failures, gyro rates
near the range limit, excessive heading error, a stalled turn or a turn timeout
stop the run.

A run allows at most 40 approach steps and 15 minutes, checked between phases.
The 40-step count excludes the forwards and backwards direction probes.
BLE connection and transfer phases have five-second timeouts and a bounded
cleanup period. Failure to stop the BLE operation prevents further movement.
These limits stop an unattended search from continuing after lost signals
or a failed transfer. They can be changed in `getting_there.c` and
`payload_client.c` after measuring the robot's travel.

The movement is timed, not measured in centimetres. Reversing for the same
duration only approximates the original position. RSSI varies with orientation
and reflections, so this programme does not guarantee the shortest route or
obstacle avoidance. The -50 dBm threshold is a configurable starting value;
it does not establish a particular physical distance. Test the assembled robot
on a clear floor with GP21 available to stop it.

## Stored report

The latest 32767 bytes of report text and the received payload remain in RAM.
When the report fills, whole older lines are discarded so completion or failure
messages remain available. Keep the battery on after the run, connect Pico USB
and open its serial port at 115200 baud. The stored report prints on connection;
GP20 prints it again. GP20 does not repeat the movement. Reset or power loss
erases the report and payload.

Successful runs include these lines:

```text
AP1 close-range check passed; motors stopped for payload transfer
AP1 reply payload length and CRC verified
AP1 PAYLOAD COMPLETE: verified reply and final acknowledgement
AP1 payload: Reply file from beacon AP1
COMPLETION TONE: GP22, 2000 Hz for 1000 ms
RESULT: PAYLOAD CONFIRMED
```

## Build on Windows

Run from the repository root using the installed Pico SDK v1.5.1 tools:

```powershell
cmake -S getting_there -B getting_there/build -G Ninja -DCMAKE_MAKE_PROGRAM='C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/ninja/ninja.exe' -DPICO_SDK_PATH='C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk' -DPICO_TOOLCHAIN_PATH='C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/gcc-arm-none-eabi' -DPICO_BOARD=pico_w
cmake --build getting_there/build --parallel 4
```

Flash `getting_there/build/getting_there.uf2` to the robot. The build and host
checks do not establish hardware navigation, radio range or transfer reliability.

## Host checks

These checks compile the actual navigation and BLE client sources with mocked
hardware and SDK event definitions. They cover approach steps, repeated
direction passes, missing signals, stationary retries, run limits, signed RSSI,
the close-range gate, payload verification, notifications arriving before write
completion, connection failures and cleanup. They also check the one-second
tone and stopping it.

```powershell
python getting_there/tests/run_host_checks.py --cc C:/msys64/ucrt64/bin/gcc.exe --sdk 'C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk'
```
