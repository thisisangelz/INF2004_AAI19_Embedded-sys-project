# AP1 approach and payload transfer

The `getting-there` branch runs this separate Pico W programme. Flash
`build/getting_there.uf2` onto the robot. It approaches AP1, receives and
verifies AP1's reply payload over BLE, then plays a completion tone. The
buzzer reports each stage while it runs, and the run log is saved to flash so
it can be read after a reset or power-off.

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
AP2 and AP3 are ignored. Tap GP20 (press and release) to start. The robot
waits one second so your hand is clear, then keeps still for the two-second
gyro calibration while the buzzer sounds a steady tone. GP21 stops and latches until reset.
Holding GP20 for two seconds instead prints the previous run's saved log
(see below) and does not start a run.

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
   AP1's `COMPLETE_ACK`. Disconnect, sound three rising beeps on GP22, then
   remain stopped and quiet. GP21 also interrupts the completion beeps.

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

## Buzzer sounds

All sounds play in the background from a timer, so they never pause driving,
scanning or BLE.

| Sound | Meaning |
|---|---|
| One short beep | GP20 tapped: hands off |
| Steady tone (about 2 s) | Gyro calibrating: keep the robot still. The tone stopping means calibration is done |
| One long beep (0.6 s) | A four-direction pass is starting |
| High beeps that speed up, then stay fast | BLE close-range check passed; the transfer is running. They stop when the attempt ends |
| Three beeps, each higher | AP1 payload verified: run complete |
| Three beeps, each lower | Stopped without a payload, start-up failed or GP21 pressed |
| Two very short beeps | GP20 held for 2 s: saved log will print |

The buzzer is driven by its own PWM output and a timer interrupt only when a
note changes. It does not share the gyro's I2C pins or the radio, so the tone
during calibration does not disturb the gyro reading or a transfer.

Each beacon also beeps: once when it starts advertising, continuously while
the robot is connected (the robot passed its range check), and three rising
beeps when the transfer is complete on the beacon's side. If the robot
disconnects before finishing, the beacon goes quiet.

## Run log

The latest 32767 bytes of report text and the received payload are kept in
RAM. When the report fills, whole older lines are discarded so completion or
failure messages remain available. Connect Pico USB and open its serial port
at 115200 baud: after a run the report prints on connection and a GP20 tap
prints it again. GP20 does not repeat the movement.

The report is also saved to flash, so it survives reset and power loss.
Checkpoints are written only while the motors are stopped and the radio is
idle: at the start of a run, after calibration, after each direction pass,
after each approach step and when the run ends. A battery failure loses only
the lines since the last checkpoint. The log lives in nine 4 KB flash sectors
just below BTstack's own storage at the end of flash; only sectors that
changed are rewritten.

To read the previous run's log after a reset or power-off, connect USB before
tapping GP20: it prints automatically as `SAVED LOG: RUN #n`. Holding GP20 for
two seconds prints it again, before or after a run. The previous log is kept
until a new run is started; the new run then overwrites it.

Successful runs include these lines:

```text
AP1 close-range check passed; motors stopped for payload transfer
AP1 reply payload length and CRC verified
AP1 PAYLOAD COMPLETE: verified reply and final acknowledgement
AP1 payload: Reply file from beacon AP1
COMPLETION TONE: AP1 collected, three rising beeps
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
completion, connection failures and cleanup. They also check the completion
cues, stopping them with GP21, the in-range callback and the flash
checkpoint points.

```powershell
python getting_there/tests/run_host_checks.py --cc C:/msys64/ucrt64/bin/gcc.exe --sdk 'C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk'
```

## Minimum-bit data representation

Application data is stored in the fewest bits its valid range needs. Each width
is checked at compile time with `_Static_assert(... == BITS_NEEDED(max))`, so a
field can be neither too narrow nor wider than necessary.

| Data | Range | Bits | Where |
|---|---|---:|---|
| BLE state machine | 23 states | 5 | `payload_client.c` status word |
| Attempt / closing result | 4 values | 2 + 2 | `payload_client.c` status word |
| BLE flags (ready, running, abort, close, scan, verified, listening, service, characteristic) | on/off | 1 each | `payload_client.c` status word |
| Fault conditions (IMU, gyro saturated, heading, turn, Wi-Fi lost, Wi-Fi scan, BLE radio, payload, stop latched, run limit, no direction, step limit) | on/off | 1 each in a `uint16_t` | `faults.c` |
| RSSI sample, probe score, Wi-Fi result | -1..-127 dBm | 7 | `packed_bits.h` (`rssi_encode`) |
| 12 BLE RSSI samples / 4 probe scores | | 84 / 28 | `bitarray_*` |
| Sample count / sample index / close count | 0..12 / 0..11 / 0..3 | 4 / 4 / 2 | `payload_client.c` `counters` |
| Pending length / file offsets / reply length | 0..20 / 0..256 | 5 / 9 | `payload_client.c` `counters` |
| Approach step / steps since pass | 0..40 / 0..4 | 6 / 3 | `getting_there.c` |
| Gyro turn sign | unknown, +1, -1 | 2 | `motion.c` |

Fault bits are raised with `fault_raise()` and the run report ends with
`FAULT MASK: 0x...` plus the name of each raised fault.

Shared data is updated with interrupts masked around the whole read-modify-write
(`packed_put`, `packed_set_bits`, `packed_clear_bits`, `fault_raise`,
`fault_clear`), because BTstack callbacks and the Wi-Fi scan callback run in the
async-context IRQ while the main loop reads the same words. The RP2040 (Cortex-M0+)
has no exclusive-access instructions, so C11 atomics are not used. Only core 0
touches this data; use a spin lock instead if core 1 ever does.

The wire protocol (`ble_transfer/transfer_protocol.h`) is already binary and is
unchanged. Its 8-bit message type, sequence number and 16/32-bit length and CRC
fields are fixed by the beacon firmware.
