# AP1 approach and payload transfer

The `Alvin_2` branch runs this separate Pico W programme, based on `Alvin`. Flash
`build/getting_there.uf2` onto the robot. It approaches AP1, receives and
verifies AP1's reply payload over BLE, then plays a one-second completion tone.

## Hardware and starting a run

| Function | Robo Pico connection |
|---|---|
| Left motor | M1, GP8/GP9 |
| Right motor | M2, GP10/GP11 |
| MPU-9250/6500 | 3V3, GND, SDA GP6, SCL GP7, AD0 GND |
| Start button | GP20 |
| Replay previous run BEFORE GP20 / latched STOP AFTER GP20 | GP21 |
| Onboard piezo buzzer | GP22 |

GP22 is the Robo Pico's onboard buzzer connection, as shown in
[Cytron's robot example](https://my.cytron.io/tutorial/build-an-rc-robot-car-using-pi-pico-w-and-robo-pico).
Set its mute switch so the buzzer can sound. The motor wiring and software
polarity are the same as the `first-beacon` movement firmware. Check that both
wheels drive forwards with positive commands before a floor test.

Power the robot from its battery and switch on AP1. AP1 must run the existing
Wi-Fi and BLE beacon firmware from `picow_access_point`, with Wi-Fi SSID
`I AM PICO W`, BLE service `0xFF20`, protocol version 1 and beacon ID 1.
AP2 and AP3 are ignored. Before GP20, GP21 prints the previous saved run over
USB serial without starting the robot. Press and release GP20 to replace that
log and begin a new run. Allow several seconds for flash preparation/radio
startup, and keep the robot still for the two-second gyro calibration.
After GP20, GP21 stops and latches until reset. No movement is allowed before
STOP is armed. If both buttons are pressed at idle, replay takes precedence.

## Approach sequence

1. With the motors stopped, check AP1 BLE proximity. If AP1 is already close
   enough, perform the transfer without moving.
2. Otherwise, require a valid Wi-Fi starting sample. Take five fresh scans,
   require AP1 in at least three, discard the lowest and highest quarter of
   the valid readings and average the rest. Retry a missing starting sample
   three times while stopped, checking BLE between retries.
3. Learn the gyro's turn sign using a 15-degree turn and return. Start with
   the current heading and take a new Wi-Fi baseline AFTER turning.
   Lock Wi-Fi samples to the first matching AP1 BSSID for this run.
4. Advance for 750 ms, stop, check BLE, then take a fresh Wi-Fi sample.
   Compare it to the same-heading baseline. A change must exceed the
   observed retained sample spread: the margin is
   `max(3, baseline_spread + current_spread + 1)` dB. This is a conservative
   heuristic, not a statistical confidence interval.
5. Confirm an apparent gain or loss with another stationary sample.
   Confirmed gain keeps the heading and establishes a new baseline.
   Confirmed loss causes an approximate timed retreat and a 90-degree
   change of heading. An unclear result allows up to three cumulative
   750 ms segments against the SAME baseline (2250 ms total), then rejects
   that direction. Take a new baseline after each turn/retreat; do not assume
   that timed reversal returned to the exact same position. Four consecutively
   rejected headings stop the robot for calibration/repositioning.
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

A run allows at most 40 forward segments, including exploratory trials, and
15 minutes, checked between phases. Each rejection permits at most three
reverse segments; a BLE success during retreat ends the run immediately.
BLE connection and transfer phases have five-second timeouts and a bounded
cleanup period. Failure to stop the BLE operation prevents further movement.
These limits stop an unattended search from continuing after lost signals
or a failed transfer. They can be changed in `getting_there.c` and
`payload_client.c` after measuring the robot's travel.
Each IMU I2C transaction now has a 5000 microsecond timeout. USB logging and
flash programming occur with motors stopped. These bounds are not a measured
20 ms hardware STOP guarantee; verify with the actual assembled robot.

The movement is timed, not measured in centimetres. Reversing for the same
duration only approximates the original position. RSSI varies with orientation
and reflections, so this programme does not guarantee the shortest route or
obstacle avoidance. The -50 dBm threshold is a configurable starting value;
it does not establish a particular physical distance. Test the assembled robot
on a clear floor with GP21 available to stop it.

## Previous-run log: power-bank workflow

1. Flash this version to the ROBOT. Older runs made with RAM-only firmware
   cannot be recovered.
2. Run it from the power bank using GP20. GP21 remains STOP during the run.
3. Let it finish or press GP21 and allow several seconds for cleanup/saving.
4. You can now switch it off. Connect the Pico to the computer, open its USB
   serial port (115200 is fine), and wait for startup.
5. **Press GP21 before GP20.** It prints only the saved previous run, from its
   boot/configuration lines through its result and faults. Release and press
   GP21 again to repeat. Reboots and replays do not erase or append to that log.
6. GP20 replaces the previous run; save the serial text first if you need it.
   After a run ends, GP21 also replays that run; reset is required to run again.

The banner is `Alvin_2 progress-v2 flash-log`. The last 512 KiB of the Pico W's
2 MiB flash is reserved for a versioned, CRC-protected page journal. The linker
limits firmware to the first 1536 KiB. Up to 499224 bytes of text fit when pages
are full; partial checkpoint pages reduce effective capacity. Two 32 KiB RAM
buffers hold pending text and a protected snapshot, as in the former report
implementation. This preserves the whole recorded run, not a rolling last-32-KiB
window. Storage exhaustion, formatting truncation or write failure stops further
navigation and is marked incomplete rather than silently losing earlier text.

Saving happens after completed radio operations and before movement, with motors
OFF. At run end, the radio is shut down before final saving. No flash erase or
program runs inside a radio callback or while motors are commanded on. Startup
erases the old journal only after GP20 and before radio initialisation. Flash
safety assumes only core 0 runs application code; re-audit for multicore/RTOS.

`[COMPLETE: ...]` means the recorded run was fully saved (it may still have ended
in STOP or failure). Cutting power mid-run can lose text since the last saved
checkpoint, including an in-progress BLE phase. Replay then prints the valid
prefix with `[INCOMPLETE: ...]`. Cutting power during initial log preparation can
leave no readable run. Absolute power-cut-proof logging needs power hold-up or
different storage; this implementation cannot promise the unsaved final lines.
Allow several seconds after stopping before removing power.

Only application `report_printf()` output is persisted, including navigation,
RSSI, motion, BLE transfer and fault messages. USB replay prompts and SDK panic/
debug output are not run records. The payload itself remains in RAM, but its
existing printable payload log is saved. One journal is reused per run; frequent
continuous logging wears flash. A full-chip erase or different firmware may
destroy saved reports; export important logs before reflashing.

Successful runs include these lines:

```text
AP1 close-range check passed; motors stopped for payload transfer
AP1 reply payload length and CRC verified
AP1 PAYLOAD COMPLETE: verified reply and final acknowledgement
AP1 payload: Reply file from beacon AP1
COMPLETION TONE: GP22, 2000 Hz for 1000 ms
RESULT: PAYLOAD CONFIRMED
```

## Build

Tested with Pico SDK **2.3.0**, GCC ARM 15.2.1 and CMake/Ninja.
The persistent-log build requires the SDK's `pico_override_flash_size` helper.
Replace the example paths with your installed tools:

```powershell
cmake -S getting_there -B getting_there/build -G Ninja -DPICO_SDK_PATH='C:/pico-sdk/2.3.0' -DPICO_TOOLCHAIN_PATH='C:/arm-gnu-toolchain' -DPICO_BOARD=pico_w
cmake --build getting_there/build --parallel 4
```

Flash `getting_there/build/getting_there.uf2` to the robot. The build and host
checks do not establish hardware navigation, radio range or transfer reliability.

## Host checks

These checks compile actual navigation, BLE, report, motion and IMU sources with
mock hardware. They cover confirmed weakening, cumulative small gains, outliers,
bounded search, missing signals, stationary retries, run limits, BSSID, packed
RSSI, payload CRC/final acknowledgement, early notifications, cleanup failures
and the completion tone. Additional tests exercise the real boot/start loop,
GP21 replay versus armed STOP, bounded I2C failures, logging only while stopped,
120 KB whole-report replay, reboot isolation, interrupted page writes at every
byte boundary, corruption, RAM/flash limits and storage failure.
The four-direction path-loss simulation is a control-logic check, not physical
proof of navigation or timing. See `NAVIGATION_REVIEW.md` for remaining risks.

```powershell
python getting_there/tests/run_host_checks.py --cc gcc --sdk 'C:/pico-sdk/2.3.0'
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
| Fault conditions (12 existing + report storage fault) | on/off | 13 meaningful bits in a `uint16_t` | `faults.c` |
| RSSI sample / Wi-Fi result | -126..-1 dBm; code 127 missing | 7 | `packed_bits.h` (`rssi_encode`) |
| 12 BLE RSSI samples / 5 Wi-Fi scan readings | | 84 / 35 (11 / 5 bytes) | `bitarray_*` |
| Wi-Fi averaged magnitude / retained spread | 0..127 / 0..125 | 7 + 7 (2 bytes) | `wifi_sample_t` |
| Sample count / sample index / close count | 0..12 / 0..11 / 0..3 | 4 / 4 / 2 | `payload_client.c` `counters` |
| Pending length / file offsets / reply length | 0..20 / 0..256 | 5 / 9 | `payload_client.c` `counters` |
| Approach step / trial segments / rejected headings / needs baseline | 0..40 / 0..3 / 0..4 / boolean | 6 / 2 / 3 / 1 | `getting_there.c` |
| Gyro turn sign / STOP armed | three states / boolean | 2 / 1 | `motion.c` |
| Wi-Fi identity locked / BSSID | boolean / six octets | 1 / 48 | `getting_there.c` |
| Report active / failed / closed | boolean each | 1 each | `run_report.c` |
| Journal next page / active | 0..2048 / boolean | 12 / 1 | `report_store.c` |

Gyro calculations remain floating point; reducing their precision without
measuring heading error would change control behaviour. Protocol/storage records
use fixed byte-oriented fields for a stable format, not compiler bitfield layout.

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
