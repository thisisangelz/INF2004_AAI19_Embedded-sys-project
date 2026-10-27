# AP1 approach and payload transfer

The `line-search` branch runs this AP1 programme, based on `Alvin_2` at
`2012e7b`. Flash `getting_there/build/getting_there.uf2` onto the robot.
It approaches AP1, receives and verifies AP1's reply payload over BLE, then plays a one-second completion tone.

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

1. With the motors stopped, check AP1 BLE proximity. If it is already close
   enough, complete the payload exchange without moving. Otherwise require
   fresh Wi-Fi readings, lock AP1's BSSID and learn the gyro turn sign.
2. Try the current forward heading first. Take two fresh stationary baseline
   batches after turning. Move in blocks of at most 250 ms at 35% base duty
   with gyro correction. After each block, stop, check BLE and take two fresh
   Wi-Fi batches. As soon as improvement is confirmed, continue on that heading
   from the current position. No initial four-probe pass or heading bisection
   is required, and the old 2400 ms probe limit does not end an improving run.
3. Retain the strongest repeatable point and its accumulated powered time.
   Direction improvement and decline use the margin
   `max(3, previous_spread + current_spread + 1)` dB. Both new batches must
   support the change. Once a direction has improved, also retain a stronger
   two-batch point if it still confirms improvement over that direction's
   original baseline, even when its gain over the previous best is under 3 dB.
   This prevents small successive increases from being lost before a return.
4. Require three successive weak positions before reversing. At each position,
   both fresh batches must confirm decline against the retained best using the
   noise margin. Counted positions must be at least 250 ms of powered travel
   apart, including when a moving scan or recovered IMU error ends a block early.
   A new best or a point without confirmed decline resets the sequence. Keep the
   original best throughout the sequence. A pending sequence can extend past a
   trial deadline, but cannot bypass the run or return budgets.
   After the third position, reverse only the powered time beyond that best.
   For example, best 3010 ms and weak positions at 3260, 3510 and 3760 ms request
   a 750 ms reverse. Stop there and measure twice. Returned batches must agree
   and must not be clearly weaker than the recorded best. This checks RSSI
   consistency; timed travel does not establish exact position.
5. After a consistent return, check only the two perpendicular headings:
   90 degrees clockwise, then 90 degrees anticlockwise from the previous travel
   heading. Take new same-heading baselines. Follow the first direction that
   improves immediately; test the other side only if the first fails. A side
   trial without confirmed improvement returns approximately to its starting
   best point and rechecks RSSI before the other side is tested. Both side
   trials therefore start near the same retained point. Positive headings turn
   left with the current motor wiring.
6. Stop with `NO_DIRECTION` if neither side improves or a return check is
   inconsistent. No four-direction recovery pass is available. The initial
   forward direction can also stop without a turn if its trial ends with no
   confirmed improvement or decline. Perpendicular checks are allowed after a
   consistent decline return even if that initial direction never improved.
7. BLE may complete the run at any travel checkpoint, including a return or a
   side trial. Remain stopped during transfer and allow three stationary attempts
   after proximity passes. Verify reply length and CRC-32 and require the final
   `COMPLETE_ACK`. Only then play the one-second tone.

Initial forward and perpendicular trials allow 2400 ms of powered travel to
establish their first improvement. An initial forward trial that remains flat
stops at its current position. An unsuccessful side trial returns to its
recorded starting best and rechecks before testing the opposite side. Such a
return is part of an unsuccessful side check; an improving forward line keeps
its strongest point and uses the three-position decline rule.

A confirmed direction continues through six-second profile intervals. A flat
signal lasting six powered seconds after its last retained best stops the run
in place. If both batches have weakened but the loss is still below the margin,
continue sampling for a sustained confirmed decline, subject to run budgets.
Ending a profile interval does not trigger a return or another heading search.

Each batch contains five fresh scans, requires at least three valid target
readings, trims the lowest and highest quarter, and averages the remainder.
The two confirmation batches do not share scans. Comparing pairs uses the
strongest old batch and weakest new batch; spread includes disagreement
between the batches. This is a conservative heuristic, not a confidence
interval. One isolated strong or weak batch cannot establish a peak or decline.

## Moving samples and recorded profiles

The delivered UF2 uses **stationary sampling for navigation decisions**.
Continuous acquisition is implemented but disabled by default until the robot's
scan and STOP timing have been measured. Configure `-DLINE_SEARCH_MOVING_SAMPLES=ON`
only after that hardware check. Runtime qualification additionally requires
three complete, valid five-scan windows of at most 750 ms, with scan-start calls
of at most 5 ms. Slow or missing readings disable moving acquisition for the
rest of the run. The robot continues with stationary sampling.

When enabled and qualified, the cooperative motion loop services Wi-Fi at
5 ms intervals while maintaining gyro steering. A filtered moving decline may
request an earlier stop; reversal still requires the three stationary position
checks described above.
A pending scan finishes with the motors stopped before BLE or flash work starts.
A scan failure or timeout prevents further BLE, checkpointing and movement until
main shuts the radio down. A normal pause can finish a scan while stopped;
that observation is logged but excluded from the moving window.

The bounded RAM buffers hold 64 raw scan records and 32 line points.
The line buffer is reset for each direction and six-second profile interval;
the retained best point and its powered coordinate survive interval resets. Buffer exhaustion stops the run.
Callbacks only capture the strongest matching target result for each scan;
formatting, USB output and flash saving happen while stopped. Raw records use
callback receipt time, which is not a hardware radio-measurement timestamp.
The records are included in the existing saved previous-run log:

```text
RSSI_RAW,scan=...,t_ms=...,powered_ms=...,heading=...,dbm=...,stopped=...
RSSI_SAMPLE,scan=...,valid=...,dbm=...,spread=...,duration_ms=...,moving=...,fallback=...
LINE_POINT,line=...,point=...,t_ms=...,powered_ms=...,heading=...,scan=...,dbm1=...,dbm2=...
FOLLOW_POINT,powered_ms=...,best_ms=...,gain_db=...,margin_db=...,baseline_gain_db=...,baseline_margin_db=...,trend=...
```

`powered_ms` is the estimated position along the current direction, based
on actual motor-running time. Reversal decreases it. Settling, turning, BLE,
stationary scanning and log-saving time do not advance it. Each direction
starts a new local origin; these records are not global coordinates or distances
in centimetres. `t_ms` is the wrapping 32-bit uptime clock.

`FOLLOW_POINT` records the comparison against the retained best before any
update. `baseline_gain_db` and `baseline_margin_db` show whether the point still
confirms improvement over the direction's original baseline. `LINE_POINT`
records its two batch averages, scan identity and timestamps. `FOLLOW` messages
identify new best points, confirmed declines, partial returns and interval
changes. `DIRECTION SEARCH` identifies the two perpendicular trials.
`SIDE TRIAL` and `SIDE RETURN` identify an unsuccessful side check and its
return to the shared starting point.
Sampling pauses do not add powered travel.

The BLE close-range check retains the requested threshold of **-50 dBm**.
It uses 12 fresh target advertisements, trims the lowest and highest quarter,
and requires three consecutive qualifying averages. Samples are cleared for
each check, so readings from different positions are not mixed. Wi-Fi and
BLE scanning are performed separately, with the motors stopped during BLE scanning and transfer. The BLE scan window is five seconds.

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
of its target after at most two corrective attempts. Unrecovered IMU failures, gyro rates
near the range limit, excessive heading error, a stalled turn or a turn timeout
stop the run.

A run allows 60 seconds of total powered translation, including all trials,
following and reversals, and 15 minutes of wall-clock navigation time. A
six-second profile interval bounds the stored trace, while an improving
heading can continue through further intervals. Before every forward block,
reserve enough powered time to return to the retained best. Each translation
block reserves another 25 ms for loop/I2C overrun; an unexpected overrun ends
the run. Reserve one block and its possible return at a time, rather than
requiring the budget for both side trials in advance. Turns
retain separate timeouts. Exhausted budgets stop the motors.

## IMU communication and stopped recovery

The robot uses I2C1 on GP6/GP7 at 100 kHz, reduced from 400 kHz. Each register
selection or data transaction keeps its five-millisecond timeout. Before every
straight movement or turn, read the gyro while the motors are stopped.

An I2C error records the register, register-selection result and data-read
result. Stop PWM before attempting recovery. Up to two attempts reset the I2C
controller and verify sensor identity, wake state and gyro range, then read a
fresh sample. Controller recovery preserves sensor configuration and calibrated
bias; a changed configuration fails validation rather than changing the scale
silently. Persistent errors stop the run.

A recovered error during straight travel ends that powered block early. The
robot remains stopped for its normal settling and heading checks before another
block can start. Motor-running time freezes at the stop and excludes recovery
and settling. A block shorter than one millisecond is charged one millisecond.

Do not continue after a missed sample during a turn. For a straight movement,
accept at most two recovered gaps per run. Each must be at most 20 ms, and the
accumulated yaw uncertainty estimate must remain within 20 degrees. The estimate
uses the configured 1000 dps range plus the recovered rate over the missing
interval. Its value is added to measured heading error when enforcing the
25-degree straight-motion limit. A gap that cannot satisfy these checks stops
the run, even if the sensor becomes readable again. This is a conservative
software estimate, not an independent measurement of heading.

Recovery messages are queued with motors stopped. No USB or flash write occurs
under motor power. Useful saved-log messages are `IMU IO ERROR`, `IMU RETRY`,
`IMU RECOVERED` and `IMU GAP`. SDK timeout and acknowledgement error codes can
now be distinguished from a short transfer or a configuration mismatch.

The previous log identifies a failed gyro read but does not establish whether
its cause was wiring, supply noise, a controller state or a scheduling delay.
These changes handle bounded transient failures; they do not establish that
physical wiring or power faults have been repaired. Test the assembled robot.

BLE connection and transfer phases retain five-second timeouts and bounded
cleanup. IMU I2C transactions retain their 5000 microsecond timeout. These bounds
are not a measured 20 ms hardware STOP guarantee. Measure the assembled robot's
STOP latency and stopping distance, particularly before enabling moving Wi-Fi.

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

The banner is `line-search v7 two-directions`. The last 512 KiB of the Pico W's
2 MiB flash is reserved for a versioned, CRC-protected page journal. The linker
limits firmware to the first 1528 KiB. The next 8 KiB holds BTstack storage,
and the final 512 KiB holds the log. SDK flash operations retain the full 2 MiB
physical-chip bounds. Up to 499224 bytes of text fit when pages
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

Application `report_printf()` and `report_defer_printf()` output is persisted, including navigation,
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

Built with Pico SDK **2.3.0**, GCC ARM 10.3.1 and CMake/Ninja.
The persistent-log build uses the SDK's `pico_add_linker_script_override_path`
helper. Reserve space with `linker/pico_flash_region.ld`, while keeping
`PICO_FLASH_SIZE_BYTES=2097152`. Do not use `pico_override_flash_size` to reduce
that physical-chip size: SDK 2.3.0 rejects the journal's first erase address.
BTstack's banks are explicitly placed before the log to prevent overlap.
Replace the example paths with your installed tools:

```powershell
cmake -S getting_there -B getting_there/build -G Ninja -DPICO_SDK_PATH='C:/pico-sdk/2.3.0' -DPICO_TOOLCHAIN_PATH='C:/arm-gnu-toolchain' -DPICO_BOARD=pico_w
cmake --build getting_there/build --parallel 4
```

Flash `getting_there/build/getting_there.uf2` to the robot. The build and host
checks do not establish hardware navigation, radio range or transfer reliability.

## Host checks

These checks compile actual line-search decisions, Wi-Fi acquisition, navigation,
BLE, report, motion and IMU sources with mock hardware. They cover confirmed
peaks and declines, noise spikes, plateaux, multiple peaks, first-success
direction selection, two perpendicular trials, stopping after unsuccessful
side checks or inconsistent returns, angle
wraparound, partial and uncertain returns, timing budgets, delayed/stale scans,
moving hints and stationary fallback, BSSID, STOP and storage faults. Existing
payload CRC/final acknowledgement, notification ordering, report persistence,
power-cut recovery, boot/replay and bounded IMU failure tests remain in place.
A separate test runs the actual flash adapter with SDK-style bounds checks,
covering all 128 erase sectors, first/last pages and protected firmware/Bluetooth
regions.
An ideal path-loss model includes antenna orientation bias. It checks arrival
when the initial heading improves, and a bounded stop when a side or rear
target supplies no useful initial gradient. Removing the broad search means
those placements may stop without reaching AP1. The model assumes ideal wheel
movement and does not establish hardware navigation reliability.

```powershell
python getting_there/tests/run_host_checks.py --cc gcc --sdk 'C:/pico-sdk/2.3.0'
```

## Minimum-bit data representation

Existing BLE and protocol state retains its packed representation and compile-time
width checks. Wi-Fi sample magnitudes and spreads remain 7-bit fields. The new
line-profile timestamps and motion durations use 32-bit millisecond values; raw
records and navigation scratch data use bounded, fixed-size arrays.

| Data | Range | Bits | Where |
|---|---|---:|---|
| BLE state machine | 23 states | 5 | `payload_client.c` status word |
| Attempt / closing result | 4 values | 2 + 2 | `payload_client.c` status word |
| BLE flags (ready, running, abort, close, scan, verified, listening, service, characteristic) | on/off | 1 each | `payload_client.c` status word |
| Fault conditions (including translation, report and profile limits) | on/off | 15 meaningful bits in a `uint16_t` | `faults.c` |
| RSSI sample / Wi-Fi result | -126..-1 dBm; code 127 missing | 7 | `packed_bits.h` (`rssi_encode`) |
| 12 BLE RSSI samples | | 84 (11 bytes) | `bitarray_*` |
| 5 Wi-Fi scan scratch readings | | 5 bytes | `wifi_sampler.c` |
| Wi-Fi averaged magnitude / retained spread | 0..127 / 0..125 | 7 + 7 (2 bytes) | `wifi_sample_t` |
| Sample count / sample index / close count | 0..12 / 0..11 / 0..3 | 4 / 4 / 2 | `payload_client.c` `counters` |
| Pending length / file offsets / reply length | 0..20 / 0..256 | 5 / 9 | `payload_client.c` `counters` |
| Gyro turn sign / STOP armed | three states / boolean | 2 / 1 | `motion.c` |
| Wi-Fi BSSID | six octets | 48 | `wifi_sampler.c` |
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

## Hardware comparison before changing settings

1. Confirm the `line-search v7 two-directions` banner and keep GP21 available.
2. Lift the wheels and check motor polarity, gyro heading, turns and STOP.
   Measure STOP latency during driving, turning, delayed scans and an IMU fault.
3. On the test floor, measure forward and reverse travel for 250, 750 and 2400 ms
   at the configured duty. Record return error; equal times do not imply equal
   distances. Keep the entire outward and reverse path clear.
4. Record stationary RSSI and scan durations. Start with the default stationary
   firmware. Test moving acquisition separately only after measuring its effect
   on the motor loop and STOP response.
5. From marked positions facing towards, across and away from a stationary AP1,
   run `Alvin_2` and `line-search` three times each. Record verified payload
   completion, elapsed time, powered translation, best-point return error and
   the reason for any stop. Export GP21 logs after each run.
6. Include a reflected-signal location and a position without clear improvement.
   A bounded failure is expected when RSSI cannot supply a useful direction.

No physical robot trials were performed during this implementation.

## Startup stopping at persistent-log preparation

Versions before `line-search v2 flash-log` incorrectly set the SDK flash size to
1536 KiB while placing the log at that address. SDK 2.3.0 checks flash operations
against that size even in release builds. The first erase therefore stops the
firmware after `GP20 START: preparing persistent log; motors remain stopped`,
before gyro calibration or navigation. The earlier storage mock did not exercise
this hardware-adapter check.

Flash the rebuilt v7 UF2, reset, then press and release GP20. The log should show
`Preparing persistent log: erasing 128 sectors`, sector progress, `GP20 released`,
and then gyro calibration and radio startup. Keep the robot stationary during
calibration. `No saved run found` before the first successful run is normal.

## Startup checks in v3

Wait for `Waiting for GP20`, then press and release GP20. Before the first drive:

1. Arm GP21 STOP, prepare the saved log and confirm GP20 has been released.
   A held or shorted GP20 produces `START_BUTTON` after ten seconds. A STOP
   already latched when GP20 is accepted cancels startup and preserves the old log.
2. If the initial boot-time IMU check failed, retry it once after GP20 while
   stopped. A successful retry clears the stale `IMU` fault. A second failure
   stops startup. Calibrate the gyro for two seconds before starting the radio.
3. Start Wi-Fi and queue Bluetooth configuration in its own processing context.
   A controller power-on error is reported immediately. Wait at most ten seconds
   for Bluetooth readiness, showing progress each second. A failed radio start
   sets `BLE_RADIO`; it cannot fall through into movement.
4. Perform the stationary BLE proximity check, then collect fresh AP1 Wi-Fi.
   Missing AP1 or a scan error stops the run with a recorded reason. Only then
   learn gyro polarity with a 15-degree turn and return, and try the forward heading.

Radio callbacks queue log messages instead of writing USB. Main drains the
separate 4 KiB USB queue while stopped. A slow serial monitor therefore does not
block Bluetooth callbacks. The saved report remains separate from the live USB
display; a display queue overflow tells the operator to replay the saved log.
Repeated gyro calibration also clears the old bias before measuring a new one.

The host checks now run the real main loop through the first forward command,
including transient/permanent IMU failure, held GP20, radio/controller failure,
Bluetooth timeout, missing AP1, log preparation failure and GP21 cancellation.
The real motion controller is tested through gyro sign learning for both sensor
polarities, a subsequent drive and a stalled learning turn. Deferred serial
output is checked for callback isolation, ordering and queue wraparound.
These simulated checks and the firmware build do not verify physical motor
power, wiring, floor friction, sensor mounting or real radio reception.

## Three-position decline confirmation in v6

The latest v5 run improved from -72/-73 dBm to -57/-57 dBm at 3010 ms of powered
forward travel. The next position, at 3260 ms, was -63/-64 dBm. V5 treated that
single position as enough to reverse and later ended on an IMU read failure.
V6 records that weak position as 1/3 and continues forward to check the next
positions. Recovery above the decline threshold clears the count; three weak
positions trigger a partial return to the retained best.

Host checks replay the recorded forward RSSI observations and motor times,
then supply two synthetic alternatives. One recovers after the weak patch and
continues without reversing. The other supplies two more weak positions and a
consistent best-point recheck, producing a 750 ms reverse to the 3010 ms best.
These continuations are software tests, not predictions of physical RSSI.

Additional checks cover isolated spikes, one or two weak positions, uncertain
readings, points too close together, a pending sequence at the trial deadline,
return-budget exhaustion, successful and failed stopped I2C recovery, short
transfers, changed range, missing-sample bounds, STOP and powered-time accounting.
Startup, payload CRC/final acknowledgement, report persistence, flash bounds
and ideal forward arrival and bounded off-axis stops remain covered.

Re-test the marked AP1 position. Inspect `FOLLOW: decline position 1/3`, `2/3`
and `3/3`, the retained best in `FOLLOW_POINT`, and IMU recovery details. The
BLE close-range threshold remains -50 dBm; measure BLE beside AP1 before changing
it. A 250 ms step is a time setting, not a measured distance. Hardware trials
remain necessary.

## Two perpendicular directions only in v7

V7 removes the four-direction recovery search. After three weak positions and
a consistent return to the retained best, check the clockwise perpendicular
side first. Follow it as soon as improvement is confirmed. If it fails to
improve, return to its starting best point, check RSSI and try the opposite
side. Stop after two unsuccessful checks or an inconsistent return. A failed
initial direction with a consistent decline return uses the same two checks.
There is no heading bisection or four-direction restart.

Host tests exercise both unsuccessful sides, inconsistent initial and side
returns, flat side trials returning to their common start, following the first
improving side, and the absence of a further search. IMU recovery, sustained
RSSI decline, transfer verification and run budgets retain their checks.
