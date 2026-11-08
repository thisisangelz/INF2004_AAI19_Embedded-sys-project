# Beacon Lab

A separate, local 2D simulator for single-beacon tests and a three-beacon route.
It executes the actual robot and isolated AP1/AP2/AP3 beacon C firmware from a
selected Git branch while replacing the physical hardware with
deterministic models. The source repository, its checked-out branch, and firmware
files are not edited.

See [the three-beacon guide](THREE_BEACON.md) for the search design, route
completion rules, hardware assumptions and the split between firmware and model.

## Start

From the repository's `car_beacon_simulator` folder, double-click
**Start Simulator.command**, or run:

```sh
python3 simulator.py
```

Open **http://127.0.0.1:8765**. Keep the terminal running. Stop the server with
Ctrl+C when finished. If the address is already in use, the simulator may already
be running; open the URL, or use `--port 8766` for a separate instance.

Requirements: Python 3.10+, native Clang/GCC, Git, and a Pico SDK containing
BTstack. No Python or JavaScript packages need installing. The firmware repository
is detected from the simulator's parent folder. A standalone copy beside the
`INF2004_AAI19_Embedded-sys-project` folder also works; use `--repo` to select
another checkout. The SDK is detected from `PICO_SDK_PATH` or
`~/.pico-sdk/sdk/`; use `--sdk` to override it.

```sh
python3 simulator.py --repo /path/to/firmware-repository --sdk /path/to/pico-sdk
```

## Run an experiment

Select **codex/three-beacon-route** and the **route** robot build to test the new
AP1 → AP2 → AP3 firmware. Select **origin/main** with a single-target build to
compare main's existing v18 navigation and colour gate. Selecting a route build
does not turn single-target source into route firmware: an incompatible source
produces a build error. Both versions use their own real filtering, colour
classification, motor control and BLE validation. The simulator never copies
those decisions into its hardware model.

The separate `codex/beacon-simulator` branch contains the older navigation with
the previously requested **2 dB improvement** tuning. Selecting it tests that
older code, not the latest main. The source identity is recorded in each trace.

1. Select the source branch or Git ref, then its robot build: AP1, AP2, AP3 or route.
2. Choose a preset, type coordinates, or drag the car and beacons in the arena. Drag
   the small green circle above the car around its centre to rotate its starting
   direction; the Heading field updates with it. All numeric inputs accept two
   decimal places; dragging uses that same precision. After inspecting a run,
   **Edit start** returns to its original setup so you can change the next run.
3. Click **Build & run experiment**. Each run compiles a source snapshot, then
   starts a fresh firmware process and simulates the complete execution quickly
   using a virtual clock. Every click creates a new run ID, displayed below the
   playback controls. Identical inputs can produce the same duration and outcome
   because the model is deterministic; they are still separate executions.
4. The new experiment starts playing automatically, restoring the original build
   flow. Playback defaults to **20×**. Pause, step by 0.1 s, scrub the timeline,
   or jump to the final result. Imported and history recordings still open paused.
   Playback speed changes viewing speed only, not firmware
   timing or physics. **Total** is the duration of the recorded execution, known
   after it runs; it is not a preset countdown. Changing the setup, selecting
   **Custom arrangement**, or selecting another source clears the recording and
   resets the elapsed time to zero. Until the next run, the duration says **Not run**.
5. Inspect the car, colour sensor, per-beacon progress and firmware's decision log.
   Click an event to seek to it. Wi-Fi/BLE graphs distinguish ideal signal from
   integer readings delivered to firmware.
6. Compare saved runs using **Run comparison**. Use **CSV** for frame data or
   **Full trace** for settings, source identity, statistics, result and all logs.

An experiment executes only when you explicitly choose **Build & run experiment**
(or explicitly run the headless command). Page loads, setup edits, imports,
history and playback never launch a new firmware execution. Code-review and
verification work must not run experiments unless explicitly requested; see
`AGENTS.md` for the repository instruction.

## Replay a downloaded run

Click **Full trace** after an experiment to download its JSON recording. Later,
click **Load trace** beside it and select that file. It opens paused at the start;
use **Play**, the timeline, speed selector, step button or jump-to-result button
to inspect the original movement, signals and decision log. You can download the
imported trace again without changing its contents.

Replay uses the recorded frames and original outcome. It does not compile or run
the firmware again, and it works even if the original branch has changed or is
unavailable. Keep the JSON to reopen it after refreshing the page; imports are
not added to server history. Locally saved runs can also be reopened from **Run comparison**.
Use **Load setup** for starting conditions only; a setup JSON or CSV cannot replay
an experiment. Invalid or incomplete trace files leave the current run intact.

Pan by dragging empty space, zoom with the wheel, and click **Fit view** to include
the whole route. World X points right, Y points up, and positive headings turn
counterclockwise. The gyro heading is relative to the car's initial orientation;
world heading includes its configured initial orientation.

To exercise the actual firmware stop, set **Press GP21 at simulated second** and
rerun. The playback pause button only pauses viewing; it is not the robot's STOP.
GP20 is automatically pressed and released when the firmware waits for start.
GP21 retains the firmware's real semantics: before arming it can replay a log;
after arming it stops the robot and latches.

## Plug in a different branch

The branch picker reads the repository's local and remote-tracking refs and tags.
The refresh button runs `git fetch origin`; it does not check out or merge a branch.
Every trace records the exact commit, car and beacon source paths, and hardware adapter hash.
The branch selection applies to the robot and all compiled beacon instances.
Choosing a dropdown branch
also clears a previously entered custom ref so it cannot silently override your choice.

**Current working tree** snapshots saved source edits, including uncommitted edits.
For a different checkout, restart with `--repo /path/to/that/checkout`.
This simulator does not load UF2 files: it compiles the selected branch's C for
the host computer. Changes to supported C source therefore run directly.

Supported families include current `main`, the archived line-search milestone,
`Alvin_2`, `Alvin`, and `dev/tooling-setup`. They must provide:

- `getting_there/getting_there.c`, `motion.c`, `payload_client.c`, `faults.c`
- the corresponding headers and `ble_transfer` AP1 v1 protocol definitions
- `imu_mpu9250.c/.h` in `getting_there` or `pico_rssi_tracker`
- `line_search.c` and `wifi_sampler.c` where the navigation uses them

Branches with other entry points or hardware interfaces need an adapter. They
are labelled in the picker; a mismatch results in a build error, never a fake
controller. `native/navigation.c` selects the entry point, `native/runtime.c`
provides Pico hardware calls, `native/ble_peer.c` supplies BTstack transport
events, and `native/beacon_adapter.c` runs the selected beacon’s startup and registered
event callbacks in an isolated translation unit. Its GATT header is generated from
the branch’s `file_transfer.gatt`; each beacon's source and definitions come from
its own CMake target. Extend these boundaries if an interface changes.

## What executes, and what is simulated

| Executes from selected branch | Simulated boundary |
| --- | --- |
| Firmware `main`, start/calibration/approach/stop sequence | GPIO buttons, virtual time, terminal output |
| Navigation, line comparison, Wi-Fi sampler and filters | Wi-Fi scan reports containing deterministic RSSI |
| Motor controller and steering, turns and timed reversals | PWM-driven differential-drive car, equal wheels, no slip/inertia |
| IMU driver, gyro calibration, heading integration | MPU register map and quantised gyro measurements from physical motion |
| BLE client and beacon protocol, filters, packet validation, CRC, reply bytes, final ACK | Ideal BLE transport events |
| Each beacon's startup, advertising payload and SSID | Independent radio identities and modelled reception |
| TCS34725 driver, colour classification, arrival checks and mat steering | Raw RGB/clear registers from sensor position over calibrated concentric bands |
| Robot and beacon buzzer patterns | Device-owned PWM and scheduled timer callbacks |
| Fault handling | Modelled report limits; physical flash writes are not emulated |

The beacon source is compiled from the same selected branch as the car. It checks
the robot payload, constructs its own reply and issues acknowledgements using its
actual code. Changes to beacon advertisements, reply text or validation therefore
affect the run. Main currently uses `ble_transfer_powerbank/beacon_server.c`; older
compatible branches use `ble_transfer/beacon_server.c`.

The enabled beacons boot before the car, each with separate firmware state.
Their registered packet handlers continue to run; their idle heartbeat loops
and physical power-bank loads are not simulated. BTstack
internals, RP2040 instruction timing, actual interrupt concurrency, flash persistence
and radio coexistence are **not emulated**. Native C has the host ABI; exact embedded
memory layout and undefined behaviour may differ. This is a software logic and
control test, not a full MCU emulator or proof of hardware correctness.

The controller is never given the simulated car/beacon coordinates or distance.
These appear only in diagnostic telemetry. Physical movement follows the actual
PWM outputs on GP8–GP11. It is not snapped onto target headings or steered by the
simulator. Controller tolerances and deterministic gyro quantisation remain visible.

## Ideal signal and motion assumptions

Default settings:

- Noise, packet loss, orientation bias, multipath, slip and drift: **zero**.
- Full-duty wheel speed: **0.76 m/s**. With 35% duty this is **0.266 m/s**.
- Wheelbase: **0.14 m**. Both are model parameters, not measured robot calibration.
- Mat radius: **0.35 m**; sensor offset: **0.08 m** ahead of the effective driven
  axle; circular beacon box radius: **0.07 m**. Mat bands have equal radial widths.
  Adjust these assumptions to measured hardware. The user reports 8 cm from the
  rear axle and 3 cm from the front; the powered axle has not yet been confirmed.
  The point-body box-contact model blocks translation without steering the car
  or declaring firmware failure/success. It is not a complete chassis model.
- Colour readings use representative raw calibration captures from the selected
  mat profile. They are not generated by inverting the firmware's thresholds.
- Radio visibility cutoff: **5 m**, adjustable and illustrative. It is not fitted
  to the reported three-chair/four-chair range observation.
- Wi-Fi: **−50 dBm at 1 m**, path-loss exponent **2**.
- BLE: **−50 dBm** at an assumed **0.25 m**. This is a configurable model
  calibration, not a real measured range. It stays fixed across firmware branches,
  including when their proximity thresholds differ. The ring shows the selected
  firmware's threshold under that unchanged signal model.
  Before a run, no firmware threshold ring or line is shown. After execution,
  those displays use the threshold reported by the compiled firmware. Imported
  older traces without threshold telemetry retain their original −50 dBm gate.
- Scan completion: **100 ms**. BLE advertising reports: every **100 ms** while
  scanning. BLE command/transport events: **20 ms**.
- Moving Wi-Fi acquisition: **OFF**, matching current main's CMake default.
- Firmware delays and timeouts are executed in simulated time, including its
  five-second BLE checks. Therefore long stationary periods can be correct code
  behaviour even with ideal signals.

The radio model is `RSSI = reference − 10 × exponent × log10(distance/reference_distance)`.
Distance is clamped to at least 1 cm and RSSI to −126…−1 dBm. Within those bounds,
moving closer strictly increases the continuous signal. The firmware's radio
interface accepts **integer dBm**, so small changes become equal adjacent readings.
Removing quantisation would change the firmware interface and hide real threshold
behaviour. The chart shows the distinction.

## Repeatable checks

The Python suite and headless commands below execute firmware experiments and
must only be run on explicit instruction. The Node suite uses synthetic traces
and a mocked server; it does not execute firmware.

```sh
python3 -m unittest discover -s tests -v
node --test tests/test_replay.js
python3 simulator.py --headless --ref origin/main --scenario scenarios/straight.json
python3 simulator.py --headless --ref origin/main --scenario scenarios/weak-gradient.json
python3 simulator.py --headless --ref origin/Alvin --scenario scenarios/straight.json
```

The checks cover real payload completion, corrupt-reply rejection, no movement
when already close, GP21 stop, deterministic repeated execution, signal monotonicity,
world rotation/translation, actual turns/reversals, branch changes, and a deliberately
modified temporary working tree to prove that edited car **and beacon** C code
really executes. The decimal-input check covers arbitrary two-decimal coordinates
and environment values, including values produced by dragging.

`scenarios/` contains the five dashboard presets. Headless mode also accepts setup
JSON downloaded from the dashboard. Add `--moving-samples` to enable that build flag.
Complete traces are saved under `runs/`; immutable source snapshots and compiled
host executables live under `.build/`. Those generated folders can be cleared when
the server is stopped. No tests require the actual robot.

## Interpreting failures

An unsuccessful run is useful evidence. Older main (`a2641fc`) could stop with
`NO_DIRECTION` when its initial 2.4 seconds of travel produced too little signal
gain. Main `d35c860` removes that initial trial cutoff and the six-second plateau
stop. Finite side trials and the firmware's STOP, return-travel, radio, fault and
15-minute run limits remain active. A run is not required to succeed: an initially
rearward beacon can still expose the perpendicular-only search policy.
Use the **Decision log** to locate the branch and exact condition responsible.
The default decision filter includes `DIP` and `DIP_CHECK`; the BLE filter includes
the new GATT discovery diagnostics. A temporary dip is not injected by the default
noiseless model, so running a normal scenario does not prove fading recovery.
Check model speed, wheelbase and signal calibration before treating a result as
proof that the same failure will occur on hardware. The simulator does not alter
the firmware to force a successful result.

The v16 compatibility adapter supplies the active connection handle on service
and characteristic discovery events and supports disabling automatic ATT MTU
negotiation. Discovery handle ranges come from the selected beacon's generated
GATT database. These are transport interfaces, not bypasses of firmware checks.
Single-target success requires `RESULT: PAYLOAD CONFIRMED`, after reply length/CRC
validation and a valid target `COMPLETE_ACK`. Route success requires the actual
firmware's `RESULT: ROUTE COMPLETE` after all three verified transfers. A BLE
connection or one completed leg alone is not a successful route. The beacon
protocol is compiled from the selected source, independently for each identity.
