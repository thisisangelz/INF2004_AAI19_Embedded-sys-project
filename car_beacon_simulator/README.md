# Beacon Lab

A separate, local 2D simulator for the single-beacon robot. It executes the actual
car and AP1 beacon C firmware from a selected Git branch while replacing the physical hardware with
deterministic models. The source repository, its checked-out branch, and firmware
files are not edited.

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

The `codex/beacon-simulator` branch now requires a minimum **2 dB improvement**
to confirm a direction. Select that branch in **Car + AP1 source branch** for
this change; selecting `origin/main` still runs that branch's original 3 dB rule.
The existing noise margin, two-batch confirmation, 3 dB decline/return checks,
BLE proximity gate and payload completion checks remain in place.

1. Select **origin/main**, another compatible branch/tag, or enter its Git ref.
2. Choose a preset, type coordinates, or drag the car and AP1 in the arena. Drag
   the small green circle above the car around its centre to rotate its starting
   direction; the Heading field updates with it. All numeric inputs accept two
   decimal places; dragging uses that same precision. After inspecting a run,
   **Edit start** returns to its original setup so you can change the next run.
3. Click **Build & run experiment**. Each run compiles a source snapshot, then
   starts a fresh firmware process and simulates the complete execution quickly
   using a virtual clock. Every click creates a new run ID, displayed below the
   playback controls. Identical inputs can produce the same duration and outcome
   because the model is deterministic; they are still separate executions.
4. Play, pause, step by 0.1 s, scrub the timeline, or jump to the final result.
   Every recording opens paused, including a newly requested experiment. Press
   **Play** explicitly to animate it. Playback defaults to **1×**. Its speed changes viewing speed only, not firmware
   timing or physics. **Total** is the duration of the recorded execution, known
   after it runs; it is not a preset countdown. Changing the setup, selecting
   **Custom arrangement**, or selecting another source clears the recording and
   resets the elapsed time to zero. Until the next run, the duration says **Not run**.
5. Inspect the car and beacon statistics and firmware's actual decision log.
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
The branch selection applies to **both** car and AP1. Choosing a dropdown branch
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
the branch’s `file_transfer.gatt`; AP1 source and definitions come from its CMake target. Extend these boundaries if an interface changes.

## What executes, and what is simulated

| Executes from selected branch | Simulated boundary |
| --- | --- |
| Firmware `main`, start/calibration/approach/stop sequence | GPIO buttons, virtual time, terminal output |
| Navigation, line comparison, Wi-Fi sampler and filters | Wi-Fi scan reports containing deterministic RSSI |
| Motor controller and steering, turns and timed reversals | PWM-driven differential-drive car, equal wheels, no slip/inertia |
| IMU driver, gyro calibration, heading integration | MPU register map and quantised gyro measurements from physical motion |
| BLE client and beacon protocol, filters, packet validation, CRC, reply bytes, final ACK | Ideal BLE transport events |
| AP1 startup, advertising payload and SSID | Working radio and power hardware |
| Fault handling | Successful flash/report storage calls; full logs exported to disk |

The beacon source is compiled from the same selected branch as the car. It checks
the robot payload, constructs its own reply and issues acknowledgements using its
actual code. Changes to beacon advertisements, reply text or validation therefore
affect the run. Main currently uses `ble_transfer_powerbank/beacon_server.c`; older
compatible branches use `ble_transfer/beacon_server.c`.

The beacon boots before the car. Its registered packet handlers continue to run;
its idle heartbeat loop and physical power-bank load are not simulated. BTstack
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
- Wi-Fi: **−50 dBm at 1 m**, path-loss exponent **2**.
- BLE: **−50 dBm** at an assumed **0.25 m**. This is a configurable model
  calibration, not a real measured range. It stays fixed across firmware branches,
  including when their proximity thresholds differ. The ring shows the selected
  firmware's threshold under that unchanged signal model.
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

An unsuccessful run is useful evidence. For example, current main can stop with
`NO_DIRECTION` when its initial 2.4 seconds of travel produces less than its
required signal gain, even though every reading is consistent and increasing.
An initially rearward beacon can also expose the perpendicular-only search policy.
Use the **Decision log** to locate the branch and exact condition responsible.
Check model speed, wheelbase and signal calibration before treating a result as
proof that the same failure will occur on hardware. The simulator does not alter
the firmware to force a successful result.
