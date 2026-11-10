# Continuous navigation: audit, design and test plan

Branch: `feature/continuous-navigation`, cut from `origin/two-beacon-route` at `af68d43`.
That is the firmware family that produced the 10 November route log
(`docs/runs/2026-11-10-ap1-ap2-route-wifi-scan-fault.txt`). `main` still carries v18 and has no
route code. The log itself came from an earlier commit on this branch: it collects 5 scans per
batch and treats AP1 as pink. `af68d43` moved to 7 scans per batch and blue AP1.

Labels used below: **Confirmed** means read in the code or measured in a log. **Hypothesis**
means consistent with the evidence but not proven.

## 1. Audit

### 1.1 Hardware actually used by the firmware

| Part | Interface | Where |
|---|---|---|
| Robo Pico motor driver, 2 DC motors | PWM GP8/GP9 (M1, left), GP10/GP11 (M2, right), 10 kHz | `motion.c` |
| MPU-9250 IMU, gyro Z only for heading | I2C1, GP6/GP7, 100 kHz | `motion.c`, `imu_mpu9250.c` |
| TCS34725 colour sensor | I2C0, GP0/GP1, 100 kHz, 4x gain, 103 ms integration | `colour_sensor.c` |
| Buttons | GP20 start, GP21 stop (latched) and replay | `motion.c` |
| Buzzer | GP22 | `robot_cues.c` |
| CYW43439 Wi-Fi (STA scans) and BLE (BTstack central) | on-board | `wifi_sampler.c`, `payload_client.c` |
| Flash log | last 512 KiB, `flash_safe_execute` | `report_flash.c` |

**Confirmed:** there is no IR sensor code anywhere in the repository, no wheel encoder and no
obstacle sensor. Travel is measured only as powered milliseconds. There is no RTOS and core 1 is
unused (`PICO_FLASH_ASSUME_CORE1_SAFE=1`).

### 1.2 Call graph of a route run

```text
main
 ├─ startup: report_begin, motion_init/calibrate, colour_sensor_init, cyw43_arch_init, payload_client_init
 └─ approach_route                       (one leg per beacon)
     └─ approach_beacon
         ├─ check_payload                (stationary: colour_arrival_check, then BLE only on the mat)
         ├─ wait_for_wifi → wifi_sampler_sample
         ├─ motion_learn_direction       (15° turn and back)
         ├─ follow_direction(heading)    ← the loop that dominates run time
         │    loop:
         │      translate_block          drive ≤750 ms → settle 300 ms → quiesce → check_payload
         │      measure_pair             2 × wifi_sampler_sample (2 × N full scans, motors off)
         │      decision                 best / decline / dip / trial / watchdog
         ├─ check_directions             up to 7 more follow_direction trials, return_along_line between
         ├─ mat_approach                 colour_scan_turn (12 × 30°), small steps, arrival
         └─ depart_previous_mat          (route builds, between legs)
```

### 1.3 Blocking operations and their cost

| Operation | Blocks for | Runs while | Evidence |
|---|---|---|---|
| One Wi-Fi scan (`cyw43_wifi_scan` + poll) | ≈0.71 s | stationary | Confirmed: log, 5 scans per 3535 to 3612 ms batch |
| `wifi_sampler_sample` (one batch) | N × 0.71 s: 3.6 s at N=5, ≈5.0 s at N=7 | stationary | Confirmed (N=5), arithmetic (N=7) |
| `measure_pair` (two batches, every stop) | 7.1 s at N=5, ≈10 s at N=7 | stationary | Confirmed (N=5) |
| `motion_drive_finish` settle | 300 ms | motors off | Confirmed: `MOTOR_SETTLE_MS` |
| `colour_arrival_check` | 3 × 115 ms = 345 ms | stationary | Confirmed: `read_fresh` sleeps one integration |
| `turn_degrees` | 0.5 to 1.0 s measured, 6 s timeout | turning | Confirmed: log |
| `wifi_sampler_quiesce` poll | up to 15 s (`SCAN_TIMEOUT_MS`) | stationary | Confirmed |
| BLE attempt | up to 30 s | stationary, on the mat only | Confirmed |
| `report_checkpoint` flash append | not measured | motors off, radio idle | Confirmed: rule enforced in code |
| `translate_block` drive loop | `sleep_ms(5)` per update | driving | Confirmed: about 200 Hz heading control |

**Confirmed root cause of the stationary time.** In the 10 November log every stop is
`LINE_POINT` to `LINE_POINT` ≈ 8.3 s: a 0.39 to 0.50 s drive, 0.30 s settle, 0.35 s colour check,
and 7.1 s of Wi-Fi scanning (86% of the stop). Two full-channel scan batches per stop make the
robot stationary about 92% of the time. `af68d43` raised N to 7, which makes this worse (≈10 s
per stop, by arithmetic).

**Confirmed: a single scan cannot be made shorter through the public SDK.** `cyw43_ll_wifi_scan`
overwrites `channel_num = 0` and the dwell times on every call (cyw43-driver in SDK 2.3.1), so
every scan sweeps all channels. Only `ssid` and `scan_type` are honoured.

**Confirmed: the drive itself is already non-blocking.** `motion_drive_start/update/finish` run a
heading PD controller (yaw gain 0.65, rate gain 0.10, ±8 duty points) on every 5 ms update.
`translate_block` already accepts a Wi-Fi poll hook (`wifi_sampler_poll_moving`), behind
`LINE_SEARCH_MOVING_SAMPLES=0`. In that mode moving scans can only stop a block early. Every
decision still waits for two stationary batches.

### 1.4 Shared resources and radio rules

| Resource | Users | Rule in the code |
|---|---|---|
| CYW43 radio | Wi-Fi scans, BLE attempts | One at a time. BLE only after `wifi_sampler_quiesce`. |
| Flash | `report_checkpoint` | Only with motors off and no scan open (`save_checkpoint`). After a scan failure, no flash or BLE until `cyw43_arch_deinit`. |
| USB stdio | `report_printf` | Foreground only. The host tests assert no `report_printf` while motors run. Radio callbacks use `report_defer_printf`. |
| I2C1 / I2C0 | IMU / colour sensor | Separate buses. Colour reads are stationary by design today. |
| Fault mask | everything | Atomic set and clear. |

**Confirmed:** the CYW43 driver has no scan cancel. `cyw43_ctrl.c` keeps `wifi_scan_state = 1`
until the firmware reports the end of the scan, and only driver init resets it (a `TODO need to
abort scan` comment sits in the result handler). A hung scan can only be cleared by
`cyw43_arch_deinit`, which also takes down Bluetooth.

### 1.5 Safety constraints that stay

GP21 stop latched until reset. A 15 minute mission limit. A 60 s powered-travel budget per leg
that reserves the return and departure travel. Heading error plus gyro uncertainty at most 25°.
Stop on gyro saturation. IMU recovery only while stopped, at most 2 recoveries per calibration.
Route builds stop if the colour sensor is lost. No flash or BLE while a scan may still be active.

### 1.6 The 10 November faults

**`WIFI_SCAN` + `STOP_LATCHED`.** Confirmed: `wifi_sampler_quiesce` prints the same line,
`FAILED: Wi-Fi scan stopped or timed out`, and raises `FAULT_WIFI_SCAN` for four different
causes: GP21 pressed, the 15 s scan timeout, a log storage failure or the run limit. The log
shows the batch started and no scan finished. **Hypothesis A (most likely):** GP21 was pressed
during the first scan of that batch. That is the only cause that also sets `STOP_LATCHED`, and
it is reported as a Wi-Fi fault. **Hypothesis B:** the scan hung for 15 s and GP21 was pressed
afterwards. The log cannot separate them because the line has no cause and no elapsed time.
The defect to fix is the attribution, not the radio.

**IMU read errors.** Confirmed: two `IMU IO ERROR: DRIVE, reg 0x43, read -1` events, each
recovered on the first retry. The budget is 2 moving recoveries per calibration, so this run was
one error away from a fatal `FAULT_IMU`. The cause of the I2C failures is unknown
(hypothesis: motor noise or wiring on the Grove I2C1 cable). Firmware can't remove that cause.

### 1.7 Test baseline

Confirmed on this machine (gcc 16.2, SDK 2.3.1) before any change:

- `tests/test_route.c` does not compile: four signed and unsigned comparisons under `-Werror`.
- `tests/test_navigation.c` fails at line 1237. The test clears `mat_from_check` but the mock
  still holds the previous check's ring observation, so `colour_on_mat()` stays true. This is
  a stale test after the mock gained `last_colour_observations`, not a firmware bug.
- Windows Smart App Control sometimes blocks a freshly built test `.exe` launched from
  Python (`WinError 4551`). Building with `--build-only` and launching from the shell works.

## 2. Design

### 2.1 What changes and what doesn't

The decision logic in `follow_direction` (best point, three-position decline reference, dip
recheck and probes, trials, watchdog, returns, direction search) has been tuned run by run and
is covered by over a thousand lines of host tests. It stays exactly as it is. What changes is
how each step's evidence is acquired.

| Navigation state (requested) | Where it lives |
|---|---|
| SEARCH | `approach_beacon` start, `check_directions` |
| CRUISE | **new:** `cruise_segment` inside `follow_direction` |
| LOCALIZE | existing stationary `measure_pair`, dip recheck and probes |
| FINAL_APPROACH | existing `mat_approach` (colour steering, small steps) |
| TARGET_REACHED | existing `check_payload` returning `PAYLOAD_COMPLETE` |
| RECOVERY | existing `return_along_line` and direction trials |
| SAFE_STOP | existing `run_can_continue`, fault mask and latched stop |

### 2.2 CRUISE

On the floor, with cruise enabled, one follow step becomes:

1. Drive one continuous segment of up to `CRUISE_SEGMENT_MS` (6 s) with the existing gyro
   heading control, updating every `CRUISE_CONTROL_MS` (5 ms).
2. During the segment, run Wi-Fi scans back to back without blocking. Each finished scan is
   stored with the powered position at which it arrived.
3. Poll the colour sensor without blocking, once per integration time.
4. End the segment early for: any non-floor colour reading (mat, white edge, foreign mat),
   `CRUISE_MAX_MISSING` missing scans in a row, a moving decline or loss against the best
   point, a control-loop gap over `CRUISE_MAX_LOOP_GAP_MS`, the travel budget, STOP or a fault.
5. Stop, settle, quiesce, flash checkpoint and the stationary colour arrival check. These are
   the existing `translate_block` steps and they take about 0.7 s.
6. If the segment ran its full length, the newest `2 × LINE_MOVING_HALF_SCANS` scans of this
   heading are valid, and they already confirm a new best point, use them as the step's
   evidence (a *moving pair*). Otherwise LOCALIZE with the existing stationary `measure_pair`.
7. Run the unchanged decision logic.

A moving pair is formed like a stationary pair: two batches, each the median of
`LINE_MOVING_HALF_SCANS` (3) scans with their range as the spread. The window of scans spans
the short stops between segments of one heading, because one segment alone is shorter than six
scans. The older batch was taken further back along the line, so a moving pair understates a
gain. A first version accepted any moving pair that showed no decline. In the host model that
lost about 1 dB at the end of a 4.8 s trial toward a target 4 m away, the trial failed its 3 dB
confirmation, and the robot never arrived. Moving evidence is therefore accepted only when it
already confirms a new best (`line_new_best`, the same test the stationary path uses), so
cruise never reaches a verdict the stationary measurement would not.

Cruise is used only on the floor. Once a stationary check sees the mat, the existing 250 ms
and 100 ms steps and the colour steering take over (FINAL_APPROACH). If a scan start or a
control gap breaks the timing limits, cruise switches off for the rest of the run and the
robot falls back to stationary steps. Every segment prints a `CRUISE,…` line after it stops.

### 2.3 Scan fault attribution

`wifi_sampler_quiesce` reports the cause and the elapsed scan time. A GP21 stop during a scan
no longer raises `FAULT_WIFI_SCAN`, because `STOP_LATCHED` already records it. The radio stays
marked busy, so flash and BLE are still refused until shutdown. A real timeout still raises
`FAULT_WIFI_SCAN` and ends the run, because the driver offers no safe cancel. Every
`RSSI_RAW` line also gains the scan duration (`dur_ms`).

## 3. Configuration

| Parameter | Value | File | Meaning |
|---|---|---|---|
| `CONTINUOUS_CRUISE` | 0, or 1 in `route2_cruise` | `getting_there.c` | Cruise on at boot |
| `CRUISE_SEGMENT_MS` | 6000 | `getting_there.c` | Longest floor segment. Override with `-DCRUISE_SEGMENT_MS=…` |
| `CRUISE_CONTROL_MS` | 5 | `getting_there.c` | Heading control and polling period |
| `CRUISE_MAX_LOOP_GAP_MS` | 40 | `getting_there.c` | A longer stall ends cruise for the run |
| `CRUISE_MAX_MISSING` | 2 | `getting_there.c` | Missing scans in a row that end a segment |
| `LINE_MOVING_HALF_SCANS` | 3 | `line_search.h` | Scans per moving batch (two batches per pair) |
| `SCANS_PER_SAMPLE`, `MIN_VALID_SCANS` | 7 and 5 (route2), 5 and 3 | `wifi_sampler.h` | Stationary batch size, unchanged |
| `SCAN_TIMEOUT_MS` | 15000 | `wifi_sampler.h` | Scan wait that raises `FAULT_WIFI_SCAN` |
| `MIN_PROGRESS_DB`, `MIN_DECLINE_DB` | 2 or 3, and 3 | `line_search.h` | Confirmation margins, unchanged |

Segment length in the host model (710 ms scans, cruise on): 4 s, 6 s, 8 s and 10 s give the same
result for targets 2 m away. For a target 6 m ahead the run takes 81.9 s at 4 s and 59.7 to 60.5 s
from 6 s up. 6 s is the knee, and it equals the existing 6 s no-new-best watchdog, so a segment
never drives further past the best point than the stationary search could.

## 4. Verification

| Check | Result |
|---|---|
| Host suite, every fixture, WSL Ubuntu gcc 15.2 | Passes (`run_host_checks.py`) |
| New sampler tests (GP21 during a scan, timeout text, `dur_ms`) fail against the old sampler | Confirmed |
| Cruise segment ends: length, decline, lost, timing, STOP, colour | Each has a passing test |
| Colour poll: one read per integration, never waits | Passing test in `test_colour.c` |
| Cross-build: `getting_there`, `getting_there_route`, `getting_there_route2`, `route2_cruise` | Link with SDK 2.3.1 |
| `CONTINUOUS_CRUISE=1` only on `route2_cruise` | Confirmed in `build.ninja` |
| On the robot | **Not run** |

Host model comparison (`tests/test_navigation.c`, `check_cruise`; set `CRUISE_DUMP=1` to print).
The model drives 0.20 m per 750 ms, reads RSSI as `-50 - 20 log10(d)` plus a heading term, has
no RSSI noise and places a pink mat around the beacon.

| Target (m) | Stationary time | Cruise time | Driving share before → after | Powered travel |
|---|---|---|---|---|
| (2, 0) | 110.1 s | 36.6 s | 6.2% → 18.5% | 6800 → 6770 ms |
| (0, 2) | 224.4 s | 92.1 s | 7.3% → 17.8% | 16400 → 16370 ms |
| (−2, 0) | 281.5 s | 119.8 s | 7.5% → 17.7% | 21200 → 21170 ms |
| (0, −2) | 167.2 s | 64.3 s | 6.9% → 18.0% | 11600 → 11570 ms |
| (4, 0) | 228.8 s | 52.0 s | 6.3% → 27.6% | 14300 → 14350 ms |
| (6, 0) | 347.0 s | 59.9 s | 6.3% → 36.5% | 21850 → 21845 ms |
| (3, 3) | 558.4 s | 190.7 s | 7.9% → 23.1% | 44000 → 44070 ms |

The test asserts that cruise arrives at every target with powered travel within 2%, the same
number of reversals, and at most half the time.

**Why the share stays under 50%.** What remains is stationary by design and outside the
segment: the opening Wi-Fi sample and baseline pair of each leg (15 scans), the baseline pair of
each new heading, a stationary verdict whenever moving scans don't confirm a new best, the 0.7 s
stop after each segment, and colour steering on the mat. In the 2 m case that is about 25 s of
36.6 s. Removing more of it means changing the confirmation logic, which this change leaves
alone. The model has no RSSI noise, so real moving scans may confirm less often than this.

## 5. Hardware test procedure

Build (Pico SDK 2.3.1 from the VS Code extension, or these commands in Git Bash):

```bash
export PATH="$HOME/.pico-sdk/cmake/v4.3.4/bin:$HOME/.pico-sdk/ninja/v1.13.2:$HOME/.pico-sdk/toolchain/15_2_Rel1/bin:$PATH"
cmake -S getting_there -B build-route -G Ninja -DPICO_BOARD=pico_w \
  -DPICO_SDK_PATH=$HOME/.pico-sdk/sdk/2.3.1 -DPICO_TOOLCHAIN_PATH=$HOME/.pico-sdk/toolchain/15_2_Rel1 \
  -Dpicotool_DIR=$HOME/.pico-sdk/picotool/2.3.1/picotool
ninja -C build-route getting_there_route2 route2_cruise
```

Flash by holding BOOTSEL, plugging in USB and copying `build-route/route2_cruise.uf2` to the
`RPI-RP2` drive. Host tests: `python3 getting_there/tests/run_host_checks.py --sdk <SDK path>`
(on this Windows machine, run it inside WSL; Smart App Control blocks the test executables).

1. **Wheels lifted.** Start with GP20 and let it run one minute. Press GP21 during a segment:
   the wheels must stop within a few milliseconds. Replay the log (GP21 before GP20). Check that
   every `CRUISE,…` line has `max_gap_ms` of 10 or less, and that GP21 shows as `STOPPED: GP21
   STOP during Wi-Fi scan …` with no `FAULT: WIFI_SCAN`.
2. **Same course, both builds.** Use the course and start pose of the 10 November run. Run
   `getting_there_route2`, then `route2_cruise`, three times each, without pressing GP21.
   Save each replayed log to `docs/runs/` and compare them on the run log dashboard
   (`feature/run-log-dashboard`).
3. **Pass criteria.** Cruise arrives wherever the stationary build arrives. No `FAULT: WIFI_SCAN`.
   No `end=timing`. Driving share above the stationary build's. Any `end=colour` should be at
   the mat edge or a foreign mat, not on bare floor.
4. **Also read from the logs.** `RSSI_RAW … stopped=0` against `stopped=1` spread: if moving
   scans are much noisier than stationary ones, motor noise is affecting the radio. `dur_ms`
   per scan: anything far above 710 ms suggests radio contention.

## 6. Remaining limits

- **Not tested on the robot.** Every number above comes from the host model.
- **Scan start cost on hardware is unknown.** If `cyw43_wifi_scan` blocks the control loop for
  more than 40 ms, cruise turns itself off for the run and logs why.
- **Colour while moving.** The sensor integrates over 103 ms of travel. A floor reading taken as a
  ring only costs a stop. A ring missed between two polls lets the robot run about one poll period
  (about 3 cm at model speed) past it before the next reading.
- **A hung scan still ends the run.** The driver has no scan cancel. The only recovery is
  `cyw43_arch_deinit` and a fresh init, which also restarts Bluetooth. That was not attempted.
- **IMU recovery budget.** Two moving recoveries per calibration. The 10 November run used both.
  A per-time budget would be safer, since the heading uncertainty bound already guards the heading.
- **No IR sensing exists in the code**, so there is nothing to integrate.
- **The main loop was not rewritten as independent periodic managers.** The procedural search
  stays, with non-blocking work only inside the drive segment. A full rewrite would discard the
  tuned decision logic and its tests without a robot to verify against.

**Hardware that would help most.** Wheel encoders, to measure travel instead of assuming it from
powered milliseconds. That would make returns, the 60 s budget and position tags on moving scans
real distances. An obstacle sensor for the beacon box. Another radio would not shorten scans:
the limit is the full-channel sweep, not radio count.
