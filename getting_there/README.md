# Pico approach and payload transfer

This programme developed from `Alvin_2` at `2012e7b` through the archived
`line-search` branch. This branch adds the AP1 → AP2 → AP3 route below.
The tracked files under `build/` are earlier main's prebuilt images; they do not
contain this branch's new changes. Build this branch and use its fresh output.


## Current route build: AP1 → AP2 → AP3

CMake target `getting_there_route` defines `THREE_BEACON_ROUTE=1` and produces
`getting_there_route.uf2`. Existing `getting_there`, `robot_ap2`, `robot_ap3`
and `colour_scan` targets remain available. The common navigation fixes below
also apply to the single-target builds.

The locally built route image is copied to
`getting_there/build-v2/getting_there_route.uf2` (generated and ignored by Git).
The firmware-files table later in this document records the older main images;
use the route image for this branch's three-target mission.

- Every heading, including the initial heading, has a 2400 ms powered no-gain
  trial. Positive but unconfirmed improvement permits one extension to 4800 ms.
  No gain returns approximately to that trial's start before testing −90°,
  +90° and 180°, then the intermediate +45°, −45°, +135° and −135° directions.
  All eight headings together still share the existing travel/run budgets.
  Each trial measures a fresh same-heading baseline; the
  spread-aware minimum 3 dB margin is unchanged. No coordinates are used.
- After progress, 6000 powered ms or 120 seconds without a supported new best
  triggers a timed return to the retained best and direction reacquisition.
  This watchdog is logged separately from a confirmed signal decline.
  Invalid/read-failed mat scans restore their entry heading before fallback.
- The route shares one 900000 ms mission deadline and one persistent 512 KiB
  journal. Each leg has 60000 ms powered translation, including returns and its
  departure; the first two legs reserve departure time. The gyro is calibrated
  and its turn sign learnt once. Heading, STOP latch and mission clock persist.
- Route builds require a working colour sensor and genuine colour arrival.
  An inferred centre stops with a colour fault. Single-target builds retain
  their labelled inferred-centre override and missing-sensor BLE fallback.
- After CRC and final ACK verification **and successful BLE cleanup**, log and
  checkpoint the reply and `ROUTE: confirmed=1,mask=1` (then masks 3 and 7).
  `ROUTE: target=1`, `2`, `3` mark the active leg. Only mask 7 may produce
  `RESULT: ROUTE COMPLETE`; intermediate legs produce no overall payload result.
- Departure keeps the previous target active. It drives opposite the last
  successful approach block, including a reverse approach. A fresh valid
  colour gradient followed by a successful inward turn also supplies an
  observed departure heading for arrival by rotation. It makes no extra turn
  beside the box. Missing evidence or >5° misalignment stops the route.
  This is an observed approach direction, not exact physical path replay.
- Departure uses at most 3000 powered ms, with STOP polling, no BLE calls,
  three fresh all-mat floor readings and another 250 powered ms clearance
  followed by another floor check. White edges and other mats are not floor.
  Failure stops with the completed mask retained and the old target unchanged.
- Only after departure and Wi-Fi quiescence does main atomically select/reset
  under the CYW43 async-context lock. BLE must be idle/disconnected, with no
  listener, queued callback, unacknowledged scan command or uncertain cleanup.
  Wi-Fi reinitialisation drops the old BSSID lock; arrival/lighter-ring memory,
  centre override, mat scan count, profiles and translation budget reset per leg.
  Log exhaustion remains a stopped failure; no leg erases or restarts the log.

### AP3 search correction after the 8 November trace

The supplied [failed route trace analysis](../docs/runs/2026-11-08-route-ap3-no-direction.md)
records two verified transfers and a later `NO_DIRECTION`, with AP3 still visible
at -60 dBm. A white edge on AP2's blue mat was accepted as the new target's
white edge. After the inconclusive colour scan, blue rings read as floor in
the purple-only profile, so the robot continued into the previous beacon box.
Its timed return then overestimated actual motion. A later improving heading
only gained 2 dB within its trial; the four original headings exhausted.

Colour steering now requires an actual coloured ring of the active mat, not
white alone. The stationary check also records raw foreign coloured rings;
encountering one rejects the trial and returns before the next heading. Dip
probes stop on that evidence too. New `target_ring` and `foreign_mat` log fields
share the one-byte colour-observation container. White still selects small
steps but is not itself a bearing or foreign-mat proof. After cardinal trials,
the four diagonals use exactly the same confidence margin and bounded travel.

These changes use only physical sensor readings. They do not read simulator
positions or a collision flag, and do not make timed reversing into measured
odometry. Unmarked obstacles, sensor errors, slip and ambiguous colours remain
hardware limitations. No beacon protocol change or simulator range, signal,
geometry, colour calibration or success-rule change was made for this fix.
Build-only checks are not evidence that the submitted arrangement now succeeds;
assess a new, explicitly requested run from its own trace.

The runtime API is in `target_beacon.h`: ID (`uint8_t`), name/SSID (`const char *`),
SSID length (`size_t`, excluding NUL), mat (`colour_mat_t`) and select (`bool`,
ID 1–3). Single-target builds reject another ID. On hardware use
`payload_client_reset_target_qualification(uint8_t id)`, after draining Wi-Fi,
for the guarded atomic selection. `payload_client_quiescent()` is synchronised
with BTstack callbacks. The wire file text remains target-specific and unchanged.

Compile/link host fixtures without executing them:

```sh
python3 tests/run_host_checks.py --sdk /path/to/pico-sdk --build-only --output /tmp/route-host-build
```

This change was validated by compile/link only. No host fixture, firmware run,
flash operation or physical three-beacon route experiment was performed. Scalar
RSSI, colour inference and timed returns do not guarantee a hardware bearing,
collision clearance or arrival. The sections below retain earlier run history;
this section supersedes their former unlimited initial/plateau policy.

## Firmware files

| Board | File | What it is |
|---|---|---|
| Robot, going to AP1 (pink mat) | `build/getting_there.uf2` | Current robot firmware: v18 navigation with mat steering, colour-gated transfer, buzzer sounds. Banner `line-search v18 steps500-mat250 back-search mat-steer colour-gate own-mat ble-minus58 cues \| AP1 only`, then `Colour mat: PINK (AP1)` |
| Robot, going to AP2 (blue mat) | `build/getting_there_ap2.uf2` | The same firmware aimed at AP2 (`I AM PICO W 2`, beacon ID 2, blue mat); banner ends `AP2 only` |
| Robot, going to AP3 (purple mat) | `build/getting_there_ap3.uf2` | The same firmware aimed at AP3 (`I AM PICO W 3`, beacon ID 3, purple mat); banner ends `AP3 only` |
| Robot (test) | `build/getting_there_colour_scan.uf2` | Colour scan test only: turns on the spot, no Wi-Fi, BLE or driving |
| Robot (tool) | `../tcs34725_calib/build/tcs34725_calib.uf2`, `tcs34725_calib6.uf2` | Colour sensor calibration tool: pink 5-ring mat, blue and purple 6-ring mats |
| AP1-AP3 | `../picow_access_point*_powerbank/build/picow_access_point*_powerbank.uf2` | Current beacon firmware with buzzer sounds |

## One colour mat per beacon (branch feature/colour-ring-sensing)

Each colour reading identifies its mat as well as its ring, from the
calibrations of the pink mat (6 November, 5 rings) and the blue and purple
mats ([7 November](../docs/calibration/2026-11-07-tcs34725-blue-purple-mats.md),
6 rings each). The three mats do not overlap in colour: blue/red splits the
floor and pink from blue and purple, and green/red splits blue from purple.

Each beacon has its own mat: **AP1 pink (red), AP2 blue, AP3 purple**
(`target_beacon_mat()` in `target_beacon.h`). The robot build for a beacon treats the
rings of the other two mats as floor, so they never start colour steering or
count toward arrival. A white edge always counts as the beacon's own edge,
because the edges differ only slightly (on 7 November the purple edge read as
pink white). The start of each run logs `Colour mat: PINK (AP1)` (or BLUE
(AP2), PURPLE (AP3)). The colour scan test build still recognises all three.

| Mat | Rings | Ring cue | Arrival |
|---|---|---|---|
| Pink | 5 | G/R falls inward (C and B/R separate the floor) | ring 4 or darker |
| Blue | 6 | B/R rises inward (G/R splits rings 5 and 6) | ring 5 or darker |
| Purple | 6 | B/R rises inward (G/R splits white edge and ring 1) | ring 5 or darker |

Arrival is one ring short of the darkest on every mat, so the robot's front
stays clear of the beacon box. All three readings of a check must be on the
same mat, and a lighter ring of that same mat must have been seen first (by
an earlier check or a colour scan). `COLOUR` lines now show `mats=` and `br=`,
and `COLOUR_SCAN` lines show `mat=` and `br=`. Floor-like shadows, including a
person's shadow at G/R 975 and B/R 842, read as floor: the pink white edge now
needs B/R of at least 857 (ring 1 still 842).

One firmware per target beacon: `getting_there.uf2` (AP1),
`getting_there_ap2.uf2` and `getting_there_ap3.uf2` (CMake targets
`getting_there`, `robot_ap2`, `robot_ap3`, set by `TARGET_BEACON_ID` in
`target_beacon.h`). A host check reads each beacon's `CMakeLists.txt` and
confirms the robot build looks for the same Wi-Fi SSID and beacon ID. The additional `getting_there_route` target visits all three in one run, subject
to the stopped-failure conditions described above.

## v18: bigger steps and searching behind (branch feature/bigger-steps-and-back-search)

Changes to the v16 line search, made after the 7 November three-chair runs
(success facing AP1; failures sideways and facing away). The v16 sections
below still describe everything else; v17 is holsoma's separate coarse-fine
experiment on `experiment/coarse-fine-navigation`.

| | v16 | v18 |
|---|---|---|
| Forward step on the floor | 250 ms | **500 ms** (`LINE_FORWARD_BLOCK_MS`) |
| Forward step once the colour sensor sees the mat | 250 ms | 250 ms (`LINE_MAT_BLOCK_MS`) |
| Dip probes after three weak positions | 2 x 100 ms | **2 x 250 ms** (`LINE_DIP_PROBE_MS`) |
| Directions tried after a line declines | left and right | **left, right, then straight back** |
| Return RSSI disagrees with the best point | run stops | **logged; the search continues** |

Why:

- **Fewer stops.** Every stop costs about 13 s (BLE check plus two Wi-Fi
  batches). The successful three-chair run needed about 8 s of driving in
  about 32 stops (7 min 20 s); 500 ms steps halve the stops on the floor, which also
  leaves time within the 15-minute limit for four chairs.
- **Weak patches.** Both three-chair logs show a sudden 12-17 dB drop about 3 s
  into a good line. In the sideways run it lasted longer than v16's check
  (three weak positions in 0.75 s plus 0.2 s of probes), so the robot reversed
  off the line that led to AP1. The same three positions now span 1.5 s plus
  0.5 s of probes.
- **Facing away.** v16 never tried the direction behind it, so a robot started
  facing away from AP1 could not turn round; that run also stopped on an
  inconsistent return. The third trial is straight back along the last line.
- **On the mat** the robot is already close, and 250 ms steps keep it from
  driving past the centre; within two rings of the arrival ring (pink ring 2,
  blue or purple ring 3, and inward) steps are 100 ms. The switch follows the
  colour sensor reading at each stop and is logged as `STEP: colour sensor sees
  the mat; forward blocks now ... ms`.

### Colour gate and mat steering

The [side-on run](../docs/runs/2026-11-07-side-on-mat-ring1-transfer.md)
drove along the rings and transferred on ring 1, because BLE alone reached
-58 dBm there. Two further changes:

- **Colour gate.** With a working colour sensor the transfer happens only
  after colour arrival (ring 4 or darker after a lighter ring), and BLE then
  only needs -85 dBm. Until then each stop logs `ARRIVAL: not on RING4 or
  darker yet; BLE transfer waits for the colour mat` and skips the 5 s BLE
  check, so floor stops take about 8 s instead of 13 s. A missing sensor, or
  one disabled after three failed reads, brings back the -58 dBm BLE gate.
- **Mat steering.** As soon as a stop sees the mat (white edge or any ring),
  or the robot starts on it, colour steering replaces the Wi-Fi line
  (`FOLLOW: colour sensor sees the mat ...; colour steering takes over`). The
  robot does the colour scan from the scan test (12 stops of 30 degrees,
  `COLOUR_SCAN` lines), turns to the estimated centre direction (`MAT: scan N
  points ...`), then drives up to 1 s in steps (`MAT_STEP` lines, with `ms=`),
  checking the colour at each stop: 250 ms steps on the outer rings, 100 ms
  within two rings of the arrival ring. The blue and purple mats fit six rings
  where pink has five, so their inner rings are narrower; with 250 ms steps the
  robot drove into the beacon box on the blue mat on 7 November and nearly on
  the purple one. In the host model the 100 ms steps stop on the arrival ring
  itself from every start, where 250 ms steps reached blue ring 6. It scans again if the surface gets lighter. If a
  scan sees no ring, a read fails, or six scans have been used, the Wi-Fi
  search resumes. On ring 4 with AP1 failing even the -85 dBm gate it never
  drives on into the beacon box: three stationary retries, then a payload fault.
  A scan that sees a lighter ring also counts for arrival, so a robot started
  on ring 4 can still arrive.
- **Floor vs ring 1.** Under the brighter 7 November light the floor read as
  ring 1. White edge and ring 1 now also need blue/red >= 842; see the
  [calibration update](../docs/calibration/2026-11-06-tcs34725-mat.md).

Unchanged: the decline and recovery rules, return travel, the 2400 ms trial
length (now five stops), the 60 s powered-travel and 15-minute limits and the
arrival rule itself. Host checks keep v16's step sizes for the recorded-run
regressions and add v18 cases: step sizes, a 1.1 s weak patch that v16 reverses
on and v18 drives through, and all four model target directions (including
directly behind) at both step sizes.

## Colour scan test (branch feature/colour-scan-mat-direction)

Test firmware: `build/getting_there_colour_scan.uf2` (CMake target
`colour_scan`). Banner line `TEST BUILD: colour scan on the spot after
calibration; no radio, no approach`. It checks whether turning on the spot can
tell the robot which way the mat centre is, before any steering uses it.

Because the colour sensor sits ahead of the wheels, turning on the spot swings
it across neighbouring rings. After GP20, gyro calibration and the 15-degree
direction check, the robot turns through a full circle in 30-degree steps,
stops at each and keeps the median of three colour readings. It does this
twice, returning to the starting heading after each round. No Wi-Fi or BLE is
used and the robot does not drive.

Procedure: place the robot on the mat side-on across a ring (and in later tries
facing away from the centre, or on the white edge), press GP20, keep clear
while it turns, then read the log with GP21 as usual. Each step logs:

```text
COLOUR_SCAN,round=1,angle=90,heading=90,zone=RING3,depth=4,c=3770,r=1573,g=993,b=1025,gr=631
COLOUR_SCAN_RESULT,round=1,valid=1,darkest_angle=90,vector_angle=84,depth_range=2
```

`angle` is relative to the starting heading, in the log's heading convention.
`depth` is 0 floor, 1 white edge, 2-6 rings 1-5. `vector_angle` is the
depth-weighted direction estimate and `depth_range` how many rings the sweep
spanned; 0 means the turn could not tell directions apart. Compare
`vector_angle` with the real direction to the centre, and the two rounds with
each other. The result ends `RESULT: COLOUR SCAN COMPLETE` with the rising
completion beeps, or `COLOUR SCAN FAILED` with the falling ones.

A host check simulates a 35 cm mat with five equal rings and a sensor 6-10 cm
ahead of the wheels: the estimate points within 30 degrees of the centre when
placed side-on, facing away, diagonally or on the white edge next to the floor.
## Buzzer sounds (branch feature/buzzer-status-cues)

Firmware: `build/getting_there.uf2`, banner ending `colour-ring4 cues`. Navigation and colour arrival are unchanged. The sounds
come from the archived `robot-buzzer-cues-flash-log` design and play from a
timer alarm on GP22, so they never pause driving, scanning or BLE.

| When | Sound |
|---|---|
| GP20 accepted | One short beep |
| Gyro calibrating (keep still) | Steady tone; it stops when calibration is done |
| A new search direction starts (first line and each perpendicular trial) | One long beep (0.6 s) |
| BLE close-range check passed, transfer running | High beeps that speed up, then stay fast until the attempt ends |
| Payload verified | Three beeps, each higher (replaces the three same-pitch beeps) |
| Stopped without a payload, start-up failed or GP21 | Three beeps, each lower |

The beacons have matching sounds; see `ble_transfer_powerbank/README.md`.
## Colour-ring arrival (branch feature/colour-ring-sensing)

Firmware: `build/getting_there.uf2` (it also includes the buzzer sounds
above). Boot banner: `line-search v16 dip-probes ble-minus58 colour-ring4 cues`. The v16 navigation is
unchanged; only the arrival decision before each BLE check is new.

The TCS34725 colour sensor on Grove 1 (I2C0, SDA GP0, SCL GP1) classifies the
surface under the robot's front as floor, white edge or ring 1-5, using the
thresholds in [the calibration record](../docs/calibration/2026-11-06-tcs34725-mat.md).
At every stationary BLE check, before BLE starts, the robot takes three fresh
colour readings (about 0.35 s):

- **Arrived:** all three are ring 4 or darker, and a lighter mat surface (white
  edge or rings 1-3) was seen at an earlier check in this run. BLE then only has
  to show that AP1 is in range: filtered average >= -85 dBm
  (`COLOUR_ARRIVAL_BLE_DBM`) three times in a row. The handshake still checks
  beacon ID 1 and the payload CRC.
- **Not arrived:** the strict -58 dBm BLE gate applies exactly as in v16.

The lighter-ring requirement stops a dark or red object on the floor, or a run
that starts on the mat, from counting as arrival. Shadows do not change the
classified ring because the sensor lights the surface with its own LED.
Stationary retries after arrival keep the loose gate without re-reading colour.

If the sensor is missing at startup, or three colour checks in a row fail, the
robot logs this and keeps the v16 BLE-only behaviour. Every check is logged:

```text
COLOUR,zones=RING3/RING4/RING4,c=2640,r=1333,g=629,b=666,gr=471,lighter_seen=1,arrived=0
ARRIVAL: colour sensor confirms RING4 or darker; BLE only needs AP1 >= -85 dBm
```

On-mat BLE averages measured on 6 November were -64 to -73 dBm, so -85 dBm
leaves margin. Not yet tested on the robot: whether 250 ms steps sample each
ring before ring 4 is reached, and readings just after the robot stops.
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
Wi-Fi and BLE beacon firmware from `picow_access_point_powerbank`, with Wi-Fi SSID
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
   Wi-Fi batches. Continue on that heading through inconclusive readings,
   subject to the run and travel limits. The initial heading does not require
   confirmed improvement within 2400 ms. No initial four-probe pass or heading
   bisection is required.
3. Retain the strongest repeatable point and its accumulated powered time.
   Direction improvement uses the margin
   `max(3, largest_individual_batch_spread + 1)` dB across the four batches
   being compared. Both new batches must
   support the change. Once a direction has improved, also retain a stronger
   two-batch point if it still confirms improvement over that direction's
   original baseline, even when its gain over the previous best is under 3 dB.
   This prevents small successive increases from being lost before a return.
4. Use a separate reference for decline: the median of three positions, ranked
   by each position's weaker batch. Keep both batches from the selected position.
   The reference starts at the direction's baseline until three positions are
   available. It can strengthen but never weaken. Freeze it and its three-position
   history during a suspected dip, so weakening readings cannot lower the reference.
   Require three successive weak positions, each at least 250 ms of powered travel
   apart. Both batches must be weaker than both reference batches by **more than**
   `max(3, largest_individual_batch_spread + 1)` dB. Equality does not count as
   decline. A point without confirmed decline clears the consecutive count;
   an inconclusive point leaves the reference frozen until recovery is confirmed.
5. After the third weak position, stop and recheck BLE, then collect two new Wi-Fi
   batches. If they recover within the consistency tolerance, continue forwards.
   Otherwise allow at most two additional **100 ms forward probes**, with a stopped
   BLE check and two fresh Wi-Fi batches after each. Recovery ends probing and
   resumes normal 250 ms blocks on the same heading. A missing signal, STOP, fault
   or exhausted budget prevents further probing. A pending dip can extend a side
   trial, but never bypasses run or return budgets.
6. If both probes fail to recover, reverse only the accumulated powered time
   beyond the retained best point, including the probes. For example, best 2500 ms,
   third weak position 3500 ms and two 100 ms probes request a 1200 ms reverse.
   Stop there and measure twice. The return and dip-recovery checks both require
   batch agreement and no loss beyond the consistency tolerance. This tolerance
   is `max(3, reference_spread + current_batch_spread + 1)` dB; reference spread
   includes its two-batch disagreement. Batch agreement uses the two current
   spreads in the same formula. Equality at the tolerance is accepted. These are
   RSSI checks; timed travel does not establish exact position.
7. After a consistent return, check only the two perpendicular headings:
   90 degrees clockwise, then 90 degrees anticlockwise from the previous travel
   heading. Take new same-heading baselines. Follow the first direction that
   improves immediately; test the other side only if the first fails. A side
   trial without confirmed improvement returns approximately to its starting
   best point and rechecks RSSI before the other side is tested. Both side
   trials therefore start near the same retained point. Positive headings turn
   left with the current motor wiring.
8. Stop with `NO_DIRECTION` if neither side improves or a return check is
   inconsistent. No four-direction recovery pass is available. Inconclusive
   initial forward readings continue within the run and travel limits.
   Perpendicular checks are allowed after a
   consistent decline return even if that initial direction never improved.
9. BLE may complete the run at any travel checkpoint, including a probe, return or a
   side trial. Remain stopped during transfer and allow three stationary attempts
   after proximity passes. Retries reconnect to the same qualified BLE address
   without repeating the RSSI check or moving again. Verify reply length and
   CRC-32 and require the final `COMPLETE_ACK`. Only then play three 200 ms beeps
   at 2000 Hz, separated by 150 ms gaps. GP21 can interrupt the beeps.

Perpendicular trials allow 2400 ms of powered travel to establish their first
improvement. An unsuccessful side trial returns to its recorded starting best
and rechecks before testing the opposite side. The initial forward heading has
no separate trial deadline. It continues towards a BLE checkpoint, confirmed
decline, STOP, fault or the existing run/travel limits.

Both the initial heading and a confirmed direction continue through six-second
profile intervals, even if the signal stays flat or weakens by less than the
decline margin. Ending a profile interval only clears the bounded recording
buffer; it does not stop, return or trigger another heading search.

Each batch contains five fresh scans, requires at least three valid target
readings, trims the lowest and highest quarter, and averages the remainder.
The two confirmation batches do not share scans. Comparing pairs uses the
strongest old batch and weakest new batch. That comparison already accounts
for disagreement between batches, so the progress margin uses only the largest
individual retained spread, plus 1 dB for rounding, with a 3 dB minimum. This
is a heuristic, not a confidence interval. One isolated strong or weak batch
cannot establish a peak or decline. The additional three-position median reduces
the effect of a whole position reading unusually strongly. Each normal movement
adds at most one position to this history, after any dip checks; stationary
rechecks are not additional positions. The best return point remains independent
of this reference. Approximate return checks and optional moving-sample hints
retain their previous tolerances.

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
request an earlier stop; reversal still requires the stationary position checks,
recheck and bounded probes described above. Dip probes use stationary sampling.
A pending scan finishes with the motors stopped before BLE or flash work starts.
A scan failure or timeout prevents further BLE, checkpointing and movement until
main shuts the radio down. A normal pause can finish a scan while stopped;
that observation is logged but excluded from the moving window.

The bounded RAM buffers hold 64 raw scan records and 32 line points.
The line buffer is reset for each direction and six-second profile interval;
the retained best point, its powered coordinate and the decline reference survive
interval resets. Each new heading starts its own reference. Buffer exhaustion stops the run.
Callbacks only capture the strongest matching target result for each scan;
formatting, USB output and flash saving happen while stopped. Raw records use
callback receipt time, which is not a hardware radio-measurement timestamp.
The records are included in the existing saved previous-run log:

```text
RSSI_RAW,scan=...,t_ms=...,powered_ms=...,heading=...,dbm=...,stopped=...
RSSI_SAMPLE,scan=...,valid=...,dbm=...,spread=...,duration_ms=...,moving=...,fallback=...
LINE_POINT,line=...,point=...,t_ms=...,powered_ms=...,heading=...,scan=...,dbm1=...,dbm2=...
FOLLOW_POINT,powered_ms=...,best_ms=...,gain_db=...,margin_db=...,baseline_gain_db=...,baseline_margin_db=...,trend=...
DIP_CHECK,stage=...,probe=...,powered_ms=...,reference_dbm1=...,reference_dbm2=...,dbm1=...,dbm2=...,recovered=...
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
`DIP` messages identify reference updates, freezing, probes and recovery.
`DIP_CHECK` records the stopped recheck (`probe=0`) or probe number 1/2, its
actual powered position, frozen reference and both new batch averages. These
checks have their own saved records and do not consume extra line-profile slots.
Sampling pauses do not add powered travel.

The BLE close-range check uses **-58 dBm**, a stronger trial requirement after
the v11 run qualified too early at -66 dBm. Change `AP1_CLOSE_RSSI_DBM`
in `payload_client.h` or override it at compile time after measuring the mounted
boards at the desired stopping distance.
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
the run. The two dip probes therefore request 200 ms in total and allow at most
250 ms of measured powered travel including these two overrun allowances.
Reserve one block and its possible return at a time, rather than
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

BLE connection, service/characteristic discovery and notification setup have
15-second phase timeouts. Other transfer phases retain five-second timeouts,
each complete attempt remains bounded to 30 seconds before cleanup, and cleanup
remains bounded to three seconds. IMU I2C transactions retain their 5000 microsecond timeout. These bounds
are not a measured 20 ms hardware STOP guarantee. Measure the assembled robot's
STOP latency and stopping distance, particularly before enabling moving Wi-Fi.

The movement is timed, not measured in centimetres. Reversing for the same
duration only approximates the original position. RSSI varies with orientation
and reflections, so this programme does not guarantee the shortest route or
obstacle avoidance. The -58 dBm threshold is a configurable starting value;
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

The banner is `line-search v16 dip-probes ble-minus58`. The last 512 KiB of the Pico W's
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
COMPLETION BEEPS: GP22, 3 beeps at 2000 Hz, 200 ms on / 150 ms gap
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
Dip checks cover an isolated strong position, a slowly weakening signal, frozen
references, recovery during the recheck or either probe, and persistent decline.
They also check BLE completion, STOP, missing Wi-Fi, faults and budget exhaustion
during the new stages. The v14 regression replays its 145 raw Wi-Fi observations
and 13 forward durations up to the original reversal. It verifies that the new
policy reaches another forward checkpoint. Subsequent recovery and sustained
decline traces are explicitly synthetic; they do not predict the real signal
beyond the point where that recorded run reversed.
A separate test runs the actual flash adapter with SDK-style bounds checks,
covering all 128 erase sectors, first/last pages and protected firmware/Bluetooth
regions.
An ideal path-loss model includes antenna orientation bias. It checks arrival
for a forward target and for side targets reached after a confirmed decline
and perpendicular search, plus a bounded stop for a rear target. These results
assume ideal wheel movement and do not establish hardware navigation reliability.

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
| BLE flags (ready, running, abort, close, scan, verified, listening, service, characteristic, startup failure, reconnect) | on/off | 1 each | `payload_client.c` status word |
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

1. Confirm the `line-search v16 dip-probes ble-minus58` banner and keep GP21 available.
2. Lift the wheels and check motor polarity, gyro heading, turns and STOP.
   Measure STOP latency during driving, turning, delayed scans and an IMU fault.
3. On the test floor, measure forward and reverse travel for 100, 250, 750 and 2400 ms
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

Flash the rebuilt v9 UF2, reset, then press and release GP20. The log should show
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

## Forward continuation and Wi-Fi margin in v8

The 3 November v7 run stopped after 2405 ms of powered forward travel because
every comparison was inconclusive. Its initial -69/-73 dBm pair contributed
uncertainty repeatedly, producing progress margins of 10 to 18 dB. V8 accepts
the later -63/-62 dBm pair: its conservative gain is 6 dB and the revised margin
is 4 dB. The best point is retained at 2264 powered ms.

The initial heading now continues through inconclusive readings. A direction
that has improved also continues through a flat signal after six powered
seconds. Side trials still have a 2400 ms deadline before their first gain.
Three successive confirmed weak positions still trigger a return and the two
perpendicular checks. No travel or time budget has been increased: each move
still reserves possible return travel within the 60000 ms combined budget,
and the 15-minute run limit, GP21 STOP and fault checks remain active.

Host checks replay all 115 raw Wi-Fi readings and ten measured powered
durations from that v7 run, then simulate BLE completion at the next step.
They also check flat travel and a plateau beyond both former stopping points,
travel reservation exhaustion, the wall-clock limit with realistic radio
delays, and STOP/IMU failure after prolonged forward travel. Synthetic BLE
completion is a control-flow check, not evidence of physical arrival.

V8 retained the -50 dBm BLE threshold. That change cannot guarantee arrival or a
handshake if that threshold is not reached. Measure actual step distance and
BLE at the intended arrival point before the next supervised floor run.

## Calibrated BLE entry, stationary retries and completion beeps in v9

The saved v8 run connected on forward step 18, then timed out in service
discovery. Its next two attempts restarted the -50 dBm proximity check; readings
of roughly -61 to -56 dBm prevented reconnection even though the robot had not
moved. No verified payload was received, so the success sound did not run.

The user reported two extra forward steps after reaching AP1 and authorised
calibration. Taking step 16 as that stopping point, its filtered BLE readings
were consistently -63 dBm. Step 15 had only one qualifying average at -63 dBm.
The v9 -63 dBm entry threshold retained the 12-packet trimmed average and three
consecutive qualifying averages. Replaying the logged filtered values qualifies
step 16 first. This is one-run calibration, not a measured distance guarantee;
raw BLE advertisements were not recorded, so the filter cannot be fully replayed.

After qualification the same BLE address is retained for up to three stationary
connection/transfer attempts. A retry never repeats proximity scanning. Setup
has a longer bounded phase window, and automatic MTU negotiation is disabled
because all protocol values already fit the default 20-byte value capacity.
These changes remove an unnecessary setup exchange and allow delayed service
responses; the log does not establish the original radio-side timeout cause.
GATT results and timeouts now identify the phase and status, and failed payload
attempts retain `FAULT_PAYLOAD` until verified success. Exhaustion also sets that
fault. The robot remains stopped on failure and does not play success beeps.

Host checks cover the calibrated gate, a discovery timeout followed by direct
reconnection to the same address without advertisements, a delayed discovery
response, full discovery/notification setup and payload exchange, CRC and final
acknowledgement failures, stopping at step 16 through a retry, and three success
beeps interrupted by GP21. The UF2 still needs a physical transfer test. AP1's
simultaneous serial log will be needed if service discovery continues to fail.

## BLE threshold revision in v11

The v10 run received AP1 advertisements but never confirmed the -63 dBm gate.
Step 14 recorded twenty filtered readings at -66 dBm; step 17 reached -63 dBm
only once, then reset its confirmation count. The user reported contact before
pressing GP21. V11 uses -66 dBm and retains the 12-reading trimmed filter and
three consecutive qualifying averages. Connection still waits for scanning to
stop, and success still requires the verified payload and final acknowledgement.
Host checks cover the new threshold, all three confirmations and resetting the
count when a filtered reading falls below the threshold.

## Stronger BLE requirement in v12

The v11 run completed the handshake, verified the 26-byte reply and received
the final acknowledgement after one 252 ms forward step. Three filtered
readings at -66 dBm qualified, but the user reported that the robot was still
too far away. V12 requires -55 dBm, which is 11 dB stronger, with the same
12-reading filter and three consecutive confirmations. The earlier -66 dBm
readings now fail the host test. This setting needs a physical stopping-distance
check: previous runs produced different BLE readings near contact, and the log
does not establish which signal level corresponds to the desired distance.

V13 uses the user's selected -60 dBm threshold. The 12-reading filter and three
consecutive confirmations remain in place.

V14 changes the threshold to the user's selected -57 dBm, retaining the same
12-reading filter and three consecutive confirmations.

V15 uses the user's selected -58 dBm threshold with the same filter and
confirmations. The Wi-Fi navigation and three-position reversal rule are unchanged.

## Wi-Fi dip handling in v16

V16 keeps BLE at -58 dBm. It separates the best return point from a median
reference over three positions, freezes that reference during a suspected dip,
and adds a stopped recheck followed by at most two 100 ms forward probes before
returning. A loss exactly at the decline margin no longer counts as a weak point.

The recorded v14 run retained -50/-50 dBm at 2517 powered ms, then measured
-55/-55, -57/-57 and -59/-59 dBm. V14 reversed immediately after those three
positions. With the same recorded input, v16 retains a median reference of
-52/-52 dBm and reaches another forward checkpoint before a third qualifying
decline. The log does not tell us whether the signal would recover beyond that
point. Host tests therefore use separate, explicitly synthetic continuations
to check recovery during probing and a bounded return when recovery fails.

The earlier successful v13 run started with -58/-60 dBm baseline batches and
qualified for BLE at -60 dBm. V14 started at -72/-73 dBm and required -57 dBm
for BLE. Those runs did not present the same inputs or proximity requirement
to the navigation code. Neither log establishes physical distance from RSSI.

## Recorded v16 hardware result

The [run supplied on 3 November 2026](../docs/runs/2026-11-03-v16-backtrack.md)
completed dip checks, a 9032 ms measured backtrack and a clockwise perpendicular
approach. Three -57 dBm BLE averages passed the unchanged -58 dBm requirement;
the reply CRC and final acknowledgement were verified. The run ended with
`PAYLOAD CONFIRMED` and fault mask `0x000`. The linked record contains the full
saved log and the UF2 checksum. This is one successful hardware run, not a
repeatability result or a measurement of physical return accuracy.
