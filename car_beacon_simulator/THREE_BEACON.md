# Three-beacon firmware and simulator

The firmware branch is `codex/three-beacon-route`. The simulator branch is
`codex/three-beacon-simulator`. Keep them separate; neither is a merge into
`main`. The simulator can also select `origin/main` to inspect its existing
single-target behaviour under the same environment.

## What the robot can observe

The robot receives Wi-Fi SSIDs, BSSIDs and integer RSSI, BLE advertisements and
protocol responses, a relative gyro heading, and the front colour sensor's raw
red, green, blue and clear channels. Coordinates, distance to each beacon, mat
geometry and collision status are simulator diagnostics. Navigation never
receives those diagnostics.

Turning an ordinary omnidirectional radio is not a reliable bearing measurement.
The search therefore compares fresh signal batches before and after actual
movement. An inconclusive trial has a limit and returns before trying another
heading. A promising direction must keep producing evidence of progress; one
early improvement cannot justify an indefinitely flat heading. Existing
spread-aware comparisons and the colour-mat approach remain part of the actual
firmware rather than a controller implemented in the simulator.

## Route contract

1. Approach AP1 on its pink mat and complete the actual file-transfer protocol.
2. Record AP1's verified reply and final acknowledgement. Complete radio cleanup.
3. Depart opposite the observed arrival movement while the previous mat is still
   the active colour profile. A validated inward colour-scan heading can also
   supply this direction. Establish floor before switching identity. This is a
   bounded timed departure with colour checks, not obstacle detection or precise
   wheel odometry.
4. Reset target-local Wi-Fi identity, BLE qualification, colour history and search
   state. Retain the gyro reference, mission clock, log and latched STOP.
5. Repeat for AP2 on blue, then AP3 on purple.

A route is successful only when the firmware reports all three verified
transfers and emits `RESULT: ROUTE COMPLETE`. Connecting to a beacon, touching a
mat, a beacon-side completion notification, and completing only AP1 are not
overall success. A missing beacon, failed CRC, invalid acknowledgement, unsafe
handover or exhausted search remains a visible failure.

## Set up a comparison

Select the exact firmware source and robot build separately. The `route` build
requires a source that actually implements the route; selecting it for an older
commit must report an incompatibility instead of supplying a replacement
controller. Per-target `ap1`, `ap2` and `ap3` builds remain useful for comparison.

Arrange the physical beacons and mats, set the robot's initial position and
heading, then explicitly press **Build & run experiment**. The rearward and
sideways presets change starting conditions, not firmware rules. Each run starts
a new native process. Playback, history and imports inspect recordings without
executing firmware again. Full traces retain all beacon states and source
identity; legacy single-beacon recordings remain replayable.

Keep beacon identity separate from mat colour. AP1's actual advertisement is
still AP1 if it is placed on a blue mat. Removing AP2 or AP3 must not create a
stand-in radio or award that leg of the route.

## Physical assumptions to measure

Model v3 retains the older AP1-referenced scalar fields for trace compatibility
(`metadata.scalar_reference_id = 1`). Use the per-beacon records and active target
when analysing a route. The interface calculates target distance and closest
approach from the recorded positions; those calculations never reach navigation.

The initial model uses a 0.35 m mat radius, a sensor 0.08 m ahead of the wheel
axle, a 0.07 m circular beacon box, 0.76 m/s full-duty wheel speed and a 0.14 m
wheelbase. The user reports about 8 cm between the sensor and the rear axle,
and about 3 cm ahead of the front axle. Confirm which axle is driven before
choosing the offset: the model measures from the effective differential-drive
axle, not from the body edge. The other dimensions remain editable assumptions.
Concentric bands use representative readings from the repository's recorded
colour calibrations. Actual mat dimensions, uneven bands, sensor footprint,
mounting height, lighting, wheel slip and antenna effects can change the result.

The Wi-Fi reference remains -50 dBm at 1 m and the BLE reference remains -50 dBm
at the configured reference radius. A firmware threshold change does not
recalibrate those signals. The adjustable radio visibility distance is an
illustrative range cutoff; the reported three-chair/four-chair observations
have not been converted into metres or used to tune the navigation.

The model must expose failed progress and box contact instead of steering
around the box for the firmware. It does not provide general obstacle avoidance.
An ideal-model success is not proof of hardware success or indoor RF robustness.

## Verification without executing experiments

Validation for this change: 32 mocked interface tests and 11 independent adapter
tests passed. Current main and the committed route firmware compiled/linked in
the native harness. All five hardware firmware targets and all 12 firmware host
fixtures compiled/linked; the firmware fixtures and experiments were not run.

Build/link checks and mocked interface tests are permitted without starting a
server or firmware run. Firmware-based navigation, BLE, route and integration
tests execute controller code and require an explicit request before running;
see [AGENTS.md](AGENTS.md). The normal UI never runs an experiment on page load,
setup changes, branch selection or trace import.

Before claiming hardware readiness, test the unchanged physical arrangement
with forward, 90-degree and 180-degree starts; then test verified departure and
identity changes between two beacons before the full three-beacon route. Record
the firmware commit, complete log, STOP response, each CRC/final acknowledgement
and remaining budgets. Keep failed traces as evidence rather than changing the
environment to make them pass.
