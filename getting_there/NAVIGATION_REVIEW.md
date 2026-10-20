# Alvin → Alvin_2 navigation review

Baseline: `origin/Alvin` at `feb64f7f49f8e42560d29d148261c7b55665780f`.
Scope: the AP1-only robot in `getting_there`, not the separate legacy tracker.
No physical robot was connected or flashed during this review.

## Findings and changes

| Finding in Alvin | Consequence | Alvin_2 |
|---|---|---|
| Chooses the highest absolute endpoint RSSI of four timed probes without requiring improvement | A direction can win even when every direction is worse; orientation changes are mixed with position changes | Same-heading before/after comparison, noise margin, stationary confirmation |
| Four 750 ms steps continue while AP1 is visible, even when RSSI weakens | The robot can keep moving the wrong way | Confirmed weakening rejects the direction after that step; bounded retreat and 90° replan |
| No cumulative test for weak per-step changes | Small steps can disappear in normal RSSI variation | Keep one baseline for up to three 750 ms segments before rejecting an unclear direction |
| Assumes equal forward/reverse time restores the probe origin | Wheel slip, inertia, battery voltage and surface differences invalidate comparisons | Still only an approximate retreat, but always re-baseline at the actual new position/orientation |
| Wi-Fi matches SSID only | Another device with the same SSID can change the target mid-run | Lock to first matching BSSID for the run; initial identification still needs a unique SSID/known AP |
| USB log call while a learning turn is powered; unbounded IMU bus calls | Blocking work can delay motor shutdown | Move log after motor stop; 5 ms timeout per I2C transaction |
| Saved report only in RAM, oldest lines discarded | A power-bank run cannot be read after power-off | Flash journal and GP21-before-GP20 replay; explicit incomplete recovery |
| Root build omits this robot; committed UF2 predates Alvin's packing changes | It is easy to test an old binary despite editing current source | Root includes `getting_there`; rebuild its UF2 and identify by startup banner |

## What the online work suggests

[Parasuraman et al., Spatial sampling methods for improved communication for
wireless relay robots (2013)](https://ramviyas.github.io/pdf/ICCVE2013_Parasuraman.pdf)
uses spatial signal-strength changes and temporal averaging. Its experimental
holonomic platform avoids orientation changes; it is not proof that a turning
Pico with one antenna can identify a compass bearing. We borrow the idea of
measuring a change over movement, not a single loud reading. Our same-heading
comparison, confirmation and thresholds are engineering heuristics, not a
reproduction of the paper or a guarantee of reaching the emitter.

[Pan Wang, A RSSI based Locating Algorithm for Multi-Robot System (2013),
abstract](https://commons.library.stonybrook.edu/stony-brook-theses-and-dissertations-collection/3304/)
describes multiple movements and variable steps to address RSSI errors, evaluated
in simulation. This supports testing a cumulative movement baseline; it does
not establish a suitable step duration for this rover.

[A Single RF Emitter-Based Indoor Navigation Method for Autonomous Service Robots
(2018)](https://www.mdpi.com/1424-8220/18/2/585) uses multiple directional antennas.
That direction-estimation method is not a software-only replacement for the
Pico W's one onboard antenna.

## Are 750 ms steps too short?

Possibly: milliseconds are not distance. Measure how far this exact chassis
travels at 35% duty on the test floor. In an ideal path-loss model with exponent
2, moving from 2.0 m to 1.9 m changes RSSI by only about 0.45 dB. That is an
illustration, not a measured value for this room. Reflections and antenna
orientation can overwhelm such a small change.

Now a weak individual change can accumulate over 2250 ms, but the margin of
at least 3 dB may still be too conservative at longer ranges. Do not increase
travel blindly: record stationary variation and actual distance first. No
encoder or obstacle avoidance has been added, and reverse travel needs clear
space too. Four rejected directions can mean noise, a local maximum, insufficient
travel, wrong motor polarity, or a hardware/radio fault—not necessarily no AP1.

## What the AP1 status does and does not prove

`BLE: ADVERTISING`, no connection, and notifications not enabled are normal
while AP1 waits for a client. They do not prove the robot received qualifying
advertisements. The rover's log now explains when BLE ended with no target
readings, too few readings, or insufficient qualifying close averages.

The -50 dBm BLE threshold is unchanged: 12 samples, trimmed average and three
consecutive qualifying averages in a five-second scan. Nearby averages overlap
and are not independent confirmations. RSSI is not a guaranteed distance.
Arrival still requires payload length/CRC verification and final acknowledgement.

## First physical test

1. Flash `getting_there/build/getting_there.uf2` to the robot and confirm
   `Alvin_2 progress-v2 flash-log`, not another tracker's binary.
2. Lift the wheels and check M1 is left, M2 is right, and both forward commands
   move in the expected physical direction. Keep GP21 available.
3. Check gyro calibration while stationary and a measured 90° turn. Check STOP
   during driving/turning and with a disconnected IMU. Measure PWM-off latency;
   host tests cannot certify a 20 ms real-hardware deadline or stopping distance.
4. Place AP1 alone in a clear, bounded test area. Test starting toward it,
   sideways and away. Clear the whole retreat path. Watch for `PROGRESS`,
   `PROGRESS UNCONFIRMED`, `REPLAN`, and `FAILED` records.
5. After a normal finish/STOP, allow several seconds to save, power-cycle onto
   USB, open serial, press GP21 before GP20, and export the complete log.
   Repeat once after a mid-run power cut and expect an INCOMPLETE prefix.
6. Compare actual 750 ms/2250 ms travel and stationary RSSI spread against
   decisions before changing the movement duration, noise margin or BLE gate.

Host tests and the firmware build pass; physical arrival, range, flash persistence
on the actual board and STOP latency remain to be measured.
