# First cruise run: reached blue ring 4, stopped one ring short of arrival (10 November)

Log: [2026-11-10-cruise-blue-mat-box-stop.txt](2026-11-10-cruise-blue-mat-box-stop.txt).
Firmware: `route2_cruise` (cruise on), target AP1 on the blue mat, `floor_step_ms=750`, `minimum_improvement_db=2`.

The robot drove 4.6 s on the floor, found the blue mat, steered by colour scan to ring 4, and stopped there.
It never reached the arrival ring, so no BLE transfer was attempted.

| Phase | What the log shows |
|---|---|
| Baseline | 3 stationary batches of 7 scans at −78 dBm. |
| Cruise 1 | 2400 ms driven in one segment, `end=length`, 4 scans taken while moving (−77 to −69 dBm). The stop that followed measured −68 dBm (+10 dB), a confirmed new best. |
| Cruise 2 | `end=colour` after 2206 ms: the sensor read the white edge of the blue mat. Signal rose to −59 dBm. |
| Mat | Stationary check at the white edge, then at ring 2. Colour steering took over. One 12-step scan pointed 0° (straight on). |
| Steering | Steps of 250 ms then 100 ms through rings 3 and 4 (`depth=5`). Three flat steps on ring 4 (`previous=5`). |
| End | `MAT: three small steps one ring from arrival read no darker; the robot is against the beacon box`. `RESULT: ROUTE STOPPED`, fault `COLOUR_SENSOR` (`0x8000`). |

## What it showed

1. **Cruise worked on the robot.** Largest control-loop gap `max_gap_ms=9` with `late_updates=0`
   on both segments, so Wi-Fi scanning did not disturb the 5 ms heading loop. Heading error stayed
   within ±2° while moving. The first cruise segment gave real moving evidence (4 scans) that was
   followed by a confirming stationary pair, as designed. The robot drove 4.6 s in the first 43 s, before the mat edge.
   The stationary build on the earlier 10 November route run drove 2.0 s in its first 45 s.
   The course and beacon colour differ, so the comparison is only indicative.
2. **The run ended at ring 4, and the blue arrival ring is ring 5.** On pink, arrival is ring 4.
   `depth=5` is ring 4 (floor 0, white 1, ring 1 is 2). The blue mat needs `depth=6`. The robot
   read ring 4 on four consecutive steps (`c` fell from 2924 to 2190 and `br` stayed 2012 to 2119),
   so the classifier never reported ring 5.
3. **The stop rule fired on a reading that was still changing.** The brightness `c` kept falling
   (2924, 2828, 2377, 2190) while the ring label stayed `RING4`. The flat-step rule compares ring
   depth only, so it read "no darker" while the sensor was clearly getting darker within the ring.
   Whether ring 5 was one step further in, or the inner rings read lighter than calibrated as
   on 7 November, cannot be told from this log.
4. **The fault label is misleading.** `FAULT: COLOUR_SENSOR` is raised because the route build
   refuses an inferred centre, not because the sensor failed. The sensor read normally throughout.
