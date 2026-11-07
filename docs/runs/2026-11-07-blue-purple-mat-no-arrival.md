# Blue and purple mats: no arrival, robot into the beacon box (7 November)

Firmware: `line-search v18 ... multi-mat`, AP1 build with 100 ms steps near
the centre. One beacon (AP1), moved between mats. Pink worked as before.
Purple log: [2026-11-07-purple-mat-no-arrival.txt](2026-11-07-purple-mat-no-arrival.txt).
The blue log was shared in chat; its key lines are quoted below.

## What happened

Both runs reached the mat, steered by colour scans and took 100 ms steps near
the centre as intended (`MAT_STEP ... ms=100`). Neither ever logged
`arrived=1`, so the robot kept inching inward until it reached the box.

| Mat | Arrival needs (ring 5) | Darkest reading in the run | Logged as |
|---|---|---|---|
| Blue | B/R >= 2283 | B/R 2085 (C 1708) | RING4 |
| Purple | B/R >= 1353 | B/R 1330 (C 1242) | RING4 |

- **Blue:** after reaching "RING4" the robot took seven more 100 ms steps on
  the same reading with the heading error growing (+1 to +4 degrees), the sign
  of pushing against the box. Its next scan read ring 3 in all twelve
  directions (B/R 1843-1935), which the code treated as "no ring to steer
  toward", so it resumed the Wi-Fi line search and drove on. GP21 stopped it.
- **Purple:** the reading moved between RING3 and RING4 for several steps; GP21
  stopped the run before the box. The purple white edge (B/R 989-1032) was
  also classed as pink white.

## Why

The sensor received about 40-50% of the light it did during the
[blue/purple calibration](../calibration/2026-11-07-tcs34725-blue-purple-mats.md):
blue white edge C 5700-7700 against 14000, purple white 7000 against
13000-15800, floor 2200-2900 against 3400-4000. The outer rings still read
close to their calibrated ratios, but the dark inner rings read about one ring
lighter, which is what an extra constant light offset does to dim readings.
Possible causes, not yet confirmed: the beacon box shading the centre (if it
was not on the mat during calibration) or different room lighting.

## Changes (feature/colour-ring-sensing)

Two stop rules so the robot cannot drive into the box even when the inner
rings read lighter than calibrated. Either one ends all movement: the robot
transfers with the loose BLE gate, retries in place if AP1 is not in range,
then stops with a payload fault.

- **Pushing against the box:** within one ring of arrival, three 100 ms steps
  with no darker reading.
- **Surrounded by inner rings:** a scan where every direction is within two
  rings of arrival. Also used if the colour scans run out that close to the
  centre. It never hands back to Wi-Fi driving there.

In the host model with the inner rings reading one ring lighter and a solid
box, the robot now stops after two or three blocked steps from every start;
with normal readings it still arrives by colour and the stop rules never fire.

## Still to do

Recalibrate the blue and purple inner rings in run conditions: the beacon box
on the mat, the robot on its wheels facing inward, normal room lighting, nobody
leaning over. Then the arrival thresholds can match what the robot actually
sees, and the stop rules stay as a backstop.
