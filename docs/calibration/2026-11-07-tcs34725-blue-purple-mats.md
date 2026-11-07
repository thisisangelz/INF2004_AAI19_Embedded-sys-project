# TCS34725 calibration: blue and purple mats, 7 November 2026

Measurements of the robot's front colour sensor on the blue and purple beacon
mats, taken with `tcs34725_calib6` (6 rings, gain setting 1 = 4x, 103 ms
integration). Each mat has seven bands: a white edge and six coloured rings,
darkest at the centre where the beacon box sits. Two rounds per mat at
different spots, 20 stationary readings per surface, robot on its wheels.

`C` is overall brightness; `G/R` and `B/R` are colour ratios x1000. Raw serial
logs: [raw/](raw/) (`2026-11-07-blue-round1.txt` ... `purple-round2.txt`).

| Round | "Other" surface |
|---|---|
| 1 (both mats) | Shadow of the person calibrating, on the floor |
| 2 (both mats) | Shadow of an object, on the floor |

## Blue mat

| Surface | C (both rounds) | G/R | B/R |
|---|---|---|---|
| Floor | 3866-3923 | 950-959 | 805-814 |
| Shadows on floor | 3585-3724 | 947-950 | 812-814 |
| White edge | 13932-14173 (round 1: 44032, saturated) | 1202-1244 | 1182-1219 |
| Ring 1 | 8418-8612 | 1264-1268 | 1304-1321 |
| Ring 2 | 7053-7114 | 1363-1367 | 1514-1539 |
| Ring 3 | 5440-6653 | 1476-1496 | 1782-1835 |
| Ring 4 | 4111-4687 | 1607-1635 | 2151-2220 |
| Ring 5 | 3261-3433 | 1622-1642 | 2345-2386 |
| Ring 6 (centre) | 2577-2863 | 1541-1588 | 2354-2422 |

## Purple mat

| Surface | C (both rounds) | G/R | B/R |
|---|---|---|---|
| Floor | 3517-3999 | 947-948 | 779-795 |
| Shadows on floor | 2944-3218 | 935-976 | 779-842 |
| White edge | 13004-15846 | 1107-1124 | 1082-1107 |
| Ring 1 | 7713-8371 | 1081-1086 | 1111-1131 |
| Ring 2 | 6304-6786 | 1049-1052 | 1171-1191 |
| Ring 3 | 5157-5271 | 1011-1018 | 1243-1253 |
| Ring 4 | 3181-3387 | 955-965 | 1307-1311 |
| Ring 5 | 2319-2427 | 932-964 | 1395-1423 |
| Ring 6 (centre) | 2024-2139 | 947-976 | 1565-1585 |

## Findings

1. **B/R is the ring cue on both mats.** It rises at every step inward, from
   the white edge to the centre, while pink rings lower G/R instead. B/R does
   not depend on brightness, so the 30-60% brightness change seen on the pink
   mat on 7 November should not move these thresholds.
2. **Blue rings 5 and 6 overlap in B/R** (2345-2386 against 2354-2422). G/R
   separates them (ring 5 at least 1622, ring 6 at most 1588). Both are past
   any likely arrival ring, so this only matters for steering depth.
3. **Purple white edge and ring 1 are only 4 apart in B/R** across rounds
   (1107 against 1111). G/R separates them (white at least 1107, ring 1 at most
   1086). Both are "lighter mat" for the arrival rule, so a mix-up there cannot
   cause a false arrival.
4. **The floor is far from both mats.** Floor and floor shadows reach B/R 842
   at most; the lightest mat surface is 1082 (purple white). Floor readings
   were the same as on the pink-mat day (G/R 943-959, B/R 774-814).
5. **The white edges are not the same white.** Blue-mat white reads B/R
   1182-1219, purple 1082-1107, pink 943-958. Each mat needs its own white
   threshold, so the robot should use the profile of the beacon it is heading
   for.
6. **Round 1 blue white edge saturated** (`C` 44032 = full scale) at one spot,
   probably a glossy patch. The ratios were still in line with round 2, and the
   classification below uses ratios only.
7. **Shadows read as floor.** All four shadows had floor ratios, as on the pink
   mat.

The pink captures from 6 November (also in [raw/](raw/)) include B/R that the
[pink calibration record](2026-11-06-tcs34725-mat.md) did not list: floor
747-768, floor shadows 771-781, ring 1 898-906, white edge 943-958. They confirm
the B/R >= 842 floor check proposed for the pink mat after the 7 November
side-on run.

## Proposed classification

Midpoints between the pooled ranges of both rounds. Ratios only, no brightness.

**Blue mat**

| B/R x1000 | Surface |
|---|---|
| below 998 | Floor |
| 998-1261 | White edge |
| 1262-1417 | Ring 1 |
| 1418-1660 | Ring 2 |
| 1661-1992 | Ring 3 |
| 1993-2282 | Ring 4 |
| 2283 and above | Ring 6 if G/R < 1605, otherwise ring 5 |

Smallest margins: 42 (white edge against ring 1), 62 (ring 4 against ring 5),
17 in G/R (ring 5 against ring 6).

**Purple mat**

| B/R x1000 | Surface |
|---|---|
| below 962 | Floor |
| 962-1150 | White edge if G/R >= 1097, otherwise ring 1 |
| 1151-1216 | Ring 2 |
| 1217-1279 | Ring 3 |
| 1280-1352 | Ring 4 |
| 1353-1493 | Ring 5 |
| 1494 and above | Ring 6 |

Smallest margins: 10 in G/R (white edge against ring 1), 20 in B/R (ring 1
against ring 2), 26-27 (rings 2-4). Tighter than the blue mat; a third round
at another spot would confirm them.

## Arrival ring (to decide)

The pink mat arrives on ring 4 of 5: one ring short of the darkest, so the
robot's front stays clear of the beacon box. The same position on a 6-ring mat
is **ring 5 or darker**: blue B/R >= 2283, purple B/R >= 1353. Ring 4 or darker
would be one ring earlier (blue 1993, purple 1280), with larger margins.

## Still to check

- Which beacon each mat belongs to (blue AP2, purple AP3 assumed).
- Whether the mats are the same size as the pink mat (ring widths affect how
  far the robot stops from the box).
- The demo room's lighting.
