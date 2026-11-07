# Side-on mat entry: transfer on ring 1 (7 November)

Log: [2026-11-07-side-on-mat-ring1-transfer.txt](2026-11-07-side-on-mat-ring1-transfer.txt).
Firmware: v16 with colour-ring arrival and sounds (banner `line-search v16
dip-probes ble-minus58 colour-ring4 cues`), before v18.

The robot started close to AP1, beside the mat and pointing along the rings
rather than at the centre, to see whether it would turn in and reach ring 4.
It drove straight on heading 0 for 12 steps of 250 ms. The colour sensor read
(check 1 is before the first step):

| Checks | Logged zones | Actually |
|---|---|---|
| 1-5 | RING1, RING1, FLOOR/RING1, RING1, FLOOR | Floor: `C` 4622-5012, `G/R` 958-963, `B/R` 806-813 |
| 6-7 | WHITE | White edge |
| 8 | RING1 | Ring 1 (`C` 11385) |
| 9-11 | RING2 | Ring 2 |
| 12-13 | RING1 | Ring 1 again: the line ran along the rings, past the centre |

At check 13 BLE averaged -58 dBm three times and the payload transferred, with
the robot on the border of ring 1 and the white edge. Colour arrival was never
reached (`arrived=0` throughout); the strict BLE gate passed on its own.

## What it showed

1. **BLE -58 dBm is not a close-range test on its own.** It passed about one mat
   radius from AP1. Only the colour sensor knows the robot is on ring 4.
2. **Wi-Fi does not steer across the mat.** Wi-Fi rose from -63 to -44 dBm
   along the line, so the robot kept driving straight across the rings.
3. **Brighter light made the floor read as ring 1** (`C` above the 4660
   threshold). See the [calibration update](../calibration/2026-11-06-tcs34725-mat.md).

## Changes made (v18, branch feature/bigger-steps-and-back-search)

- With a working colour sensor, the transfer happens only after colour arrival
  (ring 4 or darker after a lighter ring); BLE then only needs -85 dBm. Without
  a working sensor the -58 dBm gate still applies.
- Once a stop sees the mat, colour steering replaces the Wi-Fi line: the robot
  scans on the spot, turns toward the estimated centre, and takes 250 ms steps,
  scanning again if the surface gets lighter.
- White edge and ring 1 also need `B/R` >= 842, so this floor is floor.
