# Side probe finds the mat on the left, transfer on ring 4 (17 November)

Log: [2026-11-17-01-side-probe-left-mat-transfer.txt](2026-11-17-01-side-probe-left-mat-transfer.txt).
Firmware banner: `line-search v18 steps500-mat250 back-search side-probe mat-steer colour-gate own-mat ble-minus58 cues`,
AP1, pink mat. Branch `feature/side-probe-direction-search` (commit 2b081ef). Saved run replayed with GP21.

The robot started about 3 tiles from AP1. It faced along the side of the mat rather than at it, so a straight line
would pass the mat without touching it. This was the first full run with the side probe, after re-seating a loose
IMU cable. The previous attempt with the same firmware stopped with `FAULT: IMU` at the start of the first side probe.

| Phase | What the log shows |
|---|---|
| Start, heading 1 | −76/−77 dBm. Every `COLOUR` check on the floor read `FLOOR` (`c` 1180-1700). |
| Line 1 | 500 ms steps. Signal rose to a best of −55/−55 dBm at 5526 powered ms, then fell to −64 by 8037 ms. |
| Dip check | Stopped recheck −63/−63, two 250 ms probes −66/−66 and −65/−64: no recovery. |
| Return | Reversed 3014 ms to the best point; it read −55/−56 there, consistent with the best. |
| Side probe −90 (heading 271) | Drove 2002 ms, turned back to face heading 1, read −65/−64 dBm, 9 dB below the best point. Returned. |
| Side probe +90 (heading 91) | The last drive block ended on the white edge (`c` 12872, `gr` 1035, `br` 1000) at 2001 ms. Colour steering took over. |
| Mat | One 12 × 30° scan: darkest at 60° (ring 2). Turned to heading 150 and took five 100 ms steps: ring 2, ring 3, ring 3, ring 3, ring 4. |
| Arrival | Ring 4 confirmed (`arrived=1`). BLE −62 dBm, 3 of 3 averages above −85. |
| End | Payload transferred, reply CRC verified, `RESULT: PAYLOAD CONFIRMED`, fault mask `0x000`. |

## What it showed

1. **The side probe found the beacon's side.** The best Wi-Fi point was beside the mat, not in front of it. On v18
   without the probe, the first direction tried is −90, which here was the wrong side (−65 against −55 dBm). The
   probe drove toward +90 and reached the white edge at the end of its 2000 ms drive.
2. **The mat was only just within the probe distance.** The white edge appeared on the last block of the +90 probe.
   If the best point had been a little further from the mat, the probe would have measured +90 and compared both
   sides instead of meeting the mat directly.
3. **The IMU was stable after the cable fix.** Five 90° and 180° turns peaked at 153-178 °/s and settled within
   ±1°. No `IMU IO ERROR`. In both earlier runs on 17 November an IMU I2C error followed a fast 90° turn: the
   first recovered, the second stopped the run.
4. **The floor read as floor at this spot.** `c` stayed at 1180-1700 with `gr` 869-946 and `br` 762-850, well below
   the white edge. The earlier run at this location misread a lighter floor patch (`c` about 2600-3600) as pink
   white edge, so that patch still needs a look.
5. **Colour steering went straight to ring 4.** One scan and five 100 ms steps, with no second scan.
