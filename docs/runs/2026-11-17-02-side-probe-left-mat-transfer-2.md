# Side probe repeat: mat on the left again, transfer on ring 4 (17 November, run 2)

Log: [2026-11-17-02-side-probe-left-mat-transfer-2.txt](2026-11-17-02-side-probe-left-mat-transfer-2.txt).
Same firmware and similar start as [run 1](2026-11-17-01-side-probe-left-mat-transfer.md): banner
`line-search v18 steps500-mat250 back-search side-probe mat-steer colour-gate own-mat ble-minus58 cues`, AP1, pink mat,
about 3 tiles from AP1 and facing along the side of the mat. Saved run replayed with GP21. About 4.25 minutes from
start to transfer.

| Phase | What the log shows |
|---|---|
| Start, heading 1 | −81/−82 dBm, weaker than run 1 (−76/−77). 5 of the first 55 scans missed AP1 (`dbm=-127`), none later; every sample still had at least 3 valid scans. |
| Line 1 | 500 ms steps. Best −51/−51 dBm at 6533 powered ms, then down to −65/−65 by 9547 ms. |
| Dip check | Stopped recheck −66/−65, two 250 ms probes −65/−65 and −65/−66: no recovery. |
| Return | Reversed 3518 ms to the best point; it read −54/−53 there, consistent with the best. |
| Side probe −90 (heading 271) | Drove 2002 ms, turned back to face heading 1, read −65/−65 dBm, 12 dB below the best point. Returned. |
| Side probe +90 (heading 91) | The second drive block ended on ring 1 at 1505 ms (`c` 10758). The white edge passed between two checks. |
| Mat | One 12 × 30° scan: ring 2 at 30° and 60°, aimed 51°. Turned to heading 142 and took four 100 ms steps: ring 2, ring 3, ring 3, ring 4. |
| Arrival | Ring 4 confirmed (`arrived=1`). BLE −60 dBm, 3 of 3 averages above −85. |
| End | Payload transferred, reply CRC verified, `RESULT: PAYLOAD CONFIRMED`, fault mask `0x000`. |

## What it showed

1. **Run 1 repeated.** Again the best Wi-Fi point was beside the mat, the −90 side read much weaker (12 dB here,
   9 dB in run 1), and the +90 probe reached the mat. In both runs the mat was on the left, within the 2000 ms
   probe distance.
2. **The mat was closer this time.** The probe met it at 1505 ms instead of 2001 ms, so the best point was a little
   nearer the mat than in run 1.
3. **The colour sensor can miss the white edge.** It went straight from floor to ring 1 within one 750 ms block.
   Mat detection still worked because any ring of the target mat counts, not only the white edge.
4. **The IMU stayed stable.** Five large turns (90°, 180° and an 82° aim) peaked at 134-199 °/s with no
   `IMU IO ERROR`.
5. **Most of the time is still Wi-Fi sampling.** Each stop takes two 5-scan batches of about 3.6 s each. Driving
   and turning are a small share of the 4.25 minutes.
