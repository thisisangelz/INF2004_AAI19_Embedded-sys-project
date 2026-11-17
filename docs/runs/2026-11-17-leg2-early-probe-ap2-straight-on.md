# Leg 2 with the three fixes: early probe, then AP2 straight on (17 November, run 1)

Log: [2026-11-17-leg2-early-probe-ap2-straight-on.txt](2026-11-17-leg2-early-probe-ap2-straight-on.txt).
Firmware: `robot_leg2` (`getting_there_leg2.uf2`), branch `feature/ap1-to-ap2-route` at 09527ae, banner
`... side-probe early-probe mat-steer ... route | AP1 -> AP2 (test: start on the first mat)`. Same set-up as
[the failed run](2026-11-17-leg2-fail-turn-fault.md): start on AP1's ring 4 facing AP1, AP2 two tiles away.
A little over 7 minutes (last Wi-Fi sample at 430 s).

| Phase | What the log shows |
|---|---|
| Leaving the pink mat | Off after 2015 ms. First AP2 sample −71 dBm. |
| Three-direction probe | +180: −75/−75, +90: −73/−72, −90: −73/−73 dBm. Followed +90 (heading 90), "strongest, not by 3 dB". One `IMU IO ERROR` during the +180 drive, recovered at once (7.6 ms gap). |
| Line 1, heading 90 | One very weak stop (−86/−87, one scan missed), then −68 for three stops, a dip to −74, then −61/−62 at 3521 ms (+8 dB). −63, −63: **early probe fired** after two stops with no new best. Back at the best point: −61/−61. |
| Side probe | −90 side (heading 0): −73/−72; +90 side (heading 180): −63/−64. Tried 180 first. |
| Line 2, heading 180 | Flat around −61 to −64, then −76. Confirmed decline, drove back 3914 ms (−61/−62). |
| Line 3, heading 0 | −64 down to −75, no gain after 2906 ms. Drove back. |
| Line 4, heading 90 (straight on) | −64, −61, −62, −63, then the blue white edge at 2518 ms. |
| Mat | One scan: ring 1 at 60° and 90°, aimed 65° (heading 155). Steps: ring 3, 3, 3, 4, 4, 5. |
| Arrival | Ring 5, BLE −58 dBm, AP2 payload verified first try. `RESULT: ROUTE COMPLETE, AP2 payload confirmed`, fault mask `0x000`. |

## What it showed

1. **All three fixes ran without trouble.** Every turn was within 1 degree; no `TURN FAILED`.
2. **The early probe fired too soon here.** AP2 was further along line 1: line 4 (the same heading, from
   the same best point) reached the mat 2518 ms later. Two wasted side lines cost about 3 minutes.
3. **"Then straight on" saved the run.** Without it the robot would have tried straight back.
