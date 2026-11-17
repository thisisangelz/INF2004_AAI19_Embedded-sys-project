# Full route AP1 then AP2: AP1 at 12 o'clock, AP2 at 6 o'clock (17 November, route run 3)

Log: [2026-11-17-09-route-complete-ap1-12oclock-ap2-6oclock.txt](2026-11-17-09-route-complete-ap1-12oclock-ap2-6oclock.txt).
Firmware: `robot_route` (`getting_there_route.uf2`), branch `feature/ap1-to-ap2-route` at 09527ae. From the
robot's start: AP1 straight ahead (12 o'clock), AP2 behind it (6 o'clock). 7.23 minutes for both beacons
(leg 2 began at 72 s).

| Phase | What the log shows |
|---|---|
| Leg 1, heading 0 | −65 at the start, −64, −64, then pink ring 1 at 1510 ms. |
| AP1 | One scan, aimed heading 1. Steps: ring 2, 2, 3, 3, 3, 4. BLE −68 dBm, AP1 payload verified. About 1 minute. |
| Leaving the pink mat | Reversed on heading 4: ring 3, ring 2, ring 1, white, floor, floor, then 500 ms more. Off after 2015 ms. |
| Three-direction probe | +180: −68/−68, +90: −69/−70, −90: −68/−68 dBm. Followed +180 (heading 184), "strongest, not by 3 dB". |
| Line 2, heading 184 | −74 at the start, −68 (+5 dB), dip to −80, then −64/−64 at 2012 ms (+9 dB). −65, −66: **early probe fired**. Back at the best point: −67/−66. |
| Side probe | −90 side (heading 94): −75/−75; +90 side (heading 274): −67/−67. Tried 274 first. |
| Line 3, heading 274 | −76 to −71, no gain after 2402 ms. Drove back. |
| Line 4, heading 94 | −70 to −66, no gain after 2403 ms. Drove back. |
| Line 5, heading 184 (straight on) | −75 at the start, −67, −64, −62, −64, −58/−57, −62, then the blue white edge at 3521 ms. |
| AP2 | One scan: ring 1 at 300° and 330°, aimed heading 141. Steps: ring 2, 4, 4, 4, 5. BLE −58 dBm, payload verified first try. |
| End | `RESULT: ROUTE COMPLETE, AP1 and AP2 payloads confirmed`, fault mask `0x000`. |

## What it showed

1. **AP2 behind the robot is found.** The +180 probe tied with −90 but won, and that was the right way.
2. **The early probe was too soon again.** AP2 was 3.5 s further along heading 184. Both side lines failed,
   then "straight on" reached the mat. The two side lines cost about 2.5 minutes.
3. **Three lines in the saved text are cut short** (for example `ollow the strongest; then as before`).
   The run itself was not affected.
