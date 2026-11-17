# Full route AP1 then AP2: AP1 at 12 o'clock, AP2 at 4 o'clock (17 November, route run 2)

Log: [2026-11-17-08-route-complete-ap1-12oclock-ap2-4oclock.txt](2026-11-17-08-route-complete-ap1-12oclock-ap2-4oclock.txt).
Firmware: `robot_route` (`getting_there_route.uf2`), branch `feature/ap1-to-ap2-route` at 09527ae. From the
robot's start: AP1 straight ahead (12 o'clock), AP2 at about 4 o'clock. 7.23 minutes for both beacons
(leg 2 began at 111 s).

| Phase | What the log shows |
|---|---|
| Leg 1, heading 0 | −74 at the start, −72, −72, dip to −79, then −69/−68 (+5 dB), then the pink white edge at 2516 ms. |
| AP1 | Scan 1 saw only the white edge; Wi-Fi resumed at heading 332 (−58/−58). One 250 ms step reached ring 1; scan 2 aimed heading 347. Steps: ring 2, 2, 3, 3, 4. BLE −60 dBm, AP1 payload verified. |
| Leaving the pink mat | Reversed on heading 349: ring 2, ring 1, white, white, floor, floor, then 500 ms more. Off after 2015 ms. The second "white" (`c` 4824) was most likely light floor. |
| Three-direction probe | +180: −68/−67, +90: −67/−68, −90: −64/−64 dBm. Followed −90 (heading 259), "clearly strongest". |
| Line 3, heading 259 | −76 at the start (facing AP2), then −68, −65, −62, −58/−59 at 3018 ms (+16 dB). −58, −59: **early probe fired**. Back at the best point: −60/−60. |
| Side probe 1 | −90 side (heading 169): −67/−67; +90 side (heading 349): −73/−72. Tried 169 first. |
| Line 4, heading 169 | −71 (+8 dB), then −75, −78: **early probe fired again**. Back: −69/−70. |
| Side probe 2 | −90 side (heading 79): −71/−73; +90 side (heading 259): −67/−65. Tried 259 first. |
| Line 5, heading 259 | −59, −56, −56, −53/−53, −59, then the blue white edge at 2516 ms. |
| AP2 | One scan, aimed heading 217. Steps: ring 2, 3, 4, 4, 4, 4. Three small steps on ring 4 read no darker: against the box, treated as arrived. BLE −62 dBm. First connection failed (`status 0x1f`), the retry transferred the payload. |
| End | `RESULT: ROUTE COMPLETE, AP1 and AP2 payloads confirmed`, fault mask `0x000`. |

## What it showed

1. **The whole route works from one flash again**, with no faults.
2. **The early probe on line 3 was too soon.** Line 5 went the same way (heading 259) and reached the mat;
   AP2 was still ahead of line 3's best point. The detour along heading 169 cost about 2.5 minutes.
3. **The "against the box" rule still matters**: AP2 transferred from ring 4.
