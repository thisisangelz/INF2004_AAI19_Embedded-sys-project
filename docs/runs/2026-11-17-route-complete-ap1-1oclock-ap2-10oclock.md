# Full route AP1 then AP2: AP1 at 1 o'clock, AP2 at 10 o'clock (17 November, route run 1)

Log: [2026-11-17-route-complete-ap1-1oclock-ap2-10oclock.txt](2026-11-17-route-complete-ap1-1oclock-ap2-10oclock.txt).
Firmware: `robot_route` (`getting_there_route.uf2`), branch `feature/ap1-to-ap2-route` at 09527ae, banner
`... side-probe early-probe mat-steer ... route | AP1 -> AP2`. From the robot's start: AP1 at about 1 o'clock,
AP2 at about 10 o'clock. 10.49 minutes for both beacons (leg 2 began at 340 s).

| Phase | What the log shows |
|---|---|
| Leg 1, line 1 (heading 0) | −62/−61 at the start, then down to −73. Dip check, no recovery; drove back 3021 ms (−61/−61). |
| Leg 1, side probe | −90 side (heading 270): −56/−57; +90 side (heading 90): −79/−79. Tried 270 first. |
| Leg 1, line 2 (heading 270) | Rose 4 dB to −56/−56, flat around −58 to −61, then −67. Only 4 dB, so no early probe; decline confirmed, drove back 5535 ms (−59/−59). |
| Leg 1, side probe | −90 side (heading 180): −65/−66. The +90 side (heading 0) met the pink white edge at 1507 ms. |
| AP1 | One scan, aimed heading 315. Steps: ring 3, 3, 4. BLE −60 dBm, AP1 payload verified, three beeps. |
| Leaving the pink mat | Reversed on heading 316: ring 2, ring 1, white, floor, floor, then 500 ms more. Off after 1763 ms. |
| Three-direction probe | +180: −70/−69, +90: −63/−63, −90: −75/−74 dBm. Followed +90 (heading 46), "clearly strongest". |
| Leg 2, heading 46 | Noisy for 6 s (−66 to −81, one cleared decline), then −63/−63 at 6037 ms, best −61/−62 at 6540 ms. −64, −65: **early probe fired**. Back at the best point: −59/−60. |
| Leg 2, side probe | −90 side (heading 316): −76/−76; +90 side (heading 136): −55/−56. Tried 136 first. |
| Leg 2, heading 136 | −66, −65, −62, −57, then the blue white edge at 2516 ms. |
| AP2 | One scan: ring 2 at 60°, heading 196. Steps: ring 3, 3, 4, 4, 5. BLE −57 dBm, payload verified first try. |
| End | `RESULT: ROUTE COMPLETE, AP1 and AP2 payloads confirmed`, fault mask `0x000`. |

## What it showed

1. **The whole route works from one flash.** Both payloads verified, no faults.
2. **The early probe saved time on leg 2.** It turned toward AP2 after two flat stops; the stronger side led
   to the mat in 2.5 s of driving.
3. **Leg 1 took longest** (about 5.5 minutes): two lines declined before the side probe found the mat. AP1
   was off to one side of the start heading.
