# Leg 2 with the three fixes: early probe finds AP2's mat on the side (17 November, run 2)

Log: [2026-11-17-leg2-early-probe-ap2-side.txt](2026-11-17-leg2-early-probe-ap2-side.txt).
Firmware: `robot_leg2` (`getting_there_leg2.uf2`), branch `feature/ap1-to-ap2-route` at 09527ae. Same set-up as
[run 1](2026-11-17-leg2-early-probe-ap2-straight-on.md): start on AP1's ring 4 facing AP1, AP2 two tiles away.
About 3.5-4 minutes (last Wi-Fi sample at 205 s).

| Phase | What the log shows |
|---|---|
| Leaving the pink mat | Off after 2013 ms. First AP2 sample −79 dBm. |
| Three-direction probe | +180: −73/−73, +90: −77/−76, −90: −63/−64 dBm. Followed −90 (heading 270), "clearly strongest". |
| Line 1, heading 270 | −71, −70, −71, dip to −77, then rising: −67, −69, −65, −63/−64 at 4026 ms (+7 dB). −63, −67: **early probe fired**. Back at the best point: −63/−62. |
| Side probe | The −90 side (heading 180) met the blue white edge at 2005 ms, the end of the probe drive. |
| Mat | Scan 1 saw only the white edge, so Wi-Fi resumed at heading 152 (−55/−55). The first 250 ms step was on white again; scan 2 found ring 1 at 30° and 60°, aimed heading 197. Steps: ring 2, 3, 4, 4, 5. |
| Arrival | Ring 5, BLE −58 dBm. The first BLE connection failed (`status 0x1f`); the stationary retry transferred the payload. `RESULT: ROUTE COMPLETE`, fault mask `0x000`. |

## What it showed

1. **The early probe worked as intended.** Line 1 passed beside the mat; the side probe reached it straight
   away, without the 1-1.5 minutes of confirming a decline.
2. **A scan that sees only the white edge is handled.** The robot went back to Wi-Fi, stepped once and
   scanned again.
3. **The BLE retry is needed.** The first GATT service discovery was rejected; the retry worked.
