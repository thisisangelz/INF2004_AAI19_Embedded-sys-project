# AP1 to AP2 route: AP1 payload delivered, then AP2 dropped out of the scans (12 November)

Log: [2026-11-12-ap1-ap2-route-ap1-done-ap2-wifi-lost.txt](2026-11-12-ap1-ap2-route-ap1-done-ap2-wifi-lost.txt).
Firmware banner: `bounded eight-heading acquisition, foreign-mat guard, no-new-best watchdog | AP1 -> AP2 route`,
with cruise. Legs: AP1 on the blue mat, then AP2 on the pink mat. `floor_step_ms=750`, `minimum_improvement_db=2`.
Copied from the live serial output, so it has no `SAVED RUN` header.

This is the first saved route run to finish a leg: AP1's payload was transferred and verified
(`ROUTE: confirmed=1,mask=1`). The AP2 leg then stopped itself after 187 s in total with `WIFI_LOST`. GP21 was not
pressed: the fault mask has no `STOP_LATCHED`.

| Phase | What the log shows |
|---|---|
| AP1, heading 0 | −80/−77 dBm at the start. One 2401 ms cruise (`end=length`), moving scans −84 rising to −67. The stop read −71/−70: +6 dB, the first confirmed improvement. |
| AP1, cruise onto the mat | The next cruise ended on colour at 5062 ms on a bright white edge (`c=35733`), −60/−61 dBm (+16 dB on the start). One 250 ms step reached blue ring 1. |
| AP1, on the mat | One 12-point colour scan pointed straight ahead (0°). One 250 ms step to ring 3, then four 100 ms steps through ring 4 to ring 6: `arrived=1` after a lighter ring. |
| AP1, transfer | BLE −57 dBm, 3/3 close readings. Handshake, file, 26-byte reply with CRC `74ea41b4` verified, final `COMPLETE` acknowledged. `translation_ms=6080`. |
| Departure | Reversed at heading 0 in 250 ms blocks. Floor at 1258 ms, clearance reached at 1509 ms. |
| AP2, heading 360 | −78/−81 at the start. The cruise stopped on colour after 467 ms (`WHITE`, labelled `PINK`, `c=17202`). The stop read −81/−81 (−3 dB). The next 250 ms step read a foreign mat (`c=6145`, `gr=1349`, `br=1510`). Reversed 724 ms to the start. |
| AP2, back at the start | One sample had only 4 valid scans of 7 (missing), then −77 and −76. |
| AP2, trial 1/7, heading 270 | −80/−81 at the start. One 2401 ms cruise, then −74/−73: +6 dB, first confirmed improvement. A 6002 ms cruise (`end=length`) carried it to 8403 ms. The stop read −75/−73, 2 dB below the best. The no-new-best watchdog reversed 6002 ms back to 2401 ms. |
| End | Back at 2401 ms, three samples in a row had 3, 2 and 3 valid scans. `FAILED: AP2 missing from three fresh Wi-Fi samples; no blind movement`, `RESULT: ROUTE STOPPED`, `ROUTE: final_mask=1`, fault mask `0x010` (`WIFI_LOST`). |

## What it showed

1. **The full AP1 path works end to end.** Wi-Fi gain on the floor, cruise stopping on the mat edge, one colour
   scan, ring-by-ring steps to the arrival ring, BLE transfer with CRC and final acknowledgement, then a reverse
   departure to the floor. Only 6.1 s of powered travel was needed, and AP1 dropped out of 3 of 57 scans.
2. **The AP2 leg started by driving back towards AP1 (inferred).** The robot reversed off AP1 at heading 0, and
   the AP2 leg then tried heading 360, the same direction. After 467 ms it hit a white edge, and 250 ms later a
   foreign mat. That is about the 1509 ms it had just reversed. The foreign reading (`c=6145`, `gr=1349`,
   `br=1510`) is close to AP1's blue ring 1 (`c=6982`, `gr=1281`, `br=1357`). White edges look the same on every
   mat, so the classifier labelled it `PINK`, the target mat. The foreign-mat guard then turned it back, as
   designed, but the first heading of every later leg will face the beacon just served.
3. **The 6 s cruise overshot the best point again.** As in the heading-fault run earlier today, the moving scans
   at heading 270 stayed inside the decline limit, so the cruise ran its full 6000 ms. The stop was 2 dB below the
   best, and the watchdog spent another 6 s reversing.
4. **At the returned best point, AP2 had faded by about 10 dB.** At 2401 ms on heading 270 the robot first read
   −74/−73, with 1 scan of 14 missing. After the 6 s out and 6 s back, 13 of 21 scans missed and the rest read −82
   to −91. The return is an approximate reverse, so the robot may not be on the same spot. The log cannot show
   whether something else changed, such as a person or object between the robot and AP2.
5. **The 5-of-7 rule ended the run again.** Three samples with fewer than 5 valid scans end the run (`wait_for_wifi`,
   `getting_there.c`). AP2 was still audible at −82 to −91 dBm, and six of the seven headings were never tried.
   This is the same stop as the `WIFI_LOST` run earlier today. Over the AP2 leg, 28 of 133 scans missed.
6. **Time.** 186.9 s in total. Wi-Fi sampling took 120.1 s (64%). The motors ran 23.4 s (12.5%), of which 8.2 s was
   reversing (departure 1.5 s, foreign-mat return 0.7 s, watchdog return 6.0 s). Five cruise segments, largest
   control-loop gap 9 ms.
