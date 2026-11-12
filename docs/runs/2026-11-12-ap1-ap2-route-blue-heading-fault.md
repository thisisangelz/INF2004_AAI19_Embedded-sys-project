# AP1 to AP2 route, blue mat: reached AP1's white edge, then a heading fault on the way back (12 November)

Log: [2026-11-12-ap1-ap2-route-blue-heading-fault.txt](2026-11-12-ap1-ap2-route-blue-heading-fault.txt).
Firmware banner: `bounded eight-heading acquisition, foreign-mat guard, no-new-best watchdog | AP1 -> AP2 route`,
with cruise. Target: AP1, blue mat. `floor_step_ms=750`, `minimum_improvement_db=2`. Saved-run replay.

The robot found the strongest AP1 signal of any route run so far (−58 dBm) on a white mat edge, but never saw a
blue ring. It drove on past the edge, and the return to its best point failed on a heading fault after 291 s.
GP21 was not pressed: the fault mask has only `HEADING`.

| Phase | What the log shows |
|---|---|
| Heading −1 | −72/−72 at the start. The cruise stopped at 931 ms on a faint white (`c=1851`). Two 250 ms steps, then floor at 1435 ms. AP1 then dropped out of most scans: two missing samples, then −76/−75, two more missing samples at 2400 ms, then −73/−73. No gain. |
| Return | Reversed 2400 ms. The start read −68/−69 at heading 358. |
| Trial 1/7, heading 268 | −62/−66 at the start, −67/−67 after 2405 ms (−5 dB). Returned and read −63/−64. |
| Trial 2/7, heading 88 | −64/−63 at the start, −65/−66 after 2404 ms (−3 dB). Two colour checks on the way back flagged a foreign mat. Returned and read −63/−62. |
| Trial 3/7, heading 178 | −74/−73 at the start. The cruise stopped at 2203 ms on a bright white edge (`c=5727`) at −63/−62. Then −60/−60 at 2401 ms, the first confirmed improvement, and a best of −59/−58 at 2653 ms. Five 250 ms steps on white, no blue ring. Floor again at 3660 ms (−61/−62). |
| Cruise past the edge | One 6002 ms cruise at heading 178 (`end=length`), moving scans −62 to −66. The stop at 9662 ms read −62/−63, 5 dB below the best. The no-new-best watchdog started a return to 2653 ms. |
| End | The first reverse block turned the robot 6° (heading error −8°) and the colour read `UNKNOWN`, `c=500`. The second block failed: `DRIVE FAILED: heading error -25 ... exceeds 25`, fault mask `0x004` (`HEADING`). |

## What it showed

1. **White alone does not start colour steering.** At heading 178 the robot sat on AP1's white edge at −58 to
   −60 dBm for five stops. Mat steering needs a ring of the target mat, so it kept stepping straight along 178 and
   came out onto floor at 3660 ms.
2. **The 6 s cruise carried it 6 s past its best point.** After the white, the moving scans stayed within the
   decline limit, so the cruise ran its full 6000 ms. The watchdog then had to reverse 7 s of driving.
3. **The heading fault looks like a collision (guess).** The cruise ended on an unusually bright surface
   (`c=9336`) with a peak correction of 8 duty points, against 2 to 4 elsewhere. The first reverse block twisted
   the robot 6° and the sensor read almost dark (`c=500`). The log cannot show what the robot touched.
4. **The robot's own heading shifts AP1's reading by up to 11 dB on one spot.** From the start point: 268 read
   −62/−66, 88 read −64/−63, 358 read −68/−69, 178 read −74/−73. Trials compare readings at one heading, so this
   does not fool the gain test. But much of the +10 dB at 178 comes from its low starting baseline. Against the
   start point's best reading (−62), the white edge (−58) is about 4 dB stronger.
5. **AP1 dropped out of scans mainly at heading −1.** 32 of 102 scans there returned `-127`, against 4 of 252 at
   the other headings. Four samples there counted as missing, in two pairs. Each time the next sample passed, one
   short of the three misses in a row that ended the earlier run today.
6. **The first white was probably not AP1's mat (inferred).** It read `c≈1850` and `br=883` to `903` on both
   passes. The floor read `br=801` to `865`, and the real white edge at 178 read `c≈5700` and `br=920` to `936`.
   Wi-Fi did not rise there either.
7. **Cruise segments skip the foreign-mat guard.** On the return along 88, two stopped colour checks flagged a
   foreign mat (`gr=760` to `778`, `br=623` to `645`, most likely AP2's pink mat) between about 900 and 1650 ms.
   The forward cruise crossed that stretch without stopping. In the route build other mats classify as floor, and
   `cruise_segment` stops only on a non-floor colour, so only stopped checks can set `foreign_mat`.
8. **Time.** 290.6 s in total, motors running 25.6 s (8.8%). Six cruise segments, largest control-loop gap 9 ms.
