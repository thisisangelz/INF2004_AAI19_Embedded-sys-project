# Leg 2 test: beside AP2's mat, stopped by a 2-degree turn error (17 November)

Log: [2026-11-17-04-leg2-fail-turn-fault.txt](2026-11-17-04-leg2-fail-turn-fault.txt).
Firmware: `robot_leg2` (`getting_there_leg2.uf2`), branch `feature/ap1-to-ap2-route` at 7065aa2. Same set-up as
[the run that reached AP2](2026-11-17-03-leg2-reached-ap2.md): start on AP1's ring 4 facing AP1, AP2 two tiles away.
The robot ended right next to AP2's mat without touching it.

| Phase | What the log shows |
|---|---|
| Leaving the pink mat | Off after 2013 ms. First AP2 sample −69 dBm. |
| Three-direction probe | +180: −80/−79, +90: −64/−66, −90: −75/−78 dBm. Followed +90 (heading 91), "clearly strongest". |
| Line 1, heading 91 | Two samples with only 2 of 5 scans heard AP2 at the start. Rose to −62/−63 at 8050 ms, flat for two stops, then down to −76. Back to the best point: −65/−64. |
| Side probe 1 | −90 side (heading 1): −80/−83; +90 side (heading 181): −58/−59. Tried +90 first. |
| Line 2, heading 181 | Rose to −55/−55 at 3524 ms, then fell. Back to the best point: −55/−58. |
| Side probe 2 | −90 side (heading 91): −65/−67; +90 side (heading 271): **−63, then −56**. "No clear side, keep the usual order". |
| End | Turned toward heading 91, settled about 2.3 degrees short. Three nudges did not move the robot: `TURN FAILED`, `FAULT: TURN`, `RESULT: ROUTE STOPPED before the AP2 payload`, about 8.5 minutes in. |

## What it showed

1. **The run was ended by a 2-degree turn error, not by navigation.** The robot was beside AP2 with −55 dBm.
2. **The side choice threw away a clear answer.** The 271 side was 2 dB better in one measurement and 11 dB in
   the other. The rule needed both to be 3 dB better; on average it was about 6 dB stronger, very likely the
   side the mat was on.
3. **Both lines went flat before they fell** (line 1: −63, −62 after −62). Each confirmed fall and drive back took
   about 1-1.5 minutes.
4. **A floor reading looked like a white edge.** While reversing from the −90 probe, one stop read
   `WHITE, BLUE` (c 3491, G/R 991, B/R 860): light floor, as on the earlier 17 November run at this spot. It
   was harmless while reversing, but could start an unneeded colour scan when driving forward.

## Changes made

- A turn within 5 degrees after its corrections counts (it used to need 2).
- When a side's two measurements disagree, the average decides (at least 3 dB, neither measurement the
  other way). Here: 271 first.
- Early side probe: after a 6 dB rise, two stops with no new best send the robot back to its best point to
  probe the sides at once.
