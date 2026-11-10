# Cruise run: five directions tried, 60 s travel budget used up (10 November)

Log: [2026-11-10-cruise-blue-translation-limit.txt](2026-11-10-cruise-blue-translation-limit.txt).
`route2_cruise`, AP1 on the blue mat, `floor_step_ms=750`, `minimum_improvement_db=2`. The saved copy ends at
`[COMPLETE: all recorded run text saved]`, without the usual `END OF SAVED RUN` line.

The robot never got past the white edge of the blue mat and ran out of powered travel after 7 minutes.

| Heading | Trial | What happened |
|---|---|---|
| 0° | 2404 ms | The cruise stopped after 235 ms on the white edge, then stationary steps. Stops read −68 to −82 dBm. No gain. |
| 269° (−90°) | 4800 ms | Two cruise segments. Stops read −70 to −75 dBm. No gain. |
| 89° (+90°) | 4800 ms | Same. Stops read −71 to −79 dBm. No gain. |
| 179° | 2400 ms | Stops read −65 to −74 dBm. No gain. |
| 44° | 2402 ms | White edge again at 252 ms, then stops read −70 to −80 dBm. No gain. |
| 314° | 2404 ms, then 6003 ms and 4982 ms | +5 dB at 2404 ms, a new best at 8407 ms, then a decline. The budget ran out here. |
| End | | `FAILED: translation budget, used 47017 ms, required including return 20682 ms, limit 60000 ms`, fault `TRANSLATION_LIMIT`. |

## What it showed

1. **Cruise ran for 7 minutes with no timing problem.** Largest control-loop gap `max_gap_ms=9` over 13 segments, `late_updates=0`. 45 of 523 scans were taken while driving. The moving-pair rule fired 3 times (`shows no new best; measuring while stopped`) and the stationary measurement decided each time.
2. **The travel budget, not the run time, ended it.** The robot drove 47.0 s of its 60 s allowance. 16.8 s of that (36%) was reversing to return to the start between direction trials. Each trial costs its forward distance twice.
3. **The Wi-Fi signal gave no clear direction here.** All five of the first trials ended 3 to 6 dB below where they started. The robot sat close to the mat (white edge within about 250 ms of driving on two headings). The only gain, on 314°, followed a low baseline of −80 to −82 dBm and faded after 8.4 s.
4. **Two of the four `end=colour` stops were false alarms.** At 89° (1509 ms) and 179° (582 ms) the stationary colour check at the stop read plain floor. The moving poll saw something non-floor that the stopped robot did not, most likely the mat edge it had just passed or a shadow (`c=2140` was low for floor at 89°). Each costs a stop, not a failure.
5. **AP1 was missing in 7 of 523 scans** (`dbm=-127`), three of them at the start of the 314° trial. That sample had only 4 valid scans of the 5 required, so the robot paused to check Bluetooth, which the colour gate refused as off the mat.
6. **The robot never read a coloured ring.** Every mat reading was `WHITE`. The blue arrival ring needs ring 5 or darker.
