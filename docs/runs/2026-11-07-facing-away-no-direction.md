# Facing away from AP1, three-chair start, 7 November 2026

Run log: [complete saved serial log](2026-11-07-facing-away-no-direction.txt).
Firmware banner `line-search v16 dip-probes ble-minus58 colour-ring4 cues`.
Same start position as the successful colour-arrival run, but the robot was
placed facing away from AP1. Result: `STOPPED WITHOUT PAYLOAD`, fault
`NO_DIRECTION`, after about 4 minutes. The colour sensor read FLOOR throughout.

## What happened

| Powered ms | Wi-Fi (two batches) | Decision |
|---:|---|---|
| 0 | -64 / -67 | Starting point |
| 252 | -62 / -62 | |
| 504 | -60 / -60 | 4 dB gain over the start: "first confirmed improvement", best point |
| 756–2521 | -61 to -72 | Mostly weaker; one -72 dip recovered |
| 2773 | -71 / -69 | Third weak position: stationary recheck -69/-69, two 100 ms probes -69 and -71 |
| 2978 → 504 | | Reverse 2474 ms to the best point |
| 504 (returned) | -65 / -65 | 5 dB below the -60/-60 recorded there: "inconsistent", run stopped |

## Why it failed

1. **A false best point.** Moving away from AP1 should weaken Wi-Fi, but the
   first two steps read 4 dB stronger. The gain only just exceeded the 3 dB
   noise margin, so a small fluctuation was treated as real progress.
2. **The return check rejected the same spot.** Back at the "best" point Wi-Fi
   read -65/-65 instead of -60/-60. The timed reverse is approximate and Wi-Fi
   at one spot varies by several dB, so the 3 dB return tolerance failed.
   v16 deliberately stops when a return is inconsistent, rather than guess.
3. **v16 never tries the opposite direction.** After a decline it returns to
   the best point and tests only the two perpendicular headings. Facing away,
   the correct direction is behind the robot, which v16 does not test.

## Possible changes (for team review, not implemented)

- When the first line ends in a decline with only a marginal gain (for example
  under 6 dB), try 180 degrees before the perpendiculars.
- When a return reads inconsistent, take a fresh reference at the current
  position and continue the search within the run limits instead of stopping.
- Require a larger first gain before a point becomes the best return target.

These change holsoma's line-search logic, so they need agreement first.