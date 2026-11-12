# AP1 to AP2 route, blue mat: AP1 dropped out of the scans after the first turn (12 November)

Log: [2026-11-12-ap1-ap2-route-blue-wifi-lost.txt](2026-11-12-ap1-ap2-route-blue-wifi-lost.txt).
Firmware banner: `bounded eight-heading acquisition, foreign-mat guard, no-new-best watchdog | AP1 -> AP2 route`,
with cruise. Target: AP1, blue mat. `floor_step_ms=750`, `minimum_improvement_db=2`. Copied from the live serial
output, so it has no `SAVED RUN` header.

The robot never reached the mat and stopped itself after 75 s. Every `COLOUR` check read `FLOOR` with `mats=NONE`.
GP21 was not pressed: the fault mask has no `STOP_LATCHED`.

| Phase | What the log shows |
|---|---|
| Start, heading 0 then −1 | −71 dBm. The next sample had only 4 valid scans of 7 and counted as missing. Then −70 and −70. |
| Heading −1 | One cruise segment of 2401 ms (`end=length`), 4 scans while driving at −66 to −71 dBm. The stop read −71, then a 3-valid sample (missing), then −70. Gain −1 dB, no confirmed improvement. |
| Return | Reversed 2401 ms in blocks of 754, 754, 754 and 139 ms. Back at the start it read −66 and −66 dBm. |
| Direction trial 1/7 | Turned −88° to heading 268. Three samples in a row had 2, 1 and 4 valid scans of 7. |
| End | `FAILED: AP1 missing from three fresh Wi-Fi samples; no blind movement`, `RESULT: ROUTE STOPPED`, fault mask `0x010` (`WIFI_LOST`). |

## What it showed

1. **AP1 was in range but often absent from a scan.** 30 of 88 scans returned `dbm=-127`. The scans that heard
   AP1 read −62 to −79 dBm.
2. **Heading 268 is where it fell apart.** From the same spot, heading 358 missed 1 scan of 14 at −62 to −69 dBm.
   After the turn to 268, 14 of 21 scans missed and the rest read −69 to −79 dBm, a median of −75 against −66. A cause is a
   guess: the robot body or battery may shadow the Pico W antenna when AP1 is behind it.
3. **The 5-of-7 rule turns a weak heading into a run failure.** A sample with fewer than 5 valid scans counts as
   missing, and three missing samples in a row end the run (`wait_for_wifi`, `getting_there.c`). At heading −1 two
   samples failed (4 and 3 valid) and the next one passed. At 268 none passed. The robot never tried the other six
   headings, although AP1 was still audible at −73 to −79 dBm.
4. **The start reading moved 4 dB.** Before the trial the start read −70 and −70 dBm. After the return, at nearly
   the same heading (358 against −1), it read −66 and −66. The trial looks for a 2 dB improvement. The return is an
   approximate reverse, so the robot may not be on the same spot, but the difference is twice the threshold.
5. **Most of the run was Wi-Fi sampling.** 60.5 s of 74.6 s (81%) was scanning. The motors ran 4.8 s (6.4%), half
   of it reversing. The one cruise segment had `max_gap_ms=9` and `late_updates=0`.
