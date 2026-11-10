# Second cruise run: same blue ring 4 stop (10 November)

Log: [2026-11-10-cruise-blue-mat-box-stop-2.txt](2026-11-10-cruise-blue-mat-box-stop-2.txt).
Repeat of [the first cruise run](2026-11-10-cruise-blue-mat-box-stop.md): `route2_cruise`, AP1 on the
blue mat, `floor_step_ms=750`, `minimum_improvement_db=2`.

| Phase | What the log shows |
|---|---|
| Baseline | 3 stationary batches of 7 scans, −72 to −76 dBm. |
| Cruise 1 | 2403 ms in one segment, `end=length`, 4 scans while moving. The stop measured −67 dBm (+5 dB), a confirmed new best. |
| Cruise 2 | 5672 ms in one segment, 8 scans while moving, −68 rising to −56 dBm, `end=colour` on ring 1. The stationary check at the stop read ring 1. |
| Mat, scan 1 | 12-step colour scan pointed 12°. Five steps (250, 250, 100, 100, 100 ms) read ring 2, ring 3, ring 3, ring 4, then ring 3 again. The lighter last reading triggered a second scan. |
| Mat, scan 2 | Pointed 356°. Three 100 ms steps on ring 4 (`depth=5`). |
| End | `three small steps one ring from arrival read no darker`, `RESULT: ROUTE STOPPED`, fault `COLOUR_SENSOR`. No BLE attempt. |

## What it showed

1. **Cruise repeated well.** `max_gap_ms=9` and `late_updates=0` on both segments again, heading error
   within ±2° while moving, and 12 of 47 scans taken while driving. Cruise 2 drove 5.7 s in one go
   where a stationary run would have taken about eight stops.
2. **Both blue runs stopped at the same place.** Each run read ring 4 (`depth=5`) on three or more
   consecutive steps and never reached ring 5, which is the blue arrival ring. On ring 4 the `br`
   ratio sat at 2000 to 2119 in both runs. `colour_mat.c` needs `br` of at least 2283
   (`BLUE_BR_RING5_MIN`) for ring 5. Two runs with the same plateau point to the calibration or the
   mat, not the steering.
3. **The sensor kept getting darker on ring 4.** `c` fell 2484, 2074, 1997, 1735 in the last three
   steps (first run: 2924 to 2190), while `br` stayed flat. A falling `c` at a flat `br` could be the
   sensor moving into the beacon box's shadow rather than onto a darker ring. This is a guess. The
   log cannot separate the two, and a reading with the robot lifted over the centre would.
4. **The stop is by design, and the fault name misleads.** The route build refuses to transfer
   from an inferred centre. `FAULT: COLOUR_SENSOR` here means "no genuine arrival", not a sensor
   failure.
5. **Timing is partly estimated.** The log has no timestamps after the last Wi-Fi scan, and the
   colour scans and mat steering after it account for about 33 s of the 68 s shown. The driving time
   (9.5 s) is measured.
