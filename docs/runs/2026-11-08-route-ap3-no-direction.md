# AP1 and AP2 verified; AP3 search exhausted (8 November)

Evidence: user-supplied full trace `1794140620429156000-62f0acd0.json`, firmware
`528631fa8aeaf8f562d8c2c131c3e5051387813f`, model version 3. The uploaded trace was
read for this diagnosis; it was not rerun. The screenshot is consistent with
the recorded final position.

## Recorded result

- AP1 confirmed at 132.888 s; AP2 at 253.113 s. Both received 39 bytes and
  returned 26 bytes with verified CRC and final acknowledgement.
- AP3 became the active target at 258.483 s. Its Wi-Fi remained visible; its
  final received signal was -60 dBm, inside the unchanged 5 m model range.
- At 261.473 s the active purple profile labelled the blue mat's white edge
  as purple white. That triggered an unnecessary colour scan. The scan found
  only white and floor, then correctly restored its entry heading.
- The next forward trial encountered blue rings, which the filtered purple
  profile treated as floor. At 285.308 s the model first recorded contact with
  AP2's box while the wheel commands remained forward. Multiple powered steps
  changed neither position nor RSSI.
- The 4800 ms timed return then reversed farther than the actual preceding
  displacement. The search restarted about 3.265 m from AP3, compared with
  about 2.761 m at the leg's start. The controller has no wheel odometry with
  which to measure this error; the trace coordinates are diagnostic only.
- One subsequent heading improved AP3's RSSI by 2 dB by 4800 powered ms. The
  unchanged confirmation margin was 3 dB. The cardinal trials exhausted and
  the robot stopped at 384.138 s with `NO_DIRECTION` (`0x400`), completed mask
  3 and zero AP3 connections or payload bytes. This was not a CRC failure or
  evidence that AP3 was out of radio range.

## Firmware changes

Require a coloured ring of the active mat before colour steering. White alone
remains ambiguous and can still select smaller steps, but is not a bearing.
Record fresh unfiltered foreign-colour observations in the stationary sensor
check. Stop and reject a direction at a foreign coloured mat, then retreat
before trying another heading. A failed sensor check clears that observation;
the route's existing sensor-failure policy remains in force. Dip probing also
stops when it encounters another mat.

After the three existing cardinal alternatives, try the four intermediate
diagonals. All use the existing finite trial, one supported extension, same-
heading before/after batches and spread-aware minimum 3 dB margin. Returns,
departures, scan work and trials remain subject to the existing 60-second
per-leg translation budget, 15-minute mission limit, STOP and fault checks.
No beacon coordinates or simulated contact flag are supplied to navigation.

The beacon protocol and beacon firmware do not need changing for this trace.
No simulator signal strengths, visibility range, mat geometry, collision
handling, colour captures or completion rules were adjusted to obtain a pass.
The simulator already compiles the selected firmware and exposes the new logs.

## Validation boundary

Regression fixtures cover white-only mat evidence, foreign rings, sensor
failure/reset, cardinal exhaustion, diagonal trials and rejection without
steering into a foreign mat. Compile/link checks validate build compatibility;
they are not a successful rerun of this arrangement or proof of hardware
arrival. Actual firmware tests and the full experiment require an explicit
execution request. The new result must be assessed from its own complete trace.
