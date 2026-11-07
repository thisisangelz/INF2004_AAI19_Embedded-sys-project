# Colour scan test on the mat (7 November)

Firmware: `colour_scan` test build (feature/colour-scan-mat-direction). After gyro
calibration the robot turns on the spot through two full rounds, 12 stops of 30
degrees each, and reads the TCS34725 at every stop. Radios and the approach are
off. Angles are measured from the starting heading; in these runs a positive
motor turn gave gyro sign +1, so angles increase toward the robot's left (both
"beacon on left" runs point there).

| Log | Start | Round 1 darkest / vector | Round 2 darkest / vector | Floor seen at |
|---|---|---|---|---|
| [ring2-beacon-left](2026-11-07-colour-scan-ring2-beacon-left.txt) | Ring 2, mat centre on the left | 60 / 66 | 60 / 60 | 210-270 |
| [ring2-slanted-inward](2026-11-07-colour-scan-ring2-slanted-inward.txt) | Border of rings 2 and 3, angled inward | 60 / 48 | 60 / 45 | 180-270 |
| [ring2-slanted-outward](2026-11-07-colour-scan-ring2-slanted-outward.txt) | Border of rings 1 and 2, angled outward | 300 / 282 | 300 / 286 | 60-120 |
| [ring4-beacon-left](2026-11-07-colour-scan-ring4-beacon-left.txt) | Ring 4, mat centre on the left | 60 / 45 | 60 / 45 | none (white at 210-240) |

## What worked

- **Repeatable.** In all four runs the darkest angle was the same in both rounds
  and the vector moved by 6 degrees at most.
- **Self-consistent.** The lightest readings (floor or white) were always about
  180 degrees from the darkest ones, as expected for a robot off the centre.
- **Turning is accurate.** 24 turns per run with errors of 0 or 1 degree; the
  heading was back at 0-1 after each round, and round 2 readings almost matched
  round 1, so the robot barely moved while turning.
- **Classification held** although everything read about 30-40% brighter than
  in the 6 November calibration (white C about 11,000 against 7,800; ring 5 about
  2,600 against 1,900). G/R did not change, and only the floor/ring 1 split uses
  brightness: floor stayed at or below C 3,900 and ring 1 at or above 8,200
  (threshold 4,660).

## Observations to keep in mind

- **From ring 2 the sensor swings off the mat onto the floor** (user's note:
  "sometimes when it turns it turns out of the colour mat and onto the ground").
  The sensor sits ahead of the wheels, so from ring 2 it reaches past the white
  edge on the outward side. For the direction estimate this helps (floor is depth
  0 and always lay opposite the dark side). For a steering mode it means a scan
  near the edge will read floor, and the robot must not treat that as having left
  the mat.
- **Dim white readings near the edge.** Several readings had white G/R
  (1,004-1,031) but low brightness (C 4,509-5,495), likely the sensor half over
  the edge or tilted. One reading (slanted-outward, round 1, angle 150: C 3,652,
  G/R 981) was classified WHITE although its brightness matches the floor. Floor
  G/R reached 965 in these runs against the white threshold of 968, so floor and
  white are only 3 units apart. Harmless for direction (depth 0 versus 1), but a
  floor reading taken as white could count as the "lighter ring seen" before
  arrival. A later fix could also require brightness for white (for example
  C of 4,000 or more); not changed yet.
- **Ring 3/4 boundary.** Ring2-beacon-left at angle 90 read G/R 551 (ring 4) in
  round 1 and 576 (ring 3) in round 2: a reading on the ring border.

## Still to confirm

- The true direction of the mat centre in each run, to measure the error. If
  "beacon on left" means about 45-60 degrees front-left, both of those runs were
  close; the slanted-outward run points about 75-80 degrees to the right.
- Timing: the log has no timestamps; one round is estimated at about 12 s
  (12 turns of about 0.6 s plus three readings of about 0.12 s per stop).
