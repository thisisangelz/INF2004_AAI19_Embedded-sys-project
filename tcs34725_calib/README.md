# TCS34725 colour-mat calibration

Measures what the robot's colour sensor reads on the floor, the white mat edge,
each of the five pink rings and one "other" surface such as a shadow or chair
leg. The results decide whether the rings can be told apart and which values
the robot should treat as "arrived". No motors or radio are used.

Based on the I2C bring-up in Babydage's `tcs34725_diag` (`dev/tooling-setup`).

| Item | Connection |
|---|---|
| TCS34725 | Robo Pico Grove 1: SDA GP0, SCL GP1, address 0x29 |
| Capture | GP20 |
| Back / redo | GP21 |
| Buzzer | GP22 (short beep at capture start, longer beep at the end) |

## Which file

| Mat | Coloured rings inside the white edge | File |
|---|---|---|
| Pink (AP1) | 5 | `build/tcs34725_calib.uf2` |
| Blue and purple | 6 (seven bands counting the white edge) | `build/tcs34725_calib6.uf2` |

Both are the same programme; only the list of surfaces differs. The first
serial line and the summary heading say `5-ring mat` or `6-ring mat`. The
summary compares each pair of neighbouring rings on `C`, `G/R` and `B/R`, so it
works whichever value changes as a mat's colour darkens.

## Procedure

1. Flash the file for the mat (table above) to the robot's Pico W (hold
   BOOTSEL while plugging in, drag the file onto RPI-RP2).
2. Keep USB connected and open the serial monitor. `LIVE` lines show the current
   reading twice a second.
3. Use the room lighting planned for the demo. Keep the sensor at its mounted
   height: the robot stands on its wheels as normal.
4. For each surface in the order printed (floor, white edge, ring 1 lightest to
   the darkest ring, other): put the robot so the sensor is over the middle of
   that surface, press GP20 and keep still until the second beep (about 2 s).
   GP21 steps back to redo the previous surface.
5. After the last surface, copy the whole serial output, including the `CSV`
   lines and the neighbour check, into the team notes.

Repeat at two or three spots per ring if time allows. Readings at the edge
between two rings are expected to fall between their values.

## Reading the result

- `C` is overall brightness; `G/R` and `B/R` are colour ratios x1000. The pink
  rings have lower `C` and lower `G/R` as they darken; blue or purple rings may
  change `B/R` instead. Ratios change less than `C` when the height or lighting
  changes.
- The neighbour check prints `separate` when no reading of the lighter ring
  overlapped the darker one, and `OVERLAP` otherwise. Two neighbouring rings
  only need to separate on one of the two values.
- The floor and the other surface have no expected order, so they are compared
  with every mat surface in both directions and marked `distinguishable` or
  `CONFUSABLE`. A gap of only a few percent of `C` is too small to rely on.
- `SATURATED` means a reading hit the sensor's maximum. Rebuild with a lower
  gain (`-DTCS_GAIN=0`) and repeat.
- Compare `OTHER` with the ring chosen as the arrival target. If it looks
  similar, the robot will need extra checks before treating a dark reading as
  arrival.

## Build

Builds with Pico SDK 1.5.1 or 2.x. From the repository root:

```bash
cmake -G Ninja -S tcs34725_calib -B tcs34725_calib/build
cmake --build tcs34725_calib/build
```
