# Colour-ring arrival from the three-chair start, 7 November 2026

Run log: [complete saved serial log](2026-11-07-colour-arrival-three-chairs.txt).
Firmware banner `line-search v16 dip-probes ble-minus58 colour-ring4`
(`getting_there/build/getting_there_colour_ring4.uf2`, branch
`feature/colour-ring-sensing`). The log ends with `RESULT: PAYLOAD CONFIRMED`,
`FAULT MASK: 0x000` and a complete-log marker.

The start position was the same as the previous day's v16 run, about three
chair widths from AP1. That run's log is kept for comparison:
[2026-11-06-v16-three-chairs-timeout.txt](2026-11-06-v16-three-chairs-timeout.txt).

## Comparison

| | 6 November (v16) | 7 November (v16 + colour) |
|---|---|---|
| Result | Stopped without payload | **Payload confirmed** |
| Faults | `RUN_LIMIT`, `WIFI_SCAN` (scan cut off by the limit) | none |
| Path | Three headings tried, two backtracks, final heading | One straight heading, no reversing |
| Powered travel | about 18.5 s (13.6 s forward, 4.9 s reverse) | 8.3 s forward (33 blocks of 250 ms) |
| Time | 15-minute limit reached | about 7 min 20 s from the first BLE check to arrival |
| On the mat | Reached it, BLE only -64 to -73 dBm | Ring 4 confirmed, transfer completed |

The 6 November run spent its time on wrong directions before finding the final
line. Different starting headings are the main reason the two paths differ; the
colour sensor does not steer the robot.

## Colour readings on arrival

Every check before the mat read `FLOOR` (G/R 903–951). The last five checks
followed the mat inwards, one ring per 250 ms block:

| Powered ms | Zone | C | G/R | Wi-Fi at that position |
|---:|---|---:|---:|---|
| 7300 | WHITE | 8697 | 1037 | -53/-53 dBm |
| 7552 | RING1 | 6903 | 940 | -49/-48 dBm |
| 7804 | RING2 | 5138 | 793 | -43/-43 dBm |
| 8056 | RING3 | 3974 | 628 | -49/-49 dBm |
| 8307 | RING4 | 2822 | 475 | (BLE check only) |

All values fall within the calibrated ranges. At ring 4 the colour check
reported `arrived=1`, the BLE gate became -85 dBm and the three filtered
averages were -58, -57 and -58 dBm. The handshake, 26-byte reply
(CRC `74ea41b4`) and final acknowledgement completed on the first attempt.

## Observations

- Each ring was sampled at least once: at this speed a 250 ms block moved the
  sensor about one ring width, so the lighter-ring requirement was met.
- The BLE averages at ring 4 (-58/-57/-58 dBm) would also have passed the
  strict -58 dBm gate. This run confirms the colour arrival sequence works on
  the robot; it does not by itself show a case the strict gate would have
  missed. The 6 November run reached the mat with BLE at -64 to -73 dBm, which
  is the case colour arrival is meant to cover.
- Wi-Fi peaked at ring 2 (-43 dBm) and was weaker at ring 3 (-49 dBm), so Wi-Fi
  alone is not a reliable indicator this close to the beacon.
- Each 250 ms block costs about 13 s of stationary BLE and Wi-Fi checks, so
  most of the 7 minutes was spent stopped.

## Firmware

- UF2 SHA-256 (as built): `98B50C5690C9FB9D1E89A0410E4AF143FFAEB0C93345ED90B7877B8226F778F8`.
- Log SHA-256: `16BCB26E7274FC642E6BE125E106EAF67547FA76F3AB3E42097C9C86D7D66C2E`.
- One run; repeatability from other starting positions is not yet established.
