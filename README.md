# Pico W Three-Beacon Wi-Fi + BLE Test

This branch implements a four-board prototype:

- AP1: Wi-Fi SSID `I AM PICO W`, BLE name `PICO-BEACON-1`
- AP2: Wi-Fi SSID `I AM PICO W 2`, BLE name `PICO-BEACON-2`
- AP3: Wi-Fi SSID `I AM PICO W 3`, BLE name `PICO-BEACON-3`
- Robot Pico W: Wi-Fi RSSI tracker and BLE central/client

The Pico W radio time-shares Wi-Fi and Bluetooth Low Energy. The robot keeps
scanning the three Wi-Fi APs while it waits for the target beacon. Once BLE
proximity is accepted, new Wi-Fi scans are paused until the transfer finishes.

## Test sequence

1. Power all three beacon Pico W boards and the robot Pico W.
2. The robot scans Wi-Fi until all three SSIDs have been seen recently.
3. Press the Maker Pi Pico button on **GP20**.
4. The robot targets AP1 and averages that beacon's BLE RSSI advertisements.
5. Manually move the robot Pico W towards AP1.
6. When the configured average threshold is met repeatedly, the buzzer becomes
   a constant tone, the boards connect, handshake and exchange test payloads.
7. Both boards verify the received payload with CRC-32 and acknowledge success.
8. The buzzer stops and the robot automatically repeats the process for AP2,
   then AP3.
9. After AP3 succeeds, all three LEDs stay on and the buzzer stays off. Pressing
   GP20 starts a fresh sequence.

The access points are *detected*, not joined by the robot. Their Wi-Fi networks
remain useful as RSSI beacons; BLE is used for close-range gating and transfer.

## BLE proximity settings

The settings are near the top of
`pico_rssi_tracker/pico_rssi_tracker.c`:

```c
#define BLE_CLOSE_RSSI_DBM (-35)
#define BLE_RSSI_SAMPLE_COUNT 8
#define BLE_CLOSE_REQUIRED_AVERAGES 3
```

The current `-35 dBm` value is an arbitrary starting point. It does **not** mean
15 cm on every Pico W. The robot maintains a rolling average of eight target
advertisements and requires three consecutive qualifying averages. A single
strong packet therefore cannot start a transfer.

RSSI cannot provide an exact distance because antenna orientation, the robot
body, people, reflections, power supply noise and the room change the result.
For a real 15 cm gate, calibrate the assembled hardware:

1. Put the robot and one beacon in their normal mounted orientations at 15 cm.
2. Record at least 50 BLE readings from the robot serial output.
3. Repeat for each beacon and for several orientations.
4. Repeat just outside the allowed zone, for example at 20 cm and 30 cm.
5. Choose a threshold that normally passes the 15 cm samples but rejects the
   farther samples. Use the weakest per-beacon value if one common threshold is
   required.
6. Increase the sample count or confirmation count if the result chatters.

A two-threshold enter/exit hysteresis can be added later if the robot must keep
making a proximity decision after connecting. This test only needs the entry
threshold.

## What the BLE handshake transfers

BLE first performs its normal connection procedure. The firmware then performs
this application protocol on custom service `0xFF20`, characteristic `0xFF21`:

```text
HELLO / HELLO_ACK
FILE_START (length + CRC-32) / FILE_READY
FILE_DATA chunks / FILE_END
FILE_RECEIVED (status + CRC-32)
REPLY_REQUEST
REPLY_START / REPLY_DATA chunks / REPLY_END
COMPLETE / COMPLETE_ACK
```

The robot sends `Robot test file delivered to beacon APn`. The beacon sends
`Reply file from beacon APn`. These are small byte payloads compiled into the
firmware, not files stored on an SD card or filesystem. The chunking, length,
sequence and CRC checks exercise the same control flow needed for a later real
file source. The current maximum received payload is 256 bytes.

This is a functional lab protocol, not a secure transfer protocol. It currently
uses an unpaired BLE connection. Add BLE pairing/bonding and application
authentication before sending private or safety-critical data.

## Robot pin assignment

| Function | GPIO |
|---|---:|
| AP1 indicator LED | GP2 |
| AP2 indicator LED | GP3 |
| AP3 indicator LED | GP4 |
| Maker Pi Pico buzzer | GP18 |
| Start button, active-low with pull-up | GP20 |

The pins can be overridden at compile time with `START_BUTTON_PIN` and
`BUZZER_PIN`. Check the carrier-board model before flashing: this project is
configured for the Maker Pi Pico arrangement already used by the repository.

## Buzzer and LED behaviour

- Before the sequence: the strongest Wi-Fi AP LED is selected and stronger
  Wi-Fi RSSI produces faster proximity beeps.
- During AP1/AP2/AP3 search: the current target LED is on.
- From accepted BLE proximity until transfer acknowledgement: constant buzzer.
- After each completed AP: its LED remains on.
- After all three complete: all LEDs on, buzzer off.

## Serial evidence

Use a USB serial monitor on the robot and, when debugging, on the beacon being
tested. Important robot messages include:

```text
START accepted: AP1 -> AP2 -> AP3
Wi-Fi RSSI | AP1: -51 dBm | AP2: -63 dBm | AP3: -70 dBm
BLE status | Target: AP1 | State: SCANNING FOR TARGET | Latest: -39 dBm | Average: -41 dBm | Threshold: >= -35 dBm | Close: 0/3
AP1 RANGE REACHED: average BLE RSSI -34 dBm passed threshold -35 dBm.
AP1 FILE TRANSFER STATUS: STARTING. Constant buzzer ON until completion.
AP1 BLE CONNECTION: connected successfully.
AP1 HANDSHAKE: HELLO sent; waiting for HELLO_ACK.
AP1 FILE SEND: sending chunk 1, 17/39 bytes queued.
AP1 FILE SEND: chunk 1 acknowledged by beacon.
AP1 FILE TRANSFER: beacon verified the robot file and matching CRC.
AP1 REPLY TRANSFER: received chunk 1, 17/26 bytes.
AP1 FILE TRANSFER COMPLETE: both Pico W boards acknowledged success.
AP1 disconnected cleanly. Moving on to AP2.
ALL THREE BEACON FILE TRANSFERS COMPLETE
```

Before the eight-reading BLE average is ready, the robot prints the latest
reading and `collecting sample n/8`. During transfer, the periodic BLE status
uses readable stages such as `CLOSE ENOUGH - CONNECTING`, `SENDING HANDSHAKE`,
`SENDING FILE DATA` and `WAITING FOR FILE CRC RESULT`.

The corresponding beacon prints handshake, received-chunk, CRC, reply-chunk and
final acknowledgement messages, then turns on its onboard LED.

## Project structure

```text
ble_transfer/
  beacon_server.c       shared AP1/AP2/AP3 BLE server and transfer protocol
  transfer_protocol.h   packet types, limits and CRC-32
  file_transfer.gatt    custom GATT service
  btstack_config.h      BTstack configuration
pico_rssi_tracker/      robot/central firmware
picow_access_point/     AP1 build target
picow_access_point2/    AP2 build target
picow_access_point3/    AP3 build target
```

Each AP CMake file compiles the shared server with a different beacon ID and
Wi-Fi SSID.

## Build and flash

Pico SDK 2.3.0 was used to verify all four builds.

```bash
cmake -S pico_rssi_tracker -B pico_rssi_tracker/build
cmake --build pico_rssi_tracker/build -j

cmake -S picow_access_point -B picow_access_point/build
cmake --build picow_access_point/build -j

cmake -S picow_access_point2 -B picow_access_point2/build
cmake --build picow_access_point2/build -j

cmake -S picow_access_point3 -B picow_access_point3/build
cmake --build picow_access_point3/build -j
```

Flash these UF2 files:

| Board | UF2 |
|---|---|
| AP1 | `picow_access_point/build/picow_access_point.uf2` |
| AP2 | `picow_access_point2/build/picow_access_point.uf2` |
| AP3 | `picow_access_point3/build/picow_access_point3.uf2` |
| Robot | `pico_rssi_tracker/build/pico_rssi_tracker.uf2` |

The firmware has been compile-tested. The BLE range threshold, radio coexistence
and full transfer sequence still require a four-board hardware test.
