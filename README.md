# Pico W Three-Beacon RSSI Finder

## Overview

This project uses Raspberry Pi Pico W boards to create a simple three-transmitter Wi-Fi beacon finder.

Three Pico W boards operate as Wi-Fi access points:

- **AP1:** `I AM PICO W`
- **AP2:** `I AM PICO W 2`
- **AP3:** `I AM PICO W 3`

A fourth Pico W continuously scans for all three access points and compares their RSSI values.

The tracker:

- prints the latest RSSI values to the serial monitor every **200 ms**
- turns on a different LED depending on which access point has the stronger signal
- changes the beep interval according to the RSSI strength
- turns the buzzer off if none of the access points can be detected

RSSI is used as an indication of signal strength. A less negative RSSI value means a stronger signal.

For example:

```text
-40 dBm = stronger
-70 dBm = weaker
```

The stronger signal is treated as the access point that is likely to be closer.

---

## Hardware

- 4 × Raspberry Pi Pico W
- Maker Pi Pico board for the tracker Pico
- USB cables
- Computer with Raspberry Pi Pico SDK installed

The v2 firmware was verified using:

- **Pico SDK:** 2.3.0
- **GNU Arm Embedded Toolchain:** 15.2.1
- **Ninja:** 1.13.2
- **CMake:** 4.3.4

---

## System Layout

```text
Pico W #1 (AP1: I AM PICO W)   ──\
Pico W #2 (AP2: I AM PICO W 2) ───> Pico W #4 RSSI Tracker
Pico W #3 (AP3: I AM PICO W 3) ──/
```

The tracker does not need to connect to any access point. It only scans for their Wi-Fi advertisements and records their RSSI values.

---

## Tracker Pin Assignment

| Function | GPIO |
|---|---:|
| AP1 indicator LED | GP2 |
| AP2 indicator LED | GP3 |
| AP3 indicator LED | GP4 |
| Maker Pi Pico buzzer | GP18 |

### LED behaviour

- **GP2 LED ON:** AP1 has the stronger RSSI
- **GP3 LED ON:** AP2 has the stronger RSSI
- **GP4 LED ON:** AP3 has the stronger RSSI
- **All LEDs OFF:** none of the three access points has been detected recently

Only one indicator LED is on at a time. All three LEDs are off when no target access point has been detected recently.

A 3 dB switching margin is used to reduce rapid switching when the strongest RSSI values are very similar.

---

## Access Point Configuration

### Access Point 1

```c
#define WIFI_SSID "I AM PICO W"
#define WIFI_PASSWORD "pico12345"
```

### Access Point 2

```c
#define WIFI_SSID "I AM PICO W 2"
#define WIFI_PASSWORD "pico12345"
```

### Access Point 3

```c
#define WIFI_SSID "I AM PICO W 3"
#define WIFI_PASSWORD "pico12345"
```

The access points must use different SSIDs so that the tracker can distinguish between them.

---

## RSSI Behaviour

RSSI values are measured in dBm.

Typical values are:

| RSSI | Approximate strength |
|---:|---|
| -30 dBm | Very strong |
| -40 dBm | Strong |
| -50 dBm | Good |
| -60 dBm | Moderate |
| -70 dBm | Weak |
| -80 dBm | Very weak |
| -90 dBm | Extremely weak |

Example:

```text
AP1: -45 dBm
AP2: -72 dBm
AP3: -61 dBm
```

AP1 has the stronger signal because `-45` is greater than `-72`.

RSSI is not an exact distance measurement. Walls, people, reflections, interference and antenna orientation can change the reading.

---

## Buzzer Behaviour

The buzzer uses a fixed tone and emits short beeps. The time between beeps changes with RSSI, like a proximity sensor:

- weak signal: slow beeps
- strong signal: rapid beeps
- no detected access point: buzzer off

Approximate mapping:

| RSSI | Beep interval |
|---:|---:|
| -90 dBm | 1000 ms |
| -80 dBm | 850 ms |
| -70 dBm | 700 ms |
| -60 dBm | 550 ms |
| -50 dBm | 400 ms |
| -40 dBm | 250 ms |
| -30 dBm | 100 ms |

The beep is on for approximately 80 ms. Moving towards the currently selected access point should generally make the beeps occur closer together.

The Maker Pi Pico buzzer is connected to **GP18** and is controlled using PWM.

Make sure the Maker Pi Pico buzzer is not muted using its hardware mute switch.

---

## Serial Monitor Output

The programme prints the latest RSSI information every 200 ms.

Example:

```text
AP1: -68 dBm | AP2: -51 dBm | AP3: -74 dBm | Closer: AP2 | Buzzer: BEEP every 415 ms
AP1: -66 dBm | AP2: -49 dBm | AP3: -57 dBm | Closer: AP2 | Buzzer: BEEP every 385 ms
AP1: -55 dBm | AP2: -63 dBm | AP3: -70 dBm | Closer: AP1 | Buzzer: BEEP every 475 ms
```

If only one access point is detected:

```text
AP1: -64 dBm | AP2: N/A | AP3: N/A | Closer: AP1 | Buzzer: BEEP every 610 ms
```

If none of the access points is detected:

```text
AP1: N/A | AP2: N/A | AP3: N/A | Closer: NONE | Buzzer: OFF
```

The programme prints every 200 ms, but Wi-Fi scans can take longer than 200 ms. Therefore, each line displays the most recently received RSSI measurement.

---

## Suggested Project Structure

```text
beacon-finder/
│
├── picow_access_point/
│   ├── CMakeLists.txt
│   ├── pico_sdk_import.cmake
│   ├── lwipopts.h
│   └── picow_access_point.c
│
├── picow_access_point2/
│   └── ...
│
├── picow_access_point3/
│   ├── CMakeLists.txt
│   ├── pico_sdk_import.cmake
│   ├── lwipopts.h
│   └── picow_access_point.c
│
└── pico_rssi_tracker/
    ├── CMakeLists.txt
    ├── pico_sdk_import.cmake
    ├── lwipopts.h
    └── pico_rssi_tracker.c
```

Each access-point project uses a distinct SSID so the tracker can identify all three transmitters.

---

## Building the Tracker

From the repository root, configure and build the tracker with:

```bash
cmake -S pico_rssi_tracker -B pico_rssi_tracker/build
cmake --build pico_rssi_tracker/build -j
```

A successful build should generate:

```text
pico_rssi_tracker/build/pico_rssi_tracker.uf2
```

Build each transmitter from its own project folder in the same way. For example, AP3 is built with:

```bash
cmake -S picow_access_point3 -B picow_access_point3/build
cmake --build picow_access_point3/build -j
```

Flash the generated images to the boards as follows:

| Board role | Firmware image |
|---|---|
| AP1 transmitter | `picow_access_point/build/picow_access_point.uf2` |
| AP2 transmitter | `picow_access_point2/build/picow_access_point.uf2` |
| AP3 transmitter | `picow_access_point3/build/picow_access_point3.uf2` |
| RSSI tracker | `pico_rssi_tracker/build/pico_rssi_tracker.uf2` |

---

## Flashing the Pico W

1. Disconnect the Pico W.
2. Hold the **BOOTSEL** button.
3. Connect the Pico W to the computer while holding BOOTSEL.
4. Release BOOTSEL.
5. The Pico should appear as a drive named `RPI-RP2`.
6. Copy the generated `.uf2` file to `RPI-RP2`.
7. The Pico will reboot automatically.

---

## How the Tracker Decides Which Beacon Is Closer

The tracker stores the latest RSSI value received for each of the three target SSIDs.

For example:

```text
AP1 = -61 dBm
AP2 = -74 dBm
AP3 = -68 dBm
```

AP1 is selected because its RSSI is stronger.

A small hysteresis margin is used so that the selected LED does not rapidly switch when the strongest readings differ by only a small amount.

Example:

```text
AP1 = -60 dBm
AP2 = -59 dBm
```

A 1 dB difference may not immediately cause a switch.

If AP2 becomes clearly stronger:

```text
AP1 = -65 dBm
AP2 = -57 dBm
```

the tracker switches to AP2.

---

## Limitations

- RSSI does not provide an exact physical distance.
- Wi-Fi scans are not instantaneous.
- Nearby Wi-Fi networks may cause interference.
- Obstacles can weaken or reflect signals.
- The three access points should be placed far enough apart to make their signal-strength differences noticeable.
- Rapid movement may cause the displayed RSSI to lag slightly behind the actual position.

---

## Possible Improvements

Future versions could include:

- RSSI averaging to reduce noisy readings
- a moving-average filter
- NeoPixel colour indication
- different buzzer patterns for each beacon
- calibration for known distances
- BSSID/MAC-address identification instead of SSID identification
- more than three access points
- trilateration or fingerprint-based indoor positioning
- an OLED display showing the selected beacon and RSSI
