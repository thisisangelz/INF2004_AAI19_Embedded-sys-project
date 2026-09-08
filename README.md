# Pico W RSSI Beacon Finder

## Overview

This project uses Raspberry Pi Pico W boards to create a simple Wi-Fi beacon finder.

Two Pico W boards operate as Wi-Fi access points:

- **AP1:** `I AM PICO W`
- **AP2:** `I AM PICO W 2`

A third Pico W continuously scans for both access points and compares their RSSI values.

The tracker:

- prints the latest RSSI values to the serial monitor every **200 ms**
- turns on a different LED depending on which access point has the stronger signal
- changes the buzzer pitch according to the RSSI strength
- turns the buzzer off if neither access point can be detected

RSSI is used as an indication of signal strength. A less negative RSSI value means a stronger signal.

For example:

```text
-40 dBm = stronger
-70 dBm = weaker
```

The stronger signal is treated as the access point that is likely to be closer.

---

## Hardware

- 3 × Raspberry Pi Pico W
- Maker Pi Pico board for the tracker Pico
- USB cables
- Computer with Raspberry Pi Pico SDK installed

The project was developed using:

- **Pico SDK:** 1.5.1
- **GNU Arm Embedded Toolchain:** 10.3.1
- **Ninja:** 1.11.1
- **CMake**

---

## System Layout

```text
Pico W #1
Access Point
SSID: I AM PICO W
        \
         \
          > Pico W #3
         /  RSSI Tracker
        /
Pico W #2
Access Point
SSID: I AM PICO W 2
```

The tracker does not need to connect to either access point. It only scans for their Wi-Fi advertisements and records their RSSI values.

---

## Tracker Pin Assignment

| Function | GPIO |
|---|---:|
| AP1 indicator LED | GP2 |
| AP2 indicator LED | GP3 |
| Maker Pi Pico buzzer | GP18 |

### LED behaviour

- **GP2 LED ON:** AP1 has the stronger RSSI
- **GP3 LED ON:** AP2 has the stronger RSSI
- **Both LEDs OFF:** neither access point has been detected recently

A 3 dB switching margin is used to reduce rapid switching when both RSSI values are very similar.

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
```

AP1 has the stronger signal because `-45` is greater than `-72`.

RSSI is not an exact distance measurement. Walls, people, reflections, interference and antenna orientation can change the reading.

---

## Buzzer Behaviour

The buzzer pitch increases as the RSSI becomes stronger.

Approximate mapping:

| RSSI | Buzzer frequency |
|---:|---:|
| -90 dBm | 300 Hz |
| -80 dBm | 750 Hz |
| -70 dBm | 1200 Hz |
| -60 dBm | 1650 Hz |
| -50 dBm | 2100 Hz |
| -40 dBm | 2550 Hz |
| -30 dBm | 3000 Hz |

This means moving towards the currently selected access point should generally produce a higher-pitched tone.

The Maker Pi Pico buzzer is connected to **GP18** and is controlled using PWM.

Make sure the Maker Pi Pico buzzer is not muted using its hardware mute switch.

---

## Serial Monitor Output

The programme prints the latest RSSI information every 200 ms.

Example:

```text
AP1: -68 dBm | AP2: -51 dBm | Closer: AP2 | Buzzer: 2055 Hz
AP1: -66 dBm | AP2: -49 dBm | Closer: AP2 | Buzzer: 2145 Hz
AP1: -55 dBm | AP2: -63 dBm | Closer: AP1 | Buzzer: 1875 Hz
```

If only one access point is detected:

```text
AP1: -64 dBm | AP2: N/A | Closer: AP1 | Buzzer: 1470 Hz
```

If neither access point is detected:

```text
AP1: N/A | AP2: N/A | Closer: NONE | Buzzer: OFF
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
└── pico_rssi_tracker/
    ├── CMakeLists.txt
    ├── pico_sdk_import.cmake
    ├── lwipopts.h
    └── pico_rssi_tracker.c
```

A second copy of the access-point project can be used for AP2 with its SSID changed to `I AM PICO W 2`.

---

## Building the Tracker

From the `pico_rssi_tracker` directory:

```bash
rm -rf build
cmake -S . -B build -G Ninja -DPICO_BOARD=pico_w
cmake --build build
```

A successful build should generate:

```text
build/pico_rssi_tracker.uf2
```

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

The tracker stores the latest RSSI value received for each target SSID.

For example:

```text
AP1 = -61 dBm
AP2 = -74 dBm
```

AP1 is selected because its RSSI is stronger.

A small hysteresis margin is used so that the selected LED does not rapidly switch when the two readings differ by only a small amount.

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
- The two access points should be placed far enough apart to make their signal-strength difference noticeable.
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
- more than two access points
- trilateration or fingerprint-based indoor positioning
- an OLED display showing the selected beacon and RSSI
