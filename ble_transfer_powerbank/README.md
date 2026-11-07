# Powerbank beacon firmware

These are the current AP1, AP2 and AP3 builds. They use the server and load code
in this folder and the common protocol, GATT definition and BTstack configuration
in `ble_transfer`. The original AP builds have been removed from the current
files; their source is linked from the [development history](../docs/history.md).

| Board | Project folder | Wi-Fi SSID | BLE name |
|---|---|---|---|
| AP1 | `picow_access_point_powerbank` | `I AM PICO W` | `PICO-BEACON-1` |
| AP2 | `picow_access_point2_powerbank` | `I AM PICO W 2` | `PICO-BEACON-2` |
| AP3 | `picow_access_point3_powerbank` | `I AM PICO W 3` | `PICO-BEACON-3` |

The `getting_there` robot uses these beacons. Run only one board for each beacon
identity at a time.

## Buzzer sounds (branch feature/buzzer-status-cues)

Firmware: `build/picow_access_point*_powerbank_cues.uf2` in each AP folder.
The Robo Pico buzzer on GP22 plays from a core 0 timer alarm; core 1's load,
Wi-Fi and BLE behaviour are unchanged. Set the buzzer mute switch to sound.

| When | Sound |
|---|---|
| BLE advertising started | One short beep |
| Robot connected (it passed its range check) | Continuous beeping; stops if the robot disconnects before finishing |
| Transfer complete on the beacon's side | Three beeps, each higher |
## Changes

- Version 2 keeps the second processor core (core 1) busy with continuous
  arithmetic, starting before the two-second startup delay. The load remains
  active during idle periods, BLE connections and transfers.
- The load loop runs from scratch X RAM using register arithmetic. Its running
  loop makes no radio calls, accesses no shared state and sends no
  additional Wi-Fi or BLE packets. Core 0 continues to run the radio software.
- Core 1 registers with the SDK's
  [flash-write pause mechanism](https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#group_pico_flash).
  Startup waits for that registration before initialising CYW43/BTstack. This
  allows BLE storage writes to pause the load when necessary. Core 1 interrupts
  remain enabled.
- The onboard LED stays on after wireless initialisation, including through BLE
  connection, transfer and disconnection. It adds a small continuous load.
- The firmware requests that Wi-Fi power saving be disabled after AP setup and
  prints the result. This uses the
  [Pico SDK power-management API](https://www.raspberrypi.com/documentation/pico-sdk/networking.html).
- The startup banner includes `power-bank test v2`, followed by
  `core 1 continuous arithmetic: ON (scratch RAM)`. The two-second serial status
  includes uptime in seconds, so a restart is easier to identify.
- Wi-Fi and BLE settings, the BLE protocol, advertising interval and core 0's
  main-loop sleep are unchanged from version 1. Clock speeds and core voltage
  are unchanged.

The LED no longer indicates transfer completion. Use the serial
`TRANSFER COMPLETE` message or the robot's completion indicators instead.
The LED alone does not prove that the AP is responding; check Wi-Fi detection too.

## Build and flash

From the repository root, with Pico SDK, CMake, Ninja and the ARM compiler
configured, build each project separately:

```sh
cmake -G Ninja -S picow_access_point_powerbank -B picow_access_point_powerbank/build
cmake --build picow_access_point_powerbank/build
cmake -G Ninja -S picow_access_point2_powerbank -B picow_access_point2_powerbank/build
cmake --build picow_access_point2_powerbank/build
cmake -G Ninja -S picow_access_point3_powerbank -B picow_access_point3_powerbank/build
cmake --build picow_access_point3_powerbank/build
```

The root build includes all three of these targets and `getting_there`.
It requires Pico SDK 2.3.0 for the robot's persistent log layout. The separate
beacon builds above also support SDK 1.5.1.

| Board | UF2 from a separate beacon build |
|---|---|
| AP1 | [picow_access_point_powerbank.uf2](../picow_access_point_powerbank/build/picow_access_point_powerbank.uf2) |
| AP2 | [picow_access_point2_powerbank.uf2](../picow_access_point2_powerbank/build/picow_access_point2_powerbank.uf2) |
| AP3 | [picow_access_point3_powerbank.uf2](../picow_access_point3_powerbank/build/picow_access_point3_powerbank.uf2) |

Hold BOOTSEL while connecting the board to a computer, then copy its matching
UF2 to the `RPI-RP2` drive. Reconnect it to the power bank for testing. To inspect
or rebuild the original firmware, use the source snapshot in the development history.

## What must be checked on the hardware

The reported shutdown occurs in less than a minute, including with the LED on.
The power bank's model, minimum current threshold and actual cause of shutdown
are unknown. Core 1 activity is intended to increase the load, but the additional
current must be measured with a USB current meter. Firmware cannot set or
guarantee a particular input current, or guarantee that the bank stays on.
Disabling Wi-Fi power saving may make little difference in AP mode.

The separate processor workload avoids changes to the radio configuration or
routine radio processing. Both cores and the radio still share the board's
power supply, so unchanged radio performance must be checked on the hardware.
Expect shorter battery runtime. The firmware has no programmed shutdown timer.

1. Power the flashed board from the actual power bank and USB cable, without a
   USB connection to a computer. Confirm the LED lights and the correct SSID
   appears in a Wi-Fi scan. If checking the version over USB serial first,
   confirm `power-bank test v2` before reconnecting to the power bank.
2. Leave it idle with no robot connected for at least five minutes as an initial
   check, then for the full intended test duration. Five minutes alone does not
   establish that it will last through a longer test.
3. Connect the robot, complete a BLE transfer and disconnect. Confirm that the
   LED remains lit and the AP is detectable before and after the transfer.
   Repeat transfers and compare Wi-Fi detection and BLE completion with the
   original firmware using the same positions and a stable USB power supply.

If it still shuts down, use a USB power supply that stays on or check whether
the power bank supports low-current operation for the required duration. Some
modes have a timeout, such as the
[two-hour limit described by Anker](https://service.anker.com/au/article-description/What-is-Trickle-Charging-Mode).
