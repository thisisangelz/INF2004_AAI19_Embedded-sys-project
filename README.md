# Pico W Three-Beacon Wi-Fi + BLE Test

## Current firmware

The default build contains the robot in `getting_there` and the three
`picow_access_point*_powerbank` beacons. Use the [robot instructions](getting_there/README.md)
and [powerbank beacon instructions](ble_transfer_powerbank/README.md) for these boards.
The older tracker and wheel tests are available through Git history.

[Development history](docs/history.md) lists the earlier versions with links to
their exact source snapshots and explains how to replace completed branches with tags.

The current robot programme follows the first direction with confirmed AP1 RSSI
improvement in 250 ms forward steps. Reversal requires three successive weak
positions, with two fresh batches at each. On confirmed decline it returns only to
the retained best point, then checks only the two perpendicular directions.
An inconsistent return or two unsuccessful side checks stops the run. See [the robot instructions](getting_there/README.md)
for limits, sampling settings and the hardware test procedure. Flash
`getting_there/build/getting_there.uf2` and confirm `line-search v7 two-directions`.
The default firmware stops for Wi-Fi samples; moving acquisition requires
explicit enablement and timing qualification.

## Project structure

```text
ble_transfer/
  transfer_protocol.h          shared packet types, limits and CRC-32
  file_transfer.gatt           custom GATT service
  btstack_config.h             shared BTstack configuration
ble_transfer_powerbank/
  beacon_server.c              shared AP1/AP2/AP3 server
  powerbank_load.c             core 1 arithmetic load
getting_there/                 current robot programme
picow_access_point_powerbank/  AP1 build target
picow_access_point2_powerbank/ AP2 build target
picow_access_point3_powerbank/ AP3 build target
docs/history.md               earlier versions and branch archive procedure
```

Hardware bring-up for the navigation work is documented in
[`MPU9250_CONNECTION.md`](MPU9250_CONNECTION.md). The first firmware step
probes an MPU-9250/MPU-6500 over I2C1 on GP6/GP7 and reports WHO_AM_I at boot.

Each AP CMake file compiles the shared server with a different beacon ID and
Wi-Fi SSID.

## Build and flash

The default root build produces four firmware files: `getting_there` and the
three powerbank beacons. Use Pico SDK 2.3.0 for this build: the robot's persistent
log layout requires linker support unavailable in SDK 1.5.1. The beacons can also
be built separately with SDK 1.5.1.

### From VS Code (Raspberry Pi Pico extension)

1. Open the repository root folder in VS Code.
2. Pico sidebar icon -> **Import Project**, select this folder, choose an SDK
   version and click **Import**. This adds the extension's header block to the
   root `CMakeLists.txt` and creates `.vscode/`. Do this once per machine.
3. Click **Compile Project**. The UF2 files appear under `build/`.

### From a terminal

Set `PICO_SDK_PATH` to Pico SDK 2.3.0 and put CMake, Ninja and the ARM compiler
on the path.

```bash
cmake -G Ninja -S . -B build
cmake --build build
```

Each subfolder can still be built on its own, e.g.
`cmake -G Ninja -S picow_access_point_powerbank -B picow_access_point_powerbank/build`.

Flash these UF2 files (paths for the root build):

| Board | UF2 |
|---|---|
| AP1 | `build/picow_access_point_powerbank/picow_access_point_powerbank.uf2` |
| AP2 | `build/picow_access_point2_powerbank/picow_access_point2_powerbank.uf2` |
| AP3 | `build/picow_access_point3_powerbank/picow_access_point3_powerbank.uf2` |
| Robot | `build/getting_there/getting_there.uf2` |

The firmware has been compile-tested. The BLE range threshold, radio coexistence
and full transfer sequence still require a four-board hardware test.
