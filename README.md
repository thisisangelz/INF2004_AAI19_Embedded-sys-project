# Pico W Three-Beacon Wi-Fi + BLE Test

## Current firmware

The default build contains the robot in `getting_there` and the three
`picow_access_point*_powerbank` beacons. Use the [robot instructions](getting_there/README.md)
and [powerbank beacon instructions](ble_transfer_powerbank/README.md) for these boards.
The older tracker and wheel tests are available through Git history.

[Development history](docs/history.md) lists the earlier versions with links to
their exact source snapshots and explains how to replace completed branches with tags.

This branch, `experiment/coarse-fine-navigation`, contains experimental v17.
It starts with 500 ms forward movements and allows 750 ms after confirmed
improvement. It retains decline evidence across one inconclusive reading,
reassesses after at most 2 seconds without a confirmed strong region, and uses
125 ms movements to refine the turning point after backtracking.

BLE qualification remains **-58 dBm**, with the 12-reading filter and three
consecutive confirmations. A separate -64 dBm hint only shortens forward steps
to 250 ms; it does not permit a handshake.

See the [v17 instructions](getting_there/EXPERIMENTAL_V17.md) for settings,
limits and tests. Flash
[`getting_there_v17_experimental.uf2`](getting_there/build/getting_there_v17_experimental.uf2)
and confirm `line-search v17 coarse-fine ble-minus58`.

The working v16 code is preserved on `main` at
[`d35c860`](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/d35c86069a749db46ebfdbfe956ebbf2a87617d4).
Its [`getting_there.uf2`](getting_there/build/getting_there.uf2) is retained unchanged.
The [v16 hardware run](docs/runs/2026-11-03-v16-backtrack.md) records successful
backtracking and payload transfer. V17 has host tests and a firmware build;
it still needs its own hardware run.

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
| Robot (experimental v17) | `build/getting_there/getting_there_v17_experimental.uf2` |

The firmware has been compile-tested. The BLE range threshold, radio coexistence
and full transfer sequence still require a four-board hardware test.
