# Pico W Three-Beacon Wi-Fi + BLE Test

## Current firmware

The default build contains the robot in `getting_there` and the three
`picow_access_point*_powerbank` beacons. Use the [robot instructions](getting_there/README.md)
and [powerbank beacon instructions](ble_transfer_powerbank/README.md) for these boards.
The older tracker and wheel tests are available through Git history.

[Development history](docs/history.md) lists the earlier versions with links to
their exact source snapshots and explains how to replace completed branches with tags.

The current robot programme (v18) continues along its initial heading in 500 ms
forward steps on the floor and 250 ms steps once the colour sensor sees the mat,
including when AP1 RSSI changes are inconclusive. A median reference from
three positions reduces the effect of an isolated strong point. After three weak
positions, it rechecks while stopped, then allows up to two 250 ms forward probes.
Recovery resumes the same heading. Persistent decline returns only to the retained
best point, then tries both perpendicular directions and then straight back.
An inconsistent return no longer stops the run; three unsuccessful trials do.
Once the colour sensor sees the mat, colour scans steer the robot to ring 4,
and with a working sensor the payload transfers only there. See
[the robot instructions](getting_there/README.md) for the v18 changes, limits,
sampling settings and the hardware test procedure. Flash
`getting_there/build/getting_there.uf2` and confirm `line-search v18 steps500-mat250 back-search mat-steer colour-gate`.
Flat readings no longer end the initial forward trial or an improving line.
The 60-second combined powered travel budget, reserved return travel, 15-minute
run limit, GP21 STOP and fault checks still apply. BLE must still pass the
-58 dBm proximity check before the handshake and verified payload transfer.
This is a stronger trial requirement after v11 completed too early at -66 dBm. The
12-reading filter and three consecutive qualifying averages are retained.
Once proximity passes, failed transfers retry the same BLE address while stopped.
Verified payload completion and final acknowledgement trigger three short beeps.
The default firmware stops for Wi-Fi samples; moving acquisition requires
explicit enablement and timing qualification.

The [v16 hardware run](docs/runs/2026-11-03-v16-backtrack.md) records successful
backtracking, a perpendicular approach and verified payload completion with no
reported faults. Its full saved log and firmware checksum are retained there.

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
