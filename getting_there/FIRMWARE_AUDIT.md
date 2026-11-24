# Firmware audit: data representation and shared state

Audit of the `getting_there` robot firmware, 24 November 2026, branch
`refactor/firmware-audit`. It covers how each piece of application data is
sized, which data is shared between the main loop and interrupt context, and
what this pass changed. It was re-run after merging
`feature/early-probe-straight-on-first` (commit `159191b`); see
"Re-audit after the early-probe merge".

## What the firmware is

The audit brief assumed an RTOS, MQTT, wheel encoders, an ultrasonic sensor,
line and barcode sensors and hump estimation. None of these exist here, so the
matching items in the brief do not apply.

- **Scheduling.** A bare-metal main loop on core 0. Core 1 is unused. The
  other execution contexts are the CYW43 background IRQ (Wi-Fi scan
  callbacks), the BTstack async context (BLE callbacks), and the buzzer's
  alarm IRQ. There are no tasks and no queues.
- **Radio link.** BLE GATT to the beacons, using the binary 20-byte packet
  protocol in `ble_transfer/transfer_protocol.h`. Multi-byte fields are
  written little-endian byte by byte, and each file carries a CRC32.
- **Telemetry.** CSV text lines, written to a flash journal with the motors
  off and replayed over USB. `docs/assets/logparser.js` parses them for the
  dashboard. They stay text (see "Message sizes").
- **Sensing.** An MPU9250 gyro holds heading during timed PWM drives. A
  TCS34725 colour sensor finds the beacon's mat. "Line" in this code means a
  Wi-Fi RSSI search line.

## Changes in this pass

| Change | File | Effect |
|---|---|---|
| Removed the `line_profile[32]` array. It was written on every point and never read. | `getting_there.c` | Clearer code. 0 bytes of RAM, because `-Os` already removed the unread array. |
| Packed `wifi_record_t` from 20 to 16 bytes. | `wifi_sampler.c` | 260 bytes less `.bss`. |
| The navigation test now checks the logged `LINE_POINT` lines instead of the deleted array. | `tests/test_navigation.c` | Tests what the dashboard reads. |
| Removed the stale `artifacts/qa/` output (test executables and a report from `dev/tooling-setup` naming targets that no longer exist). | `artifacts/qa/` | Repo hygiene. |

Considered and not done:

- **Splitting `getting_there.c`** (1,444 lines after the merge) into route,
  mat-approach and search modules. This is the main structural improvement
  left. The host tests now run in CI, so it can be done as its own change.
- **Merging the three `picow_access_point*_powerbank` folders.** Each one
  builds on its own, `.gitignore` keeps their `.uf2` files on purpose, and
  `run_host_checks.py` reads each folder's SSID and beacon ID. Their
  `lwipopts.h` files differ only by a trailing newline.
- **Binary telemetry.** The flash journal has 512 KiB and no bandwidth limit.
  Making it binary would break `logparser.js` and the paste-to-dashboard
  workflow.
- **Bit-fields for main-loop-only scalars** (the flags in `getting_there.c`,
  the colour state, the drive struct). Only one context writes them, so
  packing saves a few bytes and adds shifts to the 5 ms control loop.

## Re-audit after the early-probe merge

`159191b` changes what happens after an early side probe. The robot now tries
the stronger side first only if that side reads at least 3 dB stronger than
the best point itself. Otherwise it carries straight on from the best point.
The merge with this branch had no conflicts.

| Check | Result |
|---|---|
| New file-scope or shared variables | None. The only new state is `follow_result_t.at_best`, a main-loop stack value. |
| New code in interrupt or callback paths | None. All changes are in `approach_loop`, `side_probe` and `end_line_at_best`, which run in the main loop. |
| This branch's changes still in place | Yes. `line_profile` is gone, `wifi_record_t` is 16 bytes, and the test still reads `LINE_POINT` lines. |
| Static RAM (`.bss`) | Unchanged by the merge (table below). |
| Code size | +443 bytes for the new comparison and log line. |
| Committed `.uf2` files | Built on the early-probe branch, so they do not include this branch's record packing. Rebuild before flashing from this branch. |

## Data representation table

"Bits needed" is the minimum for the valid range. Widths marked with
`_Static_assert` fail the build if the field becomes too narrow or wider than
`BITS_NEEDED` allows.

| Field | Owner | Valid range | Bits needed | Implementation | Contexts |
|---|---|---|---:|---|---|
| Fault mask | `faults.c` | 15 independent flags | 15 | `static volatile uint16_t faults`, one bit per `fault_t`; asserts `8 < FAULT_COUNT <= 16` | main, BTstack, button polling |
| BLE state | `payload_client.c` `status` | 0..22 (23 states) | 5 | bits 0-4, `packed_get/put`, asserted against `BLE_STATE_LAST` | main, BTstack |
| Attempt result | `status` | 4 `payload_result_t` values | 2 | bits 5-6, asserted | main, BTstack |
| Result pending cleanup | `status` | 4 values | 2 | bits 7-8, asserted not to overlap | main, BTstack |
| BLE flags (11) | `status` | boolean each | 11 | bits 9-19, `F_*` masks | main, BTstack |
| `sample_count` | `payload_client.c` `counters` | 0..12 | 4 | `uint32_t : 4`, asserted | BTstack only |
| `sample_index` | `counters` | 0..11 | 4 | `uint32_t : 4`, asserted | BTstack only |
| `close_count` | `counters` | 0..3 | 2 | `uint32_t : 2`, asserted | BTstack only |
| `pending_length` | `counters` | 0..20 | 5 | `uint32_t : 5`, asserted | BTstack only |
| `robot_offset`, `reply_offset`, `reply_length` | `counters` | 0..256 | 9 each | `uint32_t : 9`, asserted | BTstack; main reads `reply_length` after the attempt ends |
| BLE RSSI history | `payload_client.c` `rssi_samples` | 12 readings, 1..127 | 84 | `bitarray_put/get` over 11 bytes | BTstack only |
| RSSI code | `packed_bits.h` | -1..-126 dBm stored as 1..126, 127 = missing | 7 | `RSSI_BITS`, `rssi_encode/decode`, asserted | all |
| `wifi_sample_t.code` | `wifi_sampler.h` | 1..127 | 7 | `uint16_t : RSSI_BITS` | main |
| `wifi_sample_t.spread` | `wifi_sampler.h` | 0..126 | 7 | `uint16_t : 7` | main |
| `wifi_record_t.code` | `wifi_sampler.c` | 1..127 | 7 | `uint8_t : RSSI_BITS` (new) | CYW43 IRQ writes during a scan, main reads after |
| `wifi_record_t.stopped` | `wifi_sampler.c` | boolean | 1 | `uint8_t : 1`, shares the byte with `code` (new) | as above |
| `wifi_record_t.heading` | `wifi_sampler.c` | whole degrees as logged | 16 | `int16_t`, set by `whole_degrees()`, which truncates toward zero like the old `(int)` log cast (new) | as above |
| `wifi_record_t.wall_ms`, `powered_ms`, `scan_id` | `wifi_sampler.c` | no bound provable in this module | 32 | `uint32_t` | as above |
| Turn sign | `motion.c` `motion_state` | unknown, +1, -1 | 2 | `uint8_t : 2`, asserted | main writes, IRQ reads |
| Stop armed | `motion_state` | boolean | 1 | `uint8_t : 1` | main writes, IRQ reads |
| Drive active / start / elapsed | `motion.c` `drive` | boolean, ms timestamps | 1, 32, 32 | `volatile bool`, `volatile uint32_t` | main writes, CYW43 IRQ reads |
| Report state | `run_report.c` `report_state` | 3 booleans | 3 | `uint8_t : 1` x3, every write under `report_lock` | main, BTstack (deferred log) |
| Journal next page | `report_store.c` `store.next` | 0..2048 | 12 | `uint16_t : 12`, asserted `PAGE_COUNT == 2048` | main only |
| Journal active | `store.active` | boolean | 1 | `uint16_t : 1` | main only |
| Reading at the best point | `getting_there.c` `follow_result_t.at_best` | two 7-bit RSSI codes plus spreads | 28 | `line_pair_t` (two `wifi_sample_t`, 4 bytes), stack only (new in `159191b`) | main only |
| Target beacon | `target_beacon.c` `current` | 1..3 | 2 | `uint8_t` (main only, not packed) | main only |
| Colour depth | `colour_sensor.c` `last_depth` | -1..7 | 4 | `int` (main only, not packed) | main only |
| Lighter mats seen | `colour_sensor.c` `lighter_mats` | 3 mat bits | 3 | `unsigned` bitmask | main only |
| Buzzer playing | `common/buzzer.c` `playing` | boolean | 1 | `volatile bool` | alarm IRQ writes, main reads |

## Shared-state and race review

Every field that more than one context writes, or that an interrupt reads,
was checked for an unprotected read-modify-write. None was found.

- **Fault mask.** `fault_raise` and `fault_clear` mask interrupts around the
  OR or AND. Reads are single 16-bit loads.
- **BLE status word.** Every update goes through `packed_put`,
  `packed_set_bits` or `packed_clear_bits`, which mask interrupts. Aligned
  32-bit reads are single instructions on the RP2040.
- **BLE counters.** Only BTstack callbacks write them, so one context owns
  them. Attempts start inside `start_attempt_callback`, in BTstack context.
- **Report state.** Each write takes `report_lock`.
- **Motion flags.** Only the main loop writes `motion_state`. Interrupts only
  read it, so a bit-field write cannot lose an update.
- **Drive timing.** `stop_drive` saves `elapsed_ms` before it clears
  `active`. The Wi-Fi callback therefore reads either a live drive or a
  finished elapsed time, never a stale one.
- **Wi-Fi scan result.** `pending_result` is reset by the main loop before a
  scan starts. While the scan is open only the callback writes it, and the
  main loop copies it only after `cyw43_wifi_scan_active` reports the scan
  finished. Packing `code` and `stopped` into one byte adds no race, because
  the callback is the only writer while the scan is open.

This pass added no code to interrupt or callback paths except the 16-byte
copy, which replaces a 20-byte copy.

## RAM usage

Measured with `arm-none-eabi-gcc 15.2.1 -mcpu=cortex-m0plus -mthumb -Os`,
compiling the changed files against `tests/stubs`, and reading section and
symbol sizes with `arm-none-eabi-size -A` and `arm-none-eabi-nm -S`. These are
exact RP2040 object sizes. A fully linked map file was not available (see
"Verification").

| Symbol or section | Before (bytes) | After this branch | After the merge | Change overall |
|---|---:|---:|---:|---:|
| `wifi_sampler.o` `records[64]` | 1,280 | 1,024 | 1,024 | -256 |
| `wifi_sampler.o` `pending_result` | 20 | 16 | 16 | -4 |
| `wifi_sampler.o` `.bss` total | 1,376 | 1,116 | 1,116 | -260 |
| `getting_there.o` `.bss` total | 34 | 34 | 34 | 0 |
| `line_profile[32]` | 0 (removed by the compiler) | 0 | 0 | 0 |
| Code (`.text` plus `.rodata`, both objects) | 15,968 | 16,056 | 16,499 | +531 |
| `sizeof(wifi_record_t)` | 20 | 16 (asserted) | 16 | -4 |

Of the +531 bytes of code, 88 come from this branch's packing and 443 from
the early-probe feature.

For scale, the most recent linked image (`getting_there/build`, built before
this branch) has 130,272 bytes of `.bss`. The two 32 KiB report buffers in
`run_report.c` take 64 KiB of that. If RAM is ever short, removing the
`snapshot` copy is worth 32 KiB, far more than any bit-field.

## Message sizes

There are no inter-task queues. The nearest equivalents are the Wi-Fi
callback handoff (`pending_result`, 20 to 16 bytes, above) and the 4,096-byte
USB display ring in `run_report.c`, which is unchanged.

The BLE protocol was already compact binary and is unchanged. Sizes are on
the wire, before ATT overhead.

| Message | Direction | Layout | Bytes |
|---|---|---|---:|
| `MSG_HELLO` | robot to beacon | type, protocol version, target ID | 3 |
| `MSG_FILE_START` | robot to beacon | type, u16 length LE, u32 CRC32 LE | 7 |
| `MSG_FILE_DATA` | robot to beacon | type, sequence, chunk length, up to 17 data | 4..20 |
| `MSG_FILE_END`, `MSG_REPLY_REQUEST`, `MSG_REPLY_NEXT`, `MSG_COMPLETE` | robot to beacon | type | 1 |
| `MSG_HELLO_ACK`, `MSG_FILE_READY`, `MSG_COMPLETE_ACK`, `MSG_ERROR` | beacon to robot | type, beacon ID or error code | 2 |
| `MSG_FILE_RECEIVED` | beacon to robot | type, status, u32 CRC32 LE | 6 |
| `MSG_REPLY_START` | beacon to robot | type, u16 length LE, u32 CRC32 LE | 7 |
| `MSG_REPLY_DATA` | beacon to robot | type, sequence, chunk length, up to 17 data | 4..20 |

Telemetry lines (`LINE_POINT`, `RSSI_RAW`, `RSSI_SAMPLE` and others) are
unchanged, field for field, so the dashboard parser needs no change.

## Verification

- **Host compile.** `tests/test_navigation.c`, which includes
  `getting_there.c` and links `wifi_sampler.c`, compiles with
  `gcc -std=c11 -Wall -Wextra -Werror`.
- **ARM compile.** Both changed files compile cleanly for Cortex-M0+.
- **Compile-time checks.** `_Static_assert(sizeof(wifi_record_t) == 16)` and
  the existing width asserts all pass.
- **CI.** Smart App Control on the development laptop blocks newly built
  executables and `pioasm.exe`, so `.github/workflows/firmware.yml` runs the
  checks on Linux. Run 45342777587 (this branch, before the merge) passed the
  full host suite and built every target. On merge commit `066ffbb` the host
  suite passed again, including the new straight-on-first tests.
- **Timing.** No hardware measurement was taken. The control loop and IRQ
  paths gained no work. The callback's record copy is 4 bytes shorter.
