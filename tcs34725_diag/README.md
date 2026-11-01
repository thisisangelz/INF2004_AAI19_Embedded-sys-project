# tcs34725_diag

Standalone TCS34725 bring-up. No motors, no radio, no other GPIOs. Wiring: I2C0, SDA=GP0, SCL=GP1, addr 0x29, 100 kHz, 5 ms bound on every I2C transfer.

Stage 1 probes 0x29 and reads ID (0x12; expect 0x44, 0x4D also accepted). Stage 2 runs only after stage 1 passes: it sets ATIME/gain, powers on, enables ADC, then prints clear/R/G/B every 500 ms.

## Build (never flashes)
    . .\scripts\dev\env.ps1
    .\scripts\dev\build.ps1 tcs34725_diag
ELF: `C:\tmp_pico\tcs34725_diag\tcs34725_diag.elf` (Debug, -O0 -g).

## Debug in VS Code
1. Install the Cortex-Debug extension. Wire Debug Probe SWD/GND to the Pico W debug pins.
2. Run and Debug, pick **TCS34725 diag (Debug Probe...)**, press F5. Cortex-Debug starts OpenOCD, loads the ELF, flashes it, halts at `main`. Loading is done by that launch; this repo's scripts never flash.
3. Press F5 again to run. Pause, then watch `tcs_diag` in the Variables/Watch panel (or Live Watch).
4. In the Debug Console: `-exec print/x tcs_diag`

## Reading `tcs_diag`
| Field | Pass |
|---|---|
| `probe_ack` | 1 |
| `id_value` / `id_ok` | 0x44 / 1 |
| `stage` | 4 (MEASURE); 1 = probe NACK, 2 = ID read failed |
| `init_ok` | 1 |
| `clear,red,green,blue`, `data_valid` | non-zero, 1 |
| `i2c_error_count`, `last_error` | 0 (-2 = PICO_ERROR_GENERIC/NACK, -1 = timeout) |

USB serial (if enumerated within 3 s) prints the same values; a missing port does not block the test.
