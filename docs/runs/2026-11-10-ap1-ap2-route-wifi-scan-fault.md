# AP1 to AP2 route: no gain on heading 0, run stopped with a Wi-Fi scan fault (10 November)

Log: [2026-11-10-ap1-ap2-route-wifi-scan-fault.txt](2026-11-10-ap1-ap2-route-wifi-scan-fault.txt).
Firmware banner: `bounded eight-heading acquisition, foreign-mat guard, no-new-best watchdog | AP1 -> AP2 route`.
Target: AP1, pink mat. Robot COM port: COM5.

The robot never reached the mat. Every `COLOUR` check read `FLOOR` with `mats=NONE`.

| Phase | What the log shows |
|---|---|
| Heading 0 | 10 stops of about 500 ms, 4802 powered ms in total. Wi-Fi stayed between −61 and −70 dBm, never more than +1 dB above the start. The trial ended with no confirmed improvement. |
| Return | Reversed 4802 ms in 750 ms blocks back to the start. One block ended early after an IMU read error. |
| Direction trial 1/7 | Turned −90° to heading 269 and drove 4 stops. Wi-Fi fell from −63 to −71 dBm, then recovered to −65/−67. |
| End | The second Wi-Fi batch of the next stop never produced a scan. `FAILED: Wi-Fi scan stopped or timed out`, `RESULT: ROUTE STOPPED`, fault mask `0x120` (`WIFI_SCAN` and `STOP_LATCHED`). |

## What it showed

1. **Time per stop fell from about 13 s to 8.3 s.** The colour gate skips the 5 s BLE check
   at stops that are off the mat. Each stop is now two five-scan Wi-Fi batches (about 7.1 s)
   plus a colour read. The robot was moving for 11.5 s of 149 s (7.7%).
2. **The Wi-Fi signal is flat here.** Ten steps along heading 0 changed it by about 1 dB, so the
   trial found nothing to follow.
3. **Two IMU read errors** (`DRIVE, reg 0x43`) each stopped a drive block early (239 ms of 750 ms,
   340 ms of 500 ms). Both recovered on the first retry.
4. **The `WIFI_SCAN` fault is probably a side effect of the STOP.** `STOP_LATCHED` is set too,
   and no scan result appears in the last batch. If GP21 was pressed during that batch,
   the scan would be abandoned and raise the fault. The log cannot confirm this. If GP21 was not
   pressed, the scan really hung and that is a separate Wi-Fi fault.
