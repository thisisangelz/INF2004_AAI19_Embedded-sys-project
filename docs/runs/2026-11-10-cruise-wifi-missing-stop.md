# AP1 not heard at start: run stopped on the first sample (10 November)

Log: [2026-11-10-cruise-wifi-missing-stop.txt](2026-11-10-cruise-wifi-missing-stop.txt).
`route2_cruise`, AP1 on the blue mat. The robot printed this same run four times in the pasted serial capture,
because it was replayed four times; this file holds one copy.

The robot never moved. All 8 Wi-Fi scans of AP1 returned `dbm=-127` (not found), so the first 7-scan sample was
empty (`valid=0`). The robot printed `AP1 Wi-Fi missing; checking BLE while remaining stopped`, and the colour
gate refused the BLE check (`BLE transfer waits for the colour mat`). It then started a second sample, and GP21
was pressed after scan 8 finished. The run ended with `ROUTE STOPPED` and faults `WIFI_LOST` and `STOP_LATCHED`.

## What it showed

1. **AP1 was off or out of range.** The Wi-Fi scan itself worked: 8 scans completed in 707 to 717 ms each. The
   target SSID was not in any result. The beacon may not have been powered or advertising, or its power bank may
   have shut it off (the earlier low-current problem).
2. **GP21 was pressed after scan 8, not during it.** There is no `STOPPED:` line, so the new sampler message was
   not triggered. The next scan check saw the latched stop and ended the run.
3. **The fault pair reads as one cause.** `WIFI_LOST` here means "AP1 never seen". `STOP_LATCHED` is the operator.
