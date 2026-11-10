# Run log dashboard

A local page for reading the robot's saved run logs. It needs no server, no internet and no
install: open `dashboard/index.html` in a browser.

## Use

- The logs in `docs/runs/*.txt` are already embedded. Pick one in the left list, or open
  **Compare runs** to see every run side by side.
- **Open log file…**, or drag a `.txt` onto the page, to add a new log. Get one by pressing
  GP21 before GP20 on the robot with the serial monitor open, then save the output.
- Deep links: `index.html#run=purple` opens the first log whose file name contains `purple`;
  `index.html#compare` opens the comparison.
- After adding logs to `docs/runs`, run `node dashboard/build_runs.js` to embed them.
- `node dashboard/test_logparser.js` checks the parser against every saved log.

## What it shows

| Section | Source lines |
|---|---|
| Run time, moving share, stops | `LINE_POINT`, `DRIVE` |
| Where the time went | `RSSI_SAMPLE` (scan time), `DRIVE`, `TURN`, `COLOUR` |
| Timeline: signal strength | `RSSI_RAW`, `LINE_POINT`, `AP1 BLE: filtered …` |
| Timeline: colour under the sensor | `COLOUR`, `COLOUR_SCAN`, `MAT_STEP` |
| Timeline: driving time so far | `DRIVE` |
| Phase strip | `FOLLOW`, `DIP`, `DIP_CHECK`, `RETURN`, `TRIAL`, `DIRECTION SEARCH`, `MAT`, `ARRIVAL` |
| Estimated path | `DRIVE` (heading and duration) |
| Colour scans | `COLOUR_SCAN`, `COLOUR_SCAN_RESULT` |

## Limits to keep in mind

- **Only Wi-Fi scan lines and `LINE_POINT` carry a timestamp.** Every other line gets a time
  interpolated between the nearest stamped lines (marked `~` in the event log). After the last
  stamped line, the run end is estimated from the drive, turn and colour durations recorded there.
- **Drive time is what the motors actually ran** when the log has a `DRIVE OK` line (newer
  firmware), otherwise the commanded time. The colour time is about 345 ms per arrival check. "Everything else" is what is left: BLE checks, settling, flash log writes.
- **The path is dead reckoning** from commanded drive time and gyro heading. It shows shape and
  direction, not distance in centimetres. The sign of the heading follows the gyro sign the robot
  learned, so the picture may be mirrored.
- Scan-test logs (`colour_scan` build) have no timestamps, so they only show the colour scans.

## Files

| File | Purpose |
|---|---|
| `index.html`, `style.css`, `app.js` | The page |
| `logparser.js` | Log text to events, time series and summary numbers (browser and Node) |
| `build_runs.js` | Embeds `docs/runs/*.txt` into `runs.js` |
| `runs.js` | Generated. Do not edit by hand |
| `test_logparser.js` | Parser checks against every saved log |
