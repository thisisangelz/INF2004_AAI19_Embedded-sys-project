# RUN LOG DASHBOARD

A page for reading the robot's run logs. `docs/` is the GitHub Pages site, and
`docs/index.html` is its home page. It needs no server and no install, so you can also open
`docs/index.html` straight from disk.

## Publish on GitHub Pages

In the repository settings, open **Pages** and set **Source** to **Deploy from a branch**,
branch `main`, folder `/docs`. The site then rebuilds on every push to `main`.

`docs/.nojekyll` turns off Jekyll, so GitHub serves the files as they are. Keep it.

## Password

The page asks for a password before it shows anything. The build encrypts the run data (gzip,
then AES-256-GCM with a key from PBKDF2-SHA256, 250 000 rounds) into `assets/data.js`. The page
holds only that encrypted data, so neither the password nor the runs can be read with inspect
element. After a correct password, the derived key (not the password) is kept for that browser
tab, so a reload does not ask again.

The password is not in the repository. Pass it to the build:

```sh
DASHBOARD_PASSWORD=<password> node docs/tools/build_runs.js
```

To change the password, rebuild with the new one and commit `assets/data.js`.

**Limit:** this protects the dashboard page only. The raw logs and notes in `docs/runs/` are
still served by GitHub Pages at their own addresses, and anyone who can read the repository can
read them.

## Use

- **All runs** lists every log by date, newest first. Click a run to open it. **Notes** opens
  the written analysis on GitHub. **Raw log** opens the serial output.
- **Run** shows one run. Use the picker or the ‹ › buttons to move between runs.
- **Compare runs** shows every run side by side.
- **Field** places runs on the 5 × 5 grid of 60 cm tiles. In the Run tab, click the grid to place
  the start, AP1 (blue) and AP2 (red), and pick the way the robot faced. The Field tab
  calibrates the drive speed from runs that reached AP1. Per run, it lists the time to AP1, the
  distance driven, how direct the path was and how close the robot came to each beacon.
- **Add run…** takes raw serial monitor output. Paste it, pick the date and a short name, and the
  run opens with its timeline and event log. You can also choose or drag in `.txt` files. Added
  runs stay in this browser, marked *draft*, until they are committed.
- Deep links: `#run=purple` opens the first log whose file name contains `purple`. `#runs`,
  `#compare` and `#field` open those tabs. The address follows what you open, so you can share it.

## Add a run

1. In the dashboard, open **Add run…** and paste the serial monitor output. Check the event log.
2. Click **Download .txt**, or **Download all** on the All runs tab, and put the file in
   `docs/runs/`. Its name is `YYYY-MM-DD-short-name.txt`.
3. Optional: place the run on the field in the Run tab. Then click **Export layouts.json** and
   replace `docs/runs/layouts.json` with the downloaded file.
4. Rebuild with `DASHBOARD_PASSWORD=<password> node docs/tools/build_runs.js`, then check with
   `node docs/tools/test_logparser.js`.
5. Commit and push to `main`. Once a log is committed, its draft disappears from the browser.
6. Notes come later: a `docs/runs/YYYY-MM-DD-short-name.md` whose `#` heading becomes the run
   title and which links the log as `(YYYY-MM-DD-short-name.txt)`. Until then the card says
   *No notes yet*.

## What it shows

| Section | Source lines |
|---|---|
| Run time, moving share, stops | `LINE_POINT`, `DRIVE` |
| Where the time went | `RSSI_SAMPLE` (scan time), `DRIVE`, `TURN`, `COLOUR` |
| Timeline: signal strength | `RSSI_RAW`, `LINE_POINT`, `AP1 BLE: filtered …` |
| Timeline: colour under the sensor | `COLOUR`, `COLOUR_SCAN`, `MAT_STEP` |
| Timeline: driving time so far | `DRIVE` |
| Phase strip | `FOLLOW`, `DIP`, `DIP_CHECK`, `RETURN`, `TRIAL`, `DIRECTION SEARCH`, `MAT`, `ARRIVAL` |
| Field map | `DRIVE` (heading and motor time), `AP1 PAYLOAD COMPLETE`, and the positions you enter |
| Estimated path (no positions yet) | `DRIVE` (heading and duration) |
| Colour scans | `COLOUR_SCAN`, `COLOUR_SCAN_RESULT` |

## Limits to keep in mind

- **Only Wi-Fi scan lines and `LINE_POINT` carry a timestamp.** Every other line gets a time
  interpolated between the nearest stamped lines (marked `~` in the event log). After the last
  stamped line, the run end is estimated from the drive, turn and colour durations recorded there.
- **Drive time is what the motors actually ran** when the log has a `DRIVE OK` line (newer
  firmware), otherwise the commanded time. The colour time is about 345 ms per arrival check.
  "Everything else" is what is left: BLE checks, settling, flash log writes.
- **The path is dead reckoning.** The log has no position sensor. Distance is motor time × a
  drive speed, and heading comes from the gyro. On the field map, the speed is calibrated from
  runs that reached AP1, and the path snaps onto AP1 at the transfer. Turns, wheel slip and the
  arrival ring's offset from the beacon still show up as error, which the Field tab reports.
  Without positions, the estimated path may be mirrored.
- **The result chip follows the `RESULT` line.** A route run that delivered AP1 and then stopped
  still shows "no payload".
- Scan-test logs (`colour_scan` build) have no timestamps, so they only show the colour scans.

## Files

| File | Purpose |
|---|---|
| `index.html` | The page and the password screen |
| `assets/style.css`, `assets/app.js` | Layout and views |
| `assets/gate.js` | Asks for the password and decrypts `data.js` |
| `assets/logparser.js` | Log text to events, time series and summary numbers (browser and Node) |
| `assets/fieldmap.js` | Places a run on the field grid, calibrates speed, computes per-run field numbers |
| `assets/data.js` | Generated and encrypted by `tools/build_runs.js`. Do not edit by hand |
| `tools/build_runs.js` | Embeds `runs/*.txt`, the notes' titles and `runs/layouts.json` |
| `tools/test_logparser.js` | Parser and field-map checks against every saved log |
| `runs/` | Saved logs (`.txt`), their notes (`.md`) and field positions (`layouts.json`) |
| `.nojekyll` | Tells GitHub Pages to serve the files without Jekyll |
