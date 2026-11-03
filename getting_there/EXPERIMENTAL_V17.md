# Experimental v17: longer forward steps and return refinement

Branch: `experiment/coarse-fine-navigation`.
Working v16 source remains on `main` at
[`d35c860`](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/d35c86069a749db46ebfdbfe956ebbf2a87617d4).
Its successful [hardware run](../docs/runs/2026-11-03-v16-backtrack.md) is retained.

| Firmware | File | Boot banner |
|---|---|---|
| Experimental v17 | [getting_there_v17_experimental.uf2](build/getting_there_v17_experimental.uf2) | `line-search v17 coarse-fine ble-minus58` |
| Working v16 | [getting_there.uf2](build/getting_there.uf2) | `line-search v16 dip-probes ble-minus58` |

The v16 UF2 is unchanged. The experimental CMake target has a separate output
name, including when building in the usual directory. No beacon update is needed.

The supplied v17 UF2 is 1042944 bytes. Its SHA-256 is
`1fa602ee22831cae2a6d74cfb62f76e25011c5493701c4f93199a36290a3ef96`.
The preserved v16 SHA-256 remains
`08ca210e6c80391b2605baf091dd1f838e250d28ea53aa824bc4d224849b4bcb`.

## Movement and decisions

1. Start each heading with 500 ms forward blocks at the existing 35% duty and
   gyro correction. After two fresh Wi-Fi batches confirm improvement over that
   heading's baseline, allow 750 ms blocks. During an uncertain dip, use 500 ms.
2. A complete 12-reading BLE filtered average of at least -64 dBm latches a
   request for 250 ms forward approach steps for the remainder of the run.
   This only changes step size. **Handshake qualification remains -58 dBm with
   three consecutive qualifying averages**, followed by payload verification.
3. Keep the three-position median reference. It may strengthen but cannot
   weaken. Three weak positions among the last four trigger a stopped recheck.
   One inconclusive or recovered position no longer erases the other votes.
   Counted positions must be separated by at least 250 ms of powered travel.
4. Recovery requires stable readings: each batch's retained spread must be at
   most 3 dB, the two batch averages must agree within 2 dB, and both must be
   within 2 dB below the reference's weaker batch. Noise cannot widen this
   recovery tolerance. Two consecutive qualifying positions clear the decline
   evidence and update the latest confirmed strong region.
5. Reassess after at most 2000 ms of further powered travel without another
   confirmed strong region, even if fewer than three positions count as weak.
   Clamp the next movement to the remaining allowance. The existing per-block
   execution allowance can add at most 25 ms before the stopped reassessment.
6. Reassessment checks BLE while stopped, then collects **two fresh pairs** of
   Wi-Fi batches. Both pairs must satisfy the fixed recovery rule. One good
   pair is insufficient. If both pass, continue from this confirmed position;
   otherwise backtrack. These four batches describe one position, not four
   spatial observations. V16's extra 100 ms forward dip probes are removed.
7. On a heading that has improved, use its latest supported strong region as
   the return target. This allows later near-equal readings to replace an old
   isolated peak without weakening the reference. A heading that has never
   improved keeps its original starting point as the turning target; a broad
   near-equal plateau must not move that target sideways.
8. Backtrack in blocks of at most 750 ms to 250 powered ms beyond the target,
   then sweep backwards in 125 ms blocks across a local window extending up to
   250 powered ms on either side. Stop for BLE and two Wi-Fi batches after each
   fine movement. Select a stable point only when both batches beat the current
   selection; an isolated raw spike cannot select the turning point.
9. Reposition to the selected point in blocks of at most 125 ms, checking BLE
   after each, then verify it with two fresh Wi-Fi batches. Require consistency
   with the selected readings and the retained strong region before turning.
   A failed check stops navigation. Test the two perpendicular headings as in
   v16, following the first that improves.

Each fine sweep and reposition is limited to eight powered blocks. The bounded
window is a local search, not a claim to find the highest signal everywhere.
Shortened movements and execution overruns use their measured powered time.
An unsuccessful flat side trial still ends at 2400 powered ms and returns to its
starting point before trying the other side.

Wi-Fi remains two fresh five-scan trimmed-mean batches per normal checkpoint.
BLE remains stopped and separate from Wi-Fi, with its five-second scan window.
Longer forward blocks reduce how often these pauses occur per unit of travel.
The two-pair recheck and local refinement add scans only when reassessing or
choosing a turning point. Moving Wi-Fi acquisition remains disabled by default.

## Limits and verification

GP21 STOP, IMU checks, radio cleanup, missing-target handling, saved logging,
the 60-second total powered-translation limit and the 15-minute run limit remain.
Forward movement reserves its possible return plus both phases of refinement.
BLE completion interrupts every movement phase, including reverse and refinement.
No forward movement is permitted after a fatal scan, storage or motion error.

Host checks exercise longer steps, the near-BLE hint, three-of-four voting,
recovery at the tolerance boundary, the recorded v16 false recovery, recent
strong regions, the exploration cap, fine peak selection and verification,
STOP/faults/arrival at each return stage, shortened blocks, missing Wi-Fi,
both budgets, startup and the existing forward/side/rear ideal route models.
The old v16 test sequences remain available at the preserved source commit.
Old measured paths are not replayed as if they were measurements from the new,
longer steps. New route continuations are simulations.

No v17 physical run has been recorded yet. Durations are not distances: measure
500/750 ms forward and 125 ms fine movement on the assembled robot. Confirm the
v17 banner and export its GP21 log after testing. A strong Wi-Fi region does not
establish the AP's exact position or prove repeatable return accuracy.

Useful new log records are `REGION_POINT`, `REGION`, `REASSESS`, `RECHECK`,
`REGION RETURN`, `REFINE_POINT` and `REFINE`. The fine records are persisted
separately and do not consume the normal line-profile buffer.

## Build

Use Pico SDK 2.3.0 and the existing ARM toolchain. For a configured root build:

```powershell
cmake --build build --target getting_there --parallel 6
```

The root build produces `build/getting_there/getting_there_v17_experimental.uf2`.
A standalone build in `getting_there/build` produces the same filename there.
The supplied UF2 was copied to `getting_there/build/getting_there_v17_experimental.uf2`.

Run the host checks with the installed compiler and SDK paths:

```powershell
python getting_there/tests/run_host_checks.py --cc gcc --sdk 'C:/pico-sdk/2.3.0'
```

To return to the working source after saving any local changes, use
`git switch main`. To restore only the working firmware, flash the retained
`getting_there/build/getting_there.uf2` and confirm its v16 banner.
