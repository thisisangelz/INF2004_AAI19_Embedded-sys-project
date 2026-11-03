# Development history

This index records the branch endpoints checked on 3 November 2026. Source links
use exact commit IDs so they keep showing the same version when a branch moves.
Dates below are commit dates, not hardware test or submission dates.

Use `main` for the maintained programme. Keep branches for unfinished work. Use
annotated tags named `archive/<milestone>` for completed versions, and create a
GitHub Release only when a version has firmware files and recorded test results
worth distributing. Deleting a branch after its commits are preserved does not
erase those commits or its existing pull request.

## Main development sequence

| Date | Earlier branch or milestone | Source snapshot | Purpose |
|---|---|---|---|
| 8 September | `pico-beacon-proximity-signal-testing` | [a66c856](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/a66c85651202ed75269255e2edf5bc3d9c2fc64b) | Initial beacon and scanner test; separate history, not merged into `main`. |
| 11 September | `beacon-finder-v1` | [485d899](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/485d8992727a427d49b5211b6ab3b71ab4f06fd6) | First beacon-finder firmware checkpoint. |
| 22 September | `beacon-finder-v2`, published endpoint | [08557b5](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/08557b5872f40e489b00ebdd6c7d6eb21172e0e9) | Three beacons, BLE transfers and advertising diagnostics. |
| 22 September | `beacon-finder-v2`, local endpoint | [14337c9](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/14337c91d81721f77ae7085ca04c43b97533973b) | Advertising fix, already preserved in `main` despite the older remote branch endpoint. |
| 29 September | `rssi` | [276450f](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/276450fc0332b3af7550c8d675a167210f4aec86) | RSSI smoothing; merged through [PR #1](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/pull/1). |
| 29 September | `first-steps` | [4503624](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/4503624b8cca5359f728d76296953648437823d5) | Gyro, wheel and RSSI diagnostics. |
| 6 October | `first-beacon` | [cb68630](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/cb686301ffbf2e702311fa92926d73f20ad905db) | Single-beacon movement and gyro corrections. |
| 6 October | `getting-there`, published endpoint | [a29b521](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/a29b521667fc2f9d91d30e17f32553ec8a8037c6) | AP1 approach, verified payload and completion tone; merged through [PR #2](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/pull/2) on 18 October. |
| 20 October | `Alvin_2` | [2012e7b](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/2012e7baac6ea527235ba982b2f338fe38e6cdc8) | Navigation, saved run logs and gyro calibration after GP20. |
| 28 October | `line-search` and `main` at the review | [c5e413a](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/c5e413a7f885120e6bc68a35488583184bec0533) | AP1 line search with two perpendicular checks. |

The uncommitted working files can contain later changes than these snapshots.
This table does not claim that a commit has passed hardware testing.

## Work outside main

| Branch | Endpoint at the review | Commits absent from `main` | Treatment |
|---|---|---:|---|
| Formerly local `getting-there` | [672e98d](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/archive/getting-there-local) | 3 | Preserved by `archive/getting-there-local`; these changes have not been merged into `main`. |
| `robot-buzzer-cues-flash-log` | [aa15c6c](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/aa15c6ceb3a554dde2ed661bd5928d683bba3762) | 3 | Preserved by `archive/robot-buzzer-cues-flash-log` for later review. |
| `Alvin` | [e84d0f9](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/e84d0f9522fe06b8afcf148b7870808d232c8c85) | 1 | Preserved by `archive/alvin` and the retained `dev/tooling-setup` branch. |
| `dev/tooling-setup` | [f09e31f](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/f09e31f7560010c4002a6fd17b64c0552480bc46) | 5 | Keep while tooling and diagnostic additions remain under review. |
| `pico-beacon-proximity-signal-testing` | [a66c856](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/a66c85651202ed75269255e2edf5bc3d9c2fc64b) | 1 | Preserved by `archive/pico-beacon-proximity-signal-testing`, including its separate history. |

The local `getting-there` additions are `2c5373b`, `da756e5` and `672e98d`.
They are not a replacement for all changes on `main`: at this review, `main`
also has ten commits absent from that local branch.

## Archive tags

The cleanup preserves the following exact endpoints as annotated tags. Archive
names are permanent references to earlier work; their presence does not mean
that every archived change was merged or passed hardware tests.

`main` and `dev/tooling-setup` are the retained branches. The tooling branch has
five commits absent from the pre-cleanup `main` and remains available for review.
The old branches listed below are retired only after the corresponding tags
have been pushed and their commit IDs verified on GitHub.

| Archive tag | Commit | Previous branch or checkpoint |
|---|---|---|
| [archive/pico-beacon-proximity-signal-testing](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/archive/pico-beacon-proximity-signal-testing) | `a66c856` | pico-beacon-proximity-signal-testing |
| [archive/beacon-finder-v1](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/archive/beacon-finder-v1) | `485d899` | beacon-finder-v1 |
| [archive/beacon-finder-v2-published](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/archive/beacon-finder-v2-published) | `08557b5` | beacon-finder-v2 |
| [archive/beacon-finder-v2-local](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/archive/beacon-finder-v2-local) | `14337c9` | Local beacon-finder-v2 |
| [archive/rssi](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/archive/rssi) | `276450f` | rssi |
| [archive/first-steps](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/archive/first-steps) | `4503624` | first-steps |
| [archive/first-beacon](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/archive/first-beacon) | `cb68630` | first-beacon |
| [archive/getting-there-published](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/archive/getting-there-published) | `a29b521` | getting-there |
| [archive/getting-there-local](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/archive/getting-there-local) | `672e98d` | Local getting-there, including three unpublished commits |
| [archive/alvin-2](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/archive/alvin-2) | `2012e7b` | Alvin_2 |
| [archive/robot-buzzer-cues-flash-log](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/archive/robot-buzzer-cues-flash-log) | `aa15c6c` | robot-buzzer-cues-flash-log |
| [archive/alvin](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/archive/alvin) | `e84d0f9` | Alvin |
| [archive/line-search](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/archive/line-search) | `c5e413a` | line-search |
| [archive/main-before-cleanup-2026-11-03](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/archive/main-before-cleanup-2026-11-03) | `c5e413a` | main before the folder cleanup |

The local `getting-there` tag preserves all three previously unpublished
commits: `2c5373b`, `da756e5` and `672e98d`. The published branch endpoint is
preserved separately, so the original merge and the later local work can both
be inspected. The two v2 tags likewise preserve its different local and remote
endpoints.

Browse [GitHub Tags](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tags)
or inspect a snapshot without changing the current files:

```sh
git show archive/getting-there-local:getting_there/getting_there.c
git log archive/getting-there-published..archive/getting-there-local
```

In a clean checkout, an archived version can become a new working branch:

```sh
git switch -c review/getting-there archive/getting-there-local
```

For later cleanups, refresh remote references, create an annotated archive tag
at each exact branch endpoint, push it and verify it on the remote before
removing the branch name. Check for new remote commits immediately before
removal. Preserve unmerged experiments in the same way; do not rewrite the
existing commit history to make the branch list shorter.

## Folder cleanup

The current firmware folders are `getting_there` and the three powerbank beacon
targets. The shared BLE definitions and powerbank server remain in `ble_transfer`
and `ble_transfer_powerbank`. The robot's IMU driver and network configuration
have moved unchanged into `getting_there`, removing its dependency on the old tracker.

The old diagnostics are available in the pre-cleanup source snapshot:

| Programme | Source and instructions |
|---|---|
| Three-beacon RSSI and BLE tracker | [pico_rssi_tracker](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/c5e413a7f885120e6bc68a35488583184bec0533/pico_rssi_tracker) |
| Single-beacon movement and RSSI test | [wheel_rssi_test](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/c5e413a7f885120e6bc68a35488583184bec0533/wheel_rssi_test) |
| Gyro-controlled turn test | [wheel_turn_check](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/c5e413a7f885120e6bc68a35488583184bec0533/wheel_turn_check) |

The superseded `picow_access_point`, `picow_access_point2` and
`picow_access_point3` folders and the old `ble_transfer/beacon_server.c` are
removed from the current files. Their tracked source remains in the
[pre-cleanup snapshot](https://github.com/thisisangelz/INF2004_AAI19_Embedded-sys-project/tree/c5e413a7f885120e6bc68a35488583184bec0533).
The retained powerbank server uses the same protocol, GATT definition and
BTstack configuration as the robot, avoiding duplicate copies.

The local cleanup also preserved the old folders, including their untracked
build files, under `.local-archive/2026-11-03-access-points/`. The old tracker and
wheel programmes are backed up under `.local-archive/2026-11-03-diagnostics/`.
SHA-256 manifests record every moved file. These ignored local backups are not
published to GitHub.
Stashed files, other untracked source, assets and tribunal documents still need
their own deliberate backup or commit; branch tags do not preserve them.
