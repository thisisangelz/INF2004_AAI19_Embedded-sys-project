# Build and debug (Windows, Pico SDK 1.5.1)

Inventory: SDK, ARM GCC 10.3.1, Ninja, CMake 3.25, picotool and OpenOCD live under
`C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\`. Host gcc: WinLibs (winget). No Pico VS Code extension.
`scripts/qa/targets.json` names SDK 2.3.0; the local SDK is 1.5.1 (see open items).

```powershell
. .\scripts\dev\env.ps1                       # PATH + PICO_SDK_PATH, installs nothing
.\scripts\dev\build.ps1 getting_there         # primary rover; also: rssi_env_test, wheel_rssi_test, wheel_turn_check, .
.\scripts\dev\host_tests.ps1                  # existing host tests
python scripts/qa/run.py quick                # shared QA gate (owned by Codex)
```

Builds go to `C:\tmp_pico\<target>` (deep paths such as the Claude scratchpad break the compiler check) and
refresh `compile_commands.json` (gitignored) for clangd. The scripts never flash. Manual flash: hold BOOTSEL,
copy the `.uf2` from the build dir. Debug: `openocd` + `picotool` from the env script need a debug probe (none assumed).
Serial: USB CDC (`pico_enable_stdio_usb`).

## Open items
- Root `CMakeLists.txt` omits `getting_there`, `rssi_env_test`, `wheel_*`; propose adding them.
- Windows Application Control blocks running the unsigned host test `.exe` here (WinError 4551): host tests compile but cannot execute on this PC until the policy allows it or they run in CI/another machine.
- `.vscode/settings.json` has a teammate's absolute paths; left unchanged.
- SDK 1.5.1 vs 2.3.0 mismatch with QA config.

## Proposed incremental architecture (no rewrite)
1. Split `getting_there.c` into wifi_rssi, buzzer, app. 2. Keep `motion.c` the sole motor-PWM writer; separate IMU/turn logic from actuation.
3. Reading structs `{value, valid, timestamp_ms}` for new sensors. 4. Explicit radio acquire/release (shared CYW43 radio).
5. STOP/FAULT latch stays in `faults.c`; never auto-cleared.
