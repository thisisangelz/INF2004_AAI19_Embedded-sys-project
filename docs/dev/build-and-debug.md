# Build and debug (Windows, Pico SDK 2.3.1)

The reproducible firmware baseline is Pico SDK 2.3.1 with Arm GNU Toolchain
15.2.Rel1. The Raspberry Pi Pico VS Code extension installs these under
`%USERPROFILE%\.pico-sdk\`. Native host tests require a separate native GCC or
Clang; MSYS2 UCRT64 GCC is the preferred Windows installation.

```powershell
. .\scripts\dev\env.ps1                       # PATH + PICO_SDK_PATH, installs nothing
.\scripts\dev\build.ps1 getting_there         # primary rover; also: rssi_env_test, wheel_rssi_test, wheel_turn_check, .
.\scripts\dev\host_tests.ps1                  # compatibility wrapper for quick QA
.\scripts\qa\run.ps1 quick                    # shared local/CI QA gate
```

Development builds default to `artifacts/dev-build/<target>` and QA builds to
`artifacts/qa/build/`. Both are gitignored. The scripts never flash. Debugging
and physical validation require separately approved hardware procedures.

On Windows, native host executables default to
`%TEMP%\inf2004-firmware-qa-host` because Application Control can block newly
linked unsigned executables inside a OneDrive workspace. Override this with
`QA_HOST_BUILD_DIR` or `--host-build-dir`. Reports and coverage output remain
under `artifacts/qa/`.

## Open items
- Install native GCC (prefer MSYS2 UCRT64) or set `QA_HOST_CC`/`--cc` to a
  native compiler. The ARM cross-compiler is intentionally rejected for host tests.
- Cppcheck, Clang-Tidy and Gcovr are advisory until their baselines are accepted.

## Proposed incremental architecture (no rewrite)
1. Split `getting_there.c` into wifi_rssi, buzzer, app. 2. Keep `motion.c` the sole motor-PWM writer; separate IMU/turn logic from actuation.
3. Reading structs `{value, valid, timestamp_ms}` for new sensors. 4. Explicit radio acquire/release (shared CYW43 radio).
5. STOP/FAULT latch stays in `faults.c`; never auto-cleared.
