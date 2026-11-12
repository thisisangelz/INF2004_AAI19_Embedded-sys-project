# Firmware QA report

Mode: `full`

Repository: `dev/tooling-setup` at `f09e31f7560010c4002a6fd17b64c0552480bc46` (dirty worktree)

Physical 20 ms PWM-off requirement: **HARDWARE_UNVERIFIED**

| Check | Class | Evidence | Status | Tool | Duration |
|---|---|---|---:|---|---:|
| QA tooling regression tests | mandatory | HOST_VERIFIED | PASS | 3.14.7 (tags/v3.14.7:823f032, Aug  5 2026, 10:51:32) [MSC v.1944 64 bit (AMD64)] [C:\Users\alvin\AppData\Local\Python\pythoncore-3.14-64\python.exe] | 0.101s |
| Pico SDK baseline | mandatory | SOURCE_CONFIRMED | PASS | 2.3.1 [C:\Users\alvin\.pico-sdk\sdk\2.3.1] | 0.000s |
| host test build: packed-bit fields | mandatory | HOST_VERIFIED | PASS | gcc.exe (Rev4, Built by MSYS2 project) 16.2.0 [C:\msys64\ucrt64\bin\gcc.exe] | 0.198s |
| host test run: packed-bit fields | mandatory | HOST_VERIFIED | PASS | gcc.exe (Rev4, Built by MSYS2 project) 16.2.0 [C:\msys64\ucrt64\bin\gcc.exe] | 2.800s |
| host test build: transfer protocol | mandatory | HOST_VERIFIED | PASS | gcc.exe (Rev4, Built by MSYS2 project) 16.2.0 [C:\msys64\ucrt64\bin\gcc.exe] | 0.195s |
| host test run: transfer protocol | mandatory | HOST_VERIFIED | PASS | gcc.exe (Rev4, Built by MSYS2 project) 16.2.0 [C:\msys64\ucrt64\bin\gcc.exe] | 1.510s |
| host test build: shared state | mandatory | HOST_VERIFIED | PASS | gcc.exe (Rev4, Built by MSYS2 project) 16.2.0 [C:\msys64\ucrt64\bin\gcc.exe] | 0.193s |
| host test run: shared state | mandatory | HOST_VERIFIED | PASS | gcc.exe (Rev4, Built by MSYS2 project) 16.2.0 [C:\msys64\ucrt64\bin\gcc.exe] | 2.684s |
| host test build: IMU processing | mandatory | HOST_VERIFIED | PASS | gcc.exe (Rev4, Built by MSYS2 project) 16.2.0 [C:\msys64\ucrt64\bin\gcc.exe] | 0.214s |
| host test run: IMU processing | mandatory | HOST_VERIFIED | PASS | gcc.exe (Rev4, Built by MSYS2 project) 16.2.0 [C:\msys64\ucrt64\bin\gcc.exe] | 2.643s |
| host test build: motor safety | mandatory | HOST_VERIFIED | PASS | gcc.exe (Rev4, Built by MSYS2 project) 16.2.0 [C:\msys64\ucrt64\bin\gcc.exe] | 0.248s |
| host test run: motor safety | mandatory | HOST_VERIFIED | PASS | gcc.exe (Rev4, Built by MSYS2 project) 16.2.0 [C:\msys64\ucrt64\bin\gcc.exe] | 2.891s |
| existing host test: navigation | mandatory | HOST_VERIFIED | PASS | gcc.exe (Rev4, Built by MSYS2 project) 16.2.0 [C:\msys64\ucrt64\bin\gcc.exe] | 3.120s |
| existing host test: BLE payload protocol | mandatory | HOST_VERIFIED | PASS | gcc.exe (Rev4, Built by MSYS2 project) 16.2.0 [C:\msys64\ucrt64\bin\gcc.exe] | 2.924s |
| configure build group: root-firmware | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 5.042s |
| firmware build: pico_rssi_tracker | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 0.127s |
| firmware artifacts: pico_rssi_tracker | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 0.000s |
| firmware build: picow_access_point | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 0.135s |
| firmware artifacts: picow_access_point | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 0.000s |
| firmware build: picow_access_point2 | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 0.128s |
| firmware artifacts: picow_access_point2 | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 0.000s |
| firmware build: picow_access_point3 | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 0.130s |
| firmware artifacts: picow_access_point3 | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 0.000s |
| firmware build: tcs34725_diag | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 0.119s |
| firmware artifacts: tcs34725_diag | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 0.000s |
| configure build group: getting-there | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 4.215s |
| firmware build: getting_there | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 48.194s |
| firmware artifacts: getting_there | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 0.000s |
| configure build group: rssi-environment | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 4.287s |
| firmware build: rssi_env_test | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 32.424s |
| firmware artifacts: rssi_env_test | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 0.000s |
| configure build group: wheel-rssi | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 3.926s |
| firmware build: wheel_rssi_test | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 31.623s |
| firmware artifacts: wheel_rssi_test | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 0.000s |
| configure build group: wheel-turn | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 3.841s |
| firmware build: wheel_turn_check | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 11.684s |
| firmware artifacts: wheel_turn_check | mandatory | FIRMWARE_COMPILE_VERIFIED | PASS | cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE] | 0.000s |
| Cppcheck advisory | advisory | SOURCE_CONFIRMED | NOT_INSTALLED | not found | 0.000s |
| Clang-Tidy advisory | advisory | SOURCE_CONFIRMED | NOT_INSTALLED | not found | 0.000s |
| Gcovr advisory | advisory | HOST_VERIFIED | NOT_INSTALLED | not found | 0.000s |

## Tool versions

- `python`: 3.14.7 (tags/v3.14.7:823f032, Aug  5 2026, 10:51:32) [MSC v.1944 64 bit (AMD64)] [C:\Users\alvin\AppData\Local\Python\pythoncore-3.14-64\python.exe]
- `platform`: Windows-11-10.0.26200-SP0
- `host_cc`: gcc.exe (Rev4, Built by MSYS2 project) 16.2.0 [C:\msys64\ucrt64\bin\gcc.exe]
- `host_target`: x86_64-w64-mingw32
- `cmake`: cmake version 4.3.4 [C:/Users/alvin/.pico-sdk/cmake/v4.3.4/bin\cmake.EXE]
- `ninja`: 1.13.2 [C:/Users/alvin/.pico-sdk/ninja/v1.13.2\ninja.EXE]
- `arm_gcc`: arm-none-eabi-gcc.exe (Arm GNU Toolchain 15.2.Rel1 (Build arm-15.86)) 15.2.1 20251203 [C:\Users\alvin\.pico-sdk\toolchain\15_2_Rel1\bin\arm-none-eabi-gcc.exe]
- `picotool`: picotool v2.3.1 (Windows, GNU-16.2.0, Release) [C:\Users\alvin\.pico-sdk\picotool\2.3.1\picotool\picotool.exe]
- `pioasm`: pioasm version: 2.3.1 [C:\Users\alvin\.pico-sdk\tools\2.3.1\pioasm\pioasm.exe]
- `pico_sdk`: 2.3.1 [C:\Users\alvin\.pico-sdk\sdk\2.3.1]
- `cppcheck`: not found
- `clang_tidy`: not found
- `gcovr`: not found
- `host_build_dir`: C:\Users\alvin\AppData\Local\Temp\inf2004-firmware-qa-host

## Non-passing checks

### Cppcheck advisory (NOT_INSTALLED)

Command: not run

```text
cppcheck not installed; set --cppcheck or QA_CPPCHECK
```

### Clang-Tidy advisory (NOT_INSTALLED)

Command: not run

```text
clang-tidy not installed; set --clang-tidy or QA_CLANG_TIDY
```

### Gcovr advisory (NOT_INSTALLED)

Command: not run

```text
gcovr not installed; set --gcovr or QA_GCOVR
```
