# Dot-source:  . .\scripts\dev\env.ps1
# Points this shell at the already-installed Pico SDK toolchain and host gcc. Installs nothing.
$sdkRoot = if ($env:PICO_INSTALL_ROOT) { $env:PICO_INSTALL_ROOT } else { 'C:\Program Files\Raspberry Pi\Pico SDK v1.5.1' }
if (-not (Test-Path "$sdkRoot\pico-sdk")) { Write-Error "Pico SDK not found at $sdkRoot (set PICO_INSTALL_ROOT)"; return }
$env:PICO_SDK_PATH = "$sdkRoot\pico-sdk"
$dirs = @("$sdkRoot\gcc-arm-none-eabi\bin", "$sdkRoot\ninja", "$sdkRoot\cmake\bin", "$sdkRoot\picotool", "$sdkRoot\openocd")
$gcc = Get-ChildItem "$env:LOCALAPPDATA\Microsoft\WinGet\Packages" -Recurse -Filter gcc.exe -ErrorAction SilentlyContinue | Select-Object -First 1
if ($gcc) { $dirs += $gcc.DirectoryName }
$env:PATH = ($dirs -join ';') + ';' + $env:PATH
$env:PICO_BUILD_ROOT = if ($env:PICO_BUILD_ROOT) { $env:PICO_BUILD_ROOT } else { 'C:\tmp_pico' }  # short path: deep paths break the Windows compiler check
"PICO_SDK_PATH=$env:PICO_SDK_PATH  build root=$env:PICO_BUILD_ROOT  host gcc=$([bool]$gcc)"
