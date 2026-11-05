# Dot-source:  . .\scripts\dev\env.ps1
# Discovers the pinned Pico SDK 2.3.1 environment and a native host compiler. Installs nothing.

$picoSdkBaseline = '2.3.1'
$picoRepoRoot = (Resolve-Path "$PSScriptRoot\..\..").Path
$picoUserRoot = Join-Path $env:USERPROFILE '.pico-sdk'

$sdkCandidates = @(
    $env:PICO_SDK_PATH,
    (Join-Path $picoUserRoot "sdk\$picoSdkBaseline"),
    "C:\Program Files\Raspberry Pi\Pico SDK v$picoSdkBaseline\pico-sdk",
    'C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\pico-sdk'
) | Where-Object { $_ }
$picoSdk = $sdkCandidates | Where-Object {
    Test-Path -LiteralPath (Join-Path $_ 'pico_sdk_init.cmake')
} | Select-Object -First 1
if (-not $picoSdk) {
    Write-Error 'Pico SDK not found. Set PICO_SDK_PATH to a Pico SDK 2.3.1 checkout.'
    return
}
$env:PICO_SDK_PATH = (Resolve-Path -LiteralPath $picoSdk).Path
$installedPicotool = Join-Path $picoUserRoot 'picotool\2.3.1\picotool'
if (Test-Path -LiteralPath (Join-Path $installedPicotool 'picotoolConfig.cmake')) {
    $env:QA_PICOTOOL_DIR = $installedPicotool
}
$installedPioasm = Join-Path $picoUserRoot 'tools\2.3.1\pioasm'
if (Test-Path -LiteralPath (Join-Path $installedPioasm 'pioasmConfig.cmake')) {
    $env:QA_PIOASM_DIR = $installedPioasm
}

$toolchainCandidates = @(
    $env:PICO_TOOLCHAIN_PATH,
    (Join-Path $picoUserRoot 'toolchain\15_2_Rel1'),
    'C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\gcc-arm-none-eabi'
) | Where-Object { $_ }
$picoToolchain = $toolchainCandidates | Where-Object {
    Test-Path -LiteralPath (Join-Path $_ 'bin\arm-none-eabi-gcc.exe')
} | Select-Object -First 1
if ($picoToolchain) {
    $env:PICO_TOOLCHAIN_PATH = (Resolve-Path -LiteralPath $picoToolchain).Path
}

$hostCandidates = @(
    $env:QA_HOST_CC,
    'C:\msys64\ucrt64\bin\gcc.exe',
    'C:\msys64\mingw64\bin\gcc.exe',
    'C:\Program Files\LLVM\bin\clang.exe'
) | Where-Object { $_ }
$hostCompiler = $hostCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
if (-not $hostCompiler) {
    $wingetRoot = Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Packages'
    if (Test-Path -LiteralPath $wingetRoot) {
        $hostCompiler = Get-ChildItem -LiteralPath $wingetRoot -Directory -Filter '*WinLibs*' -ErrorAction SilentlyContinue |
            ForEach-Object { Get-ChildItem -LiteralPath $_.FullName -Recurse -Filter gcc.exe -ErrorAction SilentlyContinue } |
            Select-Object -First 1 -ExpandProperty FullName
    }
}
if ($hostCompiler) {
    $env:QA_HOST_CC = (Resolve-Path -LiteralPath $hostCompiler).Path
}

$pathEntries = @()
if ($picoToolchain) { $pathEntries += Join-Path $picoToolchain 'bin' }
$pathEntries += @(
    (Join-Path $picoUserRoot 'cmake\v4.3.4\bin'),
    (Join-Path $picoUserRoot 'ninja\v1.13.2')
)
if ($hostCompiler) { $pathEntries += Split-Path -Parent $hostCompiler }
$pathEntries = $pathEntries | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -Unique
$env:PATH = ($pathEntries -join ';') + ';' + $env:PATH
$env:PICO_BUILD_ROOT = if ($env:PICO_BUILD_ROOT) {
    $env:PICO_BUILD_ROOT
} else {
    Join-Path $picoRepoRoot 'artifacts\dev-build'
}

"PICO_SDK_PATH=$env:PICO_SDK_PATH"
"PICO_TOOLCHAIN_PATH=$($env:PICO_TOOLCHAIN_PATH)"
"QA_PICOTOOL_DIR=$($env:QA_PICOTOOL_DIR)"
"QA_PIOASM_DIR=$($env:QA_PIOASM_DIR)"
"QA_HOST_CC=$($env:QA_HOST_CC)"
"PICO_BUILD_ROOT=$env:PICO_BUILD_ROOT"
