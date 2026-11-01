# Usage: .\scripts\dev\build.ps1 <getting_there|rssi_env_test|wheel_rssi_test|wheel_turn_check|.>
# Out-of-tree build (never touches the committed build/*.uf2). Never flashes.
param([Parameter(Mandatory)][string]$Target)
. "$PSScriptRoot\env.ps1"
$repo = Resolve-Path "$PSScriptRoot\..\.."
$name = if ($Target -eq '.') { 'root' } else { $Target }
$out = Join-Path $env:PICO_BUILD_ROOT $name
cmake -G Ninja -S (Join-Path $repo $Target) -B $out "-DPICO_SDK_PATH=$env:PICO_SDK_PATH" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
if ($LASTEXITCODE) { exit $LASTEXITCODE }
cmake --build $out
if ($LASTEXITCODE) { exit $LASTEXITCODE }
Copy-Item "$out\compile_commands.json" (Join-Path $repo 'compile_commands.json') -Force  # for clangd; gitignored
Get-ChildItem $out -Filter *.uf2 | ForEach-Object { "Built: $($_.FullName)  (flash manually; this script never flashes)" }
