# Runs the existing getting_there host tests (getting_there/tests/run_host_checks.py).
. "$PSScriptRoot\env.ps1"
$repo = Resolve-Path "$PSScriptRoot\..\.."
python "$repo\getting_there\tests\run_host_checks.py" --cc gcc --sdk $env:PICO_SDK_PATH --output-dir (Join-Path $env:PICO_BUILD_ROOT 'host')
exit $LASTEXITCODE
