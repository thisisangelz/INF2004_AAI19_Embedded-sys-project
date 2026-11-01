param(
    [ValidateSet('quick', 'full', 'hardware-report')]
    [string]$Mode = 'quick',
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$RemainingArgs
)

$candidates = @()
foreach ($name in @('python3', 'python')) {
    $command = Get-Command $name -ErrorAction SilentlyContinue
    if ($command) { $candidates += $command.Source }
}
$installRoot = if ($env:PICO_INSTALL_ROOT) { $env:PICO_INSTALL_ROOT } else { 'C:\Program Files\Raspberry Pi\Pico SDK v1.5.1' }
$candidates += Join-Path $installRoot 'python\python.exe'

$runner = Join-Path $PSScriptRoot 'run.py'
foreach ($candidate in $candidates | Select-Object -Unique) {
    if (-not (Test-Path -LiteralPath $candidate)) { continue }
    try {
        & $candidate --version *> $null
        if ($LASTEXITCODE -ne 0) { continue }
        & $candidate $runner $Mode @RemainingArgs
        exit $LASTEXITCODE
    } catch {
        continue
    }
}

Write-Error 'No working Python 3 interpreter found. Install Python 3 or set PICO_INSTALL_ROOT to the Pico SDK installer directory.'
exit 2

