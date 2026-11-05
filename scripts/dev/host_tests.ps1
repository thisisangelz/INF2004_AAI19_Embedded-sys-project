# Compatibility wrapper for the shared local/CI quick QA entry point.
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$RemainingArgs
)

& "$PSScriptRoot\..\qa\run.ps1" quick @RemainingArgs
exit $LASTEXITCODE
