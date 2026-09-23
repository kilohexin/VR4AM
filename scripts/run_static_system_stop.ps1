# One diagnostic only. Never dot-source. Requires separately authorized onsite use.
[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$Source,
    [Parameter(Mandatory=$true)][string]$Python,
    [Parameter(Mandatory=$true)][string]$Config,
    [Parameter(Mandatory=$true)][string]$Output,
    [Parameter(Mandatory=$true)][string]$ExpectedConfigSha256,
    [Parameter(Mandatory=$true)][string]$Confirm
)
$ErrorActionPreference = 'Stop'
if ($Confirm -cne 'I_UNDERSTAND_SINGLE_SYSTEM_STOP_MAY_MOVE') { throw 'CONFIRMATION_REQUIRED' }
$Source = (Resolve-Path -LiteralPath $Source).Path
$Python = (Resolve-Path -LiteralPath $Python).Path
$Config = (Resolve-Path -LiteralPath $Config).Path
$Output = [IO.Path]::GetFullPath($Output)
if (Test-Path -LiteralPath $Output) { throw 'OUTPUT_ALREADY_EXISTS' }
$original = [IO.File]::ReadAllBytes($Config)
$sha = [Security.Cryptography.SHA256]::Create()
try { $originalHash = -join ($sha.ComputeHash($original) | ForEach-Object { $_.ToString('x2') }) }
finally { $sha.Dispose() }
if ($originalHash -ine $ExpectedConfigSha256) { throw 'CONFIG_BASELINE_MISMATCH' }
$script = Join-Path $Source 'scripts\real_robot_static_system_stop.py'
if (-not (Test-Path -LiteralPath $script)) { throw 'DIAGNOSTIC_SCRIPT_MISSING' }
$configBuilder = Join-Path $Source 'scripts\build_static_stop_control_config.py'
if (-not (Test-Path -LiteralPath $configBuilder)) { throw 'CONFIG_BUILDER_MISSING' }
New-Item -ItemType Directory -Path ([IO.Path]::GetDirectoryName($Output)) -Force | Out-Null
New-Item -ItemType Directory -Path $Output | Out-Null
$backupPath = Join-Path $Output 'config-before.yaml'
$candidatePath = Join-Path $Output 'config-control.yaml'
[IO.File]::WriteAllBytes($backupPath, $original)
& $Python $configBuilder --original $backupPath --output $candidatePath
if ($LASTEXITCODE -ne 0) { throw 'CONTROL_CONFIG_BUILD_FAILED' }
$control = [IO.File]::ReadAllBytes($candidatePath)
$oldPath = $env:PYTHONPATH
$oldConfirm = $env:VR4ARM_REAL_ROBOT_CONFIRM
$oldLocation = Get-Location
$exitCode = $null
$restored = $false
$changed = $false
try {
    Set-Location -LiteralPath $Source
    $env:PYTHONPATH = Join-Path $Source 'backend'
    $env:VR4ARM_REAL_ROBOT_CONFIRM = 'I_UNDERSTAND_REAL_ROBOT_MOTION'
    if ([Convert]::ToBase64String([IO.File]::ReadAllBytes($Config)) -cne [Convert]::ToBase64String($original)) { throw 'CONFIG_CHANGED_BEFORE_SWITCH' }
    $changed = $true
    [IO.File]::WriteAllBytes($Config, $control)
    & $Python $script --config $Config --output (Join-Path $Output 'probe') --confirm $Confirm `
        1> (Join-Path $Output 'stdout.txt') 2> (Join-Path $Output 'stderr.txt')
    $exitCode = $LASTEXITCODE
} finally {
    try {
        $current = [Convert]::ToBase64String([IO.File]::ReadAllBytes($Config))
        if ($current -ceq [Convert]::ToBase64String($control) -and $changed) {
            [IO.File]::WriteAllBytes($Config, $original)
        }
        $restored = ([Convert]::ToBase64String([IO.File]::ReadAllBytes($Config)) -ceq [Convert]::ToBase64String($original))
    } finally {
        $env:PYTHONPATH = $oldPath
        $env:VR4ARM_REAL_ROBOT_CONFIRM = $oldConfirm
        Set-Location -LiteralPath $oldLocation.Path
        @{ child_exit_code=$exitCode; config_restored=$restored; completed_utc=[DateTime]::UtcNow.ToString('o') } |
            ConvertTo-Json | Set-Content -LiteralPath (Join-Path $Output 'execution.json') -Encoding UTF8
    }
}
if (-not $restored) { throw 'CONFIG_NOT_RESTORED_DO_NOT_RETRY_OR_OVERWRITE' }
if ($null -eq $exitCode) { throw 'CHILD_EXIT_NOT_CAPTURED' }
exit $exitCode
