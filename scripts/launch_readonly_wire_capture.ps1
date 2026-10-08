# Default: offline preview. Elevation and onsite approval are supplied by operator.
[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$Config,
    [Parameter(Mandatory=$true)][string]$Output,
    [Parameter(Mandatory=$true)][int]$ComponentId,
    [Parameter(Mandatory=$true)][string]$RecordDirectory,
    [string]$ToolBaseline,
    [string]$ExpectedConfigSha256,
    [string]$ApprovedPreview,
    [string]$ApprovedPreviewSha256,
    [string]$AuthorizationExpiresUtc,
    [string]$Confirm,
    [switch]$Execute
)
$ErrorActionPreference = 'Stop'
$OutputEncoding = [Console]::OutputEncoding = New-Object System.Text.UTF8Encoding($false)
$env:PYTHONUTF8 = '1'
$repoRoot = Split-Path $PSScriptRoot -Parent
$tmpPrefix = [IO.Path]::GetFullPath((Join-Path $repoRoot '.tmp')).TrimEnd('\','/') + [IO.Path]::DirectorySeparatorChar
$recordPath = [IO.Path]::GetFullPath($RecordDirectory).TrimEnd('\','/')
$capturePath = [IO.Path]::GetFullPath($Output).TrimEnd('\','/')
if (!$recordPath.StartsWith($tmpPrefix,[StringComparison]::OrdinalIgnoreCase) -or
    !$capturePath.StartsWith($tmpPrefix,[StringComparison]::OrdinalIgnoreCase)) {
    throw 'OUTPUTS_MUST_BE_UNDER_REPO_TMP'
}
if ($recordPath -eq $capturePath -or $recordPath.StartsWith(($capturePath + '\'),[StringComparison]::OrdinalIgnoreCase)) {
    throw 'LAUNCH_RECORD_MUST_BE_SEPARATE_FROM_CAPTURE'
}
if (Test-Path -LiteralPath $recordPath) { throw 'REFUSE_EXISTING_LAUNCH_RECORD' }
New-Item -ItemType Directory -Path $recordPath | Out-Null
$record = @{ started_utc=[DateTime]::UtcNow.ToString('o'); mode=$(if ($Execute) { 'approved_readonly' } else { 'offline_preview' }) }
$exitCode = 1
try {
    $python = Join-Path $repoRoot '.venv/Scripts/python.exe'
    $entry = Join-Path $PSScriptRoot 'readonly_wire_capture.py'
    $arguments = @('-X','utf8',$entry,'--config',[IO.Path]::GetFullPath($Config),
                   '--output',$capturePath,'--component-id',[string]$ComponentId)
    if ($Execute) {
        if ($Confirm -ne 'I_APPROVE_ONE_READONLY_WIRE_CAPTURE') { throw 'ONSITE_SCOPE_CONFIRMATION_REQUIRED' }
        if (!$ToolBaseline -or !$ExpectedConfigSha256 -or !$ApprovedPreview -or
            !$ApprovedPreviewSha256 -or !$AuthorizationExpiresUtc) { throw 'FROZEN_APPROVAL_ARTIFACTS_REQUIRED' }
        $arguments += @('--execute','--confirm',$Confirm,'--tool-baseline',[IO.Path]::GetFullPath($ToolBaseline),
                        '--expected-config-sha256',$ExpectedConfigSha256,
                        '--approved-preview',[IO.Path]::GetFullPath($ApprovedPreview),
                        '--approved-preview-sha256',$ApprovedPreviewSha256,
                        '--authorization-expires-utc',$AuthorizationExpiresUtc)
        $existing = @(Get-NetTCPConnection -ErrorAction Stop | Where-Object { $_.RemotePort -eq 3031 -and $_.State -eq 'Established' })
        $record.existing_rpc_connections = $existing.Count
        if ($existing.Count -ne 0) { throw 'EXISTING_RPC_CLIENT_ABORT' }
    }
    foreach ($argument in $arguments) {
        if ($argument.Contains('"') -or $argument.Contains("`n") -or $argument.Contains("`r") -or $argument.EndsWith('\')) {
            throw 'UNSUPPORTED_NATIVE_ARGUMENT'
        }
    }
    $quoted = @($arguments | ForEach-Object { '"' + $_ + '"' })
    # -Wait retains ExitCode; native stderr is a file, not a PowerShell error.
    $child = Start-Process -FilePath $python -ArgumentList $quoted -WindowStyle Hidden -PassThru -Wait -RedirectStandardOutput (Join-Path $recordPath 'stdout.txt') -RedirectStandardError (Join-Path $recordPath 'stderr.txt')
    $record.child_pid = $child.Id
    $record.python_exit = $child.ExitCode
    $record.process_exited = $child.HasExited
    if ($null -eq $child.ExitCode) { throw 'MISSING_CHILD_EXIT_CODE' }
    $exitCode = $child.ExitCode
    if ($Execute) {
        $record.rpc_connections_after = @(Get-NetTCPConnection -ErrorAction Stop | Where-Object { $_.RemotePort -eq 3031 -and $_.State -eq 'Established' }).Count
    }
} catch { $record.error_type = $_.Exception.GetType().Name }
finally {
    $record.completed_utc = [DateTime]::UtcNow.ToString('o')
    $record | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $recordPath 'launch.json') -Encoding UTF8
}
exit $exitCode
