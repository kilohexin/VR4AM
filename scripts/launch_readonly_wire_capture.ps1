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
    [switch]$Execute,
    [switch]$MotionStop
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
$record = @{ started_utc=[DateTime]::UtcNow.ToString('o'); mode=$(if ($MotionStop) {
    if ($Execute) { 'approved_motion_stop' } else { 'motion_stop_offline_preview' }
} elseif ($Execute) { 'approved_readonly' } else { 'offline_preview' }) }
$exitCode = 1
try {
    $python = Join-Path $repoRoot '.venv/Scripts/python.exe'
    function Get-LocalRpcCount {
        $probeOutput = & $python -X utf8 (Join-Path $PSScriptRoot 'local_rpc_connections.py')
        if ($LASTEXITCODE -ne 0) { throw 'LOCAL_CONNECTION_TABLE_UNAVAILABLE' }
        $count = ($probeOutput -join "`n") | ConvertFrom-Json
        if ($count -isnot [int] -or $count -lt 0) { throw 'INVALID_LOCAL_CONNECTION_COUNT' }
        return $count
    }
    $entry = Join-Path $PSScriptRoot $(if ($MotionStop) { 'motion_stop_wire_capture.py' } else { 'readonly_wire_capture.py' })
    $arguments = @('-X','utf8',$entry,'--config',[IO.Path]::GetFullPath($Config),
                   '--output',$capturePath,'--component-id',[string]$ComponentId)
    if ($ToolBaseline) {
        $arguments += @('--tool-baseline',[IO.Path]::GetFullPath($ToolBaseline))
    }
    if ($Execute) {
        $requiredConfirm = if ($MotionStop) { 'I_APPROVE_ONE_Z3MM_MOTION_STOP_WIRE_CAPTURE' } else { 'I_APPROVE_ONE_READONLY_WIRE_CAPTURE' }
        if ($Confirm -ne $requiredConfirm) { throw 'ONSITE_SCOPE_CONFIRMATION_REQUIRED' }
        if (!$ToolBaseline -or !$ExpectedConfigSha256 -or !$ApprovedPreview -or
            !$ApprovedPreviewSha256 -or !$AuthorizationExpiresUtc) { throw 'FROZEN_APPROVAL_ARTIFACTS_REQUIRED' }
        $arguments += @('--execute','--confirm',$Confirm,
                        '--expected-config-sha256',$ExpectedConfigSha256,
                        '--approved-preview',[IO.Path]::GetFullPath($ApprovedPreview),
                        '--approved-preview-sha256',$ApprovedPreviewSha256,
                        '--authorization-expires-utc',$AuthorizationExpiresUtc)
        $record.existing_rpc_connections = Get-LocalRpcCount
        if ($record.existing_rpc_connections -ne 0) { throw 'EXISTING_RPC_CLIENT_ABORT' }
    }
    foreach ($argument in $arguments) {
        if ($argument.Contains('"') -or $argument.Contains("`n") -or $argument.Contains("`r") -or $argument.EndsWith('\')) {
            throw 'UNSUPPORTED_NATIVE_ARGUMENT'
        }
    }
    $quoted = @($arguments | ForEach-Object { '"' + $_ + '"' })
    $launchOptions = @{ FilePath=$python; ArgumentList=$quoted; WindowStyle='Hidden'; PassThru=$true;
        RedirectStandardOutput=(Join-Path $recordPath 'stdout.txt'); RedirectStandardError=(Join-Path $recordPath 'stderr.txt') }
    if ($MotionStop -and $Execute) {
        function Publish-MotionAttention {
            $signal = Join-Path $capturePath 'attention-required.json'
            $printed = $record.operator_attention_required -eq $true
            # File presence is conservative: no need to wait for JSON parsing.
            # Also forward the ASCII token if creating the signal file failed.
            $alert = Test-Path -LiteralPath $signal
            if (!$alert -and !$printed) {
                try { $alert = [IO.File]::ReadAllText((Join-Path $recordPath 'stdout.txt')).Contains('VR4ARM_STOP_UNKNOWN') } catch {}
            }
            if ($alert -and !$printed) {
                $record.operator_attention_required = $true
                [Console]::Out.WriteLine('VR4ARM_STOP_UNKNOWN: Robot stopping is unverified. Inspect the site and follow the onsite safety procedure. Do not retry.')
                [Console]::Out.Flush()
            }
        }
        $child = Start-Process @launchOptions
        # Retain a process handle before Refresh sees an exited process;
        # Windows PowerShell otherwise may lose its native exit code.
        $retainedHandle = $child.Handle
        do {
            Publish-MotionAttention
            Start-Sleep -Milliseconds 100
            $child.Refresh()
        } while (!$child.HasExited)
        # Explicitly wait after observing exit so redirected output/ExitCode
        # are settled; never kill or terminate a motion client.
        $child.WaitForExit()
        Publish-MotionAttention
    } else {
        # Preview and readonly keep the original synchronous exit handling.
        $child = Start-Process @launchOptions -Wait
    }
    $record.child_pid = $child.Id
    $record.python_exit = $child.ExitCode
    $record.process_exited = $child.HasExited
    if ($null -eq $child.ExitCode) { throw 'MISSING_CHILD_EXIT_CODE' }
    $exitCode = $child.ExitCode
    if ($Execute) {
        $record.rpc_connections_after_status = 'unknown'
        $record.rpc_connections_after = Get-LocalRpcCount
        $record.rpc_connections_after_status = 'readable'
    }
} catch {
    $record.error_type = $_.Exception.GetType().Name
    if ($exitCode -eq 0) { $exitCode = 1 }
}
finally {
    $record.launcher_exit = $exitCode
    $record.completed_utc = [DateTime]::UtcNow.ToString('o')
    $record | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $recordPath 'launch.json') -Encoding UTF8
}
exit $exitCode
