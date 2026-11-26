param(
    [int]$Port = 8770,
    [string]$Python = 'python'
)
$ErrorActionPreference = 'Stop'
$tools = Join-Path (Split-Path -Parent $PSScriptRoot) 'tools'
$paths = @($tools)
if ($env:PYTHONPATH) { $paths += $env:PYTHONPATH }
$env:PYTHONPATH = $paths -join [IO.Path]::PathSeparator
$env:PYTHONDONTWRITEBYTECODE = '1'
& $Python -m uvicorn server:app --app-dir $PSScriptRoot --host 127.0.0.1 --port $Port
exit $LASTEXITCODE
