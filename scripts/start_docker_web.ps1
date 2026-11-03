param([switch]$Run)
$ErrorActionPreference='Stop'
$projectRoot=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$python=Join-Path $projectRoot '.venv/Scripts/python.exe'
$nodeCommand=Get-Command node -ErrorAction SilentlyContinue
$node=if($nodeCommand){$nodeCommand.Source}else{Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe'}
if(-not (Test-Path -LiteralPath $node)){throw 'Node runtime missing'}
if(-not (Test-Path -LiteralPath $python)){throw 'Python environment missing'}
if(-not $Run){
    [pscustomobject]@{execute=$false;backend='http://127.0.0.1:8773/';https_port=8443;mode='Docker PVAT VR';requires='Running admitted official local Docker'} | ConvertTo-Json
    return
}
# Verify local simulation before launching any SDK owner. This never starts,
# resets or retries a stopped/faulted robot or an existing faulted web service.
& $python (Join-Path $projectRoot 'scripts/run_lmaster_docker.py') --pvat-vr
if($LASTEXITCODE -ne 0){throw 'Docker admission failed; start the official local simulator first'}
$launchDirectory=Join-Path $projectRoot ('.tmp/docker-web-services/'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
New-Item -ItemType Directory -Path $launchDirectory -Force | Out-Null
$backendPort=Get-NetTCPConnection -State Listen -LocalPort 8773 -ErrorAction SilentlyContinue
if(-not $backendPort){
    $backend=Start-Process -FilePath $python -ArgumentList @('-u',('"'+(Join-Path $projectRoot 'scripts/run_lmaster_docker.py')+'"'),'--run','--pvat-vr') -WorkingDirectory $projectRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $launchDirectory 'backend.stdout.log') -RedirectStandardError (Join-Path $launchDirectory 'backend.stderr.log')
}
$status=$null
for($attempt=0;$attempt -lt 100;$attempt++){
    try{$status=Invoke-RestMethod 'http://127.0.0.1:8773/simulation/status' -TimeoutSec 1;break}catch{Start-Sleep -Milliseconds 150}
}
if(-not $status -or $status.runtime -ne 'LEBAI_DOCKER' -or $status.sdk_target -ne '127.0.0.1:3030'){throw "Backend unavailable or wrong runtime. Logs: $launchDirectory"}
if($status.control_fault -or $status.unknown_reads.Count -or $status.unknown_writes.Count){throw 'Existing fault/unknown result retained; launcher will not reset or restart it'}
$proxyPort=Get-NetTCPConnection -State Listen -LocalPort 8443 -ErrorAction SilentlyContinue
if(-not $proxyPort){
    $proxy=Start-Process -FilePath $node -ArgumentList @('node_modules/vite/bin/vite.js','--config','vite.docker.config.ts','--port','8443') -WorkingDirectory (Join-Path $projectRoot 'web') -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $launchDirectory 'https.stdout.log') -RedirectStandardError (Join-Path $launchDirectory 'https.stderr.log')
}
$proxyStatus=$null
for($attempt=0;$attempt -lt 80;$attempt++){
    $body=& curl.exe -k -s --max-time 1 'https://127.0.0.1:8443/simulation/status'
    if($LASTEXITCODE -eq 0){try{$proxyStatus=$body|ConvertFrom-Json}catch{}}
    if($proxyStatus -and $proxyStatus.run_id -eq $status.run_id){break}
    Start-Sleep -Milliseconds 150
}
if(-not $proxyStatus -or $proxyStatus.run_id -ne $status.run_id){throw "HTTPS proxy unavailable or wrong runtime. Logs: $launchDirectory"}
$report=[pscustomobject]@{backend_pid=$status.pid;run_id=$status.run_id;https_pid=(Get-NetTCPConnection -State Listen -LocalPort 8443|Select-Object -First 1 -ExpandProperty OwningProcess);desktop='http://127.0.0.1:8773/';https_port=8443;logs=$launchDirectory;state=$status.control_mode}
$report|ConvertTo-Json|Set-Content -LiteralPath (Join-Path $launchDirectory 'services.json') -Encoding utf8
$report|ConvertTo-Json
