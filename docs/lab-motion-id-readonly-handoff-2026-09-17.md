# 实验室下一步：单次只读采集运动 ID 状态

## 发布与范围

本文件随本地修正生成。**只有主机明确通知对应新提交已推送后才能执行；旧的ddfe894没有本次修正。** 主机提供的完整提交号为版本依据；下文不猜测提交号或隔离目录名称。

不需要再次跑整套离线测试或重新排查Git。沿用已验证的新隔离源码目录方式；不修引用、不覆盖旧产物。若拉取/导入异常则回传，不自行修复。

本轮仅一次 `real_robot_preflight.py`，配置保持readonly，读取实际运动ID及其状态；不调用stop、stop_sys、prepare、启动、复位、平移、旋转、夹爪、Home或VR，不自动接续任何动作。不访问443、不扫描端口、不要求换Wi-Fi。

## 执行前

- 保留现场当前状态，不为了满足IDLE条件点击启动或恢复。当前状态未知就写未知，不复制18:59:45的历史状态。
- 操作员在场、区域无人及障碍、急停可操作；不在测试期间使用其它控制客户端。需要安全处置时优先现场安全流程，不为留证延迟处置。
- 配置仍为 `readonly`、`ip: 10.20.17.1`；原件SHA-256为 `97a2c87c218fce7613029a3487dfb312a4a02ed4488ed556b99708fb0f4ce208`。若不同，停止并报告，不改成匹配哈希。
- 在**新版本隔离源码根目录src**执行。Python沿用 `C:\WorkSpace\VR4AM\.venv\Scripts\python.exe`。输出目录必须全新。

## PowerShell 原文（待主机通知发布后，仅执行一次）

```powershell
$ErrorActionPreference = 'Stop'
$Python = 'C:\WorkSpace\VR4AM\.venv\Scripts\python.exe'
$Config = 'C:\WorkSpace\VR4AM\config\real-robot.local.yaml'
$Source = (Get-Location).Path
$OutputDir = Join-Path $Source 'artifacts\acceptance\lab-motion-id-readonly-001'
if (Test-Path -LiteralPath $OutputDir) { throw 'Output exists; use a new numbered directory, do not overwrite' }
$OldPythonPath = $env:PYTHONPATH
try {
    $env:PYTHONPATH = Join-Path $Source 'backend'
    # 本地导入检查：不调用SDK、不连接设备。强制检查三个模块均来自当前隔离目录。
    & $Python -c "from pathlib import Path; import app, app.commissioning.preflight as p, app.robots.lebai_adapter as a; root=(Path.cwd()/'backend').resolve(); mods=(app,p,a); [print(m.__file__) for m in mods]; assert all(Path(m.__file__).resolve().is_relative_to(root) for m in mods)"
    if ($LASTEXITCODE -ne 0) { throw 'Import check failed; do not run preflight' }
    $BeforeHash = (Get-FileHash -LiteralPath $Config -Algorithm SHA256).Hash
    if ($BeforeHash -ne '97a2c87c218fce7613029a3487dfb312a4a02ed4488ed556b99708fb0f4ce208') { throw 'Config differs from readonly baseline' }
    New-Item -ItemType Directory -Path $OutputDir -ErrorAction Stop | Out-Null
    $Started = (Get-Date).ToString('o')
    & $Python .\scripts\real_robot_preflight.py --config $Config `
        --output (Join-Path $OutputDir 'preflight.json') `
        1> (Join-Path $OutputDir 'stdout.txt') 2> (Join-Path $OutputDir 'stderr.txt')
    $PreflightExit = $LASTEXITCODE
    $Ended = (Get-Date).ToString('o')
    $AfterHash = (Get-FileHash -LiteralPath $Config -Algorithm SHA256).Hash
    [ordered]@{ started=$Started; ended=$Ended; exit_code=$PreflightExit; config_before=$BeforeHash; config_after=$AfterHash } |
        ConvertTo-Json | Set-Content -LiteralPath (Join-Path $OutputDir 'execution.json') -Encoding UTF8
    Write-Output "PreflightExit=$PreflightExit"
    # 无论退出码为0、2或异常，均不接续任何设备动作，不重试。
} finally {
    $env:PYTHONPATH = $OldPythonPath
}
```

如果工具无回显/中断，不重发命令；先只读检查输出文件和本机进程是否仍存在。PowerShell通道报启动错误则回传，不能在设备执行阶段自动改用另一通道。

## 如何读结果

检查 `preflight_observation` 和 `state_observation` 两组，各保留：

- `raw_robot_state` / `robot_state`；
- `raw_running_motion`（SDK原始ID）、`motion_state`（查询结果）、`running_motion`（适配器有效ID）；
- actual/target关节与TCP、actual_qd、estop/fault及sdk_latencies_ms。

HOLD下 `preflight_reason=robot_not_idle`、`complete=false`、退出码2是**运动准入拒绝**，并不表示本次读取没有拿到证据。即使 `motion_state=FINISHED`、有效ID变为null，也不把HOLD写成IDLE或允许运动。

- FINISHED：支持残留ID已结束这一分支，后续仍需单独安排受控停止验证。
- WAIT/RUNNING/UNKNOWN/null：不能强行忽略ID、不改阈值、不自动恢复。
- 状态已变/无ID：照实回传，不为了重建ID111去执行动作。
- 读取错误/超时/JSON未产生：保存stdout/stderr及退出信息，不补跑。

## 最小回传

新提交号、隔离目录、输出目录原件（JSON及执行记录、stdout/stderr），一句话确认本轮是否有人工干预。无需再次写多轮长报告或复制全部历史产物；原始数据由主机分析。

**本轮执行完即暂停。该只读采集不验证停止性能，不放行真机运动或VR。**
