# 实验室交接：停止等待期诊断开关

## 本轮只做离线核验

停止后约22mm位移尚未解释。此版本增加诊断能力，不是运动安全修复。本文件不授权连接设备、切control、prepare、stop实机命令、平移或VR真机操作。

不重复已经完成的版本环境排查，不修Git引用。原仓库仍有引用异常时，沿用已验证的隔离源码目录方案。主机端明确告知新提交已推送后再获取；不要在旧的c745290隔离目录中直接套用本说明。

## 代码变化

- 新参数 `--stop-rpc-diagnostics`，默认关闭，仅接受stop和translate动作。
- 开启后，同连接独立读取状态/运动数据/急停原因；不更新控制快照，不清故障、不改变200ms停止升级条件。
- `session_started.metadata.stop_rpc_diagnostics` 留存 enabled、schema_version、读取/持续/间隔/清理预算及diagnostic_only标记。schema_version是诊断格式版本，不是Git提交号；源代码版本仍需单独记录。
- 此开关不自动切换mode、不执行prepare、不追加任何动作。
- 独立诊断读取与 `--observe-stop-seconds` 是不同阶段；前者补停止请求等待期，后者仍负责停止后的观察。

## 单次离线测试

在已核对的新源码根目录运行PowerShell。使用既有Python环境，不安装或升级现场SDK，不修改现场YAML；测试仅使用假SDK。输出目录必须是全新目录，如同名目录已存在则换编号，不覆盖。

```powershell
$Python = 'C:\WorkSpace\VR4AM\.venv\Scripts\python.exe'
$OldPythonPath = $env:PYTHONPATH
try {
    $env:PYTHONPATH = (Join-Path (Get-Location).Path 'backend')
    & $Python -c "import app, app.commissioning.smoke as s, app.robots.lebai_stop_diagnostics as d; print(app.__file__); print(s.__file__); print(d.__file__)"
    if ($LASTEXITCODE -ne 0) { throw 'Import check failed' }
    # 先检查上面三条路径：必须均属于本次新源码目录。
    # 如指向旧仓库，停在这里；不要执行后续命令。
} finally {
    $env:PYTHONPATH = $OldPythonPath
}
```

确认三条导入路径正确后，执行一次：

```powershell
$OldPythonPath = $env:PYTHONPATH
try {
    $env:PYTHONPATH = (Join-Path (Get-Location).Path 'backend')
    & $Python .\scripts\observe_process.py `
        --output .\artifacts\acceptance\lab-stop-rpc-diagnostics-001 `
        --warn-after 30 -- `
        $Python -m pytest -c backend/pyproject.toml `
        backend/tests/commissioning/test_stop_rpc_diagnostics.py `
        backend/tests/robots/test_lebai_stop_diagnostics.py -q --tb=short
    $ObserverExit = $LASTEXITCODE
    Write-Output "ObserverExit=$ObserverExit"
} finally {
    $env:PYTHONPATH = $OldPythonPath
}
```

预期 **24 passed、0 failed**，观察器与子进程退出码均0，process.json为EXITED、observer_errors=[]。第21轮后新增一个确定性的同刻度读取用例，原生时钟用例保留。计数或结果不一致就保存原件回传，不反复重跑到通过。warn-after仅告警，不是自动终止或停止机械臂。

## 回传最小清单

1. 实际源提交号/隔离归档标识、执行目录、三个导入路径。
2. 测试通过失败数量、观察器退出码及process.json中的child_exit_code和observer_errors。
3. 原始四件产物stdout.bin、stderr.bin、events.jsonl、process.json，按原字节归档。
4. 确认本轮未连接/操作机器人，现场配置仍readonly且未改；机械臂状态未采集就写“本轮未采集”。

本轮不要求重新采集状态、不访问443、不扫描端口、不联系厂家、不操作原厂网页。

## 主机端验证记录

以下23项/820项记录属于原始1610453版本；第21轮修正的验证记录见 `lab-round21-clock-response-2026-09-16.md`，不要混用计数。

- 全量后端：820 passed，101.33秒，退出码0。
- 独立只读代码审查通过；审查方运行本轮23项测试全部通过。
- 实际进程观察器包装本轮23项：23 passed，ObserverExit=0，child_exit_code=0，EXITED，observer_errors=[]；本地原件 `artifacts/acceptance/host-stop-rpc-diagnostics-20260916-001/`。
- `scripts/real_robot_smoke.py stop --help` 已显示新开关；只运行帮助，不连接设备。

以上是本机离线证据，不代替实验室导入隔离检查，也不代表位移故障已修复。

## 下一次真机方案的放行条件

先由主机端核对这24项离线结果，再单独明确：是否只做静止stop诊断基线，以及是否需要后续一次受控运动。不得自动串联stop→prepare→translate。

日志中的 `outcome=sample` 只说明三项读取完成；它们有各自时间戳，不是原子快照，更不代表已安全停止。`timeout/error/cancelled`不能补齐为正常读数。若出现stop_unverified、未解决读取、非预期位移或现场异常，停止后续测试并按现场安全流程处置，不能靠切readonly或退出程序代替物理安全处置。
