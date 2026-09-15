# 新版停止事务：现场复测准备与分阶段放行

## 当前版本与本轮范围

运行源码固定为已在实验室归档核验通过的提交：

- commit：`c7452903a59493b0a2097225ac3139ac9bebe951`
- tree：`0ca86c5cc90e5d7698d852bffa3851ecb3c8c4b5`
- 实验室隔离目录：`C:\WorkSpace\isolated-verify\2026-09-15-c745290\src`
- 实验室报告的离线子集：173 passed / 0 failed。该结果不代表真机停止漂移已消除。

原仓库 HEAD 仍为 `3651a71`，不从原仓库启动控制程序，不修引用，不重新下载或重复跑已通过的离线子集。

**本轮只执行 R 阶段：环境核对、现场目视确认、一次 readonly 预检。P/M 阶段只是后续计划，尚未开放执行。** 本文不允许启动机器人系统、解除急停/保护、Home 或其他动作。现场安全处置与离场停机按既有规程优先处理。

## R0：现场确认（不按控制按钮）

操作员记录时间及现场实际情况：控制盒是否已上电、原厂界面显示状态/报警、硬件急停情况、当前姿态、是否有负载、是否有人或障碍物在运动范围内、其它控制客户端是否已退出。

不要沿用历史 IDLE/HOLD。如设备未上电，不为本轮自行上电；如有非预期动作或报警、现场无法确认安全状况，停止本轮，按现场安全流程处理。不能为了采集完整日志而延迟安全处置。

设备已上电且现场允许只读查询时，进入 R1。保持原厂界面只观察，不点击“启动机械臂”或切换模式。

## R1：固定目录与普通解释器导入核对（不连接设备）

在 PowerShell 中逐条执行。下列路径均为实验室回执提供的路径，若不存在或不同，回报实际情况，不自动创建替代项目或重新安装依赖。

```powershell
$src = 'C:\WorkSpace\isolated-verify\2026-09-15-c745290\src'
$py = 'C:\WorkSpace\VR4AM\.venv\Scripts\python.exe'
$cfg = 'C:\WorkSpace\VR4AM\config\real-robot.local.yaml'
Set-Location -LiteralPath $src
$env:PYTHONPATH = (Resolve-Path .\backend).Path
& $py -c "from pathlib import Path; import app; import app.robots.lebai_adapter as a; import app.robots.lebai_stop_transaction as s; root=(Path.cwd()/'backend').resolve(); modules=(app,a,s); paths=[Path(m.__file__).resolve() for m in modules]; print(*paths,sep='\n'); assert all(p.is_relative_to(root) for p in paths), 'wrong source import'"
$ImportExit = $LASTEXITCODE
Get-FileHash -Algorithm SHA256 -LiteralPath $cfg
(Get-Item -LiteralPath $cfg).Length
& $py -c "import sys,yaml; p=yaml.safe_load(open(sys.argv[1],encoding='utf-8')); print('mode=',p['real_robot']['mode']); assert p['real_robot']['mode']=='readonly'" $cfg
$ConfigExit = $LASTEXITCODE
```

保存原始输出。要求两个退出码都是 0，全部模块来自隔离目录，不是 editable 安装指向的旧仓库。PYTHONPATH 仅改变当前终端进程环境，不修改全局配置；预检子进程继承它。

配置仍使用现场原件的绝对路径，只读、不复制、不编辑。基线为 1567 字节、SHA-256 `97a2c87c218fce7613029a3487dfb312a4a02ed4488ed556b99708fb0f4ce208`。不同就回传，不能改回旧值来通过核对。

## R2：单次 readonly 预检（仅此步连接设备）

仅 R0/R1 满足后执行。不启动 Web 服务、VR 或其它机器人客户端。预检代码在 readonly 模式下不发运动/停止写命令；本步骤不是停止测试。

```powershell
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
$out = Join-Path $src "logs\preflight-stop-owner-R-$stamp.json"
if (Test-Path -LiteralPath $out) { throw 'Output already exists' }
& $py .\scripts\real_robot_preflight.py --config $cfg --output $out
$PreflightExit = $LASTEXITCODE
"PreflightExit=$PreflightExit"
Get-Content -Raw -LiteralPath $out
Get-FileHash -Algorithm SHA256 -LiteralPath $cfg
```

只运行一次。无输出或错误时不重启、不补跑其它探针；保留退出码、stderr 和已有文件。若程序未退出，不启动第二个机器人连接。

`complete=true`、退出码 0、`real_robot_readonly` 只说明本次只读检查符合预期，**不等于可运动**。需同时报告 `preflight_observation`、`state_observation`、采样时间、实际关节/TCP/速度/加速度、急停映射字段及故障。读取时刻不同，不把顶层与后续快照拼成一个瞬间。

原始 running_motion 非零不能单独认定仍有运动；新实例 fault=null 不能清除历史停止故障。单帧速度为零也不代替现场持续观察。

## R3：回传后暂停

回传：R0 现场观察时间及事实；R1 导入路径、退出码和配置哈希；R2 完整原始 JSON、退出码、终端输出、前后配置哈希；实际工作目录和隔离归档身份。明确本轮是否有人工操作及时间，不把必要安全处置漏掉。

机器人仍保持原现场安排。本轮不恢复 IDLE、不 prepare、不平移、不运行 stop smoke。若结果不满足条件，只报告问题，不自行修改包络、TCP、姿态或停止超时。

## 后续 P/M 阶段（仅计划，等待主机端核对 R 回执后单独确认）

1. P：根据新鲜姿态判定是否需要现场人工恢复；必要时由操作员另行确认安全恢复路径。随后仅一次 prepare，保存完整会话。不能从未知姿态直接套用 Home 关节数组。
2. M：准备位、现场保护及版本确认后，拟做一次基坐标 +x 2 mm 与 60 秒停止后观察，不叠加其它方向、旋转、夹爪或 VR 输入。参数不放宽。具体命令待 P 条件明确后提供，不从旧 A/B/C 文档复制执行。
3. 首轮 stop_move 与 stop_sys 仍可能重叠；新代码只减少同事务重复发送，并未证明系统失能路径正常。物理急停和现场监督不可由软件 stop、Ctrl+C 或切 readonly 替代。
4. 发生抖动、非预期位移、停止超时或无法确认安全，现场立即按既有安全流程处理，不等待观察满 60 秒；保存完整或不完整日志，不点击“启动机械臂”后继续本轮测试。

M 回传重点：episode_id/request_id、reused、首次与重复调用时间、stop_rpc_lifecycle 的 returned_late/error/unknown_on_local_cancel、stop_diagnostics、stop_transaction_closed、PVAT 在途情况及完整观察。RPC 返回、停止确认、停止后位移、记录完整性、再次允许运动分别判定。同事务同方法重复写入属于实现异常；迟到返回不是成功放行依据。
