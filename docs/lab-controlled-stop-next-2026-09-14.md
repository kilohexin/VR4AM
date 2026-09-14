# 下一步现场安排：恢复准备条件与单次 PVAT 停止诊断

## 已完成与本轮目标

实验室 `1bc66ffd479f33be32c3b00df9570f9342236efb` 的版本、Tree、工作区及 10 项离线自检均已核对通过。本轮无需重复这些工作。

9 月 14 日 11:28 的预检读到 IDLE、速度全零；这只描述采样时状态，不证明此后持续静止。关节约 `[100, -80, 28, -61, 37, 0]` 度，TCP `[0.3851, -0.1773, 0.9089]` m，因 x/y 超出既有启动包络被拒绝。跨天期间缺少连续记录，不能将该姿态变化归因于先前停止漂移，也没有理由无限期冻结已保存的姿态。

本轮目标为：现场确认与必要的人工姿态恢复 → 单次 prepare → 单次 +x 2mm 加 60 秒停止后观察 → 回传。各步依赖上一条结果，不能整段批量执行。

## A. 先确认现场与恢复条件

由现场操作员补充当前急停/报警显示、当前姿态照片（方便时）、是否有其它控制客户端，以及已知的跨天姿态调整情况。无法确认姿态来源时如实写未知，不猜测根因。

动作前要求工具无负载、底座及线缆固定、运动区域无人且无障碍，现场操作员能直接使用实体急停，并有人员持续观察。已知软件停止链路可能超时，因此不能依赖软件 stop 或 Ctrl+C 作为唯一停止手段。

若目前有急停、报警、非预期运动或现场人员无法确认恢复路径，则本轮暂停在此。不要自动复位、启动系统或解除保护。

若界面无异常、机器人静止且现场操作员熟悉既有示教/恢复流程，可由其依据现场路径条件，用原厂界面人工恢复到既有 Home。此为真实运动，路径由现场人员确认；本文不提供从未知姿态直接运动的关节脚本。参考 Home 为 `[0, -90, 0, -90, 0, 0]` 度，仅用于核对，不能绕过现场路径判断。

保留恢复前后的状态与时间，记录使用的按钮/示教操作。恢复结束后关闭其它控制程序，再执行一次 readonly 预检。要求 IDLE、速度全零、TCP 匹配、包络内、原因仅 `real_robot_readonly`。Home 的 J3/J5 接近零，不能从 Home 直接执行笛卡尔平移。

不得修改启动包络、TCP、关节限制或速度来通过上述条件。

每次需要采集只读预检时，从仓库根目录执行以下命令，使用新文件名并保存退出码：

```powershell
$env:PYTHONPATH = (Resolve-Path .\backend).Path
$preflightStamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
.\.venv\Scripts\python.exe .\scripts\real_robot_preflight.py --config config/real-robot.local.yaml --output ".\logs\preflight-next-$preflightStamp.json"
"PreflightExit=$LASTEXITCODE"
```

## B. 单次进入既有准备位

仅 A 完成后执行。下面的命令在仓库根目录的 PowerShell 中逐条运行，使用既有虚拟环境：

```powershell
$env:PYTHONPATH = (Resolve-Path .\backend).Path
$diagConfig = 'config/real-robot.local.yaml'
$diagConfirm = 'I_UNDERSTAND_REAL_ROBOT_MOTION'
```

只将配置的 `real_robot.mode` 从 readonly 切为 control。保留其它参数及变更前配置原件。单次 prepare：

```powershell
.\.venv\Scripts\python.exe .\scripts\real_robot_smoke.py prepare --config $diagConfig --confirm $diagConfirm
$prepareExit = $LASTEXITCODE
"PrepareExit=$prepareExit"
```

prepare 不支持 `--observe-stop-seconds`；保存其完整 session。要求正常退出、`stable=true`、IDLE，并由现场确认静止。配置准备位仍为 `[0.0004,-1.5681,0.252,-1.5722,0.4017,-0.0011]` rad，沿用程序现有到位容差。

随后切回 readonly，采集一份带新文件名的预检确认准备位及故障状态。如失败、程序未结束或现场观察到异常，不执行 C，不重试，不点击“启动机械臂”来继续实验。

## C. 单次 +x 2mm 与停止后观察

仅 B 到位并确认无异常后执行。沿用已核对的低速参数，只切 mode=control。目标沿机器人基坐标系 +x 移动 2mm；不叠加旋转、夹爪或 VR 输入。

```powershell
$diagStamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
$diagOutput = ".\artifacts\acceptance\real-x2mm-$diagStamp"
.\.venv\Scripts\python.exe .\scripts\observe_process.py --output $diagOutput --warn-after 180 -- .\.venv\Scripts\python.exe .\scripts\real_robot_smoke.py translate --config $diagConfig --confirm $diagConfirm --axis x --distance-m 0.002 --observe-stop-seconds 60
$diagExit = $LASTEXITCODE
"ObserverExit=$diagExit"
```

这是首次将独立进程观察器用于该受控真机诊断：它保存终端证据和退出码，不具备机械臂安全控制能力。`--warn-after` 仅告警；无输出不能作为重复启动依据；Ctrl+C 在观察器监测循环中不会触发机器人停止。保持现场目视监督，发生非预期位移或停止异常时，现场立即按既有安全流程处置，不等待观察满 60 秒。

停止后观察期不要运行第二个预检或操作原厂界面。必要的紧急处置优先，准确记录发生时间；不能为了日志完整延迟处置。

程序结束后恢复 readonly，在现场确认无异常、适合只读采集的前提下执行一次退出预检。若程序仍活着，不启动第二个机器人客户端。切换配置文件不会停止已运行的进程或机器人。

## D. 一次回传完成所有证据

保存 prepare、translate 的完整 session 目录；进程观察器四个原始文件；恢复后、prepare 后、退出后的预检；配置原件与哈希；人工恢复及任何紧急干预的时间线。不得因失败而删除无 summary.json 的 session。

重点查看：

- PVAT：`sdk_request_lifecycle` 的请求编号、开始/结束时间、RPC 结果、generation、invalidated、实际发送参数；与 `pvat_sent` 对照。缺失事件不证明未发送。
- 停止：`stop_diagnostics` 每次 RPC 的结果及验证采样，`stop_failed`、`stop_confirmed`、最终故障及 preflight 字段。RPC 返回不代表物理停止。
- 观察：完整 `stop_observation_summary`、读取错误及丢失计数；相对首个观察样本的最大关节/TCP 变化，另列峰峰值，不能混用。结合现场目视记录判断异常。
- 进程：`process.json` 的 status、child_exit_code、observer_errors 与原始输出。测试结果、停止结果、记录完整性分别报告。

只执行上述一次 C。即使成功，也不自动进入下一方向、夹爪或 VR；先分析新版本是否仍有停止超时，以及在途 PVAT 与停止的时间关系。本轮用相同 +x 2mm 幅度对照 B01，结果可能受位形、通信负载和时序影响，单次结果不能单独确定根因。

## 预检补充字段的版本说明

本文件与预检报告增强在同一提交中发布：新增 `started_utc`、`sampled_utc`、`preflight_observation`、`state_observation`。两个 observation 分别保存预检判定时与其后状态读取时的完整已采集运动学事件，含 `captured_ns`，避免将不同读取时刻混为一个快照。

`state_observation` 提供 `raw_robot_state`、`estop`、`running_motion`、`fault`、实际/目标关节和 TCP。`estop` 是 SDK 读数经适配器映射后的结果，并非 SDK 原始急停数值；`fault` 是本次新建适配器的状态字段，不是控制盒完整故障历史。`sampled_utc` 记录读取完成时间，不是退出或断开连接完成时间。

这项增强复用已有查询，不增加机器人 RPC；旧顶层字段和 complete/退出码语义保留。使用增强版需先取得实际发布提交号。现场已有 1bc66ff 仍可执行上述流程，旧版预检缺失字段由现场目视记录补充，无需仅为此重复完成过的离线核验。
