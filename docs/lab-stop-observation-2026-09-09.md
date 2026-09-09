# 停止后连续只读观察：实验室交接

## 本版用途与边界

解决的是旧会话退出后没有连续记录、只能用相隔几十秒的探针对比的问题。
本版不修订 IK、PVAT、运动限位、停止超时或停止兜底；不表示此前约 18 mm
异常位置变化已查明或消除，也不放行 B 段、+x 重测或 VR 真机控制。

三个阶段的数据保存在同一个 session：

| 阶段 | 数据来源 | 是否新增 SDK 读取 |
|---|---|---|
| 动作前与动作中 | 已有 robot_kinematics、pvat_sent 等事件 | 否 |
| 停止验证 | 已有 stop_diagnostics 中的 RPC、samples | 否 |
| 停止处理结束后 | 新增 stop_observation 系列事件 | 是，只读、单连接、同一 SDK 锁 |

“连续”是持续的离散采样，不是无间隙或硬实时记录。SDK 状态与运动学来自
多次读取，不是控制盒提供的同一时刻原子快照。不能推断两个样本之间绝无运动。
运动阶段不启动第二个采样任务，避免加重现有发送/读状态锁竞争。

## 开关

在既有 smoke 命令后加 `--observe-stop-seconds 60`。默认 0（关闭），允许
0 到 120 秒；NaN、无限大或越界值会在连接前拒绝。

本次现场只使用 **stop**。代码还支持具备同一停止清理路径的 translate、
rotate、gripper、home，但支持参数不等于允许现场测试。
`prepare` 有独立清理路径，携带非零观察参数会在连接前拒绝；不能为了观察
而延后其失败清理，也不增加新的停止调用改变已有动作流程。

观察阶段：

- 不调用 start_sys、stop_move、stop_sys、movej、move_pvat 或 set_claw；
- 即使之前停止失败，仍尝试读取真实原始状态，不清除故障、不重试动作；
- 约每 200 ms 发起一次读取，慢读跳过错过的周期，不突发补采；
- 单次读取有 500 ms 协作式超时；最后一次读取可在观察窗口结束后完成，
  避免把“窗口到点”误报成网络超时；
- 读取失败会记录后继续观察，不自动重连或恢复系统；
- 若动作/停止失败，立即显示失败提示，最终仍保留原失败；
- Ctrl+C/取消不会强制等满 60 秒，仍走已有安全清理。不要用 Ctrl+C 替代硬件急停。

**观察只读不等于配置自动切为 readonly。** 配置由现场在命令结束后手动恢复。
程序的观察过程不会因为配置 mode 仍是 control 就发运动或恢复命令。

## 新日志如何解释

- `stop_observation_started`：UTC 时间、观察时长、目标间隔、读取期限、之前的失败原因。
- `stop_observation_sample`：读取起止单调时间、UTC、读取耗时、与前次尝试/成功样本的间隔；
  `state` 包括 raw_robot_state、映射状态、estop、latched_fault、running_motion、
  actual_q/qd/qdd、TCP、目标关节/速度/TCP、各 SDK 调用耗时和锁等待。
- `stop_observation_read_failed`：失败序号、时刻、异常类型/原因和耗时，不补造状态值。
- `stop_observation_summary`：成功样本数、读取错误数、队列丢失数、最长成功样本间隔、
  尾部空档、是否中断、实际观察时长。

`assessment=diagnostic_only`。`complete=true` **仅表示观察结束且没有读错误、
没有检测到队列丢失、有成功样本**，不表示机械臂全程静止、没有漂移、达到固定
5 Hz 或可以继续运动。长间隔被记录，但不会仅因长间隔自动置 complete=false。
本版不自动判定观察期漂移合格；必须检查整条状态/位置时间线。

读取错误、无成功样本或队列丢失会使观察段报错，若原动作无错误，最终退出码
非零；若已有动作/停止错误，保留原错误。`smoke_result` 可能早于观察段打印，
不要只看它的 stable=true；需等最终退出码并核对全部记录。
日志 summary 入队不等于已持久化，必须等进程完成并核对 session.jsonl 和
summary.json 中的 dropped_normal_events/fatal_error。记录器损坏时不能保证
保存全部诊断；不能据缺失数据得出安全结论。

使用单调时间排序/分析同一主机上的事件；UTC 用于与现场记录对应。记录器
优先写关键事件，文件行顺序可能不等于采样时间顺序。不要跨主机直接相减单调时间。

## 下一次只做静止 stop + 60 秒观察

须先由现场再次确认：仍为已核对的 Home 位形、IDLE、实际静止、TCP 匹配，
周围无人、空载、观察员就位、硬件急停可直接操作。若条件改变先回传，不自行归位。
此前现场 Home 为 `[0, -90°, 0, -90°, 0, 0]`，不是六轴全零。

1. 核对主机提供的新提交号，保留本地配置与日志。更新异常就停，不 reset 或重建 refs。
2. readonly 模式执行一次既有预检，保存新时间戳报告和配置 SHA256。
3. 现场确认后，只切换 mode 到 control，保留其它参数。在笔记本项目根目录执行一次：

```powershell
$env:PYTHONPATH = (Resolve-Path .\backend).Path
$cfg = "config/real-robot.local.yaml"
$confirm = "I_UNDERSTAND_REAL_ROBOT_MOTION"
python .\scripts\real_robot_smoke.py stop --config $cfg --confirm $confirm --observe-stop-seconds 60
$exitCode = $LASTEXITCODE
Write-Output "smoke exit code: $exitCode"
```

4. 观察期不要追加命令、启动另一个预检或控制网页，不操作 xLab。保持目视观察。
   如有实际位移、异常或停止失败，现场按安全规程立即处理，不等 60 秒结束；
   记录是否触发急停、手动操作及准确时间。诊断脚本不是安全监督系统。
5. 命令完成后恢复 readonly，做一次退出预检。不执行 prepare、+x、其它轴或 VR。
6. 回传完整新 session 目录（不要只摘最后一条）、退出码、前后预检、配置哈希、
   现场观察和所有人工干预的时间。本次不要求上传凭据或远程访问配置。

主机先检查新记录链路是否完整，再单独决定下一次受控运动诊断。即使本次
静止观察全部通过，也不能替代运动后停止验收。

## 本地验证记录

- 完整后端回归：`python -m pytest -q --tb=short`，708 项通过，86.57 秒。
- 新增 12 项观察相关测试，覆盖故障锁定下只读采样、成功/失败停止后的观察、
  时间边界、读失败后继续记录、全部超时、队列丢失、取消与断开连接、参数限制。
- 实际 CLI 的 `stop --help` 已核对显示新参数。只读代码审查完成。
- 未连接或驱动真机；这些结果不能替代实验室记录链路及运动停止验收。
