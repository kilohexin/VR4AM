# 下一次实验室测试清单：发送耗时 + 准备位小幅活动余量

本清单是此前“仅单次 +x 5 mm”诊断安排的后续扩展。只能逐条执行，禁止把动作放进循环或整段批量粘贴运行。
**+z 问题已提升为本轮重点：本轮默认执行 P 段只读 IK 探测与 A 段发送诊断。B 段为保留的候选动作清单，等待主机分析 P 段后确认，不直接批量实施。**
目标是取得诊断数据并检查准备位附近的局部余量，不是工作空间覆盖率、连续运动性能或 VR 真机验收。

## 0. 版本与现场条件

在笔记本项目根目录、既有 Python 环境执行以下非运动命令：

```powershell
git status --short
git switch codex/offline-rehearsal
git pull --ff-only origin codex/offline-rehearsal
git rev-parse HEAD
git diff HEAD --
python --version
python -m pip show lebai-sdk-asyncio
Get-FileHash .\config\real-robot.local.yaml -Algorithm SHA256
$env:PYTHONPATH = (Resolve-Path .\backend).Path
$cfg = "config/real-robot.local.yaml"
$confirm = "I_UNDERSTAND_REAL_ROBOT_MOTION"
```

- 与主机端推送后告知的提交号一致；有冲突先停，不 reset、不修改 Git refs、不覆盖本地真机配置。
- 只切换配置中的 `real_robot.mode`；保留 IP、TCP、限位、包络、速度、加速度等实验室参数。
- 现场操作员与观察员就位，硬件急停可直接操作；确认底座、夹爪、线缆固定，动作空间无人、无障碍、无负载。
- 关闭其它会给机器人下指令的程序。配置为 readonly 或软件 stop 均不能替代硬件急停。
- 本轮不测试急停复位、自动 Home、夹爪抓物、实际 +z 运动、混合平移旋转或 VR 连续控制；+z 的只读 IK 查询必须包含，不是长期跳过。

## 1. 只读基线与 stop

先将 `real_robot.mode` 手动设为 `readonly`：

```powershell
python .\scripts\real_robot_preflight.py --config $cfg --output logs\next-baseline.json
```

确认读取完整、IDLE、TCP 匹配、关节速度全零，并由现场确认姿态及包络。readonly 的 `preflight_ready=false / real_robot_readonly` 是模式阻挡，不能当成 control 下的运动就绪判定。

经现场确认允许后，手动切换 `mode: control`，设置确认词，执行一次 stop：

```powershell
$env:VR4ARM_REAL_ROBOT_CONFIRM = $confirm
python .\scripts\real_robot_smoke.py stop --config $cfg --confirm $confirm
```

保存 EXIT 和 session；确认停止成功。若 HOLD、FAULT、停止未确认或读取失败，停止本轮动作并回传日志。Home 中已知奇异位形不代表可直接平移；只有既有 prepare 专用保护路径可以在其余条件合格时尝试移出。
若 prepare 被包络或其它保护拒绝，不修改配置绕过，也不自动 Home；由现场人员使用既有安全操作规程处理后重新做只读确认。

## 2. P段：+z 与准备位只读 IK 探测（优先）

可先在当前已确认静止且符合只读预检要求的姿态、readonly 模式下采集一份当前姿态报告：

```powershell
python .\scripts\real_robot_ik_probe.py --config $cfg --output logs\ik-current.json
```

这不是运动命令：只读取状态与调用 SDK kinematics_inverse。目标为当前 TCP 的原位、六方向 ±2 mm 和六方向 ±0.5°；每个目标分别使用当前关节、配置准备位关节作求解参考。求解参考不是要机器人到达的新姿态。

坐标约定与 smoke 一致：平移沿机器人基坐标轴，旋转是在基坐标系施加增量，绕当前 TCP 保持位置。SDK 参考关节的含义见[乐白位置和位姿文档](https://help.lebai.ltd/sdk/python/posture.html)。

核心数据必须在既有 prepare 到达准备位后再采集。经第1节现场确认后切 control，单次执行：

```powershell
python .\scripts\real_robot_smoke.py prepare --config $cfg --confirm $confirm
```

prepare 成功且静止后，手动切 readonly，再执行：

```powershell
python .\scripts\real_robot_ik_probe.py --config $cfg --output logs\ik-ready.json
```

- EXIT=0 / complete=true 仅说明所有查询完成；`no_solution` 是有效诊断数据，不代表探测失败或允许运动。
- `motion_authorized` 始终为 false。检查 +z 的两组结果、baseline 原位能否求解、J3/J5、关节变化与限位余量。
- 如果 EXIT非0、原位不能求解、机器人运动、TCP/状态异常，停止后续动作并回传报告。超时和 SDK 异常会保存部分报告并中止，不自动重试。
- +z 单独无解时，不强行发 +z、不更改 TCP 或准备位配置。将报告回传主机，分析是位形、参考分支还是目标构造问题。
- 查询成功没有进行正解回验、碰撞和完整路径验证，不能据此把 IK 解写进 Home/prepare，也不能据此放行 +z 实际运动。

## 3. A段：发送诊断

以下每条都必须等上一条结束并确认结果后才执行：

```powershell
python .\scripts\real_robot_smoke.py prepare --config $cfg --confirm $confirm
```

prepare 必须 EXIT=0、stable=true、IDLE。随后仅一次：

```powershell
python .\scripts\real_robot_smoke.py translate --axis x --distance-m 0.005 --config $cfg --confirm $confirm
```

必须同时核对：

- EXIT=0，stable=true，目标 5 mm 的 reached 与 settled 误差各不超过 0.5 mm；正交方向 reached／settled 漂移各不超过 0.5 mm。另单独记录轨迹 max 漂移与峰值，不能用 reached 冒充峰值。
- stop_requested 对应 stop_confirmed，无 stop_failed。
- `smoke_final_state`：stop_confirmed=true、IDLE、fault=null、preflight_ready=true、preflight_reason=null。
- 日志含 `pvat_sent.timing_ms`、`previous_pvat_gap_ms`、`robot_kinematics.snapshot_lock_wait_ms`、`smoke_cycle_timing`。

如果指标缺失或任何条件未满足，不进入 B 段；不要现场调整容差、频率、轨迹时长或速度。

手动切回 readonly，再独立核对退出状态：

```powershell
python .\scripts\real_robot_preflight.py --config $cfg --output logs\next-after-A.json
```

确认 IDLE、TCP 匹配、速度全零。把 P/A 结果回传主机后再确定 B 段；不要因为 +x 通过就自动开始后面的动作表。

## 4. B段：准备位附近逐方向检查（候选，P/A结果审核后才执行）

每个方向独立完成“control → prepare → 一次动作 → 核对结果 → readonly 探针”。不得从上一动作终点直接接下一方向。

| 顺序 | 类型 | axis | 幅度参数 |
|---|---|---|---|
| B01 | translate | x | --distance-m 0.002 |
| B02 | translate | x | --distance-m -0.002 |
| B03 | translate | y | --distance-m 0.002 |
| B04 | translate | y | --distance-m -0.002 |
| B05 | translate | z | --distance-m -0.002 |
| B06 | rotate | pitch | --angle-deg 0.5 |
| B07 | rotate | pitch | --angle-deg -0.5 |
| B08 | rotate | yaw | --angle-deg 0.5 |
| B09 | rotate | yaw | --angle-deg -0.5 |
| B10 | rotate | roll | --angle-deg -0.5 |
| B11（最后） | rotate | roll | --angle-deg 0.5 |

动作命令沿用 A 段格式，仅按表替换类型、axis 和幅度参数。例如 B06：

```powershell
python .\scripts\real_robot_smoke.py rotate --axis pitch --angle-deg 0.5 --config $cfg --confirm $confirm
```

上面的示例不是让你跳过 B01～B05；任何一个方向出现问题，即停止后续方向。每次 prepare 之前都核对当前现场状态。

每项保存实际 before／after 六轴角、TCP、reached、settled、停止及最终预检结果；特别记录 J3/J5 和与现有 5° 判定阈值的关系。0.5° 旋转仍使用既有 0.2° 验收容差，仅作局部可操作性检查，不作高精度标定或最大活动范围结论。

结束后切 readonly，将探针分别保存为 `logs/next-after-B01.json` 等独立文件，不能覆盖。只有当前项所有检查通过，才切回 control 为下一项 prepare。

## 5. 立即结束本轮动作的条件

- 任意非零退出码、过冲、反向、异常漂移、不可达、跟踪故障、限位或奇异保护。
- `preflight_ready=false`（control 的最终事件）、HOLD、FAULT、stop_failed，或停止状态无法确认。
- 意外声响、碰撞风险、线缆拉扯、肉眼观察到异常运动。现场按安全规程停止，必要时使用硬件急停；不可为采集日志延误停止。

进入奇异保护区时，“停止成功”不等于“可以继续试”。不要反复解锁、自动恢复或自行补测；保留原始日志并联系主机端。

本轮不做 C段连续往返；先用 A/B 数据决定后续是否需要新准备位或调度修改。

## 6. 回传清单

建立当天交付目录，包含：

1. 版本信息、工作区差异、本地配置哈希（切换 mode 前后分别注明）、Python/SDK 版本。
2. 全部本轮 session 子目录（包括 stop、prepare、失败尝试），保留 session.jsonl 与 summary.json。
3. 每项结束后的独立 readonly JSON，原始终端 EXIT/错误。
4. 一张汇总表：测试编号 → session ID → reached/settled → 最大漂移 → J3/J5 → stop_confirmed → control 最终 preflight_reason → 后续 readonly 状态。
5. 准备位现场照片，以及是否存在人工移动/重启/配置变更。若有，明确发生在哪两个 session 之间。
6. `ik-current.json`（如采集）和必须的 `ik-ready.json`，包括未完成的部分报告；不要只截取 +z 一行。

直接复制 deliverables 目录给主机端即可。logs 被 Git 忽略不等于不存在，不必强行提交日志或私有配置到 GitHub。

## 本机准备验证

只读 IK 探测 12 项针对性测试通过；完整后端 678 项通过（98.29秒），CLI `--help` 可运行。
已覆盖 control 模式拒绝、六方向目标、双参考、无解、无运动写入、超时中止、异常返回及位形改变中止。未在本机连接真机；+z 的现场根因与新准备位选择尚待探测结果。
