# LM3 真机部署与首次实验流程

本文用于乐白 LM3、原厂 LMG-90 夹爪和 Meta Quest 3 的首次真机接入。默认运行模式仍是模拟器；只有本地配置、环境确认、后端预检、WebXR Grip 松开门和单控制端所有条件同时满足，真机才可能接收运动命令。

首次实验不要求眼在手外相机，也不要求相机标定。相机与时间同步接口继续保留，完成安全、稳定的遥操作后再接入 Gemini 330。

当 LM3 或 Quest 暂不可用时，推荐先完成 [无真机离线实验演练](offline-rehearsal.md)，用 `LEBAI_FAKE / SIMULATION` 复核当前软件路径。离线报告始终是 `hardware_verified=false`，不替代本流程中的真实 SDK、方向、延迟、停止距离、急停和夹爪负载检查，也不授权直接进入 `control`。到达现场后仍从第 2 节的 `mode: readonly` 和第 4 节的零写入只读预检开始。

当前 Task 9 冻结点的完整 PC 浏览器演练仍为 **pending/failed**：Fake 准备位姿已在 `11.488 s` 内通过，但平移阶段的一次 `399.063 ms` 帧空洞触发了预期的 stale 停止，后续阶段未完成。该状态不改变现场流程；即使未来离线浏览器演练通过，也仍必须从 `readonly` 开始并完成全部八项真机检查。

## 0. 必须遵守的边界

- 不在无人值守、急停不可达或工作区不清空时运行真机。
- 首次只读预检必须先于 control 模式。
- `real-robot.local.yaml`、机器人 IP 和现场测得参数不得提交到 Git。
- 不猜测 TCP、Home、软关节限位、启动 TCP 包络或夹爪幅值方向。
- 每次 smoke 进程只测试一个轴，观察员确认方向和停止后才能测试下一轴。
- Quest 控制时关闭 PC 控制网页，保持单一控制连接。
- 测试结果只有实际操作者可以在验收报告中勾选；自动测试不能代替真机验收。
- 离线演练通过也不能勾选任何真机待验证项；`hardware_verified` 必须保持 `false`，直到现场清单由实际操作者完成。

## 1. 从私有 GitHub 仓库部署

以下 `<...>` 都是必须由操作者替换的占位符，不可原样执行。私有仓库需要先在服务器配置 GitHub 登录或 SSH/令牌凭据。

Linux：

```bash
git clone https://github.com/kilohexin/VR4AM.git
cd VR4AM
git checkout codex/offline-rehearsal
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -e "./backend[dev,real]"
cd web
npm ci
npm run build
```

PowerShell：

```powershell
git clone https://github.com/kilohexin/VR4AM.git
Set-Location VR4AM
git checkout codex/offline-rehearsal
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e ".\backend[dev,real]"
Set-Location web
npm.cmd ci
npm.cmd run build
```

若该功能分支之后已合并到主分支，应将 `git checkout` 的值替换为实验负责人确认的分支或提交号，并把实际提交号写入验收报告。

## 2. 建立现场本地配置

从仓库根目录复制模板：

```powershell
Copy-Item .\config\real-robot.example.yaml .\config\real-robot.local.yaml
```

首次保持：

```yaml
backend: lebai
real_robot:
  mode: readonly
```

必须在 L Master、官方 SDK 只读结果和机械臂铭牌/说明中核实并填写：

- LM3 局域网 IP；
- 原厂 LMG-90 的 `get_tcp()` 六个值；
- 经现场批准的 `home_q`；
- 经现场批准、无碰撞且避开 J3/J5 零位的 `teleop_ready_q`；
- 六轴软限位和限位余量；
- 当前初始姿态允许的 TCP 三轴启动包络；
- TCP 对比容差；
- IK 相邻解连续性阈值与跟踪误差包络；
- 夹爪开/闭幅值方向。

LM3 的 IP 写在 `config/real-robot.local.yaml` 的 `real_robot.ip`，该文件只保留在实验室电脑，不提交 Git。当前乐白 Python SDK 连接入口只接收机器人 IP，项目没有额外配置“机械臂端口”；`8000` 是本项目 FastAPI/WebSocket 后端端口，`5173` 是本项目 HTTPS 前端端口，二者都不是 LM3 控制端口。

真机控制页面显示的是 SDK 状态驱动的 3D 机械臂模型、TCP、关节和诊断数据；当前版本没有接入相机视频，也不是 Quest 透视 MR。首次现场测试应由操作者直接观察真机，并安排独立观察员守在急停旁。MR 透视与外部/腕部相机叠加属于后续阶段，不能作为当前安全观察手段。

`expected_tcp` 是工具 TCP 配置，不是当前末端实时位置。LMG-90 虽是标配，也必须用 `get_tcp()` 核对，不可凭外观猜测。

`home_q` 与 `teleop_ready_q` 的职责不同：Home 是安全回位点，可以不是笛卡尔遥操作的理想起点；`teleop_ready_q` 是进入平移/旋转测试前由操作者显式执行的准备姿态。程序不会在连接真机时自动移动。配置加载会拒绝距离 J3 或 J5 零位小于 5° 的准备姿态。2026-09-01 现场已验证的候选值为 `[0.0004, -1.5681, 0.2520, -1.5722, 0.4017, -0.0011]`，再次使用前仍须由现场操作者确认周围无碰撞风险。

确认本地文件不会进入提交：

```powershell
git status --short
```

## 3. 现场网络与物理准备

1. LM3、实验服务器和 Quest 3 位于同一可信局域网；关闭访客网络和 AP/客户端隔离。
2. 记录机器人、服务器和 Quest IPv4；从服务器 ping 机器人并记录延迟。
3. 确保 LM3 急停按钮可立即触及，并安排一名独立观察员。
4. 清空机械臂最大可达空间；首轮不放置易碎、尖锐或较重物体。
5. 首轮把 TCP 速度、加速度、相对平移和相对转角保持为模板中的保守值。
6. 启动姿态应位于填写的软关节限位和 TCP 启动包络内。

## 4. 零写入只读预检

从仓库根目录运行：

```powershell
$env:VR4ARM_CONFIG = "D:\path\VR4AM\config\real-robot.local.yaml"
python .\scripts\real_robot_preflight.py `
  --config $env:VR4ARM_CONFIG `
  --output ".\logs\preflight.json"
```

Linux：

```bash
export VR4ARM_CONFIG="/path/to/VR4AM/config/real-robot.local.yaml"
python scripts/real_robot_preflight.py \
  --config "$VR4ARM_CONFIG" \
  --output "./logs/preflight.json"
```

合格条件：

- 进程退出码为 0；
- 报告中 `complete` 为 `true`；
- `preflight_ready` 为 `false` 且原因是 `real_robot_readonly`；
- TCP 对比通过，机器人为 IDLE，无运行任务和急停原因；
- 实际关节和 TCP 位于配置的启动安全范围；
- 报告包含实际 q/qd/qdd、TCP、夹爪、能力和 SDK 延迟；
- 机器人、夹爪没有动作。

只读模式本来就不能显示为“允许运动”。不要为了让 `preflight_ready` 变成 `true` 而把只读模式改成 control。

保持只读模式运行后端进行五分钟观察：

```powershell
$env:VR4ARM_CONFIG = "D:\path\VR4AM\config\real-robot.local.yaml"
Set-Location backend
python -m uvicorn app.main:app --host 127.0.0.1 --port 8000
```

打开 `http://127.0.0.1:8000/health`，应看到：

- `backend: "LEBAI"`；
- `real_robot_mode: "readonly"`；
- `real_robot_enabled: false`；
- `preflight_reason: "real_robot_readonly"`。

## 5. 切换 control 与当前 `+roll 1°` smoke

只有验收报告中“只读阶段”和物理安全项全部通过后，才把本地 YAML 的 `mode` 改为 `control`。

PowerShell：

```powershell
$env:VR4ARM_REAL_ROBOT_CONFIRM = "I_UNDERSTAND_REAL_ROBOT_MOTION"
python scripts/real_robot_smoke.py prepare --config $env:VR4ARM_CONFIG --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
python scripts/real_robot_smoke.py rotate --config $env:VR4ARM_CONFIG --axis roll --angle-deg 1 --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
```

Linux：

```bash
export VR4ARM_REAL_ROBOT_CONFIRM="I_UNDERSTAND_REAL_ROBOT_MOTION"
python scripts/real_robot_smoke.py prepare --config "$VR4ARM_CONFIG" --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
python scripts/real_robot_smoke.py rotate --config "$VR4ARM_CONFIG" --axis roll --angle-deg 1 --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
```

当前现场动作严格限定为两个独立进程：先执行一次 `prepare` 并核对实际关节到达
`teleop_ready_q`，再只执行一次 `+roll 1°`。保存完整日志后立即停止本轮并把配置
改回 `readonly`，不得接着测试 `-roll`、`±pitch`、`±yaw`、`+z`、Quest 真机
连续遥操作、速度提升或 PVAT 频率调整。只有主机端复核新会话后，才会给出下一
个单动作指令。

截至 2026-09-02，`+x/-x/+y/-y/-z` 平移已取得现场通过记录；`+z` 仍因准备
位形接近工作空间/IK 边界而暂停。此前 `+roll 1°` 日志暴露的是完整 IK 目标相对
实际关节的跟踪积压，不是机械臂单帧实际跳动相同角度。本次复测用于验证新的
连续性/跟踪分离与回压策略，不能扩展为多轴验收。

脚本仍会拒绝超出配置边界、确认词错误、readonly 配置、非 IDLE、TCP/Home
缺失或预检失败的运动请求。

平移/旋转成功输出额外包含 `requested_displacement`、`reached_displacement`、`settled_displacement` 和单位，用于区分“满足释放条件”与松开 Grip、稳定停止后的最终位移。平移动作还输出 `reached_cross_axis_drift_m`、`settled_cross_axis_drift_m` 和 `max_cross_axis_drift_m`。释放前必须同时满足目标轴误差与两个正交轴漂移不超过 `0.5 mm`，并且已有运动学日志中的 `max(|actual_qd|)` 不超过配置的 `home_velocity_tolerance_radps`（默认 `0.02 rad/s`），以上条件连续保持 `home_stable_ms`（默认 `300 ms`）。若目标轴曾经到达但正交轴在超时前始终没有收敛，命令以 `smoke_cross_axis_not_settled` 失败；位置已经收敛但关节速度未在超时前持续稳定，则以 `smoke_velocity_not_settled` 失败。两种情况都不会把运动中的中间姿态记为成功。`max_cross_axis_drift_m` 仅记录路径中的峰值，本次修复不会把关节空间路径改成笛卡尔直线。

若准备姿态未执行或当前 J3/J5 过于接近零位，笛卡尔预检会以 `singular_configuration` 拒绝且不发送 IK/PVAT；只有显式 `prepare` 和 `home` 可在其他安全检查全部通过时从该状态执行关节运动。

真机快照读取与命令新鲜度使用同一预算关系：完整 SDK 快照读取最多 `300 ms`，命令发送门槛再保留一个状态采样周期（`state_hz=25` 时总计 `340 ms`）。读取、锁等待与 IK 的短时抖动在该范围内不会误报 stale；在发送 IK/PVAT 前超过总门槛仍会以 `robot_state_stale` 拒绝，不会取消陈旧状态保护。

现场本地配置不会随 Git 更新。拉取本次代码后，必须手动在
`real_robot.control` 中确认以下两项同时存在：

```yaml
max_joint_step_rad: 0.05
max_joint_tracking_error_rad: 0.25
```

缺少新字段、跟踪包络小于连续性阈值或大于 `0.50 rad` 时配置会失败关闭。
`max_joint_step_rad` 现在只表示相邻两个完整 IK 解的最大轴差，用于识别解分支
跳变；`max_joint_tracking_error_rad` 表示完整 IK 解相对最新实际关节的最大允许
积压。`0.25 rad` 不代表关节可以一次跳动 `0.25 rad`。

实际 PVAT 点仍先按 `max_joint_speed_radps=0.15` 对完整六轴速度向量同比例
缩放，再按 `max_joint_acceleration_radps2=0.5` 对速度修正向量同比例缩放。
以 `pvat_horizon_s=0.08`、静止起步为例，第一个 PVAT 点最大只前进
`0.0032 rad`。该方式保持关节向量方向，但不承诺 TCP 路径是严格笛卡尔
直线。

每次 `pvat_sent` 的 `pvat_mode` 有三种：`advance` 表示完整请求已安全推进；
`interpolated_advance` 表示原请求的相邻解不连续，经过最多两次 Cartesian
插值重求解后推进了较近目标；`catch_up` 表示新目标连续但真机积压过大，本
周期只追赶最后已接受完整解，不推进 TCP/IK 历史。`catch_up` 会伴随 best-
effort 的 `ik_tracking_backpressure` 诊断，并在 HUD 复用“运动连续性边界”软
提示；它不累计 `ik_failure_persistent`。

当候选被关节限位、真实解分支跳变或关节速度保护拒绝时，`session.jsonl`
仍会写入 `ik_candidate_rejected`。日志中的 `solution_step_rad` 应与
`tracking_error_rad` 分开解释，不得再把 `solution_q-actual_q` 全部当作单帧
关节跳变，也不得仅凭一次离线整角度 IK 探针继续放宽阈值。

当前准备姿态的 `+z` 方向仍需单独处理：现场记录显示该方向接近工作空间/IK
边界，不得通过继续放大任一 IK 阈值来掩盖。

若方向错误、抖动、意外转动或停止不完整：

1. 立即按急停或使用现场批准的停止方式；
2. 不运行下一轴；
3. 将 YAML 改回 `readonly`；
4. 保存会话日志并在验收报告记录现象。

## 6. Quest 真机遥操作

本节是后续阶段说明。当前 `+roll 1°` smoke 未经新会话复核前，不得进入
Quest 真机连续遥操作；可继续使用 Quest 控制仿真机械臂。

后端仍只监听服务器回环地址：

```powershell
$env:VR4ARM_CONFIG = "D:\path\VR4AM\config\real-robot.local.yaml"
$env:VR4ARM_REAL_ROBOT_CONFIRM = "I_UNDERSTAND_REAL_ROBOT_MOTION"
Set-Location backend
python -m uvicorn app.main:app --host 127.0.0.1 --port 8000
```

第二个终端：

```powershell
Set-Location web
npm.cmd run dev -- --host 0.0.0.0
```

Quest 打开 `https://<SERVER-LAN-IP>:5173/`。Vite 通过同源 `wss` 把 `/ws` 代理到同机 `127.0.0.1:8000`；不要把后端 8000 暴露到局域网。

首次访问会出现开发证书警告。只有确认地址确为本实验服务器后才接受证书。Windows 防火墙仅允许可信专用网络访问 TCP 5173；Linux 可按现场网段限定来源，例如：

```bash
sudo ufw allow from <QUEST-SUBNET-CIDR> to any port 5173 proto tcp
```

`<SERVER-LAN-IP>` 和 `<QUEST-SUBNET-CIDR>` 必须替换为现场值。

操作顺序：

1. 关闭 PC 控制网页，只保留 Quest 页面。
2. 确认 HUD 显示 LEBAI/control、预检通过且没有故障。
3. 完全松开 A、B、Grip 和 Trigger。
4. 短按 A 请求解锁，等待后端确认。
5. 按住 Grip 在当前手柄/TCP 姿态建立零位，再缓慢平移。
6. 松开 Grip，机械臂必须停止；重新按住 Grip 会从当前 TCP 重锚定。
7. 先验收平移，再单独验收 roll/pitch/yaw，最后才做轻物体夹取。
8. B 的第一次操作用于停止；Home 只能在停止且 Grip 松开后请求。

## 7. 失联、停止和关机

下列事件必须停止并锁定：Grip 松开、Quest 页面关闭/刷新、XR 隐藏、跟踪丢失、WebSocket 断开、Wi-Fi 丢失、后端关闭或记录器不可用。

正常结束：

1. 松开 Grip；
2. 等待 HUD 显示已停止；
3. 退出 VR 并关闭 Quest 页面；
4. 在后端终端按 `Ctrl+C`；
5. 等待后端完成 stop、disconnect 和日志关闭后再断电。

若 stop 确认失败，后端会尝试 `stop_sys` 升级并保持故障锁定。不得通过刷新页面绕过故障。

## 8. 日志与反馈材料

每个真机会话位于：

```text
logs/commissioning/<session-id>/
  session.jsonl
  summary.json
  commissioning-report.md
```

实验后保留：

- `logs/preflight.json`；
- 完整会话目录；
- 填写完成的 `commissioning-report.md`；
- 必要的视频或照片；
- 后端终端输出。

不要上传包含密码、令牌或未批准网络信息的文件。首次实验反馈至少给出 Git 提交、配置哈希/参数版本、预检报告、会话目录、异常时间点和现场观察。

## 9. 当前变更回滚

若 `+roll 1°` 出现突跳、抖动、异常平移、停止失败或日志无法解释，立即停止并
将本地配置改回 `readonly`。代码回滚基线为诊断提交 `5e49d3a`；切换后必须
重新运行第 4 节只读预检，不得在旧版本下保留 control 模式继续试错。新增的
`max_joint_tracking_error_rad` 会被旧提交忽略，但可保留在未跟踪的本地 YAML
中。回滚和预检结果都要写入验收报告。
