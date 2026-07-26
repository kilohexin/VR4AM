# LM3 真机部署与首次实验流程

本文用于乐白 LM3、原厂 LMG-90 夹爪和 Meta Quest 3 的首次真机接入。默认运行模式仍是模拟器；只有本地配置、环境确认、后端预检、WebXR Grip 松开门和单控制端所有条件同时满足，真机才可能接收运动命令。

首次实验不要求眼在手外相机，也不要求相机标定。相机与时间同步接口继续保留，完成安全、稳定的遥操作后再接入 Gemini 330。

## 0. 必须遵守的边界

- 不在无人值守、急停不可达或工作区不清空时运行真机。
- 首次只读预检必须先于 control 模式。
- `real-robot.local.yaml`、机器人 IP 和现场测得参数不得提交到 Git。
- 不猜测 TCP、Home、软关节限位、启动 TCP 包络或夹爪幅值方向。
- 每次 smoke 进程只测试一个轴，观察员确认方向和停止后才能测试下一轴。
- Quest 控制时关闭 PC 控制网页，保持单一控制连接。
- 测试结果只有实际操作者可以在验收报告中勾选；自动测试不能代替真机验收。

## 1. 从私有 GitHub 仓库部署

以下 `<...>` 都是必须由操作者替换的占位符，不可原样执行。私有仓库需要先在服务器配置 GitHub 登录或 SSH/令牌凭据。

Linux：

```bash
git clone https://github.com/kilohexin/VR4AM.git
cd VR4AM
git checkout codex/teleoperation-ux-recovery
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
git checkout codex/teleoperation-ux-recovery
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
- 六轴软限位和限位余量；
- 当前初始姿态允许的 TCP 三轴启动包络；
- TCP 对比容差；
- 夹爪开/闭幅值方向。

`expected_tcp` 是工具 TCP 配置，不是当前末端实时位置。LMG-90 虽是标配，也必须用 `get_tcp()` 核对，不可凭外观猜测。

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

## 5. 切换 control 与单轴 5 mm smoke

只有验收报告中“只读阶段”和物理安全项全部通过后，才把本地 YAML 的 `mode` 改为 `control`。

PowerShell：

```powershell
$env:VR4ARM_REAL_ROBOT_CONFIRM = "I_UNDERSTAND_REAL_ROBOT_MOTION"
python .\scripts\real_robot_smoke.py `
  --config $env:VR4ARM_CONFIG `
  --axis x `
  --distance-m 0.005 `
  --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
```

Linux：

```bash
export VR4ARM_REAL_ROBOT_CONFIRM="I_UNDERSTAND_REAL_ROBOT_MOTION"
python scripts/real_robot_smoke.py \
  --config "$VR4ARM_CONFIG" \
  --axis x \
  --distance-m 0.005 \
  --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
```

先测试 `x`，进程结束且确认实际方向和停止后，再以新进程分别测试 `y`、`z`。脚本拒绝超过 `0.005 m` 的距离、错误确认词、只读配置、非 IDLE、TCP/Home 缺失或预检失败。

若方向错误、抖动、意外转动或停止不完整：

1. 立即按急停或使用现场批准的停止方式；
2. 不运行下一轴；
3. 将 YAML 改回 `readonly`；
4. 保存会话日志并在验收报告记录现象。

## 6. Quest 真机遥操作

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
