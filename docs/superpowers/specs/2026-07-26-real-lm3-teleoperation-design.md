# VR4Arm 真机 LM3 遥操作与首次实验设计

日期：2026-07-26

状态：已讨论确认，待用户审阅本文档

目标分支基线：`codex/teleoperation-ux-recovery`

## 1. 目标

在保留现有 Quest 3 WebXR 遥操作体验和仿真展示的前提下，为乐白 LM3 与原厂 LMG-90 夹爪增加可审计、默认不可运动、可分阶段验收的真机控制路径。

第一阶段只完成：

- 真机只读连接、预检和状态采集；
- Quest 3 原厂手柄控制真实 LM3 的六自由度 TCP；
- LMG-90 低力度开合；
- 使用乐白官方 IK 和 PVAT 进行连续关节伺服；
- Grip 松开、WebXR 失联、SDK 异常和机器人异常的停止；
- Home 基线及显式恢复；
- Markdown 人工验收表、JSON 摘要和 JSONL 自动日志；
- Fake SDK 自动测试与实验室分阶段冒烟实验。

## 2. 非目标

本阶段不实现：

- Gemini 330 RGB-D 采集；
- 眼在手上或眼在手外的手眼标定；
- 图像与机器人状态的数据集导出；
- VLA、ACT 或 Diffusion Policy 训练格式；
- VR HUD 或 PC 网页的详细状态/日志面板；
- 用仿真 IK、碰撞胶囊或虚拟关节参数控制真机；
- 完整动力学或物理仿真。

现有仿真继续用于展示交互效果、验证 WebXR 输入和离线控制逻辑。

## 3. 已确认的现场条件

- 机器人：乐白 LM3。
- 夹爪：原厂 LMG-90，直接安装在法兰上，无加长或自制指尖。
- TCP：优先采用并校验 L Master 中的 LMG-90 原厂 TCP。
- 操作端：Meta Quest 3 原厂手柄和 WebXR。
- 后端：实验室服务器或电脑上的 FastAPI。
- 网络：Quest、服务器和 LM3 位于同一局域网；LM3 优先有线，Quest 使用稳定的 5 GHz/6 GHz Wi-Fi。
- 首次真机实验：一人佩戴 Quest，一人站在安全区域外观察并守住物理急停。
- 第一阶段：只做真机遥操作与实验，保留相机/时间同步接口，不启用相机数据流。

以下信息在回到实验室后补录，不在设计阶段猜测：

- LM3/L Master 系统版本；
- `lebai_sdk_asyncio` 版本；
- 机器人 IP；
- `get_tcp()` 返回值；
- LMG-90 负载质量和重心；
- 经人工确认的安全 Home 关节角；
- L Master 中的关节安全范围和速度比例。

## 4. TCP 与坐标定义

### 4.1 TCP 来源

LMG-90 是原厂直装夹爪，第一阶段不要求制作专用四点标定针。使用 L Master 中已启用的原厂 LMG-90 TCP，并将 `get_tcp()` 的实际返回值保存为本项目的真机基准。

乐白文档中的 `x=0, y=0, z=0.175 m, rz=0, ry=0, rx=0` 仅作为合理性参考，不作为未经现场确认即可写入真机的默认值。

程序遵循以下规则：

- 连接时只读取 `get_tcp()`；
- 不自动调用 `set_tcp()`；
- 当前 TCP 与保存基准不匹配时拒绝使能；
- 位置容差初值为 `1 mm`；
- 姿态容差初值为 `0.5°`；
- 更换指尖、转接件或夹爪安装方式后必须重新确认 TCP。

### 4.2 控制坐标

真机反馈采用机器人基坐标系中的 `actual_tcp_pose`。WebXR 继续传输原始手柄位姿；Grip 按下时，后端同时捕获：

- 当前手柄位姿 `T_hand_0`；
- 真机反馈 TCP 位姿 `T_tcp_0`；
- 当前实际关节角 `q_actual_0`。

后续目标为相对零位映射：

```text
T_tcp_target = T_tcp_0 · calibrated_delta(T_hand_0, T_hand_now)
```

平移采用用户直觉方向；姿态采用已确认的手柄局部坐标到 LMG-90 工具坐标的固定旋转标定。手柄前伸对应夹爪向前，手柄绕自身前向轴翻滚对应夹爪翻滚。

真机路径必须以现场低速轴向实验确认映射。不得仅根据仿真视觉方向推断真机方向。

## 5. 总体架构

```text
Quest 3 / WebXR (60–72 Hz)
  -> FastAPI WebSocket，只保留最新有效帧
  -> RobotControl (50 Hz)，唯一运动授权者
  -> 坐标映射、软工作区、速度/加速度限制
  -> RealLebaiAdapter
  -> 乐白官方 IK，以 actual_joint_pose 为参考
  -> 关节连续性、软限位、速度和加速度检查
  -> PVAT 发送器（首次 25 Hz，80 ms 时间窗）
  -> LM3
  -> get_robot_state/get_estop_reason/get_kin_data/get_claw
  -> RobotControl、WebSocket 状态和异步日志
```

### 5.1 组件职责

`RobotControl`

- 唯一允许产生运动、夹爪和停止请求的组件；
- 保持现有 Grip 零位、使能、失联、Home 和故障语义；
- 维护控制代次 token，停止后让所有旧异步结果失效；
- 不直接依赖乐白 SDK。

`RealLebaiAdapter`

- 延迟导入 `lebai_sdk_asyncio`；
- 管理 SDK 连接与能力检测；
- 读取机器人、TCP、运动学和夹爪状态；
- 转换四元数与乐白 Euler ZYX；
- 调用官方逆运动学；
- 生成和发送受限 PVAT；
- 实现 `stop_move()`、显式 Home、夹爪命令及统一错误转换。

`Lebai SDK 所有者`

- 单一协程串行拥有 SDK 客户端；
- 禁止 FastAPI 请求、状态轮询和运动发送并发调用同一个客户端；
- 使用有界 latest-wins 命令槽，禁止积压旧运动目标；
- 停止请求优先清空运动槽；
- SDK 调用超时后不假定取消成功，由 PVAT 超时、状态确认和物理急停形成后备保护。

`CommissioningRecorder`

- 使用有界异步队列写 JSONL；
- 不阻塞 50 Hz 控制循环；
- 普通状态样本允许降采样；
- 使能、停止、故障和失联事件不可丢弃；
- 日志无法创建或持续写入失败时停止并禁止 ACTIVE。

## 6. 运行模式与双重使能

### 6.1 `SIMULATOR`

- 保持当前仿真行为；
- 不安装、不导入、不连接乐白 SDK；
- 仿真 IK 只影响虚拟机械臂。

### 6.2 `LEBAI_READONLY`

- 允许连接 SDK并读取状态、TCP、运动数据和夹爪；
- 所有运动、Home 和夹爪写命令明确拒绝；
- 首次实验必须先运行此模式；
- 默认真机配置模式。

### 6.3 `LEBAI_CONTROL`

必须同时满足：

1. 配置文件显式设置 `mode: control`；
2. 环境变量提供固定的真机确认短语；
3. 启动预检通过；
4. 操作者在当前会话显式使能；
5. 首个有效 WebXR 帧显示 Grip 已松开。

仅修改 YAML 不能让机器人运动。程序不会从只读模式自动升级，也不会因 PVAT 不可用而静默切换到其他运动接口。

连接 SDK 不自动调用 `start_sys()`。机器人必须由现场人员在 L Master 中启动并确认进入 `IDLE`。

## 7. 启动预检

进入可使能状态前必须验证：

- SDK 连接和能力检测成功；
- `get_robot_state()` 为 `IDLE`；
- 没有正在执行的运动；
- `get_estop_reason()` 表示无急停；
- `get_tcp()` 与人工确认的 LMG-90 基准一致；
- `actual_joint_pose`、`actual_joint_speed`、`actual_tcp_pose` 全部有限；
- 实际关节角位于配置的软限位内并保留安全余量；
- 实际 TCP 位于配置的启动包络内；
- Home 关节角已人工确认；
- 日志目录创建并写入探测成功；
- WebXR 跟踪有效且 Grip 松开；
- 当前会话获得单控制端所有权。

任一项失败只允许保持只读，并将结构化原因写入日志和状态消息。

## 8. PVAT 控制路线

### 8.1 为什么选择 PVAT

第一阶段主路线为：

```text
受限 TCP 目标
  -> lebai.kinematics_inverse(target_tcp, actual_q)
  -> 关节解连续性检查
  -> 受限 q/qd/qdd
  -> move_pvat(q, qd, qdd, t)
```

原因：

- 使用真机内部精标运动学和当前 TCP；
- 能显式限制每个关节的位置、速度与加速度；
- 适合连续跟随和后续动作数据记录；
- 官方说明相邻 PVAT 点发送间隔超过 `t` 时机器人自动减速停止。

`towardj` 仅作为未来经过现场能力验证和人工配置的备用，不自动降级。第一阶段禁止无限时长 `speedl(t=0)`。

### 8.2 初始频率与限制

首次实验采用：

- RobotControl：`50 Hz`；
- 真机状态轮询：目标 `25–50 Hz`，以实测 SDK 延迟为准；
- PVAT 下发：`25 Hz`；
- PVAT 时间窗：`80 ms`；
- 最大 TCP 平移速度：`0.03 m/s`；
- 最大 TCP 旋转速度：`0.25 rad/s`；
- 最大关节速度：`0.15 rad/s`；
- 最大关节加速度：`0.5 rad/s²`；
- 单控制周期最大 TCP 位移：`2 mm`；
- 单控制周期最大姿态变化：`1°`；
- 手柄平移比例：`0.5`；
- 相对零位每轴最大位移：`±0.10 m`；
- 相对零位最大旋转：`±30°`。

首轮通过后再逐项增加，目标平移速度不一次性超过 `0.08–0.12 m/s`，平移比例再调整到 `1.0`。

### 8.3 IK 与连续性

每个 IK 请求都使用最新 `actual_joint_pose` 作为参考。结果必须：

- 恰好包含六个有限关节值；
- 位于现场配置的软关节范围内；
- 与最新实际关节角和上一已发送目标连续；
- 不超过单周期关节变化限制；
- 经速度和加速度限制后才能进入 PVAT。

单帧不可达或软工作区越界不进入锁定故障：保持最后一个安全目标、输出零速度，并允许手柄退回。连续 IK 失败、解跳变或状态陈旧时停止并解除使能。

真机路径不使用仿真 DH、虚拟关节窗口或仿真自碰撞胶囊做最终安全判断。

## 9. 安全状态机

```text
DISCONNECTED
  -> READ_ONLY
  -> DISARMED
  -> READY
  -> ACTIVE
  -> STOPPING
  -> DISARMED
```

另有：

- `HOMING`：仅显式 Home 请求进入；
- `FAULT`：锁定故障，需要人工检查和恢复；
- `DISCONNECTED`：SDK 断开后禁止自动恢复 ACTIVE。

关键转换：

- Grip 按下：在 READY 捕获手柄/TCP/关节零位后进入 ACTIVE；
- Grip 松开：立即撤销命令代次并进入 STOPPING；
- WebSocket 断开、跟踪无效、帧超时、页面隐藏：STOPPING；
- 机器人进入 `ERROR`、`ESTOP`、`DISCONNECTED`：FAULT；
- SDK 连续异常、关节跳变、无法确认停止：FAULT；
- 重连后：只能回到 READ_ONLY/DISARMED，必须重新预检和使能。

## 10. 停止与故障升级

例行停止顺序：

1. 原子地关闭本地命令门；
2. 增加控制代次，使旧异步结果失效；
3. 清空 PVAT latest-wins 槽；
4. 调用 `stop_move()`；
5. 轮询实际关节速度；
6. 连续约 `300 ms` 低于阈值后确认停止；
7. 清除 Grip 零位并进入 DISARMED。

若 SDK 连接正常但约 `500 ms` 后机器人仍在运动：

- 调用 `stop_sys()`；
- 锁定 FAULT；
- 要求人工在 L Master 和现场检查。

若网络已中断，软件无法保证发出任何停止指令。此时依赖 PVAT 点位超时自动减速和现场观察员的物理急停。

程序不把 `estop()` 当作普通停止，因为软急停会下电刹车并可能产生轻微下落。

`stop_move()` 不能阻止后续新指令，因此控制代次 token 是强制要求，不能只依赖 SDK 停止调用。

## 11. Home 与夹爪

### 11.1 Home

- Home 必须是现场人员在 L Master 中确认的六关节位置；
- 未配置 Home 时真机 Home 请求拒绝；
- Home 仅允许在 Grip 松开、机器人已停止、无其他运动时执行；
- 使用受限 `movej` 关节运动，不使用 TCP IK；
- Home 期间持续监测状态和实际关节速度；
- 超时、急停或 SDK 异常立即进入停止/故障流程；
- 不从故障姿态自动回 Home；
- 第一阶段实验允许优先在 L Master 中人工回 Home。

### 11.2 LMG-90

- 不在启动时强制 `init_claw(True)`；
- 注意机器人上电或急停恢复后夹爪可能自动张合确认行程；
- 第一轮夹爪力度上限为 `20–30%`；
- 夹爪命令目标频率为 `10 Hz`；
- 读取并记录 `force`、`amplitude` 和 `hold_on`；
- 夹爪错误不应阻塞急停/停止通道。

## 12. 配置设计

真机配置单独保存在不提交 Git 的本地 YAML 中；仓库只提交示例文件。

```yaml
backend: lebai

real_robot:
  mode: readonly
  ip: 192.168.x.x

  expected_tcp:
    x: 0.0
    y: 0.0
    z: 0.175
    rz: 0.0
    ry: 0.0
    rx: 0.0

  home_q: []
  tcp_position_tolerance_m: 0.001
  tcp_rotation_tolerance_deg: 0.5

  control:
    loop_hz: 50
    state_hz: 25
    pvat_send_hz: 25
    pvat_horizon_s: 0.08
    max_tcp_speed_mps: 0.03
    max_tcp_rotation_radps: 0.25
    max_joint_speed_radps: 0.15
    max_joint_acceleration_radps2: 0.5
    translation_scale: 0.5

  gripper:
    max_force_percent: 30
    command_hz: 10
```

示例中的 TCP 不是部署默认真值。实际配置必须由 `LEBAI_READONLY` 输出和人工确认产生。

## 13. 日志与实验反馈

仓库跟踪人工模板：

```text
docs/real-robot-commissioning-report.md
```

运行时生成且默认不提交 Git：

```text
logs/commissioning/<session-id>/
  commissioning-report.md
  summary.json
  session.jsonl
  attachments/
```

### 13.1 JSONL 事件

- `session_started`
- `sdk_connected`
- `capability_detected`
- `preflight_result`
- `robot_state_sample`
- `vr_input_sample`
- `tcp_target_limited`
- `ik_result`
- `pvat_sent`
- `gripper_command`
- `state_transition`
- `soft_constraint`
- `stop_requested`
- `stop_confirmed`
- `sdk_error`
- `robot_fault`
- `log_drop_summary`
- `session_ended`

每条事件包含适用的：

- UTC 时间；
- PC 单调时钟；
- Quest 会话、序号和客户端时间；
- RobotControl 状态和控制代次；
- actual/target TCP；
- actual/target 关节角、速度和加速度；
- SDK 调用耗时；
- 限速、拒绝、停止或故障原因；
- 日志队列丢弃计数。

`summary.json` 汇总：

- Git commit；
- OS、Python、Node、L Master 和 SDK 版本；
- TCP、Home 和配置摘要；
- SDK 延迟分位数；
- 控制帧和 PVAT 计数；
- 最大目标/实际跟踪误差；
- 每次停止的确认延迟；
- 故障、软约束和日志丢弃计数。

人工报告记录现场方向是否正确、机械臂是否抖动/跳变、停止是否及时、夹爪是否正确、视频附件和主观操控感受。

## 14. 测试边界

普通自动测试永远不连接真机，使用 Fake Lebai SDK 验证：

- TCP/Euler ZYX/四元数转换；
- SDK 能力检测；
- 只读模式拒绝所有写操作；
- 官方 IK 的参数、返回值和错误转换；
- 关节解连续性及跳变拒绝；
- PVAT 参数、频率和限速；
- Grip 松开、帧超时和断线停止；
- 停止后旧控制代次不能发命令；
- TCP 不匹配拒绝使能；
- 日志关键事件不可丢；
- 一条 WebSocket 到 Fake SDK 的端到端冒烟链路。

真机测试必须通过单独脚本、显式环境变量和人工步骤触发，普通 `pytest` 不得包含机器人 IP 或 SDK 真连接。

验证报告必须区分：

- 新增真机功能的聚焦测试；
- 当前仓库的完整后端测试；
- 当前仓库的完整前端测试；
- 实验室只读测试；
- 实验室运动冒烟测试。

不得修改测试期望来迎合实现，不得将部分测试通过描述为完整测试通过。当前已知测试基线问题在实施前单独处理或明确记录。

## 15. 实验室部署

仅从 GitHub `git pull` 不能保证立即运行。服务器首次需要完成一次环境安装和本地配置。

### 15.1 一次性准备

1. 从 GitHub 克隆仓库或拉取目标分支和明确 commit；
2. 安装项目要求的 Python 版本并创建虚拟环境；
3. 安装 backend 及开发/测试依赖；
4. 根据乐白官方发布方式安装与现场系统兼容的 `lebai_sdk_asyncio`；
5. 在 `web/` 运行 `npm ci`；
6. 放通 Quest 访问前端 HTTPS 端口；
7. 确认服务器能访问 LM3 IP；
8. 为 Quest 建立可信或已人工接受的 HTTPS 访问方式；
9. 从仓库示例复制本地真机 YAML，先设置 `mode: readonly`；
10. 设置 `VR4ARM_CONFIG` 指向本地 YAML。

当前 Vite 开发服务器通过 HTTPS 提供 Quest 页面，并将 `/ws` 代理到同机 `127.0.0.1:8000`，因此 LM3 SDK、FastAPI 和 Vite 应在同一台实验室服务器上运行。

### 15.2 每次运行

1. 拉取并确认 Git commit；
2. 激活 Python 虚拟环境；
3. 检查本地配置仍为期望模式；
4. 启动 FastAPI；
5. 启动 HTTPS 前端；
6. 先检查健康状态和日志目录；
7. Quest 打开服务器 HTTPS 地址；
8. 只读预检通过后才允许进入下一阶段。

### 15.3 从只读切换到控制

只有完成只读报告并填入现场 TCP/Home 后：

1. 修改本地配置为 `mode: control`；
2. 设置真机控制确认环境变量；
3. 由观察员确认物理急停；
4. 重启后端；
5. 重新执行完整预检；
6. 按分阶段实验执行，不直接进入完整 VR 遥操作。

部署文档最终提供 Linux/PowerShell 两套明确命令，但不会把机器人 IP、确认短语或现场 TCP 提交到 GitHub。

## 16. 分阶段真机实验

1. **只读观察**：持续约 5 分钟记录状态、TCP、关节和 SDK 延迟。
2. **停止接口**：机器人静止时调用停止，确认状态和日志。
3. **夹爪测试**：无物体、低力度、低幅度。
4. **Home 基线**：在 L Master 中人工确认并记录。
5. **PC 小步 PVAT**：不戴 VR，单轴移动约 `5 mm`。
6. **VR 平移**：锁定姿态，验证前后、左右和上下。
7. **VR 旋转**：固定位置，验证翻滚、俯仰和偏航。
8. **六自由度低速遥操作**。
9. **失联与停止**：Grip、页面关闭、Wi-Fi、后端停止依次测试。
10. **轻质抓取**：最后抓海绵或轻质方块。

每阶段失败立即停止，不进入下一阶段。

## 17. 验收标准

第一阶段完成需满足：

- `SIMULATOR` 不安装 SDK仍可运行；
- `LEBAI_READONLY` 不可能产生运动；
- TCP 不匹配不能使能；
- 真机使用官方 IK 和 PVAT，不使用仿真 IK；
- 手柄六方向和三种旋转方向经现场确认；
- IK 无明显解跳变；
- Grip 松开、帧超时和 WebSocket 断开均关闭命令门；
- 例行停止有实际关节速度确认和延迟记录；
- 旧异步命令不能在停止后重新发送；
- 夹爪力度受配置上限约束；
- Home 只使用人工确认的关节位置；
- 自动日志和人工报告能完整复现实验过程；
- 完成一次低速轻质物体抓取与释放；
- 所有测试结果按真实范围报告。

## 18. 延后接口

本阶段保留但不启用：

- 相机帧记录接口；
- PC 单调时钟、Quest 客户端时钟和未来相机硬件时间戳字段；
- VR/网页详细机器人状态和日志面板的数据源；
- 数据集 episode/session 标识。

未来增加 Gemini 330 时，优先从眼在手外全局相机开始；相机外参与 TCP 是独立标定，不影响第一阶段真机遥操作上线。

## 19. 官方依据

- Python SDK 简介：https://help.lebai.ltd/sdk/python/introduce.html
- Python 初始化连接：https://help.lebai.ltd/sdk/python/init.html
- Python 状态数据：https://help.lebai.ltd/sdk/python/status.html
- Python 系统控制：https://help.lebai.ltd/sdk/python/control.html
- Python 运动接口：https://help.lebai.ltd/sdk/python/motion.html
- Python 位置和位姿：https://help.lebai.ltd/sdk/python/posture.html
- Python TCP 配置：https://help.lebai.ltd/sdk/python/setting.html
- Python 夹爪接口：https://help.lebai.ltd/sdk/python/claw.html
- L Master TCP 设置：https://help.lebai.ltd/guide/using.html
- 位置和姿态定义：https://help.lebai.ltd/guide/pose.html
