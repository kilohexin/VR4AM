# VR4Arm 真机动作上线与诊断界面设计

日期：2026-07-29
状态：已完成讨论确认，待用户审阅书面规格
目标分支：`codex/teleoperation-ux-recovery`
设计基线：`7db770a`

## 1. 目标

下一次具备 LM3 真机条件时，操作者至少能够使用 Meta Quest 3 原厂手柄，
通过现有 VR4Arm 系统安全地控制真实 LM3 完成：

- 末端前后、左右、上下平移；
- 末端 Roll、Pitch、Yaw 姿态调整；
- 原厂 LMG-90 夹爪开合；
- 显式 Home；
- Grip 松开、B 键、页面关闭和失联停止；
- 一次低速轻质方块抓取与释放。

现场开放 Quest 控制前，必须先用 PC 分动作工具完成小步平移、旋转、夹爪、
Home 和停止验证。

本阶段同时完成状态与日志显示：

- PC 提供完整真机诊断台；
- Quest 提供不遮挡主要视野的精简 HUD；
- 继续执行单控制端规则，PC 网页和 Quest 不同时控制或观察实时流。

## 2. 非目标

本阶段不实现：

- 训练数据、Episode 或数据集导出；
- Gemini 330 或其他相机接入；
- 图像与机器人状态时间同步；
- VLA、ACT、Diffusion Policy；
- ROS 2、MoveIt 或新的机器人中间件；
- 新的刚体物理引擎；
- 自动轨迹规划或批量动作脚本；
- 无现场人工确认的真机自动使能。

## 3. 已有基础

继续复用现有组件：

- FastAPI 与 WebSocket 单控制端；
- `RobotControl` 作为唯一运动授权者；
- 手柄/TCP 相对零位映射；
- `RealLebaiAdapter`；
- 乐白 SDK 延迟加载和能力检测；
- 官方 IK 到 PVAT 的真机控制路线；
- latest-wins PVAT 发送器；
- 关节连续性、软限位、速度和加速度限制；
- Grip、失联、故障、Home 和停止状态机；
- `LEBAI_READONLY`、`LEBAI_CONTROL` 双重使能；
- `CommissioningRecorder`；
- 虚拟 LM3/LMG-90 模型；
- Fake Lebai 自动测试；
- 虚拟 LM3 软件验收门禁。

现有真机控制代码并不等于真机已经验证。本阶段仍必须把软件离线通过和现场
硬件通过分开报告。

## 4. 方案选择

采用“强化现有 FastAPI + RealLebaiAdapter”路线。

不采用 Quest 指令直接转发 SDK，因为这会绕过现有预检、限速、停止确认和
控制代次。

不引入 ROS 2/MoveIt，因为当前目标是尽快完成 LM3 遥操作上线，重建通信和
控制栈会扩大风险与周期。

## 5. 生产控制架构

```text
Quest 3 原厂手柄
  -> WebXR / 单控制 WebSocket
  -> RobotControl 50 Hz
  -> 手柄/TCP 相对零位坐标映射
  -> 速度、加速度、步长、工作区限制
  -> RealLebaiAdapter
  -> 乐白官方 kinematics_inverse(actual_q)
  -> 六轴有限值、软限位、连续性和跳变检查
  -> latest-wins PVAT 25 Hz / 80 ms horizon
  -> LM3
  -> actual TCP、q/qd/qdd、机器人和夹爪状态
  -> RobotControl、Quest HUD、PC 诊断和异步日志
```

生产真机路径必须满足：

- 不调用仿真 IK 控制真实 LM3；
- 不从只读模式自动升级到控制模式；
- 不自动调用 `start_sys()`、`set_tcp()` 或 `init_claw()`；
- 不使用无限时长 `speedl`；
- 不因 PVAT 不可用而静默切换其他运动接口；
- 停止后所有旧 IK/PVAT 结果因控制代次失效；
- 真实 TCP、Home、限位和夹爪方向只能来自现场确认。

## 6. 离线真机链路数字孪生

### 6.1 目的

无真机时，让 Quest 和 PC 走与真机相同的上层路径：

```text
Quest / PC
  -> RobotControl
  -> RealLebaiAdapter
  -> Fake Lebai SDK 形状接口
  -> kinematics_inverse / move_pvat / stop_move / movej / set_claw
  -> Fake actual joint state
  -> 虚拟 LM3 可视化
```

这里验证的是接口、状态机、限速、代次、PVAT 参数和 UI 数据流。Fake IK
可委托当前虚拟 LM3 求解器，但它不代表真实控制器内部的精标 IK。

### 6.2 启动边界

增加单独的开发启动器，例如：

```powershell
python scripts/run_fake_lebai_stack.py
```

该启动器通过现有 `create_app(..., client_factory=...)` 注入 Fake 客户端。
生产 YAML 仍只允许 `simulator` 或 `lebai`，不能通过误填普通环境变量进入
Fake 模式。

运行身份必须显示为：

```text
LEBAI_FAKE / DIGITAL_TWIN / hardware_verified=false
```

不得显示为已连接真实 LM3。

### 6.3 Fake 行为

Fake 客户端支持：

- 状态、TCP、运动学和夹爪读取；
- SDK 形状的逆运动学调用；
- PVAT 目标随虚拟时间连续更新 actual q/qd/qdd；
- `movej` Home；
- `set_claw`；
- `stop_move` 和 `stop_sys`；
- 可配置 SDK 延迟、状态陈旧、IK 失败、PVAT 失败、断连和停止失败；
- 调用记录与实际状态分离，防止“发送成功”被误当成“实际已执行”。

虚拟 LM3 只根据 Fake 返回的 actual joint state 更新，不根据未执行的 target
直接跳动。

## 7. 现场分动作工具

扩展现有 `real_robot_smoke.py`，采用互斥子命令，每次进程只执行一个动作。

### 7.1 平移

```powershell
python scripts/real_robot_smoke.py translate `
  --config config/real-robot.local.yaml `
  --axis x `
  --distance-m 0.005 `
  --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
```

规则：

- 轴只允许 `x/y/z`；
- 符号表示正负方向；
- 绝对距离硬限制为 `<= 0.005 m`；
- 使用 actual TCP 生成目标；
- 经过生产 `RealLebaiAdapter`、官方 IK 和 PVAT；
- 达到稳定条件或超时后结束。

### 7.2 旋转

```powershell
python scripts/real_robot_smoke.py rotate `
  --config config/real-robot.local.yaml `
  --axis roll `
  --angle-deg 2 `
  --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
```

规则：

- 轴只允许 `roll/pitch/yaw`；
- 绝对角度硬限制为 `<= 2°`；
- 位置保持当前 actual TCP；
- 姿态增量使用与 Quest 工具坐标映射一致的定义；
- 每个方向使用新进程单独验证。

### 7.3 夹爪

```powershell
python scripts/real_robot_smoke.py gripper `
  --config config/real-robot.local.yaml `
  --target open `
  --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
```

规则：

- `target` 只允许 `open/close`；
- 幅值方向来自现场本地配置；
- 力度受配置上限约束，首轮不超过 `30%`；
- 发送后读取夹爪状态并记录；
- 不自动执行开合循环。

### 7.4 Home 与停止

```powershell
python scripts/real_robot_smoke.py home `
  --config config/real-robot.local.yaml `
  --confirm I_UNDERSTAND_REAL_ROBOT_MOTION

python scripts/real_robot_smoke.py stop `
  --config config/real-robot.local.yaml `
  --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
```

Home 只使用非空、有限、在软限位内且经现场批准的 `home_q`。Stop 调用
正常停止并等待实际关节速度连续稳定；确认失败沿用生产故障升级逻辑。

### 7.5 共同行为

每个动作都必须：

1. 校验 control 模式和固定确认短语；
2. 重新连接并执行完整预检；
3. 确认机器人 IDLE、无急停且状态新鲜；
4. 记录初始 actual TCP、q/qd/qdd 和夹爪；
5. 只执行一个受硬上限保护的动作；
6. 记录 SDK/IK/PVAT 延迟；
7. 等待实际状态稳定；
8. 生成机器可读摘要和人工确认项；
9. 无论成功或失败都关闭命令门并执行停止/断开清理。

不得提供多轴批量运行参数。

## 8. Quest 真机操作

现场 PC 分动作测试全部通过后，才开放 Quest。

操作语义保持：

- A：请求会话解锁；
- Grip 按下：同时捕获手柄零位和真实 actual TCP 零位；
- Grip 保持：控制六自由度相对增量；
- Grip 松开：立即撤销运动并停止；
- Trigger：控制夹爪；
- B：停止；停止后才允许显式 Home；
- 每次重新按 Grip 都从当前 actual TCP 重锚定。

验证顺序：

1. 平移，姿态锁定；
2. Roll、Pitch、Yaw 分别验证，位置保持；
3. 六自由度联合控制；
4. 夹爪；
5. 轻质方块抓取与释放；
6. Grip、页面关闭、WebSocket 和 Wi-Fi 失联停止。

## 9. 单控制端与显示策略

采用已确认的 A 型分层界面：

- PC：完整诊断台；
- Quest：精简可操作 HUD。

继续禁止 PC 网页和 Quest 同时保持实时控制/观察连接。

PC 页面用于预检、分动作和诊断。进入 Quest 前必须关闭 PC 页面。Quest
获得所有权后，新网页连接被拒绝，并显示：

```text
当前由 Quest 控制，请关闭电脑网页。
```

Quest 操作期间完整日志继续写到服务器 JSONL 和终端。退出 Quest 后，PC
可以重新连接并查看最近会话摘要。

## 10. 诊断数据架构

### 10.1 实时状态

现有 `RobotStateMessage` 继续承担约 20–25 Hz 的控制状态：

- backend、readonly/control；
- mode 和 robot_state；
- actual TCP 和 actual q；
- gripper；
- sample age；
- preflight；
- constraint、fault 和 recovery phase；
- ack sequence。

Quest 的链路延迟继续由现有 ping/ack 机制计算。

### 10.2 有界诊断存储

新增独立 `DiagnosticsStore`：

- 保存最新详细快照；
- 保存固定容量的最近事件环形缓冲；
- 普通事件合并/限频，最高约 `5 Hz`；
- 停止、故障、断连、预检和控制权事件立即保留；
- 不读取完整 JSONL 文件服务 UI；
- UI 消费失败不影响记录器；
- 诊断更新不得阻塞 50 Hz 控制循环。

详细快照包括：

- actual/target TCP；
- q/qd/qdd；
- SDK 状态、IK、PVAT 和停止调用延迟；
- PVAT 发送频率和最近序号；
- 状态年龄；
- 控制代次；
- 预检各项结果；
- TCP、Home、限位和夹爪配置摘要；
- 最近停止确认耗时；
- 日志会话目录和丢弃计数。

PC 页面只在其拥有当前连接、且 Quest 未占用时轮询详细快照和最近事件。

## 11. PC 诊断台

PC 页面包含：

- 顶部状态条：backend、运行身份、模式、控制权、预检、急停；
- TCP 卡片：actual/target 位置与姿态；
- 关节表：六轴 q/qd/qdd、软限位余量；
- 链路卡片：WebSocket、状态、SDK、IK、PVAT 延迟和频率；
- 安全卡片：Grip、状态新鲜度、约束、停止和 Home；
- 夹爪卡片：归一化值、原始幅值、力度和保持状态；
- 有界事件流：时间、级别、事件名、原因和序号；
- 日志位置和配置摘要；
- 明确的“为何不能解锁”原因。

PC 诊断台不提供绕过预检、解除硬故障或任意运动参数输入。

## 12. Quest HUD

Quest HUD 保留视野中心和夹爪目标区域，显示：

- `SIMULATOR/LEBAI/LEBAI_FAKE`；
- `readonly/control`；
- `READY/ACTIVE/HOLD/FAULT`；
- 预检、控制权、Grip 和停止状态；
- 简化 actual TCP；
- 夹爪状态；
- 总体链路延迟；
- 当前软约束或故障；
- 可执行的下一步提示。

示例：

```text
接近工作区边界：向反方向退回
IK 暂时不可达：保持 Grip，退回上一位置
状态已过期：机械臂已停止，请检查网络
停止确认失败：保持安全距离，检查 L Master/急停
需要 Home：松开 Grip 后按 B
```

Quest 不显示完整 q/qd/qdd、连续高速日志或长堆栈。

## 13. 错误处理

### 13.1 软约束

以下情况不立即锁死：

- 工作区边界；
- 单帧 IK 不可达；
- 接近软关节边界；
- 仿真自碰撞预检查。

行为：

- 不发送不安全目标；
- 保持最后安全目标或零速度；
- 保持足够信息让操作者向反方向退回；
- 连续有效状态满足清除时间后自动清除提示。

### 13.2 硬故障

以下情况撤销控制权并锁定：

- SDK 断连；
- actual state 持续陈旧；
- PVAT 连续失败；
- 非有限关节解；
- 关节解跳变；
- 真实机器人 ERROR/ESTOP；
- 无法确认停止；
- 关键日志持续不可用。

行为：

1. 原子关闭命令门；
2. 增加控制代次；
3. 清空 latest-wins 槽；
4. 调用停止；
5. 读取 actual qd 确认；
6. 必要时升级 `stop_sys`；
7. 保持结构化故障和人工恢复提示。

网页刷新不能清除硬故障。

## 14. 自动测试

普通自动测试不连接真机，不导入真实 SDK。

新增测试覆盖：

- Fake actual state 只随已接受 PVAT/Home 变化；
- 完整 Quest WebSocket 到 RealLebaiAdapter/Fake 客户端路径；
- X/Y/Z 正负平移；
- Roll/Pitch/Yaw 正负旋转；
- 夹爪、Home 和停止；
- 动作 CLI 的参数硬上限和单动作约束；
- SDK 延迟、IK 失败、PVAT 失败、状态陈旧和断连；
- 停止后旧代次不能产生 PVAT；
- PC/Quest 单控制权；
- DiagnosticsStore 有界、限频且关键事件不丢；
- PC 诊断台字段和 Quest HUD 提示；
- 完整后端测试、前端测试和生产构建。

不增加与目标无关的复杂压力测试。主体控制链至少完成一次确定性的端到端
离线运行。

## 15. 离线验收报告

增加真机链路离线验收入口，报告至少包含：

- Git commit 和 dirty paths；
- 运动学和 GLB 哈希；
- Fake SDK 配置；
- 六向平移和三轴旋转结果；
- 夹爪、Home 和停止结果；
- 故障注入结果；
- 后端、前端和构建结果；
- 诊断 UI 结果；
- `hardware_verified: false`；
- 所有现场待验项。

数字孪生通过只表示真机代码路径的软件行为已验证。

## 16. 现场放行顺序

1. `LEBAI_READONLY` 读取并保存真实 TCP、q/qd/qdd、机器人、急停和夹爪；
2. 填写并人工审查本地 TCP、Home、软限位、启动包络和夹爪方向；
3. PC 执行 +X/-X、+Y/-Y、+Z/-Z，每次最多 5 mm；
4. PC 执行正负 Roll、Pitch、Yaw，每次最多 2°；
5. 夹爪低力度 open/close；
6. Home 与正常 Stop；
7. Quest 平移，姿态锁定；
8. Quest 分别验证 Roll/Pitch/Yaw；
9. Quest 六自由度和夹爪；
10. Grip、页面、WebSocket 和 Wi-Fi 失联停止；
11. 低速轻质方块抓取与释放。

任一阶段失败，不进入下一阶段。

## 17. 完成标准

### 17.1 无真机阶段

- Quest 指令通过生产 RobotControl 和 RealLebaiAdapter 驱动数字孪生；
- 六向平移、三轴旋转、夹爪、Home 和停止离线通过；
- PC 分动作工具具有硬上限和单动作约束；
- PC 诊断台与 Quest HUD 完成；
- 故障注入不会留下旧运动指令；
- 完整自动测试和构建通过；
- 离线报告明确 `hardware_verified: false`。

### 17.2 现场阶段

只有实际完成以下项目，才允许记录 `hardware_verified: true`：

- 只读预检和真实配置确认；
- PC 六方向 5 mm；
- PC 三轴小角度旋转；
- 夹爪和 Home；
- Quest 六方向；
- Quest Roll/Pitch/Yaw；
- Grip、页面和失联停止；
- 一次低速轻质抓取与释放。

自动测试、Fake SDK 或数字孪生不能代替上述现场结论。

## 18. 安全与版本控制边界

- 不提交机器人 IP、确认短语或现场本地 YAML；
- 不修改用户当前未提交的 `README.md`、`backend/app/sim/ik.py` 和两份
  2026-07-22 草稿；
- 每个实现任务独立本地提交；
- 不自动合并或 push；
- 不为了合并或报告修改测试结果；
- 所有通过结论必须来自实际运行的命令输出。
