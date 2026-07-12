# Quest 3 遥操作乐白 LM3 系统设计

日期：2026-07-12  
状态：设计已确认，等待书面规格复核  
项目目录：`D:\MyWork\VR4Arm`

## 1. 背景与目标

本项目使用 Meta Quest 3 原厂 Touch Plus 手柄遥操作乐白 LM3 六轴机械臂及其原厂夹爪。系统首先在纯 VR 中控制虚拟 LM3，验证坐标映射、离合控制、滤波、限速、失联停止和交互体验；通过分阶段验收后，再以 MR 透视形式低速控制真机。

长期目标是在同一架构中加入固定 RGB/RGB-D 相机和统一时间戳记录，为 ACT、Diffusion Policy、VLA 等模仿学习或视觉语言动作模型采集示范数据。

第一实施里程碑不连接真机，只交付桌面浏览器和 Quest 3 纯 VR 运动学仿真。第二里程碑加入 Cannon-es 虚拟物体、碰撞与抓取。真机 MR 遥操作属于后续独立里程碑，必须在仿真验收全部通过后实施。

## 2. 设计原则

1. **同一控制链切换仿真与真机。** Quest 输入、消息协议、RobotControl、安全状态机和状态反馈不得因后端不同而分叉。
2. **仿真状态由 PC 后端权威维护。** Quest 只采集输入和渲染，不能成为机器人安全状态的唯一来源。
3. **网络输入永远不直接调用机械臂。** FastAPI 只负责连接、校验和状态分发；固定周期 RobotControl 负责生成安全目标。
4. **只消费最新输入。** VR 帧使用容量为 1 的 latest-value 通道，不累计旧目标。
5. **默认失败即停止。** 追踪丢失、页面隐藏、WebSocket 断开、输入过期、后端退出或机器人故障均关闭命令门并停止运动。
6. **重连不得自动续跑。** 恢复后必须松开 Grip、重新解锁并建立新锚点。
7. **MR 显示对齐不参与控制计算。** 虚拟模型放置误差只能影响显示，不能改变真机目标。
8. **仿真不冒充安全认证。** 运动学数字孪生用于降低调试风险，不能证明真机无碰撞、无奇异或动力学安全。

## 3. 范围

### 3.1 第一里程碑包含

- TypeScript、Vite、Three.js WebXR 前端。
- Quest 3 原厂右手柄六自由度位姿和按键读取。
- Grip 离合式相对控制。
- Trigger 到虚拟夹爪连续开合映射。
- FastAPI WebSocket 接入层。
- Python 50 Hz RobotControl。
- `SimRobotAdapter` 运动学数字孪生。
- 理论 LM3 改进 DH 前向运动学、完整 TCP 四元数和六维数值 IK。
- 关节、TCP、夹爪、连接、延迟和故障状态回传。
- 坐标、速度、加速度、工作空间和看门狗限制。
- 桌面浏览器调试模式与 Quest 纯 VR 模式。
- 版本化消息协议和跨端契约测试。
- `RecorderSink`、相机帧和真实机器人适配器的稳定接口占位。

### 3.2 第一里程碑不包含

- 真实 LM3 运动命令。
- Cannon-es 物体物理、碰撞和抓取。
- 自碰撞或环境碰撞安全证明。
- 高保真电机、力矩、摩擦或接触动力学。
- 固定相机采集和训练数据集导出。
- 毫米级 MR 注册。
- ROS 2、MoveIt、MuJoCo 或 Isaac Sim。

### 3.3 后续里程碑

1. Cannon-es 虚拟桌面、物体、夹爪碰撞体和抓取约束。
2. `RealLebaiAdapter` 真机只读连接与状态监控。
3. MR 手动基座对齐、真机不使能校准和低速单轴测试。
4. MR 六自由度真机遥操作和真实抓取。
5. 多相机、统一时间轴、episode 记录及 LeRobot/HDF5/Zarr 转换。

## 4. 最终用户体验

### 4.1 VR 仿真模式

用户在 Quest 3 浏览器中打开 HTTPS 页面并进入 `immersive-vr`。场景包含虚拟工作台、虚拟 LM3、原厂夹爪外观、目标 TCP 幽灵、工作空间边界和状态 HUD。

用户显式解锁后，按住右手 Grip 建立手柄与虚拟 TCP 锚点。按住期间，机械臂跟随手柄相对位姿；松开后停止。右手 Trigger 控制夹爪开合。用户可以自由移动手柄，再次按住 Grip 继续操作，不要求手柄绝对位置与机械臂绝对位置重合。

第一里程碑中场景没有受物理引擎驱动的可抓取物体。第二里程碑加入 Cannon-es 后，用户可抓取虚拟方块或圆柱并放入目标区域。

### 4.2 MR 真机模式

后续真机阶段使用 `immersive-ar` 透视。用户直接看到真实机械臂和物体，虚拟层只叠加目标 TCP、方向箭头、安全工作空间、延迟、连接和故障提示。系统不得使用不透明虚拟机械臂遮挡真实机械臂。

系统默认始终进入仿真模式。切换真机需要 PC 与 Quest 两侧显式确认，且任何断线或刷新都会退回未解锁状态。

### 4.3 非目标使用形式

当前设计不是远程视频遥操作系统。操作者位于机械臂附近并通过 Quest 透视观察真实工作区。若以后需要异地控制，应另行设计低延迟相机传输、深度感知、网络服务质量和远程安全审批。

## 5. 总体架构

```text
Quest 3 WebXR
  手柄位姿 / Grip / Trigger / tracking / visibility
                  │
                  │ WSS，60 Hz 上行；20 Hz 状态下行
                  ▼
FastAPI 接入层
  WebSocket、消息校验、session、健康检查
                  │
                  │ latest-value，容量 1
                  ▼
RobotControl，50 Hz
  锚点、坐标映射、滤波、步长限制、状态机、看门狗
                  │
          RobotBackend Protocol
             ┌────┴────┐
             ▼         ▼
   SimRobotAdapter   RealLebaiAdapter
   第一里程碑启用     后续启用
             │         │
             └────┬────┘
                  ▼
RobotState / RecorderSink / Quest 渲染
```

## 6. 技术栈

### 6.1 前端

- TypeScript 5.x
- Vite 7.x
- Three.js
- WebXR Device API
- Vitest
- `Lebai_LM3.glb` 视觉模型
- 第二里程碑：Cannon-es

第一里程碑不使用 React，以保持 XR 渲染和输入循环简单。若以后增加复杂管理 UI，可在不影响 XR 核心循环的前提下单独引入 UI 框架。

### 6.2 后端

- Python 3.11
- FastAPI
- Uvicorn
- Pydantic v2
- asyncio
- NumPy
- SciPy Rotation 或等价经过测试的四元数/旋转向量实现
- pytest、pytest-asyncio、httpx
- structlog 或标准库结构化 JSON logging
- 后续真机：`lebai_sdk_asyncio`

### 6.3 网络

- Quest 与 PC 位于同一受信局域网。
- WebXR 页面通过 HTTPS 提供。
- 遥操作使用单个 `wss://` WebSocket。
- 开发环境可使用局域网可信证书或 Quest 可接受的开发证书流程。
- 后端不暴露到公网，不在第一里程碑加入用户账户系统。

## 7. 项目结构

```text
VR4Arm/
  backend/
    pyproject.toml
    app/
      main.py
      config.py
      api/
        health.py
        teleop_ws.py
      control/
        robot_control.py
        coordinate_mapper.py
        filters.py
        safety.py
        state_machine.py
      robots/
        base.py
        sim_adapter.py
        lebai_adapter.py
      sim/
        lm3_model.py
        kinematics.py
        ik.py
        virtual_robot.py
      recording/
        base.py
        noop.py
      schemas/
        messages.py
      timebase.py
    tests/
      contract/
      control/
      sim/
      api/
  web/
    package.json
    public/
      models/Lebai_LM3.glb
    src/
      main.ts
      xr/
        session.ts
        controllerInput.ts
      transport/
        teleopSocket.ts
      robot/
        robotModel.ts
        robotState.ts
      scenes/
        simulationScene.ts
        mrScene.ts
      ui/
        hud.ts
        armPanel.ts
      protocol/
        messages.ts
    tests/
  schemas/
    teleop-v1.json
    fixtures/
  config/
    default.yaml
  docs/
```

`lebai_adapter.py` 在第一里程碑中只实现接口占位，并且任何调用都会明确返回 `real_robot_disabled`。第一里程碑不得安装或导入真实乐白 SDK。

## 8. 参考项目复用策略

参考项目：`D:\MyWork\ArmPlannerAgent\.worktrees\phase-b-reproducible-system`。

可迁移内容：

- `web/public/models/Lebai_LM3.glb`
- `robotModel.ts` 中 `Joint1` 至 `Joint6` 节点查找、视觉转轴、`setJointAngles()`、夹爪动画和 TCP 节点读取思路
- Three.js 场景初始化和模型加载方式
- 近似运动学的测试用例和几何常量，作为交叉检查材料
- 虚拟 SDK 的方法命名和 FastAPI 分层思路

不得直接沿用的行为：

- REST 轮询机器人状态
- 命令调用后立即跳到终点并标记 `finished`
- 只约束位置与末端方向的近似 IK 作为六维真值
- 六个关节统一使用 `[-π, π]` 占位限制
- 让 Web 前端成为机器人状态权威源

VR4Arm 不在运行时依赖 ArmPlannerAgent 路径。迁移的模型和代码应进入当前仓库，并在文档中记录来源；两个项目以后确有共同维护需求时再提取共享包。

## 9. 核心领域接口

### 9.1 RobotBackend

```python
class RobotBackend(Protocol):
    async def connect(self) -> None: ...
    async def disconnect(self) -> None: ...
    async def command_tcp(self, target: TcpPose, command_id: int) -> None: ...
    async def set_gripper(self, value: float) -> None: ...
    async def stop(self, reason: StopReason) -> None: ...
    async def get_state(self) -> RobotState: ...
```

实现：

- `SimRobotAdapter`：第一里程碑的权威虚拟机器人。
- `RealLebaiAdapter`：后续封装 `lebai_sdk_asyncio`，对外保持相同语义。

### 9.2 RobotControl

RobotControl 是唯一允许调用 `RobotBackend.command_tcp()` 的组件。FastAPI、WebSocket 回调和 UI API 不得直接调用机器人后端。

RobotControl 每 20 ms 执行一次：

1. 读取最新 VRFrame 快照。
2. 检查连接、序号、追踪、页面可见性和样本年龄。
3. 执行遥操作状态机。
4. 在 Grip 上升沿建立手柄/TCP 锚点。
5. 计算相对位姿目标。
6. 应用坐标映射、滤波、死区、步长、速度、加速度和工作空间限制。
7. 向当前 RobotBackend 发送目标或停止命令。
8. 获取 RobotState 并发布给 Quest 和 RecorderSink。

### 9.3 RecorderSink

```python
class RecorderSink(Protocol):
    async def write_vr_frame(self, frame: VRFrame, server_mono_ns: int) -> None: ...
    async def write_robot_state(self, state: RobotState, server_mono_ns: int) -> None: ...
    async def write_event(self, event: TeleopEvent, server_mono_ns: int) -> None: ...
    async def write_camera_frame(self, frame: CameraFrame) -> None: ...
```

第一里程碑使用 `NoopRecorder`。诊断日志与训练数据记录分离；日志不得被当作未来训练数据格式。

## 10. 消息协议

协议版本为 `v=1`，规范文件为 `schemas/teleop-v1.json`。Pydantic 和 TypeScript 使用相同合法/非法 fixture 做契约测试。

### 10.1 Quest 到 PC：vr_frame

```json
{
  "v": 1,
  "type": "vr_frame",
  "session_id": "c2b1...",
  "seq": 1842,
  "client_mono_ms": 35218.4,
  "tracking_valid": true,
  "visibility": "visible",
  "right": {
    "p": [0.12, 1.31, -0.42],
    "q": [0.10, 0.20, 0.30, 0.92],
    "grip": true,
    "trigger": 0.65
  }
}
```

约束：

- `seq` 在一个 session 内严格递增。
- `p` 单位为米。
- `q` 顺序为 `[x,y,z,w]`，服务端归一化前先检查范数处于 `[0.9,1.1]`。
- `trigger` 限制在 `[0,1]`。
- `visibility` 第一里程碑接受 `visible`、`visible-blurred`、`hidden`；只有 `visible` 允许 ACTIVE。
- 非有限数、缺字段、旧协议版本或序号倒退均拒绝更新 latest-value。

### 10.2 PC 到 Quest：robot_state

```json
{
  "v": 1,
  "type": "robot_state",
  "server_mono_ns": 8123456789,
  "ack_seq": 1842,
  "mode": "ACTIVE",
  "robot_state": "MOVING",
  "actual_tcp": {
    "p": [0.31, -0.12, 0.25],
    "q": [0.0, 0.707, 0.0, 0.707]
  },
  "actual_q": [0.0, -0.8, 1.4, -0.6, 1.5, 0.0],
  "gripper": 0.65,
  "sample_age_ms": 18.7,
  "fault": null
}
```

`mode` 为遥操作状态机状态，`robot_state` 为后端机器人状态；两者不得混为一个字段。

### 10.3 控制消息

WebSocket 还支持低频、要求响应的控制消息：

- `hello`：建立协议和能力协商。
- `arm_request`：请求从 READY 进入 ARMED；第一里程碑只对 simulator 有效。
- `disarm`：关闭命令门并停止。
- `ping` / `pong`：估算网络 RTT，不用于跨设备绝对时钟同步。

## 11. 时间基准与频率

PC 的 `time.monotonic_ns()` 是系统主时钟。VRFrame 到达、RobotState 读取、状态机事件和未来相机帧都在 PC 侧加单调时间戳。Quest `performance.now()` 仅用于序列、抖动和 RTT 诊断。

默认频率：

| 环节 | 默认频率 | 行为 |
|---|---:|---|
| Quest WebXR 采样 | 72/90 Hz | 每个 XR 帧读取手柄 |
| Quest 上行 | 60 Hz | 上限；只发送最新状态 |
| RobotControl | 50 Hz | 20 ms 固定周期 |
| SimRobotAdapter | 50 Hz | 专用后台任务，使用固定 20 ms 时间步 |
| RobotState 采集 | 25 Hz | 真机后可根据 SDK 实测调整 |
| 状态下行 | 20 Hz | HUD 和数字孪生插值 |
| 夹爪命令 | 不高于 10 Hz | 变化超过 0.02 才更新 |

调度采用绝对 deadline 递增，避免简单 `sleep(0.02)` 的累计漂移。控制周期晚于 deadline 40 ms 记为一次 overrun；连续两次 overrun 触发停止和 DISARMED。

## 12. 坐标映射与离合

WebXR `local-floor` 坐标为右手系：`+X` 向右、`+Y` 向上、`-Z` 向前。右手柄使用 `gripSpace`，不使用为射线交互优化的 `targetRaySpace`。

默认假定操作者面向机器人正向工作区：

- WebXR 向前 `-Z` → 机器人基坐标 `+X`
- WebXR 向右 `+X` → 机器人基坐标 `-Y`
- WebXR 向上 `+Y` → 机器人基坐标 `+Z`

默认旋转映射：

```text
R_BX =
[ 0  0 -1 ]
[-1  0  0 ]
[ 0  1  0 ]
```

该矩阵行列式为 `+1`，保持右手旋转。现场可选择其他经过验证的正交旋转预设，不允许直接使用包含镜像的矩阵。

Grip 上升沿保存：

- `p_X0, R_XC0`：手柄锚点。
- `p_B0, R_BT0`：机器人实际 TCP 锚点。

平移目标：

```text
p_B_target = p_B0 + R_BX · S · (p_X - p_X0)
```

`S` 第一里程碑默认为各轴 `0.8`。配置范围为 `[0.25, 1.5]`，但第一里程碑 UI 只暴露 `[0.5,1.0]`。

旋转目标：

```text
Delta_R_X = R_XC · transpose(R_XC0)
Delta_R_B = R_BX · Delta_R_X · transpose(R_BX)
R_BT_target = Delta_R_B · R_BT0
```

旋转增益默认为 `1.0`。若以后提供小于 1 的增益，应通过旋转向量或 SLERP 缩放，不得逐分量缩放四元数。

Grip 下降沿关闭命令门、清除锚点并进入 HOLD。下一次 Grip 上升沿读取当时实际 TCP 建立新锚点，因此手柄可重新摆到舒适位置而不引起目标跳变。

## 13. 滤波与运动限制

第一里程碑默认值：

- 平移比例：0.8。
- 旋转比例：1.0。
- 位置死区：1 mm。
- 旋转死区：0.5°。
- 最大 TCP 线速度：0.15 m/s。
- 最大 TCP 角速度：0.6 rad/s。
- 50 Hz 下最大单周期平移：3 mm。
- 50 Hz 下最大单周期旋转：0.012 rad，约 0.69°。
- 最大 TCP 线加速度：0.4 m/s²。
- 最大 TCP 角加速度：1.2 rad/s²。
- 会话相对锚点包络：每轴不超过 0.25 m。

位置和旋转分别使用 One Euro Filter 或具有同等可测试语义的低延迟滤波。滤波参数存入配置并记录在 session metadata 中。任何限幅都作用于目标增量，不能修改实际反馈。

全局工作空间第一里程碑使用仿真模型可达性和配置边界。真机阶段边界必须在不使能状态下重新设置，不能直接沿用虚拟边界。

## 14. 仿真运动学与状态推进

### 14.1 模型

SimRobotAdapter 使用乐白公开的 LM3 理论改进 DH 参数计算完整齐次变换和 TCP 四元数。视觉 GLB 的节点轴和层级只用于动画，不能作为机器人运动学真值。

实际机器存在制造和装配标定偏差；真机精确离线运动学需要厂商提供对应机器的标定参数。第一里程碑明确只使用理论参数。

### 14.2 六维 IK

当前关节向量作为种子。误差向量为：

```text
e = [position_error_xyz, orientation_error_rotvec_xyz]
```

雅可比可先使用中心有限差分实现，后续再替换解析雅可比。阻尼最小二乘更新：

```text
delta_q = J^T · inverse(J·J^T + lambda^2·I) · e
```

每次迭代限制 `delta_q`，每步投影到实际配置的关节范围。终止条件：

- 位置误差不高于 2 mm，且
- 姿态误差不高于 1°，或
- 达到最大 30 次迭代。

不收敛、矩阵非有限、接近配置奇异阈值或目标越界时返回明确故障，不保留部分求解为新目标。

### 14.3 50 Hz 推进

SimRobotAdapter 拥有一个专用 asyncio 后台任务。RobotControl 只更新该适配器的 latest target；仿真任务按绝对 deadline 每 20 ms 推进一步并发布不可变 RobotState 快照。测试中同一个推进器可脱离真实时钟，以显式 `step(dt)` 确定性运行。

对已接受的关节目标 `q*`：

1. 计算比例关节速度目标。
2. 按每关节速度上限裁剪。
3. 按每关节加速度上限限制速度变化。
4. 使用固定 `dt=0.02s` 积分得到新 `q`。
5. 计算 FK 得到实际 TCP。
6. 更新 MOVING、IDLE、HOLD 或 FAULT。

乐白官方资料说明 LM3 六个关节的机械运动范围均为“无限制”，但排除自干涉，并允许用户在设备安全设置中配置应用级角度范围。因此第一里程碑不把 `±π` 描述为硬件限位，而是定义一组纯仿真的安全窗口：每个关节相对 session 参考姿态 `q_ref ± π`。离开该窗口即返回 `joint_safety_window`，避免数值 IK 绕行多圈。仿真关节速度上限默认 0.5 rad/s、加速度上限默认 1.0 rad/s²，明显低于官方 180°/s 最大关节速度。真机阶段必须读取并核对用户在设备中的实际安全角度和速度配置，不能直接沿用仿真窗口。

### 14.4 停止语义

停止时目标立即失效，虚拟关节速度按配置减速度降到零。`stop()` 幂等，多次调用不产生新运动。新命令只有在状态重新进入 ARMED/ACTIVE 后才可接受。

## 15. 遥操作状态机

状态：

- `DISCONNECTED`：无有效 WebSocket 或机器人后端未连接。
- `READY`：连接和追踪有效，但未解锁。
- `ARMED`：已解锁、Grip 未按下。
- `ACTIVE`：Grip 按下且持续接收有效样本。
- `HOLD`：Grip 松开后的受控停止。
- `STALE`：输入超时。
- `FAULT`：机器人、协议、数值或调度故障。
- `DISARMED`：停止完成，要求用户重新解锁。

主要转换：

```text
DISCONNECTED -> READY     连接、追踪和后端正常
READY -> ARMED            用户显式 arm_request
ARMED -> ACTIVE           Grip 上升沿并建立锚点
ACTIVE -> HOLD            Grip 下降沿
ACTIVE -> STALE           样本年龄 >= 100 ms
任意允许运动状态 -> FAULT 后端或数值故障
STALE/FAULT -> DISARMED   停止完成
DISARMED -> READY         故障清除、Grip 已松开
```

从 STALE、FAULT 或重连恢复时，必须观察到至少一帧 `grip=false`，防止用户仍按着 Grip 时意外恢复。

## 16. 失联与故障处理

### 16.1 看门狗

- VRFrame 年龄低于 100 ms：可继续 ACTIVE。
- 年龄达到 100 ms：关闭命令门、调用 `stop()`、进入 STALE。
- 年龄达到 250 ms：进入 DISARMED，要求重新解锁。
- WebSocket close、`hidden`、追踪无效：不等待 100 ms，立即停止。
- 连续两次 40 ms 控制周期 overrun：立即停止。

### 16.2 后端退出

FastAPI lifespan shutdown、SIGINT、SIGTERM 和未捕获控制任务异常均通过同一 `EmergencyStopCoordinator` 调用 backend `stop()`。真机阶段物理急停独立于这些软件路径。

### 16.3 错误类别

- `protocol_error`
- `tracking_lost`
- `input_stale`
- `control_overrun`
- `invalid_numeric`
- `workspace_violation`
- `joint_limit`
- `ik_unreachable`
- `ik_singular`
- `backend_disconnected`
- `backend_fault`
- `real_robot_disabled`

错误通过结构化字段返回，不依赖解析中文字符串。HUD 可将错误码映射为中文提示。

## 17. 夹爪控制

Trigger 连续值 `[0,1]` 映射为夹爪闭合比例。前端和协议统一约定：

- `0.0`：完全打开。
- `1.0`：完全闭合。

SimRobotAdapter 保存规范化值并驱动 GLB 动画。只有变化超过 0.02 或距离上次命令超过 100 ms 才发送更新。

真机适配器负责把规范化闭合比例转换为乐白夹爪 amplitude 语义，并通过真机测试确认 amplitude 的开合方向；控制核心不得假设真机 API 与规范化值方向相同。

Grip 松开、失联和停止不会自动打开或关闭夹爪，夹爪保持最后命令。只有用户命令或明确的恢复流程改变夹爪状态。

## 18. WebXR 前端

### 18.1 启动

页面首先以普通 2D 模式显示：

- 后端连接状态。
- 当前 backend 必须显示为 `SIMULATOR`。
- 进入 VR 按钮。
- 解锁按钮。
- 轴映射和比例的只读摘要。
- 故障与延迟。

第一里程碑若后端报告 `LEBAI`，前端必须拒绝 arm 并显示“第一里程碑禁用真机”。

### 18.2 XR 渲染

- `renderer.xr.enabled = true`。
- 使用 WebXR animation loop，不使用普通 `requestAnimationFrame` 驱动 XR。
- 读取右手 `gripSpace`。
- RobotState 以 20 Hz 到达，渲染端可在相邻状态间插值，但不得外推超过 100 ms。
- 目标 TCP 使用半透明幽灵显示。
- ACTIVE、HOLD、STALE 和 FAULT 使用文字、形状和颜色共同表达，不能只依赖颜色。

### 18.3 桌面调试

桌面模式提供键鼠或开发面板模拟手柄位姿与按钮，用于不佩戴 Quest 运行绝大多数测试。桌面模拟输入与 WebXR 输入必须产出同一个 VRFrame 类型。

## 19. 校准

### 19.1 控制轴校准

第一里程碑在纯 VR 中验证默认 `R_BX`：用户分别向前、向右、向上移动手柄，UI 显示预期虚拟 TCP 轴；确认后才允许 arm。

真机阶段先在机器人不使能状态核对基座正向，再以 10–20% 速度逐轴空载测试。轴映射配置包含矩阵、版本、确认时间和设备说明。

### 19.2 MR 视觉对齐

MR 第一版通过手动放置虚拟基座获得 `T_XR_B_visual`。该变换只进入渲染模块，不传给 RobotControl。后续可加入三点或标记校准，但仍保持显示与控制分离。

### 19.3 真机安全空间

真机安全空间在不使能状态下配置，并与机械臂自身安全设置双重约束。仿真工作空间配置不能直接复制为真机生产值。

## 20. 数据采集扩展

未来 episode 至少包含：

- session/episode ID。
- 后端、软件、协议和配置版本。
- PC 单调时间与可选 UTC 元数据。
- 原始 VR 手柄位姿和按钮。
- 过滤后目标 TCP。
- 实际 TCP、关节位置、速度、力矩（SDK 可提供时）。
- 夹爪命令和反馈。
- 状态机和故障事件。
- 相机帧及其 PC 侧采集时间。
- 任务、成功标记和人工备注。

训练数据必须优先记录实际机器人反馈，而不是只记录 VR 目标。第一里程碑只稳定这些接口和字段语义，不选择最终存储容器；后续根据训练工具决定 LeRobot、HDF5 或 Zarr，并提供无损转换。

Quest 透视画面不是计划中的训练相机。训练视觉源采用固定外部 RGB 或 RGB-D 相机。

## 21. 测试策略

### 21.1 单元测试

- WebXR 到机器人坐标的已知轴向映射。
- 旋转矩阵正交性和行列式。
- 四元数归一化、相对旋转和 Z-Y-X 转换。
- Grip 上升/下降沿和锚点无跳变。
- 死区、单步、速度和加速度限制。
- 状态机每条合法/非法转换。
- 100 ms 和 250 ms 看门狗边界。
- 理论 DH FK 已知姿态。
- 六维 IK 的可达、不可达、奇异和非有限输入。
- 固定时间步积分与停止减速。
- 夹爪方向和节流。

### 21.2 契约测试

- Python 与 TypeScript 对同一合法 fixture 均接受。
- 缺字段、错误版本、NaN、Infinity、四元数范数异常和序号倒退均拒绝。
- RobotState 中 mode 与 robot_state 独立。
- WebSocket hello、arm、disarm、ping/pong 流程。

### 21.3 集成测试

- FastAPI TestClient/WebSocket 驱动 SimRobotAdapter。
- 断开 socket 后 backend 在阈值内进入停止。
- latest-value 不累计旧命令。
- 控制循环异常由 stop coordinator 捕获。
- 10 分钟确定性随机输入无 NaN、内存/队列无持续增长。

### 21.4 前端测试

- GLB 中六个关节和夹爪节点均存在。
- joint state 正确应用到视觉节点。
- simulator 标识、arm 门和故障 HUD。
- 页面 hidden 立即发送 disarm 并停止上传 ACTIVE 输入。
- WebSocket 重连不自动 arm。

## 22. 分阶段验收

### 22.1 阶段 0：桌面仿真

- 所有单元、契约和集成测试通过。
- 随机控制 10 分钟无 NaN、无未处理异常、无命令队列增长。
- 输入超时后 150 ms 内观察到命令门关闭并进入 STALE；在默认仿真速度/减速度上限下，关节速度 600 ms 内降为零。
- Grip 重抓时目标位置跳变量不高于 2 mm，姿态不高于 0.5°。
- 提交 `sim-reachability-v1.json` 固定 fixture：由已知安全窗口内关节姿态经 FK 生成 1000 个可达六维目标。六维 IK 以扰动后的邻近关节作种子时成功率不低于 99%；失败点必须返回明确错误。

### 22.2 阶段 1：Quest 纯 VR

- 三个平移轴和三个旋转轴方向全部正确。
- 连续 20 分钟运行无页面崩溃、后端异常或自动重连续跑。
- 正常局域网下，从 VRFrame 采集到对应 RobotState 返回的 p95 小于 100 ms。
- Grip 松开、追踪丢失、页面隐藏和 WebSocket 断开均通过停止测试。
- 夹爪连续值可稳定控制虚拟动画。

阶段 1 通过即完成第一实施里程碑。

### 22.3 阶段 2：Cannon-es 虚拟抓取

- 方块受重力落在桌面且不穿透。
- 夹爪闭合满足条件时建立抓取约束，松开后解除。
- 标准抓取与放置任务能够重复执行。
- 失败可区分 IK、接触、夹爪、工作空间和操作原因。

### 22.4 阶段 3：MR 真机低速

- 真实 SDK 连接和状态读取独立验收。
- 机械臂 10–20% 速度逐轴空载测试通过。
- Grip、追踪、WebSocket、后端退出和机器人故障停止测试全部通过。
- PC 与 Quest 双重确认门生效。
- MR overlay 不遮挡真实机械臂，且对齐误差不会进入控制目标。

## 23. 风险与缓解

| 风险 | 缓解 |
|---|---|
| WebXR HTTPS/WSS 证书配置困难 | 第一周即验证 Quest 局域网 HTTPS；保留 ADB 端口转发开发路径 |
| Python/Windows 调度抖动 | 绝对 deadline、overrun 检测、latest-value、实测后降低频率 |
| 理论 DH 与 GLB/真机不一致 | GLB 只渲染；运动学单独测试；真机阶段使用厂商标定信息 |
| 数值 IK 在奇异点失败 | 阻尼最小二乘、步长限制、当前姿态种子、失败即停止 |
| MR 对齐误差误导用户 | overlay 半透明且不遮挡；显示变换与控制变换彻底隔离 |
| 仿真产生虚假安全感 | 分阶段验收；真机从不使能、低速、单轴开始；物理急停独立 |
| 浏览器刷新或休眠 | visibility、socket 和样本看门狗；恢复必须重新解锁 |
| 未来数据时间不同步 | PC 单调时钟作为主时钟；每种数据在采集边界盖章 |

## 24. 参考资料

- 乐白 Python SDK 简介：https://help.lebai.ltd/sdk/python/introduce.html
- 乐白运动接口：https://help.lebai.ltd/sdk/python/motion.html
- 乐白状态接口：https://help.lebai.ltd/sdk/python/status.html
- 乐白位置和姿态：https://help.lebai.ltd/guide/pose.html
- 乐白产品信息与关节最大速度：https://help.lebai.ltd/guide/product.html
- 乐白 Python 运动学接口：https://help.lebai.ltd/sdk/python/posture.html
- WebXR Device API：https://immersive-web.github.io/webxr/
- Meta WebXR First Steps：https://github.com/meta-quest/webxr-first-steps
- Meta WebXR Showcases：https://github.com/oculus-samples/webxr-showcases
- 参考虚拟 SDK 计划：`D:\MyWork\ArmPlannerAgent\.worktrees\phase-b-reproducible-system\docs\superpowers\plans\2026-07-11-virtual-lebai-sdk.md`

## 25. 第一实施里程碑完成定义

只有同时满足以下条件，第一里程碑才算完成：

1. 第一里程碑范围内代码和自动化测试完成。
2. 阶段 0 桌面仿真全部验收通过。
3. 阶段 1 Quest 纯 VR 全部验收通过。
4. 默认配置不能连接或解锁真机。
5. 文档明确说明启动、证书、桌面调试、Quest 操作和故障恢复方法。
6. 相机、记录器和 RealLebaiAdapter 接口已稳定，但未声称实现。
7. 未将运动学仿真描述为真实动力学或安全认证。
