# VR4Arm VR 手柄 → 机械臂 TCP 姿态映射详解

> **文档目标：** 以数学和工程并重的方式，完整解释从 Meta Quest 3 手柄位姿到乐白 LM3 机械臂末端姿态的映射全链路。

---

## 目录

1. [概览：五大阶段](#1-概览五大阶段)
2. [参考坐标系](#2-参考坐标系)
3. [第零阶段：硬件 → WebXR 位姿](#3-第零阶段硬件--webxr-位姿)
4. [第一阶段：前端捕获与组帧](#4-第一阶段前端捕获与组帧)
5. [第二阶段：坐标映射](#5-第二阶段坐标映射)
6. [第三阶段：位姿滤波](#6-第三阶段位姿滤波)
7. [第四阶段：安全限制](#7-第四阶段安全限制)
8. [第五阶段：逆运动学求解与虚拟机器人](#8-第五阶段逆运动学求解与虚拟机器人)
9. [第六阶段：状态反馈闭环](#9-第六阶段状态反馈闭环)
10. [数学总结：全链路张量流](#10-数学总结全链路张量流)
11. [附录：参数速查表](#11-附录参数速查表)

---

## 1. 概览：五大阶段

从 VR 手柄的物理运动到屏幕上的机械臂跟随运动，数据经过 **六个阶段、三个进程、跨越网络边界**：

```text
  [Quest Touch Plus 手柄]
         │  WebXR API
         ▼
  ╔═══════════════════════════════════════════════════╗
  ║ 阶段 0 ：WebXR gripSpace → local-floor 位姿     ║
  ╚═══════════════════════════════════════════════════╝
         │  ControllerSample {p, q, grip, trigger}
         ▼
  ╔═══════════════════════════════════════════════════╗
  ║ 阶段 1 ：前端捕获与组帧 (TypeScript / Quest)     ║
  ║          ControllerSample → VRFrame (JSON)        ║
  ╚═══════════════════════════════════════════════════╝
         │  WebSocket (WSS)  ~60 Hz
         ▼
  ╔═══════════════════════════════════════════════════╗
  ║ 阶段 2 ：坐标映射 (Python / FastAPI)              ║
  ║          VR 手柄位姿 → 机器人 TCP 目标位姿        ║
  ║          锚点捕获 + 相对增量 + R_BX 旋转变换      ║
  ╚═══════════════════════════════════════════════════╝
         │  raw_requested_pose (in robot frame)
         ▼
  ╔═══════════════════════════════════════════════════╗
  ║ 阶段 3 ：位姿滤波 (8 Hz 一阶低通)                 ║
  ║          1EMA 平滑位置 + SLERP 平滑姿态            ║
  ╚═══════════════════════════════════════════════════╝
         │  filtered_pose
         ▼
  ╔═══════════════════════════════════════════════════╗
  ║ 阶段 4 ：安全限制                                  ║
  ║          速度限制 + 加速度限制 + 工作空间包络       ║
  ╚═══════════════════════════════════════════════════╝
         │  safe_target_pose
         ▼
  ╔═══════════════════════════════════════════════════╗
  ║ 阶段 5 ：逆运动学 + 虚拟机器人                     ║
  ║          IK → joint angles → VirtualRobot → FK    ║
  ╚═══════════════════════════════════════════════════╝
         │  RobotStateMessage @ 20 Hz
         ▼
  ╔═══════════════════════════════════════════════════╗
  ║ 阶段 6 ：状态反馈与渲染                            ║
  ║          RobotStateBuffer 插值 → Three.js 动画    ║
  ╚═══════════════════════════════════════════════════╝
```

**关键频率：**

| 频率 | 环节 | 位置 |
|------|------|------|
| ~60 Hz | VR 手柄位姿采样 | Quest 前端 (WebXR frame callback) |
| ~60 Hz | 桌面模拟位姿采样 | 桌面前端 (requestAnimationFrame) |
| 50 Hz | 控制循环 tick | 后端 RobotControl.run() |
| 20 Hz | 机器人状态广播 | 后端 state_message() → WebSocket |
| ~60 fps | 3D 渲染刷新 | 前端 SimulationScene |

---

## 2. 参考坐标系

本系统涉及 **四个三维参考坐标系**：

### 2.1 WebXR local-floor 参考空间

```
        Y (up)
        │
        │     Z (toward user)
        │    ╱
        │   ╱
        │  ╱
        │ ╱
        └────────── X (right)
      原点：用户站立位置的地面
```

- 由 Quest 的 inside-out 追踪系统维护
- Y 轴指向上方（重力方向）
- 右手坐标系

### 2.2 手柄 gripSpace

```
      Quest Touch Plus 右手柄
         ┌─────┐
         │     │  →  -Z (指向前方)
         │  🅰 🅱│
         │     │  ↓  -Y
         └─────┘
         握持位置 (grip button 位置)
```

- gripSpace 的原点位于手柄握持按钮附近
- 方向由手柄内部 IMU 和光学追踪确定
- WebXR 自动计算 gripSpace → local-floor 的变换矩阵

### 2.3 机器人基坐标系

```
    乐白 LM3 基座
          Z (up)
          │
          │   Y
          │  ╱
          │ ╱
          └────────── X
       原点：基座安装面中心
```

- 标准工业机器人坐标系
- TCP (Tool Center Point) 的法兰端面中心偏移 (0, 0, 0.09) m

### 2.4 VR 手柄坐标系 → 机器人坐标系映射关系

VR4Arm 使用 `R_BX` 矩阵建立两套坐标系的对应关系：

```text
 VR 坐标轴        →        机器人坐标轴
 ─────────────             ─────────────
    -Z          →           +X
    -X          →           -Y
    +Y          →           +Z
```

数值形式：

```
          ┌              ┐
          │  0   0  -1   │
 R_BX  =  │ -1   0   0   │
          │  0   1   0   │
          └              ┘
```

**物理直观理解：**

- VR 中手柄**向前推**（−Z 方向）→ 机器人 TCP **向前伸**（+X 方向）
- VR 中手柄**向左移**（−X 方向）→ 机器人 TCP **向左移**（−Y 方向）
- VR 中手柄**向上抬**（+Y 方向）→ 机器人 TCP **向上升**（+Z 方向）

`R_BX` 是一个 **proper rotation matrix**（行列式为 +1，满足 RᵀR = I），代码强制验证这一点。

---

## 3. 第零阶段：硬件 → WebXR 位姿

### 3.1 手柄读取流程

**代码位置：** `web/src/xr/controllerInput.ts:21-47` — `readRightController()`

```typescript
function readRightController(frame, referenceSpace, inputSources): ControllerSample {
  // 1. 找到右手手柄
  const source = Array.from(inputSources).find(s => s.handedness === 'right');

  // 2. 获取 gripSpace → local-floor 的位姿
  const pose = frame.getPose(source.gripSpace, referenceSpace);

  // 3. 读取按键值
  return {
    p: [position.x, position.y, position.z],   // 3D 位置 (m)
    q: [orientation.x, orientation.y,            // 单位四元数
        orientation.z, orientation.w],
    grip:     buttons[1].pressed,                // Grip 按键
    trigger:  clamp(buttons[0].value, 0, 1),     // Trigger 模拟量
    armButton:  questButtons && buttons[4],      // A 键
    stopButton: questButtons && buttons[5],      // B 键
    trackingValid: true,
  };
}
```

### 3.2 WebXR Pose 的数学含义

`frame.getPose(gripSpace, referenceSpace)` 返回的 `XRPose` 包含：

```text
 position   = (x, y, z)      ← gripSpace 原点在 local-floor 中的 3D 位置
 orientation = (qx, qy, qz, qw) ← gripSpace 相对于 local-floor 的姿态四元数
```

四元数约定（Three.js / WebXR）：

- 标量在最后：`(x, y, z, w)`
- 单位四元数：‖q‖ = 1
- 旋转方向：将向量从 gripSpace 旋转到 local-floor 空间

### 3.3 桌面模式的等效输入

**代码位置：** `web/src/scenes/simulationScene.ts:349-369` — `sendDesktopFrame()`

桌面模式不使用 WebXR，而是通过鼠标/键盘模拟手柄位姿：

| 操作 | 影响 |
|------|------|
| 鼠标 X 拖动 | controllerPosition.x 变化 (±0.0012 每像素) |
| 鼠标 Y 拖动 | controllerPosition.y 变化 (±0.0012 每像素) |
| 滚轮 | controllerPosition.z 变化 (±0.0008 每像素) |
| 空格键按下 | trigger = 1.0 |
| 左键按下 | grip = true |

桌面位姿不模拟旋转（quaternion 始终为单位四元数），因此仅支持平移遥操作。

---

## 4. 第一阶段：前端捕获与组帧

### 4.1 VRFrame 消息结构

**代码位置：** `web/src/scenes/simulationScene.ts:29-45` — `createVRFrame()`

```typescript
function createVRFrame(input): VRFrame {
  return {
    v: 1,                           // 协议版本
    type: 'vr_frame',
    session_id: 'quest-<uuid>',     // 会话 ID（进入 VR 时生成）
    seq: 0, 1, 2, ... ,             // 会话内递增序号
    client_mono_ms: performance.now(),  // 客户端单调时钟 (ms)
    tracking_valid: true,            // 追踪是否有效
    visibility: 'visible',           // VR 会话可见性
    right: {
      p: [x, y, z],                  // 手柄位置 (m)
      q: [qx, qy, qz, qw],          // 手柄姿态四元数
      grip: true/false,              // Grip 按键
      trigger: 0.0 ~ 1.0,            // Trigger 值
    },
  };
}
```

### 4.2 发送时序

VR 模式下由 `XRSessionController` 管理发送，桌面模式下由 `SimulationScene.animate` 管理：

```text
VR 模式:
  WebXR frame callback → readRightController() → onFrame(VRFrame)
  节流：两次发送间隔 ≥ 16.67ms (60Hz 上限)

桌面模式:
  requestAnimationFrame → sendDesktopFrame()
  节流：两次发送间隔 ≥ 16.67ms (60Hz 上限)
```

### 4.3 会话管理

- 进入 VR → 生成新 `quest-<uuid>` session ID
- 退出 VR → 生成新 `desktop-<uuid>` session ID
- 后端通过 session ID 区分新旧会话，拒绝旧 session 的帧

---

## 5. 第二阶段：坐标映射

**代码位置：** `backend/app/control/coordinate_mapper.py`

这是整个映射链路的**核心算法**。采用**锚点 + 相对增量**策略，保证 Grip 按下瞬间无跳变。

### 5.1 锚点捕获

当 Grip 按下、状态机从 ARMED/HOLD 转移到 ACTIVE 时，捕获两个锚点：

```python
def capture(self, hand: Pose, tcp: Pose) -> None:
    self._hand_anchor = hand.model_copy(deep=True)   # 当前手柄位姿
    self._tcp_anchor  = tcp.model_copy(deep=True)    # 当前机器人 TCP 位姿
```

触发时机（`robot_control.py:214-222`）：

```python
if previous_mode in {ARMED, HOLD} and self.machine.mode == ACTIVE:
    state = await self.backend.get_state()
    self.mapper.capture(
        Pose(p=received.frame.right.p, q=received.frame.right.q),  # hand
        state.actual_tcp,                                            # tcp
    )
```

**关键设计：** 锚点捕获的是当前**实际** TCP 位姿（从虚拟机器人取回），而非目标值。这保证了即使仿真未完全收敛，映射也能从实际状态出发。

### 5.2 目标位姿计算

每次控制循环 tick（50 Hz），处于 ACTIVE 状态且有新帧时：

```python
def target(self, hand: Pose) -> Pose:
    # ── 第 1 步：位置增量映射 ──
    # 计算手柄在 VR 空间中的位移
    delta_p_hand = hand.p - hand_anchor.p          # VR 空间位移向量

    # 旋转变换到机器人空间，并缩放
    p_target = tcp_anchor.p + R_BX @ (0.8 × delta_p_hand)
    #                              ^^^^  ^^^^^^^^^^^^^^^
    #                              旋转矩阵  平移缩放 0.8×

    # ── 第 2 步：姿态增量映射 ──
    # 计算手柄在 VR 空间中的相对旋转
    R_now    = Rotation.from_quat(hand.q)           # 当前手柄姿态矩阵
    R_anchor = Rotation.from_quat(hand_anchor.q)    # 锚点手柄姿态矩阵
    delta_R_hand = R_now @ R_anchor.T               # VR 空间的相对旋转

    # 将相对旋转变换到机器人坐标系
    delta_R_robot = R_BX @ delta_R_hand @ R_BX.T    # 相似变换

    # 旋转缩放 (通过 SLERP 实现)
    if rotation_scale != 1.0:
        delta_R_robot = SLERP(I, delta_R_robot, rotation_scale)

    # 施加到机器人 TCP 锚点姿态
    R_target = delta_R_robot @ R_tcp_anchor

    return Pose(p=p_target, q=R_target.as_quat())
```

### 5.3 平移映射详解

```
 p_target = p_anchor_tcp + R_BX · (scale_trans · (p_hand - p_anchor_hand))
           \____________/   \___/   \______/  \_______________________/
                │              │        │               │
                │              │        │        VR 空间中的手柄位移量
                │              │        │
                │              │   平移缩放因子 (0.8)
                │              │
                │       坐标轴旋转矩阵 R_BX
                │
          最终机器人 TCP 目标位置
```

**缩放因子 0.8 的意义：** 操作者的手在 VR 中移动 10 cm → 机器人 TCP 移动 8 cm。缩小动作幅度提升精细操作能力，同时减少因手抖导致的末端抖动。

**R_BX 作用：** 将 VR 空间的 `(x, y, z)` 位移旋转到机器人空间的 `(x', y', z')` 位移。

### 5.4 姿态映射详解

姿态映射比平移映射更复杂，因为旋转矩阵在不同坐标系下需要进行**相似变换**（similarity transform）：

```text
步骤 1: 计算 VR 空间中的相对旋转
─────────────────────────────────────
  手柄当前姿态: R_now       (gripSpace → local-floor)
  手柄锚点姿态: R_anchor    (gripSpace → local-floor)
  相对旋转:     ΔR_hand = R_now · R_anchorᵀ

  ΔR_hand 的含义：
    "从锚点时的姿态到当前姿态，手柄在 VR 空间旋转了多少"
    这是一个在 local-floor 坐标系中表达的旋转矩阵。

步骤 2: 将相对旋转变换到机器人基坐标
─────────────────────────────────────
  ΔR_robot = R_BX · ΔR_hand · R_BXᵀ

  这是一个相似变换（也称为共轭）：
    R_BX 将 VR 坐标轴方向映射到机器人坐标轴方向
    R_BXᵀ 将旋转结果映射回机器人坐标系的分解

  直观理解：
    ΔR_hand 绕着 VR 空间的某个轴旋转了 θ 度
    → ΔR_robot 应该在机器人空间中绕着对应的轴旋转同样的 θ 度
    → 例如：绕 VR 空间 -Z 轴旋转 = 绕机器人空间 +X 轴旋转

步骤 3: 应用旋转缩放
─────────────────────
  rotation_scale 默认为 1.0（不缩放）
  如果 scale ≠ 1.0，通过 SLERP 对旋转角度进行缩放：
    ΔR_robot_scaled = SLERP(I, ΔR_robot, scale)

  SLERP(I, R, s) = exp(s · log(R))
  即将旋转角度乘以缩放因子

步骤 4: 合成最终目标姿态
─────────────────────────
  R_target = ΔR_robot_scaled · R_tcp_anchor

  R_tcp_anchor: 锚点捕获时的机器人 TCP 实际姿态
  R_target:     期望的机器人 TCP 姿态
```

### 5.5 为什么用 R_BX @ ΔR @ R_BXᵀ 而不是直接用 R_BX @ R_hand？

这是旋转矩阵在不同坐标系间转换时的经典操作（相似变换 / 共轭）：

- `R_BX @ R_hand`：将手柄的**绝对姿态**从 VR 空间映射到机器人空间 —— 但我们不需要绝对姿态，我们需要**相对旋转**。
- `R_BX @ ΔR_hand @ R_BXᵀ`：将 VR 空间中表达的**相对旋转**变换到机器人空间中表达的相对旋转。这保证旋转轴和旋转角在两个坐标系中具有一致的物理意义。

数学上：

- ΔR_hand 的旋转轴 `ω_vr` 在 VR 空间中
- ΔR_robot 的旋转轴 `ω_robot = R_BX · ω_vr` 在机器人空间中
- 两者的旋转角相同（不考虑 scaling）

### 5.6 锚点清除时机

| 触发条件 | 状态变化 | 锚点处理 |
|---------|---------|---------|
| Grip 按下 → ACTIVE | ARMED/HOLD → ACTIVE | **设置**锚点 |
| Grip 松开 | ACTIVE → HOLD | **清除**锚点 |
| 按 B 停止 | ACTIVE → DISARMED | **清除**锚点 |
| 断连 | → DISCONNECTED | **清除**锚点 |
| Session 变化 | — | **清除**锚点 |

---

## 6. 第三阶段：位姿滤波

**代码位置：** `backend/app/control/filters.py`

映射后的目标位姿进入一阶低通滤波器，抑制手柄抖动和高频噪声。

### 6.1 滤波器设计

```python
class PoseFilter:
    cutoff_hz: float = 8.0    # 截止频率 8 Hz

    def update(self, pose: Pose, dt: float) -> Pose:
        # α = 1 - exp(-2π · f_c · Δt)
        alpha = 1 - exp(-2 * pi * 8.0 * dt)
        # dt = 0.02s → α ≈ 1 - exp(-2π · 8 · 0.02) ≈ 0.634
```

### 6.2 位置滤波

一阶指数移动平均（1 Exponential Moving Average）：

```
 p_filtered(n) = p_filtered(n-1) + α · (p_raw(n) - p_filtered(n-1))
               = (1-α) · p_filtered(n-1) + α · p_raw(n)
```

**频域特性（8 Hz 截止, 50 Hz 采样）：**

| 频率 | 衰减 |
|------|------|
| 1 Hz | ~0% (几乎直通) |
| 8 Hz | −3 dB (截止点) |
| 20 Hz | −12 dB |
| 50 Hz | −22 dB |

这意味着手柄的生理性手抖（8–12 Hz 频段）被显著衰减，而正常的慢速操作动作（0.5–3 Hz）几乎不受影响。

### 6.3 姿态滤波

对四元数不能直接做 EMA，需使用 **SLERP (Spherical Linear Interpolation)**：

```
 q_filtered = SLERP(q_filtered(n-1), q_raw(n), α)
```

SLERP 保证插值结果始终是单位四元数，且以恒定角速度沿大圆弧插值。

```python
rotations = Rotation.concatenate([Rotation.from_quat(prev.q),
                                   Rotation.from_quat(pose.q)])
orientation = Slerp([0.0, 1.0], rotations)([alpha])[0]
```

**注意：** `scipy.spatial.transform.Slerp` 返回插值后的 `Rotation` 对象，自动保证四元数归一化。

### 6.4 滤波器重置

在以下时机重置滤波器为当前 TCP 实际位姿：

```python
# 进入 ACTIVE 时
self.filter.reset(state.actual_tcp)
```

这防止滤波器从旧状态平滑过渡到新模式时产生"飞越"伪影。

---

## 7. 第四阶段：安全限制

**代码位置：** `backend/app/control/safety.py`

滤波后的目标位姿进入二级安全校验器，在发送给逆运动学前施加硬限幅。

### 7.1 工作空间包络检查

```python
if abs(target_pos - anchor_pos) > envelope (0.25 m):
    raise SafetyViolation("workspace_violation")
    → 触发 FAULT → DISARMED
```

- **锚点** (anchor)：进入 ACTIVE 时的实际 TCP 位置
- **包络** (envelope)：以锚点为中心的 ±0.25 m 正方体
- 任何超界目标会触发安全违规，立即停止机械臂

### 7.2 线速度限制

```python
desired_velocity = (target_pos - start_pos) / dt    # 期望速度向量
speed = |desired_velocity|
if speed > max_linear_speed (0.15 m/s):
    desired_velocity *= 0.15 / speed                 # 等比例缩放
```

- 限速 0.15 m/s ≈ 每 tick 最大位移 3 mm（@ 50 Hz）
- 等比例缩放保留运动方向，只限制速率

### 7.3 线加速度限制

```python
velocity_delta = desired_velocity - current_velocity
delta_norm = |velocity_delta|
if delta_norm > max_linear_accel * dt (0.4 * 0.02 = 0.008 m/s):
    velocity_delta *= 0.008 / delta_norm             # 裁剪加速度
current_velocity += velocity_delta
position = start_pos + current_velocity * dt
```

- 通过限制速度变化量间接实现加速度限制
- 每个 tick 速度变化上限：0.4 × 0.02 = 0.008 m/s

### 7.4 角速度限制

```python
relative_rotation = requested_rotation * start_rotation.inv()
desired_angular_velocity = rotvec(relative_rotation) / dt
angular_speed = |desired_angular_velocity|
if angular_speed > max_angular_speed (0.6 rad/s):
    desired_angular_velocity *= 0.6 / angular_speed
```

- 限速 0.6 rad/s ≈ 34°/s ≈ 每 tick 0.012 rad (@ 50 Hz)
- `rotvec()` 提取旋转向量（方向 = 旋转轴，模长 = 旋转角）

### 7.5 角加速度限制

```python
angular_velocity_delta = desired_angular_vel - current_angular_vel
delta_norm = |angular_velocity_delta|
if delta_norm > max_angular_accel * dt (1.2 * 0.02 = 0.024 rad/s):
    angular_velocity_delta *= 0.024 / delta_norm
current_angular_velocity += angular_velocity_delta
limited_rotation = from_rotvec(current_angular_velocity * dt) * start_rotation
```

### 7.6 安全限制参数汇总

| 参数 | 值 | 物理含义 |
|------|-----|---------|
| max_linear_speed | 0.15 m/s | TCP 末端最大移动速度 |
| max_angular_speed | 0.6 rad/s | TCP 末端最大旋转速度 (~34°/s) |
| max_linear_accel | 0.4 m/s² | TCP 末端最大加速度 |
| max_angular_accel | 1.2 rad/s² | TCP 末端最大角加速度 |
| envelope | 0.25 m | 从锚点的最大允许位移半径 |
| control_period | 0.02 s | 控制 tick 周期 |

---

## 8. 第五阶段：逆运动学求解与虚拟机器人

### 8.1 逆运动学 (IK)

**代码位置：** `backend/app/sim/ik.py`

通过安全限制后的 TCP 位姿目标，输入 IK 求解器得到关节角目标。

```text
输入:  TCP 目标位姿 (p_target, q_target) + 当前关节角 seed_q
输出:  目标关节角 q_target

算法: 阻尼最小二乘法 (Damped Least Squares)
```

**迭代过程：**

```python
for i in range(max_iterations=30):
    # 1. 正运动学计算当前 TCP
    current_pose = forward_pose(q_current)

    # 2. 计算位姿误差
    pos_error = p_target - p_current
    rot_error = rotvec(R_target @ R_current.T)

    # 3. 收敛判断
    if |pos_error| < 2mm and |rot_error| < 1°:
        return IKResult(q_current, converged=True)

    # 4. 数值 Jacobian (中心差分, ε=1e-5)
    J = numerical_jacobian(q_current, ε=1e-5)   # 6×6 矩阵

    # 5. 阻尼最小二乘步
    Δq = Jᵀ @ inv(J @ Jᵀ + λ²I) @ error          # λ=0.04

    # 6. 步长裁剪
    Δq = clip(Δq, ±0.12 rad)

    # 7. 更新关节角 + 安全检查
    q_new = q_current + Δq
    q_new = clip(q_new, home_q - π, home_q + π)  # 关节窗口

    q_current = q_new
```

**关键参数：**

| 参数 | 值 | 说明 |
|------|-----|------|
| 最大迭代次数 | 30 | 超时即报 IKError |
| 收敛位置阈值 | 2 mm | 位置误差判断 |
| 收敛姿态阈值 | 1° | 姿态误差判断 |
| 阻尼因子 λ | 0.04 | 防止奇异点附近发散 |
| 每步裁剪 | ±0.12 rad | 防止关节角突变 |
| Jacobian 步长 ε | 1×10⁻⁵ | 中心差分步长 |
| 关节安全窗口 | ±π rad | 从 home_q 算起 |

**可达性：** 在 1000 点测试夹具上 ≥ 99% 成功率。

### 8.2 虚拟机器人仿真

**代码位置：** `backend/app/sim/virtual_robot.py`

```python
class VirtualRobot:
    max_joint_speed: 0.5 rad/s    # 单关节最大速度
    max_joint_accel: 1.0 rad/s²   # 单关节最大加速度

    def step(self, dt) -> joint_state:
        # 1. P-控制器跟踪目标
        error = target_q - current_q
        desired_velocity = Kp * error           # Kp = 2.0

        # 2. 速度限幅
        desired_velocity = clip(desired_velocity, ±max_joint_speed)

        # 3. 加速度限幅
        velocity_delta = desired_velocity - current_velocity
        velocity_delta = clip(velocity_delta, ±max_joint_accel * dt)
        current_velocity += velocity_delta

        # 4. 积分位置
        current_q += current_velocity * dt

        # 5. 快照锁定 (误差 < 1e-4 rad 且可减速停止)
        if |error| < 1e-4 and can_stop(current_velocity, dt):
            current_q = target_q
            current_velocity = 0
```

### 8.3 正运动学 (FK) 反馈

**代码位置：** `backend/app/sim/kinematics.py`

```python
def forward_pose(q) -> Pose:
    # Modified DH 6-DOF 正运动学链
    T = I_4x4
    for i in range(6):
        T = T @ mdh_transform(q[i], d[i], a[i], alpha[i])
    T = T @ tcp_offset    # TCP 偏移 (0, 0, 0.09) m
    return Pose(p=T[:3,3], q=quaternion_from_matrix(T))
```

虚拟机器人的**实际**关节角通过 FK 算出实际 TCP 位姿，这个位姿通过 WebSocket 发回前端用于渲染。

---

## 9. 第六阶段：状态反馈闭环

### 9.1 状态广播

每 50 ms（20 Hz），后端从虚拟机器人读取当前状态、构造 `RobotStateMessage` 并通过 WebSocket 发送：

```python
async def state_message(self):
    state = await self.backend.get_state()           # 虚拟机器人当前状态
    return RobotStateMessage(
        mode=self.machine.mode,                      # TeleopMode 枚举
        robot_state=state.robot_state,               # BackendState 枚举
        actual_tcp=state.actual_tcp,                 # 实际 TCP 位姿 (FK 计算)
        actual_q=state.actual_q,                     # 6 个关节角
        gripper=state.gripper,                       # 夹爪开度 0~1
        sample_age_ms=...,                           # 最近帧的时效
        ack_seq=...,                                 # 最新处理的 VR 帧序号
        server_mono_ns=...,                          # 服务端单调时间戳
        fault=...,                                   # 故障码
    )
```

### 9.2 前端状态插值

**代码位置：** `web/src/robot/robotState.ts`

前端只保留最新的 **2 帧**状态，渲染时线性插值：

```typescript
class RobotStateBuffer {
    sample(nowNs: number): InterpolatedState | null {
        const [older, newer] = this.store;   // 最多 2 帧

        // 时效检查：最新帧距今 > 100ms → stale
        if (nowNs - newer.server_mono_ns > 100_000_000) return null;

        // 线性插值
        const fraction = (nowNs - older.server_mono_ns) /
                         (newer.server_mono_ns - older.server_mono_ns);
        return {
            actual_q: older.q + fraction * (newer.q - older.q),
            gripper:  older.gripper + fraction * (newer.gripper - older.gripper),
            ...
        };
    }
}
```

### 9.3 渲染

```typescript
// 每帧执行:
const interpolated = stateBuffer.sample(estimatedServerNow);
robotModel.setJointAngles(interpolated.actual_q);    // 更新 6 个关节
robotModel.setGripper(interpolated.gripper);          // 更新夹爪
renderer.render(scene, camera);                       // WebGL 渲染
```

---

## 10. 数学总结：全链路张量流

以下用统一数学符号描述单个控制周期（20 ms）内的完整变换：

### 10.1 符号定义

| 符号 | 含义 | 空间 |
|------|------|------|
| `p_h(n)` | 第 n 帧手柄位置 | VR (local-floor) |
| `q_h(n)` | 第 n 帧手柄姿态四元数 | VR (local-floor) |
| `R_h(n)` | q_h(n) 对应的旋转矩阵 | SO(3) |
| `p_a` | 锚点捕获时的手柄位置 | VR |
| `R_a` | 锚点捕获时的手柄姿态 | SO(3) |
| `p_T` | 锚点捕获时的 TCP 位置 | 机器人基坐标 |
| `R_T` | 锚点捕获时的 TCP 姿态 | SO(3) |
| `s_t` | 平移缩放因子 (0.8) | — |
| `s_r` | 旋转缩放因子 (1.0) | — |

### 10.2 单帧变换方程

**阶段 2 — 坐标映射：**

```
 p_raw = p_T + s_t · R_BX · (p_h(n) - p_a)              (1) 位置

 ΔR = R_h(n) · R_aᵀ                                    (2a) VR 空间相对旋转
 ΔRᴮ = R_BX · ΔR · R_BXᵀ                               (2b) 旋转相似变换到机器人空间
 ΔRᴮ_scaled = exp(s_r · log(ΔRᴮ))                      (2c) 旋转角度缩放
 R_raw = ΔRᴮ_scaled · R_T                               (2d) 最终目标姿态
```

**阶段 3 — 位姿滤波：**

```
 α = 1 − exp(−2π · f_c · Δt)                            (3a) 滤波系数

 p_filt = (1−α) · p_filt_prev + α · p_raw               (3b) 位置 EMA
 q_filt = SLERP(q_filt_prev, q_raw, α)                   (3c) 姿态 SLERP
```

**阶段 4 — 安全限制：**

```
 v_des = (p_filt − p_lim_prev) / Δt                      (4a) 期望线速度
 v_des = clip_speed(v_des, 0.15)                         (4b) 速度限幅
 v_lim = v_lim_prev + clip_accel(v_des − v_lim_prev,     (4c) 加速度限幅
                                  0.4 · Δt)
 p_lim = p_lim_prev + v_lim · Δt                         (4d) 安全位置

 ω_des = rotvec(R_filt · R_lim_prevᵀ) / Δt              (4e) 期望角速度
 ω_des = clip_speed(ω_des, 0.6)                          (4f) 角速度限幅
 ω_lim = ω_lim_prev + clip_accel(ω_des − ω_lim_prev,     (4g) 角加速度限幅
                                  1.2 · Δt)
 R_lim = exp(ω_lim · Δt) · R_lim_prev                    (4h) 安全姿态
```

**阶段 5 — 逆运动学：**

```
 q_target = damped_least_squares(                         (5) IK 求解
     p_lim, R_lim, q_current, λ=0.04, max_iter=30
 )
```

### 10.3 端到端延迟

从手柄运动到屏幕可见机器人运动的总延迟：

```text
手柄运动 → WebXR frame     : ~0 ms (硬件同步)
→ VRFrame 组帧发送          : 0-16 ms (等待帧回调)
→ WebSocket 传输            : 1-5 ms (同机 localhost)
→ 控制 tick 处理            : 0-20 ms (等待 tick 到达)
→ 坐标映射 + 滤波 + 限制    : <0.5 ms (纯数值计算)
→ IK 求解                   : <1 ms (收敛 < 30 次迭代)
→ 虚拟机器人步进            : <0.1 ms
→ 状态广播等待              : 0-50 ms (等待 20 Hz 发送)
→ WebSocket 回传            : 1-5 ms
→ 前端接收 + 插值 + 渲染    : <16 ms (等待下一帧回调)
────────────────────────────────────────
  端到端 (典型)             : ~50-100 ms
  端到端 (最坏)             : ~150 ms
```

---

## 11. 附录：参数速查表

### 11.1 坐标映射参数

| 参数 | 值 | 可配置 | 说明 |
|------|-----|--------|------|
| R_BX | `[[0,0,-1],[-1,0,0],[0,1,0]]` | 代码中硬编码 | VR → 机器人坐标轴映射 |
| translation_scale | 0.8 | ✅ config.yaml | 位移缩放比例 |
| rotation_scale | 1.0 | ✅ config.yaml | 旋转缩放比例 |

### 11.2 PoseFilter 参数

| 参数 | 值 | 可配置 | 说明 |
|------|-----|--------|------|
| cutoff_hz | 8.0 | 代码中硬编码 | 低通截止频率 |
| 滤波器类型 | 1EMA (位置) + SLERP (姿态) | — | 一阶低通 |

### 11.3 SafetyLimiter 参数

| 参数 | 值 | 可配置 | 说明 |
|------|-----|--------|------|
| max_linear_speed | 0.15 m/s | ✅ config.yaml | 线速度限制 |
| max_angular_speed | 0.6 rad/s | ✅ config.yaml | 角速度限制 |
| max_linear_accel | 0.4 m/s² | ✅ config.yaml | 线加速度限制 |
| max_angular_accel | 1.2 rad/s² | ✅ config.yaml | 角加速度限制 |
| envelope | 0.25 m | 代码中硬编码 | 工作空间包络半径 |

### 11.4 IK 求解器参数

| 参数 | 值 | 可配置 | 说明 |
|------|-----|--------|------|
| 求解算法 | 阻尼最小二乘法 | — | Damped Least Squares |
| λ (阻尼) | 0.04 | 代码中硬编码 | 防止奇异 |
| max_iter | 30 | 代码中硬编码 | 最大迭代次数 |
| pos_tol | 2 mm | 代码中硬编码 | 位置收敛阈值 |
| rot_tol | 1° | 代码中硬编码 | 姿态收敛阈值 |
| step_clip | 0.12 rad | 代码中硬编码 | 每步关节增量上限 |
| ε (Jacobian) | 1×10⁻⁵ | 代码中硬编码 | 数值差分步长 |

### 11.5 虚拟机器人参数

| 参数 | 值 | 可配置 | 说明 |
|------|-----|--------|------|
| Kp | 2.0 | 代码中硬编码 | P 控制器增益 |
| max_joint_speed | 0.5 rad/s | ✅ config.yaml | 单关节最大角速度 |
| max_joint_accel | 1.0 rad/s² | ✅ config.yaml | 单关节最大角加速度 |
| snap_threshold | 1×10⁻⁴ rad | 代码中硬编码 | 快照锁定阈值 |
| joint_window | π rad | ✅ config.yaml | 关节安全窗口（从 home_q） |

### 11.6 时序参数

| 参数 | 值 | 可配置 | 说明 |
|------|-----|--------|------|
| control_hz | 50 Hz | ✅ config.yaml | 后端控制循环频率 |
| state_hz | 20 Hz | ✅ config.yaml | 状态反馈广播频率 |
| stale_ms | 100 ms | ✅ config.yaml | 帧软超时阈值 |
| disarm_ms | 250 ms | ✅ config.yaml | 帧硬超时阈值 |
| gripper_period | 100 ms | 代码中硬编码 | 夹爪最小更新间隔 |
| gripper_min_delta | 0.02 | 代码中硬编码 | 夹爪最小变化阈值 |

---

> **总结：** VR4Arm 的 VR→机器人姿态映射是一个**六阶段、多层次**的信号处理流水线。从 WebXR 底层 API 读取的单位四元数+位置向量，经过坐标变换、低通滤波、多级安全限制、逆运动学求解和虚拟机器人仿真，最终转化为 LM3 机械臂的关节角运动。整个过程在 50 Hz 的控制频率下保证了**安全、平滑、低延迟**的遥操作体验。
