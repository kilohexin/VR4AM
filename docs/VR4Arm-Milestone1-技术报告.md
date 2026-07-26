# VR4Arm 项目技术报告 —— Milestone 1

> **报告日期：** 2026-07-16
> **项目阶段：** Milestone 1 完成，Quest 3 真机验收待执行
> **报告人：** [待填写]

---

## 目录

1. [项目概述](#1-项目概述)
2. [系统架构](#2-系统架构)
3. [技术栈](#3-技术栈)
4. [核心模块详解](#4-核心模块详解)
5. [数据流与通信协议](#5-数据流与通信协议)
6. [安全设计](#6-安全设计)
7. [三 maintMeasurements仿真与运动学](#7-三维仿真与运动学)
8. [VR/WebXR 遥操作](#8-vrwebxr-遥操作)
9. [测试与自动化验证](#9-测试与自动化验证)
10. [Milestone 1 完成情况](#10-milestone-1-完成情况)
11. [后续工作规划](#11-后续工作规划)

---

## 1. 项目概述

### 1.1 项目目标

VR4Arm 是一个**基于 Web 的 VR 机械臂遥操作仿真平台**，目标是以 Meta Quest 3 头显实现对乐白（Lebai）LM3 六轴协作机械臂的沉浸式远程操控。项目采用渐进式交付策略，Milestone 1 聚焦于建立**安全、可验证的纯仿真基础平台**，不接入任何真实硬件。

### 1.2 项目进展

| 项目 | 状态 |
|------|------|
| Quest 3 / 桌面浏览器遥操作链路 | ✅ 完成 |
| Three.js LM3 三维仿真与运动学 | ✅ 完成 |
| 50 Hz 控制循环 + 20 Hz 状态反馈 | ✅ 完成 |
| FastAPI + WebSocket 后端 | ✅ 完成 |
| 安全状态机 + 坐标映射 + 滤波 + 限速 | ✅ 完成 |
| 头显内中文安全状态牌 | ✅ 完成 |
| 自动化测试（后端 192 项 + 前端 153 项） | ✅ 通过 |
| 10 分钟确定性 soak 测试（零错误、零 NaN） | ✅ 通过 |
| Quest 3 真机 20 分钟验收 | 🔲 未开始 |
| Cannon-es 物体碰撞与抓取 | 🔲 未开始 |
| MR/Passthrough 真机遥操作 | 🔲 未开始 |
| 乐白真实 SDK 接入 | 🔲 未开始 |
| 相机、Episode 记录、数据集导出 | 🔲 未开始 |

---

## 2. 系统架构

### 2.1 总体架构图

```text
┌─────────────────────────────────────────────────────────────────┐
│                     Quest 3 / Desktop Browser                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │ WebXR (VR)   │  │ Desktop Input│  │ HUD + ArmPanel +     │   │
│  │ Controller    │  │ Mouse+KB     │  │ VrSafetyPanel        │   │
│  │ Input Reader  │  │ Safety Layer │  │ (Chinese UI)         │   │
│  └──────┬───────┘  └──────┬───────┘  └──────────┬───────────┘   │
│         │                 │                      │               │
│         └────────┬────────┘                      │               │
│                  │ VRFrame (60 Hz)               │ RobotState    │
│                  ▼                               │ (20 Hz)       │
│  ┌──────────────────────────────┐                │               │
│  │     SimulationScene          │◄───────────────┘               │
│  │  Three.js WebGLRenderer      │                                │
│  │  LM3 GLB Model + Joint Anim  │  RobotStateBuffer              │
│  │  Target Marker + Grid        │  (Interpolation,               │
│  │  Workbench + Bounding Box    │   100ms staleness)              │
│  └──────────────┬───────────────┘                                │
│                 │ TeleopSocket (WSS, auto-reconnect)              │
└─────────────────┼────────────────────────────────────────────────┘
                  │
    HTTPS + WSS (Vite dev server :5173, self-signed cert)
                  │
┌─────────────────┼────────────────────────────────────────────────┐
│   Vite Dev Server (0.0.0.0:5173)                                 │
│   └─ /ws → ws://127.0.0.1:8000  (WebSocket proxy)                │
└─────────────────┼────────────────────────────────────────────────┘
                  │
┌─────────────────┼────────────────────────────────────────────────┐
│   FastAPI Backend (127.0.0.1:8000)                               │
│                                                                   │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │  WebSocket /ws/v1/teleop                                     │ │
│  │  ├─ Single-owner enforcement (4409 on duplicate)             │ │
│  │  ├─ _receive_messages: VRFrame ingestion                    │ │
│  │  └─ _delayed_state_sender: RobotState @ 20 Hz               │ │
│  └──────────────────────────┬──────────────────────────────────┘ │
│                             │                                     │
│  ┌──────────────────────────▼──────────────────────────────────┐ │
│  │  RobotControl (50 Hz tick, 20ms period)                      │ │
│  │  ┌─────────────────┐  ┌──────────────┐  ┌────────────────┐  │ │
│  │  │ TeleopStateMachine│  │CoordinateMapper│  │ SafetyLimiter  │  │ │
│  │  │ (8-mode FSM)     │  │ VR→Robot frame │  │ Vel/Accel/     │  │ │
│  │  │ DISCONNECTED→    │  │ Anchor+Delta   │  │ Envelope       │  │ │
│  │  │ READY→ARMED→     │  │ R_BX rotation  │  │ bounds         │  │ │
│  │  │ ACTIVE↔HOLD      │  │ Scale: 0.8×    │  │                │  │ │
│  │  │ STALE/FAULT→     │  └──────────────┘  └───────┬────────┘  │ │
│  │  │ DISARMED         │                             │           │ │
│  │  └─────────────────┘                             │           │ │
│  │                    ┌──────────────────────────────┘           │ │
│  │                    │ PoseFilter (8 Hz 1EMA + SLERP)            │ │
│  │                    ▼                                          │ │
│  │              backend.command_tcp(limited_pose)                 │ │
│  └──────────────────────────┬──────────────────────────────────┘ │
│                             │                                     │
│  ┌──────────────────────────▼──────────────────────────────────┐ │
│  │  SimRobotAdapter (in-memory simulator)                       │ │
│  │  ┌────────────┐  ┌──────────────┐  ┌────────────────────┐   │ │
│  │  │ LM3Model   │  │ solve_ik()   │  │ VirtualRobot       │   │ │
│  │  │ MDH params │  │ Damped       │  │ P-controller       │   │ │
│  │  │ 6-DOF arm  │  │ Least-Squares│  │ Vel/Accel limits   │   │ │
│  │  └────────────┘  │ Jacobian IK  │  │ 50 Hz sim step     │   │ │
│  │                  └──────┬───────┘  └─────────┬──────────┘   │ │
│  │                         │                    │               │ │
│  │                         ▼                    ▼               │ │
│  │                  forward_pose()  ←──  joint_state            │ │
│  │                  (Modified DH)                               │ │
│  └──────────────────────────────────────────────────────────────┘ │
│                                                                   │
│  GET /health → {"status":"ok", "backend":"SIMULATOR", ...}        │
└───────────────────────────────────────────────────────────────────┘
```

### 2.2 部署拓扑

- **后端：** Python FastAPI 进程，绑定 `127.0.0.1:8000`，不直接暴露于局域网
- **前端：** Vite HTTPS 开发服务器，绑定 `0.0.0.0:5173`，使用自签名证书
- **代理：** Vite 将 `/ws` 路径代理到后端 WebSocket
- **客户端：** Quest 3 通过 `https://<PC-LAN-IP>:5173` 访问，WSS 同源连接

### 2.3 关键设计决策

| 决策 | 理由 |
|------|------|
| 纯仿真（不导入 Lebai SDK） | Milestone 1 聚焦安全基础，避免过早耦合真实硬件 |
| 后端仅监听 localhost | 安全隔离，仅 Vite 代理对外暴露 |
| 单一 WS 连接持有者 | 防止多客户端冲突，确保控制权唯一 |
| 60 Hz 前向 / 50 Hz 控制 / 20 Hz 状态反馈 | 解耦渲染、控制与反馈速率，匹配各环节需求 |
| Python 后端计算 IK + TypeScript 前端渲染 | 将计算密集型运动学放在服务端，前端专注可视化 |
| 无前端框架（纯 Three.js + DOM） | 减少依赖、降低复杂度，以 3D 场景为核心 |

---

## 3. 技术栈

### 3.1 后端

| 组件 | 技术选型 | 版本 |
|------|---------|------|
| Web 框架 | FastAPI | ≥0.115 |
| ASGI 服务器 | Uvicorn | ≥0.30 |
| 数据校验 | Pydantic | ≥2.8 |
| 数值计算 | NumPy | ≥2.0 |
| 旋转/插值 | SciPy (Rotation, Slerp) | ≥1.14 |
| 配置管理 | PyYAML | ≥6.0 |
| 测试框架 | pytest + pytest-asyncio + httpx | ≥8.2 |
| JSON Schema 验证 | jsonschema | ≥4.23 |
| 语言 | Python | ≥3.11 |

### 3.2 前端

| 组件 | 技术选型 | 版本 |
|------|---------|------|
| 构建工具 | Vite | 7.x |
| 3D 渲染引擎 | Three.js | 0.181 |
| VR 接口 | WebXR Device API (immersive-vr) | — |
| 类型系统 | TypeScript (strict) | 5.9 |
| 测试框架 | Vitest + jsdom | 4.x |
| HTTPS 开发证书 | @vitejs/plugin-basic-ssl | — |
| 语言 | TypeScript (ES2022) | — |

### 3.3 工具链

| 用途 | 工具 |
|------|------|
| 版本控制 | Git (master 分支) |
| 协议定义 | JSON Schema (Draft 2020-12) |
| 确定性仿真测试 | 自研 soak_simulator.py |
| 可达性测试数据 | 1000 点自动生成夹具 |

---

## 4. 核心模块详解

### 4.1 后端模块树

```
backend/app/
├── main.py                    # FastAPI 工厂函数 + lifespan 生命周期
├── config.py                  # YAML 配置加载 (强制 simulator)
├── timebase.py               # MonotonicClock 时钟抽象
├── api/
│   ├── health.py             # GET /health 健康检查
│   └── teleop_ws.py          # WebSocket /ws/v1/teleop 遥操作端点
├── control/
│   ├── state_machine.py      # 8 状态安全状态机
│   ├── safety.py             # 速度/加速度/工作空间限制器
│   ├── coordinate_mapper.py  # VR→机器人坐标系映射
│   ├── filters.py            # 8 Hz 低通姿态滤波器
│   └── robot_control.py      # 50 Hz 控制循环编排器
├── robots/
│   ├── base.py               # RobotBackend 协议定义
│   ├── sim_adapter.py        # 仿真后端适配器
│   └── lebai_adapter.py      # 真实乐白适配器 (硬禁用占位)
├── schemas/
│   └── messages.py           # Pydantic 消息模型 (Wire Protocol v1)
├── sim/
│   ├── lm3_model.py          # LM3 机械臂 Modified DH 参数
│   ├── kinematics.py         # 正运动学 (Modified DH)
│   ├── ik.py                 # 阻尼最小二乘逆运动学
│   └── virtual_robot.py      # 虚拟机器人关节动力学仿真
└── recording/
    ├── base.py               # RecorderSink 协议
    └── noop.py               # 空操作记录器 (Milestone 1 不记录数据)
```

### 4.2 前端模块树

```
web/src/
├── main.ts                   # 应用入口，组件装配
├── appDisposal.ts            # 可处置接口定义与拆卸顺序
├── protocol/
│   └── messages.ts           # TypeScript 类型定义 + 运行时类型守卫
├── transport/
│   ├── socketUrl.ts          # WebSocket URL 解析
│   └── teleopSocket.ts       # WebSocket 客户端 (自动重连)
├── robot/
│   ├── robotModel.ts         # GLB 模型加载、关节/夹爪动画
│   └── robotState.ts         # 机器人状态缓冲与插值
├── scenes/
│   ├── simulationScene.ts    # Three.js 仿真场景 (桌面/VR 双模)
│   └── vrSafetyPanel.ts      # VR 头显内安全状态精灵
├── ui/
│   ├── armPanel.ts           # 解锁/停止/VR 控制面板
│   ├── hud.ts                # 操作员控制台 HUD
│   └── latency.ts            # 往返延迟追踪器 (P95)
└── xr/
    ├── controllerInput.ts    # Quest Touch Plus 手柄读取
    └── session.ts            # WebXR 会话生命周期管理
```

---

## 5. 数据流与通信协议

### 5.1 端到端数据流

```text
[手柄物理运动]
      │
      ▼
┌─────────────────┐
│ WebXR Frame      │  60 Hz (VR) 或 requestAnimationFrame (桌面)
│ readRightController()
│ → ControllerSample {position, quaternion, grip, trigger, buttons}
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ VRFrame           │  JSON over WebSocket
│ {session_id, seq, │  60 Hz send rate
│  client_mono_ms,  │
│  tracking_valid,  │
│  visibility,      │
│  right: {         │
│    position, quat,│
│    grip, trigger  │
│  }}               │
└────────┬────────┘
         │  WSS → Vite proxy → WS
         ▼
┌─────────────────┐
│ LatestVRFrame    │  Capacity 1, session-gated
│ .publish(frame)  │  旧 session 帧自动丢弃
└────────┬────────┘
         │
         ▼  50 Hz (20ms tick)
┌─────────────────────────────────────────────┐
│ RobotControl.tick()                           │
│                                               │
│ 1. Stale Check: age < 100ms? tracking_valid?  │
│    ├─ soft stale (≥100ms) → STALE stop       │
│    └─ hard stale (≥250ms) → instant DISARM   │
│                                               │
│ 2. StateMachine.observe_grip(grip)            │
│    ├─ grip press + ARMED → ACTIVE (capture)  │
│    ├─ grip release + ACTIVE → HOLD (stop)    │
│    └─ grip release + DISARMED → READY        │
│                                               │
│ 3. CoordinateMapper                           │
│    capture(hand_pose, tcp_pose) on ACTIVE     │
│    target(hand_pose) → raw_requested_pose     │
│    Δ = R_BX · (hand - anchor_hand) · 0.8     │
│                                               │
│ 4. PoseFilter.update(raw, dt=0.02)            │
│    8 Hz cutoff 1EMA + SLERP                  │
│                                               │
│ 5. SafetyLimiter.limit(prev, filtered, dt)    │
│    ├─ speed cap: 0.15 m/s linear             │
│    ├─ speed cap: 0.6 rad/s angular           │
│    ├─ accel cap: 0.4 m/s² linear             │
│    ├─ accel cap: 1.2 rad/s² angular          │
│    └─ envelope: ±0.25m from anchor           │
│                                               │
│ 6. backend.command_tcp(limited_pose)          │
└──────────────────┬──────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────┐
│ SimRobotAdapter                      │
│                                      │
│ solve_ik(target_pose, current_q)     │
│   ├─ Numerical Jacobian (ε=1e-5)    │
│   ├─ Damping λ=0.04                 │
│   ├─ Max 30 iterations              │
│   ├─ Convergence: 2mm pos, 1° rot   │
│   └─ Step clip: 0.12 rad/iter       │
│                                      │
│ VirtualRobot.set_target(target_q)    │
│   ├─ P-controller (gain=2.0)        │
│   ├─ Joint speed ≤ 0.5 rad/s        │
│   └─ Joint accel ≤ 1.0 rad/s²       │
│                                      │
│ forward_pose(current_q)              │
│   └─ Modified DH 6-DOF chain        │
└──────────────────┬──────────────────┘
                   │
                   ▼  20 Hz (50ms interval)
┌─────────────────────────────────────┐
│ RobotStateMessage → WebSocket        │
│ {mode, robot_state, actual_tcp,     │
│  actual_q, gripper, sample_age_ms,  │
│  fault, ack_seq, server_mono_ns}    │
└──────────────────┬──────────────────┘
                   │
                   ▼
┌─────────────────────────────────────┐
│ Frontend                             │
│                                      │
│ RobotStateBuffer.push(state)         │
│   ├─ Capacity 2 (latest only)       │
│   └─ Monotonic timestamp enforced   │
│                                      │
│ RobotStateBuffer.sample(nowNs)       │
│   ├─ Linear interp joints + gripper │
│   └─ 100ms staleness flag           │
│                                      │
│ SimulationScene render loop          │
│   ├─ setJointAngles(interpolated_q) │
│   ├─ setGripper(interpolated_grip)  │
│   └─ update HUD + ArmPanel          │
└─────────────────────────────────────┘
```

### 5.2 通信协议 (Wire Protocol v1)

所有消息以 JSON 编码，通过 WebSocket 双向传输：

**客户端 → 服务端：**

| 消息类型 | 用途 | 频率 |
|---------|------|------|
| `hello` | 初始握手，建立连接 | 连接时 1 次 |
| `vr_frame` | VR 手柄追踪数据（位姿、按键） | ~60 Hz |
| `arm_request` | 请求解锁/使能机械臂 | 按需（A 键） |
| `disarm` | 请求解除使能 | 按需（B 键/退出 VR） |
| `ping` | 心跳/延迟测量 | 按需 |

**服务端 → 客户端：**

| 消息类型 | 用途 | 频率 |
|---------|------|------|
| `hello_ack` | 握手确认 | 1 次 |
| `robot_state` | 机器人状态广播（TCP 位姿、关节角、夹爪、模式） | 20 Hz |
| `arm_ack` / `arm_rejected` | 解锁请求结果反馈 | 按需 |
| `disarm_ack` | 解除使能确认 | 按需 |
| `pong` | 心跳响应 | 按需 |
| `protocol_error` | 协议错误（关闭连接 1008） | 异常时 |
| `connection_rejected` | 连接被拒（关闭连接 4409） | 第二客户端时 |

### 5.3 坐标映射

VR 坐标系到机器人坐标系的转换矩阵 `R_BX`：

```
        VR轴      →    机器人轴
        ─────         ────────
        -Z       →     +X
        -X       →     -Y
        +Y       →     +Z
```

数值形式：
```
R_BX = [[ 0,  0, -1],
        [-1,  0,  0],
        [ 0,  1,  0]]
```

平移缩放因子 0.8：VR 中手移动 10 cm → 机器人 TCP 移动 8 cm，提供精细操作能力。

---

## 6. 安全设计

安全是 VR4Arm 的核心设计原则。系统在多个层面实现了纵深防御。

### 6.1 安全状态机

```
                    ┌──────────────┐
                    │ DISCONNECTED │  初始状态 / 客户端断连
                    └──────┬───────┘
                           │ connect()
                           ▼
                    ┌──────────────┐
          ┌────────│    READY     │  等待解锁
          │        └──────┬───────┘
          │               │ arm() + grip_released
          │               ▼
          │        ┌──────────────┐
          │        │    ARMED     │  已解锁，等待 Grip
          │        └──────┬───────┘
          │               │ grip_press
          │               ▼
          │        ┌──────────────┐
          │   ┌───▶│    ACTIVE    │  跟手运动中
          │   │    └──┬───┬───┬───┘
          │   │       │   │   │
          │   │  grip │   │   │ stale / fault
          │   │release│   │   │
          │   │       │   │   │
          │   │    ┌──┘   │   └──────────┐
          │   │    ▼      │              ▼
          │   │ ┌──────┐  │  ┌──────┐  ┌──────┐
          │   └─│ HOLD │  │  │STALE │  │FAULT │
          │     └──────┘  │  └──┬───┘  └──┬───┘
          │               │     │         │
          │               │     │ stop_complete()
          │               │     ▼         ▼
          │               │  ┌──────────────┐
          │               └─▶│  DISARMED    │  已解除使能
          │                  └──────┬───────┘
          │                         │ grip_release
          └─────────────────────────┘  (返回 READY)
```

**关键安全性质：**

1. **Grip 释放门禁：** 从 DISCONNECTED / DISARMED / 任何停止事件恢复后，必须**先松开 Grip**，然后按 A 键才能解锁。长按无效。
2. **B 优先级：** B（停止）与 A（解锁）同帧触发时，B 总是优先。
3. **断连即锁定：** WebSocket 断开立即触发断开停止；重连后保持锁定状态。
4. **追踪丢失即停止：** 手柄追踪失效 → STALE → DISARMED。
5. **可见性即停止：** VR 会话隐藏/模糊 → 立即 disarm + 重置锁定。
6. **无自动重锁：** 不存在 DISARMED → ARMED 的直通路径，必须经过 READY。

### 6.2 SafetyLimiter — 在线安全约束

| 约束类型 | 限值 | 作用 |
|---------|------|------|
| 线速度上限 | 0.15 m/s | 防止 TCP 移动过快 |
| 角速度上限 | 0.6 rad/s | 防止姿态旋转过快 |
| 线加速度上限 | 0.4 m/s² | 防止突然加速 |
| 角加速度上限 | 1.2 rad/s² | 防止突然旋转 |
| 工作空间包络 | ±0.25 m（从锚点） | 防止运动超出安全范围 |
| 关节窗口 | ±π rad（从 home_q） | 防止关节超限 |
| 关节速度上限 | 0.5 rad/s | 单关节最大速度 |
| 关节加速度上限 | 1.0 rad/s² | 单关节最大加速度 |

### 6.3 多层级停止路径

| 停止触发 | 响应 | 最终状态 |
|---------|------|---------|
| 松开 Grip | 立即停止运动 | HOLD（可恢复） |
| 按 B 键 | 立即停止并解除使能 | DISARMED |
| 手柄追踪丢失 | 立即停止并锁定 | DISARMED |
| VR 可见性变化 | 立即 disarm + 锁定 | DISARMED |
| WebSocket 断连 | 后端执行断开停止 | DISCONNECTED |
| 页面关闭/刷新 | 前端本地停止路径 | — |
| IK 求解失败 | FAULT → DISARMED | DISARMED |
| 控制循环超限 | FAULT → DISARMED | DISARMED |
| 工作空间越界 | FAULT → DISARMED | DISARMED |
| 后端命令错误 | FAULT → DISARMED | DISARMED |

---

## 7. 三维仿真与运动学

### 7.1 LM3 机械臂模型

**Modified Denavit-Hartenberg (MDH) 参数**描述 6 自由度串联机械臂：

| 关节 | a_prev (m) | α_prev (rad) | d (m) | home_q (rad) |
|------|-----------|--------------|-------|-------------|
| 1 | 0 | 0 | 0.21583 | 0 |
| 2 | 0 | π/2 | 0 | −0.7854 |
| 3 | −0.28 | 0 | 0 | 1.5708 |
| 4 | −0.26 | 0 | 0.12063 | −0.7854 |
| 5 | 0 | π/2 | 0.09833 | 1.5708 |
| 6 | 0 | −π/2 | 0.08343 | 0 |

TCP 偏移：(0, 0, 0.09) m

### 7.2 逆运动学 (IK)

- **算法：** 阻尼最小二乘法 (Damped Least Squares)
- **Jacobian：** 数值法，中心差分，ε = 1×10⁻⁵
- **阻尼因子：** λ = 0.04
- **最大迭代：** 30 次
- **收敛条件：** 位置误差 ≤ 2 mm，姿态误差 ≤ 1°
- **每步裁剪：** 关节增量 ≤ 0.12 rad
- **可达性：** 1000 点测试夹具上 ≥ 99% 成功率

### 7.3 虚拟机器人仿真

- **控制器：** P 控制器，增益 Kp = 2.0
- **关节限速：** 0.5 rad/s
- **关节限加速：** 1.0 rad/s²
- **收敛快照：** 关节误差 < 1×10⁻⁴ rad 且减速完成时锁定到目标值
- **确定性：** 同 seed 产生完全相同的轨迹

### 7.4 前端三维渲染

- **模型：** Lebai_LM3.glb (3.3 MB)，GLTFLoader 加载
- **关节驱动：** 6 个 Joint 节点按轴独立旋转
- **夹爪动画：** AnimationMixer 驱动 "Take 001" 片段（前 21 帧映射到 0–1 范围）
- **场景：** 工作台、网格地面、虚线工作空间边界盒、目标位姿标记
- **渲染器：** WebGLRenderer + ACES 色调映射 + 阴影
- **状态插值：** RobotStateBuffer 保留最近 2 帧，线性插值关节角和夹爪值

---

## 8. VR/WebXR 遥操作

### 8.1 手柄映射

| 按键 | 功能 | 触发方式 |
|------|------|---------|
| **A** | 解锁/使能 | 上升沿（必须先释放再按下），Grip 必须松开 |
| **B** | 立即停止并解除使能 | 上升沿，优先级高于 A |
| **Grip** | 运动离合 | 按住 = 跟手移动，松开 = 停止运动 |
| **Trigger** | 夹爪控制 | 模拟量 0–1，连续控制夹爪开合 |

### 8.2 操作序列

```
进入 VR → 自动锁定
  → 松开 Grip, A, B
  → 按 A 一次
  → 等待状态牌显示"已解锁"
  → 按住 Grip 移动手柄
  → 机械臂跟手运动（缩放 0.8×）
  → Trigger 控制夹爪
  → 松开 Grip → 停止运动
  → 按 B → 完全锁定
```

### 8.3 双模支持

| 特性 | 桌面模式 | VR 模式 |
|------|---------|--------|
| 控制器输入 | 鼠标拖拽 + 滚轮 + 空格 | Quest Touch Plus 右手柄 |
| 3D 场景 | 独立 PerspectiveCamera | WebXR 渲染循环 |
| 安全提示 | HUD 状态面板 | 世界空间 Sprite 状态牌 (中文) |
| 会话 ID 前缀 | `desktop-` | `quest-` |
| 帧率 | requestAnimationFrame | WebXR frame callback |
| 模式切换 | "进入 VR" 按钮 | "退出 VR" / session end |

---

## 9. 测试与自动化验证

### 9.1 测试总览

| 层级 | 文件数 | 测试项数 | 覆盖范围 |
|------|-------|---------|---------|
| 后端 API 测试 | 3 | ~23 | WebSocket 端点、健康检查、故障路径 |
| 后端协议测试 | 1 | ~10 | Pydantic 模型、JSON Schema 合规性 |
| 后端控制测试 | 6 | ~45 | 状态机、安全限制器、坐标映射、滤波器、控制循环、记录 |
| 后端仿真测试 | 4 | ~31 | 虚拟机器人、IK、正运动学、适配器 |
| 后端 Soak 测试 | 1 | ~3 | 10 分钟确定性仿真、队列深度、时钟 |
| 前端协议测试 | 1 | ~10 | 消息类型守卫、边界条件 |
| 前端传输测试 | 2 | ~12 | WebSocket 生命周期、URL 解析 |
| 前端场景测试 | 2 | ~19 | 仿真场景、VR 安全面板 |
| 前端 UI 测试 | 3 | ~26 | 解锁面板、HUD、延迟追踪 |
| 前端 XR 测试 | 2 | ~27 | 手柄输入、XR 会话生命周期 |
| 前端其他测试 | 4 | ~22 | 机器人模型、状态缓冲、配置、拆卸 |
| **合计** | **29** | **~208** | — |

> 注：用户报告为后端 192 项 + 前端 153 项 = 345 项；以上为测试函数/logical case 层面的分类统计。

### 9.2 关键自动化验证结果

| 验证项 | 结果 |
|--------|------|
| 10 分钟确定性 soak (seed=42) | ✅ 30,000 控制/仿真步，零 NaN |
| 最大队列深度 | ✅ 1（无消息堆积） |
| 所有 6 项注入事件验证 | ✅ 全部通过 |
| 最终模式 | ✅ DISARMED |
| Soak 相同 seed 复现 | ✅ 完全一致 |
| Soak 不同 seed 不同结果 | ✅ 符合预期 |
| Soak 执行时间 | ✅ < 8 秒（30,000 步） |

---

## 10. Milestone 1 完成情况

### 10.1 已完成 ✅

| 交付物 | 状态 |
|--------|------|
| Quest 3 / 桌面浏览器 WebXR 遥操作链路 | ✅ |
| A 解锁、B 停止、Grip 运动离合、Trigger 夹爪控制 | ✅ |
| Three.js 乐白 LM3 三维仿真（DH 参数、FK、IK、50 Hz 控制循环） | ✅ |
| FastAPI + WebSocket 后端 | ✅ |
| 安全状态机（8 模式、完整停止路径、锁定门禁） | ✅ |
| 坐标映射（VR→机器人） | ✅ |
| PoseFilter（8 Hz 低通） | ✅ |
| SafetyLimiter（速度/加速度/工作空间多级限制） | ✅ |
| 失联/追踪丢失/可见性变化的自动停止 | ✅ |
| 单客户端持有者强制（防冲突） | ✅ |
| 桌面浏览器调试模式 | ✅ |
| Quest 纯 VR 模式 | ✅ |
| 头显内中文安全状态牌 | ✅ |
| 后端 192 项自动化测试 | ✅ 全部通过 |
| 前端 153 项自动化测试 | ✅ 全部通过 |
| 10 分钟确定性 soak（零错误、零 NaN、最终 DISARMED） | ✅ |
| 代码合并到 master | ✅ |
| 工作区干净 | ✅ |

### 10.2 待完成 ⏳🔲

| 交付物 | 状态 | 说明 |
|--------|------|------|
| Quest 3 实机 20 分钟验收 | ⏳ PENDING | 需真人佩戴 Quest 3 完成全部验收清单 |
| Cannon-es 物体碰撞与抓取 | 🔲 | Milestone 2 |
| MR/Passthrough 真机遥操作 | 🔲 | Milestone 2 |
| 乐白真实 SDK 接入 | 🔲 | Milestone 3 |
| 相机采集与时间同步 | 🔲 | Milestone 3 |
| Episode 记录与数据集导出 | 🔲 | Milestone 3 |
| ACT / Diffusion Policy / VLA 数据集导出 | 🔲 | Milestone 3 |

### 10.3 不在此范围内的内容（明确排除）

- 真实机械臂驱动（配置强制拒绝非 simulator 后端）
- Lebai SDK 安装或导入（Python 环境不含此包）
- Cannon-es 物理引擎集成
- MR / Passthrough 功能
- 相机数据采集
- 模仿学习数据集

---

## 11. 后续工作规划

### 11.1 近期

1. **Quest 3 真机 20 分钟验收：** 按照 `docs/milestone-1-acceptance.md` 完成全部 7 大类硬件验收项目，将 PENDING 状态更新为 PASS/FAIL。
2. **网络延迟优化：** 若真机测试中 P95 延迟 > 100ms，需优化渲染管线或网络配置。

### 11.2 Milestone 2（虚拟物体交互）

- 集成 Cannon-es 物理引擎
- 实现虚拟物体的碰撞检测与抓取（Trigger 控制夹爪闭合 → 抓取物体）
- 多物体场景下的物理交互

### 11.3 Milestone 3（真实硬件与数据）

- 接入乐白 LM3 真实 SDK（`RealLebaiAdapter` 目前为空占位）
- 实现 `backend: lebai` 配置路径
- 相机采集、时间同步
- Episode 记录器（`RecorderSink` 协议已定义，待实现）
- 数据集导出（ACT / Diffusion Policy / VLA 格式）

---

## 附录 A：关键文件索引

| 文件 | 用途 |
|------|------|
| `README.md` | 项目概述、架构、启动说明 |
| `config/default.yaml` | 运行时参数配置 |
| `schemas/teleop-v1.json` | 通信协议 JSON Schema |
| `docs/milestone-1-acceptance.md` | Quest 3 真机验收清单 |
| `docs/quest-development.md` | Quest 开发与操作指南 |
| `backend/app/main.py` | FastAPI 应用入口 |
| `backend/app/control/state_machine.py` | 安全状态机实现 |
| `backend/app/control/robot_control.py` | 50 Hz 控制循环 |
| `backend/app/sim/lm3_model.py` | LM3 机械臂 MDH 参数 |
| `backend/app/sim/ik.py` | 逆运动学求解器 |
| `web/src/main.ts` | 前端应用入口 |
| `web/src/scenes/simulationScene.ts` | Three.js 仿真场景 |
| `web/src/xr/session.ts` | WebXR 会话管理 |
| `web/src/transport/teleopSocket.ts` | WebSocket 客户端 |
| `scripts/soak_simulator.py` | 确定性 soak 测试运行器 |

## 附录 B：运行参数速查

| 参数 | 值 |
|------|-----|
| 控制频率 (control_hz) | 50 Hz |
| 状态广播频率 (state_hz) | 20 Hz |
| 软超时 (stale_ms) | 100 ms |
| 硬超时 (disarm_ms) | 250 ms |
| 线速度上限 | 0.15 m/s |
| 角速度上限 | 0.6 rad/s |
| 线加速度上限 | 0.4 m/s² |
| 角加速度上限 | 1.2 rad/s² |
| 平移缩放 | 0.8 |
| 旋转缩放 | 1.0 |
| 关节速度上限 | 0.5 rad/s |
| 关节加速度上限 | 1.0 rad/s² |
| 关节安全窗口 | ±π rad |
| 位姿滤波器截止频率 | 8 Hz |
| IK 阻尼因子 | 0.04 |
| IK 收敛位置阈值 | 2 mm |
| IK 收敛姿态阈值 | 1° |

---

> **结论：** VR4Arm 在 Milestone 1 已经建立起一个**架构清晰、安全可靠、自动化验证充分**的 VR 机械臂遥操作仿真基础平台。项目采用前后端分离的模块化设计，以安全状态机作为核心控制逻辑，具备完善的测试覆盖和多层级安全防护。建议尽快完成 Quest 3 真机验收，然后进入虚拟物体交互或真实硬件接入阶段。
