# VR 参数候选离线比较 — 2026-10-03

本轮只增加合成实验和报告，沿用当前生产映射、滤波、限幅、控制与 IK 选择代码。没有修改运行参数、部署、真机连接或推送。Home 行为、200 ms RPC 截止、`stop_unverified` 均未改动。

## 1. 比较范围与候选

手柄在 1 秒内匀速平移 30 mm、绕 Z 转动 6°，随后保持 3 秒。分别比较纯平移、纯旋转、两者同时发生。原始实验结果见 [45 组汇总 JSON](offline-vr-profile-results-2026-10-03.json)，全部由合成输入生成，不包含现场日志或配置。

| 平移增益 | 30 mm 手柄位移的映射目标 | 本轮用途 |
|---|---:|---|
| 0.1 | 3 mm | 较低请求幅度的对照候选 |
| 0.2 | 6 mm | 既有低速上限的基准候选 |
| 0.35 | 10.5 mm | 超出既有低速增益上限的压力对照，不生成可运行配置 |

旋转映射在当前 `CoordinateMapper` 中固定为 1:1，非 1 值会被拒绝；因此不提出 0.5 旋转增益的配置。旋转死区用仓库默认的 0.5°，不将它认作现场实测值。

三组都使用仓库 `LOW_SPEED_CAPS` 的 TCP 速度 5 mm/s、角速度 0.05 rad/s、加速度 0.02 m/s² 与 0.1 rad/s²、单步 0.5 mm 与 0.2°；关节速度 0.05 rad/s、加速度 0.2 rad/s²。关节解步长 0.05 rad、跟踪误差门限 0.25 rad、PVAT horizon 0.08 s 沿用现有测试配置。实际机器人分支不增加 Grip 锚点工作空间包络。

这不是从现场配置导出的完整 profile，也未证明任何候选可用于真机。

## 2. 输入帧率会影响目标推进

第一组 18 个案例直接调用生产 `CoordinateMapper → PoseFilter → SafetyLimiter`，每个新输入使用控制层固定的 20 ms 积分步长。没有 IK 或机器人反馈。

| 增益 | 50 Hz 新输入：4 秒末目标平移 | 25 Hz 新输入：4 秒末目标平移 |
|---|---:|---:|
| 0.1 | 3.000 mm | 3.000 mm |
| 0.2 | 6.000 mm | 6.000 mm |
| 0.35 | 10.500 mm | 9.424 mm |

纯平移及组合动作的平移结果一致。含旋转的动作在 50 Hz 下末值为 6.000°，在 25 Hz 下为 4.927°，与平移增益无关。

- 按合成墙钟时间计算，50 Hz 下峰值推进速度为 5 mm/s、0.05 rad/s；25 Hz 下为 2.5 mm/s、0.025 rad/s。
- `RobotControl.tick()` 只在新输入帧时推进目标。第二组实验以 50 Hz 调用控制 tick、25 Hz 发布新帧，确认夹在两帧之间的 tick 不产生额外 PVAT。
- 这证明了当前固定步长在不同新输入帧率下的差异，不证明现场当时只有 25 Hz 输入。也不应把统计到的 PVAT 25 Hz 当作 VR 输入 25 Hz。
- 映射目标、限幅目标、IK 接受目标和实际 TCP 是不同量；表中不是机械臂位移。限幅后的瞬时峰值也不能简单由“手速 × 增益”代替。

## 3. 背压敏感性

第二组 27 个案例使用完整 `RobotControl → RealLebaiAdapter` 生产路径，客户端由 `FakeLebaiClient` 注入。每组从新连接的 fake 初态开始，无先前实验的预热关节历史。控制 tick 为 50 Hz，新输入及每帧等待的 PVAT 结果为 25 Hz，4 秒共 100 个发送结果。每帧结果完成后再推进下一模型步，不模拟真实 SDK 延迟或并发积压。

为隔离参数影响，IK 被明确替换为线性敏感度模型：

```text
q0_requested = Kx × (requested_x - 0.3) + Kr × requested_rz
q0_actual_next = q0_actual + response × sent_v0 × 0.04 s
```

其余关节保持测试初值。使用 `(Kx, Kr, response)` 三种组合：`(10 rad/m, 1, 0.25)`、`(50 rad/m, 3, 0.25)`、`(50 rad/m, 3, 0)`。这些数值是压力对照假设，没有从冻结日志拟合，也不是 LM3 的雅可比或动力学。fake 的 TCP 反馈固定在初始姿态，未建立关节至 TCP 的物理一致映射；本轮只比较请求与选择路径，不能预测实际 TCP。

低敏感度组合的 9 个案例均为 100 次 advance、0 次 catch_up。高敏感度结果如下，每个数是 100 次发送中的 catch_up 次数：

| 动作 | response | 增益 0.1 | 增益 0.2 | 增益 0.35 |
|---|---:|---:|---:|---:|
| 平移 | 0.25 | 0 | 18 | 18 |
| 旋转 | 0.25 | 0 | 0 | 0 |
| 组合 | 0.25 | 11 | 34 | 34 |
| 平移 | 0 | 0 | 45 | 45 |
| 旋转 | 0 | 3 | 3 | 3 |
| 组合 | 0 | 53 | 60 | 60 |

高敏感度、response=0.25 时，平移增益 0.2 与 0.35 均在 2.48 s 首次 catch_up，最后接受的平移目标均约 5.952 mm；组合动作均在 1.72 s 首次 catch_up，最后接受平移目标约 4.084 mm。这说明在这些限定场景里，增加增益未增加末尾接受目标。

response=0 不表示系统已经检测或停止了停滞：在本模型窗口内仍能不断发送 advance/catch_up，保持 ACTIVE。较小请求也可能一直未触发跟踪门限；例如纯平移增益 0.1 的目标关节变化为 0.15 rad，小于 0.25 rad 门限，即使反馈完全不动也没有 catch_up。因此“没有背压”不等于“跟随良好”。

旋转案例不受平移增益影响。只降低平移增益不能解决由旋转请求造成的关节跟踪差异。所有案例均断言发送关节速度和加速度不越过所选低速上限；这些断言不代表真机验收。

## 4. 本轮结论与后续

保留 0.2 为离线基准、0.1 为较低请求幅度的对照；0.35 暂只作为压力对照。现有证据不支持通过直接放大增益来解决现场平移幅度小，也不足以选定“正常真机 profile”。本轮没有生成或更改现场 YAML。

下一项离线工作应复核冻结日志的新 VR 帧间隔、控制目标与 PVAT 时间关系，确认固定 20 ms 步长是否可能影响当时的目标推进。需要把“日志可见输入节奏”和“控制实际消费节奏”分开，不能单凭日志条数反推后者；更改积分规则会改变运动行为，应另立证据和定向测试后再判断。

停止 SDK await 的 609 ms 来源仍未确定，本次参数实验不解决这一问题。

## 5. 验证记录

新增 45 个案例：18 个映射/限幅比较、27 个完整控制与 fake IK 敏感性案例。新实验首轮 18 passed / 27 failed，失败源于实验夹具漏掉 fake 适配器初始化；同时校正了发送调用名称断言。修正夹具后 45 passed，随后加入 50 Hz 中间 tick 和固定计数断言再次 45 passed。未修改生产代码来迎合实验。

复现（仓库根目录，已安装离线依赖）：

```text
.venv/Scripts/python.exe -m pytest backend/tests/robots/test_vr_profile_experiments.py -q -s -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

`PROFILE_RESULT` 行包含每组输入条件与输出。使用 `-p no:cacheprovider` 避免已有 pytest 缓存目录的 Windows 写权限提示。

新增实验、既有背压实验，以及坐标映射、滤波、限幅、控制层、IK 策略、PVAT/请求生命周期、停止事务定向回归共 **290 passed**（14.42 s）。未运行后端全集；本轮没有前端改动，未重跑前端构建。

45 条保存结果已逐项核对成功测试输出。独立审查任务因工具额度不足未完成；由主代理复核新增实验、生产调用路径和报告边界，不将其记作独立审查通过。

```text
.venv/Scripts/python.exe -m pytest backend/tests/robots/test_vr_profile_experiments.py backend/tests/robots/test_lebai_backpressure_experiments.py backend/tests/control/test_coordinate_mapper.py backend/tests/control/test_filters.py backend/tests/control/test_safety.py backend/tests/control/test_robot_control.py backend/tests/robots/test_lebai_ik_policy.py backend/tests/robots/test_lebai_pvat.py backend/tests/robots/test_pvat_request_lifecycle.py backend/tests/robots/test_stop_transaction.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```
