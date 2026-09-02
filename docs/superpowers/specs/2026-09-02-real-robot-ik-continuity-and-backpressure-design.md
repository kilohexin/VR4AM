# 真机 IK 连续性与跟踪回压设计

日期：2026-09-02  
状态：用户已批准
适用范围：乐白 LM3 真机 PVAT 遥操作与真机 smoke；`LEBAI_FAKE` 复用同一策略
做离线覆盖，但不构成硬件验收；不改变 `SIMULATOR` 数值 IK、夹爪、WebXR
坐标映射或相机功能。

## 1. 背景与现场证据

提交 `5e49d3a` 增加了只读诊断事件 `ik_candidate_rejected`。现场会话
`20260902T103828Z-27b34b9a` 在 `teleop_ready_q` 执行 `+roll 1°` 时记录到：

- command 3、4、6、8 成功发送 4 个 PVAT 点；
- command 10、12、14、16、17 因 `ik_joint_jump` 被拒绝；
- 被拒绝目标相对实际 TCP 的旋转误差从 `0.414°` 增至 `0.703°`；
- `max_abs_delta_q` 从 `0.0529 rad` 增至约 `0.087 rad`；
- command 14、16、17 的完整 `solution_q` 相同，但随着实际 J3 继续追赶，
  `max_abs_delta_q` 从 `0.0858 rad` 降至 `0.0816 rad`；
- 停止前 J3 实际变化约 `-0.01064 rad`（`-0.61°`），TCP 实际旋转约
  `0.308°`，TCP 位置漂移约 `0.63 mm`；
- 相邻成功 PVAT 事件间隔约 `93–110 ms`，本次会话的有效发送频率约
  `10 Hz`，低于配置的 `25 Hz`。

因此，“机械臂完全未动”和“J3 每帧直接跳变 `0.05–0.09 rad`”都不是
准确描述。当前 `delta_q` 的含义是“完整 IK 解减去仍在追赶的实际关节角”，
它同时混合了两种本应分离的量：

1. 相邻完整 IK 解是否连续；
2. 完整 IK 目标领先实际机械臂多少。

现有 `build_pvat_point()` 用 `max_joint_step_rad=0.05` 直接限制
`solution_q - actual_q`，所以目标与真机之间的正常跟踪积压被误判为 IK
分支跳变。直接把该值改成 `0.09–0.10` 只会推迟故障；静态 `+roll 1°`
探针已经测得约 `0.196 rad` 的完整关节差。直接全局放到 `0.20` 又会失去
独立的分支连续性保护。

## 2. 目标与非目标

### 2.1 目标

- `max_joint_step_rad` 只判断相邻完整 IK 解，恢复“关节解连续性”语义；
- 用独立上限约束完整 IK 解相对实际关节的跟踪积压；
- 积压过大时实施回压：暂停推进新目标，同时继续用受限 PVAT 追赶最后一个
  已接受的安全 IK 解；
- 保持现有关节软限位、PVAT 速度、加速度、停止、急停和失联保护；
- Grip 松开、Stop、Home、断线或控制代际变化后不得复用旧 IK 解；
- 给现场日志提供足够信息，区分正常推进、插值推进、追赶和真实分支跳变。

### 2.2 非目标

- 本次不提高 `max_joint_speed_radps=0.15`、
  `max_joint_acceleration_radps2=0.5` 或 TCP 速度；
- 本次不修改 `pvat_send_hz`，约 `10 Hz` 的现场有效频率作为后续独立性能问题；
- 本次不处理准备姿态的 `+z` IK 边界；
- 本次不开放 Quest 真机连续遥操作，只恢复并验收 `+roll 1°` smoke；
- 本次不承诺 TCP 路径为严格笛卡尔直线。

## 3. 三种方案与选择

### 3.1 方案 A：旋转时直接放宽到 `0.20 rad`

改动最少，可能快速通过 smoke，但同一个阈值继续同时承担分支跳变和跟踪
积压两种职责。它无法可靠区分“同一分支上的安全远目标”和“错误 IK 分支”，
不作为正式路线。

### 3.2 方案 B：降低旋转目标推进速度

保留现有 `solution_q - actual_q <= 0.05`，通过降低角速度和角加速度让真机
尽量跟上目标。它会进一步恶化已经偏慢的跟手体验，而且现场有效 PVAT 频率
低于配置时仍可能再次积压，不作为正式路线。

### 3.3 方案 C：分离连续性与跟踪误差，并加入回压（采用）

相邻完整 IK 解使用 `0.05 rad` 连续性阈值；完整解相对实际关节使用独立的
`0.25 rad` 跟踪误差上限。目标推进过快时不锁存 IK 故障，而是继续追赶最后
一个安全目标。实际写入的 PVAT 点仍由速度和加速度限制缩短。

`0.25 rad` 是首轮现场值：它覆盖已测的 `+roll 1° ≈ 0.196 rad` 完整关节
差并保留约 25% 余量，但不代表允许关节单步移动 `0.25 rad`。它必须作为
显式真机配置项，经 smoke 数据复核后才能调整。

## 4. 数据模型与配置

### 4.1 配置语义

`real_robot.control` 保留：

```yaml
max_joint_step_rad: 0.05
```

其语义改为：

```text
max(abs(candidate_solution_q - previous_accepted_solution_q))
```

新增必填项：

```yaml
max_joint_tracking_error_rad: 0.25
```

其语义为：

```text
max(abs(candidate_solution_q - actual_q))
```

配置加载必须满足：

- 两项均为有限正数；
- `max_joint_tracking_error_rad >= max_joint_step_rad`；
- `max_joint_tracking_error_rad <= 0.50 rad`，防止误配置成无界积压；
- 真机本地 YAML 缺少新字段时失败关闭，不使用静默默认值。

### 4.2 适配器运行状态

`RealLebaiAdapter` 新增并维护：

- `last_accepted_solution_q`：上一成功写入 PVAT 所对应的完整 IK 解；
- 复用现有 `last_sent_tcp`：上述完整解对应的 TCP 目标；
- `previous_sent_qd`：上一实际发送 PVAT 点的速度，语义不变。

只有 `move_pvat()` 成功返回后，才能同时提交新的 TCP 目标和完整 IK 解历史。
IK 求解成功但 PVAT 写入失败时不得推进历史。
`catch_up` 虽然也会成功写入 PVAT，但它只更新 `previous_sent_qd`，不得更新
`last_sent_tcp` 或 `last_accepted_solution_q`；这两个字段始终表示最后一次
获准推进的完整目标，而不是最后一次物理写入。

## 5. 候选求解与回压流程

### 5.1 首个候选

每个新请求先调用乐白 SDK IK：

```text
seed_q = last_accepted_solution_q（存在时）
       | actual_q（控制段首帧）
candidate_solution_q = kinematics_inverse(requested_tcp, seed_q)
```

结果必须先通过六轴有限性、软关节限位和 SDK 异常检查。

控制段首帧没有历史时，连续性参考为 `actual_q`。由于 Grip 建锚首帧应当接近
实际 TCP，若首帧仍超过 `max_joint_step_rad`，保持现有拒绝和故障升级逻辑，
不得用较大的跟踪误差阈值掩盖锚点跳变。

### 5.2 相邻解连续性

有历史时计算：

```text
solution_step = max(abs(candidate_solution_q - last_accepted_solution_q))
```

若 `solution_step <= max_joint_step_rad`，候选连续。若超过，允许最多两次有界
Cartesian 插值重试：

1. 从 `last_sent_tcp` 到 `requested_tcp` 插值一个更近目标；
2. 初始比例按 `0.8 * max_joint_step_rad / solution_step` 估计，并限制在
   `(0, 1)`；
3. 使用上一完整解作为 IK seed，重新求解后按精确关节差重新验证；
4. 若第一次仍超限，比例减半再试一次；
5. 两次仍无法得到连续解时，记为真正的 `ik_joint_jump`，沿用连续失败后的
   停止与故障升级。

该插值只限制目标推进量，不对 IK 关节值做裁剪，也不接受未经 SDK 验证的
线性关节插值结果。

### 5.3 跟踪误差与追赶

连续候选还需计算：

```text
tracking_error = max(abs(candidate_solution_q - actual_q))
```

- 若不超过 `max_joint_tracking_error_rad`，候选可进入 PVAT 构造；
- 若超过，新候选不提交为历史，也不计入连续 IK 失败；
- 若已有 `last_accepted_solution_q`，本周期改为对该已接受解构造一个新的受限
  PVAT 点，使实际机械臂继续追赶；
- 若没有历史，无法安全追赶，按 `ik_joint_jump` 拒绝；
- 若已接受解本身相对最新 `actual_q` 也超过跟踪上限，说明实际机械臂偏离了
  已批准轨迹，停止并进入故障，不继续追赶。

追赶周期设置内部原因 `ik_tracking_lag`，对现有 RobotControl/HUD 继续映射为
`motion_continuity_boundary`，不扩展 WebSocket 枚举。RobotControl 在该软约束
存在时不推进自己的 `last_target`，从而给真机追赶时间；下一周期仍可提交同一
目标进行重试。新候选成功后清除该约束和相关计数。

### 5.4 PVAT 物理边界

连续性和跟踪检查通过后，`build_pvat_point()` 才根据最新 `actual_q`、
`actual_qd` 和 `previous_sent_qd` 生成实际点：

- 完整 IK 解必须在软关节范围内；
- 纯数学层再次验证完整解相对实际关节不超过
  `max_joint_tracking_error_rad`，作为适配器检查之外的纵深保护；
- 实际和参考关节速度必须有效且未越界；
- 完整六轴速度向量按 `max_joint_speed_radps=0.15` 同比例缩放；
- 完整六轴速度修正向量按 `max_joint_acceleration_radps2=0.5` 同比例缩放；
- `p = actual_q + qd * pvat_horizon_s`；
- 静止起步、`80 ms` 时间窗下首点最大仍为 `0.0032 rad`，不会因为
  `max_joint_tracking_error_rad=0.25` 而直接跳到完整 IK 解。

## 6. 状态提交、停止与并发

以下操作必须原子地遵守控制代际：

- IK/PVAT 期间 Stop、Grip 松开、Home、断线或 fault 会使 generation 失效；
- generation 失效后，旧请求不得写 PVAT，不得提交新的 TCP/IK 历史，不得
  恢复 `motion_accepted`，也不得锁存迟到故障；
- `stop()`、Home、disconnect、pump fault 和未验证停止均清除
  `last_accepted_solution_q`、`last_sent_tcp`、`previous_sent_qd`；
- 诊断回调不得持有 SDK 锁；诊断失败不得改变运动结果；
- 真正的 `ik_joint_jump`、软关节越界、SDK 超时和状态陈旧仍沿用现有失败关闭
  逻辑。

## 7. 日志

`pvat_sent` 增加：

- `ik_solution_q`：本周期使用的完整安全 IK 解；
- `solution_step_rad`：相对上一已接受完整解的最大轴差；
- `tracking_error_rad`：相对本周期实际关节的最大轴差；
- `pvat_mode`：`advance`、`interpolated_advance` 或 `catch_up`；
- `accepted_target_command_id`：最后一次推进 TCP/完整解历史的命令 ID；
- `requested_tcp` 与 `target_tcp` 继续分别记录原请求和本次采用目标。

回压发生时增加 best-effort `ik_tracking_backpressure`，至少记录当前请求 ID、
实际关节、已接受完整解、请求候选解、跟踪误差、上限和采用模式。日志写入失败
不得阻塞 stop 或覆盖原控制结果。

## 8. 测试与验收

### 8.1 离线自动测试

先写失败测试，再实现：

1. `solution_q - actual_q > 0.05`、但相邻完整解差 `< 0.05` 且跟踪误差
   `< 0.25` 时必须允许，并证明实际 PVAT 点仍满足速度/加速度上限；
2. 相邻完整解差 `> 0.05` 时，即使相对实际关节很近，也必须拒绝或通过有界
   插值缩短后再接受；
3. 跟踪误差 `> 0.25` 时不推进目标历史，转为追赶上一完整解且不累计
   `ik_failure_persistent`；
4. 已接受完整解相对实际关节也越过 `0.25` 时停止失败关闭；
5. 复放本次 roll 诊断中的连续解：command 10 到 12 最大相邻解差约
   `0.0361 rad`，后续更小，不得再被误判成分支跳变；
6. 首帧锚点跳变、真正 IK 分支跳变、关节软限位和速度越界仍被拒绝；
7. Stop 与 IK、插值、追赶、PVAT 写入和诊断阻塞的竞态均不会恢复旧历史或
   产生迟到写入/故障；
8. 配置缺失、上下界错误和本地 YAML 迁移均失败关闭；
9. 完整后端回归通过，前端协议因复用现有约束枚举无需变更。

### 8.2 首轮现场验收

现场更新本地 YAML，显式加入：

```yaml
max_joint_tracking_error_rad: 0.25
```

随后严格按以下顺序：

1. readonly 预检并确认 IDLE、无急停、TCP/Home/准备姿态匹配；
2. 切回 control，观察员与急停就位；
3. 运行一次 `prepare`；
4. 只运行一次 `+roll 1°`；
5. 保存完整会话，不继续其他旋转轴，等待主机端复核。

本轮通过条件：

- 不出现 `ik_failure_persistent`、`ik_joint_limit`、状态陈旧或停止失败；
- 动作达到 smoke 旋转容差并稳定停止；
- 所有实际 PVAT 点满足配置的关节速度、加速度和软限位；
- 日志能解释每次 `advance/interpolated_advance/catch_up`；
- Grip 松开后的最终状态为 IDLE，关节速度持续稳定到停止阈值以内；
- 操作者观察方向正确，无突跳、抖动、异常平移或碰撞趋势。

通过后才依次测试 `-roll`、`±pitch`、`±yaw`。`+z`、Quest 连续真机遥操作、
速度提升和有效 PVAT 频率优化仍分别保留为后续阶段。

## 9. 回滚

代码回滚到 `5e49d3a` 即恢复现有行为。本地 YAML 中新增字段在旧版本中会被
忽略；回滚后仍应切回 readonly 并重新运行预检。任何现场异常都先 Grip 松开或
急停，停止后保存日志，不通过继续放宽阈值绕过。
