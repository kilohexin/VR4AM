# 显示快照离线集成验证 — 2026-10-03

本轮新增纯显示合成器 `display_state.py` 和测试专用发布夹具。默认 WebSocket 推送、适配器读取及 `RobotControl.state_message()` 没有改动。没有连接真机、启动服务、部署或推送。

## 1. 接入中发现的控制职责

当前 `RobotControl.state_message()` 包含四种工作：

1. 调用 `backend.get_state()` 做读取。
2. 将发现的后端故障锁存到控制状态。
3. 合成并记录 `robot_state` 消息；处理记录器失败。
4. 在既有条件满足时完成 pending stop 的控制状态转换并推进 control generation。

**因此不能把默认推送直接替换成缓存显示。** 被动合成不执行上述故障、记录和停止转换职责；在准备好独立权威观察流程之前，去掉原调用会改变控制行为。

本轮用测试明确验证：被动合成可以呈现 `stop_unverified`，但不会替控制器锁存故障；它不会凭缓存 IDLE 完成 pending stop，也不会写状态日志。随后调用原 `state_message()` 时，这些职责仍由原路径执行。测试中的 pending stop 状态是合成夹具，不是物理停止验收。

## 2. 发布夹具与连接生命周期

测试夹具包裹 fake 适配器的 `_read_snapshot()`：开始读之前捕获显示连接代次，等待原读取完成后，将该次返回的快照及原始读取边界发布到 `DisplaySnapshotView`。这只是测试钩子，生产适配器没有新增回调或任务。

- 状态、preflight 和运动快照均能通过同一夹具发布；不会为了显示再读一次。
- 断连清空视图，重连创建新代次；旧代次读取即使迟到完成也被拒绝。
- 同时验证了视图代次切换，以及注入 fake 适配器真正执行 `disconnect/connect` 时的旧读取拒绝和新 connection 快照发布。
- 显示消费者被取消时，不取消已经阻塞在 fake SDK 中的生产者；该读取完成后仍正常发布。

这些结果只覆盖本地异步生命周期。fake 适配器使用注入工厂，网络与发现调用均为 0；不代表真实 SDK 的取消或重连行为已验收。

## 3. 当前控制元数据与缓存机器人测量分离

新合成器输出 `ComposedDisplayState`，不是原 WebSocket `RobotStateMessage`，也没有 `robot_state` 消息类型标识。

| 内容 | 来源 |
|---|---|
| 当前时间、ack_seq、控制模式、VR 输入帧龄 | 调用方提供的 `LiveDisplayControl` |
| constraint、recovery_phase、translation_scale、control_generation | 当前控制状态 |
| actual_tcp、actual_q、gripper、robot_state、backend、real_robot_mode | 白名单复制的缓存机器人测量 |
| status、本地读取起止、年龄、读取用途、连接代次 | 被动显示快照视图 |
| 汇总故障 | 当前权威故障优先，其次缓存样本中的故障 |
| source_freshness | 始终 unknown |

测试故意在缓存消息中写入错误的 ack_seq=999、VR 帧龄=999、增益=9。合成后使用实时控制的 ack_seq=42、VR 帧龄=10 ms、增益=1。新 VR 帧到达时，即使机器人测量已经过期，新的 ack_seq 和 VR 输入帧龄仍可以更新，机器人读取年龄也仍单独保留。

RobotControl 进入 ARMED 后，显示控制模式来自当前控制器，不沿用缓存 READY。缓存测量的输出类型根本不包含控制模式、ack_seq、sample_age_ms 或 preflight_ready，避免误把历史元数据作为当前控制许可。

`control.fault` 是调用方提供的实时故障值；本夹具合并了控制器、适配器锁存及 pump 故障，因此它不等同于 `RobotControl._fault`。顶层 `fault` 还会保留缓存样本中的故障。被动合成不反向改变控制模式或锁存状态。未来 UI 接入必须显示汇总故障，并保留后端控制门禁。

## 4. 同负载下的条件性收益

沿用上一轮注入时延：一次完整快照 16 ms，IK 8 ms，PVAT await 8 ms。先排入 3 个显示消费者，再提交一个运动或停止请求。

| 指标 | 三次主动状态读取 | 三次被动显示合成 |
|---|---:|---:|
| 本次运动前后完整运动学读取数 | 4（3 次显示 + 1 次运动） | 1（运动自身） |
| 运动快照 SDK 锁等待 | 48 ms | 0 ms |
| 运动处理开始至发送返回 | 80 ms | 32 ms |
| 停止 RPC 创建前锁等待 | 48 ms | 0 ms |
| 停止验证读取 | 原样执行 | 原样执行 |
| 停止 RPC 创建后的截止 | 200 ms | 200 ms |

这些是固定顺序、固定时延的 fake 对照，量化的是减少显示侧读取的条件性空间。**被动列没有包括未来独立权威观察器的完整开销，不能当作系统整体吞吐或现场停止延迟的改善承诺。** 默认路径没有切换，因此目前没有现场降载效果。两组停止都只有一次 fake `stop_move`，没有 `stop_sys`。

2026-10-04 补充：计入一次权威观察后，三消费者模型的被动侧运动处理为 48 ms、停止前等待为 16 ms。当前默认 WebSocket 仅允许一个控制连接；单消费者模型两侧均为 48/16 ms，未证明当前现场形态减少读取。详见 [共享观察实验](offline-shared-observer-2026-10-04.md)。

## 5. 验证记录

先写测试，确认新合成模块缺失的预期导入失败；实现后首批 7 项通过，补充记录职责、消费者取消、停止等待、ARMED 元数据和 fake 适配器重连，最终 **13 个新增案例通过**。

新旧显示原型、快照争用、停止时序、适配器控制和停止事务定向回归共 **154 passed**（5.06 s）。没有后端全集或前端构建；当前没有前端及生产调用路径改动。由主代理复核边界与差异。

```text
.venv/Scripts/python.exe -m pytest backend/tests/diagnostics/test_display_integration.py backend/tests/diagnostics/test_display_snapshot.py backend/tests/robots/test_snapshot_contention.py backend/tests/robots/test_stop_timing_boundaries.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_stop_transaction.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

## 6. 下一步

先离线拆分“权威状态观察/状态记录/停止转换”与“显示数据合成”，保留原 `state_message()` 作为行为不变的入口，建立等价性回归。之后才评估如何发布已完成快照、如何保留权威观察频率，以及是否切换默认推送。

缺失或过期时仍不触发显示回退读取；空闲时如何更新显示必须与独立观察策略一起确定。暂不改变现场增益、积分步长、反馈新鲜度门限或 200 ms 停止截止。现场 609 ms await 来源仍未解决。
