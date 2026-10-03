# 独立显示快照接口：离线原型 — 2026-10-03

新增 `backend/app/diagnostics/display_snapshot.py` 的 `DisplaySnapshotView`，只由离线测试使用。当前 `state_sender`、`RobotControl`、适配器和 WebSocket 协议均未接入它，因此默认状态推送仍执行原来的读取流程。本轮没有现场降载或性能提升的实测结论。

机械臂保持用户声明的断电状态。没有连接、启动服务、部署、推送或修改现场配置。

## 1. 原型接口

```text
view = DisplaySnapshotView(clock=monotonic_clock, max_local_age_ms=display_threshold)
generation = view.begin_connection()

# 调用方在读操作开始前保留 generation，完成后发布匹配的状态和时间。
view.publish(generation, completed_state,
             read_started_ns=read_start, read_completed_ns=read_end,
             read_purpose="motion")

display = view.read(fault=current_authoritative_fault)
view.disconnect()
```

这是单事件循环中的被动视图：没有 SDK、锁、后台任务、网络调用或自动回退读取。缺失或过期时明确返回状态，新增回退读取数的上限为 **0**。显示端取消也不持有或取消生产者任务。

发布方负责传入同一次已完成读取的状态和时间边界；连接代次必须在开始读操作时捕获，不能等旧操作返回后再贴上新连接代次。连接和断连操作会清空缓存并变更代次，旧代次发布被拒绝。代次只用于本地生命周期隔离，不是身份认证或运动许可。

## 2. 显示状态与新鲜度

| status | 含义 | 数据处理 |
|---|---|---|
| `available` | 本地读取起点年龄小于显示阈值 | 返回防御性复制的缓存数据 |
| `stale` | 年龄达到或超过阈值 | 返回旧数据及明确年龄，由显示层标为过期 |
| `missing` | 此连接代次尚无完成快照 | 不返回位姿，不发起读取 |
| `disconnected` | 调用方已使本地视图断连 | 不返回缓存位姿，仍可携带当前故障 |
| `invalid_clock` | 当前时钟早于已记录的读取完成时间 | 不返回位姿，不将负年龄伪装成新鲜数据 |

`local_read_age_ms` 从原始读取起点计算；`local_completion_age_ms` 从读取完成时间计算。反复读取不会更新时间戳，不会让缓存重新变新。`source_freshness` 始终为 `unknown`，因为本地读取时间不能证明设备源采样时间。

阈值由构造方显式指定，原型不提供生产默认值。测试使用 80 ms 作为显示阈值，覆盖 79/80/81 ms 边界；这不是新的控制新鲜度门限或现场参数建议。`available` 也不表示机器人安全、已停止或设备源反馈新鲜。

## 3. 故障、顺序与数据隔离

- 每次 `read` 都必须传入当前权威故障参数；当前故障覆盖缓存内旧的健康状态。缺失当前故障时保留快照自身的故障。
- 视图不拥有故障锁存或清除权限。调用方传入 `None` 不会改变适配器的 `stop_unverified`；是否可以清除故障仍由现有控制逻辑决定。
- 输入状态在发布时深复制，输出也深复制。修改输入或任一输出中的嵌套 TCP 不影响后续读者。
- 读取边界必须是非负整数，起点不晚于终点、终点不在未来。乱序或重复的完成快照不会覆盖更新的快照。
- SDK 锁被占用时，显示读取仍同步返回；取消显示消费者不取消独立的生产者。

## 4. 与控制路径的边界

这个类没有实现 `RobotBackend`，不能作为 `get_state()` 的替代品。测试通过注入 fake 适配器取样，再显式发布到视图：

- 连续 100 次显示读取新增 SDK 读取数、写入数均为 0，也不替换适配器内部快照。
- 经过 341 ms 后即使显示读取 100 次，运动请求仍按原有规则报 `robot_state_stale`，没有 PVAT 写入。
- `stop_unverified` 仍阻止 preflight 通过，显示读取不能解除锁存。
- 直接调用现有 `adapter.get_state()` 仍执行新的 SDK 读取。

测试里的缓存状态保留原始 `RobotStateMessage` 字段，包括 `server_mono_ns`、`sample_age_ms`、`ack_seq` 和控制模式。**未来不能直接把这份缓存消息当作当前 WebSocket 状态发送。** 控制模式、确认序号和 VR 输入帧龄等字段必须由当前权威控制状态重新合成；机器人本地读取年龄应单独显示。原型未改变任何现有协议或前端。

## 5. 验证

先添加测试，确认新模块缺失导致预期导入失败；实现后首批 19 项通过，补充运动快照年龄不被续期、故障锁存和时钟回退后，共 **22 个新增案例**。

新原型、SDK 争用、停止时序、适配器控制和停止事务的定向回归共 **141 passed**（5.01 s）。未运行后端全集或前端构建。生产引用检索确认：目前只有测试导入这个模块。

复现（仓库根目录）：

```text
.venv/Scripts/python.exe -m pytest backend/tests/diagnostics/test_display_snapshot.py backend/tests/robots/test_snapshot_contention.py backend/tests/robots/test_stop_timing_boundaries.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_stop_transaction.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

## 6. 下一步范围

原型验证了数据与任务隔离，尚未验证真实发布流程或 UI 展示。下一步可在离线集成夹具中：

1. 把适配器已完成快照、连接代次和当前故障显式接入发布方，验证断连/重连时序以及迟到回调。
2. 将当前控制元数据与缓存机器人测量分开合成，设计独立的本地读取年龄及缺失/过期展示，避免误用原来的 VR `sample_age_ms`。
3. 在相同注入读取负载下比较 SDK 读取数量和运动/停止等待，确认显示消费者不会隐式触发读取。

默认生产路径保持现状。后续接入前还需考虑没有运动快照时的显示更新策略；本轮选择明确返回缺失/过期，不自行增加回退轮询。现场 609 ms 停止 await 的来源仍未解决。
