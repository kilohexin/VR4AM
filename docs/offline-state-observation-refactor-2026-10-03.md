# 状态观察职责拆分（2026-10-03，离线）

## 范围与结论

在 `801c27b` 的显示集成原型之后，将 `RobotControl.state_message()` 拆成四个私有步骤，保留公开入口和原有执行顺序。默认 WebSocket 状态发送器、SDK 读取次数、故障/停止规则、消息协议均未切换。被动显示原型仍未接入默认路径，本轮不产生 SDK 锁争用或现场延迟改善的结论。

仅使用内存 FakeBackend、注入的 FakeLebaiClient 和进程内 API 测试。机械臂保持离线，没有启动现场服务、连接真机、推送或部署。Home 按用户确认的本人误触处理，本轮没有改动。

## 实现

文件：`backend/app/control/robot_control.py`。

| 步骤 | 职责 | 保留的边界 |
| --- | --- | --- |
| `_read_observed_state()` | 一次 `backend.get_state()`，按原条件锁存新后端故障 | 已有控制故障不被后端故障替换；读取异常/取消直接传播 |
| `_compose_state_message(state)` | 同步读取当前控制元数据，构造消息 | 不读 SDK、不记录、不锁存故障、不完成停止；`sample_age_ms` 仍为 VR 输入帧龄 |
| `_record_state_message(message)` | 记录已构造消息及其时间戳 | 仅捕获 `RecorderUnavailable`，调用原有故障处理；其他异常/取消传播 |
| `_complete_pending_stop_from_state(state)` | 用本次原始读取结果及当前控制标志执行原停止转换 | 在记录 await 之后调用，保留 `stop_unverified`、模式、pending、hard-stop 等条件 |

四个方法由原 `state_message()` 顺序调用。没有新增任务、锁、回退读取、配置或观察周期。组合方法使用原有浅复制语义；它没有测量年龄信息，也不是可供缓存消息直接发送的公共 API。

## 必须保留的时序

- 消息和记录描述停止转换前的模式：本次返回 `STALE`，调用结束时控制器可能已是 `DISARMED`。本轮刻意保留这一现有行为。
- 记录期间出现新的 `stop_unverified` 锁存，后续完成步骤必须读取更新后的标志，不能使用 await 前缓存的判断。
- 完成步骤仍使用本次读取所得机器人状态，不额外读取；记录期间进入 `DISCONNECTED` 则不能按旧模式完成停止。
- 记录失败可能触发停止及故障处理。内存后端返回 IDLE 的案例中，请求停止、锁存记录故障、完成停止分别推进一次控制代次；已构造消息仍保留失败前的内容。这是既有语义，不是新增故障恢复策略或物理停稳证据。
- 后端读取或记录发生非专门处理的异常/取消时，不继续执行停止完成步骤。

## 测试证据

新文件：`backend/tests/control/test_state_observation.py`，24 个案例：

- 10 个停止条件组合：STALE/FAULT/READY、IDLE/MOVING/FAULT、pending、hard-stop、unverified；检查记录发生在转换之前。
- 2 个后端故障优先级案例；1 个读取后、记录前的控制元数据采样案例。
- 1 个记录不可用案例；4 个读取/记录异常与取消案例。
- 1 个记录期间锁存未确认停止案例；2 个记录期间状态变化案例。
- 3 个纯组合案例：没有 VR 帧、正常帧龄、时钟回退归零；禁止后端读取和控制副作用。

先在未重构代码上运行 19 个公开入口案例。初次 18 通过、1 失败：测试漏算了 `_stop_backend()` 自身的代次递增，核对源码后修正为三次，19 个全部通过。纯组合测试初次使用了无 setter 属性，改用已有构造参数后，3 个案例按预期因组合方法尚不存在而失败；实现拆分后 22 个通过。再补充 2 个 await 边界案例。

最终定向命令（仓库根目录）：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/control/test_state_observation.py backend/tests/control/test_robot_control.py backend/tests/control/test_state_machine.py backend/tests/control/test_recording.py backend/tests/api/test_teleop_ws.py backend/tests/diagnostics/test_display_integration.py backend/tests/diagnostics/test_display_snapshot.py backend/tests/robots/test_snapshot_contention.py backend/tests/robots/test_stop_timing_boundaries.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_stop_transaction.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

结果：**375 passed**。未重跑后端全集。主代理完成差异和调用顺序复核；本轮没有独立第二审阅者。

## 后续

下一步离线明确权威观察的触发频率和生命周期，再用集成夹具评估一次观察供多个显示消费者复用的总成本。必须保留故障观察、记录失败响应和停止转换的执行机会，并覆盖无人订阅、空闲、断连重连、迟到结果和观察任务失败。只有完成这些验证后，才评估默认发送器接入。现有 80 ms 显示过期阈值只是测试参数，不能直接成为生产策略；本轮不调整增益、积分步长、200 ms 停止 RPC 截止或运动新鲜度限制。
