# 共享状态观察离线实验（2026-10-04）

## 本轮决定

在 `f917abb` 的职责拆分基础上，仅新增测试夹具和实验案例，验证由会话持有者驱动 `state_message()`、显示消费者只合成被动快照的方案。**没有新增生产观察任务，也没有切换默认发送器。** 本轮使用注入的 FakeLebaiClient、内存时钟、手动调度门和进程内 API 测试，没有真机连接、现场服务、推送或部署。

当前 `/ws/v1/teleop` 通过 `teleop_owner` 拒绝第二个控制连接（`controller_occupied`）。因此，先前多消费者实验展示的是未来扩展场景，不能用于声称当前单连接现场已有三倍读取负担。

## 夹具策略与边界

`backend/tests/diagnostics/test_shared_observer.py` 中的 `SharedObserverProbe` 复用上一轮 `DisplayHarness`：

1. 每个合成会话只有一个观察任务，每次完整调用原 `state_message()`，保留故障锁存、状态记录及停止转换。
2. 第一次立即观察，以后每次完成后等待指定间隔；无重叠读取、无补发积压周期。消费者数量不控制观察周期。0 消费者表示合成会话仍存活，不代表允许真实连接退出后继续观察。
3. 显示缺失或过期时不回退 SDK 读取，始终标记设备源采样新鲜度未知。读取用途和起止边界仍来自测试挂钩捕获的原快照。
4. 关闭时先令视图失效，再取消并等待旧观察任务退出；旧任务尚未 join 时拒绝开始新代次。
5. 观察异常向持有者传播，视图失效、不自动重启。原有 `RecorderUnavailable` 路径仍由控制器处理。

这只是离线夹具，不是可直接投产的生命周期实现。特别是，测试模拟了忽略一次取消的 SDK 读取：关闭请求后，旧权威调用仍可能完成记录/状态处理，但旧显示发布被拒绝。只有等待旧任务退出后才允许 fake 断连重连，不能把显示代次检查当成控制副作用隔离。本轮也没有为永不返回的 SDK 调用发明关闭超时或强制释放锁规则。

## 计入权威观察的争用对照

固定假设：完整快照 16 ms、IK 8 ms、PVAT await 8 ms。将状态观察排入 SDK 锁队列后，再提交运动或停止。共享组保留一次真实的 fake `state_message()`，同时多个显示消费者零读取合成。

| 场景 | 状态观察数 | 含运动自身的完整读取数 | 运动处理时间 | 停止 RPC 前锁等待 |
| --- | ---: | ---: | ---: | ---: |
| 单消费者各自观察 | 1 | 2 | 48 ms | 16 ms |
| 单消费者共享观察 | 1 | 2 | 48 ms | 16 ms |
| 三消费者各自观察（假设场景） | 3 | 4 | 80 ms | 48 ms |
| 三消费者共享观察（假设场景） | 1 | 2 | 48 ms | 16 ms |

停止验证读取另外计数且保持执行；RPC 创建后的 200 ms 截止不变，没有 `stop_sys`。这些是固定排序下的合成数值，不是现场性能，也不能解释现场 609 ms SDK await。

此前 80→32 ms、48→0 ms 的对照未计入独立权威观察，本轮补齐为多消费者模型中的 **80→48 ms、48→16 ms**。对当前单消费者形态，没有证明减少读取或 SDK 锁等待。

## 节奏与显示阻塞

16 ms 读取之后分别等待 20/40/80 ms，三次记录完成时间为：

| 等待间隔 | 三次完成时间 | 稳态观察间隔 | 下一次唤醒前 1 ms 的本地读取年龄 |
| --- | --- | --- | --- |
| 20 ms | 16 / 52 / 88 ms | 36 ms | 35 ms |
| 40 ms | 16 / 72 / 128 ms | 56 ms | 55 ms |
| 80 ms | 16 / 112 / 208 ms | 96 ms | 95 ms |

使用已有测试显示阈值 80 ms，第三种在间隙末端已经过期。后续读取耗时增为 120 ms 时，40 ms 等待方案的后续观察间隔变为 160 ms，显示可在刚读取完成时就过期。以上阈值和间隔均是实验参数，未作为生产配置。

调用真实 `state_sender()` 的阻塞发送对照表明：它等待 `send_json()` 返回期间，不会进入下一次状态观察。共享夹具中，被动消费者阻塞或取消不影响观察继续；下一次观察可锁存 fake 后端的 `stop_unverified` 并记录。此结论只针对状态观察路径，不能推断现有整个系统完全停止观察故障。独立观察在网络阻塞时仍会增加 SDK 工作量，必须一并评估。

## 验证结果

新增 **23 个案例**：0/1/3 消费者空闲更新、慢读取、3 个间隔候选、阻塞生产者过期且无回退、阻塞/取消消费者、原发送器阻塞对照、无消费者停止转换、读取/记录异常、记录不可用、取消不立即生效时的关闭/重连，以及 8 个含权威观察的运动/停止争用组合。

首批 19 个案例全部通过；补充 4 个节奏与过期案例后，定向回归 **398 passed**（9.59 s）。这是实验/特征测试，本轮没有新增生产实现的红绿循环。主代理复核；没有独立第二审阅者。未运行后端全集。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/diagnostics/test_shared_observer.py backend/tests/control/test_state_observation.py backend/tests/control/test_robot_control.py backend/tests/control/test_state_machine.py backend/tests/control/test_recording.py backend/tests/api/test_teleop_ws.py backend/tests/diagnostics/test_display_integration.py backend/tests/diagnostics/test_display_snapshot.py backend/tests/robots/test_snapshot_contention.py backend/tests/robots/test_stop_timing_boundaries.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_stop_transaction.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

## 下一步

离线将观察任务的失败与取消纳入现有会话收尾验证：`_run_coupled_session()` 的 FIRST_COMPLETED、同组取消/join、`on_disconnect()` 以及 owner 释放顺序。只让显示失效不足以完成控制会话收尾；当前夹具的通用异常案例中控制模式可以仍为 READY，必须由监督它的会话执行原有清理。

优先验证发送阻塞、观察失败、记录失败、关闭期间迟到读取及重连竞争。默认发送器接入、生产观察周期和无人订阅策略仍未决定。保留 Home 行为、增益、积分步长、`stop_unverified`、停止截止与运动新鲜度限制。
