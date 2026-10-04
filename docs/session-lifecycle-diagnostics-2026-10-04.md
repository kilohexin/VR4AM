# 会话收尾诊断时间边界（2026-10-04）

## 实现与范围

在 `d94ee1a` 的取消时间实验之后，将生命周期诊断接入**现有生产会话代码**，本地开发及验证完成，尚未推送或部署。保留原 `state_sender()`、会话监督、取消/join、`on_disconnect()` 和 owner 释放顺序，没有启用独立状态观察器。

新增事件 `teleop_session_lifecycle`，同一会话使用随机 `session_trace_id`。每条事件携带截至当前已观察到的累计边界；内存诊断合并相邻同类事件时，后续记录仍包含前面的时间戳。没有记录用户帧、配置、凭据或异常正文。

## 字段定义

| 字段 | 含义 |
| --- | --- |
| `accepted_ns` | 本连接取得 owner 后的本地时刻 |
| `tasks_started_ns` | 会话子任务创建、登记完成后的时刻 |
| `cancel_requested_ns` | 会话监督者准备取消尚未结束的同组任务的时刻 |
| `cancel_reason` | `peer_completed`、`external_cancel` 或 `wait_error` |
| `task_exits` | 最多 8 条任务名、回调观察时刻、返回/取消/异常结果和异常类型 |
| `task_done_observed_ns` | 首次任务完成回调被观察到的时刻；各任务自己的时间在 `task_exits` 中 |
| `tasks_joined_ns` | gather 返回、同组任务全部退出后的时刻 |
| `disconnect_started_ns` | 即将调用 `control.on_disconnect()` 的时刻 |
| `disconnect_completed_ns` | 该调用返回或抛错后的时刻 |
| `disconnect_outcome` / `disconnect_error_type` | 返回、取消或错误；只记录异常类型，不记录正文 |
| `owner_released_ns` | 原 owner 已释放的时刻 |
| `phase` / `control_generation` | 当前记录阶段和控制代次 |
| `prior_dropped_events` | 此 trace 在本条记录尝试前未完整写入的次数 |

任务完成时间是 **done callback 的观察时间**，不是协程最后一条语句的精确退出时刻。`cancel_requested_ns` 是监督者发起同组取消的边界，不是用户按键、网络断开或首次异常发生的时刻。未执行到的边界不填充，也不补造零值。

可按同一主机的单调时间轴比较：

- `tasks_joined_ns - cancel_requested_ns`：取消后等待同组任务收尾。
- `disconnect_started_ns - tasks_joined_ns`：任务收尾到断连处理之间的工作，例如已有排演清理。
- 已有停止事件中的 SDK 锁请求/获取、RPC 创建及 await 起止：断连内部的进一步细分。

停止事件尚未增加 `session_trace_id` 外键；关联依靠单 owner 会话区间、时间戳、控制代次及停止原因。重叠的其他控制操作仍需结合原始事件，不能仅按最近时间强行配对。

## 记录方式

`SessionTrace` 只做同步、有限大小的字段整理和入队，没有 await、文件 I/O、后台 flush 任务或异步回退。提供可选的 `try_write_event()` 通道：

- `CommissioningRecorder` 复用原普通事件队列；满队列直接丢弃并累计 `dropped_normal_events`，不占用关键日志队列。后台磁盘写入沿用原 writer。
- 记录器已关闭、不可用或事件无法入队时返回 false。原异步 `write_event()` 和关键记录接口仍按原规则抛错，未改变控制器的记录失败处理。
- `DiagnosticsRecorder` 即使文件入队失败也尝试保留有界内存事件；无法完整记录的次数记入 `dropped_nonblocking_events`。这个计数不保证只有磁盘丢失，例如内存 store 失败也会计数。
- 缺少同步通道或只提供 async 同名方法的 sink 会跳过该通道；不调用异步回退。自定义同步实现必须遵守仅做有界内存工作的契约。

“不等待”不等于零 CPU 开销。序列化和内存更新仍有成本；本轮验证没有新增协程等待、磁盘等待或停止前排队，并未进行真实系统延迟基准。新增事件会使用普通队列容量，繁忙时可能与其他普通诊断一起丢弃。内存环形缓冲也会覆盖旧事件，因此这不是保证持久化的安全审计通道。

`disconnect_outcome=returned` 只表示函数返回。既有记录故障可能使断连方法返回但控制器仍锁定；owner 释放、函数返回、SDK 返回均不表示物理停稳。

## 定向验证

新增 **21 个案例**：

- 同步普通队列入队与磁盘落盘、满队列不占关键停止日志容量。
- fatal/closed 记录器同步丢弃，原异步接口仍抛出原错误。
- 文件队列满/不可用、缺少接口、接口异常或 async 实现时，内存诊断继续保留。
- 累计字段在内存事件合并后保留；任务异常正文不进入诊断；取消回调和最多 8 个任务明细限制。
- 内存 store 异常不会撤销已经完成的普通文件队列入队。
- 使用真实原 `state_sender()` 的会话关闭、外部取消、发送错误和清理错误；队列满、记录失败等场景仍在同一 fake 时刻开始停止，错误和锁存语义保持。

先写记录通道测试，10 个因接口/模块尚不存在而失败；实现后相关 17 个测试通过。再写会话接入测试，8 个因缺少生命周期事件失败；接入后相关 55 个通过。补充 fatal、store 异常和任务容量案例后，记录器、诊断、会话、控制器、显示原型和停止事务定向回归 **465 passed**（10.40 s）。未运行后端全集。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/diagnostics/test_session_trace.py backend/tests/api/test_session_diagnostics.py backend/tests/diagnostics/test_recorder.py backend/tests/diagnostics/test_store.py backend/tests/recording/test_commissioning.py backend/tests/api/test_observer_cancel_timing.py backend/tests/api/test_observer_session_lifecycle.py backend/tests/diagnostics/test_shared_observer.py backend/tests/control/test_state_observation.py backend/tests/control/test_robot_control.py backend/tests/control/test_state_machine.py backend/tests/control/test_recording.py backend/tests/api/test_teleop_ws.py backend/tests/diagnostics/test_display_integration.py backend/tests/diagnostics/test_display_snapshot.py backend/tests/robots/test_snapshot_contention.py backend/tests/robots/test_stop_timing_boundaries.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_stop_transaction.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

主代理复核同步通道、原异步异常行为、任务回调和清理顺序；没有独立第二审阅者。所有 SDK 操作为注入 fake，未启动现场服务、连接真机、发送真实命令或推送。Home、增益、积分步长、200 ms RPC 截止、未确认停止锁存及运动新鲜度限制不变。

## 后续

下一步用合成日志验证生命周期字段与停止事件的离线汇总：明确分出取消/join、排演收尾、SDK 锁等待和 RPC await，保留缺失、丢弃及重叠导致的关联不确定性。已有冻结现场日志不具备新增字段，不能事后填补这些时间边界。默认独立观察接入继续暂缓。
