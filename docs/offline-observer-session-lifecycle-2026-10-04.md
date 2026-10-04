# 观察任务与会话收尾离线验证（2026-10-04）

## 范围

在 `86ab2a6` 的共享观察实验基础上，新增 `backend/tests/api/test_observer_session_lifecycle.py`。使用真实的 `teleop_websocket()`、`_run_coupled_session()`、接收消息循环和 `RobotControl.on_disconnect()`；仅把 `_delayed_state_sender` 替换为测试夹具中的观察/显示任务组合。底层使用注入的 FakeLebaiClient，Socket 为内存对象，不启动 HTTP 服务、不连接设备。

没有改动生产代码、默认发送器、协议、现场配置或停止策略。显示夹具只发送一次明确标注的 `offline_display_probe` 测试消息以构造阻塞，它不代表可部署的消息格式或完整显示循环。

## 接入实验的职责

- 外层沿用现有会话的 FIRST_COMPLETED、同组任务取消/join、异常传播及 owner 清理。
- 测试替代发送器内含一个共享观察任务和一个被动显示任务。任一任务结束时，立即令显示失效，取消同伴并等待两个任务结束，然后向外层传播异常。
- 内层同时失败时按观察任务、显示任务顺序选择主错误，并收集两个结果。外层既有错误优先级不变。
- 通过包装 `on_disconnect()` 记录进入时序并设置测试等待门，之后实际执行原方法。断连清理进入时检查所有内层任务已退出、发送任务集合已清空，而 owner 仍存在。
- 测试结束释放等待门并收集会话任务结果，fake fixture 检查 network/discovery 计数为零。

## 结果

新增 **14 个案例**，相关定向回归 **412 passed**（9.54 s）。

| 场景 | 验证结果 |
| --- | --- |
| 已运行后的读取、记录、发送异常（3） | 内层同伴退出，外层接收者退出，执行一次断连清理；主异常传播 |
| 接收端关闭、外部会话取消、观察任务取消（3） | 阻塞显示和观察任务退出，取消语义保留，执行断连清理 |
| 断连停止失败，有/无先前发送异常（2） | 保留 `stop_unverified`、FAULT 和未确认标志；先前会话异常优先，否则传播清理异常 |
| 旧 SDK 读取忽略一次取消，接收关闭/发送失败（2） | 旧任务退出前不开始断连清理、不释放 owner；旧显示发布被拒绝 |
| 清理后的新连接（1） | 清理期间第二连接被拒绝；清理后新会话通过 hello，开始新显示代次并正常收尾 |
| 首次读取/记录即失败（2） | 等待首个样本的显示任务被取消，不因缺少初始样本挂起 |
| 观察与显示同时异常（1） | 两个异常均被收集，观察异常为测试组合的主错误，仍完成外层清理 |

在旧读取尚未退出、以及断连清理被测试门暂停这两个阶段，第二连接均收到 `controller_occupied` / 4409。完成收尾后，owner 和发送任务集合被清空，`LatestVRFrame` 被替换为空的新实例。

重连案例也核对了停止幂等性：两次会话收尾均调用 `backend.stop(DISCONNECT)`；没有新运动时，适配器同一停止事务复用已完成的 RPC，fake SDK 的 `stop_move` 只出现一次。没有增加重试或 `stop_sys`。

## 初次失败与修正

初次 10 通过、1 个重连案例超时：测试显示任务固定等待累计周期数等于 1，而第二个会话已经进入下一周期。改为在本次任务开始前捕获周期基线，并绑定控制器清理后新建的 `LatestVRFrame`。随后该案例暴露测试预期错误：它把“两次 backend.stop”当成“两次 SDK stop_move”。核对 `lebai_stop_transaction.py` 的已完成调用复用行为后，同时断言两次后端调用与一次 SDK 调用。补充首次失败及双异常案例后，14 个全部通过。没有为迎合测试修改生产行为。

## 不能据此得出的结论

- join 保证旧任务不跨入新 owner，但 **不保证停止及时性**。本次取消不立即生效的读取会推迟 `on_disconnect()` 开始；若读取永不返回，本轮没有证明收尾可以完成。没有添加强制解锁、遗弃 SDK 任务或新的停止超时。
- 显示代次只能拒绝旧发布，旧权威调用在 join 前仍可能记录或执行原有状态转换。测试保留并展示了这一行为。
- 断连清理失败后，现有代码仍释放连接 owner，同时保留控制器的未确认停止锁存。owner 空闲不等于已停稳、可复位或可运动。
- 本轮只验证测试接入组合。生产观察周期、显示消息完整语义、记录速率和默认接入尚未决定；也没有证明当前单消费者形态减少 SDK 读取。
- 真实机械臂保持用户要求的离线开发边界；测试结果不是物理停止验证，现场 609 ms SDK await 的来源仍未解决。

## 定向命令

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/api/test_observer_session_lifecycle.py backend/tests/diagnostics/test_shared_observer.py backend/tests/control/test_state_observation.py backend/tests/control/test_robot_control.py backend/tests/control/test_state_machine.py backend/tests/control/test_recording.py backend/tests/api/test_teleop_ws.py backend/tests/diagnostics/test_display_integration.py backend/tests/diagnostics/test_display_snapshot.py backend/tests/robots/test_snapshot_contention.py backend/tests/robots/test_stop_timing_boundaries.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_stop_transaction.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

未跑后端全集。主代理复核代码与调用顺序；没有独立第二审阅者。本轮仅本地提交，不推送。

## 下一步

离线核对观察任务取消前后的 SDK 调用退出边界，重点是读取阻塞如何推迟会话断连停止。用受控延迟分别覆盖锁等待、状态读取、运动学读取及记录等待，并量化停止 RPC 创建前的等待。继续保留现有停止截止和未确认锁存；在确认可行的生命周期边界前，不启用生产观察任务。
