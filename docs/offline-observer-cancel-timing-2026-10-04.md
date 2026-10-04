# 取消观察到断连停止的时间边界（2026-10-04）

## 范围与结论

在 `cc7b59b` 的会话收尾夹具上新增 `backend/tests/api/test_observer_cancel_timing.py`。复用真实会话监督、控制器断连处理、适配器读取及停止事务，注入 fake SDK 行为和虚拟时钟，**没有修改生产代码或启用独立观察任务**。

本轮把三段时间分开：

1. 取消请求 → 观察任务退出 → 进入断连清理。
2. 断连清理 → 获得 SDK 锁 → 创建停止 RPC。
3. 停止 RPC 创建 → SDK 返回；200 ms 截止从 RPC 创建开始。

前两段不在第三段的 200 ms 截止内。300 ms 快照读取限制也不能被解读为“取消请求必在 300 ms 内退出”的保证：等待中的操作是否响应取消仍决定实际退出时机。保持 owner、join 旧任务和显示代次隔离不能替代停止及时性验证。

## 实验设置

- 初次观察和显示已启动；在下一次观察的状态 SDK 读取、运动学 SDK 读取或状态记录中设置单次等待门。
- 通过内存 Socket 的接收端关闭触发真实会话取消路径。没有人为暂停断连清理。
- fake 调用分别立即抛出取消、收到取消后等待 75/350 ms 再抛出、收到取消后等待 75/350 ms 再返回。
- SDK 状态读取与运动学读取各自原有模拟耗时 8 ms；记录等待之后不添加 SDK 工作。
- 时间由注入时钟推进，事件门控制先后。下面的毫秒数是模型结果，不是墙钟性能、真实 SDK 行为或现场测量。

## 观察退出导致的停止前等待

下表为取消请求到观察任务退出的间隔；这些案例中，随后直接进入断连处理，停止前 SDK 锁等待为 0，停止 RPC 创建时间等于观察退出时间。

| fake 取消行为 | 状态读取处 | 运动学读取处 | 状态记录处 |
| --- | ---: | ---: | ---: |
| 立即抛出取消 | 0 ms | 0 ms | 0 ms |
| 延后 75 ms 抛出取消 | 75 ms | 75 ms | 75 ms |
| 延后 350 ms 抛出取消 | 350 ms | 350 ms | 350 ms |
| 延后 75 ms 返回 | 91 ms | 83 ms | 75 ms |
| 延后 350 ms 返回 | 358 ms | 358 ms | 350 ms |

75 ms 后状态读取返回时，还会执行 8 ms 状态读取与 8 ms 运动学读取；在运动学读取处则剩余 8 ms。350 ms 后的 SDK 返回完成其 8 ms 模型工作，触发原有 `robot_state_stale` 检查，状态读取案例不会继续下一次运动学读取。

等待期间不执行断连停止，也未创建停止 RPC；新的控制连接继续被拒绝。旧记录可能完成，这与上一轮已确认的 join 语义一致。没有遗弃仍在执行的 SDK 调用、强制释放锁或增加停止命令。

## SDK 锁等待是另一段边界

额外 2 个案例中，观察正在等待被其他持有者占用的 SDK 锁。取消观察可以立即退出，但随后的断连停止仍需等待实际持有者释放锁。

| 其他持有者剩余占锁 | 观察退出等待 | 停止前锁等待 | 取消到停止 RPC 创建 |
| --- | ---: | ---: | ---: |
| 75 ms | 0 ms | 75 ms | 75 ms |
| 350 ms | 0 ms | 350 ms | 350 ms |

测试检查取消的观察没有执行 SDK 读取，停止没有绕开锁。RPC 创建后的截止仍是 200 ms，两个模型中的即时 fake 停止均没有被判为 RPC 超时。

## RPC 自身迟到仍保留未确认锁存

以状态记录取消后延迟退出 75 ms 为前置等待，分别模拟停止 SDK await 16/609 ms：

| 指标 | 及时返回对照 | 迟到返回对照 |
| --- | ---: | ---: |
| 取消 → RPC 创建 | 75 ms | 75 ms |
| RPC 前锁等待 | 0 ms | 0 ms |
| RPC 自身 SDK await | 16 ms | 609 ms |
| RPC 截止长度 | 200 ms | 200 ms |
| RPC 结果 | returned | returned_late |
| 控制故障 | 无新增故障 | stop_unverified / FAULT |

609 ms 对照仍触发 `sdk_timeout:stop_move`，没有 `stop_sys` 或自动恢复。这个人为输入只是验证边界，不能用于解释冻结现场日志中同为 609 ms 的来源。

## 异常顺序

初次 19 个案例中 17 通过、2 个预期失败：原测试要求 350 ms 后返回导致的 `robot_state_stale` 从会话抛出，但接收端关闭早已使发送任务组合进入取消收尾，主取消会覆盖后发生的观察错误。

核对任务层次后，改为验证迟到错误保存在已退出的观察子任务中且被 gather 收集，而会话仍按原关闭语义退出。再增加 2 个对照：只取消观察、保持接收者存活，则 `robot_state_stale` 成为主要退出错误并向调用者传播。两个顺序都仍执行断连清理。

这是测试预期修正，没有改变生产错误优先级。后续诊断应记录退出原因及其时间边界，不能仅凭会话最终是否抛错判断内部有无迟到读取错误。

## 验证与后续

新增 **21 个离线案例**；控制、会话、显示原型、适配器及停止事务定向回归 **433 passed**（10.23 s）。未运行后端全集，未启动现场服务、连接真机、修改配置、推送或部署。主代理复核，未进行独立第二审阅。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/api/test_observer_cancel_timing.py backend/tests/api/test_observer_session_lifecycle.py backend/tests/diagnostics/test_shared_observer.py backend/tests/control/test_state_observation.py backend/tests/control/test_robot_control.py backend/tests/control/test_state_machine.py backend/tests/control/test_recording.py backend/tests/api/test_teleop_ws.py backend/tests/diagnostics/test_display_integration.py backend/tests/diagnostics/test_display_snapshot.py backend/tests/robots/test_snapshot_contention.py backend/tests/robots/test_stop_timing_boundaries.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_stop_transaction.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

下一步优先补充当前生产会话路径的取消请求、任务退出、断连清理起止诊断边界，使未来日志能与已有 SDK 锁/RPC 时间戳对应。应采用有界、尽量不等待的记录方式，并验证记录不可用或队列满不会新增停止前阻塞。先保持原发送器及停止规则；本轮证据不足以启用独立观察默认路径或选择新的生产周期。
