# 逐请求软件发送结果（本地实现，未部署）

## 已实现

在结构化入队回执上增加 `completion` 结果句柄。正常 pump 入队都会创建句柄；调用者可直接读取 `completion.result`（未完成时为 None），也可 `await completion.wait()`。结果为不可变 `CommandResult`，包含原请求引用、软件处理结局、SDK 本地调用结局、SDK 请求编号，以及有明确证据时的接受引用和 PVAT 模式。

它不需要下一条请求才能看到反馈，也不依赖日志成功送达。SDK 返回/接受状态或 IK 拒绝确定后，在可能阻塞的记录调用之前发布结果。日志后续失败仍保留原故障路径；已经确定的发送事实不被重写成“没有发送”。

默认 RobotControl 仍使用旧 command_tcp 及原限制/约束处理。此结果接口尚未被用于更新候选速度历史，不能据此声称原历史不一致已修复。

## 结果含义

| outcome | 含义 |
| --- | --- |
| overwritten | 待发请求被更新请求覆盖，未进入处理函数 |
| invalidated | 请求所在代次失效；待发请求立即完成，在途请求等处理退出后完成 |
| handled | 通用 pump 处理函数正常结束，但未提供适配器级结果；SDK 状态为 unknown |
| ik_rejected | 本次 IK 未被接受；保留具体错误，包括第五次持久失败升级 |
| sdk_returned | 本地 move_pvat 调用返回，并完成软件接受状态更新；模式可为 advance、interpolated_advance 或 catch_up |
| failed | 软件/SDK 处理异常；保留既有错误码，未知内部异常使用通用码 |
| cancelled | 在途处理被取消；不是“等待结果的调用者取消”等待 |

`sdk_outcome` 单独记录 not_started、unknown、returned、error、timeout、cancelled，因此可以表达“请求已经失效，但在途 SDK 随后仍然返回”。没有 SDK 证据的通用 handler 不能标为 not_started。`sdk_request_id` 与已有 SDK 诊断编号一致；它与入队 submission_id 是不同概念。

`accepted_command` 只在明确完成软件接受/拒绝判断时提供：完整推进对应本请求，catch_up/IK 拒绝可对应先前接受目标。失败或失效路径中的 None 不等于适配器全局历史一定为空。SDK 返回和软件接受均不证明远端执行、物理停止或跟踪成功。

## 失效、取消与时序

- 停止沿用原有 pump generation 撤销规则，立即把在途句柄标记为 invalidated，但不会凭空断言尚未返回的 SDK 已结束。如果底层忽略取消且一直不返回，结果可以保持未完成；本次不提高 60 ms PVAT/200 ms stop 截止，也不把未完成改写成成功。
- `CommandResult.invalidated` 是结果发布时的快照；句柄的 invalidated 标记只反映该请求仍被 pump 跟踪期间观察到的失效。后续 stop 不能把历史 SDK 返回事实改写。使用方仍必须核对当前代次、控制权限和故障，不能把结果或标记当作继续运动的许可证。
- 等待使用 asyncio.shield；一个等待者被取消不会取消实际请求或其他等待者。多个等待者得到同一结果对象。
- 结果只发布一次。SDK 成功返回后若记录失败，结果仍是 sdk_returned，pump 原有 recording_unavailable 故障照常锁存；两者分别表达发送事实和后续运行状态。
- 新补齐异常收尾：外部直接取消 pump 任务或 SDK 自发抛出 CancelledError 导致任务意外退出时，结束待发结果、关闭运行标志，并锁存既有 pvat_pump_failed，禁止预检自动重启。正常 pump.stop 的取消不触发这个额外锁存。此边界修复不自动重试、不复位、不发送额外机器人命令。

## 存储边界

无全局结果表、无结果历史队列、无新增后台任务。pump 仅跟踪当前 pending/in-flight 请求；历史结果由持有回执的调用者保留。1000 次连续覆盖的弱引用测试证明被丢弃的历史句柄可回收。调用者主动保存大量回执仍会占用自己的内存。

`CommandAdmission.completion` 和直接构造 `PvatRequest.completion` 缺省为 None，以兼容旧构造方式；正常 submit 入队均有句柄。旧入口虽不返回回执，内部请求仍能正常结束，失败通过结果值而非未领取的 Future 异常传递。

## 验证

新增 `backend/tests/robots/test_command_completion.py` **25 例**：

- 覆盖立即结束、待发与在途撤销区别、通用 handler 的成功/异常/取消、取消一个等待者。
- advance/catch_up/IK 拒绝无需后续请求即可取得结果；第五次 IK 持久失败仍升级故障。
- 发送日志/拒绝诊断阻塞或失败时，已确定结果仍可取得，原记录故障行为保留。
- SDK 返回/异常/超时/取消 × 是否提前失效，保留实际 RPC 事实。
- 1000 次覆盖的句柄回收；外部取消 pump 不留下待发请求或无消费者队列；SDK 自发取消锁存故障，预检不重启。

初始测试因回执缺 completion 观察到 RED；实现后 19 例通过，扩展后相关小集 62 例通过。自查新增“通用 handler SDK 状态应为 unknown”断言，并临时撤掉本轮外部取消收尾，分别观察到错误状态与未结束 pending 的两个失败；补齐/恢复后重新回归。进一步加入“异常取消必须锁存故障”断言，观察到 fault 为 None 的失败后修复，再增加适配器 SDK 自发取消的端到端案例。

最终相关 12 个文件 **408 passed in 9.72 s**。包含原入队回执、适配器、pump、停止事务、取消边界、VR 输入/夹爪新鲜度，以及前述候选历史不一致案例。未例行重跑后端全集；无前端改动。作者自审，无独立第二审阅。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/robots/test_command_completion.py backend/tests/robots/test_command_admission.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_lebai_pump.py backend/tests/robots/test_pvat_request_lifecycle.py backend/tests/robots/test_lebai_backpressure_experiments.py backend/tests/robots/test_retarget_backpressure.py backend/tests/robots/test_stop_transaction.py backend/tests/control/test_braking_supervision.py backend/tests/control/test_robot_control.py backend/tests/control/test_input_await_boundaries.py backend/tests/control/test_gripper_frame_freshness.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

## 下一步

利用明确结果实现测试专用候选状态协调：核对当前权限/代次，再比较覆盖、拒绝、catch_up、成功接受时的状态提交与冻结，避免对未发送目标持续积分或把非零速度无证据清零。结果不能被当作实际机器人速度观测。默认控制暂不切换，stop_unverified、停止截止、Home/复位规则保持；继续不推送、不部署、不连接设备。
