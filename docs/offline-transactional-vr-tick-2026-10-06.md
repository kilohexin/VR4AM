# 候选事务与追赶调度接入完整 VR tick 的离线实验

日期：2026-10-06。仅注入假 SDK，本地开发；未连接设备、未部署、未推送。生产代码、现场配置与原始日志未修改。

## 接入方式

新增测试专用 `TransactionalControl`，调用原 `RobotControl.tick()`，保留输入/会话检查、Grip 状态转换、Trigger 夹爪、坐标映射、工作空间投影、滤波、目标采样、await 后新鲜度检查和停止事务。

两个测试替换点把候选生成与接受历史拆开：

1. 限幅位置的 `InputGate` 保存原滤波后的最新目标，向旧 tick 返回已提交姿态作为保持值。旧 tick 在提交返回后给 `last_target` 赋值时，不会因此提前推进历史。
2. backend 的 `command_tcp` 替身在原记录/新鲜度检查之后调用候选协调器或显式追赶接口。一次只保留一个未完成请求；等待期间滤波继续读取最新输入，但不排队积压候选、不重复积分。只有匹配 completion、接受引用和当前控制/pump 权限时，才在收集结果时更新已提交姿态。

此夹具没有复制 tick 主流程。实际 PVAT、IK、夹爪和停止通过原 `RealLebaiAdapter`，其 factory 始终注入 `FakeLebaiClient`。测试直接发布 VR 帧、调用 tick，并按实验节奏读取假状态；**没有运行 WebXR、WebSocket 会话、定时 run 循环或真实网络**。

## 追赶与退出

- 候选收到 `catch_up` 后，固定已有接受引用和关节解；后续新手柄位置/朝向继续经过映射、滤波，但只供给显式追赶请求。候选的非零速度与阶段历史保持冻结。
- 采用上一轮测试探针的 500 ms 总预算及三次合格新反馈观察条件，重复时间戳不增加观察计数。参数仍是实验输入，不是推荐真机配置。
- 观察结束或预算耗尽后，实验调度器调用原 `disarm()`。不自动 Arm，不延续旧速度。显式 Arm、Grip 激活和真实控制流程捕获新锚点后，协调器重新锚定并从零速度开始；旧追赶会话失效。
- 正常释放 Grip 仍进入 HOLD 并停止；下一次有效抓握重捕获，不沿用追赶期间保存的旧手柄目标。
- IK 拒收走实验故障 `candidate_dispatch_failed`；底层 SDK/记录故障保留原故障原因。原 `stop_unverified` 和复位拒绝保持。

## 本轮发现与修正

| 边界 | 发现 | 测试夹具修正 |
|---|---|---|
| 夹爪 await 消耗追赶预算，但 VR 帧仍新鲜 | 直接调用追赶提交会抛出 `budget_expired` | 派发前复核预算，同轮 tick 返回后立即调用原停止流程 |
| 停止发生在旧 tick 的提交回调内部 | 旧 tick 随后的 `last_target` 赋值可能重新写回已清除姿态 | 预算退出在原 tick 返回后执行，断言最终 `last_target is None` |
| pump 的 SDK/记录错误 | 统一改写成候选失败会丢失原因 | 先传播原 pump 故障；记录异常对应 `recording_unavailable` |

第二行是代码审查发现的赋值顺序风险，未作为生产故障结论。首轮重捕获测试曾注入与停止后的实际关节位置相差过大的 IK 解，触发了原跳变门禁；已改用接近该分支实际反馈的假解，没有放宽门禁。

## 验证

新增 29 例：

- 等待 IK 时仅一个候选在途，已提交姿态不前移；继续保留最新滤波输入。重复 tick 不重复提交。
- 追赶期间更新平移/旋转输入，原接受引用与候选历史不变，不新增 IK。
- 观察结束/预算耗尽后停止；显式重新激活、捕获新 TCP，丢弃旧目标，从零历史重新提交。
- 停止异常和 609 ms 迟到保留 `stop_unverified`，禁止 Arm/复位；无自动 `stop_sys` 或 Home。
- 普通候选和追赶两种状态下，夹爪等待期间分别发生 Grip 释放、100 ms 输入过期、新帧到来、并发 disarm，旧输入均不派发。夹爪 RPC 已发送的事实不被撤回。
- 目标记录等待期间释放、过期或新帧到来，不派发旧目标、不清空已提交速度历史。
- 夹爪等待跨越追赶预算，在同一 tick 中结束追赶并清除 `last_target`。工作空间保持也不能绕过预算监督。
- 原记录/SDK 故障原因保留；真实 IK 拒收不会误入追赶。
- 两种调度状态下，正常 Trigger 从 0 到 0.8 再回 0，假 SDK 收到 30% 力设置、20%/100% 开口指令，原 tick 保持 ACTIVE；这只证明软件调用链，不证明物理抓取成功。

定向回归（仓库根目录）：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/control/test_transactional_vr_tick.py backend/tests/control/test_catch_up_supervision.py backend/tests/control/test_candidate_commit.py backend/tests/robots/test_explicit_catch_up.py backend/tests/robots/test_command_admission.py backend/tests/robots/test_command_completion.py backend/tests/robots/test_stop_transaction.py backend/tests/control/test_robot_control.py backend/tests/control/test_input_await_boundaries.py backend/tests/control/test_gripper_frame_freshness.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

结果：**351 passed in 6.89s**，10 个相关文件。未跑后端全集或前端套件。测试先观察缺失模块失败，再实现；预算边界及故障原因扩展测试先失败后修复。主代理自查，无独立第二审查。

## 限制与下一步

- 默认控制仍使用原路径；本轮未修复默认控制中的候选历史不一致，也未证明连续 VR 跟随性能。追赶后显式停止/重激活的实验策略还不适合作为流畅操作的最终交互。
- `InputGate` 的原 `control_target_sample.limited_tcp` 是已提交保持值，不能拿来当新候选的限幅结果；真正候选输入和回执存在夹具的独立 trace 中。这些 trace 列表只用于有限测试，不能直接作为运行时存储设计。
- 夹具访问 adapter/协调器私有状态；反馈由测试注入，没有机器人动力学或关节到 TCP 一致性模型。不把 SDK 返回、IDLE 或观察退出当物理停止证据。
- 下一步提取生产控制中的目标生成/提交边界，建立可显式注入的策略接口和准确的候选采样语义，以消除测试替身对旧 tick 赋值顺序的依赖；默认策略保持兼容，先在假 SDK 中验证。没有明确连续性依据，不自动恢复旧速度历史。
