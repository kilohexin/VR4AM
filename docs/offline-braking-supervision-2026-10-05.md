# 减速候选的控制与停止监督：离线验证

## 范围与结论

承接 `offline-braking-approach-2026-10-05.md`。本次只增加测试桥接和定向测试：原始 `RobotControl`、坐标映射、PoseFilter、停止事务保持不变，提前减速候选仍未接入默认生产路径。

测试桥接将 `InfeasibleApproach` 转为 `SafetyViolation("target_braking_infeasible")`，让控制层进入已有故障/停止流程。候选本身不负责停止。现有复位白名单不包含此错误，所以故障即使经模拟停止确认转为 DISARMED，也不能通过 Arm、Grip 或在线复位恢复。没有修改白名单、Home 或 200 ms RPC 截止，没有自动 `stop_sys`。

这些结果证明测试桥接可以复用现有监督路径，并不证明候选适合正常 VR 操作。其直线可行域保守，正常换向也可能触发故障；尚未解决连续操作体验、真实 IK 背压或现场停止延迟。

## 场景与证据

新增 `backend/tests/control/test_braking_supervision.py` 共 16 例：

| 场景 | 例数 | 观察与断言 |
| --- | ---: | --- |
| 内存后端，平移/旋转加速后反向 | 2 | 原始滤波后仍真实触发候选不可行；拒绝帧不提交；记录 robot_fault → stop_requested → stop_confirmed；一次 FAULT stop；IDLE 后 DISARMED 仍保留故障 |
| 正常释放 Grip 后重捕获 | 2 | 先执行既有 GRIP_RELEASED stop；再次 Grip 使用新的 actual_tcp 重建锚点和零速度历史；捕获帧不发目标；下一帧保持新锚点 |
| Grip 释放停止失败 | 2 | stop_unverified；清除锚点与 last_target；后续 Grip 不重捕获、不提交目标、不自动补发停止 |
| 实际适配器配注入 SDK，平移/旋转各五种停止结果 | 10 | 成功、持续非零速度、SDK 异常、609 ms 返回、停止期间取消；旧 IK 和后续排队目标撤销；停止后无新增 PVAT、Home 或 stop_sys |

适配器案例使用真实 `RealLebaiAdapter`、生产 pump/stop transaction 和 `FakeLebaiClient`。先提交 20 帧同向输入加速，最后一个 IK 人为阻塞，再提交一个更新目标进入 pending，然后用反向输入触发候选故障。等停止事务建立后释放 IK，以检查旧 IK 结果和 pending 目标均不能继续写入 PVAT。

输入周期固定 20 ms，平移输入为 ±0.1 m、旋转为 ±0.4 rad；映射平移比例 0.2，使用现有低速测试限制与原始 PoseFilter。输入不是现场轨迹，也不拟合真实机器人动力学。模拟 IK 解是固定关节姿态，不能据此声称运动中的物理停止已验证。

| 注入停止结果 | 控制故障 | 模式/权限结果 |
| --- | --- | --- |
| 模拟静止确认成功 | target_braking_infeasible | 状态观察后 DISARMED，Arm/在线复位仍拒绝 |
| 速度持续非零 | stop_incomplete | FAULT，后续零速度/IDLE 不解锁 |
| SDK 抛异常 | stop_unverified | FAULT，不自动恢复 |
| SDK 本地时钟经过 609 ms 返回 | stop_unverified | 200 ms deadline 保持；returned_late，不产生 stop_confirmed |
| SDK 等待期间取消控制 tick | stop_unverified | CancelledError 传播；释放迟到 RPC 后仍不解锁 |

所有适配器案例断言 `network_calls == discovery_calls == 0`，并在 finally 中结束注入等待和断开 fake 客户端。测试中的 control 配置与 stop_move 都只作用于注入客户端；未导入现场配置、未访问设备或启动服务。

## 验证记录

- 新测试先因桥接模块尚未实现而无法收集，随后实现桥接。第一轮集成出现 `robot_state_stale`：测试仅推进时钟，没有按状态消费者刷新快照。修正测试夹具，热身期通过公开 `get_state()` 读取 fake 状态；保留生产新鲜度门槛。14 例通过后增加 pending 和释放失败检查，16 例通过（1.40 s）。这不是生产缺陷修复。
- 10 个相关测试文件合计 **411 passed in 9.01 s**（包含新增 16 例）；不运行后端全集，不改前端。
- 作者自审：检查桥接只有 tests 导入、错误不在生产恢复白名单、最终 SDK 写入序列无新增运动、取消与迟到不会清锁。没有独立第二审阅。

可复现命令（仓库根目录）：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/control/test_braking_supervision.py backend/tests/control/test_braking_approach.py backend/tests/control/test_robot_control.py backend/tests/control/test_input_await_boundaries.py backend/tests/control/test_gripper_frame_freshness.py backend/tests/control/test_state_machine.py backend/tests/control/test_safety.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_stop_transaction.py backend/tests/robots/test_stop_timing_boundaries.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

## 后续

先研究连续手柄输入下的减速/换向处理，明确目标跟随误差与减速路径约束，避免把频繁故障当成可用的 VR 控制。当前证据不足以采用此候选或放宽速度、加速度、停止与复位规则。仿真物理抓取继续排在真机 VR 控制之后。本次仅本地保存，不推送、不部署、不连接真机。
