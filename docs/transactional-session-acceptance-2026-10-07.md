# 事务策略完整会话离线验收

日期：2026-10-07。接管计划第 34 项。版本标识：`vr-transactional-session-rc1-2026-10-07`。最终 Git 提交及本地冻结清单由本轮提交后记录；清单位于 `.tmp/first-live-test-candidate-2026-10-07.json`，不包含现场配置或原始日志。

## 已验证的组合路径

`create_app` 正式 lifespan → ASGI WebSocket `/ws/v1/teleop` → 原消息解析/接收、状态与诊断发送 → 原 RobotControl 50 Hz 循环 → 事务策略 → RealLebaiAdapter/pump → 注入内存 SDK → 正式 CommissioningRecorder JSONL → 会话收尾与应用关闭。

没有覆写 tick/run、手动调用 tick、替换会话 supervisor 或改用旧策略。ASGI 消息在内存队列中传输，没有建立 TCP 服务、实际头显或机器人连接。backend_label 使用 LEBAI 以覆盖实际 IK 策略分支；合成日志 metadata 明确标注 `INJECTED_SDK_TEST`、`hardware_verified=false`。全部工厂均为注入 fake，network/discovery 均为 0。

合成 IK 回复和遥测是测试输入，不是机械臂动力学模型。常规会话用显式 1,000 ms 追赶预算、80 ms 观测年龄/间隔；预算退出案例用 200 ms。它们只用于容纳真实调度和测试边界，不是现场参数推荐。

10 个完整会话案例验证：

- 连续 VR 帧推进接受目标；Trigger 从 0 到 0.8 再回 0，假夹爪收到 30% 力度、20%/100% 开度；Grip 释放进入 HOLD 后无新 PVAT。
- 追赶保持接受引用与关节解，不增加 IK；新手位置和 Trigger 仍被处理；足够反馈观测后走原 disarm。
- 追赶预算耗尽停止，保持旧算术历史但不恢复速度；须松开 Grip、显式 Arm 和新捕获，重新以实际 TCP 和零速度历史开始。
- IK 等待中释放或断连，撤销的旧回执不能在迟到返回后派发 PVAT。
- 候选与追赶期间 VR 帧流中断，原过期路径停止命令供给。
- `move_pvat` SDK 异常保留 adapter pump 故障及原 WebSocket 任务异常，不自动继续 IK/运动。
- 断连释放 owner、清空输入、join 发送子任务；再次连接重新获得上下文，旧追赶和速度历史不恢复。
- 注入 609 ms 停止延迟，超过原 200 ms 截止后保持 `stop_unverified`，SDK 迟到返回不清除锁存，协议复位仍拒绝。
- JSONL 的 PVAT command_id 对应被记录的 VR 帧，submission_id 为有效请求序号；停止确认与会话记录可读。writer、control 和 pump 在应用关闭后结束。

SDK 故障时，状态发送任务可能先于控制循环观察到异常，随后会话退出；本轮断言 backend 的故障锁存和传播的原异常，不把它描述成 owner 一定先进入 FAULT。HOLD/STALE 在原流程中可保留显示目标，本轮验证的是权限撤销和没有新运动命令。

## 发现并修复的断连竞态

未修复时，正常追赶会话断连产生 `stop_requested:disconnect`，然后在收尾等待中产生 `robot_fault:candidate_dispatch_failed`，导致重新连接后无法恢复正常操作。原 `on_disconnect` 先撤销代次，但在停止/记录 await 期间，旧输入仍在 newest-frame store，运行循环继续 collect 已撤销的追赶。

新增断连计数守卫，在第一次 await 前撤销当前控制权限；整个收尾期间 tick 返回，异步命令返回后的权限检查也拒绝写回。原收尾抽入 `_disconnect_session`；finally 清除守卫，停止异常/取消仍保留原锁存和异常传播。没有扩大任何 SDK 截止或恢复自动 stop_sys。

4 个确定性回归先失败、修复后通过：默认/可选生命周期两条路径在停止等待中均无新 tick 动作，权限立即失效；取消和异常清除计数但 `stop_unverified` 继续撤销权限。原正常断连/重新连接的完整会话随后通过。

## 验证结果与限制

新增 14 个有实际边界的案例。测试构建过程中的失败均保留在 `.tmp`：先误等持续流的“无 pending”窗口，改为等历史提交；启动/rearm 原门禁要求释放帧被控制循环消费，不能仅等接收端发布；人工跳变 fake TCP 触发旧停止核验不足，改为用当前实际 TCP 验证新捕获。没有为这些夹具条件放宽生产门禁。

专项 14 passed in 9.44s。最终 19 个受影响控制/适配器/会话文件 **545 passed in 20.13s**。仅主执行者差异自查，无独立审查结论；未例行运行后端全集，也未进行前端、WebXR、实际网络或现场验收。

专项复现命令（仓库根目录）：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/api/test_transactional_session.py backend/tests/control/test_disconnect_authority.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

最终回归另包括 `test_transactional_runtime`、`test_target_lifecycle`、`test_target_strategy`、`test_transactional_vr_tick`、`test_robot_control`、`test_input_await_boundaries`、`test_gripper_frame_freshness`、`test_catch_up_supervision`、`test_braking_supervision`、`test_stop_transaction`、`test_explicit_catch_up`、`test_command_completion`、API `test_health`、`test_teleop_ws`、`test_session_diagnostics`、`test_observer_session_lifecycle`、`test_observer_cancel_timing`。

## 现在距离现场测试

此前待办的事务策略运行实现及完整软件会话验收已完成，可以冻结这版代码供首轮现场评估。当前默认策略仍为 legacy，未修改现场配置、部署或推送。

下一项应是用户另行批准的**现场收尾核对与只读连接检查**，具体范围见 `docs/first-live-test-scope-2026-10-07.md`。运动中停止的现场 609 ms 问题仍未得到新证据，不能直接转入连续 VR 抓取。实际头显输入、工具/夹爪标定、可用工作区和正常操作参数也仍需现场核对。

物理仿真抓取继续排后；本轮没有物理引擎接入。
