# 目标策略的回执、撤销与锚点生命周期

日期：2026-10-06。仅本地开发和注入假 SDK 验证；未连接设备、未部署、未推送。

## 本次接口

`RobotControl` 新增可选 `target_lifecycle`。默认 None，不启用实验策略，也不增加默认路径的 SDK 调用、后台任务或配置开关。原 tick 帧处理主体保留在 `_tick_frame` 中。

| 接口 | 调用边界 |
|---|---|
| `collect(TargetContext)` | 有显式生命周期对象时，在帧处理前后同步调用；只检查已完成回执，不等待 SDK |
| `on_anchor(pose, context)` | 原流程校验并捕获锚点、进入 ACTIVE 后同步通知，随后仍执行原状态记录与 await 后输入检查 |
| `on_revoke(generation)` | 控制代次递增后立即通知，先于随后的 backend 停止等待 |

`TargetContext` 包含控制代次、当前单调时间、ACTIVE 状态和输入是否具备当前运动权限。权限检查包括模式、故障、恢复/解除使能状态、当前会话、Grip、跟踪/可见性及帧龄。

`TargetUpdate` 关联控制代次，最多选择一个动作：提交姿态、请求停止、报告故障；也可以不做处理。过时代次的结果被丢弃。姿态提交和停止建议必须有当前运动权限；同代次硬故障仍可在输入过期时锁存。姿态以深拷贝交给控制器保存。

停止建议由控制器调用原 `disarm()`，沿用其现有停止事务和原因编码；实验的预算耗尽/观察退出原因仍在测试 `exits` 中区分。生命周期回调不调用机器人命令、停止或恢复方法，不拥有自动 Arm/复位权限。

## 异常与取消

- collect、anchor 回调失败：禁用该生命周期的运动权限，由原故障流程处理 `target_strategy_failed`。
- revoke 回调失败：先完成控制代次撤销，禁止其继续授权；不抛出回调异常去截断当前停止事务，后续 tick 处理故障。
- 同步钩子自己抛出的 `CancelledError` 也按回调错误处理；同步钩子没有 await，因此它与控制器等待停止操作时收到的任务取消分开处理。
- 控制器在真实 await 处收到取消仍向上传播；停止未确认时继续锁存 `stop_unverified`。回调故障不得把该锁存替换成普通策略错误。

`target_strategy_failed` 没有加入在线复位白名单。回调必须是受信任的同步、非阻塞代码；接口没有给阻塞回调提供强制抢占或物理停止时间保证。

## 实验迁移

实验策略通过正式接口提供回执收集、停止/故障建议、锚点重捕获及撤销通知。测试控制器不再覆写 tick，锚点观察用的限幅器包装也已移除。测试断言：

- `type(control).tick is RobotControl.tick`；
- 使用原 `SafetyLimiter` 实例；
- backend 直接是原 adapter，factory 注入假 SDK。

原有候选/追赶、Trigger 开合、夹爪与记录等待中的输入撤销、预算退出、停止失败和重捕获测试保持通过。实验策略仍保留测试追踪数据和 adapter 私有接受状态读取，尚未变成默认生产策略。

## 验证证据

新增 **23 例**：锚点一次通知与姿态隔离；当前/旧代次/过期/释放/换会话的姿态提交；owner 停止与撤销顺序；过期输入的当前硬故障；三个钩子的普通异常和同步取消异常；互斥动作验证；旧停止/故障建议丢弃；撤销失败不覆盖停止未确认；owner 停止取消传播；非法 collect 返回；移除测试 tick/limiter 包装。

先观察缺少接口的失败，首轮 14 例通过，迁移后三组 74 例通过。扩展测试暴露三处同步 `CancelledError` 可绕过普通 Exception 捕获，修正后通过。最终 **435 passed in 9.06s**，14 个相关文件。主代理自查，无独立第二审查；没有运行后端全集、前端、WebXR 或现场测试。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/control/test_target_lifecycle.py backend/tests/control/test_target_strategy.py backend/tests/control/test_transactional_vr_tick.py backend/tests/control/test_candidate_commit.py backend/tests/control/test_robot_control.py backend/tests/control/test_input_await_boundaries.py backend/tests/control/test_gripper_frame_freshness.py backend/tests/control/test_catch_up_supervision.py backend/tests/control/test_braking_supervision.py backend/tests/robots/test_stop_transaction.py backend/tests/robots/test_retarget_backpressure.py backend/tests/robots/test_explicit_catch_up.py backend/tests/robots/test_command_admission.py backend/tests/robots/test_command_completion.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

## 下一步

为 adapter 提供有代次关联的只读接受状态/观察接口，替代实验策略对 `_accepted_target_ref`、关节解和快照等私有字段的读取，再整理可显式选择的事务策略。默认控制的候选历史问题、流畅连续跟随参数和真机停止及时性仍不能宣称已解决。
