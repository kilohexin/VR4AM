# 目标生成与提交策略接口

日期：2026-10-06。仅本地实现与离线验证；未连接真机、未部署、未推送。

## 正式接口

`RobotControl` 新增可选构造参数 `target_strategy`，默认 `DefaultTargetStrategy`，继续使用原 `SafetyLimiter.limit_motion` 和 backend `command_tcp`。生产启动代码没有选择实验事务策略，也没有新增配置开关或前端入口。

`backend/app/control/target_strategy.py` 定义：

| 接口 | 含义 |
|---|---|
| `prepare(previous, requested, dt, limiter=...)` | 同步生成 `TargetPlan`，不发送命令、不提交已接受姿态。记录或权限检查失败时可直接丢弃，不能依赖随后必然调用 dispatch 来清理资源 |
| `TargetPlan.kind` | `candidate`：新候选；`catch_up`：追赶；`wait`：本轮不提交 |
| `TargetPlan.pose` | 实际准备提交的姿态；wait 必须为 None，另两类必须有姿态 |
| `TargetPlan.payload` | 策略自己的局部草案，不参与日志或通用控制判断 |
| `dispatch(plan, command_id, backend=...)` | 在原记录/新鲜度门禁后提交；返回 Pose 才要求 owner 更新 `last_target`，返回 None 表示本轮不提交该历史 |

wait 不调用 dispatch。默认策略保留原候选内部限幅历史的处理方式，本次抽取没有把它变成按 completion 提交的事务策略。接口是受信任程序代码的注入点，不是外部客户端可执行的插件接口。

## 控制层的等待边界

提交可能 await。返回成功或旧软约束后，控制层重新核对同一 VR 帧、100 ms 新鲜度、控制代次及当前权限：

- 新帧、输入过期或 disarm 已发生：不写回旧姿态、不覆盖新一轮的帧序号，不用旧软约束重置当前历史或显示状态。
- SDK 等硬故障仍进入原故障/停止路径，不因输入变旧而忽略。
- 这不能撤回已发送的 RPC；没有扩大停止截止、改变复位白名单或加入自动 `stop_sys`。

此返回后检查也适用于默认策略，是本轮明确的行为修正。正常有效输入下的默认生成/提交规则保持原样。

## 采样语义和实验迁移

原 `control_target_sample` 新增 `dispatch_kind`，取值包括 candidate、catch_up、wait，以及工作空间保持时的 workspace_hold。它描述记录时的准备意图，不证明随后已派发、IK 接受或物理执行；原采样频率和字段保持。

上一轮实验的 backend 替身已移除，测试控制器直接持有原 adapter。限幅器包装仅记录锚点生命周期，不再伪造已提交保持姿态作为 `limited_tcp`：

- candidate 的 `limited_tcp` 是实际候选草案。
- catch_up 的 `limited_tcp` 是被追赶的已接受目标。
- wait 的 `limited_tcp` 为 None。

测试候选协调器拆分 `prepare` 和 `submit_prepared`。前者复制候选历史供采样，不入队、不推进版本；后者复核协调器身份、版本、权限和在途互斥。丢弃旧草案后可以重建；已提交或属于其他协调器的草案不得再次使用。已有 `propose` 保留为两步组合，原测试仍适用。

## 验证

新增 **31 例**，覆盖：四种准备/提交结果组合；默认与显式策略各自在新帧/过期/disarm 后返回成功或软错误；记录等待后的草案丢弃；默认与显式策略的迟到硬故障锁存；四种非法计划；完整 tick 的真实候选/追赶/等待采样；草案丢弃、重复使用和跨协调器提交拒绝。

TDD 过程：先观察缺少接口模块失败；接口与默认控制 166 项通过；迁移后三组 72 项通过。新增迟到硬错误案例使三项失败，修正软约束与硬故障分支后通过，再加入默认策略同类边界验证。

最终定向回归：**412 passed in 8.90s**，13 个相关文件。上一轮 29 项完整 tick 实验，以及旧默认路径的背压特征测试继续通过。主代理自查，无独立第二审查。没有运行后端全集、前端或真机测试。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/control/test_target_strategy.py backend/tests/control/test_transactional_vr_tick.py backend/tests/control/test_candidate_commit.py backend/tests/control/test_robot_control.py backend/tests/control/test_input_await_boundaries.py backend/tests/control/test_gripper_frame_freshness.py backend/tests/robots/test_retarget_backpressure.py backend/tests/control/test_catch_up_supervision.py backend/tests/control/test_braking_supervision.py backend/tests/robots/test_explicit_catch_up.py backend/tests/robots/test_command_admission.py backend/tests/robots/test_command_completion.py backend/tests/robots/test_stop_transaction.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

## 后续边界

事务候选和追赶仍是测试专用，默认控制中的候选历史不一致没有在本轮全面解决。实验仍由测试子类收集 completion、监督追赶预算及处理重捕获，也仍访问 adapter 的私有接受状态；接口抽取不意味着已经可以直接启用正常真机 VR。

下一步建立明确的回执收集、撤销和锚点生命周期接口，替代测试子类对 tick 的包裹，并继续用假 SDK 验证。正常运行参数、连续跟随策略和真机停止及时性仍不能由本轮离线结果确认。
