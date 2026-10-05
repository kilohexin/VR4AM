# 显式追赶已接受目标：接口与离线监督实验

日期：2026-10-05。仅本地开发、注入假 SDK；未连接设备、未部署、未推送。

## 本次实现

- `RealLebaiAdapter.submit_catch_up(accepted_command, command_id)` 提交一次追赶请求。复用原提交硬门禁、PVAT 构造、速度/加速度限制、运行状态与快照检查；不调用 IK，不生成候选解，不改变已接受目标的 TCP、关节解或请求引用。
- `PvatRequest.catch_up_from` 固定被追赶的接受引用。入队时检查引用与 pump 代次，取得 SDK 锁后再次检查。若前一个在途候选已经改变接受目标，追赶请求以 `invalidated / catch_up_target_changed / not_started` 结束，不发 PVAT，不把它改成追赶另一个目标，也不为此锁存 pump 故障。
- 追赶成功仍返回独立 completion，`pvat_mode=catch_up`，`accepted_command` 指向原接受请求。`pvat_sent.request_kind` 区分显式追赶与普通目标请求。普通请求触发的旧背压追赶仍标 `request_kind=target, pvat_mode=catch_up`。
- 已接受关节解若也超过原跟踪误差上限，显式追赶失败并进入既有 pump 故障路径，不扩大限值或重新选解。请求供给本身不会自动重试。
- 两种提交方式共用同步的 `_check_submission_ready`，检查与入队之间没有新增 await。取得 SDK 锁后也跳过已撤销代次，避免继续无效的候选选择。

默认 `RobotControl.command_tcp` 路径没有切换到该接口。新接口与 `submit_tcp` 一样要求调用者负责输入新鲜度、权限和停止监督；不能直接把它当作可独立运行的 VR 控制循环。

## 测试专用追赶监督

`backend/tests/control/catch_up_probe.py` 不被生产代码导入。实验在候选收到背压后冻结候选历史，固定接受引用，最多保留一个未完成追赶请求。

| 情况 | 实验行为 |
|---|---|
| SDK 返回或机器人状态为 IDLE，但反馈未接近固定关节解 | 保持追赶；不提交候选历史 |
| completion 被覆盖、引用不符或请求失败 | 终止追赶供给；不恢复历史 |
| Grip 释放、输入满 100 ms、控制/pump 代次撤销或故障 | 禁止继续提交；原 owner 执行停止流程 |
| 500 ms 实验预算耗尽 | `budget_expired`；owner 显式停止 |
| 连续三个不同时间戳、间隔不超过 40 ms 的新鲜反馈，在 1 mrad 位置及 2 mrad/s 速度带内 | `observed_settled`；结束供给，仍不恢复候选 |
| owner 完成停止并重新激活、捕获锚点 | 候选显式 reanchor，从零历史开始；旧追赶会话仍被撤销 |

500/40 ms、位置/速度带和样本数是测试输入，不是建议的真机参数。重复、过期、非有限数、仍运动或偏离的反馈都会打断观察计数。测试向假 SDK 注入反馈，**没有建立机器人动力学或关节到 TCP 的物理一致性模型**；观察退出用例不代表真机能在 500 ms 内追上。`observed_settled` 也不替代原停止事务或证明物理停止。

## 验证证据

新增 27 例：接口 9 例，监督 18 例。先观察缺少接口/模块的失败，再实现；扩展重捕获案例时修正了夹具把候选权限错误绑定到旧 epoch 的问题。

- 原接受引用、TCP、解保持不变，追赶完全不运行 IK；错误/缺失引用、停止代次、陈旧快照拒绝入队。
- 在途候选改变接受目标时，已排队追赶撤销且不发送；超过跟踪误差上限不发送。
- 保留非零候选历史，追赶 completion 和 IDLE 不解冻；三个合格新反馈可结束观察，但候选仍须原 owner 停止和重捕获。
- 在等待运动快照、PVAT RPC 已发出两阶段，分别覆盖释放 Grip、100 ms 过期、故障、停止异常和 609 ms 迟到。停止在解除阻塞前撤销 pending；在途返回保留 `not_started` 或 `returned` 的真实软件结果。已发 RPC 不能撤回，不承诺它被瞬间取消。停止之后没有额外 PVAT。
- 停止失败保留 `stop_unverified`，复位拒绝；没有自动 `stop_sys` 或 Home，未改变夹爪策略。所有连接均为注入假 factory，网络与发现调用为零。

定向回归命令（仓库根目录，PowerShell）：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/control/test_catch_up_supervision.py backend/tests/robots/test_explicit_catch_up.py backend/tests/control/test_candidate_commit.py backend/tests/robots/test_command_admission.py backend/tests/robots/test_command_completion.py backend/tests/robots/test_lebai_pump.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_lebai_backpressure_experiments.py backend/tests/robots/test_stop_transaction.py backend/tests/control/test_robot_control.py backend/tests/robots/test_retarget_backpressure.py backend/tests/control/test_braking_supervision.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

结果：405 passed。未运行后端全集。由主代理自查，无独立第二审查。

## 后续接入边界

下一步把候选事务和显式追赶组成一个测试专用调度策略，接到完整的 VR 映射/滤波/tick 链路；覆盖追赶期间新手柄目标、退出后的停止/重捕获，以及夹爪等待中的输入撤销。现有生产路径中的候选历史不一致仍由原特征测试复现，本轮不宣称已经修复默认 VR 策略。任何自动延续速度的设计都仍需明确连续性依据。
