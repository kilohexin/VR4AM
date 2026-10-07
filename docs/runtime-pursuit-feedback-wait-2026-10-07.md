# 实验运行策略：发送后等待新反馈，再决定追赶或停止

2026-10-07，起点 `84b28117c8f89133a2540c3edfabacddcbd9cbca`。本轮修改生产 `CatchUpSupervisor` 和 `TransactionalTargetStrategy`，接入现有、必须显式选择的 `transactional_experimental` 策略。没有启用该现场策略、修改配置或默认 legacy 路径，没有真机连接、命令、服务启动、部署或推送。现场约 610 ms 的停止迟到未解决，原 200 ms 停止规则与 `stop_unverified` 保持。

## 已复现的问题

原监督器仅检查 `dispatched`、当前没有 pending 和样本晚于上一观察，没有为最近一次返回建立新的读取边界。注入的命令完成测试中：读取在返回结果被收集前开始，结果收集后再传入这帧，会把接近目标的有效计数从 0 增至 1。

原运行策略在 SDK 返回后也可以继续准备、补发 catch-up，等待反馈只是一个隐含状态。接近目标时仍可能每帧补发，而缺失／坏反馈只靠总追赶预算结束。现在明确区分这三个阶段：

| 阶段 | 允许动作 | 进入下一阶段的条件 |
| --- | --- | --- |
| `dispatchable` | 最多提交一个 catch-up 请求 | 提交后进入 `inflight` |
| `inflight` | 等待已有 receipt；禁止第二个请求 | 合法 `sdk_returned` 被本地收集后进入等待 |
| `awaiting_feedback` | 仅消费已有缓存；禁止补发 | 新有效反馈仍偏离目标则可发送；接近目标则继续累计；过期则请求原停止 |

退出状态包括原 `observed_settled`／`budget_expired`／失败／撤销，以及新增 `feedback_expired`。`observed_settled` 仍只是观测状态，运行 owner 随后必须调用原 disarm／停止流程，不直接恢复候选提交或宣布物理到位。

## 本地读取边界与门禁

收集合法返回结果时记录 `feedback_after_ns`。只有**读取开始严格晚于这个本地收集时刻**的观察能参与最近请求的反馈判断。重复收集同一个完成结果不更新边界；不把该时间称为 SDK 内部返回时间、控制器执行起点或传感器采样时间。读取早于结果收集但完成较晚也不通过。

反馈必须具备合法的非负整数捕获／完成时间、捕获 ≤ 完成 ≤ 当前时刻、state/motion 用途、有限六维 q/qd、当前代次及原参数要求的新鲜度／间隔。生产消费的是公开 `JointObservation`；原直接注入 `LebaiSnapshot` 的实验仍可使用外部泵代次检查，不需要改成私有适配器访问。

接近目标时暂停 PVAT 补发，沿用已有 q/qd 容差和 `settle_samples` 计数；没有新增加速度到位证明。坏向量、旧帧、重复缓存、其它读取用途不能释放发送或延长等待。新 catch-up 提交清空计数，避免把上一次动作前的观察累计到新请求。

收集返回后的等待截止为 `min(原总预算截止, 本地收集时刻 + 原 max_observation_gap)`；新接近目标帧的截止以它原始捕获时间加原 gap 计算。远离目标帧释放发送后，发送许可也受原 age／gap 的较小值约束；recording／夹爪 await 消耗这段时间，不能在恢复时用过龄帧发旧计划。恰好等于 gap 边界的有效帧可被处理，**超过**边界进入 `feedback_expired`；这不是对原 200 ms 停止截止的修改。

等待没有额外任务、SDK 查询、重试、保持窗口或新总预算。运行 collect 还会在等待期间检查完整 accepted 命令、连接引用、预检、停止中、泵状态与后端锁存；`stop_unverified` 不被改写成普通追赶失败。`prepare` 和 `dispatch` 均复查是否允许发送，超时／撤销发生于 await 中时不能继续派发。

## 与有限参考的边界

[上轮有限末段](offline-pvat-time-alignment-2026-10-07.md)仍到原截止就过期，本轮没有把它接入生产，也没有延长覆盖或实现末端 hold。这次实现的是**现有实验追赶请求的反馈等待接口**，原预算保留；不能把其间隔期限当成 PVAT 的控制器执行覆盖。

原生产 PVAT 构造器、控制 owner 停止函数、SDK stop transaction、限幅、现场 profile 和配置都未改。厂家插补、执行／采样时间关系和真机停止迟到仍需独立证据。等待门禁可能减少补发或更早退出追赶，不等于真机平移幅度、控制器连续性或物理安全已经验收。

## 定向验证及失败记录

先写 16 项测试，在原实现上全部失败；其中旧读帧被计数直接复现原问题。首次实现后，四组 8 failed／62 passed：旧测试仍预期可在未收到新反馈时补发、50 ms 无反馈继续追赶、坏反馈超过 gap 仍不停止，以及 Trigger 100 ms 间隔不补反馈仍可派发。逐项核对后，按新的等待／退出契约更新测试，Trigger 案例在 debounce 内保持原 20 ms 节拍与有效反馈，而不放宽时间要求。

随后 7 failed／135 passed：一个过期状态保留旧有效计数的问题通过在终止状态清零修复；其余是旧测试的“不得产生退出”断言和自定义 age/gap 超时结果，更新为 `feedback_expired`。停止矩阵新增后 3 failed／158 passed 均在显式 disarm 分支：原 API 本就抛出 `BackendCommandError`，测试错误地当作 tick 内部捕获；改为断言原三种异常，未修改生产停止逻辑。之后首轮九文件 208 通过，补无新 VR 帧、await 和后台门禁后完成最终验证。

新增等待相关 44 项和一个原 VR tick 的 feedback-expired 退出案例，共 **45 个新增案例**。其中等待 gap／Grip 释放／显式 disarm × 确认／仍运动／异常／fake 610 ms 迟到／取消，共 15 项，实际通过原 owner／adapter 停止路径：

- 不继续补发 PVAT，不清除候选历史；停止后需要显式 Arm／新 anchor。
- 等待过程中没有新 VR 帧也会检查 gap；夹爪 await 恢复后过期，在同 tick 请求停止。
- 不完整／迟到／错误／取消保留原 `stop_incomplete`／`stop_unverified`，随后 fake 静止、新帧、reset／Arm 不解除失败锁存。
- 没有自动 `stop_sys`、Home、movej、第二次停止或复位写入。fake 迟到 lifecycle 仍是 200 ms 截止、610 ms await。
- 所有适配器均注入客户端，network/discovery 计数为 0；ASGI 测试是进程内 app／WebSocket／原控制循环，未启动网络服务。

11 个相关文件 **386 passed，20.27 s**，退出 0：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/control/test_pursuit_feedback_wait.py backend/tests/control/test_catch_up_supervision.py backend/tests/control/test_transactional_motion_view.py backend/tests/control/test_transactional_vr_tick.py backend/tests/control/test_transactional_runtime.py backend/tests/api/test_transactional_session.py backend/tests/robots/test_explicit_catch_up.py backend/tests/robots/test_stop_transaction.py backend/tests/robots/test_motion_view.py backend/tests/control/test_pvat_alignment.py backend/tests/control/test_pvat_window_supervision.py -q --tb=short -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

没有后端全集、独立审查、前端构建或真机测试。

## 本机证据与下一步

忽略目录 `.tmp/early-stop-window-20261007-l0erdkr4/` 的本地脚本及 JSON 不入 Git。脚本调用生产监督器与合成 completion／观察；只提交两次合成请求，三帧接近目标期间不再提交；40 ms + 1 ns 无反馈状态为 `feedback_expired`，原总预算 500 ms 未变。不是冻结机器人日志的物理回放。

原日志、只读配置、生产 PVAT 构造器和上轮分析输出固定哈希核对通过；保存字节哈希如下。

| 项目 | SHA-256 |
| --- | --- |
| `evaluate_runtime_feedback_wait.py` | `7ba2d0c8073fed93c6514ce975eec98e6d2f71c5919c8ef41915bfdc560ecdaa` |
| `runtime-feedback-wait-evaluation.json` | `12e3f5729018229aa1fb94167c806f92a4c0ae388b3d9007e729ac3aa8186122` |
| 生产追赶监督器 | `282e78d45f3344b906689dc699046ebf64081bd9b1fa08d3d769392d444e838b` |
| 生产实验运行策略 | `aa83aca34eeff67dde784b2dd47e3da696cd75a53c906b1d164ab2e5144bc108` |
| 新等待测试 | `aae982d33af35453e0894fa4bf3f29e2a77c3d5b50eaa7181cd5843b42dc24fe` |

下一步离线评估新等待节拍对追赶位移／背压／停发时机的影响，并核对实验策略何时能与有限参考的执行证据接口衔接。暂不启用现场策略或连接真机；约 610 ms 的停止迟到仍是现场测试阻塞项。
