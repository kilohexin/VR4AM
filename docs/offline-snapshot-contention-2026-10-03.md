# 快照读取与 SDK 锁争用 — 2026-10-03

本轮使用注入的 `FakeLebaiClient`、本地时钟和任务顺序屏障。未连接真机、启动服务、部署或推送。新增读取用途标签，运动、停止、锁与新鲜度策略保持原样。

## 1. 调用路径与方案边界

状态推送 `state_sender → RobotControl.state_message → adapter.get_state` 会读取完整快照，推送后再等待配置周期。运动发送 `_send_target_impl` 也会读取完整快照，然后再次获取同一 SDK 锁进行 IK 和发送。

读取、IK/发送、停止 RPC 使用同一把 `_sdk_lock`。已经排在前面的读取会增加后续运动或停止的等待。这里没有假定 SDK 支持并行调用。

`get_state()` 同时被 UI 推送和控制逻辑使用，不能直接将整个方法改为返回缓存。`preflight`、停止验证及运动发送仍需维持各自现有的完整读取与检查。当前快照来自多次顺序 SDK 调用，也不能当作设备同一时刻的原子采样。

## 2. 可控争用实验

单次快照模型：状态读取 8 ms、运动学读取 8 ms，其余 SDK 读取为 0 ms；IK 8 ms、PVAT await 8 ms。均为推进 fake 时钟的设定值，不是现场延迟拟合。通过任务屏障固定排队顺序，没有依赖真实 sleep 猜测顺序。

| 排在前面的状态读取数 | 运动读取前：快照锁等待 | 运动读取与 IK 之间：命令锁等待 | 两种排序下的总处理时间 |
|---:|---:|---:|---:|
| 0 | 0 ms | 0 ms | 32 ms |
| 1 | 16 ms | 16 ms | 48 ms |
| 3 | 48 ms | 48 ms | 80 ms |

两种排序的总处理时间相同，但对运动快照年龄的影响不同：

- 若状态查询排在运动读取之前，运动读到的快照更晚，发送开始时的本地快照年龄保持 24 ms。
- 若状态查询排在运动读取和 IK 之间，运动使用的快照已经生成，其发送开始时的年龄分别增至 24、40、72 ms。
- 因此不能仅按总处理耗时判断快照新鲜度，也不能把 `snapshot_read` 和其内部 `snapshot_lock_wait` 重复相加。

另 3 个停止案例：在停止请求前排入 0、1、3 次读取，停止 RPC 创建前的锁等待为 0、16、48 ms。创建后的截止仍为 200 ms，fake `stop_move` 立即返回，写入只有一次 `stop_move`，没有 `stop_sys`。这些案例证明前置读取的等待机制，**不能解释现场 609 ms SDK await 的来源**；该 await 是拿到锁之后的另一区段。

## 3. 新快照不能替换旧运动快照的检查

一个边界案例先生成运动快照，再让 3 个状态读取各花 120 ms：

- 运动快照从模型 0 ms 开始，读取于 16 ms 完成。
- 后续状态读取依次在 16、136、256 ms 开始，各自都在现有 300 ms 单次读取期限内完成。
- 共享 `_snapshot` 已被最新状态读取更新，仍在当前 340 ms 本地年龄上限内。
- 运动请求自身持有的旧快照已经超龄，发送被 `robot_state_stale` 阻止，没有 PVAT 写入。

这验证了现有保护：不能因为别的读取更新了全局缓存，就给正在处理的旧运动快照换时间戳或跳过检查。这里的 340 ms 来自测试配置下的既有规则，不是新推荐的时间阈值，也不是设备反馈延迟保证。

## 4. 已实施的小范围改动

在既有时间字段旁增加用途标签：

- `robot_kinematics.snapshot_read_purpose`：`connection`、`state`、`preflight`、`motion`、`gripper`、`home`、`prepare`、`joint_move`。
- 停止验证/观察中的 `read_purpose`：`stop_verification`、`stop_observation`。
- 未显式传入用途的私有读取使用 `unspecified`；旧日志字段缺失表示未知。

标签只说明适配器入口，`state` 并不等于纯显示查询。失败读取若没有形成快照，也不会形成成功快照事件。没有新增 SDK 调用、UI 协议字段或轮询任务；没有改变缓存、锁顺序、停止策略和任何运动限制。

## 5. 优化候选与选择

| 候选 | 潜在收益 | 需要解决的问题 | 本轮决定 |
|---|---|---|---|
| 降低显示轮询频率 | 减少读取占用 | UI 更新变慢；不能减少控制安全读取 | 可作后续离线对照，不改现场参数 |
| 为显示建立独立读取接口，复用已完成快照 | 可能减少与运动的重复读取 | 必须显示本地快照年龄与未知状态；禁止给 arm/preflight/stop/motion 复用陈旧结果；限制并发回退读取 | 优先进入下一轮小范围离线原型 |
| 直接缓存 `get_state()` | 改动表面较小 | 影响共享此方法的控制调用，容易隐藏旧数据 | 不采用 |
| 将快照与 IK/发送长期合并在一个锁区间 | 减少中间插入读取 | 可能延长停止等待；不能中断已执行的 SDK await | 不采用本轮改动 |
| 放开 SDK 并发或延长新鲜度/停止期限 | 可能掩盖等待 | SDK 并发能力未证明，且改变现有保护 | 不采用 |

上述收益是模型中的条件性空间，不是现场性能承诺。下一轮原型必须把显示数据通道与控制检查分开，并覆盖断连、故障锁存、快照缺失/过期和取消；通过前不替换生产默认路径。

## 6. 验证

11 个新增案例：6 个运动争用排序、3 个停止排队、1 个旧运动快照拒绝、1 个用途标签与读取数量核对。最初 10 个机制实验通过；加入字段断言后出现预期 6 failed / 4 passed，缺失键分别为 `snapshot_read_purpose` 和 `read_purpose`；实现标签并补充读取数量案例后 11 passed。

定向覆盖适配器控制/readonly、停止时序/事务/诊断/证据/运动 ID，以及停止观察记录，共 **213 passed**（9.24 s）；未运行后端全集。`git diff --check` 通过。本轮没有前端改动。独立审查任务此前因工具额度不足未完成，本轮由主代理复核。

复现新增实验（仓库根目录）：

```text
.venv/Scripts/python.exe -m pytest backend/tests/robots/test_snapshot_contention.py -q -s -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

`CONTENTION_RESULT` 输出 9 组定量结果；其余两个案例验证过期拒绝和用途字段。测试中的连接、停止和断连只作用于注入的 fake 客户端；没有真实 SDK 连接或网络请求。

定向回归命令：

```text
.venv/Scripts/python.exe -m pytest backend/tests/robots/test_snapshot_contention.py backend/tests/robots/test_stop_timing_boundaries.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_lebai_adapter_readonly.py backend/tests/robots/test_lebai_stop_motion_id.py backend/tests/robots/test_lebai_stop_evidence.py backend/tests/robots/test_lebai_stop_diagnostics.py backend/tests/robots/test_stop_transaction.py backend/tests/commissioning/test_stop_observation.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```
