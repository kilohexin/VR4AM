# 停止时间边界与反馈新鲜度 — 2026-10-03

本次只使用源码、冻结文件和注入的 fake 客户端。机械臂保持用户已声明的断电状态；没有连接、启动服务、部署或推送。停止策略、200 ms RPC 截止及 `stop_unverified` 均保持原样。

## 1. 更正前次报告中的字段误读

我在前次离线分析中把状态消息的 `sample_age_ms` 当作机器人采样年龄，因而将末尾约 0.9558 mm 的 TCP 位移读数称为“陈旧机器人样本”。该判断不成立，现已撤回并更正原报告。

核对当前源码和现场历史部署源码，`RobotControl.state_message()` 都按以下方式生成此字段：

```text
sample_age_ms = (当前服务端时间 - 最近收到的 VR 帧时间) / 1e6
```

因此它表示 VR 输入帧龄。底层 LEBAI 的 `get_state()` 则会进行一轮 SDK 读取；不能由 VR 帧龄判断这轮机器人读取的新鲜程度。旧字段保持兼容，HUD 标签改为“VR 输入帧龄”，诊断面板改为“VR input age”。

对冻结运动日志 `r29v-motion-final-20260929T163350/session.jsonl` 复核：

| 项目 | 结果 |
|---|---:|
| 最后一条状态报告时间 | `1913823250000000` ns |
| 该报告 `sample_age_ms` | 129375 ms，表示 VR 输入帧龄 |
| 前序同值 `robot_kinematics` 的本地读取起点 | 早于状态报告 32 ms |
| 上述运动学记录与状态消息的 TCP 差 | 0 mm |
| 上述运动学记录的 SDK 等待之和 | 32 ms：状态读取 16 ms、运动学读取 16 ms |

0.9558 mm 的位移计算仍成立；但之前通过 `sample_age_ms < 100` 选取“新鲜机器人反馈”的方法无效，相关 0.9572 mm 行已从主结论表撤回。原始日志未改写。

历史日志没有逐次读取关联 ID，以上采用前序同值事件核对；不据此假定设备源采样时间或 SDK 内部缓存状态。这里的 32 ms 是本地读取边界，不是设备反馈延迟的测量值。

## 2. 可以区分的停止时间

| 时间区段 | 现有观测 | 边界 |
|---|---|---|
| Grip 释放到控制层停止请求 | VR 帧和 `stop_requested` | 需要按同一会话与时间窗口关联 |
| 公共 stop 调用等待事务串行锁 | 当前没有独立开始/结束字段 | 不能假定为零 |
| SDK 锁等待 | `stop_move_lock.requested_ns/acquired_ns/wait_ns` | 本地锁争用，发生在 RPC 请求创建之前 |
| 请求创建到 SDK await 开始 | `started_ns` 至 `sdk_await_started_ns` | 本地任务调度时间 |
| SDK await 本身 | `sdk_await_started_ns` 至 `sdk_await_completed_ns` | 含 SDK 内部、可能的网络/控制器等待；无法继续拆分 |
| 停止状态验证 | `stop_diagnostics.samples` 的读取及观察时间 | 按既有状态、速度、运动 ID、漂移和稳定窗口验证；不是独立物理测量 |

**200 ms 截止从 RPC 请求创建开始**，不是从 Grip 释放或公共 `stop()` 调用开始。SDK 锁等待不在此 200 ms 内。这解释了为什么“RPC 在期限内返回”不能代表整个停止调用耗时小于 200 ms。

新增 fake 对照（模拟时间，不是真实 SDK 性能）：

| SDK 锁等待 | SDK await | 结果 |
|---:|---:|---|
| 15 ms | 16 ms | RPC 返回在截止内；随后走既有停止验证 |
| 31 ms | 609 ms | RPC 晚于截止，`returned_late`，保留 `stop_unverified`；后续 IDLE 和 preflight 不解除锁存 |
| 250 ms | 16 ms | RPC 本身仍在截止内；说明截止不覆盖前置锁等待 |

每组 fake 写入只有一次 `stop_move`，没有自动 `stop_sys`。此前已有单独测试覆盖请求创建后 7 ms 才进入 SDK await 的调度延迟；此次保持该回归。

冻结运动日志的约 609 ms await 仍只能归入 SDK await 区段。没有证据将它具体归因于 SDK 内部、网络或控制器；本轮没有修改超时来掩盖它。

## 3. 新增本地读取边界

`robot_kinematics` 新增：

- `snapshot_read_requested_ns`：等待 SDK 锁之前。
- `snapshot_read_started_ns`：获得 SDK 锁之后，开始顺序读取之前；与既有事件 `server_mono_ns` 对应。
- `snapshot_read_completed_ns`：最后一项 SDK 读取完成并退出锁之后，最终构建快照及日志写入之前。

停止验证样本和停止观察快照也带有对应的 `read_requested_ns/read_completed_ns`，沿用 `captured_ns` 表示读取起点。只增加记录字段，没有增加 SDK 调用，也没有替换现有新鲜度或停止判据使用的时间。

fake 将“锁等待 7 ms、SDK 读取 23 ms、记录回调 11 ms”分开，验证日志的读取耗时是 23 ms，而不是把 41 ms 都算作读取。另一个用例验证：VR 输入已过去 1023 ms 时，仍可能刚完成一轮 23 ms 的机器人本地读取。

## 4. 仍不能证明设备源反馈新鲜

两个 fake 读取返回完全相同的位置和关节数据，本地时间戳相隔 100 ms。仅凭这些字段，无法区分“设备确实没动”与“SDK 重复返回旧数据”。顺序读取也不是同一设备时刻的原子快照。

目前没有在这些证据中建立设备源采样时间/序列号与本地时钟之间的关系。新字段可区分本地锁等待、读取和日志开销，但不能据此宣称设备实时反馈或真机停止已验收。旧日志缺少新字段时按未知处理，不能补零。

## 5. 定向验证

- 新增 7 个时间边界案例。最初 6 项中，3 项通过、3 项因缺少新时间字段失败；实现后补充停止验证/观察字段的一个案例，全部通过。
- 停止事务、停止诊断/证据、适配器控制、控制层和停止超时生命周期：295 passed。
- 适配器 readonly、停止运动 ID、停止观察记录：62 passed。两组无测试文件重叠，本轮后端共 357 passed，未运行后端全集。
- HUD 与诊断面板：44 passed；TypeScript 检查与 Vite 构建通过。JS 主包约 743.27 kB，仍有超过 500 kB 的体积提示。
- `git diff --check` 通过。前端本地验证因已有沙箱目录限制在获准的沙箱外运行；未启动服务器。

后端命令（backend 目录，使用隔离环境 Python）：

```text
python -m pytest tests/robots/test_stop_timing_boundaries.py tests/robots/test_stop_transaction.py tests/robots/test_lebai_stop_diagnostics.py tests/robots/test_lebai_stop_evidence.py tests/robots/test_lebai_adapter_control.py tests/control/test_robot_control.py tests/commissioning/test_stop_timeout_lifecycle.py -q -o asyncio_default_fixture_loop_scope=function
python -m pytest tests/robots/test_lebai_adapter_readonly.py tests/robots/test_lebai_stop_motion_id.py tests/commissioning/test_stop_observation.py -q -o asyncio_default_fixture_loop_scope=function
```

前端命令（web 目录）：

```text
node node_modules/vitest/vitest.mjs run tests/hud.test.ts tests/diagnosticsPanel.test.ts
node node_modules/typescript/bin/tsc --noEmit
node node_modules/vite/bin/vite.js build
```

## 6. 后续范围

现有证据足以解释字段和本地等待边界，仍不足以解决 609 ms 延迟的来源。进一步归因需要 SDK 内部或设备侧可关联的请求/响应时间证据，是否具备这些接口尚待确认。任何新真机连接、数据采集或停止动作测试均等待用户另行明确授权。

下一项可继续离线比较 VR 参数候选下的请求速度和背压敏感性，并明确合成结果的适用范围；不据此批准现场增益、速度或运动。
