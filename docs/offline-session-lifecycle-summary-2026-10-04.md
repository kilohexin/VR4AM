# 会话与停止日志离线汇总（2026-10-04）

## 交付

新增只读工具 `scripts/analyze_session_lifecycle.py`，仅使用 Python 标准库，无 SDK 导入、网络访问或机器人命令。读取一个 commissioning JSONL 文件，输出 JSON 摘要及源文件 SHA-256；不写入源文件，不输出原会话 ID、位置数据、异常正文或完整事件载荷。没有改动运行时行为。

```powershell
.venv/Scripts/python.exe scripts/analyze_session_lifecycle.py <session.jsonl> --expected-sha256 <sha256>
```

可显式提供该记录对应的 summary 文件：

```powershell
.venv/Scripts/python.exe scripts/analyze_session_lifecycle.py <session.jsonl> --recording-summary <summary.json>
```

输出分别保留 JSONL 和 summary 的哈希。工具不自动寻找 summary，也不证明用户指定的两个文件必属同一记录；调用者需核对来源。没有 summary 时 `normal_log_drop_count` 为 null，不视作零丢失。

## 汇总和关联规则

1. 按 `session_trace_id` 合并累计生命周期字段，但输出仅使用报告内会话序号。完全重复的停止/RPC 行去重；不同累计行对同一边界给出不同值时，将该边界设为 null，不取最后一行覆盖。
2. 字段缺失、未来边界、时间倒序保留为缺失或问题项。反向耗时不钳制成零；非法时间类型、负时间、损坏 JSON、非有限 JSON 数值拒绝生成报告。
3. 断连起止和 owner 区间完整且自洽、其他已观察会话不重叠且不开放时，才评估停止关联。只有一个相交停止诊断，且原因是 disconnect、其完整区间位于断连处理内，才标记 `unique_observed_candidate`。
4. 多个相交停止、其他原因停止、未完成停止或不完整会话覆盖都不能强行归属。不会按“距离最近”或记录行顺序选取。
5. RPC 通过停止调用记录中的 episode、request、request_started_ns 核对，并校验停止事务 ID、锁获取及 RPC 时间范围。缺失/冲突标识、多条不同 RPC 记录或时间矛盾均保持关联失败。
6. 复用旧请求标记为 `reused_request`，不把此前的 RPC 创建和 await 计入新会话。锁等待若有独立且一致的字段，仍可单独计算。

时间关联缺少会话外键，因此即使候选唯一，输出仍标注 `association_is_inference=true`。缺失日志可能隐藏另一个候选；唯一已观察候选不等于确定归因。工具只针对单个源文件工作，不提供跨进程、跨记录的时钟拼接保证。

## 输出的耗时

| 字段（ms） | 范围 |
| --- | --- |
| `cancel_to_join` | 发起同组取消 → gather 收尾完成 |
| `post_join_gap` | gather 完成 → 调用 on_disconnect；不能全部命名为排演耗时 |
| `disconnect_total` | on_disconnect 调用起止 |
| `cleanup_to_stop_entry` | on_disconnect 开始 → 匹配停止诊断开始 |
| `stop_lock_wait` | 停止 SDK 锁请求 → 获取，另核对已有 wait_ns |
| `cancel_to_rpc_created` | 发起同组取消 → 本次新停止 RPC 创建 |
| `rpc_deadline_window` | RPC 创建 → 记录的截止 |
| `sdk_await` | SDK await 起止 |
| `sdk_return_after_deadline` | SDK 返回晚于截止的时长，及时返回为 0；不是停止及时性的唯一标准 |

这些范围可能互相包含，**不能将整张表的数值相加**。`disconnect_total` 还可能包含停止验证和既有记录工作。任务完成回调不是精确协程退出时刻，局部读取时间也不证明设备源采样新鲜度。

## 验证

新增 31 个案例，包含完整/部分/旧日志、累计字段冲突、时间类型和倒序、多候选停止、开放或重叠会话、旧请求复用、身份冲突、锁时间矛盾、609 ms SDK await 与 409 ms 晚于截止、丢弃证据、CLI 源哈希及只读性。

首轮先确认缺少新模块的预期导入失败，实现后 24 个通过。之后使用真实 `SessionTrace`、`DiagnosticsRecorder`、`CommissioningRecorder`、会话清理和注入 FakeLebaiClient 生成临时 JSONL，验证工具能识别真实输出格式，且 summary 的零丢弃来自实际生成的文件；临时 fake 日志不入 Git。

另对原冻结运动日志进行了只读兼容性检查：

- 来源：`C:\WorkSpace\deliverables\2026-09-29\artifacts\r29v-motion-final-20260929T163350\session.jsonl`
- SHA-256：`d48d1768cc5ce74157b4efafec3b90d8e86e7cda51b35d3e554d6ac187caff5b`，与预期一致。
- 已观察停止诊断 2 条、RPC 2 条；生命周期会话 0 个，未关联停止 2 条。
- 未提供对应 summary，丢弃数量保持未知。旧日志没有新增字段，工具没有推断或补造取消/join/清理阶段耗时。

相关定向回归 **98 passed**（2.78 s）。定向验证命令：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/scripts/test_analyze_session_lifecycle.py backend/tests/scripts/test_analyze_vr_cadence.py backend/tests/scripts/test_stop_timeline_audit.py backend/tests/diagnostics/test_session_trace.py backend/tests/api/test_session_diagnostics.py backend/tests/api/test_observer_cancel_timing.py backend/tests/api/test_observer_session_lifecycle.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

未跑后端全集；未启动服务、连接真机、修改现场配置或原始日志。主代理自查，无独立第二审阅者。本轮只做本地提交，暂不推送。

## 后续

汇总工具、生命周期诊断及相关离线验证已形成可复核链条。下一步整理本地尚未推送的改动和验收证据，复核运行时改动之间的兼容性及未解决事项。平移与现场停止问题仍需区分已实现诊断、模型验证与真实设备证据；不将本轮结果作为启用独立观察、调整增益或真机动作授权。
