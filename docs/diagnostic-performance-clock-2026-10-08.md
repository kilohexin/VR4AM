# 真机 VR 停止诊断：独立高精度计时

2026-10-08，Asia/Shanghai。接续 [只读收发结果](readonly-wire-result-2026-10-08.md)。目标是补齐下次停止诊断的观测边界，真机 VR 控制优先，物理抓取仿真仍后置。本轮仅离线实现、Fake SDK 验证、本机计时与冻结抓包解析；没有新机器人连接、部署、采集或机器人命令。

## 根因与修正范围

上次只读 79 次调用中 65 次 monotonic await 为 0，本机 Python 3.12.14 的 monotonic 报告实现 GetTickCount64、分辨率 15.625 ms。新增独立 `time.perf_counter_ns` 诊断戳；本机实现 QueryPerformanceCounter、报告分辨率 100 ns。短时计数与 clock info 接口见 [Python 3.12 官方文档](https://docs.python.org/3.12/library/time.html#time.perf_counter_ns)。报告分辨率和整数纳秒单位都不意味着相同的实测准确度。

控制仍用原 monotonic/注入时钟；SDK 超时、连接/读取窗口、原 200 ms、停止锁存及后续动作门禁没有改用诊断时钟。它不能解决或解释原约 610 ms 停止迟到。本轮不是软件停止验收。

| 观测位置 | 新字段 | 保留的旧字段 |
| --- | --- | --- |
| 只读连接/每次读取 | connection_diagnostic_timing / reads[].diagnostic_timing | connect/started/completed/window_deadline_ns |
| stop_move 的 SDK 锁等待 | stop_move_lock.diagnostic_timing | requested/acquired/wait_ns |
| 停止请求创建、await 与生命周期结束 | stop_rpc_lifecycle.diagnostic_timing | started/sdk_await/deadline/outcome/wait_failed |
| 已有只读停止诊断采样 | reads[].diagnostic_timing | 原采样时钟、读超时与周期 |
| 会话开始/结束、只读报告及停止请求 | performance_anchor | clock_anchor / server_mono_ns |

`diagnostic_timing.clock` 明确为 `time.perf_counter_ns`；SDK await 开始/结束字段限于 await 边界，不是首包发送、末包接收、控制器接收或物理停止。旧字段与新字段不可直接相减。锁的 completed 可能在 RPC 返回或取消后，不可误认为 acquired。

`performance_anchor` 含 performance_before/after_ns、wall_time_ns、采样跨度、perf_counter/time 的 implementation/resolution 等信息；`alignment_verified=false`、assessment=diagnostic_only。两侧夹取只描述采样边界，不界定 wall 量化/调时、驱动包时间戳和时钟漂移。没有因高精度计时自动宣布时钟已对齐。失败、非法类型/负数或倒序锚点留 None，不输出敏感异常文字、不影响控制与收尾。

## WebSocket 控制帧

FIN 完整的 Ping/Pong/Close（opcode 9/10/8）单列 `websocket_control_frames`，记录帧号、stream、opcode、长度与解码完成时间，不计入 JSON-RPC。控制帧分片、长度 >125、Close 单字节载荷仍显式记录 invalid_websocket_control；其它不支持的数据片段保留原提示。帧结构限制参考 [RFC 6455 §5.5](https://www.rfc-editor.org/rfc/rfc6455#section-5.5)。该分类不等于验证关闭状态码、UTF-8 或全部协议语义。

以旧冻结 `ipv4-interpretation.pcap` 离线重放新分析器：79 对完整 RPC 的所有配对字段逐一与原报告相同，158 条 JSON-RPC；4 个控制帧单列，新的 issues 为空。旧报告中的四个 fragment 标签和原件保留。派生报告 `wire-analysis-controls-v2.json` SHA-256 `d955fadaa0739cb1a1c1ee8a1092460a0b58c1a1a8e43be744eb429e4a8d7819`，仅本机忽略目录，不提交原包或载荷。

抓包的无线头解释仍采用上次逐帧核对的离线副本，不将本次控制帧修正称为通用 WLAN 解码器。下次实际停止抓包的封装仍须明确处理同样的驱动格式；没有直接改成任意机器人命令采集入口。

## 验证证据

新增 19 个定向用例：独立锚点/非法值/时钟异常、只读白名单及结果保持、诊断时钟超长不触发停止截止/控制时钟迟到仍超时、SDK 错误原样保留、采样与会话时钟不混用、SDK 锁等待、完整/非法控制帧。先得到 18 红/35 通过，分别为字段/功能缺失；实现后 53 通过。锁等待用例随后单独得到 1 红/14 通过，补齐后与相关 9 文件 **171 passed，15.94 秒，退出 0**。

首次 pytest 另报旧 backend/.pytest_cache 写入拒绝的两个警告；将缓存改到本机忽略目录后没有该警告，未删除旧目录。独立静态审查没有发现本轮可操作 P0–P2 问题；只读审查，没有运行测试或现场动作。

最终扩大到受影响的记录器、停止证据、运行运动 ID 与关节调用限速回归，共 **13 文件 230 passed，35.16 秒，退出 0**；仍为定向验证，没有跑后端全集。限速回归只证明 Fake SDK 调用与软件行为，不能验收真实 J5 速度。

本机 50 次约 2 ms sleep 对照：新计时全部得到非零时长，旧计时有 41 次为 0；新计时观测范围 2.2298–5.2943 ms，中位 2.9769 ms。操作系统调度可能延长 sleep；这是本机短时观察，不是 SDK 性能、真实动作或准确度标定。结果 `.tmp/performance-clock-local-20261008.json` SHA-256 `53d301b65fe064d8237127da9fd0e8f37f6056dffa5451eed5150372e224a24d`。

重新核对上一轮冻结清单 23 个文件的完整哈希/大小，均未变化。PVAT 构造与 RobotControl 源码 SHA-256 分别仍为 `cb6a1a4060c1728163d97baa08e745efe680440a820c14cf5a11212d324a0307` / `1e6f26eff7ca2e59e6341bd27a8f890c335e4f32b1f90da2d3d1d08b2a19fe43`。原 stop transaction 源码因增加诊断字段而改变，不将其称为哈希保持。

## 下一步与现场边界

当前没有新的现场授权额度。上次一次只读已经结束；旧冻结批准文件绑定运行源码，新增诊断使其失效，且原输出目录已使用，不能复用。没有创建新的 live launcher、启动有效期或待确认 UAC。

继续离线准备能记录一次受限运动及停止收发的入口：固定动作/限幅/禁止自动准备位姿、明确批准范围、捕获归属收尾、输出唯一绑定、QPC 与包时钟的不确定性报告。准备位姿 J5 峰值 0.105461 rad/s 对比请求 0.05 的原因和实际限速仍未验收，不直接复用旧准备移动。残留的本轮 inactive Pktmon 筛选器仍需单独复核归属后清理。

入口与动作预检材料完成后，再向用户请求新的具体现场范围。实际收发证据才能决定优先查客户端排队/本机解码或网络与控制器区间；厂家版本与响应语义依然需要证据。软件停止/真实限速达到验收条件后，依次验证小幅 VR 平移与 Grip 松开、Trigger 开合。没有调整 200 ms 或放行 stop_unverified，也没有恢复自动 stop_sys/复位/重试。暂不推送。
