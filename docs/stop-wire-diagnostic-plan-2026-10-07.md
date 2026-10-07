# 加速真机准入：停止收发诊断

2026-10-07，Asia/Shanghai。接续 [单次早停失败](early-stop-result-2026-10-07.md) 和 [实验策略反馈等待](runtime-pursuit-feedback-wait-2026-10-07.md)。用户要求加速，优先真机 VR；本轮集中处理停止证据缺口，物理抓取随后。**本文不是现场执行授权；本轮没有连接、抓包、动作或推送。**

**2026-10-08 工具进展：** 标准 Wireshark/TShark 4.6.9 安装已验证；离线 RPC 解析器、默认预览的只读捕获入口及子进程配置哈希绑定已实现，150 项定向回归通过，独立静态审查发现已修复并复核。Windows 管理员确认后本机驱动/采集空闲/空筛选器检查完成，已生成审阅基线并通过本地路由/网卡映射确定组件 3，没有网络探测、新现场授权或抓包；详见 [工具实现与验证](readonly-wire-tooling-2026-10-08.md)。下面安装前缺少解析器的描述保留为当时状态。

## 已定位到哪里

- 上一次唯一 stop_move：锁等待 0 ms，SDK await 610 ms，原 200 ms 截止失败、stop_unverified 保留；没有停止后的 PVAT 请求。
- 同一安装 SDK 0.3.13 对本机立即应答伪服务 await 约 0.85 ms；不支持 SDK 总是固定等待约 600 ms 的假设。该对照已完成，无须重复。
- 固定 SDK 包装仅传 Empty 并 await RPC，没有显式固定 sleep。[官方固定源码](https://github.com/lebai-robotics/lebai-sdk.rs/blob/79bc9bba3cc6d710c9c7d3f40f0fe01742911187/src/rpc/motion.rs)。厂家文档未给 stop_move 的 200 ms 响应保证，也未解释响应是否等待减速完成；需要厂家确认。[官方运动接口](https://help.lebai.ltd/sdk/motion.html)。
- 还缺对应请求出网、对应回复入网的时间；客户端收发间隔包含网络与控制器，不能独自分离两者或证明物理停稳。控制器版本及厂家对此问题的实际答复未在当前证据中建立。

## 本轮软件准备

增加 `clock_anchor`：读取 `time.monotonic_ns` → `time.time_ns` → `time.monotonic_ns`，保留墙钟样本两侧的单调时钟界限和采样跨度。仅在 commissioning 会话开始/结束，以及 standalone readonly 报告开始/结束采样；没有插入 SDK 停止热路径、控制 tick 或逐帧采样。缺失/异常返回 null，不阻止日志或原只读结果。

原事件 server_mono_ns、SDK await 起止、200 ms 截止及停止验证均不变。该记录只支持后续时钟对应核对；注入/模拟 clock 不可冒充 OS 时钟。采样跨度不是系统时钟调整、漂移、抓包精度或控制器时间的误差界。开始/结束偏移不一致或现场校时证据不足时，应用与抓包的分段耗时保留未知，不能强行插值。旧日志缺失 anchor，不能回填。

## 下一次现场范围：先只读验证采集

AGENTS.md 要求新连接/现场试验另行明确范围与刷新现场条件。当前尚未批准以下范围，之前的动作额度已消费。

| 项目 | 待批准的具体范围 |
| --- | --- |
| 连接 | 一个独立 standalone readonly 进程，一次连接；连接等待最多名义 5 秒，后续读取窗口最多名义 5 秒；外层进程看门狗 15 秒，无自动重试 |
| 读取 | 固定 get_robot_state / get_estop_reason / get_kin_data / get_tcp / get_claw / get_payload / get_gravity；后续仅原四项轮询，不加入倍率或版本 RPC |
| 工具 | scripts/real_robot_readonly_state.py，独立已核对 readonly 配置，输出为本机忽略目录中的新文件；原配置/旧原件不覆盖 |
| 抓包 | 同机独占 Pktmon，先于连接开始，捕获握手；仅现场机器人端点、已核实的 SDK TCP 端口（默认 3031，执行前核对），最长名义 15 秒；不是全网抓包 |
| 现场条件 | 目前上电、IDLE、目视静止、用户在场观察并能操作急停，没有其它任务/客户端驱动；无需点击启动、Home 或进入动作模式 |
| 结束 | IDLE/急停/字段异常、读取失败或超时即结束；核对采集停止、采集进程与端口收尾；进程结束不能独立证明控制柜连接释放 |

该范围不包含运动、stop_move、stop_sys、Home、prepare、夹爪写入、复位或 VR 测试。异步/看门狗时间不是硬实时保证。若设备关机，不要求用户先上电；先补版本/工具信息即可。

### Pktmon 准备与执行前门禁

本机已找到 Windows Pktmon；本轮只读取 start / etl2pcap / filter add / filter remove 帮助。当前进程 **不是管理员**，PATH 与常见安装目录未找到 Wireshark/tshark；未启动采集、安装软件或更改筛选器。管理员采集与解析工具需在现场准备，不能宣称已具备完整链路。

1. 先离线确认可用的 TCP/WebSocket 重组解析工具（Wireshark/tshark 或经过验证的等价工具）；用合成材料核对解码能力。准备好后才申请上述只读范围，避免采集无法解析的原件。
2. 用户准备管理员终端；核对 Pktmon 没有正在运行，filter list 无既有筛选器。筛选器之间为 OR，残留筛选器会扩大采集范围。存在其它采集/筛选器则暂停，不替用户删除。
3. 新建专用筛选器，条件同时含 TCP、机器人 IP、SDK 端口；命令形式为 `pktmon filter add VR4ArmStopDiag -t TCP -i <现场IP> -p <核实端口>`。参数只在本机生成/核对，不提交地址。
4. 采集使用 `--capture --comp nics --pkt-size 0 --file-name <新忽略目录文件> --file-size 32 --log-mode multi-file`；默认 128 字节可能截断 JSON，默认 all 会在多个组件重复记录。记录选定 NIC 与工具版本，跟踪分片文件；如发生丢包/缺片则不能用于完整性判断。
5. 只读进程结束或外层预算到期，停止本轮独占采集，使用 `pktmon etl2pcap <etl> --out <新pcapng>` 转换全部文件。本机 filter remove 帮助只提供删除全部筛选器；仅在仍独占且只有本轮筛选器时清理，否则留存并告知用户处理。不得停止别人正在使用的捕获。
6. 保存文件 SHA-256、原只读配置哈希前后、进程退出/看门狗状态、完整请求/响应 JSON-RPC ID 对应、时间锚及采集丢失情况。原 ETL/PCAP/配置/输出都留本机忽略目录。

命令形式是审阅材料，含占位符，当前未执行。[微软 Pktmon 筛选说明](https://learn.microsoft.com/en-us/windows-server/administration/windows-commands/pktmon-filter-add)。

只读窗口用于验证捕获、TCP/WebSocket 重组、双向 JSON-RPC 匹配和时钟对应，**不测运动中 stop_move**。SDK await 开始/结束是调用边界，不精确等于首包发送/末包接收；客户端捕获也不是控制器接收时间。若使用加密传输，不能从密文臆造方法或 ID，需另定经审阅的解密/诊断路径。

## 后续停止诊断的判别标准

采集链路核对后，再拟定并单独批准一次运动中停止诊断，附新现场检查和最新冻结源码。不得直接复用上次 +Z 3 mm 的已消费 execute_once 范围。记录完整 JSON-RPC 消息在同一 TCP 流的 ID；本地 stop request_id 不等于 SDK JSON-RPC ID。不能把仅含部分 JSON 的包或 TCP ACK 当成 RPC 回复；重传/乱序要重组、去重，保留首/末字节时间与不确定性。

| 证据 | 下一步指向（不是既定根因） |
| --- | --- |
| SDK await 已进入，而完整请求较晚才出网 | SDK/客户端排队、调度或发送链路；比较同时期只读/运动 RPC，避免猜测 |
| 请求很快出网，完整匹配回复约 600 ms 后入网 | 网络与控制器区间；核对重传/丢包，并取得控制器日志或厂家响应语义，不能直接归因固件 |
| 匹配回复很快入网，SDK await 较晚结束 | SDK 接收、解码、运行时调度或本机负载；按证据做单项修复 |
| 捕获/时钟/ID 不完整 | 不分配延迟，不再重复相同无测量的运动；先修采集 |

修复后仍需要运动中停止的独立验证、连续停止证据与现场观察。RPC 变快不代表物理停止达标。保留 200 ms / stop_unverified；不增加截止，不恢复自动 stop_sys。实际 J5 曾达到约 0.105461 rad/s 而请求 0.05 rad/s，仍是另一项未通过的真机速度问题；只读 100% 倍率和本轮软件测试不能追认通过。两项满足后，依次验证小幅 VR 平移/Grip 松开，再验证 Trigger 开合和实际抓取。

## 可交给厂家确认的问题（尚未发送）

用户提供当前 L Master/控制器版本后补齐：SDK 0.3.13；一次非 VR PVAT 早停中锁等待 0 ms，SDK await 610 ms，客户端原 200 ms 期限失败，之后没有新 PVAT，最后 MOVING 读取窗 422–437 ms、首 IDLE 625–640 ms；回环立即应答约 0.85 ms。请确认：

1. stop_move 的 JSON-RPC 回复表示请求受理、规划停止还是减速完成？此控制器版本是否有运动中响应时限或已知问题？
2. 运动/只读并发 RPC 对停止处理是否共享排队？需要何种控制器侧日志，才能核对 stop_move 接收、处理、回复和实际停止的时间？
3. 200 ms 未确认停止时，厂家建议何种可验证且不禁用关节的受控路径？请提供语义及失败处置；不能直接以 stop_sys 替换，既有独立 stop_sys 曾伴随位移。

不向厂家发送消息或原始日志，除非用户另行授权。未收到回复前不猜固件版本、升级效果或新的保护接口。

## 定向离线验证

新增 12 个测试：时钟夹取/类型/倒序/异常、会话时间字段保持、只读白名单保持、缺失诊断不改变结果。初次红灯为尚不存在 clock_anchor 模块；实现后与已有日志、只读、原停止时序/迟到锁存/诊断测试一起验证，8 文件 **79 passed，11.02 秒，退出 0**；首轮 77 通过后补充两例缺失诊断。没有后端全集、前端构建、独立审查或现场验证。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/recording/test_clock_anchor.py backend/tests/recording/test_commissioning.py backend/tests/scripts/test_readonly_state.py backend/tests/scripts/test_readonly_velocity_factor.py backend/tests/robots/test_stop_transaction.py backend/tests/robots/test_stop_timing_boundaries.py backend/tests/commissioning/test_stop_timeout_lifecycle.py backend/tests/commissioning/test_stop_rpc_diagnostics.py -q --tb=short -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```
