# 真机 VR 主线：离线抓包解释完成与下一次停止诊断范围

2026-10-08，Asia/Shanghai。接续 [单次运动停止收发入口](motion-stop-wire-entry-2026-10-08.md)。本轮只处理已结束的本机证据和合成数据，没有连接机器人、启动捕获或发送机器人命令；暂不推送。

## 1. 本轮解决的问题

上次 Windows 捕获的 PCAPNG 声明为 Ethernet，实际帧包含无线数据头与 LLC/SNAP。直接按声明解释会丢失 RPC。本轮将此前针对冻结文件的人工提取流程整理成 `scripts/normalize_rpc_capture.py`，供新采集结束后离线检查，避免每次临时编写提取器。

工具只读取文件，默认按声明解释。它输出独立 IPv4 PCAP 和逐帧 manifest，不改原始文件。输出目录必须全新；原始文件变化时拒绝完成。输入上限为 64 MiB、100000 帧。工具没有 SDK、网络、捕获或机器人调用。

支持明确的 Ethernet/IPv4、IEEE 802.11 Data/QoS + LLC/SNAP/IPv4、直接 IPv4/TCP 3031。截断、分片、未知封装、非零 FCS/包标志、未知尾部、无法精确表示的时间戳等均拒绝；不是通用无线解密或聚合包解析器。

PCAPNG 时间按每个 section 的字节序、各 interface 的分辨率与偏移换算，微秒来源写入纳秒容器仍保留来源分辨率，不能称为新增纳秒精度。依据 [PCAPNG 草案第 06 版](https://www.ietf.org/archive/id/draft-ietf-opsawg-pcapng-06.html)；该文件仍是草案。链路类型编号依据 [IANA PCAP 注册表](https://www.iana.org/assignments/pcap)。

## 2. 格式确认与真机授权是两件独立事项

显式使用 `pktmon_wlan_llc`，或者帧带 Protected/A-MSDU 标志时，检查结果始终要求逐文件确认。即使看起来是明文 IPv4，也不会自动批准转换。检查本身不生成已批准配置。

确认文件必须精确绑定完整源文件 SHA-256、解释布局、端口、完整逐帧映射与 `reviewed_plaintext_ipv4=true`。逐帧映射记录完整包哈希、提取 IP 哈希、偏移、长度、标志、时间字段和被排除的填充。MAC 头变化但 IP 未变也不能沿用确认；JSON 的 true 与 1 也不能互换。

格式确认文件只描述这个已结束文件如何解释，不能批准机器人连接或运动。旧确认不能用于未来新抓包。只有已核对字节与时间证据后才创建确认，不能将 inspection 自动复制为“已审”。

以下命令在仓库根目录执行，只处理本机离线文件。输入与输出名称为示例，输出目录必须尚不存在：

```powershell
& .venv/Scripts/python.exe scripts/normalize_rpc_capture.py .tmp/closed-capture.pcapng --layout pktmon_wlan_llc --inspect --output-directory .tmp/new-inspection
```

人工或独立离线证据核对后，使用只对应这个文件的确认：

```powershell
& .venv/Scripts/python.exe scripts/normalize_rpc_capture.py .tmp/closed-capture.pcapng --layout pktmon_wlan_llc --review .tmp/reviewed-format.json --output-directory .tmp/new-normalized
& .venv/Scripts/python.exe scripts/analyze_rpc_capture.py .tmp/new-normalized/ipv4.pcap --output .tmp/new-rpc-analysis.json
```

`normalization_complete` 仅表示支持的转换流程完成，不代表原始捕获无丢包、时钟准确或机器人已停止。输出始终保留 `hardware_stop_verified=false`、`app_clock_alignment_verified=false`。

## 3. 冻结证据重放

输入 `.tmp/readonly-wire-next-20261008/merged-readonly.pcapng`：SHA-256 `69ee7de1352de6f64d557bed4d5ad8095ecfc4ff4e5b76d918318ab4ed17ea2f`。此前已有独立核对的 IP 副本与逐帧清单，因此本轮先比较旧证据，再创建仅绑定旧文件的格式确认。

| 检查 | 本轮结果 |
| --- | --- |
| 帧数、IP 字节、时间戳、旧逐帧字段 | 229 帧全部相同 |
| Protected 标志 | 105 帧；检查阶段保持未确认 |
| 聚合标志 | 第 5、6 帧；检查阶段保持未确认 |
| 新 IPv4 PCAP 与旧副本 | 整个文件逐字节相同 |
| RPC | 158 条消息、79 对完整配对，全部配对字段相同 |
| 完整 WebSocket 控制帧 | 4 个，与旧分析相同 |
| RPC 分析 issues | 空列表；不等于原始捕获完整性证明 |
| 原始文件与旧冻结清单 | 23 文件大小和 SHA-256 核对不变 |

新证据位于 `.tmp/normalized-readonly-proof-20261008/`，保留本机忽略目录：

| 文件 | SHA-256 |
| --- | --- |
| `format-review.json` | `19631e5b1dfd1bb9de28be54addedc1032355cf96f0551b51105be293b1f2874` |
| `ipv4.pcap` | `f18a81ed567bd06fc8eeef966594e61e46b6da42b12868e32f2a5735f101f2fd` |
| `manifest.json` | `01760d1bfb89f033461c12c429a0acf4d20f6a6b59972d10474b57f8be1b27bc` |
| `rpc-analysis.json` | `d955fadaa0739cb1a1c1ee8a1092460a0b58c1a1a8e43be744eb429e4a8d7819` |
| `verification.json` | `99c79e024f4d8e66b4bbf015b6adaa09215066bb9ec804ea6e1eb8fac147d83f` |

旧数据都是只读调用，其快回复不能解释运动中 stop_move 的约 610 ms 等待，也不能证明物理停止。

## 4. 本轮验证

新增转换用例 52 项。合成停止会话实际经过本机离线 TShark：转换后 RPC 配对与原 Ethernet 一致，stop_move 的请求/完整响应间隔保持 610000000 ns；缺失 first-byte 证据、孤立响应与重传情况没有被隐藏。

最终定向回归：**187 passed，33.80 秒，退出 0，无跳过**。范围为新转换器、RPC 分析器、单次运动捕获、子入口准入、只读捕获、配置哈希绑定与 Windows 启动器。没有后端全集、前端构建或真机测试。独立静态审查在这个范围未发现可操作 P0–P2；审查者没有运行入口或测试。

200 ms、stop_unverified、禁止自动 stop_sys/复位/重试均保持。没有修改 PVAT 与 RobotControl；完整源码哈希仍分别为 `cb6a1a4060c1728163d97baa08e745efe680440a820c14cf5a11212d324a0307`、`1e6f26eff7ca2e59e6341bd27a8f890c335e4f32b1f90da2d3d1d08b2a19fe43`。

## 5. 下一次真机诊断的具体范围（尚未授权）

目的：获得运动中 stop_move 的实际请求与响应收发证据，判断约 610 ms 等待发生在哪一段，再决定代码修复或需要厂商核对的事项。采用已经实现并离线审查的固定入口：

- 一次 SDK 连接；当前起始姿态允许时，只进行 +Z 3 mm 平移，观测到权威运动证据后触发已有早停。
- TCP 软件速度上限 0.005 m/s、加速度上限 0.02 m/s²；关节软件速度上限 0.05 rad/s、加速度上限 0.2 rad/s²。它们不能保证实际峰值，已有 J5 实测偏差仍未验收。
- 停止后只读观测 2 秒；指定组件 TCP 3031 捕获，名义 30 秒预算。预算不是磁盘或进程执行的硬上限。
- 不自动移动到准备位姿，不执行 Home、复位、夹爪动作、stop_sys 或重复尝试。沿用现有 200 ms 与 stop_unverified；未知停止状态须现场处理，不能靠强杀客户端或退出码证明机械臂停止。

执行前须用户明确批准这个范围，并确认当前 IDLE、目视静止、工具无物、整个机械臂路径空间无障碍、用户在场观察并可操作急停、没有其它控制或捕获客户端。起始姿态不适合时中止，不扩大为准备移动。仓库 `AGENTS.md` 明确要求新的现场范围授权，既往批准的只读额度已经执行结束。

本机还需重新核对捕获/筛选器归属。残留 inactive 筛选器 `VR4ArmReadonly_91f36a2d1799` 尚未清理；旧“空筛选器”基线已过时，新入口会拒绝。不得全局删除或重标旧基线为当前可用。刷新条件后才能生成本轮唯一目录、当前源码/配置绑定的完整预览与有效期；本轮没有创建可执行批准、live launcher 或 UAC。

采集后保留所有 ETL/分卷原件，核对结束状态与各卷哈希；多个转换分卷须使用经过核对的离线合并过程，不能把单卷冒充完整事件。对新的已结束 PCAPNG 重新 inspection 与逐帧核对，再进行 RPC 配对。应用与包时钟尚未对齐时，不能直接相减声称精确拆分 SDK/网络/控制器耗时；应用请求编号也不能直接当作线上的 JSON-RPC ID。

## 6. 距离真机 VR 控制的位置

停止诊断入口、独立性能计时、捕获转换与 RPC 分析已具备离线验证证据。下一步可以申请上述单次运动诊断；目前仍不能宣布连续 VR 控制已通过验收。

主要现场阻塞仍是运动中停止迟到，以及 J5 实际峰值相对请求限值的偏差。解决并验收之后，按小幅 VR 平移、Grip 松开停止、Trigger 开合顺序验证。此轮使下一次采集结果能够直接进入离线分析，没有修复或消除这两个现场阻塞。仿真物理抓取继续后置。
