# 单次只读收发诊断结果

2026-10-08，Asia/Shanghai。接续 [启动器修正](readonly-wire-launch-2026-10-08.md)。用户回复“现场无误，开始吧”刷新原已批准条件后，在提交 `4a7f5bae3d3885e3b58b15bfa13030f3a69e3307` 上执行了**一次只读连接、正常 5 秒读取窗口、名义 15 秒指定组件捕获**。该一次额度已经使用，没有重试。此次没有运动、夹爪写入、Home、复位、停止或倍率 RPC，不构成任何后续现场测试授权。

## 现场执行与收尾

- 管理员入口 UTC `03:39:40.2389319Z` 开始，`03:39:51.8212391Z` 结束；读取报告为 `03:39:44.532549Z` 至 `03:39:49.614476Z`。有效期仅限制开始连接；没有将名义窗口宣称为硬实时截止。
- 只读进程退出 0、没有超时；父进程也已退出。启动器记录本机 TCP 3031 established 在启动前和退出后均为 0。该本机表不能证明其它设备没有控制连接。
- Pktmon 已停止，本轮没有待清理采集或子进程。`acquisition_complete=true`。**人工筛选器清理仍待处理**：`VR4ArmReadonly_91f36a2d1799`，故 `complete=false`、入口退出 2，不掩盖为完全收尾。停止的采集不会因为保留筛选器继续抓包。
- 本机的 remove 帮助只提供全局删除，未自动删除任何筛选器。未来清理前必须复核采集空闲且筛选器仅属于本轮；出现外来筛选器或采集时停止清理，不以旧列表证明当前归属。
- 采集启动指定文件大小 32 MB，但第一份实际 ETL 为 **150,994,944 字节（144 MiB）**，第二份 11,739 字节。不能把此参数作为实际磁盘硬上限；没有删减原件，也没有将文件大小异常归因于已证实的特定机制。

## 已核对的结果

应用记录与线上 JSON-RPC 的方法、数量、顺序逐一一致。单一 TCP stream，整数 ID 0–78，每个请求匹配一条 result 回复。

| 方法 | 次数 |
| --- | ---: |
| get_robot_state | 19 |
| get_estop_reason | 19 |
| get_kin_data | 19 |
| get_claw | 19 |
| get_tcp | 1 |
| get_payload | 1 |
| get_gravity | 1 |

共 79 请求、79 回复、158 条 JSON-RPC；79 次应用读取全部返回。19 次状态为 IDLE，19 次急停原因 NONE，记录的关节速度绝对最大值为 0。它们是只读遥测；不代替现场目视判断，也不证明运动时软件停止有效。

229 个捕获帧中，79 对 RPC 的请求/回复完成帧间隔：最小 **1.226 ms**，中位 **1.579 ms**，最大 **4.263 ms**，均值约 **1.783 ms**。这是捕获位置上请求完成帧到回复完成帧的间隔，包含传输与回复过程，不能解释为控制器执行耗时。首字节时间未知，源时间戳实际精度为微秒。

独立离线复核重组现有 TCP/WebSocket：未发现序列缺口、冲突重叠、重复 ID 或未匹配消息；79 对方法及完成帧与分析器一致。此结论限于已捕获会话，不证明任意网络层或未来会话没有丢包。

## 解析障碍与证据保留

Pktmon 转出的 PCAPNG 声明 Ethernet linktype=1，实际 229 帧均具有 802.11 Data 头、LLC/SNAP 和明文 IPv4/TCP。直接 TShark 解析得到零 RPC，初始失败报告已保留。

仅将副本重标为 802.11 仍不充分：105 帧保护位与明文 LLC 不一致，帧 5/6 的 QoS A-MSDU 标志也与实际单个 LLC 载荷不一致，其中帧 6 承载 HTTP 101。没有解密流量或猜测缺失载荷。对此批已核对证据，离线按各帧实际地址/QoS/Order 头长定位 LLC，强制要求 IPv4 SNAP、完整非分片 TCP 与指定端口，提取精确 IP 字节至 raw IPv4 副本。

独立验证全部 229 帧 IP 字节、逐帧哈希、源文件哈希与时间戳；微秒 ticks 乘 1000 写入纳秒容器，**没有增加实际时间精度**。未覆盖 ETL、转换 PCAPNG 或初始失败分析。此解释只适用于本批驱动输出，不能推广为通用无线加密或聚合帧解码器。

原分析器保留四条 `unsupported_websocket_fragment` 提示。TShark 与独立 WebSocket 重组均确认它们为 FIN=true 的完整控制帧：7 Ping/opcode9、8 Pong/opcode10、224/226 Close/opcode8；没有把这些标签删除后声称原分析器无问题，也没有把控制帧计入 RPC。

## 应用计时限制

本机离线 `time.get_clock_info`：Python 3.12.14，monotonic 为 GetTickCount64，报告分辨率 15.625 ms；perf_counter 为 QueryPerformanceCounter，报告分辨率 100 ns。分辨率不是实测准确度。

79 次应用 monotonic await 中 **65 次为 0**，最大 16 ms。开始/结束时钟锚点的采样跨度均为 0，但偏移相差 **3.9272 ms**、偏移区间不重叠。因此不确认应用时钟与捕获时钟精确对齐，不将“0 ms”当作瞬时返回，不计算 SDK 排队/传输/控制器阶段的精确差值。独立的 wall 读取间隔也不能弥补这一限制。`app_clock_alignment_verified=false` 与 `hardware_stop_verified=false` 保持。

## 本机证据索引

所有配置、原始网络包、日志和完整工具输出留在忽略目录；提交仅包含本摘要与文档链接。

本轮目录 `.tmp/readonly-wire-next-20261008/`；启动记录 `.tmp/readonly-wire-launch-v2-20261008/launch.json`。逐帧映射、初始/派生分析、验证脚本及清单均在本机。完整 23 文件清单 `frozen-evidence-manifest.json` SHA-256：`cda5a236100dd7dd56032a5a171f3b20f149604026ae5f6490935ff6db2d70e4`。

| 文件 | SHA-256 |
| --- | --- |
| capture1.etl | `d9c9663320ce8166faa81ae710f0e55f53c2a2a1f04071f08f79defaa7a7cc8b` |
| capture2.etl | `619a0465140b3fdb50b94f3c4d7255fa8616aa58f7283e2cde526018d37a99f4` |
| state-report.json | `f73fc5573a003a82a691de36fb48f6ee7bcf0ad8613abb88108b64c93e646a61` |
| execution.json | `e350a9e548510421c0b0582300ee2a88993c86327c2dda5f3cc1f6897886cdba` |
| merged-readonly.pcapng | `69ee7de1352de6f64d557bed4d5ad8095ecfc4ff4e5b76d918318ab4ed17ea2f` |
| ipv4-interpretation.pcap | `f18a81ed567bd06fc8eeef966594e61e46b6da42b12868e32f2a5735f101f2fd` |
| verified-readonly-summary.json | `4a4b8dc4856d9b180c3fb9d0cd4412b099dd3db6180a814dc5a40795a39fb48d` |

该清单固定生成时的文件范围，不包括随后新增的运行时钟说明；后者位于 `.tmp/stop-wire-tools-20261007/clock-runtime-facts-20261008.json`。初次离线核对脚本曾因人工清理字段名不匹配而失败，修正字段后逐帧及全部查询断言通过、退出 0；未重跑现场采集。此次没有新增生产代码、跑后端全集或把此前 118 项测试算作本轮新测试。

## 真机 VR 的剩余阻塞与下一步

只读诊断链路已取得可配对证据；没有取得运动时 stop_move 请求/回复包。本次快速读取不能解释原冻结日志中的 SDK await 约 610 ms，也不能证明已修复。准备移动的真实限速验收、控制器/L Master 版本及厂家响应语义仍有缺口。

下一步先离线补充独立 QPC 诊断时间戳及其 wall 锚点，保留现有控制/截止时钟；明确时钟名称、分辨率、误差和缺失标志。再把本批捕获格式验证及控制帧分类纳入受限离线工具。完成相关定向验证后，准备一次可审阅的运动停止收发诊断范围；已有准备速度异常未解释前，不直接复用准备移动。任何新的真机连接或动作仍需用户另行明确批准并刷新现场条件。

200 ms 截止、stop_unverified、禁自动 stop_sys/复位/重试保持。本轮重新核对原 motion 日志、只读配置及 PVAT/停止事务/控制源码哈希均未变。暂不推送。物理抓取仿真排在真机 VR 核心验证之后。
