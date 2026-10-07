# 只读收发诊断工具：实现与离线验证

2026-10-08（用户端 Asia/Shanghai 日期）。接续 [停止收发计划](stop-wire-diagnostic-plan-2026-10-07.md)。用户要求加速并完成更多工作后汇报；本轮完成解析器、只读采集入口和配置哈希绑定。**没有新的机器人连接/命令、现场抓包、服务部署或推送。运动中 stop_move 约 610 ms 与实际限速问题仍未验收。**

## 本机工具状态

- 用户明确要求代为安装后，标准 Wireshark 4.6.9 x64 已安装到 `C:\Program Files\Wireshark`：系统卸载项、TShark 版本与有效签名均核对，安装进程已结束。下载包 97,931,232 字节，SHA-256 `bf9b5ce8a89f244c376a9b1a946276eaa06463dde3e33069a34d7f102f5878cf`，与 [官方发布记录](https://lists.wireshark.org/archives/wireshark-announce/202609/msg00000.html) 一致。TShark 本机 SHA-256 `662e10fd28a8c58575fe567845d0248080a2856c3d18d6f2ff7a4b4c8511f69b`。
- 之前便携包的无人值守解包未完成，等待 60 秒后结束了本轮所属的 PID 51960；没有以它宣称成功。改用标准包 `/S /desktopicon=no`。标准静默安装不会安装 Npcap；后续采集方案采用 Windows Pktmon。[官方安装说明](https://www.wireshark.org/docs/wsug_html_chunked/ChBuildInstallWinInstall.html)。
- 本机 Windows 11 build 22000；Pktmon 驱动服务原为手动启动/Stopped。非管理员的 status/filter/list 查询失败，不能当作空闲或无筛选器证明。用户完成 Windows 管理员确认后，**本机工具准备**仅启用驱动并查询状态、筛选器和组件，退出 0；驱动现在 Running，采集状态输出“数据包监视器没有运行。”、筛选器“无”，三项查询均退出 0。没有启动抓包或连接机器人。
- 原始本机预检、网络库存、审阅脚本和基线均保留在忽略目录 `.tmp/stop-wire-tools-20261007/`。审阅实际空闲/空筛选器输出，结合本机 IPv4 路由的最长前缀及路由/接口 metric、Up 网卡与 Pktmon 微型端口 ifIndex/描述对应关系，得到单一组件 **3**、路由前缀长度 **24**。仅检查本地路由表，没有网络探测；不能据此证明机器人或抓包链路可用。`reviewed-tool-baseline-20261008.json` SHA-256 `ef9f8eb871fdab36b13a7f7710a751c4052714e28f246f4dba2e33d665a6d3e0`，记录原预检/库存/配置哈希。只读现场范围还未批准；执行前仍须刷新现场条件与复核基线、组件。

## 两个可复用入口

### `scripts/analyze_rpc_capture.py`

只解析已有 PCAP/PCAPNG：TShark 参数固定为 `-2 -n -r`、指定端口 HTTP/WebSocket 解码和 TCP 重组，无 `-i` 或实时采集。记录原件与解析二进制哈希，保留 prior output，拒绝解析期间原件变化。JSON-RPC 只按 TCP stream + ID 类型 + ID 匹配；数字 7 与字符串 "7" 不合并，重复 ID 不猜选，缺失请求/回复、缺失握手、倒序时刻、截断/错误 JSON、重复 JSON 键及不支持的 WebSocket 分片显式列出。逐流记录带数据的 TCP 帧与 HTTP/WebSocket 重组 carrier；没有被完整解码覆盖的帧标记 `unaccounted_tcp_payload`。同帧完整消息与尾部残片共存时，以 PDU 大小、重组长度及当前 carrier 字节数核对，未被后续重组覆盖的尾部标记 `incomplete_application_payload`；即使此前已有有效匹配也保留缺口。重传作为独立 transport observation，不重复计 RPC。

报告时间为**完整消息由解析器重组的捕获帧**，用 Decimal 转换为整数 ns。TCP carrier frame 列表保留，但不泛化为精确首字节时间；first_byte_epoch_ns 为 null。RPC error 回复不变成成功。报告始终 `hardware_stop_verified=false / app_clock_alignment_verified=false`；CLI 退出 0 表示产生分析文件，不证明抓包完整、控制器接收、物理停止或时钟已对齐。单输入文件限 64 MiB、解析输出限 128 MiB；分片 ETL 转成多个 PCAP 时，需要先离线合并为一个完整时间线，不能把多个文件各自的 stream 编号混用。

### `scripts/readonly_wire_capture.py`

默认只做本机配置预览，不构造执行后端、不创建采集目录或连接。执行要求：新的现场范围授权、显式确认常量、管理员进程、配置预期哈希、**已审阅为空闲/无筛选器的本机输出基线**及一个明确组件 ID。基线记录 `reviewed_idle=true`、`reviewed_empty_filters=true` 和规范化 status/filter 输出 SHA-256；这些声明必须来自对实际输出的审阅，**哈希相等本身不证明语义安全**，不能拿任意 active 输出签成基线。

预期配置必须为 readonly / 字面 IP；捕获目录只能是仓库 `.tmp` 下的新目录，并一次性占用。冻结 backend/app Python 源码及只读入口/本入口哈希；捕获前及连接前复核。仅增加本轮命名的 TCP/IP/端口筛选器，使用指定单组件、`--pkt-size 0`、32 MiB multi-file 日志；不捕获全部网卡。固定启动 `real_robot_readonly_state.py` 一次，七项初始化查询及原四项轮询不变，不加入倍率/版本或通用 RPC，不构造 control adapter。

原配置不改写；执行目录内生成独立 readonly 副本，连接前再次核对，并把预期 SHA-256 传入只读子进程。`backend/app/commissioning/readonly_state.py` 新增可选 `--expected-config-sha256`：在读取配置后、连接前验证批准字节；默认旧接口保留。任何源文件/配置变化不自动恢复或覆盖。

外层名义捕获/子进程预算 15 秒，正常只读窗口 5 秒；超时终止本轮只读子进程，不重试。系统调度、native 命令等待和进程清理不是硬实时保证，不能承诺 15 秒物理/网络上界。cleanup 只有在**当前日志字段仍指向本轮目录中的 capture*.etl**时停止本轮采集；未知/外部采集不停止。本机 `pktmon filter remove help` 只说明全局删除；即使输出指纹相等，也不能排除并发新增筛选器。因此入口**不执行任何 filter remove**，保留本轮筛选器名与创建后输出指纹，明确 `manual_filter_cleanup_required=true`。后续必须先核实本机采集已空闲并人工审阅筛选器归属，才能另行处理，不能自动全局删除。Pktmon 是全局工具，现场窗口还须保证没有其他操作者或客户端并发修改采集状态/筛选器；这些前后检查不提供原子互斥。失败明确输出 manual_capture/filter/process_cleanup_required；不隐藏为通过。ETL 转换逐文件保存哈希。

该脚本没有改变 SDK 停止策略、200 ms 截止、stop_unverified、速度/加速度或 VR runtime。它不是一次新的动作额度。`acquisition_complete` 只描述只读进程、采集停止与文件转换成功；存在人工筛选器收尾时仍 `complete=false`、CLI 退出 2，不能把产物成功解释为所有收尾完成或运动就绪。脚本没有自动解除人工收尾状态的入口。

## 验证与发现

- 实际 TShark 对人工 PCAP：14 帧、5 条 JSON-RPC 消息、2 对匹配，重组帧 6/7 和 11/12；完成帧间隔分别 610,000,000 ns / 10,000,000 ns。重复包没有变成第二次 RPC，无请求 ID 99 回复独立标记。未发任何网络包或 SDK 调用。人工文件 SHA-256 `8d294cc99c448f1f333218b11bf8341949a80382d98a19354b11ebc6f2d7b9ba`；原件在忽略目录，测试仓库只提交生成源码。
- 首轮 API 缺失红灯后 26 通过；实际 TShark 首次产生 0 对，原因是 PDML 布尔显示 `True` 与人工样例 `1` 不同。新增红灯复现并改读原始值后 27 通过，实际结果正确；保留首次与 v2 输出。后续边界测试发现 Decimal 溢出、配置副本改变仍可启动子进程、源配置删除会丢失收尾记录，以及外部采集可能被误停；对应红灯复现后修复。
- 独立静态审查发现 P1 全局筛选器删除有并发归属漏洞、P2 无 WebSocket 节点的尾部数据会被忽略。新增回归先得到 3 failed / 1 passed（包括实际 TShark 对 15 帧尾部不完整样例）。复核再发现同帧完整消息加残片会被整帧记账隐藏，实际 TShark 红灯复现后补齐；首次仅看 segment_data 对正常重组误报，经 PDU/当前 carrier 大小核对后修复。最终复核确认两项发现已解决，未发现此次范围内其它可操作问题；审查仅静态读取，没有测试或现场动作。
- 新增 71 项：解析 35、入口 28、只读哈希 8；包含实际 TShark 合成重组、独立尾部、同帧完整/部分消息及重组末帧残片，以及真实启动一个无害 sleeping Python 的 watchdog 结束核对。不是机器人 SDK mock 对真机效果的证明。
- 与既有记录、只读、停止时序/迟到锁存/诊断回归共同运行 **11 文件 150 passed，11.81 秒，退出 0**。未跑后端全集、前端构建。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/scripts/test_rpc_capture_analysis.py backend/tests/scripts/test_readonly_wire_capture.py backend/tests/scripts/test_readonly_capture_hash_binding.py backend/tests/recording/test_clock_anchor.py backend/tests/recording/test_commissioning.py backend/tests/scripts/test_readonly_state.py backend/tests/scripts/test_readonly_velocity_factor.py backend/tests/robots/test_stop_transaction.py backend/tests/robots/test_stop_timing_boundaries.py backend/tests/commissioning/test_stop_timeout_lifecycle.py backend/tests/commissioning/test_stop_rpc_diagnostics.py -q --tb=short -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

## 原件与下一步

旧现场 session SHA-256 `f6bdc1375bc713bfacdf172b868d8e2676815fbee4a091c3eda78a276a70dcb6`、PVAT 构造器 `cb6a1a4060c1728163d97baa08e745efe680440a820c14cf5a11212d324a0307`、readonly 候选 `faba03b57d9f18514cbb6813ee6780a80c1513a270d425620922bb14b0372a94` 均复核不变。先前使用 component 1 的软件预览仅示例，不是现场 NIC 的认定，也未创建输出目录；后续本机审阅确定组件 3。对应离线冻结预览 `.tmp/stop-wire-tools-20261007/readonly-wire-reviewed-preview-20261008.json` SHA-256 `62726baa08823e28bdeaf780eb4957856e216a632cdaf2f1d32ceb76e3843400`；候选捕获目录 `.tmp/readonly-wire-next-20261008` 尚未创建，没有构造 Native 或执行采集/SDK。

本机工具安装、空闲/无筛选器审阅和组件映射已完成。下一步申请 [原计划](stop-wire-diagnostic-plan-2026-10-07.md) 中的一次只读连接加定向捕获，并刷新上电/IDLE/静止、现场观察及无其它机器人客户端/并发采集操作者条件；不能把用户“继续”或安装授权直接当作该现场范围的批准。此次只读收发只验证证据链路，不能定位或验收运动中的停止延迟。L Master 版本/厂家回复尚未补齐，但无需为此启动或移动机械臂。
