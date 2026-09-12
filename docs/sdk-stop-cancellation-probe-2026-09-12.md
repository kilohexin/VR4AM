# SDK 停止超时隔离核验（2026-09-12）

## 范围

仅本机 127.0.0.1 WebSocket 假服务端；不读取现场 IP，不连接真机，不修改生产停止逻辑或保护参数。使用实际 `lebai-sdk-asyncio==0.3.13` Windows wheel，隔离安装在 `artifacts/acceptance/lebai-sdk-probe`，未改全局环境。临时实验脚本为 `artifacts/acceptance/stop-sdk-probe.py`（Git 忽略的实验目录）。

## 源码核对

通过 GitHub API 读取官方 tag v0.3.13：

- [src/rpc/mod.rs](https://github.com/lebai-robotics/lebai-sdk.rs/blob/v0.3.13/src/rpc/mod.rs)：`is_connected` 读取客户端连接标志，不发送探测 RPC。默认 request_timeout 为 30×60 秒；这不是本项目的 200 ms 等待预算，也不是建议使用的安全停止期限。
- [src/rpc/motion.rs](https://github.com/lebai-robotics/lebai-sdk.rs/blob/v0.3.13/src/rpc/motion.rs)：stop_move 直接等待底层 RPC。
- [src/rpc/system.rs](https://github.com/lebai-robotics/lebai-sdk.rs/blob/v0.3.13/src/rpc/system.rs)：stop_sys 同样等待底层 RPC。

## 实验

假服务端收到请求后，独立等待 650 ms 再完成并返回。客户端依次调用 stop_move、stop_sys、stop_move、stop_sys，每次均使用与项目相同的 `asyncio.wait_for(..., .20)`。每次超时后读取 is_connected。服务器只模拟请求生命周期，不模拟机械臂运动学、制动或真实控制盒任务调度。

首次运行记录（毫秒，相对实验开始）：

| 请求 | 假服务端收到 | Python 超时 | 假服务端完成并发送回复 |
|---|---|---|---|
| stop_move / id 0 | 2.1 | 202.6 | 652.7 |
| stop_sys / id 1 | 203.1 | 403.4 | 853.4 |
| stop_move / id 2 | 403.8 | 605.5 | 1055.3 |
| stop_sys / id 3 | 605.9 | 806.4 | 1256.4 |

四次 is_connected 均 true，服务器只收到上述四个请求，没有取消 RPC，也没有 is_connected RPC。断言通过，进程 EXIT=0。

## 可以得出的结论

1. 实际 SDK 的 Python 等待取消不会撤回本实验中已经到达服务器的停止请求。迟到的服务器操作与后续请求可以重叠存在。
2. 在当前项目中，200 ms 超时→stop_sys→shutdown 再次 stop_move/stop_sys 的流程，不能假定前一请求已被控制盒取消。
3. is_connected=true 不是控制盒响应健康、停止完成或请求未执行的证明。

## 不能得出的结论

- 假服务端的 650 ms 是实验设定，不是真机停止正常耗时。
- 没有证明真实控制盒并发执行、串行排队或迟到响应的确切行为。
- 没有证明请求重叠就是 22 mm 变化的根因，也没有证明放宽超时、删除兜底或减少停止调用足以安全修复。
- 本实验直接调用 SDK，未复现真实控制盒，也不算完整后端回归测试。

## 下一阶段建议

停止状态机需要设计审查，而不是继续简单增加超时：区分请求已发出、结果未知、状态已确认以及升级停止；保留既有禁止新运动和故障锁存；评估在 shutdown 中如何处理已经发出且结果未知的停止请求。不能为了避免重复而直接跳过必要的紧急安全升级。

落实生产变更之前，需核实厂商对重复 stop_move/stop_sys、请求响应时机及关节禁用的说明。现场仍暂停运动。可以把上述四个问题及 B01 原始日志交给厂商，不需要现场重复故障。

原始观察期收尾缺失的问题仍独立存在，等待终端超时/取消记录，不由本实验解释。

## 集成测试补充

新增 `backend/tests/commissioning/test_stop_timeout_lifecycle.py`，使用真实 run_smoke、RobotControl、RealLebaiAdapter 和磁盘记录器，仅替换外部 SDK 边界；不依赖临时安装的 SDK 包。

两种情形：服务器任务迟到返回、服务器任务永不返回。调用等待被取消时，模拟服务器任务保持存活；平移先通过模拟反馈达到目标，然后触发停止超时及 shutdown 再次停止。观察期提供 STOPPING→STOP、零速度但位置变化的样本。

已验证：

- 真实平移路径产生 PVAT，停止开始后无新增 PVAT。
- 两次停止失败均保留请求时序，未生成 stop_confirmed 或成功 smoke_result。
- 原始 smoke_stop_failed:stop_unverified 异常保留，不被收尾掩盖。
- 观察期位置变化和锁存故障留在原始日志中；观察汇总及 summary.json 正常生成。
- 观察 complete=true 只表示记录完成，不表示停止成功或安全。

验证命令：在 backend 运行 `python -m pytest tests/commissioning tests/robots/test_lebai_stop_evidence.py -q --tb=short`，73 passed（25.73 s）。未运行全后端套件。

敏感性验证：另一个独立 Python 进程内临时将观察函数替换为空实现，两个新增测试均在“缺少观察样本”断言失败；没有修改生产文件。正常进程中的上述 73 项随后通过。

这些是当前行为的刻画与保护测试，不是停止策略修复。服务器调度和位置变化是测试输入，不是真实控制盒物理模型。异常结束缺少汇总并非在本地自然复现；仍需现场进程执行/取消证据。
