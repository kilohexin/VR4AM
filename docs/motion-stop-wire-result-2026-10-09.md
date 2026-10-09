# 运动停止抓包结果

2026-10-09，Asia/Shanghai。接续 [状态刷新修复](motion-stop-snapshot-refresh-2026-10-09.md)。用户明确批准一次 SDK 连接、+Z 3 mm 目标、取得运动证据后早停、2 秒只读尾段；本次额度已经用完，没有自动重试。

## 本次结论

状态刷新修复已通过本次真实运动验证：首个目标不再被过期状态拒绝，记录了 25 次 PVAT 和关节位置变化。但运动停止仍超过原 200 ms 门限，`stop_unverified` 保持锁存，不能放行连续 VR 控制。

抓包中唯一 `stop_move` 的完整请求至完整回复间隔为 **613.042 ms**。此前约 610 ms 的等待，现在有独立的网络报文证据：延迟发生在本机捕获点所见的请求与回复之间。它包含网络传输、控制器处理及回复交付，不能据此全部归因于控制器，也不是首字节延迟或物理停止时间。

## 动作和停止事务

- 会话 `20261009T110511Z-aa9a5827`；早停依据为两份关节位置变化样本，最大关节位移 0.0159151 rad，报告速度 0.00958738 rad/s。当时 Z 方向进度约 0.130130 mm；3 mm 是目标，不是已完成位移。本次是限定非 VR 诊断，尚非头显实操验收。
- 25 次 `move_pvat` 均在停止请求之前开始；最后一次开始于停止前 94 ms、完成于停止前 79 ms，没有停止后新开始的运动写入。停止入口有 pending，shutdown 时已无 pending。
- Grip release 停止在约 +203 ms 失败，shutdown 在约 +218 ms 复用 request 1，未另发第二个 `stop_move`。日志无 `stop_confirmed`，最终 SDK 返回不能解除故障锁存。
- 控制 monotonic 的 SDK await 为 625 ms；独立 QPC await 为 613.7317 ms、SDK 锁等待为 0.2314 ms。不同时间基准不作精确相减，也未完成应用时钟与抓包时钟校准。
- 2.016 秒只读尾段成功取得 10 份样本，无读取错误。最后 MOVING 样本的读取窗口为停止后 +421～453 ms，首个 IDLE 样本为约 +625 ms。这些是非原子状态读取的窗口，不能给出精确物理停止时刻。

首个 IDLE 之后的 8 份 IDLE 样本中，相对于首份的最大关节位置变化约 0.000862864 rad、最大 TCP 位置差约 0.320129 mm。遥测仍有变化，不能单凭状态枚举推断已经停稳；本轮未区分真实微动、测量变化或异步字段更新。

告警后，用户明确回复：**“完全静止，无异常，未使用急停”**。这是后续现场观察，未提供停下的准确时刻，不替代 200 ms 软件停止验收。

## 报文解释与独立核对

两份已闭合 PCAPNG 合并后有 1475 帧。原文件声明 Ethernet，但实际载荷需按显式 WLAN / LLC SNAP 布局解释。新文件独立解析逐帧确认 IPv4 偏移、长度、原帧与提取数据哈希；1475 份映射与可复用解析器一致。730 帧有 protected 标志、188 帧有 aggregation 标志；审查仅认可本文件所记录偏移处经验证的明文 IPv4，不宣称解密能力或通用聚合解析能力。

仅为这个新文件签发完整映射 review，没有复用旧文件批准。独立提取与可复用 normalize 输出的 IPv4 PCAP **逐字节相同**；原始帧数据不变，源时间单位仍为微秒，纳秒容器不增加精度。

离线 TShark 得到 1256 条 JSON-RPC 消息、628 对请求回复，issues 为空。唯一停止请求：stream 0、整数 id 547，完整请求 frame 1269、完整回复 frame 1333，间隔 613042000 ns。`first_byte_epoch_ns=null`，`app_clock_alignment_verified=false`，`hardware_stop_verified=false`。

## 收尾与证据

捕获已停止并核对 idle，运动子进程自然退出 1，没有强杀；管理员入口退出 2。连接表可读，最后本机 TCP 3031 Established 为 0。执行报告保留 `robot_stop_unknown=true`、`complete=false`、`hardware_stop_verified=false`。inactive 筛选器 `VR4ArmStop_d46f5d60ef94` 保留待处理；本次批准的旧筛选器清理已执行一次，不用于新筛选器自动清理。

原始日志、抓包、配置、地址与工具输出只保存在本机忽略目录：

| 材料 | 路径或 SHA-256 |
| --- | --- |
| 本次会话 | `.tmp/motion-stop-wire-20261009T1103/sessions/20261009T110511Z-aa9a5827/session.jsonl` |
| 会话 SHA-256 | `486480bbd496e4a0f5d3ce9f5c8d46e19b141169bc3dd32af6fe832cf11918e8` |
| 合并抓包 SHA-256 | `f993881021640187a42ebb2aacc6217198339be31571719cfee1f744df9ff930` |
| 两路一致的 IPv4 PCAP SHA-256 | `10fedcdecfee8b54a1f4e93c99f2e35cbc9c9c4e62c23ac3ce71b933b69ea0ff` |
| 文件专属 review SHA-256 | `f80591b6331c9e80ca0521da92922bf28b2585e737525b433ce1ec5f5dc6157a` |
| 报文分析 | `.tmp/normalized-motion-stop-20261009T1103/rpc-analysis.json` |
| 现场回复与离线摘要 | `.tmp/motion-stop-wire-20261009T1103/offline-evidence-summary.json` |

本轮离线检查还包括 8 项清理 guard 合成检查、绑定预览一致性和独立逐帧核对；没有重跑后端全集。状态刷新修复此前的 143 项定向测试是上一轮结果，本轮没有重新运行。PVAT、RobotControl、200 ms 门限及停止规则未改变。

57 个本次闭合材料已冻结到 `.tmp/motion-stop-wire-20261009T1103/frozen-evidence-manifest.json`，SHA-256 `21b524189072afe3a45bfcee59c382c6a1e16ec69aa78542bf672e1ff1805649`，全部逐项复核通过。上一轮 41 个冻结材料及清单哈希也全部保持不变。

独立只读复核确认报文配对、两路提取一致性、SDK/锁计时及停止后无新 PVAT，未发现推翻当前结论的严重证据缺口；复核指出的 IDLE 后遥测变化已补充到上述解释边界。

下一步先离线检查 SDK 请求分发与回复处理、运动队列及停止语义可获得的源码证据，区分下一次诊断应检验的假设。网络与控制器内部耗时仍未拆分；轨迹精度、实际速度、连续 VR 与 Trigger 抓取验收仍未完成。任何新的真机连接或动作需新的明确范围和现场条件。暂不推送。
