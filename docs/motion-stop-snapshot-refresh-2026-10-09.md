# 单次抓包诊断结果与状态刷新修复

2026-10-09，Asia/Shanghai。接续 [网络前置记录](motion-stop-network-preflight-2026-10-09.md)。本轮实际使用一次 SDK 连接并完成抓包；**3 mm 动作在运动写入前被拒绝，没有复现运动中的停止等待。** 之后仅进行离线定位、修复和定向测试，没有重试机器人测试。

## 当前网络与本次范围

用户提供当前 L Master 的本地地址，并确认旧地址也能通过 Wi-Fi 使用。此前“只有默认路由”的结果不能证明机器人不可达；实际是当前接入实验室网络，与旧同网段基线不同。

新的临时配置只改目标 IP，其余 YAML 字段逐项一致；原配置字节与 SHA-256 保留。新配置 SHA-256 `8beddfd668530e0449758b59991390b2c5160a21b94de7a14e9457614494e698`。重新核对 /22 同网段直连路由、原无线网卡 ifGuid / ifIndex / WLAN 身份、当前 Pktmon 组件 259、idle / 空筛选器 / RPC 0；没有重复清理已经删除的旧筛选器。

完整批准预览与 Windows 默认预览逐项一致：一次 +Z 3 mm、运动证据后早停、2 秒只读尾段，保留原低速约束。预览 SHA-256 `b5153e25b0e3e1d27cddf23658e3682154cf1126bcbe1905b81a547e8823f53f`，工具基线 SHA-256 `cae6518726056fdbdbd3a34b3de61eb8880e3e8ed13fe81fcca4da26a37cff2c`。

## 真机记录与收尾

会话 `20261009T104428Z-517d9ec4`，原始 `session.jsonl` 431077 字节，SHA-256 `62e36486fc836502fb467408f4f1fe81df66e8938d8f9c3c53836e73e9cd9ebf`。记录确认：

- 预检两次 ready；模式先后为 ARMED、ACTIVE。
- 首个目标映射为 +Z 3 mm，但提交时触发 `robot_state_stale`；没有 IK / PVAT 事件，也没有运动证据。
- 210 份 kinematics 全部为 IDLE，实际 q 与 TCP 各只有一个不同值，报告速度最大值为 0。该记录不能替代独立的现场观察。
- 故障停止调用正常返回：原控制 monotonic await 31 ms，独立 performance clock await 19.9058 ms；这些分别来自不同时间基准，未做跨时钟对齐。它们属于静止状态下的停止调用，不能推翻旧约 610 ms 的运动停止问题。
- 日志有两次 `stop_confirmed`、没有 `stop_failed` / `stop_unverified`；shutdown 复用 request 1，没有新的 stop_move 请求。最终出现 `smoke_early_stop_not_reproduced`，运动子进程退出 1；2 秒只读尾段完成 10 份样本。

父入口对诊断失败保守设置 `robot_stop_unknown=true` / operator attention。已立即向用户提示检查现场，并请求完全静止、异常及急停情况；截至本报告写入该观察回复仍待补充。告警没有自动清除，不以 IDLE 或子进程退出作为物理停止证明。

捕获已停止并核对 idle，两个 PCAPNG 已保存；运动子进程自然退出，没有强杀。Windows 启动器与管理员入口均退出 2，收尾连接表可读且 TCP 3031 Established 为 0。入口保持 `complete=false` / `acquisition_complete=false` / `hardware_stop_verified=false`，一次 SDK 诊断额度已使用。当前短名称筛选器留为 inactive、`manual_filter_cleanup_required=true`；未自动清空。

## 根因与离线修复

首个目标前最近一次状态捕获到拒绝相隔 **516 ms**，状态读取完成到拒绝相隔 **500 ms**。调用链是：读取初始状态 → 本机 Pktmon status / filter 与材料检查 → 发布首个 target。同步本机检查使此前快照老化，adapter 的原新鲜度检查正确拒绝了写入。

采用限定修复：完整 `before_motion` 检查后，通过正常 backend 路径重新读取状态；该 await 之后，专用 `before_motion_commit` 仅做本地批准、配置、源码哈希与到期检查，不再次调用慢 Pktmon 查询。没有专用 callback 时复用原 guard 再检查；commit callback 不能脱离完整 guard 使用。修改仅涉及 commissioning 调度与专用诊断子入口，没有改变 PVAT、RobotControl、新鲜度门限、200 ms 停止门限或故障锁存规则。

先用 fake robot 和 550 ms 本机检查复现：旧代码零 PVAT，新增回归明确失败。补齐刷新等待期间到期、配置变化与禁止末次检查再构造 Native 的失败路径；修复后测试验证正常刷新可提交，故意将末次检查再次拖慢仍产生 `robot_state_stale`。

最终 **7 个定向文件，143 passed，68.89 秒，退出 0**：动作 commissioning、SDK 准入、运动捕获、Windows 启动器、停止诊断、超时生命周期和只读尾段。没有后端全集或新的现场测试。独立静态复核未发现新增 P0–P2。

## 证据解释边界与下一步

41 个已闭合材料冻结于 `.tmp/motion-stop-wire-20261009T1043/frozen-evidence-manifest.json`，SHA-256 `21709671aa0f7aad065fe15e731573853e691b58322e2da133eadfc618d40bd5`。现场配置、地址、原始工具输出与日志均只留在本机忽略目录。

离线合并 PCAPNG SHA-256 `2b2f99d33d592f54862ced2bb10bcf981a2dfe0a5b9fbc723c095d16293ba703`。声明的 Ethernet 格式被解析器拒绝；显式 WLAN inspection 可读取 3721 帧，但 `review_required=true` / `interpretation_verified=false`。尚未为这个新文件签发逐帧解释 review，未复用旧文件 review，也未将这些帧报告为已验证的运动停止 RPC 配对或跨时钟定位结果。

新修复尚未验证真机。旧预览 runtime 哈希因修复失效，不能用旧材料重跑；下一次需当前现场观察确认、新的单次动作授权、当前绑定材料与独立目录。本轮不自动重新连接、Home、复位、stop_sys 或重试。约 610 ms 运动停止、J5 实际速度与轨迹精度验收仍未完成，不能放行连续 VR 控制。暂不推送。
