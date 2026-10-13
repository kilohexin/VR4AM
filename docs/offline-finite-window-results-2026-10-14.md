# 单窗口完整流程离线排练 — 2026-10-14

## 当前范围

用户明确答复“现场条件不满足，继续离线”。已将 `.tmp/paused-stop-cleanup-approval-20261013T174835Z/authorization-state.json` 标记为未批准、用户取消、未执行；对应现场输出目录没有创建。本批没有连接机器人、导入原生 SDK、发送设备命令或修改现场配置。

新增 `scripts/offline_finite_window.py`：只提供合成控制器，CLI 没有 live、目标 IP 或 execute-once 选项。原配置仅用于读取原约束，冻结现场结果仅提供关节/PVA/TCP 数字输入。合成控制器故意从 IDLE 开始，这不是将真实 PAUSED 转为 IDLE。

## 完整流程已接通

排练使用真实 `FiniteWindowDiagnosticAdapter`、原停止事务、日志记录器和两秒只读尾段。其余边界是替身：FK 是关节 1 到 +Z 的一维仿射映射，PVAT 立即更新合成反馈，没有机械臂运动学或物理引擎。

正常流程是准入 → 一个 IK → 反馈/目标/中点/终点四个 FK → 一个完整有限窗口 → 原 stop_move 事务 → 只读尾段 → 资源关闭和结果报告。窗口为 80 ms、末端六关节速度和加速度全部为零；没有连续补发、恢复、Home 或夹爪目标。

保留原 100 ms 租约、200 ms 停止 RPC、300 ms 稳定/500 ms 验证及故障锁存。此入口没有接入正常 VR/API，也没有引入可执行真机运动的入口。

## 冻结现场参数的独立 CLI 排练

数字输入取自 `.tmp/readonly-kinematics-site-20261013T140732Z/result.json`，原控制配置保持原 SHA-256。三次独立进程均只运行合成控制器：

| 场景 | 合成 PVAT / stop_move 次数 | 结果 |
| --- | --- | --- |
| 正常 | 1 / 1 | 一个 IK、四个 FK、80 ms 零末端 PVA，原验证与尾段完成，模拟 complete=true，工具 exit 0 |
| FK 超时 | 0 / 1 | 不发送窗口，保留 finite_fk_timeout，complete=false，工具非零退出 |
| 合成停止等待 621 ms | 1 / 1 | 保留 stop_unverified，complete=false，工具非零退出 |

两个故障场景工具均报告 exit 1，脚本失败分支设计返回 2；没有将非零退出改写为成功。它们用于检查失败路径。迟到停止或 disconnect 不会增加第二次停止。失败流程可能在停止异常后直接收尾，tail_complete=false，不能把缺失尾段算成完成。

所有结果标记 SIMULATION、simulation_only=true、hardware_verified=false、motion_authorized=false。上述模拟毫秒数不是实机停止耗时或性能证据。

## 测试与审查

新增 17 项用例，覆盖完整流程、四类数学/租约失败、两类停止失败、无效反馈、输出目录一次占用、拒绝真机执行参数及四种取消路径。数学故障用例检查实际 IK/FK 进入次数，防止在更早准入失败时误通过。

初始缺入口产生 13 个 setup 错误；调整测试结构后确认 13 个预期失败。实现后发现替身缺少原适配器的能力声明：4 失败/9 通过；加强分支断言后为 8 失败/5 通过。补齐声明，但禁止实际调用 stop_sys/movej/set_claw 后，13 项通过。

独立审查发现两项 P2，并用四个有效失败重现：

- PVAT 后取消时，disconnect 首次执行停止，旧报告在它之前采集 stop/latch，漏记新 stop_unverified。
- 记录器启动、适配器断开或记录器关闭阶段取消会跳过 result.json；失败运行另出现 pending writer 日志。

现已把启动纳入收尾范围，断开后采集最终停止状态，逐段处理取消，先保存 interrupted/incomplete 结果再传播取消。修改后 17 项通过，没有上述 pending writer 日志。

最终五个相关文件 **109 passed，33.37 s**，无失败、错误或跳过；没有重跑后端全集。独立最终静态复审无剩余 P0/P1/P2；审查没有运行测试或访问 SDK、机器人、网络、系统。

## 实机进度和下一步

本批补齐了诊断组件到可重复离线入口的整条流程及异常记录，减少下次现场操作前临时拼接代码的工作。它没有解决真实 FK/IK 超时、约 621 ms 的运动停止回复或横向偏移，也没有验收连续 VR / Trigger。

现场恢复后，仍需先用新范围确认当前状态，决定 PAUSED 收尾或独立只读 FK/IK 核对。取得真实一致性证据后，才准备专用单窗口运动范围与授权。当前继续离线，不要求用户启动、恢复或操作机械臂。

本机证据目录：`.tmp/offline-finite-window-20261014/`。配置、冻结原始结果和合成排练日志均不提交 Git。
