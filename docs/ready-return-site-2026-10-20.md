# 单次返回原工作准备位：2026-10-20 现场结果

## 结果

用户在 `call_f3f7a9c9f6e344c9ae8a1023177cc75c` 批准新范围和当前现场条件后，使用实现 `a2144d5`（执行时 HEAD `65a436a`）完成一次程序回位：**SDK 连接 1、movej 1、stop_move 0，正常到位、后检及两秒只读尾段通过，整体 complete=true。** 无 Home、夹爪、恢复、复位、speedl／PVAT、抓包、重试或追加动作。

用户随后在 `call_4a658b6cb3fc4799960540fbd4e3b4ba` 确认：完全静止、过程无异常、未使用急停。此目视确认与 SDK／软件判据分别记录。

这次解决了当前记录中的准备位起点偏差：末次最大关节误差约 **0.000383495 rad／0.0219727°**，在原 **0.001 rad** 容差内。未改变原准备位、配置或容差。此结论限于本次末次反馈，不能替代下一次现场的新鲜核对。

## 调用与到位记录

| 项目 | 本次证据 |
| --- | --- |
| 准备位方法 | 原 prepare(HomeOptions)，唯一关节 movej |
| 请求 | v=0.05 rad/s、a=0.2 rad/s²、t/r=0 |
| 回执 | 已知正常返回，运动编号 579；原生 await 约 41.41 ms |
| 准备位误差 | 最大 0.000383495 rad，原门槛 0.001 rad |
| 到位后正常预检 | ready=true |
| 两秒只读尾段 | 11 组 state 快照，末次 IDLE／FINISHED |
| 速度反馈审计 | 14 组有效、0 组无效、0 组超限；within_observed_limit |
| 运动／读取未知结果 | 无运动未知结果；本次 latched_fault=null |
| 日志与进程 | 日志关闭核验通过，CLI 退出码 0，退出时未决本地任务 0 |
| 本机资源 | 局部资源释放；退出后读取本机 RPC 表为 0 |

审计的 **14 组 actual_qd 均报告 0**；target_qd 峰值约 **0.00649848 rad/s**，关节位置记录发生了变化。因此只能说明该离散审计未观测到实际速度超限，**不能宣称已测得实际运动峰值、验证物理限速或消除历史速度反馈问题**。最大读取起点间隔约 124.89 ms，最长快照约 74.90 ms；这些是客户端读取边界，不是控制器同步采样时刻。

本次自然到位，未调用 stop_move，不能验收运动中停止；历史 stop_unverified、跟随和连续控制问题保持。原始结果的 hardware_verified／physical_speed_verified／motion_time_stop_verified 仍为 false。

## 批准与证据

新批准材料生成于北京时间 17:12:11，启动有效期至 17:22:11。先确认默认 CLI 预览与批准材料完全一致，再通过 Python 原样传递参数执行一次。未复用旧回位包装或已消耗授权。

材料目录：`.tmp/ready-return-site-20261020T091211Z/`。118 个运行文件、三个输入文件执行前后哈希一致；原配置、原准备位文件、静止清理原始会话及本机用户总结未修改。原始材料不提交。

| 文件（相对材料目录） | SHA-256 |
| --- | --- |
| `approved-preview.json` | `2d409fe3ad60b5a5d6661b81a9be222eb300efba430d011307770e724decec97` |
| `result/result.json` | `07057115b12750c9736756887383ab857e51ef006c71066a126eb45c24e25ecf` |
| `result/sessions/20261020T091241Z-3cb0af6d/session.jsonl` | `45cc938010eeeaad7bb3f0cf360fad5204440ac83773051993e21e3cbad94f4a` |

同目录保留 authorization、execution-attempt、process-exit、offline-analysis、onsite-closure 和 scope-consumed。本次授权已消耗。日志中的 ready_return_result 是明确标注的 provisional／complete=false 事件；日志关闭后的 `result/result.json` 才是本次最终结果，不把 provisional 当作失败或提前成功。

## 下一步

可为已有的“唯一 FK／新快照配对后单次 +Z speedl”重新准备独立范围。它仍需新现场授权、原全部预检和发送前新鲜度核对，不能因本次回位成功自动执行。软件继续使用原 100／60 ms 和 200／300／500 ms 门槛，不开放网页／VR 连续操纵。

相关：[离线实现与保留的时序波动](ready-return-entry-offline-2026-10-20.md)、[此前静止清理](static-cleanup-after-jog-admission-2026-10-20.md)。
