# 单次静止清理结果 — 2026-10-20

## 软件结果

用户批准 `call_c55929809f8a4c408a414c7fda9d5dbd` 后，使用代码 `1dd09c3`、既有静止清理入口执行了一次 SDK 连接。新读取确认 PAUSED／FINISHED、零报告关节速度和原几何／连续性等预检后，**仅一次 stop_move，原停止验证及两秒只读尾段通过**。没有发送运动目标、准备位、Home、夹爪、恢复、复位或追加停止；未重试。

**本结果只证明本次静止清理的软件判据满足，不能验收历史运动中停止、真实速度／跟随或连续 VR／网页控制。** 原始结果的 `hardware_stop_verified=false` 保持。用户随后在 `call_3206e4aff6fe40ff84cad435dedd33c4` 明确确认：全程完全静止、无异常、未使用急停。该目视反馈独立于 SDK 的 IDLE 和零反馈速度记录。

## 记录

材料目录：`.tmp/paired-jog-static-cleanup-site-20261020T081405Z/`。新材料生成于北京时间 16:14:05，启动有效期至 16:24:05；默认 CLI 预览与批准预览一致，Python 原样传参，唯一执行记录先于启动保存。

| 项目 | 本次结果 |
| --- | --- |
| SDK 连接／原生 stop_move | 各 1 次 |
| 原生读取 | 161 次，均已知返回，无未决读取结果 |
| IK／FK／运动目标 | 均为 0 |
| 原生 stop_move await 的 QPC 时长 | 4.0222 ms；这是代理记录的原生 await 区间，不是整条停止事务或物理制动时间 |
| 原停止验证样本 | 7 组：先 PAUSED，随后 6 组 IDLE；任务均 FINISHED，实际关节数据相同 |
| 原验证耗时／观测稳定跨度 | 406／375 ms（既有 monotonic 时钟） |
| 两秒只读尾段 | 10 组，全部 IDLE／FINISHED |
| 结果／锁存 | complete=true、stop_confirmed=true；本次 latched_fault=null |
| 进程与连接 | CLI 退出码 0、本地资源释放；退出后本机 RPC 连接数 0 |

原 100 ms 新鲜度、200／300／500 ms 停止判据和全部静止清理限制保持。退出时读取门按原 `finally` 逻辑主动关闭，结果未知标志为 false。归档核对脚本初次误将最终读取门应保持打开作为断言；核对代码后修正该断言，无产品修改、无再次连接，修正记录保存在本机。

这条既有入口的调度／验证时间使用 monotonic，额外 QPC await 计时另行记录；本报告不将两条时钟直接混算，也不据粗粒度调度记录推断物理制动上界。

## 下一项阻塞

尾段实际关节仍与清理前相同。J5 相对原准备位最大偏差仍约 0.001054612 rad／0.0604248°，略超过单次 speedl 候选的原 0.001 rad 起点容差；静止清理没有改变这项偏差。保持该门槛，不能自动改参考、放宽容差、命令回位或重新执行运动诊断。新动作需要独立范围及现场授权；本次清理授权已消耗。

已离线核对原程序回位材料：现有产品 `prepare(HomeOptions)` API 可供后续复用，但 10 月 16 日的一次性执行包装绑定旧起点、旧源码和旧授权问题编号，不能直接重用或将当前清理批准代入。后续若选择程序回位，须为当前冻结起点准备新材料；历史回位的速度审计失败仍保留，见 `ready-return-result-2026-10-16.md`。

## 本机证据

配置 SHA 与已审阅范围一致，117 个运行文件执行前后核对一致。配置、批准材料和原始日志仅保存在本机；本次未修改产品代码、运行测试集或推送。

| 文件（相对材料目录） | SHA-256 |
| --- | --- |
| `approved-preview.json` | `d2bcc9dc61c78fcbd9521c09e033f0f61940358d329e8bf752155384592a1fc3` |
| `result/result.json` | `12dfd32d084f179c115e7d362699f10572ed8be6b217f2ec6aeabec8ccdb9869` |
| `result/sessions/20261020T081432Z-ef1b2746/session.jsonl` | `60b1efc5b93800be1b1248dcefe9298785fba25dacd5c4b1daaa8fbbb4fd8e7c` |

同目录保存 `authorization.json`、`execution-attempt.json`、`process-exit.json`、`offline-analysis.json`、`onsite-closure.json`、`scope-consumed.json` 和核对脚本断言修正说明。关联的零写入运动准入拒绝见 [此前现场记录](native-jog-feedback-pair-site-2026-10-20.md)。
