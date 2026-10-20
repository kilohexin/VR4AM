# 单次 speedl 发送前核验：离线计时与拒绝复现

## 范围

本轮承接[准备位恢复后的现场零写入结果](native-jog-after-ready-site-2026-10-20.md)，仅检查原有实现。产品版本为 `88ef4b2`，运行代码未修改。采用现有完整 CLI、原 117 个运行文件的身份核验、冻结配置及参考证据，显式注入 Fake；原生 SDK 加载入口被禁止，网络 connect／DNS／bind 被审计阻断。四次 Fake 执行均无网络尝试、未导入 SDK，执行前后运行文件哈希一致。

Fake 使用冻结反馈中的关节值和工具配置，TCP／FK 由本地 MDH 纯数学生成，报告始终 SIMULATION。它没有控制器响应、物理制动、真实网络或物理运动模型，不是完整现场数据回放。合成批准文件不构成真机授权。本轮没有新的真实连接、运动、停止或抓包。

## 观测结果

| 独立 Fake 运行 | 计时包装 | 额外字段等待 | Fake speedl／stop_move | 整体结果 |
| --- | --- | --- | --- | --- |
| baseline | 无方法级计时 | 0 ms | 1／1 | complete=true |
| profiled-fast | 有 | 0 ms | 1／1 | complete=true |
| profiled-delayed | 有 | 每字段 30 ms | 0／0 | 本地快照过期；complete=false |
| profiled-delayed-repeat | 有 | 每字段 30 ms | 1／1 | finite_scope_motion_observation_not_clear；complete=false |

两个有等待的运行不是自动重试现场测试，而是单独输出目录的 Fake 检查。相同合成等待下结果不同，说明调度和剩余窗口有波动，不能把其中一次过门当作稳定持续控制的证据。第二个有等待运行的观测读取未清理问题另行保留，不归并成同一快照过期问题。

无方法级计时的 baseline 从 submit 调用到返回约 31.28 ms；这个区间包含原返回后核验，**不是纯发送前耗时**。profiled-fast 到 Fake 原生方法调用前约 25.10 ms，到提交返回约 31.93 ms。profiled-delayed 在零写入拒绝前提交区间约 25.46 ms。

profiled-fast 的发送前方法级计数如下。时间为本次观测值，不是固定上界；包含关系存在，不能把 inclusive 列直接相加。

| 检查 | 调用次数 | 累计 inclusive 时间 |
| --- | ---: | ---: |
| 输入批准／配置核对 Inputs.read | 8 | 9.61 ms |
| 其中：输入文件身份扫描 input_fingerprints | 8 | 3.79 ms |
| 全量运行文件身份扫描 fingerprints | 2 | 8.90 ms |
| 模型配置绑定 require_binding | 36 | 1.37 ms |
| 模型与反馈验证 validate | 9 | 4.10 ms |
| 已有快照预检 _preflight_from_snapshot | 3 | 0.55 ms |

两次全量身份扫描均保留现有线程池创建与等待。重复次数多不等于可以删除校验：各次检查分别位于提交、锁等待、独立调用所有权、原生代理等边界。当前没有证据表明可以直接合并跨边界的权限检查。

## 已复现的本地错误

profiled-delayed 捕获的原始本地 ValueError 为 `future or stale input/snapshot`，调用链末段为：

`SiteJogClient.speedl → motion_permission → motion_checks → validate_packet → _fresh`。

这次 Fake 明确复现了发送前新鲜度拒绝，但正式结果仍只记录 `native_jog_diagnostic_failed`。因此“具体局部校验错误被通用理由掩盖”的诊断缺口已离线复现。**不能把这个 Fake 调用位置当成前次现场已经证实的确切失败位置**；现场原始异常栈未留存。

## 未采用的优化

针对运行文件身份扫描，进行了 30 轮只读对照。每轮分别调用当前四线程池实现、顺序 stat、保留原行结构的顺序 stat，轮换调用次序；117 个文件的设备／inode／大小／mtime_ns／ctime_ns 有序结果每次均完全一致。

| 实现 | 本机热文件系统的中位耗时 |
| --- | ---: |
| 当前四线程池 | 3.34 ms |
| 顺序 stat | 5.68 ms |
| 保留原行结构的顺序 stat | 5.63 ms |

顺序扫描在本次对照中更慢，**没有将其替换进产品**。本次数据仅适用于这台机器的当时负载及文件布局，不证明实时上界，也不证明线程池在所有环境都更快。

## 后续建议及证据

优先补足本地校验的固定拒绝码、校验位置和快照／输入年龄，再做针对性优化；继续保留每个实际发送边界、原 100／60 ms、未知结果隔离及全部停止规则。不会因一个 Fake 成功或减少几毫秒就开放网页／VR 连续运动。

材料均位于 `.tmp/native-jog-after-ready-site-20261020T093755Z/`：

- `profile_existing_entry.py`：临时计时包装与显式 Fake，仅作为本机调查工具。
- 四个 `offline-profile-*/profile.json`：含原始时间点、嵌套时间汇总、本地异常栈、结果哈希及源码一致性。
- `benchmark_metadata.py`、`metadata-benchmark.json`：30 轮相同五字段身份扫描对照。

原始材料不提交。未运行后端全集，未修改现场配置、冻结日志或用户总结，未推送。
