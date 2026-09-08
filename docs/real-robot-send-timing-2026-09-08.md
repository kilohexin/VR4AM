# 真机发送节奏：分段计时诊断

## 当前结论

905e21a 的 +x 5 mm 复测通过，但 23 个成功 PVAT 调用起点间隔全部超过 80 ms，范围 93～219 ms，平均约 138 ms。该结果来自 smoke，不可直接等同于 VR 运行时性能。

当前代码可见：发送任务每次重新读取状态；smoke 的控制 tick 与显式 get_state 也可能增加状态读取，均与 IK、PVAT 共享 SDK 锁。仅靠原有日志不能确认各项耗时占比。

## 本次只补诊断，不优化运动行为

保留轨迹时长、循环频率、状态新鲜度要求、IK 路线、速度与安全阈值，不并发调用 SDK，不复用未确认的新鲜快照，不跳过安全检查。

### pvat_sent.timing_ms

| 字段 | 含义 |
|---|---|
| snapshot_read | 发送任务读取快照的总耗时，包含读锁等待、SDK 读取、解析及该次状态记录 |
| snapshot_lock_wait | 上项中读 SDK 锁的等待耗时，是子集，不要重复相加 |
| command_lock_wait | 快照完成后等待 IK／发送锁的耗时 |
| candidate_selection | 获取锁后到下发前的候选计算总耗时，包含 IK、可能的多次候选重试及限幅，不是纯 SDK IK 耗时 |
| send_sdk | move_pvat 调用耗时 |
| handler_to_send_complete | 处理请求起点到发送调用完成，不包含该事件自身记录及其后的调度 |
| snapshot_age_at_send | 发出调用时距状态快照采样起点的时间 |

保留 `previous_pvat_gap_ms`。成功发送之间的间隔还可能包含等候新目标、发送后的日志处理、调度与失败候选轮次；不能把未解释的时间全部称为 IK 耗时。当前分段计时仅随成功 pvat_sent 输出，失败路径仍看已有故障与候选拒绝日志。

### robot_kinematics

新增 `snapshot_lock_wait_ms`；结合原有 `sdk_latencies_ms` 可以区分排队和各次 SDK 读取耗时。未额外发起任何 SDK 探针调用。

### smoke_cycle_timing

平移／旋转循环新增普通队列事件，带 `seq`、`tick_ms`、`sleep_ms`、`state_read_ms`、`cycle_to_state_ms`。
cycle_to_state_ms 不含后续判定及该事件自身写入，不应与 PVAT 并发任务的时间简单求和。

## 下一轮建议

后续已批准的 A/B 分段安排见 [下一次实验室测试清单](lab-next-session-checklist-2026-09-08.md)，该清单扩展本节的单次诊断范围，仍不允许自动连续运动。

先完成本机回归并同步版本。在现场人员确认条件、已有预检与 prepare 全部允许后，仍只复测准备位后的单次 +x 5 mm；不扩大幅度或追加方向。
返回完整 session 与退出后 readonly 预检，主机端按分段的中位数／高分位及超长样本分析，再决定是否减少重复读取、调整任务调度，或需要进一步细化 IK 计时。

本次诊断不是发送频率已修复的声明，不代表授权直接对真机发动作。

## 本机验证

注入 7 ms 状态读取、11 ms 候选 IK、5 ms 发送延迟，分段计时与总计 23 ms 一致；SDK 锁实际竞争的 9 ms 等待也能独立记录。
完整后端回归：666 项通过，90.45 秒。尚无该诊断版本的真机数据。
