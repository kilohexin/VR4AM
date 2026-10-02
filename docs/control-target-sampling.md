# 控制目标分阶段采样（离线验证，2026-10-02）

## 目的与范围

冻结现场日志缺少映射、工作空间投影、滤波和动态限幅之间的中间目标。本次在 `RobotControl` 增加 `control_target_sample`，为未来获准采集的日志和当前合成测试提供这些字段。它无法补回旧日志缺失的数据，也不能独立解释真实 IK 分支、关节跟踪或停止延迟。

当前机械臂已由用户关闭电源。本次只修改隔离开发副本并运行 fake/合成测试；没有连接机械臂、启动服务或部署。运动算法、增益、限幅和停止策略保持原值。

## 记录字段与语义

| 字段 | 含义 |
|---|---|
| `session_id`、`command_id` | 输入会话及 VR 帧的 `seq`；同一帧传给后端的命令编号 |
| `control_generation` | 控制代次；用于区分停止、断连等边界 |
| `frame_received_ns` | 此帧在服务端的接收时间 |
| `server_mono_ns` | 构造本次样本的服务端单调时钟时间 |
| `control_dt_s` | 本次滤波和动态限幅使用的步长，目前为 0.02 s；不是相邻日志的实际时间间隔 |
| `sample_period_ns` | 采样最小间隔，目前为 100000000 ns |
| `rate_limited_frames` | 上次采样尝试以来，因采样间隔跳过的新 ACTIVE 帧数；不是网络丢包数或写盘丢弃数 |
| `translation_scale`、`hand_pose` | 当前平移增益和输入右手位姿 |
| `previous_target_tcp` | 控制层计算下一目标时使用的上一目标；不是实际 TCP，也不是后端最终接受的 IK 解 |
| `mapped_tcp` | 手部相对位姿经过坐标映射后的 TCP 请求 |
| `workspace_tcp` | 工作空间投影返回的目标 |
| `workspace_constrained`、`workspace_hold` | 是否触发工作空间约束，以及是否直接保持不发送 |
| `filtered_tcp` | 滤波后目标；工作空间 hold 时为 `null` |
| `limited_tcp` | 动态限幅后准备提交的目标；工作空间 hold 时为 `null` |

`workspace_hold=true` 时，`workspace_tcp` 可以仍包含未被允许的请求位姿，不能当作安全可执行目标。`limited_tcp` 是**提交前的请求**：后续可能因输入更新、停止、断连、过期、IK 拒绝或泵覆盖而不执行。

分析时按同一会话与控制代次，结合 `vr_frame`、后端 `pvat_sent` 的 `command_id`、`accepted_target_command_id`、`pvat_mode` 和时间窗口关联；后端历史事件未必有全部会话字段，编号重复时不能仅按整数编号关联。实际运动仍需另查遥测及其新鲜度，不以请求日志代替执行证据。

## 开销、队列及失败行为

- 只在 ACTIVE 且收到新帧时采样；捕获锚点的首帧不发送目标，也不生成目标样本。
- 单个控制实例采样间隔至少 100 ms，串行控制循环最多 10 Hz；其余控制帧继续按原有逻辑处理。
- 沿用 `CommissioningRecorder` 有界普通队列，不创建新的磁盘写线程、缓存或后台任务。普通队列满时允许丢弃，并由现有 `dropped_normal_events` 总计报告。因此日志是采样记录，不是完整重放轨迹。
- `RecorderUnavailable` 在提交 TCP 目标前进入既有 recording fault / stop 路径。它与普通队列满时丢弃样本的行为不同。
- 记录接口可能让出执行权；恢复后重新检查控制代次、模式、故障、关闭/恢复状态、最新输入和 100 ms 输入有效期。失效目标不发送；同代次下撤回未发送目标造成的滤波/限幅内部推进。
- 采样不占用关键事件队列；队列满的测试仍保留 `stop_confirmed`。

原始位姿仅写入已有本地记录系统，不上传 GitHub。将来真实环境中的 CPU/写盘开销仍需在用户批准范围内测量，不能把单元测试耗时当作硬实时保证。

## 合成对照

下表为新 ACTIVE 锚点之后第一个 20 ms 步长的独立合成输入。测试参数：平移增益 0.2，滤波截止 8 Hz，线速度上限 0.005 m/s、线加速度 0.02 m/s²，角速度上限 0.1 rad/s、角加速度 0.2 rad/s²。所有值都是测试输入，不是已批准的现场 profile。

| 输入 | 映射后 | 滤波后（约） | 动态限幅后（约） |
|---|---|---|---|
| 纯平移 30 mm | 6 mm / 0° | 3.804 mm / 0° | 0.008 mm / 0° |
| 纯旋转 10° | 0 mm / 10° | 0 mm / 6.341° | 0 mm / 0.004584° |
| 平移 30 mm + 旋转 10° | 6 mm / 10° | 3.804 mm / 6.341° | 0.008 mm / 0.004584° |

滤波系数为 `1 - exp(-2π × 8 × 0.02)`，约 0.6341。首步从零速度开始，因此平移步长受加速度限制为 `0.02 × 0.02² m`，角步长为 `0.2 × 0.02² rad`。这是首步结果，不能外推整个四秒运动始终只有这一速度，也不能用来分摊旧现场日志的位移损失。

## 定向验证

新增 14 个参数化测试案例，覆盖：三种合成动作、workspace hold/clamp、采样限频、记录器不可用、记录等待中的 disarm/断连/新帧/过期、IK 跟踪拒绝，以及实际 JSONL 队列的正常写入/满队列。

最初 12 个新增案例在实现前全部失败（缺少记录，含等待记录入口的超时），实现后通过；之后增加真实本地记录器队列的两个案例。控制、坐标映射、滤波、安全限幅、记录器、诊断包装器的定向回归：186 passed。没有运行后端全集，也没有调用真实 SDK。

最终追加停止事务、停止诊断、停止证据与超时生命周期的 47 项回归，合计 **233 passed**；`git diff --check` 通过。前端源码未改动，本轮未重复构建前端。

命令（backend 目录，使用隔离环境 Python）：

```text
python -m pytest tests/control/test_robot_control.py tests/control/test_coordinate_mapper.py tests/control/test_filters.py tests/control/test_safety.py tests/recording/test_commissioning.py tests/diagnostics/test_recorder.py -q -o asyncio_default_fixture_loop_scope=function
python -m pytest tests/robots/test_stop_transaction.py tests/robots/test_lebai_stop_diagnostics.py tests/robots/test_lebai_stop_evidence.py tests/commissioning/test_stop_timeout_lifecycle.py -q -o asyncio_default_fixture_loop_scope=function
```

后续：以这些阶段字段为基础，用 fake 慢反馈/分支变化分析背压冻结与恢复。真实 IK 背压根因、正常 VR 参数与停止 SDK 晚返回仍未解决。
