# 固定 20 ms 步长的离线节奏对照

## 实验范围

本轮只增加测试和合成结果，不修改生产逻辑、运动参数或现场配置。机械臂保持断电；没有 SDK 实例、设备连接、服务启动、部署和 push。Home 仍按用户本人误触处理，`stop_unverified` 与 200 ms 停止 RPC 截止不变。

使用真实 `RobotControl.tick`、`CoordinateMapper`、`PoseFilter`、`SafetyLimiter` 和 `PvatPump`，接内存 `FakeBackend`。pump handler 仅等待假时间并保存目标，不做 IK、PVAT 数值计算、关节反馈或机械运动。因此表中“处理”是 handler 处理目标，不能称为机器人执行或 TCP 位移。

固定条件：0.2 平移增益；沿 x 方向在 1 s 内从 0 增至 30 mm 的手部轨迹，再保持至 2 s；低速上限 5 mm/s，复用既有低速限幅夹具。姿态不变，工作空间不构成约束。每个被消费的新帧仍使用生产常量 20 ms。初始 Grip 捕获锚点，不计作推进帧。

变化条件：

- 新输入间隔 20/27/40 ms，即 50/约 37.037/25 Hz。27 ms 为合成周期，不是冻结日志的实际重放。
- 显式调用 tick 的间隔 10/20/40 ms。没有运行 `RobotControl.run`，不能据此说明生产循环的 overrun 行为或把 10/40 ms 当作可部署配置。
- pump 配置周期 40 ms，handler 耗时 12/53/80 ms。假时间分辨率 1 ms，同刻顺序为到期 handler 等待完成、输入发布、控制 tick、新 handler 调度。
- 2 s 后额外推进 200 ms 只排空发送槽，不再调用 tick；该阶段不计入控制窗口，不能当作 ACTIVE 允许过期输入的证明。暂停/过期由另组测试覆盖。

## 目标推进结果

下表保持 tick=20 ms、handler=12 ms。位置均为相对锚点的控制目标 x 增量：

| 新输入间隔 | 2 s 内消费帧数 | 累计固定积分步长 | 1 s 时滤波目标 | 1 s 时限幅目标 | 2 s 时限幅目标 |
|---|---:|---:|---:|---:|---:|
| 20 ms | 100 | 2000 ms | 5.930746 mm | 4.424 mm | 6.000 mm |
| 27 ms | 74 | 1480 ms | 5.900507 mm | 3.124 mm | 6.000 mm |
| 40 ms | 50 | 1000 ms | 5.861492 mm | 1.924 mm | 4.424 mm |

1 s 时 27 ms 组最后输入在 999 ms，对应映射目标 5.994 mm；其余两组为 6 mm。到 2 s 时三组映射目标均为 6 mm，滤波目标均近似 6 mm。25 Hz 组的 4.424 mm 主要体现本实验限幅状态按较少的新帧推进，不是映射增益发生改变。

每次提交的正向位移均不超过 `5 mm/s × 0.02 s = 0.1 mm`。25 Hz 组观察到的最大提交增量/实际提交间隔为 2.5 mm/s；50 Hz 组为 5 mm/s。27 ms 输入在控制 tick 上离散消费，不能把所有实际提交间隔视作恒定 27 ms。

将 tick 从 20 ms 缩短为 10 ms，三组消费数量、累计步长和最终目标分别保持一致；重复帧不会再次推进滤波/限幅。将 tick 延长为 40 ms，三组都只消费 50 帧，最终目标均为 4.424 mm：50 Hz 输入被最新帧槽覆盖 50 次，27 ms 输入覆盖 24 次，25 Hz 输入没有覆盖。这里的覆盖有完整合成事件证据，不能套用为历史日志的丢包数量。

这表明固定步长与新帧消费节奏确实可以影响有限窗口内的目标推进，但影响不是恒定比例：27 ms 组在保持阶段仍追到 6 mm，25 Hz 组在本窗口尚未追到。不能将累计步长比例直接转换成现场位移损失比例。

## 发送耗时与覆盖

保持 tick=20 ms。以下计数包含窗口结束后的发送排空；括号内为已提交但在待发槽被新目标覆盖的数量：

| 输入间隔 | handler 12 ms | handler 53 ms | handler 80 ms |
|---|---:|---:|---:|
| 20 ms | 51（49） | 39（61） | 26（74） |
| 27 ms | 51（23） | 38（36） | 26（48） |
| 40 ms | 50（0） | 38（12） | 26（24） |

同一输入节奏下，三种耗时的完整控制提交目标序列逐项相等；慢 handler 减少中间目标的处理次数，排空后仍处理到最后提交的目标。这个结论依赖内存后端没有 IK 背压、共享 SDK 锁或软约束返回，不能推断真实适配器也不受发送耗时影响。此前[背压实验](offline-backpressure-experiments-2026-10-02.md)仍是单独证据。

pump 的 40 ms 是 deadline 调度周期，不保证所有相邻 handler 起点至少相隔 40 ms。请求在空槽等待后才到达时，后续 deadline 可能使相邻起点间隔缩短；本实验 12 ms handler 组最小可见间隔为 20/21 ms。53/80 ms 组仍保持 handler 串行不重叠。初版新增断言误以为跳过首个间隔后均至少 40 ms，导致 5 项失败；检查 `_run` 的 deadline 和空槽等待后改为验证串行不重叠并输出最小间隔，没有修改生产代码。

## 暂停与过期边界

五个案例在已推进一帧之后，分别将同一帧保留 20/99/100/101/250 ms：

- 所有重复帧均不新增目标或滤波调用。
- 20/99 ms 后补一个新帧仍仅使用 20 ms，位移不超过 0.1 mm；没有根据暂停时长补走一大步。
- 100/101/250 ms 触发原 STALE 停止路径；随后仅补新 Grip 按下帧不会自动恢复 ACTIVE 或追加目标。这只是内存后端状态机验证，不证明物理停止。

## 验证与复现

新增 **23 个案例**：9 个输入/tick 组合、3 个重复 tick 对照、6 个发送耗时对照、5 个暂停边界。首轮 20 项通过；扩充观察与对照后遇到上述 5 项错误假设，修正断言后 23 项通过。15 个唯一参数组合已保存为[合成结果 JSON](offline-input-cadence-results-2026-10-04.json)，重复运行的相同组合输出逐项核对一致。

相关控制、映射、滤波、限幅、记录、pump、请求生命周期、背压、参数及停止事务定向回归 **292 passed（20.29 s）**。未跑后端全集；无前端改动，未重跑前端。主代理自查：完整目标序列、帧与待发覆盖分别计数、假时间调度、任务收尾、结果表与输出一致性；不记作独立第二审查人通过。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/control/test_input_cadence_experiments.py -q -s -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
.venv/Scripts/python.exe -m pytest backend/tests/control/test_input_cadence_experiments.py backend/tests/control/test_robot_control.py backend/tests/control/test_state_machine.py backend/tests/control/test_coordinate_mapper.py backend/tests/control/test_filters.py backend/tests/control/test_safety.py backend/tests/control/test_recording.py backend/tests/robots/test_lebai_pump.py backend/tests/robots/test_pvat_request_lifecycle.py backend/tests/robots/test_vr_profile_experiments.py backend/tests/robots/test_lebai_backpressure_experiments.py backend/tests/robots/test_stop_transaction.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

下一项离线工作：在测试专用对照中比较固定步长与有上限的实际消费间隔，覆盖抖动、长暂停、重新捕获锚点及限速/限加速度；先判断收益和风险，再决定是否有理由修改生产积分。真实 SDK 的 609 ms 停止 await 归因仍未解决；不据此修改停止策略或部署新参数。
