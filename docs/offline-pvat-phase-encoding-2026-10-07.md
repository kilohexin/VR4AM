# PVAT 阶段内编码与移动状态改目标

2026-10-07，起点 `155500b889b5bbce9c5e3460c8ac70b7295a34dd`。本轮仅新增测试专用数学编码器、扩展注入 SDK 的停止测试，没有生产算法、参数、配置、SDK、部署或真机连接改动。未推送。此前真机 `stop_move` 610 ms 迟到仍未解决；保留 200 ms 截止与 `stop_unverified`，此前现场授权已消费。

## 阶段内部编码

[上轮](offline-pvat-cruise-supervision-2026-10-07.md)证明，限幅合格的分段参考跨阶段直接拟合一条五次曲线，窗口内部仍可能超速。因此新增 `backend/tests/robots/pvat_phase_encoding_probe.py`，在每个原始阶段内部均匀划分窗口：

```text
窗口数 n = ceil(阶段时长 / 最大窗口时长)
每个窗口时长 = 阶段时长 / n
```

声明的最小／最大时长为 20／80 ms，最多 2048 个窗口，均为实验常量，未写入现场配置或确认控制器能力。均分避免先切 80 ms 再留下几毫秒尾段；任何阶段均分后短于最小值则整批拒绝 `pvat_phase_too_short`。预算超限不返回部分序列。

原阶段必须有有限、六轴的规范化多项式系数、正时长及连续 P/V/A。先检查原阶段整段的速度、加速度和关节位置极值，再精确限制到每个窗口，并根据实际端点 P/V/A 重建五次曲线、重新检查整段极值。窗口对象与系数不可变；`sdk_arguments()` 返回独立的端点 `p/v/a/t` 列表，仅为惰性数据，没有 SDK 导入或写入。

| 固定目标 | 窗口数 | 全计划时长 | 最短／最长窗口 |
| --- | ---: | ---: | --- |
| ±.5 rad | 131 | 10.375 s | 75／79.545 ms |
| ±.001 rad | 4 | .173205 s | 43.301／43.301 ms |

四条参考每窗口采样 17 点，P/V/A 与原计划最大残差分别约 `1.11e-16 rad / 1.33e-15 rad/s / 5.37e-14 rad/s²`。独立测试另按手工积分的加速／巡航／减速公式核对 .5 rad 全部窗口，不把待测编码器当作预期值。数值峰值为 .05／.2，存在约 1e-16 至 1e-15 的浮点舍入，编码速度／加速度比较容差 1e-10，关节边界容差 1e-12；不代表已批准的物理容差。

该表达解决的是声明五次模型下、阶段内端点的数学一致性。厂家实际插补未知，也没有 jerk 限制、TCP 直线或碰撞保证。

## 窗口序列不是发送调度

现有测试监督器要求新窗口在旧窗口过期前续接，并保持续接起点的 P/V/A。新增两条反例确认门禁仍然有效：

- 第一窗口时长 75 ms，在 75 ms 原样提交第二窗口，保留 `pvat_window_expired`，不能抹掉截止。
- 在 20 ms 提前提交声明起点为 75 ms 的第二窗口，起始状态不一致，保留 `pvat_reference_discontinuous`。

因此不能按上述序列的端点时间逐条发送，也不能把未来起点改成当前时刻后提前发送。没有添加默认调度、缓冲队列或绕过门禁。连续巡航的重叠续接尚未实现；控制器接收、替换、执行起点与 SDK 时刻也未对齐。

## 从覆盖中的移动状态改目标

新增 `retarget_window`：只在已声明窗口内 `0 <= elapsed < duration` 继承预测 P/V/A，调用既有有界单段五次到点候选，再编码其窗口。不是实际反馈，不能外推过期的 80 ms 窗口到 110 ms。初始速度和加速度不清零；旧候选族无解时仍拒绝，不能声称任意状态都可规划。

合成反向案例：在匀速窗口 40 ms 处，继承 `q=.0008 rad / v=.02 rad/s / a=0`，新目标 `q=-.001 rad`。参考先保留制动行程后反向，8 个窗口、约 .596046 s 到点；采样正向最大位置 .002844777 rad，反向速度最低约 -.01555953 rad/s。制动行程不是误差清零，也不是最短制动或物理安全证明。终点数学速度／加速度归零不确认真实停止。

另核对加速参考的非零初始 a 被保留；原已知 `v=.01 / a=.1 / 目标 .1 rad` 的有界候选族拒绝仍保留。合法 40 ms 改目标首窗口可在过期前续接原监督器，要求刷新观察值，旧观察不会沿用。

## 拒绝与停止事务

扩展原 `test_pvat_window_supervision.py`，通过实际函数产生三个新增原因：短阶段、过期改目标、改目标候选族无解。每个原因分别覆盖停止确认、反馈仍运动、异常、fake 610 ms 迟到和取消，共 15 个新增组合。

全部经过原 `RobotControl._enter_fault → _stop_backend → RealLebaiAdapter.stop` 与注入 SDK；迟到仍按 200 ms 截止锁存 `stop_unverified`。未增加 Home、movej、stop_sys 或第二次停止写入；后续 fake 静止、新输入、Arm、reset 均不能恢复停止失败。network/discovery 调用为零。此次无新的现场停止测量。

## 定向验证

首批 33 项对占位实现全部红灯，原因均为缺少实现；实现后 **33 passed，3.39 s**。加入停止矩阵与续接反例后 **97 passed，6.23 s**。最终新增 **51 个案例**，10 个相关文件 **303 passed，26.71 s**，退出 0：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/robots/test_pvat_phase_encoding.py backend/tests/robots/test_pvat_cruise_candidate.py backend/tests/control/test_pvat_window_supervision.py backend/tests/robots/test_pvat_settling_candidate.py backend/tests/robots/test_pvat_ramp_candidate.py backend/tests/robots/test_pvat_interpolation_probe.py backend/tests/robots/test_lebai_pvat.py backend/tests/robots/test_pvat_point_inputs.py backend/tests/robots/test_stop_transaction.py backend/tests/control/test_braking_supervision.py -q --tb=short -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

没有后端全集、独立审查、前端构建或真机动作。本轮生产差异为空，应用未导入编码器。

本机忽略目录 `.tmp/early-stop-window-20261007-l0erdkr4/` 的分析脚本与完整输出不入 Git。只核对既有原日志、只读配置和生产构造器哈希，没有把本轮合成轨迹称为原日志闭环回放。报告按实际写入的 UTF-8 字节计算哈希：

| 项目 | SHA-256 |
| --- | --- |
| `evaluate_phase_encoding.py` | `101952078fe73df197b46982ee6a0e29c6ed6467bfbee8fa5694e27c201383bf` |
| `phase-encoding-evaluation.json` | `0fffd6fcbf7c72e54d4bf753615864fd82ad24f25c7f9a56ad289e46e16065dc` |
| 阶段编码源码 | `08a6f47cb5ecf04d7e0650d09e6186326f648a4a2882f468227951218931c67c` |
| 编码测试 | `43a4fe59025e726fb949d35258b4a31609a7a72339e1ea67e039dfe262bdb707` |
| 停止监督测试 | `ce9663a4916b4f00d0f6a17278e18f14a74372bb173b36d8827ac8e52cc696e8` |

下一步离线处理跨阶段重叠续接：从仍覆盖当前时刻的已编码窗口继承预测 P/V/A，明确保留或更换目标、检查新窗口整段及后续制动能力；不能用完整未来计划替代已发窗口覆盖。真实快照来源、执行起点和停止迟到仍是生产接入及新真机测试的阻塞。
