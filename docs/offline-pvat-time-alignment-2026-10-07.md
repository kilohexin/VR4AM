# 参考与反馈时间关系、末段观察交接（离线）

2026-10-07，起点 `5fe6a08d57e504a2a5d3920e5b5ae07005d97637`。新增工具仅位于测试目录，没有生产、配置或 SDK 改动，没有真机连接、命令、服务启动、部署或推送。现场约 610 ms 的 `stop_move` 迟到仍未解决；200 ms 截止、`stop_unverified` 和已消费的现场授权状态不变。

## 时间默认未知

上轮缓存观察保留本地读取开始／完成时间，却没有控制器采样时间或 PVAT 执行起点。不能把视图读取时刻、SDK 调用开始、SDK 返回或本地读取区间中点当成反馈的真实参考时刻。

新增测试专用 `ReferenceWindow`／`SyntheticTiming`／`compare_reference`。默认没有时间模型，拒绝 `pvat_alignment_time_unknown`。只有显式声明**合成执行起点区间、样本采样发生在本地读取区间内、q/v/a 同时采样**三项假设时才比较。这些条件尚未得到现场证据支持，不是控制器协议，也没有新增自动估计或默认延迟补偿。

条件模型得到的参考时间区间为：

```text
最早参考时间 = 读取开始 - 最晚合成执行起点
最晚参考时间 = 读取完成 - 最早合成执行起点
```

先做整数 ns 相减，保留完整区间；不选取中点或用最新视图时间刷新样本。读取映射区间超出多项式覆盖、检查时刻达到本地窗口截止、owner／完整命令不匹配时拒绝，不能外推。

在归一化时间中，对六关节 × q/v/a 的误差多项式求容差边界根，分割区间并检查边界和区间内部，寻找**全部 18 个字段共享的一个可行时刻**。逐关节、逐字段各自落入参考范围不足以通过。本工具是浮点可行性探针，求根与比较分别使用 1e-10／1e-12 数值余量，不是经过认证的区间算术、轨迹安全证明或控制器执行时间测量。

默认比较容差 q=1e-4 rad、v=1e-3 rad/s、a=.02 rad/s² 延续既有合成监督实验，仅用于测试。返回 `synthetic_time_compatibility`，其 witness 是条件模型中的可行时间，不是实测时刻。

## 缓存比较案例

声明 .02 rad/s 匀速参考，样本合成状态取 20 ms，本地读取区间 15..25 ms，检查时刻 40 ms。旧的测试 `WindowGuard` 把样本同 40 ms 参考比较，报 `pvat_reference_position_mismatch`；新条件模型在 15..25 ms 内存在共享时刻。默认容差下返回的最早可行 witness 为 15 ms，不能据此反推样本真的在 15 ms 产生。

未提供合成假设时，该例仍拒绝时间未知。分别把两个关节、或者同一关节的位置与速度取自 16／24 ms 的混合状态，均因没有共享时刻拒绝。离网格的窄可行区间、区间两端、1e18 ns 大时钟起点也有独立测试，不靠固定密度时间采样。

## 末段观察状态

新增测试专用 `TerminalFeedbackProbe`，只接受数学末端为目标、零 v/a 的有限参考窗口；没有 SDK I/O、发送器或完成通知。

- 初始 `awaiting_feedback`。符合来源门禁且通过条件时间比较的观察，才参与目标附近的 q/v/a 计数。
- 重复缓存、倒退／重叠读取区间不增加有效观察数；间隔大于合成 30 ms 或新观察仍在运动，重新累计。
- 至少两帧、起点跨度至少合成 20 ms，才报告历史观察 `feedback_consistent`。这不是物理到位、软件停止或新运动权限，也不产生 `observed_settled`。
- 同目标不延长截止；改目标锁存 `pvat_terminal_target_changed`，等待调用方的显式重新规划／停止，不能忽略新目标或把当前 v/a 清零。
- 撤销清空观察，不能重新激活该探针；已有故障不会因撤销而清除。倒退检查时钟锁存故障。
- **窗口截止优先**：即使两帧已经一致，或者刚收到匹配反馈，截止仍是 `pvat_window_expired`，进入原停止桥接。没有在覆盖外等待、补保持窗口或延长截止。

这些状态不接入生产 `WindowGuard`、`CatchUpSupervisor` 或运行策略；调用方仍需按当前上下文重新资格校验反馈，不能复用旧的检查结果作为当前权限。

## 已有末段的真实数学限制

用此前 ±.001 rad 案例的正向有限末段实际构造最后窗口，长度 62.574186 ms，不用匀速或静止替代。以下是声明参考生成的合成反馈，不是机器人日志：

| 最后窗口相对时刻 | q（rad） | v（rad/s） | a（rad/s²） | 结果 |
| --- | ---: | ---: | ---: | --- |
| 60 ms | .0009999714 | .0000330636 | -.02531594 | 加速度超过合成接近零阈值，有效计数 0 |
| 61 ms | .0009999934 | .0000125044 | -.01574668 | 有效计数 1，等待 |
| 62 ms | .0009999997 | .0000016823 | -.00584111 | 有效计数 2，仅相隔 1 ms，等待 |
| 62.574186 ms | 数学终点 | 数学零速度 | 数学零加速度 | 截止，`pvat_window_expired` |

因此这条参考不能靠当前窗口和该观察标准自动完成交接。本轮暴露了终点后的反馈等待必须有独立、明确覆盖语义的问题，没有通过降低门槛、追加小尾段或延长原截止把它标成到位。

## 定向验证

初始 31 项对占位实现全部失败；实现后 31 通过。补充边界后 **2 failed / 36 passed**：布尔 owner 在人工篡改的资格结果中可冒充整数，整数 ns 四舍五入可放行超过多项式末端 .4 ns 的比较区间。增加严格结果元数据类型检查，并将数学采样范围同实际浮点时长比较后，38 通过；再补实际有限末段和窄时间区间，40 通过。

未知时间／覆盖越界／不相容反馈三类 × 原五类停止结果新增 15 项。实际资格与时间比较错误被送入测试监督器，沿原 `RobotControl → RealLebaiAdapter.stop` 处理；确认、仍运动、异常、fake 610 ms 迟到、取消保持原停止语义和锁存，注入客户端 network/discovery 均 0。未增加 Home、movej、stop_sys、第二次停止或自动复位命令。

本轮新增 **55 个案例**，8 个相关文件 **315 passed，23.23 s**，退出 0：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/control/test_pvat_alignment.py backend/tests/control/test_pvat_observation.py backend/tests/control/test_pvat_window_supervision.py backend/tests/robots/test_motion_view.py backend/tests/robots/test_pvat_terminal.py backend/tests/robots/test_stop_transaction.py backend/tests/control/test_transactional_vr_tick.py backend/tests/control/test_catch_up_supervision.py -q --tb=short -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

没有后端全集、独立审查、前端构建或真机测试。生产目录无差异，也没有导入本轮探针。

## 本机证据与下一步

忽略目录 `.tmp/early-stop-window-20261007-l0erdkr4/` 的脚本及 JSON 不入 Git。分析退出 0，原日志、只读配置、生产 PVAT 构造器及上一轮观察分析输出哈希匹配；实际保存字节哈希如下。

| 项目 | SHA-256 |
| --- | --- |
| `evaluate_alignment.py` | `255deed1531fa58a4ef6637d423c58d0210c30d7889a6d183facc70cff4980ec` |
| `alignment-evaluation.json` | `e021842f86bc38c9db86a920e0c144294218a5aff4fb9082bf338279f4f476f2` |
| 时间比较与末段探针 | `280692d5b66cdbfc1eb84ce2564331290eb1104cbbb228effccd425644eca393` |
| 时间比较测试 | `51cb93ee7329591d643a888377a61796b59c1bd303e1e9ef79a7e185839586e3` |
| 原停止桥接测试 | `2c4c2a7b84a2c664deac055180d14e5329289d368ffa590f55efc8025767b76e` |

下一步优先离线确定有限参考完成后等待反馈的覆盖／撤销接口，并审查接入实际运行策略所需的执行与采样时间证据。当前默认时间未知、终点后无等待覆盖、厂家插补未确认和现场停止迟到均不能靠本轮探针解除。暂不进入真机 VR 或仿真物理抓取测试。

后续 [实验运行追赶的反馈等待门禁](runtime-pursuit-feedback-wait-2026-10-07.md)已接入现有 opt-in 策略；它为请求返回后等待新反馈提供有界接口，未延长本页有限参考的截止或定义控制器执行覆盖。
