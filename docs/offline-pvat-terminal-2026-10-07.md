# 有限末段参考：消除重拟合导致的目标越过

2026-10-07，起点 `98162ea6527cde652791bb374c756da2779bd320`。本轮仅修改测试目录和报告，没有生产算法、参数、配置或 SDK 改动，没有真机连接、部署或推送。现场 610 ms `stop_move` 迟到仍未解决；保留 200 ms 截止与 `stop_unverified`，此前现场授权已消费。

## 原因与处理

[上轮滚动窗口](offline-pvat-overlap-2026-10-07.md)在窗口末端跨过计划终点后，仍向终点零 v/a 重拟合一条固定 80 ms 五次曲线。继承的当前 P/V/A 与延长后的末段不一定单调，产生近目标约 3.90e-6 rad、远目标约 9.26e-7 rad 的内部越过。

新增测试专用 `backend/tests/robots/pvat_terminal_probe.py`：当参考剩余时长在 20..160 ms 内，从仍覆盖当前时刻的旧窗口继承 P/V/A，锁定一条到目标、末端零 v/a 的有限五次曲线。保留原计划的绝对终点（取最近整数 ns），不重新推迟到点，也不清零当前速度／加速度。

整条末段必须通过内部速度方向、位置范围、速度／加速度峰值及关节边界检查。速度方向检查使用导数极值，不靠有限采样；原始状态已反向、该曲线族不能满足限幅或单调性时明确拒绝。比较容差：单调位置／速度 1e-12，速度／加速度限幅 1e-10，与先前数学工具一致；不是现场物理容差。

后续窗口只精确截取同一条末段多项式，并用实际端点字段重建、重新检查；窗口最长仍 80 ms。窗口末端及续接状态保留原数学制动分支检查。目标改变时拒绝 `pvat_terminal_target_changed`，不能忽略新目标继续旧末段；重新规划／停止仍是未接入运行策略的调用方事项。

## 最后窗口保留原截止

当新窗口已覆盖完整末段终点，后续覆盖内检查返回 `refresh=False`，保留原窗口对象、起点和截止，不追加小尾段、不向未来重拟合保持。该标记只表示“数学参考仍在已声明窗口内”，不是 SDK 执行确认或真实到位。

近目标案例在 120 ms 生成 62.574186 ms 的最后窗口；140／160／180 ms 三次检查均保留它，180 ms 时剩余 2.574186 ms，没有发送该长度的新窗口。若未覆盖终点的旧窗口在 175 ms 才续接，所需新窗口短于声明 20 ms 最小值，仍拒绝 `pvat_terminal_short_window`。

到绝对终点 182.574186 ms，`WindowGuard` 仍返回 `pvat_window_expired`，末段续接仍拒绝 `pvat_terminal_time`。没有新增到位复位、延长覆盖或解除故障的分支，也没有将数学 v/a 归零视为物理静止。

## 合成对比

保留上轮显式合成参考 .045 rad/s／.18 rad/s²，实际候选检查上限仍 .05／.2；未修改现场 profile。滚动参考接有限末段的全部声明窗口内部位置极值结果：

| 固定目标 | 锁定末段时刻 | 末段时长 | 最后窗口 | 上轮越过量 | 本轮越过量 |
| --- | ---: | ---: | ---: | ---: | ---: |
| ±.001 rad | 40 ms | 142.574186 ms | 62.574186 ms | 3.90019e-6 rad | 约 1.52e-18 rad |
| ±.5 rad | 11340 ms | 146.111111 ms | 66.111111 ms | 9.25668e-7 rad | 0（数值结果） |

每条末段均生成 5 个窗口、保留最后窗口 3 次；加上此前滚动窗口，近目标各 7 个、远目标各 572 个。本轮越过量在声明 1e-12 数值容差内；只证明这四条合成参考及末段曲线，不保证任意状态／目标、控制器轨迹或碰撞安全。

近目标整串窗口 v/a 峰值约 .01092139／.18043816，远目标 .04507433／.18，均在 .05／.2 内。最大交接 P/V/A 残差约 2.23e-14，末段窗口相对完整末段采样残差约 3.02e-14；初始非零 v/a 均保留。SDK 插补语义、执行起点、真实快照来源及窗口末端实际到位仍未确认。

## 定向测试与失败记录

初始 26 项对占位实现全部红灯。实现后 24 通过、2 项远目标采样因 `.08 + .066111111` 得到 `.14611111100000002`、超出完整末段约 2.78e-17 s 而失败。定位后，测试先断言仅有 1e-12 以内的求和舍入，再把参考采样夹到该数学末端；未放宽窗口时间门禁或更改生产代码。随后 **26 passed，14.46 s**。

停止桥接新增短末段窗口、目标改变、非单调末段和实际最后窗口截止四类触发 × 原五类停止结果，共 20 项。经过原 `RobotControl._enter_fault → _stop_backend → RealLebaiAdapter.stop`，注入 SDK 的 network/discovery 均为 0。确认、反馈仍运动、异常、fake 610 ms 迟到、取消均保留原 200 ms 规则；停止失败后后续 fake 静止、新帧、Arm、reset 不清除门禁，没有新增 Home／movej／stop_sys／第二次停止写入。最后窗口截止案例只是合成窗口相对 fake 时钟的注入，不是现场采样。

本轮 **46 个新增案例**，12 个相关文件 **394 passed，59.91 s**，退出 0：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/robots/test_pvat_terminal.py backend/tests/robots/test_pvat_overlap.py backend/tests/robots/test_pvat_phase_encoding.py backend/tests/robots/test_pvat_cruise_candidate.py backend/tests/control/test_pvat_window_supervision.py backend/tests/robots/test_pvat_settling_candidate.py backend/tests/robots/test_pvat_ramp_candidate.py backend/tests/robots/test_pvat_interpolation_probe.py backend/tests/robots/test_lebai_pvat.py backend/tests/robots/test_pvat_point_inputs.py backend/tests/robots/test_stop_transaction.py backend/tests/control/test_braking_supervision.py -q --tb=short -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

没有后端全集、独立审查、前端构建或真机测试。应用未导入此末段候选，生产差异为空。

## 本机证据

忽略目录 `.tmp/early-stop-window-20261007-l0erdkr4/` 中完整脚本／输出不入 Git。分析退出 0，原日志、只读配置、生产构造器与上一轮冻结分析哈希匹配；输出按实际 UTF-8 文件字节计算哈希。本轮不把合成参考称为原日志物理闭环回放。

| 项目 | SHA-256 |
| --- | --- |
| `evaluate_terminal.py` | `2bf840105463c22719ddb7969bdad5dda0e52c56e5ab2cb1052873d9adc33c6c` |
| `terminal-evaluation.json` | `2d71f10ad97018aad8a1e6a8d3f0bf7517dbe495f036e59cc532959c35ff8a60` |
| 末段源码 | `ee8bd4ae4d8553432394e11fb4d9d4fb6cf4ac5037b15fb5987552839183624a` |
| 末段测试 | `dfd827e53c3f7bf0a9830da7e48a42cd23308696c3d42cce685a18a0ed5d8d54` |
| 停止监督测试 | `33f4543e5b7b66f505587f641fb7025bf3a4f39fe496ac5dd0a2c6273d7317eb` |

下一步离线明确参考预测与真实只读快照的来源、帧龄及代次，并处理末段到点确认／输入改目标的状态交接。当前数学候选不能替代运行控制器，终点到位、厂家插补和 610 ms 停止迟到仍阻塞真机测试；不依据本轮结果恢复运动授权。
