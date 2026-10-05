# 减速换向候选与 IK 背压：目标历史核对

## 结论

**当前候选不能直接接入默认控制。** 在原始 RobotControl 与 RealLebaiAdapter 的接口语义下，生成目标、入队目标、IK 接受目标、SDK 调用目标并不等价。新增 14 个离线案例复现了以下状态不一致：

1. 适配器 `command_tcp()` 先 submit，再抛出上一轮的 `_constraint_error`；控制层把这一返回作为约束处理，保留旧 `last_target` 并调用 `limiter.reset_motion()`。因此当前命令仍在队列里，但候选的非零速度甚至正在执行的减速阶段已被清除。
2. 第一次异步 IK 拒绝或 catch_up 尚未反馈时，控制层已推进 `last_target` 和候选历史；实际接受目标仍是旧目标。
3. pump 的 latest-wins 覆盖可跳过中间目标，候选却已经为这些未发送目标积分。其逐步预算不能直接当作 SDK 发送序列的预算。

这些是接口与有状态候选的兼容性证据，不是实测物理加速度、停止距离或新的现场事故。既有控制逻辑原本就采用约束后重置的策略；本次未将测试中的状态不一致直接定性为真机失控，也未修改该策略。

## 实现与边界

- 新增 `retarget_backpressure_probe.py`：复用注入 SDK 的 `Experiment`，运行原始控制、映射、滤波、pump 和适配器；仅用测试子类记录候选生成与 reset 边界。
- 生产 LEBAI 选择分支使用显式假客户端；连接工厂为 AsyncMock，断言网络/发现调用为零，finally 释放 IK 等待并结束 fake 连接。
- 以固定关节反馈与人工 IK 解序列构造 advance、catch_up、三次 ik_joint_jump 和恢复，不模拟 TCP/关节之间的真实运动学。
- 常规输入间隔 40 ms，候选仍由真实控制传入固定 20 ms；覆盖 pending 的阻塞场景增加三帧 20 ms 输入。手动时钟和 yield-only pump sleep 用于确定性排序，不是实际调度性能测量。
- 快照通过公开 get_state 刷新；阻塞 IK 占锁期间只用未过期缓存，没有放宽状态新鲜度、限速、200 ms 停止截止或恢复权限。

## 按命令编号核对的结果

14 例为下列 7 个场景各覆盖平移、旋转：

| 场景 | 观察结果 |
| --- | --- |
| catch_up 反馈 | 热身接受至 #16；#17 请求返回后控制层先推进，pump 使用 #16；#18 已入队但收到旧 ik_tracking_lag，控制层保留 #17 并清零速度，pump 仍使用 #16 |
| 下一条恢复成功 | #18 收到旧错误后控制层仍停在 #17；pump 后续成功 advance 并接受 #18；候选速度仍为零。#19 才重新推进本地目标 |
| 真正 IK 拒绝 | #17 的三个候选全部 ik_joint_jump，无新 PVAT；控制层已经记住 #17。#18 后续可成功接受，但因旧错误再次清零本地历史 |
| pending 覆盖 | 生成 #17、#18、#19、#20；阻塞解除后只发送 #17、#20，#18/#19 没有 SDK 调用，但生成器已处理其历史 |
| 减速阶段丢失 | #18 反向生成时已进入 braking；返回旧约束后 reset 清除 braking，期间没有 stop_move |
| 背压中释放 Grip | 已在途 IK 与 pending 均撤销；只增加一次 stop_move；模拟确认后重新 Grip 捕获锚点并从零历史开始 |
| 释放停止失败 | 同样撤销在途/pending；FAULT + stop_unverified，Arm 拒绝，没有自动 stop_sys、Home 或新增 PVAT |

平移场景 #18 旧反馈将历史从 0.005 m/s 清为零；旋转从约 0.030 rad/s 清为零。反向减速场景原生成速度为 0.0046 m/s 或 0.028 rad/s，生成时 braking=true，约束处理后均为零且 braking=false。这是**候选内部状态重置**，不能解释为机器人已经停止，也不能直接计算为真实关节加速度超限。

完整 14 组摘要见 `offline-retarget-backpressure-results-2026-10-05.json`。`accepted_id` 是适配器最后接受的目标编号；`current_id` 是控制输入编号。相同 Pose 或空值相等不代表时间线同步，所以同时检查编号、请求/接受 Pose、发送事件及控制模式。断连清理会重置接受历史，不把清理后的 null 误当作运动时接受目标。

## 验证

测试先因新夹具未实现而无法收集；实现后第一轮 8 例通过、1 例因测试误用 ManualClock.advance_ms 失败，改为已有的 ControlClock 单位适配后 12 例通过；补充减速阶段被清除的两例后 **14 passed in 2.11 s**。这些是针对现有行为的特征测试，断言问题确实存在；通过不意味着问题已修复。

最终 10 个相关测试文件合计 **412 passed in 16.60 s**；未运行后端全集或前端构建。相关回归命令：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/robots/test_retarget_backpressure.py backend/tests/robots/test_lebai_backpressure_experiments.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_lebai_pump.py backend/tests/robots/test_stop_transaction.py backend/tests/control/test_brake_retarget.py backend/tests/control/test_braking_supervision.py backend/tests/control/test_robot_control.py backend/tests/control/test_input_await_boundaries.py backend/tests/control/test_gripper_frame_freshness.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

作者自审：逐一核对入队与异常的顺序、旧错误来源、当前命令编号、IK 接受编号和发送事件；未进行独立第二审阅。仅本地保存，没有生产代码或参数变更、推送、部署、设备访问。

## 下一步实现方向

先把“当前目标入队结果”与“先前异步 IK 反馈”明确分开，并让反馈带所属命令/控制代次，补齐覆盖、失效、接受、发送的状态契约，再评估候选历史如何提交。不能简单把 submit 移到旧错误检查之后：那会阻断恢复输入，且当前 catch_up 依赖新请求驱动。也不能把“成功入队”视作“成功运动”，或直接删掉 reset 让候选持续向前积分。

下一步先实现不改变默认运动行为的结构化入队回执及关联测试；再用明确的结果评估候选状态的提交/撤销。保持停止、故障、复位与 Home 边界。仿真物理抓取仍在真机 VR 控制之后。
