# 只读运动视图与真机 VR 测试准备进度

日期：2026-10-06。接管计划第 32 项。本轮仅离线开发，所有客户端均为注入的内存 fake。没有服务探测、真机连接、部署或推送。

## 本轮实现

- `RealLebaiAdapter.read_motion_view()` 同步读取本地缓存，返回不可变 `MotionView`。没有 SDK 刷新、后台任务、日志写入或缺失数据回退读取。
- 接受状态含逐请求 `CommandRef`、接受 TCP、接受关节解。TCP 属性每次生成独立 Pose，外部修改不能改写 adapter。等待 SDK 返回时不提前推进接受状态；显式 catch_up 保留原接受引用。
- 观测含源 pump 代次、采集开始/完成时间、读取用途和实际关节位置/速度；不暴露原始 SDK 字典。源代次在等待 SDK 锁之前固定，跨代次的排队/在途读取不发布为新观测；旧缓存仍保留原代次。客户端已拆除时，迟到读取不能重新填充公开缓存。
- 视图另含本地客户端引用是否存在、pump 状态/故障、锁存故障、preflight 状态、停止锁状态。`client_attached` 不表示网络已连接；`is_current()` 仅比较 pump 运行/故障/代次，不是运动许可或遥测新鲜度判断。
- 测试专用事务策略改用此接口，不再读取 adapter 的私有 pump、接受目标或快照字段。开始追赶必须匹配 completion 中的接受引用；采样必须属于当前 pump 与追赶目标的同一代次。

这是事件循环 owner 内的同步观察接口，不提供跨线程一致性保证。旧视图是历史值，不能长期缓存后当作当前权限。读取年龄应根据 `observed_ns` 与采集时间计算，负年龄也不能当成新鲜证据。

接口保存单份最新关节观测，未增加全局请求历史。现有 `_snapshot` 与默认控制流程仍按原实现工作；本轮只约束新公开观测的发布。未更改 SDK 停止截止、运动阈值、Home 行为、故障复位白名单或默认策略。

## 验证证据

新增 20 个案例（机器人 13、策略消费者 7）：

- 冷启动只读不连接、不创建命令；不可变字段、独立 Pose、读取不刷新、年龄自然增长。
- SDK 等待期间接受状态不变，返回后才推进；显式追赶保持原接受引用和关节解。
- SDK 锁排队及正在读取两个边界的代次撤销；新一代显式读取才能提供新一代观测。
- 停止、断连、重连（包含复用同一个 fake 对象）；断连后的迟到读取；连接日志失败的缓存清理。
- pump/锁存故障可读且不清除门禁；停止等待可读；成功、异常及 609 ms 迟到停止均保留原行为，异常/迟到仍为 `stop_unverified`，无 `stop_sys`。
- 实验策略拒绝缺失、旧代次、未来时间和过期观测；接受目标缺失或变更不能生成追赶计划。

先运行新增接口案例，8 项因接口尚不存在而失败。首次接入后发现 3 处测试夹具使用问题（IK 队列字段名、StopReason 枚举、模拟锁存时未同步 preflight 状态），按既有接口修正；没有为测试放宽生产门禁。扩大后 49 项通过。最终 18 个相关文件 **564 passed in 14.06s**；未运行后端全集。由主执行者自查差异，没有独立审查结论。

复现命令（仓库根目录，PowerShell）：

```powershell
.venv/Scripts/python.exe -m pytest `
  backend/tests/robots/test_motion_view.py `
  backend/tests/control/test_transactional_motion_view.py `
  backend/tests/control/test_target_lifecycle.py `
  backend/tests/control/test_target_strategy.py `
  backend/tests/control/test_transactional_vr_tick.py `
  backend/tests/control/test_candidate_commit.py `
  backend/tests/control/test_robot_control.py `
  backend/tests/control/test_input_await_boundaries.py `
  backend/tests/control/test_gripper_frame_freshness.py `
  backend/tests/control/test_catch_up_supervision.py `
  backend/tests/control/test_braking_supervision.py `
  backend/tests/robots/test_stop_transaction.py `
  backend/tests/robots/test_retarget_backpressure.py `
  backend/tests/robots/test_explicit_catch_up.py `
  backend/tests/robots/test_command_admission.py `
  backend/tests/robots/test_command_completion.py `
  backend/tests/robots/test_lebai_adapter_control.py `
  backend/tests/robots/test_lebai_adapter_readonly.py `
  -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

## 距离真机 VR 运行测试

当前处于**控制策略离线集成阶段**，尚未达到可直接进行真机连续 VR 运行测试的状态。不能按测试数量给出可靠完成百分比，也没有依据承诺真机调试耗时。

| 工作 | 当前证据与剩余事项 |
| --- | --- |
| VR 输入与夹爪软件链路 | 原 tick 的映射、滤波、工作空间、输入过期/释放、Trigger 开合已有定向 fake 测试；未代表真实头显、WebXR 或设备体验已验收。 |
| 命令接受、背压与追赶 | 逐请求回执/completion、显式追赶、策略及生命周期接口、只读观测接口已实现；完整 tick 实验已使用正式接口。 |
| 可运行事务策略 | 仍在 tests 中；默认仍为旧策略，候选历史与实际接受不一致的问题未在默认运行路径修复。需要搬入运行模块、用有界状态替换测试 trace，并提供明确选择入口和参数验证。 |
| 启动与会话验收 | 需在 fake 后端完成配置选择→真实 WebSocket 会话/循环→输入与夹爪→撤销/断连/故障的组合验收，形成可复现启动方式及首轮测试版本。当前完整 tick 实验不能代替这些组合验收。 |
| 运动中的停止及时性 | 尚未得到真机证据。冻结日志记录 SDK await 609 ms，超过 200 ms 截止；离线保留迟到记录和 `stop_unverified`，未证明 SDK/网络/控制器根因已解决。 |
| 真机测试 | 本地接管开发至今未执行。机器人电源关闭是用户报告；本轮未验证现场收尾状态。新的连接、动作、停止诊断等均需用户另行授权具体范围。 |

### 下一步顺序

1. 将事务候选提交、追赶监督和减速候选整理为正式运行策略，明确非默认选择与有界状态；移除测试模块依赖。500 ms 等实验输入不能未经论证直接成为现场 profile。
2. 完成 fake 启动/会话组合验收和配置冻结，交付具体版本、启动方式、预期日志与现场测试清单。继续优先真机 VR 控制，物理仿真暂排后。
3. 软件材料齐备后提交首轮现场范围供用户决定：先现场收尾/状态核对，再独立受限停止诊断，再短程 VR 跟随及夹爪开合。范围应明确工作空间、起始姿态、幅度/速度、现场操作人和中止条件；不自动 Home、复位、恢复旧速度或重试。
4. 停止未确认时不扩展为连续 VR 抓取测试。物体抓取还需机械夹持、工具参数及物体条件的现场验证；Web 物理仿真不代替这些证据。

上述现场顺序是待批准建议，未执行。当前无需用户通电或改变现场配置，继续离线完成前两组软件工作。
