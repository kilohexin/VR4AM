# 候选状态提交协调器：保守离线原型

## 本次范围

新增测试专用 `CandidateCoordinator`，使用上轮的入队回执和逐请求结果管理 `BrakeBeforeRetarget` 的状态副本。只修改 tests 和报告，没有接入默认 RobotControl，也没有更改适配器、速度限制、停止、Home 或复位规则。

原型解决的是**测试中的软件历史归属**：不把所有生成目标都当成已接受目标，不把旧 IK 反馈当成当前请求拒绝，不在拒绝时直接清零非零候选历史。它没有证明物理运动连续性或实际速度，仍不具备日常 VR 控制的采用依据。

## 状态规则

1. 复制已提交的候选状态，在副本上计算草案。入队、等待或反复查询都不改变正式历史。只允许一条未结束草案；新的输入不会排成待重放轨迹。
2. 仅当 completion 属于本请求、控制权限/代次和 pump 代次仍有效、未被撤销、版本一致，且结果为 `sdk_returned + returned + advance`、接受引用恰为本请求、无错误，才提交草案。提交后版本递增；重复 settle 不重复提交。
3. 覆盖、拒绝、失败、取消、catch_up、部分推进、缺少明确接受证据或标识不匹配均冻结。冻结保留原位置、速度历史和减速阶段，不调用 reset_motion，不自动发下一条请求。
4. 冻结不是机器人停止。协调器不发 stop、Home 或复位；在途结果未结束时，也不允许自行重捕获。
5. 新状态必须由拥有控制权限的一方显式建立：请求已结束、当前 ACTIVE、控制代次严格增大，并提供重捕获姿态，才能重新建立零速度历史。原型只验证这些软件条件，不能独立证明现场停止；测试通过原有控制 disarm/arm/Grip 流程建立新状态，生产接入仍需生命周期保证。

首个回执确定本次状态所属的 pump 代次。随后即使 RobotControl 代次没变，只要 pump 代次已经改变，也会在新请求入队前阻止沿用旧历史。提交期间还有独立排他标志，避免 await 边界引入第二条草案或重捕获。

## 验证结果

新增 `backend/tests/control/test_candidate_commit.py` **30 例**：

| 类别 | 证据 |
| --- | --- |
| 等待与成功提交 | 多次查询不推进历史；只提交一次；同一时刻最多一个请求 |
| 八类非完整接受结果 | 覆盖/失效/IK拒绝/失败/取消/通用handled/catch_up/部分推进均冻结，原非零历史不变 |
| 六类关联或权限异常 | 控制代次、ACTIVE状态、pump代次、句柄失效、错误接受引用、错误结果引用均不提交 |
| 生命周期与异常边界 | 未结束请求拒绝重捕获，要求更新且有效的控制代次；入队异常和候选计算错误冻结；并发、撤权和取消提交不污染历史 |
| 已接受的减速阶段 | 减速阶段一旦正式提交，后续 IK 拒绝不会把它清成零速度或退出减速阶段 |
| 六个真实适配器＋注入SDK案例 | advance、旧反馈后advance、catch_up、IK拒绝、外部pending覆盖、发送日志失败；使用原 RobotControl 权限和停止/重捕获流程 |

真实适配器案例中，旧 `ik_tracking_lag` 仍在回执里、当前请求却成功 advance 时，协调器能根据当前 completion 正确提交。与旧控制路径“当前已入队/发送却因旧反馈重置”的特征案例形成对照；旧生产路径本次没有修改。

集成夹具使用原 RobotControl 管理 ACTIVE/控制代次、disarm 和重新 Grip 捕获；目标直接交给协调器计算，未替换整段 VR tick 的映射/滤波调度。fake IK 解和关节反馈为人工序列；每轮输入 40 ms、候选计算仍用 20 ms 模型步长。结果不能解释为实际运动速度或性能测量。

测试流程：初始因模块未实现观察到 RED；首批 22 例通过。补充边界时，观察到“请求间 pump 代次变化仍可提交”“旧权限代次允许重捕获”“候选计算失败不冻结”三个失败，修复原型后 28 例通过；再加入减速阶段保留和旧反馈后成功的对照。最终 9 个相关文件 **337 passed in 15.03 s**。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/control/test_candidate_commit.py backend/tests/control/test_brake_retarget.py backend/tests/control/test_braking_supervision.py backend/tests/robots/test_command_admission.py backend/tests/robots/test_command_completion.py backend/tests/robots/test_retarget_backpressure.py backend/tests/robots/test_lebai_backpressure_experiments.py backend/tests/robots/test_stop_transaction.py backend/tests/control/test_robot_control.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

作者自审；没有独立第二审阅。未重跑后端全集，无前端改动。没有设备访问、现场配置/原始日志变更、推送或部署。

## 明确的限制与下一步

这是保守冻结策略，**不能直接用于持续 VR 跟随**。现有 catch_up 依赖不断到来的请求驱动；冻结后拒绝新请求，会中断后续追赶请求的供给，无法自动恢复。这不是已完成的 IK 背压解决方案。单未完成请求的吞吐影响、固定模型步长与真实时间间隔也尚未解决。

下一步在离线测试中分开“继续追赶已接受目标的请求”和“生成新目标的候选历史”，核对追赶退出条件与停止抢占。不能单凭 SDK 返回或观察到 IDLE 就恢复旧速度状态；没有明确连续性依据时仍冻结并交由现有停止/重捕获流程处理。真机 VR 控制继续优先，物理仿真抓取仍后置。
