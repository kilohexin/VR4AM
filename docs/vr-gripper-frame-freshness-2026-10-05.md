# VR 夹爪调用返回后的旧输入校验

## 用户优先级与运行范围

用户明确要求先完成真机 VR 控制和夹爪抓取链路，再引入 Web 物理引擎处理仿真平台浮空与动画抓取问题。当前机器人按用户说明断电，继续只做离线开发和定向测试；不推送、不部署、不启动现场服务或发送机器人命令。

本次修复真实控制层代码，但验证使用内存后端，不代表真机抓取验收。已完成的步长候选比较见[离线对照](offline-elapsed-step-2026-10-04.md)，两种新步长均未启用。

## 复现与修复

原 `tick` 在检查帧龄后等待 `_send_latest_gripper`。等待结束后直接继续处理同一份 Grip 输入，包括 ARMED → ACTIVE 和锚点捕获。若夹爪等待期间有新帧、旧帧已达到100 ms，或发生解除武装，原输入已经不再代表当前控制权限。

新增案例复现：ARMED 下 Trigger 改变伴随 Grip 按下，内存夹爪调用返回前替换输入或推进假时间到100 ms，原控制层仍消费旧序号并捕获旧Grip锚点。ACTIVE 的部分目标提交原已有后置校验，但解除武装场景仍会把旧序号写回。初版八例有四例失败。

修复在夹爪 await 前保存控制代次，返回后检查模式仍为 ARMED/ACTIVE、代次一致、输入对象仍是最新、帧龄小于100 ms，并检查故障、循环失败、关闭、解除武装和恢复锁状态。不符合就结束本轮 tick，不再处理旧Grip或旧序号；随后正常 tick 处理新帧或原有过期停止路径。

公共 `_frame_is_current` 复用已有目标提交检查条件；`_target_frame_is_current` 仍额外要求 ACTIVE。没有重新实现停止、新的延时任务或新队列。原10 Hz夹爪节流、Trigger有意按下门限、ARMED成功夹爪等待的调度补偿、ACTIVE overrun和失败故障策略均保留。

### 实际边界

- 该检查发生在夹爪调用返回后，不能撤销已经发出的 SDK 命令，也不能中断忽略取消的 SDK 调用。
- 过期输入在返回时被丢弃；本次没有增加即时 stop 调用，仍由下一 tick 执行已有过期处理。不要把它描述为缩短物理停止时间。
- 本次只覆盖夹爪返回至Grip转换的边界。其他记录、预检和锚点读取 await 周围仍需逐项检查；不能宣称所有控制竞态已解决。
- 新鲜0/99 ms输入仍可捕获锚点；100 ms输入不可。等待期间解除武装再重新Arm，旧调用也不能取得新代次的控制权限。

## 验证

新增10案例：ARMED/ACTIVE分别覆盖输入替换、100 ms过期、解除武装、解除后重新Arm；另有0/99 ms新鲜成功返回。新增重新Arm案例最初漏掉释放帧的实际消费，触发原 `arm_requires_grip_release`；补齐释放tick后验证旧调用不会覆盖新序号，未改Arm规则来迁就测试。

首次修复后相关161项通过；最终联合验证 **497 passed（29.69 s）**，退出码0。没有运行后端全集或重复前端构建。主代理复核，未计作独立第二审查人通过。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/control/test_gripper_frame_freshness.py backend/tests/control/test_elapsed_step_experiments.py backend/tests/control/test_input_cadence_experiments.py backend/tests/control/test_robot_control.py backend/tests/control/test_state_observation.py backend/tests/control/test_state_machine.py backend/tests/control/test_coordinate_mapper.py backend/tests/control/test_filters.py backend/tests/control/test_safety.py backend/tests/control/test_recording.py backend/tests/robots/test_lebai_pump.py backend/tests/robots/test_pvat_request_lifecycle.py backend/tests/robots/test_vr_profile_experiments.py backend/tests/robots/test_lebai_backpressure_experiments.py backend/tests/robots/test_stop_transaction.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/api/test_teleop_ws.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

## 后续开发顺序

1. 真机VR控制软件：继续修复记录、预检、锚点读取等待期间的输入与权限失效；保留夹爪失败、停止未确认和断连收尾行为。
2. 目标推进：针对到点截断和抖动时的速度状态定义减速方案并离线验证，再决定积分策略。当前不能直接采用可变步长。
3. 真机验证：仅在用户另行给出明确现场范围后执行，离线通过不能替代现场验收。
4. 后续仿真物理抓取：先修正平台与地面的位置关系，再评估 Web 物理引擎；纳入静态平台、动态物体、质量/摩擦、夹爪接触与约束、释放后重力与碰撞，以及仿真/真机后端隔离。本轮未选型、安装或实现该部分。
