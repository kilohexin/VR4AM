# 目标接近减速：离线候选与不可行边界

## 实现范围

本轮在 `backend/tests/control/braking_approach_probe.py` 实现测试专用候选，生产 SafetyLimiter、固定20 ms控制步长、停止策略和配置均未改动。只比较限幅器输入/输出，不接SDK、RobotControl调度、滤波、IK或机械臂。不推送、部署或运行现场服务。

候选的目标是让可行的目标接近过程提前减速，并明确暴露无法同时满足约束的请求。它只建模20至80 ms的离散消费间隔，使用实际间隔，不截短；更短、过长、非正或非有限间隔直接拒绝。不能将这个测试范围当作生产允许的帧间隔。

## 候选规则

位置与旋转分别沿当前位置至请求的方向推进，旋转使用世界坐标下的相对旋转向量。对单位方向u、旧速度v、加速度预算`a·dt`，先求速度直线`s·u`与以v为中心的加速度球的交集，再与非负速度、速度上限、单步上限及剩余距离相交。

正常接近还要求 `s·dt + s²/(2a) ≤ 剩余距离`，为本步之后预留保守减速距离。接近到足够小的最后一步时，允许精确到点，但保存该步实际区间速度；仅当它能在模型最短20 ms下一步内降至零时才采用此分支。不会在到点当帧直接抹掉速度历史。

平移和旋转都求得可行速度后才提交内部状态。如果任一通道不满足，抛出 `InfeasibleApproach`，两个速度状态均保持原值，不输出新Pose、不重试。这个拒绝**不是停止命令**，不能使上一条设备命令消失。

直线方向、减速距离预留和20 ms最短间隔都是候选自身的限制；“候选不可行”不意味着所有曲线轨迹或其他规划器都无解。

## 确实无法同时满足的近目标例子

旧线速度5 mm/s，加速度上限20 mm/s²，dt=20 ms：即使按最大幅度减速，下一步速度至少4.6 mm/s，位移至少0.092 mm。若新目标仅在前方0.01 mm，既要求不越过它，又要求速度变化不超过本步加速度预算，这组约束没有可行解。角速度0.05 rad/s、上限0.1 rad/s²的对应下一步至少0.00096 rad，也超过突然缩到0.0001 rad的请求。

因此不能简单删除原到点截断后仍承诺不越界，也不能把“丢弃新目标”称为已经安全减速。需要独立的故障/停止衔接规则；本轮没有新增自动停止、复位或重试策略。

## 对照结果

三种对照：原限幅器+固定20 ms；原限幅器+实际间隔；新候选+实际间隔。消费间隔分别20、40、20/60交替、60/20交替。以下为请求在1秒内到6 mm/0.1 rad并保持至4秒的完整窗口，表中峰值是**区间平均目标速度的右区间差分加速度**，不是设备实测加速度。

| 间隔 | 原固定20：线加速度峰值 mm/s² | 原实际间隔：线峰值 | 候选：线峰值 | 候选：角峰值 rad/s² |
|---|---:|---:|---:|---:|
| 20 ms | 190 | 190 | 20 | 0.1 |
| 40 ms | 47.5 | 80 | 20 | 0.1 |
| 20/60 ms | 170 | 82.222 | 20 | 0.1 |
| 60/20 ms | 166.667 | 86.667 | 20 | 0.1 |

四组候选均完成整个窗口并到达6 mm/0.1 rad，内部速度最终为零，没有越过当前至目标的线段。负方向静止目标、斜向平移及非零初始旋转也有独立验证。速度5 mm/s、角速度0.05 rad/s、单步0.5 mm/0.2°及速度差预算逐项检查。

反向轨迹在1秒后返回原点，候选在约1.10至1.12秒出现不可行，实验立即终止；不能把此前的合规段算作完整反向成功。原限幅器反向时还会出现输出在“本步起点至当前请求”线段之外的情况，这是目标方向变化后的惯性行为，不等于越过工作空间。

突然移近案例按各策略当时状态选取前方0.01 mm/0.0001 rad并保持，故该压力输入是状态相对的，并非各策略共享的绝对轨迹。候选在首次突变时拒绝。相同状态下重复不可行请求仍被拒绝，但候选没有故障锁存：随后若给出可行目标，它仍会输出。这正是必须补控制层衔接的原因。测试中的 reset 仅表示一个已经停止并重捕获的新阶段，不能作为运行中自动恢复。

完整[36组结果](offline-braking-approach-results-2026-10-05.json)包含`completed_window`、拒绝时刻和已处理数量。未完成窗口的末尾位置仅表示最后有效输出，不是全窗口终点。没有原始现场日志或现场参数入库。

## 验证和下一步

新增59案例：8个正负静止目标/节奏；4个不可行且不污染状态；6个无效/未建模间隔；1个重置；36组合成比较；3个斜向/非零初始旋转；1个明确拒绝不等于停止或故障锁存。先出现预期缺少测试helper的导入失败，实现后54项通过，补充边界后59项通过。由主代理复核公式、半径相交、双通道提交、端点速度和未完成指标；未计作独立第二审查人通过。

相关12文件定向回归 **402 passed（11.29 s）**；补强“随后可行目标仍可输出”的非锁存断言后，候选59项再次通过（5.27 s）。没有运行后端全集或前端构建。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/control/test_braking_approach.py -q -s -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
.venv/Scripts/python.exe -m pytest backend/tests/control/test_braking_approach.py backend/tests/control/test_safety.py backend/tests/control/test_filters.py backend/tests/control/test_coordinate_mapper.py backend/tests/control/test_robot_control.py backend/tests/control/test_input_await_boundaries.py backend/tests/control/test_gripper_frame_freshness.py backend/tests/control/test_state_machine.py backend/tests/control/test_elapsed_step_experiments.py backend/tests/control/test_input_cadence_experiments.py backend/tests/robots/test_lebai_pvat.py backend/tests/robots/test_stop_transaction.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

采用决定：暂不替换生产限幅器。下一步把“不可行请求”的测试候选接到内存控制与fake停止链路，验证故障锁存、停止未确认、取消、禁止自动恢复和重新捕获；仍不启用默认策略。完成这些软件验证也不等于真机运动验收。真机VR控制和夹爪仍优先于平台修正/Web物理抓取。
