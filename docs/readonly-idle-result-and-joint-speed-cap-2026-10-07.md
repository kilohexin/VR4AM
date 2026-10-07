# IDLE 只读窗口与关节移动限速修正

2026-10-07，Asia/Shanghai。用户批准“原厂手动启动一次后，新的一次 5 秒只读连接”，随后确认“已 IDLE，静止”。用户补充启动期间会有自检小幅摆动后归位；这是现场描述，本次采集始于其确认 IDLE、静止之后，没有采集或验收启动自检过程。

## 本次实机只读结果

执行源码为 `7cf8fa5a4eb93ee9e684a70526cfb48ddcf2b8ca`，SDK 0.3.13。执行前再次核对原配置与 readonly 副本哈希一致、旧服务无监听、Git 工作区干净。创建独立新证据目录与报告，单次连接，采样窗口 **5,000 ms**，connect await 约 **15 ms**，退出码 **0**、`complete=true`。79 次固定查询均返回：状态/急停/运动学/夹爪各 19 次，工具 TCP、负载、重力各一次。19 个状态均为 IDLE，急停原因均为 NONE。

独立采集进程已退出；复查该 PID 不存在，8000/5173 无旧服务监听。没有创建控制 adapter、RobotControl 或 pump，没有代理启动/prepare/Home/运动/夹爪写入/停止/复位请求，也没有重试。配置副本与原件未修改。进程退出不等于控制柜侧连接释放已验收。

原始报告 `.tmp/readonly-idle-window-20261007T133102-18gtjrf0/state-report.json`，SHA-256 `df7e877cf7fb728e70f525d3e218b9588442d79d18335fb941cc6b9dfe6cc330`。执行清单、退出结果、进程复查、原始 SDK 值和后续离线分析同目录保留；不提交原始日志或配置。本轮一次连接授权已执行结束。

## 对保存数据的离线核对

不建立第二个连接，只读取保存的 JSON。19 组运动学读数跨度约 **4,797 ms**，最大组间开始时间间隔 **281 ms**；该只读轮询不是控制循环遥测频率验收。报告的关节速度全为零，关节与实际 TCP 读数相对首组没有变化。这只描述离散遥测，机械臂物理静止来自用户现场确认，不能代替运动中停止证据。

所有保存的关节均在现存软限位及余量内，实际 TCP 位于保存的启动包络内。工具 TCP 位置与配置偏差约 **0.031 mm**、旋转偏差 **0°**，在现存容差内。夹爪幅值均为 **0**；只记录读数，本轮未进行 Trigger 或夹爪开合，不能由此确认开闭方向或抓取。负载与重力返回了所需有限数字，但其物理正确性仍需现场核实。

J3 最小绝对角度约 **0.0275°**，J5 约 **0.0824°**，均在现有 **5°** 笛卡尔奇异保护阈值内。按照控制模式的当前门禁，这些读数会触发 `singular_configuration`，不能直接进行 VR 平移或非 VR 笛卡尔早停诊断。没有降低门禁、切换 control 或调用完整实机 preflight；本轮也未查询运行任务 ID。

## 准备位姿限速问题与离线修复

将保存的实际关节与已有 `teleop_ready_q` 比较，最大关节调整约 **23.10°**。准备位姿入口会真实执行 `movej`，不能将其当作连接操作。离线追踪确认：`Settings` 提供默认 `HomeOptions.max_speed_radps=0.5`；`_move_to_joint_pose` 原来直接将其发送给 SDK，而保存现场的关节速度上限为 0.15。低速诊断 profile 将上限收紧到 0.05 时，原路径仍可能发送 0.5，违背 profile 的速度上限。

修复共享关节移动入口：SDK 速度取 `min(options.max_speed_radps, control.max_joint_speed_radps)`。因此 prepare 与显式 Home 都遵守调用速度及 profile 上限中的较小值，加速度沿用 profile 原值。此改动没有修改 Home 按钮/触发路径、目标关节、停止截止、异常清理、奇异保护或复位白名单，亦未执行任何真机移动。

新增 8 个注入 SDK 用例：两个入口 × 高于 0.15/高于 0.05/低于上限/等于上限。最初 prepare 替身未更新实际关节导致超时；修正测试替身模拟到位后，原实现的 4 个超上限用例确定性失败，其余 4 个通过。修复后全部通过。测试模拟瞬间到位，只验证请求参数与原稳定/退出流程，不是物理动力学或停止验证。

最终六个相关文件 **162 项通过，7.47 秒**，包含 adapter 控制/readonly、低速 profile、停止事务与脚本回归。输出 `.tmp/joint-move-speed-regression.txt`。没有后端全集、VR 头显测试或独立审查，使用主作者自查。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/robots/test_joint_move_speed_limit.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_lebai_adapter_readonly.py backend/tests/config/test_low_speed_profile.py backend/tests/robots/test_stop_transaction.py backend/tests/scripts/test_real_robot_scripts.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

## 当时拟议范围与后续执行

以下保留当时提出的范围。此后用户已明确批准一次 prepare 并选择本人观察/控制急停，执行已结束，授权已消费；到位完成，但实际速度反馈与请求不符，后续真机动作暂停。当前结论见 [单次准备位姿结果](prepare-motion-result-2026-10-07.md)，不能继续按“等待 prepare 授权”的旧状态执行。

已生成独立低速 **readonly** 候选与 `prepare-motion-scope.json`，包括保存的当前/目标准备关节、变化量、profile 哈希及待确认事实；没有生成或选用 control 配置。拟议仅一次 `real_robot_smoke.py prepare`：关节速度最多 **0.05 rad/s**（约 2.86°/s），加速度最多 **0.2 rad/s²**，原关节移动超时 **15 s**，目标仅为既有准备位姿。最大角变化约 23.10°；只按速度计算的时长下界约 8.06 s，不是完成时间保证。

这需要用户明确批准新的真机连接/准备位姿移动/既有停止与清理范围，并确认全臂扫掠路径无遮挡、独立急停观察人到位、执行前重新核实 IDLE 静止。配置合法、关节端点在限位内和过去使用过此目标都不能证明当前路径无碰撞。

动作异常或超时仍走原停止清理/锁存，运动中停止约 609 ms 返回与 200 ms 截止问题仍未解决；未确认停止时结束试验并按现场应急流程处置，无自动重试/reset/stop_sys。完成准备后先评估遥测和控制 preflight；不自动续接早停诊断、VR、Home 或夹爪测试。未推送、未部署服务。
