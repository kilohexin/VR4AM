# 显式可选的事务控制运行实现

日期：2026-10-07。接管计划第 33 项。仅离线开发和注入 fake SDK 验证，未推送、部署或连接真机。

## 本轮交付

将候选提交、减速换向和追赶监督从测试原型迁入运行模块：

- `backend/app/control/braking_approach.py` / `brake_retarget.py`：保留已验证的命令区间速度模型，换向前先降低命令历史速度；生成点越界仍拒绝。不代表机械臂的物理制动或碰撞保证。
- `candidate_commit.py`：一次一个待完成草案，按 completion、接受引用及 owner/pump 代次提交历史；拒收、非完整接受、撤销继续冻结。重捕获要求新的有效 owner 代次。
- `catch_up.py`：显式 `CatchUpSupervisor`，追赶已接受关节解，预算/观测阈值来自必须提供的参数；观测退出或预算耗尽请求原 owner 停止，不恢复旧速度。
- `transactional_strategy.py`：独立 `TransactionalTargetStrategy` 同时实现准备/派发和生命周期接口。使用 owner 提供的当前上下文及 adapter 公开视图，无 RobotControl 子类依赖、无测试模块依赖、无后台任务。
- `transactional_settings.py`：不可变 `PursuitSettings` 与严格配置解析。

策略只保存当前候选、追赶状态、最新请求姿态、最后回执和最后退出原因。连续 1,000 次成功提交的弱引用测试确认仅当前回执仍被策略保留。旧测试 trace 列表只留在测试侧；旧原型导入变为运行模块的兼容导入或显式实验参数包装，从而让既有算法及完整 tick 案例覆盖运行实现。

`RobotControl(transactional_settings=...)` 将同一策略绑定到两个接口；与手动注入 `target_strategy` / `target_lifecycle` 冲突时立即拒绝。策略会重新检查权限、后端身份和草案归属；旧候选/追赶计划不能跨重捕获派发。停止、夹爪、输入撤销和故障流程继续由原 RobotControl 执行。

## 配置与证据

顶层 `target_strategy` 可省略或显式设为 `{kind: legacy}`，均保持旧策略。实验运行入口格式如下；这些数值仅用于本轮 fake 测试，不是批准的真机参数：

```yaml
target_strategy:
  kind: transactional_experimental
  pursuit:
    budget_ms: 500
    max_observation_age_ms: 40
    max_observation_gap_ms: 40
    position_tolerance_rad: 0.001
    velocity_tolerance_radps: 0.002
    settle_samples: 3
```

所有追赶字段必须显式提供。未知模式、缺失/多余字段、布尔值、字符串数字、非有限或非正数、非整数时间/计数、少于两个观测样本、超过追赶预算的观测年龄/间隔均拒绝。配置选择要求 LEBAI control；不能借此把 readonly 或 simulator 自动切换为机器人运动。原 `VR4ARM_REAL_ROBOT_CONFIRM` 配置门禁保持，参数存在不代表现场授权。

会话 metadata 新增 `target_strategy`，选中实验策略时另记录 `pursuit_settings`。不记录 IP、凭据或完整现场配置。`create_app` 正式 lifespan 已用注入 fake 完成启动/收尾，确认创建的是原 RobotControl 和运行策略，不是测试子类；全程 network/discovery 为 0。

默认配置文件、现场配置、运动限值、200 ms 停止截止、`stop_unverified` 和复位白名单未更改。默认运行路径仍是旧策略，历史推进问题尚未在默认选择下修复。

## 验证

新增 71 个案例，覆盖配置和参数拒绝、接口冲突、不兼容 backend、确认门禁、时钟自然前进的重捕获、可选单步限值、非默认追赶预算及观测阈值、1,000 次提交的有界保留、旧计划跨重捕获拒绝、正式 fake 启动和会话 metadata。

TDD：先出现缺失运行模块的导入失败；迁移测试首轮 149 通过、2 失败，原因是旧测试重新构造了空策略，现改为检查正在运行的策略实例。扩展时新增 metadata 案例按预期失败，补实现后通过。迁移相关 12 文件 352 通过；最终 25 文件 **825 passed in 24.78s**。主执行者完成差异自查，无独立审查结论。

最终定向命令（仓库根目录）：

```powershell
.venv/Scripts/python.exe -m pytest `
  backend/tests/control/test_transactional_runtime.py `
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
  backend/tests/control/test_braking_approach.py `
  backend/tests/control/test_brake_retarget.py `
  backend/tests/robots/test_stop_transaction.py `
  backend/tests/robots/test_retarget_backpressure.py `
  backend/tests/robots/test_explicit_catch_up.py `
  backend/tests/robots/test_command_admission.py `
  backend/tests/robots/test_command_completion.py `
  backend/tests/robots/test_lebai_adapter_control.py `
  backend/tests/robots/test_lebai_adapter_readonly.py `
  backend/tests/config/test_real_robot_config.py `
  backend/tests/config/test_low_speed_profile.py `
  backend/tests/api/test_health.py `
  backend/tests/api/test_teleop_ws.py `
  -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

未例行运行后端全集。没有前端改动，未进行头显/WebXR 或真实网络验证。既有 WebSocket 回归通过，不等于新策略已完成完整 WebSocket 动作会话验收。

## 当前进度与下一步

上一轮“可实际启动的事务策略”这一软件缺口已补齐，正式 fake 启动已通过。接下来完成**选中此策略的 WebSocket 会话与运行循环组合验收**：输入→接受回执→追赶/夹爪→释放或断连→故障收尾，并检查日志关联与任务退出，再冻结首轮现场测试版本。

目前仍是实验运行策略：追赶退出要求停止并重新捕获，尚未证明连续操作手感；20–80 ms 是候选算法支持的命令区间，原 tick 仍使用 20 ms。不能把离线轨迹或配置合法性视为真机性能或安全证明。

冻结现场日志中的运动停止 SDK await 609 ms 问题仍未解决，200 ms 截止与 `stop_unverified` 保留。软件组合验收完成后，再提交具体现场范围供用户授权；当前不要求通电。物理引擎、平台位置与仿真抓取继续排在真机 VR 控制之后。
