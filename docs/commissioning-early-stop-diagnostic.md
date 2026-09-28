# 非 VR 运动中停止诊断

`real_robot_smoke.py translate` 新增默认关闭的 `--early-stop-on-motion`。它只接受 `translate`，继续沿用原有 `--config`、`--confirm`、单轴位移不超过 5 mm、预检和停止核验；不改变网页 VR 控制或生产 `stop_move` 策略。运行程序会真实发起机械臂运动，**离线测试通过不构成运行许可**，真机命令、现场条件和执行窗口须另行明确授权。

启用后，程序在同一 SDK 客户端上发送既有虚拟 Grip 目标帧。在指定轴尚未进入目标的 0.5 mm 容差时，可由两种证据之一触发释放 Grip：新鲜运动学采样的六轴最大速度高于 `home_velocity_tolerance_radps`；或实际 TCP 沿指令方向已推进至少 0.1 mm **之后**，新鲜实际关节位置相对起点超过 4 个编码器 LSB、在 200 ms 内有连续两次进一步变化。后一途径用于速度读数低于阈值的低速运动；TCP 达到 0.1 mm 之前的关节变化不累计，因此不接受单 LSB 抖动或已停止的旧位置证据。触发后仍走原有 `stop_move` 停止和核验路径。`smoke_early_stop_triggered` 额外记录 `motion_evidence`、关节位移与位置证据次数；`smoke_result.early_stop_triggered=true` 只表示捕获窗口且后续停止核验成功，不表示历史延迟故障已解决。

若期限内未捕获上述窗口，即使到达目标，也记录 `smoke_early_stop_not_reproduced` 并以 `smoke_early_stop_not_reproduced` 错误退出，不产出成功的 `smoke_result`。此时仍按原清理路径释放 Grip 并请求停止；如停止未确认，仍保留 `stop_unverified` 故障，绝不把无窗口或未确认停止写成通过。无自动重试、无追加 Home、无自动 `stop_sys`。停止锁等待、SDK await 和核验结果沿用已有 `stop_diagnostics` / `stop_rpc_lifecycle` 事件；缺失字段不得倒算。

旧版诊断在 R64 的一次真机低速平移中未捕获早停窗口，随后 `stop_move` 超时并闩锁 `stop_unverified`。新增的位置证据途径目前仅经假客户端定向离线测试；它是否能在真机上及时捕获窗口、`stop_move` 是否及时返回及机械臂是否实际静止，均不能由离线测试保证。再次运行必须重新取得现场授权。
