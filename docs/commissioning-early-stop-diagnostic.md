# 非 VR 运动中停止诊断（仅离线实现，未获真机执行授权）

`real_robot_smoke.py translate` 新增默认关闭的 `--early-stop-on-motion`。它只接受 `translate`，继续沿用原有 `--config`、`--confirm`、单轴位移不超过 5 mm、预检和停止核验；不改变网页 VR 控制或生产 `stop_move` 策略。运行程序会真实发起机械臂运动，**离线测试通过不构成运行许可**，真机命令、现场条件和执行窗口须另行明确授权。

启用后，程序在同一 SDK 客户端上发送既有虚拟 Grip 目标帧。只有新鲜的机器人运动学采样同时表明：实际六轴最大速度高于既有 `home_velocity_tolerance_radps`，且指定轴尚未进入目标的 0.5 mm 容差，才记录一次 `smoke_early_stop_triggered` 并释放 Grip，走现有 `stop_move` 停止和核验路径。事件记录速度、目标、当时有符号进度与采样时间；`smoke_result.early_stop_triggered=true` 只表示捕获了该窗口且后续流程成功，不表示历史延迟故障已被复现或解决。

若期限内未捕获上述窗口，即使到达目标，也记录 `smoke_early_stop_not_reproduced` 并以 `smoke_early_stop_not_reproduced` 错误退出，不产出成功的 `smoke_result`。此时仍按原清理路径释放 Grip 并请求停止；如停止未确认，仍保留 `stop_unverified` 故障，绝不把无窗口或未确认停止写成通过。无自动重试、无追加 Home、无自动 `stop_sys`。停止锁等待、SDK await 和核验结果沿用已有 `stop_diagnostics` / `stop_rpc_lifecycle` 事件；缺失字段不得倒算。

本功能只以假客户端和定向离线测试验证。真机是否存在足够长的“已开始运动且尚未到目标”采样窗口、`stop_move` 是否及时返回、机械臂是否实际静止，均未验证。
