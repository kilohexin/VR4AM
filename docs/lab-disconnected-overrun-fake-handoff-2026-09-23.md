# R49 实验室 AI 交接：断连空闲 `control_overrun` 的假 SDK 复核

目标：验证 R48 在 `DISCONNECTED`、无 VR 帧、已确认断连停止时出现的空闲误报不再闩锁 `control_overrun`。这是**假 SDK**回归，不是停止故障修复，也不是真机 VR 放行。

目标版本以主机端随通知给出的完整提交号为准；先在全新隔离目录核对提交、源码导入路径及依赖。不要推进原仓库异常引用，不覆盖 R48 原件。

1. 离线运行 `backend/tests/control/test_robot_control.py` 与 `backend/tests/api/test_fault_paths.py`；保存退出码、完整 stdout/stderr。重点记录“已断连空闲延迟不产生 FAULT”和 `READY` 超时仍产生 FAULT 的测试结果。任一失败即停止并回传，不重跑到通过。
2. 仅用 `scripts/run_fake_lebai_stack.py` 启动回环后端，先核验 `/health` 的 `backend=LEBAI_FAKE`、`hardware_verified=false`、`real_robot_enabled=false`。若任何一项不符，立即关停并回传；不要改用 `python -m uvicorn app.main:app`。
3. 新建一次性产物目录。只做一次 `hello` WebSocket 握手后正常关闭，记录 `close_code`；**不发 VR 帧、运动、夹爪或 Home，不启 Quest，不启动前端**。断连停止确认后继续让假后端空闲运行至少 **240 秒**，覆盖 R48 的 206 秒故障时点。期间不重连、不人为触发调度阻塞。
4. 完整保留 `session.jsonl`、`summary.json`（若生成）、进程输出及 `/health` 前后结果。核对 `stop_requested/stop_confirmed reason=disconnect`、后续 `robot_fault reason=control_overrun` 数量、`stop_requested reason=fault` 数量和末态。程序正常关闭；若只能强制结束，明确标注 `summary.json` 缺失原因，不能由缺失推断无故障。
5. 若再次出现故障，原样回传 `robot_fault` 整行和相邻事件，尤其 `mode_before_fault`、`deadline_lateness_ns`、`previous_tick_duration_ns`、`wakeup_lateness_ns`、`frame_age_ms`；**不重试**。若未出现，仅报告“本次 240 秒窗口未复现”，不要宣称所有调度问题已解决。

全程不连接 `10.20.17.1`/`172.16.2.162`，不切现场配置的 `mode`，不运行真机预检、stop、prepare 或 VR。收尾核对无 8000/5173 监听、无残留进程、现场配置字节未变。R37/R41 的非预期位移及真机停止失败保持独立未闭环；本轮结果不得用于放行真机运动。
