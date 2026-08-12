# 无真机离线实验演练

本文说明如何在没有乐白 LM3 和 Meta Quest 3 的情况下，在一台 PC 上演练当前遥操作软件链路。仓库已经包含真实 LM3 适配器、预检、停止和诊断代码，但本流程固定使用 `LEBAI_FAKE / DIGITAL_TWIN`，不会导入或连接真实乐白 SDK，不读取真实机器人配置，也不授权真实运动。

离线演练的结构是：一个 PC 页面持有唯一的 `/ws/v1/teleop` WebSocket 控制连接，浏览器中的 `OfflineRehearsalController` 通过现有 Home、Arm、Grip、相对位姿、停止和复位消息进入 `RobotControl`，后端再通过 `RealLebaiAdapter` 形状相同的上层路径驱动 `FakeLebaiClient`。页面只根据 Fake 返回的实际关节和诊断状态推进。此模式没有第二控制端，也没有 observer/观察端；不要同时打开第二个 PC 标签页或 Quest 页面。

> **边界：**离线演练和三条非硬件门禁只能证明软件路径在 Fake/仿真环境中的行为。它们不证明真实方向、延迟、停止距离、急停、夹爪力或负载能力，不得作为进入真机 `control`、自由遥操作或无人值守运动的依据。现场仍必须从 `readonly` 零写入预检开始。

## 前提

- Windows PowerShell、Python 3.11 或更高版本，以及与 `web/package-lock.json` 兼容的 Node.js/npm。
- 已按 README 安装 `backend[dev]` 和 `web` npm 依赖。
- 从仓库根目录运行命令；本机的 `127.0.0.1:8000` 和 `127.0.0.1:5173` 未被其他进程占用。
- 不设置 `VR4ARM_REAL_ROBOT_CONFIRM`。若已经设置 `VR4ARM_CONFIG`，它只能是 `config/fake-lebai.yaml`；启动器会拒绝其他值。
- 关闭其他 VR4Arm 控制页面。演练页面必须是唯一 WebSocket 所有者，且不进入 WebXR。

## 一条命令启动

从仓库根目录运行：

```powershell
python scripts/run_offline_rehearsal.py
```

启动器固定加载 `config/fake-lebai.yaml`，依次启动 Task 8 的 Fake Lebai 后端和 HTTPS Vite 前端。看到以下信息后，在同一台 PC 打开打印出的 loopback URL：

```text
Offline rehearsal ready: https://127.0.0.1:5173/
Runtime: LEBAI_FAKE / DIGITAL_TWIN / hardware_verified=false
```

本地开发证书可能触发浏览器证书警告；仅在地址确实为上面的回环地址时继续。页面顶端必须永久显示 `DIGITAL TWIN / 数字孪生，不是真机`，运行身份必须是 `数字孪生 · LEBAI_FAKE`。若显示 `SIMULATOR`、`LEBAI` 或未知身份，不要开始演练。

## PC 页面操作

浏览器 smoke 的语义是直接操作 PC 离线演练页面，不模拟 WebXR，也不把单元测试或构建结果当作视觉/交互验收。页面应显示离线演练面板、当前阶段、步骤、进度、目标/实际误差、超时、首个失败原因、两条报告路径和八项真机待验证项。

页面只有两个演练按钮：

- `开始演练`：仅当唯一 WebSocket 已连接、页面未进入 XR、前后端身份均为 `LEBAI_FAKE`、`hardware_verified=false`、模式为 `READY`、Fake 后端为 `IDLE`，并且没有故障或软约束时可用。点击一次启动完整的确定性流程；运行中该按钮禁用。
- `停止演练`：仅在演练运行中可用。点击后立即停止产生新的 VRFrame，清空本地轨迹并松开 Grip，再经现有停止请求等待后端确认、释放控制权并写出失败/中止报告。不要刷新页面来绕过失败；停止无法确认时结果不得解释为安全复位。

预期阶段严格按以下顺序显示：

1. `identity_preflight`
2. `home`
3. `arm_and_anchor`
4. `translate`（`+X/-X/+Y/-Y/+Z/-Z`）
5. `rotate`（`+roll/-roll/+pitch/-pitch/+yaw/-yaw`）
6. `gripper`
7. `pick_place`
8. `soft_boundary`
9. `tracking_loss`
10. `recovery_and_home`
11. `final_stop`
12. `finalize`

`arm_and_anchor` 不会改写现有 Home。Fake 后端在 Home 和解锁确认后，继续通过同一个控制 WebSocket 的普通 Grip/VRFrame 路径移动到 `config/fake-offline-rehearsal.json` 固定的演练准备位姿；连续 3 个递增的权威状态都达到 `3 mm / 2°` 后，才在准备位姿重新设定演练锚点并进入平移。准备动作单独拥有 `15 s` 超时；每一个平移/返回、旋转/返回子目标分别拥有 `8 s`，其他非运动阶段仍为 `8 s`，完整演练还有 `300 s` 硬超时。任一超时都走同一条已验证停止和报告路径。该准备位姿严格属于 `LEBAI_FAKE / hardware_verified=false`，不改变真机 Home 或真机控制行为。

后端 SafetyLimiter 的原始锚点仍是第一次 Grip 在未改写 Home 捕获的 TCP；浏览器准备位姿只是后续任务锚点，两者不会混用。准备确认后，页面按共享 Fake 配置重排橙色方块、抬高支撑台和绿色放置标记。抓取接近、抬升、转移和下降相对原始 Home 的最大分量偏移均小于 `0.10 m`，并继续经过真实 `SafetyLimiter -> FakeLebaiClient` 路径；控制器只消费页面返回的方块/放置观测并发送普通 VRFrame，不注入权威机器人状态或报告专用运动目标。

从点击 Start 到清理完成，桌面指针、滚轮、Grip 和 Trigger 的手动处理均保持禁用，包括尚无合成样本的 begin、Home 与 cleanup 窗口；VR 退出安全行为不变。若文档变为 hidden，页面立即调用现有 `requestStop('page_hidden')`，先进入安全停止/报告路径，而不是等待 stale 看门狗兜底。visible 事件不停止演练，页面销毁时会移除该监听器。

完整浏览器 smoke 至少要实际观察平移、旋转、夹爪、方块抓放、跟踪丢失后的停止与恢复，以及最终停止。成功时进度到达 `12 / 12`，阶段变为 `passed`，Start 重新可用、Stop 禁用，并显示 JSON 和 Markdown 报告路径。页面隐藏、刷新、关闭、WebSocket 断开、所有权丢失、状态/诊断过期、FAULT/STALE/急停、阶段超时、非有限数值或后端拒绝都会进入相同的安全停止收尾。

### 当前 Task 9 浏览器验收状态

当前冻结里程碑的 `browser_smoke` 必须如实保持 **pending/failed**，不能标记为 `done`：

- 应用内 Browser 首先按计划使用，但导航被 `net::ERR_CERT_AUTHORITY_INVALID` 阻止；安全边界禁止绕过该证书错误，因此没有把应用内 Browser 记为完成。
- 经明确允许后使用了本机渲染 Edge fallback。桌面首屏的页面身份、永久 Fake 警告、有效 DOM、离线演练面板、Start/Stop 初始资格、无框架错误覆盖层以及应用控制台健康检查通过；这只证明首屏，不是完整流程通过。
- Edge 实际运行中，`identity_preflight` 用时约 `0.080 s`、`home` 用时约 `0.837 s`，Fake-only 准备位姿在 `arm_and_anchor` 中用时约 `11.488 s`，满足独立 `15 s` 准备超时。
- 后端数字孪生测试只以有界迭代检查准备位姿和 12 个目标/返回的几何可达性与安全性；共享 FakeClock 的迭代次数不是模拟秒数或墙钟时间证据。`15 s / 8 s / 300 s` 的选择、逐目标重置和失败收尾语义由确定性的控制器测试覆盖；本轮真实渲染证据只证明上述 `11.488 s` 准备阶段，不证明尚未完成的平移/旋转阶段耗时。
- 完整流程没有完成：`translate` 期间从已接收 `seq=504` 到清理帧 `seq=510` 出现 `399.063 ms` 的浏览器/后端帧接收空洞；后端在超过固定 `100 ms` stale 阈值后正确进入 `STALE`、执行停止并生成失败报告，首个失败为 `unexpected_stale`。证据对已从正式候选目录移至本次任务的外部证据目录 `C:\Users\Kilo\.codex\visualizations\2026\07\12\019f5689-0b13-76c3-8ce0-b2c05b7976ff\task9-offline-rehearsal-diagnostics\2c37fc5bb0524c3fa313d2cd6d917692.json` 和同名 `.md`，内容 SHA-256 保持不变。
- 随后的诊断运行在首个 VRFrame 到达后端前过早点击 Start，因 Home 的 Grip/最新帧前提不满足而得到 `home_rejected`；该运行无效，不计入浏览器验收，也不能用来覆盖前述失败。

因此本冻结点没有完成旋转、夹爪、抓放、跟踪丢失恢复、最终停止、最终桌面状态或 `390 px` 响应式页面的端到端浏览器确认。三条自动化非硬件门禁当前可以通过，但聚合报告的 `browser_smoke.status` 必须保持 `pending` 且理由为 `browser_smoke_not_run`；自动化门禁通过不等于浏览器 smoke 通过。不得用单元测试、构建或无效诊断运行替代。

本轮仍可恢复的八组失败/诊断报告对均保留在本次任务的外部证据目录 `C:\Users\Kilo\.codex\visualizations\2026\07\12\019f5689-0b13-76c3-8ce0-b2c05b7976ff\task9-offline-rehearsal-diagnostics\`，并由同目录 `hashes.sha256` 固定内容哈希。run ID 为：

- `644768a0932f41b6ad9bd7e20cf31871`
- `7ed77ee406ac44dd8e610a9036f2fa28`
- `7e668a5ca6c64f709917a2bd648594d5`
- `203bad4c7d4c4d9b882dea9c436b59c4`
- `281148299014447d9d84b46994181081`
- `6d0d37abb4854c62a481e3b79b36c841`
- `2467076474d74783a3d1939b2546ab5f`
- `2c37fc5bb0524c3fa313d2cd6d917692`

每个 run ID 都对应原文件名不变的 `.json` 与 `.md`。无效的提前 Start 诊断 `980a70e8486e46d6a457376914b89412` 在收到“移动并保留”要求之前已从候选目录移除，后续按文件名和 run ID 搜索没有找到可恢复的逐字节副本；不得重建或把它当作验收证据。正式候选目录 `artifacts/acceptance/offline-rehearsal/` 保持为空，避免将失败诊断误判为已完成的浏览器候选。

正常退出时回到启动器终端按 `Ctrl+C`。启动器按前端、后端的逆序停止子进程；应等它退出后再关闭终端。

## 报告与三条非硬件门禁

一次页面演练生成同一 `run-id` 的两份报告：

```text
artifacts/acceptance/offline-rehearsal/<run-id>.json
artifacts/acceptance/offline-rehearsal/<run-id>.md
```

JSON 是机器判定来源，Markdown 是同一结果的人读摘要。两者记录 schema/run ID、开始和结束时间、Git commit 与 dirty 状态、运行身份、配置/运动学/GLB 哈希、每阶段目标/实际/误差/耗时/状态/失败原因、最终 state 与 diagnostics、`outcome`、`hardware_verified=false` 和固定的 `hardware_pending`。失败或人工停止也必须写报告，不能只保留成功结果。

从仓库根目录按顺序运行三条非硬件门禁：

```powershell
python scripts/accept_virtual_lm3.py
python scripts/accept_fake_lebai.py
python scripts/accept_offline_rehearsal.py
```

对应的聚合报告为：

```text
artifacts/acceptance/virtual-lm3-latest.json
artifacts/acceptance/fake-lebai-latest.json
artifacts/acceptance/offline-rehearsal-latest.json
```

三条命令都必须退出 `0`，三份报告都必须是 `passed=true` 和 `hardware_verified=false`。Fake 与 Offline 报告必须保留下面全部八项。Offline 聚合报告还记录子命令的 `argv/cwd/returncode/duration_s/passed_count/failed_count/output_tail`、三条 gate 的报告位置与错误、Git/model 来源，以及 `browser_smoke.status/report_path/reason`。只有真实 PC 页面完成了一次有效演练并生成通过报告时，`browser_smoke.status` 才能是 `done`；浏览器工具没有实际运行时必须保持 `pending`，不能用 Vitest 替代。

Virtual 报告中的 `soak_performance.warning=true` 表示长场景绝对吞吐低于 `2000 steps/s`，用于提示当前 PC 负载或性能余量。若完整 soak 安全不变量、`60 s` 硬超时和长/短吞吐比（至少 `0.70`）仍通过，它是非阻塞性能警告，不等于安全失败，也不应通过放宽阈值或删除检查来隐藏。

`npm.cmd run build` 可能继续给出 Vite 大 chunk/bundle 警告；只要构建退出 `0`，同样应如实记录为非阻塞警告。

## 八项现场真机检查（全部待完成）

以下标识必须在 Fake、页面演练和 Offline 报告中原样保留。离线成功不会勾销任何一项：

1. `sdk_connection`
2. `tcp_home_joint_limits`
3. `translation_direction`
4. `rotation_direction`
5. `gripper_direction_force`
6. `pvat_tracking_latency`
7. `stop_distance_estop`
8. `lightweight_grasp_release`

它们分别需要现场确认 SDK/网络连接；TCP、Home 与关节限位；六向平移；三轴双向旋转；夹爪方向与力；PVAT 跟踪与延迟；停止距离与急停；轻物体抓取/释放。在实际 LM3、LMG-90、Quest、急停可达环境和独立观察员条件下逐项完成之前，真机状态始终是 **PENDING / `hardware_verified=false`**。

## 故障排查

- 启动器提示 `unsafe_offline_environment`：清除 `VR4ARM_REAL_ROBOT_CONFIRM`，并清除 `VR4ARM_CONFIG` 或把它设为 `config/fake-lebai.yaml`；不要绕过保护。
- 启动器等待超时或子进程提前退出：检查 `logs/offline-rehearsal-backend.log` 与 `logs/offline-rehearsal-frontend.log`，并确认 8000/5173 端口未占用。
- HTTPS 页面打不开：确认使用打印的 `https://127.0.0.1:5173/`，只处理该回环地址的本地开发证书警告。
- Start 一直禁用：核对页面身份、连接、`READY/IDLE`、XR 未启动、无故障/软约束，并关闭其他 VR4Arm 页面；系统没有 observer 模式，第二个连接会被拒绝。
- 演练失败或停止：先读面板“首个失败”和生成的 Markdown，再查对应 JSON、后端日志与前端日志。保留失败证据，不要手改报告为通过。
- Offline gate 报 `browser_smoke_not_run`：先用可用的浏览器控制工具在真实 PC 页面完成上述完整流程。若浏览器自动化不可用，如实保留 `pending`，不要以单元测试代替。
- 报告缺失或字段/哈希不一致：从仓库根目录重新依序运行 Virtual、Fake 和 Offline 门禁；不要复制旧提交的 latest 报告。

真机部署、`readonly` 预检和逐轴低速 commissioning 见 [LM3 真机部署与首次实验流程](real-robot-deployment.md)。
