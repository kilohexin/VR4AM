# 单次 3 mm 运动中停止诊断草案

2026-10-07，Asia/Shanghai。**尚未授权、未执行、没有创建新 control 配置。** 当前完成的是 [速度倍率只读采集](velocity-factor-readonly-result-2026-10-07.md)，它的授权已经消费。

## 目的与仍未解决的问题

建议使用现有非 VR 诊断入口，捕获一次实际运动中的 Grip 释放及 `stop_move` 起止/锁等待/后续状态证据。历史 SDK 停止 await 约 609 ms，超过本地 200 ms 截止；上次 prepare 请求 0.05 rad/s，而 J5 实际速度反馈峰值约 0.105461 rad/s。**这两项没有修复或验收通过。** 本提案是获取新诊断证据，需要用户明确决定是否接受这次受限现场试验。

现有 CLI 的 RobotControl 未传入事务策略参数，因此使用默认 legacy 路径。它用于复核共同 adapter/SDK 停止链路，不是 `transactional_experimental` VR 会话验收；不把成功结果扩大为连续 VR、Trigger 或抓取可用。

## 具体范围

| 项目 | 拟议范围 |
| --- | --- |
| 次数 | 一个独立 CLI 进程，一次控制连接，一次平移，无第二次运动或自动重试 |
| 起始核对 | 同一连接先执行原只读 preflight，必须 IDLE、急停 NONE、工具匹配、任务/运动状态及关节/姿态门禁通过；失败即结束 |
| 目标 | 机器人基座坐标 +Z，目标增加 0.003 m，保持姿态；捕获到运动证据后提前释放 Grip，可能未走满 3 mm |
| 请求 profile | 沿用已验证低速 readonly 候选，新授权后才能创建独立 control 副本；TCP 速度 ≤0.005 m/s、加速度 ≤0.02 m/s²；关节速度 ≤0.05 rad/s、加速度 ≤0.2 rad/s²；TCP 单步 ≤0.0005 m，增益 0.2；保留更紧原值和其它门禁 |
| 停止与记录 | 现有 `--early-stop-on-motion`、原停止处理和清理，`--stop-rpc-diagnostics`；停止处理后最多名义 5 秒只读观察，随后断开并核对进程退出 |
| 动作边界 | 不使用头显，不发送 prepare/Home/夹爪写入/复位/stop_sys，不修改倍率、碰撞检测或关节限位；原 Grip 释放/清理停止属于本次拟授权范围 |

上述速度是请求参数和软件预算，**不保证实际速度不超过该值**；3 mm 是目标位移，不保证实际轨迹不会超出。实际关节运动和整臂扫掠不能从 TCP 位移单独推断。原程序名义 4 秒运动帧预算，SDK/读取/清理等待会增加墙钟耗时；异步超时和观察时长不是硬实时保证。

新批准后才可生成 control 副本。参数草案如下，只用于审阅，当前未运行：

```text
scripts/real_robot_smoke.py translate
  --config <新授权后生成并核对哈希的独立 control 副本>
  --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
  --axis z --distance-m 0.003
  --early-stop-on-motion --stop-rpc-diagnostics
  --observe-stop-seconds 5
```

## 离线准备与限制

执行源码候选为 `055828c14e111885a2996cd2e70de285f4f7aa92`；本轮后续提交只包含文档。已有 readonly 低速候选 SHA-256 `faba03b57d9f18514cbb6813ee6780a80c1513a270d425620922bb14b0372a94`，保持不变。

使用保存的末次 TCP 和生产 CoordinateMapper，手位移 15 mm 经增益 0.2 映射为基座 +Z 3 mm，姿态不变。保存起点/目标均在配置的启动 TCP 盒内，最近盒边界余量约 125.88 mm；保存关节软限位扣除原 margin 后的最小余量约 1.867 rad。它们是离线几何核对，**没有调用 IK，没有验证可达性、整臂碰撞或现场净空，没有运行新的 preflight**。不得以这些数值代替现场判断。

现有早停触发仍要求未到目标容差区：新鲜关节速度超过原静止判据，或 TCP 已沿指令方向推进 ≥0.1 mm 后有多个新鲜关节位置变化证据。Trigger 固定为初始化读取值，没有主动开合意图，原控制基线门禁阻止夹爪写入。抓不到窗口即 `smoke_early_stop_not_reproduced`，不能称成功；详见 [现有触发实现说明](commissioning-early-stop-diagnostic.md)。

停止诊断读数与 stop RPC 使用同一个 SDK 客户端，并发读取可能影响 SDK/控制器调度。本次启用结果不能直接当作无诊断读取时的性能；不会凭一次结果分配 SDK/网络/控制器根因。停止诊断字段也不参与停止确认。

## 执行前需要用户决定

AGENTS.md 要求新的连接/动作有独立明确范围和新现场条件。请只在准备执行时确认上述范围、当前上电/IDLE/目视静止、基座 +Z 周围及整臂运动空间无障碍、工具无夹持物、没有其它任务/客户端驱动，以及本人在场观察并能立即操作物理急停。用户本人兼任操作与观察，不记录为独立观察人。若姿态、安装或范围有变化，重评估后再执行。

出现不预期运动、急停/故障、停止超时或 `stop_unverified` 即结束动作试验；现场不等待软件观察尾段才处理异常。保留原锁存，不自动复位/恢复/重试。原清理复用停止事务不是再次试验，不能承诺独立只有一条 stop RPC。后续 VR/夹爪动作不包含在本范围中。

## 验收

准备本草案时定向复核三个已有测试文件，**67 项通过，20.65 秒**。覆盖早停窗口/低速位置证据、无窗口失败、未确认停止不返回成功、诊断读数不触发动作及停止事务迟到/复用/收尾。未跑全集，没有新增测试或生产代码，没有真机动作。命令如下：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/commissioning/test_actions.py backend/tests/robots/test_lebai_stop_diagnostics.py backend/tests/robots/test_stop_transaction.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

必须分别报告运动窗口是否捕获、Grip 释放与停止请求关联、锁等待/RPC await/200 ms 截止、连续停止反馈、最后故障/锁存和现场观察。缺失字段保留缺失；SDK 返回、IDLE 或零速度不单独证明物理停稳。速度反馈仍按未解决问题单独复核，不用新的停止成功追认历史限速。离线通过仅支持软件路径，没有执行本草案。
