# VR4Arm 离线分析与后续任务 — 2026-10-02

## 范围与证据

基线：`c561377cdc41123f78d6b67d9870a640237a1fdf`；开发分支：`codex/offline-evidence-audit`。2026-09-30 曾核对 GitHub `codex/offline-rehearsal` 尖端与基线一致，不代表之后远端没有变化。

用户已明确：Home 事件是本人误触 Return Home；现有 B/停止/Home 随状态变化的行为不作为缺陷处理。用户另已确认机械臂关闭电源，当前只允许离线测试。本轮使用文件、冻结日志及 fake 测试，没有启动服务、部署或发送机器人命令。断电是用户现场声明，并非本程序检测结果。

原始迁移目录、现场配置、日志和运行目录均保留。以下路径相对于实验室交付树 `C:\WorkSpace\deliverables\2026-09-29`，原始文件不随报告提交。

| 证据 | 大小（字节） | 本地复核 SHA-256 |
|---|---:|---|
| `artifacts/r29v-motion-final-20260929T163350/session.jsonl` | 17854427 | `d48d1768cc5ce74157b4efafec3b90d8e86e7cda51b35d3e554d6ac187caff5b` |
| `artifacts/r29v2-home-incident-20260929T165322/session.jsonl` | 6521229 | `99e99d108081490258152f5fe8388f2483bb53261b41d31aeb303696be449bc3` |

此前找到的 16:46 停服记录早于 16:49 的 Home 会话，不能用它证明 Home 后已停服。用户后续的断电说明更新了现场状态；此后不再查询现场服务或连接。

## 1. 平移与 IK 背压

按 `server_mono_ns` 排序，避免 critical 与普通日志写入顺序影响时间线。第一次 `ACTIVE` 转换为 `1913580687000000`，Grip 释放停止请求为 `1913584875000000`；ACTIVE 区间长 4.188 s，第一条至最后一条 PVAT 相隔 4.000 s。

| 项目 | 复算结果 | 定义与边界 |
|---|---:|---|
| 手部峰值位移 | 28.6666 mm | ACTIVE 区间内右手相对第一帧右手位置的最大欧氏距离 |
| 平移增益 | 0.2 | 状态日志直接记录 |
| 仅乘增益后的峰值 | 5.7333 mm | 上一位移乘 0.2；未包含死区、滤波和限幅，不是已发送目标 |
| PVAT | 76 条 | 33 次 `advance`、43 次 `catch_up` |
| IK 拒绝 | 44 次 | 43 次 `ik_tracking_diverged`、1 次 `ik_joint_jump` |
| 背压诊断 | 43 次 | `ik_tracking_backpressure` |
| 最后接受的目标 | command 6489 | 约 ACTIVE +1.766 s 接受；直到最后 command 6587 仍追赶此目标 |
| 最后接受目标位移 | 3.2790 mm | 相对于 ACTIVE 前最近状态样本中的 TCP；样本事件早于 ACTIVE 31 ms，样本年龄为 16 ms |
| 末尾状态报告的 TCP 位移 | 0.9558 mm | 同一 TCP 基准；该末尾样本年龄达 129375 ms，不能当作结束时的新观测 |
| 最后一条年龄小于 100 ms 的状态报告 | 0.9572 mm | 停止请求后 109093 ms，样本年龄 93 ms；仍只是遥测，不是物理静止证明 |

这些量的时间窗口不同，不能把表中的数值串成同一时刻的逐级损失百分比。统计样本年龄时排除 `null`，不将缺失年龄当作新鲜数据。

第一次背压拒绝为 command 6491：IK 目标相对当时实际 TCP 的平移差约 2.8625 mm、旋转差约 2.4779°，最大关节偏差 0.25603 rad，超过跟踪误差门限 0.25 rad。目标与实际关节逐渐分离后，后续 43 条 PVAT 沿用上一接受解；它们不是 43 个继续前进的新目标。现场关节速度上限为 0.05 rad/s，这个窗口内发送速度达到该上限。

代码路径：手部相对位姿 → `CoordinateMapper` → 工作空间投影 → 滤波 → 运动限幅 → 最新请求泵 → IK 选择 → PVAT。平移采用缩放，旋转通过相对姿态映射，因此手部旋转也会改变关节需求。控制层收到 `ik_tracking_lag` 后保持 `last_target` 并重置滤波/限幅状态；适配器可以继续追赶最后接受解。

**已证实：** 小平移增益、较大的关节跟踪误差、目标停止推进同时存在，旧文案把跟踪滞后归入了“运动变化过快”。**尚未证实：** 各级滤波和限幅分别消耗多少位移，以及小 TCP 变化对应较大关节变化的主因。日志没有完整记录每次映射、滤波、限幅的中间目标，也没有足以逐次复现现场 IK 分支选择的信息。不能仅据此认定奇异点、SDK 或控制器故障，也不据此放宽限制。

## 2. stop_move 晚返回

仅分析运动后的 Grip 释放停止事务。日志中另一次成功停止属于运动之前的 `stale`，不能合并为这次停止成功。

| 阶段 | 相对停止请求 |
|---|---:|
| 停止请求 | 0 ms |
| SDK await 开始 | +31 ms |
| RPC 记录的截止 | +231 ms（SDK 开始后 200 ms） |
| SDK await 完成 | +640 ms（SDK await 持续 609 ms） |
| 超过记录截止 | 409 ms |

结果是 `returned_late`、`wait_failed=true`，运动停止没有 `stop_confirmed`，`stop_unverified` 保留。日志仍读到约 +625 ms 的 `MOVING`，首次 `IDLE` 约 +671 ms；这些状态读数不是物理停止证明。

本次 31 ms 的前置等待无法解释 609 ms 的 SDK await。现有证据无法继续拆分 SDK 内部、网络传输与控制器处理时间。保留 200 ms 限制和锁存，不恢复自动 `stop_sys`，不通过扩大超时掩盖未确认停止。

## 3. 本次修改

新增独立约束值 `tracking_lag`，贯穿适配器、控制层、Python 模型、JSON Schema、TypeScript 校验和两处 UI。

- HUD：`机械臂尚未跟上目标`。
- VR 提示：`减小位移或旋转，等待跟随`。
- 跳变和速度限制仍对应 `motion_continuity_boundary`。
- 测试继续断言跟踪滞后时目标不推进、保持 ACTIVE、无新故障；适配器继续沿用旧解。

这次修正的是反馈分类。没有修改 Home、停止策略、运动增益、速度或 IK 门限，也没有解决真实跟踪和停止延迟。

**兼容性：** 协议 v1 增加枚举值；旧前端严格校验会拒绝含 `tracking_lag` 的状态消息。未来部署必须配套更新前后端并处理旧客户端缓存。本轮未部署。

## 4. 离线验证

| 范围 | 结果 |
|---|---|
| 控制层、LEBAI 适配器、协议模型三个受影响测试文件 | 321 passed |
| 停止事务、停止诊断、停止证据、超时生命周期 | 47 passed |
| 映射、滤波、IK 策略、PVAT | 48 passed |
| 协议、HUD、VR 安全面板、操作面板、XR 会话五个前端文件 | 161 passed |
| TypeScript `tsc --noEmit` | 通过 |
| Vite 生产构建 | 通过；JS 主包 743.26 kB，有大于 500 kB 的体积提示 |
| `git diff --check` | 通过 |

未运行后端全集。此前 Home 定向基线为 21 passed；前端操作面板与 VR 面板基线为 81 passed，与后续覆盖重叠，不累加为总数。

新增约束的测试先出现预期失败：后端 4 failed / 3 passed，前端 3 failed / 87 passed，分别暴露旧分类、枚举拒绝及缺失文案；实现后受影响文件全部通过。曾临时加入“B 只能停止”的两个用例，用户纠正需求后已删除；它们与既有设计冲突，不作为产品缺陷证据。

工具限制：Vite 默认沙箱构建因父目录读取被拒失败，获准在沙箱外执行本地构建后通过，未启动服务器。旧的跨轮构建进程结果无法恢复，已重新运行类型检查与构建取得确定结果。初次统计脚本遇到 `sample_age_ms=null` 后改用数值筛选再复算；未改写日志。

复跑命令（使用隔离环境的 Python 和项目已安装依赖；backend/web 分别为执行目录）：

```text
python -m pytest tests/control/test_robot_control.py tests/robots/test_lebai_adapter_control.py tests/contract/test_messages.py -q -o asyncio_default_fixture_loop_scope=function
python -m pytest tests/robots/test_stop_transaction.py tests/robots/test_lebai_stop_diagnostics.py tests/robots/test_lebai_stop_evidence.py tests/commissioning/test_stop_timeout_lifecycle.py -q
python -m pytest tests/control/test_coordinate_mapper.py tests/control/test_filters.py tests/robots/test_lebai_ik_policy.py tests/robots/test_lebai_pvat.py -q
node node_modules/vitest/vitest.mjs run tests/messages.test.ts tests/hud.test.ts tests/vrSafetyPanel.test.ts tests/armPanel.test.ts tests/xrSession.test.ts
node node_modules/typescript/bin/tsc --noEmit
node node_modules/vite/bin/vite.js build
```

## 5. 后续任务与验收

1. **P1：补齐离线运动链路可观测性。** 设计有采样上限、带 command_id 的映射/滤波/限幅阶段记录；用合成输入区分纯平移、纯旋转和组合动作。验收：可重算每级目标差异，记录失败不会放行运动，不上传现场原始位姿。
2. **P1：跟踪背压实验。** 用 fake 的慢反馈、固定 IK 解及分支变化，量化目标冻结和恢复；明确请求目标、接受解与实际发送关节点的区别。验收：恢复可推进，停止代次切换后旧请求不可发送；保持关节和碰撞限制。是否更换策略由离线证据决定。
3. **P1：停止延迟证据缺口。** 离线整理 SDK await 的时间边界与可观测接口，维持晚返回/取消/断连的锁存回归。现有日志不足以对 SDK、网络、控制器分摊耗时，涉及新连接或采集的实验另行申请现场授权。
4. **P2：正常 VR 参数候选。** 待前两项可量化后比较候选增益与平移/旋转需求；仅形成离线候选，不把诊断 profile 或未经验证的新参数部署到机械臂。
5. **未来现场验证。** 机械臂当前断电，未安排上电或动作。之后如申请测试，将另列目的、动作上限、停止条件、观察员与日志采集范围，等待用户明确授权。

当前离线任务不需要用户补充目录或再次解释 Home 事件。真实运动停止验收、现场姿态和参数批准仍留待后续现场决策。
