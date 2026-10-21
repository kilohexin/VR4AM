# 长按有限位置小步 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. 保留用户此前选择的当前会话直接实现方式，不派发实现子任务。

**Goal:** 在现有网页上提供可观察、可回归的离线 XYZ 长按有限小步：单个 movej 自然到位后续发，正常松开不再追加，异常仍走原停止。

**Architecture:** 新增输入门、有限位置适配器和会话协调器；通过显式 Fake 注入接到现有 RobotControl、WebSocket 和摇杆。正常释放与异常撤权分开，旧 PVAT／speedl 路径和默认配置不变。先做完整离线闭环，不新增真实地址运动入口。

**Tech Stack:** 现有 Python／asyncio、FastAPI／Pydantic、NumPy／SciPy、TypeScript／Vitest、原生 SDK 自有 127.0.0.1 回环；不升级或新增依赖。

**Spec:** [已认可设计](../specs/2026-10-21-held-finite-position-design.md)，原始设计提交 `6140672`；用户认可书面设计并要求继续实现计划。

## Global Constraints

- 首个候选只含 XYZ 平移，固定姿态；原厂 IK 是关节目标来源，视觉 IK 只限 Fake。
- 最大步长 0.5 mm，斜向按向量模限幅；不累计未执行目标，不形成未来点队列。
- 离线会话最多 10 步／10 秒；完成超时默认每步 2 秒；稳定至少 3 组且跨越 100 ms。准备位原 0.3 秒稳定时间不变。
- 新输入和实际发送前反馈均须小于 100 ms；正常释放后仅监控已发送步到固定截止，不伪造续租心跳。
- movej 的 t=0、r=0；回执期限沿用 200 ms，不改 speedl／PVAT 的 60 ms。异常原停止 200／300／500 ms、stop_unverified 和未知隔离不变。
- 完成要求自己的有效运动 ID 为 FINISHED、机器人 IDLE、无外部任务／故障，实际位姿到位且稳定；TCP 误差≤步长20%、沿方向进度≥70%、横向偏差≤20%。不使用旧准备位容差替代实际进度。
- 正常松开与回中结束续发；停止按钮／B、过期、失焦隐藏、断线、撤权和异常走原停止。通用 stop 消息不能被重新解释成正常释放。
- 初版平移与准备／Home／自转／夹爪互斥；新能力默认只允许显式 Fake，拒绝真实地址。现有界面不等于所有功能真机已验收。
- AGENTS.md 允许定向离线验证，不例行跑后端全集；不连接真机、不重试／复位／恢复、不推送；原始现场材料和配置不提交。
- 用户文件 `docs/project-overview-and-progress-2026-10-08.md` 不修改、不暂存。使用当前专用分支／检出，实施时先检查 using-git-worktrees 规则，不覆盖实验数据。

## Review Focus

1. pointercancel、失焦、页面隐藏和断线不能被 UI 公用 cancel 回调误标为正常松开。Task 4 钉住事件分类，Task 3 验证异常停止。
2. 松开后很快再按、旧消息迟到及 WebSocket 重连，不能使旧规划或旧会话继续写入。Task 1／3 验证释放屏障和连接代次。
3. SDK 返回 ID、FINISHED 或零速度，但实际位置完全不变，不能被旧到位容差判成成功。Task 2 验证实际进度，Task 5 通过真实 SDK 回环复现。
4. movej 已进入后回执超时或忽略取消；即使停止稍后返回，也不能发送第二目标、清锁存或无限等待。Task 2／3／5 验证迟到隔离。
5. 状态广播读取、准备和夹爪请求并发到达，不能绕过单写入仲裁或让停止等待普通工作锁。Task 3／4 验证互斥，保留旧模式回归。

## File Structure 与共用类型

新增代码按职责放置，不将新状态机堆入已有大型 adapter／RobotControl：

| 文件 | 职责 |
| --- | --- |
| `backend/app/control/finite_position_types.py` | 冻结输入、绑定、规划、结果、显示类型与候选常量 |
| `backend/app/control/finite_position_gate.py` | 最新输入、释放屏障、次数／期限，不做 I/O |
| `backend/app/robots/finite_position_plan.py` | TCP 小目标、IK/FK 核验与路径／速率估计，区分模型估计和硬件证据 |
| `backend/app/robots/finite_position_adapter.py` | 独立注入候选，复用原读取／调用所有权／停止；一笔 movej 与自然到位 |
| `backend/app/control/finite_position_session.py` | 单工作任务、长按续接、正常释放、异常中止和状态 |
| `backend/app/digital_twin/finite_position_demo.py` | 有真实位置推进的本机模拟及现有网页装配 |
| `web/src/ui/finitePositionInput.ts` | 候选输入编码、事件分类、序号；不积分虚拟目标 |
| `scripts/finite_position_loopback.py` | 固定自有回环协议矩阵，不接受任意地址 |
| `scripts/run_finite_position_demo.py` | 显式启动新离线网页，身份检查及使用说明 |

类型约定（实现为冻结 dataclass，元组内也不得藏可变 Pose／列表）：

- `PositionIntent(session_id: str, generation: int, seq: int, received_ns: int, kind: Literal['held','release'], vector: tuple[float,float,float])`；vector 模长包含强度，release 必须为零向量。
- `PositionBinding(connection: object, generation: int, config_sha256: str, tool_sha256: str)`；connection 只按对象身份比较，不上网传输。
- `PositionSample(binding: PositionBinding, sequence: int, captured_ns: int, actual_q: JointVector, target_q: JointVector, actual_qd: JointVector, actual_qdd: JointVector, target_qd: JointVector, target_qdd: JointVector, tcp_p: Vec3, tcp_q: Quat, robot_state: BackendState, running_id: int|None, task_state: str|None, fault: str|None)`；数据由己方已知完成读取转换，禁止接收客户端伪造反馈。
- `PositionPlan(binding: PositionBinding, intent_seq: int, start: PositionSample, target_p: Vec3, target_q: Quat, goal_joints: JointVector, joint_speed: float, joint_acceleration: float, created_ns: int, evidence: PathAudit)`；不可复用到另一单步。
- `PathAudit(model_sha256: str, sample_fractions: tuple[float,...], workspace_ok: bool, joint_limits_ok: bool, model_rates_ok: bool, factory_calibrated: bool, joint_speed: float, joint_acceleration: float, tcp_speed_estimate: float, tcp_acceleration_estimate: float)`；采样固定为 0／0.25／0.5／0.75／1，任一 *_ok 为False、缺失或绑定不符即拒绝；factory_calibrated=False须保留并禁止真机准入，允许已标注的Fake估计。Plan的joint_speed／acceleration必须与本audit完全一致。
- `PositionResult(complete: bool, motion_id: int|None, entered: bool, remote_unknown: bool, final_sample: PositionSample|None, reason: str|None)`；完成不能仅由 SDK 返回决定。
- `PositionView(phase, generation, entered_steps, motion_id, reason, hardware_verified=False)`；phase 限于 inactive／ready／planning／executing／draining／hold／fault，不与旧 target_control 联合类型混用。

所有新数值入口拒绝 bool、非有限值、溢出、形状错误；上网计数为非负安全整数，严格拒绝额外字段。原因码采用固定白名单，不输出任意厂商异常文本。

## Task 1: 输入门和不可变契约

**Files:** Create `backend/app/control/finite_position_types.py`、`backend/app/control/finite_position_gate.py`；Test `backend/tests/control/test_finite_position_gate.py`。

**Interfaces:** `PositionGate(binding, *, clock)`；`offer(intent: PositionIntent) -> None`；`take_latest() -> PositionIntent|None`；`require_entry(intent: PositionIntent) -> None`；`mark_entered(intent: PositionIntent) -> None`；`finish() -> None`；`revoke(reason: str) -> None`。Gate 保存最多一个最新输入和一个活动输入，不创建任务。首次新鲜输入确定不可续租的 10 秒会话截止；第十步后 HOLD，须释放并显式开始新手势才能建立新会话。完全相同向量的新持按输入可作为新鲜权限核对，不反复作废同方向数学规划；改向／release必须作废未发送旧规划。日志分别记录规划输入seq与实际授权seq，不延长规划／反馈的100ms期限。

- [x] **Step 1: 写失败测试。** `test_input_vector_norm_is_clamped` 断言(1,1,1)归一化而不逐轴累加；`test_release_fences_older_hold` 依次提供 held seq1、release seq3、held seq2，断言旧seq被拒绝且零新写入许可；`test_release_does_not_cancel_entered_step` 断言已进入步可 finish，但没有下一步。增加 99／100 ms 边界、旧连接／代次、倒退序号／时钟、非法值、零输入、10 步／10 秒、到位前重按不能提前进入及同方向高频更新不饿死规划的用例。最小断言：

```python
def test_release_leaves_no_next_goal():
    binding = PositionBinding(object(), 1, 'a'*64, 'b'*64)
    gate = PositionGate(binding, clock=lambda: 0)
    gate.offer(PositionIntent('web', 1, 1, 0, 'held', (1., 0., 0.)))
    gate.offer(PositionIntent('web', 1, 2, 0, 'release', (0., 0., 0.)))
    assert gate.take_latest() is None
```
- [x] **Step 2: 运行 RED。** 仓库根：`.venv/Scripts/python.exe -m pytest -c backend/pyproject.toml backend/tests/control/test_finite_position_gate.py -q`，预期新模块／契约尚不存在导致失败；记录实际失败，不能只假设 RED。
- [x] **Step 3: 实现类型与 Gate。** 正常释放增加独立屏障，不取消在途监控；异常 revoke 清空后续输入并给会话留下固定原因。监控权限由Task3处理。旧输入被替换不排队；零向量按释放处理，不累计小输入为大目标。
- [x] **Step 4: 同命令运行 GREEN。** 所有输入门用例通过；检查无 SDK／网络导入。
- [x] **Step 5: 仅暂存本任务文件并提交** `feat: add held position input gate`。

## Task 2: 有限位置规划与单步自然到位

**Files:** Create `backend/app/robots/finite_position_plan.py`、`backend/app/robots/finite_position_adapter.py`；Test `backend/tests/robots/test_finite_position_plan.py`、`backend/tests/robots/test_finite_position_adapter.py`。读取复用 `lebai_adapter.py`、`lebai_motion_calls.py`、`lebai_snapshot_calls.py`、`lebai_ik_policy.py`；不重写原 prepare／stop 事务。

**Interfaces:**

- `build_position_target(sample: PositionSample, intent: PositionIntent) -> tuple[Vec3,Quat]`，固定姿态，最大模长 0.0005。
- `async plan_position_step(adapter, intent: PositionIntent, *, authorize: Callable[[],None]) -> PositionPlan`，读取新起点、原厂 IK 和五个 FK 样本；绑定相同工具／关节源。不得调用旧 `_build_advancing_candidate` 将结果强制变为 80 ms PVAT。
- `audit_position_path(start: PositionSample, goal_joints: JointVector, *, fk_samples: tuple[tuple[Vec3,Quat],...], settings, model: PositionPathModel) -> PathAudit`，只进行显式模型估计。`PositionPathModel`协议同文件定义，提供model_sha256／tool_sha256／factory_calibrated和 `forward(q: JointVector) -> tuple[Vec3,Quat]`、`jacobian(q: JointVector) -> np.ndarray`、`jdot(q: JointVector, qd: JointVector) -> np.ndarray`。离线模型先与FK样本交叉检查，不能用视觉模型估计覆盖原厂FK。初始请求joint v／a取配置且分别≤.05 rad/s／.2 rad/s²；若五点模型TCP估计超过配置，则v按1/2、a按1/4同步收缩，至多10轮，不可行或预计无法在2秒内完成即拒绝。估计保留模型、关节插值及采样假设，不声称这是控制器全轨迹约束。不得让调用方塞入全True的audit直接运动。
- `evaluate_position_arrival(plan: PositionPlan, sample: PositionSample, *, motion_id: int) -> str|None`，None 表示该样本满足全部条件，不代表已经连续稳定。
- `FinitePositionAdapter(RealLebaiAdapter)` 构造必须显式 `backend_label='LEBAI_FAKE'`、注入 factory、`settings.ip='digital-twin'`；其它组合初始化即拒绝。`async plan_step(intent, *, authorize) -> PositionPlan`；`async execute_step(plan, *, before_entry, monitor_authorize, on_entered) -> PositionResult`；`stop(reason)` 继承原事务。`on_entered()` 在实际原生入口且只调用一次。

- [x] **Step 1: 写失败测试。** `test_one_move_reaches_goal_without_stop` 断言 `result.complete is True`、`result.entered is True`、`result.remote_unknown is False`、`[call[0] for call in client.write_calls] == ['movej']`；Fake实际位置必须渐进变化，有效ID及3个稳定样本跨100ms。`test_finished_without_actual_progress_is_rejected` 令ID为FINISHED、IDLE、速度0但位置不变，断言 `result.complete is False` 且没有第二目标。`test_diagonal_target_is_half_millimetre` 断言 `math.dist(target_p, sample.tcp_p) <= .0005`。覆盖错误方向／横向>20%／终点误差>20%／进度<70%、任务仍RUNNING／PAUSED／外部ID、工具变化、断续采样不能拼成稳定区间、100ms发送边界、IK None／跳支／关节限位／路径越框、模型绑定错、极小步无法分辨和不可达速率。
- [x] **Step 2: 对上述两个新测试文件运行 RED。** 命令形式同 Task 1，明确期望因新规划／适配器缺失而失败。
- [x] **Step 3: 实现规划与到位纯函数。** 从实际 TCP 生成目标；IK 种子为实际关节。五点 FK 检查姿态、绝对工作区、关节路径；工具／配置绑定固定。路径速率估计使用显式绑定模型的 J／Jdot，记录采样及未校准边界；按配置和模型约束降低请求 joint v／a，不能放宽限制或以遥测代替规划。无法建立估计时拒绝。不得直接使用仅适用10–80 ms的 `native_jog_region` 准入函数；不得把离线模型估计升级为硬件路径证明。
- [x] **Step 4: 实现单步 SDK 所有权。** 复用己方快照／读取调用所有权、SDK锁和原未知隔离；原生入口前执行 before_entry、绑定／配置／工具和100 ms反馈复查。movej 200 ms 有界等待，合法正整数 ID（不接受 bool），迟到／取消不当作远端取消。后续使用 monitor_authorize；正常释放不使已发送步失权。任务查询、实际进度和连续稳定全部通过才解除本步运动义务；完成截止为入口+2秒与会话截止较早者。
- [x] **Step 5: 增加异常用例并运行 GREEN。** stop前已知零写入可零停止拒绝；进入后超时／撤权／取消／坏读取走原唯一 stop，并保留 stop_unverified。忽略取消的 movej 与迟到 ID、停止迟到后新生效均不得恢复权限；未结束的原生任务被隔离，有界收尾不无限 await。适配器禁止 prepare／claw／第二movej 穿过该步所有权。观察任务 FINISHED 但 retained running ID 仍有值时，只接受明确属于本步且状态已完成的 ID，不全局忽略编号。
- [x] **Step 6: 回归与提交。** 新增两文件加 `backend/tests/commissioning/test_ready_return.py`、`backend/tests/robots/test_stop_transaction.py`、`backend/tests/robots/test_stop_timing_boundaries.py`、`backend/tests/robots/test_stop_priority.py`。定向全部通过后提交 `feat: add isolated finite position step executor`。

## Task 3: 会话协调、协议与控制仲裁

**Files:** Create `backend/app/control/finite_position_session.py`；Modify `backend/app/control/robot_control.py`、`backend/app/api/teleop_ws.py`、`backend/app/main.py`、`backend/app/schemas/messages.py`、`schemas/teleop-v1.json`；Test Create `backend/tests/control/test_finite_position_session.py`、`backend/tests/api/test_finite_position_ws.py`；扩展 `backend/tests/contract/test_messages.py`。

**Interfaces:** `FinitePositionSession(adapter: FinitePositionAdapter, binding: PositionBinding, *, clock, recorder, authorize)`；`offer(intent: PositionIntent) -> None`（同步无阻塞）；`async advance() -> None`（至多启动／收集一个己方工作任务，不等待整段完成）；`async abort(reason: StopReason) -> None`；`view() -> PositionView`。RobotControl 新增默认 None 的 `finite_position_session` 参数和 `accept_position_input(message, received_ns: int) -> None` 方法；main 新增默认 False 的 `offline_finite_position` 显式注入选择，与 transactional／segmented 同时选择即拒绝。

协议：客户端 `v:1,type:'finite_position_input',session_id, generation,seq,kind:'held'|'release',vector:[x,y,z]`。接收端填 received_ns，不信任客户端时间作为新鲜度依据；完整意图仅主操作 WebSocket 可发，释放同样受序号／代次约束。`RobotStateMessage.finite_position` 为可选独立状态，含 policy='finite_position_offline'、phase、generation、entered_steps、motion_id、reason、hardware_verified=false；不改变旧 target_control 格式。

- [x] **Step 1: 写失败测试。** `test_held_release_drains_without_second_move`、`test_repress_waits_for_completion_and_new_input`、`test_unfinished_step_never_queues_next_goal`；输入50Hz、Fake每步600ms，期间 movej最多1，完成后从新反馈取最新方向。覆写3个数据源证明目标不累计。新协议误送 legacy／真实模式即拒绝。
- [x] **Step 2: 运行新会话／WS／协议用例 RED。** 使用根目录 Python命令并显式列出三个测试文件。
- [x] **Step 3: 实现会话调度及 tick 分流。** 单个后台工作任务执行 plan_step／execute_step；Task 1 Gate只控制新写入，monitor_authorize单独检查连接、代次、配置、固定会话截止、异常撤权和记录健康。RobotControl.tick 保留原恢复／撤权检查，然后仅在显式候选里调用新 advance，不让旧 `_consume_grip_release`／`_tick_frame` 再处理该候选手势；旧路径不变。新adapter声明supports_prepare_on_arm=True；main仅在offline_finite_position分支开启prepare_on_arm／explicit_preparation，不依赖segmented配置。prepare成功后才安装绑定；原状态机ACTIVE／HOLD由候选入口／释放同步维护，不能被中性VR帧重新触发运动。准备／Home／claw请求与活动步互斥，异常abort则允许原停止越过普通仲裁。原故障／stop_unverified仍回传到共享控制状态，不创建新适配器绕过锁存。
- [x] **Step 4: 接收不等待运动，记录后再续发。** WS将新消息快速送入输入门，不能 await execute_step阻塞释放；日志失败立即撤权，已进入则原停止。持按期间monitor_authorize及每次advance均检查输入年龄，≥100ms且未收到有效release就异常abort；已经有效release则只监控当前步到固定截止。会话与连接关闭时隔离晚到结果，先有界收尾再退出。完成一段还必须有完成之后收到的新鲜持按输入才可续发；同一旧输入不能跨步复用。
- [x] **Step 5: 增加异常／仲裁测试并运行 GREEN。** stop/B、显式失焦、过期、断线、配置变化、记录失败在规划中／原生入口／执行中各覆盖；同时 prepare、claw、Home、普通state读取不得增加运动写入。10步后持按不自动新建会话；10秒已在途时异常停止。重连旧seq／旧任务拒绝；release后不因100ms自然过去而错误停止。正常release后的抖动／任务失败仍走异常停止。旧 WS stop 和准备回归通过。
- [x] **Step 6: 只提交本任务代码／测试** `feat: coordinate bounded held position sessions`。

## Task 4: 接入现有网页摇杆与可运行 Fake

**Files:** Create `web/src/ui/finitePositionInput.ts`、`backend/app/digital_twin/finite_position_demo.py`、`scripts/run_finite_position_demo.py`；Modify `web/src/ui/holdControlPanel.ts`、`web/src/transport/teleopSocket.ts`、`web/src/protocol/messages.ts`、`web/src/main.ts`；Test Create `web/tests/finitePositionInput.test.ts`、`backend/tests/digital_twin/test_finite_position_demo.py`、`backend/tests/scripts/test_finite_position_delivery.py`；扩展 `web/tests/holdControlPanel.test.ts`、`web/tests/messages.test.ts`、`web/tests/teleopSocket.test.ts`。

**Interfaces:** `FinitePositionInput` 只负责 `offer(vector: Vec3): void`、`release(): void`、`abort(): void`、`reset(sessionId: string, generation: number): void`，通过构造参数 `send` 发送候选消息。`HoldControlPanel` 新增可选候选输出回调和事件原因；旧构造参数兼容，legacy applyFrame 不改变。候选帧不再积分手柄位置为累计目标，也不能同时给 legacy 映射器发送 grip=true。`TeleopSocket.sendPositionInput(message): void` 使用当前连接一次发送，不缓存重放。

`create_finite_position_demo_app(web_directory: Path)` 显式装配 Fake；`FinitePositionDemoClient` 复用 SegmentedDemoClient 的本机 FK／IK及路径检查，但独立允许本次准入的小步 movej，准备位仍走原prepare。模拟实际 q／qd／qdd 平滑推进，任务先RUNNING后FINISHED；不能直接采用父类 t=0 时固定80ms完成的简化模型。模拟时长由关节距离、请求／配置速度和加速度计算，不改真实发送的 t=0。全部标注未校准、本机动画、非物理引擎。

- [x] **Step 1: 写失败前端与 Fake 测试。** pointerup／摇杆回中产生release，pointercancel／失焦／隐藏／capture异常产生abort；旧构造不出现候选消息。候选运行时松开文案为“松开后完成当前小步”，阶段显示“正在完成当前小步”，stop按钮仍发送旧stop。模拟有中间实际位移，不能只更新target；零速度字段不能掩盖位置变化。
- [x] **Step 2: 运行 RED。** 在web目录执行 `npm test -- tests/finitePositionInput.test.ts tests/holdControlPanel.test.ts`；根目录执行Python的新增Fake／delivery两文件，预期功能缺失失败。
- [x] **Step 3: 实现前端候选接线。** 只在明确finite_position状态、Fake身份、已准备且本页面为操作者时启用；消息使用当前session和服务器代次。实际处理中的持按采样按原动画帧频率更新，不使用后台setInterval给失效手势续命。spin候选中禁用；夹爪在无活动步、无故障且IDLE时保留渐进开合，原输入和取消流程不变。XR进入先abort候选，不声称VR已接入。
- [x] **Step 4: 实现独立离线启动器。** 仅绑定127.0.0.1，拒绝真实配置／确认环境变量和已加载原生SDK；无机器人地址参数。默认不替换现有8768服务、不结束其它进程；显式演示端口8769若被占用即报告而不抢占。沿用现有网页／桌面模型，不新增方块、浮空平台或离线演练界面。identity返回finite_position_offline、LEBAI_FAKE、hardware_verified=false、physics_engine=false；使用现有构建产物并记录源码身份。
- [x] **Step 5: 运行 GREEN 和构建。** 上述前端文件加messages／teleopSocket定向用例；Python的Fake／delivery用例；`npm run build`。实际浏览器检查：解锁不运动→独立准备→长按XY→松开排空→反向→Z→松开→无在途时渐进夹爪；另查隐藏／失焦／停止分流。通过可用浏览器工具先读取本机页面身份；只操作8769的Fake，不打开L Master或任何真实目标。保存输入／写入计数、阶段、位置和日志对照，不以截图判定实际机器人行为。
- [x] **Step 6: 提交** `feat: wire finite position control into offline web demo`。

## Task 5: 原生自有回环与整条离线交付复核

**Files:** Create `scripts/finite_position_loopback.py`、`backend/tests/scripts/test_finite_position_loopback.py`、`docs/held-finite-position-offline-2026-10-21.md`。复用 `scripts/sdk_loopback_stop_probe.py` 的自有监听、`scripts/native_jog_loopback_probe.py` 的端口／请求校验；不改SDK二进制、不新增真机CLI。

**Interfaces:** `async run_case(case: str, output: Path) -> dict`，case固定为 normal／release_before_entry／release_during_step／reverse_latest／no_actual_progress／foreign_task／move_reply_late／stop_reply_late／late_effect；CLI只有--case（上述枚举或all）及--output，地址固定127.0.0.1，连接端口来自本进程创建且仍持有的监听。生产候选适配器继续拒绝127.0.0.1地址选项；脚本通过显式注入的己方端点包装器进行回环，不提供外部地址逃生开关。

- [x] **Step 1: 写脚本边界失败测试。** 拒绝--ip／外部地址／未经己方监听验证的端口；验证源码绑定及既有SDK SHA；异常case不能输出complete=true；输出不含真实配置或原始现场记录。
- [x] **Step 2: 运行 RED。** 仅新增脚本测试文件，应因入口缺失而失败。
- [x] **Step 3: 实现协议矩阵。** 模拟端按实际SDK序列化校验movej关节／a／v／t=0／r=0、任务ID、读取及唯一异常stop。normal至少两步有实际中间反馈；release期间只完成一段、不追加；FINISHED但未动必须失败；晚返／忽略取消／停止后延迟生效保留remote_unknown及原锁存。每case独立生命周期，没有失败自动重试。
- [x] **Step 4: 运行 GREEN、回环及最终定向集合。** 先新增脚本测试通过，再 `.venv/Scripts/python.exe scripts/finite_position_loopback.py --case all --output .tmp/finite-position-offline-20261021/loopback`；列出Task1–4新增用例和受影响现有准备／WS／停止／gripper回归，不运行后端全集；前端定向用例和构建再核对一次最终代码。长耗时命令通过非阻塞会话等待并继续汇报。
- [x] **Step 5: 独立复核与修正。** 按requesting-code-review技能，对本轮产品差异、设计和最终证据做整轮复核；如可用reviewer不足，明确记录未完成，不虚报通过。不派发实现子任务、不要求用户逐个批准局部修正；规格之外的控制语义变更才回到设计确认。修正仍跑受影响定向用例。
- [x] **Step 6: 保存结果与提交。** 文档列出原始证据本机路径／SHA、步数、实际进度、正常release残余采样／延迟、观测间隙、异常case、网页体验和未通过项。明确“离线小步可用／真机未验收”，不能称停止故障已修复。通过后本地提交 `test: verify held finite position offline flow`，不推送。

## 完成与下一次现场决策

本计划完成物为：可运行的现有网页新候选、一次小步与长按行为的定向证据、真实SDK自有回环、可追溯结果。不是只有Fake成功计数或孤立诊断入口。实现时逐项勾选，报告失败和实际写入／位移，不拿进程退出当远端结果已知。

本轮不生成或执行新的真机批准材料。离线实际跟随和自然完成足够可信后，才准备单步真机范围；任何连接／运动／收尾仍需新的明确授权。单步／持续网页／夹爪／VR／物理仿真分别报告进度，历史停止确认与速度证据不足继续保留。

## 计划自查记录

- Spec覆盖：输入／所有权Task1，规划／自然完成Task2，续接／异常／互斥Task3，页面／FakeTask4，协议／证据Task5；不引入未审批真实入口。
- 类型／接口：跨任务使用同一PositionIntent／Binding／Sample／Plan／Result／View，Web状态独立于旧target_control；旧构造调用兼容。
- 五项Review Focus均有明确所属用例；正常释放不取消已发命令、旧通用stop不被截获、零实际进度不通过、未知结果不能再次进入。
- 参数与权限：0.5mm、10步／10秒、2秒完成、3样本／100ms、100ms发送／输入、200ms movej回执、原停止判据逐项保留；模型采样不宣称物理边界。
- 执行方式：沿用用户此前选择，由当前会话连续实现。writing-plans技能要求用户审阅本计划后才开始编码；本文件不构成真机授权。

## 2026-10-21 执行结论

任务1–5已执行，本地提交8879d3e、13d1c92、feec48d、7c13c4c、c521e5a及结果归档提交。勾选Step5表示已发起审查、完成作者修正并披露不足：独立审查因额度耗尽未交付最终报告，不代表独立复核通过。详见[实现结果、原始证据哈希、裁定与保留项](../../held-finite-position-offline-2026-10-21.md)。Python两组定向集合194+130通过，前端60通过，SDK自有回环9/9，最终桌面流程与窄屏检查完成。旧background集合11失败、抽查基线复现、审查不完整均保留；无真机连接或推送。
