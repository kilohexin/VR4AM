# Simulation Teleoperation Responsiveness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让仿真遥操作在工作空间或 IK/关节连续性边界附近仍保持单连接、逐轴可控和反向可恢复，并提高末端跟手速度，同时保持真机安全行为不变。

**Architecture:** 保留现有普通 VRFrame、单一 TeleopSocket、RobotControl 状态机和 Fake 乐白适配链路。在 SafetyLimiter 增加由运行时选择的 `hold`/`axis_clamp` 投影策略；在 `LEBAI_FAKE` 适配器中增加纯函数式位姿回退和可恢复软约束；用 Fake 专属配置提高响应；HUD 只优化恢复说明，不改变消息协议。真实 `LEBAI` 标签始终强制 `hold` 和现有连续失败升级行为。

**Tech Stack:** Python 3.11、FastAPI/WebSocket、Pydantic、NumPy、SciPy Rotation/Slerp、pytest/pytest-asyncio、TypeScript、Three.js、Vitest、Vite。

## Global Constraints

- 只有 `LEBAI_FAKE` 可以使用逐轴软限位、IK 位姿回退和新的响应参数；`LEBAI` 真机路径不得放宽。
- 不创建第二条控制连接，不增加观察端，不绕过 RobotControl，不自动重设坐标锚点或手柄零位。
- 控制环保持 50 Hz，状态和 PVAT 保持 25 Hz；stale、断线停止、自碰撞、SDK 超时和停止未确认策略不变。
- Fake 可恢复错误仅限 `ik_unreachable`、`ik_invalid`、`ik_joint_limit`、`ik_joint_jump`、`joint_speed_limit`。
- SDK 调用失败、录制失败、控制循环异常和未知异常继续 fail closed。
- 所有实现遵循 TDD：先看到针对缺失行为的失败，再写最小生产改动。
- 不连接真实乐白 SDK、不接触真机、不执行 Git push。
- 设计规格：`docs/superpowers/specs/2026-08-12-simulation-teleoperation-responsiveness-design.md`。

## File Map

- `backend/app/control/safety.py`：纯安全空间投影和动态限速；新增 `WorkspaceBoundaryMode`，不感知 WebSocket 或机器人类型。
- `backend/app/main.py`：根据 `RuntimeBackend` 选择投影策略，确保真实 `LEBAI` 强制 `hold`。
- `backend/app/robots/lebai_recovery.py`：新增纯位姿插值/候选生成模块，不访问 SDK、不保存状态。
- `backend/app/robots/lebai_adapter.py`：拥有上次安全 TCP、尝试候选、区分 Fake 可恢复约束与真实后端永久错误。
- `backend/app/digital_twin/lebai_client.py`：Fake SDK 的 IK 自碰撞结果必须返回不可达，不伪造可行关节目标。
- `config/fake-lebai.yaml`：仅仿真的响应参数。
- `web/src/ui/hud.ts`、`web/src/scenes/vrSafetyPanel.ts`：桌面 HUD 和 VR HUD 的恢复文案。
- `backend/tests/digital_twin/test_responsive_teleop.py`：单控制 WebSocket 的真实 Fake 栈耐久回归。

---

### Task 1: Fake-only 逐轴工作空间投影

**Files:**
- Modify: `backend/app/control/safety.py`
- Modify: `backend/app/main.py`
- Modify: `backend/tests/control/test_safety.py`
- Modify: `backend/tests/control/test_robot_control.py`
- Modify: `backend/tests/digital_twin/test_runtime.py`

**Interfaces:**
- Produces: `WorkspaceBoundaryMode = Literal["hold", "axis_clamp"]`。
- Produces: `SafetyLimiter(..., workspace_boundary_mode: WorkspaceBoundaryMode = "hold")`。
- Changes: `_build_limiter(settings: Settings, runtime_backend: RuntimeBackend) -> SafetyLimiter`。
- Preserves: `WorkspaceProjection(pose, constrained, hold)` 和 RobotControl 的调用方式。

- [ ] **Step 1: 写逐轴投影的失败测试**

在 `backend/tests/control/test_safety.py` 增加以下行为测试：

```python
def test_axis_clamp_limits_only_violating_axes_and_preserves_rotation() -> None:
    anchor = Pose(p=(0.3, 0.0, 0.3), q=IDENTITY)
    requested_q = tuple(Rotation.from_euler("z", 20, degrees=True).as_quat())
    limiter = SafetyLimiter(
        workspace_half_extent_m=0.10,
        max_rotation_from_anchor_rad=np.deg2rad(45),
        workspace_boundary_mode="axis_clamp",
    )
    limiter.set_pose_anchor(anchor)

    projection = limiter.project_workspace(
        Pose(p=(0.45, 0.04, 0.27), q=requested_q),
    )

    assert projection.constrained is True
    assert projection.hold is False
    assert projection.pose.p == pytest.approx((0.4, 0.04, 0.27))
    assert abs(sum(a * b for a, b in zip(projection.pose.q, requested_q))) == pytest.approx(1.0)


def test_axis_clamp_projects_rotation_and_reverse_request_is_unconstrained() -> None:
    limiter = SafetyLimiter(
        workspace_half_extent_m=0.10,
        max_rotation_from_anchor_rad=np.deg2rad(30),
        workspace_boundary_mode="axis_clamp",
    )
    limiter.set_pose_anchor(Pose(p=(0.3, 0.0, 0.3), q=IDENTITY))
    outside = limiter.project_workspace(Pose(
        p=(0.3, 0.0, 0.3),
        q=tuple(Rotation.from_euler("z", 45, degrees=True).as_quat()),
    ))
    reverse = limiter.project_workspace(Pose(
        p=(0.39, 0.0, 0.3),
        q=tuple(Rotation.from_euler("z", 25, degrees=True).as_quat()),
    ))

    assert outside.constrained is True
    assert outside.hold is False
    assert Rotation.from_quat(outside.pose.q).magnitude() == pytest.approx(np.deg2rad(30))
    assert reverse.constrained is False
```

保留现有 `test_optional_box_envelope_constrains_each_axis_without_changing_defaults`，并补充 `assert outside.hold is True`，证明默认真机语义没有变化。

- [ ] **Step 2: 运行 SafetyLimiter RED**

Run:

```powershell
cd backend
python -m pytest tests/control/test_safety.py -q
```

Expected: 新测试因 `workspace_boundary_mode` 参数不存在而失败；现有测试通过。

- [ ] **Step 3: 实现 `hold` 与 `axis_clamp` 投影**

在 `backend/app/control/safety.py`：

```python
from typing import Literal

WorkspaceBoundaryMode = Literal["hold", "axis_clamp"]
```

构造函数验证模式只接受两个固定值。`project_workspace()` 的顺序固定为：逐轴盒状投影、相对锚点旋转投影、球形半径投影。`hold` 遇到盒状或旋转越界时继续返回原请求且 `hold=True`；`axis_clamp` 使用：

```python
projected_target = np.clip(
    target,
    self.anchor - self.workspace_half_extent_m,
    self.anchor + self.workspace_half_extent_m,
)
delta = requested_rotation * self.anchor_rotation.inv()
angle = delta.magnitude()
projected_rotation = (
    Rotation.from_rotvec(delta.as_rotvec() * (limit / angle))
    * self.anchor_rotation
)
```

返回标准化的 `Pose`，只要发生任一投影就设置 `constrained=True, hold=False`。

- [ ] **Step 4: 增加运行时隔离 RED**

在 `backend/tests/digital_twin/test_runtime.py`：

```python
def test_runtime_selects_axis_clamp_only_for_fake_backend() -> None:
    settings = load_digital_twin_settings()
    fake = app_main._build_limiter(settings, "LEBAI_FAKE")
    real = app_main._build_limiter(settings, "LEBAI")

    assert fake.workspace_boundary_mode == "axis_clamp"
    assert real.workspace_boundary_mode == "hold"
```

Run:

```powershell
cd backend
python -m pytest tests/digital_twin/test_runtime.py::test_runtime_selects_axis_clamp_only_for_fake_backend -q
```

Expected: `_build_limiter` 当前不接受 `runtime_backend`，测试失败。

- [ ] **Step 5: 把运行标签传入 limiter 工厂**

在 `backend/app/main.py` 修改为：

```python
def _build_limiter(
    settings: Settings,
    runtime_backend: RuntimeBackend,
) -> SafetyLimiter:
    ...
    workspace_boundary_mode=(
        "axis_clamp" if runtime_backend == "LEBAI_FAKE" else "hold"
    )
```

`create_app()` 构造 RobotControl 时调用 `_build_limiter(runtime_settings, runtime_backend)`。即使真实配置文件被误写，`LEBAI` 标签也不能启用 `axis_clamp`。

- [ ] **Step 6: 证明 RobotControl 会发送夹紧后的目标并允许其他轴继续**

在 `backend/tests/control/test_robot_control.py` 保留现有真机 hold 测试，并增加 Fake-style limiter 测试。测试先发送 X/Y 同时越界的帧，再发送 X 继续越界但 Y 反向变化的帧；断言两次均调用 `backend.command_tcp`、X 均不超过锚点边界、第二次 Y 继续变化、状态为 ACTIVE 且约束为 `workspace_boundary`。随后发布范围内反向帧并推进 `constraint_clear_ms`，断言约束清除且 mapper 锚点没有改变。

- [ ] **Step 7: 跑 Task 1 全部测试**

Run:

```powershell
cd backend
python -m pytest tests/control/test_safety.py tests/control/test_robot_control.py tests/digital_twin/test_runtime.py -q
```

Expected: 全部 PASS。

- [ ] **Step 8: 本地提交 Task 1**

```powershell
git add -- backend/app/control/safety.py backend/app/main.py backend/tests/control/test_safety.py backend/tests/control/test_robot_control.py backend/tests/digital_twin/test_runtime.py
git diff --cached --check
git commit -m "feat: add simulation axis soft limits"
```

---

### Task 2: Fake IK 回退与 PVAT 单连接稳定性

**Files:**
- Create: `backend/app/robots/lebai_recovery.py`
- Modify: `backend/app/robots/lebai_adapter.py`
- Modify: `backend/app/digital_twin/lebai_client.py`
- Create: `backend/tests/robots/test_lebai_recovery.py`
- Modify: `backend/tests/robots/test_lebai_adapter_control.py`
- Modify: `backend/tests/digital_twin/test_lebai_client.py`

**Interfaces:**
- Produces: `RECOVERY_FRACTIONS: tuple[float, ...] = (1.0, 0.75, 0.5, 0.25)`。
- Produces: `interpolate_pose(start: Pose, requested: Pose, fraction: float) -> Pose`。
- Produces: `recovery_candidates(last_safe: Pose | None, requested: Pose, *, fake: bool) -> tuple[tuple[float, Pose], ...]`。
- Adds adapter state: `_last_sent_tcp: Pose | None`；停止、Home、断线、故障或代际失效时清空。
- Preserves: `RobotBackend.command_tcp(target, command_id)` 和 WebSocket 消息结构。

- [ ] **Step 1: 写纯位姿候选 RED**

创建 `backend/tests/robots/test_lebai_recovery.py`：

```python
def test_fake_recovery_candidates_are_ordered_nearest_request_first() -> None:
    start = Pose(p=(0.30, 0.00, 0.40), q=(0.0, 0.0, 0.0, 1.0))
    requested = Pose(
        p=(0.34, 0.04, 0.40),
        q=tuple(Rotation.from_euler("z", 40, degrees=True).as_quat()),
    )

    candidates = recovery_candidates(start, requested, fake=True)

    assert [fraction for fraction, _ in candidates] == [1.0, 0.75, 0.5, 0.25]
    assert candidates[1][1].p == pytest.approx((0.33, 0.03, 0.40))
    assert Rotation.from_quat(candidates[1][1].q).magnitude() == pytest.approx(
        np.deg2rad(30)
    )


def test_real_or_missing_history_attempts_only_original_target() -> None:
    requested = Pose(p=(0.34, 0.04, 0.40), q=(0.0, 0.0, 0.0, 1.0))
    assert recovery_candidates(None, requested, fake=True) == ((1.0, requested),)
    assert recovery_candidates(requested, requested, fake=False) == ((1.0, requested),)
```

Run:

```powershell
cd backend
python -m pytest tests/robots/test_lebai_recovery.py -q
```

Expected: 模块不存在，collection 失败。

- [ ] **Step 2: 实现纯候选模块**

`interpolate_pose` 拒绝非 `[0, 1]` 有限 fraction。平移用 NumPy 线性插值；旋转用：

```python
rotation = Slerp(
    [0.0, 1.0],
    Rotation.from_quat([start.q, requested.q]),
)([fraction])[0]
```

输出 Pydantic `Pose`，因此四元数继续经过协议归一化验证。模块不导入适配器或数字孪生客户端。

- [ ] **Step 3: 写 Fake 连续失败和真实隔离 RED**

修改 `_connected_control_adapter`，增加 `backend_label: Literal["LEBAI", "LEBAI_FAKE"] = "LEBAI"` 并传给适配器。保留现有 `test_one_ik_miss_recovers_but_five_consecutive_misses_fault` 作为真实 `LEBAI` 行为证明，再增加：

```python
@pytest.mark.asyncio
async def test_fake_repeated_ik_misses_remain_soft_and_keep_pump_running() -> None:
    adapter, client, _ = await _connected_control_adapter(
        backend_label="LEBAI_FAKE",
    )
    client.ik_results = deque([None] * 10)

    for command_id in range(1, 11):
        await _send_and_wait_for_ik(adapter, client, command_id)

    assert adapter.constraint == "ik_boundary"
    assert adapter.pump_fault is None
    assert adapter._pump.running is True
    await adapter.get_state()
    await adapter.disconnect()
```

再写回退测试：第一条请求成功以建立 `_last_sent_tcp`；第二条请求的完整候选返回 `None`、0.75 候选返回合法关节解；断言 IK 调用目标依次为完整目标和 0.75 位姿，只有一次 `move_pvat`，事件中的 `recovery_fraction == 0.75`，Pump 保持运行。

Run:

```powershell
cd backend
python -m pytest tests/robots/test_lebai_adapter_control.py -k "ik_miss or fake_repeated or fallback" -q
```

Expected: Fake 第五次失败仍产生 `ik_failure_persistent`；回退测试没有第二候选调用。

- [ ] **Step 4: 重构适配器的单次候选发送**

在 `RealLebaiAdapter` 中增加：

```python
RECOVERABLE_IK_ERRORS = {
    "ik_unreachable",
    "ik_invalid",
    "ik_joint_limit",
    "ik_joint_jump",
    "joint_speed_limit",
}

async def _solve_candidate(
    self,
    client: LebaiClientProtocol,
    snapshot: LebaiSnapshot,
    target: Pose,
) -> PvatPoint:
    ...
```

`_send_target` 在一个 SDK lock 内按 `recovery_candidates(...)` 顺序调用 `_solve_candidate`。只捕获 `RECOVERABLE_IK_ERRORS` 并继续候选；SDK timeout/调用失败立即上抛。首个成功候选执行一次 `move_pvat`，将 `_last_sent_tcp` 设为实际候选并在 `pvat_sent` 记录：

```python
{
    "requested_tcp": request.target.model_dump(),
    "target_tcp": candidate.model_dump(),
    "recovery_fraction": fraction,
}
```

如果所有候选失败，调用 `_note_soft_constraint(reason, persistent=self._backend_label != "LEBAI_FAKE")`。Fake 只记录约束并返回；真实标签在第五次维持 `ik_failure_persistent`。成功发送完整 fraction 1.0 时清除约束；成功发送回退候选时保留最近的软约束，等待后续完整成功和 RobotControl 去抖清除。

- [ ] **Step 5: 清除跨会话安全目标**

新增 `_reset_pvat_history()`，同时执行：

```python
self._previous_sent_qd = None
self._last_sent_tcp = None
```

用它替换 `disconnect`、`stop`、Home 开始、fault stop、`_latch_unverified_stop` 以及所有 `_pump.invalidate()` 后的单独 `_previous_sent_qd = None`。新增测试证明 `stop()` 后下一次失败只尝试原目标一次，而不是使用停止前缓存。

- [ ] **Step 6: 让 Fake IK 拒绝自碰撞候选**

在 `backend/tests/digital_twin/test_lebai_client.py` mock `cartesian_servo_step` 返回 `self_collision_limited=True`，断言 `kinematics_inverse` 返回 `None` 且关节状态不变。实现时在 `DigitalTwinLebaiClient.kinematics_inverse` 返回 q 之前检查该标志。真实 SDK 路径不受影响。

- [ ] **Step 7: 证明未知错误仍 fail closed**

增加 Fake 标签测试，将 `client.kinematics_inverse` 设为抛出 `RuntimeError`，等待 Pump 后断言 `str(adapter.pump_fault) == "sdk_call_failed:ik"`，并断言 `get_state()` 抛出同一错误。该错误不能进入候选回退。

- [ ] **Step 8: 跑 Task 2 测试**

Run:

```powershell
cd backend
python -m pytest tests/robots/test_lebai_recovery.py tests/robots/test_lebai_adapter_control.py tests/digital_twin/test_lebai_client.py tests/robots/test_lebai_pump.py -q
```

Expected: 全部 PASS。

- [ ] **Step 9: 本地提交 Task 2**

```powershell
git add -- backend/app/robots/lebai_recovery.py backend/app/robots/lebai_adapter.py backend/app/digital_twin/lebai_client.py backend/tests/robots/test_lebai_recovery.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/digital_twin/test_lebai_client.py
git diff --cached --check
git commit -m "fix: keep simulation IK constraints recoverable"
```

---

### Task 3: 仿真响应参数与恢复提示

**Files:**
- Modify: `config/fake-lebai.yaml`
- Modify: `backend/tests/digital_twin/test_runtime.py`
- Modify: `backend/tests/control/test_safety.py`
- Modify: `web/src/ui/hud.ts`
- Modify: `web/src/scenes/vrSafetyPanel.ts`
- Modify: `web/tests/hud.test.ts`
- Modify: `web/tests/vrSafetyPanel.test.ts`

**Interfaces:**
- Consumes: Task 1 的 `_build_limiter(settings, runtime_backend)`。
- Preserves: `RobotStateMessage.constraint` 枚举和所有 WebSocket schema。
- Produces: Fake profile 的确切响应参数和统一中文恢复文案。

- [ ] **Step 1: 写 Fake profile 精确值 RED**

在 `backend/tests/digital_twin/test_runtime.py`：

```python
def test_fake_profile_uses_responsive_but_bounded_motion_values() -> None:
    settings = Settings.load(DIGITAL_TWIN_CONFIG)
    assert settings.lebai is not None
    control = settings.lebai.control
    assert (
        control.max_tcp_speed_mps,
        control.max_tcp_rotation_radps,
        control.max_tcp_acceleration_mps2,
        control.max_tcp_angular_acceleration_radps2,
        control.max_joint_speed_radps,
        control.max_joint_acceleration_radps2,
        control.max_joint_step_rad,
        control.max_tcp_step_m,
        control.max_tcp_rotation_step_deg,
        control.max_relative_translation_m,
        control.max_relative_rotation_deg,
        control.translation_scale,
    ) == pytest.approx((
        0.06, 0.50, 0.20, 1.00, 0.30, 1.00,
        0.015, 0.003, 1.5, 0.16, 45.0, 0.70,
    ))
```

Run:

```powershell
cd backend
python -m pytest tests/digital_twin/test_runtime.py::test_fake_profile_uses_responsive_but_bounded_motion_values -q
```

Expected: 当前保守值与期望不匹配，测试失败。

- [ ] **Step 2: 更新 Fake-only 配置**

只修改 `config/fake-lebai.yaml` 的 12 个值为规格中的确切数值。不得修改 `config/real-robot.example.yaml`、`backend/tests/robots/real_settings.py` 或真实部署文档的真机限速。

- [ ] **Step 3: 增加 2 秒动态响应特征测试**

在 `backend/tests/control/test_safety.py` 用 Fake 设置构造 limiter，分别以 0.02 秒执行 100 个 tick：

```python
for _ in range(100):
    translated = limiter.limit_motion(translated, translate_target, 0.02)
assert np.linalg.norm(np.asarray(translated.p) - np.asarray(translate_target.p)) <= 0.003

for _ in range(100):
    rotated = rotation_limiter.limit_motion(rotated, rotate_target, 0.02)
rotation_error = Rotation.from_quat(rotate_target.q) * Rotation.from_quat(rotated.q).inv()
assert rotation_error.magnitude() <= np.deg2rad(2)
```

目标分别为 50 mm 和 20°。测试只验证动态 limiter 的确定性能力，不把 FakeClock 循环次数伪装成真实设备耗时。

- [ ] **Step 4: 写 HUD 文案 RED**

在桌面 HUD 和 VR Safety Panel 测试中精确断言：

- `workspace_boundary`：`已到达操作边界，保持 Grip 向反方向退回`
- `ik_boundary`：`当前姿态暂不可达，保持 Grip 退回上一个位置`
- `joint_boundary`：`已到达关节操作边界，保持 Grip 反向退回`

并断言这些状态 tone 为 amber，页面连接文案仍为已连接，不包含 `FAULT` 或 `正在重连`。

Run:

```powershell
cd web
npm.cmd test -- --run tests/hud.test.ts tests/vrSafetyPanel.test.ts
```

Expected: 旧短文案导致精确断言失败。

- [ ] **Step 5: 更新桌面与 VR 提示**

仅修改 `readableConstraint()` 和 `constraintInstruction()`。不改状态优先级：真实 fault、STALE、断线仍必须覆盖软约束显示。

- [ ] **Step 6: 跑 Task 3 测试和构建**

Run:

```powershell
cd backend
python -m pytest tests/control/test_safety.py tests/digital_twin/test_runtime.py -q
cd ..\web
npm.cmd test -- --run tests/hud.test.ts tests/vrSafetyPanel.test.ts
npm.cmd run build
```

Expected: 全部 PASS；仅允许现有 Vite 大包 advisory。

- [ ] **Step 7: 本地提交 Task 3**

```powershell
git add -- config/fake-lebai.yaml backend/tests/digital_twin/test_runtime.py backend/tests/control/test_safety.py web/src/ui/hud.ts web/src/scenes/vrSafetyPanel.ts web/tests/hud.test.ts web/tests/vrSafetyPanel.test.ts
git diff --cached --check
git commit -m "feat: tune responsive simulation controls"
```

---

### Task 4: 单连接耐久、全量回归与网页验收

**Files:**
- Create: `backend/tests/digital_twin/test_responsive_teleop.py`
- Modify only if a newly reproduced defect requires it: the exact production file proven by the failing test
- Create: `.superpowers/sdd/2026-08-12-simulation-teleoperation-responsiveness/task-4-report.md`（仓库忽略的本地证据报告）

**Interfaces:**
- Consumes: Task 1 的 `axis_clamp`、Task 2 的 Fake recoverable Pump、Task 3 的配置和 HUD。
- Produces: 一条普通 WebSocket、至少 15 秒/200 帧的跨层稳定性证据。

- [ ] **Step 1: 写真实 Fake 栈单连接耐久 RED**

创建 `backend/tests/digital_twin/test_responsive_teleop.py`。使用 `create_digital_twin_app()` 和 `TestClient`，只打开一次 `/ws/v1/teleop`：

1. 发送 hello。
2. 发送 Grip 松开的有效帧并等待 ack。
3. 发送 `arm_request` 并等待 `arm_ack`。
4. 发送 Grip=true 帧进入 ACTIVE。
5. 在至少 15 秒内发送至少 200 个普通 VRFrame，发送间隔保持小于 100 ms：正常移动、X 轴越界、`DigitalTwinFaults(ik_failure=True)`、恢复 IK、反向退回。
6. 每个阶段接收推进的 `ack_seq`，记录所有 constraint、mode 和连接关闭事件。

最终断言：

```python
assert app.state.backend.pump_fault is None
assert app.state.backend._pump.running is True
assert "workspace_boundary" in constraints
assert "ik_boundary" in constraints
assert final_state["mode"] in {"ACTIVE", "HOLD"}
assert final_state["fault"] is None
assert hello_ack_count == 1
assert reconnect_count == 0
assert sent_frames >= 200
assert elapsed_s >= 15.0
```

测试结束前松开 Grip 并等待安全停止，保证 TestClient teardown 不留下后台任务。

- [ ] **Step 2: 关联已经确认的历史 RED，不伪造第二次失败**

该集成测试验收的是 Task 1/2 已分别通过单元 RED 驱动完成的行为，不再为了形式重复破坏生产代码。报告引用基线 SHA `6d97888` 的真实故障证据：连续 IK 失败后 Pump fault 为 `ik_failure_persistent`，唯一 WebSocket 随后反复重连；同时记录 Task 1/2 的精确 RED 命令和失败摘要。不得把 GREEN 运行写成 RED，也不得临时放宽断言制造失败。

- [ ] **Step 3: 运行实现后的耐久 GREEN**

Run:

```powershell
cd backend
python -m pytest tests/digital_twin/test_responsive_teleop.py -q
```

Expected: 通过；耗时至少 15 秒；无重连、无 fault、单连接正常停止。

- [ ] **Step 4: 跑后端全量回归**

Run:

```powershell
cd backend
python -m pytest -q
```

Expected: 全部 PASS。记录测试数、耗时和任何非阻断 warning。

- [ ] **Step 5: 跑前端全量回归与构建**

Run:

```powershell
cd web
npm.cmd test
npm.cmd run build
```

Expected: 全部 PASS；构建成功；只允许既有 `>500 kB` advisory。

- [ ] **Step 6: 跑 Virtual/Fake/Offline 接受脚本**

Run:

```powershell
python scripts/accept_virtual_lm3.py
python scripts/accept_fake_lebai.py
python scripts/accept_offline_rehearsal.py
```

Expected: Virtual 和 Fake gate 通过且 `hardware_verified=false`；Offline 自动门通过但 Browser 字段仍按实际证据标记，不得把未完成的 Quest/Browser 验收写成通过。

- [ ] **Step 7: 启动实际程序并完成桌面渲染验收**

Run:

```powershell
python scripts/run_offline_rehearsal.py
```

使用打印的 `https://127.0.0.1:5173/`。优先使用现有 in-app Browser；如果证书或 Browser runtime 阻塞，记录原因后使用已安装 Edge/Playwright fallback。验收顺序固定：

1. 页面加载后保持一个 Teleop WebSocket，状态为已连接。
2. 解锁仿真，鼠标控制末端进行 X/Y/Z 平移和姿态旋转。
3. 故意让 X 到边界，确认 X 停在边界而 Y/Z/旋转继续。
4. 反向移动，确认无需 Home、刷新或重新零位即可恢复。
5. 连续操作 60 秒，连接不得在“已连接/正在重连”间闪烁。
6. 检查控制台、HUD、后台日志和诊断面板；不存在 `ik_failure_persistent`、WebSocket 异常关闭或新增未处理异常。
7. 点击停止并确认程序保持安全可再次解锁。

若出现新的故障，只允许一次“采集首个不同故障边界”的诊断；先写失败测试和根因证据，再决定是否修复，禁止反复调参掩盖问题。

- [ ] **Step 8: 记录 Quest 3 人工边界**

如果 Quest 3 当时不可用，报告明确写 `Quest 3 rendered validation: pending`；不得用鼠标验收替代。若可用，则额外验证手柄前伸/下压/左右、手柄翻滚与夹爪姿态、Grip 松开停止。

- [ ] **Step 9: 写证据报告并检查仓库卫生**

报告必须包含：四个提交 SHA、RED/GREEN 命令、测试总数、15 秒单连接统计、浏览器截图路径、连接/约束/故障计数、真机未接触声明、Quest 状态、剩余限制。

Run:

```powershell
git diff --check
git status --short
```

Expected: 只有有意修改；无监听器和遗留启动器进程；报告位于仓库忽略目录。

- [ ] **Step 10: 本地提交 Task 4**

```powershell
git add -- backend/tests/digital_twin/test_responsive_teleop.py
git diff --cached --check
git commit -m "test: verify stable responsive simulation teleoperation"
```

如果 Step 7 发现并以 TDD 修复了独立生产缺陷，先用独立提交保存该修复，再提交最终耐久测试；不得把失败测试改成虚假通过。

## Final Verification Checklist

- [ ] `workspace_boundary` 在 Fake 中为 `constrained=true, hold=false`，在真实 `LEBAI` 中仍为 `hold=true`。
- [ ] 单轴越界不冻结其他平移轴或姿态。
- [ ] 反向恢复不改变 CoordinateMapper anchor。
- [ ] Fake 连续十次可恢复 IK 失败不停止 Pump、不关闭 WebSocket。
- [ ] 真实 `LEBAI` 连续失败升级测试仍通过。
- [ ] Fake SDK timeout/unknown exception 仍产生 fault。
- [ ] 50 mm/20° 的 2 秒确定性 limiter 能力测试通过。
- [ ] HUD 把软约束显示为 amber 恢复提示，不显示 FAULT/重连。
- [ ] 15 秒/至少 200 帧单连接测试通过。
- [ ] 后端全量、前端全量、Vite build、Virtual/Fake/Offline gates 均有新鲜证据。
- [ ] 真机未连接、未调用真实 SDK、未 push。
