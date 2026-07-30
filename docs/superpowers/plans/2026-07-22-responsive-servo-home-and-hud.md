# Responsive Servo, Manual Home, and VR HUD Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the simulated LM3 follow Quest controller motion at the intended speed and fixed 1:1 pose scale, start and recover with horizontal jaws and the stereo camera above them, provide a safe manual Home flow on B, and move the fully labeled VR HUD out of the work view.

**Architecture:** Keep the existing FastAPI/WebSocket/Three.js boundary, but remove the second joint-position servo from the teleoperation path: differential IK produces a velocity command and `VirtualRobot` integrates it once at 50 Hz. Add a backward-compatible protocol-v1 manual Home request that reuses the simulator Home executor after an explicit stop, while frontend state keeps B as immediate stop before it can mean Home.

**Tech Stack:** Python 3.11+, NumPy, SciPy Rotation, FastAPI, Pydantic, pytest, TypeScript 5.9, Three.js, Vitest, Vite.

## Global Constraints

- `backend: simulator` remains mandatory; do not import or contact the Lebai SDK.
- Translation defaults to one fixed `1.0` scale and remains startup-configurable; rotation is fixed at `1.0` outside a `0.5°` dead zone.
- Do not add nonlinear motion scaling, runtime scale switching, physics, collision, camera capture, or dataset writing.
- The control and simulator integration frequencies are `50 Hz`; robot-state feedback becomes `50 Hz` for the single controlling client.
- Simulator limits are `0.30 m/s`, `1.2 m/s²`, `1.5 rad/s`, `4.0 rad/s²`, `1.5 rad/s` joint speed, and `4.0 rad/s²` joint acceleration.
- Position remains higher priority than orientation. Ordinary orientation or workspace limitation stays a soft constraint, not a hard reset fault.
- B always stops first from an unlocked/moving state. Only a later B press with Grip released and the logical state stopped may request Home.
- Home ends in `DISARMED`; it never unlocks or resumes motion automatically.
- Preserve raw VR pose, camera/time-sync extension points, the single-controller connection model, and unrelated working-tree files.
- Run only the focused tests listed by each task plus one final frontend build; do not run soak or the entire historical suite.

## File Structure

- `backend/app/sim/cartesian_servo.py`: produce bounded joint velocity as well as the predicted step.
- `backend/app/sim/virtual_robot.py`: own mutually exclusive teleoperation velocity mode and Home position mode.
- `backend/app/robots/sim_adapter.py`: send differential-IK velocity to the simulator.
- `backend/app/control/coordinate_mapper.py`: Grip-anchored fixed-scale world-pose mapping.
- `config/default.yaml`: simulator response, scale, feedback-rate, and Home-speed settings.
- `config/lm3_visual_kinematics_v1.json`: standard Home and simulator joint limits.
- `backend/app/control/robot_control.py`: validate and execute fault-free manual Home.
- `backend/app/schemas/messages.py`, `backend/app/api/teleop_ws.py`: protocol-v1 manual Home request/result.
- `web/src/protocol/messages.ts`, `web/src/transport/teleopSocket.ts`: typed Home result transport.
- `web/src/ui/armPanel.ts`: B stop/Home state logic and request correlation.
- `web/src/scenes/vrSafetyPanel.ts`: high HUD placement and complete button guidance.
- `README.md`, `docs/quest-development.md`: operator instructions.

---

### Task 1: Remove the second teleoperation servo

**Files:**
- Modify: `backend/app/sim/cartesian_servo.py`
- Modify: `backend/app/sim/virtual_robot.py`
- Modify: `backend/app/robots/sim_adapter.py`
- Modify: `backend/tests/sim/test_cartesian_servo.py`
- Modify: `backend/tests/sim/test_virtual_robot.py`

**Interfaces:**
- Produces: `CartesianServoResult.joint_velocity: tuple[float, float, float, float, float, float]`.
- Produces: `VirtualRobot.set_target_qd(target: Sequence[float]) -> None`.
- Preserves: `VirtualRobot.set_target_q(...)` for Home and `SimRobotAdapter.command_tcp(...)` for the backend protocol.

- [ ] **Step 1: Replace the adapter single-position-step assertion with a velocity-mode assertion**

In `backend/tests/sim/test_cartesian_servo.py`, keep the convergence test and replace the adapter test with:

```python
@pytest.mark.asyncio
async def test_sim_adapter_commands_velocity_without_a_second_position_servo() -> None:
    adapter = SimRobotAdapter()
    start = forward_pose(adapter.robot.q, adapter.model)
    target = start.model_copy(
        update={"p": (start.p[0] + 0.03, start.p[1], start.p[2] - 0.02)}
    )

    await adapter.command_tcp(target, command_id=7)

    assert adapter.robot.target_q is None
    assert adapter.robot.target_qd is not None
    assert np.max(np.abs(adapter.robot.target_qd)) <= (
        adapter.model.max_joint_speed_radps + 1e-12
    )
    assert adapter.command_id == 7
```

Add this focused behavior to `backend/tests/sim/test_virtual_robot.py`:

```python
def test_velocity_mode_accelerates_to_and_integrates_the_requested_speed_once() -> None:
    robot = VirtualRobot(LM3Model())
    requested = np.full(6, 0.6)
    robot.set_target_qd(requested)

    for _ in range(25):
        robot.step(0.02)

    assert robot.target_q is None
    assert robot.target_qd == pytest.approx(requested)
    assert robot.qd == pytest.approx(requested)
    assert np.min(robot.q - np.asarray(robot.model.home_q)) > 0.15
```

- [ ] **Step 2: Run RED**

Run from `backend`:

```powershell
python -m pytest tests/sim/test_cartesian_servo.py tests/sim/test_virtual_robot.py -q
```

Expected: FAIL because `target_qd`, `set_target_qd`, and `joint_velocity` do not exist and the adapter still sets a one-step position target.

- [ ] **Step 3: Expose the bounded joint velocity from differential IK**

Add the field to `CartesianServoResult` and return the velocity after joint-window projection:

```python
@dataclass(frozen=True)
class CartesianServoResult:
    q: tuple[float, float, float, float, float, float]
    joint_velocity: tuple[float, float, float, float, float, float]
    position_error_m: float
    orientation_error_rad: float
    joint_limited: bool

# after next_q is clipped
bounded_velocity = (next_q - q) / dt
return CartesianServoResult(
    q=tuple(float(value) for value in next_q),
    joint_velocity=tuple(float(value) for value in bounded_velocity),
    position_error_m=float(np.linalg.norm(solved_position_error)),
    orientation_error_rad=float(np.linalg.norm(solved_orientation_error)),
    joint_limited=joint_limited,
)
```

- [ ] **Step 4: Add a mutually exclusive velocity mode to `VirtualRobot`**

Initialize `target_qd`, clear the opposite mode in both setters, and let `step()` use the velocity command directly before its one acceleration-limited integration:

```python
self.target_qd: np.ndarray | None = None

def set_target_qd(self, target: Sequence[float]) -> None:
    target_qd = np.asarray(target, dtype=float)
    if (
        target_qd.shape != (6,)
        or not np.all(np.isfinite(target_qd))
        or np.max(np.abs(target_qd)) > self.model.max_joint_speed_radps + 1e-12
    ):
        raise ValueError("joint_safety_window")
    self.target_q = None
    self.target_speed_radps = None
    self.target_qd = target_qd.copy()

# at the start of set_target_q
self.target_qd = None

# in stop
self.target_qd = None

# in step, before the existing position branch
if self.target_qd is not None:
    desired_qd = self.target_qd.copy()
elif self.target_q is None:
    desired_qd = np.zeros(6, dtype=float)
else:
    # preserve the existing Home position-mode calculation
```

After `self.q += self.qd * dt`, clip to `home_q ± joint_window_rad` and zero only velocity components that would continue outward at a reached boundary.

- [ ] **Step 5: Switch the adapter to velocity mode**

In `SimRobotAdapter.command_tcp()`:

```python
result = cartesian_servo_step(
    target,
    self.robot.q,
    self.model,
    dt=self.STEP_SECONDS,
)
self.robot.set_target_qd(result.joint_velocity)
self.command_id = command_id
```

- [ ] **Step 6: Run GREEN and commit**

Run the Step 2 command. Expected: all focused servo and virtual-robot tests pass.

Commit only the five Task 1 files:

```powershell
git commit -m "fix: integrate teleoperation velocity once"
```

### Task 2: Fixed 1:1 pose mapping and responsive simulator settings

**Files:**
- Modify: `backend/app/control/coordinate_mapper.py`
- Modify: `backend/app/sim/cartesian_servo.py`
- Modify: `backend/app/api/teleop_ws.py`
- Modify: `config/default.yaml`
- Modify: `backend/tests/control/test_coordinate_mapper.py`
- Modify: `backend/tests/api/test_teleop_ws.py`

**Interfaces:**
- Preserves: `CoordinateMapper.capture(...)`, `.target(...)`, and startup-configurable `translation_scale`.
- Produces: world-relative rigid orientation mapping and a `0.02 s` state-feedback period.

- [ ] **Step 1: Write the rigid-pose and 50 Hz feedback expectations**

Replace the mapper behavior with a single world-delta test:

```python
def test_mapper_applies_fixed_one_to_one_world_pose_delta_without_a_jump() -> None:
    mapper = CoordinateMapper(
        translation_scale=1.0,
        rotation_scale=1.0,
        rotation_dead_zone_deg=0.0,
    )
    hand_rotation = Rotation.from_euler("xyz", (15, -20, 10), degrees=True)
    tcp_rotation = Rotation.from_euler("xyz", (-30, 5, 40), degrees=True)
    hand_anchor = Pose(p=(0.0, 1.2, -0.3), q=tuple(hand_rotation.as_quat()))
    tcp_anchor = Pose(p=(0.3, 0.4, -0.2), q=tuple(tcp_rotation.as_quat()))
    mapper.capture(hand_anchor, tcp_anchor)
    delta = Rotation.from_euler("xyz", (12, -8, 20), degrees=True)

    target = mapper.target(Pose(
        p=(0.10, 1.15, -0.22),
        q=tuple((delta * hand_rotation).as_quat()),
    ))

    assert target.p == pytest.approx((0.40, 0.35, -0.12))
    np.testing.assert_allclose(
        Rotation.from_quat(target.q).as_matrix(),
        (delta * tcp_rotation).as_matrix(),
        atol=1e-8,
    )
```

Rename the two API tests to 50 Hz and change exact sleeps to:

```python
assert sleeps == [0.02, 0.02, 0.02]
```

The integration timing assertion becomes `0.012 <= delta <= 0.08` to tolerate Windows scheduling while still rejecting the old fixed 50 ms period.

- [ ] **Step 2: Run RED**

```powershell
python -m pytest tests/control/test_coordinate_mapper.py tests/api/test_teleop_ws.py::test_state_sender_sleeps_exactly_twenty_ms -q
```

Expected: mapper orientation mismatch and old `[0.05, 0.05, 0.05]` sleeps.

- [ ] **Step 3: Implement rigid Grip orientation**

Store rotations at capture and derive the world delta in `target()`:

```python
# in __init__
if not np.isclose(rotation_scale, 1.0):
    raise ValueError("rotation_scale_must_equal_one")

# in target
r_now = Rotation.from_quat(hand.q)
r_anchor = Rotation.from_quat(self._hand_anchor.q)
delta_world = r_now * r_anchor.inv()
rotation_vector = delta_world.as_rotvec()
angle = float(np.linalg.norm(rotation_vector))
if angle <= self.rotation_dead_zone_rad:
    delta_world = Rotation.identity()
target_rotation = delta_world * Rotation.from_quat(self._tcp_anchor.q)
```

Keep translation as `tcp_anchor + translation_scale * (hand_now - hand_anchor)`. Remove the obsolete controller-local calibration matrix from the active mapping path.

- [ ] **Step 4: Apply the approved simulator profile**

Set these exact values in `config/default.yaml`:

```yaml
control_hz: 50
state_hz: 50
max_linear_speed_mps: 0.30
max_angular_speed_radps: 1.5
max_linear_accel_mps2: 1.2
max_angular_accel_radps2: 4.0
translation_scale: 1.0
rotation_scale: 1.0
rotation_dead_zone_deg: 0.5
joint_speed_radps: 1.5
joint_accel_radps2: 4.0
```

Change the two internal Cartesian command caps in `cartesian_servo.py` to `0.30` and `1.5`. In `state_sender()`, calculate the period from the configured app state with a test-friendly fallback:

```python
settings = getattr(getattr(websocket, "app", None), "state", None)
state_hz = getattr(getattr(settings, "settings", None), "state_hz", 50)
period_s = 1.0 / state_hz
# inside the loop
await asyncio.sleep(period_s)
```

- [ ] **Step 5: Run GREEN and commit**

Run both focused mapper and API sender test files. Expected: pass.

```powershell
git commit -m "feat: use responsive one-to-one pose mapping"
```

### Task 3: Standard horizontal-jaw Home

**Files:**
- Modify: `config/lm3_visual_kinematics_v1.json`
- Modify: `backend/tests/sim/test_kinematics.py`
- Modify: `config/default.yaml`

**Interfaces:**
- Produces: `LM3Model.home_q == (0, -π/4, π/2, -π/4, π/2, -π/2)`.
- Preserves: the existing Home TCP position and optical-axis direction.

- [ ] **Step 1: Add the visual-axis Home test**

Append to `backend/tests/sim/test_kinematics.py`:

```python
def test_home_keeps_tcp_and_forward_while_camera_is_up_and_jaws_are_horizontal() -> None:
    model = LM3Model()
    legacy_q = np.asarray((0.0, -np.pi / 4, np.pi / 2, -np.pi / 4, np.pi / 2, 0.0))
    home = forward_matrix(model.home_q, model)
    legacy = forward_matrix(legacy_q, model)
    camera_up = home[:3, :3] @ np.asarray((-1.0, 0.0, 0.0))
    jaw_axis = home[:3, :3] @ np.asarray((0.0, 0.0, 1.0))
    forward = home[:3, :3] @ np.asarray((0.0, -1.0, 0.0))
    legacy_forward = legacy[:3, :3] @ np.asarray((0.0, -1.0, 0.0))

    assert model.home_q[5] == pytest.approx(-np.pi / 2)
    np.testing.assert_allclose(home[:3, 3], legacy[:3, 3], atol=1e-9)
    np.testing.assert_allclose(forward, legacy_forward, atol=1e-9)
    np.testing.assert_allclose(camera_up, (0.0, 1.0, 0.0), atol=1e-8)
    assert jaw_axis[1] == pytest.approx(0.0, abs=1e-8)
```

- [ ] **Step 2: Run RED**

```powershell
python -m pytest tests/sim/test_kinematics.py -q
```

Expected: the sixth Home joint is `0`, camera-up is not world `+Y`, and jaw axis is vertical.

- [ ] **Step 3: Change the authoritative Home and joint dynamics**

In `config/lm3_visual_kinematics_v1.json` set:

```json
"home_q": [0.0, -0.7853981634, 1.5707963268, -0.7853981634, 1.5707963268, -1.5707963268],
"max_joint_speed_radps": 1.5,
"max_joint_accel_radps2": 4.0
```

Set `home_joint_speed_radps: 0.5` in `config/default.yaml`; the Home trajectory remains visibly controlled but no longer takes the old slow-motion duration.

- [ ] **Step 4: Run GREEN and commit**

Run the Step 2 command. Expected: both shared-FK/Jacobian and standard-Home tests pass.

```powershell
git commit -m "feat: set camera-up horizontal-jaw home"
```

### Task 4: Backend manual Home protocol and executor

**Files:**
- Modify: `backend/app/schemas/messages.py`
- Modify: `backend/app/robots/base.py`
- Modify: `backend/app/control/robot_control.py`
- Modify: `backend/app/api/teleop_ws.py`
- Modify: `backend/tests/contract/test_messages.py`
- Modify: `backend/tests/control/test_robot_control.py`
- Modify: `backend/tests/api/test_teleop_ws.py`

**Interfaces:**
- Produces: control type `home_request`.
- Produces: `HomeResult(accepted, reason=None, message=None)` and `RobotControl.home() -> HomeResult`.
- Produces: server message `home_result`, accepted with `mode: DISARMED` or rejected with a fixed reason/message.

- [ ] **Step 1: Add focused contract, control, and websocket tests**

Add a contract assertion:

```python
def test_home_request_is_a_valid_v1_control_message() -> None:
    message = ClientControlMessage.model_validate(
        {"v": 1, "type": "home_request", "request_id": "home-1"}
    )
    assert message.type == "home_request"
```

Extend the existing `FakeBackend` with:

```python
self.home_phases: list[str] = []

async def home(self, options, on_phase) -> None:
    on_phase("homing")
    self.home_phases.append("homing")
    on_phase("stabilizing")
    self.home_phases.append("stabilizing")
```

Add one control behavior:

```python
@pytest.mark.asyncio
async def test_manual_home_requires_released_grip_and_ends_disarmed() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()
    result = await control.home()

    assert result == robot_control_module.HomeResult(True)
    assert backend.stops[-1] is StopReason.HOME
    assert backend.home_phases == ["homing", "stabilizing"]
    assert control.mode is TeleopMode.DISARMED
    assert control.mapper._hand_anchor is None
    assert control.last_target is None
```

Add one websocket test that sends `home_request`, mocks `control.home()` as accepted, and expects `home_result` with the same `request_id` and `mode: DISARMED`.

- [ ] **Step 2: Run RED**

```powershell
python -m pytest tests/contract/test_messages.py::test_home_request_is_a_valid_v1_control_message tests/control/test_robot_control.py::test_manual_home_requires_released_grip_and_ends_disarmed tests/api/test_teleop_ws.py::test_home_request_returns_correlated_result -q
```

Expected: missing control type, `HomeResult`, `RobotControl.home`, `StopReason.HOME`, and websocket branch.

- [ ] **Step 3: Add exact backend types and validation**

Extend `ClientControlMessage.type` with `home_request`, add `StopReason.HOME = "home"`, and define in `robot_control.py`:

```python
HomeRejectReason = Literal[
    "fault_present",
    "grip_pressed",
    "not_stopped",
    "control_loop_unavailable",
    "home_failed",
]

@dataclass(frozen=True)
class HomeResult:
    accepted: bool
    reason: HomeRejectReason | None = None
    message: str | None = None
```

`RobotControl.home()` accepts only when `_fault is None`, the loop is available, mode is `READY` or `DISARMED`, the newest frame exists, and `latest.frame.right.grip` is false. On rejection return one of the fixed reasons above without calling the backend.

- [ ] **Step 4: Execute stop, settle, Home, and state clearing**

Implement this sequence inside `RobotControl.home()`:

```python
self._recovery_phase = "stopping"
await self.backend.stop(StopReason.HOME)
deadline = asyncio.get_running_loop().time() + 1.0
while True:
    state = await self.backend.get_state()
    if state.robot_state is BackendState.IDLE:
        break
    if state.robot_state is not BackendState.MOVING:
        return HomeResult(False, "home_failed", "仿真后端无法进入停止状态。")
    if asyncio.get_running_loop().time() >= deadline:
        return HomeResult(False, "home_failed", "等待仿真停止超时。")
    await asyncio.sleep(0.02)

def on_home_phase(phase: HomePhase) -> None:
    self._recovery_phase = phase

await self.backend.home(self.home_options, on_home_phase)
```

In `finally`, clear `_recovery_phase`. On success clear mapper, filter, limiter, `last_target`, constraint, and queued-frame cutoff exactly as fault recovery does, then call `machine.disarm()` and return `HomeResult(True)`. Catch backend exceptions and return `home_failed` while staying stopped and locked.

- [ ] **Step 5: Add the websocket result branch**

For `home_request`, return one of:

```python
{"v": 1, "type": "home_result", "request_id": message.request_id,
 "accepted": True, "mode": "DISARMED"}
```

or:

```python
{"v": 1, "type": "home_result", "request_id": message.request_id,
 "accepted": False, "reason": result.reason, "message": result.message}
```

- [ ] **Step 6: Run GREEN and commit**

Run the three focused tests from Step 2. Expected: `3 passed`.

```powershell
git commit -m "feat: add safe manual home request"
```

### Task 5: Frontend B-state logic and elevated complete HUD

**Files:**
- Modify: `web/src/protocol/messages.ts`
- Modify: `web/src/transport/teleopSocket.ts`
- Modify: `web/src/main.ts`
- Modify: `web/src/ui/armPanel.ts`
- Modify: `web/src/scenes/vrSafetyPanel.ts`
- Modify: `web/tests/messages.test.ts`
- Modify: `web/tests/armPanel.test.ts`
- Modify: `web/tests/teleopSocket.test.ts`
- Modify: `web/tests/vrSafetyPanel.test.ts`

**Interfaces:**
- Produces: `HomeResultMessage`, `isHomeResultMessage(...)`, and `ArmPanel.handleHomeResult(...)`.
- Extends: `ArmSafetySnapshot.homePending?: boolean`.
- Preserves: `XRSessionController.onStopOrResetRequest`; B release edges still call one ArmPanel method.

- [ ] **Step 1: Add one test per user-visible behavior**

Extend control types and add an exact Home-result guard test:

```ts
expect(isClientControlMessage({v: 1, type: 'home_request', request_id: 'home-1'})).toBe(true);
expect(isHomeResultMessage({
  v: 1, type: 'home_result', request_id: 'home-1', accepted: true, mode: 'DISARMED',
})).toBe(true);
```

Add an ArmPanel B sequence:

```ts
it('uses the first B to stop and a later released-Grip B to request Home', () => {
  const send = vi.fn();
  const panel = new ArmPanel(document.querySelector('#panel')!, send);
  panel.setConnectionStatus({state: 'connected'});
  panel.observeGrip(false);
  panel.setMode('READY');
  panel.requestArm('xr');
  panel.setMode('ARMED');

  panel.requestStopOrReset('xr');
  expect(send).toHaveBeenLastCalledWith(expect.objectContaining({type: 'disarm'}));
  panel.setMode('DISARMED');
  panel.observeGrip(false);
  panel.requestStopOrReset('xr');
  expect(send).toHaveBeenLastCalledWith(expect.objectContaining({type: 'home_request'}));
  expect(panel.safetyState.homePending).toBe(true);
});
```

In the sprite resource test assert:

```ts
expect(panel.sprite.position.toArray()).toEqual([-0.82, 1.52, -0.72]);
expect(describeVrSafety(state('stopped'), true).footer).toBe(
  'A 解锁 · B 停止/回 Home · Grip 移动 · Trigger 夹爪',
);
expect(describeVrSafety(state('stopped'), true).instruction).toBe(
  '保持 Grip 松开：A 解锁，B 回 Home',
);
```

- [ ] **Step 2: Run RED**

```powershell
npm.cmd test -- messages.test.ts armPanel.test.ts teleopSocket.test.ts vrSafetyPanel.test.ts
```

Expected: Home types/guards/callbacks are missing, stopped B still sends only `disarm`, and HUD coordinates/copy are old.

- [ ] **Step 3: Add the TypeScript protocol and transport callback**

Add `home_request` to `ClientControlType`, define exact accepted/rejected `HomeResultMessage`, and implement `isHomeResultMessage` using the same exact-key and 64-code-point request-ID checks as fault reset.

Add a final optional constructor callback to `TeleopSocket`:

```ts
private readonly onHomeResult: (message: HomeResultMessage) => void = () => {},
```

Dispatch `isHomeResultMessage(message)` after fault-reset results. In `main.ts`, pass:

```ts
(message) => armPanel?.handleHomeResult(message),
```

- [ ] **Step 4: Implement the two-stage B behavior**

Add `pendingHomeRequestId`, optional `homePending` in the safety snapshot, and include `home_request` in `control(...)`. In `requestStopOrReset(...)` keep the fault branch first, then:

```ts
if (this.pendingHomeRequestId !== null) return;
const stopped = this.mode === 'DISARMED' || (!this.armed && this.mode === 'READY');
if (stopped && this.connected && !this.gripPressed) {
  const message = this.control('home_request', source);
  this.pendingHomeRequestId = message.request_id;
  this.eligible = false;
  this.syncButtonState();
  this.sendControl(message);
  return;
}
this.requestDisarm(source);
```

`handleHomeResult` accepts only the matching request. Success calls `resetToLocked()`; rejection clears pending, keeps the panel stopped, and exposes fixed local feedback. Disable A, B Home, and duplicate requests while Home is pending.

- [ ] **Step 5: Elevate the HUD and write complete context copy**

Set:

```ts
const CONTROL_FOOTER = 'A 解锁 · B 停止/回 Home · Grip 移动 · Trigger 夹爪' as const;
this.sprite.position.set(-0.82, 1.52, -0.72);
```

When `homePending` or `recoveryPhase` is set, display `正在返回 Home，请保持 Grip 松开`. Use these context instructions:

```text
active: 松开 Grip 停止 · B 紧急停止
stopped: 保持 Grip 松开：A 解锁，B 回 Home
fault reset: 松开 Grip，按 B 复位并回 Home
```

Desktop stop-button text is `停止` while armed/active and `回到 Home` while stopped with Grip released.

- [ ] **Step 6: Run GREEN and commit**

Run the Step 2 command. Expected: the four focused frontend files pass.

```powershell
git commit -m "feat: add B-button Home and elevate VR HUD"
```

### Task 6: Documentation, focused verification, and restart

**Files:**
- Modify: `README.md`
- Modify: `docs/quest-development.md`

**Interfaces:**
- Documents: fixed scale, Grip re-anchoring, responsive simulator limits, standard Home, B-state semantics, and HUD guidance.

- [ ] **Step 1: Update operator documentation**

Document exactly:

```text
平移默认 1:1，姿态固定 1:1；松开并重新按 Grip 可在舒适范围内继续覆盖工作空间。
B 在运动/已解锁时立即停止；停止且 Grip 松开后再次按 B 返回 Home。
Home 为左右夹爪、双目摄像头在上；Home 完成后必须重新按 A 解锁。
HUD 位于视野左上方并固定显示 A、B、Grip、Trigger 的用途。
```

Keep the real-robot-disabled warning adjacent to these instructions.

- [ ] **Step 2: Run only the agreed backend verification**

From `backend`:

```powershell
python -m pytest tests/sim/test_cartesian_servo.py tests/sim/test_virtual_robot.py tests/control/test_coordinate_mapper.py tests/sim/test_kinematics.py tests/contract/test_messages.py::test_home_request_is_a_valid_v1_control_message tests/control/test_robot_control.py::test_manual_home_requires_released_grip_and_ends_disarmed tests/api/test_teleop_ws.py::test_home_request_returns_correlated_result tests/api/test_teleop_ws.py::test_state_sender_sleeps_exactly_twenty_ms -q
```

Expected: all selected tests pass.

- [ ] **Step 3: Run only the agreed frontend verification and one build**

From `web`:

```powershell
npm.cmd test -- messages.test.ts armPanel.test.ts teleopSocket.test.ts vrSafetyPanel.test.ts
npm.cmd run build
```

Expected: four selected test files pass and Vite build exits `0`; the existing large-chunk warning is non-blocking.

- [ ] **Step 4: Inspect scope and commit documentation**

Confirm the final diff has no Lebai SDK enablement, real robot IP, physics dependency, nonlinear scale, camera/dataset implementation, or unrelated old experimental file.

```powershell
git commit -m "docs: explain responsive teleoperation and manual Home"
```

- [ ] **Step 5: Restart and hand off for Quest acceptance**

Restart:

```powershell
# backend workdir
python -m uvicorn app.main:app --host 127.0.0.1 --port 8000

# web workdir
npm.cmd run dev
```

Verify `/health` returns `backend: SIMULATOR` and `real_robot_enabled: false`, Vite listens at `https://192.168.1.8:5173/`, and WebSocket reconnects. Ask the user to close the PC page, refresh Quest, and check:

1. 10 cm hand translation gives about 10 cm TCP translation without slow-motion accumulation;
2. controller roll/pitch/yaw maps 1:1 while Grip is held;
3. Home shows horizontal left/right jaws with camera above;
4. B once stops and the next released-Grip B returns Home;
5. A remains required after Home;
6. HUD is high enough and shows every button role.

## Self-Review

- Spec coverage: single integration, fixed scale, 50 Hz feedback, standard Home, B-state safety, HUD position/copy, simulator boundary, and focused verification each map to a task.
- Placeholder scan: all values, signatures, message names, commands, expected failures, and commit messages are explicit.
- Type consistency: backend `home_request`/`home_result`, frontend `HomeResultMessage`, ArmPanel correlation, and transport callback use the same names and accepted/rejected shapes.
- Scope: no nonlinear scaling, live scale UI, real robot, physics, recording, camera, or MR work is included.
