# Teleoperation UX and Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Quest/desktop teleoperation use a user-relative Grip zero, three-dimensional left-hand workspace placement, non-latching workspace/IK boundaries, and a visible simulator Home recovery.

**Architecture:** Keep raw XR poses in the v1 protocol and add optional head orientation plus independent soft-constraint/recovery fields. The backend remains authoritative for zero capture, pose mapping, motion limiting, IK acceptance, and recovery; the frontend owns only XR input extraction, visual workspace placement, presentation, and optional haptics.

**Tech Stack:** Python 3.11+, FastAPI, Pydantic, NumPy/SciPy, TypeScript 5.9, Three.js, WebXR, Vite.

## Global Constraints

- Do not enable or contact a real Lebai robot; `backend: simulator` remains mandatory.
- Do not add Cannon-es or any other physics dependency.
- Do not implement gravity, collision, friction, contact, or physical grasping.
- Preserve raw right-controller poses and camera/time-sync extension interfaces.
- Translation scale defaults to `0.5`; rotation scale defaults to `0.5`; rotation dead zone is `2°`.
- The soft workspace radius defaults to `0.45 m` and never becomes a latched fault.
- Simulator Home is `[0°, -45°, 90°, -45°, 90°, 0°]` at at most `0.25 rad/s`, with a `15 s` timeout.
- Per the user's explicit request, do not create, edit, or run automated tests; do not run the application, build, type-check, pytest, Vitest, or Vite. The user performs experiential acceptance.
- Preserve the user's existing `README.md` modification and untracked Chinese documents.

## File Structure

- `backend/app/config.py`: parse all runtime teleoperation and recovery settings.
- `config/default.yaml`: hold exact default parameters.
- `backend/app/schemas/messages.py`: optional `head_q`, `constraint`, and `recovery_phase` protocol fields.
- `backend/app/control/coordinate_mapper.py`: capture the Grip zero and map user-relative translation/rotation.
- `backend/app/control/safety.py`: workspace projection and dynamic motion limiting.
- `backend/app/control/robot_control.py`: soft-constraint lifecycle, mapper integration, and Home recovery orchestration.
- `backend/app/robots/base.py`: simulator recovery backend interface.
- `backend/app/robots/sim_adapter.py`: continuous IK seed and asynchronous Home recovery.
- `backend/app/robots/lebai_adapter.py`: explicitly disabled recovery implementation.
- `backend/app/sim/virtual_robot.py`: per-target joint-speed override for Home.
- `backend/app/main.py`: inject configured mapper/limiter/recovery values.
- `web/src/protocol/messages.ts`: TypeScript mirrors and strict optional-field validation.
- `web/src/xr/controllerInput.ts`: left X/Y/Grip input and raw controller poses.
- `web/src/xr/session.ts`: raw head quaternion transport and right-controller haptic feedback.
- `web/src/scenes/workspacePlacementController.ts`: bounded persistent XYZ visual placement.
- `web/src/scenes/simulationScene.ts`: integrate XYZ placement and enriched VR frames.
- `web/src/ui/armPanel.ts`: carry constraint and recovery presentation state without disarming.
- `web/src/ui/hud.ts`: display soft-boundary/recovery information.
- `web/src/scenes/vrSafetyPanel.ts`: VR copy for boundary, Home recovery, and new left-hand controls.
- `web/src/main.ts`: route state constraint/recovery fields to UI, scene, and haptics.
- `web/src/styles.css`: amber constraint presentation.
- `README.md`: update operator instructions without overwriting unrelated user edits.

---

### Task 1: Runtime configuration and compatible protocol fields

**Files:**
- Modify: `config/default.yaml`
- Modify: `backend/app/config.py`
- Modify: `backend/app/schemas/messages.py`
- Modify: `backend/app/main.py`
- Modify: `web/src/protocol/messages.ts`

**Interfaces:**
- Produces: `ConstraintKind`, `RecoveryPhase`, `VRFrame.head_q`, `RobotStateMessage.constraint`, and `RobotStateMessage.recovery_phase`.
- Produces: a `Settings` instance containing every configured control/recovery scalar.
- Consumes: existing protocol version `1`; all added fields are optional to retain compatibility.

- [ ] **Step 1: Add exact configuration values**

Add these keys to `config/default.yaml` and retain existing simulator-only values:

```yaml
workspace_radius_m: 0.45
rotation_dead_zone_deg: 2.0
constraint_clear_ms: 100
home_joint_speed_radps: 0.25
home_timeout_s: 15.0
home_position_tolerance_rad: 0.017453292519943295
home_velocity_tolerance_radps: 0.02
home_stable_ms: 300
```

Change `translation_scale` to `0.5` and `rotation_scale` to `0.5`.

- [ ] **Step 2: Make `Settings` parse and validate every consumed value**

Expand the frozen dataclass in `backend/app/config.py` with typed fields for `control_hz`, `state_hz`, stale thresholds, motion limits, mapping scales, workspace radius, rotation dead zone, joint values, and Home thresholds. Use small helpers that reject missing, boolean, non-finite, zero, or negative numeric values. Keep the existing hard check:

```python
if backend != "simulator":
    raise RuntimeError("real_robot_disabled")
```

- [ ] **Step 3: Add optional protocol fields without bumping v1**

In Python define:

```python
ConstraintKind = Literal["workspace_boundary", "ik_boundary", "joint_boundary"]
RecoveryPhase = Literal["stopping", "homing", "stabilizing"]

class VRFrame(StrictMessage):
    # existing fields
    head_q: Quat | None = None

class RobotStateMessage(StrictMessage):
    # existing fields
    fault: str | None = None
    constraint: ConstraintKind | None = None
    recovery_phase: RecoveryPhase | None = None
```

Validate and normalize `head_q` with the same quaternion rule as `Pose.q`.

Mirror the unions and optional properties in `web/src/protocol/messages.ts`. Update `isVRFrame` and `isRobotStateMessage` exact-key checks to allow the new optional keys while still rejecting unknown fields.

- [ ] **Step 4: Inject configured control objects**

Change `RobotControl.__init__` to accept configured `CoordinateMapper`, `SafetyLimiter`, and Home settings rather than silently constructing defaults. Construct these objects in `backend/app/main.py` from `Settings` and pass them into `RobotControl`.

- [ ] **Step 5: Review and commit without running tests**

Review only the staged diff for accidental real-robot enablement or protocol-version changes, then commit:

```text
feat: wire teleoperation runtime settings
```

### Task 2: User-relative Grip zero and continuous rotation mapping

**Files:**
- Modify: `backend/app/control/coordinate_mapper.py`
- Modify: `backend/app/control/robot_control.py`
- Modify: `web/src/scenes/simulationScene.ts`
- Modify: `web/src/xr/session.ts`

**Interfaces:**
- Consumes: `VRFrame.head_q: Quat | None` from Task 1.
- Produces: `CoordinateMapper.capture(hand: Pose, tcp: Pose, head_q: Quat | None) -> None`.
- Produces: `CoordinateMapper.target(hand: Pose) -> Pose` with user-forward/right/up semantics.

- [ ] **Step 1: Capture a deterministic user basis**

At the Grip rising edge, derive the horizontal basis from `head_q`:

```python
world_up = np.array([0.0, 1.0, 0.0])
head_rotation = Rotation.from_quat(head_q or (0.0, 0.0, 0.0, 1.0))
user_forward = head_rotation.apply((0.0, 0.0, -1.0))
user_forward[1] = 0.0
user_forward /= np.linalg.norm(user_forward)
user_right = np.cross(user_forward, world_up)
user_basis_x = np.column_stack((user_forward, user_right, world_up))
robot_semantic_basis = np.diag((1.0, -1.0, 1.0))
self._r_bx = robot_semantic_basis @ user_basis_x.T
```

Reject a degenerate horizontal head direction with `RuntimeError("invalid_control_basis")`; this is an invalid-numeric hard fault, not an IK boundary.

- [ ] **Step 2: Map translation from the locked zero**

Keep mapping absolute relative to the captured anchors:

```python
delta_x = np.asarray(hand.p) - np.asarray(self._hand_anchor.p)
target_p = np.asarray(self._tcp_anchor.p) + self._r_bx @ (
    self.translation_scale * delta_x
)
```

Do not integrate from the previous target. Clearing/re-capturing the mapper on Grip release/repress must remain unchanged.

- [ ] **Step 3: Apply a continuous rotation dead zone**

Compute `delta_b` with conjugation by `_r_bx`. Convert it to a rotation vector. For magnitude `angle`, use:

```python
effective_angle = max(0.0, angle - self.rotation_dead_zone_rad)
scaled_angle = effective_angle * self.rotation_scale
```

Use identity below the dead zone; otherwise rebuild the rotation from the normalized axis and `scaled_angle`, then left-multiply the captured TCP orientation.

- [ ] **Step 4: Send raw head orientation from WebXR**

Extend `VRFrameInput` with `headQ?: Quat` and copy it to `head_q` only when present. In `XRSessionController.onXRFrame`, read `viewerPose.transform.orientation` and pass the raw quaternion. Desktop frames omit it and therefore use the canonical default basis.

- [ ] **Step 5: Feed the head quaternion into zero capture**

At the `ARMED/HOLD -> ACTIVE` transition in `RobotControl.tick`, call:

```python
self.mapper.capture(
    Pose(p=received.frame.right.p, q=received.frame.right.q),
    state.actual_tcp,
    received.frame.head_q,
)
```

- [ ] **Step 6: Review and commit without running tests**

Verify from code inspection that Grip capture returns the locked hand pose to the locked TCP pose exactly, then commit:

```text
feat: map Grip motion from a user-relative zero
```

### Task 3: Soft workspace and IK boundaries

**Files:**
- Modify: `backend/app/control/safety.py`
- Modify: `backend/app/control/robot_control.py`
- Modify: `backend/app/robots/sim_adapter.py`

**Interfaces:**
- Produces: `WorkspaceProjection(pose: Pose, constrained: bool)`.
- Produces: `SafetyLimiter.project_workspace(requested: Pose) -> WorkspaceProjection`.
- Produces: `SafetyLimiter.limit_motion(previous: Pose, requested: Pose, dt: float) -> Pose`.
- Produces: `RobotControl._constraint: ConstraintKind | None` in authoritative state.

- [ ] **Step 1: Replace the axis-aligned exception with spherical projection**

Remove `SafetyViolation("workspace_violation")` from normal target handling. Introduce:

```python
@dataclass(frozen=True)
class WorkspaceProjection:
    pose: Pose
    constrained: bool
```

When an anchor exists and `norm(requested.p - anchor) > workspace_radius`, scale the displacement to exactly `workspace_radius`, retain the requested quaternion, and return `constrained=True`. Otherwise return the original pose and `False`.

- [ ] **Step 2: Separate workspace projection from speed/acceleration limiting**

Rename the existing dynamic portion to `limit_motion`. In `RobotControl.tick` use this order:

```python
raw_requested = self.mapper.target(hand_pose)
workspace = self.limiter.project_workspace(raw_requested)
filtered = self.filter.update(workspace.pose, 0.02)
target = self.limiter.limit_motion(self.last_target, filtered, 0.02)
```

Set `workspace_boundary` when `workspace.constrained` is true, but still send the projected target.

- [ ] **Step 3: Use the last successful IK solution as the next seed**

Add `self._ik_seed_q = np.asarray(self.model.home_q)` to `SimRobotAdapter`. Solve with `_ik_seed_q`; update it only after `solve_ik` and `set_target_q` both succeed. Do not change it on failed requests.

- [ ] **Step 4: Convert legal IK failures into soft constraints**

In the ACTIVE branch, catch `BackendCommandError` around only `command_tcp`:

```python
try:
    await self.backend.command_tcp(target, received.frame.seq)
except BackendCommandError as error:
    code = str(error)
    if code in {"ik_unreachable", "ik_singular"}:
        self._set_constraint("ik_boundary", now_ns)
    elif code == "joint_safety_window":
        self._set_constraint("joint_boundary", now_ns)
    else:
        raise
else:
    self.last_target = target
```

Do not call `_enter_fault` for the three soft codes. Keep the last valid target and keep the teleoperation state ACTIVE.

- [ ] **Step 5: Add deterministic constraint clearing**

Store the latest constraint and a first-valid timestamp. A constrained frame clears the valid timer. Valid frames keep control flowing immediately; after `constraint_clear_ms` of valid frames, clear the presentation constraint. Reset the constraint on disconnect, disarm, hard fault, and completed recovery.

Populate `constraint=self._constraint` in `state_message`. Remove workspace/IK/joint target codes from the path that creates a new latched fault; retain handling of illegal numeric data and unexpected backend errors.

- [ ] **Step 6: Review and commit without running tests**

Inspect every `_enter_fault` caller and confirm the three legal target-boundary codes cannot reach it, then commit:

```text
feat: keep target boundaries non-latching
```

### Task 4: Simulator Home recovery with visible phases

**Files:**
- Modify: `backend/app/robots/base.py`
- Modify: `backend/app/robots/lebai_adapter.py`
- Modify: `backend/app/sim/virtual_robot.py`
- Modify: `backend/app/robots/sim_adapter.py`
- Modify: `backend/app/control/robot_control.py`

**Interfaces:**
- Produces: `RobotBackend.home(options: HomeOptions, on_phase: Callable[[str], None]) -> Awaitable[None]`.
- Produces: `HomeOptions(max_speed_radps, timeout_s, position_tolerance_rad, velocity_tolerance_radps, stable_seconds)`.
- Produces: `RobotControl._recovery_phase: RecoveryPhase | None`.

- [ ] **Step 1: Add a backend recovery contract**

Define a frozen `HomeOptions` dataclass in `backend/app/robots/base.py` and add `home(options, on_phase)` to `RobotBackend`. The callback accepts only `"homing"` or `"stabilizing"`. `RealLebaiAdapter.home` must call `_disabled()` exactly like all other real-robot operations.

- [ ] **Step 2: Add per-target joint speed to the virtual robot**

Change `VirtualRobot.set_target_q` to accept `max_speed_radps: float | None = None`, validate it as finite and positive, and store it for the active target. In `step`, clip desired joint velocity by the smaller of the model limit and target override. `stop()` clears both the target and override.

- [ ] **Step 3: Implement asynchronous simulator Home**

`SimRobotAdapter.home` calls `on_phase("homing")`, sets `model.home_q` with the configured `0.25 rad/s` override, then polls at `STEP_SECONDS` without holding `_lock` across sleeps. When error and velocity first enter tolerance it calls `on_phase("stabilizing")`; if either leaves tolerance before the stable interval completes, it calls `on_phase("homing")` and restarts the interval. It requires both thresholds continuously for `stable_seconds`. On timeout, stop the robot and raise `BackendCommandError("home_timeout")`. On success, set `_ik_seed_q` to Home.

- [ ] **Step 4: Orchestrate recovery before clearing the fault**

In `RobotControl.reset_fault`:

1. keep all existing generation/race checks;
2. set `recovery_phase="stopping"` and call `backend.stop`;
3. pass a callback that assigns only `"homing"` or `"stabilizing"` to `_recovery_phase` and await `backend.home(self.home_options, callback)`;
4. let the simulator callback reflect whether the arm is travelling or inside the stability window;
5. only then clear mapper/filter/limiter, target, fault, constraint, and disarm;
6. clear `recovery_phase` in `finally` on rejection or failure.

Populate `recovery_phase` in `state_message` so the independent state sender can report progress while the reset request waits.

- [ ] **Step 5: Contain recovery failures**

Map `home_timeout` and other Home exceptions to a rejected `FaultResetResult` with reason `stop_incomplete`; retain the original fault and locked state. Never partially clear a fault before Home succeeds.

- [ ] **Step 6: Review and commit without running tests**

Inspect the reset path for every early return and exception, confirming `_recovery_phase` cannot remain stale and real hardware is still disabled, then commit:

```text
feat: return simulator to Home during reset
```

### Task 5: Three-dimensional left-hand workspace placement

**Files:**
- Modify: `web/src/xr/controllerInput.ts`
- Modify: `web/src/xr/session.ts`
- Create: `web/src/scenes/workspacePlacementController.ts`
- Modify: `web/src/scenes/simulationScene.ts`

**Interfaces:**
- Produces: `LeftControllerSample.thumbstickX`, `.thumbstickY`, `.grip`, and `.thumbstickPressed`.
- Produces: `WorkspacePlacementController.update(input) -> Vec3`.
- Consumes: current `ArmSafetySnapshot` and `XRPresentationSample`.

- [ ] **Step 1: Read the complete left-hand input**

Use Quest axes index `2` for X and `3` for Y, button index `1` for left Grip, and button index `3` for stick press. Normalize all values. Invalid tracking returns zeros and false values.

- [ ] **Step 2: Implement a focused XYZ placement controller**

Create `WorkspacePlacementController` with:

```typescript
export interface WorkspacePlacementInput {
  headY: number | null;
  axisX: number;
  axisY: number;
  heightModifier: boolean;
  resetPressed: boolean;
  enabled: boolean;
  nowMs: number;
}

export class WorkspacePlacementController {
  beginSession(): void;
  endSession(): Vec3;
  update(input: WorkspacePlacementInput): Vec3;
}
```

Preserve the current head-height baseline calculation. Without left Grip, integrate X into lateral offset and `-axisY` into depth offset. With left Grip, integrate X laterally and `-axisY` vertically. Apply the exact global bounds and speeds from the design. Persist three manual offsets under new stable local-storage keys. Stick-press edge resets all three to zero; require a neutral stick after disabled input before movement resumes.

- [ ] **Step 3: Apply placement only to the visual root**

Replace the `TableHeightController` usage in `SimulationScene` with the XYZ controller. Apply its return value to `robotVisualRoot.position`; do not modify `VRFrame.right`, backend state, target TCP, or joint values.

Enable placement only when phase is `locked`, `stopped`, or `fault`, no arm/reset request is pending, and right Grip is released. End XR by resetting the desktop visual root to `(0, 0, 0)` while retaining stored offsets for the next XR session.

- [ ] **Step 4: Review and commit without running tests**

Inspect the frame payload and confirm no left-hand placement value is sent as a robot command, then commit:

```text
feat: place the VR workbench in three dimensions
```

### Task 6: Constraint, recovery, and haptic presentation

**Files:**
- Modify: `web/src/ui/armPanel.ts`
- Modify: `web/src/ui/hud.ts`
- Modify: `web/src/scenes/vrSafetyPanel.ts`
- Modify: `web/src/xr/session.ts`
- Modify: `web/src/main.ts`
- Modify: `web/src/styles.css`

**Interfaces:**
- Consumes: `RobotStateMessage.constraint` and `.recovery_phase`.
- Produces: `ArmPanel.setConstraint`, `ArmPanel.setRecoveryPhase`, and matching snapshot fields.
- Produces: `XRSessionController.setConstraint(constraint)` for edge-triggered haptics.

- [ ] **Step 1: Keep soft constraints separate from faults**

Add `constraint` and `recoveryPhase` to `ArmSafetySnapshot`. `setConstraint` must not change armed/eligible/mode/fault fields. Reset these fields only on disconnect or authoritative state updates.

- [ ] **Step 2: Display soft boundaries without red fault styling**

Add an amber constraint row to the desktop HUD and readable labels:

```typescript
workspace_boundary: '已到达操作边界'
ik_boundary: '当前方向暂时不可达'
joint_boundary: '已到达关节操作边界'
```

In the VR panel, hard fault and recovery remain highest priority. During ACTIVE with a constraint, show the amber boundary title and “将手柄移回可达区域，无需复位”. Update the footer to describe left-stick plane movement and left-Grip height adjustment.

- [ ] **Step 3: Show actual Home recovery phases**

When `faultResetPending` is true, choose copy from `recoveryPhase`:

```text
stopping -> 正在确认停止
homing -> 正在回到初始姿态
stabilizing -> 正在确认 Home 稳定
null -> 正在准备复位
```

Keep the reset button disabled until the matching `fault_reset_result` arrives.

- [ ] **Step 4: Add safe optional haptic feedback**

Implement `XRSessionController.setConstraint`. On transition from null to a constraint, or between constraint kinds, locate the right-hand input source and call its first available haptic actuator with approximately `pulse(0.35, 40)`. Catch unsupported APIs and rejected promises. Do not repeat while the same constraint remains active; this is stricter than the `300 ms` maximum frequency.

- [ ] **Step 5: Route authoritative state in `main.ts`**

For every robot state, call the new panel, HUD, and XR-controller methods before rendering. A null constraint clears the amber presentation. Recovery phase comes only from the backend and is never synthesized from timers in the browser.

- [ ] **Step 6: Review and commit without running tests**

Inspect priority order so a soft boundary can never hide connection loss, stale input, hard fault, or reset progress, then commit:

```text
feat: present teleoperation boundaries and recovery
```

### Task 7: Operator documentation and manual handoff

**Files:**
- Modify: `README.md`
- Reference: `docs/superpowers/specs/2026-07-22-teleoperation-ux-and-recovery-design.md`

**Interfaces:**
- Consumes: final control bindings and status semantics from Tasks 1–6.
- Produces: a concise manual acceptance sequence for the user.

- [ ] **Step 1: Update only the relevant README operation section**

Document:

- A unlocks; B stops or begins a real fault recovery;
- right Grip captures a new hand/TCP zero and controls relative six-DoF motion;
- Trigger controls the gripper;
- left stick moves the visual workbench laterally and in depth;
- left Grip plus stick Y changes workbench height;
- left stick press resets visual placement;
- amber boundary prompts recover automatically and do not require reset;
- hard faults stay locked; simulator recovery returns to Home before clearing.

Preserve every unrelated user edit already present in `README.md`.

- [ ] **Step 2: Perform a no-execution consistency review**

Do not run tests, builds, or the app. Review only changed-file diffs for:

- real-robot calls;
- Cannon-es or new physics dependencies;
- mismatch between Python and TypeScript optional fields;
- soft boundary codes entering `_enter_fault`;
- left-workspace values entering backend commands;
- fault clearing before Home success;
- accidental inclusion of the user's unrelated files.

- [ ] **Step 3: Commit documentation and hand off**

Commit only the relevant README hunk:

```text
docs: explain intuitive VR teleoperation controls
```

Provide the user with the ten-step manual acceptance checklist from the approved design and explicitly state that no automated test, build, type-check, or program launch was performed.
