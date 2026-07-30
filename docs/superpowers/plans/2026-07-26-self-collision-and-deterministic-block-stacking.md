# Self-Collision and Deterministic Block Stacking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent the simulated LM3 from entering visual self-collision states, repair world-space cube grasping, and add five deterministic, non-overlapping stackable blocks without enabling real hardware or adding a physics engine.

**Architecture:** Add versioned capsule proxies to the shared GLB kinematics configuration and reject colliding differential-IK candidate steps before they reach `VirtualRobot`. Keep block interaction frontend-only, but drive it from authoritative `RobotStateMessage.actual_tcp` and `gripper`, with all blocks remaining under the unscaled `robotVisualRoot`; releases select the highest deterministic support surface and resolve AABB overlap.

**Tech Stack:** Python 3.11+, NumPy, SciPy Rotation, FastAPI/Pydantic, pytest, TypeScript 5.9, Three.js, Vitest, Vite.

## Global Constraints

- Work only in `D:\MyWork\VR4Arm\.worktrees\teleoperation-ux-recovery` on `codex/teleoperation-ux-recovery`.
- `backend: simulator` remains mandatory; do not import, connect to, or call the Lebai SDK.
- Do not add Cannon-es, Rapier, Ammo, MuJoCo, Isaac Sim, collision libraries, or any new dependency.
- Preserve protocol version `1`; only add `self_collision` to the existing `constraint` enum.
- Preserve 50 Hz control/simulator timing, 50 Hz robot-state feedback, fixed 1:1 orientation, startup-configurable translation scale, Home behavior, A/B/Grip/Trigger bindings, camera/time-sync extension points, and the single-controller model.
- Self-collision is an amber soft constraint. It must hold the last safe joint state and recover automatically when the commanded direction becomes safe.
- `Home ± π` remains a numerical session window, not a claim about LM3 hardware joint stops.
- Blocks remain frontend-only and are not recorded or added to the WebSocket protocol.
- Static/released blocks must not overlap the table or one another. Continuous carried-block physics is explicitly out of scope.
- Preserve existing unrelated working-tree changes and do not stage them:
  - `backend/app/sim/ik.py`
  - `README.md` changes not made by this plan
  - `docs/quest-development.md` changes not made by this plan
  - `docs/superpowers/plans/2026-07-22-tool-frame-and-ik-continuity.md`
  - `docs/superpowers/specs/2026-07-22-tool-frame-and-ik-continuity-design.md`
- Run only the focused tests named by each task, one frontend build, and the final focused verification. Do not run soak or the complete historical suite.

## File Structure

- `config/lm3_visual_kinematics_v1.json`: authoritative collision proxy radii, safety margin, segment endpoints, and checked pairs.
- `backend/app/sim/lm3_model.py`: validated immutable collision configuration.
- `backend/app/sim/kinematics.py`: expose the eight chain points used by FK, Jacobian, and collision proxies.
- `backend/app/sim/self_collision.py`: segment distance and configured self-collision evaluation.
- `backend/app/sim/cartesian_servo.py`: reject a colliding `next_q` before producing a velocity command.
- `backend/app/robots/sim_adapter.py`: translate a rejected collision step into `BackendCommandError("self_collision")`.
- `backend/app/control/robot_control.py`: map `self_collision` to the existing soft-constraint state.
- `backend/app/schemas/messages.py`, `schemas/teleop-v1.json`: canonical protocol-v1 constraint extension.
- `web/src/protocol/messages.ts`: frontend exact guard for the new constraint.
- `web/src/ui/hud.ts`, `web/src/scenes/vrSafetyPanel.ts`: amber operator guidance.
- `web/src/scenes/kinematicGraspController.ts`: multi-block world-space capture, carry, release, and stacking.
- `web/src/scenes/simulationScene.ts`: create five blocks and feed authoritative TCP/gripper state.
- `docs/quest-development.md`: focused operator acceptance instructions.

---

### Task 1: Shared collision proxies and self-collision evaluator

**Files:**
- Modify: `config/lm3_visual_kinematics_v1.json`
- Modify: `backend/app/sim/lm3_model.py`
- Modify: `backend/app/sim/kinematics.py`
- Create: `backend/app/sim/self_collision.py`
- Create: `backend/tests/sim/test_self_collision.py`
- Modify: `backend/tests/sim/test_kinematics.py`

**Interfaces:**
- Produces: `CollisionSegment(name: str, start: str, end: str, radius_m: float)`.
- Produces: `LM3Model.collision_segments`, `.collision_check_pairs`, and `.collision_safety_margin_m`.
- Produces: `chain_points(q, model) -> dict[str, np.ndarray]`.
- Produces: `segment_distance(a0, a1, b0, b1) -> float`.
- Produces: `is_self_colliding(q, model) -> bool`.
- Preserves: `forward_matrix`, `forward_pose`, and `geometric_jacobian`.

- [ ] **Step 1: Add failing collision-model tests**

Create `backend/tests/sim/test_self_collision.py`:

```python
import numpy as np
import pytest

from app.sim.lm3_model import LM3Model
from app.sim.self_collision import is_self_colliding, segment_distance


FOLDED_COLLISION_Q = np.asarray(
    (-2.9743, -3.6972, 2.7814, 1.3861, 2.4257, -3.7945)
)


def test_segment_distance_handles_crossing_and_parallel_segments() -> None:
    assert segment_distance(
        np.asarray((-1.0, 0.0, 0.0)),
        np.asarray((1.0, 0.0, 0.0)),
        np.asarray((0.0, -1.0, 0.0)),
        np.asarray((0.0, 1.0, 0.0)),
    ) == pytest.approx(0.0)
    assert segment_distance(
        np.asarray((0.0, 0.0, 0.0)),
        np.asarray((1.0, 0.0, 0.0)),
        np.asarray((0.0, 0.2, 0.0)),
        np.asarray((1.0, 0.2, 0.0)),
    ) == pytest.approx(0.2)


def test_home_is_clear_and_known_folded_pose_self_collides() -> None:
    model = LM3Model()
    assert is_self_colliding(model.home_q, model) is False
    assert is_self_colliding(FOLDED_COLLISION_Q, model) is True
```

Append one focused contract to `backend/tests/sim/test_kinematics.py`:

```python
def test_chain_points_expose_base_joints_and_tcp() -> None:
    model = LM3Model()
    points = chain_points(model.home_q, model)

    assert tuple(points) == (
        "base", "joint1", "joint2", "joint3",
        "joint4", "joint5", "joint6", "tcp",
    )
    assert all(point.shape == (3,) and np.all(np.isfinite(point)) for point in points.values())
    np.testing.assert_allclose(points["tcp"], forward_matrix(model.home_q, model)[:3, 3])
```

- [ ] **Step 2: Run RED**

From `backend`:

```powershell
C:\Users\Kilo\miniconda3\envs\deepresearch\python.exe -m pytest tests/sim/test_self_collision.py tests/sim/test_kinematics.py::test_chain_points_expose_base_joints_and_tcp -q
```

Expected: collection fails because `self_collision`, `CollisionSegment`, collision model fields, and `chain_points` do not exist.

- [ ] **Step 3: Add the exact versioned collision configuration**

Add this object at the root of `config/lm3_visual_kinematics_v1.json`:

```json
"collision": {
  "safety_margin_m": 0.008,
  "segments": [
    {"name": "base", "start": "base", "end": "joint1", "radius_m": 0.075},
    {"name": "shoulder", "start": "joint1", "end": "joint2", "radius_m": 0.060},
    {"name": "upper_arm", "start": "joint2", "end": "joint3", "radius_m": 0.055},
    {"name": "forearm", "start": "joint3", "end": "joint4", "radius_m": 0.055},
    {"name": "wrist_1", "start": "joint4", "end": "joint5", "radius_m": 0.045},
    {"name": "wrist_2", "start": "joint5", "end": "joint6", "radius_m": 0.045},
    {"name": "tool", "start": "joint6", "end": "tcp", "radius_m": 0.040}
  ],
  "check_pairs": [
    ["base", "forearm"],
    ["base", "wrist_1"],
    ["base", "wrist_2"],
    ["base", "tool"],
    ["shoulder", "wrist_1"],
    ["shoulder", "wrist_2"],
    ["shoulder", "tool"],
    ["upper_arm", "wrist_1"],
    ["upper_arm", "wrist_2"],
    ["upper_arm", "tool"],
    ["forearm", "tool"]
  ]
}
```

Do not change Home, joint speed, acceleration, tool rotation, TCP offset, or the existing numerical joint window.

- [ ] **Step 4: Validate collision configuration in `LM3Model`**

Add:

```python
CHAIN_POINT_NAMES = frozenset(
    {"base", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "tcp"}
)


@dataclass(frozen=True)
class CollisionSegment:
    name: str
    start: str
    end: str
    radius_m: float
```

The loader must reject:

- missing/non-dict `collision`;
- non-positive/non-finite safety margin or radius;
- duplicate segment names;
- endpoints outside `CHAIN_POINT_NAMES`;
- self-pairs, duplicate pairs, or pairs referencing unknown segments.

Expose immutable tuples on `LM3Model`:

```python
collision_segments: tuple[CollisionSegment, ...]
collision_check_pairs: tuple[tuple[str, str], ...]
collision_safety_margin_m: float
```

- [ ] **Step 5: Expose chain points without duplicating FK**

Refactor `_chain()` to continue returning the final TCP matrix, joint origins, and axes. Add:

```python
def chain_points(q: Sequence[float], model: LM3Model) -> dict[str, np.ndarray]:
    matrix, origins, _ = _chain(q, model)
    names = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
    return {
        "base": np.zeros(3, dtype=float),
        **{name: origin.copy() for name, origin in zip(names, origins, strict=True)},
        "tcp": matrix[:3, 3].copy(),
    }
```

Return a fresh mapping and arrays so callers cannot mutate FK state.

- [ ] **Step 6: Implement robust finite segment distance and configured checks**

In `self_collision.py`, use the standard closest-points solution for two finite 3D segments, including degenerate zero-length segments. Normalize all numeric errors to `ValueError("invalid_self_collision_geometry")`.

`is_self_colliding()` must:

1. call `chain_points`;
2. build each configured segment from endpoint names;
3. evaluate only `collision_check_pairs`;
4. compare distance strictly below `radius_a + radius_b + margin`;
5. return `True` on the first collision, otherwise `False`;
6. fail closed on non-finite geometry by raising `ValueError`.

- [ ] **Step 7: Run GREEN and commit**

Run the Step 2 command. Expected: `3 passed`.

Then run:

```powershell
git diff --check -- config/lm3_visual_kinematics_v1.json backend/app/sim/lm3_model.py backend/app/sim/kinematics.py backend/app/sim/self_collision.py backend/tests/sim/test_self_collision.py backend/tests/sim/test_kinematics.py
```

Commit only Task 1 files:

```powershell
git commit -m "feat: add LM3 self-collision proxies"
```

---

### Task 2: Reject colliding servo steps and publish a soft constraint

**Files:**
- Modify: `backend/app/sim/cartesian_servo.py`
- Modify: `backend/app/robots/sim_adapter.py`
- Modify: `backend/app/control/robot_control.py`
- Modify: `backend/app/schemas/messages.py`
- Modify: `schemas/teleop-v1.json`
- Modify: `backend/tests/sim/test_cartesian_servo.py`
- Modify: `backend/tests/sim/test_adapters.py`
- Modify: `backend/tests/control/test_robot_control.py`
- Modify: `backend/tests/contract/test_messages.py`
- Modify: `web/src/protocol/messages.ts`
- Modify: `web/src/ui/hud.ts`
- Modify: `web/src/scenes/vrSafetyPanel.ts`
- Modify: `web/tests/messages.test.ts`
- Modify: `web/tests/hud.test.ts`
- Modify: `web/tests/vrSafetyPanel.test.ts`

**Interfaces:**
- Extends: `CartesianServoResult.self_collision_limited: bool`.
- Produces: `BackendCommandError("self_collision")` for a rejected simulator command.
- Extends: `ConstraintKind` with `"self_collision"` in backend, canonical JSON schema, and frontend.
- Preserves: hard-fault handling and existing workspace/IK/joint soft constraints.

- [ ] **Step 1: Write the failing servo rejection test**

Append to `backend/tests/sim/test_cartesian_servo.py`:

```python
def test_servo_holds_current_q_when_candidate_self_collides(monkeypatch) -> None:
    model = LM3Model()
    q = np.asarray(model.home_q)
    start = forward_pose(q, model)
    target = start.model_copy(update={"p": (start.p[0] + 0.02, start.p[1], start.p[2])})
    monkeypatch.setattr(
        "app.sim.cartesian_servo.is_self_colliding",
        lambda candidate, _model: True,
    )

    result = cartesian_servo_step(target, q, model)

    assert result.self_collision_limited is True
    assert result.q == pytest.approx(q)
    assert result.joint_velocity == pytest.approx(np.zeros(6))
```

Update existing non-colliding assertions to require:

```python
assert result.self_collision_limited is False
```

- [ ] **Step 2: Write failing adapter/control/protocol tests**

Add a simulator adapter case that monkeypatches `cartesian_servo_step` to return a result with `self_collision_limited=True`, then asserts:

```python
with pytest.raises(BackendCommandError, match="^self_collision$"):
    await adapter.command_tcp(target, command_id=8)
assert adapter.robot.target_qd is None
assert adapter.command_id is None
```

Add a RobotControl case using the existing fake backend:

```python
backend.command_error = BackendCommandError("self_collision")
# connect, publish a released frame, arm, then publish an active Grip frame
await control.tick()

assert control.mode is TeleopMode.ACTIVE
assert control._fault is None
assert (await control.state_message()).constraint == "self_collision"
```

Add exact contract expectations:

```python
state = robot_state_fixture.model_copy(update={"constraint": "self_collision"})
assert state.constraint == "self_collision"
```

Frontend tests must assert `isRobotStateMessage({...robotFixture, constraint: 'self_collision'})` is true and both HUD surfaces render:

```text
机械臂接近自碰撞边界，请将手柄退回
```

- [ ] **Step 3: Run RED**

Backend:

```powershell
C:\Users\Kilo\miniconda3\envs\deepresearch\python.exe -m pytest tests/sim/test_cartesian_servo.py tests/sim/test_adapters.py tests/control/test_robot_control.py -k "self_collision" tests/contract/test_messages.py -q
```

Frontend:

```powershell
npm.cmd test -- messages.test.ts hud.test.ts vrSafetyPanel.test.ts
```

Expected: failures because the result field and constraint enum do not exist.

- [ ] **Step 4: Reject collision before simulator integration**

In `cartesian_servo.py`, after joint-window clipping but before returning:

```python
self_collision_limited = is_self_colliding(next_q, model)
if self_collision_limited:
    next_q = q.copy()
    bounded_velocity = np.zeros(6, dtype=float)
else:
    bounded_velocity = (next_q - q) / dt
```

Measure returned pose errors from the final `next_q`. Add the boolean field to every result construction.

In `SimRobotAdapter.command_tcp()`:

```python
if result.self_collision_limited:
    raise BackendCommandError("self_collision")
self.robot.set_target_qd(result.joint_velocity)
self.command_id = command_id
```

Do not update the target or command ID on rejection.

- [ ] **Step 5: Map the adapter rejection to the existing soft-constraint path**

In `RobotControl.tick()`:

```python
elif code == "self_collision":
    self._set_constraint("self_collision")
```

Keep the existing filter reset, limiter motion reset, and last valid target. Do not add `self_collision` to `RECOVERABLE_FAULTS`, because it is not a fault.

- [ ] **Step 6: Extend exact protocol enums**

Update:

```python
ConstraintKind = Literal[
    "workspace_boundary",
    "ik_boundary",
    "joint_boundary",
    "self_collision",
]
```

Add the same enum value to the canonical `schemas/teleop-v1.json`, TypeScript `ConstraintKind`, and `CONSTRAINT_KINDS`. Do not add optional keys or new message types.

Add UI mapping:

```ts
self_collision: '机械臂接近自碰撞边界，请将手柄退回',
```

For the VR presentation, use the constraint label as title and retain the amber tone. The generic instruction may remain “将手柄移回可达区域，无需复位”.

- [ ] **Step 7: Run GREEN and commit**

Run both Step 3 commands. Expected: all selected backend tests and the three frontend files pass.

Commit only Task 2 files:

```powershell
git commit -m "feat: block self-colliding servo steps"
```

---

### Task 3: Multi-block world-space grasp and deterministic placement

**Files:**
- Replace: `web/src/scenes/kinematicGraspController.ts`
- Replace: `web/tests/kinematicGraspController.test.ts`

**Interfaces:**
- Produces: `GraspBlock { id: string; object: THREE.Object3D; sizeM: number }`.
- Produces: `TcpPose { p: readonly [number, number, number]; q: readonly [number, number, number, number] }`.
- Produces: `KinematicGraspController.update(actualTcp: TcpPose, gripper: number): void`.
- Preserves: `.reset(): void`.
- Removes: runtime dependence on `tool`, `tcpOffset`, and one `cube`.

- [ ] **Step 1: Replace the old test with failing real-root/scaled-tool regressions**

Create helpers for five plain meshes under one `visualRoot`. Keep a separate scaled tool only to prove it is irrelevant:

```ts
const scaledTool = new THREE.Group();
scaledTool.scale.setScalar(0.007950832135975361);
visualRoot.add(scaledTool);
```

Add these tests:

1. closest block inside the oriented capture box is selected;
2. captured block remains a direct child of `visualRoot` and keeps world scale `[1, 1, 1]`;
3. updating TCP moves and rotates only the carried block;
4. release to an empty table produces center Y `0.025`;
5. release over one block produces center Y `0.085`;
6. sequential release of five blocks produces Y centers `[0.025, 0.085, 0.145, 0.205, 0.265]`;
7. pairwise block AABBs do not overlap after every release;
8. closing far from all blocks consumes the close attempt; moving near while still closed does not vacuum-grab until Trigger is released and closed again.

- [ ] **Step 2: Run RED**

```powershell
npm.cmd test -- kinematicGraspController.test.ts
```

Expected: TypeScript/behavior failures because the current constructor and `update()` accept one cube and one scalar only.

- [ ] **Step 3: Define the exact multi-block API and capture state**

Use:

```ts
export interface GraspBlock {
  id: string;
  object: THREE.Object3D;
  sizeM: number;
}

export interface TcpPose {
  p: readonly [number, number, number];
  q: readonly [number, number, number, number];
}

export interface KinematicGraspOptions {
  visualRoot: THREE.Object3D;
  blocks: readonly GraspBlock[];
  tableTopY: number;
  tableHalfWidth: number;
  tableHalfDepth: number;
  captureHalfExtents?: THREE.Vector3;
  closeThreshold?: number;
  releaseThreshold?: number;
  supportCenterTolerance?: number;
}
```

Constructor validation must reject duplicate IDs, duplicate objects, non-positive/non-finite sizes, and blocks not parented to `visualRoot`.

Maintain:

```ts
private carried: GraspBlock | null = null;
private closeArmed = true;
```

When `gripper <= 0.35`, re-arm closing. On the first update with `gripper >= 0.65`, consume the close attempt whether or not a block is found.

- [ ] **Step 4: Select candidates in the authoritative TCP frame**

Build a TCP position/quaternion after finite and normalized-quaternion validation. For each free block:

```ts
const local = block.object.position.clone()
  .sub(tcpPosition)
  .applyQuaternion(tcpQuaternion.clone().invert());
```

Accept when each absolute component is within default half extents:

```ts
new THREE.Vector3(0.055, 0.045, 0.055)
```

Select the accepted block with the smallest squared world distance; break exact ties by stable input order.

- [ ] **Step 5: Carry without reparenting or inherited scale**

For the carried block:

```ts
block.object.position.copy(tcpPosition);
block.object.quaternion.copy(tcpQuaternion);
block.object.scale.set(1, 1, 1);
```

Never call `tool.add`, `Object3D.attach`, or change its parent. Update matrices after the pose write.

- [ ] **Step 6: Release to the highest deterministic support**

Use:

```ts
const half = block.sizeM / 2;
const x = clamp(block.object.position.x, -tableHalfWidth + half, tableHalfWidth - half);
const z = clamp(block.object.position.z, -tableHalfDepth + half, tableHalfDepth - half);
```

A support block is eligible when:

```ts
Math.abs(other.object.position.x - x) <= 0.039
&& Math.abs(other.object.position.z - z) <= 0.039
```

Choose the eligible block with the highest top surface. If selected, align X/Z to its center. Otherwise use the clamped release X/Z and table top.

Set identity quaternion and center Y to support top plus half. Re-check all static blocks with strict AABB overlap:

```ts
Math.abs(dx) < (a.sizeM + b.sizeM) / 2 - 1e-9
```

on all three axes. While overlap remains, raise the released block to the highest horizontally overlapping top. Cap the loop at `blocks.length`; throw `invalid_block_stack` if it cannot resolve, because silent penetration is forbidden.

- [ ] **Step 7: Implement deterministic reset**

Store every block’s initial position, quaternion, and scale. `reset()` must:

- keep/reparent every block to `visualRoot` if needed;
- restore all initial transforms;
- clear `carried`;
- set `closeArmed = true`;
- update matrices.

- [ ] **Step 8: Run GREEN and commit**

Run the Step 2 command. Expected: all focused grasp/stacking tests pass.

Commit only:

```powershell
git commit -m "feat: add deterministic multi-block stacking"
```

---

### Task 4: Scene integration, five blocks, and operator guidance

**Files:**
- Modify: `web/src/scenes/simulationScene.ts`
- Modify: `web/tests/simulationScene.test.ts`
- Modify: `docs/quest-development.md`

**Interfaces:**
- Produces: `createGraspBlocks() -> GraspBlock[]` with five exact IDs/colors/positions.
- Consumes: `RobotStateMessage.actual_tcp` and `.gripper`.
- Preserves: scene disposal, robot model animation, target marker, workspace placement, controller hints, and VR HUD placement.

- [ ] **Step 1: Add a failing five-block scene helper test**

Export a resource helper that does not require `WebGLRenderer`:

```ts
const blocks = createGraspBlocks();
expect(blocks.map(({id}) => id)).toEqual([
  'block-orange',
  'block-blue',
  'block-green',
  'block-yellow',
  'block-purple',
]);
expect(blocks.map(({object}) => (object.material as THREE.MeshStandardMaterial).color.getHex()))
  .toEqual([0xff8a3d, 0x39a8ff, 0x58d68d, 0xffd84d, 0xa77bff]);
expect(blocks.every(({sizeM}) => sizeM === 0.06)).toBe(true);
```

Assert initial positions equal:

```ts
[
  [0.18, 0.025, -0.32],
  [0.32, 0.025, -0.22],
  [0.04, 0.025, -0.28],
  [0.28, 0.025, -0.38],
  [0.10, 0.025, -0.18],
]
```

- [ ] **Step 2: Run RED**

```powershell
npm.cmd test -- simulationScene.test.ts
```

Expected: failure because `createGraspBlocks` does not exist and the scene still owns one `graspCube`.

- [ ] **Step 3: Replace the single cube with five blocks**

Replace:

```ts
private readonly graspCube = createGraspCube();
```

with:

```ts
private readonly graspBlocks = createGraspBlocks();
```

Add every `block.object` directly to `robotVisualRoot` before model loading. Construct `KinematicGraspController` with `visualRoot`, `blocks`, table top `-0.005`, half width `0.61`, and half depth `0.43`. The controller no longer needs `model.tool` or `LM3_TCP_OFFSET`.

- [ ] **Step 4: Feed authoritative TCP and gripper together**

After applying sampled joints and gripper:

```ts
this.graspController?.update(sample.state.actual_tcp, sample.state.gripper);
```

Do not derive TCP from `robotgrabber`. Keep `robotVisualRoot.updateMatrixWorld(true)` for normal GLB rendering, but grasp correctness must no longer depend on tool-node scale.

- [ ] **Step 5: Update concise Quest acceptance instructions**

In `docs/quest-development.md`, add a section adjacent to current artificial acceptance steps:

```text
- 五个彩色方块均可抓取；Trigger 闭合时只选择夹爪中心内最近的方块。
- 方块抓取后尺寸不得变化或消失。
- Trigger 松开可放到桌面或另一个方块顶部；静止方块不得互相穿透。
- “机械臂接近自碰撞边界”是琥珀色软约束；退回手柄即可恢复，无需复位。
```

Retain simulator-only and real-robot-disabled warnings. Do not describe gravity, falling, friction, or true-robot collision as implemented.

- [ ] **Step 6: Run GREEN, build, and commit**

```powershell
npm.cmd test -- simulationScene.test.ts kinematicGraspController.test.ts messages.test.ts hud.test.ts vrSafetyPanel.test.ts
npm.cmd run build
```

Expected: selected frontend files pass; Vite build exits `0`. The existing chunk-size warning is non-blocking.

Use `git diff --check`, then commit only Task 4 files:

```powershell
git commit -m "feat: integrate five graspable blocks"
```

---

### Task 5: Focused final verification, scope audit, and restart

**Files:**
- No product-file changes expected.
- Reference: `docs/superpowers/specs/2026-07-26-self-collision-and-deterministic-block-stacking-design.md`

**Interfaces:**
- Verifies: backend self-collision rejection and frontend grasp/stacking.
- Verifies: simulator-only runtime health.

- [ ] **Step 1: Run the complete agreed backend selection**

From `backend`:

```powershell
C:\Users\Kilo\miniconda3\envs\deepresearch\python.exe -m pytest tests/sim/test_self_collision.py tests/sim/test_kinematics.py tests/sim/test_cartesian_servo.py tests/sim/test_adapters.py tests/control/test_robot_control.py tests/contract/test_messages.py -k "self_collision or chain_points or position_priority_servo or commands_velocity_without_a_second_position_servo or home_keeps_tcp or home_request" -q
```

Expected: all selected tests pass with zero failures.

- [ ] **Step 2: Run the complete agreed frontend selection and build**

From `web`:

```powershell
npm.cmd test -- kinematicGraspController.test.ts simulationScene.test.ts messages.test.ts hud.test.ts vrSafetyPanel.test.ts
npm.cmd run build
```

Expected: all five selected test files pass and build exits `0`.

- [ ] **Step 3: Audit the complete feature range**

Confirm:

- no Lebai SDK import, IP address, or real backend enablement;
- no package manifest/lockfile or physics dependency change;
- no block pose in protocol or recording;
- no nonlinear teleoperation scale or changed control frequency;
- no changes to the unrelated IK and tool-frame files;
- constraint schema agrees across Python, canonical JSON, and TypeScript;
- `git diff --check` reports no whitespace errors.

- [ ] **Step 4: Restart exact simulator services**

Identify and stop only listeners currently bound to `127.0.0.1:8000` and `0.0.0.0:5173`, verifying their process names before stopping. Restart:

```powershell
# backend workdir
python -m uvicorn app.main:app --host 127.0.0.1 --port 8000

# web workdir
npm.cmd run dev
```

Verify:

```json
{"status": "ok", "backend": "SIMULATOR", "real_robot_enabled": false}
```

and Vite reports:

```text
https://192.168.1.8:5173/
```

- [ ] **Step 5: Hand off the Quest check**

Ask the user to close the PC controller page, refresh Quest, and check:

1. normal Grip following remains responsive;
2. folding toward the arm stops before visual overlap with an amber self-collision message;
3. retreating resumes without reset;
4. all five colored blocks can be captured and remain full size;
5. a block can be returned to the table;
6. two- and five-layer stacks settle without static overlap;
7. Home and B/A safety behavior are unchanged.

Do not start real-robot implementation until the user confirms this Quest acceptance.

## Self-Review

- Spec coverage: every approved collision, grasp, stacking, error, testing, and real-hardware boundary requirement maps to a task.
- Placeholder scan: all file paths, interfaces, numeric constants, protocol values, test commands, expected failures, and commit messages are explicit.
- Type consistency: `self_collision_limited` flows from servo result to adapter rejection; `"self_collision"` flows through backend schema, canonical JSON, RobotControl, TypeScript guard, and both HUDs.
- Grasp consistency: the controller consumes `actual_tcp`, all blocks stay under `robotVisualRoot`, and no tool scale or TCP local offset remains in the grasp path.
- Scope: no physics dependency, true-robot code, camera/recording implementation, or protocol block state is introduced.
