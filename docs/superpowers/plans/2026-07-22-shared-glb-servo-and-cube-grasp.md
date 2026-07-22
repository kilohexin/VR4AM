# Shared GLB Servo and Cube Grasp Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the simulated LM3 visually follow Quest controller translation and rotation by sharing the GLB kinematic chain between FK, differential IK, and rendering, then add one deterministic graspable desktop cube.

**Architecture:** Replace the simulator's MDH runtime model with a versioned JSON description of the actual GLB node chain. Keep the existing FastAPI/WebSocket/state-machine boundary, but make `SimRobotAdapter.command_tcp` issue one position-priority damped differential-IK step from actual joints. Keep the cube frontend-only: it attaches to the GLB `robotgrabber` TCP when the authoritative gripper value closes nearby and returns to the tabletop when opened.

**Tech Stack:** Python 3.11+, NumPy, SciPy Rotation, FastAPI, TypeScript 5.9, Three.js, Vitest, Vite.

## Global Constraints

- `backend: simulator` remains mandatory; never import or contact the Lebai SDK.
- Do not add dependencies, Cannon-es, collision, gravity, friction, contact, or rigid-body physics.
- Preserve protocol v1, raw controller poses, camera/time-sync extension interfaces, safety state machine, and Home recovery flow.
- Position tracking has priority over orientation; ordinary orientation limitation is not a hard fault.
- Use only three focused behavior tests plus one frontend build; do not run soak, random reachability, or the entire historical test suite.
- Preserve unrelated working-tree files and the user's root checkout changes.

## File Structure

- `config/lm3_visual_kinematics_v1.json`: single source for GLB node names, axes, offsets, Home, tool transform, TCP, and simulator joint limits.
- `backend/app/sim/lm3_model.py`: validated immutable view of the shared JSON.
- `backend/app/sim/kinematics.py`: GLB-chain FK and geometric Jacobian.
- `backend/app/sim/cartesian_servo.py`: one position-priority damped differential-IK step.
- `backend/app/robots/sim_adapter.py`: apply servo steps to the virtual robot instead of global target IK.
- `backend/app/control/coordinate_mapper.py`: direct WebXR/Three.js translation and calibrated controller-local rotation.
- `web/src/robot/robotModel.ts`: consume shared node/axis metadata and expose `robotgrabber`.
- `web/src/scenes/kinematicGraspController.ts`: deterministic cube attach/release behavior.
- `web/src/scenes/simulationScene.ts`: create the cube and update grasp state from authoritative robot state.
- `README.md`: describe position-priority following and simple cube interaction.

---

### Task 1: Shared GLB kinematics source and backend FK

**Files:**
- Create: `config/lm3_visual_kinematics_v1.json`
- Modify: `backend/app/sim/lm3_model.py`
- Modify: `backend/app/sim/kinematics.py`
- Modify: `backend/tests/sim/test_kinematics.py`

**Interfaces:**
- Produces: `LM3Model.joint_names`, `.joint_axes`, `.joint_offsets_m`, `.tool_root_offset_m`, `.tool_rotation_xyzw`, `.tcp_offset_m`.
- Produces: `forward_matrix(q, model) -> np.ndarray`, `forward_pose(q, model) -> Pose`, and `geometric_jacobian(q, model) -> np.ndarray` with shape `(6, 6)`.

- [ ] **Step 1: Replace the old MDH golden test with one focused GLB-chain test**

Keep validation coverage small. The test must assert that the new model loads six named axes, Home FK is a rigid transform in the Three.js frame, and the geometric Jacobian predicts the same small TCP displacement as FK:

```python
def test_shared_glb_fk_and_jacobian_are_consistent() -> None:
    model = LM3Model()
    q = np.asarray(model.home_q)
    matrix = forward_matrix(q, model)
    jacobian = geometric_jacobian(q, model)
    dq = np.array([1e-5, -2e-5, 1e-5, 0.0, 1e-5, 0.0])
    actual_delta = np.asarray(forward_pose(q + dq, model).p) - matrix[:3, 3]

    assert model.joint_names == tuple(f"Joint{i}" for i in range(1, 7))
    assert model.joint_axes == ("y", "z", "z", "z", "y", "z")
    assert np.linalg.det(matrix[:3, :3]) == pytest.approx(1.0)
    np.testing.assert_allclose(actual_delta, jacobian[:3] @ dq, atol=1e-8)
```

- [ ] **Step 2: Run RED**

Run from `backend`:

```powershell
python -m pytest tests/sim/test_kinematics.py -q
```

Expected: FAIL because the MDH `LM3Model` has no `joint_names`, `joint_axes`, or GLB geometric Jacobian.

- [ ] **Step 3: Create the exact shared JSON**

Use version `1`, Home `[0, -0.7853981634, 1.5707963268, -0.7853981634, 1.5707963268, 0]`, axes `y,z,z,z,y,z`, and the six node translations extracted from `Lebai_LM3.glb`. Include:

```json
{
  "version": 1,
  "home_q": [0.0, -0.7853981634, 1.5707963268, -0.7853981634, 1.5707963268, 0.0],
  "joint_window_rad": 3.141592653589793,
  "max_joint_speed_radps": 1.0,
  "max_joint_accel_radps2": 2.0,
  "joints": [
    {"name": "Joint1", "axis": "y", "offset_m": [0.0, 0.20332999527454376, 0.0]},
    {"name": "Joint2", "axis": "z", "offset_m": [0.0, 0.012500002980232239, -0.08833000063896179]},
    {"name": "Joint3", "axis": "z", "offset_m": [0.0, 0.2800000011920929, 0.04602999985218048]},
    {"name": "Joint4", "axis": "z", "offset_m": [0.0, 0.25999999046325684, -0.07833000272512436]},
    {"name": "Joint5", "axis": "y", "offset_m": [0.0, 0.029999971389770508, 0.0]},
    {"name": "Joint6", "axis": "z", "offset_m": [0.0, 0.06832998991012573, -0.03793000429868698]}
  ],
  "tool": {
    "node": "robotgrabber",
    "root_offset_m": [0.0, 0.0, -0.030000001192092896],
    "rotation_xyzw": [0.70710688829422, 0.0, 0.0, 0.7071066498756409],
    "tcp_offset_m": [0.0, -0.09, 0.0],
    "forward_axis": [0.0, -1.0, 0.0]
  }
}
```

- [ ] **Step 4: Load and validate the shared model**

Resolve the JSON from the repository root, validate version, six joints, unique names, axes in `x/y/z`, finite 3-vectors/quaternion, and positive finite speed/window values. Keep `LM3Model()` construction argument-free for existing callers.

- [ ] **Step 5: Implement GLB FK and Jacobian**

For each joint apply local translation then local-axis rotation. Record joint origin and world axis before its rotation. After all joints apply tool root translation, fixed quaternion, and TCP translation. Build:

```python
jacobian[:3, index] = np.cross(axis_world, tcp_position - joint_origin)
jacobian[3:, index] = axis_world
```

- [ ] **Step 6: Run GREEN**

Run the same targeted pytest command. Expected: `1 passed`.

- [ ] **Step 7: Commit Task 1 files only**

Commit message: `feat: share GLB kinematics with simulator`.

### Task 2: Direct controller mapping in Three.js axes

**Files:**
- Modify: `backend/tests/control/test_coordinate_mapper.py`
- Modify: `backend/app/control/coordinate_mapper.py`

**Interfaces:**
- Preserves: `CoordinateMapper.capture(...)` and `.target(...)`.
- Produces: direct `+X/+Y/-Z` translation and controller-local `-Z` to tool-local `-Y` rotation calibration.

- [ ] **Step 1: Write one mapping behavior test**

Capture an identity controller at an arbitrary TCP. Assert a hand delta `(0.02, -0.04, -0.06)` produces exactly half that TCP delta at the default scale. Apply controller roll about local `-Z` and assert the resulting TCP relative rotation axis aligns with tool local `-Y` after the configured rotation scale/dead zone.

- [ ] **Step 2: Run RED**

Run:

```powershell
python -m pytest tests/control/test_coordinate_mapper.py -q
```

Expected: FAIL because the current mapper remaps translation into robot-base axes and uses the old controller-to-tool calibration.

- [ ] **Step 3: Implement direct translation and exact calibration**

Use identity translation mapping. Set:

```python
DEFAULT_R_TH = np.array([
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0],
])
```

Keep the local rotation formula and existing dead-zone/scale behavior. `head_q` stays accepted for protocol compatibility but no longer changes the translation basis.

- [ ] **Step 4: Run GREEN and commit**

Expected: the focused mapper file passes. Commit message: `fix: map Quest deltas in visual scene axes`.

### Task 3: Position-priority Cartesian servo

**Files:**
- Create: `backend/app/sim/cartesian_servo.py`
- Create: `backend/tests/sim/test_cartesian_servo.py`
- Modify: `backend/app/robots/sim_adapter.py`
- Modify: `backend/app/sim/virtual_robot.py` only if required to keep model-configured limits authoritative.

**Interfaces:**
- Produces: `CartesianServoResult(q, position_error_m, orientation_error_rad, joint_limited)`.
- Produces: `cartesian_servo_step(target, actual_q, model, dt=0.02) -> CartesianServoResult`.
- Preserves: `SimRobotAdapter.command_tcp(target, command_id)`.

- [ ] **Step 1: Write one servo smoke test**

From Home, create a target offset by a reachable `3 cm` translation and `8°` tool-local rotation. Repeatedly call `cartesian_servo_step` for at most `80` steps, feeding each returned `q` into the next step. Assert final position error is below `0.01 m`, final orientation error below `0.08 rad`, all joints remain inside the configured window, and no step exceeds the speed limit times `dt`.

- [ ] **Step 2: Run RED**

Run:

```powershell
python -m pytest tests/sim/test_cartesian_servo.py -q
```

Expected: FAIL because `app.sim.cartesian_servo` does not exist.

- [ ] **Step 3: Implement the minimal servo**

Use `geometric_jacobian`, DLS position pseudoinverse, position nullspace, and a second DLS solve for orientation. Use finite checks, norm clipping, joint-speed scaling, and joint-window clipping. Defaults:

```text
position gain 4.0
orientation gain 2.5
max Cartesian speed 0.20 m/s
max angular speed 0.8 rad/s
base damping 0.03
singular-value threshold 0.08
```

Return errors measured after the integrated step. Invalid numeric input raises `IKError("ik_singular")`; ordinary orientation limitation returns a valid result.

- [ ] **Step 4: Switch the simulator adapter to servo steps**

Inside the existing lock, call `cartesian_servo_step(target, self.robot.q, self.model, STEP_SECONDS)`, set the returned joint target, update command ID, and translate only numeric/hard-limit failures to `BackendCommandError`. Remove `_ik_seed_q` from the active TCP command path. Home remains a direct joint-space target.

- [ ] **Step 5: Run GREEN**

Expected: the focused servo smoke test passes.

- [ ] **Step 6: Commit Task 3 files only**

Commit message: `feat: servo TCP with position-priority IK`.

### Task 4: Shared frontend joint metadata and kinematic cube grasp

**Files:**
- Modify: `web/src/robot/robotModel.ts`
- Create: `web/src/scenes/kinematicGraspController.ts`
- Create: `web/tests/kinematicGraspController.test.ts`
- Modify: `web/src/scenes/simulationScene.ts`
- Modify: `web/vite.config.ts` only if the root shared JSON requires an explicit filesystem allow entry.

**Interfaces:**
- Produces: `RobotModel.tool: THREE.Object3D` for the configured `robotgrabber`.
- Produces: `KinematicGraspController.update(gripper: number): void` and `.reset(): void`.

- [ ] **Step 1: Write one frontend grasp test**

Build a `robotVisualRoot`, tool group, and cube. Place cube at the configured TCP point. Verify:

```text
gripper 0.70 -> cube parent becomes tool and local position equals TCP offset
move tool -> cube world position changes with tool
gripper 0.20 -> cube parent becomes visual root and cube local Y equals tabletop rest height
```

- [ ] **Step 2: Run RED**

Run from `web`:

```powershell
npm.cmd test -- kinematicGraspController.test.ts
```

Expected: FAIL because the controller module does not exist.

- [ ] **Step 3: Consume shared metadata in `robotModel.ts`**

Import the root JSON using `resolveJsonModule`, derive joint node/axis entries, require the configured tool node, and expose it on `RobotModel`. Preserve finite/six-joint checks and non-accumulating angle application.

- [ ] **Step 4: Implement deterministic grasp behavior**

Use `0.07 m` capture distance, `0.65` close threshold, `0.35` release threshold, configured TCP local offset, and tabletop cube-center Y. On attach, parent to tool and snap to TCP. On release, use `visualRoot.attach(cube)`, restore tabletop Y, and clamp X/Z to the table footprint.

- [ ] **Step 5: Add the cube to `SimulationScene`**

Create one `0.06 m` cube under `robotVisualRoot` at a reachable table position. After applying sampled joints and gripper, update matrices and call the grasp controller with the authoritative sampled gripper. Reset the cube when a new scene is constructed; no protocol fields are added.

- [ ] **Step 6: Run GREEN and build**

Run the focused Vitest file, then:

```powershell
npm.cmd run build
```

Expected: focused test passes and Vite build exits `0`.

- [ ] **Step 7: Commit Task 4 files only**

Commit message: `feat: add kinematic cube grasp interaction`.

### Task 5: Documentation, focused verification, and experiential handoff

**Files:**
- Modify: `README.md`
- Reference: `docs/superpowers/specs/2026-07-22-shared-glb-cartesian-servo-and-cube-grasp-design.md`

- [ ] **Step 1: Update operator instructions**

Explain that simulator FK/IK/rendering now share the GLB chain, position has priority over orientation, Grip re-anchors from actual TCP, and a nearby cube attaches when Trigger closes and releases to the tabletop when opened. Keep real hardware explicitly disabled.

- [ ] **Step 2: Run only the agreed verification set**

Backend:

```powershell
python -m pytest tests/sim/test_kinematics.py tests/control/test_coordinate_mapper.py tests/sim/test_cartesian_servo.py -q
```

Frontend:

```powershell
npm.cmd test -- kinematicGraspController.test.ts
npm.cmd run build
```

- [ ] **Step 3: Inspect final diff**

Confirm no real-robot enablement, dependency changes, physics engine, protocol change, unrelated root-checkout file, or large test expansion.

- [ ] **Step 4: Commit documentation**

Commit message: `docs: explain GLB servo and cube grasp`.

- [ ] **Step 5: Restart and hand off**

Restart FastAPI on `127.0.0.1:8000` and Vite HTTPS on `0.0.0.0:5173`. Report exact targeted-test/build results and give the user this Quest check:

1. Grip lock produces no jump;
2. forward/back/right/left/up/down visual TCP directions match the hand;
3. small roll/pitch/yaw follows while translation remains responsive;
4. close Trigger within the cube threshold attaches it;
5. move the arm with the cube and open Trigger to place it back on the table.

## Self-Review

- Spec coverage: shared kinematics, direct mapping, position-priority servo, cube attach/release, minimal tests, and real-hardware boundary each have a task.
- Placeholder scan: all paths, thresholds, matrices, commands, and commit messages are explicit.
- Type consistency: backend continues to emit protocol-v1 `Pose`/joint vectors; frontend only adds `RobotModel.tool` and a scene-local grasp controller.
- Scope: cube remains frontend-only and the control kernel remains simulator-only, so no camera/recording or real-robot work is pulled into this phase.
