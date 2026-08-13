# High-Response Simulation Profile Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver a `0.30 m/s`, 50 Hz simulation profile with an authoritative adjustable `0.5–10.0` translation scale, accurate soft-constraint labels, and independent PC right-drag orbit viewing without changing real-robot limits.

**Architecture:** Keep `LEBAI_FAKE` on the production-shaped CoordinateMapper → SafetyLimiter → IK → PVAT path, but give Fake its own responsive YAML values. Add a correlated WebSocket scale-setting protocol owned by `RobotControl`; the browser presents a stopped-only control and treats the backend reply as authority. Extract PC orbit math/input from robot input so right-click can never generate Grip, while WebXR remains unchanged.

**Tech Stack:** Python 3.11+, FastAPI, Pydantic v2, pytest, TypeScript, Three.js, Vite, Vitest/jsdom.

## Global Constraints

- User-facing runtime names remain only **仿真** and **真机**; do not introduce “数字孪生”.
- `LEBAI_FAKE` uses `50 Hz` control/state/PVAT, `0.04 s` PVAT horizon, `0.30 m/s`, `1.2 m/s²`, `1.5 rad/s`, `4.0 rad/s²`, `0.006 m`, `2.0°`, `1.5 rad/s`, `4.0 rad/s²`, `0.06 rad`, and default translation scale `1.5`.
- Simulation scale range is `0.5–10.0`, step `0.1`, default `1.5`; it applies only to the next Grip anchor after an authoritative stopped-state acknowledgement.
- `config/real-robot.example.yaml` remains byte-for-byte unchanged and retains `0.03 m/s` and translation scale `0.5`.
- Real joint soft limits, self-collision, Grip stop, stale/disconnect stop, single controller ownership, and hard-fault locking remain enabled.
- No browser runtime switch from simulation to real hardware.
- No Quest camera, `immersive-ar`, native Quest application, camera capture, or dataset writing in this plan.
- Every production change follows TDD red → green and every task ends with a local commit only; do not push.

---

## File Structure

- `config/fake-lebai.yaml`: authoritative high-response Fake defaults.
- `backend/app/control/coordinate_mapper.py`: stopped-only mutable simulation translation scale.
- `backend/app/control/robot_control.py`: scale eligibility, mutation, result data, and critical audit event.
- `backend/app/schemas/messages.py`: strict client scale request and server result schemas.
- `backend/app/api/teleop_ws.py`: owner-socket scale request routing and correlated response.
- `backend/app/robots/lebai_adapter.py`: precise soft-constraint classification.
- `web/src/protocol/messages.ts`: TypeScript request/result contract and runtime guards.
- `web/src/transport/teleopSocket.ts`: scale request transport and result callback.
- `web/src/ui/simulationSettingsPanel.ts`: focused scale/view UI, local persistence, stopped-state gating.
- `web/src/scenes/orbitCameraController.ts`: PC-only orbit state and pointer lifecycle.
- `web/src/scenes/simulationScene.ts`: integrate orbit controller and expose `resetView()`.
- `web/src/ui/hud.ts`, `web/src/main.ts`, `web/src/styles.css`: mount controls and wire authoritative state.

---

### Task 1: High-Response Fake Profile and Real-Template Isolation

**Files:**
- Modify: `config/fake-lebai.yaml`
- Test: `backend/tests/digital_twin/test_runtime.py`
- Test: `backend/tests/config/test_real_robot_config.py`
- Test: `backend/tests/control/test_safety.py`

**Interfaces:**
- Consumes: `load_digital_twin_settings()` and `_build_limiter(settings, runtime_backend)`.
- Produces: a Fake `LebaiControlSettings` with the exact global-constraint values; real example values remain unchanged.

- [ ] **Step 1: Add failing exact-profile and isolation tests**

Add assertions in `test_runtime.py` for:

```python
assert control.loop_hz == 50
assert control.state_hz == 50
assert control.pvat_send_hz == 50
assert control.pvat_horizon_s == pytest.approx(0.04)
assert control.max_tcp_speed_mps == pytest.approx(0.30)
assert control.max_tcp_acceleration_mps2 == pytest.approx(1.2)
assert control.max_tcp_rotation_radps == pytest.approx(1.5)
assert control.max_tcp_angular_acceleration_radps2 == pytest.approx(4.0)
assert control.max_tcp_step_m == pytest.approx(0.006)
assert control.max_tcp_rotation_step_deg == pytest.approx(2.0)
assert control.max_joint_speed_radps == pytest.approx(1.5)
assert control.max_joint_acceleration_radps2 == pytest.approx(4.0)
assert control.max_joint_step_rad == pytest.approx(0.06)
assert control.translation_scale == pytest.approx(1.5)
```

In `test_real_robot_config.py`, load `config/real-robot.example.yaml` as YAML and assert its control block still contains `max_tcp_speed_mps == 0.03` and `translation_scale == 0.5`.

- [ ] **Step 2: Run RED**

Run:

```powershell
Set-Location backend
python -m pytest tests/digital_twin/test_runtime.py tests/config/test_real_robot_config.py -q
```

Expected: Fake exact-profile assertions fail with the current `25 Hz / 0.06 m/s / 0.7` values; real-template assertions pass.

- [ ] **Step 3: Update only the Fake YAML**

Set the exact values from Global Constraints in `config/fake-lebai.yaml`. Do not modify `config/real-robot.example.yaml` or `backend/tests/robots/real_settings.py`.

- [ ] **Step 4: Run GREEN plus limiter checks**

Run:

```powershell
python -m pytest tests/digital_twin/test_runtime.py tests/config/test_real_robot_config.py tests/control/test_safety.py -q
```

Expected: all pass; Fake still uses `workspace_boundary_mode == "axis_clamp"`, real uses `"hold"`.

- [ ] **Step 5: Commit**

```powershell
git add -- config/fake-lebai.yaml backend/tests/digital_twin/test_runtime.py backend/tests/config/test_real_robot_config.py backend/tests/control/test_safety.py
git commit -m "feat: add high-response simulation profile"
```

---

### Task 2: Precise Motion-Continuity Constraint Classification

**Files:**
- Modify: `backend/app/schemas/messages.py`
- Modify: `backend/app/robots/lebai_adapter.py`
- Modify: `backend/app/control/robot_control.py`
- Test: `backend/tests/robots/test_lebai_adapter_control.py`
- Test: `backend/tests/control/test_robot_control.py`
- Modify: `web/src/protocol/messages.ts`
- Modify: `web/src/ui/hud.ts`
- Modify: `web/src/scenes/vrSafetyPanel.ts`
- Test: `web/tests/messages.test.ts`
- Test: `web/tests/hud.test.ts`
- Test: `web/tests/vrSafetyPanel.test.ts`

**Interfaces:**
- Produces: `ConstraintKind` adds literal `motion_continuity_boundary` in Python and TypeScript.
- Mapping: `ik_joint_limit → joint_boundary`; `ik_unreachable|ik_invalid → ik_boundary`; `ik_joint_jump|joint_speed_limit → motion_continuity_boundary`; `self_collision → self_collision`.

- [ ] **Step 1: Write failing backend classification tests**

Replace the parameterized expectation that maps all three joint-related errors to `joint_boundary` with:

```python
@pytest.mark.parametrize(
    ("error", "expected"),
    [
        ("ik_joint_limit", "joint_boundary"),
        ("ik_joint_jump", "motion_continuity_boundary"),
        ("joint_speed_limit", "motion_continuity_boundary"),
    ],
)
```

Add adapter tests confirming `_note_soft_constraint()` exposes the same classification.

- [ ] **Step 2: Run backend RED**

Run:

```powershell
Set-Location backend
python -m pytest tests/robots/test_lebai_adapter_control.py tests/control/test_robot_control.py -q
```

Expected: failures because the new literal is absent and continuity errors currently map to `joint_boundary`.

- [ ] **Step 3: Implement backend literal and classification**

Add `motion_continuity_boundary` to `ConstraintKind`. Centralize the reason-to-constraint mapping in `lebai_adapter.py`:

```python
def _constraint_for_error(reason: str) -> ConstraintKind:
    if reason in {"ik_unreachable", "ik_invalid"}:
        return "ik_boundary"
    if reason in {"ik_joint_jump", "joint_speed_limit"}:
        return "motion_continuity_boundary"
    return "joint_boundary"
```

Use the same grouping in `RobotControl.tick()` when adapter errors propagate synchronously.

- [ ] **Step 4: Run backend GREEN**

Run the Step 2 command. Expected: all pass.

- [ ] **Step 5: Write frontend RED guards and copy tests**

Add `motion_continuity_boundary` to fixture cases and expect:

```text
运动变化过快，保持 Grip 放慢或反向退回
```

Use the same copy in PC HUD and VR safety panel.

- [ ] **Step 6: Run frontend RED**

```powershell
Set-Location ..\web
npm.cmd test -- --run tests/messages.test.ts tests/hud.test.ts tests/vrSafetyPanel.test.ts
```

Expected: runtime guard and readable-copy failures.

- [ ] **Step 7: Implement frontend literal, guards, and copy**

Update `ConstraintKind`, `CONSTRAINT_KINDS`, `readableConstraint()`, and `constraintInstruction()`.

- [ ] **Step 8: Run frontend GREEN and commit**

```powershell
npm.cmd test -- --run tests/messages.test.ts tests/hud.test.ts tests/vrSafetyPanel.test.ts
Set-Location ..
git add -- backend/app/schemas/messages.py backend/app/robots/lebai_adapter.py backend/app/control/robot_control.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/control/test_robot_control.py web/src/protocol/messages.ts web/src/ui/hud.ts web/src/scenes/vrSafetyPanel.ts web/tests/messages.test.ts web/tests/hud.test.ts web/tests/vrSafetyPanel.test.ts
git commit -m "fix: distinguish simulation continuity limits"
```

---

### Task 3: Authoritative Simulation-Scale Backend Protocol

**Files:**
- Modify: `backend/app/control/coordinate_mapper.py`
- Modify: `backend/app/control/robot_control.py`
- Modify: `backend/app/schemas/messages.py`
- Modify: `backend/app/api/teleop_ws.py`
- Test: `backend/tests/control/test_coordinate_mapper.py`
- Test: `backend/tests/control/test_robot_control.py`
- Test: `backend/tests/contract/test_messages.py`
- Test: `backend/tests/api/test_ws_lifecycle.py`

**Interfaces:**
- Client request:

```json
{"v":1,"type":"set_simulation_scale","request_id":"scale-1","translation_scale":1.5}
```

- Server result:

```json
{"v":1,"type":"simulation_scale_result","request_id":"scale-1","accepted":true,"translation_scale":1.5}
```

or rejected with `reason` in `not_simulation|not_stopped|invalid_scale|automation_active` plus `translation_scale` containing the authoritative current value.
- Produces: `RobotControl.set_simulation_scale(value: float, *, automation_active: bool) -> SimulationScaleResult`.

- [ ] **Step 1: Write strict schema RED tests**

Test acceptance of exact finite tenth-step values `0.5`, `1.5`, `2.0`; reject `0.49`, `2.01`, `1.55`, NaN, booleans, unknown keys, missing request ID, and real-runtime mutation attempts.

- [ ] **Step 2: Run schema RED**

```powershell
Set-Location backend
python -m pytest tests/contract/test_messages.py -q
```

Expected: request/result models are missing.

- [ ] **Step 3: Add Pydantic request/result models**

Use an integer-tenths validator rather than binary floating equality:

```python
scaled = round(value * 10)
if not math.isclose(value * 10, scaled, abs_tol=1e-9) or not 5 <= scaled <= 100:
    raise ValueError("translation_scale must be 0.5..10.0 in 0.1 steps")
```

Add the request model to `ClientMessage` before `ClientControlMessage` dispatch.

- [ ] **Step 4: Write mapper/control RED tests**

Assert:

- mapper scale updates while stopped and affects the next `capture()`/`target()`;
- an existing anchor cannot change scale mid-Grip;
- Fake stopped modes `READY|HOLD|DISARMED` accept;
- `ARMED|ACTIVE|STALE|FAULT|DISCONNECTED`, automation active, and `backend != LEBAI_FAKE` reject without mutation;
- accepted mutation writes one critical `simulation_scale_changed` event with previous/new values.

- [ ] **Step 5: Run mapper/control RED**

```powershell
python -m pytest tests/control/test_coordinate_mapper.py tests/control/test_robot_control.py -q
```

Expected: mutable scale API and result are absent.

- [ ] **Step 6: Implement stopped-only authoritative mutation**

Expose `CoordinateMapper.translation_scale` as a read-only property and `set_translation_scale()` that raises while anchors are captured. In `RobotControl`, check runtime identity from `backend.get_state().backend`, current mode, mapper anchor state, and automation flag before mutation; record only accepted changes.

- [ ] **Step 7: Write WebSocket RED tests**

Using the existing owner socket fixture, assert exact correlated result, no socket close on rejection, scale state survives ordinary VRFrames, and the same request against a real-labeled backend returns `not_simulation`.

- [ ] **Step 8: Implement WebSocket routing**

Route the strict request before generic `message.type` handling. Use `app.state.offline_rehearsal_store.has_active_owner(owner_token)` (or an equivalent read-only store method added with tests) for the automation gate. Send only `simulation_scale_result`; never forward this message into `LatestVRFrame` or motion.

- [ ] **Step 9: Run GREEN and commit**

```powershell
python -m pytest tests/contract/test_messages.py tests/control/test_coordinate_mapper.py tests/control/test_robot_control.py tests/api/test_ws_lifecycle.py -q
Set-Location ..
git add -- backend/app/control/coordinate_mapper.py backend/app/control/robot_control.py backend/app/schemas/messages.py backend/app/api/teleop_ws.py backend/tests/control/test_coordinate_mapper.py backend/tests/control/test_robot_control.py backend/tests/contract/test_messages.py backend/tests/api/test_ws_lifecycle.py
git commit -m "feat: add authoritative simulation scale control"
```

---

### Task 4: Frontend Scale Transport, Persistence, and UI

**Files:**
- Create: `web/src/ui/simulationSettingsPanel.ts`
- Modify: `web/src/protocol/messages.ts`
- Modify: `web/src/transport/teleopSocket.ts`
- Modify: `web/src/ui/hud.ts`
- Modify: `web/src/main.ts`
- Modify: `web/src/styles.css`
- Test: `web/tests/messages.test.ts`
- Test: `web/tests/teleopSocket.test.ts`
- Create: `web/tests/simulationSettingsPanel.test.ts`
- Modify: `web/tests/hud.test.ts`

**Interfaces:**
- Produces `SimulationSettingsPanel.update({runtime, connected, mode, grip, automationActive, authoritativeScale})`.
- Produces callback `onScaleRequest(value: number): void` and `handleResult(message: SimulationScaleResultMessage): void`.
- Storage key: `vr4arm.simulation.translationScale`; persisted values are hints until accepted by backend.

- [ ] **Step 1: Write protocol/transport RED tests**

Add exact guards for request/result, reject malformed/out-of-range results, assert `TeleopSocket.sendSimulationScale()` serializes a valid request, and assert the callback receives a valid result without closing/reconnecting.

- [ ] **Step 2: Run protocol RED**

```powershell
Set-Location web
npm.cmd test -- --run tests/messages.test.ts tests/teleopSocket.test.ts
```

Expected: types, guards, send method, and callback are absent.

- [ ] **Step 3: Implement transport types and guards**

Add dedicated `SimulationScaleRequestMessage` and `SimulationScaleResultMessage`; do not widen `ClientControlMessage`. Extend the transport `send()` union and constructor callback at the end to preserve existing call sites.

- [ ] **Step 4: Write UI RED tests**

Build jsdom tests for:

- label `平移倍率` and display `1.5 : 1`;
- range `min=0.5 max=10 step=0.1` and decrement/increment buttons;
- disabled while disconnected, non-Fake, `ACTIVE|ARMED|STALE|FAULT`, Grip pressed, automation active, or request pending;
- real runtime shows YAML scale read-only;
- persisted `1.8` is requested only after Fake connected/stopped authority arrives;
- rejected result restores authoritative value and does not overwrite storage;
- accepted result updates display and storage.

- [ ] **Step 5: Run UI RED**

```powershell
npm.cmd test -- --run tests/simulationSettingsPanel.test.ts tests/hud.test.ts
```

Expected: panel/container APIs are absent.

- [ ] **Step 6: Implement the focused panel**

Create a small class owning DOM, request-pending state, localStorage parsing, and button/input events. It must never infer acceptance from socket send; only `simulation_scale_result.accepted === true` changes authoritative display/storage.

- [ ] **Step 7: Wire panel in main/HUD**

Add `hud.simulationSettingsContainer`. Construct the panel after `ArmPanel`, pass results from `TeleopSocket`, and update it from `onRobotState`, runtime identity, controller Grip, connection, and rehearsal snapshot. Generate request IDs as `desktop-scale-${++sequence}`.

- [ ] **Step 8: Run GREEN and commit**

```powershell
npm.cmd test -- --run tests/messages.test.ts tests/teleopSocket.test.ts tests/simulationSettingsPanel.test.ts tests/hud.test.ts
Set-Location ..
git add -- web/src/ui/simulationSettingsPanel.ts web/src/protocol/messages.ts web/src/transport/teleopSocket.ts web/src/ui/hud.ts web/src/main.ts web/src/styles.css web/tests/messages.test.ts web/tests/teleopSocket.test.ts web/tests/simulationSettingsPanel.test.ts web/tests/hud.test.ts
git commit -m "feat: add adjustable simulation translation scale"
```

---

### Task 5: Independent PC Right-Drag Orbit Camera

**Files:**
- Create: `web/src/scenes/orbitCameraController.ts`
- Modify: `web/src/scenes/simulationScene.ts`
- Modify: `web/src/ui/simulationSettingsPanel.ts`
- Test: `web/tests/orbitCameraController.test.ts`
- Modify: `web/tests/simulationScene.test.ts`
- Modify: `web/tests/simulationSettingsPanel.test.ts`

**Interfaces:**
- Produces `OrbitCameraController.attach()`, `dispose()`, `reset()`, `apply(camera)`, and `snapshot`.
- Constructor consumes canvas, camera, fixed target `[0.03, 0.42, 0]`, default position `[1.55, 1.06, 1.9]`.
- `SimulationScene.resetView(): void` delegates to orbit controller.

- [ ] **Step 1: Write orbit-controller RED tests**

Test right-button pointer capture, yaw/pitch update with pitch clamp, radius preservation, context-menu prevention, pointerup/cancel/lost capture/blur cleanup, `reset()` exact default pose, and disposal listener removal.

- [ ] **Step 2: Run orbit RED**

```powershell
Set-Location web
npm.cmd test -- --run tests/orbitCameraController.test.ts
```

Expected: module missing.

- [ ] **Step 3: Implement orbit controller**

Use spherical coordinates derived from the current default camera vector. Clamp pitch to `[-75°, 75°]`; ignore left/middle buttons; use right-button pointer ID only. `apply()` sets camera position and `lookAt(target)`.

- [ ] **Step 4: Write scene integration RED tests**

Assert a right-button drag changes camera but leaves `DesktopInputSafety.snapshot()` as `{activePointer:null, grip:false, trigger:0}` and does not change `controllerPosition`. Assert left drag still changes the synthetic controller. Assert automation blocks robot input but does not block right-camera orbit. Assert `resetView()` restores default.

- [ ] **Step 5: Integrate scene and UI reset**

Attach orbit lifecycle alongside existing scene listeners. Do not reuse `DesktopInputSafety` pointer capture. Add a `复位视角` button to `SimulationSettingsPanel`, whose callback calls `scene.resetView()`; it remains enabled in simulation regardless of arm mode, except disconnected scene teardown.

- [ ] **Step 6: Run GREEN and commit**

```powershell
npm.cmd test -- --run tests/orbitCameraController.test.ts tests/simulationScene.test.ts tests/simulationSettingsPanel.test.ts
Set-Location ..
git add -- web/src/scenes/orbitCameraController.ts web/src/scenes/simulationScene.ts web/src/ui/simulationSettingsPanel.ts web/tests/orbitCameraController.test.ts web/tests/simulationScene.test.ts web/tests/simulationSettingsPanel.test.ts
git commit -m "feat: add PC orbit camera controls"
```

---

### Task 6: End-to-End Regression, Documentation, and Restart

**Files:**
- Modify: `README.md`
- Modify: `docs/quest-development.md`
- Modify: `docs/real-robot-deployment.md`
- Modify: `backend/tests/digital_twin/test_responsive_teleop.py`
- Test: existing backend/frontend suites

**Interfaces:**
- Consumes all prior task interfaces.
- Produces one locally committed, running simulation stack for manual acceptance.

- [ ] **Step 1: Strengthen the one-socket integration test**

Extend `test_responsive_teleop.py` so it:

1. stays connected for at least 15 seconds;
2. confirms one `hello_ack`, no reconnect/owner change;
3. reaches sustained ACTIVE motion without hard fault;
4. releases Grip, receives HOLD/IDLE, sets scale to `1.8`, gets accepted authoritative response;
5. re-Grips and resumes motion;
6. deliberately reaches workspace and motion-continuity soft constraints, retreats, and confirms automatic clearing;
7. verifies real-template values remain untouched.

- [ ] **Step 2: Run targeted integration RED/GREEN gate**

```powershell
Set-Location backend
python -m pytest tests/digital_twin/test_responsive_teleop.py tests/digital_twin/test_runtime.py tests/api/test_ws_lifecycle.py -q
```

Expected after Tasks 1–5: pass with one socket and no hard fault.

- [ ] **Step 3: Update operator documentation**

Document:

- `0.30 m/s` is simulation-only;
- scale defaults to `1.5`, simulation range `0.5–10.0`, step `0.1`, changed only stopped/Grip released; real-robot scale remains configuration-only;
- right-drag orbits, left-drag controls, wheel controls depth, and `复位视角`;
- real IP comes from `real-robot.local.yaml`; SDK currently accepts IP only, while ports `8000/5173` belong to FastAPI/web;
- current real view is a joint-state-driven 3D model; direct observation is mandatory for first tests; MR is a later stage.

- [ ] **Step 4: Run full backend verification**

```powershell
Set-Location backend
python -m pytest -q
```

Expected: zero failures.

- [ ] **Step 5: Run full frontend verification and build**

```powershell
Set-Location ..\web
npm.cmd test
npm.cmd run build
```

Expected: zero test failures; build exit `0`; existing Vite chunk-size advisory may remain non-blocking.

- [ ] **Step 6: Review diff and commit docs/integration**

```powershell
Set-Location ..
git diff --check
git status --short
git add -- README.md docs/quest-development.md docs/real-robot-deployment.md backend/tests/digital_twin/test_responsive_teleop.py
git commit -m "docs: document responsive simulation controls"
```

- [ ] **Step 7: Restart the simulation program**

Stop the existing `scripts/run_offline_rehearsal.py` process cleanly with Ctrl+C and wait until ports `8000` and `5173` are closed. Start:

```powershell
python scripts/run_offline_rehearsal.py
```

Verify `http://127.0.0.1:8000/health` returns `backend=LEBAI_FAKE`, `preflight_ready=true`, `hardware_verified=false`, and open `https://127.0.0.1:5173/` for manual acceptance. Do not claim real-hardware verification.

---

## Plan Self-Review

- Spec coverage: exact Fake performance profile (Task 1), constraint semantics (Task 2), stopped-only authoritative scale and audit (Tasks 3–4), local persistence and real read-only behavior (Task 4), right-drag orbit/reset and WebXR isolation (Task 5), one-socket acceptance/docs/restart (Task 6).
- Type consistency: `set_simulation_scale` → `simulation_scale_result`; `motion_continuity_boundary` is identical in Python and TypeScript; `SimulationSettingsPanel` consumes authoritative backend results only.
- Scope: MR/passthrough, external cameras, dataset writing, and real runtime switching remain explicitly excluded.
- Placeholder scan: no TBD/TODO or unspecified implementation steps remain.
