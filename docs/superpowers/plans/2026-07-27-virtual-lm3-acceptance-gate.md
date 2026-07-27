# Virtual LM3 Acceptance Gate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore a truthful all-green non-hardware baseline and add a deterministic software-in-the-loop acceptance gate for the LM3 + stock LMG-90 teleoperation stack.

**Architecture:** Keep `RobotControl` and `RobotBackend` as the shared control boundary. Validate the simulator with its actual differential Cartesian servo, validate the real adapter through `FakeLebaiClient`, compare the versioned simulation profile with an optional onsite profile, and aggregate backend, frontend, build, soak, grasp, and profile results into one machine-readable report that always declares hardware verification separately.

**Tech Stack:** Python 3.11+, FastAPI application modules, asyncio, pytest, NumPy, SciPy, TypeScript, Vitest, Three.js, Vite, YAML/JSON, Git.

## Global Constraints

- Never import, connect to, or command the real Lebai SDK while executing this plan.
- Never set `VR4ARM_REAL_ROBOT_CONFIRM` or change a real configuration from `readonly` to `control`.
- Do not add Cannon-es, Rapier, Ammo, MuJoCo, Isaac Sim, or another physics dependency.
- Keep protocol version `1`; this plan does not add a WebSocket message type.
- Treat `ik_unreachable`, `ik_singular`, `joint_safety_window`, and `self_collision` as retreatable soft constraints.
- Use a distinct `backend_command_failed` error to test hard command-fault behavior.
- A virtual acceptance report must always contain `hardware_verified: false`.
- Do not overwrite or include the user's existing uncommitted `README.md`, `backend/app/sim/ik.py`, or the two 2026-07-22 plan/spec drafts in any task commit.
- Make local commits only. Do not push.
- Do not claim a test passes without running its exact command and reading its output.

## File Structure

### Baseline repair

- `backend/tests/api/test_fault_paths.py`: exercise the current Cartesian-servo seam and current soft-constraint semantics.
- `backend/tests/control/test_robot_control.py`: use a true hard-error code in hard-fault lifecycle tests.
- `backend/tests/control/test_safety.py`: make the intended 0.25 m test radius explicit.
- `backend/tests/sim/test_adapters.py`: inject errors through `cartesian_servo_step`, the seam used by `SimRobotAdapter`.
- `backend/tests/sim/test_ik.py`: validate the current-model reachability fixture and its provenance.
- `backend/tests/test_soak.py`: assert corrected deterministic soak invariants.
- `scripts/generate_reachability_fixture.py`: generate a current-model fixture from any working directory.
- `schemas/fixtures/sim-reachability-v2.json`: replace the stale MDH-era fixture with current GLB-chain targets and metadata.
- `web/tests/controllerInput.test.ts`: include the current left-controller fields.
- `web/tests/xrSession.test.ts`: include the current XR presentation fields.

### Acceptance implementation

- `backend/app/acceptance/__init__.py`: public acceptance package exports.
- `backend/app/acceptance/profile.py`: normalize simulation and optional onsite settings, compare them, and serialize differences.
- `backend/app/acceptance/scenarios.py`: deterministic mapping, servo-tracking, soft-constraint, stop, and Home scenarios.
- `backend/tests/acceptance/test_profile.py`: profile normalization and difference tests.
- `backend/tests/acceptance/test_scenarios.py`: deterministic scenario result and safety-invariant tests.
- `backend/tests/robots/test_backend_contract.py`: common `RobotBackend` behavior against simulator and fake Lebai implementations.
- `web/tests/virtualAcceptance.test.ts`: authoritative TCP/gripper grasp, release, and stack acceptance flow.
- `scripts/accept_virtual_lm3.py`: run the full non-hardware gate and write a JSON report.
- `backend/tests/scripts/test_accept_virtual_lm3.py`: command orchestration, failure propagation, and report-schema tests.
- `.gitignore`: ignore generated `artifacts/acceptance/` reports.
- `docs/virtual-lm3-acceptance.md`: operator-facing command, report, and hardware-boundary documentation.

---

### Task 1: Restore the truthful backend baseline

**Files:**
- Modify: `backend/tests/api/test_fault_paths.py`
- Modify: `backend/tests/control/test_robot_control.py`
- Modify: `backend/tests/control/test_safety.py`
- Modify: `backend/tests/sim/test_adapters.py`
- Modify: `backend/tests/sim/test_ik.py`
- Modify: `backend/tests/test_soak.py`
- Modify: `scripts/soak_simulator.py`
- Modify: `scripts/generate_reachability_fixture.py`
- Create: `schemas/fixtures/sim-reachability-v2.json`
- Delete: `schemas/fixtures/sim-reachability-v1.json`

**Interfaces:**
- Consumes: `SimRobotAdapter.command_tcp()`, `cartesian_servo_step()`, `RobotControl` soft-constraint behavior, `LM3Model`, `forward_pose()`.
- Produces: a zero-failure backend baseline and a provenance-bound current-model reachability fixture used by later full-gate runs.

- [ ] **Step 1: Change adapter tests to inject through the live servo seam**

In `backend/tests/sim/test_adapters.py`, replace the obsolete `solve_ik` monkeypatch with:

```python
def fail_servo(*args, **kwargs):
    raise IKError(error_code)

monkeypatch.setattr(
    sim_adapter_module,
    "cartesian_servo_step",
    fail_servo,
)
```

Keep the assertions that `target_q`, `target_qd`, and `command_id` are not mutated on failure.

- [ ] **Step 2: Run the adapter cases and verify the old tests now reach production behavior**

Run:

```powershell
cd backend
python -m pytest tests/sim/test_adapters.py -q
```

Expected: the three previously broken monkeypatch cases execute; any remaining failure is an assertion about current behavior, not `AttributeError: solve_ik`.

- [ ] **Step 3: Separate soft IK constraints from hard backend command faults**

In `backend/tests/api/test_fault_paths.py`, rename the stale fault test to:

```python
async def test_simulator_ik_error_publishes_soft_constraint_without_disarming(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
```

Patch `app.robots.sim_adapter.cartesian_servo_step` to raise `IKError("ik_unreachable")`, then assert:

```python
assert control.mode is TeleopMode.ACTIVE
assert adapter.stop_reasons.count(StopReason.FAULT) == 0
published = await control.state_message()
assert published.constraint == "ik_boundary"
assert published.fault is None
```

In the four hard-fault lifecycle tests in `backend/tests/control/test_robot_control.py`, change only the injected error and matching expected fault string:

```python
BackendCommandError("backend_command_failed")
```

This preserves coverage of unpublished/published hard-fault priority without redefining `ik_unreachable` as a hard fault.

- [ ] **Step 4: Make the old safety-radius intent explicit**

Change `test_rejects_anchor_envelope_violation()` to instantiate:

```python
limiter = SafetyLimiter(anchor=(0, 0, 0), workspace_radius=0.25)
```

Keep the 0.26 m request and `workspace_violation` assertion. Do not change the production default of 0.45 m.

- [ ] **Step 5: Fix hard-fault injection and cycle accounting in the soak**

In `scripts/soak_simulator.py`, inject:

```python
raise BackendCommandError("backend_command_failed")
```

for `fail_next_command`.

In `_recover()`, execute exactly one `_tick_control()` on every path. If explicit reset is rejected, append the invariant failure and still tick before returning. Task 4 adds a separate `ik_boundary` soft-constraint event after the original six-event baseline is restored.

Expected event dictionary:

```python
{
    "tracking_loss": 1,
    "hidden": 1,
    "disconnect": 1,
    "command_fault": 1,
    "safety_fault": 1,
    "visible_blurred": 1,
}
```

Update `backend/tests/test_soak.py` to require all six original events while keeping:

```python
assert summary["control_steps"] == 30_000
assert summary["virtual_steps"] == 30_000
assert summary["nan_count"] == 0
assert summary["max_queue_depth"] == 1
assert summary["error_count"] == 0
assert summary["final_mode"] == "DISARMED"
```

- [ ] **Step 6: Write failing provenance tests for the current reachability fixture**

Change `backend/tests/sim/test_ik.py` to load `sim-reachability-v2.json` as:

```python
fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
assert fixture["version"] == 2
assert fixture["generator_seed"] == 42
assert fixture["sample_count"] == 1000
assert fixture["model_sha256"] == hashlib.sha256(
    MODEL_CONFIG.read_bytes()
).hexdigest()
rows = fixture["rows"]
```

Run:

```powershell
cd backend
python -m pytest tests/sim/test_ik.py::test_ik_solves_at_least_99_percent_of_reachability_fixture -q
```

Expected: FAIL because `sim-reachability-v2.json` does not exist.

- [ ] **Step 7: Make fixture generation deterministic and repository-relative**

Refactor `scripts/generate_reachability_fixture.py` to expose:

```python
ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / "backend"
OUTPUT = ROOT / "schemas" / "fixtures" / "sim-reachability-v2.json"

if str(BACKEND) not in sys.path:
    sys.path.insert(0, str(BACKEND))

def build_fixture(*, seed: int = 42, sample_count: int = 1000) -> dict[str, object]:
    rng = np.random.default_rng(seed)
    model = LM3Model()
    home = np.asarray(model.home_q)
    rows: list[dict[str, object]] = []
    for _ in range(sample_count):
        q = home + rng.uniform(-0.35, 0.35, 6)
        nearby_seed = q + rng.uniform(-0.05, 0.05, 6)
        rows.append(
            {
                "target": forward_pose(q, model).model_dump(mode="json"),
                "seed": nearby_seed.tolist(),
            }
        )
    return {
        "version": 2,
        "generator_seed": seed,
        "sample_count": sample_count,
        "model_sha256": hashlib.sha256(MODEL_CONFIG.read_bytes()).hexdigest(),
        "rows": rows,
    }

def main() -> int:
    OUTPUT.write_text(
        json.dumps(build_fixture(), separators=(",", ":")),
        encoding="utf-8",
    )
    return 0
```

The payload must be:

```python
{
    "version": 2,
    "generator_seed": seed,
    "sample_count": sample_count,
    "model_sha256": hashlib.sha256(MODEL_CONFIG.read_bytes()).hexdigest(),
    "rows": rows,
}
```

Generate each target from `home_q + uniform(-0.35, 0.35)` and each seed from target joints plus `uniform(-0.05, 0.05)`, both with `numpy.random.default_rng(seed)`.

- [ ] **Step 8: Generate and verify the current-model fixture**

Run from the repository root:

```powershell
python scripts/generate_reachability_fixture.py
cd backend
python -m pytest tests/sim/test_ik.py -q
```

Expected: 1,000 rows, at least 990 solved; the observed pre-plan audit on the current model was 1,000/1,000.

- [ ] **Step 9: Run the repaired backend suite**

Run:

```powershell
cd backend
python -m pytest -q
```

Expected: zero failures. Record the actual pass count instead of assuming the previous count remains 324.

- [ ] **Step 10: Commit only the baseline repair**

```powershell
git add backend/tests/api/test_fault_paths.py backend/tests/control/test_robot_control.py backend/tests/control/test_safety.py backend/tests/sim/test_adapters.py backend/tests/sim/test_ik.py backend/tests/test_soak.py scripts/soak_simulator.py scripts/generate_reachability_fixture.py schemas/fixtures/sim-reachability-v1.json schemas/fixtures/sim-reachability-v2.json
git commit -m "test: restore virtual LM3 baseline"
```

Before committing, run `git diff --cached --name-only` and confirm that `README.md`, `backend/app/sim/ik.py`, and the 2026-07-22 drafts are absent.

---

### Task 2: Add simulation/onsite profile normalization and comparison

**Files:**
- Create: `backend/app/acceptance/__init__.py`
- Create: `backend/app/acceptance/profile.py`
- Create: `backend/tests/acceptance/__init__.py`
- Create: `backend/tests/acceptance/test_profile.py`
- Modify: `backend/app/config.py`

**Interfaces:**
- Consumes: `LM3Model`, `LebaiSettings`, `TcpExpectation`, and the current simulation tool transform.
- Produces:
  - `RobotProfileSnapshot`
  - `ProfileDifference`
  - `ProfileComparison`
  - `simulation_profile()`
  - `onsite_profile(settings)`
  - `compare_profiles(simulation, onsite)`
  - `Settings.load(path: Path | None = None)`

- [ ] **Step 1: Write tests for explicit configuration paths**

Add to the existing config tests:

```python
def test_settings_load_accepts_an_explicit_path_without_mutating_environment(
    tmp_path, monkeypatch
) -> None:
    path = tmp_path / "sim.yaml"
    path.write_text(DEFAULT_CONFIG.read_text(encoding="utf-8"), encoding="utf-8")
    monkeypatch.delenv("VR4ARM_CONFIG", raising=False)

    settings = Settings.load(path)

    assert settings.backend == "simulator"
    assert "VR4ARM_CONFIG" not in os.environ
```

Run the single test and expect `TypeError` because `Settings.load()` currently accepts no argument.

- [ ] **Step 2: Add the backward-compatible explicit path**

Change:

```python
@classmethod
def load(cls, path: Path | None = None) -> "Settings":
    selected = path or Path(os.environ.get("VR4ARM_CONFIG", DEFAULT_CONFIG))
```

Read YAML from `selected`. Existing callers using `Settings.load()` must remain unchanged.

- [ ] **Step 3: Write profile normalization tests**

Create tests that assert:

```python
simulation = simulation_profile()
assert simulation.source == "simulation"
assert simulation.hardware_verified is False
assert simulation.home_q == LM3Model().home_q
assert len(simulation.joint_min_rad) == 6
assert len(simulation.joint_max_rad) == 6
assert np.linalg.norm(simulation.tcp.p) > 0
assert simulation.gripper_closes_as_normalized_value_increases is True
```

For `onsite_profile(control_settings())`, assert:

```python
assert onsite.source == "onsite_config"
assert onsite.hardware_verified is False
assert onsite.joint_min_rad == settings.soft_joint_min_rad
assert onsite.joint_max_rad == settings.soft_joint_max_rad
assert onsite.gripper_closes_as_normalized_value_increases is True
```

Also test a reversed onsite amplitude mapping and require a `gripper_direction` difference.

- [ ] **Step 4: Implement immutable profile types**

In `profile.py` define:

```python
@dataclass(frozen=True)
class RobotProfileSnapshot:
    source: Literal["simulation", "onsite_config"]
    hardware_verified: bool
    home_q: tuple[float, ...]
    joint_min_rad: tuple[float, ...]
    joint_max_rad: tuple[float, ...]
    tcp: Pose
    gripper_closes_as_normalized_value_increases: bool

@dataclass(frozen=True)
class ProfileDifference:
    field: str
    simulation: object
    onsite: object
    severity: Literal["info", "warning"]

@dataclass(frozen=True)
class ProfileComparison:
    onsite_configured: bool
    differences: tuple[ProfileDifference, ...]
```

All serialized values must be JSON-compatible via explicit `to_dict()` methods; do not serialize dataclasses through `__dict__`.

- [ ] **Step 5: Normalize the simulation tool transform correctly**

Compute flange-to-TCP as:

```python
transform = (
    translation(model.tool_root_offset_m)
    @ rotation(model.tool_rotation_xyzw)
    @ translation(model.tcp_offset_m)
)
```

Return its translation and quaternion as `Pose`. Do not compare the raw `tcp_offset_m` directly with Lebai TCP because the GLB tool root includes an additional rotation and offset.

- [ ] **Step 6: Implement difference rules**

`compare_profiles()` must report:

- `home_q[i]` when absolute difference exceeds `0.01 rad`;
- `joint_min_rad[i]` and `joint_max_rad[i]` for every non-equal limit, severity `info`;
- `tcp.position` when Euclidean difference exceeds `0.001 m`;
- `tcp.orientation` when quaternion rotation difference exceeds `0.5 deg`;
- `gripper_direction` when the semantic direction differs.

If onsite settings are absent, return:

```python
ProfileComparison(onsite_configured=False, differences=())
```

This is not a software-gate failure.

- [ ] **Step 7: Run and commit profile support**

Run:

```powershell
cd backend
python -m pytest tests/config tests/acceptance/test_profile.py -q
```

Expected: zero failures.

Commit:

```powershell
git add backend/app/config.py backend/app/acceptance/__init__.py backend/app/acceptance/profile.py backend/tests/acceptance/__init__.py backend/tests/acceptance/test_profile.py
git commit -m "feat: compare virtual and onsite LM3 profiles"
```

---

### Task 3: Establish a dual-backend contract

**Files:**
- Create: `backend/tests/robots/test_backend_contract.py`
- Modify: `backend/tests/robots/real_settings.py` only if a named fixture is needed; do not change production limits.

**Interfaces:**
- Consumes: `RobotBackend`, `SimRobotAdapter`, `RealLebaiAdapter`, `FakeLebaiClient`, `control_settings()`, `HomeOptions`.
- Produces: one parameterized behavioral contract proving that both backends satisfy the same upper-layer expectations without requiring identical internal trajectories.

- [ ] **Step 1: Define a test harness with backend-specific observations**

In `test_backend_contract.py` define:

```python
@dataclass
class BackendHarness:
    name: str
    backend: RobotBackend
    reachable_target: Pose
    advance: Callable[[float], Awaitable[None]]
    command_observed: Callable[[], bool]
    stop_count: Callable[[], int]
    close: Callable[[], Awaitable[None]]
```

The simulator harness advances by calling `adapter.robot.step(0.02)`. The fake-Lebai harness waits for `move_pvat` and observes `FakeLebaiClient.write_calls`.

- [ ] **Step 2: Write the shared preflight and state test**

Parameterize over `simulator` and `fake_lebai` and assert:

```python
before = await harness.backend.get_state()
preflight = await harness.backend.preflight()
after = await harness.backend.get_state()

assert preflight.ready is True
assert preflight.actual_q == before.actual_q
assert after.actual_q == before.actual_q
assert "command_tcp" in preflight.capabilities
assert "home" in preflight.capabilities
assert "gripper" in preflight.capabilities
```

- [ ] **Step 3: Write the shared command/ack contract**

For both harnesses:

```python
await backend.command_tcp(harness.reachable_target, command_id=17)
await harness.advance(0.10)
assert harness.command_observed()
state = await backend.get_state()
assert state.ack_seq in {None, 17}
```

The real fake path may acknowledge only after a PVAT write; the simulator must acknowledge `17`. Express that difference in the harness observation rather than weakening safety assertions.

- [ ] **Step 4: Write the shared idempotent stop contract**

Call:

```python
await backend.stop(StopReason.GRIP_RELEASED)
await backend.stop(StopReason.GRIP_RELEASED)
```

Assert that no new motion command is accepted from stale pending work. For the simulator, `target_q` and `target_qd` must be `None`; for fake Lebai, delayed IK must not produce `move_pvat` after the first stop. Do not require the two adapters to make the same number of vendor calls.

- [ ] **Step 5: Write the Home phase contract**

Capture phases in a list and require:

```python
assert phases[0] == "homing"
assert phases[-1] in {"homing", "stabilizing"}
assert set(phases) <= {"homing", "stabilizing"}
```

For the simulator, advance the virtual robot until tolerance is stable. For fake Lebai, set `motion_state = "FINISHED"` and `actual_joint_pose = home_q`. Verify neither path publishes an unknown phase.

- [ ] **Step 6: Run focused and adjacent adapter tests**

Run:

```powershell
cd backend
python -m pytest tests/robots/test_backend_contract.py tests/sim/test_adapters.py tests/robots/test_lebai_adapter_control.py tests/robots/test_lebai_adapter_readonly.py -q
```

Expected: zero failures and zero real SDK imports.

- [ ] **Step 7: Commit the backend contract**

```powershell
git add backend/tests/robots/test_backend_contract.py backend/tests/robots/real_settings.py
git commit -m "test: enforce shared robot backend contract"
```

If `real_settings.py` did not change, omit it from `git add`.

---

### Task 4: Add deterministic virtual motion scenarios and repair soak coverage

**Files:**
- Create: `backend/app/acceptance/scenarios.py`
- Create: `backend/tests/acceptance/test_scenarios.py`
- Modify: `backend/app/robots/sim_adapter.py`
- Modify: `scripts/soak_simulator.py`
- Modify: `backend/tests/test_soak.py`

**Interfaces:**
- Consumes: `CoordinateMapper`, `SafetyLimiter`, `SimRobotAdapter`, `RobotControl`, `LatestVRFrame`, `VRFrame`.
- Produces:
  - `ScenarioResult(name: str, passed: bool, metrics: dict[str, object], failures: tuple[str, ...])`
  - `run_mapping_scenario()`
  - `run_servo_tracking_scenario()`
  - `run_virtual_scenarios()`
  - `SimRobotAdapter(*, servo: CartesianServo = cartesian_servo_step)`
  - corrected `run_soak()`

- [ ] **Step 1: Write exact six-axis mapping tests**

For a captured hand pose `(0.0, 1.2, -0.3)` and TCP pose `(0.3, 0.4, -0.2)`, run independent `+0.02 m` deltas in XR X/Y/Z and assert:

```python
target.p == pytest.approx(
    np.asarray(tcp_anchor.p) + np.asarray(delta),
    abs=1e-9,
)
```

Run independent `+10 deg` roll, pitch, and yaw controller deltas and assert the target rotation matrix equals:

```python
Rotation.from_euler(axis, 10, degrees=True) * Rotation.from_quat(tcp_anchor.q)
```

This test encodes the approved one-to-one world-delta mapping: controller forward/down/side motion maps to the same signed XR scene axis at the TCP target.

- [ ] **Step 2: Write a simulator tracking test**

Starting from `model.home_q`, generate a reachable target from:

```python
target_q = np.asarray(model.home_q) + np.array(
    [0.03, -0.02, 0.025, 0.015, -0.01, 0.02]
)
target = forward_pose(target_q, model)
```

For 150 cycles:

```python
await adapter.command_tcp(target, command_id=cycle)
adapter.robot.step(0.02)
```

Assert:

```python
final_position_error < initial_position_error
final_position_error <= 0.005
np.max(np.abs(adapter.robot.qd)) <= model.max_joint_speed_radps + 1e-9
np.all(np.abs(adapter.robot.q - np.asarray(model.home_q)) <= model.joint_window_rad)
```

- [ ] **Step 3: Implement result objects and mapping metrics**

`ScenarioResult.to_dict()` must sort metric keys and serialize tuples as lists. `run_mapping_scenario()` returns metrics for all six axes:

```python
translation_errors = [
    float(np.linalg.norm(np.asarray(actual) - np.asarray(expected)))
    for actual, expected in translation_pairs
]
rotation_errors = [
    float((actual * expected.inv()).magnitude())
    for actual, expected in rotation_pairs
]

{
    "translation_axes_checked": 3,
    "rotation_axes_checked": 3,
    "max_translation_error_m": max(translation_errors, default=0.0),
    "max_rotation_error_rad": max(rotation_errors, default=0.0),
}
```

No metric may contain NaN or infinity.

- [ ] **Step 4: Implement servo tracking without a background task**

`run_servo_tracking_scenario()` must instantiate `SimRobotAdapter` but advance `VirtualRobot` manually at exactly `0.02 s`; do not call `connect()` and do not create a wall-clock task. Return:

```python
initial_position_error_m = float(
    np.linalg.norm(np.asarray(initial_tcp.p) - np.asarray(target.p))
)
final_position_error_m = float(
    np.linalg.norm(np.asarray(final_tcp.p) - np.asarray(target.p))
)

{
    "cycles": 150,
    "initial_position_error_m": initial_position_error_m,
    "final_position_error_m": final_position_error_m,
    "max_joint_speed_radps": observed_max_joint_speed_radps,
    "joint_window_violations": 0,
    "nan_count": 0,
}
```

- [ ] **Step 5: Add a narrow injectable servo seam**

In `backend/app/robots/sim_adapter.py` define:

```python
CartesianServo = Callable[..., CartesianServoResult]

def __init__(
    self,
    *,
    servo: CartesianServo = cartesian_servo_step,
) -> None:
    self.model = LM3Model()
    self.robot = VirtualRobot(self.model)
    self.gripper = 0.0
    self.command_id: int | None = None
    self.mode = TeleopMode.READY
    self.fault: str | None = None
    self._lock = asyncio.Lock()
    self._task: asyncio.Task[None] | None = None
    self._servo = servo
```

Import `CartesianServoResult` from `app.sim.cartesian_servo` and replace only the call inside `command_tcp()`:

```python
result = self._servo(
    target,
    self.robot.q,
    self.model,
    dt=self.STEP_SECONDS,
)
```

Normal construction remains `SimRobotAdapter()` and uses the same production function.

- [ ] **Step 6: Add a retreatable soft-constraint scenario**

Construct `SimRobotAdapter(servo=fail_once_then_delegate)` where the first call raises `IKError("ik_unreachable")` and later calls delegate to `cartesian_servo_step`. Drive it through `RobotControl` and require:

```python
mode == "ACTIVE"
constraint == "ik_boundary"
fault is None
fault_stop_count == 0
```

Then send valid targets for at least `constraint_clear_ms` and require the constraint to clear without Reset or Home.

- [ ] **Step 7: Complete the corrected soak event schedule**

Add `ik_boundary` to the deterministic event schedule. Its verification is:

```python
control.mode is TeleopMode.ACTIVE
state.constraint == "ik_boundary"
stop_counts[StopReason.FAULT.value] unchanged
```

`command_fault` continues to use `backend_command_failed` and must enter/publish `FAULT`, issue one fault stop, accept explicit reset, and resume.

Split event verification counters so a correctly non-stopping soft constraint is not mistaken for a missing stop:

```python
assert summary["injected_events"] == 7
assert summary["verified_injected_events"] == 7
assert summary["stop_expected_events"] == 6
assert summary["verified_injected_stops"] == 6
```

The six stop-expected events remain tracking loss, hidden, disconnect, command fault, safety fault, and visible-blurred. `ik_boundary` increments `verified_injected_events` only after ACTIVE mode, published constraint, and unchanged fault-stop count are all observed.

- [ ] **Step 8: Run scenario and soak tests**

Run:

```powershell
cd backend
python -m pytest tests/acceptance/test_scenarios.py tests/test_soak.py -q
```

Expected: zero failures; ten simulated minutes complete with 30,000 control and virtual steps for each seeded run.

- [ ] **Step 9: Commit deterministic scenarios**

```powershell
git add backend/app/acceptance/scenarios.py backend/tests/acceptance/test_scenarios.py backend/app/robots/sim_adapter.py scripts/soak_simulator.py backend/tests/test_soak.py
git commit -m "test: add deterministic virtual LM3 scenarios"
```

---

### Task 5: Complete frontend controller and interaction acceptance

**Files:**
- Modify: `web/tests/controllerInput.test.ts`
- Modify: `web/tests/xrSession.test.ts`
- Create: `web/tests/virtualAcceptance.test.ts`

**Interfaces:**
- Consumes: `readControllers()`, `XRSessionController`, `KinematicGraspController`, `createGraspBlocks()`, authoritative `RobotStateMessage.actual_tcp` and `gripper`.
- Produces: a zero-failure frontend baseline plus one end-to-end deterministic interaction test for all five blocks.

- [ ] **Step 1: Update exact controller expectations**

For an unavailable left-hand pose, require:

```typescript
expect(sample.left).toEqual({
  p: [0, 0, 0],
  q: [0, 0, 0, 1],
  trackingValid: false,
  thumbstickX: 0,
  thumbstickY: 0,
  thumbstickPressed: false,
  grip: false,
});
```

Do not replace this with `objectContaining`; exact shape is useful protocol-to-scene coverage.

- [ ] **Step 2: Update XR presentation expectations**

Add:

```typescript
headQ: null
```

to zero/invalid presentation samples, and add `thumbstickX: 0` plus `grip: false` to exact left-controller samples. Keep existing `objectContaining` only where a test intentionally ignores unrelated right-controller face buttons.

- [ ] **Step 3: Run the repaired controller tests**

Run:

```powershell
cd web
npm.cmd test -- --run tests/controllerInput.test.ts tests/xrSession.test.ts
```

Expected: zero failures.

- [ ] **Step 4: Write a full five-block acceptance test**

In `virtualAcceptance.test.ts`:

1. create `visualRoot` and `createGraspBlocks()`;
2. add every block directly to `visualRoot`;
3. construct `KinematicGraspController`;
4. for each block, send authoritative TCP at the block with gripper `0.2`, then `0.7`;
5. carry it to a common stack TCP at `[0, 0.3, 0]`;
6. release with gripper `0.2`;
7. after every release, assert no pair has strict AABB overlap;
8. assert final Y centers equal `[0.025, 0.085, 0.145, 0.205, 0.265]`;
9. call `reset()` and assert all five initial positions and identity rotations are restored.

Use the existing close thresholds `<= 0.35` and `>= 0.65`; do not expose a second threshold configuration.

- [ ] **Step 5: Verify the authoritative-state boundary**

Add a negative assertion: move an unexecuted target TCP while leaving the supplied authoritative `actual_tcp` unchanged and assert the carried block does not move. The test must call the grasp controller only with `actual_tcp`.

- [ ] **Step 6: Run full frontend tests and build**

Run:

```powershell
cd web
npm.cmd test -- --run
npm.cmd run build
```

Expected: zero test failures and build exit code `0`. Record the existing Vite chunk-size warning as non-blocking if it remains.

- [ ] **Step 7: Commit frontend acceptance**

```powershell
git add web/tests/controllerInput.test.ts web/tests/xrSession.test.ts web/tests/virtualAcceptance.test.ts
git commit -m "test: validate virtual LM3 interaction flow"
```

---

### Task 6: Build the machine-readable acceptance report

**Files:**
- Create: `scripts/accept_virtual_lm3.py`
- Create: `backend/tests/scripts/test_accept_virtual_lm3.py`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: `run_virtual_scenarios()`, `run_soak()`, profile comparison, Git CLI, pytest, Vitest, and Vite.
- Produces:
  - `CommandResult`
  - `AcceptanceReport`
  - `run_gate(repo_root: Path, real_config: Path | None = None, command_runner: CommandRunner = run_command)`
  - `report_exit_code(report: AcceptanceReport) -> int`
  - CLI exit `0` only when all non-hardware checks pass
  - JSON report under `artifacts/acceptance/`

- [ ] **Step 1: Write report-schema tests**

Define the immutable report types in the script:

```python
@dataclass(frozen=True)
class CommandResult:
    name: str
    argv: tuple[str, ...]
    cwd: str
    returncode: int
    duration_s: float
    passed_count: int | None
    failed_count: int | None
    output_tail: str

@dataclass(frozen=True)
class AcceptanceReport:
    generated_at: str
    git: dict[str, object]
    model: dict[str, object]
    profile: dict[str, object]
    commands: tuple[CommandResult, ...]
    scenarios: tuple[ScenarioResult, ...]
    soak: dict[str, object]
    hardware_pending: tuple[str, ...]
    passed: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": 1,
            "generated_at": self.generated_at,
            "git": self.git,
            "model": self.model,
            "profile": self.profile,
            "commands": [
                {
                    "name": command.name,
                    "argv": list(command.argv),
                    "cwd": command.cwd,
                    "returncode": command.returncode,
                    "duration_s": command.duration_s,
                    "passed_count": command.passed_count,
                    "failed_count": command.failed_count,
                    "output_tail": command.output_tail,
                }
                for command in self.commands
            ],
            "scenarios": [scenario.to_dict() for scenario in self.scenarios],
            "soak": self.soak,
            "hardware_verified": False,
            "hardware_pending": list(self.hardware_pending),
            "passed": self.passed,
        }
```

`to_dict()` must always emit `schema_version: 1` and `hardware_verified: false`; it must explicitly convert nested dataclasses rather than using `__dict__`.

Require the serialized top-level keys:

```python
{
    "schema_version",
    "generated_at",
    "git",
    "model",
    "profile",
    "commands",
    "scenarios",
    "soak",
    "hardware_verified",
    "hardware_pending",
    "passed",
}
```

Require:

```python
assert report["schema_version"] == 1
assert report["hardware_verified"] is False
assert report["passed"] is True
assert report["hardware_pending"]
```

Use a fake command runner; unit tests must not recursively launch the full suite.

- [ ] **Step 2: Write failure-propagation tests**

Provide one fake result:

```python
failure = CommandResult(
    name="backend",
    argv=("python", "-m", "pytest", "-q"),
    cwd=str(repo_root / "backend"),
    returncode=1,
    duration_s=0.1,
    passed_count=323,
    failed_count=1,
    output_tail="1 failed, 323 passed",
)
report = run_gate(
    repo_root,
    command_runner=lambda _name, _argv, _cwd: failure,
)

assert report.passed is False
assert report_exit_code(report) == 1
```

Provide `real_config=None` and assert this does not fail the software gate, but `profile.onsite_configured` remains false.

- [ ] **Step 3: Define exact commands and cross-platform npm selection**

The default gate commands are:

```python
[
    [sys.executable, "-m", "pytest", "-q"],  # cwd backend
    [npm_executable, "test", "--", "--run"], # cwd web
    [npm_executable, "run", "build"],        # cwd web
]
```

Use `npm.cmd` on Windows and `npm` elsewhere. Capture stdout/stderr, duration, return code, and parse summary counts with regexes that accept both `313 passed` and `Tests  258 passed`.

- [ ] **Step 4: Implement Git and model provenance**

Record:

```python
{
    "commit": git_rev_parse_head,
    "dirty": bool(git_status_porcelain),
    "dirty_paths": sorted(paths),
}
```

Record the SHA-256 of:

- `config/lm3_visual_kinematics_v1.json`;
- `web/public/models/Lebai_LM3.glb`.

Do not fail solely because the worktree contains the known user changes, but include them in the report. A command/test failure still fails the gate.

- [ ] **Step 5: Add scenario, soak, and profile sections**

Call:

```python
scenario_results = run_virtual_scenarios()
soak = run_soak(minutes=10, seed=42)
```

If `--real-config` is supplied, load it with `Settings.load(path)` and require `settings.lebai is not None`; otherwise report `onsite_configured: false`.

Set:

```python
hardware_verified = False
hardware_pending = [
    "sdk_connection",
    "tcp_home_joint_limits",
    "gripper_direction_force",
    "pvat_tracking_latency",
    "stop_distance_estop",
    "physical_collision_load",
]
```

- [ ] **Step 6: Implement atomic report output**

Default output:

```text
artifacts/acceptance/virtual-lm3-latest.json
```

Write to a sibling `.tmp` path, flush and close it, then replace the final path with `Path.replace()`. Delete a stale temporary file only if it is inside `artifacts/acceptance/`.

Add:

```gitignore
artifacts/acceptance/
```

- [ ] **Step 7: Run script unit tests**

Run:

```powershell
cd backend
python -m pytest tests/scripts/test_accept_virtual_lm3.py -q
```

Expected: zero failures without launching nested pytest/npm processes.

- [ ] **Step 8: Run the real non-hardware gate once**

Run from repository root:

```powershell
python scripts/accept_virtual_lm3.py
```

Expected:

- exit code `0`;
- report file exists;
- `passed: true`;
- `hardware_verified: false`;
- backend, frontend, build, scenarios, and soak all show success;
- no Lebai SDK connection occurs.

- [ ] **Step 9: Commit the report runner**

```powershell
git add .gitignore scripts/accept_virtual_lm3.py backend/tests/scripts/test_accept_virtual_lm3.py
git commit -m "feat: add virtual LM3 acceptance report"
```

Do not add the generated JSON report.

---

### Task 7: Document, audit, and freeze the non-hardware milestone

**Files:**
- Create: `docs/virtual-lm3-acceptance.md`

**Interfaces:**
- Consumes: the final CLI and report schema.
- Produces: an operator procedure and verified local milestone commits, without push.

- [ ] **Step 1: Write the operator procedure**

Document:

```powershell
python scripts/accept_virtual_lm3.py
python scripts/accept_virtual_lm3.py --real-config config/real-robot.local.yaml
```

Explain:

- the first command performs software-only validation;
- the second compares onsite values but still performs no hardware motion;
- `hardware_verified: false` is expected;
- a failed command causes a nonzero exit;
- the report location;
- the six hardware-only categories that remain.

- [ ] **Step 2: Document the trust boundary**

State explicitly:

- GLB equality supports geometry, kinematics, visualization, and approximate self-collision testing;
- deterministic grasping supports control-flow and data-interface tests;
- neither proves physical collision avoidance, friction, force, payload, stop distance, or emergency-stop behavior;
- passing the gate permits only the existing `LEBAI_READONLY` preflight as the next stage.

- [ ] **Step 3: Run a safety-string audit**

Run:

```powershell
rg -n "start_sys|set_tcp|init_claw|speedl" backend scripts
rg -n "move_pvat|stop_move|stop_sys" backend/app scripts
```

Expected:

- no `start_sys`, `set_tcp`, `init_claw`, or `speedl`;
- `move_pvat` only in the guarded adapter/bridge path;
- `stop_move` remains normal stop;
- `stop_sys` remains failed-stop escalation only.

- [ ] **Step 4: Run final complete verification**

Run:

```powershell
cd backend
python -m pytest -q
cd ..\web
npm.cmd test -- --run
npm.cmd run build
cd ..
python scripts/accept_virtual_lm3.py
git diff --check
git status --short
```

Read every exit code and report the actual counts. Do not infer full-suite success from focused tests.

- [ ] **Step 5: Verify the generated report**

Read `artifacts/acceptance/virtual-lm3-latest.json` and verify:

```python
report["passed"] is True
report["hardware_verified"] is False
len(report["hardware_pending"]) == 6
report["soak"]["error_count"] == 0
```

Verify the model and GLB hashes are non-empty 64-character lowercase hexadecimal strings.

- [ ] **Step 6: Commit documentation only**

```powershell
git add docs/virtual-lm3-acceptance.md
git commit -m "docs: add virtual LM3 acceptance workflow"
```

Do not stage or modify `README.md` in this plan.

- [ ] **Step 7: Final repository audit**

Run:

```powershell
git log --oneline -8
git status --short
git diff --name-only HEAD -- README.md backend/app/sim/ik.py docs/superpowers/plans/2026-07-22-tool-frame-and-ik-continuity.md docs/superpowers/specs/2026-07-22-tool-frame-and-ik-continuity-design.md
```

Report:

- each new local commit hash;
- exact full-test counts;
- acceptance report path;
- known user-owned dirty files;
- all hardware-only pending checks;
- “not pushed”.

Do not merge or push unless the user separately asks.

## Plan Self-Review

- **Spec coverage:** Tasks 1–7 cover baseline repair, current-model IK provenance, shared logical contract, profile comparison, deterministic six-axis/servo/constraint/stop/Home tests, long soak, frontend grasp/stacking, JSON reporting, and the hardware boundary.
- **Scope:** No physics engine, camera, recording expansion, real SDK call, protocol change, merge, or push is included.
- **Type consistency:** `RobotProfileSnapshot`, `ProfileComparison`, `ScenarioResult`, `CommandResult`, and `AcceptanceReport` are each defined before their consumers.
- **Dirty-worktree safety:** Every commit step uses explicit paths and requires staged-name inspection; the four user-owned paths are excluded.
- **No placeholders:** Every implementation step specifies concrete functions, fields, assertions, commands, and expected outcomes.
