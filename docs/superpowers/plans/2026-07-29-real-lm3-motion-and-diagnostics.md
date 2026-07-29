# Real LM3 Motion and Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the existing Quest 3 teleoperation stack ready to move a real LM3 on the next lab visit, while providing a deterministic RealLebaiAdapter digital twin, guarded per-action commissioning commands, a full PC diagnostics rail, and a compact Quest safety HUD.

**Architecture:** Preserve `RobotControl` as the only motion authority and `RealLebaiAdapter` as the only production LM3 adapter. Add a separately launched Fake Lebai client behind the same adapter, feed bounded diagnostics through the existing single-owner WebSocket, and expand the existing one-axis smoke workflow into mutually exclusive guarded actions.

**Tech Stack:** Python 3.11+, FastAPI, asyncio, Pydantic, NumPy, SciPy, pytest, TypeScript, Vitest, Three.js, Vite, YAML/JSON, Lebai SDK-shaped async interfaces.

## Global Constraints

- Never connect to, import, start, configure, or move a real LM3 during implementation or ordinary automated tests.
- Never call `start_sys()`, `set_tcp()`, `init_claw()`, `estop()`, or unlimited `speedl(t=0)`.
- Real motion continues to require local `mode: control`, exact `VR4ARM_REAL_ROBOT_CONFIRM`, successful preflight, explicit session arm, and a fresh released Grip.
- Production LM3 Cartesian commands must continue through vendor `kinematics_inverse(actual_q)` and bounded latest-wins PVAT.
- Digital-twin IK may use the virtual LM3 solver but must always report `LEBAI_FAKE`, `DIGITAL_TWIN`, and `hardware_verified: false`.
- A normal production YAML file cannot select Fake Lebai. The Fake client is injected only by the dedicated launcher.
- PC and Quest remain a single-control-client system. Diagnostics travel only on the current owner's WebSocket.
- Translation smoke actions are signed and hard-limited to `abs(distance_m) <= 0.005`.
- Rotation smoke actions are signed and hard-limited to `abs(angle_deg) <= 2.0`.
- First-pass gripper force remains bounded by the existing configuration maximum of `30%`.
- No batch action list, arbitrary trajectory, camera, dataset, VLA export, ROS 2, MoveIt, or new physics engine is added.
- Preserve protocol version `1`; new fields and message types must be exact, finite, and backward-compatible within the jointly shipped backend/frontend.
- Generated acceptance reports must always contain `hardware_verified: false`.
- Do not modify or stage the user's existing `README.md`, `backend/app/sim/ik.py`, `docs/superpowers/plans/2026-07-22-tool-frame-and-ik-continuity.md`, or `docs/superpowers/specs/2026-07-22-tool-frame-and-ik-continuity-design.md`.
- Make local commits only. Do not merge or push.
- Do not claim a test passes without running its exact command and reading its exit code and output.

## File Structure

### Digital-twin Lebai path

- `backend/app/digital_twin/__init__.py`: public digital-twin exports.
- `backend/app/digital_twin/lebai_client.py`: deterministic SDK-shaped client, authoritative state evolution, and fault injection.
- `backend/app/digital_twin/runtime.py`: safe readonly-config loading, Fake-only control promotion, and app creation.
- `backend/tests/digital_twin/test_lebai_client.py`: authoritative state, IK, PVAT, Home, gripper, stop, and injected-failure tests.
- `backend/tests/digital_twin/test_runtime.py`: runtime identity, no-real-connector, and health tests.
- `config/fake-lebai.yaml`: versioned safe readonly source profile for the dedicated launcher.
- `scripts/run_fake_lebai_stack.py`: explicit Fake Lebai FastAPI entrypoint.

### Runtime identity and protocol

- `backend/app/main.py`: optional explicit backend label passed to the adapter and application state.
- `backend/app/robots/lebai_adapter.py`: immutable `LEBAI`/`LEBAI_FAKE` state label.
- `backend/app/schemas/messages.py`: allow exact `LEBAI_FAKE` robot state identity and define diagnostics messages.
- `backend/tests/contract/test_messages.py`: Python schema contract.
- `web/src/protocol/messages.ts`: matching TypeScript runtime identity and diagnostics guards.
- `web/tests/messages.test.ts`: matching frontend contract tests.
- `schemas/fixtures/robot-state-valid.json`: keep the canonical fixture valid while adding optional identity support.

### Guarded onsite actions

- `backend/app/commissioning/actions.py`: action dataclasses, signed bounds, frame construction, and result serialization.
- `backend/app/commissioning/smoke.py`: subcommand parsing, preflight, one-action execution, stable-state confirmation, and cleanup.
- `backend/tests/commissioning/test_actions.py`: exact action mapping and bounds.
- `backend/tests/scripts/test_real_robot_scripts.py`: parser and Fake-client integration.
- `scripts/real_robot_smoke.py`: remains a thin executable wrapper.

### Bounded diagnostics

- `backend/app/diagnostics/__init__.py`: public diagnostics exports.
- `backend/app/diagnostics/store.py`: bounded event ring, coalescing, kinematic snapshot, rate metrics, and message construction.
- `backend/app/diagnostics/recorder.py`: `RecorderSink` wrapper that preserves durable recorder failure semantics while observing events.
- `backend/tests/diagnostics/test_store.py`: boundedness, critical-event retention, finite snapshots, and send-rate tests.
- `backend/tests/diagnostics/test_recorder.py`: delegation and failure propagation tests.
- `schemas/fixtures/diagnostics-valid.json`: canonical cross-language diagnostics payload.
- `backend/app/api/teleop_ws.py`: 5 Hz diagnostics sender coupled to the single owner session.
- `backend/tests/api/test_teleop_ws.py`: owner-only diagnostics delivery and disconnect cleanup.

### PC diagnostics and Quest HUD

- `web/src/ui/diagnosticsPanel.ts`: complete PC diagnostics rail.
- `web/src/ui/hud.ts`: runtime identity header, simplified state, and diagnostics host.
- `web/src/ui/armPanel.ts`: backend-aware labels without changing authorization.
- `web/src/scenes/vrSafetyPanel.ts`: compact runtime/TCP/latency line and actionable real-robot wording.
- `web/src/scenes/simulationScene.ts`: pass authoritative runtime summary to the VR safety panel.
- `web/src/transport/teleopSocket.ts`: diagnostics message callback.
- `web/src/main.ts`: join robot state, diagnostics, PC rail, and Quest summary.
- `web/src/styles.css`: A-layout desktop rail and compact HUD styling.
- `web/tests/diagnosticsPanel.test.ts`: complete PC content and bounded event rendering.
- `web/tests/hud.test.ts`, `web/tests/armPanel.test.ts`, `web/tests/vrSafetyPanel.test.ts`, `web/tests/teleopSocket.test.ts`: runtime wording, action hints, and message transport.

### Offline acceptance and operator docs

- `backend/app/acceptance/fake_lebai.py`: deterministic six-translation, six-rotation, gripper, Home, stop, and fault scenarios.
- `backend/tests/acceptance/test_fake_lebai.py`: scenario invariants.
- `scripts/accept_fake_lebai.py`: full backend/frontend/build plus Fake-real-path JSON gate.
- `backend/tests/scripts/test_accept_fake_lebai.py`: report schema and failure propagation.
- `docs/fake-lebai-acceptance.md`: offline operating procedure and trust boundary.
- `docs/real-robot-deployment.md`: exact new smoke subcommands and final staged lab sequence.
- `.gitignore`: generated Fake acceptance report directory only if the existing acceptance ignore does not already cover it.

---

### Task 1: Build the deterministic SDK-shaped LM3 digital twin

**Files:**
- Create: `backend/app/digital_twin/__init__.py`
- Create: `backend/app/digital_twin/lebai_client.py`
- Create: `backend/tests/digital_twin/__init__.py`
- Create: `backend/tests/digital_twin/test_lebai_client.py`

**Interfaces:**
- Consumes: `LebaiSettings`, `LebaiClientProtocol`, `LM3Model`, `forward_pose()`, `cartesian_servo_step()`, and Lebai pose codecs.
- Produces:
  - `DigitalTwinFaults`
  - `DigitalTwinLebaiClient`
  - `DigitalTwinLebaiClient.idle(settings, *, clock, faults=None)`
  - `DigitalTwinLebaiClient.set_faults(faults)`
  - all methods required by `LebaiClientProtocol`

- [ ] **Step 1: Write the authoritative PVAT-state test**

```python
from tests.robots.fake_lebai import ACTUAL_TCP, IDLE_Q


@pytest.mark.asyncio
async def test_pvat_changes_actual_state_only_after_virtual_time_advances() -> None:
    clock = FakeNanosecondClock()
    settings = control_settings()
    client = DigitalTwinLebaiClient.idle(settings, clock=clock.now_ns)
    before = await client.get_kin_data()
    q = np.asarray(before["actual_joint_pose"], dtype=float)
    target_q = (q + np.array([0.01, -0.01, 0.01, 0, 0, 0])).tolist()

    await client.move_pvat(target_q, [0.1] * 6, [0.0] * 6, 0.08)
    immediate = await client.get_kin_data()
    assert immediate["actual_joint_pose"] == pytest.approx(q)

    clock.advance_seconds(0.04)
    halfway = await client.get_kin_data()
    assert np.linalg.norm(np.asarray(halfway["actual_joint_pose"]) - q) > 0
    assert np.linalg.norm(np.asarray(halfway["actual_joint_pose"]) - target_q) > 0

    clock.advance_seconds(0.04)
    final = await client.get_kin_data()
    assert final["actual_joint_pose"] == pytest.approx(target_q)
    assert final["actual_joint_speed"] == pytest.approx([0.0] * 6)
```

Use a test-local clock with:

```python
class FakeNanosecondClock:
    def __init__(self) -> None:
        self.value = 0

    def now_ns(self) -> int:
        return self.value

    def advance_seconds(self, seconds: float) -> None:
        self.value += round(seconds * 1_000_000_000)
```

- [ ] **Step 2: Run the PVAT test and verify the missing module failure**

Run:

```powershell
cd backend
python -m pytest tests/digital_twin/test_lebai_client.py::test_pvat_changes_actual_state_only_after_virtual_time_advances -q
```

Expected: FAIL during collection because `app.digital_twin.lebai_client` does not exist.

- [ ] **Step 3: Define the immutable fault model and client state**

Implement:

```python
@dataclass(frozen=True)
class DigitalTwinFaults:
    sdk_latency_s: float = 0.0
    disconnect: bool = False
    ik_failure: bool = False
    pvat_failure: bool = False
    stop_failure: bool = False


@dataclass(frozen=True)
class _Motion:
    started_ns: int
    duration_ns: int
    start_q: JointVector
    target_q: JointVector
    target_qd: JointVector
    target_qdd: JointVector


class DigitalTwinLebaiClient:
    def __init__(
        self,
        settings: LebaiSettings,
        *,
        clock: Callable[[], int] = time.monotonic_ns,
        faults: DigitalTwinFaults | None = None,
    ) -> None:
        self.settings = settings
        self.model = LM3Model()
        self._clock = clock
        self._faults = faults or DigitalTwinFaults()
        self._q = tuple(float(value) for value in settings.home_q)
        self._qd = (0.0,) * 6
        self._qdd = (0.0,) * 6
        self._motion: _Motion | None = None
        self._gripper_amplitude = settings.gripper.open_amplitude_percent
        self._write_calls: list[tuple[object, ...]] = []

    @classmethod
    def idle(
        cls,
        settings: LebaiSettings,
        *,
        clock: Callable[[], int] = time.monotonic_ns,
        faults: DigitalTwinFaults | None = None,
    ) -> "DigitalTwinLebaiClient":
        return cls(settings, clock=clock, faults=faults)

    def set_faults(self, faults: DigitalTwinFaults) -> None:
        self._faults = faults
```

Reject negative/non-finite `sdk_latency_s` in `DigitalTwinFaults.__post_init__`.

- [ ] **Step 4: Implement deterministic motion advancement**

Use one private advancement function before every state read:

```python
def _advance(self) -> None:
    motion = self._motion
    if motion is None:
        return
    elapsed = max(0, self._clock() - motion.started_ns)
    alpha = min(1.0, elapsed / motion.duration_ns)
    start = np.asarray(motion.start_q)
    target = np.asarray(motion.target_q)
    self._q = _joint_tuple(start + alpha * (target - start))
    if alpha >= 1.0:
        self._qd = (0.0,) * 6
        self._qdd = (0.0,) * 6
        self._motion = None
    else:
        self._qd = motion.target_qd
        self._qdd = motion.target_qdd
```

Use this exact finite conversion helper:

```python
def _joint_tuple(values: object) -> JointVector:
    array = np.asarray(values, dtype=float)
    if array.shape != (6,) or not np.all(np.isfinite(array)):
        raise RuntimeError("digital_twin_invalid_joint_vector")
    return tuple(float(value) for value in array)  # type: ignore[return-value]
```

`get_kin_data()` must compute `actual_tcp_pose` from `forward_pose(self._q, self.model)` and must never expose a target as actual before `_advance()` reaches it.

- [ ] **Step 5: Implement IK, PVAT, Home, gripper, and stop**

Implement the SDK-shaped behavior:

```python
async def kinematics_inverse(
    self,
    pose: dict[str, float],
    joints: list[float],
) -> object:
    await self._delay()
    self._require_connected()
    if self._faults.ik_failure:
        return None
    result = cartesian_servo_step(
        pose_from_lebai(pose),
        np.asarray(joints, dtype=float),
        self.model,
        dt=1 / self.settings.control.pvat_send_hz,
    )
    return list(result.target_q)

async def move_pvat(
    self,
    p: list[float],
    v: list[float],
    a: list[float],
    t: float,
) -> object:
    await self._delay()
    self._require_connected()
    if self._faults.pvat_failure:
        raise RuntimeError("digital_twin_pvat_failure")
    self._advance()
    self._motion = _Motion(
        started_ns=self._clock(),
        duration_ns=max(1, round(t * 1_000_000_000)),
        start_q=self._q,
        target_q=joint_vector(p, "pvat_position"),
        target_qd=joint_vector(v, "pvat_velocity"),
        target_qdd=joint_vector(a, "pvat_acceleration"),
    )
    self._write_calls.append(("move_pvat", list(p), list(v), list(a), t))
    return len(self._write_calls)
```

`movej()` creates a `_Motion` to the requested joints, `set_claw()` updates raw force/amplitude, `stop_move()` freezes the advanced current state, and `stop_sys()` does the same while recording the escalated method. If `stop_failure` is true, both stop methods leave nonzero velocity.

State methods reflect authoritative execution:

- `get_robot_state()` returns `MOVING` while `_motion` exists and `IDLE` after completion;
- `get_running_motion()` returns the current motion identifier or `None`;
- `get_motion_state(id)` returns `RUNNING` or `FINISHED`;
- `get_tcp()` returns the configured `TcpExpectation` in Lebai Euler form;
- `get_claw()` returns the latest raw force and amplitude;
- every public SDK-shaped method applies `sdk_latency_s` through `_delay()`.

- [ ] **Step 6: Add fault and capability tests**

Cover:

```python
async def test_digital_twin_implements_every_detected_sdk_capability() -> None:
    client = DigitalTwinLebaiClient.idle(control_settings())
    assert detect_capabilities(client).control_ready is True

async def test_ik_failure_returns_none_without_mutating_state() -> None:
    client = DigitalTwinLebaiClient.idle(
        control_settings(),
        faults=DigitalTwinFaults(ik_failure=True),
    )
    before = await client.get_kin_data()
    assert await client.kinematics_inverse(ACTUAL_TCP, list(IDLE_Q)) is None
    assert await client.get_kin_data() == before

async def test_disconnect_rejects_reads_and_writes() -> None:
    client = DigitalTwinLebaiClient.idle(
        control_settings(),
        faults=DigitalTwinFaults(disconnect=True),
    )
    assert await client.is_connected() is False
    with pytest.raises(RuntimeError, match="digital_twin_disconnected"):
        await client.get_kin_data()
```

- [ ] **Step 7: Run focused and adjacent backend tests**

Run:

```powershell
cd backend
python -m pytest tests/digital_twin/test_lebai_client.py tests/robots/test_lebai_adapter_control.py tests/robots/test_backend_contract.py -q
```

Expected: zero failures and no import of `lebai_sdk`.

- [ ] **Step 8: Commit the digital-twin client**

Before committing, confirm staged paths:

```powershell
git add backend/app/digital_twin/__init__.py backend/app/digital_twin/lebai_client.py backend/tests/digital_twin/__init__.py backend/tests/digital_twin/test_lebai_client.py
git diff --cached --name-only
git commit -m "feat: add deterministic Fake Lebai client"
```

---

### Task 2: Run the full RealLebaiAdapter path as an explicit digital twin

**Files:**
- Create: `backend/app/digital_twin/runtime.py`
- Create: `backend/tests/digital_twin/test_runtime.py`
- Create: `config/fake-lebai.yaml`
- Create: `scripts/run_fake_lebai_stack.py`
- Modify: `backend/app/main.py`
- Modify: `backend/app/robots/lebai_adapter.py`
- Modify: `backend/app/schemas/messages.py`
- Modify: `backend/tests/contract/test_messages.py`
- Modify: `web/src/protocol/messages.ts`
- Modify: `web/tests/messages.test.ts`

**Interfaces:**
- Consumes: `DigitalTwinLebaiClient`, `Settings.load(path)`, `create_app()`, and `RealLebaiAdapter`.
- Produces:
  - `RuntimeBackend = Literal["SIMULATOR", "LEBAI", "LEBAI_FAKE"]`
  - `load_digital_twin_settings(path: Path = DIGITAL_TWIN_CONFIG) -> Settings`
  - `create_digital_twin_app(path: Path = DIGITAL_TWIN_CONFIG) -> FastAPI`
  - `create_app(*, settings: Settings | None = None, client_factory: ClientFactory = connect_real_client, backend_label: RuntimeBackend | None = None) -> FastAPI`
  - `RealLebaiAdapter.__init__` gains keyword-only `backend_label: Literal["LEBAI", "LEBAI_FAKE"] = "LEBAI"`; every existing constructor parameter remains unchanged.

- [ ] **Step 1: Write runtime-identity and no-real-connector tests**

```python
def test_digital_twin_runtime_promotes_only_the_in_memory_copy() -> None:
    source = Settings.load(DIGITAL_TWIN_CONFIG)
    runtime = load_digital_twin_settings()
    assert source.lebai is not None
    assert source.lebai.mode == "readonly"
    assert runtime.lebai is not None
    assert runtime.lebai.mode == "control"
    assert os.environ.get("VR4ARM_REAL_ROBOT_CONFIRM") is None


def test_digital_twin_app_reports_exact_fake_identity(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    real_connect = Mock(side_effect=AssertionError("real connector called"))
    monkeypatch.setattr(
        "app.robots.lebai_sdk_bridge.connect_real_client",
        real_connect,
    )
    monkeypatch.setattr(
        "app.main.build_recorder",
        lambda _settings: NoopRecorder(),
    )
    app = create_digital_twin_app()
    with TestClient(app) as client:
        health = client.get("/health").json()
    assert health["backend"] == "LEBAI_FAKE"
    assert health["real_robot_mode"] == "control"
    assert health["hardware_verified"] is False
    real_connect.assert_not_called()
```

- [ ] **Step 2: Run runtime tests and verify they fail**

Run:

```powershell
cd backend
python -m pytest tests/digital_twin/test_runtime.py -q
```

Expected: FAIL because the runtime module and Fake config do not exist.

- [ ] **Step 3: Add the safe readonly Fake source config**

Create `config/fake-lebai.yaml` with concrete finite values:

```yaml
backend: lebai
real_robot:
  mode: readonly
  ip: digital-twin
  expected_tcp: {x: 0.0, y: 0.0, z: 0.175, rz: 0.0, ry: 0.0, rx: 0.0}
  home_q: [0.0, -0.7853981633974483, 1.5707963267948966, -0.7853981633974483, 1.5707963267948966, -1.5707963267948966]
  soft_joint_min_rad: [-3.141592653589793, -3.9269908169872414, -1.5707963267948966, -3.9269908169872414, -1.5707963267948966, -4.71238898038469]
  soft_joint_max_rad: [3.141592653589793, 2.356194490192345, 4.71238898038469, 2.356194490192345, 4.71238898038469, 1.5707963267948966]
  joint_limit_margin_rad: 0.05
  startup_tcp_min_m: [-1.0, -1.0, -1.0]
  startup_tcp_max_m: [1.0, 1.0, 1.0]
  tcp_position_tolerance_m: 0.001
  tcp_rotation_tolerance_deg: 0.5
  control:
    loop_hz: 50
    state_hz: 25
    pvat_send_hz: 25
    pvat_horizon_s: 0.08
    max_tcp_speed_mps: 0.03
    max_tcp_rotation_radps: 0.25
    max_tcp_acceleration_mps2: 0.10
    max_tcp_angular_acceleration_radps2: 0.5
    max_joint_speed_radps: 0.15
    max_joint_acceleration_radps2: 0.5
    max_joint_step_rad: 0.01
    max_tcp_step_m: 0.002
    max_tcp_rotation_step_deg: 1.0
    max_relative_translation_m: 0.10
    max_relative_rotation_deg: 30
    translation_scale: 0.5
  gripper:
    max_force_percent: 30
    command_hz: 10
    open_amplitude_percent: 100
    closed_amplitude_percent: 0
```

The Fake client's `get_tcp()` returns this configured TCP exactly. The config remains readonly on disk so ordinary `create_app()` cannot use it for motion.

- [ ] **Step 4: Implement explicit in-memory Fake runtime promotion**

```python
DIGITAL_TWIN_CONFIG = (
    Path(__file__).resolve().parents[3] / "config" / "fake-lebai.yaml"
)


def load_digital_twin_settings(
    path: Path = DIGITAL_TWIN_CONFIG,
) -> Settings:
    settings = Settings.load(path)
    if settings.backend != "lebai" or settings.lebai is None:
        raise RuntimeError("digital_twin_requires_lebai_profile")
    if settings.lebai.mode != "readonly" or settings.lebai.ip != "digital-twin":
        raise RuntimeError("unsafe_digital_twin_source_profile")
    return replace(
        settings,
        lebai=replace(settings.lebai, mode="control"),
    )


def create_digital_twin_app(
    path: Path = DIGITAL_TWIN_CONFIG,
) -> FastAPI:
    settings = load_digital_twin_settings(path)
    assert settings.lebai is not None
    client = DigitalTwinLebaiClient.idle(settings.lebai)

    async def factory(_ip: str) -> LebaiClientProtocol:
        return client

    return create_app(
        settings=settings,
        client_factory=factory,
        backend_label="LEBAI_FAKE",
    )
```

- [ ] **Step 5: Add explicit backend labels through app and adapter**

Define:

```python
RuntimeBackend = Literal["SIMULATOR", "LEBAI", "LEBAI_FAKE"]
```

`create_app()` accepts `backend_label: RuntimeBackend | None = None`; the default remains derived from `settings.backend`. Pass `LEBAI` or `LEBAI_FAKE` into `RealLebaiAdapter`, and make `get_state()` publish that exact value. Add `hardware_verified: False` to `/health`; no software runtime sets it true.

- [ ] **Step 6: Update strict Python and TypeScript contracts**

Python:

```python
backend: Literal["SIMULATOR", "LEBAI", "LEBAI_FAKE"] | None = None
```

TypeScript:

```typescript
export type RuntimeBackend = 'SIMULATOR' | 'LEBAI' | 'LEBAI_FAKE';
```

Change the existing `RobotStateMessage.backend` declaration to
`backend?: RuntimeBackend | null`. Update `isRobotStateMessage()` to allow
only these three string values or null/absence. Add positive `LEBAI_FAKE` and
negative `LEBAI_MOCK` tests.

- [ ] **Step 7: Add the executable launcher**

```python
from app.digital_twin.runtime import create_digital_twin_app


def main() -> int:
    uvicorn.run(
        create_digital_twin_app(),
        host="127.0.0.1",
        port=8000,
        log_level="info",
    )
    return 0
```

The launcher accepts no real IP, no production config, and no confirmation phrase.

- [ ] **Step 8: Run backend and frontend contract tests**

Run:

```powershell
cd backend
python -m pytest tests/digital_twin/test_runtime.py tests/contract/test_messages.py tests/api/test_health.py -q
cd ..\web
npm.cmd test -- --run tests/messages.test.ts
```

Expected: zero failures.

- [ ] **Step 9: Commit the explicit Fake runtime**

```powershell
git add backend/app/digital_twin/runtime.py backend/tests/digital_twin/test_runtime.py config/fake-lebai.yaml scripts/run_fake_lebai_stack.py backend/app/main.py backend/app/robots/lebai_adapter.py backend/app/schemas/messages.py backend/tests/contract/test_messages.py web/src/protocol/messages.ts web/tests/messages.test.ts
git diff --cached --name-only
git commit -m "feat: run RealLebaiAdapter as a digital twin"
```

---

### Task 3: Expand the guarded onsite smoke tool into one-action subcommands

**Files:**
- Create: `backend/app/commissioning/actions.py`
- Create: `backend/tests/commissioning/__init__.py`
- Create: `backend/tests/commissioning/test_actions.py`
- Modify: `backend/app/commissioning/smoke.py`
- Modify: `backend/tests/scripts/test_real_robot_scripts.py`

**Interfaces:**
- Consumes: `RobotControl`, `Settings`, `VRFrame`, `Rotation`, `RobotStateMessage`, and existing commissioning recorder.
- Produces:
  - `TranslationAction(axis, distance_m)`
  - `RotationAction(axis, angle_deg)`
  - `GripperAction(target)`
  - `HomeAction`
  - `StopAction`
  - `SmokeAction` union
  - `SmokeOptions(config_path, action, confirmation)`
  - `SmokeResult.to_dict()`
  - `parse_smoke_args(argv)`
  - `run_smoke(options, client_factory)`

- [ ] **Step 1: Write exact signed-bound parser tests**

```python
ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / "config" / "fake-lebai.yaml"


@pytest.mark.parametrize("distance", [-0.005, 0.005])
def test_translate_accepts_signed_five_millimetres(distance: float) -> None:
    options = parse_smoke_args(
        [
            "translate",
            "--config", str(CONFIG),
            "--axis", "x",
            "--distance-m", str(distance),
            "--confirm", REAL_ROBOT_CONFIRMATION,
        ]
    )
    assert options.action == TranslationAction("x", distance)


@pytest.mark.parametrize("distance", [-0.0051, 0.0, 0.0051, math.nan])
def test_translate_rejects_out_of_bounds_values(distance: float) -> None:
    with pytest.raises(ValueError, match="smoke_translation_out_of_bounds"):
        parse_smoke_args(
            [
                "translate",
                "--config", str(CONFIG),
                "--axis", "x",
                "--distance-m", str(distance),
                "--confirm", REAL_ROBOT_CONFIRMATION,
            ]
        )


@pytest.mark.parametrize("angle", [-2.0, 2.0])
def test_rotate_accepts_signed_two_degrees(angle: float) -> None:
    options = parse_smoke_args(
        [
            "rotate",
            "--config", str(CONFIG),
            "--axis", "roll",
            "--angle-deg", str(angle),
            "--confirm", REAL_ROBOT_CONFIRMATION,
        ]
    )
    assert options.action == RotationAction("roll", angle)
```

- [ ] **Step 2: Run parser tests and verify the old parser rejects subcommands**

Run:

```powershell
cd backend
python -m pytest tests/commissioning/test_actions.py -q
```

Expected: FAIL because action dataclasses and subparsers do not exist.

- [ ] **Step 3: Define action and result types**

```python
TranslationAxis = Literal["x", "y", "z"]
RotationAxis = Literal["roll", "pitch", "yaw"]
GripperTarget = Literal["open", "close"]


@dataclass(frozen=True)
class TranslationAction:
    axis: TranslationAxis
    distance_m: float


@dataclass(frozen=True)
class RotationAction:
    axis: RotationAxis
    angle_deg: float


@dataclass(frozen=True)
class GripperAction:
    target: GripperTarget


@dataclass(frozen=True)
class HomeAction:
    pass


@dataclass(frozen=True)
class StopAction:
    pass


SmokeAction = (
    TranslationAction | RotationAction | GripperAction | HomeAction | StopAction
)


@dataclass(frozen=True)
class SmokeResult:
    action: str
    before: RobotStateMessage
    after: RobotStateMessage
    stable: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "action": self.action,
            "before": self.before.model_dump(mode="json"),
            "after": self.after.model_dump(mode="json"),
            "stable": self.stable,
        }
```

- [ ] **Step 4: Replace the flat parser with mutually exclusive subparsers**

Use `argparse` subcommands `translate`, `rotate`, `gripper`, `home`, and `stop`. Every subparser repeats required `--config` and `--confirm`, so no invocation can omit the safety inputs.

Validation:

```python
if not math.isfinite(distance) or distance == 0 or abs(distance) > 0.005:
    raise ValueError("smoke_translation_out_of_bounds")
if not math.isfinite(angle) or angle == 0 or abs(angle) > 2.0:
    raise ValueError("smoke_rotation_out_of_bounds")
if confirmation != REAL_ROBOT_CONFIRMATION:
    raise ValueError("smoke_confirmation_required")
```

- [ ] **Step 5: Implement exact controller-frame action mapping**

Keep all Cartesian and gripper actions inside `RobotControl`.

Translation controller offset:

```python
hand_distance = action.distance_m / settings.lebai.control.translation_scale
offset = [0.0, 0.0, 0.0]
offset[{"x": 0, "y": 1, "z": 2}[action.axis]] = hand_distance
```

Rotation controller quaternion:

```python
controller_axis = {
    "roll": "x",
    "pitch": "y",
    "yaw": "z",
}[action.axis]
q = tuple(
    float(value)
    for value in Rotation.from_euler(
        controller_axis,
        action.angle_deg,
        degrees=True,
    ).as_quat()
)
```

Extend `_frame()` with `q` and `trigger`. Arm with a released frame, capture at identity Grip, submit exactly one changed frame, then submit a released Grip frame.

Gripper uses `trigger=0.0` for open and `trigger=1.0` for close while Grip remains held at the captured pose.

Home publishes a released frame and calls `await control.home()`. Stop calls `await control.disarm()` and verifies stationary state.

- [ ] **Step 6: Verify action execution uses only one write category**

With the existing test Fake client, assert:

```python
translate_methods == {"move_pvat", "stop_move"}
rotate_methods == {"move_pvat", "stop_move"}
gripper_methods == {"set_claw", "stop_move"}
home_methods == {"stop_move", "movej"}
stop_methods == {"stop_move"}
```

Ignore repeated calls within the same method; reject any unexpected method including `start_sys`, `set_tcp`, `init_claw`, `speedl`, or a second action category.

- [ ] **Step 7: Add stable-state result and cleanup assertions**

After action execution, poll `backend.get_state()` until all absolute `actual_qd` values remain `<= 0.02 rad/s` for `300 ms`, bounded by the existing Home/stop timeouts. Serialize `SmokeResult`, print compact JSON, and always run `control.stop()`, `backend.disconnect()`, and `recorder.close()` in the existing guarded cleanup order.

- [ ] **Step 8: Run commissioning and adapter tests**

Run:

```powershell
cd backend
python -m pytest tests/commissioning/test_actions.py tests/scripts/test_real_robot_scripts.py tests/robots/test_lebai_adapter_control.py -q
```

Expected: zero failures.

- [ ] **Step 9: Commit guarded action subcommands**

```powershell
git add backend/app/commissioning/actions.py backend/app/commissioning/smoke.py backend/tests/commissioning/__init__.py backend/tests/commissioning/test_actions.py backend/tests/scripts/test_real_robot_scripts.py
git diff --cached --name-only
git commit -m "feat: add guarded LM3 commissioning actions"
```

---

### Task 4: Stream bounded diagnostics through the single-owner WebSocket

**Files:**
- Create: `backend/app/diagnostics/__init__.py`
- Create: `backend/app/diagnostics/store.py`
- Create: `backend/app/diagnostics/recorder.py`
- Create: `backend/tests/diagnostics/__init__.py`
- Create: `backend/tests/diagnostics/test_store.py`
- Create: `backend/tests/diagnostics/test_recorder.py`
- Create: `schemas/fixtures/diagnostics-valid.json`
- Modify: `backend/app/schemas/messages.py`
- Modify: `backend/tests/contract/test_messages.py`
- Modify: `backend/app/main.py`
- Modify: `backend/app/control/robot_control.py`
- Modify: `backend/app/api/teleop_ws.py`
- Modify: `backend/tests/api/test_teleop_ws.py`

**Interfaces:**
- Consumes: `RecorderSink`, existing `robot_kinematics`, `pvat_sent`, robot state, and the WebSocket owner session.
- Produces:
  - `DiagnosticEvent`
  - `DiagnosticsMessage`
  - `DiagnosticsStore(capacity=200, normal_interval_ns=200_000_000)`
  - `DiagnosticsStore.observe_event(event: object, server_mono_ns: int, *, critical: bool) -> None`
  - `DiagnosticsStore.observe_robot_state(state: RobotStateMessage, server_mono_ns: int) -> None`
  - `DiagnosticsStore.message(*, runtime: RuntimeBackend, hardware_verified: Literal[False], server_mono_ns: int, control_generation: int, log_session_dir: str | None) -> DiagnosticsMessage`
  - `DiagnosticsRecorder(primary, store)`
  - `RobotControl.control_generation`
  - `diagnostics_sender(websocket: WebSocket, control: RobotControl, store: DiagnosticsStore, runtime: RuntimeBackend, log_session_dir: str | None, send_lock: asyncio.Lock) -> None`

- [ ] **Step 1: Write bounded-store tests**

```python
def test_store_is_bounded_and_keeps_newest_events() -> None:
    store = DiagnosticsStore(capacity=3, normal_interval_ns=0)
    for index in range(5):
        store.observe_event(
            {"kind": "normal", "index": index},
            server_mono_ns=index,
            critical=False,
        )
    message = store.message(
        runtime="LEBAI_FAKE",
        hardware_verified=False,
        server_mono_ns=6,
        control_generation=2,
        log_session_dir=None,
    )
    assert [event.payload["index"] for event in message.recent_events] == [2, 3, 4]


def test_normal_events_are_coalesced_but_critical_events_are_retained() -> None:
    store = DiagnosticsStore(capacity=10, normal_interval_ns=200_000_000)
    store.observe_event({"kind": "robot_kinematics", "n": 1}, 0, critical=False)
    store.observe_event({"kind": "robot_kinematics", "n": 2}, 1, critical=False)
    store.observe_event({"kind": "stop_requested"}, 2, critical=True)
    message = store.message(
        runtime="LEBAI",
        hardware_verified=False,
        server_mono_ns=3,
        control_generation=4,
        log_session_dir="logs/commissioning/session",
    )
    assert [event.kind for event in message.recent_events] == [
        "robot_kinematics",
        "stop_requested",
    ]
```

- [ ] **Step 2: Run store tests and verify missing types**

Run:

```powershell
cd backend
python -m pytest tests/diagnostics/test_store.py -q
```

Expected: FAIL because the diagnostics package does not exist.

- [ ] **Step 3: Define strict diagnostics schemas**

In `schemas/messages.py`:

```python
RuntimeBackend = Literal["SIMULATOR", "LEBAI", "LEBAI_FAKE"]


class DiagnosticEvent(StrictMessage):
    event_id: int = Field(ge=1)
    server_mono_ns: int = Field(ge=0)
    kind: str = Field(min_length=1, max_length=64)
    critical: bool
    payload: dict[str, object]


class DiagnosticsMessage(StrictMessage):
    v: Literal[1] = 1
    type: Literal["diagnostics"] = "diagnostics"
    server_mono_ns: int = Field(ge=0)
    runtime: RuntimeBackend
    hardware_verified: Literal[False] = False
    control_generation: int = Field(ge=0)
    actual_qd: JointVector | None = None
    actual_qdd: JointVector | None = None
    target_q: JointVector | None = None
    target_qd: JointVector | None = None
    target_qdd: JointVector | None = None
    target_tcp: Pose | None = None
    sdk_latencies_ms: dict[str, float]
    pvat_send_hz: float | None = Field(default=None, ge=0)
    log_session_dir: str | None = None
    dropped_events: int = Field(ge=0)
    recent_events: tuple[DiagnosticEvent, ...]
```

Add validators that reject non-finite/negative SDK latency values and non-finite PVAT rate.

Create the canonical fixture:

```json
{
  "v": 1,
  "type": "diagnostics",
  "server_mono_ns": 1000000000,
  "runtime": "LEBAI_FAKE",
  "hardware_verified": false,
  "control_generation": 2,
  "actual_qd": [0, 0, 0, 0, 0, 0],
  "actual_qdd": [0, 0, 0, 0, 0, 0],
  "target_q": [0, -0.7853981633974483, 1.5707963267948966, -0.7853981633974483, 1.5707963267948966, -1.5707963267948966],
  "target_qd": [0, 0, 0, 0, 0, 0],
  "target_qdd": [0, 0, 0, 0, 0, 0],
  "target_tcp": {"p": [0.3, 0, 0.4], "q": [0, 0, 0, 1]},
  "sdk_latencies_ms": {"get_kin_data": 4.5, "move_pvat": 7.0},
  "pvat_send_hz": 25.0,
  "log_session_dir": null,
  "dropped_events": 0,
  "recent_events": [
    {
      "event_id": 1,
      "server_mono_ns": 900000000,
      "kind": "pvat_sent",
      "critical": false,
      "payload": {"command_id": 17}
    }
  ]
}
```

Load it in `test_messages.py` and require exact round-trip validation.

- [ ] **Step 4: Implement store observation and metrics**

`observe_event()`:

- extracts a nonempty `kind`;
- recognizes `robot_kinematics` and updates qd/qdd/target/TCP/SDK latency;
- recognizes `pvat_sent`, appends its timestamp to a two-second deque, and computes send rate from intervals;
- coalesces normal events with the same kind inside `normal_interval_ns`;
- never coalesces critical events;
- evicts the oldest event when capacity is reached and increments `dropped_events`;
- converts payloads through Pydantic/JSON-safe values and rejects NaN/Inf.

`message()` returns immutable copies and at most the newest 20 events:

```python
return DiagnosticsMessage(
    server_mono_ns=server_mono_ns,
    runtime=runtime,
    hardware_verified=False,
    control_generation=control_generation,
    actual_qd=self._actual_qd,
    actual_qdd=self._actual_qdd,
    target_q=self._target_q,
    target_qd=self._target_qd,
    target_qdd=self._target_qdd,
    target_tcp=self._target_tcp,
    sdk_latencies_ms=dict(self._sdk_latencies_ms),
    pvat_send_hz=self._pvat_send_hz(),
    log_session_dir=log_session_dir,
    dropped_events=self.dropped_events,
    recent_events=tuple(self._events)[-20:],
)
```

- [ ] **Step 5: Wrap the durable recorder without weakening failures**

`DiagnosticsRecorder` delegates first, then observes:

```python
async def write_event(self, event: object, server_mono_ns: int) -> None:
    await self.primary.write_event(event, server_mono_ns)
    self.store.observe_event(event, server_mono_ns, critical=False)

async def write_critical_event(
    self,
    kind: str,
    payload: object,
    server_mono_ns: int,
) -> None:
    await self.primary.write_critical_event(kind, payload, server_mono_ns)
    self.store.observe_event(
        {"kind": kind, "payload": payload},
        server_mono_ns,
        critical=True,
    )

async def write_robot_state(
    self,
    state: RobotStateMessage,
    server_mono_ns: int,
) -> None:
    await self.primary.write_robot_state(state, server_mono_ns)
    self.store.observe_robot_state(state, server_mono_ns)
```

`start()`, `write_vr_frame()`, `write_camera_frame()`, and `close()` delegate. A `RecorderUnavailable` from the primary must propagate unchanged; diagnostics never converts it into success.

Expose the durable session path without depending on a concrete primary recorder:

```python
@property
def log_session_dir(self) -> str | None:
    session_dir = getattr(self.primary, "session_dir", None)
    return None if session_dir is None else str(session_dir)
```

- [ ] **Step 6: Wire the store into app lifespan**

Create the store before the primary recorder, wrap it, and pass the wrapper to both `build_backend()` and `RobotControl`. Store on:

```python
app.state.diagnostics = diagnostics
app.state.runtime_backend = backend_label
app.state.log_session_dir = recorder.log_session_dir
```

Expose:

```python
@property
def control_generation(self) -> int:
    return self._control_generation
```

Do not expose a setter.

- [ ] **Step 7: Add a 5 Hz sender inside the coupled owner session**

```python
async def diagnostics_sender(
    websocket: WebSocket,
    control: RobotControl,
    store: DiagnosticsStore,
    runtime: RuntimeBackend,
    log_session_dir: str | None,
    send_lock: asyncio.Lock,
) -> None:
    while True:
        message = store.message(
            runtime=runtime,
            hardware_verified=False,
            server_mono_ns=control.clock.now_ns(),
            control_generation=control.control_generation,
            log_session_dir=log_session_dir,
        )
        await _send_json(websocket, send_lock, message.model_dump(mode="json"))
        await asyncio.sleep(0.2)
```

Create and clean up this task alongside the existing state sender and receiver. It is never started for a rejected second connection.

- [ ] **Step 8: Test owner-only delivery and task cleanup**

Extend the WebSocket test to assert:

```python
def receive_until_types(
    socket: WebSocketTestSession,
    required: set[str],
) -> dict[str, dict[str, object]]:
    received: dict[str, dict[str, object]] = {}
    for _ in range(20):
        payload = socket.receive_json()
        message_type = payload.get("type")
        if isinstance(message_type, str) and message_type in required:
            received[message_type] = payload
        if required <= received.keys():
            return received
    raise AssertionError(f"missing message types: {required - received.keys()}")


messages = receive_until_types(socket, {"robot_state", "diagnostics"})
assert messages["diagnostics"]["runtime"] == "LEBAI_FAKE"
assert messages["diagnostics"]["hardware_verified"] is False
assert app.state.teleop_owner is not None
```

Open a second socket, require exact `connection_rejected`, and verify it receives no diagnostics. After the owner disconnects, require `app.state.teleop_owner is None` and no live diagnostics task in `teleop_sender_tasks`.

- [ ] **Step 9: Run diagnostics, protocol, and WebSocket tests**

Run:

```powershell
cd backend
python -m pytest tests/diagnostics tests/contract/test_messages.py tests/api/test_teleop_ws.py tests/control/test_recording.py -q
```

Expected: zero failures.

- [ ] **Step 10: Commit bounded diagnostics**

```powershell
git add backend/app/diagnostics backend/tests/diagnostics schemas/fixtures/diagnostics-valid.json backend/app/schemas/messages.py backend/tests/contract/test_messages.py backend/app/main.py backend/app/control/robot_control.py backend/app/api/teleop_ws.py backend/tests/api/test_teleop_ws.py
git diff --cached --name-only
git commit -m "feat: stream bounded LM3 diagnostics"
```

---

### Task 5: Implement the A-layout PC diagnostics rail and compact Quest HUD

**Files:**
- Create: `web/src/ui/diagnosticsPanel.ts`
- Create: `web/tests/diagnosticsPanel.test.ts`
- Modify: `web/src/protocol/messages.ts`
- Modify: `web/tests/messages.test.ts`
- Modify: `web/src/transport/teleopSocket.ts`
- Modify: `web/tests/teleopSocket.test.ts`
- Modify: `web/src/ui/hud.ts`
- Modify: `web/tests/hud.test.ts`
- Modify: `web/src/ui/armPanel.ts`
- Modify: `web/tests/armPanel.test.ts`
- Modify: `web/src/scenes/vrSafetyPanel.ts`
- Modify: `web/tests/vrSafetyPanel.test.ts`
- Modify: `web/src/scenes/simulationScene.ts`
- Modify: `web/tests/simulationScene.test.ts`
- Modify: `web/src/main.ts`
- Modify: `web/src/styles.css`

**Interfaces:**
- Consumes: `RobotStateMessage`, `DiagnosticsMessage`, `LatencyTracker`, `ArmSafetySnapshot`, and authoritative actual TCP.
- Produces:
  - TypeScript `DiagnosticEvent` and `DiagnosticsMessage`
  - `isDiagnosticsMessage(value)`
  - `TeleopSocket` gains final optional callback `onDiagnostics: (message: DiagnosticsMessage) => void`
  - `DiagnosticsPanel.update(state, diagnostics, latency)`
  - `RobotRuntimeSummary`
  - `SimulationScene.setRuntimeSummary(summary)`
  - `VrSafetyPanel.update(state, controllerSupported, summary)`

- [ ] **Step 1: Write the strict frontend diagnostics guard**

```typescript
import validDiagnostics from '../../schemas/fixtures/diagnostics-valid.json';

expect(isDiagnosticsMessage(validDiagnostics)).toBe(true);
expect(isDiagnosticsMessage({...validDiagnostics, hardware_verified: true})).toBe(false);
expect(isDiagnosticsMessage({...validDiagnostics, runtime: 'LEBAI_MOCK'})).toBe(false);
expect(
  isDiagnosticsMessage({
    ...validDiagnostics,
    sdk_latencies_ms: {get_kin_data: Number.NaN},
  }),
).toBe(false);
```

- [ ] **Step 2: Run the message test and verify missing guard failure**

Run:

```powershell
cd web
npm.cmd test -- --run tests/messages.test.ts
```

Expected: FAIL because `isDiagnosticsMessage` is not exported.

- [ ] **Step 3: Implement exact TypeScript diagnostics types and guard**

Mirror the Python schema exactly. Reuse existing finite tuple, pose, plain-object, exact-key, and nonnegative-number helpers. `hardware_verified` must equal `false`, not merely be boolean.

Add `onDiagnostics` as the final optional constructor callback in `TeleopSocket`; dispatch only after `isDiagnosticsMessage(message)` succeeds.

- [ ] **Step 4: Write PC diagnostics rendering tests**

```typescript
it('renders authoritative real-path diagnostics and recent events', () => {
  const robotState = robotFixture as RobotStateMessage;
  const diagnostics = validDiagnostics as DiagnosticsMessage;
  const panel = new DiagnosticsPanel(root);
  panel.update(robotState, diagnostics, {currentMs: 18, p95Ms: 27});

  expect(root.textContent).toContain('LEBAI_FAKE');
  expect(root.textContent).toContain('DIGITAL TWIN');
  expect(root.textContent).toContain('TCP');
  expect(root.textContent).toContain('q1');
  expect(root.textContent).toContain('PVAT 25.0 Hz');
  expect(root.textContent).toContain('pvat_sent');
  expect(root.textContent).toContain('hardware_verified=false');
});

it('renders only the twenty events supplied by the bounded backend message', () => {
  const diagnosticsWithTwentyEvents: DiagnosticsMessage = {
    ...(validDiagnostics as DiagnosticsMessage),
    recent_events: Array.from({length: 20}, (_, index) => ({
      event_id: index + 1,
      server_mono_ns: 900_000_000 + index,
      kind: 'pvat_sent',
      critical: false,
      payload: {command_id: index},
    })),
  };
  const panel = new DiagnosticsPanel(root);
  panel.update(
    robotFixture as RobotStateMessage,
    diagnosticsWithTwentyEvents,
    {currentMs: null, p95Ms: null},
  );
  expect(root.querySelectorAll('[data-diagnostic-event]')).toHaveLength(20);
});
```

- [ ] **Step 5: Implement the PC A-layout panel**

`DiagnosticsPanel` owns focused render methods:

```typescript
export class DiagnosticsPanel {
  constructor(private readonly root: HTMLElement) {}

  update(
    state: RobotStateMessage,
    diagnostics: DiagnosticsMessage | null,
    latency: Readonly<{currentMs: number | null; p95Ms: number | null}>,
  ): void {
    this.renderIdentity(state, diagnostics);
    this.renderTcp(state.actual_tcp, diagnostics?.target_tcp ?? null);
    this.renderJoints(state.actual_q, diagnostics);
    this.renderLink(state, diagnostics, latency);
    this.renderSafety(state);
    this.renderEvents(diagnostics?.recent_events ?? []);
  }
}
```

Create sections for identity, actual/target TCP, six q/qd/qdd rows, WebSocket/SDK/IK/PVAT metrics, preflight/fault/constraint, gripper, log directory, dropped count, and recent events. Render text with `textContent`, never event payload HTML.

- [ ] **Step 6: Make existing desktop labels backend-aware**

Add to `Hud`:

```typescript
setRuntimeIdentity(
  backend: RuntimeBackend | null,
  realMode: 'readonly' | 'control' | null,
): void
```

Labels:

```typescript
SIMULATOR -> '仅仿真 · SIMULATOR'
LEBAI_FAKE -> '数字孪生 · LEBAI_FAKE'
LEBAI + readonly -> '真机只读 · LEBAI'
LEBAI + control -> '真机控制 · LEBAI'
```

Add `ArmPanel.setRuntimeIdentity()` so the arm button reads `解锁仿真`, `解锁数字孪生`, or `解锁真机`. This method changes copy only; it must not change `eligible`, `armed`, or pending state.

- [ ] **Step 7: Add the compact Quest runtime summary**

Define:

```typescript
export interface RobotRuntimeSummary {
  backend: RuntimeBackend | null;
  realRobotMode: 'readonly' | 'control' | null;
  actualTcp: Pose | null;
  gripper: number | null;
  latencyMs: number | null;
  hardwareVerified: false;
}
```

Add one compact `statusLine` to `VrSafetyPresentation`, for example:

```text
LEBAI · CONTROL · TCP 0.312/−0.041/0.428 · 18 ms
```

Keep existing title, actionable instruction, and footer. Update wording:

- Fake: `数字孪生已连接`
- Real ready: `真机已连接`
- `workspace_boundary`: `向反方向退回`
- `ik_boundary`: `保持 Grip，退回上一位置`
- stale: `机械臂已停止，请检查网络`
- stop failure: `保持安全距离，检查 L Master/急停`

Do not render q/qd/qdd or the event list in VR.

Add a scene regression test proving that a `LEBAI_FAKE` state updates the GLB
from authoritative `actual_q`, while changing diagnostics target joints alone
does not move the model.

- [ ] **Step 8: Join diagnostics in main without creating a second connection**

The single `TeleopSocket` receives both state and diagnostics. Cache the latest of each; update the desktop panel and Quest summary from the same callbacks. Do not add polling, SSE, a second WebSocket, or an observer connection.

- [ ] **Step 9: Implement responsive A-layout styles**

Keep the 3D viewport largest. On desktop, the diagnostics rail may scroll internally while the scene remains fixed. The Quest sprite position remains above and left of the main target area. Add specific selectors for diagnostics cards, joint table, bounded event stream, healthy/amber/red tones, and `LEBAI_FAKE` identity.

- [ ] **Step 10: Run focused frontend tests**

Run:

```powershell
cd web
npm.cmd test -- --run tests/messages.test.ts tests/teleopSocket.test.ts tests/diagnosticsPanel.test.ts tests/hud.test.ts tests/armPanel.test.ts tests/vrSafetyPanel.test.ts tests/simulationScene.test.ts
```

Expected: zero failures.

- [ ] **Step 11: Run the full frontend suite and build**

Run:

```powershell
cd web
npm.cmd test -- --run
npm.cmd run build
```

Expected: all tests pass and build exits `0`. Report any chunk-size warning as a warning, not a test failure.

- [ ] **Step 12: Commit the A-layout diagnostics UI**

```powershell
git add web/src/ui/diagnosticsPanel.ts web/tests/diagnosticsPanel.test.ts web/src/protocol/messages.ts web/tests/messages.test.ts web/src/transport/teleopSocket.ts web/tests/teleopSocket.test.ts web/src/ui/hud.ts web/tests/hud.test.ts web/src/ui/armPanel.ts web/tests/armPanel.test.ts web/src/scenes/vrSafetyPanel.ts web/tests/vrSafetyPanel.test.ts web/src/scenes/simulationScene.ts web/tests/simulationScene.test.ts web/src/main.ts web/src/styles.css
git diff --cached --name-only
git commit -m "feat: show LM3 diagnostics on PC and Quest"
```

---

### Task 6: Add deterministic full real-path motion and fault acceptance

**Files:**
- Create: `backend/app/acceptance/fake_lebai.py`
- Create: `backend/tests/acceptance/test_fake_lebai.py`

**Interfaces:**
- Consumes: `DigitalTwinLebaiClient`, `RealLebaiAdapter`, `RobotControl`, commissioning actions, Fake clock, and existing scenario result conventions.
- Produces:
  - `FakeLebaiScenarioResult`
  - `run_translation_scenario()`
  - `run_rotation_scenario()`
  - `run_gripper_home_stop_scenario()`
  - `run_fault_scenario()`
  - `run_fake_lebai_scenarios()`

- [ ] **Step 1: Write exact scenario-result tests**

```python
def test_fake_lebai_scenarios_cover_every_required_axis_and_action() -> None:
    results = asyncio.run(run_fake_lebai_scenarios())
    by_name = {result.name: result for result in results}
    assert set(by_name) == {
        "translation",
        "rotation",
        "gripper_home_stop",
        "faults",
    }
    assert by_name["translation"].metrics["axes"] == [
        "+x", "-x", "+y", "-y", "+z", "-z"
    ]
    assert by_name["rotation"].metrics["axes"] == [
        "+roll", "-roll", "+pitch", "-pitch", "+yaw", "-yaw"
    ]
    assert all(result.passed for result in results)
```

- [ ] **Step 2: Run the scenario test and verify the missing module failure**

Run:

```powershell
cd backend
python -m pytest tests/acceptance/test_fake_lebai.py -q
```

Expected: FAIL because the Fake-real-path acceptance module does not exist.

- [ ] **Step 3: Implement fresh isolated harness creation**

Each signed axis/action receives a fresh harness:

```python
class FakeClock:
    def __init__(self) -> None:
        self.value_ns = 0

    def now_ns(self) -> int:
        return self.value_ns

    async def sleep(self, seconds: float) -> None:
        self.value_ns += round(seconds * 1_000_000_000)


@dataclass
class Harness:
    settings: Settings
    clock: FakeClock
    client: DigitalTwinLebaiClient
    backend: RealLebaiAdapter
    control: RobotControl
    latest: LatestVRFrame


@asynccontextmanager
async def fake_real_harness() -> AsyncIterator[Harness]:
    settings = load_digital_twin_settings()
    assert settings.lebai is not None
    clock = FakeClock()
    client = DigitalTwinLebaiClient.idle(
        settings.lebai,
        clock=clock.now_ns,
    )

    async def factory(_ip: str) -> LebaiClientProtocol:
        return client

    recorder = NoopRecorder()
    backend = RealLebaiAdapter(
        settings.lebai,
        client_factory=factory,
        clock=clock.now_ns,
        sleep=clock.sleep,
        backend_label="LEBAI_FAKE",
    )
    # Construct RobotControl with production mapper and limiter.
    yield Harness(settings, clock, client, backend, control, latest)
```

Cleanup always stops `RobotControl` and disconnects the adapter.

- [ ] **Step 4: Implement signed translation and rotation scenarios**

For every signed translation, apply exactly `0.005 m`. For every signed rotation, apply exactly `2°`. Assert:

- at least one `move_pvat` call;
- actual TCP changes in the expected signed component or expected relative rotation;
- unchanged components remain within limiter tolerances;
- actual q is finite and within configured soft limits;
- no target is copied directly into actual state before virtual time advances;
- release produces `stop_move`;
- final mode is `DISARMED`.

- [ ] **Step 5: Implement gripper, Home, and stop scenario**

Require:

- open command uses configured open amplitude;
- close command uses configured closed amplitude and force `<= 30`;
- Home ends within configured tolerance of `home_q`;
- repeated stop is safe;
- no motion command occurs after the final stop.

- [ ] **Step 6: Implement fault-injection scenario**

Inject one condition per fresh harness:

```text
ik_failure      -> soft ik_boundary first, hard stop after persistent threshold
pvat_failure    -> hard fault and stop
disconnect      -> hard fault, no later write
stale_snapshot  -> advance the adapter clock beyond its state-age bound,
                   then require robot_state_stale and stop
stop_failure    -> stop_sys escalation and latched fault
```

Count injected/verified cases exactly and require no NaN/Inf in every published state.

- [ ] **Step 7: Run scenario and adjacent tests**

Run:

```powershell
cd backend
python -m pytest tests/acceptance/test_fake_lebai.py tests/acceptance/test_scenarios.py tests/robots/test_backend_contract.py tests/scripts/test_real_robot_scripts.py -q
```

Expected: zero failures.

- [ ] **Step 8: Commit full-path scenarios**

```powershell
git add backend/app/acceptance/fake_lebai.py backend/tests/acceptance/test_fake_lebai.py
git diff --cached --name-only
git commit -m "test: validate the Fake Lebai real-control path"
```

---

### Task 7: Build the Fake Lebai acceptance gate and operator documentation

**Files:**
- Create: `scripts/accept_fake_lebai.py`
- Create: `backend/tests/scripts/test_accept_fake_lebai.py`
- Create: `docs/fake-lebai-acceptance.md`
- Modify: `docs/real-robot-deployment.md`
- Modify: `.gitignore` only if `artifacts/acceptance/` is not already ignored.

**Interfaces:**
- Consumes: `run_fake_lebai_scenarios()`, full pytest, full Vitest, Vite build, Git, model hashes, and diagnostics contracts.
- Produces:
  - `FakeLebaiAcceptanceReport`
  - `run_gate(repo_root, command_runner=run_command)`
  - CLI JSON at `artifacts/acceptance/fake-lebai-latest.json`
  - nonzero exit on any software failure

- [ ] **Step 1: Write report-schema and failure-propagation tests**

Require:

```python
payload = report.to_dict()
assert payload["schema_version"] == 1
assert payload["runtime"] == "LEBAI_FAKE"
assert payload["digital_twin"] is True
assert payload["hardware_verified"] is False
assert payload["passed"] is True
assert payload["hardware_pending"] == [
    "sdk_connection",
    "tcp_home_joint_limits",
    "translation_direction",
    "rotation_direction",
    "gripper_direction_force",
    "pvat_tracking_latency",
    "stop_distance_estop",
    "lightweight_grasp_release",
]
```

With one fake command return code `1`, require `passed is False` and CLI exit `1`. Unit tests use a fake command runner and do not recursively launch npm/pytest.

- [ ] **Step 2: Run script tests and verify missing runner failure**

Run:

```powershell
cd backend
python -m pytest tests/scripts/test_accept_fake_lebai.py -q
```

Expected: FAIL because the gate script does not exist.

- [ ] **Step 3: Implement exact commands and provenance**

Run:

```python
[
    ("backend_tests", [sys.executable, "-m", "pytest", "-q"], repo / "backend"),
    ("frontend_tests", [npm, "test", "--", "--run"], repo / "web"),
    ("frontend_build", [npm, "run", "build"], repo / "web"),
]
```

Record:

- Git HEAD, dirty status, and correctly parsed porcelain paths;
- `config/lm3_visual_kinematics_v1.json` SHA-256;
- `web/public/models/Lebai_LM3.glb` SHA-256;
- `config/fake-lebai.yaml` SHA-256;
- exact scenario results;
- all eight hardware-pending categories;
- `hardware_verified: false`.

- [ ] **Step 4: Write the report atomically**

Use the existing acceptance directory:

```text
artifacts/acceptance/fake-lebai-latest.json
```

Write a sibling `.tmp`, close it, then `Path.replace()` it. Do not add generated JSON to Git.

- [ ] **Step 5: Write the offline operating document**

Document:

```powershell
python scripts/run_fake_lebai_stack.py
cd web
npm.cmd run dev -- --host 0.0.0.0
```

Document Quest use, `LEBAI_FAKE` identity, PC/Quest single-owner rule, diagnostics fields, injected faults, and:

```powershell
python scripts/accept_fake_lebai.py
```

State that the gate proves software-path behavior only and always reports `hardware_verified: false`.

- [ ] **Step 6: Update exact onsite smoke commands**

In `docs/real-robot-deployment.md`, replace the old flat smoke invocation with all mutually exclusive subcommands:

```powershell
python scripts/real_robot_smoke.py translate --config $env:VR4ARM_CONFIG --axis x --distance-m 0.005 --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
python scripts/real_robot_smoke.py rotate --config $env:VR4ARM_CONFIG --axis roll --angle-deg 2 --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
python scripts/real_robot_smoke.py gripper --config $env:VR4ARM_CONFIG --target open --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
python scripts/real_robot_smoke.py home --config $env:VR4ARM_CONFIG --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
python scripts/real_robot_smoke.py stop --config $env:VR4ARM_CONFIG --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
```

Repeat the signed negative-axis rule and the complete staged lab sequence. Do not include a real IP or local config values.

- [ ] **Step 7: Run focused gate tests**

Run:

```powershell
cd backend
python -m pytest tests/scripts/test_accept_fake_lebai.py tests/acceptance/test_fake_lebai.py -q
```

Expected: zero failures without a real SDK.

- [ ] **Step 8: Run the actual offline Fake Lebai gate once**

Run from repository root:

```powershell
python scripts/accept_fake_lebai.py
```

Expected:

- exit code `0`;
- full backend tests pass;
- full frontend tests pass;
- Vite build exits `0`;
- every Fake real-path scenario passes;
- report exists;
- report says `LEBAI_FAKE`, `digital_twin: true`, `hardware_verified: false`;
- no real SDK connector is called.

- [ ] **Step 9: Commit gate and documentation**

```powershell
git add scripts/accept_fake_lebai.py backend/tests/scripts/test_accept_fake_lebai.py docs/fake-lebai-acceptance.md docs/real-robot-deployment.md .gitignore
git diff --cached --name-only
git commit -m "feat: add Fake Lebai acceptance workflow"
```

If `.gitignore` is unchanged, omit it from `git add`.

---

### Task 8: Audit and freeze the real-motion-ready software milestone

**Files:**
- No required production changes.
- Modify only a failing implementation/test discovered by the commands below; preserve task-specific commit boundaries if a correction is required.

**Interfaces:**
- Consumes: final code, tests, documents, and generated report.
- Produces: fresh verification evidence and a local unmerged milestone.

- [ ] **Step 1: Run the prohibited-call audit**

Run:

```powershell
rg -n "start_sys|set_tcp|init_claw|estop\\(|speedl" backend/app scripts
rg -n "connect_real_client|move_pvat|stop_move|stop_sys|movej|set_claw" backend/app scripts
```

Expected:

- no production call to `start_sys`, `set_tcp`, `init_claw`, `estop`, or `speedl`;
- `connect_real_client` is absent from the digital-twin runtime execution path;
- `move_pvat`, `movej`, and `set_claw` remain behind RealLebaiAdapter control/preflight guards;
- `stop_move` is normal stop;
- `stop_sys` is failed-stop escalation.

- [ ] **Step 2: Run the full backend suite**

Run:

```powershell
cd backend
python -m pytest -q
```

Read the actual count and require zero failures.

- [ ] **Step 3: Run the full frontend suite**

Run:

```powershell
cd web
npm.cmd test -- --run
```

Read the actual count and require zero failures.

- [ ] **Step 4: Run the production build**

Run:

```powershell
cd web
npm.cmd run build
```

Require exit code `0`. Record any bundle-size warning without disguising it as failure or removing the warning threshold.

- [ ] **Step 5: Run both non-hardware acceptance gates**

Run:

```powershell
python scripts/accept_virtual_lm3.py
python scripts/accept_fake_lebai.py
```

Require both exit `0`, both `passed: true`, and both `hardware_verified: false`.

- [ ] **Step 6: Verify final report invariants**

Read `artifacts/acceptance/fake-lebai-latest.json` and require:

```python
assert report["runtime"] == "LEBAI_FAKE"
assert report["digital_twin"] is True
assert report["hardware_verified"] is False
assert len(report["hardware_pending"]) == 8
assert report["passed"] is True
assert all(result["passed"] for result in report["scenarios"])
```

Require all recorded SHA-256 values to match `^[0-9a-f]{64}$`.

- [ ] **Step 7: Check repository hygiene**

Run:

```powershell
git diff --check
git status --short
git log --oneline -12
git diff --name-only HEAD -- README.md backend/app/sim/ik.py docs/superpowers/plans/2026-07-22-tool-frame-and-ik-continuity.md docs/superpowers/specs/2026-07-22-tool-frame-and-ik-continuity-design.md
```

Expected: only the four known user-owned paths remain dirty. Generated reports and `.superpowers/` visual-companion files remain ignored.

- [ ] **Step 8: Report the frozen local milestone**

Report:

- every new local commit hash;
- actual backend and frontend counts;
- build result and warning;
- both acceptance report paths;
- the eight hardware-only pending checks;
- exact user-owned dirty files;
- branch and worktree path;
- `not merged`;
- `not pushed`.

Do not merge or push unless the user separately asks.

## Plan Self-Review

- **Spec coverage:** Tasks 1–2 cover the RealLebaiAdapter digital twin and explicit Fake identity; Task 3 covers every guarded onsite action; Task 4 covers bounded owner-only diagnostics; Task 5 covers the approved A layout and compact Quest HUD; Task 6 covers signed motion, gripper, Home, stop, and fault scenarios; Tasks 7–8 cover operator procedures, acceptance reporting, full verification, and the hardware trust boundary.
- **Scope:** The plan adds no camera, recording dataset, training export, ROS 2, MoveIt, new physics engine, arbitrary trajectory, real IP, or automated real-hardware call.
- **Type consistency:** `RuntimeBackend`, `DigitalTwinFaults`, `DigitalTwinLebaiClient`, action dataclasses, `DiagnosticEvent`, `DiagnosticsMessage`, `DiagnosticsStore`, and `RobotRuntimeSummary` are defined before their consumers and keep the same names throughout.
- **Safety consistency:** The versioned Fake profile is readonly on disk; only the dedicated injected-client runtime promotes its in-memory copy. Production control confirmation and preflight remain unchanged.
- **Single-owner consistency:** Diagnostics share the current owner's WebSocket; no observer endpoint or second connection is added.
- **No placeholders:** Every task names exact files, signatures, limits, commands, expected failures, passing commands, and commit boundaries.
- **Dirty-worktree safety:** Every commit uses explicit paths and excludes the four known user-owned paths.
