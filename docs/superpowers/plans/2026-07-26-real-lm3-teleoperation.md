# Real LM3 Teleoperation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an explicitly enabled, logged, PVAT-based real-robot backend for a Lebai LM3 with the stock LMG-90 gripper while preserving the simulator as the default and preventing ordinary tests from contacting hardware.

**Architecture:** Keep `RobotControl` as the only motion authority and implement a lazy-loaded `RealLebaiAdapter` behind the existing `RobotBackend` interface. The adapter uses the official asynchronous SDK, real TCP feedback, vendor inverse kinematics, a latest-wins PVAT pump, read-only/control runtime modes, stop verification, and a commissioning recorder. Real motion is impossible unless local configuration, an environment confirmation phrase, backend preflight, WebXR grip-release gating, and the existing single-controller ownership all agree.

**Tech Stack:** Python 3.11+, FastAPI, Pydantic 2, NumPy, SciPy Rotation, PyYAML, `lebai-sdk-asyncio` 0.3.x as an optional real-hardware dependency, pytest/pytest-asyncio, TypeScript/Vite/WebXR, JSONL.

## Global Constraints

- The approved design is `docs/superpowers/specs/2026-07-26-real-lm3-teleoperation-design.md`.
- Keep `backend: simulator` as the tracked default.
- Never import `lebai_sdk` while loading or running the simulator.
- Ordinary `pytest`, frontend tests, and build commands must never discover, connect to, start, stop, initialize, or move a real robot.
- Real SDK calls are allowed only from `RealLebaiAdapter` and its private collaborators.
- `RobotControl` remains the only component authorized to request robot motion.
- Do not automatically call `start_sys()`, `set_tcp()`, `set_payload()`, `init_claw(True)`, or `estop()`.
- Do not use simulator IK, simulator joint limits, or simulator self-collision proxies in the real-robot command path.
- Real motion requires both `real_robot.mode: control` and `VR4ARM_REAL_ROBOT_CONFIRM=I_UNDERSTAND_REAL_ROBOT_MOTION`.
- `LEBAI_READONLY` must never issue an SDK write: reject PVAT, Home, gripper, and every other motion request; its public stop/shutdown path is an idempotent local no-op because no motion could have been accepted.
- Initial control values are fixed at: control loop `50 Hz`, state polling `25 Hz`, PVAT send `25 Hz`, horizon `0.08 s`, TCP speed `0.03 m/s`, TCP angular speed `0.25 rad/s`, conservative TCP linear/angular acceleration `0.10 m/s²` / `0.5 rad/s²`, joint speed `0.15 rad/s`, joint acceleration `0.5 rad/s²`, translation scale `0.5`.
- The real path additionally enforces `2 mm`/cycle translation, `1°`/cycle rotation, `±0.10 m` per-axis displacement from the captured TCP zero, `±30°` rotation from that zero, and locally measured six-joint soft limits.
- Initial TCP tolerance is `0.001 m` and `0.5°`.
- Initial gripper force limit is `30%`; real open/closed amplitudes must be supplied by local configuration after the lab read-only test.
- No automatic reconnect may restore ACTIVE. A reconnect returns to read-only/disarmed preflight.
- Every stop invalidates queued or in-flight commands before calling the SDK.
- A real configuration file, IP, TCP, Home pose, confirmation phrase, and runtime logs stay out of Git.
- All Git commits are local; do not push.
- Do not add the deferred Gemini data path or detailed VR/desktop diagnostics panel in this plan.
- Do not claim the complete repository suite passes unless a fresh full run actually has zero failures.

## Known Test Baseline

Before Task 1, run these commands without editing anything:

```powershell
Set-Location backend
python -m pytest
Set-Location ..\web
npm.cmd test
```

Record the exact output in the execution commentary. On the clean committed simulator milestone, the last observed full runs had:

- backend collection errors from tests importing the removed Starlette 1.0 symbol `StarletteDeprecationWarning`;
- five frontend assertion failures whose exact-object expectations predate `thumbstickX`, left `grip`, and `headQ`.

Those are pre-existing baseline findings, not permission to weaken or rewrite tests. This plan adds focused real-robot tests and reports the full-suite baseline separately. If the baseline changes, investigate with `superpowers:systematic-debugging` before touching any test.

## File Structure

Create or modify these focused units:

- `backend/app/config.py`: parse simulator, read-only Lebai, and control Lebai settings without importing the SDK.
- `backend/app/robots/base.py`: backend preflight contract shared by simulator and real adapters.
- `backend/app/robots/lebai_sdk_bridge.py`: lazy SDK import, typed client protocol, async connection factory, and capability detection.
- `backend/app/robots/lebai_codec.py`: Euler ZYX, quaternion, TCP, joint-vector, robot-state, and emergency-stop conversions.
- `backend/app/robots/lebai_pvat.py`: pure IK-result validation and bounded PVAT point generation.
- `backend/app/robots/lebai_pump.py`: one-slot latest-wins command pump with generation invalidation.
- `backend/app/robots/lebai_adapter.py`: lifecycle, read-only enforcement, state cache, IK, PVAT, stop verification, Home, and gripper.
- `backend/app/robots/sim_adapter.py`: simulator preflight implementation only; no behavior changes.
- `backend/app/control/robot_control.py`: call backend preflight before arming and record critical control events.
- `backend/app/control/safety.py`: optional real-robot box/orientation envelope and hard per-cycle caps while preserving simulator defaults.
- `backend/app/recording/base.py`: recorder lifecycle and structured event contract.
- `backend/app/recording/noop.py`: no-op lifecycle implementation.
- `backend/app/recording/commissioning.py`: bounded asynchronous JSONL session recorder and summary.
- `backend/app/commissioning/preflight.py`: reusable read-only preflight workflow.
- `backend/app/commissioning/smoke.py`: reusable guarded 5 mm RobotControl workflow.
- `backend/app/main.py`: backend/recorder factories and lifecycle wiring.
- `backend/app/api/health.py`: dynamic backend, mode, preflight, and real-motion enable flags.
- `backend/app/schemas/messages.py` and `schemas/teleop-v1.json`: optional preflight/backend fields only; preserve protocol v1 compatibility.
- `config/real-robot.example.yaml`: non-runnable example with empty IP/TCP/Home and conservative limits.
- `scripts/real_robot_preflight.py`: read-only report generator.
- `scripts/real_robot_smoke.py`: explicit 5 mm RobotControl-driven commissioning move.
- `docs/real-robot-deployment.md`: server setup and run commands.
- `docs/real-robot-commissioning-report.md`: human checklist.
- `.gitignore`: runtime real configuration and commissioning logs.

Test files:

- `backend/tests/config/test_real_robot_config.py`
- `backend/tests/robots/fake_lebai.py`
- `backend/tests/robots/test_lebai_codec.py`
- `backend/tests/robots/test_lebai_sdk_bridge.py`
- `backend/tests/robots/test_lebai_pvat.py`
- `backend/tests/robots/test_lebai_pump.py`
- `backend/tests/robots/test_lebai_adapter_readonly.py`
- `backend/tests/robots/test_lebai_adapter_control.py`
- `backend/tests/recording/test_commissioning.py`
- existing `backend/tests/control/test_robot_control.py`
- existing `backend/tests/api/test_health.py`
- existing `backend/tests/api/test_teleop_ws.py`

---

### Task 1: Parse Explicit Real-Robot Modes Without Importing the SDK

**Files:**
- Create: `backend/tests/config/test_real_robot_config.py`
- Create: `config/real-robot.example.yaml`
- Modify: `backend/app/config.py:1-74`
- Modify: `backend/pyproject.toml:1-24`
- Modify: `config/default.yaml:1-22`
- Modify: `.gitignore`

**Interfaces:**
- Produces: `BackendKind = Literal["simulator", "lebai"]`
- Produces: `RealRobotMode = Literal["readonly", "control"]`
- Produces: immutable `TcpExpectation`, `LebaiControlSettings`, `LebaiGripperSettings`, and `LebaiSettings`
- Produces: `Settings.lebai: LebaiSettings | None`
- Consumes: `VR4ARM_CONFIG`
- Consumes: `VR4ARM_REAL_ROBOT_CONFIRM`

- [ ] **Step 1: Write configuration tests that fail against the simulator-only parser**

Create tests with exact assertions:

```python
from __future__ import annotations

import builtins
from pathlib import Path

import pytest

from app.config import Settings


def write_config(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "real.yaml"
    path.write_text(text, encoding="utf-8")
    return path


def test_readonly_lebai_config_loads_without_importing_sdk(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = write_config(
        tmp_path,
        """
backend: lebai
real_robot:
  mode: readonly
  ip: 192.168.10.20
  expected_tcp: {x: 0, y: 0, z: 0.175, rz: 0, ry: 0, rx: 0}
  home_q: [0, -1.0, 1.0, 0, 1.57, 0]
  soft_joint_min_rad: [-3.0, -2.5, -2.5, -3.0, -2.5, -6.0]
  soft_joint_max_rad: [3.0, 2.5, 2.5, 3.0, 2.5, 6.0]
  joint_limit_margin_rad: 0.05
  startup_tcp_min_m: [0.15, -0.30, 0.10]
  startup_tcp_max_m: [0.50, 0.30, 0.60]
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
""",
    )
    monkeypatch.setenv("VR4ARM_CONFIG", str(path))
    real_import = builtins.__import__

    def reject_sdk(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "lebai_sdk":
            raise AssertionError("config loading imported the real SDK")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", reject_sdk)
    settings = Settings.load()

    assert settings.backend == "lebai"
    assert settings.lebai is not None
    assert settings.lebai.mode == "readonly"
    assert settings.lebai.control.pvat_horizon_s == pytest.approx(0.08)


def test_control_mode_requires_exact_environment_confirmation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = write_config(tmp_path, MINIMAL_CONTROL_YAML)
    monkeypatch.setenv("VR4ARM_CONFIG", str(path))
    monkeypatch.delenv("VR4ARM_REAL_ROBOT_CONFIRM", raising=False)

    with pytest.raises(RuntimeError, match="^real_robot_confirmation_required$"):
        Settings.load()

    monkeypatch.setenv(
        "VR4ARM_REAL_ROBOT_CONFIRM",
        "I_UNDERSTAND_REAL_ROBOT_MOTION",
    )
    assert Settings.load().lebai.mode == "control"


def test_tracked_default_remains_simulator(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("VR4ARM_CONFIG", raising=False)
    settings = Settings.load()
    assert settings.backend == "simulator"
    assert settings.lebai is None
```

Define `MINIMAL_CONTROL_YAML` in the test file as the complete YAML from the first test with `mode: control`; do not use filesystem fixtures outside `tmp_path`.

- [ ] **Step 2: Run the new tests and verify RED**

Run:

```powershell
Set-Location backend
python -m pytest tests/config/test_real_robot_config.py -q
```

Expected: collection or assertion failure because `Settings` has no Lebai settings and rejects `backend: lebai`.

- [ ] **Step 3: Implement nested immutable configuration**

In `backend/app/config.py`, define:

```python
REAL_ROBOT_CONFIRMATION = "I_UNDERSTAND_REAL_ROBOT_MOTION"
BackendKind = Literal["simulator", "lebai"]
RealRobotMode = Literal["readonly", "control"]


@dataclass(frozen=True)
class TcpExpectation:
    x: float
    y: float
    z: float
    rz: float
    ry: float
    rx: float


@dataclass(frozen=True)
class LebaiControlSettings:
    loop_hz: int
    state_hz: int
    pvat_send_hz: int
    pvat_horizon_s: float
    max_tcp_speed_mps: float
    max_tcp_rotation_radps: float
    max_tcp_acceleration_mps2: float
    max_tcp_angular_acceleration_radps2: float
    max_joint_speed_radps: float
    max_joint_acceleration_radps2: float
    max_joint_step_rad: float
    max_tcp_step_m: float
    max_tcp_rotation_step_deg: float
    max_relative_translation_m: float
    max_relative_rotation_deg: float
    translation_scale: float


@dataclass(frozen=True)
class LebaiGripperSettings:
    max_force_percent: int
    command_hz: int
    open_amplitude_percent: int
    closed_amplitude_percent: int


@dataclass(frozen=True)
class LebaiSettings:
    mode: RealRobotMode
    ip: str
    expected_tcp: TcpExpectation
    home_q: tuple[float, float, float, float, float, float]
    soft_joint_min_rad: tuple[float, float, float, float, float, float]
    soft_joint_max_rad: tuple[float, float, float, float, float, float]
    joint_limit_margin_rad: float
    startup_tcp_min_m: tuple[float, float, float]
    startup_tcp_max_m: tuple[float, float, float]
    tcp_position_tolerance_m: float
    tcp_rotation_tolerance_deg: float
    control: LebaiControlSettings
    gripper: LebaiGripperSettings
```

Keep the existing flat simulator settings intact. Parse the nested block only when `backend == "lebai"`. Reject blank IP, non-six-element Home or soft-limit vectors, non-three-element startup TCP bounds, non-finite values, any lower bound greater than or equal to its upper bound, a margin that consumes either side of a joint range, Home outside the margin-adjusted soft limits, invalid percentages, `pvat_send_hz > loop_hz`, and `pvat_horizon_s <= 1 / pvat_send_hz`. Require the exact confirmation phrase only for `mode == "control"`.

Add an optional dependency group without installing it in simulator development:

```toml
real = [
  "lebai-sdk-asyncio>=0.3.7,<0.4",
]
```

Add these ignore entries:

```gitignore
config/real-robot.local.yaml
logs/commissioning/
```

Create `config/real-robot.example.yaml` with empty `ip`, `home_q: []`, `soft_joint_min_rad: []`, `soft_joint_max_rad: []`, `startup_tcp_min_m: []`, and `startup_tcp_max_m: []`, plus comments stating that L Master/onsite values are mandatory and the example is intentionally non-runnable. Keep `config/default.yaml` unchanged except for an explanatory comment that simulator is the tracked default.

- [ ] **Step 4: Run configuration and simulator health tests**

Run:

```powershell
Set-Location backend
python -m pytest tests/config/test_real_robot_config.py tests/api/test_health.py::test_health_is_simulator_only -q
```

Expected: the new configuration tests pass; if the existing health test cannot collect because of the known Starlette baseline, report it separately and do not alter it in this task.

- [ ] **Step 5: Commit only Task 1 files**

```powershell
git add backend/app/config.py backend/pyproject.toml backend/tests/config/test_real_robot_config.py config/default.yaml config/real-robot.example.yaml .gitignore
git commit -m "feat: configure explicit real robot modes"
```

### Task 2: Add a Lazy Async SDK Bridge and Exact Pose Codec

**Files:**
- Create: `backend/app/robots/lebai_sdk_bridge.py`
- Create: `backend/app/robots/lebai_codec.py`
- Create: `backend/tests/robots/fake_lebai.py`
- Create: `backend/tests/robots/test_lebai_sdk_bridge.py`
- Create: `backend/tests/robots/test_lebai_codec.py`

**Interfaces:**
- Produces: `LebaiClientProtocol`
- Produces: `async def connect_real_client(ip: str) -> LebaiClientProtocol`
- Produces: `SdkCapabilities`
- Produces: `pose_to_lebai(Pose) -> dict[str, float]`
- Produces: `pose_from_lebai(Mapping[str, object]) -> Pose`
- Produces: `joint_vector(value: object, field: str) -> JointVector`
- Produces: `map_robot_state(value: object) -> BackendState`
- Produces: `estop_fault(value: object) -> str | None`

- [ ] **Step 1: Write RED codec tests**

Test identity and a non-trivial intrinsic ZYX rotation:

```python
import math

import numpy as np
from scipy.spatial.transform import Rotation

from app.robots.lebai_codec import pose_from_lebai, pose_to_lebai
from app.schemas.messages import Pose


def test_lebai_pose_round_trip_uses_intrinsic_zyx() -> None:
    source = {
        "x": 0.1,
        "y": -0.2,
        "z": 0.3,
        "rz": math.radians(30),
        "ry": math.radians(-20),
        "rx": math.radians(10),
    }
    pose = pose_from_lebai(source)
    expected = Rotation.from_euler(
        "ZYX",
        [source["rz"], source["ry"], source["rx"]],
    ).as_quat()

    np.testing.assert_allclose(pose.q, expected, atol=1e-9)
    recovered = pose_to_lebai(pose)
    for key, value in source.items():
        assert recovered[key] == pytest.approx(value, abs=1e-9)


def test_lebai_pose_rejects_missing_and_non_finite_fields() -> None:
    with pytest.raises(BackendCommandError, match="^invalid_sdk_pose$"):
        pose_from_lebai({"x": 0, "y": 0, "z": float("nan")})
```

- [ ] **Step 2: Write RED bridge tests**

Use `monkeypatch` on `importlib.import_module`:

```python
@pytest.mark.asyncio
async def test_connect_real_client_initializes_and_awaits_async_connect(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = FakeLebaiModule()
    monkeypatch.setattr(importlib, "import_module", lambda name: module)

    client = await connect_real_client("192.168.10.20")

    assert module.init_count == 1
    assert module.connect_calls == [("192.168.10.20", False)]
    assert client is module.client


@pytest.mark.asyncio
async def test_missing_sdk_becomes_public_backend_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def missing(name: str):
        raise ModuleNotFoundError(name)

    monkeypatch.setattr(importlib, "import_module", missing)
    with pytest.raises(BackendCommandError, match="^lebai_sdk_unavailable$"):
        await connect_real_client("192.168.10.20")
```

`FakeLebaiModule.connect` and every method on `FakeLebaiClient` must be `async`. Store every call as plain serializable data for later assertions.

- [ ] **Step 3: Run Task 2 tests and verify RED**

```powershell
Set-Location backend
python -m pytest tests/robots/test_lebai_codec.py tests/robots/test_lebai_sdk_bridge.py -q
```

Expected: FAIL because both modules are absent.

- [ ] **Step 4: Implement the exact bridge and codec**

Define the protocol with the methods used in this plan:

```python
class LebaiClientProtocol(Protocol):
    async def is_connected(self) -> bool: ...
    async def get_robot_state(self) -> object: ...
    async def get_estop_reason(self) -> object: ...
    async def get_kin_data(self) -> dict[str, object]: ...
    async def get_tcp(self) -> dict[str, object]: ...
    async def get_claw(self) -> dict[str, object]: ...
    async def get_running_motion(self) -> object: ...
    async def kinematics_inverse(
        self, pose: dict[str, float], joints: list[float]
    ) -> object: ...
    async def move_pvat(
        self,
        p: list[float],
        v: list[float],
        a: list[float],
        t: float,
    ) -> object: ...
    async def stop_move(self) -> None: ...
    async def stop_sys(self) -> None: ...
    async def movej(
        self,
        p: list[float],
        a: float,
        v: float,
        t: float,
        r: float,
    ) -> object: ...
    async def get_motion_state(self, motion_id: object) -> object: ...
    async def set_claw(self, force: int, amplitude: int) -> None: ...
```

`connect_real_client` must import the distribution's module name `lebai_sdk`, call synchronous `lebai_sdk.init()`, and await `lebai_sdk.connect(ip, False)`. Do not call discovery or `start_sys()`.

Use `Rotation.from_euler("ZYX", [rz, ry, rx])` and `Rotation.from_quat(q).as_euler("ZYX")`. Normalize angles to finite floats and translate all malformed SDK payloads to stable `BackendCommandError` reason strings.

Map robot states by accepting the documented numeric codes and uppercase string names. Map `ERROR`, `ESTOP`, and `DISCONNECTED` to `BackendState.FAULT`; `MOVING` to `MOVING`; `IDLE` to `IDLE`; all transitional non-fault states to `HOLD`.

- [ ] **Step 5: Run Task 2 tests and commit**

```powershell
Set-Location backend
python -m pytest tests/robots/test_lebai_codec.py tests/robots/test_lebai_sdk_bridge.py -q
Set-Location ..
git add backend/app/robots/lebai_sdk_bridge.py backend/app/robots/lebai_codec.py backend/tests/robots
git commit -m "feat: add lazy Lebai SDK bridge"
```

Expected: all Task 2 tests pass with no installed real SDK.

### Task 3: Define Backend Preflight and Implement Read-Only Lebai State

**Files:**
- Modify: `backend/app/robots/base.py:1-43`
- Modify: `backend/app/robots/sim_adapter.py:15-147`
- Replace: `backend/app/robots/lebai_adapter.py`
- Create: `backend/tests/robots/test_lebai_adapter_readonly.py`
- Modify: `backend/tests/sim/test_adapters.py`

**Interfaces:**
- Produces:

```python
@dataclass(frozen=True)
class BackendPreflight:
    ready: bool
    reason: str | None
    robot_state: BackendState
    actual_tcp: Pose
    actual_q: JointVector
    tcp_matches: bool
    capabilities: tuple[str, ...]
```

- Extends `RobotBackend` with `async def preflight(self) -> BackendPreflight`
- Produces `RealLebaiAdapter(settings, client_factory=connect_real_client, clock=time.monotonic_ns)`
- Accepts optional `event_callback(event: dict[str, object], server_mono_ns: int) -> Awaitable[None]` for non-critical SDK/PVAT telemetry; it has no control authority

- [ ] **Step 1: Write read-only behavior tests**

Use `FakeLebaiClient` initialized with documented payloads:

```python
@pytest.mark.asyncio
async def test_readonly_preflight_reads_state_without_writes() -> None:
    client = FakeLebaiClient.idle(
        tcp={"x": 0, "y": 0, "z": 0.175, "rz": 0, "ry": 0, "rx": 0},
        q=[0, -1, 1, 0, 1.57, 0],
    )
    adapter = RealLebaiAdapter(
        readonly_settings(),
        client_factory=AsyncMock(return_value=client),
        clock=lambda: 123,
    )

    await adapter.connect()
    result = await adapter.preflight()

    assert result.ready is False
    assert result.reason == "real_robot_readonly"
    assert result.tcp_matches is True
    assert client.write_calls == []


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "operation",
    [
        lambda adapter: adapter.command_tcp(identity_pose(), 1),
        lambda adapter: adapter.set_gripper(0.5),
        lambda adapter: adapter.home(home_options(), lambda phase: None),
    ],
)
async def test_readonly_rejects_state_changes(operation) -> None:
    adapter, client = await connected_readonly_adapter()
    with pytest.raises(BackendCommandError, match="^real_robot_readonly$"):
        await operation(adapter)
    assert client.write_calls == []


@pytest.mark.asyncio
async def test_readonly_shutdown_stop_is_a_zero_write_local_noop() -> None:
    adapter, client = await connected_readonly_adapter()
    await adapter.stop(StopReason.SHUTDOWN)
    await adapter.disconnect()
    assert client.write_calls == []
```

Add cases for TCP mismatch, non-IDLE state, running motion (`None`/`0` are the only accepted no-motion forms after codec normalization), estop reason, non-finite kinematic data, actual joint position inside the configured margin, actual TCP inside the configured startup box, and disconnected SDK.

- [ ] **Step 2: Run and verify RED**

```powershell
Set-Location backend
python -m pytest tests/robots/test_lebai_adapter_readonly.py -q
```

Expected: FAIL because the placeholder adapter has no constructor or preflight.

- [ ] **Step 3: Implement the shared preflight contract**

Add `BackendPreflight` and `preflight()` to the protocol. `SimRobotAdapter.preflight()` returns `ready=True`, reason `None`, current simulator state, current TCP/Q, `tcp_matches=True`, and capabilities `("command_tcp", "home", "gripper")`.

Implement `RealLebaiAdapter.connect()` to:

1. call only `client_factory(ip)`;
2. verify `await client.is_connected()`;
3. fetch initial state through one private `_read_snapshot()` method;
4. never call `start_sys()` or any write method.

`_read_snapshot()` calls `get_robot_state`, `get_estop_reason`, `get_kin_data`, `get_tcp`, `get_claw`, and `get_running_motion` serially under one `asyncio.Lock`. Normalize documented actual and target `q/qd/qdd`, actual/target TCP, flange pose, state, gripper, and running-motion data into the private snapshot. Record per-call monotonic latency; never store raw exception text.

`preflight()` checks exact mode, IDLE, no running motion, estop reason zero, TCP tolerances, finite six-element joint vectors, actual joints inside `soft_joint_[min,max] ± margin`, actual TCP inside `startup_tcp_[min,max]`, and capability presence. Compare TCP translation by Euclidean distance and orientation by quaternion angular distance so Euler wrapping at `±π` does not create a false mismatch. Read-only always returns `ready=False` with `real_robot_readonly` after reporting the other diagnostic fields.

`get_state()` converts the cached/fresh snapshot into `RobotStateMessage`; map gripper amplitude to normalized `[0, 1]` using configured open/closed values without writing.

After a successful refresh, emit one normal `robot_kinematics` event containing the normalized actual/target q/qd/qdd, actual/target TCP, state age, SDK call latencies, and current gripper. The event callback may drop/downsample normal data in the recorder, but a callback failure must surface as `recording_unavailable`.

`stop()` and `disconnect()` in read-only mode cancel/discard only local state and perform zero SDK writes. If the mode is control and the adapter had accepted motion, `disconnect()` calls the verified stop path before discarding the client.

- [ ] **Step 4: Run adapter and simulator contract tests**

```powershell
Set-Location backend
python -m pytest tests/robots/test_lebai_adapter_readonly.py tests/sim/test_adapters.py -q
```

Expected: all selected tests pass.

- [ ] **Step 5: Commit**

```powershell
git add backend/app/robots/base.py backend/app/robots/sim_adapter.py backend/app/robots/lebai_adapter.py backend/tests/robots/test_lebai_adapter_readonly.py backend/tests/sim/test_adapters.py
git commit -m "feat: add read-only Lebai preflight"
```

### Task 4: Build a Pure, Bounded PVAT Planner

**Files:**
- Create: `backend/app/robots/lebai_pvat.py`
- Create: `backend/tests/robots/test_lebai_pvat.py`

**Interfaces:**
- Produces:

```python
@dataclass(frozen=True)
class PvatLimits:
    horizon_s: float
    max_joint_speed_radps: float
    max_joint_acceleration_radps2: float
    max_joint_step_rad: float
    soft_joint_min_rad: JointVector
    soft_joint_max_rad: JointVector


@dataclass(frozen=True)
class PvatPoint:
    q: JointVector
    qd: JointVector
    qdd: JointVector
    horizon_s: float


def build_pvat_point(
    solution_q: object,
    actual_q: JointVector,
    actual_qd: JointVector,
    previous_qd: JointVector | None,
    limits: PvatLimits,
) -> PvatPoint
```

- Raises stable `BackendCommandError` reasons: `ik_invalid`, `ik_joint_limit`, `ik_joint_jump`, `joint_speed_limit`

- [ ] **Step 1: Write RED tests for valid motion and every rejection**

```python
def test_small_continuous_solution_produces_consistent_pvat() -> None:
    point = build_pvat_point(
        solution_q=[0.0032, 0, 0, 0, 0, 0],
        actual_q=(0, 0, 0, 0, 0, 0),
        actual_qd=(0, 0, 0, 0, 0, 0),
        previous_qd=None,
        limits=PvatLimits(
            horizon_s=0.08,
            max_joint_speed_radps=0.15,
            max_joint_acceleration_radps2=0.5,
            max_joint_step_rad=0.01,
            soft_joint_min_rad=(-3, -2.5, -2.5, -3, -2.5, -6),
            soft_joint_max_rad=(3, 2.5, 2.5, 3, 2.5, 6),
        ),
    )
    assert point.q[0] == pytest.approx(0.0032)
    assert point.qd[0] == pytest.approx(0.04)
    assert point.qdd[0] == pytest.approx(0.5)


@pytest.mark.parametrize(
    ("solution", "reason"),
    [
        ([0, 0, 0, 0, 0], "ik_invalid"),
        ([float("nan"), 0, 0, 0, 0, 0], "ik_invalid"),
        ([3.01, 0, 0, 0, 0, 0], "ik_joint_limit"),
        ([0.02, 0, 0, 0, 0, 0], "ik_joint_jump"),
    ],
)
def test_invalid_or_discontinuous_ik_is_rejected(solution, reason) -> None:
    with pytest.raises(BackendCommandError, match=f"^{reason}$"):
        build_pvat_point(
            solution,
            actual_q=(0, 0, 0, 0, 0, 0),
            actual_qd=(0, 0, 0, 0, 0, 0),
            previous_qd=None,
            limits=LIMITS,
        )
```

Add exact tests proving the returned q/qd/qdd never exceed the configured values and that acceleration is derived from actual/previous speed without silently clipping an unsafe IK jump.
Also reject a non-finite or already-over-limit `actual_qd`/`previous_qd` as `joint_speed_limit`; acceleration itself is shortened to the configured bound rather than faulted.

- [ ] **Step 2: Run and verify RED**

```powershell
Set-Location backend
python -m pytest tests/robots/test_lebai_pvat.py -q
```

Expected: FAIL because `lebai_pvat.py` does not exist.

- [ ] **Step 3: Implement minimal pure math**

Construct `PvatLimits` from the margin-adjusted bounds (`configured_min + margin`, `configured_max - margin`). Convert every vector and both effective limit vectors to finite NumPy arrays of shape `(6,)`. Validate `min < max`, then compute:

```python
delta_q = solution - actual
if np.any(solution < joint_min) or np.any(solution > joint_max):
    raise BackendCommandError("ik_joint_limit")
if np.max(np.abs(delta_q)) > limits.max_joint_step_rad:
    raise BackendCommandError("ik_joint_jump")

qd = delta_q / limits.horizon_s
if np.max(np.abs(qd)) > limits.max_joint_speed_radps:
    raise BackendCommandError("joint_speed_limit")

reference_qd = np.asarray(
    previous_qd if previous_qd is not None else actual_qd,
    dtype=float,
)
qdd = (qd - reference_qd) / limits.horizon_s
if np.max(np.abs(qdd)) > limits.max_joint_acceleration_radps2:
    bounded_delta = np.clip(
        qd - reference_qd,
        -limits.max_joint_acceleration_radps2 * limits.horizon_s,
        limits.max_joint_acceleration_radps2 * limits.horizon_s,
    )
    qd = reference_qd + bounded_delta
    qdd = bounded_delta / limits.horizon_s
    solution = actual + qd * limits.horizon_s
```

Reject unsafe positional jumps; acceleration limiting may shorten the accepted target because that is a safer continuous trajectory. Return immutable six-tuples.

- [ ] **Step 4: Run Task 4 tests**

```powershell
python -m pytest tests/robots/test_lebai_pvat.py -q
```

Expected: all Task 4 tests pass.

- [ ] **Step 5: Commit**

```powershell
git add backend/app/robots/lebai_pvat.py backend/tests/robots/test_lebai_pvat.py
git commit -m "feat: bound real robot PVAT points"
```

### Task 5: Implement the Latest-Wins PVAT Pump

**Files:**
- Create: `backend/app/robots/lebai_pump.py`
- Create: `backend/tests/robots/test_lebai_pump.py`

**Interfaces:**
- Produces `PvatRequest(target: Pose, command_id: int, generation: int)`
- Produces `PvatPump.submit(target: Pose, command_id: int) -> None`
- Produces `PvatPump.invalidate() -> int`
- Produces `PvatPump.start()`, `PvatPump.stop()`, `PvatPump.fault`
- Consumes one async handler:

```python
Callable[[PvatRequest], Awaitable[None]]
```

- [ ] **Step 1: Write RED concurrency tests**

```python
@pytest.mark.asyncio
async def test_latest_request_replaces_unsent_request() -> None:
    gate = asyncio.Event()
    handled: list[PvatRequest] = []

    async def handler(request: PvatRequest) -> None:
        await gate.wait()
        handled.append(request)

    pump = PvatPump(handler, period_s=0.04)
    await pump.start()
    pump.submit(identity_pose(), 1)
    pump.submit(offset_pose(0.001), 2)
    gate.set()
    await wait_until(lambda: len(handled) == 1)
    await pump.stop()

    assert handled[0].command_id == 2


@pytest.mark.asyncio
async def test_invalidate_blocks_delayed_generation() -> None:
    started = asyncio.Event()
    release = asyncio.Event()
    sent: list[int] = []

    async def handler(request: PvatRequest) -> None:
        started.set()
        await release.wait()
        if pump.is_current(request.generation):
            sent.append(request.command_id)

    pump = PvatPump(handler, period_s=0.04)
    await pump.start()
    pump.submit(identity_pose(), 1)
    await started.wait()
    pump.invalidate()
    release.set()
    await asyncio.sleep(0)
    await pump.stop()

    assert sent == []
```

Add tests for handler exception becoming a stable pump fault and `stop()` leaving no task or queued request.

- [ ] **Step 2: Run and verify RED**

```powershell
Set-Location backend
python -m pytest tests/robots/test_lebai_pump.py -q
```

Expected: FAIL because the pump is absent.

- [ ] **Step 3: Implement a one-slot condition-based pump**

Use an `asyncio.Condition`, one `PvatRequest | None`, a monotonic generation integer, and one task. `submit()` replaces the pending request. `invalidate()` increments generation and clears pending. The handler receives a request only if its generation remains current immediately before dispatch. Never use an unbounded `asyncio.Queue`.

The period is `1 / pvat_send_hz`. Use deadline-based scheduling rather than chained fixed sleeps:

```python
next_deadline = loop.time()
while self._running:
    request = await self._take_latest()
    next_deadline = max(next_deadline + self.period_s, loop.time())
    if self.is_current(request.generation):
        await self._handler(request)
    await asyncio.sleep(max(0.0, next_deadline - loop.time()))
```

If the handler raises, store `BackendCommandError("pvat_pump_failed")`, invalidate, clear pending, and stop the task. Do not auto-restart.

- [ ] **Step 4: Run and commit**

```powershell
python -m pytest tests/robots/test_lebai_pump.py -q
Set-Location ..
git add backend/app/robots/lebai_pump.py backend/tests/robots/test_lebai_pump.py
git commit -m "feat: add latest-wins PVAT pump"
```

### Task 6: Connect Vendor IK to PVAT in Control Mode

**Files:**
- Modify: `backend/app/robots/lebai_adapter.py`
- Create: `backend/tests/robots/test_lebai_adapter_control.py`

**Interfaces:**
- Consumes: `pose_to_lebai`, `build_pvat_point`, `PvatPump`
- Implements: `RealLebaiAdapter.command_tcp(target: Pose, command_id: int) -> None`
- Maintains: latest `actual_q`, `actual_qd`, previous sent `qd`, last acknowledged command ID

- [ ] **Step 1: Write RED control tests**

```python
@pytest.mark.asyncio
async def test_control_command_uses_vendor_ik_and_pvat_in_documented_order() -> None:
    adapter, client = await connected_control_adapter()
    target = Pose(p=(0.30, 0.0, 0.40), q=(0, 0, 0, 1))
    client.ik_result = [0.004, -1, 1, 0, 1.57, 0]

    await adapter.command_tcp(target, command_id=7)
    await client.wait_for_write("move_pvat")

    assert client.ik_calls == [
        (pose_to_lebai(target), [0, -1, 1, 0, 1.57, 0])
    ]
    method, p, v, a, horizon = client.write_calls[-1]
    assert method == "move_pvat"
    assert horizon == pytest.approx(0.08)
    assert len(p) == len(v) == len(a) == 6


@pytest.mark.asyncio
async def test_adapter_never_calls_pvat_after_stop_invalidates_delayed_ik() -> None:
    adapter, client = await connected_control_adapter(block_ik=True)
    await adapter.command_tcp(identity_pose(), 9)
    await client.ik_started.wait()

    stop_task = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
    client.release_ik.set()
    await stop_task

    assert [call[0] for call in client.write_calls].count("move_pvat") == 0
    assert "stop_move" in [call[0] for call in client.write_calls]
```

Add tests for read-only rejection, invalid IK, joint soft-limit rejection, joint jump, state cache staleness, and pump fault surfacing through `get_state()`.

Add an explicit transient/persistent boundary test:

```python
@pytest.mark.asyncio
async def test_one_ik_miss_holds_but_persistent_misses_fault_the_pump() -> None:
    adapter, client = await connected_control_adapter()
    client.ik_results.extend([None, valid_solution(), None, None, None, None, None])

    await send_and_wait(adapter, command_id=1)
    assert adapter.constraint == "ik_boundary"
    assert adapter.pump_fault is None

    await send_and_wait(adapter, command_id=2)
    assert adapter.constraint is None

    for command_id in range(3, 8):
        await send_and_wait(adapter, command_id=command_id)
    assert str(adapter.pump_fault) == "ik_failure_persistent"
```

- [ ] **Step 2: Run and verify RED**

```powershell
Set-Location backend
python -m pytest tests/robots/test_lebai_adapter_control.py -q
```

Expected: FAIL because control commands still raise `real_robot_disabled` or `real_robot_readonly`.

- [ ] **Step 3: Implement the control handler**

At connect time in control mode, start the PVAT pump only after `preflight().ready` is true. `command_tcp()` performs no fresh SDK calls; it checks:

1. mode is control;
2. the latest cached preflight/state remains ready;
3. pump has no fault;
4. cached state age is within one state period plus `40 ms`;
5. target pose is finite.

It then submits to the latest slot and returns without awaiting SDK motion.

Coalesce state refreshes: `get_state()` refreshes only when the cache exceeds one state period, while the 25 Hz pump handler forces the one state refresh used for IK. This prevents duplicate SDK polling during ACTIVE.

The pump handler:

```python
async def _send_target(self, request: PvatRequest) -> None:
    snapshot = await self._read_snapshot()
    solution = await self._client.kinematics_inverse(
        pose_to_lebai(request.target),
        list(snapshot.actual_q),
    )
    point = build_pvat_point(
        solution_q=solution,
        actual_q=snapshot.actual_q,
        actual_qd=snapshot.actual_qd,
        previous_qd=self._previous_sent_qd,
        limits=self._pvat_limits,
    )
    if not self._pump.is_current(request.generation):
        return
    await self._client.move_pvat(
        list(point.q),
        list(point.qd),
        list(point.qdd),
        point.horizon_s,
    )
    await self._emit_event(
        {
            "kind": "pvat_sent",
            "command_id": request.command_id,
            "target_tcp": request.target.model_dump(),
            "p": list(point.q),
            "v": list(point.qd),
            "a": list(point.qdd),
            "horizon_s": point.horizon_s,
            "sdk_latency_ms": pvat_latency_ms,
        }
    )
    self._previous_sent_qd = point.qd
    self._command_id = request.command_id
```

Treat SDK “no IK solution”, `ik_joint_limit`, `ik_joint_jump`, and `joint_speed_limit` as soft constraints for one frame: do not enqueue PVAT, preserve the last safe command, publish the constraint on the next `command_tcp()`, and allow a later valid target to clear it. Five consecutive failed 25 Hz attempts (`200 ms`) latch `ik_failure_persistent`, invalidate the pump, and surface a hard backend fault so `RobotControl` stops. A valid PVAT resets the counter.

Serialize all client methods under the adapter's SDK lock. Use `asyncio.wait_for` with operation-specific limits: connect `3 s`, state/IK `0.20 s`, PVAT call `0.06 s`. Convert timeouts to `sdk_timeout:<operation>`.

- [ ] **Step 4: Run all real-adapter focused tests**

```powershell
python -m pytest tests/robots/test_lebai_adapter_readonly.py tests/robots/test_lebai_adapter_control.py tests/robots/test_lebai_pump.py tests/robots/test_lebai_pvat.py -q
```

Expected: all selected tests pass and the fake client's network/discovery counters remain zero.

- [ ] **Step 5: Commit**

```powershell
git add backend/app/robots/lebai_adapter.py backend/tests/robots/test_lebai_adapter_control.py
git commit -m "feat: drive LM3 targets through vendor PVAT"
```

### Task 7: Verify Stops, Escalate Failed Stops, and Implement Explicit Home/Gripper

**Files:**
- Modify: `backend/app/robots/lebai_adapter.py`
- Modify: `backend/tests/robots/test_lebai_adapter_control.py`

**Interfaces:**
- Implements routine `stop_move()` verification
- Escalates to `stop_sys()` only when connected control mode remains moving after `0.5 s`
- Implements configured `movej` Home
- Maps normalized gripper input through configured open/closed amplitudes

- [ ] **Step 1: Write RED stop-verification tests**

```python
@pytest.mark.asyncio
async def test_stop_clears_pvat_then_confirms_300_ms_stationary() -> None:
    adapter, client, clock = await connected_control_adapter_with_clock()
    client.queue_joint_speeds(
        [0.1] * 6,
        [0.0] * 6,
        [0.0] * 6,
        [0.0] * 6,
    )

    await adapter.stop(StopReason.GRIP_RELEASED)

    assert client.write_calls[0][0] == "stop_move"
    assert "stop_sys" not in [call[0] for call in client.write_calls]
    assert clock.elapsed_s >= 0.3


@pytest.mark.asyncio
async def test_failed_stop_escalates_once_and_latches_fault() -> None:
    adapter, client, _clock = await connected_control_adapter_with_clock()
    client.repeat_joint_speed([0.1] * 6)

    with pytest.raises(BackendCommandError, match="^stop_incomplete$"):
        await adapter.stop(StopReason.FAULT)

    assert [call[0] for call in client.write_calls].count("stop_sys") == 1
    state = await adapter.get_state()
    assert state.robot_state is BackendState.FAULT
```

Use a fake sleeper/clock so the test has no real 500 ms delay.

- [ ] **Step 2: Write RED Home and gripper tests**

```python
@pytest.mark.asyncio
async def test_home_uses_only_configured_joint_pose() -> None:
    adapter, client = await connected_control_adapter()
    phases: list[str] = []

    await adapter.home(HOME_OPTIONS, phases.append)

    movej = next(call for call in client.write_calls if call[0] == "movej")
    assert movej[1] == list(adapter.settings.home_q)
    assert movej[2] == pytest.approx(
        adapter.settings.control.max_joint_acceleration_radps2
    )
    assert movej[3] == pytest.approx(HOME_OPTIONS.max_speed_radps)
    assert movej[4:] == (0.0, 0.0)
    assert phases == ["homing", "stabilizing"]


@pytest.mark.asyncio
async def test_gripper_force_and_amplitude_are_config_bounded() -> None:
    adapter, client = await connected_control_adapter()
    await adapter.set_gripper(0.25)
    assert client.write_calls[-1] == ("set_claw", 30, 75)
```

For `open=100`, `closed=0`, normalized `0.25` maps to `75`. Add tests for missing Home, non-IDLE Home, read-only writes, clamp at `[0, 1]`, and no forced `init_claw`.

- [ ] **Step 3: Run and verify RED**

```powershell
Set-Location backend
python -m pytest tests/robots/test_lebai_adapter_control.py -q
```

Expected: new stop/Home/gripper tests fail.

- [ ] **Step 4: Implement stop, Home, and gripper exactly**

Routine stop:

```python
self._pump.invalidate()
self._previous_sent_qd = None
await self._sdk_call("stop_move", self._client.stop_move(), timeout=0.20)
```

Poll `get_kin_data()` at `50 Hz`. Require every absolute joint speed below `home_options.velocity_tolerance_radps` continuously for `0.30 s`. At `0.50 s`, if still moving and still connected, call `stop_sys()` exactly once, latch `stop_incomplete`, and raise.

Home checks configured six-joint Home, control mode, IDLE, no running motion, and no active pump request. Call:

```python
motion_id = await client.movej(
    list(settings.home_q),
    settings.control.max_joint_acceleration_radps2,
    options.max_speed_radps,
    0.0,
    0.0,
)
```

Poll motion state and actual q/qd until position, speed, and stable-time criteria are met or timeout. Call phase callbacks on transition only.

Gripper mapping:

```python
amplitude = round(
    open_percent
    + min(1.0, max(0.0, value)) * (closed_percent - open_percent)
)
await client.set_claw(max_force_percent, amplitude)
```

- [ ] **Step 5: Run focused adapter tests and commit**

```powershell
python -m pytest tests/robots/test_lebai_adapter_control.py tests/robots/test_lebai_adapter_readonly.py -q
Set-Location ..
git add backend/app/robots/lebai_adapter.py backend/tests/robots/test_lebai_adapter_control.py
git commit -m "feat: verify LM3 stops home and gripper"
```

### Task 8: Add a Bounded Commissioning Recorder

**Files:**
- Modify: `backend/app/recording/base.py`
- Modify: `backend/app/recording/noop.py`
- Create: `backend/app/recording/commissioning.py`
- Create: `backend/tests/recording/test_commissioning.py`
- Modify: `backend/tests/control/test_recording.py`
- Modify: `.gitignore`

**Interfaces:**
- Extends recorder lifecycle with `start()` and `close()`
- Produces `CommissioningRecorder(root: Path, metadata: dict[str, object], capacity: int = 2048, critical_capacity: int = 128)`
- Produces `write_critical_event(kind, payload, server_mono_ns)`
- Raises `RecorderUnavailable` immediately if the critical queue cannot accept an event
- Keeps existing `write_vr_frame`, `write_robot_state`, `write_event`, and `write_camera_frame`

- [ ] **Step 1: Write RED recorder tests**

```python
@pytest.mark.asyncio
async def test_recorder_creates_jsonl_summary_and_human_template(
    tmp_path: Path,
) -> None:
    recorder = CommissioningRecorder(
        tmp_path,
        metadata={"git_commit": "abc123", "backend": "LEBAI_READONLY"},
        session_id="test-session",
    )
    await recorder.start()
    await recorder.write_critical_event(
        "preflight_result",
        {"ready": False, "reason": "real_robot_readonly"},
        100,
    )
    await recorder.close()

    session = tmp_path / "test-session"
    records = [
        json.loads(line)
        for line in (session / "session.jsonl").read_text().splitlines()
    ]
    assert records[0]["kind"] == "session_started"
    assert records[1]["kind"] == "preflight_result"
    assert records[-1]["kind"] == "session_ended"
    assert (session / "summary.json").exists()
    assert (session / "commissioning-report.md").exists()


@pytest.mark.asyncio
async def test_full_buffer_drops_old_normal_state_not_critical_stop(
    tmp_path: Path,
) -> None:
    recorder = CommissioningRecorder(
        tmp_path,
        metadata={},
        session_id="bounded",
        capacity=2,
    )
    await recorder.start(paused_writer=True)
    await recorder.write_event({"kind": "robot_state_sample", "n": 1}, 1)
    await recorder.write_event({"kind": "robot_state_sample", "n": 2}, 2)
    await recorder.write_event({"kind": "robot_state_sample", "n": 3}, 3)
    await recorder.write_critical_event("stop_requested", {"reason": "stale"}, 4)
    await recorder.resume_writer()
    await recorder.close()

    kinds = read_kinds(tmp_path / "bounded" / "session.jsonl")
    assert "stop_requested" in kinds
    assert recorder.dropped_normal_events == 1


@pytest.mark.asyncio
async def test_full_critical_queue_fails_fast_instead_of_blocking_control(
    tmp_path: Path,
) -> None:
    recorder = CommissioningRecorder(
        tmp_path,
        metadata={},
        session_id="critical-full",
        capacity=2,
        critical_capacity=1,
    )
    await recorder.start(paused_writer=True)
    await recorder.write_critical_event("stop_requested", {}, 1)

    with pytest.raises(RecorderUnavailable, match="^critical_log_queue_full$"):
        await recorder.write_critical_event("robot_fault", {}, 2)

    assert recorder.fatal_error == "critical_log_queue_full"
```

- [ ] **Step 2: Run and verify RED**

```powershell
Set-Location backend
python -m pytest tests/recording/test_commissioning.py tests/control/test_recording.py -q
```

Expected: FAIL because the recorder and lifecycle methods are absent.

- [ ] **Step 3: Implement non-blocking normal and critical queues**

Use two `collections.deque` instances protected by one `asyncio.Condition`:

- the normal queue is bounded by `capacity`; if full, drop the new normal event and increment `dropped_normal_events`;
- the critical queue is independently bounded by `critical_capacity`;
- `write_critical_event` never waits for disk or queue space;
- if the critical queue is full, atomically set `fatal_error`, raise `RecorderUnavailable("critical_log_queue_full")`, and let `RobotControl` execute its safety-stop path;
- the writer always drains critical events before normal samples, without reordering critical events relative to each other.

This preserves critical events during normal telemetry bursts without ever pausing the 50 Hz control loop. A full critical queue is treated as a safety subsystem failure, not as permission to silently discard an event.

Write UTF-8 JSON Lines with `allow_nan=False`. Flush each critical event and every 20 normal events. Write `summary.json` atomically using a temporary file in the same session directory and `Path.replace()`.

Use stable JSONL event shapes:

- `vr_frame`: Quest session/sequence/client time, server receive time, controller/head pose, tracking, Grip/Trigger/thumbstick;
- `robot_state_sample`: protocol state and actual TCP/q;
- `robot_kinematics`: actual/target q/qd/qdd, actual/target TCP, gripper, SDK latencies, sample age;
- `pvat_sent`: command ID, target TCP, p/v/a/t, SDK latency;
- critical `preflight_result`, `state_transition`, `stop_requested`, `stop_confirmed`, and `robot_fault`;
- `session_started` metadata includes Git commit, app/SDK/Python/OS versions, backend mode, sanitized limits and TCP/Home hashes but never the confirmation phrase.

Copy the tracked Markdown template into the session directory at start. `write_camera_frame` remains a no-op placeholder.

Add `start()` and `close()` no-ops to `NoopRecorder`.

- [ ] **Step 4: Run tests and commit**

```powershell
python -m pytest tests/recording/test_commissioning.py tests/control/test_recording.py -q
Set-Location ..
git add backend/app/recording backend/tests/recording backend/tests/control/test_recording.py .gitignore
git commit -m "feat: record bounded commissioning logs"
```

### Task 9: Gate Arming, Enforce the Real Envelope, and Record Safety Events

**Files:**
- Modify: `backend/app/control/robot_control.py:102-829`
- Modify: `backend/app/control/safety.py`
- Modify: `backend/tests/control/test_robot_control.py`
- Modify: `backend/tests/control/test_safety.py`
- Modify: `backend/app/recording/base.py`

**Interfaces:**
- `RobotControl.arm()` calls `await backend.preflight()` before state-machine arming
- Adds stable rejection `arm_blocked_by_preflight:<reason>`
- Records critical `state_transition`, `stop_requested`, `stop_confirmed`, `robot_fault`, and `preflight_result`
- Adds optional per-axis workspace, orientation-from-zero, and hard per-cycle caps to `SafetyLimiter`
- Preserves existing simulator behavior because simulator preflight is ready

- [ ] **Step 1: Write RED preflight gate tests**

Append to `backend/tests/control/test_robot_control.py`:

```python
@pytest.mark.asyncio
async def test_arm_rejects_failed_backend_preflight_without_motion() -> None:
    control, latest, backend, clock = make_control()
    latest.publish(frame(1, grip=False), clock.now_ns())
    await control.tick()
    backend.preflight_result = BackendPreflight(
        ready=False,
        reason="tcp_mismatch",
        robot_state=BackendState.IDLE,
        actual_tcp=backend.actual_tcp,
        actual_q=tuple(backend.actual_q),
        tcp_matches=False,
        capabilities=("pvat",),
    )

    with pytest.raises(
        RuntimeError,
        match="^arm_blocked_by_preflight:tcp_mismatch$",
    ):
        await control.arm()

    assert control.mode is TeleopMode.READY
    assert backend.targets == []


@pytest.mark.asyncio
async def test_stop_records_request_before_backend_and_confirmation_after() -> None:
    recorder = RecordingRecorder()
    control, latest, backend, clock = make_control(recorder=recorder)
    await activate(control, latest, clock)
    latest.publish(frame(10, grip=False), clock.now_ns())

    await control.tick()

    kinds = [event["kind"] for event in recorder.events]
    assert kinds.index("stop_requested") < kinds.index("stop_confirmed")


@pytest.mark.asyncio
async def test_critical_recorder_failure_never_prevents_physical_stop() -> None:
    recorder = RecordingRecorder()
    control, latest, backend, clock = make_control(recorder=recorder)
    await activate(control, latest, clock)
    recorder.fail_critical_with = RecorderUnavailable("critical_log_queue_full")

    await control.disarm()

    assert backend.stop_reasons[-1] is StopReason.GRIP_RELEASED
    assert control.mode is TeleopMode.FAULT
    assert control._fault == "recording_unavailable"
```

Update the existing `FakeBackend` with a default ready preflight; do not special-case tests in production.

Add `test_safety.py` cases proving that an optionally configured limiter:

- marks any per-axis displacement beyond `±0.10 m` as constrained;
- marks an orientation delta beyond `30°` from the captured TCP orientation as constrained;
- never produces a per-cycle displacement above `0.002 m` or rotation above `1°`, even after a long scheduler `dt`;
- preserves the current spherical/default behavior when the new options are `None`.

Add a `RobotControl` test proving an out-of-envelope frame holds the last safe target without calling `backend.command_tcp`, sets a soft constraint, and a subsequent hand retreat inside the envelope resumes commands without fault/reset.

- [ ] **Step 2: Run and verify RED**

```powershell
Set-Location backend
python -m pytest tests/control/test_robot_control.py -q
```

Expected: the new tests fail; report any pre-existing failures separately.

- [ ] **Step 3: Implement the minimal gate and event helpers**

At the start of `arm()` after existing local state checks:

```python
preflight = await self.backend.preflight()
recorded = await self._record_critical(
    "preflight_result",
    {
        "ready": preflight.ready,
        "reason": preflight.reason,
        "robot_state": preflight.robot_state.value,
        "tcp_matches": preflight.tcp_matches,
        "capabilities": list(preflight.capabilities),
    },
)
if not recorded:
    raise RuntimeError("arm_blocked_by_preflight:recording_unavailable")
if not preflight.ready:
    raise RuntimeError(f"arm_blocked_by_preflight:{preflight.reason}")
```

Make `write_critical_event` part of `RecorderSink` and implement it in every recorder, including `NoopRecorder` and test recorders. Add a private `_record_critical(kind, payload) -> bool` helper that catches `RecorderUnavailable`, latches the public fault `recording_unavailable`, and returns immediately.

Record a stop request before every backend stop and confirmation only after `backend.stop()` returns. A recorder exception must never skip, cancel, or delay `backend.stop()`: the stop path attempts logging, performs the physical stop unconditionally, and then enters `FAULT` if logging failed. Failure while ACTIVE from any normal recorder method also routes through this same non-recursive stop path. Record faults using stable public reasons, never raw SDK exception strings.

Extend `SafetyLimiter` with optional `workspace_half_extent_m`, `max_rotation_from_anchor_rad`, `max_linear_step_m`, and `max_angular_step_rad`. Add `set_pose_anchor(Pose)` and keep the current `set_anchor(tuple)` API for simulator compatibility. Project position against the captured zero using an axis-aligned box and measure orientation with quaternion/rotation-vector angular distance. When the envelope is constrained, `RobotControl` holds `last_target`, resets the filter/dynamic velocity, publishes the soft constraint, and sends no robot command; an in-bounds retreat follows the existing constraint-clear path.

Do not change WebXR Grip-zero capture, controller-to-TCP rotation composition, or default simulator limiter parameters.

- [ ] **Step 4: Run focused RobotControl tests**

```powershell
python -m pytest tests/control/test_robot_control.py -q
```

Expected: all tests in this file pass, or the exact known baseline remains and is reported without changing assertions.

- [ ] **Step 5: Commit**

```powershell
git add backend/app/control/robot_control.py backend/app/control/safety.py backend/app/recording/base.py backend/tests/control/test_robot_control.py backend/tests/control/test_safety.py
git commit -m "feat: gate and bound real teleoperation"
```

### Task 10: Wire Runtime Backend, Recorder, Health, and Protocol Diagnostics

**Files:**
- Modify: `backend/app/main.py:1-91`
- Modify: `backend/app/api/health.py`
- Modify: `backend/app/schemas/messages.py:1-104`
- Modify: `schemas/teleop-v1.json`
- Modify: `backend/tests/api/test_health.py`
- Modify: `backend/tests/contract/test_messages.py`
- Modify: `backend/tests/api/test_teleop_ws.py`

**Interfaces:**
- Produces `build_backend(settings, recorder, client_factory=connect_real_client) -> RobotBackend`
- Produces `build_recorder(settings) -> RecorderSink`
- Extends `create_app(*, settings=None, client_factory=connect_real_client)` for dependency injection in Fake SDK tests
- `/health` returns dynamic `backend`, `real_robot_mode`, `real_robot_enabled`, `preflight_ready`, and public `preflight_reason`
- Adds optional protocol fields:

```python
backend: Literal["SIMULATOR", "LEBAI"] | None = None
real_robot_mode: Literal["readonly", "control"] | None = None
preflight_ready: bool | None = None
preflight_reason: str | None = None
```

- [ ] **Step 1: Write RED backend-factory and health tests**

```python
def test_simulator_app_never_imports_lebai_sdk(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    real_import = builtins.__import__

    def guarded(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "lebai_sdk":
            raise AssertionError("simulator imported real SDK")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", guarded)
    with TestClient(create_app()) as client:
        payload = client.get("/health").json()
    assert payload["backend"] == "SIMULATOR"
    assert payload["real_robot_enabled"] is False


def test_readonly_health_reports_not_enabled(
    real_config, fake_client_factory, monkeypatch
) -> None:
    monkeypatch.setenv("VR4ARM_CONFIG", str(real_config))
    settings = Settings.load()
    with TestClient(
        create_app(settings=settings, client_factory=fake_client_factory)
    ) as client:
        payload = client.get("/health").json()
    assert payload == {
        "status": "ok",
        "backend": "LEBAI",
        "real_robot_mode": "readonly",
        "real_robot_enabled": False,
        "preflight_ready": False,
        "preflight_reason": "real_robot_readonly",
    }
```

Add protocol tests proving old protocol-v1 messages without new optional fields still validate.

- [ ] **Step 2: Run and verify RED**

```powershell
Set-Location backend
python -m pytest tests/api/test_health.py tests/contract/test_messages.py tests/api/test_teleop_ws.py -q
```

Expected: new assertions fail; if Starlette collection errors occur, record them as baseline before making production changes.

- [ ] **Step 3: Implement factories and lifecycle order**

`build_backend`:

```python
def build_backend(
    settings: Settings,
    recorder: RecorderSink,
    client_factory: ClientFactory = connect_real_client,
) -> RobotBackend:
    if settings.backend == "simulator":
        return SimRobotAdapter()
    if settings.lebai is None:
        raise RuntimeError("missing_lebai_settings")
    return RealLebaiAdapter(
        settings.lebai,
        client_factory=client_factory,
        event_callback=recorder.write_event,
    )
```

`create_app` loads settings only when no settings object is injected and otherwise preserves the current module-level `app = create_app()` entry point. It builds the recorder before the backend, passes the recorder and client factory to `build_backend`, and starts the recorder before backend connect so connection failures are logged. `build_recorder` returns `NoopRecorder` for simulator and `CommissioningRecorder` for Lebai. Shutdown order is:

1. revoke RobotControl and stop control;
2. stop/disconnect backend;
3. close recorder.

Health reads only cached app state; the endpoint must not call the SDK.

Build the limiter from the existing flat settings in simulator mode. In Lebai mode, use the approved real values from `settings.lebai.control`:

```python
SafetyLimiter(
    max_linear_speed=control.max_tcp_speed_mps,
    max_angular_speed=control.max_tcp_rotation_radps,
    max_linear_accel=control.max_tcp_acceleration_mps2,
    max_angular_accel=control.max_tcp_angular_acceleration_radps2,
    workspace_half_extent_m=control.max_relative_translation_m,
    max_rotation_from_anchor_rad=np.deg2rad(
        control.max_relative_rotation_deg
    ),
    max_linear_step_m=control.max_tcp_step_m,
    max_angular_step_rad=np.deg2rad(
        control.max_tcp_rotation_step_deg
    ),
)
```

Use `CoordinateMapper(translation_scale=control.translation_scale, rotation_scale=1.0, ...)` for Lebai so controller orientation continues to map directly onto the captured TCP orientation.

Keep protocol fields optional and update canonical JSON schema and fixtures without changing `v: 1`.

- [ ] **Step 4: Run focused API/contract tests**

```powershell
python -m pytest tests/api/test_health.py tests/contract/test_messages.py tests/api/test_teleop_ws.py -q
```

Expected: focused tests pass after any separately justified Starlette compatibility work; do not hide collection failures.

- [ ] **Step 5: Commit**

```powershell
git add backend/app/main.py backend/app/api/health.py backend/app/schemas/messages.py schemas/teleop-v1.json backend/tests/api/test_health.py backend/tests/contract/test_messages.py backend/tests/api/test_teleop_ws.py
git commit -m "feat: wire explicit Lebai runtime modes"
```

### Task 11: Add Read-Only Preflight and Explicit 5 mm Smoke Scripts

**Files:**
- Create: `backend/app/commissioning/__init__.py`
- Create: `backend/app/commissioning/preflight.py`
- Create: `backend/app/commissioning/smoke.py`
- Create: `scripts/real_robot_preflight.py`
- Create: `scripts/real_robot_smoke.py`
- Create: `backend/tests/scripts/test_real_robot_scripts.py`

**Interfaces:**
- `async run_preflight(config_path, output_path, client_factory) -> int`
- `parse_smoke_args(argv) -> SmokeOptions`
- `async run_smoke(options, client_factory) -> int`
- `real_robot_preflight.py --config PATH --output PATH`
- `real_robot_smoke.py --config PATH --axis {x,y,z} --distance-m 0.005 --confirm EXACT_PHRASE`
- Root scripts are thin argument/exit-code wrappers around `app.commissioning`; neither root script imports `lebai_sdk` directly

- [ ] **Step 1: Write RED CLI tests using Fake SDK**

```python
from app.commissioning.preflight import run_preflight
from app.commissioning.smoke import parse_smoke_args


@pytest.mark.asyncio
async def test_preflight_script_never_arms_or_writes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = FakeLebaiClient.idle()
    result = await run_preflight(
        config_path=readonly_config(tmp_path),
        output_path=tmp_path / "report.json",
        client_factory=AsyncMock(return_value=client),
    )
    assert result == 0
    assert client.write_calls == []


def test_smoke_rejects_distance_over_five_mm(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="^smoke_distance_exceeds_0.005_m$"):
        parse_smoke_args(
            [
                "--config", str(control_config(tmp_path)),
                "--axis", "x",
                "--distance-m", "0.006",
                "--confirm", "I_UNDERSTAND_REAL_ROBOT_MOTION",
            ]
        )
```

Add tests that readonly mode, wrong confirmation, missing Home/TCP, and non-IDLE state reject before any write.

- [ ] **Step 2: Run and verify RED**

```powershell
Set-Location backend
python -m pytest tests/scripts/test_real_robot_scripts.py -q
```

Expected: FAIL because the reusable commissioning modules are absent.

- [ ] **Step 3: Implement preflight as read-only**

`app.commissioning.preflight.run_preflight`:

1. sets `VR4ARM_CONFIG`;
2. requires configured mode `readonly`;
3. constructs the adapter through `build_backend`;
4. connects and calls `preflight`;
5. writes versions, TCP, state, actual q/qd/TCP, gripper, capabilities, and latencies;
6. disconnects without any SDK write;
7. returns zero only when data acquisition is complete, even though motion readiness is false by design.

- [ ] **Step 4: Implement the smoke script through RobotControl**

`app.commissioning.smoke.run_smoke` requires control mode and exact CLI confirmation. It constructs `RobotControl`, publishes a synthetic released frame, arms, publishes a Grip-capture frame at origin, then one frame offset by at most `5 mm` on the requested axis. It holds for one PVAT horizon, publishes Grip release, awaits verified stop, and exits.

Use the same `CoordinateMapper`, `SafetyLimiter`, `RealLebaiAdapter`, recorder, and stop path as Quest. Do not call adapter motion directly.

Print a final message directing the operator to inspect actual direction before another axis. One invocation moves one axis once.

- [ ] **Step 5: Run fake CLI tests and commit**

```powershell
python -m pytest tests/scripts/test_real_robot_scripts.py -q
Set-Location ..
git add backend/app/commissioning scripts/real_robot_preflight.py scripts/real_robot_smoke.py backend/tests/scripts/test_real_robot_scripts.py
git commit -m "feat: add guarded LM3 commissioning scripts"
```

### Task 12: Write Deployment and Human Commissioning Documents

**Files:**
- Create: `docs/real-robot-deployment.md`
- Create: `docs/real-robot-commissioning-report.md`
- Modify: `docs/quest-development.md`

**Interfaces:**
- Documents the exact GitHub pull, environment, SDK, HTTPS, firewall, read-only, control-enable, smoke, Quest, stop, and log-upload sequence
- Produces the Markdown template copied by `CommissioningRecorder`

- [ ] **Step 1: Write the deployment document with exact Linux and PowerShell commands**

Include these commands, with placeholders explicitly described as operator substitutions rather than runnable defaults:

```bash
git clone https://github.com/kilohexin/VR4AM.git
cd VR4AM
git checkout codex/teleoperation-ux-recovery
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -e "./backend[dev,real]"
cd web
npm ci
npm run build
```

PowerShell:

```powershell
git clone https://github.com/kilohexin/VR4AM.git
Set-Location VR4AM
git checkout codex/teleoperation-ux-recovery
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e ".\backend[dev,real]"
Set-Location web
npm.cmd ci
npm.cmd run build
```

Document that the operator copies `config/real-robot.example.yaml` to `config/real-robot.local.yaml`, fills IP/TCP/Home locally, starts readonly first, and never commits that file.

Document current runtime:

```powershell
$env:VR4ARM_CONFIG = "D:\path\VR4AM\config\real-robot.local.yaml"
Set-Location backend
python -m uvicorn app.main:app --host 127.0.0.1 --port 8000
```

In a second terminal:

```powershell
Set-Location web
npm.cmd run dev -- --host 0.0.0.0
```

Quest connects to `https://<server-lan-ip>:5173/`; Vite proxies `/ws` to the co-located FastAPI process. Include certificate acceptance and firewall steps without hard-coding a server IP.

- [ ] **Step 2: Write the human commissioning checklist**

Include checkboxes and fill-in fields for:

- Git commit, OS, Python, Node, SDK, L Master versions;
- robot/server IP and ping latency;
- `get_tcp`, payload mass/CoM, Home q, joint limits;
- physical emergency stop, observer, clear workspace;
- readonly five-minute observation;
- stop API;
- gripper amplitude direction and force;
- 5 mm x/y/z direction;
- translation-only Quest;
- roll/pitch/yaw Quest;
- Grip release, page close, Wi-Fi loss, backend shutdown;
- stop confirmation latency from `summary.json`;
- light-object grasp/release;
- subjective lag, jitter, mapping, boundary, and safety notes;
- attachment filenames.

- [ ] **Step 3: Link the documents from Quest development notes**

Add a “真机实验” section that links to deployment, commissioning report, approved design, and implementation plan. State that the simulator instructions remain the default.

- [ ] **Step 4: Validate document links and format**

```powershell
rg -n "real-robot|LEBAI_READONLY|I_UNDERSTAND_REAL_ROBOT_MOTION" docs config
git diff --check -- docs/real-robot-deployment.md docs/real-robot-commissioning-report.md docs/quest-development.md
```

Expected: every referenced local file exists and `git diff --check` prints nothing.

- [ ] **Step 5: Commit**

```powershell
git add docs/real-robot-deployment.md docs/real-robot-commissioning-report.md docs/quest-development.md
git commit -m "docs: add LM3 commissioning workflow"
```

### Task 13: Run Focused End-to-End Verification and Report the Full Baseline Honestly

**Files:**
- Modify only if a genuine new-feature defect is found in a file from Tasks 1–12
- Do not edit unrelated tests to make this task green

**Interfaces:**
- Verifies simulator isolation, Fake SDK real path, frontend compatibility, build, and documentation
- Produces a report with separate focused/full/lab statuses

- [ ] **Step 1: Verify no ordinary path can import or contact the SDK**

Run:

```powershell
Set-Location backend
python -m pytest tests/config tests/robots tests/recording tests/scripts -q
```

Expected: all new focused tests pass with no installed SDK and no network calls.

- [ ] **Step 2: Verify control and protocol integration**

```powershell
python -m pytest tests/control/test_robot_control.py tests/contract/test_messages.py tests/api/test_health.py tests/api/test_teleop_ws.py -q
```

Expected: all new/modified integration assertions pass. If the known Starlette baseline blocks collection, report the exact files and exception; do not describe this command as passing.

- [ ] **Step 3: Run the complete backend suite**

```powershell
python -m pytest
```

Expected reporting format:

```text
Backend full suite: PASS (<count> passed)
```

or:

```text
Backend full suite: FAIL (<count> failed, <count> errors)
Known/pre-existing: <exact tests and messages>
New-feature failures: <exact tests and messages>
```

Do not merge the categories.

- [ ] **Step 4: Run frontend tests and production build**

```powershell
Set-Location ..\web
npm.cmd test
npm.cmd run build
```

Report test and build separately. A successful build does not imply tests passed.

- [ ] **Step 5: Verify diffs and safety strings**

```powershell
Set-Location ..
git diff --check
rg -n "start_sys|set_tcp|init_claw|speedl|estop" backend/app scripts
rg -n "move_pvat|stop_move|stop_sys|real_robot_readonly" backend/app
git status --short
```

Inspect every match. Accepted matches:

- `stop_move` in routine stop;
- `stop_sys` only in failed-stop escalation;
- `move_pvat` only in the adapter;
- forbidden methods only in explicit guards, docs, or tests asserting they are not called.

- [ ] **Step 6: Run read-only lab verification only when physically present**

Do not run this during ordinary development. In the lab, with observer and emergency stop ready:

```powershell
$env:VR4ARM_CONFIG = "D:\path\VR4AM\config\real-robot.local.yaml"
python scripts/real_robot_preflight.py --config $env:VR4ARM_CONFIG --output ".\logs\preflight.json"
```

Expected: zero SDK writes, a complete report, and `real_robot_readonly`.

- [ ] **Step 7: Run the 5 mm smoke only after the human checklist approves it**

```powershell
$env:VR4ARM_REAL_ROBOT_CONFIRM = "I_UNDERSTAND_REAL_ROBOT_MOTION"
python scripts/real_robot_smoke.py `
  --config $env:VR4ARM_CONFIG `
  --axis x `
  --distance-m 0.005 `
  --confirm I_UNDERSTAND_REAL_ROBOT_MOTION
```

Run one axis per process. The observer confirms direction and stop before the next invocation.

- [ ] **Step 8: Commit only genuine verification fixes**

If no new-feature fix was needed, do not create an empty commit. If a focused test found a real defect, follow TDD, stage only the affected Task 1–12 files and their tests, then:

```powershell
git commit -m "fix: harden real robot commissioning"
```

## Execution Completion Criteria

Implementation work is ready for the user's lab deployment only when:

- all focused real-robot/Fake SDK tests pass;
- simulator mode starts without the SDK installed;
- read-only mode has no write-capable path;
- control mode requires both explicit config and exact environment confirmation;
- vendor IK and bounded PVAT are the only normal real motion path;
- stop invalidation is proven against delayed IK and queued commands;
- routine stop has stationary confirmation and failed-stop escalation;
- TCP mismatch, non-IDLE, running motion, estop, stale state, and malformed SDK data reject arming;
- Home and gripper use only local, human-confirmed configuration;
- commissioning logs preserve critical events;
- deployment and human experiment documents are complete;
- complete backend and frontend suite results are reported exactly as observed;
- no real motion test has been run outside the lab;
- all commits remain local until the user pushes.
