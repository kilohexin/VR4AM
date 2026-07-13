# Quest 3 LM3 VR Simulation Milestone 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a desktop- and Quest-runnable WebXR application in which the Quest 3 right controller safely drives a 50 Hz Python kinematic digital twin of a Lebai LM3 and its gripper, without importing or connecting the real robot SDK.

**Architecture:** A FastAPI process receives versioned WebSocket messages and stores only the newest VR frame. A fixed-rate `RobotControl` task applies clutching, coordinate mapping, filtering, safety limits, and watchdog rules before commanding a `SimRobotAdapter`; the WebXR client renders backend joint state using the existing LM3 GLB. The simulator and disabled real adapter share one `RobotBackend` protocol so later real-hardware work does not fork the control path.

**Tech Stack:** Python 3.11, FastAPI, Uvicorn, Pydantic 2, asyncio, NumPy, SciPy, pytest, TypeScript 5.9, Vite 7, Three.js 0.181, WebXR, Vitest.

## Global Constraints

- Milestone 1 must not install, import, discover, or connect `lebai_sdk` or `lebai_sdk_asyncio`.
- The only enabled backend is `SIMULATOR`; every real-backend method raises `real_robot_disabled`.
- RobotControl runs at 50 Hz with an absolute monotonic deadline and a capacity-1 latest-value input.
- Quest sends at no more than 60 Hz; robot state is sent at 20 Hz.
- Grip is the motion clutch; Trigger is normalized gripper closure where `0=open` and `1=closed`.
- Input age `>=100 ms`, tracking loss, hidden page, socket close, or two consecutive 40 ms overruns closes the command gate and stops.
- Input age `>=250 ms` forces DISARMED even if controlled deceleration is still completing.
- Reconnect never resumes automatically; one valid `grip=false` frame and a new `arm_request` are required.
- Cross-process pose messages use metres and quaternion `[x,y,z,w]`; Lebai Z-Y-X Euler values do not cross the WebSocket.
- PC `time.monotonic_ns()` is the authoritative clock.
- GLB is visual only; Python theoretical LM3 modified-DH kinematics is authoritative for simulation state.
- The simulator uses a session safety window `q_ref ± π`, maximum joint speed `0.5 rad/s`, and acceleration `1.0 rad/s²`; these are not hardware limits.
- Milestone 1 contains no Cannon-es, physical object grasping, camera capture, dataset writer, MR passthrough, or real robot motion.
- User-facing UI and errors are Chinese.
- Use TDD for every behavior task and commit after every task.

## Approved Implementation Clarifications (2026-07-13)

- An overrun is measured as lateness relative to the absolute monotonic deadline, not as raw iteration execution time. Two consecutive overruns greater than `40 ms` close the command gate and stop with `FAULT`; publish `FAULT` at least once, then transition to `DISARMED` after stop completion.
- Recovery is strictly `DISARMED -> READY -> ARMED`: a valid `grip=false` frame unlocks `READY`, and a new explicit `arm_request` is required for `ARMED`. Direct `DISARMED -> ARMED` is forbidden.
- `STALE` must be observable in at least one 20 Hz state message before the stopped backend can complete the transition to `DISARMED`.
- Gripper commands have a hard `10 Hz` ceiling. Accumulate the newest value and send it in an eligible 100 ms window only when its delta from the last command exceeds `0.02`; never queue historical gripper commands.
- The Locked File Structure is the baseline layout. A later task's explicit `Create:` entry is an approved extension to it.
- Corrupted symbols are interpreted as `q_ref ± π`, `home_q ± 0.35`, and `rad/s²`. The gripper uses the reference `Take 001` animation frames `0–20`. User-facing strings must be readable Chinese, never mojibake.

## Locked File Structure

```text
backend/
  pyproject.toml
  app/
    __init__.py
    main.py
    config.py
    timebase.py
    api/health.py
    api/teleop_ws.py
    control/coordinate_mapper.py
    control/filters.py
    control/robot_control.py
    control/state_machine.py
    control/safety.py
    robots/base.py
    robots/sim_adapter.py
    robots/lebai_adapter.py
    sim/lm3_model.py
    sim/kinematics.py
    sim/ik.py
    sim/virtual_robot.py
    recording/base.py
    recording/noop.py
    schemas/messages.py
  tests/
    api/
    contract/
    control/
    sim/
web/
  package.json
  tsconfig.json
  vite.config.ts
  index.html
  public/models/Lebai_LM3.glb
  src/
    main.ts
    styles.css
    protocol/messages.ts
    transport/teleopSocket.ts
    robot/robotModel.ts
    robot/robotState.ts
    scenes/simulationScene.ts
    xr/controllerInput.ts
    xr/session.ts
    ui/armPanel.ts
    ui/hud.ts
  tests/
schemas/
  teleop-v1.json
  fixtures/
config/default.yaml
scripts/soak_simulator.py
docs/quest-development.md
README.md
```

---

### Task 1: Bootstrap the Monorepo and Versioned Protocol Contract

**Files:**
- Create: `backend/pyproject.toml`
- Create: `backend/app/__init__.py`
- Create: `backend/app/schemas/messages.py`
- Create: `backend/tests/contract/test_messages.py`
- Create: `schemas/teleop-v1.json`
- Create: `schemas/fixtures/vr-frame-valid.json`
- Create: `schemas/fixtures/robot-state-valid.json`
- Create: `config/default.yaml`
- Modify: `.gitignore`

**Interfaces:**
- Produces: `Pose(p: tuple[float, float, float], q: tuple[float, float, float, float])`
- Produces: `ControllerState`, `VRFrame`, `RobotStateMessage`, `ClientControlMessage`
- Produces enums: `TeleopMode`, `BackendState`
- Produces protocol version constant: `PROTOCOL_VERSION = 1`

- [ ] **Step 1: Add the failing message contract tests**

Create `backend/tests/contract/test_messages.py`:

```python
import json
from pathlib import Path

import pytest
from pydantic import ValidationError

from app.schemas.messages import RobotStateMessage, VRFrame

ROOT = Path(__file__).resolve().parents[3]


def load_fixture(name: str) -> dict:
    return json.loads((ROOT / "schemas" / "fixtures" / name).read_text(encoding="utf-8"))


def test_valid_vr_frame_fixture_round_trips() -> None:
    frame = VRFrame.model_validate(load_fixture("vr-frame-valid.json"))
    assert frame.v == 1
    assert frame.right.q == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_valid_robot_state_fixture_round_trips() -> None:
    state = RobotStateMessage.model_validate(load_fixture("robot-state-valid.json"))
    assert state.mode == "ARMED"
    assert len(state.actual_q) == 6


@pytest.mark.parametrize("bad_q", [[0, 0, 0, 0], [0, 0, 0, 2], [float("nan"), 0, 0, 1]])
def test_vr_frame_rejects_invalid_quaternion(bad_q: list[float]) -> None:
    payload = load_fixture("vr-frame-valid.json")
    payload["right"]["q"] = bad_q
    with pytest.raises(ValidationError):
        VRFrame.model_validate(payload)


def test_vr_frame_rejects_non_finite_position() -> None:
    payload = load_fixture("vr-frame-valid.json")
    payload["right"]["p"][1] = float("inf")
    with pytest.raises(ValidationError):
        VRFrame.model_validate(payload)
```

- [ ] **Step 2: Run the contract test and verify import failure**

Run:

```powershell
cd D:\MyWork\VR4Arm\backend
python -m pytest tests\contract\test_messages.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'app.schemas'`.

- [ ] **Step 3: Add backend dependencies and complete Pydantic messages**

Create `backend/pyproject.toml`:

```toml
[project]
name = "vr4arm-backend"
version = "0.1.0"
requires-python = ">=3.11"
dependencies = [
  "fastapi>=0.115,<1",
  "uvicorn[standard]>=0.30,<1",
  "pydantic>=2.8,<3",
  "numpy>=2.0,<3",
  "scipy>=1.14,<2",
  "pyyaml>=6.0,<7"
]

[project.optional-dependencies]
dev = ["pytest>=8.2,<9", "pytest-asyncio>=0.24,<1", "httpx>=0.27,<1"]

[tool.pytest.ini_options]
pythonpath = ["."]
testpaths = ["tests"]
asyncio_mode = "auto"
```

Create empty `backend/app/__init__.py` and `backend/app/schemas/__init__.py`, then create `backend/app/schemas/messages.py`:

```python
from __future__ import annotations

import math
from enum import StrEnum
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator

PROTOCOL_VERSION = 1
Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]
JointVector = tuple[float, float, float, float, float, float]


class StrictMessage(BaseModel):
    model_config = ConfigDict(extra="forbid")


def _finite(values: tuple[float, ...], name: str) -> tuple[float, ...]:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} must contain only finite values")
    return values


class Pose(StrictMessage):
    p: Vec3
    q: Quat

    @field_validator("p")
    @classmethod
    def validate_position(cls, value: Vec3) -> Vec3:
        return _finite(value, "position")  # type: ignore[return-value]

    @field_validator("q")
    @classmethod
    def validate_quaternion(cls, value: Quat) -> Quat:
        _finite(value, "quaternion")
        norm = math.sqrt(sum(component * component for component in value))
        if not 0.9 <= norm <= 1.1:
            raise ValueError("quaternion norm must be between 0.9 and 1.1")
        return tuple(component / norm for component in value)  # type: ignore[return-value]


class ControllerState(Pose):
    grip: bool
    trigger: Annotated[float, Field(ge=0.0, le=1.0)]


class VRFrame(StrictMessage):
    v: Literal[1]
    type: Literal["vr_frame"]
    session_id: str = Field(min_length=1, max_length=64)
    seq: int = Field(ge=0)
    client_mono_ms: float = Field(ge=0)
    tracking_valid: bool
    visibility: Literal["visible", "visible-blurred", "hidden"]
    right: ControllerState


class TeleopMode(StrEnum):
    DISCONNECTED = "DISCONNECTED"
    READY = "READY"
    ARMED = "ARMED"
    ACTIVE = "ACTIVE"
    HOLD = "HOLD"
    STALE = "STALE"
    FAULT = "FAULT"
    DISARMED = "DISARMED"


class BackendState(StrEnum):
    DISCONNECTED = "DISCONNECTED"
    IDLE = "IDLE"
    MOVING = "MOVING"
    HOLD = "HOLD"
    FAULT = "FAULT"


class RobotStateMessage(StrictMessage):
    v: Literal[1] = 1
    type: Literal["robot_state"] = "robot_state"
    server_mono_ns: int = Field(ge=0)
    ack_seq: int | None = Field(default=None, ge=0)
    mode: TeleopMode
    robot_state: BackendState
    actual_tcp: Pose
    actual_q: JointVector
    gripper: Annotated[float, Field(ge=0.0, le=1.0)]
    sample_age_ms: float | None = Field(default=None, ge=0)
    fault: str | None = None


class ClientControlMessage(StrictMessage):
    v: Literal[1]
    type: Literal["hello", "arm_request", "disarm", "ping"]
    request_id: str = Field(min_length=1, max_length=64)
    client_mono_ms: float | None = Field(default=None, ge=0)
```

- [ ] **Step 4: Add exact fixtures, JSON Schema, and default configuration**

Create `schemas/fixtures/vr-frame-valid.json`:

```json
{"v":1,"type":"vr_frame","session_id":"fixture","seq":1,"client_mono_ms":10.0,"tracking_valid":true,"visibility":"visible","right":{"p":[0.0,1.2,-0.3],"q":[0.0,0.0,0.0,1.0],"grip":false,"trigger":0.0}}
```

Create `schemas/fixtures/robot-state-valid.json`:

```json
{"v":1,"type":"robot_state","server_mono_ns":1,"ack_seq":1,"mode":"ARMED","robot_state":"IDLE","actual_tcp":{"p":[0.3,0.0,0.3],"q":[0.0,0.0,0.0,1.0]},"actual_q":[0,0,0,0,0,0],"gripper":0.0,"sample_age_ms":5.0,"fault":null}
```

Generate `schemas/teleop-v1.json` from the Pydantic models with this one-time command and commit the result:

```powershell
cd D:\MyWork\VR4Arm\backend
python -c "import json; from app.schemas.messages import VRFrame,RobotStateMessage,ClientControlMessage; print(json.dumps({'$schema':'https://json-schema.org/draft/2020-12/schema','$defs':{'VRFrame':VRFrame.model_json_schema(),'RobotStateMessage':RobotStateMessage.model_json_schema(),'ClientControlMessage':ClientControlMessage.model_json_schema()}},ensure_ascii=False,indent=2))" | Set-Content -Encoding utf8 ..\schemas\teleop-v1.json
```

Create `config/default.yaml`:

```yaml
backend: simulator
control_hz: 50
state_hz: 20
stale_ms: 100
disarm_ms: 250
max_linear_speed_mps: 0.15
max_angular_speed_radps: 0.6
max_linear_accel_mps2: 0.4
max_angular_accel_radps2: 1.2
translation_scale: 0.8
rotation_scale: 1.0
joint_speed_radps: 0.5
joint_accel_radps2: 1.0
joint_window_rad: 3.141592653589793
```

- [ ] **Step 5: Run contract tests**

Run: `cd D:\MyWork\VR4Arm\backend; python -m pytest tests\contract\test_messages.py -q`
Expected: `6 passed`.

- [ ] **Step 6: Extend `.gitignore` and commit**

Append exactly:

```gitignore
.venv/
__pycache__/
.pytest_cache/
node_modules/
dist/
*.log
```

Run:

```powershell
git add .gitignore backend schemas config
git commit -m "chore: bootstrap protocol and backend models"
```

---

### Task 2: Implement Coordinate Mapping and Clutch Pose Math

**Files:**
- Create: `backend/app/control/coordinate_mapper.py`
- Create: `backend/tests/control/test_coordinate_mapper.py`

**Interfaces:**
- Consumes: `Pose` from `app.schemas.messages`
- Produces: `CoordinateMapper.capture(hand: Pose, tcp: Pose) -> None`
- Produces: `CoordinateMapper.target(hand: Pose) -> Pose`
- Produces: `CoordinateMapper.clear() -> None`
- Produces: `DEFAULT_R_BX: numpy.ndarray`

- [ ] **Step 1: Write failing axis and clutch tests**

Create `backend/tests/control/test_coordinate_mapper.py`:

```python
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.control.coordinate_mapper import CoordinateMapper, DEFAULT_R_BX
from app.schemas.messages import Pose

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def pose(p=(0.0, 0.0, 0.0), q=IDENTITY) -> Pose:
    return Pose(p=p, q=q)


def test_default_mapping_is_proper_rotation() -> None:
    assert np.allclose(DEFAULT_R_BX.T @ DEFAULT_R_BX, np.eye(3))
    assert np.linalg.det(DEFAULT_R_BX) == pytest.approx(1.0)


@pytest.mark.parametrize(
    ("hand_delta", "robot_delta"),
    [((0, 0, -0.1), (0.08, 0, 0)), ((0.1, 0, 0), (0, -0.08, 0)), ((0, 0.1, 0), (0, 0, 0.08))],
)
def test_default_axes_and_translation_scale(hand_delta, robot_delta) -> None:
    mapper = CoordinateMapper(translation_scale=0.8)
    mapper.capture(pose(), pose((0.3, 0.0, 0.2)))
    assert mapper.target(pose(hand_delta)).p == pytest.approx(tuple(np.add((0.3, 0.0, 0.2), robot_delta)))


def test_capture_makes_current_hand_equal_current_tcp() -> None:
    hand = pose((1, 2, 3), Rotation.from_euler("x", 20, degrees=True).as_quat())
    tcp = pose((0.2, 0.1, 0.4), Rotation.from_euler("z", 30, degrees=True).as_quat())
    mapper = CoordinateMapper()
    mapper.capture(hand, tcp)
    target = mapper.target(hand)
    assert target.p == pytest.approx(tcp.p)
    assert abs(np.dot(target.q, tcp.q)) == pytest.approx(1.0)


def test_target_requires_anchor() -> None:
    with pytest.raises(RuntimeError, match="anchor_not_captured"):
        CoordinateMapper().target(pose())
```

- [ ] **Step 2: Run tests and verify import failure**

Run: `cd D:\MyWork\VR4Arm\backend; python -m pytest tests\control\test_coordinate_mapper.py -q`
Expected: FAIL with missing `app.control.coordinate_mapper`.

- [ ] **Step 3: Implement complete mapper**

Create `backend/app/control/coordinate_mapper.py`:

```python
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from app.schemas.messages import Pose

DEFAULT_R_BX = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


class CoordinateMapper:
    def __init__(self, translation_scale: float = 0.8, rotation_scale: float = 1.0, r_bx: np.ndarray = DEFAULT_R_BX):
        self.translation_scale = translation_scale
        self.rotation_scale = rotation_scale
        self.r_bx = np.asarray(r_bx, dtype=float)
        if not np.allclose(self.r_bx.T @ self.r_bx, np.eye(3), atol=1e-8) or not np.isclose(np.linalg.det(self.r_bx), 1.0):
            raise ValueError("r_bx must be a proper rotation")
        self._hand_anchor: Pose | None = None
        self._tcp_anchor: Pose | None = None

    def capture(self, hand: Pose, tcp: Pose) -> None:
        self._hand_anchor = hand.model_copy(deep=True)
        self._tcp_anchor = tcp.model_copy(deep=True)

    def clear(self) -> None:
        self._hand_anchor = None
        self._tcp_anchor = None

    def target(self, hand: Pose) -> Pose:
        if self._hand_anchor is None or self._tcp_anchor is None:
            raise RuntimeError("anchor_not_captured")
        p = np.asarray(self._tcp_anchor.p) + self.r_bx @ (self.translation_scale * (np.asarray(hand.p) - np.asarray(self._hand_anchor.p)))
        r_now = Rotation.from_quat(hand.q).as_matrix()
        r_anchor = Rotation.from_quat(self._hand_anchor.q).as_matrix()
        delta_x = r_now @ r_anchor.T
        delta_b = Rotation.from_matrix(self.r_bx @ delta_x @ self.r_bx.T)
        if self.rotation_scale != 1.0:
            slerp = Slerp([0.0, 1.0], Rotation.concatenate([Rotation.identity(), delta_b]))
            delta_b = slerp([self.rotation_scale])[0]
        target_rotation = delta_b * Rotation.from_quat(self._tcp_anchor.q)
        return Pose(p=tuple(p), q=tuple(target_rotation.as_quat()))
```

- [ ] **Step 4: Run mapper tests and all contract tests**

Run: `cd D:\MyWork\VR4Arm\backend; python -m pytest tests\control\test_coordinate_mapper.py tests\contract -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```powershell
git add backend/app/control backend/tests/control
git commit -m "feat: add clutch coordinate mapper"
```

---

### Task 3: Implement Teleoperation State Machine and Safety Gates

**Files:**
- Create: `backend/app/control/state_machine.py`
- Create: `backend/app/control/safety.py`
- Create: `backend/tests/control/test_state_machine.py`
- Create: `backend/tests/control/test_safety.py`

**Interfaces:**
- Produces: `TeleopStateMachine.connect()`, `arm()`, `observe_grip()`, `fault()`, `disarm()`
- Produces: `SafetyLimiter.limit(previous: Pose, requested: Pose, dt: float) -> Pose`
- Produces: `SafetyViolation(code: str)`

- [ ] **Step 1: Write failing state-machine tests**

Create `backend/tests/control/test_state_machine.py`:

```python
from app.control.state_machine import TeleopStateMachine
from app.schemas.messages import TeleopMode


def test_normal_clutch_lifecycle() -> None:
    machine = TeleopStateMachine()
    machine.connect()
    assert machine.mode == TeleopMode.READY
    machine.observe_grip(False)
    machine.arm()
    machine.observe_grip(True)
    assert machine.mode == TeleopMode.ACTIVE
    machine.observe_grip(False)
    assert machine.mode == TeleopMode.HOLD


def test_recovery_requires_grip_release_and_rearm() -> None:
    machine = TeleopStateMachine()
    machine.connect(); machine.observe_grip(False); machine.arm(); machine.observe_grip(True)
    machine.stale()
    assert machine.mode == TeleopMode.STALE
    machine.stop_complete()
    assert machine.mode == TeleopMode.DISARMED
    machine.observe_grip(True)
    assert machine.can_arm is False
    machine.observe_grip(False)
    assert machine.can_arm is True


def test_hold_regrip_returns_active_without_global_rearm() -> None:
    machine = TeleopStateMachine()
    machine.connect(); machine.observe_grip(False); machine.arm(); machine.observe_grip(True)
    machine.observe_grip(False); assert machine.mode == TeleopMode.HOLD
    machine.observe_grip(True); assert machine.mode == TeleopMode.ACTIVE


def test_disconnect_never_preserves_armed_state() -> None:
    machine = TeleopStateMachine()
    machine.connect(); machine.observe_grip(False); machine.arm()
    machine.disconnect()
    machine.connect()
    assert machine.mode == TeleopMode.READY
```

- [ ] **Step 2: Write failing safety tests**

Create `backend/tests/control/test_safety.py`:

```python
import math
import pytest
from scipy.spatial.transform import Rotation

from app.control.safety import SafetyLimiter, SafetyViolation
from app.schemas.messages import Pose


def test_first_translation_tick_obeys_acceleration_limit() -> None:
    limiter = SafetyLimiter()
    previous = Pose(p=(0, 0, 0), q=(0, 0, 0, 1))
    requested = Pose(p=(1, 0, 0), q=(0, 0, 0, 1))
    assert limiter.limit(previous, requested, 0.02).p == pytest.approx((0.00016, 0, 0))


def test_first_rotation_tick_obeys_angular_acceleration_limit() -> None:
    limiter = SafetyLimiter()
    previous = Pose(p=(0, 0, 0), q=(0, 0, 0, 1))
    requested = Pose(p=(0, 0, 0), q=tuple(Rotation.from_rotvec([0, 0, 1]).as_quat()))
    actual = Rotation.from_quat(limiter.limit(previous, requested, 0.02).q)
    assert actual.magnitude() == pytest.approx(0.00048)


def test_rejects_anchor_envelope_violation() -> None:
    limiter = SafetyLimiter(anchor=(0, 0, 0))
    with pytest.raises(SafetyViolation, match="workspace_violation"):
        limiter.limit(Pose(p=(0,0,0), q=(0,0,0,1)), Pose(p=(0.26,0,0), q=(0,0,0,1)), 0.02)
```

- [ ] **Step 3: Run tests and verify missing modules**

Run: `cd D:\MyWork\VR4Arm\backend; python -m pytest tests\control\test_state_machine.py tests\control\test_safety.py -q`
Expected: FAIL on missing modules.

- [ ] **Step 4: Implement the complete state machine**

Create `backend/app/control/state_machine.py`:

```python
from app.schemas.messages import TeleopMode


class TeleopStateMachine:
    def __init__(self) -> None:
        self.mode = TeleopMode.DISCONNECTED
        self._grip_released = False

    @property
    def can_arm(self) -> bool:
        return self.mode == TeleopMode.READY and self._grip_released

    def connect(self) -> None:
        self.mode = TeleopMode.READY
        self._grip_released = False

    def disconnect(self) -> None:
        self.mode = TeleopMode.DISCONNECTED
        self._grip_released = False

    def observe_grip(self, pressed: bool) -> None:
        if not pressed:
            self._grip_released = True
            if self.mode == TeleopMode.DISARMED:
                self.mode = TeleopMode.READY
            elif self.mode == TeleopMode.ACTIVE:
                self.mode = TeleopMode.HOLD
        elif self.mode in {TeleopMode.ARMED, TeleopMode.HOLD}:
            self.mode = TeleopMode.ACTIVE

    def arm(self) -> None:
        if not self.can_arm:
            raise RuntimeError("arm_requires_grip_release")
        self.mode = TeleopMode.ARMED

    def stale(self) -> None:
        self.mode = TeleopMode.STALE
        self._grip_released = False

    def stop_complete(self) -> None:
        if self.mode not in {TeleopMode.STALE, TeleopMode.FAULT}:
            raise RuntimeError("stop_complete_requires_stale_or_fault")
        self.mode = TeleopMode.DISARMED

    def fault(self) -> None:
        self.mode = TeleopMode.FAULT
        self._grip_released = False

    def disarm(self) -> None:
        self.mode = TeleopMode.DISARMED
        self._grip_released = False
```

- [ ] **Step 5: Implement the complete safety limiter**

Create `backend/app/control/safety.py`:

```python
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose


class SafetyViolation(RuntimeError):
    pass


class SafetyLimiter:
    def __init__(self, anchor: tuple[float, float, float] | None = None, max_linear_speed=0.15, max_angular_speed=0.6, max_linear_accel=0.4, max_angular_accel=1.2, envelope=0.25):
        self.anchor = np.asarray(anchor, dtype=float) if anchor is not None else None
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_linear_accel = max_linear_accel
        self.max_angular_accel = max_angular_accel
        self.envelope = envelope
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)

    def set_anchor(self, anchor: tuple[float, float, float]) -> None:
        self.anchor = np.asarray(anchor, dtype=float)
        self.linear_velocity[:] = 0
        self.angular_velocity[:] = 0

    def limit(self, previous: Pose, requested: Pose, dt: float) -> Pose:
        target = np.asarray(requested.p)
        if self.anchor is not None and np.any(np.abs(target - self.anchor) > self.envelope):
            raise SafetyViolation("workspace_violation")
        start = np.asarray(previous.p)
        desired_velocity = (target - start) / dt
        speed = float(np.linalg.norm(desired_velocity))
        if speed > self.max_linear_speed: desired_velocity *= self.max_linear_speed / speed
        velocity_delta = desired_velocity - self.linear_velocity
        velocity_delta_norm = float(np.linalg.norm(velocity_delta)); max_dv = self.max_linear_accel * dt
        if velocity_delta_norm > max_dv: velocity_delta *= max_dv / velocity_delta_norm
        self.linear_velocity += velocity_delta
        delta = self.linear_velocity * dt
        start_r = Rotation.from_quat(previous.q)
        requested_r = Rotation.from_quat(requested.q)
        relative = requested_r * start_r.inv()
        rotvec = relative.as_rotvec()
        angle = float(np.linalg.norm(rotvec))
        desired_angular_velocity = rotvec / dt
        desired_speed = float(np.linalg.norm(desired_angular_velocity))
        if desired_speed > self.max_angular_speed: desired_angular_velocity *= self.max_angular_speed / desired_speed
        angular_delta = desired_angular_velocity - self.angular_velocity
        angular_delta_norm = float(np.linalg.norm(angular_delta)); max_da = self.max_angular_accel * dt
        if angular_delta_norm > max_da: angular_delta *= max_da / angular_delta_norm
        self.angular_velocity += angular_delta
        limited_r = Rotation.from_rotvec(self.angular_velocity * dt) * start_r
        return Pose(p=tuple(start + delta), q=tuple(limited_r.as_quat()))
```

- [ ] **Step 6: Run tests and commit**

Run: `cd D:\MyWork\VR4Arm\backend; python -m pytest tests\control -q`
Expected: all PASS.

```powershell
git add backend/app/control backend/tests/control
git commit -m "feat: add teleop safety state machine"
```

---

### Task 4: Implement Theoretical LM3 Forward Kinematics

**Files:**
- Create: `backend/app/sim/lm3_model.py`
- Create: `backend/app/sim/kinematics.py`
- Create: `backend/tests/sim/test_kinematics.py`

**Interfaces:**
- Produces: `LM3Model.home_q`, `LM3Model.tcp_offset_m`, `LM3Model.joint_window_rad`
- Produces: `forward_matrix(q: Sequence[float], model: LM3Model) -> numpy.ndarray`
- Produces: `forward_pose(q: Sequence[float], model: LM3Model) -> Pose`

- [ ] **Step 1: Write failing FK tests**

Create `backend/tests/sim/test_kinematics.py`:

```python
import numpy as np
import pytest

from app.sim.kinematics import forward_matrix, forward_pose
from app.sim.lm3_model import LM3Model


def test_fk_returns_rigid_transform_and_pose() -> None:
    model = LM3Model()
    matrix = forward_matrix(model.home_q, model)
    pose = forward_pose(model.home_q, model)
    assert matrix.shape == (4, 4)
    assert np.allclose(matrix[3], [0, 0, 0, 1])
    assert np.linalg.det(matrix[:3, :3]) == pytest.approx(1.0)
    assert sum(value * value for value in pose.q) == pytest.approx(1.0)


def test_fk_rejects_wrong_joint_count() -> None:
    with pytest.raises(ValueError, match="six joints"):
        forward_pose([0, 0], LM3Model())


def test_fk_is_repeatable() -> None:
    q = [0.2, -0.5, 0.8, -0.2, 0.4, 0.1]
    assert forward_pose(q, LM3Model()) == forward_pose(q, LM3Model())
```

- [ ] **Step 2: Run and verify missing module failure**

Run: `cd D:\MyWork\VR4Arm\backend; python -m pytest tests\sim\test_kinematics.py -q`
Expected: FAIL on missing `app.sim.kinematics`.

- [ ] **Step 3: Implement model constants and modified-DH FK**

Create `backend/app/sim/lm3_model.py`:

```python
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LM3Model:
    home_q: tuple[float, ...] = (0.0, -0.7853981634, 1.5707963268, -0.7853981634, 1.5707963268, 0.0)
    a_prev_m: tuple[float, ...] = (0.0, 0.0, -0.28, -0.26, 0.0, 0.0)
    alpha_prev_rad: tuple[float, ...] = (0.0, math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2)
    d_m: tuple[float, ...] = (0.21583, 0.0, 0.0, 0.12063, 0.09833, 0.08343)
    tcp_offset_m: tuple[float, float, float] = (0.0, 0.0, 0.09)
    joint_window_rad: float = math.pi
    max_joint_speed_radps: float = 0.5
    max_joint_accel_radps2: float = 1.0
```

Create `backend/app/sim/kinematics.py`:

```python
from collections.abc import Sequence
import numpy as np
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose
from app.sim.lm3_model import LM3Model


def _mdh(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    ca, sa, ct, st = np.cos(alpha), np.sin(alpha), np.cos(theta), np.sin(theta)
    return np.array([[ct, -st, 0, a], [st * ca, ct * ca, -sa, -d * sa], [st * sa, ct * sa, ca, d * ca], [0, 0, 0, 1]], dtype=float)


def forward_matrix(q: Sequence[float], model: LM3Model) -> np.ndarray:
    if len(q) != 6:
        raise ValueError("LM3 requires six joints")
    transform = np.eye(4)
    for theta, d, a, alpha in zip(q, model.d_m, model.a_prev_m, model.alpha_prev_rad):
        transform = transform @ _mdh(float(theta), d, a, alpha)
    tcp = np.eye(4)
    tcp[:3, 3] = model.tcp_offset_m
    return transform @ tcp


def forward_pose(q: Sequence[float], model: LM3Model) -> Pose:
    matrix = forward_matrix(q, model)
    return Pose(p=tuple(matrix[:3, 3]), q=tuple(Rotation.from_matrix(matrix[:3, :3]).as_quat()))
```

- [ ] **Step 4: Run FK tests and commit**

Run: `cd D:\MyWork\VR4Arm\backend; python -m pytest tests\sim\test_kinematics.py -q`
Expected: `3 passed`.

```powershell
git add backend/app/sim backend/tests/sim
git commit -m "feat: add LM3 theoretical forward kinematics"
```

---

### Task 5: Implement Six-Dimensional Damped-Least-Squares IK

**Files:**
- Create: `backend/app/sim/ik.py`
- Create: `backend/tests/sim/test_ik.py`
- Create: `scripts/generate_reachability_fixture.py`
- Create: `schemas/fixtures/sim-reachability-v1.json`

**Interfaces:**
- Consumes: `forward_pose`, `LM3Model`, `Pose`
- Produces: `IKResult(q, position_error_m, orientation_error_rad, iterations)`
- Produces: `solve_ik(target: Pose, seed_q: Sequence[float], model: LM3Model) -> IKResult`
- Raises: `IKError(code)` with `ik_unreachable`, `ik_singular`, or `joint_safety_window`

- [ ] **Step 1: Write failing IK tests**

Create `backend/tests/sim/test_ik.py`:

```python
import numpy as np
import pytest

from app.sim.ik import IKError, solve_ik
from app.sim.kinematics import forward_pose
from app.sim.lm3_model import LM3Model


def test_ik_recovers_known_nearby_joint_pose() -> None:
    model = LM3Model()
    expected = np.asarray(model.home_q) + np.array([0.05, -0.03, 0.04, 0.02, -0.02, 0.03])
    result = solve_ik(forward_pose(expected, model), model.home_q, model)
    assert result.position_error_m <= 0.002
    assert result.orientation_error_rad <= np.deg2rad(1.0)


def test_ik_rejects_far_unreachable_target() -> None:
    model = LM3Model()
    target = forward_pose(model.home_q, model).model_copy(update={"p": (10.0, 10.0, 10.0)})
    with pytest.raises(IKError, match="ik_unreachable"):
        solve_ik(target, model.home_q, model)


def test_ik_is_deterministic() -> None:
    model = LM3Model()
    target = forward_pose(np.asarray(model.home_q) + 0.02, model)
    assert solve_ik(target, model.home_q, model).q == solve_ik(target, model.home_q, model).q
```

- [ ] **Step 2: Run and verify missing module failure**

Run: `cd D:\MyWork\VR4Arm\backend; python -m pytest tests\sim\test_ik.py -q`
Expected: FAIL on missing `app.sim.ik`.

- [ ] **Step 3: Implement complete finite-difference DLS IK**

Create `backend/app/sim/ik.py`:

```python
from dataclasses import dataclass
from collections.abc import Sequence
import numpy as np
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose
from app.sim.kinematics import forward_pose
from app.sim.lm3_model import LM3Model


class IKError(RuntimeError):
    pass


@dataclass(frozen=True)
class IKResult:
    q: tuple[float, float, float, float, float, float]
    position_error_m: float
    orientation_error_rad: float
    iterations: int


def _error(target: Pose, current: Pose) -> np.ndarray:
    position = np.asarray(target.p) - np.asarray(current.p)
    rotation = (Rotation.from_quat(target.q) * Rotation.from_quat(current.q).inv()).as_rotvec()
    return np.concatenate([position, rotation])


def _jacobian(q: np.ndarray, model: LM3Model, epsilon: float = 1e-5) -> np.ndarray:
    columns = []
    for index in range(6):
        plus, minus = q.copy(), q.copy()
        plus[index] += epsilon; minus[index] -= epsilon
        delta = _error(forward_pose(plus, model), forward_pose(minus, model)) / (2 * epsilon)
        columns.append(delta)
    return np.column_stack(columns)


def solve_ik(target: Pose, seed_q: Sequence[float], model: LM3Model, max_iterations: int = 30) -> IKResult:
    q_ref = np.asarray(model.home_q)
    q = np.asarray(seed_q, dtype=float).copy()
    if q.shape != (6,):
        raise IKError("invalid_joint_count")
    for iteration in range(1, max_iterations + 1):
        current = forward_pose(q, model)
        error = _error(target, current)
        position_error = float(np.linalg.norm(error[:3]))
        orientation_error = float(np.linalg.norm(error[3:]))
        if position_error <= 0.002 and orientation_error <= np.deg2rad(1.0):
            return IKResult(tuple(float(v) for v in q), position_error, orientation_error, iteration)
        jacobian = _jacobian(q, model)
        if not np.all(np.isfinite(jacobian)):
            raise IKError("ik_singular")
        damping = 0.04
        delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping * damping * np.eye(6), error)
        delta = np.clip(delta, -0.12, 0.12)
        candidate = q + delta
        if np.any(np.abs(candidate - q_ref) > model.joint_window_rad):
            raise IKError("joint_safety_window")
        q = candidate
    raise IKError("ik_unreachable")
```

- [ ] **Step 4: Generate deterministic reachability fixture and test it**

Create `scripts/generate_reachability_fixture.py` in the implementation task with NumPy RNG seed `42`, sample 1000 joint vectors within `home_q ± 0.35`, run FK, and write pose plus a seed perturbed by at most `0.05 rad` to `schemas/fixtures/sim-reachability-v1.json`. Add a parametrized test that loads all entries, calls `solve_ik`, counts successes, and asserts `successes >= 990`.

Exact generator body:

```python
import json
from pathlib import Path
import numpy as np
from app.sim.kinematics import forward_pose
from app.sim.lm3_model import LM3Model

rng = np.random.default_rng(42); model = LM3Model(); home = np.asarray(model.home_q); rows = []
for _ in range(1000):
    q = home + rng.uniform(-0.35, 0.35, 6); seed = q + rng.uniform(-0.05, 0.05, 6)
    rows.append({"target": forward_pose(q, model).model_dump(mode="json"), "seed": seed.tolist()})
Path("schemas/fixtures/sim-reachability-v1.json").write_text(json.dumps(rows, separators=(",", ":")), encoding="utf-8")
```

Run the generator from the repository root with:

```powershell
cd D:\MyWork\VR4Arm
$env:PYTHONPATH='backend'
python scripts\generate_reachability_fixture.py
```

- [ ] **Step 5: Run IK suite and commit**

Run: `cd D:\MyWork\VR4Arm\backend; python -m pytest tests\sim\test_ik.py -q`
Expected: all PASS including 99% fixture threshold.

```powershell
git add backend/app/sim/ik.py backend/tests/sim/test_ik.py scripts/generate_reachability_fixture.py schemas/fixtures/sim-reachability-v1.json
git commit -m "feat: add six dimensional LM3 inverse kinematics"
```

---

### Task 6: Implement RobotBackend, Fixed-Step Virtual Robot, and Disabled Real Adapter

**Files:**
- Create: `backend/app/robots/base.py`
- Create: `backend/app/robots/sim_adapter.py`
- Create: `backend/app/robots/lebai_adapter.py`
- Create: `backend/app/sim/virtual_robot.py`
- Create: `backend/tests/sim/test_virtual_robot.py`
- Create: `backend/tests/sim/test_adapters.py`

**Interfaces:**
- Produces: `StopReason`, `BackendCommandError`, `RobotBackend`
- Produces: `VirtualRobot.step(dt: float) -> None`
- Produces: `SimRobotAdapter.start()`, `command_tcp()`, `set_gripper()`, `stop()`, `get_state()`
- Produces: `RealLebaiAdapter` whose public methods all raise `BackendCommandError("real_robot_disabled")`

- [ ] **Step 1: Write failing virtual dynamics and adapter tests**

Create `backend/tests/sim/test_virtual_robot.py`:

```python
import numpy as np
import pytest

from app.sim.kinematics import forward_pose
from app.sim.lm3_model import LM3Model
from app.sim.virtual_robot import VirtualRobot


def test_virtual_robot_moves_over_time_without_jumping() -> None:
    model = LM3Model(); robot = VirtualRobot(model)
    goal_q = np.asarray(model.home_q) + 0.2
    robot.set_target_q(goal_q)
    robot.step(0.02)
    assert np.max(np.abs(np.asarray(robot.q) - np.asarray(model.home_q))) <= 0.00041
    for _ in range(600): robot.step(0.02)
    assert robot.q == pytest.approx(goal_q, abs=2e-3)


def test_stop_invalidates_target_and_decelerates_to_zero() -> None:
    robot = VirtualRobot(LM3Model()); robot.set_target_q(np.asarray(robot.model.home_q) + 0.5)
    for _ in range(20): robot.step(0.02)
    robot.stop()
    for _ in range(30): robot.step(0.02)
    assert robot.target_q is None
    assert robot.qd == pytest.approx([0] * 6, abs=1e-8)
```

Create `backend/tests/sim/test_adapters.py`:

```python
import pytest
from app.robots.base import BackendCommandError
from app.robots.lebai_adapter import RealLebaiAdapter
from app.robots.sim_adapter import SimRobotAdapter
from app.sim.kinematics import forward_pose


@pytest.mark.asyncio
async def test_sim_adapter_accepts_tcp_and_gripper_commands() -> None:
    adapter = SimRobotAdapter()
    await adapter.connect(); target = forward_pose(adapter.robot.model.home_q, adapter.robot.model)
    await adapter.command_tcp(target, command_id=1); await adapter.set_gripper(0.7)
    state = await adapter.get_state()
    assert state.gripper == pytest.approx(0.7)


@pytest.mark.asyncio
async def test_real_adapter_is_hard_disabled() -> None:
    with pytest.raises(BackendCommandError, match="real_robot_disabled"):
        await RealLebaiAdapter().connect()
```

- [ ] **Step 2: Run tests and verify missing modules**

Run: `cd D:\MyWork\VR4Arm\backend; python -m pytest tests\sim\test_virtual_robot.py tests\sim\test_adapters.py -q`
Expected: FAIL on missing modules.

- [ ] **Step 3: Define backend protocol and disabled real adapter**

Create `backend/app/robots/base.py`:

```python
from enum import StrEnum
from typing import Protocol
from app.schemas.messages import Pose, RobotStateMessage


class StopReason(StrEnum):
    GRIP_RELEASED = "grip_released"; STALE = "stale"; DISCONNECT = "disconnect"; FAULT = "fault"; SHUTDOWN = "shutdown"


class BackendCommandError(RuntimeError):
    pass


class RobotBackend(Protocol):
    async def connect(self) -> None: ...
    async def disconnect(self) -> None: ...
    async def command_tcp(self, target: Pose, command_id: int) -> None: ...
    async def set_gripper(self, value: float) -> None: ...
    async def stop(self, reason: StopReason) -> None: ...
    async def get_state(self) -> RobotStateMessage: ...
```

Create `backend/app/robots/lebai_adapter.py`:

```python
from app.robots.base import BackendCommandError


class RealLebaiAdapter:
    def _disabled(self): raise BackendCommandError("real_robot_disabled")
    async def connect(self): self._disabled()
    async def disconnect(self): self._disabled()
    async def command_tcp(self, target, command_id): self._disabled()
    async def set_gripper(self, value): self._disabled()
    async def stop(self, reason): self._disabled()
    async def get_state(self): self._disabled()
```

- [ ] **Step 4: Implement complete fixed-step virtual robot**

Create `backend/app/sim/virtual_robot.py`:

```python
from __future__ import annotations
import numpy as np
from app.sim.lm3_model import LM3Model


class VirtualRobot:
    def __init__(self, model: LM3Model):
        self.model = model; self.q = np.asarray(model.home_q, dtype=float); self.qd = np.zeros(6); self.target_q: np.ndarray | None = None

    def set_target_q(self, target) -> None:
        target = np.asarray(target, dtype=float)
        if target.shape != (6,) or np.any(np.abs(target - np.asarray(self.model.home_q)) > self.model.joint_window_rad):
            raise ValueError("joint_safety_window")
        self.target_q = target.copy()

    def stop(self) -> None:
        self.target_q = None

    def step(self, dt: float) -> None:
        if self.target_q is None:
            desired_qd = np.zeros(6)
        else:
            desired_qd = np.clip(2.0 * (self.target_q - self.q), -self.model.max_joint_speed_radps, self.model.max_joint_speed_radps)
        max_dv = self.model.max_joint_accel_radps2 * dt
        self.qd += np.clip(desired_qd - self.qd, -max_dv, max_dv)
        self.q += self.qd * dt
        if self.target_q is not None and np.max(np.abs(self.target_q - self.q)) < 1e-4:
            self.q = self.target_q.copy(); self.qd[:] = 0
```

- [ ] **Step 5: Implement complete simulator adapter with an absolute-deadline task**

Create `backend/app/robots/sim_adapter.py` with `SimRobotAdapter.__init__` owning `LM3Model`, `VirtualRobot`, gripper, command ID, mode and lock; `connect()` starts `_run()`; `_run()` increments `next_deadline += 0.02`, calls `robot.step(0.02)`, and sleeps `max(0,next_deadline-loop.time())`; `command_tcp()` calls `solve_ik(target, robot.q, model)` and `robot.set_target_q(result.q)`; `stop()` calls `robot.stop()`; `get_state()` returns `RobotStateMessage` using `forward_pose(robot.q, model)` and `BackendState.MOVING` when any `abs(qd)>1e-4` else `IDLE`. Use `asyncio.Lock` around mutations and `contextlib.suppress(asyncio.CancelledError)` during disconnect.

Required `get_state()` construction:

```python
return RobotStateMessage(server_mono_ns=time.monotonic_ns(), ack_seq=self.command_id,
    mode=TeleopMode.READY, robot_state=backend_state, actual_tcp=forward_pose(self.robot.q, self.model),
    actual_q=tuple(float(v) for v in self.robot.q), gripper=self.gripper, sample_age_ms=None, fault=self.fault)
```

- [ ] **Step 6: Run simulator suites and commit**

Run: `cd D:\MyWork\VR4Arm\backend; python -m pytest tests\sim -q`
Expected: all PASS.

```powershell
git add backend/app/robots backend/app/sim/virtual_robot.py backend/tests/sim
git commit -m "feat: add fixed step virtual robot backend"
```

---

### Task 7: Implement RobotControl, Latest-Value Input, Watchdog, and Recorder Interfaces

**Files:**
- Create: `backend/app/control/filters.py`
- Create: `backend/app/control/robot_control.py`
- Create: `backend/app/timebase.py`
- Create: `backend/app/recording/base.py`
- Create: `backend/app/recording/noop.py`
- Create: `backend/tests/control/test_robot_control.py`
- Create: `backend/tests/control/test_filters.py`

**Interfaces:**
- Consumes: `RobotBackend`, `CoordinateMapper`, `SafetyLimiter`, `TeleopStateMachine`, `VRFrame`
- Produces: `LatestVRFrame.publish(frame, received_ns)`, `snapshot()`
- Produces: `RobotControl.start()`, `stop()`, `arm()`, `disarm()`, `on_disconnect()`
- Produces: `RecorderSink` and `NoopRecorder`
- Produces: `PoseFilter.reset(pose)`, `update(pose, dt) -> Pose`

- [ ] **Step 1: Write failing RobotControl tests with a fake backend**

Create `backend/tests/control/test_robot_control.py` with these complete helpers before the tests:

```python
import pytest
from app.control.robot_control import LatestVRFrame, RobotControl
from app.recording.noop import NoopRecorder
from app.robots.base import StopReason
from app.schemas.messages import BackendState, ControllerState, Pose, RobotStateMessage, TeleopMode, VRFrame

class FakeClock:
    def __init__(self): self.value = 1_000_000_000
    def now_ns(self): return self.value
    def advance_ms(self, value): self.value += int(value * 1_000_000)

class FakeBackend:
    def __init__(self): self.targets=[]; self.stops=[]; self.gripper=0.0
    async def connect(self): return None
    async def disconnect(self): return None
    async def command_tcp(self, target, command_id): self.targets.append((command_id, target))
    async def set_gripper(self, value): self.gripper=value
    async def stop(self, reason): self.stops.append(reason)
    async def get_state(self):
        return RobotStateMessage(server_mono_ns=1, mode=TeleopMode.READY, robot_state=BackendState.IDLE,
            actual_tcp=Pose(p=(0.3,0.0,0.3),q=(0,0,0,1)), actual_q=(0,0,0,0,0,0), gripper=self.gripper)

def frame(seq, grip, p=(0.0,1.2,-0.3)):
    return VRFrame(v=1,type="vr_frame",session_id="test",seq=seq,client_mono_ms=float(seq),tracking_valid=True,
        visibility="visible",right=ControllerState(p=p,q=(0,0,0,1),grip=grip,trigger=0.0))

def make_control():
    latest=LatestVRFrame(); backend=FakeBackend(); clock=FakeClock()
    control=RobotControl(backend=backend,latest=latest,clock=clock,recorder=NoopRecorder())
    return control,latest,backend,clock
```

Create `backend/tests/control/test_filters.py`:

```python
import pytest
from scipy.spatial.transform import Rotation
from app.control.filters import PoseFilter
from app.schemas.messages import Pose

def test_pose_filter_reset_has_no_jump() -> None:
    value=Pose(p=(1,2,3),q=(0,0,0,1));f=PoseFilter();f.reset(value);assert f.update(value,0.02)==value

def test_pose_filter_moves_toward_position_and_rotation() -> None:
    f=PoseFilter(cutoff_hz=8);f.reset(Pose(p=(0,0,0),q=(0,0,0,1)))
    target=Pose(p=(1,0,0),q=tuple(Rotation.from_euler('z',90,degrees=True).as_quat()));actual=f.update(target,0.02)
    assert 0 < actual.p[0] < 1
    assert 0 < Rotation.from_quat(actual.q).magnitude() < Rotation.from_quat(target.q).magnitude()
```

Then add the tests:

```python
@pytest.mark.asyncio
async def test_only_latest_frame_is_consumed():
    control, latest, backend, clock = make_control()
    latest.publish(frame(seq=1, grip=False), clock.now_ns()); latest.publish(frame(seq=2, grip=False), clock.now_ns())
    await control.tick(); assert control.last_seq == 2

@pytest.mark.asyncio
async def test_active_frame_captures_anchor_then_commands_target():
    control, latest, backend, clock = make_control(); await control.connect()
    latest.publish(frame(seq=1, grip=False), clock.now_ns()); await control.tick(); await control.arm()
    latest.publish(frame(seq=2, grip=True), clock.now_ns()); await control.tick()
    latest.publish(frame(seq=3, grip=True, p=(0,1.2,-0.31)), clock.now_ns()); await control.tick()
    assert len(backend.targets) == 1

@pytest.mark.asyncio
async def test_stale_input_stops_and_requires_release():
    control, latest, backend, clock = make_control(); await control.connect()
    latest.publish(frame(1,False),clock.now_ns()); await control.tick(); await control.arm()
    latest.publish(frame(2,True),clock.now_ns()); await control.tick(); clock.advance_ms(101)
    await control.tick(); assert backend.stops[-1] == StopReason.STALE; assert control.mode == TeleopMode.STALE

@pytest.mark.asyncio
async def test_hard_stale_disarms_at_250ms():
    control,latest,backend,clock=make_control();await control.connect();latest.publish(frame(1,False),clock.now_ns());await control.tick();clock.advance_ms(251);await control.tick();assert control.mode==TeleopMode.DISARMED

@pytest.mark.asyncio
async def test_trigger_is_rate_and_delta_limited():
    control,latest,backend,clock=make_control();await control.connect();latest.publish(frame(1,False).model_copy(update={"right":frame(1,False).right.model_copy(update={"trigger":0.7})}),clock.now_ns());await control.tick();assert backend.gripper==pytest.approx(0.7)
```

- [ ] **Step 2: Run and verify missing RobotControl**

Run: `cd D:\MyWork\VR4Arm\backend; python -m pytest tests\control\test_robot_control.py -q`
Expected: FAIL on missing module.

- [ ] **Step 3: Implement timebase, recorder protocols, and latest-value holder**

Create `backend/app/timebase.py` with `MonotonicClock.now_ns() -> time.monotonic_ns()` and a test-only `FakeClock` in the test file. Create `RecorderSink` protocol with async `write_vr_frame`, `write_robot_state`, `write_event`, `write_camera_frame`; create `NoopRecorder` whose four methods return `None`.

In `robot_control.py`, implement:

```python
@dataclass(frozen=True)
class ReceivedFrame:
    frame: VRFrame
    received_ns: int

class LatestVRFrame:
    def __init__(self): self._value: ReceivedFrame | None = None
    def publish(self, frame, received_ns):
        if self._value is None or frame.session_id != self._value.frame.session_id or frame.seq > self._value.frame.seq:
            self._value = ReceivedFrame(frame, received_ns)
    def snapshot(self): return self._value
```

`RobotControl.__init__` has the exact signature `RobotControl(*, backend: RobotBackend, latest: LatestVRFrame, clock: MonotonicClock, recorder: RecorderSink)` and initializes `last_seq=None`, `anchor_seq=None`, `last_target=None`, `CoordinateMapper()`, `SafetyLimiter()`, and `TeleopStateMachine()`.

- [ ] **Step 4: Implement the low-latency pose filter**

Create `backend/app/control/filters.py`:

```python
import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from app.schemas.messages import Pose

class PoseFilter:
    def __init__(self, cutoff_hz: float = 8.0): self.cutoff_hz=cutoff_hz; self.value:Pose|None=None
    def reset(self, pose: Pose): self.value=pose.model_copy(deep=True)
    def update(self, pose: Pose, dt: float) -> Pose:
        if self.value is None: self.reset(pose); return pose
        alpha=1-math.exp(-2*math.pi*self.cutoff_hz*dt)
        p=np.asarray(self.value.p)+alpha*(np.asarray(pose.p)-np.asarray(self.value.p))
        rotations=Rotation.concatenate([Rotation.from_quat(self.value.q),Rotation.from_quat(pose.q)])
        q=Slerp([0.0,1.0],rotations)([alpha])[0].as_quat()
        self.value=Pose(p=tuple(p),q=tuple(q));return self.value
```

Reset this filter to the actual TCP every time a new Grip anchor is captured. Apply `requested = self.filter.update(self.mapper.target(...), 0.02)` before `SafetyLimiter.limit()`.

- [ ] **Step 5: Implement RobotControl tick semantics**

`tick()` must: reject missing/hidden/untracked frames; compute age from received PC timestamp; stop at 100 ms; observe Grip; capture anchor on `ARMED/HOLD -> ACTIVE`; clear and stop on `ACTIVE -> HOLD`; map and limit only after the anchor frame; command the newest gripper value at no more than 10 Hz and only when its delta from the last sent value is `>0.02`; never queue historical values; publish state with current mode and sample age. `run()` uses absolute deadlines; lateness relative to the absolute deadline greater than 40 ms counts as an overrun, and two consecutive overruns fault and stop. `stop()` is idempotent and calls backend stop with SHUTDOWN.

Use this exact ordering in `tick()`:

```python
received = self.latest.snapshot()
if received is None: return
age_ms = (self.clock.now_ns() - received.received_ns) / 1_000_000
if age_ms >= 250:
    await self._safe_stop(StopReason.STALE); self.machine.disarm(); return
if age_ms >= 100 or not received.frame.tracking_valid or received.frame.visibility != "visible":
    await self._safe_stop(StopReason.STALE); return
previous_mode = self.machine.mode
self.machine.observe_grip(received.frame.right.grip)
if previous_mode in {TeleopMode.ARMED, TeleopMode.HOLD} and self.machine.mode == TeleopMode.ACTIVE:
    state = await self.backend.get_state(); self.mapper.capture(Pose(p=received.frame.right.p, q=received.frame.right.q), state.actual_tcp); self.limiter.set_anchor(state.actual_tcp.p); self.filter.reset(state.actual_tcp); self.last_target = state.actual_tcp; self.anchor_seq = received.frame.seq
elif previous_mode == TeleopMode.ACTIVE and self.machine.mode == TeleopMode.HOLD:
    self.mapper.clear(); await self.backend.stop(StopReason.GRIP_RELEASED)
elif self.machine.mode == TeleopMode.ACTIVE and received.frame.seq != self.anchor_seq:
    raw_requested = self.mapper.target(Pose(p=received.frame.right.p, q=received.frame.right.q))
    requested = self.filter.update(raw_requested, 0.02)
    target = self.limiter.limit(self.last_target, requested, 0.02)
    await self.backend.command_tcp(target, received.frame.seq); self.last_target = target
self.last_seq = received.frame.seq
```

`state_message()` reads the backend state and returns a copy with RobotControl's current `mode`, latest `ack_seq`, and sample age. `STALE` or `FAULT` must be returned in at least one 20 Hz state message before a stopped backend may call `machine.stop_complete()` and transition to `DISARMED`. This is the only STALE/FAULT to DISARMED completion path.

- [ ] **Step 6: Run control tests and commit**

Run: `cd D:\MyWork\VR4Arm\backend; python -m pytest tests\control -q`
Expected: all PASS.

```powershell
git add backend/app/control backend/app/recording backend/app/timebase.py backend/tests/control
git commit -m "feat: add fixed rate robot control loop"
```

---

### Task 8: Expose FastAPI Health and Teleoperation WebSocket

**Files:**
- Create: `backend/app/config.py`
- Create: `backend/app/api/health.py`
- Create: `backend/app/api/teleop_ws.py`
- Create: `backend/app/main.py`
- Create: `backend/tests/api/test_health.py`
- Create: `backend/tests/api/test_teleop_ws.py`

**Interfaces:**
- Produces: `create_app() -> FastAPI`
- Produces: `GET /health`
- Produces: `WS /ws/v1/teleop`
- Consumes JSON messages: `VRFrame | ClientControlMessage`

- [ ] **Step 1: Write failing API tests**

Create `backend/tests/api/test_health.py`:

```python
from fastapi.testclient import TestClient
from app.main import create_app

def test_health_is_simulator_only() -> None:
    with TestClient(create_app()) as client:
        assert client.get("/health").json() == {"status":"ok","backend":"SIMULATOR","real_robot_enabled":False}
```

Create `backend/tests/api/test_teleop_ws.py`:

```python
import json
from pathlib import Path
from fastapi.testclient import TestClient
from app.main import create_app

ROOT=Path(__file__).resolve().parents[3]

def test_hello_and_frame_ack() -> None:
    with TestClient(create_app()) as client, client.websocket_connect("/ws/v1/teleop") as ws:
        ws.send_json({"v":1,"type":"hello","request_id":"h1"})
        assert ws.receive_json()["type"] == "hello_ack"
        payload=json.loads((ROOT/"schemas/fixtures/vr-frame-valid.json").read_text())
        ws.send_json(payload)
        messages=[ws.receive_json() for _ in range(3)]
        assert any(item.get("type")=="robot_state" and item.get("ack_seq")==1 for item in messages)

def test_arm_before_grip_release_is_rejected() -> None:
    with TestClient(create_app()) as client, client.websocket_connect("/ws/v1/teleop") as ws:
        ws.send_json({"v":1,"type":"arm_request","request_id":"a1"})
        assert ws.receive_json()["type"] == "arm_rejected"

def test_unknown_field_closes_policy_violation() -> None:
    with TestClient(create_app()) as client, client.websocket_connect("/ws/v1/teleop") as ws:
        ws.send_json({"v":1,"type":"hello","request_id":"h1","unexpected":True})
        event=ws.receive()
        assert event["type"] == "websocket.close" and event["code"] == 1008
```

- [ ] **Step 2: Run and verify missing app failure**

Run: `cd D:\MyWork\VR4Arm\backend; python -m pytest tests\api -q`
Expected: FAIL importing `app.main`.

- [ ] **Step 3: Implement config and health endpoint**

`Settings.load()` reads `VR4ARM_CONFIG` or `../config/default.yaml`, validates `backend == "simulator"`, and raises on any other value. `/health` returns the exact payload from Step 1.

- [ ] **Step 4: Implement application lifespan**

In `create_app()`, construct one SimRobotAdapter, LatestVRFrame, RobotControl and NoopRecorder in lifespan; connect/start on startup; on shutdown call `control.stop()` then `backend.disconnect()`. Store them on `app.state`. Include the health router and websocket router.

- [ ] **Step 5: Implement WebSocket protocol**

The receive loop parses JSON dicts by `type`, validates with Pydantic, publishes VRFrame with `time.monotonic_ns()`, handles `hello`, `arm_request`, `disarm`, and `ping`; a sender task emits a RobotStateMessage every 50 ms. In `finally`, call `control.on_disconnect()`. Validation errors send `protocol_error` then close code `1008`.

Exact sender cadence:

```python
while True:
    state = await control.state_message()
    await websocket.send_json(state.model_dump(mode="json"))
    await asyncio.sleep(0.05)
```

- [ ] **Step 6: Run backend suite and commit**

Run: `cd D:\MyWork\VR4Arm\backend; python -m pytest -q`
Expected: all PASS.

```powershell
git add backend/app backend/tests/api
git commit -m "feat: expose simulator teleoperation websocket"
```

---

### Task 9: Bootstrap the TypeScript WebXR Client and Protocol Contract

**Files:**
- Create: `web/package.json`
- Create: `web/tsconfig.json`
- Create: `web/vite.config.ts`
- Create: `web/index.html`
- Create: `web/src/protocol/messages.ts`
- Create: `web/src/transport/teleopSocket.ts`
- Create: `web/tests/messages.test.ts`
- Create: `web/tests/teleopSocket.test.ts`

**Interfaces:**
- Produces TypeScript types matching protocol v1.
- Produces: `TeleopSocket.connect()`, `sendFrame()`, `sendControl()`, `close()`
- Produces callback: `onRobotState(state: RobotStateMessage)`

- [ ] **Step 1: Add package configuration and failing tests**

Use dependencies `three ^0.181.2`; dev dependencies `typescript ^5.9.3`, `vite ^7.3.0`, `vitest ^4.0.15`, `@vitejs/plugin-basic-ssl ^2.1.0`, `jsdom ^29.1.1`. Scripts: `dev`, `build`, `test`.

Create `web/tests/messages.test.ts`:

```ts
import {describe,expect,it} from 'vitest';
import vr from '../../schemas/fixtures/vr-frame-valid.json';
import robot from '../../schemas/fixtures/robot-state-valid.json';
import {isRobotStateMessage,isVRFrame} from '../src/protocol/messages';
describe('protocol guards',()=>{
  it('accepts shared fixtures',()=>{expect(isVRFrame(vr)).toBe(true);expect(isRobotStateMessage(robot)).toBe(true)});
  it('rejects versions and non-finite values',()=>{
    expect(isVRFrame({...vr,v:2})).toBe(false);
    expect(isVRFrame({...vr,right:{...vr.right,p:[0,Number.NaN,0]}})).toBe(false);
  });
});
```

Create `web/tests/teleopSocket.test.ts` with an injected WebSocket factory and fake timer:

```ts
import {beforeEach,expect,it,vi} from 'vitest';
import {TeleopSocket} from '../src/transport/teleopSocket';
import fixture from '../../schemas/fixtures/vr-frame-valid.json';
class FakeSocket {
  readyState=0; sent:string[]=[]; onopen:(()=>void)|null=null; onclose:(()=>void)|null=null; onmessage:null=null; onerror:null=null;
  send(value:string){this.sent.push(value)} close(){this.readyState=3} open(){this.readyState=1;this.onopen?.()} closeFromServer(){this.readyState=3;this.onclose?.()}
}
const validFrame=()=>fixture as never;
beforeEach(()=>vi.useFakeTimers());
it('drops frames before open and never auto-arms on reconnect',()=>{
  const sockets:FakeSocket[]=[]; const client=new TeleopSocket('wss://test',()=>{},()=>{const s=new FakeSocket();sockets.push(s);return s as unknown as WebSocket});
  client.connect(); client.sendFrame(validFrame()); expect(sockets[0].sent).toEqual([]);
  sockets[0].open(); expect(sockets[0].sent.some(x=>JSON.parse(x).type==='hello')).toBe(true);
  sockets[0].closeFromServer(); vi.advanceTimersByTime(2100);
  expect(sockets).toHaveLength(2); expect(sockets.flatMap(s=>s.sent).some(x=>JSON.parse(x).type==='arm_request')).toBe(false);
});
```

The production constructor signature is `TeleopSocket(url, onRobotState, socketFactory = url => new WebSocket(url))`.

- [ ] **Step 2: Run and verify missing modules**

Run: `cd D:\MyWork\VR4Arm\web; npm.cmd test`
Expected: FAIL importing protocol and transport modules.

- [ ] **Step 3: Implement exact protocol types and guards**

Define `Vec3`, `Quat`, `Pose`, `ControllerState`, `VRFrame`, `TeleopMode`, `BackendState`, `RobotStateMessage`, and `ClientControlMessage`. Guards verify `v===1`, exact tuple lengths, finite numbers, trigger range, six joints, and known enum strings. Reject unknown protocol versions.

- [ ] **Step 4: Implement TeleopSocket**

Constructor takes URL and callbacks. `connect()` creates WebSocket, sends `hello` only on open, parses state with guard, and schedules exponential reconnect capped at 2 s. `sendFrame()` drops the frame unless socket is OPEN; it never queues historical frames. `close()` sets explicitClose, clears timer, and closes socket. No code path automatically sends `arm_request`.

- [ ] **Step 5: Run tests/build and commit**

Run:

```powershell
cd D:\MyWork\VR4Arm\web
npm.cmd test
npm.cmd run build
```

Expected: tests PASS and Vite build succeeds.

```powershell
git add web
git commit -m "feat: add typed teleoperation web client"
```

---

### Task 10: Reuse the LM3 GLB and Render Authoritative Joint/Gripper State

**Files:**
- Copy: `D:\MyWork\ArmPlannerAgent\.worktrees\phase-b-reproducible-system\web\public\models\Lebai_LM3.glb` 鈫?`web/public/models/Lebai_LM3.glb`
- Create: `web/src/robot/robotModel.ts`
- Create: `web/src/robot/robotState.ts`
- Create: `web/tests/robotModel.test.ts`

**Interfaces:**
- Produces: `loadRobotModel() -> Promise<RobotModel>`
- Produces: `createRobotModelForTest(scene: THREE.Object3D, animations?: THREE.AnimationClip[]) -> RobotModel`
- Produces: `RobotModel.setJointAngles(q: readonly number[])`
- Produces: `RobotModel.setGripper(value: number)`
- Produces: `RobotStateBuffer.push(state)`, `sample(nowNs)`

- [ ] **Step 1: Copy model and write failing tests**

Copy the model exactly:

```powershell
New-Item -ItemType Directory -Force -Path 'D:\MyWork\VR4Arm\web\public\models'
Copy-Item -LiteralPath 'D:\MyWork\ArmPlannerAgent\.worktrees\phase-b-reproducible-system\web\public\models\Lebai_LM3.glb' -Destination 'D:\MyWork\VR4Arm\web\public\models\Lebai_LM3.glb'
```

Create `web/tests/robotModel.test.ts`:

```ts
import * as THREE from 'three'; import {expect,it} from 'vitest';
import {createRobotModelForTest} from '../src/robot/robotModel';
function scene(count=6){const root=new THREE.Group();for(let i=1;i<=count;i++){const n=new THREE.Group();n.name=`Joint${i}`;root.add(n)}return root}
it('requires all six joints',()=>expect(()=>createRobotModelForTest(scene(5))).toThrow('缺少关节节点: Joint6'));
it('requires six joint values',()=>expect(()=>createRobotModelForTest(scene()).setJointAngles([0,1])).toThrow('需要 6 个关节角'));
it('applies finite joint angles',()=>{const model=createRobotModelForTest(scene());model.setJointAngles([0.1,0.2,0.3,0.4,0.5,0.6]);expect(model.group.getObjectByName('Joint1')?.rotation.y).toBeCloseTo(0.1)});
```

Create `web/tests/robotState.test.ts`:

```ts
import {expect,it} from 'vitest'; import {RobotStateBuffer} from '../src/robot/robotState';
const state=(time:number,q:number[],gripper:number)=>({v:1,type:'robot_state',server_mono_ns:time,ack_seq:1,mode:'ACTIVE',robot_state:'MOVING',actual_tcp:{p:[0,0,0],q:[0,0,0,1]},actual_q:q,gripper,sample_age_ms:1,fault:null} as const);
it('interpolates and stops extrapolating after 100ms',()=>{const b=new RobotStateBuffer();b.push(state(0,[0,0,0,0,0,0],0));b.push(state(50_000_000,[1,1,1,1,1,1],1));expect(b.sample(25_000_000).state.actual_q[0]).toBeCloseTo(0.5);const stale=b.sample(151_000_000);expect(stale.stale).toBe(true);expect(stale.state.actual_q[0]).toBe(1)});
```

- [ ] **Step 2: Run and verify missing robot modules**

Run: `cd D:\MyWork\VR4Arm\web; npm.cmd test -- robotModel`
Expected: FAIL on missing module.

- [ ] **Step 3: Implement robotModel by adapting the reviewed reference**

Reuse the reference node mapping exactly: `Joint1:y`, `Joint2:z`, `Joint3:z`, `Joint4:z`, `Joint5:y`, `Joint6:z`; preserve original rotations before applying state. Require all six nodes and throw a Chinese error listing missing names. Reuse the `Take 001` animation and frames 0–20 for normalized gripper. Do not import kinematics into the renderer.

- [ ] **Step 4: Implement state interpolation**

Keep the newest two RobotStateMessage instances. Interpolate with `alpha=clamp((now-a.time)/(b.time-a.time),0,1)`; interpolate joints and gripper linearly; return newest state after its timestamp, but mark the visual stale when newest age exceeds 100 ms and stop changing pose.

- [ ] **Step 5: Run tests/build and commit**

Run: `cd D:\MyWork\VR4Arm\web; npm.cmd test; npm.cmd run build`
Expected: PASS.

```powershell
git add web/public/models web/src/robot web/tests
git commit -m "feat: render LM3 simulator state"
```

---

### Task 11: Build the Desktop Simulation Scene and Chinese Safety HUD

**Files:**
- Create: `web/src/scenes/simulationScene.ts`
- Create: `web/src/ui/hud.ts`
- Create: `web/src/ui/latency.ts`
- Create: `web/src/ui/armPanel.ts`
- Create: `web/src/main.ts`
- Create: `web/src/styles.css`
- Create: `web/tests/armPanel.test.ts`
- Create: `web/tests/latency.test.ts`

**Interfaces:**
- Produces: `SimulationScene.start()`, `applyRobotState()`, `dispose()`
- Produces: explicit UI actions `解锁仿真`, `停止`, `进入 VR`
- Consumes: TeleopSocket and RobotStateBuffer

- [ ] **Step 1: Write failing UI safety tests**

Create `web/tests/armPanel.test.ts`:

```ts
// @vitest-environment jsdom
import {beforeEach,expect,it,vi} from 'vitest'; import {ArmPanel} from '../src/ui/armPanel';
beforeEach(()=>document.body.innerHTML='<div id="panel"></div>');
it('is simulator-only and starts locked',()=>{const send=vi.fn();const panel=new ArmPanel(document.querySelector('#panel')!,send);expect(panel.backendText).toBe('SIMULATOR');expect(document.body.textContent).not.toContain('LEBAI');panel.armButton.click();expect(send).not.toHaveBeenCalled()});
it('arms only after grip release and stop disarms',()=>{const send=vi.fn();const p=new ArmPanel(document.querySelector('#panel')!,send);p.observeGrip(false);p.armButton.click();expect(send.mock.calls[0][0].type).toBe('arm_request');p.stopButton.click();expect(send.mock.calls.at(-1)?.[0].type).toBe('disarm')});
it('fault and reconnect reset lock',()=>{const p=new ArmPanel(document.querySelector('#panel')!,vi.fn());p.observeGrip(false);p.setFault('ik_unreachable');expect(p.armButton.disabled).toBe(true);p.onSocketReconnect();expect(p.isArmed).toBe(false)});
```

Create `web/tests/latency.test.ts`:

```ts
import {expect,it} from 'vitest'; import {LatencyTracker} from '../src/ui/latency';
it('reports p95 over a bounded 512-sample window',()=>{const t=new LatencyTracker(512);for(let i=1;i<=600;i++)t.add(i);expect(t.count).toBe(512);expect(t.p95()).toBe(575)});
```

- [ ] **Step 2: Run and verify missing UI modules**

Run: `cd D:\MyWork\VR4Arm\web; npm.cmd test -- armPanel`
Expected: FAIL.

- [ ] **Step 3: Implement minimal desktop scene**

Create Three.js renderer, perspective camera, lights, grid/table, load LM3 model, apply RobotStateBuffer in animation loop, show target TCP as transparent axes/marker, and expose resize/dispose. Desktop drag controls alter a synthetic controller pose but use the same VRFrame factory as WebXR.

- [ ] **Step 4: Implement HUD and arm panel**

Show backend, TeleopMode, BackendState, sample age, RTT, tracking, Grip, Trigger, and fault. Use text plus shape/color. Wire explicit arm/disarm buttons; every socket close calls `resetToLocked()`. Implement `LatencyTracker` as a ring buffer of at most 512 ack latencies; `p95()` sorts a copy and returns index `ceil(0.95*n)-1`. Display current and p95 latency so the Quest acceptance checklist has direct evidence.

- [ ] **Step 5: Wire main entry, run tests/build, and commit**

Run: `cd D:\MyWork\VR4Arm\web; npm.cmd test; npm.cmd run build`
Expected: PASS.

```powershell
git add web/src web/tests web/index.html
git commit -m "feat: add desktop LM3 simulator interface"
```

---

### Task 12: Add Quest WebXR Controller Input and Pure VR Mode

**Files:**
- Create: `web/src/xr/controllerInput.ts`
- Create: `web/src/xr/session.ts`
- Create: `web/tests/controllerInput.test.ts`
- Modify: `web/src/scenes/simulationScene.ts`
- Modify: `web/src/main.ts`

**Interfaces:**
- Produces: `readRightController(frame, referenceSpace, session) -> ControllerSample | null`
- Produces: `XRSessionController.enterVR()`, `exitVR()`
- Consumes right-hand `gripSpace`, not `targetRaySpace`

- [ ] **Step 1: Write failing controller mapping tests**

Create `web/tests/controllerInput.test.ts`:

```ts
import {expect,it} from 'vitest'; import {readRightController} from '../src/xr/controllerInput';
const source=(handedness:'left'|'right',trigger=.3,grip=.6)=>({handedness,gripSpace:{},gamepad:{buttons:[{value:trigger},{value:grip}]}} as unknown as XRInputSource);
const frame=(hasPose=true)=>({getPose:()=>hasPose?{transform:{position:{x:1,y:2,z:3},orientation:{x:0,y:0,z:0,w:1}}}:null} as unknown as XRFrame);
it('selects right gripSpace and maps buttons',()=>{const sample=readRightController(frame(),{} as XRReferenceSpace,[source('left'),source('right')]);expect(sample?.p).toEqual([1,2,3]);expect(sample?.grip).toBe(true);expect(sample?.trigger).toBeCloseTo(.3);expect(sample?.trackingValid).toBe(true)});
it('reports tracking loss when pose is absent',()=>expect(readRightController(frame(false),{} as XRReferenceSpace,[source('right')])?.trackingValid).toBe(false));
it('does not use left controller',()=>expect(readRightController(frame(),{} as XRReferenceSpace,[source('left')])).toBeNull());
```

- [ ] **Step 2: Run and verify missing XR modules**

Run: `cd D:\MyWork\VR4Arm\web; npm.cmd test -- controllerInput`
Expected: FAIL.

- [ ] **Step 3: Implement controller reader**

Request `local-floor`; per XR frame find `inputSource.handedness==='right'`, query `frame.getPose(source.gripSpace, referenceSpace)`, copy position/quaternion, read standard Gamepad trigger index 0 and squeeze/grip index 1, and construct VRFrame with monotonic sequence/session ID. Cap transport sends to 60 Hz.

- [ ] **Step 4: Implement XR session lifecycle safety**

Call `renderer.xr.enabled=true` and `renderer.setAnimationLoop`. On `visibilitychange` where state is not `visible`, immediately send `disarm` and a final `tracking_valid=false` frame. On session `end`, send disarm, reset locked UI, and return to desktop loop. Enter only `immersive-vr`; do not request passthrough.

- [ ] **Step 5: Run tests/build and commit**

Run: `cd D:\MyWork\VR4Arm\web; npm.cmd test; npm.cmd run build`
Expected: PASS.

```powershell
git add web/src/xr web/src/scenes web/src/main.ts web/tests
git commit -m "feat: add Quest WebXR teleoperation input"
```

---

### Task 13: Add End-to-End Fault Tests and the Deterministic Soak Runner

**Files:**
- Create: `backend/tests/api/test_fault_paths.py`
- Create: `scripts/soak_simulator.py`
- Create: `backend/tests/test_soak.py`

**Interfaces:**
- Produces CLI: `python scripts/soak_simulator.py --minutes 10 --seed 42`
- Produces JSON summary: frames, commands, stops, max_queue_depth, nan_count, final_mode

- [ ] **Step 1: Write failing fault-path tests**

Create `backend/tests/api/test_fault_paths.py`:

```python
import pytest
from pydantic import ValidationError
from app.robots.base import StopReason
from app.schemas.messages import VRFrame
from tests.control.test_robot_control import frame, make_control

@pytest.mark.asyncio
@pytest.mark.parametrize(("tracking","visibility"),[(False,"visible"),(True,"hidden")])
async def test_tracking_or_visibility_fault_stops(tracking,visibility):
    control,latest,backend,clock=make_control();await control.connect();f=frame(1,False).model_copy(update={"tracking_valid":tracking,"visibility":visibility});latest.publish(f,clock.now_ns());await control.tick();assert backend.stops[-1] == StopReason.STALE

@pytest.mark.asyncio
async def test_sequence_rollback_is_not_published():
    control,latest,backend,clock=make_control();latest.publish(frame(2,False),clock.now_ns());latest.publish(frame(1,False),clock.now_ns());assert latest.snapshot().frame.seq == 2

def test_nan_is_rejected_before_control():
    payload=frame(1,False).model_dump();payload["right"]["p"][0]=float("nan")
    with pytest.raises(ValidationError): VRFrame.model_validate(payload)

@pytest.mark.asyncio
async def test_socket_disconnect_stops():
    control,latest,backend,clock=make_control();await control.connect();await control.on_disconnect();assert backend.stops[-1] == StopReason.DISCONNECT
```

Append to `test_robot_control.py`:

```python
@pytest.mark.asyncio
async def test_backend_command_error_faults_and_stops():
    control,latest,backend,clock=make_control();backend.command_tcp=AsyncMock(side_effect=BackendCommandError("ik_unreachable"));await control.connect();latest.publish(frame(1,False),clock.now_ns());await control.tick();await control.arm();latest.publish(frame(2,True),clock.now_ns());await control.tick();latest.publish(frame(3,True,p=(0,1.2,-.31)),clock.now_ns());await control.tick();assert control.mode==TeleopMode.FAULT;assert backend.stops[-1]==StopReason.FAULT

@pytest.mark.asyncio
async def test_two_control_overruns_fault_and_stop():
    control,latest,backend,clock=make_control();await control.note_iteration_duration_ms(41);await control.note_iteration_duration_ms(41);assert control.mode==TeleopMode.FAULT;assert backend.stops[-1]==StopReason.FAULT
```

Import `AsyncMock` from `unittest.mock` and `BackendCommandError` from `app.robots.base` in that test file. `note_iteration_duration_ms()` resets the consecutive counter on any duration `<=40` and faults on the second consecutive value `>40`.

- [ ] **Step 2: Implement deterministic soak runner**

Use FakeClock and explicit `VirtualRobot.step(0.02)` rather than wall time for automated test mode. Generate bounded smooth controller deltas with RNG seed, toggle Grip every 2鈥? seconds, randomly insert tracking loss and reconnect events, and assert only one latest frame is stored. Wall-clock CLI mode runs the same scenario at 50 Hz.

- [ ] **Step 3: Add soak assertions**

For a 10-minute simulated duration assert `nan_count == 0`, `max_queue_depth == 1`, no unhandled exception, all injected faults produced a stop, and final mode is DISARMED. Keep the automated test under 15 seconds by using explicit steps.

- [ ] **Step 4: Run complete backend suite and soak smoke**

Run:

```powershell
cd D:\MyWork\VR4Arm\backend
python -m pytest -q
cd ..
python scripts\soak_simulator.py --minutes 0.2 --seed 42
```

Expected: tests PASS and JSON reports `nan_count: 0`, `max_queue_depth: 1`.

- [ ] **Step 5: Commit**

```powershell
git add backend/tests scripts/soak_simulator.py
git commit -m "test: add simulator fault and soak coverage"
```

---

### Task 14: Document Startup, Quest HTTPS, Operation, and Milestone Acceptance

**Files:**
- Create: `README.md`
- Create: `docs/quest-development.md`
- Create: `docs/milestone-1-acceptance.md`
- Modify: `.gitignore`

**Interfaces:**
- Documents exact developer and operator workflows.
- Does not claim Cannon-es, MR, camera recording, or real SDK support.

- [ ] **Step 1: Write README with exact startup commands**

Include Python venv/install/test/uvicorn commands; npm install/test/build/dev commands; PC IP and firewall note; simulator-only warning in the first screen; and expected URLs `/health`, `/ws/v1/teleop`, and Vite HTTPS page.

- [ ] **Step 2: Write Quest development guide**

Document developer mode, same-LAN access, Vite basic SSL warning flow, optional ADB port forwarding, Quest Browser WebXR entry, right-controller controls, arm/disarm sequence, and how to recover from hidden/refresh/disconnect. State that the browser page never connects to LM3 in milestone 1.

- [ ] **Step 3: Write executable manual acceptance checklist**

Include checkboxes for six axis directions, Grip no-jump, 20-minute session, p95 ack latency under 100 ms, release/tracking/hidden/socket stop paths, gripper analog animation, and confirmation that any backend other than SIMULATOR is rejected. Record date, Quest OS/browser versions, PC network, commit hash, and observed latency.

- [ ] **Step 4: Run final verification**

Run:

```powershell
cd D:\MyWork\VR4Arm\backend
python -m pytest -q
cd ..\web
npm.cmd test
npm.cmd run build
cd ..
git status --short
```

Expected: backend and frontend tests PASS; build succeeds; only intended documentation changes are uncommitted before the final commit.

- [ ] **Step 5: Commit documentation**

```powershell
git add README.md docs .gitignore
git commit -m "docs: add Quest simulator operation and acceptance guide"
```

---

## Final Verification Gate

Before declaring Milestone 1 implemented, run:

```powershell
cd D:\MyWork\VR4Arm\backend
python -m pytest -q
cd ..\web
npm.cmd test
npm.cmd run build
cd ..
python scripts\soak_simulator.py --minutes 10 --seed 42
git status --short
```

Required evidence:

- All Python tests pass.
- All Vitest tests pass.
- TypeScript and Vite build pass.
- Soak JSON reports zero NaN, queue depth one, and final DISARMED state.
- Working tree is clean.
- Manual Quest checklist is completed on the actual Quest 3 before claiming Quest acceptance.
- No real Lebai SDK package is installed or imported and no robot IP is contacted.
