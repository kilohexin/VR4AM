# Real-Robot IK Continuity and Backpressure Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Separate adjacent IK-solution continuity from robot tracking lag so the LM3 can complete a guarded `+roll 1°` motion without weakening PVAT speed, acceleration, soft-limit, stop, or branch-jump protection.

**Architecture:** Add a pure IK-policy module that measures solution continuity and tracking error, retain `max_joint_step_rad=0.05` for adjacent full IK solutions, and add an explicit `max_joint_tracking_error_rad=0.25` envelope. The real adapter stores the last accepted full solution/TCP, uses that solution as the next SDK IK seed, performs at most two proportional Cartesian backoff solves for discontinuous targets, and enters a non-faulting catch-up mode when the physical robot lags. Actual PVAT points remain bounded from the latest measured joints and velocity; all history is generation-scoped and cleared by stop/home/disconnect/fault.

**Tech Stack:** Python 3.11, asyncio, FastAPI backend, Pydantic message models, NumPy, SciPy `Rotation`/`Slerp`, `pytest`/`pytest-asyncio`, Lebai `lebai-sdk-asyncio` through the existing client protocol.

**Spec:** `docs/superpowers/specs/2026-09-02-real-robot-ik-continuity-and-backpressure-design.md`

## Global Constraints

- Do not connect to or command real hardware during implementation or automated verification.
- Keep the real-robot values `max_joint_step_rad=0.05`, `max_joint_speed_radps=0.15`, `max_joint_acceleration_radps2=0.5`, and `pvat_horizon_s=0.08` unchanged; do not overwrite the separate tracked `LEBAI_FAKE` profile values.
- Add `max_joint_tracking_error_rad=0.25` as an explicit required real-robot configuration value; missing or invalid values fail closed.
- `max_joint_tracking_error_rad` must be at least `max_joint_step_rad` and at most `0.50 rad`.
- Do not change the `SIMULATOR` numerical IK, WebXR mapping, gripper behavior, `+z` policy, TCP speed, or configured PVAT frequency.
- Reuse the existing public `motion_continuity_boundary` state; do not expand the WebSocket schema.
- `LEBAI_FAKE` exercises the production policy for offline coverage but never counts as hardware verification.
- Diagnostics never hold `_sdk_lock`, never authorize motion, and never overwrite the motion result if recording fails.
- Each production change begins with a failing test and ends with focused verification plus a local commit.

---

### Task 1: Add the Explicit Tracking-Error Configuration

**Files:**
- Modify: `backend/app/config.py`
- Modify: `backend/tests/config/test_real_robot_config.py`
- Modify: `backend/tests/robots/real_settings.py`
- Modify: `config/real-robot.example.yaml`
- Modify: `config/fake-lebai.yaml`

**Interfaces:**
- Consumes: existing `LebaiControlSettings.max_joint_step_rad: float` and `_positive_float()` parsing.
- Produces: `LebaiControlSettings.max_joint_tracking_error_rad: float`, required YAML key `real_robot.control.max_joint_tracking_error_rad`, and validation against `max_joint_step_rad` and `0.50`.

- [ ] **Step 1: Add failing configuration tests**

Add `max_joint_tracking_error_rad: 0.25` to `REAL_CONFIG_TEMPLATE`, assert it loads, assert the tracked example contains the exact value, and add explicit failures:

```python
def test_tracking_error_limit_must_cover_solution_step(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    text = REAL_CONFIG_TEMPLATE.format(mode="readonly").replace(
        "max_joint_tracking_error_rad: 0.25",
        "max_joint_tracking_error_rad: 0.04",
    )
    with pytest.raises(
        RuntimeError,
        match="^invalid_config:max_joint_tracking_error_rad$",
    ):
        _load(tmp_path, monkeypatch, text)


def test_tracking_error_limit_rejects_values_above_half_radian(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    text = REAL_CONFIG_TEMPLATE.format(mode="readonly").replace(
        "max_joint_tracking_error_rad: 0.25",
        "max_joint_tracking_error_rad: 0.51",
    )
    with pytest.raises(
        RuntimeError,
        match="^invalid_config:max_joint_tracking_error_rad$",
    ):
        _load(tmp_path, monkeypatch, text)
```

Also remove the field from one template instance and assert missing input fails with `invalid_config:max_joint_tracking_error_rad` rather than receiving a default.

- [ ] **Step 2: Run the configuration tests and verify RED**

Run:

```powershell
Set-Location backend
.\.venv\Scripts\python.exe -m pytest tests/config/test_real_robot_config.py -q
```

Expected: failures because `LebaiControlSettings` and `_parse_lebai()` do not yet expose or validate the new field.

- [ ] **Step 3: Implement parsing and tracked-config migration**

Add the dataclass field immediately after `max_joint_step_rad`, parse it as required, and validate it after `LebaiControlSettings` construction:

```python
max_joint_tracking_error_rad=_positive_float(
    control_payload,
    "max_joint_tracking_error_rad",
),
```

```python
if not (
    control.max_joint_step_rad
    <= control.max_joint_tracking_error_rad
    <= 0.50
):
    raise RuntimeError("invalid_config:max_joint_tracking_error_rad")
```

Add `max_joint_tracking_error_rad: 0.25` directly after `max_joint_step_rad` in both tracked Lebai YAML files. Add `max_joint_tracking_error_rad=0.25` to `readonly_settings()` so every adapter test uses an explicit safety value.

- [ ] **Step 4: Run focused config tests and verify GREEN**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/config/test_real_robot_config.py tests/digital_twin/test_runtime.py -q
```

Expected: all selected tests pass and no fixture relies on an implicit tracking limit.

- [ ] **Step 5: Commit Task 1**

```powershell
git add backend/app/config.py backend/tests/config/test_real_robot_config.py backend/tests/robots/real_settings.py config/real-robot.example.yaml config/fake-lebai.yaml
git commit -m "config: separate IK tracking error limit"
```

---

### Task 2: Create a Pure IK Continuity Policy

**Files:**
- Create: `backend/app/robots/lebai_ik_policy.py`
- Create: `backend/tests/robots/test_lebai_ik_policy.py`

**Interfaces:**
- Consumes: `JointVector` from `app.schemas.messages` and `BackendCommandError` from `app.robots.base`.
- Produces:
  - `IkCandidateMetrics(solution_q, solution_step_rad, tracking_error_rad)`;
  - `evaluate_ik_candidate(solution_q, actual_q, previous_solution_q) -> IkCandidateMetrics`;
  - `proportional_recovery_fraction(solution_step_rad, max_solution_step_rad, previous_fraction: float | None = None) -> float`.

- [ ] **Step 1: Write failing pure-policy tests**

Create tests for first-frame reference, previous-solution reference, field-log replay, invalid vectors, and proportional backoff:

```python
def test_tracking_lag_is_not_adjacent_solution_step() -> None:
    metrics = evaluate_ik_candidate(
        solution_q=(0.0, -1.52, 0.1622, -1.5568, 0.4041, 0.0294),
        actual_q=(0.0, -1.567, 0.2495, -1.5721, 0.4016, -0.0011),
        previous_solution_q=(0.0, -1.54, 0.1983, -1.5648, 0.4032, 0.0189),
    )

    assert metrics.solution_step_rad == pytest.approx(0.0361, abs=1e-4)
    assert metrics.tracking_error_rad == pytest.approx(0.0873, abs=1e-4)


def test_initial_solution_uses_actual_joint_reference() -> None:
    metrics = evaluate_ik_candidate(
        solution_q=(0.04, 0, 0, 0, 0, 0),
        actual_q=(0, 0, 0, 0, 0, 0),
        previous_solution_q=None,
    )
    assert metrics.solution_step_rad == pytest.approx(0.04)


def test_recovery_fraction_has_inward_margin() -> None:
    assert proportional_recovery_fraction(0.10, 0.05) == pytest.approx(0.4)
    assert proportional_recovery_fraction(0.08, 0.05, 0.4) == pytest.approx(0.2)
```

Invalid joint vectors must raise the existing deterministic `BackendCommandError` codes (`ik_invalid`, `invalid_actual_joint_state`, or `invalid_previous_ik_solution`). Non-positive limits and invalid fractions raise deterministic `ValueError` messages; this module does not issue backend commands.

- [ ] **Step 2: Run the policy tests and verify RED**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/robots/test_lebai_ik_policy.py -q
```

Expected: collection fails because `app.robots.lebai_ik_policy` does not exist.

- [ ] **Step 3: Implement the pure policy**

Use immutable data and six-element finite NumPy vectors:

```python
@dataclass(frozen=True)
class IkCandidateMetrics:
    solution_q: JointVector
    solution_step_rad: float
    tracking_error_rad: float


def evaluate_ik_candidate(
    solution_q: object,
    actual_q: JointVector,
    previous_solution_q: JointVector | None,
) -> IkCandidateMetrics:
    solution = _vector(solution_q, "ik_invalid")
    actual = _vector(actual_q, "invalid_actual_joint_state")
    reference = actual if previous_solution_q is None else _vector(
        previous_solution_q,
        "invalid_previous_ik_solution",
    )
    return IkCandidateMetrics(
        solution_q=_joint_tuple(solution),
        solution_step_rad=float(np.max(np.abs(solution - reference))),
        tracking_error_rad=float(np.max(np.abs(solution - actual))),
    )
```

When `previous_fraction is None`, implement the first retry as
`min(1.0, 0.8 * limit / measured_step)`. When a previous fraction is supplied,
return `previous_fraction * 0.5` for the second retry. Reject non-finite,
non-positive, or out-of-range inputs rather than silently repairing them.

- [ ] **Step 4: Run focused policy tests and verify GREEN**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/robots/test_lebai_ik_policy.py -q
```

Expected: all policy tests pass, including the `0.0361` versus `0.0873` field replay.

- [ ] **Step 5: Commit Task 2**

```powershell
git add backend/app/robots/lebai_ik_policy.py backend/tests/robots/test_lebai_ik_policy.py
git commit -m "feat: model IK continuity separately from tracking lag"
```

---

### Task 3: Make PVAT Enforce the Tracking Envelope, Not IK Branch Continuity

**Files:**
- Modify: `backend/app/robots/lebai_pvat.py`
- Modify: `backend/tests/robots/test_lebai_pvat.py`
- Modify: `backend/app/robots/lebai_adapter.py` only to update `PvatLimits` construction

**Interfaces:**
- Consumes: `LebaiControlSettings.max_joint_tracking_error_rad` from Task 1 and `evaluate_ik_candidate()` from Task 2.
- Produces: `PvatLimits.max_joint_tracking_error_rad: float`; `build_pvat_point()` raises `BackendCommandError("ik_tracking_diverged")` only when the full solution is farther than the tracking envelope from actual joints.

- [ ] **Step 1: Rewrite the PVAT tests to express the new boundary**

Change the shared test limit:

```python
LIMITS = PvatLimits(
    horizon_s=0.08,
    max_joint_speed_radps=0.15,
    max_joint_acceleration_radps2=0.5,
    max_joint_tracking_error_rad=0.25,
    soft_joint_min_rad=(-3.0, -2.5, -2.5, -3.0, -2.5, -6.0),
    soft_joint_max_rad=(3.0, 2.5, 2.5, 3.0, 2.5, 6.0),
)
```

Replace the old `0.051 -> ik_joint_jump` assertion with:

```python
def test_large_safe_tracking_target_is_physically_bounded() -> None:
    point = build_pvat_point(
        solution_q=[0.196, 0, 0, 0, 0, 0],
        actual_q=ZERO,
        actual_qd=ZERO,
        previous_qd=None,
        limits=LIMITS,
    )
    assert point.q[0] == pytest.approx(0.0032)
    assert point.qd[0] == pytest.approx(0.04)
    assert point.qdd[0] == pytest.approx(0.5)


def test_solution_beyond_tracking_envelope_is_rejected() -> None:
    with pytest.raises(BackendCommandError, match="^ik_tracking_diverged$"):
        build_pvat_point(
            solution_q=[0.251, 0, 0, 0, 0, 0],
            actual_q=ZERO,
            actual_qd=ZERO,
            previous_qd=None,
            limits=LIMITS,
        )
```

Keep all existing vector-ratio, floating-boundary, speed, acceleration, and
soft-joint tests, updating every local `PvatLimits(...)` constructor to the new
keyword. Retain an adapter-level test proving a `0.051 rad` first-frame
solution is still rejected as `ik_joint_jump`; moving the physical guard must
not create an intermediate commit without continuity protection.

- [ ] **Step 2: Run PVAT tests and verify RED**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/robots/test_lebai_pvat.py -q
```

Expected: constructor failures and the `0.196` case still raises the old `ik_joint_jump`.

- [ ] **Step 3: Implement the tracking-envelope guard**

Rename the dataclass field and replace only the positional guard:

```python
tracking_error = solution - actual
if np.max(np.abs(tracking_error)) > limits.max_joint_tracking_error_rad:
    raise BackendCommandError("ik_tracking_diverged")
```

Do not change `_scale_to_max_abs()`, velocity-vector proportional scaling,
acceleration-vector proportional scaling, soft joint bounds, or
`bounded_q = actual + qd * horizon_s`. Construct adapter limits from
`settings.control.max_joint_tracking_error_rad`.

In the adapter's existing `_solve_candidate()` path, evaluate the solution with
`previous_solution_q=None`. Preserve the established error precedence by first
calling the pure `build_pvat_point()` calculation so soft joint limits and
measured-speed violations remain authoritative. If that calculation reports
`ik_tracking_diverged` while the first-frame solution step exceeds
`settings.control.max_joint_step_rad`, translate it to `ik_joint_jump`; otherwise
reject a returned point when the same continuity threshold is exceeded. Task 4
will replace the temporary actual-joint reference with committed full-solution
history. This preserves the exact pre-Task-3 first-frame safety behavior at
every commit. Update the existing
`ik_candidate_rejected` payload to read `max_joint_step_rad` from
`settings.control`, because that field no longer belongs to `PvatLimits`.

- [ ] **Step 4: Run focused math tests and verify GREEN**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/robots/test_lebai_pvat.py tests/robots/test_lebai_ik_policy.py tests/robots/test_lebai_adapter_control.py -q
```

Expected: the `0.196` solution yields a `0.0032 rad` first point, `0.251` fails, and all vector-direction tests remain green.

- [ ] **Step 5: Commit Task 3**

```powershell
git add backend/app/robots/lebai_pvat.py backend/tests/robots/test_lebai_pvat.py backend/app/robots/lebai_adapter.py
git commit -m "refactor: bound PVAT tracking lag independently"
```

---

### Task 4: Track and Log the Last Accepted Full IK Solution

**Files:**
- Modify: `backend/app/robots/lebai_adapter.py`
- Modify: `backend/tests/robots/test_lebai_adapter_control.py`

**Interfaces:**
- Consumes: `evaluate_ik_candidate()` and `IkCandidateMetrics` from Task 2; tracking-aware `build_pvat_point()` from Task 3.
- Produces:
  - adapter field `_last_accepted_solution_q: JointVector | None`;
  - internal immutable `_SolvedCandidate(target, metrics, point, recovery_fraction, pvat_mode, advances_target)`;
  - adapter methods `_inverse_kinematics(client, snapshot, target) -> JointVector` and `_build_advancing_candidate(snapshot, target, metrics, fraction) -> _SolvedCandidate`; `_inverse_kinematics()` selects the previous accepted solution as seed, or the current actual joints when history is empty;
  - enriched `pvat_sent` fields `ik_solution_q`, `solution_step_rad`, `tracking_error_rad`, `pvat_mode`, and `accepted_target_command_id`.

- [ ] **Step 1: Add failing adapter tests for accepted-solution history**

Add shared test helpers before the new cases:

```python
async def _wait_for_pvat_count(
    client: FakeLebaiClient,
    expected: int,
) -> None:
    await _wait_until(
        lambda: [call[0] for call in client.write_calls].count("move_pvat")
        == expected
    )


def _last_pvat_event(events: list[dict[str, object]]) -> dict[str, object]:
    return [event for event in events if event.get("kind") == "pvat_sent"][-1]


async def _adapter_with_accepted_history(
    *,
    actual_q: JointVector = tuple(IDLE_Q),
    solution: JointVector,
    target: Pose,
    backend_label: Literal["LEBAI", "LEBAI_FAKE"] = "LEBAI",
) -> tuple[RealLebaiAdapter, FakeLebaiClient, list[dict[str, object]]]:
    events: list[dict[str, object]] = []

    async def recorder(event: dict[str, object], _timestamp: int) -> None:
        events.append(event)

    client = FakeLebaiClient.idle(q=list(actual_q))
    clock = FakeClock()
    adapter = RealLebaiAdapter(
        control_settings(),
        client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns,
        sleep=clock.sleep,
        event_callback=recorder,
        backend_label=backend_label,
    )
    await adapter.connect()
    adapter._last_accepted_solution_q = solution
    adapter._last_sent_tcp = target.model_copy(deep=True)
    adapter._accepted_target_command_id = 1
    return adapter, client, events
```

The direct private-state arrangement is limited to tests that need a specific
tracking backlog; separate Task 4 tests prove production history is committed
only after a successful PVAT write. Add tests that assert the SDK seed and
distinguish full solution from physical point:

```python
@pytest.mark.asyncio
async def test_second_ik_uses_previous_accepted_solution_as_seed() -> None:
    adapter, client, _ = await _connected_control_adapter()
    first_solution = [0.04, -1.0, 1.0, 0.0, 1.57, 0.0]
    second_solution = [0.08, -1.0, 1.0, 0.0, 1.57, 0.0]
    client.ik_results = deque([first_solution, second_solution])

    await adapter.command_tcp(_target(0.301), command_id=1)
    await _wait_for_pvat_count(client, 1)
    await adapter.command_tcp(_target(0.302), command_id=2)
    await _wait_for_pvat_count(client, 2)

    assert client.ik_calls[1][1] == first_solution
    assert adapter._last_accepted_solution_q == tuple(second_solution)
```

Add a recorder assertion proving `ik_solution_q` is the full solution while event field `p` remains the physically shortened PVAT point. Add stop, home, disconnect, and runtime-fault assertions that `_last_accepted_solution_q` is cleared together with `_last_sent_tcp` and `_previous_sent_qd`.

- [ ] **Step 2: Run focused adapter tests and verify RED**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/robots/test_lebai_adapter_control.py -k "accepted_solution or previous_accepted_solution" -q
```

Expected: failures because the adapter always seeds IK with `actual_q` and discards the full solution after building a PVAT point.

- [ ] **Step 3: Implement solution history and enriched success records**

Add the internal result type near `LebaiSnapshot`:

```python
PvatMode = Literal["advance", "interpolated_advance", "catch_up"]


@dataclass(frozen=True)
class _SolvedCandidate:
    target: Pose
    metrics: IkCandidateMetrics
    point: PvatPoint
    recovery_fraction: float
    pvat_mode: PvatMode
    advances_target: bool
```

Split the current `_solve_candidate()` so SDK IK returns a validated full solution, policy metrics are computed before PVAT construction, and the previous full solution is used as seed when present. After `move_pvat()` succeeds:

```python
self._previous_sent_qd = selected.point.qd
if selected.advances_target:
    self._last_sent_tcp = selected.target.model_copy(deep=True)
    self._last_accepted_solution_q = selected.metrics.solution_q
    self._accepted_target_command_id = request.command_id
self._command_id = request.command_id
self._motion_accepted = True
```

Extend `_reset_pvat_history()` to clear solution and accepted-command history. Emit enriched success fields from the committed `_SolvedCandidate`; do not log a full solution as accepted before SDK `move_pvat()` returns.

After `move_pvat()` returns, re-check `request.generation` while still inside
the SDK-serialized section and before assigning `accepted`. If Stop/Home/
disconnect invalidated the request while the SDK call was in flight, return
without restoring history, `_motion_accepted`, acknowledgement, constraint, or
success logs. The explicit stop path is then solely responsible for settling
the robot.

- [ ] **Step 4: Run adapter history tests and verify GREEN**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/robots/test_lebai_adapter_control.py -k "accepted_solution or previous_accepted_solution or clears" -q
```

Expected: the second IK seed equals the first full solution, PVAT point remains shortened, and every stop/reset path clears the history.

- [ ] **Step 5: Commit Task 4**

```powershell
git add backend/app/robots/lebai_adapter.py backend/tests/robots/test_lebai_adapter_control.py
git commit -m "feat: preserve accepted IK solution history"
```

---

### Task 5: Add Bounded Cartesian Recovery for Discontinuous Requested Targets

**Files:**
- Modify: `backend/app/robots/lebai_adapter.py`
- Modify: `backend/app/robots/lebai_recovery.py`
- Modify: `backend/tests/robots/test_lebai_adapter_control.py`
- Modify: `backend/tests/robots/test_lebai_recovery.py`

**Interfaces:**
- Consumes: `interpolate_pose()` and `proportional_recovery_fraction()`.
- Produces: adapter method `_select_advancing_candidate(client, snapshot, request: PvatRequest) -> _SolvedCandidate | None`; `None` means the request generation was superseded; at most three SDK IK calls total per current request: original plus two recovery solves.

- [ ] **Step 1: Add failing recovery tests**

Cover one-retry recovery, second-retry recovery, and hard discontinuity:

```python
@pytest.mark.asyncio
async def test_real_target_uses_proportional_cartesian_recovery() -> None:
    adapter, client, events = await _adapter_with_accepted_history(
        solution=(0.04, -1.0, 1.0, 0.0, 1.57, 0.0),
        target=_target(0.301),
    )
    client.ik_results = deque(
        [
            [0.14, -1.0, 1.0, 0.0, 1.57, 0.0],
            [0.08, -1.0, 1.0, 0.0, 1.57, 0.0],
        ]
    )

    await adapter.command_tcp(_target(0.311), command_id=2)
    await _wait_for_pvat_count(client, 1)

    event = _last_pvat_event(events)
    assert event["pvat_mode"] == "interpolated_advance"
    assert event["recovery_fraction"] == pytest.approx(0.4)
    assert event["solution_step_rad"] == pytest.approx(0.04)
    assert len(client.ik_calls) == 2  # original request + one recovery solve
```

For a full step of `0.10 rad` with a `0.05 rad` limit, assert the first retry fraction is `0.4`. If that retry still returns a `0.06` step, assert the second retry uses `0.2`. If all three candidate solutions remain discontinuous, assert no PVAT is sent and existing `ik_joint_jump` persistence remains active.

- [ ] **Step 2: Run recovery tests and verify RED**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/robots/test_lebai_recovery.py tests/robots/test_lebai_adapter_control.py -k "proportional or discontinuous_requested" -q
```

Expected: failures because real mode currently attempts only the original target and has no solution-aware fraction.

- [ ] **Step 3: Implement bounded recovery without joint interpolation**

Keep `interpolate_pose()` as the only interpolation primitive. `_select_advancing_candidate()` must:

```python
requested = request.target
fraction = 1.0
for attempt in range(3):
    if not self._pump.is_current(request.generation):
        return None
    target = (
        requested
        if fraction == 1.0
        else interpolate_pose(self._last_sent_tcp, requested, fraction)
    )
    solution = await self._inverse_kinematics(client, snapshot, target)
    metrics = evaluate_ik_candidate(
        solution,
        snapshot.actual_q,
        self._last_accepted_solution_q,
    )
    if metrics.solution_step_rad <= self.settings.control.max_joint_step_rad:
        return self._build_advancing_candidate(
            snapshot,
            target,
            metrics,
            fraction,
        )
    if attempt == 2 or self._last_sent_tcp is None:
        raise BackendCommandError("ik_joint_jump")
    fraction = proportional_recovery_fraction(
        metrics.solution_step_rad,
        self.settings.control.max_joint_step_rad,
        previous_fraction=None if attempt == 0 else fraction,
    )
```

Use the exact re-solved metrics to accept or reject each retry. Never linearly interpolate `solution_q`, never exceed three IK calls, and check pump generation before every additional IK, immediately after every awaited IK on both success and exception paths, before PVAT write, and after PVAT returns. `_send_target()` treats `None` as silent cancellation and performs no logging or state mutation for that superseded request. Any SDK/IK exception that arrives after invalidation is discarded as a stale result rather than becoming a pump fault; the concurrent stop path remains authoritative.

- [ ] **Step 4: Run recovery and adapter tests and verify GREEN**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/robots/test_lebai_recovery.py tests/robots/test_lebai_adapter_control.py -q
```

Expected: proportional real recovery passes, hard discontinuity remains fail-closed, existing fake fallback behavior remains covered, and no test performs more than three IK calls per request.

- [ ] **Step 5: Commit Task 5**

```powershell
git add backend/app/robots/lebai_adapter.py backend/app/robots/lebai_recovery.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_lebai_recovery.py
git commit -m "feat: recover continuous real IK targets with bounded backoff"
```

---

### Task 6: Add Non-Faulting Tracking Backpressure and Catch-Up PVAT

**Files:**
- Modify: `backend/app/robots/lebai_adapter.py`
- Modify: `backend/app/control/robot_control.py`
- Modify: `backend/tests/robots/test_lebai_adapter_control.py`
- Modify: `backend/tests/control/test_robot_control.py`

**Interfaces:**
- Consumes: `_SolvedCandidate`, accepted solution/TCP history, and tracking-aware `build_pvat_point()`.
- Produces:
  - internal soft reason `ik_tracking_lag` mapped to public `motion_continuity_boundary`;
  - hard error `ik_tracking_diverged` when the already accepted solution itself leaves the envelope;
  - `_build_catch_up_candidate(snapshot, request: PvatRequest) -> _SolvedCandidate` with `advances_target=False` and `pvat_mode="catch_up"`;
  - best-effort `ik_tracking_backpressure` diagnostics.

- [ ] **Step 1: Add failing backpressure tests**

Cover the main behavior with real adapter mode:

```python
@pytest.mark.asyncio
async def test_tracking_lag_reuses_last_solution_without_advancing_history() -> None:
    adapter, client, events = await _adapter_with_accepted_history(
        solution=(0.22, -1.0, 1.0, 0.0, 1.57, 0.0),
        target=_target(0.301),
    )
    client.ik_results = deque(
        [[0.26, -1.0, 1.0, 0.0, 1.57, 0.0]]
    )

    await adapter.command_tcp(_target(0.302), command_id=2)
    await _wait_for_pvat_count(client, 1)

    assert adapter._last_accepted_solution_q[0] == pytest.approx(0.22)
    assert adapter._last_sent_tcp == _target(0.301)
    assert adapter.constraint == "motion_continuity_boundary"
    assert adapter.pump_fault is None
    assert _last_pvat_event(events)["pvat_mode"] == "catch_up"
```

The arranged actual J1 is `0.0`, so the stored `0.22` solution remains inside the
tracking envelope while the continuous `0.26` candidate is outside it. Repeat
more than five catch-up cycles and prove `ik_failure_persistent` is never latched.
Then update the fake SDK's actual J1 to `0.04`, return the same continuous
candidate inside the envelope, and prove normal advance clears the constraint.

Add a divergence case where the stored accepted solution is itself more than `0.25` from actual; assert `ik_tracking_diverged`, no PVAT, and the existing fault/stop path. Add RobotControl mapping:

```python
@pytest.mark.asyncio
async def test_tracking_lag_is_public_motion_continuity_boundary() -> None:
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()
    safe_target = control.last_target

    backend.command_tcp = AsyncMock(
        side_effect=BackendCommandError("ik_tracking_lag")
    )
    latest.publish(
        frame(3, True, p=(0.0, 1.2, -0.31)),
        clock.now_ns(),
    )
    await control.tick()

    state = await control.state_message()
    assert state.constraint == "motion_continuity_boundary"
    assert control.mode is TeleopMode.ACTIVE
    assert control._fault is None
    assert control.last_target == safe_target
```

- [ ] **Step 2: Run backpressure tests and verify RED**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/robots/test_lebai_adapter_control.py tests/control/test_robot_control.py -k "tracking_lag or catch_up or tracking_diverged" -q
```

Expected: failures because all tracking excess currently becomes `ik_joint_jump`/persistent failure and RobotControl does not recognize `ik_tracking_lag`.

- [ ] **Step 3: Implement catch-up state and soft mapping**

Inside candidate selection, after an exact IK result passes adjacent-solution
continuity but before `_build_advancing_candidate()` calls
`build_pvat_point()`, compare `metrics.tracking_error_rad` with the configured
envelope. If it exceeds the envelope and history exists, preserve those blocked
metrics for diagnostics, validate the stored solution against current actual
joints, then construct a PVAT point toward the stored solution:

```python
catch_up_metrics = evaluate_ik_candidate(
    self._last_accepted_solution_q,
    snapshot.actual_q,
    self._last_accepted_solution_q,
)
if (
    catch_up_metrics.tracking_error_rad
    > self.settings.control.max_joint_tracking_error_rad
):
    raise BackendCommandError("ik_tracking_diverged")
point = build_pvat_point(
    solution_q=self._last_accepted_solution_q,
    actual_q=snapshot.actual_q,
    actual_qd=snapshot.actual_qd,
    previous_qd=self._previous_sent_qd,
    limits=self._pvat_limits,
)
```

Set `_constraint_error="ik_tracking_lag"` and public adapter constraint `motion_continuity_boundary` without incrementing `_consecutive_ik_failures`. Catch-up success updates only `_previous_sent_qd`, `_command_id`, and `_motion_accepted`; it never changes accepted TCP/solution/command history.

Teach RobotControl to map `ik_tracking_lag` to `motion_continuity_boundary`. The following tick may submit the held target but must not advance `RobotControl.last_target` while the soft reason is visible. A successful advancing PVAT clears the reason and failure counter.

- [ ] **Step 4: Add generation and diagnostic race tests**

Add deterministic tests using blocking IK, blocking diagnostic callbacks, and `FakeClock`:

```python
@pytest.mark.asyncio
async def test_stop_during_catch_up_cannot_restore_history_or_write_late_pvat() -> None:
    adapter, client, _ = await _adapter_with_accepted_history(
        solution=(0.22, -1.0, 1.0, 0.0, 1.57, 0.0),
        target=_target(0.301),
    )
    client.block_ik = True
    client.ik_results = deque(
        [[0.26, -1.0, 1.0, 0.0, 1.57, 0.0]]
    )
    await adapter.command_tcp(_target(0.302), command_id=2)
    await client.ik_started.wait()

    stop_task = asyncio.create_task(adapter.stop(StopReason.STALE))
    await asyncio.sleep(0)
    client.release_ik.set()
    await stop_task

    assert "move_pvat" not in [call[0] for call in client.write_calls]
    assert adapter._last_accepted_solution_q is None
    assert adapter._last_sent_tcp is None
    assert adapter._motion_accepted is False
    assert adapter.pump_fault is None
```

Add a recorder that raises on `ik_tracking_backpressure` and prove it does not change catch-up success. Add a recorder that blocks and prove `stop_move` acquires `_sdk_lock` and completes before releasing the diagnostic.

Add separate blocked-`move_pvat` coverage: invalidate the generation while the
SDK write is in flight, release the write, and prove the stale handler neither
commits full-solution/TCP history nor latches a late timeout/fault. It is
acceptable that the already-started vendor call occurred; the required safety
outcome is that Stop owns all subsequent state and sends `stop_move`.

- [ ] **Step 5: Run backpressure, control, and race tests and verify GREEN**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/robots/test_lebai_adapter_control.py tests/control/test_robot_control.py -q
```

Expected: catch-up does not count toward persistent IK failures, genuine divergence fails closed, RobotControl remains ACTIVE with a public soft boundary, and every stop race ends with empty history and no late PVAT/fault.

- [ ] **Step 6: Commit Task 6**

```powershell
git add backend/app/robots/lebai_adapter.py backend/app/control/robot_control.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/control/test_robot_control.py
git commit -m "feat: backpressure lagging real robot targets"
```

---

### Task 7: Replay the Field Roll Sequence and Complete Offline Verification

**Files:**
- Modify: `backend/tests/robots/test_lebai_adapter_control.py`
- Modify: `docs/real-robot-deployment.md`
- Modify: `docs/real-robot-commissioning-report.md`

**Interfaces:**
- Consumes: all Tasks 1–6 and field evidence from session `20260902T103828Z-27b34b9a`.
- Produces: deterministic roll-regression coverage, local-config migration instructions, and a one-action onsite handoff.

- [ ] **Step 1: Add the field-sequence regression test**

Use the recorded solution sequence and controlled actual snapshots. The regression must assert that adjacent solutions are measured independently from tracking lag:

```python
FIELD_SOLUTIONS = [
    (-0.0030881, -1.5400230, 0.1983114, -1.5647952, 0.4032416, 0.0189356),
    (-0.0048767, -1.5215283, 0.1622087, -1.5568198, 0.4041157, 0.0293936),
    (-0.0049698, -1.5204396, 0.1600690, -1.5562742, 0.4041618, 0.0299422),
]

assert evaluate_ik_candidate(
    FIELD_SOLUTIONS[1],
    actual_q=(-0.0000, -1.5669614, 0.2494636, -1.5721386, 0.4016153, -0.0010546),
    previous_solution_q=FIELD_SOLUTIONS[0],
).solution_step_rad == pytest.approx(0.0361, abs=1e-4)
```

Drive the adapter through one accepted seed plus recorded candidates and assert no candidate is rejected merely because `solution_q - actual_q > 0.05`; every emitted physical point must remain within `0.15 rad/s` and `0.5 rad/s²`.

- [ ] **Step 2: Run the regression and verify it protects the root cause**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/robots/test_lebai_adapter_control.py -k "field_roll_sequence" -q
```

Expected: PASS only with separated continuity/tracking semantics; changing the policy back to `solution_q - actual_q <= 0.05` must make the test fail.

- [ ] **Step 3: Update deployment and commissioning documentation**

Document all of the following explicitly:

- `max_joint_step_rad=0.05` is adjacent full-solution continuity, not physical PVAT step;
- `max_joint_tracking_error_rad=0.25` is a bounded desired-versus-actual envelope, not permission to jump `0.25 rad`;
- `advance`, `interpolated_advance`, and `catch_up` event meanings;
- manual migration of untracked `config/real-robot.local.yaml`;
- rollback to `5e49d3a` and readonly preflight after rollback;
- the next onsite run is `prepare` then one `+roll 1°` only, with observer and E-stop ready;
- no `-roll/pitch/yaw`, Quest real teleoperation, `+z`, speed increase, or PVAT-frequency change until the new session is reviewed.

Add report checkboxes for `ik_tracking_backpressure`, maximum `solution_step_rad`, maximum `tracking_error_rad`, PVAT mode counts, actual max joint speed/acceleration, final IDLE state, and operator-observed direction/abnormal translation.

- [ ] **Step 4: Run all non-hardware verification**

Run from `backend`:

```powershell
.\.venv\Scripts\python.exe -m pytest -q
```

Expected: all backend tests pass with zero failures.

Run from `web` because the public constraint enum remains unchanged:

```powershell
npm.cmd test
npm.cmd run build
```

Expected: all frontend tests pass and the production build exits zero. These commands are software regression evidence only; they do not claim real hardware passed.

- [ ] **Step 5: Request independent code review and fix findings**

Review the full diff against the approved spec with emphasis on:

- no full-solution history commit before successful `move_pvat()`;
- at most three IK calls per request;
- catch-up never advances target history or persistent failure count;
- stop/generation invalidation prevents late PVAT, state resurrection, and late faults;
- the `0.25 rad` envelope never bypasses PVAT speed/acceleration or soft limits;
- logging cannot block stop or change control results.

Rerun the focused test named by each finding after fixing it, then rerun the complete backend suite.

- [ ] **Step 6: Commit Task 7**

```powershell
git add backend/tests/robots/test_lebai_adapter_control.py docs/real-robot-deployment.md docs/real-robot-commissioning-report.md
git commit -m "test: replay real roll tracking backlog"
```

- [ ] **Step 7: Prepare the onsite handoff without claiming hardware success**

Record the final commit, clean status, backend test count, frontend test count/build result, and exact local YAML migration. Push only after user authorization or an already approved execution checkpoint. The laboratory operator must return the complete `session.jsonl`, `summary.json`, smoke terminal output, and a short observation of direction, smoothness, abnormal translation, and stop behavior.
