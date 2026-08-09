# Offline Rehearsal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a PC-only, single-controller rehearsal that drives the existing `LEBAI_FAKE` RealLebaiAdapter path through normal teleoperation messages, renders the result, writes authoritative offline reports, and makes every non-hardware gate repeatable without weakening safety coverage.

**Architecture:** A browser-side `OfflineRehearsalController` owns a deterministic phase machine and supplies synthetic controller samples to the existing `SimulationScene`, which continues publishing ordinary `VRFrame` messages over the one owner WebSocket. Three new report-only WebSocket messages let a Fake-only backend store bind browser phase results to authoritative robot state, diagnostics, Git provenance, and model hashes; they never carry motion targets. Existing Virtual/Fake acceptance remains the safety oracle, while the new rehearsal gate verifies the PC workflow and permanently reports `hardware_verified=false`.

**Tech Stack:** Python 3.11+, FastAPI, asyncio, Pydantic, pytest, TypeScript 5.9, Vitest 4, Three.js, Vite, JSON/Markdown, Fake Lebai digital twin.

## Global Constraints

- Work only on local branch `codex/offline-rehearsal`; create local commits, but do not merge or push.
- Do not import, instantiate, or call the real Lebai SDK in implementation or tests.
- Offline rehearsal is enabled only for `backend=LEBAI_FAKE`, `runtime=DIGITAL_TWIN`, `digital_twin=true`, and `hardware_verified=false`.
- Preserve the existing one-WebSocket-owner policy; do not add an observer connection or a second control channel.
- Report lifecycle messages contain metadata only. Arm, Home, stop, reset, Grip, TCP mapping, IK, and PVAT must continue through existing production paths.
- Every generated rehearsal and acceptance report must contain `hardware_verified: false` and the fixed eight-item hardware-pending list.
- Translation targets are `±0.020 m`, rotation targets are `±8°`, translation tolerance is `0.003 m`, and rotation tolerance is `2°`. Per the 2026-08-09 user ruling, each individual translate/return and rotate/return target has an `8 s` deadline; that deadline is renewed only after three strictly advancing authoritative confirmations advance to the next target. Other phases retain an `8 s` deadline, and the complete rehearsal has a `300 s` hard deadline.
- Per the later 2026-08-09 user rulings, Fake rehearsal keeps the existing Home definition unchanged, then uses ordinary grip-held `VRFrame` messages on the existing owner socket to reach the deterministic prep pose frozen in `config/fake-offline-rehearsal.json`. Three strictly advancing authoritative states inside `3 mm / 2°` are required before rebasing the rehearsal anchors there. Prep alone has a bounded `15 s` deadline; translate/return and rotate/return remain `8 s` per subtarget, other non-motion phases remain `8 s`, and the full run remains `300 s`. This preparation is available only for `LEBAI_FAKE / hardware_verified=false`; it does not change real Home or real control behavior.
- The complete soak still executes 30,000 cycles and all seven scheduled safety events.
- Do not make the soak gate green by deleting coverage or merely replacing `<15 s` with a larger ordinary threshold.
- A full soak has a `60 s` hard timeout; throughput below `2000 steps/s` is a warning; the `2.0` minute throughput must remain at least `70%` of the `0.5` minute throughput after a `0.1` minute warm-up.
- PC manual input and automatic rehearsal are mutually exclusive. Page hide, unload, socket loss, ownership loss, FAULT, STALE, invalid data, or timeout must end frame generation and invoke the existing stop path.
- Do not add Playwright, WebXR emulation, a trajectory editor, a scripting language, data collection, or a general fault-injection API.

## File Structure

### Backend and scripts

- `backend/app/rehearsal/report.py`: Fake-only run ownership, phase ordering, provenance, atomic JSON/Markdown report writing, and disconnect abort.
- `backend/app/rehearsal/__init__.py`: package marker and public report-store exports.
- `backend/app/schemas/messages.py`: strict rehearsal client/server protocol models and phase enums.
- `backend/app/api/teleop_ws.py`: report-message routing on the existing owner socket; no motion bypass.
- `backend/app/main.py`: construct one `OfflineRehearsalStore` and expose it through app state.
- `scripts/soak_simulator.py`: reusable timed measurement and relative-throughput benchmark helpers.
- `scripts/accept_virtual_lm3.py`: include stable soak performance data and warning status.
- `scripts/run_offline_rehearsal.py`: start/stop Fake backend and Vite from one PC command.
- `scripts/accept_offline_rehearsal.py`: aggregate targeted rehearsal checks, provenance, and report invariants.

### Frontend

- `web/src/rehearsal/types.ts`: phase order, config, controller sample, scene snapshot, run snapshot, and protocol-facing result types.
- `web/src/rehearsal/trajectory.ts`: finite pose math, axis targets, quaternion errors, and bounded closed-loop controller updates.
- `web/src/rehearsal/offlineRehearsalController.ts`: deterministic phase machine and fail-closed cleanup.
- `web/src/ui/offlineRehearsalPanel.ts`: Fake-only start/stop UI and progress/result rendering.
- `web/src/protocol/messages.ts`: rehearsal wire types and exact runtime guards.
- `web/src/transport/teleopSocket.ts`: send rehearsal metadata and dispatch validated rehearsal feedback.
- `web/src/scenes/simulationScene.ts`: exclusive synthetic-input seam and scene snapshot access.
- `web/src/scenes/kinematicGraspController.ts`: read-only carried-block and placement snapshot.
- `web/src/ui/armPanel.ts`: external automation lock that prevents manual actions while a run is active.
- `web/src/ui/hud.ts`, `web/src/main.ts`, `web/src/styles.css`: panel host, application wiring, layout, and permanent digital-twin copy.

### Tests and documentation

- Backend tests mirror each backend module and WebSocket behavior.
- Frontend tests mirror each new frontend unit and affected integration.
- `docs/offline-rehearsal.md`: operator procedure, report interpretation, and hardware boundary.
- `README.md`: link to the new procedure without changing the true-hardware status.

---

### Task 1: Stabilize soak correctness and performance evidence

**Files:**
- Modify: `scripts/soak_simulator.py`
- Modify: `backend/tests/test_soak.py`
- Modify: `scripts/accept_virtual_lm3.py`
- Modify: `backend/tests/scripts/test_accept_virtual_lm3.py`

**Interfaces:**
- Produces: `SoakMeasurement`, `measure_soak(minutes, seed, *, runner, perf_counter)`, `benchmark_soak(*, runner, perf_counter)`, and `run_full_soak_process(repo_root, timeout_s=60.0)`.
- Produces report keys: `soak.duration_s`, `soak.steps_per_second`, and `soak_performance.{short_steps_per_second,long_steps_per_second,long_to_short_ratio,warning,passed}`.
- Consumed by: Tasks 8–9 final gates.

- [ ] **Step 1: Replace the brittle wall-clock test with failing safety, determinism, and benchmark tests**

Write tests that keep one complete 10-minute run, use `0.2` minute runs for seed equality, and require explicit benchmark semantics:

```python
def test_ten_minute_soak_preserves_every_safety_invariant() -> None:
    summary = _load_soak_module().run_soak(minutes=10, seed=42)
    _assert_safety_invariants(summary)


def test_short_soak_is_deterministic_and_seeded() -> None:
    run_soak = _load_soak_module().run_soak
    assert run_soak(minutes=0.2, seed=42) == run_soak(minutes=0.2, seed=42)
    assert (
        run_soak(minutes=0.2, seed=42)["path_checksum"]
        != run_soak(minutes=0.2, seed=43)["path_checksum"]
    )


def test_benchmark_separates_warning_from_failure() -> None:
    module = _load_soak_module()
    calls: list[tuple[float, int]] = []
    clock_values = iter((0.0, 0.6, 0.6, 3.933333333333333))

    def fake_counter() -> float:
        return next(clock_values)

    def fake_runner(minutes: float, seed: int) -> dict[str, object]:
        calls.append((minutes, seed))
        return {"control_steps": round(minutes * 3000), "error_count": 0}

    benchmark = module.benchmark_soak(
        runner=fake_runner,
        perf_counter=fake_counter,
    )
    assert calls == [(0.1, 42), (0.5, 42), (2.0, 42)]
    assert benchmark["long_to_short_ratio"] >= 0.70
    assert benchmark["passed"] is True
    assert benchmark["warning"] is True
```

- [ ] **Step 2: Run the new tests and verify the missing interfaces fail**

Run:

```powershell
cd backend
python -m pytest tests/test_soak.py -q
```

Expected: FAIL because `SoakMeasurement`, `measure_soak`, and `benchmark_soak` do not exist; the old `<15.0` assertion must still be visible before implementation.

- [ ] **Step 3: Implement timed measurement without changing the safety summary**

Add to `scripts/soak_simulator.py`:

```python
@dataclass(frozen=True)
class SoakMeasurement:
    summary: dict[str, Any]
    duration_s: float
    steps_per_second: float


def measure_soak(
    minutes: float,
    seed: int,
    *,
    runner: Callable[[float, int], dict[str, Any]] = run_soak,
    perf_counter: Callable[[], float] = time.perf_counter,
) -> SoakMeasurement:
    started = perf_counter()
    summary = runner(minutes, seed)
    duration_s = max(perf_counter() - started, 1e-12)
    return SoakMeasurement(
        summary=summary,
        duration_s=duration_s,
        steps_per_second=float(summary["control_steps"]) / duration_s,
    )
```

Implement `benchmark_soak()` with `0.1` warm-up, measured `0.5` and `2.0` minute runs, `ratio >= 0.70`, and `warning = long_steps_per_second < 2000.0`. The function must reject non-finite/non-positive measurements and return JSON-safe finite numbers.

- [ ] **Step 4: Enforce the 60-second hard timeout in an isolated soak process**

Add `run_full_soak_process()` to `scripts/accept_virtual_lm3.py`. It invokes
`scripts/soak_simulator.py --minutes 10 --seed 42` with `subprocess.run(...,
timeout=60.0)`, measures total duration, parses the single JSON object, and
returns `SoakMeasurement`. Convert `subprocess.TimeoutExpired`, nonzero exit,
invalid JSON, missing counts, and non-finite metrics into a failed gate report.
This prevents a stuck soak from keeping the gate process alive.

- [ ] **Step 5: Make the Virtual gate report the isolated measurement and relative benchmark**

Run its single full soak through `run_full_soak_process()`, fail if the process
times out or the relative benchmark fails, and merge these fields into the report:

```python
soak = {
    **measurement.summary,
    "duration_s": measurement.duration_s,
    "steps_per_second": measurement.steps_per_second,
}
passed = (
    commands_passed
    and scenarios_passed
    and soak["error_count"] == 0
    and soak_performance["passed"]
)
```

Update the acceptance unit test to assert that a low absolute throughput produces `warning=True` while `passed` remains driven by safety, hard timeout, and scaling.

- [ ] **Step 6: Run focused and full backend verification**

Run:

```powershell
cd backend
python -m pytest tests/test_soak.py tests/scripts/test_accept_virtual_lm3.py -q
python -m pytest -q
```

Expected: all tests PASS; the complete soak runs once per full suite, retains `control_steps=30000`, `injected_events=7`, and no `<15.0` assertion remains.

- [ ] **Step 7: Commit the stable soak gate**

```powershell
git add scripts/soak_simulator.py scripts/accept_virtual_lm3.py backend/tests/test_soak.py backend/tests/scripts/test_accept_virtual_lm3.py
git commit -m "test: stabilize offline soak evidence"
```

---

### Task 2: Add Fake-only rehearsal reports and strict lifecycle storage

**Files:**
- Create: `backend/app/rehearsal/__init__.py`
- Create: `backend/app/rehearsal/report.py`
- Create: `backend/tests/rehearsal/__init__.py`
- Create: `backend/tests/rehearsal/test_report.py`

**Interfaces:**
- Produces: `REHEARSAL_PHASES`, `HARDWARE_PENDING`, `RehearsalReportStore`, `BeginResult`, `FinishResult`.
- Produces methods: `begin(owner, plan_version, state, diagnostics)`, `record_phase(owner, run_id, result, state, diagnostics)`, `finish(owner, run_id, outcome, failure, state, diagnostics)`, and `abort_owner(owner, reason, state, diagnostics)`.
- Consumed by: Task 3 WebSocket routing and Task 8 offline gate.

- [ ] **Step 1: Write failing store tests for runtime, ownership, ordering, atomic reports, and abort**

Use `tmp_path`, injected clock/provenance providers, and minimal valid state/diagnostics dictionaries:

```python
PROVENANCE = {
    "git": {"commit": "a" * 40, "dirty": False, "dirty_paths": []},
    "model": {
        "kinematics_sha256": "b" * 64,
        "fake_config_sha256": "c" * 64,
        "glb_sha256": "d" * 64,
    },
}
READY_STATE = {
    "mode": "READY",
    "robot_state": "IDLE",
    "fault": None,
    "constraint": None,
}
DIAGNOSTICS = {
    "runtime": "LEBAI_FAKE",
    "hardware_verified": False,
    "log_session_dir": "logs/commissioning/session-1",
    "dropped_events": 0,
}


def phase_result(phase: str) -> dict[str, object]:
    return {
        "phase": phase,
        "status": "passed",
        "started_client_ms": 10.0,
        "completed_client_ms": 20.0,
        "target": {},
        "measurements": {},
        "failure": None,
    }


@pytest.mark.asyncio
async def test_finish_writes_authoritative_false_hardware_report(tmp_path: Path) -> None:
    store = RehearsalReportStore(
        root=tmp_path,
        runtime="LEBAI_FAKE",
        provenance=lambda: PROVENANCE,
        now=lambda: datetime(2026, 8, 2, tzinfo=timezone.utc),
    )
    owner = object()
    begun = await store.begin(owner, 1, READY_STATE, DIAGNOSTICS)
    for phase in REHEARSAL_PHASES:
        await store.record_phase(
            owner,
            begun.run_id,
            phase_result(phase),
            READY_STATE,
            DIAGNOSTICS,
        )
    finished = await store.finish(
        owner, begun.run_id, "passed", None, READY_STATE, DIAGNOSTICS
    )
    payload = json.loads(finished.json_path.read_text(encoding="utf-8"))
    assert payload["hardware_verified"] is False
    assert payload["hardware_pending"] == list(HARDWARE_PENDING)
    assert [item["phase"] for item in payload["phases"]] == list(REHEARSAL_PHASES)
    assert not finished.json_path.with_suffix(".json.tmp").exists()
```

Also require: `LEBAI` and `SIMULATOR` reject begin, another owner cannot append, skipped/repeated phases reject, non-finite nested payloads reject, a failed outcome still writes both files, and `abort_owner()` is idempotent.

- [ ] **Step 2: Run the report tests and verify import failure**

Run:

```powershell
cd backend
python -m pytest tests/rehearsal/test_report.py -q
```

Expected: FAIL with `ModuleNotFoundError: No module named 'app.rehearsal'`.

- [ ] **Step 3: Implement the report store with one active run per owner socket**

Use immutable public results and a private active-run record:

```python
REHEARSAL_PHASES = (
    "identity_preflight", "home", "arm_and_anchor", "translate", "rotate",
    "gripper", "pick_place", "soft_boundary", "tracking_loss",
    "recovery_and_home", "final_stop", "finalize",
)

HARDWARE_PENDING = (
    "sdk_connection", "tcp_home_joint_limits", "translation_direction",
    "rotation_direction", "gripper_direction_force", "pvat_tracking_latency",
    "stop_distance_estop", "lightweight_grasp_release",
)

@dataclass(frozen=True)
class BeginResult:
    run_id: str


@dataclass(frozen=True)
class FinishResult:
    json_path: Path
    markdown_path: Path
```

Guard all lifecycle mutations with one `asyncio.Lock`. Store the actual `owner` object by identity, enforce exact phase order, recursively reject non-finite/non-JSON-safe payloads, snapshot server state/diagnostics at every phase, and write `run-id.json` plus `run-id.md` through same-directory temporary files followed by `Path.replace()`.

- [ ] **Step 4: Add authoritative provenance and Markdown rendering**

Production provenance must include Git HEAD/dirty paths and SHA-256 for:

```text
config/lm3_visual_kinematics_v1.json
config/fake-lebai.yaml
web/public/models/Lebai_LM3.glb
```

The client cannot supply or override these fields. Markdown must show the offline identity, overall outcome, every phase, first failure, log directory, and all eight hardware-pending items.

- [ ] **Step 5: Run focused tests and commit**

```powershell
cd backend
python -m pytest tests/rehearsal/test_report.py -q
cd ..
git add backend/app/rehearsal backend/tests/rehearsal
git commit -m "feat: record offline rehearsal reports"
```

Expected: all report tests PASS.

---

### Task 3: Route rehearsal metadata on the existing owner WebSocket

**Files:**
- Modify: `backend/app/schemas/messages.py`
- Modify: `backend/app/api/teleop_ws.py`
- Modify: `backend/app/main.py`
- Modify: `backend/tests/contract/test_messages.py`
- Modify: `backend/tests/api/test_teleop_ws.py`
- Modify: `backend/tests/api/test_health.py`

**Interfaces:**
- Consumes: Task 2 `RehearsalReportStore`.
- Produces client models: `OfflineRehearsalBeginMessage`, `OfflineRehearsalPhaseMessage`, `OfflineRehearsalFinishMessage`.
- Produces server payloads: `offline_rehearsal_begin_result`, `offline_rehearsal_phase_ack`, and `offline_rehearsal_finish_result`.
- Consumed by: Task 4 frontend protocol.

- [ ] **Step 1: Write failing strict-schema tests**

Require exact keys, finite nested payloads, phase literals, run/request IDs of 1–64 code points, `plan_version=1`, and outcomes `passed|failed|aborted`:

```python
def test_rehearsal_phase_rejects_motion_fields_and_non_finite_metrics() -> None:
    valid = {
        "v": 1,
        "type": "offline_rehearsal_phase",
        "request_id": "phase-1",
        "run_id": "run-1",
        "phase": "home",
        "status": "passed",
        "started_client_ms": 10.0,
        "completed_client_ms": 20.0,
        "target": {},
        "measurements": {"max_error": 0.0},
        "failure": None,
    }
    OfflineRehearsalPhaseMessage.model_validate(valid)
    with pytest.raises(ValidationError):
        OfflineRehearsalPhaseMessage.model_validate({**valid, "target_q": [0] * 6})
    with pytest.raises(ValidationError):
        OfflineRehearsalPhaseMessage.model_validate(
            {**valid, "measurements": {"max_error": math.nan}}
        )
```

- [ ] **Step 2: Write failing WebSocket lifecycle tests**

Cover accepted begin/phase/finish, non-Fake begin rejection, wrong-owner/run rejection, disconnect abort, and confirmation that no rehearsal message calls `arm`, `home`, `disarm`, `command_tcp`, or `set_gripper`.

- [ ] **Step 3: Run the focused tests to verify RED**

```powershell
cd backend
python -m pytest tests/contract/test_messages.py tests/api/test_teleop_ws.py -q -k rehearsal
```

Expected: FAIL because the message types and router branches do not exist.

- [ ] **Step 4: Add strict Pydantic message models and a parsed union**

Define `RehearsalPhaseName`, `RehearsalOutcome`, and three `StrictMessage` subclasses. Change `_parse_message()` to return:

```python
ClientMessage = (
    VRFrame
    | ClientControlMessage
    | OfflineRehearsalBeginMessage
    | OfflineRehearsalPhaseMessage
    | OfflineRehearsalFinishMessage
)
```

Keep `ClientControlMessage` unchanged so report messages cannot be mistaken for control operations.

- [ ] **Step 5: Construct the store in app lifespan and snapshot authoritative state**

Create `app.state.offline_rehearsal_store` with output root
`REPOSITORY_ROOT / "artifacts" / "acceptance" / "offline-rehearsal"` and current runtime. Add a helper in `teleop_ws.py` that obtains `control.state_message()` and `DiagnosticsStore.message(...)` while holding the existing send lock only for outbound serialization, not filesystem writes.

- [ ] **Step 6: Route begin, phase, finish, and disconnect abort**

Pass the connection's existing `owner_token` into `_run_coupled_session()` and `_receive_messages()`. For each rehearsal message, call the store and reply with the same `request_id`. On connection cleanup, call:

```python
await store.abort_owner(
    owner_token,
    "connection_closed",
    await control.state_message(),
    diagnostics_snapshot(...),
)
```

Map store errors to fixed reject reasons without closing a valid WebSocket. Protocol validation failures still close with code `1008`.

- [ ] **Step 7: Verify backend routing and commit**

```powershell
cd backend
python -m pytest tests/contract/test_messages.py tests/api/test_teleop_ws.py tests/api/test_health.py -q
cd ..
git add backend/app/schemas/messages.py backend/app/api/teleop_ws.py backend/app/main.py backend/tests/contract/test_messages.py backend/tests/api/test_teleop_ws.py backend/tests/api/test_health.py
git commit -m "feat: add rehearsal websocket lifecycle"
```

Expected: all focused tests PASS and existing one-owner tests remain unchanged.

---

### Task 4: Add frontend rehearsal protocol guards and transport dispatch

**Files:**
- Modify: `web/src/protocol/messages.ts`
- Modify: `web/src/transport/teleopSocket.ts`
- Modify: `web/tests/messages.test.ts`
- Modify: `web/tests/teleopSocket.test.ts`
- Create: `schemas/fixtures/offline-rehearsal-finish-valid.json`

**Interfaces:**
- Consumes: Task 3 wire contract.
- Produces: `OfflineRehearsalClientMessage`, `OfflineRehearsalFeedbackMessage`, `isOfflineRehearsalFeedbackMessage()`.
- Produces: `TeleopSocket.sendRehearsal(message)` and final constructor callback `onOfflineRehearsalFeedback`.
- Consumed by: Task 6 controller and Task 7 application wiring.

- [ ] **Step 1: Write failing runtime-guard tests for every response variant**

Test an accepted and rejected begin, phase ack, accepted and rejected finish. Mutate each with extra keys, empty IDs, unknown phases, `hardware_verified:true`, non-finite metrics, or missing paths and require rejection.

```typescript
expect(isOfflineRehearsalFeedbackMessage(validFinish)).toBe(true);
expect(isOfflineRehearsalFeedbackMessage({...validFinish, hardware_verified: true})).toBe(false);
expect(isOfflineRehearsalFeedbackMessage({...validFinish, extra: true})).toBe(false);
```

- [ ] **Step 2: Write failing transport tests**

Require rehearsal sends to drop before open, serialize only while open, dispatch only exact valid server messages, and never route them through Arm/Home callbacks.

- [ ] **Step 3: Run RED tests**

```powershell
cd web
npm.cmd test -- --run tests/messages.test.ts tests/teleopSocket.test.ts
```

Expected: FAIL because rehearsal types, guards, callback, and send method do not exist.

- [ ] **Step 4: Implement exact TypeScript types and recursive JSON-safety guards**

Use a discriminated client union:

```typescript
export type OfflineRehearsalClientMessage =
  | OfflineRehearsalBeginMessage
  | OfflineRehearsalPhaseMessage
  | OfflineRehearsalFinishMessage;

export type OfflineRehearsalFeedbackMessage =
  | OfflineRehearsalBeginResultMessage
  | OfflineRehearsalPhaseAckMessage
  | OfflineRehearsalFinishResultMessage;
```

Reuse `hasExactKeys`, `isFiniteNumber`, and recursive diagnostic JSON validation. Server finish success must require `hardware_verified:false` and exactly eight strings in `hardware_pending`.

- [ ] **Step 5: Extend `TeleopSocket` without changing existing callback positions**

Append `onOfflineRehearsalFeedback` as the final optional constructor parameter, add `sendRehearsal()`, widen the private `send()` union, and dispatch feedback after diagnostics. Existing callers must compile without edits.

- [ ] **Step 6: Run tests/build and commit**

```powershell
cd web
npm.cmd test -- --run tests/messages.test.ts tests/teleopSocket.test.ts
npm.cmd run build
cd ..
git add web/src/protocol/messages.ts web/src/transport/teleopSocket.ts web/tests/messages.test.ts web/tests/teleopSocket.test.ts schemas/fixtures/offline-rehearsal-finish-valid.json
git commit -m "feat: add rehearsal transport protocol"
```

Expected: focused tests and TypeScript build PASS.

---

### Task 5: Add exclusive synthetic input and read-only grasp observations

**Files:**
- Create: `web/src/rehearsal/types.ts`
- Create: `web/src/rehearsal/trajectory.ts`
- Modify: `web/src/scenes/simulationScene.ts`
- Modify: `web/src/scenes/kinematicGraspController.ts`
- Create: `web/tests/rehearsalTrajectory.test.ts`
- Modify: `web/tests/simulationScene.test.ts`
- Modify: `web/tests/kinematicGraspController.test.ts`

**Interfaces:**
- Produces: `OfflineControllerSample`, `OfflineSceneSnapshot`, `GraspSceneSnapshot`, `REHEARSAL_CONFIG`, and `REHEARSAL_PHASES`.
- Produces `SimulationScene.setOfflineController(sample | null)`, `getOfflineSceneSnapshot()`, and `resetOfflineScene()`.
- Produces `KinematicGraspController.snapshot()`.
- Consumed by: Task 6 controller.

- [ ] **Step 1: Write failing pure trajectory tests**

Require exact `±0.020 m` TCP targets, exact `±8°` world-axis quaternion targets, shortest-path quaternion error, bounded finite updates, and rejection of NaN/invalid quaternions:

```typescript
const target = translationTarget(anchor, 'x', 1, 0.020);
expect(target.p).toEqual([anchor.p[0] + 0.020, anchor.p[1], anchor.p[2]]);

const update = nextControllerSample({
  sample: neutralSample,
  actualTcp: anchor,
  targetTcp: target,
  maxPositionStepM: 0.002,
  maxRotationStepRad: Math.PI / 180,
});
expect(update.position[0]).toBeCloseTo(neutralSample.position[0] + 0.002);
```

- [ ] **Step 2: Write failing scene exclusivity and grasp-snapshot tests**

Verify that synthetic input replaces, rather than combines with, pointer/wheel/Grip input; `setOfflineController(null)` restores desktop input only after resetting safety state; page-hidden samples publish invalid tracking; snapshots contain copied finite values and cannot mutate Three.js objects.

- [ ] **Step 3: Run RED tests**

```powershell
cd web
npm.cmd test -- --run tests/rehearsalTrajectory.test.ts tests/simulationScene.test.ts tests/kinematicGraspController.test.ts
```

Expected: FAIL because the rehearsal modules and scene methods do not exist.

- [ ] **Step 4: Implement immutable rehearsal types and pose math**

Define:

```typescript
export interface OfflineControllerSample {
  position: Vec3;
  quaternion: Quat;
  grip: boolean;
  trigger: number;
  trackingValid: boolean;
}

export interface OfflineSceneSnapshot {
  controller: OfflineControllerSample | null;
  carriedBlockId: string | null;
  blocks: ReadonlyArray<{id: string; position: Vec3; sizeM: number}>;
  invalidOverlap: boolean;
}
```

All exported functions validate finite values and return fresh arrays/objects.

- [ ] **Step 5: Add the scene automation seam without a second frame publisher**

Keep the existing animation loop as the only 50 Hz frame source. When `offlineController` is non-null, `sendDesktopFrame()` uses it instead of `DesktopInputSafety` and controller mouse state. Pointer/wheel handlers return immediately while automation is active. Clearing automation resets controller pose/quaternion, Grip/trigger, and input safety before manual control can resume.

- [ ] **Step 6: Expose read-only grasp state and deterministic reset**

`snapshot()` returns carried ID, copied block positions, sizes, and overlap status. `SimulationScene.resetOfflineScene()` calls the existing grasp reset and returns every block to its declared initial transform.

- [ ] **Step 7: Verify and commit**

```powershell
cd web
npm.cmd test -- --run tests/rehearsalTrajectory.test.ts tests/simulationScene.test.ts tests/kinematicGraspController.test.ts
cd ..
git add web/src/rehearsal/types.ts web/src/rehearsal/trajectory.ts web/src/scenes/simulationScene.ts web/src/scenes/kinematicGraspController.ts web/tests/rehearsalTrajectory.test.ts web/tests/simulationScene.test.ts web/tests/kinematicGraspController.test.ts
git commit -m "feat: add offline controller scene seam"
```

Expected: all scene/trajectory tests PASS.

---

### Task 6: Implement the deterministic offline rehearsal phase machine

**Files:**
- Create: `web/src/rehearsal/offlineRehearsalController.ts`
- Create: `web/tests/offlineRehearsalController.test.ts`

**Interfaces:**
- Consumes: Tasks 4–5 protocol, trajectory, synthetic-input, and scene snapshots.
- Produces: `OfflineRehearsalController`, `OfflineRehearsalPorts`, and `OfflineRehearsalSnapshot`.
- Produces methods: `start()`, `requestStop(reason)`, `onRobotState(state)`, `onDiagnostics(message)`, `onFeedback(message)`, `onConnection(status)`, and `dispose()`.
- Consumed by: Task 7 application wiring.

- [ ] **Step 1: Build a deterministic fake-port test harness**

The harness controls time and records every frame/control/report message:

```typescript
function harness() {
  let nowMs = 1000;
  const controls: ClientControlMessage[] = [];
  const reports: OfflineRehearsalClientMessage[] = [];
  const samples: Array<OfflineControllerSample | null> = [];
  const scene: OfflineSceneSnapshot = {
    controller: null,
    carriedBlockId: null,
    blocks: [{id: 'block-orange', position: [0.18, 0.025, -0.32], sizeM: 0.06}],
    invalidOverlap: false,
  };
  return {
    ports: {
      nowMs: () => nowMs,
      sendControl: (message) => controls.push(message),
      sendRehearsal: (message) => reports.push(message),
      setOfflineController: (sample) => samples.push(sample),
      readScene: () => structuredClone(scene),
    },
    advance: (milliseconds: number) => { nowMs += milliseconds; },
    controls,
    reports,
    samples,
  };
}
```

- [ ] **Step 2: Write failing happy-path phase tests**

Drive valid begin feedback, Home result, Arm ack, ACTIVE states, every translation/rotation target and return, gripper thresholds, attached/released block snapshots, soft boundary/retreat, tracking loss, recovery/Home, final stop, and finish. Assert exact phase order and only ordinary VR/control messages carry motion intent.

- [ ] **Step 3: Write failing fail-closed tests**

Parameterize connection loss, page hide, owner occupied, FAULT, STALE outside the planned tracking phase, non-finite state, phase timeout, report rejection, Home rejection, stop-unverified, invalid block placement, and dispose during a pending request. For every case require:

```typescript
expect(harness.samples.at(-1)).toBeNull();
expect(harness.controls.at(-1)?.type).toBe('disarm');
expect(controller.snapshot.phase).toBe('failed');
expect(controller.snapshot.failure).toBe(expectedReason);
```

If the backend remains FAULT/STALE, the controller must retain `stopping` or `failed` rather than claiming a safe reset.

- [ ] **Step 4: Run RED tests**

```powershell
cd web
npm.cmd test -- --run tests/offlineRehearsalController.test.ts
```

Expected: FAIL because `OfflineRehearsalController` does not exist.

- [ ] **Step 5: Implement run identity and request correlation**

`start()` first verifies the latest runtime snapshot is exactly Fake/control-ready, resets the scene, sends `offline_rehearsal_begin`, and waits for the matching request. Ignore stale feedback by request ID and run ID. Disallow a second `start()` until the first run reaches a terminal state.

- [ ] **Step 6: Implement state-driven Home, Arm, anchor, and axis motion**

Keep the configured real/Fake Home definition unchanged. After Home, Arm acceptance, Grip activation, and three ACTIVE confirmations, the Fake-only rehearsal uses `nextControllerSample()` to move through ordinary `VRFrame` messages to the frozen prep pose in `config/fake-offline-rehearsal.json`; no report message may carry this target and no second socket/control path may be introduced. Require three consecutive, strictly advancing authoritative states within `3 mm / 2°`, then rebase both robot and controller anchors at prep. The frozen joint/TCP evidence must stay inside the configured joint/collision envelope with minimum Jacobian singular value at least `0.08`; prep has its own `15 s` deadline and must be validated by the full rendered Fake rehearsal rather than an accelerated test clock. The backend digital-twin prep/envelope test is only a bounded geometric reachability and safety characterization: its shared FakeClock iteration count is not simulated or wall-clock timing evidence. Deterministic controller tests verify deadline selection/reset/fail-closed semantics; the archived rendered run's `11.488 s` `arm_and_anchor` measurement is the timing evidence for prep only.

For each later target, `nextControllerSample()` steps toward the fixed inverse CoordinateMapper command: divide robot translation delta by the explicit Fake `0.5` translation scale and apply the rotation delta one-to-one. It must not integrate target-minus-lagging-actual error into an ever-growing previous command; the hand command is bounded to the exact inverse of the anchor-relative `±0.020 m` target and the identity-mapped `±8°` rotation target. A target completes only after three consecutive, strictly advancing robot states inside tolerance. Each individual target and return starts an `8 s` deadline, renewed only when those confirmations advance the target index. Return to the anchor between axes. Other phase deadlines remain `8 s`; a `300 s` full-rehearsal hard deadline uses the same fail-closed verified stop/report path. Never sleep to assume completion; `nowMs()` is used only for deadlines.

- [ ] **Step 7: Implement gripper, pick/place, boundary, tracking loss, and recovery**

Use trigger `0` for open and `1` for close, confirm from `state.gripper`, select `block-orange`, and derive approach/lift/place targets from copied scene positions. Require attached state, stable carried offset, released support, no overlap, and target-zone intersection. For soft boundary, increment outward until `workspace_boundary`, then retreat to anchor and wait for constraint clear. For tracking loss, publish invalid tracking and require STALE/stop before recovery.

- [ ] **Step 8: Implement one fail-closed terminal path**

Every error calls one idempotent `beginCleanup(reason)` that stops sample generation, clears scene automation, sends existing `disarm`, waits for authoritative stop when possible, submits remaining failed/aborted phase data, and sends finish. `dispose()` and connection loss invoke the same path; neither can resume an old run.

- [ ] **Step 9: Verify and commit**

```powershell
cd web
npm.cmd test -- --run tests/offlineRehearsalController.test.ts tests/rehearsalTrajectory.test.ts
cd ..
git add web/src/rehearsal/offlineRehearsalController.ts web/tests/offlineRehearsalController.test.ts
git commit -m "feat: add deterministic offline rehearsal"
```

Expected: controller and trajectory tests PASS.

---

### Task 7: Add the PC rehearsal panel and application wiring

**Files:**
- Create: `web/src/ui/offlineRehearsalPanel.ts`
- Create: `web/tests/offlineRehearsalPanel.test.ts`
- Modify: `web/src/ui/armPanel.ts`
- Modify: `web/tests/armPanel.test.ts`
- Modify: `web/src/ui/hud.ts`
- Modify: `web/tests/hud.test.ts`
- Modify: `web/src/main.ts`
- Modify: `web/src/styles.css`
- Modify: `web/tests/appDisposal.test.ts`

**Interfaces:**
- Consumes: Task 6 controller snapshots and callbacks.
- Produces: `OfflineRehearsalPanel.update(snapshot, eligibility)`, `ArmPanel.setAutomationActive(active)`, and `Hud.rehearsalContainer`.
- Consumed by: Task 8 launcher/manual smoke and Task 9 final verification.

- [ ] **Step 1: Write failing panel eligibility and rendering tests**

Require the start button only for connected Fake identity with `hardware_verified=false`, READY/IDLE, no fault, and inactive automation. Require permanent “数字孪生，不是真机”, current phase, progress, target/actual error, remaining timeout, first failure, report paths, and all eight pending hardware checks.

- [ ] **Step 2: Write failing mutual-exclusion and disposal tests**

While automation is active, Arm/Home/reset/VR/manual actions are disabled and return without sending. Stop remains enabled. Page hide and `beforeunload` call controller stop/dispose before socket close and scene disposal.

- [ ] **Step 3: Run RED tests**

```powershell
cd web
npm.cmd test -- --run tests/offlineRehearsalPanel.test.ts tests/armPanel.test.ts tests/hud.test.ts tests/appDisposal.test.ts
```

Expected: FAIL because the panel host, lock, and coordinator do not exist.

- [ ] **Step 4: Implement a presentation-only panel**

Constructor:

```typescript
new OfflineRehearsalPanel(
  hud.rehearsalContainer,
  () => void rehearsal.start(),
  () => rehearsal.requestStop('operator_stop'),
);
```

The panel owns no socket, timers, scene, or robot state. It renders immutable controller snapshots and computed eligibility only.

- [ ] **Step 5: Add the external automation lock to `ArmPanel`**

`setAutomationActive(true)` clears no robot state but makes every manual request method return before sending. It changes button copy to `离线演练运行中` and disables VR entry. `false` restores eligibility from the latest authoritative state.

- [ ] **Step 6: Wire one controller through existing callbacks**

In `main.ts`, instantiate the controller after socket/scene/panel construction. Forward every robot state, diagnostics message, rehearsal feedback, and connection change. Runtime eligibility comes from diagnostics plus robot state. Do not create another socket or frame timer. Panel updates and ArmPanel lock derive from `controller.snapshot`.

- [ ] **Step 7: Add layout and responsive styling**

Place the rehearsal panel above diagnostics in the existing right rail on desktop and below the scene on narrow screens. Keep start/stop buttons at least `44 px` high, use existing tone colors, and never hide the digital-twin identity when scrolling.

- [ ] **Step 8: Run full frontend verification and commit**

```powershell
cd web
npm.cmd test -- --run
npm.cmd run build
cd ..
git add web/src/ui/offlineRehearsalPanel.ts web/src/ui/armPanel.ts web/src/ui/hud.ts web/src/main.ts web/src/styles.css web/tests/offlineRehearsalPanel.test.ts web/tests/armPanel.test.ts web/tests/hud.test.ts web/tests/appDisposal.test.ts
git commit -m "feat: add PC offline rehearsal console"
```

Expected: full Vitest PASS and Vite build exits `0`; record but do not conceal the existing chunk-size warning.

---

### Task 8: Add the one-command launcher and offline rehearsal gate

**Files:**
- Create: `scripts/run_offline_rehearsal.py`
- Create: `scripts/accept_offline_rehearsal.py`
- Create: `backend/tests/scripts/test_run_offline_rehearsal.py`
- Create: `backend/tests/scripts/test_accept_offline_rehearsal.py`
- Modify: `.gitignore` only if the existing `artifacts/acceptance/` rule does not already cover the new reports.

**Interfaces:**
- Consumes: Fake stack, frontend build/tests, report schema, and all prior task tests.
- Produces: `LaunchSpec`, `build_launch_spec(env, os_name)`, `run_launcher(spec, process_factory, probe)`, and `run_gate(repo_root, command_runner)`.
- Produces commands: `python scripts/run_offline_rehearsal.py` and `python scripts/accept_offline_rehearsal.py`.
- Produces report: `artifacts/acceptance/offline-rehearsal-latest.json`.

- [ ] **Step 1: Write failing launcher safety tests**

Inject process, port-probe, and signal functions. Require exact child commands, `VR4ARM_CONFIG=config/fake-lebai.yaml`, loopback backend, Vite on `127.0.0.1`, startup failure cleanup, Ctrl+C cleanup in reverse order, and rejection of real config/confirmation environment variables.

```python
launch_spec = build_launch_spec({}, os.name)
assert launch_spec.backend == (
    sys.executable, "scripts/run_fake_lebai_stack.py"
)
assert launch_spec.frontend == (
    npm, "run", "dev", "--", "--host", "127.0.0.1"
)
assert launch_spec.runtime == "LEBAI_FAKE"
```

- [ ] **Step 2: Write failing gate aggregation tests**

Inject a command runner and provenance providers. Require targeted backend rehearsal tests, targeted frontend rehearsal tests, full build, Virtual/Fake report validation, all eight pending checks, valid 64-char hashes, clean failure reports, and nonzero exit on any failed command or `hardware_verified != false`.

- [ ] **Step 3: Run RED tests**

```powershell
cd backend
python -m pytest tests/scripts/test_run_offline_rehearsal.py tests/scripts/test_accept_offline_rehearsal.py -q
```

Expected: FAIL because both scripts do not exist.

- [ ] **Step 4: Implement the launcher with dependency injection and bounded readiness**

Start backend first, poll `http://127.0.0.1:8000/health` until it reports `LEBAI_FAKE`, then start Vite and poll its root. Print only:

```text
Offline rehearsal ready: http://127.0.0.1:5173/
Runtime: LEBAI_FAKE / DIGITAL_TWIN / hardware_verified=false
Press Ctrl+C to stop.
```

On any failure, terminate children, wait up to five seconds, kill only the still-running child, and return nonzero. Never open a browser automatically in tests.

- [ ] **Step 5: Implement the aggregate gate and atomic report**

Run these commands with captured output/counts:

```python
command_specs = (
    ("backend_rehearsal", (sys.executable, "-m", "pytest", "tests/rehearsal", "tests/api/test_teleop_ws.py", "-q"), root / "backend"),
    ("frontend_rehearsal", (npm, "test", "--", "--run", "tests/offlineRehearsalController.test.ts", "tests/offlineRehearsalPanel.test.ts"), root / "web"),
    ("frontend_build", (npm, "run", "build"), root / "web"),
)
```

Load the latest Virtual and Fake reports and require `passed=true`, `hardware_verified=false`, Fake `runtime=LEBAI_FAKE`, all scenarios passed, and all required hashes valid. The offline gate itself always writes `hardware_verified=false` and never upgrades hardware status.

- [ ] **Step 6: Verify scripts and commit**

```powershell
cd backend
python -m pytest tests/scripts/test_run_offline_rehearsal.py tests/scripts/test_accept_offline_rehearsal.py -q
cd ..
git add scripts/run_offline_rehearsal.py scripts/accept_offline_rehearsal.py backend/tests/scripts/test_run_offline_rehearsal.py backend/tests/scripts/test_accept_offline_rehearsal.py .gitignore
git commit -m "feat: add offline rehearsal workflow"
```

Expected: all script tests PASS; do not stage `.gitignore` when it is unchanged.

---

### Task 9: Document, exercise, and freeze the offline milestone

**Files:**
- Create: `docs/offline-rehearsal.md`
- Modify: `README.md`
- Modify: `docs/real-robot-deployment.md`

**Interfaces:**
- Consumes: every prior task.
- Produces: operator procedure and final local milestone evidence.

- [ ] **Step 1: Write the operator procedure**

Document prerequisites, the single launch command, the two buttons, expected phase order, report paths, performance warning meaning, safe stop behavior, troubleshooting, and explicit statement that the result does not authorize real motion. Include the eight onsite checks verbatim.

- [ ] **Step 2: Link the procedure without changing true-hardware status**

Add a README “无真机离线实验演练” section and a deployment cross-link. State that the rehearsal is the recommended preparation when LM3/Quest are unavailable and that onsite still begins in readonly.

- [ ] **Step 3: Run complete backend, frontend, and build verification**

```powershell
cd backend
python -m pytest -q
cd ..\web
npm.cmd test -- --run
npm.cmd run build
```

Expected: zero test failures and build exit `0`. Record the actual counts and the existing bundle warning.

- [ ] **Step 4: Run all three non-hardware gates in order**

```powershell
cd ..
python scripts/accept_virtual_lm3.py
python scripts/accept_fake_lebai.py
python scripts/accept_offline_rehearsal.py
```

Require all three exit `0`, all three `passed=true`, and all three `hardware_verified=false`. Require the Fake and offline reports to retain all eight pending hardware checks.

- [ ] **Step 5: Perform one PC browser smoke run**

Start `python scripts/run_offline_rehearsal.py`, open the printed loopback URL with the available browser-control tool, confirm the permanent Fake identity, start one full rehearsal, observe at least translation, rotation, gripper, pick/place, tracking-loss recovery, and final stop, then inspect both report paths. If browser automation is unavailable, stop and report that this manual/visual item remains pending; do not substitute a unit-test result.

- [ ] **Step 6: Audit hardware boundary and repository hygiene**

```powershell
rg -n "start_sys|set_tcp|init_claw|speedl" backend/app scripts web/src
git diff --check
git status --short
git log --oneline -12
```

Confirm no new production invocation of prohibited SDK operations, generated artifacts remain ignored, only intended files are staged, and no real SDK/hardware call occurred.

- [ ] **Step 7: Commit documentation and final test-only fixes**

```powershell
git add README.md docs/offline-rehearsal.md docs/real-robot-deployment.md
git commit -m "docs: add offline rehearsal procedure"
```

If verification requires a source correction, fix it through a focused failing test and a separate local commit before this documentation commit. Do not amend earlier reviewed task commits.

- [ ] **Step 8: Report the frozen local milestone**

Report every new commit, full test counts, build result/warning, three gate report paths, browser-smoke status, branch/worktree, eight hardware-pending checks, and explicit `not merged / not pushed / hardware not verified` status.
