# Stop-Move-Only Failure Policy Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove automatic `stop_sys` from the production LEBAI stop-failure path without treating a late or failed `stop_move` as a confirmed stop.

**Architecture:** Keep the existing owned `stop_move` request, 200 ms deadline, stationary verification, fault latch, and control-layer motion gate. Remove only the adapter's automatic escalation call and method. Mark each `stop_diagnostics` event with the active policy, then align adapter, control, smoke, and acceptance tests to the new contract. Explicit static-system-stop diagnostic scripts and SDK capability detection remain untouched.

**Tech Stack:** Python 3.11+, asyncio, pytest/pytest-asyncio, FastAPI backend.

**Spec:** `docs/superpowers/specs/2026-09-25-stop-move-only-failure-policy-design.md`

## Global Constraints

- Do not change the 200 ms `StopRpc` deadline or the existing 300 ms stationary window and drift/speed thresholds.
- Do not add retries, suppress `stop_unverified`/`stop_incomplete`, mark late returns confirmed, or broaden fault recovery.
- Do not change explicit `backend/app/commissioning/static_system_stop.py`, the SDK bridge's advertised `stop_sys` capability, or real-robot configuration.
- Do not connect to equipment, start the control stack, switch `mode`, run VR, or present offline tests as physical-stop validation.
- Run selected tests during red/green development. Run the full backend suite **once at the end**, not after every edit; report any failure by name and do not rerun to manufacture green.

## Review Focus

1. Cancellation before acquiring `_sdk_lock`: no `stop_move` or `stop_sys` is sent, but `stop_unverified` remains latched — Task 1 test.
2. A `stop_move` timeout followed by a late return: only one owned request, failure remains latched, lifecycle says `returned_late` — Task 1 test.
3. Repeated shutdown/disconnect after a failed stop: no second request and no `stop_sys` — Task 1 and Task 2 tests.
4. A returned RPC followed by nonstationary/failed snapshot reads: `stop_incomplete`/`stop_unverified` persists without escalation — Task 2 tests.
5. The smoke observer's scripted tail and the explicit static-stop diagnostic must not be confused with the production policy — Task 2 and Task 3 tests.

---

### Task 1: Adapter stop contract and production policy

**Files:**
- Modify: `backend/tests/robots/test_stop_transaction.py`
- Modify: `backend/tests/robots/test_lebai_adapter_control.py`
- Modify: `backend/app/robots/lebai_adapter.py:355-628`

**Interfaces:**
- Consumes: `RealLebaiAdapter.stop(reason)`, `StopTransaction.call(method, send, call)`.
- Produces: unchanged public stop API and fault values; `stop_diagnostics.stop_policy == "stop_move_only"`; no adapter call site for `stop_sys`.

- [ ] **Step 1: Write the failing tests.** In `test_stop_transaction.py`, change the cancellation-before-lock assertion from `['stop_sys']` to `[]` and assert the diagnostic policy. In `test_production_stop_reuses_pending_requests_and_preserves_late_evidence`, use the existing blocked `stop_move` coroutine but require one write and one lifecycle method:

```python
assert client.write_calls == [('stop_move',)]
assert {event['method'] for event in lifecycle} == {'stop_move'}
assert lifecycle[0]['outcome'] == 'returned_late'
assert all(d['stop_policy'] == 'stop_move_only' for d in diagnostics)
assert (await adapter.get_state()).fault == 'stop_unverified'
```

  In `test_lebai_adapter_control.py`, change the existing SDK-error, cancellation, and nonsettled-speed tests to require `client.write_calls == [('stop_move',)]`, retaining their existing exception and fault assertions. Replace `test_stop_sys_records_contended_lock_wait_without_extra_writes` with a test that forces `stop_move` to raise, requires `diagnostic['stop_policy'] == 'stop_move_only'`, and asserts `diagnostic['rpc_calls']` contains only `stop_move`.

- [ ] **Step 2: Run red tests.** From `backend`, run:

```powershell
python -m pytest tests/robots/test_stop_transaction.py::test_production_stop_reuses_pending_requests_and_preserves_late_evidence tests/robots/test_stop_transaction.py::test_cancelled_stop_records_interrupted_sdk_lock_wait tests/robots/test_lebai_adapter_control.py::test_stop_move_failure_escalates_after_the_failed_stop_move -q
```

  Expected: assertion failures on old `stop_sys` calls and/or missing `stop_policy`, not import or collection errors. Rename tests after the red run to remove obsolete `escalates` wording.

- [ ] **Step 3: Implement the smallest adapter change.** Initialize the diagnostic with the explicit marker and remove the escalation call and its private method. Preserve the latch and existing exception paths. Remove the now-unused `client` argument from `_fail_unverified_stop` and its call sites, but do not alter `_call_stop_rpc` or `StopTransaction` deadlines:

```python
diagnostics["stop_policy"] = "stop_move_only"

async def _fail_unverified_stop(
    self, *, diagnostics: dict[str, Any],
    preserve_fault: bool = False,
    from_cancelled_stop: bool = False,
) -> None:
    if not preserve_fault:
        self._latch_unverified_stop(from_cancelled_stop=from_cancelled_stop)
    else:
        self._motion_accepted = False
        self._preflight_ready = False
        self._pump.invalidate()
        self._reset_pvat_history()
    # No automatic system-stop RPC.
```

  Add the diagnostic assignment immediately after the existing dictionary initialization so every other field stays unchanged. The method body above replaces the current one. Delete `_safety_escalate_stop_sys`, including its SDK-lock and `is_connected` RPC branch. Do not delete `LebaiClientProtocol.stop_sys` or explicit static-stop tooling.

- [ ] **Step 4: Run green tests.** Run the three red-test selectors above and the complete `tests/robots/test_stop_transaction.py` and `tests/robots/test_lebai_adapter_control.py`. Expected: no failures in the tests updated by this task. Record any still-failing legacy stop-escalation assertions for Task 2 rather than changing the production policy to satisfy them.

- [ ] **Step 5: Inspect the Task 1 checkpoint.** Run `git diff --check` and retain the selected-test output. Do not commit yet: other existing tests still require the old policy, and the commit should not intentionally leave the stop suite red.

### Task 2: Align stop evidence and commissioning tests

**Files:**
- Modify: `backend/tests/robots/test_lebai_stop_motion_id.py`
- Modify: `backend/tests/robots/test_lebai_stop_evidence.py`
- Modify: `backend/tests/robots/test_lebai_stop_diagnostics.py`
- Modify: `backend/tests/commissioning/test_stop_timeout_lifecycle.py`
- Modify: `backend/tests/commissioning/test_owned_stop_integration.py`
- Modify: `backend/tests/commissioning/test_stop_observation.py`
- Modify: `backend/tests/robots/test_stop_transaction.py` (obsolete escalation-specific tests)
- Modify: `backend/tests/robots/test_lebai_adapter_control.py` (remaining obsolete escalation-specific tests)

**Interfaces:**
- Consumes: Task 1's `stop_diagnostics.stop_policy` and unchanged fault values.
- Produces: tests of one production stop method, preserved stop-failure evidence, and the same observation-tail completeness checks.

- [ ] **Step 1: Establish the red baseline against the new code.** From `backend`, run:

```powershell
python -m pytest tests/robots/test_lebai_stop_motion_id.py tests/robots/test_lebai_stop_evidence.py tests/robots/test_lebai_stop_diagnostics.py tests/commissioning/test_stop_timeout_lifecycle.py tests/commissioning/test_owned_stop_integration.py tests/commissioning/test_stop_observation.py -q
```

  Expected: old assertions that demand `stop_sys`, two lifecycle methods, or stop-method overlap fail. Preserve the output; do not change the application to meet those old assertions.

- [ ] **Step 2: Update assertion contracts.** In each listed adapter evidence and motion-ID test, retain existing fault/stationary/drift checks and require a single `('stop_move',)` production write. For a timeout diagnostic, require exactly one RPC entry and the policy marker:

```python
assert [(call['method'], call['outcome']) for call in event['rpc_calls']] == [
    ('stop_move', 'timeout'),
]
assert event['stop_policy'] == 'stop_move_only'
assert event['latched_fault'] == 'stop_unverified'
```

  For the two commissioning lifecycle tests, replace two-method overlap/reuse assertions with `client.stops == 1`, `client.overlap_seen is False`, and one `stop_move` lifecycle event. Preserve `stop_confirmed` absence, `stop_unverified`, observation duration, read-error accounting, and the `returned_late`/`unknown_on_local_cancel`/`error` expectations for **that one method**. The `PendingStopClient` and `DriftAfterEscalationClient` synthetic tail should start after their first stop request, not after a nonexistent second request. Keep comments explicit that this is scripted observation data, not robot physics.

- [ ] **Step 3: Replace remaining obsolete escalation-only tests with the remaining invariant.** The old `_safety_escalate_stop_sys` monkeypatch test was replaced in Task 1. Update the concurrent-disconnect test to hold `stop_move` itself and verify disconnect waits for the stop transaction before closing. In the recorder-blocked tests, require the recorder cannot change one-method stop failure/diagnostic completion. Keep the existing explicit `test_static_system_stop.py` untouched.

- [ ] **Step 4: Run the Task 2 selected command again.** Expected: exit 0, with no skipped or deselected failure paths. Then run:

```powershell
python -m pytest tests/robots tests/commissioning -k 'stop or motion_id' -q
```

  Any failure in the selected scope must be fixed or named in the handoff; do not use a reduced selection to hide it.

- [ ] **Step 5: Commit the policy and aligned evidence tests together.** Stage Task 1 production/test files and Task 2 test files; inspect `git diff --cached --check`; commit as `fix: use stop-move-only failure policy` only after the selected tests from both tasks pass.

### Task 3: Document the policy and verify the complete backend once

**Files:**
- Modify: `docs/stop-timing-diagnostics.md`
- Modify: `docs/real-robot-deployment.md`
- Modify: `backend/app/acceptance/fake_lebai.py` (replace the old `stop_sys` expectation)
- Modify: `backend/tests/acceptance/test_fake_lebai.py` (assert the new metric value)

**Interfaces:**
- Consumes: Task 1 policy marker and Task 2 one-method evidence contract.
- Produces: operator-facing statement that the escalation policy changed but stop latency/physical result did not become verified.

- [ ] **Step 1: Add the acceptance regression before changing its production harness.** Assert that the existing fake fault scenario still faults while reporting zero automatic system-stop calls:

```python
result = asyncio.run(run_fault_scenario())
assert result.metrics['stop_sys_calls'] == 0
assert result.metrics['verified'] == result.metrics['injected']
```

  Run `python -m pytest tests/acceptance/test_fake_lebai.py -q` from `backend`; expect the old fake harness to fail because it requires `stop_sys`. In `_run_stop_failure_case`, replace `assert "stop_sys" in methods` with `assert "stop_sys" not in methods`, while retaining `assert "stop_move" in methods` and `assert state.fault == "stop_incomplete"`. Keep the `stop_sys_calls` metric for compatibility; its value becomes zero. Do not change fake robot physics or the SDK bridge.

- [ ] **Step 2: Update the two docs.** In `docs/stop-timing-diagnostics.md`, replace the old statement that escalation is unchanged with this precise behavior: after an unconfirmed `stop_move`, no automatic `stop_sys` request is sent, the stop remains failed and latched, and a late return proves neither prompt RPC response nor physical stationarity. In `docs/real-robot-deployment.md`, replace the current claim that a failed stop may auto-escalate with the new policy and explicitly require independent on-site stopping if motion persists. Retain historical R37/R41 records as historical facts, not descriptions of the new code.

- [ ] **Step 3: Run verification once after all edits.** From `backend`, run the full `python -m pytest -q` **once**. Before any completion claim, inspect its complete summary and list each failure by exact test name. Also run `git diff --check` and search for remaining production adapter call sites:

```powershell
rg -n '_safety_escalate_stop_sys|_call_stop_rpc\([^\n]*stop_sys' app/robots
rg -n 'stop_sys' app/commissioning/static_system_stop.py app/robots/lebai_sdk_bridge.py
```

  Expected: no adapter escalation call; explicit static-stop and bridge capability remain. A failing full suite is reported, not silently rerun or called green.

- [ ] **Step 4: Commit documentation and acceptance updates.** Stage only Task 3 files and commit as `docs: explain stop-move-only failure policy`. Report commit IDs and targeted/full-suite output; do not push or schedule a live test without a separate user request.

## Self-review against the spec

- Selected behavior: Tasks 1 and 2 preserve `stop_move`, verification, fault labels, request ownership, and the motion gate while removing automatic `stop_sys`.
- Observability: Task 1 adds the policy marker; Task 2 checks its log shape; Task 3 documents it.
- Failure boundaries: Tasks 1 and 2 cover timeout, exception, cancellation, nonstationary state, repeated stop, disconnect, and late return.
- Out of scope: no changes to real config, explicit static-system-stop diagnostics, physical-stop claims, or the source of the 625 ms latency.
