# Simulation Scale Rehearsal Compatibility Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Allow safe scale changes while the Fake robot is armed but idle, and make offline rehearsal generate ordinary VRFrame motion using the authoritative scale active when the run starts.

**Architecture:** The backend remains authoritative for scale mutations and broadens only the Fake, anchor-free stopped state to include `ARMED`. The frontend panel mirrors that state. `OfflineRehearsalController` snapshots `robot_state.translation_scale` at start, uses it for all inverse controller samples, and fails closed if the value is invalid or changes during the run.

**Tech Stack:** Python 3/FastAPI control layer with pytest; TypeScript/Vite with Vitest and jsdom.

## Global Constraints

- Only `LEBAI_FAKE` can change scale from the page; real `LEBAI` stays YAML-controlled and read-only.
- Scale range remains `0.5–2.0` in exact `0.1` steps, default `1.5:1` from configuration.
- Never change scale with Grip pressed, an active mapper anchor, motion in progress, or rehearsal active.
- Rehearsal uses one WebSocket and ordinary VRFrame/control/safety paths; it never bypasses the mapper or limiter.

---

### Task 1: Allow stopped ARMED scale changes

**Files:**
- Modify: `backend/tests/control/test_robot_control.py`
- Modify: `backend/app/control/robot_control.py`
- Modify: `web/tests/simulationSettingsPanel.test.ts`
- Modify: `web/src/ui/simulationSettingsPanel.ts`
- Modify: `web/src/main.ts`

**Interfaces:**
- Consumes: `RobotControl.set_simulation_scale(value, automation_active)` and `SimulationSettingsPanel.update(state)`.
- Produces: identical scale request/result protocol with `ARMED + IDLE + no Grip + no anchor` accepted only for `LEBAI_FAKE`.

- [ ] Add backend and frontend tests proving stopped `ARMED` is editable while `ACTIVE`, motion, automation, anchor, and real runtime remain rejected.
- [ ] Run focused pytest/Vitest and confirm failures are caused by the existing mode allowlists.
- [ ] Add `ARMED` to the stopped-mode allowlists; retain backend anchor and runtime checks.
- [ ] Re-run focused tests and commit the scoped change.

### Task 2: Lock rehearsal to authoritative scale

**Files:**
- Modify: `web/tests/offlineRehearsalController.test.ts`
- Modify: `web/src/rehearsal/offlineRehearsalController.ts`
- Modify: `web/src/rehearsal/types.ts`

**Interfaces:**
- Consumes: finite `RobotStateMessage.translation_scale` from the latest authoritative Fake state.
- Produces: a per-run `rehearsalTranslationScale` used by every `nextControllerSample(...)` call and cleared on cleanup/reset.

- [ ] Add regression tests proving scale `1.5` produces the inverse controller displacement required for the fixed prep TCP, and invalid/missing or changed scale fails closed.
- [ ] Run focused Vitest and confirm current fixed `0.5` behavior fails the displacement assertion.
- [ ] Snapshot the scale at `start()`, replace fixed-scale calls with the snapshot, validate it on advancing states, and clear it when the run ends.
- [ ] Re-run controller/trajectory tests and commit the scoped change.

### Task 2B: Bound pick-and-place by confirmed substeps

**Files:**
- Modify: `web/tests/offlineRehearsalController.test.ts`
- Modify: `web/src/rehearsal/offlineRehearsalController.ts`
- Modify: `web/src/rehearsal/types.ts`

**Interfaces:**
- Consumes: existing pick/place steps and three-sample confirmation rule.
- Produces: `pickStepTimeoutMs = 15_000`, renewed only when a pick/place substep is authoritatively completed; the 300-second full-run limit is unchanged.

- [ ] Add a regression where each substep takes 9 seconds and must renew to a fresh 15-second budget after confirmation.
- [ ] Run focused Vitest and confirm the existing shared 8-second phase deadline fails.
- [ ] Add the pick-step timeout and renew it at `pick_attach`, `pick_lift`, `pick_transfer`, `pick_lower`, and `pick_release` transitions.
- [ ] Re-run controller tests and the rendered full rehearsal.

### Task 2C: Renew confirmed soft-boundary retreat

**Files:**
- Modify: `web/tests/offlineRehearsalController.test.ts`
- Modify: `web/src/rehearsal/offlineRehearsalController.ts`

**Interfaces:**
- Consumes: three consecutive `workspace_boundary` confirmations.
- Produces: one fresh 8-second retreat deadline while retaining the existing boundary distance and limiter behavior.

- [ ] Add a regression proving two confirmations do not renew and the third enters retreat with 8 seconds.
- [ ] Run focused Vitest and confirm the existing shared deadline remains partially consumed.
- [ ] Renew only at the authoritative `boundary_outward -> boundary_retreat` transition.
- [ ] Re-run controller tests and rendered rehearsal.

### Task 3: Verification and rendered handoff

**Files:**
- Test only; no planned production edits.

**Interfaces:**
- Consumes: Tasks 1–2.
- Produces: fresh regression evidence and a restarted Fake application for operator testing.

- [ ] Run focused backend and frontend tests.
- [ ] Run full backend pytest, full frontend Vitest, and `npm.cmd run build`.
- [ ] Restart `scripts/run_offline_rehearsal.py`, confirm ports 8000/5173 and Fake health.
- [ ] Verify the page control is editable in stopped ARMED state and inspect the first rehearsal phase without claiming a complete browser rehearsal if the local certificate blocks Browser automation.
