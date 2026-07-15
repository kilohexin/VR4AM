# Task 12 Report: Quest WebXR Controller Input and Pure VR Mode

## Status

Implemented Quest WebXR controller input and pure immersive-VR rendering for the simulator frontend. The existing `进入 VR` action now requests a real WebXR session, remains truthful during startup/failure/exit, and preserves the Task 11 command-gate safety rules.

## TDD Evidence

### Initial RED

Command:

```powershell
cd web
npm.cmd test -- controllerInput.test.ts xrSession.test.ts
```

Observed result: 2 failed suites because `../src/xr/controllerInput` and `../src/xr/session` did not exist. This was the expected feature-missing failure.

### Core GREEN

Command:

```powershell
npm.cmd test -- controllerInput.test.ts xrSession.test.ts
```

Observed result after the initial implementation: 2 files passed, 15 tests passed.

### Additional RED/GREEN cycles

- Scene/UI handoff tests failed because `SimulationScene.startXR()`, `SimulationScene.stopXR()`, and `ArmPanel.setVRStatus()` did not exist; after implementation, the focused scene/UI set passed 35/35.
- Unsupported WebXR tests failed until every entry attempt immediately disarmed and reset the local lock before browser capability checks.
- HUD stale-error test failed until `clearSceneError()` was added and called on successful session activation.
- The TypeScript build exposed optional `gripSpace`; a regression test proved the reader had incorrectly accepted a pose without it. The reader now returns tracking-invalid and never falls back to `targetRaySpace`.
- A tracking-loss safety test failed until the first valid-to-invalid transition immediately sent disarm and reset lock eligibility; repeated invalid frames do not repeatedly emit that transition action.

Final focused result: 5 files passed, 54 tests passed. After the last tracking-loss addition, the XR/controller subset passed 17/17.

## Implementation

- Added `web/src/xr/controllerInput.ts`:
  - selects only the right-hand input source;
  - consumes only `gripSpace`;
  - copies metres and quaternion in `[x,y,z,w]` order;
  - maps Trigger index 0 and Grip index 1;
  - clamps Trigger to `[0,1]` and uses `pressed || value >= 0.5` for Grip;
  - returns tracking-invalid neutral input for missing grip space or pose.
- Added `web/src/xr/session.ts`:
  - requests only `immersive-vr` with required `local-floor`;
  - owns fresh Quest session IDs and monotonic per-session sequences;
  - sends directly at no more than 60 Hz with no queue;
  - disarms and locks on entry attempts, tracking loss, visibility loss, session end, and disposal cleanup;
  - emits one final tracking-invalid frame on visibility loss;
  - contains browser/DOMException details behind readable Chinese feedback;
  - removes listeners and blocks stale callbacks after disposal.
- Updated `SimulationScene`:
  - enables `renderer.xr`;
  - hands rendering to `renderer.setAnimationLoop` during XR;
  - stops desktop RAF/input on entry and restores desktop RAF after exit;
  - keeps shared robot visualization/rendering behavior in both modes.
- Updated `ArmPanel`, `Hud`, and `main.ts`:
  - truthful `正在进入 VR` / `退出 VR` / `进入 VR` states;
  - real enter/exit action wiring;
  - readable Chinese failure feedback without raw browser text;
  - existing socket, simulator-only, grip-release, and explicit-arm paths remain authoritative.

## Files Changed

- `.superpowers/sdd/task-12-report.md`
- `web/src/main.ts`
- `web/src/scenes/simulationScene.ts`
- `web/src/ui/armPanel.ts`
- `web/src/ui/hud.ts`
- `web/src/xr/controllerInput.ts`
- `web/src/xr/session.ts`
- `web/tests/armPanel.test.ts`
- `web/tests/controllerInput.test.ts`
- `web/tests/hud.test.ts`
- `web/tests/simulationScene.test.ts`
- `web/tests/xrSession.test.ts`

## Final Verification

From `web`:

```powershell
npm.cmd test
```

Result: PASS — 10 test files, 101 tests, 0 failures.

```powershell
npm.cmd run build
```

Result: PASS — `tsc --noEmit` and Vite production build completed. Vite emitted its non-blocking existing-style warning that the minified main chunk exceeds 500 kB.

From the worktree root:

```powershell
git diff --check
```

Result: PASS — no whitespace errors. Git printed only line-ending conversion notices.

## Self-Review

- Confirmed no imports, discovery, selector, or connection logic for Lebai or any real robot SDK/backend.
- Confirmed no camera, passthrough, anchors, hit-test, plane detection, MR, Cannon-es, grasp physics, or dataset work.
- Confirmed `targetRaySpace` appears only in a negative regression test proving it is not used.
- Confirmed session restart changes the session ID, resets sequence to zero, and never emits `arm_request` automatically.
- Confirmed visibility loss and tracking loss close both backend and local command gates; reconnect/restart still require a fresh released-Grip observation and explicit arm request.
- Confirmed unsupported, rejected, and reference-space failure paths never publish an active status and do not expose raw `DOMException` text.
- Preserved the unrelated untracked `backend/vr4arm_backend.egg-info/` directory and excluded it from staging.

## Browser and Manual Limitations

- The HTTPS Vite development server started successfully at `https://127.0.0.1:5173/`.
- The in-app browser runtime reported no available browser bindings (`[]`), so a rendered unsupported-WebXR click smoke could not be completed in this environment. The server was stopped and no unrelated browser fallback was used.
- No Quest headset was attached. Physical controller handedness, Quest browser permission UI, headset visibility transitions, and perceived in-headset rendering require manual Quest validation.

## Concerns

- Quest hardware/manual WebXR validation remains outstanding.
- The Vite build reports a 639 kB minified main chunk warning; bundle splitting is outside Task 12 and does not affect correctness.

## Review Fix Addendum (2026-07-15)

### RED Evidence

Focused command:

```powershell
cd web
npm.cmd test -- controllerInput.test.ts simulationScene.test.ts xrSession.test.ts appDisposal.test.ts
```

Observed result before the review fixes: FAIL — 4 failed files, 7 failed tests, and the new `appDisposal` suite could not import the intentionally missing unload helper. The failures deterministically reproduced cleanup reentry during a pending `stopXR`, an `active` status after `end` during pending `startXR`, a missing synchronous dispose disarm, missing desktop RAF recovery after `setSession(null)` rejection, and throws for absent/null `gamepad.buttons`. Vitest also reported the raw renderer teardown rejection as one unhandled rejection.

Additional startup RED command:

```powershell
npm.cmd test -- xrSession.test.ts
```

Observed result: FAIL — 1 of 17 tests failed because a session acquired before `host.startXR()` rejected was not explicitly ended (`session.end` called 0 times).

### GREEN Evidence

Final focused command:

```powershell
npm.cmd test -- controllerInput.test.ts simulationScene.test.ts xrSession.test.ts appDisposal.test.ts
```

Result: PASS — 4 files, 37 tests, 0 failures, and no unhandled rejections.

### Review Fix Implementation

- Replaced controller-global session callbacks/state with a `SessionContext` that owns its generation, listeners, frame identity/sequence, input state, activation barrier, and cleanup promise.
- Blocked `enterVR()` while prior cleanup is pending. Cleanup waits for pending renderer activation to settle before calling `stopXR()`, so stale teardown cannot affect a newer session.
- Added ownership checks after every WebXR/renderer await and before publishing `active`; an `end` during pending `startXR()` now closes the context and never publishes active.
- Made `dispose()` synchronously mark the controller disposed, invalidate the generation, disarm, reset the lock, and publish neutral controller state before awaiting `session.end()` or renderer teardown. Stale frame/session callbacks are inert from that point.
- Added `disposeAppForUnload()` and wired `main.ts` through it so XR safety disposal is invoked before scene disposal and socket close; the disarm send is attempted while the socket is still open.
- Made `SimulationScene.stopXR()` restore desktop RAF in `finally` even when `renderer.xr.setSession(null)` rejects. Session cleanup contains renderer rejection, reports concise Chinese feedback, and does not leak raw `DOMException` text or an unhandled rejection.
- Renderer startup rejection now ends the acquired session, completes local teardown, restores desktop operation, and reports readable Chinese feedback.
- Treats absent/null `gamepad.buttons` as neutral tracking-invalid controller input without throwing.

### Final Review Verification

From `web`:

```powershell
npm.cmd test
```

Result: PASS — 11 test files, 110 tests, 0 failures.

```powershell
npm.cmd run build
```

Result: PASS — `tsc --noEmit` and Vite production build completed. The existing non-blocking chunk-size warning remains (`640.21 kB` minified main chunk).

From the worktree root:

```powershell
git diff --check
```

Result: PASS — no whitespace errors; Git printed only line-ending conversion notices.

### Review Fix Files

- `.superpowers/sdd/task-12-report.md`
- `web/src/appDisposal.ts`
- `web/src/main.ts`
- `web/src/scenes/simulationScene.ts`
- `web/src/xr/controllerInput.ts`
- `web/src/xr/session.ts`
- `web/tests/appDisposal.test.ts`
- `web/tests/controllerInput.test.ts`
- `web/tests/simulationScene.test.ts`
- `web/tests/xrSession.test.ts`

### Remaining Concerns

- Quest hardware/manual WebXR validation remains outstanding.
- The existing Vite main-chunk warning remains outside this safety-fix scope.
