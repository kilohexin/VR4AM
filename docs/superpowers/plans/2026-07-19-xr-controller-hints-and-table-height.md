# XR Controller Hints and Table Height Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 Quest 3 中显示左右 Touch Plus 的位置和方向，并让未解锁用户通过左摇杆调整虚拟操作台观察高度。

**Architecture:** 输入层每帧读取左右 `gripSpace`、右手控制键和左摇杆；会话层只负责追踪/边沿与把显示样本交给渲染宿主。场景层用独立 `ControllerHints` 渲染双手提示，用纯状态类 `TableHeightController` 计算 XR 专用高度，并通过 `robotVisualRoot` 移动机器人相关视觉对象；机器人控制帧仍只包含右手位姿。

**Tech Stack:** TypeScript 5.9、WebXR Device API、Three.js 0.181、Vitest 4、jsdom、Vite 7

## Global Constraints

- 左手位姿、摇杆和高度偏移不得写入 `VRFrame`，后端继续只接收右手控制数据。
- 手柄提示使用本地 Three.js 基础几何体，不加载外部模型、不增加依赖。
- 左手紫灰色并显示 `L`，右手青色并显示 `R`，两者均显示短朝向轴；追踪丢失立即隐藏。
- 每次进入 XR，以 `headY - 0.72 m` 计算台面基准并限制在 `[0.55 m, 1.10 m]`。
- 左摇杆手动偏移速度为 `0.35 m/s`、死区 `0.2`、范围 `[-0.6 m, +0.6 m]`，最终高度限制在 `[0.30 m, 1.40 m]`。
- 高度调节只在 XR、左手追踪有效、右 Grip 松开、无 pending，且安全阶段为 `locked` 或 `stopped` 时生效。
- 退出 XR 后 `robotVisualRoot.position.y` 恢复 `0`，桌面视图不得受影响。
- `localStorage` 只保存手动偏移，键名为 `vr4arm.xr.tableHeightOffsetM`。

---

## File Structure

- `web/src/xr/controllerInput.ts`：读取左右手位姿、右手安全按钮和左摇杆输入。
- `web/src/xr/session.ts`：保持右手安全逻辑，并把双手/头显/摇杆显示样本转交给渲染宿主。
- `web/src/scenes/controllerHints.ts`：新建并管理左右手柄提示的 Three.js 资源。
- `web/src/scenes/tableHeightController.ts`：纯计算高度基准、手动偏移、死区门禁、复位边沿与持久化。
- `web/src/scenes/simulationScene.ts`：建立 `robotVisualRoot`，接入提示与高度控制，维护 XR/桌面生命周期。
- `web/src/scenes/vrSafetyPanel.ts`：页脚增加左摇杆高度提示。
- 对应测试文件验证输入、会话、纯高度逻辑、资源释放和场景生命周期。

### Task 1: Dual-Controller Input Contract

**Files:**
- Modify: `web/src/xr/controllerInput.ts`
- Test: `web/tests/controllerInput.test.ts`

**Interfaces:**
- Consumes: `XRFrame`, `XRReferenceSpace`, `XRInputSource[]`。
- Produces: `ControllerPairSample`，其中 `right` 保持现有机器人控制字段，`left` 只含显示位姿与摇杆。

- [ ] **Step 1: Add failing left/right input tests**

Add a left Quest source with button index `3` and axes `2/3`, then assert:

```typescript
const sample = readControllers(frame(), {} as XRReferenceSpace, [
  leftQuestSource({stickY: -0.75, stickPressed: true}),
  questSource({a: true}),
]);

expect(sample.left).toMatchObject({
  trackingValid: true,
  thumbstickY: -0.75,
  thumbstickPressed: true,
});
expect(sample.right).toMatchObject({
  trackingValid: true,
  armButton: true,
});
```

Also test that a missing left pose returns a tracking-invalid left sample without invalidating the right sample, and that no left field appears in a `VRFrame` created from `sample.right`.

- [ ] **Step 2: Run the focused test and verify RED**

Run: `cd web; npm test -- controllerInput.test.ts`

Expected: FAIL because `readControllers` and left samples do not exist.

- [ ] **Step 3: Implement the dual-controller types and reader**

Keep `ControllerSample` for the right side and add:

```typescript
export interface TrackedPoseSample {
  p: Vec3;
  q: Quat;
  trackingValid: boolean;
}

export interface LeftControllerSample extends TrackedPoseSample {
  thumbstickY: number;
  thumbstickPressed: boolean;
}

export interface ControllerPairSample {
  left: LeftControllerSample;
  right: ControllerSample;
}
```

Use constants:

```typescript
const THUMBSTICK_BUTTON_INDEX = 3;
const THUMBSTICK_Y_AXIS_INDEX = 3;
```

Implement `readControllers` by locating each handedness independently. Reuse a private `readPose` helper, clamp finite axis values to `[-1, 1]`, and return tracking-invalid identity samples when a source or pose is unavailable. Preserve `readRightController` as a compatibility wrapper with its current contract: return `null` when no right input source exists, otherwise return the right sample (including a tracking-invalid sample when its `gripSpace` or pose is unavailable). This keeps all existing tests valid while `XRSessionController` migrates to `readControllers`.

- [ ] **Step 4: Run controller input tests**

Run: `cd web; npm test -- controllerInput.test.ts`

Expected: all tests PASS, including existing A/B mappings.

- [ ] **Step 5: Commit the input contract**

```bash
git add web/src/xr/controllerInput.ts web/tests/controllerInput.test.ts
git commit -m "feat: read both XR controllers"
```

### Task 2: Session-to-Scene XR Presentation Samples

**Files:**
- Modify: `web/src/xr/session.ts`
- Test: `web/tests/xrSession.test.ts`

**Interfaces:**
- Consumes: `ControllerPairSample` from Task 1 and `XRFrame.getViewerPose(referenceSpace)`.
- Produces: `XRPresentationSample` through `XRRenderHost.updateXRPresentation(sample, nowMs)`.

- [ ] **Step 1: Add the presentation interface and failing session tests**

Define the expected interface in the test import path:

```typescript
export interface XRPresentationSample {
  left: LeftControllerSample;
  right: ControllerSample;
  headY: number | null;
}

export interface XRRenderHost {
  startXR(session: XRSession, loop: XRFrameRequestCallback): Promise<void>;
  stopXR(): Promise<void>;
  updateXRPresentation(sample: XRPresentationSample, nowMs: number): void;
  renderXR(nowMs: number): void;
}
```

Extend `FakeHost` with `updateXRPresentation = vi.fn()`. Add a test with distinct left/right poses and a viewer height of `1.68`, asserting the host receives both hands and `headY: 1.68` before `renderXR`.

- [ ] **Step 2: Run session tests and verify RED**

Run: `cd web; npm test -- xrSession.test.ts`

Expected: FAIL because the host has no presentation callback.

- [ ] **Step 3: Read both controllers and viewer height in the XR loop**

In `onXRFrame`, replace `readRightController` with `readControllers`, keep all disarm/A/B/frame logic driven exclusively by `pair.right`, and call:

```typescript
const viewerPose = frame.getViewerPose(context.referenceSpace);
this.options.host.updateXRPresentation(
  {
    left: pair.left,
    right: pair.right,
    headY: viewerPose?.transform.position.y ?? null,
  },
  nowMs,
);
this.options.host.renderXR(nowMs);
```

Call `updateXRPresentation` on every live visible render frame, even when the 60 Hz network throttle skips `VRFrame`; this keeps hand visuals at headset refresh rate. On suspension, tracking loss, session end, and dispose, send identity tracking-invalid samples before teardown so hints hide synchronously.

- [ ] **Step 4: Verify right-hand transport remains unchanged**

Add an assertion:

```typescript
expect(frames.at(-1)).toMatchObject({
  right: {p: [0.4, 0.5, 0.6], grip: false, trigger: 0.25},
});
expect(frames.at(-1)).not.toHaveProperty('left');
```

Run: `cd web; npm test -- xrSession.test.ts controllerInput.test.ts`

Expected: all selected tests PASS.

- [ ] **Step 5: Commit session presentation plumbing**

```bash
git add web/src/xr/session.ts web/tests/xrSession.test.ts
git commit -m "feat: forward XR presentation poses"
```

### Task 3: Procedural Left and Right Controller Hints

**Files:**
- Create: `web/src/scenes/controllerHints.ts`
- Create: `web/tests/controllerHints.test.ts`

**Interfaces:**
- Consumes: `TrackedPoseSample` for left and right.
- Produces: `ControllerHints.update(left, right)`, `setVisible(boolean)`, and `dispose()`.

- [ ] **Step 1: Write failing resource and visibility tests**

Create a jsdom test that instantiates the class under a `THREE.Group`, then asserts:

```typescript
const parent = new THREE.Group();
const hints = new ControllerHints(parent);
hints.update(validPose([-0.2, 1.1, -0.4]), validPose([0.2, 1.1, -0.4]));

expect(hints.left.visible).toBe(true);
expect(hints.right.visible).toBe(true);
expect(hints.left.position.toArray()).toEqual([-0.2, 1.1, -0.4]);
expect(hints.left.userData.handedness).toBe('left');
expect(hints.right.userData.handedness).toBe('right');

hints.update(invalidPose(), validPose([0.3, 1.0, -0.5]));
expect(hints.left.visible).toBe(false);
expect(hints.right.visible).toBe(true);
```

Spy on every owned geometry/material/texture `dispose` and assert one disposal plus removal from parent.

- [ ] **Step 2: Run the new test and verify RED**

Run: `cd web; npm test -- controllerHints.test.ts`

Expected: FAIL because the module does not exist.

- [ ] **Step 3: Implement lightweight procedural hints**

Create one `THREE.Group` per hand. Each contains a rounded-looking grip assembled from `CapsuleGeometry` or cylinder/sphere primitives, a short `ArrowHelper` pointing along local `-Z`, and a canvas-texture sprite with `L` or `R`. Use `0x75648f` for left and `0x32d7ff` for right. Set:

```typescript
left.userData.handedness = 'left';
right.userData.handedness = 'right';
```

`updateHand(group, sample)` must hide invalid samples; valid samples copy position/quaternion exactly. `setVisible(false)` hides both without destroying their last pose. `dispose()` removes the root and disposes every unique geometry, material, and canvas texture once.

- [ ] **Step 4: Run hint tests**

Run: `cd web; npm test -- controllerHints.test.ts`

Expected: all tests PASS.

- [ ] **Step 5: Commit the visual component**

```bash
git add web/src/scenes/controllerHints.ts web/tests/controllerHints.test.ts
git commit -m "feat: render XR controller hints"
```

### Task 4: Pure Table Height Controller

**Files:**
- Create: `web/src/scenes/tableHeightController.ts`
- Create: `web/tests/tableHeightController.test.ts`

**Interfaces:**
- Consumes: `{headY, axisY, resetPressed, enabled, nowMs}`.
- Produces: `heightM`, `manualOffsetM`, `beginSession()`, `endSession()`, and deterministic persistence through a `Storage` dependency.

- [ ] **Step 1: Write failing calculation and gate tests**

Cover these exact cases:

```typescript
const control = new TableHeightController(storage);
control.beginSession();
expect(control.update({headY: 1.67, axisY: 0, resetPressed: false, enabled: true, nowMs: 100}))
  .toBeCloseTo(0.95);

for (let step = 1; step <= 10; step += 1) {
  control.update({
    headY: 1.67,
    axisY: -1,
    resetPressed: false,
    enabled: true,
    nowMs: 100 + step * 100,
  });
}
expect(control.heightM).toBeCloseTo(1.30); // 10 × 0.1 s × 0.35 m/s
```

Also assert head baselines clamp to 0.55/1.10, dead-zone inputs do nothing, manual offset clamps to ±0.6, final height clamps to 0.30/1.40, disabled input requires a neutral sample before resuming, joystick press resets manual offset to zero on one released edge, invalid stored values become zero, and `endSession()` returns height zero.

- [ ] **Step 2: Run the pure test and verify RED**

Run: `cd web; npm test -- tableHeightController.test.ts`

Expected: FAIL because the module does not exist.

- [ ] **Step 3: Implement deterministic height state**

Use exported constants and this update contract:

```typescript
export const TABLE_HEIGHT_STORAGE_KEY = 'vr4arm.xr.tableHeightOffsetM';

export interface TableHeightInput {
  headY: number | null;
  axisY: number;
  resetPressed: boolean;
  enabled: boolean;
  nowMs: number;
}
```

On the first finite `headY`, calculate `baseHeightM = clamp(headY - 0.72, 0.55, 1.10)`. Integrate `-axisY * 0.35 * dtSeconds`, with `dtSeconds` clamped to `[0, 0.1]` to avoid jumps after frame pauses. When disabled, set `neutralRequired=true`; clear it only after `abs(axisY) <= 0.2`. Reset uses a release-seen rising-edge latch. Persist only a finite clamped manual offset after it changes. `endSession` clears baseline/timestamps/latches and returns `0` without deleting the saved offset.

- [ ] **Step 4: Run height tests**

Run: `cd web; npm test -- tableHeightController.test.ts`

Expected: all tests PASS.

- [ ] **Step 5: Commit the height controller**

```bash
git add web/src/scenes/tableHeightController.ts web/tests/tableHeightController.test.ts
git commit -m "feat: calculate safe XR table height"
```

### Task 5: Scene Root and XR Lifecycle Integration

**Files:**
- Modify: `web/src/scenes/simulationScene.ts`
- Modify: `web/src/scenes/vrSafetyPanel.ts`
- Test: `web/tests/simulationScene.test.ts`
- Test: `web/tests/vrSafetyPanel.test.ts`

**Interfaces:**
- Consumes: `ControllerHints`, `TableHeightController`, and `XRPresentationSample`.
- Produces: `SimulationScene.updateXRPresentation(sample, nowMs)` implementing the extended `XRRenderHost`.

- [ ] **Step 1: Add failing scene ownership tests**

Add tests asserting:

```typescript
expect(scene.robotVisualRoot.parent).toBe(scene.scene);
expect(scene.targetMarker.parent).toBe(scene.robotVisualRoot);
expect(scene.grid.parent).toBe(scene.scene);
```

Use a prototype-call fixture to verify `updateXRPresentation` forwards both hands and sets root Y from the height controller only when:

```typescript
const adjustable = ['locked', 'stopped'].includes(scene.armSafetyState.phase)
  && !scene.armSafetyState.pending
  && !sample.right.grip;
```

Extend `stopXR` tests to expect root Y `0` and controller hints hidden even if `renderer.xr.setSession(null)` rejects.

- [ ] **Step 2: Run scene tests and verify RED**

Run: `cd web; npm test -- simulationScene.test.ts vrSafetyPanel.test.ts`

Expected: FAIL because the visual root and presentation method do not exist.

- [ ] **Step 3: Build `robotVisualRoot` without changing desktop geometry**

Add fields:

```typescript
private readonly robotVisualRoot = new THREE.Group();
private readonly controllerHints: ControllerHints;
private readonly tableHeight = new TableHeightController(window.localStorage);
```

Add `robotVisualRoot` to `scene` before environment construction. Keep lights and `GridHelper` directly on `scene`; add the table top, legs, workspace box, loaded robot group, target marker, and `VrSafetyPanel` to `robotVisualRoot`. With root Y `0`, desktop positions must remain byte-for-byte equivalent to current behavior.

- [ ] **Step 4: Integrate XR presentation updates and lifecycle cleanup**

Implement:

```typescript
updateXRPresentation(sample: XRPresentationSample, nowMs: number): void {
  this.controllerHints.update(sample.left, sample.right);
  const enabled = (this.armSafetyState.phase === 'locked' || this.armSafetyState.phase === 'stopped')
    && !this.armSafetyState.pending
    && !sample.right.grip;
  this.robotVisualRoot.position.y = this.tableHeight.update({
    headY: sample.headY,
    axisY: sample.left.thumbstickY,
    resetPressed: sample.left.thumbstickPressed,
    enabled: enabled && sample.left.trackingValid,
    nowMs,
  });
}
```

Call `tableHeight.beginSession()` and show hints only after XR session installation succeeds. In the `stopXR` `finally`, call `tableHeight.endSession()`, set root Y to `0`, and hide hints. In `dispose`, dispose hints before generic scene traversal so owned resources are not double-disposed.

- [ ] **Step 5: Update the VR footer copy**

Change the presentation footer type and value to:

```typescript
'A 解锁 · B 停止 · Grip 移动 · 左摇杆调高度'
```

The panel instruction remains status-specific; the footer is only a compact reminder.

- [ ] **Step 6: Run scene, session, and resource tests**

Run: `cd web; npm test -- simulationScene.test.ts controllerHints.test.ts tableHeightController.test.ts vrSafetyPanel.test.ts xrSession.test.ts`

Expected: all selected tests PASS.

- [ ] **Step 7: Build the frontend**

Run: `cd web; npm run build`

Expected: TypeScript and Vite build PASS with no new dependency.

- [ ] **Step 8: Commit scene integration**

```bash
git add web/src/scenes/simulationScene.ts web/src/scenes/vrSafetyPanel.ts web/tests/simulationScene.test.ts web/tests/vrSafetyPanel.test.ts
git commit -m "feat: add XR hand hints and table height"
```

### Task 6: Quest 3 Acceptance Gate

**Files:**
- Verify only; do not change production code unless an automated test first reproduces a defect.

**Interfaces:**
- Consumes: Tasks 1–5.
- Produces: Quest-tested XR visualization and observation-height behavior.

- [ ] **Step 1: Run all frontend tests and build**

Run: `cd web; npm test`

Expected: full Vitest suite PASS.

Run: `cd web; npm run build`

Expected: production build PASS.

- [ ] **Step 2: Verify both hands in Quest 3**

Enter VR with both Touch Plus controllers tracked. Confirm left is purple-gray with `L`, right is cyan with `R`, direction axes match physical pointing direction, and moving either physical controller updates only its matching hint.

Expected: losing tracking for one hand hides only that hint; the other remains visible.

- [ ] **Step 3: Verify height in standing and seated sessions**

In a standing session, confirm the automatic table top is about `0.72 m` below eye height. Exit, sit down, re-enter, and confirm a new baseline is computed. Move the left stick up/down and verify smooth motion; press the stick to return to the session baseline.

Expected: manual offset persists between sessions, while the automatic baseline follows current head height.

- [ ] **Step 4: Verify safety gates and coordinate separation**

While armed or holding right Grip, move/click the left stick and confirm the scene height does not change. Disarm, return the stick to neutral, then adjust height. Re-arm and move the right controller.

Expected: robot target motion remains continuous and identical regardless of visual table height; no height value appears in WebSocket `vr_frame` payloads.
