# Quest 3 In-VR Safety Controls Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the simulator safely operable after entering Quest 3 immersive VR by mapping right-controller A to explicit arm, B to explicit disarm, and showing a readable world-space safety status panel.

**Architecture:** `ArmPanel` remains the single command gate for desktop and XR arm/disarm requests and publishes an immutable safety snapshot. The pure controller reader exposes Quest face-button state, while `XRSessionController` owns per-session edge latches and calls the same `ArmPanel` methods only after publishing the current released-Grip frame. A focused Three.js `VrSafetyPanel` renders snapshot state as text plus shape inside the immersive scene without adding DOM Overlay or ray interaction.

**Tech Stack:** TypeScript 5.9, Three.js 0.181, WebXR Gamepads, Vite 7, Vitest 4, jsdom.

## Global Constraints

- Target Meta Quest 3 original Touch Plus right controller only; recognize Meta/Oculus Touch profiles and use WebXR button indices `4=A`, `5=B` only when at least six buttons are exposed.
- A sends one explicit `arm_request` only on a released-to-pressed edge after the same XR tracking epoch has observed A released, tracking valid, Grip released, Socket connected, and no request pending.
- B sends one explicit `disarm` on a released-to-pressed edge and resets the local command gate synchronously.
- A/B held through tracking loss, visibility loss, session restart, or reconnect never generate an action after recovery; release then re-press is mandatory.
- Grip remains the motion clutch; Trigger remains normalized gripper closure (`0=open`, `1=closed`).
- Tracking loss, hidden/blurred session, Socket close, XR end, and dispose keep the Task 12 immediate disarm/lock behavior.
- The world-space panel uses Chinese text plus a shape/accent; color alone is never the only state signal.
- Do not add DOM Overlay, controller rays, 3D buttons, passthrough/MR, camera, Cannon-es, dataset writing, real robot SDK/import/IP, or new npm dependencies.
- Keep the existing accepted desktop layout unchanged.
- Use TDD for each behavior task and commit after every task.

---

### Task 1: Unify Desktop and XR Arm/Disarm Through ArmPanel

**Files:**
- Modify: `web/src/ui/armPanel.ts`
- Modify: `web/tests/armPanel.test.ts`

**Interfaces:**
- Produces: `ControlSource = 'desktop' | 'xr'`
- Produces: `ArmSafetyPhase = 'disconnected' | 'fault' | 'locked' | 'pending' | 'armed' | 'active' | 'stopped'`
- Produces: `ArmSafetySnapshot`
- Produces: `ArmPanel.requestArm(source?: ControlSource) -> boolean`
- Produces: `ArmPanel.requestDisarm(source?: ControlSource) -> void`
- Produces: `ArmPanel.safetyState -> ArmSafetySnapshot`
- Constructor adds optional `onSafetyChange(snapshot)` callback after the existing `onEnterVR` argument.

- [ ] **Step 1: Add failing public-gate and snapshot tests**

Append tests that call the public methods directly rather than clicking buttons:

```ts
it('uses one safety gate for desktop and XR arm requests', () => {
  const send = vi.fn();
  const states = vi.fn();
  const panel = new ArmPanel(document.querySelector('#panel')!, send, undefined, states);

  expect(panel.requestArm('xr')).toBe(false);
  panel.observeGrip(false);
  expect(panel.requestArm('xr')).toBe(true);
  expect(send).toHaveBeenCalledWith(expect.objectContaining({
    type: 'arm_request',
    request_id: expect.stringMatching(/^xr-arm_request-/),
  }));
  expect(panel.requestArm('xr')).toBe(false);
  expect(panel.safetyState.phase).toBe('pending');
});

it('XR disarm closes the local gate before transport returns', () => {
  const events: string[] = [];
  let panel!: ArmPanel;
  panel = new ArmPanel(
    document.querySelector('#panel')!,
    () => events.push(`send:${panel.safetyState.phase}`),
  );
  panel.observeGrip(false);
  panel.requestArm('xr');
  panel.setMode('ARMED');

  panel.requestDisarm('xr');

  expect(events.at(-1)).toBe('send:stopped');
  expect(panel.safetyState).toMatchObject({phase: 'stopped', armed: false, eligible: false});
});

it('publishes immutable readable safety snapshots', () => {
  const snapshots: unknown[] = [];
  const panel = new ArmPanel(
    document.querySelector('#panel')!,
    vi.fn(),
    undefined,
    (snapshot) => snapshots.push(snapshot),
  );
  panel.setConnected(false);
  expect(panel.safetyState.phase).toBe('disconnected');
  panel.setConnected(true);
  panel.setFault('ik_unreachable');
  expect(panel.safetyState.phase).toBe('fault');
  expect(snapshots.length).toBeGreaterThan(0);
  expect(snapshots.at(-1)).not.toBe(panel.safetyState);
});
```

- [ ] **Step 2: Run focused tests and verify RED**

Run:

```powershell
cd D:\MyWork\VR4Arm\.worktrees\vr-lm3-simulator-m1\web
npm.cmd test -- armPanel.test.ts
```

Expected: FAIL because `requestArm`, `requestDisarm`, `safetyState`, and the fourth constructor argument are not public/defined.

- [ ] **Step 3: Implement the single ArmPanel command gate**

Add these exported types and state:

```ts
export type ControlSource = 'desktop' | 'xr';
export type ArmSafetyPhase =
  | 'disconnected'
  | 'fault'
  | 'locked'
  | 'pending'
  | 'armed'
  | 'active'
  | 'stopped';

export type ArmSafetySnapshot = Readonly<{
  phase: ArmSafetyPhase;
  connected: boolean;
  eligible: boolean;
  armed: boolean;
  pending: boolean;
  mode: TeleopMode;
  fault: string | null;
}>;
```

Track `mode: TeleopMode = 'DISCONNECTED'` and `stopRequested = false`. Extend the constructor with:

```ts
private readonly onSafetyChange: (snapshot: ArmSafetySnapshot) => void = () => {},
```

Make both button listeners call the public source-aware methods:

```ts
this.armButton.addEventListener('click', () => this.requestArm('desktop'));
this.stopButton.addEventListener('click', () => this.requestDisarm('desktop'));
```

Expose a new object on every read:

```ts
get safetyState(): ArmSafetySnapshot {
  return Object.freeze({
    phase: this.safetyPhase(),
    connected: this.connected,
    eligible: this.eligible,
    armed: this.armed,
    pending: this.isArmPending,
    mode: this.mode,
    fault: this.fault,
  });
}
```

Replace the private request/stop methods with:

```ts
requestArm(source: ControlSource = 'desktop'): boolean {
  if (!this.connected || !this.eligible || this.fault || this.armed || this.isArmPending) {
    return false;
  }
  const message = this.control('arm_request', source);
  this.pendingArmRequestId = message.request_id;
  this.eligible = false;
  this.stopRequested = false;
  this.armFeedback = null;
  this.syncButtonState();
  this.sendControl(message);
  return true;
}

requestDisarm(source: ControlSource = 'desktop'): void {
  this.armed = false;
  this.eligible = false;
  this.pendingArmRequestId = null;
  this.awaitingArmedMode = false;
  this.armFeedback = null;
  this.stopRequested = true;
  this.syncButtonState();
  this.sendControl(this.control('disarm', source));
}

private control(type: 'arm_request' | 'disarm', source: ControlSource): ClientControlMessage {
  this.requestSequence += 1;
  return {v: PROTOCOL_VERSION, type, request_id: `${source}-${type}-${this.requestSequence}`};
}
```

`setMode()` must assign `this.mode = mode`; authoritative ACTIVE maps to `active`, ARMED/HOLD maps to `armed`, and READY clears `stopRequested`. `resetToLocked()` clears `stopRequested`. Add:

```ts
private safetyPhase(): ArmSafetyPhase {
  if (!this.connected) return 'disconnected';
  if (this.fault) return 'fault';
  if (this.stopRequested || this.mode === 'DISARMED') return 'stopped';
  if (this.mode === 'ACTIVE') return 'active';
  if (this.isArmPending) return 'pending';
  if (this.armed) return 'armed';
  return 'locked';
}
```

At the end of `syncButtonState()`, call `this.onSafetyChange(this.safetyState)`. Do not expose the mutable internal object.

- [ ] **Step 4: Run focused and full frontend tests**

Run:

```powershell
npm.cmd test -- armPanel.test.ts
npm.cmd test
npm.cmd run build
```

Expected: all tests PASS and Vite build succeeds.

- [ ] **Step 5: Commit Task 1**

```powershell
git add web/src/ui/armPanel.ts web/tests/armPanel.test.ts
git commit -m "refactor: share simulator arm safety gate"
```

---

### Task 2: Read Quest A/B and Apply Per-Session Edge Safety

**Files:**
- Modify: `web/src/xr/controllerInput.ts`
- Modify: `web/src/xr/session.ts`
- Modify: `web/tests/controllerInput.test.ts`
- Modify: `web/tests/xrSession.test.ts`

**Interfaces:**
- Extends `ControllerSample` with `armButton`, `stopButton`, `questFaceButtonsSupported`.
- Extends `XRSessionControllerOptions` with `onArmRequest()`, `onStopRequest()`, and `onControllerSupport(supported: boolean | null)`.
- Produces per-session face-button latches; no global/static input state.

- [ ] **Step 1: Add failing Quest profile/button tests**

Build right-hand sources with `profiles: ['meta-quest-touch-plus']` or `['oculus-touch-v3']` and seven `GamepadButton` values. Add:

```ts
it('reads Quest right-controller A and B from indices 4 and 5', () => {
  const sample = readRightController(
    frame(),
    {} as XRReferenceSpace,
    [questSource({a: true, b: false})],
  );
  expect(sample).toMatchObject({
    armButton: true,
    stopButton: false,
    questFaceButtonsSupported: true,
  });
});

it.each([
  ['unknown profile', unknownProfileSource()],
  ['short button array', questSource({buttonCount: 5})],
])('keeps face actions disabled for %s', (_name, source) => {
  const sample = readRightController(frame(), {} as XRReferenceSpace, [source]);
  expect(sample).toMatchObject({
    armButton: false,
    stopButton: false,
    questFaceButtonsSupported: false,
  });
});
```

- [ ] **Step 2: Add failing session edge/order tests**

Extend the XR setup callbacks with an ordered event log. Add tests proving:

```ts
it('publishes released Grip before one A-edge arm request and never repeats while held', async () => {
  const events: string[] = [];
  const {controller, host} = setup({events});
  await controller.enterVR();

  host.loop?.(100, questPoseFrame({grip: false, a: false}));
  host.loop?.(120, questPoseFrame({grip: false, a: true}));
  host.loop?.(140, questPoseFrame({grip: false, a: true}));

  expect(events.filter((event) => event === 'arm')).toHaveLength(1);
  expect(events.indexOf('controller:released')).toBeLessThan(events.indexOf('frame'));
  expect(events.indexOf('frame')).toBeLessThan(events.indexOf('arm'));
});

it('requires A release after tracking loss before another arm edge', async () => {
  const {controller, host, armRequests} = setup();
  await controller.enterVR();
  host.loop?.(100, questPoseFrame({a: false}));
  host.loop?.(120, questPoseFrame({a: true}));
  host.loop?.(140, missingPoseFrame({a: true}));
  host.loop?.(160, questPoseFrame({a: true}));
  expect(armRequests).toHaveLength(1);
  host.loop?.(180, questPoseFrame({a: false}));
  host.loop?.(200, questPoseFrame({a: true}));
  expect(armRequests).toHaveLength(2);
});

it('B edge stops once and wins when A and B are pressed together', async () => {
  const {controller, host, armRequests, stopRequests} = setup();
  await controller.enterVR();
  host.loop?.(100, questPoseFrame({a: false, b: false}));
  host.loop?.(120, questPoseFrame({a: true, b: true}));
  host.loop?.(140, questPoseFrame({a: true, b: true}));
  expect(stopRequests).toHaveLength(1);
  expect(armRequests).toHaveLength(0);
});
```

- [ ] **Step 3: Run focused tests and verify RED**

Run:

```powershell
npm.cmd test -- controllerInput.test.ts xrSession.test.ts
```

Expected: FAIL because the sample fields and callbacks do not exist.

- [ ] **Step 4: Implement Quest profile-aware face button reading**

In `controllerInput.ts` add:

```ts
const ARM_BUTTON_INDEX = 4;
const STOP_BUTTON_INDEX = 5;
const QUEST_PROFILE_PREFIXES = ['meta-quest-touch', 'oculus-touch'] as const;

function pressed(button: GamepadButton | undefined): boolean {
  return button?.pressed === true || normalizeButton(button?.value) >= 0.5;
}

function supportsQuestFaceButtons(source: XRInputSource, buttons: readonly GamepadButton[]): boolean {
  return buttons.length > STOP_BUTTON_INDEX && source.profiles.some((profile) =>
    QUEST_PROFILE_PREFIXES.some((prefix) => profile.startsWith(prefix)),
  );
}
```

Extend every valid and invalid sample with:

```ts
armButton: supported && pressed(buttons[ARM_BUTTON_INDEX]),
stopButton: supported && pressed(buttons[STOP_BUTTON_INDEX]),
questFaceButtonsSupported: supported,
```

If pose tracking is valid but `gamepad.buttons` is absent, keep the pose sample tracking-valid with neutral Grip/Trigger and `questFaceButtonsSupported=false`; this prevents a false A/B capability while preserving the distinction between pose loss and controller-profile mismatch. Missing `gripSpace` or pose remains tracking-invalid. Invalid samples use `armButton=false` and `stopButton=false`.

- [ ] **Step 5: Implement per-session latches and strict callback order**

Add to `SessionContext`:

```ts
armLatch: {pressed: boolean; releaseSeen: boolean};
stopLatch: {pressed: boolean; releaseSeen: boolean};
```

Initialize both to `{pressed: false, releaseSeen: false}`. Add:

```ts
function takeReleasedEdge(
  latch: {pressed: boolean; releaseSeen: boolean},
  pressedNow: boolean,
): boolean {
  const rising = latch.releaseSeen && !latch.pressed && pressedNow;
  if (!pressedNow) latch.releaseSeen = true;
  latch.pressed = pressedNow;
  return rising;
}

function resetFaceLatches(context: SessionContext): void {
  context.armLatch = {pressed: false, releaseSeen: false};
  context.stopLatch = {pressed: false, releaseSeen: false};
}
```

Extend options:

```ts
onArmRequest(): void;
onStopRequest(): void;
onControllerSupport(supported: boolean | null): void;
```

Inside `emitFrame()`, preserve this exact order:

```ts
this.options.onController({tracking: sample.trackingValid, grip: sample.grip, trigger: sample.trigger});
this.options.onControllerSupport(sample.trackingValid ? sample.questFaceButtonsSupported : null);
this.options.onFrame(createVRFrame(/* existing fields */));

if (!sample.trackingValid || !sample.questFaceButtonsSupported) {
  resetFaceLatches(context);
  return;
}
const stopEdge = takeReleasedEdge(context.stopLatch, sample.stopButton);
const armEdge = takeReleasedEdge(context.armLatch, sample.armButton);
if (stopEdge) this.options.onStopRequest();
else if (armEdge && !sample.grip) this.options.onArmRequest();
```

Every lifecycle safety reset calls `resetFaceLatches(context)` and `onControllerSupport(null)`. This prevents held buttons from turning into an action after recovery.

- [ ] **Step 6: Run focused/full tests and build**

```powershell
npm.cmd test -- controllerInput.test.ts xrSession.test.ts
npm.cmd test
npm.cmd run build
```

Expected: all tests PASS; existing lifecycle race tests remain green.

- [ ] **Step 7: Commit Task 2**

```powershell
git add web/src/xr/controllerInput.ts web/src/xr/session.ts web/tests/controllerInput.test.ts web/tests/xrSession.test.ts
git commit -m "feat: add Quest face-button safety edges"
```

---

### Task 3: Render and Wire the World-Space Safety Panel

**Files:**
- Create: `web/src/scenes/vrSafetyPanel.ts`
- Create: `web/tests/vrSafetyPanel.test.ts`
- Modify: `web/src/scenes/simulationScene.ts`
- Modify: `web/src/main.ts`
- Modify: `web/tests/simulationScene.test.ts`

**Interfaces:**
- Consumes: `ArmSafetySnapshot` from Task 1.
- Produces: `describeVrSafety(snapshot, controllerSupported) -> VrSafetyPresentation`
- Produces: `VrSafetyPanel.update(snapshot, controllerSupported)`, `setVisible()`, `dispose()`.
- Extends `SimulationScene` with `setArmSafetyState(snapshot)` and `setQuestControllerSupport(supported)`.

- [ ] **Step 1: Add failing presentation mapping tests**

Create `web/tests/vrSafetyPanel.test.ts` with exact Chinese expectations:

```ts
import {describe, expect, it} from 'vitest';
import {describeVrSafety} from '../src/scenes/vrSafetyPanel';
import type {ArmSafetyPhase, ArmSafetySnapshot} from '../src/ui/armPanel';

const state = (phase: ArmSafetyPhase): ArmSafetySnapshot => ({
  phase,
  connected: phase !== 'disconnected',
  eligible: false,
  armed: phase === 'armed' || phase === 'active',
  pending: phase === 'pending',
  mode: phase === 'active' ? 'ACTIVE' : 'READY',
  fault: phase === 'fault' ? 'ik_unreachable' : null,
} as const);

describe('VR safety presentation', () => {
  it.each([
    ['locked', '未解锁', '松开 Grip 后按 A'],
    ['pending', '解锁中', '等待仿真确认'],
    ['armed', '已解锁', '按住 Grip 移动'],
    ['active', '运动中', '松开 Grip 停止'],
    ['stopped', '已停止', '松开 Grip 后按 A'],
    ['fault', '故障/失联', '保持 Grip 松开'],
    ['disconnected', '故障/失联', '保持 Grip 松开'],
  ])('maps %s to readable text', (phase, title, instruction) => {
    expect(describeVrSafety(state(phase), true)).toMatchObject({title, instruction});
  });

  it('blocks with a readable unsupported-controller message', () => {
    expect(describeVrSafety(state('locked'), false)).toMatchObject({
      title: '手柄不受支持',
      instruction: '当前配置不支持 A/B 安全控制',
    });
  });
});
```

- [ ] **Step 2: Run the new test and verify RED**

```powershell
npm.cmd test -- vrSafetyPanel.test.ts
```

Expected: FAIL because `vrSafetyPanel.ts` does not exist.

- [ ] **Step 3: Implement presentation mapping and Three.js panel**

Create `vrSafetyPanel.ts` with:

```ts
import * as THREE from 'three';
import type {ArmSafetySnapshot} from '../ui/armPanel';

export interface VrSafetyPresentation {
  title: string;
  instruction: string;
  footer: 'A 解锁 · B 停止 · Grip 移动 · Trigger 夹爪';
  tone: 'cyan' | 'red' | 'muted';
  shape: 'shield' | 'stop' | 'warning';
}

export function describeVrSafety(
  state: ArmSafetySnapshot,
  controllerSupported: boolean | null,
): VrSafetyPresentation {
  if (controllerSupported === false) return presentation('手柄不受支持', '当前配置不支持 A/B 安全控制', 'red', 'warning');
  if (state.phase === 'disconnected' || state.phase === 'fault') return presentation('故障/失联', '保持 Grip 松开', 'red', 'warning');
  if (state.phase === 'pending') return presentation('解锁中', '等待仿真确认', 'cyan', 'shield');
  if (state.phase === 'active') return presentation('运动中', '松开 Grip 停止', 'cyan', 'shield');
  if (state.phase === 'armed') return presentation('已解锁', '按住 Grip 移动', 'cyan', 'shield');
  if (state.phase === 'stopped') return presentation('已停止', '松开 Grip 后按 A', 'red', 'stop');
  return presentation('未解锁', '松开 Grip 后按 A', 'muted', 'shield');
}

function presentation(
  title: string,
  instruction: string,
  tone: VrSafetyPresentation['tone'],
  shape: VrSafetyPresentation['shape'],
): VrSafetyPresentation {
  return {title, instruction, tone, shape, footer: 'A 解锁 · B 停止 · Grip 移动 · Trigger 夹爪'};
}
```

Implement `VrSafetyPanel` with a 1024×256 canvas, `THREE.CanvasTexture`, `THREE.SpriteMaterial({transparent: true, depthTest: false})`, and `THREE.Sprite`. Draw a solid graphite rectangle, a left shield/stop/warning shape, white title, muted instruction, and the fixed footer. Use explicit font strings with Chinese fallbacks; redraw only when the presentation changes:

```ts
export class VrSafetyPanel {
  readonly sprite: THREE.Sprite;
  private readonly canvas: HTMLCanvasElement;
  private readonly context: CanvasRenderingContext2D;
  private readonly texture: THREE.CanvasTexture;
  private readonly material: THREE.SpriteMaterial;
  private lastKey = '';

  constructor(parent: THREE.Object3D) {
    this.canvas = document.createElement('canvas');
    this.canvas.width = 1024;
    this.canvas.height = 256;
    const context = this.canvas.getContext('2d');
    if (!context) throw new Error('无法创建 VR 安全状态牌');
    this.context = context;
    this.texture = new THREE.CanvasTexture(this.canvas);
    this.texture.colorSpace = THREE.SRGBColorSpace;
    this.material = new THREE.SpriteMaterial({map: this.texture, transparent: true, depthTest: false});
    this.sprite = new THREE.Sprite(this.material);
    this.sprite.position.set(-0.82, 1.18, -0.58);
    this.sprite.scale.set(1.15, 0.2875, 1);
    this.sprite.renderOrder = 1000;
    this.sprite.visible = false;
    parent.add(this.sprite);
  }

  update(state: ArmSafetySnapshot, controllerSupported: boolean | null): void {
    const view = describeVrSafety(state, controllerSupported);
    const key = `${view.title}|${view.instruction}|${view.tone}|${view.shape}`;
    if (key === this.lastKey) return;
    this.lastKey = key;
    this.draw(view);
    this.texture.needsUpdate = true;
  }

  setVisible(visible: boolean): void {
    this.sprite.visible = visible;
  }

  dispose(): void {
    this.sprite.removeFromParent();
    this.texture.dispose();
    this.material.dispose();
  }

  private draw(view: VrSafetyPresentation): void {
    const ctx = this.context;
    const accent = view.tone === 'cyan' ? '#32d7ff' : view.tone === 'red' ? '#ff3347' : '#8ca5b6';
    ctx.clearRect(0, 0, 1024, 256);
    ctx.fillStyle = 'rgba(7, 19, 30, 0.96)';
    ctx.fillRect(0, 0, 1024, 256);
    ctx.strokeStyle = '#315064';
    ctx.lineWidth = 4;
    ctx.strokeRect(2, 2, 1020, 252);
    this.drawShape(view.shape, accent);
    ctx.fillStyle = '#ffffff';
    ctx.font = "700 58px 'Microsoft YaHei', 'Noto Sans SC', sans-serif";
    ctx.fillText(view.title, 150, 82);
    ctx.fillStyle = accent;
    ctx.font = "500 36px 'Microsoft YaHei', 'Noto Sans SC', sans-serif";
    ctx.fillText(view.instruction, 150, 137);
    ctx.strokeStyle = '#315064';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(40, 174);
    ctx.lineTo(984, 174);
    ctx.stroke();
    ctx.fillStyle = '#c9d8e2';
    ctx.font = "500 28px 'Microsoft YaHei', 'Noto Sans SC', sans-serif";
    ctx.fillText(view.footer, 52, 220);
  }

  private drawShape(shape: VrSafetyPresentation['shape'], accent: string): void {
    const ctx = this.context;
    ctx.save();
    ctx.translate(82, 82);
    ctx.strokeStyle = accent;
    ctx.fillStyle = accent;
    ctx.lineWidth = 8;
    if (shape === 'stop') {
      ctx.strokeRect(-32, -32, 64, 64);
    } else if (shape === 'warning') {
      ctx.beginPath();
      ctx.moveTo(0, -38);
      ctx.lineTo(40, 34);
      ctx.lineTo(-40, 34);
      ctx.closePath();
      ctx.stroke();
      ctx.fillRect(-4, -14, 8, 26);
      ctx.fillRect(-4, 20, 8, 8);
    } else {
      ctx.beginPath();
      ctx.moveTo(0, -40);
      ctx.lineTo(36, -25);
      ctx.lineTo(30, 18);
      ctx.quadraticCurveTo(0, 44, -30, 18);
      ctx.lineTo(-36, -25);
      ctx.closePath();
      ctx.stroke();
    }
    ctx.restore();
  }
}
```

`dispose()` must dispose the CanvasTexture and SpriteMaterial and remove the sprite from its parent.

- [ ] **Step 4: Wire the panel into SimulationScene**

Create the panel after the environment and add it to the scene. Add cached default snapshot and support state, plus:

```ts
setArmSafetyState(snapshot: ArmSafetySnapshot): void {
  this.armSafetyState = snapshot;
  this.vrSafetyPanel.update(snapshot, this.questControllerSupported);
}

setQuestControllerSupport(supported: boolean | null): void {
  this.questControllerSupported = supported;
  this.vrSafetyPanel.update(this.armSafetyState, supported);
}
```

In `startXR()`, set the panel visible only after `renderer.xr.setSession(session)` succeeds. In `stopXR()` finally and `dispose()`, hide/dispose it. Add focused tests that a mocked panel becomes visible on successful XR start, is hidden even when `setSession(null)` rejects, and is disposed with the scene.

- [ ] **Step 5: Wire main through the shared gate**

Construct `ArmPanel` with the fourth callback:

```ts
armPanel = new ArmPanel(
  hud.actionContainer,
  (message) => socket.sendControl(message),
  () => void (xrController.isActive ? xrController.exitVR() : xrController.enterVR()),
  (snapshot) => scene?.setArmSafetyState(snapshot),
);
```

After constructing `SimulationScene`, immediately call `scene.setArmSafetyState(armPanel.safetyState)`. Extend `XRSessionController` options:

```ts
onArmRequest: () => { armPanel.requestArm('xr'); },
onStopRequest: () => { armPanel.requestDisarm('xr'); },
onControllerSupport: (supported) => scene.setQuestControllerSupport(supported),
```

Keep lifecycle `onDisarm: sendVRDisarm` and `onLockReset` unchanged; A/B actions must not replace Task 12 safety stops.

- [ ] **Step 6: Run complete verification**

```powershell
cd D:\MyWork\VR4Arm\.worktrees\vr-lm3-simulator-m1\web
npm.cmd test
npm.cmd run build
cd ..
git diff --check
```

Expected: all Vitest tests pass, TypeScript/Vite build succeeds, and diff check is clean. The existing non-failing Three.js chunk advisory may remain.

- [ ] **Step 7: Commit Task 3**

```powershell
git add web/src/scenes/vrSafetyPanel.ts web/src/scenes/simulationScene.ts web/src/main.ts web/tests/vrSafetyPanel.test.ts web/tests/simulationScene.test.ts
git commit -m "feat: show in-VR simulator safety status"
```

---

## Extension Verification Gate

Before returning to Milestone 1 Task 14, run:

```powershell
cd D:\MyWork\VR4Arm\.worktrees\vr-lm3-simulator-m1\web
npm.cmd test
npm.cmd run build
cd ..
git status --short
```

Required evidence:

- All prior Task 11/12 frontend tests remain green.
- A/B edge tests pass for supported Quest profiles and remain locked for unsupported/malformed inputs.
- Desktop and XR actions share `ArmPanel`; no direct XR `arm_request` bypass exists.
- Held buttons never trigger after tracking/session recovery.
- The world-space panel uses exact Chinese status text and is hidden outside XR.
- No new dependencies or prohibited MR/real-robot features were added.
- Quest hardware acceptance remains explicitly pending until the actual headset checklist is executed.
