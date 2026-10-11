// @vitest-environment jsdom
// In-memory transport and synthetic XR only; no browser, headset or robot connection.
import {afterEach, expect, it, vi} from 'vitest';
import scenario from '../../schemas/fixtures/xr-grip-release-wire.json';
import robotFixture from '../../schemas/fixtures/robot-state-valid.json';
import {forwardTeleopFrame} from '../src/appFrameForwarding';
import type {RobotStateMessage} from '../src/protocol/messages';
import {TeleopSocket} from '../src/transport/teleopSocket';
import {ArmPanel} from '../src/ui/armPanel';
import {XRSessionController, type XRRenderHost} from '../src/xr/session';

class MemorySocket {
  readyState = 0;
  sent: string[] = [];
  onopen: (() => void) | null = null;
  onmessage: ((event: {data: string}) => void) | null = null;
  onclose: (() => void) | null = null;
  onerror: (() => void) | null = null;
  send(value: string): void { this.sent.push(value); }
  close(): void { this.readyState = 3; }
  open(): void { this.readyState = 1; this.onopen?.(); }
  receive(message: unknown): void { this.onmessage?.({data: JSON.stringify(message)}); }
}

function inputSource(input: {grip?: boolean; trigger?: number; a?: boolean; b?: boolean}): XRInputSource {
  const buttons = Array.from({length: 7}, () => ({value: 0, pressed: false}));
  buttons[0].value = input.trigger ?? 0;
  for (const [index, value] of [[1, input.grip], [4, input.a], [5, input.b]] as const) {
    buttons[index] = {value: value ? 1 : 0, pressed: value ?? false};
  }
  return {handedness: 'right', profiles: ['meta-quest-touch-plus'], gripSpace: {},
    gamepad: {buttons}} as unknown as XRInputSource;
}

function setup() {
  document.body.innerHTML = '<div id="panel"></div>';
  const wire = new MemorySocket();
  let panel: ArmPanel;
  const socket = new TeleopSocket('ws://synthetic.invalid', (state) => {
    panel.setRuntimeIdentity(state.backend ?? null);
    panel.setFault(state.fault ?? null);
    panel.setConstraint(state.constraint ?? null);
    panel.setRecoveryPhase(state.recovery_phase ?? null);
    panel.setMode(state.mode);
  }, () => wire as unknown as WebSocket,
  (status) => panel.setConnectionStatus(status), (ack) => panel.handleArmFeedback(ack));
  panel = new ArmPanel(document.querySelector('#panel')!, (message) => socket.sendControl(message));
  const session = Object.assign(new EventTarget(), {
    visibilityState: 'visible', inputSources: [] as XRInputSource[],
    requestReferenceSpace: async () => ({} as XRReferenceSpace),
    end: async () => { session.dispatchEvent(new Event('end')); },
  });
  let loop: XRFrameRequestCallback | null = null;
  const host: XRRenderHost = {
    startXR: async (_session, callback) => { loop = callback; },
    stopXR: async () => { loop = null; }, updateXRPresentation: () => {}, renderXR: () => {},
  };
  let now = 0;
  let disarm = 0;
  const controller = new XRSessionController({
    xr: {isSessionSupported: async () => true, requestSession: async () => session} as unknown as XRSystem,
    host, createSessionId: () => scenario.session_id, now: () => now,
    onFrame: (frame) => forwardTeleopFrame(frame, 'xr', false, (message) => socket.sendFrame(message)),
    onController: (state) => { if (state.tracking) panel.observeGrip(state.grip); },
    onArmRequest: () => { panel.requestArm('xr'); },
    onStopOrResetRequest: () => { panel.requestStopOrReset('xr'); },
    onDisarm: () => socket.sendControl({v: 1, type: 'disarm',
      request_id: `vr-disarm-${++disarm}`, client_mono_ms: now}),
    onLockReset: () => panel.resetToLocked(), onStatus: (status) => panel.setVRStatus(status),
  });
  function emit(at: number, input: Parameters<typeof inputSource>[0], tracking = true) {
    now = at;
    session.inputSources = [inputSource(input)];
    loop?.(at, {getPose: () => tracking ? {transform: {
      position: {x: 0.4, y: 0.5, z: 0.6}, orientation: {x: 0, y: 0, z: 0, w: 1},
    }} : null, getViewerPose: () => null} as unknown as XRFrame);
  }
  function feedback(mode: RobotStateMessage['mode'], fault: string | null = null) {
    wire.receive({...robotFixture, backend: 'LEBAI_FAKE', mode, fault});
  }
  return {wire, socket, panel, controller, emit, feedback, setNow: (at: number) => { now = at; },
    savedLoop: () => loop};
}

afterEach(() => vi.restoreAllMocks());

it('generates the shared wire fixture through XR, ArmPanel and TeleopSocket', async () => {
  const rig = setup();
  try {
    for (const step of scenario.steps) {
      const before = rig.wire.sent.length;
      rig.setNow(step.at_ms);
      if (step.name === 'connect') { rig.socket.connect(); rig.wire.open(); }
      else if (step.name === 'enter') await rig.controller.enterVR();
      else if (step.name === 'exit') {
        const stale = rig.savedLoop();
        await rig.controller.exitVR();
        stale?.(300, {} as XRFrame); // Revoked callback cannot even read this invalid frame.
      } else rig.emit(step.at_ms, step.input ?? {}, step.tracking !== false);
      expect(rig.wire.sent.slice(before).map((value) => JSON.parse(value)), step.name).toEqual(step.messages);
      if (step.name === 'arm') rig.wire.receive({v: 1, type: 'arm_ack', request_id: 'xr-arm_request-1'});
      if (step.mode) rig.feedback(step.mode as RobotStateMessage['mode']);
    }
    expect(rig.wire.sent.map((value) => JSON.parse(value).type).filter((type) => type === 'arm_request')).toHaveLength(1);
    expect(rig.wire.sent.some((value) => ['home_request', 'fault_reset'].includes(JSON.parse(value).type))).toBe(false);
  } finally { await rig.controller.dispose(); rig.socket.close(); }
});

it.each(['stop_unverified', 'stop_incomplete'])('keeps %s locked after feedback and transport reconnect', async (fault) => {
  const rig = setup();
  try {
    rig.socket.connect(); rig.wire.open();
    await rig.controller.enterVR();
    rig.feedback('FAULT', fault);
    rig.emit(100, {});
    const before = rig.wire.sent.length;
    rig.emit(120, {a: true});
    rig.emit(140, {});
    rig.emit(160, {b: true});
    expect(rig.wire.sent.slice(before).every((value) => JSON.parse(value).type === 'vr_frame')).toBe(true);
    rig.socket.close(); rig.socket.connect(); rig.wire.open();
    rig.emit(180, {});
    const reconnectBefore = rig.wire.sent.length;
    rig.emit(200, {a: true}); rig.emit(220, {}); rig.emit(240, {b: true});
    expect(rig.wire.sent.slice(reconnectBefore).every((value) => JSON.parse(value).type === 'vr_frame')).toBe(true);
    expect(rig.panel.armButton.disabled).toBe(true);
    expect(rig.panel.stopButton.disabled).toBe(true);
  } finally { await rig.controller.dispose(); rig.socket.close(); }
});
