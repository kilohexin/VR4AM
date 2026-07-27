import {beforeEach, describe, expect, it, vi} from 'vitest';
import type {VRFrame} from '../src/protocol/messages';
import {
  XRSessionController,
  type XRPresentationSample,
  type XRRenderHost,
  type XRSessionStatus,
} from '../src/xr/session';

class FakeSession extends EventTarget {
  visibilityState: XRVisibilityState = 'visible';
  inputSources: XRInputSource[] = [rightSource()];
  readonly requestReferenceSpace = vi.fn(async (_type: XRReferenceSpaceType) => (
    {} as XRReferenceSpace
  ));
  readonly end = vi.fn(async () => {
    this.dispatchEvent(new Event('end'));
  });
}

class FakeHost implements XRRenderHost {
  loop: XRFrameRequestCallback | null = null;
  readonly startXR = vi.fn(async (_session: XRSession, loop: XRFrameRequestCallback) => {
    this.loop = loop;
  });
  readonly stopXR = vi.fn(async () => {
    this.loop = null;
  });
  readonly updateXRPresentation = vi.fn((_sample: XRPresentationSample, _nowMs: number) => {
    this.events?.push('presentation');
  });
  readonly renderXR = vi.fn();

  constructor(private readonly events?: string[]) {
    this.renderXR.mockImplementation(() => this.events?.push('render'));
  }
}

function rightSource(grip = 0.6, trigger = 0.25): XRInputSource {
  return {
    handedness: 'right',
    profiles: ['generic-trigger-squeeze'],
    gripSpace: {handedness: 'right'},
    gamepad: {buttons: [{value: trigger}, {value: grip}]},
  } as unknown as XRInputSource;
}

function leftSource(thumbstickY = -0.75, thumbstickPressed = true): XRInputSource {
  const buttons = Array.from({length: 4}, () => ({value: 0, pressed: false}));
  buttons[3] = {value: thumbstickPressed ? 1 : 0, pressed: thumbstickPressed};
  return {
    handedness: 'left',
    profiles: ['meta-quest-touch-plus'],
    gripSpace: {handedness: 'left'},
    gamepad: {buttons, axes: [0, 0, 0, thumbstickY]},
  } as unknown as XRInputSource;
}

function questSource(options: {
  profile?: string;
  grip?: boolean;
  a?: boolean;
  b?: boolean;
} = {}): XRInputSource {
  const buttons = Array.from({length: 7}, () => ({value: 0, pressed: false}));
  buttons[0] = {value: 0.25, pressed: false};
  buttons[1] = {value: options.grip ? 1 : 0, pressed: options.grip ?? false};
  buttons[4] = {value: options.a ? 1 : 0, pressed: options.a ?? false};
  buttons[5] = {value: options.b ? 1 : 0, pressed: options.b ?? false};
  return {
    handedness: 'right',
    profiles: [options.profile ?? 'meta-quest-touch-plus'],
    gripSpace: {handedness: 'right'},
    gamepad: {buttons},
  } as unknown as XRInputSource;
}

function poseFrame(hasPose = true, leftHasPose = true, headY = 1.68): XRFrame {
  return {
    getPose: (space: XRSpace) => {
      const handedness = (space as unknown as {handedness?: XRHandedness}).handedness;
      if ((handedness === 'left' && !leftHasPose) || (handedness !== 'left' && !hasPose)) {
        return null;
      }
      const position = handedness === 'left'
        ? {x: -0.1, y: 0.2, z: 0.3}
        : {x: 0.4, y: 0.5, z: 0.6};
      return {
      transform: {
        position,
        orientation: {x: 0, y: 0, z: 0, w: 1},
      },
      };
    },
    getViewerPose: () => ({transform: {position: {x: 0, y: headY, z: 0}}}),
  } as unknown as XRFrame;
}

function setup(options: {
  supported?: boolean;
  requestError?: unknown;
  referenceError?: unknown;
  missingXR?: boolean;
  events?: string[];
} = {}) {
  const session = new FakeSession();
  if (options.referenceError) session.requestReferenceSpace.mockRejectedValue(options.referenceError);
  const requestSession = options.requestError
    ? vi.fn().mockRejectedValue(options.requestError)
    : vi.fn().mockResolvedValue(session as unknown as XRSession);
  const xr = {
    isSessionSupported: vi.fn().mockResolvedValue(options.supported ?? true),
    requestSession,
  } as unknown as XRSystem;
  const host = new FakeHost(options.events);
  const frames: VRFrame[] = [];
  const controllers: Array<{tracking: boolean; grip: boolean; trigger: number}> = [];
  const controls: string[] = [];
  const statuses: XRSessionStatus[] = [];
  const locks: string[] = [];
  const armRequests: string[] = [];
  const stopRequests: string[] = [];
  const controllerSupport: Array<boolean | null> = [];
  let sessionIndex = 0;
  const controller = new XRSessionController({
    xr: options.missingXR ? undefined : xr,
    host,
    onFrame: (frame) => {
      frames.push(frame);
      options.events?.push('frame');
    },
    onController: (state) => {
      controllers.push(state);
      options.events?.push(`controller:${state.grip ? 'pressed' : 'released'}`);
    },
    onArmRequest: () => {
      armRequests.push('arm');
      options.events?.push('arm');
    },
    onStopOrResetRequest: () => {
      stopRequests.push('stop');
      options.events?.push('stop');
    },
    onControllerSupport: (supported) => {
      controllerSupport.push(supported);
      options.events?.push(`support:${supported}`);
    },
    onDisarm: () => controls.push('disarm'),
    onLockReset: () => locks.push('reset'),
    onStatus: (status) => statuses.push(status),
    createSessionId: () => `quest-session-${++sessionIndex}`,
  });
  return {
    controller,
    session,
    xr,
    host,
    frames,
    controllers,
    controls,
    statuses,
    locks,
    armRequests,
    stopRequests,
    controllerSupport,
  };
}

function emitQuest(
  session: FakeSession,
  host: FakeHost,
  nowMs: number,
  options: Parameters<typeof questSource>[0] = {},
  hasPose = true,
): void {
  session.inputSources = [questSource(options)];
  host.loop?.(nowMs, poseFrame(hasPose));
}

beforeEach(() => vi.restoreAllMocks());

describe('XRSessionController lifecycle', () => {
  it('publishes controller and support before the frame, then emits one A edge', async () => {
    const events: string[] = [];
    const {controller, session, host, armRequests} = setup({events});
    await controller.enterVR();

    emitQuest(session, host, 100, {grip: false, a: false});
    events.length = 0;
    emitQuest(session, host, 120, {grip: false, a: true});
    emitQuest(session, host, 140, {grip: false, a: true});

    expect(events.filter((event) => event !== 'presentation' && event !== 'render').slice(0, 4)).toEqual([
      'controller:released',
      'support:true',
      'frame',
      'arm',
    ]);
    expect(armRequests).toHaveLength(1);
  });

  it('allows another A edge only after release and requires Grip released', async () => {
    const {controller, session, host, armRequests} = setup();
    await controller.enterVR();

    emitQuest(session, host, 100, {a: false});
    emitQuest(session, host, 120, {grip: true, a: true});
    emitQuest(session, host, 140, {a: false});
    emitQuest(session, host, 160, {grip: false, a: true});
    emitQuest(session, host, 180, {a: false});
    emitQuest(session, host, 200, {grip: false, a: true});

    expect(armRequests).toHaveLength(2);
  });

  it('requires A release after tracking loss before another arm edge', async () => {
    const {controller, session, host, armRequests} = setup();
    await controller.enterVR();
    emitQuest(session, host, 100, {a: false});
    emitQuest(session, host, 120, {a: true});
    emitQuest(session, host, 140, {a: true}, false);
    emitQuest(session, host, 160, {a: true});
    expect(armRequests).toHaveLength(1);
    emitQuest(session, host, 180, {a: false});
    emitQuest(session, host, 200, {a: true});
    expect(armRequests).toHaveLength(2);
  });

  it('requires release after visibility recovery when A was held', async () => {
    const {controller, session, host, armRequests, controllerSupport} = setup();
    await controller.enterVR();
    emitQuest(session, host, 100, {a: false});

    session.visibilityState = 'hidden';
    session.dispatchEvent(new Event('visibilitychange'));
    expect(controllerSupport.at(-1)).toBeNull();
    session.visibilityState = 'visible';
    session.dispatchEvent(new Event('visibilitychange'));

    emitQuest(session, host, 120, {a: true});
    expect(armRequests).toHaveLength(0);
    emitQuest(session, host, 140, {a: false});
    emitQuest(session, host, 160, {a: true});
    expect(armRequests).toHaveLength(1);
  });

  it('resets latches for unsupported profiles without auto-arming on recovery', async () => {
    const {controller, session, host, armRequests, controllerSupport} = setup();
    await controller.enterVR();
    emitQuest(session, host, 100, {a: false});
    emitQuest(session, host, 120, {profile: 'generic-trigger-squeeze', a: true});
    expect(controllerSupport.at(-1)).toBe(false);
    emitQuest(session, host, 140, {a: true});
    expect(armRequests).toHaveLength(0);
    emitQuest(session, host, 160, {a: false});
    emitQuest(session, host, 180, {a: true});
    expect(armRequests).toHaveLength(1);
  });

  it('B released edge routes one context-sensitive stop-or-reset request and wins over A', async () => {
    const {controller, session, host, armRequests, stopRequests} = setup();
    await controller.enterVR();
    emitQuest(session, host, 100, {a: false, b: false});
    emitQuest(session, host, 120, {a: true, b: true});
    emitQuest(session, host, 140, {a: true, b: true});

    expect(stopRequests).toHaveLength(1);
    expect(armRequests).toHaveLength(0);

    emitQuest(session, host, 160, {a: false, b: false});
    emitQuest(session, host, 180, {grip: true, b: true});
    emitQuest(session, host, 200, {grip: true, b: true});
    expect(stopRequests).toHaveLength(2);
  });

  it('requires release in a restarted session before a held A can arm', async () => {
    const first = setup();
    await first.controller.enterVR();
    emitQuest(first.session, first.host, 100, {a: false});
    emitQuest(first.session, first.host, 120, {a: true});
    await first.controller.exitVR();

    const secondSession = new FakeSession();
    vi.mocked(first.xr.requestSession).mockResolvedValue(secondSession as unknown as XRSession);
    await first.controller.enterVR();
    emitQuest(secondSession, first.host, 200, {a: true});
    expect(first.armRequests).toHaveLength(1);
    emitQuest(secondSession, first.host, 220, {a: false});
    emitQuest(secondSession, first.host, 240, {a: true});
    expect(first.armRequests).toHaveLength(2);
  });

  it('requests only immersive-vr with required local-floor', async () => {
    const {controller, session, xr, host} = setup();

    await controller.enterVR();

    expect(xr.requestSession).toHaveBeenCalledWith('immersive-vr', {
      requiredFeatures: ['local-floor'],
    });
    expect(session.requestReferenceSpace).toHaveBeenCalledWith('local-floor');
    expect(host.startXR).toHaveBeenCalledOnce();
    expect(controller.isActive).toBe(true);
  });

  it('caps transport at 60 Hz and keeps sequence and session identity monotonic', async () => {
    const {controller, host, frames} = setup();
    await controller.enterVR();

    host.loop?.(100, poseFrame());
    host.loop?.(110, poseFrame());
    host.loop?.(117, poseFrame());
    host.loop?.(134, poseFrame());

    expect(frames.map(({seq}) => seq)).toEqual([0, 1, 2]);
    expect(new Set(frames.map(({session_id}) => session_id))).toEqual(new Set(['quest-session-1']));
    expect(frames.map(({client_mono_ms}) => client_mono_ms)).toEqual([100, 117, 134]);
  });

  it('forwards both hands and viewer height before every visible XR render', async () => {
    const events: string[] = [];
    const {controller, session, host, frames} = setup({events});
    session.inputSources = [leftSource(), rightSource()];
    await controller.enterVR();
    events.length = 0;

    host.loop?.(100, poseFrame());
    host.loop?.(110, poseFrame());

    expect(host.updateXRPresentation).toHaveBeenCalledTimes(2);
    expect(host.renderXR).toHaveBeenCalledTimes(2);
    expect(events.filter((event) => event === 'presentation' || event === 'render')).toEqual([
      'presentation',
      'render',
      'presentation',
      'render',
    ]);
    expect(host.updateXRPresentation).toHaveBeenLastCalledWith({
      left: {
        p: [-0.1, 0.2, 0.3],
        q: [0, 0, 0, 1],
        trackingValid: true,
        thumbstickX: 0,
        thumbstickY: -0.75,
        thumbstickPressed: true,
        grip: false,
      },
      right: expect.objectContaining({
        p: [0.4, 0.5, 0.6],
        q: [0, 0, 0, 1],
        trackingValid: true,
        grip: true,
        trigger: 0.25,
      }),
      headY: 1.68,
      headQ: null,
    }, 110);
    expect(frames).toHaveLength(1);
    expect(frames.at(-1)).toMatchObject({
      right: {p: [0.4, 0.5, 0.6], grip: true, trigger: 0.25},
    });
    expect(frames.at(-1)).not.toHaveProperty('left');
  });

  it('keeps the valid hand in presentation samples when only one hand loses tracking', async () => {
    const {controller, session, host, controls, locks} = setup();
    session.inputSources = [leftSource(), rightSource()];
    await controller.enterVR();
    host.loop?.(100, poseFrame());
    const safety = {controls: controls.length, locks: locks.length};

    host.loop?.(117, poseFrame(true, false));
    expect(host.updateXRPresentation).toHaveBeenLastCalledWith(expect.objectContaining({
      left: expect.objectContaining({p: [0, 0, 0], trackingValid: false}),
      right: expect.objectContaining({p: [0.4, 0.5, 0.6], trackingValid: true}),
    }), 117);
    expect(controls).toHaveLength(safety.controls);
    expect(locks).toHaveLength(safety.locks);

    host.loop?.(134, poseFrame(false, true));
    expect(host.updateXRPresentation).toHaveBeenLastCalledWith(expect.objectContaining({
      left: expect.objectContaining({p: [-0.1, 0.2, 0.3], trackingValid: true}),
      right: expect.objectContaining({p: [0, 0, 0], trackingValid: false}),
    }), 134);
    expect(controls).toHaveLength(safety.controls + 1);
    expect(locks).toHaveLength(safety.locks + 1);
  });

  it('visibility loss immediately disarms and emits one final tracking-invalid frame', async () => {
    const {controller, session, host, frames, controllers, controls, locks} = setup();
    await controller.enterVR();
    host.loop?.(100, poseFrame());
    const beforeLoss = frames.length;

    session.visibilityState = 'hidden';
    session.dispatchEvent(new Event('visibilitychange'));

    expect(controls.at(-1)).toBe('disarm');
    expect(locks).toHaveLength(2);
    expect(frames).toHaveLength(beforeLoss + 1);
    expect(frames.at(-1)).toMatchObject({
      tracking_valid: false,
      visibility: 'hidden',
      right: {grip: false, trigger: 0},
    });
    expect(controllers.at(-1)).toEqual({tracking: false, grip: false, trigger: 0});
    expect(host.updateXRPresentation).toHaveBeenLastCalledWith({
      left: expect.objectContaining({p: [0, 0, 0], trackingValid: false}),
      right: expect.objectContaining({p: [0, 0, 0], trackingValid: false}),
      headY: null,
      headQ: null,
    }, expect.any(Number));

    host.loop?.(140, poseFrame());
    expect(frames).toHaveLength(beforeLoss + 1);
  });

  it('tracking loss closes the local command gate on the first invalid sample', async () => {
    const {controller, host, frames, controls, locks} = setup();
    await controller.enterVR();
    host.loop?.(100, poseFrame());
    const beforeLoss = {controls: controls.length, locks: locks.length};

    host.loop?.(117, poseFrame(false));

    expect(controls).toHaveLength(beforeLoss.controls + 1);
    expect(locks).toHaveLength(beforeLoss.locks + 1);
    expect(frames.at(-1)?.tracking_valid).toBe(false);

    host.loop?.(134, poseFrame(false));
    expect(controls).toHaveLength(beforeLoss.controls + 1);
    expect(locks).toHaveLength(beforeLoss.locks + 1);
  });

  it('session end disarms, locks, clears input, and restores the desktop loop', async () => {
    const {controller, session, host, controllers, controls, locks, statuses} = setup();
    await controller.enterVR();
    host.loop?.(100, poseFrame());

    session.dispatchEvent(new Event('end'));
    await Promise.resolve();

    expect(controls.at(-1)).toBe('disarm');
    expect(locks).toHaveLength(2);
    expect(controllers.at(-1)).toEqual({tracking: false, grip: false, trigger: 0});
    expect(host.updateXRPresentation).toHaveBeenLastCalledWith({
      left: expect.objectContaining({p: [0, 0, 0], trackingValid: false}),
      right: expect.objectContaining({p: [0, 0, 0], trackingValid: false}),
      headY: null,
      headQ: null,
    }, expect.any(Number));
    expect(host.stopXR).toHaveBeenCalledOnce();
    expect(statuses.at(-1)).toEqual({state: 'idle'});
    expect(controller.isActive).toBe(false);
  });

  it('exitVR is safe without a session and awaits the active session end', async () => {
    const {controller, session} = setup();

    await expect(controller.exitVR()).resolves.toBeUndefined();
    await controller.enterVR();
    await controller.exitVR();

    expect(session.end).toHaveBeenCalledOnce();
    expect(controller.isActive).toBe(false);
  });

  it.each([
    ['unsupported browser', {missingXR: true}, '此浏览器不支持 WebXR'],
    ['unsupported headset', {supported: false}, '当前设备不支持沉浸式 VR'],
  ])('reports readable Chinese for %s and never claims VR started', async (_name, options, message) => {
    const {controller, statuses, host, controls, locks} = setup(options);

    await controller.enterVR();

    expect(statuses.at(-1)).toEqual({state: 'error', message});
    expect(statuses.some(({state}) => state === 'active')).toBe(false);
    expect(host.startXR).not.toHaveBeenCalled();
    expect(controls).toEqual(['disarm']);
    expect(locks).toEqual(['reset']);
  });

  it.each([
    ['request rejection', {requestError: new DOMException('permission raw')}, '无法进入 VR，请确认浏览器权限后重试。'],
    ['reference-space failure', {referenceError: new DOMException('space raw')}, 'VR 空间初始化失败，请退出后重试。'],
  ])('contains raw browser errors after %s', async (_name, options, expected) => {
    const {controller, statuses} = setup(options);

    await controller.enterVR();

    expect(statuses.at(-1)).toEqual({state: 'error', message: expected});
    expect(JSON.stringify(statuses)).not.toContain('raw');
    expect(controller.isActive).toBe(false);
  });

  it('restart uses a new session id and never emits arm_request automatically', async () => {
    const first = setup();
    await first.controller.enterVR();
    first.host.loop?.(100, poseFrame());
    await first.controller.exitVR();

    const secondSession = new FakeSession();
    vi.mocked(first.xr.requestSession).mockResolvedValue(secondSession as unknown as XRSession);
    await first.controller.enterVR();
    first.host.loop?.(200, poseFrame());

    expect(first.frames.at(0)?.session_id).not.toBe(first.frames.at(-1)?.session_id);
    expect(first.frames.at(-1)?.seq).toBe(0);
    expect(first.controls).not.toContain('arm_request');
  });

  it('dispose removes listeners and blocks later frames and actions', async () => {
    const {controller, session, host, frames, controls, statuses} = setup();
    await controller.enterVR();
    const staleLoop = host.loop;

    await controller.dispose();
    expect(host.updateXRPresentation).toHaveBeenLastCalledWith({
      left: expect.objectContaining({p: [0, 0, 0], trackingValid: false}),
      right: expect.objectContaining({p: [0, 0, 0], trackingValid: false}),
      headY: null,
      headQ: null,
    }, expect.any(Number));
    const counts = {frames: frames.length, controls: controls.length, statuses: statuses.length};
    session.visibilityState = 'hidden';
    session.dispatchEvent(new Event('visibilitychange'));
    session.dispatchEvent(new Event('end'));
    staleLoop?.(100, poseFrame());
    await Promise.resolve();

    expect(frames).toHaveLength(counts.frames);
    expect(controls).toHaveLength(counts.controls);
    expect(statuses).toHaveLength(counts.statuses);
  });

  it('blocks a new session until the prior session cleanup owns and completes teardown', async () => {
    const firstStop = deferred<void>();
    const first = setup();
    first.host.stopXR.mockImplementationOnce(async () => firstStop.promise);
    await first.controller.enterVR();

    first.session.dispatchEvent(new Event('end'));
    await Promise.resolve();

    const secondSession = new FakeSession();
    vi.mocked(first.xr.requestSession).mockResolvedValue(secondSession as unknown as XRSession);
    await first.controller.enterVR();
    expect(first.xr.requestSession).toHaveBeenCalledOnce();

    firstStop.resolve();
    await firstStop.promise;
    await Promise.resolve();
    await first.controller.enterVR();
    expect(first.xr.requestSession).toHaveBeenCalledTimes(2);
    expect(first.host.loop).not.toBeNull();

    const safetyBeforeSecondEnd = {controls: first.controls.length, locks: first.locks.length};
    secondSession.dispatchEvent(new Event('end'));
    await Promise.resolve();
    await Promise.resolve();

    expect(first.controls).toHaveLength(safetyBeforeSecondEnd.controls + 1);
    expect(first.locks).toHaveLength(safetyBeforeSecondEnd.locks + 1);
    expect(first.host.stopXR).toHaveBeenCalledTimes(2);
    expect(first.host.loop).toBeNull();
  });

  it('never publishes active when the session ends while renderer startup is pending', async () => {
    const startup = deferred<void>();
    const {controller, session, host, statuses} = setup();
    host.startXR.mockImplementationOnce(async (_session, loop) => {
      host.loop = loop;
      await startup.promise;
    });

    const entering = controller.enterVR();
    await vi.waitFor(() => expect(host.startXR).toHaveBeenCalledOnce());
    session.dispatchEvent(new Event('end'));
    startup.resolve();
    await entering;
    await Promise.resolve();

    expect(statuses.some(({state}) => state === 'active')).toBe(false);
    expect(controller.isActive).toBe(false);
    expect(host.stopXR).toHaveBeenCalledOnce();
  });

  it('closes the local gate synchronously when dispose starts and waits safely for delayed end', async () => {
    const ending = deferred<void>();
    const events: string[] = [];
    const {controller, session, host, frames, controls, statuses} = setup();
    session.end.mockImplementationOnce(async () => {
      events.push('session-end-start');
      await ending.promise;
      events.push('session-end-finish');
    });
    await controller.enterVR();
    const staleLoop = host.loop;
    const beforeDispose = {frames: frames.length, statuses: statuses.length};
    const controlsBeforeDispose = controls.length;

    events.push('dispose-start');
    const disposal = controller.dispose();
    if (controls.length > controlsBeforeDispose) events.push('disarm');
    events.push('socket-close');

    expect(events.indexOf('disarm')).toBeLessThan(events.indexOf('socket-close'));
    expect(controls).toHaveLength(controlsBeforeDispose + 1);
    staleLoop?.(100, poseFrame());
    session.visibilityState = 'hidden';
    session.dispatchEvent(new Event('visibilitychange'));
    session.dispatchEvent(new Event('end'));
    expect(frames).toHaveLength(beforeDispose.frames);
    expect(statuses).toHaveLength(beforeDispose.statuses);
    expect(controls).toHaveLength(controlsBeforeDispose + 1);

    ending.resolve();
    await disposal;
  });

  it('contains teardown rejection and reports a readable Chinese end failure', async () => {
    const {controller, session, host, statuses} = setup();
    host.stopXR.mockRejectedValueOnce(new DOMException('raw renderer teardown'));
    await controller.enterVR();

    session.dispatchEvent(new Event('end'));
    await vi.waitFor(() => expect(statuses.at(-1)?.state).toBe('error'));

    expect(statuses.at(-1)).toEqual({
      state: 'error',
      message: 'VR 退出失败，桌面模式已恢复，请刷新页面后重试。',
    });
    expect(JSON.stringify(statuses)).not.toContain('raw renderer teardown');
  });

  it('ends the acquired session and reports readable Chinese when renderer startup rejects', async () => {
    const {controller, session, host, statuses} = setup();
    host.startXR.mockRejectedValueOnce(new DOMException('raw renderer startup'));

    await controller.enterVR();

    expect(session.end).toHaveBeenCalledOnce();
    expect(statuses.at(-1)).toEqual({
      state: 'error',
      message: 'VR 渲染初始化失败，请退出后重试。',
    });
    expect(JSON.stringify(statuses)).not.toContain('raw renderer startup');
    expect(controller.isActive).toBe(false);
  });
});

function deferred<T>() {
  let resolve!: (value: T | PromiseLike<T>) => void;
  let reject!: (reason?: unknown) => void;
  const promise = new Promise<T>((resolvePromise, rejectPromise) => {
    resolve = resolvePromise;
    reject = rejectPromise;
  });
  return {promise, resolve, reject};
}
