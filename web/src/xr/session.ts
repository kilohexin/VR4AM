import type {ConstraintKind, Quat, VRFrame, VisibilityState} from '../protocol/messages';
import {createVRFrame} from '../scenes/simulationScene';
import {
  invalidControllerSample,
  readControllers,
  type ControllerSample,
  type LeftControllerSample,
} from './controllerInput';

const FRAME_INTERVAL_MS = 1_000 / 60;
const TEARDOWN_ERROR = 'VR 退出失败，桌面模式已恢复，请刷新页面后重试。';

interface HapticActuator {
  pulse(value: number, duration: number): Promise<boolean>;
}

type HapticGamepad = Gamepad & {
  hapticActuators?: readonly HapticActuator[];
  vibrationActuator?: HapticActuator | null;
};

export interface XRRenderHost {
  startXR(session: XRSession, loop: XRFrameRequestCallback): Promise<void>;
  stopXR(): Promise<void>;
  updateXRPresentation(sample: XRPresentationSample, nowMs: number): void;
  renderXR(nowMs: number): void;
}

export interface XRPresentationSample {
  left: LeftControllerSample;
  right: ControllerSample;
  headY: number | null;
  headQ?: Quat | null;
}

export type XRSessionStatus =
  | {state: 'idle'}
  | {state: 'starting'}
  | {state: 'active'}
  | {state: 'error'; message: string};

export interface XRSessionControllerOptions {
  xr?: XRSystem;
  host: XRRenderHost;
  onFrame(frame: VRFrame): void;
  onController(state: {tracking: boolean; grip: boolean; trigger: number}): void;
  onArmRequest?(): void;
  onStopOrResetRequest?(): void;
  onControllerSupport?(supported: boolean | null): void;
  onDisarm(): void;
  onLockReset(): void;
  onStatus(status: XRSessionStatus): void;
  createSessionId?: () => string;
  now?: () => number;
}

interface SessionContext {
  readonly generation: number;
  readonly session: XRSession;
  readonly referenceSpace: XRReferenceSpace;
  readonly sessionId: string;
  readonly activationSettled: Promise<void>;
  readonly settleActivation: () => void;
  readonly frameListener: XRFrameRequestCallback;
  readonly visibilityListener: () => void;
  readonly endListener: () => void;
  sequence: number;
  lastFrameMs: number;
  lastClientMs: number;
  lastSample: ControllerSample;
  armLatch: {pressed: boolean; releaseSeen: boolean};
  stopLatch: {pressed: boolean; releaseSeen: boolean};
  suspended: boolean;
  activationComplete: boolean;
  ending: boolean;
  announceIdle: boolean;
  cleanupPromise: Promise<void> | null;
}

export class XRSessionController {
  private current: SessionContext | null = null;
  private lastConstraint: ConstraintKind | null = null;
  private entering = false;
  private disposed = false;
  private cleanupPromise: Promise<void> | null = null;
  private disposePromise: Promise<void> | null = null;
  private generation = 0;

  constructor(private readonly options: XRSessionControllerOptions) {}

  get isActive(): boolean {
    return this.current !== null && !this.current.ending;
  }

  setConstraint(constraint: ConstraintKind | null): void {
    if (constraint === this.lastConstraint) return;
    const shouldPulse = constraint !== null;
    this.lastConstraint = constraint;
    if (!shouldPulse) return;

    const source = [...(this.current?.session.inputSources ?? [])]
      .find((candidate) => candidate.handedness === 'right');
    const gamepad = source?.gamepad as HapticGamepad | undefined;
    const actuator = gamepad?.hapticActuators?.[0] ?? gamepad?.vibrationActuator;
    if (actuator) void actuator.pulse(0.35, 40).catch(() => undefined);
  }

  async enterVR(): Promise<void> {
    if (this.disposed || this.entering || this.current || this.cleanupPromise) return;
    const generation = ++this.generation;
    this.entering = true;
    this.signalSafety();
    this.options.onStatus({state: 'starting'});
    try {
      const xr = this.options.xr;
      if (!xr) {
        this.failIfEntryLive(generation, '此浏览器不支持 WebXR');
        return;
      }

      let supported = false;
      try {
        supported = await xr.isSessionSupported('immersive-vr');
      } catch {
        this.failIfEntryLive(generation, '无法检测 VR 支持，请使用 Quest 浏览器重试。');
        return;
      }
      if (!this.entryIsLive(generation)) return;
      if (!supported) {
        this.fail('当前设备不支持沉浸式 VR');
        return;
      }

      let session: XRSession;
      try {
        session = await xr.requestSession('immersive-vr', {requiredFeatures: ['local-floor']});
      } catch {
        this.failIfEntryLive(generation, '无法进入 VR，请确认浏览器权限后重试。');
        return;
      }
      if (!this.entryIsLive(generation)) {
        await safeEnd(session);
        return;
      }

      let referenceSpace: XRReferenceSpace;
      try {
        referenceSpace = await session.requestReferenceSpace('local-floor');
      } catch {
        await safeEnd(session);
        this.failIfEntryLive(generation, 'VR 空间初始化失败，请退出后重试。');
        return;
      }
      if (!this.entryIsLive(generation)) {
        await safeEnd(session);
        return;
      }

      const context = this.createContext(generation, session, referenceSpace);
      this.current = context;
      session.addEventListener('visibilitychange', context.visibilityListener);
      session.addEventListener('end', context.endListener);

      let startupFailed = false;
      try {
        await this.options.host.startXR(session, context.frameListener);
      } catch {
        startupFailed = true;
      } finally {
        context.activationComplete = true;
        context.settleActivation();
      }

      if (startupFailed) {
        const endedDuringStartup = context.ending;
        await safeEnd(session);
        await this.finishSession(context, false);
        if (!endedDuringStartup) {
          this.failIfEntryLive(generation, 'VR 渲染初始化失败，请退出后重试。');
        }
        return;
      }
      if (!this.ownsLiveContext(context)) {
        await this.finishSession(context, false);
        return;
      }
      this.options.onStatus({state: 'active'});
    } finally {
      this.entering = false;
    }
  }

  async exitVR(): Promise<void> {
    const context = this.current;
    if (!context) {
      if (this.cleanupPromise) await this.cleanupPromise;
      return;
    }
    await safeEnd(context.session);
    await this.finishSession(context, true);
  }

  dispose(): Promise<void> {
    if (this.disposePromise) return this.disposePromise;
    if (this.disposed) return Promise.resolve();

    const context = this.current;
    this.disposed = true;
    this.generation += 1;
    this.signalSafety(context ?? undefined);

    const endPromise = context ? safeEnd(context.session) : Promise.resolve();
    const teardownPromise = context
      ? this.finishSession(context, false, false)
      : (this.cleanupPromise ?? Promise.resolve());
    this.disposePromise = Promise.all([endPromise, teardownPromise]).then(() => undefined);
    return this.disposePromise;
  }

  private createContext(
    generation: number,
    session: XRSession,
    referenceSpace: XRReferenceSpace,
  ): SessionContext {
    let settleActivation!: () => void;
    const activationSettled = new Promise<void>((resolve) => {
      settleActivation = resolve;
    });
    const context: SessionContext = {
      generation,
      session,
      referenceSpace,
      sessionId: (this.options.createSessionId ?? createXRSessionId)(),
      activationSettled,
      settleActivation,
      frameListener: (nowMs, frame) => this.onXRFrame(context, nowMs, frame),
      visibilityListener: () => this.onVisibilityChange(context),
      endListener: () => this.onSessionEnd(context),
      sequence: 0,
      lastFrameMs: Number.NEGATIVE_INFINITY,
      lastClientMs: 0,
      lastSample: invalidControllerSample(),
      armLatch: {pressed: false, releaseSeen: false},
      stopLatch: {pressed: false, releaseSeen: false},
      suspended: false,
      activationComplete: false,
      ending: false,
      announceIdle: false,
      cleanupPromise: null,
    };
    return context;
  }

  private onXRFrame(context: SessionContext, nowMs: number, frame: XRFrame): void {
    if (!this.ownsLiveContext(context)) return;
    if (context.suspended || context.session.visibilityState !== 'visible') return;
    const pair = readControllers(frame, context.referenceSpace, context.session.inputSources);
    const viewerPose = frame.getViewerPose(context.referenceSpace);
    const headOrientation = viewerPose?.transform.orientation;
    const headQ: Quat | null = headOrientation
      ? [headOrientation.x, headOrientation.y, headOrientation.z, headOrientation.w]
      : null;
    this.options.host.updateXRPresentation({
      left: pair.left,
      right: pair.right,
      headY: viewerPose?.transform.position.y ?? null,
      headQ,
    }, nowMs);
    this.options.host.renderXR(nowMs);
    if (nowMs - context.lastFrameMs < FRAME_INTERVAL_MS) return;

    const sample = pair.right;
    if (!sample.trackingValid && context.lastSample.trackingValid) {
      this.options.onDisarm();
      this.options.onLockReset();
    }
    this.emitFrame(context, nowMs, sample, 'visible', false, headQ);
  }

  private onVisibilityChange(context: SessionContext): void {
    if (!this.ownsLiveContext(context)) return;
    if (context.session.visibilityState === 'visible') {
      context.suspended = false;
      context.lastFrameMs = Number.NEGATIVE_INFINITY;
      return;
    }

    if (context.suspended) return;
    context.suspended = true;
    const nowMs = this.now();
    this.options.host.updateXRPresentation(invalidPresentationSample(), nowMs);
    this.options.onDisarm();
    this.options.onLockReset();
    this.emitFrame(
      context,
      nowMs,
      invalidControllerSample(),
      visibilityFor(context.session.visibilityState),
      true,
    );
  }

  private onSessionEnd(context: SessionContext): void {
    void this.finishSession(context, true, !this.disposed);
  }

  private emitFrame(
    context: SessionContext,
    nowMs: number,
    sample: ControllerSample,
    visibility: VisibilityState,
    force = false,
    headQ: Quat | null = null,
  ): void {
    if (!this.ownsLiveContext(context)) return;
    if (!force) context.lastFrameMs = nowMs;
    const clientMs = Math.max(nowMs, context.lastClientMs);
    context.lastClientMs = clientMs;
    context.lastSample = sample;
    this.options.onController({
      tracking: sample.trackingValid,
      grip: sample.grip,
      trigger: sample.trigger,
    });
    this.options.onControllerSupport?.(
      sample.trackingValid ? sample.questFaceButtonsSupported : null,
    );
    this.options.onFrame(createVRFrame({
      sessionId: context.sessionId,
      sequence: context.sequence++,
      nowMs: clientMs,
      trackingValid: sample.trackingValid,
      position: sample.p,
      quaternion: sample.q,
      ...(headQ ? {headQ} : {}),
      grip: sample.grip,
      trigger: sample.trigger,
      visibility,
    }));

    if (!sample.trackingValid || !sample.questFaceButtonsSupported) {
      resetFaceLatches(context);
      return;
    }
    const stopEdge = takeReleasedEdge(context.stopLatch, sample.stopButton);
    const armEdge = takeReleasedEdge(context.armLatch, sample.armButton);
    if (stopEdge) this.options.onStopOrResetRequest?.();
    else if (armEdge && !sample.grip) this.options.onArmRequest?.();
  }

  private finishSession(
    context: SessionContext,
    announceIdle: boolean,
    emitSafety = true,
  ): Promise<void> {
    context.announceIdle ||= announceIdle;
    if (context.cleanupPromise) return context.cleanupPromise;

    context.ending = true;
    context.session.removeEventListener('visibilitychange', context.visibilityListener);
    context.session.removeEventListener('end', context.endListener);
    if (this.current === context) this.current = null;
    if (emitSafety && !this.disposed) this.signalSafety(context);

    let cleanup!: Promise<void>;
    cleanup = (async () => {
      try {
        if (!context.activationComplete) await context.activationSettled;
        let teardownFailed = false;
        try {
          await this.options.host.stopXR();
        } catch {
          teardownFailed = true;
        }

        if (!this.disposed) {
          if (teardownFailed) this.fail(TEARDOWN_ERROR);
          else if (context.announceIdle) this.options.onStatus({state: 'idle'});
        }
      } finally {
        this.clearCleanup(context, cleanup);
      }
    })();
    context.cleanupPromise = cleanup;
    this.cleanupPromise = cleanup;
    return cleanup;
  }

  private clearCleanup(context: SessionContext, cleanup: Promise<void>): void {
    if (context.cleanupPromise === cleanup) context.cleanupPromise = null;
    if (this.cleanupPromise === cleanup) this.cleanupPromise = null;
  }

  private signalSafety(context?: SessionContext): void {
    this.lastConstraint = null;
    if (context) this.options.host.updateXRPresentation(invalidPresentationSample(), this.now());
    this.options.onDisarm();
    this.options.onLockReset();
    if (context) {
      context.lastSample = invalidControllerSample();
      resetFaceLatches(context);
    }
    this.options.onController({tracking: false, grip: false, trigger: 0});
    this.options.onControllerSupport?.(null);
  }

  private ownsLiveContext(context: SessionContext): boolean {
    return !this.disposed
      && !context.ending
      && this.current === context
      && context.generation === this.generation;
  }

  private entryIsLive(generation: number): boolean {
    return !this.disposed && generation === this.generation;
  }

  private failIfEntryLive(generation: number, message: string): void {
    if (this.entryIsLive(generation)) this.fail(message);
  }

  private fail(message: string): void {
    if (!this.disposed) this.options.onStatus({state: 'error', message});
  }

  private now(): number {
    return (this.options.now ?? (() => performance.now()))();
  }
}

function invalidPresentationSample(): XRPresentationSample {
  return {
    left: {
      p: [0, 0, 0],
      q: [0, 0, 0, 1],
      trackingValid: false,
      thumbstickX: 0,
      thumbstickY: 0,
      grip: false,
      thumbstickPressed: false,
    },
    right: invalidControllerSample(),
    headY: null,
    headQ: null,
  };
}

function visibilityFor(state: XRVisibilityState): VisibilityState {
  if (state === 'visible-blurred') return 'visible-blurred';
  return state === 'visible' ? 'visible' : 'hidden';
}

function createXRSessionId(): string {
  const token = typeof crypto !== 'undefined' && 'randomUUID' in crypto
    ? crypto.randomUUID()
    : `${Date.now()}-${Math.random().toString(16).slice(2)}`;
  return `quest-${token}`;
}

async function safeEnd(session: XRSession): Promise<void> {
  try {
    await session.end();
  } catch {
    // A session may already be ending; local safety cleanup still proceeds.
  }
}

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
