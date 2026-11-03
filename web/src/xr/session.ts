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
  | {state: 'starting'; message?: string}
  | {state: 'active'}
  | {state: 'error'; message: string};

export interface XRSessionControllerOptions {
  xr?: XRSystem;
  host: XRRenderHost;
  onFrame(frame: VRFrame): void;
  onController(state: {tracking: boolean; grip: boolean; trigger: number}): void;
  onArmRequest?(): void;
  onClawHold?(direction:-1|0|1,nowMs:number):void;
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
  clawReleasedSeen:boolean;
  suspended: boolean;
  activationComplete: boolean;
  ending: boolean;
  announceIdle: boolean;
  cleanupPromise: Promise<void> | null;
}

export class XRSessionController {
  private current: SessionContext | null = null;
  private lastConstraint: ConstraintKind | null = null;
  private enteringGeneration: number | null = null;
  private disposed = false;
  private cleanupPromise: Promise<void> | null = null;
  private exitPromise: Promise<void> | null = null;
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
    if (this.disposed || this.enteringGeneration !== null || this.current || this.cleanupPromise || this.exitPromise) return;
    const generation = ++this.generation;
    this.enteringGeneration = generation;
    this.signalSafety();
    let stage='检测设备支持';
    let pendingSession:XRSession|null=null;
    const progress=(message:string)=>{stage=message;this.options.onStatus({state:'starting',message});};
    progress(stage);
    const startupTimer=setTimeout(()=>{
      if(!this.entryIsLive(generation))return;
      this.generation+=1;
      const context=this.current;
      if(context){void this.finishSession(context,false);void safeEnd(context.session);}
      else{this.signalSafety();if(pendingSession)void safeEnd(pendingSession);}
      this.fail(`VR 启动超时（${stage}），已撤销控制。请确认头显内权限提示；仍无响应请刷新页面。`);
    },15000);
    try {
      const xr = this.options.xr;
      if (!xr) {
        this.failIfEntryLive(generation, globalThis.isSecureContext===false
          ?'VR 需要安全连接，请在头显浏览器打开 HTTPS 地址。':'此浏览器不支持 WebXR');
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
        progress('等待头显授权');
        session = await xr.requestSession('immersive-vr', {requiredFeatures: ['local-floor']});
        pendingSession=session;
      } catch (error) {
        const name=error instanceof DOMException?error.name:'';
        const message=name==='SecurityError'||name==='NotAllowedError'
          ?'VR 权限被拒绝：请在头显浏览器使用 HTTPS 地址，允许沉浸式 VR 后再次点击进入。'
          :name==='NotSupportedError'
            ?'当前设备或浏览器不支持沉浸式 VR，请在头显浏览器打开此页面。'
            :name==='InvalidStateError'
              ?'已有 VR 会话正在运行或启动，请先退出其它 VR 页面再试。'
              :'无法进入 VR，请确认浏览器权限后重试。';
        this.failIfEntryLive(generation,message);
        return;
      }
      if (!this.entryIsLive(generation)) {
        await safeEnd(session);
        return;
      }

      let referenceSpace: XRReferenceSpace;
      try {
        progress('初始化 VR 空间');
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
        progress('初始化 VR 渲染');
        await this.options.host.startXR(session, context.frameListener);
      } catch {
        startupFailed = true;
      } finally {
        context.activationComplete = true;
        context.settleActivation();
      }

      if (startupFailed) {
        const endedDuringStartup = context.ending;
        if (!endedDuringStartup) await safeEnd(session);
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
      clearTimeout(startupTimer);
      this.enteringGeneration = null;
    }
  }

  exitVR(): Promise<void> {
    if (this.exitPromise) return this.exitPromise;
    const context = this.current;
    if (!context) {
      if (this.enteringGeneration === this.generation && !this.cleanupPromise && !this.disposed) {
        // Late acquisition is ended by enterVR's generation checks, never activated.
        this.generation += 1;
        this.signalSafety();
        this.options.onStatus({state: 'idle'});
      }
      return this.cleanupPromise ?? Promise.resolve();
    }

    // Revoke the old input before browser end can stall; retain the entry gate
    // until both browser end and renderer teardown have completed.
    const teardownPromise = this.finishSession(context, true);
    let exiting!: Promise<void>;
    exiting = Promise.all([safeEnd(context.session), teardownPromise])
      .then(() => undefined)
      .finally(() => {
        if (this.exitPromise === exiting) this.exitPromise = null;
      });
    this.exitPromise = exiting;
    return exiting;
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
    this.disposePromise = Promise.all([
      endPromise, teardownPromise, this.exitPromise,
    ]).then(() => undefined);
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
      clawReleasedSeen:false,
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
    const left=pair.left;
    if(!left.trackingValid||!left.faceButtonsSupported||left.openButton&&left.closeButton){
      context.clawReleasedSeen=false;this.options.onClawHold?.(0,nowMs);
    }else{
      if(!left.openButton&&!left.closeButton)context.clawReleasedSeen=true;
      this.options.onClawHold?.(context.clawReleasedSeen?(left.openButton?-1:left.closeButton?1:0):0,nowMs);
    }
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
    const sample = pair.right;
    if (!sample.trackingValid && context.lastSample.trackingValid) {
      this.options.onDisarm();
      this.options.onLockReset();
      // Safety transitions must not disappear between transport samples.
      this.emitFrame(context, nowMs, sample, 'visible', true, headQ);
      return;
    }
    if (!sample.grip && context.lastSample.grip) {
      // Preserve clutch release even if a new press arrives before 60 Hz output.
      this.emitFrame(context, nowMs, sample, 'visible', true, headQ);
      return;
    }
    if (nowMs - context.lastFrameMs < FRAME_INTERVAL_MS) return;
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
    this.options.onClawHold?.(0,this.now());
    if(context)context.clawReleasedSeen=false;
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
  context.clawReleasedSeen=false;
  context.armLatch = {pressed: false, releaseSeen: false};
  context.stopLatch = {pressed: false, releaseSeen: false};
}
