import type {VRFrame, VisibilityState} from '../protocol/messages';
import {createVRFrame} from '../scenes/simulationScene';
import {
  invalidControllerSample,
  readRightController,
  type ControllerSample,
} from './controllerInput';

const FRAME_INTERVAL_MS = 1_000 / 60;

export interface XRRenderHost {
  startXR(session: XRSession, loop: XRFrameRequestCallback): Promise<void>;
  stopXR(): Promise<void>;
  renderXR(nowMs: number): void;
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
  onDisarm(): void;
  onLockReset(): void;
  onStatus(status: XRSessionStatus): void;
  createSessionId?: () => string;
  now?: () => number;
}

export class XRSessionController {
  private session: XRSession | null = null;
  private referenceSpace: XRReferenceSpace | null = null;
  private sessionId = '';
  private sequence = 0;
  private lastFrameMs = Number.NEGATIVE_INFINITY;
  private lastClientMs = 0;
  private lastSample: ControllerSample = invalidControllerSample();
  private entering = false;
  private suspended = false;
  private disposed = false;
  private cleanupPromise: Promise<void> | null = null;
  private generation = 0;

  constructor(private readonly options: XRSessionControllerOptions) {}

  get isActive(): boolean {
    return this.session !== null;
  }

  async enterVR(): Promise<void> {
    if (this.disposed || this.entering || this.session) return;
    const generation = ++this.generation;
    this.entering = true;
    this.options.onDisarm();
    this.options.onLockReset();
    this.options.onController({tracking: false, grip: false, trigger: 0});
    this.options.onStatus({state: 'starting'});
    try {
      const xr = this.options.xr;
      if (!xr) {
        this.fail('此浏览器不支持 WebXR');
        return;
      }

      let supported = false;
      try {
        supported = await xr.isSessionSupported('immersive-vr');
      } catch {
        this.fail('无法检测 VR 支持，请使用 Quest 浏览器重试。');
        return;
      }
      if (!supported) {
        this.fail('当前设备不支持沉浸式 VR');
        return;
      }

      let session: XRSession;
      try {
        session = await xr.requestSession('immersive-vr', {requiredFeatures: ['local-floor']});
      } catch {
        this.fail('无法进入 VR，请确认浏览器权限后重试。');
        return;
      }
      if (this.disposed || generation !== this.generation) {
        await safeEnd(session);
        return;
      }

      let referenceSpace: XRReferenceSpace;
      try {
        referenceSpace = await session.requestReferenceSpace('local-floor');
      } catch {
        await safeEnd(session);
        this.fail('VR 空间初始化失败，请退出后重试。');
        return;
      }
      if (this.disposed || generation !== this.generation) {
        await safeEnd(session);
        return;
      }

      this.session = session;
      this.referenceSpace = referenceSpace;
      this.sessionId = (this.options.createSessionId ?? createXRSessionId)();
      this.sequence = 0;
      this.lastFrameMs = Number.NEGATIVE_INFINITY;
      this.lastClientMs = 0;
      this.lastSample = invalidControllerSample();
      this.suspended = false;
      session.addEventListener('visibilitychange', this.onVisibilityChange);
      session.addEventListener('end', this.onSessionEnd);
      try {
        await this.options.host.startXR(session, this.onXRFrame);
      } catch {
        this.removeSessionListeners(session);
        this.session = null;
        this.referenceSpace = null;
        await safeEnd(session);
        await this.options.host.stopXR();
        this.fail('VR 渲染初始化失败，请退出后重试。');
        return;
      }
      if (this.disposed || generation !== this.generation) {
        await this.finishSession(session, false);
        return;
      }
      this.options.onStatus({state: 'active'});
    } finally {
      this.entering = false;
    }
  }

  async exitVR(): Promise<void> {
    const session = this.session;
    if (!session) return;
    await safeEnd(session);
    if (this.session === session) await this.finishSession(session, true);
    if (this.cleanupPromise) await this.cleanupPromise;
  }

  async dispose(): Promise<void> {
    if (this.disposed) return;
    this.generation += 1;
    await this.exitVR();
    this.disposed = true;
  }

  private readonly onXRFrame: XRFrameRequestCallback = (nowMs, frame): void => {
    const session = this.session;
    const referenceSpace = this.referenceSpace;
    if (this.disposed || !session || !referenceSpace) return;
    this.options.host.renderXR(nowMs);
    if (this.suspended || session.visibilityState !== 'visible') return;
    if (nowMs - this.lastFrameMs < FRAME_INTERVAL_MS) return;

    const sample = readRightController(frame, referenceSpace, session.inputSources)
      ?? invalidControllerSample();
    if (!sample.trackingValid && this.lastSample.trackingValid) {
      this.options.onDisarm();
      this.options.onLockReset();
    }
    this.emitFrame(nowMs, sample, 'visible');
  };

  private readonly onVisibilityChange = (): void => {
    const session = this.session;
    if (this.disposed || !session) return;
    if (session.visibilityState === 'visible') {
      this.suspended = false;
      this.lastFrameMs = Number.NEGATIVE_INFINITY;
      return;
    }

    if (this.suspended) return;
    this.suspended = true;
    this.options.onDisarm();
    this.options.onLockReset();
    const sample = invalidControllerSample();
    this.emitFrame(this.now(), sample, visibilityFor(session.visibilityState), true);
  };

  private readonly onSessionEnd = (): void => {
    const session = this.session;
    if (!session || this.disposed) return;
    void this.finishSession(session, true);
  };

  private emitFrame(
    nowMs: number,
    sample: ControllerSample,
    visibility: VisibilityState,
    force = false,
  ): void {
    if (!force) this.lastFrameMs = nowMs;
    const clientMs = Math.max(nowMs, this.lastClientMs);
    this.lastClientMs = clientMs;
    this.lastSample = sample;
    this.options.onController({
      tracking: sample.trackingValid,
      grip: sample.grip,
      trigger: sample.trigger,
    });
    this.options.onFrame(createVRFrame({
      sessionId: this.sessionId,
      sequence: this.sequence++,
      nowMs: clientMs,
      trackingValid: sample.trackingValid,
      position: sample.p,
      quaternion: sample.q,
      grip: sample.grip,
      trigger: sample.trigger,
      visibility,
    }));
  }

  private async finishSession(session: XRSession, announceIdle: boolean): Promise<void> {
    if (this.cleanupPromise) return this.cleanupPromise;
    this.cleanupPromise = (async () => {
      this.removeSessionListeners(session);
      if (this.session === session) {
        this.session = null;
        this.referenceSpace = null;
      }
      if (!this.disposed) {
        this.options.onDisarm();
        this.options.onLockReset();
        this.lastSample = invalidControllerSample();
        this.options.onController({tracking: false, grip: false, trigger: 0});
      }
      await this.options.host.stopXR();
      if (!this.disposed && announceIdle) this.options.onStatus({state: 'idle'});
    })();
    try {
      await this.cleanupPromise;
    } finally {
      this.cleanupPromise = null;
    }
  }

  private removeSessionListeners(session: XRSession): void {
    session.removeEventListener('visibilitychange', this.onVisibilityChange);
    session.removeEventListener('end', this.onSessionEnd);
  }

  private fail(message: string): void {
    if (!this.disposed) this.options.onStatus({state: 'error', message});
  }

  private now(): number {
    return (this.options.now ?? (() => performance.now()))();
  }
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
