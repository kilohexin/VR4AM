import {
  PROTOCOL_VERSION,
  type ArmFeedbackMessage,
  type ClientControlMessage,
  type ConstraintKind,
  type FaultResetRejectReason,
  type FaultResetResultMessage,
  type HomeRejectReason,
  type HomeResultMessage,
  type RecoveryPhase,
  type RuntimeBackend,
  type TeleopMode,
} from '../protocol/messages';
import type {XRSessionStatus} from '../xr/session';
import type {TeleopConnectionStatus} from '../transport/teleopSocket';

export type SendControl = (message: ClientControlMessage) => void;
export type ControlSource = 'desktop' | 'xr';
export type ArmConnectionState = TeleopConnectionStatus['state'];
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
  connectionState: ArmConnectionState;
  eligible: boolean;
  armed: boolean;
  pending: boolean;
  mode: TeleopMode;
  fault: string | null;
  faultRecoverable: boolean;
  faultResetPending: boolean;
  homePending?: boolean;
  constraint: ConstraintKind | null;
  recoveryPhase: RecoveryPhase | null;
}>;

export const RECOVERABLE_FAULTS = new Set([
  'workspace_violation',
  'ik_unreachable',
  'ik_singular',
  'joint_safety_window',
]);

const RESET_REJECTION_FEEDBACK: Readonly<Record<FaultResetRejectReason, string>> = {
  no_fault: '当前没有可复位故障。',
  stop_incomplete: '停止尚未完成，请稍后重试。',
  backend_moving: '机械臂仍在运动，请稍后重试。',
  unrecoverable_fault: '该故障无法在线复位，请重启后端并重新检查。',
  control_loop_unavailable: '控制循环不可用，请重启后端并重新检查。',
};

const HOME_REJECTION_FEEDBACK: Readonly<Record<HomeRejectReason, string>> = {
  fault_present: '存在未清除故障，请先完成故障复位。',
  grip_pressed: '请保持 Grip 松开后重试。',
  not_stopped: '机械臂尚未停止，请先停止后重试。',
  control_loop_unavailable: '控制循环不可用，请重启后端并重新检查。',
  home_failed: '无法返回 Home，请稍后重试。',
};

export class ArmPanel {
  readonly backendText = 'SIMULATOR';
  readonly armButton: HTMLButtonElement;
  readonly stopButton: HTMLButtonElement;
  readonly vrButton: HTMLButtonElement;

  private armed = false;
  private eligible = false;
  private connected = true;
  private connectionState: ArmConnectionState = 'disconnected';
  private fault: string | null = null;
  private constraint: ConstraintKind | null = null;
  private recoveryPhase: RecoveryPhase | null = null;
  private requestSequence = 0;
  private pendingArmRequestId: string | null = null;
  private pendingFaultResetId: string | null = null;
  private pendingHomeRequestId: string | null = null;
  private awaitingArmedMode = false;
  private armFeedback: string | null = null;
  private resetFeedback: string | null = null;
  private homeFeedback: string | null = null;
  private gripPressed = true;
  private mode: TeleopMode = 'READY';
  private stopRequested = false;
  private safetyPublishQueued = false;
  private automationActive = false;
  private vrStarting = false;
  private readonly armLabel: HTMLElement;
  private readonly stopLabel: HTMLElement;
  private runtimeArmLabel = '解锁仿真';

  constructor(
    container: Element,
    private readonly sendControl: SendControl,
    onEnterVR: () => void = () => {},
    private readonly onSafetyChange: (snapshot: ArmSafetySnapshot) => void = () => {},
  ) {
    container.classList.add('command-actions');
    container.innerHTML = `
      <button class="command-button command-button--arm" type="button" disabled>
        ${lockIcon()}<span>解锁仿真</span>
      </button>
      <button class="command-button command-button--stop" type="button">
        ${stopIcon()}<span>停止</span>
      </button>
      <button class="command-button command-button--vr" type="button">
        ${vrIcon()}<span>进入 VR</span>
      </button>
    `;

    this.armButton = requireButton(container, '.command-button--arm');
    this.stopButton = requireButton(container, '.command-button--stop');
    this.vrButton = requireButton(container, '.command-button--vr');
    this.armLabel = requireElement(this.armButton, 'span');
    this.stopLabel = requireElement(this.stopButton, 'span');

    this.armButton.addEventListener('click', () => this.requestArm('desktop'));
    this.stopButton.addEventListener('click', () => this.requestStopOrReset('desktop'));
    this.vrButton.addEventListener('click', () => {
      if (!this.automationActive) onEnterVR();
    });
  }

  get isArmed(): boolean {
    return this.armed;
  }

  get isArmPending(): boolean {
    return this.pendingArmRequestId !== null || this.awaitingArmedMode;
  }

  get feedbackText(): string {
    return this.resetFeedback ?? this.homeFeedback ?? this.armFeedback ?? '';
  }

  get safetyState(): ArmSafetySnapshot {
    const phase = this.safetyPhase();
    return Object.freeze({
      phase,
      connected: this.connected && this.mode !== 'DISCONNECTED',
      connectionState: this.connectionState,
      eligible: this.automationActive || phase === 'disconnected' || phase === 'fault'
        ? false
        : this.eligible,
      armed: (phase === 'armed' || phase === 'active') && this.armed,
      pending: this.isArmPending,
      mode: this.mode,
      fault: this.fault,
      faultRecoverable: this.isFaultRecoverable,
      faultResetPending: this.pendingFaultResetId !== null,
      homePending: this.pendingHomeRequestId !== null,
      constraint: this.constraint,
      recoveryPhase: this.recoveryPhase,
    });
  }

  observeGrip(grip: boolean): void {
    if (this.automationActive) return;
    this.gripPressed = grip;
    if (
      !grip &&
      !this.fault &&
      this.connected &&
      !this.isAuthoritativelyUnavailable() &&
      !this.isArmPending &&
      this.pendingFaultResetId === null &&
      this.pendingHomeRequestId === null &&
      this.recoveryPhase === null
    ) {
      this.eligible = true;
    }
    this.syncButtonState();
  }

  setConnected(connected: boolean): void {
    this.setConnectionStatus({state: connected ? 'connected' : 'disconnected'});
  }

  setConnectionStatus(status: TeleopConnectionStatus): void {
    this.connectionState = status.state;
    this.connected = status.state === 'connected';
    if (status.state !== 'connected') this.setRuntimeIdentity(null);
    this.resetToLocked();
  }

  setRuntimeIdentity(backend: RuntimeBackend | null): void {
    this.runtimeArmLabel = backend === 'LEBAI'
      ? '解锁真机'
      : backend === 'LEBAI_FAKE'
        ? '解锁数字孪生'
        : '解锁仿真';
    this.syncButtonState();
  }

  setAutomationActive(active: boolean): void {
    if (this.automationActive === active) return;
    this.automationActive = active;
    this.syncButtonState();
  }

  setFault(fault: string | null): void {
    if (fault === null && this.pendingFaultResetId !== null) {
      this.syncButtonState();
      return;
    }
    this.fault = fault;
    if (fault) {
      this.armed = false;
      this.eligible = false;
      this.pendingArmRequestId = null;
      this.awaitingArmedMode = false;
      this.armFeedback = null;
      this.stopRequested = true;
      if (!RECOVERABLE_FAULTS.has(fault)) this.pendingFaultResetId = null;
      if (!this.isAuthoritativelyUnavailable()) {
        this.mode = this.connected ? 'DISARMED' : 'DISCONNECTED';
      }
      this.syncButtonState();
    } else {
      this.syncButtonState();
    }
  }

  setConstraint(constraint: ConstraintKind | null): void {
    this.constraint = constraint;
    this.syncButtonState();
  }

  setRecoveryPhase(phase: RecoveryPhase | null): void {
    this.recoveryPhase = phase;
    if (phase !== null) this.eligible = false;
    this.syncButtonState();
  }

  setMode(mode: TeleopMode): void {
    this.mode = mode;
    if (mode === 'READY' || mode === 'ARMED') this.stopRequested = false;
    const authoritativeArmed =
      !this.stopRequested && (mode === 'ARMED' || mode === 'ACTIVE' || mode === 'HOLD');
    this.armed = authoritativeArmed;
    if (authoritativeArmed) {
      this.pendingArmRequestId = null;
      this.awaitingArmedMode = false;
    }
    if (
      mode === 'FAULT' ||
      mode === 'STALE' ||
      mode === 'DISCONNECTED' ||
      mode === 'DISARMED'
    ) {
      this.eligible = false;
      this.pendingArmRequestId = null;
      this.awaitingArmedMode = false;
    }
    this.syncButtonState();
  }

  handleArmFeedback(message: ArmFeedbackMessage): void {
    if (message.request_id !== this.pendingArmRequestId) return;
    this.pendingArmRequestId = null;
    this.eligible = false;
    if (message.type === 'arm_ack') {
      this.awaitingArmedMode = true;
      this.armed = false;
      this.armFeedback = null;
    } else {
      this.awaitingArmedMode = false;
      this.armed = false;
      this.armFeedback = '解锁请求被拒绝，请松开 Grip 后重试';
    }
    this.syncButtonState();
  }

  resetToLocked(): void {
    this.armed = false;
    this.eligible = false;
    this.gripPressed = true;
    this.pendingArmRequestId = null;
    this.awaitingArmedMode = false;
    this.armFeedback = null;
    this.pendingFaultResetId = null;
    this.resetFeedback = null;
    this.pendingHomeRequestId = null;
    this.homeFeedback = null;
    this.constraint = null;
    this.recoveryPhase = null;
    this.stopRequested = true;
    this.mode = this.connected ? 'DISARMED' : 'DISCONNECTED';
    this.syncButtonState();
  }

  onSocketReconnect(): void {
    this.resetToLocked();
  }

  setVRStatus(status: XRSessionStatus): void {
    const label = requireElement(this.vrButton, 'span');
    this.vrStarting = status.state === 'starting';
    this.vrButton.disabled = this.automationActive || this.vrStarting;
    this.vrButton.dataset.state = status.state;
    this.vrButton.title = status.state === 'error' ? status.message : '';
    label.textContent = status.state === 'starting'
      ? '正在进入 VR'
      : status.state === 'active'
        ? '退出 VR'
        : '进入 VR';
  }

  requestArm(source: ControlSource = 'desktop'): boolean {
    if (
      this.automationActive ||
      !this.connected ||
      !this.eligible ||
      this.fault ||
      this.isAuthoritativelyUnavailable() ||
      this.armed ||
      this.isArmPending ||
      this.pendingHomeRequestId !== null ||
      this.recoveryPhase !== null
    ) {
      return false;
    }
    const message = this.control('arm_request', source);
    this.pendingArmRequestId = message.request_id;
    this.eligible = false;
    this.armFeedback = null;
    this.homeFeedback = null;
    this.syncButtonState();
    this.sendControl(message);
    return true;
  }

  requestDisarm(source: ControlSource = 'desktop'): void {
    if (this.automationActive) return;
    this.armed = false;
    this.eligible = false;
    this.pendingArmRequestId = null;
    this.awaitingArmedMode = false;
    this.armFeedback = null;
    this.stopRequested = true;
    this.syncButtonState();
    this.sendControl(this.control('disarm', source));
  }

  requestStopOrReset(source: ControlSource = 'desktop'): void {
    if (this.automationActive) return;
    if (this.fault) {
      if (!this.isFaultRecoverable) return;
      if (!this.connected || this.gripPressed || this.pendingFaultResetId !== null) return;
      const message = this.control('reset_fault', source);
      this.pendingFaultResetId = message.request_id;
      this.resetFeedback = null;
      this.eligible = false;
      this.syncButtonState();
      this.sendControl(message);
      return;
    }
    if (this.isUnlockedOrMoving()) {
      this.requestDisarm(source);
      return;
    }
    if (this.pendingHomeRequestId !== null || this.recoveryPhase !== null) return;
    const stopped = this.mode === 'DISARMED' || (!this.armed && this.mode === 'READY');
    if (stopped && this.connected && !this.gripPressed) {
      const message = this.control('home_request', source);
      this.pendingHomeRequestId = message.request_id;
      this.resetFeedback = null;
      this.homeFeedback = null;
      this.eligible = false;
      this.syncButtonState();
      this.sendControl(message);
      return;
    }
    this.requestDisarm(source);
  }

  handleFaultResetResult(message: FaultResetResultMessage): void {
    if (message.request_id !== this.pendingFaultResetId) return;
    this.pendingFaultResetId = null;
    if (message.accepted) {
      this.fault = null;
      this.resetFeedback = null;
      this.resetToLocked();
      return;
    }
    this.resetFeedback = RESET_REJECTION_FEEDBACK[message.reason];
    this.syncButtonState();
  }

  handleHomeResult(message: HomeResultMessage): void {
    if (message.request_id !== this.pendingHomeRequestId) return;
    this.pendingHomeRequestId = null;
    if (message.accepted) {
      this.homeFeedback = null;
      this.resetToLocked();
      return;
    }
    this.armed = false;
    this.eligible = false;
    this.pendingArmRequestId = null;
    this.awaitingArmedMode = false;
    this.stopRequested = true;
    this.mode = this.connected ? 'DISARMED' : 'DISCONNECTED';
    this.homeFeedback = HOME_REJECTION_FEEDBACK[message.reason];
    this.syncButtonState();
  }

  private control(
    type: 'arm_request' | 'disarm' | 'reset_fault' | 'home_request',
    source: ControlSource,
  ): ClientControlMessage {
    this.requestSequence += 1;
    return {
      v: PROTOCOL_VERSION,
      type,
      request_id: `${source}-${type}-${this.requestSequence}`,
    };
  }

  private safetyPhase(): ArmSafetyPhase {
    if (!this.connected || this.mode === 'DISCONNECTED') return 'disconnected';
    if (this.fault || this.mode === 'FAULT' || this.mode === 'STALE') return 'fault';
    if (this.isArmPending) return 'pending';
    if (this.stopRequested || this.mode === 'DISARMED') return 'stopped';
    if (this.mode === 'ACTIVE' && this.armed) return 'active';
    if (this.armed) return 'armed';
    return 'locked';
  }

  private isAuthoritativelyUnavailable(): boolean {
    return this.mode === 'DISCONNECTED' || this.mode === 'FAULT' || this.mode === 'STALE';
  }

  private isUnlockedOrMoving(): boolean {
    return (
      this.mode === 'ARMED' ||
      this.mode === 'ACTIVE' ||
      this.mode === 'HOLD' ||
      this.armed ||
      this.isArmPending
    );
  }

  private get isFaultRecoverable(): boolean {
    return this.fault !== null && RECOVERABLE_FAULTS.has(this.fault);
  }

  private syncButtonState(): void {
    if (this.automationActive) {
      this.armButton.disabled = true;
      this.armButton.dataset.state = 'automation';
      this.armLabel.textContent = '离线演练运行中';
      this.armButton.title = '自动演练拥有控制权';
      this.armButton.setAttribute('aria-label', '离线演练运行中，手动控制已锁定');
      this.stopLabel.textContent = '离线演练运行中';
      this.stopButton.disabled = true;
      this.stopButton.title = '请使用离线演练面板停止';
      this.vrButton.disabled = true;
      this.publishSafetyChange();
      return;
    }
    this.armButton.disabled =
      !this.connected ||
      !this.eligible ||
      this.armed ||
      Boolean(this.fault) ||
      this.isArmPending ||
      this.pendingHomeRequestId !== null ||
      this.recoveryPhase !== null;
    this.armButton.dataset.state = this.armed
      ? 'armed'
      : this.isArmPending
        ? 'pending'
        : this.eligible
          ? 'ready'
          : 'locked';
    this.armLabel.textContent = this.armFeedback
      ? '解锁被拒绝'
      : this.isArmPending
        ? '正在解锁'
        : this.runtimeArmLabel;
    this.armButton.title = this.armFeedback ?? '';
    this.armButton.setAttribute(
      'aria-label',
      this.fault
        ? '故障状态，无法解锁仿真'
        : this.armFeedback
          ? this.armFeedback
          : this.isArmPending
            ? '正在等待后端确认解锁'
            : this.armed
              ? '仿真已解锁'
              : this.runtimeArmLabel,
    );
    if (this.pendingFaultResetId !== null) {
      this.stopLabel.textContent = this.recoveryPhase === 'stopping'
        ? '正在确认停止…'
        : this.recoveryPhase === 'homing'
          ? '正在回到初始姿态…'
          : this.recoveryPhase === 'stabilizing'
            ? '正在确认 Home 稳定…'
            : '复位中…';
      this.stopButton.disabled = true;
    } else if (this.isFaultRecoverable) {
      this.stopLabel.textContent = '复位故障';
      this.stopButton.disabled = !this.connected || this.gripPressed;
    } else if (this.fault) {
      this.stopLabel.textContent = '无法在线复位';
      this.stopButton.disabled = true;
    } else if (this.isUnlockedOrMoving()) {
      this.stopLabel.textContent = '停止';
      this.stopButton.disabled = false;
    } else if (this.pendingHomeRequestId !== null || this.recoveryPhase !== null) {
      this.stopLabel.textContent = '正在返回 Home…';
      this.stopButton.disabled = true;
    } else {
      const stopped = this.mode === 'DISARMED' || (!this.armed && this.mode === 'READY');
      const canRequestHome = stopped && !this.isArmPending && this.connected && !this.gripPressed;
      this.stopLabel.textContent = canRequestHome ? '回到 Home' : '停止';
      this.stopButton.disabled = false;
    }
    this.stopButton.title = this.resetFeedback ?? this.homeFeedback ?? '';
    this.vrButton.disabled = this.vrStarting;
    this.publishSafetyChange();
  }

  private publishSafetyChange(): void {
    if (this.safetyPublishQueued) return;
    this.safetyPublishQueued = true;
    queueMicrotask(() => {
      this.safetyPublishQueued = false;
      try {
        this.onSafetyChange(this.safetyState);
      } catch {
        // Safety consumers must never interrupt or reorder control transport.
      }
    });
  }
}

function requireButton(container: Element, selector: string): HTMLButtonElement {
  const button = container.querySelector<HTMLButtonElement>(selector);
  if (!button) throw new Error(`缺少控制按钮: ${selector}`);
  return button;
}

function requireElement<T extends HTMLElement>(container: Element, selector: string): T {
  const element = container.querySelector<T>(selector);
  if (!element) throw new Error(`缺少控制元素: ${selector}`);
  return element;
}

function lockIcon(): string {
  return `<svg aria-hidden="true" viewBox="0 0 24 24"><rect x="5" y="10" width="14" height="11" rx="1"/><path d="M8 10V7a4 4 0 0 1 8 0v3"/></svg>`;
}

function stopIcon(): string {
  return `<svg aria-hidden="true" viewBox="0 0 24 24"><rect x="6" y="6" width="12" height="12" rx="1"/></svg>`;
}

function vrIcon(): string {
  return `<svg aria-hidden="true" viewBox="0 0 24 24"><path d="M4 7h16a2 2 0 0 1 2 2v7a3 3 0 0 1-5.3 1.9L14.3 15H9.7l-2.4 2.9A3 3 0 0 1 2 16V9a2 2 0 0 1 2-2Z"/><circle cx="7.5" cy="11" r="1"/><circle cx="16.5" cy="11" r="1"/></svg>`;
}
