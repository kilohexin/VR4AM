import {
  PROTOCOL_VERSION,
  type ArmFeedbackMessage,
  type ClientControlMessage,
  type TeleopMode,
} from '../protocol/messages';
import type {XRSessionStatus} from '../xr/session';

export type SendControl = (message: ClientControlMessage) => void;
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

export class ArmPanel {
  readonly backendText = 'SIMULATOR';
  readonly armButton: HTMLButtonElement;
  readonly stopButton: HTMLButtonElement;
  readonly vrButton: HTMLButtonElement;

  private armed = false;
  private eligible = false;
  private connected = true;
  private fault: string | null = null;
  private requestSequence = 0;
  private pendingArmRequestId: string | null = null;
  private awaitingArmedMode = false;
  private armFeedback: string | null = null;
  private mode: TeleopMode = 'DISCONNECTED';
  private stopRequested = false;
  private readonly armLabel: HTMLElement;

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

    this.armButton.addEventListener('click', () => this.requestArm('desktop'));
    this.stopButton.addEventListener('click', () => this.requestDisarm('desktop'));
    this.vrButton.addEventListener('click', onEnterVR);
  }

  get isArmed(): boolean {
    return this.armed;
  }

  get isArmPending(): boolean {
    return this.pendingArmRequestId !== null || this.awaitingArmedMode;
  }

  get feedbackText(): string {
    return this.armFeedback ?? '';
  }

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

  observeGrip(grip: boolean): void {
    if (!grip && !this.fault && this.connected && !this.isArmPending) {
      this.eligible = true;
    }
    this.syncButtonState();
  }

  setConnected(connected: boolean): void {
    this.connected = connected;
    this.resetToLocked();
  }

  setFault(fault: string | null): void {
    this.fault = fault;
    if (fault) this.resetToLocked();
    this.syncButtonState();
  }

  setMode(mode: TeleopMode): void {
    this.mode = mode;
    const authoritativeArmed = mode === 'ARMED' || mode === 'ACTIVE' || mode === 'HOLD';
    this.armed = authoritativeArmed;
    if (authoritativeArmed) {
      this.pendingArmRequestId = null;
      this.awaitingArmedMode = false;
    }
    if (mode === 'FAULT' || mode === 'DISCONNECTED' || mode === 'DISARMED') {
      this.eligible = false;
      this.pendingArmRequestId = null;
      this.awaitingArmedMode = false;
    }
    if (mode === 'READY') this.stopRequested = false;
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
    this.pendingArmRequestId = null;
    this.awaitingArmedMode = false;
    this.armFeedback = null;
    this.stopRequested = false;
    this.syncButtonState();
  }

  onSocketReconnect(): void {
    this.resetToLocked();
  }

  setVRStatus(status: XRSessionStatus): void {
    const label = requireElement(this.vrButton, 'span');
    this.vrButton.disabled = status.state === 'starting';
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
      !this.connected ||
      !this.eligible ||
      this.fault ||
      this.armed ||
      this.isArmPending
    ) {
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
    return {
      v: PROTOCOL_VERSION,
      type,
      request_id: `${source}-${type}-${this.requestSequence}`,
    };
  }

  private safetyPhase(): ArmSafetyPhase {
    if (!this.connected) return 'disconnected';
    if (this.fault) return 'fault';
    if (this.stopRequested || this.mode === 'DISARMED') return 'stopped';
    if (this.mode === 'ACTIVE') return 'active';
    if (this.isArmPending) return 'pending';
    if (this.armed) return 'armed';
    return 'locked';
  }

  private syncButtonState(): void {
    this.armButton.disabled =
      !this.connected ||
      !this.eligible ||
      this.armed ||
      Boolean(this.fault) ||
      this.isArmPending;
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
        : '解锁仿真';
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
              : '解锁仿真',
    );
    this.onSafetyChange(this.safetyState);
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
