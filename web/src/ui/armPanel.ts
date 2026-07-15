import {PROTOCOL_VERSION, type ClientControlMessage, type TeleopMode} from '../protocol/messages';

export type SendControl = (message: ClientControlMessage) => void;

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

  constructor(
    container: Element,
    private readonly sendControl: SendControl,
    onEnterVR: () => void = () => {},
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

    this.armButton.addEventListener('click', () => this.requestArm());
    this.stopButton.addEventListener('click', () => this.stop());
    this.vrButton.addEventListener('click', onEnterVR);
  }

  get isArmed(): boolean {
    return this.armed;
  }

  observeGrip(grip: boolean): void {
    if (!grip && !this.fault && this.connected) this.eligible = true;
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
    this.armed = mode === 'ARMED' || mode === 'ACTIVE' || mode === 'HOLD';
    if (mode === 'FAULT' || mode === 'DISCONNECTED' || mode === 'DISARMED') {
      this.eligible = false;
    }
    this.syncButtonState();
  }

  resetToLocked(): void {
    this.armed = false;
    this.eligible = false;
    this.syncButtonState();
  }

  onSocketReconnect(): void {
    this.resetToLocked();
  }

  private requestArm(): void {
    if (!this.connected || !this.eligible || this.fault || this.armed) return;
    this.sendControl(this.control('arm_request'));
    this.armed = true;
    this.eligible = false;
    this.syncButtonState();
  }

  private stop(): void {
    this.sendControl(this.control('disarm'));
    this.resetToLocked();
  }

  private control(type: 'arm_request' | 'disarm'): ClientControlMessage {
    this.requestSequence += 1;
    return {
      v: PROTOCOL_VERSION,
      type,
      request_id: `desktop-${type}-${this.requestSequence}`,
    };
  }

  private syncButtonState(): void {
    this.armButton.disabled = !this.connected || !this.eligible || this.armed || Boolean(this.fault);
    this.armButton.dataset.state = this.armed ? 'armed' : this.eligible ? 'ready' : 'locked';
    this.armButton.setAttribute(
      'aria-label',
      this.fault ? '故障状态，无法解锁仿真' : this.armed ? '仿真已解锁' : '解锁仿真',
    );
  }
}

function requireButton(container: Element, selector: string): HTMLButtonElement {
  const button = container.querySelector<HTMLButtonElement>(selector);
  if (!button) throw new Error(`缺少控制按钮: ${selector}`);
  return button;
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
