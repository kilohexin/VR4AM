import type {
  BackendState,
  RuntimeBackend,
  SimulationScaleRejectReason,
  SimulationScaleResultMessage,
  TeleopMode,
} from '../protocol/messages';

export interface SimulationSettingsState {
  runtime: RuntimeBackend | null;
  connected: boolean;
  mode: TeleopMode | null;
  backendState: BackendState | null;
  grip: boolean;
  automationActive: boolean;
  authoritativeScale: number | null;
}

const STORAGE_KEY = 'vr4arm.simulation.translationScale';

export class SimulationSettingsPanel {
  private readonly input: HTMLInputElement;
  private readonly apply: HTMLButtonElement;
  private readonly label: HTMLElement;
  private readonly status: HTMLElement;
  private pending = false;
  private editing = false;
  private statusOverride: string | null = null;
  private state: SimulationSettingsState | null = null;

  constructor(
    root: Element,
    private readonly onScaleRequest: (value: number) => void,
    onResetView: () => void = () => {},
  ) {
    root.innerHTML = `
      <section class="simulation-settings-panel" aria-label="仿真操作设置">
        <h2>操作设置</h2>
        <label data-field="scale-label" for="simulation-scale">仿真位移比例</label>
        <div class="simulation-scale-control">
          <input id="simulation-scale" data-field="simulation-scale" type="number" min="0.5" max="2" step="0.1" value="1.5">
          <span>: 1</span>
          <button type="button" data-action="apply-scale">应用</button>
        </div>
        <p data-field="scale-status">仅在 Grip 松开且机械臂停止时可修改</p>
        <button type="button" data-action="reset-view">复位视角</button>
      </section>`;
    this.input = requireElement(root, '[data-field="simulation-scale"]');
    this.apply = requireElement(root, '[data-action="apply-scale"]');
    this.label = requireElement(root, '[data-field="scale-label"]');
    this.status = requireElement(root, '[data-field="scale-status"]');
    this.input.addEventListener('input', () => {
      this.editing = true;
      this.statusOverride = null;
    });
    this.apply.addEventListener('click', () => this.request());
    requireElement<HTMLButtonElement>(root, '[data-action="reset-view"]')
      .addEventListener('click', onResetView);
  }

  update(state: SimulationSettingsState): void {
    this.state = state;
    if (!state.connected) {
      this.pending = false;
      this.editing = false;
    }
    const real = state.runtime === 'LEBAI';
    this.label.textContent = real ? '真机比例（配置只读）' : '仿真位移比例';
    const stopped =
      state.connected &&
      state.runtime === 'LEBAI_FAKE' &&
      state.backendState === 'IDLE' &&
      state.mode !== null &&
      ['READY', 'HOLD', 'DISARMED', 'ARMED'].includes(state.mode) &&
      !state.grip &&
      !state.automationActive;
    if (!stopped && !this.pending) this.editing = false;
    if (state.authoritativeScale !== null && !this.pending && !this.editing) {
      this.input.value = state.authoritativeScale.toFixed(1);
    }
    this.input.disabled = !stopped || this.pending;
    this.apply.disabled = !stopped || this.pending;
    this.status.textContent = real
      ? '真机比例由 YAML 配置，页面不可修改'
      : this.pending
        ? '等待仿真后端确认…'
        : this.statusOverride ?? '仅在 Grip 松开且机械臂停止时可修改';
  }

  handleResult(message: SimulationScaleResultMessage): void {
    this.pending = false;
    this.editing = false;
    this.input.value = message.translation_scale.toFixed(1);
    if (message.accepted) {
      localStorage.setItem(STORAGE_KEY, message.translation_scale.toFixed(1));
      this.statusOverride = `已应用 ${message.translation_scale.toFixed(1)} : 1`;
    } else {
      this.statusOverride = rejectionCopy(message.reason);
    }
    if (this.state) this.update({...this.state, authoritativeScale: message.translation_scale});
  }

  preferredScale(): number | null {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (stored === null) return null;
    const value = Number(stored);
    const scaled = Math.round(value * 10);
    return Number.isFinite(value) && scaled >= 5 && scaled <= 20 && Math.abs(value * 10 - scaled) <= 1e-9
      ? scaled / 10
      : null;
  }

  private request(): void {
    const value = Number(this.input.value);
    const scaled = Math.round(value * 10);
    if (!Number.isFinite(value) || scaled < 5 || scaled > 20 || Math.abs(value * 10 - scaled) > 1e-9) {
      this.statusOverride = '请输入 0.5–2.0，步进 0.1';
      this.status.textContent = this.statusOverride;
      return;
    }
    this.pending = true;
    this.editing = false;
    this.statusOverride = null;
    this.input.disabled = true;
    this.apply.disabled = true;
    this.status.textContent = '等待仿真后端确认…';
    this.onScaleRequest(scaled / 10);
  }
}

function rejectionCopy(reason: SimulationScaleRejectReason): string {
  return ({
    not_simulation: '真机比例只能通过 YAML 配置',
    not_stopped: '请先松开 Grip 并等待机械臂停止',
    invalid_scale: '比例必须为 0.5–2.0，步进 0.1',
    automation_active: '自动演练进行中不能修改比例',
  } as Record<string, string>)[reason] ?? '比例修改被拒绝';
}

function requireElement<T extends HTMLElement>(root: Element, selector: string): T {
  const element = root.querySelector<T>(selector);
  if (!element) throw new Error(`missing simulation settings element: ${selector}`);
  return element;
}
