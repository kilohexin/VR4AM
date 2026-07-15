import type {BackendState, TeleopMode} from '../protocol/messages';

export interface ControllerHudState {
  tracking: boolean;
  grip: boolean;
  trigger: number;
}

export interface RobotHudState {
  mode: TeleopMode;
  backendState: BackendState;
  sampleAgeMs: number | null;
  fault: string | null;
}

export class Hud {
  readonly sceneContainer: HTMLElement;
  readonly actionContainer: HTMLElement;

  private readonly modeValue: HTMLElement;
  private readonly backendStateValue: HTMLElement;
  private readonly connectionValue: HTMLElement;
  private readonly trackingValue: HTMLElement;
  private readonly gripValue: HTMLElement;
  private readonly gripShape: HTMLElement;
  private readonly triggerValue: HTMLElement;
  private readonly triggerFill: HTMLElement;
  private readonly sampleAgeValue: HTMLElement;
  private readonly currentLatencyValue: HTMLElement;
  private readonly p95LatencyValue: HTMLElement;
  private readonly faultValue: HTMLElement;
  private readonly faultRow: HTMLElement;
  private readonly sceneNotice: HTMLElement;

  constructor(root: Element) {
    root.classList.add('operator-console');
    root.innerHTML = `
      <header class="top-bar">
        <div class="brand-lockup">
          <h1>LM3 遥操作仿真</h1>
          <div class="simulator-label">仅仿真 · SIMULATOR</div>
        </div>
        <div class="command-actions-host"></div>
      </header>
      <main class="console-main">
        <section class="simulation-viewport" aria-label="LM3 三维仿真场景">
          <div class="scene-canvas"></div>
          <div class="scene-notice" role="status" hidden></div>
        </section>
        <aside class="status-rail" aria-label="遥操作安全状态">
          <div class="mode-heading" data-tone="muted">
            ${icon('shield')}
            <strong data-field="mode">DISCONNECTED · 未连接</strong>
          </div>
          ${statusRow('robot', '机器人', '<span data-field="backend-state">DISCONNECTED</span>')}
          ${statusRow('link', '连接', '<span data-field="connection">未连接</span>')}
          ${statusRow('target', '追踪', '<span data-field="tracking">无效</span>')}
          <div class="status-row" data-row="grip">
            ${icon('hand')}
            <span class="status-label">Grip</span>
            <span class="status-value" data-field="grip">松开</span>
            <span class="grip-shape" data-field="grip-shape" aria-hidden="true"></span>
          </div>
          <div class="status-row trigger-row" data-row="trigger">
            ${icon('trigger')}
            <span class="status-label">Trigger</span>
            <span class="status-value" data-field="trigger">0%</span>
            <span class="trigger-meter" aria-hidden="true"><span data-field="trigger-fill"></span></span>
          </div>
          ${statusRow('clock', '样本年龄', '<span data-field="sample-age">— ms</span>')}
          <div class="status-row latency-row" data-row="latency">
            ${icon('gauge')}
            <span class="status-label">延迟</span>
            <span class="latency-values"><span data-field="latency-current">— ms</span><small>P95</small><span data-field="latency-p95">— ms</span></span>
          </div>
          <div class="status-row fault-row" data-row="fault">
            ${icon('warning')}
            <span class="status-label">故障</span>
            <span class="status-value" data-field="fault">无</span>
          </div>
        </aside>
      </main>
      <footer class="help-strip">
        ${icon('info')}
        <span>按住右手 Grip 建立锚点并移动 · Trigger 控制夹爪 · 松开 Grip 停止</span>
      </footer>
    `;

    this.sceneContainer = requireElement(root, '.scene-canvas');
    this.actionContainer = requireElement(root, '.command-actions-host');
    this.modeValue = requireElement(root, '[data-field="mode"]');
    this.backendStateValue = requireElement(root, '[data-field="backend-state"]');
    this.connectionValue = requireElement(root, '[data-field="connection"]');
    this.trackingValue = requireElement(root, '[data-field="tracking"]');
    this.gripValue = requireElement(root, '[data-field="grip"]');
    this.gripShape = requireElement(root, '[data-field="grip-shape"]');
    this.triggerValue = requireElement(root, '[data-field="trigger"]');
    this.triggerFill = requireElement(root, '[data-field="trigger-fill"]');
    this.sampleAgeValue = requireElement(root, '[data-field="sample-age"]');
    this.currentLatencyValue = requireElement(root, '[data-field="latency-current"]');
    this.p95LatencyValue = requireElement(root, '[data-field="latency-p95"]');
    this.faultValue = requireElement(root, '[data-field="fault"]');
    this.faultRow = requireElement(root, '[data-row="fault"]');
    this.sceneNotice = requireElement(root, '.scene-notice');
  }

  setConnection(connected: boolean): void {
    this.connectionValue.textContent = connected ? '已连接' : '未连接';
    this.connectionValue.dataset.tone = connected ? 'healthy' : 'muted';
    if (!connected) {
      this.modeValue.textContent = 'DISCONNECTED · 未连接';
      this.modeValue.parentElement?.setAttribute('data-tone', 'muted');
      this.backendStateValue.textContent = 'DISCONNECTED';
      this.backendStateValue.dataset.tone = 'muted';
      this.sampleAgeValue.textContent = formatMilliseconds(null);
      this.setLatency(null, null);
      this.faultValue.textContent = readableFault(null);
      this.faultRow.dataset.active = 'false';
      this.setController({tracking: false, grip: false, trigger: 0});
    }
  }

  setController(state: ControllerHudState): void {
    const trigger = clamp01(state.trigger);
    this.trackingValue.textContent = state.tracking ? '有效' : '无效';
    this.trackingValue.dataset.tone = state.tracking ? 'healthy' : 'muted';
    this.gripValue.textContent = state.grip ? '按住' : '松开';
    this.gripValue.dataset.tone = state.grip ? 'active' : 'healthy';
    this.gripShape.dataset.grip = state.grip ? 'closed' : 'open';
    this.triggerValue.textContent = `${Math.round(trigger * 100)}%`;
    this.triggerFill.style.height = `${trigger * 100}%`;
  }

  setRobotState(state: RobotHudState): void {
    this.modeValue.textContent = `${state.mode} · ${modeLabel(state.mode)}`;
    this.modeValue.parentElement?.setAttribute('data-tone', modeTone(state.mode));
    this.backendStateValue.textContent = state.backendState;
    this.backendStateValue.dataset.tone = state.backendState === 'FAULT' ? 'fault' : 'healthy';
    this.sampleAgeValue.textContent = formatMilliseconds(state.sampleAgeMs);
    this.faultValue.textContent = readableFault(state.fault);
    this.faultRow.dataset.active = state.fault ? 'true' : 'false';
  }

  setLatency(current: number | null, p95: number | null): void {
    this.currentLatencyValue.textContent = formatMilliseconds(current);
    this.p95LatencyValue.textContent = formatMilliseconds(p95);
  }

  showSceneError(message: string): void {
    this.sceneNotice.textContent = message;
    this.sceneNotice.hidden = false;
  }

  clearSceneError(): void {
    this.sceneNotice.textContent = '';
    this.sceneNotice.hidden = true;
  }
}

function statusRow(iconName: IconName, label: string, value: string): string {
  return `<div class="status-row">${icon(iconName)}<span class="status-label">${label}</span><span class="status-value">${value}</span></div>`;
}

function requireElement<T extends HTMLElement = HTMLElement>(root: Element, selector: string): T {
  const element = root.querySelector<T>(selector);
  if (!element) throw new Error(`界面缺少元素: ${selector}`);
  return element;
}

function clamp01(value: number): number {
  return Math.min(1, Math.max(0, value));
}

function formatMilliseconds(value: number | null): string {
  return value === null || !Number.isFinite(value) ? '— ms' : `${Math.round(value)} ms`;
}

function modeLabel(mode: TeleopMode): string {
  const labels: Record<TeleopMode, string> = {
    DISCONNECTED: '未连接',
    READY: '等待解锁',
    ARMED: '已解锁',
    ACTIVE: '遥操作中',
    HOLD: '保持',
    STALE: '数据陈旧',
    FAULT: '故障',
    DISARMED: '已停止',
  };
  return labels[mode];
}

function modeTone(mode: TeleopMode): string {
  if (mode === 'FAULT') return 'fault';
  if (mode === 'DISCONNECTED' || mode === 'STALE' || mode === 'DISARMED') return 'muted';
  return 'healthy';
}

function readableFault(fault: string | null): string {
  if (!fault) return '无';
  const labels: Record<string, string> = {
    protocol_error: '协议消息无效',
    tracking_lost: '追踪已丢失',
    input_stale: '控制输入已超时',
    control_overrun: '控制周期连续超时',
    invalid_numeric: '控制数据包含无效数值',
    workspace_violation: '目标超出工作空间',
    joint_limit: '目标超出关节限制',
    joint_safety_window: '目标超出仿真关节安全范围',
    invalid_joint_count: '机器人关节数据无效',
    ik_unreachable: '目标不可达',
    ik_singular: '目标接近奇异位形',
    backend_disconnected: '仿真后端已断开',
    backend_fault: '仿真后端故障',
    real_robot_disabled: '第一里程碑禁用真机',
    stale_frame: '控制帧超时',
    backend_error: '仿真后端错误',
  };
  return labels[fault] ?? '未知仿真故障';
}

type IconName = 'shield' | 'robot' | 'link' | 'target' | 'hand' | 'trigger' | 'clock' | 'gauge' | 'warning' | 'info';

function icon(name: IconName): string {
  const paths: Record<IconName, string> = {
    shield: '<path d="M12 2 20 5v6c0 5.1-3.3 8.7-8 11-4.7-2.3-8-5.9-8-11V5l8-3Z"/><path d="m8.5 11.5 2.2 2.2 4.8-5"/>',
    robot: '<path d="M6 20h12M8 20v-6l3-3 4 2 2-5"/><circle cx="17.5" cy="6" r="2"/><path d="m10 12-2-4 4-3 3 2"/>',
    link: '<path d="m9 15-2 2a4 4 0 0 1-6-6l3-3a4 4 0 0 1 6 0M15 9l2-2a4 4 0 0 1 6 6l-3 3a4 4 0 0 1-6 0M8 16l8-8"/>',
    target: '<circle cx="12" cy="12" r="7"/><circle cx="12" cy="12" r="2"/><path d="M12 1v4M12 19v4M1 12h4M19 12h4"/>',
    hand: '<path d="M7 12V6a1.5 1.5 0 0 1 3 0v5-7a1.5 1.5 0 0 1 3 0v7-6a1.5 1.5 0 0 1 3 0v7-4a1.5 1.5 0 0 1 3 0v7c0 5-3 8-8 8-3 0-5-1.6-6.5-4L2 15a1.7 1.7 0 0 1 2.7-2Z"/>',
    trigger: '<path d="M3 7h8l2 2h6l2 2-2 4h-6l-2-3H7V9H3Z"/><path d="m12 13 2 7h-4l-2-8"/>',
    clock: '<circle cx="12" cy="12" r="9"/><path d="M12 6v6l4 2"/>',
    gauge: '<circle cx="12" cy="12" r="9"/><path d="M5 15a8 8 0 0 1 14 0M12 12l4-4"/>',
    warning: '<path d="m12 3 10 18H2L12 3Z"/><path d="M12 9v5M12 18v.1"/>',
    info: '<circle cx="12" cy="12" r="9"/><path d="M12 11v6M12 7v.1"/>',
  };
  return `<svg class="line-icon line-icon--${name}" aria-hidden="true" viewBox="0 0 24 24">${paths[name]}</svg>`;
}
