import type {
  BackendState,
  ConstraintKind,
  RealRobotMode,
  RecoveryPhase,
  RuntimeBackend,
  TeleopMode,
} from '../protocol/messages';
import type {TeleopConnectionStatus} from '../transport/teleopSocket';

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
  constraint: ConstraintKind | null;
  recoveryPhase: RecoveryPhase | null;
}

export class Hud {
  readonly sceneContainer: HTMLElement;
  readonly actionContainer: HTMLElement;
  readonly rehearsalBanner: HTMLElement;
  readonly rehearsalContainer: HTMLElement;
  readonly diagnosticsContainer: HTMLElement;

  private readonly modeValue: HTMLElement;
  private readonly runtimeIdentityValue: HTMLElement;
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
  private readonly constraintValue: HTMLElement;
  private readonly constraintRow: HTMLElement;
  private readonly sceneNotice: HTMLElement;

  constructor(root: Element) {
    root.classList.add('operator-console');
    if (root instanceof HTMLElement) root.style.paddingTop = '44px';
    root.innerHTML = `
      <header class="top-bar">
        <div class="brand-lockup">
          <h1>LM3 遥操作仿真</h1>
          <div class="simulator-label">仅仿真 · SIMULATOR</div>
        </div>
        <div class="command-actions-host"></div>
      </header>
      <div class="offline-rehearsal-banner" role="note" style="position: fixed; top: 0; left: 0; right: 0; z-index: 20;">仿真模式，不是真机</div>
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
          <div class="status-row constraint-row" data-row="constraint">
            ${icon('target')}
            <span class="status-label">边界/恢复</span>
            <span class="status-value" data-field="constraint">无</span>
          </div>
          <div class="offline-rehearsal-rail" aria-label="离线演练控制台"></div>
          <div class="diagnostics-rail" aria-label="LM3 diagnostics"></div>
        </aside>
      </main>
      <footer class="help-strip">
        ${icon('info')}
        <span>右 Grip 建立末端零位 · Trigger 控制夹爪 · 左摇杆平移工作台 · 左 Grip+摇杆调高度</span>
      </footer>
    `;

    this.sceneContainer = requireElement(root, '.scene-canvas');
    this.actionContainer = requireElement(root, '.command-actions-host');
    this.rehearsalBanner = requireElement(root, '.offline-rehearsal-banner');
    this.rehearsalContainer = requireElement(root, '.offline-rehearsal-rail');
    this.diagnosticsContainer = requireElement(root, '.diagnostics-rail');
    this.modeValue = requireElement(root, '[data-field="mode"]');
    this.runtimeIdentityValue = requireElement(root, '.simulator-label');
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
    this.constraintValue = requireElement(root, '[data-field="constraint"]');
    this.constraintRow = requireElement(root, '[data-row="constraint"]');
    this.sceneNotice = requireElement(root, '.scene-notice');
  }

  setConnectionStatus(status: TeleopConnectionStatus): void {
    const labels: Record<TeleopConnectionStatus['state'], string> = {
      connected: '已连接',
      occupied: '控制端已被占用，请关闭电脑端网页后重试',
      unreachable: '后端不可达',
      disconnected: '连接已中断',
      reconnecting: '正在重连',
    };
    this.connectionValue.textContent = labels[status.state];
    this.connectionValue.dataset.tone = status.state === 'connected' ? 'healthy' : 'muted';
    if (status.state !== 'connected') {
      this.setRuntimeIdentity(null, null);
      this.modeValue.textContent = 'DISCONNECTED · 未连接';
      this.modeValue.parentElement?.setAttribute('data-tone', 'muted');
      this.backendStateValue.textContent = 'DISCONNECTED';
      this.backendStateValue.dataset.tone = 'muted';
      this.sampleAgeValue.textContent = formatMilliseconds(null);
      this.setLatency(null, null);
      this.faultValue.textContent = readableFault(null);
      this.faultRow.dataset.active = 'false';
      this.constraintValue.textContent = '无';
      this.constraintRow.dataset.active = 'false';
      this.setController({tracking: false, grip: false, trigger: 0});
    }
  }

  setRuntimeIdentity(
    backend: RuntimeBackend | null,
    realMode: RealRobotMode | null,
  ): void {
    this.runtimeIdentityValue.textContent = runtimeIdentityLabel(backend, realMode);
    this.runtimeIdentityValue.dataset.backend = backend ?? 'SIMULATOR';
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
    const displayMode: TeleopMode = state.fault ? 'FAULT' : state.mode;
    this.modeValue.textContent = `${displayMode} · ${modeLabel(displayMode)}`;
    this.modeValue.parentElement?.setAttribute('data-tone', modeTone(displayMode));
    this.backendStateValue.textContent = state.backendState;
    this.backendStateValue.dataset.tone = state.backendState === 'FAULT' ? 'fault' : 'healthy';
    this.sampleAgeValue.textContent = formatMilliseconds(state.sampleAgeMs);
    this.faultValue.textContent = readableFault(state.fault);
    this.faultRow.dataset.active = state.fault ? 'true' : 'false';
    this.constraintValue.textContent = state.recoveryPhase
      ? readableRecoveryPhase(state.recoveryPhase)
      : readableConstraint(state.constraint);
    this.constraintRow.dataset.active = state.recoveryPhase || state.constraint ? 'true' : 'false';
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

export function readableFault(fault: string | null): string {
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
    ik_joint_jump: '逆解关节变化过大',
    ik_failure_persistent: '逆解连续失败',
    stop_unverified: '停止状态未确认',
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

function runtimeIdentityLabel(backend: RuntimeBackend | null, realMode: RealRobotMode | null): string {
  if (backend === 'LEBAI_FAKE') return '仿真 · LEBAI_FAKE';
  if (backend === 'LEBAI') {
    return realMode === 'control' ? '真机控制 · LEBAI' : '真机只读 · LEBAI';
  }
  return '仅仿真 · SIMULATOR';
}

export function readableConstraint(constraint: ConstraintKind | null): string {
  if (!constraint) return '无';
  return {
    workspace_boundary: '已到达操作边界，保持 Grip 向反方向退回',
    ik_boundary: '当前姿态暂不可达，保持 Grip 退回上一个位置',
    joint_boundary: '已到达关节操作边界，保持 Grip 反向退回',
    self_collision: '机械臂接近自碰撞边界，请将手柄退回',
  }[constraint];
}

function readableRecoveryPhase(phase: RecoveryPhase): string {
  return {
    stopping: '正在确认停止',
    homing: '正在回到初始姿态',
    stabilizing: '正在确认 Home 稳定',
  }[phase];
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
