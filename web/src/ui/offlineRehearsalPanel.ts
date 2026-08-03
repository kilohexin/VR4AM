import type {
  BackendState,
  ConstraintKind,
  Pose,
  RuntimeBackend,
  TeleopMode,
} from '../protocol/messages';
import type {OfflineRehearsalSnapshot} from '../rehearsal/offlineRehearsalController';
import {REHEARSAL_PHASES} from '../rehearsal/types';
import {quaternionAngularError} from '../rehearsal/trajectory';
import type {XRSessionStatus} from '../xr/session';

export interface OfflineRehearsalEligibility {
  connected: boolean;
  runtime: RuntimeBackend | null;
  robotRuntime: RuntimeBackend | null;
  hardwareVerified: false | null;
  mode: TeleopMode | null;
  backendState: BackendState | null;
  fault: string | null;
  constraint: ConstraintKind | null;
  actualTcp: Pose | null;
  xrState: XRSessionStatus['state'];
}

export class OfflineRehearsalPanel {
  readonly startButton: HTMLButtonElement;
  readonly stopButton: HTMLButtonElement;

  private readonly phaseValue: HTMLElement;
  private readonly stepValue: HTMLElement;
  private readonly progressValue: HTMLElement;
  private readonly errorValue: HTMLElement;
  private readonly timeoutValue: HTMLElement;
  private readonly failureValue: HTMLElement;
  private readonly jsonPathValue: HTMLElement;
  private readonly markdownPathValue: HTMLElement;
  private readonly hardwareList: HTMLElement;

  constructor(
    container: Element,
    onStart: () => void,
    onStop: () => void,
  ) {
    container.classList.add('offline-rehearsal-host');
    container.innerHTML = `
      <section class="offline-rehearsal-panel" aria-labelledby="offline-rehearsal-heading">
        <h2 id="offline-rehearsal-heading">离线自动演练</h2>
        <div class="offline-rehearsal-actions">
          <button type="button" data-action="start">开始演练</button>
          <button type="button" data-action="stop" disabled>停止演练</button>
        </div>
        <dl class="offline-rehearsal-status">
          <div><dt>当前阶段</dt><dd data-field="rehearsal-phase">idle</dd></div>
          <div><dt>当前步骤</dt><dd data-field="rehearsal-step">—</dd></div>
          <div><dt>进度</dt><dd data-field="rehearsal-progress">0 / ${REHEARSAL_PHASES.length}</dd></div>
          <div><dt>目标/实际误差</dt><dd data-field="rehearsal-error">—</dd></div>
          <div><dt>剩余超时</dt><dd data-field="rehearsal-timeout">—</dd></div>
          <div><dt>首次失败</dt><dd data-field="rehearsal-failure">—</dd></div>
          <div><dt>JSON 报告</dt><dd data-field="rehearsal-json">—</dd></div>
          <div><dt>Markdown 报告</dt><dd data-field="rehearsal-markdown">—</dd></div>
        </dl>
        <div class="offline-rehearsal-hardware">
          <strong>真机待验证（8 项）</strong>
          <ul data-field="hardware-pending"></ul>
        </div>
      </section>
    `;

    this.startButton = requireButton(container, '[data-action="start"]');
    this.stopButton = requireButton(container, '[data-action="stop"]');
    this.phaseValue = requireElement(container, '[data-field="rehearsal-phase"]');
    this.stepValue = requireElement(container, '[data-field="rehearsal-step"]');
    this.progressValue = requireElement(container, '[data-field="rehearsal-progress"]');
    this.errorValue = requireElement(container, '[data-field="rehearsal-error"]');
    this.timeoutValue = requireElement(container, '[data-field="rehearsal-timeout"]');
    this.failureValue = requireElement(container, '[data-field="rehearsal-failure"]');
    this.jsonPathValue = requireElement(container, '[data-field="rehearsal-json"]');
    this.markdownPathValue = requireElement(container, '[data-field="rehearsal-markdown"]');
    this.hardwareList = requireElement(container, '[data-field="hardware-pending"]');
    this.startButton.addEventListener('click', onStart);
    this.stopButton.addEventListener('click', onStop);
  }

  update(
    snapshot: Readonly<OfflineRehearsalSnapshot>,
    eligibility: Readonly<OfflineRehearsalEligibility>,
  ): void {
    this.startButton.disabled = snapshot.active || !canStart(eligibility);
    this.stopButton.disabled = !snapshot.active;
    this.phaseValue.textContent = snapshot.currentPhase ?? snapshot.phase;
    this.stepValue.textContent = snapshot.step ?? '—';
    this.progressValue.textContent = `${snapshot.completedPhases.length} / ${REHEARSAL_PHASES.length}`;
    this.errorValue.textContent = formatPoseError(snapshot.targetTcp, eligibility.actualTcp);
    this.timeoutValue.textContent = snapshot.remainingTimeoutMs === null
      ? '—'
      : `${(snapshot.remainingTimeoutMs / 1_000).toFixed(1)} s`;
    this.failureValue.textContent = snapshot.failure ?? '—';
    this.failureValue.dataset.active = snapshot.failure === null ? 'false' : 'true';
    this.jsonPathValue.textContent = snapshot.reportPaths?.json ?? '—';
    this.markdownPathValue.textContent = snapshot.reportPaths?.markdown ?? '—';
    this.hardwareList.replaceChildren(...snapshot.hardwarePending.map((check) => {
      const item = document.createElement('li');
      item.dataset.hardwareCheck = check;
      item.textContent = check;
      return item;
    }));
  }
}

function canStart(eligibility: Readonly<OfflineRehearsalEligibility>): boolean {
  return eligibility.connected
    && eligibility.xrState === 'idle'
    && eligibility.runtime === 'LEBAI_FAKE'
    && eligibility.robotRuntime === 'LEBAI_FAKE'
    && eligibility.hardwareVerified === false
    && eligibility.mode === 'READY'
    && eligibility.backendState === 'IDLE'
    && eligibility.fault === null
    && eligibility.constraint === null;
}

function formatPoseError(target: Pose | null, actual: Pose | null): string {
  if (target === null || actual === null) return '—';
  const positionErrorMm = Math.hypot(
    target.p[0] - actual.p[0],
    target.p[1] - actual.p[1],
    target.p[2] - actual.p[2],
  ) * 1_000;
  const rotationErrorDeg = quaternionAngularError(target.q, actual.q) * 180 / Math.PI;
  return `${positionErrorMm.toFixed(1)} mm / ${rotationErrorDeg.toFixed(1)}°`;
}

function requireElement<T extends HTMLElement>(container: Element, selector: string): T {
  const element = container.querySelector<T>(selector);
  if (!element) throw new Error(`缺少离线演练界面元素: ${selector}`);
  return element;
}

function requireButton(container: Element, selector: string): HTMLButtonElement {
  return requireElement<HTMLButtonElement>(container, selector);
}
