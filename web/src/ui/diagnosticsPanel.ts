import type {
  DiagnosticsMessage,
  JointVector,
  Pose,
  RobotStateMessage,
  RuntimeBackend,
} from '../protocol/messages';

type LatencyView = Readonly<{currentMs: number | null; p95Ms: number | null}>;

type DiagnosticsRuntimeRender = (
  state: RobotStateMessage,
  diagnostics: DiagnosticsMessage | null,
  latency: LatencyView,
) => void;

export class DiagnosticsUpdateCoordinator {
  private state: RobotStateMessage | null = null;
  private diagnostics: DiagnosticsMessage | null = null;

  constructor(
    private readonly panel: Pick<DiagnosticsPanel, 'update' | 'clear'>,
    private readonly onRuntimeRender: DiagnosticsRuntimeRender,
    private readonly readLatency: () => LatencyView,
  ) {}

  onRobotState(
    state: RobotStateMessage,
    acknowledge: (sequence: number | null) => void,
  ): void {
    this.state = state;
    acknowledge(state.ack_seq ?? null);
    this.render();
  }

  onDiagnostics(diagnostics: DiagnosticsMessage): void {
    this.diagnostics = diagnostics;
    this.render();
  }

  clear(): void {
    this.state = null;
    this.diagnostics = null;
    this.panel.clear();
  }

  private render(): void {
    if (!this.state) return;
    const latency = this.readLatency();
    this.panel.update(this.state, this.diagnostics, latency);
    this.onRuntimeRender(this.state, this.diagnostics, latency);
  }
}

export class DiagnosticsPanel {
  constructor(private readonly root: HTMLElement) {
    this.root.classList.add('diagnostics-panel');
  }

  update(
    state: RobotStateMessage,
    diagnostics: DiagnosticsMessage | null,
    latency: LatencyView,
  ): void {
    const rail = this.root.closest<HTMLElement>('.status-rail');
    const railScrollTop = rail?.scrollTop ?? null;
    const eventScrollTop = this.root.querySelector<HTMLElement>(
      '.diagnostics-event-stream',
    )?.scrollTop ?? null;
    this.root.replaceChildren(
      this.renderIdentity(state, diagnostics),
      this.renderTcp(state.actual_tcp, diagnostics?.target_tcp ?? null),
      this.renderJoints(state.actual_q, diagnostics),
      this.renderLink(state, diagnostics, latency),
      this.renderSafety(state),
      this.renderEvents(diagnostics),
    );
    if (rail !== null && railScrollTop !== null) rail.scrollTop = railScrollTop;
    const refreshedEventStream = this.root.querySelector<HTMLElement>(
      '.diagnostics-event-stream',
    );
    if (refreshedEventStream !== null && eventScrollTop !== null) {
      refreshedEventStream.scrollTop = eventScrollTop;
    }
  }

  clear(): void {
    const card = cardElement('DIAGNOSTICS');
    appendMetric(card, 'Status', 'Diagnostics unavailable');
    this.root.replaceChildren(card);
  }

  private renderIdentity(
    state: RobotStateMessage,
    diagnostics: DiagnosticsMessage | null,
  ): HTMLElement {
    const runtime = diagnostics?.runtime ?? state.backend ?? null;
    const card = cardElement('RUNTIME', runtime === 'LEBAI_FAKE' ? 'healthy' : 'muted');
    appendMetric(card, 'Backend', runtime ?? 'awaiting state');
    appendMetric(card, 'Identity', runtimeIdentity(runtime));
    appendMetric(card, 'Mode', state.mode);
    appendMetric(card, 'Robot', state.robot_state);
    appendMetric(card, `hardware_verified=${String(diagnostics?.hardware_verified ?? false)}`, '');
    if (diagnostics) appendMetric(card, 'Generation', String(diagnostics.control_generation));
    return card;
  }

  private renderTcp(actual: Pose, target: Pose | null): HTMLElement {
    const card = cardElement('TCP', 'healthy');
    appendMetric(card, 'Actual', formatPose(actual));
    appendMetric(card, 'Target', target ? formatPose(target) : '—');
    return card;
  }

  private renderJoints(actualQ: JointVector, diagnostics: DiagnosticsMessage | null): HTMLElement {
    const card = cardElement('JOINTS');
    const table = document.createElement('div');
    table.className = 'diagnostics-joint-table';
    const header = document.createElement('div');
    header.className = 'diagnostics-joint-row diagnostics-joint-row--header';
    ['Joint', 'q', 'qd', 'qdd', 'target q'].forEach((label) => appendText(header, 'span', label));
    table.append(header);
    for (let index = 0; index < actualQ.length; index += 1) {
      const row = document.createElement('div');
      row.className = 'diagnostics-joint-row';
      appendText(row, 'span', `q${index + 1}`);
      appendText(row, 'span', formatNumber(actualQ[index]));
      appendText(row, 'span', formatNumber(diagnostics?.actual_qd?.[index] ?? null));
      appendText(row, 'span', formatNumber(diagnostics?.actual_qdd?.[index] ?? null));
      appendText(row, 'span', formatNumber(diagnostics?.target_q?.[index] ?? null));
      table.append(row);
    }
    card.append(table);
    return card;
  }

  private renderLink(
    state: RobotStateMessage,
    diagnostics: DiagnosticsMessage | null,
    latency: LatencyView,
  ): HTMLElement {
    const card = cardElement('LINK & CONTROL');
    appendMetric(card, 'WebSocket', `${formatMilliseconds(latency.currentMs)} / P95 ${formatMilliseconds(latency.p95Ms)}`);
    appendMetric(card, 'Sample age', formatMilliseconds(state.sample_age_ms ?? null));
    appendMetric(card, 'SDK', formatSdkLatencies(diagnostics?.sdk_latencies_ms ?? {}));
    appendMetric(card, 'IK', diagnostics?.target_tcp ? 'target available' : 'no target');
    appendMetric(card, 'PVAT', `PVAT ${formatRate(diagnostics?.pvat_send_hz ?? null)} Hz`);
    return card;
  }

  private renderSafety(state: RobotStateMessage): HTMLElement {
    const card = cardElement('SAFETY', state.fault ? 'fault' : state.constraint ? 'amber' : 'healthy');
    appendMetric(card, 'Preflight', formatPreflight(state.preflight_ready ?? null, state.preflight_reason ?? null));
    appendMetric(card, 'Fault', state.fault ?? 'none');
    appendMetric(card, 'Constraint', state.constraint ?? 'none');
    appendMetric(card, 'Recovery', state.recovery_phase ?? 'none');
    appendMetric(card, 'Gripper', `${Math.round(state.gripper * 100)}%`);
    return card;
  }

  private renderEvents(diagnostics: DiagnosticsMessage | null): HTMLElement {
    const card = cardElement('RECENT EVENTS');
    const events = diagnostics?.recent_events ?? [];
    const stream = document.createElement('ol');
    stream.className = 'diagnostics-event-stream';
    for (const event of events.slice(-20)) {
      const item = document.createElement('li');
      item.dataset.diagnosticEvent = String(event.event_id);
      item.dataset.critical = String(event.critical);
      item.textContent = `${event.event_id} · ${event.kind} · ${JSON.stringify(event.payload)}`;
      stream.append(item);
    }
    if (!events.length) appendText(stream, 'li', 'No diagnostics events yet');
    card.append(
      metricElement('Log directory', diagnostics?.log_session_dir ?? '—'),
      metricElement('Dropped events', String(diagnostics?.dropped_events ?? 0)),
      stream,
    );
    return card;
  }
}

function cardElement(title: string, tone: 'healthy' | 'amber' | 'fault' | 'muted' = 'muted'): HTMLElement {
  const card = document.createElement('section');
  card.className = 'diagnostics-card';
  card.dataset.tone = tone;
  appendText(card, 'h2', title);
  return card;
}

function appendMetric(parent: HTMLElement, label: string, value: string): void {
  parent.append(metricElement(label, value));
}

function metricElement(label: string, value: string): HTMLElement {
  const row = document.createElement('div');
  row.className = 'diagnostics-metric';
  appendText(row, 'span', label);
  appendText(row, 'strong', value);
  return row;
}

function appendText(parent: HTMLElement, tag: 'h2' | 'span' | 'strong' | 'li', value: string): void {
  const element = document.createElement(tag);
  element.textContent = value;
  parent.append(element);
}

function runtimeIdentity(runtime: RuntimeBackend | null): string {
  if (runtime === 'LEBAI_FAKE') return 'SIMULATION';
  if (runtime === 'LEBAI') return 'REAL ROBOT';
  if (runtime === 'SIMULATOR') return 'SIMULATOR';
  return 'awaiting state';
}

function formatPose(pose: Pose): string {
  return pose.p.map((value) => formatNumber(value, 3)).join(' / ');
}

function formatNumber(value: number | null, digits = 3): string {
  if (value === null || !Number.isFinite(value)) return '—';
  return value.toFixed(digits).replace('-', '−');
}

function formatMilliseconds(value: number | null): string {
  return value === null || !Number.isFinite(value) ? '—' : `${Math.round(value)} ms`;
}

function formatRate(value: number | null): string {
  return value === null || !Number.isFinite(value) ? '—' : value.toFixed(1);
}

function formatSdkLatencies(latencies: Readonly<Record<string, number>>): string {
  const entries = Object.entries(latencies);
  return entries.length
    ? entries.map(([name, value]) => `${name} ${formatMilliseconds(value)}`).join(' · ')
    : '—';
}

function formatPreflight(ready: boolean | null, reason: string | null): string {
  if (ready === null) return reason ?? 'not reported';
  return ready ? 'ready' : reason ?? 'not ready';
}
