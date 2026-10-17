// @vitest-environment jsdom
import {beforeEach, describe, expect, it, vi} from 'vitest';
import validDiagnostics from '../../schemas/fixtures/diagnostics-valid.json';
import robotFixture from '../../schemas/fixtures/robot-state-valid.json';
import type {DiagnosticsMessage, RobotStateMessage} from '../src/protocol/messages';
import {DiagnosticsPanel, DiagnosticsUpdateCoordinator} from '../src/ui/diagnosticsPanel';

beforeEach(() => {
  document.body.innerHTML = '<aside id="diagnostics"></aside>';
});

describe('DiagnosticsUpdateCoordinator cadence', () => {
  const state = robotFixture as RobotStateMessage;
  const diagnostics = validDiagnostics as unknown as DiagnosticsMessage;

  function setup() {
    let now = 0;
    const panel = {update: vi.fn(), clear: vi.fn()};
    const runtime = vi.fn();
    const acknowledge = vi.fn();
    const coordinator = new DiagnosticsUpdateCoordinator(
      panel, runtime, () => ({currentMs: 18, p95Ms: 27}), () => now,
    );
    return {panel, runtime, acknowledge, coordinator, at: (time: number) => {now = time;}};
  }

  it('coalesces 50 Hz details to 5 Hz while processing every acknowledgement and summary', () => {
    const {panel, runtime, acknowledge, coordinator, at} = setup();
    for (let i = 0; i < 50; i++) {
      at(i * 20);
      coordinator.onRobotState({...state, ack_seq: i, gripper: i / 100}, acknowledge);
    }
    expect(panel.update).toHaveBeenCalledTimes(5);
    expect(panel.update.mock.calls.map(([s]) => s.ack_seq)).toEqual([0, 10, 20, 30, 40]);
    expect(acknowledge).toHaveBeenCalledTimes(50);
    expect(acknowledge).toHaveBeenLastCalledWith(49);
    expect(runtime).toHaveBeenCalledTimes(50);
    expect(runtime.mock.calls.at(-1)?.[0].gripper).toBe(0.49);
    expect(runtime.mock.invocationCallOrder[0]).toBeLessThan(panel.update.mock.invocationCallOrder[0]);
  });

  it.each<Partial<RobotStateMessage>>([
    {mode: 'FAULT'}, {robot_state: 'MOVING'}, {fault: 'stop_unverified'},
    {constraint: 'workspace_boundary'}, {recovery_phase: 'stopping'},
    {backend: 'LEBAI'}, {real_robot_mode: 'control'},
    {manual_input_ready: true}, {preparation_required: true}, {preparation_unlocked: true},
    {preflight_ready: false}, {preflight_reason: 'feedback_stale'},
    {target_control: {policy: 'segmented_experimental', phase: 'fault', samples_observed: 0,
      samples_required: 2, hardware_verified: false}},
  ])('immediately renders safety/readiness changes and their clearance: %j', (change) => {
    const {panel, coordinator, acknowledge, at} = setup();
    coordinator.onRobotState(state, acknowledge);
    at(10);
    coordinator.onRobotState({...state, ...change}, acknowledge);
    at(20);
    coordinator.onRobotState(state, acknowledge);
    expect(panel.update).toHaveBeenCalledTimes(3);
  });

  it('keeps the newest diagnostic data and refreshes runtime identity immediately', () => {
    const {panel, runtime, coordinator, acknowledge, at} = setup();
    coordinator.onRobotState(state, acknowledge);
    at(50);
    coordinator.onDiagnostics({...diagnostics, runtime: 'LEBAI_FAKE', dropped_events: 1});
    const afterIdentity = panel.update.mock.calls.length;
    at(60);
    const newest = {...diagnostics, runtime: 'LEBAI_FAKE' as const, dropped_events: 2};
    coordinator.onDiagnostics(newest);
    expect(panel.update).toHaveBeenCalledTimes(afterIdentity);
    expect(runtime).toHaveBeenCalledTimes(3);
    at(250);
    coordinator.onRobotState(state, acknowledge);
    expect(panel.update).toHaveBeenLastCalledWith(state, newest, {currentMs: 18, p95Ms: 27});
    at(260);
    coordinator.onDiagnostics({...newest, runtime: 'LEBAI'});
    expect(panel.update).toHaveBeenCalledTimes(afterIdentity + 2);
  });

  it('clears connection data and gives the next connection an immediate first render', () => {
    const {panel, coordinator, acknowledge, at} = setup();
    coordinator.onRobotState(state, acknowledge);
    coordinator.onDiagnostics(diagnostics);
    coordinator.clear();
    const before = panel.update.mock.calls.length;
    at(10);
    coordinator.onDiagnostics(diagnostics);
    expect(panel.update).toHaveBeenCalledTimes(before);
    coordinator.clear();
    coordinator.onRobotState(state, acknowledge);
    expect(panel.clear).toHaveBeenCalledTimes(2);
    expect(panel.update).toHaveBeenLastCalledWith(state, null, {currentMs: 18, p95Ms: 27});
    expect(panel.update).toHaveBeenCalledTimes(before + 1);
  });

  it.each([-1, Number.NaN, Number.POSITIVE_INFINITY])('does not freeze details for invalid/backwards clock %s', (time) => {
    const {panel, coordinator, acknowledge, at} = setup();
    coordinator.onRobotState(state, acknowledge);
    at(time);
    coordinator.onRobotState(state, acknowledge);
    at(20);
    coordinator.onRobotState(state, acknowledge);
    expect(panel.update.mock.calls.length).toBeGreaterThanOrEqual(2);
  });
});

describe('DiagnosticsPanel', () => {
  it('renders authoritative real-path diagnostics and recent events', () => {
    const root = document.querySelector<HTMLElement>('#diagnostics')!;
    const panel = new DiagnosticsPanel(root);

    panel.update(
      robotFixture as RobotStateMessage,
      validDiagnostics as unknown as DiagnosticsMessage,
      {currentMs: 18, p95Ms: 27},
    );

    expect(root.textContent).toContain('LEBAI_FAKE');
    expect(root.textContent).toContain('SIMULATION');
    expect(root.textContent).toContain('TCP');
    expect(root.textContent).toContain('q1');
    expect(root.textContent).toContain('PVAT 25.0 Hz');
    expect(root.textContent).toContain('pvat_sent');
    expect(root.textContent).toContain('hardware_verified=false');
    expect(root.textContent).toContain('Dropped events0');
    expect(root.textContent).toContain('Log directory—');
    expect(root.textContent).toContain('VR input age');
    expect(root.textContent).not.toContain('Sample age');
  });

  it('renders only the twenty events supplied by the bounded backend message', () => {
    const root = document.querySelector<HTMLElement>('#diagnostics')!;
    const diagnosticsWithTwentyEvents: DiagnosticsMessage = {
      ...(validDiagnostics as unknown as DiagnosticsMessage),
      recent_events: Array.from({length: 20}, (_, index) => ({
        event_id: index + 1,
        server_mono_ns: 900_000_000 + index,
        kind: 'pvat_sent',
        critical: false,
        payload: {command_id: index},
      })),
    };
    const panel = new DiagnosticsPanel(root);

    panel.update(
      robotFixture as RobotStateMessage,
      diagnosticsWithTwentyEvents,
      {currentMs: null, p95Ms: null},
    );

    expect(root.querySelectorAll('[data-diagnostic-event]')).toHaveLength(20);
  });

  it('renders event payloads as text instead of executable HTML', () => {
    const root = document.querySelector<HTMLElement>('#diagnostics')!;
    const panel = new DiagnosticsPanel(root);
    const diagnostics: DiagnosticsMessage = {
      ...(validDiagnostics as unknown as DiagnosticsMessage),
      recent_events: [{
        event_id: 1,
        server_mono_ns: 1,
        kind: 'external_payload',
        critical: true,
        payload: {detail: '<img src=x onerror=alert(1)>'},
      }],
    };

    panel.update(robotFixture as RobotStateMessage, diagnostics, {currentMs: null, p95Ms: null});

    expect(root.textContent).toContain('<img src=x onerror=alert(1)>');
    expect(root.querySelector('img')).toBeNull();
  });

  it('preserves operator scroll positions while live diagnostics refresh', () => {
    document.body.innerHTML = `
      <aside class="status-rail">
        <div id="diagnostics"></div>
      </aside>
    `;
    const rail = document.querySelector<HTMLElement>('.status-rail')!;
    const root = document.querySelector<HTMLElement>('#diagnostics')!;
    const panel = new DiagnosticsPanel(root);
    const diagnostics = validDiagnostics as unknown as DiagnosticsMessage;
    panel.update(robotFixture as RobotStateMessage, diagnostics, {currentMs: 18, p95Ms: 27});
    const originalStream = root.querySelector<HTMLElement>('.diagnostics-event-stream')!;
    rail.scrollTop = 320;
    originalStream.scrollTop = 90;

    panel.update(robotFixture as RobotStateMessage, diagnostics, {currentMs: 19, p95Ms: 28});

    const refreshedStream = root.querySelector<HTMLElement>('.diagnostics-event-stream')!;
    expect(refreshedStream).toBe(originalStream);
    expect(rail.scrollTop).toBe(320);
    expect(refreshedStream.scrollTop).toBe(90);
  });

  it('does not rebuild diagnostics while the operator is reading below the top', () => {
    document.body.innerHTML = `
      <aside class="status-rail">
        <div id="diagnostics"></div>
      </aside>
    `;
    const rail = document.querySelector<HTMLElement>('.status-rail')!;
    const root = document.querySelector<HTMLElement>('#diagnostics')!;
    const panel = new DiagnosticsPanel(root);
    const diagnostics = validDiagnostics as unknown as DiagnosticsMessage;
    panel.update(robotFixture as RobotStateMessage, diagnostics, {currentMs: 18, p95Ms: 27});
    const originalCard = root.firstElementChild;
    const originalText = root.textContent;
    rail.scrollTop = 320;

    panel.update(
      {...robotFixture, mode: 'ACTIVE'} as RobotStateMessage,
      diagnostics,
      {currentMs: 99, p95Ms: 120},
    );

    expect(root.firstElementChild).toBe(originalCard);
    expect(root.textContent).toBe(originalText);
    expect(rail.scrollTop).toBe(320);

    rail.scrollTop = 0;
    panel.update(
      {...robotFixture, mode: 'ACTIVE'} as RobotStateMessage,
      diagnostics,
      {currentMs: 99, p95Ms: 120},
    );
    expect(root.firstElementChild).not.toBe(originalCard);
    expect(root.textContent).toContain('ACTIVE');
    expect(root.textContent).toContain('99 ms');
  });

  it('does not rebuild diagnostics while the operator is scrolling recent events', () => {
    document.body.innerHTML = `
      <aside class="status-rail">
        <div id="diagnostics"></div>
      </aside>
    `;
    const rail = document.querySelector<HTMLElement>('.status-rail')!;
    const root = document.querySelector<HTMLElement>('#diagnostics')!;
    const panel = new DiagnosticsPanel(root);
    const diagnostics = validDiagnostics as unknown as DiagnosticsMessage;
    panel.update(robotFixture as RobotStateMessage, diagnostics, {currentMs: 18, p95Ms: 27});
    const originalStream = root.querySelector<HTMLElement>('.diagnostics-event-stream')!;
    rail.scrollTop = 0;
    originalStream.scrollTop = 90;

    panel.update(robotFixture as RobotStateMessage, diagnostics, {currentMs: 19, p95Ms: 28});

    expect(root.querySelector<HTMLElement>('.diagnostics-event-stream')).toBe(originalStream);
    expect(originalStream.scrollTop).toBe(90);
  });

  it('clears stale diagnostics when the owner connection is no longer usable', () => {
    const root = document.querySelector<HTMLElement>('#diagnostics')!;
    const panel = new DiagnosticsPanel(root);
    panel.update(
      robotFixture as RobotStateMessage,
      validDiagnostics as unknown as DiagnosticsMessage,
      {currentMs: 18, p95Ms: 27},
    );

    panel.clear();

    expect(root.textContent).toContain('Diagnostics unavailable');
    expect(root.textContent).not.toContain('pvat_sent');
    expect(root.textContent).not.toContain('LEBAI_FAKE');
  });

  it('renders diagnostics once for an acknowledged RobotState callback', () => {
    const panel = {update: vi.fn(), clear: vi.fn()};
    const runtimeUpdate = vi.fn();
    const coordinator = new DiagnosticsUpdateCoordinator(
      panel,
      runtimeUpdate,
      () => ({currentMs: 18, p95Ms: 27}),
    );
    const acknowledgement = vi.fn();
    const state = {...robotFixture, ack_seq: 7} as RobotStateMessage;

    coordinator.onRobotState(state, acknowledgement);

    expect(acknowledgement).toHaveBeenCalledOnce();
    expect(acknowledgement).toHaveBeenCalledWith(7);
    expect(panel.update).toHaveBeenCalledOnce();
    expect(runtimeUpdate).toHaveBeenCalledOnce();
  });
});
