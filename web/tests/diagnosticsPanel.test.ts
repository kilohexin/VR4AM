// @vitest-environment jsdom
import {beforeEach, describe, expect, it, vi} from 'vitest';
import validDiagnostics from '../../schemas/fixtures/diagnostics-valid.json';
import robotFixture from '../../schemas/fixtures/robot-state-valid.json';
import type {DiagnosticsMessage, RobotStateMessage} from '../src/protocol/messages';
import {DiagnosticsPanel, DiagnosticsUpdateCoordinator} from '../src/ui/diagnosticsPanel';

beforeEach(() => {
  document.body.innerHTML = '<aside id="diagnostics"></aside>';
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
