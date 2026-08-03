// @vitest-environment jsdom
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {OFFLINE_REHEARSAL_HARDWARE_PENDING} from '../src/protocol/messages';
import type {OfflineRehearsalSnapshot} from '../src/rehearsal/offlineRehearsalController';
import {
  OfflineRehearsalPanel,
  type OfflineRehearsalEligibility,
} from '../src/ui/offlineRehearsalPanel';

const idleSnapshot: OfflineRehearsalSnapshot = {
  phase: 'idle', currentPhase: null, step: null, active: false, failure: null, runId: null,
  completedPhases: [], targetTcp: null, targetTrigger: null, placementTarget: null,
  remainingTimeoutMs: null, stopVerified: false, reportPaths: null, hardwareVerified: false,
  hardwarePending: OFFLINE_REHEARSAL_HARDWARE_PENDING,
};

const eligible: OfflineRehearsalEligibility = {
  connected: true,
  runtime: 'LEBAI_FAKE',
  robotRuntime: 'LEBAI_FAKE',
  hardwareVerified: false,
  mode: 'READY',
  backendState: 'IDLE',
  fault: null,
  constraint: null,
  actualTcp: {p: [0, 0, 0], q: [0, 0, 0, 1]},
};

beforeEach(() => {
  document.body.innerHTML = '<div id="panel"></div>';
});

describe('OfflineRehearsalPanel', () => {
  it('allows start only for the connected idle Fake digital twin and keeps stop for an active run', () => {
    const start = vi.fn();
    const stop = vi.fn();
    const panel = new OfflineRehearsalPanel(document.querySelector('#panel')!, start, stop);

    panel.update(idleSnapshot, eligible);
    expect(panel.startButton.disabled).toBe(false);
    expect(panel.stopButton.disabled).toBe(true);
    panel.startButton.click();
    expect(start).toHaveBeenCalledOnce();

    const unsafeCases: OfflineRehearsalEligibility[] = [
      {...eligible, connected: false},
      {...eligible, runtime: 'SIMULATOR'},
      {...eligible, robotRuntime: 'SIMULATOR'},
      {...eligible, hardwareVerified: null},
      {...eligible, mode: 'ACTIVE'},
      {...eligible, backendState: 'MOVING'},
      {...eligible, fault: 'workspace_violation'},
      {...eligible, constraint: 'workspace_boundary'},
    ];
    for (const unsafe of unsafeCases) {
      panel.update(idleSnapshot, unsafe);
      expect(panel.startButton.disabled).toBe(true);
    }

    panel.update({...idleSnapshot, phase: 'translate', currentPhase: 'translate', active: true}, eligible);
    expect(panel.startButton.disabled).toBe(true);
    expect(panel.stopButton.disabled).toBe(false);
    panel.stopButton.click();
    expect(stop).toHaveBeenCalledOnce();
  });

  it('permanently identifies the digital twin and renders all truthful run evidence', () => {
    const panel = new OfflineRehearsalPanel(document.querySelector('#panel')!, vi.fn(), vi.fn());
    panel.update({
      ...idleSnapshot,
      phase: 'translate', currentPhase: 'translate', step: '+x', active: true,
      failure: 'tracking_lost', completedPhases: ['identity_preflight', 'home'],
      targetTcp: {p: [0.003, 0, 0], q: [0, 0, 0, 1]}, remainingTimeoutMs: 1_520,
      reportPaths: {json: 'reports/run.json', markdown: 'reports/run.md'},
    }, eligible);

    const text = document.body.textContent ?? '';
    expect(text).toContain('DIGITAL TWIN / 数字孪生，不是真机');
    expect(text).toContain('translate');
    expect(text).toContain('+x');
    expect(text).toContain('2 / 12');
    expect(text).toContain('3.0 mm');
    expect(text).toContain('0.0°');
    expect(text).toContain('1.5 s');
    expect(text).toContain('tracking_lost');
    expect(text).toContain('reports/run.json');
    expect(text).toContain('reports/run.md');
    for (const check of OFFLINE_REHEARSAL_HARDWARE_PENDING) expect(text).toContain(check);
    expect(document.querySelectorAll('[data-hardware-check]')).toHaveLength(8);

    panel.update(idleSnapshot, {...eligible, runtime: 'LEBAI'});
    expect(document.body.textContent).toContain('DIGITAL TWIN / 数字孪生，不是真机');
  });
});
