// @vitest-environment jsdom
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {ArmPanel} from '../src/ui/armPanel';

beforeEach(() => {
  document.body.innerHTML = '<div id="panel"></div>';
});

async function flushSafetyChange(): Promise<void> {
  await Promise.resolve();
}

describe('ArmPanel simulator safety', () => {
  it('is simulator-only and starts locked', () => {
    const send = vi.fn();
    const panel = new ArmPanel(document.querySelector('#panel')!, send);

    expect(panel.backendText).toBe('SIMULATOR');
    expect(document.body.textContent).not.toContain('LEBAI');
    expect(panel.armButton.disabled).toBe(true);

    panel.armButton.click();
    expect(send).not.toHaveBeenCalled();
  });

  it('does not arm before a valid released-grip sample', () => {
    const send = vi.fn();
    const panel = new ArmPanel(document.querySelector('#panel')!, send);

    panel.observeGrip(true);
    panel.armButton.click();

    expect(send).not.toHaveBeenCalled();
  });

  it('stays locked while disconnected even when local grip is released', () => {
    const send = vi.fn();
    const panel = new ArmPanel(document.querySelector('#panel')!, send);

    panel.setConnected(false);
    panel.observeGrip(false);
    panel.armButton.click();

    expect(panel.armButton.disabled).toBe(true);
    expect(send).not.toHaveBeenCalled();
  });

  it('arms only after grip release and stop disarms', () => {
    const send = vi.fn();
    const panel = new ArmPanel(document.querySelector('#panel')!, send);

    panel.observeGrip(false);
    panel.armButton.click();
    expect(send.mock.calls[0][0].type).toBe('arm_request');
    expect(panel.isArmed).toBe(false);
    expect(panel.isArmPending).toBe(true);

    panel.stopButton.click();
    expect(send.mock.calls.at(-1)?.[0].type).toBe('disarm');
    expect(panel.isArmed).toBe(false);
  });

  it('suppresses duplicate arm requests while one is in flight', () => {
    const send = vi.fn();
    const panel = new ArmPanel(document.querySelector('#panel')!, send);
    panel.observeGrip(false);

    panel.armButton.click();
    panel.armButton.click();

    expect(send).toHaveBeenCalledTimes(1);
    expect(panel.armButton.disabled).toBe(true);
  });

  it('reconciles pending arm state with acknowledgement and authoritative modes', () => {
    const send = vi.fn();
    const panel = new ArmPanel(document.querySelector('#panel')!, send);
    panel.observeGrip(false);
    panel.armButton.click();
    const request = send.mock.calls[0][0];

    panel.handleArmFeedback({v: 1, type: 'arm_ack', request_id: request.request_id});
    expect(panel.isArmPending).toBe(true);
    expect(panel.isArmed).toBe(false);

    panel.setMode('READY');
    expect(panel.isArmPending).toBe(true);

    panel.setMode('ARMED');
    expect(panel.isArmPending).toBe(false);
    expect(panel.isArmed).toBe(true);

    panel.setMode('DISARMED');
    expect(panel.isArmed).toBe(false);
    expect(panel.armButton.disabled).toBe(true);
  });

  it('returns to a readable safe locked state when arm is rejected', () => {
    const send = vi.fn();
    const panel = new ArmPanel(document.querySelector('#panel')!, send);
    panel.observeGrip(false);
    panel.armButton.click();
    const request = send.mock.calls[0][0];

    panel.handleArmFeedback({
      v: 1,
      type: 'arm_rejected',
      request_id: request.request_id,
      message: 'raw backend detail',
    });

    expect(panel.isArmPending).toBe(false);
    expect(panel.isArmed).toBe(false);
    expect(panel.armButton.disabled).toBe(true);
    expect(panel.feedbackText).toContain('解锁请求被拒绝');
    expect(document.body.textContent).not.toContain('raw backend detail');

    panel.observeGrip(false);
    expect(panel.armButton.disabled).toBe(false);
    expect(document.body.textContent).toContain('解锁被拒绝');
  });

  it('fault and socket reconnect reset lock', () => {
    const panel = new ArmPanel(document.querySelector('#panel')!, vi.fn());

    panel.observeGrip(false);
    panel.armButton.click();
    panel.setMode('ARMED');
    expect(panel.isArmed).toBe(true);

    panel.setFault('ik_unreachable');
    expect(panel.armButton.disabled).toBe(true);
    expect(panel.isArmed).toBe(false);

    panel.setFault(null);
    panel.observeGrip(false);
    panel.armButton.click();
    panel.onSocketReconnect();
    expect(panel.isArmed).toBe(false);
    expect(panel.armButton.disabled).toBe(true);
  });

  it('shows truthful VR starting, active, idle, and error states', () => {
    const panel = new ArmPanel(document.querySelector('#panel')!, vi.fn());

    panel.setVRStatus({state: 'starting'});
    expect(panel.vrButton.textContent).toContain('正在进入 VR');
    expect(panel.vrButton.disabled).toBe(true);

    panel.setVRStatus({state: 'active'});
    expect(panel.vrButton.textContent).toContain('退出 VR');
    expect(panel.vrButton.disabled).toBe(false);

    panel.setVRStatus({state: 'idle'});
    expect(panel.vrButton.textContent).toContain('进入 VR');

    panel.setVRStatus({state: 'error', message: '当前设备不支持沉浸式 VR'});
    expect(panel.vrButton.textContent).toContain('进入 VR');
    expect(panel.vrButton.title).toBe('当前设备不支持沉浸式 VR');
    expect(panel.vrButton.dataset.state).toBe('error');
  });

  it('uses one safety gate for desktop and XR arm requests', () => {
    const send = vi.fn();
    const states = vi.fn();
    const panel = new ArmPanel(document.querySelector('#panel')!, send, undefined, states);

    expect(panel.requestArm('xr')).toBe(false);
    panel.observeGrip(false);
    expect(panel.requestArm('xr')).toBe(true);
    expect(send).toHaveBeenCalledWith(expect.objectContaining({
      type: 'arm_request',
      request_id: expect.stringMatching(/^xr-arm_request-/),
    }));
    expect(panel.requestArm('xr')).toBe(false);
    expect(send).toHaveBeenCalledTimes(1);
    expect(panel.safetyState.phase).toBe('pending');
  });

  it('XR disarm closes the local gate before transport returns', () => {
    const events: string[] = [];
    let panel!: ArmPanel;
    panel = new ArmPanel(
      document.querySelector('#panel')!,
      (message) => events.push(`${message.type}:${panel.safetyState.phase}`),
    );
    panel.observeGrip(false);
    panel.requestArm('xr');
    panel.setMode('ARMED');

    panel.requestDisarm('xr');

    expect(events.at(-1)).toBe('disarm:stopped');
    expect(panel.safetyState).toMatchObject({
      phase: 'stopped',
      armed: false,
      eligible: false,
      pending: false,
    });
  });

  it('publishes fresh immutable readable safety snapshots asynchronously', async () => {
    const snapshots: unknown[] = [];
    const panel = new ArmPanel(
      document.querySelector('#panel')!,
      vi.fn(),
      undefined,
      (snapshot) => snapshots.push(snapshot),
    );

    panel.setConnected(false);
    expect(snapshots).toHaveLength(0);
    await flushSafetyChange();
    const disconnected = panel.safetyState;
    expect(disconnected.phase).toBe('disconnected');
    expect(snapshots.at(-1)).toMatchObject({phase: 'disconnected'});
    expect(Object.isFrozen(disconnected)).toBe(true);
    expect(() => {
      (disconnected as {phase: string}).phase = 'active';
    }).toThrow(TypeError);

    panel.setConnected(true);
    panel.setFault('ik_unreachable');
    await flushSafetyChange();
    const fault = panel.safetyState;
    expect(fault.phase).toBe('fault');
    expect(snapshots.length).toBeGreaterThan(0);
    expect(snapshots.at(-1)).not.toBe(fault);
    expect(panel.safetyState).not.toBe(fault);
  });

  it('keeps a local stop durable while a rearm waits for authoritative readiness', () => {
    const send = vi.fn();
    const panel = new ArmPanel(document.querySelector('#panel')!, send);

    panel.setMode('READY');
    panel.observeGrip(false);
    expect(panel.requestArm('xr')).toBe(true);
    panel.setMode('ACTIVE');
    expect(panel.safetyState).toMatchObject({phase: 'active', armed: true});

    panel.requestDisarm('xr');
    panel.observeGrip(false);
    expect(panel.requestArm('xr')).toBe(true);
    expect(panel.safetyState).toMatchObject({phase: 'pending', armed: false, mode: 'ACTIVE'});

    panel.setMode('ACTIVE');
    expect(panel.safetyState).toMatchObject({phase: 'pending', armed: false});
    panel.setMode('READY');
    expect(panel.safetyState).toMatchObject({phase: 'pending', armed: false});
    panel.setMode('ARMED');
    expect(panel.safetyState).toMatchObject({phase: 'armed', armed: true, pending: false});
  });

  it('normalizes stale ACTIVE mode across disconnect, reconnect, and lifecycle reset', () => {
    const panel = new ArmPanel(document.querySelector('#panel')!, vi.fn());

    panel.setMode('ACTIVE');
    panel.setConnected(false);
    expect(panel.safetyState).toMatchObject({
      phase: 'disconnected',
      connected: false,
      armed: false,
      mode: 'DISCONNECTED',
    });

    panel.setConnected(true);
    expect(panel.safetyState).toMatchObject({phase: 'stopped', armed: false, mode: 'DISARMED'});
    panel.setMode('ACTIVE');
    panel.onSocketReconnect();
    expect(panel.safetyState).toMatchObject({phase: 'stopped', armed: false, mode: 'DISARMED'});

    panel.setMode('ACTIVE');
    panel.resetToLocked();
    expect(panel.safetyState).toMatchObject({phase: 'stopped', armed: false, mode: 'DISARMED'});
  });

  it('does not revive stale ACTIVE mode when a fault clears', () => {
    const panel = new ArmPanel(document.querySelector('#panel')!, vi.fn());

    panel.setMode('ACTIVE');
    panel.setFault('ik_unreachable');
    expect(panel.safetyState).toMatchObject({phase: 'fault', armed: false, mode: 'DISARMED'});

    panel.setFault(null);
    expect(panel.safetyState).toMatchObject({phase: 'stopped', armed: false, mode: 'DISARMED'});

    panel.setMode('FAULT');
    panel.setFault('lagging_fault_detail');
    panel.setFault(null);
    expect(panel.safetyState).toMatchObject({phase: 'fault', armed: false, fault: null});
  });

  it.each([
    ['DISCONNECTED', 'disconnected', false],
    ['STALE', 'fault', true],
    ['FAULT', 'fault', true],
  ] as const)(
    'maps authoritative %s mode to %s even when auxiliary fields lag',
    (mode, phase, connected) => {
      const panel = new ArmPanel(document.querySelector('#panel')!, vi.fn());

      panel.setConnected(true);
      panel.setFault(null);
      panel.setMode(mode);
      panel.setFault('lagging_fault_detail');
      panel.setFault(null);

      expect(panel.safetyState).toMatchObject({phase, connected, armed: false, fault: null});
    },
  );

  it('delivers callback-requested disarm after the outer arm transport', async () => {
    const events: string[] = [];
    let panel!: ArmPanel;
    panel = new ArmPanel(
      document.querySelector('#panel')!,
      (message) => events.push(`send:${message.type}`),
      undefined,
      (snapshot) => {
        events.push(`callback:${snapshot.phase}`);
        if (snapshot.phase === 'pending') panel.requestDisarm('xr');
      },
    );

    panel.observeGrip(false);
    expect(panel.requestArm('xr')).toBe(true);
    expect(events).toEqual(['send:arm_request']);

    await flushSafetyChange();
    expect(events.slice(0, 3)).toEqual([
      'send:arm_request',
      'callback:pending',
      'send:disarm',
    ]);
    await flushSafetyChange();
    expect(events.at(-1)).toBe('callback:stopped');
  });

  it('contains callback exceptions without blocking arm or disarm transport', async () => {
    const send = vi.fn();
    const callback = vi.fn(() => {
      throw new Error('consumer failed');
    });
    const panel = new ArmPanel(
      document.querySelector('#panel')!,
      send,
      undefined,
      callback,
    );

    panel.observeGrip(false);
    expect(panel.requestArm('xr')).toBe(true);
    await flushSafetyChange();
    panel.requestDisarm('xr');
    await flushSafetyChange();

    expect(send.mock.calls.map(([message]) => message.type)).toEqual(['arm_request', 'disarm']);
    expect(callback).toHaveBeenCalledTimes(2);
  });

  it('maps authoritative and local safety phases with stop precedence', () => {
    const panel = new ArmPanel(document.querySelector('#panel')!, vi.fn());

    panel.setMode('READY');
    expect(panel.safetyState.phase).toBe('locked');

    panel.observeGrip(false);
    panel.requestArm('xr');
    expect(panel.safetyState.phase).toBe('pending');

    panel.setMode('ARMED');
    expect(panel.safetyState.phase).toBe('armed');
    panel.setMode('HOLD');
    expect(panel.safetyState.phase).toBe('armed');
    panel.setMode('ACTIVE');
    expect(panel.safetyState.phase).toBe('active');

    panel.requestDisarm('xr');
    expect(panel.safetyState.phase).toBe('stopped');
    panel.setMode('ACTIVE');
    expect(panel.safetyState.phase).toBe('stopped');
    panel.setMode('READY');
    expect(panel.safetyState.phase).toBe('locked');
    panel.setMode('DISARMED');
    expect(panel.safetyState.phase).toBe('stopped');

    panel.setFault('ik_unreachable');
    expect(panel.safetyState.phase).toBe('fault');
    panel.setConnected(false);
    expect(panel.safetyState.phase).toBe('disconnected');
  });

  it('keeps desktop button request IDs source-qualified', () => {
    const send = vi.fn();
    const panel = new ArmPanel(document.querySelector('#panel')!, send);

    panel.observeGrip(false);
    panel.armButton.click();
    expect(send).toHaveBeenLastCalledWith(expect.objectContaining({
      type: 'arm_request',
      request_id: expect.stringMatching(/^desktop-arm_request-/),
    }));

    panel.stopButton.click();
    expect(send).toHaveBeenLastCalledWith(expect.objectContaining({
      type: 'disarm',
      request_id: expect.stringMatching(/^desktop-disarm-/),
    }));
  });
});
