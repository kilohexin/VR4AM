// @vitest-environment jsdom
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {ArmPanel} from '../src/ui/armPanel';

beforeEach(() => {
  document.body.innerHTML = '<div id="panel"></div>';
});

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
});
