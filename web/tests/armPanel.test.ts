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

    panel.stopButton.click();
    expect(send.mock.calls.at(-1)?.[0].type).toBe('disarm');
    expect(panel.isArmed).toBe(false);
  });

  it('fault and socket reconnect reset lock', () => {
    const panel = new ArmPanel(document.querySelector('#panel')!, vi.fn());

    panel.observeGrip(false);
    panel.armButton.click();
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
});
