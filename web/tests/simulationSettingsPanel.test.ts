// @vitest-environment jsdom
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {SimulationSettingsPanel} from '../src/ui/simulationSettingsPanel';

describe('SimulationSettingsPanel', () => {
  beforeEach(() => {
    document.body.innerHTML = '<div id="panel"></div>';
    localStorage.clear();
  });

  it('offers 0.5..2.0 in 0.1 steps and applies only while fake runtime is stopped', () => {
    const request = vi.fn();
    const panel = new SimulationSettingsPanel(document.querySelector('#panel')!, request);
    panel.update({
      runtime: 'LEBAI_FAKE', connected: true, mode: 'READY', backendState: 'IDLE',
      grip: false, automationActive: false, authoritativeScale: 1.5,
    });
    const input = document.querySelector<HTMLInputElement>('[data-field="simulation-scale"]')!;
    const button = document.querySelector<HTMLButtonElement>('[data-action="apply-scale"]')!;
    expect([input.min, input.max, input.step, input.value]).toEqual(['0.5', '2', '0.1', '1.5']);
    input.value = '1.8';
    button.click();
    expect(request).toHaveBeenCalledWith(1.8);

    panel.update({
      runtime: 'LEBAI_FAKE', connected: true, mode: 'ACTIVE', backendState: 'MOVING',
      grip: true, automationActive: false, authoritativeScale: 1.5,
    });
    expect(input.disabled).toBe(true);
    expect(button.disabled).toBe(true);

    panel.update({
      runtime: 'LEBAI_FAKE', connected: false, mode: null, backendState: null,
      grip: false, automationActive: false, authoritativeScale: 1.5,
    });
    panel.update({
      runtime: 'LEBAI_FAKE', connected: true, mode: 'READY', backendState: 'IDLE',
      grip: false, automationActive: false, authoritativeScale: 1.5,
    });
    expect(input.disabled).toBe(false);
  });

  it('stores only accepted values and renders real robot scale read-only', () => {
    const panel = new SimulationSettingsPanel(document.querySelector('#panel')!, vi.fn());
    panel.handleResult({
      v: 1, type: 'simulation_scale_result', request_id: 'scale-1',
      accepted: true, translation_scale: 1.7,
    });
    expect(localStorage.getItem('vr4arm.simulation.translationScale')).toBe('1.7');

    panel.update({
      runtime: 'LEBAI', connected: true, mode: 'READY', backendState: 'IDLE',
      grip: false, automationActive: false, authoritativeScale: 0.5,
    });
    expect(document.body.textContent).toContain('真机比例（配置只读）');
    expect(document.querySelector<HTMLInputElement>('[data-field="simulation-scale"]')!.disabled).toBe(true);
  });
});
