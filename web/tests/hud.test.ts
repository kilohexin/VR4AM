// @vitest-environment jsdom
import {beforeEach, describe, expect, it} from 'vitest';
import {Hud} from '../src/ui/hud';

beforeEach(() => {
  document.body.innerHTML = '<div id="app"></div>';
});

describe('Chinese simulator HUD', () => {
  it('renders only the approved simulator console regions and Chinese safety copy', () => {
    const hud = new Hud(document.querySelector('#app')!);

    expect(hud.sceneContainer).toBeInstanceOf(HTMLElement);
    expect(document.body.textContent).toContain('LM3 遥操作仿真');
    expect(document.body.textContent).toContain('仅仿真 · SIMULATOR');
    expect(document.body.textContent).toContain('按住右手 Grip 建立锚点并移动');
    expect(document.body.textContent).not.toContain('LEBAI');
  });

  it('updates mode, safety inputs, latency, and readable fault state', () => {
    const hud = new Hud(document.querySelector('#app')!);

    hud.setConnection(true);
    hud.setController({tracking: true, grip: false, trigger: 0.4});
    hud.setRobotState({mode: 'ACTIVE', backendState: 'MOVING', sampleAgeMs: 15, fault: null});
    hud.setLatency(24, 48);

    expect(document.body.textContent).toContain('ACTIVE · 遥操作中');
    expect(document.body.textContent).toContain('已连接');
    expect(document.body.textContent).toContain('有效');
    expect(document.body.textContent).toContain('松开');
    expect(document.body.textContent).toContain('40%');
    expect(document.body.textContent).toContain('24 ms');
    expect(document.body.textContent).toContain('48 ms');

    hud.setRobotState({
      mode: 'FAULT',
      backendState: 'FAULT',
      sampleAgeMs: 120,
      fault: 'ik_unreachable',
    });
    expect(document.body.textContent).toContain('目标不可达');
  });
});
