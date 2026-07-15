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

  it('clears stale backend, age, fault, and latency values on disconnect', () => {
    const hud = new Hud(document.querySelector('#app')!);
    hud.setConnection(true);
    hud.setRobotState({
      mode: 'ACTIVE',
      backendState: 'MOVING',
      sampleAgeMs: 31,
      fault: 'workspace_violation',
    });
    hud.setLatency(18, 42);

    hud.setConnection(false);

    expect(document.body.textContent).toContain('DISCONNECTED · 未连接');
    expect(document.body.textContent).not.toContain('MOVING');
    expect(document.body.textContent).not.toContain('31 ms');
    expect(document.body.textContent).not.toContain('18 ms');
    expect(document.body.textContent).not.toContain('42 ms');
    expect(document.body.textContent).toContain('故障');
    expect(document.body.textContent).toContain('无');
  });

  it.each([
    ['protocol_error', '协议消息无效'],
    ['tracking_lost', '追踪已丢失'],
    ['input_stale', '控制输入已超时'],
    ['control_overrun', '控制周期连续超时'],
    ['invalid_numeric', '控制数据包含无效数值'],
    ['workspace_violation', '目标超出工作空间'],
    ['joint_limit', '目标超出关节限制'],
    ['joint_safety_window', '目标超出仿真关节安全范围'],
    ['invalid_joint_count', '机器人关节数据无效'],
    ['ik_unreachable', '目标不可达'],
    ['ik_singular', '目标接近奇异位形'],
    ['backend_disconnected', '仿真后端已断开'],
    ['backend_fault', '仿真后端故障'],
    ['real_robot_disabled', '第一里程碑禁用真机'],
  ])('maps backend fault %s to readable Chinese', (fault, message) => {
    const hud = new Hud(document.querySelector('#app')!);

    hud.setRobotState({mode: 'FAULT', backendState: 'FAULT', sampleAgeMs: 1, fault});

    expect(document.body.textContent).toContain(message);
    expect(document.body.textContent).not.toContain(fault);
  });

  it('clears stale scene feedback after a later successful VR start', () => {
    const hud = new Hud(document.querySelector('#app')!);
    hud.showSceneError('当前设备不支持沉浸式 VR');

    hud.clearSceneError();

    expect(document.body.textContent).not.toContain('当前设备不支持沉浸式 VR');
    expect(document.querySelector<HTMLElement>('.scene-notice')?.hidden).toBe(true);
  });
});
