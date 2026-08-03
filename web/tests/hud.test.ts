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
    expect(hud.rehearsalContainer).toBeInstanceOf(HTMLElement);
    expect(hud.rehearsalContainer.nextElementSibling).toBe(hud.diagnosticsContainer);
    expect(document.body.textContent).toContain('LM3 遥操作仿真');
    expect(document.body.textContent).toContain('仅仿真 · SIMULATOR');
    expect(document.body.textContent).toContain('右 Grip 建立末端零位');
    expect(document.body.textContent).not.toContain('LEBAI');
  });

  it('updates mode, safety inputs, latency, and readable fault state', () => {
    const hud = new Hud(document.querySelector('#app')!);

    hud.setConnectionStatus({state: 'connected'});
    hud.setController({tracking: true, grip: false, trigger: 0.4});
    hud.setRobotState({
      mode: 'ACTIVE', backendState: 'MOVING', sampleAgeMs: 15, fault: null,
      constraint: null, recoveryPhase: null,
    });
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
      constraint: null,
      recoveryPhase: null,
    });
    expect(document.body.textContent).toContain('目标不可达');
  });

  it('keeps a latched fault visible when the backend mode already returned to READY', () => {
    const hud = new Hud(document.querySelector('#app')!);

    hud.setRobotState({
      mode: 'READY',
      backendState: 'IDLE',
      sampleAgeMs: 1,
      fault: 'workspace_violation',
      constraint: null,
      recoveryPhase: null,
    });

    expect(document.querySelector('[data-field="mode"]')?.textContent).toBe('FAULT · 故障');
    expect(document.body.textContent).toContain('目标超出工作空间');
    expect(document.body.textContent).not.toContain('READY · 等待解锁');
  });

  it('shows self-collision as an amber soft constraint instruction', () => {
    const hud = new Hud(document.querySelector('#app')!);

    hud.setRobotState({
      mode: 'ACTIVE',
      backendState: 'IDLE',
      sampleAgeMs: 1,
      fault: null,
      constraint: 'self_collision',
      recoveryPhase: null,
    });

    expect(document.body.textContent).toContain(
      '机械臂接近自碰撞边界，请将手柄退回',
    );
    expect(
      document.querySelector('[data-row="constraint"]')
        ?.getAttribute('data-active'),
    ).toBe('true');
  });

  it('updates the runtime identity copy for simulator, twin, and real modes', () => {
    const hud = new Hud(document.querySelector('#app')!);

    hud.setRuntimeIdentity('SIMULATOR', null);
    expect(document.querySelector('.simulator-label')?.textContent).toBe('仅仿真 · SIMULATOR');
    hud.setRuntimeIdentity('LEBAI_FAKE', null);
    expect(document.querySelector('.simulator-label')?.textContent).toBe('数字孪生 · LEBAI_FAKE');
    expect(document.querySelector('.simulator-label')?.getAttribute('data-backend')).toBe('LEBAI_FAKE');
    hud.setRuntimeIdentity('LEBAI', 'readonly');
    expect(document.querySelector('.simulator-label')?.textContent).toBe('真机只读 · LEBAI');
    hud.setRuntimeIdentity('LEBAI', 'control');
    expect(document.querySelector('.simulator-label')?.textContent).toBe('真机控制 · LEBAI');
  });

  it('resets a real control identity to a null-safe copy on disconnect', () => {
    const hud = new Hud(document.querySelector('#app')!);
    hud.setRuntimeIdentity('LEBAI', 'control');
    expect(document.querySelector('.simulator-label')?.textContent).toBe('真机控制 · LEBAI');

    hud.setConnectionStatus({state: 'disconnected'});

    expect(document.querySelector('.simulator-label')?.textContent).toBe('仅仿真 · SIMULATOR');
    expect(document.querySelector('.simulator-label')?.getAttribute('data-backend')).toBe('SIMULATOR');
  });

  it('clears stale backend, age, fault, and latency values on disconnect', () => {
    const hud = new Hud(document.querySelector('#app')!);
    hud.setConnectionStatus({state: 'connected'});
    hud.setRobotState({
      mode: 'ACTIVE',
      backendState: 'MOVING',
      sampleAgeMs: 31,
      fault: 'workspace_violation',
      constraint: null,
      recoveryPhase: null,
    });
    hud.setLatency(18, 42);

    hud.setConnectionStatus({state: 'disconnected'});

    expect(document.body.textContent).toContain('DISCONNECTED · 未连接');
    expect(document.body.textContent).not.toContain('MOVING');
    expect(document.body.textContent).not.toContain('31 ms');
    expect(document.body.textContent).not.toContain('18 ms');
    expect(document.body.textContent).not.toContain('42 ms');
    expect(document.body.textContent).toContain('故障');
    expect(document.body.textContent).toContain('无');
  });

  it('shows controller occupancy guidance and clears stale telemetry', () => {
    const hud = new Hud(document.querySelector('#app')!);
    hud.setRobotState({
      mode: 'ACTIVE',
      backendState: 'MOVING',
      sampleAgeMs: 31,
      fault: null,
      constraint: null,
      recoveryPhase: null,
    });

    hud.setConnectionStatus({state: 'occupied', message: '占用'});

    expect(document.body.textContent).toContain('请关闭电脑端网页');
    expect(document.body.textContent).not.toContain('MOVING');
    expect(document.body.textContent).not.toContain('31 ms');
  });

  it.each([
    [{state: 'unreachable'} as const, '后端不可达'],
    [{state: 'disconnected'} as const, '连接已中断'],
    [{state: 'reconnecting'} as const, '正在重连'],
  ])('renders the exact structured connection label for $status.state', (status, label) => {
    const hud = new Hud(document.querySelector('#app')!);

    hud.setConnectionStatus(status);

    expect(document.body.textContent).toContain(label);
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

    hud.setRobotState({
      mode: 'FAULT', backendState: 'FAULT', sampleAgeMs: 1, fault,
      constraint: null, recoveryPhase: null,
    });

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
