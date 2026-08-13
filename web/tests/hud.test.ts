// @vitest-environment jsdom
import {beforeEach, describe, expect, it} from 'vitest';
// @ts-expect-error Vitest executes this CSS-contract test in Node without project Node typings.
import {readFileSync} from 'node:fs';
// @ts-expect-error Vitest executes this CSS-contract test in Node without project Node typings.
import {resolve} from 'node:path';
import {Hud} from '../src/ui/hud';

declare const process: {cwd(): string};

const styles = readFileSync(resolve(process.cwd(), 'src/styles.css'), 'utf8');

beforeEach(() => {
  document.body.innerHTML = '<div id="app"></div>';
});

describe('Chinese simulator HUD', () => {
  it('constrains narrow intrinsic widths and keeps rehearsal actions within one viewport column', () => {
    const narrowRules = styles.slice(styles.indexOf('@media (max-width: 900px)'));

    expect(narrowRules).toMatch(/\.operator-console\s*\{[^}]*grid-template-columns:\s*minmax\(0, 1fr\)/s);
    expect(narrowRules).toMatch(/\.help-strip\s*\{[^}]*white-space:\s*normal/s);
    expect(narrowRules).toMatch(/\.mode-heading strong\s*\{[^}]*white-space:\s*normal/s);
    expect(narrowRules).toMatch(/\.offline-rehearsal-actions\s*\{[^}]*grid-template-columns:\s*minmax\(0, 1fr\)/s);
    expect(narrowRules).toMatch(/\.offline-rehearsal-actions button\s*\{[^}]*width:\s*100%/s);
  });

  it('renders only the approved simulator console regions and Chinese safety copy', () => {
    const hud = new Hud(document.querySelector('#app')!);

    expect(hud.sceneContainer).toBeInstanceOf(HTMLElement);
    expect(hud.rehearsalBanner).toBeInstanceOf(HTMLElement);
    expect(hud.rehearsalBanner.textContent).toBe('仿真模式，不是真机');
    expect(hud.rehearsalBanner.parentElement).toBe(document.querySelector('.operator-console'));
    expect(hud.rehearsalBanner.closest('.status-rail')).toBeNull();
    expect(hud.rehearsalBanner.nextElementSibling).toBe(document.querySelector('.console-main'));
    expect(getComputedStyle(hud.rehearsalBanner).position).toBe('fixed');
    expect(getComputedStyle(hud.rehearsalBanner).top).toBe('0px');
    expect(getComputedStyle(hud.rehearsalBanner).left).toBe('0px');
    expect(getComputedStyle(hud.rehearsalBanner).right).toBe('0px');
    expect(getComputedStyle(document.querySelector('.operator-console')!).paddingTop).toBe('44px');
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

  it.each([
    ['workspace_boundary', '已到达操作边界，保持 Grip 向反方向退回'],
    ['ik_boundary', '当前姿态暂不可达，保持 Grip 退回上一个位置'],
    ['joint_boundary', '已到达关节操作边界，保持 Grip 反向退回'],
    ['motion_continuity_boundary', '运动变化过快，保持 Grip 放慢或反向退回'],
  ] as const)('shows %s as a connected recoverable constraint', (constraint, copy) => {
    const hud = new Hud(document.querySelector('#app')!);
    hud.setConnectionStatus({state: 'connected'});
    hud.setRobotState({
      mode: 'ACTIVE',
      backendState: 'HOLD',
      sampleAgeMs: 1,
      fault: null,
      constraint,
      recoveryPhase: null,
    });

    expect(document.body.textContent).toContain(copy);
    expect(document.body.textContent).toContain('已连接');
    expect(document.body.textContent).not.toContain('正在重连');
    expect(document.body.textContent).not.toContain('FAULT');
  });

  it('updates the runtime identity copy for simulator, twin, and real modes', () => {
    const hud = new Hud(document.querySelector('#app')!);

    hud.setRuntimeIdentity('SIMULATOR', null);
    expect(document.querySelector('.simulator-label')?.textContent).toBe('仅仿真 · SIMULATOR');
    hud.setRuntimeIdentity('LEBAI_FAKE', null);
    expect(document.querySelector('.simulator-label')?.textContent).toBe('仿真 · LEBAI_FAKE');
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
    ['ik_joint_jump', '逆解关节变化过大'],
    ['ik_failure_persistent', '逆解连续失败'],
    ['stop_unverified', '停止状态未确认'],
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
