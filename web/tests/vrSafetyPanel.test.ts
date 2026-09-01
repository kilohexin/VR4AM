// @vitest-environment jsdom
import * as THREE from 'three';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {
  describeVrSafety,
  type RobotRuntimeSummary,
  VrSafetyPanel,
} from '../src/scenes/vrSafetyPanel';
import type {ArmSafetyPhase, ArmSafetySnapshot} from '../src/ui/armPanel';

const state = (phase: ArmSafetyPhase): ArmSafetySnapshot => ({
  phase,
  connected: phase !== 'disconnected',
  connectionState: phase === 'disconnected' ? 'disconnected' : 'connected',
  eligible: false,
  armed: phase === 'armed' || phase === 'active',
  pending: phase === 'pending',
  mode: phase === 'active' ? 'ACTIVE' : 'READY',
  fault: phase === 'fault' ? 'ik_unreachable' : null,
  faultRecoverable: false,
  faultResetPending: false,
  constraint: null,
  recoveryPhase: null,
} as const);

describe('VR safety presentation', () => {
  it.each([
    ['locked', '未解锁', '松开 Grip 后按 A', 'muted', 'shield'],
    ['pending', '解锁中', '等待仿真确认', 'cyan', 'shield'],
    ['armed', '已解锁', '按住 Grip 移动', 'cyan', 'shield'],
    ['active', '运动中', '松开 Grip 停止 · B 紧急停止', 'cyan', 'shield'],
    ['stopped', '已停止', '保持 Grip 松开：A 解锁，B 回 Home', 'red', 'stop'],
    ['fault', '无法在线复位', '保持安全距离，检查 L Master/急停', 'red', 'warning'],
    ['disconnected', '连接已中断', '保持 Grip 松开', 'red', 'warning'],
  ] as const)('maps %s to readable text and a shape', (phase, title, instruction, tone, shape) => {
    expect(describeVrSafety(state(phase), true)).toEqual({
      title,
      instruction,
      footer: 'A解锁 B停止/Home · 右Grip末端 Trigger夹爪 · 左摇杆平台/按下归零 · 左Grip+摇杆升降',
      statusLine: 'SIMULATOR · TCP — · —',
      tone,
      shape,
    });
  });

  it('prioritizes occupied connection guidance over generic safety state', () => {
    expect(describeVrSafety({...state('disconnected'), connectionState: 'occupied'}, true))
      .toMatchObject({
        title: '控制端已被占用',
        instruction: '请关闭电脑端网页后重试',
        tone: 'red',
      });
  });

  it('prioritizes occupied connection guidance over unsupported controller copy', () => {
    expect(describeVrSafety({
      ...state('pending'),
      connected: false,
      connectionState: 'occupied',
      fault: 'workspace_violation',
      faultRecoverable: true,
      faultResetPending: true,
    }, false))
      .toMatchObject({
        title: '控制端已被占用',
        instruction: '请关闭电脑端网页后重试',
        tone: 'red',
      });
  });

  it.each([
    ['unreachable', '后端不可达'],
    ['disconnected', '连接已中断'],
    ['reconnecting', '正在重连'],
  ] as const)('renders a distinct %s connection title', (connectionState, title) => {
    expect(describeVrSafety({
      ...state('pending'),
      connectionState,
      connected: false,
      fault: 'workspace_violation',
      faultRecoverable: true,
      faultResetPending: true,
    }, false))
      .toMatchObject({title, tone: 'red'});
  });

  it('uses the connected safety flag as a disconnected fallback before fault state', () => {
    expect(describeVrSafety({
      ...state('pending'),
      connected: false,
      connectionState: 'connected',
      fault: 'workspace_violation',
      faultRecoverable: true,
      faultResetPending: true,
    }, false)).toMatchObject({
      title: '连接已中断',
      instruction: '保持 Grip 松开',
      tone: 'red',
    });
  });

  it.each([
    ['workspace_violation', '目标超出工作空间'],
    ['ik_unreachable', '目标不可达'],
    ['ik_singular', '目标接近奇异位形'],
    ['joint_safety_window', '目标超出仿真关节安全范围'],
  ] as const)('shows a readable reset instruction for recoverable fault %s', (fault, title) => {
    expect(describeVrSafety({
      ...state('fault'),
      fault,
      faultRecoverable: true,
    }, true)).toMatchObject({
      title,
      instruction: '松开 Grip，按 B 复位并回 Home',
      tone: 'red',
      shape: 'warning',
    });
  });

  it('shows reset progress before the underlying recoverable fault', () => {
    expect(describeVrSafety({
      ...state('fault'),
      fault: 'workspace_violation',
      faultRecoverable: true,
      faultResetPending: true,
    }, true)).toMatchObject({
      title: '复位中',
      instruction: '等待仿真确认',
    });
  });

  it('shows the Home recovery instruction for a pending Home or active recovery phase', () => {
    expect(describeVrSafety({...state('stopped'), homePending: true}, true)).toMatchObject({
      title: '正在返回 Home',
      instruction: '请保持 Grip 松开',
    });
    expect(describeVrSafety({
      ...state('fault'),
      fault: 'workspace_violation',
      faultRecoverable: true,
      faultResetPending: true,
      recoveryPhase: 'homing',
    }, true)).toMatchObject({
      title: '正在返回 Home',
      instruction: '请保持 Grip 松开',
    });
  });

  it('shows explicit restart guidance for an unrecoverable fault', () => {
    expect(describeVrSafety({
      ...state('fault'),
      fault: 'control_loop_error',
      faultRecoverable: false,
    }, true)).toMatchObject({
      title: '无法在线复位',
      instruction: '保持安全距离，检查 L Master/急停',
      tone: 'red',
    });
  });

  it('distinguishes stale data from a latched backend fault', () => {
    expect(describeVrSafety({
      ...state('fault'),
      mode: 'STALE',
      fault: null,
      faultRecoverable: false,
    }, true)).toMatchObject({
      title: '数据陈旧',
      instruction: '机械臂已停止，请检查网络',
      tone: 'red',
    });
  });

  it('blocks normal operation with a readable unsupported-controller message', () => {
    expect(describeVrSafety(state('locked'), false)).toEqual({
      title: '手柄不受支持',
      instruction: '当前配置不支持 A/B 安全控制',
      footer: 'A解锁 B停止/Home · 右Grip末端 Trigger夹爪 · 左摇杆平台/按下归零 · 左Grip+摇杆升降',
      statusLine: 'SIMULATOR · TCP — · —',
      tone: 'red',
      shape: 'warning',
    });
  });

  it.each([
    [{faultResetPending: true, faultRecoverable: true}, '复位中'],
    [{faultResetPending: false, faultRecoverable: true}, '目标超出工作空间'],
    [{faultResetPending: false, faultRecoverable: false}, '无法在线复位'],
  ] as const)('prioritizes fault handling over unsupported controller copy', (faultState, title) => {
    expect(describeVrSafety({
      ...state('fault'),
      fault: 'workspace_violation',
      ...faultState,
    }, false)).toMatchObject({title, tone: 'red'});
  });

  it('presents self-collision as a recoverable amber constraint', () => {
    expect(describeVrSafety({
      ...state('active'),
      constraint: 'self_collision',
    }, true)).toMatchObject({
      title: '机械臂接近自碰撞边界，请将手柄退回',
      tone: 'amber',
      shape: 'warning',
    });
  });

  it.each([
    ['workspace_boundary', '已到达操作边界，保持 Grip 向反方向退回'],
    ['ik_boundary', '当前姿态暂不可达，保持 Grip 退回上一个位置'],
    ['joint_boundary', '已到达关节操作边界，保持 Grip 反向退回'],
    ['motion_continuity_boundary', '运动变化过快，保持 Grip 放慢或反向退回'],
  ] as const)('uses the specific safe recovery instruction for %s', (constraint, instruction) => {
    const presentation = describeVrSafety({...state('active'), constraint}, true);
    expect(`${presentation.title} ${presentation.instruction}`).toContain(instruction);
    expect(presentation).toMatchObject({
      tone: 'amber',
    });
  });

  it('renders a compact real-arm runtime summary without joint telemetry', () => {
    const summary: RobotRuntimeSummary = {
      backend: 'LEBAI',
      realRobotMode: 'control',
      actualTcp: {p: [0.312, -0.041, 0.428], q: [0, 0, 0, 1]},
      gripper: 0.4,
      latencyMs: 18,
      hardwareVerified: false,
    };

    expect(describeVrSafety(state('locked'), true, summary)).toMatchObject({
      statusLine: '真机已连接 · CONTROL · TCP 0.312/−0.041/0.428 · 18 ms',
    });
  });

  it('keeps the safety phase presentation while controller support is unknown', () => {
    expect(describeVrSafety(state('locked'), null)).toMatchObject({
      title: '未解锁',
      instruction: '松开 Grip 后按 A',
      tone: 'muted',
      shape: 'shield',
    });
  });
});

describe('VR safety sprite resources', () => {
  let context: CanvasRenderingContext2D;

  beforeEach(() => {
    context = canvasContext();
    vi.spyOn(HTMLCanvasElement.prototype, 'getContext').mockReturnValue(context);
  });

  afterEach(() => vi.restoreAllMocks());

  it('uses a 1024 by 256 hidden sprite and redraws only when presentation changes', () => {
    const parent = new THREE.Group();
    const panel = new VrSafetyPanel(parent);
    const clearRect = vi.mocked(context.clearRect);

    expect(panel.sprite.visible).toBe(false);
    expect(panel.sprite.parent).toBe(parent);
    expect(panel.sprite.position.toArray()).toEqual([-0.82, 1.52, -0.72]);
    expect(describeVrSafety(state('stopped'), true).footer).toBe(
      'A解锁 B停止/Home · 右Grip末端 Trigger夹爪 · 左摇杆平台/按下归零 · 左Grip+摇杆升降',
    );
    expect(describeVrSafety(state('stopped'), true).instruction).toBe(
      '保持 Grip 松开：A 解锁，B 回 Home',
    );
    const texture = (panel.sprite.material as THREE.SpriteMaterial).map as THREE.CanvasTexture;
    expect(texture.image).toMatchObject({width: 1024, height: 256});

    panel.update(state('locked'), null);
    panel.update(state('locked'), true);
    expect(clearRect).toHaveBeenCalledOnce();

    panel.update(state('stopped'), true);
    expect(clearRect).toHaveBeenCalledTimes(2);
  });

  it('changes the real sprite visibility', () => {
    const panel = new VrSafetyPanel(new THREE.Group());

    panel.setVisible(true);
    expect(panel.sprite.visible).toBe(true);

    panel.setVisible(false);
    expect(panel.sprite.visible).toBe(false);
  });

  it('removes the sprite and disposes its texture and material', () => {
    const parent = new THREE.Group();
    const panel = new VrSafetyPanel(parent);
    const material = panel.sprite.material as THREE.SpriteMaterial;
    const texture = material.map as THREE.CanvasTexture;
    const disposeTexture = vi.spyOn(texture, 'dispose');
    const disposeMaterial = vi.spyOn(material, 'dispose');

    panel.dispose();

    expect(panel.sprite.parent).toBeNull();
    expect(disposeTexture).toHaveBeenCalledOnce();
    expect(disposeMaterial).toHaveBeenCalledOnce();
  });
});

function canvasContext(): CanvasRenderingContext2D {
  return {
    beginPath: vi.fn(),
    clearRect: vi.fn(),
    closePath: vi.fn(),
    fillRect: vi.fn(),
    fillText: vi.fn(),
    lineTo: vi.fn(),
    moveTo: vi.fn(),
    quadraticCurveTo: vi.fn(),
    restore: vi.fn(),
    save: vi.fn(),
    stroke: vi.fn(),
    strokeRect: vi.fn(),
    translate: vi.fn(),
  } as unknown as CanvasRenderingContext2D;
}
