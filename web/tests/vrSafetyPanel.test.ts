// @vitest-environment jsdom
import * as THREE from 'three';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {
  describeVrSafety,
  type RobotRuntimeSummary,
  VrSafetyPanel,
} from '../src/scenes/vrSafetyPanel';
import type {ArmSafetyPhase, ArmSafetySnapshot} from '../src/ui/armPanel';
import {ArmPanel} from '../src/ui/armPanel';
import type {ClientControlMessage} from '../src/protocol/messages';

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
  it('shows the distinct second A action after explicit unlock', () => {
    expect(describeVrSafety({...state('locked'), preparationRequired:true,
      preparationUnlocked:true}, true)).toMatchObject({
      title:'待进入准备姿态',instruction:'保持 Grip 松开，再按 A 进入准备位',
    });
  });
  it.each([
    [null,'正在准备'],['stopping','正在准备'],['homing','正在前往准备位'],['stabilizing','正在确认到位'],
  ] as const)('labels explicit preparation phase %s without Home', (recoveryPhase,title) => {
    const view=describeVrSafety({...state('pending'), preparationRequired:true,
      preparationPending:true,recoveryPhase},true);
    expect(view.title).toBe(title);expect(view.instruction).toContain('B');
    expect(view.footer).toContain('再按A准备');expect(JSON.stringify(view)).not.toContain('Home');
  });
  it('shows explicit ready state as controllable and preserves stop failure priority',()=>{
    expect(describeVrSafety({...state('armed'),preparationRequired:true},true).title).toBe('可操纵');
    expect(describeVrSafety({...state('locked'),preparationRequired:true,preparationUnlocked:true,
      fault:'stop_unverified'},true)).toMatchObject({tone:'red',title:'停止状态未确认'});
  });
  it.each([
    [{fault:'snapshot_rpc_result_unknown',phase:'fault',mode:'FAULT'},true,'无法在线复位'],
    [{mode:'STALE'},true,'数据陈旧'],
    [{},false,'手柄不受支持'],
  ] as const)('keeps warnings above explicit preparation progress %j',(changes,supported,title)=>{
    expect(describeVrSafety({...state('pending'),preparationRequired:true,preparationPending:true,
      recoveryPhase:'homing',...changes},supported)).toMatchObject({title,tone:'red'});
  });
  it('keeps a constraint visible while preparation is unlocked',()=>{
    expect(describeVrSafety({...state('locked'),preparationRequired:true,preparationUnlocked:true,
      constraint:'workspace_boundary'},true)).toMatchObject({tone:'amber',title:'已到达操作边界，保持 Grip 向反方向退回'});
  });
  it.each([
    ['locked', '未解锁', '松开 Grip 后按 A', 'muted', 'shield'],
    ['pending', '解锁中', '等待仿真确认', 'cyan', 'shield'],
    ['armed', '已解锁', '按住 Grip 移动', 'cyan', 'shield'],
    ['active', '运动中', '松开 Grip 请求停止 · B 请求停止', 'cyan', 'shield'],
    ['stopped', '控制已锁定', '保持 Grip 松开，确认现场静止后再操作', 'red', 'stop'],
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
      instruction: '保持 Grip 松开，停止状态未确认',
      tone: 'red',
    });
  });

  it.each(['stop_unverified', 'stop_incomplete'])('keeps %s explicit ahead of Home progress and IDLE summary', (fault) => {
    const view = describeVrSafety({
      ...state('fault'), fault, homePending: true, recoveryPhase: 'stopping',
    }, false, {
      backend: 'LEBAI', realRobotMode: 'control', actualTcp: null,
      gripper: 0, latencyMs: 1, hardwareVerified: false,
    });
    expect(view.title).toContain('未确认');
    expect(view.instruction).toBe('保持 Grip 松开，确认现场静止；异常按急停');
    expect(view.title).not.toContain('Home');
    expect(view.tone).toBe('red');
    expect(view.shape).toBe('warning');
  });

  it('does not turn a local disarm request into a physical standstill claim', () => {
    const container = document.createElement('div');
    const sent: ClientControlMessage[] = [];
    const panel = new ArmPanel(container, (message) => sent.push(message));
    panel.setConnected(true);
    panel.setMode('READY');
    panel.setMode('ARMED');
    panel.setMode('ACTIVE');
    expect(panel.safetyState.phase).toBe('active');
    panel.requestDisarm('xr');
    expect(sent).toHaveLength(1);
    expect(sent[0].type).toBe('disarm');
    expect(panel.safetyState.mode).toBe('ACTIVE');
    const view = describeVrSafety(panel.safetyState, true);
    expect(view.title).toBe('正在确认停止');
    expect(view.instruction).toContain('等待收尾');
    expect(view.instruction).not.toContain('Home');
    expect(view.title).not.toContain('已停止');
  });

  it.each(['stop_unverified', 'stop_incomplete'])('keeps the last %s warning while disconnected', (fault) => {
    const container = document.createElement('div');
    const sent: ClientControlMessage[] = [];
    const panel = new ArmPanel(container, (message) => sent.push(message));
    panel.setConnected(true);
    panel.setFault(fault);
    panel.setConnectionStatus({state: 'reconnecting'});
    panel.observeGrip(false);
    const view = describeVrSafety(panel.safetyState, true);
    expect(view.title).toBe('正在重连');
    expect(view.instruction).toContain('上次停止未确认');
    expect(view.instruction).not.toContain('Home');
    expect(panel.requestArm('xr')).toBe(false);
    expect(sent).toHaveLength(0);
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
    ['tracking_lag', '机械臂尚未跟上目标 减小位移或旋转，等待跟随'],
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
      statusLine: '真机已连接 · CONTROL · TCP 0.312/−0.041/0.428 · 夹爪闭合度40% · 18 ms',
    });
  });

  it('does not mislabel an unknown real-arm mode as readonly', () => {
    const summary: RobotRuntimeSummary = {
      backend: 'LEBAI',
      realRobotMode: null,
      actualTcp: null,
      gripper: null,
      latencyMs: null,
      hardwareVerified: false,
    };

    expect(describeVrSafety(state('locked'), true, summary)).toMatchObject({
      statusLine: '真机已连接 · 模式未确认 · TCP — · 夹爪闭合度— · —',
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

it('shows an outstanding stop as waiting, with fault warnings retaining priority',()=>{
  expect(describeVrSafety({...state('stopped'),stopPending:true},true)).toMatchObject({title:'正在确认停止',tone:'amber'});
  expect(describeVrSafety({...state('stopped'),stopPending:true,fault:'stop_unverified'},true)).toMatchObject({title:'停止状态未确认',tone:'red'});
});

it('describes natural finite-step release instead of the legacy streaming stop',()=>{
  const summary:RobotRuntimeSummary={backend:'LEBAI_FAKE',realRobotMode:'control',actualTcp:null,gripper:0,
    latencyMs:0,hardwareVerified:false,finitePosition:{policy:'finite_position_offline',phase:'draining',
      generation:1,entered_steps:1,motion_id:1,reason:null,hardware_verified:false}};
  const view=describeVrSafety({...state('active'),preparationRequired:true},true,summary);
  expect(view.title).toBe('正在完成当前小步');expect(view.instruction).toContain('B');
  expect(view.instruction).not.toContain('松开 Grip 请求停止');
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
      '保持 Grip 松开，确认现场静止后再操作',
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
