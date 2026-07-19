// @vitest-environment jsdom
import * as THREE from 'three';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {describeVrSafety, VrSafetyPanel} from '../src/scenes/vrSafetyPanel';
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
} as const);

describe('VR safety presentation', () => {
  it.each([
    ['locked', '未解锁', '松开 Grip 后按 A', 'muted', 'shield'],
    ['pending', '解锁中', '等待仿真确认', 'cyan', 'shield'],
    ['armed', '已解锁', '按住 Grip 移动', 'cyan', 'shield'],
    ['active', '运动中', '松开 Grip 停止', 'cyan', 'shield'],
    ['stopped', '已停止', '松开 Grip 后按 A', 'red', 'stop'],
    ['fault', '故障/失联', '保持 Grip 松开', 'red', 'warning'],
    ['disconnected', '连接已中断', '保持 Grip 松开', 'red', 'warning'],
  ] as const)('maps %s to readable text and a shape', (phase, title, instruction, tone, shape) => {
    expect(describeVrSafety(state(phase), true)).toEqual({
      title,
      instruction,
      footer: 'A 解锁 · B 停止 · Grip 移动 · Trigger 夹爪',
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

  it.each([
    ['unreachable', '后端不可达'],
    ['disconnected', '连接已中断'],
    ['reconnecting', '正在重连'],
  ] as const)('renders a distinct %s connection title', (connectionState, title) => {
    expect(describeVrSafety({...state('disconnected'), connectionState}, true))
      .toMatchObject({title, tone: 'red'});
  });

  it('blocks with a readable unsupported-controller message', () => {
    expect(describeVrSafety(state('fault'), false)).toEqual({
      title: '手柄不受支持',
      instruction: '当前配置不支持 A/B 安全控制',
      footer: 'A 解锁 · B 停止 · Grip 移动 · Trigger 夹爪',
      tone: 'red',
      shape: 'warning',
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
