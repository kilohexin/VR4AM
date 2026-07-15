// @vitest-environment jsdom
import * as THREE from 'three';
import {describe, expect, it, vi} from 'vitest';
import {
  createVRFrame,
  DesktopInputSafety,
  disposeObjectResources,
  modelLoadErrorMessage,
  ServerClockAnchor,
  SimulationScene,
} from '../src/scenes/simulationScene';
import type {ArmSafetySnapshot} from '../src/ui/armPanel';

describe('desktop VR frame path', () => {
  it('uses the shared VRFrame shape with metre poses and xyzw quaternion order', () => {
    const frame = createVRFrame({
      sessionId: 'desktop-test',
      sequence: 7,
      nowMs: 123.5,
      trackingValid: true,
      position: [0.4, 0.2, -0.1],
      quaternion: [0, 0.5, 0, 0.866],
      grip: false,
      trigger: 0.25,
    });

    expect(frame).toMatchObject({
      v: 1,
      type: 'vr_frame',
      session_id: 'desktop-test',
      seq: 7,
      client_mono_ms: 123.5,
      tracking_valid: true,
      visibility: 'visible',
      right: {
        p: [0.4, 0.2, -0.1],
        q: [0, 0.5, 0, 0.866],
        grip: false,
        trigger: 0.25,
      },
    });
  });
});

describe('desktop input release safety', () => {
  it.each([
    ['window blur', (canvas: HTMLCanvasElement) => window.dispatchEvent(new Event('blur'))],
    ['document visibility loss', (_canvas: HTMLCanvasElement) => {
      Object.defineProperty(document, 'visibilityState', {configurable: true, value: 'hidden'});
      document.dispatchEvent(new Event('visibilitychange'));
    }],
    ['lost pointer capture', (canvas: HTMLCanvasElement) => canvas.dispatchEvent(pointerEvent('lostpointercapture', 7))],
  ])('clears Grip, Trigger, and pointer state on %s', (_name, interrupt) => {
    Object.defineProperty(document, 'visibilityState', {configurable: true, value: 'visible'});
    const canvas = document.createElement('canvas');
    const captured = new Set<number>();
    canvas.setPointerCapture = vi.fn((pointerId: number) => captured.add(pointerId));
    canvas.hasPointerCapture = vi.fn((pointerId: number) => captured.has(pointerId));
    canvas.releasePointerCapture = vi.fn((pointerId: number) => captured.delete(pointerId));
    const input = new DesktopInputSafety(canvas);
    input.attach();
    canvas.dispatchEvent(pointerEvent('pointerdown', 7, 0));
    window.dispatchEvent(new KeyboardEvent('keydown', {code: 'Space'}));
    expect(input.snapshot()).toEqual({activePointer: 7, grip: true, trigger: 1});

    interrupt(canvas);

    expect(input.snapshot()).toEqual({activePointer: null, grip: false, trigger: 0});
    input.dispose();
  });

  it('contains pointer-capture release errors while clearing latched input', () => {
    const canvas = document.createElement('canvas');
    canvas.setPointerCapture = vi.fn();
    canvas.hasPointerCapture = vi.fn(() => true);
    canvas.releasePointerCapture = vi.fn(() => {
      throw new DOMException('capture already gone');
    });
    const input = new DesktopInputSafety(canvas);
    input.attach();
    canvas.dispatchEvent(pointerEvent('pointerdown', 3, 0));

    expect(() => window.dispatchEvent(new Event('blur'))).not.toThrow();
    expect(input.snapshot().grip).toBe(false);
    input.dispose();
  });

  it('clears Trigger on lost capture even after pointer-up already cleared the pointer id', () => {
    const canvas = document.createElement('canvas');
    canvas.setPointerCapture = vi.fn();
    canvas.hasPointerCapture = vi.fn(() => false);
    canvas.releasePointerCapture = vi.fn();
    const input = new DesktopInputSafety(canvas);
    input.attach();
    window.dispatchEvent(new KeyboardEvent('keydown', {code: 'Space'}));
    expect(input.snapshot().trigger).toBe(1);

    canvas.dispatchEvent(pointerEvent('lostpointercapture', 99));

    expect(input.snapshot().trigger).toBe(0);
    input.dispose();
  });
});

describe('scene connection and model lifetime helpers', () => {
  it('clears the server clock anchor at connection boundaries', () => {
    const anchor = new ServerClockAnchor();
    anchor.update(5_000_000_000, 100);
    expect(anchor.estimate(125)).toBe(5_025_000_000);

    anchor.reset();

    expect(anchor.estimate(150)).toBeNull();
  });

  it('hides raw loader errors behind readable Chinese feedback', () => {
    const message = modelLoadErrorMessage(new Error('Failed to fetch https://secret/model.glb'));

    expect(message).toBe('LM3 模型加载失败，请检查仿真资源。');
    expect(message).not.toContain('Failed to fetch');
    expect(message).not.toContain('https://');
  });

  it('disposes geometry and materials from a model that finishes loading late', () => {
    const geometry = new THREE.BoxGeometry();
    const texture = new THREE.Texture();
    const material = new THREE.MeshBasicMaterial({map: texture});
    const geometryDispose = vi.spyOn(geometry, 'dispose');
    const materialDispose = vi.spyOn(material, 'dispose');
    const textureDispose = vi.spyOn(texture, 'dispose');
    const group = new THREE.Group();
    group.add(new THREE.Mesh(geometry, material));

    disposeObjectResources(group);

    expect(geometryDispose).toHaveBeenCalledOnce();
    expect(materialDispose).toHaveBeenCalledOnce();
    expect(textureDispose).toHaveBeenCalledOnce();
  });
});

describe('XR render-loop handoff', () => {
  it('shows the safety panel only after the XR session is installed', async () => {
    const loop = vi.fn() as unknown as XRFrameRequestCallback;
    const session = {} as XRSession;
    let installSession!: () => void;
    const setSession = vi.fn().mockReturnValue(new Promise<void>((resolve) => {
      installSession = resolve;
    }));
    const setAnimationLoop = vi.fn();
    const setVisible = vi.fn();
    const resetInput = vi.fn();
    const cancel = vi.spyOn(window, 'cancelAnimationFrame').mockImplementation(() => {});
    const scene = {
      started: true,
      animationHandle: 42,
      inputSafety: {reset: resetInput},
      renderer: {xr: {setSession}, setAnimationLoop},
      vrSafetyPanel: {setVisible},
    };

    const starting = (SimulationScene.prototype.startXR as Function).call(scene, session, loop);

    expect(cancel).toHaveBeenCalledWith(42);
    expect(scene.animationHandle).toBeNull();
    expect(resetInput).toHaveBeenCalledOnce();
    expect(setSession).toHaveBeenCalledWith(session);
    expect(setVisible).not.toHaveBeenCalledWith(true);

    installSession();
    await starting;

    expect(setVisible).toHaveBeenLastCalledWith(true);
    expect(setAnimationLoop).toHaveBeenCalledWith(loop);
  });

  it('keeps the safety panel hidden when XR session installation fails', async () => {
    const setVisible = vi.fn();
    const scene = {
      started: true,
      animationHandle: null,
      inputSafety: {reset: vi.fn()},
      renderer: {
        xr: {setSession: vi.fn().mockRejectedValue(new DOMException('install failed'))},
        setAnimationLoop: vi.fn(),
      },
      vrSafetyPanel: {setVisible},
    };

    await expect((SimulationScene.prototype.startXR as Function).call(
      scene,
      {} as XRSession,
      vi.fn() as unknown as XRFrameRequestCallback,
    )).rejects.toThrow();

    expect(setVisible).toHaveBeenCalledWith(false);
    expect(setVisible).not.toHaveBeenCalledWith(true);
  });

  it('clears XR state and restores the desktop animation loop', async () => {
    const setSession = vi.fn().mockResolvedValue(undefined);
    const setAnimationLoop = vi.fn();
    const request = vi.spyOn(window, 'requestAnimationFrame').mockReturnValue(77);
    const animate = vi.fn();
    const setVisible = vi.fn();
    const scene = {
      started: true,
      animationHandle: null,
      animate,
      renderer: {xr: {setSession}, setAnimationLoop},
      vrSafetyPanel: {setVisible},
    };

    await (SimulationScene.prototype.stopXR as Function).call(scene);

    expect(setAnimationLoop).toHaveBeenCalledWith(null);
    expect(setSession).toHaveBeenCalledWith(null);
    expect(setVisible).toHaveBeenCalledWith(false);
    expect(request).toHaveBeenCalledWith(animate);
    expect(scene.animationHandle).toBe(77);
  });

  it('restores the desktop animation loop when clearing the XR session rejects', async () => {
    const setSession = vi.fn().mockRejectedValue(new DOMException('raw setSession failure'));
    const setAnimationLoop = vi.fn();
    const request = vi.spyOn(window, 'requestAnimationFrame').mockReturnValue(88);
    const animate = vi.fn();
    const setVisible = vi.fn();
    const scene = {
      started: true,
      animationHandle: null,
      animate,
      renderer: {xr: {setSession}, setAnimationLoop},
      vrSafetyPanel: {setVisible},
    };

    await expect((SimulationScene.prototype.stopXR as Function).call(scene)).rejects.toThrow();

    expect(setAnimationLoop).toHaveBeenCalledWith(null);
    expect(setVisible).toHaveBeenCalledWith(false);
    expect(request).toHaveBeenCalledWith(animate);
    expect(scene.animationHandle).toBe(88);
  });

  it('forwards cached arm state and controller support to the safety panel', () => {
    const snapshot = {
      phase: 'locked',
      connected: true,
      eligible: false,
      armed: false,
      pending: false,
      mode: 'READY',
      fault: null,
    } satisfies ArmSafetySnapshot;
    const update = vi.fn();
    const scene = {
      armSafetyState: snapshot,
      questControllerSupported: null,
      vrSafetyPanel: {update},
    };

    SimulationScene.prototype.setQuestControllerSupport.call(scene as never, false);
    expect(scene.questControllerSupported).toBe(false);
    expect(update).toHaveBeenLastCalledWith(snapshot, false);

    const stopped = {...snapshot, phase: 'stopped', mode: 'DISARMED'} satisfies ArmSafetySnapshot;
    SimulationScene.prototype.setArmSafetyState.call(scene as never, stopped);
    expect(scene.armSafetyState).toBe(stopped);
    expect(update).toHaveBeenLastCalledWith(stopped, false);
  });

  it('hides and disposes the safety panel with the scene', () => {
    const setVisible = vi.fn();
    const disposePanel = vi.fn();
    const removeListeners = vi.fn();
    const disposeRenderer = vi.fn();
    const removeCanvas = vi.fn();
    const scene = {
      started: true,
      animationHandle: null,
      renderer: {
        setAnimationLoop: vi.fn(),
        dispose: disposeRenderer,
        domElement: {remove: removeCanvas},
      },
      removeListeners,
      scene: new THREE.Scene(),
      vrSafetyPanel: {setVisible, dispose: disposePanel},
    };

    (SimulationScene.prototype.dispose as Function).call(scene);

    expect(setVisible).toHaveBeenCalledWith(false);
    expect(disposePanel).toHaveBeenCalledOnce();
    expect(removeListeners).toHaveBeenCalledOnce();
    expect(disposeRenderer).toHaveBeenCalledOnce();
    expect(removeCanvas).toHaveBeenCalledOnce();
  });
});

function pointerEvent(type: string, pointerId: number, button = 0): Event {
  const event = new Event(type);
  Object.defineProperties(event, {
    pointerId: {value: pointerId},
    button: {value: button},
    movementX: {value: 0},
    movementY: {value: 0},
  });
  return event;
}
