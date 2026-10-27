// @vitest-environment jsdom
import * as THREE from 'three';
import {describe, expect, it, vi} from 'vitest';
import {
  createGraspBlocks,
  createGraspWorkbench,
  GRASP_WORKBENCH_SURFACE,
  createVRFrame,
  disposeObjectResources,
  modelLoadErrorMessage,
  ServerClockAnchor,
  SimulationScene,
  fakeRehearsalWorkspaceLayout,
} from '../src/scenes/simulationScene';
import type {ArmSafetySnapshot} from '../src/ui/armPanel';
import type {Pose, RobotStateMessage} from '../src/protocol/messages';
import type {XRPresentationSample} from '../src/xr/session';
import type {OfflineControllerSample} from '../src/rehearsal/types';
import {DesktopOrbitCamera} from '../src/scenes/desktopOrbitCamera';
import {KinematicGraspController} from '../src/scenes/kinematicGraspController';

describe('grasp block resources', () => {
  it('provides a workbench whose supports touch the existing desk and hold the blocks', () => {
    const bench = createGraspWorkbench();
    const environment = {
      scene: new THREE.Scene(), robotVisualRoot: new THREE.Group(),
      grid: new THREE.GridHelper(),
    };
    (SimulationScene.prototype as unknown as {createEnvironment(this: typeof environment): void})
      .createEnvironment.call(environment);
    const deskBounds = new THREE.Box3().setFromObject(environment.robotVisualRoot.getObjectByName('desk-top')!);
    const worktop = bench.getObjectByName('workbench-top')!;
    const topBounds = new THREE.Box3().setFromObject(worktop);
    const supports = bench.children.filter(({name}) => name.startsWith('workbench-post-'));
    expect(supports).toHaveLength(4);
    for (const support of supports) {
      const bounds = new THREE.Box3().setFromObject(support);
      expect(bounds.min.y).toBeCloseTo(-0.005, 7);
      expect(bounds.min.y).toBeCloseTo(deskBounds.max.y, 7);
      expect(bounds.max.y).toBeCloseTo(topBounds.min.y, 7);
      expect(bounds.min.x).toBeGreaterThanOrEqual(deskBounds.min.x);
      expect(bounds.max.x).toBeLessThanOrEqual(deskBounds.max.x);
      expect(bounds.min.z).toBeGreaterThanOrEqual(deskBounds.min.z);
      expect(bounds.max.z).toBeLessThanOrEqual(deskBounds.max.z);
    }
    for (const {object} of createGraspBlocks()) {
      const bounds = new THREE.Box3().setFromObject(object);
      expect(bounds.min.y).toBeCloseTo(topBounds.max.y, 7);
      expect(bounds.min.x).toBeGreaterThan(topBounds.min.x);
      expect(bounds.max.x).toBeLessThan(topBounds.max.x);
      expect(bounds.min.z).toBeGreaterThan(topBounds.min.z);
      expect(bounds.max.z).toBeLessThan(topBounds.max.z);
    }
  });

  it('releases a block onto the visible worktop within its surface bounds', () => {
    const root = new THREE.Group();
    const block = createGraspBlocks()[0];
    const bench = createGraspWorkbench();
    root.add(block.object, bench);
    const grasp = new KinematicGraspController({
      visualRoot: root, blocks: [block], ...GRASP_WORKBENCH_SURFACE,
    });
    const q = [0, 0, 0, 1] as const;
    grasp.update({p: [-0.05, 0.56, -0.23], q}, 0.9);
    grasp.update({p: [2, 0.8, 2], q}, 0.9);
    grasp.update({p: [2, 0.8, 2], q}, 0.1);
    const blockBounds = new THREE.Box3().setFromObject(block.object);
    const topBounds = new THREE.Box3().setFromObject(bench.getObjectByName('workbench-top')!);
    expect(grasp.snapshot().carriedBlockId).toBeNull();
    expect(blockBounds.min.y).toBeCloseTo(topBounds.max.y, 7);
    expect(blockBounds.max.x).toBeCloseTo(topBounds.max.x, 7);
    expect(blockBounds.max.z).toBeCloseTo(topBounds.max.z, 7);
  });
  it('creates five exact colored 60 mm blocks at deterministic positions', () => {
    const blocks = createGraspBlocks();

    expect(blocks.map(({id}) => id)).toEqual([
      'block-orange',
      'block-blue',
      'block-green',
      'block-yellow',
      'block-purple',
    ]);
    expect(blocks.map(({object}) => (
      (object as THREE.Mesh).material as THREE.MeshStandardMaterial
    ).color.getHex())).toEqual([
      0xff8a3d,
      0x39a8ff,
      0x58d68d,
      0xffd84d,
      0xa77bff,
    ]);
    expect(blocks.map(({object}) => object.position.toArray())).toEqual([
      [-0.05, 0.56, -0.23],
      [-0.14, 0.56, -0.26],
      [-0.24, 0.56, -0.25],
      [-0.27, 0.56, -0.14],
      [-0.16, 0.56, -0.10],
    ]);
    expect(blocks.every(({sizeM}) => sizeM === 0.06)).toBe(true);
  });

  it('keeps every block center comfortably inside the Fake Home workspace', () => {
    const homeTcp = [-0.14378786228061097, 0.6959976154948492, -0.12063003385763715];

    for (const {object} of createGraspBlocks()) {
      object.position.toArray().forEach((value, axis) => {
        expect(Math.abs(value - homeTcp[axis])).toBeLessThanOrEqual(0.145);
      });
    }
  });
});

describe('desktop VR frame path', () => {
  it('keeps left-drag, Space and wheel neutral while retaining right-drag camera orbit', () => {
    Object.defineProperty(document, 'visibilityState', {configurable: true, value: 'visible'});
    const canvas = document.createElement('canvas');
    document.body.append(canvas);
    const captures = new Set<number>();
    canvas.setPointerCapture = vi.fn(id => captures.add(id));
    canvas.hasPointerCapture = vi.fn(id => captures.has(id));
    canvas.releasePointerCapture = vi.fn(id => captures.delete(id));
    const camera = new THREE.PerspectiveCamera();
    const orbitCamera = new DesktopOrbitCamera(canvas, camera);
    const onFrame = vi.fn();
    const scene = Object.assign(Object.create(SimulationScene.prototype), {
      renderer: {domElement: canvas}, orbitCamera,
      controllerPosition: new THREE.Vector3(0.56, 0.42, 0.18),
      controllerQuaternion: new THREE.Quaternion(),
      offlineController: null, automationActive: false,
      sessionId: 'canvas-neutral', sequence: 0, lastFrameMs: Number.NEGATIVE_INFINITY,
      options: {onFrame, onController: vi.fn()}, resizeFromEvent: () => {},
      preventContextMenu: (event: Event) => event.preventDefault(),
    });
    scene.addListeners();
    try {
      canvas.dispatchEvent(pointerEvent('pointerdown', 3));
      canvas.dispatchEvent(pointerEvent('pointermove', 3, 0, 50, -40));
      const key = new KeyboardEvent('keydown', {code: 'Space', bubbles: true, cancelable: true});
      canvas.dispatchEvent(key);
      const wheel = new WheelEvent('wheel', {deltaY: 100, cancelable: true});
      canvas.dispatchEvent(wheel);
      scene.sendDesktopFrame(20);
      expect(scene.controllerPosition.toArray()).toEqual([0.56, 0.42, 0.18]);
      expect(onFrame.mock.calls[0][0].right).toEqual({
        p: [0.56, 0.42, 0.18], q: [0, 0, 0, 1], grip: false, trigger: 0,
      });
      expect(key.defaultPrevented).toBe(false);
      expect(wheel.defaultPrevented).toBe(false);
      const beforeOrbit = camera.position.clone();
      canvas.dispatchEvent(pointerEvent('pointerup', 3));
      canvas.dispatchEvent(pointerEvent('pointerdown', 4, 2));
      canvas.dispatchEvent(pointerEvent('pointermove', 4, 2, 30, 20));
      canvas.dispatchEvent(pointerEvent('pointerup', 4, 2));
      expect(camera.position.equals(beforeOrbit)).toBe(false);
      scene.sendDesktopFrame(40);
      expect(onFrame.mock.calls[1][0].right).toEqual(onFrame.mock.calls[0][0].right);
    } finally {
      scene.removeListeners();
      canvas.remove();
    }
  });
  it('publishes neutral manual heartbeats even when obsolete canvas input is latched', () => {
    Object.defineProperty(document, 'visibilityState', {configurable: true, value: 'visible'});
    const onFrame = vi.fn();
    const scene = {
      lastFrameMs: Number.NEGATIVE_INFINITY,
      inputSafety: {snapshot: () => ({activePointer: 5, grip: true, trigger: 1})},
      offlineController: null,
      automationActive: false,
      controllerPosition: new THREE.Vector3(0.56, 0.42, 0.18),
      controllerQuaternion: new THREE.Quaternion(),
      sessionId: 'manual-neutral', sequence: 0,
      options: {onFrame, onController: vi.fn()},
    };
    (SimulationScene.prototype as unknown as {sendDesktopFrame(this: typeof scene, nowMs: number): void})
      .sendDesktopFrame.call(scene, 20);
    expect(onFrame).toHaveBeenCalledWith(expect.objectContaining({
      right: {p: [0.56, 0.42, 0.18], q: [0, 0, 0, 1], grip: false, trigger: 0},
    }), 'manual');
  });
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

describe('offline controller scene seam', () => {
  it('publishes a safe rehearsal heartbeat before a synthetic sample exists', () => {
    Object.defineProperty(document, 'visibilityState', {configurable: true, value: 'visible'});
    const frame = vi.fn();
    const scene = {
      lastFrameMs: Number.NEGATIVE_INFINITY,
      inputSafety: {snapshot: vi.fn(() => ({activePointer: 5, grip: true, trigger: 1}))},
      offlineController: null,
      automationActive: true,
      controllerPosition: new THREE.Vector3(0.56, 0.42, 0.18),
      controllerQuaternion: new THREE.Quaternion(0, 0, 0, 1),
      sessionId: 'offline',
      sequence: 0,
      options: {onFrame: frame, onController: vi.fn()},
    };

    (SimulationScene.prototype as unknown as {sendDesktopFrame(this: typeof scene, nowMs: number): void})
      .sendDesktopFrame.call(scene, 20);

    expect(scene.inputSafety.snapshot).not.toHaveBeenCalled();
    expect(frame).toHaveBeenCalledWith(expect.objectContaining({
      tracking_valid: true,
      right: expect.objectContaining({grip: false, trigger: 0}),
    }), 'offline_rehearsal');
  });

  it('publishes the exclusive synthetic sample instead of latched desktop input', () => {
    Object.defineProperty(document, 'visibilityState', {configurable: true, value: 'visible'});
    const offline: OfflineControllerSample = {
      position: [0.1, 0.2, 0.3], quaternion: [0, 0, 0, 1],
      grip: false, trigger: 0.75, trackingValid: true,
    };
    const frame = vi.fn();
    const controller = vi.fn();
    const scene = {
      lastFrameMs: Number.NEGATIVE_INFINITY,
      inputSafety: {snapshot: vi.fn(() => ({activePointer: 5, grip: true, trigger: 1}))},
      offlineController: offline,
      automationActive: true,
      sessionId: 'offline',
      sequence: 0,
      options: {onFrame: frame, onController: controller},
    };

    (SimulationScene.prototype as unknown as {sendDesktopFrame(this: typeof scene, nowMs: number): void})
      .sendDesktopFrame.call(scene, 20);

    expect(frame).toHaveBeenCalledWith(expect.objectContaining({
      tracking_valid: true,
      right: {p: [0.1, 0.2, 0.3], q: [0, 0, 0, 1], grip: false, trigger: 0.75},
    }), 'offline_rehearsal');
    expect(scene.inputSafety.snapshot).not.toHaveBeenCalled();
    expect(controller).toHaveBeenCalledWith({tracking: true, grip: false, trigger: 0.75});
  });

  it('marks an active synthetic sample invalid while the page is hidden', () => {
    Object.defineProperty(document, 'visibilityState', {configurable: true, value: 'hidden'});
    const frame = vi.fn();
    const scene = {
      lastFrameMs: Number.NEGATIVE_INFINITY,
      inputSafety: {snapshot: vi.fn()},
      offlineController: {
        position: [0.1, 0.2, 0.3], quaternion: [0, 0, 0, 1],
        grip: false, trigger: 0, trackingValid: true,
      },
      automationActive: true,
      sessionId: 'offline', sequence: 0,
      options: {onFrame: frame, onController: vi.fn()},
    };

    (SimulationScene.prototype as unknown as {sendDesktopFrame(this: typeof scene, nowMs: number): void})
      .sendDesktopFrame.call(scene, 20);

    expect(frame.mock.calls[0][0].tracking_valid).toBe(false);
  });

  it('resets the dormant synthetic controller pose before restoring manual heartbeats', () => {
    const position = new THREE.Vector3(9, 8, 7);
    const quaternion = new THREE.Quaternion(1, 0, 0, 0);
    const scene = {
      offlineController: {
        position: [0.1, 0.2, 0.3], quaternion: [0, 0, 0, 1],
        grip: true, trigger: 1, trackingValid: true,
      },
      controllerPosition: position,
      controllerQuaternion: quaternion,
    };

    SimulationScene.prototype.setOfflineController.call(scene as never, null);

    expect(scene.offlineController).toBeNull();
    expect(position.toArray()).toEqual([0.56, 0.42, 0.18]);
    expect(quaternion.toArray()).toEqual([0, 0, 0, 1]);
  });

  it('returns a copied controller observation without retired grasp props', () => {
    const controller: OfflineControllerSample = {
      position: [0.1, 0.2, 0.3], quaternion: [0, 0, 0, 1],
      grip: true, trigger: 1, trackingValid: true,
    };
    const scene = {
      offlineController: controller,
      offlineWorkspace: {
        limiterAnchor: [0.3, 0.2, -0.2],
        taskAnchor: [0.31, 0.22, -0.19],
        placementTarget: [0.27, 0.165, -0.215],
        liftM: 0.03,
      },
      graspController: {
        snapshot: vi.fn(() => ({
          carriedBlockId: 'block-orange',
          blocks: [{id: 'block-orange', position: [0.18, 0.025, -0.32], sizeM: 0.06}],
          invalidOverlap: false,
        })),
      },
    };

    const snapshot = SimulationScene.prototype.getOfflineSceneSnapshot.call(scene as never);
    snapshot.controller?.position.splice(0, 1, 9);
    expect(snapshot.blocks).toEqual([]);

    expect(controller.position).toEqual([0.1, 0.2, 0.3]);
    expect(scene.graspController.snapshot).not.toHaveBeenCalled();
    expect(SimulationScene.prototype.getOfflineSceneSnapshot.call(scene as never)).toEqual({
      controller,
      carriedBlockId: null,
      blocks: [],
      invalidOverlap: false,
      workspace: null,
    });
  });

  it('lays out the Fake block and observed placement target from distinct Home and prep anchors', () => {
    const home: Pose = {p: [-0.14378786228061097, 0.6959976154948492, -0.12063003385763715], q: [0, 0, 0, 1]};
    const prep: Pose = {p: [-0.06972270013518106, 0.7320848696673521, -0.13552758188831457], q: [0, 0, 0, 1]};

    const layout = fakeRehearsalWorkspaceLayout(home, prep);

    expect(layout.pickBlockCenter).toEqual(expect.arrayContaining([
      expect.closeTo(-0.07378786228061097, 12),
      expect.closeTo(0.6609976154948491, 12),
      expect.closeTo(-0.13563003385763714, 12),
    ]));
    expect(layout.placementTarget).toEqual(expect.arrayContaining([
      expect.closeTo(-0.11378786228061097, 12),
      expect.closeTo(0.6609976154948491, 12),
      expect.closeTo(-0.13563003385763714, 12),
    ]));
    expect(layout.supportTopY).toBeCloseTo(0.6309976154948491);
  });
});

describe('scene connection and model lifetime helpers', () => {
  it('keeps retired grasp props absent across real and Fake state changes', () => {
    const blocks = createGraspBlocks();
    const platform = new THREE.Group();
    const rehearsalSupport = new THREE.Group();
    const rehearsalPlacementMarker = new THREE.Group();
    for (const {object} of blocks) object.visible = false;
    platform.visible = rehearsalSupport.visible = rehearsalPlacementMarker.visible = false;
    const scene = {
      options: {stateBuffer: {push: vi.fn()}},
      clockAnchor: {update: vi.fn()},
      graspBlocks: blocks,
      graspWorkbench: platform,
      rehearsalSupport,
      rehearsalPlacementMarker,
      offlineWorkspace: null,
    };

    SimulationScene.prototype.applyRobotState.call(scene as never, {
      backend: 'LEBAI', server_mono_ns: 1,
    } as RobotStateMessage);
    expect(blocks.every(({object}) => !object.visible)).toBe(true);
    expect(platform.visible).toBe(false);
    expect(rehearsalSupport.visible).toBe(false);
    expect(rehearsalPlacementMarker.visible).toBe(false);

    SimulationScene.prototype.applyRobotState.call(scene as never, {
      backend: 'LEBAI_FAKE', server_mono_ns: 2,
    } as RobotStateMessage);
    expect(blocks.every(({object}) => !object.visible)).toBe(true);
    expect(platform.visible).toBe(false);
    expect(scene.options.stateBuffer.push).toHaveBeenCalledTimes(2);
    expect(scene.clockAnchor.update).toHaveBeenCalledTimes(2);
  });

  it('cannot recreate props through the removed rehearsal workspace entry', () => {
    const root = new THREE.Group();
    const scene = {robotVisualRoot: root};
    const pose: Pose = {p: [0, 0, 0], q: [0, 0, 0, 1]};
    expect(() => SimulationScene.prototype.configureFakeRehearsalWorkspace.call(scene as never, pose, pose))
      .toThrow('feature_removed');
    expect(root.children).toEqual([]);
  });

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
  it('updates robot joints and gripper without running animated grasping', () => {
    const actualTcp = {
      p: [0.2, 0.3, -0.1] as [number, number, number],
      q: [0, 0, 0, 1] as [number, number, number, number],
    };
    const updateGrasp = vi.fn();
    const scene = {
      clockAnchor: {estimate: vi.fn().mockReturnValue(123n)},
      options: {
        stateBuffer: {
          sample: vi.fn().mockReturnValue({
            state: {
              backend: 'LEBAI_FAKE',
              actual_q: [0, 0, 0, 0, 0, 0],
              actual_tcp: actualTcp,
              gripper: 0.72,
            },
          }),
        },
      },
      robotModel: {
        setJointAngles: vi.fn(),
        setGripper: vi.fn(),
      },
      robotVisualRoot: new THREE.Group(),
      graspController: {update: updateGrasp},
      targetMarker: new THREE.Group(),
      controllerPosition: new THREE.Vector3(),
      controllerQuaternion: new THREE.Quaternion(),
    };

    (SimulationScene.prototype as unknown as {
      updateScene(this: typeof scene, nowMs: number): void;
    }).updateScene.call(scene, 10);

    expect(updateGrasp).not.toHaveBeenCalled();
    expect(scene.robotModel.setJointAngles).toHaveBeenCalledWith([0, 0, 0, 0, 0, 0], 'LEBAI_FAKE');
    expect(scene.robotModel.setGripper).toHaveBeenCalledWith(0.72);
  });

  it('keeps lights and grid in the fixed scene while placing robot visuals under the height root', () => {
    const fixedScene = new THREE.Scene();
    const robotVisualRoot = new THREE.Group();
    fixedScene.add(robotVisualRoot);
    const targetMarker = new THREE.Group();
    const scene = {
      scene: fixedScene,
      robotVisualRoot,
      grid: new THREE.GridHelper(4, 40),
      targetMarker,
      controllerPosition: new THREE.Vector3(0.56, 0.42, 0.18),
    };

    (SimulationScene.prototype as unknown as {createEnvironment(this: typeof scene): void})
      .createEnvironment.call(scene);
    (SimulationScene.prototype as unknown as {createTargetMarker(this: typeof scene): void})
      .createTargetMarker.call(scene);

    expect(scene.robotVisualRoot.parent).toBe(scene.scene);
    expect(scene.targetMarker.parent).toBe(scene.robotVisualRoot);
    expect(scene.targetMarker.visible).toBe(false);
    expect(scene.grid.parent).toBe(scene.scene);
    expect(scene.robotVisualRoot.position.y).toBe(0);
  });

  it.each([
    ['locked', false, false, true],
    ['stopped', false, false, true],
    ['armed', false, false, true],
    ['locked', true, false, false],
    ['locked', false, true, false],
  ] as const)(
    'forwards both hands and enables height for phase=%s pending=%s grip=%s only when safe',
    (phase, pending, grip, expectedEnabled) => {
      const updateHints = vi.fn();
      const updatePlacement = vi.fn().mockReturnValue([0.1, 0.93, -0.2]);
      const scene = {
        controllerHints: {update: updateHints},
        workspacePlacement: {update: updatePlacement},
        robotVisualRoot: new THREE.Group(),
        armSafetyState: {phase, pending, faultResetPending: false},
      };
      const sample = presentationSample({grip});

      SimulationScene.prototype.updateXRPresentation.call(scene as never, sample, 425);

      expect(updateHints).toHaveBeenCalledWith(sample.left, sample.right);
      expect(updatePlacement).toHaveBeenCalledWith({
        headY: 1.68,
        axisX: 0,
        axisY: -0.75,
        heightModifier: false,
        resetPressed: true,
        enabled: expectedEnabled,
        nowMs: 425,
      });
      expect(scene.robotVisualRoot.position.toArray()).toEqual([0.1, 0.93, -0.2]);
      expect(scene.robotVisualRoot.position.y).toBe(0.93);
    },
  );

  it('disables height input when left-hand tracking is lost while still forwarding both hints', () => {
    const updateHints = vi.fn();
    const updatePlacement = vi.fn().mockReturnValue([0.1, 0.95, -0.2]);
    const scene = {
      controllerHints: {update: updateHints},
      workspacePlacement: {update: updatePlacement},
      robotVisualRoot: new THREE.Group(),
      armSafetyState: {
        phase: 'locked',
        pending: false,
        faultResetPending: false,
      },
    };
    const sample = presentationSample({leftTrackingValid: false});

    SimulationScene.prototype.updateXRPresentation.call(scene as never, sample, 510);

    expect(updateHints).toHaveBeenCalledWith(sample.left, sample.right);
    expect(updatePlacement).toHaveBeenCalledWith(
      expect.objectContaining({enabled: false}),
    );
  });

  it('shows the safety panel only after the XR session is installed', async () => {
    const loop = vi.fn() as unknown as XRFrameRequestCallback;
    const session = {} as XRSession;
    let installSession!: () => void;
    const setSession = vi.fn().mockReturnValue(new Promise<void>((resolve) => {
      installSession = resolve;
    }));
    const setAnimationLoop = vi.fn();
    const setVisible = vi.fn();
    const setHintsVisible = vi.fn();
    const beginSession = vi.fn();
    const cancel = vi.spyOn(window, 'cancelAnimationFrame').mockImplementation(() => {});
    const scene = {
      started: true,
      desktopInput: {setXROwned: vi.fn(), stop: vi.fn()},
      animationHandle: 42,
      renderer: {xr: {setSession}, setAnimationLoop},
      vrSafetyPanel: {setVisible},
      controllerHints: {setVisible: setHintsVisible},
      workspacePlacement: {beginSession},
      targetMarker: Object.assign(new THREE.Group(), {visible: false}),
    };

    const starting = (SimulationScene.prototype.startXR as Function).call(scene, session, loop);

    expect(cancel).toHaveBeenCalledWith(42);
    expect(scene.animationHandle).toBeNull();
    expect(setSession).toHaveBeenCalledWith(session);
    expect(setVisible).not.toHaveBeenCalledWith(true);
    expect(setHintsVisible).not.toHaveBeenCalledWith(true);
    expect(beginSession).not.toHaveBeenCalled();

    installSession();
    await starting;

    expect(setVisible).toHaveBeenLastCalledWith(true);
    expect(setHintsVisible).toHaveBeenLastCalledWith(true);
    expect(beginSession).toHaveBeenCalledOnce();
    expect(scene.targetMarker.visible).toBe(true);
    expect(setAnimationLoop).toHaveBeenCalledWith(loop);
  });

  it('keeps the safety panel hidden when XR session installation fails', async () => {
    const setVisible = vi.fn();
    const setHintsVisible = vi.fn();
    const beginSession = vi.fn();
    const scene = {
      started: true,
      desktopInput: {setXROwned: vi.fn(), stop: vi.fn()},
      animationHandle: null,
      renderer: {
        xr: {setSession: vi.fn().mockRejectedValue(new DOMException('install failed'))},
        setAnimationLoop: vi.fn(),
      },
      vrSafetyPanel: {setVisible},
      controllerHints: {setVisible: setHintsVisible},
      workspacePlacement: {beginSession},
      targetMarker: Object.assign(new THREE.Group(), {visible: false}),
    };

    await expect((SimulationScene.prototype.startXR as Function).call(
      scene,
      {} as XRSession,
      vi.fn() as unknown as XRFrameRequestCallback,
    )).rejects.toThrow();

    expect(setVisible).toHaveBeenCalledWith(false);
    expect(setVisible).not.toHaveBeenCalledWith(true);
    expect(setHintsVisible).not.toHaveBeenCalledWith(true);
    expect(beginSession).not.toHaveBeenCalled();
  });

  it('clears XR state and restores the desktop animation loop', async () => {
    const setSession = vi.fn().mockResolvedValue(undefined);
    const setAnimationLoop = vi.fn();
    const request = vi.spyOn(window, 'requestAnimationFrame').mockReturnValue(77);
    const animate = vi.fn();
    const setVisible = vi.fn();
    const setHintsVisible = vi.fn();
    const endSession = vi.fn().mockReturnValue(0);
    const robotVisualRoot = new THREE.Group();
    robotVisualRoot.position.y = 0.92;
    const scene = {
      started: true,
      desktopInput: {setXROwned: vi.fn(), stop: vi.fn()},
      animationHandle: null,
      sessionId: 'desktop-before-xr',
      sequence: 41,
      lastFrameMs: 912.5,
      animate,
      renderer: {xr: {setSession}, setAnimationLoop},
      vrSafetyPanel: {setVisible},
      controllerHints: {setVisible: setHintsVisible},
      workspacePlacement: {endSession},
      robotVisualRoot,
      targetMarker: new THREE.Group(),
    };

    await (SimulationScene.prototype.stopXR as Function).call(scene);

    expect(setAnimationLoop).toHaveBeenCalledWith(null);
    expect(setSession).toHaveBeenCalledWith(null);
    expect(setVisible).toHaveBeenCalledWith(false);
    expect(setHintsVisible).toHaveBeenCalledWith(false);
    expect(endSession).toHaveBeenCalledOnce();
    expect(robotVisualRoot.position.y).toBe(0);
    expect(scene.targetMarker.visible).toBe(false);
    expect(request).toHaveBeenCalledWith(animate);
    expect(scene.animationHandle).toBe(77);
    expect(scene.sessionId).not.toBe('desktop-before-xr');
    expect(scene.sequence).toBe(0);
    expect(scene.lastFrameMs).toBe(Number.NEGATIVE_INFINITY);

    const firstResumedEpoch = scene.sessionId;
    scene.animationHandle = null;
    scene.sequence = 9;
    scene.lastFrameMs = 123;
    await (SimulationScene.prototype.stopXR as Function).call(scene);

    expect(scene.sessionId).not.toBe(firstResumedEpoch);
    expect(scene.sequence).toBe(0);
    expect(scene.lastFrameMs).toBe(Number.NEGATIVE_INFINITY);
  });

  it('restores the desktop animation loop when clearing the XR session rejects', async () => {
    const setSession = vi.fn().mockRejectedValue(new DOMException('raw setSession failure'));
    const setAnimationLoop = vi.fn();
    const request = vi.spyOn(window, 'requestAnimationFrame').mockReturnValue(88);
    const animate = vi.fn();
    const setVisible = vi.fn();
    const setHintsVisible = vi.fn();
    const endSession = vi.fn().mockReturnValue(0);
    const robotVisualRoot = new THREE.Group();
    robotVisualRoot.position.y = 1.04;
    const scene = {
      started: true,
      desktopInput: {setXROwned: vi.fn(), stop: vi.fn()},
      animationHandle: null,
      sessionId: 'desktop-before-rejected-stop',
      sequence: 52,
      lastFrameMs: 1_024,
      animate,
      renderer: {xr: {setSession}, setAnimationLoop},
      vrSafetyPanel: {setVisible},
      controllerHints: {setVisible: setHintsVisible},
      workspacePlacement: {endSession},
      robotVisualRoot,
      targetMarker: new THREE.Group(),
    };

    await expect((SimulationScene.prototype.stopXR as Function).call(scene)).rejects.toThrow();

    expect(setAnimationLoop).toHaveBeenCalledWith(null);
    expect(setVisible).toHaveBeenCalledWith(false);
    expect(setHintsVisible).toHaveBeenCalledWith(false);
    expect(endSession).toHaveBeenCalledOnce();
    expect(robotVisualRoot.position.y).toBe(0);
    expect(scene.targetMarker.visible).toBe(false);
    expect(request).toHaveBeenCalledWith(animate);
    expect(scene.animationHandle).toBe(88);
    expect(scene.sessionId).not.toBe('desktop-before-rejected-stop');
    expect(scene.sequence).toBe(0);
    expect(scene.lastFrameMs).toBe(Number.NEGATIVE_INFINITY);
  });

  it('forwards cached arm state and controller support to the safety panel', () => {
    const snapshot = {
      phase: 'locked',
      connected: true,
      connectionState: 'connected',
      eligible: false,
      armed: false,
      pending: false,
      mode: 'READY',
      fault: null,
      faultRecoverable: false,
      faultResetPending: false,
      constraint: null,
      recoveryPhase: null,
    } satisfies ArmSafetySnapshot;
    const update = vi.fn();
    const runtimeSummary = {
      backend: null,
      realRobotMode: null,
      actualTcp: null,
      gripper: null,
      latencyMs: null,
      hardwareVerified: false,
    } as const;
    const scene = {
      armSafetyState: snapshot,
      questControllerSupported: null,
      runtimeSummary,
      vrSafetyPanel: {update},
    };

    SimulationScene.prototype.setQuestControllerSupport.call(scene as never, false);
    expect(scene.questControllerSupported).toBe(false);
    expect(update).toHaveBeenLastCalledWith(snapshot, false, runtimeSummary);

    const stopped = {...snapshot, phase: 'stopped', mode: 'DISARMED'} satisfies ArmSafetySnapshot;
    SimulationScene.prototype.setArmSafetyState.call(scene as never, stopped);
    expect(scene.armSafetyState).toBe(stopped);
    expect(update).toHaveBeenLastCalledWith(stopped, false, runtimeSummary);
  });

  it('keeps the GLB on authoritative actual joints when diagnostics runtime summary changes', () => {
    const actualQ = [0.11, -0.22, 0.33, -0.44, 0.55, -0.66];
    const setJointAngles = vi.fn();
    const update = vi.fn();
    const scene = {
      armSafetyState: {
        phase: 'locked', connected: true, connectionState: 'connected', eligible: false,
        armed: false, pending: false, mode: 'READY', fault: null, faultRecoverable: false,
        faultResetPending: false, constraint: null, recoveryPhase: null,
      },
      questControllerSupported: true,
      vrSafetyPanel: {update},
      runtimeSummary: {
        backend: null, realRobotMode: null, actualTcp: null, gripper: null, latencyMs: null,
        hardwareVerified: false,
      },
      clockAnchor: {estimate: vi.fn().mockReturnValue(1)},
      options: {
        stateBuffer: {
          sample: vi.fn().mockReturnValue({
            state: {
              backend: 'LEBAI',
              actual_q: actualQ,
              actual_tcp: {p: [0.2, 0.3, 0.4], q: [0, 0, 0, 1]},
              gripper: 0.4,
            },
          }),
        },
      },
      robotModel: {setJointAngles, setGripper: vi.fn()},
      robotVisualRoot: new THREE.Group(),
      graspController: {update: vi.fn()},
      targetMarker: new THREE.Group(),
      controllerPosition: new THREE.Vector3(),
      controllerQuaternion: new THREE.Quaternion(),
    };

    (SimulationScene.prototype.setRuntimeSummary as Function).call(scene, {
      backend: 'LEBAI_FAKE',
      realRobotMode: null,
      actualTcp: {p: [0.9, 0.8, 0.7], q: [0, 0, 0, 1]},
      gripper: 0.9,
      latencyMs: 18,
      hardwareVerified: false,
    });
    expect(setJointAngles).not.toHaveBeenCalled();

    (SimulationScene.prototype as unknown as {updateScene(this: typeof scene, nowMs: number): void})
      .updateScene.call(scene, 10);

    expect(setJointAngles).toHaveBeenCalledWith(actualQ, 'LEBAI');
    expect(scene.graspController.update).not.toHaveBeenCalled();
  });

  it('disposes owned XR visuals before generic scene traversal', () => {
    const order: string[] = [];
    const setVisible = vi.fn();
    const disposePanel = vi.fn(() => order.push('panel'));
    const disposeHints = vi.fn(() => order.push('hints'));
    const removeListeners = vi.fn();
    const disposeRenderer = vi.fn();
    const removeCanvas = vi.fn();
    const scene = {
      started: true,
      desktopInput: {setXROwned: vi.fn(), stop: vi.fn()},
      animationHandle: null,
      renderer: {
        setAnimationLoop: vi.fn(),
        dispose: disposeRenderer,
        domElement: {remove: removeCanvas},
      },
      removeListeners,
      scene: new THREE.Scene(),
      vrSafetyPanel: {setVisible, dispose: disposePanel},
      controllerHints: {setVisible: vi.fn(), dispose: disposeHints},
    };

    (SimulationScene.prototype.dispose as Function).call(scene);

    expect(setVisible).toHaveBeenCalledWith(false);
    expect(disposePanel).toHaveBeenCalledOnce();
    expect(disposeHints).toHaveBeenCalledOnce();
    expect(order).toEqual(['hints', 'panel']);
    expect(removeListeners).toHaveBeenCalledOnce();
    expect(disposeRenderer).toHaveBeenCalledOnce();
    expect(removeCanvas).toHaveBeenCalledOnce();
  });
});

function presentationSample(options: {grip?: boolean; leftTrackingValid?: boolean} = {}): XRPresentationSample {
  return {
    left: {
      p: [-0.2, 1.1, -0.4],
      q: [0, 0, 0, 1],
      trackingValid: options.leftTrackingValid ?? true,
      thumbstickX: 0,
      thumbstickY: -0.75,
      thumbstickPressed: true,
      grip: false,
    },
    right: {
      p: [0.2, 1.1, -0.4],
      q: [0, 0, 0, 1],
      trackingValid: true,
      grip: options.grip ?? false,
      trigger: 0,
      armButton: false,
      stopButton: false,
      questFaceButtonsSupported: true,
    },
    headY: 1.68,
  };
}

function pointerEvent(type: string, pointerId: number, button = 0, movementX = 0, movementY = 0): Event {
  const event = new Event(type);
  Object.defineProperties(event, {
    pointerId: {value: pointerId},
    button: {value: button},
    movementX: {value: movementX},
    movementY: {value: movementY},
  });
  return event;
}
