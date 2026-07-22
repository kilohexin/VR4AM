import * as THREE from 'three';
import {describe, expect, it} from 'vitest';

import {KinematicGraspController} from '../src/scenes/kinematicGraspController';


describe('KinematicGraspController', () => {
  it('attaches a nearby cube, follows the tool, and releases to the tabletop', () => {
    const visualRoot = new THREE.Group();
    const tool = new THREE.Group();
    tool.position.set(0.1, 0.25, -0.2);
    visualRoot.add(tool);
    const cube = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, 0.06));
    visualRoot.add(cube);
    const tcpOffset = new THREE.Vector3(0, -0.09, 0);
    visualRoot.updateMatrixWorld(true);
    cube.position.copy(tool.localToWorld(tcpOffset.clone()));
    const controller = new KinematicGraspController({
      visualRoot,
      tool,
      cube,
      tcpOffset,
      tableRestY: 0.025,
      tableHalfWidth: 0.56,
      tableHalfDepth: 0.38,
    });

    controller.update(0.70);

    expect(cube.parent).toBe(tool);
    expect(cube.position.toArray()).toEqual([0, -0.09, 0]);
    const attachedWorld = cube.getWorldPosition(new THREE.Vector3()).clone();
    tool.position.x += 0.2;
    visualRoot.updateMatrixWorld(true);
    expect(cube.getWorldPosition(new THREE.Vector3()).x).toBeCloseTo(
      attachedWorld.x + 0.2,
    );

    controller.update(0.20);

    expect(cube.parent).toBe(visualRoot);
    expect(cube.position.y).toBeCloseTo(0.025);
  });
});
