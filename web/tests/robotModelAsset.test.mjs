import {readFileSync} from 'node:fs';

import * as THREE from 'three';
import {GLTFLoader} from 'three/examples/jsm/loaders/GLTFLoader.js';
import {expect, it} from 'vitest';

import {createRobotModelForTest} from '../src/robot/robotModel';

it('renders normalized open wider than closed in the shipped LM3 gripper', async () => {
  const bytes = readFileSync(new URL('../public/models/Lebai_LM3.glb', import.meta.url));
  const buffer = bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
  const gltf = await new GLTFLoader().parseAsync(buffer, '');
  const model = createRobotModelForTest(gltf.scene, gltf.animations);

  const fingerCenter = (name) => {
    const finger = model.robot.getObjectByName(name);
    if (!(finger instanceof THREE.Mesh)) throw new Error(`Missing finger mesh: ${name}`);
    finger.geometry.computeBoundingBox();
    const center = finger.geometry.boundingBox?.getCenter(new THREE.Vector3());
    if (!center) throw new Error(`Missing finger bounds: ${name}`);
    return finger.localToWorld(center);
  };
  const fingerGap = () => {
    model.group.updateMatrixWorld(true);
    return fingerCenter('polySurface42_black_claw_0').distanceTo(
      fingerCenter('polySurface40_black_claw_0'),
    );
  };

  model.setGripper(0);
  const openGap = fingerGap();
  model.setGripper(1);
  const closedGap = fingerGap();

  expect(openGap).toBeGreaterThan(closedGap * 1.5);
});
