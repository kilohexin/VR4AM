import {readFileSync} from 'node:fs';

import * as THREE from 'three';
import {GLTFLoader} from 'three/examples/jsm/loaders/GLTFLoader.js';
import {expect, it} from 'vitest';

import {createRobotModelForTest} from '../src/robot/robotModel';

// Independent controller reference: the six MDH alpha values used by the
// official local L Master UI (Ui-D7LdMbF-.js). No visual offsets are used here.
function controllerToolQuaternion(q) {
  const rotation = new THREE.Matrix4().makeRotationX(-Math.PI / 2); // base Z-up -> scene Y-up
  const alpha = [0, 1.5708, 0, 0, 1.5708, -1.5708];
  q.forEach((angle, index) => {
    rotation.multiply(new THREE.Matrix4().makeRotationX(alpha[index]));
    rotation.multiply(new THREE.Matrix4().makeRotationZ(angle));
  });
  // GLB tool forward is -Y; controller flange forward is +Z, with X unchanged.
  rotation.multiply(new THREE.Matrix4().makeRotationX(-Math.PI / 2));
  return new THREE.Quaternion().setFromRotationMatrix(rotation);
}

it.each(['LEBAI', 'LEBAI_DOCKER'])('matches %s flange direction and roll across wrist poses in the shipped asset', async (backend) => {
  const bytes = readFileSync(new URL('../public/models/Lebai_LM3.glb', import.meta.url));
  const gltf = await new GLTFLoader().parseAsync(
    bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength), '',
  );
  const model = createRobotModelForTest(gltf.scene, gltf.animations);
  const poses = [
    [0, 0, 0, 0, 0, 0],
    [0, -Math.PI / 4, Math.PI / 2, -Math.PI / 4, Math.PI / 2, 0],
    [-0.05, -0.98, 0.85, -0.66, 1.59, 0.36],
    [0.3, -1.1, 1.2, -0.8, -0.7, -1.4],
    [0.3, -1.1, 1.2, -0.8, -0.7, 1.4],
  ];
  for (const q of poses) {
    const original = [...q];
    for (const opening of [0, 0.5, 1]) {
      model.setJointAngles(q, backend);
      model.setGripper(opening);
      model.group.updateMatrixWorld(true);
      const actual = model.tool.getWorldQuaternion(new THREE.Quaternion());
      expect(actual.angleTo(controllerToolQuaternion(q))).toBeLessThan(0.00002);
      expect(q).toEqual(original);
    }
  }
});

it('matches a frozen official Docker TCP orientation rather than only the mathematical reference', async () => {
  const bytes = readFileSync(new URL('../public/models/Lebai_LM3.glb', import.meta.url));
  const gltf = await new GLTFLoader().parseAsync(
    bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength), '',
  );
  const model = createRobotModelForTest(gltf.scene, gltf.animations);
  // Local official simulator read on 2026-10-29, with zero TCP tool rotation.
  model.setJointAngles([
    -0.052423612124578314, -0.9778784926001508, 0.8539922789863466,
    -0.6659604947568813, 1.588614925277415, 0.36093977758923257,
  ], 'LEBAI_DOCKER');
  const controller = new THREE.Quaternion(
    0.20127993373596909, -0.32313886963268845, -0.5483689564796842, 0.7445529845330919,
  );
  const frameRotation = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
  const expected = frameRotation.clone().multiply(controller).multiply(frameRotation);
  model.group.updateMatrixWorld(true);
  expect(model.tool.getWorldQuaternion(new THREE.Quaternion()).angleTo(expected)).toBeLessThan(0.00002);
});

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
