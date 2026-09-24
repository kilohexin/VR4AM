import * as THREE from 'three';
import {describe, expect, it} from 'vitest';

import {createRobotModelForTest} from '../src/robot/robotModel';

const JOINT_AXES = ['y', 'z', 'z', 'z', 'y', 'z'] as const;

function createScene(count = 6): THREE.Group {
  const root = new THREE.Group();
  for (let index = 1; index <= count; index += 1) {
    const node = new THREE.Group();
    node.name = `Joint${index}`;
    root.add(node);
  }
  const tool = new THREE.Group();
  tool.name = 'robotgrabber';
  root.add(tool);
  return root;
}

describe('RobotModel', () => {
  it('requires all six joints', () => {
    expect(() => createRobotModelForTest(createScene(5))).toThrow(
      '缺少关节节点: Joint6',
    );
  });

  it('lists every missing joint node', () => {
    const root = createScene(4);
    expect(() => createRobotModelForTest(root)).toThrow(
      '缺少关节节点: Joint5, Joint6',
    );
  });

  it('requires exactly six joint values', () => {
    const model = createRobotModelForTest(createScene());
    expect(() => model.setJointAngles([0, 1])).toThrow('需要 6 个关节角');
    expect(() => model.setJointAngles([0, 0, 0, 0, 0, 0, 0])).toThrow(
      '需要 6 个关节角',
    );
  });

  it('rejects non-finite joint values with the joint name', () => {
    const model = createRobotModelForTest(createScene());
    expect(() => model.setJointAngles([0, 0, Number.NaN, 0, 0, 0])).toThrow(
      '关节角必须是有限数值: Joint3',
    );
  });

  it('applies all angles on the authoritative joint axes', () => {
    const root = createScene();
    const model = createRobotModelForTest(root);
    const q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6];

    model.setJointAngles(q);

    JOINT_AXES.forEach((axis, index) => {
      const joint = root.getObjectByName(`Joint${index + 1}`);
      expect(joint?.rotation[axis]).toBeCloseTo(q[index]);
    });
  });

  it('renders the captured real LM3 pose with its upper arm rising above the shoulder', () => {
    const root = new THREE.Group();
    const joint1 = new THREE.Group();
    joint1.name = 'Joint1';
    joint1.position.set(0, 0.20333, 0);
    root.add(joint1);
    const joint2 = new THREE.Group();
    joint2.name = 'Joint2';
    joint2.position.set(0, 0.0125, -0.08833);
    joint1.add(joint2);
    const joint3 = new THREE.Group();
    joint3.name = 'Joint3';
    joint3.position.set(0, 0.28, 0.04603);
    joint2.add(joint3);
    let parent = joint3;
    for (let index = 4; index <= 6; index += 1) {
      const joint = new THREE.Group();
      joint.name = `Joint${index}`;
      parent.add(joint);
      parent = joint;
    }
    const tool = new THREE.Group();
    tool.name = 'robotgrabber';
    parent.add(tool);

    const model = createRobotModelForTest(root);
    model.setJointAngles([
      0.003930825768956955,
      -1.5572781211016544,
      0.31437018771731357,
      -1.5768363761471962,
      0.3939454410888813,
      -0.004026699568199808,
    ], 'LEBAI');
    root.updateMatrixWorld(true);

    const shoulderY = joint2.getWorldPosition(new THREE.Vector3()).y;
    const elbowY = joint3.getWorldPosition(new THREE.Vector3()).y;
    expect(elbowY - shoulderY).toBeGreaterThan(0.25);
  });

  it('adds angles to the original model rotations without accumulating updates', () => {
    const root = createScene();
    const joint1 = root.getObjectByName('Joint1');
    const joint2 = root.getObjectByName('Joint2');
    if (!joint1 || !joint2) throw new Error('测试场景缺少关节');
    joint1.rotation.set(0.2, 0.3, 0.4);
    joint2.rotation.set(0.5, 0.6, 0.7);
    const model = createRobotModelForTest(root);

    model.setJointAngles([0.1, 0.2, 0, 0, 0, 0]);
    model.setJointAngles([0.4, 0.5, 0, 0, 0, 0]);

    expect(joint1.rotation.x).toBeCloseTo(0.2);
    expect(joint1.rotation.y).toBeCloseTo(0.7);
    expect(joint1.rotation.z).toBeCloseTo(0.4);
    expect(joint2.rotation.x).toBeCloseTo(0.5);
    expect(joint2.rotation.y).toBeCloseTo(0.6);
    expect(joint2.rotation.z).toBeCloseTo(1.2);
  });

  it('maps clamped gripper values to Take 001 frames 0 through 20', () => {
    const root = createScene();
    const finger = new THREE.Object3D();
    finger.name = 'Finger';
    root.add(finger);
    const times = Array.from({length: 21}, (_, index) => index / 30);
    const values = Array.from({length: 21}, (_, index) => index);
    const clip = new THREE.AnimationClip('Take 001', -1, [
      new THREE.NumberKeyframeTrack('Finger.position[x]', times, values),
    ]);
    const model = createRobotModelForTest(root, [clip]);

    model.setGripper(0.5);
    expect(finger.position.x).toBeCloseTo(10);
    model.setGripper(-1);
    expect(finger.position.x).toBeCloseTo(0);
    model.setGripper(2);
    expect(finger.position.x).toBeCloseTo(20);
  });

  it('does nothing when the gripper animation is absent', () => {
    const root = createScene();
    const model = createRobotModelForTest(root);
    expect(() => model.setGripper(0.5)).not.toThrow();
  });
});
