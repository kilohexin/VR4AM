import * as THREE from 'three';
import {GLTFLoader} from 'three/examples/jsm/loaders/GLTFLoader.js';

const JOINTS = [
  {name: 'Joint1', axis: 'y'},
  {name: 'Joint2', axis: 'z'},
  {name: 'Joint3', axis: 'z'},
  {name: 'Joint4', axis: 'z'},
  {name: 'Joint5', axis: 'y'},
  {name: 'Joint6', axis: 'z'},
] as const;

export interface RobotModel {
  group: THREE.Group;
  robot: THREE.Object3D;
  setJointAngles(q: readonly number[]): void;
  setGripper(value: number): void;
}

export async function loadRobotModel(): Promise<RobotModel> {
  const loader = new GLTFLoader();
  const gltf = await loader.loadAsync(resolvePublicAsset('models/Lebai_LM3.glb'));
  return createRobotModel(gltf.scene, gltf.animations);
}

export function createRobotModelForTest(
  scene: THREE.Object3D,
  animations: THREE.AnimationClip[] = [],
): RobotModel {
  return createRobotModel(scene, animations);
}

function createRobotModel(
  scene: THREE.Object3D,
  animations: readonly THREE.AnimationClip[],
): RobotModel {
  const joints = JOINTS.map(({name}) => scene.getObjectByName(name));
  const missing = JOINTS.filter((_, index) => !joints[index]).map(({name}) => name);
  if (missing.length > 0) {
    throw new Error(`缺少关节节点: ${missing.join(', ')}`);
  }

  const jointNodes = joints as THREE.Object3D[];
  const originalRotations = jointNodes.map((joint) => joint.rotation.clone());
  const gripper = createGripperController(scene, animations);
  const group = new THREE.Group();
  group.name = 'Lebai_LM3_visual';
  group.add(scene);

  return {
    group,
    robot: scene,
    setJointAngles(q: readonly number[]): void {
      if (q.length !== JOINTS.length) {
        throw new Error('需要 6 个关节角');
      }

      q.forEach((value, index) => {
        if (!Number.isFinite(value)) {
          throw new Error(`关节角必须是有限数值: ${JOINTS[index].name}`);
        }
      });

      JOINTS.forEach(({axis}, index) => {
        const joint = jointNodes[index];
        joint.rotation.copy(originalRotations[index]);
        joint.rotation[axis] += q[index];
      });
    },
    setGripper(value: number): void {
      if (!Number.isFinite(value)) {
        throw new Error('夹爪开合值必须是有限数值');
      }
      gripper?.set(clamp01(value));
    },
  };
}

interface GripperController {
  set(value: number): void;
}

function createGripperController(
  scene: THREE.Object3D,
  animations: readonly THREE.AnimationClip[],
): GripperController | null {
  const clip = animations.find(({name}) => name === 'Take 001');
  const track = clip?.tracks[0];
  if (!clip || !track || track.times.length === 0) return null;

  const start = track.times[0];
  const end = track.times[Math.min(20, track.times.length - 1)];
  const mixer = new THREE.AnimationMixer(scene);
  const action = mixer.clipAction(clip);
  action.play();
  action.paused = true;

  return {
    set(value: number): void {
      action.time = start + (end - start) * value;
      mixer.update(0);
    },
  };
}

function resolvePublicAsset(path: string): string {
  const rawBase = import.meta.env.BASE_URL || './';
  const normalizedBase = rawBase === '/' ? './' : rawBase;
  const base = normalizedBase.endsWith('/') ? normalizedBase : `${normalizedBase}/`;
  return `${base}${path}`;
}

function clamp01(value: number): number {
  return Math.min(1, Math.max(0, value));
}
