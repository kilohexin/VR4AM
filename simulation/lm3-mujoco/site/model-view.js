import * as THREE from 'three';
import {GLTFLoader} from 'three/examples/jsm/loaders/GLTFLoader.js';
import {createRobotModelForTest as createRobotModel} from '../../../web/src/robot/robotModel.ts';
import asset from '../../../web/public/models/Lebai_LM3.glb';
import {toWebJointAngles, toWebPosition} from './model-frame.mjs';

const GREEN = 0x6ef0c0;
const BLUE = 0x69b9ff;
const RED = 0xff797f;
const PURPLE = 0xd4a4ff;
const world = point => new THREE.Vector3(...toWebPosition(point));

function block({center, size}, color) {
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(...toWebPosition(size)),
    new THREE.MeshStandardMaterial({color, roughness: .78}),
  );
  mesh.position.copy(world(center));
  return mesh;
}

function trace(frames, color, dashed = false) {
  const geometry = new THREE.BufferGeometry().setFromPoints(frames.map(({p}) => world(p)));
  const properties = {color, depthTest: false, depthWrite: false, transparent: true};
  const material = dashed
    ? new THREE.LineDashedMaterial({...properties, dashSize: .012, gapSize: .008, opacity: .85})
    : new THREE.LineBasicMaterial(properties);
  const path = new THREE.Line(geometry, material);
  if (dashed) path.computeLineDistances();
  path.renderOrder = 1;
  return path;
}

function marker(point, text, color) {
  const group = new THREE.Group();
  group.position.copy(world(point));
  const sphere = new THREE.Mesh(new THREE.SphereGeometry(.006, 12, 8), new THREE.MeshBasicMaterial({color, depthTest: false}));
  sphere.renderOrder = 2;
  group.add(sphere);

  const canvas = document.createElement('canvas');
  canvas.width = 144;
  canvas.height = 56;
  const ctx = canvas.getContext('2d');
  ctx.font = '600 38px sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillStyle = '#' + color.toString(16).padStart(6, '0');
  ctx.fillText(text, 72, 28);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({map: texture, depthTest: false, depthWrite: false}));
  sprite.scale.set(.089, .035, 1);
  sprite.position.y = .025;
  sprite.renderOrder = 3;
  group.add(sprite);
  return group;
}

function panel(host, data, gltf, planning) {
  const renderer = new THREE.WebGLRenderer({antialias: true});
  renderer.setClearColor(0x101925);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));

  const scene = new THREE.Scene();
  scene.add(new THREE.HemisphereLight(0xdbefff, 0x6a7793, 2.2));
  const key = new THREE.DirectionalLight(0xffefe0, 2.5);
  key.position.set(-.7, 1.4, .8);
  scene.add(key);
  const rim = new THREE.DirectionalLight(0x8dc7ff, 1.1);
  rim.position.set(.9, .9, -1.4);
  scene.add(rim);

  const camera = new THREE.OrthographicCamera(-.6, .6, .36, -.36, .01, 10);
  camera.position.set(1.15, 1.04, 1.18);
  camera.lookAt(-.29, .18, -.16);

  const tableInfo = data.scene.find(({id}) => id === 'table');
  const table = block(tableInfo, 0x273748);
  const boxInfo = data.scene.find(({id}) => id === 'box');
  const box = boxInfo ? block(boxInfo, 0xc78638) : null;
  if (box) {
    const boxOutline = new THREE.LineSegments(new THREE.EdgesGeometry(box.geometry),
      new THREE.LineBasicMaterial({color: 0xf4c37a}));
    box.add(boxOutline);
    scene.add(box);
  }
  const grid = new THREE.GridHelper(1.2, 12, 0x46596e, 0x34485c);
  grid.position.copy(world(tableInfo.center));
  grid.position.y += tableInfo.size[2] / 2 + .001;
  scene.add(table, grid);

  const baseline = planning && data.baseline.length ? trace(data.baseline, RED, true) : null;
  const nominal = trace(data.nominal, planning ? GREEN : 0x9dacc2, !planning);
  if (baseline) scene.add(baseline);
  scene.add(nominal);
  const actualMujoco = planning ? null : trace(data.mujoco, GREEN);
  const actualDocker = planning ? null : trace(data.docker, BLUE);
  if (actualMujoco) scene.add(actualDocker, actualMujoco);
  let physicalTrace;
  if (data.kind !== 'short') scene.add(marker(data.nominal[0].p, '起点', 0xeaf3f9), marker(data.nominal.at(-1).p, '目标', GREEN));

  const robot = createRobotModel(gltf.scene.clone(true), gltf.animations);
  robot.setGripper(1);
  scene.add(robot.group);
  renderer.domElement.className = 'model-canvas';
  renderer.domElement.setAttribute('aria-label', planning ? 'LM3 规划三维模型' : 'LM3 反馈三维模型');
  renderer.domElement.setAttribute('role', 'img');
  host.after(renderer.domElement);

  let width = 0;
  let height = 0;
  return {
    dispose() {
      renderer.domElement.remove();
      renderer.dispose();
    },
    setPhysicalTrace(frames) {
      if (physicalTrace) {
        scene.remove(physicalTrace);
        physicalTrace.geometry.dispose();
        physicalTrace.material.dispose();
      }
      physicalTrace = frames ? trace(frames, PURPLE, true) : null;
      if (physicalTrace) scene.add(physicalTrace);
    },
    render(pose, phase) {
      const w = renderer.domElement.clientWidth;
      const h = renderer.domElement.clientHeight;
      if (w !== width || h !== height) {
        width = Math.max(1, w);
        height = Math.max(1, h);
        camera.left = -.36 * width / height;
        camera.right = -camera.left;
        camera.updateProjectionMatrix();
        renderer.setSize(width, height, false);
      }

      if (box) box.visible = phase > 0;
      if (baseline) baseline.material.color.setHex(phase === 0 ? 0x9dacc2 : RED);
      nominal.visible = phase === 2;
      if (actualMujoco) actualMujoco.visible = actualDocker.visible = phase === 2;
      if (physicalTrace) physicalTrace.visible = phase === 2;
      robot.group.visible = Boolean(pose);
      if (pose) robot.setJointAngles(toWebJointAngles(pose.q), 'LEBAI_DOCKER');
      renderer.render(scene, camera);
    },
  };
}

export async function createModelViews(planner, execution, data) {
  const gltf = await new GLTFLoader().parseAsync(asset.buffer, '');
  const planned = panel(planner, data, gltf, true);
  try {
    const feedback = panel(execution, data, gltf, false);
    planner.hidden = execution.hidden = true;
    return {planned, feedback};
  } catch (error) {
    planned.dispose();
    throw error;
  }
}
