import * as THREE from 'three';
import {describe, expect, it} from 'vitest';

import {
  type GraspBlock,
  KinematicGraspController,
  type TcpPose,
} from '../src/scenes/kinematicGraspController';


const TABLE_TOP_Y = -0.005;
const BLOCK_SIZE_M = 0.06;

function pose(
  p: readonly [number, number, number],
  q: readonly [number, number, number, number] = [0, 0, 0, 1],
): TcpPose {
  return {p, q};
}

function createWorld(
  positions: readonly (readonly [number, number, number])[],
): {
  visualRoot: THREE.Group;
  scaledTool: THREE.Group;
  blocks: GraspBlock[];
  controller: KinematicGraspController;
} {
  const visualRoot = new THREE.Group();
  const scaledTool = new THREE.Group();
  scaledTool.scale.setScalar(0.007950832135975361);
  visualRoot.add(scaledTool);
  const blocks = positions.map((position, index) => {
    const object = new THREE.Mesh(
      new THREE.BoxGeometry(BLOCK_SIZE_M, BLOCK_SIZE_M, BLOCK_SIZE_M),
    );
    object.position.set(...position);
    visualRoot.add(object);
    return {
      id: `block-${index}`,
      object,
      sizeM: BLOCK_SIZE_M,
    };
  });
  const controller = new KinematicGraspController({
    visualRoot,
    blocks,
    tableTopY: TABLE_TOP_Y,
    tableHalfWidth: 0.61,
    tableHalfDepth: 0.43,
  });
  return {visualRoot, scaledTool, blocks, controller};
}

function strictlyOverlap(a: GraspBlock, b: GraspBlock): boolean {
  const threshold = (a.sizeM + b.sizeM) / 2 - 1e-9;
  return (
    Math.abs(a.object.position.x - b.object.position.x) < threshold
    && Math.abs(a.object.position.y - b.object.position.y) < threshold
    && Math.abs(a.object.position.z - b.object.position.z) < threshold
  );
}

function positionOf(block: GraspBlock): [number, number, number] {
  return [
    block.object.position.x,
    block.object.position.y,
    block.object.position.z,
  ];
}

describe('KinematicGraspController', () => {
  it('captures the nearest block within the expanded VR assist volume', () => {
    const {blocks, controller} = createWorld([
      [0.09, 0.07, 0.09],
      [0.20, 0, 0],
    ]);

    controller.update(pose([0, 0, 0]), 0.2);
    controller.update(pose([0, 0, 0]), 0.7);
    controller.update(pose([0.3, 0.2, -0.1]), 0.7);

    expect(blocks[0].object.position.toArray()).toEqual([0.3, 0.2, -0.1]);
    expect(blocks[1].object.position.toArray()).toEqual([0.20, 0, 0]);
  });

  it('selects the closest block inside the oriented TCP capture box', () => {
    const {blocks, controller} = createWorld([
      [0, 0.04, 0],
      [0, 0.05, 0],
      [0.2, 0, 0],
    ]);
    const quarterTurnZ = Math.sin(Math.PI / 4);

    controller.update(
      pose([0, 0, 0], [0, 0, quarterTurnZ, quarterTurnZ]),
      0.7,
    );
    controller.update(pose([0.1, 0.2, 0.3]), 0.7);

    expect(blocks[0].object.position.toArray()).toEqual([0.1, 0.2, 0.3]);
    expect(blocks[1].object.position.toArray()).toEqual([0, 0.05, 0]);
  });

  it('keeps a captured block under the unscaled visual root at world scale one', () => {
    const {visualRoot, scaledTool, blocks, controller} = createWorld([
      [0, 0, 0],
    ]);

    controller.update(pose([0, 0, 0]), 0.7);

    expect(scaledTool.scale.toArray()).toEqual([
      0.007950832135975361,
      0.007950832135975361,
      0.007950832135975361,
    ]);
    expect(blocks[0].object.parent).toBe(visualRoot);
    expect(blocks[0].object.getWorldScale(new THREE.Vector3()).toArray())
      .toEqual([1, 1, 1]);
  });

  it('moves and rotates only the carried block from authoritative TCP', () => {
    const {blocks, controller} = createWorld([
      [0, 0, 0],
      [0.3, 0.025, 0.2],
    ]);
    const quarterTurnY = Math.sin(Math.PI / 4);

    controller.update(pose([0, 0, 0]), 0.7);
    controller.update(
      pose([0.12, 0.18, -0.2], [0, quarterTurnY, 0, quarterTurnY]),
      0.7,
    );

    expect(blocks[0].object.position.toArray()).toEqual([0.12, 0.18, -0.2]);
    expect(blocks[0].object.quaternion.x).toBeCloseTo(0);
    expect(blocks[0].object.quaternion.y).toBeCloseTo(quarterTurnY);
    expect(blocks[0].object.quaternion.z).toBeCloseTo(0);
    expect(blocks[0].object.quaternion.w).toBeCloseTo(quarterTurnY);
    expect(blocks[1].object.position.toArray()).toEqual([0.3, 0.025, 0.2]);
    expect(blocks[1].object.quaternion.toArray()).toEqual([0, 0, 0, 1]);
  });

  it('releases a block onto an empty table at center Y 0.025', () => {
    const {blocks, controller} = createWorld([[0, 0.2, 0]]);

    controller.update(pose([0, 0.2, 0]), 0.7);
    controller.update(pose([0.2, 0.1, -0.2]), 0.7);
    controller.update(pose([0.2, 0.1, -0.2]), 0.2);

    expect(blocks[0].object.position.toArray()).toEqual([0.2, 0.025, -0.2]);
  });

  it('releases onto a configured Fake rehearsal support plane and reset restores the table', () => {
    const {blocks, controller} = createWorld([[0, 0.66, 0]]);
    controller.setTableTopY(0.63);
    controller.update(pose([0, 0.66, 0]), 0.7);
    controller.update(pose([0.1, 0.69, 0]), 0.7);
    controller.update(pose([0.1, 0.69, 0]), 0.2);
    expect(blocks[0].object.position.toArray()).toEqual([0.1, 0.66, 0]);

    controller.reset();
    controller.update(pose([0, 0.66, 0]), 0.7);
    controller.update(pose([0.1, 0.69, 0]), 0.7);
    controller.update(pose([0.1, 0.69, 0]), 0.2);
    expect(blocks[0].object.position.toArray()).toEqual([0.1, 0.025, 0]);
  });

  it('releases a block on one support block at center Y 0.085', () => {
    const {blocks, controller} = createWorld([
      [0.2, 0.15, 0],
      [0, 0.025, 0],
    ]);

    controller.update(pose([0.2, 0.15, 0]), 0.7);
    controller.update(pose([0.01, 0.15, 0.01]), 0.7);
    controller.update(pose([0.01, 0.15, 0.01]), 0.2);

    expect(blocks[0].object.position.toArray()).toEqual([0, 0.085, 0]);
  });

  it('stacks five sequential releases at exact deterministic heights', () => {
    const {blocks, controller} = createWorld([
      [0.15, 0.2, -0.3],
      [0.27, 0.2, -0.3],
      [0.39, 0.2, -0.3],
      [0.51, 0.2, -0.3],
      [0.63, 0.2, -0.3],
    ]);

    for (const block of blocks) {
      controller.update(pose(positionOf(block)), 0.2);
      controller.update(pose(positionOf(block)), 0.7);
      controller.update(pose([0, 0.3, 0]), 0.7);
      controller.update(pose([0, 0.3, 0]), 0.2);

      for (let first = 0; first < blocks.length; first += 1) {
        for (let second = first + 1; second < blocks.length; second += 1) {
          expect(strictlyOverlap(blocks[first], blocks[second])).toBe(false);
        }
      }
    }

    expect(blocks.map(({object}) => object.position.y)).toEqual([
      0.025,
      0.085,
      0.145,
      0.205,
      0.265,
    ]);
  });

  it('consumes an empty close attempt instead of vacuum-grabbing later', () => {
    const {blocks, controller} = createWorld([[0.2, 0.025, 0]]);

    controller.update(pose([0, 0.2, 0]), 0.7);
    controller.update(pose([0.2, 0.025, 0]), 0.7);
    controller.update(pose([0.3, 0.2, 0]), 0.7);
    expect(blocks[0].object.position.toArray()).toEqual([0.2, 0.025, 0]);

    controller.update(pose([0.2, 0.025, 0]), 0.2);
    controller.update(pose([0.2, 0.025, 0]), 0.7);
    controller.update(pose([0.3, 0.2, 0]), 0.7);

    expect(blocks[0].object.position.toArray()).toEqual([0.3, 0.2, 0]);
  });

  it('reset restores every initial transform and direct parent', () => {
    const {visualRoot, blocks, controller} = createWorld([
      [0.1, 0.025, -0.2],
      [0.3, 0.025, -0.1],
    ]);

    controller.update(pose([0.1, 0.025, -0.2]), 0.7);
    controller.update(pose([0, 0.3, 0]), 0.7);
    blocks[0].object.scale.set(0.8, 0.9, 1.1);
    new THREE.Group().add(blocks[1].object);
    controller.reset();

    expect(blocks[0].object.parent).toBe(visualRoot);
    expect(blocks[0].object.position.toArray()).toEqual([0.1, 0.025, -0.2]);
    expect(blocks[0].object.scale.toArray()).toEqual([1, 1, 1]);
    expect(blocks[1].object.parent).toBe(visualRoot);
    expect(blocks[1].object.position.toArray()).toEqual([0.3, 0.025, -0.1]);
  });

  it('publishes copied finite grasp state without exposing Three.js block objects', () => {
    const {blocks, controller} = createWorld([[0.1, 0.025, -0.2]]);

    controller.update(pose([0.1, 0.025, -0.2]), 0.7);
    const snapshot = controller.snapshot();
    snapshot.blocks[0].position[0] = 99;

    expect(snapshot.carriedBlockId).toBe('block-0');
    expect(snapshot.invalidOverlap).toBe(false);
    expect(snapshot.blocks).toEqual([{id: 'block-0', position: [99, 0.025, -0.2], sizeM: 0.06}]);
    expect(blocks[0].object.position.toArray()).toEqual([0.1, 0.025, -0.2]);
    expect(controller.snapshot().blocks[0].position).toEqual([0.1, 0.025, -0.2]);
  });
});
