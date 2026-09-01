import * as THREE from 'three';
import {describe, expect, it} from 'vitest';

import {
  type GraspBlock,
  KinematicGraspController,
  type TcpPose,
} from '../src/scenes/kinematicGraspController';
import {createGraspBlocks} from '../src/scenes/simulationScene';


const TABLE_TOP_Y = 0.53;

function pose(
  p: readonly [number, number, number],
  q: readonly [number, number, number, number] = [0, 0, 0, 1],
): TcpPose {
  return {p, q};
}

function createAcceptanceWorld(): {
  visualRoot: THREE.Group;
  blocks: GraspBlock[];
  controller: KinematicGraspController;
} {
  const visualRoot = new THREE.Group();
  const blocks = createGraspBlocks();
  for (const block of blocks) visualRoot.add(block.object);
  return {
    visualRoot,
    blocks,
    controller: new KinematicGraspController({
      visualRoot,
      blocks,
      tableTopY: TABLE_TOP_Y,
      tableHalfWidth: 0.61,
      tableHalfDepth: 0.43,
    }),
  };
}

function positionOf(block: GraspBlock): [number, number, number] {
  return [
    block.object.position.x,
    block.object.position.y,
    block.object.position.z,
  ];
}

function strictlyOverlap(first: GraspBlock, second: GraspBlock): boolean {
  const threshold = (first.sizeM + second.sizeM) / 2 - 1e-9;
  return (
    Math.abs(first.object.position.x - second.object.position.x) < threshold
    && Math.abs(first.object.position.y - second.object.position.y) < threshold
    && Math.abs(first.object.position.z - second.object.position.z) < threshold
  );
}

function expectNoStaticOverlap(blocks: readonly GraspBlock[]): void {
  for (let first = 0; first < blocks.length; first += 1) {
    for (let second = first + 1; second < blocks.length; second += 1) {
      expect(strictlyOverlap(blocks[first], blocks[second])).toBe(false);
    }
  }
}

describe('virtual LM3 interaction acceptance', () => {
  it('uses authoritative actual TCP instead of an unexecuted target', () => {
    const {blocks, controller} = createAcceptanceWorld();
    const actualTcp = positionOf(blocks[0]);
    const unexecutedTarget: [number, number, number] = [0.45, 0.35, -0.4];

    controller.update(pose(actualTcp), 0.2);
    controller.update(pose(actualTcp), 0.7);
    controller.update(pose(actualTcp), 0.7);

    expect(positionOf(blocks[0])).toEqual(actualTcp);
    expect(positionOf(blocks[0])).not.toEqual(unexecutedTarget);
  });

  it('grasps releases and stacks all five blocks without penetration then resets', () => {
    const {visualRoot, blocks, controller} = createAcceptanceWorld();
    const initial = blocks.map((block) => ({
      position: positionOf(block),
      quaternion: block.object.quaternion.toArray(),
    }));

    for (const block of blocks) {
      const blockTcp = positionOf(block);
      controller.update(pose(blockTcp), 0.2);
      controller.update(pose(blockTcp), 0.7);
      controller.update(pose([-0.07, 0.72, -0.13]), 0.7);
      controller.update(pose([-0.07, 0.72, -0.13]), 0.2);

      expectNoStaticOverlap(blocks);
      expect(block.object.parent).toBe(visualRoot);
      expect(block.object.getWorldScale(new THREE.Vector3()).toArray())
        .toEqual([1, 1, 1]);
    }

    expect(blocks.map(({object}) => object.position.y)).toEqual([
      0.56,
      0.62,
      0.68,
      0.74,
      0.80,
    ]);

    controller.reset();

    for (const [index, block] of blocks.entries()) {
      expect(positionOf(block)).toEqual(initial[index].position);
      expect(block.object.quaternion.toArray()).toEqual(initial[index].quaternion);
      expect(block.object.parent).toBe(visualRoot);
    }
    expectNoStaticOverlap(blocks);
  });
});
