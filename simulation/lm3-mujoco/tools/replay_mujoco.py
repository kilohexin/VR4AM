"""Replay a saved LM3 joint reference in MuJoCo without a controller connection."""

import argparse
import json
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

from trajectory import Trajectory, path_hash


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_EXPERIMENT = ROOT / 'experiments/site-detour-eight-full'


def load_model(scene):
    directory = ROOT / 'model'
    xml = ET.parse(directory / 'lm3-with-gripper-compensated.xml').getroot()
    xml.find('compiler').set('meshdir', (directory / 'assets').as_posix())
    world = xml.find('worldbody')
    for obstacle in scene:
        ET.SubElement(world, 'geom', name=obstacle['id'], type='box',
                      pos=' '.join(map(str, obstacle['center'])),
                      size=' '.join(str(size / 2) for size in obstacle['size']),
                      rgba='.30 .45 .60 1')
    return mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding='unicode'))


def run(experiment):
    points = json.loads((experiment / 'shared-trajectory.json').read_bytes())
    summary = json.loads((experiment / 'summary.json').read_bytes())
    reference = Trajectory(points)
    identity = path_hash(points)
    if identity != summary['pathHash'] or reference.times[-1] != summary['planDuration']:
        raise ValueError('Reference and experiment summary describe different paths')
    model = load_model(summary['scene'])
    state = mujoco.MjData(model)
    state.qpos[:] = state.ctrl[:] = points[0]['q']
    mujoco.mj_forward(model, state)
    tool = model.site('tool0').id
    offset = np.array(summary['dynamics']['tipOffset'])
    step = model.opt.timestep
    sample_every = round(.02 / step)
    frames = []

    def sample(time):
        point = state.site_xpos[tool] + state.site_xmat[tool].reshape(3, 3) @ offset
        frames.append(dict(t=time, q=state.qpos.tolist(), p=point.tolist()))

    sample(0.)
    for index in range(1, round((reference.times[-1] + 2.9) / step) + 1):
        time = index * step
        state.ctrl[:] = reference.sample(time)[0]
        mujoco.mj_step(model, state)
        if index % sample_every == 0:
            mujoco.mj_forward(model, state)
            sample(time)
    return dict(mujocoVersion=mujoco.__version__, pathHash=identity, frames=frames)


def verify(result):
    text = (ROOT / 'site/demo-data.js').read_text(encoding='utf-8').strip()
    expected = json.loads(text.removeprefix('window.LM3_DEMO = ').removesuffix(';'))
    if result['pathHash'] != expected['pathHash']:
        raise ValueError('Verification applies to the published 8-second reference')
    actual, saved = result['frames'], expected['mujoco']
    if [f['t'] for f in actual] != [f['t'] for f in saved]:
        raise ValueError('Replay timestamps differ from the published frames')
    peak = 1000 * np.linalg.norm(np.array([f['p'] for f in actual]) - [f['p'] for f in saved], axis=1).max()
    if peak > .001:
        raise ValueError(f'Published MuJoCo replay differs by {peak:.6g} mm')
    print(f'Verified {len(actual)} published MuJoCo frames, maximum difference {peak:.6g} mm')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--experiment', type=Path, default=DEFAULT_EXPERIMENT)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--verify', action='store_true')
    options = parser.parse_args()
    result = run(options.experiment)
    if options.verify:
        verify(result)
    if options.output:
        with options.output.open('x', encoding='utf-8') as handle:
            json.dump(result, handle, allow_nan=False)
    print(f"MuJoCo {result['mujocoVersion']} | {len(result['frames'])} frames | {result['pathHash']}")


if __name__ == '__main__':
    main()
