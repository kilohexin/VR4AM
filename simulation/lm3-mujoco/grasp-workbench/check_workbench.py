"""Headless acceptance test with the real MuJoCo engine and isolated MoveIt."""

import argparse
import asyncio

import mujoco
import numpy as np

from physics import Physics
from scene import GRIPPER, HERE, model_directory
from task import MoveIt, Task


async def check(insert_obstacle, frictionless=False):
    archive = model_directory()
    p = Physics(archive)
    assert np.all(p.model.eq_type == mujoco.mjtEq.mjEQ_JOINT)
    for ids in (p.model.eq_obj1id, p.model.eq_obj2id):
        assert all(p.model.body(p.model.jnt_bodyid[joint]).name in GRIPPER for joint in ids)
    assert len(p.model.eq_type) == 5
    assert .089 <= p.jaw_gap() <= .092
    for name in GRIPPER:
        body = p.model.body(name).id
        mask = (p.model.geom_bodyid == body) & (p.model.geom_contype != 0)
        assert mask.sum() == 1
        assert np.all(p.model.geom_type[mask] == mujoco.mjtGeom.mjGEOM_MESH)
    if frictionless:
        p.model.geom_condim[p.fingers] = 1
    planner = MoveIt(archive,HERE,HERE/'moveit-workbench-test.log')
    try:
        task = Task(p,planner)
        p.step(200)
        assert task.start()
        inserted = False
        history = []
        held_gaps = []
        held_forces = []
        max_slip = 0.
        max_lift = 0.
        cup_height = p.pos('cup')[2]
        for _ in range(4400):
            task.tick()
            p.step(20)
            if p.relative is not None:
                held_gaps.append(p.jaw_gap())
                held_forces.append(p.finger_forces())
                max_slip = max(max_slip, float(np.linalg.norm(p.relative_pose()[0]-p.relative[0])))
            max_lift = max(max_lift, float(p.pos('cup')[2]-cup_height))
            for contact in p.data.contact:
                groups = {p.geometry[index] for index in contact.geom}
                assert not (contact.dist < -.001 and 'block' in groups and groups & {'arm','cup'}), groups
            current = task.stage,task.status
            if not history or history[-1] != current:
                history.append(current)
                print(*current, flush=True)
            if insert_obstacle and not inserted and current == ('carry','moving'):
                assert task.drag('block',-.49,.005,False)
                inserted = True
            if task.status in ('done','blocked','failed'):
                if frictionless:
                    assert task.status == 'failed' and task.events[-1]['event'] == 'grasp_lost', task.detail
                    assert any(event['event'] == 'bilateral_grasp' for event in task.events)
                    print(f'PASS frictionless control: grip lost, peak lift {max_lift:.4f} m', flush=True)
                    return
                assert task.status=='done', (task.stage,task.detail,task.events)
                if insert_obstacle:
                    assert inserted and task.replans>=1, task.events
                    assert ('carry','braking') in history
                    assert history.count(('carry','moving'))>=2
                assert p.relative is None
                assert held_gaps and .050 <= min(held_gaps) <= max(held_gaps) <= .060, held_gaps[:10]
                cup, tray = p.pos('cup'), p.pos('tray')
                assert np.linalg.norm(cup[:2]-tray[:2])<.045
                assert max_lift > .14
                print(f'Gap: {min(held_gaps)*1000:.2f}–{max(held_gaps)*1000:.2f} mm; '
                      f'normal forces, L/R median: {np.median(held_forces, axis=0).round(2)} N; '
                      f'max relative translation: {max_slip*1000:.2f} mm; '
                      f'peak cup lift: {max_lift*1000:.2f} mm; '
                      f'final cup relative to tray: {((cup-tray)*1000).round(2)} mm', flush=True)
                print('PASS', task.replans, task.events, flush=True)
                return
            await asyncio.sleep(.022 if task.status=='planning' else .002)
        raise AssertionError(f'Timed out at {task.stage}: {task.status}')
    finally:
        planner.close()


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--dynamic',action='store_true')
    mode.add_argument('--frictionless',action='store_true')
    args = parser.parse_args()
    asyncio.run(check(args.dynamic, args.frictionless))
