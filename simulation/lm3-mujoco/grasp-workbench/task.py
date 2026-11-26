"""Supervise one simulated pick-and-place task."""

import asyncio
import json
import math
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from scene import (BLOCK_SIZE, CUP_HEIGHT, CUP_RADIUS, APPROACH, FLOOR, LIFT, START,
    TABLE_CENTER, TABLE_SIZE, candidates, inside_tray, planner_description, tray_boxes)
from trajectory import Trajectory

TICK = .02
STAGES = ('approach','lower','grip','lift','carry','place','release','retreat','settle')


def mount(source, destination):
    return ['--mount',f'type=bind,source={source},target={destination},readonly']


class MoveIt:
    def __init__(self, archive, directory, logfile):
        image = next((image for image in ('vr4arm-moveit-jazzy:local','moveit/moveit2:jazzy-release')
                      if subprocess.run(['docker','image','inspect',image],capture_output=True,
                                        timeout=10).returncode==0), None)
        if image is None:
            raise RuntimeError('MoveIt 2 Jazzy Docker image is missing. Run the repository planner setup.')
        command = ['docker','run','--rm','--network','none','--pull','never','-i',
                   '--env','ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST']
        command += mount(archive,'/model')+mount(directory,'/workbench')
        self.log = logfile.open('w',encoding='utf-8')
        self.collision_assets = TemporaryDirectory(prefix='lmg90-collision-',dir=directory)
        try:
            collision_path = Path(self.collision_assets.name)
            description = planner_description(archive, collision_path)
            command += mount(collision_path,'/gripper_collision')
            command += [image,'python3','-u','/workbench/moveit_worker.py']
            self.process = subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                stderr=self.log,text=True,encoding='utf-8',errors='replace',bufsize=1)
            self.send(dict(start=START,**description))
            assert self.read()['ready']
        except BaseException:
            self.close()
            raise

    def close(self):
        process = getattr(self,'process',None)
        if process is not None:
            try:
                process.stdin.close()
                process.wait(timeout=7)
            except (TimeoutError, subprocess.TimeoutExpired, OSError):
                process.terminate()
                process.wait(timeout=7)
        self.log.close()
        self.collision_assets.cleanup()

    def send(self, request):
        self.process.stdin.write(json.dumps(request,allow_nan=False)+'\n')
        self.process.stdin.flush()

    def read(self):
        prefix = 'LM3_WORKBENCH '
        for line in self.process.stdout:
            if line.startswith(prefix):
                return json.loads(line[len(prefix):])
        raise RuntimeError(f'MoveIt exited. See {self.log.name}')

    def solve(self, request):
        self.send(request)
        result = self.read()
        if 'error' in result:
            raise RuntimeError(result['error'])
        return Trajectory(result['path'])


class Task:
    def __init__(self, physics, planner):
        self.physics, self.planner = physics, planner
        self.future = None
        self.version = 0
        self.clear()

    def clear(self):
        self.stage, self.status = 'idle','idle'
        self.detail = '选择杯子和托盘后开始'
        self.route = None
        self.elapsed = self.wait = 0.
        self.replans = 0
        self.scene_retries = 0
        self.events = []
        self.preview = []
        self.editing = False
        self.edited = False
        self.candidate = 0
        self.target_origin = None
        self.grasp_options = []
        self.orientation = None
        self.transport_level = LIFT
        self.place_attempt = 0
        self.retreat_goal = None
        self.grip_time = self.lost_grip_time = 0.

    def record(self, event):
        self.events.append({'event':event,'t':round(float(self.physics.data.time),2)})
        self.events = self.events[-12:]

    def start(self):
        if self.status not in ('idle','done','failed','blocked') or self.future is not None:
            return False
        if self.physics.relative is not None:
            return False
        self.clear()
        self.grasp_options = candidates(self.physics.pos('cup'))
        self.target_origin = self.physics.pos('cup')
        self.stage, self.status = 'approach','request'
        self.record('task_started')
        return True

    def reset(self):
        self.version += 1
        self.physics.reset()
        self.clear()

    def stop(self, detail):
        self.version += 1
        self.route = None
        self.preview = []
        self.physics.data.ctrl[:6] = self.physics.data.qpos[:6]
        self.status, self.detail = 'braking', detail
        self.wait = 0.
        self.replans += 1
        self.scene_retries += 1
        self.edited = False
        self.record('stopped_for_scene_change')

    def drag(self, name, x, y, ongoing):
        if name == 'cup' and self.physics.relative is not None:
            return False
        if not self.physics.drag(name,x,y):
            if self.editing and not ongoing:
                self.editing = False
            return False
        self.editing = ongoing
        self.version += 1
        self.edited = True
        self.scene_retries = 0
        if self.status == 'blocked':
            self.status = 'request'
        if self.status == 'moving' and name != 'block':
            self.stop('目标或托盘已移动，准备重新规划')
        return True

    def next_stage(self):
        self.stage = STAGES[STAGES.index(self.stage)+1]
        self.route = None
        self.preview = []
        self.elapsed = self.wait = 0.
        self.scene_retries = 0
        self.status = 'request'

    def request(self):
        p = self.physics
        target = p.pos('tray')
        grasp, approach, attitude = self.grasp_options[self.candidate]
        if self.stage == 'approach':
            self.orientation = attitude
            goal = approach
        elif self.stage == 'lower':
            goal = grasp
        elif self.stage == 'lift':
            goal = grasp+[0,0,LIFT]
        elif self.stage == 'retreat':
            if self.retreat_goal is None:
                current, rotation = p.pose('tool0')
                self.retreat_goal = np.asarray(current)+[0,0,APPROACH]
                self.orientation = np.asarray(rotation)
            goal = self.retreat_goal
        else:
            placement_offsets = ((0.,0.,.029), (0.,.027,.039), (-.024,0.,.052))
            dx,dy,height = placement_offsets[self.place_attempt]
            destination = target+[dx,dy,FLOOR+height]
            offset = Rotation.from_quat(self.orientation).apply(p.relative_pose()[0])
            goal = destination-offset
            if self.stage == 'carry':
                goal[2] += self.transport_level
        shapes = [dict(id='table',center=TABLE_CENTER,size=TABLE_SIZE)]
        shapes += [dict(id=f'tray_{i}',center=center,size=size)
                   for i,(center,size) in enumerate(tray_boxes(target))]
        shapes += [dict(id='obstacle',center=(p.pos('block')+[0,0,BLOCK_SIZE[2]/2]).tolist(),
                        size=[d+.012 for d in BLOCK_SIZE])]
        held = None
        if p.relative is not None:
            relative, rotation = p.relative_pose()
            held = dict(id='payload',center=(relative+rotation.apply([0,0,CUP_HEIGHT/2])).tolist(),
                        rotation=rotation.as_quat().tolist(),cylinder=[CUP_HEIGHT,CUP_RADIUS])
        else:
            position, quaternion = p.pose('cup')
            rotation = Rotation.from_quat(quaternion)
            shapes.append(dict(id='cup',center=(position+rotation.apply([0,0,CUP_HEIGHT/2])).tolist(),
                               rotation=quaternion,cylinder=[CUP_HEIGHT,CUP_RADIUS]))
        return dict(start=p.data.qpos[:6].tolist(),jaw=p.jaw_angle(),
                    goal=[goal.tolist(),self.orientation.tolist()],
                    cartesian=self.stage in ('lower','lift','place','retreat'),
                    global_fallback=self.stage in ('place','retreat') and self.place_attempt>0,
                    shapes=shapes,held=held)

    def launch(self):
        self.detail = 'MoveIt 2 正在规划'
        self.status = 'planning'
        self.edited = False
        self.planned_revision = self.version
        request = self.request()
        self.future = asyncio.create_task(asyncio.to_thread(self.planner.solve,request))

    def report_failure(self, error):
        if self.stage in ('approach','lower') and self.candidate+1 < len(self.grasp_options):
            self.candidate += 1
            self.stage,self.status = 'approach','request'
            self.record('next_grasp_candidate')
            return
        if self.stage == 'carry' and self.transport_level < .26:
            self.transport_level = .28
            self.status = 'request'
            return
        if self.stage in ('place','retreat') and self.place_attempt < 2:
            self.place_attempt += 1
            self.status = 'request'
            return
        self.status,self.detail = 'blocked',str(error)
        self.record('no_valid_path')

    def tick(self):
        p = self.physics
        state = p.data
        if p.relative is not None:
            offset, attitude = p.relative_pose()
            retained = (min(p.finger_forces()) >= .5 and
                        np.linalg.norm(offset-p.relative[0]) < .02 and
                        (p.relative[1].inv()*attitude).magnitude() < .30)
            self.lost_grip_time = 0. if retained else self.lost_grip_time+TICK
            if self.lost_grip_time >= .14:
                self.version += 1
                state.ctrl[:6] = state.qpos[:6]
                self.route = None
                self.preview = []
                p.release()
                p.set_jaws(opened=True)
                self.status,self.detail = 'failed','杯子已滑出稳定夹持范围，机械臂已停止搬运'
                self.record('grasp_lost')
        if self.future is not None and self.future.done():
            pending,self.future = self.future,None
            try:
                route = pending.result()
            except (RuntimeError,ValueError,OSError) as error:
                if self.status == 'planning':
                    if self.planned_revision == self.version:
                        self.report_failure(error)
                    else:
                        self.status = 'request'
            else:
                if self.status == 'planning':
                    if self.planned_revision != self.version:
                        self.status = 'request'
                    else:
                        self.route,self.elapsed = route,0.
                        self.status,self.detail = 'moving','正在执行 MoveIt 轨迹'
                        self.edited = True
                        for t in np.linspace(0,route.times[-1],48):
                            p.scratch.qpos[:] = state.qpos
                            p.scratch.qpos[:6] = route.sample(t)[0]
                            mujoco.mj_kinematics(p.model,p.scratch)
                            self.preview.append(p.scratch.site_xpos[p.grip].tolist())
                        self.record('plan_accepted')
        if self.status == 'braking':
            self.wait += TICK
            if self.wait >= .16 and float(np.max(np.abs(state.qvel[:6]))) < .015:
                if self.scene_retries >= 3:
                    self.status,self.detail = 'blocked','仿真碰撞复查未通过，请调整障碍物'
                    self.record('no_valid_path')
                else:
                    self.status = 'request'
            elif self.wait >= 2:
                self.status,self.detail = 'blocked','仿真机械臂未能稳定停下，请重置场景'
                self.record('tracking_failed')
            return
        if self.status == 'moving':
            route = self.route
            end = route.times[-1]
            if self.edited:
                self.edited = False
                count = max(2,math.ceil((end-min(self.elapsed,end))/.04))
                if any(p.predicted_collision(route.sample(t)[0],moving_obstacle_only=True)
                       for t in np.linspace(min(self.elapsed,end),end,count)):
                    self.stop('障碍挡住剩余路线，已刹停，等待重规划')
                    return
            self.elapsed += TICK
            state.ctrl[:6] = route.sample(self.elapsed)[0]
            if self.elapsed >= end + .24 and np.max(np.abs(state.qpos[:6]-state.ctrl[:6])) < .008:
                self.next_stage()
            elif self.elapsed >= end + 2:
                self.status,self.detail = 'failed','模拟关节未能跟踪到规划终点'
                self.record('tracking_failed')
            return
        if self.status != 'request' or self.future is not None or self.editing:
            return
        if p.relative is None and self.stage in ('approach','lower'):
            position = p.pos('cup')
            if np.linalg.norm(position-self.target_origin) > .004:
                self.grasp_options = candidates(position)
                self.target_origin = position
                self.candidate,self.stage = 0,'approach'
        if self.stage == 'grip':
            p.set_jaws(opened=False)
            self.wait += TICK
            self.detail = '夹爪闭合，等待两侧接触杯子'
            self.grip_time = self.grip_time+TICK if min(p.finger_forces()) >= 1. else 0.
            if self.grip_time >= .10 and p.grasp():
                self.record('bilateral_grasp')
                self.next_stage()
            elif self.wait >= .85:
                p.set_jaws(opened=True)
                self.status,self.detail = 'failed','没有检测到夹爪两侧与杯子接触'
                self.record('grasp_failed')
        elif self.stage == 'release':
            if self.wait == 0:
                p.release()
                p.set_jaws(opened=True)
                self.record('cup_released')
            self.wait += TICK
            if self.wait >= .28 and p.jaw_angle() >= 1.03:
                self.next_stage()
            elif self.wait >= 1.:
                self.status,self.detail = 'failed','仿真夹爪未能张开到撤离位'
                self.record('tracking_failed')
        elif self.stage == 'settle':
            self.wait += TICK
            if self.wait >= .40:
                upright = state.xmat[p.cup].reshape(3,3)[:,2]
                quiet = np.linalg.norm(state.qvel[p.cup_velocity:p.cup_velocity+6]) < .05
                if quiet and inside_tray(p.pos('cup'),upright,p.pos('tray')):
                    self.status,self.detail = 'done','杯子已放入托盘并停稳'
                    self.record('placed_and_settled')
                elif self.wait >= 2:
                    self.status,self.detail = 'failed','杯子未在托盘内部停稳'
                    self.record('placement_failed')
        else:
            self.launch()
