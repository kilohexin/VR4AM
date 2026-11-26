"""MuJoCo dynamics, contact grasp and prediction for the workbench."""

import math
from xml.etree import ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from scene import (BLOCK_SIZE, CUP_HEIGHT, CUP_RADIUS, FINGERS, GRIPPER, GRIPPER_JOINT,
    GRIPPER_MASS, GRIPPER_OPEN, GRIP_CENTER_Z, OBJECTS, START, TABLE_CENTER,
    TABLE_SIZE, TABLE_TOP, arm_model, tray_boxes, xyz)

FINGER_FRICTION = .65
GRIP_TORQUE = .95


def box(body, name, center, size, **attributes):
    ET.SubElement(body, 'geom', name=name, type='box', pos=xyz(center),
                  size=xyz(np.asarray(size)/2), **attributes)


def add_cup(world, location):
    body = ET.SubElement(world, 'body', name='cup', pos=xyz(location))
    ET.SubElement(body, 'freejoint', name='cup_free')
    ET.SubElement(body, 'geom', name='cup_floor', type='cylinder', size=f'{CUP_RADIUS} .003',
                  pos='0 0 .003', mass='.025', friction='1.2 .02 .002', condim='4')
    thick = .004
    radius = CUP_RADIUS-thick/2
    for index in range(12):
        yaw = index*math.tau/12
        box(body, f'cup_wall_{index}', [radius*math.cos(yaw),radius*math.sin(yaw),CUP_HEIGHT/2],
            [thick, .0152, CUP_HEIGHT], euler=xyz([0,0,yaw]), mass='.007',
            friction='1.2 .02 .002', condim='4')


def articulate_gripper(xml, archive):
    tool = xml.find(".//body[@name='tool0']")
    ET.SubElement(tool, 'site', name='grip', pos=xyz([0, 0, GRIP_CENTER_Z]))
    equality = ET.SubElement(xml, 'equality')
    for joint in ET.parse(archive/'lm3-expanded.urdf').findall('joint'):
        name = joint.get('name')
        child = joint.find('child').get('link')
        if child not in GRIPPER or joint.get('type') != 'revolute':
            continue
        body = tool.find(f".//body[@name='{child}']")
        limits = joint.find('limit')
        ET.SubElement(body, 'joint', name=name, axis=joint.find('axis').get('xyz'),
                      range=xyz([limits.get('lower'), limits.get('upper')]),
                      damping='.015', armature='.00015')
        mimic = joint.find('mimic')
        if mimic is not None:
            ET.SubElement(equality, 'joint', joint1=name, joint2=mimic.get('joint'),
                          polycoef=xyz([mimic.get('offset'), mimic.get('multiplier'), 0, 0, 0]),
                          solref='.002 1', solimp='.995 .9999 .001')
    for name in FINGERS:
        geom = tool.find(f".//body[@name='{name}']/geom[@group='3']")
        geom.attrib.update(name=name, friction=f'{FINGER_FRICTION} .004 .0001', condim='4',
                           priority='1', solref='.005 1', solimp='.96 .99 .002')
    ET.SubElement(xml.find('actuator'), 'position', name='gripper_motor', joint=GRIPPER_JOINT,
                  kp='18', kv='.25', forcerange=xyz([-GRIP_TORQUE, GRIP_TORQUE]),
                  ctrlrange=xyz([0, GRIPPER_OPEN]))
    # Start at the URDF open position and keep the reference angles at URDF zero.
    keyframe = xml.find('keyframe')
    if keyframe is not None:
        xml.remove(keyframe)


def create_model(archive):
    xml = arm_model(archive)
    articulate_gripper(xml, archive)
    world = xml.find('worldbody')
    box(world, 'table', TABLE_CENTER, TABLE_SIZE, friction='1.0 .01 .001')
    add_cup(world, OBJECTS['cup'])
    for name in ('tray','block'):
        body = ET.SubElement(world, 'body', name=name, mocap='true', pos=xyz(OBJECTS[name]))
        if name == 'tray':
            for index,(center,size) in enumerate(tray_boxes([0,0,0])):
                box(body, f'tray_{index}',center,size,friction='1.0 .01 .001')
        else:
            box(body,'block_body',[0,0,BLOCK_SIZE[2]/2],BLOCK_SIZE)
    model = mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'))
    bodies = [model.body(name).id for name in GRIPPER]
    mass_scale = GRIPPER_MASS / model.body_mass[bodies].sum()
    model.body_mass[bodies] *= mass_scale
    model.body_inertia[bodies] *= mass_scale
    mujoco.mj_setConst(model, mujoco.MjData(model))
    return model


class Physics:
    def __init__(self, archive):
        self.model = create_model(archive)
        self.data = mujoco.MjData(self.model)
        self.scratch = mujoco.MjData(self.model)
        self.tool = self.model.body('tool0').id
        self.cup = self.model.body('cup').id
        self.cup_address = self.model.joint('cup_free').qposadr[0]
        self.cup_velocity = self.model.joint('cup_free').dofadr[0]
        self.jaw_joint = self.model.joint(GRIPPER_JOINT).id
        self.jaw_address = self.model.jnt_qposadr[self.jaw_joint]
        self.jaw_motor = self.model.actuator('gripper_motor').id
        self.mocap = {name: int(self.model.body(name).mocapid[0]) for name in ('tray','block')}
        self.grip = self.model.site('grip').id
        self.geometry = {}
        for idx in range(self.model.ngeom):
            body = self.model.geom_bodyid[idx]
            name = self.model.body(body).name
            if name in ('table','cup','tray','block') or body == 0:
                group = ('table' if body == 0 else name)
            elif self.model.geom_group[idx] == 1:
                group = 'visual'
            else:
                group = 'arm'
            self.geometry[idx] = group
        self.fingers = [self.model.geom(name).id for name in FINGERS]
        self.finger_vertices = {}
        for geom in self.fingers:
            mesh = self.model.geom_dataid[geom]
            start = self.model.mesh_vertadr[mesh]
            count = self.model.mesh_vertnum[mesh]
            self.finger_vertices[geom] = self.model.mesh_vert[start:start+count]
        self.relative = None
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model,self.data)
        self.data.qpos[:6] = self.data.ctrl[:6] = START
        self.set_jaws(opened=True)
        self.data.qpos[self.jaw_address] = GRIPPER_OPEN
        for follower, leader, coefficients in zip(self.model.eq_obj1id, self.model.eq_obj2id,
                                                   self.model.eq_data, strict=True):
            if leader == self.jaw_joint:
                self.data.qpos[self.model.jnt_qposadr[follower]] = coefficients[0]+coefficients[1]*GRIPPER_OPEN
        self.relative = None
        mujoco.mj_forward(self.model,self.data)

    def step(self, count):
        mujoco.mj_step(self.model,self.data,nstep=count)
        mujoco.mj_forward(self.model,self.data)

    def pos(self, name):
        return self.data.xpos[self.model.body(name).id].copy()

    def pose(self, name):
        body=self.model.body(name).id
        w,x,y,z=self.data.xquat[body]
        return [self.data.xpos[body].tolist(),[x,y,z,w]]

    def jaw_gap(self):
        axis = self.data.xmat[self.tool].reshape(3,3)[:,0]
        edges = []
        for geom in self.fingers:
            local_axis = self.data.geom_xmat[geom].reshape(3,3).T @ axis
            edges.append(self.finger_vertices[geom] @ local_axis + self.data.geom_xpos[geom] @ axis)
        return max(0., float(edges[1].min() - edges[0].max()))

    def set_jaws(self, *, opened):
        self.data.ctrl[self.jaw_motor] = GRIPPER_OPEN if opened else 0.

    def jaw_angle(self):
        return float(np.clip(self.data.qpos[self.jaw_address], 0., GRIPPER_OPEN))

    def drag(self, name, x, y):
        state = self.data
        if name == 'cup':
            index = self.cup_address
            old = state.qpos[index:index+7].copy()
            state.qpos[index:index+7] = [x,y,TABLE_TOP+.002,1,0,0,0]
        else:
            index = self.mocap[name]
            old = state.mocap_pos[index].copy()
            state.mocap_pos[index] = [x,y,OBJECTS[name][2]]
        mujoco.mj_forward(self.model,state)
        group_pairs = [set(map(self.geometry.__getitem__, contact.geom)) for contact in state.contact
                       if contact.dist < -.001]
        conflict = any(name in pair and pair & ({'arm','cup','tray','block'}-{name}) for pair in group_pairs)
        if conflict:
            if name == 'cup':
                state.qpos[index:index+7] = old
            else:
                state.mocap_pos[index] = old
            mujoco.mj_forward(self.model,state)
            return False
        if name == 'cup':
            state.qvel[self.cup_velocity:self.cup_velocity+6] = 0
        return True

    def finger_forces(self):
        forces = dict.fromkeys(self.fingers, 0.)
        wrench = np.zeros(6)
        for index, contact in enumerate(self.data.contact):
            for a, b in (contact.geom, contact.geom[::-1]):
                if a in forces and self.geometry[b] == 'cup':
                    mujoco.mj_contactForce(self.model, self.data, index, wrench)
                    forces[a] += float(wrench[0])
        return tuple(forces.values())

    def relative_pose(self):
        tool_rotation = Rotation.from_matrix(self.data.xmat[self.tool].reshape(3,3))
        cup_rotation = Rotation.from_matrix(self.data.xmat[self.cup].reshape(3,3))
        offset = tool_rotation.inv().apply(self.data.xpos[self.cup]-self.data.xpos[self.tool])
        return offset, tool_rotation.inv()*cup_rotation

    def grasp(self):
        if min(self.finger_forces()) < 1.:
            return False
        self.relative = self.relative_pose()
        return True

    def release(self):
        self.relative = None

    def predicted_collision(self, joints, moving_obstacle_only=False):
        state = self.scratch
        state.qpos[:] = self.data.qpos
        state.qpos[:6] = joints
        state.mocap_pos[:] = self.data.mocap_pos
        state.mocap_quat[:] = self.data.mocap_quat
        mujoco.mj_kinematics(self.model,state)
        if self.relative is not None:
            offset, attitude = self.relative_pose()
            rotation = Rotation.from_matrix(state.xmat[self.tool].reshape(3,3))
            state.qpos[self.cup_address:self.cup_address+7] = [
                *(state.xpos[self.tool]+rotation.apply(offset)), *(rotation*attitude).as_quat(scalar_first=True)]
        mujoco.mj_fwdPosition(self.model,state)
        for contact in state.contact:
            if contact.dist >= -.0008:
                continue
            pair = {self.geometry[g] for g in contact.geom}
            if moving_obstacle_only:
                if 'block' in pair and ('arm' in pair or (self.relative is not None and 'cup' in pair)):
                    return True
            elif 'arm' in pair and pair.intersection(('block','tray','table')):
                return True
            elif self.relative is not None and 'cup' in pair and pair.intersection(('block','table','tray')):
                return True
        return False
