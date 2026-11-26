"""Shared geometry for MuJoCo, MoveIt and the browser."""

import math
import os
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
TABLE_TOP = -.04
TABLE_CENTER = (-.25, 0., TABLE_TOP-.035)
TABLE_SIZE = (1.20, 1.16, .07)
OBJECTS = {
    'cup': (-.50, -.24, TABLE_TOP+.002),
    'tray': (-.49, .21, TABLE_TOP+.001),
    'block': (-.69, .29, TABLE_TOP),
}
START = [0., -.8, 1.4, -.6, 1.57, 0.]
JOINTS = [f'joint_{i}' for i in range(1, 7)]
ARM = ['base_link', *(f'link_{i}' for i in range(1, 7))]
CUP_RADIUS = .028
CUP_HEIGHT = .086
FLOOR = .012
TRAY_SIZE = (.218, .202)
WALL = .012
BLOCK_SIZE = (.104, .13, .29)
GRIPPER_JOINT = 'gripper_r_joint1'
GRIPPER_OPEN = math.pi/3
FINGERS = ('gripper_l_link_finger', 'gripper_r_link_finger')
GRIPPER = ('gripper_base_link', 'gripper_l_link1', 'gripper_l_link2', FINGERS[0],
           'gripper_r_link1', 'gripper_r_link2', FINGERS[1])
GRIPPER_MASS = .46
GRIP_CENTER_Z = .151
GRASP_FLANGE_HEIGHT = .242  # Finger tips straddle the rim with clearance for the linkages.
APPROACH = .079
LIFT = .155


def model_directory():
    folder = Path(os.environ.get('LM3_MODEL_DIR', HERE.parent/'model')).resolve()
    if not (folder/'lm3-with-gripper-compensated.xml').is_file():
        raise FileNotFoundError(f'LM3 model archive is missing: {folder}')
    return folder


def xyz(values):
    return ' '.join(map(str, values))


def tray_boxes(origin):
    width, depth = TRAY_SIZE
    center = np.asarray(origin)
    height = .033
    return [(list(center+[0, 0, FLOOR/2]), [width, depth, FLOOR]),
            *[(list(center+[side*(width-WALL)/2, 0, FLOOR+height/2]), [WALL, depth, height])
              for side in (-1, 1)],
            *[(list(center+[0, side*(depth-WALL)/2, FLOOR+height/2]), [width-2*WALL, WALL, height])
              for side in (-1, 1)]]


def inside_tray(cup, up, tray):
    dx, dy, dz = np.subtract(cup, tray)
    return (abs(dx)+CUP_RADIUS < TRAY_SIZE[0]/2-WALL and
            abs(dy)+CUP_RADIUS < TRAY_SIZE[1]/2-WALL and
            FLOOR-.004 <= dz <= FLOOR+.015 and up[2] >= .94)


def candidates(origin):
    from scipy.spatial.transform import Rotation

    grasp = np.asarray(origin)+[0, 0, GRASP_FLANGE_HEIGHT]
    pregrasp = grasp + [0, 0, APPROACH]
    return [(grasp.copy(), pregrasp.copy(),
             (Rotation.from_euler('z', yaw)*Rotation.from_euler('x', math.pi)).as_quat())
            for yaw in (0., math.pi/2, -math.pi/2, math.pi)]


def arm_model(archive):
    xml = ET.parse(archive/'lm3-with-gripper-compensated.xml').getroot()
    xml.find('compiler').set('meshdir', (archive/'assets').as_posix())
    return xml


def visual_gripper(archive):
    xml = arm_model(archive)
    assets = {mesh.get('name'): mesh.get('file') for mesh in xml.findall('asset/mesh')}
    result = {}
    for name in GRIPPER:
        geom = xml.find(f".//body[@name='{name}']/geom[@group='1']")
        w, x, y, z = map(float, geom.get('quat', '1 0 0 0').split())
        result[name] = dict(mesh=assets[geom.get('mesh')],
                            position=list(map(float, geom.get('pos', '0 0 0').split())),
                            rotation=[x, y, z, w])
    return result


def write_collision_hull(source, destination):
    from scipy.spatial import ConvexHull

    # The archived binary STL has one vertex triplet per triangle.
    facets = np.dtype([('normal','<f4',(3,)), ('vertices','<f4',(3,3)), ('attribute','<u2')])
    points = np.unique(np.fromfile(source, dtype=facets, offset=84)['vertices'].reshape(-1,3), axis=0)
    hull = ConvexHull(points)
    indices = {vertex: index for index, vertex in enumerate(hull.vertices, start=1)}
    with destination.open('w', encoding='ascii') as obj:
        for point in points[hull.vertices]:
            obj.write('v '+xyz(point)+'\n')
        for triangle, plane in zip(hull.simplices, hull.equations, strict=True):
            a, b, c = points[triangle]
            if np.cross(b-a,c-a) @ plane[:3] < 0:
                triangle = triangle[::-1]
            obj.write('f '+xyz(indices[vertex] for vertex in triangle)+'\n')


def planner_description(archive, gripper_collision):
    robot = ET.parse(archive/'lm3-expanded.urdf').getroot()
    converted = set()
    for link in list(robot.findall('link')):
        for geometry in link.findall('visual'):
            link.remove(geometry)
        for mesh in link.findall('.//mesh'):
            name = Path(mesh.get('filename'))
            stem = name.parent.name+'_'+name.stem
            if link.get('name') in GRIPPER:
                if stem not in converted:
                    write_collision_hull(archive/'assets'/f'{stem}.stl', gripper_collision/f'{stem}.obj')
                    converted.add(stem)
                mesh.set('filename', f'file:///gripper_collision/{stem}.obj')
            else:
                mesh.set('filename', f'file:///model/assets/{stem}{name.suffix}')
    semantic = ET.Element('robot', name=robot.get('name'))
    arm = ET.SubElement(semantic, 'group', name='manipulator')
    ET.SubElement(arm, 'chain', base_link='base_link', tip_link='tool0')
    semantic.extend(ET.parse(archive/'lm3.srdf').findall('disable_collisions'))
    limits = {}
    for joint in robot.findall('joint'):
        if joint.get('name') in JOINTS:
            raw = joint.find('limit').attrib
            limits[joint.get('name')] = dict(has_position_limits=True, min_position=float(raw['lower']),
                max_position=float(raw['upper']), has_velocity_limits=True, max_velocity=.75,
                has_acceleration_limits=True, max_acceleration=1.2)
    return dict(robot=ET.tostring(robot,encoding='unicode'), srdf=ET.tostring(semantic,encoding='unicode'),
                limits=limits)
