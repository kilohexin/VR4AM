"""Rebuild the archived MJCF and meshes from the pinned Lebai ROS source."""

import argparse
from pathlib import Path
import re
import shutil
import subprocess
from tempfile import TemporaryDirectory
from xml.etree import ElementTree as ET

import mujoco
import numpy as np
import trimesh
import xacro
from scipy.spatial.transform import Rotation


UPSTREAM_REVISION = 'f1f13b6849f34f61dce0483e09f531816d292b63'
DESCRIPTION = 'lebai_lm3_support/urdf/lm3_with_gripper.xacro'


def expand_description(source):
    with TemporaryDirectory(prefix='lm3-xacro-') as directory:
        stage = Path(directory)
        for package in ('lebai_lm3_support', 'lebai_resources'):
            for path in (source / package / 'urdf').glob('*.xacro'):
                target = stage / package / 'urdf' / path.name
                target.parent.mkdir(parents=True, exist_ok=True)
                contents = re.sub(
                    r'\$\(find (\w+)\)',
                    lambda match: (stage / match[1]).as_posix(),
                    path.read_text(encoding='utf-8'),
                )
                target.write_text(contents, encoding='utf-8')
        return ET.fromstring(xacro.process_file(str(stage / DESCRIPTION)).toxml())


def transform(node):
    if node is None:
        return {}
    xyz = node.get('xyz', '0 0 0')
    xyzw = Rotation.from_euler('xyz', [float(x) for x in node.get('rpy', '0 0 0').split()]).as_quat()
    return {'pos': xyz, 'quat': ' '.join(str(v) for v in xyzw[[3, 0, 1, 2]])}


def convert(source, output):
    urdf = expand_description(source)
    assets = output / 'assets'
    assets.mkdir(parents=True, exist_ok=False)
    ET.indent(urdf)
    ET.ElementTree(urdf).write(output / 'lm3-expanded.urdf', encoding='utf-8', xml_declaration=True)
    srdf_path = source / 'lebai_lm3_moveit_config/config/lebai_lm3.srdf'
    shutil.copyfile(srdf_path, output / 'lm3.srdf')
    shutil.copyfile(source / 'LICENSE', output / 'LICENSE')
    root = ET.Element('mujoco', model='Lebai LM3 with gripper')
    ET.SubElement(root, 'compiler', angle='radian', meshdir='assets', inertiafromgeom='auto')
    ET.SubElement(root, 'option', timestep='0.001', integrator='implicitfast', gravity='0 0 -9.81')
    asset = ET.SubElement(root, 'asset')
    world = ET.SubElement(root, 'worldbody')
    actuator = ET.SubElement(root, 'actuator')
    contact = ET.SubElement(root, 'contact')
    links = {link.get('name'): link for link in urdf.findall('link')}
    joints = urdf.findall('joint')
    names = {f'joint_{i}' for i in range(1, 7)}
    meshes = {}

    def mesh(geometry):
        uri = geometry.get('filename')
        if uri not in meshes:
            path = source / uri.removeprefix('package://')
            kind = path.parent.name
            name = f'{kind}_{path.stem}'
            dest = assets / f'{name}.stl'
            if path.suffix.lower() == '.dae':
                scene = trimesh.load_scene(path)
                scene.to_geometry().export(dest)
            elif path.stat().st_size > 10_000_084:
                shutil.copyfile(path, dest)
                dest = dest.with_suffix('.obj')
                trimesh.load_mesh(path).export(dest)
            else:
                shutil.copyfile(path, dest)
            ET.SubElement(asset, 'mesh', name=name, file=dest.name, scale=geometry.get('scale', '1 1 1'))
            meshes[uri] = name
        return meshes[uri]

    def add_link(name, parent, joint=None):
        body = ET.SubElement(parent, 'body', name=name, **transform(None if joint is None else joint.find('origin')))
        link = links[name]
        if joint is not None and joint.get('name') in names:
            joint_name = joint.get('name')
            limit = joint.find('limit')
            effort = float(limit.get('effort'))
            ET.SubElement(body, 'joint', name=joint_name, axis=joint.find('axis').get('xyz'),
                          range=f"{limit.get('lower')} {limit.get('upper')}", damping='0.3', armature='0.05')
            ET.SubElement(actuator, 'position', name=joint_name, joint=joint_name,
                          kp='300', kv='12', forcerange=f'{-effort} {effort}')
        inert = link.find('inertial')
        if inert is not None:
            tensor = inert.find('inertia')
            inertia = np.array([[float(tensor.get('i'+a+b) or tensor.get('i'+b+a)) for b in 'xyz'] for a in 'xyz'])
            origin = inert.find('origin')
            rotation = Rotation.from_euler('xyz', list(map(float, origin.get('rpy', '0 0 0').split()))).as_matrix()
            inertia = rotation @ inertia @ rotation.T
            values = [inertia[i,j] for i,j in ((0,0),(1,1),(2,2),(0,1),(0,2),(1,2))]
            ET.SubElement(body, 'inertial', pos=origin.get('xyz'), mass=inert.find('mass').get('value'),
                          fullinertia=' '.join(map(str, values)))
        for kind in ('visual', 'collision'):
            for shape in link.findall(kind):
                geometry = shape.find('geometry')
                if geometry is None or len(geometry) != 1 or geometry[0].tag != 'mesh':
                    continue
                attributes = transform(shape.find('origin'))
                if kind == 'visual':
                    attributes.update(group='1', contype='0', conaffinity='0', mass='0')
                    color = shape.find('material/color')
                    if color is not None:
                        attributes['rgba'] = color.get('rgba')
                else:
                    attributes.update(group='3', rgba='0.48 0.59 0.70 0.25')
                    if name.startswith('gripper_'):
                        attributes['density'] = '1100'
                    else:
                        attributes['mass'] = '0'
                ET.SubElement(body, 'geom', type='mesh', mesh=mesh(geometry[0]), **attributes)
        if name in ('tool0', 'gripper_tip'):
            ET.SubElement(body, 'site', name=name, size='.003')
        for dependent in joints:
            if dependent.find('parent').get('link') == name:
                add_link(dependent.find('child').get('link'), body, dependent)

    add_link('base_link', world)
    srdf = ET.parse(srdf_path)
    seen = set()
    for pair in srdf.findall('disable_collisions'):
        a, b = sorted((pair.get('link1'), pair.get('link2')))
        if a in links and b in links and (a, b) not in seen:
            ET.SubElement(contact, 'exclude', body1=a, body2=b)
            seen.add((a, b))
    ET.indent(root)
    path = output / 'lm3-with-gripper.xml'
    ET.ElementTree(root).write(path, encoding='unicode')
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('upstream', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    source = args.upstream.resolve(strict=True)
    revision = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != UPSTREAM_REVISION:
        raise ValueError(f'Check out Lebai ROS SDK revision {UPSTREAM_REVISION}')
    path = convert(source, args.output)
    tree = ET.parse(path)
    for body in tree.iter('body'):
        body.set('gravcomp', '1')
    compensated = args.output / 'lm3-with-gripper-compensated.xml'
    tree.write(compensated, encoding='unicode')
    for file in (path, compensated):
        model = mujoco.MjModel.from_xml_path(str(file))
        if model.njnt != 6 or model.nu != 6:
            raise ValueError(f'Invalid six-axis model: {file}')
    print(f'Built both LM3 MJCF variants in {args.output}')


if __name__ == '__main__':
    main()
