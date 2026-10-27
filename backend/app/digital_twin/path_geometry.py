"""Batched geometry for the synthetic demo, not hardware path certification."""
import math
import numpy as np
from scipy.spatial.transform import Rotation

from app.sim.kinematics import _translation,_rotation


def chain_points_batch(samples,model):
    """Same FK order as scalar chain_points; retain every sample, batch rotations."""
    values=np.asarray(samples,dtype=float)
    if values.ndim!=2 or values.shape[1]!=6 or not len(values) or not np.isfinite(values).all():
        raise ValueError('invalid_ready_path_samples')
    count=len(values);transform=np.repeat(np.eye(4)[None,:,:],count,axis=0)
    points={'base':np.zeros((count,3))}
    for index,axis,offset in zip(range(6),model.joint_axes,model.joint_offsets_m,strict=True):
        transform=transform@_translation(offset)
        points[f'joint{index+1}']=transform[:,:3,3].copy()
        axis_local=np.eye(3)['xyz'.index(axis)]
        rotations=np.repeat(np.eye(4)[None,:,:],count,axis=0)
        rotations[:,:3,:3]=Rotation.from_rotvec(values[:,index,None]*axis_local).as_matrix()
        transform=transform@rotations
    transform=transform@_translation(model.tool_root_offset_m)
    transform=transform@_rotation(Rotation.from_quat(model.tool_rotation_xyzw).as_matrix())
    transform=transform@_translation(model.tcp_offset_m)
    if (not np.isfinite(transform).all()
            or not all(math.isclose(float(d),1.,abs_tol=1e-8) for d in np.linalg.det(transform[:,:3,:3]))):
        raise ValueError('invalid LM3 transform')
    points['tcp']=transform[:,:3,3].copy()
    return points


def segment_distances(a0,a1,b0,b1):
    """Same clamped closest-point algorithm as sim.self_collision.segment_distance."""
    points=[np.asarray(p,dtype=float) for p in (a0,a1,b0,b1)]
    if (any(p.ndim!=2 or p.shape[1]!=3 for p in points)
            or any(p.shape!=points[0].shape for p in points)
            or any(not np.all(np.isfinite(p)) for p in points)):
        raise ValueError('invalid_self_collision_geometry')
    a0,a1,b0,b1=points
    first,second,offset=a1-a0,b1-b0,a0-b0
    dot=lambda x,y:np.einsum('ij,ij->i',x,y)
    aa,bb,cross=dot(first,first),dot(second,second),dot(first,second)
    c,f=dot(first,offset),dot(second,offset)
    epsilon=np.finfo(float).eps
    nonzero_a,nonzero_b=aa>epsilon,bb>epsilon
    safe_a,safe_b=np.where(nonzero_a,aa,1.),np.where(nonzero_b,bb,1.)
    denominator=aa*bb-cross*cross
    regular=nonzero_a & nonzero_b & (denominator>epsilon)
    s=np.clip(np.divide(cross*f-c*bb,denominator,
        out=np.zeros_like(aa),where=regular),0.,1.)
    s=np.where(nonzero_a & ~nonzero_b,np.clip(-c/safe_a,0.,1.),s)
    t=np.where(nonzero_b,(cross*s+f)/safe_b,0.)
    s=np.where((t<0.) & nonzero_a,np.clip(-c/safe_a,0.,1.),s)
    s=np.where((t>1.) & nonzero_a,np.clip((cross-c)/safe_a,0.,1.),s)
    t=np.clip(t,0.,1.)
    distance=np.linalg.norm(offset+s[:,None]*first-t[:,None]*second,axis=1)
    if not np.all(np.isfinite(distance)):
        raise ValueError('invalid_self_collision_geometry')
    return distance


def validate_ready_samples(start,goal,settings,model):
    """Retain all original 65 samples and pairs, sharing each sample's FK."""
    samples=np.array(start)+(np.array(goal)-start)*np.linspace(0.,1.,65)[:,None]
    if (not np.all(np.isfinite(samples))
            or np.any(samples<np.array(settings.soft_joint_min_rad)+settings.joint_limit_margin_rad)
            or np.any(samples>np.array(settings.soft_joint_max_rad)-settings.joint_limit_margin_rad)
            or np.any(abs(samples-np.array(model.home_q))>model.joint_window_rad)):
        raise ValueError('demo_ready_path_invalid')
    points=chain_points_batch(samples,model)
    positions=points['tcp']
    if (np.any(positions<np.array(settings.startup_tcp_min_m))
            or np.any(positions>np.array(settings.startup_tcp_max_m))):
        raise ValueError('demo_ready_path_invalid')
    segments={segment.name:segment for segment in model.collision_segments}
    for first_name,second_name in model.collision_check_pairs:
        first,second=segments[first_name],segments[second_name]
        threshold=first.radius_m+second.radius_m+model.collision_safety_margin_m
        if not np.isfinite(threshold):
            raise ValueError('invalid_self_collision_geometry')
        if np.any(segment_distances(points[first.start],points[first.end],
                points[second.start],points[second.end])<threshold):
            raise ValueError('demo_ready_path_invalid')
