"""Bounded tool-Z reference samples; not Cartesian path or hardware proof."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from app.robots.lebai_adapter import _MotionSnapshotFault

MAX_SPIN_RAD=math.radians(.1)

def tool_spin_angle(origin,target):
    vector=(Rotation.from_quat(origin.q).inv()*Rotation.from_quat(target.q)).as_rotvec()
    if (math.dist(origin.p,target.p)>1e-9 or np.linalg.norm(vector[:2])>1e-7
            or not 1e-9<abs(vector[2])<=MAX_SPIN_RAD+1e-12):
        raise _MotionSnapshotFault('segment_tool_spin_only')
    return float(vector[2])

def check_spin_sample(origin,pose,angle,*,complete=False):
    vector=(Rotation.from_quat(origin.q).inv()*Rotation.from_quat(pose.q)).as_rotvec()
    progress=float(vector[2])*math.copysign(1.,angle)
    if (math.dist(origin.p,pose.p)>1e-6 or progress < -1e-9
            or progress>abs(angle)+1e-6
            or np.linalg.norm(vector[:2])>max(1e-7,max(0.,progress)*math.tan(math.radians(5)))
            or (complete and abs(progress-abs(angle))>max(1e-7,abs(angle)*.05))):
        raise _MotionSnapshotFault('segment_tool_spin_fk_mismatch')
    return progress
