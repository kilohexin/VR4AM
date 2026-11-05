"""Offline sampled IK survey of the requested filming dimensions.

No time law, PVAT validation, collision checking, grasp inference or motion
permission. Inability of this local IK branch to solve is not a proof that
all robot configurations are unreachable. No automatic lowering is assumed.
"""
import math
import numpy as np


def survey(model,start,direction,*,lift_m=.3,travel_m=.5):
    if (direction not in ('x+','x-','y+','y-') or
            any(type(x) not in (float,int) or not math.isfinite(x) or not 0<x<=2. for x in (lift_m,travel_m))):
        raise ValueError('film_route_configuration')
    q=np.array(start,dtype=float)
    if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('film_route_origin')
    if np.any(q<model.lower) or np.any(q>model.upper):raise ValueError('film_route_joint_bounds')
    base=model.pose(tuple(q));samples=0;peak=0.
    result=dict(requested_lift_m=lift_m,requested_travel_m=travel_m,direction=direction,
        geometrically_reachable=False,path_verified=False,motion_admitted=False,
        phases=['lift','transfer'],grid_m=.01)
    for phase,length in (('lift',lift_m),('transfer',travel_m)):
        count=math.ceil(length/.01)
        for i in range(1,count+1):
            distance=length*i/count;p=list(base.p)
            p[2]+=distance if phase=='lift' else lift_m
            if phase=='transfer':p[0 if direction[0]=='x' else 1]+=(1 if direction[1]=='+' else -1)*distance
            goal=base.model_copy(update={'p':tuple(p)})
            try:
                solved=model.inverse(goal,q.copy())
                jump=float(np.max(np.abs(solved-q)))
                if jump>=.15:raise ValueError('film_route_branch_jump')
            except ValueError as error:
                # Fixed classification; never expose arbitrary vendor text.
                result.update(failure_phase=phase,failure_distance_m=distance,
                    reason='local_ik_or_branch_rejected',samples=samples,last_q=tuple(map(float,q)),
                    max_joint_step_rad=peak)
                return result
            q=solved;samples+=1;peak=max(peak,jump)
    result.update(geometrically_reachable=True,samples=samples,last_q=tuple(map(float,q)),max_joint_step_rad=peak)
    return result
