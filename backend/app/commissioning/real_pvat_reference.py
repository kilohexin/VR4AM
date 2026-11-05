"""Pure finite reference candidates for a later scoped physical empty trial.

No SDK, native transport, motion admission, preparation or gripper. Inputs
are caller-bound model geometry and a nominal exactly quiet reference, not
permission to replace nonzero measured velocity or acceleration with zero.
"""
from dataclasses import dataclass,field
import math

from scipy.spatial.transform import Rotation
from app.control.cartesian_pvat_reference import (JointReference,PvatSegment,CartesianPlan,
    plan_cartesian_reference,validate_segment)
from app.control.cartesian_tool_binding import tool_digest


@dataclass(frozen=True)
class EmptyPvatTrial:
    axis: str
    plan: CartesianPlan
    tool_sha256: str
    motion_admitted: bool=field(default=False,init=False)


def build_empty_trial(model,start,*,axis='z'):
    if axis not in ('z','pitch','spin'):raise ValueError('real_pvat_trial_axis')
    if type(start) is not JointReference or any(v!=0 for v in start.v+start.a):
        raise ValueError('real_pvat_trial_requires_quiet_reference')
    digest=tool_digest(model.tool);base=model.pose(start.q)
    if axis=='z':target=base.model_copy(update={'p':(base.p[0],base.p[1],base.p[2]+.002)})
    else:
        turn=Rotation.from_euler('x' if axis=='pitch' else 'z',math.radians(.5))
        target=base.model_copy(update={'q':tuple(float(v) for v in (Rotation.from_quat(base.q)*turn).as_quat())})
    limits=dict(speed=.05,acceleration=.2)
    plan=plan_cartesian_reference(model,start,target,duration=1.6,period=.08,**limits)
    if len(plan.segments)!=20:raise ValueError('real_pvat_trial_grid')
    last=plan.segments[-1]
    # The declared target is rest-to-rest; polynomial evaluation can leave
    # roundoff residual derivatives. Construct its exact symbolic terminal,
    # then REVALIDATE the entire changed last segment with identical limits.
    # This never edits measured feedback or a moving reference start.
    terminal=JointReference(last.end.q)
    final=PvatSegment(last.start,terminal,last.duration)
    check=validate_segment(final,model,plan.curve,19*.08,**limits)
    checked=CartesianPlan(plan.curve,plan.segments[:-1]+(final,),plan.checks[:-1]+(check,))
    return EmptyPvatTrial(axis,checked,digest)
