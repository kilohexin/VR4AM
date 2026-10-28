"""Nominal FK for the admitted official simulator; controller IK stays native."""
import math
from app.robots.lebai_codec import pose_from_lebai,pose_to_lebai,joint_vector
from app.schemas.messages import Pose

PROBES=((0.,-.7853981633974483,1.5707963267948966,-.7853981633974483,1.5707963267948966,0.),
    (.2,-.6,1.2,-.7,1.3,.1),(-.3,-.8,1.4,-.4,1.5,-.2))


class _VerifiedLocalFK:
    def __init__(self,native,model):self.native=native;self.model=model
    def __getattr__(self,name):return getattr(self.native,name)
    async def kinematics_forward(self,q):
        p,r=self.model.forward(joint_vector(q,'local_fk'))
        return pose_to_lebai(Pose(p=p,q=r))


async def admit_local_fk(native,model,*,check=lambda:None):
    # These are mathematical queries only. A disagreement never changes the
    # controller model/tool or falls back to a different endpoint.
    for q in PROBES:
        check()
        reported=pose_from_lebai(await native.kinematics_forward(list(q)))
        check()
        p,r=model.forward(q)
        dot=min(1.,abs(sum(a*b for a,b in zip(reported.q,r))))
        if math.dist(reported.p,p)>1e-6 or 2*math.acos(dot)>1e-5:
            raise ValueError('docker_local_fk_model_mismatch')
    return _VerifiedLocalFK(native,model)
