"""Optional strict-FP theoretical MDH kernel. No SDK, transport or executor.

Only immutable model geometry is compiled/cached; robot state and admission
results are never cached. Warm-up is explicit, before any future actuation.
The independent Python MDH implementation remains the comparison oracle.
"""
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation

from app.control.cartesian_pvat_reference import MdhReferenceModel,_vector


def _geometry(q,qd,tool_p,tool_r,rate):
    d=(.21583,0.,0.,.12063,.09833,.08343)
    a=(0.,0.,-.28,-.26,0.,0.)
    alpha=(0.,1.5708,0.,0.,1.5708,-1.5708)
    r=np.eye(3);p=np.zeros(3);origins=np.empty((6,3));axes=np.empty((6,3))
    for joint in range(6):
        ct=math.cos(q[joint]);st=math.sin(q[joint])
        ca=math.cos(alpha[joint]);sa=math.sin(alpha[joint])
        local_r=np.array(((ct,-st,0.),(st*ca,ct*ca,-sa),(st*sa,ct*sa,ca)))
        local_p=np.array((a[joint],-d[joint]*sa,d[joint]*ca))
        following=np.empty((3,3));displacement=np.zeros(3)
        for row in range(3):
            for axis in range(3):
                displacement[row]+=r[row,axis]*local_p[axis]
            for column in range(3):
                value=0.
                for axis in range(3):value+=r[row,axis]*local_r[axis,column]
                following[row,column]=value
        p+=displacement;r=following
        origins[joint,:]=p;axes[joint,:]=r[:,2]
    transform=np.eye(4)
    for row in range(3):
        value=p[row]
        for axis in range(3):value+=r[row,axis]*tool_p[axis]
        transform[row,3]=value
        for column in range(3):
            value=0.
            for axis in range(3):value+=r[row,axis]*tool_r[axis,column]
            transform[row,column]=value
    j=np.empty((6,6));jd=np.zeros((6,6))
    for i in range(6):
        x=transform[0,3]-origins[i,0];y=transform[1,3]-origins[i,1];z=transform[2,3]-origins[i,2]
        j[0,i]=axes[i,1]*z-axes[i,2]*y
        j[1,i]=axes[i,2]*x-axes[i,0]*z
        j[2,i]=axes[i,0]*y-axes[i,1]*x
        for axis in range(3):j[axis+3,i]=axes[i,axis]
    if rate:
        pdot=np.zeros(3)
        for axis in range(3):
            for i in range(6):pdot[axis]+=j[axis,i]*qd[i]
        for i in range(6):
            zdot=np.zeros(3);odot=np.zeros(3)
            for k in range(i):
                for axis in range(3):
                    b=(axis+1)%3;c=(axis+2)%3
                    zdot[axis]+=(axes[k,b]*axes[i,c]-axes[k,c]*axes[i,b])*qd[k]
                    odot[axis]+=(axes[k,b]*(origins[i,c]-origins[k,c])
                        -axes[k,c]*(origins[i,b]-origins[k,b]))*qd[k]
            delta=transform[:3,3]-origins[i,:];relative=pdot-odot
            for axis in range(3):
                b=(axis+1)%3;c=(axis+2)%3
                jd[axis,i]=(zdot[b]*delta[c]-zdot[c]*delta[b]
                    +axes[i,b]*relative[c]-axes[i,c]*relative[b])
                jd[axis+3,i]=zdot[axis]
    return transform,j,jd


class CompiledReferenceModel(MdhReferenceModel):
    """Explicit offline selection; missing acceleration never falls back silently."""
    def __init__(self,tool,lower,upper):
        super().__init__(tool,lower,upper)
        try:from numba import njit
        except ImportError as error:
            raise ValueError('cartesian_reference_acceleration_unavailable') from error
        self._compiled_tool=tool
        self._tool_p=np.array(tool.translation_m)
        self._tool_r=Rotation.from_quat(tool.rotation_xyzw).as_matrix()
        self._tool_p.setflags(write=False);self._tool_r.setflags(write=False)
        self._kernel=njit(cache=True,fastmath=False,parallel=False)(_geometry)
        before=time.perf_counter()
        # Compile the actual float64 signatures and compare fresh independent
        # geometry before this model can be passed to a reference supervisor.
        for q in (np.zeros(6),np.array([0.,-.8,1.4,-.6,1.57,0.]),np.array([.4,-1.,1.8,-.2,.9,-.7])):
            for qd in (None,np.array([.1,-.2,.05,.02,-.07,.03])):
                expected=super().geometry(q,qd);actual=self.geometry(q,qd)
                if any(not np.allclose(x,y,rtol=2e-12,atol=2e-12) for x,y in zip(expected,actual)):
                    raise ValueError('cartesian_reference_acceleration_mismatch')
        self.warmup_wall_s=time.perf_counter()-before

    def geometry(self,q,qd=None):
        if self.tool!=self._compiled_tool:raise ValueError('cartesian_reference_acceleration_model_changed')
        q=_vector(q,6);velocity=np.zeros(6) if qd is None else _vector(qd,6)
        t,j,jd=self._kernel(q,velocity,self._tool_p,self._tool_r,qd is not None)
        if any(not np.isfinite(x).all() for x in (t,j,jd)):
            raise ValueError('cartesian_reference_nonfinite_or_shape')
        return (t,j) if qd is None else (t,j,jd)
