"""Offline Cartesian PVAT candidate; intentionally has no executor or SDK access.

Quintic Hermite interpolation is an explicit analysis assumption, NOT a
verified description of the native controller. Passing this model cannot
unquarantine the live pose-follow entry or authorize hardware motion.
"""
from dataclasses import dataclass
from math import comb, factorial

import numpy as np
from scipy.optimize import LinearConstraint,least_squares,minimize
from scipy.spatial.transform import Rotation

from app.commissioning.native_jog_mdh import MdhTool, mdh_tcp_jacobian, mdh_tcp_jacobian_rate
from app.schemas.messages import Pose


def _vector(value, count):
    a=np.array(value,dtype=float,copy=True)
    if a.shape!=(count,) or not np.isfinite(a).all():
        raise ValueError('cartesian_reference_nonfinite_or_shape')
    return a


@dataclass(frozen=True)
class JointReference:
    q: tuple[float,...]
    v: tuple[float,...]=(0.,)*6
    a: tuple[float,...]=(0.,)*6

    def __post_init__(self):
        for name in ('q','v','a'):
            object.__setattr__(self,name,tuple(float(x) for x in _vector(getattr(self,name),6)))


def _quintic(p0,v0,a0,p1,v1,a1,duration):
    # Coefficients use normalized time u=t/T, with physical-time derivatives.
    if not np.isfinite(duration) or duration<=0:raise ValueError('cartesian_reference_duration')
    c=np.zeros((6,len(p0)))
    c[0]=p0;c[1]=v0*duration;c[2]=a0*duration**2/2
    b=np.array([p1-c[:3].sum(axis=0),v1*duration-c[1]-2*c[2],a1*duration**2-2*c[2]])
    c[3:]=np.linalg.solve(np.array([[1.,1.,1.],[3.,4.,5.],[6.,12.,20.]]),b)
    if not np.isfinite(c).all():raise ValueError('cartesian_reference_nonfinite_or_shape')
    return c


def _evaluate(c,u,order=0,duration=1.):
    return sum((factorial(i)/factorial(i-order))*c[i]*u**(i-order)
        for i in range(order,len(c)))/duration**order


def _range(c,lo=0.,hi=1.):
    """Conservative polynomial range using the Bernstein convex hull.

    This avoids assuming a dense set of samples covers all extrema. Numerical
    roundoff gets an outward allowance; this is not a formal FP proof.
    """
    n=len(c)-1
    power=np.array([sum(comb(i,k)*c[i]*lo**(i-k)*(hi-lo)**k for i in range(k,n+1)) for k in range(n+1)])
    bernstein=np.array([sum(power[k]*comb(j,k)/comb(n,k) for k in range(j+1)) for j in range(n+1)])
    margin=256*np.finfo(float).eps*(1+np.abs(c).sum(axis=0)+np.abs(power).sum(axis=0))
    return bernstein.min(axis=0)-margin,bernstein.max(axis=0)+margin


def _derivative(c,order,duration):
    return np.array([factorial(i)/factorial(i-order)*c[i]/duration**order for i in range(order,len(c))])


class MdhReferenceModel:
    """Theoretical model for offline analysis, never factory calibration."""
    def __init__(self,tool,lower,upper):
        if type(tool) is not MdhTool:raise ValueError('cartesian_reference_tool')
        self.tool=tool;self.lower=_vector(lower,6);self.upper=_vector(upper,6)
        if np.any(self.lower>=self.upper):raise ValueError('cartesian_reference_joint_bounds')
        # A conservative radius for each revolute joint. Include all remaining
        # link translations, even offsets lying on an axis, plus the tool.
        lengths=np.array([.21583,.0,.28,.26+.12063,.09833,.08343])
        self.position_lipschitz=np.cumsum(lengths[::-1])[::-1]+np.linalg.norm(tool.translation_m)

    def geometry(self,q,qd=None):
        args=(tuple(float(x) for x in q),self.tool)
        if qd is None:return mdh_tcp_jacobian(*args)
        return mdh_tcp_jacobian_rate(args[0],tuple(float(x) for x in qd),args[1])

    def pose(self,q):
        t,_=self.geometry(q)
        return Pose(p=tuple(float(x) for x in t[:3,3]),q=tuple(float(x) for x in Rotation.from_matrix(t[:3,:3]).as_quat()))

    def inverse(self,pose,seed):
        goal=Rotation.from_quat(pose.q)
        def residual(q):
            t,_=self.geometry(q)
            return np.r_[5*(t[:3,3]-pose.p),(goal.inv()*Rotation.from_matrix(t[:3,:3])).as_rotvec()]
        solved=least_squares(residual,seed,bounds=(self.lower,self.upper),max_nfev=40,
            ftol=1e-11,xtol=1e-11,gtol=1e-11)
        p=self.pose(solved.x)
        if (np.linalg.norm(np.array(p.p)-pose.p)>2e-7
                or (Rotation.from_quat(p.q).inv()*goal).magnitude()>2e-6):
            raise ValueError('cartesian_reference_ik_unreachable')
        return solved.x


def _angular(r,rd,rdd):
    # Spatial omega/alpha for R(t)=Exp(r(t))*R0, including the noncommuting
    # rotation-vector Jacobian terms. Euler component derivatives are invalid.
    theta=np.linalg.norm(r);dot=float(r@rd)
    if theta<1e-3:
        a=.5-theta**2/24+theta**4/720
        b=1/6-theta**2/120+theta**4/5040
        ad=(-1/12+theta**2/180)*dot;bd=(-1/60+theta**2/1260)*dot
    else:
        a=(1-np.cos(theta))/theta**2;b=(theta-np.sin(theta))/theta**3
        ad=(theta*np.sin(theta)-2*(1-np.cos(theta)))/theta**4*dot
        bd=(theta*(1-np.cos(theta))-3*(theta-np.sin(theta)))/theta**5*dot
    cross=np.cross(r,rd);double=np.cross(r,cross)
    return (rd+a*cross+b*double,
        rdd+a*np.cross(r,rdd)+b*np.cross(r,np.cross(r,rdd))+ad*cross+bd*double+b*np.cross(rd,cross))


class CartesianCurve:
    def __init__(self,start,twist,acceleration,target,duration):
        twist=_vector(twist,6);acceleration=_vector(acceleration,6)
        self.rotation=Rotation.from_quat(start.q);self.duration=duration
        end=np.r_[target.p,(Rotation.from_quat(target.q)*self.rotation.inv()).as_rotvec()]
        self.coefficients=_quintic(np.r_[start.p,(0.,)*3],twist,acceleration,end,np.zeros(6),np.zeros(6),duration)

    @classmethod
    def cubic(cls,start,twist,acceleration,jerk,duration):
        if not np.isfinite(duration) or duration<=0:raise ValueError('cartesian_reference_duration')
        twist=_vector(twist,6);acceleration=_vector(acceleration,6);jerk=_vector(jerk,6)
        curve=cls.__new__(cls);curve.rotation=Rotation.from_quat(start.q);curve.duration=duration
        curve.coefficients=np.zeros((6,6));curve.coefficients[0,:3]=start.p
        curve.coefficients[1]=twist*duration;curve.coefficients[2]=acceleration*duration**2/2
        jerk[3:]-=.5*np.cross(twist[3:],acceleration[3:])
        curve.coefficients[3]=jerk*duration**3/6
        return curve

    def state(self,t):
        if not 0<=t<=self.duration:raise ValueError('cartesian_reference_time')
        u=t/self.duration;c=self.coefficients
        x=_evaluate(c,u);v=_evaluate(c,u,1,self.duration);a=_evaluate(c,u,2,self.duration)
        w,wd=_angular(x[3:],v[3:],a[3:])
        q=(Rotation.from_rotvec(x[3:])*self.rotation).as_quat()
        return Pose(p=tuple(x[:3]),q=tuple(q)),np.r_[v[:3],w],np.r_[a[:3],wd]


@dataclass(frozen=True)
class PvatSegment:
    start: JointReference
    end: JointReference
    duration: float

    def coefficients(self):
        return _quintic(*(np.array(x) for x in (self.start.q,self.start.v,self.start.a,
            self.end.q,self.end.v,self.end.a)),self.duration)

    def state(self,t):
        if not 0<=t<=self.duration:raise ValueError('cartesian_reference_time')
        c=self.coefficients();u=t/self.duration
        return JointReference(*(tuple(_evaluate(c,u,d,self.duration)) for d in range(3)))


def validate_segment(segment,model,curve,offset,*,position_tolerance=.001,angle_tolerance=.01,
        speed=.2,acceleration=.2,jerk=1.,tcp_speed=.15,tcp_acceleration=.15,
        angular_speed=.3,angular_acceleration=.3,max_depth=12):
    """Validate the entire assumed Hermite segment, not endpoints alone.

    FK displacement bounds plus recursive subdivision enclose unsampled
    intervals. Exhausting depth rejects; no 'best effort' acceptance.
    """
    if (not all(np.isfinite(x) and x>0 for x in (position_tolerance,angle_tolerance,speed,acceleration,jerk,
            tcp_speed,tcp_acceleration,angular_speed,angular_acceleration))
            or type(max_depth) is not int or not 0<=max_depth<=16):
        raise ValueError('cartesian_reference_validation_limits')
    if (not np.isfinite(offset) or offset<0 or offset+segment.duration>curve.duration+1e-9):
        raise ValueError('cartesian_reference_interval')
    c=segment.coefficients();lo,hi=_range(c)
    if np.any(lo<model.lower) or np.any(hi>model.upper):raise ValueError('cartesian_reference_joint_bounds')
    peaks=[]
    for d,limit in ((1,speed),(2,acceleration),(3,jerk)):
        low,high=_range(_derivative(c,d,segment.duration))
        peaks.append(float(np.maximum(-low,high).max()))
        if np.any(np.maximum(-low,high)>limit+1e-10):raise ValueError('cartesian_reference_rate_'+str(d))
    # Preserve enough velocity reserve to cancel terminal acceleration with
    # the allowed jerk. A valid next point can otherwise enter a state from
    # which the subsequent point cannot satisfy the speed limit.
    terminal_v=np.array(segment.end.v);terminal_a=np.array(segment.end.a)
    if (np.any(terminal_v+np.maximum(terminal_a,0)**2/(2*jerk)>speed+1e-10)
            or np.any(-terminal_v+np.maximum(-terminal_a,0)**2/(2*jerk)>speed+1e-10)):
        raise ValueError('cartesian_reference_rate_reserve')
    derivatives=[_derivative(c,d,segment.duration) for d in (1,2)]
    radii=(model.position_lipschitz,np.ones(6))
    coupling=[np.minimum.outer(r,r) for r in radii]
    third=[np.minimum(h[:,:,None],r[None,None,:]) for h,r in zip(coupling,radii)]
    velocity_limits=np.array([tcp_speed,angular_speed]);acceleration_limits=np.array([tcp_acceleration,angular_acceleration])
    max_velocity=np.zeros(2);max_acceleration=np.zeros(2)
    stack=[(0.,1.,0)];visited=0;max_position=0.;max_angle=0.
    while stack:
        a,b,depth=stack.pop();mid=(a+b)/2;visited+=1
        q=_evaluate(c,mid);actual=model.pose(q)
        expected,_,_=curve.state(offset+mid*segment.duration)
        error=np.linalg.norm(np.array(actual.p)-expected.p)
        angle=(Rotation.from_quat(actual.q)*Rotation.from_quat(expected.q).inv()).magnitude()
        if error>position_tolerance or angle>angle_tolerance:raise ValueError('cartesian_reference_corridor')
        qlo,qhi=_range(c,a,b);dq=np.maximum(np.abs(qlo-q),np.abs(qhi-q))
        velocity=_evaluate(c,mid,1,segment.duration);acc=_evaluate(c,mid,2,segment.duration)
        vlo,vhi=_range(derivatives[0],a,b);alo,ahi=_range(derivatives[1],a,b)
        vmax=np.maximum(-vlo,vhi);amax=np.maximum(-alo,ahi)
        dv=np.maximum(np.abs(vlo-velocity),np.abs(vhi-velocity))
        da=np.maximum(np.abs(alo-acc),np.abs(ahi-acc))
        _,j,jd=model.geometry(q,velocity)
        task_v=j@velocity;task_a=j@acc+jd@velocity
        sv=np.array([np.linalg.norm(task_v[:3]),np.linalg.norm(task_v[3:])])
        sa=np.array([np.linalg.norm(task_a[:3]),np.linalg.norm(task_a[3:])])
        if np.any(sv>velocity_limits+1e-10) or np.any(sa>acceleration_limits+1e-10):
            raise ValueError('cartesian_reference_task_rate')
        # Product-of-rotations derivatives have operator norm <=1. A mixed
        # derivative can affect only link offsets downstream of its latest
        # joint, giving min(L_i,L_j,...) bounds. These retain cancellation at
        # the midpoint and bound its variation over the complete interval.
        bv=[];ba=[]
        for k,radius in enumerate(radii):
            h=coupling[k]
            bv.append(sv[k]+radius@dv+np.einsum('ij,i,j->',h,vmax,dq))
            ba.append(sa[k]+radius@da+np.einsum('ij,i,j->',h,amax,dq)
                +np.einsum('ij,i,j->',h,dv,vmax)+np.einsum('ij,i,j->',h,np.abs(velocity),dv)
                +np.einsum('ijk,i,j,k->',third[k],vmax,vmax,dq))
        bv=np.array(bv);ba=np.array(ba)
        ca=(offset+a*segment.duration)/curve.duration;cb=(offset+b*segment.duration)/curve.duration
        plo,phi=_range(curve.coefficients,ca,cb);center=_evaluate(curve.coefficients,(ca+cb)/2)
        dp=np.maximum(np.abs(plo-center),np.abs(phi-center))
        position_bound=error+model.position_lipschitz@dq+np.linalg.norm(dp[:3])
        angle_bound=angle+sum(dq)+np.linalg.norm(dp[3:])
        if (position_bound<=position_tolerance and angle_bound<=angle_tolerance
                and np.all(bv<=velocity_limits) and np.all(ba<=acceleration_limits)):
            max_position=max(max_position,float(position_bound));max_angle=max(max_angle,float(angle_bound))
            max_velocity=np.maximum(max_velocity,bv);max_acceleration=np.maximum(max_acceleration,ba)
        else:
            if depth>=max_depth:raise ValueError('cartesian_reference_unresolved_interval')
            stack.extend(((a,mid,depth+1),(mid,b,depth+1)))
    return dict(intervals=visited,position_bound_m=max_position,angle_bound_rad=max_angle,
        joint_speed_bound_radps=peaks[0],joint_acceleration_bound_radps2=peaks[1],joint_jerk_bound_radps3=peaks[2],
        tcp_speed_bound_mps=float(max_velocity[0]),tcp_acceleration_bound_mps2=float(max_acceleration[0]),
        angular_speed_bound_radps=float(max_velocity[1]),angular_acceleration_bound_radps2=float(max_acceleration[1]),
        interpolation='quintic_hermite_assumed',native_interpolation_verified=False)


@dataclass(frozen=True)
class CartesianPlan:
    curve: CartesianCurve
    segments: tuple[PvatSegment,...]
    checks: tuple[dict,...]

    def state(self,t):
        if not 0<=t<=self.curve.duration:raise ValueError('cartesian_reference_time')
        dt=self.segments[0].duration;i=min(int(t/dt),len(self.segments)-1)
        return self.segments[i].state(min(dt,max(0.,t-i*dt)))


def _curve_from_reference(model,start,target,duration):
    q=np.array(start.q)
    if np.any(q<model.lower) or np.any(q>model.upper):raise ValueError('cartesian_reference_joint_bounds')
    _,j,jd=model.geometry(q,np.array(start.v))
    return CartesianCurve(model.pose(q),j@start.v,j@start.a+jd@start.v,target,duration)


def _solve_segment(model,start,curve,offset,period,validation):
    pose,twist,acc=curve.state(min(curve.duration,offset+period))
    solved=model.inverse(pose,np.array(start.q))
    if np.max(np.abs(solved-start.q))>=.15:raise ValueError('cartesian_reference_ik_branch')
    _,j=model.geometry(solved)
    if np.linalg.svd(j,compute_uv=False)[-1]<1e-4:raise ValueError('cartesian_reference_singular')
    v=np.linalg.solve(j,twist);_,_,jd=model.geometry(solved,v)
    a=np.linalg.solve(j,acc-jd@v)
    segment=PvatSegment(start,JointReference(tuple(solved),tuple(v),tuple(a)),period)
    return segment,validate_segment(segment,model,curve,offset,**validation)


class OfflineCartesianFollower:
    """One validated reference step at a time; never an actuator session.

    Compute one Cartesian cubic with a constrained common task jerk, solve
    its IK/PVA and validate the complete assumed joint interpolation. No IK
    cursor advances beyond this reference. Rejection commits no partial state.
    """
    def __init__(self,model,start,*,period=.08,frequency=4.,**validation):
        if not np.isfinite(period) or not .01<=period<=.08 or not np.isfinite(frequency) or not 0<frequency<=4:
            raise ValueError('cartesian_reference_duration')
        self.model=model;self.state=start;self.period=period;self.frequency=frequency;self.validation=dict(validation)

    def advance(self,target):
        q=np.array(self.state.q);v=np.array(self.state.v);a=np.array(self.state.a);dt=self.period
        p=self.model.pose(q);_,j,jd=self.model.geometry(q,v)
        if np.linalg.svd(j,compute_uv=False)[-1]<1e-4:raise ValueError('cartesian_reference_singular')
        h=1e-4
        _,_,plus=self.model.geometry(q+v*h+a*h*h/2,v+a*h)
        _,_,minus=self.model.geometry(q-v*h+a*h*h/2,v-a*h)
        bias=2*jd@a+((plus-minus)/(2*h))@v
        twist=j@v;acc=j@a+jd@v;f=self.frequency
        error=np.r_[np.array(target.p)-p.p,(Rotation.from_quat(target.q)*Rotation.from_quat(p.q).inv()).as_rotvec()]
        desired=f**3*error-3*f*f*twist-3*f*acc
        # Reserve 10% within the unchanged analysis limits for nonlinear IK
        # and interpolation effects. The final validator still checks the
        # original limits and can reject the optimized candidate.
        speed=.9*self.validation.get('speed',.2);acceleration=.9*self.validation.get('acceleration',.2)
        jerk=.9*self.validation.get('jerk',1.)
        braking_jerk=.75*self.validation.get('jerk',1.)
        low=np.maximum(-jerk,(-acceleration-a)/dt);high=np.minimum(jerk,(acceleration-a)/dt)
        low=np.maximum(low,2*(-speed-v-a*dt)/dt**2);high=np.minimum(high,2*(speed-v-a*dt)/dt**2)
        if np.any(low>high):raise ValueError('cartesian_reference_optimizer_infeasible')
        def reserve(x):
            vn=v+a*dt+x*dt*dt/2;an=a+x*dt
            return np.r_[speed-vn-np.maximum(an,0)**2/(2*braking_jerk),speed+vn-np.maximum(-an,0)**2/(2*braking_jerk)]
        task_speed=.8*np.array([self.validation.get('tcp_speed',.15),self.validation.get('angular_speed',.3)])
        task_acceleration=.8*np.array([self.validation.get('tcp_acceleration',.15),self.validation.get('angular_acceleration',.3)])
        def task_rates(x):
            gamma=j@x+bias;vn=twist+acc*dt+gamma*dt*dt/2;an=acc+gamma*dt
            # Integrate the same rotation-vector cubic used by the candidate;
            # spatial angular jerk is not an independent scalar polynomial.
            local_jerk=gamma[3:]-.5*np.cross(twist[3:],acc[3:])
            r=twist[3:]*dt+acc[3:]*dt*dt/2+local_jerk*dt**3/6
            rd=twist[3:]+acc[3:]*dt+local_jerk*dt*dt/2
            vn[3:],an[3:]=_angular(r,rd,acc[3:]+local_jerk*dt)
            return np.r_[1-np.array([np.linalg.norm(vn[:3]),np.linalg.norm(vn[3:])])/task_speed,
                1-np.array([np.linalg.norm(an[:3]),np.linalg.norm(an[3:])])/task_acceleration]
        constraints=[dict(type='ineq',fun=reserve),dict(type='ineq',fun=task_rates)]
        locked=(np.linalg.norm(error[:3])<1e-8 and np.linalg.norm(twist[:3])<1e-8 and np.linalg.norm(acc[:3])<1e-8)
        if locked:constraints.append(LinearConstraint(j[:3],-bias[:3],-bias[:3]))
        weights=np.array([10.,10.,10.,1.,1.,1.]);matrix=weights[:,None]*j;rhs=weights*(desired-bias)
        solved=minimize(lambda x:float(np.linalg.norm(matrix@x-rhs)**2+1e-8*x@x),
            np.minimum(high,np.maximum(low,np.zeros(6))),jac=lambda x:2*matrix.T@(matrix@x-rhs)+2e-8*x,
            bounds=list(zip(low,high)),constraints=constraints,method='SLSQP',options=dict(maxiter=80,ftol=1e-12))
        # SLSQP status 8 may contain a feasible point without proving objective
        # convergence. Feasibility is checked independently, then the full
        # nonlinear IK/PVAT segment must still pass. Keep the status in evidence.
        if (solved.status not in (0,8) or not np.isfinite(solved.x).all()
                or np.any(solved.x<low-1e-10) or np.any(solved.x>high+1e-10)
                or min(reserve(solved.x))<-1e-8 or min(task_rates(solved.x))<-1e-8):
            rejected=ValueError('cartesian_reference_optimizer_infeasible')
            rejected.optimizer_status=int(solved.status)
            rejected.minimum_reserve=float(min(reserve(solved.x)))
            rejected.minimum_task_margin=float(min(task_rates(solved.x)))
            raise rejected
        task_jerk=j@solved.x+bias
        if locked:
            if np.linalg.norm(task_jerk[:3])>1e-8:raise ValueError('cartesian_reference_optimizer_infeasible')
            task_jerk[:3]=0.
        curve=CartesianCurve.cubic(p,twist,acc,task_jerk,dt)
        segment,check=_solve_segment(self.model,self.state,curve,0.,dt,self.validation)
        self.state=segment.end
        return segment,dict(check,position_locked=locked,optimizer_iterations=int(solved.nit),
            optimizer_status=int(solved.status),optimizer_converged=bool(solved.success))


def plan_cartesian_reference(model,start,target,*,duration=4.,period=.08,**validation):
    """Build an offline C2 reference, including a retarget from nonzero P/V/A.

    Caller chooses duration; infeasible rates reject instead of adjusting
    hardware settings or emitting a fallback motion. No planner is installed
    in the product executor by importing or calling this function.
    """
    if not np.isfinite(duration) or not np.isfinite(period) or period<.01 or duration<period:
        raise ValueError('cartesian_reference_duration')
    count=round(duration/period)
    if count>2000 or abs(count*period-duration)>1e-9:raise ValueError('cartesian_reference_grid')
    curve=_curve_from_reference(model,start,target,duration)
    previous=start;segments=[];checks=[]
    for i in range(1,count+1):
        segment,check=_solve_segment(model,previous,curve,(i-1)*period,period,validation)
        checks.append(check);segments.append(segment);previous=segment.end
    return CartesianPlan(curve,tuple(segments),tuple(checks))
