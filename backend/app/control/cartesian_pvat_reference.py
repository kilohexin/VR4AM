"""Offline Cartesian PVAT candidate; intentionally has no executor or SDK access.

Quintic Hermite interpolation is an explicit analysis assumption, NOT a
verified description of the native controller. Passing this model cannot
unquarantine the live pose-follow entry or authorize hardware motion.
"""
from dataclasses import dataclass
from functools import lru_cache
from math import comb, factorial
from types import SimpleNamespace

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


@lru_cache(maxsize=4096)
def _range_matrices(n,lo,hi):
    # Pure polynomial basis changes only. No poses, time, SDK results or
    # admission decisions are cached; every new segment is checked afresh.
    power=np.zeros((n+1,n+1));basis=np.zeros_like(power)
    for k in range(n+1):
        for i in range(k,n+1):power[k,i]=comb(i,k)*lo**(i-k)*(hi-lo)**k
        for j in range(k,n+1):basis[j,k]=comb(j,k)/comb(n,k)
    bernstein=basis@power
    power.setflags(write=False);bernstein.setflags(write=False)
    return power,bernstein


def _range(c,lo=0.,hi=1.):
    """Conservative polynomial range using the Bernstein convex hull.

    This avoids assuming a dense set of samples covers all extrema. Numerical
    roundoff gets an outward allowance; this is not a formal FP proof.
    """
    p,b=_range_matrices(len(c)-1,float(lo),float(hi))
    power=p@c;bernstein=b@c
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

    def inverse_linearization(self,pose,q):
        goal=Rotation.from_quat(pose.q)
        t,j=self.geometry(q)
        r=(goal.inv()*Rotation.from_matrix(t[:3,:3])).as_rotvec();theta=np.linalg.norm(r)
        skew=np.array([[0.,-r[2],r[1]],[r[2],0.,-r[0]],[-r[1],r[0],0.]])
        factor=(1/12+theta**2/720+theta**4/30240 if theta<1e-3
            else 1/theta**2-1/(2*theta*np.tan(theta/2)))
        log_inverse=np.eye(3)-skew/2+factor*(skew@skew)
        return (np.r_[5*(t[:3,3]-pose.p),r],np.vstack((5*j[:3],log_inverse@goal.inv().as_matrix()@j[3:])))

    def inverse(self,pose,seed):
        goal=Rotation.from_quat(pose.q);cached=[None,None]
        def linearization(q):
            if cached[0] is None or not np.array_equal(cached[0],q):
                cached[:]=[np.array(q,copy=True),self.inverse_linearization(pose,q)]
            return cached[1]
        solved=least_squares(lambda q:linearization(q)[0],seed,jac=lambda q:linearization(q)[1],
            bounds=(self.lower,self.upper),max_nfev=40,
            ftol=1e-11,xtol=1e-11,gtol=1e-11)
        p=self.pose(solved.x)
        if (np.linalg.norm(np.array(p.p)-pose.p)>2e-7
                or (Rotation.from_quat(p.q).inv()*goal).magnitude()>2e-6):
            raise ValueError('cartesian_reference_ik_unreachable')
        return solved.x


def _cross(a,b):
    return np.array((a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]))


def _angular(r,rd,rdd):
    # Spatial omega/alpha for R(t)=Exp(r(t))*R0, including the noncommuting
    # rotation-vector Jacobian terms. Euler component derivatives are invalid.
    # Analytic extension also permits complex-step differentiation of this
    # pure formula; actual reference inputs and outputs remain real.
    theta=np.sqrt(r@r);dot=r@rd
    if abs(theta)<1e-3:
        a=.5-theta**2/24+theta**4/720
        b=1/6-theta**2/120+theta**4/5040
        ad=(-1/12+theta**2/180)*dot;bd=(-1/60+theta**2/1260)*dot
    else:
        a=(1-np.cos(theta))/theta**2;b=(theta-np.sin(theta))/theta**3
        ad=(theta*np.sin(theta)-2*(1-np.cos(theta)))/theta**4*dot
        bd=(theta*(1-np.cos(theta))-3*(theta-np.sin(theta)))/theta**5*dot
    cross=_cross(r,rd);double=_cross(r,cross)
    return (rd+a*cross+b*double,
        rdd+a*_cross(r,rdd)+b*_cross(r,_cross(r,rdd))+ad*cross+bd*double+b*_cross(rd,cross))


def _angular_endpoint(gamma,omega,alpha,dt):
    local_jerk=gamma-.5*_cross(omega,alpha)
    r=omega*dt+alpha*dt*dt/2+local_jerk*dt**3/6
    rd=omega+alpha*dt+local_jerk*dt*dt/2
    return _angular(r,rd,alpha+local_jerk*dt)


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
    curve_acceleration=_derivative(curve.coefficients,2,curve.duration)
    radii=(model.position_lipschitz,np.ones(6))
    coupling=[np.minimum.outer(r,r) for r in radii]
    third=[np.minimum(h[:,:,None],r[None,None,:]) for h,r in zip(coupling,radii)]
    velocity_limits=np.array([tcp_speed,angular_speed]);acceleration_limits=np.array([tcp_acceleration,angular_acceleration])
    max_velocity=np.zeros(2);max_acceleration=np.zeros(2)
    stack=[(0.,1.,0)];visited=0;max_position=0.;max_angle=0.
    while stack:
        a,b,depth=stack.pop();mid=(a+b)/2;visited+=1
        q=_evaluate(c,mid)
        velocity=_evaluate(c,mid,1,segment.duration);acc=_evaluate(c,mid,2,segment.duration)
        actual_matrix,j,jd=model.geometry(q,velocity)
        expected,_,_=curve.state(offset+mid*segment.duration)
        error=np.linalg.norm(actual_matrix[:3,3]-expected.p)
        angle=(Rotation.from_matrix(actual_matrix[:3,:3])*Rotation.from_quat(expected.q).inv()).magnitude()
        if error>position_tolerance or angle>angle_tolerance:raise ValueError('cartesian_reference_corridor')
        qlo,qhi=_range(c,a,b);dq=np.maximum(np.abs(qlo-q),np.abs(qhi-q))
        vlo,vhi=_range(derivatives[0],a,b);alo,ahi=_range(derivatives[1],a,b)
        vmax=np.maximum(-vlo,vhi);amax=np.maximum(-alo,ahi)
        dv=np.maximum(np.abs(vlo-velocity),np.abs(vhi-velocity))
        da=np.maximum(np.abs(alo-acc),np.abs(ahi-acc))
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
        # Enclose the ERROR curve, retaining cancellation between coordinated
        # FK and Cartesian translation. Taylor's integral remainder bounds
        # every time in this interval, not only the sampled midpoint. Both
        # this and the original displacement enclosure must remain valid.
        reference_v=_evaluate(curve.coefficients,(ca+cb)/2,1,curve.duration)[:3]
        ralo,rahi=_range(curve_acceleration,ca,cb)
        reference_acc_bound=np.linalg.norm(np.maximum(-ralo[:3],rahi[:3]))
        half_time=(b-a)*segment.duration/2
        error_taylor=error+np.linalg.norm(task_v[:3]-reference_v)*half_time
        error_taylor+=(ba[0]+reference_acc_bound)*half_time**2/2
        position_bound=min(position_bound,error_taylor)
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
    def __init__(self,model,start,*,period=.08,frequency=4.,lookahead_position=.02,lookahead_rotation=.08,
            demand_mode='coupled',**validation):
        if not np.isfinite(period) or not .01<=period<=.08 or not np.isfinite(frequency) or not 0<frequency<=4:
            raise ValueError('cartesian_reference_duration')
        if not all(np.isfinite(x) and x>0 for x in (lookahead_position,lookahead_rotation)):
            raise ValueError('cartesian_reference_lookahead')
        if demand_mode not in ('coupled','separate'):raise ValueError('cartesian_reference_demand_mode')
        self.model=model;self.state=start;self.period=period;self.frequency=frequency;self.validation=dict(validation)
        self.lookahead_position=lookahead_position;self.lookahead_rotation=lookahead_rotation
        self.demand_mode=demand_mode

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
        # Govern the demand relative to the LAST VALIDATED reference, not an
        # independent future IK cursor. Default to one common fraction; the
        # separate-demand candidate keeps translation independent of a remote
        # orientation goal. Both still use ONE coordinated IK/PVA validator.
        # The original hand endpoint and fixed workspace are intact.
        fraction=1/max(1.,np.linalg.norm(error[:3])/self.lookahead_position,
            np.linalg.norm(error[3:])/self.lookahead_rotation)
        factors=np.full(6,fraction)
        if self.demand_mode=='separate':
            factors[:3]=1/max(1.,np.linalg.norm(error[:3])/self.lookahead_position)
            factors[3:]=1/max(1.,np.linalg.norm(error[3:])/self.lookahead_rotation)
        desired=f**3*(factors*error)-3*f*f*twist-3*f*acc
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
        def reserve_jacobian(x):
            an=a+x*dt
            return np.vstack((np.diag(-dt*dt/2-np.maximum(an,0)*dt/braking_jerk),
                np.diag(dt*dt/2+np.maximum(-an,0)*dt/braking_jerk)))
        task_speed=.8*np.array([self.validation.get('tcp_speed',.15),self.validation.get('angular_speed',.3)])
        task_acceleration=.8*np.array([self.validation.get('tcp_acceleration',.15),self.validation.get('angular_acceleration',.3)])
        def task_rates(x):
            gamma=j@x+bias;vn=twist+acc*dt+gamma*dt*dt/2;an=acc+gamma*dt
            # Integrate the same rotation-vector cubic used by the candidate;
            # spatial angular jerk is not an independent scalar polynomial.
            vn[3:],an[3:]=_angular_endpoint(gamma[3:],twist[3:],acc[3:],dt)
            return np.r_[1-np.array([np.linalg.norm(vn[:3]),np.linalg.norm(vn[3:])])/task_speed,
                1-np.array([np.linalg.norm(an[:3]),np.linalg.norm(an[3:])])/task_acceleration]
        def task_jacobian(x):
            gamma=j@x+bias
            vn=twist+acc*dt+gamma*dt*dt/2;an=acc+gamma*dt
            vn[3:],an[3:]=_angular_endpoint(gamma[3:],twist[3:],acc[3:],dt)
            angular_v=np.empty((3,3));angular_a=np.empty((3,3));h=1e-24
            for axis in range(3):
                perturbation=np.array(gamma[3:],dtype=complex);perturbation[axis]+=h*1j
                w,alpha=_angular_endpoint(perturbation,twist[3:],acc[3:],dt)
                angular_v[:,axis]=w.imag/h;angular_a[:,axis]=alpha.imag/h
            derivatives=(dt*dt/2*j[:3],angular_v@j[3:],dt*j[:3],angular_a@j[3:])
            values=(vn[:3],vn[3:],an[:3],an[3:]);limits=(*task_speed,*task_acceleration)
            return np.array([-value@derivative/(np.linalg.norm(value)*limit) if np.linalg.norm(value)>1e-15
                else np.zeros(6) for value,derivative,limit in zip(values,derivatives,limits)])
        constraints=[dict(type='ineq',fun=reserve,jac=reserve_jacobian),
            dict(type='ineq',fun=task_rates,jac=task_jacobian)]
        stationary=(np.abs(twist[:3])<1e-8)&(np.abs(acc[:3])<1e-8)
        held_axes=np.flatnonzero(stationary&(np.abs(error[:3])<1e-8))
        directed_axes=np.flatnonzero(stationary&(np.abs(error[:3])>=1e-8))
        held=j[held_axes];held_rhs=-bias[held_axes]
        directions=np.sign(error[directed_axes]);directed=directions[:,None]*j[directed_axes]
        directed_bias=directions*bias[directed_axes]
        locked=len(held_axes)==3
        if len(held_axes):constraints.append(LinearConstraint(held,held_rhs,held_rhs))
        if len(directed_axes):constraints.append(LinearConstraint(directed,-directed_bias,np.inf))
        def linear_ok(x):
            return ((not len(held_axes) or np.max(np.abs(held@x-held_rhs))<=1e-8)
                and (not len(directed_axes) or np.min(directed@x+directed_bias)>=-1e-10))
        weights=np.array([10.,10.,10.,1.,1.,1.]);matrix=weights[:,None]*j;rhs=weights*(desired-bias)
        normal=matrix.T@matrix+1e-8*np.eye(6);right=matrix.T@rhs
        if len(held_axes):
            system=np.block([[normal,held.T],[held,np.zeros((len(held_axes),len(held_axes)))]])
            direct=np.linalg.solve(system,np.r_[right,held_rhs])[:6]
        else:direct=np.linalg.solve(normal,right)
        # A feasible equality-constrained quadratic optimum needs no iterative
        # optimization. It still passes every constraint and the same complete
        # nonlinear segment validation below.
        if (np.isfinite(direct).all() and np.all(direct>=low) and np.all(direct<=high)
                and min(reserve(direct))>=0 and min(task_rates(direct))>=0
                and linear_ok(direct)):
            solved=SimpleNamespace(x=direct,success=True,status=0,nit=0)
        else:
            solved=minimize(lambda x:float(np.linalg.norm(matrix@x-rhs)**2+1e-8*x@x),
                np.minimum(high,np.maximum(low,direct)),jac=lambda x:2*matrix.T@(matrix@x-rhs)+2e-8*x,
                bounds=list(zip(low,high)),constraints=constraints,method='SLSQP',options=dict(maxiter=80,ftol=1e-12))
        # SLSQP status 8 may contain a feasible point without proving objective
        # convergence. Feasibility is checked independently, then the full
        # nonlinear IK/PVAT segment must still pass. Keep the status in evidence.
        if (solved.status not in (0,8) or not np.isfinite(solved.x).all()
                or np.any(solved.x<low-1e-10) or np.any(solved.x>high+1e-10)
                or min(reserve(solved.x))<-1e-8 or min(task_rates(solved.x))<-1e-8 or not linear_ok(solved.x)):
            rejected=ValueError('cartesian_reference_optimizer_infeasible')
            rejected.optimizer_status=int(solved.status)
            rejected.minimum_reserve=float(min(reserve(solved.x)))
            rejected.minimum_task_margin=float(min(task_rates(solved.x)))
            raise rejected
        task_jerk=j@solved.x+bias
        if len(held_axes):task_jerk[held_axes]=0.
        curve=CartesianCurve.cubic(p,twist,acc,task_jerk,dt)
        segment,check=_solve_segment(self.model,self.state,curve,0.,dt,self.validation)
        self.state=segment.end
        return segment,dict(check,position_locked=locked,optimizer_iterations=int(solved.nit),
            optimizer_status=int(solved.status),optimizer_converged=bool(solved.success),demand_fraction=fraction,
            held_position_axes=tuple(int(x) for x in held_axes))


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


def plan_cartesian_braking(model,start,*,period=.08,durations=(.8,.96,1.04,1.12,1.2,1.28,1.44,1.6,1.92,2.4,3.2,4.8),
        position_radius=.02,angle_radius=.15,**validation):
    """Offline bounded continuation to zero PVA rates, never a stop command.

    Search mathematical durations without any executor access. Every segment
    must pass the same checks as motion, with an extra fixed radius about the
    braking origin. Exhaustion yields NO plan; do not substitute a zero-speed
    point, independent joint braking, or a physical stop guarantee.
    """
    pt=validation.get('position_tolerance',.001);at=validation.get('angle_tolerance',.01)
    if (not np.isfinite(position_radius) or not np.isfinite(angle_radius)
            or position_radius<=pt or angle_radius<=at or not durations):
        raise ValueError('cartesian_brake_budget')
    pose=model.pose(start.q);_,j,jd=model.geometry(start.q,start.v)
    velocity=j@start.v;acceleration=j@start.a+jd@start.v
    for duration in durations:
        if not np.isfinite(duration) or duration<period:raise ValueError('cartesian_reference_duration')
        # A quartic stopping chart has v(T)=a(T)=0, retaining the genuine
        # initial v/a. Its endpoint is forward of the origin rather than a
        # demand to return to the release position after decelerating.
        displacement=velocity*duration/2+acceleration*duration**2/12
        target=Pose(p=tuple(np.array(pose.p)+displacement[:3]),
            q=tuple((Rotation.from_rotvec(displacement[3:])*Rotation.from_quat(pose.q)).as_quat()))
        curve=_curve_from_reference(model,start,target,duration)
        lo,hi=_range(curve.coefficients)
        position_bound=np.linalg.norm(np.maximum(np.abs(lo[:3]-pose.p),np.abs(hi[:3]-pose.p)))+pt
        angle_bound=np.linalg.norm(np.maximum(-lo[3:],hi[3:]))+at
        if position_bound>position_radius or angle_bound>angle_radius:continue
        derivative=_derivative(curve.coefficients,1,duration)
        reversed_chart=False
        for part in (slice(0,3),slice(3,6)):
            tangent=velocity[part] if np.linalg.norm(velocity[part])>1e-8 else acceleration[part]
            low,_=_range((derivative[:,part]@tangent).reshape(-1,1))
            if low[0]<-1e-9:reversed_chart=True
        if reversed_chart:continue
        try:plan=plan_cartesian_reference(model,start,target,duration=duration,period=period,**validation)
        except ValueError as error:
            if not str(error).startswith('cartesian_reference_'):raise
            continue
        return plan,dict(duration_s=duration,position_radius_bound_m=float(position_bound),
            angle_radius_bound_rad=float(angle_bound),native_stop_verified=False,
            native_interpolation_verified=False)
    raise ValueError('cartesian_brake_no_feasible_plan')
