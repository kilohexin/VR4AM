"""Pure theoretical kinematics; no SDK, controller or motion tests."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.commissioning.native_jog_mdh import MdhTool
from app.control.cartesian_pvat_reference import (CartesianCurve,JointReference,
    MdhReferenceModel,OfflineCartesianFollower,PvatSegment,plan_cartesian_reference,validate_segment)
from app.control.cartesian_pvat_reference import plan_cartesian_braking
from app.schemas.messages import Pose


@pytest.fixture
def model():
    return MdhReferenceModel(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)


def ready():return JointReference((0.,-.8,1.4,-.6,1.57,0.))


def pitch(pose,angle):
    return pose.model_copy(update={'q':tuple((Rotation.from_quat(pose.q)*Rotation.from_euler('x',angle)).as_quat())})


def test_rotating_reference_spatial_rates_match_independent_rotation_differences():
    start=Pose(p=(.3,0.,.2),q=tuple(Rotation.from_euler('xyz',(.2,-.3,.4)).as_quat()))
    target=Pose(p=(.31,.02,.2),q=tuple(Rotation.from_euler('xyz',(-.4,.3,.2)).as_quat()))
    curve=CartesianCurve(start,np.array([.01,0,0,.02,.04,-.03]),np.array([0,0,0,.03,-.01,.02]),target,4.)
    # Initial angular velocity and acceleration do not commute with the final
    # axis, so treating rotation-vector derivatives as omega would fail here.
    for t in (0.,.2,1.3,3.5):
        h=1e-4
        if t==0:
            _,v,a=curve.state(t)
            assert v==pytest.approx([.01,0,0,.02,.04,-.03])
            assert a==pytest.approx([0,0,0,.03,-.01,.02])
            continue
        minus,vm,_=curve.state(t-h);p,v,a=curve.state(t);plus,vp,_=curve.state(t+h)
        omega=(Rotation.from_quat(plus.q)*Rotation.from_quat(minus.q).inv()).as_rotvec()/(2*h)
        assert v[:3]==pytest.approx((np.array(plus.p)-minus.p)/(2*h),abs=1e-8)
        assert v[3:]==pytest.approx(omega,abs=1e-8)
        assert a==pytest.approx((vp-vm)/(2*h),abs=1e-8)


def test_pitch_reversal_keeps_fixed_tcp_height_and_c2_joint_state(model):
    initial=ready();base=model.pose(initial.q)
    first=plan_cartesian_reference(model,initial,pitch(base,.25),duration=4.)
    moving=first.state(2.)
    assert np.linalg.norm(moving.v)>.01
    with pytest.raises(ValueError,match='cartesian_reference_rate_2'):
        plan_cartesian_reference(model,moving,pitch(base,-.25),duration=6.)
    reverse=plan_cartesian_reference(model,moving,pitch(base,-.25),duration=10.)
    assert reverse.segments[0].start==moving  # No zeroing velocity at reversal.
    assert np.linalg.norm(reverse.segments[-1].end.v)<1e-8
    assert np.linalg.norm(reverse.segments[-1].end.a)<1e-8
    assert all(c['position_bound_m']<=.001 and c['angle_bound_rad']<=.01 for c in reverse.checks)
    maximum_error=0.
    for plan in (first,reverse):
        for t in np.linspace(0.,plan.curve.duration,151):
            actual=model.pose(plan.state(float(t)).q)
            maximum_error=max(maximum_error,np.linalg.norm(np.array(actual.p)-base.p))
    assert maximum_error<1e-5
    for before,after in zip(reverse.segments,reverse.segments[1:]):
        assert before.end==after.start


def test_valid_endpoints_do_not_admit_a_segment_with_an_interior_tcp_drop(model):
    origin=ready();base=model.pose(origin.q)
    v=(0.,.02,0.,0.,0.,0.)
    segment=PvatSegment(JointReference(origin.q,v),JointReference(origin.q,tuple(-x for x in v)),4.)
    curve=CartesianCurve(base,np.zeros(6),np.zeros(6),base,4.)
    # Both joint endpoints and their FK are identical. Its interior leaves the
    # fixed-height corridor despite low speeds, accelerations and jerks.
    assert model.pose(segment.start.q)==model.pose(segment.end.q)
    assert np.linalg.norm(np.array(model.pose(segment.state(2.).q).p)-base.p)>.005
    with pytest.raises(ValueError,match='cartesian_reference_corridor'):
        validate_segment(segment,model,curve,0.)


def test_translation_and_rotation_share_one_cartesian_reference(model):
    start=ready();base=model.pose(start.q)
    target=pitch(base,.12).model_copy(update={'p':(base.p[0]+.01,base.p[1]-.01,base.p[2]+.01)})
    plan=plan_cartesian_reference(model,start,target,duration=4.)
    end=model.pose(plan.state(4.).q)
    assert end.p==pytest.approx(target.p,abs=2e-7)
    assert (Rotation.from_quat(end.q)*Rotation.from_quat(target.q).inv()).magnitude()<2e-6
    # Interior validation explicitly labels the native interpolation unknown.
    assert all(not c['native_interpolation_verified'] for c in plan.checks)


def test_infeasible_duration_rejects_instead_of_changing_rate_limits(model):
    start=ready()
    with pytest.raises(ValueError,match='cartesian_reference_rate'):
        plan_cartesian_reference(model,start,pitch(model.pose(start.q),.25),duration=.8)


def test_singular_and_unreachable_targets_reject_without_a_fallback(model):
    singular=JointReference((0.,)*6)
    with pytest.raises(ValueError,match='cartesian_reference_singular'):
        plan_cartesian_reference(model,singular,model.pose(singular.q),duration=.8)
    start=ready();target=model.pose(start.q).model_copy(update={'p':(10.,10.,10.)})
    with pytest.raises(ValueError,match='cartesian_reference_ik_unreachable'):
        model.inverse(target,np.array(start.q))


@pytest.mark.parametrize('values',[(float('nan'),)*6,(0.,)*5])
def test_nonfinite_or_incomplete_boundary_state_is_rejected(values):
    with pytest.raises(ValueError,match='cartesian_reference_nonfinite_or_shape'):JointReference(values)


def test_unresolved_interval_cannot_be_accepted_by_exhausting_subdivisions(model):
    start=ready();base=model.pose(start.q)
    following=JointReference(tuple(np.array(start.q)+.001))
    segment=PvatSegment(start,following,1.)
    curve=CartesianCurve(base,np.zeros(6),np.zeros(6),model.pose(following.q),1.)
    with pytest.raises(ValueError,match='cartesian_reference_unresolved_interval'):
        validate_segment(segment,model,curve,0.,max_depth=0)


def test_tcp_speed_is_checked_independently_from_joint_rates(model):
    start=ready();base=model.pose(start.q)
    target=base.model_copy(update={'p':(base.p[0]+.01,base.p[1],base.p[2])})
    with pytest.raises(ValueError,match='cartesian_reference_task_rate'):
        plan_cartesian_reference(model,start,target,duration=4.,tcp_speed=.001)


@pytest.mark.parametrize('demand_mode',['coupled','separate'])
def test_continuous_follower_reverses_before_the_first_goal_without_a_height_drop(model,demand_mode):
    initial=ready();base=model.pose(initial.q);follower=OfflineCartesianFollower(model,initial,demand_mode=demand_mode)
    peak_error=0.;peak_speed=0.;moving_at_reversal=None
    for tick in range(180):
        if tick==15:moving_at_reversal=np.linalg.norm(follower.state.v)
        goal=pitch(base,.15 if tick<15 else -.15)
        previous=follower.state
        segment,check=follower.advance(goal)
        assert segment.start==previous and segment.end==follower.state
        assert check['position_locked']
        peak_speed=max(peak_speed,max(abs(x) for x in segment.end.v))
        # Include interiors of every assumed controller segment.
        for t in (0.,.04,.08):
            pose=model.pose(segment.state(t).q)
            peak_error=max(peak_error,np.linalg.norm(np.array(pose.p)-base.p))
    assert moving_at_reversal>.01 and peak_speed>.02
    assert peak_error<1e-6
    end=model.pose(follower.state.q)
    assert (Rotation.from_quat(end.q)*Rotation.from_quat(pitch(base,-.15).q).inv()).magnitude()<1e-4


def test_failed_candidate_does_not_commit_partial_follower_state(model,monkeypatch):
    initial=ready();follower=OfflineCartesianFollower(model,initial)
    def unreachable(*args):raise ValueError('cartesian_reference_ik_unreachable')
    monkeypatch.setattr(model,'inverse',unreachable)
    with pytest.raises(ValueError,match='cartesian_reference_ik_unreachable'):
        follower.advance(pitch(model.pose(initial.q),.1))
    assert follower.state is initial


def test_mdh_link_radii_bound_tcp_displacement_over_joint_intervals(model):
    rng=np.random.default_rng(17)
    for _ in range(30):
        q=rng.uniform(-2.5,2.5,6);change=rng.uniform(-.1,.1,6)
        a=model.pose(q);b=model.pose(q+change)
        assert np.linalg.norm(np.array(a.p)-b.p)<=model.position_lipschitz@np.abs(change)+1e-12
        assert (Rotation.from_quat(a.q)*Rotation.from_quat(b.q).inv()).magnitude()<=sum(abs(change))+1e-12


def test_interval_task_rate_bounds_enclose_independently_evaluated_mixed_segments(model):
    rng=np.random.default_rng(19);start=ready();base=model.pose(start.q)
    for _ in range(8):
        end=JointReference(tuple(np.array(start.q)+rng.uniform(-.005,.005,6)))
        segment=PvatSegment(start,end,1.)
        curve=CartesianCurve(base,np.zeros(6),np.zeros(6),model.pose(end.q),1.)
        check=validate_segment(segment,model,curve,0.)
        for t in np.linspace(0.,1.,25):
            s=segment.state(float(t));_,j,jd=model.geometry(s.q,s.v)
            v=j@s.v;a=j@s.a+jd@s.v
            assert np.linalg.norm(v[:3])<=check['tcp_speed_bound_mps']+1e-10
            assert np.linalg.norm(a[:3])<=check['tcp_acceleration_bound_mps2']+1e-10
            assert np.linalg.norm(v[3:])<=check['angular_speed_bound_radps']+1e-10
            assert np.linalg.norm(a[3:])<=check['angular_acceleration_bound_radps2']+1e-10


def test_governed_demand_preserves_the_original_large_hand_endpoint(model):
    start=ready();target=pitch(model.pose(start.q),.6)
    original=target.model_dump()
    follower=OfflineCartesianFollower(model,start)
    segment,check=follower.advance(target)
    assert 0<check['demand_fraction']<1
    assert target.model_dump()==original
    assert np.linalg.norm(np.array(segment.end.q)-start.q)>0
    assert check['position_locked']


def test_direct_quadratic_path_cannot_skip_segment_validation(model,monkeypatch):
    import app.control.cartesian_pvat_reference as module
    start=ready();follower=OfflineCartesianFollower(model,start)
    monkeypatch.setattr(module,'minimize',lambda *a,**kw:pytest.fail('quiet feasible optimum needs no iterations'))
    def reject(*a,**kw):raise ValueError('cartesian_reference_corridor')
    monkeypatch.setattr(module,'validate_segment',reject)
    with pytest.raises(ValueError,match='cartesian_reference_corridor'):
        follower.advance(model.pose(start.q))
    assert follower.state is start


@pytest.mark.parametrize('angle',[0.,.4,np.pi-1e-4])
def test_inverse_log_rotation_jacobian_matches_independent_central_differences(model,angle):
    q=np.array(ready().q)
    base=model.pose(q)
    axis=np.array([1.,2.,-3.]);axis/=np.linalg.norm(axis)
    target=base.model_copy(update={'q':tuple((Rotation.from_rotvec(axis*angle)*Rotation.from_quat(base.q)).as_quat())})
    _,jac=model.inverse_linearization(target,q)
    h=1e-7
    numeric=np.column_stack([(model.inverse_linearization(target,q+np.eye(6)[i]*h)[0]
        -model.inverse_linearization(target,q-np.eye(6)[i]*h)[0])/(2*h) for i in range(6)])
    assert jac==pytest.approx(numeric,abs=1e-7)


def test_error_enclosure_rejects_an_excursion_hidden_between_matching_midpoint_and_endpoints(model):
    initial=ready();base=model.pose(initial.q)
    a=(0.,.02,0.,0.,0.,0.)
    segment=PvatSegment(JointReference(initial.q,a=a),JointReference(initial.q,a=tuple(-x for x in a)),4.)
    assert model.pose(segment.state(2.).q).p==pytest.approx(base.p,abs=1e-12)
    assert np.linalg.norm(np.array(model.pose(segment.state(1.).q).p)-base.p)>.001
    curve=CartesianCurve(base,np.zeros(6),np.zeros(6),base,4.)
    with pytest.raises(ValueError,match='cartesian_reference_corridor'):
        validate_segment(segment,model,curve,0.)


def test_bounded_cartesian_brake_preserves_pitch_height_and_ends_with_zero_rates(model):
    initial=ready();base=model.pose(initial.q);follower=OfflineCartesianFollower(model,initial)
    for _ in range(12):follower.advance(pitch(base,.15))
    moving=follower.state
    assert np.linalg.norm(moving.v)>.01
    plan,report=plan_cartesian_braking(model,moving)
    assert plan.segments[0].start==moving
    assert np.linalg.norm(plan.segments[-1].end.v)<1e-8
    assert np.linalg.norm(plan.segments[-1].end.a)<1e-8
    assert report['position_radius_bound_m']<=.02 and report['angle_radius_bound_rad']<=.15
    assert not report['native_stop_verified']
    for t in np.linspace(0.,plan.curve.duration,41):
        pose=model.pose(plan.state(float(t)).q)
        assert pose.p==pytest.approx(base.p,abs=1e-6)


def test_too_small_braking_budget_rejects_without_mutating_the_moving_reference(model):
    initial=ready();base=model.pose(initial.q);follower=OfflineCartesianFollower(model,initial)
    for _ in range(12):follower.advance(pitch(base,.15))
    moving=follower.state
    with pytest.raises(ValueError,match='cartesian_brake_no_feasible_plan'):
        plan_cartesian_braking(model,moving,angle_radius=.010001)
    assert follower.state is moving


def test_remote_wrist_goal_does_not_suppress_a_small_lift_in_separate_demand_mode(model):
    initial=ready();base=model.pose(initial.q)
    target=pitch(base,.6).model_copy(update={'p':(base.p[0],base.p[1],base.p[2]+.004)})
    common=OfflineCartesianFollower(model,initial)
    separate=OfflineCartesianFollower(model,initial,demand_mode='separate')
    common.advance(target);separate.advance(target)
    common_lift=model.pose(common.state.q).p[2]-base.p[2]
    separate_lift=model.pose(separate.state.q).p[2]-base.p[2]
    assert common_lift>=-1e-9 and separate_lift>2*max(0.,common_lift)
    assert model.pose(separate.state.q).p[:2]==pytest.approx(base.p[:2],abs=1e-8)


def test_optimizer_constraint_jacobians_match_independent_central_differences(model,monkeypatch):
    import app.control.cartesian_pvat_reference as module
    original=module.minimize;checked=[]
    def verified_minimize(*args,**kwargs):
        x=np.arange(1.,7.)*.02;h=1e-5
        for item in kwargs['constraints']:
            if not isinstance(item,dict):continue
            numeric=np.column_stack([(item['fun'](x+np.eye(6)[i]*h)-item['fun'](x-np.eye(6)[i]*h))/(2*h)
                for i in range(6)])
            assert item['jac'](x)==pytest.approx(numeric,abs=1e-7)
            checked.append(True)
        return original(*args,**kwargs)
    monkeypatch.setattr(module,'minimize',verified_minimize)
    initial=ready();base=model.pose(initial.q);follower=OfflineCartesianFollower(model,initial)
    target=pitch(base,.6).model_copy(update={'p':(base.p[0],base.p[1],base.p[2]+.004)})
    for _ in range(5):follower.advance(target)
    assert len(checked)>=2


@pytest.mark.parametrize('omega',[(0.,0.,0.),(.2,-.15,.1)])
def test_angular_endpoint_complex_step_matches_real_differences_for_both_formula_branches(omega):
    from app.control.cartesian_pvat_reference import _angular_endpoint
    omega=np.array(omega);alpha=np.array([.1,.05,-.1]);gamma=np.array([.2,-.1,.3])
    complex_jac=[];numeric=[]
    for axis in range(3):
        c=gamma.astype(complex);c[axis]+=1e-24j
        complex_jac.append(np.concatenate(_angular_endpoint(c,omega,alpha,.08)).imag/1e-24)
        h=np.eye(3)[axis]*1e-5
        numeric.append((np.concatenate(_angular_endpoint(gamma+h,omega,alpha,.08))
            -np.concatenate(_angular_endpoint(gamma-h,omega,alpha,.08)))/(2e-5))
    assert np.array(complex_jac)==pytest.approx(np.array(numeric),abs=1e-7)
