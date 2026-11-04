"""Pure theoretical kinematics; no SDK, controller or motion tests."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.commissioning.native_jog_mdh import MdhTool
from app.control.cartesian_pvat_reference import (CartesianCurve,JointReference,
    MdhReferenceModel,OfflineCartesianFollower,PvatSegment,plan_cartesian_reference,validate_segment)
from app.control.cartesian_pvat_reference import plan_cartesian_braking
from app.control.cartesian_pvat_reference import SplitCartesianBrakeCurve,SegmentPlanningBudget
from app.schemas.messages import Pose


@pytest.fixture(params=['python','compiled'])
def model(request):
    factory=MdhReferenceModel
    if request.param=='compiled':
        pytest.importorskip('numba')
        from app.control.compiled_reference_geometry import CompiledReferenceModel
        factory=CompiledReferenceModel
    return factory(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)


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


def test_coordinated_rotation_error_can_be_enclosed_without_small_displacement_subintervals(model):
    start=ready();v=np.zeros(6);v[5]=.2;dt=.08
    initial=JointReference(start.q,tuple(v));end=JointReference(tuple(np.array(start.q)+v*dt),tuple(v))
    _,j=model.geometry(start.q)
    curve=CartesianCurve.cubic(model.pose(start.q),j@v,np.zeros(6),np.zeros(6),dt)
    segment=PvatSegment(initial,end,dt)
    check=validate_segment(segment,model,curve,0.,max_depth=0)
    assert check['intervals']==1 and check['angle_bound_rad']<.001
    for t in np.linspace(0.,dt,31):
        actual=model.pose(segment.state(float(t)).q);expected=curve.state(float(t))[0]
        assert (Rotation.from_quat(expected.q).inv()*Rotation.from_quat(actual.q)).magnitude()<1e-10


def test_angular_error_enclosure_rejects_interior_wrist_swing_with_matching_endpoints(model):
    start=ready();v=np.zeros(6);v[5]=.03;dt=2.
    segment=PvatSegment(JointReference(start.q,tuple(v)),JointReference(start.q,tuple(-v)),dt)
    base=model.pose(start.q);curve=CartesianCurve.cubic(base,np.zeros(6),np.zeros(6),np.zeros(6),dt)
    assert (Rotation.from_quat(base.q).inv()*Rotation.from_quat(model.pose(segment.state(1.).q).q)).magnitude()>.01
    with pytest.raises(ValueError,match='cartesian_reference_corridor'):validate_segment(segment,model,curve,0.)


def test_reported_angular_error_bound_encloses_dense_noncommuting_reference(model):
    start=ready();base=model.pose(start.q)
    target=base.model_copy(update={'q':tuple((Rotation.from_rotvec([.12,-.08,.05])
        *Rotation.from_quat(base.q)).as_quat())})
    plan=plan_cartesian_reference(model,start,target,duration=4.)
    for index,(segment,check) in enumerate(zip(plan.segments,plan.checks)):
        for t in np.linspace(0.,segment.duration,7):
            actual=model.pose(segment.state(float(t)).q)
            expected=plan.curve.state(min(plan.curve.duration,index*segment.duration+float(t)))[0]
            error=(Rotation.from_quat(expected.q).inv()*Rotation.from_quat(actual.q)).magnitude()
            assert error<=check['angle_bound_rad']+1e-12


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


def test_neighboring_inverse_uses_bounded_local_newton_with_fresh_fk_confirmation(model,monkeypatch):
    import app.control.cartesian_pvat_reference as module
    start=np.array(ready().q);expected=start+np.array([.002,-.001,.001,.002,-.001,.003])
    target=model.pose(expected)
    monkeypatch.setattr(module,'least_squares',lambda *a,**kw:pytest.fail('unneeded general solver'))
    actual=model.inverse(target,start)
    assert actual==pytest.approx(expected,abs=1e-9)
    assert model.pose(actual).p==pytest.approx(target.p,abs=1e-11)


def test_local_inverse_cannot_skip_general_solver_when_seed_is_singular(model,monkeypatch):
    import app.control.cartesian_pvat_reference as module
    calls=[];original=module.least_squares
    def recorded(*a,**kw):
        calls.append(True);return original(*a,**kw)
    monkeypatch.setattr(module,'least_squares',recorded)
    target=model.pose(ready().q).model_copy(update={'p':(10.,10.,10.)})
    with pytest.raises(ValueError,match='ik_unreachable'):model.inverse(target,np.zeros(6))
    assert calls==[True]


def test_local_inverse_does_not_clip_or_accept_an_out_of_bounds_solution(model):
    bounded=MdhReferenceModel(model.tool,(-.1,-.9,1.3,-.7,1.4,-.1),(.1,-.7,1.5,-.5,1.7,.1))
    target=model.pose((.2,-.8,1.4,-.6,1.57,0.))
    with pytest.raises(ValueError,match='ik_unreachable'):bounded.inverse(target,np.array(ready().q))


def test_zero_inverse_residual_cannot_replace_fresh_fk_evidence(model,monkeypatch):
    import app.control.cartesian_pvat_reference as module
    from types import SimpleNamespace
    target=pitch(model.pose(ready().q),.1)
    monkeypatch.setattr(model,'inverse_linearization',lambda *a:(np.zeros(6),np.eye(6)))
    monkeypatch.setattr(module,'least_squares',lambda *a,**kw:SimpleNamespace(x=np.array(ready().q)))
    with pytest.raises(ValueError,match='ik_unreachable'):model.inverse(target,np.array(ready().q))


@pytest.mark.parametrize('values',[(float('nan'),)*6,(0.,)*5])
def test_nonfinite_or_incomplete_boundary_state_is_rejected(values):
    with pytest.raises(ValueError,match='cartesian_reference_nonfinite_or_shape'):JointReference(values)


def test_exactly_quiet_braking_is_one_fully_validated_hold(model,monkeypatch):
    import app.control.cartesian_pvat_reference as module
    start=ready();budget=SegmentPlanningBudget(1)
    plan,_=plan_cartesian_braking(model,start,segment_budget=budget)
    assert len(plan.segments)==1 and budget.used==1
    assert plan.segments[0].start==plan.segments[0].end==start
    def reject(*a,**kw):raise ValueError('cartesian_reference_corridor')
    monkeypatch.setattr(module,'validate_segment',reject)
    with pytest.raises(ValueError,match='corridor'):plan_cartesian_braking(model,start)


def test_budget_exhaustion_never_returns_a_partly_checked_braking_plan(model):
    start=ready();follower=OfflineCartesianFollower(model,start)
    moving,_=follower.advance(pitch(model.pose(start.q),.15))
    budget=SegmentPlanningBudget(1)
    with pytest.raises(ValueError,match='search_budget_exhausted'):
        plan_cartesian_braking(model,moving.end,segment_budget=budget)
    assert budget.used==1


def test_joint_fit_braking_uses_one_inverse_but_validates_all_cartesian_intervals(model,monkeypatch):
    follower=OfflineCartesianFollower(model,ready());base=model.pose(ready().q)
    for _ in range(4):segment,_=follower.advance(pitch(base,.15))
    inverse=model.inverse;calls=[]
    def recorded(*a,**kw):
        calls.append(True);return inverse(*a,**kw)
    monkeypatch.setattr(model,'inverse',recorded)
    plan,_=plan_cartesian_braking(model,segment.end,method='joint_fit',durations=(1.04,),allow_split=False)
    assert len(calls)==1 and len(plan.segments)>1 and len(plan.checks)==len(plan.segments)
    assert plan.segments[0].start==segment.end
    assert all(a.end==b.start for a,b in zip(plan.segments,plan.segments[1:]))
    assert plan.segments[-1].end.v==plan.segments[-1].end.a==(0.,)*6
    for index,(piece,check) in enumerate(zip(plan.segments,plan.checks)):
        assert not check['native_interpolation_verified']
        for t in np.linspace(0.,piece.duration,7):
            actual=model.pose(piece.state(float(t)).q)
            expected=plan.curve.state(min(plan.curve.duration,index*piece.duration+float(t)))[0]
            assert np.linalg.norm(np.array(actual.p)-expected.p)<=check['position_bound_m']+1e-12
            assert (Rotation.from_quat(expected.q).inv()*Rotation.from_quat(actual.q)).magnitude()<=check['angle_bound_rad']+1e-12


def test_fitted_stop_cannot_return_a_plan_when_an_interior_check_refuses(model,monkeypatch):
    import app.control.cartesian_pvat_reference as module
    follower=OfflineCartesianFollower(model,ready());piece,_=follower.advance(pitch(model.pose(ready().q),.15))
    calls=[]
    def refusal(*a,**kw):
        calls.append(True);raise ValueError('cartesian_reference_corridor')
    monkeypatch.setattr(module,'validate_segment',refusal)
    with pytest.raises(ValueError,match='search_budget_exhausted'):
        plan_cartesian_braking(model,piece.end,method='joint_fit',segment_budget=SegmentPlanningBudget(2))
    assert len(calls)==2


@pytest.mark.parametrize('limit',[True,0,-1,1.5,2001])
def test_invalid_model_work_budget_is_rejected(limit):
    with pytest.raises(ValueError,match='configuration'):SegmentPlanningBudget(limit)


def test_curve_budget_stops_expensive_repeated_inverse_attempts_before_segment_budget(model,monkeypatch):
    import app.control.cartesian_pvat_reference as module
    follower=OfflineCartesianFollower(model,ready());piece,_=follower.advance(pitch(model.pose(ready().q),.15))
    calls=[]
    def reject(*a,**kw):
        calls.append(True);raise ValueError('cartesian_reference_corridor')
    monkeypatch.setattr(module,'validate_segment',reject)
    budget=SegmentPlanningBudget(24,2)
    with pytest.raises(ValueError,match='search_budget_exhausted'):
        plan_cartesian_braking(model,piece.end,method='joint_fit',segment_budget=budget)
    assert budget.curves==2 and budget.used==2 and budget.exhausted and len(calls)==2


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


def mixed_braking_state(model):
    # Synthetic task derivatives, not a field pose or recorded hardware state.
    q=np.array(ready().q);_,j=model.geometry(q)
    velocity=np.array([0.,.015,.006,-.004,.002,-.003])
    acceleration=np.array([-.003,.03,.001,.028,.002,.018])
    v=np.linalg.solve(j,velocity);_,_,jd=model.geometry(q,v)
    a=np.linalg.solve(j,acceleration-jd@v)
    return JointReference(tuple(q),tuple(v),tuple(a))


def test_independent_task_stop_horizons_solve_a_shared_horizon_refusal_without_larger_limits(model):
    start=mixed_braking_state(model)
    with pytest.raises(ValueError,match='cartesian_brake_no_feasible_plan'):
        plan_cartesian_braking(model,start,allow_split=False)
    plan,report=plan_cartesian_braking(model,start)
    assert isinstance(plan.curve,SplitCartesianBrakeCurve)
    assert report['rotation_stop_s']<report['position_stop_s']
    assert report['position_radius_bound_m']<=.02 and report['angle_radius_bound_rad']<=.15
    assert plan.segments[0].start==start
    assert all(a.end==b.start for a,b in zip(plan.segments,plan.segments[1:]))
    knot=report['rotation_stop_s']
    state=plan.state(knot);_,j,jd=model.geometry(state.q,state.v)
    assert np.linalg.norm((j@state.v)[3:])<1e-8
    assert np.linalg.norm((j@state.v)[:3])>.001
    assert np.linalg.norm(plan.segments[-1].end.v)<1e-8
    assert np.linalg.norm(plan.segments[-1].end.a)<1e-8
    fixed_rotation=Rotation.from_quat(model.pose(state.q).q)
    for t in np.linspace(knot,plan.curve.duration,13):
        p=model.pose(plan.state(float(t)).q)
        assert (Rotation.from_quat(p.q)*fixed_rotation.inv()).magnitude()<1e-5


def test_split_braking_curve_is_c2_at_the_shorter_stop_knot(model):
    start=mixed_braking_state(model);plan,report=plan_cartesian_braking(model,start)
    knot=report['rotation_stop_s'];before=plan.curve.state(knot-1e-7);at=plan.curve.state(knot);after=plan.curve.state(knot+1e-7)
    assert before[0].p==pytest.approx(after[0].p,abs=1e-8)
    assert before[1]==pytest.approx(after[1],abs=1e-7)
    assert before[2]==pytest.approx(after[2],abs=1e-6)
    assert np.linalg.norm(at[1][3:])==0 and np.linalg.norm(at[2][3:])==0


def test_validator_rejects_a_stop_knot_inside_a_pvat_interval(model):
    start=ready();base=model.pose(start.q)
    curve=SplitCartesianBrakeCurve(base,np.zeros(6),np.zeros(6),.8,.3)
    with pytest.raises(ValueError,match='brake_knot_alignment'):
        validate_segment(PvatSegment(start,start,.08),model,curve,.24)


def test_bad_limits_are_not_reinterpreted_as_a_known_braking_infeasibility(model):
    with pytest.raises(ValueError,match='validation_limits'):
        plan_cartesian_braking(model,ready(),speed=float('nan'))
