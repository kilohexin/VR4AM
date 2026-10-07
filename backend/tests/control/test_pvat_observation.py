"""Cache facts stay distinct from predicted references, with no extra I/O."""
from dataclasses import replace

import pytest

from app.control.target_strategy import TargetContext
from app.robots.command_admission import CommandRef
from app.robots.motion_view import AcceptedTarget,JointObservation,MotionView
from tests.control.pvat_observation_probe import ObservationBinding,qualify_observation
from tests.robots.pvat_cruise_candidate import build_cruise_plan
from tests.robots.test_lebai_backpressure_experiments import experiment

ZERO=(0.,)*6
COMMAND=CommandRef(42,3,9)
BINDING=ObservationBinding(7,COMMAND,1_000_000_000)
NOW=1_040_000_000
CONTEXT=TargetContext(7,NOW,True,True)
AGE=50_000_000


def observation():
    return JointObservation(3,1_010_000_000,1_025_000_000,'state',
                            (.001,0.,0.,0.,0.,0.),(.02,0.,0.,0.,0.,0.),
                            (.1,0.,0.,0.,0.,0.))


def view():
    accepted=AcceptedTarget(COMMAND,(.3,0.,.3),(0.,0.,0.,1.),ZERO)
    return MotionView(NOW,3,True,True,None,None,True,False,accepted,observation())


def qualify(v=None,context=CONTEXT,binding=BINDING,**kwargs):
    return qualify_observation(view() if v is None else v,context,binding,
                               max_age_ns=kwargs.get('max_age_ns',AGE))


def test_qualified_pva_retains_source_read_interval_and_both_distinct_generations():
    evidence=qualify()
    assert evidence.owner_generation==7 and evidence.sample.generation==3
    assert evidence.command==COMMAND and evidence.checked_ns==NOW
    assert evidence.age_ns==30_000_000 and evidence.read_span_ns==15_000_000
    assert evidence.sample.actual_qdd==(.1,0.,0.,0.,0.,0.)
    assert evidence.sample.captured_ns==1_010_000_000
    assert evidence.sample.read_completed_ns==1_025_000_000


@pytest.mark.parametrize('purpose',['state','motion'])
def test_only_declared_state_or_motion_sources_qualify(purpose):
    v=view();v=replace(v,sample=replace(v.sample,read_purpose=purpose))
    assert qualify(v).sample.read_purpose==purpose


@pytest.mark.parametrize('purpose',['prediction','terminal_reference','connection','prepare','stop_verification'])
def test_predicted_and_other_purpose_sources_cannot_be_relabelled_as_motion_feedback(purpose):
    v=view();v=replace(v,sample=replace(v.sample,read_purpose=purpose))
    with pytest.raises(ValueError,match='pvat_observation_source'):qualify(v)


def test_a_mathematical_reference_window_cannot_replace_a_joint_observation():
    v=replace(view(),sample=build_cruise_plan(ZERO,(.001,0.,0.,0.,0.,0.)).window(0.,.08))
    with pytest.raises(ValueError,match='pvat_observation_source'):qualify(v)


@pytest.mark.parametrize('ctx',[replace(CONTEXT,generation=8),replace(CONTEXT,active=False),replace(CONTEXT,authorized=False)])
def test_robotcontrol_authority_is_not_replaced_by_pump_currency(ctx):
    assert view().is_current(3)
    with pytest.raises(ValueError,match='pvat_observation_authority'):qualify(context=ctx)


def test_boolean_owner_generation_cannot_alias_integer_authority():
    with pytest.raises(ValueError,match='pvat_observation_authority'):
        qualify(context=replace(CONTEXT,generation=True),binding=replace(BINDING,owner_generation=1))


def test_boolean_accepted_generation_cannot_alias_integer_pump_generation():
    v=view();command=replace(COMMAND,generation=1)
    v=replace(v,generation=1,sample=replace(v.sample,generation=1),
              accepted=replace(v.accepted,command=replace(command,generation=True)))
    with pytest.raises(ValueError,match='pvat_observation_generation'):
        qualify(v,binding=replace(BINDING,command=command))


def test_unknown_accepted_record_rejects_as_missing_binding_evidence():
    with pytest.raises(ValueError,match='pvat_observation_accepted_changed'):
        qualify(replace(view(),accepted=object()))


@pytest.mark.parametrize('change',[
    {'client_attached':False},{'pump_running':False},{'pump_fault':'injected'},
    {'latched_fault':'stop_unverified'},{'preflight_ready':False},{'stop_in_progress':True}])
def test_local_backend_gates_reject_even_if_sample_is_fresh(change):
    with pytest.raises(ValueError,match='pvat_observation_backend'):qualify(replace(view(),**change))


@pytest.mark.parametrize('change',['view','sample','command'])
def test_owner_and_sample_cannot_cross_pump_generation(change):
    v=view()
    if change=='view':v=replace(v,generation=4)
    elif change=='sample':v=replace(v,sample=replace(v.sample,generation=2))
    else:v=replace(v,accepted=replace(v.accepted,command=replace(COMMAND,generation=2)))
    with pytest.raises(ValueError,match='pvat_observation_generation'):qualify(v)


@pytest.mark.parametrize('command',[None,replace(COMMAND,command_id=43),replace(COMMAND,submission_id=10)])
def test_reused_command_id_is_insufficient_to_bind_acceptance(command):
    v=view();v=replace(v,accepted=None if command is None else replace(v.accepted,command=command))
    with pytest.raises(ValueError,match='pvat_observation_accepted_changed'):qualify(v)


@pytest.mark.parametrize('change',[
    {'captured_ns':-1},{'captured_ns':True},{'read_completed_ns':1_000_000_000},
    {'read_completed_ns':NOW+1},{'captured_ns':1.5}])
def test_invalid_sample_time_order_never_becomes_recent_feedback(change):
    v=view();v=replace(v,sample=replace(v.sample,**change))
    with pytest.raises(ValueError,match='pvat_observation_clock'):qualify(v)


def test_view_timestamp_cannot_be_from_future_relative_to_owner_context():
    with pytest.raises(ValueError,match='pvat_observation_clock'):
        qualify(replace(view(),observed_ns=NOW+1))


def test_a_recent_completion_does_not_hide_an_old_capture_before_the_sdk_lock():
    v=view();v=replace(v,sample=replace(v.sample,captured_ns=980_000_000,read_completed_ns=NOW))
    with pytest.raises(ValueError,match='pvat_observation_stale'):
        qualify(v,binding=replace(BINDING,not_before_ns=900_000_000))


def test_fresh_snapshot_started_before_reference_binding_is_not_new_reference_evidence():
    v=view();v=replace(v,sample=replace(v.sample,captured_ns=999_000_000))
    with pytest.raises(ValueError,match='pvat_observation_before_binding'):qualify(v)


@pytest.mark.parametrize('field,value',[
    ('actual_q',(0.,)*5),('actual_qd',(float('nan'),)*6),
    ('actual_qdd',(float('inf'),)*6)])
def test_invalid_actual_vectors_reject_without_clamping(field,value):
    v=view();v=replace(v,sample=replace(v.sample,**{field:value}))
    with pytest.raises(ValueError,match='pvat_observation_invalid'):qualify(v)


def test_legacy_sample_acceleration_is_missing_instead_of_assumed_zero():
    v=view();v=replace(v,sample=replace(v.sample,actual_qdd=None))
    with pytest.raises(ValueError,match='pvat_observation_acceleration_missing'):qualify(v)


@pytest.mark.parametrize('age',[0,-1,1.5,True])
def test_invalid_experiment_age_bound_rejects(age):
    with pytest.raises(ValueError,match='invalid_observation_limits'):qualify(max_age_ns=age)


async def test_real_motion_view_with_injected_sdk_is_passive_and_repeat_reads_do_not_refresh_age():
    async with experiment() as run:
        run.client.kin_data['actual_joint_acc']=[.01,-.02,.03,-.04,.05,-.06]
        run.clock.advance(.02);await run.adapter.get_state()
        before=run.adapter.read_motion_view();command=before.accepted.command
        binding=ObservationBinding(7,command,before.sample.captured_ns)
        ctx=TargetContext(7,run.clock.now_ns(),True,True)
        reads,writes=list(run.client.read_calls),list(run.client.write_calls)
        evidence=qualify_observation(before,ctx,binding,max_age_ns=AGE)
        assert evidence.sample.actual_qdd==(.01,-.02,.03,-.04,.05,-.06)
        run.clock.advance(.05)
        cached=run.adapter.read_motion_view();ctx=replace(ctx,now_ns=run.clock.now_ns())
        assert qualify_observation(cached,ctx,binding,max_age_ns=AGE).sample.captured_ns==before.sample.captured_ns
        run.clock.advance(.001);cached=run.adapter.read_motion_view();ctx=replace(ctx,now_ns=run.clock.now_ns())
        with pytest.raises(ValueError,match='pvat_observation_stale'):
            qualify_observation(cached,ctx,binding,max_age_ns=AGE)
        assert run.client.read_calls==reads and run.client.write_calls==writes


async def test_passive_qualification_of_latched_real_adapter_view_does_not_reset_or_send():
    async with experiment() as run:
        v=run.adapter.read_motion_view()
        binding=ObservationBinding(7,v.accepted.command,v.sample.captured_ns)
        run.adapter._latched_fault='stop_unverified'
        reads,writes=list(run.client.read_calls),list(run.client.write_calls)
        with pytest.raises(ValueError,match='pvat_observation_backend'):
            qualify_observation(run.adapter.read_motion_view(),
                                TargetContext(7,run.clock.now_ns(),True,True),binding,max_age_ns=AGE)
        assert run.adapter.read_motion_view().latched_fault=='stop_unverified'
        assert run.client.read_calls==reads and run.client.write_calls==writes
