"""Local validation evidence must not change admission or publish vendor text."""
import pytest

from app.commissioning import native_jog_packet as packets
from app.commissioning import native_jog_site_run as runner
from tests.commissioning.test_native_jog_packet import packet, LIMITS, SHA


def validate(**changes):
    args=dict(limits=LIMITS,actual_p=(0,0,0),snapshot_ns=1_000_000_000,
        now_ns=1_000_000_000,generation=0,config_sha256=SHA)
    args.update(changes)
    fn=getattr(packets,'validate_packet_diagnostic',None)
    assert callable(fn), 'missing local validation diagnostics wrapper'
    return fn(packet(),boundary='site_motion_permission',**args)


@pytest.mark.parametrize('changes,code,snapshot_age,input_age', [
    ({'snapshot_ns':900_000_000},'native_jog_snapshot_stale',100_000_000,0),
    ({'snapshot_ns':1_000_000_001},'native_jog_snapshot_future',-1,0),
    ({'now_ns':1_100_000_000,'snapshot_ns':1_100_000_000},'native_jog_input_stale',0,100_000_000),
    ({'now_ns':999_999_999,'snapshot_ns':999_999_999},'native_jog_input_future',0,-1),
    ({'generation':1},'native_jog_packet_identity_or_expiry',0,0),
    ({'actual_p':(1,0,0)},'native_jog_packet_outside_planning_box',0,0),
])
def test_fixed_local_code_and_exact_failure_time(changes,code,snapshot_age,input_age):
    with pytest.raises(ValueError) as caught:validate(**changes)
    details=packets.local_validation_diagnostic(caught.value)
    assert details['code']==code
    assert details['boundary']=='site_motion_permission'
    assert details['snapshot_age_ns']==snapshot_age
    assert details['input_age_ns']==input_age
    assert runner._safe_reason(caught.value)==code
    assert caught.value.__cause__ is None
    # Return independent evidence; changing a report must not alter the error.
    details['code']='untrusted'
    assert packets.local_validation_diagnostic(caught.value)['code']==code


@pytest.mark.parametrize('age,accepted',[(-1,False),(0,True),(99_999_999,True),
    (100_000_000,False),(100_000_001,False)])
def test_exact_original_freshness_boundary(age,accepted):
    if accepted:assert validate(snapshot_ns=1_000_000_000-age) is None
    else:
        with pytest.raises(ValueError):validate(snapshot_ns=1_000_000_000-age)


@pytest.mark.parametrize('error',[ValueError('future or stale input/snapshot'),
    RuntimeError('secret vendor address/token'),ValueError('unstructured private data')])
def test_arbitrary_exception_is_not_promoted_to_local_evidence(error):
    fn=getattr(packets,'local_validation_diagnostic',None)
    assert callable(fn), 'missing safe local evidence reader'
    error.local_validation_rejection={'code':'native_jog_snapshot_stale','secret':'private'}
    assert fn(error) is None
    assert runner._safe_reason(error)=='native_jog_diagnostic_failed'


def test_unrecognized_boundary_and_unmapped_valueerror_stay_plain(monkeypatch):
    fn=getattr(packets,'validate_packet_diagnostic',None)
    assert callable(fn), 'missing local validation diagnostics wrapper'
    p=packet();args=dict(limits=LIMITS,actual_p=(0,0,0),snapshot_ns=900_000_000,
        now_ns=1_000_000_000,generation=0,config_sha256=SHA)
    with pytest.raises(ValueError) as caught:fn(p,boundary='private arbitrary text',**args)
    assert type(caught.value) is ValueError
    assert packets.local_validation_diagnostic(caught.value) is None
    def unrecognized(*args,**kwargs):raise ValueError('not a registered local reason')
    monkeypatch.setattr(packets,'validate_packet',unrecognized)
    with pytest.raises(ValueError) as caught:fn(p,boundary='site_motion_permission',**args)
    assert type(caught.value) is ValueError


@pytest.mark.parametrize('boundary',[
    'submission_before_model','submission_after_model','sdk_lock_before_model',
    'sdk_lock_after_model','owned_entry_before_model','owned_entry_after_model'])
async def test_core_failure_boundary_reaches_final_report_without_native_write(monkeypatch,boundary):
    from app.commissioning import native_jog_core as core
    from tests.commissioning.test_native_jog_site_run import setup,run
    original=getattr(core,'validate_packet_diagnostic',None)
    assert callable(original), 'core has no diagnostic validation boundary'
    def fail_at(packet,**kwargs):
        if kwargs['boundary']==boundary:
            kwargs['snapshot_ns']=kwargs['now_ns']-100_000_000
        return original(packet,**kwargs)
    monkeypatch.setattr(core,'validate_packet_diagnostic',fail_at)
    scope,client=setup();result,_=await run(scope,client)
    assert result['primary_rejection']=='native_jog_snapshot_stale',result
    assert result['local_validation_rejection']['boundary']==boundary
    assert result['local_validation_rejection']['snapshot_age_ns']==100_000_000
    assert result['speedl_entries']==result['stop_entries']==0
    assert not client.write_calls and result['latched_fault'] is None


async def test_site_permission_keeps_local_error_through_boolean_authority(monkeypatch):
    from tests.commissioning.test_native_jog_site_run import setup,run
    original=getattr(runner,'validate_packet_diagnostic',None)
    assert callable(original), 'runner has no diagnostic validation boundary'
    def expire(packet,**kwargs):
        kwargs['snapshot_ns']=kwargs['now_ns']-100_000_000
        return original(packet,**kwargs)
    monkeypatch.setattr(runner,'validate_packet_diagnostic',expire)
    scope,client=setup();result,_=await run(scope,client)
    assert result['primary_rejection']=='native_jog_snapshot_stale',result
    assert result['authority_rejection']=='native_jog_snapshot_stale'
    assert result['local_validation_rejection']['boundary']=='site_motion_permission'
    assert result['speedl_entries']==result['stop_entries']==0 and not client.write_calls


async def test_vendor_exception_after_entry_never_gets_local_payload():
    from tests.commissioning.test_native_jog_site_run import setup,run
    scope,client=setup()
    async def fail(*args):raise ValueError('secret vendor address/token')
    client.speedl=fail
    result,_=await run(scope,client)
    # An unknown native result must retain the existing stop-fault precedence.
    assert result['primary_rejection']=='stop_unverified',result
    assert 'local_validation_rejection' not in result
    assert 'secret' not in str(result)
    assert result['speedl_entries']==result['stop_entries']==1


async def test_normal_result_has_no_failure_diagnostics():
    from tests.commissioning.test_native_jog_site_run import run
    result,_=await run()
    assert result['complete'],result
    assert 'local_validation_rejection' not in result


@pytest.mark.parametrize('payload', [None, ('not a pair',),
    (('schema',1),('code',[])),
    (('schema',True),('code','native_jog_snapshot_stale'),
     ('boundary','site_motion_permission'),('checked_ns',1),
     ('snapshot_age_ns',0),('input_age_ns',0))])
def test_malformed_local_payload_cannot_break_reporting_or_leak(payload):
    with pytest.raises(ValueError) as caught:validate(snapshot_ns=900_000_000)
    caught.value._local_evidence=payload
    assert packets.local_validation_diagnostic(caught.value) is None


@pytest.mark.parametrize('boundary', [[],None,True,'private arbitrary boundary'])
def test_bad_diagnostic_boundary_does_not_replace_original_error(boundary):
    fn=getattr(packets,'validate_packet_diagnostic',None)
    assert callable(fn)
    with pytest.raises(ValueError) as caught:
        fn(packet(),boundary=boundary,limits=LIMITS,actual_p=(0,0,0),snapshot_ns=900_000_000,
            now_ns=1_000_000_000,generation=0,config_sha256=SHA)
    assert type(caught.value) is ValueError
    assert str(caught.value)=='future or stale input/snapshot'


def test_wrapper_calls_existing_validator_once_with_unchanged_sample(monkeypatch):
    p=packet();original=packets.validate_packet;calls=[]
    def measured(*args,**kwargs):calls.append(kwargs.copy());return original(*args,**kwargs)
    monkeypatch.setattr(packets,'validate_packet',measured)
    with pytest.raises(ValueError) as caught:
        packets.validate_packet_diagnostic(p,boundary='site_motion_permission',
            limits=LIMITS,actual_p=(0,0,0),snapshot_ns=900_000_000,
            now_ns=1_000_000_000,generation=0,config_sha256=SHA)
    assert len(calls)==1
    assert calls[0]['now_ns']==1_000_000_000
    assert str(caught.value)=='future or stale input/snapshot'


async def test_reused_report_does_not_keep_stale_local_failure():
    from tests.commissioning.test_native_jog_site_run import run
    result,_=await run(report={'local_validation_rejection':{'secret':'old'}})
    assert result['complete'],result
    assert 'local_validation_rejection' not in result
