"""Pure report checks: no SDK, network, configuration or robot access."""
import copy
import pytest


def report():
    return dict(runtime='LIVE_SINGLE_READY_RETURN',complete=True,connect_entries=1,
        movej_entries=1,stop_entries=0,post_preflight_ready=True,tail_complete=True,
        recording_closed_verified=True,local_resources_released=True,latched_fault=None,
        read_remote_result_unknown=False,movej_remote_result_unknown=False,
        pending_tasks_at_process_exit=0,cleanup_errors=[],speed_audit=dict(status='within_observed_limit'),
        connection_diagnostics=dict(factory_entries=1,outcome='returned',remote_result_unknown=False,
            native_sdk_init_entered=True,native_sdk_connect_entered=True))


def gate():
    from scripts.ready_then_position_gate import require_accepted_ready
    return require_accepted_ready


def test_only_terminal_live_accepted_return_can_continue():
    assert gate()(report(),exit_code=0,local_rpc_connections=0) is None


@pytest.mark.parametrize('field,value',[
    ('runtime','SIMULATION'),('complete',False),('connect_entries',2),('movej_entries',0),
    ('stop_entries',1),('post_preflight_ready',False),('tail_complete',False),
    ('recording_closed_verified',False),('local_resources_released',False),
    ('latched_fault','stop_unverified'),('read_remote_result_unknown',True),
    ('movej_remote_result_unknown',True),('pending_tasks_at_process_exit',1),
    ('cleanup_errors',['failure']),('movej_entries',True),('pending_tasks_at_process_exit',False),
    ('speed_audit',dict(status='exceeded')),
    ('connection_diagnostics',dict(factory_entries=1,outcome='returned',remote_result_unknown=True))])
def test_failed_or_unknown_stage_never_continues(field,value):
    r=copy.deepcopy(report());r[field]=value
    with pytest.raises(ValueError,match='ready_then_position_stage_one_not_accepted'):
        gate()(r,exit_code=0,local_rpc_connections=0)


@pytest.mark.parametrize('exit_code,rpc',[(2,0),(0,1),(True,0),(0,False)])
def test_process_or_connection_release_is_required(exit_code,rpc):
    with pytest.raises(ValueError):gate()(report(),exit_code=exit_code,local_rpc_connections=rpc)


def test_missing_evidence_never_continues():
    r=report();del r['read_remote_result_unknown']
    with pytest.raises(ValueError):gate()(r,exit_code=0,local_rpc_connections=0)


def test_both_stage_previews_bind_gate_source_without_importing_sdk():
    import sys
    from scripts import real_robot_ready_return,real_robot_finite_position
    for entry in (real_robot_ready_return,real_robot_finite_position):
        path=entry.ROOT/'scripts/ready_then_position_gate.py'
        assert entry.runtime_hashes()['scripts/ready_then_position_gate.py']==entry.sha(path)
    assert 'lebai_sdk' not in sys.modules
