"""One diagnostic read must never own the motion-stop schedule."""
import asyncio
import copy
import importlib
from types import SimpleNamespace

import pytest

from tests.robots.fake_lebai import FakeLebaiClient


def observer(native=None):
    module=importlib.import_module('app.commissioning.finite_motion_observation')
    now=[1_000_000_000]
    clock=lambda:now[0]
    completed=asyncio.create_task(asyncio.sleep(0))
    proxy=SimpleNamespace(client=native or FakeLebaiClient(),owner=asyncio.current_task(),
        clock=clock,deadline_clock=clock,read_deadline_ns=16_000_000_000,read_calls=[],
        sdk_stop_entries=0,pvat_entries=1,pvat_call={'outcome':'returned'},
        pvat_started_ns=now[0]-40_000_000,pvat_horizon_s=.08,pending_pvat=completed,
        read_guard=lambda:None,require_clear_read_gate=lambda:None)
    return module,proxy,now


async def spin():
    for _ in range(8):await asyncio.sleep(0)


async def test_one_owned_read_records_only_validated_telemetry():
    module,proxy,now=observer();await spin()
    observation=module.MotionObservation(proxy);observation.start();await spin()
    result=observation.snapshot()
    assert result['outcome']=='returned' and result['within_reference']
    assert result['native_entries']==1 and result['max_native_entries']==1
    assert result['sample']['actual_q']==proxy.client.kin_data['actual_joint_pose']
    assert result['physical_motion_verified'] is False
    assert proxy.client.read_calls==['get_kin_data'] and not proxy.client.write_calls
    proxy.client.kin_data['actual_joint_pose'][0]=123
    assert observation.snapshot()['sample']['actual_q'][0]!=123
    observation.revoke()
    assert not observation.blocks_reads and observation.snapshot()['outcome']=='returned'


async def test_revocation_before_task_runs_sends_no_read():
    module,proxy,now=observer();await spin()
    observation=module.MotionObservation(proxy);observation.start();observation.revoke();await spin()
    assert not proxy.client.read_calls
    assert observation.snapshot()['native_entries']==0


async def test_expired_reference_or_unknown_pvat_never_starts_read():
    module,proxy,now=observer();await spin()
    now[0]+=80_000_000
    observation=module.MotionObservation(proxy);observation.start();await spin()
    assert not proxy.client.read_calls and observation.snapshot()['native_entries']==0
    now[0]-=80_000_000;proxy.pvat_call['outcome']='timeout'
    observation=module.MotionObservation(proxy);observation.start();await spin()
    assert not proxy.client.read_calls


async def test_stubborn_native_read_cannot_delay_revocation_or_install_late_data():
    native=FakeLebaiClient();entered=asyncio.Event();release=asyncio.Event()
    async def stubborn():
        entered.set()
        while not release.is_set():
            try:await release.wait()
            except asyncio.CancelledError:pass
        return copy.deepcopy(native.kin_data)
    native.get_kin_data=stubborn
    module,proxy,now=observer(native);await spin()
    observation=module.MotionObservation(proxy);observation.start();await entered.wait()
    observation.revoke()  # Synchronous; never waits for native cancellation.
    assert observation.blocks_reads
    result=observation.snapshot()
    assert result['remote_result_unknown'] and result['sample'] is None
    release.set();await spin()
    assert observation.snapshot()['remote_result_unknown']
    assert observation.snapshot()['sample'] is None
    assert observation.blocks_reads


async def test_reply_after_reference_deadline_is_not_a_sample():
    native=FakeLebaiClient();entered=asyncio.Event();release=asyncio.Event()
    async def delayed():entered.set();await release.wait();return native.kin_data
    native.get_kin_data=delayed
    module,proxy,now=observer(native);await spin()
    observation=module.MotionObservation(proxy);observation.start();await entered.wait()
    now[0]+=80_000_000;release.set();await spin()
    assert observation.snapshot()['sample'] is None
    assert observation.blocks_reads


async def test_malformed_reply_cannot_become_motion_or_stop_proof():
    native=FakeLebaiClient();native.kin_data['actual_joint_pose']=[float('nan')]*6
    module,proxy,now=observer(native);await spin()
    observation=module.MotionObservation(proxy);observation.start();await spin()
    assert observation.snapshot()['outcome']=='error'
    assert observation.snapshot()['sample'] is None and observation.blocks_reads


async def test_hot_permission_is_rechecked_at_native_entry():
    module,proxy,now=observer();await spin()
    observation=module.MotionObservation(proxy);observation.start()
    def reject():raise ValueError('permission_revoked')
    proxy.read_guard=reject;await spin()
    assert not proxy.client.read_calls and observation.snapshot()['native_entries']==0


async def test_observation_opportunity_cannot_be_reused():
    module,proxy,now=observer();await spin()
    observation=module.MotionObservation(proxy);observation.start();await spin()
    with pytest.raises(ValueError,match='already_used'):observation.start()
    assert observation.snapshot()['native_entries']==1


async def test_native_cancel_after_entry_remains_remote_unknown():
    native=FakeLebaiClient();entered=asyncio.Event()
    async def pending():entered.set();await asyncio.Event().wait()
    native.get_kin_data=pending
    module,proxy,now=observer(native);await spin()
    observation=module.MotionObservation(proxy);observation.start();await entered.wait()
    observation.read_owner.pending_read.cancel();await spin()
    assert observation.snapshot()['remote_result_unknown']
    observation.revoke()
    assert observation.snapshot()['remote_result_unknown'] and observation.blocks_reads


async def test_shared_read_budget_is_not_reset_by_the_observer():
    module,proxy,now=observer();await spin()
    proxy.read_calls.extend([{} for _ in range(512)])
    observation=module.MotionObservation(proxy);observation.start();await spin()
    assert not proxy.client.read_calls
    assert len(proxy.read_calls)==512 and observation.snapshot()['native_entries']==0


async def test_sdk_future_reply_is_supported():
    module,proxy,now=observer();await spin()
    future=asyncio.get_running_loop().create_future();future.set_result(proxy.client.kin_data)
    proxy.client.get_kin_data=lambda:future
    observation=module.MotionObservation(proxy);observation.start();await spin()
    assert observation.snapshot()['outcome']=='returned'


async def test_stop_already_started_refuses_observation():
    module,proxy,now=observer();await spin()
    proxy.sdk_stop_entries=1
    observation=module.MotionObservation(proxy);observation.start();await spin()
    assert not proxy.client.read_calls and observation.snapshot()['native_entries']==0


async def test_final_recorded_timestamp_must_still_precede_deadline(monkeypatch):
    module,proxy,now=observer();await spin()
    observation=module.MotionObservation(proxy)
    poses=[0];original_pose=module.pose_from_lebai;original_authorize=observation._authorize
    def pose(value):
        result=original_pose(value);poses[0]+=1;return result
    def authorize():
        original_authorize()
        if poses[0]>=2:now[0]+=20_000_000  # Cross the end after the last guard.
    monkeypatch.setattr(module,'pose_from_lebai',pose)
    observation._authorize=authorize;observation.read_owner.read_authorize=authorize
    observation.start();await spin()
    assert observation.snapshot()['sample'] is None
    assert observation.blocks_reads
