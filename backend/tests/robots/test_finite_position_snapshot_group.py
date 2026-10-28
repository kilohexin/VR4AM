"""Bounded finite-web read scheduling with injected clients only; no SDK/network."""
import asyncio
import time
import pytest
from app.robots.finite_position_web import RealFinitePositionAdapter
from app.robots.base import BackendCommandError
from tests.robots.test_finite_position_web import profile


async def rig():
    s,c,_=profile()
    async def factory(ip):return c
    a=RealFinitePositionAdapter(s,client_factory=factory,position_model=c.model,
        authorize=lambda:None,clock=time.perf_counter_ns)
    await a.connect()
    return a,c


async def test_slow_known_connection_check_does_not_consume_the_field_group_budget():
    a,c=await rig();connected=c.is_connected;tcp=c.get_tcp
    async def slow_connected():await asyncio.sleep(.27);return await connected()
    async def slow_tcp():await asyncio.sleep(.05);return await tcp()
    c.is_connected=slow_connected;c.get_tcp=slow_tcp
    try:
        started=time.perf_counter_ns()
        state=await a.get_state()
        assert state.server_mono_ns>=started
        assert time.perf_counter_ns()-state.server_mono_ns>=240_000_000
        assert not a._snapshot_calls.unknown_methods
        assert not c.write_calls
        # A completed slow display read never becomes new motion authority.
        with pytest.raises(BackendCommandError,match='position_snapshot_expired'):
            a._web_fresh_feedback()
    finally:await a.disconnect()


async def test_stop_verification_still_checks_connection_before_any_field():
    a,c=await rig();fields=[];entered=asyncio.Event();release=asyncio.Event()
    connected=c.is_connected;robot_state=c.get_robot_state
    async def waiting():entered.set();await release.wait();return await connected()
    async def field():fields.append('state');return await robot_state()
    c.is_connected=waiting;c.get_robot_state=field
    task=asyncio.create_task(a._read_snapshot(read_purpose='stop_verification'))
    try:
        await entered.wait();await asyncio.sleep(.01);assert not fields
        release.set();await task;assert fields==['state']
    finally:
        release.set();await asyncio.gather(task,return_exceptions=True);await a.disconnect()


@pytest.mark.parametrize('replace_public_snapshot',[False,True])
async def test_display_preflight_is_from_the_same_snapshot_and_cannot_enable_motion(replace_public_snapshot):
    a,c=await rig()
    original_snapshot=a._snapshot
    async def replace_during_recording(snapshot):a._snapshot=original_snapshot
    if replace_public_snapshot:a._emit_kinematics=replace_during_recording
    p=c.kin_data['actual_tcp_pose'];p['z']=a.settings.startup_tcp_min_m[2]-.03
    try:
        state=await a.get_state()
        assert state.preflight_ready is False
        assert state.preflight_reason=='tcp_outside_startup_envelope'
        assert not c.write_calls
    finally:await a.disconnect()


async def test_connection_and_fields_enter_together_without_extra_queries():
    a,c=await rig();field_entered=asyncio.Event();calls=[];get_state=c.get_robot_state
    async def connected():calls.append('connected');await field_entered.wait();return True
    async def field():calls.append('state');field_entered.set();return await get_state()
    c.is_connected=connected;c.get_robot_state=field
    try:
        await a.get_state()
        assert calls.count('connected')==calls.count('state')==1
        assert not c.write_calls
    finally:await a.disconnect()


async def test_unfinished_connection_still_hits_the_original_total_deadline_and_quarantines():
    a,c=await rig();original=a._snapshot;calls=[]
    async def connected():calls.append(1);await asyncio.sleep(.34);return True
    c.is_connected=connected
    try:
        with pytest.raises(BackendCommandError,match='robot_state_stale'):await a.get_state()
        assert a._snapshot is original
        assert 'is_connected' in a._snapshot_calls.unknown_methods
        with pytest.raises(BackendCommandError,match='snapshot_rpc_result_unknown'):await a.get_state()
        assert len(calls)==1 and not c.write_calls
    finally:await a.disconnect()


@pytest.mark.parametrize('abort',['cancel','disconnected','client_change'])
async def test_interrupted_group_never_publishes_partial_state_or_retries(abort):
    a,c=await rig();client=a._client;original=a._snapshot;field_entered=asyncio.Event();release=asyncio.Event()
    async def connected():
        await field_entered.wait()
        if abort=='disconnected':return False
        await release.wait();return True
    async def tcp():field_entered.set();await release.wait();return {}
    c.is_connected=connected;c.get_tcp=tcp
    task=asyncio.create_task(a.get_state())
    try:
        await asyncio.wait_for(field_entered.wait(),.5)
        if abort=='cancel':task.cancel()
        if abort=='client_change':a._client=object();release.set()
        with pytest.raises((asyncio.CancelledError,BackendCommandError)):await task
        await asyncio.sleep(.01)
        assert a._snapshot is original and not c.write_calls
        assert a._snapshot_calls.unknown_methods
    finally:
        release.set();a._client=client
        await asyncio.gather(task,return_exceptions=True)
        await a.disconnect()


async def test_ready_display_evidence_cannot_arm_a_revoked_preflight():
    a,c=await rig()
    a._preflight_ready=False
    try:
        state=await a.get_state()
        assert state.preflight_ready is True
        assert a._preflight_ready is False
        assert not c.write_calls
    finally:await a.disconnect()
