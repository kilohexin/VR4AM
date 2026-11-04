"""Presentation deadlines and terminal task identity; no SDK factory."""
from types import SimpleNamespace as NS
from unittest.mock import AsyncMock
import asyncio
import pytest
from app.robots.base import BackendCommandError
from app.robots.finite_position_docker import DockerFinitePositionAdapter
from app.robots.finite_position_web import RealFinitePositionAdapter


def rig():
    a=object.__new__(DockerFinitePositionAdapter)
    a._docker_native=True;a.position_pose_follow_vr=True
    a._client=object();a._snapshot_calls=NS(closed=False,unknown_methods=set())
    a._pump=NS(generation=1);a.position_binding=object()
    a._docker_finished_task=None;a.position_busy=False;a.position_aborted=False
    a.docker_idle_input_safe=lambda:False
    return a


def test_only_native_pose_presentation_gets_longer_read_budget():
    a=rig()
    assert a._snapshot_read_timeout_ns('state')==1_000_000_000
    for purpose in ('preflight','position_progress','stop_verification'):
        assert a._snapshot_read_timeout_ns(purpose)==300_000_000
    for flag in ('_docker_native','position_pose_follow_vr'):
        setattr(a,flag,False)
        assert a._snapshot_read_timeout_ns('state')==300_000_000
        setattr(a,flag,True)


async def test_known_finished_idle_task_is_not_polled_on_every_display_frame(monkeypatch):
    a=rig();read=AsyncMock(return_value='FINISHED')
    monkeypatch.setattr(RealFinitePositionAdapter,'_read_retained_task_state',read)
    for _ in range(4):assert await a._read_retained_task_state(a._client,7,{},1,'state')=='FINISHED'
    assert read.await_count==1
    await a._read_retained_task_state(a._client,7,{},1,'stop_verification')
    assert read.await_count==2  # Presentation reuse never replaces a stop read.


@pytest.mark.parametrize('flag',['_docker_native','position_pose_follow_vr'])
async def test_other_modes_do_not_reuse_terminal_tasks_while_idle(monkeypatch,flag):
    a=rig();setattr(a,flag,False)
    read=AsyncMock(return_value='FINISHED')
    monkeypatch.setattr(RealFinitePositionAdapter,'_read_retained_task_state',read)
    for _ in range(2):await a._read_retained_task_state(a._client,7,{},1,'state')
    assert read.await_count==2


@pytest.mark.parametrize('change',['client','owner','generation','binding','task','aborted'])
async def test_terminal_proof_cannot_cross_its_identity(monkeypatch,change):
    a=rig();read=AsyncMock(return_value='FINISHED')
    monkeypatch.setattr(RealFinitePositionAdapter,'_read_retained_task_state',read)
    await a._read_retained_task_state(a._client,7,{},1,'state')
    task=7
    if change=='client':a._client=object()
    if change=='owner':a._snapshot_calls=NS(closed=False,unknown_methods=set())
    if change=='generation':a._pump.generation+=1
    if change=='binding':a.position_binding=object()
    if change=='task':task=8
    if change=='aborted':a.position_aborted=True
    await a._read_retained_task_state(a._client,task,{},1,'state')
    assert read.await_count==2


async def test_running_or_unknown_task_never_becomes_terminal_proof(monkeypatch):
    a=rig();read=AsyncMock(side_effect=['RUNNING','FINISHED'])
    monkeypatch.setattr(RealFinitePositionAdapter,'_read_retained_task_state',read)
    assert await a._read_retained_task_state(a._client,7,{},1,'state')=='RUNNING'
    assert a._docker_finished_task is None
    await a._read_retained_task_state(a._client,7,{},1,'state')
    a._snapshot_calls.unknown_methods={'get_motion_state'}
    read.side_effect=BackendCommandError('snapshot_rpc_result_unknown')
    with pytest.raises(BackendCommandError,match='unknown'):
        await a._read_retained_task_state(a._client,7,{},1,'state')
    assert a._snapshot_calls.unknown_methods=={'get_motion_state'}


async def test_late_return_after_generation_change_does_not_install_proof(monkeypatch):
    a=rig()
    async def changed(*args,**kwargs):a._pump.generation+=1;return 'FINISHED'
    monkeypatch.setattr(RealFinitePositionAdapter,'_read_retained_task_state',changed)
    await a._read_retained_task_state(a._client,7,{},1,'state')
    assert a._docker_finished_task is None


@pytest.mark.parametrize('pose',[False,True])
async def test_slow_retained_task_tail_after_250ms_of_other_reads(pose):
    from app.robots.lebai_snapshot_calls import OwnedSnapshotCalls
    a=rig();a.position_pose_follow_vr=pose
    owner=OwnedSnapshotCalls();a._snapshot_calls=owner
    loop=asyncio.get_running_loop();start=loop.time()
    # The recorded incident consumed ~249ms before a ~107ms tail RPC.
    # Keep the real owned RPC implementation, including unknown quarantine.
    a._clock=lambda:250_000_000+int((loop.time()-start)*1e9)
    a._snapshot_native_evidence=lambda client,name:(None,None)
    a._schedule_sdk_diagnostic=lambda event:None
    async def slow(task):await asyncio.sleep(.11);return 'FINISHED'
    a._client=NS(get_motion_state=slow)
    deadline=a._snapshot_read_timeout_ns('state')
    try:
        if pose:
            assert await a._read_retained_task_state(a._client,7,{},deadline,'state')=='FINISHED'
            assert not owner.unknown_methods
        else:
            with pytest.raises(BackendCommandError,match='sdk_timeout:get_motion_state'):
                await a._read_retained_task_state(a._client,7,{},deadline,'state')
            assert owner.unknown_methods=={'get_motion_state'}
    finally:await asyncio.gather(*tuple(owner.tasks),return_exceptions=True)
