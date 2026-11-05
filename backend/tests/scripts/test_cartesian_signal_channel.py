import asyncio
from types import SimpleNamespace

import pytest

from scripts.cartesian_signal_channel import ALLOWED,SDK_BRIDGE,CartesianSignalChannel
from scripts.cartesian_signal_publisher import CartesianSignalPublisher
from app.control.cartesian_clock import ControllerClock


def test_tunnel_is_fixed_virtual_loopback_and_exports_no_actuator_calls():
    assert "PeerAddr=>'127.0.0.1',PeerPort=>3030" in SDK_BRIDGE
    assert '3031' not in SDK_BRIDGE and 'PeerPort=>3010' not in SDK_BRIDGE
    assert ALLOWED=={'get_signal','get_signals','set_signal','set_signals','get_kin_data','get_robot_state'}


def test_channel_does_not_cancel_an_inflight_sdk_call_or_forget_unknown():
    async def run():
        done=asyncio.Event();result=asyncio.get_running_loop().create_future()
        channel=CartesianSignalChannel({})
        channel.client=SimpleNamespace(get_signal=lambda index:result)
        task=asyncio.create_task(channel.call('get_signal',0));await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):await task
        assert channel.unknown==['get_signal'] and channel.closed and not result.cancelled()
        result.set_result(0);await channel.close()
        assert channel.unknown==['get_signal']
        with pytest.raises(ValueError,match='operation'):await channel.call('move_pvat',[],[],[],.08)
    asyncio.run(run())


def test_sdk_publisher_withdraws_before_commit_without_inventing_unknown():
    from scripts.check_native_bundle_owner import fixtures
    packet=fixtures()[0]
    async def run():
        calls=[]
        planner=SimpleNamespace(session='01'*16,generation=1,closed=False,pending=SimpleNamespace(bundle=packet))
        async def call(name,*args):
            calls.append((name,args))
            if name=='set_signals':planner.closed=True
        publisher=CartesianSignalPublisher(SimpleNamespace(call=call),planner,epoch_ms=0,
            clock_pair=ControllerClock(0,0,0),host_clock=lambda:1_000_000)
        assert not await publisher.publish(packet)
        assert [name for name,_ in calls]==['set_signal','set_signals']
        assert calls[0][1]==(0,0)  # Inactive bank invalidated, never committed.
        assert not await publisher.publish(packet)
    asyncio.run(run())


def test_only_a_pending_checked_transaction_can_write_sdk_signals():
    async def run():
        planner=SimpleNamespace(session='01'*16,generation=1,closed=False,pending=None)
        publisher=CartesianSignalPublisher(None,planner,epoch_ms=0,
            clock_pair=ControllerClock(0,0,0),host_clock=lambda:1_000_000)
        with pytest.raises(ValueError,match='uncertified'):await publisher.publish(object())
    asyncio.run(run())


def test_release_before_first_input_is_an_explicit_bound_withdrawal():
    async def run():
        calls=[]
        async def call(name,*args):calls.append((name,args))
        planner=SimpleNamespace(session='01'*16,generation=7,closed=True,pending=None)
        publisher=CartesianSignalPublisher(SimpleNamespace(call=call),planner,epoch_ms=0,
            clock_pair=ControllerClock(0,0,0),host_clock=lambda:0)
        await publisher.release()
        assert publisher.closed
        assert calls==[('set_signals',(241,[16843009]*4+[7])),('set_signal',(240,-1))]
        assert not await publisher.offer_authority(0)
        assert len(calls)==2
    asyncio.run(run())
