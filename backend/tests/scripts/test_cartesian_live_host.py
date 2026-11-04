import asyncio
import re
from types import SimpleNamespace

import pytest

from scripts import check_cartesian_live_host as live
from scripts import check_native_cartesian_authority as authority
from scripts.cartesian_native_publisher import CartesianNativePublisher
from app.control.cartesian_clock import ControllerClock
from app.control.cartesian_pvat_reference import JointReference


def test_live_dry_executable_has_no_actuator_or_configuration_methods():
    source=live.program(JointReference((0.,-.8,1.4,-.6,1.57,0.)),'01'*16,'a9'*32)
    assert set(re.findall(r'lebai:(\w+)\(',source))=={
        'get_kin_data','set_signal'}
    assert '__BOOT_CLOCK__' not in source and '/proc/uptime' in source
    assert '__OWNER__' not in source and 'timestamp()' not in source
    assert not re.findall(r'(?<![:.\w])\b(?:move\w+|stop_\w+|start_\w+|set_claw)\s*\(',source)
    assert 'structural guard, not an independent' in source


def test_authority_evidence_requires_all_unique_native_contracts():
    raw='\n'.join('AUTHORITY_CHECK\t'+s for s in sorted(authority.CHECKS))+'\nAUTHORITY_DONE\ttrue\tnil'
    assert authority.analyze(raw)['passed']
    assert not authority.analyze(raw+'\nAUTHORITY_CHECK\tjoints_unchanged')['passed']
    assert not authority.analyze(raw.replace('true\tnil','false\terror'))['passed']
    source=authority.program()
    assert set(re.findall(r'lebai:(\w+)\(',source))=={'get_kin_data'}


def test_coarse_clock_bounds_are_explicit_and_conservative():
    c=ControllerClock(1_000_000_000,1_020_000_000,500,resolution_ms=10)
    assert c.capture_lower_ms(1_030_000_000)==500
    assert c.sample_lower_ns(530)==1_020_000_000
    assert c.check_pair(1_040_000_000,1_060_000_000,540).resolution_ms==10


def test_publisher_cannot_commit_a_packet_outside_pending_host_transaction():
    async def run():
        rpc=SimpleNamespace()
        planner=SimpleNamespace(session='01'*16,generation=1,closed=False,pending=None)
        p=CartesianNativePublisher(rpc,planner,epoch_ms=0,
            clock_pair=ControllerClock(0,0,0),host_clock=lambda:1_000_000)
        with pytest.raises(ValueError,match='uncertified'):await p.publish(object())
    asyncio.run(run())


def test_publication_keeps_raw_input_stamp_and_checks_native_age_after_payload():
    from scripts.check_native_bundle_owner import fixtures
    packet=fixtures()[0];bodies=[]
    async def run():
        async def request(body):
            bodies.append(body);return SimpleNamespace(fields=('committed',))
        planner=SimpleNamespace(session='01'*16,generation=1,closed=False,
            pending=SimpleNamespace(bundle=packet))
        p=CartesianNativePublisher(SimpleNamespace(request=request),planner,epoch_ms=1000,
            clock_pair=ControllerClock(0,0,1000),host_clock=lambda:1_000_000)
        assert await p.publish(packet)
        assert len(bodies)==1
        body=bodies[0]
        assert body.index('set_signals(')<body.index('if cartesian_now()')<body.rindex('set_signal(')
        assert 'cartesian_now()-1100>=100' in body
        assert 'get_signal(240)==0' in body
    asyncio.run(run())


def test_known_late_header_refusal_does_not_advance_bank():
    from scripts.check_native_bundle_owner import fixtures
    packet=fixtures()[0]
    async def run():
        async def request(body):return SimpleNamespace(fields=('late',))
        planner=SimpleNamespace(session='01'*16,generation=1,closed=False,pending=SimpleNamespace(bundle=packet))
        p=CartesianNativePublisher(SimpleNamespace(request=request),planner,epoch_ms=1000,
            clock_pair=ControllerClock(0,0,1000),host_clock=lambda:1_000_000)
        assert not await p.publish(packet) and p.bank==0
    asyncio.run(run())
