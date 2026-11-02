import asyncio
from types import SimpleNamespace

import pytest

from app.control.blended_position_session import BlendedPositionSession, BlendFeedback
from app.control.finite_position_types import PositionIntent


class Port:
    policy = 'docker_blended_candidate'

    def __init__(self):
        self.now = 1_000_000_000
        self.position = (0., 0., 0.)
        self.writes = []
        self.stops = 0
        self.stop_confirmed = True
        self.on_audit = None
        self.on_submit = None

    async def read(self, pending):
        self.now += 1_000_000
        return BlendFeedback(self.position, self.now, pending, True, True)

    async def audit(self, target):
        if self.on_audit:
            await self.on_audit()
        return target

    async def submit(self, plan, *, entering):
        entering()
        self.writes.append(plan.position)
        self.position = plan.position
        if self.on_submit:
            await self.on_submit()
        return len(self.writes)

    async def stop(self):
        self.stops += 1
        return self.stop_confirmed


def rig():
    p = Port()
    s = BlendedPositionSession(p, SimpleNamespace(generation=1),
        lower=(-.1,)*3, upper=(.1,)*3, clock=lambda:p.now, authorize=lambda:None)
    return s, p


def offer(s, p, seq, direction=(0., 0., 1.), session='web'):
    s.offer(PositionIntent(session, 1, seq, p.now,
        'held' if any(direction) else 'release', direction))


async def tick(s, p):
    p.now += 20_000_000
    await s.advance()
    if s.worker is not None:
        await s.worker


@pytest.mark.parametrize('source', ['web', 'xr-finite-controller'])
async def test_product_intents_continue_release_and_reverse(source):
    s, p = rig()
    for seq in range(1, 13):
        offer(s, p, seq, session=source)
        await tick(s, p)
    assert len(p.writes) == 12
    offer(s, p, 13, (0., 0., -1.), session=source)
    for seq in range(14, 26):
        offer(s, p, seq, (0., 0., -1.), session=source)
        await tick(s, p)
    assert any(b[2] < a[2] for a, b in zip(p.writes, p.writes[1:]))
    offer(s, p, 26, (0., 0., 0.), session=source)
    count = len(p.writes)
    for _ in range(15):
        await tick(s, p)
    assert len(p.writes) == count and s.phase == 'ready'
    assert not s.closed and not p.stops and not s.gate.pending


async def test_release_during_planning_prevents_write():
    s, p = rig()
    async def release():offer(s, p, 2, (0., 0., 0.))
    p.on_audit = release
    offer(s, p, 1)
    await tick(s, p)
    assert not p.writes and not p.stops and not s.closed


async def test_live_hand_direction_noise_does_not_drain_each_segment():
    s,p=rig()
    for seq in range(1,13):
        offer(s,p,seq,(0. if seq==1 else .02*(-1)**seq,0.,1.))
        await tick(s,p)
        assert not s.gate.draining
    assert len(p.writes)==12 and not p.stops
    offer(s,p,13,(0.,0.,0.))
    for _ in range(15):await tick(s,p)
    assert s.phase=='ready' and len(p.writes)==12


async def test_release_during_rpc_owns_late_result_and_drains():
    s, p = rig()
    async def release():offer(s, p, 2, (0., 0., 0.))
    p.on_submit = release
    offer(s, p, 1)
    await tick(s, p)
    assert s.gate.pending == [1] and s.phase == 'draining'
    for _ in range(15):await tick(s, p)
    assert len(p.writes) == 1 and not p.stops and s.phase == 'ready'


async def test_stale_held_input_stops_once_and_preserves_unverified():
    s, p = rig()
    offer(s, p, 1);await tick(s, p)
    p.stop_confirmed = False
    p.now += 100_000_000
    await s.advance()
    await s.abort('disconnect')
    await s.advance()
    assert p.stops == 1 and s.reason == 'stop_unverified' and s.closed
    with pytest.raises(ValueError, match='closed'):offer(s, p, 2)


async def test_disconnect_during_planning_cannot_later_enter():
    s, p = rig()
    paused, release = asyncio.Event(), asyncio.Event()
    async def audit():paused.set();await release.wait()
    p.on_audit = audit
    offer(s, p, 1);await s.advance();await paused.wait()
    await s.abort('disconnect')
    release.set();await s.worker
    assert not p.writes and not p.stops and s.reason == 'disconnect'


async def test_wrong_generation_rotation_and_session_rejected():
    s, p = rig();offer(s, p, 1)
    for intent in (
        PositionIntent('web', 2, 2, p.now, 'held', (1., 0., 0.)),
        PositionIntent('other', 1, 2, p.now, 'held', (1., 0., 0.)),
        PositionIntent('web', 1, 2, p.now, 'held', (0., 0., 0.), pitch=1.),
    ):
        with pytest.raises(ValueError):s.offer(intent)
    assert not p.writes


async def test_unknown_submission_stops_and_never_retries():
    s, p = rig()
    async def fail():raise TimeoutError('native_result_unknown')
    p.on_submit = fail
    offer(s, p, 1);await tick(s, p)
    await s.advance()
    assert s.closed and len(p.writes) == 1 and p.stops == 1
    assert s.reason == 'native_result_unknown'


async def test_port_cannot_enter_twice():
    s, p = rig()
    async def duplicate(plan, *, entering):
        entering();p.writes.append(plan.position);entering()
    p.submit = duplicate
    offer(s, p, 1);await tick(s, p)
    assert s.closed and s.reason == 'blend_duplicate_entry'
    assert s.entered_steps == 1 and p.stops == 1


async def test_held_queue_wait_remains_executing_until_release():
    s,p=rig()
    async def moving(pending):
        p.now+=1_000_000
        return BlendFeedback((0.,0.,0.),p.now,(),False,False)
    p.read=moving
    for seq in range(1,4):
        offer(s,p,seq);await tick(s,p)
    assert len(p.writes)==2 and s.phase=='executing'
    offer(s,p,4,(0.,0.,0.));await tick(s,p)
    assert s.phase=='draining' and len(p.writes)==2


@pytest.mark.parametrize('fail',[False,True])
async def test_page_read_pause_is_released_on_success_and_failure(fail):
    s,p=rig();events=[]
    p.begin_cycle=lambda:events.append('begin')
    p.end_cycle=lambda:events.append('end')
    async def audit():
        assert events==['begin']
        if fail:raise ValueError('path_invalid')
    p.on_audit=audit
    offer(s,p,1);await tick(s,p)
    assert events==['begin','end']
    assert len(p.writes)==(0 if fail else 1)


async def test_docker_expiry_drains_and_requires_release_before_fresh_rearm():
    s,p=rig();p.input_expiry_drains=True
    offer(s,p,1);await tick(s,p)
    p.now+=101_000_000
    await tick(s,p)
    count=len(p.writes)
    for seq in range(2,18):
        offer(s,p,seq);await tick(s,p)
    assert not s.closed and not p.stops and len(p.writes)==count
    assert s.phase=='hold' and s.reason=='position_input_expired'
    offer(s,p,18,(0.,0.,0.));await tick(s,p)
    offer(s,p,19);await tick(s,p)
    assert len(p.writes)==count+1 and s.reason is None


async def test_docker_expiry_during_audit_is_zero_write_hold():
    s,p=rig();p.input_expiry_drains=True
    async def expire():p.now+=101_000_000
    p.on_audit=expire
    offer(s,p,1);await tick(s,p)
    assert s.phase=='hold' and not p.writes and not p.stops and not s.closed


async def test_invalid_release_cannot_rearm_expired_docker_input():
    s,p=rig();p.input_expiry_drains=True
    offer(s,p,1);await tick(s,p);p.now+=101_000_000
    for _ in range(15):await tick(s,p)
    assert s.phase=='hold'
    with pytest.raises(ValueError):offer(s,p,1,(0.,0.,0.))
    offer(s,p,2);await tick(s,p)
    assert s.phase=='hold' and len(p.writes)==1


async def test_docker_expiry_does_not_mask_unknown_native_reply():
    s,p=rig();p.input_expiry_drains=True
    async def failed(plan,*,entering):
        entering();p.now+=101_000_000;s._monitor()
        raise ValueError('native_unknown')
    p.submit=failed;p.stop_confirmed=False
    offer(s,p,1);await tick(s,p)
    assert s.closed and s.reason=='stop_unverified' and p.stops==1
