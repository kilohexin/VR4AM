"""Public-view contract consumed by the offline transaction strategy."""

from dataclasses import replace
import inspect

import pytest

from app.control.safety import SafetyViolation
from app.schemas.messages import TeleopMode
from tests.control.test_transactional_vr_tick import rig
from tests.control import transactional_tick_probe


def test_strategy_has_no_private_adapter_access():
    assert 'adapter._' not in inspect.getsource(transactional_tick_probe)


@pytest.mark.parametrize('sample_kind', ['missing', 'old_generation', 'future', 'stale'])
async def test_unusable_observations_cannot_count_toward_pursuit_exit(sample_kind, monkeypatch):
    async with rig() as r:
        await r.warm(pursuit=True)
        await r.dispatch()
        probe = r.control.pursuit
        assert probe.dispatched and probe.pending is None
        read_view = r.run.adapter.read_motion_view
        for _ in range(3):
            r.clock.advance_ms(20)
            r.publish()
            view = read_view()
            sample = replace(view.sample, captured_ns=r.clock.now_ns(),
                actual_q=probe.solution, actual_qd=(0.,) * 6)
            if sample_kind == 'missing':
                sample = None
            elif sample_kind == 'old_generation':
                sample = replace(sample, generation=view.generation - 1)
            elif sample_kind == 'future':
                sample = replace(sample, captured_ns=r.clock.now_ns() + 1)
            else:
                sample = replace(sample, captured_ns=r.clock.now_ns() - 41_000_000)
            monkeypatch.setattr(r.run.adapter, 'read_motion_view', lambda: replace(view, sample=sample))
            update = r.control.collect(r.control._target_context())
            assert not update.stop and update.fault is None
            assert probe.good_samples == 0 and probe.state == 'pursuing'
        assert r.control.mode is TeleopMode.ACTIVE
        assert not r.control.exits


@pytest.mark.parametrize('acceptance', ['missing', 'changed'])
async def test_prepare_rejects_disappeared_or_changed_accepted_target(acceptance, monkeypatch):
    async with rig() as r:
        await r.warm(pursuit=True)
        view = r.run.adapter.read_motion_view()
        accepted = None if acceptance == 'missing' else replace(view.accepted,
            command=replace(view.accepted.command, submission_id=999))
        monkeypatch.setattr(r.run.adapter, 'read_motion_view', lambda: replace(view, accepted=accepted))
        strategy = r.control.target_strategy
        with pytest.raises(SafetyViolation, match='candidate_dispatch_failed'):
            strategy.prepare(r.control.last_target, r.control.latest_requested, .02,
                limiter=r.control.limiter)
