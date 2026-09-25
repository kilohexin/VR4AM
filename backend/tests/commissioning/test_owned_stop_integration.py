"""Real smoke/adapter/recorder with production request ownership.

No SDK import, sockets, site config, or real robot. Does not validate physics
or prove stop_move latency is repaired. Production app modules are not monkeypatched.
"""

import asyncio
import json

import pytest

from app.commissioning.actions import SmokeOptions, TranslationAction
from app.commissioning.smoke import run_smoke
from app.config import REAL_ROBOT_CONFIRMATION
from tests.commissioning.test_actions import _control_config
from tests.commissioning.test_stop_timeout_lifecycle import PendingStopClient
from tests.robots.fake_lebai import FakeLebaiClient


class DriftAfterFailedStopClient(PendingStopClient):
    def __init__(self, reply):
        super().__init__('late' if reply in {'late_error', 'read_disconnect'} else reply)
        self.late_error = reply == 'late_error'
        self.read_disconnect = reply == 'read_disconnect'

    async def _request_stop(self, method):
        await super()._request_stop(method)
        if self.late_error:
            raise RuntimeError('remote_stop_failed')

    async def get_kin_data(self):
        # Scripted fault injection, NOT a model of physical stop behavior.
        if self.stops >= 1:
            self.tail_reads += 1
            # First read belongs to smoke's stop-failure check; inject the
            # connection error into the subsequent observation instead.
            if self.read_disconnect and self.tail_reads == 2:
                raise ConnectionError('injected_read_connection_lost')
            if self.tail_reads > 1:
                self.robot_state = 'STOP'
                self.kin_data['actual_joint_pose'][1] += .004
                self.kin_data['actual_tcp_pose']['x'] -= .004
        return await FakeLebaiClient.get_kin_data(self)


@pytest.mark.parametrize('reply', ['late', 'never', 'late_error', 'read_disconnect'])
async def test_owned_stop_deduplicates_shutdown_without_hiding_fault_or_tail(
    tmp_path, monkeypatch, reply,
):
    # Production ownership retains the one stop_move request across shutdown.
    remote = DriftAfterFailedStopClient(reply)
    client = remote

    async def factory(ip):
        return client

    monkeypatch.setattr('app.commissioning.smoke.COMMISSIONING_LOG_ROOT', tmp_path / 'logs')
    try:
        with pytest.raises(RuntimeError, match='^smoke_stop_failed:stop_unverified$'):
            await asyncio.wait_for(run_smoke(
                SmokeOptions(_control_config(tmp_path), TranslationAction('x', .002),
                             REAL_ROBOT_CONFIRMATION, observe_stop_seconds=1.6),
                factory,
            ), timeout=12)
        folder, = (tmp_path / 'logs').iterdir()
        events = [json.loads(line) for line in (folder / 'session.jsonl').read_text().splitlines()]
        kinds = [e['kind'] for e in events]
        assert 'pvat_sent' in kinds
        first_stop = next(i for i, c in enumerate(remote.write_calls) if c[0] == 'stop_move')
        assert remote.write_calls[first_stop:] == [('stop_move',)]
        assert 'stop_confirmed' not in kinds
        assert 'smoke_result' not in kinds
        assert 'session_ended' in kinds
        diagnostics = [e for e in events if e['kind'] == 'stop_diagnostics']
        assert len(diagnostics) == 2  # Both callers remain visible in app logs.
        assert all(e['outcome'] == 'failed' for e in diagnostics)
        assert all(e['stop_policy'] == 'stop_move_only' for e in diagnostics)
        samples = [e['state'] for e in events if e['kind'] == 'stop_observation_sample']
        assert len(samples) >= 2
        assert all(s['latched_fault'] == 'stop_unverified' for s in samples)
        assert samples[-1]['actual_tcp']['p'][0] < samples[0]['actual_tcp']['p'][0] - .002
        summary = next(e for e in events if e['kind'] == 'stop_observation_summary')
        # A read failure must stay visible, not be turned into a clean pass.
        assert summary['complete'] is (reply != 'read_disconnect')
        assert summary['interrupted'] is False
        assert summary['successful_samples'] == len(samples)
        assert summary['read_errors'] == (1 if reply == 'read_disconnect' else 0)
        assert summary['dropped_events'] == 0
        assert json.loads((folder / 'summary.json').read_text())['dropped_normal_events'] == 0
        expected = {'late': 'returned_late', 'never': 'unknown_on_local_cancel',
                    'late_error': 'error', 'read_disconnect': 'returned_late'}[reply]
        lifecycle = [e for e in events if e['kind'] == 'stop_rpc_lifecycle']
        assert len(lifecycle) == 1
        assert {e['outcome'] for e in lifecycle} == {expected}
        assert all(
            e['started_ns'] <= e['sdk_await_started_ns']
            <= e['sdk_await_completed_ns'] <= e['completed_ns']
            for e in lifecycle
        )
        if reply == 'late_error':
            assert {e['error_type'] for e in lifecycle} == {'RuntimeError'}
        assert all(e['wait_failed'] for e in lifecycle)
        assert remote.overlap_seen is False
    finally:
        await remote.cleanup()
