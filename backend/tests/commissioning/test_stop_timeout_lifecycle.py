"""Characterize the real smoke/adapter lifecycle against an external SDK fake.

Not a controller physics model and not evidence that real motion is safe.
"""
import asyncio
import json

import pytest

from app.commissioning.actions import SmokeOptions, TranslationAction
from app.commissioning.smoke import run_smoke
from app.config import REAL_ROBOT_CONFIRMATION
from tests.commissioning.test_actions import _control_config
from tests.robots.fake_lebai import FakeLebaiClient


class PendingStopClient(FakeLebaiClient):
    def __init__(self, reply):
        super().__init__()
        self.reply = reply
        self.pending = []
        self.request_pose = None
        self.stops = 0
        self.overlap_seen = False
        self.tail_reads = 0

    async def kinematics_inverse(self, pose, joints):
        self.request_pose = dict(pose)
        return list(joints)

    async def move_pvat(self, p, v, a, t):
        result = await super().move_pvat(p, v, a, t)
        self.kin_data['actual_tcp_pose'] = dict(self.request_pose)
        return result

    async def _request_stop(self, method):
        self.overlap_seen |= any(not t.done() for t in self.pending)
        self.write_calls.append((method,))
        self.stops += 1
        self.robot_state = 'STOPPING'
        self.running_motion = 195

        async def server_reply():
            if self.reply == 'late':
                await asyncio.sleep(1.2)
            else:
                await asyncio.Event().wait()

        task = asyncio.create_task(server_reply())
        self.pending.append(task)
        # Cancellation of the caller does not cancel work already on a server.
        await asyncio.shield(task)

    async def stop_move(self):
        await self._request_stop('stop_move')

    async def stop_sys(self):
        await self._request_stop('stop_sys')

    async def get_kin_data(self):
        if self.stops >= 2:
            self.tail_reads += 1
            if self.tail_reads > 1:
                self.robot_state = 'STOP'
                self.kin_data['actual_joint_pose'][1] += .004
                self.kin_data['actual_tcp_pose']['x'] -= .004
        return await super().get_kin_data()

    async def cleanup(self):
        for task in self.pending:
            task.cancel()
        await asyncio.gather(*self.pending, return_exceptions=True)


@pytest.mark.asyncio
@pytest.mark.parametrize('reply', ['late', 'never'])
async def test_translation_stop_timeouts_keep_fault_and_finish_observation(
    tmp_path, monkeypatch, reply,
):
    # Catches swallowed stop errors, false confirmations, lost diagnostic tails,
    # and resumed PVAT writes after stop. The fake replaces only the SDK boundary.
    client = PendingStopClient(reply)

    async def factory(ip):
        return client

    monkeypatch.setattr('app.commissioning.smoke.COMMISSIONING_LOG_ROOT', tmp_path / 'logs')
    try:
        with pytest.raises(RuntimeError, match='^smoke_stop_failed:stop_unverified$'):
            await asyncio.wait_for(run_smoke(
                SmokeOptions(_control_config(tmp_path), TranslationAction('x', .002),
                             REAL_ROBOT_CONFIRMATION, observe_stop_seconds=.8),
                factory,
            ), timeout=12)
        folder, = (tmp_path / 'logs').iterdir()
        events = [json.loads(line) for line in (folder / 'session.jsonl').read_text().splitlines()]
        kinds = [e['kind'] for e in events]
        assert 'pvat_sent' in kinds  # Real translation path actually ran.
        first_stop = next(i for i, call in enumerate(client.write_calls) if call[0] == 'stop_move')
        assert all(call[0] in {'stop_move', 'stop_sys'} for call in client.write_calls[first_stop:])
        assert 'stop_confirmed' not in kinds
        assert 'smoke_result' not in kinds
        assert 'smoke_final_state' not in kinds
        diagnostics = [e for e in events if e['kind'] == 'stop_diagnostics']
        assert len(diagnostics) == 2
        assert client.overlap_seen  # First stop_move/stop_sys may still overlap.
        assert client.stops == 2  # Shutdown must not send either method again.
        assert [e['reason'] for e in diagnostics] == ['grip_released', 'shutdown']
        for index, event in enumerate(diagnostics):
            assert event['outcome'] == 'failed'
            expected_calls = [
                ('stop_move', 'timeout'), ('is_connected', 'returned'), ('stop_sys', 'timeout'),
            ]
            if index == 1:
                expected_calls = [('stop_move', 'timeout'), ('stop_sys', 'timeout')]
            assert [(c['method'], c['outcome']) for c in event['rpc_calls']] == expected_calls
        for method in ('stop_move', 'stop_sys'):
            first = next(c for c in diagnostics[0]['rpc_calls'] if c['method'] == method)
            second = next(c for c in diagnostics[1]['rpc_calls'] if c['method'] == method)
            assert first['request_id'] == second['request_id']
            assert first['reused'] is False and second['reused'] is True
        samples = [e['state'] for e in events if e['kind'] == 'stop_observation_sample']
        assert len(samples) >= 2
        assert all(s['latched_fault'] == 'stop_unverified' for s in samples)
        assert samples[-1]['actual_tcp']['p'][0] < samples[0]['actual_tcp']['p'][0] - .002
        assert samples[-1]['actual_q'][1] > samples[0]['actual_q'][1] + .002
        assert samples[-1]['raw_robot_state'] == 'STOP'
        start = next(e for e in events if e['kind'] == 'stop_observation_started')
        assert start['prior_error'] == 'smoke_stop_failed:stop_unverified'
        summary = next(e for e in events if e['kind'] == 'stop_observation_summary')
        assert summary['complete']  # Recording completeness only, NOT stop success.
        assert summary['successful_samples'] == len(samples)
        assert summary['read_errors'] == summary['dropped_events'] == 0
        assert json.loads((folder / 'summary.json').read_text())['dropped_normal_events'] == 0
    finally:
        await client.cleanup()
