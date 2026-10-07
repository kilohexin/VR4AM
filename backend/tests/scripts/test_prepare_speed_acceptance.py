import json
from unittest.mock import AsyncMock

import pytest

from app.commissioning.smoke import parse_smoke_args, run_smoke
from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE
from tests.robots.fake_lebai import FakeLebaiClient


def options(tmp_path, monkeypatch):
    config = tmp_path/'control.yaml'
    config.write_text('home_stable_ms: 1\n'+REAL_CONFIG_TEMPLATE.format(mode='control')
                      .replace('max_joint_speed_radps: 0.15', 'max_joint_speed_radps: 0.05'))
    monkeypatch.setattr('app.commissioning.smoke.COMMISSIONING_LOG_ROOT', tmp_path/'logs')
    return parse_smoke_args(['prepare', '--config', str(config), '--confirm',
                             'I_UNDERSTAND_REAL_ROBOT_MOTION'])


def converging_client(speed):
    client = FakeLebaiClient.idle()
    move = client.movej
    read = client.get_kin_data
    moved = False
    samples = 0

    async def movej(p, a, v, t, r):
        nonlocal moved
        result = await move(p, a, v, t, r)
        moved = True
        client.kin_data['actual_joint_pose'] = list(p)
        client.kin_data['target_joint_pose'] = list(p)
        return result

    async def get_kin_data():
        nonlocal samples
        result = await read()
        if moved:
            result['actual_joint_speed'] = [0, 0, 0, 0, speed if samples == 0 else 0, 0]
            result['target_joint_speed'] = [0, 0, 0, 0, .05 if samples == 0 else 0, 0]
            samples += 1
        return result

    client.movej = movej
    client.get_kin_data = get_kin_data
    return client


def events(tmp_path):
    path = next((tmp_path/'logs').glob('*/session.jsonl'))
    return [json.loads(line) for line in path.read_text().splitlines()]


@pytest.mark.parametrize('speed', [.10546117916713782, -.105])
async def test_arrived_prepare_rejects_overspeed_and_preserves_evidence(tmp_path, monkeypatch, speed):
    action = options(tmp_path, monkeypatch)
    client = converging_client(speed)
    with pytest.raises(RuntimeError, match='^smoke_joint_speed_exceeded$'):
        await run_smoke(action, AsyncMock(return_value=client))
    log = events(tmp_path)
    audit = next(x for x in log if x['kind'] == 'joint_move_speed_audit')
    assert audit['status'] == 'exceeded'
    assert audit['actual_peak_radps'] == pytest.approx(abs(speed))
    assert audit['target_peak_radps'] == .05
    assert audit['arrival_stable'] is True
    assert audit['physical_speed_verified'] is False
    assert not any(x['kind'] == 'smoke_result' for x in log)
    # Recorder writes critical records ahead of already queued normal records.
    assert sum(x['kind'] == 'session_ended' for x in log) == 1
    # It completed its normal arrival; post-arrival acceptance adds no stop or retry.
    assert [c[0] for c in client.write_calls] == ['movej']


async def test_arrived_prepare_reports_observed_speed_separately_from_stability(tmp_path, monkeypatch):
    result = await run_smoke(options(tmp_path, monkeypatch),
                             AsyncMock(return_value=converging_client(.05)))
    assert result.stable is True
    audit = result.to_dict()['joint_move_speed_audit']
    assert audit['status'] == 'within_observed_limit'
    assert audit['actual_peak_radps'] == .05
    assert audit['physical_speed_verified'] is False
    assert audit['motion_time_stop_verified'] is False


async def test_arrived_prepare_with_missing_move_evidence_is_not_success(tmp_path, monkeypatch):
    from app.commissioning.smoke import _SmokeRecorder
    write = _SmokeRecorder.write_event

    async def missing(self, event, timestamp):
        if isinstance(event, dict) and event.get('snapshot_read_purpose') == 'joint_move':
            return
        await write(self, event, timestamp)

    monkeypatch.setattr(_SmokeRecorder, 'write_event', missing)
    with pytest.raises(RuntimeError, match='^smoke_joint_speed_incomplete$'):
        await run_smoke(options(tmp_path, monkeypatch),
                        AsyncMock(return_value=converging_client(0)))
    log = events(tmp_path)
    assert next(x for x in log if x['kind'] == 'joint_move_speed_audit')['valid_samples'] == 0
    assert not any(x['kind'] == 'smoke_result' for x in log)
