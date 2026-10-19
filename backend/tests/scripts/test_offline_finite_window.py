"""End-to-end rehearsal only; no native SDK or physical kinematics."""
import importlib.util
import asyncio
import json
from pathlib import Path
import sys

import pytest
import yaml

from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE

SOURCE = Path(__file__).resolve().parents[3] / 'scripts/offline_finite_window.py'


@pytest.fixture
def context(tmp_path, monkeypatch):
    def build():
        assert SOURCE.exists(), 'offline finite-window entry is not implemented'
        spec = importlib.util.spec_from_file_location('offline_finite_window', SOURCE)
        entry = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(entry)
        monkeypatch.setattr(entry.idle, 'ROOT', tmp_path)
        monkeypatch.setattr(entry.idle, 'runtime_hashes', lambda: {})
        data = yaml.safe_load(REAL_CONFIG_TEMPLATE.format(mode='control'))
        data['real_robot']['control']['max_joint_speed_radps'] = .05
        data['real_robot']['control']['max_joint_acceleration_radps2'] = .2
        config = tmp_path / 'control.yaml'
        config.write_text(yaml.safe_dump(data))
        fixture = tmp_path / 'fixture.json'
        fixture.write_text(json.dumps(dict(initial=dict(state='PAUSED',
            q=[0., -1., 1., 0., 1.57, 0.], qd=[0.]*6, qdd=[0.]*6,
            tcp=dict(p=[.3, 0., .4], q=[0., 0., 0., 1.]),
            tcp_setting=dict(p=[0., 0., .175], q=[0., 0., 0., 1.])))))
        return entry, config, fixture, tmp_path / '.tmp/rehearsal'
    return build


async def test_whole_workflow_uses_one_complete_window_and_one_owned_stop(context):
    entry, config, fixture, output = context()
    result = await entry.run(config, fixture, output)
    assert result['complete'] and result['runtime'] == 'SIMULATION'
    assert result['simulation_only'] and not result['hardware_verified'] and not result['motion_authorized']
    assert result['ik_entries'] == 1 and result['fk_entries'] == 4
    assert [c[0] for c in result['synthetic_writes']] == ['move_pvat', 'stop_move']
    point = result['synthetic_writes'][0]
    assert point[2] == point[3] == [0.]*6 and point[4] == .08
    assert 0 < point[1][0] <= .0002 and result['tail_complete'] and result['stop_confirmed']
    assert result['source_state'] == 'PAUSED' and result['controller_state_is_synthetic']
    assert json.loads((output/'result.json').read_text()) == result
    assert 'lebai_sdk' not in sys.modules


@pytest.mark.parametrize('scenario', ['fk_mismatch', 'fk_timeout', 'ik_timeout', 'expired_lease'])
async def test_failed_math_or_expired_lease_cannot_send_window(context, scenario):
    entry, config, fixture, output = context()
    result = await entry.run(config, fixture, output, scenario=scenario)
    assert not result['complete'] and result['pvat_entries'] == 0
    assert result['ik_entries'] == 1
    assert result['fk_entries'] == (1 if scenario.startswith('fk_') else 0)
    assert result['stop_entries'] <= 1 and not result['motion_authorized']
    assert result['assessment'] == 'simulation_only_not_live_acceptance'
    if scenario == 'expired_lease':
        # A timeout injection must expire the lease, not happen to pass through
        # an early asyncio timer or fail for a different mathematical reason.
        assert result['command_error'] == 'finite_diagnostic_expired'


@pytest.mark.parametrize('scenario,latch', [('stop_late', 'stop_unverified'), ('stop_moving', 'stop_incomplete')])
async def test_failed_stop_is_not_hidden_by_tail_or_disconnect(context, scenario, latch):
    entry, config, fixture, output = context()
    result = await entry.run(config, fixture, output, scenario=scenario)
    assert not result['complete'] and not result['stop_confirmed']
    assert result['latched_fault'] == latch and result['pvat_entries'] == result['stop_entries'] == 1
    assert [c[0] for c in result['synthetic_writes']] == ['move_pvat', 'stop_move']


@pytest.mark.parametrize('field,value', [('q', [0.]*5), ('qd', [False]*6), ('qdd', [float('nan')]*6), ('tcp', None)])
async def test_invalid_frozen_feedback_is_rejected_before_output(context, field, value):
    entry, config, fixture, output = context()
    data = json.loads(fixture.read_text()); data['initial'][field] = value
    fixture.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        await entry.run(config, fixture, output)
    assert not output.exists()


async def test_duplicate_output_cannot_run_another_rehearsal(context):
    entry, config, fixture, output = context()
    first = await entry.run(config, fixture, output)
    saved = (output/'result.json').read_bytes()
    with pytest.raises(FileExistsError):
        await entry.run(config, fixture, output)
    assert (output/'result.json').read_bytes() == saved and first['pvat_entries'] == 1


def test_cli_exposes_no_live_execute_flag(context):
    entry, config, fixture, output = context()
    with pytest.raises(SystemExit) as error:
        entry.main(['--config', str(config), '--fixture', str(fixture), '--output', str(output), '--execute-once'])
    assert error.value.code == 2 and not output.exists()


async def test_cancel_after_pvat_records_fault_created_by_owned_cleanup(context, monkeypatch):
    entry, config, fixture, output = context()
    owner = [None]
    original = entry.SyntheticController.move_pvat
    async def cancel(self, *args):
        result = await original(self, *args)
        owner[0].cancel()
        return result
    monkeypatch.setattr(entry.SyntheticController, 'move_pvat', cancel)
    owner[0] = asyncio.create_task(entry.run(config, fixture, output, scenario='stop_late'))
    with pytest.raises(asyncio.CancelledError):
        await owner[0]
    result = json.loads((output/'result.json').read_text())
    assert result['interrupted'] and not result['complete']
    assert result['latched_fault'] == 'stop_unverified' and not result['stop_confirmed']
    assert result['pvat_entries'] == result['stop_entries'] == 1


@pytest.mark.parametrize('stage', ['start', 'disconnect', 'close'])
async def test_cancel_during_resource_lifecycle_still_saves_incomplete_result(context, monkeypatch, stage):
    entry, config, fixture, output = context()
    cls = entry.FiniteWindowDiagnosticAdapter if stage=='disconnect' else entry.CommissioningRecorder
    original = getattr(cls, stage)
    async def cancel(self):
        await original(self)
        raise asyncio.CancelledError
    monkeypatch.setattr(cls, stage, cancel)
    with pytest.raises(asyncio.CancelledError):
        await entry.run(config, fixture, output)
    result = json.loads((output/'result.json').read_text())
    assert result['interrupted'] and not result['complete'] and not result['hardware_verified']
    assert result['pvat_entries'] == result['stop_entries'] == (0 if stage=='start' else 1)
