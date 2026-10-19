"""The real finite CLI chain refuses impossible parameters before any SDK use."""
from dataclasses import replace
import json
import sys
from unittest.mock import AsyncMock

import pytest
import yaml

from tests.scripts import test_finite_window_entry as legacy
from tests.scripts import test_finite_pause_entry as pause


@pytest.fixture
def context(tmp_path, monkeypatch):
    return legacy.context.__wrapped__(tmp_path, monkeypatch)


def infeasible_config(plan):
    data = yaml.safe_load(plan['config'].read_text())
    data['real_robot']['control'].update(max_tcp_step_m=.00005,
        max_tcp_speed_mps=.005, max_tcp_acceleration_mps2=.02)
    plan['config'].write_text(yaml.safe_dump(data))
    return plan['config'].read_bytes()


def test_prepare_rejects_frozen_50um_80ms_scale_without_changing_configuration(context):
    entry, plan, _, _ = context()
    saved = infeasible_config(plan)
    with pytest.raises(ValueError, match='finite_reference_rest_to_rest_infeasible'):
        entry.prepare(plan['config'], plan['output'])
    assert plan['config'].read_bytes() == saved
    assert not plan['output'].exists() and 'lebai_sdk' not in sys.modules


def test_default_cli_refuses_impossible_parameters_before_execution(context, monkeypatch):
    entry, plan, _, _ = context()
    infeasible_config(plan)
    monkeypatch.setattr(entry, 'execute', AsyncMock(side_effect=AssertionError('execution is forbidden')))
    with pytest.raises(ValueError, match='finite_reference_rest_to_rest_infeasible'):
        entry.main(['--config', str(plan['config']), '--output', str(plan['output'])])
    assert not plan['output'].exists() and 'lebai_sdk' not in sys.modules


def test_preview_binds_necessary_only_assessment(context):
    entry, plan, _, _ = context()
    value = entry.preview(plan)
    assert value['reference_parameter_check']['necessary_conditions_satisfied']
    assert not value['reference_parameter_check']['trajectory_verified']
    assert not value['reference_parameter_check']['hardware_verified']


async def test_modified_in_memory_plan_cannot_bypass_check_before_factory(context):
    entry, plan, kw, _ = context()
    plan['settings'] = replace(plan['settings'], control=replace(plan['settings'].control,
        max_tcp_step_m=.00005, max_tcp_speed_mps=.005, max_tcp_acceleration_mps2=.02))
    factory = AsyncMock(side_effect=AssertionError('impossible parameters reached transport'))
    with pytest.raises(ValueError, match='finite_reference_rest_to_rest_infeasible'):
        await entry.execute(plan, **kw, client_factory=factory)
    assert not plan['output'].exists() and 'lebai_sdk' not in sys.modules


@pytest.mark.parametrize('filename', ['real_robot_finite_pause.py',
    'real_robot_finite_pause_sampling.py', 'real_robot_finite_pause_parallel.py'])
def test_pause_entry_chain_cannot_bypass_parameter_check(tmp_path, monkeypatch, filename):
    monkeypatch.setattr(pause, 'SOURCE', legacy.SOURCE.with_name(filename))
    entry, plan, _, _ = pause.context.__wrapped__(tmp_path, monkeypatch)()
    saved = infeasible_config(plan)
    with pytest.raises(ValueError, match='finite_reference_rest_to_rest_infeasible'):
        entry.prepare(plan['config'], plan['output'])
    assert plan['config'].read_bytes() == saved and not plan['output'].exists()


async def test_old_preview_without_new_assessment_cannot_authorize_connection(context):
    entry, plan, kw, _ = context()
    value = json.loads(kw['approved'].read_text())
    # Old approval is otherwise valid and correctly hashed.
    value.pop('reference_parameter_check', None)
    kw['approved'].write_text(json.dumps(value))
    kw['expected_hash'] = entry.sha(kw['approved'])
    factory = AsyncMock(side_effect=AssertionError('old scope reached transport'))
    with pytest.raises(ValueError, match='finite_scope_preview_or_hash_changed'):
        await entry.execute(plan, **kw, client_factory=factory)
    assert not plan['output'].exists()
