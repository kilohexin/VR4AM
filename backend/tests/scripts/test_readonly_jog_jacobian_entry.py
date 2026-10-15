"""Synthetic admission profile tests; no SDK import or robot connections."""
import json
from unittest.mock import AsyncMock

import pytest

from tests.scripts.test_readonly_kinematics_entry import context
from tests.commissioning.test_readonly_jog_jacobian import JacobianClient


def jog_context(context):
    entry,old,kwargs=context
    plan=entry.prepare(old['config'],old['output'],expires=old['authorization_expires_utc'],survey='jog-jacobian')
    kwargs['approved'].write_text(json.dumps(entry.preview(plan)))
    kwargs.update(expected_hash=entry.sha(kwargs['approved']),confirm=entry.CONFIRM_JOG)
    return entry,plan,kwargs


def test_profile_preview_has_exact_readonly_budget(context,monkeypatch,capsys):
    entry,old,_=context
    monkeypatch.setattr(entry,'execute',AsyncMock(side_effect=AssertionError('preview only')))
    assert entry.main(['--config',str(old['config']),'--output',str(old['output']),'--survey','jog-jacobian'])==0
    value=json.loads(capsys.readouterr().out)
    assert value['max_total_calls_including_connect']==29
    assert value['max_motion_state_calls']==2
    assert value['running_motion_policy']=='zero_or_positive_u32_with_explicit_FINISHED'
    assert 'get_motion_state' in value['allowed_queries']
    assert value['max_fk_calls']==13 and value['max_ik_calls']==0
    assert value['joint_perturbation_rad']==.001
    assert value['allowed_writes']==[] and 'get_velocity_factor' in value['allowed_queries']
    assert 'kinematics_inverse' not in value['allowed_queries']
    assert not old['output'].exists()


async def test_prior_zero_only_preview_cannot_authorize_expanded_reads(context):
    entry,plan,kwargs=jog_context(context);factory=AsyncMock()
    old_preview=entry.preview(plan)
    old_preview.update(max_total_calls_including_connect=27,running_motion_required=0)
    old_preview.pop('running_motion_policy',None);old_preview.pop('max_motion_state_calls',None)
    old_preview['allowed_queries']=[v for v in old_preview['allowed_queries'] if v!='get_motion_state']
    kwargs['approved'].write_text(json.dumps(old_preview))
    kwargs['expected_hash']=entry.sha(kwargs['approved'])
    with pytest.raises(ValueError,match='kinematics_scope_or_hash_changed'):
        await entry.execute(plan,**kwargs,client_factory=factory)
    assert not plan['output'].exists()
    factory.assert_not_called()


async def test_old_confirmation_cannot_authorize_jog_profile(context):
    entry,plan,kwargs=jog_context(context);factory=AsyncMock()
    with pytest.raises(ValueError):
        await entry.execute(plan,**{**kwargs,'confirm':entry.CONFIRM},client_factory=factory)
    factory.assert_not_called()
    assert not plan['output'].exists()


async def test_jog_profile_injected_execution_is_one_attempt_with_saved_report(context):
    entry,plan,kwargs=jog_context(context);client=JacobianClient();factory=AsyncMock(return_value=client)
    r=await entry.execute(plan,**kwargs,client_factory=factory)
    assert r['complete'] and r['jacobian_collected'] and not r['continuous_control_admitted']
    assert len(r['calls'])==27 and len(client.fk_inputs)==13 and client.write_calls==[]
    assert json.loads((plan['output']/'result.json').read_text())==r
    with pytest.raises(FileExistsError):await entry.execute(plan,**kwargs,client_factory=factory)
    assert factory.await_count==1


async def test_profile_change_invalidates_approved_preview(context):
    entry,plan,kwargs=jog_context(context);plan['survey']='fk-ik';factory=AsyncMock()
    with pytest.raises(ValueError):await entry.execute(plan,**kwargs,client_factory=factory)
    factory.assert_not_called()


async def test_wrong_sdk_identity_rejected_before_native_import(context,monkeypatch):
    entry,plan,kwargs=jog_context(context)
    monkeypatch.setattr(entry,'verify_sdk_source',lambda:None)
    def forbidden(name):pytest.fail('must check pinned binary before native import')
    monkeypatch.setattr(entry.importlib,'import_module',forbidden)
    r=await entry.execute(plan,**kwargs)
    assert not r['complete'] and not r['native_sdk_init_entered'] and not r['native_sdk_connect_entered']
    assert r['error']=='jog_survey_sdk_binary_mismatch'


@pytest.mark.parametrize('change',['expired','config','runtime','rpc'])
async def test_new_profile_cannot_bypass_original_admission(context,monkeypatch,change):
    entry,plan,kwargs=jog_context(context);factory=AsyncMock()
    if change=='expired':kwargs['expires']='2000-01-01T00:00:00Z'
    elif change=='config':plan['config'].write_text(plan['config'].read_text()+'\n# change')
    elif change=='runtime':monkeypatch.setattr(entry,'runtime_hashes',lambda:{'synthetic':'b'*64})
    else:monkeypatch.setattr(entry,'existing_rpc_connections',lambda:1)
    with pytest.raises(ValueError):await entry.execute(plan,**kwargs,client_factory=factory)
    factory.assert_not_called()
    assert not plan['output'].exists()


@pytest.mark.parametrize('wrong_loaded',[True,False])
async def test_native_loaded_identity_checked_before_init_and_fake_path_works(context,monkeypatch,wrong_loaded):
    from types import SimpleNamespace
    entry,plan,kwargs=jog_context(context)
    package=entry.ROOT/'.venv/Lib/site-packages/lebai_sdk'
    monkeypatch.setattr(entry,'runtime_hashes',lambda:{'.venv/Lib/site-packages/lebai_sdk/lebai_sdk.pyd':entry.SDK_BINARY_SHA256})
    plan['runtime_hashes']=entry.runtime_hashes()
    kwargs['approved'].write_text(json.dumps(entry.preview(plan)));kwargs['expected_hash']=entry.sha(kwargs['approved'])
    original_sha=entry.sha
    monkeypatch.setattr(entry,'sha',lambda p:entry.SDK_BINARY_SHA256 if str(p).endswith('lebai_sdk.pyd') else original_sha(p))
    monkeypatch.setattr(entry,'verify_sdk_source',lambda:None)
    initialized=[]
    native=SimpleNamespace(__file__=str(package/'lebai_sdk.pyd'))
    client=JacobianClient()
    module=SimpleNamespace(__file__=str(package/'__init__.py'),init=lambda:initialized.append(True),connect=AsyncMock(return_value=client))
    if wrong_loaded:native.__file__=str(package/'wrong.pyd')
    monkeypatch.setitem(entry.sys.modules,'lebai_sdk.lebai_sdk',native)
    monkeypatch.setattr(entry,'importlib',SimpleNamespace(import_module=lambda _:module))
    r=await entry.execute(plan,**kwargs)
    if wrong_loaded:
        assert not r['complete'] and not initialized
        assert r['error']=='jog_survey_loaded_sdk_mismatch'
        module.connect.assert_not_called()
    else:
        assert r['complete'] and initialized==[True]
        assert r['native_sdk_init_entered'] and r['native_sdk_connect_entered']
        assert module.connect.await_count==1
