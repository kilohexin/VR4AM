from datetime import UTC,datetime,timedelta
import hashlib
import json
import pytest
from scripts.real_robot_blended_trial import Inputs,CONFIRM


@pytest.mark.parametrize('damage',[None,'expired','wrong_confirmation','changed_preview','changed_config','changed_source'])
def test_fresh_bound_approval_only(tmp_path,monkeypatch,damage):
    config=tmp_path/'config';config.write_text('synthetic')
    expiry=(datetime.now(UTC)+timedelta(minutes=5)).isoformat()
    row={'scope':'synthetic-blend','source':'old'}
    approved=tmp_path/'preview.json';approved.write_text(json.dumps(row))
    p=dict(config=config,input_hashes={str(config):hashlib.sha256(config.read_bytes()).hexdigest()},authorization_expires_utc=expiry)
    gate=Inputs(p,approved,hashlib.sha256(approved.read_bytes()).hexdigest(),expiry,CONFIRM)
    monkeypatch.setattr(gate,'expected_preview',lambda:row)
    if damage=='expired':gate.expires=(datetime.now(UTC)-timedelta(seconds=1)).isoformat()
    if damage=='wrong_confirmation':gate.confirm='old_scope'
    if damage=='changed_preview':approved.write_text('{}')
    if damage=='changed_config':config.write_text('changed')
    if damage=='changed_source':row['source']='changed'
    if damage:
        with pytest.raises(ValueError,match='scope_expired_or_changed'):gate.read()
    else:gate.read()
