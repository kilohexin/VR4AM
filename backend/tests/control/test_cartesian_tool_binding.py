from hashlib import sha256
import json

import pytest
from scipy.spatial.transform import Rotation

from app.control.cartesian_tool_binding import ZERO_TOOL,load_tool_report,tool_digest
from app.commissioning.native_jog_mdh import MdhTool


def saved(tmp_path,change=lambda x:x):
    report=dict(mode='readonly',complete=True,config_unchanged=True,reads=[dict(method='get_tcp',outcome='returned',
        raw=dict(x=.03,y=-.02,z=.07,rx=.2,ry=-.4,rz=.3))])
    change(report);raw=json.dumps(report).encode();path=tmp_path/'snapshot.json';path.write_bytes(raw)
    return path,sha256(raw).hexdigest()


def test_saved_tool_binds_translation_and_full_rotation_with_distinct_digest(tmp_path):
    path,digest=saved(tmp_path);tool=load_tool_report(path,digest)
    assert tool.translation_m==(.03,-.02,.07)
    assert (Rotation.from_quat(tool.rotation_xyzw).inv()*Rotation.from_euler('xyz',(.2,-.4,.3))).magnitude()<1e-12
    assert tool_digest(tool)!=tool_digest(ZERO_TOOL)
    different=MdhTool(tool.translation_m,tuple(float(v) for v in Rotation.from_euler('xyz',(.21,-.4,.3)).as_quat()))
    assert tool_digest(different)!=tool_digest(tool)
    # Later file mutation cannot silently change the immutable bound geometry.
    path.write_text('{}')
    with pytest.raises(ValueError,match='source_changed'):load_tool_report(path,digest)
    assert tool.translation_m==(.03,-.02,.07)


@pytest.mark.parametrize('change',[
    lambda r:r.update(complete=False),lambda r:r.update(mode='control'),
    lambda r:r.update(config_unchanged=False),lambda r:r.update(error_type='TimeoutError'),
    lambda r:r['reads'][0].update(outcome='failed'),lambda r:r['reads'].append(r['reads'][0].copy()),
    lambda r:r['reads'][0]['raw'].update(z=float('nan')),
    lambda r:r['reads'][0]['raw'].update(z='0.07'),lambda r:r['reads'][0]['raw'].update(z=True),
    lambda r:r['reads'][0]['raw'].update(z=1.),
])
def test_invalid_or_ambiguous_report_never_defaults_to_zero_tool(tmp_path,change):
    path,digest=saved(tmp_path,change)
    with pytest.raises(ValueError,match='report_rejected'):load_tool_report(path,digest)


@pytest.mark.parametrize('digest',[None,'','a'*63,'x'*64])
def test_missing_hash_rejected_without_reading_a_file(tmp_path,digest):
    with pytest.raises(ValueError,match='hash_required'):load_tool_report(tmp_path/'missing',digest)
