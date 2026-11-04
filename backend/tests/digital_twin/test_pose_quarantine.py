import sys
import pytest
from app.control.docker_pose_profile import POSE_PROFILE


def test_pose_launcher_rejects_execution_before_docker_inspection(monkeypatch):
    from app.digital_twin import docker_runtime
    from scripts.run_lmaster_docker import main
    assert POSE_PROFILE.quarantined
    monkeypatch.setattr(docker_runtime,'verify_local_docker',lambda *a,**kw:pytest.fail('no connection/inspection'))
    with pytest.raises(SystemExit) as caught:main(['--run','--pose-follow-vr'])
    assert caught.value.code==2


def test_pose_runtime_cannot_bypass_the_launcher(tmp_path):
    from app.digital_twin.docker_runtime import create_docker_app
    with pytest.raises(ValueError,match='pose_follow_quarantined_cartesian_path'):
        create_docker_app(tmp_path,pose_follow_vr=True)


@pytest.mark.parametrize('name',['docker_pose_continuation_probe','docker_pose_follow_probe'])
def test_pose_motion_probe_is_blocked_before_docker_inspection(monkeypatch,name):
    from importlib import import_module
    probe=import_module('scripts.'+name)
    monkeypatch.setattr(sys,'argv',['probe','--container','offline','--output','unused.json','--run'])
    monkeypatch.setattr(probe,'admission',lambda *a,**kw:pytest.fail('no inspection'))
    with pytest.raises(SystemExit) as caught:probe.main()
    assert caught.value.code==2
