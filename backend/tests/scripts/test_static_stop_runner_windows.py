"""Exercise the real PowerShell wrapper with a harmless child, no SDK import."""
import hashlib
import shutil
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
POWERSHELL = shutil.which('powershell')
pytestmark = pytest.mark.skipif(not POWERSHELL, reason='Windows PowerShell unavailable')


@pytest.mark.parametrize('mode,expected_probe', [('readonly', True), ('control', False)])
def test_wrapper_restores_bytes_and_checks_semantic_mode(tmp_path, mode, expected_probe):
    source = tmp_path / 'source'
    scripts = source / 'scripts'
    scripts.mkdir(parents=True)
    shutil.copy2(ROOT / 'scripts' / 'build_static_stop_control_config.py', scripts)
    # This substitute only inspects the switched config. It never imports or
    # contacts the SDK, and returns a deliberately nonzero child exit code.
    (scripts / 'real_robot_static_system_stop.py').write_text(
        "import argparse, pathlib, yaml\n"
        "p=argparse.ArgumentParser(); p.add_argument('--config'); p.add_argument('--output'); p.add_argument('--confirm'); a=p.parse_args()\n"
        "assert yaml.safe_load(pathlib.Path(a.config).read_bytes())['real_robot']['mode']=='control'\n"
        "pathlib.Path(a.output).mkdir(); print('MOCK_CHILD_ONLY'); raise SystemExit(2)\n",
        encoding='utf-8')
    config = tmp_path / 'config.yaml'
    other_mode = 'auto' if mode == 'readonly' else 'readonly'
    original = (f'backend: lebai\r\nreal_robot:\r\n  mode: {mode}\r\n'
                f'  ip: 192.0.2.1\r\nother:\r\n  mode: {other_mode}\r\n').encode()
    config.write_bytes(original)
    output = tmp_path / 'once'
    completed = subprocess.run(
        [POWERSHELL, '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File',
         str(ROOT / 'scripts' / 'run_static_system_stop.ps1'),
         '-Source', str(source), '-Python', sys.executable,
         '-Config', str(config), '-Output', str(output),
         '-ExpectedConfigSha256', hashlib.sha256(original).hexdigest(),
         '-Confirm', 'I_UNDERSTAND_SINGLE_SYSTEM_STOP_MAY_MOVE'],
        capture_output=True, text=True, timeout=20)
    assert config.read_bytes() == original
    assert (output / 'probe').exists() == expected_probe, (completed.stdout, completed.stderr)
    if expected_probe:
        assert completed.returncode == 2
        assert 'MOCK_CHILD_ONLY' in (output / 'stdout.txt').read_bytes().decode('utf-16')
        assert '"config_restored":  true' in (output / 'execution.json').read_bytes().decode('utf-8-sig')
    else:
        assert completed.returncode != 0
        assert not (output / 'execution.json').exists()
