import importlib.util
from pathlib import Path
import pytest

ROOT=Path(__file__).resolve().parents[3]


def test_demo_launcher_cannot_accept_real_target_or_bind_external():
    path=ROOT/'scripts/run_finite_position_demo.py'
    assert path.is_file(),'finite demo launcher missing'
    spec=importlib.util.spec_from_file_location('finite_delivery',path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    for args in (['--ip','172.16.2.162'],['--host','0.0.0.0']):
        with pytest.raises(SystemExit):m.parser().parse_args(args)
