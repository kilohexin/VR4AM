import importlib.util
from pathlib import Path
import pytest

SOURCE=Path(__file__).resolve().parents[3]/'scripts/finite_position_site_loopback.py'


def test_owned_single_position_sdk_loopback_exists():
    assert SOURCE.is_file(),'single-position SDK loopback missing'


def test_loopback_rejects_endpoint_overrides():
    assert SOURCE.is_file(),'single-position SDK loopback missing'
    spec=importlib.util.spec_from_file_location('single_position_loopback',SOURCE)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    for args in (['--ip','172.16.2.162'],['--host','0.0.0.0'],['--port','5866']):
        with pytest.raises(SystemExit):m.parser().parse_args(args)
