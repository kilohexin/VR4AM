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


@pytest.mark.parametrize('failure',[False,True])
def test_demo_balances_existing_process_timer_request_even_on_server_failure(monkeypatch,failure):
    import sys
    from contextlib import contextmanager
    from app.digital_twin import finite_position_demo,offline_timing
    import uvicorn
    path=ROOT/'scripts/run_finite_position_demo.py'
    spec=importlib.util.spec_from_file_location('finite_timer_delivery',path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);events=[]
    @contextmanager
    def timer():
        events.append('timer_on')
        try:yield
        finally:events.append('timer_off')
    class Server:
        def __init__(self,config):pass
        async def serve(self):
            assert events==['timer_on'];events.append('serve')
            if failure:raise ValueError('test_server_failure')
    monkeypatch.setattr(offline_timing,'offline_timer_period',timer)
    monkeypatch.setattr(finite_position_demo,'create_finite_position_demo_app',lambda path:object())
    monkeypatch.setattr(uvicorn,'Server',Server);monkeypatch.setattr(sys,'argv',['run_finite_position_demo.py'])
    if failure:
        with pytest.raises(ValueError,match='test_server_failure'):m.main()
    else:m.main()
    assert events==['timer_on','serve','timer_off']
