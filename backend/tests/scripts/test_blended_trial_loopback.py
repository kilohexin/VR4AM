import pytest
from scripts.blended_trial_loopback import parser


@pytest.mark.parametrize('args',[['--ip','172.16.2.162'],['--host','0.0.0.0'],['--port','3030']])
def test_loopback_accepts_no_endpoint_override(args):
    with pytest.raises(SystemExit):parser().parse_args(['--output','synthetic',*args])
