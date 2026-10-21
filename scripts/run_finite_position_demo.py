"""Run the explicit offline-only web candidate; no target or host overrides."""
import argparse
import asyncio
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--web-directory',type=Path,default=ROOT/'web/dist')
    return p


def main():
    args=parser().parse_args()
    import uvicorn
    from app.digital_twin.finite_position_demo import create_finite_position_demo_app
    from app.digital_twin.offline_timing import offline_timer_period
    from app.commissioning.qpc_event_loop import new_qpc_loop
    app=create_finite_position_demo_app(args.web_directory)
    server=uvicorn.Server(uvicorn.Config(app,host='127.0.0.1',port=8769,log_level='info'))
    with offline_timer_period():
        with asyncio.Runner(loop_factory=new_qpc_loop) as runner:runner.run(server.serve())


if __name__=='__main__':main()
