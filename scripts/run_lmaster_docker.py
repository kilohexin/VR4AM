"""Preview or run the official local Docker web/VR controller, never real hardware."""
import argparse
import asyncio
import json
from pathlib import Path
import socket
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))


def main(argv=None):
    from app.digital_twin import docker_runtime as runtime
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true',help='Start the virtual controller and web app')
    parser.add_argument('--port',type=int,default=8773)
    parser.add_argument('--host',choices=('127.0.0.1','0.0.0.0'),default='127.0.0.1')
    parser.add_argument('--cert');parser.add_argument('--key')
    args=parser.parse_args(argv)
    if not 1024<=args.port<=65535:parser.error('port must be 1024..65535')
    if bool(args.cert)!=bool(args.key):parser.error('cert and key are required together')
    if args.host!='127.0.0.1' and not args.cert:parser.error('LAN WebXR requires cert and key')
    if args.cert and not all(Path(p).is_file() for p in (args.cert,args.key)):parser.error('certificate files missing')
    admission=runtime.verify_local_docker()
    print(json.dumps(dict(execute=args.run,runtime='LEBAI_DOCKER',sdk_target='127.0.0.1:3030',
        simulator=True,web_host=args.host,web_port=args.port,admission=admission,
        startup='STOP -> start_sys -> IDLE; otherwise require IDLE',
        controls='unlock,prepare,2mm translation,0.5deg pitch,progressive claw',
        gripper_provider='local_virtual',gripper_physics=False,hardware_verified=False)),flush=True)
    if not args.run:return 0
    import uvicorn
    from app.commissioning.qpc_event_loop import new_qpc_loop
    listener=socket.socket();listener.bind((args.host,args.port));listener.listen(128)
    async def serve():
        app=runtime.create_docker_app(ROOT/'web/dist')
        server=uvicorn.Server(uvicorn.Config(app,host=args.host,port=args.port,log_level='info',
            ssl_certfile=args.cert,ssl_keyfile=args.key))
        await server.serve(sockets=[listener])
    try:
        with asyncio.Runner(loop_factory=new_qpc_loop) as runner:runner.run(serve())
    finally:listener.close()
    return 0


if __name__=='__main__':raise SystemExit(main())
