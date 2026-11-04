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
    parser.add_argument('--isolated-test-container',help='Explicit independent Docker test slot (loopback 13030/13020 only)')
    parser.add_argument('--test-seconds',type=int,help='Isolated test server lifetime, 30..600 seconds (default 120)')
    parser.add_argument('--port',type=int,default=8773)
    parser.add_argument('--blended-translation',action='store_true',help='Opt-in experimental bounded Cartesian translation')
    parser.add_argument('--pvat-vr',action='store_true',help='Experimental Docker-only XR PVAT; six base directions, bounded gestures')
    parser.add_argument('--pose-follow-vr',action='store_true',help='Opt-in Docker clutch-relative full-pose PVAT candidate')
    parser.add_argument('--blended-rotation',action='store_true',help='Opt-in combined translation/pitch/spin continuation; implies blended translation')
    parser.add_argument('--host',choices=('127.0.0.1','0.0.0.0'),default='127.0.0.1')
    parser.add_argument('--cert');parser.add_argument('--key')
    args=parser.parse_args(argv)
    if args.isolated_test_container is not None:
        if not args.isolated_test_container:parser.error('test container must be explicit and nonempty')
        if args.test_seconds is None:args.test_seconds=120
    elif args.test_seconds is not None:parser.error('test lifetime requires an isolated test container')
    if args.pose_follow_vr:args.pvat_vr=True
    if args.pvat_vr:args.blended_rotation=True
    if args.blended_rotation:args.blended_translation=True
    if not 1024<=args.port<=65535:parser.error('port must be 1024..65535')
    if bool(args.cert)!=bool(args.key):parser.error('cert and key are required together')
    if args.host!='127.0.0.1' and not args.cert:parser.error('LAN WebXR requires cert and key')
    if args.cert and not all(Path(p).is_file() for p in (args.cert,args.key)):parser.error('certificate files missing')
    if args.isolated_test_container and (args.host!='127.0.0.1' or args.port==8773 or not 30<=args.test_seconds<=600):
        parser.error('isolated tests require loopback, a separate web port and 30..600 seconds')
    admission=(runtime.verify_local_docker(args.isolated_test_container,isolated_test=True)
        if args.isolated_test_container else runtime.verify_local_docker())
    print(json.dumps(dict(execute=args.run,runtime='LEBAI_DOCKER',sdk_target='127.0.0.1:'+str(admission['port']),
        isolated_test=bool(args.isolated_test_container),test_seconds=args.test_seconds if args.isolated_test_container else None,
        simulator=True,web_host=args.host,web_port=args.port,admission=admission,pvat_vr=args.pvat_vr,
        pose_follow_vr=args.pose_follow_vr,
        pose_follow_window_s=runtime.POSE_PROFILE.duration_ms/1000 if args.pose_follow_vr else None,
        pose_follow_grip_span_mm=runtime.POSE_PROFILE.target_tcp_span_m*1000 if args.pose_follow_vr else None,
        pose_mailbox_ttl_ms=runtime.POSE_PROFILE.mailbox_ttl_ms if args.pose_follow_vr else None,
        pose_reference_speed_radps=runtime.POSE_PROFILE.reference_speed_radps if args.pose_follow_vr else None,
        pose_translation_scale=runtime.POSE_PROFILE.translation_scale if args.pose_follow_vr else None,
        startup='STOP/ROBOT_ON -> start_sys -> IDLE; otherwise require IDLE',
        controls='unlock,prepare,translation,pitch/spin,progressive claw',startup_velocity_factor_percent=100,
        blended_translation=args.blended_translation,translation_step_mm=4 if args.blended_translation else 8,
        blended_rotation=args.blended_rotation,
        rotation_blend_step_deg=1. if args.blended_rotation else None,
        gripper_provider='local_virtual',gripper_physics=False,hardware_verified=False)),flush=True)
    if not args.run:return 0
    import uvicorn
    from app.commissioning.qpc_event_loop import new_qpc_loop
    listener=socket.socket();listener.bind((args.host,args.port));listener.listen(128)
    async def serve():
        app=runtime.create_docker_app(ROOT/'web/dist',blended_translation=args.blended_translation,blended_rotation=args.blended_rotation,pvat_vr=args.pvat_vr,pose_follow_vr=args.pose_follow_vr,
            docker_test_container=admission['container_id'] if args.isolated_test_container else None)
        server=uvicorn.Server(uvicorn.Config(app,host=args.host,port=args.port,log_level='info',
            ssl_certfile=args.cert,ssl_keyfile=args.key))
        async def retire():
            await asyncio.sleep(args.test_seconds)
            server.should_exit=True  # Normal lifespan cleanup, never a process kill.
        timer=asyncio.create_task(retire()) if args.isolated_test_container else None
        try:await server.serve(sockets=[listener])
        finally:
            if timer:
                timer.cancel()
                try:await timer
                except asyncio.CancelledError:pass
    try:
        with asyncio.Runner(loop_factory=new_qpc_loop) as runner:runner.run(serve())
    finally:listener.close()
    return 0


if __name__=='__main__':raise SystemExit(main())
