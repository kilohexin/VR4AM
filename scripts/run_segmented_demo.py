"""Build/check/serve an offline-only PC delivery demo; no live CLI options."""
import argparse
import asyncio
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path,PurePosixPath
import shutil
import subprocess
import sys
import uuid

ROOT=Path(__file__).resolve().parents[1]
PACKAGES=ROOT/'artifacts/delivery/segmented-demo'
PORT=8768
SINGLE_SOURCES=('config/fake-lebai.yaml','config/default.yaml','web/index.html','web/package.json',
    'config/lm3_visual_kinematics_v1.json','web/package-lock.json','schemas/teleop-v1.json',
    'scripts/run_segmented_demo.py','web/vite.config.ts','web/tsconfig.json')
SOURCE_FOLDERS=('backend/app','web/src','web/public')

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def sources(root):
    paths=set()
    for folder in SOURCE_FOLDERS:
        for path in (root/folder).rglob('*'):
            if (path.is_file() and '__pycache__' not in path.parts and path.suffix not in ('.pyc','.pyd')):
                if path.is_symlink() or not path.resolve().is_relative_to(root.resolve()):
                    raise ValueError('delivery_source_escape')
                paths.add(path)
    paths.update(root/name for name in SINGLE_SOURCES if (root/name).is_file())
    for path in paths:
        if path.is_symlink() or not path.resolve().is_relative_to(root.resolve()):
            raise ValueError('delivery_source_escape')
    return {p.relative_to(root).as_posix():sha(p) for p in sorted(paths)}

def assets(directory):
    result={}
    for path in directory.rglob('*'):
        if path==directory/'delivery-manifest.json':continue
        if path.is_file():
            if path.is_symlink() or not path.resolve().is_relative_to(directory.resolve()):
                raise ValueError('delivery_asset_escape')
            result[path.relative_to(directory).as_posix()]=sha(path)
    if 'index.html' not in result:raise ValueError('delivery_web_missing')
    return result

def write_manifest(root,directory):
    payload=dict(schema=1,runtime='LEBAI_FAKE',hardware_verified=False,
        target_strategy='segmented_experimental',sources=sources(root),assets=assets(directory))
    (directory/'delivery-manifest.json').write_text(json.dumps(payload,indent=2)+'\n',encoding='utf-8')
    return payload

def verify_manifest(root,directory):
    try:
        payload=json.loads((directory/'delivery-manifest.json').read_text(encoding='utf-8'))
        if (set(payload)!={'schema','runtime','hardware_verified','target_strategy','sources','assets'}
                or payload['schema']!=1 or payload['runtime']!='LEBAI_FAKE'
                or payload['hardware_verified'] is not False
                or payload['target_strategy']!='segmented_experimental'):
            raise ValueError('delivery_identity_invalid')
        for names in (payload['sources'],payload['assets']):
            if not isinstance(names,dict) or any(PurePosixPath(name).is_absolute()
                    or '..' in PurePosixPath(name).parts or '\\' in name or ':' in name for name in names):
                raise ValueError('delivery_path_invalid')
        if payload['sources']!=sources(root):raise ValueError('delivery_sources_changed')
        if payload['assets']!=assets(directory):raise ValueError('delivery_assets_changed')
        return payload
    except (OSError,KeyError,TypeError,json.JSONDecodeError) as error:
        raise ValueError('delivery_manifest_invalid') from error

def node_executable(selected=None):
    candidates=[Path(selected)] if selected else [
        Path.home()/'.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe',
        Path(shutil.which('node') or 'node')]
    for candidate in candidates:
        try:
            output=subprocess.check_output([str(candidate),'--version'],text=True).strip()
            if int(output.lstrip('v').split('.')[0])>=22:return str(candidate)
        except (OSError,ValueError,subprocess.SubprocessError):pass
    raise ValueError('delivery_node_22_required:use_--node')

def build_package(node):
    web=ROOT/'web'
    for name in ('typescript/bin/tsc','vite/bin/vite.js'):
        if not (web/'node_modules'/name).is_file():raise ValueError('delivery_web_dependencies_missing')
    directory=PACKAGES/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8])
    directory.mkdir(parents=True,exist_ok=False)
    environment=dict(os.environ)
    environment['VITE_TELEOP_WS_URL']=f'ws://127.0.0.1:{PORT}/ws/v1/teleop'
    subprocess.run([node,'node_modules/typescript/bin/tsc','--noEmit'],cwd=web,env=environment,check=True)
    subprocess.run([node,'node_modules/vite/bin/vite.js','build','--outDir',str(directory)],
        cwd=web,env=environment,check=True)
    write_manifest(ROOT,directory)
    verify_manifest(ROOT,directory)
    (PACKAGES/'latest.json').write_text(json.dumps({'directory':directory.name})+'\n',encoding='utf-8')
    return directory

def latest_package():
    try:
        name=json.loads((PACKAGES/'latest.json').read_text())['directory']
        if not isinstance(name,str) or Path(name).name!=name or name in ('.','..'):
            raise ValueError('delivery_latest_path_invalid')
        directory=PACKAGES/name
        if not directory.resolve().is_relative_to(PACKAGES.resolve()):raise ValueError('delivery_latest_path_invalid')
        return directory
    except (OSError,KeyError,TypeError,json.JSONDecodeError) as error:
        raise ValueError('delivery_package_missing:run_--build') from error

def parser():
    result=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    result.add_argument('--build',action='store_true',help='Build before serving')
    result.add_argument('--build-only',action='store_true',help='Build and exit')
    result.add_argument('--check',action='store_true',help='Verify current package without starting services')
    result.add_argument('--node',type=Path,help='Node.js 22+ executable for build')
    return result

def _run_offline_server(server):
    # Keep this scheduler local to the offline-only entry. Robot timestamps,
    # freshness limits and the process-wide event-loop policy stay unchanged.
    from app.commissioning.qpc_event_loop import new_qpc_loop
    from app.digital_twin.offline_timing import offline_timer_period
    # Hold through Runner cleanup so cancelled tasks and executor work finish
    # before releasing this process's timer request.
    with offline_timer_period():
        with asyncio.Runner(loop_factory=new_qpc_loop) as runner:
            runner.run(server.serve())


def main(argv=None):
    options=parser().parse_args(argv)
    sys.path.insert(0,str(ROOT/'backend'))
    try:
        from app.digital_twin.segmented_demo import validate_demo_environment
        validate_demo_environment(os.environ)
        if options.check and (options.build or options.build_only):raise ValueError('delivery_check_cannot_build')
        directory=build_package(node_executable(options.node)) if options.build or options.build_only else latest_package()
        verify_manifest(ROOT,directory)
        print(json.dumps(dict(runtime='LEBAI_FAKE',hardware_verified=False,
            target_strategy='segmented_experimental',package=str(directory)),ensure_ascii=False),flush=True)
        if options.check or options.build_only:return 0
        from app.digital_twin.segmented_demo import create_segmented_demo_app
        import uvicorn
        print(f'Offline demo: http://127.0.0.1:{PORT}/ ; Ctrl+C to stop',flush=True)
        server=uvicorn.Server(uvicorn.Config(create_segmented_demo_app(directory),
            host='127.0.0.1',port=PORT,log_level='info',lifespan='on'))
        try:
            _run_offline_server(server)
        except KeyboardInterrupt:
            # Uvicorn re-raises the signal after its application cleanup.
            # A forced/interrupted cleanup must still be reported as failure.
            pass
        if not server.started:raise ValueError('delivery_service_startup_failed')
        lifecycle=getattr(server,'lifespan',None)
        if (server.force_exit or lifecycle is None or lifecycle.shutdown_failed
                or lifecycle.error_occurred or not lifecycle.shutdown_event.is_set()):
            raise ValueError('delivery_service_shutdown_unconfirmed')
        print('Local fake demo closed; this is not hardware stop verification.',flush=True)
        return 0
    except (ValueError,OSError,RuntimeError,subprocess.SubprocessError) as error:
        print(f'Offline delivery refused: {error}',file=sys.stderr)
        return 2

if __name__=='__main__':raise SystemExit(main())
