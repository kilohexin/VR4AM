"""Dedicated motion child. Default is a scope preview; never a generic RPC CLI."""
import argparse
import asyncio
from datetime import UTC, datetime
import importlib.util
import json
from pathlib import Path
import re

_spec = importlib.util.spec_from_file_location('motion_capture_guard', Path(__file__).with_name('motion_stop_wire_capture.py'))
capture = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(capture)


def checked_scope(args):
    if args.confirm != capture.CONFIRM:
        raise ValueError('motion_scope_confirmation_required')
    output = Path(args.output).resolve(strict=True)
    if (Path(args.config).resolve() != output/'config-control.yaml'
            or Path(args.approved_preview).resolve() != output/'approved-preview.json'):
        raise ValueError('motion_child_paths_not_bound')
    baseline = json.loads(args.tool_baseline_json)
    plan = capture.prepare(args.config,output,component_id=args.component_id,baseline=baseline)
    if plan['config_sha256'] != args.expected_config_sha256:
        raise ValueError('approved_config_hash_mismatch')
    capture.validate_approval(plan,args.approved_preview,args.approved_preview_sha256,args.authorization_expires_utc)
    return plan


def checked_capture(args, plan, *, before_connection=False):
    receipt = json.loads((plan['output']/'capture-ready.json').read_text(encoding='utf-8'))
    if (not isinstance(receipt,dict) or receipt.get('config_sha256') != plan['config_sha256']
            or receipt.get('approved_preview_sha256') != args.approved_preview_sha256
            or type(receipt.get('component_id')) is not int or receipt['component_id'] != plan['component_id']
            or type(receipt.get('filter_name')) is not str
            or re.fullmatch('VR4ArmMotionStop_[a-f0-9]{12}',receipt['filter_name']) is None):
        raise ValueError('capture_ready_receipt_not_bound')
    native = capture.Native()
    if not native.admin:
        raise ValueError('administrator_required')
    if (not capture.owns_capture(native.command(['status']),plan['output'])
            or not capture.owns_filters(native.command(['filter','list']),plan,receipt['filter_name'])):
        raise ValueError('capture_not_exclusively_owned')
    if before_connection and native.existing_rpc_connections() != 0:
        raise ValueError('existing_rpc_connections')
    checked_scope(args)  # Recheck expiry after the local tool queries.


async def run_once(args, client_factory=None):
    if args.execute_once is not True:
        raise ValueError('execute_once_required')
    plan = checked_scope(args)
    # Delayed app imports: invalid admission never reaches SDK factory setup.
    from app.commissioning import smoke
    from app.commissioning.actions import SmokeOptions, TranslationAction
    from app.config import REAL_ROBOT_CONFIRMATION
    from app.robots.lebai_sdk_bridge import connect_real_client
    factory = client_factory or connect_real_client

    async def admitted_connect(ip):
        current = checked_scope(args)  # actual SDK entry, not just parent launch
        if current['ip'] != ip:
            raise ValueError('sdk_endpoint_not_approved')
        checked_capture(args,current,before_connection=True)
        # Reserve once before SDK connection; a failed episode cannot be retried.
        with (plan['output']/'motion-admission.json').open('x',encoding='utf-8') as file:
            json.dump(dict(mode='single_guarded_motion_stop_wire',action=dict(capture.ACTION),
                           started_utc=datetime.now(UTC).isoformat(),config_sha256=current['config_sha256'],
                           approved_preview_sha256=args.approved_preview_sha256,
                           connection_attempted=True,hardware_stop_verified=False),file,indent=2)
            file.write('\n')
        return await factory(ip)

    def admit_motion():
        current = checked_scope(args)
        checked_capture(args,current)

    old_root = smoke.COMMISSIONING_LOG_ROOT
    smoke.COMMISSIONING_LOG_ROOT = plan['output']/'sessions'
    try:
        await smoke.run_smoke(SmokeOptions(args.config,TranslationAction('z',.003),REAL_ROBOT_CONFIRMATION,
                                          observe_stop_seconds=2.,stop_rpc_diagnostics=True,early_stop_on_motion=True),
                              admitted_connect,before_motion=admit_motion)
    finally:
        smoke.COMMISSIONING_LOG_ROOT = old_root
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--component-id',type=int,required=True)
    parser.add_argument('--tool-baseline-json',required=True)
    parser.add_argument('--expected-config-sha256',required=True)
    parser.add_argument('--approved-preview',type=Path,required=True)
    parser.add_argument('--approved-preview-sha256',required=True)
    parser.add_argument('--authorization-expires-utc',required=True)
    parser.add_argument('--execute-once',action='store_true')
    parser.add_argument('--confirm',required=True)
    args = parser.parse_args(argv)
    if not args.execute_once:
        print(json.dumps(capture.preview(checked_scope(args)),ensure_ascii=False,indent=2))
        return 0
    return asyncio.run(run_once(args))


if __name__ == '__main__':
    raise SystemExit(main())
