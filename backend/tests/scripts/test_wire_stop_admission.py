"""Dedicated child's authorization boundary, synthetic factory only."""
from argparse import Namespace
from datetime import UTC, datetime, timedelta
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest
import yaml
from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def context(tmp_path,monkeypatch):
    path = ROOT/'scripts/real_robot_wire_stop.py'
    assert path.is_file(), 'dedicated admitted child missing'
    spec = importlib.util.spec_from_file_location('wire_child',path)
    child = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(child)
    output = ROOT/'.tmp' / ('admission-test-' + tmp_path.name)
    # Parent script production path guard uses ROOT; test changes that root
    # consistently rather than creating artifacts in the checkout.
    child.capture.ROOT = tmp_path
    output = tmp_path/'.tmp/once'
    output.mkdir(parents=True)
    data = yaml.safe_load(REAL_CONFIG_TEMPLATE.format(mode='control'))
    data['real_robot']['ip']='192.0.2.1'
    data['real_robot']['control'].update(child.capture.CAPS)
    config = output/'config-control.yaml'
    config.write_text(yaml.safe_dump(data),encoding='utf-8')
    baseline = dict(reviewed_idle=True,reviewed_empty_filters=True,idle_status_sha256='1'*64,empty_filters_sha256='2'*64)
    plan = child.capture.prepare(config,output,component_id=3,baseline=baseline)
    approved = output/'approved-preview.json'
    approved.write_text(json.dumps(child.capture.preview(plan)),encoding='utf-8')
    args = Namespace(config=config,output=output,component_id=3,tool_baseline_json=json.dumps(baseline),
         expected_config_sha256=plan['config_sha256'],approved_preview=approved,
         approved_preview_sha256=hashlib.sha256(approved.read_bytes()).hexdigest(),
         authorization_expires_utc=(datetime.now(UTC)+timedelta(minutes=10)).isoformat(),
         execute_once=True,confirm=child.capture.CONFIRM)
    receipt=dict(config_sha256=plan['config_sha256'],approved_preview_sha256=args.approved_preview_sha256,
                 component_id=3,filter_name='VR4ArmStop_000000000001')
    (output/'capture-ready.json').write_text(json.dumps(receipt),encoding='utf-8')
    class LocalCapture:
        admin=True
        def command(self,args):
            if args==['status']: return 'Log file: '+str(output/'capture.etl')
            if args==['filter','list']: return '1 VR4ArmStop_000000000001 TCP 192.0.2.1 3031'
            pytest.fail('unexpected local capture operation')
        def existing_rpc_connections(self): return 0
    monkeypatch.setattr(child.capture,'Native',LocalCapture)
    return child,args


@pytest.mark.parametrize('failure',['hash','expired','confirm','approval','mode'])
async def test_child_invalid_admission_never_enters_sdk_factory(context,failure):
    child,args = context
    if failure=='hash': args.expected_config_sha256='0'*64
    if failure=='expired': args.authorization_expires_utc='2000-01-01T00:00:00Z'
    if failure=='confirm': args.confirm='wrong'
    if failure=='approval': args.approved_preview_sha256='0'*64
    if failure=='mode': args.config.write_text('backend: lebai\nreal_robot:\n  mode: readonly\n')
    async def forbidden(_): pytest.fail('SDK factory must not run')
    with pytest.raises(ValueError):
        await child.run_once(args,client_factory=forbidden)
    assert not (args.output/'motion-admission.json').exists()


@pytest.mark.parametrize('problem', ['truncated_display', 'foreign_filter', 'bad_nonce', 'legacy_long_name'])
async def test_full_short_filter_identity_is_required_before_sdk_factory(context, monkeypatch, problem):
    child, args = context
    from app.commissioning import smoke
    local_type = child.capture.Native
    if problem in ('bad_nonce', 'legacy_long_name'):
        receipt = json.loads((args.output/'capture-ready.json').read_text(encoding='utf-8'))
        receipt['filter_name'] = ('VR4ArmStop_' + 'g'*12 if problem == 'bad_nonce'
                                  else 'VR4ArmMotionStop_000000000001')
        (args.output/'capture-ready.json').write_text(json.dumps(receipt), encoding='utf-8')
    else:
        class ChangedFilters(local_type):
            def command(self, values):
                text = super().command(values)
                if values == ['filter', 'list']:
                    if problem == 'truncated_display':
                        return '1 VR4ArmStop_000000... TCP 192.0.2.1 3031'
                    return text + '\n2 VR4ArmStop_000000000002 TCP 192.0.2.1 3031'
                return text
        monkeypatch.setattr(child.capture, 'Native', ChangedFilters)
    async def fake_workflow(options, client_factory, *, before_motion):
        await client_factory('192.0.2.1')
    monkeypatch.setattr(smoke, 'run_smoke', fake_workflow)
    async def forbidden(_):
        pytest.fail('incomplete or foreign filter identity must not enter SDK factory')
    with pytest.raises(ValueError):
        await child.run_once(args, client_factory=forbidden)
    assert not (args.output/'motion-admission.json').exists()


async def test_fixed_options_factory_rechecks_and_unique_admission_prevents_second_connection(context,monkeypatch):
    child,args = context
    from app.commissioning import smoke
    received,connections=[],[]
    async def factory(ip):
        connections.append(ip)
        return object()
    async def fake_workflow(options,client_factory,*,before_motion):
        received.append(options)
        await client_factory('192.0.2.1')
        before_motion()
    monkeypatch.setattr(smoke,'run_smoke',fake_workflow)
    assert await child.run_once(args,client_factory=factory)==0
    option,=received
    assert option.action.axis=='z' and option.action.distance_m==.003
    assert option.observe_stop_seconds==2 and option.early_stop_on_motion and option.stop_rpc_diagnostics
    assert len(connections)==1
    with pytest.raises(FileExistsError):
        await child.run_once(args,client_factory=factory)
    assert len(connections)==1


async def test_child_expiry_is_checked_again_at_actual_factory(context,monkeypatch):
    child,args=context
    from app.commissioning import smoke
    async def fake_workflow(options,client_factory,*,before_motion):
        args.authorization_expires_utc='2000-01-01T00:00:00Z'
        await client_factory('192.0.2.1')
    monkeypatch.setattr(smoke,'run_smoke',fake_workflow)
    async def forbidden(_): pytest.fail('expired actual connection')
    with pytest.raises(ValueError):
        await child.run_once(args,client_factory=forbidden)
    assert not (args.output/'motion-admission.json').exists()


async def test_child_expiry_after_connection_blocks_motion_guard(context,monkeypatch):
    child,args=context
    from app.commissioning import smoke
    async def factory(_): return object()
    async def fake_workflow(options,client_factory,*,before_motion):
        await client_factory('192.0.2.1')
        args.authorization_expires_utc='2000-01-01T00:00:00Z'
        before_motion()
        pytest.fail('must not admit motion')
    monkeypatch.setattr(smoke,'run_smoke',fake_workflow)
    with pytest.raises(ValueError):
        await child.run_once(args,client_factory=factory)
    assert (args.output/'motion-admission.json').exists()


async def test_real_child_workflow_expiry_during_setup_blocks_motion(context, monkeypatch):
    from app.commissioning import smoke
    from tests.robots.fake_lebai import FakeLebaiClient
    child, args = context
    client = FakeLebaiClient.idle()
    connections = []
    original = smoke.RobotControl.initialize_observed_gripper
    async def expiring_read(self):
        result = await original(self)
        args.authorization_expires_utc = '2000-01-01T00:00:00Z'
        return result
    monkeypatch.setattr(smoke.RobotControl, 'initialize_observed_gripper', expiring_read)
    async def factory(ip):
        connections.append(ip)
        return client
    with pytest.raises(child.capture._base.AuthorizationExpired):
        await child.run_once(args, client_factory=factory)
    assert connections == ['192.0.2.1']
    assert all(call[0] == 'stop_move' for call in client.write_calls)
    assert (args.output/'motion-admission.json').exists()


@pytest.mark.parametrize('problem',['missing_receipt','receipt_hash','foreign_capture'])
async def test_child_capture_not_owned_prevents_sdk_connection(context,monkeypatch,problem):
    child,args=context
    from app.commissioning import smoke
    if problem=='missing_receipt': (args.output/'capture-ready.json').unlink()
    if problem=='receipt_hash':
        receipt=json.loads((args.output/'capture-ready.json').read_text())
        receipt['approved_preview_sha256']='0'*64
        (args.output/'capture-ready.json').write_text(json.dumps(receipt))
    if problem=='foreign_capture':
        class Foreign:
            admin=True
            def command(self,args): return 'Log file: C:/foreign/capture.etl'
            def existing_rpc_connections(self): return 0
        monkeypatch.setattr(child.capture,'Native',Foreign)
    async def fake_workflow(options,client_factory,*,before_motion):
        await client_factory('192.0.2.1')
    monkeypatch.setattr(smoke,'run_smoke',fake_workflow)
    async def forbidden(_): pytest.fail('unowned capture must not connect')
    with pytest.raises((ValueError,FileNotFoundError)):
        await child.run_once(args,client_factory=forbidden)
    assert not (args.output/'motion-admission.json').exists()
