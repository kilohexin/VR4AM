"""One motion diagnostic orchestration, synthetic tools only; no SDK/capture."""
from datetime import UTC, datetime, timedelta
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def module():
    path = ROOT / 'scripts/motion_stop_wire_capture.py'
    assert path.is_file(), 'motion capture entry missing'
    spec = importlib.util.spec_from_file_location('motion_wire', path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


@pytest.fixture
def candidate(module, tmp_path, monkeypatch):
    monkeypatch.setattr(module, 'ROOT', tmp_path)
    (tmp_path / 'scripts').mkdir()
    (tmp_path / 'scripts/real_robot_wire_stop.py').write_text('# synthetic fixed child')
    config = tmp_path / 'control.yaml'
    config.write_text('backend: lebai\nreal_robot:\n  mode: control\n  ip: 192.0.2.1\n  control:\n'
                      '    max_tcp_speed_mps: 0.005\n    max_joint_speed_radps: 0.05\n'
                      '    max_tcp_acceleration_mps2: 0.02\n    max_joint_acceleration_radps2: 0.2\n'
                      '    max_tcp_step_m: 0.0005\n    max_tcp_rotation_radps: 0.05\n'
                      '    max_tcp_angular_acceleration_radps2: 0.1\n    max_tcp_rotation_step_deg: 0.2\n')
    baseline = dict(reviewed_idle=True, reviewed_empty_filters=True,
                    idle_status_sha256=module.digest_text('IDLE'), empty_filters_sha256=module.digest_text('EMPTY'))
    plan = module.prepare(config, tmp_path / '.tmp/one', component_id=3, baseline=baseline)
    approved = tmp_path / 'approved.json'
    approved.write_text(json.dumps(module.preview(plan)), encoding='utf-8')
    expires = (datetime.now(UTC) + timedelta(minutes=10)).isoformat()
    plan['approval'] = dict(path=approved, sha256=hashlib.sha256(approved.read_bytes()).hexdigest(), expires=expires)
    return plan


class FakeNative:
    admin = True
    def __init__(self, *, child_exit=0, child_active=False, filters='EMPTY', foreign=False, connections=0):
        self.calls, self.active = [], None
        self.child_exit, self.child_active, self.filters, self.foreign, self.connections = child_exit, child_active, filters, foreign, connections
    def existing_rpc_connections(self):
        return self.connections
    def command(self, args):
        self.calls.append(list(args))
        if args == ['status']:
            return 'Log file: ' + self.active if self.active else 'IDLE'
        if args == ['filter', 'list']:
            return self.filters
        if args[:2] == ['filter','add']:
            self.filters=f'1 {args[2]} TCP 192.0.2.1 3031'
        if args[0] == 'start':
            self.active = args[args.index('--file-name') + 1]
            Path(self.active).write_bytes(b'artificial ETL')
        if args[0] == 'stop':
            self.active = None
        if args[0] == 'etl2pcap':
            Path(args[args.index('--out') + 1]).write_bytes(b'artificial PCAP')
        return 'OK'
    def motion(self, args, stdout, stderr, budget_s):
        self.calls.append(['motion', *args])
        assert budget_s <= 30
        if self.foreign:
            self.active = str(Path(stdout).parent.parent / 'foreign/capture.etl')
        return dict(exit_code=self.child_exit, timed_out=self.child_active,
                    process_exited=not self.child_active, pid=7, force_terminated=False)


def execute(module, plan, native):
    approval = plan['approval']
    return module.execute(plan, native, confirm=module.CONFIRM, approved_preview=approval['path'],
                          approved_preview_sha256=approval['sha256'], authorization_expires_utc=approval['expires'])


def test_default_preview_has_fixed_scope_and_no_native_or_output(module, candidate, monkeypatch, capsys):
    def forbidden():
        pytest.fail('preview must not create Native')
    monkeypatch.setattr(module, 'Native', forbidden)
    assert module.main(['--config', str(candidate['config']), '--output', str(candidate['output']), '--component-id', '3']) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['action'] == {'axis': 'z', 'distance_m': .003, 'early_stop_on_motion': True, 'observe_stop_seconds': 2.0}
    assert report['capture_budget_ms'] == 30000 and report['hardware_stop_verified'] is False
    assert not candidate['output'].exists() and '192.0.2.1' not in json.dumps(report)


@pytest.mark.parametrize('pin', [None, '0'*64])
def test_execute_cli_requires_expected_config_hash_before_native(module, candidate, monkeypatch, pin):
    monkeypatch.setattr(module, 'Native', lambda: pytest.fail('unbound CLI must not construct Native'))
    approval = candidate['approval']
    args = ['--config', str(candidate['config']), '--output', str(candidate['output']), '--component-id', '3',
            '--execute', '--confirm', module.CONFIRM, '--approved-preview', str(approval['path']),
            '--approved-preview-sha256', approval['sha256'], '--authorization-expires-utc', approval['expires']]
    if pin is not None:
        args += ['--expected-config-sha256', pin]
    with pytest.raises(ValueError, match='approved_config_hash_mismatch'):
        module.main(args)
    assert not candidate['output'].exists()


def test_one_capture_and_fixed_motion_child_with_pinned_files_and_no_auto_filter_remove(module, candidate):
    native = FakeNative()
    report = execute(module, candidate, native)
    assert report['acquisition_complete'] and not report['complete'] and report['manual_filter_cleanup_required']
    assert report['motion_process']['process_exited'] and not report['hardware_stop_verified']
    assert sum(c[0] == 'start' for c in native.calls) == sum(c[0] == 'motion' for c in native.calls) == sum(c[0] == 'stop' for c in native.calls) == 1
    child = next(c for c in native.calls if c[0] == 'motion')
    assert Path(child[1]).name == 'real_robot_wire_stop.py'
    assert '--approved-preview' in child and '--authorization-expires-utc' in child
    assert '--expected-config-sha256' in child and '--execute-once' in child
    assert not any(c[:2] == ['filter','remove'] for c in native.calls)
    with pytest.raises(FileExistsError):
        execute(module, candidate, FakeNative())


@pytest.mark.parametrize('problem', ['confirm', 'admin', 'connections', 'expired', 'approval', 'config', 'runtime', 'output'])
def test_changed_or_unapproved_scope_rejects_before_capture_or_connection(module, candidate, problem):
    native = FakeNative()
    if problem == 'admin': native.admin = False
    if problem == 'connections': native.connections = 1
    if problem == 'expired': candidate['approval']['expires'] = '2000-01-01T00:00:00Z'
    if problem == 'approval': candidate['approval']['sha256'] = '0' * 64
    if problem == 'config': candidate['config'].write_text('changed')
    if problem == 'runtime': (module.ROOT / 'scripts/real_robot_wire_stop.py').write_text('# changed')
    if problem == 'output': candidate['output'] = module.ROOT / '.tmp/another'
    with pytest.raises(ValueError):
        if problem == 'confirm':
            module.execute(candidate, native, confirm='wrong', approved_preview=candidate['approval']['path'],
                           approved_preview_sha256=candidate['approval']['sha256'], authorization_expires_utc=candidate['approval']['expires'])
        else:
            execute(module, candidate, native)
    assert not native.calls


def test_nonempty_filters_block_all_capture_and_child_actions(module, candidate):
    native = FakeNative(filters='OLD_FILTER')
    report = execute(module, candidate, native)
    assert not report['capture_started'] and not report['acquisition_complete']
    assert not any(c[0] in ('start', 'motion', 'stop') for c in native.calls)


@pytest.mark.parametrize('active,exit_code', [(True,None), (False,2)])
def test_child_failed_or_still_active_never_counts_as_motion_stop_proof(module, candidate, active, exit_code):
    native = FakeNative(child_active=active, child_exit=exit_code)
    report = execute(module, candidate, native)
    assert not report['acquisition_complete'] and not report['hardware_stop_verified']
    assert report['manual_process_cleanup_required'] is active
    assert sum(c[0] == 'motion' for c in native.calls) == 1
    assert not report['motion_process']['force_terminated']


@pytest.mark.parametrize('active,exit_code', [(True, None), (False, 2)])
def test_failed_child_immediately_warns_even_if_client_exited(module, candidate, capsys, active, exit_code):
    report = execute(module, candidate, FakeNative(child_active=active, child_exit=exit_code))
    assert report['robot_stop_unknown'] and report['operator_attention_required']
    assert '停止状态未知' in capsys.readouterr().out
    alert = json.loads((candidate['output']/'attention-required.json').read_text(encoding='utf-8'))
    assert alert['robot_stop_unknown'] and alert['operator_attention_required']
    assert report['pcaps']  # Failed motion still retains capture evidence.


def test_motion_launch_exception_keeps_process_and_stop_state_unknown(module, candidate, capsys):
    native = FakeNative()
    def interrupted(*args): raise OSError('synthetic failure after possible spawn')
    native.motion = interrupted
    report = execute(module, candidate, native)
    assert report['motion_start_attempted'] and report['manual_process_cleanup_required']
    assert report['robot_stop_unknown'] and report['operator_attention_required']
    assert '停止状态未知' in capsys.readouterr().out
    assert (candidate['output']/'attention-required.json').exists()


def test_stop_return_without_idle_confirmation_never_counts_as_capture_stopped(module, candidate):
    native = FakeNative()
    original = native.command
    def not_stopped(args):
        if args == ['stop']:
            native.calls.append(args)
            return 'OK'
        return original(args)
    native.command = not_stopped
    report = execute(module, candidate, native)
    assert not report['capture_stopped'] and report['manual_capture_cleanup_required']
    assert not report['acquisition_complete']


def test_foreign_capture_never_stopped(module, candidate):
    native = FakeNative(foreign=True)
    report = execute(module, candidate, native)
    assert report['manual_capture_cleanup_required'] and not report['capture_stopped']
    assert not any(c[0] == 'stop' for c in native.calls)


def test_foreign_filter_added_with_ours_blocks_before_capture_and_motion(module,candidate):
    native=FakeNative()
    original=native.command
    def mixed(args):
        value=original(args)
        if args[:2]==['filter','add']:
            native.filters+='\n2 Foreign TCP 192.0.2.99 3031'
        return value
    native.command=mixed
    report=execute(module,candidate,native)
    assert not report['capture_started']
    assert not any(c[0] in ('start','motion') for c in native.calls)


def test_ready_receipt_binds_actual_capture_and_approval_before_child(module,candidate):
    native=FakeNative()
    original=native.motion
    def check(*args):
        receipt=json.loads((candidate['output']/'capture-ready.json').read_text())
        assert receipt['config_sha256']==candidate['config_sha256']
        assert receipt['approved_preview_sha256']==candidate['approval']['sha256']
        assert receipt['component_id']==3
        assert receipt['filter_name'].startswith(module.FILTER_PREFIX)
        return original(*args)
    native.motion=check
    assert execute(module,candidate,native)['acquisition_complete']


def test_filter_name_is_fully_visible_in_observed_pktmon_name_column(module, candidate):
    # Windows listed the first 23 characters plus "..." for a 28-char name;
    # the earlier 26-char readonly name was displayed in full. Keep full-name
    # comparison and make the generated name fit that observed column.
    native = FakeNative()
    original = native.command
    def displayed(args):
        value = original(args)
        if args == ['filter', 'list'] and value != 'EMPTY':
            fields = value.split()
            if len(fields[1]) > 26:
                fields[1] = fields[1][:23] + '...'
            return ' '.join(fields)
        return value
    native.command = displayed
    report = execute(module, candidate, native)
    assert report['acquisition_complete']
    assert len(report['episode_filter_name']) <= 26
    suffix = report['episode_filter_name'].rsplit('_', 1)[1]
    assert len(suffix) == 12 and all(value in '0123456789abcdef' for value in suffix)
    assert sum(call[0] == 'motion' for call in native.calls) == 1


def test_truncated_filter_display_is_not_accepted_as_full_ownership(module, candidate):
    name = 'VR4ArmMotionStop_8a52ed2d299e'
    displayed = '1 VR4ArmMotionStop_8a52ed2... TCP 192.0.2.1 3031'
    assert not module.owns_filters(displayed, candidate, name)


def test_connection_admission_uses_only_numeric_local_netstat_table(module, monkeypatch):
    from types import SimpleNamespace
    def run(args, **kwargs):
        assert args == ['netstat.exe', '-ano']
        return SimpleNamespace(returncode=0, stderr=b'', stdout=(
            b'Active Connections\nProto Local Address Foreign Address State PID\n'
            b'TCP 127.0.0.1:5000 192.0.2.1:3031 ESTABLISHED 12\n'))
    monkeypatch.setattr(module.subprocess, 'run', run)
    assert module.Native().existing_rpc_connections() == 1


@pytest.mark.parametrize('key,value', [('max_joint_speed_radps', .5), ('max_tcp_step_m', True), ('max_tcp_speed_mps', 0)])
def test_unsafe_caps_cannot_create_candidate(module, candidate, key, value):
    import yaml
    data = yaml.safe_load(candidate['config'].read_bytes())
    data['real_robot']['control'][key] = value
    candidate['config'].write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='cap'):
        module.prepare(candidate['config'], module.ROOT/'.tmp/bad', component_id=3)


def test_motion_native_timeout_observes_own_child_without_force_kill(module, tmp_path, monkeypatch):
    calls = []
    class Child:
        pid, returncode = 7, None
        def wait(self, timeout):
            raise subprocess.TimeoutExpired('harmless_fake', timeout)
        def poll(self): return None
        def kill(self): pytest.fail('motion child must not be force-killed')
        def terminate(self): pytest.fail('motion child must not be force-terminated')
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *args, **kwargs: calls.append(args) or Child())
    native = module.Native()
    result = native.motion(['fixed_fake_child.py'], tmp_path/'stdout', tmp_path/'stderr', .01)
    assert result['timed_out'] and not result['process_exited'] and not result['force_terminated']
    assert len(calls) == 1
