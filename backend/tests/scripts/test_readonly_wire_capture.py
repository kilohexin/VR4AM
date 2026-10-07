"""Capture orchestration with injected commands and harmless fake children."""
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('readonly_wire_capture', ROOT / 'scripts/readonly_wire_capture.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FakeNative:
    def __init__(self, *, fail=None, timeout=False, mutate_filters=False):
        self.calls = []
        self.fail, self.timeout, self.mutate_filters = fail, timeout, mutate_filters
        self.filtered = False
        self.child_exit = 0
        self.child_started = False
        self.admin = True
        self.active_path = None
    def command(self, args):
        self.calls.append(list(args))
        if self.fail == args[0]:
            raise RuntimeError('synthetic failure')
        if args[:2] == ['filter', 'list']:
            return 'ONLY_OUR_FILTER' if self.filtered and not self.mutate_filters else ('OTHER_FILTER' if self.filtered else 'EMPTY')
        if args[0] == 'status':
            return 'Log file: ' + self.active_path if self.active_path else 'IDLE'
        if args[:2] == ['filter', 'add']:
            self.filtered = True
        if args[:2] == ['filter', 'remove']:
            self.filtered = False
        if args[0] == 'start':
            self.active_path = args[args.index('--file-name') + 1]
            Path(args[args.index('--file-name') + 1]).write_bytes(b'artificial ETL')
        if args[0] == 'stop':
            self.active_path = None
        if args[0] == 'etl2pcap':
            Path(args[args.index('--out') + 1]).write_bytes(b'artificial PCAP')
        return 'OK'
    def readonly(self, args, stdout, stderr, budget_s):
        self.child_started = True
        self.calls.append(['readonly', *args])
        assert budget_s <= 15
        assert '--execute-readonly' in args and '--include-velocity-factor' not in args
        if self.timeout:
            return dict(exit_code=-1, timed_out=True, process_exited=True)
        return dict(exit_code=self.child_exit, timed_out=False, process_exited=True)


@pytest.fixture
def candidate(tmp_path, monkeypatch):
    monkeypatch.setattr(module, 'ROOT', tmp_path)
    (tmp_path / 'scripts').mkdir()
    (tmp_path / 'scripts/real_robot_readonly_state.py').write_text('# harmless fixture')
    config = tmp_path / 'readonly.yaml'
    config.write_text('real_robot:\n  mode: readonly\n  ip: 192.0.2.1\n')
    baseline = dict(schema_version=1, reviewed_idle=True, reviewed_empty_filters=True,
                    idle_status_sha256=module.digest_text('IDLE'),
                    empty_filters_sha256=module.digest_text('EMPTY'))
    return module.prepare(config, tmp_path / '.tmp/once', component_id=10, baseline=baseline)


def test_preview_does_not_execute_commands_or_create_output(candidate):
    view = module.preview(candidate)
    assert view['mode'] == 'offline_preview'
    assert view['readonly_window_ms'] == 5000
    assert view['capture_budget_ms'] == 15000
    assert not candidate['output'].exists()
    assert '192.0.2.1' not in json.dumps(view)


def test_confirmation_and_admin_required_before_any_command(candidate):
    native = FakeNative()
    with pytest.raises(ValueError, match='confirmation'):
        module.execute(candidate, native, confirm='wrong')
    native.admin = False
    with pytest.raises(ValueError, match='administrator'):
        module.execute(candidate, native, confirm=module.CONFIRM)
    assert native.calls == []


def test_single_capture_reader_and_cleanup_use_scoped_fixed_arguments(candidate):
    native = FakeNative()
    report = module.execute(candidate, native, confirm=module.CONFIRM)
    assert report['acquisition_complete'] is True
    assert report['complete'] is False
    assert report['manual_filter_cleanup_required'] is True
    assert report['hardware_stop_verified'] is False
    assert native.child_started
    assert [c[0] for c in native.calls].count('start') == 1
    assert [c[0] for c in native.calls].count('stop') == 1
    assert [c[0] for c in native.calls].count('readonly') == 1
    start = next(c for c in native.calls if c[0] == 'start')
    assert start[start.index('--comp') + 1] == '10'
    assert start[start.index('--pkt-size') + 1] == '0'
    assert start[start.index('--log-mode') + 1] == 'multi-file'
    assert next(c for c in native.calls if c[:2] == ['filter', 'add'])[-4:] == ['-i', '192.0.2.1', '-p', '3031']
    assert candidate['config'].read_bytes() == candidate['config_bytes']
    assert not any(c[:2] == ['filter', 'remove'] for c in native.calls)


@pytest.mark.parametrize('method', ['status', 'filter'])
def test_failed_tool_query_never_starts_capture_or_robot_reader(candidate, method):
    native = FakeNative(fail=method)
    report = module.execute(candidate, native, confirm=module.CONFIRM)
    assert not report['complete']
    assert not native.child_started
    assert not any(c[0] in ('start', 'stop', 'readonly') for c in native.calls)


def test_active_or_unknown_status_is_rejected_without_stopping_other_capture(candidate):
    candidate['baseline']['idle_status_sha256'] = module.digest_text('OTHER_STATE')
    native = FakeNative()
    report = module.execute(candidate, native, confirm=module.CONFIRM)
    assert not report['complete']
    assert not any(c[0] in ('start', 'stop', 'readonly') for c in native.calls)


def test_existing_filters_are_preserved_and_no_capture_is_started(candidate):
    candidate['baseline']['empty_filters_sha256'] = module.digest_text('OTHER_FILTERS')
    native = FakeNative()
    report = module.execute(candidate, native, confirm=module.CONFIRM)
    assert not report['complete']
    assert not any(c[:2] in (['filter', 'add'], ['filter', 'remove']) for c in native.calls)


def test_capture_start_failure_does_not_connect_or_stop_unowned_capture(candidate):
    native = FakeNative(fail='start')
    report = module.execute(candidate, native, confirm=module.CONFIRM)
    assert not report['complete']
    assert not native.child_started
    assert not any(c[0] == 'stop' for c in native.calls)
    assert report['manual_capture_cleanup_required']


@pytest.mark.parametrize('timeout,exit_code', [(True, 0), (False, 2)])
def test_reader_failure_still_stops_owned_capture_once_without_retry(candidate, timeout, exit_code):
    native = FakeNative(timeout=timeout)
    native.child_exit = exit_code
    report = module.execute(candidate, native, confirm=module.CONFIRM)
    assert not report['complete']
    assert sum(c[0] == 'stop' for c in native.calls) == 1
    assert sum(c[0] == 'readonly' for c in native.calls) == 1


def test_failed_stop_is_reported_as_unconfirmed_local_capture_cleanup(candidate):
    native = FakeNative(fail='stop')
    report = module.execute(candidate, native, confirm=module.CONFIRM)
    assert not report['complete']
    assert report['manual_capture_cleanup_required']
    assert not any(c[0] == 'etl2pcap' for c in native.calls)


def test_filter_change_during_capture_never_deletes_someone_elses_filters(candidate):
    native = FakeNative()
    original = native.readonly
    def changed(*args):
        result = original(*args)
        native.mutate_filters = True
        return result
    native.readonly = changed
    report = module.execute(candidate, native, confirm=module.CONFIRM)
    assert report['manual_filter_cleanup_required']
    assert not any(c[:2] == ['filter', 'remove'] for c in native.calls)


def test_changed_config_rejected_before_commands_and_not_restored(candidate):
    candidate['config'].write_text('changed by owner')
    native = FakeNative()
    with pytest.raises(ValueError, match='config_changed'):
        module.execute(candidate, native, confirm=module.CONFIRM)
    assert candidate['config'].read_text() == 'changed by owner'
    assert native.calls == []


def test_existing_output_is_never_overwritten_and_episode_never_rerun(candidate):
    candidate['output'].mkdir(parents=True)
    (candidate['output'] / 'existing').write_text('old evidence')
    native = FakeNative()
    with pytest.raises(FileExistsError):
        module.execute(candidate, native, confirm=module.CONFIRM)
    assert native.calls == []
    assert (candidate['output'] / 'existing').read_text() == 'old evidence'


@pytest.mark.parametrize('mode', ['control', 'null', 'missing'])
def test_configuration_must_be_semantically_readonly(tmp_path, mode, monkeypatch):
    monkeypatch.setattr(module, 'ROOT', tmp_path)
    config = tmp_path / 'config.yaml'
    config.write_text(f'real_robot:\n  mode: {mode}\n  ip: 192.0.2.1\n')
    with pytest.raises(ValueError, match='readonly'):
        module.prepare(config, tmp_path / '.tmp/out', component_id=1)


@pytest.mark.parametrize('component', [0, True, -1, 'all'])
def test_component_must_be_one_positive_id(tmp_path, component):
    with pytest.raises(ValueError):
        module.prepare(tmp_path / 'missing', tmp_path / '.tmp/out', component_id=component)


def test_unreviewed_tool_baseline_cannot_allow_execution(candidate):
    candidate['baseline']['reviewed_idle'] = False
    native = FakeNative()
    with pytest.raises(ValueError, match='baseline'):
        module.execute(candidate, native, confirm=module.CONFIRM)
    assert native.calls == []


def test_unchanged_mode_but_changed_shadow_endpoint_never_starts_reader(candidate):
    native = FakeNative()
    original = native.command
    def mutate(args):
        result = original(args)
        if args[0] == 'start':
            (candidate['output'] / 'config-readonly.yaml').write_text('real_robot:\n  mode: readonly\n  ip: 192.0.2.2\n')
        return result
    native.command = mutate
    report = module.execute(candidate, native, confirm=module.CONFIRM)
    assert not native.child_started
    assert not report['complete']
    assert sum(c[0] == 'stop' for c in native.calls) == 1


def test_deleted_original_config_is_not_restored_and_cleanup_report_survives(candidate):
    native = FakeNative()
    original = native.readonly
    def remove(*args):
        result = original(*args)
        candidate['config'].unlink()
        return result
    native.readonly = remove
    report = module.execute(candidate, native, confirm=module.CONFIRM)
    assert not candidate['config'].exists()
    assert not report['config_unchanged']
    assert not report['complete']
    assert (candidate['output'] / 'execution.json').is_file()


def test_native_reader_watchdog_with_harmless_sleeping_child(tmp_path):
    child = tmp_path / 'harmless.py'
    child.write_text('import time\ntime.sleep(60)\n')
    result = module.Native().readonly([str(child)], tmp_path / 'stdout', tmp_path / 'stderr', .05)
    assert result['timed_out'] and result['process_exited']
    assert result['exit_code'] != 0


def test_foreign_capture_replaces_our_episode_and_is_never_stopped(candidate):
    native = FakeNative()
    original = native.readonly
    def replace(*args):
        result = original(*args)
        native.active_path = 'C:\\OtherTrace\\other.etl'
        return result
    native.readonly = replace
    report = module.execute(candidate, native, confirm=module.CONFIRM)
    assert not report['complete']
    assert report['manual_capture_cleanup_required']
    assert not any(c[0] == 'stop' for c in native.calls)
    assert not any(c[:2] == ['filter', 'remove'] for c in native.calls)


def test_cli_preview_never_constructs_native_backend_or_creates_directory(candidate, monkeypatch):
    def forbidden():
        pytest.fail('preview must not construct native executor')
    monkeypatch.setattr(module, 'Native', forbidden)
    assert module.main(['--config', str(candidate['config']), '--output', str(candidate['output']),
                        '--component-id', '10']) == 0
    assert not candidate['output'].exists()


def test_foreign_filter_added_before_post_add_fingerprint_is_preserved(candidate):
    native = FakeNative(mutate_filters=True)
    report = module.execute(candidate, native, confirm=module.CONFIRM)
    assert not any(c[:2] == ['filter', 'remove'] for c in native.calls)
    assert native.filtered
    assert report['manual_filter_cleanup_required']
    assert report['episode_filter_name'].startswith('VR4ArmReadonly_')
