"""Motion approval material must bind expiry and installed SDK; synthetic tools only."""
from datetime import UTC, datetime, timedelta
import hashlib
import json

import pytest

from tests.scripts.test_motion_wire_capture import candidate, execute, FakeNative, module
from tests.scripts.test_wire_stop_admission import context


def approve(module, plan, expires):
    plan['authorization_expires_utc'] = expires
    path = plan['approval']['path']
    path.write_text(json.dumps(module.preview(plan)), encoding='utf-8')
    plan['approval'].update(sha256=hashlib.sha256(path.read_bytes()).hexdigest(), expires=expires)


def test_cli_expiry_extension_cannot_reuse_approved_material(module, candidate):
    approve(module, candidate, (datetime.now(UTC) + timedelta(seconds=30)).isoformat())
    candidate['approval']['expires'] = (datetime.now(UTC) + timedelta(minutes=5)).isoformat()
    native = FakeNative()
    with pytest.raises(ValueError, match='authorization_expiry_not_bound'):
        execute(module, candidate, native)
    assert not native.calls and not candidate['output'].exists()


def test_motion_preview_names_target_and_fixed_expiry(module, candidate):
    expires = (datetime.now(UTC) + timedelta(minutes=5)).isoformat()
    approve(module, candidate, expires)
    scope = module.preview(candidate)
    assert scope.get('target_ip') == '192.0.2.1'
    assert scope.get('authorization_expires_utc') == expires


def test_installed_sdk_change_rejects_before_capture_or_motion(module, candidate):
    sdk = module.ROOT / '.venv/Lib/site-packages/lebai_sdk/lebai_sdk.pyd'
    sdk.parent.mkdir(parents=True)
    sdk.write_bytes(b'synthetic SDK binary v1')
    candidate['runtime_hashes'] = module.runtime_hashes()
    approve(module, candidate, (datetime.now(UTC) + timedelta(minutes=5)).isoformat())
    sdk.write_bytes(b'synthetic SDK binary v2')
    native = FakeNative()
    with pytest.raises(ValueError, match='inputs_changed_before_execution'):
        execute(module, candidate, native)
    assert not native.calls and not candidate['output'].exists()


def test_motion_cli_rejects_other_python_before_native_tools(module, candidate, monkeypatch):
    approval = candidate['approval']
    baseline = module.ROOT / 'baseline.json'
    baseline.write_text(json.dumps(candidate['baseline']), encoding='utf-8')
    monkeypatch.setattr(module.sys, 'executable', str(module.ROOT / 'other-python.exe'))
    monkeypatch.setattr(module, 'Native', lambda: pytest.fail('unapproved interpreter reached Native'))
    with pytest.raises(ValueError, match='repository_venv_required'):
        module.main(['--config', str(candidate['config']), '--output', str(candidate['output']),
            '--component-id', '3', '--tool-baseline', str(baseline),
            '--expected-config-sha256', candidate['config_sha256'], '--execute', '--confirm', module.CONFIRM,
            '--approved-preview', str(approval['path']), '--approved-preview-sha256', approval['sha256'],
            '--authorization-expires-utc', approval['expires']])
    assert not candidate['output'].exists()


def test_motion_child_cli_rejects_other_python_before_sdk(context, monkeypatch):
    child, args = context
    monkeypatch.setattr(child.sys, 'executable', str(child.capture.ROOT / 'other-python.exe'))
    monkeypatch.setattr(child.capture, 'Native', lambda: pytest.fail('unapproved child reached capture tools'))
    with pytest.raises(ValueError, match='repository_venv_required'):
        child.main(['--config', str(args.config), '--output', str(args.output),
            '--component-id', str(args.component_id), '--tool-baseline-json', args.tool_baseline_json,
            '--expected-config-sha256', args.expected_config_sha256,
            '--approved-preview', str(args.approved_preview), '--approved-preview-sha256', args.approved_preview_sha256,
            '--authorization-expires-utc', args.authorization_expires_utc,
            '--execute-once', '--confirm', args.confirm])
    assert not (args.output / 'motion-admission.json').exists()


def test_sdk_package_entry_change_rejects_before_tools(module, candidate):
    package = module.ROOT / '.venv/Lib/site-packages/lebai_sdk'
    package.mkdir(parents=True)
    entry = package / '__init__.py'
    entry.write_text('# SDK package entry v1', encoding='utf-8')
    (package / 'lebai_sdk.pyd').write_bytes(b'synthetic SDK')
    candidate['runtime_hashes'] = module.runtime_hashes()
    approve(module, candidate, (datetime.now(UTC) + timedelta(minutes=5)).isoformat())
    entry.write_text('# SDK package entry v2', encoding='utf-8')
    native = FakeNative()
    with pytest.raises(ValueError, match='inputs_changed_before_execution'):
        execute(module, candidate, native)
    assert not native.calls


def sdk_sources(module):
    package = module.ROOT / '.venv/Lib/site-packages/lebai_sdk'
    package.mkdir(parents=True)
    marker = module.ROOT / 'sdk-imported.txt'
    (package / '__init__.py').write_text('from pathlib import Path\nPath(' + repr(str(marker)) + ').write_text("imported")', encoding='utf-8')
    (package / 'lebai_sdk.pyd').write_bytes(b'not a real extension: must not load')
    return package, marker


def test_sdk_source_check_rejects_same_name_shadow_without_import(module, candidate, monkeypatch):
    package, marker = sdk_sources(module)
    shadow = module.ROOT / 'backend'
    shadow.mkdir()
    (shadow / 'lebai_sdk.py').write_text('raise AssertionError("must not import shadow")', encoding='utf-8')
    monkeypatch.setattr(module.sys, 'path', [str(shadow), str(package.parent)])
    with pytest.raises(ValueError, match='sdk_import_source_not_bound'):
        module.verify_sdk_source()
    assert not marker.exists()


def test_sdk_source_check_rejects_missing_binary(module, candidate, monkeypatch):
    package, marker = sdk_sources(module)
    (package / 'lebai_sdk.pyd').unlink()
    monkeypatch.setattr(module.sys, 'path', [str(package.parent)])
    with pytest.raises(ValueError, match='sdk_bound_files_missing'):
        module.verify_sdk_source()
    assert not marker.exists()


def test_sdk_source_check_validates_origin_without_loading_extension(module, candidate, monkeypatch):
    package, marker = sdk_sources(module)
    monkeypatch.setattr(module.sys, 'path', [str(package.parent)])
    module.verify_sdk_source()
    assert not marker.exists()
