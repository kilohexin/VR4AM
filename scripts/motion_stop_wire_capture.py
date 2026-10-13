"""Preview by default; separately approved single +Z3mm early-stop capture."""
import argparse
import hashlib
import importlib.util
import importlib.machinery
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time
from uuid import uuid4

import yaml

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location('wire_capture_safety', Path(__file__).with_name('readonly_wire_capture.py'))
_base = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_base)
_connection_spec = importlib.util.spec_from_file_location('local_rpc_table', Path(__file__).with_name('local_rpc_connections.py'))
_connections = importlib.util.module_from_spec(_connection_spec)
_connection_spec.loader.exec_module(_connections)
digest_text, owns_capture = _base.digest_text, _base.owns_capture
CONFIRM = 'I_APPROVE_ONE_Z3MM_MOTION_STOP_WIRE_CAPTURE'
# This host displays at most 26 name characters in `pktmon filter list`.
# Keep the 12-hex nonce and full-name ownership checks; never trust ellipses.
FILTER_PREFIX = 'VR4ArmStop_'
ACTION = dict(axis='z', distance_m=.003, early_stop_on_motion=True, observe_stop_seconds=2.0)
CAPS = dict(max_tcp_speed_mps=.005, max_tcp_rotation_radps=.05, max_tcp_acceleration_mps2=.02,
            max_tcp_angular_acceleration_radps2=.1, max_joint_speed_radps=.05,
            max_joint_acceleration_radps2=.2, max_tcp_step_m=.0005, max_tcp_rotation_step_deg=.2)


def runtime_hashes():
    paths = sorted((ROOT / 'backend/app').rglob('*.py'))
    paths += [ROOT / 'scripts' / name for name in
              ('readonly_wire_capture.py', 'motion_stop_wire_capture.py', 'real_robot_wire_stop.py',
               'real_robot_smoke.py', 'launch_readonly_wire_capture.ps1', 'local_rpc_connections.py')]
    paths += [ROOT / '.venv/Lib/site-packages/lebai_sdk' / name
              for name in ('__init__.py', 'lebai_sdk.pyd')]
    return {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if p.is_file()}


def prepare(config, output, *, component_id, port=3031, baseline=None, expires=None):
    if type(component_id) is not int or component_id <= 0 or type(port) is not int or port != 3031:
        raise ValueError('one_component_and_verified_port_required')
    config = Path(config).resolve(strict=True)
    raw = config.read_bytes()
    data = yaml.safe_load(raw)
    real = data.get('real_robot') if isinstance(data, dict) else None
    if not isinstance(real, dict) or data.get('backend') != 'lebai' or real.get('mode') != 'control':
        raise ValueError('control_lebai_configuration_required')
    control = real.get('control')
    if not isinstance(control, dict):
        raise ValueError('diagnostic_caps_required')
    for key, cap in CAPS.items():
        value = control.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= cap:
            raise ValueError('diagnostic_cap_invalid:' + key)
    ip = str(ipaddress.ip_address(real.get('ip')))
    output = Path(output).resolve()
    if not output.is_relative_to((ROOT / '.tmp').resolve()) or output == (ROOT / '.tmp').resolve():
        raise ValueError('unique_output_under_repo_tmp_required')
    return dict(config=config, config_bytes=raw, config_sha256=hashlib.sha256(raw).hexdigest(),
                output=output, ip=ip, component_id=component_id, port=port,
                baseline=dict(baseline or {}), runtime_hashes=runtime_hashes(),
                authorization_expires_utc=expires)


def preview(plan):
    return dict(mode='motion_stop_wire_offline_preview', action=dict(ACTION),
                target_ip=plan['ip'], authorization_expires_utc=plan.get('authorization_expires_utc'),
                capture_output=str(plan['output']), config_sha256=plan['config_sha256'],
                component_id=plan['component_id'], port=plan['port'], capture_budget_ms=30000,
                tool_baseline_sha256=hashlib.sha256(json.dumps(plan['baseline'], sort_keys=True).encode()).hexdigest(),
                runtime_hashes=plan['runtime_hashes'], automatic_prepare=False, automatic_home=False,
                motion_child_force_termination=False, new_site_authorization_required=True,
                hardware_stop_verified=False)


def validate_approval(plan, approved_preview, approved_preview_sha256, authorization_expires_utc):
    # Reuse reviewed UTC parsing, then require this entry's complete scope binding.
    if not approved_preview or not approved_preview_sha256 or not authorization_expires_utc:
        raise ValueError('approved_preview_and_expiry_required')
    from datetime import UTC, datetime, timedelta
    try:
        expires = datetime.fromisoformat(authorization_expires_utc.replace('Z', '+00:00'))
        if expires.tzinfo is None or expires.utcoffset() != timedelta(0):
            raise ValueError
    except (ValueError, TypeError):
        raise ValueError('invalid_authorization_expiry') from None
    _base.check_expiry(expires)
    if authorization_expires_utc != plan.get('authorization_expires_utc'):
        raise ValueError('authorization_expiry_not_bound')
    raw = Path(approved_preview).read_bytes()
    if (type(approved_preview_sha256) is not str or not re.fullmatch('[a-f0-9]{64}', approved_preview_sha256)
            or hashlib.sha256(raw).hexdigest() != approved_preview_sha256):
        raise ValueError('approved_preview_hash_mismatch')
    if json.loads(raw) != preview(plan):
        raise ValueError('approved_scope_or_inputs_changed')
    return expires


def unchanged(plan):
    if plan['config'].read_bytes() != plan['config_bytes'] or runtime_hashes() != plan['runtime_hashes']:
        raise ValueError('inputs_changed_before_execution')


def verify_sdk_source():
    """Resolve installed entry/extension without importing or initializing SDK."""
    package = (ROOT / '.venv/Lib/site-packages/lebai_sdk').resolve()
    entry, binary = package / '__init__.py', package / 'lebai_sdk.pyd'
    if not entry.is_file() or not binary.is_file():
        raise ValueError('sdk_bound_files_missing')
    if 'lebai_sdk' in sys.modules or 'lebai_sdk.lebai_sdk' in sys.modules:
        raise ValueError('sdk_already_loaded_before_admission')
    spec = importlib.machinery.PathFinder.find_spec('lebai_sdk', sys.path)
    if (spec is None or spec.origin is None or Path(spec.origin).resolve() != entry
            or list(spec.submodule_search_locations or ()) != [str(package)]):
        raise ValueError('sdk_import_source_not_bound')
    extension = importlib.machinery.PathFinder.find_spec('lebai_sdk.lebai_sdk', [str(package)])
    if (extension is None or extension.origin is None or Path(extension.origin).resolve() != binary
            or not isinstance(extension.loader, importlib.machinery.ExtensionFileLoader)):
        raise ValueError('sdk_import_source_not_bound')


def owns_filters(text, plan, filter_name):
    rows = [line.split() for line in text.splitlines() if re.match(r'^\s*\d+\s+', line)]
    return (len(rows) == 1 and rows[0][1:] == [filter_name, 'TCP', plan['ip'], str(plan['port'])])


class Native:
    command = _base.Native.command
    @property
    def admin(self):
        return _base.Native().admin
    def existing_rpc_connections(self):
        # Local numeric table only; unknown or failed output never becomes zero.
        return _connections.read_rpc_connections()
    def motion(self, args, stdout, stderr, budget_s):
        environment = dict(os.environ, PYTHONPATH=str(ROOT / 'backend'))
        with Path(stdout).open('xb') as out, Path(stderr).open('xb') as err:
            child = subprocess.Popen([sys.executable, *args], stdout=out, stderr=err,
                                     cwd=ROOT, env=environment, shell=False,
                                     creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            result = dict(exit_code=None, timed_out=False, process_exited=False, pid=child.pid, force_terminated=False)
            try:
                child.wait(timeout=budget_s)
            except subprocess.TimeoutExpired:
                result['timed_out'] = True
            except Exception as error:
                result['error_type'] = type(error).__name__
            # Never equate killing a motion client with a robot stop. A pending
            # child keeps its scoped workflow; operator must inspect the site.
            result.update(exit_code=child.returncode, process_exited=child.poll() is not None)
            if not result['process_exited']:
                self.pending_child = child
            return result


def execute(plan, native, *, confirm, approved_preview, approved_preview_sha256, authorization_expires_utc, clock=time.monotonic):
    expires = validate_approval(plan, approved_preview, approved_preview_sha256, authorization_expires_utc)
    if confirm != CONFIRM:
        raise ValueError('onsite_scope_confirmation_required')
    if not native.admin:
        raise ValueError('administrator_required')
    baseline = plan['baseline']
    if (baseline.get('reviewed_idle') is not True or baseline.get('reviewed_empty_filters') is not True
            or any(type(baseline.get(k)) is not str or re.fullmatch('[a-f0-9]{64}', baseline[k]) is None
                   for k in ('idle_status_sha256', 'empty_filters_sha256'))):
        raise ValueError('reviewed_tool_baseline_required')
    unchanged(plan)
    if native.existing_rpc_connections() != 0:
        raise ValueError('existing_rpc_connections')
    output = plan['output']
    output.mkdir(parents=True, exist_ok=False)
    report = dict(complete=False, acquisition_complete=False, hardware_stop_verified=False, tool_commands=[],
                  action=dict(ACTION), capture_started=False, capture_stopped=False, authorization_expired=False,
                  manual_capture_cleanup_required=False, manual_filter_cleanup_required=False,
                  manual_process_cleanup_required=False, motion_start_attempted=False,
                  robot_stop_unknown=False, operator_attention_required=False,
                  config_sha256=plan['config_sha256'], runtime_hashes=plan['runtime_hashes'],
                  approved_preview_sha256=approved_preview_sha256,
                  authorization_expires_utc=authorization_expires_utc,
                  episode_filter_name=FILTER_PREFIX + uuid4().hex[:12])
    attempted = False
    def stop_unknown():
        report.update(robot_stop_unknown=True, operator_attention_required=True)
        # Separate signal lets the hidden Windows child retain full logs while
        # its foreground launcher forwards the warning before ETL conversion.
        try:
            with (output/'attention-required.json').open('x', encoding='utf-8') as file:
                json.dump(dict(robot_stop_unknown=True, operator_attention_required=True), file)
                file.write('\n')
        except Exception as error:
            report['attention_signal_error_type'] = type(error).__name__
        print('VR4ARM_STOP_UNKNOWN: 运动诊断未正常完成，停止状态未知；请现场观察并按安全规程处理，禁止重试。', flush=True)
    def command(args):
        entry = dict(args=args, outcome='pending')
        report['tool_commands'].append(entry)
        try:
            value = native.command(args)
            entry.update(outcome='returned', output=value)
            return value
        except Exception as error:
            entry.update(outcome='failed', error_type=type(error).__name__)
            raise
    try:
        if digest_text(command(['status'])) != baseline['idle_status_sha256']:
            raise ValueError('capture_not_idle')
        if digest_text(command(['filter','list'])) != baseline['empty_filters_sha256']:
            raise ValueError('filters_not_empty')
        shadow = output / 'config-control.yaml'
        shadow.write_bytes(plan['config_bytes'])
        approved = output / 'approved-preview.json'
        approved.write_bytes(Path(approved_preview).read_bytes())
        validate_approval(plan, approved, approved_preview_sha256, authorization_expires_utc)
        report['manual_filter_cleanup_required'] = True
        command(['filter','add',report['episode_filter_name'],'-t','TCP','-i',plan['ip'],'-p','3031'])
        if not owns_filters(command(['filter','list']),plan,report['episode_filter_name']):
            raise ValueError('filters_not_exclusively_owned')
        if digest_text(command(['status'])) != baseline['idle_status_sha256']:
            raise ValueError('capture_became_active')
        unchanged(plan)
        _base.check_expiry(expires)
        deadline = clock() + 30
        attempted = True
        command(['start','--capture','--comp',str(plan['component_id']),'--pkt-size','0',
                 '--file-name',str(output/'capture.etl'),'--file-size','32','--log-mode','multi-file'])
        report['capture_started'] = True
        unchanged(plan)
        if shadow.read_bytes() != plan['config_bytes']:
            raise ValueError('shadow_config_changed')
        _base.check_expiry(expires)
        if (not owns_capture(command(['status']),output)
                or not owns_filters(command(['filter','list']),plan,report['episode_filter_name'])):
            raise ValueError('capture_or_filters_changed_before_child')
        budget = deadline - clock()
        if budget <= 0:
            raise TimeoutError('capture_budget_consumed')
        with (output/'capture-ready.json').open('x',encoding='utf-8') as file:
            json.dump(dict(config_sha256=plan['config_sha256'],approved_preview_sha256=approved_preview_sha256,
                           component_id=plan['component_id'],filter_name=report['episode_filter_name']),file,indent=2)
            file.write('\n')
        # Set before spawn: exceptions after launch cannot prove client exit.
        report.update(motion_start_attempted=True, manual_process_cleanup_required=True)
        report['motion_process'] = native.motion([
            str(ROOT/'scripts/real_robot_wire_stop.py'), '--config',str(shadow),'--output',str(output),
            '--component-id',str(plan['component_id']), '--tool-baseline-json',json.dumps(baseline),
            '--expected-config-sha256',plan['config_sha256'], '--approved-preview',str(approved),
            '--approved-preview-sha256',approved_preview_sha256, '--authorization-expires-utc',authorization_expires_utc,
            '--execute-once','--confirm',CONFIRM], output/'motion-stdout.txt', output/'motion-stderr.txt',budget)
        report['manual_process_cleanup_required'] = not report['motion_process'].get('process_exited')
        child = report['motion_process']
        if (report['manual_process_cleanup_required'] or child.get('exit_code') != 0
                or child.get('timed_out') or child.get('error_type')):
            stop_unknown()
    except Exception as error:
        report['error_type'] = type(error).__name__
        report['authorization_expired'] = isinstance(error, _base.AuthorizationExpired)
        if report['motion_start_attempted'] and not report['operator_attention_required']:
            stop_unknown()
    finally:
        if report['capture_started']:
            try:
                status = command(['status'])
                if owns_capture(status, output):
                    command(['stop'])
                    if digest_text(command(['status'])) == baseline['idle_status_sha256']:
                        report['capture_stopped'] = True
                    else:
                        report['manual_capture_cleanup_required'] = True
                else:
                    report['manual_capture_cleanup_required'] = True
            except Exception:
                report['manual_capture_cleanup_required'] = True
        elif attempted:
            report['manual_capture_cleanup_required'] = True
        pcaps = []
        if report['capture_stopped']:
            try:
                for etl in sorted(output.glob('capture*.etl')):
                    pcap = etl.with_suffix('.pcapng')
                    if pcap.exists():
                        raise FileExistsError('pcap_output_exists')
                    command(['etl2pcap',str(etl),'--out',str(pcap)])
                    pcaps.append(dict(name=pcap.name,sha256=hashlib.sha256(pcap.read_bytes()).hexdigest()))
            except Exception as error:
                report['conversion_error_type'] = type(error).__name__
        report['pcaps'] = pcaps
        try:
            unchanged(plan)
            report['inputs_unchanged'] = True
        except Exception:
            report['inputs_unchanged'] = False
        child = report.get('motion_process', {})
        report['acquisition_complete'] = bool(child.get('process_exited') and child.get('exit_code') == 0
            and not child.get('timed_out') and not child.get('error_type') and pcaps
            and report['capture_stopped'] and report['inputs_unchanged']
            and 'error_type' not in report and 'conversion_error_type' not in report)
        report['complete'] = report['acquisition_complete'] and not report['manual_filter_cleanup_required']
        with (output/'execution.json').open('x',encoding='utf-8') as file:
            json.dump(report,file,ensure_ascii=False,indent=2)
            file.write('\n')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--component-id',type=int,required=True)
    parser.add_argument('--tool-baseline',type=Path)
    parser.add_argument('--expected-config-sha256')
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--confirm')
    parser.add_argument('--approved-preview',type=Path)
    parser.add_argument('--approved-preview-sha256')
    parser.add_argument('--authorization-expires-utc')
    args = parser.parse_args(argv)
    baseline = json.loads(args.tool_baseline.read_text(encoding='utf-8-sig')) if args.tool_baseline else None
    plan = prepare(args.config,args.output,component_id=args.component_id,baseline=baseline,
                   expires=args.authorization_expires_utc)
    if not args.execute:
        print(json.dumps(preview(plan),ensure_ascii=False,indent=2))
        return 0
    if args.expected_config_sha256 != plan['config_sha256']:
        raise ValueError('approved_config_hash_mismatch')
    # Validate before constructing Native, including the expiry and scope bytes.
    validate_approval(plan,args.approved_preview,args.approved_preview_sha256,args.authorization_expires_utc)
    if Path(sys.executable).resolve() != (ROOT / '.venv/Scripts/python.exe').resolve():
        raise ValueError('repository_venv_required')
    verify_sdk_source()
    report = execute(plan,Native(),confirm=args.confirm,approved_preview=args.approved_preview,
                     approved_preview_sha256=args.approved_preview_sha256,authorization_expires_utc=args.authorization_expires_utc)
    print(json.dumps({k:report[k] for k in ('complete','acquisition_complete','hardware_stop_verified',
          'manual_capture_cleanup_required','manual_filter_cleanup_required','manual_process_cleanup_required',
          'robot_stop_unknown','operator_attention_required')},ensure_ascii=False))
    return 0 if report['complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
