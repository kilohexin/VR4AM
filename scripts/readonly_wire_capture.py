"""Preview by default; one separately approved readonly process and Pktmon capture."""
import argparse
import ctypes
from datetime import datetime, UTC, timedelta
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
from uuid import uuid4

import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIRM = 'I_APPROVE_ONE_READONLY_WIRE_CAPTURE'


def digest_text(text):
    normalized = '\n'.join(line.rstrip() for line in text.splitlines()).strip()
    return hashlib.sha256(normalized.encode('utf-8')).hexdigest()


def runtime_hashes():
    paths = sorted((ROOT / 'backend/app').rglob('*.py'))
    paths += [ROOT / 'scripts/real_robot_readonly_state.py', ROOT / 'scripts/readonly_wire_capture.py',
              ROOT / 'scripts/launch_readonly_wire_capture.ps1']
    return {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in paths if path.is_file()}


def prepare(config, output, *, component_id, port=3031, baseline=None):
    if type(component_id) is not int or component_id <= 0:
        raise ValueError('one_positive_component_id_required')
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError('invalid_port')
    config = Path(config).resolve(strict=True)
    raw = config.read_bytes()
    value = yaml.safe_load(raw)
    real = value.get('real_robot') if isinstance(value, dict) else None
    if not isinstance(real, dict) or real.get('mode') != 'readonly':
        raise ValueError('readonly_configuration_required')
    ip = str(ipaddress.ip_address(real.get('ip')))
    output = Path(output).resolve()
    if not output.is_relative_to((ROOT / '.tmp').resolve()) or output == (ROOT / '.tmp').resolve():
        raise ValueError('output_must_be_new_directory_under_repo_tmp')
    return dict(config=config, config_bytes=raw, config_sha256=hashlib.sha256(raw).hexdigest(),
                ip=ip, output=output, component_id=component_id, port=port,
                baseline=dict(baseline or {}), runtime_hashes=runtime_hashes())


def preview(plan):
    return dict(mode='offline_preview', config_sha256=plan['config_sha256'],
                capture_output=str(plan['output']),
                component_id=plan['component_id'], port=plan['port'],
                readonly_window_ms=5000, capture_budget_ms=15000,
                new_site_authorization_required=True, reviewed_tool_baseline_required=True,
                hardware_stop_verified=False, runtime_hashes=plan['runtime_hashes'])


class Native:
    @property
    def admin(self):
        return os.name == 'nt' and bool(ctypes.windll.shell32.IsUserAnAdmin())

    def command(self, args):
        # Arguments are a list; no shell, no user-selected RPC or generic command.
        result = subprocess.run(['pktmon.exe', *args], capture_output=True, text=True,
                                errors='replace', timeout=5, shell=False)
        if result.returncode:
            raise RuntimeError(f'pktmon_exit_{result.returncode}')
        return result.stdout

    def readonly(self, args, stdout, stderr, budget_s):
        environment = dict(os.environ, PYTHONPATH=str(ROOT / 'backend'))
        with Path(stdout).open('xb') as out, Path(stderr).open('xb') as err:
            child = subprocess.Popen([sys.executable, *args], stdout=out, stderr=err,
                                     cwd=ROOT, env=environment, shell=False)
            result = dict(exit_code=None, timed_out=False, process_exited=False, pid=child.pid)
            try:
                child.wait(timeout=budget_s)
            except subprocess.TimeoutExpired:
                result['timed_out'] = True
            except Exception as error:
                result['error_type'] = type(error).__name__
            finally:
                if child.poll() is None:
                    try:
                        child.kill()
                        child.wait(timeout=5)
                    except Exception as error:
                        result['cleanup_error_type'] = type(error).__name__
                result.update(exit_code=child.returncode, process_exited=child.poll() is not None)
            return result


def owns_capture(status, directory):
    """Only a recognized current-log field inside this unique episode qualifies."""
    fields = re.findall(r'(?im)^\s*(?:Log file|日志文件)\s*[:：]\s*(.+?)\s*$', status)
    if len(fields) != 1:
        return False
    path = Path(fields[0].strip('"')).resolve()
    return (path.parent == directory.resolve()
            and re.fullmatch(r'capture(?:_?\d+)?\.etl', path.name, re.IGNORECASE) is not None)


class AuthorizationExpired(ValueError):
    pass


def check_expiry(expires):
    if expires is not None and datetime.now(UTC) >= expires:
        raise AuthorizationExpired('authorization_expired')


def approved_launch(plan, preview_path, preview_sha256, expiry):
    if not preview_path or not preview_sha256 or not expiry:
        raise ValueError('approved_preview_and_expiry_required')
    try:
        expires = datetime.fromisoformat(expiry.replace('Z', '+00:00'))
        if expires.tzinfo is None or expires.utcoffset() != timedelta(0):
            raise ValueError
    except (ValueError, TypeError):
        raise ValueError('invalid_authorization_expiry') from None
    check_expiry(expires)
    raw = Path(preview_path).read_bytes()
    if (not re.fullmatch('[a-f0-9]{64}', preview_sha256)
            or hashlib.sha256(raw).hexdigest() != preview_sha256):
        raise ValueError('approved_preview_hash_mismatch')
    try:
        value = json.loads(raw)
    except (ValueError, UnicodeError):
        raise ValueError('approved_preview_invalid') from None
    if (not isinstance(value, dict) or value.get('mode') != 'offline_preview'
            or value.get('capture_output') != str(plan['output'])
            or type(value.get('component_id')) is not int or type(value.get('port')) is not int
            or any(value.get(name) != plan[name] for name in
                   ('component_id', 'port', 'config_sha256', 'runtime_hashes'))):
        raise ValueError('approved_preview_inputs_changed')
    return expires


def execute(plan, native, *, confirm, clock=time.monotonic, authorization_expires=None):
    check_expiry(authorization_expires)
    if confirm != CONFIRM:
        raise ValueError('onsite_scope_confirmation_required')
    if not native.admin:
        raise ValueError('administrator_required')
    baseline = plan['baseline']
    if (baseline.get('reviewed_idle') is not True or baseline.get('reviewed_empty_filters') is not True
            or any(not isinstance(baseline.get(name), str)
                   or not re.fullmatch('[a-f0-9]{64}', baseline[name])
                   for name in ('idle_status_sha256', 'empty_filters_sha256'))):
        raise ValueError('reviewed_local_tool_baseline_required')
    if plan['config'].read_bytes() != plan['config_bytes']:
        raise ValueError('config_changed_before_execution')
    if runtime_hashes() != plan['runtime_hashes']:
        raise ValueError('runtime_changed_before_execution')
    output = plan['output']
    output.mkdir(parents=True, exist_ok=False)  # Consumes episode; never repeat/overwrite.
    report = dict(complete=False, acquisition_complete=False, hardware_stop_verified=False, tool_commands=[],
                  authorization_expired=False,
                  authorization_expires_utc=authorization_expires.isoformat() if authorization_expires else None,
                  capture_started=False, capture_stopped=False,
                  manual_capture_cleanup_required=False, manual_filter_cleanup_required=False,
                  manual_process_cleanup_required=False,
                  config_sha256_before=plan['config_sha256'], runtime_hashes=plan['runtime_hashes'])
    start_attempted = False
    child_result = None
    filter_name = 'VR4ArmReadonly_' + uuid4().hex[:12]
    report['episode_filter_name'] = filter_name

    def command(args):
        entry = dict(args=args, outcome='pending')
        report['tool_commands'].append(entry)
        try:
            result = native.command(args)
            entry.update(outcome='returned', output=result)
            return result
        except Exception as error:
            entry.update(outcome='failed', error_type=type(error).__name__)
            raise

    def idle():
        return digest_text(command(['status'])) == baseline['idle_status_sha256']

    try:
        if not idle():
            raise ValueError('pktmon_state_changed_or_not_idle')
        if digest_text(command(['filter', 'list'])) != baseline['empty_filters_sha256']:
            raise ValueError('pktmon_filters_changed_or_not_empty')
        shadow = output / 'config-readonly.yaml'
        shadow.write_bytes(plan['config_bytes'])
        # This host documents only global filter removal. Never delete all filters:
        # an equal fingerprint cannot exclude a concurrent foreign addition.
        report['manual_filter_cleanup_required'] = True
        command(['filter', 'add', filter_name, '-t', 'TCP', '-i', plan['ip'], '-p', str(plan['port'])])
        report['post_add_filters_sha256'] = digest_text(command(['filter', 'list']))
        if not idle():
            raise ValueError('pktmon_became_active_before_start')
        if (plan['config'].read_bytes() != plan['config_bytes']
                or runtime_hashes() != plan['runtime_hashes']):
            raise ValueError('inputs_changed_before_capture')
        capture_deadline = clock() + 15
        check_expiry(authorization_expires)
        start_attempted = True
        command(['start', '--capture', '--comp', str(plan['component_id']), '--pkt-size', '0',
                 '--file-name', str(output / 'capture.etl'), '--file-size', '32', '--log-mode', 'multi-file'])
        report['capture_started'] = True
        budget = capture_deadline - clock()
        if budget <= 0:
            raise TimeoutError('capture_budget_consumed_before_reader')
        if (plan['config'].read_bytes() != plan['config_bytes'] or shadow.read_bytes() != plan['config_bytes']
                or runtime_hashes() != plan['runtime_hashes']):
            raise ValueError('inputs_changed_before_readonly_connection')
        check_expiry(authorization_expires)
        child_result = native.readonly([
            str(ROOT / 'scripts/real_robot_readonly_state.py'), '--config', str(shadow),
            '--output', str(output / 'state-report.json'), '--expected-config-sha256', plan['config_sha256'],
            '--execute-readonly', *(['--authorization-expires-utc', authorization_expires.isoformat()]
                                   if authorization_expires else [])],
            output / 'reader-stdout.txt', output / 'reader-stderr.txt', budget)
        report['readonly_process'] = child_result
        report['manual_process_cleanup_required'] = not child_result.get('process_exited')
    except Exception as error:
        report['error_type'] = type(error).__name__
        report['authorization_expired'] = isinstance(error, AuthorizationExpired)
    finally:
        if report['capture_started']:
            try:
                current_status = command(['status'])
                if owns_capture(current_status, output):
                    command(['stop'])
                    report['capture_stopped'] = True
                elif digest_text(current_status) == baseline['idle_status_sha256']:
                    report['capture_stopped'] = True
                    report['capture_ended_externally'] = True
                else:
                    report['manual_capture_cleanup_required'] = True
            except Exception:
                report['manual_capture_cleanup_required'] = True
        elif start_attempted:
            # Failed/timeout native start does not prove whether global capture began.
            # Do not stop an unowned capture or remove its restricting filters.
            report['manual_capture_cleanup_required'] = True
        try:
            report['config_unchanged'] = plan['config'].read_bytes() == plan['config_bytes']
        except OSError as error:
            report.update(config_unchanged=False, config_check_error_type=type(error).__name__)
        try:
            report['runtime_unchanged'] = runtime_hashes() == plan['runtime_hashes']
        except OSError as error:
            report.update(runtime_unchanged=False, runtime_check_error_type=type(error).__name__)
        pcaps = []
        if report['capture_stopped']:
            try:
                for etl in sorted(output.glob('capture*.etl')):
                    pcap = etl.with_suffix('.pcapng')
                    if pcap.exists():
                        raise FileExistsError('pcap_output_exists')
                    command(['etl2pcap', str(etl), '--out', str(pcap)])
                    pcaps.append(dict(name=pcap.name, sha256=hashlib.sha256(pcap.read_bytes()).hexdigest()))
            except Exception as error:
                report['conversion_error_type'] = type(error).__name__
        report['pcaps'] = pcaps
        report['acquisition_complete'] = bool(child_result and child_result.get('exit_code') == 0
                                  and not child_result.get('timed_out') and child_result.get('process_exited')
                                  and report['capture_stopped'] and pcaps
                                  and report['config_unchanged'] and report['runtime_unchanged']
                                  and not report.get('capture_ended_externally')
                                  and 'conversion_error_type' not in report)
        report['complete'] = report['acquisition_complete'] and not report['manual_filter_cleanup_required']
        with (output / 'execution.json').open('x', encoding='utf-8') as file:
            json.dump(report, file, ensure_ascii=False, indent=2)
            file.write('\n')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--component-id', type=int, required=True)
    parser.add_argument('--port', type=int, default=3031)
    parser.add_argument('--tool-baseline', type=Path)
    parser.add_argument('--expected-config-sha256')
    parser.add_argument('--approved-preview', type=Path)
    parser.add_argument('--approved-preview-sha256')
    parser.add_argument('--authorization-expires-utc')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--confirm')
    args = parser.parse_args(argv)
    baseline = json.loads(args.tool_baseline.read_text(encoding='utf-8-sig')) if args.tool_baseline else None
    plan = prepare(args.config, args.output, component_id=args.component_id, port=args.port, baseline=baseline)
    if not args.execute:
        print(json.dumps(preview(plan), ensure_ascii=False, indent=2))
        return 0
    if args.expected_config_sha256 != plan['config_sha256']:
        raise ValueError('approved_config_hash_required')
    expires = approved_launch(plan, args.approved_preview, args.approved_preview_sha256,
                              args.authorization_expires_utc)
    report = execute(plan, Native(), confirm=args.confirm, authorization_expires=expires)
    print(json.dumps(dict(complete=report['complete'], acquisition_complete=report['acquisition_complete'],
                          authorization_expired=report['authorization_expired'],
                          episode_filter_name=report['episode_filter_name'], hardware_stop_verified=False,
                          manual_capture_cleanup_required=report['manual_capture_cleanup_required'],
                          manual_filter_cleanup_required=report['manual_filter_cleanup_required'],
                          manual_process_cleanup_required=report['manual_process_cleanup_required'])))
    return 0 if report['complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
