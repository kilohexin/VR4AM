"""Standalone state queries; CLI defaults to an offline configuration preview."""
import argparse
import asyncio
import hashlib
import json
from asyncio import sleep
from datetime import UTC, datetime
from pathlib import Path
from time import monotonic_ns

import yaml

from app.recording.clock_anchor import sample_clock_anchor
from app.robots.lebai_codec import estop_fault, joint_vector, map_robot_state, pose_from_lebai
from app.robots.lebai_sdk_bridge import connect_real_client
from app.schemas.messages import BackendState


READ_METHODS = ('get_robot_state', 'get_estop_reason', 'get_kin_data',
                'get_tcp', 'get_claw', 'get_payload', 'get_gravity')
POLL_METHODS = READ_METHODS[:3] + ('get_claw',)
WINDOW_NS = 5_000_000_000
READ_TIMEOUT_NS = 1_000_000_000


def utc():
    return datetime.now(UTC).isoformat()


def load_readonly(path):
    before = Path(path).read_bytes()
    config = yaml.safe_load(before)
    real = config.get('real_robot') if isinstance(config, dict) else None
    if not isinstance(real, dict) or real.get('mode') != 'readonly':
        raise ValueError('state_collection_requires_readonly_mode')
    ip = real.get('ip')
    if not isinstance(ip, str) or not ip.strip():
        raise ValueError('invalid_robot_ip')
    return before, ip


def validate_read(name, raw):
    # IDLE is only an admission condition for these queries, never stop proof.
    if name == 'get_robot_state' and map_robot_state(raw) is not BackendState.IDLE:
        raise ValueError('readonly_state_not_idle')
    if name == 'get_estop_reason' and estop_fault(raw) is not None:
        raise ValueError('readonly_estop_reported')
    if name == 'get_kin_data':
        if not isinstance(raw, dict):
            raise ValueError('invalid_kin_data')
        joint_vector(raw.get('actual_joint_pose'), 'actual_joint_pose')
        joint_vector(raw.get('actual_joint_speed'), 'actual_joint_speed')
        pose_from_lebai(raw.get('actual_tcp_pose'))
    if name == 'get_velocity_factor' and type(raw) is not int:
        raise ValueError('invalid_velocity_factor')


async def collect_state(config_path, output_path, client_factory=None, *, include_velocity_factor=False):
    """One SDK connection, fixed query list, no adapter or control cleanup.

    Run as a standalone CLI process for live use. Async deadlines bound query
    admission and detect late returns; SDK cancellation is not a hard deadline.
    Process exit is required to release SDK resources. Release is not inferred
    from dropping this function's client reference.
    """
    config_path, output_path = Path(config_path), Path(output_path)
    if type(include_velocity_factor) is not bool:
        raise ValueError('invalid_velocity_factor_option')
    before, ip = load_readonly(config_path)
    initial_methods = READ_METHODS + (('get_velocity_factor',) if include_velocity_factor else ())
    report = dict(started_utc=utc(), complete=False, mode='readonly', reads=[],
                  clock_anchor_started=sample_clock_anchor(),
                  scope='state queries only; no motion readiness or stopping verdict',
                  config_sha256_before=hashlib.sha256(before).hexdigest(),
                  query_allowlist=list(initial_methods), window_ms=5_000,
                  hardware_stop_verified=False,
                  connection_release='process_exit_required_not_verified')
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # Reserve the artifact before any connection; never replace earlier evidence.
    with output_path.open('x', encoding='utf-8') as output:
        try:
            if config_path.read_bytes() != before:
                raise ValueError('config_changed_before_connect')
            connect = client_factory or connect_real_client
            report['connect_started_ns'] = monotonic_ns()
            connect_deadline = report['connect_started_ns'] + WINDOW_NS
            client = await asyncio.wait_for(connect(ip), WINDOW_NS / 1e9)
            report['connect_completed_ns'] = monotonic_ns()
            if monotonic_ns() >= connect_deadline:
                raise TimeoutError('late_connect')
            if config_path.read_bytes() != before:
                raise ValueError('config_changed_during_connect')
            deadline = monotonic_ns() + WINDOW_NS
            report['window_deadline_ns'] = deadline
            methods = initial_methods
            first_cycle = True
            while monotonic_ns() < deadline:
                for name in methods:
                    if config_path.read_bytes() != before:
                        raise ValueError('config_changed_during_collection')
                    started = monotonic_ns()
                    if started >= deadline:
                        if first_cycle:
                            raise TimeoutError('initial_read_set_incomplete')
                        break
                    item = dict(method=name, started_ns=started, started_utc=utc(),
                                read_purpose='readonly_site_state')
                    report['reads'].append(item)
                    rpc_deadline = min(deadline, started + READ_TIMEOUT_NS)
                    try:
                        # Fixed names only, no user-selected/generic RPC facility.
                        raw = await asyncio.wait_for(getattr(client, name)(),
                                                     (rpc_deadline - started) / 1e9)
                        item['raw'] = json.loads(json.dumps(raw, allow_nan=False))
                        if monotonic_ns() >= rpc_deadline:
                            raise TimeoutError('late_read')
                        validate_read(name, item['raw'])
                        item['outcome'] = 'returned'
                    except asyncio.CancelledError:
                        item['outcome'] = 'cancelled'
                        raise
                    except Exception as error:
                        item.update(outcome='failed', error_type=type(error).__name__)
                        raise
                    finally:
                        item.update(completed_ns=monotonic_ns(), completed_utc=utc())
                first_cycle = False
                methods = POLL_METHODS
                remaining = (deadline - monotonic_ns()) / 1e9
                if remaining > 0:
                    await sleep(min(.25, remaining))
            report['complete'] = True
        except asyncio.CancelledError:
            report['cancelled'] = True
            raise
        except Exception as error:
            # Exception messages may contain credentials or device details.
            report['error_type'] = type(error).__name__
        finally:
            report['clock_anchor_completed'] = sample_clock_anchor()
            report['completed_utc'] = utc()
            report['completed_ns'] = monotonic_ns()
            try:
                after = config_path.read_bytes()
                report['config_sha256_after'] = hashlib.sha256(after).hexdigest()
                report['config_unchanged'] = after == before
                if after != before:
                    report['complete'] = False
            except OSError as error:
                report.update(complete=False, config_unchanged=False,
                              config_check_error=type(error).__name__)
            output.write(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2) + '\n')
    return 0 if report['complete'] else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--execute-readonly', action='store_true',
                        help='Connect only after explicit onsite read-only authorization')
    parser.add_argument('--include-velocity-factor', action='store_true',
                        help='Add exactly one get_velocity_factor query; needs scope approval for live use')
    args = parser.parse_args(argv)
    if not args.execute_readonly:
        before, _ = load_readonly(args.config)
        print(json.dumps(dict(action='offline_config_preview', mode='readonly',
                              config_sha256=hashlib.sha256(before).hexdigest(),
                              query_allowlist=list(READ_METHODS + (('get_velocity_factor',)
                                                   if args.include_velocity_factor else ())), window_ms=5_000)))
        return 0
    if args.output is None:
        parser.error('--execute-readonly requires --output')
    return asyncio.run(collect_state(args.config, args.output,
                                    include_velocity_factor=args.include_velocity_factor))
