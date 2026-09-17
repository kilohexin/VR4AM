"""Independent read-only dynamics collection; never constructs a control backend."""
import argparse
import asyncio
import hashlib
import json
from datetime import UTC, datetime
from pathlib import Path
from time import monotonic_ns

import yaml

from app.robots.lebai_sdk_bridge import connect_real_client


def utc():
    return datetime.now(UTC).isoformat()


async def collect_dynamics(config_path, output_path, client_factory=None):
    config_path, output_path = Path(config_path), Path(output_path)
    before = config_path.read_bytes()
    config = yaml.safe_load(before)
    real = config.get('real_robot') if isinstance(config, dict) else None
    if not isinstance(real, dict) or real.get('mode') != 'readonly':
        raise ValueError('dynamics_requires_readonly_mode')
    ip = real.get('ip')
    if not isinstance(ip, str) or not ip.strip():
        raise ValueError('invalid_robot_ip')
    report = dict(started_utc=utc(), complete=False, ip=ip, mode='readonly',
                  config_sha256_before=hashlib.sha256(before).hexdigest(), reads=[],
                  source_module=str(Path(__file__).resolve()),
                  scope='Configuration reads only; no robot state or motion authorization',
                  units={'mass': 'kg (SDK declaration)', 'cog': 'm (SDK declaration)',
                         'cog_frame': 'unverified', 'gravity': 'unverified'})
    # Reserve before connecting; a previous report is never overwritten.
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('x', encoding='utf-8') as output:
        try:
            if config_path.read_bytes() != before:
                raise ValueError('config_changed_before_connect')
            connect = client_factory or connect_real_client
            report['connect_started_utc'] = utc()
            connect_deadline = monotonic_ns() + 5_000_000_000
            client = await asyncio.wait_for(connect(ip), 5.0)
            if monotonic_ns() >= connect_deadline:
                raise TimeoutError('late_connect')
            report['connected_utc'] = utc()
            # Deliberate allowlist; no generic RPC dispatch, backend disconnect,
            # stop, startup, settings writes, or automatic retries.
            readers = (('get_payload', client.get_payload), ('get_gravity', client.get_gravity))
            for name, read in readers:
                item = dict(method=name, started_utc=utc(), started_ns=monotonic_ns())
                report['reads'].append(item)
                try:
                    raw = await asyncio.wait_for(read(), 2.0)
                    # Freeze returned JSON values without normalizing vectors or
                    # dropping unknown fields. Non-finite/non-JSON data fail closed.
                    item['raw'] = json.loads(json.dumps(raw, allow_nan=False))
                    if monotonic_ns() >= item['started_ns'] + 2_000_000_000:
                        raise TimeoutError('late_read')
                    item['outcome'] = 'returned'
                except Exception as error:
                    item.update(outcome='failed', error_type=type(error).__name__)
                    raise
                finally:
                    item.update(completed_ns=monotonic_ns(), completed_utc=utc())
            report['complete'] = True
        except Exception as error:
            report['error_type'] = type(error).__name__
        finally:
            report['completed_utc'] = utc()
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
    # CLI process exit releases its SDK connection. No adapter is created, so
    # there is no automatic stop/disconnect control path to run on exit.
    return 0 if report['complete'] else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    return asyncio.run(collect_dynamics(args.config, args.output))
