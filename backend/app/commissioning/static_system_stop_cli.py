"""Independent, single-write diagnostic. System stop can cause physical movement."""
import argparse
import asyncio
import hashlib
import json
import os
from datetime import UTC, datetime
from pathlib import Path
from time import monotonic_ns

from app.config import Settings
from app.commissioning.static_system_stop import CONFIRM, run_probe
from app.robots.lebai_sdk_bridge import connect_real_client


async def execute(config_path, output_dir, confirm, *, client_factory=None):
    config_path, output_dir = Path(config_path), Path(output_dir)
    if confirm != CONFIRM:
        raise ValueError('explicit_system_stop_confirmation_required')
    original = config_path.read_bytes()
    settings = Settings.load(config_path)
    if settings.backend != 'lebai' or settings.lebai.mode != 'control':
        raise ValueError('control_configuration_required')
    # Exclusive reservation precedes any connection or command. Never reuse it,
    # including after failure or interruption.
    output_dir.mkdir(parents=True, exist_ok=False)
    report = dict(complete=False, write_started=False,
                  started_utc=datetime.now(UTC).isoformat(),
                  source_module=str(Path(__file__).resolve()),
                  config_sha256_before=hashlib.sha256(original).hexdigest())

    def unchanged():
        if config_path.read_bytes() != original:
            raise ValueError('config_changed')

    with (output_dir / 'events.jsonl').open('x', encoding='utf-8') as journal:
        def emit(event):
            journal.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
            journal.flush()
            os.fsync(journal.fileno())
            if event.get('kind') == 'probe_summary':
                report.update({k: v for k, v in event.items() if k != 'kind'})

        try:
            unchanged()
            emit({'kind': 'probe_started', **report})
            deadline = monotonic_ns() + 5_000_000_000
            client = await asyncio.wait_for((client_factory or connect_real_client)(settings.lebai.ip), 5.)
            if monotonic_ns() >= deadline:
                raise TimeoutError('late_connect')
            unchanged()
            report.update(await run_probe(client, settings.lebai, confirm=confirm,
                                          emit=emit, before_write=unchanged))
        except asyncio.CancelledError:
            report.update(complete=False, interrupted=True)
            raise
        except Exception as error:
            report.update(complete=False, error_type=type(error).__name__)
        finally:
            report['completed_utc'] = datetime.now(UTC).isoformat()
            try:
                current = config_path.read_bytes()
                report.update(config_sha256_after=hashlib.sha256(current).hexdigest(),
                              config_unchanged=current == original)
                if current != original:
                    report['complete'] = False
            except OSError:
                report.update(complete=False, config_unchanged=False)
            with (output_dir / 'result.json').open('x', encoding='utf-8') as output:
                json.dump(report, output, ensure_ascii=False, allow_nan=False, indent=2)
                output.write('\n')
    # Deliberately no adapter, shutdown recovery, or SDK control/disconnect hook.
    return 0 if report['complete'] else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--confirm', required=True)
    args = parser.parse_args(argv)
    print('System stop may cause movement. On abnormal motion prioritize onsite safety; '
          'do not wait for observation to finish.', flush=True)
    return asyncio.run(execute(args.config, args.output, args.confirm))
