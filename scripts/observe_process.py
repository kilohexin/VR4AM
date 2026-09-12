"""Observe one foreground command without killing or restarting it.

Not a robot supervisor. No shell expansion, SDK calls, or stop confirmation.
Output is written directly by the child to files; terminal mirroring is secondary.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time


def observe(command: list[str], output: Path, warn_after: float) -> int:
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    state = dict(status='STARTING', pid=None, child_exit_code=None,
                 started_utc=datetime.now(timezone.utc).isoformat(),
                 command=command, cwd=str(Path.cwd()), wait_threshold_exceeded=False,
                 robot_stop_confirmed=None, observer_errors=[])

    def note_error(value):
        if value not in state['observer_errors']:
            state['observer_errors'].append(value)

    def record(event):
        row = dict(state, event=event, utc=datetime.now(timezone.utc).isoformat(),
                   elapsed_s=time.monotonic() - started)
        with (output / 'events.jsonl').open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(row, ensure_ascii=False) + '\n')
        temp = output / 'process.json.tmp'
        temp.write_text(json.dumps(row, ensure_ascii=False, indent=2), encoding='utf-8')
        temp.replace(output / 'process.json')

    def safe_record(event):
        try:
            record(event)
        except OSError:
            note_error('metadata_write_failed')

    # Create every capture file before launch. Existing directories are rejected.
    with (output / 'stdout.bin').open('wb', buffering=0) as out, \
         (output / 'stderr.bin').open('wb', buffering=0) as err, \
         (output / 'stdout.bin').open('rb', buffering=0) as out_reader, \
         (output / 'stderr.bin').open('rb', buffering=0) as err_reader:
        record('PREPARED')
        flags = (subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW) if os.name == 'nt' else 0
        try:
            process = subprocess.Popen(
                command, stdin=subprocess.DEVNULL, stdout=out, stderr=err,
                shell=False, creationflags=flags, start_new_session=os.name != 'nt',
                env={**os.environ, 'PYTHONUNBUFFERED': '1'},
            )
        except OSError as error:
            state.update(status='START_FAILED', start_error=str(error))
            safe_record('START_FAILED')
            return 2
        state.update(status='RUNNING', pid=process.pid)
        safe_record('STARTED')
        # A slow terminal must never block child polling or durable metadata.
        # os.write avoids interpreter-buffer locks in a daemon blocked on a pipe.
        pending = queue.Queue(maxsize=16)
        mirrored = threading.Event()

        def terminal_worker():
            broken = set()
            try:
                while True:
                    item = pending.get()
                    if item is None:
                        return
                    descriptor, data = item
                    if descriptor in broken:
                        continue
                    try:
                        view = memoryview(data)
                        while view:
                            written = os.write(descriptor, view)
                            if written <= 0:
                                raise OSError('terminal write made no progress')
                            view = view[written:]
                    except OSError:
                        broken.add(descriptor)
                        note_error(f'terminal_mirror_failed:{descriptor}')
            finally:
                mirrored.set()

        threading.Thread(target=terminal_worker, daemon=True).start()

        def enqueue(item):
            try:
                pending.put_nowait(item)
            except queue.Full:
                note_error('terminal_mirror_backpressure')

        def mirror():
            more = False
            for reader, descriptor in ((out_reader, 1), (err_reader, 2)):
                try:
                    data = reader.read(65536)
                except OSError:
                    note_error('capture_read_failed')
                    continue
                if data:
                    more = True
                    enqueue((descriptor, data))
            return more

        while True:
            try:
                mirror()
                code = process.poll()
                if code is not None:
                    # Drain currently available direct-child output, not descendants.
                    while mirror():
                        pass
                    enqueue(None)
                    if not mirrored.wait(.5):
                        note_error('terminal_mirror_incomplete')
                    state.update(status='EXITED', child_exit_code=code)
                    safe_record('EXITED')
                    return code if not state['observer_errors'] else 2
                if not state['wait_threshold_exceeded'] and time.monotonic() - started >= warn_after:
                    state['wait_threshold_exceeded'] = True
                    safe_record('WAIT_THRESHOLD_EXCEEDED')
                    enqueue((2, (f'Wait threshold exceeded; PID {process.pid} is still RUNNING. '
                                 'No kill, retry, or robot stop was issued.\n').encode('utf-8')))
                time.sleep(.02)
            except KeyboardInterrupt:
                # Deliberately not a robot-stop shortcut. Keep evidence collection.
                safe_record('OBSERVER_INTERRUPT_CONTINUING')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New, exclusive capture directory')
    parser.add_argument('--warn-after', type=float, default=120., help='Warning only; never kills the child')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command or not math.isfinite(args.warn_after) or args.warn_after <= 0:
        parser.error('a command and a finite positive --warn-after are required')
    return observe(command, args.output, args.warn_after)


if __name__ == '__main__':
    raise SystemExit(main())
