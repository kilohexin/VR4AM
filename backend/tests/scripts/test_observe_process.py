import json
import subprocess
import sys
import queue
import threading
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[3] / 'scripts' / 'observe_process.py'


def run_probe(tmp_path, code, warning='5'):
    output = tmp_path / 'capture'
    result = subprocess.run([
        sys.executable, str(SCRIPT), '--output', str(output), '--warn-after', warning,
        '--', sys.executable, '-u', '-c', code,
    ], capture_output=True, timeout=10)
    return result, output


@pytest.mark.parametrize('exit_code', [0, 7])
def test_full_streams_and_original_exit_code_are_preserved(tmp_path, exit_code):
    result, output = run_probe(tmp_path,
        f"import sys; print('OUT'*10000); print('ERR', file=sys.stderr); sys.exit({exit_code})")
    assert result.returncode == exit_code
    assert (output / 'stdout.bin').read_bytes().strip() == b'OUT' * 10000
    assert (output / 'stderr.bin').read_bytes().strip() == b'ERR'
    assert b'OUT' * 10000 in result.stdout
    assert b'ERR' in result.stderr
    state = json.loads((output / 'process.json').read_text(encoding='utf-8'))
    assert state['status'] == 'EXITED'
    assert state['child_exit_code'] == exit_code
    assert state['pid'] > 0
    assert state['robot_stop_confirmed'] is None


def test_warning_does_not_kill_or_restart_silent_child(tmp_path):
    result, output = run_probe(tmp_path,
        "import time; print('ONCE'); time.sleep(.4); print('FINISHED')", warning='.05')
    assert result.returncode == 0
    assert (output / 'stdout.bin').read_bytes().splitlines() == [b'ONCE', b'FINISHED']
    events = [json.loads(s) for s in (output / 'events.jsonl').read_text(encoding='utf-8').splitlines()]
    assert sum(e['event'] == 'STARTED' for e in events) == 1
    warning, = [e for e in events if e['event'] == 'WAIT_THRESHOLD_EXCEEDED']
    assert warning['status'] == 'RUNNING'
    assert warning['child_exit_code'] is None
    assert events[-1]['wait_threshold_exceeded']
    assert events[-1]['event'] == 'EXITED'


def test_silent_success_has_empty_files_not_missing_output(tmp_path):
    result, output = run_probe(tmp_path, 'import time; time.sleep(.1)')
    assert result.returncode == 0
    assert (output / 'stdout.bin').read_bytes() == b''
    assert (output / 'stderr.bin').read_bytes() == b''


def test_existing_capture_is_not_overwritten_or_child_launched(tmp_path):
    output = tmp_path / 'capture'
    output.mkdir()
    marker = output / 'keep'
    marker.write_text('original')
    result, _ = run_probe(tmp_path, "print('SHOULD_NOT_RUN')")
    assert result.returncode != 0
    assert marker.read_text() == 'original'
    assert not (output / 'stdout.bin').exists()


def test_output_is_visible_before_child_finishes(tmp_path):
    release = tmp_path / 'release'
    code = ("import pathlib,time; print('LIVE',flush=True); "
            f"p=pathlib.Path({str(release)!r}); "
            "exec('while not p.exists():\\n time.sleep(.02)')")
    process = subprocess.Popen([
        sys.executable, str(SCRIPT), '--output', str(tmp_path / 'live'),
        '--', sys.executable, '-u', '-c', code,
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    received = queue.Queue()
    reader = threading.Thread(target=lambda: received.put(process.stdout.readline()), daemon=True)
    reader.start()
    try:
        assert received.get(timeout=3).strip() == b'LIVE'
        assert process.poll() is None
    finally:
        release.write_text('release local fake child')
        process.communicate(timeout=5)
        reader.join(timeout=1)
    assert process.returncode == 0


def test_launch_failure_has_no_fabricated_child_exit_code(tmp_path):
    output = tmp_path / 'bad-command'
    result = subprocess.run([
        sys.executable, str(SCRIPT), '--output', str(output), '--',
        str(tmp_path / 'nonexistent-executable'),
    ], capture_output=True, timeout=5)
    assert result.returncode == 2
    state = json.loads((output / 'process.json').read_text(encoding='utf-8'))
    assert state['status'] == 'START_FAILED'
    assert state['pid'] is state['child_exit_code'] is None


@pytest.mark.parametrize('warning', ['0', '-1', 'nan', 'inf'])
def test_invalid_threshold_rejected_before_launch(tmp_path, warning):
    result, output = run_probe(tmp_path, "print('must not launch')", warning)
    assert result.returncode == 2
    assert not output.exists()


def test_unread_terminal_pipe_cannot_block_exit_recording(tmp_path):
    output = tmp_path / 'backpressure'
    process = subprocess.Popen([
        sys.executable, str(SCRIPT), '--output', str(output), '--',
        sys.executable, '-u', '-c', "import sys; sys.stdout.write('X'*2000000); sys.exit(7)",
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        # Intentionally do not consume stdout until the observer has exited.
        assert process.wait(timeout=4) == 2  # Mirror incomplete, child code separately retained.
        state = json.loads((output / 'process.json').read_text(encoding='utf-8'))
        assert state['status'] == 'EXITED'
        assert state['child_exit_code'] == 7
        assert state['observer_errors']
        assert (output / 'stdout.bin').stat().st_size == 2000000
    finally:
        # Unblock old/broken implementation if the regression assertion fails.
        process.communicate(timeout=5)
