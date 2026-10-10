"""Probe guards and protocol behavior; no SDK import in these tests."""
import asyncio
import json
import subprocess
import sys

import pytest
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve

from scripts import sdk_loopback_stop_probe as probe


def test_only_owned_ipv4_loopback_listener_supplies_sdk_port():
    assert probe.owned_port([("127.0.0.1", 43210)]) == 43210


@pytest.mark.parametrize("addresses", [
    [], [("0.0.0.0", 43210)], [("172.16.2.162", 3031)],
    [("127.0.0.1", 0)], [("127.0.0.1", 65536)],
    [("127.0.0.1", 43210), ("127.0.0.1", 43211)],
])
def test_nonlocal_or_ambiguous_listener_is_rejected_before_sdk_connect(addresses):
    with pytest.raises(ValueError):
        probe.owned_port(addresses)


def test_probe_request_accepts_only_fixed_methods_and_empty_params():
    request = {"jsonrpc": "2.0", "id": 7, "method": "stop_move", "params": [{}]}
    assert probe.parse_request(json.dumps(request)) == request


@pytest.mark.parametrize("changes", [
    {"method": "move_pvat"}, {"method": "set_claw"}, {"method": "stop_sys"},
    {"params": [{"other": 1}]}, {"id": True}, {"id": None}, {"jsonrpc": "1.0"},
    {"method": []}, {"method": {}},
])
def test_other_methods_or_malformed_requests_are_refused(changes):
    request = {"jsonrpc": "2.0", "id": 7, "method": "stop_move", "params": [{}], **changes}
    with pytest.raises(ValueError):
        probe.parse_request(json.dumps(request))


def test_cli_has_no_robot_target_or_configuration_switch(tmp_path):
    result = subprocess.run([
        sys.executable, "scripts/sdk_loopback_stop_probe.py", "--output", str(tmp_path / "result.json"),
        "--ip", "172.16.2.162",
    ], capture_output=True, text=True, timeout=5)
    assert result.returncode == 2
    assert not (tmp_path / "result.json").exists()


async def test_read_reply_completes_while_loopback_stop_reply_is_pending():
    peer = probe.LoopbackPeer(delay_s=0.1)
    async with serve(peer.handler, "127.0.0.1", 0) as server:
        port = probe.owned_port([sock.getsockname() for sock in server.sockets])
        try:
            async with connect(f"ws://127.0.0.1:{port}") as connection:
                await connection.send(json.dumps({"jsonrpc": "2.0", "id": 1, "method": "stop_move", "params": [{}]}))
                await asyncio.wait_for(peer.stop_received.wait(), 1)
                await connection.send(json.dumps({"jsonrpc": "2.0", "id": 2, "method": "get_robot_state", "params": [{}]}))
                read = json.loads(await asyncio.wait_for(connection.recv(), 1))
                stop = json.loads(await asyncio.wait_for(connection.recv(), 1))
                assert read == {"jsonrpc": "2.0", "id": 2, "result": {"state": "IDLE"}}
                assert stop == {"jsonrpc": "2.0", "id": 1, "result": {}}
            assert not peer.errors
            stop_record, read_record = peer.records
            assert read_record["send_completed_ns"] < stop_record["send_started_ns"]
            # Host sleep is a requested delay, not a perf_counter lower bound.
            # Removing the delay breaks the reply ordering assertion above.
            assert stop_record["send_started_ns"] > stop_record["received_ns"]
        finally:
            await peer.close()


def test_existing_evidence_is_preserved_before_sdk_initialization(tmp_path, monkeypatch):
    path = tmp_path / "results.json"
    path.write_bytes(b"previous evidence")

    def forbidden():
        pytest.fail("SDK import must not occur for an existing evidence path")

    monkeypatch.setattr(probe, "_sdk_identity", forbidden)
    with pytest.raises(SystemExit) as error:
        probe.main(["--output", str(path)])
    assert error.value.code == 2
    assert path.read_bytes() == b"previous evidence"


async def test_loopback_rpc_error_is_returned_without_a_success_result():
    peer = probe.LoopbackPeer(reply_error=True)
    async with serve(peer.handler, "127.0.0.1", 0) as server:
        port = probe.owned_port([sock.getsockname() for sock in server.sockets])
        try:
            async with connect(f"ws://127.0.0.1:{port}") as connection:
                await connection.send(json.dumps({"jsonrpc": "2.0", "id": "stop-1", "method": "stop_move", "params": [{}]}))
                reply = json.loads(await asyncio.wait_for(connection.recv(), 1))
                assert reply["id"] == "stop-1"
                assert reply["error"]["code"] == -32001
                assert "result" not in reply
            assert not peer.errors
        finally:
            await peer.close()
