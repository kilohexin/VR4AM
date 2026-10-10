"""Probe protocol/guard tests; installed native SDK is never imported here."""
import json
import asyncio
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve

from scripts import application_stop_loopback_probe as probe


def request(**changes):
    return json.dumps({"jsonrpc": "2.0", "id": 1, "method": "stop_move", "params": [{}], **changes})


@pytest.mark.parametrize("method", ["stop_move", "get_robot_state", "get_estop_reason",
                                   "get_kin_data", "get_tcp", "get_claw", "get_running_motion"])
def test_only_stop_and_required_empty_read_requests_are_accepted(method):
    assert probe.parse_request(request(method=method))["method"] == method


@pytest.mark.parametrize("change", [{"method": "stop_sys"}, {"method": "move_pvat"},
                                    {"method": "movej"}, {"method": "set_claw"},
                                    {"method": []}, {"id": True}, {"params": [{"x": 1}]},
                                    {"jsonrpc": "1.0"}])
def test_motion_gripper_reset_escalation_and_bad_protocol_are_refused(change):
    with pytest.raises(ValueError):
        probe.parse_request(request(**change))


def test_snapshot_fixture_is_full_and_has_explicit_wire_cartesian_pose():
    kin = probe.fixture_result("get_kin_data")
    assert len(kin["actual_joint_pose"]) == 6
    assert kin["actual_joint_speed"] == [0.0] * 6
    assert kin["actual_tcp_pose"]["position"] == {"x": 0.3, "y": 0.0, "z": 0.4}
    assert "rotation" in kin["actual_tcp_pose"]
    assert probe.fixture_result("get_robot_state", moving=True) == {"state": "MOVING"}
    assert probe.fixture_result("get_estop_reason") == {"reason": "NONE"}


def test_fixed_settings_do_not_read_configuration_or_use_field_target(monkeypatch):
    monkeypatch.setenv("VR4ARM_CONFIG", "missing-field-configuration.yaml")
    settings = probe.loopback_settings()
    assert settings.ip == "127.0.0.1"
    assert settings.mode == "control"
    assert settings.expected_tcp.z == 0.175


def test_prompt_ack_with_still_moving_cannot_be_classified_as_confirmed_stop():
    probe.assert_case_outcome("immediate_ack_moving_600ms",
        {"outcome": "error", "error": "stop_incomplete"}, "stop_incomplete", False, False)
    with pytest.raises(RuntimeError):
        probe.assert_case_outcome("immediate_ack_moving_600ms",
            {"outcome": "returned"}, None, True, True)


@pytest.mark.parametrize("host,port", [("172.16.2.162", 3031), ("localhost", 3031),
                                      ("0.0.0.0", 3031), ("127.0.0.1", False),
                                      ("127.0.0.1", 0), ("127.0.0.1", 65536)])
async def test_nonowned_or_nonliteral_loopback_target_is_rejected_before_sdk(host, port):
    sdk = SimpleNamespace(connect=AsyncMock())
    with pytest.raises(ValueError):
        await probe.connect_owned(sdk, host, port)
    sdk.connect.assert_not_called()


def test_existing_output_is_preserved_before_native_sdk_initialization(tmp_path, monkeypatch):
    path = tmp_path / "existing.json"
    path.write_bytes(b"existing evidence")
    def forbidden():
        pytest.fail("native SDK must not be imported")
    monkeypatch.setattr(probe, "_sdk_identity", forbidden)
    with pytest.raises(SystemExit) as result:
        probe.main(["--output", str(path)])
    assert result.value.code == 2
    assert path.read_bytes() == b"existing evidence"


def test_cli_has_no_target_or_field_configuration_argument(tmp_path):
    result = subprocess.run([sys.executable, "scripts/application_stop_loopback_probe.py",
                             "--output", str(tmp_path / "result.json"), "--ip", "172.16.2.162"],
                            capture_output=True, text=True, timeout=8)
    assert result.returncode == 2
    assert not (tmp_path / "result.json").exists()


@pytest.mark.parametrize("case,error,fault", [
    ("delay_600ms_idle", "sdk_timeout:stop_move", "stop_unverified"),
    ("rpc_error", "sdk_call_failed:stop_move", "stop_unverified"),
    ("immediate_ack_moving_600ms", "stop_incomplete", "stop_incomplete")])
def test_failed_case_requires_correct_error_latch_and_revoked_preflight(case, error, fault):
    failed = {"outcome": "error", "error": error}
    probe.assert_case_outcome(case, failed, fault, False, False)
    for altered in ((None, False, False), (fault, True, False), (fault, False, True)):
        with pytest.raises(RuntimeError):
            probe.assert_case_outcome(case, failed, *altered)


@pytest.mark.parametrize("case", ["immediate", "delay_100ms"])
def test_timely_ack_requires_the_adapter_to_finish_verification(case):
    probe.assert_case_outcome(case, {"outcome": "returned"}, None, True, True)
    with pytest.raises(RuntimeError):
        probe.assert_case_outcome(case, {"outcome": "returned"}, None, True, False)


async def test_full_read_wire_reply_finishes_before_pending_stop_reply():
    peer = probe.ApplicationPeer(delay_s=.1)
    async with serve(peer.handler, "127.0.0.1", 0) as server:
        port = probe.owned_port([sock.getsockname() for sock in server.sockets])
        try:
            async with connect(f"ws://127.0.0.1:{port}") as connection:
                await connection.send(request())
                await asyncio.wait_for(peer.stop_received.wait(), 1)
                await connection.send(request(id=2, method="get_kin_data"))
                read = json.loads(await asyncio.wait_for(connection.recv(), 1))
                stop = json.loads(await asyncio.wait_for(connection.recv(), 1))
                assert read["id"] == 2 and read["result"] == probe.fixture_result("get_kin_data")
                assert stop == {"jsonrpc": "2.0", "id": 1, "result": {}}
            assert peer.records[1]["send_completed_ns"] < peer.records[0]["send_started_ns"]
            assert not peer.errors
        finally:
            await peer.close()
