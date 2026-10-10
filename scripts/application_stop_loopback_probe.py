"""Loopback-only production adapter stop probe; never a live robot entry."""

import argparse
import asyncio
import copy
import json
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "backend"))

from app.config import (LebaiSettings, LebaiControlSettings, LebaiGripperSettings, TcpExpectation)
from app.robots.base import StopReason
from app.robots.lebai_adapter import RealLebaiAdapter
from scripts.sdk_loopback_stop_probe import HOST, LoopbackPeer, owned_port, _sdk_identity, _sdk_call

METHODS = {"stop_move", "get_robot_state", "get_estop_reason", "get_kin_data",
           "get_tcp", "get_claw", "get_running_motion"}
CASES = (("immediate", 0.0, 0.0, False), ("delay_100ms", 0.1, 0.0, False),
         ("delay_600ms_idle", 0.6, 0.0, False), ("rpc_error", 0.0, 0.0, True),
         ("immediate_ack_moving_600ms", 0.0, 0.6, False))


def parse_request(raw):
    request = json.loads(raw)
    if (not isinstance(request, dict) or request.get("jsonrpc") != "2.0"
            or type(request.get("id")) not in (int, str)
            or not isinstance(request.get("method"), str) or request["method"] not in METHODS
            or request.get("params") != [{}]):
        raise ValueError("unexpected application loopback RPC request")
    return request


def _wire_pose(x, y, z):
    return {"position": {"x": x, "y": y, "z": z},
            "rotation": {"euler_zyx": {"x": 0.0, "y": 0.0, "z": 0.0}}}


def fixture_result(method, moving=False):
    if method == "stop_move":
        return {}
    if method == "get_robot_state":
        return {"state": "MOVING" if moving else "IDLE"}
    if method == "get_estop_reason":
        return {"reason": "NONE"}
    if method == "get_tcp":
        return _wire_pose(0.0, 0.0, 0.175)
    if method == "get_claw":
        return {"force": 0, "amplitude": 100}
    if method == "get_running_motion":
        return {"id": 0}
    if method == "get_kin_data":
        kin = {}
        for prefix in ("actual", "target"):
            kin[f"{prefix}_joint_pose"] = [0.0, -1.0, 1.0, 0.0, 1.57, 0.0]
            for field in ("speed", "acc", "torque"):
                kin[f"{prefix}_joint_{field}"] = [0.0] * 6
            kin[f"{prefix}_tcp_pose"] = _wire_pose(0.3, 0.0, 0.4)
        kin["actual_flange_pose"] = _wire_pose(0.3, 0.0, 0.4)
        return kin
    raise ValueError("unknown fixture method")


def loopback_settings():
    # Synthetic constants only. Never load Settings or a field configuration.
    return LebaiSettings(
        mode="control", ip=HOST, expected_tcp=TcpExpectation(0, 0, .175, 0, 0, 0),
        home_q=(0, -1, 1, 0, 1.57, 0), teleop_ready_q=(0, -1, 1, 0, .2, 0),
        soft_joint_min_rad=(-3, -2.5, -2.5, -3, -2.5, -6),
        soft_joint_max_rad=(3, 2.5, 2.5, 3, 2.5, 6), joint_limit_margin_rad=.05,
        startup_tcp_min_m=(.1, -.3, .1), startup_tcp_max_m=(.5, .3, .6),
        tcp_position_tolerance_m=.001, tcp_rotation_tolerance_deg=.5,
        control=LebaiControlSettings(
            loop_hz=50, state_hz=25, pvat_send_hz=25, pvat_horizon_s=.08,
            max_tcp_speed_mps=.03, max_tcp_rotation_radps=.25,
            max_tcp_acceleration_mps2=.1, max_tcp_angular_acceleration_radps2=.5,
            max_joint_speed_radps=.15, max_joint_acceleration_radps2=.5,
            max_joint_step_rad=.05, max_joint_tracking_error_rad=.25,
            max_tcp_step_m=.002, max_tcp_rotation_step_deg=1,
            max_relative_translation_m=.1, max_relative_rotation_deg=30, translation_scale=.5),
        gripper=LebaiGripperSettings(30, 10, 100, 0))


class ApplicationPeer(LoopbackPeer):
    def __init__(self, *, delay_s=0.0, moving_s=0.0, reply_error=False):
        super().__init__(delay_s=delay_s, reply_error=reply_error)
        self.moving_s = moving_s
        self.stop_received_ns = None

    async def handler(self, connection):
        self._connections.add(connection)
        try:
            async for raw in connection:
                try:
                    request = parse_request(raw)
                    key = (type(request["id"]), request["id"])
                    if key in self._ids:
                        raise ValueError("duplicate RPC id")
                    self._ids.add(key)
                except ValueError as error:
                    self.errors.append(str(error))
                    await connection.close(code=1008, reason="unexpected probe request")
                    return
                record = {"id": request["id"], "method": request["method"],
                          "received_ns": time.perf_counter_ns()}
                self.records.append(record)
                if request["method"] == "stop_move":
                    self.stop_received_ns = record["received_ns"]
                    self.stop_received.set()
                task = asyncio.create_task(self._reply(connection, record))
                self._tasks.add(task)
                task.add_done_callback(self._reply_done)
        finally:
            self._connections.discard(connection)

    async def _reply(self, connection, record):
        if record["method"] == "stop_move" and self.delay_s:
            await asyncio.sleep(self.delay_s)
        moving = (self.stop_received_ns is not None
                  and time.perf_counter_ns() - self.stop_received_ns < self.moving_s * 1e9)
        reply = {"jsonrpc": "2.0", "id": record["id"]}
        if record["method"] == "stop_move" and self.reply_error:
            reply["error"] = {"code": -32001, "message": "simulated stop failure"}
        else:
            reply["result"] = fixture_result(record["method"], moving)
        record["simulated_moving"] = moving
        record["send_started_ns"] = time.perf_counter_ns()
        await connection.send(json.dumps(reply))
        record["send_completed_ns"] = time.perf_counter_ns()


async def connect_owned(sdk, host, port):
    # No target is accepted through CLI/config/environment/DNS.
    if host != HOST:
        raise ValueError("nonlocal SDK target refused")
    owned_port([(host, port)])
    return await asyncio.wait_for(sdk.connect(HOST, port, {"request_timeout": 2}), 3)


def assert_case_outcome(name, stop, final_fault, ready, confirmed):
    _, delay_s, moving_s, reply_error = next(case for case in CASES if case[0] == name)
    if delay_s > .2 or reply_error:
        expected = "sdk_call_failed:stop_move" if reply_error else "sdk_timeout:stop_move"
        if stop.get("error") != expected or final_fault != "stop_unverified" or ready or confirmed:
            raise RuntimeError("failed stop incorrectly accepted")
    elif moving_s > .5:
        if stop.get("error") != "stop_incomplete" or final_fault != "stop_incomplete" or ready or confirmed:
            raise RuntimeError("moving after acknowledgement incorrectly accepted")
    elif stop["outcome"] != "returned" or final_fault is not None or not ready or not confirmed:
        raise RuntimeError("timely simulated stop failed")


async def one_case(sdk, name, delay_s, moving_s, reply_error):
    from websockets.asyncio.server import serve

    peer = ApplicationPeer(delay_s=delay_s, moving_s=moving_s, reply_error=reply_error)
    events, adapter, client = [], None, None

    async def record(event, timestamp):
        events.append({**copy.deepcopy(event), "callback_ns": time.perf_counter_ns(),
                       "event_timestamp_ns": timestamp})

    async with serve(peer.handler, HOST, 0) as server:
        port = owned_port([sock.getsockname() for sock in server.sockets])
        async def factory(host):
            nonlocal client
            client = await connect_owned(sdk, host, port)
            return client
        try:
            adapter = RealLebaiAdapter(loopback_settings(), client_factory=factory,
                                       clock=time.perf_counter_ns, event_callback=record)
            await asyncio.wait_for(adapter.connect(), 4)
            if not (await adapter.preflight()).ready:
                raise RuntimeError("synthetic adapter preflight not ready")
            stop = await asyncio.wait_for(_sdk_call(lambda: adapter.stop(StopReason.GRIP_RELEASED)), 4)
            request = adapter._stop_transaction.requests["stop_move"]
            pending_at_stop_exit = not request.task.done()
            # Real native read while a shielded stop may still await its reply.
            state_before_late_reply = await adapter.get_state()
            raw_before_late_reply = adapter._snapshot.raw_robot_state
            read_completed_ns = time.perf_counter_ns()
            await asyncio.wait_for(asyncio.shield(request.task), 3)
            if moving_s:
                remaining = moving_s - (time.perf_counter_ns() - peer.stop_received_ns) / 1e9
                await asyncio.sleep(max(0, remaining) + .02)
            state_after = await adapter.get_state()
            raw_after = adapter._snapshot.raw_robot_state
            preflight = await adapter.preflight()
            assert_case_outcome(name, stop, state_after.fault, preflight.ready,
                                adapter._stop_transaction.confirmed)
            await adapter.disconnect()
            adapter = None
            await peer.close()
            await asyncio.wait_for(client.wait_disconnect(), 2)
            if await client.is_connected() or peer.errors:
                raise RuntimeError("loopback close or protocol verification failed")
            stop_records = [r for r in peer.records if r["method"] == "stop_move"]
            if len(stop_records) != 1:
                raise RuntimeError("unexpected stop RPC count")
            lifecycles = [e for e in events if e["kind"] == "stop_rpc_lifecycle"]
            if len(lifecycles) != 1:
                raise RuntimeError("unexpected stop lifecycle count")
            diagnostics = next(e for e in events if e["kind"] == "stop_diagnostics")
            return {"case": name, "requested_reply_delay_ms": delay_s * 1000,
                    "requested_moving_ms": moving_s * 1000, "listener": {"host": HOST, "port": port},
                    "application_stop": stop, "stop_rpc_lifecycle": lifecycles[0],
                    "stop_diagnostics": diagnostics, "events": events,
                    "stop_rpc_pending_at_application_exit": pending_at_stop_exit,
                    "read_after_application_exit": {"completed_ns": read_completed_ns,
                        "robot_state": state_before_late_reply.robot_state.value,
                        "raw_robot_state": raw_before_late_reply,
                        "fault": state_before_late_reply.fault},
                    "final_fault": state_after.fault, "final_robot_state": state_after.robot_state.value,
                    "final_raw_robot_state": raw_after,
                    "final_preflight_ready": preflight.ready,
                    "server_rpc_records": peer.records, "local_connection_closed": True,
                    "application_stop_elapsed_ms": (stop["completed_ns"] - stop["started_ns"]) / 1e6,
                    "sdk_await_elapsed_ms": (request.sdk_await_completed_ns - request.sdk_await_started_ns) / 1e6}
        finally:
            if adapter is not None:
                await adapter.disconnect()
            await peer.close()


async def run_matrix(repeats):
    if type(repeats) is not int or not 1 <= repeats <= 10:
        raise ValueError("repeats must be 1..10")
    sdk, identity = _sdk_identity()
    sdk.init()
    results = []
    for _ in range(repeats):
        for case in CASES:
            results.append(await asyncio.wait_for(one_case(sdk, *case), 10))
    summary = {}
    for name, *_ in CASES:
        rows = [r for r in results if r["case"] == name]
        summary[name] = {"runs": len(rows)}
        for key in ("application_stop_elapsed_ms", "sdk_await_elapsed_ms"):
            values = [r[key] for r in rows]
            summary[name][key] = {"min": min(values), "median": statistics.median(values), "max": max(values)}
        summary[name]["rpc_outcomes"] = [r["stop_rpc_lifecycle"]["outcome"] for r in rows]
        summary[name]["final_faults"] = [r["final_fault"] for r in rows]
    return {"schema": 1, "offline_only": True, "application_stop_path_exercised": True,
            "hardware_stop_verified": False, "real_robot_address_used": False,
            "clock": "time.perf_counter_ns", "clock_scope": "one local process",
            "sdk_identity": identity, "repeats": repeats, "results": results, "summary": summary}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, choices=range(1, 11), default=3)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error("output exists; choose a new evidence path")
    result = asyncio.run(run_matrix(args.repeats))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps({"sdk_identity": result["sdk_identity"], "summary": result["summary"]}, indent=2))


if __name__ == "__main__":
    main()
