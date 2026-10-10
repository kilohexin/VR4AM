"""Loopback-only SDK stop-reply probe. Never a real-robot test entry."""

import argparse
import asyncio
import hashlib
import importlib
import importlib.metadata
import json
from pathlib import Path
import statistics
import time


HOST = "127.0.0.1"
METHODS = {"stop_move", "get_robot_state", "get_running_motion"}
FIELD_SDK_SHA256 = "ad2a68619b25d627593bd5f13f83f17784e7a2c10aa9615db8bd1b681f9fd03b"
CASES = (("immediate", 0.0, False), ("delay_100ms", 0.1, False),
         ("delay_600ms", 0.6, False), ("rpc_error", 0.0, True))


def owned_port(addresses):
    if len(addresses) != 1:
        raise ValueError("exactly one owned IPv4 loopback listener required")
    host, port = addresses[0]
    if host != HOST or type(port) is not int or not 0 < port <= 65535:
        raise ValueError("nonlocal or invalid listener refused")
    return port


def parse_request(raw):
    request = json.loads(raw)
    if (not isinstance(request, dict) or request.get("jsonrpc") != "2.0"
            or type(request.get("id")) not in (int, str)
            or not isinstance(request.get("method"), str)
            or request.get("method") not in METHODS
            or request.get("params") != [{}]):
        raise ValueError("unexpected loopback RPC request")
    return request


class LoopbackPeer:
    def __init__(self, *, delay_s=0.0, reply_error=False):
        self.delay_s = delay_s
        self.reply_error = reply_error
        self.stop_received = asyncio.Event()
        self.records = []
        self.errors = []
        self._connections = set()
        self._tasks = set()
        self._ids = set()

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
                    self.stop_received.set()
                task = asyncio.create_task(self._reply(connection, record))
                self._tasks.add(task)
                task.add_done_callback(self._reply_done)
        finally:
            self._connections.discard(connection)

    def _reply_done(self, task):
        self._tasks.discard(task)
        if not task.cancelled() and task.exception() is not None:
            self.errors.append(type(task.exception()).__name__)

    async def _reply(self, connection, record):
        method = record["method"]
        if method == "stop_move" and self.delay_s:
            await asyncio.sleep(self.delay_s)
        reply = {"jsonrpc": "2.0", "id": record["id"]}
        if method == "stop_move" and self.reply_error:
            reply["error"] = {"code": -32001, "message": "simulated stop failure"}
        else:
            reply["result"] = ({"state": "IDLE"} if method == "get_robot_state" else
                               {"id": 0} if method == "get_running_motion" else {})
        record["send_started_ns"] = time.perf_counter_ns()
        await connection.send(json.dumps(reply))
        record["send_completed_ns"] = time.perf_counter_ns()

    async def close(self):
        for connection in tuple(self._connections):
            await connection.close()
        tasks = tuple(self._tasks)
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


async def _sdk_call(call):
    record = {"started_ns": time.perf_counter_ns()}
    try:
        record["value"] = await call()
        record["outcome"] = "returned"
    except Exception as error:
        record["outcome"] = "error"
        record["error"] = str(error)
    finally:
        record["completed_ns"] = time.perf_counter_ns()
    return record


async def _one_case(sdk, name, delay_s, reply_error):
    from websockets.asyncio.server import serve

    peer = LoopbackPeer(delay_s=delay_s, reply_error=reply_error)
    stop = None
    async with serve(peer.handler, HOST, 0) as server:
        port = owned_port([sock.getsockname() for sock in server.sockets])
        try:
            # Port comes only from our listener, never from CLI/config/DNS.
            client = await asyncio.wait_for(sdk.connect(HOST, port, {"request_timeout": 2}), 3)
            if not await client.is_connected():
                raise RuntimeError("loopback SDK connection not established")
            stop = asyncio.create_task(_sdk_call(client.stop_move))
            await asyncio.wait_for(peer.stop_received.wait(), 2)
            reads = []
            if delay_s:
                for _ in range(3):
                    for method, expected in (("get_robot_state", "IDLE"), ("get_running_motion", 0)):
                        read = await asyncio.wait_for(_sdk_call(getattr(client, method)), 2)
                        if read.get("outcome") != "returned" or read.get("value") != expected:
                            raise RuntimeError("invalid loopback SDK read result")
                        reads.append({"method": method, **read})
            stop_result = await asyncio.wait_for(stop, 3)
            if stop_result["outcome"] != ("error" if reply_error else "returned"):
                raise RuntimeError("unexpected SDK stop outcome")
            if reply_error and ("-32001" not in stop_result.get("error", "")
                                or "simulated stop failure" not in stop_result.get("error", "")):
                raise RuntimeError("SDK did not report the simulated RPC error")
            await peer.close()
            await asyncio.wait_for(client.wait_disconnect(), 2)
            if await client.is_connected() or peer.errors:
                raise RuntimeError("loopback close or protocol verification failed")
            stop_records = [r for r in peer.records if r["method"] == "stop_move"]
            if len(stop_records) != 1 or len(peer.records) != 1 + len(reads):
                raise RuntimeError("unexpected RPC counts")
            rpc = stop_records[0]
            elapsed = stop_result["completed_ns"] - stop_result["started_ns"]
            return {
                "case": name, "requested_server_delay_ms": delay_s * 1000,
                "listener": {"host": HOST, "port": port}, "sdk_stop": stop_result,
                "server_rpc_records": peer.records, "sdk_reads": reads,
                "stop_sdk_elapsed_ms": elapsed / 1e6,
                "server_reply_wait_ms": (rpc["send_started_ns"] - rpc["received_ns"]) / 1e6,
                "server_send_complete_to_sdk_return_ms": (
                    stop_result["completed_ns"] - rpc["send_completed_ns"]) / 1e6,
                "reads_completed_before_stop_reply": sum(
                    r["completed_ns"] < rpc["send_started_ns"] for r in reads),
                "local_connection_closed": True,
            }
        finally:
            await peer.close()
            if stop is not None and not stop.done():
                stop.cancel()
                await asyncio.gather(stop, return_exceptions=True)


def _sdk_identity():
    sdk = importlib.import_module("lebai_sdk")
    binary = Path(importlib.import_module("lebai_sdk.lebai_sdk").__file__)
    digest = hashlib.sha256(binary.read_bytes()).hexdigest()
    return sdk, {"distribution_version": importlib.metadata.version("lebai-sdk-asyncio"),
                 "binary_sha256": digest, "matches_field_binary": digest == FIELD_SDK_SHA256,
                 "websockets_version": importlib.metadata.version("websockets")}


async def run_matrix(repeats):
    if type(repeats) is not int or not 1 <= repeats <= 10:
        raise ValueError("repeats must be 1..10")
    sdk, identity = _sdk_identity()
    sdk.init()  # Runtime initialization only; no discover_devices call.
    results = []
    for _ in range(repeats):
        for case in CASES:
            results.append(await asyncio.wait_for(_one_case(sdk, *case), 8))
    summary = {}
    for name, _delay, _error in CASES:
        rows = [row for row in results if row["case"] == name]
        values = [row["stop_sdk_elapsed_ms"] for row in rows]
        summary[name] = {"runs": len(rows), "sdk_elapsed_ms": {
            "min": min(values), "median": statistics.median(values), "max": max(values)},
            "reads_before_reply": [row["reads_completed_before_stop_reply"] for row in rows]}
    return {"schema": 1, "offline_only": True, "hardware_stop_verified": False,
            "application_stop_path_exercised": False, "real_robot_address_used": False,
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
