"""Test-only observer integration with the real WebSocket session supervisor."""

import asyncio
from contextlib import asynccontextmanager
from types import SimpleNamespace

import pytest
from fastapi import WebSocketDisconnect

from app.api import teleop_ws
from app.robots.base import BackendCommandError, StopReason
from app.schemas.messages import TeleopMode
from tests.diagnostics.test_shared_observer import SharedObserverProbe
from tests.robots.test_snapshot_contention import delayed_adapter


class ProbeSocket:
    """In-memory socket. Its display payload is explicitly not a wire proposal."""

    def __init__(self, state):
        self.app = SimpleNamespace(state=state)
        self.incoming = asyncio.Queue()
        self.incoming.put_nowait({"v": 1, "type": "hello", "request_id": "offline"})
        self.sent = []
        self.closed = []
        self.sending = asyncio.Event()
        self.release_send = asyncio.Event()
        self.receiver_exited = asyncio.Event()
        self.send_error = None

    async def accept(self):
        pass

    async def close(self, **kwargs):
        self.closed.append(kwargs)

    async def receive_json(self):
        try:
            value = await self.incoming.get()
            if isinstance(value, BaseException):
                raise value
            return value
        except BaseException:
            self.receiver_exited.set()
            raise

    async def send_json(self, payload):
        self.sent.append(payload)
        if payload["type"] == "offline_display_probe":
            self.sending.set()
            await self.release_send.wait()
            if self.send_error is not None:
                raise self.send_error


class SessionProbe:
    def __init__(self, adapter, clock):
        self.observer = SharedObserverProbe(adapter, clock)
        self.control = self.observer.control
        self.state = SimpleNamespace(
            control=self.control, latest=self.control.latest,
            teleop_sender_tasks=set(), teleop_owner=None,
            teleop_owner_lock=asyncio.Lock(),
        )
        self.socket = ProbeSocket(self.state)
        self.children = []
        self.joined = asyncio.Event()
        self.cleanup_entered = asyncio.Event()
        self.release_cleanup = asyncio.Event()
        self.cleanup_count = 0
        self.original_disconnect = self.control.on_disconnect
        self.control.on_disconnect = self.disconnect
        self.session = None

    async def disconnect(self):
        self.cleanup_count += 1
        assert self.children and all(t.done() for t in self.children)
        assert self.state.teleop_sender_tasks == set()
        assert self.state.teleop_owner is not None
        self.cleanup_entered.set()
        await self.release_cleanup.wait()
        await self.original_disconnect()

    async def delayed_sender(self, websocket, control, start_sender, send_lock):
        """Experimental two-task sender; outer supervisor remains unmodified."""
        await start_sender.wait()
        self.joined.clear()
        self.observer.latest = control.latest
        first_cycle = len(self.observer.gate.delays) + 1
        observation = self.observer.start()

        async def display():
            await self.observer.gate.wait(first_cycle)
            # Model one blocked send without proposing a new UI wire schema.
            self.observer.compose()
            async with send_lock:
                await websocket.send_json({"type": "offline_display_probe"})
            await asyncio.Event().wait()

        consumer = asyncio.create_task(display())
        self.children = [observation, consumer]
        primary = None
        try:
            await asyncio.wait(self.children, return_when=asyncio.FIRST_COMPLETED)
        except BaseException as error:
            primary = error
        finally:
            self.observer.request_close()
            consumer.cancel()
            results = await asyncio.gather(*self.children, return_exceptions=True)
            self.observer.task = None
            self.joined.set()
        if primary is not None:
            raise primary
        for result in results:
            if isinstance(result, BaseException) and not isinstance(result, asyncio.CancelledError):
                raise result

    async def start(self):
        self.session = asyncio.create_task(teleop_ws.teleop_websocket(self.socket))
        await asyncio.wait_for(self.socket.sending.wait(), 1)

    async def assert_competing_connection_rejected(self):
        owner = self.state.teleop_owner
        intruder = ProbeSocket(self.state)
        await teleop_ws.teleop_websocket(intruder)
        assert intruder.sent[0]["reason"] == "controller_occupied"
        assert intruder.closed[0]["code"] == 4409
        assert self.state.teleop_owner is owner

    def assert_finished(self, previous_latest):
        assert self.joined.is_set()
        assert all(t.done() for t in self.children)
        assert self.socket.receiver_exited.is_set()
        assert self.cleanup_count == 1
        assert self.state.teleop_sender_tasks == set()
        assert self.state.teleop_owner is None
        assert self.state.latest is self.control.latest
        assert self.state.latest is not previous_latest
        assert self.state.latest.snapshot() is None
        assert self.observer.compose().status == "disconnected"


@asynccontextmanager
async def session_probe(monkeypatch):
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        probe = SessionProbe(adapter, clock)
        probe.stop_attempts = []
        original_stop = adapter.stop

        async def tracked_stop(reason):
            probe.stop_attempts.append(reason)
            await original_stop(reason)

        adapter.stop = tracked_stop
        monkeypatch.setattr(teleop_ws, "_delayed_state_sender", probe.delayed_sender)
        try:
            yield probe, adapter, client
        finally:
            probe.release_cleanup.set()
            if probe.session is not None:
                if not probe.session.done():
                    probe.session.cancel()
                await asyncio.wait_for(asyncio.gather(probe.session, return_exceptions=True), 2)


@pytest.mark.parametrize("failure", ["read", "record", "send"])
async def test_task_failure_joins_peers_then_disconnects_before_releasing_owner(monkeypatch, failure):
    async with session_probe(monkeypatch) as (probe, adapter, client):
        previous_latest = probe.state.latest
        await probe.start()

        async def fail():
            raise RuntimeError(f"offline_{failure}_failed")

        if failure == "read":
            adapter.get_state = fail
        elif failure == "record":
            probe.observer.recorder.on_write = fail
        else:
            probe.socket.send_error = RuntimeError("offline_send_failed")
            probe.socket.release_send.set()
        if failure != "send":
            probe.observer.gate.tokens.put_nowait(None)

        await asyncio.wait_for(probe.cleanup_entered.wait(), 1)
        assert probe.joined.is_set()
        assert not probe.session.done()
        assert client.write_calls == []  # Cleanup is deliberately paused.
        await probe.assert_competing_connection_rejected()
        probe.release_cleanup.set()
        with pytest.raises(RuntimeError, match=f"offline_{failure}_failed"):
            await asyncio.wait_for(probe.session, 1)
        probe.assert_finished(previous_latest)
        assert client.write_calls == [("stop_move",)]
        assert probe.control.mode is TeleopMode.DISCONNECTED


@pytest.mark.parametrize("exit_kind", ["receiver_close", "session_cancel", "observer_cancel"])
async def test_session_exit_cleans_observer_and_blocked_display(monkeypatch, exit_kind):
    async with session_probe(monkeypatch) as (probe, adapter, client):
        previous_latest = probe.state.latest
        await probe.start()
        if exit_kind == "receiver_close":
            probe.socket.incoming.put_nowait(WebSocketDisconnect())
        elif exit_kind == "session_cancel":
            probe.session.cancel()
        else:
            probe.observer.task.cancel()
        await asyncio.wait_for(probe.cleanup_entered.wait(), 1)
        await probe.assert_competing_connection_rejected()
        probe.release_cleanup.set()
        if exit_kind == "session_cancel":
            with pytest.raises(asyncio.CancelledError):
                await asyncio.wait_for(probe.session, 1)
        else:
            await asyncio.wait_for(probe.session, 1)
        probe.assert_finished(previous_latest)
        assert probe.control.mode is TeleopMode.DISCONNECTED
        assert client.write_calls == [("stop_move",)]


@pytest.mark.parametrize("primary_failure", [False, True])
async def test_disconnect_stop_failure_keeps_latch_and_preserves_primary_error(monkeypatch, primary_failure):
    async with session_probe(monkeypatch) as (probe, adapter, client):
        previous_latest = probe.state.latest
        await probe.start()
        stop_attempts = []

        async def failed_stop(reason):
            stop_attempts.append(reason)
            raise BackendCommandError("offline_disconnect_stop_failed")

        adapter.stop = failed_stop
        if primary_failure:
            probe.socket.send_error = RuntimeError("offline_send_failed")
            probe.socket.release_send.set()
        else:
            probe.socket.incoming.put_nowait(WebSocketDisconnect())
        await asyncio.wait_for(probe.cleanup_entered.wait(), 1)
        probe.release_cleanup.set()
        expected = "offline_send_failed" if primary_failure else "offline_disconnect_stop_failed"
        with pytest.raises(RuntimeError, match=expected):
            await asyncio.wait_for(probe.session, 1)
        probe.assert_finished(previous_latest)
        assert len(stop_attempts) == 1
        assert probe.control._fault == "stop_unverified"
        assert probe.control._stop_unverified is True
        assert probe.control.mode is TeleopMode.FAULT
        assert client.write_calls == []


@pytest.mark.parametrize("trigger", ["receiver_close", "send_failure"])
async def test_late_observation_is_joined_before_disconnect_and_owner_handoff(monkeypatch, trigger):
    async with session_probe(monkeypatch) as (probe, adapter, client):
        previous_latest = probe.state.latest
        await probe.start()
        entered, cancelled, release = asyncio.Event(), asyncio.Event(), asyncio.Event()
        original = client.get_kin_data

        async def resistant_read():
            entered.set()
            try:
                await release.wait()
            except asyncio.CancelledError:
                cancelled.set()
                await release.wait()
            return await original()

        client.get_kin_data = resistant_read
        probe.observer.gate.tokens.put_nowait(None)
        try:
            await asyncio.wait_for(entered.wait(), 1)
            generation = probe.observer.generation
            if trigger == "receiver_close":
                probe.socket.incoming.put_nowait(WebSocketDisconnect())
            else:
                probe.socket.send_error = RuntimeError("offline_send_failed")
                probe.socket.release_send.set()
            await asyncio.wait_for(cancelled.wait(), 1)
            assert not probe.joined.is_set()
            assert not probe.cleanup_entered.is_set()
            assert probe.state.teleop_owner is not None
            assert probe.observer.compose().status == "disconnected"
            assert client.write_calls == []
            await probe.assert_competing_connection_rejected()
            release.set()
            await asyncio.wait_for(probe.cleanup_entered.wait(), 1)
            assert (generation, False) in probe.observer.publications
            assert len(probe.observer.recorder.rows) == 2
            await probe.assert_competing_connection_rejected()
            probe.release_cleanup.set()
            if trigger == "send_failure":
                with pytest.raises(RuntimeError, match="offline_send_failed"):
                    await asyncio.wait_for(probe.session, 1)
            else:
                await asyncio.wait_for(probe.session, 1)
            probe.assert_finished(previous_latest)
            assert client.write_calls == [("stop_move",)]
        finally:
            release.set()


async def test_new_connection_is_accepted_only_after_prior_session_cleanup(monkeypatch):
    async with session_probe(monkeypatch) as (probe, adapter, client):
        await probe.start()
        probe.socket.incoming.put_nowait(WebSocketDisconnect())
        await asyncio.wait_for(probe.cleanup_entered.wait(), 1)
        await probe.assert_competing_connection_rejected()
        probe.release_cleanup.set()
        await asyncio.wait_for(probe.session, 1)

        # Reuse the same app/control through the real endpoint. The second
        # receiver closes after hello; this is a new session, not a fake reset.
        old_token = probe.observer.generation
        second = ProbeSocket(probe.state)
        new_session = asyncio.create_task(teleop_ws.teleop_websocket(second))
        try:
            await asyncio.wait_for(second.sending.wait(), 1)
            assert second.sent[0]["type"] == "hello_ack"
            assert probe.state.teleop_owner is not None
            assert probe.observer.generation != old_token
            second.incoming.put_nowait(WebSocketDisconnect())
            await asyncio.wait_for(new_session, 1)
            assert probe.cleanup_count == 2
            assert probe.state.teleop_owner is None
            assert probe.state.teleop_sender_tasks == set()
            assert all(t.done() for t in probe.children)
            assert probe.stop_attempts == [StopReason.DISCONNECT, StopReason.DISCONNECT]
            # No new movement: the existing adapter stop transaction reuses its
            # completed RPC. Both session cleanups still invoke backend.stop.
            assert client.write_calls == [("stop_move",)]
        finally:
            if not new_session.done():
                new_session.cancel()
            await asyncio.gather(new_session, return_exceptions=True)


@pytest.mark.parametrize("failure", ["read", "record"])
async def test_first_observation_failure_cancels_display_waiting_for_initial_sample(monkeypatch, failure):
    async with session_probe(monkeypatch) as (probe, adapter, client):
        previous_latest = probe.state.latest

        async def fail():
            raise RuntimeError("offline_initial_observation_failed")

        if failure == "read":
            adapter.get_state = fail
        else:
            probe.observer.recorder.on_write = fail
        probe.session = asyncio.create_task(teleop_ws.teleop_websocket(probe.socket))
        await asyncio.wait_for(probe.cleanup_entered.wait(), 1)
        assert not probe.socket.sending.is_set()
        assert probe.observer.compose().status == "disconnected"
        probe.release_cleanup.set()
        with pytest.raises(RuntimeError, match="offline_initial_observation_failed"):
            await asyncio.wait_for(probe.session, 1)
        probe.assert_finished(previous_latest)
        assert probe.stop_attempts == [StopReason.DISCONNECT]


async def test_simultaneous_observer_and_display_errors_are_joined_with_observer_primary(monkeypatch):
    async with session_probe(monkeypatch) as (probe, adapter, client):
        release = asyncio.Event()
        record_entered = asyncio.Event()
        probe.socket.release_send = release
        probe.socket.send_error = RuntimeError("offline_send_failed")
        previous_latest = probe.state.latest
        await probe.start()

        async def fail_record():
            record_entered.set()
            await release.wait()
            raise RuntimeError("offline_record_failed")

        probe.observer.recorder.on_write = fail_record
        probe.observer.gate.tokens.put_nowait(None)
        await asyncio.wait_for(record_entered.wait(), 1)
        release.set()
        await asyncio.wait_for(probe.cleanup_entered.wait(), 1)
        probe.release_cleanup.set()
        with pytest.raises(RuntimeError, match="offline_record_failed"):
            await asyncio.wait_for(probe.session, 1)
        probe.assert_finished(previous_latest)
        assert [str(t.exception()) for t in probe.children] == [
            "offline_record_failed", "offline_send_failed",
        ]
