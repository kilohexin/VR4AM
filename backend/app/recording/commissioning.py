from __future__ import annotations

import asyncio
import json
import re
import shutil
from collections import deque
from datetime import UTC, datetime
from pathlib import Path
from typing import Any
from uuid import uuid4


class RecorderUnavailable(RuntimeError):
    pass


class CommissioningRecorder:
    def __init__(
        self,
        root: Path,
        metadata: dict[str, object],
        *,
        session_id: str | None = None,
        capacity: int = 2048,
        critical_capacity: int = 128,
        template_path: Path | None = None,
    ) -> None:
        if capacity <= 0 or critical_capacity <= 0:
            raise ValueError("recorder_capacity_must_be_positive")
        self.root = Path(root)
        self.metadata = dict(metadata)
        self.session_id = session_id or _default_session_id()
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}", self.session_id):
            raise ValueError("invalid_session_id")
        self.capacity = capacity
        self.critical_capacity = critical_capacity
        self.template_path = template_path or (
            Path(__file__).resolve().parents[3]
            / "docs"
            / "real-robot-commissioning-report.md"
        )
        self._normal: deque[dict[str, object]] = deque()
        self._critical: deque[dict[str, object]] = deque()
        self._wake = asyncio.Event()
        self._writer_task: asyncio.Task[None] | None = None
        self._started = False
        self._closing = False
        self._normal_written = 0
        self._critical_written = 0
        self.dropped_normal_events = 0
        self.fatal_error: str | None = None
        self.session_dir = self.root / self.session_id
        self.jsonl_path = self.session_dir / "session.jsonl"

    async def start(self) -> None:
        if self._started:
            return
        if self.fatal_error is not None:
            raise RecorderUnavailable(self.fatal_error)
        try:
            await asyncio.to_thread(self._prepare_session)
            await asyncio.to_thread(
                self._append_batch,
                [
                    self._make_entry(
                        "session_started",
                        {"metadata": self.metadata},
                        0,
                        critical=True,
                    )
                ],
            )
        except RecorderUnavailable:
            raise
        except Exception:
            self.fatal_error = "log_directory_unavailable"
            raise RecorderUnavailable(self.fatal_error) from None
        self._critical_written = 1
        self._started = True
        self._writer_task = asyncio.create_task(
            self._writer(),
            name="commissioning-jsonl-writer",
        )
        if self._critical or self._normal:
            self._wake.set()

    async def write_vr_frame(
        self,
        frame: object,
        server_mono_ns: int,
    ) -> None:
        payload = _serializable_payload(frame)
        await self.write_event(
            {"kind": "vr_frame", "frame": payload},
            server_mono_ns,
        )

    async def write_robot_state(
        self,
        state: object,
        server_mono_ns: int,
    ) -> None:
        payload = _serializable_payload(state)
        await self.write_event(
            {"kind": "robot_state_sample", "state": payload},
            server_mono_ns,
        )

    async def write_event(
        self,
        event: object,
        server_mono_ns: int,
    ) -> None:
        self._raise_if_unavailable()
        payload = _serializable_payload(event)
        if isinstance(payload, dict):
            kind = str(payload.get("kind", "event"))
            fields = {key: value for key, value in payload.items() if key != "kind"}
        else:
            kind = "event"
            fields = {"payload": payload}
        entry = self._make_entry(
            kind,
            fields,
            server_mono_ns,
            critical=False,
        )
        if len(self._normal) >= self.capacity:
            self.dropped_normal_events += 1
            return
        self._normal.append(entry)
        if self._started:
            self._wake.set()

    async def write_critical_event(
        self,
        kind: str,
        payload: object,
        server_mono_ns: int,
    ) -> None:
        self._raise_if_unavailable()
        fields = _serializable_payload(payload)
        if not isinstance(fields, dict):
            fields = {"payload": fields}
        entry = self._make_entry(
            kind,
            fields,
            server_mono_ns,
            critical=True,
        )
        if len(self._critical) >= self.critical_capacity:
            self.fatal_error = "critical_log_queue_full"
            raise RecorderUnavailable(self.fatal_error)
        self._critical.append(entry)
        if self._started:
            self._wake.set()

    async def write_camera_frame(self, frame: object) -> None:
        return None

    async def close(self) -> None:
        if not self._started:
            return
        if not self._closing:
            await self.write_critical_event(
                "session_ended",
                {},
                0,
            )
            self._closing = True
            self._wake.set()
        task, self._writer_task = self._writer_task, None
        if task is not None:
            await task
        await asyncio.to_thread(self._write_summary)
        self._started = False
        if self.fatal_error is not None:
            raise RecorderUnavailable(self.fatal_error)

    async def _writer(self) -> None:
        try:
            while True:
                await self._wake.wait()
                while self._critical or self._normal:
                    if self._critical:
                        batch = [self._critical.popleft()]
                        critical_count = 1
                    else:
                        batch = [
                            self._normal.popleft()
                            for _ in range(min(20, len(self._normal)))
                        ]
                        critical_count = 0
                    await asyncio.to_thread(self._append_batch, batch)
                    self._critical_written += critical_count
                    self._normal_written += len(batch) - critical_count
                self._wake.clear()
                if self._closing:
                    return
                if self._critical or self._normal:
                    self._wake.set()
        except Exception:
            self.fatal_error = "log_write_failed"
            self._normal.clear()
            self._critical.clear()

    def _prepare_session(self) -> None:
        self.session_dir.mkdir(parents=True, exist_ok=False)
        self.jsonl_path.touch(exist_ok=False)
        destination = self.session_dir / "commissioning-report.md"
        if self.template_path.exists():
            shutil.copyfile(self.template_path, destination)
        else:
            destination.write_text(
                "# Real Robot Commissioning Report\n",
                encoding="utf-8",
            )

    def _append_batch(self, batch: list[dict[str, object]]) -> None:
        with self.jsonl_path.open("a", encoding="utf-8", newline="\n") as file:
            for entry in batch:
                file.write(
                    json.dumps(
                        entry,
                        ensure_ascii=False,
                        allow_nan=False,
                        separators=(",", ":"),
                    )
                    + "\n"
                )
            file.flush()

    def _write_summary(self) -> None:
        summary = {
            "session_id": self.session_id,
            "normal_events": self._normal_written,
            "critical_events": self._critical_written,
            "dropped_normal_events": self.dropped_normal_events,
            "fatal_error": self.fatal_error,
        }
        temporary = self.session_dir / "summary.json.tmp"
        destination = self.session_dir / "summary.json"
        temporary.write_text(
            json.dumps(
                summary,
                ensure_ascii=False,
                allow_nan=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        temporary.replace(destination)

    def _make_entry(
        self,
        kind: str,
        payload: dict[str, object],
        server_mono_ns: int,
        *,
        critical: bool,
    ) -> dict[str, object]:
        entry = {
            "kind": kind,
            "server_mono_ns": int(server_mono_ns),
            "critical": critical,
            **payload,
        }
        try:
            json.dumps(entry, allow_nan=False)
        except (TypeError, ValueError, OverflowError):
            raise RecorderUnavailable("event_not_json_serializable") from None
        return entry

    def _raise_if_unavailable(self) -> None:
        if self.fatal_error is not None:
            raise RecorderUnavailable(self.fatal_error)
        if self._closing:
            raise RecorderUnavailable("recorder_closed")


def _serializable_payload(value: object) -> Any:
    if hasattr(value, "model_dump"):
        return value.model_dump(mode="json")
    return value


def _default_session_id() -> str:
    timestamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    return f"{timestamp}-{uuid4().hex[:8]}"
