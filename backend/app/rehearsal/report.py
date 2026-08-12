from __future__ import annotations

import asyncio
import hashlib
import json
import math
import subprocess
import uuid
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path


REHEARSAL_PHASES = (
    "identity_preflight",
    "home",
    "arm_and_anchor",
    "translate",
    "rotate",
    "gripper",
    "pick_place",
    "soft_boundary",
    "tracking_loss",
    "recovery_and_home",
    "final_stop",
    "finalize",
)

HARDWARE_PENDING = (
    "sdk_connection",
    "tcp_home_joint_limits",
    "translation_direction",
    "rotation_direction",
    "gripper_direction_force",
    "pvat_tracking_latency",
    "stop_distance_estop",
    "lightweight_grasp_release",
)

_FAKE_RUNTIME = "LEBAI_FAKE"
_MODEL_PATHS = {
    "kinematics_sha256": "config/lm3_visual_kinematics_v1.json",
    "fake_config_sha256": "config/fake-lebai.yaml",
    "glb_sha256": "web/public/models/Lebai_LM3.glb",
}

ProvenanceProvider = Callable[[], Mapping[str, object]]
Clock = Callable[[], datetime]


@dataclass(frozen=True)
class BeginResult:
    run_id: str


@dataclass(frozen=True)
class FinishResult:
    json_path: Path
    markdown_path: Path


@dataclass
class _ActiveRun:
    owner: object
    run_id: str
    plan_version: int
    started_at: str
    provenance: dict[str, object]
    phases: list[dict[str, object]] = field(default_factory=list)


class RehearsalReportStore:
    """Persists Fake-only rehearsal evidence for one active owner at a time."""

    def __init__(
        self,
        *,
        root: Path,
        runtime: str,
        provenance: ProvenanceProvider | None = None,
        now: Clock | None = None,
    ) -> None:
        self.root = Path(root)
        self.runtime = runtime
        self._provenance = provenance or _production_provenance
        self._now = now or (lambda: datetime.now(timezone.utc))
        self._lock = asyncio.Lock()
        self._active_runs: list[_ActiveRun] = []

    async def begin(
        self,
        owner: object,
        plan_version: int,
        state: object,
        diagnostics: object,
    ) -> BeginResult:
        async with self._lock:
            self._require_fake_runtime()
            self._snapshot_context(state, diagnostics)
            if isinstance(plan_version, bool) or not isinstance(plan_version, int):
                raise ValueError("plan_version must be an integer")
            if plan_version < 1:
                raise ValueError("plan_version must be positive")
            if self._find_active_by_owner(owner) is not None:
                raise ValueError("owner already has an active rehearsal run")
            provenance = _mapping_snapshot(
                await _run_blocking(self._provenance),
                "provenance",
            )
            run = _ActiveRun(
                owner=owner,
                run_id=uuid.uuid4().hex,
                plan_version=plan_version,
                started_at=_utc_timestamp(self._now()),
                provenance=provenance,
            )
            self._active_runs.append(run)
            return BeginResult(run_id=run.run_id)

    async def record_phase(
        self,
        owner: object,
        run_id: str,
        result: object,
        state: object,
        diagnostics: object,
    ) -> None:
        async with self._lock:
            run = self._owned_run(owner, run_id)
            result_snapshot = _mapping_snapshot(result, "phase result")
            expected = REHEARSAL_PHASES[len(run.phases)] if len(run.phases) < len(
                REHEARSAL_PHASES
            ) else None
            if result_snapshot.get("phase") != expected:
                raise ValueError("phase order does not match the rehearsal plan")
            _validate_phase_result(result_snapshot)
            state_snapshot, diagnostics_snapshot = self._snapshot_context(
                state, diagnostics
            )
            result_snapshot["state"] = state_snapshot
            result_snapshot["diagnostics"] = diagnostics_snapshot
            run.phases.append(result_snapshot)

    async def finish(
        self,
        owner: object,
        run_id: str,
        outcome: str,
        failure: object,
        state: object,
        diagnostics: object,
    ) -> FinishResult:
        async with self._lock:
            run = self._owned_run(owner, run_id)
            return await self._finish_locked(
                run,
                outcome,
                failure,
                state,
                diagnostics,
            )

    async def abort_owner(
        self,
        owner: object,
        reason: str,
        state: object,
        diagnostics: object,
    ) -> FinishResult | None:
        async with self._lock:
            run = self._find_active_by_owner(owner)
            if run is None:
                return None
            return await self._finish_locked(
                run,
                "aborted",
                {"reason": reason},
                state,
                diagnostics,
            )

    async def _finish_locked(
        self,
        run: _ActiveRun,
        outcome: str,
        failure: object,
        state: object,
        diagnostics: object,
    ) -> FinishResult:
        if outcome not in {"passed", "failed", "aborted"}:
            raise ValueError("outcome must be passed, failed, or aborted")
        if outcome == "passed" and len(run.phases) != len(REHEARSAL_PHASES):
            raise ValueError("all phases must be recorded before a passing finish")
        if outcome == "passed" and any(
            phase["status"] != "passed" or phase["failure"] is not None
            for phase in run.phases
        ):
            raise ValueError(
                "passed report requires every phase to pass without a failure"
            )
        state_snapshot, diagnostics_snapshot = self._snapshot_context(state, diagnostics)
        failure_snapshot = None if failure is None else _json_snapshot(failure, "failure")
        if outcome == "passed" and failure_snapshot is not None:
            raise ValueError("a passing finish cannot contain a failure")
        payload = {
            "schema_version": 1,
            "run_id": run.run_id,
            "started_at": run.started_at,
            "finished_at": _utc_timestamp(self._now()),
            "runtime": _FAKE_RUNTIME,
            "digital_twin": True,
            "hardware_verified": False,
            "hardware_pending": list(HARDWARE_PENDING),
            "plan_version": run.plan_version,
            "git": run.provenance.get("git"),
            "model": run.provenance.get("model"),
            "outcome": outcome,
            "failure": failure_snapshot,
            "state": state_snapshot,
            "diagnostics": diagnostics_snapshot,
            "phases": _json_snapshot(run.phases, "phases"),
        }
        json_path = self.root / f"{run.run_id}.json"
        markdown_path = self.root / f"{run.run_id}.md"
        json_content = json.dumps(
            payload,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        ) + "\n"
        markdown_content = _render_markdown(payload)
        await _run_blocking(
            self._publish_pair,
            json_path,
            json_content,
            markdown_path,
            markdown_content,
        )
        self._active_runs.remove(run)
        return FinishResult(json_path=json_path, markdown_path=markdown_path)

    def _snapshot_context(
        self,
        state: object,
        diagnostics: object,
    ) -> tuple[dict[str, object], dict[str, object]]:
        state_snapshot = _mapping_snapshot(state, "state")
        diagnostics_snapshot = _mapping_snapshot(diagnostics, "diagnostics")
        if diagnostics_snapshot.get("runtime") != _FAKE_RUNTIME:
            raise ValueError("diagnostics runtime must be LEBAI_FAKE")
        if diagnostics_snapshot.get("hardware_verified") is not False:
            raise ValueError("diagnostics hardware_verified must be false")
        return state_snapshot, diagnostics_snapshot

    def _owned_run(self, owner: object, run_id: str) -> _ActiveRun:
        run = self._find_active_by_run_id(run_id)
        if run is None:
            raise ValueError("rehearsal run is not active")
        if run.owner is not owner:
            raise PermissionError("rehearsal run owner does not match")
        return run

    def _find_active_by_owner(self, owner: object) -> _ActiveRun | None:
        return next((run for run in self._active_runs if run.owner is owner), None)

    def _find_active_by_run_id(self, run_id: str) -> _ActiveRun | None:
        return next((run for run in self._active_runs if run.run_id == run_id), None)

    def _require_fake_runtime(self) -> None:
        if self.runtime != _FAKE_RUNTIME:
            raise ValueError("rehearsal reports are available only for LEBAI_FAKE")

    def _publish_pair(
        self,
        json_path: Path,
        json_content: str,
        markdown_path: Path,
        markdown_content: str,
    ) -> None:
        json_temporary = json_path.with_name(f"{json_path.name}.tmp")
        markdown_temporary = markdown_path.with_name(f"{markdown_path.name}.tmp")
        try:
            self._stage_file(json_path, json_content)
            self._stage_file(markdown_path, markdown_content)
            self._replace_staged_file(markdown_temporary, markdown_path)
            self._replace_staged_file(json_temporary, json_path)
        except Exception:
            for path in (
                json_temporary,
                markdown_temporary,
                json_path,
                markdown_path,
            ):
                path.unlink(missing_ok=True)
            raise

    @staticmethod
    def _stage_file(path: Path, content: str) -> Path:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(f"{path.name}.tmp")
        temporary.write_text(content, encoding="utf-8")
        return temporary

    @staticmethod
    def _replace_staged_file(temporary: Path, final: Path) -> None:
        temporary.replace(final)


def _json_snapshot(value: object, label: str) -> object:
    if hasattr(value, "model_dump"):
        return _json_snapshot(value.model_dump(mode="json"), label)
    if isinstance(value, Mapping):
        result: dict[str, object] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError(f"{label} must use string object keys")
            result[key] = _json_snapshot(item, label)
        return result
    if isinstance(value, (list, tuple)):
        return [_json_snapshot(item, label) for item in value]
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"{label} values must be finite")
        return value
    if value is None or isinstance(value, (str, bool, int)):
        return value
    raise ValueError(f"{label} values must be JSON safe")


async def _run_blocking(function: Callable[..., object], *args: object) -> object:
    task = asyncio.create_task(asyncio.to_thread(function, *args))
    try:
        return await asyncio.shield(task)
    except asyncio.CancelledError:
        await task
        raise


def _mapping_snapshot(value: object, label: str) -> dict[str, object]:
    snapshot = _json_snapshot(value, label)
    if not isinstance(snapshot, dict):
        raise ValueError(f"{label} must be an object")
    return snapshot


def _validate_phase_result(result: Mapping[str, object]) -> None:
    status = result.get("status")
    failure = result.get("failure")
    if status not in {"passed", "failed"}:
        raise ValueError("phase status must be passed or failed")
    if status == "passed" and failure is not None:
        raise ValueError("a passed phase cannot contain a failure")
    if status == "failed" and failure is None:
        raise ValueError("a failed phase must contain a failure")


def _utc_timestamp(value: datetime) -> str:
    if value.tzinfo is None:
        value = value.replace(tzinfo=timezone.utc)
    return value.astimezone(timezone.utc).isoformat()


def _render_markdown(payload: Mapping[str, object]) -> str:
    phases = payload["phases"]
    if not isinstance(phases, list):
        raise ValueError("phases must be a list")
    first_failure = _first_failure(phases, payload.get("failure"))
    diagnostics = payload.get("diagnostics")
    log_directory = (
        diagnostics.get("log_session_dir") if isinstance(diagnostics, Mapping) else None
    )
    lines = [
        "# Offline Fake-only rehearsal report",
        "",
        f"- Run ID: `{payload['run_id']}`",
        f"- Runtime: `{payload['runtime']}` (simulation only; no hardware verification)",
        f"- Outcome: {payload['outcome']}",
        "- Hardware verified: false",
        f"- Log directory: {log_directory if log_directory is not None else 'not recorded'}",
        f"- First failure: {_render_value(first_failure)}",
        "",
        "## Phases",
        "",
    ]
    for phase in phases:
        if not isinstance(phase, Mapping):
            raise ValueError("phase must be an object")
        lines.append(f"- `{phase.get('phase', 'unknown')}`: {phase.get('status', 'unknown')}")
    lines.extend(("", "## Hardware verification still pending", ""))
    lines.extend(f"- {item}" for item in HARDWARE_PENDING)
    return "\n".join(lines) + "\n"


def _first_failure(phases: list[object], final_failure: object) -> object:
    for phase in phases:
        if isinstance(phase, Mapping) and phase.get("failure") is not None:
            return phase["failure"]
    return final_failure


def _render_value(value: object) -> str:
    if value is None:
        return "none"
    return json.dumps(value, ensure_ascii=False, sort_keys=True, allow_nan=False)


def _production_provenance() -> dict[str, object]:
    repo_root = Path(__file__).resolve().parents[3]
    return {
        "git": _git_provenance(repo_root),
        "model": {
            name: _sha256(repo_root / relative_path)
            for name, relative_path in _MODEL_PATHS.items()
        },
    }


def _git_provenance(repo_root: Path) -> dict[str, object]:
    dirty_paths = _dirty_paths(repo_root)
    return {
        "commit": _git_output(repo_root, "rev-parse", "HEAD"),
        "dirty": bool(dirty_paths),
        "dirty_paths": dirty_paths,
    }


def _dirty_paths(repo_root: Path) -> list[str]:
    status = _git_output(repo_root, "status", "--porcelain=v1", "-z")
    entries = status.split("\0")
    paths: list[str] = []
    index = 0
    while index < len(entries):
        entry = entries[index]
        index += 1
        if not entry or len(entry) < 4:
            continue
        state = entry[:2]
        paths.append(entry[3:])
        if "R" in state or "C" in state:
            if index < len(entries) and entries[index]:
                paths.append(entries[index])
            index += 1
    return sorted(set(paths))


def _git_output(repo_root: Path, *args: str) -> str:
    try:
        completed = subprocess.run(
            ["git", *args],
            cwd=repo_root,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=False,
        )
    except OSError as error:
        raise RuntimeError(f"git invocation failed: {error}") from error
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise RuntimeError(f"git command failed: {' '.join(args)}: {detail}")
    return completed.stdout.rstrip("\r\n")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
