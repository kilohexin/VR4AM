from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import pytest

from app.rehearsal import report
from app.rehearsal.report import (
    HARDWARE_PENDING,
    REHEARSAL_PHASES,
    RehearsalReportStore,
)


PROVENANCE = {
    "git": {"commit": "a" * 40, "dirty": False, "dirty_paths": []},
    "model": {
        "kinematics_sha256": "b" * 64,
        "fake_config_sha256": "c" * 64,
        "glb_sha256": "d" * 64,
    },
}
READY_STATE = {
    "mode": "READY",
    "robot_state": "IDLE",
    "fault": None,
    "constraint": None,
}
DIAGNOSTICS = {
    "runtime": "LEBAI_FAKE",
    "hardware_verified": False,
    "log_session_dir": "logs/commissioning/session-1",
    "dropped_events": 0,
}


def phase_result(phase: str, *, status: str = "passed") -> dict[str, object]:
    return {
        "phase": phase,
        "status": status,
        "started_client_ms": 10.0,
        "completed_client_ms": 20.0,
        "target": {},
        "measurements": {},
        "failure": None,
    }


def store(tmp_path: Path) -> RehearsalReportStore:
    return RehearsalReportStore(
        root=tmp_path,
        runtime="LEBAI_FAKE",
        provenance=lambda: PROVENANCE,
        now=lambda: datetime(2026, 8, 2, tzinfo=timezone.utc),
    )


async def record_all_phases(
    report_store: RehearsalReportStore,
    owner: object,
    run_id: str,
) -> None:
    for phase in REHEARSAL_PHASES:
        await report_store.record_phase(
            owner,
            run_id,
            phase_result(phase),
            READY_STATE,
            DIAGNOSTICS,
        )


@pytest.mark.asyncio
async def test_finish_writes_authoritative_false_hardware_report(tmp_path: Path) -> None:
    report_store = store(tmp_path)
    owner = object()
    begun = await report_store.begin(owner, 1, READY_STATE, DIAGNOSTICS)
    await record_all_phases(report_store, owner, begun.run_id)

    finished = await report_store.finish(
        owner, begun.run_id, "passed", None, READY_STATE, DIAGNOSTICS
    )

    payload = json.loads(finished.json_path.read_text(encoding="utf-8"))
    assert payload["runtime"] == "LEBAI_FAKE"
    assert payload["hardware_verified"] is False
    assert payload["hardware_pending"] == list(HARDWARE_PENDING)
    assert [item["phase"] for item in payload["phases"]] == list(REHEARSAL_PHASES)
    assert payload["git"] == PROVENANCE["git"]
    assert payload["model"] == PROVENANCE["model"]
    assert not finished.json_path.with_suffix(".json.tmp").exists()
    markdown = finished.markdown_path.read_text(encoding="utf-8")
    assert "Offline Fake-only rehearsal" in markdown
    assert "simulation only; no hardware verification" in markdown
    assert "digital twin" not in markdown.lower()
    assert "Outcome: passed" in markdown
    assert "logs/commissioning/session-1" in markdown
    assert all(item in markdown for item in HARDWARE_PENDING)


@pytest.mark.asyncio
@pytest.mark.parametrize("runtime", ["LEBAI", "SIMULATOR"])
async def test_begin_rejects_non_fake_runtime(tmp_path: Path, runtime: str) -> None:
    report_store = RehearsalReportStore(
        root=tmp_path,
        runtime=runtime,
        provenance=lambda: PROVENANCE,
    )

    with pytest.raises(ValueError, match="LEBAI_FAKE"):
        await report_store.begin(object(), 1, READY_STATE, DIAGNOSTICS)


@pytest.mark.asyncio
async def test_only_the_owner_identity_can_append_to_an_active_run(
    tmp_path: Path,
) -> None:
    report_store = store(tmp_path)
    owner = object()
    begun = await report_store.begin(owner, 1, READY_STATE, DIAGNOSTICS)

    with pytest.raises(PermissionError, match="owner"):
        await report_store.record_phase(
            object(),
            begun.run_id,
            phase_result(REHEARSAL_PHASES[0]),
            READY_STATE,
            DIAGNOSTICS,
        )

    await report_store.record_phase(
        owner,
        begun.run_id,
        phase_result(REHEARSAL_PHASES[0]),
        READY_STATE,
        DIAGNOSTICS,
    )


@pytest.mark.asyncio
async def test_phases_must_be_recorded_once_in_declared_order(tmp_path: Path) -> None:
    report_store = store(tmp_path)
    owner = object()
    begun = await report_store.begin(owner, 1, READY_STATE, DIAGNOSTICS)

    with pytest.raises(ValueError, match="phase order"):
        await report_store.record_phase(
            owner,
            begun.run_id,
            phase_result(REHEARSAL_PHASES[1]),
            READY_STATE,
            DIAGNOSTICS,
        )

    await report_store.record_phase(
        owner,
        begun.run_id,
        phase_result(REHEARSAL_PHASES[0]),
        READY_STATE,
        DIAGNOSTICS,
    )
    with pytest.raises(ValueError, match="phase order"):
        await report_store.record_phase(
            owner,
            begun.run_id,
            phase_result(REHEARSAL_PHASES[0]),
            READY_STATE,
            DIAGNOSTICS,
        )
    with pytest.raises(ValueError, match="all phases"):
        await report_store.finish(
            owner, begun.run_id, "passed", None, READY_STATE, DIAGNOSTICS
        )


@pytest.mark.asyncio
async def test_nonfinite_nested_report_payloads_are_rejected(tmp_path: Path) -> None:
    report_store = store(tmp_path)
    owner = object()
    begun = await report_store.begin(owner, 1, READY_STATE, DIAGNOSTICS)
    invalid_result = phase_result(REHEARSAL_PHASES[0])
    invalid_result["measurements"] = {"nested": [float("nan")]}

    with pytest.raises(ValueError, match="finite"):
        await report_store.record_phase(
            owner,
            begun.run_id,
            invalid_result,
            READY_STATE,
            DIAGNOSTICS,
        )


@pytest.mark.asyncio
async def test_failed_outcome_writes_both_reports(tmp_path: Path) -> None:
    report_store = store(tmp_path)
    owner = object()
    begun = await report_store.begin(owner, 1, READY_STATE, DIAGNOSTICS)
    await record_all_phases(report_store, owner, begun.run_id)

    finished = await report_store.finish(
        owner,
        begun.run_id,
        "failed",
        {"phase": "finalize", "reason": "offline_interlock"},
        READY_STATE,
        DIAGNOSTICS,
    )

    payload = json.loads(finished.json_path.read_text(encoding="utf-8"))
    assert payload["outcome"] == "failed"
    assert payload["failure"] == {
        "phase": "finalize",
        "reason": "offline_interlock",
    }
    assert finished.markdown_path.exists()
    assert not finished.markdown_path.with_suffix(".md.tmp").exists()


@pytest.mark.asyncio
async def test_abort_owner_is_idempotent_and_persists_an_aborted_report(
    tmp_path: Path,
) -> None:
    report_store = store(tmp_path)
    owner = object()
    await report_store.begin(owner, 1, READY_STATE, DIAGNOSTICS)

    first = await report_store.abort_owner(
        owner, "socket_disconnected", READY_STATE, DIAGNOSTICS
    )
    second = await report_store.abort_owner(
        owner, "socket_disconnected", READY_STATE, DIAGNOSTICS
    )

    assert first is not None
    assert second is None
    payload = json.loads(first.json_path.read_text(encoding="utf-8"))
    assert payload["outcome"] == "aborted"
    assert payload["failure"] == {"reason": "socket_disconnected"}


def test_provenance_uses_one_consistent_git_status_snapshot(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    calls: list[tuple[str, ...]] = []

    def git_output(_root: Path, *args: str) -> str:
        calls.append(args)
        if args == ("rev-parse", "HEAD"):
            return "a" * 40
        if args == ("status", "--porcelain=v1", "-z"):
            return " M reports/offline rehearsal.md\0"
        raise AssertionError(f"unexpected git arguments: {args}")

    monkeypatch.setattr(report, "_git_output", git_output)

    provenance = report._git_provenance(tmp_path)

    assert provenance == {
        "commit": "a" * 40,
        "dirty": True,
        "dirty_paths": ["reports/offline rehearsal.md"],
    }
    assert calls.count(("status", "--porcelain=v1", "-z")) == 1


@pytest.mark.asyncio
async def test_passing_finish_rejects_failed_phases_with_recorded_failures(
    tmp_path: Path,
) -> None:
    report_store = store(tmp_path)
    owner = object()
    begun = await report_store.begin(owner, 1, READY_STATE, DIAGNOSTICS)
    for phase in REHEARSAL_PHASES:
        failed_phase = phase_result(phase, status="failed")
        failed_phase["failure"] = {"reason": "offline_interlock"}
        await report_store.record_phase(
            owner,
            begun.run_id,
            failed_phase,
            READY_STATE,
            DIAGNOSTICS,
        )

    with pytest.raises(ValueError, match="passed report requires"):
        await report_store.finish(
            owner, begun.run_id, "passed", None, READY_STATE, DIAGNOSTICS
        )

    assert not (tmp_path / f"{begun.run_id}.json").exists()
    assert not (tmp_path / f"{begun.run_id}.md").exists()


@pytest.mark.asyncio
async def test_markdown_staging_failure_leaves_no_final_pair_and_run_can_retry(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    report_store = store(tmp_path)
    owner = object()
    begun = await report_store.begin(owner, 1, READY_STATE, DIAGNOSTICS)
    await record_all_phases(report_store, owner, begun.run_id)
    original_stage = report_store._stage_file

    def fail_markdown_stage(path: Path, content: str) -> Path:
        if path.suffix == ".md":
            raise OSError("markdown staging exploded")
        return original_stage(path, content)

    monkeypatch.setattr(report_store, "_stage_file", fail_markdown_stage)
    with pytest.raises(OSError, match="markdown staging exploded"):
        await report_store.finish(
            owner, begun.run_id, "passed", None, READY_STATE, DIAGNOSTICS
        )

    assert not (tmp_path / f"{begun.run_id}.json").exists()
    assert not (tmp_path / f"{begun.run_id}.md").exists()
    assert not (tmp_path / f"{begun.run_id}.json.tmp").exists()
    assert not (tmp_path / f"{begun.run_id}.md.tmp").exists()

    monkeypatch.setattr(report_store, "_stage_file", original_stage)
    finished = await report_store.finish(
        owner, begun.run_id, "passed", None, READY_STATE, DIAGNOSTICS
    )
    assert finished.json_path.exists()
    assert finished.markdown_path.exists()


@pytest.mark.asyncio
async def test_json_publish_failure_rolls_back_markdown_and_keeps_run_active(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    report_store = store(tmp_path)
    owner = object()
    begun = await report_store.begin(owner, 1, READY_STATE, DIAGNOSTICS)
    await record_all_phases(report_store, owner, begun.run_id)
    original_replace = report_store._replace_staged_file

    def fail_json_publish(temporary: Path, final: Path) -> None:
        if final.suffix == ".json":
            raise OSError("json publish exploded")
        original_replace(temporary, final)

    monkeypatch.setattr(report_store, "_replace_staged_file", fail_json_publish)
    with pytest.raises(OSError, match="json publish exploded"):
        await report_store.finish(
            owner, begun.run_id, "passed", None, READY_STATE, DIAGNOSTICS
        )

    assert not (tmp_path / f"{begun.run_id}.json").exists()
    assert not (tmp_path / f"{begun.run_id}.md").exists()
    assert not (tmp_path / f"{begun.run_id}.json.tmp").exists()
    assert not (tmp_path / f"{begun.run_id}.md.tmp").exists()

    monkeypatch.setattr(report_store, "_replace_staged_file", original_replace)
    finished = await report_store.finish(
        owner, begun.run_id, "passed", None, READY_STATE, DIAGNOSTICS
    )
    assert finished.json_path.exists()
    assert finished.markdown_path.exists()
