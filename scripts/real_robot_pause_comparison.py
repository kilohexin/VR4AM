"""Preview by default. One separately authorized +Z3mm early pause, no capture."""
import argparse
import asyncio
from datetime import UTC, datetime, timedelta
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "backend"))


def load_local(name, filename):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(filename))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


idle = load_local("pause_idle_helpers", "real_robot_idle_stop.py")
wire = load_local("pause_source_helpers", "motion_stop_wire_capture.py")
sha = idle.sha
CAPS = dict(wire.CAPS)
CONFIRM = "I_APPROVE_ONE_Z3MM_EARLY_PAUSE_COMPARISON"


def runtime_hashes():
    hashes = wire.runtime_hashes()
    for path in (Path(__file__).resolve(), ROOT / "scripts/real_robot_idle_stop.py"):
        hashes[path.relative_to(ROOT).as_posix()] = sha(path)
    return hashes


def existing_rpc_connections():
    # Local numeric connection table only; no remote probing or capture tools.
    return wire._connections.read_rpc_connections()


def prepare(config, output, *, expires=None):
    plan = idle.prepare(config, output, expires=expires)
    for key, cap in CAPS.items():
        value = getattr(plan["settings"].control, key)
        if not math.isfinite(value) or not 0 < value <= cap:
            raise ValueError("diagnostic_cap_invalid:" + key)
    plan["runtime_hashes"] = runtime_hashes()
    return plan


def preview(plan):
    return dict(
        schema=1, scope="one_z3mm_early_pause_comparison",
        target_ip=plan["settings"].ip, output=str(plan["output"]),
        config_sha256=plan["config_sha256"], runtime_hashes=plan["runtime_hashes"],
        authorization_expires_utc=plan["authorization_expires_utc"],
        action=dict(axis="z", distance_m=.003, early_stop_on_motion=True),
        allowed_writes=["move_pvat", "pause_move"], max_sdk_pause_entries=1,
        stop_rpc_timeout_ms=200, stable_window_ms=300, verification_timeout_ms=500,
        readonly_tail_s=2., caps=CAPS, packet_capture=False,
        automatic_prepare=False, automatic_home=False, automatic_resume=False,
        automatic_reset=False, automatic_retry=False, stop_move_fallback=False,
        requires_fresh_onsite_authorization=True, hardware_stop_verified=False,
        completion_means="diagnostic_completed_not_vr_acceptance",
        paused_motion_may_be_retained=True,
        native_connection_release="standalone_process_exit_required_not_verified",
    )


def check(plan, *, approved, expected_hash, expires, confirm):
    if confirm != CONFIRM:
        raise ValueError("pause_scope_confirmation_required")
    try:
        expiration = datetime.fromisoformat(expires.replace("Z", "+00:00"))
        now = datetime.now(UTC)
        if (expiration.tzinfo is None or expiration.utcoffset() != timedelta(0)
                or not now < expiration <= now + timedelta(minutes=10)
                or expires != plan["authorization_expires_utc"]):
            raise ValueError
    except (ValueError, TypeError, AttributeError):
        raise ValueError("pause_expiry_invalid_or_expired") from None
    if approved is None or type(expected_hash) is not str or not re.fullmatch("[a-f0-9]{64}", expected_hash):
        raise ValueError("pause_preview_hash_mismatch")
    try:
        raw = Path(approved).read_bytes()
    except OSError:
        raise ValueError("pause_preview_unavailable") from None
    if hashlib.sha256(raw).hexdigest() != expected_hash or json.loads(raw) != preview(plan):
        raise ValueError("pause_scope_or_hash_changed")
    if sha(plan["config"]) != plan["config_sha256"] or runtime_hashes() != plan["runtime_hashes"]:
        raise ValueError("pause_inputs_changed")


async def execute(plan, *, approved, expected_hash, expires, confirm, client_factory=None):
    def admit():
        check(plan, approved=approved, expected_hash=expected_hash, expires=expires, confirm=confirm)

    admit()
    if existing_rpc_connections() != 0:
        raise ValueError("existing_rpc_connections")
    output = plan["output"]
    output.mkdir(parents=True, exist_ok=False)  # A failed connection consumes the attempt too.
    (output / "admission.json").write_text(json.dumps(dict(
        scope=preview(plan), approved_preview_sha256=expected_hash,
        started_utc=datetime.now(UTC).isoformat(), hardware_stop_verified=False,
    ), indent=2), encoding="utf-8")
    config = output / "config-control.yaml"
    config.write_bytes(plan["config"].read_bytes())

    # These imports define software; the native SDK is loaded only by the
    # admitted factory below, after all local checks and single-use reservation.
    from app.commissioning import smoke
    from app.commissioning.actions import SmokeOptions, TranslationAction
    from app.commissioning.pause_comparison import build_pause_backend, pause_is_settled
    from app.config import REAL_ROBOT_CONFIRMATION
    from app.robots.lebai_sdk_bridge import connect_real_client
    factory = client_factory or connect_real_client
    connection_attempted = False

    def admit_motion():
        admit()
        if sha(config) != plan["config_sha256"]:
            raise ValueError("pause_copied_config_changed")

    async def admitted_connect(ip):
        nonlocal connection_attempted
        admit_motion()
        if connection_attempted or ip != plan["settings"].ip:
            raise ValueError("pause_connection_not_admitted")
        if existing_rpc_connections() != 0:
            raise ValueError("existing_rpc_connections")
        if client_factory is None:
            wire.verify_sdk_source()
        connection_attempted = True
        return await factory(ip)

    result = dict(complete=False, hardware_stop_verified=False,
                  runtime="LIVE_DIAGNOSTIC" if client_factory is None else "SIMULATION",
                  early_stop_triggered=False, automatic_resume=False, packet_capture=False)
    old_root = smoke.COMMISSIONING_LOG_ROOT
    smoke.COMMISSIONING_LOG_ROOT = output / "sessions"
    try:
        action = await smoke.run_smoke(
            SmokeOptions(config, TranslationAction("z", .003), REAL_ROBOT_CONFIRMATION,
                         observe_stop_seconds=2., stop_rpc_diagnostics=True, early_stop_on_motion=True),
            admitted_connect, before_motion=admit_motion, before_motion_commit=admit_motion,
            backend_builder=build_pause_backend, settled_state_check=pause_is_settled,
        )
        result.update(complete=True, early_stop_triggered=action.early_stop_triggered,
                      action=action.to_dict())
    except (asyncio.CancelledError, KeyboardInterrupt, SystemExit):
        result.update(error="interrupted", operator_attention_required=True)
        raise
    except Exception as error:
        result.update(error=str(error), error_type=type(error).__name__, operator_attention_required=True)
    finally:
        smoke.COMMISSIONING_LOG_ROOT = old_root
        result["connection_attempted"] = connection_attempted
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--execute-once", action="store_true")
    parser.add_argument("--approved-preview", type=Path)
    parser.add_argument("--approved-preview-sha256")
    parser.add_argument("--authorization-expires-utc")
    parser.add_argument("--confirm")
    args = parser.parse_args(argv)
    plan = prepare(args.config, args.output, expires=args.authorization_expires_utc)
    if not args.execute_once:
        print(json.dumps(preview(plan), indent=2))
        return 0
    if Path(sys.executable).resolve() != (ROOT / ".venv/Scripts/python.exe").resolve():
        raise ValueError("repository_venv_required")
    result = asyncio.run(execute(plan, approved=args.approved_preview,
                                expected_hash=args.approved_preview_sha256,
                                expires=args.authorization_expires_utc, confirm=args.confirm))
    print(json.dumps(result, indent=2))
    return 0 if result["complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
