"""One bounded +Z experiment; this module never chooses a native factory."""
import asyncio
import copy
import time
import re

from app.commissioning.native_jog_site_scope import SiteJogScope, SiteJogDiagnosticAdapter
from app.commissioning.native_jog_site_client import SiteJogClient, FEEDBACK_READ_POLICY
from app.commissioning.native_jog_packet import NativeJogPacket,plan_jog,validate_packet,validate_inflight_input
from app.commissioning.finite_motion_observation import MotionObservation, MotionReference
from app.commissioning.idle_stop_baseline import AdmissionRejected, validate_idle_sample, validate_position_continuity
from app.commissioning.paused_stop_cleanup import _cached_sample, _geometry
from app.commissioning.readonly_kinematics import _agree, _pose_error
from app.robots.lebai_codec import pose_from_lebai
from app.robots.base import StopReason, BackendCommandError


def _safe_reason(error):
    # Only local reason codes, never arbitrary vendor exception text.
    reason=str(error)
    if (isinstance(error,(AdmissionRejected,ValueError,BackendCommandError))
            and re.fullmatch(r'[a-z0-9_:.]{1,128}',reason)
            and (reason.startswith(('native_jog_','idle_baseline_','paused_cleanup_',
                                    'finite_scope_','sdk_timeout:'))
                 or reason in ('stop_unverified','stop_incomplete','robot_state_stale',
                               'snapshot_rpc_result_unknown'))):
        return reason
    return 'native_jog_diagnostic_failed'


def _packet_values(packet):
    if type(packet) is not NativeJogPacket:
        raise AdmissionRejected('native_jog_packet_binding_changed')
    values=(packet.velocity_mps,*packet.velocity_mps,packet.acceleration_mps2,
        packet.duration_s,packet.generation,packet.sequence,packet.input_ns,
        packet.expires_ns,packet.config_sha256)
    return tuple((type(value),value) for value in values)


def _snapshot_values(snapshot):
    # Independent scalar/tuple values, including the original timestamps.
    # Nested Pose objects are mutable; object identity alone is insufficient.
    values=[type(snapshot),snapshot.captured_ns,snapshot.read_requested_ns,
        snapshot.read_completed_ns,snapshot.read_purpose,snapshot.robot_state,
        snapshot.estop,snapshot.running_motion,snapshot.motion_state,snapshot.gripper]
    vectors=(snapshot.actual_q,snapshot.actual_qd,snapshot.actual_qdd,snapshot.target_q,
        snapshot.target_qd,snapshot.target_qdd,snapshot.actual_tcp.p,snapshot.actual_tcp.q,
        snapshot.target_tcp.p,snapshot.target_tcp.q,snapshot.actual_flange.p,
        snapshot.actual_flange.q,snapshot.tcp_setting.p,snapshot.tcp_setting.q)
    for vector in vectors:values.extend((type(vector),*vector))
    return tuple((type(value),value) for value in values)


async def _bounded(awaitable, seconds):
    deadline=time.perf_counter_ns()+round(max(0.,seconds)*1e9)
    task = asyncio.create_task(awaitable)
    task.add_done_callback(lambda done:None if done.cancelled() else done.exception())
    try:
        done, _ = await asyncio.wait({task}, timeout=max(0., seconds))
        if not done or time.perf_counter_ns()>=deadline:
            raise AdmissionRejected('native_jog_operation_timeout')
        return task.result()
    finally:
        if not task.done() and not task.cancelling(): task.cancel()


async def run_site_window(scope, recorder, client_factory, *, clock=time.perf_counter_ns,
                          sleep=asyncio.sleep, report=None):
    if type(scope) is not SiteJogScope or not callable(client_factory):
        raise ValueError('native_jog_explicit_scope_and_factory_required')
    report = {} if report is None else report
    report.update(complete=False, mode=report.get('runtime','SIMULATION'), hardware_verified=False,
        joint_bounds_verified=False, braking_verified=False, continuous_control_admitted=False,
        stop_confirmed=False, original_stop_confirmed=False, latched_fault=None,
        speedl_entries=0, stop_entries=0, fk_entries=0, velocity_factor_entries=0,
        read_entries=0, tail_complete=False, cleanup_errors=[], native_connection_release_verified=False,
        process_exit_required=True, command_outcome='not_started', motion_observation={},
        model=scope.model_guard().assessment,
        motion_authorization_policy='feedback_at_entry_original_input_until_rpc_terminal')
    owner = asyncio.current_task()
    started = clock(); deadline = started+20_000_000_000
    proxy = adapter = observer = None
    used = closed = stop_attempted = False
    packet = snapshot = None
    packet_binding = None
    snapshot_binding = None
    fk_completed = None
    pair_client = pair_owner = pair_generation = None
    primary = None
    interruption = None
    phase = 'admission'
    rejection_phase = None

    def reserved_stop():
        if closed or proxy is None or proxy.speedl_entries != 1 or clock() >= deadline:
            raise AdmissionRejected('native_jog_reserved_stop_unavailable')

    def motion_checks():
        # Caller owns input authorization: the adapter at each authority gate,
        # or motion_permission at the native proxy. Do not read twice per gate.
        if (closed or packet is None or snapshot is None or adapter._snapshot is not snapshot
                or adapter._client is not pair_client or adapter._snapshot_calls is not pair_owner
                or adapter._pump.generation != pair_generation
                or owner.done() or owner.cancelling() or deadline-clock() < 5_000_000_000
                or getattr(recorder,'fatal_error',None) is not None
                or getattr(recorder,'dropped_normal_events',0)):
            raise AdmissionRejected('native_jog_motion_permission_revoked')
        if _packet_values(packet)!=packet_binding:
            raise AdmissionRejected('native_jog_packet_binding_changed')
        if _snapshot_values(snapshot)!=snapshot_binding:
            raise AdmissionRejected('native_jog_snapshot_binding_changed')
        proxy.require_clear_read_gate()
        guard=scope.model_guard()  # Recheck current configuration/model binding.
        if proxy.speedl_entries==0:
            guard.validate(packet,snapshot,settings=scope.settings,
                limits=scope.limits,config_sha256=scope.config_sha256)
            validate_packet(packet, limits=scope.limits, actual_p=snapshot.actual_tcp.p,
                snapshot_ns=snapshot.captured_ns, now_ns=clock(), generation=adapter._pump.generation,
                config_sha256=scope.config_sha256)
            if fk_completed is None or not 0 <= clock()-fk_completed < 100_000_000:
                raise AdmissionRejected('native_jog_feedback_fk_expired_before_entry')
        elif (proxy.speedl_entries==1 and type(proxy.speedl_started_ns) is int
                and proxy.speedl_started_ns<=clock() and proxy.pending_speedl is not None
                and proxy.speedl_call is not None
                and proxy.speedl_call.get('native_started_ns')==proxy.speedl_started_ns
                and proxy.speedl_call.get('outcome') in ('entered','returned')):
            # Only trusted actual entry selects this phase. Check original input
            # and generation; the already-used feedback timestamp stays intact.
            validate_inflight_input(packet,now_ns=clock(),generation=adapter._pump.generation,
                config_sha256=scope.config_sha256)
        else:raise AdmissionRejected('native_jog_native_phase_invalid')

    def motion_permission():
        # Full SHA finished before FK. The proxy checks metadata at write.
        scope.read()
        motion_checks()

    async def factory(ip):
        nonlocal used, proxy
        scope.check()
        if used or ip != scope.settings.ip:
            raise AdmissionRejected('native_jog_connection_consumed_or_changed')
        used = True
        native = await _bounded(client_factory(ip), min(5., (deadline-clock())/1e9))
        proxy = SiteJogClient(native,scope,owner=owner,clock=clock,
            motion_permission=motion_permission,stop_permission=reserved_stop)
        proxy.read_deadline_ns = deadline
        return proxy

    async def stop_once():
        nonlocal stop_attempted, primary, rejection_phase
        if proxy is None or not proxy.speedl_entries or stop_attempted: return
        stop_attempted = True
        if observer is not None: observer.revoke()  # Deliberately no await.
        try:
            await adapter.stop(StopReason.SHUTDOWN)
        except BaseException as error:
            # A stop failure is the primary result, even after a motion error.
            primary = error
            rejection_phase = 'stop'

    try:
        scope.check()
        await recorder.start()
        adapter = SiteJogDiagnosticAdapter(scope,client_factory=factory,clock=clock,sleep=sleep,
            event_callback=recorder.write_event)
        phase = 'connect'
        await adapter.connect()
        scope.check()
        phase = 'velocity_factor'
        factor = await proxy.get_velocity_factor()
        if type(factor) not in (float,int) or factor != 100:
            raise AdmissionRejected('native_jog_velocity_factor_not_100')
        phase = 'preflight'
        preflight_started=clock()
        preflight=await adapter.preflight()
        report['preflight']=dict(ready=preflight.ready,
            duration_ms=(clock()-preflight_started)/1e6,
            snapshot_age_ms=(clock()-adapter._snapshot.captured_ns)/1e6)
        if not preflight.ready:
            raise AdmissionRejected('native_jog_preflight_rejected')
        anchor = _cached_sample(adapter)
        validate_idle_sample(anchor,clock()); _geometry(adapter)
        scope.check()  # Cold checks finish before the fresh motion snapshot lease.
        old = adapter._snapshot
        pair_client,pair_owner,pair_generation = adapter._client,adapter._snapshot_calls,adapter._pump.generation
        phase = 'feedback_pair'
        fk_raw,fk_completed,snapshot = await proxy.feedback_pair(old.actual_q,
            lambda:adapter._read_snapshot(read_purpose='motion'))
        fk = pose_from_lebai(fk_raw)
        read=adapter._latest_snapshot_read
        if (adapter._client is not pair_client or adapter._snapshot_calls is not pair_owner
                or adapter._pump.generation!=pair_generation or not adapter._pump.is_current(pair_generation)
                or read is None or read.client is not pair_client or read.owner is not pair_owner
                or read.generation!=pair_generation or read.snapshot is not snapshot
                or adapter._snapshot is not snapshot):
            raise AdmissionRejected('native_jog_feedback_pair_owner_changed')
        report['feedback_fk'] = dict(age_ms=(clock()-fk_completed)/1e6,
            q_identical=tuple(snapshot.actual_q)==tuple(old.actual_q),
            tool_identical=snapshot.tcp_setting==old.tcp_setting,
            **_pose_error(fk,snapshot.actual_tcp))
        if (tuple(snapshot.actual_q) != tuple(old.actual_q)
                or snapshot.tcp_setting != old.tcp_setting
                or not 0 <= clock()-fk_completed < 100_000_000
                or not _agree(fk,snapshot.actual_tcp)):
            raise AdmissionRejected('native_jog_feedback_fk_not_current_or_matching')
        fresh = _cached_sample(adapter)
        validate_idle_sample(fresh,clock());validate_position_continuity(fresh,anchor);_geometry(adapter)
        now = clock()
        packet = plan_jog((0,0,1),1,limits=scope.limits,actual_p=snapshot.actual_tcp.p,
            snapshot_ns=snapshot.captured_ns,now_ns=now,generation=pair_generation,
            sequence=1,input_ns=now,input_expires_ns=now+100_000_000,duration_s=.08,
            config_sha256=scope.config_sha256)
        packet_binding=_packet_values(packet)
        snapshot_binding=_snapshot_values(snapshot)
        phase = 'speedl'
        await adapter.submit_jog(packet,authorize=motion_checks)
        report['command_outcome'] = 'returned'
        def observe_permission():
            # The one diagnostic read grants no new motion. Its repeated
            # ownership checks use the same light input/expiry path as reads.
            scope.read()
            if proxy.speedl_call is None or proxy.speedl_call['outcome'] != 'returned':
                raise AdmissionRejected('native_jog_observation_not_admitted')
        observer = proxy.motion_observation = MotionObservation(proxy,
            reference=MotionReference(proxy.speedl_started_ns,.08,observe_permission))
        observer.start()
        end = proxy.speedl_started_ns+80_000_000
        while (remaining := (end-clock())/1e9) > 0:
            await sleep(remaining)
        await stop_once()
    except BaseException as error:
        primary = error
        rejection_phase = phase
        if isinstance(error,(asyncio.CancelledError,KeyboardInterrupt,SystemExit)):
            interruption = error
        report['command_outcome'] = 'error'
    finally:
        await stop_once()
        if observer is not None: observer.revoke()
        if proxy is not None and proxy.sdk_stop_entries:
            # Bounded tail; an unknown read gate is never reset for diagnostics.
            tail_end = min(deadline,clock()+2_000_000_000)
            tail_ok = True
            tail_samples = 0
            try:
                tail_anchor = _cached_sample(adapter) if adapter._snapshot is not None else None
                while clock() < tail_end:
                    remaining=(tail_end-clock())/1e9
                    if remaining < .2:
                        # No request is started without its normal read budget.
                        # A timer waking slightly early cannot manufacture a
                        # millisecond RPC timeout at the two-second boundary.
                        await sleep(max(0.,remaining))
                        continue
                    proxy.read_deadline_ns = min(proxy.read_deadline_ns,tail_end)
                    sample = await _bounded(adapter.read_stop_observation(),min(.2,(tail_end-clock())/1e9))
                    proxy.require_clear_read_gate()
                    checked=copy.deepcopy(sample)
                    if checked['latched_fault'] in ('stop_unverified','stop_incomplete'):
                        checked['latched_fault']=None  # Diagnostic validation cannot clear the adapter latch.
                    validate_idle_sample(checked,clock());_geometry(adapter)
                    if tail_anchor is not None:validate_position_continuity(sample,tail_anchor)
                    tail_samples += 1
                    await recorder.write_event(dict(kind='native_jog_tail',sample=sample),clock())
                    await sleep(min(.2,max(0.,(tail_end-clock())/1e9)))
            except BaseException as error:
                tail_ok = False
                if primary is None:
                    primary = error
                    rejection_phase = 'tail'
            report['tail_complete'] = tail_ok and tail_samples > 0
            report['tail_samples'] = tail_samples
        if adapter is not None:
            if adapter._site_authority_error is not None:
                report['authority_rejection']=_safe_reason(adapter._site_authority_error)
            report['original_stop_confirmed'] = bool(adapter._stop_transaction and adapter._stop_transaction.confirmed)
            if proxy is not None and (proxy.speedl_remote_result_unknown or observer is not None and observer.blocks_reads):
                adapter._latched_fault = 'stop_unverified'
            report['latched_fault'] = adapter._latched_fault
            try: await adapter.disconnect()
            except BaseException as error: report['cleanup_errors'].append(type(error).__name__)
            report['local_resources_released'] = adapter._client is None
        closed = True
        if proxy is not None:
            proxy.read_gate_closed = True
            native_reads=copy.deepcopy(proxy.native_reads)
            for row in native_reads:
                if row['outcome']=='entered':row['remote_result_unknown']=True
            report.update(speedl_entries=proxy.speedl_entries,stop_entries=proxy.sdk_stop_entries,
                feedback_read_pair=proxy.feedback_pair_report(),read_scheduling_policy=FEEDBACK_READ_POLICY,
                speedl_started_ns=proxy.speedl_started_ns, speedl_call=copy.deepcopy(proxy.speedl_call),
                speedl_remote_result_unknown=proxy.speedl_remote_result_unknown,
                fk_entries=proxy.special_entries['kinematics_forward'],
                velocity_factor_entries=proxy.special_entries['get_velocity_factor'],read_entries=proxy.read_entries,
                snapshot_groups=copy.deepcopy(proxy.snapshot_groups),
                archived_snapshot_owners=len(proxy._snapshot_owner_archive),
                read_calls=copy.deepcopy(proxy.read_calls),native_reads=native_reads,
                read_remote_result_unknown=proxy.read_remote_result_unknown,
                stop_started_ns=proxy.native_started_ns, stop_completed_ns=proxy.native_completed_ns)
        if observer is not None: report['motion_observation'] = observer.snapshot()
        report['stop_confirmed'] = report['original_stop_confirmed'] and report['latched_fault'] is None
        if primary is not None:
            report['error_type'] = type(primary).__name__
            report['primary_rejection'] = _safe_reason(primary)
            report['rejection_phase'] = rejection_phase
        report['complete'] = (primary is None and not report['cleanup_errors']
            and report['speedl_entries'] == report['stop_entries'] == 1 and report['stop_confirmed']
            and report['tail_complete'] and report['motion_observation'].get('outcome') == 'returned')
        try: await recorder.write_critical_event('native_jog_result',copy.deepcopy(report),clock())
        except BaseException as error:
            report['complete'] = False;report['cleanup_errors'].append(type(error).__name__)
        try: await recorder.close()
        except BaseException as error:
            report['complete'] = False;report['cleanup_errors'].append(type(error).__name__)
    if interruption is not None: raise interruption
    if isinstance(primary,(asyncio.CancelledError,KeyboardInterrupt,SystemExit)): raise primary
    return copy.deepcopy(report)
