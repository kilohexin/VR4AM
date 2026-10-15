"""Offline JSONL evidence extraction. No SDK imports or device access."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def stop_anchor(events):
    episodes={e['episode_id'] for e in events if e.get('episode_id') is not None
              and e['kind'] in {'stop_diagnostics','stop_rpc_lifecycle'}}
    legacy_requests=[e for e in events if e['kind']=='stop_requested']
    if len(episodes)>1 or len(legacy_requests)>1:
        raise ValueError('Multiple stop episodes; extract one episode before analysis')
    for kind,field in (('stop_requested','server_mono_ns'),
                       ('stop_diagnostics','started_ns'),('stop_rpc_lifecycle','started_ns')):
        event=next((e for e in events if e['kind']==kind),None)
        if event is not None:return event[field],kind+'.'+field
    raise ValueError('No stop anchor in session')


def finite_motion_evidence(events):
    """Recorded feedback only; no model reconstruction or stop-policy decision."""
    results=[e for e in events if e['kind']=='finite_scope_result']
    if not results:return None
    result=results[-1]
    evidence=dict(assessment='insufficient_or_ambiguous_evidence',
        physical_motion_verified=False,stop_confirmed=result.get('stop_confirmed'),
        latched_fault=result.get('latched_fault'))
    commands=[e for e in events if e['kind']=='pvat_sent']
    metadata=next((e.get('metadata',{}) for e in events if e['kind']=='session_started'),{})
    clock=metadata.get('event_clock')
    if (len(results)!=1 or len(commands)!=1 or result.get('pvat_entries')!=1
            or clock not in {'time.perf_counter_ns','time.monotonic_ns'}
            or result.get('event_clock')!=clock):return evidence
    command=commands[0]
    point=command.get('point_inputs',{})
    native_start=result.get('pvat_call',{}).get('native_started_ns')
    if (type(native_start) is not int
            or type(point.get('snapshot_read_completed_ns')) is not int):return evidence
    baselines=[e for e in events if e['kind']=='robot_kinematics'
        and e.get('snapshot_read_completed_ns')==point.get('snapshot_read_completed_ns')]
    if len(baselines)!=1:return evidence
    baseline=baselines[0]
    def vector(value,length):
        return (isinstance(value,list) and len(value)==length
                and all(type(v) in (int,float) and math.isfinite(v) for v in value))
    if (not vector(baseline.get('actual_q'),6) or not vector(command.get('p'),6)
            or not vector(baseline.get('actual_tcp',{}).get('p'),3)
            or type(command.get('horizon_s')) not in (int,float)
            or not math.isfinite(command['horizon_s']) or command['horizon_s']<=0
            or any(not vector(baseline.get(key),6) or baseline[key]!=point.get(key)
                   for key in ('actual_q','actual_qd','actual_qdd'))
            or any(type(baseline.get(key)) is not int or baseline[key]!=point.get(key)
                   for key in ('snapshot_read_requested_ns','snapshot_read_started_ns','snapshot_read_completed_ns'))
            or not baseline['snapshot_read_requested_ns']<=baseline['snapshot_read_started_ns']
                       <=baseline['snapshot_read_completed_ns']<=native_start):
        return evidence
    reference_end=native_start+round(command['horizon_s']*1e9)
    samples=[]
    for e in events:
        if e['kind']=='robot_kinematics':
            begin,end=e['snapshot_read_requested_ns'],e['snapshot_read_completed_ns'];state=e
        elif e['kind']=='stop_observation_sample':
            begin,end=e['read_started_ns'],e['read_finished_ns'];state=e['state']
        else:continue
        # A read beginning before native entry cannot prove subsequent motion.
        if (native_start<=begin<=end and vector(state.get('actual_q'),6)
                and vector(state.get('actual_tcp',{}).get('p'),3)):
            samples.append((begin,end,state))
    samples.sort(key=lambda row:row[0])
    origin=baseline['actual_tcp']['p']
    evidence.update(assessment='recorded_feedback_only',event_clock=clock,
        baseline='exact command input snapshot, completed before native entry',
        command_max_joint_delta_rad=max(abs(a-b) for a,b in zip(command['p'],baseline['actual_q'])),
        observed_samples=len(samples),
        within_reference_samples=sum(end<=reference_end for begin,end,state in samples),
        first_feedback_started_after_native_ms=(samples[0][0]-native_start)/1e6 if samples else None,
        max_observed_tcp_delta_mm=max((math.dist(state['actual_tcp']['p'],origin)*1000
                                      for begin,end,state in samples),default=None),
        max_observed_joint_delta_rad=max((max(abs(a-b) for a,b in zip(state['actual_q'],baseline['actual_q']))
                                          for begin,end,state in samples),default=None),
        limitations=['No samples during reference does not prove no physical motion.',
                     'A zero measured delta does not establish encoder resolution or a deadband.',
                     'Sequential read windows do not establish controller acquisition times.'])
    return evidence


def deadline_evidence(events, start):
    request = next((e for e in events if e['kind'] == 'stop_rpc_lifecycle'
                    and e['method'] == 'stop_move'), None)
    if request is None:
        return None
    deadline = request['deadline_ns']
    reads = sorted((e for e in events if e['kind'] == 'stop_diagnostic_read'
                    and e.get('outcome') == 'sample'
                    and start <= e['started_ns'] <= e['completed_ns'] <= deadline),
                   key=lambda e: e['completed_ns'])
    states = [e['raw_robot_state'] for e in reads]
    return dict(
        deadline_ms=(deadline-start)/1e6,
        completed_reads=len(reads), raw_states=states,
        observed_span_ms=(reads[-1]['completed_ns']-reads[0]['completed_ns'])/1e6 if reads else None,
        settled_state_reads=sum(s in {'IDLE', 'STOP', 'PAUSED', 5, 6, 12} for s in states),
        max_abs_qd=max((abs(v) for e in reads for v in
                       e['raw_kinematics']['actual_joint_speed']), default=None),
        supports_deferred_escalation=None,
        scope='Completed diagnostic reads only; missing other safety fields are not presumed safe.')


def analyze(path):
    raw = path.read_bytes()
    events = [json.loads(line) for line in raw.splitlines() if line.strip()]
    start,anchor_source = stop_anchor(events)
    observations = [e for e in events if e['kind'] == 'stop_observation_sample']
    x0 = observations[0]['state']['actual_tcp']['p'][0] if observations else None
    rows = []
    for e in observations:
        s = e['state']
        raw_kin = s.get('raw_kinematics', {})
        rows.append(dict(
            read_started_ms=(e['read_started_ns'] - start) / 1e6,
            read_finished_ms=(e['read_finished_ns'] - start) / 1e6,
            state=s['raw_robot_state'],
            dx_from_first_observation_mm=(s['actual_tcp']['p'][0] - x0) * 1000,
            max_abs_qd=max(abs(v) for v in s['actual_qd']),
            actual_torque=raw_kin.get('actual_joint_torque'),
            target_torque=raw_kin.get('target_joint_torque')))
    return dict(
        source_sha256=hashlib.sha256(raw).hexdigest(),
        stop_anchor_source=anchor_source,
        event_count=len(events), observation_count=len(rows),
        stop_move_deadline_evidence=deadline_evidence(events, start),
        baseline='first stop_observation_sample actual_tcp.x; NOT pre-motion',
        final_dx_mm=rows[-1]['dx_from_first_observation_mm'] if rows else None,
        min_dx_mm=min((r['dx_from_first_observation_mm'] for r in rows),default=None),
        max_dx_mm=max((r['dx_from_first_observation_mm'] for r in rows),default=None),
        rpc=[dict(method=e['method'], outcome=e['outcome'],
                  start_ms=(e['started_ns'] - start) / 1e6,
                  completion_ms=(e['completed_ns'] - start) / 1e6,
                  duration_ms=(e['completed_ns'] - e['started_ns']) / 1e6,
                  sdk_await_ms=((e['diagnostic_timing']['sdk_await_completed_ns']
                                 -e['diagnostic_timing']['sdk_await_started_ns'])/1e6)
                      if e.get('diagnostic_timing',{}).get('sdk_await_completed_ns') is not None
                      and e.get('diagnostic_timing',{}).get('sdk_await_started_ns') is not None
                      and e.get('diagnostic_timing',{}).get('clock')=='time.perf_counter_ns'
                      else None)
             for e in events if e['kind'] == 'stop_rpc_lifecycle'],
        first_samples=rows[:15], last_sample=rows[-1] if rows else None,
        finite_motion_evidence=finite_motion_evidence(events),
        limitations=['Sequential SDK reads are not atomic.',
                     'Torque units/freshness and physical cause not established.',
                     'Recorded RPC completion is not physical stop confirmation.'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.session), ensure_ascii=False, indent=2))
