# Stop timing evidence

`stop_diagnostics` records local timing evidence for each stop attempt. These
fields do not issue SDK reads or confirm that the robot has physically stopped.
The current production LEBAI policy is identified by
`stop_policy: "stop_move_only"`.

- `stop_entry.pvat_request_id_in_flight` is the local `move_pvat` request ID
  whose SDK await was active when this stop attempt entered the serialized
  stop transaction, or `null`. It is not the time the public `stop()` call
  began, nor a controller acknowledgement. `pvat_pending` says whether the
  local pump had an unsent target; `sdk_lock_held` may also be true for a
  different SDK operation.
- `stop_move_lock` contains `requested_ns`, `acquired_ns`, and `wait_ns`. On
  acquisition, `wait_ns` is the difference between those timestamps. If
  waiting is interrupted, `acquired_ns` is `null` and `wait_ns` is the elapsed
  wait until interruption. These measure local SDK-lock contention, not RPC
  execution time. Older sessions may contain `stop_sys_lock`; that field alone
  does **not** prove `stop_sys` was called. Inspect `rpc_calls` and
  `stop_rpc_lifecycle` for an actual request and its outcome.
- Each `rpc_calls` item records the caller's bounded wait. Its `reused`,
  `episode_id`, and `request_id` distinguish a later attempt from a new SDK
  request. `stop_rpc_lifecycle` records eventual RPC completion, including
  `returned_late`. Its `started_ns` is request-object creation;
  `sdk_await_started_ns` is the local timestamp immediately before awaiting
  the SDK coroutine, and `sdk_await_completed_ns` is when that await exits
  locally (return, error, or cancellation). A `null` value does not establish
  whether the SDK ran; an unstarted task may emit no lifecycle event at all.
  The timestamps distinguish local scheduling delay
  from time spent awaiting the SDK, but do **not** timestamp network send,
  controller receipt, or physical stop. A late return is not proof of
  physical stationarity.
- The 200 ms stop-RPC deadline starts when `StopRpc` is created, after SDK-lock
  acquisition. It is not a bound on elapsed time from Grip release or the public
  `stop()` call. The stop-transaction serialization wait is also outside this
  RPC deadline. A long lock wait followed by a fast RPC is distinct from a late
  RPC return; neither guarantees a physical stopping time.
- Use the stop verification samples (velocity, raw state, running motion,
  joint and TCP drift) to assess the configured stationary criterion. A
  `STOP` / `FINISHED` status word alone is insufficient.

The timing instrumentation did not change RPC deadlines. The current
production policy separately removes automatic `stop_sys` escalation: if
`stop_move` cannot be confirmed, the stop remains failed and fault-latched;
new motion remains blocked. A late `stop_move` return proves neither an
on-time response nor physical stationarity. A real-robot trial must treat
`stop_unverified` as an unconfirmed stop, not as a recoverable telemetry error.

## VR input age versus robot read timing

In public `RobotControl.state_message()` output, the legacy `sample_age_ms`
field is the age of the latest received VR frame. It is **not robot telemetry
age**. The HUD and diagnostics panel now label it `VR 输入帧龄` / `VR input age`.
This preserves the existing wire field and its meaning.

`robot_kinematics` now includes three local monotonic timestamps:

- `snapshot_read_requested_ns`: before waiting for the SDK lock.
- `snapshot_read_started_ns`: after acquiring the lock, before the sequential
  SDK reads. This is the existing `server_mono_ns` / `captured_ns` time origin.
- `snapshot_read_completed_ns`: after the final SDK read returns and the lock
  is released, before payload validation, model construction and logging.

Stop verification samples and stop-observation snapshots include equivalent
`read_requested_ns` and `read_completed_ns`, alongside the existing
`captured_ns` and `observed_ns`. Missing fields in older logs mean unknown,
not zero duration. These additions issue no extra SDK calls and do not change
the timestamps used for freshness gates or stop confirmation.

Reads are sequential, not an atomic device snapshot. These times measure
local retrieval, not sensor/controller sample times. Identical values with
new local timestamps cannot establish whether the device stayed still or the
SDK returned cached source data. Device-side timestamps or sequence numbers
would be needed for an independent source-freshness assessment.

## Snapshot read purposes

`robot_kinematics.snapshot_read_purpose` labels the adapter entry point:
`connection`, `state`, `preflight`, `motion`, `gripper`, `home`, `prepare`,
or `joint_move`. Stop samples and stop observations expose `read_purpose`
as `stop_verification` and `stop_observation`. Direct private reads default
to `unspecified`; absent fields in older logs remain unknown.

These labels add no reads and change no lock ordering, caching, freshness
checks or stop deadlines. `state` means a `get_state()` call, which can be
used by display code **or control decisions**; it does not uniquely identify
UI polling. A read that fails before producing a snapshot has no successful
snapshot event. Purpose labels cannot retrospectively fill either gap.

## Finite-window log audit (15 October 2026)

`scripts/analyze_stop_timeline.py SESSION_JSONL` now accepts single finite-window
sessions without the historical `stop_requested` event. The output identifies
whether its stop anchor came from that event, `stop_diagnostics.started_ns`, or
the RPC lifecycle. Multiple explicit episodes or multiple legacy stop requests
are rejected rather than merged. Missing observation samples produce unknown
displacement values, while preserving the stop failure evidence.

`finite_motion_evidence` is available only for an unambiguous single native
PVAT with a matching event clock and the exact input snapshot: requested,
started and completed read timestamps plus actual q/qd/qdd must match. It
reports the commanded joint delta, measured TCP/joint differences, and complete
read windows inside the reference interval. It does not reconstruct controller
sample times, infer encoder resolution/deadband, or certify physical movement
or stopping. Nested QPC SDK durations are calculated within their own domain.

For the 13:58 QPC diagnostic, the maximum commanded joint delta was
0.0001295637 rad. The first **successful** post-entry feedback read began
498.5103 ms after native PVAT entry; none of its nine successful read windows
was inside the 80 ms reference interval. Recorded TCP and joint differences
were zero. This leaves actual execution during that interval unobserved; it
does not establish that IK failed, that the controller ignored the target, or
that physical movement did not occur. The stop RPC SDK await remained
218.3874 ms, beyond the original 200 ms limit, with `stop_unverified` preserved.

Local derived reports and red/green verification records are under
`.tmp/finite-timeline-20261015/`; raw sessions remain private. Three related
test files passed 71 targeted cases. Initial review found incomplete baseline
matching and ambiguous legacy episodes; both were reproduced with failing
tests, corrected, and reviewed again. This change imports no SDK, creates no
connections, changes no control policy, and requires no frontend rebuild.

The next execution investigation needs feedback during the reference window,
with independent ownership and a bound that cannot postpone the original stop.
That is a new diagnostic scope, not permission to add reads or motions to the
consumed scope. Increasing the target or accepting a late stop is not justified
by these records. The vendor describes PVAT as continuous servo motion with a
minimum 10 ms point and a send interval shorter than its reference time; that
description is not evidence that this single short reference physically ran.
See the [official motion reference](https://help.lebai.ltd/en/sdk/motion.html).
