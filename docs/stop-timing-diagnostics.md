# Stop timing evidence

`stop_diagnostics` records local timing evidence for each stop attempt. These
fields do not change the stop transaction, issue SDK reads, or confirm that the
robot has physically stopped.

- `stop_entry.pvat_request_id_in_flight` is the local `move_pvat` request ID
  whose SDK await was active when this stop attempt entered the serialized
  stop transaction, or `null`. It is not the time the public `stop()` call
  began, nor a controller acknowledgement. `pvat_pending` says whether the
  local pump had an unsent target; `sdk_lock_held` may also be true for a
  different SDK operation.
- `stop_move_lock` and `stop_sys_lock` contain `requested_ns`, `acquired_ns`,
  and `wait_ns`. On acquisition, `wait_ns` is the difference between those
  timestamps. If waiting is interrupted, `acquired_ns` is `null` and
  `wait_ns` is the elapsed wait until interruption. These measure local
  SDK-lock contention, not RPC execution time. A `stop_sys_lock` record does
  **not** prove `stop_sys` was called: inspect `rpc_calls` and
  `stop_rpc_lifecycle` for an actual request and its outcome.
- Each `rpc_calls` item records the caller's bounded wait. Its `reused`,
  `episode_id`, and `request_id` distinguish a later attempt from a new SDK
  request. `stop_rpc_lifecycle` records eventual RPC completion, including
  `returned_late`. A late return is not proof of physical stationarity.
- Use the stop verification samples (velocity, raw state, running motion,
  joint and TCP drift) to assess the configured stationary criterion. A
  `STOP` / `FINISHED` status word alone is insufficient.

The original `stop_move` and `stop_sys` deadlines and the escalation/fault
policy are unchanged by this instrumentation. A future real-robot trial must
still treat `stop_unverified` as an unconfirmed stop, not as a recoverable
telemetry error.
