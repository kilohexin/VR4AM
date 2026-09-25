# Stop-move-only failure policy design

## Intent and status

The operator explicitly requests removal of the production adapter's automatic `stop_sys` escalation, including when `stop_move` times out, raises, is cancelled, or returns without a verified stationary state. The purpose is to avoid automatically entering the system-stop/disabled-joint path during VR operation. This is a change in failure policy, **not** a repair of late `stop_move` responses. It does not authorize a robot run.

R58 observed a 200 ms local `stop_move` deadline and a 625 ms eventual return. The source of the delay is not established. R59/R61 added passive timing fields but have not yet produced new real-robot timing evidence. The R58 record also does not establish what the arm would have done without `stop_sys`.

## Selected behavior

In the LEBAI production adapter, every control-mode stop still invalidates queued PVAT work. When a client is available, it issues the owned `stop_move` request once per stop transaction and attempts the existing stationary verification when that request returns in time. The existing 200 ms request deadline, verification thresholds, request ownership, and no-new-motion gates stay unchanged.

On a timeout, SDK error, cancellation, disconnected client, or failed stationary verification, the adapter does **not** initiate or reuse an automatic `stop_sys` request. It preserves the existing `stop_unverified` or `stop_incomplete` fault distinction, the control layer's existing failure event and motion gate, and late `stop_move` lifecycle evidence. A late return is not promoted to a confirmed physical stop. A later disconnect/shutdown in the same stop episode does not send `stop_sys` either. Existing fault-clear and cancelled-stop/disconnect rules are not broadened.

This policy is unconditional for the production LEBAI adapter, not an opt-in test profile. It does not remove the SDK's `stop_sys` capability or change explicit, separately authorized diagnostic scripts that call it. It does not introduce an automatic `stop_move` retry, a new stop API, or a longer timeout.

## Observability and operator boundary

The stop diagnostic record must make the selected policy explicit, so absence of a `stop_sys` RPC cannot be mistaken for an omitted log. Preserve the existing `stop_move` request ID, deadline, final/late outcome, samples, and latched fault. A failed stop remains visibly faulted under existing control-layer rules, including the existing special case for a verified disconnect after cancellation; `/health` alone is not a runtime-fault indicator. The operator must use the independent on-site stopping procedure if motion persists or stopping cannot be confirmed. Software fault latching prevents *new* commands while latched; it does not prove ongoing motion has stopped. Physical emergency stop or power removal is not equivalent to the application fault latch.

## Verification and release boundary

Test first, observing failures under current code, then make the smallest production change. Targeted offline tests must cover: normal confirmed stop; timeout with late return; SDK error; cancellation; failed stationary verification; repeated stop/disconnect/shutdown in one episode; and absence of any automatic `stop_sys` call in each failure path. They must also show unchanged fault latching and rejection of new motion. Run the repository backend suite before claiming code completion, reporting any failures by name rather than rerunning until green. No real-robot test, VR action, service startup, config-mode switch, or deployment is part of this implementation.

The delivered change must be described as "automatic system-stop escalation removed; `stop_move` latency unresolved." Offline passing tests cannot establish that a failed `stop_move` leaves the arm stationary or make the real-robot path safe for operation.
