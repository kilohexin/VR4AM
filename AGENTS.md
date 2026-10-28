# VR4Arm agent notes

- Current development entry: `README.md` and `docs/project-status.md`. The user has authorized autonomous development and tests against the verified local official Docker simulator; physical hardware is unavailable. Use `scripts/run_lmaster_docker.py`, then `web`'s `docker:vr` / `test:docker-xr` commands. Docker results do not authorize a real connection.
- For real-robot control, commissioning, or deployment work, read `docs/lab-codex-handoff-2026-09-29.md` first, then the latest dated field evidence. The handoff is historical, not a statement of today's onsite conditions. The old Milestone 1 README is archived.
- You may inspect code, analyze existing logs, edit software, and run relevant offline tests without asking for permission at every step. Prefer targeted tests; do not routinely rerun the entire backend suite.
- Never treat an offline test, SDK return, single `IDLE` sample, or physical E-stop availability as proof that motion-time software stopping works. Preserve the `stop_unverified` latch and do not silently restore automatic `stop_sys` escalation.
- Do not connect to the real robot, switch to `control`, send motion/gripper/Home/stop/reset commands, or start a new live test without an explicit user-approved test scope and onsite safety conditions. Stop on unplanned motion or an unconfirmed stop; do not auto-reset or auto-retry.
- Never commit local real-robot configuration, credentials, raw session logs, or sensitive tool output. Keep field evidence on the lab host and reference it by path/hash in reports.
