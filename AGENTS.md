# VR4Arm agent notes

- For real-robot control, commissioning, or deployment work, read `docs/lab-codex-handoff-2026-09-29.md` first. It records the current lab status; the README's Milestone 1 introduction is historical.
- You may inspect code, analyze existing logs, edit software, and run relevant offline tests without asking for permission at every step. Prefer targeted tests; do not routinely rerun the entire backend suite.
- Never treat an offline test, SDK return, single `IDLE` sample, or physical E-stop availability as proof that motion-time software stopping works. Preserve the `stop_unverified` latch and do not silently restore automatic `stop_sys` escalation.
- Do not connect to the real robot, switch to `control`, send motion/gripper/Home/stop/reset commands, or start a new live test without an explicit user-approved test scope and onsite safety conditions. Stop on unplanned motion or an unconfirmed stop; do not auto-reset or auto-retry.
- Never commit local real-robot configuration, credentials, raw session logs, or sensitive tool output. Keep field evidence on the lab host and reference it by path/hash in reports.
