# Docker-first functional delivery

**Goal:** Make the official local controller the primary web/VR development and user acceptance environment, then archive the local commits on GitHub.

**Authority:** The user explicitly authorizes autonomous Docker simulation development/testing, repository cleanup and this push. Real hardware is unavailable; no real connection is part of this work. Existing offline design approval opt-out applies.

**Design:** Keep one finite-position web/VR control implementation and the independent visual simulator. Separate the Docker speed profile from historical commissioning limits: joint speed cap 0.3 rad/s, TCP cap 0.03 m/s, angular cap 0.5 rad/s; retain acceleration 0.2 rad/s² and existing step, arrival, freshness and unknown-result handling. These are simulation choices, not physical acceptance. Preserve the default real planner cap of 0.05 rad/s. Provide a dedicated HTTPS proxy to the bound Docker application, not a frontend that accidentally targets the old port 8000.

**Implementation:**

- [x] Add regression coverage for Docker speed selection, native planner defaults, prepare options, and fixed Docker HTTPS routing. Implement narrow policy wiring.
- [x] Run targeted backend/frontend tests, build, official Docker web operations and synthetic XR translation/pitch/claw/release/tracking-loss/reprepare/disconnect.
- [x] Replace obsolete README with current entry points; archive the previous README with its historical status explicit. Inventory obsolete tools without deleting evidence or active regression dependencies.
- [ ] Review diff and outgoing commits for local data, record validation limits, commit and push the current branch without force. Verify remote tip.

**Review focus:** Higher configured speed must reach the planner without changing real defaults; preparation must use the same simulation profile; proxy must only target 127.0.0.1:8773; real devices cannot be selected through Docker; unknown calls never become successful because a page reconnects. Actual headset and physical arm acceptance remain separate.

**Execution record:** See `docs/project-status.md`. Browser automation and independent review became unavailable partway through; validation is explicitly limited to completed UI actions, native Docker synthetic XR, protocol/TLS checks and targeted tests. Archived the old README instead of deleting still-referenced diagnostic implementations. Added same-origin CSP after review identified embedded frontend URL overrides. No physical connection.
