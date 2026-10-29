# Docker user experience preparation

User authorizes continuous offline development and official Docker tests, followed by their own headset acceptance. No physical hardware access; no new push is implied by this continuation.

1. Fix existing status copy in `holdControlPanel.ts`: distinguish disconnected, fault, unlocked awaiting prepare, recovery/prepare, planning, executing and draining without changing authority or timing. Cover rendered text plus unchanged disabled states.
2. Extend finite XR regression to recapture a moved hand at the latest TCP/orientation after release and to reject regrip during draining. Extend the Docker protocol check with a stationary regrip/no-new-goal observation.
3. Run targeted tests, build and official Docker protocol acceptance; attempt browser validation through the existing computer-use API. Record unavailable UI/headset checks explicitly.
4. Supply one short desktop/headset user checklist. Freeze a local commit with test evidence. Hardware acceptance remains a subsequent explicitly approved scope.

Completed 2026-10-29: all four steps, including browser interaction and native Docker synthetic XR. Browser reproduction also required HUD/ARIA preparation labels to align with hold-panel text. Native Docker cold boot exposed ROBOT_ON; verified local start_sys then added that unstarted state to the Docker-only startup branch, keeping PAUSED/MOVING/error rejection. Evidence and limits are in docs/project-status.md; the user checklist is in README.md. Real hardware was not accessed.
