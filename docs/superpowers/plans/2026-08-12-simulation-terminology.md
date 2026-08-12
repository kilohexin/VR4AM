# Simulation Terminology Compatibility Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将所有当前用户可见术语统一为“仿真 / 真机”，同时保留内部 `digital_twin` 兼容字段、模块目录和历史设计归档。

**Architecture:** 仅修改呈现层、启动器输出、Markdown 报告和现行操作文档；不修改 WebSocket 消息结构、JSON 报告兼容字段、Python 模块路径或后端类型。已有精确文案测试先改为新术语并确认失败，再实施最小文案替换。

**Tech Stack:** TypeScript/Vitest、Python/Pytest、FastAPI、Markdown。

## Global Constraints

- 用户可见内容只使用“仿真”或“真机”。
- 内部 `digital_twin` JSON 字段、Python 模块路径和历史设计归档保持不变。
- `LEBAI_FAKE` 与 `hardware_verified=false` 继续明确显示，禁止把仿真冒充真机。
- 不改变控制、安全、演练状态机或报告 JSON 结构。

---

### Task 1: 界面与启动输出

**Files:**
- Modify: `web/src/ui/hud.ts`
- Modify: `web/src/ui/armPanel.ts`
- Modify: `web/src/ui/diagnosticsPanel.ts`
- Modify: `web/src/scenes/vrSafetyPanel.ts`
- Modify: `scripts/run_offline_rehearsal.py`
- Test: `web/tests/hud.test.ts`
- Test: `web/tests/armPanel.test.ts`
- Test: `web/tests/diagnosticsPanel.test.ts`
- Test: `backend/tests/scripts/test_run_offline_rehearsal.py`

**Interfaces:**
- Consumes: existing `LEBAI_FAKE` runtime identity.
- Produces: exact visible labels `仿真模式，不是真机`, `仿真 · LEBAI_FAKE`, `解锁仿真`, `仿真已连接`, `SIMULATION`.

- [ ] **Step 1:** Change exact-copy assertions to the new terminology.
- [ ] **Step 2:** Run the four focused test files and verify they fail on the old visible copy.
- [ ] **Step 3:** Replace only user-visible strings in the four frontend components and launcher.
- [ ] **Step 4:** Re-run the focused tests and verify they pass.

### Task 2: Reports and current operator documentation

**Files:**
- Modify: `backend/app/rehearsal/report.py`
- Modify: `backend/app/main.py`
- Modify: `backend/tests/rehearsal/test_report.py`
- Modify: `README.md`
- Modify: `docs/fake-lebai-acceptance.md`
- Modify: `docs/offline-rehearsal.md`
- Modify: `docs/real-robot-deployment.md`

**Interfaces:**
- Consumes: report payload with unchanged `runtime=LEBAI_FAKE`, `hardware_verified=false`, and compatibility Boolean.
- Produces: Markdown/log/operator prose that says simulation only; JSON compatibility schema remains byte-for-byte compatible by field name and type.

- [ ] **Step 1:** Add/change Markdown report assertions so old English terminology fails.
- [ ] **Step 2:** Change report/log prose and current operator documents; do not edit archived specs/plans.
- [ ] **Step 3:** Run focused report tests.

### Task 3: Verification and local commit

**Files:**
- Verify all files from Tasks 1–2.

- [ ] **Step 1:** Run full backend tests.
- [ ] **Step 2:** Run full frontend tests and production build.
- [ ] **Step 3:** Scan current user-facing sources/docs for forbidden terminology, allowing only internal compatibility identifiers and archived history.
- [ ] **Step 4:** Run `git diff --check`, review the diff, and create one local commit without pushing.
