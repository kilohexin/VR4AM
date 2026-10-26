# 工具俯仰 Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans in the current session. Offline only.

**Goal:** 网页长按与 VR 相对手腕俯仰接入现有有限小步。
**Architecture:** 可选 pitch 意图、显式能力；共享 IK/FK/movej；真机默认拒绝。
**Tech Stack:** Python/NumPy/SciPy/Pydantic；TypeScript/Three.js/Vitest；本机 Fake/Edge。
**Spec:** `docs/superpowers/specs/2026-10-27-tool-pitch-design.md`。

## Global Constraints

每步2 mm/0.5°，固定准备参考±20°，不扩大工作区；保持100 ms新鲜度、原停止及未知锁存。不连接SDK或真实地址，不推送。

## Review Focus

纯旋转零长度、相反/无实际进展；不合法四元数；释放/过期重抓；绕过累计边界；旧真机客户端伪造能力。

### Task 1: 后端契约与姿态规划

修改 finite_position_types/gate/plan/adapter、schemas/messages、robot_control、teleop_ws；新增 tests/robots/test_finite_position_pitch.py 及协议回归。

- [x] 先失败测试：纯俯仰、组合输入、非法/释放非零、路径角度、真实入口默认拒绝。
- [x] 实现可选 pitch，工具本地旋转目标、固定参考路径限制、旋转进展到位检查，保留原平移行为。
- [x] 运行相关规划/到位/会话/API定向测试。

### Task 2: 网页及 VR 输入

修改 protocol/messages、FinitePositionInput、HoldControlPanel、FiniteXRInput 和 main 接线；新增按钮、四元数相对姿态、松手与 capability 回归。

- [x] 先失败测试：长按俯仰、混合意图、旧能力禁用、手腕相对跟随/追踪丢失/符号不变性。
- [x] 实现前后端一致的 pitch 字段，网页升降旁俯仰按钮，Grip姿态锚点；保留夹爪互斥及等待回执。
- [x] 前端全套、TypeScript和构建。

### Task 3: 完整离线验收

- [x] 真实 LM3 仿真模型规划和自然到位回归。
- [x] 实际页面及合成 XR → WebSocket → Fake 后端完成正反俯仰、释放及继续平移；不加载SDK。
- [x] 记录边界、实测结果和未验收事项；检查用户原文件未改；本地提交，不推送。

执行记录：后端304项、前端511项通过，TypeScript与构建通过。独立复核发现HOLD释放误故障，新增失败回归后修复；后续审查含组合步与默认关闭的真机 --allow-pitch 接线，无新增重要问题。各项作者取舍见设计文档的Ruling，完整证据与未验收项见 `docs/tool-pitch-control-2026-10-27.md`。
