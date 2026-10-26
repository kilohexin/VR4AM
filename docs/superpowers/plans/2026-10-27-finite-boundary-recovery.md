# 有限操纵边界恢复 Implementation Plan

> Use superpowers:executing-plans in the current session. User requested continuous offline development; no real access or software approval round.

**Goal:** 已准备的网页/VR遇到零写入的规划边界时，提示释放并允许新的反向手势，避免无谓重新解锁准备。
**Architecture:** 仅规划阶段可选的明确边界原因，且适配器能证明同连接/绑定/调用所有者、运动记录未变、无已接受运动/停止义务/在途写入/未知结果/故障才进入HOLD。预期工作区目标在IK前拒绝。任何证明不成立仍FAULT。网页与VR释放当前手势，必须新的实际按压/Grip释放重抓后才能继续。
**Tech Stack:** Existing Python adapter/session and TypeScript inputs/presentation; no dependencies.

## Short design and decisions

此前已允许零写入超时HOLD及俯仰上限HOLD；本次扩展到规划中的workspace、joint_limit、joint_jump、ik_unreachable、estimated_duration、path_rotation。不包含模型失配、无效模型、反馈速率、读写超时或未授权。输入/反馈100 ms、动作/停止门槛、预算、准备参考和工作区都不改，不自动缩小目标或重试。

备选是继续整机FAULT或自动裁剪/重试目标；前者打断正常控制，后者改变用户目标并增加动作，因此选显式零写入拒绝后等待新手势。作者决定，沿用用户已要求的连续离线实施方式。

## Tasks / review focus

- [x] Backend RED→GREEN: 真实内存客户端目标越界零写入HOLD，释放静置后新反向步自然到位且绑定不变；未知/已接受/在途写入、执行阶段相同文本、模型故障不得降级。拒绝事件记录原因及零写入证明，记录失败仍FAULT。
- [x] Frontend RED→GREEN: 网页边界自动释放输入但不abort；VR保持Grip不会自动再发，必须物理释放再抓；显示边界/手势上限，故障提示优先；不借展示更新时间续租。
- [x] Actual local-only browser and synthetic XR: 准备一次→正常步→工作区边界→松开反向→俯仰/夹爪仍可用；检查console和本机Fake无SDK/未知结果。Browser plugin absent, use existing Playwright/Edge.
- [x] Related backend tests and frontend suite/build; independent read-only review; local commit, no push. Preserve protected user document and field logs.

Ruling: HOLD发布早于worker退休，释放可能在日志收尾中抵达。只在适配器无在途运动时保留释放，worker退休前不重启持按；固定运动期限不变。两次快速拒绝可能被遥测合并，故以可选blocked_input_seq区分拒绝决策，不依赖恰好看到planning。失败复现与修复、复核及最终515/319项结果见 `docs/finite-boundary-recovery-2026-10-27.md`。
