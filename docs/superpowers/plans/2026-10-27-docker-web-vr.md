# 原厂 Docker 网页与 VR 控制实施计划

> **For agentic workers:** Use superpowers:executing-plans in the current session. User explicitly authorizes continuous simulator development/testing without per-step approval; real hardware remains excluded.

**Goal:** 完成用户确认的三项任务：独立Docker接入、网页/VR完整操作、异常与恢复验证。

**Architecture:** 保留自研仿真作为独立交付模块。增加明确标识为LEBAI_DOCKER的运行方式，固定SDK连接127.0.0.1:3030，复用有限目标执行/状态机，使用现有理论MDH模型并逐段对照原厂FK；不冒充Fake或真机。普通启动和真机配置不变。

**Tech Stack:** Python/FastAPI、现有lebai_sdk、Three.js/WebXR、现有测试框架、用户已安装的官方Docker。

**Spec:** 用户2026-10-27本轮确认的三项任务及AGENTS.md。执行细节由当前会话决定并记入本计划。

## Global Constraints

- 只控制已验证官方Docker仿真；无真实机器人地址、发现、真实端口或连接失败回退。
- 虚拟启动/准备/运动/夹爪/停止已获本轮授权，不再逐项询问。真机仍须以后单独授权。
- 正常松手完成当前有限步；异常、未知结果与stop_unverified不得伪造成功或自动清除。
- 原输入新鲜度、动作/关节限制保留；如暴露不合理流程，先复现、说明模拟专用边界再修复。
- 不推送，不提交原始日志、凭据或现场配置。

## Tasks

- [x] 1. 原厂Docker运行入口：独立适配器/标签、固定连接与容器核验、MDH/工具/准备位配置、默认预览CLI及启动；先写选择/隔离/模型测试，再实现并定向验证。
- [ ] 2. 网页和VR控制接线：解锁/独立准备、2mm平移、0.5度俯仰、渐进夹爪，共用有限输入；明确Docker环境提示；先回归再构建和实际本机Docker验证。
- [ ] 3. 连续操纵/松手/断连/异常/重新准备：产品前后端测试与实际Docker闭环，合成Quest验证；最终独立只读复核，记录完整通过与未通过项。

## Rulings / evidence

- 复用已有MdhPositionModel。三组不同关节输入的原厂Docker FK与该模型位置误差≤8.4e-17m、姿态误差≤3.9e-16rad；这是模型对照，不是真机标定。
- Docker工具偏置当前为零；模拟配置使用零TCP和独立准备位，不覆盖现场工具/准备记录。
- 仿真控制器初始STOP，需要在独立Docker启动流程中显式调用仿真start_sys并确认IDLE；不会向真实机器人发送启动。
- 本计划直接执行，沿用用户确认的目标和不逐项审批要求；完成后再报告证据，不追加设计审批轮次。
