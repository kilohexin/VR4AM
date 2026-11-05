# 真机VR PVAT抓取录像实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现并经现场验收VR真机连续PVAT平移、转腕、夹取轻物体及搬放录像；不以仅仿真或空载代替完整目标。

**Architecture:** 复用已有笛卡尔参考与完整尾段所有权，新增独立真机通道，工具由当天摘要固定的报告绑定。普通延迟可后期优化，但未知、路径及执行节拍失败不能忽略。夹爪先在机械臂已确认完成尾段后串行操作，完成后仅采用新输入，保持物体时不自动松爪。

**Tech Stack:** 现有Python/asyncio、LEBAI SDK及原厂Lua、WebXR/TypeScript。

**Spec:** 本聊天11月5日用户明确要求PVAT、连续控制、转腕与抓物；接受优化响应延迟后移。物体反馈：抓取部位约20mm宽、较轻，重量/材质/放置距离与桌面高度仍未具体确认。此文是软件工作计划，不是机器人动作授权。

## Global Constraints

- 原真实TCP精确值仅保存在本机；公开代码不写入实体配置或原始日志。
- 普通松手消费已拥有的尾段，不强制依赖stop_move；异常保留原停止与未知锁存，不能把一次IDLE或后来静止当作旧停止通过。
- Docker专用/proc时钟、docker exec、虚拟夹爪均不能直接迁到真机；旧位姿入口保持封锁。
- 每次物理连接/动作必须有具体范围与现场批准；当前仅已结束的一次SDK只读有授权，不包括Lua3011、运动或夹爪。
- 抓取工作区、搬运高度、载荷、软限位、方向与开合需要实体核验，不猜测扩大。

## Review Focus

- 控制器函数存在不等于调用语义或PVAT执行已经验证。
- 取消/断流时未来参考不得继续发布，未决写入不能通过取消线程消失。
- 夹爪完成/失败与手臂任务分开记录；未知夹爪不得自动重复闭合或开爪。
- 手腕反向时检查完整TCP路径，不以关节速度合规代替。
- 迟到输入不刷新时间，重新进入跟随不重放旧目标。

### Task 1: 独立真机通道无运动核验

**Files:** `scripts/preview_real_pvat_capability.py`、`scripts/real_pvat_capability.py`及对应`backend/tests/scripts/test_real_pvat_capability*.py`。

- [x] 固定程序、默认预览、独立批准摘要、单连接单提交及有界回执，不提供任意代码/端口/地址选择。
- [x] 测试正常、坏摘要、部分提交、跨输出/复制授权复用、损坏完成标记；12项通过。
- [x] 独立审查并修复授权复用及完成标记问题。
- [x] 当天明确批准后执行一次原厂3011无运动程序，核对状态/TCP/FK、五个时钟样本及基础函数类型；完整回执通过，TCP匹配、结束连接0、机器人写入0，详见`docs/real-pvat-capability-2026-11-05.md`。这不验证信号写入/PVAT执行。

### Task 2: 真机PVAT入口和控制器收尾

**Files:** 新建`backend/app/commissioning/real_cartesian_pvat.py`、`scripts/real_robot_cartesian_pvat.py`及定向测试；复用`cartesian_bundle_owner.lua`、`cartesian_host_planner.py`、`cartesian_tool_binding.py`。

- [ ] 依据Task 1结果确定可用运行时；编写先失败回归，绑定实体配置/工具/模型/连接/代次、读写预算与范围摘要。
- [ ] 先只支持一条有完整收尾的空载平移，再扩至固定TCP转腕反向；进入前装有完整尾段，所有迟到/断流/未决保持原隔离。
- [ ] 本机合成传输及隔离Docker实际程序对照；不能复用Docker特殊停止期限作实体证据。
- [ ] 提供可审阅单次空载范围，现场批准后执行；失败结束，不自动开启VR或重试。

### Task 3: 真实夹爪串行握持

**Files:** 现有`backend/app/control/robot_control.py`及实体夹爪执行/所有权测试；必要时增加独立`real_grasp_session.py`，不更改虚拟夹爪冒充实体。

- [ ] 先失败回归复现历史第四次夹爪迟到与unknown；有界单请求、原结果明确前不重发。
- [ ] X/Y按住时在原尾段完成后逐步开合；松开保持，不在负载未知/失联时自动开爪。夹爪动作期间不并发发布机械臂目标。
- [ ] 空载独立现场开口/方向/反馈核验；工具20mm抓取宽度不直接当作SDK百分比。

### Task 4: VR与完整录像会话

**Files:** 现有WebXR输入与协议；新增真机选择器（默认关闭），不解除旧Docker Pose封锁。

- [ ] 接入XYZ与俯仰/自转、Grip捕获/取消、X/Y开合，输入/退出流程回归及HTTPS头显链对照。
- [ ] 现场空载完整顺序通过后，按实际物体/路径生成包含负载的整体范围；批准后再做抓取—抬升—转腕/搬放—松爪录像。
- [ ] 保存实际动作、剩余位移/延迟、写入/未知及操作者收尾。未通过项明确记录，不保证一次过或提前宣布可拍摄。

## 当前执行记录

Ruling: 用户拍摄目标优先，跟手延迟优化后移；有限目标movej不替代用户明确要求的PVAT。代价：在真机PVAT收尾和夹爪验证前不能用旧网页入口交付完整录像。

Ruling: Task 1只用固定无执行器程序调查通道，不在核验中尝试信号写入或运动。代价：即便通过，写入/插值/制动仍需Task 2单独验证。
