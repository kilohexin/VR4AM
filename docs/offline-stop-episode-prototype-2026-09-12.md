# 离线停止流程原型

## 隔离边界

模型位于 `backend/tests/prototypes/stop_episode_model.py`，仅被测试引用。无 SDK、网络、异步任务、真实时钟或硬件动作；不接入 app，不替换现有 adapter、smoke 或 RobotControl。

输入为人工构造的事件与单调毫秒时间。输出是设计评审用符号，不是可执行命令。原型中的 200/700 ms 是固定演示预算，不是新的真机安全期限；真实截止时间起点与升级策略尚需单独设计。

## 两条独立状态

- 请求：NOT_SENT → SENT → UNKNOWN → RETURNED_LATE；及时响应则 SENT → RETURNED。
- 停止证据：未确认 / 已确认，可因后续位置变化或证据失效撤销。

超时、取消、断连、观察到位置变化均可锁存故障。迟到回复、IDLE、完整稳定证据均不自动清除故障。整个模型的 motion_allowed 始终为 false；它不是运动授权器。

稳定证据 `stable_window_complete` 由测试显式注入，表示独立验证器的结论，不是在本模型中计算的传感器验收。不能把此参数直接接到 raw_state==IDLE 或 qd==0，也不能把原型当作已经实现的物理停止判定。

## 重复收尾

同一个模型对象表示一次停止流程。首次 request_stop 输出 PROPOSE_STOP_MOVE，后续 shutdown/disconnect 复用该流程，不新增模拟请求、不重置截止时间。

“复用”不消除风险：fault_latched 保持，tick 持续输出 REVIEW_ESCALATION。该输出的含义是策略尚需评审，不是等待厂商期间允许真机继续运行，也不是自动省略安全停止。原型没有真实 stop_sys 发送策略，不可部署。

## 验证范围

覆盖及时/迟到回复、无回复截止、重复收尾、断连、取消、错误请求 ID、位置变化撤销确认、确认静止但 RPC 未决、截止边界及非法事件时间。

首轮 12 个测试在模型尚未实现时失败；实现后通过，再增加截止与复用边界用例。正常流程与故障锁存分别断言，不把“测试通过”表述为真机停止问题修复。

本轮验证：`python -m pytest tests/prototypes tests/commissioning tests/robots/test_lebai_stop_evidence.py -q --tb=short`，91 passed（26.48 s），其中原型 18 项。未运行全后端套件。生产 app 无对本原型的引用。

## 尚未实现、不能推出的事项

- 没有真实控制盒任务排队、系统停止或制动模型。
- 没有多流程并发仲裁、稳定窗口传感器验证、断线重连或人工解锁。
- 没有确定何时重发 stop_move、何时发送/重复 stop_sys。
- 没有解释 B01 的 22 mm 位置反馈变化根因。

下一步应在厂商接口语义确认后评审安全升级规则，再决定是否将经过验证的部分接入生产。现阶段真机运动测试继续暂停。
