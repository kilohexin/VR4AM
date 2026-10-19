# 有限速度包持按续接离线实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 在显式Fake及自有回环中完成“新鲜持按输入续接、最新目标替换、松手撤权并原停止”的有界软件闭环，为长按控制补齐单次候选尚不具备的续接能力。

**Architecture:** 沿用已认可speedl设计，新增独立的最新输入门和仅loopback的续接适配器；原单次候选、现场入口及网页/VR选择器保持原样。首次发送仍需原IDLE预检，随后仅允许同一连接/代次已知运动ID的续接；不能全局放宽IDLE预检。本阶段每会话最多10次speedl、最多1秒，只有新输入才能续发，结束复用一次原stop_move事务。

**Tech Stack:** 已有Python、asyncio/QPC、pytest、固定SHA的原生SDK和自有127.0.0.1模拟端；不安装或升级依赖。

**Spec:** [已认可的原生笛卡尔平移设计](../specs/2026-10-15-native-cartesian-jog-design.md)，第3–5节规定持续输入及所有权，第6节规定分阶段验收。本计划仅补充离线续接验证，不进入第6节的真机或网页验收阶段。当前用户已选择当前会话直接实施；用户已批准全部三项离线实现及验证，现已完成；执行结果和取舍见下方记录。

## 当前证据与方案选择

- 1728c1d已阻止50 µm/80 ms/.02 m/s²这一已知不合理的静止起止参考。将其简单扩大至1 mm/600 ms会增加松手时可能剩余的参考行程，且旧构造器不允许该时长，不作为长按控制的直接替代。
- `_SingleJogSubmission`永久消费一次提交，且实际入口调用IDLE预检。因此目前speedl实现是单次诊断器，不能通过循环调用它得到持按控制器，也不能在原实例上清空consumed标记冒充新授权。
- 原厂说明速度运动可被新的运动命令替换；暂停不改变任务运行状态，stop_move也不能取消随后发出的运动。见[原厂运动接口](https://help.lebai.ltd/en/sdk/motion.html#linear-constant-speed-motion)。这些说明没有提供本项目所需的物理制动或晚到命令取消保证。
- 最新冻结暂停记录前6组PAUSED/RUNNING的报告关节速度为0、位置未变，19组验证期间关节位置也未变；任务首次FINISHED在验证起点后206.5701 ms。该数据不能证明物理静止，不能把原stop_incomplete改成通过，不能据此重算稳定锚点。
- 选择保留10–80 ms有限速度包，先完成离线续接；不增加暂停判据候选、不改变300/500 ms语义、不把长段点位控制冒充实时摇杆。实物跟随、约束/制动余量及停止现场证据仍是后续准入前提。

## Global Constraints

- 范围仅XYZ平移；旋转速度固定0、显式基座帧，速度向量模和加速度分别不超过配置上限，禁止t=0或缺省。
- 保留100 ms输入/反馈新鲜度、60 ms运动RPC、200 ms停止RPC、300 ms稳定、500 ms验证及未知结果锁存；不续租、不缓存造心跳。
- 仅字面127.0.0.1、显式注入工厂和LEBAI_FAKE标签；注入工厂是可信依赖，并非网络沙箱。每个会话最多10次实际speedl入口、从建立输入门起最多1秒。第10包仍按其固定期限收尾，之后原停止且禁止第11包；1秒截止直接撤权，不自动创建新会话。
- 最新输入队列容量1；只有严格递增序号及不倒退的输入时间可更新。每个输入最多消费一次；没有新输入就不自动重发。任一时刻至多一个原生运动写入在途。
- 松开、失焦/断线对应的revoke立即清空未发意图，禁止后续native入口；已进入的RPC仍按原所有权收尾，不能把取消当成远端取消。未知/迟到结果永久隔离本会话。
- 原单次候选继续一次消费，现场CLI继续原范围；不新增真实地址执行入口、不接入网页或VR、不改变夹爪/准备位/Home，不修改现场配置或原停止实现。
- 局部模型校验仅为理论模型约束，不能把joint_bounds_verified/braking_verified/hardware_verified/continuous_control_admitted写为true。所有回环输出明确非物理验收。
- TDD、定向回归，不运行后端全集。原始输出和配置留.tmp，仅本地提交，未获新要求不推送。

## Review Focus

1. 新输入替换发生在SDK锁等待与native入口之间：旧包零写入，只能选择仍有效的新包；任务1/2覆盖。
2. 新输入在旧RPC已进入后到来：不改变旧包的输入期限、结果归属或在途校验；任务1/2覆盖。
3. MOVING来自外部任务、未知ID或另一连接/代次：不能借“续接”跳过预检；任务2覆盖。
4. 停止快速返回后旧速度才生效，或运动晚返ID：保留stop_unverified，不恢复发送；任务2/3覆盖。
5. 保持最后一次持按但输入已过期、无新序号或会话达到限额：不能重发、延长或重建会话；任务1/2覆盖。

---

## Task 1: 最新输入门与有界会话

**Files:** Create `backend/app/commissioning/native_jog_stream_gate.py`; test `backend/tests/commissioning/test_native_jog_stream_gate.py`.

**Interfaces:** `JogStreamGate(*, generation: int, config_sha256: str, clock)`，固定10写入/1秒；`offer(packet: NativeJogPacket) -> None`、`take_latest() -> NativeJogPacket | None`、`require_before_entry(packet) -> None`、`mark_entered(packet) -> None`、`require_inflight(packet) -> None`、`finish(packet) -> None`、`revoke() -> None`。每会话一个正在处理的包和至多一个未发送最新包；阶段区分等待入口、已进入和终结，不能由新输入改写已进入包的授权。

- [x] **RED:** 测试连续offer序号1/2/3只取3，取出的包在native前被4替换时拒绝旧包；不改任何input_ns/expires_ns。相同/倒退序号、未来/倒退时间、代次/SHA变化、过期、布尔/NaN等输入拒绝。
- [x] **RED:** 取出包只消费一次；没有新offer时下一次take返回None；mark_entered后允许另一个最新输入等待，但第二包不能同时进入；revoke前后、刚好100 ms、刚好1秒、实际第11次入口均拒绝。已进入包不因新序号而续租或自动撤销其结果归属。
- [x] **验证失败:** `.venv/Scripts/python.exe -m pytest backend/tests/commissioning/test_native_jog_stream_gate.py -q -p no:cacheprovider`，确认缺失门/行为的失败。
- [x] **GREEN:** 实现纯输入门，复用原NativeJogPacket及validate_inflight_input，不引入SDK、网络或前端依赖。
- [x] **验证/提交:** 运行新测试与`backend/tests/commissioning/test_native_jog_packet.py`，保留初次失败，提交本任务。

## Task 2: 仅Fake的续接适配器与原停止闭环

**Files:** Create `backend/app/commissioning/native_jog_stream_candidate.py`; test `backend/tests/commissioning/test_native_jog_stream_candidate.py`。复用原RealLebaiAdapter/OwnedMotionCalls/ModelJogRegionGuard，不改变原单次类或普通预检。

**Interfaces:** `LoopbackJogStreamAdapter(settings, *, limits, config_sha256, client_factory, clock, sleep, event_callback, model_region_guard, backend_label='LEBAI_FAKE')`；`offer_jog(packet) -> None`、`dispatch_latest() -> int | None`、`revoke_stream(reason: StopReason) -> None`（异步）。前两者使用任务1输入门；revoke先同步撤权再等待原stop。connect启动唯一截止监听任务，disconnect撤权并收拢该任务；截止监听不能生成输入或运动。不提供通用submit_tcp/Home/prepare/gripper/恢复。

- [x] **RED:** 拒绝真实IP、缺省工厂、非Fake身份及不匹配模型/设置；第一包要求原完整IDLE预检。正常合成流程依次发送新输入，实际包数≤10、每包显式10–80 ms；中间不逐包停止，松开后恰好一次原stop，停止后再offer不能重开会话。
- [x] **RED:** 后续快照为MOVING时，仅同连接、同代次、已知正常返回的己方motion ID可以续接；未知/外部ID、PAUSED、错误状态、急停、工具变化、软限位/奇异、工作区、模型边界、未决读取或过期快照均拒绝并撤权。不能修改/伪造快照robot_state或running_motion来通过原预检。
- [x] **RED:** SDK锁前/锁后/原生任务入口各处撤权或输入替换；已进入后到达更新输入；60 ms晚返、无效ID、取消不生效、读取消或未知结果；任何情况不产生额外运动、重试或第二次stop，原停止失败不能被新会话状态覆盖。只有一次输入、随后不再调用任何适配器API时，截止监听仍须在原输入到期时撤权并原停止；第10包正常收尾不被写入计数提前变成未知，第11包不能进入。
- [x] **验证失败:** `.venv/Scripts/python.exe -m pytest backend/tests/commissioning/test_native_jog_stream_candidate.py -q -p no:cacheprovider`。
- [x] **GREEN:** 同一手势只建立一个运动epoch；复用原SDK锁及运动调用所有权。首次原预检不放宽，己方续接检查逐项保留状态以外的全部前提，并校验真实原始状态及已知ID归属。每包原生入口重查最新输入、模型、快照和身份，已进入后保持该包固定截止。截止监听基于最后真实输入的固定到期时间及会话截止，offer仅唤醒重算，不修改旧包期限；结束等待原停止并释放监听任务。
- [x] **验证/提交:** 新测试、原`test_native_jog_candidate.py`、`test_native_jog_packet.py`及`backend/tests/robots/test_motion_call_ownership.py`、`test_stop_transaction.py`、`test_stop_timing_boundaries.py`定向通过；旧单次实例仍只能发送一次。提交本任务。

## Task 3: 自有SDK回环与阶段交付

**Files:** Create `scripts/native_jog_stream_loopback.py`; test `backend/tests/scripts/test_native_jog_stream_loopback.py`; results `docs/native-jog-held-input-offline-2026-10-19.md`。

**Interfaces:** CLI仅`--output PATH`，路径必须尚不存在；`async run_matrix() -> dict`创建自己的127.0.0.1随机监听。沿用固定SDK SHA和QPC循环，只能发送合成预检/运动反馈读取、有界speed_linear及一次stop_move（固定SDK实际协议名）；不接受IP、配置、端口或真实执行选项，不加载现场材料。

- [x] **RED:** 输出冲突、任意IP/端口/execute参数、SDK身份错误均在初始化前拒绝；监听端身份必须由本进程创建的唯一IPv4 loopback socket取得。
- [x] **RED:** 固定场景为正常持按松开、未发送目标被新输入替换、输入中断/超时、运动回复晚到、停止后旧运动晚生效、外部任务ID、工具/区域变化、停止回复迟到。验证服务端实际接收顺序和参数；控制续接拒绝与原停止结果分别记录；外部ID/工具/区域变化可以拒绝续接且原停止通过，迟到与晚生效仍保留原fault，不以快速回执代替停稳（执行取舍）。
- [x] **验证失败:** `.venv/Scripts/python.exe -m pytest backend/tests/scripts/test_native_jog_stream_loopback.py -q -p no:cacheprovider`。
- [x] **GREEN:** 复用现有`native_jog_loopback_probe.py`已核对的协议fixture和关闭流程，新场景不放宽旧peer白名单。每场景独立门/连接，真实连接0，每个包都来自合成新输入事件；无广播、发现或任意RPC。
- [x] **回环验证:** `.venv/Scripts/python.exe scripts/native_jog_stream_loopback.py --output .tmp/native-jog-held-input-20261019/sdk-matrix-unique.json`。输出必须标注offline_only、原生调用数、实际间隔/期限、原停止结果、晚到结果及资源释放；不声称控制器限时/制动或真实关节边界通过。
- [x] **交付:** 定向复跑三个新文件、任务2旧用例、原单次/现场隔离用例与1728c1d参数拒绝用例；不运行全集或无关前端构建。保留偶发和预期失败；独立复核后提交代码及无敏感信息的阶段报告，核对用户原总结未改，不推送。

## 完成口径与后续门槛

本计划交付的是可观察的离线多包控制闭环，不能称为网页/VR真机运行、运动中停止修复或抓取完成。生产选择器和真实地址始终不可选，原单次授权全部已消耗。只有后续明确的现场范围、关节/工作区及制动余量证据、实际方向/速度跟随和原停止行为得到验收，才能另行计划网页接线。

自审：原设计中的真实动作前提在本阶段仍保持未满足/禁用，不提前实施网页阶段。五项Review Focus均分配测试，所有接口由任务1/2明确产生；任务3仅消费它们。执行方式沿用当前会话直接实施，用户已审阅，全部三项及定向验证已完成。

## 执行完成记录

- Task 1：`8278f5c`，输入门及原速度包61项通过；随后补充输入到期永久关闭和不可变签名边界。
- Task 2：`5c31172`，适配器及原单次/所有权/停止150项通过；保留模型守卫零报告速度/加速度限制，合成MOVING/己方任务ID只证明协议续接。
- Task 3：`41ca8af`，27项脚本回归及九场景自有SDK回环；作者修复`3bbb7f6`，独立复核两项P2修复`0317ffe`。
- 最终319项定向回归、87项原入口隔离回归通过，九场景最终身份回环符合预期；未运行后端全集、未连接真机、未推送。
- [完整结果与执行取舍](../../native-jog-held-input-offline-2026-10-19.md)记录早期失败、两项复核修复、最终原始材料路径/哈希和限制。

完成范围为**零速合成反馈下的离线协议续接**；原模型不接受非零速度起点，真实连续运动、动态关节约束、制动及停止仍未验收，不能从以上勾选推导为真机网页/VR可用。
