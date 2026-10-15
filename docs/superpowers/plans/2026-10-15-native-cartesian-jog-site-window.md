# Single Native Cartesian Jog Site Window Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans in the current session. Steps use checkbox (`- [ ]`) syntax for tracking. This plan does not authorize robot access.

**Goal:** 实现默认只预览、需另行单次授权的 +Z speedl 真机诊断入口，并在 Fake 与自有本机回环验证完整执行和原停止收尾。

**Architecture:** 保留原本机候选的地址隔离，抽取其已验证的单次发送核心；新增专用现场 scope、受限代理和执行器。共享原运动所有权与停止事务，预检和数学结果不能自行授予真实动作权限；生产网页/VR 不选择新路径。

**Tech Stack:** 仓库 Python 3.12、asyncio QPC 循环、pytest、NumPy/SciPy、既有固定 SHA 乐白 SDK；不安装或升级依赖。

**Spec:** [已认可的 speedl 设计](../specs/2026-10-15-native-cartesian-jog-design.md)，实施其第 6 节阶段 3 的软件准备。[模型接线基线](../../native-cartesian-jog-region-binding-2026-10-15.md)为 `3732ea8`。

**状态：** 待用户审阅本计划。沿用已选择的当前会话直接实施方式；批准本计划仅批准离线编码和验证。独立审查此前因工作区额度不足未执行，不声称已有独立审查通过。

## Global Constraints

- 首个固定候选仅为基座 +Z：`v=0.005 m/s`、`a=0.010 m/s²`、`t=0.080 s`，三个角速度为 0；它们是软件候选参数，尚未获现场动作授权。无任意方向/速度/时长 CLI。
- 关节比较限值不超过 `0.05 rad/s`、`0.2 rad/s²`；模型参考盒各轴半径 `0.005 rad`、起始关节偏差上限 `0.001 rad`。参考来自配置记录准备位，工具来自同范围冻结只读证据并与配置严格匹配。
- 软件工作区与显式名义 reserve 必须来自新范围文件并由未来现场批准；不能继承旧 ±5 mm 授权。速度×时长和名义 reserve 都不视作物理位移/制动上界。
- 本次属于有观察者的单次实验候选；理论模型和 13 FK 局部核对不等于控制器个体标定或整个区域的物理关节边界已证实。即使单次结果正常，`hardware_verified/joint_bounds_verified/braking_verified/continuous_control_admitted` 仍为 false，不开放持续控制。
- 保留运动输入/快照新鲜度 100 ms、速度调用等待 60 ms、停止 RPC 200 ms、稳定 300 ms、验证 500 ms；不延长，不重置截止。
- 一次连接、一次 speedl、仅实际进入速度调用后才准许一次原 stop_move；FK 最多 1、IK 0、速度倍率读取最多 1 且必须为 100，不修改倍率。所有 SDK 只读调用共享 256 次上限，总逻辑调用上限 259（含连接和两项写入）。
- 总本地预算 20 秒，连接最多 5 秒，普通读最多 200 ms，数学 FK 最多 1 秒且仍受总预算约束。发运动前至少保留 5 秒原停止与两秒只读尾段收尾预算；不足就拒绝，不进入运动。每个原生请求独立记录进入/完成/未知，局部取消不声称远端退出。
- 未进入速度调用时，断连/异常不追加停止写入。进入后撤权、过期、中断仍使用已预留的唯一停止权限；不恢复、复位、Home、准备位、夹爪、抓包、自动重试、stop_sys、pause_move、零速度替代或 PVAT 回退。
- 输出在仓库 `.tmp`，目录单次占用；配置、冻结报告、原始日志、SDK 二进制不提交。不推送、不启动服务、不访问真实机器人。新现场操作须另有精确范围及当天条件。

## Review Focus

1. 速度已进入后授权到期/源码变化：运动永久撤权，但预留停止仍可发送一次；任务 2/3 测试。
2. 只读请求不响应取消：不能等待读取才开始停止，迟到读取不得改写已封存报告；任务 2 测试。
3. 预检通过但速度实际 SDK 入口时快照、工具、配置或文件变动：零速度写入，不把本地拒绝报成远端未知；任务 1/3 测试。
4. 参考计时误用包创建时刻：段末必须按实际原生 speedl 进入时间，SDK 回复和观察读取不能续租；任务 2 测试。
5. 历史或外来报告、旧预览/确认、重复输出、并发连接：进入 SDK 初始化前拒绝；任务 3 测试。

## File Map

| 文件 | 职责 |
| --- | --- |
| `backend/app/commissioning/native_jog_core.py`（新） | 从现有候选抽取内部单次发送核心，保持原所有权和停止调用关系 |
| `native_jog_candidate.py`、`native_jog_model_guard.py`（局部调整） | 保留公开本机类的硬地址限制；共享纯模型校验实现，不新增 allow_real 布尔开关 |
| `backend/app/commissioning/native_jog_site_scope.py`（新） | 固定范围对象、匹配现场目标的模型 guard 和专用 adapter |
| `backend/app/commissioning/native_jog_site_client.py`（新） | 白名单/全局预算、一次速度/停止与有所有权的并行六字段读取 |
| `backend/app/commissioning/native_jog_site_run.py`（新） | 一次预检、FK/新快照绑定、定时速度/原停止和只读尾段 |
| `backend/app/commissioning/finite_motion_observation.py`（局部调整） | 增加明确的通用参考输入，保留旧 PVAT 缺省行为 |
| `scripts/real_robot_native_jog_window.py`（新） | 默认预览、身份/授权检查、固定 SDK/一次连接、落盘与进程退出 |
| `scripts/native_jog_site_loopback.py`（新） | 固定自有 127.0.0.1 原生 SDK、新 runner 四案验证，无目标地址选项 |
| 对应 `backend/tests/commissioning/test_native_jog_site_*.py`、`backend/tests/scripts/test_native_jog_site_entry.py` | 合成、权限、截止、未知结果与 CLI 无副作用验证 |
| `docs/native-cartesian-jog-site-window-2026-10-15.md`（新） | 实际实现/测试结果、尚未现场验收的限制与以后批准范围 |

## Task 1: 共享发送核心与显式现场范围（无连接）

**Interfaces:** 内部 `_SingleJogSubmission` 复用现有 `submit_jog` 与 `_submission_authorized`，提供 `_initialize_jog(settings, limits, config_sha256, model_guard)`；仅初始化既有状态、所有权和 guard 绑定，不创建工厂或连接。公开 `NativeJogCandidateAdapter` 继续只接受 127.0.0.1 与原 `ModelJogRegionGuard`。

`SiteJogScope(settings, limits, config_sha256, reference_tool, authorize, hot_check, write_check)` 为 frozen 范围对象：固定上述参数；校验 RFC1918 字面 IPv4 或唯一 `127.0.0.1`（仅本机测试使用）、control 配置、范围/关节上限、明确可调用权限函数及名义工作区。现场 CLI 的 prepare 另行严格要求 RFC1918 非本机地址。对象只能用于专用入口，不注册配置选择器。权限函数须正常返回 None，异常或 awaitable 均拒绝。

`SiteJogScope.model_guard()` 构造显式绑定同一 scope 的现场模型 guard；它共享模型校验主体，但不能被塞入旧本机候选。`SiteJogDiagnosticAdapter(scope, *, client_factory, clock, sleep, event_callback)` 必须显式注入工厂；构造/连接调用 scope 授权并核对目标与配置，提交固定包且只有一次机会。禁用其它运动入口，保持原停止独立。guard 内容与旧本机类一致，不通过假装 IP=127.0.0.1 再转发真实目标绕过旧限制。

现场 adapter 只接受带同一 `_jog_scope` 身份及 `_native_motion_entry_count(method)` 的受限客户端；任务1的合成客户端显式实现此协议，任务2的 SiteJogClient 提供真实实现。实际计数源为该原生计数接口，不能把进入 Python 代理当作进入 SDK 或误置远端未知；沿用 RealLebaiAdapter 已有的受信计数逻辑。

- [ ] **RED：** 写 `test_native_jog_site_scope.py` 和 `test_native_jog_site_adapter.py`：旧本机类仍拒绝所有非本机地址；新的现场类缺失 scope/工厂、public target、不同 guard/配置身份、超过固定参数均在工厂前拒绝。合成固定 +Z 只提交一次，原 stop 确认，未知速度结果/撤权/到期保留旧锁存。
- [ ] **运行 RED：** `.venv/Scripts/python.exe -m pytest backend/tests/commissioning/test_native_jog_site_scope.py backend/tests/commissioning/test_native_jog_site_adapter.py -q -p no:cacheprovider`，保存 XML，预期新接口缺失/行为未实现。
- [ ] **GREEN：** 抽取原核心且保留错误分类；现场 scope/adapter 仅通过显式来源绑定工作。新增并行运动快照方法：复用 RealLebaiAdapter 的字段解码和快照 owner，六字段并发等待，失败只撤销本组任务，不等原生取消；不改旧 PVAT 读取路径。
- [ ] **验证/提交：** 上述测试加现有 `test_native_jog_candidate.py`、`test_native_jog_region_binding.py`、`test_native_jog_region.py`、运动/停止/夹爪所有权五组回归，预期全部通过；本地提交。本任务不含真实 SDK 默认工厂。

## Task 2: 有预算的单次执行与停止收尾

**Interfaces:** `SiteJogClient(native, scope, *, owner, clock, motion_permission, stop_permission)` 复用 `BoundedStopOnlyClient` 的读取所有权，加入全局原生读预算、一次 FK/倍率预算和六字段组；白名单只增加 `speedl/get_velocity_factor/kinematics_forward`。所有其它写入拒绝。其 `.client` 为内部只读预算面，所有读取（包括 MotionObservation 的独立 reader）必须在同一 SDK 进入计数器中计费，不能从原始 client 绕过全局预算。速度/停止仅从专门方法访问原始 native。`speedl(a,v,t,frame)` 在实际 SDK 进入前校验固定参数，记录 `speedl_started_ns`、`speedl_entries`、原生结果状态；取消/迟到不解锁。`stop_move` 使用独立一次预留权限。

`run_site_window(scope, recorder, client_factory, *, clock, sleep, report=None) -> dict` 只连接一次。读取倍率100，原预检和记录位检查后，对实际关节做唯一 FK；读取新的六字段运动快照，仅 q/工具输入完全一致、FK 年龄<100 ms 且反馈位置≤0.1 mm/姿态≤0.1°才复用。其余模型/原预检检查均通过才创建固定包。FK 不匹配/过期没有第二次查询或回退。

读取/写入权限在锁前和实际入口均核对。`clock` 必须与所有 deadline 同一 QPC 来源。新 `MotionReference(started_ns, duration_s, authorized)` 是 frozen 数据；`MotionObservation(proxy, *, reference=None)` 无 reference 时完全保留原 PVAT 行为，提供 reference 时不读取 PVAT 字段并由回调确认本次速度正常返回。实际开始后中点最多一次 get_kin_data；50 ms 读限且不超参考结束，读取结果只作为观察。

段末按代理记录的实际 speedl 进入时刻+80 ms 计时；到点同步撤销观察、不 await 读取或取消，再发原唯一 stop。速度等待失败或中断立即进入收尾，不等段末。停止失败保留首要错误/锁存，仅追加最多两秒有预算只读尾段；尾段不能改成成功。adapter.disconnect 不得重发停止，也不等待不响应取消的原生请求。

- [ ] **RED：** `test_native_jog_site_run.py` 覆盖一次完整 Fake 流程、零写预检拒绝、256 读上限/FK及倍率各1、六字段重复/外来请求拒绝、延迟组未知、输入不同则不复用 FK、原生前到期；固定速度/时长不能经原始代理参数改写。
- [ ] **RED：** `test_native_jog_site_timing.py` 用独立受控事件/时钟验证实际进入时刻与包创建相差时，停止仍锚定实际进入；观察卡住不能阻挡段末停止；撤权/中断/过期仍至多一次原 stop，原生前拒绝无 stop；迟到报告不改变。原 `test_finite_motion_observation.py` 所有既有用例保持通过。
- [ ] **运行 RED→GREEN：** 上述两个新文件与 `test_finite_motion_observation.py` 定向执行、保存 XML，再实现最小客户端/执行器与兼容观察输入。错误码只输出本地定义原因，不透出原厂异常字符串。
- [ ] **验证/提交：** 运行任务 1 回归与本任务新测试，外加 `backend/tests/commissioning/test_finite_window_run.py`、`backend/tests/commissioning/test_finite_motion_observation.py`，所有受影响测试通过后本地提交。报告始终区分速度调用结果、原停止确认、观察结果、尾段及物理权限字段。

## Task 3: 默认预览的现场 CLI 与最终离线验证

**Interfaces:** `prepare(config, reference_survey, planning_limits, output, *, expires=None) -> dict`、`preview(plan) -> dict`、`execute(plan, *, approved, expected_hash, expires, confirm, client_factory=None) -> dict`。planning_limits 为固定速度/加速度下的显式 NativeJogLimits JSON 文件，盒与 reserve 必填，不以缺省余量冒充验证。reference_survey 及同目录 admission.json 都保存 SHA：须为完成的 LIVE_READONLY 六轴采集，13 FK、0 IK/写入、相同目标、明确工具匹配和有限局部导数；它仅支持局部一致性，不提升物理边界标记。

CLI 只有四个路径参数 `--config/--reference-survey/--planning-limits/--output` 与执行授权参数 `--execute-once/--approved-preview/--approved-preview-sha256/--authorization-expires-utc/--confirm`。默认只打印预览，不导入 SDK、不占用输出、不连接。执行确认固定为 `I_APPROVE_ONE_SPEEDL_Z_DIAGNOSTIC_AND_ONE_ORIGINAL_STOP`，旧确认不得复用；启动有效期 UTC 且≤10分钟，过期拒绝。

范围绑定配置/参考/规划文件字节 SHA、全部 backend/app 与本脚本、`local_rpc_connections.py`、SDK包及固定扩展 SHA `ad2a68619b25d627593bd5f13f83f17784e7a2c10aa9615db8bd1b681f9fd03b`。SDK 导入前和加载后检查原生路径/同一 SHA，随后才 init/connect(ip,False)；不是 SDK 启动/复位命令。执行前核对本地 numeric RPC 表为0，单次输出目录，私有副本和 admission 先落盘；任何失败都消耗本次目录。

完整哈希检查在连接/预检和进入运动租约前执行；快路径核对文件元数据、配置/批准预览字节和绝对有效期，实际写入前再作完整身份核对，并在其后检查原100 ms截止。输入变更/到期不能阻挡已预留的一次原停止；不能通过重建包续租。只读总预算与连接诊断独立记录，最终 report 在 finally 封存；终端只输出本地安全摘要。

- [ ] **RED：** `test_native_jog_site_entry.py` 覆盖默认预览零SDK/输出、错误/已消耗输出、旧确认/范围/期限/配置/来源变化、SDK导入前及加载后身份错误、已有RPC、参考为SIMULATION/不完整/外来目标、启动后撤权、工厂计数一次以及保留首要停止错误。
- [ ] **GREEN：** 实现固定入口，原生执行仅允许仓库 venv 与独立 QPC 循环。所有自动验证传入 Fake 工厂，标注 SIMULATION；不通过运行该 CLI 的 execute-once 做真机自测。
- [ ] **本机原生回环：** 新建 `scripts/native_jog_site_loopback.py`，仅接受 `--output PATH`，用当前测试公开的合成准备位加微小关节扰动（若该点的固定参数模型不满足，脚本输出模型拒绝，不调宽限值）；使用同一个新 runner、固定 SDK 与自己创建的 127.0.0.1 监听端，scope 本来就是本机类型，不改写真实目标。固定正常、速度迟到100 ms、stop迟到250 ms、观察卡住四案，核对每案≤1 speedl/stop、原生进入时间、未知结果、socket关闭。回环源固定参数不接受 CLI 覆写，输出文件存在时 SDK 初始化前拒绝。
- [ ] **最终验证：** 运行全部新测试加旧候选/模型/只读/有限窗口/停止与所有权相关回归，不跑全后端；TypeScript/Vite 演示包 `--build-only` 及 `--check` 通过。记录 SDK、结果/来源/代码哈希、用户原文件未变及所有实际失败。独立审查若仍不可执行，明确记录缺失，不用作者复核冒称通过。
- [ ] **交付/提交：** 仅提交代码、合成测试和说明，不推送。准备无期限、不可执行的本机候选范围和只读预览；不生成已获现场批准的记录，不连接设备。报告软件交付与现场未验收状态。之后才申请具体一次现场连接/动作，不以用户批准本计划替代。

## Self-review and Next Gate

旧本机地址限制与生产路径隔离由任务1保持；预算/并行读取/观察/原停止由任务2覆盖；授权、来源、SDK和落盘由任务3覆盖。三个任务需要顺序实施，执行方法沿用当前会话。只有软件与定向离线验证完成后才请求现场授权。单次现场通过也不能自动进入网页/VR持续控制或声称停止延迟已解决。
