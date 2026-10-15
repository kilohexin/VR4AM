# Native Cartesian Jog Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans (recommended for this plan) or superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 完成默认隔离的speedl单次离线候选和固定本机回环协议验证，为新的真机范围提供可审查实现。

**Architecture:** 先建立不可变命令和纯校验器，再用专用候选适配器复用现有运动所有权及停止事务。适配器仅接受显式注入工厂及127.0.0.1配置；回环脚本仅连接自己创建的本机监听端。正常生产配置和网页/VR不选择新路径。

**Tech Stack:** 仓库Python 3.12、asyncio独立QPC事件循环、pytest、现有websockets及固定SHA的乐白SDK；不升级或安装依赖。

**Spec:** [已认可设计](../specs/2026-10-15-native-cartesian-jog-design.md)，用户已批准书面设计；本计划待审阅及选择执行方式。

## Global Constraints

- 本计划仅交付设计第6节的阶段1、2；真机、持续网页、VR/夹取分别留到后续批准范围与阶段计划，不把它们列为本计划已完成。
- XYZ平移，旋转速度固定0，显式基座帧；合成速度按向量模限幅，不让斜向超过上限。
- `a`为有限正数且不超过批准配置TCP加速度上限；`t`显式且为10–80 ms，禁止0、None、缺省和负值。
- 保留100 ms运动新鲜度、60 ms原生运动调用等待上限、200 ms停止RPC、300 ms稳定、500 ms验证；不得重启入队后的截止。
- 保留stop_unverified、原生未知结果、单在途写入、停止优先级，无自动stop_sys、pause替换、重试、复位、恢复或PVAT回退。
- 不把速度×时长当作物理位移界，不把笛卡尔低速当作关节限速证明；关节/制动余量证据不足时持续真机保持禁用。
- 原±5 mm单次工作区和已消耗现场授权不继承。不修改现场配置，不连接真实机器人；本计划不得创建可连接非本机地址的执行入口。
- 每任务按TDD先失败后实现，定向回归，不例行运行后端全集。原始日志、配置、SDK二进制、回环输出不提交；仅本地提交，未获新要求不推送。
- 仓库根目录为`C:/Users/Kilo/Documents/ChatGPT/Agent4ARM/vr4arm-offline`；命令在此目录执行，Python使用`.venv/Scripts/python.exe`。

## Review Focus

1. 布尔值、NaN、Inf及嵌套可变数组伪装成合法速度/时长/身份：任务1拒绝或冻结，不能传入SDK。
2. 斜向/边界输入与停止余量未知：任务1检查名义规划区，不宣称物理界；任务2拒绝非本机/未验证的真机入口。
3. 等SDK锁期间撤权、过期、代次变化：任务2在实际原生入口再次检查，零native speedl，不能续租。
4. native不响应取消、迟到ID、无效ID：任务2永久保留未知并阻止后续运动；任务3用延迟/错误回环再次检查。
5. 回环输出已存在、IPv6/多监听、CLI地址覆盖或错误SDK身份：任务3在初始化/连接前拒绝，不覆盖证据，不连接其它地址。

## File Structure

| 文件（仓库相对路径） | 职责 |
| --- | --- |
| `backend/app/commissioning/native_jog_packet.py`（新） | 不可变限值/包、单位/帧/时限、名义工作区校验 |
| `backend/app/commissioning/native_jog_candidate.py`（新） | 仅注入/本机的单次候选、权限、speedl所有权及原停止复用 |
| `backend/app/robots/lebai_adapter.py`（局部修改） | 将现有未知运动方法集合抽为可覆写类属性，原默认集合不变 |
| `scripts/native_jog_loopback_probe.py`（新） | 固定127.0.0.1、自有端口、真实SDK协议与适配器回环 |
| `backend/tests/commissioning/test_native_jog_packet.py`（新） | 任务1合成边界 |
| `backend/tests/commissioning/test_native_jog_candidate.py`（新） | 任务2Fake完整路径/竞态 |
| `backend/tests/scripts/test_native_jog_loopback_probe.py`（新） | 任务3端口、输出、协议、错误边界 |
| `docs/native-cartesian-jog-offline-2026-10-15.md`（新） | 三项结果、限制、后续真实准入缺口 |

## Task 1: 不可变有限速度包与纯规划校验

**Interfaces:** 在`native_jog_packet.py`提供frozen dataclass `NativeJogLimits(max_speed_mps, max_acceleration_mps2, tcp_min_m, tcp_max_m, planning_reserve_m)`；三位置字段均为长度3的有限float元组。`NativeJogPacket`字段为`velocity_mps: tuple[float,float,float]`、`acceleration_mps2: float`、`duration_s: float`、`generation: int`、`sequence: int`、`input_ns: int`、`expires_ns: int`、`config_sha256: str`。

`plan_jog(direction, strength, *, limits, actual_p, snapshot_ns, now_ns, generation, sequence, input_ns, input_expires_ns, duration_s, config_sha256) -> NativeJogPacket | None`返回不可变计划，零方向/零力度返回None；不是停止命令或真机授权。`packet.sdk_arguments() -> tuple[float, dict, float, dict]`生成`a,v,t,frame`全新字典，v含x/y/z/rx/ry/rz，旋转与基座帧六分量均显式0。

同时提供`validate_packet(packet, *, limits, actual_p, snapshot_ns, now_ns, generation, config_sha256) -> None`，供后续锁前/native入口使用当前实际位置重新校验；不能重建或修改包的expires_ns。strength只接受有限数值0–1，越界/bool拒绝，非零direction归一化后乘力度与上限。

- [ ] **1. RED:** 写`test_diagonal_speed_is_norm_limited`：方向(1,1,0)、力度1、速度上限0.005，断言速度模≤0.005、x=y；`test_positive_duration_is_explicit`参数化0/None/-0.01/True/NaN/Inf/0.009/0.081均拒绝，0.01/0.08接受。
- [ ] **2. RED:** 写`test_no_motion_for_zero_intent`、`test_expired_or_future_inputs_rejected`（快照/输入时间未来、年龄≥100 ms、输入自身过期、bool身份拒绝）、`test_sdk_arguments_do_not_alias_packet`、`test_nominal_box_includes_explicit_reserve`。期限为`min(input_expires_ns, now_ns+100_000_000)`，不反复生成新期限；SHA严格64位小写十六进制。
- [ ] **3. 验证失败:** `.venv/Scripts/python.exe -m pytest backend/tests/commissioning/test_native_jog_packet.py -q -p no:cacheprovider`；预期缺少接口导致失败，保存XML到本机新目录。
- [ ] **4. GREEN:** 实现上述接口，先校验结构/时长再处理中立输入，包acceleration取limits.max_acceleration_mps2。名义区检查覆盖当前点到`actual_p + velocity*t`线段两端及各轴显式reserve；命名/输出说明仅是规划检查，不生成`hardware_verified=True`或物理停止距离承诺。限值正数、各轴min<max，reserve非负有限，包字段不保留调用者可变列表。
- [ ] **5. 验证/提交:** 同一命令全部通过，`git diff --check`；仅提交本任务模块和测试，提交说明`feat: add immutable finite Cartesian jog packets`。

## Task 2: 单次Fake候选、原停止与未知结果隔离

**Interfaces:** `NativeJogCandidateAdapter(settings, *, limits: NativeJogLimits, config_sha256: str, client_factory, clock, sleep, event_callback)`继承`RealLebaiAdapter`；必须显式工厂，`settings.ip`必须为字面127.0.0.1，禁止真实地址/域名；limits不得超过settings内TCP速度/加速度及工作区，配置SHA在构造时固定。暴露`async submit_jog(packet: NativeJogPacket, *, authorize: Callable[[], None]) -> int`及原preflight/stop/disconnect。一次实例只有一次提交机会，失败也不重试；不注册生产配置，不调用默认SDK连接工厂。`submit_tcp/submit_catch_up/home/prepare/set_gripper`在此隔离候选拒绝，旧入口保持不变。

在`RealLebaiAdapter`抽取`_stop_blocking_motion_methods=frozenset({'move_pvat','set_claw'})`与`_immediate_stop_latch_methods=frozenset({'move_pvat'})`两个类属性，替换对应硬编码，保持旧语义。新子类分别增加speedl；复用`_await_motion_call`、`SdkLock`、原停止事务，禁止另写停止算法。

- [ ] **1. RED:** 写`test_injected_once_speedl_uses_original_stop`：Fake client扩展异步speedl，显式127.0.0.1，原preflight通过；speedl仅一次，返回正整数motion ID；随后唯一原stop_move及原稳定验证。记录SIMULATION，真实连接数0，禁止PVAT/Home/夹爪等其它写入。
- [ ] **2. RED:** 写`test_nonloopback_factory_refused_before_connect`、`test_authority_revoked_while_waiting_lock_has_zero_native_entry`、`test_packet_expires_at_native_boundary`、`test_unknown_speedl_blocks_motion_and_stop_confirmation`、`test_cancel_does_not_wait_for_stubborn_native`、`test_invalid_or_late_motion_id_cannot_restore_owner`、`test_old_pvat_and_gripper_unknown_policy_unchanged`。含bool/负数/非整数ID拒绝及未入native的本地拒绝不误报远端执行。
- [ ] **3. 验证失败:** `.venv/Scripts/python.exe -m pytest backend/tests/commissioning/test_native_jog_candidate.py -q -p no:cacheprovider`，预期缺少候选类而失败。
- [ ] **4. GREEN:** 提交时消费唯一机会，检查原控制预检、IDLE、新鲜实际快照、连接/代次/输入序号/固定期限与配置身份。将本次快照实例、read owner/序列和完成时间绑定到提交；native入口要求来源仍一致且无新增已知故障。锁前和native入口都调用authorize（必须正常返回None），并用任务1的validate_packet对绑定快照、当前generation和构造时SHA复查；不可重基期限。只有实际调用speedl前才计数并设置原`_motion_write_stop_required`；用entered回调区分本地拒绝。60 ms等待由原所有权管理；在operation内部验证返回ID，无效ID作为已进入native后的错误隔离。停止、撤权、断连不等待原生取消确认，不能因晚到ID重新解锁。
- [ ] **5. 验证/提交:** 运行新两个测试文件，加`backend/tests/robots/test_stop_transaction.py`、`backend/tests/robots/test_motion_call_ownership.py`、`backend/tests/robots/test_gripper_call_ownership.py`、`backend/tests/robots/test_gripper_sdk_entry.py`与`backend/tests/robots/test_gripper_snapshot_boundary.py`。仅受影响文件通过后提交`feat: isolate native Cartesian jog ownership behind loopback candidate`。

## Task 3: 固定本机SDK协议回环及阶段交付

**Interfaces:** 新脚本仅有`--output PATH`与`--repeats {1,2,3}`（默认1）；不提供IP、配置、端口、任意RPC或真实执行选项。`owned_port(addresses) -> int`只接受唯一IPv4字面127.0.0.1监听，端口只能来自`serve(...,'127.0.0.1',0)`。`async run_matrix(repeats: int) -> dict`在独立QPC循环下运行任务2适配器与原生SDK，配置与状态由本地合成，不读取现场控制配置。

- [ ] **1. RED:** 写输出存在/目录冲突在SDK初始化前拒绝、非法CLI地址无输出、非本机/多监听拒绝、SDK二进制SHA错误拒绝测试。先核对仓库venv中既有二进制SHA `ad2a68619b25d627593bd5f13f83f17784e7a2c10aa9615db8bd1b681f9fd03b`，导入后再核对加载路径/同一SHA，随后才init；发行元数据缺失须如实记录，可调用本地version()记录原生版本，不能把缺失当0.3.13或悄悄导入其它SDK。
- [ ] **2. RED:** 为本地peer写JSON-RPC白名单、请求ID唯一、speedl参数/显式正t/零旋转/基座帧校验，独立依据[官方运动文档](https://help.lebai.ltd/en/sdk/motion.html)与[v0.3.13封装](https://raw.githubusercontent.com/lebai-robotics/lebai-sdk.rs/v0.3.13/src/lib.rs)锁定协议fixture。仅允许speed_line、stop_move及原预检/停止读取；拒绝其它写入，不根据实测请求自行放宽白名单。
- [ ] **3. 验证失败:** `.venv/Scripts/python.exe -m pytest backend/tests/scripts/test_native_jog_loopback_probe.py -q -p no:cacheprovider`，预期脚本尚未实现而失败。
- [ ] **4. GREEN:** 实现脚本与peer，复用现有`application_stop_loopback_probe`已定义的合成状态/运动学字段和关闭流程；不改变旧peer。逐项运行立即正常、speedl迟到100 ms、无效ID、RPC错误、stop迟到250 ms五种案例。每种最多一次speedl和一次需要的原stop，不自动重试；拒绝前未进native的案例零运动写入。peer关闭、wait_disconnect及原生/包装任务归属均有输出；超时/晚返不称远端已取消。
- [ ] **5. 原生回环:** `.venv/Scripts/python.exe scripts/native_jog_loopback_probe.py --output .tmp/native-jog-20261015/loopback-unique.json --repeats 1`。每次选择不存在的输出路径；禁止重写旧输出。预期标注`offline_only=true`、`hardware_verified=false`、`real_robot_connections=0`，正常案例原停止通过；错误/迟到案例保留相应未知和未通过结果。这些失败案例是预期边界结果，不合并成真机验收。
- [ ] **6. 最终验证/交付:** 定向运行三个新测试文件及任务2引用的真实既有回归文件，保存JUnit与所有早期失败记录；若改动backend/app，重建并`--check`现有演示包。将实际来源、SDK身份、参数、案例结果、日志/包哈希固定在`.tmp`，校验操作者原未跟踪文档未变。写`docs/native-cartesian-jog-offline-2026-10-15.md`，明确本计划仅完成离线阶段，关节约束/制动余量/真实限时和停止行为仍未知，持续模式返回`native_jog_joint_bounds_unverified`。
- [ ] **7. 提交/复核:** 提交本任务脚本、测试及结果说明，排除SDK、原始输出和现场配置。进行一次独立整段代码复核，修复重要问题并定向回归，记录最终提交和本机冻结一致性，不推送。

## Scope Coverage and Next Gate

- 设计的输入、所有权、未知结果与单次协议由任务1–3覆盖；原准备位/夹爪和现有前端不变的回归在任务2/3核对，候选本身不调度它们。
- 本阶段没有真实机器人执行入口，不能把本机隔离开关当作真实关节约束证据。未来真实准入还必须解决控制器/局部模型的关节边界、制动余量与实际停止可靠性，并另定只读/单次运动范围。
- 网页长按、VR、自转和物体抓取属于设计的后续阶段；只有上述真实证据成立后才写相应阶段实现计划，不在本计划内假装完成。
- 本计划推荐当前会话由主代理顺序执行三个紧耦合任务，最后独立复核；用户审阅计划并选定执行方式后开始。没有授予新现场权限。
