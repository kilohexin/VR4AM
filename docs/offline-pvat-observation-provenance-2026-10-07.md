# PVAT 参考与缓存关节观察的来源门禁

2026-10-07，起点 `4eb740bc9260c800f2ea181bd6db2d5520565455`。本轮生产改动仅为把现有 SDK 读取中的关节加速度发布到 `MotionView`；校验器和停止桥接实验仍在测试目录。没有新现场连接、命令、服务启动、配置修改、部署或推送。现场约 610 ms 的 `stop_move` 迟到仍未解决，200 ms 截止和 `stop_unverified` 保持。

## 生产只读数据补齐

`JointObservation` 增加可选 `actual_qdd`，旧的六参数构造保持兼容，缺失值为 `None`。适配器把已有 `LebaiSnapshot.actual_qdd` 复制成元组，同 q/qd 和原读取代次、时间一起发布，不增加 SDK 查询。生产读取原本已要求 `actual_joint_acc` 为六维有限向量，缺失不填零；本轮未改变解码或运动路径。

三种时间保持区别：`captured_ns` 在等待 SDK 锁之前，`read_completed_ns` 在本次整组读取完成之后，`observed_ns` 是本机读取缓存视图的时刻。这是本地读取区间，不是控制器内部传感器采样时间；不证明不同反馈字段在控制器内部同步。重复读取视图不会刷新原样本的时间、增加 I/O 或补发命令。跨泵代次的迟到读取不能更新公开加速度缓存。

## 测试专用资格校验

新增 `backend/tests/control/pvat_observation_probe.py`，使用显式 `ObservationBinding` 把 owner 授权代次、完整 `CommandRef` 与绑定起点联系起来。owner 代次和适配器泵代次分别比较，不能相互替代；命令 ID 相同也不能掩盖 submission ID 被替换。

纯缓存校验拒绝：

- 数学预测、末段参考对象，以及非声明 state/motion 用途的观察。
- owner 已撤销、未授权，泵故障、连接引用缺失、预检未就绪、正在停止或 `stop_unverified` 锁存。
- 视图／样本／命令的泵代次不一致，完整已接受命令被替换，未知命令记录。
- 非法或颠倒时间、未来时间、读取开始早于绑定、过龄缓存。新近读取完成不能隐藏等待 SDK 锁前的旧起点。
- 缺失加速度、非六维或非有限 q/v/a；不补零、夹值或外推。

合格结果保留样本原时间、owner／泵代次、完整命令，给出读取开始至检查时刻的保守帧龄和本机读取跨度。合成例用 owner=7、pump=3、读取跨度 15 ms、帧龄 30 ms；50 ms 上限只是显式实验值，不是现场参数或反馈新鲜度验收标准。绑定起点也由实验显式传入，尚无生产参考调度器为它提供执行证据。

此校验器没有接入运行策略、PVAT 发送或到点确认；它提供后续交接所需的测试约束，不授予运动权限，也不证明观察中的 q/v/a 对应某条参考的控制器执行时刻。数学末端 v/a 归零、一次合格观察或最后一次 IDLE 均不等于物理静止。

## 验证与失败记录

先补测试并观察缺失字段／占位实现失败；发布缓存加速度后，校验器占位仍产生 41 项失败。实现后相关 56 项通过。继续加入原停止桥接和锁存观察验证，171 项通过。随后三个边界测试出现 **3 failed / 42 passed**：`True == 1` 可冒充 owner／已接受命令代次，未知已接受对象触发 AttributeError。增加严格整数类型检查和对象类型检查后，三项均通过，不改变生产运动门禁。

本轮新增 **62 个案例**，12 个相关文件分两组验证，共 **436 passed**，均退出 0：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/control/test_pvat_observation.py backend/tests/robots/test_motion_view.py backend/tests/control/test_pvat_window_supervision.py -q --tb=short -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
# 174 passed, 5.82 s

.venv/Scripts/python.exe -m pytest backend/tests/control/test_transactional_runtime.py backend/tests/control/test_transactional_vr_tick.py backend/tests/control/test_transactional_motion_view.py backend/tests/control/test_catch_up_supervision.py backend/tests/robots/test_explicit_catch_up.py backend/tests/robots/test_stop_transaction.py backend/tests/robots/test_pvat_point_inputs.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/api/test_transactional_session.py -q --tb=short -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
# 262 passed, 17.60 s
```

其中过龄／跨代次／缺加速度三类拒绝 × 五类停止结果新增 15 项，经测试监督器进入原 `RobotControl → RealLebaiAdapter.stop`。确认、仍运动、异常、fake 610 ms 迟到和取消均保持原截止与失败锁存；无自动 `stop_sys`、Home、复位或第二次停止写入。测试使用注入客户端，没有真实网络／发现调用。实际缓存视图的注入客户端测试也核对非零加速度副本、撤销期间旧缓存保留，以及重复读取不增加 SDK I/O。

没有后端全集、独立审查、前端构建或真机测试。

## 本机证据

忽略目录 `.tmp/early-stop-window-20261007-l0erdkr4/` 的脚本及 JSON 不入 Git。独立脚本退出 0，核对九类合成拒绝、原日志／只读配置／生产 PVAT 构造器／上一轮末段输出的固定哈希；原始日志和配置未变。没有把合成 DTO 资格校验称为物理闭环回放。

| 项目 | SHA-256 |
| --- | --- |
| `evaluate_observation_provenance.py` | `1087a4054d52ec68a2287586c15db27290da6cc705042c372c92a93fb18c3364` |
| `observation-provenance-evaluation.json` | `261abd7760b97f180d10d561c0ae306a620f00d6323695acd32c4fbec6c3fbaf` |
| `motion_view.py` | `66fd4b9d4ac6615c9a0a1a0b768994b67cfce05f1bcf30c75204faf2cc40361c` |
| `lebai_adapter.py` | `eb756c3d81b929361e163d965f446e65559c2d92879785e30ea450cc17c067da` |
| 资格校验源码 | `9b97948330f9bf9fe7772ace1479a5b1a8ea284f1f257cb5aea7fd0a4c87f6e7` |
| 资格校验测试 | `9f738187c53d532c5d8124893456c4642ca9c66f3fffba7f70c4904dc77ee019` |

哈希按本机已保存文件的实际字节计算；Git 换行规范化后的 blob 字节可能不同。

下一步仍为离线参考／反馈时间对齐和末段状态交接：把合格观察同覆盖窗口比较，明确输入改目标的撤销／重新规划，以及末段截止附近等待反馈与超时故障的区别。需要单独设计生产接入，不能靠这个校验器清除 `stop_unverified` 或恢复真机测试。厂家插补、控制器执行起点和现场停止迟到均未解决。

后续见 [时间关系与末段观察交接](offline-pvat-time-alignment-2026-10-07.md)：默认拒绝时间未知，合成比较要求所有 q/v/a 共享时刻；实际有限末段仍无法在原截止前满足合成连续观察标准。
