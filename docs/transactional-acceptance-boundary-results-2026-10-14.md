# 接受回执与目标历史边界 — 2026-10-14

## 本轮交付与范围

用户要求继续下一步；此前现场条件不满足、继续离线仍有效。本轮只检查代码、现有冻结文件并使用显式假后端/客户端；原生 SDK 导入、真机连接/写入、网页/网络/系统配置操作、抓包和推送均为零。

修复明确选用的 `TransactionalTargetStrategy` 与 `CandidateCoordinator` 中的接受历史边界。默认 legacy 策略、实验选择配置与默认关闭的时序候选保持；没有启用、部署或改现场 profile。缓存接受证据、SDK 返回和假反馈不是物理执行、当前现场状态或停止证明。

## 已复现的历史不一致

首批七项有效失败：回执已返回后，缓存故障/预检失效/连接移除/停止进行中，或接受目标缺失、命令被替换、TCP 被改变，策略仍可能先提交内部位置、速度和版本，再返回故障或把旧目标交给控制层。

新路径先检查当前权限代次和缓存 view：连接存在、pump 当前、预检就绪、无停止进行中及已锁存故障；原 pump/锁存错误来源保留。随后收集回执时，完整 `advance` 必须同时匹配请求 Ref、接受 Ref 和候选 TCP，并保留原结果身份、invalidated、SDK outcome、版本与错误检查，才能提交一次历史。拒绝、partial/interpolated advance、catch_up、未知或旧回执不能冒充完整提交。

这是本地命令身份与 TCP 拷贝的一致性核对，未增加容差、推导真实 FK 或忽略分支限制。候选协调器的旧独立 receipt-only 调用合同保留；生产事务策略明确要求接受缓存关联。

后续又有六项有效失败：已经提交的接受目标被清除/替换/改变后，下一草案仍能沿旧历史计算或提交；同代次 on_anchor 先消费旧回执、随后才拒绝重捕获；旧 collect 上下文可以提交新代次内部历史。修复只保存一个不可变 `committed_command`，与已有历史 TCP 核对；prepare、实际提交复核及 collect 发现漂移即拒绝。同代次锚定在消费旧回执前拒绝，旧 collect 上下文不推进新历史。新代次显式重捕获重置命令关联，不自动重试或恢复旧速度。

## 完整控制链验证

新增 **17 项**用例：七类缓存/接受变化、完整匹配且仅提交一次、三类提交后漂移、同代次锚定、草案到提交之间接受变化、旧 collect 上下文，以及三项真实控制类配假 SDK 的集成用例。

集成用例通过额外公开 `submit_tcp` 真正替换假适配器的接受命令，没有改写私有接受缓存。原回执尚未收集或已经收集两种情况下，后续 owner tick 均保留原历史，仅执行原停止流程；写入序列为两条假 PVAT（owner 与独立提交）加一条原 stop，不产生新 candidate。正向两次完整 advance 则核对 owner `last_target`、coordinator pose/Ref 和 adapter accepted TCP/Ref 始终一致。

这些调用仅进入注入替身，不是“向真机插入其它命令”的测试或授权。仅缓存 Ref，不保留结果列表或追踪数组；1000 次提交的 weakref 和只保留当前 receipt 断言继续通过。该旧夹具现在发布当前 `AcceptedTarget`，不能再以接受缓存缺失假装满足新协议。

## 回归与失败证据

首批 **7 failed/1 passed**。初步实现六相关文件 **1 failed/189 passed**，唯一失败为上述 1000 次夹具没有发布接受缓存。后续五项历史替换有效失败，加入旧 collect 代次失败后 **6 failed/8 passed**；修复并补齐夹具后六文件 **196 passed，6.47 秒**。

首次扩大十三文件 **4 failed/342 passed**。四项旧追赶监督测试假定撤权的 PVAT 必定先 invalidated、后返回 SDK；已有有界调用可能先因 authority_revoked 结束等待并冻结 timeout/failed/remote unknown。用基线 `912738a` 两个控制模块的 Git 源码仅在内存重载，复现 **4 failed/1 passed/13 deselected**；未替换工作区文件或修改停止实现。

测试现在仅为确切的 PVAT timeout 情况核对：failed/sdk_timeout、无 accepted_command/pvat_mode、已进入且 remote unknown、authority_revoked、永久 unknown 方法、`stop_unverified`、confirmed=false、reset 拒绝；其他路径保留原 invalidated/returned 或 not_started 断言。旧 stale tick 可以传播确切 `stop_unverified`，它仍作为错误，不能计作停止完成。首次对齐仍 **1 failed/4 passed**，暴露该传播；补齐匹配断言后基线代码配调整夹具 **5 passed/13 deselected**。原不再追赶、历史不变、无新运动写入和只允许原 stop 的断言保留。

最终十三相关文件 **346 passed，10.19 秒**，无失败、错误或跳过，未运行后端全集。独立最终静态复审未发现本轮范围内 P0/P1/P2；审查未编辑、测试或访问 SDK/设备/网络/系统。证据目录 `.tmp/transactional-acceptance-20261014/`，原冻结文件、现场配置、SDK 和用户原总结保持。

## 主线位置与下一步

事务候选的“完整接受才能提交历史”关联现在覆盖收集前、提交后、实际再次提交和新旧代次边界。默认 legacy 的提交与上一轮软反馈语义没有在本轮替换；本轮不能宣称整体背压或真实平移幅度已解决，也不能把上轮时序候选默认启用。

下一步继续离线对照接受回执、追赶期间的新手部输入与显式重捕获，确定能够保持历史一致且不会累积旧运动的推进路径；保持实验策略明确选择及当前停止锁存。

真机仍需当前状态及真实 FK/IK 一致性、专用微量运动方向与原停止验证两组门禁；约 621 ms 停止回复、真实 FK/IK 超时、横向偏移、连续 Grip/Trigger 实机验收仍未解决或完成。本轮不请求上电、恢复或现场动作，不准备新的现场执行范围。浏览器物理抓取继续后置。
