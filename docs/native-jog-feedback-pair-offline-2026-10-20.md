# 单次 speedl 的 FK／新快照配对读取：离线结果

日期：2026-10-20。实现提交：`a9dec1c`，基线：`22d5ce9`。

## 范围与结论

按用户对 `call_b5695f186ad247e7affe79237da8198f` 的批准，完成现有单次 speedl 入口的局部调度修复、定向回归、真实源码规模的注入 Fake 检查和自有 SDK 回环。真实机器人连接数为 0；本地提交，未推送。

原现场拒绝记录见 `native-jog-after-prepare-result-2026-10-16.md`：FK 返回后串行等待新快照约 73.19 ms，使 FK 在实际发送入口超过原 100 ms。此次把这两项读取配对并行，减少该串行等待。**离线结果不能保证现场准入成功，也没有解决或验收历史运动中停止、跟随和速度反馈问题。** 网页／VR 连续真机控制仍未准入。

## 实现与约束

- 在专用 `SiteJogClient` 内加入一次性配对：唯一 FK 使用独立读取 owner，新快照沿用现有六字段并发组。未放宽公共 `FiniteScopeClient` 的排他规则。
- 实际 FK 入口必须由其 owner 的 pending task 调用，参数必须等于固定关节输入；共享原生读取总预算仍为 256，FK 总数仍为 1。
- 两项必须全部已知完成；未决、失败、取消、撤权和迟到结果关闭读取门。配对不能重用，失败不重查、不重试；不能通过另一个回调夹带第二次数学读取。
- 配对前固定 client、快照 owner 和 generation，在汇合及实际运动核验时检查。新快照关节与工具仍须和 FK 输入逐项一致。
- FK 年龄从实际原生调用返回时间起算，本机解析或权限检查不能把结果变新。实际发送前 FK 与反馈均须小于原 100 ms。
- 预览新增 `read_scheduling_policy=one_fk_paired_with_one_fresh_motion_snapshot_independent_owners`，执行仍绑定源码／SDK／配置。缺少该策略的旧批准材料在调用连接工厂之前拒绝。

| 项目 | 保留值 |
| --- | --- |
| 唯一运动 | 基座 +Z，speedl 一次 |
| 请求速度／加速度／时长 | 5 mm/s、10 mm/s²、80 ms |
| 数学调用 | FK 1、IK 0 |
| 运动回复上限 | 60 ms |
| 原停止 | stop_move 最多一次，原 200／300／500 ms 判据 |
| 收尾 | 最多两秒只读尾段；未知结果与故障锁存保留 |

原有 SHA 前置、写前文件身份／元数据检查策略继续沿用；它不能识别刻意保留元数据的文件内容篡改。本次未改该策略及其边界。

## 验证

### 定向回归与复核

最终 15 个相关测试文件 **299 项通过，44.91 秒**，见本机 `.tmp/native-jog-feedback-pair-20261020/targeted-final-second.xml`。覆盖配对、site scope／runner／adapter／timing／preflight、packet／candidate／inflight／region binding、CLI／SDK 回环判据、motion ownership、stop transaction 和 stop timing boundaries。

新增配对测试含真实异步交叠、关节／工具变化、双侧读取失败、拒绝取消后的迟到、外来读取、用户撤权、跨代次汇合和原生 FK 时间戳。作者补充的两项回归先复现再修复：旧代次 FK 不得和新代次快照汇合；本机权限检查的耗时不得重置 FK 时间戳。独立只读代码复核未发现可操作的新增缺陷；该复核未运行 SDK 或测试。

首次相同范围批量回归为 **298 通过、1 失败**：注入 Fake 的完整 CLI 场景在发送前拒绝，speedl／stop 均为 0，FK 汇合时年龄约 79 ms。单项随后通过，完整重跑得到上述 299 项通过；期间未修改产品或测试阈值。该单项使用默认 pytest 事件循环，现场 CLI 和下述两个探针使用现有 QPC 循环。**尚未证实该波动仅由事件循环引起，不能据重跑通过声称时序问题彻底消失。** 失败结果已单独保留，未删除或计作通过。

早期实现还因重复权限读取占用预算造成三项 CLI 回归失败。修复仅移除配对 owner 轮询中的重复文件／输入工作：每个实际原生字段、数学 owner 起止、组边界和汇合仍保留授权核验；写入前所有检查不变。此后完成上述最终回归。

### 源码规模的 Fake

使用本机既有 `run_source_bound_fake.py`，绑定实际 116 个运行文件；SDK 加载器被明确禁止，注入 Fake，结果为 `SIMULATION`。最终逐项核对三个预览的 116 个源码哈希均匹配当前文件。

| 本机目录（均在 `.tmp/native-jog-preflight-20261016/` 下） | 结果 |
| --- | --- |
| `paired-feedback-20261020-01` | 完成；FK 核对时年龄 40.33 ms；Fake speedl／stop 各 1 |
| `paired-feedback-delay25-20261020-01` | 拒绝；FK 年龄 119.53 ms；写入均 0 |
| `paired-feedback-delay25-20261020-02` | 独立运行完成；运动回复注入 25 ms；FK 核对时年龄 40.66 ms；Fake speedl／stop 各 1 |

第一次带延迟检查的结束阶段与回环启动有重叠，不是严格隔离的性能测量；不能将失败全部归因于并行进程。第二次在前述进程结束后单独运行。两种结果都说明原过期拒绝仍有效，不能推导现场或 Windows 的最坏时延保证。

### 自有原生 SDK 回环

原生 SDK 0.3.13，二进制 SHA-256：`ad2a68619b25d627593bd5f13f83f17784e7a2c10aa9615db8bd1b681f9fd03b`。只连接本脚本建立的 `127.0.0.1` 监听端，固定合成反馈，没有物理机器人或制动模型。

最终 `native-matrix-third.json` 的 **10 个场景全部符合预期**，包括正常、预检延迟、运动／停止回复迟到、观测未决、发送后 FK 自然老化、在途撤权、停止后晚生效、FK／快照各注入 25 ms，以及 FK 迟到 250 ms。负面场景正确失败，不是十次运动成功。

- 25 ms 配对案的实际线上 FK 与 `get_kin_data` 请求／回复区间交叠；汇合时 FK 年龄约 2.85 ms，唯一运动与原停止完成。
- FK 迟到案在配对期限结束后才收到服务端回复，读取结果未知保持；speedl／stop 均为 0，无重试。
- 自然老化案实际发送时 FK 年龄约 75.05 ms、运动回复时约 121.45 ms；验证了已认可的“发送前新鲜度／已发送 RPC 收尾”分阶段语义。未因旧结果自然老化追加新发送或续租。
- 十案均本机连接关闭、协议错误为空。迟到、撤权和晚生效仍触发原失败／锁存路径。

前两版矩阵均为 9／10 符合预期，失败在老化场景的测试注入：先在输入建立后延迟导致输入正确过期，再用固定延迟导致发送前正确拒绝。最终仅将该探针的注入时点改到输入建立之前、按已过去的 FK 年龄补足等待；产品门槛未变。前两份结果完整保留，不能作为该场景已通过的证据。

## 本机证据索引

原始材料、配置、合成批准材料均仅留本机。以下路径相对仓库根目录；哈希是实际文件字节哈希。

| 文件 | SHA-256 |
| --- | --- |
| `.tmp/native-jog-feedback-pair-20261020/native-matrix-third.json` | `c1842f564fdc7dc5514b28727234a45874695f76081b2d3d95de3a38ec81a6ed` |
| `.tmp/native-jog-feedback-pair-20261020/targeted-final-second.xml` | `a78779a55070fe02a048a20514847270057b6155193f792e842c87bce7f51373` |
| `.tmp/native-jog-feedback-pair-20261020/expanded-first-failed-cli-result.json` | `3cd701ab82075100ff1a336ddf93b2e8950467345485cac680bd1a9b0c071a46` |
| `.tmp/native-jog-preflight-20261016/paired-feedback-20261020-01/result/result.json` | `e1a8f195a21b1b127db56657822e78fc436b9a3e7cee4d6b56a4b9a3e9ad819e` |
| `.tmp/native-jog-preflight-20261016/paired-feedback-delay25-20261020-01/result/result.json` | `c19a054ee1e28b3c78c4c3fc0c21649226b2c6926d6711a41b4feea6cc84c79f` |
| `.tmp/native-jog-preflight-20261016/paired-feedback-delay25-20261020-02/result/result.json` | `05efb6e950f607fe59a9108e39f16c87bfa938b3d1c1587a67e096f85870ebd7` |

`.tmp/native-jog-feedback-pair-20261020/final-identities.json` 还记录七个改动源文件、前三版矩阵、两次批量回归和单项复查的哈希。矩阵本身未内嵌全量源码哈希；该清单是事后同工作区的身份核对，源码规模 Fake 的预览则绑定执行代码哈希。

## 当前交付位置

本阶段离线修复完成。单次现场入口已有新调度和独立读取隔离，默认仍只预览；旧现场批准不适用于当前代码。需要新的明确现场范围及当天条件后，才能做一次真机验证。历史停止与跟随故障、真实连续操作和抓取视频仍未验收，不自动清理 PAUSED、不恢复或回位。
