# 单次 jog 本地拒绝诊断补强

## 范围及改动

用户在 `call_071fdcfdb5e94832bae8e41e67280414` 确认现有校验路径的小范围诊断设计后，完成离线实现，产品提交 **`ec0c381`**。本轮不连接真机、不推送、不调整控制门槛。

原 `validate_packet` 校验体未修改。新增包装只调用一次原校验，并传入调用方原先采样的同一个 now_ns；只有已发生的、固定白名单中的本地 ValueError 才附加结构化诊断。不重新采样时间、不更新快照、不续租、不新增读取、不重试。

接入的校验位置为：提交前、获得 SDK 锁后、独立 RPC 任务入口，各自的模型核验前／后，以及现场候选的 motion_permission。最终报告新增可选 `local_validation_rejection`：

```json
{
  "schema": 1,
  "code": "native_jog_snapshot_stale",
  "boundary": "site_motion_permission",
  "checked_ns": 3667887927649400,
  "snapshot_age_ns": 101256000,
  "input_age_ns": 39048300
}
```

示例来自本次完整源码规模 Fake 的自然过期，时间属于该进程的 QPC，不用于与其它进程／历史会话直接比较。年龄与拒绝使用同一个校验时刻，单位为 ns。

固定代码区分快照／输入过期、未来时间戳、数据包身份或期限不符、运动参数超限及名义规划框越界。原始本地 ValueError 文本继续保留在异常中以兼容原调用方；正式报告只接受精确的内部异常类型、固定代码／位置、固定字段和有界整数。返回的诊断字典是独立副本，畸形或外加字段不作为证据。普通厂商异常及伪造同名属性不会被升级为本地诊断。

本次覆盖的是上述发送前数据包校验点，**并非所有 ValueError、规划错误或模型拒绝都已增加年龄字段**。已有其它固定错误码继续沿原路径记录。

## 保留的控制行为

- 原 100 ms 快照／输入期限、60 ms 原生 speedl 回复期限及 80 ms 参考段不变。
- 原工作区、速度／加速度、模型／工具／关节核对、调用所有权、撤权、取消和未知结果隔离不变。
- 原 stop_move、停止判据和故障锁存不变。停止失败仍优先于此前运动错误，诊断字段不改变 primary_rejection、rejection_phase、complete 或 stop_confirmed。
- 正常结果不添加失败诊断；复用 report 容器时清除旧的同名诊断，避免残留证据。

## 验证

采用 TDD：初轮新测试因缺少诊断函数及接线出现 23 项失败；一个正常流程检查原本通过。初轮厂商异常检查错误地预期通用运动错误，实际原流程的 stop_unverified 应优先，因此修正测试预期，未改停止行为。

随后补测畸形诊断载荷、异常位置参数、时间采样保持及复用报告。其中发现并修复诊断读取器可能因畸形载荷报错、复用报告保留旧诊断的问题；校验次数测试最初把构建测试包时的原校验也计入，修正为先构建测试包再开始计数。所有红／绿记录保存在本机，未删除失败记录。

最终 **34 项新增检查通过**。连同相关 packet、candidate、site adapter／scope／runner、preflight、feedback pair、inflight、region binding 和 CLI 回归，**11 个文件共 272 项通过，42.63 秒**。未运行后端全集或无关前端构建。

新增检查覆盖精确的 99,999,999／100,000,000 ns 边界、未来时间戳、快照和输入年龄分离、六个核心校验位置、布尔权限层保留原拒绝证据、畸形／伪造诊断、原校验仅执行一次、厂商异常保密、停止失败优先和报告残留。

早期 pytest 默认缓存目录有 Windows 写入权限警告，最终运行以 `-p no:cacheprovider` 禁用缓存，272 项运行无该警告。一次初始命令将虚拟环境相对路径多写了一层，未运行测试；纠正后才记录红／绿结果。

独立复核 `/root/validation_diagnostics_review` 未发现可操作问题，确认原校验条件、采样点、所有权及停止优先级保留。该复核只读检查代码，没有独立运行测试，也没有连接网络／机器人／SDK。

## 完整源码规模 Fake 对照

所有运行均显式注入 Fake，使用完整 CLI／源码和冻结输入。117 个运行文件执行前后与各自合成预览一致；SDK 加载入口被禁止，网络 connect／DNS／bind 被审计阻断，网络尝试为 0、原生 SDK 导入为 0。TCP／FK 仍由本地纯 MDH 数学产生，不是物理仿真或实际机器人测量。

| 独立 Fake 运行 | 原始结果 | 新增证据 | Fake 运动／停止写入 |
| --- | --- | --- | ---: |
| diagnostic-normal | 正常配置自然过期，complete=false | site_motion_permission；快照 101.256 ms，输入 39.0483 ms | 0／0 |
| diagnostic-forced-stale | 在原拟注入点前已自然过期，complete=false | site_motion_permission；快照 103.5576 ms | 0／0 |
| diagnostic-revoked | 提交前撤权，complete=false | 保留 native_jog_offline_probe_revoked，不生成虚假的过期诊断 | 0／0 |
| diagnostic-controlled-after-model | 在权限层模型核验后人工等待约 38.19 ms，complete=false | 确认等待已执行；快照 101.942 ms，输入 47.7556 ms；site_motion_permission | 0／0 |

第二行未达到原拟受控等待点，不计作该注入点覆盖；另存第四行的明确等待标记及时间。所有输出均保留。单元回归中的正常流程通过，但完整源码规模的正常配置本次仍会因本机时序过期而拒绝，**因此本轮没有验证性能问题已解决**。

## 证据与后续

本机目录：`.tmp/native-jog-after-ready-site-20261020T093755Z/`，新增 `diagnostics-*.xml`、四个 `offline-profile-diagnostic-*/` 及 `diagnostic-final-identities.json`。该身份清单保存四个产品／测试文件、JUnit、各 Fake profile／result 的 SHA-256；原始配置、会话和诊断输出不提交。用户自己的项目总结未修改。

本次解决的是“发送前拒绝被归并为通用错误、无法区分快照／输入及具体校验位置”的诊断缺口。100 ms 窗口内的本机时序瓶颈、历史运动中停止和跟随问题仍未验收，网页／VR 连续真机控制仍未开放。下一步可依据明确位置优化本地核验开销；不以增加期限、刷新旧时间戳或重复现场微动代替优化。

前序：[离线开销调查](native-jog-admission-profile-2026-10-20.md)、[本次前的现场零写入结果](native-jog-after-ready-site-2026-10-20.md)。
