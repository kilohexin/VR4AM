# 六轴 FK 只读差分入口

## 交付范围

在现有 `scripts/real_robot_readonly_kinematics.py` 增加固定的 `--survey jog-jacobian`。默认仍是原 `fk-ik` 模式，旧查询白名单和调用上限不变。新模式只为采集控制器局部运动学导数，不进入生产控制、网页或 VR。

本轮完成了 Fake、实际安装的原生 SDK 本机回环及本地复核，未连接真实机器人，未推送。

| 项目 | 新模式固定范围 |
| --- | --- |
| 连接 | 最多一次，失败也消耗输出目录 |
| 查询预算 | 15 秒本地预算，单次读取 1 秒、连接 5 秒，均受总预算约束 |
| 调用总数 | 最多 27 次含连接调用 |
| FK | 基准 1 次；六关节各 ±0.001 rad，共再 12 次 |
| IK | 0 次 |
| 机器人写入 | 0 次；无运动、停止、启动、恢复、复位、Home 或夹爪操作 |
| 自动重试 | 无 |

±0.001 rad 仅是 FK 数学输入，不改变机器人关节。13 次 FK 使用同一个初始关节参考点，不在查询过程中追随新的姿态。

## 准入与失效处理

新模式有独立确认字符串，原 FK/IK 确认不能授权它。执行仍要求 readonly 配置、完整批准预览与 SHA256、源码/配置身份一致、十分钟以内的明确 UTC 启动期限，以及无既有机器人 RPC 连接。预览模式不导入原生 SDK、不连接设备、不占用执行输出目录。

SDK 二进制固定为 `ad2a68619b25d627593bd5f13f83f17784e7a2c10aa9615db8bd1b681f9fd03b`，在导入前核对预览身份和当前文件，导入后核对实际包/扩展加载路径及同一 SHA，随后才准许初始化和连接。身份不匹配分别报告 `jog_survey_sdk_binary_mismatch` 或 `jog_survey_loaded_sdk_mismatch`。

连接后一次 `is_connected`；前后各读取状态、急停原因、运行 motion、关节/TCP、工具 TCP 和速度倍率。只接受 IDLE/PAUSED、明确无急停、整数 0 的 running motion、报告关节速度全零、合法加速度及工具匹配；所有数学查询点须在带余量的配置关节软限内。后观测要求状态/工具/倍率不变，关节变化不超过 0.001 rad，TCP 变化符合原只读容差。它们是数据一致性检查，不宣称物理停止。

本地预算到期、撤权、中断或无法取消的读取均终止本次等待，不追加查询。未知远端结果保留；迟到数据不能改写已返回报告。入口不调用适配器断连或机器人停止来收尾。原生连接释放仍需要独立进程退出后另核对，不能从本地取消推断。

## 输出及结论边界

`backend/app/commissioning/readonly_jog_jacobian.py` 输出 13 组原始 FK、前后观测、控制器中心差分雅可比和理论 MDH 雅可比。角速度导数用基座系 `log(Rplus × Rminus⁻¹)/(2h)`，不是欧拉角逐分量相减。

报告的 `complete` 和 `jacobian_collected` 仅表示本次固定采集与前后数据核对完成。无论导数或理论模型多接近，`motion_authorized`、`hardware_verified`、`hardware_stop_verified`、`joint_bounds_verified`、`braking_verified`、`continuous_control_admitted` 均保持 false。工作区导数上界、运动中 Jdot、个体标定和制动/停止行为仍需后续证据。

## 原生 SDK 本机回环

原厂 v0.3.13 的 [posture.rs](https://raw.githubusercontent.com/lebai-robotics/lebai-sdk.rs/v0.3.13/src/rpc/posture.rs) 将 FK 转为 `get_forward_kin`；[kinematic.rs](https://raw.githubusercontent.com/lebai-robotics/lebai-sdk.rs/v0.3.13/src/rpc/kinematic.rs) 将速度倍率读取转为 `get_kin_factor`。独立白名单按固定版本源码和 lebai-proto 3.4.3 字段核对，仅接受既定 13 组关节参数和只读请求。

实际原生 SDK 0.3.13 只连接测试自己创建的 `127.0.0.1` 随机端口，完成 27 次逻辑调查调用，其中 `is_connected` 为本地 SDK 状态检查，所以线上 JSON-RPC 为 25 次。13 次 `get_forward_kin` 顺序和参数完全匹配；运动、夹爪、停止 RPC 均为 0。模拟端关闭后另外完成本地连接关闭核对。真实机器人连接数为 0。

合成回环中控制器差分与理论雅可比最大线性分量差约 `1.1162×10⁻⁷ m/rad`，角速度分量差约 `3.61×10⁻¹³`，数值秩为 6。这只是合成输入和协议核对，不是现场导数结果。

本机原始结果：`.tmp/readonly-jog-jacobian-20261015/native-loopback-result-01.json`，SHA256：

`01dd4ee34a0c54e54baf3def99ef3076db58291b9322080ac37308906e7980b0`

## 验证与复核限制

新增 38 项定向测试，覆盖 27 次调用/13 个数学输入、无写入、状态和工具变化、非法倍率/关节、软限、固定授权、SDK 身份、输出目录单次占用、预算、撤权、取消不响应和迟到结果冻结。连同原模式及速度包/所有权/停止/夹爪相关回归，最终 **372 passed**，不运行后端全集。结果保存在 `.tmp/readonly-jog-jacobian-20261015/final.xml`。演示包 `artifacts/delivery/segmented-demo/20261015T122115Z-0c4d26ca` 构建通过，沿用既有 Vite 大分包提示。

独立审查已尝试派发，但因工作区额度不足未能执行，没有独立审查通过结论。本轮改为作者本地逐项复核，并补齐两个 SDK 身份错误码的 RED→GREEN 用例；作者复核不能替代独立视角。

当前本机待授权材料：`.tmp/readonly-jog-jacobian-20261015/pending-preview.json` 和 `pending-scope.md`。预览没有启动期限，不能执行；输出目录尚未占用。只有收到新的明确只读范围和当前现场条件后，才生成一份有启动期限的最终材料。原控制配置和冻结日志保持不变，副本与原始输出不提交。
