# 有限时长 speedl：离线阶段结果

## 本次交付

按已批准设计与计划完成阶段 1、2：不可变速度包、一次性本机候选、真实 SDK 对自建 IPv4 回环端的协议验证。仅本地提交，不推送。没有连接真实机器人，没有开放网页或 VR 的 speedl 控制。

- 平移按方向向量模限速，旋转为零，显式零位姿参考帧；每包时长 10–80 ms，输入和快照须小于 100 ms，期限不续租。
- SDK 写入前再次核对权限、连接、代次、配置身份、快照及读取所有者。每实例仅一次提交机会，失败也消耗。
- 沿用原运动调用所有权、SDK 锁和停止事务；速度写入未知时保留 `stop_unverified`，晚到结果不能解除锁存。
- 生产适配器只抽取两个未知方法集合，默认行为不变；候选拒绝其它运动、准备位、Home 和夹爪入口。

## 原厂协议依据

[SDK v0.3.13 的 motion.rs](https://raw.githubusercontent.com/lebai-robotics/lebai-sdk.rs/v0.3.13/src/rpc/motion.rs) 将 `speedl(a,v,t,frame)` 转为 `speed_linear` RPC，字段为 `speed`、`param`、`frame`，`param` 含 `acc/time/constrained`。批准计划中 `speed_line` 与 `params` 的写法据此更正。

[同版本姿态转换](https://raw.githubusercontent.com/lebai-robotics/lebai-sdk.rs/v0.3.13/proto/src/serde/posture.rs) 将显式零参考位姿序列化为 `CUSTOM` 的零位置和零旋转。它在数学上与基座原点重合，不能误称为线上的枚举 `BASE`。字段及枚举另据 Cargo.lock 指定的 [lebai-proto 3.4.3](https://crates.io/crates/lebai-proto/3.4.3) 和 pbjson-build 0.9.0 核对，原始来源冻结在本机 `.tmp/native-jog-20261015/`。白名单没有因观测到未知请求而放宽。

## 定向验证

速度包 42 项、候选 22 项、回环守卫 30 项，连同既有停止/运动/夹爪所有权和边界回归，共 182 项。首轮回归中的 13 项失败也在基线 `331c7d9` 复现：12 项仍拦截旧 `asyncio.wait_for` 路径，一项仍期待已取消读取可以被新健康读取覆盖。本次仅更新测试拦截点与未知结果断言，未修改生产夹爪逻辑。

测试与早期失败的 JUnit 文件保留在 `.tmp/native-jog-20261015/`。不运行后端全集。

## 原生 SDK 回环

SDK 导入前后均核对本仓库 `.venv` 中二进制路径与 SHA256：

`ad2a68619b25d627593bd5f13f83f17784e7a2c10aa9615db8bd1b681f9fd03b`

本次实际发行元数据和原生版本都为 `0.3.13`。脚本使用独立 QPC 循环，每案仅连接自己创建的 `127.0.0.1` 随机监听端；CLI 无地址、端口或配置覆盖项。

| 案例 | 速度调用 | 原停止验证 | 保留结果 |
| --- | --- | --- | --- |
| 正常 | 返回 ID 7 | 通过 | 无锁存 |
| speedl 回复延迟 100 ms | 60 ms 上限超时 | 未通过 | speedl 未知、stop_unverified |
| 无效 ID 0 | 拒绝 | 未通过 | speedl 未知、stop_unverified |
| speedl RPC 错误 | 拒绝 | 未通过 | speedl 未知、stop_unverified |
| stop 回复延迟 250 ms | 返回 ID 7 | 200 ms 上限超时 | stop_unverified |

每案线上恰好一条 `speed_linear` 与一条 `stop_move`，未重试，连接已关闭。输出标记 `offline_only=true`、`hardware_verified=false`、`real_robot_connections=0`。

首次原生结果：`.tmp/native-jog-20261015/loopback-01.json`，SHA256：

`f8daa76d9744d9d2c71babf01153b86e22ff8cd88ec69b9d7119377bff2a258e`

## 下一道准入条件

这次证明参数序列化、应用调用所有权及合成停止边界。它没有证明控制器支持行为、实际限时结束、运动距离或真机停止可靠性。速度乘时长及名义工作区余量都不是物理位移上界；笛卡尔低速不是关节限速证明。

持续模式仍为 `native_jog_joint_bounds_unverified`。下一步需制定单独的真实准入计划，解决关节约束和制动余量证据，再按现场状态申请一次有明确范围的只读或单次运动；旧授权不继承。网页长按、VR、自转和抓取尚未在本计划交付。
