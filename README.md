# VR4Arm — 网页与VR机械臂控制

更新：**2026年11月6日**。当前开发分支：`codex/vr-toggle-follow`。

项目包含网页/WebXR操控、官方L Master Docker适配、独立模型/Fake、运动控制与真机诊断。当前优先实现可用的**终点跟随和保持**：允许运动过程不完全复刻手部动作，拍摄暂缓。先Docker功能与真人体验，再独立真机验收。

**当前状态：`towardj`终点跟随仅在独立Docker诊断中通过，尚未接入完整真人VR入口。旧相对位姿PVAT模式因TCP路径偏差继续封锁。不要把历史启动命令、单项回归或Docker通过当成真机放行。**

## 从这里开始

- [当前项目总览与接续说明](docs/project-overview-2026-11-06.md)：目标、能力、架构、入口、问题和开发顺序。
- [终点跟随候选及实际证据](docs/endpoint-follow-2026-11-06.md)：三次towardj更新、末点到位与保持。
- [难点、延迟及解决方案](docs/project-difficulties-and-latency-2026-11-05.md)：计算、调度、停止与真实端到端测量的区别。
- [完整时间线](docs/project-status.md)：最新记录在顶部，较早结论保留其历史边界。
- [仓库执行规则](AGENTS.md)。

## 当前方案

| 层 | 职责 | 状态 |
| --- | --- | --- |
| 网页/WebXR | 手柄、摇杆、俯仰、自转、X/Y渐进夹爪、3D显示 | 已有历史实现；新终点链尚待接线 |
| 最新端点邮箱 | 合并未发送目标、单在途、松手后保留最后已接受目标、未知隔离 | 已实现并有定向回归 |
| 原厂towardj | 控制器组织运动，更新到最新端点；后端不生成逐帧PVAT插值 | 独立空载零工具Docker小范围通过 |
| 笛卡尔PVAT | 整段路径与减速尾段规划、控制器所有权和反馈 | 研究/诊断保留，实时性能及完整实体执行未通过 |
| 自研模型/Fake | 显示和确定性故障测试，独立于官方Docker | 已有；接触物理引擎和负载效果未交付 |
| 真实机械臂 | 实体工具、夹爪、路径和停止验收 | 新连接/动作须具体范围及当天现场批准 |

原厂towardj不保证TCP直线运动。终点跟随中的松手表示停止更新并完成有限目标，不保证立即停在松手位置。诊断中的`holding`也不代替实际到位验证。

## 环境准备

Python **3.11+**；Node.js **22+**及npm；Windows实验室环境使用PowerShell。以下操作只安装或构建，不连接机器人：

```powershell
python -m venv .venv
.venv/Scripts/python.exe -m pip install -e "./backend[dev,real]"
Set-Location web
npm ci
npm run build
Set-Location ..
```

`real`是原厂SDK依赖组名称，安装不等于连接。编译运动学实验另需可选`reference-speed`依赖；终点邮箱本身不需要Numba。

## 入口与验证

**新终点方案当前没有可直接供头显操作的产品启动命令。** 官方Docker诊断默认只预览；需要已核对、独立、无映射端口/挂载/设备的固定原厂镜像实例：

```powershell
.venv/Scripts/python.exe scripts/check_docker_endpoint_follow.py --container <独立容器ID> --output .tmp/endpoint-check
```

加`--run`会驱动虚拟机械臂：一次虚拟准备、最多三次towardj、到位/保持检查及最多一次异常停止。它不接受真实地址；通过不代表真人VR、真实夹爪或大范围路径通过。

无需Docker的最近终点相关回归：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/control/test_endpoint_follow.py backend/tests/scripts/test_docker_endpoint_follow.py backend/tests/control/test_cartesian_pose_input.py backend/tests/control/test_vr_pose_reference.py -q -p no:cacheprovider
```

上述组合最近结果为46项通过。其他模块按改动运行定向测试；前端在`web`目录运行`npm test`、`npm run build`。

历史网页服务`run_lmaster_docker.py`、`start_docker_web.ps1`及其8773/8443入口仍保留，但**不自动使用新towardj方案**。`--pose-follow-vr`仍在连接前拒绝；不要去掉封锁恢复旧模式。原README中的详细命令和参数见[历史档案](docs/archive/README-before-endpoint-2026-11-06.md)。

## 源码导航

| 目录/文件 | 用途 |
| --- | --- |
| `backend/app/control/endpoint_follow.py` | 最新端点所有权；无SDK、IK或执行许可 |
| `backend/app/control/cartesian_*` | 笛卡尔参考、尾段、工具绑定及输入/主机调度研究 |
| `backend/app/robots/` | 原厂/Fake/Docker适配与读取、写入及故障隔离 |
| `backend/app/digital_twin/` | 仿真运行时和官方Docker身份检查 |
| `backend/app/commissioning/` | 有界诊断、路径与现场范围支持 |
| `web/src/ui/`、`web/src/xr/` | 网页和WebXR交互 |
| `scripts/check_docker_endpoint_follow.py` | 最新终点Docker独立诊断 |
| `scripts/real_robot_*`、`scripts/real_pvat_*` | 实体预览/诊断；不是默认体验入口 |
| `docs/` | 当前总览、时间线、设计计划及证据索引 |

## 数据与交付边界

源码、通用配置、测试和脱敏说明进入Git。真实本地配置、授权材料、原始日志、精确现场工具数据、TLS私钥、`.tmp`、虚拟环境和依赖目录保留本机，不上传。原始证据在说明中以本机路径及SHA256引用。

历史stop_unverified和未知结果不能用刷新、晚到回执或一次IDLE清除；不自动重试、复位或恢复stop_sys升级。项目尚未完成真实负载抓取和运动中制动验收，也没有可保证的新版VR到真机端到端延迟值。
