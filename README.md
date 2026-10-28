# VR4Arm — 网页与 VR 机械臂控制

当前主线：**官方 Docker 仿真 → 操作者网页/头显体验 → 后续真机验收**。真机当前不可用，仿真成功不代表真实抓取或制动已通过。

## 推荐启动

前提：本机官方 L Master Docker 容器运行，模拟 SDK 端口3030、管理端口3020已映射。启动器核对本机 Docker、官方镜像和端口，不重建容器，不连接真实机器人。

从仓库根目录执行（Python 3.11+、Node.js 22+ / npm）：

```powershell
python -m venv .venv
.venv/Scripts/python.exe -m pip install -e "./backend[dev,real]"
Set-Location web
npm ci
npm run build
Set-Location ..
.venv/Scripts/python.exe scripts/run_lmaster_docker.py
.venv/Scripts/python.exe scripts/run_lmaster_docker.py --run
```

第一条启动器命令只预览，带 `--run` 才启动。打开 [Docker 控制台](http://127.0.0.1:8773/)。仅保留一个控制页面，PC 与头显轮流使用。

`real` 是原厂 SDK 依赖组的历史名称；安装库本身不连接设备，Docker入口始终使用模拟端口。

1. 确认页面显示 **原厂 Docker 仿真 · LEBAI_DOCKER**、连接正常。
2. 点击“解锁仿真”，再独立点击“进入准备姿态”；解锁本身不运动。
3. 到位后持按摇杆、升降或俯仰。平移每步2 mm，俯仰每步0.5°。松开不追加目标，已发送的当前小步自然完成；规划期间松手可能取消尚未发出的目标。
4. 长按夹爪打开/闭合，松开保持目标开口。Docker夹爪为本地虚拟实现，无接触物理。
5. 停止或跟踪丢失撤销控制。可恢复且无未知结果时重新解锁、准备；故障/未知结果保留，不自动重连清除。

Docker关节速度上限 **0.3 rad/s**、TCP平移上限 **30 mm/s**，实际每步由路径计算选择速度，短距离也可能达不到上限。速度上限不是机械臂分辨率。原加速度、工作区和有限目标规则仍生效；真机历史参数未修改。

## 头显控制 Docker

后端保持运行，在另一个终端执行：

```powershell
Set-Location web
npm run docker:vr
```

头显与电脑处于同一可信局域网，打开 `https://<电脑当前局域网IP>:5173/`。此专用HTTPS入口将页面和WebSocket固定代理到 **8773**；不要用旧版 `npm run dev` 的8000后端替代。

首次自签名证书提示需使用者处理，头显须支持WebXR、连接手柄。按需允许专用网络TCP5173，不暴露公网；Python后端仍只监听回环。先关闭PC控制页，再在头显进入VR。

- A第一次按下解锁；松开并收到确认后，再按A准备。
- 右Grip控制平移与工具俯仰；松开停止续发。
- 右Trigger为虚拟夹爪模拟量控制。
- B停止；有限位置入口不开放Home。

合成XR验证只能覆盖软件路径；真实佩戴、按键、方向和舒适度需用户体验。此模式尚未开放工具自转。

开发者可关闭所有控制页，在Docker后端运行时从`web`目录执行`npm run test:docker-xr`：它先检查Docker身份，再通过产品输入模块验证平面连续/反向、升降、俯仰、模拟夹爪、跟踪丢失、重新准备和断连。可用`npm run test:docker-xr -- 8774`指定另一个本机测试端口；不接受远程主机。此命令会驱动虚拟机械臂，并非只读。

## 环境与项目导航

| 环境 | 用途与边界 |
| --- | --- |
| 官方Docker＋本项目网页/VR | 当前功能开发主线；原厂IK、运动和反馈，本地虚拟夹爪 |
| 自研模型/Fake | 独立仿真、模型显示、确定性故障回归；以后接物理引擎 |
| 真机独立入口 | 后续现场验收；新连接/动作须明确范围和现场条件 |

- [项目整理与验证记录](docs/project-status.md)
- [Docker软件整链证据与限制](docs/docker-web-vr-2026-10-27.md)
- [最近真机失败证据](docs/real-web-idle-disconnect-2026-10-28.md)
- [仓库执行规则](AGENTS.md)
- [Milestone 1历史README](docs/archive/README-milestone1.md)：旧Home、鼠标末端拖动、离线演练和硬禁用占位说明已过时。

`backend/app/control`负责控制状态机，`backend/app/robots`负责适配器，`backend/app/digital_twin`负责Docker/Fake隔离，`web/src/ui`和`web/src/xr`负责网页/VR输入。`commissioning`与`real_robot_*`脚本保留诊断能力，不是默认启动路线。

按改动运行定向测试，前端测试在`web`目录运行。现场配置、授权材料、原始日志、证书和私钥只留本机，不进入Git。
