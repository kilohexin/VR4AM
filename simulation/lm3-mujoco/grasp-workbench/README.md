# LM3 本地抓放工作台

本工作台使用仓库归档的六轴 LM3 模型。MuJoCo 在本机持续积分，MoveIt 2 Jazzy 在无网络的 Docker 容器里规划。网页只连接本地服务。

## 启动

准备 Python 3.12、Node.js 和 Docker。在 VR4AM 仓库根目录安装网页依赖，再进入仿真目录安装 Python 依赖。首次使用时构建 MoveIt 2 Jazzy 镜像，已有该镜像时可直接启动工作台。

```sh
npm ci --prefix web
cd simulation/lm3-mujoco
python -m pip install -r grasp-workbench/requirements.txt
docker build -t vr4arm-moveit-jazzy:local -f grasp-workbench/Dockerfile.planner grasp-workbench
```

在 Windows PowerShell 执行 `.\grasp-workbench\run-workbench.ps1`。用 `-Python` 可指定已安装依赖的解释器，用 `-Port` 可指定本地端口。在 bash 下执行下面的命令。

```sh
PYTHONPATH=tools python -m uvicorn server:app --app-dir grasp-workbench --host 127.0.0.1 --port 8770
```

打开 [本地抓放工作台](http://127.0.0.1:8770/)。工作台直接读取本仓库的 [模型目录](../model/)。可用 `LM3_MODEL_DIR` 指向另一套包含 MJCF、URDF、SRDF 和 `assets/` 的模型。规划器也识别已经安装的 `moveit/moveit2:jazzy-release` 镜像。

## 操作

1. 在桌面上拖动杯子、托盘或橙色障碍物。也可点右侧的物体按钮，再用方向键微调。点击杯子和托盘会同时填好抓放任务。
2. 点击「开始抓放」。绿色线显示当前这一段的规划路径。
3. 搬运时，把橙色挡块拖到杯子前面的路径下方。工作台检查剩余路径，检测到碰撞会刹停。松手后从当前关节状态重新规划。有改道空间时自动继续，否则保持停止并显示原因。

杯子是 109 g 的空杯，托盘有实际底板和四壁。MuJoCo 使用归档 LMG‑90 的七个连杆碰撞网格和 URDF 开合关节，MoveIt 使用同源碰撞网格生成的凸包以及实际夹爪角度。网页逐连杆渲染归档外观网格，位姿、指间开口和两侧接触法向力取自 MuJoCo。

杯子始终是自由刚体。两指稳定接触后开始抬升，杯子靠接触和摩擦随夹爪运动。MoveIt 把当前杯子位姿用于随臂碰撞检查。MuJoCo 的关节等式仅用于同步夹爪内部连杆。检测到持续失去接触或过量滑移时，中止任务。松爪后，杯子在重力下落到托盘，直立且在托盘内停稳后任务才完成。

指尖滑动摩擦系数设为 0.65，夹爪电机仿真扭矩上限为 0.95 N·m。模型按乐白公布的夹爪总重 0.46 kg 等比调整各连杆的质量和惯量，质量分布和接触参数仍需真机标定。[乐白 LMG‑90 规格](https://lebai.ltd/products/lmg-90/)

[真机标定方案](CALIBRATION.md)记录了后续如何用独立传感器测量夹持力与摩擦。

## 验证

在 `simulation/lm3-mujoco` 目录将 `PYTHONPATH` 设为 `tools` 后运行以下命令。PowerShell 使用 `$env:PYTHONPATH = (Resolve-Path tools).Path`，bash 使用 `export PYTHONPATH="$PWD/tools"`。

```sh
python grasp-workbench/check_workbench.py
python grasp-workbench/check_workbench.py --dynamic
python grasp-workbench/check_workbench.py --frictionless
```

首版仿真验收中，默认抓放和带障碍抓放均让 109 g 杯子直立停在托盘内。带障碍测试重规划 1 次。两指法向力中位数各约 11 N，持杯期间最大杯爪相对位移分别为 3.23 mm 和 4.89 mm。关闭指尖摩擦的对照在开始抬升后失去夹持。RRTConnect 路径可能随运行变化。归档复验观察到障碍插入后求解失败，动态避障的重复成功率仍待评估。

## 抓取候选

当前按杯子位姿生成四个绕竖直轴的候选姿态，MoveIt 用 IK、碰撞检查和笛卡尔接近段筛选。后续可让 MoveIt Task Constructor 的 `GenerateGraspPose` 和 `ComputeIK` 生成同样的候选数据。 `moveit_msgs/msg/Grasp` 是候选描述消息，可记录位姿、夹爪姿态和接近方向。
