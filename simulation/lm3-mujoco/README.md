# LM3 MuJoCo 仿真资源

[GitHub Pages 实验回放](https://kilohexin.github.io/VR4AM/)展示 2026-11-25 的 8 秒绕箱实验。三条轨迹使用统一的基座坐标和比较 TCP。真机在空场运行，箱体为仿真障碍。真机位置来自控制器反馈。

## 模型和实验

| 路径 | 内容 |
| --- | --- |
| `model/` | 带固定夹爪的原始和重力补偿 MJCF、展开 URDF、SRDF、23 个网格，以及乐白模型的来源和许可证 |
| `experiments/` | 保存的 MoveIt 规划、碰撞场景、六轴位置/速度/加速度参考和运行摘要 |
| `experiments/site-detour-eight-full/` | 公开实验使用的 201 个轨迹点，参考时长 8 秒、间隔 40 ms |
| `tools/` | 模型重建、纯 MuJoCo 复现、公开实验数据导出 |
| `site/` | 静态展示源码、三维渲染器、仿真反馈和处理后的实验曲线 |
| `SHA256SUMS` | 模型、参考和展示文件校验值 |

重力补偿 MJCF 是本轮展示实际使用的版本。积分步长为 1 ms，位置和速度控制增益为 300 和 12。六段连杆质量和惯量来自乐白 ROS 模型，夹爪惯量按碰撞网格估算。每个实验可从 `summary.json` 查到工具偏移、场景与参考哈希。预览中的 GLB 是 VR4AM 既有的浏览器视觉资源，物理仿真使用 MJCF 和 STL/OBJ 网格。

## 离线复现

在本目录用 Python 3.12 安装公开依赖并复现 8 秒轨迹。回放只使用 MuJoCo 和版本化参考数据。

```sh
python -m pip install -r requirements.txt
python tools/replay_mujoco.py --verify
python tools/replay_mujoco.py --output lm3-replay.json
```

`--verify` 对照公开页面保存的 546 个仿真帧。用 `--experiment experiments/<目录>` 可复跑带摘要的其他参考。自定义控制或训练可直接加载 `model/lm3-with-gripper-compensated.xml`。关节名称依次为 `joint_1` 至 `joint_6`，长度单位为米，角度单位为弧度。

### 从乐白源模型重建

```sh
git clone https://github.com/lebai-robotics/lebai-ros-sdk.git
git -C lebai-ros-sdk checkout f1f13b6849f34f61dce0483e09f531816d292b63
python -m pip install -r requirements-rebuild.txt
python tools/rebuild_model.py lebai-ros-sdk rebuilt-model
```

源 URDF 使用 ROS `package://` 网格 URI。重建工具从固定的乐白版本读取这些路径，并生成自包含的 MuJoCo 网格目录。版本责任和派生改动见 [模型声明](model/NOTICE)。

## 维护展示网页

本地预览可以运行 `python -m http.server 8765 --bind 127.0.0.1 --directory site`。部署源为 `gh-pages` 根目录。静态部署包含入口、样式、渲染器、仿真帧及公开结果 JSON。

如需生成新的展示数据，把同一轨迹的已核验物理回放和仿真数据传给 `tools/export_public_results.py <physical-replay.json> <demo-data.js> <experiment-results.json>`。导出保留总样本数、主要统计值、降采样 TCP 轨迹和误差曲线。原始控制器日志、六轴真机采样和现场配置保留在实验室本机。该实验的 8 秒 RMSE 为 **7.61 mm**，起动对齐峰值为 **20.65 mm**，停稳后差值为 **0.295 mm**。

三维渲染器的源文件位于 `site/`。从仓库根目录运行 `npm ci --prefix web`，然后运行 `npm install --prefix web --no-save --package-lock=false esbuild@0.28.1` 和 `node simulation/lm3-mujoco/site/build-model.cjs` 可更新静态 bundle。
