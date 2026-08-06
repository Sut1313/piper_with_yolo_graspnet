# Piper 机械臂 GraspNet 视觉抓取系统（Gazebo 仿真）

**基于 ROS 2 Humble + Gazebo + MoveIt2 + GraspNet（Docker）的完整视觉抓取流水线**：仿真相机感知 → GraspNet 6D 抓取姿态推理 → 解析 IK + MoveIt 规划 → 机械臂拾取放置，三物体完整抓取实测约 100 秒/轮。

![抓取流水线演示](resources/grasp_pipeline_1753.gif)

## 特性

- **GraspNet 6D 姿态真值驱动**——不只使用识别位置，完整采用预测的抓取姿态（`calib_18` 坐标映射，见 `grasp_frame_transform.cpp`）
- **状态驱动感知闭环**——`pick_state.json` 实时同步抓取进度，Docker 推理端只识别未放置物体（减少误检、加速推理）
- **解析 IK + 4 段 AND 预检**——ssik 解析逆解（确定性、种子无关）为每个候选预检 approach / grasp / lift / place 四段可规划性，全部通过才执行
- **共享目录桥接**——相机 15Hz 与推理 0.5Hz 的速率差异通过文件交换解耦（`shared_camera_data/`，原子写）
- **门控体系**——抓取前距离门控（8cm）+ 接近角 WARN-only 检查，防止强行抓取
- **实测 3/3 全绿**——2026-08-01 连续三轮完整抓取三个物体成功（每轮约 100s）

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│  Host（ROS 2 Humble + Gazebo Classic 11）                    │
│                                                             │
│  Gazebo 仿真世界（piper_pick.world）                         │
│    ├── 机械臂（Piper 6R + 夹爪，ros2_control 驱动）           │
│    ├── 三个抓取物体（绿柱/蓝球/红柱，可竖立/横置配置）          │
│    └── 顶部 RGB-D 相机（800×600，15Hz，60° 俯角）             │
│         │  /camera/image_raw, /camera/depth/image_raw        │
│         ▼                                                    │
│  camera_bridge_node.py（ROS 2 → 共享目录桥接，原子写）         │
│         │                                                    │
│         ▼                                                    │
│  shared_camera_data/  （color.png + depth.png + meta.mat）    │
│         │ Docker volume mount                                │
└─────────┼───────────────────────────────────────────────────┘
          ▼
┌─────────────────────────────────────────────────────────────┐
│  Docker 容器（graspnet-baseline:20.04，GPU + X11）            │
│  inference_from_camera.py：点云 → GraspNet 推理 → 碰撞过滤    │
│    → 按 pick_state 只识别未放置物体 → perception_grasps.csv   │
│    → result.png（实时显示）                                   │
└─────────┬───────────────────────────────────────────────────┘
          ▼
┌─────────────────────────────────────────────────────────────┐
│  pick_and_place（C++ 主节点）                                │
│  候选遍历(≤300) → 4段AND预检(解析IK+OMPL) → 8步执行序列        │
│  → /piper/attach · detach（Gazebo 插件）→ pick_state 更新     │
└─────────────────────────────────────────────────────────────┘
```

**数据流闭环**：相机帧 → 共享目录 → GraspNet 推理（只识别 pending 物体）→ CSV 候选 → C++ 候选选择与执行 → 抓取成功更新 pick_state → 推理端下一帧跳过已放置物体。

## 快速开始

```bash
# 终端 1：仿真 + 控制 + 抓取主节点
cd ~/piper_graspnet/gazebo_piper_pick_place
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch piper_gazebo piper_pick_and_place.launch.py

# 终端 2：GraspNet 推理（Docker，需 NVIDIA GPU）
cd ~/piper_graspnet
./graspnet-docker-camera.sh gui

# 终端 3：主机端结果显示
cd ~/piper_graspnet
python3 result_viewer.py
```

启动后主节点自动执行"随机顺序抓取三个物体并放置到放置区"（约 100s），`result_viewer.py` 实时显示推理结果与抓取过程。

## 环境要求

| 组件 | 版本 | 说明 |
|---|---|---|
| 操作系统 | Ubuntu 22.04 | 开发验证环境 |
| ROS 2 | Humble | Gazebo 与 MoveIt 全栈 |
| Gazebo | Classic 11 | ros2_control + gazebo_ros 插件 |
| MoveIt2 | Humble 对应版 | OMPL 规划、IK 服务 |
| Docker | 任意近期版 | graspnet-baseline:20.04 镜像（~25GB） |
| GPU | NVIDIA + CUDA | GraspNet 推理（Docker `--gpus all`） |

> Docker 镜像依赖链与清理方法（孤儿镜像识别、悬空清理）见 [tutorial.md](tutorial.md) 第 6.5 节。
| Python | 3.10 + ssik | 解析 IK 求解器（`pip install ssik`） |
| Open3D | 0.17 | 推理端离屏渲染 |

## 仓库结构

```
piper_graspnet/
├── gazebo_piper_pick_place/          # ROS 2 workspace
│   └── src/
│       ├── piper_description/        # 机械臂 URDF 模型（SolidWorks 导出 + ros2_control）
│       ├── piper_gazebo/             # 仿真世界、抓取主节点、解析IK、桥接
│       │   ├── worlds/               # 物体布局 + 相机世界文件
│       │   ├── src/pick_and_place.cpp    # 主流程（候选/预检/执行）
│       │   └── src/grasp_frame_transform.cpp  # GraspNet→世界坐标映射
│       ├── piper_moveit/             # MoveIt2 配置（SRDF/kinematics/OMPL）
│       └── piper_perception/         # 轻量相机桥接包（纯 Python）
├── graspnet-baseline/                # GraspNet 推理源码（Docker 构建上下文）
│   └── inference_from_camera.py      # 连续推理循环
├── shared_camera_data/               # Docker ↔ 主机交换目录
├── resources/                        # 演示 GIF
├── README.md                         # 本文档
└── tutorial.md                       # 从零开发完整教程（含问题解决）
```

## 编译

```bash
cd ~/piper_graspnet/gazebo_piper_pick_place
source /opt/ros/humble/setup.bash
colcon build --packages-select piper_gazebo
source install/setup.bash
```

> ⚠️ 若本机装有 Homebrew（`/home/linuxbrew`），其 binutils 会劫持 `ld` 导致链接失败（`libcurl undefined reference to libssh`）。编译前执行：
> ```bash
> export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "linuxbrew" | paste -sd:)
> ```

## 关键配置

| 项 | 值 | 位置 |
|---|---|---|
| red_box | (0.45, -0.12, 0.124)，横躺 | world 文件 |
| green_cylinder | (0.45, 0.02, 0.124)，竖立 | world 文件 |
| blue_sphere | (0.45, 0.17, 0.124)，竖立 | world 文件 |
| 放置点 | (0.40, -0.20, 0.19) + home 朝向 | launch `back_place_x/y` |
| 抓取高度下限 | 0.12m（`min_grasp_z`） | launch |
| 距离门控 | 0.08m（`max_grasp_distance`） | launch |
| 候选上限 | 300（`graspnet_6d_max_candidates`） | launch |
| per-object home | joint1: green=0 / red=-0.35 / blue=0.35 | pick_and_place.cpp |

> ⚠️ **world 与 launch 文件各有 4 份副本**（src + 3 个 install 目录，含 `COLCON_IGNORE` 的旧产物）。修改数据文件必须全部同步，否则运行加载旧版（详见 tutorial 第 9 章 FAQ）。

## 已知问题与限制

1. **red_box 解析空洞**——"朝下姿态 × 左区(y≈-0.13~-0.15)"在关节限位内真不可达（ssik 与 KDL 双求解器确认），GraspNet 预测姿态落入空洞时该物体抓取失败。候选修复方向：姿态松弛预检（详见 tutorial 第 8 章）
2. **结果时好时坏**——感知位姿 1cm 级方差 × 4 段 AND 预检的二进制判定 × "失败即终止整个循环"（`pick_and_place.cpp:854`），表现为偶发 2/3。候选修复：失败跳过继续
3. **放置区堆叠**——三次放置落点实际偏移后挤在 3cm 范围内，第二个放置的物体可能碰到第一个。候选修复：分散放置点

## 相关文档

- **[tutorial.md](tutorial.md)** —— 从零开发完整教程：环境搭建 → Gazebo 仿真 → MoveIt 配置 → 抓取机制 → 视觉感知 → GraspNet 集成 → 决策体系 → 调试诊断，含 12+ 个实战问题与解决过程
- `ARCHITECTURE.md` —— 早期架构说明（7/26 版本，不含 C++ 抓取节点，参考用）
- `SESSION_2026-08-01.md` —— 抓取系统诊断实验完整记录（IK 求解器三实验、空洞定位）

## 参考项目

- [GraspNet](https://github.com/graspnet/graspnet-baseline) —— 通用抓取检测网络
- [ssik](https://github.com/personalrobotics/ssik) —— 解析逆解求解器（含 Piper 预构建）
- [MoveIt2](https://moveit.picknik.ai/) —— 运动规划框架
- [Gazebo Classic](https://classic.gazebosim.org/) —— 仿真环境

## 许可证

[MIT License](LICENSE)

## 路线图

- [x] 感知流水线（相机 → GraspNet → 结果显示）
- [x] 6D 抓取姿态真值集成（calib_18 映射）
- [x] 状态驱动感知闭环（pick_state）
- [x] 三物体完整抓取（3/3 实测）
- [ ] 失败跳过继续（替代 abort）
- [ ] 姿态松弛预检（填补解析空洞）
- [ ] 分散放置点
