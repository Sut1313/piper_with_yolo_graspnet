# GraspNet 感知流水线 — 系统架构

## 概述

本项目集成 **Gazebo 仿真环境** → **ROS 2 话题** → **共享内存桥接** → **Docker GraspNet 推理** → **实时显示** 的完整感知流水线。

系统结构如下：

```
┌─────────────────────────────────────────────────────────────────┐
│  Host (ROS 2 Humble + Gazebo)                                   │
│                                                                  │
│  ┌─────────────────────────────────────────┐                    │
│  │  Gazebo Server                          │                    │
│  │  ┌─────────────────────────────────┐    │                    │
│  │  │  piper_pick_graspnet.world      │    │                    │
│  │  │  ├── objects (red_box, etc.)    │    │                    │
│  │  │  └── overhead_camera (RGB-D)    │    │                    │
│  │  └──────────┬──────────────────────┘    │                    │
│  │             │ gazebo_ros_camera.so       │                    │
│  │  ┌──────────▼──────────────────────┐    │                    │
│  │  │  /camera/image_raw              │    │                    │
│  │  │  /camera/depth/image_raw        │    │                    │
│  │  │  /camera/camera_info            │    │                    │
│  │  └─────────────────────────────────┘    │                    │
│  └─────────────────────────────────────────┘                    │
│                                                                  │
│  ┌─────────────────────────────────────────┐                    │
│  │  camera_bridge_node.py                  │                    │
│  │  (ROS 2 → 共享目录桥接节点)               │                    │
│  │  ├── 订阅 RGB, Depth, CameraInfo 话题     │                    │
│  │  ├── 转换格式（深度米→毫米, BGR→RGB）     │                    │
│  │  └── 写入共享目录                         │                    │
│  └──────────────────┬──────────────────────┘                    │
│                     │                                            │
│                     ▼                                            │
│  ┌─────────────────────────────────────────┐                    │
│  │  ~/piper_graspnet/shared_camera_data/    │                    │
│  │  ├── color.png        (RGB, uint8)       │                    │
│  │  ├── depth.png        (深度, uint16 毫米) │                    │
│  │  ├── meta.mat         (内参 + factor)    │                    │
│  │  ├── camera_info.txt  (调试信息)          │                    │
│  │  └── .ready           (就绪标志)          │                    │
│  └─────────────┬───────────────────────────┘                    │
│                │ docker volume mount                             │
└────────────────┼────────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────────┐
│  Docker Container (graspnet-baseline:20.04)                      │
│  GPU: --gpus all, X11: DISPLAY passthrough, IPC: --ipc=host     │
│                                                                  │
│  ┌─────────────────────────────────────────┐                    │
│  │  inference_from_camera.py               │                    │
│  │  (连续推理循环)                           │                    │
│  │  ├── 1. 监听 .ready 标志                  │                    │
│  │  ├── 2. 读 color.png, depth.png, meta.mat │                    │
│  │  ├── 3. 生成点云 + 采样                   │                    │
│  │  ├── 4. GraspNet 前传推理                 │                    │
│  │  ├── 5. 碰撞检测过滤                     │                    │
│  │  ├── 6. Open3D 离屏渲染 → OpenCV 显示     │                    │
│  │  └── 7. 清除 .ready → 等待下一帧          │                    │
│  └─────────────────────────────────────────┘                    │
│                                                                  │
│  屏幕显示 ← X11 ← DISPLAY="${DISPLAY}"                          │
└─────────────────────────────────────────────────────────────────┘
```

---

## 启动顺序

### 前置：安装依赖

```bash
# 一次性安装 — Gazebo ROS 相机插件
sudo apt-get install -y ros-humble-gazebo-plugins
```

### 步骤 1: 启动感知流水线（Host 侧）

```bash
cd ~/piper_graspnet

# 编译（首次或修改后）
cd gazebo_piper_pick_place
source /opt/ros/humble/setup.bash
colcon build --packages-select piper_perception
source install/setup.bash

# 启动 Gazebo + 相机桥接
ros2 launch piper_perception graspnet_pipeline.launch.py
```

这将会：
1. 启动 Gazebo（不含机械臂，仅物体 + 顶部相机）
2. 启动 camera_bridge_node（将 ROS 话题写入共享目录）

### 步骤 2: 启动 GraspNet 推理（Docker 侧）

在另一个终端中：

```bash
cd ~/piper_graspnet
./graspnet-docker-camera.sh gui
```

这将会：
1. 用 GPU、X11 显示、共享 IPC 启动 Docker 容器
2. 挂载 `shared_camera_data/` 到 `/shared_camera_data`
3. 在容器内运行 `inference_from_camera.py`
4. 持续监听新数据 → 推理 → 显示结果

---

## 文件清单

| 文件 | 说明 |
|------|------|
| `graspnet-baseline/` | GraspNet 推理源码（真实目录，Docker 构建上下文） |
| `gazebo_piper_pick_place/src/piper_gazebo/worlds/piper_pick_graspnet.world` | 带顶部相机的 Gazebo 世界（无机械臂） |
| `gazebo_piper_pick_place/src/piper_perception/` | ROS 2 Python 感知包 |
| `…/piper_perception/camera_bridge_node.py` | ROS 2 → 共享目录桥接节点 |
| `…/launch/graspnet_pipeline.launch.py` | 感知流水线启动文件 |
| `graspnet-baseline/inference_from_camera.py` | 连续推理脚本（在 Docker 内运行） |
| `graspnet-baseline/Dockerfile.20.04` | 已更新包含 inference_from_camera.py |
| `graspnet-docker-camera.sh` | Docker 启动脚本（GPU + X11 + IPC 共享） |
| `shared_camera_data/` | 相机数据共享目录 |
| `verify_pipeline.py` | 验收检查脚本 |

---

## 相机配置

- **类型**: RGB-D 深度相机（`type="depth"` 传感器）
- **位置**: x=0.0, y=0.0, z=0.55（原点上方）
- **姿态**: pitch=0.86 rad（斜向俯视 x≈0.45 的物体簇）
- **分辨率**: 640×480
- **FOV**: 约 69°（水平）
- **帧率**: 15 Hz
- **话题**: `/camera/image_raw`, `/camera/depth/image_raw`, `/camera/camera_info`

### 为何机械臂不被录到画面中
- 相机位于原点上方，斜向俯视物体簇（x≈0.45）
- 机械臂基座在 x=0，工作空间在 x<0.3 区域
- FOV≈69° 且高度 0.55m，覆盖红、绿、蓝三个物体
- 物体区域 (0.36~0.55) 在画面中央，机械臂完全在视场外

---

## 数据流格式

### 共享目录结构

```
shared_camera_data/
├── color.png          # RGB 图像, uint8, PNG 格式, BGR→RGB 转换
├── depth.png          # 深度图, uint16 (毫米), PNG 格式
├── meta.mat           # MATLAB .mat 文件
│   ├── intrinsic_matrix   # 3×3 相机内参矩阵
│   └── factor_depth       # 深度缩放因子 (1000.0)
├── camera_info.txt    # 人类可读的相机参数（调试用）
├── .ready             # 就绪标志（新帧就绪时写入）
└── output/            # GraspNet 输出结果
    ├── grasps_0001.npy    # 抓取姿态
    ├── scene_0001.pcd     # 场景点云
    └── ...
```

### GRASPNET 数据消费
1. 检测 `.ready` 文件存在
2. 读取 `color.png` → `np.float32 / 255.0`
3. 读取 `depth.png` → `uint16`
4. 读取 `meta.mat` → `intrinsic_matrix`, `factor_depth`
5. 生成点云 → 采样 20000 点 → GPU 推理

---

## 验收检查

```bash
python verify_pipeline.py
```

在开发/启动过程中随时运行，验证：
- 环境依赖是否完整
- 项目文件结构是否正确
- Docker 镜像是否就绪
- ROS 2 话题是否发布
- 共享目录数据完整性

---

## 常用命令速查

```bash
# 编译
cd ~/piper_graspnet/gazebo_piper_pick_place
colcon build --packages-select piper_gazebo
source install/setup.bash

# 启动感知流水线（终端 1）
ros2 launch piper_gazebo graspnet_pipeline.launch.py

# 启动 GraspNet 推理（终端 2）
./graspnet-docker-camera.sh gui

# 查看相机话题
ros2 topic list | grep camera
ros2 topic echo /camera/camera_info
ros2 topic hz /camera/image_raw

# 验收检查
python verify_pipeline.py

# 重建 Docker 镜像（修改后）
cd graspnet-baseline && ./docker-build.sh
```
