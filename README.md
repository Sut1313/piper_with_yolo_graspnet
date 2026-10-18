# PiPER YOLOv8-Seg + GraspNet 视觉抓取

这是一个面向 ROS 2 Humble 的视觉抓取工程：YOLOv8s-seg 先识别物体并生成二维实例掩膜，RGB-D 桥接节点利用深度和相机内参生成目标点云，GraspNet 估计 6D 抓取候选。在轻量仿真中，MoveIt 2 继续完成四段预检和抓取、抬升、转移、放置。

当前推荐链路使用 Gazebo Harmonic，只加载 AG 机器人左侧 PiPER 机械臂、D435i、桌面和三个物体。RViz 默认随 MoveIt 启动。

![抓取流水线演示](resources/grasp_pipeline_1753.gif)

## 项目来源与致谢

本项目来源于上游项目 [H-wq-ha/real_piper_graspnet](https://github.com/H-wq-ha/real_piper_graspnet)。上游项目建立了 `D435i RGB-D → GraspNet → 6D 抓取候选 → Piper 候选预检与执行` 的主流程，并包含 Piper ROS、RealSense、GraspNet 推理和真机执行相关实现。

本仓库在该流程基础上重点进行了以下改造：

| 方面 | 上游项目 | 本项目的改动 |
|---|---|---|
| 目标分割 | ROI/场景点云进入 GraspNet | 新增微调后的 YOLOv8s-seg，输出类别、检测框和逐实例像素掩膜 |
| YOLO 部署 | 无此节点 | 新增纯 ONNX Runtime ROS 2 节点，运行时不依赖 Ultralytics/PyTorch |
| 二维到三维 | RGB-D 点云 | 用实例掩膜、同步深度图和 `CameraInfo.K` 只保留目标点云，再交给 GraspNet |
| 感知调度 | 相机共享目录驱动 | 新增 `pick_state.json` 状态驱动感知，每次抓取请求只处理一帧，机械臂运动时停止推理 |
| 候选筛选 | GraspNet 候选预检 | 增加去重、几何预筛、快速 IK，再对有限候选执行 approach/grasp/lift/place 四段 AND 预检 |
| 多物体任务 | 真机单次抓取流程 | 新增 `cube/cuboid/cylinder` 随机循环、失败跨轮重试和独立放置点 |
| 仿真 | 上游以真机 D435i/Piper 为主 | 新增轻量 Gazebo Harmonic 场景和专用 MoveIt 配置；保留旧 Gazebo Classic 链路 |
| 机器人配置 | 真机/完整系统 | 轻量场景只保留左臂和深度相机，机械臂参数、相机内参及安装位姿取自整车 URDF |
| 工程部署 | 上游目录结构 | 清除用户主目录硬编码，支持仓库根目录自动发现、ROS 参数和环境变量覆盖 |
| 训练工具 | 不包含 YOLO 训练 | 新增数据集校验、YOLOv8s-seg 微调、最佳权重复制和 ONNX 自动导出脚本 |

本仓库是派生工程，不是上游项目的官方分支。上游及各第三方组件的版权和许可证仍归原作者所有；`graspnet-baseline/` 内保留了其许可证文件。上游仓库没有根级统一许可证，公开再分发或商用前请自行确认各组成部分的授权范围。

## 当前状态

已验证的完整链路为：

```text
Gazebo Harmonic RGB-D
  → piper_yolo（YOLOv8s-seg ONNX）
  → color.png + depth.png + meta.mat + instances.npz
  → GraspNet Docker（掩膜点云、6D 候选、碰撞过滤）
  → perception_grasps.csv
  → lightweight_grasp_executor
  → 快速 IK + 四段 MoveIt 预检
  → ros2_control 执行并在 Gazebo 中 attach/detach
```

`cube`、`cuboid`、`cylinder` 均已完成识别、接近、夹取、抬升、转移、放置、释放和回零测试。候选碰撞过滤和规划筛选已限制处理数量，避免无意义地遍历全部候选。

真机方面已验证 `D435i 对齐 RGB-D → 16 类 YOLO 实例分割 → 目标点云 → GraspNet 相机坐标系候选`。**尚未验证真机手眼标定、候选到 `base_link` 的转换以及真实 PiPER 运动执行**；当前真机节点仅定位为感知和抓取姿态推理。

已知问题：ROS 2 Humble 的 MoveIt/class_loader 在部分机器上收到 Ctrl-C 时可能打印退出阶段的段错误；它发生在关闭流程，不影响此前的规划和抓取运行。

## 目录结构

```text
.
├── gazebo_piper_pick_place/          # ROS 2 工作区
│   └── src/
│       ├── piper_lightweight_sim/    # 推荐：左臂+D435i+桌面+三物体
│       ├── piper_yolo/               # YOLO ONNX 与 RGB-D/GraspNet 桥接
│       ├── table_plane_detector/     # 可选：D435i 桌面平面检测与仿真验收
│       ├── piper_gazebo/             # 保留的 Gazebo Classic 链路
│       ├── piper_moveit/              # 原有 MoveIt 配置
│       ├── piper_description/         # PiPER 模型
│       └── piper_perception/          # 原有感知桥接
├── graspnet-baseline/                # GraspNet Docker 与相机推理入口
├── models/
│   ├── tabletop_yolov8s_seg_v2.onnx  # 节点默认加载的 16 类桌面模型
│   └── piper_yolov8s_seg.onnx        # 仿真抓取使用的旧三类模型
├── yolo_training/                    # 微调、导出和运行环境脚本
├── shared_camera_data/               # 运行时交换目录，不提交 Git
├── graspnet-docker-camera.sh         # 前台 GUI/headless GraspNet
└── yolo_viewer.py                    # 显示节点已生成的识别叠加图
```

## 环境要求

验证环境：

- Ubuntu 22.04 x86_64；
- ROS 2 Humble；
- Gazebo Harmonic 8.x、`ros_gz` 和 `gz_ros2_control` 0.7.x；
- MoveIt 2 2.5.x；
- Python 3.10；
- Docker、NVIDIA 驱动和 NVIDIA Container Toolkit；
- 支持 CUDA 的 NVIDIA GPU（GraspNet Docker 推理需要）；
- 建议至少 16 GB 内存和 35 GB 可用磁盘空间。

轻量仿真不再依赖开发机上的单独 `gz_ros2_control_ws`。在正确配置 ROS/Gazebo apt 软件源后，安装发布版依赖即可：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep python3-venv \
  ros-humble-moveit ros-humble-ros-gzharmonic \
  ros-humble-gz-ros2-control \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-cv-bridge ros-humble-message-filters
```

若还要编译和运行保留的旧 Gazebo Classic 链路，再安装：

```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-dev ros-humble-gazebo-msgs
```

如果 apt 找不到 `ros-humble-ros-gzharmonic`，先按 [Gazebo 官方 ROS 安装说明](https://gazebosim.org/docs/harmonic/ros_installation/) 配置 Gazebo 软件源。Docker 使用 GPU 前按 [NVIDIA Container Toolkit 安装说明](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) 配置运行时，并用下面的命令确认容器能看到显卡：

```bash
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

## 在新设备上部署

### 1. 克隆并安装 ROS 依赖

```bash
git clone <你的 GitHub 仓库 URL>
cd piper_with_yolo_graspnet
export PIPER_GRASP_ROOT="$(pwd)"

source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths gazebo_piper_pick_place/src --ignore-src -r -y
```

项目不依赖固定克隆位置。通常无需设置环境变量；若使用拆分安装目录或把模型、共享目录放在其他位置，可显式设置：

```bash
export PIPER_GRASP_ROOT="$(git rev-parse --show-toplevel)"
export PIPER_YOLO_MODEL="${PIPER_GRASP_ROOT}/models/piper_yolov8s_seg.onnx"
export PIPER_REAL_YOLO_MODEL="${PIPER_GRASP_ROOT}/models/tabletop_yolov8s_seg_v2.onnx"
export PIPER_SHARED_DIR="${PIPER_GRASP_ROOT}/shared_camera_data"
```

旧变量 `PIPER_YOLO_PROJECT_ROOT`、`PIPER_YOLO_MODEL_PATH` 和 `PIPER_YOLO_SHARED_DIR` 仍兼容。

### 2. 检查随仓库分发的模型

真机和仿真两个 ONNX 运行模型各约 46 MB，已包含在 Git 仓库中，不需要在新设备上重新训练：

```bash
test -s models/tabletop_yolov8s_seg_v2.onnx
sha256sum models/tabletop_yolov8s_seg_v2.onnx
```

期望 SHA-256：

```text
db8b059f17566202870a89f88e835cad19e76b45cbda790cfd59500df5c16620
```

真机节点的默认模型识别 16 类桌面物体：`pen/pencil/marker/eraser/ruler/scissors/`
`stapler/tape/sharpener/block/cup/bottle/book/phone/mouse/remote`。原来的
`piper_yolov8s_seg.onnx` 三类模型仍由仿真 `yolo_bridge_node` 加载，避免
`cube/cuboid/cylinder` 状态机与新模型标签发生冲突。

### 3. 安装 YOLO ONNX 运行环境

脚本会在仓库内建立隔离环境 `.venv-yolo-runtime`。该目录不会提交到 Git：

```bash
./yolo_training/setup_runtime_env.sh
```

默认安装 CPU 版 ONNX Runtime。节点设置了 `prefer_cuda=true`，检测到 GPU provider 时使用 GPU，否则自动使用 CPU。若要使用 GPU 版，请按 `yolo_training/requirements-runtime-gpu.txt` 建立兼容 CUDA 的环境。

### 4. 编译 ROS 2 工作区

```bash
source /opt/ros/humble/setup.bash
cd gazebo_piper_pick_place
colcon build --packages-up-to piper_lightweight_sim --symlink-install
source install/setup.bash
```

上面的命令只构建推荐轻量链路及其依赖。已安装 Gazebo Classic 依赖且需要旧链回归时，可改为 `colcon build --symlink-install` 全量构建。

构建目录 `build/`、`install/` 和 `log/` 都由 `.gitignore` 排除，不应上传到 GitHub。

### 5. 构建 GraspNet 镜像

```bash
cd "${PIPER_GRASP_ROOT}/graspnet-baseline"
./docker-build.sh graspnet-baseline:20.04
```

仓库包含推理所需的 `logs/log_rs/checkpoint.tar`。镜像较大，首次构建耗时取决于网络和 GPU/CUDA 依赖下载速度。

## 运行完整轻量仿真

下面示例假定新终端位于克隆目录的父目录；先进入 `piper_with_yolo_graspnet`。推荐使用桌面图形会话，确保 `DISPLAY` 有值。

### 终端 A：Gazebo、RGB-D 和 YOLO

```bash
cd piper_with_yolo_graspnet
export PIPER_GRASP_ROOT="$(pwd)"
source /opt/ros/humble/setup.bash
source gazebo_piper_pick_place/install/setup.bash

ros2 launch piper_lightweight_sim lightweight_sim.launch.py \
  camera_pitch_offset:=1.047197551
```

`camera_pitch_offset:=0.0` 严格保持整车 URDF 的相机姿态，但原始相机水平朝前，看不到与机械臂基座齐高的桌面。识别抓取演示使用 `1.047197551`（下俯 60°），只改变相机俯仰，不改变安装位置和内参。

若只想连续观察 YOLO、不启动抓取状态机：

```bash
ros2 launch piper_lightweight_sim lightweight_sim.launch.py \
  camera_pitch_offset:=1.047197551 \
  state_driven_perception:=false
```

### 终端 B：GraspNet

```bash
cd piper_with_yolo_graspnet
./graspnet-docker-camera.sh gui
```

无桌面或不需要 Open3D 窗口时：

```bash
./graspnet-docker-camera.sh headless
```

### 终端 C：MoveIt、RViz 和抓取执行器

```bash
cd piper_with_yolo_graspnet
source /opt/ros/humble/setup.bash
source gazebo_piper_pick_place/install/setup.bash

ros2 launch piper_lightweight_sim lightweight_pick.launch.py \
  camera_pitch_offset:=1.047197551
```

默认运行节奏已与上游 `real_piper_graspnet` 的实际运行请求对齐：机械臂速度缩放
为 `0.10`、加速度缩放为 `1.0`，候选预检规划预算为 `0.5 s`，正式执行规划预算为 `10.0 s`，夹爪单次
动作时间为 `1.0 s`，共享 RGB-D/识别结果最高按 `5 Hz` 更新。Gazebo 的
`real_time_factor` 保持 `1.0`，因此仿真时间不会刻意快于真机。需要做对照实验时
可以显式覆盖这些参数，例如：

```bash
ros2 launch piper_lightweight_sim lightweight_pick.launch.py \
  camera_pitch_offset:=1.047197551 \
  velocity_scale:=0.10 acceleration_scale:=1.0 \
  candidate_planning_time:=0.5 execution_planning_time:=10.0 \
  gripper_motion_time:=1.0
```

`max_candidates=10` 和 `max_prefilter_candidates=60` 是仿真链路保留的两级候选
筛选上限，用于避免把上游最多 150 个候选全部交给昂贵的 OMPL 规划；它只减少
等待时间，不改变机械臂轨迹速度。

RViz 默认开启。资源受限时可加 `start_rviz:=false`。默认随机循环处理三个物体；也可指定单个目标：

```bash
# 只做候选筛选和四段规划，不移动机械臂
ros2 launch piper_lightweight_sim lightweight_pick.launch.py \
  camera_pitch_offset:=1.047197551 target:=cube dry_run:=true

# 执行单个物体的完整抓取和放置
ros2 launch piper_lightweight_sim lightweight_pick.launch.py \
  camera_pitch_offset:=1.047197551 target:=cylinder dry_run:=false
```

有效目标为 `cube`、`cuboid` 和 `cylinder`。

### 终端 D（可选）：YOLO 识别窗口

```bash
cd piper_with_yolo_graspnet
./yolo_viewer.py
```

该窗口只读取 `shared_camera_data/yolo_overlay.png`，不会加载第二份模型或重复推理。状态驱动模式下，只有执行器请求感知时画面才更新。按 `Q` 或 `Esc` 关闭。

## 数据接口

`shared_camera_data/` 是宿主机 ROS 节点与 GraspNet 容器之间的运行时交换目录：

| 文件 | 内容 |
|---|---|
| `color.png` | 与深度对齐的 BGR 图像 |
| `depth.png` | `uint16` 深度图，默认单位毫米 |
| `meta.mat` | `intrinsic_matrix` 和 `factor_depth` |
| `instances.npz` | YOLO 每个实例的像素掩膜、类别、置信度和框 |
| `detections.json` | 便于调试的识别元数据 |
| `camera_to_world.json` | 当前仿真相机光学帧外参 |
| `.ready` | 一帧数据完成原子写入后的握手标记 |
| `perception_grasps.csv` | GraspNet 输出的带旋转矩阵 6D 候选 |
| `pick_state.json` | 当前目标、阶段、已完成目标和感知版本号 |

对有效深度像素 `(u,v,Z)`，点云反投影为：

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth / factor_depth
```

YOLO 掩膜决定哪些像素属于当前实例；深度和相机内参决定这些像素在相机坐标系中的三维位置。

## 重新训练并导出 ONNX

训练集不提交到 Git。数据集必须使用 YOLO segmentation 多边形标签，类别顺序固定为：

```text
0 cube
1 cuboid
2 cylinder
```

准备配置并训练：

```bash
cp yolo_training/piper_objects.yaml.example yolo_training/piper_objects.yaml
# 编辑 piper_objects.yaml，使 path 指向数据集根目录

python3 -m venv .venv-yolo-train
source .venv-yolo-train/bin/activate
pip install -r yolo_training/requirements-training.txt

python yolo_training/train_yolov8s_seg.py \
  --data yolo_training/piper_objects.yaml \
  --base-model yolov8s-seg.pt \
  --epochs 150 --imgsz 640 --batch 16 --device 0
```

脚本在训练结束后选择 `best.pt`，复制训练检查点到 `models/piper_yolov8s_seg_best.pt`，并将运行模型稳定导出到 `models/piper_yolov8s_seg.onnx`。节点启动时直接加载 ONNX 文件。更多细节见 [yolo_training/README.md](yolo_training/README.md) 和 [YOLO_INTEGRATION.md](YOLO_INTEGRATION.md)。

## 路径和部署约定

代码中不包含开发者用户名或固定克隆目录。默认路径解析顺序为：

1. 显式 ROS launch/节点参数；
2. `PIPER_GRASP_ROOT`、`PIPER_YOLO_MODEL`、`PIPER_SHARED_DIR`；
3. 从 ROS 包安装位置向上查找仓库标记目录；
4. 当前工作目录作为二进制拆分安装的最后回退。

因此正常的仓库内 `colcon build` 可以放在任意用户、任意目录。若只复制 `install/` 到别处而不保留仓库结构，必须显式传入 `model_path` 和 `shared_dir`。

## 旧 Gazebo Classic 链路

旧 `piper_gazebo` / `piper_moveit` 场景仍保留用于回归，但它的世界、物体命名和执行器与轻量链路不同，不要在同一次运行中混用两条链。启动示例：

```bash
source /opt/ros/humble/setup.bash
source gazebo_piper_pick_place/install/setup.bash
ros2 launch piper_gazebo piper_pick_and_place.launch.py
```

旧链还需要 `gazebo_ros_pkgs` 和 `gazebo_ros2_control`。新开发和性能测试优先使用 `piper_lightweight_sim`。


## 常见问题

- `YOLO instances: none`：确认使用本项目微调 ONNX、相机能看到目标、类别顺序一致；调试时将 `confidence_threshold` 适当降低。
- `No segmented object points`：先看 `instances.npz` 是否有掩膜，再检查掩膜区域的深度是否非零以及 `factor_depth` 是否为 1000。
- YOLO 窗口不更新：状态驱动模式需要抓取执行器写入 `pick_state.json`；独立观察时设置 `state_driven_perception:=false`。
- `gz_ros2_control` 插件找不到：安装 `ros-humble-gz-ros2-control`，重新打开终端并 `source /opt/ros/humble/setup.bash`。
- RViz 没打开：`lightweight_pick.launch.py` 默认 `start_rviz:=true`；确认当前终端有图形会话和正确的 `DISPLAY`。
- Docker 看不到 GPU：先通过 `nvidia-smi` 和 NVIDIA CUDA 测试容器验证宿主机运行时，不要从 ROS 节点侧排查。
- 候选筛选太慢：降低 `max_prefilter_candidates`、`max_candidates` 或 GraspNet 的 `collision_candidate_limit`，但会牺牲复杂场景下的成功率。

更详细的轻量场景参数见 [piper_lightweight_sim/README.md](gazebo_piper_pick_place/src/piper_lightweight_sim/README.md)。

## 真机 YOLO 物体点云节点

`real_yolo_pointcloud_node` 是不依赖 Gazebo、MoveIt 或机械臂控制的真机感知
节点。它要求 RealSense 驱动提供已经对齐到彩色图的深度图，并同时提供两种
输出：ROS 话题用于显示/调试，`shared_camera_data/` 用于 Docker GraspNet
抓取姿态推理。共享目录使用与仿真相同的 `.ready` 握手，不会覆盖 GraspNet
尚未消费的上一帧。

> **真机功能边界：**该节点和 GraspNet 输出的原始候选都在
> `camera_color_optical_frame` 中。仓库运行时可能生成的
> `shared_camera_data/camera_to_world.json` 是轻量仿真外参，不是真机手眼标定结果。
> 未建立真实 `base_link → camera_color_optical_frame` TF、未校验夹爪开口和
> MoveIt 四段规划前，不要将这些候选直接发送给机械臂。

先启动 D435i：

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_sync:=true
```

再启动真机节点：

```bash
cd piper_with_yolo_graspnet
export PIPER_GRASP_ROOT="$(pwd)"
source /opt/ros/humble/setup.bash
source gazebo_piper_pick_place/install/setup.bash
ros2 launch piper_yolo real_yolo_pointcloud.launch.py
```

节点检测到具有有效深度点的物体后，会原子写入：

```text
shared_camera_data/color.png
shared_camera_data/depth.png
shared_camera_data/meta.mat
shared_camera_data/instances.npz
shared_camera_data/detections.json
shared_camera_data/frame_manifest.json
shared_camera_data/yolo_overlay.png
shared_camera_data/.ready
```

随后可在项目根目录启动仅姿态推理的 GraspNet：

```bash
./graspnet-docker-camera.sh gui
# 无界面模式：./graspnet-docker-camera.sh headless
```

真机帧的 `frame_manifest.json` 会标记来源，GraspNet 将自动忽略共享目录中残留的
仿真 `pick_state.json`。GraspNet 完成一帧后会删除 `.ready`，真机节点再提交下一帧。

默认接口：

| 方向 | 话题 | 类型 |
|---|---|---|
| 输入 | `/camera/camera/color/image_raw` | `sensor_msgs/Image` |
| 输入 | `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` |
| 输入 | `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` |
| 可选输出（默认关闭） | `/yolo/object_cloud` | `sensor_msgs/PointCloud2` |
| 输出 | `/yolo/overlay` | `sensor_msgs/Image` |
| 输出 | `/yolo/detections` | `std_msgs/String`（JSON） |

共享目录桥接不依赖 ROS 点云，因此默认关闭 `/yolo/object_cloud` 以减少点云构建、
序列化和 DDS 传输开销。需要在 RViz 调试点云时可临时开启：

```bash
ros2 launch piper_yolo real_yolo_pointcloud.launch.py \
  publish_object_cloud:=true
```

开启后点云位于 `camera_color_optical_frame`，每个点包含 `x/y/z/rgb/class_id/`
`instance_id/confidence`。如目标机器的话题名不同，可直接覆盖 launch 参数：

```bash
ros2 launch piper_yolo real_yolo_pointcloud.launch.py \
  color_topic:=<彩色图话题> \
  depth_topic:=<对齐深度图话题> \
  camera_info_topic:=<彩色相机内参话题> \
  shared_dir:=<与GraspNet容器挂载一致的目录>
```

只需要 ROS 输出、不启动 GraspNet 时可关闭共享目录桥接：

```bash
ros2 launch piper_yolo real_yolo_pointcloud.launch.py \
  graspnet_bridge_enabled:=false
```
