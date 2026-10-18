# table_plane_detector

面向 ROS 2 Humble 的独立桌面平面识别包。节点订阅有组织或无组织的
`sensor_msgs/msg/PointCloud2`，使用带法向约束的 RANSAC、SVD 精拟合和多帧
稳定性检查，发布相机坐标系中的桌面平面及其质量指标。

生产目标是 Intel RealSense D435i，同时提供不依赖真机的 Gazebo Harmonic
确定性测试场景。

## 当前状态

| 项目 | 状态 |
|---|---|
| ROS 2 Humble 构建 | 通过 |
| 单元测试 | 通过 |
| Gazebo 端到端验收 | 通过 |
| D435i 桌面平面识别 | 通过 |
| 真机物理距离验收 | 待相机安装 TF 标定 |

真机测试已经确认节点能够稳定跟踪桌面平面；在相机安装坐标尚未标定时，距离
字段不应被解释为机器人坐标系中的桌面高度。

## 功能

- 深度、XYZ 范围裁剪和体素降采样；
- 带期望法向约束的 PCL RANSAC 平面分割；
- 使用全部内点进行 SVD 平面精拟合；
- 内点数量、内点比例、法向、距离、RMS 和平面尺寸质量门限；
- 多帧中值距离、平均法向、启动确认、跳变拒绝和跟踪丢失处理；
- TF 实时转换期望桌面法向，避免把墙面识别为桌面；
- 发布完整平面消息、便捷距离、内点点云、RViz Marker 和诊断信息；
- 检测无效时距离话题发布 `NaN`，避免下游继续使用旧值；
- D435i、通用点云和 Gazebo 仿真配置相互独立。

## 处理流程

```text
PointCloud2
  -> NaN / 深度 / XY 裁剪
  -> 体素降采样
  -> 法向约束 RANSAC
  -> 内点 SVD 精拟合
  -> 数量 / 比例 / 法向 / 距离 / RMS / 尺寸检查
  -> 多帧稳定性检查
  -> TablePlane + distance + diagnostics + visualization
```

## 坐标系与距离定义

输出平面位于输入点云的 `header.frame_id` 中：

```text
a*x + b*y + c*z + d = 0
```

系数始终归一化，并统一符号使 `d >= 0`。

- `perpendicular_distance_m = d`：相机原点到平面的最短距离；
- `optical_axis_distance_m = -d / c`：沿标准光学坐标系 `+Z` 轴与平面的交点距离；
- 当光轴与平面近似平行或交点位于相机后方时，光轴距离为 `NaN`。

这两个量都不是自动转换后的 `base_link` 高度。如果其他节点需要机器人坐标系
中的桌面高度，应使用消息时间戳对应的 TF 将平面变换到目标坐标系后再计算。

默认配置认为桌面法向是 `base_link` 的 `+Z`。节点在每帧点云时间戳处把该法向
转换到点云坐标系，再交给 RANSAC。真机运行因此必须具备正确的
`base_link -> camera_*_optical_frame` TF 链。

## 环境

已验证环境：

- Ubuntu 22.04；
- ROS 2 Humble；
- GCC 11、C++17；
- PCL 1.12、Eigen 3.4；
- Gazebo Harmonic 8.15（仅仿真需要）；
- RealSense D435i；
- RealSense ROS 4.58.4、librealsense 2.58.4（真机测试环境）。

RealSense 驱动不是本包的一部分。请从官方仓库安装一套一致的
[`librealsense`](https://github.com/realsenseai/librealsense) 与
[`realsense-ros`](https://github.com/realsenseai/realsense-ros)，不要混用源码版和
另一套二进制版 librealsense。

## 目录结构

```text
table_plane_detector/
├── config/                         # 通用、D435i 和仿真参数
├── include/table_plane_detector/   # 平面估计与时序滤波接口
├── launch/                         # 通用、D435i 和仿真启动文件
├── msg/TablePlane.msg              # 完整平面检测结果
├── rviz/                           # 仿真可视化配置
├── scripts/evaluate_table_plane.py # 仿真自动验收
├── src/                            # C++ 节点与算法实现
├── test/                           # 数学、合成点云和时序测试
├── worlds/                         # Gazebo 测试场景
├── CHANGELOG.rst
├── LICENSE
└── package.xml
```

## 构建

将本包放入 ROS 2 工作区的 `src/` 下：

```bash
export TABLE_PLANE_WS=/path/to/your/ros2_ws
source /opt/ros/humble/setup.bash
cd "$TABLE_PLANE_WS"

rosdep install --from-paths src --ignore-src -r -y

colcon build \
  --packages-select table_plane_detector \
  --symlink-install \
  --cmake-args -DBUILD_TESTING=ON

source "$TABLE_PLANE_WS/install/setup.bash"
```

在当前项目中的构建示例：

```bash
cd gazebo_piper_pick_place
source /opt/ros/humble/setup.bash
colcon build --packages-select table_plane_detector --symlink-install
source install/setup.bash
```

## 通用点云运行

```bash
ros2 launch table_plane_detector detector.launch.py \
  input_cloud:=/camera/depth/points
```

替换参数文件：

```bash
ros2 launch table_plane_detector detector.launch.py \
  input_cloud:=/your/point_cloud \
  params_file:=/absolute/path/to/parameters.yaml
```

输入点云必须使用标准 ROS 光学坐标约定：`+X` 向右、`+Y` 向下、`+Z` 向前。

## D435i 真机运行

### 1. 检查设备

建议接入 USB 3.x 接口：

```bash
rs-enumerate-devices
```

### 2. 启动官方驱动和点云

```bash
source /opt/ros/humble/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=848x480x30 \
  pointcloud.enable:=true \
  align_depth.enable:=true \
  enable_sync:=true
```

确认实际话题、坐标系和频率：

```bash
ros2 topic list | grep points
ros2 topic info /camera/camera/depth/color/points -v
ros2 topic echo /camera/camera/depth/color/points --once --field header
ros2 topic hz /camera/camera/depth/color/points
```

### 3. 检查安装 TF

RealSense 驱动会发布相机内部 TF，但机器人 URDF 或静态 TF 发布器还必须提供
`base_link -> camera_link` 的真实安装位姿。不要同时由两个节点发布同一个 TF。

```bash
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```

没有这条 TF 链时，正式配置会输出 `TF_UNAVAILABLE`，这是预期的安全行为。

### 4. 启动检测器

```bash
export TABLE_PLANE_WS=/path/to/your/ros2_ws
source /opt/ros/humble/setup.bash
source "$TABLE_PLANE_WS/install/setup.bash"

ros2 launch table_plane_detector d435i.launch.py \
  input_cloud:=/camera/camera/depth/color/points
```

检查结果：

```bash
ros2 topic echo /table_plane_detector/plane
ros2 topic echo /table_plane_detector/distance
ros2 topic echo /diagnostics
```

稳定工作时，完整消息应满足：

```yaml
valid: true
status: TRACKING
```

启动后的前几帧显示 `INITIALIZING` 属于正常现象。

> **诊断模式提示**：相机安装 TF 未标定时，可以临时把法向参考系设为点云坐标系，
> 并根据现场点云设置期望法向来验证平面拟合。不要在生产环境中简单地把法向容差
> 放宽到 90°；那样只能找到“最大平面”，不能可靠地区分桌面与墙面。

## Gazebo 确定性测试

```bash
export TABLE_PLANE_WS=/path/to/your/ros2_ws
source /opt/ros/humble/setup.bash
source "$TABLE_PLANE_WS/install/setup.bash"

ros2 launch table_plane_detector sim_test.launch.py
```

默认使用无界面服务器并自动执行数值验收。测试相机位于桌面上方 1 m，并以
60° 俯视桌面，理论结果为：

```text
perpendicular_distance_m = 1.000000
optical_axis_distance_m  = 1.154701
```

开启 Gazebo GUI 和 RViz：

```bash
ros2 launch table_plane_detector sim_test.launch.py \
  gui:=true \
  start_rviz:=true
```

场景包含桌面物体和竖直干扰面。仿真通过深度图和 `CameraInfo` 生成标准光学坐标系
点云，生产检测器内部不包含 Gazebo 专用的坐标轴修正。

## ROS 接口

### 输入

| 话题 | 类型 | 说明 |
|---|---|---|
| `~/input_cloud` | `sensor_msgs/msg/PointCloud2` | RGB-D 点云，Sensor Data QoS |

### 输出

| 话题 | 类型 | 说明 |
|---|---|---|
| `~/plane` | `table_plane_detector/msg/TablePlane` | 完整结果；主要下游接口 |
| `~/distance` | `std_msgs/msg/Float64` | 相机原点到平面的垂距；无效时为 `NaN` |
| `~/optical_axis_distance` | `std_msgs/msg/Float64` | 光轴交点距离；无效或无交点时为 `NaN` |
| `~/inliers` | `sensor_msgs/msg/PointCloud2` | RANSAC 平面内点；由参数控制 |
| `~/marker` | `visualization_msgs/msg/Marker` | RViz 平面标记 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 质量指标与状态 |

`~/plane`、两个距离话题和 Marker 使用 Reliable + Transient Local QoS；内点点云使用
Sensor Data QoS。安全相关下游应订阅 `~/plane`，检查 `valid`、时间戳、坐标系、内点
比例和 RMS，而不是只读取一个浮点距离。

### `TablePlane.msg`

| 字段 | 含义 |
|---|---|
| `header` | 输入点云时间戳和坐标系 |
| `coefficients[4]` | 归一化平面系数 `[a,b,c,d]` |
| `perpendicular_distance_m` | 相机原点到平面的最短距离 |
| `optical_axis_distance_m` | 光学 `+Z` 轴交点距离 |
| `input_point_count` | 输入点数 |
| `filtered_point_count` | 裁剪、体素处理后的点数 |
| `inlier_count` | 平面内点数 |
| `inlier_ratio` | 内点数 / 过滤后点数 |
| `rms_error_m` | 内点到精拟合平面的 RMS |
| `normal_error_deg` | 平面法向与期望法向的夹角 |
| `valid` | 当前结果是否可用 |
| `status` | 当前状态或拒绝原因 |

## 配置

预置文件：

- `config/default.yaml`：通用生产起点；
- `config/d435i.yaml`：D435i 的裁剪范围和质量门限；
- `config/simulation.yaml`：确定性仿真的严格参数。

主要参数：

| 参数 | D435i 默认值 | 说明 |
|---|---:|---|
| `processing_rate_hz` | `5.0` | 最大处理频率 |
| `depth_min_m`, `depth_max_m` | `0.30`, `1.80` | 光学 Z 深度范围 |
| `crop_min/max_x_m` | `-0.75`, `0.75` | X 裁剪范围 |
| `crop_min/max_y_m` | `-0.75`, `0.75` | Y 裁剪范围 |
| `voxel_size_m` | `0.005` | 体素边长 |
| `ransac_distance_threshold_m` | `0.008` | RANSAC 内点距离阈值 |
| `max_iterations` | `500` | 最大 RANSAC 迭代次数 |
| `minimum_inlier_count` | `5000` | 最小内点数 |
| `minimum_inlier_ratio` | `0.25` | 最小内点比例 |
| `maximum_normal_error_deg` | `10.0` | 最大法向误差 |
| `minimum/maximum_plane_distance_m` | `0.40`, `1.50` | 垂距范围 |
| `maximum_rms_error_m` | `0.008` | 最大 RMS |
| `minimum_plane_extent_m` | `0.30` | 平面两个主方向的最小尺寸 |
| `normal_reference_frame` | `base_link` | 期望法向所在坐标系 |
| `expected_normal_x/y/z` | `0,0,1` | 参考系中的期望桌面法向 |
| `tf_timeout_s` | `0.10` | TF 查询超时 |
| `temporal_window_size` | `20` | 时序窗口 |
| `minimum_tracking_frames` | `5` | 进入跟踪所需连续有效帧 |
| `maximum_failed_frames` | `5` | 清除历史前允许的失败帧数 |
| `maximum_distance_jump_m` | `0.03` | 单帧最大距离跳变 |
| `maximum_normal_jump_deg` | `5.0` | 单帧最大法向跳变 |

参数必须根据相机安装高度、视场、桌面材质、工作距离和现场噪声，用真实数据重新
验收。内点数门限与体素大小需要配合调整。

## 状态码

| 状态 | 含义 |
|---|---|
| `INITIALIZING` | 正在积累稳定帧 |
| `TRACKING` | 稳定跟踪，可供下游使用 |
| `TF_UNAVAILABLE` | 无法取得期望法向所需 TF |
| `INSUFFICIENT_POINTS` | 裁剪后点数不足 |
| `NO_PLANE` | RANSAC 未找到候选平面 |
| `INSUFFICIENT_INLIERS` | 内点数量不足 |
| `LOW_INLIER_RATIO` | 内点比例不足 |
| `NORMAL_REJECTED` | 候选平面法向不符合桌面方向 |
| `DISTANCE_REJECTED` | 垂距超出允许范围 |
| `HIGH_RMS_ERROR` | 候选平面拟合残差过大 |
| `PLANE_TOO_SMALL` | 平面覆盖范围过小 |
| `TEMPORAL_JUMP` | 与上一帧差异过大 |
| `LOST:*` | 连续失败，历史跟踪已清除 |

## 测试

```bash
export TABLE_PLANE_WS=/path/to/your/ros2_ws
source /opt/ros/humble/setup.bash
cd "$TABLE_PLANE_WS"

colcon test \
  --packages-select table_plane_detector \
  --event-handlers console_direct+

colcon test-result --verbose
```

测试覆盖平面归一化与距离计算、倾斜桌面加离群点、过小平面拒绝、稳定启动、
时序跳变和跟踪丢失。

### 已记录结果

Gazebo 自动验收：

```text
垂距检测值        0.999485 m
垂距绝对误差      0.000515 m
光轴距离检测值    1.153899 m
光轴距离绝对误差  0.000801 m
法向误差          0.018 deg
内点比例          79.3 %
RMS               0.090 mm
```

D435i 桌面平面测试（2026-10-18，距离未验收）：

```text
连续有效帧        20 / 20
状态              TRACKING
内点数量中位数    9300
内点比例中位数    34.85 %
RMS 中位数        2.395 mm
最大法向波动      0.4225 deg
```

## 已知限制

- 当前算法假设视场中只有一个占主导地位且方向符合约束的桌面；
- 透明、高反光、纯黑或红外纹理不足的桌面可能造成深度空洞；
- 未标定安装 TF 时只能验证“存在稳定大平面”，不能可靠区分桌面和墙面；
- 节点不负责相机驱动、手眼标定或机器人 URDF 中的相机安装 TF；
- `depth_image_to_cloud_node` 是仿真辅助工具，只生成 XYZ 点云，不生成彩色纹理点云；
- 真机最终门限应使用目标硬件、安装姿态和工作环境重新测试。

## License

本包采用 [Apache License 2.0](LICENSE)。
