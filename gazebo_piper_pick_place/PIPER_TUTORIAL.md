# Piper 六轴机械臂 Gazebo 仿真开发教程 —— 从零搭建 Pick-and-Place

> 本文面向 ROS 2 新手，完整讲解一个六轴机械臂从模型到自动抓取的开发全流程。

仓库地址：https://github.com/H-wq-ha/gazebo_piper_pick_place.git

---

## 目录

1. [开发环境](#1-开发环境)
2. [URDF：RViz 版 vs Gazebo 版](#2-urdfrviz-版-vs-gazebo-版)
3. [配置 Gazebo URDF](#3-配置-gazebo-urdf)
4. [使用 Setup Assistant 生成 MoveIt 配置](#4-使用-setup-assistant-生成-moveit-配置)
5. [配置完后的手动修复（必看）](#5-配置完后的手动修复必看)
6. [MoveIt 如何控制 Gazebo？](#6-moveit-如何控制-gazebo)
7. [为什么要自定义服务 + Gazebo 插件？](#7-为什么要自定义服务--gazebo-插件)
8. [Pick-and-Place 设计思路与工作流程](#8-pick-and-place-设计思路与工作流程)
9. [附录：完整文件结构与启动命令](#9-附录完整文件结构与启动命令)

---

## 1. 开发环境

| 项目 | 版本 |
|------|------|
| 操作系统 | Ubuntu 22.04 LTS (Jammy) |
| ROS 2 | Humble Hawksbill |
| Gazebo | Gazebo classic 11 (与 ROS 2 Humble 捆绑) |
| MoveIt 2 | Humble 分支 (对应 MoveIt 2) |
| 编译器 | GCC 11.4, C++17 |
| 机器人 | Piper 六轴机械臂（六旋转关节 + 二指夹爪）|

### 安装检查

```bash
# 确认 ROS 2 环境
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO  # 应输出 humble

# 确认 Gazebo
gazebo --version  # Gazebo multi-robot simulator, version 11.x

# 确认 MoveIt
ros2 pkg list | grep moveit
```

### 必须安装的依赖包

```bash
# MoveIt2 全家桶
sudo apt install ros-humble-moveit \
                 ros-humble-moveit-ros-planning-interface \
                 ros-humble-moveit-configs-utils \
                 ros-humble-moveit-simple-controller-manager \
                 ros-humble-moveit-kinematics \
                 ros-humble-moveit-planners \
                 ros-humble-moveit-ros-move-group \
                 ros-humble-moveit-ros-visualization \
                 ros-humble-moveit-setup-assistant

# Gazebo + ros2_control
sudo apt install ros-humble-gazebo-ros \
                 ros-humble-gazebo-ros2-control \
                 ros-humble-gazebo-dev \
                 ros-humble-gazebo-msgs

# 控制器
sudo apt install ros-humble-controller-manager \
                 ros-humble-joint-state-broadcaster \
                 ros-humble-joint-state-publisher \
                 ros-humble-joint-trajectory-controller

# 工具
sudo apt install ros-humble-xacro \
                 ros-humble-robot-state-publisher \
                 ros-humble-tf2-ros \
                 ros-humble-rviz2 \
                 ros-humble-rosidl-default-generators
```

---

## 2. URDF：RViz 版 vs Gazebo 版

这是新手最容易困惑的地方。同样是描述机器人，**为什么会有多个 URDF 文件？** 它们有什么区别？

### 2.1 核心区别一览

| 对比维度 | RViz 版 URDF | Gazebo 版 URDF |
|---------|-------------|---------------|
| **硬件接口** | 不需要 ros2_control | 必须包含 ros2_control 段 |
| **物理属性** | 只需要视觉 + 碰撞 | 还需要惯性参数（质量、惯量张量） |
| **控制器** | 无 | 需要指定硬件插件类型 |
| **传输层** | 可用 transmission 标签 | 推荐用 ros2_control 替代 transmission |
| **Mesh 路径** | `package://` 协议 | 两种均可，但 Gazebo 要求路径可解析 |

### 2.2 可视化背后的道理

```
RViz 显示机器人：
  URDF → robot_state_publisher → TF 树 → RViz 渲染
  （只需要视觉网格 + 关节父子关系，不需要物理引擎）

Gazebo 仿真机器人：
  URDF → robot_state_publisher → Gazebo 生成刚体 → 物理引擎驱动
  （必须要有惯性参数 + ros2_control 硬件接口 + 控制器配置）
```

### 2.3 一个典型的对比

**纯 URDF（RViz 用）** 只包含：

```xml
<link name="link1">
  <visual>
    <geometry><mesh filename="link1.STL"/></geometry>
  </visual>
  <collision>
    <geometry><mesh filename="link1.STL"/></geometry>
  </collision>
</link>
<joint name="joint1" type="revolute">
  <parent link="base_link"/>
  <child link="link1"/>
  <axis xyz="0 0 1"/>
  <limit lower="-2.618" upper="2.618" effort="100" velocity="5"/>
</joint>
```

**Gazebo 版** 在以上基础上额外增加：

```xml
<!-- 1. 惯性参数（必须！否则 Gazebo 报错） -->
<link name="link1">
  <inertial>
    <origin xyz="0.00032 -0.00041 -0.00348" rpy="0 0 0"/>
    <mass value="0.71"/>
    <inertia ixx="0.00050027" ixy="..." ixz="..." iyy="..." iyz="..." izz="..."/>
  </inertial>
  <!-- visual + collision 同上 -->
</link>

<!-- 2. ros2_control 硬件接口段 -->
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- 其余 7 个关节同理 -->
</ros2_control>

<!-- 3. Gazebo 插件激活 -->
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find piper_gazebo)/config/ros2_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

---

## 3. 配置 Gazebo URDF

### 3.1 从 SolidWorks 导出开始

通常机械臂的 URDF 由 CAD 软件（SolidWorks 的 sw_urdf_exporter 插件）导出。

导出的原始 URDF 包含：
- ✅ 全部 Link 和 Joint 的定义
- ✅ 视觉网格（STL）
- ✅ 碰撞网格
- ✅ 惯性参数（从 CAD 材质密度自动计算）

缺少的内容：
- ❌ `ros2_control` 硬件接口段
- ❌ Gazebo 插件配置

### 3.2 手动添加 ros2_control 段

在 URDF 末尾、`</robot>` 之前，加入：

```xml
<!-- 定义硬件接口：GazeboSystem 是 Gazebo 提供的标准硬件插件 -->
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>

  <!-- 对每个关节，声明命令接口和状态接口 -->
  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position">
      <param name="initial_value">0.0</param>
    </state_interface>
    <state_interface name="velocity"/>
  </joint>

  <!-- joint2 ~ joint8 同理 ... -->
</ros2_control>
```

**关于 command_interfaces 的注意事项（重要！）：**

`joint_trajectory_controller` 对 `command_interfaces` 的组合有严格限制：

| 组合 | 是否合法 |
|------|---------|
| `[position]` | ✅ |
| `[position, velocity]` | ✅ |
| `[position, velocity, acceleration]` | ✅ |
| `[velocity]` | ✅ |
| `[effort]` | ✅ |
| `[position, effort]` | ✅ |
| **`[position, velocity, effort]`** | **❌ 非法！** |

> 新手常见错误：在 `ros2_controllers.yaml` 中写了 `command_interfaces: [position, velocity, effort]`，加载控制器时会报错：
> `Invalid value set during initialization for parameter 'command_interfaces': 'effort' command interface has to be used alone`

对于位置控制的机械臂，最安全的写法是 `command_interfaces: [position]`。

### 3.3 添加 Gazebo 插件入口

```xml
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <!-- 指定控制器参数文件的路径 -->
    <parameters>$(find piper_gazebo)/config/ros2_controllers.yaml</parameters>
    <!-- 可选：指定 robot_state_publisher 节点名 -->
    <robot_param_node>robot_state_publisher</robot_param_node>
  </plugin>
</gazebo>
```

### 3.4 配置控制器参数文件

新建 `piper_gazebo/config/ros2_controllers.yaml`：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz，Gazebo 的更新频率

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    gripper_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

gripper_controller:
  ros__parameters:
    joints:
      - joint7
      - joint8
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

---

## 4. 使用 Setup Assistant 生成 MoveIt 配置

MoveIt Setup Assistant 是一个图形化工具，帮我们从 URDF 生成 MoveIt2 所需的全部配置文件包。

### 4.1 启动 Setup Assistant

```bash
ros2 launch piper_moveit setup_assistant.launch.py
```

或者直接从命令启动：

```bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```

### 4.2 操作步骤详解

#### 第一步：加载 URDF

点击 **"Create New MoveIt Configuration Package"** → **"Browse"** 选择你的 URDF 文件。

> ⚠️ **选择哪个 URDF？**
>
> 选择 **Gazebo 版 URDF**（带 ros2_control 段和惯性参数的那个）。这样生成的配置包可以直接运行在 Gazebo 上。
> 如果用纯 URDF，Setup Assistant 检测不到关节的控制接口，后续无法配置控制器。

#### 第二步：Self-Collision Matrix（自碰撞矩阵）

Setup Assistant 自动计算哪些 Link 对永远不会碰撞（如相邻 Link）。

**操作：** 直接点击 **"Generate Collision Matrix"** → 接受默认结果。

生成的碰撞矩阵保存在 SRDF 的 `<disable_collisions>` 标签中。

#### 第三步：Virtual Joints（虚拟关节）

定义机器人在世界坐标系中的位置。

**操作：**
- 点 **"Add Virtual Joint"**
- Joint Name: `fixed_base_joint`
- Child Link: `base_link`
- Parent Frame: `world`
- Joint Type: `fixed`

> 这个虚拟关节告诉 MoveIt：机器人的 base_link 相对于 world 是固定不变的。

#### 第四步：Planning Groups（规划组）

定义你希望 MoveIt 进行运动规划的分组。对于六轴机械臂 + 夹爪，我们定义两个组：

**添加 "arm" 组：**
- 点 **"Add Group"**
- Group Name: `arm`
- Kinematic Solver: `KDLKinematicsPlugin`
  - 这是 MoveIt 默认的运动学求解器。对于标准的六轴链来说够用了。
- Group Type: **Chain**
- Base Link: `base_link`
- Tip Link: `link6`

**添加 "gripper" 组：**
- 点 **"Add Group"**
- Group Name: `gripper`
- Kinematic Solver: 选择 **None**
  - ⚠️ **为什么？** 因为夹爪是两个独立的 prismatic 关节，不是一个链。KDL 只支持链式规划组，给夹爪配 KDL 会导致启动报错。
- Group Type: **Joint**
- 选中 `joint7` 和 `joint8` 添加到组中

#### 第五步：End Effector（末端执行器）

告诉 MoveIt 哪个 Link 是"手"。

**操作：**
- 点 **"Add End Effector"**
- Name: `grasp_frame`
- End Effector Group: `gripper`
- Parent Link: `link6`
- Parent Group: `arm`

#### 第六步：Passive Joints（被动关节）

Piper 机械臂的 8 个关节全部是主动关节（由电机驱动），不需要设置。

#### 第七步：Controller Configuration（控制器配置）

**关键步骤！** 这里告诉 MoveIt 用什么控制器来驱动机器人。

点 **"Add Controller"** 两次：

**arm_controller：**
- Controller Name: `arm_controller`
- Controller Type: `FollowJointTrajectory`
- Joints: 选中 `joint1` ~ `joint6`

**gripper_controller：**
- Controller Name: `gripper_controller`
- Controller Type: `FollowJointTrajectory`
- Joints: 选中 `joint7` ~ `joint8`

#### 第八步：Generate Package

- Package Name: `piper_moveit`（或你的命名）
- Package Path: 选择输出目录
- 点 **"Generate Package"**

### 4.3 Setup Assistant 自动生成了什么？

```
piper_moveit/
├── config/
│   ├── piper.srdf                 # 语义描述（规划组、碰撞对）
│   ├── kinematics.yaml            # 运动学求解器配置
│   ├── joint_limits.yaml          # 关节限位
│   ├── moveit_controllers.yaml    # 控制器映射（给 MoveIt 看）
│   └── ros2_controllers.yaml      # 控制器类型定义（给 controller_manager 看）
├── launch/
│   ├── demo.launch.py             # 一键启动（RViz 演示）
│   ├── move_group.launch.py       # 仅 move_group
│   ├── moveit_rviz.launch.py      # 仅 RViz
│   ├── rsp.launch.py              # 仅 robot_state_publisher
│   ├── spawn_controllers.launch.py
│   ├── static_virtual_joint_tfs.launch.py
│   ├── warehouse_db.launch.py
│   └── setup_assistant.launch.py
└── .setup_assistant               # 元数据（记录上次使用的 URDF 路径）
```

---

## 5. 配置完后的手动修复（必看）

Setup Assistant 自动生成的配置并非完美可用。根据实际经验，以下 **4 个地方必须手动调整**：

### 修复 1：joint_limits.yaml — 整数类型错误

**问题：** Setup Assistant 生成的限位值是整数，MoveIt2 要求 double 类型。

```yaml
# ── 生成的是这样（会崩溃）──
joint1:
  has_velocity_limits: true
  max_velocity: 5           # ❺ ← 整数！MoveIt2 报类型错误

# ── 必须改成这样 ──
joint1:
  has_velocity_limits: true
  max_velocity: 5.0         # ❺.0 ← double

# 同样对所有 max_acceleration: 0 也要改成 0.0
```

**报错日志：**
```
parameter 'robot_description_planning.joint_limits.joint1.max_velocity'
has invalid type: expected [double] got [integer]
```

这是 MoveIt2 的一个已知 bug（[Issue #2776](https://github.com/ros-planning/moveit2/issues/2776)）。

### 修复 2：kinematics.yaml — 去掉 gripper 的 KDL 条目（如果不小心为gripper配置的规划算法）

**问题：** Setup Assistant 给所有规划组都配了 KDL，但 gripper 不是链结构。

```yaml
# ── 生成的是这样 ──
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  # ...

gripper:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  # ...   ❌ ← 会报错！

# ── 必须改成 ──
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005

# gripper 条目完全删除
```

**报错日志：**
```
Group 'gripper' is not a chain. Kinematics solver could not be
instantiated for group 'gripper'.
```

**解释：** KDL（Kinematics and Dynamics Library）只能处理链式结构。gripper 在 SRDF 中定义为两个独立关节 `<joint name="joint7"/><joint name="joint8"/>`，不是一个链。不需要 IK 求解器，MoveIt 可以直接在关节空间规划夹爪运动。

### 修复 3：moveit_controllers.yaml — 补充 action_ns

**问题：** Setup Assistant 生成的控制器配置缺少 `action_ns` 字段。

```yaml
# ── 生成的是这样（MoveIt 找不到控制器）──
arm_controller:
  type: FollowJointTrajectory
  joints:
    - joint1
    - joint2
    # ...

# ── 必须加上 action_ns ──
arm_controller:
  type: FollowJointTrajectory
  action_ns: follow_joint_trajectory    # ✅ 必须加！
  joints:
    - joint1
    - joint2
    # ...
```

**报错日志：**
```
No action namespace specified for controller 'arm_controller'
```

**解释：** MoveIt Simple Controller Manager 通过 Action 接口与控制器通信。`FollowJointTrajectory` 类型的控制器默认 Action 路径是 `/arm_controller/follow_joint_trajectory`。如果不指定 `action_ns`，MoveIt 不知道去哪找这个 Action Server。

### 修复 4：use_sim_time — 时间同步（Gazebo 环境必配）

**问题：** Gazebo 使用仿真时间（自身内部的模拟时钟），而 MoveIt 和 RViz 默认使用系统时钟。如果不统一，时间差可达几十年，导致运动执行被中止。

```bash
# 报错日志
time diff: 1784522272.329s → Execute request aborted
```

**修复方法（很重要！）：**

```python
# 在 launch 文件的每个 Node 的 parameters 中加 use_sim_time
node_move_group = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    parameters=[
        moveit_config.to_dict(),
        move_group_params,
        {"use_sim_time": True},          # ✅ 必须加在这！
    ],
)

node_rviz = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["-d", rviz_config],
    parameters=[
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {"use_sim_time": True},          # ✅ 同样要加！
    ],
)
```

> ⚠️ **不要** 用全局的 `SetParameter(name="use_sim_time", value=True)` 来设置。实测发现 `SetParameter` 的时序会导致部分 Node 获取不到参数。必须将 `{"use_sim_time": True}` 显式放入每个 Node 的 `parameters` 列表中。

---

## 6. MoveIt 如何控制 Gazebo？

这是一个关键问题，理解了这个数据流，你就懂了整个 ROS 2 机器人仿真的核心。

### 6.1 整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                      ROS 2 中间件                            │
├──────────┬──────────────┬──────────────┬────────────────────┤
│  MoveIt  │  Controller  │   Gazebo     │  robot_state_      │
│  move_   │  Manager     │   物理引擎    │  publisher         │
│  group   │              │              │                    │
├──────────┼──────────────┼──────────────┼────────────────────┤
│  "规划器" │  "调度员"    │  "执行者"     │  "翻译官"           │
└────┬─────┴──────┬───────┴──────┬───────┴────────────────────┘
     │            │              │
     │    规划结果 │              │
     │───────────>│              │
     │  trajectory│              │
     │  msg       │    位置命令   │
     │            │─────────────>│
     │            │  joint1~8    │
     │            │  position    │
     │            │              │
     │            │              │    关节状态
     │            │              │───────────>
     │            │              │  /joint_states
     │            │              │
     │            │              │     TF 变换
     │            │              │───────────>
     │            │              │  /tf
     │            │              │
```

### 6.2 数据流详解（按时间顺序）

#### Step 1：用户发出运动请求

```
用户（代码或 RViz 拖拽）
  │ 设定目标位姿
  ▼
MoveIt MoveGroupInterface
  │ setPoseTarget() / setJointValueTarget()
  ▼
move_group 节点的规划流水线
```

#### Step 2：MoveIt 进行运动规划

```
move_group
  │  1. 读取 /robot_description（URDF）+ /robot_description_semantic（SRDF）
  │  2. 读取当前关节状态（/joint_states）
  │  3. 构建 PlanningScene（含碰撞检测）
  │  4. 调用 OMPL/Pilz 规划器 → 生成轨迹
  │  5. 轨迹验证
  ▼
  输出：moveit_msgs::msg::RobotTrajectory
       └─ 一系列 trajectory_msgs::msg::JointTrajectoryPoint
          每个点包含：joint1~6 的位置、速度、时间戳
```

#### Step 3：MoveIt 通过 Controller Manager 发送轨迹

```
move_group
  │  查询 moveit_controllers.yaml →
  │    arm_controller → FollowJointTrajectory → /arm_controller/follow_joint_trajectory
  │    gripper_controller → FollowJointTrajectory → /gripper_controller/follow_joint_trajectory
  │  发送 Action Goal
  ▼
controller_manager
  │  将 Action 转发给对应的 Controller 实例
  ▼
arm_controller (JointTrajectoryController 的实例)
  │  轨迹插值（trajectory sampling）
  │  计算每个控制周期（500Hz）的关节位置命令
  │  写入 ros2_control 硬件接口
  ▼
GazeboSystem 硬件插件
  │  从 ros2_control 读取位置命令
  ▼
Gazebo 物理引擎
  │  在仿真世界中设置关节位置
  │  物理引擎计算 → 更新刚体位姿 → 碰撞检测 → 下一帧
  ▼
  仿真世界中的机器人动了！
```

#### Step 4：状态反馈循环

```
Gazebo 物理引擎
  │  读取各关节的位置/速度
  ▼
GazeboSystem 硬件插件
  │  写回 ros2_control 状态接口
  ▼
joint_state_broadcaster
  │  从 controller_manager 读取关节状态
  │  发布到 /joint_states 话题
  ▼
robot_state_publisher
  │  订阅 /joint_states + 读取 /robot_description
  │  计算 TF 树（每个 link 相对于 base_link 的变换）
  │  发布到 /tf 话题
  ▼
MoveIt / RViz / 其他节点
  │  订阅 /joint_states + /tf
  │  更新规划场景 → 显示机器人当前姿态
```

### 6.3 关键配置映射关系

```
MoveIt 内部控制器名         Action 路径                           物理控制器
─────────────────────────────────────────────────────────────────
arm_controller     →  /arm_controller/follow_joint_trajectory  →  JointTrajectoryController
gripper_controller →  /gripper_controller/follow_joint_trajectory → JointTrajectoryController
(无)               →  /joint_states                            →  joint_state_broadcaster
```

这份映射在 `moveit_controllers.yaml` 中定义。

### 6.4 图解：控制数据流流向

```
                    ┌───────────────┐
                    │  launch 文件   │
                    │  spawner 加载  │
                    └──────┬────────┘
                           ▼
              ┌────────────────────────┐
              │  controller_manager    │
              │                        │
              │  ┌──────────────────┐  │
              │  │ arm_controller   │  │
              │  │ (JointTrajectory │  │
              │  │  Controller)     │  │  Action: /arm_controller/...
              │  └────────┬─────────┘  │
              │           │            │
              │  ┌────────▼─────────┐  │
              │  │ gripper_controller│  │  Action: /gripper_controller/...
              │  │ (JointTrajectory │  │
              │  │  Controller)     │  │
              │  └────────┬─────────┘  │
              │           │            │
              │  ┌────────▼─────────┐  │
              │  │ joint_state_     │  │  话题: /joint_states
              │  │ broadcaster      │  │
              │  └──────────────────┘  │
              └────────────────────────┘
                         │
              ┌──────────┴──────────┐
              │  GazeboSystem 硬件   │
              │  插件                │
              │  (libgazebo_ros2_   │
              │   control.so)       │
              └──────────┬──────────┘
                         │
              ┌──────────┴──────────┐
              │  Gazebo 物理引擎     │
              │  (Physics Engine)   │
              │  ODE / Bullet       │
              └─────────────────────┘
```

---

## 7. 为什么要自定义服务 + Gazebo 插件？

### 7.1 问题：Gazebo 的 "抓取" 怎么做？

当机械臂移动到物体位置后，如何让物体"粘"在夹爪上一起运动？

**不好的方案：** 缩小夹爪间距夹住物体。
- 问题：Gazebo 物理引擎的摩擦模型不够精确。即使夹爪闭合，物体仍可能滑落。
- 问题：物体和夹爪之间的碰撞交互非常不稳定，容易弹飞。

**好的方案：** 在 Gazebo 物理引擎中创建一个 **固定关节（Fixed Joint）**，将物体 Link 和夹爪 Link 刚性连接。

### 7.2 自定义 Service 接口

我们需要一个 Service 来告诉 Gazebo："把物体 X 固定到机器人的 Y Link 上"。

创建 `srv/AttachDetach.srv`：

```
# ── 请求 ──
string robot_model     # 机器人模型名，例如 "piper"
string robot_link      # 机器人上的连接点，例如 "link6"
string object_model    # 物体模型名，例如 "green_cylinder"
string object_link     # 物体上的连接点，本项目中统一为 "link"
---
# ── 响应 ──
bool success           # 是否成功
string message         # 成功或失败说明
```

为什么不用现有的标准 Service？因为 ROS 2 没有预定义"吸附物体"这种服务，这完全是应用层需求。

### 7.3 Gazebo WorldPlugin 插件

有了 Service 接口后，需要一个在 Gazebo 内部运行的节点来**真正执行**创建/销毁固定关节的操作。这个节点不能是普通的 ROS 2 节点，因为它需要直接调用 Gazebo 的 C++ API（`physics::Joint`）。

#### 插件提供什么功能？

```
LinkAttacherPlugin (Gazebo WorldPlugin)
├── Load() — 插件入口
│   ├── 创建 ROS 2 节点（通过 gazebo_ros::Node::Get）
│   └── 注册两个 Service：
│       ├── /piper/attach   → handleAttach()
│       └── /piper/detach   → handleDetach()
│
├── handleAttach() — 吸附物体
│   ├── 解析请求中的模型名/Link 名
│   ├── 通过 Gazebo API 获取 Link 指针
│   └── 创建 Fixed Joint：robot_link → object_link
│       physics->CreateJoint("fixed", robot_link->GetModel())
│       joint->Attach(robot_link, object_link)
│       joint->Load(...)
│       joint->Init()
│
├── handleDetach() — 分离物体
│   ├── 查找先前创建的 Joint
│   └── 销毁 Joint：Detach() → Fini()
│
└── 线程安全
    └── std::mutex 保护 attached_joints_ 字典
```

#### 为什么必须是 WorldPlugin？

| 插件类型 | 加载时机 | 适用场景 |
|---------|---------|---------|
| WorldPlugin | 世界加载时 | 需要访问世界中的全部模型和 Link |
| ModelPlugin | 模型加载时 | 只操作单个模型 |

我们的 attach 操作需要在**机器人模型**的 Link 和**物体模型**的 Link 之间创建关节，同时访问两个不同的模型，因此必须用 WorldPlugin。

#### 插件的 SDF 配置

在 world 文件的 `<world>` 段中加入：

```xml
<plugin name="piper_link_attacher" filename="libpiper_link_attacher.so"/>
```

Gazebo 加载 world 文件时会自动：
1. 加载 `libpiper_link_attacher.so` 动态库
2. 实例化 `LinkAttacherPlugin` 类
3. 调用 `Load()` 方法
4. 注册 `/piper/attach` 和 `/piper/detach` Service

### 7.4 整个抓取/释放的 Service 调用时序

```
pick_and_place 节点                    LinkAttacherPlugin
      │                                     │
      │  1. 机械臂下降到物体位置               │
      │  2. 张开夹爪（使物体进入夹爪中央）          │
      │                                     │
      │  3. Service Call: /piper/attach      │
      │     {robot_model: "piper",           │
      │      robot_link: "link6",           │
      │      object_model: "green_cylinder",│
      │      object_link: "link"}           │
      │ ─────────────────────────────────>   │
      │                                     │  4. 创建固定关节
      │                                     │     link6 → green_cylinder::link
      │  5. Service Response: success=true   │
      │ <─────────────────────────────────   │
      │                                     │
      │  6. 物体随机械臂一起运动              │
      │     （固定关节约束）                   │
      │                                     │
      │  7. 机械臂移动到码放位置               │
      │  8. 保持夹爪张开                         │
      │                                     │
      │  9. Service Call: /piper/detach     │
      │ ─────────────────────────────────>   │
      │                                     │  10. 销毁固定关节
      │  11. Service Response: success=true  │
      │ <─────────────────────────────────   │
      │                                     │
      │  12. 物体留在码放位置，机械臂收回      │
```

### 7.5 环境变量设置（易踩坑的细节）

插件编译后的 `.so` 文件在 `install/lib/piper_gazebo/` 下，Gazebo 需要搜索该路径。因此在 launch 文件中必须设置：

```python
# 设置 GAZEBO_PLUGIN_PATH 让 Gazebo 找到插件
gazebo_plugin_path = SetEnvironmentVariable(
    "GAZEBO_PLUGIN_PATH",
    f"{plugin_lib}:{os.environ.get('GAZEBO_PLUGIN_PATH', '')}",
)

# 设置 LD_LIBRARY_PATH 让插件 dlopen 时能找到 ROS2 typesupport 库
gazebo_lib_path = SetEnvironmentVariable(
    "LD_LIBRARY_PATH",
    f"{typesupport_lib}:{os.environ.get('LD_LIBRARY_PATH', '')}",
)
```

> 如果不设置 `GAZEBO_PLUGIN_PATH`，Gazebo 加载 world 时找不到 `libpiper_link_attacher.so`，插件静默失败，Service 端口注册不上。

---

## 8. Pick-and-Place 设计思路与工作流程

### 8.1 设计思路

`pick_and_place.cpp` 是整个项目的核心，设计上遵循以下原则：

**原则一：笛卡尔直线优先，失败降级**
- 抓取/码放要求末端沿直线运动（避免碰到物体或台面）
- 优先使用 `computeCartesianPath()` 做笛卡尔规划
- 如果规划覆盖率 < 90%，_降级_ 到关节空间规划（`setPoseTarget()`）

**原则二：基于 TCP（工具中心点）规划**
- 机械臂末端有夹爪（Tool），夹爪的 TCP 相对于 link6 有 0.13m 的 Z 向偏移
- 在规划时，先计算 TCP 的目标位姿，再反向补偿 offset 得到 link6 的目标位姿
- 这样用户可以在"夹爪尖"的坐标系下思考，不需要操心 link6 的位置

**原则三：运行时检测（Runtime Detection）**
- 物体可能被之前的操作碰撞移动位置
- 每次抓取前（甚至 approach 后）都重新通过 `/get_entity_state` 读取物体位置
- 不依赖固定坐标

**原则四：容错重试**
- home 动作重试最多 5 次（MoveIt 首次连接 arm_controller 需要时间）
- 整体抓取循环重试最多 3 轮
- 每个步骤失败后清理（detach + open），避免机械臂挂着物体乱动

### 8.2 工作流程（8 步骤详解）

```
┌──────────┬──────────────────────────────────────────────────┐
│   步骤   │ 动作名称    │ 说明                                │
├──────────┼──────────────────────────────────────────────────┤
│  Step 0  │ update      │ 通过 /get_entity_state 读取物体     │
│          │ Object      │ 实时位置（XYZ 坐标）                │
├──────────┼──────────────────────────────────────────────────┤
│  Step 1  │ approach    │ 从当前位置移动到物体正上方           │
│          │             │ 高度 = max(抬升高度, 物体高度 + 安全间隙)│
│          │             │ 使用笛卡尔直线规划                   │
├──────────┼──────────────────────────────────────────────────┤
│  Step 2  │ descent     │ 直线下降到抓取高度                   │
│          │             │ 高度 = max(物体底部 + 偏移, 最低抓取高度)│
│          │             │ 使用笛卡尔直线规划                   │
├──────────┼──────────────────────────────────────────────────┤
│  Step 3  │ attach      │ 调用 /piper/attach 将物体固定在末端  │
│          │             │ Gazebo 创建固定关节                  │
├──────────┼──────────────────────────────────────────────────┤
│  Step 4  │ lift        │ 抬升到安全高度（物体随末端一起上升）  │
│          │             │ 使用笛卡尔直线规划                   │
├──────────┼──────────────────────────────────────────────────┤
│  Step 5  │ transfer    │ 水平移动到码放位置上方               │
│          │             │ 使用笛卡尔直线规划                   │
├──────────┼──────────────────────────────────────────────────┤
│  Step 6  │ release     │ 下降到释放高度                      │
│          │             │ 使用笛卡尔直线规划                   │
├──────────┼──────────────────────────────────────────────────┤
│  Step 7  │ detach      │ 张开夹爪 + 调用 /piper/detach 分离   │
│          │ + open      │ Gazebo 销毁固定关节                  │
├──────────┼──────────────────────────────────────────────────┤
│  Step 8  │ retract     │ 抬升离开码放位置                    │
│          │             │ 使用笛卡尔直线规划                   │
└──────────┴──────────────────────────────────────────────────┘
```

### 8.3 数据流时序图

```
时间
│
│  main() 启动
│    │
│    ├─ waitForRuntimeDependencies()   ← 等待 Gazebo / 控制器 / 插件就绪
│    ├─ waitForMoveGroupAction()       ← 等待 MoveIt move_group 就绪
│    ├─ configureMoveGroup()           ← 配置规划参数
│    │
│    ├─ moveJoints(home_joints)        ← 回到 Home 姿态
│    │   └── MoveIt 规划 → arm_controller → Gazebo 执行
│    │
│    ├─ openGripper()                  ← 张开夹爪
│    │   └── Gripper Action → gripper_controller → Gazebo 执行
│    │
│    └─ 主循环（对每个物体）：
│        │
│        ├─ [Step 0] readObjectPosition(object)
│        │   └── /get_entity_state Service → Gazebo → 返回 XYZ
│        │
│        ├─ [Step 1] moveLinear(approach_pose)
│        │   └── MoveIt Cartesian规划 → arm_controller → Gazebo
│        │
│        ├─ [Step 2] moveLinear(grasp_pose)
│        │   └── MoveIt Cartesian规划 → arm_controller → Gazebo
│        │
│        ├─ [Step 3] attachObject(object)
│        │   └── /piper/attach Service → LinkAttacherPlugin → 创建固定关节
│        │
│        ├─ [Step 4] moveLinear(lift_pose)
│        ├─ [Step 5] moveLinear(transfer_pose)
│        ├─ [Step 6] moveLinear(release_pose)
│        │
│        ├─ [Step 7] openGripper() + detachObject(object)
│        │   └── /piper/detach Service → LinkAttacherPlugin → 销毁固定关节
│        │
│        ├─ [Step 8] moveLinear(retract_pose)
│        │
│        ├─ moveJoints(home_joints)    ← 回到 Home，继续抓下一个
│        │
│        └─ ... (下一个物体)
│
│  所有物体完成 → moveJoints(zero_joints) → shutdown
▼
程序退出
```

### 8.4 代码模块职责划分

```
pick_and_place.cpp
│
├── 匿名命名空间
│   ├── 常量定义（节点名/规划组名/末端Link/步长等）
│   ├── 数据结构（Object / Vec3 / ObjectPoses）
│   ├── objects()          ← 定义待抓取物体列表
│   ├── rotate()           ← 四元数旋转向量
│   ├── linkPoseForToolPoint()  ← TCP → link6 位姿转换
│   └── makeObjectPoses()  ← 生成 6 步位姿序列
│
├── PickNode 类
│   ├── 构造函数            ← 创建 Action/Service 客户端
│   ├── waitForRuntimeDependencies() ← 等待依赖就绪
│   ├── openGripper()       ← 张开夹爪
│   ├── readObjectPosition()← 读取物体位置
│   ├── attachObject()      ← 吸附物体
│   └── detachObject()      ← 分离物体
│
├── 全局函数
│   ├── moveLinear()        ← 笛卡尔规划 + 降级
│   ├── moveJoints()        ← 关节空间规划
│   ├── configureMoveGroup()← MoveIt 配置
│   ├── waitForMoveGroupAction() ← 等待 move_group
│   ├── updateObject()      ← 更新物体位置
│   ├── cleanupObject()     ← 失败清理
│   └── pickObject()        ← 单个物体的完整 8 步抓取
│
└── main()
    ├── 初始化 ROS2
    ├── 后台 executor spin
    ├── 等待依赖 → 配置 MoveIt → 回 Home
    ├── 主循环（3 轮 × 3 物体）
    ├── 回到零位
    └── 返回状态码
```

---

## 9. 附录：完整文件结构与启动命令

### 9.1 项目文件总览

```
src/
├── piper_description/                # 机器人模型
│   ├── urdf/
│   │   ├── piper_description.urdf    # 纯 URDF（SolidWorks 导出）
│   │   ├── piper_description.xacro   # xacro 版（transmission 方式）
│   │   └── piper_description_gazebo.xacro # Gazebo 版（GazeboSystem 方式）★ 核心
│   └── meshes/*.STL                  # 3D 网格文件
│
├── piper_gazebo/                     # Gazebo 仿真 + 抓取逻辑
│   ├── src/
│   │   ├── pick_and_place.cpp        # ★ 抓取主程序
│   │   └── link_attacher_plugin.cpp  # ★ 吸附插件
│   ├── srv/AttachDetach.srv          # ★ 自定义服务接口
│   ├── config/ros2_controllers.yaml  # 控制器参数（500Hz）
│   ├── worlds/piper_pick.world       # 抓取场景（含 3 个物体 + 插件）
│   └── launch/piper_with_gripper/
│       ├── piper_gazebo.launch.py           # 基础版
│       ├── piper_gazebo_moveit.launch.py    # 调试版
│       └── piper_pick_and_place.launch.py   # ★ 完整版（一键运行）
│
└── piper_moveit/                     # MoveIt2 配置（由 Setup Assistant 生成 + 手动修复）
    ├── config/
    │   ├── piper.srdf                # 语义描述（规划组 + 碰撞矩阵）
    │   ├── kinematics.yaml           # 运动学求解器（KDL）
    │   ├── joint_limits.yaml         # 关节限位（需手动加 .0 修复）
    │   ├── moveit_controllers.yaml   # 控制器映射（需手动加 action_ns 修复）
    │   └── ros2_controllers.yaml     # 控制器类型定义
    └── launch/*.launch.py            # 8 个 launch 文件
```

### 9.2 启动命令速查

```bash
# ★ 一键运行（推荐）
ros2 launch piper_gazebo piper_pick_and_place.launch.py

# 分步启动（调试用）
终端1: ros2 launch piper_gazebo piper_pick_and_place.launch.py
# 上面一条命令等价于以下分步操作：

# 仅启动 Gazebo + 机器人 + 控制器
ros2 launch piper_gazebo piper_gazebo.launch.py

# 启动 MoveIt + RViz（需要在 Gazebo 启动后执行）
ros2 launch piper_moveit move_group.launch.py     # 注意：需要加 use_sim_time
ros2 launch piper_moveit moveit_rviz.launch.py    # 注意：需要加 use_sim_time

# MoveIt 纯演示（无 Gazebo）
ros2 launch piper_moveit demo.launch.py
```

---

## 结语

从 SolidWorks 导出的 URDF 到 Gazebo 中自动抓取码放，整个开发流程可以概括为：

```
URDF 导出
  ↓ 添加惯性参数 + ros2_control 段 + Gazebo 插件
Gazebo URDF
  ↓ MoveIt Setup Assistant
MoveIt 配置包
  ↓ 手动修复四大问题（integer limits / gripper KDL / action_ns / use_sim_time）
可用的 MoveIt 配置
  ↓ 添加自定义 Service 接口 + Gazebo WorldPlugin
可吸附物体的仿真环境
  ↓ 编写 pick_and_place.cpp（笛卡尔优先 + 8 步流程 + 容错重试）
自动抓取码放演示
```

每一步都有或多或少的坑。希望这份教程能帮你少走弯路。

