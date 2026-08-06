"""
piper_gazebo/launch/piper_with_gripper/piper_gazebo_moveit.launch.py
====================================================================

【文件概述】
一键启动完整的 Gazebo 仿真 + MoveIt2 运动规划 + RViz 可视化环境。
用于调试机械臂运动规划和 Gazebo 交互，不包含自动抓取逻辑。

【启动的节点和动作（按顺序）】
  Gazebo 仿真层:
    1. Gazebo 服务器（带 ros2_control GazeboSystem 硬件插件）
    2. robot_state_publisher（发布 /robot_description + TF 树）
    3. spawn_entity.py（在 Gazebo 中生成机器人模型）
    4. → 加载 joint_state_broadcaster
       → 加载 arm_controller + gripper_controller

  MoveIt2 规划层:
    5. move_group 节点（运动规划核心，提供 /move_action）
    6. RViz2 + MoveIt 插件（可视化规划场景）

【数据流】
  robot_state_publisher
      └─ 发布 /robot_description + TF
          ├─ Gazebo 从中读取机器人模型
          ├─ MoveIt 从中读取 URDF/SRDF
          └─ RViz 从中读取机器人显示

  ros2_control (GazeboSystem)
      └─ 接收 MoveIt 的 trajectory_msgs
          ├─ /arm_controller → joint1~joint6
          └─ /gripper_controller → joint7~joint8

【用法】
  ros2 launch piper_gazebo piper_gazebo_moveit.launch.py
"""

import os
import re

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import SetParameter
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

from moveit_configs_utils import MoveItConfigsBuilder


def remove_comments(text: str) -> str:
    """去除 URDF/XML 字符串中的注释 <!-- ... -->"""
    return re.sub(r"<!--(.*?)-->", "", text, flags=re.DOTALL)


def generate_launch_description() -> LaunchDescription:
    # 本 launch 用于"纯运动规划调试"：Gazebo 空世界 + MoveIt + RViz，
    # 无相机、无物体、无自动抓取——与 piper_pick_and_place.launch.py 的区别：
    #   1. 不加载带物体的 world 文件（空世界）
    #   2. 不启动 camera_bridge_node 与 pick_and_place
    #   3. moveit_manage_controllers=False（控制器由本 launch 的事件链管理）
    #
    # ── 全局参数 ──────────────────────────────────────────────────────────
    # 所有节点都使用 Gazebo 仿真时间（/clock 话题），而非系统时钟
    set_sim_time = SetParameter(name="use_sim_time", value=True)

    # ── 1. 编译 Gazebo URDF ──────────────────────────────────────────────
    # 使用 piper_description_gazebo.xacro，该文件包含 ros2_control 的
    # GazeboSystem 硬件插件配置，使控制器能直接与 Gazebo 物理引擎通信。
    pkg_desc_share = FindPackageShare("piper_description").find("piper_description")
    urdf_path = os.path.join(pkg_desc_share, "urdf/piper_description_gazebo.xacro")
    doc = xacro.parse(open(urdf_path))
    xacro.process_doc(doc)
    robot_description = {"robot_description": remove_comments(doc.toxml())}

    # ── 2. 启动 Gazebo 服务器 ────────────────────────────────────────────
    # -s libgazebo_ros_init.so    : ROS2 与 Gazebo 之间的初始化桥接
    # -s libgazebo_ros_factory.so : 允许通过 ROS2 话题在 Gazebo 中生成模型
    start_gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    # ── 3. robot_state_publisher ─────────────────────────────────────────
    # 从 /robot_description 话题读取 URDF，计算并发布 TF 树
    # （base_link → link1 → ... → link6 → gripper_base → ...）
    # publish_frequency=15Hz 确保轨迹平滑显示
    node_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"publish_frequency": 15.0}],
        output="screen",
    )

    # ── 4. 在 Gazebo 中生成机器人 ────────────────────────────────────────
    # 从 /robot_description 话题读取 URDF 内容，在 Gazebo 世界中实例化机器人
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "piper", "-topic", "robot_description"],
        output="screen",
    )

    # ── 5. 加载 ros2_control 控制器 ──────────────────────────────────────
    # 控制器的加载顺序至关重要：
    #   spawn → joint_state_broadcaster → [arm_controller, gripper_controller]
    # joint_state_broadcaster 必须先加载，因为 arm_controller 依赖关节状态
    load_joint_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"],
        output="screen",
    )
    load_arm_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "arm_controller"],
        output="screen",
    )
    load_gripper_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "gripper_controller"],
        output="screen",
    )

    # 用 OnProcessExit 事件处理器实现顺序加载：
    #   生成完毕后 → 加载 joint_state_broadcaster
    on_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_broadcaster],
        )
    )
    #   joint_state_broadcaster 加载完毕后 → 同时加载 arm + gripper 控制器
    on_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_broadcaster,
            on_exit=[load_arm_controller, load_gripper_controller],
        )
    )

    # ── 6. 构建 MoveIt2 配置 ────────────────────────────────────────────
    # MoveItConfigsBuilder 自动从 piper_moveit 包中读取：
    #   - piper.srdf（语义机器人描述，定义规划组）
    #   - kinematics.yaml（运动学求解器配置）
    #   - joint_limits.yaml（关节限位）
    #   - ros2_controllers.yaml（控制器映射）
    #   - planning_pipelines（规划流水线，OMPL/Pilz）
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_moveit").to_moveit_configs()

    # ── 7. MoveIt2 move_group 节点 ──────────────────────────────────────
    # move_group 是 MoveIt 的核心节点，提供：
    #   - /move_action（运动规划与执行 Action）
    #   - /planning_scene（规划场景更新）
    #   - 运动学求解、碰撞检测、轨迹验证
    #
    # moveit_manage_controllers=False：在此启动中 MoveIt 不主动管理
    #   ros2_control 控制器（由 launch 文件管理）
    move_group_params = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": False,
        "capabilities": moveit_config.move_group_capabilities["capabilities"],
        "disable_capabilities": moveit_config.move_group_capabilities["disable_capabilities"],
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    node_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_params],
    )

    # ── 8. RViz2 + MoveIt 插件 ──────────────────────────────────────────
    # 使用预配置的 moveit.rviz 文件，包含 MotionPlanning 面板
    rviz_config = str(moveit_config.package_path / "config/moveit.rviz")

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # ── 组装 LaunchDescription ──────────────────────────────────────────
    ld = LaunchDescription()

    # 全局参数（排在前面，确保后续节点能继承）
    ld.add_action(set_sim_time)

    # Gazebo 基础
    ld.add_action(start_gazebo)
    ld.add_action(node_rsp)
    ld.add_action(spawn_entity)
    ld.add_action(on_spawn)            # spawn → joint_state_broadcaster
    ld.add_action(on_broadcaster)      # broadcaster → arm+gripper

    # MoveIt 规划 + 可视化
    ld.add_action(node_move_group)
    ld.add_action(node_rviz)

    return ld
