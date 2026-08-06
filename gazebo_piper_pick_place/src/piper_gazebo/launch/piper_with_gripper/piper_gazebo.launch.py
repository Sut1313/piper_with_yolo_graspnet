"""
piper_gazebo/launch/piper_with_gripper/piper_gazebo.launch.py
====================================================================

【文件概述】
基础版 Gazebo 启动文件 — 仅启动 Gazebo 仿真环境和机器人模型，
包含 ros2_control 控制器，但不包含 MoveIt2 和 RViz。

适用于：
  - 快速验证 URDF 加载和控制器配置
  - 单独调试 ros2_control/JointTrajectoryController
  - 资源受限环境下只跑仿真不跑规划

【启动顺序】
  1. 编译 piper_description_gazebo.xacro → 获得完整 URDF
  2. 启动 Gazebo Server（带 ros2_control 硬件插件）
  3. robot_state_publisher（发布 TF + robot_description）
  4. spawn_entity（在 Gazebo 中生成机器人）
  5. → 顺序加载控制器：joint_state_broadcaster → arm_controller + gripper_controller

【用法】
  ros2 launch piper_gazebo piper_gazebo.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.event_handlers import OnProcessExit

import xacro

import re
def remove_comments(text):
    """去除 URDF/XML 字符串中的注释 <!-- ... -->"""
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)


def generate_launch_description():
    # 本 launch 是整个抓取系统的最小子集：只启动 Gazebo + 机械臂 + 控制器，
    # 不启动 MoveIt/RViz/相机桥接/抓取节点——用于快速验证 URDF 与
    # ros2_control 配置是否正常（如关节能否被控制器驱动）。
    robot_name_in_model = 'piper'
    package_name = 'piper_description'
    urdf_name = "piper_description_gazebo.xacro"

    # ── 找到 xacro 文件路径 ──
    # FindPackageShare 解析安装目录（install/.../share/piper_description），
    # 修改 URDF 后必须重新 build 才会生效。
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    # ── 启动 Gazebo 服务器 ──
    # -s 参数加载必需的 Gazebo ROS 桥接插件
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # ── 用 xacro 编译 URDF（展开所有宏） ──
    # xacro 文件中引用 $(find mybot) 等宏调用，必须预处理
    xacro_file = urdf_model_path
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': remove_comments(doc.toxml())}

    # ── robot_state_publisher ──
    # 启动后发布 /robot_description 话题（包含完整 URDF）
    # 并计算机器人各 Link 间的 TF 变换
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency": 15.0}],
        output='screen'
    )

    # ── 在 Gazebo 世界中生成机器人 ──
    # 通过 /robot_description 话题读取 URDF 内容
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-topic', 'robot_description'],
        output='screen'
    )

    # ── 加载 ros2_control 控制器 ──
    # joint_state_broadcaster：
    #   发布所有关节状态到 /joint_states 话题
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # arm_controller：
    #   JointTrajectoryController，接收轨迹并驱动 6 个关节
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    # gripper_controller：
    #   另一个 JointTrajectoryController，驱动夹爪的 2 个关节
    load_gripper_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen'
    )

    # ── 事件链：确保控制器按顺序加载 ──
    # 生成机器人 → 加载 joint_state_broadcaster
    close_evt1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_cmd,
            on_exit=[load_joint_state_controller],
        )
    )

    # joint_state_broadcaster 完成 → 同时加载 arm_controller + gripper_controller
    close_evt2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_joint_trajectory_controller,
                     load_gripper_trajectory_controller],
        )
    )

    # ── 组装 LaunchDescription ──
    ld = LaunchDescription()

    # 事件处理器必须在目标动作之前添加才能生效
    ld.add_action(close_evt1)
    ld.add_action(close_evt2)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity_cmd)

    return ld
