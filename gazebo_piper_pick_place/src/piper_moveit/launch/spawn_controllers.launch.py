"""
piper_moveit/launch/spawn_controllers.launch.py
====================================================================

【文件概述】
启动 ros2_control 控制器管理器中的控制器。

【功能】
  使用 controller_manager/spawner 加载并激活以下控制器：
  - joint_state_broadcaster：发布 /joint_states 话题
  - arm_controller：JointTrajectoryController，驱动 6 个关节
  - gripper_controller：JointTrajectoryController，驱动夹爪

【数据流】
  spawner → controller_manager → /arm_controller + /gripper_controller
                               → /joint_state_broadcaster → /joint_states

【适用场景】
  - 控制器已在 controller_manager 中注册但尚未激活
  - 需要单独重新加载控制器

【用法】
  ros2 launch piper_moveit spawn_controllers.launch.py

【注意】
  此 launch 文件假设：
  1. controller_manager 已在运行
  2. ros2_controllers.yaml 已正确配置（位于 config/ 目录）

【依赖】
  委托给 moveit_configs_utils 的 generate_spawn_controllers_launch 模板。
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_moveit").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
