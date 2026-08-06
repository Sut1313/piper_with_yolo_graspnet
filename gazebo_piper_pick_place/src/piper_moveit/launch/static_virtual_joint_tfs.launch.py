"""
piper_moveit/launch/static_virtual_joint_tfs.launch.py
====================================================================

【文件概述】
发布虚拟关节的静态 TF 变换。

【背景】
  MoveIt 的规划场景需要一个 "虚拟关节"（virtual_joint）来连接机器人
  的 base_link 与世界坐标系（world）。这个虚拟关节在 SRDF 中定义，
  通常是一个固定的 6-DOF 关节（即 base_link 相对于 world 的位姿）。

【功能】
  - 发布从 "world" 到机器人 base_link 的静态 TF 变换
  - 使 MoveIt 知道机器人在地图中的位置

【数据流】
  static_transform_publisher
    → world -> base_link (fixed transform)

【适用场景】
  - 在非 Gazebo 环境下使用 MoveIt 时需要（如纯 RViz 演示）
  - 在 Gazebo 中此变换由 Gazebo 自动处理，通常不需要此 launch

【用法】
  ros2 launch piper_moveit static_virtual_joint_tfs.launch.py

【依赖】
  委托给 moveit_configs_utils 的 generate_static_virtual_joint_tfs_launch 模板。
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_moveit").to_moveit_configs()
    return generate_static_virtual_joint_tfs_launch(moveit_config)
