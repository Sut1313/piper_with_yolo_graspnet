"""
piper_moveit/launch/move_group.launch.py
====================================================================

【文件概述】
仅启动 MoveIt2 的 move_group 核心节点，不包含 RViz 和其他辅助节点。

【move_group 的功能】
  - 运动规划（OMPL / Pilz 规划器）
  - 运动学正逆解（KDL / TRAC-IK）
  - 碰撞检测与规划场景维护
  - 轨迹验证
  - 提供 /move_action Action Server

【适用场景】
  - 需要在自定义 launch 文件中集成 move_group 但不启动 RViz
  - 在无头服务器环境中运行 MoveIt
  - 作为其他启动文件的基础组件

【用法】
  ros2 launch piper_moveit move_group.launch.py

【依赖】
  委托给 moveit_configs_utils 的 generate_move_group_launch 模板。
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_moveit").to_moveit_configs()
    return generate_move_group_launch(moveit_config)
