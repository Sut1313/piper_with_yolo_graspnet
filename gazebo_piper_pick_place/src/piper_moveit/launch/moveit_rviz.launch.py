"""
piper_moveit/launch/moveit_rviz.launch.py
====================================================================

【文件概述】
启动 RViz2 并加载 MoveIt MotionPlanning 面板。

【功能】
  - 加载预配置的 moveit.rviz 配置文件（包含 MotionPlanning 显示面板）
  - 配置规划流水线、运动学参数和关节限位
  - 与 move_group 节点通信，实现可视化交互规划

【数据流】
  用户在 RViz 中：
    "拖动"末端 → "Plan & Execute"
    → move_group 规划 → 在 RViz 中回放

【适用场景】
  - 在已经有 move_group 运行的情况下单独启动 RViz
  - 调试 RViz 配置

【用法】
  ros2 launch piper_moveit moveit_rviz.launch.py

【依赖】
  委托给 moveit_configs_utils 的 generate_moveit_rviz_launch 模板。
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_moveit").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
