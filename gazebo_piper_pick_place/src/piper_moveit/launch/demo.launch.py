"""
piper_moveit/launch/demo.launch.py
====================================================================

【文件概述】
MoveIt2 官方推荐的 "一键启动" 入口 — 启动完整的 MoveIt2 演示环境。

包含以下功能（由 generate_demo_launch 集成）：
  - robot_state_publisher（TF 树）
  - move_group（运动规划核心，/move_action Action Server）
  - RViz2 + MoveIt 插件（可视化交互）
  - static_virtual_joint_tfs（虚拟关节 TF）
  - warehouse_db（可选：运动规划场景存储）

不包含：
  - Gazebo 仿真（如需仿真请使用 piper_gazebo 中的 launch 文件）
  - 真实硬件接口

【数据流】
  用户在 RViz 中通过 MotionPlanning 面板设置目标
    → move_group 规划路径 → 在 RViz 中显示规划结果
    → 确认后通过 /move_action 执行

【用法】
  # 纯 MoveIt 演示（无仿真/硬件）
  ros2 launch piper_moveit demo.launch.py

【依赖】
  此函数委托给 moveit_configs_utils 的 generate_demo_launch 模板，
  由 MoveIt Setup Assistant 生成的配置驱动。
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # 读取 piper_moveit 包中的所有 MoveIt 配置
    # 包括：piper.srdf, kinematics.yaml, joint_limits.yaml,
    #       ros2_controllers.yaml, ompl_planning.yaml 等
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_moveit").to_moveit_configs()
    return generate_demo_launch(moveit_config)
