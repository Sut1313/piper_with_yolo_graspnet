"""
piper_moveit/launch/setup_assistant.launch.py
====================================================================

【文件概述】
启动 MoveIt Setup Assistant（配置助手）GUI 工具。

【功能】
  MoveIt Setup Assistant 是一个图形化配置工具，用于：
  - 导入机器人 URDF 模型
  - 定义规划组（如 "arm"）
  - 配置运动学求解器（KDL / TRAC-IK）
  - 配置自碰撞矩阵
  - 生成 MoveIt 配置文件包（SRDF, kinematics.yaml 等）

【适用场景】
  - 首次配置新的机器人
  - 修改现有机器人的规划组或碰撞配置
  - 重新生成 piper_moveit 配置包

【用法】
  # 在已有配置基础上修改
  ros2 launch piper_moveit setup_assistant.launch.py

  # 从头开始新配置
  ros2 launch piper_moveit setup_assistant.launch.py \
    moveit_setup_assistant:=true

【注意】
  执行后打开 Qt 图形界面。
  配置完成后输出到 piper_moveit 包的 config/ 目录。

【依赖】
  委托给 moveit_configs_utils 的 generate_setup_assistant_launch 模板，
  需要安装 moveit_setup_assistant 包。
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_moveit").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
