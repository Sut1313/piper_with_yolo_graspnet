"""
piper_moveit/launch/rsp.launch.py
====================================================================

【文件概述】
启动 robot_state_publisher（RSP），发布机器人 TF 树和模型描述。

【功能】
  - 从 /robot_description 参数读取 URDF
  - 订阅 /joint_states 接收关节位置
  - 计算并发布所有 Link 间的 TF 变换
  - 为 MoveIt 和 RViz 提供机器人模型数据

【数据流】
  /robot_description (static) → RSP → TF tree (broadcast)
  /joint_states (from simulation/hardware) → RSP → Link transforms

【适用场景】
  - 需要将机器人模型发布到 ROS 2 网络
  - 作为其他 launch 文件的基础（通常由父 launch 隐式调用）

【用法】
  ros2 launch piper_moveit rsp.launch.py

【依赖】
  委托给 moveit_configs_utils 的 generate_rsp_launch 模板，
  自动处理 xacro 编译和参数配置。
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_moveit").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
