"""
piper_moveit/launch/warehouse_db.launch.py
====================================================================

【文件概述】
启动 MoveIt 的规划场景数据库（Warehouse DB）。

【背景】
  MoveIt 的 Warehouse 是一个可选的 MongoDB 后端，
  用于持久化存储：
  - 规划场景（PlanningScene）快照
  - 已规划的轨迹
  - 机器人配置

  默认不启动，仅当需要使用场景存储/回放功能时启用。

【功能】
  - 启动 MongoDB 数据库实例
  - 配置 Warehouse 连接参数供 move_group 使用

【适用场景】
  - 需要保存和回放规划场景
  - 多机器人共享配置
  - 离线规划和批量处理

【用法】
  ros2 launch piper_moveit warehouse_db.launch.py

【注意】
  需要预先安装 MongoDB。

【依赖】
  委托给 moveit_configs_utils 的 generate_warehouse_db_launch 模板。
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_moveit").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
