"""
graspnet_pipeline.launch.py — GraspNet 感知流水线

一键启动：
  1. Gazebo Server（带物体 + 顶部 RGB-D 相机，不含机械臂）
  2. yolo_bridge_node（同步 RGB-D，执行 YOLO 分割并写入共享目录）

用法：
  ros2 launch piper_perception graspnet_pipeline.launch.py

Docker 推理（另一个终端）：
  ./graspnet-docker-camera.sh gui
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from piper_yolo.paths import default_model_path, default_shared_dir


def generate_launch_description():
    # 感知版启动（无机械臂）：Gazebo（piper_pick_graspnet.world：三个物体
    # + 顶部 RGB-D 相机）→ yolo_bridge_node（RGB-D + YOLO mask → 共享目录）。
    # 之后在另一个终端运行 ./graspnet-docker-camera.sh gui 启动 Docker 推理。
    #
    # 与 piper_gazebo 包内的 graspnet_pipeline.launch.py 功能相同，
    # 本文件属于 piper_perception 包，二者保持同步。
    #
    # ── 世界文件路径 ──────────────────────────────────────────────
    gazebo_share = get_package_share_directory('piper_gazebo')
    world_file = os.path.join(gazebo_share, 'worlds', 'piper_pick_graspnet.world')

    # 共享目录：Docker 容器挂载点 /shared_camera_data 对应的宿主机目录
    shared_dir = str(default_shared_dir(gazebo_share))

    # ── Gazebo 插件路径 ───────────────────────────────────────────
    # ROS 2 Humble 中，libgazebo_ros_camera.so 等插件位于 /opt/ros/humble/lib/
    # 但 GAZEBO_PLUGIN_PATH 默认未设置 → 需显式添加
    ros_lib_path = os.path.join(os.environ.get('ROS_INSTALL_PATH', '/opt/ros/humble'), 'lib')
    gazebo_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    if ros_lib_path not in gazebo_plugin_path:
        if gazebo_plugin_path:
            gazebo_plugin_path = f'{ros_lib_path}:{gazebo_plugin_path}'
        else:
            gazebo_plugin_path = ros_lib_path

    # ── 1. 启动 Gazebo ───────────────────────────────────────────
    start_gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        additional_env={
            'GAZEBO_PLUGIN_PATH': gazebo_plugin_path,
        },
        output='screen',
    )

    # ── 2. YOLO 桥接节点（同步 RGB-D → 实例掩膜 → 共享目录） ─────────
    yolo_bridge_node = Node(
        package='piper_yolo',
        executable='yolo_bridge_node',
        name='yolo_bridge_node',
        parameters=[{
            'shared_dir': shared_dir,
            'model_path': str(default_model_path(gazebo_share)),
            'factor_depth': 1000.0,
        }],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(start_gazebo_cmd)
    ld.add_action(yolo_bridge_node)

    return ld
