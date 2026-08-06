"""
graspnet_pipeline.launch.py — GraspNet 感知流水线

一键启动：
  1. Gazebo Server（带物体 + 顶部 RGB-D 相机，不含机械臂）
  2. camera_bridge_node（将相机话题写入共享目录供 GraspNet 使用）

用法：
  ros2 launch piper_perception graspnet_pipeline.launch.py

Docker 推理（另一个终端）：
  ./graspnet-docker-camera.sh gui
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 感知版启动（无机械臂）：Gazebo（piper_pick_graspnet.world：三个物体
    # + 顶部 RGB-D 相机）→ camera_bridge_node（相机话题 → 共享目录）。
    # 之后在另一个终端运行 ./graspnet-docker-camera.sh gui 启动 Docker 推理。
    #
    # 与 piper_gazebo 包内的 graspnet_pipeline.launch.py 功能相同，
    # 本文件属于 piper_perception 包，二者保持同步。
    #
    # ── 世界文件路径 ──────────────────────────────────────────────
    # 路径逻辑：install 优先（get_package_share_directory），失败/不存在时
    # 回退到 src 源目录。世界文件共 4 份副本（src 1 + install 3），
    # 修改后必须同步并重新 colcon build（历史教训：只改 src 会"改了没生效"）。
    try:
        gazebo_share = get_package_share_directory('piper_gazebo')
        world_file = os.path.join(gazebo_share, 'worlds', 'piper_pick_graspnet.world')
    except Exception:
        # 回退到源目录
        world_file = os.path.join(
            os.path.expanduser('~'),
            'piper_graspnet/gazebo_piper_pick_place/src/piper_gazebo/worlds/piper_pick_graspnet.world'
        )

    # 共享目录：Docker 容器挂载点 /shared_camera_data 对应的宿主机目录
    shared_dir = os.path.join(os.path.expanduser('~'), 'piper_graspnet', 'shared_camera_data')

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

    # ── 2. 相机桥接节点（ROS → 共享目录） ─────────────────────────
    # executable='camera_bridge_node' 是 setup.py 注册的 console_scripts
    # 入口点（对应 piper_perception/piper_perception/camera_bridge_node.py），
    # 与 piper_gazebo/scripts/ 下的脚本副本同源，修改时必须两处同步。
    camera_bridge_node = Node(
        package='piper_perception',
        executable='camera_bridge_node',
        name='camera_bridge_node',
        parameters=[{
            'shared_dir': shared_dir,
            'publish_rate_hz': 15.0,   # 与 Gazebo 相机帧率一致
            'factor_depth': 1000.0,    # 深度米 → 毫米
        }],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(start_gazebo_cmd)
    ld.add_action(camera_bridge_node)

    return ld
