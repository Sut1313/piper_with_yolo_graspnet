"""
graspnet_pipeline.launch.py

【文件概述】
GraspNet 感知流水线启动文件 — 一键启动：
  1. Gazebo Server（带物体 + 顶部 RGB-D 相机，不含机械臂）
  2. robot_state_publisher（仅发布 camera_link 等 TF）
  3. yolo_bridge_node（同步 RGB-D、执行实例分割并写入共享目录）

【相机话题】
  /camera/image_raw         (sensor_msgs/Image)
  /camera/depth/image_raw   (sensor_msgs/Image)
  /camera/camera_info       (sensor_msgs/CameraInfo)

【共享目录】
  <项目根目录>/shared_camera_data/
    color.png, depth.png, meta.mat, instances.npz, detections.json, .ready

【用法】
  ros2 launch piper_gazebo graspnet_pipeline.launch.py

【提示】
  在另一个终端使用 ./graspnet-docker-camera.sh 启动 GraspNet 推理容器，
  容器将读取共享目录中的数据进行识别并将结果显示在屏幕上。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from piper_yolo.paths import default_model_path, default_shared_dir


def generate_launch_description():
    # 感知-only 流水线（无机械臂）：
    #   Gazebo（带物体+相机，无机械臂以免遮挡）
    #     → robot_state_publisher（发布相机 TF）
    #     → yolo_bridge_node（RGB-D + YOLO mask → 共享目录）
    # 之后在另一个终端运行 ./graspnet-docker-camera.sh 启动 Docker 推理。
    #
    # ── 参数 ──────────────────────────────────────────────────────
    gazebo_share = get_package_share_directory('piper_gazebo')
    world_file = os.path.join(gazebo_share, 'worlds', 'piper_pick_graspnet.world')

    # 共享目录：Docker 容器以只读挂载方式共享这里（见 graspnet-docker-camera.sh）
    shared_dir = str(default_shared_dir(gazebo_share))

    # ── 1. 启动 Gazebo Server ────────────────────────────────────
    start_gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': os.path.expanduser('~/.gazebo/models')},
    )

    # ── 2. robot_state_publisher（用于发布 camera 的 TF） ──────────
    # 注意：这里不指定 robot_description，仅用于确保 camera frame tree
    # （camera_link → 世界系）发布到 TF 树，供 RViz 等工具对齐显示。
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── 3. YOLO 桥接节点（同步 RGB-D → 实例掩膜 → 共享目录） ─────────
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

    # ── 组装 LaunchDescription ────────────────────────────────────
    ld = LaunchDescription()
    ld.add_action(start_gazebo_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(yolo_bridge_node)

    return ld
