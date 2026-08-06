"""
graspnet_pipeline.launch.py

【文件概述】
GraspNet 感知流水线启动文件 — 一键启动：
  1. Gazebo Server（带物体 + 顶部 RGB-D 相机，不含机械臂）
  2. robot_state_publisher（仅发布 camera_link 等 TF）
  3. camera_bridge_node（将相机话题写入共享目录供 GraspNet 使用）

【相机话题】
  /camera/color/image_raw   (sensor_msgs/Image)
  /camera/depth/image_raw   (sensor_msgs/Image)
  /camera/camera_info       (sensor_msgs/CameraInfo)

【共享目录】
  ~/piper_graspnet/shared_camera_data/
    color.png, depth.png, meta.mat, camera_info.txt, .ready

【用法】
  ros2 launch piper_gazebo graspnet_pipeline.launch.py

【提示】
  在另一个终端使用 ./graspnet-docker-camera.sh 启动 GraspNet 推理容器，
  容器将读取共享目录中的数据进行识别并将结果显示在屏幕上。
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 感知-only 流水线（无机械臂）：
    #   Gazebo（带物体+相机，无机械臂以免遮挡）
    #     → robot_state_publisher（发布相机 TF）
    #     → camera_bridge_node（相机话题 → 共享目录）
    # 之后在另一个终端运行 ./graspnet-docker-camera.sh 启动 Docker 推理。
    #
    # ── 参数 ──────────────────────────────────────────────────────
    # World 路径逻辑：优先使用安装后的世界文件路径，回退到源目录。
    # 为什么需要回退：FindPackageShare 依赖 ament 索引，若包未被
    # colcon build（或索引未更新）会抛异常；此时直接指向 src 下的
    # 源文件兜底。注意：world 文件共有 4 份副本（src 1 + install 3），
    # 修改后必须同步并重新 build（历史教训：只改 src 会"改了没生效"）。
    try:
        world_file = os.path.join(
            FindPackageShare('piper_gazebo').find('piper_gazebo'),
            'worlds/piper_pick_graspnet.world'
        )
    except Exception:
        world_file = ''
    if not world_file or not os.path.exists(world_file):
        world_file = os.path.join(
            os.path.expanduser('~'),
            'piper_graspnet/gazebo_piper_pick_place/src/piper_gazebo/worlds/piper_pick_graspnet.world'
        )

    # 共享目录：Docker 容器以只读挂载方式共享这里（见 graspnet-docker-camera.sh）
    shared_dir = os.path.join(os.path.expanduser('~'), 'piper_graspnet', 'shared_camera_data')

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

    # ── 3. 相机桥接节点（ROS → 共享目录） ─────────────────────────
    # 注意此处 executable='camera_bridge_node.py' 直接执行
    # piper_gazebo/scripts/ 下的脚本副本（与 piper_perception 包的
    # console_scripts 入口同源，两份必须保持同步）。
    camera_bridge_node = Node(
        package='piper_gazebo',
        executable='camera_bridge_node.py',
        name='camera_bridge_node',
        parameters=[{
            'shared_dir': shared_dir,
            'publish_rate_hz': 15.0,   # 与 Gazebo 相机帧率一致
            'factor_depth': 1000.0,    # 深度米 → 毫米
        }],
        output='screen',
    )

    # ── 组装 LaunchDescription ────────────────────────────────────
    ld = LaunchDescription()
    ld.add_action(start_gazebo_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(camera_bridge_node)

    return ld
