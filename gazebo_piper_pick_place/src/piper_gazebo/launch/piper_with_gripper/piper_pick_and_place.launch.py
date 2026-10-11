"""
piper_gazebo/launch/piper_with_gripper/piper_pick_and_place.launch.py
====================================================================

【文件概述】
完整的抓取演示启动文件 — 包含：
  1. Gazebo 仿真环境（带目标物体的世界文件）
  2. robot_state_publisher（TF 树）
  3. 机器人模型生成 + ros2_control 控制器加载
  4. MoveIt2 move_group（运动规划）
  5. RViz2 + MoveIt 插件（可视化）
  6. pick_and_place 节点（自动抓取码放）

启动后自动执行抓取：
  绿色圆柱(green_cylinder) → 红色方块(red_box) → 蓝色圆柱(blue_sphere)

【piper_pick.world 的特殊之处】
  该世界文件包含：
  - 三个抓取目标物体
  - 顶部 RGB-D 相机；Gazebo attach 插件由 4cm 中心距离门控使用
  - 地面和光照预设

【与 piper_gazebo_moveit.launch.py 的区别】
  - 加载了带物体的 world 文件而非空世界
  - 使用顶部相机和 GraspNet 输出驱动抓取
  - 使用 controller_manager/spawner 加载控制器（更稳定）
  - moveit_manage_controllers=True（MoveIt 接管控制器管理）
  - 额外启动 pick_and_place 执行节点

【用法】
  ros2 launch piper_gazebo piper_pick_and_place.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch_ros.actions import SetParameter
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro
import re

from ament_index_python.packages import get_package_prefix

from moveit_configs_utils import MoveItConfigsBuilder
from piper_yolo.paths import default_model_path, default_shared_dir


def project_shared_dir():
    return str(default_shared_dir(__file__))


def remove_comments(text: str) -> str:
    """去除 URDF/XML 字符串中的注释 <!-- ... -->"""
    return re.sub(r"<!--(.*?)-->", "", text, flags=re.DOTALL)


def cleanup_perception_files(context, *args, **kwargs):
    """删除上一次运行留下的 GraspNet 感知结果，防止启动时误用旧抓取位姿。

    清理三个文件：
      perception_targets.csv / perception_grasps.csv — 旧的抓取候选，
        若不删，pick_and_place 可能执行"上一轮"的位姿；
      pick_state.json — 旧的放置状态（pending 列表），若不删，
        本轮会误认为物体"已放置"而跳过抓取（历史坑：重启后不抓任何物体）。
    在 Gazebo 启动的同时（OpaqueFunction）执行，与后续节点并行、无依赖。
    """
    shared_dir = project_shared_dir()
    os.makedirs(shared_dir, exist_ok=True)
    for filename in ("perception_targets.csv", "perception_grasps.csv",
                     "pick_state.json", ".ready", "instances.npz",
                     "detections.json", "frame_manifest.json"):
        path = os.path.join(shared_dir, filename)
        try:
            os.remove(path)
            print(f"[piper_pick_and_place.launch] removed stale perception file: {path}")
        except FileNotFoundError:
            pass
    return []


def generate_launch_description() -> LaunchDescription:
    # ── 启动链总览 ──────────────────────────────────────────────────────
    #   1. 环境变量（GAZEBO_PLUGIN_PATH / LD_LIBRARY_PATH，必须在 Gazebo 前）
    #   2. Gazebo 服务器（加载带物体的 piper_pick.world）
    #   3. robot_state_publisher（/robot_description + TF 树）
    #   4. spawn_entity（在 Gazebo 中生成机械臂）
    #   5. 控制器事件链：spawn 完成 → joint_state_broadcaster
    #      → broadcaster 完成 → arm_controller + gripper_controller
    #   6. move_group（MoveIt 运动规划核心，moveit_manage_controllers=True
    #      由 MoveIt 接管控制器管理）
    #   7. yolo_bridge_node（同步 RGB-D → YOLO 实例掩膜 → GraspNet 共享目录）
    #   8. RViz + MoveIt 面板（可视化）
    #   9. pick_and_place（主执行节点，内部自行等待 move_group 就绪后自动抓取）
    #
    # ── 全局参数 ──────────────────────────────────────────────────────────
    # 使用 Gazebo 仿真时间，而非系统时钟
    set_sim_time = SetParameter(name="use_sim_time", value=True)

    # ── 环境变量 ──────────────────────────────────────────────────────────
    # 保留本包插件路径给 Gazebo ROS 组件使用；piper_pick.world 加载
    # link_attacher，但 pick_and_place 只会在张开夹爪且 TCP 到物体表面
    # 小于 4cm 时调用 /piper/attach。
    pkg_prefix = get_package_prefix("piper_gazebo")
    plugin_lib = os.path.join(pkg_prefix, "lib", "piper_gazebo")
    typesupport_lib = os.path.join(pkg_prefix, "lib")

    gazebo_plugin_path = SetEnvironmentVariable(
        "GAZEBO_PLUGIN_PATH",
        f"{plugin_lib}:{os.environ.get('GAZEBO_PLUGIN_PATH', '')}",
    )
    gazebo_lib_path = SetEnvironmentVariable(
        "LD_LIBRARY_PATH",
        f"{typesupport_lib}:{os.environ.get('LD_LIBRARY_PATH', '')}",
    )

    # ── 1. 编译 Gazebo URDF ──────────────────────────────────────────────
    pkg_desc_share = FindPackageShare("piper_description").find("piper_description")
    urdf_path = os.path.join(pkg_desc_share, "urdf/piper_description_gazebo.xacro")
    doc = xacro.parse(open(urdf_path))
    xacro.process_doc(doc)
    robot_description = {"robot_description": remove_comments(doc.toxml())}

    # ── 2. World 文件路径（含抓取物体） ────────────────────────────────────
    # piper_pick.world 包含：
    #   三个抓取目标物体（红/绿/蓝圆柱）
    #   顶部 RGB-D 相机；加载 LinkAttacherPlugin 供门控 attach 使用
    # FindPackageShare 解析的是"安装目录"（install/.../share/piper_gazebo），
    # 因此修改 src 下的 world 文件后必须重新 colcon build，否则 Gazebo 加载
    # 的还是 install 里的旧副本——world 文件共有 4 份副本
    # （src 1 份 + install 3 份），历史教训：只改 src 会导致"改了没生效"。
    world_path = os.path.join(
        FindPackageShare("piper_gazebo").find("piper_gazebo"),
        "worlds/piper_pick.world",
    )
    shared_dir = project_shared_dir()
    cleanup_perception = OpaqueFunction(function=cleanup_perception_files)

    # ── 3. 启动 Gazebo 服务器（加载带物体的世界） ────────────────────────────
    # 与空世界启动的区别：cmd 中包含 world_path 参数
    start_gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            world_path,
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    # ── 4. robot_state_publisher ─────────────────────────────────────────
    # 发布 /robot_description + TF 树（base_link → ... → link6 → gripper）
    node_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"publish_frequency": 15.0}],
        output="screen",
    )

    # ── 5. 在 Gazebo 中生成机器人 ────────────────────────────────────────
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "piper", "-topic", "robot_description"],
        output="screen",
    )

    # ── 6. 加载控制器（使用 controller_manager/spawner） ──────────────────
    # 此处使用 spawner 替代 ExecuteProcess("ros2 control load_controller")，
    # 因为 spawner 启动更可靠，会等待 controller_manager 就绪
    load_joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )
    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )
    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 事件链：spawn → broadcaster → [arm, gripper]
    on_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_broadcaster],
        )
    )
    on_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_broadcaster,
            on_exit=[load_arm_controller, load_gripper_controller],
        )
    )

    # ── 7. MoveIt2 配置 ─────────────────────────────────────────────────
    # 从 piper_moveit 包中读取所有 MoveIt 配置
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_moveit").to_moveit_configs()

    # ── 8. move_group（运动规划核心） ────────────────────────────────────
    # moveit_manage_controllers=True：
    #   让 MoveIt 自动通过 controller_manager 发现和管理控制器，
    #   而不是由 launch 文件在外部管理
    move_group_params = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": True,
        "capabilities": moveit_config.move_group_capabilities["capabilities"],
        "disable_capabilities": moveit_config.move_group_capabilities["disable_capabilities"],
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    node_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_params],
    )

    # ── 8.5 YOLO 桥接节点（同步 RGB-D → 二维实例掩膜 → 共享目录） ───────────
    yolo_bridge_node = Node(
        package="piper_yolo",
        executable="yolo_bridge_node",
        name="yolo_bridge_node",
        parameters=[{
            "shared_dir": shared_dir,
            "model_path": str(default_model_path(__file__)),
            "factor_depth": 1000.0,
        }],
        output="screen",
    )

    # ── 9. RViz ─────────────────────────────────────────────────────────
    rviz_config = str(moveit_config.package_path / "config/moveit.rviz")

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # ── 10. pick-and-place 节点（延时启动） ──────────────────────────────
    # pick_and_place 可执行程序内部会等待 move_group 就绪才执行，
    # 因此这里不需要显式延时或事件处理器
    node_pick = Node(
        package="piper_gazebo",
        executable="pick_and_place",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                # ── 感知数据接口（Docker 内推理脚本写入的共享文件） ──
                "perception_targets_path": os.path.join(shared_dir, "perception_targets.csv"),
                "perception_grasps_path": os.path.join(shared_dir, "perception_grasps.csv"),
                "shared_dir": shared_dir,   # pick_state.json 读写位置
                # ── 抓取/放置执行参数 ──
                "max_place_attempts": 3,      # 放置失败最多重试次数
                "lift_z": 0.35,               # 抓起后抬升高度（m）
                "release_z": 0.18,            # 放置时释放高度（m）
                "max_grasp_distance": 0.08,   # 门控：TCP 到物体表面距离阈值（m），
                                              # 超过则拒绝 attach，防止隔空抓取
                "open_gripper_lift_height": 0.07,  # 张开夹爪后的抬升高度
                "graspnet_6d_approach_clearance": 0.0,  # 接近阶段安全余量
                "min_grasp_z": 0.12,          # 抓取点最低 Z（m），低于此高度
                                              # 的候选视为穿桌/穿平台，直接丢弃
                "back_place_x": 0.40,         # 放置区中心 X（m）——与
                "back_place_y": -0.20,        # 放置区中心 Y（m）——与
                                              # verify_pipeline.py 的
                                              # DROP_TARGET_XY 必须一致
                # ── 规划策略 ──
                "enable_position_only_fallback": False,  # 禁用"仅位置"回退：
                                                         # 强制使用 GraspNet
                                                         # 完整 6D 位姿
                "randomize_pick_order": True,  # 随机化抓取顺序（避免机械臂
                                               # 每次都走同一条路径互相干扰）
                # ── MoveIt 规划场景（碰撞感知） ──
                "enable_moveit_scene_objects": True,   # 把物体注册进规划场景
                "moveit_scene_transfer_fallback": True,    # 场景同步失败时回退
                "moveit_scene_readd_after_detach": True,   # 释放物体后重加回场景
                # ── GraspNet 6D 候选处理 ──
                "graspnet_6d_max_candidates": 300,   # 每物体最多加载的候选数
                                                     # （与推理端 candidates_per_object 对应）
                "graspnet_6d_max_orientation_variants": 48,  # 候选姿态方向变体数
                "graspnet_6d_analytic_only_precheck": True,  # 优先用 ssik 解析
                                                             # IK 预检可解性
            },
        ],
    )

    # ── 组装 LaunchDescription ──────────────────────────────────────────
    ld = LaunchDescription()

    # 全局参数
    ld.add_action(set_sim_time)
    ld.add_action(cleanup_perception)

    # 环境变量（必须在 Gazebo 启动前设置）
    ld.add_action(gazebo_plugin_path)
    ld.add_action(gazebo_lib_path)

    # Gazebo + 机器人
    ld.add_action(start_gazebo)
    ld.add_action(node_rsp)
    ld.add_action(spawn_entity)
    ld.add_action(on_spawn)
    ld.add_action(on_broadcaster)

    # MoveIt + RViz + 自动抓取
    ld.add_action(node_move_group)
    ld.add_action(yolo_bridge_node)
    ld.add_action(node_rviz)
    ld.add_action(node_pick)

    return ld
