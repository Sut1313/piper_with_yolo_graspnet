import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_name = "piper_lightweight_sim"
    package_share = get_package_share_directory(package_name)
    start_rviz = LaunchConfiguration("start_rviz")
    camera_pitch_offset = LaunchConfiguration("camera_pitch_offset")

    moveit_config = (
        MoveItConfigsBuilder("piper_lightweight", package_name=package_name)
        .robot_description(
            file_path="urdf/piper_lightweight.urdf.xacro",
            mappings={"camera_pitch_offset": camera_pitch_offset},
        )
        .robot_description_semantic(file_path="config/piper_lightweight.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "allow_trajectory_execution": True,
                "moveit_manage_controllers": False,
                "trajectory_execution.allowed_start_tolerance": 0.05,
                # Gazebo may need longer than the ideal time-parameterized
                # trajectory under attached-object load. This changes only
                # the watchdog, not the commanded trajectory speed.
                "trajectory_execution.allowed_execution_duration_scaling": 3.0,
                "trajectory_execution.allowed_goal_duration_margin": 2.0,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="lightweight_moveit_rviz",
        output="screen",
        condition=IfCondition(start_rviz),
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("piper_moveit"), "config", "moveit.rviz"
            ),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("camera_pitch_offset", default_value="1.047197551"),
        move_group,
        rviz,
    ])
