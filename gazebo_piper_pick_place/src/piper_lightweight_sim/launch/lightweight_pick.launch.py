import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from piper_yolo.paths import default_shared_dir


def generate_launch_description():
    package_share = get_package_share_directory("piper_lightweight_sim")
    shared_dir = LaunchConfiguration("shared_dir")
    target = LaunchConfiguration("target")
    dry_run = LaunchConfiguration("dry_run")
    randomize = LaunchConfiguration("randomize_pick_order")
    start_rviz = LaunchConfiguration("start_rviz")
    camera_pitch_offset = LaunchConfiguration("camera_pitch_offset")
    max_candidates = LaunchConfiguration("max_candidates")
    max_prefilter_candidates = LaunchConfiguration("max_prefilter_candidates")
    max_rounds = LaunchConfiguration("max_rounds")
    candidate_planning_time = LaunchConfiguration("candidate_planning_time")
    fast_ik_timeout = LaunchConfiguration("fast_ik_timeout")
    execution_planning_time = LaunchConfiguration("execution_planning_time")
    velocity_scale = LaunchConfiguration("velocity_scale")
    acceleration_scale = LaunchConfiguration("acceleration_scale")
    gripper_motion_time = LaunchConfiguration("gripper_motion_time")

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "lightweight_moveit.launch.py")
        ),
        launch_arguments={
            "start_rviz": start_rviz,
            "camera_pitch_offset": camera_pitch_offset,
        }.items(),
    )

    executor = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="piper_lightweight_sim",
                executable="lightweight_grasp_executor",
                name="lightweight_grasp_executor",
                output="screen",
                parameters=[{
                    "shared_dir": ParameterValue(shared_dir, value_type=str),
                    "target": ParameterValue(target, value_type=str),
                    "dry_run": ParameterValue(dry_run, value_type=bool),
                    "randomize_pick_order": ParameterValue(randomize, value_type=bool),
                    "max_candidates": ParameterValue(max_candidates, value_type=int),
                    "max_prefilter_candidates": ParameterValue(
                        max_prefilter_candidates, value_type=int
                    ),
                    "max_rounds": ParameterValue(max_rounds, value_type=int),
                    "candidate_planning_time": ParameterValue(
                        candidate_planning_time, value_type=float
                    ),
                    "fast_ik_timeout": ParameterValue(fast_ik_timeout, value_type=float),
                    "execution_planning_time": ParameterValue(
                        execution_planning_time, value_type=float
                    ),
                    "velocity_scale": ParameterValue(velocity_scale, value_type=float),
                    "acceleration_scale": ParameterValue(
                        acceleration_scale, value_type=float
                    ),
                    "gripper_motion_time": ParameterValue(
                        gripper_motion_time, value_type=float
                    ),
                    "use_sim_time": True,
                }],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "shared_dir",
            default_value=str(default_shared_dir(package_share)),
        ),
        DeclareLaunchArgument(
            "target",
            default_value="",
            description="Empty runs the full cube/cuboid/cylinder loop; otherwise run one class.",
        ),
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("randomize_pick_order", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("camera_pitch_offset", default_value="1.047197551"),
        DeclareLaunchArgument("max_candidates", default_value="10"),
        DeclareLaunchArgument("max_prefilter_candidates", default_value="60"),
        DeclareLaunchArgument("max_rounds", default_value="3"),
        # Keep the user-visible execution cadence aligned with the upstream
        # real_piper_graspnet defaults.
        DeclareLaunchArgument("candidate_planning_time", default_value="0.5"),
        DeclareLaunchArgument("fast_ik_timeout", default_value="0.08"),
        DeclareLaunchArgument("execution_planning_time", default_value="10.0"),
        DeclareLaunchArgument("velocity_scale", default_value="0.10"),
        # The upstream Humble moveit_adapter writes 1.0 into each runtime
        # MotionPlanRequest, overriding its MoveIt YAML default.
        DeclareLaunchArgument("acceleration_scale", default_value="1.0"),
        DeclareLaunchArgument("gripper_motion_time", default_value="1.0"),
        moveit,
        executor,
    ])
