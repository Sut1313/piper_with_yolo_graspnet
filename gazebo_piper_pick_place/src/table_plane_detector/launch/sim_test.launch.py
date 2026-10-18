from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("table_plane_detector"))
    world = str(package_share / "worlds" / "table_plane_test.sdf")
    params = str(package_share / "config" / "simulation.yaml")
    rviz_config = str(package_share / "rviz" / "table_plane_test.rviz")
    gui = LaunchConfiguration("gui")
    start_rviz = LaunchConfiguration("start_rviz")
    run_evaluation = LaunchConfiguration("run_evaluation")

    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "3", world],
        condition=IfCondition(gui),
        output="screen",
    )
    gz_headless = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", "-v", "3", world],
        condition=UnlessCondition(gui),
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="table_camera_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/table_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/table_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output="screen",
    )

    # Gazebo camera coordinates are +X forward, +Y left, +Z up. Publish the
    # physical pose and the standard ROS optical-frame rotation separately.
    camera_pose_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="table_camera_pose_tf",
        arguments=[
            "--x", "0.0", "--y", "-0.60", "--z", "1.0",
            "--roll", "0.0", "--pitch", "1.047197551", "--yaw", "1.570796327",
            "--frame-id", "world", "--child-frame-id", "camera_gazebo_frame",
        ],
        output="screen",
    )
    optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="table_camera_optical_tf",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--roll", "-1.570796327", "--pitch", "0.0", "--yaw", "-1.570796327",
            "--frame-id", "camera_gazebo_frame",
            "--child-frame-id", "camera_depth_optical_frame",
        ],
        output="screen",
    )

    depth_to_cloud = Node(
        package="table_plane_detector",
        executable="depth_image_to_cloud_node",
        name="depth_image_to_cloud",
        parameters=[{"output_frame": "camera_depth_optical_frame"}],
        remappings=[
            ("~/depth", "/table_camera/depth_image"),
            ("~/camera_info", "/table_camera/camera_info"),
            ("~/points", "/table_camera/points_optical"),
        ],
        output="screen",
    )

    detector = Node(
        package="table_plane_detector",
        executable="table_plane_detector_node",
        name="table_plane_detector",
        parameters=[params],
        remappings=[("~/input_cloud", "/table_camera/points_optical")],
        output="screen",
    )

    evaluator = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="table_plane_detector",
                executable="evaluate_table_plane.py",
                name="table_plane_evaluator",
                parameters=[
                    {"use_sim_time": True},
                    {"expected_perpendicular_m": 1.0},
                    {"expected_optical_axis_m": 1.15470053838},
                    {"perpendicular_tolerance_m": 0.003},
                    {"optical_axis_tolerance_m": 0.005},
                    {"timeout_s": 20.0},
                ],
                output="screen",
            )
        ],
        condition=IfCondition(run_evaluation),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(start_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="Start the Gazebo graphical client.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="false",
                description="Start RViz with the included visualization config.",
            ),
            DeclareLaunchArgument(
                "run_evaluation",
                default_value="true",
                description="Run the numerical end-to-end acceptance check.",
            ),
            gz_gui,
            gz_headless,
            bridge,
            camera_pose_tf,
            optical_tf,
            depth_to_cloud,
            detector,
            evaluator,
            rviz,
        ]
    )
