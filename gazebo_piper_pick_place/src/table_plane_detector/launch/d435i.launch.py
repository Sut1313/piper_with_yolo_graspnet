from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("table_plane_detector"))
    input_cloud = LaunchConfiguration("input_cloud")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_cloud",
                default_value="/camera/camera/depth/color/points",
                description="PointCloud2 topic published by realsense2_camera.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(package_share / "config" / "d435i.yaml"),
                description="D435i detector parameter file.",
            ),
            Node(
                package="table_plane_detector",
                executable="table_plane_detector_node",
                name="table_plane_detector",
                parameters=[params_file],
                remappings=[("~/input_cloud", input_cloud)],
                output="screen",
            ),
        ]
    )
