from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("table_plane_detector"))
    params_file = LaunchConfiguration("params_file")
    input_cloud = LaunchConfiguration("input_cloud")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(package_share / "config" / "default.yaml"),
                description="Detector parameter file.",
            ),
            DeclareLaunchArgument(
                "input_cloud",
                default_value="/camera/depth/points",
                description="Input sensor_msgs/msg/PointCloud2 topic.",
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
