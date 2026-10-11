from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from piper_yolo.paths import default_model_path, default_shared_dir


def generate_launch_description():
    package_share = Path(get_package_share_directory("piper_yolo"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=str(default_model_path(package_share)),
            ),
            DeclareLaunchArgument(
                "shared_dir", default_value=str(default_shared_dir(package_share))
            ),
            Node(
                package="piper_yolo",
                executable="yolo_bridge_node",
                name="yolo_bridge_node",
                output="screen",
                parameters=[
                    str(package_share / "config/yolo_bridge.yaml"),
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "shared_dir": LaunchConfiguration("shared_dir"),
                    },
                ],
            ),
        ]
    )
