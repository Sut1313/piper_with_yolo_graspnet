from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from piper_yolo.paths import default_shared_dir, tabletop_model_path


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("piper_yolo"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path", default_value=str(tabletop_model_path(package_share))
            ),
            DeclareLaunchArgument(
                "color_topic", default_value="/camera/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera/color/camera_info",
            ),
            DeclareLaunchArgument(
                "shared_dir", default_value=str(default_shared_dir(package_share))
            ),
            DeclareLaunchArgument(
                "graspnet_bridge_enabled", default_value="true"
            ),
            DeclareLaunchArgument("publish_object_cloud", default_value="false"),
            Node(
                package="piper_yolo",
                executable="real_yolo_pointcloud_node",
                name="real_yolo_pointcloud_node",
                output="screen",
                parameters=[
                    str(package_share / "config/real_yolo_pointcloud.yaml"),
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "color_topic": LaunchConfiguration("color_topic"),
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "shared_dir": LaunchConfiguration("shared_dir"),
                        "graspnet_bridge_enabled": ParameterValue(
                            LaunchConfiguration("graspnet_bridge_enabled"),
                            value_type=bool,
                        ),
                        "publish_object_cloud": ParameterValue(
                            LaunchConfiguration("publish_object_cloud"),
                            value_type=bool,
                        ),
                        "use_sim_time": False,
                    },
                ],
            ),
        ]
    )
