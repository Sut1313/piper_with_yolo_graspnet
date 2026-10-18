import json
import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from piper_yolo.paths import default_shared_dir, legacy_grasp_model_path


def _clear_stale_perception_files(context):
    """Reset only generated handshake/output files from an earlier run."""
    shared_dir = Path(LaunchConfiguration("shared_dir").perform(context))
    shared_dir.mkdir(parents=True, exist_ok=True)
    for filename in (
        ".ready",
        "instances.npz",
        "detections.json",
        "frame_manifest.json",
        "perception_targets.csv",
        "perception_grasps.csv",
        "pick_state.json",
    ):
        path = shared_dir / filename
        try:
            path.unlink()
        except FileNotFoundError:
            pass
    return []


def _write_graspnet_camera_config(context):
    """Publish the exact Gazebo optical-frame pose for GraspNet world output."""
    shared_dir = Path(LaunchConfiguration("shared_dir").perform(context))
    shared_dir.mkdir(parents=True, exist_ok=True)
    pitch = float(LaunchConfiguration("camera_pitch_offset").perform(context))
    sin_pitch = math.sin(pitch)
    cos_pitch = math.cos(pitch)

    # world_R_optical = world_R_camera * camera_R_optical.  The fixed vehicle
    # yaw pair cancels, leaving world_R_camera=Ry(pitch).  ROS optical axes are
    # x-right, y-down, z-forward.
    rotation = [
        [0.0, -sin_pitch, cos_pitch],
        [-1.0, 0.0, 0.0],
        [0.0, -cos_pitch, -sin_pitch],
    ]
    config = {
        "schema_version": 1,
        "frame": "camera_depth_optical_frame",
        "camera_pitch_offset": pitch,
        "camera_to_world_r": rotation,
        "camera_to_world_rotation_r": rotation,
        "camera_to_world_t": [0.211853207, 0.006445351, 1.209551228],
        "source_object_priors_world": {
            "cube": [0.54, 0.08, 0.2904858],
            "cuboid": [0.68, 0.19, 0.2884858],
            "cylinder": [0.56, 0.31, 0.2954858],
        },
        "source_prior_max_xy_dist": 0.20,
    }
    destination = shared_dir / "camera_to_world.json"
    temporary = destination.with_name(f".{destination.name}.tmp-{os.getpid()}")
    temporary.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    temporary.replace(destination)
    return []


def generate_launch_description():
    pkg = get_package_share_directory("piper_lightweight_sim")
    xacro_file = os.path.join(pkg, "urdf", "piper_lightweight.urdf.xacro")
    world_file = os.path.join(pkg, "worlds", "piper_lightweight.sdf")

    camera_pitch_offset = LaunchConfiguration("camera_pitch_offset")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    start_yolo = LaunchConfiguration("start_yolo")
    state_driven_perception = LaunchConfiguration("state_driven_perception")
    shared_dir = LaunchConfiguration("shared_dir")
    model_path = LaunchConfiguration("model_path")

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " camera_pitch_offset:=", camera_pitch_offset,
        ]),
        value_type=str,
    )

    # sdformat rewrites package://piper_lightweight_sim to model:// URLs.
    # Its package parent must therefore be on the Gazebo resource path.
    resource_path = os.pathsep.join(
        [os.path.dirname(pkg), pkg]
        + ([os.environ["GZ_SIM_RESOURCE_PATH"]] if os.environ.get("GZ_SIM_RESOURCE_PATH") else [])
    )
    gz_env = {"GZ_SIM_RESOURCE_PATH": resource_path}

    gz_gui = ExecuteProcess(
        cmd=[
            "gz", "sim", "-r", "-v", "3",
            "--physics-engine", "gz-physics-bullet-featherstone-plugin",
            world_file,
        ],
        additional_env=gz_env,
        condition=IfCondition(gazebo_gui),
        output="screen",
    )
    gz_headless = ExecuteProcess(
        cmd=[
            "gz", "sim", "-r", "-s", "-v", "3",
            "--physics-engine", "gz-physics-bullet-featherstone-plugin",
            world_file,
        ],
        additional_env=gz_env,
        condition=UnlessCondition(gazebo_gui),
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="d435i_aligned_bridge",
        arguments=[
            "/camera/depth/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        remappings=[
            ("/camera/depth/image", "/camera/image_raw"),
            ("/camera/depth/depth_image", "/camera/depth/image_raw"),
            ("/camera/depth/camera_info", "/camera/camera_info"),
        ],
        output="screen",
    )
    attachment_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="grasp_attachment_bridge",
        arguments=[
            "/grasp/cube/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/grasp/cube/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/grasp/cube/state@std_msgs/msg/String[gz.msgs.StringMsg",
            "/grasp/cuboid/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/grasp/cuboid/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/grasp/cuboid/state@std_msgs/msg/String[gz.msgs.StringMsg",
            "/grasp/cylinder/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/grasp/cylinder/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/grasp/cylinder/state@std_msgs/msg/String[gz.msgs.StringMsg",
        ],
        output="screen",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "piper_lightweight"],
        output="screen",
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
        )

    joint_state_broadcaster = spawner("joint_state_broadcaster")
    arm_controller = spawner("arm_controller")
    gripper_controller = spawner("gripper_controller")

    detach_objects = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=["gz", "topic", "-t", f"/grasp/{name}/detach",
                     "-m", "gz.msgs.Empty", "-p", "unused: true"],
                output="screen",
            )
            for name in ("cube", "cuboid", "cylinder")
        ],
    )

    yolo = Node(
        package="piper_yolo",
        executable="yolo_bridge_node",
        name="yolo_bridge_node",
        parameters=[{
            "shared_dir": shared_dir,
            "model_path": model_path,
            # The lightweight grasp task still uses its original three
            # geometric labels.  Keep it isolated from the standalone
            # 16-class tabletop model used by piper_yolo by default.
            "class_names": ["cube", "cuboid", "cylinder"],
            "color_topic": "/camera/image_raw",
            "depth_topic": "/camera/depth/image_raw",
            "camera_info_topic": "/camera/camera_info",
            # The arm is visible in the optional downward-looking view. The
            # fine-tuned model scores the three scene objects above 0.90 while
            # arm-link false positives stay below 0.85.
            "confidence_threshold": 0.85,
            "max_detections": 6,
            "factor_depth": 1000.0,
            "idle_without_pick_state": ParameterValue(
                state_driven_perception, value_type=bool
            ),
            "single_instance_per_label": True,
        }],
        condition=IfCondition(start_yolo),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("start_yolo", default_value="true"),
        DeclareLaunchArgument(
            "state_driven_perception",
            default_value="true",
            description="Infer only when the grasp executor requests a fresh frame.",
        ),
        DeclareLaunchArgument(
            "camera_pitch_offset",
            default_value="0.0",
            description="0 keeps the exact AG robot camera attitude; 1.047197551 pitches it down 60 degrees for the optional tabletop perception demo.",
        ),
        DeclareLaunchArgument(
            "shared_dir",
            default_value=str(default_shared_dir(pkg)),
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value=str(legacy_grasp_model_path(pkg)),
        ),
        OpaqueFunction(function=_clear_stale_perception_files),
        OpaqueFunction(function=_write_graspnet_camera_config),
        gz_gui,
        gz_headless,
        clock_bridge,
        camera_bridge,
        attachment_bridge,
        rsp,
        spawn,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=[detach_objects, TimerAction(period=4.0, actions=[joint_state_broadcaster])],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller, gripper_controller],
        )),
        yolo,
    ])
