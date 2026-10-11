# PiPER lightweight RGB-D simulation

This Gazebo Harmonic package loads only the components needed by the grasp
pipeline:

- the AG robot's left PiPER arm and parallel gripper;
- the AG robot's D435i depth/RGBD sensor;
- a worktable whose top is exactly level with the arm base;
- one cube, one cuboid and one cylinder.

No chassis mesh, wheels, lift mesh, right arm, lidar, color-only camera or IMU
is instantiated.

## Exact parameters

The PiPER macro and 23 required mesh files are snapshotted from
`ag_robot_description`. Only the mesh package URI was retargeted; joint
origins, axes, limits, dynamics, inertias and collision geometry are unchanged.
The authoritative source macro SHA-256 at snapshot time was
`922acce0964cba99f01049651e963a27080aa607eb474437d0740ae274e258e7`.

The retained transforms are:

| Transform | xyz (m) | rpy (rad) |
|---|---|---|
| `base_footprint -> base_frame_link` | `0 0 0.2184858` | `0 0 1.5707963` |
| `base_frame_link -> base_link` (left arm) | `0.187732 -0.249047 0.032000` | `0 0 -1.5707963` |
| `base_frame_link -> camera_link` | `0.006445351 -0.211853207 0.991065428` | `0 0 -1.570796327` |

Depth resolution and intrinsics are the D435i values from the full URDF:
`848x480`, `fx=fy=418.2646789550781`,
`cx=424.1576232910156`, `cy=238.23983764648438`.

The RGB image comes from the RGBD sensor itself, so it is pixel-aligned with
the depth image. ROS topics are:

- `/camera/image_raw`
- `/camera/depth/image_raw`
- `/camera/camera_info`
- `/camera/depth/points`

## Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch piper_lightweight_sim lightweight_sim.launch.py
```

The exact vehicle camera attitude is horizontal and cannot see this tabletop.
For the recognition-and-pick demo, start the scene with the explicit 60-degree
downward override:

```bash
ros2 launch piper_lightweight_sim lightweight_sim.launch.py \
  camera_pitch_offset:=1.047197551
```

The launch starts Gazebo, both controllers, the RGB-D bridge and the YOLO ONNX
node. It also writes `shared_camera_data/camera_to_world.json`, containing the
active optical-frame extrinsics and object priors. GraspNet reads this file so
its 6D candidates are expressed in the same world frame as the simulated arm.
Start GraspNet separately from the project root:

```bash
./graspnet-docker-camera.sh gui
```

Start the matching MoveIt configuration and task executor in a third terminal.
The camera argument must match the simulation:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch piper_lightweight_sim lightweight_pick.launch.py \
  camera_pitch_offset:=1.047197551
```

RViz is enabled by default for motion-planning visualization. Use
`start_rviz:=false` for unattended or resource-constrained runs.

With no `target` argument, the executor randomizes and loops over all three
objects. Every candidate must pass approach, grasp, lift and place planning
before motion begins. It then executes close/attach/transfer/detach/release and
retries failed objects for up to three rounds. Useful single-object modes are:

Before each perception request the arm first moves to the zero-joint,
camera-clear observation posture. After a fresh YOLO/GraspNet result arrives,
perception is disabled and the arm moves to the target-specific planning home.
This prevents the wrist and gripper from occluding the requested object or
being segmented as part of it.

```bash
# Plan-only candidate selection
ros2 launch piper_lightweight_sim lightweight_pick.launch.py \
  camera_pitch_offset:=1.047197551 target:=cube dry_run:=true start_rviz:=false

# Execute the full pick, transfer and place cycle
ros2 launch piper_lightweight_sim lightweight_pick.launch.py \
  camera_pitch_offset:=1.047197551 target:=cube dry_run:=false
```

Valid targets are `cube`, `cuboid`, and `cylinder`. The executor atomically
writes `pick_state.json`. YOLO and GraspNet run only while
`perception_enabled=true`, and only for `active_target`; preflight and robot
motion therefore do not consume inference resources or generate moving-arm
false positives. Each epoch-based state version emits at most one inference
frame, including when multiple executor processes are started against one
persistent simulation.

Candidate selection first removes near-duplicate poses, then checks at most 60
candidates with MoveIt's `/compute_ik` service. Only fast-IK survivors reach
the more expensive four-stage OMPL preflight, which checks at most 10 by
default. GraspNet collision checking is likewise capped after a geometric
object-bounds prefilter. These limits can be tuned with
`max_prefilter_candidates`, `max_candidates`, and
`collision_candidate_limit`.

## Upstream speed profile

The default user-visible cadence matches the effective runtime requests in
`real_piper_graspnet`: arm velocity scaling is `0.10`, acceleration scaling is
`1.0`, candidate planning is limited to `0.5 s`,
execution planning to `10.0 s`, simulated gripper commands take `1.0 s`, and
the YOLO RGB-D handoff is capped at `5 Hz`. Gazebo remains at real-time factor
`1.0`. The pick launch exposes these as `velocity_scale`,
`acceleration_scale`, `candidate_planning_time`, `execution_planning_time`,
and `gripper_motion_time`.

The detachable-joint plugins use Gazebo's converted `link6` as the parent link
because fixed URDF links `flange_link` and `gripper_base` are merged into it.

## Camera field-of-view constraint

The full vehicle camera is horizontal and approximately 0.959 m above the arm
base. Consequently, the strict pose (`camera_pitch_offset:=0.0`) cannot see a
tabletop at arm-base height inside PiPER's reachable workspace. This is a
geometric property of the source URDF, not a simulator setting.

For perception-only experiments an explicit 60-degree downward optical override
is available:

```bash
ros2 launch piper_lightweight_sim lightweight_sim.launch.py \
  camera_pitch_offset:=1.047197551
```

This override preserves the camera position and all intrinsics, but it is not
the exact vehicle mounting attitude. The default remains the strict source-URDF
pose.
