# Piper GraspNet Project Notes

## Goal

Make the ROS2/Gazebo Piper pick-and-place project reliably recognize and grasp the three target cylinders/objects using GraspNet 6D output as a true pose source, not just a position source. The target workflow is:

1. Start Gazebo, MoveIt2, GraspNet perception, and the `pick_and_place` node.
2. Load GraspNet candidates from the perception JSON.
3. Convert each GraspNet grasp frame into the Piper `grasp_tcp` frame.
4. Plan and execute full 6D grasp poses without position-only fallback.
5. Successfully pick all three objects in sequence.

## Current Understanding

This workspace is a ROS2 Humble project under `gazebo_piper_pick_place`.

Important packages and files:

- `src/piper_gazebo/src/pick_and_place.cpp`: main pick workflow, MoveIt planning, execution, object loop.
- `src/piper_gazebo/src/graspnet_6d_grasp_planner.cpp`: GraspNet candidate loading and 6D pose planning.
- `src/piper_gazebo/src/grasp_frame_transform.cpp`: camera/world transform and GraspNet frame to Piper TCP frame mapping.
- `src/piper_gazebo/src/position_grasp_planner.cpp`: position-only planner, kept separate.
- `src/piper_gazebo/include/piper_gazebo/*.hpp`: shared types and planner interfaces.
- `src/piper_moveit/config/kinematics.yaml`: MoveIt kinematics solver configuration.
- `src/piper_description/urdf/piper_description_gazebo.xacro`: robot chain geometry.

The project has been split so that position-only grasping and pose+position grasping are separate paths. Fallback to position-only mode is intentionally disabled for the GraspNet 6D workflow.

## Key Problem

MoveIt KDL IK is failing on many GraspNet full-pose targets, especially for later objects after the first successful pick. The latest logs showed repeated failures like:

```text
Joint-target IK planning failed ... seeds=80, exact_ik_solutions=0
Unable to sample any valid states for goal tree
```

This means the bottleneck is not only OMPL path planning. For many candidate poses, KDL cannot produce an exact IK solution before planning even starts.

## Existing Fixes

- Disabled position-only fallback.
- Added multiple GraspNet-to-Piper TCP frame mappings and candidate orientation scanning.
- Preferred mapping is currently:

```text
graspnet_tcp_calib_18_preferred
R = [[0, -1, 0],
     [0,  0,-1],
     [1,  0, 0]]
```

- Added a secondary mapping that helped red object scan-only tests.
- Moved the robot to an object-specific home pose before each pick.
- Fixed grasp height calculation so the GraspNet candidate `z` does not accidentally get double-handled.
- Added a fast joint-target IK planning layer that calls MoveIt IK first, validates FK error, then plans to joint targets.

## IK Direction

Open-source search found a strong existing analytic IK option:

- `personalrobotics/ssik`
- It includes `ssik.prebuilt.agilex.piper_ik`
- It explicitly supports AgileX PiPER as a non-Pieper 6R arm.
- The baked chain is `base_link -> link6`, while this project plans to `base_link -> grasp_tcp`.

Therefore the integration must convert:

```text
target grasp_tcp pose -> target link6 pose -> ssik Piper IK -> joint target planning
```

The local environment now has `ssik` installed in the user Python environment. A FK/IK round-trip for the Piper prebuilt artifact has been verified with residual around `4e-10`.

## Implementation Plan

1. Add a small Python helper script that imports `ssik.prebuilt.agilex.piper_ik`.
2. Feed it a target `link6` transform and optional current joint seed.
3. Return candidate joint solutions as JSON.
4. Add a C++ helper module that:
   - converts target `grasp_tcp` pose to `link6` pose using the fixed TCP offset,
   - invokes the Python helper,
   - parses returned joint candidates,
   - sorts by distance from current joints,
   - hands each candidate to MoveIt as a joint-value target.
5. Keep MoveIt collision and trajectory planning in C++ so analytic IK only replaces the weak IK-sampling stage.
6. Build and run the grasp workflow.
7. If analytic IK produces solutions but planning still fails, the next problem is collision/path constraints or bad GraspNet orientation mapping, not IK availability.

## Constraints

- Do not re-enable position-only fallback unless explicitly requested.
- Do not use GraspNet only for object centers; its predicted grasp orientation must be preserved and tested.
- Do not silently accept approximate IK. FK validation must check position and orientation error.
- Avoid large unrelated refactors while testing IK.
- Treat generated GraspNet pose frame mapping as a calibration variable and keep logs explicit.

## Current Best Hypothesis

The best near-term fix is not to hand-derive a fragile closed-form solver inside this project. The best fix is to use the existing open-source `ssik` Piper analytic IK artifact, convert `grasp_tcp` targets to `link6`, and let MoveIt plan collision-aware paths to those exact analytic joint solutions.
