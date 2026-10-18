# Changelog

## Unreleased

### Added

- `real_yolo_pointcloud_node` for synchronized D435i color, aligned depth and
  camera intrinsics.
- A 16-class `tabletop_yolov8s_seg_v2.onnx` model and reproducibility metadata.
- Atomic shared-directory output for the GraspNet Docker inference process.
- Optional ROS `PointCloud2`, overlay image and JSON detection topics.
- `table_plane_detector`, an optional ROS 2 package for table-plane estimation
  and D435i/Gazebo validation.
- `scripts/check_repository.sh` for GitHub publication checks.

### Changed

- Invalid and out-of-range depth values are cleared before a frame is written
  to the GraspNet shared directory; the default accepted interval is
  `0.10-3.0 m`.
- Real-camera and simulation nodes use separate default YOLO models so their
  label sets cannot silently conflict.
- Runtime paths are resolved from ROS parameters, environment variables or the
  repository location instead of a developer-specific absolute path.
- Real-camera frames ignore stale simulation `pick_state.json` state.

### Safety / known limitations

- Real-camera perception and camera-frame GraspNet inference have been tested.
- Real-robot execution is not release-ready: a measured hand-eye transform,
  gripper-width filtering and hardware MoveIt validation are still required.
- `shared_camera_data/camera_to_world.json` is generated for simulation and
  must not be treated as a physical-camera calibration file.
