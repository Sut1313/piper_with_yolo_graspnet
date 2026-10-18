import json

import cv2
import numpy as np
from sensor_msgs.msg import Image
from scipy.io import loadmat

from piper_yolo.real_yolo_pointcloud_node import (
    GraspnetSharedWriter,
    RealYoloPointCloudNode,
    depth_meters_to_uint16,
    depth_to_meters,
    project_mask_to_points,
)
from piper_yolo.yolov8_onnx import Detection


def test_uint16_depth_is_converted_to_metres():
    depth = np.array([[0, 1000], [1500, 2000]], dtype=np.uint16)
    converted = depth_to_meters(depth, "16UC1", 0.001)
    np.testing.assert_allclose(converted, [[0.0, 1.0], [1.5, 2.0]])


def test_depth_metres_are_encoded_for_graspnet_png():
    depth = np.array([[0.0, 1.25], [np.nan, 70.0]], dtype=np.float32)
    converted = depth_meters_to_uint16(depth, 1000.0)
    np.testing.assert_array_equal(converted, [[0, 1250], [0, 65535]])


def test_mask_projection_uses_color_camera_intrinsics():
    depth = np.full((3, 3), 2.0, dtype=np.float32)
    mask = np.zeros((3, 3), dtype=bool)
    mask[1, 2] = True
    intrinsic = np.array([[2.0, 0.0, 1.0], [0.0, 2.0, 1.0], [0.0, 0.0, 1.0]])
    points, rows, columns = project_mask_to_points(
        depth, mask, intrinsic, 0.1, 3.0, point_stride=1, maximum_points=100
    )
    np.testing.assert_allclose(points, [[1.0, 0.0, 2.0]])
    np.testing.assert_array_equal(rows, [1])
    np.testing.assert_array_equal(columns, [2])


def test_projection_filters_invalid_depth_and_limits_points():
    depth = np.array([[0.0, 1.0], [4.0, 2.0]], dtype=np.float32)
    mask = np.ones((2, 2), dtype=bool)
    intrinsic = np.eye(3)
    points, _, _ = project_mask_to_points(
        depth, mask, intrinsic, 0.1, 3.0, point_stride=1, maximum_points=1
    )
    assert points.shape == (1, 3)
    assert 0.1 <= points[0, 2] <= 3.0


def test_point_cloud_contains_object_identity_fields():
    source = Image()
    source.header.frame_id = "camera_color_optical_frame"
    cloud = RealYoloPointCloudNode._build_cloud_message(
        source,
        "camera_color_optical_frame",
        np.array([[1.0, 2.0, 3.0]], dtype=np.float32),
        np.array([[10, 20, 30]], dtype=np.uint8),
        np.array([11], dtype=np.uint16),
        np.array([2], dtype=np.uint16),
        np.array([0.9], dtype=np.float32),
    )
    assert cloud.width == 1
    assert cloud.point_step == 24
    assert len(cloud.data) == 24
    assert [field.name for field in cloud.fields] == [
        "x",
        "y",
        "z",
        "rgb",
        "class_id",
        "instance_id",
        "confidence",
    ]


def test_graspnet_writer_commits_complete_frame_and_honours_ready(tmp_path):
    writer = GraspnetSharedWriter(tmp_path, factor_depth=1000.0)
    color = np.array(
        [[[0, 0, 255], [0, 255, 0]], [[255, 0, 0], [255, 255, 255]]],
        dtype=np.uint8,
    )
    depth_m = np.array([[1.0, 1.5], [0.0, 2.0]], dtype=np.float32)
    mask = np.array([[True, True], [False, False]])
    detection = Detection(
        class_id=10,
        label="cup",
        score=0.9,
        box_xyxy=np.array([0.0, 0.0, 2.0, 1.0], dtype=np.float32),
        mask=mask,
    )
    record = {
        "instance_id": 0,
        "class_id": 10,
        "label": "cup",
        "confidence": 0.9,
        "point_count": 2,
    }
    color_message = Image()
    color_message.header.frame_id = "camera_color_optical_frame"
    color_message.header.stamp.sec = 12
    depth_message = Image()
    depth_message.header = color_message.header

    status = writer.submit(
        color,
        depth_m,
        np.array([[100.0, 0.0, 1.0], [0.0, 100.0, 1.0], [0.0, 0.0, 1.0]]),
        [detection],
        [record],
        color,
        color_message,
        depth_message,
    )

    assert status == "written"
    assert (tmp_path / ".ready").read_text(encoding="utf-8") == "1\n"
    assert cv2.imread(str(tmp_path / "depth.png"), cv2.IMREAD_UNCHANGED).dtype == np.uint16
    np.testing.assert_array_equal(
        cv2.imread(str(tmp_path / "depth.png"), cv2.IMREAD_UNCHANGED),
        [[1000, 1500], [0, 2000]],
    )
    np.testing.assert_allclose(loadmat(tmp_path / "meta.mat")["factor_depth"], [[1000.0]])
    with np.load(tmp_path / "instances.npz", allow_pickle=False) as archive:
        assert archive["labels"].tolist() == ["cup"]
        np.testing.assert_array_equal(archive["masks"][0], mask)
    manifest = json.loads((tmp_path / "frame_manifest.json").read_text(encoding="utf-8"))
    assert manifest["source"] == "real_yolo_pointcloud_node"
    assert manifest["detection_count"] == 1
    assert writer.submit(
        color,
        depth_m,
        np.eye(3),
        [detection],
        [record],
        color,
        color_message,
        depth_message,
    ) == "busy"


def test_graspnet_writer_zeros_depth_outside_configured_range(tmp_path):
    writer = GraspnetSharedWriter(
        tmp_path,
        factor_depth=1000.0,
        minimum_depth_m=0.10,
        maximum_depth_m=3.0,
        require_object_points=False,
    )
    message = Image()
    depth_m = np.array(
        [[0.05, 0.10, 3.0], [3.01, 65.535, np.nan]], dtype=np.float32
    )

    assert writer.submit(
        np.zeros((2, 3, 3), dtype=np.uint8),
        depth_m,
        np.eye(3),
        [],
        [],
        np.zeros((2, 3, 3), dtype=np.uint8),
        message,
        message,
    ) == "written"

    np.testing.assert_array_equal(
        cv2.imread(str(tmp_path / "depth.png"), cv2.IMREAD_UNCHANGED),
        [[0, 100, 3000], [0, 0, 0]],
    )
    manifest = json.loads((tmp_path / "frame_manifest.json").read_text(encoding="utf-8"))
    assert manifest["minimum_depth_m"] == 0.10
    assert manifest["maximum_depth_m"] == 3.0


def test_graspnet_writer_skips_empty_object_cloud(tmp_path):
    writer = GraspnetSharedWriter(tmp_path, require_object_points=True)
    message = Image()
    assert writer.submit(
        np.zeros((2, 2, 3), dtype=np.uint8),
        np.ones((2, 2), dtype=np.float32),
        np.eye(3),
        [],
        [],
        np.zeros((2, 2, 3), dtype=np.uint8),
        message,
        message,
    ) == "no_objects"
    assert not (tmp_path / ".ready").exists()
