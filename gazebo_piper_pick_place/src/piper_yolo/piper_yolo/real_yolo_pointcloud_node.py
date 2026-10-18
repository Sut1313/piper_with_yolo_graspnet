"""Real-camera YOLOv8-seg node with ROS and GraspNet file outputs.

The node has no Gazebo, pick-state, or robot-motion dependency.  It expects
depth already aligned to the color image, as provided by the RealSense ROS
wrapper with ``align_depth`` enabled.  Every inference is published as ROS
messages; when the GraspNet bridge is enabled, the latest usable frame is also
written atomically to the shared-directory ``.ready`` handshake protocol.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from scipy.io import savemat
from std_msgs.msg import String

from .paths import default_shared_dir, tabletop_model_path
from .yolov8_onnx import Detection, Yolov8SegOnnx


TABLETOP_CLASSES = [
    "pen",
    "pencil",
    "marker",
    "eraser",
    "ruler",
    "scissors",
    "stapler",
    "tape",
    "sharpener",
    "block",
    "cup",
    "bottle",
    "book",
    "phone",
    "mouse",
    "remote",
]


def depth_to_meters(depth: np.ndarray, encoding: str, uint16_scale: float) -> np.ndarray:
    """Convert a ROS depth image to finite float32 metres."""
    normalized_encoding = encoding.upper()
    if normalized_encoding in {"16UC1", "MONO16"} or depth.dtype == np.uint16:
        result = depth.astype(np.float32) * float(uint16_scale)
    elif normalized_encoding == "32FC1" or np.issubdtype(depth.dtype, np.floating):
        result = depth.astype(np.float32, copy=False)
    else:
        raise ValueError(f"不支持的深度编码: {encoding}, dtype={depth.dtype}")
    result = np.nan_to_num(result, nan=0.0, posinf=0.0, neginf=0.0)
    result[result < 0.0] = 0.0
    return result


def project_mask_to_points(
    depth_m: np.ndarray,
    mask: np.ndarray,
    intrinsic: np.ndarray,
    minimum_depth_m: float,
    maximum_depth_m: float,
    point_stride: int,
    maximum_points: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Project one image mask to optical-frame XYZ points and pixel indices."""
    if depth_m.shape != mask.shape:
        raise ValueError(f"深度与掩膜尺寸不一致: {depth_m.shape} != {mask.shape}")
    if intrinsic.shape != (3, 3):
        raise ValueError(f"相机内参必须是 3x3，实际为 {intrinsic.shape}")

    stride = max(1, int(point_stride))
    sampled_mask = np.zeros_like(mask, dtype=bool)
    sampled_mask[::stride, ::stride] = mask[::stride, ::stride]
    valid = (
        sampled_mask
        & np.isfinite(depth_m)
        & (depth_m >= float(minimum_depth_m))
        & (depth_m <= float(maximum_depth_m))
    )
    rows, columns = np.nonzero(valid)
    if rows.size == 0:
        return np.empty((0, 3), dtype=np.float32), rows, columns

    limit = int(maximum_points)
    if limit > 0 and rows.size > limit:
        selected = np.linspace(0, rows.size - 1, limit, dtype=np.int64)
        rows = rows[selected]
        columns = columns[selected]

    fx = float(intrinsic[0, 0])
    fy = float(intrinsic[1, 1])
    cx = float(intrinsic[0, 2])
    cy = float(intrinsic[1, 2])
    if not np.isfinite([fx, fy, cx, cy]).all() or fx <= 0.0 or fy <= 0.0:
        raise ValueError(f"无效相机内参: fx={fx}, fy={fy}, cx={cx}, cy={cy}")

    z = depth_m[rows, columns]
    x = (columns.astype(np.float32) - cx) * z / fx
    y = (rows.astype(np.float32) - cy) * z / fy
    return np.column_stack((x, y, z)).astype(np.float32, copy=False), rows, columns


def depth_meters_to_uint16(depth_m: np.ndarray, factor_depth: float) -> np.ndarray:
    """Encode metre-valued depth for GraspNet's uint16 PNG + factor format."""
    factor = float(factor_depth)
    if not np.isfinite(factor) or factor <= 0.0:
        raise ValueError(f"无效 GraspNet depth factor: {factor_depth}")
    finite = np.nan_to_num(
        depth_m.astype(np.float32, copy=False), nan=0.0, posinf=0.0, neginf=0.0
    )
    finite[finite < 0.0] = 0.0
    return np.clip(
        np.rint(finite * factor), 0, np.iinfo(np.uint16).max
    ).astype(np.uint16)


class GraspnetSharedWriter:
    """Atomically submit real RGB-D detections to the Docker GraspNet loop."""

    def __init__(
        self,
        shared_dir: str | Path,
        factor_depth: float = 1000.0,
        minimum_depth_m: float = 0.10,
        maximum_depth_m: float = 3.0,
        write_overlay: bool = True,
        require_object_points: bool = True,
    ) -> None:
        self.shared_dir = Path(shared_dir).expanduser().resolve()
        self.shared_dir.mkdir(parents=True, exist_ok=True)
        self.factor_depth = float(factor_depth)
        if not np.isfinite(self.factor_depth) or self.factor_depth <= 0.0:
            raise ValueError(f"无效 GraspNet depth factor: {factor_depth}")
        self.minimum_depth_m = float(minimum_depth_m)
        self.maximum_depth_m = float(maximum_depth_m)
        if (
            not np.isfinite([self.minimum_depth_m, self.maximum_depth_m]).all()
            or self.minimum_depth_m < 0.0
            or self.maximum_depth_m <= self.minimum_depth_m
        ):
            raise ValueError(
                "无效 GraspNet 深度范围: "
                f"[{minimum_depth_m}, {maximum_depth_m}] m"
            )
        self.write_overlay = bool(write_overlay)
        self.require_object_points = bool(require_object_points)
        self.frame_sequence = self._existing_sequence()

    @property
    def ready_path(self) -> Path:
        return self.shared_dir / ".ready"

    def _existing_sequence(self) -> int:
        try:
            value = json.loads(
                (self.shared_dir / "frame_manifest.json").read_text(encoding="utf-8")
            ).get("frame_sequence", 0)
            return max(0, int(value))
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return 0

    def submit(
        self,
        color: np.ndarray,
        depth_m: np.ndarray,
        intrinsic: np.ndarray,
        detections: list[Detection],
        records: list[dict[str, Any]],
        overlay: np.ndarray,
        color_message: Image,
        depth_message: Image,
    ) -> str:
        """Write one complete frame, returning written, busy, or no_objects."""
        if self.ready_path.exists():
            return "busy"

        usable = [
            (detection, record)
            for detection, record in zip(detections, records)
            if int(record.get("point_count", 0)) > 0
        ]
        if self.require_object_points and not usable:
            return "no_objects"
        if self.require_object_points:
            detections = [item[0] for item in usable]
            records = [item[1] for item in usable]

        sequence = self.frame_sequence + 1
        masks = (
            np.stack([item.mask.astype(np.uint8) for item in detections])
            if detections
            else np.empty((0, color.shape[0], color.shape[1]), dtype=np.uint8)
        )
        labels = np.asarray([item.label for item in detections], dtype=np.str_)
        class_ids = np.asarray([item.class_id for item in detections], dtype=np.int64)
        scores = np.asarray([item.score for item in detections], dtype=np.float32)
        boxes = np.asarray(
            [item.box_xyxy for item in detections], dtype=np.float32
        ).reshape(-1, 4)
        # RealSense may use 65535 in a 16UC1 frame for saturated/invalid
        # measurements.  If written unchanged, GraspNet interprets it as
        # 65.535 m and the outliers corrupt scene collision checking and the
        # Open3D auto-fit view.  Store every non-finite or out-of-workspace
        # sample as zero, which is GraspNet's invalid-depth convention.
        valid_depth = (
            np.isfinite(depth_m)
            & (depth_m >= self.minimum_depth_m)
            & (depth_m <= self.maximum_depth_m)
        )
        filtered_depth_m = np.where(valid_depth, depth_m, 0.0)
        depth_units = depth_meters_to_uint16(filtered_depth_m, self.factor_depth)

        self._atomic_write_image(self.shared_dir / "color.png", color)
        self._atomic_write_image(self.shared_dir / "depth.png", depth_units)
        self._atomic_write_mat(
            self.shared_dir / "meta.mat",
            {
                "intrinsic_matrix": intrinsic.astype(np.float64, copy=False),
                "factor_depth": np.asarray([[self.factor_depth]], dtype=np.float64),
            },
        )
        self._atomic_write_npz(
            self.shared_dir / "instances.npz",
            masks=masks,
            class_ids=class_ids,
            scores=scores,
            boxes_xyxy=boxes,
            labels=labels,
            frame_sequence=np.asarray([sequence], dtype=np.int64),
        )
        self._atomic_write_json(
            self.shared_dir / "detections.json",
            {
                "frame_sequence": sequence,
                "detections": [
                    {
                        "instance_id": int(record["instance_id"]),
                        "class_id": int(detection.class_id),
                        "label": detection.label,
                        "score": float(detection.score),
                        "box_xyxy": [float(value) for value in detection.box_xyxy],
                        "mask_pixels": int(np.count_nonzero(detection.mask)),
                        "point_count": int(record["point_count"]),
                    }
                    for detection, record in zip(detections, records)
                ],
            },
        )
        if self.write_overlay:
            self._atomic_write_image(self.shared_dir / "yolo_overlay.png", overlay)
        self._atomic_write_json(
            self.shared_dir / "frame_manifest.json",
            {
                "frame_sequence": sequence,
                "source": "real_yolo_pointcloud_node",
                "color_stamp_ns": RealYoloPointCloudNode._stamp_ns(color_message),
                "depth_stamp_ns": RealYoloPointCloudNode._stamp_ns(depth_message),
                "frame_id": color_message.header.frame_id,
                "image_shape": [int(color.shape[0]), int(color.shape[1])],
                "detection_count": len(detections),
                "depth_encoding": "16UC1",
                "factor_depth": self.factor_depth,
                "minimum_depth_m": self.minimum_depth_m,
                "maximum_depth_m": self.maximum_depth_m,
            },
        )
        # This flag must always be last: its appearance is GraspNet's commit signal.
        self._atomic_write_text(self.ready_path, f"{sequence}\n")
        self.frame_sequence = sequence
        return "written"

    def _temporary_path(self, path: Path) -> Path:
        return path.with_name(f".{path.name}.tmp-{os.getpid()}-{time.time_ns()}")

    def _atomic_write_image(self, path: Path, image: np.ndarray) -> None:
        temporary = self._temporary_path(path)
        extension = path.suffix or ".png"
        success, encoded = cv2.imencode(extension, image)
        if not success:
            raise OSError(f"图像编码失败: {path}")
        temporary.write_bytes(encoded.tobytes())
        os.replace(temporary, path)

    def _atomic_write_mat(self, path: Path, values: dict[str, Any]) -> None:
        temporary = self._temporary_path(path)
        savemat(str(temporary), values, appendmat=False)
        os.replace(temporary, path)

    def _atomic_write_npz(self, path: Path, **values: np.ndarray) -> None:
        temporary = self._temporary_path(path)
        with temporary.open("wb") as handle:
            np.savez_compressed(handle, **values)
        os.replace(temporary, path)

    def _atomic_write_json(self, path: Path, value: Any) -> None:
        self._atomic_write_text(
            path, json.dumps(value, ensure_ascii=False, indent=2) + "\n"
        )

    def _atomic_write_text(self, path: Path, value: str) -> None:
        temporary = self._temporary_path(path)
        temporary.write_text(value, encoding="utf-8")
        os.replace(temporary, path)


class RealYoloPointCloudNode(Node):
    """Continuously recognize tabletop objects in aligned real RGB-D frames."""

    def __init__(self) -> None:
        super().__init__("real_yolo_pointcloud_node")
        self.declare_parameter("model_path", str(tabletop_model_path(__file__)))
        self.declare_parameter("class_names", TABLETOP_CLASSES)
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("cloud_topic", "/yolo/object_cloud")
        self.declare_parameter("publish_object_cloud", False)
        self.declare_parameter("overlay_topic", "/yolo/overlay")
        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.50)
        self.declare_parameter("max_detections", 20)
        self.declare_parameter("prefer_cuda", True)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("uint16_depth_scale", 0.001)
        self.declare_parameter("minimum_depth_m", 0.10)
        self.declare_parameter("maximum_depth_m", 3.0)
        self.declare_parameter("mask_erode_pixels", 1)
        self.declare_parameter("point_stride", 1)
        self.declare_parameter("max_points_per_instance", 20000)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("minimum_publish_interval", 0.10)
        self.declare_parameter("require_aligned_depth_frame", True)
        self.declare_parameter("graspnet_bridge_enabled", True)
        self.declare_parameter("shared_dir", str(default_shared_dir(__file__)))
        self.declare_parameter("graspnet_depth_factor", 1000.0)
        self.declare_parameter("write_shared_overlay", True)
        self.declare_parameter("graspnet_require_object_points", True)

        class_names = [str(value) for value in self.get_parameter("class_names").value]
        self.detector = Yolov8SegOnnx(
            model_path=str(self.get_parameter("model_path").value),
            class_names=class_names,
            confidence_threshold=float(self.get_parameter("confidence_threshold").value),
            iou_threshold=float(self.get_parameter("iou_threshold").value),
            max_detections=int(self.get_parameter("max_detections").value),
            prefer_cuda=bool(self.get_parameter("prefer_cuda").value),
            image_size=int(self.get_parameter("image_size").value),
        )
        self.bridge = CvBridge()
        self.camera_info: CameraInfo | None = None
        self.last_publish_time = 0.0
        self.last_warning_time = 0.0
        self.last_shared_wait_log_time = 0.0
        self.shared_writer: GraspnetSharedWriter | None = None
        if bool(self.get_parameter("graspnet_bridge_enabled").value):
            self.shared_writer = GraspnetSharedWriter(
                shared_dir=str(self.get_parameter("shared_dir").value),
                factor_depth=float(self.get_parameter("graspnet_depth_factor").value),
                minimum_depth_m=float(self.get_parameter("minimum_depth_m").value),
                maximum_depth_m=float(self.get_parameter("maximum_depth_m").value),
                write_overlay=bool(self.get_parameter("write_shared_overlay").value),
                require_object_points=bool(
                    self.get_parameter("graspnet_require_object_points").value
                ),
            )
            self._warn_if_pick_state_blocks_inference()

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cloud_publisher = None
        if bool(self.get_parameter("publish_object_cloud").value):
            self.cloud_publisher = self.create_publisher(
                PointCloud2, str(self.get_parameter("cloud_topic").value), output_qos
            )
        self.overlay_publisher = self.create_publisher(
            Image, str(self.get_parameter("overlay_topic").value), output_qos
        )
        self.detections_publisher = self.create_publisher(
            String, str(self.get_parameter("detections_topic").value), 10
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback,
            sensor_qos,
        )
        color_subscriber = message_filters.Subscriber(
            self,
            Image,
            str(self.get_parameter("color_topic").value),
            qos_profile=sensor_qos,
        )
        depth_subscriber = message_filters.Subscriber(
            self,
            Image,
            str(self.get_parameter("depth_topic").value),
            qos_profile=sensor_qos,
        )
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            [color_subscriber, depth_subscriber],
            queue_size=int(self.get_parameter("sync_queue_size").value),
            slop=float(self.get_parameter("sync_slop").value),
        )
        synchronizer.registerCallback(self._rgbd_callback)
        self._color_subscriber = color_subscriber
        self._depth_subscriber = depth_subscriber
        self._synchronizer = synchronizer

        self.get_logger().info(
            "真机 YOLO 点云节点已启动："
            f"providers={self.detector.providers}, classes={len(class_names)}, "
            f"cloud="
            f"{self.get_parameter('cloud_topic').value if self.cloud_publisher else 'disabled'}, "
            f"graspnet_shared_dir="
            f"{self.shared_writer.shared_dir if self.shared_writer else 'disabled'}"
        )

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self.camera_info = message

    def _warn_if_pick_state_blocks_inference(self) -> None:
        if self.shared_writer is None:
            return
        state_path = self.shared_writer.shared_dir / "pick_state.json"
        try:
            state = json.loads(state_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return
        if state.get("perception_enabled") is False:
            self.get_logger().warning(
                "共享目录中的 pick_state.json 禁用了感知；进行真机感知-only "
                f"测试前请移走该文件: {state_path}"
            )

    def _warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_warning_time >= 5.0:
            self.get_logger().warning(message)
            self.last_warning_time = now

    def _rgbd_callback(self, color_message: Image, depth_message: Image) -> None:
        now = time.monotonic()
        if now - self.last_publish_time < float(
            self.get_parameter("minimum_publish_interval").value
        ):
            return
        if self.camera_info is None:
            self._warn_throttled("等待彩色相机 CameraInfo")
            return
        if bool(self.get_parameter("require_aligned_depth_frame").value):
            color_frame = self.camera_info.header.frame_id or color_message.header.frame_id
            if (
                color_frame
                and depth_message.header.frame_id
                and color_frame != depth_message.header.frame_id
            ):
                self._warn_throttled(
                    "深度图未对齐到彩色光学帧："
                    f"color={color_frame}, depth={depth_message.header.frame_id}"
                )
                return

        try:
            color = self.bridge.imgmsg_to_cv2(color_message, desired_encoding="bgr8")
            raw_depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
            depth_m = depth_to_meters(
                raw_depth,
                depth_message.encoding,
                float(self.get_parameter("uint16_depth_scale").value),
            )
            if color.shape[:2] != depth_m.shape:
                raise ValueError(
                    f"彩色图与对齐深度图尺寸不一致: {color.shape[:2]} != {depth_m.shape}"
                )

            intrinsic = np.asarray(self.camera_info.k, dtype=np.float64).reshape(3, 3)
            detections = self._erode_masks(self.detector.infer(color))
            cloud_message, records = self._make_object_cloud(
                color,
                depth_m,
                intrinsic,
                detections,
                color_message,
                build_cloud=self.cloud_publisher is not None,
            )
            if self.cloud_publisher is not None and cloud_message is not None:
                self.cloud_publisher.publish(cloud_message)

            overlay = self._draw_overlay(color, detections, records)
            overlay_message = self.bridge.cv2_to_imgmsg(
                overlay, encoding="bgr8"
            )
            overlay_message.header = color_message.header
            self.overlay_publisher.publish(overlay_message)

            detection_message = String()
            detection_message.data = json.dumps(
                {
                    "stamp_ns": self._stamp_ns(color_message),
                    "frame_id": (
                        cloud_message.header.frame_id
                        if cloud_message is not None
                        else self.camera_info.header.frame_id
                        or color_message.header.frame_id
                    ),
                    "detections": records,
                },
                ensure_ascii=False,
            )
            self.detections_publisher.publish(detection_message)
            self._submit_graspnet_frame(
                color,
                depth_m,
                intrinsic,
                detections,
                records,
                overlay,
                color_message,
                depth_message,
            )
            self.last_publish_time = now
        except Exception as exc:
            self.get_logger().error(f"处理真机 RGB-D 帧失败: {exc}")

    def _submit_graspnet_frame(
        self,
        color: np.ndarray,
        depth_m: np.ndarray,
        intrinsic: np.ndarray,
        detections: list[Detection],
        records: list[dict[str, Any]],
        overlay: np.ndarray,
        color_message: Image,
        depth_message: Image,
    ) -> None:
        if self.shared_writer is None:
            return
        try:
            status = self.shared_writer.submit(
                color,
                depth_m,
                intrinsic,
                detections,
                records,
                overlay,
                color_message,
                depth_message,
            )
        except Exception as exc:
            self._warn_throttled(f"写入 GraspNet 共享目录失败: {exc}")
            return
        if status == "written":
            labels = ", ".join(item.label for item in detections) or "无目标"
            self.get_logger().info(
                f"已向 GraspNet 提交共享帧 {self.shared_writer.frame_sequence}: {labels}"
            )
        elif status == "busy":
            now = time.monotonic()
            if now - self.last_shared_wait_log_time >= 5.0:
                self.get_logger().info("GraspNet 正在处理上一帧，ROS 输出继续发布")
                self.last_shared_wait_log_time = now

    def _erode_masks(self, detections: list[Detection]) -> list[Detection]:
        pixels = int(self.get_parameter("mask_erode_pixels").value)
        if pixels <= 0:
            return detections
        kernel = np.ones((pixels * 2 + 1, pixels * 2 + 1), dtype=np.uint8)
        return [
            Detection(
                class_id=detection.class_id,
                label=detection.label,
                score=detection.score,
                box_xyxy=detection.box_xyxy,
                mask=cv2.erode(detection.mask.astype(np.uint8), kernel, iterations=1) > 0,
            )
            for detection in detections
        ]

    def _make_object_cloud(
        self,
        color: np.ndarray,
        depth_m: np.ndarray,
        intrinsic: np.ndarray,
        detections: list[Detection],
        color_message: Image,
        build_cloud: bool = True,
    ) -> tuple[PointCloud2 | None, list[dict[str, Any]]]:
        point_batches: list[np.ndarray] = []
        color_batches: list[np.ndarray] = []
        class_batches: list[np.ndarray] = []
        instance_batches: list[np.ndarray] = []
        confidence_batches: list[np.ndarray] = []
        records: list[dict[str, Any]] = []

        for instance_id, detection in enumerate(detections):
            points, rows, columns = project_mask_to_points(
                depth_m,
                detection.mask,
                intrinsic,
                float(self.get_parameter("minimum_depth_m").value),
                float(self.get_parameter("maximum_depth_m").value),
                int(self.get_parameter("point_stride").value),
                int(self.get_parameter("max_points_per_instance").value),
            )
            point_count = int(points.shape[0])
            center = np.median(points, axis=0).tolist() if point_count else None
            records.append(
                {
                    "instance_id": instance_id,
                    "class_id": detection.class_id,
                    "label": detection.label,
                    "confidence": detection.score,
                    "box_xyxy": [float(value) for value in detection.box_xyxy],
                    "mask_pixels": int(np.count_nonzero(detection.mask)),
                    "point_count": point_count,
                    "centroid_m": center,
                }
            )
            if point_count == 0 or not build_cloud:
                continue
            point_batches.append(points)
            color_batches.append(color[rows, columns])
            class_batches.append(np.full(point_count, detection.class_id, dtype=np.uint16))
            instance_batches.append(np.full(point_count, instance_id, dtype=np.uint16))
            confidence_batches.append(
                np.full(point_count, detection.score, dtype=np.float32)
            )

        frame_id = self.camera_info.header.frame_id if self.camera_info else ""
        frame_id = frame_id or color_message.header.frame_id
        if not build_cloud:
            return None, records
        if not point_batches:
            return self._build_cloud_message(
                color_message, frame_id, np.empty((0, 3), np.float32),
                np.empty((0, 3), np.uint8), np.empty(0, np.uint16),
                np.empty(0, np.uint16), np.empty(0, np.float32)
            ), records

        return self._build_cloud_message(
            color_message,
            frame_id,
            np.concatenate(point_batches),
            np.concatenate(color_batches),
            np.concatenate(class_batches),
            np.concatenate(instance_batches),
            np.concatenate(confidence_batches),
        ), records

    @staticmethod
    def _build_cloud_message(
        source_message: Image,
        frame_id: str,
        points: np.ndarray,
        colors_bgr: np.ndarray,
        class_ids: np.ndarray,
        instance_ids: np.ndarray,
        confidences: np.ndarray,
    ) -> PointCloud2:
        dtype = np.dtype(
            [
                ("x", "<f4"),
                ("y", "<f4"),
                ("z", "<f4"),
                ("rgb", "<f4"),
                ("class_id", "<u2"),
                ("instance_id", "<u2"),
                ("confidence", "<f4"),
            ]
        )
        packed = np.empty(points.shape[0], dtype=dtype)
        packed["x"] = points[:, 0]
        packed["y"] = points[:, 1]
        packed["z"] = points[:, 2]
        if points.shape[0]:
            rgb_uint = (
                (colors_bgr[:, 2].astype(np.uint32) << 16)
                | (colors_bgr[:, 1].astype(np.uint32) << 8)
                | colors_bgr[:, 0].astype(np.uint32)
            )
            packed["rgb"] = rgb_uint.view(np.float32)
        packed["class_id"] = class_ids
        packed["instance_id"] = instance_ids
        packed["confidence"] = confidences

        message = PointCloud2()
        message.header = source_message.header
        message.header.frame_id = frame_id
        message.height = 1
        message.width = int(points.shape[0])
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="class_id", offset=16, datatype=PointField.UINT16, count=1),
            PointField(name="instance_id", offset=18, datatype=PointField.UINT16, count=1),
            PointField(name="confidence", offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = dtype.itemsize
        message.row_step = message.point_step * message.width
        message.data = packed.tobytes()
        message.is_dense = True
        return message

    @staticmethod
    def _draw_overlay(
        color: np.ndarray,
        detections: list[Detection],
        records: list[dict[str, Any]],
    ) -> np.ndarray:
        overlay = color.copy()
        for detection, record in zip(detections, records):
            hue = (detection.class_id * 47) % 180
            hsv = np.uint8([[[hue, 210, 240]]])
            draw_color = tuple(
                int(value) for value in cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
            )
            tint = np.zeros_like(overlay)
            tint[detection.mask] = draw_color
            overlay = cv2.addWeighted(overlay, 1.0, tint, 0.35, 0.0)
            x1, y1, x2, y2 = np.rint(detection.box_xyxy).astype(int)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), draw_color, 2)
            text = (
                f"{detection.label} {detection.score:.2f} "
                f"{record['point_count']}pts"
            )
            cv2.putText(
                overlay,
                text,
                (x1, max(20, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                draw_color,
                2,
                cv2.LINE_AA,
            )
        return overlay

    @staticmethod
    def _stamp_ns(message: Image) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: RealYoloPointCloudNode | None = None
    try:
        node = RealYoloPointCloudNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
