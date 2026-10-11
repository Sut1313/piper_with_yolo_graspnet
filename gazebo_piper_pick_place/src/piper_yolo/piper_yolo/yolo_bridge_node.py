"""ROS2 node that bridges synchronized RGB-D frames and YOLO masks to GraspNet."""

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
from sensor_msgs.msg import CameraInfo, Image
from scipy.io import savemat

from .paths import default_model_path, default_shared_dir
from .yolov8_onnx import Detection, Yolov8SegOnnx


class YoloBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_bridge_node")
        self.declare_parameter("model_path", str(default_model_path(__file__)))
        self.declare_parameter("shared_dir", str(default_shared_dir(__file__)))
        self.declare_parameter("class_names", ["cube", "cuboid", "cylinder"])
        self.declare_parameter("color_topic", "/camera/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.50)
        self.declare_parameter("max_detections", 20)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("prefer_cuda", True)
        self.declare_parameter("depth_factor", 1000.0)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop", 0.05)
        # The upstream D435i bridge publishes synchronized RGB-D frames at
        # 5 Hz. State-driven mode may publish less often, but never faster.
        self.declare_parameter("min_publish_interval", 0.20)
        self.declare_parameter("mask_erode_pixels", 1)
        self.declare_parameter("filter_finished_objects", True)
        self.declare_parameter("idle_without_pick_state", False)
        self.declare_parameter("single_instance_per_label", False)
        self.declare_parameter("write_debug_overlay", True)

        self.shared_dir = Path(self.get_parameter("shared_dir").value).expanduser().resolve()
        self.shared_dir.mkdir(parents=True, exist_ok=True)
        class_names = [str(name) for name in self.get_parameter("class_names").value]
        self.detector = Yolov8SegOnnx(
            model_path=str(self.get_parameter("model_path").value),
            class_names=class_names,
            confidence_threshold=float(self.get_parameter("confidence_threshold").value),
            iou_threshold=float(self.get_parameter("iou_threshold").value),
            max_detections=int(self.get_parameter("max_detections").value),
            prefer_cuda=bool(self.get_parameter("prefer_cuda").value),
            image_size=int(self.get_parameter("image_size").value),
        )
        self.class_names = class_names
        self.bridge = CvBridge()
        self.camera_info: CameraInfo | None = None
        self.frame_sequence = 0
        self.last_publish_time = 0.0
        self.waiting_log_time = 0.0
        self.current_state_version: int | None = None
        self.last_submitted_state_version: int | None = None

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
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
            f"YOLO 桥接节点已启动，providers={self.detector.providers}, "
            f"shared_dir={self.shared_dir}"
        )

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self.camera_info = message

    def _rgbd_callback(self, color_message: Image, depth_message: Image) -> None:
        now = time.monotonic()
        if now - self.last_publish_time < float(
            self.get_parameter("min_publish_interval").value
        ):
            return
        ready_path = self.shared_dir / ".ready"
        if ready_path.exists():
            if now - self.waiting_log_time > 5.0:
                self.get_logger().info("GraspNet 尚未消费上一帧，暂缓写入新帧")
                self.waiting_log_time = now
            return
        if self.camera_info is None:
            if now - self.waiting_log_time > 5.0:
                self.get_logger().warning("尚未收到 CameraInfo，等待相机内参")
                self.waiting_log_time = now
            return

        try:
            color = self.bridge.imgmsg_to_cv2(color_message, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
            depth_mm = self._depth_to_uint16_mm(depth, depth_message.encoding)
            if color.shape[:2] != depth_mm.shape[:2]:
                raise ValueError(
                    f"彩色图与深度图尺寸不一致: {color.shape[:2]} != {depth_mm.shape[:2]}"
                )
            active_names = self._read_active_names()
            # The task executor disables perception while the arm is planning
            # or moving.  Do not emit empty frames: that would wake GraspNet
            # and waste most of the workstation on a scene we intentionally
            # do not want to perceive.
            if active_names == set():
                return
            # One executor state version represents one perception request.
            # GraspNet can consume .ready before the executor has switched to
            # preflight; without this guard YOLO immediately submits the same
            # scene again and starts an inference that nobody will use.
            if (
                self.current_state_version is not None
                and self.current_state_version == self.last_submitted_state_version
            ):
                return
            detections = self.detector.infer(color, allowed_labels=active_names)
            detections = self._erode_masks(detections)
            intrinsic = np.asarray(self.camera_info.k, dtype=np.float64).reshape(3, 3)
            detections = self._select_target_instances(detections, depth_mm, intrinsic)
            self._write_frame(color, depth_mm, detections, color_message, depth_message)
            self.last_submitted_state_version = self.current_state_version
            self.last_publish_time = now
        except Exception as exc:  # ROS callback must stay alive after a bad frame.
            self.get_logger().error(f"处理 RGB-D 帧失败: {exc}")

    def _depth_to_uint16_mm(self, depth: np.ndarray, encoding: str) -> np.ndarray:
        if encoding.upper() == "16UC1" or depth.dtype == np.uint16:
            return depth.astype(np.uint16, copy=False)
        factor = float(self.get_parameter("depth_factor").value)
        depth_float = np.nan_to_num(depth.astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        depth_float[depth_float < 0.0] = 0.0
        return np.clip(np.rint(depth_float * factor), 0, np.iinfo(np.uint16).max).astype(
            np.uint16
        )

    def _read_active_names(self) -> set[str] | None:
        self.current_state_version = None
        if not bool(self.get_parameter("filter_finished_objects").value):
            return None
        state_path = self.shared_dir / "pick_state.json"
        if not state_path.exists():
            if bool(self.get_parameter("idle_without_pick_state").value):
                return set()
            return set(self.class_names)
        try:
            state = json.loads(state_path.read_text(encoding="utf-8"))
            version = state.get("version")
            if isinstance(version, int):
                self.current_state_version = version
            if state.get("perception_enabled") is False:
                return set()
            active_target = state.get("active_target")
            if isinstance(active_target, str) and active_target in self.class_names:
                return {active_target}
            pending = state.get("pending")
            if isinstance(pending, list):
                if not pending:
                    return set()
                recognized = {str(name) for name in pending if str(name) in self.class_names}
                if recognized:
                    return recognized
                self.get_logger().warning(
                    "pick_state 的类别与 YOLO 类别不相交；暂不按状态过滤。"
                    f" pending={pending}, yolo={self.class_names}"
                )
                return None
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"读取 pick_state.json 失败，将识别全部类别: {exc}")
        return set(self.class_names)

    def _select_target_instances(
        self,
        detections: list[Detection],
        depth_mm: np.ndarray,
        intrinsic: np.ndarray,
    ) -> list[Detection]:
        """Keep one physical instance per requested class.

        The lightweight task has one source object for each label.  When YOLO
        produces duplicate same-label masks (often a placed object), merging
        them corrupts the target point cloud.  Prefer the mask whose RGB-D
        centroid is closest to the configured source-object prior; fall back
        to confidence when calibration metadata is unavailable.
        """
        if not bool(self.get_parameter("single_instance_per_label").value):
            return detections

        grouped: dict[str, list[Detection]] = {}
        for detection in detections:
            grouped.setdefault(detection.label, []).append(detection)

        config: dict[str, Any] = {}
        config_path = self.shared_dir / "camera_to_world.json"
        try:
            config = json.loads(config_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            pass
        rotation = np.asarray(config.get("camera_to_world_r", []), dtype=np.float64)
        translation = np.asarray(config.get("camera_to_world_t", []), dtype=np.float64)
        priors = config.get("source_object_priors_world", {})
        calibrated = rotation.shape == (3, 3) and translation.shape == (3,)

        selected: list[Detection] = []
        factor = float(self.get_parameter("depth_factor").value)
        fx, fy = intrinsic[0, 0], intrinsic[1, 1]
        cx, cy = intrinsic[0, 2], intrinsic[1, 2]
        for label, instances in grouped.items():
            if len(instances) == 1:
                selected.append(instances[0])
                continue

            prior = np.asarray(priors.get(label, []), dtype=np.float64)

            def instance_rank(detection: Detection) -> tuple[float, float]:
                if not calibrated or prior.shape != (3,):
                    return (float("inf"), -float(detection.score))
                valid = detection.mask & (depth_mm > 0)
                rows, columns = np.nonzero(valid)
                if len(rows) == 0:
                    return (float("inf"), -float(detection.score))
                z = depth_mm[rows, columns].astype(np.float64) / factor
                camera_points = np.stack(
                    ((columns - cx) * z / fx, (rows - cy) * z / fy, z), axis=1
                )
                camera_center = np.median(camera_points, axis=0)
                world_center = rotation @ camera_center + translation
                return (
                    float(np.linalg.norm(world_center[:2] - prior[:2])),
                    -float(detection.score),
                )

            best = min(instances, key=instance_rank)
            selected.append(best)
            self.get_logger().info(
                f"{label} 检出 {len(instances)} 个实例，仅保留源位置最近的实例"
            )
        return selected

    def _erode_masks(self, detections: list[Detection]) -> list[Detection]:
        pixels = int(self.get_parameter("mask_erode_pixels").value)
        if pixels <= 0:
            return detections
        size = pixels * 2 + 1
        kernel = np.ones((size, size), dtype=np.uint8)
        result = []
        for detection in detections:
            eroded = cv2.erode(detection.mask.astype(np.uint8), kernel, iterations=1) > 0
            result.append(
                Detection(
                    class_id=detection.class_id,
                    label=detection.label,
                    score=detection.score,
                    box_xyxy=detection.box_xyxy,
                    mask=eroded,
                )
            )
        return result

    def _write_frame(
        self,
        color: np.ndarray,
        depth_mm: np.ndarray,
        detections: list[Detection],
        color_message: Image,
        depth_message: Image,
    ) -> None:
        self.frame_sequence += 1
        sequence = self.frame_sequence
        masks = np.stack([d.mask.astype(np.uint8) for d in detections]) if detections else np.empty(
            (0, color.shape[0], color.shape[1]), dtype=np.uint8
        )
        class_ids = np.asarray([d.class_id for d in detections], dtype=np.int64)
        scores = np.asarray([d.score for d in detections], dtype=np.float32)
        boxes = np.asarray([d.box_xyxy for d in detections], dtype=np.float32).reshape(-1, 4)
        labels = np.asarray([d.label for d in detections], dtype=np.str_)
        camera_info = self.camera_info
        assert camera_info is not None
        intrinsic = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)

        self._atomic_write_image(self.shared_dir / "color.png", color)
        self._atomic_write_image(self.shared_dir / "depth.png", depth_mm)
        self._atomic_write_mat(
            self.shared_dir / "meta.mat",
            {
                "intrinsic_matrix": intrinsic,
                "factor_depth": np.asarray(
                    [[float(self.get_parameter("depth_factor").value)]], dtype=np.float64
                ),
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
        detection_records = [
            {
                "class_id": d.class_id,
                "label": d.label,
                "score": d.score,
                "box_xyxy": [float(value) for value in d.box_xyxy],
                "mask_pixels": int(np.count_nonzero(d.mask)),
            }
            for d in detections
        ]
        self._atomic_write_json(
            self.shared_dir / "detections.json",
            {"frame_sequence": sequence, "detections": detection_records},
        )
        if bool(self.get_parameter("write_debug_overlay").value):
            self._atomic_write_image(
                self.shared_dir / "yolo_overlay.png", self._draw_overlay(color, detections)
            )
        self._atomic_write_json(
            self.shared_dir / "frame_manifest.json",
            {
                "frame_sequence": sequence,
                "color_stamp_ns": self._stamp_ns(color_message),
                "depth_stamp_ns": self._stamp_ns(depth_message),
                "image_shape": [int(color.shape[0]), int(color.shape[1])],
                "detection_count": len(detections),
            },
        )
        self._atomic_write_text(self.shared_dir / ".ready", f"{sequence}\n")
        labels_text = ", ".join(f"{d.label}:{d.score:.2f}" for d in detections) or "无目标"
        self.get_logger().info(f"已提交帧 {sequence} 给 GraspNet：{labels_text}")

    @staticmethod
    def _stamp_ns(message: Image) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    @staticmethod
    def _draw_overlay(color: np.ndarray, detections: list[Detection]) -> np.ndarray:
        overlay = color.copy()
        palette = [(40, 40, 230), (40, 200, 40), (230, 100, 40), (220, 40, 220)]
        for detection in detections:
            draw_color = palette[detection.class_id % len(palette)]
            tint = np.zeros_like(overlay)
            tint[detection.mask] = draw_color
            overlay = cv2.addWeighted(overlay, 1.0, tint, 0.35, 0.0)
            x1, y1, x2, y2 = np.rint(detection.box_xyxy).astype(int)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), draw_color, 2)
            cv2.putText(
                overlay,
                f"{detection.label} {detection.score:.2f}",
                (x1, max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                draw_color,
                2,
                cv2.LINE_AA,
            )
        return overlay

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
        self._atomic_write_text(path, json.dumps(value, ensure_ascii=False, indent=2) + "\n")

    def _atomic_write_text(self, path: Path, value: str) -> None:
        temporary = self._temporary_path(path)
        temporary.write_text(value, encoding="utf-8")
        os.replace(temporary, path)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: YoloBridgeNode | None = None
    try:
        node = YoloBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
