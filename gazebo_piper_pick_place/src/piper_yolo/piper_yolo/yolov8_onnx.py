"""Minimal NumPy/OpenCV post-processing for Ultralytics YOLOv8-seg ONNX models."""

from __future__ import annotations

import ast
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    class_id: int
    label: str
    score: float
    box_xyxy: np.ndarray
    mask: np.ndarray


def letterbox(image: np.ndarray, size: tuple[int, int]) -> tuple[np.ndarray, float, tuple[float, float]]:
    """Resize with unchanged aspect ratio and symmetric padding."""
    target_h, target_w = size
    src_h, src_w = image.shape[:2]
    ratio = min(target_w / src_w, target_h / src_h)
    resized_w = int(round(src_w * ratio))
    resized_h = int(round(src_h * ratio))
    pad_w = (target_w - resized_w) / 2.0
    pad_h = (target_h - resized_h) / 2.0

    resized = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    left = int(round(pad_w - 0.1))
    right = int(round(pad_w + 0.1))
    top = int(round(pad_h - 0.1))
    bottom = int(round(pad_h + 0.1))
    padded = cv2.copyMakeBorder(
        resized,
        top,
        bottom,
        left,
        right,
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114),
    )
    return padded, ratio, (pad_w, pad_h)


def _xywh_to_xyxy(boxes: np.ndarray) -> np.ndarray:
    result = boxes.copy()
    result[:, 0] = boxes[:, 0] - boxes[:, 2] / 2.0
    result[:, 1] = boxes[:, 1] - boxes[:, 3] / 2.0
    result[:, 2] = boxes[:, 0] + boxes[:, 2] / 2.0
    result[:, 3] = boxes[:, 1] + boxes[:, 3] / 2.0
    return result


def _box_iou_one_to_many(box: np.ndarray, boxes: np.ndarray) -> np.ndarray:
    x1 = np.maximum(box[0], boxes[:, 0])
    y1 = np.maximum(box[1], boxes[:, 1])
    x2 = np.minimum(box[2], boxes[:, 2])
    y2 = np.minimum(box[3], boxes[:, 3])
    intersection = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    box_area = max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])
    boxes_area = np.maximum(0.0, boxes[:, 2] - boxes[:, 0]) * np.maximum(
        0.0, boxes[:, 3] - boxes[:, 1]
    )
    return intersection / np.maximum(box_area + boxes_area - intersection, 1e-7)


def class_aware_nms(
    boxes: np.ndarray,
    scores: np.ndarray,
    class_ids: np.ndarray,
    iou_threshold: float,
    max_detections: int,
) -> np.ndarray:
    """Return indices kept by class-aware non-maximum suppression."""
    kept: list[int] = []
    for class_id in np.unique(class_ids):
        indices = np.flatnonzero(class_ids == class_id)
        order = indices[np.argsort(scores[indices])[::-1]]
        while order.size:
            current = int(order[0])
            kept.append(current)
            if order.size == 1:
                break
            remaining = order[1:]
            ious = _box_iou_one_to_many(boxes[current], boxes[remaining])
            order = remaining[ious <= iou_threshold]
    kept.sort(key=lambda index: float(scores[index]), reverse=True)
    return np.asarray(kept[:max_detections], dtype=np.int64)


def _crop_masks(masks: np.ndarray, boxes: np.ndarray) -> np.ndarray:
    count, height, width = masks.shape
    x = np.arange(width, dtype=np.float32)[None, None, :]
    y = np.arange(height, dtype=np.float32)[None, :, None]
    x1 = boxes[:, 0, None, None]
    y1 = boxes[:, 1, None, None]
    x2 = boxes[:, 2, None, None]
    y2 = boxes[:, 3, None, None]
    crop = (x >= x1) & (x < x2) & (y >= y1) & (y < y2)
    return masks * crop.reshape(count, height, width)


class Yolov8SegOnnx:
    """Load and run a fixed-shape YOLOv8 segmentation ONNX model."""

    def __init__(
        self,
        model_path: str,
        class_names: Optional[Sequence[str]] = None,
        confidence_threshold: float = 0.35,
        iou_threshold: float = 0.50,
        max_detections: int = 20,
        prefer_cuda: bool = True,
        image_size: int = 640,
    ) -> None:
        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise RuntimeError(
                "未安装 ONNX Runtime。GPU 环境请安装 onnxruntime-gpu，"
                "CPU 环境请安装 onnxruntime。"
            ) from exc

        path = Path(model_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"YOLO ONNX 模型不存在: {path}")

        available = ort.get_available_providers()
        providers = []
        if prefer_cuda and "CUDAExecutionProvider" in available:
            providers.append("CUDAExecutionProvider")
        providers.append("CPUExecutionProvider")
        self.session = ort.InferenceSession(str(path), providers=providers)
        self.input = self.session.get_inputs()[0]
        self.output_names = [output.name for output in self.session.get_outputs()]
        self.confidence_threshold = float(confidence_threshold)
        self.iou_threshold = float(iou_threshold)
        self.max_detections = int(max_detections)

        input_shape = self.input.shape
        if len(input_shape) != 4:
            raise ValueError(f"期望 NCHW 四维输入，实际为 {input_shape}")
        input_h = input_shape[2] if isinstance(input_shape[2], int) else image_size
        input_w = input_shape[3] if isinstance(input_shape[3], int) else image_size
        self.input_size = (int(input_h), int(input_w))

        metadata_names = self._read_names_from_metadata()
        self.class_names = list(class_names or metadata_names)
        if not self.class_names:
            raise ValueError("模型没有类别元数据，必须通过 class_names 参数提供类别名称")

    @property
    def providers(self) -> list[str]:
        return list(self.session.get_providers())

    def _read_names_from_metadata(self) -> list[str]:
        raw_names = self.session.get_modelmeta().custom_metadata_map.get("names", "")
        if not raw_names:
            return []
        try:
            parsed = ast.literal_eval(raw_names)
        except (ValueError, SyntaxError):
            return []
        if isinstance(parsed, dict):
            return [str(parsed[key]) for key in sorted(parsed, key=lambda item: int(item))]
        if isinstance(parsed, (list, tuple)):
            return [str(name) for name in parsed]
        return []

    def infer(
        self,
        image_bgr: np.ndarray,
        allowed_labels: Optional[Iterable[str]] = None,
    ) -> list[Detection]:
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError(f"期望 HxWx3 BGR 图像，实际为 {image_bgr.shape}")
        original_h, original_w = image_bgr.shape[:2]
        padded, ratio, (pad_w, pad_h) = letterbox(image_bgr, self.input_size)
        tensor = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        tensor = tensor.astype(np.float32) / 255.0
        tensor = np.transpose(tensor, (2, 0, 1))[None]

        outputs = self.session.run(self.output_names, {self.input.name: tensor})
        prediction, prototype = self._identify_outputs(outputs)
        return self._postprocess(
            prediction,
            prototype,
            (original_h, original_w),
            ratio,
            (pad_w, pad_h),
            set(allowed_labels) if allowed_labels is not None else None,
        )

    @staticmethod
    def _identify_outputs(outputs: Sequence[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
        prototype = next((array for array in outputs if array.ndim == 4), None)
        prediction = next((array for array in outputs if array.ndim == 3), None)
        if prototype is None or prediction is None:
            shapes = [array.shape for array in outputs]
            raise ValueError(f"无法识别 YOLOv8-seg 输出，实际输出形状: {shapes}")
        return prediction, prototype

    def _postprocess(
        self,
        prediction: np.ndarray,
        prototype: np.ndarray,
        original_size: tuple[int, int],
        ratio: float,
        padding: tuple[float, float],
        allowed_labels: Optional[set[str]],
    ) -> list[Detection]:
        proto = np.squeeze(prototype, axis=0).astype(np.float32, copy=False)
        mask_channels, mask_h, mask_w = proto.shape
        pred = np.squeeze(prediction, axis=0)
        class_count = len(self.class_names)
        standard_features = 4 + class_count + mask_channels
        objectness_features = 5 + class_count + mask_channels

        if pred.shape[0] in (standard_features, objectness_features):
            pred = pred.T
        elif pred.shape[1] not in (standard_features, objectness_features):
            raise ValueError(
                "模型输出与类别数不匹配: "
                f"prediction={prediction.shape}, prototype={prototype.shape}, "
                f"classes={class_count}"
            )

        has_objectness = pred.shape[1] == objectness_features
        class_start = 5 if has_objectness else 4
        class_scores = pred[:, class_start : class_start + class_count]
        class_ids = np.argmax(class_scores, axis=1).astype(np.int64)
        scores = class_scores[np.arange(class_scores.shape[0]), class_ids]
        if has_objectness:
            scores = scores * pred[:, 4]

        keep = scores >= self.confidence_threshold
        if allowed_labels is not None:
            allowed_ids = {
                index for index, label in enumerate(self.class_names) if label in allowed_labels
            }
            keep &= np.fromiter(
                (int(class_id) in allowed_ids for class_id in class_ids),
                dtype=bool,
                count=class_ids.size,
            )
        if not np.any(keep):
            return []

        filtered = pred[keep]
        scores = scores[keep]
        class_ids = class_ids[keep]
        boxes_input = _xywh_to_xyxy(filtered[:, :4])
        mask_coefficients = filtered[:, class_start + class_count :]
        keep_indices = class_aware_nms(
            boxes_input,
            scores,
            class_ids,
            self.iou_threshold,
            self.max_detections,
        )
        boxes_input = boxes_input[keep_indices]
        scores = scores[keep_indices]
        class_ids = class_ids[keep_indices]
        mask_coefficients = mask_coefficients[keep_indices]

        masks = mask_coefficients @ proto.reshape(mask_channels, -1)
        masks = 1.0 / (1.0 + np.exp(-np.clip(masks, -30.0, 30.0)))
        masks = masks.reshape(-1, mask_h, mask_w)
        scale_to_proto = np.asarray(
            [
                mask_w / self.input_size[1],
                mask_h / self.input_size[0],
                mask_w / self.input_size[1],
                mask_h / self.input_size[0],
            ],
            dtype=np.float32,
        )
        masks = _crop_masks(masks, boxes_input * scale_to_proto)

        original_h, original_w = original_size
        pad_w, pad_h = padding
        left = int(round(pad_w - 0.1))
        top = int(round(pad_h - 0.1))
        resized_w = int(round(original_w * ratio))
        resized_h = int(round(original_h * ratio))
        boxes_original = boxes_input.copy()
        boxes_original[:, [0, 2]] = (boxes_original[:, [0, 2]] - pad_w) / ratio
        boxes_original[:, [1, 3]] = (boxes_original[:, [1, 3]] - pad_h) / ratio
        boxes_original[:, [0, 2]] = np.clip(boxes_original[:, [0, 2]], 0, original_w)
        boxes_original[:, [1, 3]] = np.clip(boxes_original[:, [1, 3]], 0, original_h)

        detections: list[Detection] = []
        for index, mask in enumerate(masks):
            mask_input = cv2.resize(
                mask,
                (self.input_size[1], self.input_size[0]),
                interpolation=cv2.INTER_LINEAR,
            )
            mask_unpadded = mask_input[top : top + resized_h, left : left + resized_w]
            mask_original = cv2.resize(
                mask_unpadded,
                (original_w, original_h),
                interpolation=cv2.INTER_LINEAR,
            )
            class_id = int(class_ids[index])
            detections.append(
                Detection(
                    class_id=class_id,
                    label=self.class_names[class_id],
                    score=float(scores[index]),
                    box_xyxy=boxes_original[index].astype(np.float32),
                    mask=mask_original >= 0.5,
                )
            )
        return detections
