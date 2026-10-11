import sys
from pathlib import Path

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from piper_yolo.yolov8_onnx import Yolov8SegOnnx, class_aware_nms, letterbox


def test_letterbox_preserves_aspect_ratio():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    result, ratio, padding = letterbox(image, (640, 640))
    assert result.shape == (640, 640, 3)
    assert ratio == 1.0
    assert padding == (0.0, 80.0)


def test_class_aware_nms_keeps_overlapping_different_classes():
    boxes = np.asarray([[0, 0, 10, 10], [1, 1, 11, 11], [1, 1, 11, 11]], dtype=np.float32)
    scores = np.asarray([0.9, 0.8, 0.7], dtype=np.float32)
    classes = np.asarray([0, 0, 1], dtype=np.int64)
    kept = class_aware_nms(boxes, scores, classes, 0.5, 10)
    assert kept.tolist() == [0, 2]


def test_postprocess_reconstructs_mask_and_removes_letterbox_padding():
    detector = Yolov8SegOnnx.__new__(Yolov8SegOnnx)
    detector.class_names = ["cube", "cuboid", "cylinder"]
    detector.confidence_threshold = 0.35
    detector.iou_threshold = 0.5
    detector.max_detections = 20
    detector.input_size = (640, 640)

    # 4 box + 3 classes + 2 mask coefficients, one candidate.
    prediction = np.zeros((1, 9, 1), dtype=np.float32)
    prediction[0, :4, 0] = [320.0, 320.0, 200.0, 200.0]
    prediction[0, 4:7, 0] = [0.9, 0.1, 0.05]
    prediction[0, 7:, 0] = [10.0, 0.0]
    prototype = np.zeros((1, 2, 160, 160), dtype=np.float32)
    prototype[0, 0] = 1.0

    detections = detector._postprocess(
        prediction,
        prototype,
        original_size=(480, 640),
        ratio=1.0,
        padding=(0.0, 80.0),
        allowed_labels=None,
    )
    assert len(detections) == 1
    assert detections[0].label == "cube"
    assert detections[0].mask.shape == (480, 640)
    assert detections[0].mask[240, 320]
    assert not detections[0].mask[0, 0]
    np.testing.assert_allclose(detections[0].box_xyxy, [220, 140, 420, 340])
