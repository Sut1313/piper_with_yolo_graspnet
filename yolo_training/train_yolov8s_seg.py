#!/usr/bin/env python3
"""Fine-tune YOLOv8s-seg and export the best checkpoint to a stable ONNX path."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATASET_YAML = PROJECT_ROOT / "yolo_training/piper_objects.yaml"
EXPECTED_CLASSES = ["cube", "cuboid", "cylinder"]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def portable_reference(path: Path) -> str:
    """Avoid recording a developer's home directory in committed metadata."""
    resolved = path.expanduser().resolve()
    try:
        return str(resolved.relative_to(PROJECT_ROOT))
    except ValueError:
        return resolved.name


def read_training_summary(results_csv: Path, requested_epochs: int) -> dict:
    if not results_csv.is_file():
        return {}
    with results_csv.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        return {}

    def fitness(row: dict[str, str]) -> float:
        # Ultralytics segment fitness: box/segment fitness 取平均；
        # 每项为 0.1*mAP50 + 0.9*mAP50-95。
        return (
            0.1 * float(row["metrics/mAP50(B)"])
            + 0.9 * float(row["metrics/mAP50-95(B)"])
            + 0.1 * float(row["metrics/mAP50(M)"])
            + 0.9 * float(row["metrics/mAP50-95(M)"])
        ) / 2.0

    best = max(rows, key=fitness)
    return {
        "epochs_completed": len(rows),
        "best_epoch": int(best["epoch"]),
        "early_stopped": len(rows) < requested_epochs,
        "validation_metrics": {
            "box_map50": float(best["metrics/mAP50(B)"]),
            "box_map50_95": float(best["metrics/mAP50-95(B)"]),
            "mask_map50": float(best["metrics/mAP50(M)"]),
            "mask_map50_95": float(best["metrics/mAP50-95(M)"]),
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="训练 YOLOv8s-seg，并将最佳权重导出为运行节点直接加载的 ONNX 文件。"
    )
    parser.add_argument(
        "--data",
        type=Path,
        default=DEFAULT_DATASET_YAML,
        help=f"Ultralytics 分割数据集 YAML（默认: {DEFAULT_DATASET_YAML}）",
    )
    parser.add_argument("--base-model", default="yolov8s-seg.pt", help="预训练权重或断点 .pt")
    parser.add_argument("--epochs", type=int, default=150)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default="0", help="例如 0、0,1 或 cpu")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--patience", type=int, default=30)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--project", type=Path, default=PROJECT_ROOT / "runs/segment", help="训练输出目录"
    )
    parser.add_argument("--name", default="piper_yolov8s_seg")
    parser.add_argument("--resume", action="store_true", help="从 --base-model 指定的 last.pt 续训")
    parser.add_argument("--cache", action="store_true", help="将数据集缓存到内存/磁盘以加速训练")
    parser.add_argument("--opset", type=int, default=17)
    parser.add_argument(
        "--onnx-output",
        type=Path,
        default=PROJECT_ROOT / "models/piper_yolov8s_seg.onnx",
    )
    parser.add_argument(
        "--best-output",
        type=Path,
        default=PROJECT_ROOT / "models/piper_yolov8s_seg_best.pt",
    )
    return parser.parse_args()


def validate_dataset_yaml(data_path: Path) -> None:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("训练环境缺少 PyYAML；安装 ultralytics 后会自动包含它") from exc

    config = yaml.safe_load(data_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError(f"数据集 YAML 不是映射结构: {data_path}")
    raw_names = config.get("names")
    if isinstance(raw_names, dict):
        names = [str(raw_names[key]) for key in sorted(raw_names, key=lambda item: int(item))]
    elif isinstance(raw_names, list):
        names = [str(name) for name in raw_names]
    else:
        raise ValueError("数据集 YAML 缺少 names")
    if names != EXPECTED_CLASSES:
        raise ValueError(
            f"类别必须严格为 {EXPECTED_CLASSES}，实际为 {names}；类别 ID 不可重排"
        )
    if len(names) != 3:
        raise ValueError(f"期望 nc=3，实际 nc={len(names)}")

    dataset_root = Path(str(config.get("path", data_path.parent))).expanduser()
    if not dataset_root.is_absolute():
        dataset_root = (data_path.parent / dataset_root).resolve()
    for split in ("train", "val", "test"):
        relative = config.get(split)
        if not relative:
            raise ValueError(f"数据集 YAML 缺少 {split} 划分")
        images_dir = Path(str(relative)).expanduser()
        if not images_dir.is_absolute():
            images_dir = dataset_root / images_dir
        if not images_dir.is_dir():
            raise FileNotFoundError(f"{split} 图像目录不存在: {images_dir}")

    # YOLO segment 每行必须为 class_id + 至少三个二维多边形点。
    label_files = sorted((dataset_root / "labels/train").glob("*.txt"))
    if not label_files:
        raise FileNotFoundError(f"没有找到训练标签: {dataset_root / 'labels/train'}")
    for label_path in label_files:
        for line_number, line in enumerate(label_path.read_text(encoding="utf-8").splitlines(), 1):
            tokens = line.split()
            if not tokens:
                continue  # 空标签文件是合法负样本。
            if len(tokens) < 7 or len(tokens) % 2 == 0:
                raise ValueError(
                    f"检测到非 segmentation 多边形标签: {label_path}:{line_number}"
                )
            class_id = int(tokens[0])
            if class_id not in range(len(EXPECTED_CLASSES)):
                raise ValueError(f"类别 ID 越界: {label_path}:{line_number}: {class_id}")


def main() -> None:
    args = parse_args()
    data_path = args.data.expanduser().resolve()
    if not data_path.is_file():
        raise FileNotFoundError(f"数据集 YAML 不存在: {data_path}")

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            "未安装训练依赖。请先执行: pip install -r yolo_training/requirements-training.txt"
        ) from exc

    validate_dataset_yaml(data_path)

    project_dir = args.project.expanduser().resolve()
    model = YOLO(args.base_model)
    if getattr(model, "task", None) != "segment":
        raise ValueError(
            f"基础模型任务必须是 segment，实际为 {getattr(model, 'task', None)!r}: "
            f"{args.base_model}"
        )
    model.train(
        data=str(data_path),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        patience=args.patience,
        seed=args.seed,
        project=str(project_dir),
        name=args.name,
        exist_ok=True,
        pretrained=not args.resume,
        resume=args.resume,
        cache=args.cache,
        task="segment",
        plots=True,
    )

    best_checkpoint = project_dir / args.name / "weights/best.pt"
    if not best_checkpoint.is_file():
        raise FileNotFoundError(f"训练结束但没有找到最佳权重: {best_checkpoint}")

    args.best_output.parent.mkdir(parents=True, exist_ok=True)
    best_output = args.best_output.expanduser().resolve()
    shutil.copy2(best_checkpoint, best_output)

    best_model = YOLO(str(best_checkpoint))
    exported_path = Path(
        best_model.export(
            format="onnx",
            imgsz=args.imgsz,
            batch=1,
            dynamic=False,
            simplify=True,
            opset=args.opset,
            half=False,
        )
    ).resolve()
    onnx_output = args.onnx_output.expanduser().resolve()
    onnx_output.parent.mkdir(parents=True, exist_ok=True)
    if exported_path != onnx_output:
        shutil.copy2(exported_path, onnx_output)

    training_summary = read_training_summary(
        project_dir / args.name / "results.csv", args.epochs
    )
    metadata = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "data": portable_reference(data_path),
        "data_sha256": sha256_file(data_path),
        "base_model": args.base_model,
        "epochs": args.epochs,
        "imgsz": args.imgsz,
        "batch": args.batch,
        "device": args.device,
        "seed": args.seed,
        "opset": args.opset,
        "task": "segment",
        "classes": EXPECTED_CLASSES,
        "nc": len(EXPECTED_CLASSES),
        "best_checkpoint": portable_reference(best_output),
        "onnx_model": portable_reference(onnx_output),
        "onnx_dynamic": False,
        "onnx_batch": 1,
        **training_summary,
        "sha256": {
            "onnx": sha256_file(onnx_output),
            "best_pt": sha256_file(best_output),
        },
    }
    metadata_path = onnx_output.with_suffix(".training.json")
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"最佳 PyTorch 权重: {best_output}")
    print(f"节点运行所用 ONNX: {onnx_output}")
    print(f"训练与导出记录: {metadata_path}")


if __name__ == "__main__":
    main()
