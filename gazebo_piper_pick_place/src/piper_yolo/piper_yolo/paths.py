"""Portable project-path discovery shared by ROS nodes and launch files."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable


_ROOT_ENV_VARS = ("PIPER_GRASP_ROOT", "PIPER_YOLO_PROJECT_ROOT")
_SHARED_ENV_VARS = ("PIPER_SHARED_DIR", "PIPER_YOLO_SHARED_DIR")
_MODEL_ENV_VARS = ("PIPER_YOLO_MODEL", "PIPER_YOLO_MODEL_PATH")
_REAL_MODEL_ENV_VARS = ("PIPER_REAL_YOLO_MODEL",)


def _first_environment_path(names: Iterable[str]) -> Path | None:
    for name in names:
        value = os.environ.get(name)
        if value:
            return Path(value).expanduser().resolve()
    return None


def find_project_root(anchor: str | Path | None = None) -> Path:
    """Find the checkout root without assuming a user name or clone location."""
    configured = _first_environment_path(_ROOT_ENV_VARS)
    if configured is not None:
        return configured

    starts = [Path.cwd()]
    if anchor is not None:
        anchor_path = Path(anchor).expanduser().resolve()
        starts.insert(0, anchor_path if anchor_path.is_dir() else anchor_path.parent)
    starts.insert(0, Path(__file__).resolve().parent)

    visited: set[Path] = set()
    for start in starts:
        for candidate in (start, *start.parents):
            if candidate in visited:
                continue
            visited.add(candidate)
            if (
                (candidate / "gazebo_piper_pick_place").is_dir()
                and (candidate / "graspnet-baseline").is_dir()
            ):
                return candidate

    # This supports relocatable binary installs when the caller supplies paths
    # as ROS parameters, while keeping a useful default for source checkouts.
    return Path.cwd().resolve()


def default_shared_dir(anchor: str | Path | None = None) -> Path:
    configured = _first_environment_path(_SHARED_ENV_VARS)
    return configured or find_project_root(anchor) / "shared_camera_data"


def default_model_path(anchor: str | Path | None = None) -> Path:
    configured = _first_environment_path(_MODEL_ENV_VARS)
    return configured or find_project_root(anchor) / "models/piper_yolov8s_seg.onnx"


def tabletop_model_path(anchor: str | Path | None = None) -> Path:
    """Return the 16-class model used only by the real-camera node."""
    configured = _first_environment_path(_REAL_MODEL_ENV_VARS)
    return configured or find_project_root(anchor) / "models/tabletop_yolov8s_seg_v2.onnx"


def legacy_grasp_model_path(anchor: str | Path | None = None) -> Path:
    """Return the three-class model used by the legacy simulation task."""
    return find_project_root(anchor) / "models/piper_yolov8s_seg.onnx"
