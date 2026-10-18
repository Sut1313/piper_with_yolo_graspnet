"""Start the real-camera node inside the project-local ONNX environment."""

from __future__ import annotations

import os
import sys

from .paths import find_project_root


def main() -> None:
    project_root = find_project_root(__file__)
    runtime_python = project_root / ".venv-yolo-runtime/bin/python"
    already_bootstrapped = os.environ.get("PIPER_REAL_YOLO_BOOTSTRAPPED") == "1"
    if runtime_python.is_file() and not already_bootstrapped:
        environment = os.environ.copy()
        environment["PIPER_REAL_YOLO_BOOTSTRAPPED"] = "1"
        os.execve(
            str(runtime_python),
            [
                str(runtime_python),
                "-m",
                "piper_yolo.real_yolo_pointcloud_node",
                *sys.argv[1:],
            ],
            environment,
        )

    from .real_yolo_pointcloud_node import main as run_node

    run_node()


if __name__ == "__main__":
    main()
