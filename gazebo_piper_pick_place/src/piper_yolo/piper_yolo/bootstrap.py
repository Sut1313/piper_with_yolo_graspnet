"""Start the ROS node with the project-local ONNX Runtime environment."""

from __future__ import annotations

import os
import sys

from .paths import find_project_root


def main() -> None:
    project_root = find_project_root(__file__)
    runtime_python = project_root / ".venv-yolo-runtime/bin/python"
    already_bootstrapped = os.environ.get("PIPER_YOLO_BOOTSTRAPPED") == "1"

    if runtime_python.is_file() and not already_bootstrapped:
        environment = os.environ.copy()
        environment["PIPER_YOLO_BOOTSTRAPPED"] = "1"
        os.execve(
            str(runtime_python),
            [str(runtime_python), "-m", "piper_yolo.yolo_bridge_node", *sys.argv[1:]],
            environment,
        )

    from .yolo_bridge_node import main as run_node

    run_node()


if __name__ == "__main__":
    main()
