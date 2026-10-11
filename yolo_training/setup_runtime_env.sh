#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_env="${project_root}/.venv-yolo-runtime"

if [[ ! -x "${runtime_env}/bin/python" ]]; then
  if ! python3 -m venv --system-site-packages "${runtime_env}"; then
    python3 -m pip install --user virtualenv
    python3 -m virtualenv --system-site-packages "${runtime_env}"
  fi
fi

"${runtime_env}/bin/python" -m pip install \
  -r "${project_root}/yolo_training/requirements-runtime-cpu.txt"

echo "YOLO ONNX CPU 运行环境已就绪: ${runtime_env}"
