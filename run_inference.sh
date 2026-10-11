#!/usr/bin/env bash
# run_inference.sh — 以"后台守护容器"方式启动 GraspNet 推理
#
# 与 graspnet-docker-camera.sh 的区别：
#   - 本脚本用 docker run -d 后台运行（不占用终端），容器命名 graspnet_infer
#   - 适合无人值守/持续推理场景；查看日志用：docker logs graspnet_infer
#   - 停止/重启：docker stop graspnet_infer && docker rm graspnet_infer
#
# 前提：graspnet_pipeline.launch.py（相机桥接）已运行，
# 共享目录 shared_camera_data 有数据流入。
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SHARED_DIR="${PIPER_SHARED_DIR:-${PIPER_YOLO_SHARED_DIR:-${SCRIPT_DIR}/shared_camera_data}}"
IMAGE_NAME="${IMAGE_NAME:-graspnet-baseline:20.04}"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "ERROR: DISPLAY is not set; use ./graspnet-docker-camera.sh headless instead." >&2
  exit 1
fi
if docker container inspect graspnet_infer >/dev/null 2>&1; then
  echo "ERROR: container graspnet_infer already exists; stop/remove it before restarting." >&2
  exit 1
fi
mkdir -p "${SHARED_DIR}"
# 后台守护运行；参数含义与 graspnet-docker-camera.sh 相同
# （GPU 渲染能力、共享 IPC、主机网络、X11 透传、共享目录挂载、
#  推理脚本只读挂载 —— 改脚本免重建镜像）
docker run -d --name graspnet_infer --gpus 'all,"capabilities=compute,utility,graphics"' --ipc=host --network=host \
  -e DISPLAY="${DISPLAY}" \
  -e EGL_PLATFORM=surfaceless \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${SHARED_DIR}:/shared_camera_data:rw" \
  -v "${SCRIPT_DIR}/graspnet-baseline/inference_from_camera.py:/workspace/inference_from_camera.py:ro" \
  "${IMAGE_NAME}" \
  python inference_from_camera.py \
    --checkpoint_path logs/log_rs/checkpoint.tar \
    --shared_dir /shared_camera_data \
    --num_point 20000 --num_view 300 \
    --collision_thresh 0.01 --top_k 50
echo "Container started: $(docker ps --filter name=graspnet_infer --format '{{.ID}}')"
