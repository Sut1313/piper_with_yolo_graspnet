#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-graspnet-baseline:20.04}"
MODE="${1:-gui}"
CHECKPOINT_PATH="${CHECKPOINT_PATH:-logs/log_rs/checkpoint.tar}"
DATA_DIR="${DATA_DIR:-doc/example_data}"
OUTPUT_DIR="${OUTPUT_DIR:-$(pwd)/outputs}"
NUM_POINT="${NUM_POINT:-20000}"
NUM_VIEW="${NUM_VIEW:-300}"
COLLISION_THRESH="${COLLISION_THRESH:--1}"
TOP_K="${TOP_K:-50}"

mkdir -p "${OUTPUT_DIR}"

case "${MODE}" in
  gui)
    if [[ -z "${DISPLAY:-}" ]]; then
      echo "DISPLAY is not set. Use './docker-run-demo.sh headless' or run from a graphical desktop session." >&2
      exit 1
    fi
    xhost +local:docker >/dev/null
    docker run --rm -it \
      --gpus all \
      --ipc=host \
      -e DISPLAY="${DISPLAY}" \
      -e QT_X11_NO_MITSHM=1 \
      -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v "${OUTPUT_DIR}:/workspace/outputs:rw" \
      "${IMAGE_NAME}" \
      python demo.py --checkpoint_path "${CHECKPOINT_PATH}"
    ;;
  headless)
    docker run --rm \
      --gpus all \
      --ipc=host \
      -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
      -v "${OUTPUT_DIR}:/workspace/outputs:rw" \
      "${IMAGE_NAME}" \
      xvfb-run -s "-screen 0 1920x1080x24" \
      python demo_headless.py \
        --checkpoint_path "${CHECKPOINT_PATH}" \
        --data_dir "${DATA_DIR}" \
        --save_path /workspace/outputs/demo_result.png
    echo "Saved: ${OUTPUT_DIR}/demo_result.png"
    ;;
  save)
    docker run --rm \
      --gpus all \
      --ipc=host \
      -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
      -v "${OUTPUT_DIR}:/workspace/outputs:rw" \
      "${IMAGE_NAME}" \
      python demo_save.py \
        --checkpoint_path "${CHECKPOINT_PATH}" \
        --data_dir "${DATA_DIR}" \
        --output_dir /workspace/outputs \
        --num_point "${NUM_POINT}" \
        --num_view "${NUM_VIEW}" \
        --collision_thresh "${COLLISION_THRESH}" \
        --top_k "${TOP_K}"
    echo "Saved outputs under: ${OUTPUT_DIR}"
    ;;
  *)
    echo "Usage: $0 [gui|headless|save]" >&2
    exit 2
    ;;
esac
