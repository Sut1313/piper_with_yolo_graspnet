#!/usr/bin/env bash
# ===========================================================================
# graspnet-docker-camera.sh
#
# 启动 GraspNet Docker 容器，连接共享相机数据目录，实时推理并显示结果。
#
# 前提：
#   1. Docker 镜像已构建（./graspnet-baseline/docker-build.sh）
#   2. ROS 2 + Gazebo 感知流水线正在运行（graspnet_pipeline.launch.py）
#   3. 本机有 X11 图形环境
#
# 用法：
#   ./graspnet-docker-camera.sh [gui|headless]
#
#   gui       → 显示可视化窗口（默认，需要 X11）
#   headless  → 无窗口运行（仅保存结果到 shared_camera_data/output/）
#
# 流程：
#   ROS 2 相机话题 → camera_bridge_node → shared_camera_data/
#     → Docker 内 inference_from_camera.py → GraspNet 推理 → 屏幕显示
# ===========================================================================

set -euo pipefail

# 脚本目录（无论从哪里调用都定位到 piper_graspnet 根目录）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# 可用环境变量覆盖镜像名（默认 graspnet-baseline:20.04）
IMAGE_NAME="${IMAGE_NAME:-graspnet-baseline:20.04}"
# 共享相机数据目录（与 launch 文件里的 shared_dir 一致）
SHARED_DIR="${SCRIPT_DIR}/shared_camera_data"
# 模型权重路径（容器内相对路径，推理脚本默认值）
CHECKPOINT_PATH="${CHECKPOINT_PATH:-logs/log_rs/checkpoint.tar}"
# 运行模式：gui（默认，显示窗口）/ headless（无窗口，只保存结果）
MODE="${1:-gui}"

# 共享目录映射（容器内路径固定，与推理脚本默认值一致）
SHARED_MOUNT="/shared_camera_data"

# 确保共享目录存在
mkdir -p "${SHARED_DIR}"

echo "============================================"
echo "GraspNet Docker Camera Inference"
echo "  image:       ${IMAGE_NAME}"
echo "  shared_dir:  ${SHARED_DIR} → ${SHARED_MOUNT}"
echo "  checkpoint:  ${CHECKPOINT_PATH}"
echo "  mode:        ${MODE}"
echo "============================================"

case "${MODE}" in
  gui)
    # GUI 模式：需要宿主机 X11（图形会话），DISPLAY 未设置时直接报错退出
    if [[ -z "${DISPLAY:-}" ]]; then
      echo "ERROR: DISPLAY is not set. Run from a graphical desktop session." >&2
      exit 1
    fi
    # 允许 Docker 容器访问 X11（xhost 授权，失败可忽略——多数桌面已默认开放）
    xhost +local:docker >/dev/null 2>&1 || true

    # Docker 运行参数说明：
    #   --gpus 'all,"capabilities=compute,utility,graphics"'：GPU 计算 + 渲染
    #   --ipc=host：共享主机 IPC（PyTorch/Open3D 多进程需要）
    #   --network=host：复用宿主机网络（容器内不单独建网）
    #   -e DISPLAY / -v /tmp/.X11-unix：X11 透传（GUI 显示）
    #   -e EGL_PLATFORM=surfaceless：无显示器环境下的 EGL 渲染
    #   -e QT_X11_NO_MITSHM=1：禁用 MIT-SHM（跨容器 X11 常用规避手段）
    #   -v ${SHARED_DIR}:${SHARED_MOUNT}:rw：相机数据双向共享
    #   -v 推理脚本 :ro：以只读方式挂载宿主机最新脚本（改代码免重建镜像）
    docker run --rm -it \
      --gpus 'all,"capabilities=compute,utility,graphics"' \
      --ipc=host \
      --network=host \
      -e DISPLAY="${DISPLAY}" \
      -e EGL_PLATFORM=surfaceless \
      -e QT_X11_NO_MITSHM=1 \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v "${SHARED_DIR}:${SHARED_MOUNT}:rw" \
      -v "${SCRIPT_DIR}/graspnet-baseline/inference_from_camera.py:/workspace/inference_from_camera.py:ro" \
      "${IMAGE_NAME}" \
      python inference_from_camera.py \
        --checkpoint_path "${CHECKPOINT_PATH}" \
        --shared_dir "${SHARED_MOUNT}" \
        --num_point 20000 \
        --num_view 300 \
        --collision_thresh 0.01 \
        --top_k 50
    ;;
  headless)
    # Headless 模式：不传 DISPLAY / X11 挂载，推理脚本加 --no_display，
    # 只把结果写入共享目录 output/（供 open3d_result_viewer.py 复盘）。
    docker run --rm \
      --gpus 'all,"capabilities=compute,utility,graphics"' \
      --ipc=host \
      --network=host \
      -e EGL_PLATFORM=surfaceless \
      -v "${SHARED_DIR}:${SHARED_MOUNT}:rw" \
      -v "${SCRIPT_DIR}/graspnet-baseline/inference_from_camera.py:/workspace/inference_from_camera.py:ro" \
      "${IMAGE_NAME}" \
      python inference_from_camera.py \
        --checkpoint_path "${CHECKPOINT_PATH}" \
        --shared_dir "${SHARED_MOUNT}" \
        --num_point 20000 \
        --num_view 300 \
        --collision_thresh 0.01 \
        --top_k 50 \
        --no_display
    echo "Headless inference complete. Results saved in ${SHARED_DIR}/output/"
    ;;
  *)
    # 非法参数：打印用法并退出
    echo "Usage: $0 [gui|headless]" >&2
    exit 2
    ;;
esac
