#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${1:-graspnet-baseline:20.04}"

# ============================================================
# 构建前校验:pointnet2 必须是修复版
# 背景(2026-08-06):旧版 pointnet2_utils.py 的 QueryAndGroup/GroupAll
#   forward 缺 ret_grouped_xyz 返回分支,与 pointnet2_modules.py 的
#   2 元组解包不匹配,推理必崩(ValueError: not enough values to unpack)。
#   8月2日误用旧代码目录构建出坏镜像,这里强制拦截。
# 修复版特征:文件内含 "if self.ret_grouped_xyz:" 分支(3 处);旧版 0 处。
# ============================================================
PN2_UTILS="pointnet2/pointnet2_utils.py"
if [[ ! -f "${PN2_UTILS}" ]]; then
  echo "ERROR: 找不到 ${PN2_UTILS} —— 请在 graspnet-baseline 目录内运行本脚本" >&2
  exit 1
fi
if ! grep -q "if self.ret_grouped_xyz:" "${PN2_UTILS}"; then
  echo "ERROR: ${PN2_UTILS} 是旧版(缺 ret_grouped_xyz 返回分支),构建出的镜像推理必崩!" >&2
  echo "       请使用修复版代码构建: ~/piper_graspnet 或 ~/real_piper_graspnet 的" >&2
  echo "       graspnet-baseline 目录;勿用 ~/master(旧版,442 行)" >&2
  exit 1
fi
echo "OK: pointnet2 为修复版,继续构建..."

if docker buildx version >/dev/null 2>&1; then
  export DOCKER_BUILDKIT=1
  docker build \
    --progress=plain \
    -f Dockerfile.20.04 \
    -t "${IMAGE_NAME}" \
    .
else
  echo "docker buildx is unavailable; falling back to the legacy builder without cache mounts." >&2
  LEGACY_DOCKERFILE="$(mktemp /tmp/graspnet-dockerfile.XXXXXX)"
  sed \
    -e '/^# syntax=/d' \
    -e 's/^RUN --mount=type=cache[^\\]*\\$/RUN \\/' \
    -e '/^[[:space:]]*--mount=type=cache/d' \
    Dockerfile.20.04 > "${LEGACY_DOCKERFILE}"
  DOCKER_BUILDKIT=0 docker build \
    -f "${LEGACY_DOCKERFILE}" \
    -t "${IMAGE_NAME}" \
    .
  rm -f "${LEGACY_DOCKERFILE}"
fi
