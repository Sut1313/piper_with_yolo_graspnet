#!/usr/bin/env python3
"""
inference_from_camera.py — GraspNet 相机连续推理脚本

从共享目录读取相机数据，运行 GraspNet 推理，实时显示抓取结果。

用法（在 Docker 容器内）：
  python inference_from_camera.py \
      --checkpoint_path /workspace/logs/log_rs/checkpoint.tar \
      --shared_dir /shared_camera_data

流程：
  1. 加载 GraspNet 模型
  2. 监听共享目录中的 .ready 标志
  3. 读取 color.png, depth.png, meta.mat
  4. 运行推理（点云采样 → 网络前传 → 抓取解码）
  5. 使用 Open3D 渲染标注抓取姿态的场景
  6. 显示在屏幕上（OpenCV imshow 窗口）
  7. 等待下一帧
"""

import os
import sys
import time
import argparse
import threading
import json

import numpy as np
import cv2
import open3d as o3d

import torch
from graspnetAPI import GraspGroup

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, 'models'))
sys.path.append(os.path.join(ROOT_DIR, 'dataset'))
sys.path.append(os.path.join(ROOT_DIR, 'utils'))

from graspnet import GraspNet, pred_decode
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image

# ── 参数 ──────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('--checkpoint_path', type=str, default='logs/log_rs/checkpoint.tar',
                    help='Model checkpoint path')
parser.add_argument('--shared_dir', type=str, default='/shared_camera_data',
                    help='Shared camera data directory')
parser.add_argument('--num_point', type=int, default=20000,
                    help='Point Number [default: 20000]')
parser.add_argument('--num_view', type=int, default=300,
                    help='View Number [default: 300]')
parser.add_argument('--collision_thresh', type=float, default=0.01,
                    help='Collision Threshold [default: 0.01]')
parser.add_argument('--voxel_size', type=float, default=0.01,
                    help='Voxel Size for collision detection [default: 0.01]')
parser.add_argument('--top_k', type=int, default=50,
                    help='Number of top grasps to show [default: 50]')
parser.add_argument('--poll_interval', type=float, default=0.1,
                    help='Poll interval in seconds [default: 0.1]')
parser.add_argument('--display_width', type=int, default=1280,
                    help='Display window width [default: 1280]')
parser.add_argument('--display_height', type=int, default=720,
                    help='Display window height [default: 720]')
parser.add_argument('--no_display', action='store_true',
                    help='Run without display (save results only)')
parser.add_argument('--min_object_points', type=int, default=80,
                    help='Minimum points required per colored object')
parser.add_argument('--grasp_object_margin', type=float, default=0.08,
                    help='Allowed margin around object point cloud for grasp validation [default: 0.08]')
parser.add_argument('--candidates_per_object', type=int, default=120,
                    help='Number of complete GraspNet 6D candidates to export per object')
parser.add_argument('--collision_candidate_limit', type=int, default=160,
                    help='Maximum NMS candidates sent to collision detection [default: 160]')
parser.add_argument('--seed', type=int, default=42,
                    help='Base random seed; frame index is added for repeatable retries [default: 42]')
parser.add_argument('--segmentation_source', choices=['yolo', 'color'], default='yolo',
                    help='Object segmentation source [default: yolo]')
parser.add_argument('--instances_filename', default='instances.npz',
                    help='YOLO instance-mask file in shared_dir [default: instances.npz]')
parser.add_argument('--camera_pose_file', default='camera_to_world.json',
                    help='Optional camera/world transform JSON in shared_dir; legacy constants are used when absent')
cfgs = parser.parse_args()

# ── 全局变量 ──────────────────────────────────────────────────────
running = True
frame_count = 0
inference_times = []
latest_result_image = None
display_lock = threading.Lock()

OBJECT_COLOR_RANGES = {
    # 各抓取物体的颜色分割阈值（RGB 归一化到 [0,1]）。
    # 平面（灰色平台）颜色落在所有区间之外，因此会被当作背景过滤掉；
    # 这是"平面过滤"能生效的根本原因（见 process_data / segment_object_points）。
    'red_box': {
        'lower': np.array([0.55, 0.00, 0.00], dtype=np.float32),
        'upper': np.array([1.01, 0.35, 0.35], dtype=np.float32),
    },
    'green_cylinder': {
        'lower': np.array([0.00, 0.45, 0.00], dtype=np.float32),
        'upper': np.array([0.35, 1.01, 0.35], dtype=np.float32),
    },
    'blue_cylinder': {
        'lower': np.array([0.00, 0.00, 0.45], dtype=np.float32),
        'upper': np.array([0.35, 0.35, 1.01], dtype=np.float32),
    },
}
OBJECT_NAMES = tuple(OBJECT_COLOR_RANGES.keys())

SOURCE_OBJECT_PRIORS_WORLD = {
    # 各物体的世界系源位置先验（物体在 Gazebo world 中的初始摆放位置）。
    # 用于过滤"落点距离物体太远"的候选抓取，降低误抓率；
    # 注意 z=0.124 是物体几何中心高度，抓取点应在物体附近。
    'red_box': np.array([0.45, -0.16, 0.124], dtype=np.float64),
    'green_cylinder': np.array([0.45, 0.02, 0.124], dtype=np.float64),
    'blue_cylinder': np.array([0.45, 0.17, 0.124], dtype=np.float64),
}
SOURCE_PRIOR_MAX_XY_DIST = 0.35  # 放宽源区域过滤，容忍机械臂碰撞导致的物体位移

CAMERA_TO_WORLD_R = np.array([
    # 相机系 → 世界系旋转矩阵（标定得到，55.9° 有效俯仰角）。
    # 注意本矩阵第二行符号与 CAMERA_TO_WORLD_ROTATION_R 不同——
    # 这个版本用于"平移/度量"用途（如距离计算）。
    [0.0, -0.82806033, 0.56063899],
    [-1.0, 0.0,         0.0       ],
    [0.0, -0.56063899, -0.82806033],
], dtype=np.float64)
# 姿态转换使用独立的旋转矩阵（保持原始 +1.0 符号，使 IK 可解）
CAMERA_TO_WORLD_ROTATION_R = np.array([
    # 专用于旋转变换的版本：第二行 +1.0（右手系语义），
    # 使转换后的 TCP 姿态能被 Piper IK 求到解（历史标定修复点）。
    [0.0, -0.82806033, 0.56063899],
    [1.0, 0.0,         0.0       ],
    [0.0, -0.56063899, -0.82806033],
], dtype=np.float64)
CAMERA_TO_WORLD_T = np.array([0.072, 0.021, 0.548], dtype=np.float64)


def load_camera_world_config():
    """Load scene-specific optical-frame extrinsics without breaking old worlds."""
    global CAMERA_TO_WORLD_R, CAMERA_TO_WORLD_ROTATION_R, CAMERA_TO_WORLD_T
    global SOURCE_OBJECT_PRIORS_WORLD, SOURCE_PRIOR_MAX_XY_DIST

    path = cfgs.camera_pose_file
    if not os.path.isabs(path):
        path = os.path.join(cfgs.shared_dir, path)
    if not os.path.exists(path):
        print(f"  camera/world config: legacy constants (missing {path})", flush=True)
        return

    try:
        with open(path, 'r', encoding='utf-8') as handle:
            data = json.load(handle)
        translation_r = np.asarray(data['camera_to_world_r'], dtype=np.float64)
        rotation_r = np.asarray(
            data.get('camera_to_world_rotation_r', data['camera_to_world_r']),
            dtype=np.float64,
        )
        translation = np.asarray(data['camera_to_world_t'], dtype=np.float64)
        if translation_r.shape != (3, 3) or rotation_r.shape != (3, 3):
            raise ValueError('camera rotation matrices must be 3x3')
        if translation.shape != (3,):
            raise ValueError('camera_to_world_t must contain three values')
        if not np.all(np.isfinite(translation_r)) or not np.all(np.isfinite(rotation_r)) \
                or not np.all(np.isfinite(translation)):
            raise ValueError('camera transform contains non-finite values')

        CAMERA_TO_WORLD_R = translation_r
        CAMERA_TO_WORLD_ROTATION_R = rotation_r
        CAMERA_TO_WORLD_T = translation

        priors = data.get('source_object_priors_world', {})
        for name, value in priors.items():
            prior = np.asarray(value, dtype=np.float64)
            if prior.shape != (3,) or not np.all(np.isfinite(prior)):
                raise ValueError(f'invalid source prior for {name!r}')
            SOURCE_OBJECT_PRIORS_WORLD[str(name)] = prior
        if 'source_prior_max_xy_dist' in data:
            SOURCE_PRIOR_MAX_XY_DIST = float(data['source_prior_max_xy_dist'])
        print(f"  camera/world config: {path}", flush=True)
    except Exception as exc:
        raise RuntimeError(f"Failed to load camera/world config {path}: {exc}") from exc


load_camera_world_config()
# GraspNet local +X is the gripper depth/approach direction. Piper's TCP uses
# +Z as the tool-forward direction for link6 offset compensation, so +Z must
# follow GraspNet +X; +Y remains the gripper width axis.
GRASPNET_TO_PIPER_TCP_R = np.array([
    # GraspNet 局部系 → Piper TCP 系的轴映射：
    #   GraspNet +X（夹爪进深/接近方向）→ Piper +Z（工具前进方向）
    #   GraspNet +Y（夹爪宽度轴）       → Piper +Y
    #   GraspNet +Z                     → Piper -X
    # 该映射是 C++ 主节点 grasp_frame_transform 的同一份常量的 Python 镜像，
    # 修改时必须两边同步（历史坑：不一致会导致抓取姿态整体旋转错误）。
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
], dtype=np.float64)
PIPER_TOOL_Z_OFFSET = 0.0
PIPER_MIN_GRASP_Z = 0.0
PIPER_MIN_LINK6_GRASP_Z = 0.0


# ===================================================================
#  模型加载
# ===================================================================
def get_net():
    """初始化并加载 GraspNet 模型"""
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}", flush=True)

    net = GraspNet(
        input_feature_dim=0,
        num_view=cfgs.num_view,
        num_angle=12,
        num_depth=4,
        cylinder_radius=0.05,
        hmin=-0.02,
        hmax_list=[0.01, 0.02, 0.03, 0.04],
        is_training=False,
    )
    net.to(device)

    checkpoint = torch.load(cfgs.checkpoint_path, map_location=device)
    net.load_state_dict(checkpoint['model_state_dict'])
    epoch = checkpoint['epoch']
    print(f"-> loaded checkpoint (epoch: {epoch})", flush=True)
    net.eval()
    return net


# ===================================================================
#  数据读取与处理
# ===================================================================
def read_data_from_shared(shared_dir):
    """从共享目录读取相机数据（color.png / depth.png / meta.mat）。

    调用时机：调用方已确认 .ready 存在（整帧已由 bridge 原子写完），
    因此这里的读取不会遇到半帧；返回 dict 或 (None, 错误信息)。
    """
    color_path = os.path.join(shared_dir, 'color.png')
    depth_path = os.path.join(shared_dir, 'depth.png')
    meta_path = os.path.join(shared_dir, 'meta.mat')

    if not all(os.path.exists(p) for p in [color_path, depth_path, meta_path]):
        return None, "Missing data files"

    try:
        color = cv2.imread(color_path, cv2.IMREAD_COLOR)
        if color is None:
            return None, "Failed to read color.png"
        # bridge 侧约定直接存 BGR（见 camera_bridge_node 的注释），
        # 所以这里必须做 BGR→RGB 转换一次且仅一次——
        # 若 bridge 侧也转了，这里就会转两次，红蓝互换（历史 bug）。
        color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        color_float = color.astype(np.float32) / 255.0

        depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        if depth is None:
            return None, "Failed to read depth.png"

        import scipy.io as scio
        meta = scio.loadmat(meta_path)
        intrinsic = meta['intrinsic_matrix']
        factor_depth = float(meta['factor_depth'][0][0])

        h, w = depth.shape
        camera = CameraInfo(
            float(w), float(h),
            float(intrinsic[0][0]), float(intrinsic[1][1]),
            float(intrinsic[0][2]), float(intrinsic[1][2]),
            factor_depth,
        )

        return {
            'color': color_float,
            'depth': depth.astype(np.float32),
            'camera': camera,
            'color_vis': color,
        }, None
    except Exception as e:
        return None, str(e)


def read_pick_state(shared_dir, allowed_names=None):
    """读取 pick_state.json（C++ 侧写入），返回 active 物体名列表。

    三态返回：
      None  — 文件缺失/损坏 → 回退全量识别（感知-only 流水线行为不变）
      []    — 无 pending 物体（全部已放置）→ 调用方跳过整帧
      list  — pending 名称；YOLO 模式动态匹配模型标签，颜色模式过滤旧键

    为什么只识别 pending（未放置）物体：C++ 主节点每成功抓取一个物体就把它
    从 pending 列表移除（写回本文件）。推理端据此跳过已放置物体，避免
    （1）对已被抓走的物体做无意义推理浪费时间；
    （2）机械臂停在放置区时被误识别为抓取目标（历史误检来源之一）。
    YOLO 模式不在此处写死类别；若旧规划器名称与新模型标签不相交，实例
    读取阶段会明确告警并处理全部 YOLO 标签，避免整条感知链被静默清空。
    """
    path = os.path.join(shared_dir, 'pick_state.json')
    try:
        with open(path) as f:
            state = json.load(f)
        # The executor has already captured a fresh result and is planning or
        # moving.  Consume any in-flight frame without another network pass.
        if state.get('perception_enabled') is False:
            return []
        active_target = state.get('active_target')
        if isinstance(active_target, str):
            if allowed_names is None or active_target in allowed_names:
                return [active_target]
        pending = state.get('pending')
        if not isinstance(pending, list):
            return None
        pending = [str(name) for name in pending]
        if allowed_names is None:
            # YOLO 类别由 ONNX/instances.npz 决定，不在这里写死。
            return pending
        # 颜色回退模式仅支持旧项目固定的三个颜色键。
        return [name for name in pending if name in allowed_names]
    except (OSError, ValueError):
        # 文件不存在/解析失败时按"全部待抓取"处理（感知-only 模式兼容）
        return None


def segment_object_points(cloud, color, active_names=None):
    """Return mask for Gazebo colored objects and per-object point counts.

    active_names=None 时识别全部物体（原行为）；否则只对未放置的
    active 物体做掩膜——已放置物体不再被识别和推理。
    """
    if active_names is None:
        active_names = list(OBJECT_COLOR_RANGES.keys())
    masks = {}
    object_mask = np.zeros(len(color), dtype=bool)

    for name in active_names:
        ranges = OBJECT_COLOR_RANGES[name]
        mask = np.all(color >= ranges['lower'], axis=1) & np.all(color <= ranges['upper'], axis=1)
        masks[name] = mask
        object_mask |= mask

    stats = {
        name: int(np.count_nonzero(mask))
        for name, mask in masks.items()
    }
    stats['total_object_points'] = int(np.count_nonzero(object_mask))
    stats['all_objects_visible'] = all(
        stats[name] >= cfgs.min_object_points
        for name in masks
    )

    return object_mask, masks, stats


def read_yolo_instance_masks(shared_dir, image_shape, active_names=None):
    """读取 YOLO 节点写出的二维实例掩膜，并按类别合并。

    掩膜仍与原始 RGB-D 像素一一对应；process_data 中用同一个 depth>0
    布尔索引将它投影到 organized 点云，因此不需要额外做 2D→3D 最近邻匹配。
    """
    path = os.path.join(shared_dir, cfgs.instances_filename)
    if not os.path.exists(path):
        return None, f"Missing YOLO instances file: {path}"
    try:
        with np.load(path, allow_pickle=False) as archive:
            masks = archive['masks'].astype(bool, copy=False)
            labels = archive['labels'].astype(str).tolist()
            scores = archive['scores'].astype(np.float32, copy=False)
        expected_h, expected_w = image_shape
        if masks.ndim != 3 or masks.shape[1:] != (expected_h, expected_w):
            return None, (
                f"YOLO mask shape mismatch: {masks.shape}, "
                f"expected (N, {expected_h}, {expected_w})"
            )
        if len(labels) != len(masks) or len(scores) != len(masks):
            return None, "YOLO masks/labels/scores length mismatch"

        detected_labels = set(labels)
        if active_names is None:
            enabled = detected_labels
        else:
            enabled = detected_labels.intersection(active_names)
            if active_names and not detected_labels:
                print(
                    "  WARNING: no YOLO instances detected for requested "
                    f"target(s): {active_names}",
                    flush=True,
                )
            elif active_names and not enabled:
                print(
                    "  WARNING: pick_state labels do not match YOLO labels; "
                    f"pending={active_names}, detected={sorted(detected_labels)}; "
                    "processing all YOLO labels",
                    flush=True,
                )
                enabled = detected_labels
        merged = {}
        confidence = {}
        for mask, label, score in zip(masks, labels, scores):
            if label not in enabled:
                continue
            if label not in merged:
                merged[label] = mask.copy()
                confidence[label] = float(score)
            else:
                merged[label] |= mask
                confidence[label] = max(confidence[label], float(score))
        print(
            "  YOLO instances: "
            + (" ".join(f"{name}={confidence[name]:.2f}" for name in sorted(merged)) or "none"),
            flush=True,
        )
        return merged, None
    except (OSError, KeyError, ValueError) as exc:
        return None, f"Failed to read YOLO instances: {exc}"


def object_centers_from_masks(cloud, object_masks):
    centers = {}
    for name, mask in object_masks.items():
        points = cloud[mask]
        if len(points) == 0:
            continue
        centers[name] = points.mean(axis=0).astype(float)
    return centers


def make_point_cloud(points, colors):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float32))
    cloud.colors = o3d.utility.Vector3dVector(colors.astype(np.float32))
    return cloud


def sample_points_by_instance(cloud, color, object_masks, num_point):
    """均衡采样各实例，避免大物体把较小物体的点全部挤出固定输入。"""
    visible = [name for name, name_mask in object_masks.items() if np.any(name_mask)]
    if not visible:
        return None, None
    base_count, remainder = divmod(num_point, len(visible))
    point_batches = []
    color_batches = []
    for index, name in enumerate(visible):
        source_indices = np.flatnonzero(object_masks[name])
        sample_count = base_count + (1 if index < remainder else 0)
        selected = np.random.choice(
            source_indices, sample_count, replace=len(source_indices) < sample_count)
        point_batches.append(cloud[selected])
        color_batches.append(color[selected])
    sampled_points = np.concatenate(point_batches, axis=0)
    sampled_colors = np.concatenate(color_batches, axis=0)
    permutation = np.random.permutation(len(sampled_points))
    return sampled_points[permutation], sampled_colors[permutation]


def atomic_imwrite(path, image):
    tmp_path = f"{path}.tmp.png"
    if not cv2.imwrite(tmp_path, image):
        raise RuntimeError(f"Failed to write image: {tmp_path}")
    os.replace(tmp_path, path)


def process_data(data, active_names=None, instance_masks=None):
    """将原始相机数据转换为 GraspNet 输入格式

    处理链：深度图反投影成点云 → 剔除无效深度点 → 颜色分割出物体点
    （同时滤掉灰色平台等背景）→ 只从物体点采样 num_point 个点喂给网络。
    active_names=None 时识别全部物体；否则只识别未放置（pending）的物体。
    """
    color = data['color']
    depth = data['depth']
    camera = data['camera']

    # 生成点云：用内参矩阵 + factor_depth 把 uint16 毫米深度图反投影成
    # organized 点云（与像素网格对齐，方便按颜色掩膜取点）。
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    # 有效点筛选（移除深度为0的点）：深度 0 表示"无有效测量"，
    # 反投影后位置无意义，必须剔除。
    mask = depth > 0
    cloud_masked = cloud[mask]
    color_masked = color[mask]

    if len(cloud_masked) == 0:
        return None, "No valid points in point cloud"

    print(f"  valid points: {len(cloud_masked)}", flush=True)

    if instance_masks is None:
        # 兼容旧项目：按 RGB 颜色阈值分割物体。
        object_mask, object_masks, object_stats = segment_object_points(
            cloud_masked, color_masked, active_names)
    else:
        # YOLO 的二维 mask 和 organized 点云共享同一像素网格。
        # 使用 depth>0 的同一个二维索引后，mask 与 cloud_masked 仍逐点对齐。
        # YOLO 标签从实例文件动态传入，使 cube/cuboid/cylinder 等训练类别
        # 原样进入 GraspNet 和 CSV，不再被旧颜色类别表限制。
        names = sorted(instance_masks)
        object_masks = {
            name: instance_masks.get(name, np.zeros(depth.shape, dtype=bool))[mask]
            for name in names
        }
        object_mask = np.zeros(len(cloud_masked), dtype=bool)
        for name_mask in object_masks.values():
            object_mask |= name_mask
        object_stats = {
            name: int(np.count_nonzero(name_mask))
            for name, name_mask in object_masks.items()
        }
        object_stats['total_object_points'] = int(np.count_nonzero(object_mask))
        object_stats['all_objects_visible'] = all(
            object_stats[name] >= cfgs.min_object_points for name in object_masks
        )
    object_centers_camera = object_centers_from_masks(cloud_masked, object_masks)
    object_points_by_name = {
        name: cloud_masked[mask]
        for name, mask in object_masks.items()
    }
    cloud_no_plane = cloud_masked[object_mask]
    color_no_plane = color_masked[object_mask]

    print(
        "  object points: "
        f"{object_stats['total_object_points']} "
        + " ".join(f"{n}={object_stats[n]}" for n in sorted(object_masks)),
        flush=True,
    )

    if len(cloud_no_plane) == 0:
        return None, "No segmented object points after applying instance masks"

    if not object_stats['all_objects_visible']:
        # 物体点不足只告警不报错：可能是部分遮挡/物体已移动，
        # 仍把当前能看到的物体点送入推理（尽力而为）。
        print(
            f"  WARNING: not all objects meet min_object_points={cfgs.min_object_points}",
            flush=True,
        )

    # ── 采样 ────────────────────────────────────────────────
    num_point = cfgs.num_point
    if len(cloud_no_plane) == 0:
        return None, "No object points available for sampling"
    # 固定 num_point 输入按可见实例均分配额；点数不足的实例有放回采样。
    # 这可避免像素面积最大的物体占满输入，使较小/较远物体仍能产生抓取候选。
    cloud_sampled, color_sampled = sample_points_by_instance(
        cloud_masked, color_masked, object_masks, num_point)
    if cloud_sampled is None:
        return None, "No visible instance points available for sampling"

    # scene_cloud = 全部有效点（供碰撞检测 / 渲染），
    # object_cloud = 仅物体点（供渲染 / 验证）。
    scene_cloud = make_point_cloud(cloud_masked, color_masked)
    object_cloud = make_point_cloud(cloud_no_plane, color_no_plane)

    # 转换为 Tensor（用于网络 — 仅物体点云）
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    cloud_tensor = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32)).to(device)

    end_points = {
        'point_clouds': cloud_tensor,
        'cloud_colors': color_sampled,
    }

    return {
        'end_points': end_points,
        'scene_cloud': scene_cloud,
        'object_cloud': object_cloud,
        'object_points': cloud_no_plane,
        'object_points_by_name': object_points_by_name,
        'object_masks': object_masks,
        'object_stats': object_stats,
        'object_centers_camera': object_centers_camera,
        'color_vis': data['color_vis'],
    }, None


# ===================================================================
#  推理
# ===================================================================
def run_inference(net, end_points):
    """运行 GraspNet 推理

    网络前传（无梯度）→ pred_decode 解码出原始抓取（约 300×12×4 个候选）
    → 转成 GraspGroup（统一封装，带 NMS/排序/可视化工具）。
    """
    with torch.no_grad():
        end_points_out = net(end_points)
        grasp_preds = pred_decode(end_points_out)
    gg_array = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(gg_array)
    print(f"  decoded grasps: {len(gg)}", flush=True)
    return gg


def collision_filter(gg, scene_cloud):
    """碰撞检测过滤：剔除与场景点云发生碰撞的抓取。

    用 ModelFreeCollisionDetector 在完整场景（含平台）上做无模型碰撞检测，
    把会扎进桌面/平台的抓取删掉，只剩可执行的候选。
    """
    t0 = time.time()
    mfcdetector = ModelFreeCollisionDetector(
        np.asarray(scene_cloud.points), voxel_size=cfgs.voxel_size)
    collision_mask = mfcdetector.detect(gg, approach_dist=0.05,
                                        collision_thresh=cfgs.collision_thresh)
    gg = gg[~collision_mask]
    print(f"  after collision filter: {len(gg)} ({time.time()-t0:.2f}s)", flush=True)
    return gg


def prefilter_collision_candidates(gg, processed):
    """Discard duplicate and off-object grasps before expensive collision tests."""
    if len(gg) == 0:
        return gg
    before = len(gg)
    gg.nms()
    gg.sort_by_score()
    translations = np.asarray(gg.translations)
    keep = np.zeros(len(gg), dtype=bool)
    for object_points in processed['object_points_by_name'].values():
        if object_points is None or len(object_points) == 0:
            continue
        bbox_min = object_points.min(axis=0) - cfgs.grasp_object_margin
        bbox_max = object_points.max(axis=0) + cfgs.grasp_object_margin
        keep |= np.all((translations >= bbox_min) & (translations <= bbox_max), axis=1)
    gg = gg[keep]
    limit = max(1, int(cfgs.collision_candidate_limit))
    if len(gg) > limit:
        gg = gg[:limit]
    print(f"  before collision prefilter: {before} -> {len(gg)}", flush=True)
    return gg


def validate_grasps(gg, object_points):
    """Check whether final grasp centers are near the segmented object cloud."""
    result = {
        'grasp_count': int(len(gg)),
        'centers_inside_object_bbox': 0,
        'best_grasp_inside_object_bbox': False,
        'object_bbox_min': None,
        'object_bbox_max': None,
        'best_grasp_center': None,
    }

    if len(object_points) == 0:
        return result

    bbox_min = object_points.min(axis=0) - cfgs.grasp_object_margin
    bbox_max = object_points.max(axis=0) + cfgs.grasp_object_margin
    result['object_bbox_min'] = bbox_min.astype(float).tolist()
    result['object_bbox_max'] = bbox_max.astype(float).tolist()

    if len(gg) == 0:
        return result

    translations = np.asarray(gg.translations)
    inside = np.all((translations >= bbox_min) & (translations <= bbox_max), axis=1)
    result['centers_inside_object_bbox'] = int(np.count_nonzero(inside))
    result['best_grasp_inside_object_bbox'] = bool(inside[0])
    result['best_grasp_center'] = translations[0].astype(float).tolist()
    return result


def select_object_grasp_targets(gg, processed):
    """Select GraspNet 6D grasp candidates for every segmented object class.

    为每个物体挑选候选抓取：
    1. 对全部抓取做 NMS（去重）+ 按分数排序；
    2. 用物体点云包围盒（加 grasp_object_margin 余量）筛选"落在该物体上"的抓取；
    3. 逐个检查世界系 XY 位置是否在物体源区域先验附近（防误抓）;
    4. 综合"网络分数 - 0.25×离源距离"排序，输出每个物体前
       candidates_per_object 个完整 6D 候选（含调试诊断字段）。
    """
    targets = {}
    if len(gg) == 0:
        return targets

    gg.nms()
    gg.sort_by_score()
    # grasp 数组布局（16 列）：
    #   [0]=score, [1..3]=width/height/depth, [4..12]=3x3 旋转(行主序),
    #   [13..15]=平移(camera 系)
    grasp_array = np.asarray(gg.grasp_group_array)
    translations = grasp_array[:, 13:16]

    for name, mask in processed['object_masks'].items():
        object_points = processed['object_points_by_name'].get(name)
        if object_points is None or len(object_points) == 0:
            continue

        # 包围盒过滤：抓取中心必须在物体点云包围盒（带余量）内，
        # 否则视为"抓空了"的候选。
        bbox_min = object_points.min(axis=0) - cfgs.grasp_object_margin
        bbox_max = object_points.max(axis=0) + cfgs.grasp_object_margin
        inside = np.all((translations >= bbox_min) & (translations <= bbox_max), axis=1)
        candidate_idxs = np.nonzero(inside)[0]
        if len(candidate_idxs) == 0:
            continue

        ranked = []
        for idx in candidate_idxs:
            grasp = grasp_array[int(idx)]
            translation = grasp[13:16].astype(np.float64)
            rotation = grasp[4:13].reshape(3, 3).astype(np.float64)
            # 相机系 → 世界系（平移用 CAMERA_TO_WORLD_R，
            # 旋转用 CAMERA_TO_WORLD_ROTATION_R，见上方常量注释）
            world_translation = CAMERA_TO_WORLD_R @ translation + CAMERA_TO_WORLD_T
            # 源区域先验过滤：世界系 XY 离物体初始位置太远的抓取直接丢弃
            # （通常是误检或机械臂把物体撞飞后产生的假候选）。
            source_prior = SOURCE_OBJECT_PRIORS_WORLD.get(name)
            source_dist_xy = 0.0
            if source_prior is not None:
                source_dist_xy = float(np.linalg.norm(world_translation[:2] - source_prior[:2]))
                if source_dist_xy > SOURCE_PRIOR_MAX_XY_DIST:
                    continue
            # 完整姿态链：camera 系旋转 → 世界系 → GraspNet→Piper TCP 轴映射
            world_tcp_rotation = CAMERA_TO_WORLD_ROTATION_R @ rotation @ GRASPNET_TO_PIPER_TCP_R
            tcp_forward_z = float(world_tcp_rotation[2, 2])  # TCP 前进方向(世界Z)分量，诊断用
            grasp_tcp_z = float(world_translation[2])
            link6_grasp_z = grasp_tcp_z  # 当前 TCP 与 link6 无 Z 偏移（PIPER_TOOL_Z_OFFSET=0）
            # 排序分 = 网络分数 - 0.25×离源距离：同等分数下优先抓"原位附近"的物体
            rank_score = (
                float(grasp[0])
                - 0.25 * source_dist_xy
            )
            ranked.append((rank_score, int(idx), link6_grasp_z, tcp_forward_z, source_dist_xy))

        if len(ranked) == 0:
            continue

        ranked.sort(reverse=True)
        center = processed['object_centers_camera'].get(name)
        candidates = []
        for rank_score, idx, link6_grasp_z, tcp_forward_z, source_dist_xy in ranked[:cfgs.candidates_per_object]:
            grasp = grasp_array[idx]
            candidates.append({
                'center': center,
                'count': int(processed['object_stats'].get(name, 0)),
                'grasp': grasp,
                'rank_score': float(rank_score),
                'link6_grasp_z': float(link6_grasp_z),
                'tcp_forward_z': float(tcp_forward_z),
                'source_dist_xy': float(source_dist_xy),
            })
        targets[name] = candidates

    return targets


def write_frame_validation(shared_dir, frame_idx, processed, gg, grasp_validation):
    """把本帧诊断与抓取候选写入共享目录，供 C++ 主节点 / 验收脚本使用。

    输出三份文件：
      last_validation.json   — 完整诊断（物体点数、中心、每候选的 6D 位姿等）
      perception_targets.csv — 物体中心（旧接口，C++ 早期版本用）
      perception_grasps.csv  — 抓取候选主输出：C++ graspnet_6d_grasp_planner
                               读取并转成 TCP 目标执行抓取。
    所有文件都先写 .tmp 再 rename（原子写），防止 C++ 侧读到半行。
    """
    grasp_targets = select_object_grasp_targets(gg, processed)
    output_timestamp = time.time()
    output = {
        'frame': int(frame_idx),
        'timestamp': output_timestamp,
        'active_objects': sorted(processed['object_masks']),
        'valid_scene_points': int(len(processed['scene_cloud'].points)),
        'object_stats': processed['object_stats'],
        'object_centers_camera': {
            name: center.tolist()
            for name, center in processed['object_centers_camera'].items()
        },
        'plane_filtered_out_points': int(
            len(processed['scene_cloud'].points) - len(processed['object_cloud'].points)
        ),
        'grasp_validation': grasp_validation,
        'per_object_grasps': {
            name: [
                {
                    'score': float(target['grasp'][0]),
                    'rank_score': float(target['rank_score']),
                    'width': float(target['grasp'][1]),
                    'height': float(target['grasp'][2]),
                    'depth': float(target['grasp'][3]),
                    'rotation_matrix': target['grasp'][4:13].reshape(3, 3).astype(float).tolist(),
                    'translation': target['grasp'][13:16].astype(float).tolist(),
                    'link6_grasp_z': float(target['link6_grasp_z']),
                    'tcp_forward_z': float(target['tcp_forward_z']),
                    'source_dist_xy': float(target['source_dist_xy']),
                    'object_center_camera': target['center'].astype(float).tolist()
                    if target['center'] is not None else None,
                    'count': int(target['count']),
                }
                for target in targets
            ]
            for name, targets in grasp_targets.items()
        },
    }
    path = os.path.join(shared_dir, 'last_validation.json')
    tmp_json_path = f"{path}.tmp"
    with open(tmp_json_path, 'w') as f:
        json.dump(output, f, indent=2)
    os.replace(tmp_json_path, path)

    # perception_targets.csv：每个物体的相机系中心（旧接口，仅保留兼容）
    csv_path = os.path.join(shared_dir, 'perception_targets.csv')
    tmp_path = f"{csv_path}.tmp"
    with open(tmp_path, 'w') as f:
        f.write('name,camera_x,camera_y,camera_z,count\n')
        for name in sorted(processed['object_masks']):
            center = processed['object_centers_camera'].get(name)
            if center is None:
                continue
            count = processed['object_stats'].get(name, 0)
            f.write(
                f"{name},{center[0]:.9f},{center[1]:.9f},{center[2]:.9f},{count}\n"
            )
    os.replace(tmp_path, csv_path)

    # perception_grasps.csv：抓取候选主输出（每行一个完整 6D 候选）。
    # 列布局与 C++ graspnet_6d_grasp_planner 的解析代码严格对应，
    # 修改列顺序时必须同步改 C++（历史坑：列错位导致抓取位姿完全错误）。
    grasp_csv_path = os.path.join(shared_dir, 'perception_grasps.csv')
    tmp_grasp_path = f"{grasp_csv_path}.tmp"
    header = [
        'name', 'camera_x', 'camera_y', 'camera_z', 'count',
        'score', 'width', 'height', 'depth',
        'r00', 'r01', 'r02', 'r10', 'r11', 'r12', 'r20', 'r21', 'r22',
        'center_camera_x', 'center_camera_y', 'center_camera_z',
        'link6_grasp_z', 'tcp_forward_z',
        'source_dist_xy',
        'frame_sequence', 'timestamp',
    ]
    with open(tmp_grasp_path, 'w') as f:
        f.write(','.join(header) + '\n')
        for name in sorted(processed['object_masks']):
            targets = grasp_targets.get(name)
            if not targets:
                continue
            for target in targets:
                grasp = target['grasp']
                center = target['center']
                rotation = grasp[4:13]
                translation = grasp[13:16]
                row = [
                    name,
                    f"{translation[0]:.9f}",
                    f"{translation[1]:.9f}",
                    f"{translation[2]:.9f}",
                    str(target['count']),
                    f"{grasp[0]:.9f}",
                    f"{grasp[1]:.9f}",
                    f"{grasp[2]:.9f}",
                    f"{grasp[3]:.9f}",
                    *[f"{v:.9f}" for v in rotation],
                    f"{center[0]:.9f}" if center is not None else "nan",
                    f"{center[1]:.9f}" if center is not None else "nan",
                    f"{center[2]:.9f}" if center is not None else "nan",
                    f"{target['link6_grasp_z']:.9f}",
                    f"{target['tcp_forward_z']:.9f}",
                    f"{target['source_dist_xy']:.9f}",
                    str(int(frame_idx)),
                    f"{output_timestamp:.9f}",
                ]
                f.write(','.join(row) + '\n')
    os.replace(tmp_grasp_path, grasp_csv_path)


# ===================================================================
#  可视化 — 使用 Open3D 离屏渲染 + OpenCV 显示
# ===================================================================
def render_result(gg, scene_cloud, object_cloud, color_vis, object_stats=None, grasp_validation=None):
    """将点云和抓取姿态渲染为图像

    用 Open3D 离屏渲染器把场景点云 + 物体点云 + 前 top_k 个抓取姿态
    画成一张图，叠加文字信息后返回给显示线程（OpenCV/matplotlib 窗口）。
    渲染失败时回退到纯彩色图并标注错误（不中断主循环）。
    """
    if len(gg) == 0:
        print("  No grasps to visualize", flush=True)
        # 没有抓取，直接返回彩色图
        img = color_vis.copy()
        cv2.putText(img, "No grasps detected", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return img

    gg.nms()
    gg.sort_by_score()
    top_gg = gg[:cfgs.top_k]
    print(f"  showing top {len(top_gg)} grasps", flush=True)

    try:
        grippers = top_gg.to_open3d_geometry_list()
        W, H = cfgs.display_width, cfgs.display_height

        renderer = o3d.visualization.rendering.OffscreenRenderer(W, H)
        renderer.scene.set_background([0.9, 0.9, 0.9, 1.0])

        # 添加完整场景点云（浅色）和非平面物体点云（原色）
        mat_cloud = o3d.visualization.rendering.MaterialRecord()
        mat_cloud.shader = "defaultUnlit"
        renderer.scene.add_geometry("cloud", scene_cloud, mat_cloud)

        mat_object = o3d.visualization.rendering.MaterialRecord()
        mat_object.shader = "defaultUnlit"
        renderer.scene.add_geometry("object_cloud", object_cloud, mat_object)

        # 添加抓取姿态
        for i, g in enumerate(grippers):
            mat_grip = o3d.visualization.rendering.MaterialRecord()
            mat_grip.shader = "defaultUnlit"
            renderer.scene.add_geometry(f"gripper_{i}", g, mat_grip)

        # auto fit camera view - Open3D 0.17 API: setup_camera(fov, center, eye, up)
        # 注意：Open3D 0.17 要求 float32 类型（不接收 float64，历史踩坑点）
        # 注意：点云在相机坐标系中（+Z 向前，+Y 向下），
        #       所以 up 方向是 -Y，eye 应在场景后上方
        bbox = renderer.scene.bounding_box
        if bbox is not None:
            center = bbox.get_center().astype(np.float32)
            # 根据场景大小计算合适的视角偏移
            extent = (bbox.get_max_bound() - bbox.get_min_bound()).astype(np.float32)
            offset = max(np.max(extent) * 0.5, 0.35)
            # eye: 在场景后上方 (-Z 向后, -Y 向上)
            eye = (center + np.array([0.0, -offset, -offset], dtype=np.float32)).astype(np.float32)
            up = np.array([0.0, -1.0, 0.0], dtype=np.float32)  # 相机坐标系中 -Y 是"上"
            renderer.setup_camera(60.0, center, eye, up)

        img = renderer.render_to_image()
        # Open3D Image → numpy → BGR for OpenCV
        result = np.asarray(img)
        if result.dtype == np.uint8 and len(result.shape) == 3:
            result = cv2.cvtColor(result, cv2.COLOR_RGB2BGR)
        else:
            result = result.copy()

        # 添加信息覆盖层
        cv2.putText(result, f"Grasps: {len(top_gg)}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 2)
        cv2.putText(result, f"Frame: {frame_count}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)
        if object_stats:
            cv2.putText(
                result,
                f"Object pts: " + " ".join(
                    f"{n}={object_stats.get(n, 0)}"
                    for n in sorted(object_stats)
                    if n not in ('total_object_points', 'all_objects_visible')),
                (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (60, 60, 60),
                1,
            )
        if grasp_validation:
            cv2.putText(
                result,
                f"Grasp bbox hits: {grasp_validation['centers_inside_object_bbox']} final grasps",
                (10, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (60, 60, 60),
                1,
            )

        return result
    except Exception as e:
        print(f"  Render error: {e}", flush=True)
        # 回退到彩色图
        img = color_vis.copy()
        cv2.putText(img, f"Render error: {e}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        return img


def display_loop():
    """在单独的线程中运行显示循环

    将渲染结果实时显示在 matplotlib 窗口中（Docker 内 X11 透传），
    同时保存为 result.png 供主机的 result_viewer.py 使用。

    注：Docker 镜像中的 OpenCV 3.4.11 无 GTK 支持（GTK+: NO），
    因此使用 matplotlib TkAgg 后端替代 cv2.imshow。
    """
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt

    global running, latest_result_image, frame_count

    # 创建 matplotlib 窗口
    plt.ion()
    fig, ax = plt.subplots(figsize=(cfgs.display_width / 100,
                                    cfgs.display_height / 100))
    ax.axis('off')
    fig.canvas.manager.set_window_title("GraspNet - Perception Results")
    # 窗口置顶（X11 特定）
    try:
        fig.canvas.manager.window.attributes('-topmost', True)
    except Exception:
        pass

    # 预创建一个空显示对象
    dummy = np.zeros((cfgs.display_height, cfgs.display_width, 3),
                     dtype=np.uint8)
    img_display = ax.imshow(dummy)

    while running:
        if latest_result_image is not None:
            with display_lock:
                img = latest_result_image.copy() if latest_result_image is not None else None
            if img is not None:
                try:
                    # matplotlib 用 RGB，OpenCV 用 BGR
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    img_display.set_data(img_rgb)
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                    # 同时保存为文件供主机 viewer 使用
                    result_path = os.path.join(cfgs.shared_dir, 'result.png')
                    atomic_imwrite(result_path, img)
                except Exception as e:
                    print(f"  Display error: {e}", flush=True)
        else:
            plt.pause(0.1)

    plt.close(fig)


# ===================================================================
#  就绪标志管理
# ===================================================================
def wait_for_new_frame(shared_dir):
    """等待新的 .ready 标志出现（与 camera_bridge_node 的握手协议）。

    轮询 .ready 文件存在性（poll_interval=0.1s）。.ready 由 bridge 在本帧
    所有数据落盘后最后写入（原子写），所以"出现"即代表"整帧就绪"。
    存在竞态：bridge 可能在我们 remove 之前又写下一帧的 .ready——
    但由于数据处理是顺序的，最多丢一帧，不会读到半帧。
    """
    ready_path = os.path.join(shared_dir, '.ready')
    while running:
        if os.path.exists(ready_path):
            return True
        time.sleep(cfgs.poll_interval)
    return False


def clear_ready_flag(shared_dir):
    """清除就绪标志：处理完本帧后删除 .ready，告诉 bridge"帧已消费"。

    删除失败（OSError，如文件刚被覆盖）直接忽略——下次 wait 会等到新的
    .ready，握手机制对偶发丢帧是自愈的。
    """
    ready_path = os.path.join(shared_dir, '.ready')
    try:
        if os.path.exists(ready_path):
            os.remove(ready_path)
    except OSError:
        pass


# ===================================================================
#  主循环
# ===================================================================
def main():
    global running, frame_count, latest_result_image, inference_times

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print(f"=" * 60, flush=True)
    print(f"GraspNet Continuous Inference from Camera", flush=True)
    print(f"  checkpoint:  {cfgs.checkpoint_path}", flush=True)
    print(f"  shared_dir:  {cfgs.shared_dir}", flush=True)
    print(f"  segmentation:{cfgs.segmentation_source}", flush=True)
    print(f"  device:      {device}", flush=True)
    print(f"  display:     {'ON' if not cfgs.no_display else 'OFF'}", flush=True)
    print(f"=" * 60, flush=True)

    # 加载网络
    t0 = time.time()
    net = get_net()
    print(f"Model loaded in {time.time()-t0:.1f}s", flush=True)

    # 启动显示线程
    if not cfgs.no_display:
        disp_thread = threading.Thread(target=display_loop, daemon=True)
        disp_thread.start()
        print("Display thread started", flush=True)

    # 确保共享目录存在
    os.makedirs(cfgs.shared_dir, exist_ok=True)

    print(f"Waiting for first frame in {cfgs.shared_dir}...", flush=True)

    # ── 推理主循环 ─────────────────────────────────────────────
    # 每轮：等 .ready → 读 pick_state → 读数据 → 处理 → 推理 → 碰撞过滤
    # → 写 CSV/诊断 → 渲染 → 删 .ready（握手闭环）。
    # 任何一步出错都只跳过本帧（clear_ready_flag 后 continue），不退出进程，
    # 保证流水线在偶发坏帧下持续运行。
    last_active = None
    while running:
        # 1. 等待新帧（与 bridge 的 .ready 握手）
        if not wait_for_new_frame(cfgs.shared_dir):
            break

        frame_count += 1
        t_start = time.time()
        print(f"\n--- Frame {frame_count} ---", flush=True)

        # 读取感知状态（pick_state.json）：只识别尚未放置的物体
        # （状态过滤的历史动机：减少误检、跳过已放置物体，见 read_pick_state）
        allowed_state_names = OBJECT_NAMES if cfgs.segmentation_source == 'color' else None
        active_names = read_pick_state(cfgs.shared_dir, allowed_state_names)
        if active_names is None and cfgs.segmentation_source == 'color':
            active_names = list(OBJECT_NAMES)
        if active_names != last_active:
            print(f"  pick_state: active={active_names}", flush=True)
            last_active = active_names

        try:
            # Keep retries varied but reproducible.  Gating ensures frame_count
            # now advances only for explicit perception requests.
            frame_seed = int(cfgs.seed) + frame_count
            np.random.seed(frame_seed)
            torch.manual_seed(frame_seed)
            if torch.cuda.is_available():
                torch.cuda.manual_seed_all(frame_seed)

            # 全部已放置：跳过整帧（保留上一帧 CSV），避免空识别刷屏
            if active_names == []:
                print("  No pending objects (all placed); skipping frame.", flush=True)
                clear_ready_flag(cfgs.shared_dir)
                continue

            # 2. 读取数据
            data, err = read_data_from_shared(cfgs.shared_dir)
            if err:
                print(f"  Read error: {err}", flush=True)
                clear_ready_flag(cfgs.shared_dir)
                continue

            # 3. 读取 YOLO 二维实例掩膜，并用像素对应关系裁剪三维点云。
            instance_masks = None
            if cfgs.segmentation_source == 'yolo':
                instance_masks, err = read_yolo_instance_masks(
                    cfgs.shared_dir, data['depth'].shape, active_names)
                if err:
                    print(f"  Instance-mask error: {err}", flush=True)
                    clear_ready_flag(cfgs.shared_dir)
                    continue

            # 4. 处理数据（只保留 active 物体的点云）
            processed, err = process_data(data, active_names, instance_masks)
            if err:
                print(f"  Process error: {err}", flush=True)
                clear_ready_flag(cfgs.shared_dir)
                continue

            # 5. 运行推理
            t2 = time.time()
            gg = run_inference(net, processed['end_points'])
            infer_time = time.time() - t2

            # 6. 碰撞过滤
            gg = prefilter_collision_candidates(gg, processed)
            if cfgs.collision_thresh > 0:
                t3 = time.time()
                gg = collision_filter(gg, processed['scene_cloud'])
                col_time = time.time() - t3
            else:
                col_time = 0

            grasp_validation = validate_grasps(gg, processed['object_points'])
            write_frame_validation(cfgs.shared_dir, frame_count, processed, gg, grasp_validation)

            # 7. 渲染并显示（渲染在推理主线程，显示在 display_loop 线程，
            # 通过 latest_result_image + display_lock 交接，避免跨线程竞争）
            if not cfgs.no_display:
                result_img = render_result(gg, processed['scene_cloud'],
                                           processed['object_cloud'],
                                           processed['color_vis'],
                                           processed['object_stats'],
                                           grasp_validation)
                with display_lock:
                    latest_result_image = result_img

            # 8. 保存结果（grasps_XXXX.npy / scene_XXXX.pcd / objects_XXXX.pcd，
            # 供主机端 open3d_result_viewer.py 复盘）
            output_dir = os.path.join(cfgs.shared_dir, 'output')
            os.makedirs(output_dir, exist_ok=True)
            if len(gg) > 0:
                gg.save_npy(os.path.join(output_dir, f'grasps_{frame_count:04d}.npy'))
                o3d.io.write_point_cloud(
                    os.path.join(output_dir, f'scene_{frame_count:04d}.pcd'),
                    processed['scene_cloud'])
                o3d.io.write_point_cloud(
                    os.path.join(output_dir, f'objects_{frame_count:04d}.pcd'),
                    processed['object_cloud'])

            # 统计
            total_time = time.time() - t_start
            inference_times.append(infer_time)
            avg_time = sum(inference_times[-50:]) / max(len(inference_times[-50:]), 1)
            print(f"  ├─ inference: {infer_time:.2f}s", flush=True)
            print(f"  ├─ collision: {col_time:.2f}s", flush=True)
            print(f"  ├─ total:     {total_time:.2f}s", flush=True)
            print(f"  └─ avg inf:  {avg_time:.2f}s (last {min(len(inference_times),50)})", flush=True)
            print(f"  ─> {len(gg)} grasp(s) found", flush=True)

        except Exception as e:
            print(f"  ERROR in inference loop: {e}", flush=True)
            import traceback
            traceback.print_exc()

        # 9. 清除标志 → 等待下一帧
        clear_ready_flag(cfgs.shared_dir)

    # ── 清理 ───────────────────────────────────────────────────
    print(f"\n{'='*60}", flush=True)
    print(f"Inference stopped after {frame_count} frames", flush=True)
    if inference_times:
        print(f"Average inference time: {np.mean(inference_times):.3f}s", flush=True)
        print(f"FPS: {1.0/np.mean(inference_times):.1f}", flush=True)

    if not cfgs.no_display:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user", flush=True)
        running = False
    finally:
        running = False
        if not cfgs.no_display:
            cv2.destroyAllWindows()
