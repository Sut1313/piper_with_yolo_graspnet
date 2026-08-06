#!/usr/bin/env python3
"""
Open3D viewer for the latest GraspNet camera inference result.

Reads shared_camera_data/output/{objects_XXXX.pcd, grasps_XXXX.npy} and shows
the segmented object cloud plus final grasp poses in a local Open3D window.
"""

import argparse
import glob
import os
import signal
import time

import numpy as np
import open3d as o3d


PROJECT_DIR = os.path.expanduser("~/piper_graspnet")
OUTPUT_DIR = os.path.join(PROJECT_DIR, "shared_camera_data", "output")
RUNNING = True


def latest_pair(output_dir):
    """从 output 目录中找最新的一对 (objects_*.pcd, grasps_*.npy)。

    按 grasps_*.npy 的修改时间倒序找，并校验同名序号（suffix）的
    objects_*.pcd 是否存在——只返回"成对"的结果，防止单独文件造成空显示。
    """
    grasp_files = sorted(
        glob.glob(os.path.join(output_dir, "grasps_*.npy")),
        key=os.path.getmtime,
        reverse=True,
    )
    for grasp_path in grasp_files:
        suffix = os.path.basename(grasp_path).split("_", 1)[1].rsplit(".", 1)[0]
        object_path = os.path.join(output_dir, f"objects_{suffix}.pcd")
        if os.path.exists(object_path):
            return object_path, grasp_path
    return None, None


def make_grasp_lines(grasp, color):
    """把一个 16 列 grasp 行画成夹爪线框（LineSet）。

    grasp 数组布局：[0]=score, [1..3]=width/height/depth,
    [4..12]=3x3 旋转（行主序）, [13..15]=平移（相机系）。
    在 GraspNet 局部系中：+X 是夹爪进深/接近方向，+Y 是宽度轴，
    线框由 8 个局部点经 旋转+平移 变换到相机系后连线而成。
    """
    width = max(float(grasp[1]), 0.02)
    height = max(float(grasp[2]), 0.015)
    depth = max(float(grasp[3]), 0.02)
    rotation = grasp[4:13].reshape(3, 3)
    translation = grasp[13:16]

    half_w = width * 0.5
    base = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    left_inner = np.array([0.0, -half_w, 0.0], dtype=np.float64)
    right_inner = np.array([0.0, half_w, 0.0], dtype=np.float64)
    left_tip = np.array([depth, -half_w, 0.0], dtype=np.float64)
    right_tip = np.array([depth, half_w, 0.0], dtype=np.float64)
    left_top = np.array([depth, -half_w, height], dtype=np.float64)
    right_top = np.array([depth, half_w, height], dtype=np.float64)
    # GraspNet gripper depth/approach is local +X. Draw the approach marker in
    # the same direction as the finger tips so the Open3D view matches execution.
    # （接近方向标记沿 +X，与真实执行时夹爪进深方向一致）
    approach = np.array([depth * 0.7, 0.0, 0.0], dtype=np.float64)

    local_points = np.vstack([
        base, left_inner, right_inner, left_tip, right_tip, left_top, right_top, approach
    ])
    points = (rotation @ local_points.T).T + translation
    lines = [
        [1, 2],
        [1, 3],
        [2, 4],
        [3, 5],
        [4, 6],
        [5, 6],
        [0, 7],
        [0, 1],
        [0, 2],
    ]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([color] * len(lines))
    return line_set


def load_geometries(object_path, grasp_path, top_k):
    """加载物体点云与抓取，构建 Open3D 几何列表。

    返回 (geometries, grasps)：geometries 含 点云 + 坐标轴 + 每个抓取的
    夹爪线框（第一个抓取红色高亮，其余按排名从橙渐变为蓝）。
    """
    cloud = o3d.io.read_point_cloud(object_path)
    if cloud.is_empty():
        raise RuntimeError(f"Object point cloud is empty: {object_path}")

    grasps = np.load(grasp_path)
    if grasps.ndim != 2 or grasps.shape[1] < 16:
        raise RuntimeError(f"Unexpected grasp array shape: {grasps.shape}")

    # 按分数（第 0 列）降序取前 top_k 个抓取
    order = np.argsort(-grasps[:, 0])
    grasps = grasps[order[:top_k]]

    geometries = [cloud]
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.08)
    geometries.append(frame)

    for idx, grasp in enumerate(grasps):
        if idx == 0:
            color = [1.0, 0.1, 0.0]
        else:
            t = idx / max(len(grasps) - 1, 1)
            color = [0.35 + 0.55 * (1.0 - t), 0.0, 1.0]
        geometries.append(make_grasp_lines(grasp, color))

    return geometries, grasps


def run_window(geometries):
    """创建 Open3D 窗口并进入渲染循环（Ctrl+C/SIGTERM 退出）。

    相机预置视角与推理端渲染保持一致（正前方偏上俯视、向上 -Y），
    便于对照屏幕上的推理结果做人工复盘。
    """
    global RUNNING

    def _handle_signal(_sig, _frame):
        global RUNNING
        RUNNING = False

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    vis = o3d.visualization.Visualizer()
    if not vis.create_window(
        window_name="Piper GraspNet Open3D Result",
        width=1280,
        height=720,
        visible=True,
    ):
        raise RuntimeError("Failed to create Open3D window")

    for geometry in geometries:
        vis.add_geometry(geometry)

    render = vis.get_render_option()
    render.background_color = np.asarray([0.92, 0.92, 0.92])
    render.point_size = 5.0
    render.line_width = 3.0

    view = vis.get_view_control()
    view.set_front([0.0, -0.45, -1.0])
    view.set_lookat([0.0, 0.0, 0.55])
    view.set_up([0.0, -1.0, 0.0])
    view.set_zoom(0.75)

    while RUNNING:
        if not vis.poll_events():
            break
        vis.update_renderer()
        time.sleep(0.03)

    vis.destroy_window()


def main():
    """入口：找到最新结果对（可 --wait 阻塞等待），加载并在 Open3D 窗口显示，
    打印最佳抓取的完整 16 列原始数据便于对照 C++ 侧解析结果。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--output_dir", default=OUTPUT_DIR)
    parser.add_argument("--top_k", type=int, default=50)
    parser.add_argument("--wait", action="store_true", help="Wait until a result pair exists")
    args = parser.parse_args()

    object_path = grasp_path = None
    while object_path is None:
        object_path, grasp_path = latest_pair(args.output_dir)
        if object_path or not args.wait:
            break
        print(f"Waiting for GraspNet output in {args.output_dir}...")
        time.sleep(1.0)

    if object_path is None:
        raise SystemExit(f"No objects_*.pcd + grasps_*.npy pair found in {args.output_dir}")

    geometries, grasps = load_geometries(object_path, grasp_path, args.top_k)
    print(f"Showing: {object_path}")
    print(f"Grasps: {len(grasps)} top poses from {grasp_path}")
    print("Best grasp:", grasps[0, :16].tolist() if len(grasps) else "none")

    run_window(geometries)


if __name__ == "__main__":
    main()
