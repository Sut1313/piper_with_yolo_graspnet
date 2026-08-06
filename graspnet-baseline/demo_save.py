"""Run GraspNet demo inference and save results to files.

This variant does not open an Open3D window and does not require Xvfb.
It writes point clouds / meshes to an output directory that can be mounted
from the host.
"""

import argparse
import os
import sys
import time

import numpy as np
import open3d as o3d
import scipy.io as scio
import torch
from PIL import Image
from graspnetAPI import GraspGroup

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, "models"))
sys.path.append(os.path.join(ROOT_DIR, "dataset"))
sys.path.append(os.path.join(ROOT_DIR, "utils"))

from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image
from graspnet import GraspNet, pred_decode


parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint_path", required=True, help="Model checkpoint path")
parser.add_argument("--data_dir", default="doc/example_data", help="Input RGB-D example directory")
parser.add_argument("--output_dir", default="outputs", help="Directory for saved outputs")
parser.add_argument("--num_point", type=int, default=20000, help="Point number")
parser.add_argument("--num_view", type=int, default=300, help="View number")
parser.add_argument(
    "--collision_thresh",
    type=float,
    default=-1,
    help="Collision threshold. Use -1 to skip slow CPU collision detection.",
)
parser.add_argument("--voxel_size", type=float, default=0.01, help="Collision detector voxel size")
parser.add_argument("--top_k", type=int, default=50, help="Number of grasps to save as geometry")
cfgs = parser.parse_args()


def log(message):
    print(message, flush=True)


def get_net():
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
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net.to(device)
    checkpoint = torch.load(cfgs.checkpoint_path, map_location=device)
    net.load_state_dict(checkpoint["model_state_dict"])
    log(f"-> loaded checkpoint {cfgs.checkpoint_path} (epoch: {checkpoint['epoch']})")
    log(f"-> device: {device}")
    net.eval()
    return net


def get_and_process_data(data_dir):
    color = np.array(Image.open(os.path.join(data_dir, "color.png")), dtype=np.float32) / 255.0
    depth = np.array(Image.open(os.path.join(data_dir, "depth.png")))
    workspace_mask = np.array(Image.open(os.path.join(data_dir, "workspace_mask.png")))
    meta = scio.loadmat(os.path.join(data_dir, "meta.mat"))
    intrinsic = meta["intrinsic_matrix"]
    factor_depth = meta["factor_depth"]

    camera = CameraInfo(
        1280.0,
        720.0,
        intrinsic[0][0],
        intrinsic[1][1],
        intrinsic[0][2],
        intrinsic[1][2],
        factor_depth,
    )
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    mask = workspace_mask & (depth > 0)
    cloud_masked = cloud[mask]
    color_masked = color[mask]
    log(f"-> valid scene points: {len(cloud_masked)}")

    if len(cloud_masked) >= cfgs.num_point:
        idxs = np.random.choice(len(cloud_masked), cfgs.num_point, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(len(cloud_masked), cfgs.num_point - len(cloud_masked), replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    scene_cloud = o3d.geometry.PointCloud()
    scene_cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    scene_cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    end_points = {
        "point_clouds": torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32)).to(device),
        "cloud_colors": color_sampled,
    }
    return end_points, scene_cloud


def get_grasps(net, end_points):
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)
    gg_array = grasp_preds[0].detach().cpu().numpy()
    log(f"-> decoded grasps: {len(gg_array)}")
    return GraspGroup(gg_array)


def collision_detection(gg, scene_cloud):
    detector = ModelFreeCollisionDetector(np.asarray(scene_cloud.points), voxel_size=cfgs.voxel_size)
    collision_mask = detector.detect(gg, approach_dist=0.05, collision_thresh=cfgs.collision_thresh)
    filtered = gg[~collision_mask]
    log(f"-> grasps after collision filtering: {len(filtered)}")
    return filtered


def merge_meshes(meshes):
    merged = o3d.geometry.TriangleMesh()
    for mesh in meshes:
        merged += mesh
    merged.compute_vertex_normals()
    return merged


def make_scene_with_grasps_cloud(scene_cloud, gripper_meshes):
    points = [np.asarray(scene_cloud.points)]
    colors = [np.asarray(scene_cloud.colors)]
    for mesh in gripper_meshes:
        points.append(np.asarray(mesh.vertices))
        colors.append(np.asarray(mesh.vertex_colors))

    merged_cloud = o3d.geometry.PointCloud()
    merged_cloud.points = o3d.utility.Vector3dVector(np.concatenate(points, axis=0))
    merged_cloud.colors = o3d.utility.Vector3dVector(np.concatenate(colors, axis=0))
    return merged_cloud


def save_outputs(gg, scene_cloud):
    os.makedirs(cfgs.output_dir, exist_ok=True)

    gg.nms()
    gg.sort_by_score()
    top_gg = gg[: cfgs.top_k]
    log(f"-> saving top grasps: {len(top_gg)}")

    top_gg.save_npy(os.path.join(cfgs.output_dir, "grasps.npy"))
    o3d.io.write_point_cloud(os.path.join(cfgs.output_dir, "scene_cloud.pcd"), scene_cloud)

    gripper_meshes = top_gg.to_open3d_geometry_list()
    if gripper_meshes:
        gripper_mesh = merge_meshes(gripper_meshes)
        scene_with_grasps = make_scene_with_grasps_cloud(scene_cloud, gripper_meshes)
        o3d.io.write_triangle_mesh(os.path.join(cfgs.output_dir, "top_grasps.ply"), gripper_mesh)
        o3d.io.write_point_cloud(os.path.join(cfgs.output_dir, "scene_with_grasps.pcd"), scene_with_grasps)

    log(f"-> outputs saved to {os.path.abspath(cfgs.output_dir)}")


def main():
    t0 = time.time()
    net = get_net()
    log(f"timing load_net: {time.time() - t0:.2f}s")

    t1 = time.time()
    end_points, scene_cloud = get_and_process_data(cfgs.data_dir)
    log(f"timing load_data: {time.time() - t1:.2f}s")

    t2 = time.time()
    gg = get_grasps(net, end_points)
    log(f"timing inference_decode: {time.time() - t2:.2f}s")

    if cfgs.collision_thresh > 0:
        t3 = time.time()
        gg = collision_detection(gg, scene_cloud)
        log(f"timing collision_detection: {time.time() - t3:.2f}s")
    else:
        log("-> collision detection skipped")

    t4 = time.time()
    save_outputs(gg, scene_cloud)
    log(f"timing save_outputs: {time.time() - t4:.2f}s")
    log(f"timing total: {time.time() - t0:.2f}s")


if __name__ == "__main__":
    main()
