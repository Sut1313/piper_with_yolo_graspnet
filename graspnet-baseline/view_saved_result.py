"""View saved GraspNet demo outputs on the host.

Usage:
    python3 view_saved_result.py --output_dir outputs

Inputs expected in output_dir:
    - scene_cloud.pcd
    - scene_with_grasps.pcd
    - top_grasps.ply
    - grasps.npy
"""

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output_dir", default="outputs", help="Directory containing saved demo outputs")
    parser.add_argument(
        "--mode",
        choices=["combined_cloud", "mesh", "scene_only"],
        default="combined_cloud",
        help="What to display",
    )
    parser.add_argument("--point_size", type=float, default=2.0, help="Open3D point size")
    return parser.parse_args()


def require_file(path: Path):
    if not path.exists():
        raise FileNotFoundError(f"Missing file: {path}")
    return path


def print_summary(output_dir: Path):
    grasps_path = output_dir / "grasps.npy"
    if grasps_path.exists():
        grasps = np.load(grasps_path)
        print(f"grasps.npy: shape={grasps.shape}, dtype={grasps.dtype}")
        if len(grasps) > 0:
            print(f"top score: {grasps[0, 0]:.6f}")


def show_geometries(geometries, point_size: float):
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="GraspNet saved result", width=1280, height=720)
    for geometry in geometries:
        vis.add_geometry(geometry)

    render_option = vis.get_render_option()
    render_option.point_size = point_size
    render_option.background_color = np.asarray([1.0, 1.0, 1.0])

    vis.run()
    vis.destroy_window()


def main():
    args = parse_args()
    output_dir = Path(args.output_dir)
    print_summary(output_dir)

    if args.mode == "combined_cloud":
        cloud_path = require_file(output_dir / "scene_with_grasps.pcd")
        cloud = o3d.io.read_point_cloud(str(cloud_path))
        print(f"loaded {cloud_path}: {len(cloud.points)} points")
        show_geometries([cloud], args.point_size)
    elif args.mode == "mesh":
        scene_path = require_file(output_dir / "scene_cloud.pcd")
        mesh_path = require_file(output_dir / "top_grasps.ply")
        scene = o3d.io.read_point_cloud(str(scene_path))
        mesh = o3d.io.read_triangle_mesh(str(mesh_path))
        mesh.compute_vertex_normals()
        print(f"loaded {scene_path}: {len(scene.points)} points")
        print(f"loaded {mesh_path}: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")
        show_geometries([scene, mesh], args.point_size)
    else:
        scene_path = require_file(output_dir / "scene_cloud.pcd")
        scene = o3d.io.read_point_cloud(str(scene_path))
        print(f"loaded {scene_path}: {len(scene.points)} points")
        show_geometries([scene], args.point_size)


if __name__ == "__main__":
    main()
