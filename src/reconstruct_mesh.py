#!/usr/bin/env python3
"""
Point Cloud → Triangle Mesh → STL

Supported inputs : .ply, .pcd, .txt (bare x y z)
Output           : binary STL

UNIT CONVENTION (critical — read this):
  trace_cube.cpp multiplies all STL coordinates by 0.001
  meaning it expects STL vertices in MILLIMETRES.
  
  Use --units mm  if your point cloud is in millimetres  (default, safe)
  Use --units m   if your point cloud is in metres       (script auto-scales ×1000)

Usage:
    python3 reconstruct_mesh.py scan.ply    output.stl
    python3 reconstruct_mesh.py scan.pcd    output.stl --units m
    python3 reconstruct_mesh.py scan.txt    output.stl --depth 9
    python3 reconstruct_mesh.py scan.ply    output.stl --install  # copies to meshes/ folder
"""

import argparse
import sys
import os
import shutil
import numpy as np
import open3d as o3d


# ── LOADERS ───────────────────────────────────────────────────────────────────

def load_txt(filepath: str) -> o3d.geometry.PointCloud:
    points = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            for sep in [',', None]:
                try:
                    parts = line.split(sep) if sep else line.split()
                    if len(parts) >= 3:
                        points.append([float(parts[0]),
                                       float(parts[1]),
                                       float(parts[2])])
                        break
                except ValueError:
                    continue
    if not points:
        raise ValueError(f"No valid XYZ points found in {filepath}")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float64))
    print(f"  Loaded {len(points)} points from TXT")
    return pcd


def load_ply_or_pcd(filepath: str) -> o3d.geometry.PointCloud:
    pcd = o3d.io.read_point_cloud(filepath)
    if len(pcd.points) == 0:
        raise ValueError(f"No points loaded from {filepath}. Check file integrity.")
    ext = os.path.splitext(filepath)[1].upper()
    print(f"  Loaded {len(pcd.points)} points from {ext}")
    return pcd


LOADERS = {
    '.ply': load_ply_or_pcd,
    '.pcd': load_ply_or_pcd,
    '.txt': load_txt,
}


def load_point_cloud(filepath: str) -> o3d.geometry.PointCloud:
    ext = os.path.splitext(filepath)[1].lower()
    if ext not in LOADERS:
        raise ValueError(f"Unsupported format '{ext}'. Supported: {list(LOADERS.keys())}")
    return LOADERS[ext](filepath)


# ── UNIT HANDLING ─────────────────────────────────────────────────────────────

def apply_units(pcd: o3d.geometry.PointCloud, units: str) -> o3d.geometry.PointCloud:
    """
    trace_cube.cpp does: position = vertex * 0.001
    So STL must be in MILLIMETRES for the robot to move correctly.
    
    If point cloud is already in mm → do nothing.
    If point cloud is in metres    → multiply all points by 1000.
    """
    if units == 'm':
        pts = np.asarray(pcd.points) * 1000.0
        pcd.points = o3d.utility.Vector3dVector(pts)
        print("  Unit conversion: metres → millimetres (×1000)")
    else:
        print("  Units: millimetres (no conversion needed)")
    return pcd


# ── PREPROCESSING ─────────────────────────────────────────────────────────────

def preprocess(pcd: o3d.geometry.PointCloud,
               voxel_size: float = 0.0) -> o3d.geometry.PointCloud:
    original = len(pcd.points)
    if voxel_size > 0.0:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        print(f"  Downsampled: {original} → {len(pcd.points)} points")

    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print(f"  After outlier removal: {len(pcd.points)} points")

    if len(pcd.points) < 50:
        raise ValueError(
            "Too few points after preprocessing. "
            "Lower --voxel or check sensor data quality.")
    return pcd


# ── NORMALS ───────────────────────────────────────────────────────────────────

def estimate_normals(pcd: o3d.geometry.PointCloud,
                     knn: int = 30) -> o3d.geometry.PointCloud:
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn))
    pcd.orient_normals_consistent_tangent_plane(k=knn)
    print(f"  Normals estimated (knn={knn})")
    return pcd


# ── RECONSTRUCTION ────────────────────────────────────────────────────────────

def reconstruct_poisson(pcd: o3d.geometry.PointCloud,
                        depth: int = 8,
                        density_quantile: float = 0.02
                        ) -> o3d.geometry.TriangleMesh:
    print(f"  Running Poisson reconstruction (depth={depth})...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth)

    densities_np = np.asarray(densities)
    threshold = np.quantile(densities_np, density_quantile)
    mesh.remove_vertices_by_mask(densities_np < threshold)

    print(f"  Poisson: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")
    return mesh


# ── CLEANUP ───────────────────────────────────────────────────────────────────

def clean_mesh(mesh: o3d.geometry.TriangleMesh) -> o3d.geometry.TriangleMesh:
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    mesh.compute_triangle_normals()  # trace_cube.cpp reads triangle_normals[]
    mesh.compute_vertex_normals()

    print(f"  After cleanup: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")

    if len(mesh.triangles) == 0:
        raise ValueError(
            "Mesh has 0 triangles. Increase --depth or reduce --trim.")
    return mesh


# ── STL EXPORT ────────────────────────────────────────────────────────────────

def save_stl(mesh: o3d.geometry.TriangleMesh, output_path: str):
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    success = o3d.io.write_triangle_mesh(output_path, mesh, write_ascii=False)
    if not success:
        raise IOError(f"Failed to write STL to {output_path}")
    size_kb = os.path.getsize(output_path) / 1024
    print(f"  STL saved → {output_path}  ({size_kb:.1f} KB)")


# ── INSTALL INTO PACKAGE ──────────────────────────────────────────────────────

def install_to_package(stl_path: str, stl_name: str):
    """
    Copies the STL into the meshes/ folder so trace_cube.cpp can find it.
    Also prints the exact mesh_path string you need to hardcode in trace_cube.cpp.
    """
    # Walk up from this script's location to find the package root
    # Assumes this script lives in <package_root>/src/
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_root = os.path.dirname(script_dir)
    meshes_dir = os.path.join(package_root, 'meshes')

    os.makedirs(meshes_dir, exist_ok=True)
    dest = os.path.join(meshes_dir, stl_name)
    shutil.copy2(stl_path, dest)
    print(f"\n  Installed to: {dest}")
    print(f"\n  Update trace_cube.cpp mesh_path to:")
    print(f'  "file://" + ament_index_cpp::get_package_share_directory("ur5e_surface_path")')
    print(f'  + "/meshes/{stl_name}"')


# ── MAIN PIPELINE ─────────────────────────────────────────────────────────────

def reconstruct(input_path: str,
                output_path: str,
                units: str = 'mm',
                depth: int = 8,
                knn: int = 30,
                voxel_size: float = 0.0,
                density_quantile: float = 0.02,
                install: bool = False):

    print(f"\n[1/5] Loading: {input_path}")
    pcd = load_point_cloud(input_path)

    print(f"\n[2/5] Unit handling")
    pcd = apply_units(pcd, units)

    print(f"\n[3/5] Preprocessing")
    pcd = preprocess(pcd, voxel_size=voxel_size)

    print(f"\n[4/5] Normal estimation")
    pcd = estimate_normals(pcd, knn=knn)

    print(f"\n[5/5] Poisson reconstruction")
    mesh = reconstruct_poisson(pcd, depth=depth, density_quantile=density_quantile)
    mesh = clean_mesh(mesh)

    print(f"\n[6/6] Saving STL")
    save_stl(mesh, output_path)

    if install:
        stl_name = os.path.basename(output_path)
        install_to_package(output_path, stl_name)

    print(f"\n✓ Done.\n")


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Convert point cloud (.ply/.pcd/.txt) → triangle mesh (.stl)')

    parser.add_argument('input',  help='Input point cloud file (.ply, .pcd, .txt)')
    parser.add_argument('output', help='Output STL file path')

    parser.add_argument('--units', choices=['mm', 'm'], default='mm',
                        help='Units of input point cloud. '
                             'trace_cube.cpp expects STL in mm. '
                             'Use --units m to auto-scale metres→mm. (default: mm)')

    parser.add_argument('--depth', type=int, default=8,
                        help='Poisson depth: 6=coarse, 8=standard, 10=fine (default: 8)')

    parser.add_argument('--knn', type=int, default=30,
                        help='Neighbours for normal estimation (default: 30)')

    parser.add_argument('--voxel', type=float, default=0.0,
                        help='Voxel downsample size in same units as input. '
                             '0 = skip (default: 0)')

    parser.add_argument('--trim', type=float, default=0.02,
                        help='Density quantile trim 0.01–0.10. '
                             'Higher = more aggressive boundary trimming (default: 0.02)')

    parser.add_argument('--install', action='store_true',
                        help='Copy STL into package meshes/ folder after generation')

    args = parser.parse_args()

    try:
        reconstruct(
            input_path=args.input,
            output_path=args.output,
            units=args.units,
            depth=args.depth,
            knn=args.knn,
            voxel_size=args.voxel,
            density_quantile=args.trim,
            install=args.install,
        )
    except Exception as e:
        print(f"\n✗ ERROR: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
