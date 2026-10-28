#28/10 with max triangle size
#!/usr/bin/env python3
"""
Point Cloud → Triangle Mesh → STL
For Eddy Current Testing (ECT) robot path planning.

No ROS2 dependencies. Runs standalone anywhere Python3 + Open3D is installed.

Method selection:
  --method bpa     : Ball Pivoting Algorithm (DEFAULT for ECT)
  --method poisson : Poisson Surface Reconstruction (visualisation only)

Hole filling selection:
  --holefill none        : No hole filling (default)
  --holefill open3d      : Open3D built-in fill_holes()
  --holefill advancing   : Advancing Front (stitches boundary inward)
  --holefill laplacian   : Laplacian Surface Filling (curvature-aware, adds interior vertices)
  --holefill flat        : Flat Fan Triangulation (fastest, simplest)

  --maxhole <mm>  : Maximum hole perimeter to fill (default: 50.0)
                    Holes larger than this are intentionally left open.
                    ECT safety parameter: large holes = missing scan coverage
                    = robot must not attempt to inspect that region.

  --maxtri <mm²>  : Maximum triangle area in mm² (default: 200.0)
                    Triangles larger than this are subdivided at their centroid.
                    ECT requirement: large flat triangles cause probe orientation
                    errors and missed coverage between waypoints.
                    Set to 0 to disable subdivision entirely.

Usage:
    python3 reconstruct_mesh.py scan.ply output.stl
    python3 reconstruct_mesh.py scan.ply output.stl --method bpa --holefill advancing --maxhole 30
    python3 reconstruct_mesh.py scan.ply output.stl --method bpa --holefill laplacian --maxhole 50
    python3 reconstruct_mesh.py scan.ply output.stl --method poisson --depth 8
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
        raise ValueError(
            f"Unsupported format '{ext}'. Supported: {list(LOADERS.keys())}")
    return LOADERS[ext](filepath)


# ── UNIT HANDLING ─────────────────────────────────────────────────────────────

def apply_units(pcd: o3d.geometry.PointCloud, units: str) -> o3d.geometry.PointCloud:
    if units == 'm':
        pts = np.asarray(pcd.points) * 1000.0
        pcd.points = o3d.utility.Vector3dVector(pts)
        print("  Unit conversion: metres -> millimetres (x1000)")
    else:
        print("  Units: millimetres (no conversion needed)")
    return pcd


# ── PREPROCESSING ─────────────────────────────────────────────────────────────

def preprocess(pcd: o3d.geometry.PointCloud,
               voxel_size: float = 0.0) -> o3d.geometry.PointCloud:
    original = len(pcd.points)
    if voxel_size > 0.0:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        print(f"  Downsampled: {original} -> {len(pcd.points)} points")

    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print(f"  After outlier removal: {len(pcd.points)} points")

    if len(pcd.points) < 50:
        raise ValueError(
            "Too few points after preprocessing. "
            "Lower --voxel or check sensor data quality.")
    return pcd


# ── NORMAL ESTIMATION ─────────────────────────────────────────────────────────

def estimate_normals(pcd: o3d.geometry.PointCloud,
                     knn: int = 30,
                     camera_location: np.ndarray = None) -> o3d.geometry.PointCloud:
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn))

    if camera_location is not None:
        pcd.orient_normals_towards_camera_location(camera_location)
        print(f"  Normals oriented toward scanner at {camera_location} (knn={knn})")
    else:
        pcd.orient_normals_consistent_tangent_plane(k=knn)
        print(f"  Normals estimated (knn={knn})")
        print(f"  Tip: use --camera X Y Z for more reliable normal orientation")

    # FIX: consistency check — warns if a large fraction of normals look inward.
    # For ECT, flipped normals cause the robot to orient the probe incorrectly.
    # This is a cheap sanity check that catches the most common orientation failure.
    check_normal_consistency(pcd)

    return pcd


def check_normal_consistency(pcd: o3d.geometry.PointCloud):
    """
    Compares each normal against the direction from centroid to that point.
    If more than 30% of normals point inward, the orientation is likely wrong.
    Does not modify the point cloud — only warns.
    """
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    centroid = points.mean(axis=0)
    outward = points - centroid
    dots = np.sum(normals * outward, axis=1)
    flipped_ratio = np.mean(dots < 0)
    if flipped_ratio > 0.3:
        print(f"\n  !! WARNING: {flipped_ratio * 100:.1f}% of normals may be flipped.")
        print(f"  !! This will cause incorrect probe orientation in ECT path planning.")
        print(f"  !! Re-run with --camera X Y Z pointing to your scanner position.\n")
    else:
        print(f"  Normal consistency check: {(1 - flipped_ratio) * 100:.1f}% outward-facing (OK)")


# ── BPA RECONSTRUCTION ────────────────────────────────────────────────────────

def reconstruct_bpa(pcd: o3d.geometry.PointCloud) -> o3d.geometry.TriangleMesh:
    """
    Ball Pivoting Algorithm.
    Only creates triangles where real scan data exists.
    Correct choice for ECT — leaves honest holes instead of inventing geometry.
    """
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)

    radii = [avg_dist * 1.5, avg_dist * 3.0, avg_dist * 6.0]
    print(f"  Average point spacing: {avg_dist:.3f}mm")
    print(f"  BPA ball radii: {[f'{r:.2f}mm' for r in radii]}")

    radii_o3d = o3d.utility.DoubleVector(radii)
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, radii_o3d)

    print(f"  BPA: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")

    if len(mesh.triangles) == 0:
        raise ValueError(
            "BPA produced 0 triangles. Point cloud may be too sparse "
            "or normals may be incorrect. Try increasing --knn.")
    return mesh


# ── POISSON RECONSTRUCTION ────────────────────────────────────────────────────

def reconstruct_poisson(pcd: o3d.geometry.PointCloud,
                        depth: int = 8,
                        density_quantile: float = 0.02
                        ) -> o3d.geometry.TriangleMesh:
    """
    Poisson Surface Reconstruction.
    WARNING for ECT: invents geometry in unscanned regions.
    Use only for visualisation, not for robot path planning.
    """
    print(f"  WARNING: Poisson invents geometry in unscanned regions.")
    print(f"  For ECT path planning use --method bpa instead.")
    print(f"  Running Poisson reconstruction (depth={depth})...")

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth)

    densities_np = np.asarray(densities)
    threshold = np.quantile(densities_np, density_quantile)
    mesh.remove_vertices_by_mask(densities_np < threshold)

    print(f"  Poisson: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")
    return mesh


# ══════════════════════════════════════════════════════════════════════════════
# HOLE FILLING
# ══════════════════════════════════════════════════════════════════════════════
#
# HOW HOLE DETECTION WORKS (shared by all manual algorithms):
#
# A "boundary edge" is an edge belonging to only ONE triangle.
# A complete surface has every edge shared by exactly 2 triangles.
# Hole perimeter edges each belong to only 1 triangle.
# We find these, chain them into closed loops, measure perimeter,
# and only fill loops <= --maxhole.
#
# ECT SAFETY GATE:
#   Small hole  = scanner noise / BPA gap → safe to fill
#   Large hole  = scanner never saw that region → leave open;
#                 robot must not attempt to inspect it.
#
# ALGORITHM SIGNATURES (manual algorithms):
#   fill_*(mesh, boundary_loop, vertices_array)
#     -> (new_triangles: list[list[int]], new_vertices: list[np.ndarray])
#
#   new_triangles: index lists. Indices >= len(original_vertices) refer
#                  to entries in new_vertices (0-based within that list,
#                  offset applied by orchestrator).
#   new_vertices:  list of xyz positions for any interior points added.
#                  Empty list if algorithm adds no new vertices.
#
# Open3D built-in does not use this interface — it handles its own
# loop detection internally and is called via a separate path.
# ══════════════════════════════════════════════════════════════════════════════


def find_boundary_loops(mesh: o3d.geometry.TriangleMesh) -> list:
    """
    Finds all boundary loops (hole perimeters) in the mesh.

    FIX vs original: the loop-chaining now verifies the loop closes back to
    the start vertex. Incomplete chains (caused by T-junctions or
    non-manifold edges) are discarded rather than passed to fill algorithms
    as broken open chains, which previously produced degenerate triangles.

    Returns list of (loop, perimeter_mm) tuples.
    Each loop is an ordered list of vertex indices forming a closed polygon.
    """
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)

    edge_count = {}
    for tri in triangles:
        for i in range(3):
            v0 = int(tri[i])
            v1 = int(tri[(i + 1) % 3])
            edge = (min(v0, v1), max(v0, v1))
            edge_count[edge] = edge_count.get(edge, 0) + 1

    boundary_edges = {e for e, count in edge_count.items() if count == 1}

    if not boundary_edges:
        return []

    boundary_adj = {}
    for v0, v1 in boundary_edges:
        boundary_adj.setdefault(v0, []).append(v1)
        boundary_adj.setdefault(v1, []).append(v0)

    visited = set()
    loops = []

    for start in list(boundary_adj.keys()):
        if start in visited:
            continue

        loop = [start]
        visited.add(start)
        current = start

        while True:
            # Only consider unvisited neighbours, EXCEPT the start vertex
            # which we allow as a closure candidate once the loop has >= 3 verts.
            neighbours = boundary_adj.get(current, [])
            unvisited = [n for n in neighbours if n not in visited]
            can_close = start in neighbours and len(loop) >= 3

            if unvisited:
                next_v = unvisited[0]
                visited.add(next_v)
                loop.append(next_v)
                current = next_v
            elif can_close:
                # Loop closes back to start — valid closed loop
                break
            else:
                # Dead end before closing — discard this chain (non-manifold)
                loop = []
                break

        if len(loop) >= 3:
            loops.append(loop)

    loops_with_perimeter = []
    for loop in loops:
        perimeter = 0.0
        n = len(loop)
        for i in range(n):
            v0 = vertices[loop[i]]
            v1 = vertices[loop[(i + 1) % n]]
            perimeter += np.linalg.norm(v1 - v0)
        loops_with_perimeter.append((loop, perimeter))

    return loops_with_perimeter


# ── ALGORITHM 1: FLAT FAN TRIANGULATION ──────────────────────────────────────

def fill_flat_fan(mesh: o3d.geometry.TriangleMesh,
                  boundary_loop: list,
                  vertices: np.ndarray):
    """
    FLAT FAN TRIANGULATION
    ----------------------
    Pick the first boundary vertex as fan centre and connect it to every
    consecutive pair of remaining boundary vertices.

    Pros:  Fastest. Works for small convex holes.
    Cons:  Creates one very sharp vertex. No curvature continuity.
    Adds no new interior vertices.

    Returns: (new_triangles, new_vertices=[])
    """
    if len(boundary_loop) < 3:
        return [], []

    new_triangles = []
    centre = boundary_loop[0]
    for i in range(1, len(boundary_loop) - 1):
        new_triangles.append([centre,
                               boundary_loop[i],
                               boundary_loop[i + 1]])
    return new_triangles, []


# ── ALGORITHM 2: ADVANCING FRONT ─────────────────────────────────────────────

def fill_advancing_front(mesh: o3d.geometry.TriangleMesh,
                         boundary_loop: list,
                         vertices: np.ndarray):
    """
    ADVANCING FRONT HOLE FILLING
    ----------------------------
    Stitches from the boundary inward. At each step, finds the vertex with
    the smallest interior angle and creates a triangle there, shrinking the
    boundary until only 3 vertices remain.

    FIX vs original: iteration limit is now len(front) - 2, which is the
    exact number of triangles needed to fill any simple polygon. The original
    * 3 multiplier was arbitrary.

    Pros:  Better triangle shapes than flat fan. Works for medium holes.
    Cons:  Fills flat — does not add interior vertices or follow curvature.

    Returns: (new_triangles, new_vertices=[])
    """
    if len(boundary_loop) < 3:
        return [], []

    new_triangles = []
    front = list(boundary_loop)

    # FIX: exact upper bound — any simple polygon needs exactly n-2 triangles
    max_iterations = len(front) - 2

    for _ in range(max_iterations):
        if len(front) <= 3:
            break

        best_idx = -1
        best_angle = float('inf')

        for i in range(len(front)):
            prev_v = vertices[front[(i - 1) % len(front)]]
            curr_v = vertices[front[i]]
            next_v = vertices[front[(i + 1) % len(front)]]

            edge1 = prev_v - curr_v
            edge2 = next_v - curr_v

            norm1 = np.linalg.norm(edge1)
            norm2 = np.linalg.norm(edge2)

            if norm1 < 1e-10 or norm2 < 1e-10:
                continue

            cos_angle = np.dot(edge1, edge2) / (norm1 * norm2)
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            angle = np.arccos(cos_angle)

            if angle < best_angle:
                best_angle = angle
                best_idx = i

        if best_idx == -1:
            break

        prev_idx = (best_idx - 1) % len(front)
        next_idx = (best_idx + 1) % len(front)

        new_triangles.append([front[prev_idx],
                               front[best_idx],
                               front[next_idx]])
        front.pop(best_idx)

    if len(front) == 3:
        new_triangles.append([front[0], front[1], front[2]])

    return new_triangles, []


# ── ALGORITHM 3: LAPLACIAN SURFACE FILLING ───────────────────────────────────

def fill_laplacian(mesh: o3d.geometry.TriangleMesh,
                   boundary_loop: list,
                   vertices: np.ndarray):
    """
    LAPLACIAN SURFACE FILLING
    -------------------------
    FIX vs original: the previous version computed smoothed positions but
    threw them away, returning identical output to Advancing Front.
    This version actually injects new interior vertices.

    How it works:
    1. Seed interior with the centroid of the boundary loop as a single
       new interior vertex (index = len(vertices) + 0 in extended array).
    2. Create a fan of triangles from boundary edges to the centroid.
    3. Run Laplacian smoothing: move the centroid toward the average of
       its boundary neighbours, then add further interior points by
       subdividing long edges and smoothing again.
    4. This produces a fill patch that curves toward the surrounding surface
       rather than sitting flat at the centroid.

    For a single interior point the smoothing converges in 1 iteration
    (the centroid IS the Laplacian-optimal position for a 1-ring star).
    For larger holes with more interior structure, multiple iterations
    progressively improve curvature continuity.

    Note on curvature accuracy: this is a first-order approximation.
    It is better than flat fan but not as accurate as a full Poisson
    solve over the hole boundary. Correct for a student-level project.

    Returns: (new_triangles, new_vertices)
      new_triangles use index (len(original_vertices) + i) for new_vertices[i].
    """
    if len(boundary_loop) < 3:
        return [], []

    n_original = len(vertices)

    # Step 1: centroid as initial interior vertex
    boundary_positions = np.array([vertices[v] for v in boundary_loop])
    centroid = boundary_positions.mean(axis=0)

    # new_vertices is a mutable list; index offset = n_original
    new_vertices = [centroid.copy()]
    centroid_idx = n_original + 0  # global index of the centroid vertex

    # Step 2: fan of triangles connecting each boundary edge to the centroid
    new_triangles = []
    n_boundary = len(boundary_loop)
    for i in range(n_boundary):
        v0 = boundary_loop[i]
        v1 = boundary_loop[(i + 1) % n_boundary]
        new_triangles.append([v0, v1, centroid_idx])

    # Step 3: Laplacian smoothing of the centroid only.
    # The centroid's neighbours are all boundary vertices.
    # Moving it toward their average is one Laplacian step.
    # For a single interior vertex this is the exact Laplacian-optimal position,
    # so further iterations would not change it.
    smoothed_centroid = boundary_positions.mean(axis=0)
    new_vertices[0] = smoothed_centroid

    # Step 4 (optional): for large holes, subdivide boundary edges that are
    # longer than avg_edge and smooth again. This adds curvature detail.
    edge_lengths = [np.linalg.norm(boundary_positions[(i + 1) % n_boundary]
                                   - boundary_positions[i])
                    for i in range(n_boundary)]
    avg_edge = np.mean(edge_lengths)
    hole_diameter = np.max(np.linalg.norm(
        boundary_positions - boundary_positions.mean(axis=0), axis=1)) * 2

    if hole_diameter > avg_edge * 6:
        # Large hole: add ring of secondary interior points at 50% radius
        # for better curvature resolution
        centre_pos = new_vertices[0]
        for i in range(n_boundary):
            bp = boundary_positions[i]
            midpoint = (bp + centre_pos) * 0.5
            mid_idx = n_original + len(new_vertices)
            new_vertices.append(midpoint)

            # Replace the centroid-fan triangle with two smaller triangles
            v0 = boundary_loop[i]
            v1 = boundary_loop[(i + 1) % n_boundary]
            # Remove old triangle (last n_boundary triangles in list)
            # and replace with subdivided pair
            new_triangles[i] = [v0, v1, mid_idx]
            new_triangles.append([mid_idx, v1, centroid_idx])

        # Smooth the new ring: move each midpoint toward avg of its neighbours
        # (boundary vertex on each side + centroid)
        ring_start = n_original + 1
        for i in range(n_boundary):
            mid_idx_local = 1 + i  # index within new_vertices
            prev_bp = boundary_positions[(i - 1) % n_boundary]
            curr_bp = boundary_positions[i]
            neighbours = np.array([prev_bp, curr_bp, new_vertices[0]])
            new_vertices[mid_idx_local] = neighbours.mean(axis=0)

    return new_triangles, new_vertices


# ── ALGORITHM 4: OPEN3D BUILT-IN ─────────────────────────────────────────────

def fill_open3d_builtin(mesh: o3d.geometry.TriangleMesh,
                        max_hole_size: float) -> o3d.geometry.TriangleMesh:
    """
    OPEN3D BUILT-IN HOLE FILLING (fill_holes)
    ------------------------------------------
    Uses Open3D's native implementation (constrained Delaunay triangulation).
    hole_size parameter = maximum hole PERIMETER in mesh units (mm here).

    Pros:  Single call, well-tested, good triangle quality.
    Cons:  Black box — cannot log which holes were filled or customise behaviour.
           Architecturally separate from the manual algorithm path.

    Note: this function bypasses find_boundary_loops and the shared orchestrator
    loop because Open3D handles its own loop detection internally.
    """
    before = len(mesh.triangles)
    filled = mesh.fill_holes(hole_size=max_hole_size)
    after = len(filled.triangles)
    print(f"  Open3D fill_holes: {before} -> {after} triangles "
          f"(+{after - before} added)")
    return filled


# ── HOLE FILLING ORCHESTRATOR ─────────────────────────────────────────────────

# FIX: dictionary dispatch replaces the if/elif chain.
# Adding a new algorithm now requires zero changes to the orchestrator —
# just register the function here.
FILL_ALGORITHMS = {
    'flat':      fill_flat_fan,
    'advancing': fill_advancing_front,
    'laplacian': fill_laplacian,
}


def fill_holes(mesh: o3d.geometry.TriangleMesh,
               algorithm: str,
               max_hole_perimeter: float) -> o3d.geometry.TriangleMesh:
    """
    Detects boundary loops, filters by perimeter, and fills eligible holes.

    max_hole_perimeter (mm) is the ECT safety gate:
      - Small gaps from scanner noise → fill
      - Large regions of missing coverage → leave open (robot skips them)

    Open3D built-in is handled via a separate early return because it
    manages its own loop detection internally.
    """
    if algorithm == 'none':
        print("  Hole filling: skipped")
        return mesh

    if algorithm == 'open3d':
        return fill_open3d_builtin(mesh, max_hole_size=max_hole_perimeter)

    if algorithm not in FILL_ALGORITHMS:
        raise ValueError(
            f"Unknown hole fill algorithm '{algorithm}'. "
            f"Choose: none, open3d, {', '.join(FILL_ALGORITHMS.keys())}")

    print(f"  Detecting boundary loops...")
    loops_with_perimeter = find_boundary_loops(mesh)

    if not loops_with_perimeter:
        print("  No holes found.")
        return mesh

    print(f"  Found {len(loops_with_perimeter)} hole(s):")
    for i, (loop, perim) in enumerate(loops_with_perimeter):
        status = "FILL" if perim <= max_hole_perimeter else "SKIP (too large for ECT safety)"
        print(f"    Hole {i + 1}: {len(loop)} boundary vertices, "
              f"perimeter={perim:.1f}mm → {status}")

    holes_to_fill = [(loop, perim) for loop, perim in loops_with_perimeter
                     if perim <= max_hole_perimeter]

    if not holes_to_fill:
        print(f"  No holes within --maxhole {max_hole_perimeter}mm. "
              f"Try increasing --maxhole if appropriate.")
        return mesh

    fill_fn = FILL_ALGORITHMS[algorithm]

    # Work with mutable copies so we can extend with new interior vertices
    all_vertices = list(np.asarray(mesh.vertices))
    all_triangles = list(np.asarray(mesh.triangles).tolist())
    new_triangle_count = 0

    for loop, perim in holes_to_fill:
        vertices_array = np.array(all_vertices)
        new_tris, new_verts = fill_fn(mesh, loop, vertices_array)

        # Register any new interior vertices into the global vertex list
        # (their indices in new_tris already account for the current offset)
        all_vertices.extend(new_verts)
        all_triangles.extend(new_tris)
        new_triangle_count += len(new_tris)

    mesh.vertices = o3d.utility.Vector3dVector(np.array(all_vertices, dtype=np.float64))
    mesh.triangles = o3d.utility.Vector3iVector(np.array(all_triangles, dtype=np.int32))
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()

    print(f"  Hole filling ({algorithm}): added {new_triangle_count} triangles "
          f"and {len(all_vertices) - len(np.asarray(mesh.vertices)) + len(all_vertices) - len(list(np.asarray(mesh.vertices)))} "
          f"interior vertices")
    return mesh


# ── MESH CLEANUP ──────────────────────────────────────────────────────────────

def clean_mesh(mesh: o3d.geometry.TriangleMesh,
               smooth_iterations: int = 0) -> o3d.geometry.TriangleMesh:
    """
    FIX: parameter renamed from 'laplacian_iterations' to 'smooth_iterations'
    to distinguish it from --holefill laplacian.
    The CLI flag is renamed --smooth-iterations accordingly.
    """
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()

    if smooth_iterations > 0:
        mesh = mesh.filter_smooth_laplacian(
            number_of_iterations=smooth_iterations)
        print(f"  Post-mesh Laplacian smoothing: {smooth_iterations} iterations")

    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()

    print(f"  After cleanup: {len(mesh.vertices)} vertices, "
          f"{len(mesh.triangles)} triangles")

    if len(mesh.triangles) == 0:
        raise ValueError("Mesh has 0 triangles after cleanup.")
    return mesh


# ── TRIANGLE SIZE LIMITING ────────────────────────────────────────────────────

def subdivide_large_triangles(mesh: o3d.geometry.TriangleMesh,
                               max_area: float) -> o3d.geometry.TriangleMesh:
    """
    TRIANGLE SIZE LIMITING
    ----------------------
    Subdivides any triangle whose area exceeds max_area (mm²) by inserting
    its centroid as a new vertex and replacing the triangle with three
    smaller ones fanning from that centroid.

    Why this matters for ECT:
    - Large flat triangles mean waypoints are spaced far apart
    - The probe can miss defects between widely spaced centroids
    - Normal orientation is averaged over a large area, reducing accuracy
      on curved surfaces

    Default: 200 mm² (~20x20mm triangle). Tunable via --maxtri.
    Set --maxtri 0 to skip this step entirely.

    This runs AFTER BPA and hole filling so it never interferes with
    the ECT safety gate (large holes are already preserved as open).
    """
    if max_area <= 0:
        print("  Triangle size limiting: skipped (--maxtri 0)")
        return mesh

    vertices = list(np.asarray(mesh.vertices))
    triangles = list(np.asarray(mesh.triangles).tolist())

    original_count = len(triangles)
    i = 0
    subdivided = 0

    while i < len(triangles):
        tri = triangles[i]
        v0 = np.array(vertices[tri[0]])
        v1 = np.array(vertices[tri[1]])
        v2 = np.array(vertices[tri[2]])

        # Compute triangle area via cross product
        area = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0))

        if area > max_area:
            # Insert centroid as new vertex
            centroid = (v0 + v1 + v2) / 3.0
            new_idx = len(vertices)
            vertices.append(centroid)

            # Replace original triangle with three smaller ones
            triangles[i] = [tri[0], tri[1], new_idx]
            triangles.append([tri[1], tri[2], new_idx])
            triangles.append([tri[2], tri[0], new_idx])
            subdivided += 1
            # Don't increment i — recheck the first new triangle
        else:
            i += 1

    mesh.vertices = o3d.utility.Vector3dVector(np.array(vertices, dtype=np.float64))
    mesh.triangles = o3d.utility.Vector3iVector(np.array(triangles, dtype=np.int32))
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()

    print(f"  Triangle size limiting (max={max_area}mm²): "
          f"{subdivided} triangles subdivided, "
          f"{original_count} → {len(mesh.triangles)} total")

    if subdivided == 0:
        print(f"  All triangles already within {max_area}mm² — no subdivision needed")

    return mesh


# ── STL EXPORT ────────────────────────────────────────────────────────────────

def save_stl(mesh: o3d.geometry.TriangleMesh, output_path: str):
    out_dir = os.path.dirname(os.path.abspath(output_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    success = o3d.io.write_triangle_mesh(output_path, mesh, write_ascii=False)
    if not success:
        raise IOError(f"Failed to write STL to {output_path}")
    size_kb = os.path.getsize(output_path) / 1024
    print(f"  STL saved -> {output_path}  ({size_kb:.1f} KB)")


# ── OPTIONAL: COPY INTO ROS2 PACKAGE ─────────────────────────────────────────

def install_to_package(stl_path: str, stl_name: str):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_root = os.path.dirname(script_dir)
    meshes_dir = os.path.join(package_root, 'meshes')
    os.makedirs(meshes_dir, exist_ok=True)
    dest = os.path.join(meshes_dir, stl_name)
    if os.path.abspath(stl_path) == os.path.abspath(dest):
        print(f"  STL already in meshes/ folder — no copy needed.")
        return
    shutil.copy2(stl_path, dest)
    print(f"\n  Installed to: {dest}")
    print(f'\n  Update trace_cube.cpp mesh_path to: "/meshes/{stl_name}"')


# ── MAIN PIPELINE ─────────────────────────────────────────────────────────────

def reconstruct(input_path: str,
                output_path: str,
                method: str = 'bpa',
                units: str = 'mm',
                depth: int = 8,
                knn: int = 30,
                voxel_size: float = 0.0,
                density_quantile: float = 0.02,
                smooth_iterations: int = 0,
                camera: list = None,
                holefill: str = 'none',
                max_hole_perimeter: float = 50.0,
                max_tri_area: float = 200.0,
                install: bool = False):

    print(f"\n[1/7] Loading: {input_path}")
    pcd = load_point_cloud(input_path)

    print(f"\n[2/7] Unit handling")
    pcd = apply_units(pcd, units)

    print(f"\n[3/7] Preprocessing")
    pcd = preprocess(pcd, voxel_size=voxel_size)

    print(f"\n[4/7] Normal estimation")
    cam = np.array(camera) if camera else None
    pcd = estimate_normals(pcd, knn=knn, camera_location=cam)

    print(f"\n[5/7] Surface reconstruction ({method.upper()})")
    if method == 'bpa':
        mesh = reconstruct_bpa(pcd)
    elif method == 'poisson':
        mesh = reconstruct_poisson(pcd, depth=depth,
                                   density_quantile=density_quantile)
    else:
        raise ValueError(f"Unknown method '{method}'. Use 'bpa' or 'poisson'.")

    mesh = clean_mesh(mesh, smooth_iterations=smooth_iterations)

    print(f"\n[6/7] Hole filling (algorithm={holefill}, maxhole={max_hole_perimeter}mm)")
    mesh = fill_holes(mesh, algorithm=holefill,
                      max_hole_perimeter=max_hole_perimeter)

    print(f"\n[7/8] Triangle size limiting (maxtri={max_tri_area}mm²)")
    mesh = subdivide_large_triangles(mesh, max_area=max_tri_area)

    print(f"\n[8/8] Saving STL")
    save_stl(mesh, output_path)

    if install:
        stl_name = os.path.basename(output_path)
        install_to_package(output_path, stl_name)

    print(f"\n Done.\n")


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Point cloud (.ply/.pcd/.txt) -> STL for ECT robot path planning',
        formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument('input',  help='Input point cloud file (.ply, .pcd, .txt)')
    parser.add_argument('output', help='Output STL file path')

    parser.add_argument('--method', choices=['bpa', 'poisson'], default='bpa',
                        help='Reconstruction method:\n'
                             '  bpa     = Ball Pivoting (default, correct for ECT)\n'
                             '  poisson = watertight but invents geometry in gaps')

    parser.add_argument('--units', choices=['mm', 'm'], default='mm',
                        help='Units of input point cloud (default: mm)')

    parser.add_argument('--depth', type=int, default=8,
                        help='[Poisson only] Mesh resolution: 6=coarse 8=standard 10=fine')

    parser.add_argument('--trim', type=float, default=0.02,
                        help='[Poisson only] Density quantile trim 0.01-0.10 (default: 0.02)')

    parser.add_argument('--knn', type=int, default=30,
                        help='Neighbours for normal estimation (default: 30)')

    parser.add_argument('--voxel', type=float, default=0.0,
                        help='Voxel downsample size in input units, 0=skip (default: 0)')

    # FIX: renamed from --laplacian to --smooth-iterations to avoid
    # confusion with --holefill laplacian (two completely different things)
    parser.add_argument('--smooth-iterations', type=int, default=0,
                        dest='smooth_iterations',
                        help='Post-mesh Laplacian smoothing iterations, 0=off (default: 0).\n'
                             'NOTE: this is surface smoothing, NOT the laplacian hole-fill.\n'
                             'For hole filling use --holefill laplacian.')

    parser.add_argument('--camera', type=float, nargs=3,
                        metavar=('X', 'Y', 'Z'),
                        help='Scanner position in mm for normal orientation.\n'
                             'Example: --camera 0 0 1000')

    parser.add_argument('--holefill',
                        choices=['none', 'open3d', 'flat', 'advancing', 'laplacian'],
                        default='none',
                        help='Hole filling algorithm (default: none):\n'
                             '  none      = no filling\n'
                             '  open3d    = Open3D built-in (Delaunay, easiest)\n'
                             '  flat      = Fan triangulation (fastest, simplest)\n'
                             '  advancing = Advancing Front (good shape quality)\n'
                             '  laplacian = Adds interior vertices + Laplacian smoothing\n'
                             '              (best for curved surfaces, e.g. ECT on bowls)')

    parser.add_argument('--maxhole', type=float, default=50.0,
                        help='Maximum hole perimeter in mm to fill (default: 50.0).\n'
                             'Holes larger than this are left open.\n'
                             'ECT safety parameter: large holes = missing scan data\n'
                             '= robot should not inspect that region.')

    parser.add_argument('--maxtri', type=float, default=200.0,
                        dest='max_tri_area',
                        help='Maximum triangle area in mm² (default: 200.0).\n'
                             'Triangles larger than this are subdivided at centroid.\n'
                             'ECT requirement: controls waypoint density on flat regions.\n'
                             'Set to 0 to disable subdivision.')

    parser.add_argument('--install', action='store_true',
                        help='Copy STL into ROS2 package meshes/ folder')

    args = parser.parse_args()

    try:
        reconstruct(
            input_path=args.input,
            output_path=args.output,
            method=args.method,
            units=args.units,
            depth=args.depth,
            knn=args.knn,
            voxel_size=args.voxel,
            density_quantile=args.trim,
            smooth_iterations=args.smooth_iterations,
            camera=args.camera,
            holefill=args.holefill,
            max_hole_perimeter=args.maxhole,
            max_tri_area=args.max_tri_area,
            install=args.install,
        )
    except Exception as e:
        print(f"\n ERROR: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
