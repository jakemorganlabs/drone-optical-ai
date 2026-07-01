"""
pyvista_interactive_view_with_rotation_history.py

Requirements:
    pip install pyvista numpy

Usage:
    python pyvista_interactive_view_with_rotation_history.py

Description:
    1) Loads voxel_grid.bin (written by your C++ code).
    2) Interprets the 3D array shape as (Z, Y, X).
    3) Extracts top percentile of brightness.
    4) Applies an additional Euler rotation to the entire cloud (user-defined).
    5) Displays them interactively in a PyVista window,
       so you can orbit, zoom, and pan with the mouse.
    6) On closing the window, saves a 1920×1080 screenshot named 'voxel_####.png'
       in a 'screenshots/' folder, so you can keep a history of runs.
"""

import os
import re
import math
import numpy as np
import pyvista as pv


def load_voxel_grid(filename, grid_index=0):
    """
    Reads a voxel grid written by LiveVoxelMapper::save_voxel_grid.

    Container layout (VXG1, little-endian):
      4 bytes  magic = b"VXG1"
      int32    N (grid size, NxNxN)
      float32  voxel_size (meters)
      int32    num_grids  (number of N^3 float32 blocks that follow)
      num_grids * N*N*N float32  voxel data, row-major with index
                               voxel_idx = ix*N*N + iy*N + iz
    Index in the returned ndarray is therefore voxel[ix, iy, iz].

    grid_index selects which grid to return:
      0 = dynamic, 1 = static (when present).

    Returns (voxel_grid (N,N,N) float32, voxel_size float, num_grids int).
    """
    with open(filename, "rb") as f:
        magic = f.read(4)
        if magic != b"VXG1":
            raise ValueError(
                f"Unsupported voxel file magic {magic!r}; expected b'VXG1'. "
                "Re-save with the current LiveVoxelMapper::save_voxel_grid.")
        N = int.from_bytes(f.read(4), "little")
        voxel_size = float(np.frombuffer(f.read(4), dtype="<f4")[0])
        num_grids = int.from_bytes(f.read(4), "little")

        if grid_index < 0 or grid_index >= num_grids:
            raise IndexError(
                f"grid_index {grid_index} out of range for file with {num_grids} grids")

        block_bytes = N * N * N * 4
        # Skip to the requested grid block
        f.read(block_bytes * grid_index)
        data = np.frombuffer(f.read(block_bytes), dtype="<f4")
        voxel_grid = data.reshape((N, N, N))

    return voxel_grid, voxel_size, num_grids


def load_voxel_grid_v0(filename):
    """Legacy single-grid reader kept for backwards compatibility:
    int32 N | float32 voxel_size | N^3 float32. Auto-detects VXG1 too."""
    with open(filename, "rb") as f:
        head = f.read(4)
        f.seek(0)
        if head == b"VXG1":
            grid, vs, _ = load_voxel_grid(filename)
            return grid, vs
        N = int.from_bytes(f.read(4), "little")
        voxel_size = float(np.frombuffer(f.read(4), dtype="<f4")[0])
        data = np.frombuffer(f.read(N * N * N * 4), dtype="<f4")
    return data.reshape((N, N, N)), voxel_size


def extract_top_percentile_z_up(voxel_grid, voxel_size, grid_center,
                                percentile=99.5, use_hard_thresh=False, hard_thresh=700,
                                axis_order="xyz"):
    """
    Extract the top 'percentile' bright voxels (or above 'hard_thresh').

    The C++ writer stores voxels row-major as voxel_idx = ix*N*N + iy*N + iz,
    so after reshape((N,N,N)) the natural index is voxel[ix, iy, iz].
    axis_order="xyz" (default, matches current C++ writer): voxel[ix, iy, iz].
    axis_order="zyx" (legacy behaviour): voxel[z, y, x].

    Output is an Nx3 array of points in (x, y, z) world coords plus the
    intensities array.
    """
    N = voxel_grid.shape[0]
    half_side = (N * voxel_size) * 0.5
    grid_min = grid_center - half_side

    flat_vals = voxel_grid.ravel()
    if use_hard_thresh:
        thresh = hard_thresh
    else:
        thresh = np.percentile(flat_vals, percentile)

    coords = np.argwhere(voxel_grid > thresh)
    if coords.size == 0:
        print(f"No voxels above threshold {thresh}. Nothing to display.")
        return None, None

    intensities = voxel_grid[coords[:, 0], coords[:, 1], coords[:, 2]]

    if axis_order == "xyz":
        x_idx = coords[:, 0] + 0.5
        y_idx = coords[:, 1] + 0.5
        z_idx = coords[:, 2] + 0.5
    else:
        z_idx = coords[:, 0] + 0.5
        y_idx = coords[:, 1] + 0.5
        x_idx = coords[:, 2] + 0.5

    x_world = grid_min[0] + x_idx * voxel_size
    y_world = grid_min[1] + y_idx * voxel_size
    z_world = grid_min[2] + z_idx * voxel_size

    points = np.column_stack((x_world, y_world, z_world))
    return points, intensities


def rotation_matrix_xyz(rx_deg, ry_deg, rz_deg):
    """
    Build a rotation matrix (3x3) for Euler angles (rx, ry, rz) in degrees,
    applied in X->Y->Z order. That is:
      R = Rz(rz) * Ry(ry) * Rx(rx)
    so we rotate first by rx around X, then ry around Y, then rz around Z.
    """
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)

    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    # Rx
    Rx = np.array([
        [1,   0,   0],
        [0,  cx, -sx],
        [0,  sx,  cx]
    ], dtype=np.float32)

    # Ry
    Ry = np.array([
        [ cy,  0,  sy],
        [  0,  1,   0],
        [-sy,  0,  cy]
    ], dtype=np.float32)

    # Rz
    Rz = np.array([
        [ cz, -sz,  0],
        [ sz,  cz,  0],
        [  0,   0,  1]
    ], dtype=np.float32)

    # Combined: Rz * Ry * Rx
    Rtemp = Rz @ Ry
    Rfinal = Rtemp @ Rx
    return Rfinal


def get_next_image_index(folder, prefix="voxel_", suffix=".png"):
    """
    Scan 'folder' for files named like 'voxel_XXXX.png'.
    Find the largest XXXX as int, return that + 1.
    If none found, return 1.
    """
    if not os.path.exists(folder):
        return 1

    pattern = re.compile(rf"^{prefix}(\d+){suffix}$")
    max_index = 0
    for fname in os.listdir(folder):
        match = pattern.match(fname)
        if match:
            idx = int(match.group(1))
            if idx > max_index:
                max_index = idx
    return max_index + 1


def main():
    # 1) Load the voxel grid (defaults to the dynamic grid; pass grid_index=1 for static)
    voxel_grid, vox_size, num_grids = load_voxel_grid("voxel_grid.bin")
    print("Loaded voxel grid:", voxel_grid.shape, "voxel_size=", vox_size,
          "grids=", num_grids)
    print("Max voxel value:", voxel_grid.max())

    # 2) Define the grid center (x,y,z). The C++ writer uses grid origin = config.origin;
    # the CLI default config sets origin (0,0,10). Override here if you ran with --out.
    grid_center = np.array([0, 0, 10], dtype=np.float32)

    # 3) Extract top percentile. axis_order="xyz" matches the current C++ writer
    # (voxel_idx = ix*N*N + iy*N + iz). Use "zyx" for legacy files.
    percentile_to_show = 99.9
    points, intensities = extract_top_percentile_z_up(
        voxel_grid,
        voxel_size=vox_size,
        grid_center=grid_center,
        percentile=percentile_to_show,
        use_hard_thresh=False,
        hard_thresh=700,
        axis_order="xyz",
    )
    if points is None:
        return  # nothing to show

    # 4) Optional rotation
    # e.g. rotate to fix orientation
    rx_deg = 90
    ry_deg = 270
    rz_deg = 0

    R = rotation_matrix_xyz(rx_deg, ry_deg, rz_deg)  # shape (3,3)
    points_rot = points @ R.T
    # 5) PyVista Plotter (interactive)
    plotter = pv.Plotter(off_screen=True,)
    plotter.set_background("white")
    plotter.enable_terrain_style()


    # Convert to PolyData with scalars
    cloud = pv.PolyData(points_rot)
    cloud["intensity"] = intensities

    # Add points
    plotter.add_points(
        cloud,
        scalars="intensity",
        cmap="hot",
        point_size=4.0,
        render_points_as_spheres=True,
        opacity=0.1,
    )

    plotter.add_scalar_bar(
        title="Brightness",
        n_labels=5
    )

    # 6) Determine the next screenshot index
    screenshot_folder = "screenshots"
    if not os.path.exists(screenshot_folder):
        os.makedirs(screenshot_folder)
    next_idx = get_next_image_index(screenshot_folder, prefix="voxel_", suffix=".png")
    out_name = f"voxel_{next_idx:04d}.png"
    out_path = os.path.join(screenshot_folder, out_name)
    # 7) Show the interactive window at 1920x1080 and save final screenshot
    #    The screenshot is generated when you close the plot window.
    plotter.show(window_size=[3840, 2160], auto_close=False, screenshot=out_path)



    print(f"[Info] Saved screenshot to {out_path}")
    plotter = pv.Plotter(off_screen=False, )
    plotter.set_background("white")
    plotter.enable_terrain_style()

    # Convert to PolyData with scalars
    cloud = pv.PolyData(points_rot)
    cloud["intensity"] = intensities

    # Add points
    plotter.add_points(
        cloud,
        scalars="intensity",
        cmap="hot",
        point_size=4.0,
        render_points_as_spheres=True,
        opacity=0.05,
    )

    # plotter.add_scalar_bar(
    #     title="Brightness",
    #     n_labels=5
    # )
    print("[Done]")
    plotter.show(window_size=[1920, 1080], auto_close=False)




if __name__ == "__main__":
    main()
