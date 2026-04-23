#!/usr/bin/env python3

import math

import cv2
import numpy as np
from scipy.ndimage import binary_dilation, binary_erosion, distance_transform_edt


def occupancy_to_numpy(grid):
    info = grid.info
    data = np.array(grid.data, dtype=np.int8).reshape((info.height, info.width))
    return data, info.resolution, info.origin.position.x, info.origin.position.y


def cell_to_world(row, col, resolution, origin_x, origin_y):
    wx = origin_x + (col + 0.5) * resolution
    wy = origin_y + (row + 0.5) * resolution
    return wx, wy


def compute_distance_transform(data):
    obstacle_mask = (data != 0).astype(np.float64)
    return distance_transform_edt(1.0 - obstacle_mask)


def generate_wall_following_waypoints(grid, standoff=1.0, spacing=1.0, min_clearance=0.3):
    data, resolution, origin_x, origin_y = occupancy_to_numpy(grid)
    height, width = data.shape

    walls = (data == 100)
    free = (data == 0)

    if not np.any(walls) or not np.any(free):
        return []

    wall_boundary = binary_dilation(walls, iterations=1) & free

    dist_transform = compute_distance_transform(data)

    standoff_cells = int(standoff / resolution)
    min_clearance_cells = max(1, int(min_clearance / resolution))

    boundary_coords = np.argwhere(wall_boundary)
    if len(boundary_coords) == 0:
        return []

    normals = []
    for r, c in boundary_coords:
        r_min = max(0, r - 3)
        r_max = min(height, r + 4)
        c_min = max(0, c - 3)
        c_max = min(width, c + 4)
        region = walls[r_min:r_max, c_min:c_max]
        wall_positions = np.argwhere(region)
        if len(wall_positions) == 0:
            normals.append((0.0, 0.0))
            continue
        centroid_r = wall_positions[:, 0].mean() + r_min
        centroid_c = wall_positions[:, 1].mean() + c_min
        nr = r - centroid_r
        nc = c - centroid_c
        length = math.sqrt(nr * nr + nc * nc)
        if length > 0:
            normals.append((nr / length, nc / length))
        else:
            normals.append((0.0, 0.0))

    boundary_mask = np.zeros((height, width), dtype=np.uint8)
    boundary_mask[wall_boundary] = 255

    contours, _ = cv2.findContours(boundary_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    boundary_to_normal = {}
    for i, (r, c) in enumerate(boundary_coords):
        boundary_to_normal[(r, c)] = normals[i]

    raw_waypoints = []
    for contour in contours:
        contour_points = contour.squeeze()
        if contour_points.ndim < 2 or len(contour_points) < 3:
            continue

        for pt in contour_points:
            c, r = int(pt[0]), int(pt[1])
            normal = boundary_to_normal.get((r, c), (0.0, 0.0))
            if normal == (0.0, 0.0):
                continue

            nr, nc = normal
            target_r = int(round(r + nr * standoff_cells))
            target_c = int(round(c + nc * standoff_cells))

            if target_r < 0 or target_r >= height or target_c < 0 or target_c >= width:
                continue

            if data[target_r, target_c] != 0:
                continue

            if dist_transform[target_r, target_c] < min_clearance_cells:
                continue

            yaw = math.atan2(-nr, -nc)
            raw_waypoints.append((target_r, target_c, yaw))

    spacing_cells = max(1, int(spacing / resolution))
    waypoints = []
    last_r, last_c = -spacing_cells * 2, -spacing_cells * 2

    for r, c, yaw in raw_waypoints:
        dist = math.sqrt((r - last_r) ** 2 + (c - last_c) ** 2)
        if dist >= spacing_cells:
            wx, wy = cell_to_world(r, c, resolution, origin_x, origin_y)
            waypoints.append((wx, wy, yaw))
            last_r, last_c = r, c

    return waypoints


def generate_lawnmower_waypoints(grid, sweep_spacing=3.0, min_clearance=0.3, covered_mask=None):
    data, resolution, origin_x, origin_y = occupancy_to_numpy(grid)
    height, width = data.shape

    clearance_cells = max(1, int(min_clearance / resolution))
    free = (data == 0)
    safe = binary_erosion(free, iterations=clearance_cells)

    if covered_mask is not None:
        safe = safe & ~covered_mask

    if not np.any(safe):
        return []

    coords = np.argwhere(safe)
    if len(coords) < 10:
        return []

    cov = np.cov(coords.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    long_axis = eigenvectors[:, 1]
    sweep_dir = eigenvectors[:, 0]

    sweep_angle = math.atan2(sweep_dir[0], sweep_dir[1])

    center = coords.mean(axis=0)
    centered = coords - center
    rotated = centered @ np.column_stack([sweep_dir, long_axis])

    sweep_min = rotated[:, 0].min()
    sweep_max = rotated[:, 0].max()
    line_min = rotated[:, 1].min()
    line_max = rotated[:, 1].max()

    step = max(1.0, sweep_spacing / resolution)
    waypoints = []
    direction = 1

    sweep_pos = sweep_min + step / 2
    while sweep_pos < sweep_max:
        line_points = []
        for i in range(len(rotated)):
            if abs(rotated[i, 0] - sweep_pos) < step / 2:
                line_points.append(rotated[i, 1])

        if not line_points:
            sweep_pos += step
            continue

        line_points.sort()

        segments = []
        seg_start = line_points[0]
        prev = line_points[0]
        for lp in line_points[1:]:
            if lp - prev > step:
                segments.append((seg_start, prev))
                seg_start = lp
            prev = lp
        segments.append((seg_start, prev))

        if direction == -1:
            segments.reverse()

        for seg_start, seg_end in segments:
            for line_pos in ([seg_start, seg_end] if direction == 1 else [seg_end, seg_start]):
                rotated_pt = np.array([sweep_pos, line_pos])
                original = rotated_pt @ np.column_stack([sweep_dir, long_axis]).T + center
                r, c = int(round(original[0])), int(round(original[1]))

                if 0 <= r < height and 0 <= c < width and data[r, c] == 0:
                    yaw = math.atan2(long_axis[0], long_axis[1])
                    if direction == -1:
                        yaw += math.pi
                    wx, wy = cell_to_world(r, c, resolution, origin_x, origin_y)
                    waypoints.append((wx, wy, yaw))

        direction *= -1
        sweep_pos += step

    return waypoints


def mark_covered(waypoints, sensor_range, grid_shape, resolution, origin_x, origin_y):
    height, width = grid_shape
    covered = np.zeros((height, width), dtype=bool)

    sensor_cells = int(sensor_range / resolution)

    for wx, wy, _ in waypoints:
        col = int((wx - origin_x) / resolution)
        row = int((wy - origin_y) / resolution)

        r_min = max(0, row - sensor_cells)
        r_max = min(height, row + sensor_cells + 1)
        c_min = max(0, col - sensor_cells)
        c_max = min(width, col + sensor_cells + 1)

        rr, cc = np.ogrid[r_min:r_max, c_min:c_max]
        dist_sq = (rr - row) ** 2 + (cc - col) ** 2
        covered[r_min:r_max, c_min:c_max] |= (dist_sq <= sensor_cells ** 2)

    return covered
