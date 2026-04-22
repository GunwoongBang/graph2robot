import numpy as np
import open3d as o3d

from pathlib import Path


def get_top_corner_points(z_tolerance=0.01):
    pcd_path = Path("pc_models/cloudGlobal_cleaned_excluded.pcd")

    cloud = o3d.io.read_point_cloud(str(pcd_path))
    points = np.asarray(cloud.points)

    max_z = float(points[:, 2].max())
    top_mask = points[:, 2] >= (max_z - z_tolerance)
    top_indices = np.where(top_mask)[0]

    top_points = points[top_indices]
    x = top_points[:, 0]
    y = top_points[:, 1]
    z = top_points[:, 2]

    remaining = np.ones(len(top_indices), dtype=bool)

    # Normalize top-surface coordinates to [0, 1] so corners are selected by
    # combined x/y extremeness instead of strict lexicographic priority.
    x_min, x_max = float(x.min()), float(x.max())
    y_min, y_max = float(y.min()), float(y.max())
    z_min, z_max = float(z.min()), float(z.max())

    x_span = max(x_max - x_min, 1e-12)
    y_span = max(y_max - y_min, 1e-12)
    z_span = max(z_max - z_min, 1e-12)

    x_norm = (x - x_min) / x_span
    y_norm = (y - y_min) / y_span
    z_norm = (z - z_min) / z_span

    def pick(score):
        candidate_ids = np.where(remaining)[0]
        chosen_local = candidate_ids[np.argmax(score[candidate_ids])]
        remaining[chosen_local] = False
        return chosen_local

    # Add a small z preference so higher points win ties.
    z_weight = 0.1
    p1_local = pick(x_norm + (1.0 - y_norm) + z_weight * z_norm)          # max x, min y
    p2_local = pick((1.0 - x_norm) + y_norm + z_weight * z_norm)          # min x, max y
    p3_local = pick(x_norm + y_norm + z_weight * z_norm)                  # max x, max y
    p4_local = pick((1.0 - x_norm) + (1.0 - y_norm) + z_weight * z_norm)  # min x, min y

    selected = {
        "p1_maxX_minY_maxZ": p1_local,
        "p2_minX_maxY_maxZ": p2_local,
        "p3_maxX_maxY_maxZ": p3_local,
        "p4_minX_minY_maxZ": p4_local,
    }

    result = {}
    for name, local_idx in selected.items():
        global_idx = int(top_indices[local_idx])
        px, py, pz = top_points[local_idx]
        result[name] = {
            "index": global_idx,
            "point": [float(px), float(py), float(pz+0.555)],
        }

    return result


if __name__ == "__main__":
    corners = get_top_corner_points(z_tolerance=0.01)
    print("Top 4 corner points from highest-z region:")
    for name, info in corners.items():
        x, y, z = info["point"]
        print(
            f"{name}: index={info['index']}, x={x:.6f}, y={y:.6f}, z={z:.6f}")
