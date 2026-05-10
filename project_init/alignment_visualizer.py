import open3d as o3d
import numpy as np
import yaml
import os
import glob


def load_ifc_point_cloud(mesh_dir):
    print(f"Loading IFC meshes from {mesh_dir}...")
    mesh_files = glob.glob(os.path.join(mesh_dir, "*.obj"))
    merged_pcd = o3d.geometry.PointCloud()
    for mesh_file in mesh_files:
        if "IfcSlab" in os.path.basename(mesh_file):
            continue
        mesh = o3d.io.read_triangle_mesh(mesh_file)
        if not mesh.has_vertices():
            continue
        area = mesh.get_surface_area()
        if area > 0:
            num_points = max(int(area * 500), 100)
            pcd = mesh.sample_points_uniformly(number_of_points=num_points)
            merged_pcd += pcd
    return merged_pcd


def visualize_alignment():
    ws_root = os.path.abspath(os.path.join(
        os.path.dirname(__file__), ".."))
    ifc_mesh_dir = os.path.join(
        ws_root, "src/robot_gazebo/models/worlds/meshes")
    pcd_file = os.path.join(
        ws_root, "src/robot_rviz/models/cloudGlobal_cleaned_excluded.pcd")
    transform_file = os.path.join(
        ws_root, "src/robot_task/config/transform_matrix.yaml")

    if not os.path.exists(transform_file):
        print(f"Error: Could not find matrix file at {transform_file}")
        return

    print("Loading transformation matrix...")
    with open(transform_file, "r") as f:
        config = yaml.safe_load(f)
    transformation = np.array(config["matrix"])

    print("Loading source (IFC)...")
    source_pcd = load_ifc_point_cloud(ifc_mesh_dir)
    # Paint source IFC Red
    source_pcd.paint_uniform_color([1.0, 0.0, 0.0])

    print("Loading target (PCD)...")
    target_pcd = o3d.io.read_point_cloud(pcd_file)
    print(f"Loading PCD from {pcd_file}...")
    # Paint target LiDAR Blue
    target_pcd.paint_uniform_color([0.0, 0.0, 1.0])

    print("Applying transformation...")
    source_pcd.transform(transformation)

    print("Opening Open3D Visualization... (You can use your mouse to rotate/zoom)")
    print("Close the Open3D window to exit.")

    # Render
    o3d.visualization.draw_geometries(
        [source_pcd, target_pcd],
        window_name="Aligned: IFC (Red) + LiDAR PCD (Blue)",
        width=1280,
        height=720
    )
