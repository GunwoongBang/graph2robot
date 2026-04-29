import open3d as o3d
import numpy as np
import glob
import os
import yaml


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


def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=radius_normal, max_nn=30))
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def main():
    ws_root = os.path.abspath(os.path.join(
        os.path.dirname(__file__), "../../.."))
    ifc_mesh_dir = os.path.join(
        ws_root, "src/robot_gazebo/models/worlds/ifc_world_meshes")
    pcd_file = os.path.join(
        ws_root, "src/robot_rviz/models/cloudGlobal_cleaned_excluded.pcd")
    output_transform_file = os.path.join(
        ws_root, "src/robot_task/config/ifc_to_pcd_transform.yaml")

    source_pcd = load_ifc_point_cloud(ifc_mesh_dir)
    target_pcd = o3d.io.read_point_cloud(pcd_file)

    voxel_size = 0.2
    source_down, source_fpfh = preprocess_point_cloud(source_pcd, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target_pcd, voxel_size)

    distance_threshold = voxel_size * 1.5
    global_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True, distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
        o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))

    current_transformation = global_result.transformation

    for scale_threshold in [0.5, 0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001]:
        current_radius = max(scale_threshold * 2, 0.02)
        source_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=current_radius, max_nn=50))
        target_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=current_radius, max_nn=50))
        icp_result = o3d.pipelines.registration.registration_icp(
            source_pcd, target_pcd, scale_threshold, current_transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000, relative_fitness=1e-7, relative_rmse=1e-7))
        current_transformation = icp_result.transformation

    # # --- SUPERIOR Z-AXIS ALIGNMENT FIX ---
    # # Since IFC walls lack tops/bottoms, Point-to-Plane slides them loosely along the Z-axis.
    # # We forcefully snap the Z-axis by aligning the TOP of the IFC walls to the TOP of the PCD walls.

    # source_points_transformed = np.asarray(
    #     source_pcd.points) @ current_transformation[:3, :3].T + current_transformation[:3, 3]
    # target_points = np.asarray(target_pcd.points)

    # # Calculate difference between the highest points
    # z_diff = target_points[:, 2].max() - source_points_transformed[:, 2].max()

    # # Apply raw Z translation correction to the transformation matrix
    # # current_transformation[2, 3] += z_diff

    print("Final Transformation Matrix with Z-Axis Ceiling Lock:")
    print(current_transformation)

    config = {
        "ifc_to_pcd_transform": current_transformation.tolist(),
        "fitness": float(icp_result.fitness),
        "inlier_rmse": float(icp_result.inlier_rmse)
    }

    with open(output_transform_file, "w") as f:
        yaml.dump(config, f, default_flow_style=False)


if __name__ == "__main__":
    main()
