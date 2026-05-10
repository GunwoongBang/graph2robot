import math
import os
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _decompose_matrix(matrix: list[list[float]]) -> tuple[float, float, float, float, float, float]:
    M = np.asarray(matrix, dtype=float)
    x, y, z = float(M[0, 3]), float(M[1, 3]), float(M[2, 3])
    R = M[:3, :3]
    pitch = math.asin(max(-1.0, min(1.0, -float(R[2, 0]))))
    if math.cos(pitch) > 1e-6:
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    else:
        # Gimbal lock: yaw absorbed into roll.
        roll = math.atan2(-float(R[0, 1]), float(R[1, 1]))
        yaw = 0.0
    return x, y, z, roll, pitch, yaw


def _load_matrix(matrix_yaml: str) -> list[list[float]]:
    if not os.path.exists(matrix_yaml):
        print(
            f'[robot_gazebo] WARNING: {matrix_yaml} not found; using identity pose.')
        return np.eye(4).tolist()
    with open(matrix_yaml, 'r') as f:
        cfg = yaml.safe_load(f) or {}
    return cfg.get('matrix', np.eye(4).tolist())


def _patch_ifc_poses(source_sdf: str, pose_str: str) -> str:
    tree = ET.parse(source_sdf)
    root = tree.getroot()
    count = 0
    for model in root.iter('model'):
        if not model.get('name', '').startswith('Ifc'):
            continue
        pose_elem = model.find('pose')
        if pose_elem is None:
            pose_elem = ET.SubElement(model, 'pose')
        pose_elem.text = pose_str
        count += 1

    out = tempfile.NamedTemporaryFile(
        prefix='ifc_world_', suffix='.sdf', delete=False, mode='w', encoding='utf-8')
    out.close()
    tree.write(out.name, xml_declaration=True, encoding='utf-8')
    print(
        f'[robot_gazebo] Patched {count} IFC models with pose [{pose_str}] -> {out.name}')
    return out.name


def generate_launch_description() -> LaunchDescription:
    gazebo_share = get_package_share_directory('robot_gazebo')
    task_share = get_package_share_directory('robot_task')

    source_sdf = os.path.join(
        gazebo_share, 'models', 'worlds', 'ifc_world.sdf')
    matrix_yaml = os.path.join(
        task_share, 'config', 'transform_matrix.yaml')

    x, y, z, roll, pitch, yaw = _decompose_matrix(_load_matrix(matrix_yaml))
    pose_str = f'{x:.6f} {y:.6f} {z:.6f} {roll:.6f} {pitch:.6f} {yaw:.6f}'
    world_sdf = _patch_ifc_poses(source_sdf, pose_str)

    # GAZEBO_MODEL_PATH lets <include><uri>model://...</uri></include> resolve our robots.
    os.environ['GAZEBO_MODEL_PATH'] = os.pathsep.join(filter(None, [
        os.environ.get('GAZEBO_MODEL_PATH', ''),
        os.path.join(gazebo_share, 'models', 'robots'),
    ]))

    # gazebo_ros's launch wrapper loads gazebo_ros_init + gazebo_ros_factory as
    # system plugins, providing /spawn_entity, /delete_entity, /set_entity_state.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'),
            'launch',
            'gazebo.launch.py',
        )),
        launch_arguments={
            'world': world_sdf,
            'verbose': 'true',
        }.items(),
    )

    return LaunchDescription([
        gazebo,
    ])
