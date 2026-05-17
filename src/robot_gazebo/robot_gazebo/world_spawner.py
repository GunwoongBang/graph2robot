import json
import math
import xml.etree.ElementTree as ET

import numpy as np
import rclpy

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.task import Future
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose


SDF_VERSION = '1.6'


class WorldSpawner(Node):
    def __init__(self) -> None:
        super().__init__('world_spawner')

        package_share = Path(get_package_share_directory('robot_gazebo'))
        self._sdf_path = package_share / 'models' / 'worlds' / 'ifc_world.sdf'

        self._ifc_models = self._extract_ifc_models()

        # === Service publishers and clients ===
        self._spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')

        # === Topic publishers and subscribers ===
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._matrix_sub = self.create_subscription(
            String, '/matrix', self._on_matrix, latched_qos)

        self._spawned = False
        self._pose: Pose | None = None

    def _extract_ifc_models(self) -> list[tuple[str, str]]:
        if not self._sdf_path.exists():
            self.get_logger().error(f'World SDF not found: {self._sdf_path}.')
            return []
        tree = ET.parse(self._sdf_path)
        root = tree.getroot()
        out: list[tuple[str, str]] = []
        for model in root.iter('model'):
            name = model.get('name', '')
            if not name.startswith('Ifc'):
                continue
            # Drop any embedded <pose> -- /spawn_entity's initial_pose wins.
            for pose in list(model.findall('pose')):
                model.remove(pose)
            sdf = ET.Element('sdf', {'version': SDF_VERSION})
            sdf.append(model)
            out.append((name, ET.tostring(sdf, encoding='unicode')))
        self.get_logger().info(
            f'Extracted {len(out)} IFC models from {self._sdf_path.name}.')
        return out

    def _on_matrix(self, msg: String) -> None:
        if self._spawned:
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /matrix JSON: {exc}')
            return
        mat = np.array(payload.get('matrix'), dtype=float)
        if mat.shape != (4, 4):
            self.get_logger().error(
                f'Matrix shape {mat.shape}, expected (4,4).')
            return

        x, y, z = float(mat[0, 3]), float(mat[1, 3]), float(mat[2, 3])
        R = mat[:3, :3]
        pitch = math.asin(max(-1.0, min(1.0, -float(R[2, 0]))))
        if math.cos(pitch) > 1e-6:
            roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
            yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
        else:
            roll = math.atan2(-float(R[0, 1]), float(R[1, 1]))
            yaw = 0.0

        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.orientation.z = cr * cp * sy - sr * sp * cy
        pose.orientation.w = cr * cp * cy + sr * sp * sy

        self._pose = pose
        self._spawned = True
        self.get_logger().info(
            f'Spawning {len(self._ifc_models)} IFC models at '
            f'pos=({x:.3f}, {y:.3f}, {z:.3f}) '
            f'rpy=({roll:.3f}, {pitch:.3f}, {yaw:.3f}).')
        self._spawn_next(0)

    def _spawn_next(self, idx: int) -> None:
        if idx >= len(self._ifc_models):
            self.get_logger().info('All IFC models spawned.')
            return
        name, xml = self._ifc_models[idx]
        if not self._spawn_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'Service /spawn_entity unavailable; aborting world spawn.')
            return
        req = SpawnEntity.Request()
        req.name = name
        req.xml = xml
        req.initial_pose = self._pose
        future = self._spawn_cli.call_async(req)
        future.add_done_callback(
            lambda f, i=idx: self._after_spawn(f, i))

    def _after_spawn(self, future: Future, idx: int) -> None:
        name = self._ifc_models[idx][0]
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'/spawn_entity for {name} raised: {exc}')
        else:
            if result is None or not result.success:
                reason = result.status_message if result is not None else 'no result'
                self.get_logger().error(
                    f'/spawn_entity for {name} failed: {reason}')
            else:
                self.get_logger().info(f'Spawned {name}.')
        self._spawn_next(idx + 1)


def main() -> None:
    rclpy.init()
    node = WorldSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
