import csv
import json
import numpy as np
import rclpy

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

# Both constants are arbitrary
# With UR5e's URDF with LLM(?), it can be more generalized and
SPHERE_RADIUS = 0.8  # meters; UR5e reach by default.
CENTER_Z_OFFSET = 0.529  # meters; height where the shoulder joint sits
CSV_FILENAME = 'cloudGlobal_cleaned_excluded.csv'


class TaskRepresenter(Node):
    def __init__(self) -> None:
        super().__init__('task_representer')

        package_share = Path(get_package_share_directory('robot_rviz'))
        self._csv_path = package_share / 'models' / CSV_FILENAME
        self._wall_indices: dict[str, np.ndarray] = self._load_wall_index_map()

        # === Topic publishers and subscribers ===
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Publishers
        self._representation_pub = self.create_publisher(
            PointCloud2, '/task/representation', latched_qos)
        # Subscribers
        self._cloud_sub = self.create_subscription(
            PointCloud2, '/cloud', self._on_cloud, latched_qos)
        self._matrix_sub = self.create_subscription(
            String, '/matrix', self._on_matrix, latched_qos)
        self._target_position_sub = self.create_subscription(
            String, '/task/target_position', self._on_target_position, latched_qos)
        self._filtered_elements_sub = self.create_subscription(
            String, '/task/filtered_elements', self._on_filtered_elements, latched_qos)
        self._target_wall_sub = self.create_subscription(
            String, '/task/target_wall', self._on_target_wall, latched_qos)

        self._cloud: np.ndarray | None = None
        self._frame_id: str = 'world'
        self._target_position: dict | None = None
        self._matrix: np.ndarray | None = None
        self._filtered_elements: list[dict] | None = None
        self._target_wall: str | None = None

    def _load_wall_index_map(self) -> dict[str, np.ndarray]:
        if not self._csv_path.exists():
            self.get_logger().error(f'CSV not found at {self._csv_path}.')
            return {}
        buckets: dict[str, list[int]] = {}
        with open(self._csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for cloud_idx, row in enumerate(reader):
                wall_id = row.get('ifc_global_id') or ''
                if not wall_id:
                    continue
                buckets.setdefault(wall_id, []).append(cloud_idx)
        out = {k: np.array(v, dtype=np.int64) for k, v in buckets.items()}
        total = sum(len(v) for v in out.values())
        self.get_logger().info(
            f'Loaded wall->indices map: {len(out)} walls, {total} points.')
        return out

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self._cloud is not None:
            return
        pts = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        self._cloud = np.array(
            [(p[0], p[1], p[2]) for p in pts], dtype=np.float32)
        self._frame_id = msg.header.frame_id
        self.get_logger().info(
            f'Received cloud with {len(self._cloud)} points on /cloud (frame_id={self._frame_id}).')
        self._try_process()

    def _on_matrix(self, msg: String) -> None:
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
        self._matrix = mat
        self.get_logger().info('Received transform matrix on /matrix.')
        self._try_process()

    def _on_target_position(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /task/target_position JSON: {exc}')
            return
        self._target_position = payload.get('target_position')
        if self._target_position is None:
            self.get_logger().warn(
                '/task/target_position had no "target_position" field.')
            return
        self.get_logger().info(
            f"Target position (IFC frame): "
            f"pos=({self._target_position.get('x', 0.0):.3f}, "
            f"{self._target_position.get('y', 0.0):.3f}, "
            f"{self._target_position.get('z', 0.0):.3f}) "
            f"heading=({self._target_position.get('hx', 0.0):.3f}, "
            f"{self._target_position.get('hy', 0.0):.3f})")
        self._try_process()

    def _on_filtered_elements(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /task/filtered_elements JSON: {exc}')
            return
        self._filtered_elements = payload.get('filtered_elements', [])
        if self._filtered_elements is None:
            self.get_logger().warn(
                '/task/filtered_elements had no "filtered_elements" field.')
            return
        self.get_logger().info(
            f'Received {len(self._filtered_elements)} filtered elements on /task/filtered_elements.')

    def _on_target_wall(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /task/target_wall JSON: {exc}')
            return
        self._target_wall = payload.get('wall_id')
        if self._target_wall is None:
            self.get_logger().warn(
                '/task/target_wall had no "wall_id" field.')
            return
        self.get_logger().info(
            f'Received target wall on /task/target_wall: {self._target_wall}')
        self._try_process()

    def _try_process(self) -> None:
        if (self._cloud is None or self._matrix is None
                or self._target_position is None or self._target_wall is None):
            return
        if len(self._cloud) == 0:
            self.get_logger().warn('Empty point cloud; skipping.')
            return

        # IFC -> cloud frame for the sphere center.
        ifc_pos = np.array([
            float(self._target_position.get('x', 0.0)),
            float(self._target_position.get('y', 0.0)),
            float(self._target_position.get('z', 0.0)),
            1.0,
        ])
        center = (self._matrix @ ifc_pos)[:3].astype(np.float32)
        center[2] += CENTER_Z_OFFSET  # Adjust for shoulder joint height

        wall_idx = self._wall_indices.get(self._target_wall)
        if wall_idx is None or len(wall_idx) == 0:
            self.get_logger().warn(
                f'No points on wall {self._target_wall} in CSV map.')
            return

        wall_pts = self._cloud[wall_idx]
        dists_sq = np.sum((wall_pts - center) ** 2, axis=1)
        in_sphere = dists_sq <= (SPHERE_RADIUS * SPHERE_RADIUS)
        selected = wall_pts[in_sphere]

        header = self._make_header()
        cloud_msg = point_cloud2.create_cloud_xyz32(
            header, selected.tolist())
        self._representation_pub.publish(cloud_msg)
        self.get_logger().info(
            f'Published {len(selected)} task-representation points on '
            f'/task/representation (wall={self._target_wall}, radius={SPHERE_RADIUS}).')

    def _make_header(self):
        from std_msgs.msg import Header
        h = Header()
        h.frame_id = self._frame_id
        h.stamp = self.get_clock().now().to_msg()
        return h


def main() -> None:
    rclpy.init()
    node = TaskRepresenter()
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
