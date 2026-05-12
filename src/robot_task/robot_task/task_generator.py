import json
import yaml
import rclpy
import numpy as np

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class TaskGenerator(Node):
    def __init__(self) -> None:
        super().__init__('task_generator')

        package_share = Path(get_package_share_directory('robot_task'))
        self._matrix_yaml = package_share / 'config' / 'transform_matrix.yaml'

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._selected_task_sub = self.create_subscription(
            String, '/task/selected_task', self._on_selected_task, latched_qos)
        self._walls_sub = self.create_subscription(
            String, '/task/walls', self._on_walls, latched_qos)
        self._drilling_position_pub = self.create_publisher(
            String, '/task/drilling_position', latched_qos)

        self._selected_element: dict | None = None
        self._walls: list[dict] | None = None
        self._transform_matrix = self._load_transform_matrix()

    def _on_selected_task(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /task/selected_task JSON: {exc}')
            return
        self._selected_element = payload.get('task')
        if self._selected_element is None:
            self.get_logger().warn('/task/selected_task had no "task" field.')
            return
        self.get_logger().info(
            f"Selected task: name='{self._selected_element.get('name', '')}' "
            f"id={self._selected_element.get('id', '')}")
        self._request_wall_info()

    def _load_transform_matrix(self) -> np.ndarray:
        if not self._matrix_yaml.exists():
            self.get_logger().warning(
                f'Transform file not found: {self._matrix_yaml}; using identity.')
            return np.eye(4, dtype=float)
        try:
            with open(self._matrix_yaml, 'r') as f:
                cfg = yaml.safe_load(f)
            mat = np.array(cfg.get('matrix', np.eye(4)))
            if mat.shape != (4, 4):
                raise ValueError('transform matrix must be 4x4')
            return mat.astype(float)
        except Exception as exc:
            self.get_logger().error(
                f'Failed to load transform matrix: {exc}; using identity.')
            return np.eye(4, dtype=float)

    def _on_walls(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /task/walls JSON: {exc}')
            return
        self._walls = payload.get('walls', [])
        self.get_logger().info(
            f'Received {len(self._walls)} walls on /task/walls.')

    def _request_wall_info(self) -> None:
        wall_ref = (self._selected_element or {}).get('wall') or {}
        wall_id = wall_ref.get('id')
        if not wall_id:
            self.get_logger().warn(
                'Selected task has no wall id; skipping wall lookup.')
            return
        if self._walls is None:
            self.get_logger().warn(
                '/walls not yet received; cannot look up wall.')
            return
        wall = next((w for w in self._walls if w.get('id') == wall_id), None)
        if wall is None:
            self.get_logger().warn(
                f'Wall id={wall_id} not found in cached /walls.')
            return
        self.get_logger().info(
            f"Looked up wall: name='{wall.get('name', '')}' id={wall.get('id', '')}")


def main() -> None:
    rclpy.init()
    node = TaskGenerator()
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
