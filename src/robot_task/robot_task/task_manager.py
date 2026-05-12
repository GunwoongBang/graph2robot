import json
import os
import yaml
import rclpy
import numpy as np

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from typing import Any
from std_msgs.msg import String


class TaskManager(Node):
    def __init__(self) -> None:
        super().__init__('task_manager')

        package_share = Path(get_package_share_directory('robot_task'))
        self._matrix_yaml = package_share / 'config' / 'transform_matrix.yaml'

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._mep_elements_sub = self.create_subscription(
            String, '/mep_elements', self._on_mep_elements, latched_qos)
        self._wall_sub = self.create_subscription(
            String, '/walls', self._on_walls, latched_qos)
        self._mep_elements_pub = self.create_publisher(
            String, '/task/mep_elements', latched_qos)
        self._walls_pub = self.create_publisher(
            String, '/task/walls', latched_qos)
        self._matrix_pub = self.create_publisher(
            String, '/matrix', latched_qos)

        self._mep_elements: list[dict[str, Any]] | None = None
        self._walls: list[dict[str, Any]] | None = None
        self._transform_matrix = self._load_transform_matrix()

    def _on_mep_elements(self, msg: String) -> None:
        try:
            self._mep_elements = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /mep_elements JSON: {exc}')
            return
        self.get_logger().info(
            f'Received {len(self._mep_elements)} MEP elements on /mep_elements.')
        self.publish_task()

    def _on_walls(self, msg: String) -> None:
        try:
            self._walls = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /walls JSON: {exc}')
            return
        self.get_logger().info(
            f'Received {len(self._walls)} walls on /walls.')
        self.publish_walls()

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

    def publish_task(self) -> None:
        if not self._mep_elements:
            return
        payload = {
            'count': len(self._mep_elements),
            'tasks': self._mep_elements,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._mep_elements_pub.publish(msg)
        self.get_logger().info(
            f'Published {len(self._mep_elements)} MEP elements on /task/mep_elements.')

    def publish_walls(self) -> None:
        if not self._walls:
            return
        payload = {
            'count': len(self._walls),
            'walls': self._walls,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._walls_pub.publish(msg)
        self.get_logger().info(
            f'Published {len(self._walls)} walls on /task/walls.')

    def publish_matrix(self) -> None:
        payload = {
            'count': 1,
            'matrix': self._transform_matrix.tolist(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._matrix_pub.publish(msg)
        self.get_logger().info('Published transformation matrix.')


def main() -> None:
    rclpy.init()
    node = TaskManager()
    try:
        node.publish_matrix()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
