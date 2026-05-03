from __future__ import annotations

import json
import os
import yaml
import rclpy
import numpy as np

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from typing import Any
from std_msgs.msg import String


class TaskManager(Node):
    def __init__(self) -> None:
        super().__init__('task_manager')

        # Match graph_client's /mep_elements QoS so we still receive the latest
        # payload even when started after graph_client.
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._elements_sub = self.create_subscription(
            String, '/mep_elements', self._handle_mep_elements, latched_qos)
        self._matrix_pub = self.create_publisher(String, '/matrix', 10)

        self._mep_elements: list[dict[str, Any]] | None = None
        self._transform_matrix = self._load_transform_matrix()

    def _load_transform_matrix(self) -> np.ndarray:
        transform_file = os.path.join(
            get_package_share_directory('robot_task'),
            'config',
            'transform_matrix.yaml',
        )
        if not os.path.exists(transform_file):
            self.get_logger().warning(
                f'Transform file not found: {transform_file}; using identity.')
            return np.eye(4, dtype=float)
        try:
            with open(transform_file, 'r') as f:
                cfg = yaml.safe_load(f)
            mat = np.array(cfg.get('matrix', np.eye(4)))
            if mat.shape != (4, 4):
                raise ValueError('transform matrix must be 4x4')
            return mat.astype(float)
        except Exception as exc:
            self.get_logger().error(
                f'Failed to load transform matrix: {exc}; using identity.')
            return np.eye(4, dtype=float)

    def _handle_mep_elements(self, msg: String) -> None:
        try:
            self._mep_elements = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /mep_elements JSON: {exc}')
            return
        self.get_logger().info(
            f'Received {len(self._mep_elements)} MEP elements from graph_client.')

    def publish_matrix(self) -> None:
        payload = {
            'count': 1,
            'matrix': self._transform_matrix.tolist(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._matrix_pub.publish(msg)
        self.get_logger().info('Published transformation matrix.')

    def publish_task(self) -> None:
        payload = {

        }


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
