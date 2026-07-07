import json
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
from std_msgs.msg import String

MATRIX_FILENAME = 'transform_matrix.yaml'


class MatrixPublisher(Node):
    """Standalone publisher for the IFC-to-world transformation matrix.

    Loads the 4x4 transform from config/transform_matrix.yaml and publishes it
    once on /matrix with latched (transient-local) QoS, so any node that
    subscribes later still receives it.
    """

    def __init__(self) -> None:
        super().__init__('matrix_publisher')

        package_share = Path(get_package_share_directory('robot_task'))
        self._matrix_yaml = package_share / 'config' / MATRIX_FILENAME

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._matrix_pub = self.create_publisher(String, '/matrix', latched_qos)

        self._transform_matrix = self._load_transform_matrix()

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

    def publish_matrix(self) -> None:
        payload = {'count': 1, 'matrix': self._transform_matrix.tolist()}
        msg = String()
        msg.data = json.dumps(payload)
        self._matrix_pub.publish(msg)
        self.get_logger().info('Published transformation matrix on /matrix.')


def main() -> None:
    rclpy.init()
    node = MatrixPublisher()
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
