import json
import os
import yaml
import rclpy
import numpy as np

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


class MatrixPublisher(Node):
    def __init__(self) -> None:
        super().__init__('matrix_publisher')

        # load transform matrix (IFC virtual coord. -> PCD real-world coord.)
        self.transform_matrix = self._load_transform_matrix()
        self.publisher = self.create_publisher(String, '/matrix', 10)

    def _load_transform_matrix(self) -> np.ndarray | None:
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

    def publish_matrix(self):
        # publish the transform matrix to a ROS topic
        if self.transform_matrix is None:
            return

        payload = {
            'count': 1,
            'matrix': self.transform_matrix.tolist(),
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.publisher.publish(msg)
        print(f"Published transformation matrix:\n{self.transform_matrix}")
        self.get_logger().info(f"Published transformation matrix.")


def main() -> None:
    rclpy.init()
    node = MatrixPublisher()
    try:
        node.publish_matrix()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
