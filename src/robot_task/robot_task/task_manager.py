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


class TaskManager(Node):
    def __init__(self) -> None:
        super().__init__('task_manager')

        package_share = Path(get_package_share_directory('robot_task'))
        self._matrix_yaml = package_share / 'config' / MATRIX_FILENAME

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Publishers
        self._space_pub = self.create_publisher(
            String, '/ifc/spaces', latched_qos)
        self._wall_pub = self.create_publisher(
            String, '/ifc/walls', latched_qos)
        self._layer_pub = self.create_publisher(
            String, '/ifc/layers', latched_qos)
        self._mep_element_pub = self.create_publisher(
            String, '/ifc/mep_elements', latched_qos)
        self._matrix_pub = self.create_publisher(
            String, '/matrix', latched_qos)
        # Subscribers
        self._space_sub = self.create_subscription(
            String, '/spaces', self._on_spaces, latched_qos)
        self._wall_sub = self.create_subscription(
            String, '/walls', self._on_walls, latched_qos)
        self._layer_sub = self.create_subscription(
            String, '/layers', self._on_layers, latched_qos)
        self._mep_element_sub = self.create_subscription(
            String, '/mep_elements', self._on_mep_elements, latched_qos)

        self._mep_elements: list[dict] | None = None
        self._walls: list[dict] | None = None
        self._layers: list[dict] | None = None
        self._transform_matrix = self._load_transform_matrix()

    def _on_spaces(self, msg: String) -> None:
        try:
            self._spaces = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /spaces JSON: {exc}')
            return
        self.get_logger().info(
            f'Received {len(self._spaces)} spaces on /spaces.')
        self.publish_spaces()

    def _on_walls(self, msg: String) -> None:
        try:
            self._walls = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /walls JSON: {exc}')
            return
        self.get_logger().info(
            f'Received {len(self._walls)} walls on /walls.')
        self.publish_walls()

    def _on_layers(self, msg: String) -> None:
        try:
            self._layers = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /layers JSON: {exc}')
            return
        self.get_logger().info(
            f'Received {len(self._layers)} layers on /layers.')
        self.publish_layers()

    def _on_mep_elements(self, msg: String) -> None:
        try:
            self._mep_elements = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /mep_elements JSON: {exc}')
            return
        self.get_logger().info(
            f'Received {len(self._mep_elements)} MEP elements on /mep_elements.')
        self.publish_mep_elements()

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

    def publish_spaces(self) -> None:
        if not self._spaces:
            return
        payload = {
            'count': len(self._spaces),
            'spaces': self._spaces,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._space_pub.publish(msg)
        self.get_logger().info(
            f'Published {len(self._spaces)} spaces on /spaces.')

    def publish_walls(self) -> None:
        if not self._walls:
            return
        payload = {
            'count': len(self._walls),
            'walls': self._walls,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._wall_pub.publish(msg)
        self.get_logger().info(
            f'Published {len(self._walls)} walls on /ifc/walls.')

    def publish_layers(self) -> None:
        if not self._layers:
            return
        payload = {
            'count': len(self._layers),
            'layers': self._layers,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._layer_pub.publish(msg)
        self.get_logger().info(
            f'Published {len(self._layers)} layers on /ifc/layers.')

    def publish_mep_elements(self) -> None:
        if not self._mep_elements:
            return
        payload = {
            'count': len(self._mep_elements),
            'mep_elements': self._mep_elements,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._mep_element_pub.publish(msg)
        self.get_logger().info(
            f'Published {len(self._mep_elements)} MEP elements on /ifc/mep_elements.')

    def publish_matrix(self) -> None:
        payload = {
            'count': 1,
            'matrix': self._transform_matrix.tolist(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._matrix_pub.publish(msg)
        self.get_logger().info('Published transformation matrix on /matrix.')


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
