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
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import Trigger

MATRIX_FILENAME = 'transform_matrix.yaml'


class TaskManager(Node):
    def __init__(self) -> None:
        super().__init__('task_manager')

        package_share = Path(get_package_share_directory('robot_task'))
        self._matrix_yaml = package_share / 'config' / MATRIX_FILENAME

        # Clients
        self._space_cli = self.create_client(Trigger, '/graph/list_spaces')
        self._wall_cli = self.create_client(Trigger, '/graph/list_walls')
        self._layer_cli = self.create_client(Trigger, '/graph/list_layers')
        self._mep_cli = self.create_client(Trigger, '/graph/list_mep_elements')

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

        self._transform_matrix = self._load_transform_matrix()

    def request_graph_data(self, wait_timeout_sec: float = 10.0) -> None:
        for cli, cb, name in [
            (self._space_cli, self._on_spaces, '/graph/list_spaces'),
            (self._wall_cli, self._on_walls, '/graph/list_walls'),
            (self._layer_cli, self._on_layers, '/graph/list_layers'),
            (self._mep_cli, self._on_mep_elements,
             '/graph/list_mep_elements'),
        ]:
            if not cli.wait_for_service(timeout_sec=wait_timeout_sec):
                self.get_logger().error(
                    f'Service {name} unavailable after {wait_timeout_sec}s; skipping.')
                continue
            cli.call_async(Trigger.Request()).add_done_callback(cb)

    def _parse(self, future: Future, service: str) -> list | None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'{service} call failed: {exc}')
            return None
        if result is None or not result.success:
            reason = result.message if result is not None else 'no result'
            self.get_logger().error(f'{service} returned failure: {reason}')
            return None
        try:
            return json.loads(result.message)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid JSON from {service}: {exc}')
            return None

    def _on_spaces(self, future: Future) -> None:
        data = self._parse(future, '/graph/list_spaces')
        if data is None:
            return
        msg = String()
        msg.data = json.dumps({'count': len(data), 'spaces': data})
        self._space_pub.publish(msg)
        self.get_logger().info(f'Published {len(data)} spaces on /ifc/spaces.')

    def _on_walls(self, future: Future) -> None:
        data = self._parse(future, '/graph/list_walls')
        if data is None:
            return
        msg = String()
        msg.data = json.dumps({'count': len(data), 'walls': data})
        self._wall_pub.publish(msg)
        self.get_logger().info(f'Published {len(data)} walls on /ifc/walls.')

    def _on_layers(self, future: Future) -> None:
        data = self._parse(future, '/graph/list_layers')
        if data is None:
            return
        msg = String()
        msg.data = json.dumps({'count': len(data), 'layers': data})
        self._layer_pub.publish(msg)
        self.get_logger().info(f'Published {len(data)} layers on /ifc/layers.')

    def _on_mep_elements(self, future: Future) -> None:
        data = self._parse(future, '/graph/list_mep_elements')
        if data is None:
            return
        msg = String()
        msg.data = json.dumps({'count': len(data), 'mep_elements': data})
        self._mep_element_pub.publish(msg)
        self.get_logger().info(
            f'Published {len(data)} MEP elements on /ifc/mep_elements.')

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
    node = TaskManager()
    try:
        node.publish_matrix()
        node.request_graph_data()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
