import json
import rclpy

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


class GraphClient(Node):
    def __init__(self) -> None:
        super().__init__('graph_client')

        # === Service servers and clients ===
        # Clients
        self._space_cli = self.create_client(
            Trigger, '/graph/list_spaces')
        self._wall_cli = self.create_client(
            Trigger, '/graph/list_walls')
        self._layer_cli = self.create_client(
            Trigger, '/graph/list_layers')
        self._mep_element_cli = self.create_client(
            Trigger, '/graph/list_mep_elements')

        # === Topic publishers and subscribers ===
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Publishers
        self._space_pub = self.create_publisher(
            String, '/spaces', latched_qos)
        self._wall_pub = self.create_publisher(
            String, '/walls', latched_qos)
        self._layer_pub = self.create_publisher(
            String, '/layers', latched_qos)
        self._mep_element_pub = self.create_publisher(
            String, '/mep_elements', latched_qos)

    def request_spaces(self, wait_timeout_sec: float = 10.0) -> None:
        if not self._space_cli.wait_for_service(timeout_sec=wait_timeout_sec):
            self.get_logger().error(
                f'Service /graph/list_spaces not available after {wait_timeout_sec}s; abort.')
            return
        future = self._space_cli.call_async(Trigger.Request())
        future.add_done_callback(self._handle_spaces_response)

    def request_walls(self, wait_timeout_sec: float = 10.0) -> None:
        if not self._wall_cli.wait_for_service(timeout_sec=wait_timeout_sec):
            self.get_logger().error(
                f'Service /graph/list_walls not available after {wait_timeout_sec}s; abort.')
            return
        future = self._wall_cli.call_async(Trigger.Request())
        future.add_done_callback(self._handle_walls_response)

    def request_layers(self, wait_timeout_sec: float = 10.0) -> None:
        if not self._layer_cli.wait_for_service(timeout_sec=wait_timeout_sec):
            self.get_logger().error(
                f'Service /graph/list_layers not available after {wait_timeout_sec}s; abort.')
            return
        future = self._layer_cli.call_async(Trigger.Request())
        future.add_done_callback(self._handle_layers_response)

    def request_mep_elements(self, wait_timeout_sec: float = 10.0) -> None:
        if not self._mep_element_cli.wait_for_service(timeout_sec=wait_timeout_sec):
            self.get_logger().error(
                f'Service /graph/list_mep_elements not available after {wait_timeout_sec}s; abort.')
            return
        future = self._mep_element_cli.call_async(Trigger.Request())
        future.add_done_callback(self._handle_mep_elements_response)

    def _parse_response(self, future: Future) -> list | None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Service call failed: {exc}')
            return None
        if result is None or not result.success:
            reason = result.message if result is not None else 'no result'
            self.get_logger().error(f'Service returned failure: {reason}')
            return None
        try:
            return json.loads(result.message)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid JSON in service response: {exc}')
            return None

    def _handle_spaces_response(self, future: Future) -> None:
        spaces = self._parse_response(future)
        if spaces is None:
            return
        msg = String()
        msg.data = json.dumps(spaces)
        self._space_pub.publish(msg)
        self.get_logger().info(
            f'Published {len(spaces)} spaces on /spaces.')

    def _handle_walls_response(self, future: Future) -> None:
        walls = self._parse_response(future)
        if walls is None:
            return
        msg = String()
        msg.data = json.dumps(walls)
        self._wall_pub.publish(msg)
        self.get_logger().info(
            f'Published {len(walls)} walls on /walls.')

    def _handle_layers_response(self, future: Future) -> None:
        layers = self._parse_response(future)
        if layers is None:
            return
        msg = String()
        msg.data = json.dumps(layers)
        self._layer_pub.publish(msg)
        self.get_logger().info(
            f'Published {len(layers)} layers on /layers.')

    def _handle_mep_elements_response(self, future: Future) -> None:
        elements = self._parse_response(future)
        if elements is None:
            return
        msg = String()
        msg.data = json.dumps(elements)
        self._mep_element_pub.publish(msg)
        self.get_logger().info(
            f'Published {len(elements)} MEP elements on /mep_elements.')


def main() -> None:
    rclpy.init()
    node = GraphClient()
    try:
        node.request_spaces()
        node.request_walls()
        node.request_layers()
        node.request_mep_elements()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
