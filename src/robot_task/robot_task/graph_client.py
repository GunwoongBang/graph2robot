import json
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    HistoryPolicy,
    ReliabilityPolicy,
)
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import Trigger


class GraphClient(Node):
    def __init__(self) -> None:
        super().__init__('graph_client')

        self._client = self.create_client(
            Trigger, '/graph/list_mep_elements')

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._elements_pub = self.create_publisher(
            String, '/mep_elements', latched_qos)

    def request_mep_elements(self, wait_timeout_sec: float = 10.0) -> None:
        self.get_logger().info(
            'Waiting for /graph/list_mep_elements service...')
        if not self._client.wait_for_service(timeout_sec=wait_timeout_sec):
            self.get_logger().error(
                f'Service /graph/list_mep_elements not available after {wait_timeout_sec}s; abort.')
            return
        future = self._client.call_async(Trigger.Request())
        future.add_done_callback(self._handle_response)

    def _handle_response(self, future: Future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Service call failed: {exc}')
            return
        if result is None or not result.success:
            reason = result.message if result is not None else 'no result'
            self.get_logger().error(f'Service returned failure: {reason}')
            return

        try:
            elements = json.loads(result.message)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid JSON in service response: {exc}')
            return

        msg = String()
        msg.data = json.dumps(elements)
        self._elements_pub.publish(msg)
        self.get_logger().info(
            f'Published {len(elements)} MEP elements on /mep_elements.')


def main() -> None:
    rclpy.init()
    node = GraphClient()
    try:
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
