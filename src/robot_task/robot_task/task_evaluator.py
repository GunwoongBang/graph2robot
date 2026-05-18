# Task evaluator queries Neo4j and publishes /task/danger_elements or similar. The robot_rviz visualizer then consumes that too. Clean split between evaluation (robot_task) and visualization (robot_rviz).
import json
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from std_srvs.srv import Trigger
from std_msgs.msg import String


class TaskEvaluator(Node):
    def __init__(self) -> None:
        super().__init__('task_evaluator')

        # === Topic publishers and subscribers ===
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Publishers
        self._filtered_elements_pub = self.create_publisher(
            String, '/task/filtered_elements', latched_qos)
        self._target_wall_pub = self.create_publisher(
            String, '/task/target_wall', latched_qos)
        # Subscribers
        self._mep_elements_sub = self.create_subscription(
            String, '/ifc/mep_elements', self._on_mep_elements, latched_qos)
        self._selected_element_sub = self.create_subscription(
            String, '/task/selected_element', self._on_selected_element, latched_qos)

        self._target_wall: list[dict] | None = None
        self._filtered_elements: list[dict] | None = None

    def _on_mep_elements(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /ifc/mep_elements JSON: {exc}')
            return
        self._mep_elements = payload.get('mep_elements', [])
        if self._mep_elements is None:
            self.get_logger().warn(
                '/ifc/mep_elements had no "mep_elements" field.')
            return
        # Put all the mep elements for testing
        # Later, it should be filtered based on the selected task
        self._filtered_elements = self._mep_elements
        self.get_logger().info(
            f'Received {len(self._mep_elements)} MEP elements on /ifc/mep_elements.')
        self.publish_filtered_elements()

    def _on_selected_element(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /task/selected_element JSON: {exc}')
            return
        task = payload.get('task') or {}
        wall = task.get('wall') or {}
        self._target_wall = wall.get('id')
        if self._target_wall is None:
            self.get_logger().warn(
                '/task/selected_element had no task.wall.id field.')
            return
        self.get_logger().info(
            f'Received selected element on /task/selected_element; wall_id={self._target_wall}')
        self.publish_target_wall()

    def publish_filtered_elements(self) -> None:
        payload = {
            'count': len(self._filtered_elements),
            'filtered_elements': self._filtered_elements,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._filtered_elements_pub.publish(msg)
        self.get_logger().info(
            f'Published {len(self._filtered_elements)} filtered elements on /task/filtered_elements.')

    def publish_target_wall(self) -> None:
        payload = {
            'count': 1,
            'wall_id': self._target_wall,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._target_wall_pub.publish(msg)
        self.get_logger().info(
            f'Published target wall on /task/target_wall.')


def main() -> None:
    rclpy.init()
    node = TaskEvaluator()
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
