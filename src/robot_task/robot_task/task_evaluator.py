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
        self._space_sub = self.create_subscription(
            String, '/ifc/spaces', self._on_spaces, latched_qos)
        self._wall_sub = self.create_subscription(
            String, '/ifc/walls', self._on_walls, latched_qos)
        self._mep_elements_sub = self.create_subscription(
            String, '/ifc/mep_elements', self._on_mep_elements, latched_qos)
        self._selected_element_sub = self.create_subscription(
            String, '/task/selected_element', self._on_selected_element, latched_qos)

        self._spaces: list[dict] | None = None
        self._walls: list[dict] | None = None
        self._target_wall: str | None = None
        self._filtered_elements: list[dict] | None = None

    def _on_spaces(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /ifc/spaces JSON: {exc}')
            return
        self._spaces = payload.get('spaces', [])
        if self._spaces is None:
            self.get_logger().warn('/ifc/spaces had no "spaces" field.')
            return
        self.get_logger().info(
            f'Received {len(self._spaces)} spaces on /ifc/spaces.')

    def _on_walls(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /ifc/walls JSON: {exc}')
            return
        self._walls = payload.get('walls', [])
        if self._walls is None:
            self.get_logger().warn('/ifc/walls had no "walls" field.')
            return
        self.get_logger().info(
            f'Received {len(self._walls)} walls on /ifc/walls.')

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
        self.get_logger().info(
            f'Received {len(self._mep_elements)} MEP elements on /ifc/mep_elements.')
        self._refresh_filtered_elements()

    def _on_selected_element(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /task/selected_element JSON: {exc}')
            return
        task = payload.get('selected_element') or {}
        wall = task.get('wall') or {}
        self._target_wall = wall.get('id')
        if self._target_wall is None:
            self.get_logger().warn(
                '/task/selected_element had no "selected_element.wall.id" field.')
            return
        self.get_logger().info(
            f'Received selected element on /task/selected_element; wall_id={self._target_wall}')
        self.publish_target_wall()
        self._refresh_filtered_elements()

    def _refresh_filtered_elements(self) -> None:
        """Filter MEP elements down to those hosted by any space that the
        target wall bounds. Wall -> spaces (via /ifc/walls), then keep
        elements whose space_id is in that set.
        """
        if self._target_wall is None:
            return
        if self._mep_elements is None:
            return
        if self._walls is None:
            self.get_logger().warn(
                '/ifc/walls not yet received; cannot filter elements.')
            return
        wall = next(
            (w for w in self._walls if w.get('id') == self._target_wall), None)
        if wall is None:
            self.get_logger().warn(
                f'Wall id={self._target_wall} not found in /ifc/walls; '
                'filtered list will be empty.')
            self._filtered_elements = []
            self.publish_filtered_elements()
            return
        target_space_ids = {
            s.get('id')
            for s in (wall.get('space') or [])
            if s.get('id')
        }
        self._filtered_elements = [
            mep for mep in self._mep_elements
            if (mep.get('space') or {}).get('id') in target_space_ids
        ]
        self.publish_filtered_elements()

    def publish_filtered_elements(self) -> None:
        payload = {
            'count': len(self._filtered_elements),
            'filtered_elements': self._filtered_elements,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._filtered_elements_pub.publish(msg)
        self.get_logger().info(
            f'Published {len(self._filtered_elements)} filtered elements on /task/filtered_elements: {payload}')

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
