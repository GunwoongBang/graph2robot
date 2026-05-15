import json
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class TaskGenerator(Node):
    def __init__(self) -> None:
        super().__init__('task_generator')

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._selected_task_sub = self.create_subscription(
            String, '/task/selected_task', self._on_selected_task, latched_qos)
        self._walls_sub = self.create_subscription(
            String, '/task/walls', self._on_walls, latched_qos)
        self._drilling_position_pub = self.create_publisher(
            String, '/task/drilling_position', latched_qos)

        self._selected_element: dict | None = None
        self._walls: list[dict] | None = None

    def _on_selected_task(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /task/selected_task JSON: {exc}')
            return
        self._selected_element = payload.get('task')
        if self._selected_element is None:
            self.get_logger().warn('/task/selected_task had no "task" field.')
            return
        self.get_logger().info(
            f"Selected task: name='{self._selected_element.get('name', '')}' "
            f"id={self._selected_element.get('id', '')}")
        self.publish_drilling_position()

    def _on_walls(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /task/walls JSON: {exc}')
            return
        self._walls = payload.get('walls', [])
        self.get_logger().info(
            f'Received {len(self._walls)} walls on /task/walls.')

    def publish_drilling_position(self) -> None:
        if self._selected_element is None:
            return
        if self._walls is None:
            self.get_logger().warn(
                '/task/walls not yet received; cannot compute drilling position.')
            return

        center = self._selected_element.get('center')
        face = self._selected_element.get('face')
        wall_id = (self._selected_element.get('wall') or {}).get('id')

        if center is None or len(center) != 3:
            self.get_logger().warn(
                'Selected task has no valid center; skipping.')
            return
        if face is None or len(face) != 3:
            self.get_logger().warn(
                'Selected task has no valid face; skipping.')
            return
        if not wall_id:
            self.get_logger().warn(
                'Selected task has no wall id; skipping.')
            return

        wall = next((w for w in self._walls if w.get('id') == wall_id), None)
        if wall is None:
            self.get_logger().warn(
                f'Wall id={wall_id} not found in cached /task/walls.')
            return
        axis2 = wall.get('axis2')
        if axis2 is None or len(axis2) != 3:
            self.get_logger().warn(
                f'Wall id={wall_id} has no valid axis2; skipping.')
            return

        # IFC center is in millimeters; compute robot base position in IFC
        # frame (meters) at floor level (z = 0).
        cx = center[0] / 1000.0
        cy = center[1] / 1000.0
        offset = 0.6  # distance from wall to drilling position, 0.4 causes collisions

        if abs(axis2[0]) == 1.0:
            if face[0] == 1.0:
                ifc_xy = (cx + offset, cy)
            elif face[0] == -1.0:
                ifc_xy = (cx - offset, cy)
            else:
                self.get_logger().warn(
                    f'face={face} not aligned with axis2={axis2}; skipping.')
                return
        elif abs(axis2[1]) == 1.0:
            if face[1] == 1.0:
                ifc_xy = (cx, cy + offset)
            elif face[1] == -1.0:
                ifc_xy = (cx, cy - offset)
            else:
                self.get_logger().warn(
                    f'face={face} not aligned with axis2={axis2}; skipping.')
                return
        else:
            self.get_logger().warn(
                f'axis2={axis2} not aligned with x or y axis; skipping.')
            return

        # Heading - opposite of `face`, so the robot looks at the
        # wall.
        hx = -float(face[0])
        hy = -float(face[1])

        payload = {
            'count': 1,
            'drilling_position': {
                'x': float(ifc_xy[0]),
                'y': float(ifc_xy[1]),
                'z': 0.0,
                'hx': hx,
                'hy': hy,
            },
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._drilling_position_pub.publish(msg)
        self.get_logger().info(
            f'Published IFC-frame drilling position on /task/drilling_position: '
            f'pos=({ifc_xy[0]:.3f}, {ifc_xy[1]:.3f}, 0.000) heading=({hx:.3f}, {hy:.3f}).')


def main() -> None:
    rclpy.init()
    node = TaskGenerator()
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
