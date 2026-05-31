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

        # === Topic publishers and subscribers ===
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Publishers
        self._target_position_pub = self.create_publisher(
            String, '/robot/target_position', latched_qos)
        self._target_point_pub = self.create_publisher(
            String, '/task/target_point', latched_qos)
        # Subscribers
        self._selected_element_sub = self.create_subscription(
            String, '/task/selected_element', self._on_selected_element, latched_qos)
        self._wall_sub = self.create_subscription(
            String, '/ifc/walls', self._on_walls, latched_qos)

        self._selected_element: dict | None = None
        self._walls: list[dict] | None = None

    def _on_selected_element(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /task/selected_element JSON: {exc}')
            return
        self._selected_element = payload.get('selected_element')
        if self._selected_element is None:
            self.get_logger().warn('/task/selected_element had no "selected_element" field.')
            return
        self.get_logger().info(
            f"Selected element: name='{self._selected_element.get('name', '')}' "
            f"id={self._selected_element.get('id', '')}")
        self.publish_target_position()
        self.publish_target_point()

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

    def publish_target_position(self) -> None:
        if self._selected_element is None:
            return
        if self._walls is None:
            self.get_logger().warn(
                '/ifc/walls not yet received; cannot compute drilling position.')
            return

        wall_obj = self._selected_element.get('wall') or {}
        center = wall_obj.get('center')
        face = self._selected_element.get('face')
        wall_id = wall_obj.get('id')

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
                f'Wall id={wall_id} not found in cached /ifc/walls.')
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
            'target_position': {
                'x': float(ifc_xy[0]),
                'y': float(ifc_xy[1]),
                'z': 0.0,
                'hx': hx,
                'hy': hy,
            },
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._target_position_pub.publish(msg)
        self.get_logger().info(
            f'Published IFC-frame target position on /task/target_position: '
            f'pos=({ifc_xy[0]:.3f}, {ifc_xy[1]:.3f}, 0.000) heading=({hx:.3f}, {hy:.3f}).')

    def publish_target_point(self) -> None:
        """Drill entry point as the projection of MEP center onto wall surface.

        Tangential alignment of the drill tip matches the MEP element's own
        geometric center exactly; only the wall-normal coordinate is pinned to
        the wall outer surface. This removes any drift that would come from
        relying on BIM-precomputed penetrationCenter for tangential coords.
        """
        if self._selected_element is None:
            return
        mep_center = self._selected_element.get('center')
        face = self._selected_element.get('face')
        wall_obj = self._selected_element.get('wall') or {}
        penetration_center = wall_obj.get('center')
        length = wall_obj.get('length')

        if mep_center is None or len(mep_center) != 3:
            self.get_logger().warn(
                'target_point: missing/invalid mep.center; skipping.')
            return
        if face is None or len(face) != 3:
            self.get_logger().warn(
                'target_point: missing/invalid face; skipping.')
            return
        if penetration_center is None or len(penetration_center) != 3:
            self.get_logger().warn(
                'target_point: missing/invalid wall.center; skipping.')
            return
        if length is None:
            self.get_logger().warn(
                'target_point: missing wall.length; skipping.')
            return

        mep_m = [float(mep_center[i]) / 1000.0 for i in range(3)]
        face_v = [float(face[i]) for i in range(3)]
        p_m = [float(penetration_center[i]) / 1000.0 for i in range(3)]
        half_t_m = (float(length) / 1000.0) / 2.0

        # P0 = a point on the wall outer surface plane.
        p0 = [p_m[i] + face_v[i] * half_t_m for i in range(3)]
        # Signed distance from mep_center to the plane, along face.
        d = sum((mep_m[i] - p0[i]) * face_v[i] for i in range(3))
        # Project mep_center onto the plane.
        entry = tuple(mep_m[i] - d * face_v[i] for i in range(3))
        depth_m = float(length) / 1000.0

        payload = {
            'target_point': {
                'x': entry[0],
                'y': entry[1],
                'z': entry[2],
                'nx': -float(face[0]),
                'ny': -float(face[1]),
                'nz': -float(face[2]),
                'depth': depth_m,
                'thickness': depth_m,
                'layers': None,
                'wall_id': wall_obj.get('id'),
                'mep_id': self._selected_element.get('id'),
            },
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._target_point_pub.publish(msg)
        self.get_logger().info(
            f'Published IFC-frame target point on /task/target_point: '
            f'entry=({entry[0]:.3f}, {entry[1]:.3f}, {entry[2]:.3f}) '
            f'normal=({-face[0]:.3f}, {-face[1]:.3f}, {-face[2]:.3f}) '
            f'depth={depth_m:.3f}m.')


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
