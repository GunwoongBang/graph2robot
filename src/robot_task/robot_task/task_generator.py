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
            String, '/robot/target_point', latched_qos)
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

    def _compute_facing(self, wall: dict, space_id: str | None) -> list[float] | None:
        axis2 = wall.get('axis2')
        direction_sense = wall.get('directionSense')
        spaces = wall.get('space') or []
        if axis2 is None or len(axis2) != 3:
            self.get_logger().warn(
                f"wall id={wall.get('id')} has no valid axis2.")
            return None
        if not direction_sense:
            self.get_logger().warn(
                f"wall id={wall.get('id')} has no directionSense.")
            return None
        if not space_id:
            self.get_logger().warn(
                'selected element has no host space id; cannot resolve side.')
            return None
        space_entry = next(
            (s for s in spaces if s.get('id') == space_id), None)
        if space_entry is None:
            self.get_logger().warn(
                f"wall id={wall.get('id')} is not bounded by space "
                f"id={space_id}; cannot resolve side.")
            return None
        side = space_entry.get('side')
        if not side:
            self.get_logger().warn(
                f"BOUNDED_BY entry for wall={wall.get('id')} "
                f"space={space_id} has no side.")
            return None
        if str(side).upper() == str(direction_sense).upper():
            return [float(axis2[i]) for i in range(3)]
        return [-float(axis2[i]) for i in range(3)]

    def _resolve_context(self) -> tuple[dict, list[float]] | None:
        if self._selected_element is None:
            return None
        if self._walls is None:
            self.get_logger().warn(
                '/ifc/walls not yet received; cannot compute target.')
            return None
        wall_obj = self._selected_element.get('wall') or {}
        wall_id = wall_obj.get('id')
        if not wall_id:
            self.get_logger().warn(
                'selected element has no wall id; skipping.')
            return None
        wall = next((w for w in self._walls if w.get('id') == wall_id), None)
        if wall is None:
            self.get_logger().warn(
                f'wall id={wall_id} not found in cached /ifc/walls.')
            return None
        space_obj = self._selected_element.get('space') or {}
        space_id = space_obj.get('id')
        facing = self._compute_facing(wall, space_id)
        if facing is None:
            return None
        return wall_obj, facing

    def publish_target_position(self) -> None:
        ctx = self._resolve_context()
        if ctx is None:
            return
        wall_obj, facing = ctx

        center = wall_obj.get('center')
        if center is None or len(center) != 3:
            self.get_logger().warn(
                'target_position: missing/invalid wall.center; skipping.')
            return

        # IFC center is in millimeters; compute robot base position in IFC
        # frame (meters). Robot stands behind its facing direction so:
        #   robot_pos = wall_center - facing * offset
        cx = center[0] / 1000.0
        cy = center[1] / 1000.0
        offset = 0.6  # distance from wall to robot base
        ifc_xy = (cx - facing[0] * offset, cy - facing[1] * offset)

        # Robot heading = facing direction (it looks toward the wall).
        hx = float(facing[0])
        hy = float(facing[1])

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
            f'Published IFC-frame target position on /robot/target_position: '
            f'pos=({ifc_xy[0]:.3f}, {ifc_xy[1]:.3f}, 0.000) heading=({hx:.3f}, {hy:.3f}).')

    def publish_target_point(self) -> None:
        ctx = self._resolve_context()
        if ctx is None:
            return
        wall_obj, facing = ctx

        mep_center = self._selected_element.get('center')
        penetration_center = wall_obj.get('center')
        length = wall_obj.get('length')

        if mep_center is None or len(mep_center) != 3:
            self.get_logger().warn(
                'target_point: missing/invalid mep.center; skipping.')
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
        p_m = [float(penetration_center[i]) / 1000.0 for i in range(3)]
        half_t_m = (float(length) / 1000.0) / 2.0

        # P0 = point on the wall outer surface (the side the robot is on).
        # Outer-surface normal points OUT of wall toward robot = -facing,
        # so move from penetration midplane by -facing * half_thickness.
        p0 = [p_m[i] - facing[i] * half_t_m for i in range(3)]
        # Project mep_center onto the plane (sign of normal is irrelevant
        # for the projection itself).
        d = sum((mep_m[i] - p0[i]) * facing[i] for i in range(3))
        entry = tuple(mep_m[i] - d * facing[i] for i in range(3))
        depth_m = float(length) / 1000.0

        payload = {
            'target_point': {
                'x': entry[0],
                'y': entry[1],
                'z': entry[2],
                # Drill axis points INTO the wall = facing direction.
                'nx': float(facing[0]),
                'ny': float(facing[1]),
                'nz': float(facing[2]),
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
            f'Published IFC-frame target point on /robot/target_point: '
            f'entry=({entry[0]:.3f}, {entry[1]:.3f}, {entry[2]:.3f}) '
            f'normal=({facing[0]:.3f}, {facing[1]:.3f}, {facing[2]:.3f}) '
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
