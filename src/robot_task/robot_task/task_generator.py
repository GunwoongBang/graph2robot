import json
import math
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

CENTER_Z_OFFSET = 0.529  # meters; height where the shoulder joint sits


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

        penet_z_mm = float(center[2])
        # When the penetration is low, the robot needs to step back more to avoid
        # Otherwise, it can get closer to the wall, which is better for the arm reachability and stability.
        offset = 0.9 if penet_z_mm < CENTER_Z_OFFSET * 1000.0 else 0.6

        cx = center[0] / 1000.0
        cy = center[1] / 1000.0
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
            f'pos=({ifc_xy[0]:.3f}, {ifc_xy[1]:.3f}, 0.000) '
            f'heading=({hx:.3f}, {hy:.3f}) offset={offset:.1f}m '
            f'(penet_z={penet_z_mm:.0f}mm).')

    @staticmethod
    def _cross(a: list[float], b: list[float]) -> list[float]:
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]

    @staticmethod
    def _normalize(v: list[float]) -> list[float]:
        mag = math.sqrt(sum(x * x for x in v))
        return [x / mag for x in v] if mag > 1e-9 else v

    def _project_onto_wall_surface(
            self, mep_center_m: list[float], penetration_center_m: list[float],
            half_thickness_m: float, facing: list[float]) -> tuple[float, float, float]:
        p0 = [penetration_center_m[i] - facing[i]
              * half_thickness_m for i in range(3)]
        d = sum((mep_center_m[i] - p0[i]) * facing[i] for i in range(3))
        return tuple(mep_center_m[i] - d * facing[i] for i in range(3))

    def publish_target_point(self) -> None:
        ctx = self._resolve_context()
        if ctx is None:
            return
        wall_obj, facing = ctx

        mep_center = self._selected_element.get('center')
        penetration_center = wall_obj.get('center')
        shape_type = self._selected_element.get('shapeType', 'cylindrical')

        if mep_center is None or len(mep_center) != 3:
            self.get_logger().warn(
                'target_point: missing/invalid mep.center; skipping.')
            return
        if penetration_center is None or len(penetration_center) != 3:
            self.get_logger().warn(
                'target_point: missing/invalid wall.penetration_center; skipping.')
            return

        mep_m = [float(mep_center[i]) / 1000.0 for i in range(3)]
        p_m = [float(penetration_center[i]) / 1000.0 for i in range(3)]

        if shape_type == 'cylindrical':
            length = wall_obj.get('length')
            if length is None:
                self.get_logger().warn(
                    'target_point: cylindrical element missing wall.length; skipping.')
                return
            half_t_m = float(length) / 2000.0
            entry = self._project_onto_wall_surface(
                mep_m, p_m, half_t_m, facing)
            points = [self._make_point(*entry, facing)]

        else:  # rectangular
            size_x = wall_obj.get('sizeX')
            size_y = wall_obj.get('sizeY')
            size_z = wall_obj.get('sizeZ')
            if size_x is None or size_y is None or size_z is None:
                self.get_logger().warn(
                    'target_point: rectangular element missing wall.sizeX/Y/Z; skipping.')
                return
            half_t_m = float(size_y) / 2000.0
            half_x_m = float(size_x) / 2000.0
            half_z_m = float(size_z) / 2000.0

            length_dir = self._normalize(self._cross([0.0, 0.0, 1.0], facing))
            center_on_wall = self._project_onto_wall_surface(
                mep_m, p_m, half_t_m, facing)

            # Counterclockwise order when viewed from robot side.
            offsets = [
                (+ half_x_m, + half_z_m),  # top-right
                (- half_x_m, + half_z_m),  # top-left
                (- half_x_m, - half_z_m),  # bottom-left
                (+ half_x_m, - half_z_m),  # bottom-right
            ]
            points = []
            for dx, dz in offsets:
                corner = tuple(
                    center_on_wall[i] + dx * length_dir[i] +
                    dz * (1.0 if i == 2 else 0.0)
                    for i in range(3)
                )
                points.append(self._make_point(*corner, facing))

        payload = {
            'shape_type': shape_type,
            'points': points,
            'wall_id': wall_obj.get('id'),
            'mep_id': self._selected_element.get('id'),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._target_point_pub.publish(msg)
        self.get_logger().info(
            f'Published /robot/target_point: shape={shape_type}, '
            f'{len(points)} point(s).')

    @staticmethod
    def _make_point(x: float, y: float, z: float, normal: list[float]) -> dict:
        return {
            'x': x, 'y': y, 'z': z,
            'nx': float(normal[0]),
            'ny': float(normal[1]),
            'nz': float(normal[2]),
        }


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
