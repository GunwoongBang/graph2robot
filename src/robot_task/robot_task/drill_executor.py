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


class DrillExecutor(Node):
    def __init__(self) -> None:
        super().__init__('drill_executor')

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
        self.create_subscription(
            String, '/drilling/context', self._on_context, latched_qos)

    def _on_context(self, msg: String) -> None:
        try:
            ctx = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /drilling/context JSON: {exc}')
            return
        self._publish_target_position(ctx)
        self._publish_target_point(ctx)

    # ── Target position ───────────────────────────────────────────────────────

    def _publish_target_position(self, ctx: dict) -> None:
        penet_center = ctx.get('penetration', {}).get('center')
        facing = ctx.get('facing')
        if penet_center is None or len(penet_center) != 3 or facing is None:
            self.get_logger().warn('target_position: missing penetration.center or facing.')
            return

        penet_z_mm = float(penet_center[2])
        offset = 0.9 if penet_z_mm < CENTER_Z_OFFSET * 1000.0 else 0.6

        cx = float(penet_center[0]) / 1000.0
        cy = float(penet_center[1]) / 1000.0
        tx = cx - float(facing[0]) * offset
        ty = cy - float(facing[1]) * offset

        payload = {
            'count': 1,
            'target_position': {
                'x': tx, 'y': ty, 'z': 0.0,
                'hx': float(facing[0]),
                'hy': float(facing[1]),
            },
        }
        msg_out = String()
        msg_out.data = json.dumps(payload)
        self._target_position_pub.publish(msg_out)
        self.get_logger().info(
            f'Published /robot/target_position: '
            f'pos=({tx:.3f}, {ty:.3f}, 0.000) '
            f'heading=({facing[0]:.3f}, {facing[1]:.3f}) '
            f'offset={offset:.1f}m (penet_z={penet_z_mm:.0f}mm).')

    # ── Target point(s) ───────────────────────────────────────────────────────

    def _publish_target_point(self, ctx: dict) -> None:
        element = ctx.get('element', {})
        penet = ctx.get('penetration', {})
        facing = ctx.get('facing')
        wall_id = ctx.get('wall', {}).get('id')
        mep_id = element.get('id')
        shape_type = element.get('shapeType', 'cylindrical')

        mep_center = element.get('center')
        penet_center = penet.get('center')

        if mep_center is None or len(mep_center) != 3:
            self.get_logger().warn('target_point: missing element.center.')
            return
        if penet_center is None or len(penet_center) != 3:
            self.get_logger().warn('target_point: missing penetration.center.')
            return
        if facing is None:
            self.get_logger().warn('target_point: missing facing.')
            return

        mep_m = [float(c) / 1000.0 for c in mep_center]
        p_m = [float(c) / 1000.0 for c in penet_center]

        if shape_type == 'cylindrical':
            depth_mm = penet.get('depth_mm')
            if depth_mm is None:
                self.get_logger().warn('target_point: cylindrical missing penetration.depth_mm.')
                return
            half_t_m = float(depth_mm) / 2000.0
            entry = self._project_onto_wall_surface(mep_m, p_m, half_t_m, facing)
            points = [self._make_point(*entry, facing)]

        else:  # rectangular
            size_x = penet.get('sizeX')
            size_y = penet.get('sizeY')
            size_z = penet.get('sizeZ')
            if size_x is None or size_y is None or size_z is None:
                self.get_logger().warn('target_point: rectangular missing sizeX/Y/Z.')
                return
            half_t_m = float(size_y) / 2000.0
            half_x_m = float(size_x) / 2000.0
            half_z_m = float(size_z) / 2000.0

            length_dir = self._normalize(self._cross([0.0, 0.0, 1.0], facing))
            center_on_wall = self._project_onto_wall_surface(mep_m, p_m, half_t_m, facing)

            # Counterclockwise when viewed from robot side
            offsets = [
                (+half_x_m, +half_z_m),
                (-half_x_m, +half_z_m),
                (-half_x_m, -half_z_m),
                (+half_x_m, -half_z_m),
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
            'wall_id': wall_id,
            'mep_id': mep_id,
        }
        msg_out = String()
        msg_out.data = json.dumps(payload)
        self._target_point_pub.publish(msg_out)
        self.get_logger().info(
            f'Published /robot/target_point: shape={shape_type}, {len(points)} point(s).')

    # ── Geometry helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _project_onto_wall_surface(
            mep_m: list[float], p_m: list[float],
            half_t_m: float, facing: list[float]) -> tuple[float, float, float]:
        p0 = [p_m[i] - facing[i] * half_t_m for i in range(3)]
        d = sum((mep_m[i] - p0[i]) * facing[i] for i in range(3))
        return tuple(mep_m[i] - d * facing[i] for i in range(3))

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
    node = DrillExecutor()
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
