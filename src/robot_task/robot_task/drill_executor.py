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

from robot_validation.validation_log import ValidationLog

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

        self._vlog = ValidationLog()

    def _on_context(self, msg: String) -> None:
        try:
            ctx = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /drilling/context JSON: {exc}')
            return
        base_pose = self._publish_target_position(ctx)
        derivation = self._publish_target_point(ctx)
        self._log_derivation(ctx, base_pose, derivation)

    # ── Validation: Section A[0] Task derivation ──────────────────────────────

    def _log_derivation(self, ctx: dict, base_pose: dict | None,
                        derivation: tuple | None) -> None:
        """Emit the Section A[0] derivation block: derived drill points vs. the
        BIM ground-truth penetration location (Euclidean error, mm)."""
        element = ctx.get('element', {})
        mep_id = element.get('id')
        shape_type = element.get('shapeType', 'cylindrical')
        kind = 'pipe' if shape_type == 'cylindrical' else 'rectangular'
        v = self._vlog

        v.event('Task derivation & execution validation')
        v.header(f'[0] Task derivation (target {mep_id})')
        v.line(f'Target element       : {mep_id} ({kind})')

        if derivation is None:
            v.line('Drill points derived : N/A (derivation failed)')
            v.rule()
            return
        derived, truth = derivation
        v.line(f'Drill points derived : {len(derived)}')
        if base_pose is not None:
            v.line(
                f'Base pose (x,y,z,yaw): ({base_pose["x"]:.3f}, '
                f'{base_pose["y"]:.3f}, {base_pose["z"]:.3f}, '
                f'{base_pose["yaw_deg"]:.1f} deg)')
        else:
            v.line('Base pose (x,y,z,yaw): N/A')

        v.line('Per drill point (derived vs. BIM ground truth):')
        errors = []
        for i, (d, t) in enumerate(zip(derived, truth)):
            err_mm = math.dist(d, t) * 1000.0
            errors.append(err_mm)
            v.line(f'  dp {i} | derived={ValidationLog.xyz(d, nd=3)} | '
                   f'truth={ValidationLog.xyz(t, nd=3)} | '
                   f'error= {ValidationLog.mm(err_mm)} mm')
        if errors:
            mean_e = sum(errors) / len(errors)
            v.line(f'Derivation error     : mean / max  '
                   f'{ValidationLog.mm(mean_e)} / {ValidationLog.mm(max(errors))} mm')
        v.rule()

    # ── Target position ───────────────────────────────────────────────────────

    def _publish_target_position(self, ctx: dict) -> dict | None:
        penet_center = ctx.get('penetration', {}).get('center')
        facing = ctx.get('facing')
        if penet_center is None or len(penet_center) != 3 or facing is None:
            self.get_logger().warn('target_position: missing penetration.center or facing.')
            return None

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
        yaw_deg = math.degrees(math.atan2(float(facing[1]), float(facing[0])))
        return {'x': tx, 'y': ty, 'z': 0.0, 'yaw_deg': yaw_deg}

    # ── Target point(s) ───────────────────────────────────────────────────────

    def _publish_target_point(self, ctx: dict) -> tuple[list, list] | None:
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
            return None
        if penet_center is None or len(penet_center) != 3:
            self.get_logger().warn('target_point: missing penetration.center.')
            return None
        if facing is None:
            self.get_logger().warn('target_point: missing facing.')
            return None

        mep_m = [float(c) / 1000.0 for c in mep_center]
        p_m = [float(c) / 1000.0 for c in penet_center]

        # Ground-truth drill locations project the BIM penetration centre (p_m)
        # itself onto the wall surface, in place of the derived element-centre
        # projection. The derived-vs-truth gap is therefore the lateral error
        # between where we chose to drill and where the BIM says the element
        # actually penetrates the wall (the normal standoff is excluded).
        derived_xyz: list[tuple] = []
        truth_xyz: list[tuple] = []

        if shape_type == 'cylindrical':
            depth_mm = penet.get('depth_mm')
            if depth_mm is None:
                self.get_logger().warn('target_point: cylindrical missing penetration.depth_mm.')
                return None
            half_t_m = float(depth_mm) / 2000.0
            entry = self._project_onto_wall_surface(mep_m, p_m, half_t_m, facing)
            truth = self._project_onto_wall_surface(p_m, p_m, half_t_m, facing)
            points = [self._make_point(*entry, facing)]
            derived_xyz.append(entry)
            truth_xyz.append(truth)

        else:  # rectangular
            size_x = penet.get('sizeX')
            size_y = penet.get('sizeY')
            size_z = penet.get('sizeZ')
            if size_x is None or size_y is None or size_z is None:
                self.get_logger().warn('target_point: rectangular missing sizeX/Y/Z.')
                return None
            half_t_m = float(size_y) / 2000.0
            half_x_m = float(size_x) / 2000.0
            half_z_m = float(size_z) / 2000.0

            length_dir = self._normalize(self._cross([0.0, 0.0, 1.0], facing))
            center_on_wall = self._project_onto_wall_surface(mep_m, p_m, half_t_m, facing)
            truth_center = self._project_onto_wall_surface(p_m, p_m, half_t_m, facing)

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
                truth_corner = tuple(
                    truth_center[i] + dx * length_dir[i] +
                    dz * (1.0 if i == 2 else 0.0)
                    for i in range(3)
                )
                points.append(self._make_point(*corner, facing))
                derived_xyz.append(corner)
                truth_xyz.append(truth_corner)

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
        return derived_xyz, truth_xyz

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
