import json
import math
import numpy as np
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


PLANNING_GROUP = 'ur5e_arm'
END_EFFECTOR_LINK = 'drill_tip'
PLANNING_FRAME = 'world'
VOXEL_SIZE = 0.03            # 3 cm box per RED/ORANGE point
DRILL_STANDOFF = 0.005       # 5 mm gap between drill_tip and wall surface,
MAX_PLAN_ATTEMPTS = 3
PLAN_TIME = 5.0              # seconds per plan attempt
POS_TOLERANCE = 0.005        # 5 mm position tolerance at the drill tip
ORI_TOLERANCE = 0.05         # ~3 deg per-axis tolerance

# /task/zones category codes (kept in sync with task_representer).
CATEGORY_RED = 1


class RobotMotionPlanner(Node):
    def __init__(self) -> None:
        super().__init__('robot_motion_planner')

        # === Service servers and clients ===
        # Clients
        self._move_group_cli = ActionClient(self, MoveGroup, '/move_action')
        self._scene_cli = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')

        # === Topic publishers and subscribers ===
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        volatile_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Publishers
        self._done_pub = self.create_publisher(
            Empty, '/robot/motion_done', volatile_qos)
        self._failed_pub = self.create_publisher(
            String, '/robot/motion_failed', volatile_qos)
        # Subscribers
        self.create_subscription(
            String, '/robot/target_point', self._on_target_point, latched_qos)
        self.create_subscription(
            PointCloud2, '/task/zones', self._on_zones, latched_qos)
        self.create_subscription(
            String, '/matrix', self._on_matrix, latched_qos)
        self.create_subscription(
            String, '/ifc/walls', self._on_walls, latched_qos)
        self.create_subscription(
            String, '/task/target_wall', self._on_target_wall, latched_qos)
        self.create_subscription(
            Empty, '/robot/motion_ready', self._on_motion_ready, volatile_qos)

        self._target_point: dict | None = None
        self._zones_msg: PointCloud2 | None = None
        self._matrix: np.ndarray | None = None
        self._walls: list[dict] | None = None
        self._target_wall_id: str | None = None
        self._busy = False

    def _on_target_point(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /robot/target_point JSON: {exc}')
            return
        self._target_point = payload.get('target_point')
        if self._target_point is None:
            self.get_logger().warn(
                '/robot/target_point had no "target_point" field.')
            return
        self.get_logger().info(
            f"Got target_point mep_id={self._target_point.get('mep_id')} "
            f"wall_id={self._target_point.get('wall_id')}.")

    def _on_zones(self, msg: PointCloud2) -> None:
        self._zones_msg = msg
        self.get_logger().info(
            f'Got /task/zones: {msg.width} points.')

    def _on_walls(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /ifc/walls JSON: {exc}')
            return
        self._walls = payload.get('walls', [])
        self.get_logger().info(
            f'Got /ifc/walls: {len(self._walls)} walls.')

    def _on_target_wall(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /task/target_wall JSON: {exc}')
            return
        self._target_wall_id = payload.get('wall_id')
        if self._target_wall_id:
            self.get_logger().info(
                f'Got /task/target_wall: {self._target_wall_id}.')

    def _on_matrix(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /matrix JSON: {exc}')
            return
        mat = np.array(payload.get('matrix'), dtype=float)
        if mat.shape != (4, 4):
            self.get_logger().error(
                f'Matrix shape {mat.shape}, expected (4,4).')
            return
        self._matrix = mat

    def _on_motion_ready(self, _msg: Empty) -> None:
        if self._busy:
            self.get_logger().warn(
                'Already executing motion; ignoring /robot/motion_ready.')
            return
        missing = []
        if self._target_point is None:
            missing.append('target_point')
        if self._matrix is None:
            missing.append('matrix')
        if missing:
            self.get_logger().warn(
                f'Cannot start motion; still waiting for: {missing}.')
            return
        self._busy = True
        self._start_pipeline()

    def _start_pipeline(self) -> None:
        collision_objects: list[CollisionObject] = []
        co = self._build_danger_collision_object()
        if co is not None:
            collision_objects.append(co)
        wall_co = self._build_target_wall_collision_object()
        if wall_co is not None:
            collision_objects.append(wall_co)
        if not collision_objects:
            self._send_goal(attempt=1)
            return
        if not self._scene_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '/apply_planning_scene unavailable; planning anyway.')
            self._send_goal(attempt=1)
            return
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.extend(collision_objects)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        self._scene_cli.call_async(req).add_done_callback(
            self._after_scene_applied)

    def _after_scene_applied(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'/apply_planning_scene raised: {exc}')
        else:
            if result is None or not result.success:
                self.get_logger().warn(
                    '/apply_planning_scene returned failure; planning anyway.')
        self._send_goal(attempt=1)

    def _build_danger_collision_object(self) -> CollisionObject | None:
        if self._zones_msg is None:
            self.get_logger().warn(
                'No /task/zones cached; planning without danger obstacles.')
            return None
        co = CollisionObject()
        co.header.frame_id = PLANNING_FRAME
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = 'task_danger_zones'
        co.operation = CollisionObject.ADD
        n_added = 0
        for p in point_cloud2.read_points(
                self._zones_msg,
                field_names=('x', 'y', 'z', 'category'),
                skip_nans=True):
            cat = int(p[3])
            if cat != CATEGORY_RED:
                continue
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE]
            co.primitives.append(prim)
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            pose.position.z = float(p[2])
            pose.orientation.w = 1.0
            co.primitive_poses.append(pose)
            n_added += 1
        self.get_logger().info(
            f'Built danger CollisionObject with {n_added} voxels '
            f'(size={VOXEL_SIZE} m).')
        if n_added == 0:
            return None
        return co

    def _build_target_wall_collision_object(self) -> CollisionObject | None:
        """Build an axis-aligned box CollisionObject for the target wall.

        Uses the wall's center + (bbox_max - bbox_min) from /ifc/walls, then
        transforms IFC mm -> world m via the cached /matrix. The IFC bbox is
        already axis-aligned in the IFC frame; for axis-aligned BIMs this
        survives the (typically rigid) IFC->world rotation as an axis-aligned
        box; for arbitrary rotations the box would be slightly conservative.
        """
        if not self._target_wall_id:
            self.get_logger().warn(
                'No /task/target_wall yet; planning without target wall.')
            return None
        if not self._walls:
            self.get_logger().warn(
                'No /ifc/walls cached; planning without target wall.')
            return None
        wall = next(
            (w for w in self._walls if w.get('id') == self._target_wall_id),
            None)
        if wall is None:
            self.get_logger().warn(
                f'Target wall {self._target_wall_id} not in /ifc/walls cache.')
            return None
        center_mm = wall.get('center')
        bmin_mm = wall.get('bbox_min')
        bmax_mm = wall.get('bbox_max')
        if (center_mm is None or len(center_mm) != 3
                or bmin_mm is None or len(bmin_mm) != 3
                or bmax_mm is None or len(bmax_mm) != 3):
            self.get_logger().warn(
                f'Target wall {self._target_wall_id} missing center/bbox.')
            return None

        # IFC mm -> world m via the cached matrix.
        center_ifc_m = np.array([
            float(center_mm[0]) / 1000.0,
            float(center_mm[1]) / 1000.0,
            float(center_mm[2]) / 1000.0,
            1.0,
        ])
        world_pos = self._matrix @ center_ifc_m
        size_m = [
            (float(bmax_mm[i]) - float(bmin_mm[i])) / 1000.0
            for i in range(3)
        ]

        co = CollisionObject()
        co.header.frame_id = PLANNING_FRAME
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = 'target_wall'
        co.operation = CollisionObject.ADD
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = size_m
        co.primitives.append(prim)
        pose = Pose()
        pose.position.x = float(world_pos[0])
        pose.position.y = float(world_pos[1])
        pose.position.z = float(world_pos[2])
        pose.orientation.w = 1.0
        co.primitive_poses.append(pose)
        self.get_logger().info(
            f'Built target_wall CollisionObject id={self._target_wall_id} '
            f'size=({size_m[0]:.3f}, {size_m[1]:.3f}, {size_m[2]:.3f}) m '
            f'at pos=({pose.position.x:.3f}, {pose.position.y:.3f}, '
            f'{pose.position.z:.3f}).')
        return co

    def _compute_world_goal_pose(self) -> PoseStamped:
        """Transform /robot/target_point (IFC frame, meters) into a world-frame
        PoseStamped for drill_tip. The drill_tip's local z-axis is aligned
        with the published normal (axis pointing INTO the wall)."""
        tp = self._target_point or {}
        ifc_pos = np.array([
            float(tp.get('x', 0.0)),
            float(tp.get('y', 0.0)),
            float(tp.get('z', 0.0)),
            1.0,
        ])
        world_pos = self._matrix @ ifc_pos

        ifc_normal = np.array([
            float(tp.get('nx', 0.0)),
            float(tp.get('ny', 0.0)),
            float(tp.get('nz', 1.0)),
        ])
        world_normal = self._matrix[:3, :3] @ ifc_normal
        # Pull the drill tip back from the surface by DRILL_STANDOFF along
        # the wall normal (normal points INTO the wall, so subtract).
        n_norm = float(np.linalg.norm(world_normal))
        if n_norm > 1e-6:
            world_pos[:3] -= (world_normal / n_norm) * DRILL_STANDOFF
        quat = self._quat_from_z_axis(world_normal)

        ps = PoseStamped()
        ps.header.frame_id = PLANNING_FRAME
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(world_pos[0])
        ps.pose.position.y = float(world_pos[1])
        ps.pose.position.z = float(world_pos[2])
        ps.pose.orientation.x = float(quat[0])
        ps.pose.orientation.y = float(quat[1])
        ps.pose.orientation.z = float(quat[2])
        ps.pose.orientation.w = float(quat[3])
        return ps

    @staticmethod
    def _quat_from_z_axis(target_z: np.ndarray) -> list[float]:
        """Quaternion (x,y,z,w) that rotates +z to a unit vector target_z."""
        z = np.asarray(target_z, dtype=float)
        n = float(np.linalg.norm(z))
        if n < 1e-6:
            return [0.0, 0.0, 0.0, 1.0]
        z = z / n
        if z[2] > 0.9999:
            return [0.0, 0.0, 0.0, 1.0]
        if z[2] < -0.9999:
            return [1.0, 0.0, 0.0, 0.0]
        axis = np.cross([0.0, 0.0, 1.0], z)
        axis = axis / float(np.linalg.norm(axis))
        angle = math.acos(float(z[2]))
        s = math.sin(angle / 2.0)
        return [float(axis[0] * s), float(axis[1] * s),
                float(axis[2] * s), float(math.cos(angle / 2.0))]

    def _send_goal(self, attempt: int) -> None:
        if not self._move_group_cli.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                '/move_action action server unavailable.')
            self._publish_motion_failed('move_action unavailable')
            return
        goal_pose = self._compute_world_goal_pose()
        constraints = self._build_pose_constraints(goal_pose)
        req = MotionPlanRequest()
        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = PLAN_TIME
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5
        req.goal_constraints = [constraints]

        goal = MoveGroup.Goal()
        goal.request = req

        self.get_logger().info(
            f'Sending MoveGroup goal (attempt {attempt}/{MAX_PLAN_ATTEMPTS}) '
            f'pos=({goal_pose.pose.position.x:.3f}, '
            f'{goal_pose.pose.position.y:.3f}, '
            f'{goal_pose.pose.position.z:.3f}).')
        send_future = self._move_group_cli.send_goal_async(goal)
        send_future.add_done_callback(
            lambda fut, a=attempt: self._after_goal_accepted(fut, a))

    def _build_pose_constraints(self, ps: PoseStamped) -> Constraints:
        c = Constraints()
        c.name = 'drill_pose'
        pc = PositionConstraint()
        pc.header = ps.header
        pc.link_name = END_EFFECTOR_LINK
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [POS_TOLERANCE]
        bv = BoundingVolume()
        bv.primitives.append(sphere)
        bv.primitive_poses.append(ps.pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header = ps.header
        oc.link_name = END_EFFECTOR_LINK
        oc.orientation = ps.pose.orientation
        oc.absolute_x_axis_tolerance = ORI_TOLERANCE
        oc.absolute_y_axis_tolerance = ORI_TOLERANCE
        oc.absolute_z_axis_tolerance = ORI_TOLERANCE
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        return c

    def _after_goal_accepted(self, send_future, attempt: int) -> None:
        try:
            handle = send_future.result()
        except Exception as exc:
            self.get_logger().error(f'send_goal_async raised: {exc}')
            self._maybe_retry(attempt)
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn(
                f'MoveGroup goal rejected (attempt {attempt}).')
            self._maybe_retry(attempt)
            return
        self.get_logger().info(
            f'MoveGroup goal accepted (attempt {attempt}); waiting...')
        handle.get_result_async().add_done_callback(
            lambda fut, a=attempt: self._after_result(fut, a))

    def _after_result(self, result_future, attempt: int) -> None:
        try:
            wrapped = result_future.result()
        except Exception as exc:
            self.get_logger().error(f'get_result_async raised: {exc}')
            self._maybe_retry(attempt)
            return
        result = wrapped.result
        # moveit_msgs/MoveItErrorCodes.SUCCESS == 1
        if result.error_code.val == 1:
            self.get_logger().info(
                f'Plan + execute SUCCESS (attempt {attempt}).')
            self._publish_motion_done()
            self._busy = False
            return
        self.get_logger().warn(
            f'Plan/execute failed (attempt {attempt}); '
            f'error_code={result.error_code.val}.')
        self._maybe_retry(attempt)

    def _maybe_retry(self, attempt: int) -> None:
        if attempt >= MAX_PLAN_ATTEMPTS:
            self.get_logger().error(
                f'All {MAX_PLAN_ATTEMPTS} planning attempts failed.')
            self._publish_motion_failed('plan/execute failed after retries')
            self._busy = False
            return
        self._send_goal(attempt + 1)

    def _publish_motion_done(self) -> None:
        self._done_pub.publish(Empty())
        self.get_logger().info('Published /robot/motion_done.')

    def _publish_motion_failed(self, reason: str) -> None:
        msg = String()
        msg.data = json.dumps({'reason': reason})
        self._failed_pub.publish(msg)
        self.get_logger().error(f'Published /robot/motion_failed: {reason}')


def main() -> None:
    rclpy.init()
    node = RobotMotionPlanner()
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
