import json
import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.task import Future
from std_msgs.msg import Empty, String
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import StaticTransformBroadcaster


ENTITY_NAME = 'husky_ur5e'
INITIAL_POSE = (0.0, 0.0, 0.0)
FLOOR_HUSKY_Z = -0.405


class RobotSpawner(Node):
    """Spawns the husky+UR5e once at startup, then teleports it on each
    /robot/target_position event using /gazebo/set_entity_state. The controller
    manager (gazebo_ros2_control) lives inside the spawned model, so teleporting
    (instead of delete+respawn) keeps controllers + MoveIt connected across
    multiple drilling targets."""

    def __init__(self) -> None:
        super().__init__('robot_spawner')

        self._urdf_xml: str = ''
        self._spawned = False
        self._matrix: np.ndarray | None = None
        self._last_pose: Pose | None = None

        # === Service servers and clients ===
        # === Clients ===
        self._spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self._set_state_cli = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')

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
        self._motion_ready_pub = self.create_publisher(
            Empty, '/robot/motion_ready', volatile_qos)
        # Subscribers
        self._matrix_sub = self.create_subscription(
            String, '/matrix', self._on_matrix, latched_qos)
        self._robot_description_sub = self.create_subscription(
            String, '/robot_description', self._on_robot_description, latched_qos)
        self._target_position_sub = self.create_subscription(
            String, '/robot/target_position', self._on_target_position, volatile_qos)

        self._tf_broadcaster = StaticTransformBroadcaster(self)

    def _on_robot_description(self, msg: String) -> None:
        if self._urdf_xml:
            return
        self._urdf_xml = msg.data
        self.get_logger().info(
            f'Received URDF on /robot_description ({len(msg.data)} chars); '
            'spawning robot at origin.')
        self._initial_spawn()

    def _initial_spawn(self) -> None:
        if self._spawned:
            return
        if not self._spawn_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'Service /spawn_entity unavailable; cannot spawn robot.')
            return
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = INITIAL_POSE
        pose.orientation.w = 1.0

        req = SpawnEntity.Request()
        req.name = ENTITY_NAME
        req.xml = self._urdf_xml
        req.initial_pose = pose
        future = self._spawn_cli.call_async(req)
        future.add_done_callback(self._after_spawn)
        self._last_pose = pose

    def _after_spawn(self, future: Future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'/spawn_entity raised: {exc}')
            return
        if result is None or not result.success:
            reason = result.status_message if result is not None else 'no result'
            self.get_logger().error(f'/spawn_entity failed: {reason}')
            return
        self._spawned = True
        self._broadcast_base_link_tf(self._last_pose)
        self.get_logger().info(
            f'Spawned {ENTITY_NAME} at origin; waiting for target_position.')

    def _on_target_position(self, msg: String) -> None:
        if not self._spawned:
            self.get_logger().warn(
                'Robot not yet spawned; ignoring /robot/target_position.')
            return
        if self._matrix is None:
            self.get_logger().warn(
                '/matrix not yet received; cannot transform drilling position.')
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /robot/target_position JSON: {exc}')
            return
        d = payload.get('target_position') or {}

        # IFC -> cloud frame: 4x4 to position, rotation block to heading.
        ifc_pos = np.array(
            [float(d.get('x', 0.0)),
             float(d.get('y', 0.0)),
             float(d.get('z', 0.0)),
             1.0])
        cloud_pos = self._matrix @ ifc_pos

        ifc_heading = np.array(
            [float(d.get('hx', 1.0)), float(d.get('hy', 0.0)), 0.0])
        cloud_heading = self._matrix[:3, :3] @ ifc_heading
        yaw = float(np.arctan2(cloud_heading[1], cloud_heading[0]))

        pose = Pose()
        pose.position.x = float(cloud_pos[0])
        pose.position.y = float(cloud_pos[1])
        pose.position.z = FLOOR_HUSKY_Z
        pose.orientation.z = float(np.sin(yaw / 2.0))
        pose.orientation.w = float(np.cos(yaw / 2.0))

        self.get_logger().info(
            f'Teleport target (cloud frame): pos=({pose.position.x:.3f}, '
            f'{pose.position.y:.3f}, {pose.position.z:.3f}) yaw={yaw:.3f} rad.')
        self._teleport(pose)

    def _teleport(self, pose: Pose) -> None:
        if not self._set_state_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                'Service /gazebo/set_entity_state unavailable; cannot teleport.')
            return
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = ENTITY_NAME
        req.state.pose = pose
        req.state.reference_frame = 'world'
        future = self._set_state_cli.call_async(req)
        future.add_done_callback(
            lambda fut, p=pose: self._after_teleport(fut, p))

    def _after_teleport(self, future: Future, pose: Pose) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'/gazebo/set_entity_state raised: {exc}')
            return
        if result is None or not result.success:
            reason = (result.status_message
                      if result is not None else 'no result')
            self.get_logger().error(
                f'/gazebo/set_entity_state failed: {reason}')
            return
        self._last_pose = pose
        self._broadcast_base_link_tf(pose)
        self.get_logger().info(f'Teleported {ENTITY_NAME} to target pose.')
        # Signal the motion planner that the base is at rest.
        self._motion_ready_pub.publish(Empty())

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
        self.get_logger().info('Received transform matrix on /matrix.')

    def _broadcast_base_link_tf(self, pose: Pose) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = RobotSpawner()
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
