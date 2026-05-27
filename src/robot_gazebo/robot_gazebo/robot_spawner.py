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
from std_msgs.msg import String
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import StaticTransformBroadcaster


ENTITY_NAME = 'husky_ur5e'
DRILLING_WAIT_SEC = 5.0


class RobotSpawner(Node):
    def __init__(self) -> None:
        super().__init__('robot_spawner')

        self._urdf_xml: str = ''

        # === Service publishers and clients ===
        # Clients
        self._spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self._delete_cli = self.create_client(DeleteEntity, '/delete_entity')

        # === Topic publishers and subscribers ===
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        # /robot/target_position is consumed only as a fresh event: ignore
        # any latched payload from a previous session.
        volatile_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Subscribers
        # robot_state_publisher publishes the xacro-processed URDF here. We
        # use the same string Gazebo's gazebo_ros2_control + MoveIt consume,
        # so the spawned model carries the ros2_control + gazebo plugin tags.
        self._robot_description_sub = self.create_subscription(
            String, '/robot_description', self._on_robot_description, latched_qos)
        self._target_position_sub = self.create_subscription(
            String, '/robot/target_position', self._on_target_position, volatile_qos)
        self._matrix_sub = self.create_subscription(
            String, '/matrix', self._on_matrix, latched_qos
        )

        self._busy = False
        self._pending_pose: Pose | None = None
        self._post_spawn_timer = None
        self._matrix: np.ndarray | None = None
        self._tf_broadcaster = StaticTransformBroadcaster(self)

    def _on_robot_description(self, msg: String) -> None:
        if not self._urdf_xml:
            self.get_logger().info(
                f'Received URDF on /robot_description ({len(msg.data)} chars).')
        self._urdf_xml = msg.data

    def _on_target_position(self, msg: String) -> None:
        if self._busy:
            self.get_logger().warn(
                'Spawn/drill cycle in progress; ignoring new /robot/target_position.')
            return
        if self._matrix is None:
            self.get_logger().warn(
                '/matrix not yet received; cannot transform drilling position.')
            return
        if not self._urdf_xml:
            self.get_logger().warn(
                '/robot_description not yet received; cannot spawn robot.')
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /task/target_position JSON: {exc}')
            return
        d = payload.get('target_position') or {}

        # IFC -> cloud frame: apply full 4x4 to position, rotation block to heading.
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
        pose.position.z = float(cloud_pos[2])
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = float(np.sin(yaw / 2.0))
        pose.orientation.w = float(np.cos(yaw / 2.0))

        self._pending_pose = pose
        self._busy = True
        self.get_logger().info(
            f'Target position (cloud frame): pos=({pose.position.x:.3f}, '
            f'{pose.position.y:.3f}, {pose.position.z:.3f}) yaw={yaw:.3f} rad.')
        self._delete_then_spawn()

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

    def _delete_then_spawn(self) -> None:
        if not self._delete_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                'Service /delete_entity unavailable; aborting cycle.')
            self._busy = False
            return
        req = DeleteEntity.Request()
        req.name = ENTITY_NAME
        future = self._delete_cli.call_async(req)
        future.add_done_callback(self._after_delete)

    def _after_delete(self, future: Future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f'/delete_entity raised: {exc} (continuing to spawn).')
        else:
            if result is None or not result.success:
                reason = result.status_message if result is not None else 'no result'
                self.get_logger().info(f'/delete_entity: {reason}')
            else:
                self.get_logger().info(f'Deleted {ENTITY_NAME} from world.')
        self._do_spawn()

    def _do_spawn(self) -> None:
        if not self._urdf_xml:
            self.get_logger().error('No URDF loaded; aborting spawn.')
            self._busy = False
            return
        if not self._spawn_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                'Service /spawn_entity unavailable; aborting spawn.')
            self._busy = False
            return
        req = SpawnEntity.Request()
        req.name = ENTITY_NAME
        req.xml = self._urdf_xml
        req.initial_pose = self._pending_pose
        future = self._spawn_cli.call_async(req)
        future.add_done_callback(self._after_spawn)

    def _after_spawn(self, future: Future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'/spawn_entity raised: {exc}')
            self._busy = False
            return
        if result is None or not result.success:
            reason = result.status_message if result is not None else 'no result'
            self.get_logger().error(f'/spawn_entity failed: {reason}')
            self._busy = False
            return
        self._broadcast_base_link_tf()
        self.get_logger().info(
            f'Spawned {ENTITY_NAME}; waiting {DRILLING_WAIT_SEC}s '
            '(placeholder for MoveIt drilling motion)...')
        self._post_spawn_timer = self.create_timer(
            DRILLING_WAIT_SEC, self._on_drilling_done)

    def _broadcast_base_link_tf(self) -> None:
        """Publish a static world -> base_link transform matching the spawned
        pose so RViz/MoveIt see the robot where Gazebo placed it."""
        if self._pending_pose is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self._pending_pose.position.x
        t.transform.translation.y = self._pending_pose.position.y
        t.transform.translation.z = self._pending_pose.position.z
        t.transform.rotation = self._pending_pose.orientation
        self._tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f'Broadcast world->base_link at pos=({t.transform.translation.x:.3f}, '
            f'{t.transform.translation.y:.3f}, {t.transform.translation.z:.3f}).')

    def _on_drilling_done(self) -> None:
        if self._post_spawn_timer is not None:
            self.destroy_timer(self._post_spawn_timer)
            self._post_spawn_timer = None
        self._busy = False
        self.get_logger().info(
            'Drilling placeholder finished; ready for next task.')


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
