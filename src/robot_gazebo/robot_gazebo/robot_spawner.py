import json
import rclpy
import numpy as np

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
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
from geometry_msgs.msg import Pose


ENTITY_NAME = 'husky_ur5e'
DRILLING_WAIT_SEC = 5.0


class RobotSpawner(Node):
    def __init__(self) -> None:
        super().__init__('robot_spawner')

        package_share = Path(get_package_share_directory('robot_gazebo'))
        self._urdf_path = package_share / 'models' / 'robots' / 'husky_ur5e.urdf'
        self._urdf_xml = self._load_urdf()

        self._spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self._delete_cli = self.create_client(DeleteEntity, '/delete_entity')

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._drilling_position_sub = self.create_subscription(
            String, '/task/drilling_position', self._on_drilling_position, latched_qos)
        self._matrix_sub = self.create_subscription(
            String, '/matrix', self._on_matrix, latched_qos
        )
        self._busy = False
        self._pending_pose: Pose | None = None
        self._post_spawn_timer = None
        self._matrix: np.ndarray | None = None

    def _load_urdf(self) -> str:
        if not self._urdf_path.exists():
            self.get_logger().error(f'URDF not found at {self._urdf_path}.')
            return ''
        with open(self._urdf_path, 'r') as f:
            return f.read()

    def _on_drilling_position(self, msg: String) -> None:
        if self._busy:
            self.get_logger().warn(
                'Spawn/drill cycle in progress; ignoring new /task/drilling_position.')
            return
        if self._matrix is None:
            self.get_logger().warn(
                '/matrix not yet received; cannot transform drilling position.')
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /task/drilling_position JSON: {exc}')
            return
        d = payload.get('drilling_position') or {}

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
            f'Drilling target (cloud frame): pos=({pose.position.x:.3f}, '
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
        self.get_logger().info(
            f'Spawned {ENTITY_NAME}; waiting {DRILLING_WAIT_SEC}s '
            '(placeholder for MoveIt drilling motion)...')
        self._post_spawn_timer = self.create_timer(
            DRILLING_WAIT_SEC, self._on_drilling_done)

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
