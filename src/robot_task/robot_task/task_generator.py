import json
import yaml
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
from std_msgs.msg import String


class TaskGenerator(Node):
    def __init__(self) -> None:
        super().__init__('task_generator')

        package_share = Path(get_package_share_directory('robot_task'))
        self._matrix_yaml = package_share / 'config' / 'transform_matrix.yaml'

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
        self._transform_matrix = self._load_transform_matrix()

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

    def _load_transform_matrix(self) -> np.ndarray:
        if not self._matrix_yaml.exists():
            self.get_logger().warning(
                f'Transform file not found: {self._matrix_yaml}; using identity.')
            return np.eye(4, dtype=float)
        try:
            with open(self._matrix_yaml, 'r') as f:
                cfg = yaml.safe_load(f)
            mat = np.array(cfg.get('matrix', np.eye(4)))
            if mat.shape != (4, 4):
                raise ValueError('transform matrix must be 4x4')
            return mat.astype(float)
        except Exception as exc:
            self.get_logger().error(
                f'Failed to load transform matrix: {exc}; using identity.')
            return np.eye(4, dtype=float)

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

        # IFC center is in millimeters; the cloud / world frame is in meters.
        cx = center[0] / 1000.0
        cy = center[1] / 1000.0
        # Floor height in cloud frame is the z translation of the IFC->cloud matrix.
        floor_z = float(self._transform_matrix[2, 3])
        offset = 0.4

        if abs(axis2[0]) > 0.5:
            if face[0] > 0.5:
                x, y = cx + offset, cy
            elif face[0] < -0.5:
                x, y = cx - offset, cy
            else:
                self.get_logger().warn(
                    f'face={face} not aligned with axis2={axis2}; skipping.')
                return
        elif abs(axis2[1]) > 0.5:
            if face[1] > 0.5:
                x, y = cx, cy + offset
            elif face[1] < -0.5:
                x, y = cx, cy - offset
            else:
                self.get_logger().warn(
                    f'face={face} not aligned with axis2={axis2}; skipping.')
                return
        else:
            self.get_logger().warn(
                f'axis2={axis2} not aligned with x or y axis; skipping.')
            return

        z = floor_z
        payload = {
            'count': 1,
            'drilling_position': {'x': x, 'y': y, 'z': z},
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._drilling_position_pub.publish(msg)
        self.get_logger().info(
            f'Published drilling position on /task/drilling_position: '
            f'({x:.3f}, {y:.3f}, {z:.3f}).')


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
