import json
import struct
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class TaskSubscriber(Node):
    """Highlights MEP element centers in the point cloud and labels them in RViz."""

    def __init__(self) -> None:
        super().__init__('task_subscriber')

        self.declare_parameter('text_height', 0.1)
        self.declare_parameter('text_z_offset', 0.1)
        self._text_height = float(self.get_parameter('text_height').value)
        self._text_z_offset = float(self.get_parameter('text_z_offset').value)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._cloud_sub = self.create_subscription(
            PointCloud2, '/cloud', self._on_cloud, latched_qos)
        self._task_sub = self.create_subscription(
            String, '/task', self._on_task, latched_qos)
        self._matrix_sub = self.create_subscription(
            String, '/matrix', self._on_matrix, latched_qos)

        self._highlight_pub = self.create_publisher(
            PointCloud2, '/highlighted_points', latched_qos)
        self._labels_pub = self.create_publisher(
            MarkerArray, '/element_labels', latched_qos)

        self._cloud: np.ndarray | None = None
        self._frame_id: str = 'world'
        self._elements: list[dict] | None = None
        self._matrix: np.ndarray | None = None

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self._cloud is not None:
            return
        pts = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        self._cloud = np.array(
            [(p[0], p[1], p[2]) for p in pts], dtype=np.float32)
        self._frame_id = msg.header.frame_id
        self.get_logger().info(
            f'Received cloud with {len(self._cloud)} points (frame_id={self._frame_id}).')
        self._try_process()

    def _on_task(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /task JSON: {exc}')
            return
        self._elements = payload.get('elements', [])
        self.get_logger().info(
            f'Received {len(self._elements)} elements on /task.')
        self._try_process()

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
        self.get_logger().info('Received transform matrix.')
        self._try_process()

    def _try_process(self) -> None:
        if self._cloud is None or self._elements is None or self._matrix is None:
            return
        if len(self._cloud) == 0:
            self.get_logger().warn('Empty point cloud; skipping.')
            return

        red_points: list[tuple[float, float, float]] = []
        markers: list[Marker] = []

        for idx, element in enumerate(self._elements):
            center = element.get('center')
            if center is None or len(center) != 3:
                continue

            # IFC centers from Neo4j are in millimeters; matrix expects meters.
            homog = np.array(
                [center[0] / 1000.0, center[1] / 1000.0, center[2] / 1000.0, 1.0], dtype=float)
            transformed = self._matrix @ homog
            tx, ty, tz = float(transformed[0]), float(
                transformed[1]), float(transformed[2])

            diffs = self._cloud - np.array([tx, ty, tz], dtype=np.float32)
            dists_sq = np.sum(diffs * diffs, axis=1)
            closest_idx = int(np.argmin(dists_sq))
            cx, cy, cz = self._cloud[closest_idx]
            red_points.append((float(cx), float(cy), float(cz)))

            markers.append(self._build_label_marker(
                idx, element, float(cx), float(cy), float(cz)))

        cloud_msg = self._build_red_cloud(red_points)
        self._highlight_pub.publish(cloud_msg)
        self._labels_pub.publish(MarkerArray(markers=markers))
        self.get_logger().info(
            f'Highlighted {len(red_points)} points and published {len(markers)} labels.')

    def _build_label_marker(self, idx: int, element: dict,
                            x: float, y: float, z: float) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'element_labels'
        marker.id = idx
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z + self._text_z_offset
        marker.pose.orientation.w = 1.0
        marker.scale.z = self._text_height
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"{element.get('name', '')}\nid={element.get('id', '')}"
        return marker

    def _build_red_cloud(self, points: list[tuple[float, float, float]]) -> PointCloud2:
        # Pack (255, 0, 0) into a float per RViz's legacy 'rgb' field convention.
        rgb_packed = struct.unpack('<f', struct.pack(
            '<I', (255 << 16) | (0 << 8) | 0))[0]

        buffer = bytearray(len(points) * 16)
        offset = 0
        for x, y, z in points:
            struct.pack_into('<ffff', buffer, offset, x, y, z, rgb_packed)
            offset += 16

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 16
        msg.row_step = msg.point_step * len(points)
        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = bytes(buffer)
        return msg


def main() -> None:
    rclpy.init()
    node = TaskSubscriber()
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
