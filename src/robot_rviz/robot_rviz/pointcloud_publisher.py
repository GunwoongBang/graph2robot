from __future__ import annotations

import struct
import rclpy

from pathlib import Path
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class PointCloudPublisher(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_publisher')

        package_share = Path(get_package_share_directory('robot_gazebo'))

        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('topic', '/cloud')
        self.declare_parameter('publish_rate_hz', 1.0)

        self._pcd_path = Path(
            package_share / 'worlds' / 'cloudGlobal_cleaned_excluded.pcd')
        self._frame_id = self.get_parameter('frame_id').value
        self._topic = self.get_parameter('topic').value
        self._publish_rate_hz = float(
            self.get_parameter('publish_rate_hz').value)

        self._publisher = self.create_publisher(
            PointCloud2,
            self._topic,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self._cloud_msg = self._build_pointcloud2()
        publish_period = 1.0 / max(self._publish_rate_hz, 0.1)
        self._timer = self.create_timer(publish_period, self._publish)

        self.get_logger().info(f'Reading point cloud from: {self._pcd_path}')
        self.get_logger().info(
            f'Loaded {self._cloud_msg.width} points, publishing on {self._topic} at {self._publish_rate_hz:.2f} Hz'
        )

    def _read_ascii_pcd(self) -> tuple[list[float], list[float], list[float]]:
        if not self._pcd_path.exists():
            raise FileNotFoundError(f'PCD file not found: {self._pcd_path}')

        points_x: list[float] = []
        points_y: list[float] = []
        points_z: list[float] = []
        data_section = False

        with self._pcd_path.open('r', encoding='utf-8') as handle:
            for raw_line in handle:
                line = raw_line.strip()
                if not line or line.startswith('#'):
                    continue
                if data_section:
                    x_str, y_str, z_str = line.split()[:3]
                    points_x.append(float(x_str))
                    points_y.append(float(y_str))
                    points_z.append(float(z_str))
                    continue
                if line.upper().startswith('DATA '):
                    if line.lower() != 'data ascii':
                        raise ValueError(
                            'Only ASCII PCD files are supported by this publisher.')
                    data_section = True

        if not points_x:
            raise ValueError(f'No points were read from {self._pcd_path}')

        return points_x, points_y, points_z

    def _build_pointcloud2(self) -> PointCloud2:
        points_x, points_y, points_z = self._read_ascii_pcd()
        point_count = len(points_x)
        buffer = bytearray(point_count * 12)

        offset = 0
        for x, y, z in zip(points_x, points_y, points_z):
            struct.pack_into('<fff', buffer, offset, x, y, z)
            offset += 12

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height = 1
        msg.width = point_count
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 12
        msg.row_step = msg.point_step * point_count
        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = bytes(buffer)
        return msg

    def _publish(self) -> None:
        self._cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._cloud_msg)


def main() -> None:
    rclpy.init()
    node = PointCloudPublisher()
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
