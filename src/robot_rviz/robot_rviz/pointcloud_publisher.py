import numpy as np
import rclpy

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField

PCD_FILENAME = 'cloudGlobal_cleaned_excluded.pcd'


class PointCloudPublisher(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_publisher')

        package_share = Path(get_package_share_directory('robot_rviz'))
        self._pcd_path = package_share / 'models' / PCD_FILENAME

        self.declare_parameter('frame_id', 'world')
        self._frame_id = self.get_parameter('frame_id').value

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Publishers
        self._cloud_pub = self.create_publisher(
            PointCloud2, '/cloud', latched_qos)

        cloud_msg = self._build_pointcloud2()
        self._cloud_pub.publish(cloud_msg)
        self.get_logger().info(
            f'Published {cloud_msg.width} points on /cloud.')

    def _read_xyz(self) -> np.ndarray:
        if not self._pcd_path.exists():
            raise FileNotFoundError(f'PCD file not found: {self._pcd_path}')

        rows: list[tuple[float, float, float]] = []
        data_section = False
        with self._pcd_path.open('r', encoding='utf-8') as handle:
            for raw in handle:
                line = raw.strip()
                if not line or line.startswith('#'):
                    continue
                if data_section:
                    parts = line.split()
                    rows.append(
                        (float(parts[0]), float(parts[1]), float(parts[2])))
                    continue
                if line.upper().startswith('DATA '):
                    if line.lower() != 'data ascii':
                        raise ValueError(
                            'Only ASCII PCD files are supported.')
                    data_section = True

        if not rows:
            raise ValueError(f'No points read from {self._pcd_path}')
        return np.asarray(rows, dtype=np.float32)

    def _build_pointcloud2(self) -> PointCloud2:
        xyz = self._read_xyz()

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height = 1
        msg.width = len(xyz)
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 12  # 3 x float32 = 12 bytes
        msg.row_step = msg.point_step * len(xyz)
        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = xyz.tobytes()
        return msg


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
