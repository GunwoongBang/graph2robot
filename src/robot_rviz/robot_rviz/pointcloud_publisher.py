import csv
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


_PACKED_POINT_DTYPE = np.dtype([
    ('x', np.float32),
    ('y', np.float32),
    ('z', np.float32),
    ('rgb', np.uint32),
])


class PointCloudPublisher(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_publisher')

        package_share = Path(get_package_share_directory('robot_rviz'))

        self.declare_parameter('frame_id', 'world')

        self._pcd_path = package_share / 'models' / 'cloudGlobal_cleaned_excluded.pcd'
        self._csv_path = package_share / 'models' / 'cloudGlobal_cleaned_excluded.csv'
        self._frame_id = self.get_parameter('frame_id').value

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

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

    def _read_ifc_names(self) -> list[str] | None:
        if not self._csv_path.exists():
            self.get_logger().warning(
                f'CSV not found: {self._csv_path}; using flat color.')
            return None
        with self._csv_path.open('r', encoding='utf-8') as handle:
            return [row['ifc_name'] for row in csv.DictReader(handle)]

    def _build_rgb(self, names: list[str] | None, point_count: int) -> np.ndarray:
        """Returns a uint32 array of packed 0xRRGGBB values, one per point."""
        if names is None or len(names) != point_count:
            if names is not None:
                self.get_logger().warning(
                    f'CSV rows ({len(names)}) != PCD points ({point_count}); using flat color.')
            v = 160
            return np.full(point_count, (v << 16) | (v << 8) | v, dtype=np.uint32)

        unique = sorted(set(names))
        if len(unique) <= 1:
            shades = {unique[0]: 160} if unique else {}
        else:
            ramp = np.linspace(60, 220, num=len(unique)).astype(int)
            shades = {name: int(ramp[i]) for i, name in enumerate(unique)}

        gray = np.fromiter((shades[n] for n in names),
                           dtype=np.uint32, count=point_count)
        self.get_logger().info(
            f'Mapped {len(unique)} ifc_name groups to grayscale shades [60, 220].')
        return (gray << 16) | (gray << 8) | gray

    def _build_pointcloud2(self) -> PointCloud2:
        xyz = self._read_xyz()
        rgb_u32 = self._build_rgb(self._read_ifc_names(), len(xyz))

        packed = np.empty(len(xyz), dtype=_PACKED_POINT_DTYPE)
        packed['x'] = xyz[:, 0]
        packed['y'] = xyz[:, 1]
        packed['z'] = xyz[:, 2]
        packed['rgb'] = rgb_u32

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height = 1
        msg.width = len(xyz)
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 16
        msg.row_step = msg.point_step * len(xyz)
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
        msg.data = packed.tobytes()
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
