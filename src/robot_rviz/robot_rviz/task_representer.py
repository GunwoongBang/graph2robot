import csv
import json
import struct
import numpy as np
import rclpy

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

# With UR5e's URDF and LLM, it can be more generalized
SPHERE_RADIUS = 1.00  # meters; UR5e 850mm reach + 150mm drill tip ≈ 1.0m max.
CENTER_Z_OFFSET = 0.529  # meters; height where the shoulder joint sits
CSV_FILENAME = 'cloudGlobal_cleaned_excluded.csv'

CATEGORY_RED = 1     # on-wall penetration of another MEP (high danger)
CATEGORY_ORANGE = 2  # projection shadow of a nearby MEP (medium danger)
CATEGORY_BLUE = 3    # selected MEP's own penetration (drill target zone)
CATEGORY_GREEN = 4   # within working sphere, no danger


class TaskRepresenter(Node):
    def __init__(self) -> None:
        super().__init__('task_representer')

        package_share = Path(get_package_share_directory('robot_rviz'))
        self._csv_path = package_share / 'models' / CSV_FILENAME
        self._wall_indices: dict[str, np.ndarray] = self._load_wall_index_map()

        # === Topic publishers and subscribers ===
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Publishers
        self._representation_pub = self.create_publisher(
            PointCloud2, '/task/representation', latched_qos)
        self._zones_pub = self.create_publisher(
            PointCloud2, '/task/zones', latched_qos)
        # Subscribers
        self._cloud_sub = self.create_subscription(
            PointCloud2, '/cloud', self._on_cloud, latched_qos)
        self._matrix_sub = self.create_subscription(
            String, '/matrix', self._on_matrix, latched_qos)
        self._walls_sub = self.create_subscription(
            String, '/ifc/walls', self._on_walls, latched_qos)
        self._selected_element_sub = self.create_subscription(
            String, '/task/selected_element', self._on_selected_element, latched_qos)
        self._filtered_elements_sub = self.create_subscription(
            String, '/task/filtered_elements', self._on_filtered_elements, latched_qos)
        self._target_wall_sub = self.create_subscription(
            String, '/task/target_wall', self._on_target_wall, latched_qos)
        self._target_position_sub = self.create_subscription(
            String, '/robot/target_position', self._on_target_position, latched_qos)

        self._cloud: np.ndarray | None = None
        self._frame_id: str = 'world'
        self._matrix: np.ndarray | None = None
        self._walls: list[dict] | None = None
        self._selected_element: str | None = None
        self._filtered_elements: list[dict] | None = None
        self._target_wall: str | None = None
        self._target_position: dict | None = None

    def _load_wall_index_map(self) -> dict[str, np.ndarray]:
        if not self._csv_path.exists():
            self.get_logger().error(f'CSV not found at {self._csv_path}.')
            return {}
        buckets: dict[str, list[int]] = {}
        with open(self._csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for cloud_idx, row in enumerate(reader):
                wall_id = row.get('ifc_global_id') or ''
                if not wall_id:
                    continue
                buckets.setdefault(wall_id, []).append(cloud_idx)
        out = {k: np.array(v, dtype=np.int64) for k, v in buckets.items()}
        total = sum(len(v) for v in out.values())
        self.get_logger().info(
            f'Loaded wall->indices map: {len(out)} walls, {total} points.')
        return out

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self._cloud is not None:
            return
        pts = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        self._cloud = np.array(
            [(p[0], p[1], p[2]) for p in pts], dtype=np.float32)
        self._frame_id = msg.header.frame_id
        self.get_logger().info(
            f'Received cloud with {len(self._cloud)} points on /cloud (frame_id={self._frame_id}).')
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
        self.get_logger().info('Received transform matrix on /matrix.')
        self._try_process()

    def _on_walls(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /ifc/walls JSON: {exc}')
            return
        self._walls = payload.get('walls', [])
        if self._walls is None:
            self.get_logger().warn('/ifc/walls had no "walls" field.')
            return
        self.get_logger().info(
            f'Received {len(self._walls)} walls on /ifc/walls.')
        self._try_process()

    def _on_selected_element(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /task/selected_element JSON: {exc}')
            return
        element = payload.get('selected_element') or {}
        self._selected_element = element.get('id')
        if self._selected_element is None:
            self.get_logger().warn(
                '/task/selected_element had no "selected_element.id" field.')
            return
        self.get_logger().info(
            f'Received selected element on /task/selected_element: id={self._selected_element}')
        self._try_process()

    def _on_filtered_elements(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /task/filtered_elements JSON: {exc}')
            return
        self._filtered_elements = payload.get('filtered_elements', [])
        if self._filtered_elements is None:
            self.get_logger().warn(
                '/task/filtered_elements had no "filtered_elements" field.')
            return
        self.get_logger().info(
            f'Received {len(self._filtered_elements)} filtered elements on /task/filtered_elements.')

    def _on_target_wall(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /task/target_wall JSON: {exc}')
            return
        self._target_wall = payload.get('wall_id')
        if self._target_wall is None:
            self.get_logger().warn(
                '/task/target_wall had no "wall_id" field.')
            return
        self.get_logger().info(
            f'Received target wall on /task/target_wall: {self._target_wall}')
        self._try_process()

    def _on_target_position(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /task/target_position JSON: {exc}')
            return
        self._target_position = payload.get('target_position')
        if self._target_position is None:
            self.get_logger().warn(
                '/task/target_position had no "target_position" field.')
            return
        self.get_logger().info(
            f"Target position (IFC frame): "
            f"pos=({self._target_position.get('x', 0.0):.3f}, "
            f"{self._target_position.get('y', 0.0):.3f}, "
            f"{self._target_position.get('z', 0.0):.3f}) "
            f"heading=({self._target_position.get('hx', 0.0):.3f}, "
            f"{self._target_position.get('hy', 0.0):.3f})")
        self._try_process()

    def _try_process(self) -> None:
        if (self._cloud is None or self._matrix is None
                or self._target_position is None or self._target_wall is None
                or self._selected_element is None or self._walls is None):
            return
        if len(self._cloud) == 0:
            self.get_logger().warn('Empty point cloud; skipping.')
            return
        result = self._compute_zones()
        if result is None:
            return
        kept_pts, categories, totals = result
        self._publish_representation(kept_pts, categories)
        self._publish_zones(kept_pts, categories)
        red_n, orange_n, blue_n, green_n = totals
        self.get_logger().info(
            f'Published {len(kept_pts)} task points on /task/representation '
            f'+ /task/zones (wall={self._target_wall}, '
            f'green={green_n}, orange={orange_n}, '
            f'blue={blue_n}, red={red_n}).')

    def _compute_zones(self):
        # IFC -> cloud frame for the sphere center.
        ifc_pos = np.array([
            float(self._target_position.get('x', 0.0)),
            float(self._target_position.get('y', 0.0)),
            float(self._target_position.get('z', 0.0)),
            1.0,
        ])
        center = (self._matrix @ ifc_pos)[:3].astype(np.float32)
        center[2] += CENTER_Z_OFFSET  # Adjust for shoulder joint height

        wall_idx = self._wall_indices.get(self._target_wall)
        if wall_idx is None or len(wall_idx) == 0:
            self.get_logger().warn(
                f'No points on wall {self._target_wall} in CSV map.')
            return None

        wall_pts = self._cloud[wall_idx]
        dists_sq = np.sum((wall_pts - center) ** 2, axis=1)
        in_sphere = dists_sq <= (SPHERE_RADIUS * SPHERE_RADIUS)

        target_wall_dict = next(
            (w for w in self._walls if w.get('id') == self._target_wall), None)
        if target_wall_dict is None:
            self.get_logger().warn(
                f'Wall id={self._target_wall} not found in /ifc/walls.')
            return None
        axis2 = target_wall_dict.get('axis2')
        if axis2 is None or len(axis2) != 3:
            self.get_logger().warn(
                f'Wall id={self._target_wall} has no valid axis2.')
            return None
        # Surface axes = the two indices perpendicular to axis2.
        normal_axis = int(np.argmax(np.abs(axis2)))
        surface_axes = [i for i in range(3) if i != normal_axis]

        # Convert wall cloud points and working-sphere center to IFC frame (mm).
        inv_matrix = np.linalg.inv(self._matrix)
        homog = np.hstack([
            wall_pts.astype(np.float64),
            np.ones((len(wall_pts), 1), dtype=np.float64),
        ])
        wall_pts_ifc_mm = ((inv_matrix @ homog.T).T[:, :3] * 1000.0).astype(
            np.float32)
        sphere_h = np.array(
            [center[0], center[1], center[2], 1.0], dtype=np.float64)
        sphere_ifc_mm = (inv_matrix @ sphere_h)[:3].astype(np.float64) * 1000.0
        sphere_r_mm = SPHERE_RADIUS * 1000.0

        red_mask = np.zeros_like(in_sphere)
        orange_mask = np.zeros_like(in_sphere)
        blue_mask = np.zeros_like(in_sphere)

        # 1) On-wall danger zone (red)
        if self._filtered_elements:
            for e in self._filtered_elements:
                if e.get('id') == self._selected_element:
                    continue
                wall = e.get('wall') or {}
                if wall.get('id') != self._target_wall:
                    continue
                c_mm = wall.get('center')
                if c_mm is None or len(c_mm) != 3:
                    continue
                p_radius = wall.get('radius')
                p_length = wall.get('length')
                p_sizeX = wall.get('sizeX')
                p_sizeY = wall.get('sizeY')
                p_sizeZ = wall.get('sizeZ')
                is_cyl = p_radius is not None and p_length is not None
                is_box = (p_sizeX is not None and p_sizeY is not None
                          and p_sizeZ is not None)
                if not (is_cyl or is_box):
                    continue
                center_mm = np.array(c_mm, dtype=np.float32)
                diff = wall_pts_ifc_mm - center_mm
                if is_cyl:
                    axis = np.array(axis2, dtype=np.float32)
                    along = diff @ axis
                    perp_sq = np.sum(diff * diff, axis=1) - along * along
                    inside = (np.abs(along) <= p_length / 2.0) & (
                        perp_sq <= p_radius * p_radius)
                else:
                    inside = (
                        (np.abs(diff[:, 0]) <= p_sizeX / 2.0) &
                        (np.abs(diff[:, 1]) <= p_sizeY / 2.0) &
                        (np.abs(diff[:, 2]) <= p_sizeZ / 2.0)
                    )
                red_mask |= inside & in_sphere

        # 2) Beyond-wall danger zone (orange)
        sphere_min = sphere_ifc_mm - sphere_r_mm
        sphere_max = sphere_ifc_mm + sphere_r_mm
        if self._filtered_elements:
            for e in self._filtered_elements:
                if e.get('id') == self._selected_element:
                    continue
                aabb = self._element_aabb_mm(e)
                if aabb is None:
                    continue
                e_center_mm, half_mm = aabb
                e_min = e_center_mm - half_mm
                e_max = e_center_mm + half_mm
                clip_min = np.maximum(e_min, sphere_min)
                clip_max = np.minimum(e_max, sphere_max)
                if np.any(clip_min > clip_max):
                    continue
                clip_center = (clip_min + clip_max) / 2.0
                clip_half = (clip_max - clip_min) / 2.0
                a, b = surface_axes
                shadow = (
                    (np.abs(wall_pts_ifc_mm[:, a] - clip_center[a]) <= clip_half[a]) &
                    (np.abs(wall_pts_ifc_mm[:, b] -
                     clip_center[b]) <= clip_half[b])
                )
                contrib = shadow & in_sphere & ~red_mask & ~blue_mask
                added = int(contrib.sum())
                if added > 0:
                    self.get_logger().info(
                        f'  orange from {e.get("id")} {e.get("name")} '
                        f'(shape={e.get("shapeType")}): +{added} pts')
                orange_mask |= shadow & in_sphere

        # 3) On-wall task zone (blue)
        if self._filtered_elements:
            for e in self._filtered_elements:
                if e.get('id') != self._selected_element:
                    continue
                wall = e.get('wall') or {}
                if wall.get('id') != self._target_wall:
                    break
                c_mm = wall.get('center')
                if c_mm is None or len(c_mm) != 3:
                    break
                p_radius = wall.get('radius')
                p_length = wall.get('length')
                p_sizeX = wall.get('sizeX')
                p_sizeY = wall.get('sizeY')
                p_sizeZ = wall.get('sizeZ')
                is_cyl = p_radius is not None and p_length is not None
                is_box = (p_sizeX is not None and p_sizeY is not None
                          and p_sizeZ is not None)
                if not (is_cyl or is_box):
                    break
                center_mm = np.array(c_mm, dtype=np.float32)
                diff = wall_pts_ifc_mm - center_mm
                if is_cyl:
                    # Cylinder axis = wall's axis2 (perpendicular to wall).
                    axis = np.array(axis2, dtype=np.float32)
                    along = diff @ axis
                    perp_sq = np.sum(diff * diff, axis=1) - along * along
                    inside = (np.abs(along) <= p_length / 2.0) & (
                        perp_sq <= p_radius * p_radius)
                else:
                    inside = (
                        (np.abs(diff[:, 0]) <= p_sizeX / 2.0) &
                        (np.abs(diff[:, 1]) <= p_sizeY / 2.0) &
                        (np.abs(diff[:, 2]) <= p_sizeZ / 2.0)
                    )
                blue_mask |= inside & in_sphere
                break

        # Apply priority red > blue > orange > green, then collapse into one
        # uint8 category per point.
        blue_mask &= ~red_mask
        orange_mask &= ~red_mask & ~blue_mask
        green_mask = in_sphere & ~red_mask & ~blue_mask & ~orange_mask
        keep_mask = green_mask | orange_mask | blue_mask | red_mask
        kept_pts = wall_pts[keep_mask]
        categories = np.zeros(len(kept_pts), dtype=np.uint8)
        # Assign lowest priority first so higher-priority overwrites.
        categories[green_mask[keep_mask]] = CATEGORY_GREEN
        categories[orange_mask[keep_mask]] = CATEGORY_ORANGE
        categories[blue_mask[keep_mask]] = CATEGORY_BLUE
        categories[red_mask[keep_mask]] = CATEGORY_RED
        totals = (int(red_mask.sum()), int(orange_mask.sum()),
                  int(blue_mask.sum()), int(green_mask.sum()))
        return kept_pts, categories, totals

    def _publish_representation(
            self, kept_pts: np.ndarray, categories: np.ndarray) -> None:
        """Colored PointCloud2 for RViz, derived from the category array."""
        rgb_by_cat = {
            CATEGORY_RED: self._pack_rgb(255, 0, 0),
            CATEGORY_ORANGE: self._pack_rgb(255, 140, 0),
            CATEGORY_BLUE: self._pack_rgb(0, 0, 255),
            CATEGORY_GREEN: self._pack_rgb(0, 255, 0),
        }
        default_rgb = rgb_by_cat[CATEGORY_GREEN]
        points = [
            (float(p[0]), float(p[1]), float(p[2]),
             rgb_by_cat.get(int(c), default_rgb))
            for p, c in zip(kept_pts, categories)
        ]
        fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12,
                       datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg = point_cloud2.create_cloud(
            self._make_header(), fields, points)
        self._representation_pub.publish(cloud_msg)

    def _publish_zones(
            self, kept_pts: np.ndarray, categories: np.ndarray) -> None:
        n = len(kept_pts)
        struct_dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('category', np.uint8),
            ('_pad', np.uint8, 3),
        ])
        data = np.zeros(n, dtype=struct_dtype)
        data['x'] = kept_pts[:, 0]
        data['y'] = kept_pts[:, 1]
        data['z'] = kept_pts[:, 2]
        data['category'] = categories
        msg = PointCloud2()
        msg.header = self._make_header()
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='category', offset=12,
                       datatype=PointField.UINT8, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = struct_dtype.itemsize
        msg.row_step = msg.point_step * n
        msg.data = data.tobytes()
        msg.is_dense = True
        self._zones_pub.publish(msg)

    @staticmethod
    def _element_aabb_mm(e: dict) -> tuple[np.ndarray, np.ndarray] | None:
        center = e.get('center')
        if (e.get('sizeX') is not None and e.get('sizeY') is not None
                and e.get('sizeZ') is not None):
            if center is None or len(center) != 3:
                return None
            half = np.array([
                e['sizeX'] / 2.0, e['sizeY'] / 2.0, e['sizeZ'] / 2.0,
            ], dtype=np.float32)
            return np.array(center, dtype=np.float32), half
        bmin = e.get('bbox_min')
        bmax = e.get('bbox_max')
        if (bmin is not None and bmax is not None
                and len(bmin) == 3 and len(bmax) == 3):
            bmin_a = np.array(bmin, dtype=np.float32)
            bmax_a = np.array(bmax, dtype=np.float32)
            return (bmin_a + bmax_a) / 2.0, (bmax_a - bmin_a) / 2.0
        return None

    @staticmethod
    def _aabb_intersects_sphere(
            center_mm: np.ndarray, half_mm: np.ndarray,
            sphere_center_mm: np.ndarray, sphere_r_mm: float) -> bool:
        closest = np.clip(
            sphere_center_mm, center_mm - half_mm, center_mm + half_mm)
        d2 = float(np.sum((closest - sphere_center_mm) ** 2))
        return d2 <= sphere_r_mm * sphere_r_mm

    @staticmethod
    def _pack_rgb(r: int, g: int, b: int) -> float:
        rgb_int = (r << 16) | (g << 8) | b
        return struct.unpack('f', struct.pack('I', rgb_int))[0]

    def _make_header(self):
        from std_msgs.msg import Header
        h = Header()
        h.frame_id = self._frame_id
        h.stamp = self.get_clock().now().to_msg()
        return h


def main() -> None:
    rclpy.init()
    node = TaskRepresenter()
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
