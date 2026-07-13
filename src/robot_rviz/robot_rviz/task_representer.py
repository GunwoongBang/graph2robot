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

from robot_validation.validation_log import ValidationLog

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
        self.create_subscription(
            PointCloud2, '/cloud', self._on_cloud, latched_qos)
        self.create_subscription(
            String, '/matrix', self._on_matrix, latched_qos)
        self.create_subscription(
            String, '/robot/target_position', self._on_target_position, latched_qos)
        self.create_subscription(
            String, '/drilling/context', self._on_drill_context, latched_qos)
        self.create_subscription(
            String, '/drilling/elements', self._on_drill_elements, latched_qos)

        self._cloud: np.ndarray | None = None
        self._frame_id: str = 'world'
        self._matrix: np.ndarray | None = None
        self._target_position: dict | None = None
        self._drill_context: dict | None = None
        self._drill_elements: list[dict] | None = None
        self._vlog = ValidationLog()

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

    def _on_drill_context(self, msg: String) -> None:
        try:
            self._drill_context = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /drilling/context JSON: {exc}')
            return
        mep_id = (self._drill_context.get('element') or {}).get('id', '')
        wall_id = (self._drill_context.get('wall') or {}).get('id', '')
        self.get_logger().info(
            f'Got /drilling/context: mep={mep_id} wall={wall_id}.')
        self._try_process()

    def _on_drill_elements(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /drilling/elements JSON: {exc}')
            return
        self._drill_elements = payload.get('elements', [])
        self.get_logger().info(
            f'Got /drilling/elements: {len(self._drill_elements)} elements.')

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
                or self._target_position is None or self._drill_context is None):
            return
        if len(self._cloud) == 0:
            self.get_logger().warn('Empty point cloud; skipping.')
            return
        result = self._compute_zones()
        if result is None:
            return
        kept_pts, categories, totals, classification = result
        self._publish_representation(kept_pts, categories)
        self._publish_zones(kept_pts, categories)
        red_n, orange_n, blue_n, green_n = totals
        wall_id = (self._drill_context.get('wall') or {}).get('id', '')
        self.get_logger().info(
            f'Published {len(kept_pts)} task points on /task/representation '
            f'+ /task/zones (wall={wall_id}, '
            f'green={green_n}, orange={orange_n}, '
            f'blue={blue_n}, red={red_n}).')
        self._log_classification(classification)

    # ── Validation: Section B[0] Hazard zone classification ───────────────────

    def _log_classification(self, cls: dict) -> None:
        v = self._vlog
        elems = cls['elements']
        selected_id = cls['selected_id']
        v.event('Hazard-aware planning validation')
        v.header(f'[0] Hazard zone classification (target {selected_id})')
        v.line(f'Working sphere       : center={ValidationLog.xyz(cls["center"], nd=3)} '
               f'| radius= {ValidationLog.mm(cls["radius_mm"])} mm')
        v.line(f'MEP elements in scope: {len(elems)}')
        v.line('Per element (classified vs. BIM ground truth):')
        counts = {'DANGER': 0, 'LATENT': 0, 'TARGET': 0, 'OUT_OF_ZONE': 0}
        correct = 0
        for e in elems:
            classified = e['classified']
            truth = e['truth']
            match = classified == truth
            correct += 1 if match else 0
            counts[classified] = counts.get(classified, 0) + 1
            v.line(f'  elem {e["id"]} | class={classified.ljust(11)} | '
                   f'truth={truth.ljust(11)} | match={"YES" if match else "NO"}')
        v.line(f'Zone counts          : danger={counts["DANGER"]} | '
               f'latent={counts["LATENT"]} | target={counts["TARGET"]} | '
               f'out_of_zone={counts["OUT_OF_ZONE"]}')
        v.line(f'Classification acc.  : {correct} / {len(elems)} correct')
        v.rule()

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

        wall_id = (self._drill_context.get('wall') or {}).get('id')
        wall_idx = self._wall_indices.get(wall_id)
        if wall_idx is None or len(wall_idx) == 0:
            self.get_logger().warn(
                f'No points on wall {wall_id} in CSV map.')
            return None

        wall_pts = self._cloud[wall_idx]
        dists_sq = np.sum((wall_pts - center) ** 2, axis=1)
        in_sphere = dists_sq <= (SPHERE_RADIUS * SPHERE_RADIUS)

        wall_id = (self._drill_context.get('wall') or {}).get('id')
        selected_id = (self._drill_context.get('element') or {}).get('id')
        axis2 = (self._drill_context.get('wall') or {}).get('axis2')
        if axis2 is None or len(axis2) != 3:
            self.get_logger().warn(
                f'Wall {wall_id} has no valid axis2 in context.')
            return None

        # Wall surface axes: the two axes perpendicular to the wall normal.
        normal_axis = int(np.argmax(np.abs(axis2)))
        ax_a, ax_b = [i for i in range(3) if i != normal_axis]

        # Convert wall cloud points to IFC frame (mm).
        inv_matrix = np.linalg.inv(self._matrix)
        homog = np.hstack([
            wall_pts.astype(np.float64),
            np.ones((len(wall_pts), 1), dtype=np.float64),
        ])
        wall_pts_ifc_mm = ((inv_matrix @ homog.T).T[:, :3] * 1000.0).astype(
            np.float32)

        # IFC-mm position of the working-sphere centre (robot shoulder).
        sphere_h = np.array(
            [center[0], center[1], center[2], 1.0], dtype=np.float64)
        sphere_ifc_mm = (inv_matrix @ sphere_h)[:3].astype(np.float64) * 1000.0
        sphere_r_mm = SPHERE_RADIUS * 1000.0

        red_mask = np.zeros_like(in_sphere)
        orange_mask = np.zeros_like(in_sphere)
        blue_mask = np.zeros_like(in_sphere)

        def _element_in_sphere(e: dict) -> bool:
            """Return True if any part of the element's volume overlaps the sphere."""
            c = np.array(e.get('center', [0, 0, 0]), dtype=np.float64)
            shape = e.get('shapeType', 'other')
            if shape == 'cylindrical':
                direction = e.get('direction')
                radius = e.get('radius')
                length = e.get('length')
                if direction and radius is not None and length is not None:
                    d = np.array(direction, dtype=np.float64)
                    half_len = float(length) / 2.0
                    p0, p1 = c - d * half_len, c + d * half_len
                    seg = p1 - p0
                    seg_len_sq = float(np.dot(seg, seg))
                    if seg_len_sq > 1e-9:
                        t = np.clip(float(np.dot(sphere_ifc_mm - p0, seg)) / seg_len_sq, 0.0, 1.0)
                        closest = p0 + t * seg
                    else:
                        closest = p0
                    return float(np.linalg.norm(sphere_ifc_mm - closest)) <= sphere_r_mm + float(radius)
                return float(np.linalg.norm(c - sphere_ifc_mm)) <= sphere_r_mm
            elif shape == 'rectangular':
                sizes = [e.get('sizeX'), e.get('sizeY'), e.get('sizeZ')]
                if all(s is not None for s in sizes):
                    half_extents = np.array([float(s) for s in sizes]) / 2.0
                    closest = np.clip(sphere_ifc_mm, c - half_extents, c + half_extents)
                    return float(np.sum((closest - sphere_ifc_mm) ** 2)) <= sphere_r_mm ** 2
                return float(np.linalg.norm(c - sphere_ifc_mm)) <= sphere_r_mm
            else:  # 'other' — bbox
                bmin = e.get('bbox_min')
                bmax = e.get('bbox_max')
                if bmin and bmax and len(bmin) == 3 and len(bmax) == 3:
                    closest = np.clip(
                        sphere_ifc_mm,
                        np.array(bmin, dtype=np.float64),
                        np.array(bmax, dtype=np.float64),
                    )
                    return float(np.sum((closest - sphere_ifc_mm) ** 2)) <= sphere_r_mm ** 2
                return float(np.linalg.norm(c - sphere_ifc_mm)) <= sphere_r_mm

        def _element_footprint(e: dict, c_mm: list) -> np.ndarray:
            """Boolean mask of wall points within the element's projected footprint."""
            pa = wall_pts_ifc_mm[:, ax_a] - float(c_mm[ax_a])
            pb = wall_pts_ifc_mm[:, ax_b] - float(c_mm[ax_b])
            shape = e.get('shapeType', 'other')
            if shape == 'cylindrical':
                direction = e.get('direction')
                radius = e.get('radius')
                length = e.get('length')
                if direction and radius is not None and length is not None:
                    da = float(direction[ax_a])
                    db = float(direction[ax_b])
                    half_len = float(length) / 2.0
                    norm2d_sq = da * da + db * db
                    if norm2d_sq > 1e-9:
                        s = np.clip((pa * da + pb * db) / norm2d_sq, -half_len, half_len)
                        dist2 = (pa - s * da) ** 2 + (pb - s * db) ** 2
                    else:
                        dist2 = pa ** 2 + pb ** 2
                    return dist2 <= float(radius) ** 2
            elif shape == 'rectangular':
                sizes = [e.get('sizeX'), e.get('sizeY'), e.get('sizeZ')]
                if all(s is not None for s in sizes):
                    half_a = float(sizes[ax_a]) / 2.0
                    half_b = float(sizes[ax_b]) / 2.0
                    return (np.abs(pa) <= half_a) & (np.abs(pb) <= half_b)
            else:  # 'other' — bbox
                bmin = e.get('bbox_min')
                bmax = e.get('bbox_max')
                if bmin and bmax and len(bmin) == 3 and len(bmax) == 3:
                    half_a = (float(bmax[ax_a]) - float(bmin[ax_a])) / 2.0
                    half_b = (float(bmax[ax_b]) - float(bmin[ax_b])) / 2.0
                    return (np.abs(pa) <= half_a) & (np.abs(pb) <= half_b)
            return (np.abs(pa) <= 150.0) & (np.abs(pb) <= 150.0)

        # Nearby elements: all MEP elements hosted in spaces bounding the target
        # wall, tagged robot_side True/False.
        for e in self._drill_context.get('nearby_elements', []):
            c_mm = e.get('center')
            if c_mm is None or len(c_mm) != 3:
                continue
            if not _element_in_sphere(e):
                continue
            footprint = _element_footprint(e, c_mm) & in_sphere
            if e.get('robot_side'):
                red_mask |= footprint
            else:
                added = int(footprint.sum())
                if added > 0:
                    self.get_logger().info(
                        f'  orange {e.get("id")} {e.get("name")}: +{added} pts')
                orange_mask |= footprint

        # 3) Blue — selected element's penetration footprint on the wall surface.
        for e in (self._drill_elements or []):
            if e.get('id') == selected_id:
                penet = e.get('penetration') or {}
                pc = penet.get('center')
                if pc and len(pc) == 3:
                    pa = wall_pts_ifc_mm[:, ax_a] - float(pc[ax_a])
                    pb = wall_pts_ifc_mm[:, ax_b] - float(pc[ax_b])
                    shape = e.get('shapeType', 'other')
                    if shape == 'cylindrical' and penet.get('radius') is not None:
                        r = float(penet['radius'])
                        footprint = (pa ** 2 + pb ** 2) <= r ** 2
                    elif shape == 'rectangular':
                        sizes = [penet.get('sizeX'), penet.get('sizeY'), penet.get('sizeZ')]
                        if all(s is not None for s in sizes):
                            footprint = (
                                (np.abs(pa) <= float(sizes[ax_a]) / 2.0)
                                & (np.abs(pb) <= float(sizes[ax_b]) / 2.0)
                            )
                        else:
                            footprint = (np.abs(pa) <= 150.0) & (np.abs(pb) <= 150.0)
                    else:  # 'other' — bbox fallback
                        bmin = e.get('bbox_min')
                        bmax = e.get('bbox_max')
                        if bmin and bmax and len(bmin) == 3 and len(bmax) == 3:
                            half_a = (float(bmax[ax_a]) - float(bmin[ax_a])) / 2.0
                            half_b = (float(bmax[ax_b]) - float(bmin[ax_b])) / 2.0
                        else:
                            half_a = half_b = 150.0
                        footprint = (np.abs(pa) <= half_a) & (np.abs(pb) <= half_b)
                    blue_mask |= footprint & in_sphere
                break

        # ── Validation: per-element hazard classification (Section B[0]) ──────
        # Classify every in-scope element into the thesis vocabulary and compare
        # against a ground truth derived directly from BIM geometry.
        #   classified : what the point-based zone builder produced for the
        #                element (needs scan points under its footprint).
        #   truth      : selected -> TARGET; element volume outside the working
        #                sphere -> OUT_OF_ZONE; else robot-side -> DANGER,
        #                behind-wall -> LATENT.
        # A mismatch flags an element that is geometrically in the sphere but has
        # no wall-scan points under it (so the classifier cannot voxelise it).
        classification = {
            'center': [float(center[0]), float(center[1]), float(center[2])],
            'radius_mm': SPHERE_RADIUS * 1000.0,
            'selected_id': selected_id,
            'elements': [],
        }
        for e in self._drill_context.get('nearby_elements', []):
            c_mm = e.get('center')
            if c_mm is None or len(c_mm) != 3:
                continue
            in_sph = _element_in_sphere(e)
            footprint_pts = int((_element_footprint(e, c_mm) & in_sphere).sum()) \
                if in_sph else 0
            robot_side = bool(e.get('robot_side'))
            if in_sph and robot_side:
                truth = 'DANGER'
            elif in_sph and not robot_side:
                truth = 'LATENT'
            else:
                truth = 'OUT_OF_ZONE'
            if footprint_pts > 0 and robot_side:
                classified = 'DANGER'
            elif footprint_pts > 0 and not robot_side:
                classified = 'LATENT'
            else:
                classified = 'OUT_OF_ZONE'
            classification['elements'].append({
                'id': e.get('id'),
                'classified': classified,
                'truth': truth,
            })
        # The selected element is the drill TARGET; classified TARGET when its
        # own penetration footprint produced points on the wall.
        classification['elements'].append({
            'id': selected_id,
            'classified': 'TARGET' if bool(blue_mask.any()) else 'OUT_OF_ZONE',
            'truth': 'TARGET',
        })

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
        return kept_pts, categories, totals, classification

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
