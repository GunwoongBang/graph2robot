import json
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

# Per-type marker style: RGBA colour and sphere diameter (metres).
TYPE_STYLE = {
    'building': (ColorRGBA(r=0.75, g=0.30, b=0.90, a=0.9), 0.3),
    'storey':   (ColorRGBA(r=0.20, g=0.75, b=0.95, a=0.9), 0.3),
    'space':    (ColorRGBA(r=0.30, g=0.85, b=0.40, a=0.9), 0.3),
    'wall':     (ColorRGBA(r=0.85, g=0.85, b=0.85, a=0.9), 0.2),
    'mep':      (ColorRGBA(r=1.00, g=0.55, b=0.10, a=0.9), 0.1),
}

# Edge (relationship) styling: relationship type → colour.
EDGE_STYLE = {
    'has_storey':  ColorRGBA(r=0.75, g=0.30, b=0.90, a=0.5),
    'has_space':   ColorRGBA(r=0.20, g=0.75, b=0.95, a=0.5),
    'bounded_by':  ColorRGBA(r=0.70, g=0.70, b=0.70, a=0.4),
    'hosts':       ColorRGBA(r=1.00, g=0.55, b=0.10, a=0.4),
}
EDGE_ORDER = list(EDGE_STYLE.keys())  # stable marker ids per relationship type

LABEL_SCALE_Z = 0.15  # metres; text height


class SceneGraphBuilder(Node):
    """Task-agnostic 3D scene graph visualiser.

    Renders a consolidated scene graph (nodes + edges) that a task context
    builder publishes on /scene_graph. Node centres arrive resolved in the IFC
    frame (mm); this node only transforms them into the world frame and draws
    them as an RViz MarkerArray (spheres + labels + edges). It does not read the
    raw IFC model, so any task builder can feed the same visualiser.
    """

    def __init__(self) -> None:
        super().__init__('scene_graph_builder')

        self.declare_parameter('frame_id', 'world')
        self._frame_id = self.get_parameter('frame_id').value

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Publisher
        self._markers_pub = self.create_publisher(
            MarkerArray, '/scene_graph/markers', latched_qos)
        # Subscribers
        self.create_subscription(
            String, '/scene_graph', self._on_scene_graph, latched_qos)
        self.create_subscription(
            String, '/matrix', self._on_matrix, latched_qos)

        self._graph: dict | None = None
        self._matrix: np.ndarray | None = None

        # Latest markers, published continuously on a timer. RViz drops markers
        # when a namespace is toggled off; publishing steadily restores them as
        # soon as it's toggled back on.
        self._markers = MarkerArray()
        self.create_timer(1.0, self._publish_markers)

    # --- Subscriptions ---------------------------------------------------

    def _on_scene_graph(self, msg: String) -> None:
        data = self._parse(msg, '/scene_graph')
        if data is not None:
            self._graph = data
            self._rebuild()

    def _on_matrix(self, msg: String) -> None:
        data = self._parse(msg, '/matrix')
        if data is None:
            return
        mat = np.array(data.get('matrix'), dtype=float)
        if mat.shape != (4, 4):
            self.get_logger().error(
                f'Matrix shape {mat.shape}, expected (4,4).')
            return
        self._matrix = mat
        self._rebuild()

    # --- Rendering -------------------------------------------------------

    def _rebuild(self) -> None:
        if self._graph is None or self._matrix is None:
            return

        nodes = self._graph.get('nodes') or []
        edges = self._graph.get('edges') or []

        # Transform every node centre IFC-mm → world-metres.
        centers_w: dict[str, np.ndarray] = {}
        node_type: dict[str, str] = {}
        node_label: dict[str, str] = {}
        for n in nodes:
            nid = n.get('id')
            center = n.get('center')
            if not nid or not center or len(center) != 3:
                continue
            centers_w[nid] = self._to_world(np.array(center, dtype=float))
            node_type[nid] = n.get('type')
            node_label[nid] = n.get('name') or nid[:8]

        markers = MarkerArray()
        for idx, (nid, pos) in enumerate(centers_w.items()):
            style = TYPE_STYLE.get(node_type[nid])
            if style is None:
                continue
            color, scale = style
            markers.markers.append(
                self._sphere(f'nodes/{node_type[nid]}', idx, pos, color, scale))
            markers.markers.append(
                self._label(idx, pos, node_label[nid], scale))
        markers.markers.extend(self._edge_markers(edges, centers_w))
        self._markers = markers

        n_edges = sum(1 for m in markers.markers if m.type == Marker.LINE_LIST)
        self.get_logger().info(
            f'Rebuilt scene graph: {len(centers_w)} nodes, {n_edges} edge groups '
            f'for /scene_graph/markers.')

    def _publish_markers(self) -> None:
        """Continuously publish the latest scene graph so RViz repopulates it
        after a namespace/display toggle."""
        if not self._markers.markers:
            return
        stamp = self.get_clock().now().to_msg()
        for m in self._markers.markers:
            m.header.stamp = stamp
        self._markers_pub.publish(self._markers)

    def _to_world(self, center_mm: np.ndarray) -> np.ndarray:
        # Graph centres are IFC millimetres; the transform maps IFC metres →
        # world metres, so scale down before applying it.
        homog = np.array(
            [center_mm[0] / 1000.0, center_mm[1] / 1000.0,
             center_mm[2] / 1000.0, 1.0])
        return (self._matrix @ homog)[:3]

    # --- Marker builders -------------------------------------------------

    def _sphere(self, ns: str, marker_id: int, pos: np.ndarray,
                color: ColorRGBA, scale: float) -> Marker:
        m = Marker()
        m.header.frame_id = self._frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = Point(
            x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        m.pose.orientation.w = 1.0
        m.scale = Vector3(x=scale, y=scale, z=scale)
        m.color = color
        return m

    def _label(self, marker_id: int, pos: np.ndarray, text: str,
               scale: float) -> Marker:
        m = Marker()
        m.header.frame_id = self._frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'labels'
        m.id = marker_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position = Point(
            x=float(pos[0]), y=float(pos[1]),
            z=float(pos[2] + scale / 2.0 + LABEL_SCALE_Z))
        m.pose.orientation.w = 1.0
        m.scale.z = LABEL_SCALE_Z
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        m.text = text
        return m

    def _edge_markers(
            self, edges: list[dict],
            centers_w: dict[str, np.ndarray]) -> list[Marker]:
        """One LINE_LIST per relationship type connecting node centres."""
        by_type: dict[str, Marker] = {}
        for e in edges:
            rel_type = e.get('type')
            color = EDGE_STYLE.get(rel_type)
            if color is None:
                continue
            src = centers_w.get(e.get('source'))
            dst = centers_w.get(e.get('target'))
            if src is None or dst is None:
                continue
            m = by_type.get(rel_type)
            if m is None:
                m = Marker()
                m.header.frame_id = self._frame_id
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = f'edges/{rel_type}'
                m.id = EDGE_ORDER.index(rel_type)
                m.type = Marker.LINE_LIST
                m.action = Marker.ADD
                m.pose.orientation.w = 1.0
                m.scale.x = 0.02  # line width (metres)
                m.color = color
                by_type[rel_type] = m
            m.points.append(
                Point(x=float(src[0]), y=float(src[1]), z=float(src[2])))
            m.points.append(
                Point(x=float(dst[0]), y=float(dst[1]), z=float(dst[2])))
        return list(by_type.values())

    def _parse(self, msg: String, topic: str) -> dict | None:
        try:
            return json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid {topic} JSON: {exc}')
            return None


def main() -> None:
    rclpy.init()
    node = SceneGraphBuilder()
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
