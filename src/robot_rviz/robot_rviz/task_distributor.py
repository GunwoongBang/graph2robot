import json
import numpy as np
import rclpy

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)


class TaskDistributor(Node):
    def __init__(self) -> None:
        super().__init__('task_distributor')

        self.declare_parameter('marker_diameter', 0.3)
        self._marker_diameter = float(
            self.get_parameter('marker_diameter').value)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._cloud_sub = self.create_subscription(
            PointCloud2, '/cloud', self._on_cloud, latched_qos)
        self._task_sub = self.create_subscription(
            String, '/task/mep_elements', self._on_task, latched_qos)
        self._walls_sub = self.create_subscription(
            String, '/task/walls', self._on_walls, latched_qos)
        self._matrix_sub = self.create_subscription(
            String, '/matrix', self._on_matrix, latched_qos)
        self._selected_task_pub = self.create_publisher(
            String, '/task/selected_task', latched_qos)
        self._selected_task_sub = self.create_subscription(
            String, '/task/selected_task', self._on_selected_task, latched_qos)

        self._marker_server = InteractiveMarkerServer(self, 'task')

        self._cloud: np.ndarray | None = None
        self._frame_id: str = 'world'
        self._tasks: list[dict] | None = None
        self._walls: list[dict] | None = None
        self._matrix: np.ndarray | None = None
        self._element_index: dict[str, dict] = {}
        self._marker_positions: dict[str, tuple[float, float, float]] = {}
        self._selected_id: str | None = None

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

    def _on_task(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /task JSON: {exc}')
            return
        self._tasks = payload.get('tasks', [])
        self.get_logger().info(
            f'Received {len(self._tasks)} tasks on /task.')
        self._try_process()

    def _on_walls(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid /task/walls JSON: {exc}')
            return
        self._walls = payload.get('walls', [])
        self.get_logger().info(
            f'Received {len(self._walls)} walls on /task/walls.')

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

    def _on_selected_task(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Invalid /task/selected_task JSON: {exc}')
            return
        new_id = (payload.get('task') or {}).get('id')
        if new_id == self._selected_id:
            return
        self._selected_id = new_id
        self._redraw_markers()

    def _try_process(self) -> None:
        if self._cloud is None or self._tasks is None or self._matrix is None:
            return
        if len(self._cloud) == 0:
            self.get_logger().warn('Empty point cloud; skipping.')
            return

        self._marker_server.clear()
        self._element_index.clear()
        self._marker_positions.clear()

        for element in self._tasks:
            center = element.get('center')
            element_id = element.get('id')
            if center is None or len(center) != 3 or not element_id:
                continue

            # Unit conversion from mm (ifc) to m (point cloud)
            homog = np.array(
                [center[0] / 1000.0, center[1] / 1000.0, center[2] / 1000.0, 1.0], dtype=float)
            transformed = self._matrix @ homog
            tx, ty, tz = float(transformed[0]), float(
                transformed[1]), float(transformed[2])

            diffs = self._cloud - np.array([tx, ty, tz], dtype=np.float32)
            dists_sq = np.sum(diffs * diffs, axis=1)
            closest_idx = int(np.argmin(dists_sq))
            cx, cy, cz = self._cloud[closest_idx]

            self._element_index[element_id] = element
            self._marker_positions[element_id] = (
                float(cx), float(cy), float(cz))

        self._redraw_markers()
        self.get_logger().info(
            f'Placed {len(self._marker_positions)} interactive markers on /task/update.')

    def _redraw_markers(self) -> None:
        for element_id, (x, y, z) in self._marker_positions.items():
            element = self._element_index[element_id]
            int_marker = self._build_interactive_marker(
                element, x, y, z, selected=(element_id == self._selected_id))
            self._marker_server.insert(
                int_marker, feedback_callback=self._on_marker_feedback)
        self._marker_server.applyChanges()

    def _build_interactive_marker(
            self, element: dict, x: float, y: float, z: float,
            selected: bool = False) -> InteractiveMarker:
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = self._marker_diameter
        marker.scale.y = self._marker_diameter
        marker.scale.z = self._marker_diameter
        if selected:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.markers.append(marker)

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self._frame_id
        int_marker.name = element['id']
        int_marker.description = element.get('name', '')
        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        int_marker.pose.position.z = z
        int_marker.pose.orientation.w = 1.0
        int_marker.scale = self._marker_diameter
        int_marker.controls.append(control)
        return int_marker

    def _on_marker_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type != InteractiveMarkerFeedback.BUTTON_CLICK:
            return
        element = self._element_index.get(feedback.marker_name)
        if element is None:
            self.get_logger().warn(
                f'Click on unknown marker: {feedback.marker_name}')
            return
        msg = String()
        msg.data = json.dumps({'task': element})
        self._selected_task_pub.publish(msg)
        self.get_logger().info(
            f"Selected task on /task/selected_task: id={element['id']}")


def main() -> None:
    rclpy.init()
    node = TaskDistributor()
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
