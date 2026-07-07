import json
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class DrillContextBuilder(Node):
    def __init__(self) -> None:
        super().__init__('drill_context_builder')

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Publishers
        self._elements_pub = self.create_publisher(
            String, '/drilling/elements', latched_qos)
        self._context_pub = self.create_publisher(
            String, '/drilling/context', latched_qos)
        # Task-agnostic scene graph (nodes + edges) for visualisers. Published
        # here so this node stays the single consumer of the raw /ifc/* model.
        self._scene_graph_pub = self.create_publisher(
            String, '/scene_graph', latched_qos)
        # Subscribers
        self.create_subscription(
            String, '/ifc/buildings', self._on_buildings, latched_qos)
        self.create_subscription(
            String, '/ifc/storeys', self._on_storeys, latched_qos)
        self.create_subscription(
            String, '/ifc/walls', self._on_walls, latched_qos)
        self.create_subscription(
            String, '/ifc/spaces', self._on_spaces, latched_qos)
        self.create_subscription(
            String, '/ifc/layers', self._on_layers, latched_qos)
        self.create_subscription(
            String, '/ifc/mep_elements', self._on_mep_elements, latched_qos)
        self.create_subscription(
            String, '/task/selected_element', self._on_selected_element, latched_qos)

        # Raw IFC entities
        self._buildings: list[dict] | None = None
        self._storeys: list[dict] | None = None
        self._walls: list[dict] | None = None
        self._spaces: list[dict] | None = None
        self._layers: list[dict] | None = None
        self._mep_elements: list[dict] | None = None

        # Reverse-lookup indexes (rebuilt whenever any /ifc/* topic updates)
        self._buildings_by_id: dict[str, dict] = {}
        self._storeys_by_id: dict[str, dict] = {}
        self._walls_by_id: dict[str, dict] = {}
        self._spaces_by_id: dict[str, dict] = {}
        self._layers_by_id: dict[str, dict] = {}
        self._mep_by_id: dict[str, dict] = {}
        # mep_id → penetrated_by relationship dict (includes wall_id + penetration geometry)
        self._penet_by_mep: dict[str, dict] = {}
        # mep_id → space_id
        self._space_id_by_mep: dict[str, str] = {}
        # space_id → storey_id (from storey HAS_SPACE relationships)
        self._storey_id_by_space: dict[str, str] = {}
        # storey_id → building_id (from building HAS_STOREY relationships)
        self._building_id_by_storey: dict[str, str] = {}

    def _on_buildings(self, msg: String) -> None:
        payload = self._parse(msg, '/ifc/buildings')
        if payload is None:
            return
        self._buildings = payload.get('buildings', [])
        self.get_logger().info(
            f'Got /ifc/buildings: {len(self._buildings)} buildings.')
        self._rebuild_indexes()

    def _on_storeys(self, msg: String) -> None:
        payload = self._parse(msg, '/ifc/storeys')
        if payload is None:
            return
        self._storeys = payload.get('storeys', [])
        self.get_logger().info(
            f'Got /ifc/storeys: {len(self._storeys)} storeys.')
        self._rebuild_indexes()

    def _on_walls(self, msg: String) -> None:
        payload = self._parse(msg, '/ifc/walls')
        if payload is None:
            return
        self._walls = payload.get('walls', [])
        self.get_logger().info(f'Got /ifc/walls: {len(self._walls)} walls.')
        self._rebuild_indexes()

    def _on_spaces(self, msg: String) -> None:
        payload = self._parse(msg, '/ifc/spaces')
        if payload is None:
            return
        self._spaces = payload.get('spaces', [])
        self.get_logger().info(f'Got /ifc/spaces: {len(self._spaces)} spaces.')
        self._rebuild_indexes()

    def _on_layers(self, msg: String) -> None:
        payload = self._parse(msg, '/ifc/layers')
        if payload is None:
            return
        self._layers = payload.get('layers', [])
        self.get_logger().info(f'Got /ifc/layers: {len(self._layers)} layers.')
        self._rebuild_indexes()

    def _on_mep_elements(self, msg: String) -> None:
        payload = self._parse(msg, '/ifc/mep_elements')
        if payload is None:
            return
        self._mep_elements = payload.get('mep_elements', [])
        self.get_logger().info(
            f'Got /ifc/mep_elements: {len(self._mep_elements)} elements.')
        self._rebuild_indexes()

    def _rebuild_indexes(self) -> None:
        if any(x is None for x in (
                self._walls, self._spaces, self._layers, self._mep_elements)):
            return

        self._buildings_by_id = {b['id']: b for b in self._buildings or []}
        self._storeys_by_id = {s['id']: s for s in self._storeys or []}
        self._spaces_by_id = {s['id']: s for s in self._spaces}
        self._walls_by_id = {w['id']: w for w in self._walls}
        self._layers_by_id = {l['id']: l for l in self._layers}
        self._mep_by_id = {m['id']: m for m in self._mep_elements}

        # mep_id → penetrated_by relationship (from wall entities)
        self._penet_by_mep = {}
        for wall in self._walls:
            for rel in (wall.get('relationship') or []):
                if rel.get('type') == 'penetrated_by':
                    mep_id = rel.get('id')
                    if mep_id:
                        self._penet_by_mep[mep_id] = {
                            **rel, 'wall_id': wall['id']}

        # mep_id → space_id (from space entities)
        self._space_id_by_mep = {}
        for space in self._spaces:
            for rel in (space.get('relationship') or []):
                if rel.get('type') == 'hosts':
                    mep_id = rel.get('id')
                    if mep_id:
                        self._space_id_by_mep[mep_id] = space['id']

        # space_id → storey_id (from storey entities)
        self._storey_id_by_space = {}
        for storey in (self._storeys or []):
            for rel in (storey.get('relationship') or []):
                if rel.get('type') == 'has_space':
                    space_id = rel.get('id')
                    if space_id:
                        self._storey_id_by_space[space_id] = storey['id']

        # storey_id → building_id (from building entities)
        self._building_id_by_storey = {}
        for building in (self._buildings or []):
            for rel in (building.get('relationship') or []):
                if rel.get('type') == 'has_storey':
                    storey_id = rel.get('id')
                    if storey_id:
                        self._building_id_by_storey[storey_id] = building['id']

        self.get_logger().info(
            f'Indexes built: {len(self._penet_by_mep)} drillable elements.')
        self._publish_elements()
        self._publish_scene_graph()

    def _publish_elements(self) -> None:
        elements = []
        for mep_id, penet_rel in self._penet_by_mep.items():
            mep = self._mep_by_id.get(mep_id)
            if mep is None:
                continue
            attrs = mep.get('attributes') or {}
            elements.append({
                'id': mep_id,
                'name': attrs.get('name'),
                'center': attrs.get('center'),
                'shapeType': attrs.get('shapeType'),
                'direction': attrs.get('direction'),
                'radius': attrs.get('radius'),
                'length': attrs.get('length'),
                'sizeX': attrs.get('sizeX'),
                'sizeY': attrs.get('sizeY'),
                'sizeZ': attrs.get('sizeZ'),
                'bbox_min': attrs.get('bbox_min'),
                'bbox_max': attrs.get('bbox_max'),
                'wall_id': penet_rel.get('wall_id'),
                'space_id': self._space_id_by_mep.get(mep_id),
                'penetration': {
                    'center': penet_rel.get('center'),
                    'depth_mm': penet_rel.get('depth_mm'),
                    'radius': penet_rel.get('radius'),
                    'sizeX': penet_rel.get('sizeX'),
                    'sizeY': penet_rel.get('sizeY'),
                    'sizeZ': penet_rel.get('sizeZ'),
                },
            })
        payload = {'count': len(elements), 'elements': elements}
        msg = String()
        msg.data = json.dumps(payload)
        self._elements_pub.publish(msg)
        self.get_logger().info(
            f'Published {len(elements)} drillable elements on /drilling/elements.')

    def _publish_scene_graph(self) -> None:
        """Publish the whole model as a render-ready node/edge graph.

        Every node carries a resolved centre in the IFC frame (mm): spaces use
        their centroid, walls/mep their centre, and storey/building centres are
        derived as the centroid of their children. Edges flatten the graph
        relationships. Visualisers consume this instead of the raw /ifc/* model.
        """
        centers: dict[str, list[float]] = {}
        nodes: list[dict] = []

        def add_node(node_id: str, ntype: str, center, name) -> None:
            if not node_id or not center or len(center) != 3:
                return
            centers[node_id] = list(center)
            nodes.append({
                'id': node_id,
                'type': ntype,
                'name': name,
                'center': list(center),
            })

        # Leaf geometry first so storey/building centroids can reference it.
        for s in self._spaces:
            attrs = s.get('attributes') or {}
            add_node(s['id'], 'space', attrs.get('centroid'), attrs.get('name'))
        for w in self._walls:
            attrs = w.get('attributes') or {}
            add_node(w['id'], 'wall', attrs.get('center'), attrs.get('name'))
        for m in self._mep_elements:
            attrs = m.get('attributes') or {}
            add_node(m['id'], 'mep', attrs.get('center'), attrs.get('name'))
        for st in (self._storeys or []):
            attrs = st.get('attributes') or {}
            add_node(st['id'], 'storey',
                     self._children_centroid(st, 'has_space', centers),
                     attrs.get('name'))
        for b in (self._buildings or []):
            attrs = b.get('attributes') or {}
            add_node(b['id'], 'building',
                     self._children_centroid(b, 'has_storey', centers),
                     attrs.get('name'))

        # Flatten the hierarchy + adjacency relationships into edges.
        edges: list[dict] = []
        for b in (self._buildings or []):
            for rel in (b.get('relationship') or []):
                if rel.get('type') == 'has_storey' and rel.get('id'):
                    edges.append({'type': 'has_storey',
                                  'source': b['id'], 'target': rel['id']})
        for st in (self._storeys or []):
            for rel in (st.get('relationship') or []):
                if rel.get('type') == 'has_space' and rel.get('id'):
                    edges.append({'type': 'has_space',
                                  'source': st['id'], 'target': rel['id']})
        for s in self._spaces:
            for rel in (s.get('relationship') or []):
                rtype = rel.get('type')
                if rtype in ('bounded_by', 'hosts') and rel.get('id'):
                    edges.append({'type': rtype,
                                  'source': s['id'], 'target': rel['id']})

        payload = {'nodes': nodes, 'edges': edges}
        msg = String()
        msg.data = json.dumps(payload)
        self._scene_graph_pub.publish(msg)
        self.get_logger().info(
            f'Published scene graph: {len(nodes)} nodes, {len(edges)} edges '
            f'on /scene_graph.')

    def _children_centroid(
            self, node: dict, rel_type: str,
            centers: dict[str, list[float]]) -> list[float] | None:
        """Mean centre of a node's children reachable via rel_type."""
        pts = [
            centers[rel['id']]
            for rel in (node.get('relationship') or [])
            if rel.get('type') == rel_type and rel.get('id') in centers
        ]
        if not pts:
            return None
        n = len(pts)
        return [sum(p[i] for p in pts) / n for i in range(3)]

    def _on_selected_element(self, msg: String) -> None:
        payload = self._parse(msg, '/task/selected_element')
        if payload is None:
            return
        mep_id = payload.get('id')
        if not mep_id:
            self.get_logger().warn('/task/selected_element has no "id" field.')
            return
        self.get_logger().info(f'Building drill context for MEP {mep_id}.')
        context = self._build_context(mep_id)
        if context is None:
            return
        msg_out = String()
        msg_out.data = json.dumps(context)
        self._context_pub.publish(msg_out)
        nearby = context['nearby_elements']
        n_robot = sum(1 for e in nearby if e['robot_side'])
        n_far = len(nearby) - n_robot
        storey = context['storey']
        building = context['building']
        self.get_logger().info(
            f'Published /drilling/context for MEP {mep_id}: '
            f'building={building["id"] if building else None}, '
            f'storey={storey["id"] if storey else None}, '
            f'wall={context["wall"]["id"]}, '
            f'{len(context["layers"])} layers, '
            f'wall_thickness={context["wall_thickness_mm"]:.1f} mm, '
            f'drill_depth={context["drill_depth_mm"]} mm, '
            f'nearby={len(nearby)} ({n_robot} robot-side, {n_far} far-side).')

    def _build_context(self, mep_id: str) -> dict | None:
        mep = self._mep_by_id.get(mep_id)
        if mep is None:
            self.get_logger().warn(f'MEP {mep_id} not found in index.')
            return None

        penet_rel = self._penet_by_mep.get(mep_id)
        if penet_rel is None:
            self.get_logger().warn(f'No wall penetration for MEP {mep_id}.')
            return None

        wall_id = penet_rel['wall_id']
        wall = self._walls_by_id.get(wall_id)
        if wall is None:
            self.get_logger().warn(f'Wall {wall_id} not in index.')
            return None

        space_id = self._space_id_by_mep.get(mep_id)
        if space_id is None:
            self.get_logger().warn(f'No space for MEP {mep_id}.')
            return None

        wall_attrs = wall.get('attributes') or {}

        facing = self._compute_facing(wall, space_id)
        if facing is None:
            return None
        axis_idx, facing_sign, facing_vec = facing

        mep_attrs = mep.get('attributes') or {}
        shape_type = mep_attrs.get('shapeType', 'cylindrical')

        # Pipes are hosted in the service space; drill from the habitable side.
        # Fixtures are hosted in the room they serve; approach from that same side.
        if shape_type == 'cylindrical':
            facing_sign = -facing_sign
            facing_vec = [-v for v in facing_vec]

        axis2 = wall_attrs.get('axis2', [0.0, 0.0, 0.0])
        flip = facing_sign * float(axis2[axis_idx]) > 0
        ordered_layers = self._ordered_layers(wall, flip)
        wall_thickness_mm = sum(
            l.get('thickness_mm') or 0.0 for l in ordered_layers)
        drill_depth_mm = (
            penet_rel.get('depth_mm') if shape_type == 'cylindrical' else penet_rel.get('sizeY'))

        # For cylindrical elements the robot approaches from the opposite side,
        # so the robot's space is the one adjacent to the wall that is NOT the
        # MEP's hosting space.
        if shape_type == 'cylindrical':
            robot_space_id = self._find_adjacent_space_id(wall_id, space_id)
        else:
            robot_space_id = space_id

        nearby_elements = self._get_nearby_elements(
            wall_id, robot_space_id, mep_id)

        # Spatial hierarchy: space → storey → building (from HAS_SPACE / HAS_STOREY).
        storey_id = self._storey_id_by_space.get(space_id)
        building_id = (
            self._building_id_by_storey.get(storey_id) if storey_id else None)

        return {
            'element': {
                'id': mep_id,
                'name': mep_attrs.get('name'),
                'center': mep_attrs.get('center'),
                'shapeType': shape_type,
                'direction': mep_attrs.get('direction'),
                'radius': mep_attrs.get('radius'),
                'length': mep_attrs.get('length'),
                'sizeX': mep_attrs.get('sizeX'),
                'sizeY': mep_attrs.get('sizeY'),
                'sizeZ': mep_attrs.get('sizeZ'),
                'bbox_min': mep_attrs.get('bbox_min'),
                'bbox_max': mep_attrs.get('bbox_max'),
            },
            'facing': facing_vec,
            'penetration': {
                'center': penet_rel.get('center'),
                'depth_mm': penet_rel.get('depth_mm'),
                'radius': penet_rel.get('radius'),
                'sizeX': penet_rel.get('sizeX'),
                'sizeY': penet_rel.get('sizeY'),
                'sizeZ': penet_rel.get('sizeZ'),
            },
            'wall': {
                'id': wall_id,
                'axis2': wall_attrs.get('axis2'),
                'center': wall_attrs.get('center'),
                'bbox_min': wall_attrs.get('bbox_min'),
                'bbox_max': wall_attrs.get('bbox_max'),
                'directionSense': wall_attrs.get('directionSense'),
            },
            'layers': ordered_layers,
            'wall_thickness_mm': wall_thickness_mm,
            'drill_depth_mm': drill_depth_mm,
            'space_id': space_id,
            'storey': self._node_info(self._storeys_by_id, storey_id),
            'building': self._node_info(self._buildings_by_id, building_id),
            'robot_space_id': robot_space_id,
            'nearby_elements': nearby_elements,
        }

    def _node_info(self, index: dict[str, dict], node_id: str | None) -> dict | None:
        """Build an {id, name} summary for a building/storey, or None if unknown."""
        if node_id is None:
            return None
        node = index.get(node_id)
        name = (node.get('attributes') or {}).get('name') if node else None
        return {'id': node_id, 'name': name}

    def _compute_facing(self, wall: dict, space_id: str) -> tuple[int, float, list[float]] | None:
        attrs = wall.get('attributes') or {}
        axis2 = attrs.get('axis2')
        direction_sense = attrs.get('directionSense', '')
        if not axis2 or len(axis2) != 3:
            self.get_logger().warn(
                f"Wall {wall.get('id')} has no valid axis2.")
            return None
        side = self._get_bounded_by_side(space_id, wall['id'])
        if side is None:
            return None
        same = str(side).upper() == str(direction_sense).upper()
        axis_idx = max(range(3), key=lambda i: abs(float(axis2[i])))
        raw_sign = 1.0 if float(axis2[axis_idx]) > 0.0 else -1.0
        facing_sign = raw_sign if same else -raw_sign
        facing_vec = [float(a)
                      for a in axis2] if same else [-float(a) for a in axis2]
        return axis_idx, facing_sign, facing_vec

    def _get_bounded_by_side(self, space_id: str, wall_id: str) -> str | None:
        space = self._spaces_by_id.get(space_id)
        if space is None:
            self.get_logger().warn(f'Space {space_id} not found.')
            return None
        for rel in (space.get('relationship') or []):
            if rel.get('type') == 'bounded_by' and rel.get('id') == wall_id:
                side = rel.get('side')
                if side:
                    return side
        self.get_logger().warn(
            f'Space {space_id} has no bounded_by relationship to wall {wall_id}.')
        return None

    def _ordered_layers(self, wall: dict, flip: bool) -> list[dict]:
        layer_ids = [
            r['id'] for r in (wall.get('relationship') or [])
            if r.get('type') == 'has_layer' and r.get('id')
        ]
        layers = [self._layers_by_id[lid]
                  for lid in layer_ids if lid in self._layers_by_id]
        layers.sort(key=lambda l: (
            l.get('attributes') or {}).get('layerIndex', 0))
        if flip:
            layers = list(reversed(layers))
        return [
            {
                'id': l['id'],
                'name': (l.get('attributes') or {}).get('name'),
                'order': idx,
                'thickness_mm': (l.get('attributes') or {}).get('thickness'),
            }
            for idx, l in enumerate(layers)
        ]

    def _find_adjacent_space_id(self, wall_id: str, exclude_space_id: str) -> str | None:
        """Return the first space (other than exclude_space_id) that bounds wall_id."""
        for space_id, space in self._spaces_by_id.items():
            if space_id == exclude_space_id:
                continue
            for rel in (space.get('relationship') or []):
                if rel.get('type') == 'bounded_by' and rel.get('id') == wall_id:
                    return space_id
        return None

    def _get_nearby_elements(self, wall_id: str, robot_space_id: str | None, selected_mep_id: str) -> list[dict]:
        """All MEP elements hosted in spaces that bound this wall.

        robot_side=True  → element is on the same side of the wall as the robot (→ RED in RViz)
        robot_side=False → element is behind the wall                            (→ ORANGE in RViz)
        """
        nearby: list[dict] = []
        seen_ids: set[str] = {selected_mep_id}
        robot_side_val = self._get_bounded_by_side(
            robot_space_id, wall_id) if robot_space_id else None
        for space_id, space in self._spaces_by_id.items():
            space_side_val = None
            for r in (space.get('relationship') or []):
                if r.get('type') == 'bounded_by' and r.get('id') == wall_id:
                    space_side_val = r.get('side')
                    break
            if space_side_val is None:
                continue
            robot_side = (
                str(space_side_val).upper() == str(robot_side_val).upper()
                if robot_side_val else space_id == robot_space_id
            )
            for rel in (space.get('relationship') or []):
                if rel.get('type') != 'hosts':
                    continue
                eid = rel.get('id')
                if not eid or eid in seen_ids:
                    continue
                seen_ids.add(eid)
                mep = self._mep_by_id.get(eid)
                if mep is None:
                    continue
                attrs = mep.get('attributes') or {}
                nearby.append({
                    'id': eid,
                    'name': attrs.get('name'),
                    'center': attrs.get('center'),
                    'shapeType': attrs.get('shapeType'),
                    'direction': attrs.get('direction'),
                    'radius': attrs.get('radius'),
                    'length': attrs.get('length'),
                    'sizeX': attrs.get('sizeX'),
                    'sizeY': attrs.get('sizeY'),
                    'sizeZ': attrs.get('sizeZ'),
                    'bbox_min': attrs.get('bbox_min'),
                    'bbox_max': attrs.get('bbox_max'),
                    'robot_side': robot_side,
                })
        self.get_logger().info(
            f'Nearby elements for wall {wall_id}: {len(nearby)} '
            f'({sum(1 for e in nearby if e["robot_side"])} robot-side, '
            f'{sum(1 for e in nearby if not e["robot_side"])} far-side).')
        return nearby

    def _parse(self, msg: String, topic: str) -> dict | None:
        try:
            return json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid {topic} JSON: {exc}')
            return None


def main() -> None:
    rclpy.init()
    node = DrillContextBuilder()
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
