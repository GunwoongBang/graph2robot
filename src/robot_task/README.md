# ROBOT_TASK

`robot_task` is a package responsible for transforming BIM graph data into executable robot tasks. It bridges the Neo4j graph database (via `robot_graph`) and the robot execution layer, computing where the robot should stand, which direction it should face, and which points on the wall it should drill.

The package is structured as a pipeline of three nodes launched together:

![image](../../images/robot_task.png)

When a user clicks an MEP element in RViz, `task_distributor` publishes `/task/selected_element`. `drill_context_builder` then assembles the full drill context — target wall, layer stack, facing direction, and nearby MEP elements — and publishes it on `/drilling/context`. `drill_executor` consumes the context and publishes the robot base position and drill-tip target point(s).

## Topic schema

All topics carry JSON-encoded `std_msgs/String` messages. `/ifc/*` topics use TRANSIENT_LOCAL (latched) QoS so late-joining subscribers receive the last value immediately.

### `/drilling/elements`
Published once on startup after the graph is loaded. Lists every MEP element that penetrates a wall and is therefore a candidate drilling task.

```json
{
  "count": 12,
  "elements": [
    {
      "id": "1iDZV_Brn0Z9GW0EFhIn_p",
      "name": "Pipe Types:Default:1208035",
      "center": [1322.0, 3283.49, 900.0],
      "bbox_min": [1305.3, 3246.0, 883.3],
      "bbox_max": [1338.7, 3354.0, 916.7],
      "shapeType": "cylindrical",
      "wall_id": "3S3VgMDFLDiPHeIJPk5HIn",
      "space_id": "<hosting_space_id>",
      "penetration": {
        "center": [1322.0, 3300.0, 900.0],
        "depth_mm": 108.0,
        "radius": 16.7,
        "sizeX": null, "sizeY": null, "sizeZ": null
      }
    }
  ]
}
```

### `/drilling/context`
Published each time a new element is selected. Contains everything needed for robot positioning and motion planning.

```json
{
  "element": {
    "id": "...", "name": "...", "center": [...],
    "bbox_min": [...], "bbox_max": [...], "shapeType": "cylindrical"
  },
  "facing": [0.0, -1.0, 0.0],
  "penetration": {
    "center": [1322.0, 3300.0, 900.0], "depth_mm": 108.0, "radius": 16.7,
    "sizeX": null, "sizeY": null, "sizeZ": null
  },
  "wall": {
    "id": "...", "axis2": [0.0, 1.0, 0.0], "directionSense": "NEGATIVE",
    "center": [...], "bbox_min": [...], "bbox_max": [...]
  },
  "layers": [
    {"id": "...", "name": "Gypsum", "order": 0, "thickness_mm": 12.5}
  ],
  "wall_thickness_mm": 90.0,
  "drill_depth_mm": 108.0,
  "robot_space_id": "<space_id_where_robot_stands>",
  "nearby_elements": [
    {
      "id": "...", "name": "...", "center": [...],
      "bbox_min": [...], "bbox_max": [...],
      "robot_side": false
    }
  ]
}
```

`facing` is a unit vector pointing **from the robot toward the wall**. `nearby_elements` lists all MEP elements hosted in spaces that bound the target wall, excluding the selected element itself. `robot_side: true` means the element is in the same space as the robot (RED zone in RViz); `robot_side: false` means it is behind the wall (ORANGE zone).

### `/robot/target_position`
Robot base position in IFC frame (meters), published by `drill_executor`.

```json
{
  "count": 1,
  "target_position": {"x": 1.322, "y": 4.200, "z": 0.0, "hx": 0.0, "hy": -1.0}
}
```

`hx`/`hy` is the heading direction (same as `facing` XY components), used to orient the robot base toward the wall.

### `/robot/target_point`
Drill-tip target point(s) in IFC frame (meters), published by `drill_executor`. Cylindrical elements produce one point; rectangular elements produce four corner points.

```json
{
  "shape_type": "cylindrical",
  "wall_id": "...",
  "mep_id": "...",
  "points": [
    {"x": 1.322, "y": 3.246, "z": 0.900, "nx": 0.0, "ny": -1.0, "nz": 0.0}
  ]
}
```

`nx`/`ny`/`nz` is the wall normal direction the drill tip must face (same as `facing`).

## Nodes

### task_manager

Pulls BIM data from `graph_server` on startup and distributes it to the rest of the pipeline. Also loads the IFC-to-world transform matrix from a YAML file and publishes it once.

On startup it calls all four `/graph/list_*` services, wraps each response in a `{count, <entity_key>: [...]}` envelope, and publishes on the corresponding `/ifc/*` topic. All `/ifc/*` and `/matrix` topics are latched so nodes that start later still receive the data.

**Services (client)**

| Service | Description |
|---|---|
| `/graph/list_spaces` | Rooms and MEP corridors with wall-boundary and hosted-MEP relationships |
| `/graph/list_walls` | Walls with layer stack and MEP penetration geometry |
| `/graph/list_layers` | Material layers with thickness and stacking order |
| `/graph/list_mep_elements` | MEP elements with center, bbox, and shape type |

**Topics (publisher)**

| Topic | Type | Description |
|---|---|---|
| `/ifc/spaces` | `std_msgs/String` | Latched; `{count, spaces: [...]}` |
| `/ifc/walls` | `std_msgs/String` | Latched; `{count, walls: [...]}` |
| `/ifc/layers` | `std_msgs/String` | Latched; `{count, layers: [...]}` |
| `/ifc/mep_elements` | `std_msgs/String` | Latched; `{count, mep_elements: [...]}` |
| `/matrix` | `std_msgs/String` | Latched; `{count: 1, matrix: [[4×4]]}` |

The transform matrix maps IFC coordinates (mm, right-hand Y-up) to the point-cloud world frame (m). It is loaded from `config/transform_matrix.yaml`.

---

### drill_context_builder

Maintains in-memory indexes of all IFC entities and reacts to element selection events. Produces two outputs: a static list of all drillable elements (`/drilling/elements`) and a per-selection full context (`/drilling/context`).

**Index building** — when all four `/ifc/*` topics have been received, the node rebuilds lookup dicts (`wall_id → wall`, `space_id → space`, `mep_id → mep`, `mep_id → penetration_rel`) and publishes `/drilling/elements`.

**Context building** — on each `/task/selected_element` message the node:
1. Looks up the MEP element and its wall penetration relationship.
2. Determines `facing` (unit vector from robot toward wall) using the wall's `axis2`, `directionSense`, and the hosting space's `bounded_by` side attribute.
3. For **cylindrical** elements (pipes): flips the facing so the robot approaches from the habitable side (opposite the MEP service space).
4. Identifies `robot_space_id` (the space where the robot will stand) and collects `nearby_elements` from all spaces that bound the target wall, tagging each with `robot_side`.
5. Orders wall layers from the robot's approach direction and sums their thicknesses.

**Topics (subscriber)**

| Topic | Description |
|---|---|
| `/ifc/walls` | Wall geometry and relationships |
| `/ifc/spaces` | Space-wall boundaries and hosted MEP elements |
| `/ifc/layers` | Material layer attributes |
| `/ifc/mep_elements` | MEP element geometry |
| `/task/selected_element` | `{"id": "<mep_global_id>"}` — triggers context build |

**Topics (publisher)**

| Topic | Description |
|---|---|
| `/drilling/elements` | All drillable MEP elements with penetration geometry |
| `/drilling/context` | Full drill context for the currently selected element |

---

### drill_executor

Consumes `/drilling/context` and computes the concrete robot pose and drill target(s).

**Robot base position** — the robot stands at `penetration_center - facing * offset` in the XY plane (Z fixed to floor). The standoff offset is 0.9 m when the penetration is below shoulder height (529 mm), 0.6 m otherwise. The heading is set to `facing` so the robot faces the wall.

**Drill target point(s)**:
- *Cylindrical*: one point projected onto the near wall face along `facing`, at the pipe's centreline height.
- *Rectangular*: four corner points of the fixture's bounding rectangle on the wall surface, in counterclockwise order when viewed from the robot side.

**Topics (subscriber)**

| Topic | Description |
|---|---|
| `/drilling/context` | Full drill context including facing and penetration geometry |

**Topics (publisher)**

| Topic | Description |
|---|---|
| `/robot/target_position` | Robot base pose in IFC frame (meters) |
| `/robot/target_point` | Drill-tip target point(s) with surface normal |
