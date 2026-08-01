# ROBOT_TASK

`robot_task` is a package responsible for transforming BIM graph data into executable robot tasks. It bridges the Neo4j graph database (via `robot_graph`) and the robot execution layer, computing where the robot should stand, which direction it should face, and which points on the wall it should drill.

The package is structured as a pipeline of three nodes launched together:

![image](../../images/robot_task.png)

```
config/transform_matrix.yaml ──► matrix_publisher ──► /matrix

/graph/list_* ──► drill_context_builder ──► /drilling/elements  (RViz markers)
                        ▲                    /scene_graph        (RViz scene graph)
                        │                    /drilling/context
              /task/selected_element                │
                                                    ▼
                                             drill_executor ──► /robot/target_position
                                                    │            /robot/target_point
                                                    └──────────► project.log  (validation A[0])
```

`drill_context_builder` pulls the whole BIM model from `graph_server` on startup and publishes the drillable-element list and the scene graph. When a user clicks an MEP element in RViz, `task_distributor` publishes `/task/selected_element`; the builder then assembles the full drill context — target wall, layer stack, facing direction, spatial hierarchy, and nearby MEP elements — and publishes it on `/drilling/context`. `drill_executor` consumes the context and publishes the robot base position and drill-tip target point(s).

## Launch

```bash
ros2 launch robot_task drilling_task.launch.py
```

Starts `matrix_publisher` (no namespace, so `/matrix` is global) plus `drill_context_builder` and `drill_executor` in the `drilling_task` namespace. `graph_server` from `robot_graph` must already be running — the builder waits up to 10 s per service and skips any that never appear.

## Topic schema

All topics carry JSON-encoded `std_msgs/String` messages, all with TRANSIENT_LOCAL (latched) QoS so late-joining subscribers receive the last value immediately.

### `/drilling/elements`
Published once on startup after the graph is loaded. Lists every MEP element that penetrates a wall **and is hosted by a space**, and is therefore a candidate drilling task.

```json
{
  "count": 12,
  "elements": [
    {
      "id": "1iDZV_Brn0Z9GW0EFhIn_p",
      "name": "Pipe Types:Default:1208035",
      "center": [1322.0, 3283.49, 900.0],
      "shapeType": "cylindrical",
      "direction": [0.0, 1.0, 0.0],
      "radius": 16.7,
      "length": 108.0,
      "sizeX": null, "sizeY": null, "sizeZ": null,
      "bbox_min": [1305.3, 3246.0, 883.3],
      "bbox_max": [1338.7, 3354.0, 916.7],
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

`direction`/`radius`/`length` (cylindrical) and `sizeX`/`sizeY`/`sizeZ` (rectangular) are the shape parameters `task_representer` uses for its shape-aware zone tests; whichever set does not apply is `null`.

Elements that penetrate a wall but are hosted by **no** space are embedded inside the wall rather than passing through it. They are excluded here — they cannot be drill targets, so they get no interactive marker — but they still appear in `nearby_elements` as concealed behind-wall hazards.

### `/scene_graph`
Published alongside `/drilling/elements`. A render-ready flattening of the whole model for `scene_graph_builder` in `robot_rviz`; consumers do not need the raw IFC model.

```json
{
  "nodes": [
    {"id": "...", "type": "space", "name": "Room 1", "center": [1200.0, 3000.0, 1500.0]}
  ],
  "edges": [
    {"type": "bounded_by", "source": "<space_id>", "target": "<wall_id>"}
  ]
}
```

Node `type` is one of `building`, `storey`, `space`, `wall`, `mep`. Every node carries a resolved centre in the IFC frame (mm): spaces use their centroid, walls and MEP their centre, and storey/building centres are derived as the centroid of their children. Edge `type` is one of `has_storey`, `has_space`, `bounded_by`, `intersects`.

### `/drilling/context`
Published each time a new element is selected. Contains everything needed for robot positioning and motion planning.

```json
{
  "element": {
    "id": "...", "name": "...", "center": [...], "shapeType": "cylindrical",
    "direction": [...], "radius": 16.7, "length": 108.0,
    "sizeX": null, "sizeY": null, "sizeZ": null,
    "bbox_min": [...], "bbox_max": [...]
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
  "space_id": "<space_hosting_the_element>",
  "storey": {"id": "...", "name": "Level 1"},
  "building": {"id": "...", "name": "..."},
  "robot_space_id": "<space_id_where_robot_stands>",
  "nearby_elements": [
    {
      "id": "...", "name": "...", "center": [...], "shapeType": "cylindrical",
      "direction": [...], "radius": ..., "length": ...,
      "sizeX": null, "sizeY": null, "sizeZ": null,
      "bbox_min": [...], "bbox_max": [...],
      "robot_side": false
    }
  ]
}
```

`facing` is a unit vector pointing **from the robot toward the wall**. `drill_depth_mm` is the penetration's `depth_mm` for cylindrical elements and its `sizeY` for rectangular ones.

`space_id` is the space hosting the selected element; `robot_space_id` is where the robot will stand, which differs for pipes (see `drill_context_builder` below). `storey` and `building` are `{id, name}` summaries resolved from the `has_space` / `has_storey` hierarchy, or `null` when the graph does not link them.

`nearby_elements` lists MEP elements near the target wall, excluding the selected element itself, each carrying the same shape parameters as `element`. Two sources feed it: elements hosted in any space that bounds the wall, and elements embedded inside the wall itself (which no space hosts). `robot_side: true` means the element is on the robot's side of the wall (RED zone in RViz); `robot_side: false` means it is behind the wall or embedded in it (ORANGE zone) — the concealed hazards `robot_motion_planner` checks against the drill depth.

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

### matrix_publisher

Standalone publisher for the IFC-to-world transform. Loads the 4×4 matrix from `config/transform_matrix.yaml` and publishes it once on `/matrix`, latched so every node that starts later still receives it. A missing file or a non-4×4 matrix is logged and falls back to identity rather than aborting.

The matrix maps IFC coordinates (mm, right-hand Y-up) to the point-cloud world frame (m). Note that consumers scale IFC millimetres to metres *before* applying it.

**Topics (publisher)**

| Topic | Type | Description |
|---|---|---|
| `/matrix` | `std_msgs/String` | Latched; `{count: 1, matrix: [[4×4]]}` |

---

### drill_context_builder

The pipeline's graph client. Pulls the whole BIM model from `graph_server` on startup, maintains in-memory indexes of every IFC entity, and reacts to element selection events. Produces three outputs: the drillable-element list (`/drilling/elements`), the render-ready scene graph (`/scene_graph`), and a per-selection full context (`/drilling/context`).

**Graph loading** — on startup it calls all six `/graph/list_*` services (each a `std_srvs/Trigger` returning JSON in `message`), waiting up to 10 s per service and skipping any that never appear.

**Index building** — runs as each response lands, and completes once walls, spaces, layers, and MEP elements have all arrived (buildings and storeys are optional; without them the hierarchy fields come back `null`). It builds the id lookups plus four relationship indexes: `mep_id → penetrated_by` (carrying the wall id and penetration geometry), `mep_id → space_id` from `intersects`, `space_id → storey_id` from `has_space`, and `storey_id → building_id` from `has_storey`. It then publishes `/drilling/elements` and `/scene_graph`.

**Context building** — on each `/task/selected_element` message the node:
1. Looks up the MEP element, its wall penetration relationship, and its hosting space. A missing hosting space aborts the build — which is why wall-embedded elements are never drill targets.
2. Determines `facing` (unit vector from robot toward wall) using the wall's `axis2`, `directionSense`, and the hosting space's `bounded_by` side attribute.
3. For **cylindrical** elements (pipes): flips the facing so the robot approaches from the habitable side (opposite the MEP service space), and sets `robot_space_id` to the *other* space bounding the wall. Rectangular fixtures are approached from their own hosting space.
4. Collects `nearby_elements` from all spaces that bound the target wall, tagging each with `robot_side`, then appends the elements embedded inside the wall as `robot_side: false`.
5. Orders wall layers from the robot's approach direction, sums their thicknesses, and resolves the storey/building the element sits in.

The ordered layer stack is also printed to the node log, robot-side first, as a readable trace of the drilling path.

**Services (client)**

| Service | Description |
|---|---|
| `/graph/list_buildings` | Buildings with `has_storey` relationships |
| `/graph/list_storeys` | Storeys with `has_space` relationships |
| `/graph/list_spaces` | Rooms and MEP corridors with wall-boundary and intersecting-MEP relationships |
| `/graph/list_walls` | Walls with layer stack and MEP penetration geometry |
| `/graph/list_layers` | Material layers with thickness and stacking order |
| `/graph/list_mep_elements` | MEP elements with center, bbox, and shape parameters |

**Topics (subscriber)**

| Topic | Description |
|---|---|
| `/task/selected_element` | `{"id": "<mep_global_id>"}` — triggers context build |

**Topics (publisher)**

| Topic | Description |
|---|---|
| `/drilling/elements` | Drillable MEP elements with shape and penetration geometry |
| `/scene_graph` | Whole-model node/edge graph for the RViz scene-graph visualiser |
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

## Validation output

On every context it consumes, `drill_executor` writes section **A[0] Task derivation** to the shared validation log via `robot_validation.ValidationLog` (stdout plus `src/robot_validation/output/project.log`, overridable with `GRAPH2ROBOT_VALIDATION_LOG`). Sections A[1]–A[3] of the same block come from `robot_motion_planner`.

The block records the target element, the number of drill points derived, and the computed base pose (x, y, z, yaw), then compares each derived point against a BIM ground truth:

- **derived** — the point actually published on `/robot/target_point`, obtained by projecting the *element centre* onto the wall surface.
- **truth** — the same projection applied to the *penetration centre* the BIM reports.

The per-point Euclidean distance between them, plus the mean and max, is the derivation error. Because both are projected onto the same surface plane, the error measures lateral placement only — where we chose to drill versus where the BIM says the element actually crosses the wall. The normal standoff is excluded by construction.
