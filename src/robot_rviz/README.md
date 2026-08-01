# ROBOT_RVIZ

`robot_rviz` is the user-interface package. It loads the point cloud scan into RViz, overlays clickable interactive markers on every drillable MEP element, and computes a color-coded working zone on the target wall whenever an element is selected.

The package runs four nodes: a static point-cloud publisher, a task-agnostic scene-graph visualiser, an interactive-marker layer that handles selection, and a zone-computation node whose output feeds both RViz visualization and the motion planner. The launch file also starts `rviz2` with `config/config.rviz` and the MoveIt parameters its MotionPlanning panel needs.

```
/cloud ──────────────────────────────────► task_representer ──► /task/representation  (RViz color view)
                                                │                /task/zones            (MoveIt collision input)
                                                │                project.log            (validation B[0])
/drilling/elements ──► task_distributor ──► /task/selected_element
                             │
                    interactive markers (RViz)

/scene_graph ──► scene_graph_builder ──► /scene_graph/markers  (RViz MarkerArray)
```

## Launch

```bash
ros2 launch robot_rviz robot_rviz.launch.py
```

`pointcloud_publisher`, `task_distributor`, and `task_representer` run in the `robot_rviz` namespace; `scene_graph_builder` runs in the `scene_graph` namespace.

## Zone color coding

When an element is selected, `task_representer` projects a working sphere (radius 1.0 m, centred at the robot shoulder 0.529 m above the base) onto the target wall's point cloud and assigns each point a category:

| Color | Category | Meaning |
|---|---|---|
| **Red** | `CATEGORY_RED = 1` | Another MEP element in the robot's own space whose footprint overlaps the working sphere; turned into MoveIt collision voxels |
| **Orange** | `CATEGORY_ORANGE = 2` | MEP element **behind** the wall in the adjacent space whose geometry intersects the working sphere; indicates a hidden hazard |
| **Blue** | `CATEGORY_BLUE = 3` | Selected element's footprint — the drill target |
| **Green** | `CATEGORY_GREEN = 4` | Reachable wall area with no obstruction |

Priority (highest first): red → blue → orange → green. A point can only carry one category.

### Shape-aware intersection

Both the sphere test and the wall footprint use the element's actual **volume**, not just its centre, so elongated elements like vertical pipes that pass through the sphere are correctly detected even when their centre lies outside it. The test is chosen from `shapeType`:

| `shapeType` | Sphere test | Wall footprint |
|---|---|---|
| `cylindrical` | Distance from the sphere centre to the axis segment (`center ± direction · length/2`) against `radius + sphere radius` — a capsule test | The axis segment projected onto the two wall-surface axes, dilated by `radius` |
| `rectangular` | Closest point on the AABB built from `sizeX/sizeY/sizeZ` | Half-extents of `sizeX/sizeY/sizeZ` on the two wall-surface axes |
| anything else | Closest point on `bbox_min`/`bbox_max` | Half-extents of the bbox on the two wall-surface axes |

If the geometry a branch needs is absent, it degrades to a centre-distance test and a ±150 mm square footprint. The wall-surface axes are the two axes perpendicular to the wall's dominant `axis2` component.

## Nodes

### pointcloud_publisher

Reads a pre-processed ASCII PCD file on startup and publishes the scan once as a latched `PointCloud2`. All other nodes use this topic as the shared spatial reference.

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `frame_id` | `world` | TF frame written into the message header |

**Topics (publisher)**

| Topic | Type | Description |
|---|---|---|
| `/cloud` | `sensor_msgs/PointCloud2` | Latched; XYZ float32 point cloud from `models/cloudGlobal_cleaned_excluded.pcd` |

---

### scene_graph_builder

Task-agnostic 3D scene-graph visualiser. Renders the consolidated node/edge graph that `drill_context_builder` publishes on `/scene_graph` as an RViz `MarkerArray`: one sphere plus a floating text label per node, and one `LINE_LIST` per relationship type.

Node centres arrive already resolved in the IFC frame (mm) — this node only scales them to metres, applies `/matrix`, and draws them. It never reads the raw IFC model, so any task context builder can feed the same visualiser.

Markers are rebuilt whenever `/scene_graph` or `/matrix` changes, and republished on a 1 s timer: RViz drops markers when a namespace is toggled off, and the steady republish restores them as soon as it is toggled back on.

**Marker styling**

| Node type | Color | Sphere diameter |
|---|---|---|
| `building` | purple | 0.3 m |
| `storey` | cyan | 0.3 m |
| `space` | green | 0.3 m |
| `wall` | light grey | 0.2 m |
| `mep` | orange | 0.1 m |

Edges are drawn at 0.02 m width, coloured per relationship type: `has_storey` purple, `has_space` cyan, `bounded_by` grey, `intersects` orange. Node types and relationship types outside these tables are skipped.

Marker namespaces are `nodes/<type>`, `edges/<type>`, and `labels`, so each layer can be toggled independently in RViz.

**Topics (subscriber)**

| Topic | Description |
|---|---|
| `/scene_graph` | `{nodes: [{id, type, name, center}], edges: [{type, source, target}]}` — centres in IFC mm |
| `/matrix` | 4×4 IFC-to-world transform |

**Topics (publisher)**

| Topic | Type | Description |
|---|---|---|
| `/scene_graph/markers` | `visualization_msgs/MarkerArray` | Latched; spheres + labels + edge line lists, republished every 1 s |

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `frame_id` | `world` | TF frame written into every marker header |

---

### task_distributor

Places an interactive sphere marker in RViz at the scan point nearest to each drillable element's wall-penetration centre. Clicking a marker publishes the element's ID to trigger the rest of the pipeline.

On startup it waits for `/cloud`, `/drilling/elements`, and `/matrix`. Once all three are available it transforms each element's penetration centre from IFC mm to the world frame, finds the closest cloud point, and registers a `BUTTON`-mode interactive marker there. The selected marker is recoloured blue; all others are red.

**Topics (subscriber)**

| Topic | Description |
|---|---|
| `/cloud` | Point cloud used to snap marker positions to the scan surface |
| `/drilling/elements` | List of drillable MEP elements with penetration geometry |
| `/matrix` | 4×4 IFC-to-world transform |

**Topics (publisher)**

| Topic | Type | Description |
|---|---|---|
| `/task/selected_element` | `std_msgs/String` | `{"id": "<mep_global_id>"}` — published on each marker click |

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `marker_diameter` | `0.1` | Diameter of each sphere marker in metres |

---

### task_representer

Computes the color-coded working zone on the target wall and publishes two point clouds: one for RViz rendering and one carrying raw category codes for the motion planner.

Recomputed whenever any of `/cloud`, `/matrix`, `/robot/target_position`, or `/drilling/context` is updated, once all four are present. The computation:

1. Transforms the robot base position to world frame and offsets upward by `CENTER_Z_OFFSET` (0.529 m) to get the shoulder centre.
2. Selects wall points from `/cloud` using a pre-built wall-ID → point-index map loaded from the companion CSV file, and converts them back to IFC mm with the inverse of `/matrix` (element geometry stays in IFC units throughout).
3. Tests each wall point for sphere membership (distance ≤ 1.0 m from shoulder).
4. For each entry in `nearby_elements` (from `/drilling/context`), runs the shape-aware sphere test and projects the element's footprint onto the wall surface. Tags points `robot_side=true` → RED, `robot_side=false` → ORANGE.
5. Projects the selected element's penetration footprint (radius for cylindrical, `sizeX/Y/Z` for rectangular, bbox otherwise — from `/drilling/elements`) onto the wall → BLUE.
6. Remaining in-sphere points → GREEN.
7. Publishes both outputs and logs the validation block.

**Topics (subscriber)**

| Topic | Description |
|---|---|
| `/cloud` | Full scan; wall subsets are indexed at startup from the CSV |
| `/matrix` | 4×4 IFC-to-world transform (also used for inverse, IFC←world) |
| `/robot/target_position` | Robot base position used to locate the shoulder sphere centre |
| `/drilling/context` | Selected element context including wall id and nearby elements |
| `/drilling/elements` | Used to look up the selected element's penetration centre for the blue zone |

**Topics (publisher)**

| Topic | Type | Description |
|---|---|---|
| `/task/representation` | `sensor_msgs/PointCloud2` | XYZRGB cloud for RViz; one coloured point per wall point in the working sphere |
| `/task/zones` | `sensor_msgs/PointCloud2` | XYZ + `uint8 category` cloud consumed by `robot_motion_planner` to build MoveIt collision objects for RED-category points |

## Validation output

On every zone recomputation `task_representer` writes section **B[0] Hazard zone classification** to the shared validation log via `robot_validation.ValidationLog` (stdout plus `src/robot_validation/output/project.log`, overridable with `GRAPH2ROBOT_VALIDATION_LOG`). Sections B[1]–B[3] of the same block come from `robot_motion_planner`.

The block records the working sphere's centre and radius, then one line per in-scope element comparing two labels:

- **classified** — what the point-based zone builder actually produced: `DANGER` (RED points under the footprint), `LATENT` (ORANGE points), `TARGET` (the selected element's BLUE footprint produced points), or `OUT_OF_ZONE` when no wall points fell under it.
- **truth** — derived straight from BIM geometry: the selected element is `TARGET`; an element whose volume misses the working sphere is `OUT_OF_ZONE`; otherwise robot-side is `DANGER` and behind-wall is `LATENT`.

It closes with per-zone counts and a match tally. A mismatch means the element is geometrically inside the sphere but has no wall-scan points beneath it, so the classifier cannot voxelise it — i.e. a scan-coverage gap rather than a geometry error.
