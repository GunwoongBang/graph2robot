# ROBOT_RVIZ

`robot_rviz` is the user-interface package. It loads the point cloud scan into RViz, overlays clickable interactive markers on every drillable MEP element, and computes a color-coded working zone on the target wall whenever an element is selected.

The package runs three nodes: a static point-cloud publisher, an interactive-marker layer that handles selection, and a zone-computation node whose output feeds both RViz visualization and the motion planner.

```
/cloud ──────────────────────────────────► task_representer ──► /task/representation  (RViz color view)
                                                │                /task/zones            (MoveIt collision input)
/drilling/elements ──► task_distributor ──► /task/selected_element
                             │
                    interactive markers (RViz)
```

## Zone color coding

When an element is selected, `task_representer` projects a working sphere (radius 1.0 m, centred at the robot shoulder 0.529 m above the base) onto the target wall's point cloud and assigns each point a category:

| Color | Category | Meaning |
|---|---|---|
| **Red** | `CATEGORY_RED = 1` | Another MEP element in the robot's own space whose footprint overlaps the working sphere; turned into MoveIt collision voxels |
| **Orange** | `CATEGORY_ORANGE = 2` | MEP element **behind** the wall in the adjacent space whose geometry intersects the working sphere; indicates a hidden hazard |
| **Blue** | `CATEGORY_BLUE = 3` | Selected element's footprint — the drill target |
| **Green** | `CATEGORY_GREEN = 4` | Reachable wall area with no obstruction |

Priority (highest first): red → blue → orange → green. A point can only carry one category.

The sphere-element intersection is tested against the element's full **bounding box** (not just its centre), so elongated elements like vertical pipes that pass through the sphere are correctly detected even when their centre lies outside it.

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

Triggered whenever `/robot/target_position` or `/drilling/context` is updated. The computation:

1. Transforms the robot base position to world frame and offsets upward by `CENTER_Z_OFFSET` (0.529 m) to get the shoulder centre.
2. Selects wall points from `/cloud` using a pre-built wall-ID → point-index map loaded from the companion CSV file.
3. Tests each wall point for sphere membership (distance ≤ 1.0 m from shoulder).
4. For each entry in `nearby_elements` (from `/drilling/context`), tests AABB–sphere intersection and projects the element's bounding box footprint onto the wall surface. Tags points `robot_side=true` → RED, `robot_side=false` → ORANGE.
5. Projects the selected element's penetration bbox onto the wall → BLUE.
6. Remaining in-sphere points → GREEN.
7. Publishes both outputs.

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
