# ROBOT_GRAPH
`robot_graph` is a package that facilitates communication between robot operation and the BIM-derived graph in Neo4j. It exposes the BIM model as ROS 2 services so that `drill_context_builder` in `robot_task` can query entities and their relationships on demand.

Every service is a `std_srvs/Trigger`. On success the response carries a JSON array of BIM entities in the `message` field; on failure `success` is `false` and `message` holds the reason. Every entity follows a common schema: a global IFC id, a type string taken from the node's `ifcClass` property, a flat `attributes` map, and a `relationship` list that captures outgoing edges in the Neo4j graph. The direction of each relationship is from the current entity toward its target, mirroring the arrow direction in the graph:

![image](../../images/triple.png)

For example, a wall entity that owns a layer and is penetrated by a pipe looks like this:

```json
{
  "id": "3S3VgMDFLDiPHeIJPk5HIn",
  "type": "IfcWallStandardCase",
  "attributes": {
    "axis2": [0.0, 1.0, 0.0],
    "directionSense": "NEGATIVE",
    "center": [1322.0, 3300.0, 1500.0],
    "bbox_min": [1200.0, 3246.0, 0.0],
    "bbox_max": [1450.0, 3354.0, 3000.0]
  },
  "relationship": [
    {"type": "has_layer",     "id": "<layer_global_id>"},
    {"type": "penetrated_by", "id": "<mep_global_id>",
     "center": [1322.0, 3300.0, 900.0], "depth_mm": 108.0, "radius": 16.7,
     "sizeX": null, "sizeY": null, "sizeZ": null}
  ]
}
```

The `penetrated_by` edge carries the penetration geometry itself — `center`, `depth_mm`, and then either `radius` (cylindrical) or `sizeX`/`sizeY`/`sizeZ` (rectangular). This is what makes a wall drillable: `robot_task` reads the hole's position and depth straight off the edge rather than intersecting geometry at runtime.

And a space that bounds a wall and is intersected by MEP elements:

```json
{
  "id": "20rBv4HojAY8M3avWLH9f$",
  "type": "IfcSpace",
  "attributes": {
    "name": "1",
    "longName": "Room",
    "centroid": [1200.0, 4500.0, 1500.0],
    "bbox_min": [0.0, 3354.0, 0.0],
    "bbox_max": [2400.0, 5600.0, 3000.0]
  },
  "relationship": [
    {"type": "bounded_by", "id": "<wall_global_id>", "side": "POSITIVE"},
    {"type": "intersects", "id": "<mep_global_id>"}
  ]
}
```

The `side` on `bounded_by` is what lets `drill_context_builder` decide which face of the wall the robot approaches from, by comparing it against the wall's `directionSense`.

An MEP element carries the shape parameters the zone builder and motion planner rely on:

```json
{
  "id": "1iDZV_Brn0Z9GW0EFhIn_p",
  "type": "IfcFlowSegment",
  "attributes": {
    "name": "Pipe Types:Default:1208035",
    "center": [1322.0, 3283.49, 900.0],
    "shapeType": "cylindrical",
    "direction": [0.0, 1.0, 0.0],
    "radius": 16.7,
    "length": 108.0,
    "sizeX": null, "sizeY": null, "sizeZ": null,
    "bbox_min": [1305.3, 3246.0, 883.3],
    "bbox_max": [1338.7, 3354.0, 916.7]
  },
  "relationship": []
}
```

`shapeType` selects which parameter set is populated: `cylindrical` uses `direction`/`radius`/`length`, `rectangular` uses `sizeX`/`sizeY`/`sizeZ`, and the other set is `null`. Consumers fall back to `bbox_min`/`bbox_max` when neither is available.

## Launch

```bash
ros2 launch robot_graph robot_graph.launch.py
```

Runs `graph_server` in the `robot_graph` namespace. Service names are absolute, so they stay at `/graph/list_*` regardless of the namespace.

Credentials come from `NEO4J_URI`, `NEO4J_USER`, and `NEO4J_PASSWORD`, loaded via `python-dotenv` — a `.env` file in the working directory is picked up automatically.

## Nodes

### graph_server

Connects to Neo4j on startup and verifies the connection with a `RETURN 1` probe. Exposes six services that query the database and return results as a JSON array in the `Trigger` response message field.

Missing credentials or a failed connection are logged as errors but do not stop the node: the services are still advertised and each returns `success: false` with `Neo4j driver is not connected.` A query that raises returns `success: false` with the exception text. Either way `drill_context_builder` logs the failure and carries on with whatever it did receive, so a partial graph degrades rather than deadlocks.

**Services (server)**

| Service | Neo4j label | Description |
|---|---|---|
| `/graph/list_buildings` | `Building` | Buildings with `has_storey` relationships |
| `/graph/list_storeys` | `Storey` | Storeys with `has_space` relationships |
| `/graph/list_spaces` | `Space` | All rooms and MEP corridors with their `bounded_by` and `intersects` relationships |
| `/graph/list_walls` | `Wall` | All walls with `has_layer` and `penetrated_by` relationships |
| `/graph/list_layers` | `Layer` | All wall material layers with thickness and `layerIndex` |
| `/graph/list_mep_elements` | `MEPElement` | All MEP elements (pipes, fixtures) with shape and bbox attributes |

Each entity's reported `type` is the node's own `ifcClass` property, so it reflects the actual IFC class (`IfcFlowSegment`, `IfcFlowTerminal`, …) rather than a fixed string per service.

**Query limit** — every query is capped at **50** rows by `_query_limit` in `graph_server.py`. A model with more than 50 of any entity type is silently truncated, which surfaces downstream as missing walls or unmarked elements rather than an error.

**Queries** — all Cypher lives in [util/query_handler.cypher](robot_graph/util/query_handler.cypher), split into `-- name: <KEY>` blocks that `graph_util.py` parses into a dict at import time. Adding a query means adding a block plus a thin `query_*` wrapper. The file also holds a `QUERY_MEP_SYSTEMS` block that is currently unused (its loader line is commented out).
