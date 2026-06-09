# ROBOT_GRAPH
`robot_graph` is a package that facilitates communication between robot operation and the BIM-derived graph in Neo4j. It exposes the BIM model as ROS 2 services so that `task_manager` in `robot_task` can query entities and their relationships on demand.

Each service response contains a JSON array of BIM entities. Every entity follows a common schema: a global IFC id, a type string, a flat `attributes` map, and a `relationship` list that captures outgoing edges in the Neo4j graph. The direction of each relationship is from the current entity toward its target, mirroring the arrow direction in the graph:

![image](../../images/triple.png)

For example, a wall entity that owns a layer and is penetrated by a pipe looks like this:

```json
{
  "id": "3S3VgMDFLDiPHeIJPk5HIn",
  "type": "IfcWallStandardCase",
  "attributes": {
    "name": "Basic Wall:Generic - 90mm Masonry:1234",
    "axis2": [0.0, 1.0, 0.0],
    "directionSense": "NEGATIVE",
    "center": [1322.0, 3300.0, 1500.0],
    "bbox_min": [1200.0, 3246.0, 0.0],
    "bbox_max": [1450.0, 3354.0, 3000.0]
  },
  "relationship": [
    {"type": "has_layer",     "id": "<layer_global_id>"},
    {"type": "penetrated_by", "id": "<mep_global_id>",
     "center": [1322.0, 3300.0, 900.0], "depth_mm": 108.0, "radius": 16.7}
  ]
}
```

And a space that bounds a wall and hosts MEP elements:

```json
{
  "id": "20rBv4HojAY8M3avWLH9f$",
  "type": "IfcSpace",
  "attributes": {},
  "relationship": [
    {"type": "bounded_by", "id": "<wall_global_id>", "side": "POSITIVE"},
    {"type": "hosts",      "id": "<mep_global_id>"}
  ]
}
```

## Nodes

### graph_server

Connects to Neo4j on startup using credentials from the environment variables `NEO4J_URI`, `NEO4J_USER`, and `NEO4J_PASSWORD`. Exposes four services that query the database and return results as a JSON array in the `Trigger` response message field.

**Services (server)**

| Service | Entity type | Description |
|---|---|---|
| `/graph/list_spaces` | `IfcSpace` | All rooms and MEP corridors with their `bounded_by` and `hosts` relationships |
| `/graph/list_walls` | `IfcWallStandardCase` | All walls with `has_layer` and `penetrated_by` relationships |
| `/graph/list_layers` | `IfcMaterialLayer` | All wall material layers with thickness and order index |
| `/graph/list_mep_elements` | `IfcDistributionElement` | All MEP elements (pipes, fixtures) with geometry attributes |
