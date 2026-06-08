# ROBOT_GRAPH
`robot_graph` is a package that facilitates the communication between robot operation and the BIM-derived graph in Neo4j graph database. It is responsible for providing BIM elements and relationships between them when `graph_client` in `robot_task` requests them.

Each topic consists of a certain type of BIM elements such as `IfcSpace` or `IfcWallStandardCase` and relationship data. The included relationship is defined by the direction of an arrow connected to the target element. For instance, when an `IfcMaterialLayer` is connected to an `IfcWallStandardCase` and their relationship `HAS_LAYER` is coming from `IfcWallStandardCase` and heading to `IfcMaterialLayer`, like the image below:

![image](../../images/triple.png)

the expected topic looks like below:
```json
// /ifc/layers
{
  "id": "...",
  "type": "IfcMaterialLayer",
  "attributes": {
    "name": "Gypsum", "layerIndex": 0, "thickness": 12.5
  }
}
```

This structure faciliates the `task_manager` to generate a task-specific graph from the raw Neo4j graph.

## Nodes

### robot_graph
