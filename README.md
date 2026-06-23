# graph2robot

A ROS 2 (Humble) system that turns a BIM model stored in Neo4j into executable robot tasks. A Husky+UR5e robot is placed automatically in front of a selected MEP element, a working-zone hazard map is computed on the target wall, and MoveIt plans and executes the arm trajectory to each target point.

## System architecture

```mermaid
flowchart TB
    Neo4j[("Neo4j<br/>Graph DB")]

    robot_graph["robot_graph"]
    robot_task["robot_task"]
    robot_rviz["robot_rviz"]
    robot_gazebo["robot_gazebo"]

    Neo4j -->|"exports BIM model"| robot_graph
    robot_graph -->|"serves BIM entities"| robot_task
    robot_task -->|"drillable elements <br>+ drill plan"| robot_rviz
    robot_task -->|"robot pose <br>+ drill targets"| robot_gazebo
    robot_rviz -->|"user-selected element"| robot_task
    robot_rviz -->|"working-zone hazard map"| robot_gazebo

    classDef extern fill:#374151,stroke:#fbbf24,color:#fff;
    class Neo4j extern;
```

## Packages

### [robot_graph](src/robot_graph/README.md)

Connects to Neo4j and exposes the BIM model as ROS 2 services. Each service returns a JSON array of IFC entities following a common `{id, type, attributes, relationship[]}` schema.

| Node | Role |
|---|---|
| `graph_server` | Serves `/graph/list_spaces`, `/graph/list_walls`, `/graph/list_layers`, `/graph/list_mep_elements` |

---

### [robot_task](src/robot_task/README.md)

Transforms raw BIM data into executable task parameters: which wall to drill, which direction the robot faces, in what order the layers are encountered, and what points the drill tip must reach.

| Node | Role |
|---|---|
| `task_manager` | Calls graph services on startup; publishes `/ifc/*` topics and `/matrix` |
| `drill_context_builder` | Builds per-selection drill context (facing, layers, nearby elements) on `/drilling/context` |
| `drill_executor` | Computes robot base pose (`/robot/target_position`) and drill-tip target points (`/robot/target_point`) |

---

### [robot_rviz](src/robot_rviz/README.md)

User interface. Loads the point-cloud scan, renders clickable markers on every drillable element, and computes a color-coded hazard map on the target wall.

| Node | Role |
|---|---|
| `pointcloud_publisher` | Publishes the scan once on `/cloud` (latched) |
| `task_distributor` | Places interactive markers in RViz; publishes `/task/selected_element` on click |
| `task_representer` | Computes working-sphere zones on the wall; publishes `/task/representation` (RViz) and `/task/zones` (MoveIt) |

**Zone colors**

| Color | Meaning |
|---|---|
| Blue | Selected drill target |
| Red | On-robot-side MEP element — MoveIt collision voxel |
| Orange | Behind-wall MEP element intersecting the working sphere — hidden hazard |
| Green | Clear reachable area |

---

### [robot_gazebo](src/robot_gazebo/README.md)

Simulation and execution. Populates the Gazebo world with IFC geometry, teleports the robot to the computed position, and drives MoveIt to execute the arm trajectory. Stops the arm if a behind-wall element lies within the drill depth.

| Node | Role |
|---|---|
| `world_spawner` | Spawns IFC-derived Gazebo models at the correct world pose |
| `robot_spawner` | Spawns and teleports the Husky+UR5e; signals `/robot/motion_ready` after each teleport |
| `robot_motion_planner` | Builds MoveIt collision scene from RED-zone voxels and target wall; plans and executes arm trajectory; stops on depth conflict with behind-wall elements |

## Prerequisites

```bash
sudo apt install ros-humble-clearpath-control
sudo apt install ros-humble-clearpath-description
sudo apt install ros-humble-ur-description
```

Neo4j credentials must be set in the environment or in a `.env` file (without `export`):

```bash
export NEO4J_URI=neo4j://localhost:7687
export NEO4J_USER=neo4j
export NEO4J_PASSWORD=<password>
```

## Launch

```bash
# Terminal 1 — BIM graph
ros2 launch robot_graph robot_graph.launch.py

# Terminal 2 — Task pipeline
ros2 launch robot_task drilling_task.launch.py

# Terminal 3 — Visualization
ros2 launch robot_rviz robot_rviz.launch.py

# Terminal 4 — Simulation + execution
ros2 launch robot_gazebo robot_gazebo.launch.py
```
