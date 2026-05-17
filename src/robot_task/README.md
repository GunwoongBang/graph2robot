tools/generate_transform.py -> run script to generate a transformation matrix
tools/visualize_alignment.py -> check the alignment visually
run task_publisher node to publish a topic
topic is an MEP element, retrieved from the Neo4j graph database and the position is transformed with the transformation matrix


In this package there are (currently) two nodes:
- task_publisher: publishes a target task (mep element) as a topic
- matrix_publisher: publishes the transformation matrix as a topic

Launch file binds the two nodes and deploy them together

---
task_generator
- it receives the /task/selected_task topic containing the element id
- computes the drilling position considering the drilling point
- to compute it, what is required?
    - drilling point (already have as "center")
    - wall's normal direction (maybe we can utilize the attribute "side")
    - z position (can be extracted from the transformation matrix maybe and there is the /matrix task that we can make use of)
- so finally the node `task_generator` publishes a topic called `/drilling_position` that contains (x, y, z) where the robot should be located when a task is clicked

Schritt fuer Schritt
1. first, retrieve the center point (done already) (DONE)
2. graph_client asks for wall information to graph_server (DONE)
3. from the graph_server, we are receiving a wall information including axis2 which is a denominator of in which direction the wall is lying (DONE)
4. task_generator now calculates the drilling position with the attributes "center" from /task/selected_task, and "axis2" from /walls (info collected, calculation ready)

### Task representation
1. first, from the robot urdf file, it calculates the furthrest reachable area (working_area) and creates a virtual sphere -> are we going to create a task node in neo4j and attach/detach it throughout the pipeline?
2. then the robot inspects the potential mep elements that should consider before the task execution
    - but how? using the topology with the bim graph maybe...
3. the points outside of the spherical area && the target wall are excluded
4. remaining points are color-coded
    - task: blue (filter with the bbox)
    - danger zone: red (when there is an mep element behind the wall)
---
**I think we should have a task node per robot that can be attached and detached to a task (can be mep element, wall, etc.)**
- the task node contains
    - target position (where a robot spawns) -> rename `/drilling_position` to `/target_position`
    - robot working area (sphere)
    - affordances
- when a task is clicked, the task node is connected with the target node during the execution then disconnected