# `robot_graph`
`robot_graph` is a package that talks with Neo4j graph database where the BIM-derived graph is stored. It is responsible for retrieving and updating the graph

# `robot_task`
`robot_task` is a package that ...

# `robot_rviz`
`robot_rviz` is a package that ...

# `robot_gazebo`
`robot_gazebo` is a package that ...



Neo4j graph database
->
robot_graph
->
robot_task
->
robot_rviz and robot_gazebo



### Next step
ok now we are going to place the drilling tip at the point on the wall surface where a hole should be placed. but i think we dont have to do the motion planning manually how should it be performed?

the robot drill tip is approaching to the target_point on the wall

before doing the motion planning, id like to make robot_task publish target point first
target point is a point where the robot drilling tip should be located. it is on a wall and it also contains wall target depth, thickness, layer info. 

To do so, robot_graph should publish the elements first and then robot_client redistribute them to robot_task, the point is how to combine them and what to you to process the target_point

Then how do i get the surface information of the mep element cause we are only dealing with the point cloud which is of all the surface elements of the space -> should we replace the way of presenting the mep element?

### Next step
So the motion planning has been covered somehow. But there are still some tasks left.
1. We are dealing not only with pipes but receptacles or light switches. But they have a different mechanisms with robot-pipe operation because pipe only requires one big hole drilling in the center, while those elements need 4 holes in its each corner. 
    - Then how can I encode the task information in each mep element?
    - To do so, the orientation of robot should be changed because the current robot position is too close to wall to work with the lower receptacles
2. Wall information is still missing. It does not need to be exhibited during the robot operation but it is required. (DONE)
    - Then what kind of information is needed
        + Wall thickness
        + drilling depth
        + layer info.
    - How you want to show it?
        + in the terminal --> so that it can be later shown in a customized UI
3. The whole structure needs to be modularized
    - Current graph query rule is messed up need to set a strict query rule 
    - Current modules connected to task_manager are too much biased to the drilling task.
    - `task_manager` publishes not each bim element but one message containing everything required as a chuck.
    - This might be the heaviest work
4. Minor fixes
    - Sometimes, the robot is rotating while moving its robot arm
        + Seems fixed but need to keep tracking of it
    - Too many logs, make the log output slimmer
        + server: returned
        + client: requested
        + publisher: published
        + subscriber: received
    - Still need to clean the task_representation