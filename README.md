# `project_init`

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