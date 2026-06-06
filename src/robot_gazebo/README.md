# Package: robot_gazebo

robot: husky + ur5e

husky: steering incomplete -> maybe cmd_vel-based control is not appropriate for the robot idk

world: current surface model -> full ifc world model (done)

## ROS2-Robot overview
- Clearpath Husky A200: https://docs.clearpathrobotics.com/docs/ros2humble/ros/
- Universal Robots UR5e: https://docs.ros.org/en/humble/p/ur_description/

### install necessary packages
- clearpath_control: Controllers for Clearpath Robotics platforms
```bash
sudo apt update
sudo apt install ros-humble-clearpath-control
```

- clearpath_path_description: Clearpath URDF descriptions metapackage
```bash
sudo apt update
sudo apt install ros-humble-clearpath-description
```

- ur_description: URDF description for Universal Robots
```bash
sudo apt update
sudo apt install ros-humble-ur-description
```

## Robot attach/detach
using services **SpawnEntity** & **DeleteEntity**
- To attach a robot
```bash
ros2 run gazebo_ros spawn_entity.py -entity husky_ur5e -file install/robot_gazebo/share/robot_gazebo/models/robots/husky_ur5e.urdf -x 0 -y 0 -z 0
```
Note: ur5e requires only 2 arguments -> dont really needt to figure it out tho

- To detach a robot
```bash
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'husky_ur5e'}" 
```

we are going to use these calls to spawn and delete husky5e robot
+ when a task (mep element) is selected, gazebo receives the information and spawn the robot in the drilling position. 
+ But the drilling point needs to be designated considering the drilling point and a wall's orientation, idk

After creating 



---
### Phase A — Make MoveIt drive the Gazebo arm (not just the mock).

Right now your URDF's arm <ros2_control> block uses mock_components/GenericSystem (Setup Assistant default). That works for demo.launch.py but in Gazebo the arm won't physically move. Swap to:

```xml
<plugin>gazebo_ros2_control/GazeboSystem</plugin>
And add (somewhere in the URDF, top-level <gazebo>):
```

```xml
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find husky_ur5e_moveit_config)/config/ros2_controllers.yaml</parameters>
  </plugin>
</gazebo>
```
This lets gazebo_ros2_control hand off MoveIt's FollowJointTrajectory goals into the Gazebo physics joints.

### Phase B — One launch file for everything.

Compose robot_gazebo.launch.py to also start MoveIt's move_group and the controller spawners. Include:

- Gazebo (already there)
- world_spawner, robot_spawner (already there)
- move_group.launch.py from husky_ur5e_moveit_config
- spawn_controllers.launch.py from the moveit config (boots joint_state_broadcaster + ur5e_arm_controller)

Verify by launching once and confirming, in RViz's MotionPlanning panel, that Plan & Execute moves the Gazebo arm (not just the RViz preview).

### Phase C — Replace the 5-second placeholder with real drilling motion.

Write robot_motion_planner in robot_gazebo:

1. Subscribe /robot/target_position (IFC frame) + /matrix → compute cloud-frame drill pose for ur_arm_tool0.
2. Subscribe /task/filtered_elements + /task/target_wall + /ifc/walls → add the wall and nearby MEPs as CollisionObjects to the planning scene (/planning_scene).
3. Call MoveIt to plan to the drill pose using MoveItPy (Python API) or the MoveGroupInterface action.
4. Execute.
5. Publish a "drill complete" signal that robot_spawner consumes — replacing its 5-second timer.

How to refactor drilling specific logic?
- in the previous stage they were given color-based status, and those are using in the `motion_planner` to trigger specific robot motions (like speed up, stop, etc.), and there are different motions you can choose like
  - RED: stop
  - ORANGE: print warning message, slow down, etc.
  - BLUE, GREEN: -