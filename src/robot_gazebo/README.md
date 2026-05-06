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
using services **SapwnEntity** & **DeleteEntity**
- To attach a robot
```bash
ros2 run gazebo_ros spawn_entity.py -entity <entity_name> -file install/robots/..path../robot.urdf -x 0 -y 0 -z 0
```
Note: ur5e requires only 2 arguments -> dont really needt to figure it out tho

- To detach a robot
```bash
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: '<entity_name>'}" 
```

we are going to use these calls to spawn and delete husky5e robot
+ when a task (mep element) is selected, gazebo receives the information and spawn the robot in the drilling position. 
+ But the drilling point needs to be designated considering the drilling point and a wall's orientation, idk
