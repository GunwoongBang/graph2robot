# Robot Spec — `husky_ur5e`

A **Clearpath Husky** mobile base carrying a **Universal Robots UR5e** 6-DOF arm
with a drill end-effector. Used in this project as the platform that drills MEP
penetrations through walls.

## Overview

| | |
|---|---|
| Robot name | `husky_ur5e` |
| Mobile base | Clearpath Husky (4-wheel differential drive) |
| Manipulator | Universal Robots UR5e (6-DOF) |
| End-effector | Drill (`drill_link` → `drill_tip`) |
| Planning group | `ur5e_arm` (MoveIt) |
| Control | ros2_control + MoveIt `FollowJointTrajectory` (mock/fake system) |

## Mobile base — Clearpath Husky

- 4-wheel differential drive: `front_left_wheel`, `front_right_wheel`,
  `rear_left_wheel`, `rear_right_wheel`.
- Structure: `base_link` / `base_footprint` → `top_chassis_link`, bumper mounts,
  and `robot_arm_plate` (the UR mounting plate on top of the chassis).
- Anchored to the world via a fixed `world_joint` (parent `world` → `base_link`).
  In this configuration the base is **fixed**, not driven — planning is
  arm-only.

## Manipulator — UR5e

Planning group `ur5e_arm`: kinematic chain `ur_arm_base_link` → `ur_arm_tool0`.

**Physical characteristics (UR5e stock):** ~850 mm reach, 5 kg payload,
±0.03 mm repeatability, ~20.6 kg mass. *(Not overridden in config — inherited
from the UR5e description.)*

### Joints (all prefixed `ur_arm_`)

| Joint | Position limit | Velocity | Effort |
|---|---|---|---|
| `shoulder_pan_joint`  | ±360° (±2π rad) | π rad/s (180°/s) | 150 N·m |
| `shoulder_lift_joint` | ±360° (±2π rad) | π rad/s (180°/s) | 150 N·m |
| `elbow_joint`         | ±180° (±π rad)  | π rad/s (180°/s) | 150 N·m |
| `wrist_1_joint`       | ±360° (±2π rad) | π rad/s (180°/s) | 28 N·m |
| `wrist_2_joint`       | ±360° (±2π rad) | π rad/s (180°/s) | 28 N·m |
| `wrist_3_joint`       | ±360° (±2π rad) | π rad/s (180°/s) | 28 N·m |

- Default MoveIt motion scaling: **0.1** velocity / **0.1** acceleration
  (deliberately slow); no acceleration limits set in `joint_limits.yaml`.
- IK solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`, search resolution
  and timeout both 5 ms.

### Named states (SRDF)

| State | shoulder_pan | shoulder_lift | elbow | wrist_1 | wrist_2 | wrist_3 |
|---|---|---|---|---|---|---|
| `home`  | 0 | −90° | 0    | 0    | 0    | 0 |
| `ready` | 0 | −90° | +90° | −90° | −90° | 0 |

## End-effector — Drill

- Chain: `ur_arm_tool0` → `drill_link` → `drill_tip`
  (fixed joints `tool0_to_drill`, `drill_to_tip`).
- Collision checking disabled between `drill_link` and the three wrist links
  (`wrist_1/2/3`).

## Control

- Controller manager: `moveit_simple_controller_manager/MoveItSimpleControllerManager`.
- Single controller `ur5e_arm_controller`, type `FollowJointTrajectory`
  (action ns `follow_joint_trajectory`), driving the 6 arm joints in order:
  `shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`, `wrist_2`, `wrist_3`.
- Runs on a fake/mock ros2_control system; all joints initialise at 0.

## Config sources

- URDF: [husky_ur5e.urdf](husky_ur5e.urdf)
- Xacro: [husky_ur5e.urdf.xacro](../../../husky_ur5e_moveit_config/config/husky_ur5e.urdf.xacro)
- MoveIt config: [src/husky_ur5e_moveit_config/config/](../../../husky_ur5e_moveit_config/config/)
  (SRDF, `joint_limits.yaml`, `kinematics.yaml`, `moveit_controllers.yaml`,
  `initial_positions.yaml`)
