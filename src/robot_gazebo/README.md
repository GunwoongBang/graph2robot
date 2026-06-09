# ROBOT_GAZEBO

`robot_gazebo` is the simulation and execution package. It populates the Gazebo world with IFC-derived geometry, spawns and teleports the Husky+UR5e robot to the computed drilling position, and drives MoveIt to plan and execute the arm trajectory to each drill target point.

The package contains three custom nodes. The launch file also starts `robot_state_publisher`, MoveIt `move_group`, and the `joint_state_broadcaster` / `ur5e_arm_controller` spawners.

```
/matrix ──► world_spawner    (IFC models → Gazebo)

/robot/target_position ──► robot_spawner ──► /robot/motion_ready ──► robot_motion_planner
                                                                            │
                                          /task/zones ──────────────────────┤ (collision voxels)
                                          /drilling/context ─────────────────┤ (target wall box)
                                          /robot/target_point ───────────────┘
                                                                            │
                                                              /robot/motion_done
                                                              /robot/motion_failed
                                                              /task/drill_caution
```

## Robot

**Husky A200** (mobile base) + **UR5e** (6-DOF arm) combined as a single URDF entity `husky_ur5e`. The arm is controlled via `gazebo_ros2_control` with a `FollowJointTrajectory` action served by `ur5e_arm_controller`.

- Reach: 850 mm + 150 mm drill tip = **1.0 m working radius**
- Shoulder height above base: **0.529 m** (`CENTER_Z_OFFSET`)
- Planning group: `ur5e_arm`, end-effector link: `drill_tip`

## Nodes

### world_spawner

Reads the IFC-derived Gazebo world file (`models/worlds/ifc_world.sdf`) on startup, extracts every model whose name starts with `Ifc`, and spawns them all into Gazebo at the correct world-frame pose derived from the `/matrix` transform.

Waits for `/matrix` before spawning so the IFC geometry lands exactly where the point cloud scan expects it. Models are spawned sequentially (one callback chain) to avoid flooding the `/spawn_entity` service.

**Topics (subscriber)**

| Topic | Description |
|---|---|
| `/matrix` | 4×4 IFC-to-world transform; used once to compute spawn pose from the translation + rotation columns |

**Services (client)**

| Service | Description |
|---|---|
| `/spawn_entity` | Gazebo service; called once per IFC model |

---

### robot_spawner

Manages the lifecycle of the `husky_ur5e` robot entity in Gazebo. Spawns it at the world origin on startup, then teleports it to the computed drilling position whenever `/robot/target_position` is received. After each successful teleport it broadcasts the updated `base_link` TF and signals the motion planner that the base is at rest.

**Robot base position** is given in IFC frame (meters). The node transforms it to world frame via `/matrix`, fixes Z to `FLOOR_HUSKY_Z = -0.405 m`, and computes yaw from the heading vector (`hx`, `hy`) so the robot faces the wall.

**Topics (subscriber)**

| Topic | Description |
|---|---|
| `/robot_description` | URDF XML; triggers the initial spawn at origin |
| `/matrix` | 4×4 IFC-to-world transform |
| `/robot/target_position` | `{target_position: {x, y, z, hx, hy}}` — IFC-frame base pose |

**Topics (publisher)**

| Topic | Description |
|---|---|
| `/robot/motion_ready` | `std_msgs/Empty` — published after each successful teleport; triggers `robot_motion_planner` |

**Services (client)**

| Service | Description |
|---|---|
| `/spawn_entity` | Gazebo service; called once on startup |
| `/gazebo/set_entity_state` | Gazebo service; called on each teleport |

---

### robot_motion_planner

Plans and executes arm trajectories using MoveIt. Triggered by `/robot/motion_ready` (base has settled). Before planning it builds a MoveIt collision scene from the zone data and the target wall geometry, so the arm avoids on-wall RED-zone obstacles during its approach.

**Startup sequence (per drilling task):**

1. **Orange caution check** — reads `nearby_elements` from `/drilling/context`; publishes a `/task/drill_caution` warning if any far-side (ORANGE) elements are present.
2. **Collision scene** — constructs two `CollisionObject` entries:
   - `task_danger_zones`: one 3 cm box voxel per RED-category point from `/task/zones`.
   - `target_wall`: a single box matching the wall's full bounding box from `/drilling/context`.
3. **MoveGroup goal** — sends a pose goal to `/move_action` for `drill_tip` at each point in `/robot/target_point`, with a 5 mm position tolerance and ~3° orientation tolerance. Up to 3 planning attempts per point.
4. **Multi-point sequencing** — after each successful point the arm dwells for `INTER_POINT_DELAY = 5 s` before moving to the next. After the last point it dwells `READY_POSE_DELAY = 5 s` then returns the arm to the named `ready` pose.
5. **Completion** — publishes `/robot/motion_done` on success, `/robot/motion_failed` (with reason JSON) on exhausted retries.

**Topics (subscriber)**

| Topic | Description |
|---|---|
| `/robot/target_point` | `{shape_type, points: [{x,y,z,nx,ny,nz}], wall_id, mep_id}` — drill-tip targets |
| `/task/zones` | `PointCloud2` with `uint8 category`; RED points become collision voxels |
| `/drilling/context` | Provides wall bbox for the target-wall collision object and nearby elements for caution check |
| `/matrix` | 4×4 IFC-to-world transform for coordinate conversion |
| `/robot/motion_ready` | Triggers planning; ignored if already busy |

**Topics (publisher)**

| Topic | Type | Description |
|---|---|---|
| `/robot/motion_done` | `std_msgs/Empty` | Published after all points complete and arm returns to ready |
| `/robot/motion_failed` | `std_msgs/String` | `{"reason": "..."}` — published after all retries exhausted |
| `/task/drill_caution` | `std_msgs/String` | `{"orange_caution": bool, "conflicting_element_count": N}` |

**Services (client)**

| Service | Description |
|---|---|
| `/apply_planning_scene` | MoveIt service; used to push collision objects before each plan |

**Action (client)**

| Action | Description |
|---|---|
| `/move_action` | `moveit_msgs/MoveGroup`; used for both drill-pose goals and the final ready-pose goal |
