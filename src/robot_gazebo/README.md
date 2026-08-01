# ROBOT_GAZEBO

`robot_gazebo` is the simulation and execution package. It populates the Gazebo world with IFC-derived geometry, spawns and teleports the Husky+UR5e robot to the computed drilling position, and drives MoveIt to plan and execute the arm trajectory to each drill target point.

The package contains three custom nodes. The launch file also starts Gazebo (`empty_world.sdf`), `robot_state_publisher`, MoveIt `move_group`, and the `joint_state_broadcaster` / `ur5e_arm_controller` spawners.

```
/matrix ──► world_spawner    (IFC models → Gazebo)

/robot/target_position ──► robot_spawner ──► /robot/motion_ready ──► robot_motion_planner
                                                                            │
                                          /task/zones ──────────────────────┤ (collision voxels)
                                          /drilling/context ─────────────────┤ (target wall box + caution)
                                          /robot/target_point ───────────────┘
                                                                            │
                                                              /robot/motion_done
                                                              /robot/motion_failed
                                                              /task/drill_caution
                                                              project.log  (validation A[1-3], B[1-3])
```

## Launch

```bash
ros2 launch robot_gazebo robot_gazebo.launch.py                     # hazard-aware (default)
ros2 launch robot_gazebo robot_gazebo.launch.py hazard_aware:=false # OFF baseline
```

| Launch argument | Default | Description |
|---|---|---|
| `hazard_aware` | `true` | Forwarded to `robot_motion_planner` as the `hazard_aware` parameter. `false` disables the latent-hazard halt and omits the danger collision voxels — the OFF baseline for the safety comparison. Validation metrics are logged in both modes. |

The parameter can also be flipped between runs without relaunching:

```bash
ros2 param set /robot_gazebo/robot_motion_planner hazard_aware false
```

It is re-read on every `/robot/motion_ready`, so the next target uses the new value.

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

**Sequence (per drilling task):**

1. **Latent hazard check** — reads `nearby_elements` from `/drilling/context` and keeps the far-side (concealed, ORANGE) ones. For each, the depth from the robot-side wall face along `facing` is compared against `drill_depth_mm`; anything shallower than the drill depth counts as a conflict. If the geometry needed for that projection (`facing`, wall center, drill depth) is missing, every far-side element is conservatively treated as a conflict. The verdict is published on `/task/drill_caution`.
2. **Halt decision** — when `hazard_aware` is `true` and at least one conflict exists, the node stops before planning: it emits the validation block, publishes `/robot/motion_failed` with reason `caution: hidden element within drill depth`, and goes idle. With `hazard_aware:=false` it logs the warning and proceeds anyway (OFF baseline).
3. **Collision scene** — constructs up to two `CollisionObject` entries and pushes them via `/apply_planning_scene`:
   - `task_danger_zones`: one 3 cm box voxel per RED-category point from `/task/zones`. **Omitted entirely when `hazard_aware` is `false`.**
   - `target_wall`: a single box matching the wall's full bounding box from `/drilling/context`.

   If the service is unavailable or returns failure, planning continues without the scene.
4. **MoveGroup goal** — sends a pose goal to `/move_action` for `drill_tip` at each point in `/robot/target_point`, with a 5 mm position tolerance and ~3° per-axis orientation tolerance. The tip is pulled back `DRILL_STANDOFF = 5 mm` from the surface along the wall normal, and its local +z is aligned with that normal (pointing into the wall). Each goal allows 5 s of planning over 5 internal attempts at 0.5 velocity/acceleration scaling; up to `MAX_PLAN_ATTEMPTS = 3` goals are sent per point.
5. **Multi-point sequencing** — after each successful point the arm dwells for `INTER_POINT_DELAY = 5 s` before moving to the next. After the last point it dwells `READY_POSE_DELAY = 5 s`, then goes to the `ready` pose as a joint-space goal (the SRDF `ready` joint values, ±0.01 rad, at 0.3 scaling).
6. **Completion** — emits the validation block, then publishes `/robot/motion_done` on success or `/robot/motion_failed` (with reason JSON) once retries are exhausted. A failed or rejected ready-pose goal is logged but still reports done.

**Parameters**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `hazard_aware` | `bool` | `true` | Enables the latent-hazard halt (step 2) and the danger collision voxels (step 3). Re-read at the start of every task. |

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
| `/task/drill_caution` | `std_msgs/String` | Latched; `{"has_caution": bool, "conflicting_element_count": N}` — published once per task, before the halt decision |

**Services (client)**

| Service | Description |
|---|---|
| `/apply_planning_scene` | MoveIt service; used to push collision objects before each plan |

**Action (client)**

| Action | Description |
|---|---|
| `/move_action` | `moveit_msgs/MoveGroup`; used for both drill-pose goals and the final ready-pose goal |

**TF (listener)**

`world` → `drill_tip` is looked up after each executed point to measure the achieved pose against the commanded one.

## Validation output

At the end of every task — success, planning failure, or hazard halt — the node writes a block to the shared validation log via `robot_validation.ValidationLog` (stdout plus `src/robot_validation/output/project.log`, overridable with `GRAPH2ROBOT_VALIDATION_LOG`). The block is headed `=== Execution run: target <mep_id> | hazard_aware=ON|OFF ===` and contains six sections:

| Section | Metrics |
|---|---|
| A[1] Motion planning | Per drill point: planned yes/no, goal attempts used, MoveIt planning time. Then success count over total points and mean/max planning time. |
| A[2] Execution accuracy | Per drill point: achieved-vs-commanded position error (mm) and orientation error (deg) from the `drill_tip` TF, plus the achieved standoff from the robot-side wall face. Then mean/max over the task. |
| A[3] Sequence outcome | Whether all points were reached and whether the arm returned to `ready`. |
| B[1] Front-side collision avoidance | Count of front-side elements and the danger voxels built from them, plus min clearance from the commanded goals to the nearest voxel. Reports `not enforced` when `hazard_aware=OFF`. |
| B[2] Latent hazard halt | Number of concealed elements on the drilling path, target drill depth, depth of the nearest concealed element, and whether the halt fired. |
| B[3] Safety comparison | One side of the ON/OFF pair — the run records its own mode and points at the other run for the counterpart. |

Clearance in B[1] is a proxy: it measures the commanded drill-tip goal against the voxel boxes, not the swept trajectory.
