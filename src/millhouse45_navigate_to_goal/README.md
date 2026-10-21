# millhouse45_navigate_to_goal (Lab 4)

Brian Huntley, Ankit Mehta (Millhouse - 45)

## Nodes and topics

| Node | Listens to | Publishes | Job |
|---|---|---|---|
| `fixed_odom` | `/odom` | `/global_pose` | Rotation_Script.py logic: the start of the run becomes (0, 0) facing +x |
| `get_object_range` | `/scan` | `/obstacle` | filter the lidar, closest point, measured from the look-ahead point (robot frame) |
| `go_to_goal` | `/global_pose`, `/current_goal` | `/u_gtg` | `u_GTG = K(‖e‖) e`, `K = v0(1 - e^(-a‖e‖²))/‖e‖` |
| `avoid_obstacle` | `/global_pose`, `/obstacle` | `/u_ao` | `u_AO = -K(‖e‖) e`, `K = (1/‖e‖)(c/(‖e‖² + ε))` |
| `follow_wall` | `/u_ao` | `/u_fw_c`, `/u_fw_cc` | `u_FW = α R(∓π/2) u_AO` |
| `behavior_manager` | all of the above | `/u_cmd`, `/current_goal`, `/behavior_state` | state machine + waypoints + 10 s stops |
| `lookahead_controller` | `/u_cmd`, `/global_pose` | `/cmd_vel` | `[v, w] = diag(1, 1/l) R(-θ) u`, then the wheel speed limit |

`nav_utils.py` holds the shared pieces (rotations, `LOOKAHEAD_L`, `run_node`, ...).
All `u` vectors are in the global frame. Everything runs on the robot.

## State machine

```
GO_TO_GOAL --(dist <= d, goal behind obstacle, <u_GTG,u_FW> > 0)--> FOLLOW_WALL_C / _CC   [set tau]
FOLLOW_WALL --(dist < d - eps)--> AVOID_OBSTACLE
AVOID_OBSTACLE --(dist > d, <u_GTG,u_FW> > 0)--> FOLLOW_WALL_C / _CC                       [set tau]
FOLLOW_WALL --((<u_GTG,u_AO> > 0 and closer to goal than at tau) or dist >= d + eps)--> GO_TO_GOAL
any moving state --(within GOAL_TOLERANCE)--> AT_GOAL --(10.5 s)--> GO_TO_GOAL (next point) or DONE
```

Watch it live with `ros2 topic echo /behavior_state`. Every switch is also printed with the time.

## Run it

On the robot (after building the workspace there and sourcing it):

```bash
ros2 launch turtlebot3_bringup robot.launch.py          # terminal 1
ros2 launch millhouse45_navigate_to_goal navigate_to_goal.launch.py   # terminal 2
```

Put the robot on the floor at the start, facing +x, **before** launching (fixed_odom saves the first `/odom` reading).

## Tuning values

Hard-coded constants now (no ROS params). Change them, rebuild, relaunch.

| Constant | Where | Value | Effect |
|---|---|---|---|
| `SAFE_DISTANCE` | behavior_manager.py | 0.30 m | d. Bigger = wider berth around boxes, longer detours |
| `EPSILON` | behavior_manager.py | 0.05 m | fat guard width. Bigger = fewer state switches |
| `GOAL_TOLERANCE` | behavior_manager.py | 0.05 m | when a waypoint counts as reached |
| `WAYPOINTS` | behavior_manager.py | from wayPoints.txt | the course |
| `V0` | go_to_goal.py `gtg_vector` | 0.18 m/s | top GTG speed |
| `V_FW` | follow_wall.py `fw_vectors` | 0.12 m/s | speed while sliding around a box |
| `LOOKAHEAD_L` | nav_utils.py | 0.10 m | look-ahead distance (used by get_object_range and lookahead_controller) |
