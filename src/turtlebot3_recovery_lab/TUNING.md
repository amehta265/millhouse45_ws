# Lab 5 tuning notes (Millhouse - 45)

What we changed in `config/nav2_params.yaml`, what each parameter does, why we touched it, and what we expect to see.
Every changed line in the yaml is also tagged `LAB5`.

Nothing here has been run on a robot or in Gazebo yet. The values come from the Nav2 docs, the lecture slides and the maze photos.
Treat them as a first guess and expect to adjust 1 or 2 of them once you have the real map (section 6).

## 1. What the course file got wrong for our setup

The file was made for big spaces. Our maze is a few meters wide, and from the photos the corridors look like about 0.5 m (a guess, measure it on the map). Three things stood out:

- The inflation radius (how far "danger" spreads out from a wall) was 1.0 m in the local costmap and 0.55 m in the global one. In a 0.5 m corridor that means the whole corridor looks dangerous, so the robot has no sense of "middle of the corridor is best".
- The robot radius was 0.1 m. The gap the planner must NOT use is about as wide as the robot, so with 0.1 m the planner thinks it fits.
- `use_sim_time: True` sat in the global costmap and the recoveries server. On the real robot there is no simulated clock, so the global costmap would stall.

## 2. How the costmap numbers work together

The planner never drives the robot centre through a cell that is closer to a wall than `robot_radius`. Those cells are treated as solid. Beyond that, cost starts high and fades out until `inflation_radius`. How fast it fades is `cost_scaling_factor` (bigger = fades faster).

So:
- `robot_radius` decides what is allowed at all. It is the number that keeps the robot out of the narrow gap.
- `inflation_radius` and `cost_scaling_factor` decide how much the robot prefers the middle of a corridor.

## 3. Parameters we changed

### Costmaps (`global_costmap` and `local_costmap`, kept identical on purpose)

| Parameter | Was | Now | What it does | Why / what to expect |
|---|---|---|---|---|
| `robot_radius` | 0.1 | **0.16** | Size of the circle the planner assumes the robot is. | The real burger is about 0.11 m from its centre to its furthest corner. 0.16 adds about 5 cm of padding. A gap narrower than 0.32 m is now solid for the planner, so it will not route through the narrow gap. The corridors (about 0.5 m) stay open. **This is the number to adjust after mapping (section 6).** |
| `inflation_radius` | 1.0 local, 0.55 global | **0.40** both | How far from a wall cost is added. | Cost now only covers the area right next to walls. The robot can tell "centre of the corridor" from "hugging a wall". |
| `cost_scaling_factor` | 3.0 | **5.0** | How quickly that cost fades. | Steeper fade, so the path is pushed away from walls without making the whole corridor expensive. |
| `plugins` | had `voxel_layer` | **removed `voxel_layer`** | Which layers make up the costmap. | A 2D lidar already feeds the obstacle layer. The voxel layer marked the same obstacles a second time. The Nav2 tuning guide recommends the obstacle layer for 2D lidar. |
| `update_frequency` (global) | 1.0 | **2.0** | How often the costmap refreshes. | A person stepping into the path shows up in the global costmap within about half a second. The lidar does 5 scans per second, so the local costmap stays at 5. |
| `publish_frequency` (global / local) | 1.0 / 2.0 | **2.0 / 5.0** | How often it is sent out (RViz and the recovery node read it). | Smoother view, fresher data for the recovery node. |
| `use_sim_time` (global) | True | **False** | Which clock to use. | Real robot. When you run Gazebo, the launch argument `use_sim_time:=True` overrides it anyway. |

### Planner (`planner_server`, NavFn)

| Parameter | Was | Now | What it does | Why / what to expect |
|---|---|---|---|---|
| `use_astar` | false | **true** | Dijkstra or A* search. | The handout and slides describe A*. Same kind of path, searched faster. |
| `allow_unknown` | true | **false** | Whether paths may cross cells the map knows nothing about. | The map will have unknown patches behind walls and in corners. With this off, the planner stays in space we actually mapped, so it cannot sneak through a hole in the map. |
| `tolerance` | 0.5 | **0.3** | How far from the goal the path may end if the goal is not reachable. | With the bigger robot radius a goal very close to a wall sits in the solid band. 0.3 m lets the planner still end up next to it, without stopping half a meter short. |

A commented-out Smac 2D block sits under the planner in case NavFn behaves badly. The slides mention Smac 2D, the course file used NavFn, so we kept the one the course file ships with.

### Controller (`controller_server`)

We switched from DWB to Regulated Pure Pursuit. The handout says DWB does not work well in a maze, and pure pursuit is close to the look-ahead controller from Lab 4: it follows the planned path by aiming at a point a little ahead on it. The old DWB settings are at the bottom of the yaml.

| Parameter | Was | Now | What it does | Why / what to expect |
|---|---|---|---|---|
| `FollowPath.plugin` | DWB | **RegulatedPurePursuit** | Which controller follows the path. | Follows the planned path closely instead of inventing its own wiggles. |
| `desired_linear_vel` | (DWB max 0.22) | **0.18** | Top forward speed. | The burger tops out at 0.22. The margin gives cleaner turns in tight spots. |
| `lookahead_dist` | n/a | **0.35** | How far ahead on the path it aims. | Small enough not to cut the corners of 0.5 m corridors. Larger = smoother but cuts corners. Smaller = hugs the path but may wobble. |
| `regulated_linear_scaling_min_radius` / `_min_speed` | n/a | **0.5 / 0.12** | Slows down on tight curves, never below 0.12 m/s. | Gentle slow-down at corners. |
| `use_rotate_to_heading`, `rotate_to_heading_min_angle` | n/a | **true, 0.6 rad** | Turn in place first if the path starts more than about 35 degrees off. | The robot turns on the spot instead of swinging wide, which matters in a small maze. |
| `use_collision_detection` | n/a | **true** | Stops if the path ahead hits a wall in the local costmap. | **This is what turns "a person steps in front of the robot" into a failure within about 0.3 s, which starts the recovery behavior.** If you see it stopping for no reason near walls, set it to false and rely on the planner failing. |
| `controller_frequency` | 10 | **20** | How often a velocity command is computed. | Steadier steering. |
| `progress_checker.required_movement_radius` | 0.5 | **0.2** | The robot must move this far within `movement_time_allowance` (10 s) or the controller fails. | Notices a stuck robot sooner, and does not complain about slow turns in tight areas. |
| `general_goal_checker.xy_goal_tolerance` | 0.25 | **0.15** | How close counts as "arrived". | Ends up closer to the waypoint that the TAs will check. |
| `general_goal_checker.yaw_goal_tolerance` | 0.25 | **3.14** | How well the final heading must match. | Waypoints are only (x, y). With this, the robot does not spin on the spot at every waypoint. Set it back to 0.25 if the TAs also check the heading. |

### Localization (`amcl`)

| Parameter | Was | Now | What it does | Why / what to expect |
|---|---|---|---|---|
| `max_beams` | 60 | **120** | Lidar beams used per update. | Sharper pose estimate in a small room. Cheap for the lab PC. |
| `update_min_d` | 0.25 | **0.1** | How far the robot moves before AMCL updates. | Every 10 cm instead of every 25 cm, so the pose keeps up while the robot weaves through the maze. |
| `update_min_a` | 0.2 | **0.1** | How much it turns before an update. | Same reason, for turning. |

### Other fixes in the file

- `use_sim_time: True` -> `False` in `recoveries_server` too.
- Added `behavior_server`, `smoother_server` and `velocity_smoother` sections. Humble's bringup starts these nodes, and the course file did not have sections for them, so they would run on defaults. If your Nav2 does not start one of them, that section is ignored. The velocity limits match the burger (0.22 m/s, 1.0 rad/s).
- Added a comment next to `default_nav_to_pose_bt_xml`: if `bt_navigator` cannot find the behavior tree file, replace `$(find-pkg-share ...)` with the full path.

### Other fixes outside the yaml

- `CMakeLists.txt` listed `turtlebot3_recovery_nodes/recovery.py`, which does not exist. The build would fail. It now installs `recovery_student.py` and `waypoint_navigator.py`.
- `package.xml` got the extra run dependencies those two scripts use (`nav2_msgs`, `action_msgs`, `tf2_ros_py`).

## 4. The recovery behavior (`recovery_student.py`)

We only filled in the TODOs of the instructor's skeleton. What happens when the path is blocked:

1. Navigation fails (the controller cannot move, or the planner finds no path).
2. The behavior tree clears the local costmap and calls our service `/divert_robot`.
3. Our node plays the **ERROR beep** (sound 3).
4. It looks up where the robot is and puts the recovery waypoint **0.5 m behind it**, facing the same way. The path ahead is blocked, so the way the robot came is the safest place to go.
5. It plays the **success beep** (sound 1) and sends the waypoint back.
6. The tree drives to the waypoint, then retries the original goal, up to 3 times (`number_of_retries` in the tree file).

If the robot's pose is not available, the service answers "failed" and does not beep success.

## 5. The waypoint script (`waypoint_navigator.py`)

The handout says you cannot use the RViz GUI to give waypoints on the day. So the waypoints go on the command line (`--wp x,y`, once per waypoint). The script sends them one at a time to Nav2's `navigate_to_pose` action, waits for each one to finish, and prints reached or failed. Set the start pose in RViz with "2D Pose Estimate" before you run it.

## 6. After you have the map: the numbers to check

Open the map in RViz and click points with "Publish Point" while running `ros2 topic echo /clicked_point` to measure.

1. **Width of the narrow gap.** Set `robot_radius` (both costmaps) to about `gap_width / 2 + 0.02`. Example: gap 0.25 m gives 0.145, so 0.16 already blocks it. Gap 0.35 m means use about 0.195.
2. **Width of the narrowest corridor the robot must use.** It must be wider than `2 * robot_radius + 0.1`. If the robot refuses real corridors ("no valid path"), the radius is too big.

Then drive a few goals in RViz first. What to watch:

| You see | Likely cause | Try |
|---|---|---|
| Robot stops for no reason next to a wall | Collision check on the path ahead | `use_collision_detection: false`, or a smaller `robot_radius` |
| "No valid path" to a goal in an open area | `robot_radius` or `inflation_radius` too big | Lower `robot_radius` a little, as long as the gap stays blocked |
| Cuts corners and touches walls | Look-ahead too long | `lookahead_dist` down to 0.25 |
| Wobbles along straights | Look-ahead too short | `lookahead_dist` up to 0.45 |
| Pose jumps around, path jerks | AMCL lost track (walls moved) | Re-set the pose in RViz, check `max_beams` and `update_min_d` |
| Route goes through the gap anyway | `robot_radius` too small | Raise it |
