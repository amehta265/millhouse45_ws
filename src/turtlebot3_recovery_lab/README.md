# turtlebot3_recovery_lab (Lab 5)

Brian Huntley, Ankit Mehta (Millhouse - 45)

The instructor's package plus our changes. The package name has to stay `turtlebot3_recovery_lab`, because the params file and the behavior tree look it up by name.

| File | What it is |
|---|---|
| `config/nav2_params.yaml` | Our tuned Nav2 parameters (pass with `params_file:=`). |
| `turtlebot3_recovery_nodes/recovery_student.py` | Instructor skeleton with the TODOs filled in: beep, recovery waypoint 0.5 m behind the robot, beep again. |
| `turtlebot3_recovery_nodes/waypoint_navigator.py` | Collects the waypoints and sends them to Nav2 one after the other. |
| `CMakeLists.txt` | Installs `recovery_student.py` and `waypoint_navigator.py` (the original listed a `recovery.py` that does not exist). |
| `behavior_trees/`, `src/`, `include/`, `srv/` | Instructor files, unchanged. |

## Build (lab PC)

```bash
cd ~/millhouse45_ws
colcon build --symlink-install --packages-select turtlebot3_recovery_lab
source install/setup.bash
```

## Run

In every PC terminal: `source ~/millhouse45_ws/install/setup.bash`, `export TURTLEBOT3_MODEL=burger`, and the same `ROS_DOMAIN_ID` as the robot.

```bash
# 1. robot
ros2 launch turtlebot3_bringup robot.launch.py

# 2. PC: recovery service. Must start BEFORE Nav2, otherwise the behavior tree fails to load
ros2 run turtlebot3_recovery_lab recovery_student.py

# 3. PC: Nav2 with our params and the recovery behavior tree
ros2 launch nav2_bringup bringup_launch.py \
    map:=$HOME/maps/lab5_map_new.yaml \
    params_file:=$HOME/millhouse45_ws/src/turtlebot3_recovery_lab/config/nav2_params.yaml \
    use_sim_time:=False autostart:=True

# 4. PC: RViz, then set the start pose with "2D Pose Estimate"
ros2 launch nav2_bringup rviz_launch.py

# 5. PC: waypoints
ros2 run turtlebot3_recovery_lab waypoint_navigator.py
#    click the waypoints in RViz with "Publish Point", in order, then press Enter in this terminal.
#    Or type them:  waypoint_navigator.py --wp X1,Y1 --wp X2,Y2 --wp X3,Y3
```

After pressing Enter, untick LaserScan, Amcl Particle Swarm and Local Costmap in RViz. Every display gets its own copy of the robot's data over Wi-Fi, and the lab network is slow.

## Recovery behavior

When the path is blocked, the behavior tree clears the local costmap and calls `/divert_robot`. `recovery_student.py` plays the ERROR beep (sound 3), puts a recovery waypoint 0.5 m behind the robot (the way it came is the one stretch known to be clear), plays the success beep (sound 1) and returns the waypoint. The tree drives there and retries the original goal, up to 3 times.

## Main parameter changes

| Where | Change | Why |
|---|---|---|
| Controller | Regulated Pure Pursuit instead of DWB, 0.12 m/s, lookahead 0.25 m | DWB struggles in the maze; slow and tight path following |
| Global costmap | `robot_radius` 0.14 | Makes the narrow gap impassable for the planner |
| Global costmap | inflation 0.35 m, cost scaling 5.0 | Keeps paths in the middle of corridors |
| Global costmap | obstacle layer off | Planner uses the saved map only; late scans were smearing the walls |
| Local costmap | `robot_radius` 0.10 | Real robot size, so the collision check does not fire on nearby walls |
| Controller | collision look-ahead 0.5 s, `transform_tolerance` 3.0 | Fewer false alarms; tolerates 2–3 s position delays on the lab network |
| Progress checker | 0.10 m in 40 s | The robot is not declared stuck during network freezes |
| Planner | NavFn, Dijkstra, no planning through unknown space | A* failed to extract paths on this map |
