# turtlebot3_recovery_lab (Lab 5)

Brian Huntley, Ankit Mehta (Millhouse - 45)

The instructor's package plus our changes. The package name has to stay `turtlebot3_recovery_lab`, because the params file and the behavior tree look it up by name.

| File | What it is |
|---|---|
| `config/nav2_params.yaml` | Tuned Nav2 parameters. Every change is marked `LAB5`. The full list with reasons is in `TUNING.md`. |
| `turtlebot3_recovery_nodes/recovery_student.py` | Instructor skeleton with the TODOs filled in: beep, waypoint 0.5 m behind the robot, beep again. |
| `turtlebot3_recovery_nodes/waypoint_navigator.py` | Sends the `--wp` waypoints to Nav2 in order. |
| `behavior_trees/navigate_w_recovery_divert.xml`, `src/`, `include/`, `srv/` | Instructor files, unchanged. |

## Build

```bash
cd ~/millhouse45_ws
colcon build --symlink-install --packages-select turtlebot3_recovery_lab
source install/setup.bash
```

`--symlink-install` means edits to the yaml and the Python files take effect without rebuilding.

## Run (lab day)

```bash
# 1. robot
ros2 launch turtlebot3_bringup robot.launch.py

# 2. PC: Nav2 with our params and the recovery behavior tree
ros2 launch nav2_bringup bringup_launch.py \
    map:=$HOME/maps/lab5_map.yaml \
    params_file:=$HOME/millhouse45_ws/src/turtlebot3_recovery_lab/config/nav2_params.yaml \
    use_sim_time:=False autostart:=True

# 3. PC: RViz, then set the start pose with "2D Pose Estimate"
ros2 launch nav2_bringup rviz_launch.py

# 4. PC: recovery service (must be running before the robot gets stuck)
ros2 run turtlebot3_recovery_lab recovery_student.py

# 5. PC: the waypoints the instructor gave you, in order (only after step 3)
ros2 run turtlebot3_recovery_lab waypoint_navigator.py --wp X1,Y1 --wp X2,Y2 --wp X3,Y3
```

## Notes

- Check the beeps on the robot first: `ros2 service call /sound turtlebot3_msgs/srv/Sound "{value: 3}"`, then `{value: 1}`.
- If `bt_navigator` cannot find the behavior tree file, replace the `$(find-pkg-share ...)` path in `nav2_params.yaml` with the full path to `behavior_trees/navigate_w_recovery_divert.xml`.
- Gazebo: same steps with `use_sim_time:=True` and no step 1. There is no `/sound` there, so the beeps only log a warning.
