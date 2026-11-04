#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
Lab 4 - behavior_manager
"""

# Our brain: a small state machine that decides which behavior to follow

# States and which vector they send to lookahead_controller on /u_cmd:
#  GO_TO_GOAL      -> u_gtg        (go_to_goal.py)
#  FOLLOW_WALL_C   -> u_fw^c       (follow_wall.py, slide c around the obstacle)
#  FOLLOW_WALL_CC  -> u_fw^cc      (follow_wall.py, slide cc around the obstacle)
#  AVOID_OBSTACLE  -> u_AO         (avoid_obstacle.py)
#  AT_GOAL         -> zero         (stand still for 10 s on the waypoint, lab requirement)
#  DONE            -> zero         (all waypoints visited)

# Guards that define transitions from lecture slide:
# d = SAFE_DISTANCE,
# eps = EPSILON, the "fat guard" margin
# dist = ||x - x_O|| = distance to the closest obstacle point
# tau  = the moment we last switched INTO follow wall. We store ||x(tau) - x_G||, our distance to the goal at
#        that moment, as goal_dist_at_wall_start.

#   GO_TO_GOAL -> FOLLOW_WALL_C/CC   dist <= d  and  <u_GTG, u_FW> > 0 (pick the side that agrees with the goal)
#                                    and (our addition) <u_GTG, u_AO> <= 0, Set tau.
#   FOLLOW_WALL -> AVOID_OBSTACLE    dist < d - eps                (got too close while sliding along)
#   AVOID_OBSTACLE -> FOLLOW_WALL    dist > d  and  <u_GTG, u_FW> > 0 (pick the side again). Set tau.
#   FOLLOW_WALL -> GO_TO_GOAL        ( <u_GTG, u_AO> > 0  and  ||x(t) - x_G|| < ||x(tau) - x_G|| )
#                                    or  dist >= d + eps
#                                    "clear shot" = the goal is on the far side from the obstacle
#                                    "progress"   = we are closer to the goal than when we started following
#   any moving state -> AT_GOAL      ||x - x_G|| < GOAL_TOLERANCE
#   AT_GOAL -> GO_TO_GOAL / DONE     after STOP_TIME seconds, move on to the next waypoint (or finish)

#   subscribes: /global_pose, /obstacle, /u_gtg, /u_ao, /u_fw_c, /u_fw_cc
#   publishes:  /u_cmd (Vector3, the chosen vector), /current_goal (Point, for go_to_goal), /behavior_state (String)

import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import String

from millhouse45_navigate_to_goal.msg import Obstacle, RobotPose
from nav_utils import to_vector3, seconds_now, run_node

# Copied from wayPoints.txt (global frame, meters)
WAYPOINTS = [(1.5, 0.0), (1.5, 1.4), (0.0, 1.4)]

SAFE_DISTANCE = 0.30     # d (m), from the look-ahead point to the nearest obstacle point
EPSILON = 0.05           # eps (m), width of the fat guards
PROGRESS_MARGIN = 0.05   # how much closer to the goal (m) counts as "progress" when leaving follow wall
GOAL_TOLERANCE = 0.05    # we call a waypoint reached inside this radius (m). Lab allows 0.10 / 0.15 / 0.20
STOP_TIME = 10.5         # seconds to stand on each waypoint. Lab needs 10, a little extra to be safe

GO_TO_GOAL = 'GO_TO_GOAL'
FOLLOW_WALL_C = 'FOLLOW_WALL_C'
FOLLOW_WALL_CC = 'FOLLOW_WALL_CC'
AVOID_OBSTACLE = 'AVOID_OBSTACLE'
AT_GOAL = 'AT_GOAL'
DONE = 'DONE'
MOVING_STATES = (GO_TO_GOAL, FOLLOW_WALL_C, FOLLOW_WALL_CC, AVOID_OBSTACLE)


def next_behavior(state, obs_dist, goal_dist, goal_dist_at_wall_start, behavior_vectors):
    """The guards. Returns (new_state, new goal_dist_at_wall_start). Only handles the four moving states."""
    u_gtg = behavior_vectors[GO_TO_GOAL]
    u_ao = behavior_vectors[AVOID_OBSTACLE]

    def pick_wall_side():
        # follow the side whose u_FW agrees with u_GTG.
        # u_FW^c and u_FW^cc point in opposite directions, so (unless it is an exact tie) exactly one is > 0.
        if np.dot(u_gtg, behavior_vectors[FOLLOW_WALL_C]) > 0.0:
            return FOLLOW_WALL_C
        return FOLLOW_WALL_CC

    if state == GO_TO_GOAL:
        goal_blocked = np.dot(u_gtg, u_ao) <= 0.0
        if obs_dist <= SAFE_DISTANCE and goal_blocked:
            return pick_wall_side(), goal_dist          # set tau

    elif state in (FOLLOW_WALL_C, FOLLOW_WALL_CC):
        if obs_dist < SAFE_DISTANCE - EPSILON:
            return AVOID_OBSTACLE, goal_dist_at_wall_start
        clear_shot = np.dot(u_gtg, u_ao) > 0.0
        made_progress = goal_dist < goal_dist_at_wall_start - PROGRESS_MARGIN
        if (clear_shot and made_progress) or obs_dist >= SAFE_DISTANCE + EPSILON:
            return GO_TO_GOAL, goal_dist_at_wall_start

    elif state == AVOID_OBSTACLE:
        if obs_dist > SAFE_DISTANCE:
            return pick_wall_side(), goal_dist          # set tau

    return state, goal_dist_at_wall_start


class BehaviorManager(Node):
    def __init__(self):
        super().__init__('behavior_manager')

        self.state = GO_TO_GOAL
        self.goal_index = 0
        self.goal_dist_at_wall_start = math.inf   # ||x(tau) - x_G||
        self.arrived_time = None                  # when we reached the current waypoint
        self.start_time = None                    # when the run started (for the 2:30 limit)

        # Latest inputs. None until the first message arrives.
        self.pose = None
        self.obstacle = None
        # The vector each behavior node wants us to drive with, keyed by the state that uses it
        self.behavior_vectors = {GO_TO_GOAL: None, FOLLOW_WALL_C: None, FOLLOW_WALL_CC: None, AVOID_OBSTACLE: None}

        self.u_cmd_publisher = self.create_publisher(Vector3, '/u_cmd', 5)
        self.goal_publisher = self.create_publisher(Point, '/current_goal', 5)
        self.state_publisher = self.create_publisher(String, '/behavior_state', 5)

        self.create_subscription(RobotPose, '/global_pose', self.pose_callback, 5)
        self.create_subscription(Obstacle, '/obstacle', self.obstacle_callback, 5)
        self.create_subscription(Vector3, '/u_gtg', lambda m: self.save_vector(GO_TO_GOAL, m), 5)
        self.create_subscription(Vector3, '/u_ao', lambda m: self.save_vector(AVOID_OBSTACLE, m), 5)
        self.create_subscription(Vector3, '/u_fw_c', lambda m: self.save_vector(FOLLOW_WALL_C, m), 5)
        self.create_subscription(Vector3, '/u_fw_cc', lambda m: self.save_vector(FOLLOW_WALL_CC, m), 5)

        # 20 Hz. Odometry (and so u_GTG) updates faster than the lidar, so we run faster than the lecture's 0.08 s.
        self.create_timer(0.05, self.state_transition_callback)

    def pose_callback(self, msg):
        self.pose = msg

    def obstacle_callback(self, msg):
        self.obstacle = msg

    def save_vector(self, state, msg):
        self.behavior_vectors[state] = (msg.x, msg.y)

    def state_transition_callback(self):
        # go_to_goal needs the goal before it can publish anything, so this goes out first
        goal = WAYPOINTS[self.goal_index]
        self.goal_publisher.publish(Point(x=goal[0], y=goal[1], z=0.0))

        # Wait until every other node has sent us something at least once
        if self.pose is None or self.obstacle is None or None in self.behavior_vectors.values():
            return

        now = seconds_now(self)
        if self.start_time is None:
            self.start_time = now
        goal_dist = math.hypot(goal[0] - self.pose.x, goal[1] - self.pose.y)          # ||x - x_G||
        obs_dist = self.obstacle.distance if self.obstacle.found else math.inf        # ||x - x_O||

        # ---- Step 1: check the guards and update the state ----
        old_state = self.state
        if self.state in MOVING_STATES and goal_dist < GOAL_TOLERANCE:
            self.state = AT_GOAL
            self.arrived_time = now
        elif self.state in MOVING_STATES:
            self.state, self.goal_dist_at_wall_start = next_behavior(
                self.state, obs_dist, goal_dist, self.goal_dist_at_wall_start, self.behavior_vectors)
        elif self.state == AT_GOAL and now - self.arrived_time >= STOP_TIME:
            if self.goal_index == len(WAYPOINTS) - 1:
                self.state = DONE
            else:
                self.goal_index += 1
                self.state = GO_TO_GOAL

        if self.state != old_state:
            self.get_logger().info(f'{old_state} -> {self.state} | t={now - self.start_time:.1f}s, '
                                   f'waypoint {self.goal_index + 1}, goal {goal_dist:.2f} m, obstacle {obs_dist:.2f} m')

        # ---- Step 2: drive with the vector that belongs to the state (AT_GOAL / DONE -> stand still) ----
        chosen_vector = self.behavior_vectors.get(self.state, (0.0, 0.0))
        self.u_cmd_publisher.publish(to_vector3(*chosen_vector))
        self.state_publisher.publish(String(data=self.state))


if __name__ == '__main__':
    run_node(BehaviorManager)
