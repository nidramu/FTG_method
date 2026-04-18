#!/usr/bin/env python3
import rospy
import numpy as np
import math
from collections import deque
from enum import Enum
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RobotState(Enum):
    NORMAL          = 0   # Default: run FTG gap analysis
    HEADING_ALIGN   = 1   # Startup: rotate to goal direction first
    BACKTRACK       = 2   # Stuck with rear clear: drive backwards 0.3m
    RECOVERY_ROTATE = 3   # Stuck with rear blocked: rotate to max-clearance
    DEEP_RECOVERY   = 4   # Multi-stuck: aggressive rotation + FTG reset


class FollowTheGap:
   def __init__(self):
       rospy.init_node('barn_ftg_node')

       self.max_dist          = 4.5
       self.min_safe_dist     = 0.3
       self.min_gap_threshold = 0.8

       self.robot_width      = 0.43
       # Bumped 0.25 -> 0.30 after Faz 1 v4 collision analysis: 57% of
       # collisions at closest_dist < 0.15m (bumper scraping tight gaps).
       self.safety_margin    = 0.30
       self.min_phys_gap     = self.robot_width + self.safety_margin
       self.robot_half_width = self.robot_width / 2.0

       self.max_v = 1.8
       self.max_w = 2.0

       self.lookahead = 1.0

       self.init_pos = rospy.get_param('init_position', [-2, 3, 1.57])
       self.goal_rel = rospy.get_param('goal_position', [0, 10])
       self.goal_world = [
           self.init_pos[0] + self.goal_rel[0],
           self.init_pos[1] + self.goal_rel[1],
       ]

       self.smooth_angle = None
       self.smooth_alpha = 0.5
       self.prev_closest = None
       self.prev_gap_center_idx = None

       # Velocity state for latency compensation
       self.current_v = 0.0
       self.current_w = 0.0
       self.latency_dt = 0.08  # 80ms typical for sim; use 0.12-0.15 on physical i3

       # Latency compensation always applied (see navigate()). A/B testing
       # showed adaptive activation gave no measurable benefit over the noise
       # floor of Gazebo simulation variance.

       # ────────── FAZ 1: FSM + RECOVERY ──────────
       # Current state — start in HEADING_ALIGN for initial goal alignment
       self.state = RobotState.HEADING_ALIGN

       # HEADING_ALIGN: rotate in place until within tolerance of goal
       self.align_tolerance      = math.radians(20.0)  # ±20° considered aligned
       self.align_w              = 0.8                 # rad/s rotation speed
       self.align_min_frames     = 10                  # ~0.5s at 20Hz
       self._align_counter       = 0

       # Stuck detector: pose ring buffer
       # 30 poses @ 20Hz = 1.5s window. Max displacement < 0.3m AND |v| near zero = stuck.
       # Stuck window shortened from 1.5s -> 1.0s to catch faster stuck onsets
       self.stuck_window_frames  = 20                  # 1.0s at 20Hz
       self.stuck_dist_thresh    = 0.3                 # meters
       # velocity gate removed — position-only is more reliable indicator
       # (wall-scraping at 0.1-0.2 m/s was missed; false positives in
       #  legitimate slow turns were common)
       self.stuck_min_v          = 0.1                 # kept for compatibility but unused
       self._pose_history        = deque(maxlen=self.stuck_window_frames)

       # Breadcrumb trail: every 5th frame (0.25s), save pose. ~5s of history.
       self.breadcrumb_every_n   = 5
       self.breadcrumb_max_len   = 20
       self._breadcrumb_counter  = 0
       self._breadcrumbs         = deque(maxlen=self.breadcrumb_max_len)

       # BACKTRACK: reverse 0.3m straight
       self.backtrack_distance   = 0.3                 # meters to travel
       self.backtrack_v          = -0.4                # m/s (negative = reverse)
       self.backtrack_w          = 0.0                 # straight
       self._backtrack_start_pos = None                # (x,y) when backtrack began

       # RECOVERY_ROTATE: rotate toward max-clearance direction, 2s timeout
       self.rot_rec_w            = 0.8                 # rad/s
       self.rot_rec_timeout_s    = 2.0
       self._rot_rec_start_time  = None
       self._rot_rec_target_rel  = 0.0                 # target angle, robot frame

       # DEEP_RECOVERY: stronger rotation, longer timeout
       self.deep_rec_w           = 1.0                 # rad/s
       self.deep_rec_timeout_s   = 4.0
       self._deep_rec_start_time = None
       self._deep_rec_target_rel = 0.0

       # State-transition tracking
       self._stuck_count         = 0                   # consecutive stuck detections
       self._last_state_change_t = None

       # Multi-frame gap_passed filter (Bug #5 fix)
       # Prevents single-frame LiDAR noise spikes from triggering false positives
       # 3-frame ring buffer, requires 2-out-of-3 majority vote
       self.gap_passed_buffer = deque(maxlen=3)

       self.latest_scan = None
       self.curr_pose   = [self.init_pos[0], self.init_pos[1], self.init_pos[2]]

       # CSV diagnostics disabled: file I/O in control loop was throttling
       # navigate() below 20Hz and causing collision regression.

       self.pub      = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=1)
       self.sub_scan = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)
       self.sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)

   def scan_callback(self, msg):
       self.latest_scan = msg

   def odom_callback(self, msg):
       pos = msg.pose.pose.position
       q   = msg.pose.pose.orientation
       siny_cosp = 2 * (q.w * q.z + q.x * q.y)
       cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
       yaw = math.atan2(siny_cosp, cosy_cosp)
       self.curr_pose = [
           pos.x + self.init_pos[0],
           pos.y + self.init_pos[1],
           yaw,
       ]
       # Instantaneous velocities for latency compensation
       self.current_v = msg.twist.twist.linear.x
       self.current_w = msg.twist.twist.angular.z

   def compensate_latency(self, ranges, angle_increment):
       """
       Kinematic Lookahead with speed + distance attenuation.

       Previous version applied full v*dt shift unconditionally, which caused
       medium-speed regression: bubble inflation deleted valid gaps in
       worlds 100-199.

       Fix:
       1. Speed attenuation: projection scales with v_ratio = v/max_v, using
          sqrt for smooth ramp. Low speed => minimal shift (high position
          uncertainty), high speed => full shift (real latency impact).
       2. Distance attenuation: near obstacles get full compensation (collision
          risk), far obstacles get reduced compensation (low impact, avoids
          unnecessary bubble inflation). Exponential decay with tau=2.0m.
       """
       if abs(self.current_v) < 0.05:
           return ranges  # skip projection when nearly stopped

       # Speed attenuation: sqrt ramp between 0 and max_v
       v_ratio = min(abs(self.current_v) / self.max_v, 1.0)
       speed_factor = math.sqrt(v_ratio)  # smooth low-speed behavior

       effective_forward = self.current_v * self.latency_dt * speed_factor

       n = len(ranges)
       angles = np.arange(n) * angle_increment - (n * angle_increment) / 2.0

       xs = ranges * np.cos(angles)
       ys = ranges * np.sin(angles)

       # Distance attenuation: exp(-r/tau), tau=2.0m
       # At r=0: full compensation (factor=1.0)
       # At r=2m: factor ~= 0.37
       # At r=4m: factor ~= 0.14
       tau = 2.0
       dist_factor = np.exp(-ranges / tau)

       # Per-beam forward shift (scaled by distance factor)
       per_beam_shift = effective_forward * dist_factor

       xs_shifted = xs - per_beam_shift
       ys_shifted = ys

       ranges_comp = np.sqrt(xs_shifted ** 2 + ys_shifted ** 2)
       ranges_comp = np.where(np.isfinite(ranges_comp), ranges_comp, self.max_dist)
       return np.clip(ranges_comp, 0.0, self.max_dist)

   def preprocess_lidar(self, ranges):
       proc = np.array(ranges, dtype=float)
       proc[proc < self.min_safe_dist] = 0.0
       proc[np.isnan(proc)]            = 0.0
       proc[np.isinf(proc)]            = self.max_dist
       proc = np.clip(proc, 0.0, self.max_dist)
       return np.convolve(proc, np.ones(5) / 5, mode='same')

   def get_gap_physical_width(self, gap, front_ranges, angle_increment):
       if len(gap) < 2:
           return 0.0
       return float(np.mean(front_ranges[gap])) * len(gap) * angle_increment

   def _change_state(self, new_state, reason=""):
       """Centralized state transition with log line."""
       if self.state != new_state:
           rospy.loginfo(f"[FSM] {self.state.name} -> {new_state.name}  ({reason})")
           self.state = new_state
           self._last_state_change_t = rospy.Time.now()

   def _update_pose_history(self):
       """Record pose in stuck buffer and breadcrumb trail."""
       x, y, _ = self.curr_pose
       self._pose_history.append((x, y))

       self._breadcrumb_counter += 1
       if self._breadcrumb_counter >= self.breadcrumb_every_n:
           self._breadcrumb_counter = 0
           self._breadcrumbs.append((x, y, self.curr_pose[2]))

   def _is_stuck(self):
       """Detect if robot hasn't moved in stuck window AND velocity is near zero."""
       if len(self._pose_history) < self.stuck_window_frames:
           return False

       # Max displacement from oldest pose in window
       x0, y0 = self._pose_history[0]
       max_dist = 0.0
       for xi, yi in self._pose_history:
           d = math.hypot(xi - x0, yi - y0)
           if d > max_dist:
               max_dist = d

       # Position-only check: if robot hasn't moved 0.3m in the window, stuck.
       # Removed velocity gate — it created both false positives (slow
       # legitimate turns) and false negatives (wall-scraping at 0.1+ m/s,
       # oscillating in circles without progress).
       return (max_dist < self.stuck_dist_thresh)

   def _rear_clearance(self, data):
       """Min LiDAR distance in rear 90° cone. Returns nan if no valid readings."""
       ranges = np.array(data.ranges, dtype=float)
       ranges[np.isnan(ranges) | np.isinf(ranges)] = self.max_dist
       n = len(ranges)
       # Rear is center 90° of the back half
       rear_start = 3 * n // 8
       rear_end   = 5 * n // 8
       rear = ranges[rear_start:rear_end]
       valid = rear[rear > 0.1]
       if len(valid) == 0:
           return float('nan')
       return float(np.min(valid))

   def _find_max_clearance_angle(self, data):
       """Find angle (robot frame) with maximum LiDAR clearance.
       Used by RECOVERY_ROTATE/DEEP_RECOVERY to pick rotation direction."""
       ranges = self.preprocess_lidar(data.ranges)
       n = len(ranges)
       best_idx = int(np.argmax(ranges))
       angle = (best_idx - n / 2) * data.angle_increment
       return angle

   def _goal_angle(self):
       """Current angle to goal in robot frame, wrapped to [-pi, pi]."""
       x, y, yaw = self.curr_pose
       dx = self.goal_world[0] - x
       dy = self.goal_world[1] - y
       raw = math.atan2(dy, dx) - yaw
       return math.atan2(math.sin(raw), math.cos(raw))

   def navigate(self, data):
       """FSM dispatcher. Called every control cycle from run()."""
       self._update_pose_history()

       # Stuck check runs only in NORMAL state
       if self.state == RobotState.NORMAL and self._is_stuck():
           self._stuck_count += 1
           rear = self._rear_clearance(data)

           # Front clearance: center-half beams, ignore dropouts
           raw = self.preprocess_lidar(data.ranges)
           nn = len(raw); qq = nn // 4
           front_now = raw[qq : nn - qq]
           valid_now = front_now[front_now > 0.1]
           front_closest = float(np.min(valid_now)) if len(valid_now) > 0 else self.max_dist

           rear_ok  = (not math.isnan(rear) and rear > 0.5
                       and len(self._breadcrumbs) >= 2)
           imminent = (front_closest < 0.25)

           if rear_ok:
               # Backtrack preferred when rear is clear and we have breadcrumbs
               self._backtrack_start_pos = (self.curr_pose[0], self.curr_pose[1])
               self._change_state(RobotState.BACKTRACK,
                                  f"stuck #{self._stuck_count}, rear={rear:.2f}m front={front_closest:.2f}m")
           elif imminent:
               # Rear blocked AND front imminent: skip rotate, go DEEP_RECOVERY
               self._deep_rec_start_time = rospy.Time.now()
               self._deep_rec_target_rel = self._find_max_clearance_angle(data)
               self._change_state(RobotState.DEEP_RECOVERY,
                                  f"stuck #{self._stuck_count}, imminent front={front_closest:.2f}m")
           else:
               # Rotate in place when backtrack unsafe but front has room
               self._rot_rec_start_time = rospy.Time.now()
               self._rot_rec_target_rel = self._find_max_clearance_angle(data)
               self._change_state(RobotState.RECOVERY_ROTATE,
                                  f"stuck #{self._stuck_count}, rear blocked front={front_closest:.2f}m")

       # Dispatch based on current state
       if   self.state == RobotState.HEADING_ALIGN:   self._handle_heading_align(data)
       elif self.state == RobotState.BACKTRACK:       self._handle_backtrack(data)
       elif self.state == RobotState.RECOVERY_ROTATE: self._handle_recovery_rotate(data)
       elif self.state == RobotState.DEEP_RECOVERY:   self._handle_deep_recovery(data)
       else:                                           self._run_ftg(data)

   def _handle_heading_align(self, data):
       """Rotate in place until aligned with goal within tolerance, then NORMAL.

       Fast path: if already aligned on first frame (robot spawned facing goal),
       immediately transition to NORMAL with zero rotation perturbation.
       Without this, the ~0.5s initial spin causes trajectory drift that
       contaminates smoke-test comparisons.
       """
       goal_angle = self._goal_angle()

       if abs(goal_angle) < self.align_tolerance:
           # Fast path on first aligned frame: skip the 10-frame hold entirely
           # when we're already aligned at startup. No perturbation.
           if self._align_counter == 0:
               self._change_state(RobotState.NORMAL,
                                  f"already aligned at startup, goal_angle={math.degrees(goal_angle):.1f}°")
               self.stop_robot()
               return

           self._align_counter += 1
           if self._align_counter >= self.align_min_frames:
               self._change_state(RobotState.NORMAL,
                                  f"aligned after rotation, goal_angle={math.degrees(goal_angle):.1f}°")
               self.stop_robot()
               return
       else:
           self._align_counter = 0

       msg = Twist()
       msg.linear.x  = 0.0
       msg.angular.z = float(np.clip(math.copysign(self.align_w, goal_angle),
                                     -self.max_w, self.max_w))
       self.pub.publish(msg)

   def _handle_backtrack(self, data):
       """Reverse until backtrack_distance traveled or rear becomes blocked."""
       sx, sy = self._backtrack_start_pos
       cx, cy = self.curr_pose[0], self.curr_pose[1]
       traveled = math.hypot(cx - sx, cy - sy)
       rear = self._rear_clearance(data)

       if traveled >= self.backtrack_distance:
           self._change_state(RobotState.NORMAL, f"backtrack done, {traveled:.2f}m")
           self._pose_history.clear()  # reset stuck detector
           self.stop_robot()
           return

       if not math.isnan(rear) and rear < 0.3:
           # Rear became blocked mid-backtrack: switch to rotate
           self._rot_rec_start_time = rospy.Time.now()
           self._rot_rec_target_rel = self._find_max_clearance_angle(data)
           self._change_state(RobotState.RECOVERY_ROTATE,
                              f"rear blocked during backtrack, rear={rear:.2f}m")
           return

       msg = Twist()
       msg.linear.x  = self.backtrack_v
       msg.angular.z = self.backtrack_w
       self.pub.publish(msg)

   def _handle_recovery_rotate(self, data):
       """Rotate toward max-clearance until timeout; on timeout escalate or retry."""
       if self._rot_rec_start_time is None:
           self._rot_rec_start_time = rospy.Time.now()
           self._rot_rec_target_rel = self._find_max_clearance_angle(data)

       elapsed = (rospy.Time.now() - self._rot_rec_start_time).to_sec()

       if elapsed > self.rot_rec_timeout_s:
           if self._stuck_count >= 2:
               # Repeated stuck: escalate to DEEP_RECOVERY
               self._deep_rec_start_time = rospy.Time.now()
               self._deep_rec_target_rel = self._find_max_clearance_angle(data)
               self._change_state(RobotState.DEEP_RECOVERY,
                                  f"rot timeout, stuck_count={self._stuck_count}")
           else:
               self._change_state(RobotState.NORMAL, "rot timeout, retry FTG")
               self._pose_history.clear()
           self._rot_rec_start_time = None
           self.stop_robot()
           return

       msg = Twist()
       msg.linear.x  = 0.0
       msg.angular.z = float(np.clip(math.copysign(self.rot_rec_w, self._rot_rec_target_rel),
                                     -self.max_w, self.max_w))
       self.pub.publish(msg)

   def _handle_deep_recovery(self, data):
       """Stronger rotation, longer timeout, full state reset on exit."""
       if self._deep_rec_start_time is None:
           self._deep_rec_start_time = rospy.Time.now()
           self._deep_rec_target_rel = self._find_max_clearance_angle(data)

       elapsed = (rospy.Time.now() - self._deep_rec_start_time).to_sec()

       if elapsed > self.deep_rec_timeout_s:
           self._change_state(RobotState.NORMAL, "deep recovery timeout, full reset")
           # Full reset: give FTG a truly fresh start
           self._stuck_count = 0
           self._pose_history.clear()
           self._breadcrumbs.clear()
           self._deep_rec_start_time = None
           self.stop_robot()
           return

       msg = Twist()
       msg.linear.x  = 0.0
       msg.angular.z = float(np.clip(math.copysign(self.deep_rec_w, self._deep_rec_target_rel),
                                     -self.max_w, self.max_w))
       self.pub.publish(msg)

   def _run_ftg(self, data):
       robot_x, robot_y, robot_yaw = self.curr_pose

       dx = self.goal_world[0] - robot_x
       dy = self.goal_world[1] - robot_y
       goal_angle = math.atan2(
           math.sin(math.atan2(dy, dx) - robot_yaw),
           math.cos(math.atan2(dy, dx) - robot_yaw),
       )

       # Preprocess and apply latency compensation unconditionally
       ranges       = self.preprocess_lidar(data.ranges)
       ranges       = self.compensate_latency(ranges, data.angle_increment)
       n            = len(ranges)
       quarter      = n // 4
       front_ranges = ranges[quarter : n - quarter].copy()

       valid_mask = front_ranges > 0.1
       if not valid_mask.any():
           self.stop_robot()
           return

       masked       = np.where(valid_mask, front_ranges, np.inf)
       closest_idx  = int(np.argmin(masked))
       closest_dist = front_ranges[closest_idx]

       # Bug #5 fix: single-frame gap_passed was triggering false positives
       # on LiDAR noise spikes. Now uses 3-frame majority voting.
       single_frame_passed = (
           self.prev_closest is not None and
           closest_dist > self.prev_closest + 0.8 and
           closest_dist > 1.5
       )
       self.gap_passed_buffer.append(single_frame_passed)
       self.prev_closest = closest_dist

       # Require at least 2 of last 3 frames to agree before declaring passed
       gap_passed = (
           len(self.gap_passed_buffer) == self.gap_passed_buffer.maxlen
           and sum(self.gap_passed_buffer) >= 2
       )

       if gap_passed:
           self.smooth_angle = goal_angle
           self.drive(goal_angle, closest_dist, self.min_phys_gap * 2)
           rospy.logdebug("[FTG] Gap cleared, heading to goal")
           return

       front_mid = len(front_ranges) // 2
       front_min = float(np.min(front_ranges[front_mid//2 : front_mid + front_mid//2]))
       if front_min >= 3.5:
           self.smooth_angle = None
           self.drive(goal_angle, closest_dist, self.min_phys_gap * 2)
           rospy.logdebug(f"[FTG] Open field, angle={math.degrees(goal_angle):.1f}°")
           return

       bubble_r = max(int(math.atan2(self.robot_half_width + self.safety_margin,
                                          max(closest_dist, 0.2)) / data.angle_increment), 5)
       if abs(closest_idx - len(front_ranges) // 2) > len(front_ranges) * 0.35:
           bubble_r = bubble_r // 2
       start_b = max(0,                 closest_idx - bubble_r)
       end_b   = min(len(front_ranges), closest_idx + bubble_r)
       front_ranges[start_b:end_b] = 0.0

       non_zeros = np.where(front_ranges > self.min_gap_threshold)[0]
       if len(non_zeros) == 0:
           self.stop_robot()
           return
       gaps = np.split(non_zeros, np.where(np.diff(non_zeros) > 1)[0] + 1)

       goal_idx = float(np.clip(
           (goal_angle / data.angle_increment) + len(front_ranges) / 2,
           0, len(front_ranges) - 1
       ))

       valid_gaps = [
           (g, self.get_gap_physical_width(g, front_ranges, data.angle_increment))
           for g in gaps
           if self.get_gap_physical_width(g, front_ranges, data.angle_increment) >= self.min_phys_gap
       ]
       if not valid_gaps:
           fallback = [(g, self.get_gap_physical_width(g, front_ranges, data.angle_increment))
                       for g in gaps if len(g) > 2]
           if not fallback:
               self.stop_robot()
               return
           valid_gaps = [max(fallback, key=lambda x: x[1])]
           rospy.logwarn(f"[FTG] Fallback gap: {valid_gaps[0][1]:.2f}m")

       max_goal_dist = len(front_ranges) - 1
       max_tightness = 1.0

       best_gap, best_score, best_phys_w = None, float('inf'), self.min_phys_gap
       for gap, phys_w in valid_gaps:
           gap_center   = (gap[0] + gap[-1]) / 2.0
           gap_avg_dist = float(np.mean(front_ranges[gap]))
           goal_norm      = abs(gap_center - goal_idx) / max(max_goal_dist, 1)
           tightness_norm = (self.min_phys_gap / phys_w) / max(max_tightness, 1e-6)
           score = 0.85 * goal_norm + 0.15 * tightness_norm
           if gap_avg_dist < 1.5:
               score += (1.5 - gap_avg_dist) / 1.5 * 0.5
           if (self.prev_gap_center_idx is not None
                   and abs(gap_center - self.prev_gap_center_idx) < 15):
               score *= 0.9
           if score < best_score:
               best_score, best_gap, best_phys_w = score, gap, phys_w

       if best_gap is None:
           self.stop_robot()
           return

       gap_center_idx = int((best_gap[0] + best_gap[-1]) / 2)
       self.prev_gap_center_idx = gap_center_idx
       gap_angle      = (gap_center_idx - len(front_ranges) / 2) * data.angle_increment
       d_left         = float(front_ranges[best_gap[0]])
       d_right        = float(front_ranges[best_gap[-1]])
       gap_dist       = max(float(front_ranges[gap_center_idx])
                            if gap_center_idx < len(front_ranges) else 1.0, 0.5)
       fp_offset      = math.atan2(self.robot_half_width, gap_dist)

       if d_left < d_right - 0.1:
           target_angle = gap_angle + fp_offset
       elif d_right < d_left - 0.1:
           target_angle = gap_angle - fp_offset
       else:
           target_angle = gap_angle

       if self.smooth_angle is None:
           self.smooth_angle = target_angle
       else:
           diff = abs(target_angle - self.smooth_angle)
           if diff > math.pi / 2:
               self.smooth_angle = target_angle
           else:
               self.smooth_angle = (self.smooth_alpha * target_angle
                                    + (1.0 - self.smooth_alpha) * self.smooth_angle)

       dropped_gaps = len(gaps) - len(valid_gaps)
       rospy.loginfo(
           f"[FTG] v={self.current_v:.2f} goal={math.degrees(goal_angle):.1f}° "
           f"closest={closest_dist:.2f}m target={math.degrees(self.smooth_angle):.1f}° "
           f"gaps={len(gaps)} valid={len(valid_gaps)} dropped={dropped_gaps}"
       )

       self.drive(self.smooth_angle, closest_dist, best_phys_w)

       # If FTG is driving successfully, clear stuck counter
       if abs(self.current_v) > 0.3:
           self._stuck_count = 0

   def drive(self, angle, closest_dist, gap_phys_w):
       msg = Twist()

       lateral       = self.lookahead * math.sin(angle)
       curvature     = 2.0 * lateral / (self.lookahead ** 2)
       msg.angular.z = float(np.clip(curvature * self.max_v, -self.max_w, self.max_w))

       base_v = float(np.interp(closest_dist,
                                [0.3, 0.8, 1.5, 3.0],
                                [0.35, 0.7, 1.2, self.max_v]))

       base_v *= float(np.interp(abs(angle), [0.3, 0.8, 1.2], [1.0, 0.7, 0.4]))

       tightness  = self.min_phys_gap / max(gap_phys_w, self.min_phys_gap)
       tight_factor = 1.0 - 0.4 * tightness ** 1.5
       base_v     = base_v * tight_factor

       msg.linear.x = float(base_v)
       self.pub.publish(msg)

   def stop_robot(self):
       self.pub.publish(Twist())

   def run(self):
       rate = rospy.Rate(20)

       while not rospy.is_shutdown():
           if self.latest_scan is None:
               rate.sleep()
               continue

           robot_x, robot_y, _ = self.curr_pose
           dist_to_goal = math.hypot(
               self.goal_world[0] - robot_x,
               self.goal_world[1] - robot_y,
           )
           if dist_to_goal < 1.0:
               rospy.loginfo(f"[FTG] Goal reached! dist={dist_to_goal:.2f}m")
               self.stop_robot()
               break

           self.navigate(self.latest_scan)
           rate.sleep()


if __name__ == '__main__':
   try:
       ftg = FollowTheGap()
       ftg.run()
   except rospy.ROSInterruptException:
       pass