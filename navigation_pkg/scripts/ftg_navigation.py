#!/usr/bin/env python3
import rospy
import numpy as np
import math
from collections import deque
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class FollowTheGap:
   def __init__(self):
       rospy.init_node('barn_ftg_node')

       self.max_dist          = 4.5
       self.min_safe_dist     = 0.3
       self.min_gap_threshold = 0.8

       self.robot_width      = 0.43
       self.safety_margin    = 0.25
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

       # Adaptive latency compensation thresholds
       # Compensation activates only when environment is tight enough to benefit
       # from it. In open worlds, skipping compensation avoids unnecessary
       # bubble inflation and keeps valid gaps intact.
       self.latency_adaptive_enabled = True   # master switch (True = adaptive mode)
       self.latency_force_on = False          # debug/override: force ON regardless
       self.latency_force_off = False         # debug/override: force OFF regardless

       # Activation criteria (all must be true for compensation to activate)
       self.latency_close_dist_thresh = 1.8   # meters: closest obstacle must be < this
       self.latency_max_valid_gaps = 2        # valid_gaps must be <= this

       # Hysteresis: once activated, stay active for N frames before re-evaluating
       # Prevents flicker on/off at threshold boundaries
       self.latency_active_hold_frames = 5    # ~0.25s at 20Hz
       self._latency_active_counter = 0
       self._latency_last_state = False       # for logging transitions

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

   def should_apply_latency_compensation(self, closest_dist, num_valid_gaps):
       """
       Decide whether latency compensation should be applied this frame.

       Based on A/B test results (18 Apr 2026):
       - Hard worlds (tight, close obstacles): compensation halves collisions
       - Easy/mid worlds (open, far obstacles): compensation causes timeouts

       Activation criteria (all required):
       1. Closest obstacle within latency_close_dist_thresh
       2. Few valid gaps (<= latency_max_valid_gaps) indicating tight passage

       Hysteresis:
       Once activated, stay active for latency_active_hold_frames to avoid
       rapid on/off switching at the boundary.

       Returns True if compensation should be applied this frame.
       """
       # Debug overrides (for A/B and ablation tests)
       if self.latency_force_on:
           return True
       if self.latency_force_off:
           return False
       if not self.latency_adaptive_enabled:
           return False

       # Primary decision based on current scene
       is_tight_now = (
           closest_dist < self.latency_close_dist_thresh
           and num_valid_gaps <= self.latency_max_valid_gaps
       )

       # Apply hysteresis: once active, stay active for hold_frames
       if is_tight_now:
           self._latency_active_counter = self.latency_active_hold_frames
           active = True
       elif self._latency_active_counter > 0:
           self._latency_active_counter -= 1
           active = True   # coast on residual activation
       else:
           active = False

       # Log state transitions only (not every frame)
       if active != self._latency_last_state:
           rospy.logdebug(
               f"[FTG_LATENCY] state={'ON' if active else 'OFF'} "
               f"closest={closest_dist:.2f}m valid_gaps={num_valid_gaps}"
           )
           self._latency_last_state = active

       return active

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

   def navigate(self, data):
       robot_x, robot_y, robot_yaw = self.curr_pose

       dx = self.goal_world[0] - robot_x
       dy = self.goal_world[1] - robot_y
       goal_angle = math.atan2(
           math.sin(math.atan2(dy, dx) - robot_yaw),
           math.cos(math.atan2(dy, dx) - robot_yaw),
       )

       # Step 1: preprocess raw ranges once (shared by both paths)
       raw_ranges = self.preprocess_lidar(data.ranges)
       n          = len(raw_ranges)
       quarter    = n // 4

       # Step 2: quick preliminary analysis on RAW ranges to decide compensation
       raw_front = raw_ranges[quarter : n - quarter]
       raw_valid_mask = raw_front > 0.1
       if raw_valid_mask.any():
           raw_masked = np.where(raw_valid_mask, raw_front, np.inf)
           prelim_closest = float(raw_front[int(np.argmin(raw_masked))])
       else:
           prelim_closest = self.max_dist

       # Quick gap count on raw ranges (no bubble applied yet)
       # Count how many regions exceed min_gap_threshold as a rough proxy for
       # valid_gaps. This is an approximation but correlates well in practice.
       raw_non_zeros = np.where(raw_front > self.min_gap_threshold)[0]
       if len(raw_non_zeros) > 0:
           raw_gaps = np.split(raw_non_zeros, np.where(np.diff(raw_non_zeros) > 1)[0] + 1)
           prelim_gap_count = sum(
               1 for g in raw_gaps
               if self.get_gap_physical_width(g, raw_front, data.angle_increment) >= self.min_phys_gap
           )
       else:
           prelim_gap_count = 0

       # Step 3: decide whether to apply latency compensation
       apply_comp = self.should_apply_latency_compensation(prelim_closest, prelim_gap_count)

       # Step 4: produce final ranges based on decision
       if apply_comp:
           ranges = self.compensate_latency(raw_ranges, data.angle_increment)
       else:
           ranges = raw_ranges

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
       comp_str = "ON" if apply_comp else "off"
       rospy.loginfo(
           f"[FTG] v={self.current_v:.2f} goal={math.degrees(goal_angle):.1f}° "
           f"closest={closest_dist:.2f}m target={math.degrees(self.smooth_angle):.1f}° "
           f"gaps={len(gaps)} valid={len(valid_gaps)} dropped={dropped_gaps} "
           f"comp={comp_str} prelim_close={prelim_closest:.2f} prelim_gaps={prelim_gap_count}"
       )

       self.drive(self.smooth_angle, closest_dist, best_phys_w)

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