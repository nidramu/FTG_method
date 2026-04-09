#!/usr/bin/env python3
import rospy
import numpy as np
import math
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
        self.max_w = 1.5

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

        self.prev_gap_center         = None
        self.gap_switch_score_margin = 0.15
        self.gap_index_tolerance     = 8

        self.latest_scan = None
        self.curr_pose   = [self.init_pos[0], self.init_pos[1], self.init_pos[2]]

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
        dist_to_goal = math.hypot(dx, dy)
        goal_angle = math.atan2(
            math.sin(math.atan2(dy, dx) - robot_yaw),
            math.cos(math.atan2(dy, dx) - robot_yaw),
        )

        ranges       = self.preprocess_lidar(data.ranges)
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

        self.prev_closest = closest_dist

        n_front      = len(front_ranges)
        goal_idx_raw = (goal_angle / data.angle_increment) + n_front / 2
        goal_check_c = int(np.clip(goal_idx_raw, 0, n_front - 1))
        goal_check_s = max(0,       goal_check_c - 20)
        goal_check_e = min(n_front, goal_check_c + 20)
        goal_dir_min = float(np.min(front_ranges[goal_check_s:goal_check_e]))

        if goal_dir_min >= 1.5 and closest_dist >= 1.0 and dist_to_goal > 2.0:
            self.smooth_angle    = None
            self.prev_gap_center = None
            self.drive(goal_angle, closest_dist, self.min_phys_gap * 2)
            rospy.logdebug(f"[FTG] Goal clear, dist={dist_to_goal:.2f}m angle={math.degrees(goal_angle):.1f}°")
            return

        bubble_r = int(np.clip(40 * (1.0 / max(closest_dist, 0.4)), 5, 40))
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

        scored_gaps = []
        for gap, phys_w in valid_gaps:
            gap_center   = (gap[0] + gap[-1]) / 2.0
            gap_avg_dist = float(np.mean(front_ranges[gap]))
            score = (0.85 * abs(gap_center - goal_idx)
                     + 0.15 * (self.min_phys_gap / phys_w) * len(front_ranges))
            if gap_avg_dist < 1.5:
                score += (1.5 - gap_avg_dist) * 50
            scored_gaps.append((score, gap, phys_w))

        if not scored_gaps:
            self.stop_robot()
            return

        scored_gaps.sort(key=lambda x: x[0])
        best_score, best_gap, best_phys_w = scored_gaps[0]

        if self.prev_gap_center is not None:
            for score, gap, phys_w in scored_gaps:
                gap_center = (gap[0] + gap[-1]) / 2.0
                if abs(gap_center - self.prev_gap_center) <= self.gap_index_tolerance:
                    if score <= best_score + self.gap_switch_score_margin:
                        best_gap    = gap
                        best_score  = score
                        best_phys_w = phys_w
                    break

        self.prev_gap_center = float((best_gap[0] + best_gap[-1]) / 2.0)

        if best_gap is None:
            self.stop_robot()
            return

        gap_center_idx = int((best_gap[0] + best_gap[-1]) / 2)
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

        rospy.loginfo(
            f"[FTG] goal={math.degrees(goal_angle):.1f}° dist={dist_to_goal:.2f}m "
            f"closest={closest_dist:.2f}m target={math.degrees(self.smooth_angle):.1f}° "
            f"gaps={len(gaps)} valid={len(valid_gaps)}"
        )

        self.drive(self.smooth_angle, closest_dist, best_phys_w)

    def drive(self, angle, closest_dist, gap_phys_w):
        msg = Twist()

        lateral       = self.lookahead * math.sin(angle)
        curvature     = 2.0 * lateral / (self.lookahead ** 2)
        msg.angular.z = float(np.clip(curvature * self.max_v, -self.max_w, self.max_w))

        if closest_dist < 2.0:
            base_v = float(np.interp(closest_dist, [0.4, 2.0], [0.35, self.max_v]))
        else:
            base_v = self.max_v

        if abs(angle) > 0.4:
            base_v = min(base_v, 0.5)

        tightness  = self.min_phys_gap / max(gap_phys_w, self.min_phys_gap)
        base_v     = min(base_v, self.max_v * (1.0 - 0.5 * tightness))

        msg.linear.x = float(base_v)
        self.pub.publish(msg)

    def stop_robot(self):
        self.pub.publish(Twist())

    def run(self):
        rate = rospy.Rate(10)

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