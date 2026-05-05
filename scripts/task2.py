#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import numpy as np
import math
from enum import Enum, auto


#  TUNING PARAMETERS

LINEAR_SPEED      = 0.26   # m/s forward
ARC_SPEED         = 0.12   # m/s forward while arcing around obstacle
TURN_SPEED        = 1.5    # rad/s on-spot turn
ARC_TURN_SPEED    = 1.3    # rad/s while arcing (moving + turning)
REVERSE_SPEED     = 0.11   # m/s reverse

DANGER_DIST       = 0.30   # m  emergency reverse
CAUTION_DIST      = 0.46   # m  start obstacle avoidance 
SIDE_DIST         = 0.30   # m  Braitenberg side threshold
STEER_GAIN        = 3.0

FRONT_DEG         = 30     # ±30° front arc
LEFT_DEG          = (60, 105)
RIGHT_DEG         = (275, 320)

# Waypoint navigation
WAYPOINT_TOLERANCE  = 0.45  
HEADING_GAIN        = 1.8
MAX_HEADING_STEER   = 1.5

# Arc avoidance (moving while turning)
ARC_MIN_DEG         = 80    # minimum arc angle
ARC_MAX_DEG         = 160   # maximum arc angle
REVERSE_SECS        = 0.6

# Stuck detection
STUCK_TIME          = 4.0   # seconds before declaring stuck
STUCK_DIST          = 0.15  # m  if moved less than this in STUCK_TIME

MAX_DT              = 0.3


WAYPOINTS = [
    (-1.5,  1.5),   # 0: corner TL  — zones 12, 1
    ( 0.0,  1.5),   # 1: top centre — zones 2, 3
    ( 1.5,  1.5),   # 2: corner TR  — zones 4, 5
    ( 1.5,  0.0),   # 3: right mid  — zone 5, 6
    ( 1.5, -1.5),   # 4: corner BR  — zones 6, 7
    ( 0.0, -1.5),   # 5: bot centre — zones 8, 9
    (-1.5, -1.5),   # 6: corner BL  — zones 10, 11
    (-1.5,  0.0),   # 7: left mid   — zones 11, 12
    (-1.5,  1.5),   # 8: TL again   — loop complete
]


class State(Enum):
    WAITING   = auto()
    NAVIGATE  = auto()   # heading toward waypoint
    ARCING    = auto()   # moving forward + turning to arc around obstacle
    REVERSING = auto()   # too close — back up


class Task2Explorer(Node):

    def __init__(self):
        super().__init__("task2_explorer")

        self.vel_pub  = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self._scan_cb, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_cb, 10
        )

        # LiDAR
        self.front_min = float("inf")
        self.left_min  = float("inf")
        self.right_min = float("inf")
        self.scan_ok   = False
        self.angle_inc = math.radians(1.0)

        # Odometry
        self.x       = 0.0
        self.y       = 0.0
        self.yaw     = 0.0
        self.odom_ok = False

        # Waypoint
        self.wp_index = 0

        # State
        self.state         = State.WAITING
        self.shutdown_flag = False
        self.last_time     = self.get_clock().now()

        # Arc avoidance
        self.arc_remaining = 0.0
        self.arc_dir       = 1    # +1 CCW, -1 CW

        # Reverse
        self.rev_elapsed = 0.0

        # Stuck detection
        self.stuck_timer    = 0.0
        self.stuck_ref_x    = 0.0
        self.stuck_ref_y    = 0.0
        self.stuck_check_dt = 0.0

        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            "Task 2 Team 09 v6 — perimeter sweep with arc avoidance"
        )

    #  Odometry callback 

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.odom_ok = True

    # LiDAR callback 

    def _scan_cb(self, msg: LaserScan):
        r = np.array(msg.ranges, dtype=float)
        n = len(r)
        self.angle_inc = msg.angle_increment

        valid = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        r = np.where(valid, r, 99.0)

        def deg_to_idx(deg: float) -> int:
            return int(round(math.radians(deg % 360) / self.angle_inc)) % n

        def arc_min(a_deg: float, b_deg: float) -> float:
            a, b = deg_to_idx(a_deg), deg_to_idx(b_deg)
            if a <= b:
                idx = np.arange(a, b + 1)
            else:
                idx = np.concatenate([np.arange(a, n), np.arange(0, b + 1)])
            return float(r[idx % n].min())

        self.front_min = arc_min(360 - FRONT_DEG, FRONT_DEG)
        self.left_min  = arc_min(LEFT_DEG[0],  LEFT_DEG[1])
        self.right_min = arc_min(RIGHT_DEG[0], RIGHT_DEG[1])

        if not self.scan_ok:
            self.scan_ok = True
            self.get_logger().info(
                f"LiDAR ready — F={self.front_min:.2f} "
                f"L={self.left_min:.2f} R={self.right_min:.2f}"
            )

    # Helpers 

    def _pub(self, v: float, w: float):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.twist.linear.x  = float(v)
        msg.twist.angular.z = float(w)
        self.vel_pub.publish(msg)

    def _stop(self):
        self._pub(0.0, 0.0)

    def _dt(self) -> float:
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        return min(max(dt, 0.0), MAX_DT)

    def _current_waypoint(self):
        return WAYPOINTS[self.wp_index % len(WAYPOINTS)]

    def _distance_to_waypoint(self) -> float:
        wx, wy = self._current_waypoint()
        return math.hypot(wx - self.x, wy - self.y)

    def _angle_to_waypoint(self) -> float:
        wx, wy = self._current_waypoint()
        target = math.atan2(wy - self.y, wx - self.x)
        diff   = target - self.yaw
        return math.atan2(math.sin(diff), math.cos(diff))

    def _start_arc(self):
        """Begin arcing around obstacle — move forward while turning."""
        # Arc toward whichever side has more clearance
        self.arc_dir = 1 if self.left_min >= self.right_min else -1
        self.arc_remaining = math.radians(
            ARC_MIN_DEG + (ARC_MAX_DEG - ARC_MIN_DEG) *
            max(0.0, 1.0 - self.front_min / CAUTION_DIST)
        )
        self.state = State.ARCING
        self.get_logger().info(
            f"Obstacle {self.front_min:.2f}m → arcing "
            f"{'L' if self.arc_dir > 0 else 'R'} "
            f"{math.degrees(self.arc_remaining):.0f}°"
        )

    def _check_stuck(self, dt: float):
        """
        Track whether robot has moved. If not moved STUCK_DIST in
        STUCK_TIME seconds → skip to next waypoint.
        """
        self.stuck_check_dt += dt
        if self.stuck_check_dt >= STUCK_TIME:
            moved = math.hypot(
                self.x - self.stuck_ref_x,
                self.y - self.stuck_ref_y
            )
            if moved < STUCK_DIST:
                self.get_logger().warn(
                    f"STUCK detected! Only moved {moved:.2f}m in "
                    f"{STUCK_TIME}s → skipping waypoint {self.wp_index}"
                )
                self.wp_index += 1
                self.state = State.NAVIGATE
                self.last_time = self.get_clock().now()

            # Reset stuck check
            self.stuck_ref_x    = self.x
            self.stuck_ref_y    = self.y
            self.stuck_check_dt = 0.0

    # Control loop 

    def _control_loop(self):
        if self.shutdown_flag:
            return

        if self.state == State.WAITING:
            self._pub(0.0, 0.0)
            if self.scan_ok and self.odom_ok:
                self.get_logger().info(
                    "All sensors ready — starting perimeter sweep!"
                )
                self.last_time   = self.get_clock().now()
                self.stuck_ref_x = self.x
                self.stuck_ref_y = self.y
                self.state = State.NAVIGATE
            return

        # Danger override
        if self.front_min < DANGER_DIST and self.state is not State.REVERSING:
            self.get_logger().warn(
                f"DANGER {self.front_min:.2f}m → reversing",
                throttle_duration_sec=1
            )
            self.rev_elapsed = 0.0
            self._stop()
            self.state = State.REVERSING

        {
            State.NAVIGATE : self._state_navigate,
            State.ARCING   : self._state_arcing,
            State.REVERSING: self._state_reversing,
        }[self.state]()

    #  NAVIGATE 

    def _state_navigate(self):
        dt = self._dt()
        self._check_stuck(dt)

        wx, wy = self._current_waypoint()
        dist   = self._distance_to_waypoint()
        angle  = self._angle_to_waypoint()

        # Reached waypoint
        if dist < WAYPOINT_TOLERANCE:
            self.wp_index += 1
            next_wp = self._current_waypoint()
            self.get_logger().info(
                f"✓ WP reached ({wx:.1f},{wy:.1f}) → "
                f"next WP{self.wp_index}: ({next_wp[0]:.1f},{next_wp[1]:.1f})"
            )
            # Reset stuck reference for new waypoint
            self.stuck_ref_x    = self.x
            self.stuck_ref_y    = self.y
            self.stuck_check_dt = 0.0
            return

        # Obstacle — arc around it (move forward while turning)
        if self.front_min < CAUTION_DIST:
            self._start_arc()
            return

        # Proportional heading correction toward waypoint
        heading_steer = HEADING_GAIN * angle
        heading_steer = max(-MAX_HEADING_STEER,
                            min(MAX_HEADING_STEER, heading_steer))

        # Braitenberg side correction
        l_threat   = max(0.0, SIDE_DIST - self.left_min)
        r_threat   = max(0.0, SIDE_DIST - self.right_min)
        side_steer = (r_threat - l_threat) * STEER_GAIN

        steer = heading_steer + side_steer

        # Slow for large heading corrections
        speed = LINEAR_SPEED * max(1.0 - abs(angle) / math.pi, 0.5)

        self._pub(speed, steer)

        self.get_logger().info(
            f"→ WP{self.wp_index} ({wx:.1f},{wy:.1f}) "
            f"dist={dist:.2f}m hdg={math.degrees(angle):.0f}° "
            f"F={self.front_min:.2f} L={self.left_min:.2f} R={self.right_min:.2f}",
            throttle_duration_sec=1
        )

    # ARCING 

    def _state_arcing(self):
        """
        Move forward AND turn simultaneously to arc around the obstacle.
        Much more effective than spinning on the spot — robot physically
        moves away from the wall while changing heading.
        """
        dt = self._dt()

        # If we're dangerously close while arcing — stop and reverse
        if self.front_min < DANGER_DIST:
            self._stop()
            self.rev_elapsed = 0.0
            self.state = State.REVERSING
            return

        self._pub(ARC_SPEED, ARC_TURN_SPEED * self.arc_dir)
        self.arc_remaining -= ARC_TURN_SPEED * dt

        # Arc complete AND front is clear → resume navigation
        if self.arc_remaining <= 0 and self.front_min > CAUTION_DIST:
            self.last_time = self.get_clock().now()
            self.state = State.NAVIGATE
            self.get_logger().info(
                "Arc complete — resuming waypoint navigation"
            )
        # Arc complete but still blocked → extend the arc
        elif self.arc_remaining <= 0 and self.front_min <= CAUTION_DIST:
            self.arc_remaining = math.radians(ARC_MIN_DEG)
            self.get_logger().info(
                f"Still blocked ({self.front_min:.2f}m) → extending arc"
            )

    #  REVERSING 
    def _state_reversing(self):
        dt = self._dt()
        self._pub(-REVERSE_SPEED, 0.0)
        self.rev_elapsed += dt

        if self.rev_elapsed >= REVERSE_SECS:
            self._stop()
            # After reversing, start arcing away
            self.arc_dir = 1 if self.left_min >= self.right_min else -1
            self.arc_remaining = math.radians(120)
            self.state = State.ARCING
            self.get_logger().info("Reverse done → arcing away")

    #  Shutdown 

    def on_shutdown(self):
        self.get_logger().info("Shutdown — stopping robot.")
        self._stop()
        self.shutdown_flag = True


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Task2Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"{node.get_name()} received Ctrl+C.")
    finally:
        node.on_shutdown()
        while not node.shutdown_flag:
            continue
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
