#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from com2009_team09_2026_modules.tb3_tools import quaternion_to_euler

import numpy as np
import random
from math import floor, degrees, atan2, pi, sqrt
import time

FRONT_ARC = list(range(0, 26)) + list(range(335, 360))
LEFT_ARC  = list(range(26, 91))
RIGHT_ARC = list(range(270, 335))
BACK_ARC  = list(range(150, 211))

MIN_VALID_RANGE = 0.12

FORWARD_SPEED      = 0.26
SLOW_SPEED         = 0.08
TURN_SPEED         = 0.9
RECOVER_TURN_SPEED = 1.2
BACKUP_SPEED       = -0.15

FRONT_CLEAR_THRESHOLD      = 0.43
FRONT_VERY_CLOSE_THRESHOLD = 0.30
SIDE_CLOSE_THRESHOLD       = 0.18
BACK_CLEAR_THRESHOLD       = 0.25

ZONE_SIZE  = 1.0
TOTAL_TIME = 179.0


class Explorer(Node):

    def __init__(self):
        super().__init__("task3_explorer")

        self.shutdown   = False
        self.have_odom  = False
        self.have_scan  = False
        self.have_map   = False
        self.start_time = None

        self.vel_msg = TwistStamped()

        self.x = self.y = self.theta_z = 0.0
        self.x0 = self.y0 = 0.0

        self.map_data     = None
        self.map_width    = self.map_height = 0
        self.map_res      = 0.05
        self.map_origin_x = self.map_origin_y = 0.0

        self.frontier_angle = None
        self.frontier_timer = 0

        self.visited_zones = set()
        self.start_zone    = None

        self.front_distance = 999.0
        self.left_distance  = 999.0
        self.right_distance = 999.0
        self.back_distance  = 999.0

        self.prev_x               = 0.0
        self.prev_y               = 0.0
        self.progress_timer_count = 0
        self.no_progress_count    = 0

        self.state             = "EXPLORE"
        self.blocked_counter   = 0
        self.last_turn_left    = True
        self.recover_counter   = 0
        self.recover_direction = 1
        self.total_recoveries  = 0
        self.backup_counter    = 0

        self.vel_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)

        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10)

        self.control_timer = self.create_timer(0.1, self.timer_callback)
        self.log_timer     = self.create_timer(1.0, self.log_callback)

        self.get_logger().info("Task 3: Explorer started.")

    def set_cmd(self, linear_x, angular_z):
        self.vel_msg.twist.linear.x  = linear_x
        self.vel_msg.twist.angular.z = angular_z
        self.vel_pub.publish(self.vel_msg)

    def filter_ranges(self, ranges, indices):
        vals = []
        for i in indices:
            if i >= len(ranges):
                continue
            r = ranges[i]
            if r > MIN_VALID_RANGE and not np.isinf(r) and not np.isnan(r):
                vals.append(r)
        return vals if vals else [float("inf")]

    def _current_zone(self):
        return (floor((self.x - self.x0) / ZONE_SIZE),
                floor((self.y - self.y0) / ZONE_SIZE))

    def _update_zones(self):
        zone = self._current_zone()
        if zone not in self.visited_zones:
            self.visited_zones.add(zone)
            self.get_logger().info(
                f"ZONE ENTERED: {zone} | total: {len(self.visited_zones)}")

    def _update_frontier_bias(self):
        self.frontier_timer += 1
        if self.frontier_timer < 20:
            return
        self.frontier_timer = 0

        if self.map_data is None:
            return

        grid = np.array(self.map_data).reshape(
            (self.map_height, self.map_width))

        rx = int((self.x - self.map_origin_x) / self.map_res)
        ry = int((self.y - self.map_origin_y) / self.map_res)

        frontiers = []
        step = 4
        for gy in range(step, self.map_height - step, step):
            for gx in range(step, self.map_width - step, step):
                if grid[gy, gx] != 0:
                    continue
                neighbours = grid[max(0,gy-1):gy+2, max(0,gx-1):gx+2]
                if -1 not in neighbours:
                    continue
                dist = sqrt((gx - rx)**2 + (gy - ry)**2)
                if dist > 8:
                    frontiers.append((dist, gx, gy))

        if not frontiers:
            return

        frontiers.sort(key=lambda f: f[0], reverse=True)
        top_far = frontiers[:max(1, len(frontiers) // 3)]
        _, bx, by = random.choice(top_far)

        world_x = bx * self.map_res + self.map_origin_x
        world_y = by * self.map_res + self.map_origin_y
        self.frontier_angle = atan2(world_y - self.y, world_x - self.x)

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        _, _, yaw = quaternion_to_euler(pose.orientation)
        self.x, self.y, self.theta_z = pose.position.x, pose.position.y, yaw

        if not self.have_odom:
            self.have_odom = True
            self.x0, self.y0 = self.x, self.y
            self.prev_x, self.prev_y = self.x, self.y
            self.start_zone = self._current_zone()
            self.visited_zones.add(self.start_zone)

    def scan_callback(self, msg: LaserScan):
        self.have_scan = True
        r = msg.ranges
        self.front_distance = float(np.min(self.filter_ranges(r, FRONT_ARC)))
        self.left_distance  = float(np.min(self.filter_ranges(r, LEFT_ARC)))
        self.right_distance = float(np.min(self.filter_ranges(r, RIGHT_ARC)))
        self.back_distance  = float(np.min(self.filter_ranges(r, BACK_ARC)))

    def map_callback(self, msg: OccupancyGrid):
        self.have_map     = True
        self.map_data     = msg.data
        self.map_width    = msg.info.width
        self.map_height   = msg.info.height
        self.map_res      = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

    def update_progress_check(self):
        self.progress_timer_count += 1
        if self.progress_timer_count >= 10:
            dx = self.x - self.prev_x
            dy = self.y - self.prev_y
            if sqrt(dx**2 + dy**2) < 0.03:
                self.no_progress_count += 1
            else:
                self.no_progress_count = 0
            self.prev_x, self.prev_y = self.x, self.y
            self.progress_timer_count = 0

    def timer_callback(self):
        if not self.have_scan or not self.have_odom:
            return

        if self.start_time is None:
            self.start_time = time.time()

        elapsed = time.time() - self.start_time
        if elapsed >= TOTAL_TIME:
            self.set_cmd(0.0, 0.0)
            return

        self._update_zones()
        if self.have_map:
            self._update_frontier_bias()

        if self.state not in ("RECOVER", "BACKUP"):
            self.update_progress_check()

        front = self.front_distance
        left  = self.left_distance
        right = self.right_distance
        back  = self.back_distance

        self.last_turn_left = left >= right

        # Recovery trigger
        if self.state not in ("RECOVER", "BACKUP") and self.no_progress_count >= 3:
            if back > BACK_CLEAR_THRESHOLD:
                self.state = "BACKUP"
                self.backup_counter = 10
            else:
                self.state = "RECOVER"
                self.total_recoveries += 1
                self.recover_counter = 20
                if self.total_recoveries % 3 == 0:
                    self.recover_direction = random.choice([1, -1])
                else:
                    self.recover_direction = 1 if left > right else -1
            self.no_progress_count = 0

        # STATE: BACKUP
        if self.state == "BACKUP":
            self.set_cmd(BACKUP_SPEED, 0.0)
            self.backup_counter -= 1
            if self.backup_counter <= 0:
                self.state = "RECOVER"
                self.total_recoveries += 1
                self.recover_counter = 20
                self.recover_direction = 1 if left > right else -1
                self.no_progress_count = 0
                self.progress_timer_count = 0
                self.prev_x, self.prev_y = self.x, self.y
            return

        # STATE: RECOVER
        if self.state == "RECOVER":
            self.set_cmd(0.0, RECOVER_TURN_SPEED * self.recover_direction)
            self.recover_counter -= 1
            if self.recover_counter <= 0:
                self.state = "EXPLORE"
                self.blocked_counter = 0
                self.no_progress_count = 0
                self.progress_timer_count = 0
                self.prev_x, self.prev_y = self.x, self.y
            return

        # STATE: AVOID
        if front < FRONT_CLEAR_THRESHOLD:
            self.state = "AVOID"
            self.blocked_counter += 1

            if self.blocked_counter > 15:
                self.last_turn_left = not self.last_turn_left
                self.blocked_counter = 0

            if front < FRONT_VERY_CLOSE_THRESHOLD:
                self.set_cmd(
                    0.0,
                    TURN_SPEED if self.last_turn_left else -TURN_SPEED)
                return

            self.set_cmd(
                SLOW_SPEED,
                TURN_SPEED if self.last_turn_left else -TURN_SPEED)
            return

        # STATE: EXPLORE
        self.state = "EXPLORE"
        self.blocked_counter = 0

        angular_z = 0.0

        if left < SIDE_CLOSE_THRESHOLD:
            angular_z = -0.45
        elif right < SIDE_CLOSE_THRESHOLD:
            angular_z = 0.45
        elif self.frontier_angle is not None:
            angle_error = self.frontier_angle - self.theta_z
            while angle_error > pi:
                angle_error -= 2 * pi
            while angle_error < -pi:
                angle_error += 2 * pi

            if front > FRONT_CLEAR_THRESHOLD:
                if abs(angle_error) > 0.5:
                    # Large angle error: slow down and turn more aggressively
                    angular_z = max(min(1.0 * angle_error, 0.8), -0.8)
                    self.set_cmd(SLOW_SPEED, angular_z)
                    return
                else:
                    # Small angle error: move forward with gentle correction
                    angular_z = max(min(0.6 * angle_error, 0.4), -0.4)
            else:
                error = left - right
                angular_z = max(min(0.35 * error, 0.25), -0.25)
        else:
            error = left - right
            angular_z = max(min(0.35 * error, 0.25), -0.25)

        self.set_cmd(FORWARD_SPEED, angular_z)

    def log_callback(self):
        if not self.have_scan or not self.have_odom:
            return

        elapsed = time.time() - self.start_time if self.start_time else 0.0
        zone    = self._current_zone()
        fa = f"{degrees(self.frontier_angle):.0f}deg" \
            if self.frontier_angle is not None else "None"

        self.get_logger().info(
            f"t={elapsed:.0f}s | {self.state} | zone={zone} "
            f"visited={len(self.visited_zones)} | frontier={fa} | "
            f"F={self.front_distance:.2f} L={self.left_distance:.2f} "
            f"R={self.right_distance:.2f} | rec={self.total_recoveries}"
        )

    def on_shutdown(self):
        self.get_logger().info("Explorer shutting down - stopping robot.")
        self.vel_pub.publish(TwistStamped())
        self.shutdown = True


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"{node.get_name()} received Ctrl+C.")
    finally:
        node.on_shutdown()
        while not node.shutdown:
            continue
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()